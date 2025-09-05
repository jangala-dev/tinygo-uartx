//go:build rp2040 || rp2350

// Package uartx provides a UART for RP2040/RP2350 mirroring machine.UART’s
// public behaviour (non-blocking Read/ReadByte; blocking Write/WriteByte) and
// adds blocking helpers. No embedding of machine.UART; IRQ is set in Configure.
package uartx

import (
	"context"
	"device/rp"
	"errors"
	"runtime/interrupt"
	"time"
	"unsafe"

	"machine"
)

var (
	ErrBufferEmpty = errors.New("UART buffer empty")
)

type UART struct {
	Buffer    *machine.RingBuffer
	Bus       *rp.UART0_Type
	Interrupt interrupt.Interrupt

	notify chan struct{} // wake-up hint for blocking reads
	closed chan struct{} // close signal
}

type UARTConfig = machine.UARTConfig
type UARTParity = machine.UARTParity

const (
	ParityNone = machine.ParityNone
	ParityEven = machine.ParityEven
	ParityOdd  = machine.ParityOdd
)

// Public instances (separate from machine.UARTx).
var (
	UART0 = &_UART0
	UART1 = &_UART1

	_UART0 = UART{
		Buffer: machine.NewRingBuffer(),
		Bus:    rp.UART0,
		notify: make(chan struct{}, 1),
		closed: make(chan struct{}),
	}
	_UART1 = UART{
		Buffer: machine.NewRingBuffer(),
		Bus:    rp.UART1,
		notify: make(chan struct{}, 1),
		closed: make(chan struct{}),
	}
)

// ---------------- machine.UART-compatible behaviour ----------------

func (u *UART) Configure(cfg UARTConfig) error {
	u.resetAndUnreset()

	// Default baud/pins to match machine semantics.
	if cfg.BaudRate == 0 {
		cfg.BaudRate = 115200
	}
	if cfg.TX == 0 && cfg.RX == 0 {
		cfg.TX = machine.UART_TX_PIN
		cfg.RX = machine.UART_RX_PIN
	}

	u.SetBaudRate(cfg.BaudRate)
	_ = u.SetFormat(8, 1, ParityNone)

	// Enable UART, RX, TX (and optional flow control).
	cr := uint32(rp.UART0_UARTCR_UARTEN | rp.UART0_UARTCR_RXE | rp.UART0_UARTCR_TXE)
	if cfg.RTS != 0 {
		cr |= rp.UART0_UARTCR_RTSEN
	}
	if cfg.CTS != 0 {
		cr |= rp.UART0_UARTCR_CTSEN
	}
	u.Bus.UARTCR.SetBits(cr)

	// Pin muxing through machine.
	if cfg.TX != machine.NoPin {
		cfg.TX.Configure(machine.PinConfig{Mode: machine.PinUART})
	}
	if cfg.RX != machine.NoPin {
		cfg.RX.Configure(machine.PinConfig{Mode: machine.PinUART})
	}
	if cfg.RTS != 0 {
		cfg.RTS.Configure(machine.PinConfig{Mode: machine.PinOutput})
	}
	if cfg.CTS != 0 {
		cfg.CTS.Configure(machine.PinConfig{Mode: machine.PinInput})
	}

	// Lazily install IRQ handler here (no package-level init()).
	if u.Interrupt == (interrupt.Interrupt{}) {
		irqNum := map[*rp.UART0_Type]int{
			rp.UART0: rp.IRQ_UART0_IRQ,
			rp.UART1: rp.IRQ_UART1_IRQ,
		}[u.Bus]
		u.Interrupt = interrupt.New(irqNum, u.handleInterrupt)
		u.Interrupt.SetPriority(0x80)
		u.Interrupt.Enable()
	}
	u.Bus.UARTIMSC.Set(rp.UART0_UARTIMSC_RXIM) // unmask RX IRQ

	return nil
}

func (u *UART) SetBaudRate(br uint32) {
	div := 8 * machine.CPUFrequency() / br
	ibrd := div >> 7
	var fbrd uint32
	switch {
	case ibrd == 0:
		ibrd, fbrd = 1, 0
	case ibrd >= 65535:
		ibrd, fbrd = 65535, 0
	default:
		fbrd = ((div & 0x7f) + 1) / 2
	}
	u.Bus.UARTIBRD.Set(ibrd)
	u.Bus.UARTFBRD.Set(fbrd)
	u.Bus.UARTLCR_H.SetBits(0) // dummy write per PL011 quirk
}

func (u *UART) SetFormat(databits, stopbits uint8, parity UARTParity) error {
	var pen, pev uint8
	if parity != ParityNone {
		pen = rp.UART0_UARTLCR_H_PEN
	}
	if parity == ParityEven {
		pev = rp.UART0_UARTLCR_H_EPS
	}
	u.Bus.UARTLCR_H.SetBits(uint32(
		(databits-5)<<rp.UART0_UARTLCR_H_WLEN_Pos |
			(stopbits-1)<<rp.UART0_UARTLCR_H_STP2_Pos |
			pen | pev,
	))
	return nil
}

// Non-blocking, identical semantics to machine.UART.Read.
func (u *UART) Read(p []byte) (int, error) {
	size := u.Buffered()
	if size == 0 {
		return 0, nil
	}
	if len(p) < size {
		size = len(p)
	}
	for i := 0; i < size; i++ {
		b, _ := u.ReadByte()
		p[i] = b
	}
	return size, nil
}

// Non-blocking, identical semantics to machine.UART.ReadByte.
func (u *UART) ReadByte() (byte, error) {
	if b, ok := u.Buffer.Get(); ok {
		return b, nil
	}
	return 0, ErrBufferEmpty
}

func (u *UART) Buffered() int { return int(u.Buffer.Used()) }

func (u *UART) Receive(b byte) {
	u.Buffer.Put(b)
}

// WriteByte/Write: block until sent, matching machine.
func (u *UART) WriteByte(c byte) error {
	if err := u.writeByte(c); err != nil {
		return err
	}
	u.flush()
	return nil
}

func (u *UART) Write(p []byte) (int, error) {
	for i, b := range p {
		if err := u.writeByte(b); err != nil {
			return i, err
		}
	}
	u.flush()
	return len(p), nil
}

// ---------------- Extra blocking helpers (optional to use) ----------------

func (u *UART) WaitReadable(ctx context.Context) error {
	if u.Buffered() > 0 {
		return nil
	}
	for {
		select {
		case <-u.notify:
			if u.Buffered() > 0 {
				return nil
			}
		case <-u.closed:
			return context.Canceled
		case <-ctx.Done():
			return ctx.Err()
		}
	}
}

func (u *UART) ReadBlocking(ctx context.Context, p []byte) (int, error) {
	for {
		if n, _ := u.Read(p); n > 0 {
			return n, nil
		}
		if err := u.WaitReadable(ctx); err != nil {
			return 0, err
		}
	}
}

func (u *UART) ReadFullBlocking(ctx context.Context, p []byte) (int, error) {
	read := 0
	for read < len(p) {
		if n, _ := u.Read(p[read:]); n > 0 {
			read += n
			continue
		}
		if err := u.WaitReadable(ctx); err != nil {
			return read, err
		}
	}
	return read, nil
}

func (u *UART) ReadByteBlocking(ctx context.Context) (byte, error) {
	for {
		if b, err := u.ReadByte(); err == nil {
			return b, nil
		}
		if err := u.WaitReadable(ctx); err != nil {
			return 0, err
		}
	}
}

func (u *UART) ReadWithTimeout(p []byte, d time.Duration) (int, error) {
	ctx, cancel := context.WithTimeout(context.Background(), d)
	defer cancel()
	return u.ReadBlocking(ctx, p)
}

// Close: mask RX IRQ and unblock waiters.
func (u *UART) Close() error {
	select {
	case <-u.closed:
	default:
		close(u.closed)
	}
	u.Bus.UARTIMSC.ClearBits(rp.UART0_UARTIMSC_RXIM)
	return nil
}

// ------------------------------- Internals --------------------------------

func (u *UART) resetAndUnreset() {
	var mask uint32
	switch u.Bus {
	case rp.UART0:
		mask = rp.RESETS_RESET_UART0
	case rp.UART1:
		mask = rp.RESETS_RESET_UART1
	}
	rp.RESETS.RESET.SetBits(mask)
	rp.RESETS.RESET.ClearBits(mask)
	for !rp.RESETS.RESET_DONE.HasBits(mask) {
	}
}

func (u *UART) writeByte(c byte) error {
	// wait until TX FIFO has space
	for u.Bus.UARTFR.HasBits(rp.UART0_UARTFR_TXFF) {
		time.Sleep(0) // polite yield
	}
	u.Bus.UARTDR.Set(uint32(c))
	return nil
}

func (u *UART) flush() {
	for u.Bus.UARTFR.HasBits(rp.UART0_UARTFR_BUSY) {
		time.Sleep(0)
	}
}

func (u *UART) handleInterrupt(interrupt.Interrupt) {
	// Drain RX FIFO.
	for !u.Bus.UARTFR.HasBits(rp.UART0_UARTFR_RXFE) {
		u.Receive(byte(u.Bus.UARTDR.Get() & 0xFF))
	}
	// edge-triggered notify
	select {
	case u.notify <- struct{}{}:
	default:
	}
}

// Keep for potential low-level work; otherwise harmless.
var _ = unsafe.Pointer(nil)

// uartx/rp2_uart.go
//go:build rp2040 || rp2350

package uartx

import (
	"device/arm"
	"device/rp"
	"machine"
	"runtime/interrupt"
	"sync/atomic"
)

// UART on the RP2040.
type UART struct {
	Buffer    *machine.RingBuffer
	Bus       *rp.UART0_Type
	Interrupt interrupt.Interrupt
	notify    chan struct{} // wake-up hint for blocking reads

	// SEV/WFE notifier state
	wakeFlag        uint32 // set to 1 by ISR to indicate RX activity
	notifierStarted uint32 // guards starting the notifier goroutine
}

// Configure the UART.
func (uart *UART) Configure(config machine.UARTConfig) error {
	initUART(uart)

	// Default baud rate to 115200.
	if config.BaudRate == 0 {
		config.BaudRate = 115200
	}

	// Use default pins if pins are not set.
	if config.TX == 0 && config.RX == 0 {
		// use default pins
		config.TX = machine.UART_TX_PIN
		config.RX = machine.UART_RX_PIN
	}

	uart.SetBaudRate(config.BaudRate)

	// default to 8-1-N
	uart.SetFormat(8, 1, ParityNone)

	// Enable the UART, both TX and RX
	settings := uint32(rp.UART0_UARTCR_UARTEN |
		rp.UART0_UARTCR_RXE |
		rp.UART0_UARTCR_TXE)
	const bits = rp.UART0_UARTCR_UARTEN | rp.UART0_UARTCR_TXE
	if config.RTS != 0 {
		settings |= rp.UART0_UARTCR_RTSEN
	}
	if config.CTS != 0 {
		settings |= rp.UART0_UARTCR_CTSEN
	}

	uart.Bus.UARTCR.SetBits(settings)

	// set GPIO mux to UART for the pins
	if config.TX != machine.NoPin {
		config.TX.Configure(machine.PinConfig{Mode: machine.PinUART})
	}
	if config.RX != machine.NoPin {
		config.RX.Configure(machine.PinConfig{Mode: machine.PinUART})
	}
	if config.RTS != 0 {
		config.RTS.Configure(machine.PinConfig{Mode: machine.PinOutput})
	}
	if config.CTS != 0 {
		config.CTS.Configure(machine.PinConfig{Mode: machine.PinInput})
	}

	// Enable RX IRQ.
	uart.Interrupt.SetPriority(0x80)
	uart.Interrupt.Enable()

	// Setup interrupt on receive.
	uart.Bus.UARTIMSC.Set(rp.UART0_UARTIMSC_RXIM)

	// Start the background notifier once.
	if atomic.CompareAndSwapUint32(&uart.notifierStarted, 0, 1) {
		go uart.runNotifier()
	}

	return nil
}

// SetBaudRate sets the baudrate to be used for the UART.
func (uart *UART) SetBaudRate(br uint32) {
	div := 8 * machine.CPUFrequency() / br

	ibrd := div >> 7
	var fbrd uint32

	switch {
	case ibrd == 0:
		ibrd = 1
		fbrd = 0
	case ibrd >= 65535:
		ibrd = 65535
		fbrd = 0
	default:
		fbrd = ((div & 0x7f) + 1) / 2
	}

	// set PL011 baud divisor registers
	uart.Bus.UARTIBRD.Set(ibrd)
	uart.Bus.UARTFBRD.Set(fbrd)

	// PL011 needs a (dummy) line control register write.
	// See https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_uart/uart.c#L93-L95
	uart.Bus.UARTLCR_H.SetBits(0)
}

// WriteByte writes a byte of data to the UART.
func (uart *UART) writeByte(c byte) error {
	// wait until buffer is not full
	for uart.Bus.UARTFR.HasBits(rp.UART0_UARTFR_TXFF) {
		gosched()
	}

	// write data
	uart.Bus.UARTDR.Set(uint32(c))
	return nil
}

func (uart *UART) flush() {
	for uart.Bus.UARTFR.HasBits(rp.UART0_UARTFR_BUSY) {
		gosched()
	}
}

// SetFormat for number of data bits, stop bits, and parity for the UART.
func (uart *UART) SetFormat(databits, stopbits uint8, parity UARTParity) error {
	var pen, pev uint8
	if parity != ParityNone {
		pen = rp.UART0_UARTLCR_H_PEN
	}
	if parity == ParityEven {
		pev = rp.UART0_UARTLCR_H_EPS
	}
	uart.Bus.UARTLCR_H.SetBits(uint32((databits-5)<<rp.UART0_UARTLCR_H_WLEN_Pos |
		(stopbits-1)<<rp.UART0_UARTLCR_H_STP2_Pos |
		pen | pev))

	return nil
}

func initUART(uart *UART) {
	var resetVal uint32
	switch {
	case uart.Bus == rp.UART0:
		resetVal = rp.RESETS_RESET_UART0
	case uart.Bus == rp.UART1:
		resetVal = rp.RESETS_RESET_UART1
	}

	// reset UART
	rp.RESETS.RESET.SetBits(resetVal)
	rp.RESETS.RESET.ClearBits(resetVal)
	for !rp.RESETS.RESET_DONE.HasBits(resetVal) {
	}
}

// handleInterrupt should be called from the appropriate interrupt handler for
// this UART instance.
func (uart *UART) handleInterrupt(interrupt.Interrupt) {
	for !uart.Bus.UARTFR.HasBits(rp.UART0_UARTFR_RXFE) {
		uart.Receive(byte((uart.Bus.UARTDR.Get() & 0xFF)))
	}

	// Edge-triggered: set flag and SEV.
	atomic.StoreUint32(&uart.wakeFlag, 1)
	arm.Asm("sev")
}

// runNotifier forwards SEV/WFE wake-ups to the coalesced notify channel.
func (uart *UART) runNotifier() {
	for {
		// Arm for the next event and wait.
		atomic.StoreUint32(&uart.wakeFlag, 0)
		arm.Asm("wfe")

		// Filter spurious wakes; only forward if our own ISR set wakeFlag.
		if atomic.LoadUint32(&uart.wakeFlag) == 0 {
			continue
		}
		select {
		case uart.notify <- struct{}{}:
		default:
		}
	}
}

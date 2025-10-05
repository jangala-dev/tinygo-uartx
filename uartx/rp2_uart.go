// uartx/rp2_uart.go
//go:build rp2040 || rp2350

package uartx

import (
	"device/rp"
	"errors"
	"machine"
	"runtime/interrupt"
)

// UART on the RP2040.
type UART struct {
	// RX
	Buffer *RingBuffer
	Bus    *rp.UART0_Type
	// TX
	TxBuffer *RingBuffer
	txNotify chan struct{} // TX space/drain readiness

	Interrupt interrupt.Interrupt
	notify    chan struct{} // RX readiness
}

// Configure the UART.
func (uart *UART) Configure(cfg machine.UARTConfig) error {
	initUART(uart)

	if cfg.BaudRate == 0 {
		cfg.BaudRate = 115200
	}
	if cfg.TX == machine.NoPin && cfg.RX == machine.NoPin {
		cfg.TX = machine.UART_TX_PIN
		cfg.RX = machine.UART_RX_PIN
	}

	// 1) Disable while configuring.
	uart.Bus.UARTCR.ClearBits(rp.UART0_UARTCR_UARTEN | rp.UART0_UARTCR_RXE | rp.UART0_UARTCR_TXE)

	// 2) Mux pins first.
	if cfg.TX != machine.NoPin {
		cfg.TX.Configure(machine.PinConfig{Mode: machine.PinUART})
	}
	if cfg.RX != machine.NoPin {
		cfg.RX.Configure(machine.PinConfig{Mode: machine.PinUART})
	}
	if cfg.RTS != machine.NoPin {
		cfg.RTS.Configure(machine.PinConfig{Mode: machine.PinUART})
	}
	if cfg.CTS != machine.NoPin {
		cfg.CTS.Configure(machine.PinConfig{Mode: machine.PinUART})
	}

	// 3) Divisors and LCR_H.
	uart.SetBaudRate(cfg.BaudRate)       // include the ‘dummy’ LCR_H write
	_ = uart.SetFormat(8, 1, ParityNone) // make SetFormat do a single Set(...), include FEN

	// 4) Clear pending interrupts and purge RX FIFO (read until empty).
	uart.Bus.UARTICR.Set(0x7FF) // clear all PL011 IRQs
	for !uart.Bus.UARTFR.HasBits(rp.UART0_UARTFR_RXFE) {
		_ = uart.Bus.UARTDR.Get() // discard any stale byte
	}
	// Clear sticky RX errors (ECR share-address via RSR)
	uart.Bus.UARTRSR.Set(0)

	// 5) Enable UART and flow control (only if both pins are valid).
	settings := uint32(rp.UART0_UARTCR_UARTEN | rp.UART0_UARTCR_RXE | rp.UART0_UARTCR_TXE)
	if cfg.RTS != machine.NoPin && cfg.CTS != machine.NoPin {
		settings |= rp.UART0_UARTCR_RTSEN | rp.UART0_UARTCR_CTSEN
	}
	uart.Bus.UARTCR.Set(settings)

	// 6) IRQs.
	uart.Interrupt.SetPriority(0x80)
	uart.Interrupt.Enable()
	uart.Bus.UARTIFLS.Set(0)
	uart.Bus.UARTIMSC.Set(rp.UART0_UARTIMSC_RXIM | rp.UART0_UARTIMSC_RTIM) // TXIM on demand

	// 7) Initial writable notification (FIFO starts empty).
	select {
	case uart.txNotify <- struct{}{}:
	default:
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
	uart.Bus.UARTLCR_H.Set(uart.Bus.UARTLCR_H.Get())
}

// SetFormat for number of data bits, stop bits, and parity for the UART.
func (uart *UART) SetFormat(databits, stopbits uint8, parity UARTParity) error {
	if databits < 5 || databits > 8 {
		return errors.New("invalid databits")
	}
	if stopbits != 1 && stopbits != 2 {
		return errors.New("invalid stopbits")
	}

	var pen, pev uint32
	if parity != ParityNone {
		pen = rp.UART0_UARTLCR_H_PEN
		if parity == ParityEven {
			pev = rp.UART0_UARTLCR_H_EPS
		}
	}
	const fen = rp.UART0_UARTLCR_H_FEN

	val := uint32((databits-5)<<rp.UART0_UARTLCR_H_WLEN_Pos|
		(stopbits-1)<<rp.UART0_UARTLCR_H_STP2_Pos) |
		pen | pev | fen

	uart.Bus.UARTLCR_H.Set(val) // full write, not OR
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

// --- TX helpers ---

func (uart *UART) enableTxIRQ() {
	uart.Bus.UARTIMSC.SetBits(rp.UART0_UARTIMSC_TXIM)
}

// txFifoEmpty reports PL011 TX FIFO empty.
func (uart *UART) txFifoEmpty() bool {
	return uart.Bus.UARTFR.HasBits(rp.UART0_UARTFR_TXFE)
}

// tryWriteHW pushes as many bytes as possible directly into the HW FIFO (non-blocking).
func (uart *UART) tryWriteHW(p []byte) int {
	i := 0
	for i < len(p) && !uart.Bus.UARTFR.HasBits(rp.UART0_UARTFR_TXFF) {
		uart.Bus.UARTDR.Set(uint32(p[i]))
		i++
	}
	return i
}

// enqueueTX inserts bytes into the software TX buffer (non-blocking).
func (uart *UART) enqueueTX(p []byte) int {
	i := 0
	for i < len(p) {
		if ok := uart.TxBuffer.Put(p[i]); !ok {
			break
		}
		i++
	}
	return i
}

// SendSome tries to enqueue up to len(p) bytes, non-blocking.
// Returns the number of bytes accepted into the HW FIFO and/or software TX buffer.
func (uart *UART) SendSome(p []byte) int {
	if len(p) == 0 {
		return 0
	}
	// First, opportunistically push into HW FIFO.
	n := uart.tryWriteHW(p)
	if n > 0 {
		uart.enableTxIRQ() // ensure we get TX IRQs as FIFO drains
		return n
	}
	// Next, enqueue into software buffer.
	m := uart.enqueueTX(p)
	if m > 0 {
		uart.enableTxIRQ()
		return m
	}
	return 0
}

// --- RX/TX ISR ---

func (uart *UART) handleInterrupt(interrupt.Interrupt) {
	mis := uart.Bus.UARTMIS.Get()

	// RX path
	if (mis & (rp.UART0_UARTMIS_RXMIS | rp.UART0_UARTMIS_RTMIS)) != 0 {
		for !uart.Bus.UARTFR.HasBits(rp.UART0_UARTFR_RXFE) {
			r := uart.Bus.UARTDR.Get()
			if (r & (rp.UART0_UARTDR_OE | rp.UART0_UARTDR_BE |
				rp.UART0_UARTDR_PE | rp.UART0_UARTDR_FE)) != 0 {
				// Discard errored byte; reading cleared the flags.
				continue
			}
			uart.Receive(byte(r & 0xFF))
		}
		// Clear RX level and RX timeout interrupts.
		uart.Bus.UARTICR.Set(rp.UART0_UARTICR_RXIC | rp.UART0_UARTICR_RTIC)
		// Clear any accumulated RX error flags.
		uart.Bus.UARTRSR.Set(0)
		select {
		case uart.notify <- struct{}{}:
		default:
		}
	}

	// TX path
	if mis&rp.UART0_UARTMIS_TXMIS != 0 {
		// Move bytes from SW buffer to HW FIFO.
		for !uart.Bus.UARTFR.HasBits(rp.UART0_UARTFR_TXFF) {
			b, ok := uart.TxBuffer.Get()
			if !ok {
				break
			}
			uart.Bus.UARTDR.Set(uint32(b))
		}

		// Notify writers that space likely became available.
		select {
		case uart.txNotify <- struct{}{}:
		default:
		}

		// If SW buffer empty, consider TX done for FIFO semantics.
		if uart.TxBuffer.Used() == 0 {
			// If FIFO now empty, coalesce a final "drained" notification.
			if uart.txFifoEmpty() {
				select {
				case uart.txNotify <- struct{}{}:
				default:
				}
			}
			// Disable TX IRQ until more data is queued.
			uart.Bus.UARTIMSC.ClearBits(rp.UART0_UARTIMSC_TXIM)
		}

		// Clear TX interrupt.
		uart.Bus.UARTICR.Set(rp.UART0_UARTICR_TXIC)
	}
}

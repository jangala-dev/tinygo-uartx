// uartx/uartx.go

//go:build atmega || esp || nrf || sam || sifive || stm32 || k210 || nxp || rp2040 || rp2350

package uartx

import (
	"errors"
)

var errUARTBufferEmpty = errors.New("UART buffer empty")

// UARTParity is the parity setting to be used for UART communication.
type UARTParity uint8

const (
	// ParityNone means to not use any parity checking. This is
	// the most common setting.
	ParityNone UARTParity = iota

	// ParityEven means to expect that the total number of 1 bits sent
	// should be an even number.
	ParityEven

	// ParityOdd means to expect that the total number of 1 bits sent
	// should be an odd number.
	ParityOdd
)

// Read from the RX buffer (non-blocking).
func (uart *UART) Read(data []byte) (n int, err error) {
	// check if RX buffer is empty
	size := uart.Buffered()
	if size == 0 {
		return 0, nil
	}

	// Make sure we do not read more from buffer than the data slice can hold.
	if len(data) < size {
		size = len(data)
	}

	// only read number of bytes used from buffer
	for i := 0; i < size; i++ {
		v, _ := uart.ReadByte()
		data[i] = v
	}

	return size, nil
}

// WriteByte writes a single byte and blocks until FIFO has drained (event-driven).
func (uart *UART) WriteByte(c byte) error {
	_, err := uart.Write([]byte{c})
	return err
}

// Write writes all bytes and blocks until the software buffer is empty and the TX FIFO is empty.
// Implemented using IRQ-driven SendSome and readiness notifications; no polling.
func (uart *UART) Write(p []byte) (int, error) {
	sent := 0
	for sent < len(p) {
		n := uart.SendSome(p[sent:])
		if n > 0 {
			sent += n
			continue
		}
		// Wait for TX progress (space or drain) then retry.
		<-uart.txNotify
	}

	// Drain to TX FIFO empty.
	for {
		// Fast path: all enqueued and FIFO empty => done.
		if uart.TxBuffer.Used() == 0 && uart.txFifoEmpty() {
			return sent, nil
		}
		// Wait for ISR to signal progress (space made / FIFO transitioned).
		<-uart.txNotify
	}
}

// ReadByte reads a single byte from the RX buffer.
// If there is no data in the buffer, returns an error.
func (uart *UART) ReadByte() (byte, error) {
	// check if RX buffer is empty
	buf, ok := uart.Buffer.Get()
	if !ok {
		return 0, errUARTBufferEmpty
	}
	return buf, nil
}

// Buffered returns the number of bytes currently stored in the RX buffer.
func (uart *UART) Buffered() int {
	return int(uart.Buffer.Used())
}

// Receive handles adding data to the UART's data buffer.
// Usually called by the IRQ handler for a machine.
func (uart *UART) Receive(data byte) {
	uart.Buffer.Put(data)
}

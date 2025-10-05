// uartx/uartx_extended.go

//go:build rp2040 || rp2350

package uartx

import (
	"context"
	"device/rp"
)

// Readable exposes a coalesced readiness signal suitable for select.
func (uart *UART) Readable() <-chan struct{} { return uart.notify }

// Writable exposes a coalesced TX readiness/drain signal suitable for select.
func (uart *UART) Writable() <-chan struct{} { return uart.txNotify }

// WaitReadableContext blocks until data is available or ctx is done.
func (uart *UART) WaitReadableContext(ctx context.Context) error {
	for {
		if uart.Buffered() > 0 {
			return nil
		}
		select {
		case <-uart.notify:
		case <-ctx.Done():
			return ctx.Err()
		}
	}
}

// RecvSomeContext blocks until at least one byte is available, then reads up to len(p).
func (uart *UART) RecvSomeContext(ctx context.Context, p []byte) (int, error) {
	if len(p) == 0 {
		return 0, nil
	}
	if n, _ := uart.Read(p); n > 0 {
		return n, nil
	}
	for {
		select {
		case <-uart.notify:
			if n, _ := uart.Read(p); n > 0 {
				return n, nil
			}
		case <-ctx.Done():
			return 0, ctx.Err()
		}
	}
}

// RecvByteContext blocks for a single byte or until ctx is done.
func (uart *UART) RecvByteContext(ctx context.Context) (byte, error) {
	if b, err := uart.ReadByte(); err == nil {
		return b, nil
	}
	for {
		select {
		case <-uart.notify:
			if b, err := uart.ReadByte(); err == nil {
				return b, nil
			}
		case <-ctx.Done():
			return 0, ctx.Err()
		}
	}
}

// WaitWritableContext blocks until TX can make progress (HW FIFO not full)
// or the context is cancelled. It is event-driven via txNotify.
func (uart *UART) WaitWritableContext(ctx context.Context) error {
	for {
		// If the HW FIFO is not full, callers can make progress (either write
		// directly into FIFO or the ISR can drain SW buffer to create space).
		if !uart.Bus.UARTFR.HasBits(rp.UART0_UARTFR_TXFF) {
			return nil
		}
		select {
		case <-uart.txNotify: // progress likely occurred; re-check
		case <-ctx.Done():
			return ctx.Err()
		}
	}
}

// SendSomeContext enqueues up to len(p) bytes, blocking (event-driven) until at
// least one byte is accepted or the context is cancelled. Returns the number of
// bytes queued (>=1 on success, 0 with ctx error).
func (uart *UART) SendSomeContext(ctx context.Context, p []byte) (int, error) {
	if len(p) == 0 {
		return 0, nil
	}
	// Fast path: try once without blocking.
	if n := uart.SendSome(p); n > 0 {
		return n, nil
	}
	// Wait for TX readiness and retry until we manage to queue >=1 byte.
	for {
		select {
		case <-uart.txNotify:
			if n := uart.SendSome(p); n > 0 {
				return n, nil
			}
			// spurious wake; loop
		case <-ctx.Done():
			return 0, ctx.Err()
		}
	}
}

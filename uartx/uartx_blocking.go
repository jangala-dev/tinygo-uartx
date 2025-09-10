// uartx/uartx_blocking.go

//go:build rp2040 || rp2350

package uartx

import "context"

// Readable exposes a coalesced readiness signal suitable for select.
func (uart *UART) Readable() <-chan struct{} { return uart.notify }

// WaitReadableContext blocks until data is available or ctx is done.
func (uart *UART) WaitReadableContext(ctx context.Context) error {
	for {
		if uart.Buffered() > 0 {
			return nil
		}
		uart.dbgReadWait()
		select {
		case <-uart.notify:
			// re-check; if empty, it was a spurious wake (coalesced notify)
			if uart.Buffered() == 0 {
				uart.dbgSpuriousWake()
			}
		case <-ctx.Done():
			uart.dbgTimeout()
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
		uart.dbgReadWait()
		select {
		case <-uart.notify:
			if n, _ := uart.Read(p); n > 0 {
				return n, nil
			}
			uart.dbgSpuriousWake()
		case <-ctx.Done():
			uart.dbgTimeout()
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
		uart.dbgReadWait()
		select {
		case <-uart.notify:
			if b, err := uart.ReadByte(); err == nil {
				return b, nil
			}
			uart.dbgSpuriousWake()
		case <-ctx.Done():
			uart.dbgTimeout()
			return 0, ctx.Err()
		}
	}
}

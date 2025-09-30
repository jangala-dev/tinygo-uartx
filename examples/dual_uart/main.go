//go:build rp2040 || rp2350

package main

import (
	"machine"
	"sync"
	"time"

	"github.com/jangala-dev/tinygo-uartx/uartx"
)

// Serial writes are guarded to keep lines intact across goroutines.
var serialMu sync.Mutex

func writeTaggedLine(tag string, line []byte) {
	serialMu.Lock()
	_, _ = machine.Serial.Write([]byte(tag))
	_, _ = machine.Serial.Write(line)
	_, _ = machine.Serial.Write([]byte{'\n'})
	serialMu.Unlock()
}

func runUART(uart *uartx.UART, tx, rx machine.Pin, tag string, initialStagger time.Duration) {
	// Configure UART.
	if err := uart.Configure(uartx.UARTConfig{
		BaudRate: 115200,
		TX:       tx,
		RX:       rx,
	}); err != nil {
		println("uart configure error")
		halt()
	}

	// Optional stagger so both demo writers don't align perfectly.
	time.Sleep(initialStagger)

	// 1) Periodic writer on the UART (demo traffic).
	go func() {
		n := 0
		for {
			uartWriteString(uart, "ping ")
			uartWriteInt(uart, n)
			uartWriteString(uart, "\r\n")
			n++
			time.Sleep(500 * time.Millisecond)
		}
	}()

	// 2) Reader: accumulate until newline, then emit one line to USB CDC.
	go func() {
		buf := make([]byte, 64)
		line := make([]byte, 0, 128) // per-UART line buffer

		flushLine := func() {
			if len(line) == 0 {
				return
			}
			// Trim a single trailing '\r' if present (handle CRLF).
			if line[len(line)-1] == '\r' {
				line = line[:len(line)-1]
			}
			// Copy to avoid reuse after write.
			cp := make([]byte, len(line))
			copy(cp, line)
			writeTaggedLine(tag, cp)
			line = line[:0]
		}

		// Drain any pre-existing bytes.
		for {
			if n, _ := uart.Read(buf); n > 0 {
				for _, b := range buf[:n] {
					if b == '\n' {
						flushLine()
					} else {
						line = append(line, b)
					}
				}
				continue
			}
			break
		}

		for range uart.Readable() {
			for {
				n, _ := uart.Read(buf)
				if n == 0 {
					break
				}
				for _, b := range buf[:n] {
					if b == '\n' {
						flushLine()
					} else {
						line = append(line, b)
						// Optional defence: cap the line to avoid unbounded growth
						// in case a newline never arrives.
						if len(line) > 512 {
							flushLine()
						}
					}
				}
			}
		}
	}()
}

func main() {
	// Allow USB CDC to settle.
	time.Sleep(3 * time.Second)

	// Start both UARTs with distinct tags.
	go runUART(uartx.UART0, 1, 0, "[u0] ", 0)
	go runUART(uartx.UART1, 5, 4, "[u1] ", 100*time.Millisecond)

	// LED for liveness.
	led := machine.LED
	led.Configure(machine.PinConfig{Mode: machine.PinOutput})
	go func() {
		for {
			led.High()
			time.Sleep(250 * time.Millisecond)
			led.Low()
			time.Sleep(250 * time.Millisecond)
		}
	}()

	select {}
}

// --- helpers (no fmt) ---

func uartWriteString(u *uartx.UART, s string) {
	_, _ = u.Write([]byte(s))
}

func uartWriteInt(u *uartx.UART, n int) {
	if n == 0 {
		uartWriteString(u, "0")
		return
	}
	var buf [20]byte
	i := len(buf)
	for n > 0 {
		i--
		buf[i] = byte('0' + (n % 10))
		n /= 10
	}
	_, _ = u.Write(buf[i:])
}

func halt() {
	for {
		time.Sleep(time.Hour)
	}
}

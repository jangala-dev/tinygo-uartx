//go:build rp2040 || rp2350

package main

import (
	"time"

	"machine"

	"github.com/jangala-dev/tinygo-uartx/uartx"
)

var (
	uart = uartx.UART1
	tx   = uartx.UART1_TX_PIN // Pico: GP8
	rx   = uartx.UART1_RX_PIN // Pico: GP9
)

func main() {
	// LED for liveness.
	led := machine.LED
	led.Configure(machine.PinConfig{Mode: machine.PinOutput})

	// Configure UART1 at 115200 on default UART1 pins (TX=GP8, RX=GP9).
	if err := uart.Configure(uartx.UARTConfig{
		BaudRate: 115200,
		TX:       tx,
		RX:       rx,
	}); err != nil {
		logln("uart1 configure error")
		halt()
	}

	// Writer: produce loopback traffic for the demos.
	go writer()

	// Reader: rotate through demonstrations using the new API.
	go demos()

	// LED blinker.
	go func() {
		for {
			led.High()
			time.Sleep(250 * time.Millisecond)
			led.Low()
			time.Sleep(250 * time.Millisecond)
		}
	}()

	select {} // keep running
}

// --- demo rotations ---

func demos() {
	for {
		header("DEMO 1: Readable() + TryRead drain")
		demoReadable(6 * time.Second)

		header("DEMO 2: Readable() with 1s timeout + TryRead drain")
		demoWaitReadable(6 * time.Second)

		header("DEMO 3: Read-at-least-one with 1s timeout (Readable + TryRead)")
		demoRecvSome(6 * time.Second)

		header("DEMO 4: Read exactly one byte with 1s timeout (Readable + TryRead)")
		demoRecvByte(6 * time.Second)
	}
}

// DEMO 1: wait on readiness and drain whatever is available using TryRead.
func demoReadable(d time.Duration) {
	buf := make([]byte, 64)
	deadline := time.Now().Add(d)
	for time.Now().Before(deadline) {
		<-uart.Readable() // blocks until ISR signals RX data
		// Coalesced wake; drain everything currently buffered.
		for {
			n := uart.TryRead(buf)
			if n == 0 {
				break
			}
			log("readable: ")
			log(itoa(n))
			log(": ")
			_, _ = machine.Serial.Write(buf[:n])
			logln("")
		}
	}
}

// DEMO 2: use Readable() with an explicit timeout, then drain with TryRead.
func demoWaitReadable(d time.Duration) {
	buf := make([]byte, 64)
	deadline := time.Now().Add(d)
	for time.Now().Before(deadline) {
		select {
		case <-uart.Readable():
			for {
				n := uart.TryRead(buf)
				if n == 0 {
					break
				}
				log("wait+drain: ")
				log(itoa(n))
				log(": ")
				_, _ = machine.Serial.Write(buf[:n])
				logln("")
			}
		case <-time.After(time.Second):
			logln("wait: timeout")
		}
	}
}

// DEMO 3: read “some” bytes with a 1s timeout; returns after ≥1 byte or timeout.
// Built from Readable() + TryRead (no context helpers).
func demoRecvSome(d time.Duration) {
	buf := make([]byte, 64)
	deadline := time.Now().Add(d)
	for time.Now().Before(deadline) {
		// Try immediate read first.
		if n := uart.TryRead(buf); n > 0 {
			log("some: ")
			log(itoa(n))
			log(": ")
			_, _ = machine.Serial.Write(buf[:n])
			logln("")
			continue
		}
		// Otherwise wait up to 1s for readiness.
		select {
		case <-uart.Readable():
			n := uart.TryRead(buf)
			if n == 0 {
				// Spurious wake; try again on next loop.
				continue
			}
			log("some: ")
			log(itoa(n))
			log(": ")
			_, _ = machine.Serial.Write(buf[:n])
			logln("")
		case <-time.After(time.Second):
			logln("some: timeout")
		}
	}
}

// DEMO 4: read exactly one byte with a 1s deadline using Readable()+TryRead.
func demoRecvByte(d time.Duration) {
	deadline := time.Now().Add(d)
	for time.Now().Before(deadline) {
		// Try immediately.
		var one [1]byte
		if n := uart.TryRead(one[:]); n == 1 {
			printByte(one[0])
			continue
		}
		// Otherwise wait up to 1s, then try again.
		select {
		case <-uart.Readable():
			if uart.TryRead(one[:]) == 1 {
				printByte(one[0])
			}
		case <-time.After(time.Second):
			logln("byte: timeout")
		}
	}
}

func printByte(b byte) {
	log("byte: 0x")
	log(hex2(uint8(b)))
	log(" '")
	if b >= 32 && b <= 126 {
		_, _ = machine.Serial.Write([]byte{b})
	} else {
		log("?")
	}
	logln("'")
}

// --- traffic generator ---

func writer() {
	n := 0
	t := time.NewTicker(400 * time.Millisecond)
	defer t.Stop()

	for range t.C {
		// Periodic line payload for “drain” and “some” demos.
		line := []byte("ping " + itoa(n) + "\r\n")
		_, _ = uart.Write(line) // Write blocks until accepted (no implicit flush)

		// Single-byte marker for the byte demo.
		if n%5 == 0 {
			_ = uart.WriteByte('X')
		}
		n++
	}
}

// --- helpers (no fmt) ---

func header(s string) {
	logln("")
	logln("================================")
	logln(s)
	logln("================================")
}

func log(s string) { _, _ = machine.Serial.Write([]byte(s)) }
func logln(s string) {
	_, _ = machine.Serial.Write([]byte(s))
	_, _ = machine.Serial.Write([]byte("\r\n"))
}

// Minimal non-negative int to ASCII.
func itoa(n int) string {
	if n == 0 {
		return "0"
	}
	var buf [20]byte
	i := len(buf)
	for n > 0 {
		i--
		buf[i] = byte('0' + (n % 10))
		n /= 10
	}
	return string(buf[i:])
}

// Two-digit hexadecimal for a byte.
func hex2(b uint8) string {
	const hexdigits = "0123456789ABCDEF"
	var buf [2]byte
	buf[0] = hexdigits[b>>4]
	buf[1] = hexdigits[b&0x0F]
	return string(buf[:])
}

func halt() {
	for {
		time.Sleep(time.Hour)
	}
}

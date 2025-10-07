//go:build rp2040 || rp2350

package main

import (
	"machine"
	"time"

	"github.com/jangala-dev/tinygo-uartx/uartx"
)

var (
	uart = uartx.UART1
)

func main() {
	// LED for liveness.
	led := machine.LED
	led.Configure(machine.PinConfig{Mode: machine.PinOutput})

	// Configure UART0 at 115200 on explicit pins.
	if err := uart.Configure(uartx.UARTConfig{
		BaudRate: 115200,
		TX:       uartx.UART1_TX_PIN,
		RX:       uartx.UART1_RX_PIN,
	}); err != nil {
		println("uart configure error")
		halt()
	}

	// 1) Writer: send "ping N\r\n" periodically over UART1.
	go func() {
		n := 0
		for {
			uartWriteString("ping ")
			uartWriteInt(n)
			uartWriteString("\r\n")
			n++
			time.Sleep(500 * time.Millisecond)
		}
	}()

	// 2) Reader: block on Readable() then drain available bytes.
	go func() {
		buf := make([]byte, 64)

		// Drain any bytes already present before first wait.
		drain(buf)

		for range uart.Readable() {
			// Coalesced wake: drain until empty.
			drain(buf)
		}
	}()

	// 3) LED blinker.
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

func drain(buf []byte) {
	for {
		if n := uart.TryRead(buf); n > 0 {
			_, _ = machine.Serial.Write(buf[:n]) // appears in -monitor
			continue
		}
		break
	}
}

// --- helpers (no fmt) ---

func uartWriteString(s string) {
	_, _ = uart.Write([]byte(s))
}

func uartWriteInt(n int) {
	if n == 0 {
		uartWriteString("0")
		return
	}
	var buf [20]byte
	i := len(buf)
	for n > 0 {
		i--
		buf[i] = byte('0' + (n % 10))
		n /= 10
	}
	_, _ = uart.Write(buf[i:])
}

func halt() {
	for {
		time.Sleep(time.Hour)
	}
}

//go:build rp2040 || rp2350

package main

import (
	"machine"
	"time"

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

	// Configure UART1 at 115200 on explicit pins (loop TX→RX with a jumper).
	if err := uart.Configure(uartx.UARTConfig{
		BaudRate: 115200,
		TX:       tx,
		RX:       rx,
	}); err != nil {
		println("uart1 configure error")
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

	// 2) Reader: poll UART1 and forward anything received to USB-CDC.
	go func() {
		buf := make([]byte, 64)
		for {
			if uart.Buffered() > 0 {
				if n, _ := uart.TryRead(buf); n > 0 {
					_, _ = machine.Serial.Write(buf[:n])
				}
			}
			time.Sleep(1 * time.Millisecond) // avoid a tight spin
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

package main

import (
	"context"
	"time"

	"machine"

	"github.com/jangala-dev/tinygo-uartx/uartx"
)

// Simple loopback test for uartx.UART0.
// Wire TX → RX (for Pico: GP0 to GP1) before flashing.
func main() {
	u := uartx.UART0
	_ = u.Configure(machine.UARTConfig{
		BaudRate: 115200,
		TX:       machine.UART_TX_PIN,
		RX:       machine.UART_RX_PIN,
	})

	// Periodically write "ping".
	go func() {
		for {
			u.Write([]byte("ping\n"))
			time.Sleep(1 * time.Second)
		}
	}()

	buf := make([]byte, 16)
	ctx := context.Background()
	for {
		n, err := u.ReadBlocking(ctx, buf)
		if err == nil && n > 0 {
			u.Write([]byte("echo: "))
			u.Write(buf[:n])
		}
	}
}

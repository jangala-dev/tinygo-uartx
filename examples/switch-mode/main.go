//go:build rp2040 || rp2350

package main

import (
	"machine"
	"sync"
	"time"

	"github.com/jangala-dev/tinygo-uartx/uartx"
)

var (
	uart = uartx.UART1
	tx   = uartx.UART1_TX_PIN // Pico: GP8
	rx   = uartx.UART1_RX_PIN // Pico: GP9

	cfgMu sync.Mutex
)

func main() {
	// Allow time to open the serial monitor
	time.Sleep(3 * time.Second)
	println("uartx reconfig demo starting")

	// LED for liveness.
	led := machine.LED
	led.Configure(machine.PinConfig{Mode: machine.PinOutput})

	// Configure UART0 at 115200 8N1 on explicit pins.
	if err := uart.Configure(uartx.UARTConfig{
		BaudRate: 115200,
		TX:       tx,
		RX:       rx,
	}); err != nil {
		println("uart0 configure error")
		halt()
	}

	// 1) Writer: sends "ping" for 10s, then "pong" after reconfig.
	go func() {
		n := 0
		msg := "ping "
		for {
			uartWriteString(msg)
			uartWriteInt(n)
			uartWriteString("\r\n")
			n++
			time.Sleep(500 * time.Millisecond)

			// After reconfig (signalled by LED blink pattern), switch label.
			// This is just a simple cue; no locking needed.
			if reconfigDone() {
				msg = "pong "
			}
		}
	}()

	// 2) Reader: waits on Readable() and drains available bytes.
	go func() {
		buf := make([]byte, 64)

		// Drain any bytes already present before first wait.
		drain(buf)

		for {
			<-uart.Readable() // coalesced wake
			drain(buf)
		}
	}()

	// 3) LED blinker: slow before reconfig, fast afterwards.
	go func() {
		slow := true
		for {
			led.High()
			if slow {
				time.Sleep(250 * time.Millisecond)
			} else {
				time.Sleep(75 * time.Millisecond)
			}
			led.Low()
			if slow {
				time.Sleep(250 * time.Millisecond)
			} else {
				time.Sleep(75 * time.Millisecond)
			}
			if reconfigDone() {
				slow = false
			}
		}
	}()

	// 4) After 10 seconds, reconfigure UART at runtime.
	time.Sleep(10 * time.Second)
	println("reconfiguring UART: 230400 baud, 7E2")

	// Brief quiesce to reduce in-flight bytes (optional).
	time.Sleep(50 * time.Millisecond)

	cfgMu.Lock()
	uart.SetBaudRate(230400)
	// databits=7, stopbits=2, parity=even
	if err := uart.SetFormat(7, 2, uartx.ParityEven); err != nil {
		println("set format error")
	}
	cfgMu.Unlock()

	println("reconfigured")
	markReconfigDone()

	select {} // keep running
}

func drain(buf []byte) {
	for {
		cfgMu.Lock()
		n, _ := uart.Read(buf)
		cfgMu.Unlock()
		if n > 0 {
			_, _ = machine.Serial.Write(buf[:n]) // appears in -monitor
			continue
		}
		break
	}
}

// --- helpers (no fmt) ---

func uartWriteString(s string) {
	cfgMu.Lock()
	_, _ = uart.Write([]byte(s))
	cfgMu.Unlock()
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
	cfgMu.Lock()
	_, _ = uart.Write(buf[i:])
	cfgMu.Unlock()
}

// simple reconfig flag without atomics (TinyGo-safe)
var reconfigFlag uint8

func markReconfigDone()  { reconfigFlag = 1 }
func reconfigDone() bool { return reconfigFlag == 1 }

func halt() {
	for {
		time.Sleep(time.Hour)
	}
}

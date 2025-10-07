package main

import (
	"context"
	"crypto/sha1"
	"time"

	"machine"

	"github.com/jangala-dev/tinygo-uartx/uartx"
)

var (
	u          = uartx.UART1
	txPin      = uartx.UART1_TX_PIN
	rxPin      = uartx.UART1_RX_PIN
	baud       = 921600
	lineEnding = "\r\n"
)

func drain(u *uartx.UART) {
	var tmp [64]byte
	for {
		n := u.TryRead(tmp[:])
		if n == 0 {
			return
		}
	}
}

// sendAllContext writes p using TryWrite+Writable with a context timeout.
// It returns when all bytes are accepted by the driver or ctx ends.
func sendAllContext(ctx context.Context, u *uartx.UART, p []byte) (int, error) {
	sent := 0
	for sent < len(p) {
		if n := u.TryWrite(p[sent:]); n > 0 {
			sent += n
			continue
		}
		select {
		case <-u.Writable():
		case <-ctx.Done():
			return sent, ctx.Err()
		}
	}
	return sent, nil
}

// recvExact reads exactly n bytes (or ctx error) using TryRead+Readable.
func recvExact(ctx context.Context, u *uartx.UART, n int) ([]byte, error) {
	out := make([]byte, 0, n)
	var buf [128]byte
	for len(out) < n {
		if k := u.TryRead(buf[:]); k > 0 {
			out = append(out, buf[:k]...)
			continue
		}
		select {
		case <-u.Readable():
		case <-ctx.Done():
			return out, ctx.Err()
		}
	}
	return out, nil
}

func ledBlink(times int, on time.Duration) {
	for i := 0; i < times; i++ {
		machine.LED.High()
		time.Sleep(on)
		machine.LED.Low()
		time.Sleep(on)
	}
}

func main() {
	// Give the monitor time to attach.
	time.Sleep(3 * time.Second)

	println("uartx self-test starting")

	if err := u.Configure(uartx.UARTConfig{
		BaudRate: uint32(baud),
		TX:       machine.Pin(txPin),
		RX:       machine.Pin(rxPin),
	}); err != nil {
		println("Configure failed")
		for {
			ledBlink(1, 500*time.Millisecond)
		}
	}

	machine.LED.Configure(machine.PinConfig{Mode: machine.PinOutput})
	drain(u)

	pass, fail := 0, 0
	defer func() {
		println("")
		println("Summary")
		println("  passed =", pass)
		println("  failed =", fail)
		if fail == 0 {
			ledBlink(3, 120*time.Millisecond)
		} else {
			for {
				ledBlink(1, 600*time.Millisecond)
				time.Sleep(800 * time.Millisecond)
			}
		}
	}()

	run := func(name string, f func() string) {
		println("")
		println("[Test]", name)
		if msg := f(); msg == "" {
			println("  PASS")
			pass++
		} else {
			println("  FAIL:", msg)
			fail++
		}
	}

	run("notify: initial Writable after Configure", func() string {
		drain(u)
		select {
		case <-u.Writable():
			ctx, cancel := context.WithTimeout(context.Background(), 750*time.Millisecond)
			defer cancel()
			if n, err := sendAllContext(ctx, u, []byte("X")); err != nil || n != 1 {
				return "could not enqueue"
			}
			got, err := recvExact(ctx, u, 1)
			if err != nil || len(got) != 1 || got[0] != 'X' {
				return "echo failed"
			}
			return ""
		case <-time.After(750 * time.Millisecond):
			return "no initial Writable"
		}
	})

	run("sanity: short loopback (Write + blocking Read)", func() string {
		drain(u)
		msg := []byte("hello, uartx" + lineEnding)
		// Write blocks until accepted (no drain).
		if _, err := u.Write(msg); err != nil {
			return "write failed"
		}
		// Read blocks until ≥1 byte; use recvExact to collect all.
		ctx, cancel := context.WithTimeout(context.Background(), 1*time.Second)
		defer cancel()
		got, err := recvExact(ctx, u, len(msg))
		if err != nil {
			return "timeout"
		}
		if string(got) != string(msg) {
			return "mismatch"
		}
		return ""
	})

	run("blocking: Read waits for a single byte", func() string {
		drain(u)
		want := byte('Z')
		ctx, cancel := context.WithTimeout(context.Background(), 1*time.Second)
		defer cancel()
		go func() { _, _ = sendAllContext(ctx, u, []byte{want}) }()
		var b [1]byte
		// Use blocking Read to fetch one byte.
		if _, err := u.Read(b[:]); err != nil {
			return "read error"
		}
		if b[0] != want {
			return "wrong byte"
		}
		return ""
	})

	run("timeout: no data within 200ms using TryRead+Readable", func() string {
		drain(u)
		ctx, cancel := context.WithTimeout(context.Background(), 200*time.Millisecond)
		defer cancel()
		select {
		case <-u.Readable():
			return "unexpected data"
		case <-ctx.Done():
			return ""
		}
	})

	run("notify: Readable channel", func() string {
		drain(u)
		ctx, cancel := context.WithTimeout(context.Background(), 1*time.Second)
		defer cancel()
		go func() { _, _ = sendAllContext(ctx, u, []byte("AB")) }()
		select {
		case <-u.Readable():
			got, err := recvExact(ctx, u, 2)
			if err != nil || string(got) != "AB" {
				return "wrong data"
			}
			return ""
		case <-time.After(1 * time.Second):
			return "no notification"
		}
	})

	run("framing: two lines", func() string {
		drain(u)
		data := []byte("first line\r\nsecond line\n")
		if _, err := u.Write(data); err != nil {
			return "write failed"
		}
		ctx, cancel := context.WithTimeout(context.Background(), 1*time.Second)
		defer cancel()
		got, err := recvExact(ctx, u, len(data))
		if err != nil {
			return "timeout"
		}
		if string(got) != string(data) {
			return "mismatch"
		}
		return ""
	})

	run("binary: 4 KiB integrity (SHA-1)", func() string {
		drain(u)
		n := 4 * 1024
		src := make([]byte, n)
		var x uint32 = 0x12345678
		for i := range src {
			x = 1664525*x + 1013904223
			src[i] = byte(x >> 24)
		}
		want := sha1.Sum(src)

		ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
		defer cancel()
		go func() { _, _ = sendAllContext(ctx, u, src) }()
		got, err := recvExact(ctx, u, n)
		if err != nil || len(got) != n {
			return "timeout/short read"
		}
		if sha1.Sum(got) != want {
			return "hash mismatch"
		}
		return ""
	})

	run("overflow: 8192-byte burst", func() string {
		drain(u)
		n := 8192
		src := make([]byte, n)
		for i := 0; i < n; i++ {
			src[i] = byte(i)
		}
		ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
		defer cancel()
		go func() { _, _ = sendAllContext(ctx, u, src) }()
		_, _ = recvExact(ctx, u, n)
		return ""
	})

	run("throughput: 32 KiB (event-driven TX)", func() string {
		drain(u)
		n := 32 * 1024
		src := make([]byte, n)
		for i := 0; i < n; i++ {
			src[i] = byte(i * 31)
		}

		start := time.Now()
		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		defer cancel()
		go func() { _, _ = sendAllContext(ctx, u, src) }()
		if _, err := recvExact(ctx, u, n); err != nil {
			return "timeout"
		}

		elapsed := time.Since(start)
		ms := int(elapsed / time.Millisecond)
		if ms <= 0 {
			ms = 1
		}
		kbpsX100 := (n*8*100 + ms/2) / ms
		println("  speed =", formatFixed2(kbpsX100), "kbps")
		return ""
	})

	run("format: SetFormat 8N1", func() string {
		_ = u.SetFormat(8, 1, uartx.ParityNone)
		drain(u)
		msg := []byte("format-ok" + lineEnding)
		if _, err := u.Write(msg); err != nil {
			return "write failed"
		}
		ctx, cancel := context.WithTimeout(context.Background(), 1*time.Second)
		defer cancel()
		got, err := recvExact(ctx, u, len(msg))
		if err != nil {
			return "timeout"
		}
		if string(got) != string(msg) {
			return "mismatch"
		}
		return ""
	})

	println("")
	println("All tests completed")
}

// --- tiny helpers (no fmt) ---

func itoa(n int) string {
	if n == 0 {
		return "0"
	}
	neg := false
	if n < 0 {
		neg = true
		n = -n
	}
	var buf [20]byte
	i := len(buf)
	for n > 0 {
		i--
		buf[i] = byte('0' + (n % 10))
		n /= 10
	}
	if neg {
		i--
		buf[i] = '-'
	}
	return string(buf[i:])
}

func twoDigits(n int) string {
	if n < 10 {
		return "0" + itoa(n)
	}
	return itoa(n)
}

func formatFixed2(x int) string {
	sign := ""
	if x < 0 {
		sign = "-"
		x = -x
	}
	whole := x / 100
	frac := x % 100
	return sign + itoa(whole) + "." + twoDigits(frac)
}

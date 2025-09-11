// cmd/uartx_selftest/main.go

package main

import (
	"context"
	"crypto/sha1"
	"time"

	"machine"

	"github.com/jangala-dev/tinygo-uartx/uartx"
)

const (
	baud       = 115200
	lineEnding = "\r\n"
)

func must[T any](v T, err error) T {
	if err != nil {
		panic(err)
	}
	return v
}

func drain(u *uartx.UART) {
	for u.Buffered() > 0 {
		_, _ = u.ReadByte()
	}
}

func recvExact(ctx context.Context, u *uartx.UART, n int) ([]byte, error) {
	out := make([]byte, 0, n)
	tmp := make([]byte, 64)
	for len(out) < n {
		select {
		case <-ctx.Done():
			return out, ctx.Err()
		default:
			if k, _ := u.RecvSomeContext(ctx, tmp); k > 0 {
				out = append(out, tmp[:k]...)
			}
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
	// Allow time to open the serial monitor
	time.Sleep(5 * time.Second)

	println("uartx self-test starting")

	u := uartx.UART1
	err := u.Configure(uartx.UARTConfig{
		BaudRate: baud,
		TX:       uartx.UART1_TX_PIN,
		RX:       uartx.UART1_RX_PIN,
	})
	if err != nil {
		println("Configure failed")
		for {
			ledBlink(1, 500*time.Millisecond)
		}
	}

	machine.LED.Configure(machine.PinConfig{Mode: machine.PinOutput})
	drain(u)

	pass := 0
	fail := 0
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
		msg := f()
		if msg == "" {
			println("  PASS")
			pass++
		} else {
			println("  FAIL:", msg)
			fail++
		}
	}

	run("sanity: short loopback", func() string {
		drain(u)
		msg := []byte("hello, uartx" + lineEnding)
		_, _ = u.Write(msg)
		ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
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

	run("blocking: RecvByteContext waits", func() string {
		drain(u)
		want := byte('X')
		go func() {
			time.Sleep(150 * time.Millisecond)
			_ = u.WriteByte(want)
		}()
		ctx, cancel := context.WithTimeout(context.Background(), 750*time.Millisecond)
		defer cancel()
		got, err := u.RecvByteContext(ctx)
		if err != nil {
			return "timeout"
		}
		if got != want {
			return "wrong byte"
		}
		return ""
	})

	run("timeout: RecvSomeContext", func() string {
		drain(u)
		ctx, cancel := context.WithTimeout(context.Background(), 200*time.Millisecond)
		defer cancel()
		buf := make([]byte, 32)
		n, err := u.RecvSomeContext(ctx, buf)
		if err == nil && n > 0 {
			return "expected timeout"
		}
		return ""
	})

	run("notify: Readable channel", func() string {
		drain(u)
		ready := u.Readable()
		go func() {
			_ = u.WriteByte('A')
			time.Sleep(5 * time.Millisecond)
			_ = u.WriteByte('B')
		}()
		select {
		case <-ready:
			ctx, cancel := context.WithTimeout(context.Background(), 200*time.Millisecond)
			defer cancel()
			got, _ := recvExact(ctx, u, 2)
			if string(got) != "AB" {
				return "wrong data"
			}
		case <-time.After(300 * time.Millisecond):
			return "no notification"
		}
		return ""
	})

	run("framing: two lines", func() string {
		drain(u)
		data := []byte("first line\r\nsecond line\n")
		_, _ = u.Write(data)
		ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
		defer cancel()
		got, _ := recvExact(ctx, u, len(data))
		if string(got) != string(data) {
			return "mismatch"
		}
		return ""
	})

	run("binary: 1024 bytes integrity", func() string {
		drain(u)
		src := make([]byte, 1024)
		var x uint32 = 0x12345678
		for i := range src {
			x = 1664525*x + 1013904223
			src[i] = byte(x >> 24)
		}
		wantHash := sha1.Sum(src)
		_, _ = u.Write(src)
		ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
		defer cancel()
		got, _ := recvExact(ctx, u, len(src))
		gotHash := sha1.Sum(got)
		for i := range wantHash {
			if wantHash[i] != gotHash[i] {
				return "hash mismatch"
			}
		}
		return ""
	})

	run("overflow: 8192 byte burst", func() string {
		drain(u)
		n := 8192
		src := make([]byte, n)
		for i := 0; i < n; i++ {
			src[i] = byte(i)
		}
		go func() { _, _ = u.Write(src) }()
		time.Sleep(100 * time.Millisecond)
		ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
		defer cancel()
		_, _ = recvExact(ctx, u, n)
		// Informational only
		return ""
	})

	run("throughput: 32 KiB write+read", func() string {
		drain(u)
		n := 32 * 1024
		src := make([]byte, n)
		for i := 0; i < n; i++ {
			src[i] = byte(i * 31)
		}
		go func() { _, _ = u.Write(src) }()
		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		defer cancel()
		_, err := recvExact(ctx, u, n)
		if err != nil {
			return "timeout"
		}
		return ""
	})

	run("format: SetFormat 8N1", func() string {
		_ = u.SetFormat(8, 1, uartx.ParityNone)
		drain(u)
		msg := []byte("format-ok" + lineEnding)
		_, _ = u.Write(msg)
		ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
		defer cancel()
		got, _ := recvExact(ctx, u, len(msg))
		if string(got) != string(msg) {
			return "mismatch"
		}
		return ""
	})

	println("")
	println("All tests completed")
}

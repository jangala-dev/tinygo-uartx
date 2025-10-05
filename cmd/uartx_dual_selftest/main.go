package main

import (
	"context"
	"crypto/sha1"
	"time"

	"machine"

	"github.com/jangala-dev/tinygo-uartx/uartx"
)

const baud = 115200

func main() {
	time.Sleep(3 * time.Second)
	println("uartx cross-UART self-test starting (UART0<->UART1)")

	u0 := uartx.UART0
	u1 := uartx.UART1

	_ = u0.Configure(uartx.UARTConfig{BaudRate: baud, TX: uartx.UART0_TX_PIN, RX: uartx.UART0_RX_PIN})
	_ = u1.Configure(uartx.UARTConfig{BaudRate: baud, TX: uartx.UART1_TX_PIN, RX: uartx.UART1_RX_PIN})

	machine.LED.Configure(machine.PinConfig{Mode: machine.PinOutput})
	drain(u0)
	drain(u1)

	pass, fail := 0, 0
	defer func() {
		println("")
		println("Summary")
		println("  passed =", pass)
		println("  failed =", fail)
		if fail == 0 {
			blink(machine.LED, 3, 120*time.Millisecond)
		} else {
			for {
				blink(machine.LED, 1, 600*time.Millisecond)
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

	// Short messages each way
	run("U0 -> U1 short", func() string {
		drain(u0)
		drain(u1)
		msg := []byte("hello from U0\r\n")
		ctx, cancel := context.WithTimeout(context.Background(), 1*time.Second)
		defer cancel()
		go func() { _, _ = sendAllContext(ctx, u0, msg) }()
		got, err := recvExact(ctx, u1, len(msg))
		if err != nil || string(got) != string(msg) {
			return "mismatch/timeout"
		}
		return ""
	})

	run("U1 -> U0 short", func() string {
		drain(u0)
		drain(u1)
		msg := []byte("hi from U1\r\n")
		ctx, cancel := context.WithTimeout(context.Background(), 1*time.Second)
		defer cancel()
		go func() { _, _ = sendAllContext(ctx, u1, msg) }()
		got, err := recvExact(ctx, u0, len(msg))
		if err != nil || string(got) != string(msg) {
			return "mismatch/timeout"
		}
		return ""
	})

	// Integrity 4 KiB each way (streamed)
	run("U0 -> U1 integrity 4KiB (streamed)", func() string {
		drain(u0)
		drain(u1)
		n := 4 * 1024
		ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
		defer cancel()
		done := make(chan struct{}, 1)
		go func() { _ = sendPatternContext(ctx, u0, patternA, n); done <- struct{}{} }()
		got := sha1.New()
		if err := recvStream(ctx, u1, n, got); err != nil {
			return "timeout/short read"
		}
		<-done
		if !equalHash(got.Sum(nil), sha1Pattern(patternA, n)) {
			return "hash mismatch"
		}
		return ""
	})

	run("U1 -> U0 integrity 4KiB (streamed)", func() string {
		drain(u0)
		drain(u1)
		n := 4 * 1024
		ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
		defer cancel()
		done := make(chan struct{}, 1)
		go func() { _ = sendPatternContext(ctx, u1, patternB, n); done <- struct{}{} }()
		got := sha1.New()
		if err := recvStream(ctx, u0, n, got); err != nil {
			return "timeout/short read"
		}
		<-done
		if !equalHash(got.Sum(nil), sha1Pattern(patternB, n)) {
			return "hash mismatch"
		}
		return ""
	})

	// Full-duplex 8 KiB each way: start receivers first, then senders
	run("Full-duplex: 8KiB each way (streamed)", func() string {
		drain(u0)
		drain(u1)
		n := 8 * 1024
		ctx, cancel := context.WithTimeout(context.Background(), 8*time.Second)
		defer cancel()

		errCh := make(chan string, 2)

		// Receivers first
		go func() {
			h := sha1.New()
			if err := recvStream(ctx, u1, n, h); err != nil {
				errCh <- "U0->U1 timeout"
				return
			}
			if !equalHash(h.Sum(nil), sha1Pattern(patternA, n)) {
				errCh <- "U0->U1 hash mismatch"
				return
			}
			errCh <- ""
		}()
		go func() {
			h := sha1.New()
			if err := recvStream(ctx, u0, n, h); err != nil {
				errCh <- "U1->U0 timeout"
				return
			}
			if !equalHash(h.Sum(nil), sha1Pattern(patternB, n)) {
				errCh <- "U1->U0 hash mismatch"
				return
			}
			errCh <- ""
		}()

		// Then senders
		go func() { _ = sendPatternContext(ctx, u0, patternA, n) }()
		go func() { _ = sendPatternContext(ctx, u1, patternB, n) }()

		e1, e2 := <-errCh, <-errCh
		if e1 != "" || e2 != "" {
			if e1 != "" {
				return e1
			}
			return e2
		}
		return ""
	})

	// Throughput 32 KiB each way: receive both directions concurrently
	run("Throughput: 32KiB each way (streamed)", func() string {
		drain(u0)
		drain(u1)
		n := 32 * 1024
		ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
		defer cancel()

		// Receivers (concurrent)
		rcvDone := make(chan string, 2)
		go func() {
			if err := recvStream(ctx, u1, n, nil); err != nil {
				rcvDone <- "U0->U1 timeout"
				return
			}
			rcvDone <- ""
		}()
		go func() {
			if err := recvStream(ctx, u0, n, nil); err != nil {
				rcvDone <- "U1->U0 timeout"
				return
			}
			rcvDone <- ""
		}()

		// Senders
		start := time.Now()
		go func() { _ = sendPatternContext(ctx, u0, patternA, n) }()
		go func() { _ = sendPatternContext(ctx, u1, patternB, n) }()

		e1, e2 := <-rcvDone, <-rcvDone
		elapsed := time.Since(start)
		if e1 != "" || e2 != "" {
			if e1 != "" {
				return e1
			}
			return e2
		}

		ms := int(elapsed / time.Millisecond)
		if ms <= 0 {
			ms = 1
		}
		kbpsX100 := (n*8*100 + ms/2) / ms
		println("  U0->U1 =", formatFixed2(kbpsX100), "kbps   U1->U0 =", formatFixed2(kbpsX100), "kbps")
		return ""
	})

	println("")
	println("All tests completed")
}

/*** helpers (no fmt) ***/

func drain(u *uartx.UART) {
	for u.Buffered() > 0 {
		_, _ = u.ReadByte()
	}
}

func sendAllContext(ctx context.Context, u *uartx.UART, p []byte) (int, error) {
	sent := 0
	for sent < len(p) {
		if n := u.SendSome(p[sent:]); n > 0 {
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

func sendPatternContext(ctx context.Context, u *uartx.UART, gen func(i int) byte, n int) error {
	const chunk = 128
	var buf [chunk]byte
	i := 0
	for i < n {
		k := chunk
		if n-i < k {
			k = n - i
		}
		for j := 0; j < k; j++ {
			buf[j] = gen(i + j)
		}
		if _, err := sendAllContext(ctx, u, buf[:k]); err != nil {
			return err
		}
		i += k
	}
	return nil
}

func recvExact(ctx context.Context, u *uartx.UART, n int) ([]byte, error) {
	out := make([]byte, 0, n)
	var tmp [128]byte
	for len(out) < n {
		select {
		case <-ctx.Done():
			return out, ctx.Err()
		default:
			if k, _ := u.RecvSomeContext(ctx, tmp[:]); k > 0 {
				out = append(out, tmp[:k]...)
			}
		}
	}
	return out, nil
}

func recvStream(ctx context.Context, u *uartx.UART, n int, sink interface{ Write([]byte) (int, error) }) error {
	var tmp [256]byte
	rem := n
	for rem > 0 {
		select {
		case <-ctx.Done():
			return ctx.Err()
		default:
			k := rem
			if k > len(tmp) {
				k = len(tmp)
			}
			if m, _ := u.RecvSomeContext(ctx, tmp[:k]); m > 0 {
				if sink != nil {
					_, _ = sink.Write(tmp[:m])
				}
				rem -= m
			}
		}
	}
	return nil
}

func patternA(i int) byte { return byte((i*31 + 0x55) & 0xFF) }
func patternB(i int) byte { return byte((i*17 + 0xA6) & 0xFF) }

func sha1Pattern(gen func(i int) byte, n int) []byte {
	h := sha1.New()
	const chunk = 128
	var buf [chunk]byte
	i := 0
	for i < n {
		k := chunk
		if n-i < k {
			k = n - i
		}
		for j := 0; j < k; j++ {
			buf[j] = gen(i + j)
		}
		_, _ = h.Write(buf[:k])
		i += k
	}
	return h.Sum(nil)
}

func equalHash(a, b []byte) bool {
	if len(a) != len(b) {
		return false
	}
	ok := true
	for i := range a {
		ok = ok && (a[i] == b[i])
	}
	return ok
}

func blink(pin machine.Pin, times int, on time.Duration) {
	for i := 0; i < times; i++ {
		pin.High()
		time.Sleep(on)
		pin.Low()
		time.Sleep(on)
	}
}

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

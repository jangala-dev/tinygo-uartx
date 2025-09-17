//go:build (rp2040 || rp2350) && uartxdebug

package main

import (
	"context"
	"crypto/sha1"
	"time"

	"machine"

	"github.com/jangala-dev/tinygo-uartx/uartx"
)

const baud = 115200

func must[T any](v T, err error) T {
	if err != nil {
		println("fatal:", err)
		for {
			time.Sleep(time.Hour)
		}
	}
	return v
}

func printStats(u *uartx.UART, label string) {
	s := u.DebugStats()
	r := u.DebugRegs()
	println("==", label)
	println("ISR:    count=", s.ISRCount, " bytes=", s.ISRBytes, " maxdrain=", s.ISRMaxDrain)
	println("Notify: sent=", s.NotifySent, " dropped=", s.NotifyDropped)
	println("Errors: OE=", s.ErrOverrun, " BE=", s.ErrBreak, " PE=", s.ErrParity, " FE=", s.ErrFraming)
	println("Ring:   puts=", s.RingPuts, " drops=", s.RingDrops, " maxUsed=", s.RingMaxUsed)
	println("Waits:  waits=", s.ReadWaits, " spurious=", s.SpuriousWakes, " timeouts=", s.Timeouts)
	println("Regs:   FR=0x", r.FR, " CR=0x", r.CR, " LCRH=0x", r.LCRH,
		" IFLS=0x", r.IFLS, " IMSC=0x", r.IMSC, " MIS=0x", r.MIS, " RIS=0x", r.RIS,
		" IBRD=", r.IBRD, " FBRD=", r.FBRD)
}

func drain(u *uartx.UART) {
	for u.Buffered() > 0 {
		_, _ = u.ReadByte()
	}
}

func recvExact(ctx context.Context, u *uartx.UART, n int) ([]byte, error) {
	out := make([]byte, 0, n)
	tmp := make([]byte, 128)
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

func main() {
	delay := 10
	for i := 0; i < delay; i++ {
		println("test starting in ", delay-i, " seconds")
		time.Sleep(1 * time.Second)
	}
	println("uartx probe (diagnostic)")

	u := uartx.UART1
	if err := u.Configure(uartx.UARTConfig{
		BaudRate: baud,
		TX:       uartx.UART1_TX_PIN,
		RX:       uartx.UART1_RX_PIN,
	}); err != nil {
		println("fatal:", err)
		for {
			time.Sleep(time.Hour)
		}
	}

	machine.LED.Configure(machine.PinConfig{Mode: machine.PinOutput})
	u.DebugReset()
	drain(u)

	// Phase 1: 1 KiB integrity (separate writer and reader goroutines)
	println("\n[phase] integrity-1k")
	src := make([]byte, 1024)
	var x uint32 = 0x12345678
	for i := range src {
		x = 1664525*x + 1013904223
		src[i] = byte(x >> 24)
	}
	want := sha1.Sum(src)

	ctx1, cancel1 := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel1()

	type res1 struct {
		data []byte
		err  error
	}
	rxCh1 := make(chan res1, 1)

	// Reader goroutine
	go func() {
		got, err := recvExact(ctx1, u, len(src))
		rxCh1 <- res1{data: got, err: err}
	}()

	// Writer goroutine
	go func() {
		_, _ = u.Write(src)
	}()

	// Wait for reader result
	r1 := <-rxCh1
	if r1.err != nil {
		println(" result: TIMEOUT (received", len(r1.data), "bytes)")
	} else {
		gotH := sha1.Sum(r1.data)
		mismatch := false
		for i := range want {
			if want[i] != gotH[i] {
				mismatch = true
				break
			}
		}
		if mismatch {
			println(" result: HASH MISMATCH (received", len(r1.data), "bytes)")
		} else {
			println(" result: OK (1 KiB)")
		}
	}
	printStats(u, "after integrity-1k")

	// Phase 2: burst 8 KiB (non-blocking writer) using two goroutines
	println("\n[phase] burst-8k (non-blocking writer)")
	u.DebugReset()
	drain(u)

	n := 8 * 1024
	burst := make([]byte, n)
	for i := 0; i < n; i++ {
		burst[i] = byte(i)
	}

	ctx2, cancel2 := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel2()

	type res2 struct {
		data []byte
		err  error
	}
	rxCh2 := make(chan res2, 1)

	// Writer goroutine
	go func() {
		_, _ = u.Write(burst)
	}()

	// Reader goroutine
	go func() {
		got, err := recvExact(ctx2, u, n)
		rxCh2 <- res2{data: got, err: err}
	}()

	// Wait for reader result
	r2 := <-rxCh2
	if r2.err != nil {
		println(" result: TIMEOUT (received", len(r2.data), "bytes)")
	} else {
		println(" result: received all", len(r2.data), "bytes")
	}
	printStats(u, "after burst-8k")

	// Phase 3: notify sanity (two bytes) with distinct reader and writer
	println("\n[phase] notify-2bytes")
	u.DebugReset()
	drain(u)

	ready := u.Readable()

	ctx3, cancel3 := context.WithTimeout(context.Background(), 300*time.Millisecond)
	defer cancel3()

	type res3 struct {
		data []byte
		err  error
	}
	rxCh3 := make(chan res3, 1)

	// Reader goroutine: wait for readiness then read exactly 2 bytes.
	go func() {
		select {
		case <-ready:
			ctx, cancel := context.WithTimeout(context.Background(), 200*time.Millisecond)
			got, err := recvExact(ctx, u, 2)
			cancel()
			rxCh3 <- res3{data: got, err: err}
		case <-ctx3.Done():
			rxCh3 <- res3{data: nil, err: ctx3.Err()}
		}
	}()

	// Writer goroutine
	go func() {
		_ = u.WriteByte('A')
		time.Sleep(5 * time.Millisecond)
		_ = u.WriteByte('B')
	}()

	// Wait for reader result
	r3 := <-rxCh3
	if r3.err != nil || len(r3.data) != 2 {
		println(" result: no notification within 300ms")
	} else {
		println(" result: got '", string(r3.data), "'")
	}
	printStats(u, "after notify-2bytes")

	println("\ndone")
}

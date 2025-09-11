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

	// Phase 1: 1 KiB integrity
	println("\n[phase] integrity-1k")
	src := make([]byte, 1024)
	var x uint32 = 0x12345678
	for i := range src {
		x = 1664525*x + 1013904223
		src[i] = byte(x >> 24)
	}
	want := sha1.Sum(src)
	_, _ = u.Write(src)
	ctx1, cancel1 := context.WithTimeout(context.Background(), 2*time.Second)
	got, err := recvExact(ctx1, u, len(src))
	cancel1()
	if err != nil {
		println(" result: TIMEOUT (received", len(got), "bytes)")
	} else {
		gotH := sha1.Sum(got)
		mismatch := false
		for i := range want {
			if want[i] != gotH[i] {
				mismatch = true
				break
			}
		}
		if mismatch {
			println(" result: HASH MISMATCH (received", len(got), "bytes)")
		} else {
			println(" result: OK (1 KiB)")
		}
	}
	printStats(u, "after integrity-1k")

	// Phase 2: burst 8 KiB
	println("\n[phase] burst-8k (non-blocking writer)")
	u.DebugReset()
	drain(u)
	n := 8 * 1024
	burst := make([]byte, n)
	for i := 0; i < n; i++ {
		burst[i] = byte(i)
	}
	go func() { _, _ = u.Write(burst) }()
	// Hold off reads initially to see if we force overrun/overflow.
	time.Sleep(50 * time.Millisecond)
	ctx2, cancel2 := context.WithTimeout(context.Background(), 3*time.Second)
	got2, err2 := recvExact(ctx2, u, n)
	cancel2()
	if err2 != nil {
		println(" result: TIMEOUT (received", len(got2), "bytes)")
	} else {
		println(" result: received all", len(got2), "bytes")
	}
	printStats(u, "after burst-8k")

	// Phase 3: notify sanity (two bytes)
	println("\n[phase] notify-2bytes")
	u.DebugReset()
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
		got3, _ := recvExact(ctx, u, 2)
		println(" result: got '", string(got3), "'")
	case <-time.After(300 * time.Millisecond):
		println(" result: no notification within 300ms")
	}
	printStats(u, "after notify-2bytes")

	println("\ndone")
}

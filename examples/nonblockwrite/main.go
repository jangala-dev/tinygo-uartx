package main

import (
	"device/rp"
	"fmt"
	"machine"
	"time"

	"github.com/jangala-dev/tinygo-uartx/uartx"
)

// ----- helpers/types -----

type payloadStamp struct {
	size int
	t0   time.Time
}

func bytesOf(n int, b byte) []byte {
	p := make([]byte, n)
	for i := range p {
		p[i] = b
	}
	return p
}

func firstMismatch(a, b []byte) int {
	for i := 0; i < len(a); i++ {
		if a[i] != b[i] {
			return i
		}
	}
	return -1
}

func dumpAround(label string, buf []byte, i int) {
	start := i - 8
	if start < 0 {
		start = 0
	}
	end := i + 8
	if end > len(buf) {
		end = len(buf)
	}
	fmt.Printf("%s:", label)
	for j := start; j < end; j++ {
		if j == i {
			fmt.Printf(" [%02X]", buf[j])
		} else {
			fmt.Printf(" %02X", buf[j])
		}
	}
	fmt.Println()
}

func reportConfig(u *uartx.UART) {
	fmt.Printf("UARTCR=0x%08x UARTLCR_H=0x%08x UARTFR=0x%08x IBRD=0x%08x FBRD=0x%08x FEN=%t\n",
		u.Bus.UARTCR.Get(),
		u.Bus.UARTLCR_H.Get(),
		u.Bus.UARTFR.Get(),
		u.Bus.UARTIBRD.Get(),
		u.Bus.UARTFBRD.Get(),
		(u.Bus.UARTLCR_H.Get()&rp.UART0_UARTLCR_H_FEN) != 0)
}

func txfe(u *uartx.UART) bool { return u.Bus.UARTFR.HasBits(rp.UART0_UARTFR_TXFE) }
func txff(u *uartx.UART) bool { return u.Bus.UARTFR.HasBits(rp.UART0_UARTFR_TXFF) }
func rxfe(u *uartx.UART) bool { return u.Bus.UARTFR.HasBits(rp.UART0_UARTFR_RXFE) }

func dur(d time.Duration) string {
	if d >= time.Second {
		return fmt.Sprintf("%.3fs", d.Seconds())
	}
	if d >= time.Millisecond {
		return fmt.Sprintf("%.3fms", float64(d)/1e6)
	}
	return fmt.Sprintf("%dus", d/1e3)
}

func pushInflight(inf *[8]payloadStamp, size int, t0 time.Time) {
	for i := range inf {
		if inf[i].size == 0 {
			inf[i] = payloadStamp{size: size, t0: t0}
			return
		}
	}
	older := 0
	for i := 1; i < len(inf); i++ {
		if inf[i].t0.Before(inf[older].t0) {
			older = i
		}
	}
	inf[older] = payloadStamp{size: size, t0: t0}
}

func findAndPopInflight(inf *[8]payloadStamp, size int) int {
	for i := range inf {
		if inf[i].size == size && !inf[i].t0.IsZero() {
			inf[i] = payloadStamp{}
			return i
		}
	}
	return -1
}

// ----- test parameters -----

var payloads = [][]byte{
	[]byte("hello, world\n"),
	[]byte{0x00, 0x55, 0xAA, 0xFF, 0x10, 0x20, 0x30, 0x40},
	[]byte("The quick brown fox jumps over the lazy dog.\n"),
	bytesOf(256, 0xA5),
	bytesOf(1024, 0x5A),
}

const totalTargetBytes = 32 * 1024

// ----- main test -----

func main() {
	time.Sleep(2 * time.Second) // allow USB CDC to settle

	u := uartx.UART0
	_ = u.Configure(uartx.UARTConfig{
		BaudRate: 115200,
		TX:       machine.UART_TX_PIN, // GP0
		RX:       machine.UART_RX_PIN, // GP1
	})

	reportConfig(u)
	fmt.Println("uartx loopback: single-goroutine, event-driven TX/RX")
	fmt.Println("Starting…")

	// TX state
	var pendingTx []byte
	nextPayloadIdx := 0

	// RX verification state
	expIdx, expOff := 0, 0
	verified := 0

	// Payload latency tracking
	const maxInFlight = 8
	var inflight [maxInFlight]payloadStamp
	var latMin, latMax time.Duration
	var latSum time.Duration
	var latCount int

	// Timers
	statusTick := time.NewTicker(1 * time.Second)
	defer statusTick.Stop()
	// kick := time.NewTicker(10 * time.Millisecond)
	// defer kick.Stop()

	// Throughput accounting
	start := time.Now()
	lastStatusAt := start
	lastVerified := 0

	// RX buffer
	var rxbuf [256]byte

	for verified < totalTargetBytes {
		select {
		case <-u.Readable():
			for {
				n := u.TryRead(rxbuf[:])
				if n == 0 {
					break
				}
				rx := rxbuf[:n]
				for len(rx) > 0 {
					want := payloads[expIdx%len(payloads)]
					remain := want[expOff:]
					k := len(rx)
					if len(remain) < k {
						k = len(remain)
					}

					if mis := firstMismatch(rx[:k], remain[:k]); mis >= 0 {
						fmt.Printf("Mismatch at payload %d offset %d: got 0x%02X want 0x%02X\n",
							expIdx, expOff+mis, rx[mis], remain[mis])
						dumpAround("got ", rx, mis)
						dumpAround("want", remain, mis)
					}

					verified += k
					expOff += k
					rx = rx[k:]

					if expOff == len(want) {
						stIdx := findAndPopInflight(&inflight, len(want))
						if stIdx >= 0 && !inflight[stIdx].t0.IsZero() {
							lat := time.Since(inflight[stIdx].t0)
							if latCount == 0 || lat < latMin {
								latMin = lat
							}
							if lat > latMax {
								latMax = lat
							}
							latSum += lat
							latCount++
							fmt.Printf("payload %d size %d verified, latency %s\n",
								expIdx, len(want), dur(lat))
						}
						expIdx++
						expOff = 0
					}
				}
			}

		case <-u.Writable():
			if len(pendingTx) > 0 {
				n := u.TryWrite(pendingTx)
				pendingTx = pendingTx[n:]
			}
			if len(pendingTx) == 0 {
				pl := payloads[nextPayloadIdx%len(payloads)]
				nextPayloadIdx++
				pushInflight(&inflight, len(pl), time.Now())
				pendingTx = append(pendingTx[:0], pl...)
				n := u.TryWrite(pendingTx)
				pendingTx = pendingTx[n:]
			}

		// case <-kick.C:
		// 	if len(pendingTx) == 0 {
		// 		pl := payloads[nextPayloadIdx%len(payloads)]
		// 		nextPayloadIdx++
		// 		pushInflight(&inflight, len(pl), time.Now())
		// 		pendingTx = append(pendingTx[:0], pl...)
		// 		n := u.TryWrite(pendingTx)
		// 		pendingTx = pendingTx[n:]
		// 	}

		case <-statusTick.C:
			now := time.Now()
			secs := now.Sub(lastStatusAt).Seconds()
			win := verified - lastVerified
			winRate := float64(win) / secs

			fmt.Printf("status: verified=%d (%.0f B/s) pendingTx=%d rxBuffered=%d fifo(TXFE=%t,TXFF=%t,RXFE=%t)\n",
				verified, winRate, len(pendingTx), u.Buffered(),
				txfe(u), txff(u), rxfe(u))

			lastStatusAt = now
			lastVerified = verified
		}
	}

	// Final report
	elapsed := time.Since(start)
	totalRate := float64(verified) / elapsed.Seconds()
	fmt.Printf("complete: verified=%d in %s (avg %.0f B/s)\n", verified, dur(elapsed), totalRate)
	if latCount > 0 {
		avg := time.Duration(int64(latSum) / int64(latCount))
		fmt.Printf("latency: count=%d min=%s avg=%s max=%s\n",
			latCount, dur(latMin), dur(avg), dur(latMax))
	}

	for {
		time.Sleep(2 * time.Second)
	}
}

// cmd/integrity/main.go
// Exacting cross-UART integrity test for RP2040 (Pico) using github.com/jangala-dev/tinygo-uartx/uartx
// Wiring:
//   U0 TX=GP0 -> U1 RX=GP5
//   U1 TX=GP4 -> U0 RX=GP1
// Flow control unused (RTS/CTS not connected).

package main

import (
	"context"
	"time"

	"machine"

	"github.com/jangala-dev/tinygo-uartx/uartx"
)

/*** Tunables ***/
const (
	baud           = 460800    // UART baudrate
	totalBytes     = 64 * 1024 // bytes per direction for integrity run(s)
	fullDuplex     = true      // true: duplex test; false: run each direction separately
	timeoutPerTest = 10 * time.Second
	warmupDelay    = 2 * time.Second // initial delay before starting tests

	// Mitigation for first-byte artefact:
	usePreamble  = true // send a preamble byte and have receivers skip it before verifying
	preambleByte = 0x55 // value of the preamble byte

	// Short guard after preamble before sending payload (character-time order of magnitude).
	guardDelay = 2 * time.Millisecond

	// I/O chunking and diagnostics:
	sendChunk      = 192 // bytes per SendSome burst
	recvChunk      = 256 // bytes per RecvSome read
	contextRadius  = 16  // surrounding bytes shown on mismatch (before/after pivot)
	extraFollowing = 128 // additional bytes to read and print after the first mismatch
)

/*** Patterns (deterministic) ***/
func patternA(i int) byte { return byte((i*31 + 0x55) & 0xFF) }
func patternB(i int) byte { return byte((i*17 + 0xA6) & 0xFF) }

/*** Main ***/
func main() {
	time.Sleep(warmupDelay)
	println("uartx integrity test (RP2040)")
	println("baud =", baud, "  bytes/dir =", totalBytes, "  duplex =", boolToStr(fullDuplex))
	println("U0 TX/RX = 0/1  U1 TX/RX = 4/5")

	// Hold RX high before remux to UART to ensure idle-high on the line (optional but harmless).
	machine.Pin(1).Configure(machine.PinConfig{Mode: machine.PinInputPullup})
	machine.Pin(5).Configure(machine.PinConfig{Mode: machine.PinInputPullup})

	// Configure UARTs with explicit pins.
	u0 := uartx.UART0
	u1 := uartx.UART1
	_ = u0.Configure(uartx.UARTConfig{BaudRate: baud, TX: machine.Pin(0), RX: machine.Pin(1)})
	_ = u1.Configure(uartx.UARTConfig{BaudRate: baud, TX: machine.Pin(4), RX: machine.Pin(5)})

	// LED for end-of-test indication.
	machine.LED.Configure(machine.PinConfig{Mode: machine.PinOutput})

	// Ensure RX rings are empty.
	drain(u0)
	drain(u1)

	pass, fail := 0, 0
	report := func(name, err string) {
		if err == "" {
			println("[PASS]", name)
			pass++
		} else {
			println("[FAIL]", name, ":", err)
			fail++
		}
	}

	if fullDuplex {
		err := runFullDuplex(totalBytes, u0, u1)
		report("Full-duplex integrity", err)
	} else {
		err := runOneWay("U0 -> U1", u0, u1, patternA, totalBytes)
		report("U0 -> U1 integrity", err)
		err = runOneWay("U1 -> U0", u1, u0, patternB, totalBytes)
		report("U1 -> U0 integrity", err)
	}

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
}

/*** Test runners ***/

func runOneWay(label string, tx *uartx.UART, rx *uartx.UART, gen func(int) byte, n int) string {
	drain(tx)
	drain(rx)

	ctx, cancel := context.WithTimeout(context.Background(), timeoutPerTest)
	defer cancel()

	// Start receiver that will skip preamble if configured.
	skip := 0
	if usePreamble {
		skip = 1
	}
	errCh := make(chan string, 1)
	go func() { errCh <- recvAndCheckStream(ctx, rx, gen, n, contextRadius, skip) }()

	// Send preamble (if any), short guard, then payload.
	if usePreamble {
		sendOne(tx, preambleByte)
	}
	if guardDelay > 0 {
		time.Sleep(guardDelay)
	}
	_ = sendPatternContext(ctx, tx, gen, n)

	return <-errCh
}

func runFullDuplex(n int, u0, u1 *uartx.UART) string {
	drain(u0)
	drain(u1)

	ctx, cancel := context.WithTimeout(context.Background(), timeoutPerTest)
	defer cancel()

	// Receivers first; each will skip the preamble byte if configured.
	skip := 0
	if usePreamble {
		skip = 1
	}
	errCh := make(chan string, 2)
	go func() { errCh <- recvAndCheckStream(ctx, u1, patternA, n, contextRadius, skip) }()
	go func() { errCh <- recvAndCheckStream(ctx, u0, patternB, n, contextRadius, skip) }()

	// Send preambles (if any), short guard, then payloads.
	if usePreamble {
		sendOne(u0, preambleByte)
		sendOne(u1, preambleByte)
	}
	if guardDelay > 0 {
		time.Sleep(guardDelay)
	}
	go func() { _ = sendPatternContext(ctx, u0, patternA, n) }()
	go func() { _ = sendPatternContext(ctx, u1, patternB, n) }()

	e1, e2 := <-errCh, <-errCh
	if e1 != "" {
		return e1
	}
	if e2 != "" {
		return e2
	}
	return ""
}

/*** Library-aligned helpers ***/

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

func sendPatternContext(ctx context.Context, u *uartx.UART, gen func(int) byte, n int) error {
	var buf [sendChunk]byte
	i := 0
	for i < n {
		k := sendChunk
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

func sendOne(u *uartx.UART, b byte) {
	for {
		if n := u.SendSome([]byte{b}); n == 1 {
			return
		}
		<-u.Writable()
	}
}

/*** Integrity check with diagnostics ***/

// recvAndCheckStream reads exactly n bytes and compares each byte against gen(i).
// It first discards 'skip' bytes (used for preambles). On the first mismatch it prints
// a hex dump of surrounding expected/actual context, then prints the next extraFollowing bytes.
func recvAndCheckStream(ctx context.Context, u *uartx.UART, gen func(int) byte, n int, radius int, skip int) string {
	// Discard initial 'skip' bytes if requested.
	for s := 0; s < skip; {
		var t [1]byte
		m, err := u.RecvSomeContext(ctx, t[:])
		if err != nil {
			return "timeout (waiting to skip preamble)"
		}
		if m > 0 {
			s++
		}
	}

	var buf [recvChunk]byte
	received := 0

	for received < n {
		k := n - received
		if k > len(buf) {
			k = len(buf)
		}
		m, err := u.RecvSomeContext(ctx, buf[:k])
		if err != nil {
			return "timeout"
		}
		if m == 0 {
			continue
		}

		for i := 0; i < m; i++ {
			exp := gen(received + i)
			act := buf[i]
			if act != exp {
				off := received + i
				println("First mismatch at offset", off)
				printContext(gen, off, buf[:m], i, radius)

				// Read and print additional bytes after the mismatch.
				following := make([]byte, 0, extraFollowing)
				if i+1 < m {
					following = append(following, buf[i+1:m]...)
				}
				var tmp [recvChunk]byte
				for len(following) < extraFollowing && (received+i+1+len(following)) < n {
					want := extraFollowing - len(following)
					if want > len(tmp) {
						want = len(tmp)
					}
					mm, err2 := u.RecvSomeContext(ctx, tmp[:want])
					if err2 != nil {
						break
					}
					if mm == 0 {
						continue
					}
					following = append(following, tmp[:mm]...)
				}
				printFollowing(off, following)
				return "integrity mismatch"
			}
		}

		received += m
	}

	return ""
}

/*** Context dump ***/

func printContext(gen func(int) byte, absOffset int, gotChunk []byte, rel int, radius int) {
	start := absOffset - radius
	if start < 0 {
		start = 0
	}
	end := absOffset + radius + 1

	// Build expected bytes for [start, end).
	expLen := end - start
	if expLen < 0 {
		expLen = 0
	}
	exp := make([]byte, expLen)
	for i := 0; i < expLen; i++ {
		exp[i] = gen(start + i)
	}

	// Build "actual" slice aligned to the same window (pad with zeros if not yet read).
	act := make([]byte, expLen)
	base := absOffset - rel
	for i := 0; i < expLen; i++ {
		idx := (start + i) - base
		if idx >= 0 && idx < len(gotChunk) {
			act[i] = gotChunk[idx]
		} else {
			act[i] = 0
		}
	}

	println("Context (hex): bytes", start, "to", start+len(exp)-1)
	print(" exp: ")
	printHex(exp, -1)
	print(" act: ")
	printHex(act, absOffset-start)
}

func printHex(b []byte, pivot int) {
	for i := 0; i < len(b); i++ {
		if i == pivot {
			print("[")
		} else {
			print(" ")
		}
		print(byteToHex(b[i]))
		if i == pivot {
			print("]")
		}
	}
	println("")
}

func printFollowing(mismatchOffset int, following []byte) {
	println("Following bytes actually received after mismatch (next", len(following), "bytes):")
	if len(following) == 0 {
		println(" <none>")
		return
	}
	base := mismatchOffset + 1
	for i := 0; i < len(following); i += 16 {
		end := i + 16
		if end > len(following) {
			end = len(following)
		}
		print("  +", base+i, ":")
		for j := i; j < end; j++ {
			print(" ", byteToHex(following[j]))
		}
		println("")
	}
}

/*** Utilities ***/

func byteToHex(v byte) string {
	const hexdigits = "0123456789ABCDEF"
	var s [2]byte
	s[0] = hexdigits[(v>>4)&0xF]
	s[1] = hexdigits[v&0xF]
	return string(s[:])
}

func blink(pin machine.Pin, times int, on time.Duration) {
	for i := 0; i < times; i++ {
		pin.High()
		time.Sleep(on)
		pin.Low()
		time.Sleep(on)
	}
}

func boolToStr(b bool) string {
	if b {
		return "true"
	}
	return "false"
}

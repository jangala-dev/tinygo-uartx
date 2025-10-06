package main

import (
	"device/rp"
	"machine"
	"runtime/interrupt"
	"time"
)

/*
RP2040 PL011 diagnostic (UART0 <-> UART1 on Pico)

Wiring assumed:
  UART0 TX (GP0) -> UART1 RX (GP9)
  UART1 TX (GP8) -> UART0 RX (GP1)
*/

// ---------- Tunables ----------
const (
	baud = 921600
	// IFLS levels: 0=1/8, 1=1/4, 2=1/2, 3=3/4, 4=7/8
	IFLS_RX = 2 // 1/2
	IFLS_TX = 2 // 1/2
	// Drain backoff when ring is empty (kept short for high baud)
	emptyBackoff = 50 * time.Microsecond
)

// ---------- Simple ring buffer for RX capture ----------
const rxBufSize = 4096 // power-of-two

type ring struct {
	b     [rxBufSize]byte
	head  int
	tail  int
	drops uint32
}

func (r *ring) put(b byte) bool {
	n := (r.head + 1) & (rxBufSize - 1)
	if n == r.tail {
		r.drops++
		return false // full
	}
	r.b[r.head] = b
	r.head = n
	return true
}
func (r *ring) get() (byte, bool) {
	if r.tail == r.head {
		return 0, false
	}
	v := r.b[r.tail]
	r.tail = (r.tail + 1) & (rxBufSize - 1)
	return v, true
}
func (r *ring) used() int {
	if r.head >= r.tail {
		return r.head - r.tail
	}
	return rxBufSize - r.tail + r.head
}
func (r *ring) reset() { r.head, r.tail, r.drops = 0, 0, 0 }

// ---------- Per-UART diagnostic counters ----------
type uartDiag struct {
	u *rp.UART0_Type

	// RX storage
	rx ring

	// Error flags seen in UARTDR upper bits
	errOE uint32
	errBE uint32
	errPE uint32
	errFE uint32

	// Interrupt cause counters
	irqRXLevel   uint32 // RXMIS
	irqRXTimeout uint32 // RTMIS
	irqTX        uint32 // TXMIS

	// FIFO snapshots
	snapFRNotEmpty uint32 // RXFE==0 snapshots inside ISR
	snapFREmpty    uint32 // RXFE==1 snapshots inside ISR
}

var u0 = &uartDiag{u: rp.UART0}
var u1 = &uartDiag{u: rp.UART1}

// ---------- Minimal formatting helpers (no fmt) ----------
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
func u32hex(v uint32) string {
	const hd = "0123456789abcdef"
	var b [8]byte
	for i := 0; i < 8; i++ {
		shift := uint(28 - 4*i)
		b[i] = hd[(v>>shift)&0xF]
	}
	return string(b[:])
}
func printKV(k string, v string)  { print(k); print(": "); println(v) }
func printU32(k string, v uint32) { printKV(k, "0x"+u32hex(v)) }

// ---------- RP2040 PL011 helpers ----------
func setIFLS(u *rp.UART0_Type, rx, tx uint32) {
	v := (rx << rp.UART0_UARTIFLS_RXIFLSEL_Pos) | (tx << rp.UART0_UARTIFLS_TXIFLSEL_Pos)
	u.UARTIFLS.Set(v)
}
func enableRXIRQs(u *rp.UART0_Type) {
	u.UARTIMSC.Set(rp.UART0_UARTIMSC_RXIM | rp.UART0_UARTIMSC_RTIM)
}
func disableRXIRQs(u *rp.UART0_Type) {
	u.UARTIMSC.ClearBits(rp.UART0_UARTIMSC_RXIM | rp.UART0_UARTIMSC_RTIM)
}
func disableTXIRQ(u *rp.UART0_Type) {
	u.UARTIMSC.ClearBits(rp.UART0_UARTIMSC_TXIM)
}
func clearAllICR(u *rp.UART0_Type) {
	u.UARTICR.Set(rp.UART0_UARTICR_RXIC | rp.UART0_UARTICR_RTIC | rp.UART0_UARTICR_TXIC)
}
func fifoEmpty(u *rp.UART0_Type) bool  { return u.UARTFR.HasBits(rp.UART0_UARTFR_RXFE) }
func txFifoFull(u *rp.UART0_Type) bool { return u.UARTFR.HasBits(rp.UART0_UARTFR_TXFF) }
func resetUARTSel(u *rp.UART0_Type) {
	var resetVal uint32
	switch u {
	case rp.UART0:
		resetVal = rp.RESETS_RESET_UART0
	case rp.UART1:
		resetVal = rp.RESETS_RESET_UART1
	}
	rp.RESETS.RESET.SetBits(resetVal)
	rp.RESETS.RESET.ClearBits(resetVal)
	for !rp.RESETS.RESET_DONE.HasBits(resetVal) {
	}
}
func set8N1FIFOs(u *rp.UART0_Type) {
	const WLEN = 3 << rp.UART0_UARTLCR_H_WLEN_Pos // 8 data bits
	const STP2 = 0                                // 1 stop
	const FEN = rp.UART0_UARTLCR_H_FEN
	u.UARTLCR_H.Set(WLEN | STP2 | FEN) // parity none + FIFOs
}
func enableUART(u *rp.UART0_Type) {
	const CR = rp.UART0_UARTCR_UARTEN | rp.UART0_UARTCR_RXE | rp.UART0_UARTCR_TXE
	u.UARTCR.SetBits(CR)
}

// Properly rounded divisors (64-bit, scaled by 64)
func setBaud(u *rp.UART0_Type, br uint32) {
	clk := uint64(machine.CPUFrequency())
	baud := uint64(br)
	if baud == 0 {
		baud = 115200
	}
	den := 16 * baud
	ibrd := clk / den
	rem := clk % den
	fbrd := (rem*64 + den/2) / den
	if fbrd >= 64 {
		ibrd++
		fbrd = 0
	}
	if ibrd == 0 {
		ibrd = 1
		fbrd = 0
	}
	if ibrd > 0xFFFF {
		ibrd = 0xFFFF
		fbrd = 0
	}
	u.UARTIBRD.Set(uint32(ibrd))
	u.UARTFBRD.Set(uint32(fbrd))
	// required dummy LCR_H write after divisor change
	u.UARTLCR_H.SetBits(0)
}

// Correct 'actual baud' from divisors (scaled integer maths)
func actualBaud(u *rp.UART0_Type) uint32 {
	clk := uint64(machine.CPUFrequency()) // clk_peri
	ibrd := uint64(u.UARTIBRD.Get())
	fbrd := uint64(u.UARTFBRD.Get())
	den := 16 * (ibrd*64 + fbrd)
	if den == 0 {
		return 0
	}
	return uint32((clk*64 + den/2) / den) // rounded
}

// ---------- Interrupts ----------
func (d *uartDiag) isr(interrupt.Interrupt) {
	mis := d.u.UARTMIS.Get()

	// RX causes
	if (mis & (rp.UART0_UARTMIS_RXMIS | rp.UART0_UARTMIS_RTMIS)) != 0 {
		if mis&rp.UART0_UARTMIS_RXMIS != 0 {
			d.irqRXLevel++
		}
		if mis&rp.UART0_UARTMIS_RTMIS != 0 {
			d.irqRXTimeout++
		}
		// Drain RX FIFO
		for !fifoEmpty(d.u) {
			dr := d.u.UARTDR.Get()
			// Upper bits carry FE|PE|BE|OE (bits 8..11)
			if dr&(1<<11) != 0 {
				d.errOE++
			}
			if dr&(1<<10) != 0 {
				d.errBE++
			}
			if dr&(1<<9) != 0 {
				d.errPE++
			}
			if dr&(1<<8) != 0 {
				d.errFE++
			}
			_ = d.rx.put(byte(dr & 0xFF))
			d.snapFRNotEmpty++
		}
		if fifoEmpty(d.u) {
			d.snapFREmpty++
		}
		clearAllICR(d.u)
	}

	// TX cause (not used here but count if ever enabled)
	if mis&rp.UART0_UARTMIS_TXMIS != 0 {
		d.irqTX++
		clearAllICR(d.u)
	}
}

// ISR shims (package-level; no closures allowed in TinyGo)
func irqUART0(i interrupt.Interrupt) { u0.isr(i) }
func irqUART1(i interrupt.Interrupt) { u1.isr(i) }

// ---------- TX helpers (busy-wait deterministic) ----------
func txWrite(u *rp.UART0_Type, b byte) {
	for txFifoFull(u) {
	}
	u.UARTDR.Set(uint32(b))
}
func sendPattern(u *rp.UART0_Type, gen func(i int) byte, n int) {
	for i := 0; i < n; i++ {
		txWrite(u, gen(i))
	}
}
func sendSlice(u *rp.UART0_Type, p []byte) {
	for i := 0; i < len(p); i++ {
		txWrite(u, p[i])
	}
}

// ---------- Patterns and tiny SHA-1 ----------
func patA(i int) byte { return byte((i*31 + 0x55) & 0xFF) }
func patB(i int) byte { return byte((i*17 + 0xA6) & 0xFF) }

type sha1ctx struct {
	h   [5]uint32
	x   [64]byte
	n   int
	len uint64
}

func (s *sha1ctx) init() {
	s.h[0] = 0x67452301
	s.h[1] = 0xEFCDAB89
	s.h[2] = 0x98BADCFE
	s.h[3] = 0x10325476
	s.h[4] = 0xC3D2E1F0
	s.n = 0
	s.len = 0
}
func rol(v uint32, n uint) uint32 { return (v<<n | v>>(32-n)) }
func (s *sha1ctx) block(p []byte) {
	var w [80]uint32
	for i := 0; i < 16; i++ {
		j := 4 * i
		w[i] = uint32(p[j])<<24 | uint32(p[j+1])<<16 | uint32(p[j+2])<<8 | uint32(p[j+3])
	}
	for i := 16; i < 80; i++ {
		w[i] = rol(w[i-3]^w[i-8]^w[i-14]^w[i-16], 1)
	}
	a, b, c, d, e := s.h[0], s.h[1], s.h[2], s.h[3], s.h[4]
	for i := 0; i < 80; i++ {
		var f, k uint32
		switch {
		case i < 20:
			f = (b & c) | (^b & d)
			k = 0x5A827999
		case i < 40:
			f = b ^ c ^ d
			k = 0x6ED9EBA1
		case i < 60:
			f = (b & c) | (b & d) | (c & d)
			k = 0x8F1BBCDC
		default:
			f = b ^ c ^ d
			k = 0xCA62C1D6
		}
		t := rol(a, 5) + f + e + k + w[i]
		e, d, c, b, a = d, c, rol(b, 30), a, t
	}
	s.h[0] += a
	s.h[1] += b
	s.h[2] += c
	s.h[3] += d
	s.h[4] += e
}
func (s *sha1ctx) write(p []byte) {
	s.len += uint64(len(p)) * 8
	for _, b := range p {
		s.x[s.n] = b
		s.n++
		if s.n == 64 {
			s.block(s.x[:])
			s.n = 0
		}
	}
}
func (s *sha1ctx) sum() [20]byte {
	var tmp [64]byte
	copy(tmp[:], s.x[:s.n])
	tmp[s.n] = 0x80
	n := s.n + 1
	if n > 56 {
		for i := n; i < 64; i++ {
			tmp[i] = 0
		}
		s.block(tmp[:])
		for i := 0; i < 56; i++ {
			tmp[i] = 0
		}
	} else {
		for i := n; i < 56; i++ {
			tmp[i] = 0
		}
	}
	L := s.len
	for i := 0; i < 8; i++ {
		tmp[63-i] = byte(L & 0xFF)
		L >>= 8
	}
	s.block(tmp[:])
	var out [20]byte
	for i := 0; i < 5; i++ {
		out[4*i+0] = byte(s.h[i] >> 24)
		out[4*i+1] = byte(s.h[i] >> 16)
		out[4*i+2] = byte(s.h[i] >> 8)
		out[4*i+3] = byte(s.h[i])
	}
	return out
}
func sha1Pattern(gen func(i int) byte, n int) [20]byte {
	var s sha1ctx
	s.init()
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
		s.write(buf[:k])
		i += k
	}
	return s.sum()
}

// ---------- Test orchestration ----------
func main() {
	time.Sleep(2 * time.Second)
	println("PL011 diagnostic start")

	// Pin mux for UARTs on Pico
	machine.UART0_TX_PIN.Configure(machine.PinConfig{Mode: machine.PinUART})
	machine.UART0_RX_PIN.Configure(machine.PinConfig{Mode: machine.PinUART})
	machine.UART1_TX_PIN.Configure(machine.PinConfig{Mode: machine.PinUART})
	machine.UART1_RX_PIN.Configure(machine.PinConfig{Mode: machine.PinUART})

	// Reset and configure (order chosen to avoid stray flags)
	for _, uu := range []*rp.UART0_Type{rp.UART0, rp.UART1} {
		resetUARTSel(uu)
		setBaud(uu, baud)
		set8N1FIFOs(uu)
		clearAllICR(uu)
		setIFLS(uu, IFLS_RX, IFLS_TX)
		enableRXIRQs(uu)
		enableUART(uu)
		disableTXIRQ(uu) // not used in this diagnostic
	}

	// Hook interrupts (highest priority for prompt RX service)
	irq0 := interrupt.New(rp.IRQ_UART0_IRQ, irqUART0)
	irq1 := interrupt.New(rp.IRQ_UART1_IRQ, irqUART1)
	irq0.SetPriority(0x00)
	irq1.SetPriority(0x00)
	irq0.Enable()
	irq1.Enable()

	// Report config
	report("UART0", rp.UART0)
	report("UART1", rp.UART1)

	// --- Polling sanity (bypass ISR): 64 bytes U0->U1 with RX IRQs disabled on U1 ---
	disableRXIRQs(rp.UART1)
	sendSlice(rp.UART0, []byte("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789"))
	pollCnt := 0
	deadline := time.Now().Add(200 * time.Millisecond)
	for pollCnt < 64 && time.Now().Before(deadline) {
		if !rp.UART1.UARTFR.HasBits(rp.UART0_UARTFR_RXFE) {
			_ = byte(rp.UART1.UARTDR.Get() & 0xFF) // discard
			pollCnt++
		}
	}
	enableRXIRQs(rp.UART1)
	printKV("polling.U0toU1.bytes", itoa(pollCnt))

	// Tests (streamed drains)

	run("U0->U1 4KiB streamed", func() string {
		u0.rx.reset()
		u1.rx.reset()
		u1.errOE, u1.errBE, u1.errPE, u1.errFE = 0, 0, 0, 0
		u1.irqRXLevel, u1.irqRXTimeout, u1.snapFRNotEmpty, u1.snapFREmpty = 0, 0, 0, 0

		n := 4 * 1024
		want := sha1Pattern(patA, n)

		go sendPattern(rp.UART0, patA, n)

		var s sha1ctx
		s.init()
		rem := n
		t0 := time.Now()
		for rem > 0 && time.Since(t0) < 3*time.Second {
			if b, ok := u1.rx.get(); ok {
				s.write([]byte{b})
				rem--
			} else {
				time.Sleep(emptyBackoff)
			}
		}
		if rem != 0 {
			println("  timeout diagnostics (U1):")
			dumpDiag("U1", u1)
			printU32("    UARTFR   ", rp.UART1.UARTFR.Get())
			printU32("    UARTMIS  ", rp.UART1.UARTMIS.Get())
			printU32("    UARTIMSC ", rp.UART1.UARTIMSC.Get())
			return "timeout"
		}
		got := s.sum()
		ok := true
		for i := 0; i < 20; i++ {
			ok = ok && (got[i] == want[i])
		}
		dumpDiag("U1", u1)
		if !ok {
			return "hash mismatch"
		}
		return ""
	})

	run("U1->U0 4KiB streamed", func() string {
		u0.rx.reset()
		u1.rx.reset()
		u0.errOE, u0.errBE, u0.errPE, u0.errFE = 0, 0, 0, 0
		u0.irqRXLevel, u0.irqRXTimeout, u0.snapFRNotEmpty, u0.snapFREmpty = 0, 0, 0, 0

		n := 4 * 1024
		want := sha1Pattern(patB, n)

		go sendPattern(rp.UART1, patB, n)

		var s sha1ctx
		s.init()
		rem := n
		t0 := time.Now()
		for rem > 0 && time.Since(t0) < 3*time.Second {
			if b, ok := u0.rx.get(); ok {
				s.write([]byte{b})
				rem--
			} else {
				time.Sleep(emptyBackoff)
			}
		}
		if rem != 0 {
			println("  timeout diagnostics (U0):")
			dumpDiag("U0", u0)
			printU32("    UARTFR   ", rp.UART0.UARTFR.Get())
			printU32("    UARTMIS  ", rp.UART0.UARTMIS.Get())
			printU32("    UARTIMSC ", rp.UART0.UARTIMSC.Get())
			return "timeout"
		}
		got := s.sum()
		ok := true
		for i := 0; i < 20; i++ {
			ok = ok && (got[i] == want[i])
		}
		dumpDiag("U0", u0)
		if !ok {
			return "hash mismatch"
		}
		return ""
	})

	run("Full-duplex 8KiB each way (streamed)", func() string {
		u0.rx.reset()
		u1.rx.reset()
		u0.errOE, u0.errBE, u0.errPE, u0.errFE = 0, 0, 0, 0
		u1.errOE, u1.errBE, u1.errPE, u1.errFE = 0, 0, 0, 0

		n := 8 * 1024
		wantA := sha1Pattern(patA, n)
		wantB := sha1Pattern(patB, n)

		// Start senders
		go sendPattern(rp.UART0, patA, n)
		go sendPattern(rp.UART1, patB, n)

		// Concurrent streamed drains into hashes
		errCh := make(chan string, 2)
		var sA, sB sha1ctx
		sA.init()
		sB.init()

		go func() {
			rem := n
			t0 := time.Now()
			for rem > 0 && time.Since(t0) < 6*time.Second {
				if b, ok := u1.rx.get(); ok {
					sA.write([]byte{b})
					rem--
				} else {
					time.Sleep(emptyBackoff)
				}
			}
			if rem != 0 {
				errCh <- "timeout A"
				return
			}
			if sA.sum() != wantA {
				errCh <- "U0->U1 hash mismatch"
				return
			}
			errCh <- ""
		}()
		go func() {
			rem := n
			t0 := time.Now()
			for rem > 0 && time.Since(t0) < 6*time.Second {
				if b, ok := u0.rx.get(); ok {
					sB.write([]byte{b})
					rem--
				} else {
					time.Sleep(emptyBackoff)
				}
			}
			if rem != 0 {
				errCh <- "timeout B"
				return
			}
			if sB.sum() != wantB {
				errCh <- "U1->U0 hash mismatch"
				return
			}
			errCh <- ""
		}()

		e1, e2 := <-errCh, <-errCh
		dumpDiag("U0", u0)
		dumpDiag("U1", u1)
		if e1 != "" || e2 != "" {
			if e1 != "" {
				return e1
			}
			return e2
		}
		return ""
	})

	println("")
	println("All tests completed")
}

// ---------- Small test harness ----------
func run(name string, f func() string) {
	println("")
	println("[Test]", name)
	msg := f()
	if msg == "" {
		println("  PASS")
	} else {
		println("  FAIL:", msg)
	}
}

func report(label string, u *rp.UART0_Type) {
	println("")
	println(label, "config")
	printU32("UARTCR   ", u.UARTCR.Get())
	printU32("UARTLCR_H", u.UARTLCR_H.Get())
	printU32("UARTFR   ", u.UARTFR.Get())
	printU32("UARTIBRD ", u.UARTIBRD.Get())
	printU32("UARTFBRD ", u.UARTFBRD.Get())
	printU32("UARTIFLS ", u.UARTIFLS.Get())
	ab := actualBaud(u)
	printKV("baud.target", itoa(baud))
	printKV("baud.actual", itoa(int(ab)))
	if baud > 0 {
		errppm := int((int64(ab) - int64(baud)) * 1_000_000 / int64(baud))
		printKV("baud.err_ppm", itoa(errppm))
	}
}

func dumpDiag(label string, d *uartDiag) {
	println("  [" + label + "] RX stats:")
	printKV("    bytes", itoa(d.rx.used()))
	printKV("    drops", itoa(int(d.rx.drops)))
	printKV("    irq.rx.level", itoa(int(d.irqRXLevel)))
	printKV("    irq.rx.timeout", itoa(int(d.irqRXTimeout)))
	printKV("    snap.rx.notempty", itoa(int(d.snapFRNotEmpty)))
	printKV("    snap.rx.empty", itoa(int(d.snapFREmpty)))
	printKV("    err.OE", itoa(int(d.errOE)))
	printKV("    err.BE", itoa(int(d.errBE)))
	printKV("    err.PE", itoa(int(d.errPE)))
	printKV("    err.FE", itoa(int(d.errFE)))
}

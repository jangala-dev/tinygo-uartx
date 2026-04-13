// uartx/ringbuffer.go

//go:build atmega || esp || nrf || sam || sifive || stm32 || k210 || nxp || rp2040 || rp2350

// RingBuffer is the single-producer/single-consumer byte queue that sits
// between the UART interrupt handler and the foreground reader. It used to
// be an API-compatible replacement for TinyGo's machine.RingBuffer, whose
// default capacity is 128 bytes. That default is too small for interrupt-
// driven use at typical bauds (see the file-level comment on bufferSize);
// the size and index width here have been raised so the ring can absorb
// longer foreground stalls without silently dropping bytes.

package uartx

import "runtime/volatile"

// bufferSize is the per-instance RX/TX ring capacity in bytes.
//
// At 115200 baud a byte arrives every ~87 us, so 128 bytes is only about
// 11 ms of headroom between the ISR producing and the foreground consuming.
// Any stall longer than that (a flash page program, a GC pause, a heavy
// scheduled task) causes the ISR's Put to start failing and bytes are lost
// with no visible symptom other than downstream data corruption.
//
// 4096 bytes gives ~355 ms of headroom at 115200 (and scales linearly with
// baud), which is large enough to absorb any realistic stall on the MCUs
// this package currently targets. The cost is 2 * bufferSize bytes of
// static RAM per UART (one RX, one TX); on RP2040/RP2350 with hundreds of
// KB of SRAM this is negligible.
//
// bufferSize must remain a power of two so `(h+1) % bufferSize` compiles
// to a bitmask.
const bufferSize uint16 = 4096

// RingBuffer is a byte ring buffer. The head/tail indices are uint16 so
// the capacity can exceed 255 bytes without wraparound ambiguity; the
// existing head-minus-tail arithmetic still works because uint16
// subtraction is modulo 2^16 which is a multiple of bufferSize.
type RingBuffer struct {
	rxbuffer [bufferSize]volatile.Register8
	head     volatile.Register16
	tail     volatile.Register16
}

// NewRingBuffer returns a new ring buffer.
func NewRingBuffer() *RingBuffer {
	return &RingBuffer{}
}

// Size returns the total capacity of the buffer in bytes.
func (rb *RingBuffer) Size() uint16 {
	return bufferSize
}

// Used returns how many bytes in buffer have been used.
func (rb *RingBuffer) Used() uint16 {
	return uint16(rb.head.Get() - rb.tail.Get())
}

// Put stores a byte in the buffer. If the buffer is already full, it returns false.
func (rb *RingBuffer) Put(val byte) bool {
	if rb.Used() == bufferSize { // full
		return false
	}
	h := rb.head.Get()
	rb.rxbuffer[(h+1)%bufferSize].Set(val) // 1) write data
	rb.head.Set(h + 1)                     // 2) publish
	return true
}

// Get returns a byte from the buffer. If the buffer is empty, it returns (0, false).
func (rb *RingBuffer) Get() (byte, bool) {
	if rb.Used() == 0 {
		return 0, false
	}
	t := rb.tail.Get()
	v := rb.rxbuffer[(t+1)%bufferSize].Get() // 1) read current element
	rb.tail.Set(t + 1)                       // 2) publish consumption
	return v, true
}

// Clear resets the head and tail pointers to zero.
func (rb *RingBuffer) Clear() {
	rb.head.Set(0)
	rb.tail.Set(0)
}

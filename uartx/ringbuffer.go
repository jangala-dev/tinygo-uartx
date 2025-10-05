// uartx/ringbuffer.go
// An API-compatible replacement for machine.RingBuffer with an added Size() method.
// Methods and semantics match TinyGo's implementation.

package uartx

import "runtime/volatile"

// Choose a power-of-two size for efficient modulo.
const bufferSize uint8 = 128

// RingBuffer is a byte ring buffer compatible with TinyGo's machine.RingBuffer.
type RingBuffer struct {
	rxbuffer [bufferSize]volatile.Register8
	head     volatile.Register8
	tail     volatile.Register8
}

// NewRingBuffer returns a new ring buffer.
func NewRingBuffer() *RingBuffer {
	return &RingBuffer{}
}

// Size returns the total capacity of the buffer in bytes.
func (rb *RingBuffer) Size() uint8 {
	return bufferSize
}

// Used returns how many bytes in buffer have been used.
func (rb *RingBuffer) Used() uint8 {
	return uint8(rb.head.Get() - rb.tail.Get())
}

// Put stores a byte in the buffer. If the buffer is already full, it returns false.
func (rb *RingBuffer) Put(val byte) bool {
	if rb.Used() != bufferSize {
		rb.head.Set(rb.head.Get() + 1)
		rb.rxbuffer[rb.head.Get()%bufferSize].Set(val)
		return true
	}
	return false
}

// Get returns a byte from the buffer. If the buffer is empty, it returns (0, false).
func (rb *RingBuffer) Get() (byte, bool) {
	if rb.Used() != 0 {
		rb.tail.Set(rb.tail.Get() + 1)
		return rb.rxbuffer[rb.tail.Get()%bufferSize].Get(), true
	}
	return 0, false
}

// Clear resets the head and tail pointers to zero.
func (rb *RingBuffer) Clear() {
	rb.head.Set(0)
	rb.tail.Set(0)
}

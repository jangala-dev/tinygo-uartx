//go:build !rp2040 && !rp2350

package uartx

import (
	"context"
	"errors"
	"time"
)

// Host shim: minimal UART for unit tests, no device/rp or machine deps.

var ErrBufferEmpty = errors.New("UART buffer empty (host)")

type UART struct {
	rx     ring
	notify chan struct{}
	closed chan struct{}
}

// Public instances to mirror real build.
var (
	UART0 = &_UART0
	UART1 = &_UART1

	_UART0 = UART{notify: make(chan struct{}, 1), closed: make(chan struct{})}
	_UART1 = UART{notify: make(chan struct{}, 1), closed: make(chan struct{})}
)

// ---------- Methods exercised by tests ----------

func (u *UART) Read(p []byte) (int, error) {
	if u.rx.len() == 0 {
		return 0, nil
	}
	return u.rx.readInto(p), nil
}

func (u *UART) ReadByte() (byte, error) {
	if u.rx.len() == 0 {
		return 0, ErrBufferEmpty
	}
	return u.rx.get(), nil
}

func (u *UART) Buffered() int { return u.rx.len() }

func (u *UART) Receive(b byte) {
	wasEmpty := u.rx.len() == 0
	u.rx.put(b)
	if wasEmpty {
		select {
		case u.notify <- struct{}{}:
		default:
		}
	}
}

func (u *UART) WaitReadable(ctx context.Context) error {
	if u.Buffered() > 0 {
		return nil
	}
	for {
		select {
		case <-u.notify:
			if u.Buffered() > 0 {
				return nil
			}
		case <-u.closed:
			return context.Canceled
		case <-ctx.Done():
			return ctx.Err()
		}
	}
}

func (u *UART) ReadBlocking(ctx context.Context, p []byte) (int, error) {
	for {
		if n, _ := u.Read(p); n > 0 {
			return n, nil
		}
		if err := u.WaitReadable(ctx); err != nil {
			return 0, err
		}
	}
}

func (u *UART) ReadFullBlocking(ctx context.Context, p []byte) (int, error) {
	read := 0
	for read < len(p) {
		if n, _ := u.Read(p[read:]); n > 0 {
			read += n
			continue
		}
		if err := u.WaitReadable(ctx); err != nil {
			return read, err
		}
	}
	return read, nil
}

func (u *UART) ReadByteBlocking(ctx context.Context) (byte, error) {
	for {
		if b, err := u.ReadByte(); err == nil {
			return b, nil
		}
		if err := u.WaitReadable(ctx); err != nil {
			return 0, err
		}
	}
}

func (u *UART) ReadWithTimeout(p []byte, d time.Duration) (int, error) {
	ctx, cancel := context.WithTimeout(context.Background(), d)
	defer cancel()
	return u.ReadBlocking(ctx, p)
}

func (u *UART) Close() error {
	select {
	case <-u.closed:
	default:
		close(u.closed)
	}
	return nil
}

// tryNotify is used by tests to simulate the ISR wake-up.
func (u *UART) tryNotify() {
	select {
	case u.notify <- struct{}{}:
	default:
	}
}

// -------- tiny ring buffer (bytes) --------

type ring struct {
	buf        [512]byte
	head, tail int
}

func (r *ring) len() int {
	if r.head >= r.tail {
		return r.head - r.tail
	}
	return len(r.buf) - r.tail + r.head
}

func (r *ring) put(b byte) {
	next := (r.head + 1) % len(r.buf)
	if next == r.tail {
		// drop oldest
		r.tail = (r.tail + 1) % len(r.buf)
	}
	r.buf[r.head] = b
	r.head = next
}

func (r *ring) get() byte {
	if r.len() == 0 {
		return 0
	}
	b := r.buf[r.tail]
	r.tail = (r.tail + 1) % len(r.buf)
	return b
}

func (r *ring) readInto(p []byte) int {
	n := 0
	for n < len(p) && r.len() > 0 {
		p[n] = r.get()
		n++
	}
	return n
}

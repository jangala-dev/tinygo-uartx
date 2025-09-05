package uartx

import (
	"context"
	"testing"
	"time"
)

// newTestUART returns a fresh host UART (no hardware).
func newTestUART() *UART {
	// Copy so tests don't mutate the package singletons.
	u := *UART0
	u.rx = ring{}
	u.notify = make(chan struct{}, 1)
	u.closed = make(chan struct{})
	return &u
}

func TestRead_NonBlockingSemantics(t *testing.T) {
	u := newTestUART()
	buf := make([]byte, 8)

	if n, err := u.Read(buf); err != nil || n != 0 {
		t.Fatalf("Read on empty: n=%d err=%v; want 0,nil", n, err)
	}

	u.Receive('A')
	u.Receive('B')
	u.Receive('C')

	n, err := u.Read(buf)
	if err != nil {
		t.Fatalf("unexpected err: %v", err)
	}
	if n != 3 || string(buf[:n]) != "ABC" {
		t.Fatalf("got n=%d data=%q; want 3, \"ABC\"", n, string(buf[:n]))
	}

	if n, _ := u.Read(buf); n != 0 {
		t.Fatalf("expected empty after drain, got n=%d", n)
	}
}

func TestReadByteBlocking_UnblocksOnNotify(t *testing.T) {
	u := newTestUART()

	ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
	defer cancel()

	done := make(chan struct{})
	var got byte
	var err error

	go func() {
		defer close(done)
		got, err = u.ReadByteBlocking(ctx)
	}()

	time.Sleep(20 * time.Millisecond)

	u.Receive('Z')
	u.tryNotify()

	select {
	case <-done:
	case <-time.After(300 * time.Millisecond):
		t.Fatal("timeout waiting for ReadByteBlocking")
	}

	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if got != 'Z' {
		t.Fatalf("got %q want %q", got, 'Z')
	}
}

func TestReadBlocking_ReadsSomeBytes(t *testing.T) {
	u := newTestUART()

	ctx, cancel := context.WithTimeout(context.Background(), 800*time.Millisecond)
	defer cancel()

	buf := make([]byte, 8)
	done := make(chan struct{})
	var n int
	var err error

	go func() {
		defer close(done)
		n, err = u.ReadBlocking(ctx, buf)
	}()

	time.Sleep(10 * time.Millisecond)

	u.Receive('x')
	u.Receive('y')
	u.Receive('z')
	u.tryNotify()

	select {
	case <-done:
	case <-time.After(400 * time.Millisecond):
		t.Fatal("timeout waiting for ReadBlocking")
	}

	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if n <= 0 || string(buf[:n]) != "xyz"[:n] {
		t.Fatalf("unexpected data: n=%d data=%q", n, string(buf[:n]))
	}
}

func TestReadFullBlocking_ReadsExactLen(t *testing.T) {
	u := newTestUART()

	ctx, cancel := context.WithTimeout(context.Background(), 1*time.Second)
	defer cancel()

	want := []byte("HELLO")
	got := make([]byte, len(want))

	done := make(chan struct{})
	var n int
	var err error

	go func() {
		defer close(done)
		n, err = u.ReadFullBlocking(ctx, got)
	}()

	time.Sleep(10 * time.Millisecond)

	for i := range want {
		u.Receive(want[i])
		if i == 0 {
			u.tryNotify()
		}
		time.Sleep(5 * time.Millisecond)
	}

	select {
	case <-done:
	case <-time.After(600 * time.Millisecond):
		t.Fatal("timeout waiting for ReadFullBlocking")
	}

	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if n != len(want) || string(got) != string(want) {
		t.Fatalf("got %q (n=%d), want %q", string(got), n, string(want))
	}
}

func TestWaitReadable_RespectsClose(t *testing.T) {
	u := newTestUART()

	ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
	defer cancel()

	done := make(chan error, 1)
	go func() { done <- u.WaitReadable(ctx) }()

	close(u.closed)

	select {
	case err := <-done:
		if err == nil {
			t.Fatal("expected non-nil error after close")
		}
	case <-time.After(200 * time.Millisecond):
		t.Fatal("timeout waiting for WaitReadable to return after close")
	}
}

func TestNonBlockingReadAfterMultipleNotifies(t *testing.T) {
	u := newTestUART()
	u.tryNotify()
	u.tryNotify()
	u.tryNotify() // no data
	if n, err := u.Read(make([]byte, 4)); err != nil || n != 0 {
		t.Fatalf("Read on empty after notifies: n=%d err=%v", n, err)
	}
}

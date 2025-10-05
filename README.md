# tinygo-uartx

**TinyGo 0.39:** this currently requires the single-core scheduler. Build with `-scheduler tasks`.

An experimental UART driver for RP2040/RP2350 in TinyGo.

It mirrors the public API of `machine.UART` (non-blocking `Read`, `ReadByte`; blocking `Write`, `WriteByte`) and adds a small number of helpers for **event-driven I/O** so a single goroutine can `select` over RX and TX without polling.

## What’s different from `machine.UART`

* **RX helpers (blocking without polling):**

  * `Readable() <-chan struct{}`
  * `WaitReadableContext(ctx)`
  * `RecvSomeContext(ctx, p)`
  * `RecvByteContext(ctx)`

* **TX helpers (event-driven writes):**

  * `Writable() <-chan struct{}`
  * `WaitWritableContext(ctx)`
  * `SendSomeContext(ctx, p)` and non-blocking `SendSome(p)`
    (useful inside a `select` to make incremental progress)

* **RP2040/RP2350 specifics:**

  * Hardware FIFOs are **enabled** during `Configure`/`SetFormat` (PL011 `FEN=1`).
  * TX/RX are fully interrupt-driven. No polling or `Gosched()` loops are used.
  * `Writable()` emits an **initial notification** after `Configure` so applications can start writing without a “kick”.

Blocking `Write`/`WriteByte` remain available and are implemented on top of the interrupt-driven path. “Drained” is defined as **TX FIFO empty**.

## Quick test on hardware (UART0 loopback)

1. Wire **UART0 TX → UART0 RX** on a Pico (e.g. GP0→GP1).
2. Flash the example:

```bash
tinygo flash -target=pico -scheduler tasks ./examples/nonblockwrite/main.go
```

3. Open a serial console at 115200.
4. You should see periodic status and a completion summary confirming RX interrupts and event-driven TX are functioning.

## Minimal usage pattern (single goroutine)

```go
u := uartx.UART0
_ = u.Configure(uartx.UARTConfig{ BaudRate: 115200 })

var pending []byte
buf := make([]byte, 64)

for {
    select {
    case <-u.Readable():
        if n, _ := u.Read(buf); n > 0 {
            // handle buf[:n]
        }
    case <-u.Writable():
        if len(pending) == 0 {
            pending = append(pending[:0], []byte("hello\n")...)
        }
        sent := u.SendSome(pending)    // non-blocking progress
        pending = pending[sent:]
    case <-ctx.Done():
        return
    }
}
```

## Notes

* Do not use `machine.UART0` and `uartx.UART0` together in the same programme.
* Default configuration:

  * Baud `115200` if not specified.
  * Default pins if both `TX` and `RX` are zero (`UART_TX_PIN`/`UART_RX_PIN`).
  * Frame `8N1`, FIFOs enabled (RP2040/RP2350).
* `Readable()`/`Writable()` notifications are **coalesced** (buffered capacity 1). Always re-check state after waking.
* Additional examples can be placed under `examples/`.

## API surface (summary)

```go
// Configuration (mirrors machine.UART)
func (*UART) Configure(UARTConfig) error
func (*UART) SetBaudRate(br uint32)
func (*UART) SetFormat(databits, stopbits uint8, parity UARTParity) error

// Core I/O (compatible with machine.UART)
func (*UART) Read(p []byte) (int, error)      // non-blocking
func (*UART) ReadByte() (byte, error)         // non-blocking
func (*UART) Write(p []byte) (int, error)     // blocking, drains to TX FIFO empty
func (*UART) WriteByte(b byte) error          // blocking
func (*UART) Buffered() int                   // RX bytes currently buffered

// RX helpers (blocking without polling)
func (*UART) Readable() <-chan struct{}
func (*UART) WaitReadableContext(ctx context.Context) error
func (*UART) RecvSomeContext(ctx context.Context, p []byte) (int, error)
func (*UART) RecvByteContext(ctx context.Context) (byte, error)

// TX helpers (event-driven writes)
func (*UART) Writable() <-chan struct{}
func (*UART) WaitWritableContext(ctx context.Context) error
func (*UART) SendSomeContext(ctx context.Context, p []byte) (int, error)
func (*UART) SendSome(p []byte) int // non-blocking convenience
```

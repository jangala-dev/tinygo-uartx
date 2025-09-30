# tinygo-uartx

**IN TINYGO 0.39 THIS WILL ONLY WORK WITH THE SINGLE CORE SCHEDULER** using `-scheduler tasks`

An experimental UART driver for the RP2040/RP2350 written in TinyGo.

It mirrors the public API of `machine.UART` (non-blocking `Read`, `ReadByte`; blocking `Write`, `WriteByte`) but adds extra helpers for **blocking reads** (`Readable`, `WaitReadableContext`, `RecvSomeContext`, `RecvByteContext`).

The goal is to make higher-level services (like our bridge) easier to write without rolling their own polling logic.

## Quick test on hardware (loopback)

1. Wire **UART0 TX → UART0 RX** on a Pico (e.g. GP0→GP1).
2. Flash the example:

```bash
tinygo flash -target=pico ./cmd/uartx_selftest/main.go
```

3. Open a serial console at 115200.
4. You should see the output from various tests

This confirms RX interrupts, blocking reads, and writes are working correctly.

## Notes

* On hardware, the ISR in `uartx_rp2.go` handles RX interrupts and signals blocking readers.
* Don’t use `machine.UART0` and `uartx.UART0` together in the same program.
* If you want to add more test programmes, put them under `examples/`.


Below is a concise API reference for your `uartx` package as implemented above. It reflects the RP2040/RP2350 build, with notes where behaviour differs by build tag.

## Re-exports (from `machine`)

```go
type UARTConfig = machine.UARTConfig
type Pin        = machine.Pin

const (
    NoPin        = machine.NoPin
    UART_TX_PIN  = machine.UART_TX_PIN
    UART_RX_PIN  = machine.UART_RX_PIN
    UART0_TX_PIN = machine.UART0_TX_PIN
    UART0_RX_PIN = machine.UART0_RX_PIN
    UART1_TX_PIN = machine.UART1_TX_PIN
    UART1_RX_PIN = machine.UART1_RX_PIN
)
```

## Instances

```go
var UART0 *UART
var UART1 *UART
```

Pre-initialised for the RP2 family. Interrupts are wired in `init()`; `Configure` enables them.

## Types

```go
type UART struct {
    // Public fields should be treated as implementation detail.
    // Use methods below.
}

type UARTParity uint8

const (
    ParityNone UARTParity = iota
    ParityEven
    ParityOdd
)
```

## Configuration

```go
func (u *UART) Configure(cfg UARTConfig) error
```

* Default baud: `115200` if `cfg.BaudRate==0`.
* Default pins: if both `cfg.TX` and `cfg.RX` are zero, uses `UART_TX_PIN`/`UART_RX_PIN`.
* Frame: defaults to `8N1`.
* Enables RX interrupt (RP2) and starts the internal notifier.

```go
func (u *UART) SetBaudRate(br uint32)
```

Sets baud rate (PL011 divisors on RP2).

```go
func (u *UART) SetFormat(databits, stopbits uint8, parity UARTParity) error
```

Sets frame format; typical default is `8,1,ParityNone`.

## Transmission (TX)

```go
func (u *UART) WriteByte(c byte) error
```

Blocks until the byte is transmitted (includes a flush).

```go
func (u *UART) Write(p []byte) (n int, err error)
```

Blocks until all bytes are transmitted (flush at end).

## Reception (RX) — non-blocking core

```go
func (u *UART) Read(p []byte) (n int, err error)
```

Non-blocking. Returns immediately; if no data is present, returns `(0, nil)`.

```go
func (u *UART) ReadByte() (byte, error)
```

Non-blocking. Returns an error if the RX buffer is empty. Use `Buffered()` or a blocking helper before calling.

```go
func (u *UART) Buffered() int
```

Approximate count of bytes in the RX buffer.

```go
func (u *UART) Receive(b byte)
```

Feeds the internal RX ring buffer. Called by the ISR; not for application use.

## Reception (RX) — blocking helpers (RP2040/RP2350)

```go
func (u *UART) Readable() <-chan struct{}
```

A coalesced readiness signal suitable for `select`. Notifications are edge-triggered and may be merged; always re-check `Buffered()` or attempt a read after wake.

```go
func (u *UART) WaitReadableContext(ctx context.Context) error
```

Blocks until at least one byte is available or `ctx` is done. Returns `ctx.Err()` on cancellation or deadline.

```go
func (u *UART) RecvSomeContext(ctx context.Context, p []byte) (int, error)
```

Blocks until at least one byte is available, then reads up to `len(p)` bytes. Returns `(n>0, nil)` on success, or `0, ctx.Err()` if the context completes first.

```go
func (u *UART) RecvByteContext(ctx context.Context) (byte, error)
```

Blocks for a single byte. Returns `ctx.Err()` if the context completes before data arrives.

## Semantics and guidance

* **Reads**: `Read`/`ReadByte` are non-blocking. Use `Buffered()`/`Readable()`/`WaitReadableContext`/`Recv*Context` for blocking behaviour.
* **Writes**: `Write`/`WriteByte` block until fully transmitted.
* **Notifications**: `Readable()` is coalesced (capacity 1). Do not assume one notification per byte.
* **Races**: Always re-check the buffer after any wake; missed edges are handled by design.
* **Pins**: If you supply explicit `TX`/`RX`, they must be valid for the selected UART instance.

## Minimal examples

**Blocking read of up to N bytes with a timeout**

```go
ctx, cancel := context.WithTimeout(context.Background(), 250*time.Millisecond)
defer cancel()

buf := make([]byte, 64)
n, err := uartx.UART0.RecvSomeContext(ctx, buf)
if err != nil {
    // handle ctx deadline/cancel
}
// process buf[:n]
```

**Select over readability and other work**

```go
select {
case <-uartx.UART0.Readable():
    if b, err := uartx.UART0.ReadByte(); err == nil {
        _ = uartx.UART0.WriteByte(b) // echo
    }
case <-ctx.Done():
    // shutdown path
}
```

**Standard configuration**

```go
_ = uartx.UART0.Configure(uartx.UARTConfig{
    BaudRate: 115200,
    TX:       uartx.UART0_TX_PIN,
    RX:       uartx.UART0_RX_PIN,
})
```

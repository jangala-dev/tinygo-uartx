# tinygo-uartx

An experimental UART driver for the RP2040/RP2350 written in TinyGo.

It mirrors the public API of `machine.UART` (non-blocking `Read`, `ReadByte`; blocking `Write`, `WriteByte`) but adds extra helpers for **blocking reads** (`ReadBlocking`, `ReadFullBlocking`, `ReadByteBlocking`, etc.).

The goal is to make higher-level services (like our bridge) easier to write without rolling their own polling logic.

## Repo structure

```
uartx/
  uartx_rp2.go     # real RP2040 implementation
  uartx_host.go    # host-only shim so unit tests run on a laptop
  uartx_test.go    # portable tests

examples/
  loopback/
    main.go        # simple loopback test for Pico
```

* On your laptop: `go test ./uartx` runs the host shim and checks buffer/notify logic.
* On device: flash one of the examples with `tinygo flash -target=pico ./examples/...`.

## Quick test on hardware (loopback)

1. Wire **UART0 TX → UART0 RX** on a Pico (e.g. GP0→GP1).
2. Flash the example:

```bash
tinygo flash -target=pico ./examples/loopback
```

3. Open a serial console at 115200.
4. You should see:

```
ping
echo: ping
ping
echo: ping
...
```

This confirms RX interrupts, blocking reads, and writes are working correctly.

## Notes

* On host, the shim (`uartx_host.go`) provides just enough to run unit tests.
* On hardware, the ISR in `uartx_rp2.go` handles RX interrupts and signals blocking readers.
* Don’t use `machine.UART0` and `uartx.UART0` together in the same program.
* If you want to add more test programmes, put them under `examples/`.

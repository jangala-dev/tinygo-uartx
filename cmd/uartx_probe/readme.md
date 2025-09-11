### uartx diagnostic probe

This tests the new `uartx` blocking-read implementation on RP2040/RP2350.

It does three phases:

1. **Integrity check (1 KiB)**
   Sends a deterministic 1024-byte pattern, reads it back, and compares SHA-1 hashes.
   Confirms that data passes without corruption or loss at normal throughput.

2. **Burst check (8 KiB)**
   Writes 8192 bytes in one burst while initially holding off the reader for 50 ms.
   Designed to provoke FIFO overrun or ring buffer overflow if they exist.
   Reports whether all bytes are received and shows ISR and buffer statistics.

3. **Notify sanity (2 bytes)**
   Sends “AB” and verifies that the `Readable` channel fires and both bytes are collected.
   Confirms that wake-up signalling works.

After each phase the test prints a diagnostic summary from the instrumented build: counts of ISR entries, bytes drained, ring buffer puts/drops, hardware error flags, notify activity, and selected UART registers. This lets you see exactly whether failures are due to hardware overrun, ring buffer overflow, or logic in the blocking path.

### How to run

1. **Build with debug tag**
   The visibility hooks are compiled only with the tag `uartxdebug`. Build and flash for Pico (or other RP2040 board) with:

   ```bash
   tinygo flash -target pico -tags "uartxdebug" ./cmd/uartx_probe
   ```

2. **Connect loopback**
   Link UART1's TX and RX pins on the board this time (so transmitted bytes are immediately received).

3. **Open a serial monitor**
   Open a serial terminal on the board’s USB serial port.

4. **Observe output**
   After a short countdown you will see lines such as:

   ```
   [phase] integrity-1k
    result: OK (1 KiB)
   == after integrity-1k
   ISR: count=... bytes=... maxdrain=...
   ...
   ```

5. **Interpret**

   * `ErrOverrun` > 0 means the hardware RX FIFO overflowed.
   * `RingDrops` > 0 means the software buffer overflowed.
   * `NotifyDropped` is normal (coalesced wakeups).
   * `SpuriousWakes` shows how often the reader woke up with no data available.
   * `LCRH.FEN` in the register dump tells you whether FIFOs are enabled.

This test is only for diagnosis. Run it, capture the console output, and we will know whether the 1024-byte mismatch and 8192-byte stall are due to hardware overrun, ring buffer size, or notify logic.
//go:build rp2040 || rp2350
// +build rp2040 rp2350

package main

import (
	"device/rp"
	"time"

	"github.com/jangala-dev/tinygo-uartx/uartx"
)

func readFEN(u *rp.UART0_Type) bool {
	const FEN = rp.UART0_UARTLCR_H_FEN
	return u.UARTLCR_H.Get()&FEN != 0
}

func readCR(u *rp.UART0_Type) uint32    { return u.UARTCR.Get() }
func readLCR_H(u *rp.UART0_Type) uint32 { return u.UARTLCR_H.Get() }
func readFR(u *rp.UART0_Type) uint32    { return u.UARTFR.Get() }
func readIBRD(u *rp.UART0_Type) uint32  { return u.UARTIBRD.Get() }
func readFBRD(u *rp.UART0_Type) uint32  { return u.UARTFBRD.Get() }

func main() {
	time.Sleep(2 * time.Second)

	u := rp.UART0

	println("Before configure:")
	report(u)

	// Configure UART0 with TinyGo defaults (no FIFO manipulation here).
	uartx.UART0.Configure(uartx.UARTConfig{
		BaudRate: 115200,
		// Leave TX/RX as machine defaults (GPIO0/1 on Pico) unless overridden.
	})

	println("After Configure():")
	report(u)

	// Optional: demonstrate enabling FEN (read-modify-write of LCR_H).
	// NOTE: Per RP2040 docs, LCR writes are ideally done before UARTEN is set.
	// This snippet is only to verify behaviour quickly.
	const FEN = rp.UART0_UARTLCR_H_FEN
	u.UARTLCR_H.Set(u.UARTLCR_H.Get() | FEN)

	println("After setting LCR_H.FEN bit (for verification):")
	report(u)

	for {
		time.Sleep(time.Second)
	}
}

func report(u *rp.UART0_Type) {
	println("-----------------------------")
	print("UARTCR   = 0x")
	printlnHex(readCR(u))
	print("UARTLCR_H= 0x")
	printlnHex(readLCR_H(u))
	print("UARTFR   = 0x")
	printlnHex(readFR(u))
	print("UARTIBRD = 0x")
	printlnHex(readIBRD(u))
	print("UARTFBRD = 0x")
	printlnHex(readFBRD(u))
	print("FIFOs enabled (FEN)= ")
	println(readFEN(u))
}

func printlnHex(v uint32) {
	const hexdigits = "0123456789abcdef"
	var b [8]byte
	for i := 0; i < 8; i++ {
		shift := uint(28 - 4*i)
		b[i] = hexdigits[(v>>shift)&0xF]
	}
	println(string(b[:]))
}

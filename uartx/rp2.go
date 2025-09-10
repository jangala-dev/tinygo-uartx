// uartx/machine_rp2.go

//go:build rp2040 || rp2350

package uartx

import (
	"device/rp"
	"machine"
	"runtime/interrupt"
)

const deviceName = rp.Device

// UART on the RP2040
var (
	UART0  = &_UART0
	_UART0 = UART{
		Buffer: machine.NewRingBuffer(),
		Bus:    rp.UART0,
		notify: make(chan struct{}, 1),
	}

	UART1  = &_UART1
	_UART1 = UART{
		Buffer: machine.NewRingBuffer(),
		Bus:    rp.UART1,
		notify: make(chan struct{}, 1),
	}
)

func init() {
	UART0.Interrupt = interrupt.New(rp.IRQ_UART0_IRQ, _UART0.handleInterrupt)
	UART1.Interrupt = interrupt.New(rp.IRQ_UART1_IRQ, _UART1.handleInterrupt)
}

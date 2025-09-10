// uartx/export.go

package uartx

import "machine"

type UARTConfig = machine.UARTConfig
type Pin = machine.Pin

const (
	NoPin       = machine.NoPin
	UART_TX_PIN = machine.UART_TX_PIN
	UART_RX_PIN = machine.UART_RX_PIN
)

//go:build !uartxdebug

package uartx

type Stats struct{}

func (u *UART) DebugReset()       {}
func (u *UART) DebugStats() Stats { return Stats{} }

type Regs struct{}

func (u *UART) DebugRegs() Regs { return Regs{} }

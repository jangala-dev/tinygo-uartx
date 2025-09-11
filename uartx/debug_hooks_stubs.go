//go:build !uartxdebug

package uartx

func (u *UART) dbgISR(int)             {}
func (u *UART) dbgOnByte(uint32, bool) {}
func (u *UART) dbgNotify(bool)         {}
func (u *UART) dbgReadWait()           {}
func (u *UART) dbgSpuriousWake()       {}
func (u *UART) dbgTimeout()            {}

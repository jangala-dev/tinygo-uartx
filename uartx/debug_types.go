//go:build (rp2040 || rp2350) && uartxdebug

package uartx

import "sync/atomic"

// Stats holds counters since the last reset.
type Stats struct {
	// ISR-level
	ISRCount      uint32 // number of ISR entries
	ISRBytes      uint32 // total bytes drained in ISRs
	ISRMaxDrain   uint32 // max bytes drained in a single ISR
	NotifySent    uint32 // notify channel sends that succeeded
	NotifyDropped uint32 // notify channel sends that were dropped (buffer full)

	// Per-byte error flags from UARTDR
	ErrOverrun uint32 // OE
	ErrBreak   uint32 // BE
	ErrParity  uint32 // PE
	ErrFraming uint32 // FE

	// Ring buffer
	RingPuts    uint32 // successful Put()s
	RingDrops   uint32 // failed Put()s (overflow)
	RingMaxUsed uint32 // high-water mark of ring occupancy

	// Blocking API behaviour
	ReadWaits     uint32 // times Recv* had to wait
	SpuriousWakes uint32 // notify received but no data available
	Timeouts      uint32 // context timeouts in Recv* APIs
}

func (u *UART) DebugReset() {
	// Zero the struct by reassigning (safe as Stats is POD)
	u.stats = Stats{}
}

func (u *UART) DebugStats() Stats {
	// Return a copy to avoid races; 32-bit atomic reads are fine on Cortex-M0+
	return Stats{
		ISRCount:      atomic.LoadUint32(&u.stats.ISRCount),
		ISRBytes:      atomic.LoadUint32(&u.stats.ISRBytes),
		ISRMaxDrain:   atomic.LoadUint32(&u.stats.ISRMaxDrain),
		NotifySent:    atomic.LoadUint32(&u.stats.NotifySent),
		NotifyDropped: atomic.LoadUint32(&u.stats.NotifyDropped),

		ErrOverrun: atomic.LoadUint32(&u.stats.ErrOverrun),
		ErrBreak:   atomic.LoadUint32(&u.stats.ErrBreak),
		ErrParity:  atomic.LoadUint32(&u.stats.ErrParity),
		ErrFraming: atomic.LoadUint32(&u.stats.ErrFraming),

		RingPuts:    atomic.LoadUint32(&u.stats.RingPuts),
		RingDrops:   atomic.LoadUint32(&u.stats.RingDrops),
		RingMaxUsed: atomic.LoadUint32(&u.stats.RingMaxUsed),

		ReadWaits:     atomic.LoadUint32(&u.stats.ReadWaits),
		SpuriousWakes: atomic.LoadUint32(&u.stats.SpuriousWakes),
		Timeouts:      atomic.LoadUint32(&u.stats.Timeouts),
	}
}

// Snapshot of useful HW registers for RP2 PL011.
type Regs struct {
	FR   uint32 // Flag register
	CR   uint32 // Control
	LCRH uint32 // Line control
	IFLS uint32 // FIFO interrupt level select
	IMSC uint32 // Interrupt mask set/clear
	MIS  uint32 // Masked interrupt status
	RIS  uint32 // Raw interrupt status
	IBRD uint32
	FBRD uint32
}

func (u *UART) DebugRegs() Regs {
	return Regs{
		FR:   u.Bus.UARTFR.Get(),
		CR:   u.Bus.UARTCR.Get(),
		LCRH: u.Bus.UARTLCR_H.Get(),
		IFLS: u.Bus.UARTIFLS.Get(),
		IMSC: u.Bus.UARTIMSC.Get(),
		MIS:  u.Bus.UARTMIS.Get(),
		RIS:  u.Bus.UARTRIS.Get(),
		IBRD: u.Bus.UARTIBRD.Get(),
		FBRD: u.Bus.UARTFBRD.Get(),
	}
}

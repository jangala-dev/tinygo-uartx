//go:build (rp2040 || rp2350) && uartxdebug

package uartx

import (
	"device/rp"
	"sync/atomic"
)

// Called at ISR entry and exit.
func (u *UART) dbgISR(bytesDrained int) {
	atomic.AddUint32(&u.stats.ISRCount, 1)
	atomic.AddUint32(&u.stats.ISRBytes, uint32(bytesDrained))
	for {
		max := atomic.LoadUint32(&u.stats.ISRMaxDrain)
		if uint32(bytesDrained) <= max {
			break
		}
		if atomic.CompareAndSwapUint32(&u.stats.ISRMaxDrain, max, uint32(bytesDrained)) {
			break
		}
	}
}

// Called per drained byte with the raw DR value and the Put() outcome.
func (u *UART) dbgOnByte(dr uint32, putOK bool) {
	if (dr & rp.UART0_UARTDR_OE) != 0 {
		atomic.AddUint32(&u.stats.ErrOverrun, 1)
	}
	if (dr & rp.UART0_UARTDR_BE) != 0 {
		atomic.AddUint32(&u.stats.ErrBreak, 1)
	}
	if (dr & rp.UART0_UARTDR_PE) != 0 {
		atomic.AddUint32(&u.stats.ErrParity, 1)
	}
	if (dr & rp.UART0_UARTDR_FE) != 0 {
		atomic.AddUint32(&u.stats.ErrFraming, 1)
	}
	if putOK {
		atomic.AddUint32(&u.stats.RingPuts, 1)
		// track high-water mark
		used := uint32(u.Buffer.Used())
		for {
			max := atomic.LoadUint32(&u.stats.RingMaxUsed)
			if used <= max {
				break
			}
			if atomic.CompareAndSwapUint32(&u.stats.RingMaxUsed, max, used) {
				break
			}
		}
	} else {
		atomic.AddUint32(&u.stats.RingDrops, 1)
	}
}

func (u *UART) dbgNotify(sent bool) {
	if sent {
		atomic.AddUint32(&u.stats.NotifySent, 1)
	} else {
		atomic.AddUint32(&u.stats.NotifyDropped, 1)
	}
}

func (u *UART) dbgReadWait() {
	atomic.AddUint32(&u.stats.ReadWaits, 1)
}
func (u *UART) dbgSpuriousWake() {
	atomic.AddUint32(&u.stats.SpuriousWakes, 1)
}
func (u *UART) dbgTimeout() {
	atomic.AddUint32(&u.stats.Timeouts, 1)
}

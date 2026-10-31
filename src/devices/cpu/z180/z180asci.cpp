// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic
/*********************************************************************

    z180asci.cpp

*********************************************************************/

#include "emu.h"
#include "z180.h"

//#define VERBOSE 1

#include "logmacro.h"

/* 00 ASCI control register A ch 0 */
static constexpr u8 Z180_CNTLA_MPE         = 0x80;
static constexpr u8 Z180_CNTLA_RE          = 0x40;
static constexpr u8 Z180_CNTLA_TE          = 0x20;
static constexpr u8 Z180_CNTLA_RTS0        = 0x10;
static constexpr u8 Z180_CNTLA_MPBR_EFR    = 0x08;
static constexpr u8 Z180_CNTLA_MODE        = 0x07;
static constexpr u8 Z180_CNTLA_MODE_DATA   = 0x04;
static constexpr u8 Z180_CNTLA_MODE_PARITY = 0x02;
static constexpr u8 Z180_CNTLA_MODE_STOPB  = 0x01;

/* 01 ASCI control register A ch 1 */
static constexpr u8 Z180_CNTLA1_CKA1D      = 0x10;

/* 02/03 ASCI control register B ch 0 */
static constexpr u8 Z180_CNTLB_MPBT        = 0x80;
static constexpr u8 Z180_CNTLB_MP          = 0x40;
static constexpr u8 Z180_CNTLB_CTS_PS      = 0x20;
static constexpr u8 Z180_CNTLB_PEO         = 0x10;
static constexpr u8 Z180_CNTLB_DR          = 0x08;
static constexpr u8 Z180_CNTLB_SS          = 0x07;

/* 04 ASCI status register 0 (all bits read-only except RIE and TIE) */
static constexpr u8 Z180_STAT_RDRF         = 0x80;
static constexpr u8 Z180_STAT_OVRN         = 0x40;
static constexpr u8 Z180_STAT_PE           = 0x20;
static constexpr u8 Z180_STAT_FE           = 0x10;
static constexpr u8 Z180_STAT_RIE          = 0x08;
static constexpr u8 Z180_STAT_DCD0         = 0x04;
static constexpr u8 Z180_STAT_TDRE         = 0x02;
static constexpr u8 Z180_STAT_TIE          = 0x01;

/* 05 ASCI status register 1 (all bits read-only except RIE, CTS1E and TIE) */
static constexpr u8 Z180_STAT1_CTS1E       = 0x04;

/* 12/13 (Z8S180/Z8L180) ASCI extension control register 0 (break detect is read-only) */
static constexpr u8 Z180_ASEXT_DCD0        = 0x40;
static constexpr u8 Z180_ASEXT_CTS0        = 0x20;
static constexpr u8 Z180_ASEXT_X1_BIT_CLK0 = 0x10;
static constexpr u8 Z180_ASEXT_BRG0_MODE   = 0x08;
static constexpr u8 Z180_ASEXT_BRK_EN      = 0x04;
static constexpr u8 Z180_ASEXT_BRK_DET     = 0x02;
static constexpr u8 Z180_ASEXT_BRK_SEND    = 0x01;

static constexpr u8 Z180_ASEXT0_MASK = 0xff;
static constexpr u8 Z180_ASEXT1_MASK = 0x9f;

//**************************************************************************
//  z180asci_channel_base
//**************************************************************************

z180asci_channel_base::z180asci_channel_base(
	const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, const int id, const bool ext)
	: device_t(mconfig, type, tag, owner, clock)
	, m_rcv_clock(nullptr)
	, m_tra_clock(nullptr)
	, m_bit_rate(attotime::never)
	, m_sample_rate(attotime::never)
	, m_txa_handler(*this)
	, m_rts_handler(*this)
	, m_cka_handler(*this)
	, m_cts(0)
	, m_dcd(0)
	, m_irq(0)
	, m_rxa(1)
	, m_rts(0)
	, m_divisor(0)
	, m_id(id)
	, m_ext(ext)
{
}


void z180asci_channel_base::device_start()
{
	save_item(NAME(m_asci_cntla));
	save_item(NAME(m_asci_cntlb));
	save_item(NAME(m_asci_stat));
	save_item(NAME(m_asci_tdr));
	save_item(NAME(m_asci_rdr));
	if (m_ext)
	{
		save_item(NAME(m_asci_ext));
		save_item(NAME(m_asci_tc.w));
	}
	save_item(NAME(m_tdr_empty));
	save_item(NAME(m_tsr));
	save_item(NAME(m_rsr));
	save_item(NAME(m_data_fifo));
	save_item(NAME(m_error_fifo));
	save_item(NAME(m_fifo_wr));
	save_item(NAME(m_fifo_rd));
	save_item(NAME(m_fifo_count));
	save_item(NAME(m_fifo_overrun));

	save_item(NAME(m_cts));
	save_item(NAME(m_dcd));
	save_item(NAME(m_irq));
	save_item(NAME(m_txa));
	save_item(NAME(m_rxa));
	save_item(NAME(m_rts));

	save_item(NAME(m_divisor));

	save_item(NAME(m_clock_state));

	save_item(NAME(m_tx_counter));
	save_item(NAME(m_tx_state));

	save_item(NAME(m_rx_state));
	save_item(NAME(m_rx_bits));
	save_item(NAME(m_rx_counter));
	save_item(NAME(m_rx_count_to));
	save_item(NAME(m_rx_total_bits));
	save_item(NAME(m_rx_enabled));

	save_item(NAME(m_bit_rate));
	save_item(NAME(m_sample_rate));
	save_item(NAME(m_receive_epoch));
	save_item(NAME(m_receive_parked));
	save_item(NAME(m_iostop));

	m_rcv_clock = timer_alloc(FUNC(z180asci_channel_base::rcv_clock), this);
	m_tra_clock = timer_alloc(FUNC(z180asci_channel_base::tra_clock), this);
}

void z180asci_channel_base::device_reset()
{
	m_iostop = false;
	m_asci_ext = 0;
	m_asci_tc.w = 0;

	m_fifo_wr = 0;
	m_fifo_rd = 0;
	m_fifo_count = 0;
	m_fifo_overrun = false;
	m_irq = 0;

	output_txa(1);
	output_rts(1);

	m_tsr = 0;
	m_tdr_empty = true;
	m_rxa = 1;
	m_clock_state = 0;
	m_tx_state = STATE_WAIT;
	m_rx_state = STATE_START;
	m_rx_bits = 0;
	m_rx_enabled = true;

	m_tx_counter = 0;
	m_rx_counter = 0;
	m_rx_count_to = 1;
	// Reset leaves the receive timer's phase intact. If it was parked, keep
	// its saved epoch so a later RXA transition can restart that same grid.
}

void z180asci_channel_base::device_clock_changed()
{
	uint32_t brg_divisor;
	 // Divide ratio
	m_divisor = (m_asci_ext & Z180_ASEXT_X1_BIT_CLK0) ? 1 : ((m_asci_cntlb & Z180_CNTLB_DR) ? 64 : 16);

	if ((m_asci_cntlb & Z180_CNTLB_SS) == Z180_CNTLB_SS)
	{
		// External clock
		brg_divisor = 0;
	}
	else
	{
		if (m_asci_ext & Z180_ASEXT_BRG0_MODE)
		{
			// UM0050 p144: BRG = PHI / (2 * (TC + 2)).
			brg_divisor = 2 * (u32(m_asci_tc.w) + 2);
		}
		else
		{
			// Regular bitrate generator mode
			brg_divisor = 1 << (m_asci_cntlb & Z180_CNTLB_SS);
			brg_divisor *= ((m_asci_cntlb & Z180_CNTLB_CTS_PS) ? 30 : 10); // Prescale
		}
	}

	if (brg_divisor)
	{
		LOG("Z180 ASCI%d set bitrate %d\n", m_id, uint32_t(clock() / brg_divisor / m_divisor));
		m_bit_rate = attotime::from_ticks(u64(brg_divisor) * m_divisor, clock());
		m_sample_rate = attotime::from_ticks(brg_divisor, clock());
	}
	else
	{
		LOG("Z180 ASCI%d set bitrate 0, using external\n", m_id);
		m_bit_rate = attotime::never;
		m_sample_rate = attotime::never;
	}

	m_tra_clock->adjust(attotime::never);
	m_rx_state = STATE_START;
	m_rx_count_to = 1;
	m_receive_parked = false;
	if (m_iostop)
	{
		m_receive_epoch = machine().time();
		m_receive_parked = true;
		m_rcv_clock->adjust(attotime::never);
		return;
	}
	if(!m_sample_rate.is_never())
	{
		m_rcv_clock->adjust(m_sample_rate, 0, m_sample_rate);
		if (m_rxa)
			park_receive_clock();
	}
}

void z180asci_channel_base::park_receive_clock()
{
	if (!m_idle_receive_clock_gating || m_sample_rate.seconds() || !m_sample_rate.attoseconds())
		return;
	// In STATE_START, sampling a high RXA has no observable effect, even
	// with RE enabled. Keep the divider phase and wake on the next low edge.
	// This also avoids millions of no-op timer callbacks on an unwired RXA.
	m_receive_epoch = machine().time();
	m_receive_parked = true;
	m_rcv_clock->adjust(attotime::never);
}

TIMER_CALLBACK_MEMBER(z180asci_channel_base::rcv_clock)
{
	receive_edge();
	if (m_rx_state == STATE_START && m_rxa)
		park_receive_clock();
}

uint8_t z180asci_channel_base::cntla_r()
{
	LOG("Z180 CNTLA%d rd $%02x\n", m_id, m_asci_cntla);
	return m_asci_cntla;
}

uint8_t z180asci_channel_base::cntlb_r()
{
	uint8_t data = (m_asci_cntlb & 0xdf) | (m_cts << 5);
	LOG("Z180 CNTLB%d rd $%02x\n", m_id, data);
	return data;
}

uint8_t z180asci_channel_base::stat_r()
{
	u8 const data = m_asci_stat;
	// UM0050 pp138-139: the first read after DCD goes low still returns
	// DCD=1, then clears the latch and permits auto-enabled reception.
	if (!m_id && !m_dcd && !machine().side_effects_disabled())
	{
		m_asci_stat &= ~Z180_STAT_DCD0;
		m_rx_enabled = true;
	}
	return data;
}

uint8_t z180asci_channel_base::tdr_r()
{
	LOG("Z180 TDR%d   rd $%02x\n", m_id, m_asci_tdr);
	return m_asci_tdr;
}

uint8_t z180asci_channel_base::rdr_r()
{
	u8 const data = m_fifo_count ? m_data_fifo[m_fifo_rd] : m_asci_rdr;
	if (!machine().side_effects_disabled() && m_fifo_count)
	{
		m_asci_rdr = data;
		m_fifo_rd = (m_fifo_rd + 1) & 3;
		--m_fifo_count;
		update_fifo_status();
	}
	return data;
}

uint8_t z180asci_channel_base::asext_r()
{
	LOG("Z180 ASEXT%d rd $%02x\n", m_id, m_asci_ext);
	return m_asci_ext;
}

uint8_t z180asci_channel_base::astcl_r()
{
	LOG("Z180 ASTC%dL rd $%02x ($%04x)\n", m_id, m_asci_tc.b.l, m_asci_tc.w);
	return m_asci_tc.b.l;
}

uint8_t z180asci_channel_base::astch_r()
{
	LOG("Z180 ASTC%dH rd $%02x ($%04x)\n", m_id, m_asci_tc.b.h, m_asci_tc.w);
	return m_asci_tc.b.h;
}

void z180asci_channel_base::update_total_bits()
{
	m_rx_total_bits = (m_asci_cntla & Z180_CNTLA_MODE_DATA) ? 8 : 7;
	m_rx_total_bits += (m_asci_cntla & Z180_CNTLA_MODE_STOPB) ? 2 : 1;
	m_rx_total_bits += (m_asci_cntlb & Z180_CNTLB_MP) ? 1 : ((m_asci_cntla & Z180_CNTLA_MODE_PARITY) ? 1 : 0);
}

void z180asci_channel_base::cntla_w(uint8_t data)
{
	LOG("Z180 CNTLA%d wr $%02x\n", m_id, data);
	if (m_iostop)
		data &= ~(Z180_CNTLA_RE | Z180_CNTLA_TE);
	m_asci_cntla = (data & ~Z180_CNTLA_MPBR_EFR) | (m_asci_cntla & Z180_CNTLA_MPBR_EFR);
	output_rts(BIT(data,4)); // Z180_CNTLA_RTS0
	if (!(data & Z180_CNTLA_MPBR_EFR)) // Error Flag Reset is write-zero.
	{
		// UM0050 pp126/129, OT001400 precaution 3 and TN0053 p3 agree:
		// zero clears errors; DS006002 p44's write-one text is erroneous.
		// Reset errors and the enhanced receive hold without discarding data.
		m_fifo_overrun = false;
		if (m_fifo_count)
			m_error_fifo[m_fifo_rd] = 0;
		m_asci_stat &= ~(Z180_STAT_OVRN | Z180_STAT_PE | Z180_STAT_FE);
		if (m_ext)
			m_asci_ext &= ~(Z180_ASEXT_BRK_DET);
	}
	if (!(data & Z180_CNTLA_TE))
	{
		m_tx_state = STATE_WAIT;
		m_tra_clock->adjust(attotime::never);
		if (!m_iostop)
			output_txa(1); // Table56: TXA holds its IOSTOP level.
	}
	else if (!m_tdr_empty && !m_bit_rate.is_never() && m_tra_clock->expire().is_never())
		m_tra_clock->adjust(m_bit_rate, 0, m_bit_rate);
	update_total_bits();
}

void z180asci_channel_base::cntlb_w(uint8_t data)
{
	LOG("Z180 CNTLB%d wr $%02x\n", m_id, data);
	m_asci_cntlb = data;
	device_clock_changed();
	update_total_bits();
}

void z180asci_channel_base::tdr_w(uint8_t data)
{
	LOG("Z180 TDR%d   wr $%02x\n", m_id, data);
	m_asci_tdr = data;
	m_tdr_empty = m_iostop;
	update_tdre();

	if ((m_asci_cntla & Z180_CNTLA_TE) && !m_bit_rate.is_never() && m_tra_clock->expire().is_never())
		m_tra_clock->adjust(m_bit_rate, 0, m_bit_rate);
}

void z180asci_channel_base::rdr_w(uint8_t data)
{
	LOG("Z180 RDR%d   wr $%02x\n", m_id, data);
	if (!(m_asci_stat & Z180_STAT_RDRF))
		set_fifo_data(data, 0);
}

void z180asci_channel_base::asext_w(uint8_t data)
{
	if (m_asci_ext & Z180_ASEXT_BRK_EN)
		m_tx_state = (m_asci_ext & Z180_ASEXT_BRK_SEND) ? STATE_BREAK : STATE_WAIT;
	device_clock_changed();
}

void z180asci_channel_base::astcl_w(uint8_t data)
{
	LOG("Z180 ASTC%dL wr $%02x\n", m_id, data);
	m_asci_tc.b.l = data;
	device_clock_changed();
}

void z180asci_channel_base::astch_w(uint8_t data)
{
	LOG("Z180 ASTC%dH wr $%02x\n", m_id, data);
	m_asci_tc.b.h = data;
	device_clock_changed();
}

void z180asci_channel_base::update_tdre()
{
	// UM0050 pp138-139: CTS masks TDRE, not the physical TDR state
	// or an ongoing transmission. CTS1 applies only when selected.
	bool const auto_cts = m_id ? (m_asci_stat & Z180_STAT1_CTS1E) : (!m_ext || !(m_asci_ext & Z180_ASEXT_CTS0));
	if (m_iostop || (m_tdr_empty && !(auto_cts && m_cts)))
		m_asci_stat |= Z180_STAT_TDRE;
	else
		m_asci_stat &= ~Z180_STAT_TDRE;
}

void z180asci_channel_base::cts_wr(int state)
{
	m_cts = state;
	update_tdre();
}

void z180asci_channel_base::dcd_wr(int state)
{
	if (m_id)
		return;
	m_dcd = state;
	if (m_iostop)
		return;
	if (state)
	{
		m_asci_stat |= Z180_STAT_DCD0;
		if (!m_ext || !(m_asci_ext & Z180_ASEXT_DCD0))
		{
			m_rx_enabled = false;
			m_rx_state = STATE_START;
			m_fifo_count = m_fifo_wr = m_fifo_rd = 0;
			m_fifo_overrun = false;
			m_asci_stat &= ~0xf0;
			m_asci_ext &= ~Z180_ASEXT_BRK_DET;
		}
	}
}

void z180asci_channel_base::resume_receive_clock()
{
	if (m_sample_rate.is_never())
	{
		m_receive_parked = false;
		return;
	}
	// Exact integer phase modulo the timer's rounded attosecond period.
	// Reduce the seconds separately so long idle intervals cannot overflow.
	const attotime elapsed = machine().time() - m_receive_epoch;
	if (m_sample_rate.seconds())
	{
		// IOSTOP also parks low-clock receivers whose sample period is
		// one second or longer. Keep the full period: its attoseconds
		// field alone can be zero, or omit one or more whole seconds.
		attotime remainder = elapsed;
		attotime multiple = m_sample_rate;
		while (multiple <= remainder - multiple)
			multiple *= 2;
		while (multiple >= m_sample_rate)
		{
			if (remainder >= multiple)
				remainder -= multiple;
			multiple /= 2;
		}
		m_receive_parked = false;
		m_rcv_clock->adjust(m_sample_rate - remainder, 0, m_sample_rate);
		return;
	}
	const u64 period = m_sample_rate.attoseconds();
	u64 remainder = elapsed.attoseconds() % period;
	u64 seconds = elapsed.seconds();
	u64 add = ATTOSECONDS_PER_SECOND % period;
	while (seconds)
	{
		if (seconds & 1)
			remainder = (remainder + add) % period;
		add = (add + add) % period;
		seconds >>= 1;
	}
	m_receive_parked = false;
	m_rcv_clock->adjust(attotime(0, period - remainder), 0, m_sample_rate);
}

void z180asci_channel_base::iostop_w(bool stopped)
{
	if (stopped == m_iostop)
		return;
	m_iostop = stopped;
	if (!stopped)
	{
		if (m_receive_parked)
			resume_receive_clock();
		return;
	}
	// UM0050 pp120-129 and Table56: clear functional receive/transmit
	// state, retain format/enables/data registers, and HOLD TXA/RTS.
	if (!m_receive_parked && !m_sample_rate.is_never())
		m_receive_epoch = m_rcv_clock->expire() - m_sample_rate;
	m_receive_parked = true;
	m_rcv_clock->adjust(attotime::never);
	m_tra_clock->adjust(attotime::never);
	m_asci_cntla &= ~(Z180_CNTLA_RE | Z180_CNTLA_TE);
	m_asci_stat = (m_asci_stat & 0x0d) | Z180_STAT_TDRE;
	m_fifo_count = m_fifo_wr = m_fifo_rd = 0;
	m_fifo_overrun = false;
	m_tdr_empty = true;
	m_tx_state = STATE_WAIT;
	m_rx_state = STATE_START;
	m_rx_counter = m_tx_counter = m_rx_bits = 0;
	m_rx_count_to = 1;
}

void z180asci_channel_base::rxa_wr(int state)
{
	m_rxa = state;
	if (!state && m_receive_parked && !m_iostop)
		resume_receive_clock();
}

void z180asci_channel_base::cka_wr(int state)
{
	if (m_iostop)
	{
		m_clock_state = state;
		return;
	}
	// For channel 1, CKA can be disabled
	if (m_id && (m_asci_cntla & Z180_CNTLA1_CKA1D)) return;

	if(state != m_clock_state)
	{
		m_clock_state = state;
		if(!state)
		{
			m_tx_counter++;
			if (m_tx_counter != m_divisor) return;
			m_tx_counter = 0;
			if (m_asci_cntla & Z180_CNTLA_TE)
				transmit_edge();
		}
		else
		{
			m_rx_counter++;
			if (m_rx_counter != m_rx_count_to) return;
			m_rx_counter = 0;
			if (m_rx_enabled && (m_asci_cntla & Z180_CNTLA_RE))
				receive_edge();
		}
	}
}

void z180asci_channel_base::prepare_tsr()
{
	int bits = (m_asci_cntla & Z180_CNTLA_MODE_DATA) ? 8 : 7;

	m_tsr = (m_asci_cntla & Z180_CNTLA_MODE_STOPB) ? 3 : 1; // stop bit(s)

	if ((m_asci_cntlb & Z180_CNTLB_MP) || (m_asci_cntla & Z180_CNTLA_MODE_PARITY))
	{
		m_tsr <<= 1;
		if (m_asci_cntlb & Z180_CNTLB_MP)
		{
			m_tsr |= (m_asci_cntlb & Z180_CNTLB_MPBT) ? 1 : 0;
		}
		else
		{
			uint8_t parity = 0;
			for (int i = 0; i < bits; i++)
				parity ^= BIT(m_asci_tdr, i);
			if (m_asci_cntlb & Z180_CNTLB_PEO) parity ^= 1; // odd parity
			m_tsr |= parity;
		}
	}
	m_tsr <<= bits;
	m_tsr |= m_asci_tdr & ((1U << bits) - 1); // UM0050 Table 17: 7 or 8 data bits
	m_tsr <<= 1; // start bit
	m_tdr_empty = true; // TDR -> TSR releases the buffer (UM0050 p121).
	update_tdre();
}

void z180asci_channel_base::transmit_edge()
{
	if (!(m_asci_cntla & Z180_CNTLA_TE))
		return;
	switch (m_tx_state)
	{
	case STATE_WAIT:
		if (m_tdr_empty)
		{
			m_tra_clock->adjust(attotime::never);
			return;
		}
		prepare_tsr();
		m_tx_state = STATE_DATA;
		[[fallthrough]];
	case STATE_DATA:
		output_txa(BIT(m_tsr, 0));
		m_tsr >>= 1;
		if (!m_tsr)
			m_tx_state = STATE_WAIT;
		break;
	case STATE_BREAK:
		output_txa(0);
		break;
	}
}

void z180asci_channel_base::update_received()
{
	uint8_t rx_error = 0;
	int data_bits = (m_asci_cntla & Z180_CNTLA_MODE_DATA) ? 8 : 7;
	if (m_rsr == 0) // Break detect
	{
		rx_error |= Z180_ASEXT_BRK_DET;
	}
	if (m_rxa == 0)
		rx_error |= Z180_STAT_FE;

	if (m_asci_cntlb & Z180_CNTLB_MP)
	{
		m_asci_cntla = (m_asci_cntla & ~Z180_CNTLA_MPBR_EFR) | (((m_rsr >> data_bits) & 1) ? Z180_CNTLA_MPBR_EFR : 0);
	}
	else if (m_asci_cntla & Z180_CNTLA_MODE_PARITY)
	{
		uint8_t parity = 0;
		for (int i = 0; i < data_bits; i++)
			parity ^= BIT(m_rsr, i);
		if (m_asci_cntlb & Z180_CNTLB_PEO) parity ^= 1; // odd parity
		if (((m_rsr >> data_bits) & 1) != parity)
			rx_error |= Z180_STAT_PE;
	}
	// Skip only if MPE mode active and MPB is 0
	if (!((m_asci_cntlb & Z180_CNTLB_MP) && (m_asci_cntla & Z180_CNTLA_MPE) && ((m_asci_cntla & Z180_CNTLA_MPBR_EFR) == 0)))
	{
		LOG("%s: %X received%s%s\n", machine().time().to_string(), m_rsr, (rx_error & Z180_STAT_FE) ? " (framing error)" : "", (rx_error & Z180_STAT_PE) ? " (bad parity)" : "");
		set_fifo_data(m_rsr & ((1U << data_bits) - 1), rx_error);
	}
}

void z180asci_channel_base::receive_edge()
{
	if (!m_rx_enabled) return;

	if (m_asci_cntla & Z180_CNTLA_RE)
	{
		switch (m_rx_state)
		{
		case STATE_START:
			if(m_rxa == 0)
			{
				m_rx_state = STATE_DATA;
				m_rx_bits = 0;
				m_rsr = 0;
				if(!(m_bit_rate.is_never()))
					m_rcv_clock->adjust((m_divisor == 1) ? m_bit_rate : (m_bit_rate * 3) / 2, 0, m_bit_rate);
				else
					m_rx_count_to = (m_divisor == 1) ? m_divisor : (m_divisor * 3) / 2;
			}
			break;
		case STATE_DATA:
			m_rsr |= m_rxa << m_rx_bits;
			m_rx_bits++;
			m_rx_count_to = m_divisor;
			if (m_rx_bits == m_rx_total_bits)
			{
				update_received();
				m_rx_state = STATE_START;
				if(!m_sample_rate.is_never())
					m_rcv_clock->adjust(m_sample_rate, 0, m_sample_rate);
				else
					m_rx_count_to = 1;
			}
			break;
		}
	}
}

// Z8S180 DS006002 pp41,44: four data/status entries. Errors belong
// to the oldest byte; overflow marks the most recent successfully stored byte.
void z180asci_channel_base::update_fifo_status()
{
	if (m_fifo_count)
	{
		// Original receive-register errors remain latched until EFR, DCD,
		// IOSTOP or reset (UM0050 pp120-123). Enhanced FIFO status instead
		// follows the oldest queued character (DS006002 pp41/44).
		m_asci_stat = (m_asci_stat & (m_ext ? ~0x70 : 0xff)) | Z180_STAT_RDRF | (m_error_fifo[m_fifo_rd] & 0x70);
		if (m_ext)
			m_asci_ext = (m_asci_ext & ~Z180_ASEXT_BRK_DET) | (m_error_fifo[m_fifo_rd] & Z180_ASEXT_BRK_DET);
	}
	else
		m_asci_stat &= ~Z180_STAT_RDRF;
	if ((m_asci_stat & Z180_STAT_RIE) && (m_asci_stat & 0xf0))
		m_irq = 1;
}

void z180asci_channel_base::set_fifo_data(uint8_t data, uint8_t error)
{
	// TN0053 p3: only the later N revision needs EFR to release its
	// overrun receive hold. Original K can receive again after RDR is read.
	if (m_ext && m_fifo_overrun)
		return;
	if (m_fifo_count == (m_ext ? 4 : 1))
	{
		m_error_fifo[(m_fifo_wr + 3) & 3] |= Z180_STAT_OVRN;
		m_fifo_overrun = true;
	}
	else
	{
		m_data_fifo[m_fifo_wr] = data;
		m_error_fifo[m_fifo_wr] = error;
		m_fifo_wr = (m_fifo_wr + 1) & 3;
		++m_fifo_count;
	}
	update_fifo_status();
}

void z180asci_channel_base::output_txa(int txa)
{
	if (m_txa != txa)
	{
		m_txa = txa;
		m_txa_handler(m_txa);
	}
}

void z180asci_channel_base::output_rts(int rts)
{
	if (m_rts != rts)
	{
		m_rts = rts;
		m_rts_handler(m_rts);
	}
}

//**************************************************************************
//  z180asci_channel_0
//**************************************************************************

z180asci_channel_0::z180asci_channel_0(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, const bool ext)
	: z180asci_channel_base(mconfig, type, tag, owner, clock, 0, ext)
{
}

z180asci_channel_0::z180asci_channel_0(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z180asci_channel_0(mconfig, Z180ASCI_CHANNEL_0, tag, owner, clock, false)
{
}

void z180asci_channel_0::device_reset()
{
	z180asci_channel_base::device_reset();
	cntla_w((m_asci_cntla & Z180_CNTLA_MPBR_EFR) | Z180_CNTLA_RTS0);
	cntlb_w((m_asci_cntlb & (Z180_CNTLB_MPBT | Z180_CNTLB_CTS_PS)) | 0x07);
	m_asci_stat = (m_asci_stat & Z180_STAT_DCD0) | Z180_STAT_TDRE;
}

void z180asci_channel_0::state_add(device_state_interface &parent)
{
	parent.state_add(Z180_CNTLA0, "CNTLA0",  m_asci_cntla);
	parent.state_add(Z180_CNTLB0, "CNTLB0",  m_asci_cntlb);
	parent.state_add(Z180_STAT0,  "STAT0",   m_asci_stat);
	parent.state_add(Z180_TDR0,   "TDR0",    m_asci_tdr);
	parent.state_add(Z180_RDR0,   "RDR0",    m_asci_rdr);
	if (m_ext)
	{
		parent.state_add(Z180_ASEXT0, "ASEXT0",  m_asci_ext).mask(Z180_ASEXT0_MASK);
		parent.state_add(Z180_ASTC0,  "ASTC0",   m_asci_tc.w);
	}
}

void z180asci_channel_0::stat_w(uint8_t data)
{
	LOG("Z180 STAT0  wr $%02x ($%02x)\n", data,  data & (Z180_STAT_RIE | Z180_STAT_TIE));
	m_asci_stat = (m_asci_stat & ~(Z180_STAT_RIE | Z180_STAT_TIE)) | (data & (Z180_STAT_RIE | Z180_STAT_TIE));
}

void z180asci_channel_0::asext_w(uint8_t data)
{
	LOG("Z180 ASEXT0 wr $%02x ($%02x)\n", data,  data & Z180_ASEXT0_MASK & ~Z180_ASEXT_BRK_DET);
	m_asci_ext = (m_asci_ext & Z180_ASEXT_BRK_DET) | (data & Z180_ASEXT0_MASK & ~Z180_ASEXT_BRK_DET);
	z180asci_channel_base::asext_w(data);
	update_tdre();
}

//**************************************************************************
//  z180asci_channel_1
//**************************************************************************

z180asci_channel_1::z180asci_channel_1(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, const bool ext)
	: z180asci_channel_base(mconfig, type, tag, owner, clock, 1, ext)
{
}

z180asci_channel_1::z180asci_channel_1(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z180asci_channel_1(mconfig, Z180ASCI_CHANNEL_1, tag, owner, clock, false)
{
}
void z180asci_channel_1::device_reset()
{
	z180asci_channel_base::device_reset();
	cntla_w((m_asci_cntla & Z180_CNTLA_MPBR_EFR) | Z180_CNTLA1_CKA1D);
	cntlb_w((m_asci_cntlb & Z180_CNTLB_MPBT) | 0x07);
	m_asci_stat = Z180_STAT_TDRE;
}

void z180asci_channel_1::state_add(device_state_interface &parent)
{
	parent.state_add(Z180_CNTLA1, "CNTLA1",  m_asci_cntla);
	parent.state_add(Z180_CNTLB1, "CNTLB1",  m_asci_cntlb);
	parent.state_add(Z180_STAT1,  "STAT1",   m_asci_stat);
	parent.state_add(Z180_TDR1,   "TDR1",    m_asci_tdr);
	parent.state_add(Z180_RDR1,   "RDR1",    m_asci_rdr);
	if (m_ext)
	{
		parent.state_add(Z180_ASEXT1, "ASEXT1",  m_asci_ext).mask(Z180_ASEXT1_MASK);
		parent.state_add(Z180_ASTC1,  "ASTC1",   m_asci_tc.w);
	}
}

void z180asci_channel_1::stat_w(uint8_t data)
{
	LOG("Z180 STAT1  wr $%02x ($%02x)\n", data,  data & (Z180_STAT_RIE | Z180_STAT1_CTS1E | Z180_STAT_TIE));
	m_asci_stat = (m_asci_stat & ~(Z180_STAT_RIE | Z180_STAT1_CTS1E | Z180_STAT_TIE)) | (data & (Z180_STAT_RIE | Z180_STAT1_CTS1E | Z180_STAT_TIE));
	update_tdre();
}

void z180asci_channel_1::asext_w(uint8_t data)
{
	LOG("Z180 ASEXT1 wr $%02x ($%02x)\n", data,  data & Z180_ASEXT1_MASK & ~Z180_ASEXT_BRK_DET);
	m_asci_ext = (m_asci_ext & Z180_ASEXT_BRK_DET) | (data & Z180_ASEXT1_MASK & ~Z180_ASEXT_BRK_DET);
	z180asci_channel_base::asext_w(data);
	update_tdre();
}

//**************************************************************************
//  z180asci_ext_channel_0
//**************************************************************************

z180asci_ext_channel_0::z180asci_ext_channel_0(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z180asci_channel_0(mconfig, Z180ASCI_EXT_CHANNEL_0, tag, owner, clock, true)
{
}

//**************************************************************************
//  z180asci_channel_1
//**************************************************************************

z180asci_ext_channel_1::z180asci_ext_channel_1(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z180asci_channel_1(mconfig, Z180ASCI_EXT_CHANNEL_1, tag, owner, clock, true)
{
}

//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(Z180ASCI_CHANNEL_0,   z180asci_channel_0,  "z180asci_channel_0",    "Z180 ASCI Channel 0")
DEFINE_DEVICE_TYPE(Z180ASCI_CHANNEL_1,   z180asci_channel_1,  "z180asci_channel_1",    "Z180 ASCI Channel 1")

DEFINE_DEVICE_TYPE(Z180ASCI_EXT_CHANNEL_0,   z180asci_ext_channel_0,  "z180asci_ext_channel_0",    "Z180 ASCI Extended Channel 0")
DEFINE_DEVICE_TYPE(Z180ASCI_EXT_CHANNEL_1,   z180asci_ext_channel_1,  "z180asci_ext_channel_1",    "Z180 ASCI Extended Channel 1")

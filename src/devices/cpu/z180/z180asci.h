// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic
/*********************************************************************

    z180asci.h

*********************************************************************/

#ifndef MAME_CPU_Z180_Z180ASCI_H
#define MAME_CPU_Z180_Z180ASCI_H

#pragma once

//**************************************************************************
//  z180asci_channel_base
//**************************************************************************

class z180asci_channel_base : public device_t
{
public:
	// inline configuration
	auto txa_handler() { return m_txa_handler.bind(); }
	auto rts_handler() { return m_rts_handler.bind(); }
	auto cka_handler() { return m_cka_handler.bind(); }
	// Opt in for an idle-input clock optimization; wire-level RX stays live.
	void set_idle_receive_clock_gating(bool enabled) { m_idle_receive_clock_gating = enabled; }

	uint8_t cntla_r();
	uint8_t cntlb_r();
	uint8_t stat_r();
	uint8_t tdr_r();
	uint8_t rdr_r();
	uint8_t asext_r();
	uint8_t astcl_r();
	uint8_t astch_r();
	void cntla_w(uint8_t data);
	void cntlb_w(uint8_t data);
	virtual void stat_w(uint8_t data) = 0;
	void tdr_w(uint8_t data);
	void rdr_w(uint8_t data);
	virtual void asext_w(uint8_t data);
	void astcl_w(uint8_t data);
	void astch_w(uint8_t data);

	void iostop_w(bool stopped);
	void rxa_wr(int state);
	void cts_wr(int state);
	void dcd_wr(int state);
	void cka_wr(int state);

	virtual void state_add(device_state_interface &parent) = 0;

	// UM0050 Fig55 and pp145-146: requests remain levels until status clears.
	ATTR_FORCE_INLINE int check_interrupt() const {
		if ((m_asci_stat & 3) == 3) return 1;
		if (!(m_asci_stat & 8)) return 0;
		u8 const rx = (m_asci_stat & ((m_ext && (m_asci_ext & 0x80)) ? 0x70 : 0xf0)) |
			(m_id ? 0 : (m_asci_stat & 4));
		return rx != 0;
	}
	void clear_interrupt() { }
	bool dma_request(bool receive) const { return (m_asci_stat & (receive ? 0x80 : 2)) != 0; }
protected:
	z180asci_channel_base(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, const int id, const bool ext);

	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_clock_changed() override;

	void transmit_edge();
	void receive_edge();
	void set_fifo_data(uint8_t data, uint8_t error);
	void update_fifo_status();
	void prepare_tsr();
	void update_tdre();
	void update_received();
	void update_total_bits();

	enum serial_state
	{
		STATE_START,
		STATE_DATA,
		STATE_WAIT,
		STATE_BREAK
	};


	void output_txa(int txa);
	void output_rts(int rts);
	TIMER_CALLBACK_MEMBER(rcv_clock);
	TIMER_CALLBACK_MEMBER(tra_clock) { transmit_edge(); }
	void park_receive_clock();
	void resume_receive_clock();

	emu_timer *m_rcv_clock;
	emu_timer *m_tra_clock;
	attotime m_bit_rate;
	attotime m_sample_rate;
	attotime m_receive_epoch = attotime::zero;
	bool m_receive_parked = false;
	bool m_iostop = false;
	bool m_idle_receive_clock_gating = false;

	devcb_write_line m_txa_handler;
	devcb_write_line m_rts_handler;
	devcb_write_line m_cka_handler;

	uint8_t   m_asci_cntla;                  // ASCI control register A
	uint8_t   m_asci_cntlb;                  // ASCI control register B
	uint8_t   m_asci_stat;                   // ASCI status register
	uint8_t   m_asci_tdr;                    // ASCI transmit data register
	uint8_t   m_asci_rdr;                    // ASCI receive data register
	uint8_t   m_asci_ext;                    // ASCI extension control register
	PAIR16    m_asci_tc;                     // ASCI time constant

	bool m_tdr_empty = true;
	uint16_t  m_tsr;
	uint16_t  m_rsr;
	uint8_t   m_data_fifo[4];
	uint8_t   m_error_fifo[4];
	uint8_t   m_fifo_wr;
	uint8_t   m_fifo_rd;
	uint8_t   m_fifo_count = 0;
	bool      m_fifo_overrun = false;

	uint8_t   m_cts;
	uint8_t   m_dcd;
	uint8_t   m_irq;
	uint8_t   m_txa;
	uint8_t   m_rxa;
	uint8_t   m_rts;

	uint32_t  m_divisor;

	uint8_t   m_clock_state;
	uint8_t   m_tx_state;
	uint8_t   m_rx_state;
	uint8_t   m_rx_bits;

	uint8_t   m_tx_counter;
	uint8_t   m_rx_counter;
	uint8_t   m_rx_count_to;
	uint8_t   m_rx_total_bits;

	bool m_rx_enabled;

	const int  m_id;
	const bool m_ext;
};

//**************************************************************************
//  z180asci_channel_0
//**************************************************************************

class z180asci_channel_0 : public z180asci_channel_base
{
public:
	// construction/destruction
	z180asci_channel_0(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	void stat_w(uint8_t data) override;
	void asext_w(uint8_t data) override;
	void state_add(device_state_interface &parent) override;
protected:
	z180asci_channel_0(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, const bool ext);

	// device_t implementation
	virtual void device_reset() override ATTR_COLD;
};

//**************************************************************************
//  z180asci_channel_1
//**************************************************************************

class z180asci_channel_1 : public z180asci_channel_base
{
public:
	// construction/destruction
	z180asci_channel_1(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	void stat_w(uint8_t data) override;
	void asext_w(uint8_t data) override;
	void state_add(device_state_interface &parent) override;
protected:
	z180asci_channel_1(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, const bool ext);

	// device_t implementation
	virtual void device_reset() override ATTR_COLD;
};

//**************************************************************************
//  z180asci_ext_channel_0
//**************************************************************************

class z180asci_ext_channel_0 : public z180asci_channel_0
{
public:
	// construction/destruction
	z180asci_ext_channel_0(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

//**************************************************************************
//  z180asci_ext_channel_1
//**************************************************************************

class z180asci_ext_channel_1 : public z180asci_channel_1
{
public:
	// construction/destruction
	z180asci_ext_channel_1(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

//**************************************************************************
//  DEVICE TYPE DEFINITIONS
//**************************************************************************

DECLARE_DEVICE_TYPE(Z180ASCI_CHANNEL_0, z180asci_channel_0)
DECLARE_DEVICE_TYPE(Z180ASCI_CHANNEL_1, z180asci_channel_1)

DECLARE_DEVICE_TYPE(Z180ASCI_EXT_CHANNEL_0, z180asci_ext_channel_0)
DECLARE_DEVICE_TYPE(Z180ASCI_EXT_CHANNEL_1, z180asci_ext_channel_1)

#endif // MAME_CPU_Z180_Z180ASCI_H

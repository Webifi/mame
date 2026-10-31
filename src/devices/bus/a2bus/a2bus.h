// license:BSD-3-Clause
// copyright-holders:R. Belmont
/***************************************************************************

  a2bus.h - Apple II slot bus and card emulation

  by R. Belmont

***************************************************************************/

#ifndef MAME_BUS_A2BUS_A2BUS_H
#define MAME_BUS_A2BUS_A2BUS_H

#pragma once

#include <array>
#include <utility>
#include <vector>


// /INH special addresses
static constexpr uint16_t INH_START_INVALID = 0xffff;
static constexpr uint16_t INH_END_INVALID   = 0x0000;

// /INH types
static constexpr int INH_NONE  = 0x00;
static constexpr int INH_READ  = 0x01;
static constexpr int INH_WRITE = 0x02;

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class a2bus_device;
class device_a2bus_card_interface;

// An IIgs motherboard transfer, not a CPU cache/private-RAM access.
// Signal levels are physical: m2sel_n is active low; m2b0 is the sampled CPU bank bit (before RAMWRT/80STORE).
// time is the transfer end, or the pin-transition time when phased is set.
// An unwired pin 35 is explicitly invalid.
struct a2bus_mega_cycle
{
	uint16_t address;
	uint8_t data;
	bool read;
	bool m2sel_n;
	bool m2b0;
	bool m2b0_valid;
	attotime time;
	// Optional physical phase stream. Legacy observers receive a completed
	// transfer; phase-aware hosts also deliver the intervening slot clocks.
	bool phased = false;
	bool ph0 = false;
	bool m7 = false;
	// A timed host retains this same transfer through RDY-low PH2 falls.
	bool timed = false;
	bool retry = false;
};

class a2bus_slot_device : public device_t, public device_single_card_slot_interface<device_a2bus_card_interface>
{
public:
	// construction/destruction
	template <typename T, typename U>
	a2bus_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock, T &&a2bus_tag, U &&opts, const char *dflt)
		: a2bus_slot_device(mconfig, tag, owner, clock)
	{
		set_options(std::forward<U>(opts), dflt, false);
		m_a2bus.set_tag(std::forward<T>(a2bus_tag));
	}
	a2bus_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	a2bus_slot_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device_t implementation
	virtual void device_resolve_objects() override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;

	// configuration
	required_device<a2bus_device> m_a2bus;
};

// device type definition
DECLARE_DEVICE_TYPE(A2BUS_SLOT, a2bus_slot_device)


// ======================> a2bus_device
class a2bus_device : public device_t
{
	// multi-card devices need to access m_device_list, so they get friended here.
	friend class a2bus_mcms2_device;
public:
	// construction/destruction
	a2bus_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	// inline configuration
	template <typename T> void set_space(T &&tag, int spacenum) { m_maincpu_space.set_tag(std::forward<T>(tag), spacenum); }
	auto irq_w() { return m_out_irq_cb.bind(); }
	auto nmi_w() { return m_out_nmi_cb.bind(); }
	auto inh_w() { return m_out_inh_cb.bind(); }
	auto dma_w() { return m_out_dma_cb.bind(); }
	auto rdy_w() { return m_out_rdy_cb.bind(); }
	auto read_complete_w() { return m_out_read_complete_cb.bind(); }
	void complete_mega_read(int slot, uint8_t data);
	void set_rdy_line(bool ready, int slot);
	void set_dma_sync(std::function<void ()> sync) { m_dma_sync = std::move(sync); }
	// Execution policy only: does this host deliver every physical slot phase?
	void set_mega_phase_query(std::function<bool ()> query) { m_mega_phase_query = std::move(query); }
	bool has_mega_phases() const { return m_mega_phase_query && m_mega_phase_query(); }
	void set_dma_observer(std::function<void (uint32_t, bool, uint8_t &)> observer) { m_dma_observer = std::move(observer); }
	void set_m2b0_slot_mask(uint8_t mask) { invalidate_mega_phase_permissions(); m_m2b0_slot_mask = mask; }
	bool has_mega_observers() const { return !m_mega_observers.empty(); }
	bool synchronize_mega(attotime when);
	// Permission depends on inputs and ownership, not on the clocks advanced
	// by the admitted closed form. Observers invalidate it at every other change.
	void invalidate_mega_phase_permissions() { m_idle_permission = m_read_permission = -1; }
	bool can_defer_mega_idle() const {
		if (m_idle_permission < 0) m_idle_permission = compute_mega_idle_permission();
		return m_idle_permission;
	}
	bool can_defer_mega_read(u32 address, u8 data) const {
		if (m_read_permission < 0 || address != m_permission_address || data != m_permission_data) {
			m_permission_address = address; m_permission_data = data;
			m_read_permission = compute_mega_read_permission(address, data);
		}
		return m_read_permission;
	}
	void advance_mega_idle(u64 clock);
	bool defer_mega_idle(u64 clock);
	bool can_defer_mega_transition(u32 address, bool read) const;
	void defer_mega_transition(u64 clock, u16 address, u8 data, bool unselected);
	bool synchronize_mega_irq(attotime when);
	uint8_t mega_cycle(a2bus_mega_cycle const &cycle);
	uint8_t mega_read_data(uint8_t fallback) const;
	bool mega_read_driven() const { return m_mega_drive_mask != 0; }

	void add_a2bus_card(int slot, device_a2bus_card_interface *card);
	device_a2bus_card_interface *get_a2bus_card(int slot);
	uint8_t get_a2bus_irq_mask();
	uint8_t get_a2bus_nmi_mask();

	void set_irq_line(int state, int slot);
	void set_nmi_line(int state, int slot);
	void set_dma_line(int state);
	void recalc_inh(int slot);
	// FPI/CYA DMA bank ($C037). IIe leaves this 0, so slot DMA stays in bank $00.
	void set_dma_bank(uint8_t bank) { m_dma_bank = bank; }
	uint8_t dma_bank() const { return m_dma_bank; }
	uint8_t dma_r(uint16_t offset);
	void dma_w(uint16_t offset, uint8_t data);
	void reset_bus();

	void irq_w(int state);
	void nmi_w(int state);

protected:
	a2bus_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// internal state
	required_address_space m_maincpu_space;
	std::function<void ()> m_dma_sync;
	std::function<bool ()> m_mega_phase_query;
	std::function<void (uint32_t, bool, uint8_t &)> m_dma_observer;
	std::vector<device_a2bus_card_interface *> m_mega_observers;
	uint8_t m_m2b0_slot_mask = 0;
	mutable int m_idle_permission = -1, m_read_permission = -1;
	mutable u32 m_permission_address = 0;
	mutable u8 m_permission_data = 0;
	bool compute_mega_idle_permission() const;
	bool compute_mega_read_permission(u32 address, u8 data) const;

	devcb_write_line    m_out_irq_cb;
	devcb_write_line    m_out_nmi_cb;
	devcb_write8        m_out_inh_cb;
	devcb_write_line    m_out_dma_cb;
	devcb_write_line    m_out_rdy_cb;
	devcb_write8 m_out_read_complete_cb;
	uint8_t m_mega_drive_mask = 0;
	std::array<uint8_t, 8> m_mega_data{};
	uint8_t m_rdy_low_mask = 0;
	bool m_dma_line = false, m_dma_active = false;
	unsigned m_dma_depth = 0;
	void update_dma_owner();
	void begin_dma();
	void end_dma();

	device_a2bus_card_interface *m_device_list[8];

	uint8_t m_slot_irq_mask;
	uint8_t m_slot_nmi_mask;
	uint8_t m_dma_bank;
};


// device type definition
DECLARE_DEVICE_TYPE(A2BUS, a2bus_device)

// ======================> device_a2bus_card_interface

// class representing interface-specific live a2bus card
class device_a2bus_card_interface : public device_interface
{
	friend class a2bus_device;
public:
	// construction/destruction
	virtual ~device_a2bus_card_interface();

	virtual uint8_t read_c0nx(uint8_t offset) { device().logerror("a2bus: unhandled read at C0n%x\n", offset); return 0; }       // C0nX - /DEVSEL
	virtual void write_c0nx(uint8_t offset, uint8_t data) { device().logerror("a2bus: unhandled write %02x to C0n%x\n", data, offset); }
	virtual uint8_t read_cnxx(uint8_t offset) { return 0; }       // CnXX - /IOSEL
	virtual void write_cnxx(uint8_t offset, uint8_t data) { device().logerror("a2bus: unhandled write %02x to Cn%02x\n", data, offset); }
	virtual uint8_t read_c800(uint16_t offset) { return 0; }      // C800 - /IOSTB
	virtual void write_c800(uint16_t offset, uint8_t data) {device().logerror("a2bus: unhandled write %02x to %04x\n", data, offset + 0xc800); }
	virtual bool take_c800() const { return false; }   // override and return true if your card can take over the /IOSTB space
	virtual uint8_t read_inh_rom(uint16_t offset) { return 0; }
	virtual void write_inh_rom(uint16_t offset, uint8_t data) { }
	virtual uint16_t inh_start() { return INH_START_INVALID; }
	virtual uint16_t inh_end() { return INH_END_INVALID; }
	virtual bool inh_check(uint16_t offset, bool bIsWrite) { return false; }
	virtual int inh_type() { return INH_NONE; }
	virtual void reset_from_bus() { }
	virtual bool observes_mega_cycles() const { return false; }
	virtual bool synchronize_mega_irq(attotime when) { return true; }
	// Return true only when driving a read. All observers see the same input;
	// the bus combines driven bits, independent of slot enumeration order.
	virtual bool observe_mega_cycle(a2bus_mega_cycle const &cycle, uint8_t &data) { return false; }
	virtual bool synchronize_mega(attotime when) { return true; }
	// /DMA disconnects the CPU's address buffers. RDY does not hold the
	// independent master; an unfinished CPU transfer resumes on its return.
	virtual void mega_dma_w(bool active) { }
	// An observer enabling either closed form must invalidate the bus permission
	// when its inputs/state change outside defer_mega_idle's admitted recurrence.
	virtual bool can_defer_mega_transition(u16 address, bool read) const { return false; }
	virtual void defer_mega_transition(u64 clock, u16 address, u8 data, bool unselected) { }
	virtual bool can_defer_mega_idle() const { return false; }
	virtual bool can_defer_mega_read(u16 address, u8 data) const { return false; }
	virtual void defer_mega_idle(u64 clock) { }

	device_a2bus_card_interface *next() const { return m_next; }

	// inline configuration
	void set_a2bus(a2bus_device *a2bus, const char *slottag) { m_a2bus = a2bus; m_a2bus_slottag = slottag; }
	template <typename T> void set_onboard(T &&a2bus) { m_a2bus_finder.set_tag(std::forward<T>(a2bus)); m_a2bus_slottag = device().tag(); }

	uint8_t slot_dma_read(uint16_t offset) { return m_a2bus->dma_r(offset); }
	void slot_dma_write(uint16_t offset, uint8_t data) { m_a2bus->dma_w(offset, data); }

protected:
	uint32_t get_slotromspace() { return 0xc000 | (m_slot<<8); }      // return Cn00 address for this slot
	uint32_t get_slotiospace() { return 0xc080 + (m_slot<<4); }       // return C0n0 address for this slot

	void raise_slot_irq() { m_a2bus->set_irq_line(ASSERT_LINE, m_slot); }
	void lower_slot_irq() { m_a2bus->set_irq_line(CLEAR_LINE, m_slot); }
	void raise_slot_nmi() { m_a2bus->set_nmi_line(ASSERT_LINE, m_slot); }
	void lower_slot_nmi() { m_a2bus->set_nmi_line(CLEAR_LINE, m_slot); }
	void recalc_slot_inh() { m_a2bus->recalc_inh(m_slot); }
	void raise_slot_dma() { m_a2bus->set_dma_line(ASSERT_LINE); }
	void lower_slot_dma() { m_a2bus->set_dma_line(CLEAR_LINE); }
	void complete_slot_read(uint8_t data) { m_a2bus->complete_mega_read(m_slot, data); }
	void set_slot_rdy(bool ready) { m_a2bus->set_rdy_line(ready, m_slot); }

	device_a2bus_card_interface(const machine_config &mconfig, device_t &device);

	virtual void interface_validity_check(validity_checker &valid) const override;
	virtual void interface_pre_start() override;

	int slotno() const { assert(m_a2bus); return m_slot; }
	a2bus_device &a2bus() { assert(m_a2bus); return *m_a2bus; }

private:
	optional_device<a2bus_device> m_a2bus_finder;
	a2bus_device *m_a2bus;
	const char *m_a2bus_slottag;
	int m_slot;
	device_a2bus_card_interface *m_next;
};

#endif  // MAME_BUS_A2BUS_A2BUS_H

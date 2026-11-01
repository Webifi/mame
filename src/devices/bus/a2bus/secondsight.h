// license:BSD-3-Clause
#ifndef MAME_BUS_A2BUS_SECONDSIGHT_H
#define MAME_BUS_A2BUS_SECONDSIGHT_H
#pragma once

#include "a2bus.h"
#include "secondsight_lattice_bbb1.h"
#include "secondsight_idle.h"
#include "secondsight_idle_queue.h"
#include "secondsight_video.h"
#include "cpu/z180/z180.h"
#include "secondsight_queue.h"
#include "secondsight_copy.h"
#include <thread>
#include <deque>

class a2bus_secondsight_device : public device_t, public device_a2bus_card_interface
{
  public:
	a2bus_secondsight_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock = 0);
	~a2bus_secondsight_device();

	virtual bool observes_mega_cycles() const override { return true; }

	virtual bool synchronize_mega_irq(attotime when) override;
	virtual bool observe_mega_cycle(a2bus_mega_cycle const &cycle, u8 &data) override;
	virtual bool synchronize_mega(attotime when) override;
	virtual void mega_dma_w(bool active) override;
	virtual bool can_defer_mega_transition(u16 address, bool read) const override;
	virtual void defer_mega_transition(u64 clock, u16 address, u8 data, bool unselected) override;
	virtual bool can_defer_mega_idle() const override;
	virtual bool can_defer_mega_read(u16 address, u8 data) const override;

	virtual void defer_mega_idle(u64 clock) override { m_idle_limit = clock; }

	virtual void reset_from_bus() override;

	// No slot-selected registers or ROM: Lattice chip decodes the absolute Apple address.
	virtual u8 read_c0nx(u8 offset) override { return 0xff; }

	virtual void write_c0nx(u8 offset, u8 data) override {}

	virtual u8 read_cnxx(u8 offset) override { return 0xff; }

	virtual void write_cnxx(u8 offset, u8 data) override {}

  protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_stop() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual tiny_rom_entry const *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

  private:
	enum class work_kind
	{
		phase,
		grant,
		dma,
		stop
	};

	struct work_item
	{
		work_kind kind = work_kind::grant;
		a2bus_mega_cycle cycle{};
		u64 fence = 0;
		bool state = false;
	};
	enum class output_kind
	{
		rdy,
		irq,
		read
	};

	struct output_item
	{
		output_kind kind;
		u8 value;
		attotime when;
	};

	std::unique_ptr<device_scheduler> m_card_scheduler;
	device_scheduler *m_host_scheduler = nullptr;
	secondsight_queue<work_item, 1U << 20> m_work;
	std::thread m_worker;
	std::atomic<u64> m_completed{0};
	std::atomic<bool> m_failed{false};
	std::exception_ptr m_worker_error;
	u64 m_fence = 0;
	std::vector<output_item> m_outputs;
	bool m_response_drive = false, m_response_meeting = false;
	u8 m_response_data = 0xff;
	attotime m_response_time;
	bool m_host_meeting = false;
	bool m_grant_work = false, m_servicing = false;
	bool m_shared_execution = false;
	u64 m_idle_clock = 0, m_idle_limit = 0;
	secondsight_idle_queue m_idle_inputs;
	void materialize_idle(attotime when);

	void frame_clocks() { materialize_idle(machine().time()); }

	emu_timer *m_output_timer = nullptr;
	std::deque<output_item> m_host_outputs;
	u32 m_saved_outputs = 0;
	std::array<u8, 128> m_saved_output_kind{}, m_saved_output_data{};
	std::array<attotime, 128> m_saved_output_time{};
	void start_worker();
	void stop_worker();
	void worker_main();
	void process_work(work_item const &item);
	void finish_work(work_item const &item, attotime when);
	void process_stream(work_item &item, bool &stop);
	bool enqueue(work_item const &item);
	void wait_worker(u64 fence, bool completed_inline);
	void worker_barrier(attotime when);
	void publish_outputs();
	void apply_output(output_item const &item);
	TIMER_CALLBACK_MEMBER(output_complete);
	void card_rdy(bool ready);
	void card_irq(bool asserted);
	void card_read(u8 data);
	bool process_cycle(a2bus_mega_cycle const &cycle, u8 &data);
	void process_dma(bool active);
	required_device<z8s180_device> m_cpu;
	required_device<secondsight_video_device> m_video;
	required_region_ptr<u8> m_rom;
	memory_share_creator<u8> m_sram;
	// U7 Lattice chip configuration; sampled only at power-up.
	required_ioport m_lattice_chip;
	secondsight_lattice_bbb1 m_logic;
	bool m_lattice_v2 = false, m_lattice_chip_configured = false;
	// Z180 /INT0 pull-up, a card modification; sampled only at power-up.
	required_ioport m_int0_mod;
	bool m_int0_pullup = false, m_int0_configured = false, m_cint = false;
	secondsight_lattice::inputs m_pins;
	secondsight_lattice::inputs m_cpu_pins;
	bool m_dma_owned = false, m_cpu_bank = false;
	// U2 Apple data-in latch and U3 Apple data-out latch.
	u8 m_apple_data_in = 0, m_apple_data_out = 0;
	u8 m_bus_data = 0xff, m_port_diagnostics = 0;
	bool m_bus_valid = false;
	bool m_oak_scan_test = false; // derived from Oak ext03 on reset/write/postload
	void drive_hd(u8 data);
	IRQ_CALLBACK_MEMBER(irq0_ack);
	u16 m_address_latches = 0; // U5/U6 address latches
	bool m_catch_bank = false, m_rom_overlay = true, m_vga_half = false, m_rgb = false, m_linear = false;
	u8 m_oak_index = 0, m_oak_map = 0;
	bool m_bus_request = false, m_waiting = false;
	bool m_external_phases = false;
	bool m_timed_completion = false;
	bool m_apple_irq = false;
	int m_int1 = -1;
	u8 m_catch_data = 0;
	attotime m_cycle_time;
	secondsight_copy_controller m_copy;
	u64 m_copy_overwrites = 0, m_copy_under_write = 0;
	u64 m_copies = 0, m_events = 0, m_stalls = 0, m_read_stalls = 0;
	u64 m_video_waits = 0, m_video_wait_cycles = 0;
	emu_timer *m_grant_timer = nullptr;
	bool m_idle_memory_safe = true;
	struct worker_space
	{
		address_space *space;
		std::unordered_set<handler_entry *> owned;
		util::notifier_subscription watch;
		~worker_space() { for (auto *entry : owned) entry->unref(); }
	};
	std::vector<std::unique_ptr<worker_space>> m_worker_spaces;
	unsigned m_map_changes = 0;
	bool m_inline_callbacks = false;
	void watch_worker_spaces();
	void worker_map_change(bool before);

	enum class select
	{
		none,
		rom,
		sram,
		bank,
		vga,
		lattice,
		rgb,
		linear,
		irq
	};
	select decode_board(offs_t address, bool io, bool write) const;
	u8 memory_r(offs_t address);
	void memory_w(offs_t address, u8 data);
	u8 vga_r(offs_t port);
	void vga_w(offs_t port, u8 data);
	u32 oak_address(offs_t address);
	void linear_access(bool enabled);
	void port_diagnostic(unsigned bit, char const *reason);
	void video_access(bool write);
	u8 port_r(offs_t port);
	void port_w(offs_t port, u8 data);
	void program_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;
	u8 io_r(offs_t port);
	void io_w(offs_t port, u8 data);

	void cint_w(int state);
	void rts0_w(int state)
	{
		m_rom_overlay = bool(state);
		update_direct_reads();
	}

	void update_direct_reads();
	void settle(bool latch_outputs = true);
	void clock_7m(bool latch_outputs = true);
	bool observe_mega_phase(a2bus_mega_cycle const &cycle, u8 &data);
	void finish_cycle();
	void acknowledge();
	void drive_interrupt();
	void start_catch();
	void advance_copy(attotime when);
	void schedule_copy();
	TIMER_CALLBACK_MEMBER(copy_tick);
};
DECLARE_DEVICE_TYPE(A2BUS_SECONDSIGHT, a2bus_secondsight_device)
#endif

// license:BSD-3-Clause
// Sequential Systems Second Sight, original firmware 1.4-BEEF.
// Sources: Second Sight schematics, sheets 1-2; original U7 Lattice chip
// BE8C fuse map; ROM_BEEF and VGALOW.ASM. The U4 Z180 I/O decoder,
// U15 Z180 memory decoder and U17 copy controller are undumped;
// their inferred state machines and timing parameters remain model limitations.
#include "emu.h"
#include "secondsight.h"
#include "screen.h"

#define LOG_BUS (1U << 1)
#define LOG_DIAGNOSTIC (1U << 2)
#define VERBOSE LOG_DIAGNOSTIC
#include "logmacro.h"

DEFINE_DEVICE_TYPE(A2BUS_SECONDSIGHT, a2bus_secondsight_device, "a2secondsight", "Sequential Systems Second Sight")

ROM_START(secondsight)
	ROM_REGION(0x20000, "rom", 0)
	ROM_DEFAULT_BIOS("14")
	ROM_SYSTEM_BIOS(0, "14", "Firmware 1.4-BEEF (original)")
	ROMX_LOAD("rom_beef.bin", 0, 0x20000, CRC(5354b762) SHA1(1d665efc6cbbcb565a11e58e0dcf3e1594265c1b), ROM_BIOS(0))
	// Firmware 1.5/2.0 builds, 2026-10-24; not original EPROM dumps.
	// SHA256 19e9b6f5dbbb50378c026a09e7e0ea39d4cc90463de70fb926038563021b6846
	ROM_SYSTEM_BIOS(1, "15", "Firmware 1.5-CBFC")
	ROMX_LOAD("rom_cbfc.bin", 0, 0x20000, CRC(a213fa61) SHA1(d6c63405d18a0687093b75672ad4c336566a06e6), ROM_BIOS(1))
	// SHA256 f5e252bb5704694bacc6319116f2344b3543c657181906baadec95ba1706831f
	ROM_SYSTEM_BIOS(2, "20", "Firmware 2.0-3277 (needs Lattice v2.0)")
	ROMX_LOAD("rom_3277.bin", 0, 0x20000, CRC(94ac4d51) SHA1(992eddbb0f23d60a8b4deeff16f17fb1ea20e376), ROM_BIOS(2))
ROM_END

static INPUT_PORTS_START(secondsight)
	PORT_START("LATTICE")
	PORT_CONFNAME(0x01, 0x00, "Lattice chip (power cycle)")
	PORT_CONFSETTING(0x00, "Original (BE8C)")
	PORT_CONFSETTING(0x01, "v2.0 (BBB1)")
	PORT_START("INT0")
	PORT_CONFNAME(0x01, 0x00, "INT0 pull-up (power cycle)")
	PORT_CONFSETTING(0x00, "None (stock card)")
	PORT_CONFSETTING(0x01, "Added (card modification)")
INPUT_PORTS_END

a2bus_secondsight_device::a2bus_secondsight_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, A2BUS_SECONDSIGHT, tag, owner, clock)
	, device_a2bus_card_interface(mconfig, *this)
	, m_cpu(*this, "z180")
	, m_video(*this, "video")
	, m_rom(*this, "rom")
	, m_sram(*this, "sram", 0x20000, ENDIANNESS_LITTLE)
	, m_lattice_chip(*this, "LATTICE")
	, m_int0_mod(*this, "INT0")
{
}

a2bus_secondsight_device::~a2bus_secondsight_device()
{
	stop_worker();
}

void a2bus_secondsight_device::device_stop()
{
	stop_worker();
}

void a2bus_secondsight_device::start_worker()
{
	if (m_worker.joinable())
		return;
	assert(&m_cpu->scheduler() == m_card_scheduler.get());
	m_worker = std::thread([this]() { worker_main(); });
}

void a2bus_secondsight_device::stop_worker()
{
	if (!m_worker.joinable())
		return;
	if (!m_failed.load(std::memory_order_acquire))
	{
		work_item item;
		item.kind = work_kind::stop;
		// Shutdown drains every previously published command. No transport
		// backpressure is turned into an emulated wait or discarded edge.
		// A failed consumer cannot drain a full queue. Join it directly
		// instead of waiting forever to publish the stop command.
		while (!m_failed.load(std::memory_order_acquire) && !m_work.push(item))
			std::this_thread::yield();
	}
	m_worker.join();
}

bool a2bus_secondsight_device::enqueue(work_item const &item)
{
	if (m_failed.load(std::memory_order_acquire))
		std::rethrow_exception(m_worker_error);
	// A synchronous request has no host work to overlap. If the card is idle,
	// take its consumer token and service the same ordered stream inline.
	if ((item.fence || m_inline_callbacks || machine().debug_enabled()) && m_work.claim())
	{
		try
		{
			device_scheduler::domain_scope scope(*m_card_scheduler);
			work_item pending;
			while (m_work.take(pending))
				process_work(pending);
			process_work(item);
			m_work.release();
		}
		catch (...)
		{
			m_work.release();
			throw;
		}
		return true;
	}
	start_worker();
	if (!m_work.push(item, item.fence != 0))
		fatalerror("Second Sight input queue exceeded a frame of physical edges");
	return false;
}

void a2bus_secondsight_device::wait_worker(u64 fence, bool completed_inline)
{
	// Inline service has already completed this request under the consumer
	// token. Only worker requests need the cross-thread completion handshake.
	if (!completed_inline)
		for (u64 done = m_completed.load(std::memory_order_acquire); done < fence; done = m_completed.load(std::memory_order_acquire))
			m_completed.wait(done, std::memory_order_relaxed);
	if (m_failed.load(std::memory_order_acquire))
		std::rethrow_exception(m_worker_error);
}

void a2bus_secondsight_device::process_work(work_item const &item)
{
	m_cpu->set_grant_continuation(false);
	m_servicing = true;
	m_grant_work = item.kind == work_kind::grant;
	const attotime when = m_grant_work && item.cycle.time < m_card_scheduler->base_time() ?
		m_card_scheduler->base_time() :
		m_card_scheduler->advance_domain(*m_cpu, item.cycle.time, !m_grant_work, !m_grant_work || item.state);
	finish_work(item, when);
}

void a2bus_secondsight_device::finish_work(work_item const &item, attotime when)
{
	m_response_time = when;
	if (item.kind == work_kind::phase)
	{
		m_response_data = 0xff;
		m_response_drive = process_cycle(item.cycle, m_response_data);
		m_response_meeting = m_waiting || m_logic.q[secondsight_lattice::IRQ_PENDING] || !m_logic.q[secondsight_lattice::RUN] ||
			m_logic.q[secondsight_lattice::EVENT_LATCH];
		if (!item.fence && (m_waiting || m_response_drive))
			fatalerror("Second Sight asynchronous phase required a host rendezvous at %s", item.cycle.time.as_string());
		m_card_scheduler->finish_domain_edge(item.cycle.time);
	}
	else if (item.kind == work_kind::dma)
	{
		process_dma(item.state);
		m_card_scheduler->finish_domain_edge(item.cycle.time);
	}
	m_servicing = false;
}

void a2bus_secondsight_device::process_stream(work_item &item, bool &stop)
{
	// Retain a live instruction only when another input is already queued.
	// A terminal request or isolated phase uses the original driver. Later
	// publications remain queued and are serviced at their original times.
	if (item.fence || m_work.published() == m_work.consumed())
	{
		process_work(item);
		return;
	}
	auto take_next = [&]
	{
		if (item.fence || !m_work.take(item))
			return false;
		if (item.kind == work_kind::stop)
		{
			stop = true;
			return false;
		}
		return true;
	};
	auto target = [this](work_item const &work)
	{
		m_servicing = true;
		m_grant_work = work.kind == work_kind::grant;
		return device_scheduler::domain_target{work.cycle.time, !m_grant_work, !m_grant_work || work.state};
	};
	// A past grant intentionally has no scheduler side effects in the old
	// driver. Preserve that case instead of converting it into an equal-time
	// timer dispatch. Ordinary input phases are monotonically timestamped.
	while (item.kind == work_kind::grant && item.cycle.time < m_card_scheduler->base_time())
	{
		process_work(item);
		if (!take_next())
			return;
	}
	m_cpu->set_grant_continuation(true);
	auto next = [this, &item, &target, &take_next](attotime when, device_scheduler::domain_target &next)
	{
		finish_work(item, when);
		while (take_next())
		{
			next = target(item);
			if (!m_grant_work || item.cycle.time >= m_card_scheduler->base_time())
				return true;
			finish_work(item, m_card_scheduler->base_time());
		}
		return false;
	};
	// The driver invokes this callback synchronously. Refer to its stack
	// storage instead of allocating a std::function payload for every stream.
	m_card_scheduler->advance_domain_stream(*m_cpu, target(item), std::ref(next));
	m_cpu->set_grant_continuation(false);
}

void a2bus_secondsight_device::worker_main()
{
	device_scheduler::domain_scope scope(*m_card_scheduler);
	try
	{
		while (true)
		{
			m_work.wait_available();
			if (!m_work.claim())
			{
				m_work.wait_owner();
				continue;
			}
			u64 completed = 0;
			bool stop = false;
			try
			{
				work_item item;
				while (m_work.take(item))
				{
					if (item.kind == work_kind::stop)
					{
						stop = true;
						break;
					}
					process_stream(item, stop);
					if (stop)
						break;
					if (item.fence)
					{
						completed = item.fence;
						break;
					}
				}
			}
			catch (...)
			{
				m_work.release();
				throw;
			}
			m_work.release();
			if (completed)
			{
				m_completed.store(completed, std::memory_order_release);
				m_completed.notify_one();
			}
			if (stop)
				break;
		}
	}
	catch (...)
	{
		m_worker_error = std::current_exception();
		m_failed.store(true, std::memory_order_release);
		m_completed.store(~u64(0), std::memory_order_release);
		m_completed.notify_one();
	}
}

void a2bus_secondsight_device::worker_barrier(attotime when)
{
	if (m_shared_execution)
		return;
	do
	{
		work_item item;
		item.kind = work_kind::grant;
		item.cycle.time = when;
		item.state = !machine().scheduler().in_callback();
		item.fence = ++m_fence;
		wait_worker(item.fence, enqueue(item));
		publish_outputs();
	} while (m_response_time < when);
}

void a2bus_secondsight_device::apply_output(output_item const &item)
{
	switch (item.kind)
	{
	case output_kind::rdy:
		set_slot_rdy(item.value);
		break;
	case output_kind::irq:
		if (item.value)
			raise_slot_irq();
		else
			lower_slot_irq();
		break;
	case output_kind::read:
		complete_slot_read(item.value);
		break;
	}
}

void a2bus_secondsight_device::publish_outputs()
{
	for (auto const &item : m_outputs)
	{
		if (item.when <= machine().scheduler().base_time())
			apply_output(item);
		else
			m_host_outputs.push_back(item);
	}
	m_outputs.clear();
	if (!m_host_outputs.empty())
		m_output_timer->adjust_absolute(m_host_outputs.front().when);
}

TIMER_CALLBACK_MEMBER(a2bus_secondsight_device::output_complete)
{
	while (!m_host_outputs.empty() && m_host_outputs.front().when <= machine().time())
	{
		apply_output(m_host_outputs.front());
		m_host_outputs.pop_front();
	}
	if (!m_host_outputs.empty())
		m_output_timer->adjust_absolute(m_host_outputs.front().when);
}

void a2bus_secondsight_device::card_rdy(bool ready)
{
	if (m_servicing)
		m_outputs.push_back({output_kind::rdy, u8(ready), machine().time()});
	else
		set_slot_rdy(ready);
}

void a2bus_secondsight_device::card_irq(bool asserted)
{
	if (m_servicing)
	{
		m_outputs.push_back({output_kind::irq, u8(asserted), machine().time()});
		if (m_grant_work)
			m_card_scheduler->interrupt_domain(machine().time());
	}
	else if (asserted)
		raise_slot_irq();
	else
		lower_slot_irq();
}

void a2bus_secondsight_device::card_read(u8 data)
{
	if (m_servicing)
		m_outputs.push_back({output_kind::read, data, machine().time()});
	else
		complete_slot_read(data);
}

tiny_rom_entry const *a2bus_secondsight_device::device_rom_region() const
{
	return ROM_NAME(secondsight);
}

ioport_constructor a2bus_secondsight_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(secondsight);
}

void a2bus_secondsight_device::device_add_mconfig(machine_config &config)
{
	// Photographed U18 Z180: Z8S18020VSC, 9233 KV = original K/SL1960
	// (TN0053 p1), with CCR but no N-revision FIFO/BRG/CMR enhancements.
	// The CPU divides EXTAL by two after reset; do not divide twice.
	Z8S180_K(config, m_cpu, 36'000'000);
	m_cpu->set_machine_cycle_engine(true);
	// Schematic sheet 2, U18 Z180 pin 50 (/DREQ0) is tied to GND. Set the
	// electrical level before cold reset; CPU reset preserves it without
	// manufacturing an edge for DCNTL.DMS0 edge-sense operation.
	m_cpu->set_dma_request_initial_level(0, true);
	m_cpu->set_io_strobe([this](offs_t port, bool write, u8 data, bool asserted)
	{
		if ((port & 0xf8) != 0xa0)
			return;
		// Replay preceding motherboard edges before changing the Lattice chip's AX input.
		// The machine-cycle engine exposes IORQ before the final data edge.
		materialize_idle(machine().time());
		// UM0050 pp16-17: IORQ asserts in T1/T2 according to IOC.
		// The Z180 I/O decoder AX and Lattice chip ACK_A0 clear the request at that select
		// edge; BCLK still loads the Apple data-out latch at the final write edge in port_w.
		if (asserted)
		{
			m_pins.port = port & 7;
			m_pins.host_data = write ? data : m_bus_data;
			settle();
			m_pins.AX = true;
			settle();
			drive_interrupt();
		}
		else
		{
			m_pins.AX = false;
			settle();
		}
	});
	m_cpu->set_bus_wait([this](offs_t address, bool write, bool io) -> unsigned
	{
		address = m_cpu->a18_tout_address(address);
		if (io || decode_board(address, false, write) != select::vga)
			return 0;
		// U15 Z180 memory decoder has no RD/WR input. Its forced count
		// is identical in both directions, even if Oak declines the address.
		// IOCHRDY extends this floor; DCNTL overlaps it, it is not additive.
		attotime const phi = m_cpu->phi_time(1);
		unsigned const waits = std::max(secondsight_copy_controller::forced_memory_waits,
			m_video->memory_wait(oak_address(address), write, phi));
		++m_video_waits;
		m_video_wait_cycles += waits;
		return waits;
	});
	m_cpu->set_busack([this](bool asserted)
	{
		if (asserted && m_bus_request)
		{
			// UM0050 pp25-26/113: only the engine can grant, after the
			// current MC (including WAIT, or either half of a DMA byte).
			m_copy.grant(m_cpu->next_phi_edge(m_cpu->local_time()), m_cpu->phi_time(1));
			schedule_copy();
		}
	});
	m_cpu->set_addrmap(AS_PROGRAM, &a2bus_secondsight_device::program_map);
	m_cpu->set_addrmap(AS_IO, &a2bus_secondsight_device::io_map);
	m_cpu->set_irq_acknowledge_callback(FUNC(a2bus_secondsight_device::irq0_ack));
	m_cpu->rts0_wr_callback().set(FUNC(a2bus_secondsight_device::rts0_w));
	m_cpu->set_asci_idle_clock_gating(true);
	m_cpu->set_instruction_boundary_timing(true);
	m_cpu->set_deferred_execution();
	// ASCI0 exists in the CPU but has no serial connection on this board.
	// Firmware softswitch handlers send debug characters to its TDR (port 6).
	SECONDSIGHT_VIDEO(config, m_video);
	// Schematic: Oak's open-collector CINT feeds INT0. The video callback
	// uses logical assertion (1), not the electrical active-low pin level.
	m_video->cint_cb().set(FUNC(a2bus_secondsight_device::cint_w));
	m_video->scan_test_cb().set([this](int state) {
		m_oak_scan_test = bool(state);
		// Assumed ext03 convention: SD7:0 is the last Z180 bus byte.
		// While it is visible, direct fetches must use drive_hd too, so each
		// change splits the raster at its actual CPU/DMA/capture bus edge.
		if (state)
			m_video->system_data_w(m_bus_valid ? m_bus_data : 0xff);
		update_direct_reads();
	});
	// INT2 is unconnected; NMI is pulled high. TOUT remains disabled so its
	// multiplexed pin carries A18; neither serial channel is connected.
	// Bus observations, private peripheral events and frame deadlines bound
	// CPU execution without a machine-wide quantum.
}

void a2bus_secondsight_device::program_map(address_map &map)
{
	map(0x00000, 0xfffff).rw(FUNC(a2bus_secondsight_device::memory_r), FUNC(a2bus_secondsight_device::memory_w));
}

void a2bus_secondsight_device::io_map(address_map &map)
{
	// Undriven bits retain the last driven Z180 bus
	// byte, including instruction fetches and Apple-side SRAM copies.
	map.unmap_value_high();
	map(0x0000, 0xffff).rw(FUNC(a2bus_secondsight_device::io_r), FUNC(a2bus_secondsight_device::io_w));
}

u8 a2bus_secondsight_device::io_r(offs_t port)
{
	// The Z180 I/O decoder sees HA0-HA7 only; Oak has its own full 16-bit port decoder.
	bool driven = false;
	u8 data;
	if ((port & 0xf0) == 0xa0)
	{
		data = port_r(port & 15);
		driven = (port & 15) == 0 || ((port & 15) >= 4 && (port & 15) <= 7);
	}
	else
		data = m_video->io_r(port, &driven);
	if (!driven)
		return m_bus_valid ? m_bus_data : 0xff;
	if (!machine().side_effects_disabled())
		drive_hd(data);
	return data;
}

void a2bus_secondsight_device::io_w(offs_t port, u8 data)
{
	if (machine().side_effects_disabled())
		return;
	drive_hd(data);
	if ((port & 0xf0) == 0xa0)
		port_w(port & 15, data);
	else if ((port & 0xff00) == 0x300)
		vga_w(port & 255, data);
	else
		m_video->io_w(port, data);
}

// The configured board handlers are the only callbacks owned by this worker.
// Watch every descendant memory interface, including the Oak register spaces,
// not just Z180 program and I/O. Keep references so replacement allocations
// cannot accidentally acquire the identity of an original handler.
void a2bus_secondsight_device::watch_worker_spaces()
{
	for (auto &memory : memory_interface_enumerator(*this))
		for (int n = 0; n < memory.max_space_count(); ++n)
			if (memory.has_space(n))
			{
				auto watched = std::make_unique<worker_space>();
				watched->space = &memory.space(n);
				std::vector<memory_entry> reads, writes;
				watched->space->dump_maps(reads, writes);
				for (auto *entry : {watched->space->unmap_r(), watched->space->unmap_w(), watched->space->nop_r(), watched->space->nop_w()})
					if (watched->owned.insert(entry).second) entry->ref();
				for (auto const &map : {std::cref(reads), std::cref(writes)})
					for (auto const &entry : map.get())
						if (!entry.entry->is_passthrough() && watched->owned.insert(entry.entry).second)
							entry.entry->ref();
				watched->watch = watched->space->add_map_change_notifier([this](bool before) { worker_map_change(before); });
				m_worker_spaces.emplace_back(std::move(watched));
			}
	m_map_changes = 1;
	m_inline_callbacks = true;
	worker_map_change(false);
}

void a2bus_secondsight_device::worker_map_change(bool before)
{
	if (before)
	{
		++m_map_changes;
		m_inline_callbacks = true;
		// A tap may remove/reinstall itself during inline execution. That
		// consumer already owns the queue; joining its idle peer would wait
		// for our own token. No worker can execute while we hold that token.
		if (&machine().scheduler() != m_card_scheduler.get())
			stop_worker();
		else
			assert(std::this_thread::get_id() != m_worker.get_id());
		m_idle_memory_safe = false;
		m_cpu->clear_direct_read_pages();
		return;
	}
	if (--m_map_changes)
		return;
	for (auto const &watched : m_worker_spaces)
	{
		std::vector<memory_entry> reads, writes;
		watched->space->dump_maps(reads, writes);
		for (auto const &map : {std::cref(reads), std::cref(writes)})
			for (auto const &entry : map.get())
				if (entry.entry->is_passthrough() || !watched->owned.contains(entry.entry))
					return;
	}
	// Removing the last foreign handler restores the original map. The
	// next asynchronous publication may start the worker again.
	m_inline_callbacks = false;
}

void a2bus_secondsight_device::device_start()
{
	m_logic.power_on();
	set_execution_barrier(*m_cpu);
	watch_worker_spaces();
	m_grant_timer = timer_alloc(FUNC(a2bus_secondsight_device::copy_tick), this);
	m_output_timer = timer_alloc(FUNC(a2bus_secondsight_device::output_complete), this);
	m_output_timer->set_owner(machine().root_device()); // the receiving domain
	m_host_scheduler = &machine().scheduler();
	m_card_scheduler = std::make_unique<device_scheduler>(machine(), *this);
	m_card_scheduler->set_domain_barrier([this](attotime when) { worker_barrier(when); });
	save_item(NAME(m_host_meeting));
	save_item(NAME(m_saved_outputs));
	save_item(NAME(m_saved_output_kind));
	save_item(NAME(m_saved_output_data));
	save_item(NAME(m_saved_output_time));
	machine().save().register_presave(save_prepost_delegate([this]()
	{
		materialize_idle(machine().time());
		// This input cache never runs beyond the next outside timer.
		if (m_idle_inputs.count)
			fatalerror("Second Sight future neutral input at save");
		// The domain presave barrier has drained the transport before this.
		if (m_host_outputs.size() > m_saved_output_kind.size())
			fatalerror("Second Sight output save queue overflow");
		m_saved_outputs = m_host_outputs.size();
		for (unsigned n = 0; n != m_saved_outputs; ++n)
		{
			m_saved_output_kind[n] = u8(m_host_outputs[n].kind);
			m_saved_output_data[n] = m_host_outputs[n].value;
			m_saved_output_time[n] = m_host_outputs[n].when;
		}
	}, "Second Sight output barrier"));
	machine().save().register_postload(save_prepost_delegate([this]()
	{
		m_idle_inputs.head = m_idle_inputs.count = 0;
		m_host_outputs.clear();
		for (unsigned n = 0; n != m_saved_outputs; ++n)
			m_host_outputs.push_back({output_kind(m_saved_output_kind[n]), m_saved_output_data[n], m_saved_output_time[n]});
	}, "Second Sight output restore"));
	// SRAM startup is unspecified. Deterministic zero fill is a convention;
	// subsequent resets preserve it. The normal reset path rewrites firmware
	// areas from EPROM but does not call the unused $0208 signature routine.
	std::fill_n(&m_sram[0], 0x20000, 0);
	save_item(NAME(m_logic.q));
	save_item(NAME(m_logic.clocks));
	save_item(NAME(m_lattice_v2));
	save_item(NAME(m_lattice_chip_configured));
	save_item(NAME(m_int0_pullup));
	save_item(NAME(m_int0_configured));
	save_item(NAME(m_cint));
	save_item(NAME(m_pins.address));
	save_item(NAME(m_pins.data));
	save_item(NAME(m_pins.port));
	save_item(NAME(m_pins.host_data));
	save_item(NAME(m_pins.CPU));
	save_item(NAME(m_pins.M2B0));
	save_item(NAME(m_pins.M2SEL_n));
	save_item(NAME(m_pins.AX));
	save_item(NAME(m_pins.Y0));
	save_item(NAME(m_pins.Y2));
	save_item(NAME(m_pins.read));
	save_item(NAME(m_cpu_pins.address));
	save_item(NAME(m_cpu_pins.data));
	save_item(NAME(m_cpu_pins.read));
	save_item(NAME(m_cpu_pins.M2SEL_n));
	save_item(NAME(m_dma_owned));
	save_item(NAME(m_cpu_bank));
	save_item(NAME(m_apple_data_in));
	save_item(NAME(m_apple_data_out));
	save_item(NAME(m_address_latches));
	save_item(NAME(m_bus_data));
	save_item(NAME(m_bus_valid));
	save_item(NAME(m_port_diagnostics));
	save_item(NAME(m_catch_bank));
	save_item(NAME(m_catch_data));
	save_item(NAME(m_cycle_time));
	save_item(NAME(m_copy.state));
	save_item(NAME(m_copy.pending));
	save_item(NAME(m_copy.deadline));
	save_item(NAME(m_copy_overwrites));
	save_item(NAME(m_copy_under_write));
	save_item(NAME(m_rom_overlay));
	save_item(NAME(m_vga_half));
	save_item(NAME(m_rgb));
	save_item(NAME(m_linear));
	save_item(NAME(m_oak_index));
	save_item(NAME(m_oak_map));
	save_item(NAME(m_bus_request));
	save_item(NAME(m_waiting));
	save_item(NAME(m_external_phases));
	save_item(NAME(m_idle_clock));
	save_item(NAME(m_idle_limit));
	machine().add_notifier(MACHINE_NOTIFY_FRAME, machine_notify_delegate(&a2bus_secondsight_device::frame_clocks, this), true);
	save_item(NAME(m_timed_completion));
	save_item(NAME(m_apple_irq));
	save_item(NAME(m_int1));
	machine().save().register_postload(save_prepost_delegate([this]()
	{
		// The saved fuse-map choice may differ from the current session. The
		// memoized edges are derived data, and must use the restored equations.
		m_logic.edges = {};
		update_direct_reads();
		card_irq(m_apple_irq);
	}, "Second Sight Apple IRQ"));
	save_item(NAME(m_copies));
	save_item(NAME(m_events));
	save_item(NAME(m_stalls));
	save_item(NAME(m_read_stalls));
	save_item(NAME(m_video_waits));
	save_item(NAME(m_video_wait_cycles));
}

void a2bus_secondsight_device::device_reset()
{
	// Configuration is loaded after device_start. Replacing a programmed Lattice chip
	// is a power-off operation, independent of EPROM selection; warm reset
	// must preserve both the selected equations and the existing registers.
	if (!m_lattice_chip_configured)
	{
		m_lattice_v2 = BIT(m_lattice_chip->read(), 0);
		m_lattice_chip_configured = true;
	}
	if (!m_int0_configured)
	{
		m_int0_pullup = BIT(m_int0_mod->read(), 0);
		m_int0_configured = true;
	}
	materialize_idle(machine().time());
	m_idle_limit = m_idle_clock = 0;
	m_idle_inputs.head = m_idle_inputs.count = 0;
	if (&m_cpu->scheduler() == m_card_scheduler.get() && &machine().scheduler() != m_card_scheduler.get())
		worker_barrier(m_host_scheduler->time());
	stop_worker();
	// Physical-phase hosts already synchronize every edge. Their measured
	// fastest path is the original lazy executor in the motherboard domain.
	m_shared_execution = a2bus().has_mega_phases();
	m_cpu->set_grant_continuation(false);
	a2bus().invalidate_mega_phase_permissions();
	auto &scheduler = m_shared_execution ? *m_host_scheduler : *m_card_scheduler;
	if (&m_cpu->scheduler() != &scheduler)
		scheduler.detach(*m_cpu, *this);
	m_host_outputs.clear();
	m_output_timer->adjust(attotime::never);
	m_host_meeting = false;

	m_pins = {};
	m_rom_overlay = true;
	update_direct_reads();
	m_bus_request = m_waiting = false;
	m_copy = {};
	m_dma_owned = false;
	m_external_phases = false;
	m_timed_completion = false;
	m_int1 = -1;
	m_oak_index = m_oak_map = 0;
	m_port_diagnostics = 0;
	m_grant_timer->adjust(attotime::never);
	m_cpu->bus_release_w(false);
	// Schematic sheet 2: the Z180 I/O and memory decoders have no ARES input.
	// Preserve their outputs;
	// this is an inference for the original board, whose PLDs remain undumped.
	// Member initializers supply deterministic cold-start values only.
	card_rdy(true);
	// The Z180 I/O decoder has no Apple-side IRQ acknowledge. Preserve its output across
	// Apple reset; the restarted Z180 firmware releases /IRQ through AD.
	card_irq(m_apple_irq);
	// Schematic sheet 1: Lattice chip pin 35 is unused Y1. Preserve its state.
	// Resetting the emulated input snapshot must not create latch clock edges.
	settle(false);
	// Prime the preceding CPU-high phase for the next multiplexed bank window.
	m_pins.CPU = true;
	settle(false);
	clock_7m(false);
	m_pins.CPU = false;
	settle(false);
	drive_interrupt();
}

void a2bus_secondsight_device::drive_hd(u8 data)
{
	if (m_oak_scan_test)
		m_video->system_data_w(data);
	m_bus_data = data;
	if (!m_bus_valid)
	{
		m_bus_valid = true;
		// Seed the hold before certifying direct reads: every subsequent
		// fast read drives the already-valid byte with no extra hot-path work.
		update_direct_reads();
	}
}

void a2bus_secondsight_device::cint_w(int state)
{
	// Schematic: the /INT0 net joins only Z180 pin 10 and Oak CINT (pin 90),
	// with no pull-up, so on a stock card CINT never reaches INT0.
	m_cint = bool(state);
	m_cpu->set_input_line(INPUT_LINE_IRQ0, m_int0_pullup && m_cint ? ASSERT_LINE : CLEAR_LINE);
}

IRQ_CALLBACK_MEMBER(a2bus_secondsight_device::irq0_ack)
{
	// Schematic sheets1/2: M1 is unconnected; /RD is inactive on ACK,
	// so no ordinary ROM/SRAM/Apple data-in latch read is issued. Indefinite digital hold
	// and cold FF are model conventions, not measured decay or pull-ups.
	materialize_idle(machine().time());
	return m_bus_valid ? m_bus_data : 0xff;
}

void a2bus_secondsight_device::update_direct_reads()
{
	// Cached decode of the same Z180 memory decoder ROM/SRAM regions in decode_board.
	// Reads still occur at the CPU's actual sample edge and drive m_bus_data.
	// Bank strobes and VGA retain their handlers, waits and side effects.
	m_cpu->clear_direct_read_pages();
	if (!m_idle_memory_safe || !m_bus_valid || m_oak_scan_test)
		return;
	for (unsigned page = 0; page < 4; ++page)
		m_cpu->set_direct_read_page(page, (m_rom_overlay && page < 2 ? &m_rom[0] : &m_sram[0]) + ((page & 1) << 16), m_bus_data);
	for (unsigned page = 6; page < 8; ++page)
		m_cpu->set_direct_read_page(page, &m_rom[(page & 1) << 16], m_bus_data);
}

void a2bus_secondsight_device::reset_from_bus()
{
	if (m_shared_execution)
	{
		reset();
		return;
	}
	worker_barrier(machine().time());
	stop_worker();
	device_scheduler::domain_scope scope(*m_card_scheduler);
	reset(); // device_t also resets the Z180 and video children
}

// Original board photographs suggest the Z180 I/O decoder is a reworked GAL16V8A;
// the Z180 memory decoder is a red-dot AMI 18CV8-10; the U12 AppleColor RGB
// timing GAL is an AMI 18CV8-10 by R17; the copy controller is an 18CV8-15 by R1/R16.
// They remain undumped. Keep their confirmed behavioral decode here rather
// than spreading assumed equations among access handlers. The Lattice chip
// is separate, generated from the decoded BE8C fuse map.
a2bus_secondsight_device::select a2bus_secondsight_device::decode_board(offs_t address, bool io, bool write) const
{
	if (io)
	{
		if (address == 0xa0 || address == 0xa1 || (address >= 0xa4 && address <= 0xa7))
			return select::lattice;
		if (address == 0xa8 || address == 0xa9)
			return select::rgb;
		// ROM_BEEF AA/AB accesses imply LA20 control through Z180 memory decoder; HA19 gates
		// both high Oak address bits to the Z180 video window.
		if (address == 0xaa || address == 0xab)
			return select::linear;
		// The Z180 I/O decoder even/odd pairs drive low/high on either access direction.
		// AIRQ0 passes through the U8 74F125 buffer: AC asserts Apple /IRQ, AD releases it.
		if (address == 0xac || address == 0xad)
			return select::irq;
		// A1-A3 reads are undriven. A2/A3 and AE/AF have no decoded effect.
		if (address >= 0x300 && address <= 0x3ff)
			return select::vga;
		return select::none;
	}
	// Source: schematic sheet 2 and the ROM_BEEF reset/DMA copy.
	if (address < 0x20000 && m_rom_overlay)
		return select::rom;
	if (address < 0x40000)
		return select::sram;
	// The Z180 memory decoder has no RD/WR input; either direction switches LA19.
	if (address < 0x60000)
		return select::bank;
	if (address < 0x80000)
		return select::rom;
	return select::vga;
}

u8 a2bus_secondsight_device::memory_r(offs_t address)
{
	// Schematic sheet 2: U18 Z180 pin 33 connects to HA18 at the Z180 memory
	// decoder and Oak OTI-087X VGA chip. TOUT replaces
	// A18 for instruction, data and DMA memory cycles when TCR.TOC != 00.
	address = m_cpu->a18_tout_address(address);
	u8 data = m_bus_data;
	auto const target = decode_board(address, false, false);
	bool driven = target == select::rom || target == select::sram;
	switch (target)
	{
	case select::rom:
		data = m_rom[address & 0x1ffff];
		break; // HA16 reaches EPROM A16
	case select::sram:
		data = m_sram[address & 0x1ffff];
		break;
	case select::bank:
		if (!machine().side_effects_disabled())
			m_vga_half = BIT(address, 16);
		break; // The Z180 memory decoder changes A19 but nobody drives the data bus
	case select::vga:
		video_access(false);
		data = m_video->memory_r(oak_address(address), &driven);
		if (!driven)
			data = m_bus_valid ? m_bus_data : 0xff;
		break;
	default:
		break;
	}
	if (!machine().side_effects_disabled() && driven)
		drive_hd(data);
	return data;
}

void a2bus_secondsight_device::memory_w(offs_t address, u8 data)
{
	if (machine().side_effects_disabled())
		return;
	address = m_cpu->a18_tout_address(address);
	drive_hd(data);
	switch (decode_board(address, false, true))
	{
	case select::rom:
		port_diagnostic(6, "EPROM write contention: ignored write; retained CPU byte is an undefined-level convention");
		break;
	case select::sram:
		m_sram[address & 0x1ffff] = data;
		break;
	case select::bank:
		m_vga_half = BIT(address, 16);
		break;
	case select::vga:
		video_access(true);
		m_video->memory_w(oak_address(address), data);
		break;
	default:
		break;
	}
}

void a2bus_secondsight_device::port_diagnostic(unsigned bit, char const *reason)
{
	if (!BIT(m_port_diagnostics, bit))
	{
		m_port_diagnostics |= 1 << bit;
		LOGMASKED(LOG_DIAGNOSTIC, "Second Sight: board model: %s bus=%02x time=%s (first occurrence this reset)\n", reason, m_bus_data,
			machine().time().as_string());
	}
}

u8 a2bus_secondsight_device::vga_r(offs_t port)
{
	return m_video->io_r(0x300 | port);
}

void a2bus_secondsight_device::vga_w(offs_t port, u8 data)
{
	if (machine().side_effects_disabled())
		return;
	if (port == 0xde)
		m_oak_index = data;
	if (port == 0xdf && m_oak_index == 5)
	{
		if (data == 0x19 && !m_linear)
			LOGMASKED(LOG_DIAGNOSTIC, "Second Sight: pairing mismatch: Oak OTI-087X VGA chip map=19 without preceding AB\n");
		m_oak_map = data;
	}
	m_video->io_w(0x300 | port, data);
}

void a2bus_secondsight_device::linear_access(bool enabled)
{
	// Firmware AB -> ext5=19; ext5=00 -> AA, including AA read at reset.
	// Do not make the flag follow the Oak register: they are separate latches.
	if (!enabled && m_oak_map != 0)
		LOGMASKED(LOG_DIAGNOSTIC, "Second Sight: pairing mismatch: AA with Oak OTI-087X VGA chip map=%02x\n", m_oak_map);
	if (enabled && m_oak_map != 0)
		LOGMASKED(LOG_DIAGNOSTIC, "Second Sight: pairing mismatch: AB with Oak OTI-087X VGA chip map=%02x\n", m_oak_map);
	m_linear = enabled;
}

void a2bus_secondsight_device::video_access(bool write)
{
	// The exact engine queried U15 Z180 memory decoder and Oak at T1,
	// before this data edge. There is no second charge after the transfer.
	if (!BIT(m_port_diagnostics, 5) && m_cpu->clocks_to_cycles(m_cpu->clock()) / 2 > 18'000'000)
		port_diagnostic(5,
			"CCR 36 MHz violates Oak OTI-087X VGA chip 25 ns setup; unspecified electrical result, functional transport retained by convention");
}

u32 a2bus_secondsight_device::oak_address(offs_t address)
{
	if (!machine().side_effects_disabled() && m_linear != bool(BIT(m_oak_map, 0)))
		LOGMASKED(
			LOG_DIAGNOSTIC, "Second Sight: pairing mismatch: VGA access %05x with LA20=%d Oak OTI-087X VGA chip map=%02x\n", address, m_linear, m_oak_map);
	// Only HA19-high accesses reach this handler. Routing SRAM/EPROM locally
	// is equivalent to Z180 memory decoder gating LA20=linear&HA19 and LA19=half&HA19.
	return (m_linear ? 0x100000 : 0) | (m_vga_half ? 0x80000 : 0) | (address & 0x7ffff);
}

void a2bus_secondsight_device::settle(bool latch_outputs)
{
	if (m_shared_execution)
		a2bus().invalidate_mega_phase_permissions();
	const bool adin = m_logic.q[secondsight_lattice::ADIN_ACTIVE];
	const bool caught = m_logic.q[secondsight_lattice::CATCH_ACTIVE];
	const bool event = m_logic.q[secondsight_lattice::EVENT_LATCH];
	if (!(m_lattice_v2 ? m_logic.settle(m_pins) : m_logic.secondsight_lattice::settle(m_pins)))
		fatalerror("Second Sight Lattice did not settle");
	if (!latch_outputs)
		return;
	// Physical ADIN/CATCH pins are active low. Their rising edges clock the
	// HCT574s. U17 copy controller has no RUN/ADIN/STALL input. CATCH
	// changes the address even when RUN blocks ADIN; its busy-edge handling
	// is the named approximation in secondsight_copy.h. A failed held phase
	// can therefore pair a new address with the old U2 Apple data-in byte.
	if (adin && !m_logic.q[secondsight_lattice::ADIN_ACTIVE])
		m_apple_data_in = m_pins.data;
	if (caught && !m_logic.q[secondsight_lattice::CATCH_ACTIVE])
	{
		m_address_latches = m_pins.address;
		m_catch_bank = m_logic.q[secondsight_lattice::BANK_Q];
		m_catch_data = m_apple_data_in;
		start_catch();
	}
	if (!event && m_logic.q[secondsight_lattice::EVENT_LATCH])
		++m_events;
}

void a2bus_secondsight_device::clock_7m(bool latch_outputs)
{
	// Schematic: the Lattice chip Y2 is 7M; the selected Y0 model uses the same clock.
	m_pins.Y0 = m_pins.Y2 = false;
	settle(latch_outputs);
	m_pins.Y0 = m_pins.Y2 = true;
	settle(latch_outputs);
}

void a2bus_secondsight_device::drive_interrupt()
{
	const int state = m_logic.q[secondsight_lattice::IRQ_PENDING] ? ASSERT_LINE : CLEAR_LINE;
	if (state == m_int1)
		return;
	m_int1 = state;
	// Sheet 2: INT1 is wired to BOTH CPU INT1 and DMA DREQ1.
	m_cpu->set_input_line(INPUT_LINE_IRQ1, state);
	m_cpu->set_input_line(Z180_INPUT_LINE_DREQ1, state);
}

bool a2bus_secondsight_device::can_defer_mega_idle() const
{
	return m_shared_execution && m_external_phases && !m_dma_owned && !m_copy.active() && !m_waiting &&
		m_idle_inputs.last(m_pins) == secondsight_idle_queue::pack(0, 0, true) && secondsight_idle::quiet(m_logic, m_pins);
}

bool a2bus_secondsight_device::can_defer_mega_read(u16 address, u8 data) const
{
	return m_shared_execution && m_external_phases && !m_dma_owned && !m_copy.active() && !m_waiting &&
		m_idle_inputs.last(m_pins) == secondsight_idle_queue::pack(address, data, false) && secondsight_idle::quiet(m_logic, m_pins);
}

bool a2bus_secondsight_device::can_defer_mega_transition(u16 address, bool read) const
{
	return m_shared_execution && m_external_phases && !m_dma_owned && !m_copy.active() && !m_waiting && read &&
		secondsight_idle::transition_address(address) && m_idle_inputs.room() && secondsight_idle::quiet(m_logic, m_pins);
}

void a2bus_secondsight_device::defer_mega_transition(u64 clock, u16 address, u8 data, bool unselected)
{
	m_idle_inputs.push(clock, secondsight_idle_queue::pack(address, data, unselected), m_pins);
	m_idle_limit = clock;
}

void a2bus_secondsight_device::materialize_idle(attotime when)
{
	if (m_idle_limit <= m_idle_clock)
		return;
	// The 14M boundary is ceil(n * 1056 / 15120000000 seconds). This inverse
	// floors completed boundaries, exactly as the host's finer TWGS grid.
	const u64 ticks =
		u64(when.seconds()) * 15'120'000'000ULL + u64((__uint128_t(when.attoseconds()) * 15'120'000'000ULL) / 1'000'000'000'000'000'000ULL);
	const u64 end = std::min(m_idle_limit, ticks / 1056);
	if (end <= m_idle_clock)
		return;
	// Input edges must use the selected fuse map, including when an elapsed
	// neutral span is materialized from the TWGS cache.
	const bool settled = m_lattice_v2 ? m_idle_inputs.advance(m_logic, m_pins, m_idle_clock, end) :
										  m_idle_inputs.advance(static_cast<secondsight_lattice &>(m_logic), m_pins, m_idle_clock, end);
	if (!settled)
		fatalerror("Second Sight deferred equations did not settle");
	const u64 tick = end * 1056, fraction = tick % 15'120'000'000ULL;
	m_cycle_time = attotime(tick / 15'120'000'000ULL,
		fraction ? u64((__uint128_t(fraction) * 1'000'000'000'000'000'000ULL + 15'120'000'000ULL - 1) / 15'120'000'000ULL) : 0);
}

bool a2bus_secondsight_device::synchronize_mega(attotime when)
{
	if (m_shared_execution)
		return m_host_scheduler->synchronize_before(*m_cpu, when);
	// The timestamped phase immediately following this call is the grant.
	// Its producer does not touch any private card state.
	return true;
}

bool a2bus_secondsight_device::synchronize_mega_irq(attotime when)
{
	if (m_shared_execution)
		return m_host_scheduler->synchronize_device(*m_cpu, when) && m_host_scheduler->first_timer()->expire() > when;
	if (machine().scheduler().first_timer()->expire() <= when)
		return false;
	work_item item;
	item.kind = work_kind::grant;
	item.cycle.time = when;
	item.state = true;
	item.fence = ++m_fence;
	wait_worker(item.fence, enqueue(item));
	publish_outputs();
	return m_response_time >= when && machine().scheduler().first_timer()->expire() > when;
}

bool a2bus_secondsight_device::observe_mega_cycle(a2bus_mega_cycle const &cycle, u8 &data)
{
	if (m_shared_execution)
	{
		materialize_idle(cycle.time);
		return process_cycle(cycle, data);
	}
	work_item item;
	item.kind = work_kind::phase;
	item.cycle = cycle;
	// Superset of Lattice chip EVENT_DECODE and ADOUT_ACTIVE for either MODE_D3 and
	// STORE80 value. Every input edge still reaches the full equations on the
	// worker. Only addresses that cannot affect the host avoid a rendezvous;
	// an outstanding event keeps subsequent clocks synchronous until ACK.
	const bool meeting = m_host_meeting || secondsight_idle::host_meeting_address(cycle.address, cycle.read, m_lattice_v2);
	if (meeting)
		item.fence = ++m_fence;
	const bool completed_inline = enqueue(item);
	if (!meeting)
		return false;
	wait_worker(item.fence, completed_inline);
	publish_outputs();
	m_host_meeting = m_response_meeting;
	data = m_response_data;
	return m_response_drive;
}

void a2bus_secondsight_device::mega_dma_w(bool active)
{
	if (m_shared_execution)
	{
		process_dma(active);
		return;
	}
	work_item item;
	item.kind = work_kind::dma;
	item.cycle.time = machine().time();
	item.state = active;
	item.fence = ++m_fence;
	wait_worker(item.fence, enqueue(item));
	publish_outputs();
}

bool a2bus_secondsight_device::process_cycle(a2bus_mega_cycle const &cycle, u8 &data)
{
	// The motherboard synchronizes the card before delivering this physical edge.
	if (cycle.phased)
		return observe_mega_phase(cycle, data);
	m_cycle_time = cycle.time;
	if (!m_dma_owned)
		m_timed_completion = cycle.timed;
	advance_copy(cycle.time);
	if (cycle.timed && !cycle.ph0)
	{
		const bool drive = m_logic.q[secondsight_lattice::ADOUT_ACTIVE];
		if (drive)
			data = m_apple_data_out;
		finish_cycle();
		return drive;
	}
	if (cycle.retry)
	{
		m_pins.CPU = true;
		settle();
		clock_7m();
		const bool waiting = m_logic.q[secondsight_lattice::STALL];
		if (waiting != m_waiting)
		{
			m_waiting = waiting;
			card_rdy(!waiting);
		}
		const bool drive = m_logic.q[secondsight_lattice::ADOUT_ACTIVE];
		if (drive)
			data = m_apple_data_out;
		return drive;
	}
	if (m_waiting && !m_dma_owned)
		fatalerror("Second Sight: another bus transfer while RDY was low: held=%04x new=%04x read=%d time=%s", m_pins.address,
			cycle.address, cycle.read, cycle.time.as_string());
	// Evaluate the 7M edges needed to open/close BANK_WINDOW and sample the
	// data phase. Additional edges with stable inputs leave the state unchanged.
	m_pins.address = cycle.address;
	m_pins.data = cycle.data;
	m_pins.read = cycle.read;
	m_pins.M2SEL_n = cycle.m2sel_n;
	m_pins.CPU = false;
	m_pins.M2B0 = cycle.m2b0_valid && cycle.m2b0;
	settle();
	clock_7m();
	clock_7m();
	m_pins.M2B0 = false;
	settle(); // board pull-down outside bank window
	m_pins.CPU = true;
	settle();
	clock_7m();
	const bool drive = m_logic.q[secondsight_lattice::ADOUT_ACTIVE];
	if (drive)
		data = m_apple_data_out;
	if (m_logic.q[secondsight_lattice::STALL])
	{
		// RDY cannot hold an independent DMA master. Its writes still clock the copy controller.
		if (!m_dma_owned)
		{
			m_waiting = true;
			++m_stalls;
			card_rdy(false);
			if (cycle.read)
				++m_read_stalls;
		}
		if (!cycle.timed)
			finish_cycle();
		return drive;
	}
	if (!cycle.timed)
		finish_cycle();
	return drive;
}

void a2bus_secondsight_device::process_dma(bool active)
{
	if (m_shared_execution)
		a2bus().invalidate_mega_phase_permissions();
	materialize_idle(machine().time());
	m_dma_owned = active;
	if (active)
	{
		// Retain only the CPU-side input pins. The Lattice chip, Apple data-in/out latches, address
		// latches and SRAM remain live and may change during DMA; none of them is rolled back.
		m_cpu_pins.address = m_pins.address;
		m_cpu_pins.data = m_pins.data;
		m_cpu_pins.read = m_pins.read;
		m_cpu_pins.M2SEL_n = m_pins.M2SEL_n;
		m_cpu_bank = m_logic.q[secondsight_lattice::BANK_HELD];
		return;
	}
	if (!m_waiting)
		return;
	// Re-present the held CPU address in the low-phase bank window. The
	// phase-aware host supplies the next PH0 high itself; the completed-
	// transfer host retries it here, just as after a card acknowledgement.
	m_cycle_time = std::max(m_cycle_time, machine().time());
	m_pins.address = m_cpu_pins.address;
	m_pins.data = m_cpu_pins.data;
	m_pins.read = m_cpu_pins.read;
	m_pins.M2SEL_n = m_cpu_pins.M2SEL_n;
	m_pins.CPU = false;
	m_pins.M2B0 = m_cpu_bank;
	settle();
	clock_7m();
	clock_7m();
	m_pins.M2B0 = false;
	settle();
	if (m_external_phases || m_timed_completion)
		return;
	m_pins.CPU = true;
	settle();
	clock_7m();
	if (!m_logic.q[secondsight_lattice::STALL])
	{
		m_waiting = false;
		if (m_pins.read && m_logic.q[secondsight_lattice::ADOUT_ACTIVE])
			card_read(m_apple_data_out);
	}
	finish_cycle();
	if (!m_waiting)
		card_rdy(true);
}

bool a2bus_secondsight_device::observe_mega_phase(a2bus_mega_cycle const &cycle, u8 &data)
{
	m_external_phases = true;
	m_idle_clock = m_idle_limit = (u64(cycle.time.seconds()) * 15'120'000'000ULL +
									  u64((__uint128_t(cycle.time.attoseconds()) * 15'120'000'000ULL) / 1'000'000'000'000'000'000ULL)) /
		1056;
	m_cycle_time = cycle.time;
	advance_copy(cycle.time);
	m_pins.address = cycle.address;
	m_pins.data = cycle.data;
	m_pins.read = cycle.read;
	m_pins.M2SEL_n = cycle.m2sel_n;
	m_pins.M2B0 = cycle.m2b0_valid && cycle.m2b0;
	m_pins.CPU = cycle.ph0;
	m_pins.Y0 = m_pins.Y2 = cycle.m7;
	settle();
	drive_interrupt();
	const bool waiting = m_logic.q[secondsight_lattice::STALL];
	if (waiting != m_waiting)
	{
		LOGMASKED(LOG_BUS, "Second Sight RDY address=%04x read=%d held=%d copies=%llu events=%llu bus_time=%s\n", cycle.address, cycle.read,
			waiting, (unsigned long long)m_copies, (unsigned long long)m_events, cycle.time.as_string());
		m_waiting = waiting;
		if (waiting)
		{
			++m_stalls;
			if (cycle.read)
				++m_read_stalls;
		}
		card_rdy(!waiting);
	}
	const bool drive = m_logic.q[secondsight_lattice::ADOUT_ACTIVE];
	if (drive)
		data = m_apple_data_out;
	return drive;
}

void a2bus_secondsight_device::finish_cycle()
{
	m_pins.CPU = false;
	settle(); // rising ADIN_n and CATCH_n, falling CAPTURE & CPU
	drive_interrupt();
}

void a2bus_secondsight_device::acknowledge()
{
	// Lattice chip G07.Q1: either direction at AX/HA=0 acknowledges INT1/DREQ1.
	settle();
	drive_interrupt();
	m_pins.AX = false;
	settle();
	// The disconnected CPU cannot sample the Apple data-out latch or resume its held access.
	// Acknowledgement still updates the live Lattice while DMA owns the bus.
	if (m_dma_owned)
		return;
	if (m_external_phases || m_timed_completion)
	{
		// The Apple data-out latch changes now; TWGS accepts its live value only at the eventual
		// selected GS_PH2 fall. The next real PH0 high releases STALL.
		if (m_pins.read && m_logic.q[secondsight_lattice::ADOUT_ACTIVE])
			card_read(m_apple_data_out);
		return;
	}
	if (m_waiting)
	{
		m_pins.CPU = false;
		settle();
		m_pins.CPU = true;
		settle();
		clock_7m();
		if (m_logic.q[secondsight_lattice::STALL])
			return;
		m_waiting = false;
		m_cycle_time = std::max(m_cycle_time, machine().time());
		if (m_pins.read && m_logic.q[secondsight_lattice::ADOUT_ACTIVE])
			card_read(m_apple_data_out);
		if (!m_timed_completion)
			finish_cycle();
		card_rdy(true);
	}
}

u8 a2bus_secondsight_device::port_r(offs_t port)
{
	materialize_idle(machine().time());
	if (machine().side_effects_disabled())
	{
		if (port == 0)
			return m_apple_data_in;
		return m_bus_data;
	}
	if (port >= 1 && port <= 3)
		port_diagnostic(port - 1, "A1-A3 are undriven; reads retain the preceding bus byte");
	if (port >= 14)
		port_diagnostic(3, "AE/AF are undriven and have no strobe");
	const select target = decode_board(0xa0 + port, true, false);
	if (target != select::lattice)
	{
		// The Z180 I/O decoder pairs are strobes on either access direction.
		if (target != select::none)
			port_w(port, m_bus_data);
		return m_bus_data;
	}
	m_pins.port = port;
	m_pins.host_data = m_bus_data;
	// Let the data-mux hold settle BEFORE the read-to-clear strobe.
	settle();
	m_pins.AX = true;
	settle();
	u8 data = port == 0 ? m_apple_data_in : m_bus_data;
	if (port >= 4)
		data = (m_bus_data & 0xf0) | (m_logic.q[secondsight_lattice::HD0_DRIVE] ? 1 : 0) |
			(m_logic.q[secondsight_lattice::HD1_DRIVE] ? 2 : 0) | (m_logic.q[secondsight_lattice::HD2_DRIVE] ? 4 : 0) |
			(m_logic.q[secondsight_lattice::HD3_DRIVE] ? 8 : 0);
	if (port == 0)
		acknowledge();
	else
	{
		m_pins.AX = false;
		settle();
	}
	return data;
}

void a2bus_secondsight_device::port_w(offs_t port, u8 data)
{
	materialize_idle(machine().time());
	if (machine().side_effects_disabled())
		return;
	const select target = decode_board(0xa0 + port, true, true);
	if (target == select::lattice)
	{
		// Lattice chip has no R/W input. A4-A7 writes still read/clear the flags;
		// its low-nibble outputs contend with the CPU and store no write data.
		if (port >= 4)
		{
			port_diagnostic(7, "status-write contention: low nibble follows Lattice chip by convention; electrical winner is undefined");
			drive_hd(port_r(port));
			return;
		}
		m_pins.port = port;
		m_pins.host_data = data;
		settle();
		m_pins.AX = true;
		settle();
		if (port == 0)
		{
			m_apple_data_out = data;
			acknowledge();
		}
		else
		{
			m_pins.AX = false;
			settle();
		}
	}
	else if (target == select::linear)
		linear_access(BIT(port, 0));
	else if (target == select::rgb)
	{
		m_rgb = BIT(port, 0); // The Z180 I/O decoder/AppleColor RGB timing GAL divider, separate from cable/monitor sense
		m_video->output_divider_w(m_rgb);
	}
	else if (target == select::irq)
	{
		m_apple_irq = !BIT(port, 0);
		if (!BIT(port, 0))
			port_diagnostic(4, "AC drives AIRQ0 low and asserts Apple IRQ");
		card_irq(m_apple_irq);
	}
	else if (port >= 14)
		port_diagnostic(3, "AE/AF are undriven and have no strobe");
}

void a2bus_secondsight_device::schedule_copy()
{
	m_grant_timer->adjust(m_copy.deadline == attotime::never ? attotime::never :
		std::max(attotime::zero, m_copy.deadline - machine().time()));
}

void a2bus_secondsight_device::start_catch()
{
	if (m_shared_execution)
		a2bus().invalidate_mega_phase_permissions();
	LOGMASKED(LOG_BUS, "Second Sight CATCH address=%04x data=%02x bank=%d read=%d held=%d time=%.18f\n", m_address_latches, m_catch_data, m_catch_bank,
		m_pins.read, m_waiting, m_cycle_time.as_double());
	if (m_copy.pending)
		++m_copy_overwrites;
	if (m_copy.state == secondsight_copy_controller::phase::write_end)
		++m_copy_under_write;
	// Every rising CATCH has already overwritten U5/U6 address latches.
	// U2 Apple data-in latch changes only on ADIN (gated by RUN).
	// U17 copy controller cannot preserve the old address or byte.
	m_copy.capture(m_cpu->next_phi_edge(m_cycle_time), m_cpu->phi_time(1));
	schedule_copy();
}

void a2bus_secondsight_device::advance_copy(attotime when)
{
	if (m_copy.deadline > when)
		return;
	while (m_copy.deadline <= when)
	{
		if (m_shared_execution)
			a2bus().invalidate_mega_phase_permissions();
		attotime const edge = m_copy.deadline;
		switch (m_copy.tick(m_cpu->next_phi_edge(edge), m_cpu->phi_time(1)))
		{
		case secondsight_copy_controller::action::request:
			m_bus_request = true;
			m_cpu->bus_release_w(true);
			if (m_cpu->bus_acknowledged())
				m_copy.grant(m_cpu->next_phi_edge(edge), m_cpu->phi_time(1));
			break;
		case secondsight_copy_controller::action::write:
			// Commit at /WR assertion, using the live latches, not a queued
			// capture. See retain_write_at_start for the undefined pulse case.
			m_sram[(m_catch_bank ? 0x10000 : 0) | m_address_latches] = m_apple_data_in;
			drive_hd(m_apple_data_in);
			++m_copies;
			// Convention (07-glue-logic, "U15: memory decode"): U28 Oak VGA
			// chip sees MWR, but HA16-HA19 float during copies. Working cards
			// establish that this strobe does not select/write video memory.
			break;
		case secondsight_copy_controller::action::release:
			m_bus_request = false;
			m_cpu->bus_release_w(false);
			break;
		}
	}
	schedule_copy();
}

TIMER_CALLBACK_MEMBER(a2bus_secondsight_device::copy_tick)
{
	advance_copy(machine().time());
}

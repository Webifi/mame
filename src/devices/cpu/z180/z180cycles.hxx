// license:BSD-3-Clause
// Z180 machine-cycle execution, UM0050 Tables 51-54 and Figs 9-18,32-49.
// Instruction handlers remain the only implementation of ISA arithmetic.
// A suspended handler replays completed reads from a bounded journal; completed
// writes and peripheral reads are never issued twice. DMA/peripherals are not
// rolled back. The uninterrupted path calls the handler just once.

// As in the 65816 continuation path, these calls cross only trivial CPU
// frames after callbacks return. The host signal mask never changes here;
// Darwin's plain context form avoids a system call at every bus deadline.
#if defined(__APPLE__)
#define Z180_CONTINUATION_SAVE(B) _setjmp(B)
#define Z180_CONTINUATION_RESTORE(B) _longjmp(B, 1)
#else
#define Z180_CONTINUATION_SAVE(B) setjmp(B)
#define Z180_CONTINUATION_RESTORE(B) std::longjmp(B, 1)
#endif

void z180_device::cycle_update_direct_reads()
{
	m_instruction_fast = false;
	m_replay_pristine = false;
	if (!m_direct_read_bus)
	{
		m_direct_logical.fill(nullptr);
		return;
	}
	for (unsigned page = 0; page < 16; ++page)
	{
		offs_t const physical = a18_tout_address(m_mmu[page]);
		auto const base = m_direct_read[(physical >> 16) & 15];
		m_direct_logical[page] = base ? base + (physical & 0xffff) : nullptr;
	}
}

void z180_device::cycle_save_state()
{
	PAIR const *r[] = {&m_PC, &m_SP, &m_AF, &m_BC, &m_DE, &m_HL, &m_IX, &m_IY, &m_AF2, &m_BC2, &m_DE2, &m_HL2};
	for (unsigned n = 0; n < 12; ++n)
		m_instruction_start.regs[n] = r[n]->d;
	u8 const c[] = {m_R, m_R2, m_IFF1, m_IFF2, m_HALT, m_IM, m_I, m_after_EI, 0};
	std::copy(std::begin(c), std::end(c), m_instruction_start.control);
}

void z180_device::cycle_restore_state()
{
	PAIR *r[] = {&m_PC, &m_SP, &m_AF, &m_BC, &m_DE, &m_HL, &m_IX, &m_IY, &m_AF2, &m_BC2, &m_DE2, &m_HL2};
	for (unsigned n = 0; n < 12; ++n)
		r[n]->d = m_instruction_start.regs[n];
	u8 *c[] = {&m_R, &m_R2, &m_IFF1, &m_IFF2, &m_HALT, &m_IM, &m_I, &m_after_EI};
	for (unsigned n = 0; n < 8; ++n)
		*c[n] = m_instruction_start.control[n];
	_PPC = m_instruction_start.regs[0];
	m_cursor = m_bus_number = m_nominal = 0;
	m_family = Z180_TABLE_op;
	m_opcode = 0;
	m_extra_cycles = 0;
	m_prelude_schedule = 0;
	m_trapping = m_third_trap = false;
}

void z180_device::cycle_reset()
{
	m_cycle = {};
	m_instruction_fast = m_instruction_whole = m_instruction_active = m_cycle_executing = false;
	m_replay_pristine = false;
	m_fast_irq_valid = false;
	m_prelude_schedule = 0;
	m_history_count = 0;
	m_cursor = m_bus_number = m_nominal = 0;
	m_wait_line = m_bus_acknowledged = m_sampled_bus_request = false;
	m_boundary_allowed = true;
	m_dma_allowed = m_dma_start_pending = false;
	m_refresh_pending = m_refresh_after_sleep = false;
	m_refresh_phase = m_phi_half = m_advance_left = m_refresh_address = 0;
	m_half_phi = m_io_service_stopped = m_retired = m_fast_instructions = m_resumptions = 0;
	m_quiet_phi = 0;
	m_quiet_record = 0;
	m_quiet_no_arbitration = false;
	m_dma_channel = m_dma_continue = 0xff;
	m_dma_stage = m_dma_byte = 0;
	m_dma_edge[0] = m_dma_edge[1] = false;
	m_sampled_iol = 0;
	for (unsigned n = 0; n < 2; ++n)
		m_dma_edge_generation[n] = m_dma_edge_accepted[n] = m_dma_edge_sampled[n] = 0;
	m_instruction_interrupt = m_sampled_interrupt = -1;
	m_ack_vector = -1;
	m_interrupt_recovery = m_dma_nmi = false;
	m_power_wake_left = 0;
	m_power_bus_only = false;
}

void z180_device::cycle_register_save()
{
	save_item(NAME(m_cycle_reference));
	save_item(NAME(m_instruction_active));
	save_item(NAME(m_wait_line));
	save_item(NAME(m_bus_acknowledged));
	save_item(NAME(m_sampled_bus_request));
	save_item(NAME(m_dma_start_pending));
	save_item(NAME(m_boundary_allowed));
	save_item(NAME(m_dma_allowed));
	save_item(NAME(m_refresh_pending));
	save_item(NAME(m_refresh_after_sleep));
	save_item(NAME(m_dma_edge));
	save_item(NAME(m_phi_half));
	save_item(NAME(m_refresh_address));
	save_item(NAME(m_sampled_iol));
	save_item(NAME(m_dma_edge_generation));
	save_item(NAME(m_dma_edge_accepted));
	save_item(NAME(m_dma_edge_sampled));
	save_item(NAME(m_refresh_phase));
	save_item(NAME(m_advance_left));
	save_item(NAME(m_half_phi));
	save_item(NAME(m_io_service_stopped));
	save_item(NAME(m_power_wake_left));
	save_item(NAME(m_power_bus_only));
	save_item(NAME(m_retired));
	save_item(NAME(m_fast_instructions));
	save_item(NAME(m_resumptions));
	save_item(NAME(m_history_count));
	save_item(NAME(m_reti_notified));
	save_item(NAME(m_trap_started));
	save_item(NAME(m_interrupt_started));
	save_item(NAME(m_sampled_interrupt));
	save_item(NAME(m_interrupt_recovery));
	save_item(NAME(m_dma_nmi));
	save_item(NAME(m_instruction_interrupt));
	save_item(NAME(m_ack_vector));
	save_item(NAME(m_instruction_start.regs));
	save_item(NAME(m_instruction_start.control));
	save_item(NAME(reinterpret_cast<u8 &>(m_cycle.kind)));
	save_item(NAME(reinterpret_cast<u8 &>(m_cycle.owner)));
	save_item(NAME(m_cycle.address));
	save_item(NAME(m_cycle.data));
	save_item(NAME(m_cycle.phase));
	save_item(NAME(m_cycle.waits));
	save_item(NAME(m_cycle.elapsed));
	save_item(NAME(m_cycle.nominal));
	save_item(NAME(m_cycle.active));
	save_item(NAME(m_cycle.sampled));
	save_item(NAME(m_cycle.no_arbitration));
	save_item(NAME(m_cycle.trailing));
	save_item(NAME(m_cycle.ttp));
	save_item(NAME(m_cycle.io_select_edge));
	save_item(NAME(m_cycle.internal_io));
	save_item(NAME(m_cycle.ack_selected));
	for (unsigned n = 0; n < std::size(m_history); ++n)
	{
		save_item(NAME(reinterpret_cast<u8 &>(m_history[n].kind)), n);
		save_item(NAME(m_history[n].address), n);
		save_item(NAME(m_history[n].data), n);
		save_item(NAME(m_history[n].nominal), n);
	}
	save_item(NAME(m_dma_continue));
	save_item(NAME(m_dma_channel));
	save_item(NAME(m_dma_stage));
	save_item(NAME(m_dma_byte));
	save_item(NAME(m_dma_source));
	save_item(NAME(m_dma_destination));
	save_item(NAME(m_dma_source_io));
	save_item(NAME(m_dma_destination_io));
	save_item(NAME(m_dma_source_mode));
	save_item(NAME(m_dma_destination_mode));
}

inline void z180_device::cycle_emit(u8 edge)
{
	// UM0050 p108 Fig49: TEND is low from T1 falling through T3
	// falling of the last write cycle, including all inserted waits.
	if (m_cycle.owner == cycle_owner::dma && m_dma_stage && m_dma_bcr[m_dma_channel].w == 1 && (edge == 1 || edge == 4))
	{
		unsigned const ch = m_dma_channel;
		if (edge == 1)
			m_iol |= 0x20000 << ch;
		else
			m_iol &= ~(0x20000 << ch);
		(ch ? m_tend1_cb : m_tend0_cb)(edge == 1 ? ASSERT_LINE : CLEAR_LINE);
	}
	// UM0050 pp16-17 Figs7/8: IOC=1 selects at T1 falling;
	// IOC=0 selects at T2 rising. Internal I/O also drives these strobes
	// (UM0050 p44), while ignoring external input data and WAIT.
	if (m_io_strobe && m_cycle.io_select_edge && (edge == m_cycle.io_select_edge || edge == 4))
		m_io_strobe(m_cycle.address, m_cycle.kind == cycle_kind::output, m_cycle.data, edge != 4);
	if (m_cycle_observer)
		m_cycle_observer(m_half_phi, u8(m_cycle.kind) + 16 * u8(m_cycle.owner), edge, m_cycle.address, m_cycle.data);
}

void z180_device::cycle_suspend()
{
	cycle_flush_quiet();
	// Retain exception unwinding in the independent forced-reference path.
	// Normal suspension crosses only scalar/reference ISA and engine frames;
	// memory handlers and callbacks have returned before either yield site.
	// No destructor or hardware edge is bypassed by this control transfer.
	if (m_cycle_reference)
		throw cycle_yield{};
	Z180_CONTINUATION_RESTORE(m_cycle_resume);
}

inline bool z180_device::cycle_next_grant()
{
	return m_cycle_grants && cycle_continue_grant();
}

bool z180_device::cycle_continue_grant()
{
	// Only a slow phase path may retain its current host frame. An admitted
	// whole/unrolled path must re-enter after an external input changes.
	if (!m_instruction_active || m_cycle_reference || m_instruction_fast || m_instruction_whole || !m_replay_pristine ||
		(machine().debug_flags & (DEBUG_FLAG_ENABLED | DEBUG_FLAG_CALL_HOOK)))
		return false;
	cycle_flush_quiet();
	const bool executing = std::exchange(m_cycle_executing, false);
	const bool next = scheduler().continue_domain_grant(*this);
	m_cycle_executing = executing;
	if (!next)
		return false;
	m_direct_reads_enabled = !machine().side_effects_disabled();
	m_fast_irq_valid = false;
	m_instruction_fast = false;
	m_instruction_whole = false;
	// Mapping/debugger imports invalidate the provisional ISA state. The new
	// grant is ready, but the existing journal must reconstruct that state.
	if (!m_replay_pristine)
		cycle_suspend();
	if (m_instruction_active)
		++m_resumptions;
	return true;
}

inline void z180_device::cycle_flush_quiet()
{
	if (!m_quiet_phi)
		return;
	// No observer has run between these complete CPU cycles. Their journal
	// already records every read and Ti; materialize the final completed bus
	// job once, before a callback or partial phase can inspect it. MMU and
	// pin selection cannot change before this flush.
	auto const &h = m_history[m_quiet_record];
	bool const internal = h.kind == cycle_kind::internal;
	m_cycle = {};
	m_cycle.kind = h.kind;
	m_cycle.data = h.data;
	m_cycle.address = internal ? 0 : MMU_REMAP_ADDR(h.address);
	m_cycle.nominal = internal ? 1 : 3;
	m_cycle.phase = internal ? 1 : 9;
	m_cycle.elapsed = internal ? 2 : 6;
	m_cycle.sampled = !internal;
	m_cycle.no_arbitration = m_quiet_no_arbitration;
	// Every composed span contains an even number of half-PHI ticks, so
	// phi_half is unchanged. No register/callback can change these divider
	// inputs until after this flush. Refresh is disabled, but its phase still
	// runs and must be retained for a later RCR write.
	assert(!cycle_prt_timed() && !(m_rcr & 0x80));
	cycle_quiet_timers(m_quiet_phi);
	m_refresh_phase = (m_refresh_phase + 2 * m_quiet_phi) % 160;
	m_quiet_phi = 0;
}

inline void z180_device::cycle_quiet_timers(unsigned phi)
{
	// handle_io_timers applies the FRC and PRT recurrences for any span, so
	// one update equals the same span in single steps.
	if (m_tcr & 3)
		handle_io_timers(phi);
	else
	{
		unsigned const prescale = m_frc_prescale + phi;
		m_frc -= prescale / 10;
		m_frc_prescale = prescale % 10;
	}
}

inline void z180_device::cycle_advance_quiet(unsigned ticks)
{
	assert(!m_quiet_phi && !cycle_prt_timed() && !m_advance_left && m_icount >= int(ticks));
	m_icount -= ticks;
	m_half_phi += ticks;
	m_cycle.elapsed += ticks;
	unsigned const phi = (m_phi_half + ticks) / 2;
	m_phi_half = (m_phi_half + ticks) & 1;
	cycle_quiet_timers(phi);
	unsigned const period = 20U << (m_rcr & 3);
	if ((m_rcr & 0x80) && (m_refresh_phase % period + ticks >= period) && !m_bus_acknowledged)
		m_refresh_pending = true;
	m_refresh_phase = (m_refresh_phase + ticks) % 160;
}

void z180_device::cycle_advance(unsigned ticks)
{
	assert(!m_quiet_phi);
	if (!m_advance_left)
		m_advance_left = ticks;
	if (!m_cycle_reference)
	{
		// No callback occurs inside this span. FRC/PRT already implement
		// their divider and reload recurrences arithmetically; update them
		// once at the next externally observable edge or scheduler boundary.
		// Keep the one-half-PHI implementation below as the reference.
		while (m_advance_left)
		{
			if (m_icount <= 0 && !cycle_next_grant())
				cycle_suspend();
			unsigned const n = std::min<unsigned>(m_advance_left, m_icount);
			m_icount -= n;
			m_advance_left -= n;
			m_half_phi += n;
			m_cycle.elapsed += n;
			unsigned const phi = (m_phi_half + n) / 2;
			m_phi_half = (m_phi_half + n) & 1;
			if (m_tcr & 3)
				handle_io_timers(phi);
			else
			{
				unsigned const prescale = m_frc_prescale + phi;
				m_frc -= prescale / 10;
				m_frc_prescale = prescale % 10;
			}
			unsigned const period = 20U << (m_rcr & 3);
			if ((m_rcr & 0x80) && (m_refresh_phase % period + n >= period) && !m_bus_acknowledged)
				m_refresh_pending = true;
			m_refresh_phase = (m_refresh_phase + n) % 160;
		}
		return;
	}
	while (m_advance_left)
	{
		if (m_icount <= 0)
			cycle_suspend();
		unsigned n = std::min<unsigned>(m_advance_left, m_cycle_reference ? 1 : m_icount);
		n = std::min(n, 2 * (10U - m_frc_prescale) - m_phi_half);
		m_icount -= n;
		m_advance_left -= n;
		m_half_phi += n;
		m_cycle.elapsed += n;
		unsigned const phi = (m_phi_half + n) / 2;
		m_phi_half = (m_phi_half + n) & 1;
		handle_io_timers(phi);
		unsigned const period = 20U << (m_rcr & 3);
		if ((m_rcr & 0x80) && (m_refresh_phase % period + n >= period) && !m_bus_acknowledged)
			m_refresh_pending = true;
		m_refresh_phase = (m_refresh_phase + n) % 160;
	}
}

unsigned z180_device::cycle_internal_io_wait(offs_t address, bool write) const
{
	unsigned const reg = address & (m_extended_io ? 0x7f : 0x3f);
	// Selected reconstruction, not a published silicon equation: common
	// five-PHI service window from the HD64180 Oct 1985 CSI/O aperture
	// (pp80-83), UM0050 Table4's 0-4 bound, and HD647180X p32's reload
	// classification. The common epoch is a model assumption.
	// Predict the no-wait T3 rising edge at T1+4 half-PHI. This is exactly
	// the same phase as calculating T2+2; no input can change PHI counts.
	constexpr u32 read_bank = 0x00f0fb00, write_bank = 0x00f0f8c0;
	if (reg >= 24 || !(((write ? write_bank : read_bank) >> reg) & 1))
		return 0;
	unsigned const q = ((m_half_phi - m_io_service_stopped + 4) / 2) % 5;
	return (5 - q) % 5;
}

void z180_device::cycle_begin(cycle_owner owner, cycle_kind kind, offs_t address, u8 data, bool no_arbitration)
{
	cycle_flush_quiet();
	assert(!m_cycle.active);
	m_cycle = {};
	m_cycle.owner = owner;
	m_cycle.kind = kind;
	m_cycle.address = address;
	m_cycle.data = data;
	m_cycle.active = true;
	m_cycle.no_arbitration = no_arbitration;
	bool const io = kind == cycle_kind::input || kind == cycle_kind::output;
	m_cycle.internal_io = io && is_internal_io_address(address);
	if (io)
		m_cycle.io_select_edge = (m_omcr & 0x20) ? 1 : 2;
	bool const write = kind == cycle_kind::write || kind == cycle_kind::output;
	bool const internal = kind == cycle_kind::internal || kind == cycle_kind::release;
	if (internal)
		m_cycle.nominal = 1;
	else if (kind == cycle_kind::refresh)
		m_cycle.nominal = (m_rcr & 0x40) ? 3 : 2;
	else if (kind == cycle_kind::acknowledge)
	{
		m_cycle.nominal = 5;
		m_cycle.waits =
			owner == cycle_owner::cpu && m_instruction_interrupt == 2 ? (((m_dcntl >> 4) & 3) ? ((m_dcntl >> 4) & 3) + 3 : 2) : 2;
	}
	else if (kind == cycle_kind::nmi)
		m_cycle.waits = 0;
	else
	{
		// UM0050 Table4/p44: no IWI or external WAIT on internal ports.
		// Selected internal data-bank synchronization includes PRT reloads.
		m_cycle.waits = io ? (m_cycle.internal_io ? cycle_internal_io_wait(address, write) : io_wait_states()) : memory_wait_states();
		if (m_bus_wait && !m_cycle.internal_io && (io || write || !cycle_direct_read(address)))
			m_cycle.waits = std::max<unsigned>(m_cycle.waits, m_bus_wait(address, write, io));
	}
	cycle_emit(0);
}

void z180_device::cycle_sample_opcode()
{
	if (m_cycle.kind == cycle_kind::fetch || m_cycle.kind == cycle_kind::halt || m_cycle.kind == cycle_kind::nmi)
	{
		if (auto const p = cycle_direct_read(m_cycle.address))
			m_cycle.data = *m_direct_read_bus = *p;
		else
			m_cycle.data = m_copcodes.read_byte(m_cycle.address);
		m_cycle.sampled = true;
		cycle_emit(4);
	}
	else if (m_cycle.kind == cycle_kind::acknowledge)
	{
		if (m_instruction_interrupt == 2 && m_ack_vector < 0)
		{
			auto *intf = daisy_get_irq_device();
			m_ack_vector = intf ? intf->z80daisy_irq_ack() : standard_irq_callback(0, m_PC.w.l);
		}
		m_cycle.data = m_ack_vector < 0 ? 0 : u8(m_ack_vector);
		m_cycle.sampled = true;
		cycle_emit(4);
	}
}

void z180_device::cycle_sample_data()
{
	if (!m_cycle.sampled)
	{
		// Set the guard before a handler that can abort this timeslice.
		m_cycle.sampled = true;
		auto const direct =
			(m_cycle.kind == cycle_kind::operand || m_cycle.kind == cycle_kind::read) ? cycle_direct_read(m_cycle.address) : nullptr;
		if (direct)
			m_cycle.data = *m_direct_read_bus = *direct;
		else
			switch (m_cycle.kind)
			{
			case cycle_kind::operand:
				m_cycle.data = m_cprogram.read_byte(m_cycle.address);
				break;
			case cycle_kind::read:
				m_cycle.data = m_program.read_byte(m_cycle.address);
				break;
			case cycle_kind::write:
				m_program.write_byte(m_cycle.address, m_cycle.data);
				break;
			case cycle_kind::input:
				m_cycle.data = m_cycle.internal_io ? z180_readcontrol(m_cycle.address) : m_io.read_byte(m_cycle.address);
				break;
			case cycle_kind::output:
				if (m_cycle.internal_io)
					z180_writecontrol(m_cycle.address, m_cycle.data);
				else
					m_io.write_byte(m_cycle.address, m_cycle.data);
				break;
			default:
				break;
			}
		cycle_emit(4);
		// I/O can alter the clock, timers, DMA or pins. Re-evaluate
		// arbitration for the rest of this instruction after that edge.
		if (m_cycle.kind == cycle_kind::input || m_cycle.kind == cycle_kind::output)
			m_instruction_fast = false;
	}
}

bool z180_device::cycle_run_unobserved()
{
	// This is the same six-edge memory cycle, evaluated at its observable
	// sample and end instead of dispatching six one-half-PHI phases. The
	// scheduler budget excludes external input changes; without a PRT tick
	// source, sampled pins/interrupts cannot change before the memory handler.
	// IORQ uses its own observable-edge path below. TEND, trace callbacks,
	// asynchronous WAIT and partial cycles retain the explicit phase path.
	// A memory callback can shorten the budget: in
	// that case leave its exact saved phase for the ordinary continuation.
	if (m_cycle_reference || m_cycle_observer || m_cycle.phase || m_advance_left || m_cycle.ttp || m_cycle.trailing || m_wait_line ||
		cycle_prt_timed() || (m_cycle.owner == cycle_owner::dma && m_dma_stage && m_dma_bcr[m_dma_channel].w == 1))
		return false;
	bool const internal = m_cycle.kind == cycle_kind::internal || m_cycle.kind == cycle_kind::release;
	if (!internal && (m_cycle.nominal != 3 || m_cycle.kind >= cycle_kind::internal))
		return false;
	unsigned const waits = 2U * m_cycle.waits;
	// The continuation's span counter is 16 bits. Very long externally
	// supplied waits remain a sequence of individual TW cycles.
	if (waits > 65530)
		return false;
	if (m_icount < int(internal ? 2 : 6 + waits))
		return false;
	if (m_cycle.io_select_edge)
		return cycle_run_io_unobserved();
	bool const fetch = m_cycle.kind == cycle_kind::fetch;
	m_sampled_iol = m_iol;
	std::copy(std::begin(m_dma_edge_generation), std::end(m_dma_edge_generation), m_dma_edge_sampled);
	m_sampled_bus_request = m_bus_released;
	if (m_cycle.owner == cycle_owner::cpu && m_cycle.kind != cycle_kind::release)
	{
		if (m_instruction_fast && m_fast_irq_valid)
			m_sampled_interrupt = m_fast_interrupt;
		else
		{
			cycle_sample_interrupts();
			m_fast_interrupt = m_sampled_interrupt;
			m_fast_irq_valid = m_instruction_fast;
		}
	}
	else if (m_cycle.owner == cycle_owner::dma && m_dma_stage && m_nmi_pending)
		m_dma_nmi = true;
	if (internal)
	{
		cycle_advance_quiet(2);
		m_cycle.phase = 1;
		m_cycle.active = false;
		return true;
	}
	cycle_advance_quiet((fetch ? 4 : 5) + waits);
	m_cycle.waits = 0;
	m_cycle.phase = fetch ? 5 : 6;
	if (fetch)
		cycle_sample_opcode();
	else
		cycle_sample_data();
	if (m_nmi_pending || m_bus_released || m_wait_line)
		m_instruction_fast = false;
	unsigned const tail = fetch ? 2 : 1;
	if (m_icount < int(tail))
		return false;
	if (m_tcr & 3)
		cycle_advance(tail);
	else
		cycle_advance_quiet(tail);
	m_cycle.phase = 9;
	m_cycle.active = false;
	return true;
}

bool z180_device::cycle_run_io_unobserved()
{
	// Same external I/O cycle as the phase loop: IOC selects IORQ at T1
	// falling or T2 rising (UM0050 Figs 7/8); data and IORQ release remain
	// at T3 falling. No timed source changes inside the admitted span.
	// The select callback can shorten it or change WAIT/PRT. Publish its
	// phase first, then fall back without repeating the already issued edge.
	auto sample_dma = [this]()
	{
		m_sampled_iol = m_iol;
		std::copy(std::begin(m_dma_edge_generation), std::end(m_dma_edge_generation), m_dma_edge_sampled);
	};
	auto admitted = [this](unsigned remaining)
	{ return m_icount >= int(remaining) && !m_wait_line && !cycle_prt_timed() && !m_cycle_observer && !m_cycle_reference; };
	unsigned const waits = 2U * m_cycle.waits;
	if (m_cycle.io_select_edge == 1)
	{
		cycle_advance_quiet(1);
		m_cycle.phase = 1;
		cycle_emit(1);
		if (!admitted(5 + waits))
			return false;
		cycle_advance_quiet(1);
		m_cycle.phase = 2;
		sample_dma();
	}
	else
	{
		cycle_advance_quiet(2);
		m_cycle.phase = 2;
		sample_dma();
		cycle_emit(2);
		if (!admitted(4 + waits))
			return false;
	}
	// DREQ is sampled at T2 rising and every TW rising. All TW samples
	// here see the same inputs; retain the final sample before T3.
	cycle_advance_quiet(1 + waits);
	m_cycle.waits = 0;
	m_cycle.phase = 4;
	if (waits)
		sample_dma();
	m_sampled_bus_request = m_bus_released;
	if (m_cycle.owner == cycle_owner::cpu)
		cycle_sample_interrupts();
	else if (m_cycle.owner == cycle_owner::dma && m_dma_stage && m_nmi_pending)
		m_dma_nmi = true;
	cycle_advance_quiet(2);
	m_cycle.phase = 6;
	cycle_sample_data();
	if (!admitted(1))
		return false;
	cycle_advance_quiet(1);
	m_cycle.phase = 9;
	m_cycle.active = false;
	return true;
}

unsigned z180_device::cycle_resume_quiet()
{
	// A short grant can end inside the same pending memory bus cycle. Its ISA
	// prefix would only reconstruct the provisional registers we already
	// have. Advance directly when no callback can run. A completed cycle
	// retains phase 9/active until replay returns through its original ISA
	// caller, which owns the journal and all subsequent zero-time effects.
	if ((m_instruction_active && !m_replay_pristine) || !m_cycle.active ||
		(m_cycle.owner != cycle_owner::cpu && m_cycle.owner != cycle_owner::dma) ||
		(m_cycle.owner == cycle_owner::cpu && !m_instruction_active) || m_cycle.kind > cycle_kind::write || m_cycle_reference ||
		m_cycle_observer || m_instruction_interrupt >= 0 || m_trapping ||
		(machine().debug_flags & (DEBUG_FLAG_ENABLED | DEBUG_FLAG_CALL_HOOK)) || m_icount <= 0 || m_advance_left > 1 ||
		m_cycle.nominal != 3 || m_cycle.waits > 32765 || m_cycle.ttp || m_cycle.trailing || m_wait_line || cycle_prt_timed() ||
		(m_cycle.owner == cycle_owner::dma && m_dma_stage && m_dma_bcr[m_dma_channel].w == 1))
		return false;
	unsigned const phase = m_cycle.phase;
	if (phase > 8)
		return false;
	// Count the remaining half-PHI edges before T3. A pending TW must
	// finish even if external WAIT was removed between scheduler grants.
	unsigned const pre = phase < 3 ? 3 - phase : 0;
	bool const waiting = phase <= 3 || phase >= 7;
	unsigned const half = phase == 8;
	unsigned const tw = waiting ? 2U * (phase >= 7 ? std::max<unsigned>(1, m_cycle.waits) : m_cycle.waits) - half : 0;
	unsigned const post = phase == 5 ? 1 : phase == 6 ? 2 : 0;
	unsigned const finish = pre + tw + 3 - post;
	unsigned ticks = std::min<unsigned>(m_icount, finish);
	unsigned const sample = pre + tw + (m_cycle.kind == cycle_kind::fetch ? 1 : 2) - post;
	u8 const *direct = nullptr;
	if (!m_cycle.sampled && ticks >= sample)
	{
		if (m_cycle.kind == cycle_kind::write || !(direct = cycle_direct_read(m_cycle.address)))
		{
			// Stop before the real callback, then let its original caller
			// reconstruct the ISA state and issue it at the exact data edge.
			if (sample <= 1)
				return false;
			ticks = sample - 1;
		}
	}
	m_advance_left = 0;
	cycle_advance_quiet(ticks);
	unsigned const in_tw = ticks > pre ? std::min(tw, ticks - pre) : 0;
	if ((phase < 2 && ticks >= 2 - phase) || (half + in_tw + 1) / 2 > (half + 1) / 2)
	{
		m_sampled_iol = m_iol;
		std::copy(std::begin(m_dma_edge_generation), std::end(m_dma_edge_generation), m_dma_edge_sampled);
	}
	if (waiting && ticks >= pre + tw)
	{
		m_sampled_bus_request = m_bus_released;
		if (m_cycle.owner == cycle_owner::cpu)
			cycle_sample_interrupts();
		else if (m_dma_stage && m_nmi_pending)
			m_dma_nmi = true;
	}
	m_cycle.waits -= std::min<unsigned>(m_cycle.waits, (half + in_tw) / 2);
	if (direct)
	{
		m_cycle.data = *m_direct_read_bus = *direct;
		m_cycle.sampled = true;
	}
	if (ticks < pre)
		m_cycle.phase = phase + ticks;
	else if (ticks < pre + tw)
		m_cycle.phase = ((half + ticks - pre) & 1) ? 8 : 7;
	else
	{
		static constexpr u8 after[] = {4, 5, 6, 9};
		m_cycle.phase = after[post + ticks - pre - tw];
	}
	m_advance_left = ticks < finish;

	m_instruction_whole = false;
	if (ticks == finish)
		return 2;
	if (m_icount > 0)
		return 0;
	if (m_instruction_active)
		++m_resumptions;
	return 1;
}

bool z180_device::cycle_run_partial()
{
	if (m_cycle_reference || m_cycle_observer || m_icount <= 0 || m_advance_left > 1 || m_cycle.nominal != 3 ||
		m_cycle.kind >= cycle_kind::internal || m_cycle.ttp || m_cycle.trailing || m_cycle.io_select_edge || m_wait_line || cycle_prt_timed() ||
		m_cycle.waits > 32765 || (m_cycle.owner == cycle_owner::dma && m_dma_stage && m_dma_bcr[m_dma_channel].w == 1))
		return false;
	// A deadline can split any half-PHI. Advance directly to the next pin
	// sample or data edge, preserving the exact phase that the reference
	// suspends with, including its one-half-PHI pending advance.
	m_advance_left = 0;
	auto sample_dma = [this]()
	{
		m_sampled_iol = m_iol;
		std::copy(std::begin(m_dma_edge_generation), std::end(m_dma_edge_generation), m_dma_edge_sampled);
	};
	auto sample_irq = [this]()
	{
		m_sampled_bus_request = m_bus_released;
		if (m_cycle.owner == cycle_owner::cpu)
			cycle_sample_interrupts();
		else if (m_cycle.owner == cycle_owner::dma && m_dma_stage && m_nmi_pending)
			m_dma_nmi = true;
	};
	// With no TW or timed input source, compose the pre-sample phases up to
	// the memory edge or this grant's deadline. T2 rising/falling samples
	// remain distinct at cuts: position 3 includes the zero-time IRQ/BUSREQ
	// sample and leaves phase 4 pending, even when no budget remains.
	bool const fetch = m_cycle.kind == cycle_kind::fetch;
	static constexpr u8 position[] = {0, 1, 2, 3, 3, 4};
	static constexpr u8 phase_at[] = {0, 1, 2, 4, 5, 6};
	unsigned const phase = m_cycle.phase;
	if (!m_cycle.waits && phase < 6 && !(fetch && phase == 5) && m_cycle.elapsed == position[phase])
	{
		unsigned const start = position[phase], sample = fetch ? 4 : 5;
		unsigned const n = std::min<unsigned>(m_icount, sample - start), end = start + n;
		cycle_advance_quiet(n);
		if (phase < 2 && end >= 2)
			sample_dma();
		if (phase < 4 && end >= 3)
			sample_irq();
		m_cycle.phase = phase_at[end];
		if (end == sample)
		{
			if (fetch)
				cycle_sample_opcode();
			else
				cycle_sample_data();
			if (m_nmi_pending || m_bus_released || m_wait_line)
				m_instruction_fast = false;
			if (m_tcr & 3)
				return false;
		}
	}
	for (;;)
	{
		if (m_cycle.phase == 9)
		{
			m_cycle.active = false;
			return true;
		}
		if (m_cycle.phase == 3)
		{
			if (m_cycle.waits)
			{
				m_cycle.phase = 7;
				continue;
			}
			sample_irq();
			m_cycle.phase = 4;
			continue;
		}
		if (m_icount <= 0)
		{
			m_advance_left = 1;
			if (cycle_next_grant())
				return false;
			cycle_suspend();
		}
		switch (m_cycle.phase)
		{
		case 0:
		case 1:
		{
			unsigned const n = std::min<unsigned>(m_icount, 2 - m_cycle.phase);
			cycle_advance_quiet(n);
			m_cycle.phase += n;
			if (m_cycle.phase == 2)
				sample_dma();
			break;
		}
		case 2:
			cycle_advance_quiet(1);
			m_cycle.phase = 3;
			break;
		case 7:
		{
			unsigned const n = std::min<unsigned>(m_icount, 2U * std::max<unsigned>(1, m_cycle.waits));
			cycle_advance_quiet(n);
			sample_dma();
			m_cycle.waits -= std::min<unsigned>(m_cycle.waits, n / 2);
			m_cycle.phase = (n & 1) ? 8 : 3;
			break;
		}
		case 8:
			cycle_advance_quiet(1);
			if (m_cycle.waits)
				--m_cycle.waits;
			m_cycle.phase = 3;
			break;
		case 4:
		case 5:
		{
			bool const fetch = m_cycle.kind == cycle_kind::fetch;
			unsigned const sample = fetch ? 5 : 6;
			if (m_cycle.phase < sample)
			{
				unsigned const n = std::min<unsigned>(m_icount, sample - m_cycle.phase);
				cycle_advance_quiet(n);
				m_cycle.phase += n;
				if (m_cycle.phase == sample)
				{
					if (fetch)
						cycle_sample_opcode();
					else
						cycle_sample_data();
					if (m_nmi_pending || m_bus_released || m_wait_line)
						m_instruction_fast = false;
					if (m_tcr & 3)
						return false;
				}
				break;
			}
			// A fetched opcode is already sampled; T3's remaining halves
			// have no other callback until the cycle ends.
			unsigned const n = std::min<unsigned>(m_icount, 2);
			cycle_advance_quiet(n);
			m_cycle.phase = n == 2 ? 9 : 6;
			break;
		}
		case 6:
			cycle_advance_quiet(1);
			m_cycle.phase = 9;
			break;
		default:
			fatalerror("Z180 unexpected partial phase %u", m_cycle.phase);
		}
	}
}

void z180_device::cycle_run()
{
	cycle_flush_quiet();
	// Reject impossible shortcuts before entering their host stack frames.
	// Partial scheduler grants commonly resume at a nonzero bus phase.
	if (!m_cycle_reference && !m_cycle_observer)
	{
		if (!m_cycle.phase && !m_advance_left && cycle_run_unobserved())
			return;
		if (m_icount > 0 && m_cycle.nominal == 3 && m_cycle.kind < cycle_kind::internal && !m_cycle.io_select_edge && cycle_run_partial())
			return;
	}
	// Phase numbers distinguish T1 rise/fall, T2 rise/fall and T3 rise/fall.
	// WAIT is sampled at T2/TW falling (UM0050 pp19,27); no memory or I/O
	// operation is committed until its documented sample/WR release edge.
	while (m_cycle.active)
	{
		// The no-event path invokes the ISA handler once and unrolls its
		// ordinary bus cycles. It has exactly the same edges and sample
		// functions as the resumable path below. A callback may still bring
		// a scheduler deadline forward: phase is saved before every callback.
		if (m_instruction_fast && m_cycle.owner == cycle_owner::cpu && m_cycle.phase == 0 && m_cycle.nominal == 3 && !m_cycle.ttp &&
			!m_cycle.waits && !m_wait_line && m_icount >= 6)
		{
			cycle_advance(1);
			m_cycle.phase = 1;
			cycle_emit(1);
			cycle_advance(1);
			m_cycle.phase = 2;
			m_sampled_iol = m_iol;
			std::copy(std::begin(m_dma_edge_generation), std::end(m_dma_edge_generation), m_dma_edge_sampled);
			cycle_emit(2);
			cycle_advance(1);
			m_cycle.phase = 3;
			if (m_wait_line && !m_cycle.internal_io)
				continue;
			m_sampled_bus_request = m_bus_released;
			cycle_sample_interrupts();
			m_cycle.phase = 4;
			cycle_advance(1);
			m_cycle.phase = 5;
			cycle_sample_opcode();
			cycle_advance(1);
			m_cycle.phase = 6;
			cycle_sample_data();
			cycle_advance(1);
			m_cycle.phase = 9;
			m_cycle.active = false;
			cycle_emit(5);
			return;
		}
		bool const internal = m_cycle.kind == cycle_kind::internal || m_cycle.kind == cycle_kind::release;
		if (internal || m_cycle.kind == cycle_kind::refresh)
		{
			if (!m_cycle.phase)
			{
				if (!m_advance_left && internal)
				{
					m_sampled_iol = m_iol;
					std::copy(std::begin(m_dma_edge_generation), std::end(m_dma_edge_generation), m_dma_edge_sampled);
				}
				cycle_advance(2 * m_cycle.nominal - 1);
				m_cycle.phase = 1;
				if (internal)
				{
					m_sampled_bus_request = m_bus_released;
					if (m_cycle.kind != cycle_kind::release)
						cycle_sample_interrupts();
				}
				cycle_emit(1);
			}
			cycle_advance(1);
			m_cycle.active = false;
			cycle_emit(5);
			return;
		}
		switch (m_cycle.phase)
		{
		case 0:
			cycle_advance(1);
			m_cycle.phase = 1;
			cycle_emit(1);
			break;
		case 1:
			cycle_advance(1);
			m_cycle.phase = 2;
			m_sampled_iol = m_iol;
			std::copy(std::begin(m_dma_edge_generation), std::end(m_dma_edge_generation), m_dma_edge_sampled);
			cycle_emit(2);
			break;
		case 2:
			cycle_advance(1);
			m_cycle.phase = 3;
			break;
		case 3:
			if (m_cycle.waits || (m_wait_line && !m_cycle.internal_io))
			{
				m_cycle.phase = 7;
				break;
			}
			m_sampled_bus_request = m_bus_released;
			if (m_cycle.owner == cycle_owner::cpu)
				cycle_sample_interrupts();
			else if (m_cycle.owner == cycle_owner::dma && m_dma_stage && m_nmi_pending)
				m_dma_nmi = true;
			m_cycle.phase = m_cycle.ttp ? 10 : 4;
			break;
		case 10:
			cycle_advance(2);
			m_cycle.phase = 4;
			break; // UM0050 p72: TTP is not a bus boundary.
		case 7:
			cycle_advance(1);
			m_cycle.phase = 8;
			m_sampled_iol = m_iol;
			std::copy(std::begin(m_dma_edge_generation), std::end(m_dma_edge_generation), m_dma_edge_sampled);
			break;
		case 8:
			cycle_advance(1);
			if (m_cycle.waits)
				--m_cycle.waits;
			cycle_emit(3);
			m_cycle.phase = 3;
			// UM0050 Fig35: IORQ falls at the penultimate programmed Tw
			// falling edge. External WAIT can extend the selected interval.
			if (m_cycle.kind == cycle_kind::acknowledge && m_instruction_interrupt == 2 && m_cycle.waits == 1 && !m_cycle.ack_selected)
			{
				m_cycle.ack_selected = true;
				if (m_irq0_ack_strobe)
					m_irq0_ack_strobe(true);
			}
			break;
		case 4:
			cycle_advance(1);
			m_cycle.phase = 5;
			cycle_sample_opcode();
			break;
		case 5:
			cycle_advance(1);
			m_cycle.phase = 6;
			cycle_sample_data();
			if (m_cycle.ack_selected)
			{
				m_cycle.ack_selected = false;
				if (m_irq0_ack_strobe)
					m_irq0_ack_strobe(false);
			}
			break;
		case 6:
			cycle_advance(1);
			m_cycle.phase = 9;
			break;
		case 9:
			if (m_cycle.trailing)
				cycle_advance(2 * m_cycle.trailing);
			m_cycle.active = false;
			cycle_emit(5);
			return;
		}
	}
}

void z180_device::cycle_decode(unsigned family, unsigned opcode)
{
	m_family = family;
	m_opcode = opcode;
	// Table 51's Ti positions are invariant for an opcode. Pack the eight
	// possible transaction positions once, instead of decoding the opcode
	// afresh on every operand/read/write. TRAP and RETI's M1 option below
	// retain their runtime choices. The original decoder remains the oracle.
	static constexpr auto schedules = []
	{
		std::array<std::array<u32, 256>, 5> t{};
		for (unsigned op = 0; op < 256; ++op)
		{
			auto set = [&](unsigned f, unsigned n, unsigned ti) { t[f][op] |= ti << (3 * n); };
			if (op == 0x10 || op == 0x02 || op == 0x12 || (op >= 0x70 && op <= 0x77 && op != 0x76))
				set(Z180_TABLE_op, 1, 1);
			if (op == 0x22 || op == 0x32 || op == 0xcd || (op & 0xc7) == 0xc4 || op == 0xe3)
				set(Z180_TABLE_op, 3, 1);
			if (op == 0x34 || op == 0x35 || op == 0xd3)
				set(Z180_TABLE_op, 2, 1);
			if ((op & 0xcf) == 0xc5 || (op & 0xc7) == 0xc7)
				set(Z180_TABLE_op, 1, 2);
			if ((op & 0xc7) == 0xc0)
				set(Z180_TABLE_op, 1, 1);
			if (op == 0x22 || op == 0xe3 || op == 0x34 || op == 0x35)
				set(Z180_TABLE_xy, 4, 1);
			if (op == 0xe5)
				set(Z180_TABLE_xy, 2, 2);
			if (op == 0x34 || op == 0x35 || (op >= 0x40 && op < 0xc0 && (op & 7) == 6))
				set(Z180_TABLE_xy, 3, 2);
			if (op >= 0x70 && op <= 0x77 && op != 0x76)
				set(Z180_TABLE_xy, 3, 3);
			if (op < 0x40 && (op & 7) == 1)
				set(Z180_TABLE_ed, 3, 1);
			if ((op >= 0x40 && op < 0x80 && (op & 7) == 1) || op == 0x34 || op == 0x83 || op == 0x8b || op == 0x93 || op == 0x9b)
				set(Z180_TABLE_ed, 2, 1);
			if ((op & 0xcf) == 0x43)
				set(Z180_TABLE_ed, 4, 1);
			if (op == 0x67 || op == 0x6f)
				set(Z180_TABLE_ed, 3, 4);
			set(Z180_TABLE_cb, 3, 1);
			set(Z180_TABLE_xycb, 5, 1);
		}
		return t;
	}();
	m_prelude_schedule = schedules[family][opcode];
	// Admission is made once when the opcode is known, from its maximum
	// nominal duration and the current external-event horizon. Prefixes
	// admit at their final opcode. I/O and pin changes revoke the proof;
	// an address-dependent external wait can still require a continuation.
	unsigned const bound = m_cc[family][opcode] + m_cc[Z180_TABLE_ex][opcode];
	m_instruction_whole =
		m_instruction_fast && !m_cycle_observer && m_cc[family][opcode] && m_icount >= int(2 * (bound > m_nominal ? bound - m_nominal : 0));
}

unsigned z180_device::cycle_prelude_reference(cycle_kind kind) const
{
	unsigned const n = m_bus_number, op = m_opcode;
	if (m_trapping)
	{
		if (kind == cycle_kind::write && n == (m_third_trap ? 5U : 2U))
			return m_third_trap ? 4 : 5;
		return 0;
	}
	if (m_family == Z180_TABLE_op)
	{
		if (op == 0x10 && n == 1)
			return 1; // UM p256: its following boundary is inhibited.
		if ((op == 0x02 || op == 0x12 || (op >= 0x70 && op <= 0x77 && op != 0x76)) && n == 1)
			return 1;
		if ((op == 0x22 || op == 0x32 || op == 0xcd || (op & 0xc7) == 0xc4) && n == 3)
			return 1;
		if ((op == 0x34 || op == 0x35) && n == 2)
			return 1;
		if (((op & 0xcf) == 0xc5 || (op & 0xc7) == 0xc7) && n == 1)
			return 2;
		if ((op & 0xc7) == 0xc0 && n == 1)
			return 1;
		if (op == 0xe3 && n == 3)
			return 1;
		if (op == 0xd3 && n == 2)
			return 1;
	}
	else if (m_family == Z180_TABLE_xy)
	{
		if (op == 0x22 && n == 4)
			return 1;
		if (op == 0xe3 && n == 4)
			return 1;
		if (op == 0xe5 && n == 2)
			return 2;
		if ((op == 0x34 || op == 0x35) && n == 4)
			return 1;
		if (n == 3 && (op == 0x34 || op == 0x35 || (op >= 0x40 && op < 0xc0 && (op & 7) == 6)))
			return 2;
		if (n == 3 && op >= 0x70 && op <= 0x77 && op != 0x76)
			return 3;
	}
	else if (m_family == Z180_TABLE_ed)
	{
		if (op < 0x40 && (op & 7) == 1 && n == 3)
			return 1;
		if (op >= 0x40 && op < 0x80 && (op & 7) == 1 && n == 2)
			return 1;
		if ((op == 0x34 || op == 0x83 || op == 0x8b || op == 0x93 || op == 0x9b) && n == 2)
			return 1;
		if ((op & 0xcf) == 0x43 && n == 4)
			return 1;
		if ((op == 0x67 || op == 0x6f) && n == 3)
			return 4;
		if (op == 0x4d && !(m_omcr & 0x80))
		{
			if (n == 2)
				return 3;
			if (n == 3)
				return 1;
		}
	}
	else if (m_family == Z180_TABLE_cb && kind == cycle_kind::write && n == 3)
		return 1;
	else if (m_family == Z180_TABLE_xycb && kind == cycle_kind::write && n == 5)
		return 1;
	return 0;
}

u8 z180_device::cycle_access_slow(cycle_kind kind, offs_t address, u8 data, unsigned pre)
{
	for (unsigned n = 0; n < pre; ++n)
		cycle_transaction(cycle_kind::internal, 0, 0, m_family == Z180_TABLE_op && m_opcode == 0x10);
	++m_bus_number;
	return cycle_transaction(kind, address, data);
}

void z180_device::cycle_whole_internal(unsigned count, bool no_arbitration)
{
	assert(count && m_cursor == m_history_count && !m_cycle.active);
	if (m_history_count + count > std::size(m_history))
		fatalerror("Z180 instruction journal overflow at %04x", _PPC);
	// These Ti boundaries have identical sampled inputs and no requester.
	// The exact divider recurrence covers the span; preserve every journal
	// entry in case a later memory callback introduces a continuation.
	if (!m_fast_irq_valid)
		cycle_sample_stable_inputs();
	cycle_defer_clocks(count);
	for (unsigned n = 0; n < count; ++n)
		m_history[m_history_count++] = {cycle_kind::internal, 0, 0, 1};
	m_quiet_record = m_history_count - 1;
	m_quiet_no_arbitration = no_arbitration;
	m_cursor += count;
	m_nominal += count;
	m_boundary_allowed = !no_arbitration;
	m_dma_allowed = true;
	m_dma_start_pending = false;
	m_refresh_after_sleep = false;
}

u8 z180_device::cycle_mapped_access(cycle_kind kind, offs_t address, offs_t physical, u8 data)
{
	cycle_flush_quiet();
	m_cycle = {};
	m_cycle.kind = kind;
	m_cycle.address = physical;
	m_cycle.data = data;
	m_cycle.active = true;
	m_cycle.waits = memory_wait_states();
	if (m_bus_wait)
		m_cycle.waits = std::max<unsigned>(m_cycle.waits, m_bus_wait(m_cycle.address, kind == cycle_kind::write, false));
	unsigned const waits = 2U * m_cycle.waits;
	if (!m_instruction_fast || m_icount < int(6 + waits) || waits > 65530)
	{
		m_instruction_whole = false;
		cycle_run();
	}
	else
	{
		if (!m_fast_irq_valid)
			cycle_sample_stable_inputs();
		bool const fetch = kind == cycle_kind::fetch;
		cycle_advance_quiet((fetch ? 4 : 5) + waits);
		m_cycle.waits = 0;
		m_cycle.phase = fetch ? 5 : 6;
		m_cycle.sampled = true;
		auto const direct = kind != cycle_kind::write ? cycle_direct_read(m_cycle.address) : nullptr;
		if (direct)
			m_cycle.data = *m_direct_read_bus = *direct;
		else
			switch (kind)
			{
			case cycle_kind::fetch:
				m_cycle.data = m_copcodes.read_byte(m_cycle.address);
				break;
			case cycle_kind::operand:
				m_cycle.data = m_cprogram.read_byte(m_cycle.address);
				break;
			case cycle_kind::read:
				m_cycle.data = m_program.read_byte(m_cycle.address);
				break;
			case cycle_kind::write:
				m_program.write_byte(m_cycle.address, data);
				break;
			default:
				break;
			}
		if (m_nmi_pending || m_bus_released || m_wait_line)
			m_instruction_fast = false;
		unsigned const tail = fetch ? 2 : 1;
		if (!m_instruction_fast || m_icount < int(tail))
		{
			m_instruction_whole = false;
			cycle_run();
		}
		else
		{
			cycle_advance_quiet(tail);
			m_cycle.phase = 9;
			m_cycle.active = false;
		}
	}
	if (m_history_count == std::size(m_history))
		fatalerror("Z180 instruction journal overflow at %04x", _PPC);
	m_history[m_history_count++] = {kind, address, m_cycle.data, 3};
	++m_cursor;
	m_nominal += 3;
	m_boundary_allowed = true;
	m_dma_allowed = !m_dma_start_pending;
	m_dma_start_pending = false;
	m_refresh_after_sleep = false;
	if (!m_instruction_fast)
		cycle_resume_admission();
	return m_cycle.data;
}

void z180_device::cycle_resume_admission()
{
	// Replay and the actual pending cycle are finished. Re-prove the event
	// horizon for the instruction's remainder instead of keeping it slow
	// solely because an earlier scheduler grant ended. Invalidate, but do
	// not take a new pin sample here: a late IRQ after T2 must wait for the
	// next real Ti/memory cycle, if this instruction has one at all.
	if (m_icount >= 2 && !m_cycle_reference && !m_cycle_observer && !m_trapping && m_instruction_interrupt < 0 && !m_cycle.active &&
		m_cursor == m_history_count && m_boundary_allowed && !(m_dstat & 0xc0) && !(m_rcr & 0x80) && !cycle_prt_timed() && !(m_dcntl & 0xf0) &&
		!m_wait_line && !m_bus_released && !m_sampled_bus_request && !m_bus_acknowledged && !m_nmi_pending && !m_refresh_pending &&
		m_dma_channel == 0xff && m_dma_continue == 0xff)
	{
		m_instruction_fast = true;
		m_fast_irq_valid = false;
	}
}

u8 z180_device::cycle_transaction(cycle_kind kind, offs_t address, u8 data, bool no_arbitration)
{
	if (m_cursor < m_history_count)
	{
		auto const &h = m_history[m_cursor++];
		if (h.kind != kind || h.address != address)
			fatalerror("Z180 continuation diverged at %04x transaction %u", _PPC, m_cursor - 1);
		m_nominal += h.nominal;
		return h.data;
	}
	if (!m_cycle.active || m_cycle.owner != cycle_owner::cpu || m_cycle.kind == cycle_kind::release)
	{
		// The instruction's stable-input path has no DMA/refresh requester.
		// Every I/O or external input mutation invalidates that proof. Keep
		// explicit arbitration when observing edges or replaying a suspension.
		if (!m_instruction_fast || m_cycle_observer || m_sampled_bus_request || m_refresh_pending)
			cycle_arbitrate();
		while (m_icount <= 0)
		{
			if (!cycle_next_grant())
				cycle_suspend();
			cycle_arbitrate();
		}
		offs_t physical = address;
		if (kind == cycle_kind::fetch || kind == cycle_kind::operand || kind == cycle_kind::read || kind == cycle_kind::write ||
			kind == cycle_kind::nmi || kind == cycle_kind::halt)
			physical = MMU_REMAP_ADDR(address);
		cycle_begin(cycle_owner::cpu, kind, physical, data, no_arbitration);
		if (m_trapping && m_third_trap && kind == cycle_kind::read && m_bus_number == 5)
		{
			m_cycle.ttp = true;
			++m_cycle.nominal;
		}
	}
	cycle_run();
	if (m_history_count == std::size(m_history))
		fatalerror("Z180 instruction journal overflow at %04x", _PPC);
	m_history[m_history_count++] = {kind, address, m_cycle.data, m_cycle.nominal};
	++m_cursor;
	m_nominal += m_cycle.nominal;
	m_boundary_allowed = !m_cycle.no_arbitration;
	m_dma_allowed = !m_dma_start_pending;
	m_dma_start_pending = false;
	m_refresh_after_sleep = false;
	u8 const result = m_cycle.data;
	if (!m_instruction_fast && kind <= cycle_kind::write)
		cycle_resume_admission();
	if (m_cycle_reference)
		throw cycle_yield{};
	return result;
}

void z180_device::cycle_finish_cpu(int nominal)
{
	if (m_nominal > unsigned(nominal))
		fatalerror("Z180 cycle table overrun at %04x: %u > %u", _PPC, m_nominal, nominal);
	bool const admitted = m_instruction_whole || (!m_cycle_observer && !m_cycle.active && m_cursor == m_history_count && !m_trapping);
	if (admitted && m_instruction_fast && m_nominal < unsigned(nominal) && m_icount >= int(2 * (nominal - m_nominal)))
		cycle_whole_internal(nominal - m_nominal);
	while (m_nominal < unsigned(nominal))
		cycle_transaction(cycle_kind::internal, 0, 0);
	if (m_cursor != m_history_count)
		fatalerror("Z180 unused continuation at %04x", _PPC);
}

void z180_device::cycle_reti()
{
	cycle_flush_quiet();
	if (!m_cycle_executing || !m_reti_notified)
	{
		daisy_call_reti_device();
		m_reti_notified = true;
	}
}

bool z180_device::cycle_dma_ready(unsigned ch) const
{
	if (m_HALT == 2 || !(m_dstat & 1) || !(m_dstat & (0x40 << ch)))
		return false;
	unsigned route = 0;
	bool receive = false;
	if (!ch)
	{
		unsigned sm = (m_dmode >> 2) & 3, dm = (m_dmode >> 4) & 3;
		if (sm >= 2 && dm >= 2)
			return false;
		if (sm < 2 && dm < 2)
			return true;
		if (sm == 3)
		{
			route = (m_dma_sar0.d >> 16) & 3;
			receive = true;
		}
		else if (dm == 3)
			route = (m_dma_dar0.d >> 16) & 3;
	}
	else
	{
		route = (m_dma_iar1.d >> 16) & 7;
		receive = (m_dcntl & 2) != 0;
	}
	if (route == 1 || route == 2)
		return (m_asci[route - 1]->dma_request(receive));
	if (route)
		return false; // DS006002 p57: selectors 3-7 are reserved.
	return (m_dcntl & (4 << ch)) ? (m_dma_edge_sampled[ch] != m_dma_edge_accepted[ch]) : bool(m_sampled_iol & (0x800 << ch));
}

void z180_device::cycle_dma_begin(unsigned ch)
{
	m_dma_channel = ch;
	m_dma_stage = 0;
	if (!ch)
	{
		m_dma_source_mode = (m_dmode >> 2) & 3;
		m_dma_destination_mode = (m_dmode >> 4) & 3;
		m_dma_source = m_dma_sar0.d;
		m_dma_destination = m_dma_dar0.d;
		m_dma_source_io = m_dma_source_mode == 3;
		m_dma_destination_io = m_dma_destination_mode == 3;
	}
	else
	{
		bool receive = (m_dcntl & 2) != 0;
		m_dma_source = receive ? m_dma_iar1.d : m_dma_mar1.d;
		m_dma_destination = receive ? m_dma_mar1.d : m_dma_iar1.d;
		m_dma_source_io = receive;
		m_dma_destination_io = !receive;
		m_dma_source_mode = receive ? 3 : (m_dcntl & 1);
		m_dma_destination_mode = receive ? (m_dcntl & 1) : 3;
	}
	unsigned const req = ch;
	if (m_dcntl & (4 << req))
		m_dma_edge_accepted[req] = m_dma_edge_sampled[req];
}

void z180_device::cycle_dma_run()
{
	unsigned const ch = m_dma_channel;
	bool const writing = m_dma_stage != 0;
	bool const io = writing ? m_dma_destination_io : m_dma_source_io;
	u32 const address = writing ? m_dma_destination : m_dma_source;
	unsigned const mode = writing ? m_dma_destination_mode : m_dma_source_mode;
	if (!m_cycle.active)
	{
		cycle_begin(cycle_owner::dma,
			io ? (writing ? cycle_kind::output : cycle_kind::input) : (writing ? cycle_kind::write : cycle_kind::read),
			io ? u16(address) : address, m_dma_byte);
		if (mode < 2 && ((mode == 0 && (address & 0xffff) == 0xffff) || (mode == 1 && !(address & 0xffff))))
		{
			m_cycle.trailing = 1;
			++m_cycle.nominal;
		}
	}
	cycle_run();
	if (!writing)
		m_dma_byte = m_cycle.data;
	u32 const next = mode < 2 ? (address + (mode ? -1 : 1)) & 0xfffff : address;
	if (!ch)
	{
		if (writing)
			m_dma_dar0.d = next;
		else
			m_dma_sar0.d = next;
	}
	else if (!io)
		m_dma_mar1.d = next;
	m_boundary_allowed = true;
	if (!writing)
	{
		m_dma_stage = 1;
		return;
	}
	--m_dma_bcr[ch].w;
	if (!m_dma_bcr[ch].w)
	{
		m_dstat &= ~(0x40 << ch);
	}
	// Memory-to-memory cycle steal returns one MC to the CPU. I/O DMA
	// continues while the sampled level/next edge requests another byte
	// (UM0050 pp105-108); DMODE.MM applies only to memory-to-memory.
	if (m_dma_nmi)
	{
		m_dstat &= ~1;
		m_dma_nmi = false;
		// DMA-stop sampling is distinct from CPU interrupt sampling
		// (Zilog AN0066 pp6-6/6-8; HD64180 Oct 1985 p67). Preserve the
		// CPU sample: a later NMI edge waits for the next CPU sampling
		// point, even if the instruction before this steal has retired.
	}
	bool const memory = m_dma_source_mode < 2 && m_dma_destination_mode < 2;
	m_dma_continue = (!memory || (m_dmode & 2)) && cycle_dma_ready(ch) ? ch : 0xff;
	m_dma_channel = 0xff;
	m_dma_allowed = false;
}

void z180_device::cycle_arbitrate()
{
	cycle_flush_quiet();
	if (!m_boundary_allowed)
		return;
	for (;;)
	{
		if (m_cycle.active)
		{
			if (m_cycle.owner == cycle_owner::dma)
			{
				cycle_dma_run();
				continue;
			}
			if (m_cycle.owner == cycle_owner::refresh)
			{
				// Refresh owns the bus for this MC, but does not consume a
				// pending DMA grant (UM0050 pp105,281-282). In particular a
				// memory burst cannot return to the CPU between its bytes.
				cycle_run();
				++m_refresh_address;
				continue;
			}
		}
		if (m_sampled_bus_request || m_bus_acknowledged)
		{
			if (!m_bus_acknowledged)
			{
				m_bus_acknowledged = true;
				m_refresh_pending = false;
				if (m_busack_cb)
					m_busack_cb(true);
			}
			if (!m_cycle.active && !m_cycle_reference && !m_cycle_observer && m_bus_released && m_icount >= 2)
			{
				// BUSREQ is constant until this scheduler grant ends. Compose
				// its complete one-PHI release cycles with the exact divider
				// recurrences; leave an odd final half-PHI to the phase engine.
				// No IRQ is accepted or refresh cycle issued while BUSACK is
				// asserted. The next loop still constructs the pending cycle
				// at a zero budget, preserving its continuation state.
				unsigned const ticks = unsigned(m_icount) & ~1U;
				m_sampled_iol = m_iol;
				std::copy(std::begin(m_dma_edge_generation), std::end(m_dma_edge_generation), m_dma_edge_sampled);
				m_sampled_bus_request = true;
				m_icount -= ticks;
				m_half_phi += ticks;
				unsigned const phi = (m_phi_half + ticks) / 2;
				m_phi_half = (m_phi_half + ticks) & 1;
				handle_io_timers(phi);
				m_refresh_phase = (m_refresh_phase + ticks) % 160;
				m_cycle = {};
				m_cycle.kind = cycle_kind::release;
				m_cycle.nominal = 1;
				m_cycle.elapsed = 2;
				m_cycle.phase = 1;
			}
			else
			{
				if (!m_cycle.active)
					cycle_begin(cycle_owner::cpu, cycle_kind::release, 0);
				cycle_run();
			}
			// BUSREQ suspends DMA even between a byte's read and write;
			// release resumes that grant (UM0050 p113). Preserve both a
			// pending first byte and burst continuation. A cycle-steal byte
			// still clears its grant in cycle_dma_run, owing one MC to CPU.
			if (!m_sampled_bus_request)
			{
				m_bus_acknowledged = false;
				if (m_busack_cb)
					m_busack_cb(false);
			}
			continue;
		}
		if (m_refresh_pending && (m_rcr & 0x80) && m_HALT != 2 && !m_refresh_after_sleep)
		{
			m_refresh_pending = false;
			cycle_begin(cycle_owner::refresh, cycle_kind::refresh, m_refresh_address);
			continue;
		}
		if (m_dma_channel != 0xff)
		{
			cycle_dma_run();
			continue;
		}
		if (m_dma_continue != 0xff)
		{
			unsigned const ch = m_dma_continue;
			m_dma_continue = 0xff;
			if (cycle_dma_ready(ch))
			{
				cycle_dma_begin(ch);
				continue;
			}
		}
		if (m_dma_allowed)
		{
			if (cycle_dma_ready(0))
			{
				cycle_dma_begin(0);
				continue;
			}
			if (cycle_dma_ready(1))
			{
				cycle_dma_begin(1);
				continue;
			}
		}
		return;
	}
}

void z180_device::cycle_sample_interrupts()
{
	// UM0050 pp73,75,81: falling edge before T3/T1 of the final MC.
	// Sampling each CPU MC and retaining the last sample implements that
	// rule without predicting whether a conditional instruction will stop.
	m_sampled_interrupt = -1;
	if (m_nmi_pending)
	{
		m_sampled_interrupt = 1;
		return;
	}
	// Priority encoder: lower-priority sources need not be evaluated once
	// a higher-priority asserted source fixes the result.
	if (m_irq_state[0] && (m_itc & 1))
		m_sampled_interrupt = 2;
	else if (m_irq_state[1] && (m_itc & 2))
		m_sampled_interrupt = 3;
	else if (m_irq_state[2] && (m_itc & 4))
		m_sampled_interrupt = 4;
	else if ((m_tcr & 0x50) == 0x50)
		m_sampled_interrupt = 5;
	else if ((m_tcr & 0xa0) == 0xa0)
		m_sampled_interrupt = 6;
	else if ((m_dstat & 0x44) == 4)
		m_sampled_interrupt = 7;
	else if ((m_dstat & 0x88) == 8)
		m_sampled_interrupt = 8;
	else if (m_csio->check_interrupt())
		m_sampled_interrupt = 9;
	else if (m_asci[0]->check_interrupt())
		m_sampled_interrupt = 10;
	else if (m_asci[1]->check_interrupt())
		m_sampled_interrupt = 11;
}

int z180_device::cycle_choose_interrupt()
{
	if (m_after_EI || m_interrupt_recovery)
		return -1;
	int const irq = m_sampled_interrupt;
	if (irq >= 0 && m_HALT == 2)
	{
		// UM0050 p35: stopped internal I/O cannot wake SYSTEM STOP.
		if ((m_iocr & 0x20) && irq >= 5)
			return -1;
		m_refresh_after_sleep = true;
		LEAVE_HALT();
	}
	return irq == 1 || m_IFF1 ? irq : -1;
}

int z180_device::cycle_interrupt()
{
	int const irq = m_instruction_interrupt;
	LEAVE_HALT();
	if (irq == 1)
	{
		m_IFF2 = m_IFF1;
		m_IFF1 = 0;
		if (!m_interrupt_started)
		{
			m_interrupt_started = true;
			m_dstat &= ~1;
		}
		cycle_transaction(cycle_kind::nmi, _PC, 0);
		cycle_transaction(cycle_kind::internal, 0, 0);
		cycle_transaction(cycle_kind::internal, 0, 0);
		PUSH(PC);
		_PC = 0x66;
		return 11;
	}
	m_IFF1 = m_IFF2 = 0;
	u8 const vector = cycle_transaction(cycle_kind::acknowledge, _PC, 0);
	if (irq == 2 && m_IM == 0)
	{
		// UM0050 Table52 p279: subsequent operands are memory reads at PC,
		// PC+1; an inserted CALL stacks PC+2. Only the initial byte is ACK.
		++m_R;
		++m_bus_number;
		int const nominal = exec_op(vector);
		return nominal + m_extra_cycles + 2;
	}
	if (irq == 2 && m_IM == 1)
	{
		PUSH(PC);
		_PC = 0x38;
		return 11;
	}
	cycle_transaction(cycle_kind::internal, 0, 0);
	unsigned const low = irq == 2 ? vector : ((m_il & 0xe0) + (irq - 3) * 2) & 0xff;
	PUSH(PC);
	RM16((m_I << 8) | low, &m_PC);
	return 18;
}

// 1992 Zilog book pp374-377 (K): oscillator-stop recovery is 2^19
// clocks, not enhanced N's 2^17. Fig.B returns to STANDBY after a
// BREXT-only release. UM0050 pp37-41 supplies the enhanced modes only.
bool z180_device::cycle_low_power()
{
	if (m_HALT != 2 || !(m_iocr & 0x20))
		return false;
	unsigned const delay = power_recovery_ticks();
	if (!delay)
		return false; // SYSTEM STOP: PHI/FRC keep running.
	bool const irq = m_nmi_pending || (m_irq_state[0] && (m_itc & 1)) || (m_irq_state[1] && (m_itc & 2)) || (m_irq_state[2] && (m_itc & 4));
	if (m_power_bus_only)
	{
		// The oscillator already runs during this bus release; a newly
		// eligible interrupt exits sleep without restarting its counter.
		if (irq)
		{
			m_power_bus_only = false;
			cycle_sample_interrupts();
			m_refresh_after_sleep = true;
			LEAVE_HALT();
			return false;
		}
		cycle_arbitrate();
		if (m_icount <= 0)
			return true;
		m_power_bus_only = false; // BUSREQ released: oscillator stops again.
	}
	bool const wake = irq || ((power_control() & 0x20) && m_bus_released);
	if (!wake)
		m_power_wake_left = 0;
	else if (!m_power_wake_left)
		m_power_wake_left = delay;
	unsigned const ticks = wake ? std::min<unsigned>(m_icount, m_power_wake_left) : unsigned(m_icount);
	m_icount -= ticks;
	m_half_phi += ticks;
	m_io_service_stopped += ticks;
	// Neither PHI/10, refresh nor the shared service phase runs while
	// internal clocking is stopped, including oscillator stabilization.
	if (!wake)
		return true;
	m_power_wake_left -= ticks;
	if (m_power_wake_left)
		return true;
	m_refresh_pending = false;
	m_refresh_after_sleep = true;
	m_sampled_bus_request = m_bus_released;
	m_boundary_allowed = true;
	m_dma_allowed = false;
	if (!irq)
	{
		m_power_bus_only = true;
		// Publish BUSACK at the expiry edge even at a zero grant budget.
		m_bus_acknowledged = true;
		if (m_busack_cb)
			m_busack_cb(true);
		return true;
	}
	cycle_sample_interrupts();
	LEAVE_HALT();
	return false;
}

bool z180_device::cycle_halt()
{
	if (cycle_low_power())
		return true;
	// Resume the exact unfinished HALT/SLEEP cycle before accepting IRQ.
	if (m_cycle.active && m_cycle.owner == cycle_owner::cpu && m_cycle.kind != cycle_kind::release)
	{
		cycle_run();
		m_boundary_allowed = true;
		m_dma_allowed = m_HALT != 2;
	}
	cycle_arbitrate();
	if (m_icount <= 0)
		return true;
	m_instruction_interrupt = cycle_choose_interrupt();
	if (m_instruction_interrupt >= 0 || !m_HALT)
		return false;
	if (m_HALT == 2)
	{
		m_refresh_after_sleep = true;
		if (!m_cycle.active)
			cycle_begin(cycle_owner::cpu, cycle_kind::internal, 0);
	}
	else if (!m_cycle.active)
		cycle_begin(cycle_owner::cpu, cycle_kind::halt, MMU_REMAP_ADDR(u16(_PC + 1)));
	cycle_run();
	m_boundary_allowed = true;
	m_dma_allowed = m_HALT != 2;
	return true;
}

void z180_device::execute_machine_cycles()
{
	m_direct_reads_enabled = !machine().side_effects_disabled();
	m_fast_irq_valid = false;
	m_instruction_fast = false;
	// Reject a grant with no pending memory job before entering the helper.
	unsigned const resumed = m_cycle.active && m_cycle.kind <= cycle_kind::write && m_cycle.phase < 9 ? cycle_resume_quiet() : 0;
	if (resumed == 1)
		return;
	bool completion = resumed == 2;
	// One target per scheduler invocation, rather than C++ exception-table
	// searches on every half-PHI suspension. Resume uses the same journal.
	if (Z180_CONTINUATION_SAVE(m_cycle_resume))
	{
		m_cycle_executing = false;
		completion = false;
	}
next_grant:
	// Debugger/non-mutating execution keeps the address-space handlers.
	// This context belongs to the scheduler grant, not a hardware edge.
	m_direct_reads_enabled = !machine().side_effects_disabled();
	// Peripheral timers and inputs may have run since the previous slice,
	// including while HALT/SLP had no active instruction to restart.
	m_fast_irq_valid = false;
	m_instruction_fast = false;
	while (m_icount > 0 || completion)
	{
		completion = false;
		try
		{
			bool const resuming = m_instruction_active;
			if (!m_instruction_active)
			{
				// Use the existing HALT dispatch for power modes; ordinary
				// instructions do not query CCR, a timer or a virtual hook.
				if (m_HALT)
				{
					if (cycle_halt())
						continue;
				}
				else
				{
					// A debugger may clear HALT during its unfinished bus
					// cycle. Complete that cycle before the new instruction.
					if (m_cycle.active && m_cycle.owner == cycle_owner::cpu && m_cycle.kind != cycle_kind::release)
					{
						cycle_run();
						m_boundary_allowed = true;
						m_dma_allowed = true;
					}
					if (!m_instruction_fast || m_cycle_observer || m_sampled_bus_request || m_refresh_pending)
						cycle_arbitrate();
					if (m_icount <= 0)
					{
						cycle_flush_quiet();
						return;
					}
					m_instruction_interrupt = cycle_choose_interrupt();
				}
				if (m_instruction_interrupt == 1)
					m_nmi_pending = 0;
				m_after_EI = 0;
				cycle_save_state();
				m_instruction_active = true;
				m_history_count = 0;
				m_ack_vector = -1;
				m_reti_notified = m_trap_started = m_interrupt_started = false;
				if (m_instruction_interrupt < 0 && (machine().debug_flags & DEBUG_FLAG_CALL_HOOK))
				{
					cycle_flush_quiet();
					debugger_instruction_hook(_PCD);
					// Debugger state entries can change divider/pin registers
					// without passing through the CPU's external-input API.
					m_instruction_fast = false;
				}
				// The scheduler budget is the horizon for external pin/timer
				// events. Keep the handler running once through that horizon;
				// cycle_run checks the remaining budget before each unrolled
				// MC. A deadline or handler-triggered event yields a journaled
				// continuation rather than speculating past the event.
				if (m_instruction_fast && !m_cycle_observer)
				{
					// No callback or timed source has changed these inputs
					// since the last admitted instruction. Only the budget and
					// instruction-boundary interrupt decision are new.
					m_instruction_fast = m_instruction_interrupt < 0 && m_icount >= 6;
				}
				else
				{
					m_fast_irq_valid = false;
					m_instruction_fast = !m_cycle_reference && m_instruction_interrupt < 0 && m_icount >= 6 && !(m_dstat & 0xc0) &&
						!(m_rcr & 0x80) && !cycle_prt_timed() && !(m_dcntl & 0xf0) && !m_wait_line && !m_bus_released && !m_nmi_pending;
				}
				if (m_instruction_fast)
					++m_fast_instructions;
			}
			else
			{
				++m_resumptions;
				m_instruction_fast = false;
			}
			m_instruction_whole = false;
			if (resuming || (machine().debug_flags & DEBUG_FLAG_CALL_HOOK))
				cycle_restore_state();
			else
			{
				_PPC = m_instruction_start.regs[0];
				m_cursor = m_bus_number = m_nominal = 0;
				m_family = Z180_TABLE_op;
				m_opcode = 0;
				m_extra_cycles = 0;
				m_prelude_schedule = 0;
				m_trapping = m_third_trap = false;
			}
			m_cycle_executing = true;
			m_replay_pristine = true;
			int nominal;
			if (m_instruction_interrupt >= 0)
				nominal = cycle_interrupt();
			else
			{
				u8 opcode;
				if (m_instruction_fast && !m_cycle_observer)
				{
					++m_R;
					u16 const pc = _PC++;
					opcode = cycle_whole_access(cycle_kind::fetch, pc, 0, 0);
				}
				else
					opcode = ROP();
				nominal = exec_op(opcode);
				nominal += m_extra_cycles;
			}
			// When both counts already match, finish_cpu only rechecks them.
			// Keep its internal-cycle and error paths for every other case.
			if (m_nominal != unsigned(nominal) || m_cursor != m_history_count)
				cycle_finish_cpu(nominal);
			m_cycle_executing = false;
			m_instruction_active = false;
			++m_retired;
			m_interrupt_recovery = m_instruction_interrupt >= 0;
			if (m_cycle_observer)
				m_cycle_observer(m_half_phi, 0, 6, _PC, 0);
		}
		catch (cycle_yield const &)
		{
			m_cycle_executing = false;
		}
	}
	cycle_flush_quiet();
	// At instruction retirement there is no ISA frame to reconstruct. Keep
	// the entry frame while the scheduler closes this exact grant and runs
	// every intervening callback, then perform the normal grant admission.
	// Active bus jobs and HALT retain their existing continuation paths.
	if (m_cycle_grants && !m_cycle_reference && !m_instruction_active && !m_cycle.active && !m_HALT && !m_cycle_observer &&
		!(machine().debug_flags & (DEBUG_FLAG_ENABLED | DEBUG_FLAG_CALL_HOOK)) && scheduler().continue_domain_grant(*this))
		goto next_grant;
}

#undef Z180_CONTINUATION_SAVE
#undef Z180_CONTINUATION_RESTORE

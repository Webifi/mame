// license:BSD-3-Clause
// Hot instruction accesses. Keep these small and visible to the ISA handlers;
// mapped callbacks, partial cycles and replay stay in z180cycles.hxx.

inline void z180_device::cycle_defer_clocks(unsigned phi)
{
	assert(!cycle_prt_timed() && !(m_rcr & 0x80) && !m_advance_left && m_icount >= int(2 * phi));
	m_icount -= 2 * phi;
	m_half_phi += 2 * phi;
	m_quiet_phi += phi;
}

inline unsigned z180_device::cycle_prelude(cycle_kind kind) const
{
	unsigned const n = m_bus_number, op = m_opcode;
	if (!m_cycle_reference && !m_trapping)
	{
		if (m_family == Z180_TABLE_ed && op == 0x4d && !(m_omcr & 0x80))
		{
			if (n == 2)
				return 3;
			if (n == 3)
				return 1;
		}
		if ((m_family == Z180_TABLE_cb || m_family == Z180_TABLE_xycb) && kind != cycle_kind::write)
			return 0;
		return n < 8 ? (m_prelude_schedule >> (3 * n)) & 7 : 0;
	}
	return cycle_prelude_reference(kind);
}

inline void z180_device::cycle_sample_stable_inputs()
{
	m_sampled_iol = m_iol;
	std::copy(std::begin(m_dma_edge_generation), std::end(m_dma_edge_generation), m_dma_edge_sampled);
	m_sampled_bus_request = m_bus_released;
	cycle_sample_interrupts();
	m_fast_interrupt = m_sampled_interrupt;
	m_fast_irq_valid = true;
}

inline u8 z180_device::cycle_certified_read(cycle_kind kind, offs_t address, u8 const *direct)
{
	// The deadline proof excludes input changes; certification excludes
	// callbacks/waits. Drive the data bus once and retain the exact journal.
	if (!m_fast_irq_valid)
		cycle_sample_stable_inputs();
	u8 const data = *direct;
	*m_direct_read_bus = data;
	cycle_defer_clocks(3);
	if (m_history_count == std::size(m_history))
		fatalerror("Z180 instruction journal overflow at %04x", _PPC);
	m_history[m_history_count++] = {kind, address, data, 3};
	m_quiet_record = m_history_count - 1;
	m_quiet_no_arbitration = false;
	++m_cursor;
	m_nominal += 3;
	m_boundary_allowed = true;
	m_dma_allowed = !m_dma_start_pending;
	m_dma_start_pending = false;
	m_refresh_after_sleep = false;
	return data;
}

inline u8 z180_device::cycle_whole_access(cycle_kind kind, offs_t address, u8 data, unsigned prelude)
{
	if (prelude)
		cycle_whole_internal(prelude, m_family == Z180_TABLE_op && m_opcode == 0x10);
	++m_bus_number;
	if (kind != cycle_kind::write && m_direct_reads_enabled)
	{
		if (auto const page = m_direct_logical[(address >> 12) & 15])
			return cycle_certified_read(kind, address, page + (address & 4095));
	}
	offs_t const physical = MMU_REMAP_ADDR(address);
	return cycle_mapped_access(kind, address, physical, data);
}

inline u8 z180_device::cycle_access(cycle_kind kind, offs_t address, u8 data)
{
	unsigned const pre = cycle_prelude(kind);
	// A whole-instruction proof also certifies subsequent memory cycles.
	// Otherwise admit only this complete cycle before the next deadline.
	bool const admitted = m_instruction_whole || (!m_cycle_observer && !m_cycle.active && m_cursor == m_history_count);
	if (admitted && m_instruction_fast && !m_trapping && kind <= cycle_kind::write && m_icount >= int(2 * (pre + 3)))
		return cycle_whole_access(kind, address, data, pre);
	return cycle_access_slow(kind, address, data, pre);
}

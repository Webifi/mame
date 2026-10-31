// license:BSD-3-Clause
// Sound GLU misuse log for IIgs developers. It exists only while MAME
// logging is enabled (-log or the debugger), and it never changes timing,
// memory routing or saved state.
class apple2gs_state::trace_state
{
public:
	trace_state(apple2gs_state &owner) : m_owner(owner) { }
	void reset()
	{
		m_glu_log_context = {}; m_glu_log_instruction_pc = 0; m_glu_log_access_bank = 0;
		m_glu_log_intended_address = m_owner.m_sndglu_addr;
		m_glu_log_intended_control = m_owner.m_sndglu_ctrl;
		m_glu_log_last_cycle = -1; m_glu_ram = 0;
		std::fill(std::begin(m_glu_log_counts), std::end(m_glu_log_counts), 0);
		std::fill(&m_glu_land[0][0], &m_glu_land[0][0] + 32 * 7, 0);
		std::fill(&m_glu_repl[0][0], &m_glu_repl[0][0] + 32 * 7, 0);
		m_glu_log_summary_at = m_owner.machine().time() + attotime::from_seconds(10);
	}
	template <typename... T> void emit(const char *format, T &&...args)
	{
		m_owner.logerror(format, std::forward<T>(args)...);
	}
	struct glu_log_context
	{
		attotime time;
		int64_t gap = -1;
		u32 pc = 0, cpu_hz = 0, accelerator_hz = 0;
		u8 bank = 0;
		const char *accelerator = "none";
	};
	glu_log_context m_glu_log_context;
	u32 m_glu_log_instruction_pc = 0;
	u8 m_glu_log_access_bank = 0;
	u16 m_glu_log_intended_address = 0;
	u8 m_glu_log_intended_control = 15;
	int64_t m_glu_log_last_cycle = -1;
	u64 m_glu_log_counts[6]{};
	attotime m_glu_log_summary_at;
	u32 m_glu_ram = 0;
	u32 m_glu_land[32][7]{}, m_glu_repl[32][7]{};
	void sndglu_log_note(u32 address, int type);
	std::string sndglu_log_prefix(const glu_log_context &event) const;
	std::string sndglu_log_cpu_state();
	std::string sndglu_log_stack();
	void sndglu_log_event(unsigned kind, std::string details);
	void sndglu_log_access(unsigned port, bool read, u8 data, u8 result = 0);
	void sndglu_log_summary();

	void sndglu_tally(u16 addr, bool land);
	void sndglu_log_table();
private:
	apple2gs_state &m_owner;
	static constexpr char const *classes[] = { "flo", "fhi", "vol", "ptr", "ctl", "siz", "oth" };
};

bool apple2gs_state::trace_memory(u32 address, int type, u8 &data)
{
	m_trace->sndglu_log_note(address, type);
	// The logging hook must keep the same accelerator memory route.
	return m_twgs_real && m_twgs ? twgs_memory(address, type, data) : false;
}

// Capture the instruction PC and access bank at the CPU bus hook. A later
// deferred GLU write uses this context; the CPU may have advanced by then.
void apple2gs_state::trace_state::sndglu_log_note(u32 address, int type)
{
	if (type == g65816_device::BUS_OPCODE)
		m_glu_log_instruction_pc = address;
	if ((address & 0xfffc) == 0xc03c)
		m_glu_log_access_bank = address >> 16;
}

std::string apple2gs_state::trace_state::sndglu_log_prefix(const glu_log_context &event) const
{
	const std::string gap = event.gap < 0 ? "first" : util::string_format("%lld", (long long)event.gap);
	return util::string_format("Sound GLU t=%.9f PC=%02X:%04X bank=%02X gap=%s bus cycles CPU=%.3fMHz accelerator=%s/%.3fMHz: ",
		event.time.as_double(), event.pc >> 16, event.pc & 0xffff, event.bank, gap,
		event.cpu_hz / 1.0e6, event.accelerator, event.accelerator_hz / 1.0e6);
}

std::string apple2gs_state::trace_state::sndglu_log_cpu_state()
{
	const unsigned p = m_owner.m_maincpu->state_int(g65816_device::G65816_P);
	return util::string_format("A=%04X X=%04X Y=%04X S=%04X D=%04X DBR=%02X P=%02X (m=%u x=%u e=%u)",
		unsigned(m_owner.m_maincpu->state_int(g65816_device::G65816_A)),
		unsigned(m_owner.m_maincpu->state_int(g65816_device::G65816_X)),
		unsigned(m_owner.m_maincpu->state_int(g65816_device::G65816_Y)),
		unsigned(m_owner.m_maincpu->state_int(g65816_device::G65816_S)),
		unsigned(m_owner.m_maincpu->state_int(g65816_device::G65816_D)),
		unsigned(m_owner.m_maincpu->state_int(g65816_device::G65816_DB)), p,
		BIT(p, 5), BIT(p, 4), unsigned(m_owner.m_maincpu->state_int(g65816_device::G65816_E)));
}

std::string apple2gs_state::trace_state::sndglu_log_stack()
{
	// Pushed data can resemble return addresses. Verify a preceding call
	// opcode when possible, scan upward from S, and never execute CPU hooks.
	auto const disable = m_owner.machine().disable_side_effects();
	auto &space = m_owner.m_maincpu->space(AS_PROGRAM);
	const u16 s = m_owner.m_maincpu->state_int(g65816_device::G65816_S);
	const bool emulation = m_owner.m_maincpu->state_int(g65816_device::G65816_E);
	auto stack_byte = [&space, s, emulation](unsigned offset)
	{
		const u16 address = emulation ? 0x100 | ((s + offset) & 0xff) : u16(s + offset);
		return space.read_byte(address);
	};
	unsigned bank = m_glu_log_context.pc >> 16, found = 0;
	std::string text;
	for (unsigned offset = 1; offset <= 62 && found < 3; ++offset)
	{
		const u16 saved = stack_byte(offset) | (u16(stack_byte(offset + 1)) << 8);
		const unsigned long_bank = stack_byte(offset + 2);
		const bool jsl = space.read_byte((long_bank << 16) | u16(saved - 3)) == 0x22;
		const u8 short_opcode = space.read_byte((bank << 16) | u16(saved - 2));
		const bool jsr = short_opcode == 0x20 || short_opcode == 0xfc;
		if (!jsl && !jsr)
			continue;
		if (jsl)
			bank = long_bank;
		if (found++)
			text += ", ";
		text += util::string_format("%02X:%04X (%s at S+%u)", bank, u16(saved + 1), jsl ? "JSL" : "JSR", offset);
		offset += jsl ? 2 : 1;
	}
	return text.empty() ? "no plausible return addresses in 64 bytes" : text;
}

void apple2gs_state::trace_state::sndglu_log_event(unsigned kind, std::string details)
{
	++m_glu_log_counts[kind];
	const std::string prefix = sndglu_log_prefix(m_glu_log_context);
	const std::string state = "; " + sndglu_log_cpu_state();
	emit("%s%s%s\n", prefix, details, state);
	static const char *const kinds[] = { "busy data write", "busy control write", "busy address-low write", "busy address-high write", "busy data read", "data setup change" };
	emit("%sCPU/stack for %s%s; call trace (best guess; pushed data may look like returns): %s\n",
		prefix, kinds[kind], state, sndglu_log_stack());
}

void apple2gs_state::trace_state::sndglu_log_summary()
{
	if (std::none_of(std::begin(m_glu_log_counts), std::end(m_glu_log_counts), [](u64 count) { return count != 0; }))
		return;
	emit("%slast interval: busy data writes=%llu, busy control writes=%llu, busy address-low writes=%llu, busy address-high writes=%llu, busy data reads=%llu, setup changes=%llu\n",
		sndglu_log_prefix(m_glu_log_context), (unsigned long long)m_glu_log_counts[0], (unsigned long long)m_glu_log_counts[1],
		(unsigned long long)m_glu_log_counts[2], (unsigned long long)m_glu_log_counts[3],
		(unsigned long long)m_glu_log_counts[4], (unsigned long long)m_glu_log_counts[5]);
	std::fill(std::begin(m_glu_log_counts), std::end(m_glu_log_counts), 0);
}

void apple2gs_state::trace_state::sndglu_log_access(unsigned port, bool read, u8 data, u8 result)
{
	if (m_owner.machine().side_effects_disabled())
		return;
	const u64 slot = m_owner.sndglu_slot();
	const attotime now = m_owner.bt_at(slot + ((read && port == 13) ? 7 * BT_CLK : sndglu_slot_len(slot)));
	const u64 ticks = now.as_ticks(A2GS_14M);
	const int64_t cycle = (ticks / 912) * 65 + std::min<u64>((ticks % 912) / 14, 64);
	m_glu_log_context = { now, m_glu_log_last_cycle < 0 ? -1 : cycle - m_glu_log_last_cycle,
		m_glu_log_instruction_pc, u32(m_owner.m_bt_twslow ? BT_PER_SEC / BT_FAST : (m_owner.m_bt_mode == BT_OFF ? m_owner.m_maincpu->clock() : BT_PER_SEC / m_owner.m_bt_cycle)),
		m_owner.m_accel_present ? m_owner.m_accel_speed : 0, m_glu_log_access_bank, m_owner.m_accel_present ? (m_owner.m_twgs ? "TransWarp GS" : "ZipGS") : "none" };
	m_glu_log_last_cycle = cycle;
	if (now >= m_glu_log_summary_at)
	{
		sndglu_log_summary();
		m_glu_log_summary_at = now + attotime::from_seconds(10);
	}
	auto destination = [](u16 address, bool ram)
	{
		return util::string_format("%s $%04X", ram ? "sound RAM" : "DOC register", ram ? address : address & 0xff);
	};
	const bool busy = m_owner.m_sndglu_pending && now < m_owner.m_sndglu_busy_until;
	if (busy && !read && port != 13)
	{
		const u16 next = port == 12 ? (BIT(data, 6) ? m_owner.m_sndglu_addr : m_owner.m_sndglu_addr & 0xff)
			: port == 14 ? (m_owner.m_sndglu_addr & 0xff00) | data : (m_owner.m_sndglu_addr & 0xff) | (u16(data) << 8);
		sndglu_log_event(port == 12 ? 1 : port - 12,
			util::string_format("$C03%X write $%02X while busy; local register changes; pending %s; candidate target %s (was %s); see address-lock state",
				port, data, m_owner.m_sndglu_pend_read ? "read" : "write", destination(next, m_owner.m_sndglu_pend_ram), destination(m_owner.m_sndglu_addr, m_owner.m_sndglu_pend_ram)));
	}
	if (busy && port == 13)
	{
		const std::string requested = destination(m_glu_log_intended_address, BIT(m_glu_log_intended_control, 6));
		const std::string target = destination(m_owner.m_sndglu_addr, BIT(m_owner.m_sndglu_ctrl, 6));
		if (read)
			sndglu_log_event(4, util::string_format("$C03D read while busy returned $%02X; requested read of %s at the existing deadline; direction may already be committed; program requested %s", result, target, requested));
		else
		{
			const std::string old = m_owner.m_sndglu_pend_read ? "pending read" : util::string_format("pending byte $%02X", m_owner.m_sndglu_pend_data);
			sndglu_log_event(0, util::string_format("$C03D write $%02X while busy updates holding latch (%s); current target %s; direction/address may already be committed; program requested %s", data, old, target, requested));
			if (!m_owner.m_sndglu_landed && !m_owner.m_sndglu_pend_read && !m_owner.m_sndglu_pend_ram && data != m_owner.m_sndglu_pend_data && m_owner.m_sndglu_transfer_at - now < attotime::from_nsec(150))
				sndglu_log_event(5, util::string_format("DOC data changed $%02X -> $%02X inside 150ns setup; %.1fns before transfer to %s", m_owner.m_sndglu_pend_data, data, (m_owner.m_sndglu_transfer_at - now).as_double() * 1.0e9, target));
		}
	}
	if (port == 13 && BIT(m_glu_log_intended_control, 5))
		++m_glu_log_intended_address;
	else if (!read && port == 12)
		m_glu_log_intended_control = data & 0x7f;
	else if (!read && port == 14)
		m_glu_log_intended_address = (m_glu_log_intended_address & 0xff00) | data;
	else if (!read && port == 15)
		m_glu_log_intended_address = (m_glu_log_intended_address & 0xff) | (u16(data) << 8);
}

void apple2gs_state::trace_state::sndglu_tally(u16 addr, bool land)
{
	const int osc = addr & 0x1f;
	int cls;
	switch ((addr >> 5) & 7)
	{
		case 0: cls = 0; break;
		case 1: cls = 1; break;
		case 2: cls = 2; break;
		case 4: cls = 3; break;
		case 5: cls = 4; break;
		case 6: cls = 5; break;
		default: cls = 6; break;
	}
	if (land)
		m_glu_land[osc][cls]++;
	else
		m_glu_repl[osc][cls]++;
}

void apple2gs_state::trace_state::sndglu_log_table()
{
	emit("Sound GLU vca %02x\n", m_owner.m_sndglu_ctrl & 0x0f);
	emit("Sound GLU ram bytes %u\n", m_glu_ram);
	for (int osc = 0; osc < 32; osc++)
	{
		bool any = false;
		for (int c = 0; c < 7; c++)
			any = any || m_glu_land[osc][c] || m_glu_repl[osc][c];
		if (!any)
			continue;
		// One logerror call per line: each call starts a new prefixed log line.
		std::string line = util::string_format("Sound GLU osc %02d", osc);
		for (int c = 0; c < 7; c++)
			line += util::string_format(" %s %u/%u", classes[c], m_glu_land[osc][c], m_glu_repl[osc][c]);
		emit("%s\n", line);
	}
}


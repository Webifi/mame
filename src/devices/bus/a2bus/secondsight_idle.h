// license:BSD-3-Clause
#pragma once
#include "secondsight_lattice.h"
#include <cstdint>

// Closed form of the Lattice chip for a stable read outside its active decodes. Only CPU_DELAY,
// BANK_WINDOW and BANK_HELD can change. AX transactions are materialized
// separately, before changing port/host_data. The full equations establish
// and verify the precondition; address/data changes and active I/O never
// enter this path. A selected neutral read retains its first/final edges.
struct secondsight_idle
{
	using L = secondsight_lattice;

	static bool host_meeting_address(std::uint16_t address, bool read, bool corrected)
	{
		// Superset of EVENT_DECODE/ADOUT_ACTIVE in both generated Lattice maps.
		// Lattice v2.0 adds the IIgs border write at C034; it must reach the host fence.
		const unsigned group = address & 0xfff0;
		return group == 0xc0b0 || group == 0xc050 ||
			(!read && (group == 0xc000 || group == 0xc020 || group == 0xc070 || (corrected && address == 0xc034)));
	}

	static bool read_address(std::uint16_t address)
	{
		// Lattice chip read-sensitive decodes: HIRES/PAGE2 and EVENT at C05x,
		// ADOUT/EVENT at C0Bx, and PAGE2/RAMWRT load at C068.
		return (address & 0xff00) != 0xc000 || ((address & 0xfff0) != 0xc050 && (address & 0xfff0) != 0xc0b0 && address != 0xc068);
	}

	static bool transition_address(std::uint16_t address)
	{
		// Old C0_VALID feedback can remain high in the first settle round.
		return read_address(0xc000 | (address & 255));
	}

	static bool quiet(L const &l, L::inputs const &p)
	{
		return p.read && read_address(p.address) && !p.M2B0 && !p.AX && l.q[L::RUN] && !l.q[L::IRQ_PENDING] && !l.q[L::EVENT_LATCH] &&
			!l.q[L::CAPTURE] && !l.q[L::ADIN_ACTIVE] && !l.q[L::CATCH_ACTIVE] && !l.q[L::STALL];
	}

	static std::uint64_t start(std::uint64_t n)
	{
		const auto line = n / 912 * 912, pos = n - line;
		return line + (pos < 896 ? pos / 14 * 14 : 896);
	}

	static bool cpu(std::uint64_t n)
	{
		const auto s = start(n);
		return n - s >= (s % 912 == 896 ? 9 : 7);
	}

	static std::uint64_t next_window(std::uint64_t n)
	{
		auto s = start(n);
		if (n < s + 1)
			return s + 1;
		return s + (s % 912 == 896 ? 16 : 14) + 1;
	}

	static void advance(L &l, L::inputs &p, std::uint64_t before, std::uint64_t end)
	{
		if (end <= before)
			return;
		const bool old_delay = l.q[L::CPU_DELAY];
		const auto fall = (end & 1) ? end : end - 1;
		if (fall > before)
		{
			const bool previous = fall >= before + 3 ? cpu(fall - 2) : l.q[L::CPU_DELAY];
			l.q[L::BANK_WINDOW] = previous && !cpu(fall);
			l.q[L::CPU_DELAY] = cpu(fall);
		}
		// The first low-phase falling 7M edge opens BANK_WINDOW, clearing the
		// held bank under M2B0=0. Also cover the initial sampled history.
		const auto first = (before & 1) ? before + 2 : before + 1;
		if ((first <= end && !cpu(first) && old_delay) || next_window(first) <= end)
			l.q[L::BANK_HELD] = l.q[L::BANK_HELD_ALIAS] = false;
		p.CPU = cpu(end);
		p.Y0 = p.Y2 = !(end & 1);
		for (auto signal : {L::HIRES, L::CPU_DELAY, L::BANK_WINDOW, L::PAGE2, L::RAMWRT, L::STORE80})
			l.clocks[signal] = !p.Y0;
	}
};

// license:BSD-3-Clause
#pragma once
#include "secondsight_idle.h"
#include <array>

// Timestamped changes between neutral reads. No input edge is merged: the
// old decode advances through k-1, then the full equations see the new pins
// and clock k together. AX accesses materialize only their elapsed prefix.
struct secondsight_idle_queue
{
	static constexpr unsigned SIZE = 4096;
	std::array<std::uint64_t, SIZE> clock{};
	std::array<std::uint32_t, SIZE> pins{};
	unsigned head = 0, count = 0;

	static std::uint32_t pack(std::uint16_t address, std::uint8_t data, bool unselected)
	{
		return address | (std::uint32_t(data) << 16) | (std::uint32_t(unselected) << 24);
	}

	std::uint32_t last(secondsight_lattice::inputs const &p) const
	{
		return count ? pins[(head + count - 1) % SIZE] : pack(p.address, p.data, p.M2SEL_n);
	}

	bool room() const { return count < SIZE; }

	void push(std::uint64_t tick, std::uint32_t value, secondsight_lattice::inputs const &p)
	{
		if (value == last(p))
			return;
		const unsigned at = (head + count) % SIZE;
		clock[at] = tick;
		pins[at] = value;
		++count;
	}

	template <class Lattice> bool advance(Lattice &l, secondsight_lattice::inputs &p, std::uint64_t &before, std::uint64_t end)
	{
		while (count && clock[head] <= end)
		{
			const auto tick = clock[head];
			const auto value = pins[head];
			if (tick > before + 1)
				secondsight_idle::advance(l, p, before, tick - 1);
			p.address = value & 0xffff;
			p.data = (value >> 16) & 255;
			p.M2SEL_n = (value >> 24) & 1;
			p.CPU = secondsight_idle::cpu(tick);
			p.Y0 = p.Y2 = !(tick & 1);
			if (!l.settle(p))
				return false;
			before = tick;
			head = (head + 1) % SIZE;
			--count;
		}
		secondsight_idle::advance(l, p, before, end);
		before = end;
		return true;
	}
};

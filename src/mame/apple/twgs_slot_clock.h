// license:BSD-3-Clause
#pragma once
#include <cstdint>
#include <array>

namespace twgs_decoded
{
// Apple IIgs Hardware Reference, figures 2-2 and 8-8; TN68, M2B0.
// Every 65th slot extends PH0 LOW by two 14M periods. The high phase
// remains seven periods. 7M rises on even 14M boundaries, and Q3 is
// high for the first four periods of each ordinary half-cycle.
// Only high-phase Q3 is consumed by GAL8; its low-phase value is not
// used to infer any additional signal on the stretched cycle.
template <uint64_t Scale> struct slot_clock
{
	static constexpr uint64_t CLK = 1056 * Scale, MEGA = 14 * CLK, LINE = 912 * CLK;

	struct pins
	{
		bool ph0, m7, q3, bank;
		uint64_t start, end;
	};

	// The pin levels repeat after 912 14M periods, including the two extra
	// LOW periods in slot 64. Store the exact recurrence at each period;
	// sub-period timestamps have the same pins and slot boundaries.
	inline static constexpr auto phases = []
	{
		std::array<uint8_t, 912> result{};
		for (unsigned p = 0; p < result.size(); ++p)
		{
			const unsigned start = p < 896 ? p / 14 * 14 : 896;
			const unsigned phase = p - start, high = start == 896 ? 9 : 7;
			const bool ph0 = phase >= high;
			result[p] = unsigned(ph0) | (unsigned(ph0 ? phase - high < 4 : phase < 4) << 1) | (unsigned(phase >= 2 && phase < 4) << 2) |
				(phase << 3) | (unsigned(start == 896) << 7);
		}
		return result;
	}();

	static pins at(uint64_t t) noexcept
	{
		const uint64_t tick = t / CLK;
		const unsigned code = phases[tick % 912];
		const uint64_t start = (tick - ((code >> 3) & 15)) * CLK;
		return {bool(code & 1), !(tick & 1), bool(code & 2), bool(code & 4), start, start + ((code & 128) ? 16 : 14) * CLK};
	}
};
} // namespace twgs_decoded

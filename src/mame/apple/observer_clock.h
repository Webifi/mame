// license:BSD-3-Clause
#ifndef MAME_APPLE_OBSERVER_CLOCK_H
#define MAME_APPLE_OBSERVER_CLOCK_H

#pragma once

#include "attotime.h"

namespace apple2gs_observer
{

// Keep the next access boundary fixed when a rate changes in the middle
// of an instruction. Earlier accesses used the old stride, not the new one.
inline u64 rebase(u64 start, u64 accesses, u64 old_stride, u64 new_stride)
{
	return start + accesses * old_stride - accesses * new_stride;
}

// Convert the existing bus-accounting coordinate using the scheduler's
// accumulated CPU period. Do not introduce a second, absolute clock rate.
inline attotime span(u64 ticks, u64 period, u64 divisor)
{
	const u64 clocks = ticks / divisor;
	const attotime unit(0, period);
	attotime result = unit * u32(clocks);
	if (clocks >> 32)
		result += ((unit * u32(clocks >> 32)) * 65536U) * 65536U;
	return result + attotime(0, muldivu_64(ticks % divisor, period, divisor));
}

inline attotime at(u64 ticks, attotime origin, u64 origin_tick, u64 period, u64 divisor)
{
	return ticks < origin_tick ? origin - span(origin_tick - ticks, period, divisor) : origin + span(ticks - origin_tick, period, divisor);
}

// Invert the same representable edge, including elapsed CPU time spanning
// clock changes. The two divisions bound the result within two CPU clocks.
inline u64 elapsed_ticks(attotime elapsed, u64 period, u64 divisor)
{
	assert(elapsed >= attotime::zero);
	u64 low = (muldivu_64(elapsed.seconds(), ATTOSECONDS_PER_SECOND, period) + u64(elapsed.attoseconds()) / period) * divisor;
	u64 high = low + 2 * divisor;
	while (low < high)
	{
		const u64 mid = low + (high - low + 1) / 2;
		if (span(mid, period, divisor) <= elapsed)
			low = mid;
		else
			high = mid - 1;
	}
	return low;
}

} // namespace apple2gs_observer

#endif // MAME_APPLE_OBSERVER_CLOCK_H

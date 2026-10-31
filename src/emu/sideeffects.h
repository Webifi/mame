// license:BSD-3-Clause
#ifndef MAME_EMU_SIDEEFFECTS_H
#define MAME_EMU_SIDEEFFECTS_H

#pragma once

#include <atomic>
#include <cassert>
#include <cstdint>

// A zero total proves that no execution domain has suppressed side effects.
// A nonzero total still requires the original domain-local lookup. Keep the
// total process-wide, including nested machine contexts, so this shortcut
// cannot change which domain the existing lookup selects.
class side_effects_scopes
{
public:
	static bool any() noexcept { return s_active.load(std::memory_order_relaxed) != 0; }

	static void enter(std::uint32_t &depth) noexcept
	{
		s_active.fetch_add(1, std::memory_order_relaxed);
		++depth;
	}

	static void leave(std::uint32_t &depth) noexcept
	{
		assert(depth);
		--depth;
		[[maybe_unused]] auto const previous = s_active.fetch_sub(1, std::memory_order_relaxed);
		assert(previous);
	}

private:
	// Enter increments before changing the local depth; leave decrements
	// after it. An active scope's own thread cannot read a zero preceding
	// its increment (atomic write/read coherence), even with relaxed order.
	// Other threads can only make the zero shortcut more conservative.
	inline static std::atomic<unsigned> s_active{0};
};

#endif // MAME_EMU_SIDEEFFECTS_H

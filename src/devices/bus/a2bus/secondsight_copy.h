// license:BSD-3-Clause
#ifndef MAME_BUS_A2BUS_SECONDSIGHT_COPY_H
#define MAME_BUS_A2BUS_SECONDSIGHT_COPY_H
#pragma once

// U17 copy controller, approximation of the undumped 18CV8.
// CATCH clocks the external latches independently of this controller.
// Never queue bytes here.
struct secondsight_copy_controller
{
	// Approximations: request on a
	// rising CATCH, one synchronization stage and a registered write start.
	// Use the shortest registered pulse meeting the SRAM's 60ns minimum at
	// 18MHz. Neither this sequence nor a copy-loss threshold is recovered.
	static constexpr unsigned sync_stages = 1;
	static constexpr unsigned write_start_stages = 1;
	static constexpr unsigned write_phi = 2;
	// Approximation: ideal registered edges, without an invented GAL
	// propagation delay. A CATCH while busy sets one pending bit; further
	// edges coalesce. Whether the physical controller does this is unknown.
	static constexpr bool catch_while_busy = true;
	// U15 Z180 memory decoder has no read/write input. Approximation: three
	// waits, the minimum common count meeting Oak's worst-case write timing
	// at 18MHz. A four-wait registered design also fits (05-z180, "Wait states").
	static constexpr unsigned forced_memory_waits = 3;
	// Convention for a CATCH during /WR: retain the write made on /WR's
	// falling edge; service the latest new latch contents in another pulse.
	// Real SRAM contents after changing its address under /WR are undefined.
	static constexpr bool retain_write_at_start = true;
	enum class phase : u8 { idle, synchronize, busack, write_start, write_end };
	enum class action { request, write, release };
	phase state = phase::idle;
	bool pending = false;
	attotime deadline = attotime::never;

	bool active() const { return state != phase::idle; }
	void capture(attotime next_phi, attotime phi)
	{
		if (state != phase::idle && !catch_while_busy)
			return;
		pending = true; // one edge latch; later CATCH overwrites the external latches
		if (state == phase::idle)
		{
			state = phase::synchronize;
			deadline = next_phi + phi * (sync_stages - 1);
		}
	}
	void grant(attotime next_phi, attotime phi)
	{
		if (state == phase::busack)
		{
			state = phase::write_start;
			deadline = next_phi + phi * (write_start_stages - 1);
		}
	}
	action tick(attotime next_phi, attotime phi)
	{
		switch (state)
		{
		case phase::synchronize:
			state = phase::busack;
			deadline = attotime::never;
			return action::request;
		case phase::write_start:
			state = phase::write_end;
			pending = false;
			deadline += phi * write_phi;
			return action::write;
		case phase::write_end:
			state = phase::idle;
			deadline = attotime::never;
			if (pending)
				capture(next_phi, phi);
			return action::release;
		default:
			throw emu_fatalerror("U17 copy controller clock without a pending edge");
		}
	}
};
ALLOW_SAVE_TYPE(secondsight_copy_controller::phase);
#endif

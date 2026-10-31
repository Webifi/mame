// license:BSD-3-Clause
#pragma once
#include "twgs_card.h"

namespace twgs_decoded
{
// SLOT_RDY is not wired to the accelerated CPU's RDY pin. GAL8 pin 15
// qualifies it on SLOT_7M rising edges; U27C ANDs that state with GAL8's
// DMA permission. U30A samples the result on X4_LATE, on the SAME edge
// as GAL2, which consequently sees the previous U30A output.
//
// Sources: TransWarp GS schematic U27C/U30A and decoded GAL8A/B
// pin-15 product terms, which are identical in both revisions. The supplied
// PH0/Q3 levels are after the board's RC phase conditioning; this model
// adds no invented propagation interval or extra synchronizer.
struct slot_ready
{
	bool h = true, dma = true, u30 = true;

	bool gs_ok() const noexcept { return h && dma; }

	void slot_7m_rise(bool ph0, bool q3, bool rdy) noexcept
	{
		if (ph0)
			h = q3 || (h && rdy);
	}

	template <class Next> void gs_edge(card &c, bool level, Next &&next) noexcept
	{
		// U23 sees U27C directly, not the delayed U30A output. Q1 samples on
		// every GS fall; Q2/Q4 use their feedback inputs while gs_ok is low.
		c.ctl.pause = gs_ok();
		c.gs_edge(level, next);
		c.ctl.pause = u30;
	}

	template <class Next> void late_edge(card &c, Next &&next) noexcept
	{
		c.ctl.pause = u30;
		c.late_edge(next);
		u30 = gs_ok();
		c.ctl.pause = u30;
	}
};
} // namespace twgs_decoded

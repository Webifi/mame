// license:BSD-3-Clause
// copyright-holders:MAMEdev Team
#ifndef MAME_RENDER_BGFX_INTERLACE_H
#define MAME_RENDER_BGFX_INTERLACE_H

#pragma once

#include <cstdint>

namespace bgfx_interlace {

// A cosmetic field mask on a private BGRA8 texture upload.  The selected
// field's samples and their positions are unchanged.  The CRT chain draws
// scanlines only where this field has light; the other field is black.
inline void select_field(std::uint8_t *pixels, unsigned pitch, unsigned height, unsigned field)
{
	for (unsigned y = field ^ 1; y < height; y += 2)
	{
		std::uint8_t *const row = pixels + y * pitch;
		for (unsigned x = 0; x < pitch; x += 4)
		{
			row[x] = row[x + 1] = row[x + 2] = 0;
			row[x + 3] = 255;
		}
	}
}

} // namespace bgfx_interlace

#endif // MAME_RENDER_BGFX_INTERLACE_H

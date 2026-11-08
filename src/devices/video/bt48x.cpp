// license: BSD-3-Clause
// copyright-holders: Dirk Best
/***************************************************************************

    Brooktree Bt481/Bt482

    256 Color, 15-bit, 16-bit, 24-bit RAMDAC

    Notes:
    - Bt482 additionally supports a custom shaped 32x32x2 cursor

    TODO:
    - Cursor mode 2 and 3

***************************************************************************/

#include "emu.h"
#include "bt48x.h"

#define LOG_ACCESS   (1U << 1) // logs all access to handlers
#define LOG_INDIRECT (1U << 2) // logs reads/writes to indirect registers (except cursor movement)
#define LOG_CURSOR   (1U << 3) // logs cursor movement

#define VERBOSE (0)

#include "logmacro.h"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(BT481, bt481_device, "bt481", "Brooktree Bt481 RAMDAC")
DEFINE_DEVICE_TYPE(BT482, bt482_device, "bt482", "Brooktree Bt482 RAMDAC")

bt481_device::bt481_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_palette_interface(mconfig, *this)
{
}

bt481_device::bt481_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	bt481_device(mconfig, BT481, tag, owner, clock)
{
}

bt482_device::bt482_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	bt481_device(mconfig, BT482, tag, owner, clock)
{
}


//**************************************************************************
//  MACHINE EMULATION
//**************************************************************************

void bt481_device::device_start()
{
	m_sleep_recovery_timer = timer_alloc(FUNC(bt481_device::sleep_recovery_done), this);
	// initialize power-up values
	m_cra = 0x00;
	m_crb = 0x1e;
	m_cursor = 0x00;
	m_addr = m_addr_rgb = m_indirect_index = m_hidden_reads = 0;
	m_read_mask = 0xff;
	m_overlay_mask = 0x0f;
	m_overlay_mask_pending = 0x0f;
	std::fill(std::begin(m_color), std::end(m_color), 0);

	// register for save states
	save_item(NAME(m_cra));
	save_item(NAME(m_crb));
	save_item(NAME(m_color));
	save_item(NAME(m_addr));
	save_item(NAME(m_addr_rgb));
	save_item(NAME(m_indirect_index));
	save_item(NAME(m_read_mask));
	save_item(NAME(m_overlay_mask));
	save_item(NAME(m_overlay_mask_pending));
	save_item(NAME(m_cursor));
	save_item(NAME(m_hidden_reads));
	save_item(NAME(m_sleep_recovering));
}

void bt481_device::device_reset()
{
	// L481A_C pp19-20. The palette and mask power-up values are unspecified;
	// do not discard palette RAM on a machine reset.
	m_cra = 0;
	m_crb = 0x1e;
	m_hidden_reads = 0;
	m_sleep_recovering = false;
	m_sleep_recovery_timer->adjust(attotime::never);
}

void bt481_device::command_b_w(uint8_t data)
{
	bool const was_sleeping = m_6_8_pin && BIT(m_crb, 0);
	m_crb = data;
	if (m_6_8_pin && BIT(m_crb, 0))
	{
		// Going back to sleep cancels an incomplete recovery.
		m_sleep_recovering = false;
		m_sleep_recovery_timer->adjust(attotime::never);
	}
	else if (was_sleeping)
	{
		// L481A_C p20 specifies approximately one second, dependent on
		// the COMP capacitor. Exactly one second is the selected digital
		// approximation; it does not model the analog settling waveform.
		m_sleep_recovering = true;
		m_sleep_recovery_timer->adjust(attotime::from_seconds(1));
		LOG("Bt481 color chip (RAMDAC): starting 1 s wake-up recovery\n");
	}
	// Writes which leave B0 clear do not restart a recovery in progress.
}

TIMER_CALLBACK_MEMBER(bt481_device::sleep_recovery_done)
{
	m_sleep_recovering = false;
}

uint8_t bt481_device::vga_r(offs_t rs)
{
	rs &= 3;
	if (!machine().side_effects_disabled())
	{
		if (rs == 2 && !BIT(m_cra, 2)) m_hidden_reads = std::min<unsigned>(4, m_hidden_reads + 1);
		else if (m_hidden_reads < 4) m_hidden_reads = 0;
	}
	if (BIT(m_cra, 2)) rs |= 4;
	switch (rs)
	{
	case 0: case 3: return address_r();
	case 1: return palette_r();
	case 2: return mask_r();
	case 4: case 7: return overlay_address_r();
	case 5: return overlay_r();
	case 6: return m_cra;
	}
	return 0xff;
}

void bt481_device::vga_w(offs_t rs, uint8_t data)
{
	rs &= 3;
	bool const hidden = rs == 2 && m_hidden_reads == 4;
	m_hidden_reads = 0;
	if (hidden || BIT(m_cra, 2)) rs |= 4;
	switch (rs)
	{
	case 0: address_w(data); break;
	case 1: palette_w(data); break;
	case 2: mask_w(data); break;
	case 3: address_read_w(data); break;
	case 4: overlay_address_w(data & 15); break;
	case 5: overlay_w(data); break;
	case 6: command_w(data); break;
	case 7: overlay_address_read_w(data & 15); break;
	}
}

unsigned bt481_device::bytes_per_pixel() const
{
	return !BIT(m_cra, 7) ? 1 : !BIT(m_cra, 4) ? 2 : BIT(m_cra, 5) ? 3 : 4;
}

unsigned bt481_device::pixels_per_clock_divisor() const
{
	return !BIT(m_cra, 7) ? 1 : bytes_per_pixel() / (BIT(m_cra, 5) ? 1 : 2);
}

// Pseudo color and the two-edge 15- and 16-bit modes take 7 clocks, the
// one-edge 15- and 16-bit and two-edge 32-bit modes 8, one-edge 24-bit 9.
unsigned bt481_device::pipeline_clocks() const
{
	if (!BIT(m_cra, 7))
		return 7;
	return BIT(m_cra, 5) ? 6 + bytes_per_pixel() : bytes_per_pixel() == 4 ? 8 : 7;
}

rgb_t bt481_device::pixel_select(uint8_t index, uint8_t overlay) const
{
	if (asleep()) return rgb_t::black();
	if (overlay & m_overlay_mask) return overlay_select(overlay);
	rgb_t const color = pen_color(index & m_read_mask);
	return eight_bit() ? color : rgb_t(color.r() & 0xfc, color.g() & 0xfc, color.b() & 0xfc);
}

rgb_t bt481_device::overlay_select(uint8_t index) const
{
	if (asleep()) return rgb_t::black();
	rgb_t const color = pen_color(256 + (index & m_overlay_mask));
	return eight_bit() ? color : rgb_t(color.r() & 0xfc, color.g() & 0xfc, color.b() & 0xfc);
}

rgb_t bt481_device::direct_color(uint32_t pixel, uint8_t overlay) const
{
	if (!BIT(m_cra, 7)) return pixel_select(pixel, overlay);
	if (asleep()) return rgb_t::black();
	// L481A_C p20 B6: external OL inputs can override true color. This is
	// separate from the fourth P-bus byte in 32-bit palette-overlay mode.
	if (m_6_8_pin && BIT(m_crb, 6) && (overlay & m_overlay_mask)) return overlay_select(overlay);
	// L481A_C pp7-8: unused low DAC bits are zero, not replicated.
	if (!BIT(m_cra, 4))
		return BIT(m_cra, 6)
			? rgb_t((pixel >> 8) & 0xf8, (pixel >> 3) & 0xfc, (pixel << 3) & 0xf8)
			: rgb_t((pixel >> 7) & 0xf8, (pixel >> 2) & 0xf8, (pixel << 3) & 0xf8);
	if (!BIT(m_cra, 5) && ((pixel >> 24) & m_read_mask)) return pixel_select(pixel >> 24);
	return BIT(m_cra, 1) ? rgb_t(pixel >> 16, pixel >> 8, pixel) : rgb_t(pixel, pixel >> 8, pixel >> 16);
}

void bt482_device::device_start()
{
	bt481_device::device_start();

	// register for save states
	save_item(NAME(m_cram));
	save_item(NAME(m_cxlr));
	save_item(NAME(m_cxhr));
	save_item(NAME(m_cylr));
	save_item(NAME(m_cyhr));
	save_item(NAME(m_cx));
	save_item(NAME(m_cy));
}

void bt481_device::map(address_map &map)
{
	map(0x00, 0x00).rw(FUNC(bt481_device::address_r), FUNC(bt481_device::address_w));
	map(0x01, 0x01).rw(FUNC(bt481_device::palette_r), FUNC(bt481_device::palette_w));
	map(0x02, 0x02).rw(FUNC(bt481_device::mask_r), FUNC(bt481_device::mask_w));
	map(0x03, 0x03).rw(FUNC(bt481_device::address_r), FUNC(bt481_device::address_read_w));
	map(0x04, 0x04).rw(FUNC(bt481_device::overlay_address_r), FUNC(bt481_device::overlay_address_w));
	map(0x05, 0x05).rw(FUNC(bt481_device::overlay_r), FUNC(bt481_device::overlay_w));
	map(0x06, 0x06).lr8(NAME([this] { return m_cra; })).w(FUNC(bt481_device::command_w));
	map(0x07, 0x07).rw(FUNC(bt481_device::overlay_address_r), FUNC(bt481_device::overlay_address_read_w));
}

uint8_t bt481_device::address_r()
{
	LOGMASKED(LOG_ACCESS, "address_r: %02x\n", m_addr);

	return m_addr;
}

void bt481_device::address_w(uint8_t data)
{
	LOGMASKED(LOG_ACCESS, "address_w: %02x\n", data);

	if (BIT(m_cra, 0))
		m_indirect_index = data & 0x07;

	m_addr = data;
	m_addr_rgb = 0;
}

uint8_t bt481_device::palette_r()
{
	LOGMASKED(LOG_ACCESS, "palette_r (index %02x)\n", m_addr);

	uint8_t const result = eight_bit() ? m_color[m_addr_rgb] : (m_color[m_addr_rgb] >> 2);
	if (!machine().side_effects_disabled())
	{
		m_addr_rgb = (m_addr_rgb + 1) % 3;
		if (!m_addr_rgb) palette_read_load(0);
	}
	return result;
}

void bt481_device::palette_w(uint8_t data)
{
	LOGMASKED(LOG_ACCESS, "palette_w: %02x (index %02x)\n", data, m_addr);

	m_color[m_addr_rgb] = eight_bit() ? data : (data << 2);

	// we have all components, update palette and increment index
	if (m_addr_rgb == 2)
		set_pen_color(m_addr++, rgb_t(m_color[0], m_color[1], m_color[2]));

	m_addr_rgb = (m_addr_rgb + 1) % 3;
}

uint8_t bt481_device::mask_r()
{
	uint8_t data = 0xff;

	if (BIT(m_cra, 0))
	{
		switch (m_indirect_index)
		{
			case READ_MASK_REGISTER:
				LOGMASKED(LOG_INDIRECT, "R Read Mask Register = %02x\n", m_read_mask);
				data = m_read_mask;
				break;

			case OVERLAY_MASK_REGISTER:
				LOGMASKED(LOG_INDIRECT, "R Overlay Mask Register = %02x\n", m_overlay_mask_pending);
				data = m_overlay_mask_pending;
				break;

			case COMMAND_REGISTER_B:
				LOGMASKED(LOG_INDIRECT, "R Command Register B = %02x\n", m_crb);
				data = m_crb;
				break;

			case CURSOR_REGISTER:
				LOGMASKED(LOG_INDIRECT, "R Cursor Register = %02x\n", m_cursor);
				data = m_cursor;
				break;

			default:
				LOGMASKED(LOG_INDIRECT, "Read from unsupported indirect register %02x\n", m_indirect_index);
				data = 0xff;
				break;
		}
	}
	else
	{
		data = m_read_mask;
	}

	LOGMASKED(LOG_ACCESS, "mask_r: %02x\n", data);

	return data;
}

uint8_t bt482_device::mask_r()
{
	uint8_t data = 0xff;

	if (BIT(m_cra, 0))
	{
		switch (m_indirect_index)
		{
			case READ_MASK_REGISTER:
				LOGMASKED(LOG_INDIRECT, "R Read Mask Register = %02x\n", m_read_mask);
				data = m_read_mask;
				break;

			case OVERLAY_MASK_REGISTER:
				LOGMASKED(LOG_INDIRECT, "R Overlay Mask Register = %02x\n", m_overlay_mask);
				data = m_overlay_mask;
				break;

			case COMMAND_REGISTER_B:
				LOGMASKED(LOG_INDIRECT, "R Command Register B = %02x\n", m_crb);
				data = m_crb;
				break;

			case CURSOR_REGISTER:
				LOGMASKED(LOG_INDIRECT, "R Cursor Register = %02x\n", m_cursor);
				data = m_cursor;
				break;

			case CURSOR_X_LOW_REGISTER:
				LOGMASKED(LOG_CURSOR, "R Cursor X Low = %02x\n", m_cxlr);
				data = m_cxlr;
				break;

			case CURSOR_X_HIGH_REGISTER:
				LOGMASKED(LOG_CURSOR, "R Cursor X High = %02x\n", m_cxhr);
				data = m_cxhr;
				break;

			case CURSOR_Y_LOW_REGISTER:
				LOGMASKED(LOG_CURSOR, "R Cursor Y Low = %02x\n", m_cylr);
				data = m_cylr;
				break;

			case CURSOR_Y_HIGH_REGISTER:
				LOGMASKED(LOG_CURSOR, "R Cursor Y High = %02x\n", m_cyhr);
				data = m_cyhr;
				break;
		}
	}
	else
	{
		data = m_read_mask;
	}

	LOGMASKED(LOG_ACCESS, "mask_r: %02x\n", data);

	return data;
}

void bt481_device::mask_w(uint8_t data)
{
	LOGMASKED(LOG_ACCESS, "mask_w: %02x\n", data);

	if (BIT(m_cra, 0))
	{
		switch (m_indirect_index)
		{
			case READ_MASK_REGISTER:
				LOGMASKED(LOG_INDIRECT, "W Read Mask Register = %02x\n", data);
				m_read_mask = data;
				m_overlay_mask = m_overlay_mask_pending;
				break;

			case OVERLAY_MASK_REGISTER:
				LOGMASKED(LOG_INDIRECT, "W Overlay Mask Register = %02x\n", data);
				// L481A_C p14: latch together with the next pixel-mask write.
				m_overlay_mask_pending = data & 0x0f;
				break;

			case COMMAND_REGISTER_B:
				LOGMASKED(LOG_INDIRECT, "W Command Register B = %02x\n", data);
				command_b_w(data);
				break;

			case CURSOR_REGISTER:
				LOGMASKED(LOG_INDIRECT, "W Cursor Register = %02x\n", data);
				m_cursor = data;
				break;

			default:
				LOGMASKED(LOG_INDIRECT, "Write to unsupported indirect register %02x\n", m_indirect_index);
				break;
		}
	}
	else
	{
		m_read_mask = data;
		m_overlay_mask = m_overlay_mask_pending;
	}
}

void bt482_device::mask_w(uint8_t data)
{
	LOGMASKED(LOG_ACCESS, "mask_w: %02x\n", data);

	if (BIT(m_cra, 0))
	{
		switch (m_indirect_index)
		{
			case READ_MASK_REGISTER:
				LOGMASKED(LOG_INDIRECT, "W Read Mask Register = %02x\n", data);
				m_read_mask = data;
				break;

			case OVERLAY_MASK_REGISTER:
				LOGMASKED(LOG_INDIRECT, "W Overlay Mask Register = %02x\n", data);
				m_overlay_mask = data & 0x0f;
				break;

			case COMMAND_REGISTER_B:
				LOGMASKED(LOG_INDIRECT, "W Command Register B = %02x\n", data);
				m_crb = data;
				break;

			case CURSOR_REGISTER:
				LOGMASKED(LOG_INDIRECT, "W Cursor Register = %02x\n", data);
				m_cursor = data;
				break;

			case CURSOR_X_LOW_REGISTER:
				LOGMASKED(LOG_CURSOR, "W Cursor X Low = %02x\n", data);
				m_cxlr = data;
				break;

			case CURSOR_X_HIGH_REGISTER:
				LOGMASKED(LOG_CURSOR, "W Cursor X High = %02x\n", data);
				m_cxhr = data & 0x0f;
				m_cx = (m_cxhr << 8) | m_cxlr;
				break;

			case CURSOR_Y_LOW_REGISTER:
				LOGMASKED(LOG_CURSOR, "W Cursor Y Low = %02x\n", data);
				m_cylr = data;
				break;

			case CURSOR_Y_HIGH_REGISTER:
				LOGMASKED(LOG_CURSOR, "W Cursor Y High = %02x\n", data);
				m_cyhr = data & 0x0f;
				m_cy = (m_cyhr << 8) | m_cylr;
				break;
		}
	}
	else
	{
		m_read_mask = data;
	}
}

void bt481_device::address_read_w(uint8_t data)
{
	LOGMASKED(LOG_ACCESS, "address_read_w: %02x\n", data);

	m_addr = data;
	m_addr_rgb = 0;
	palette_read_load(0);
}

void bt481_device::palette_read_load(unsigned base)
{
	// L481A_C p6: read addressing prefetches RGB and advances immediately.
	if (base && m_addr >= 20)
	{
		// An out-of-range prefetch must not overrun the palette, including
		// the prefetch following the last Bt482 cursor color.
		std::fill(std::begin(m_color), std::end(m_color), 0);
		return;
	}
	rgb_t const color = pen_color(base + m_addr);
	m_color[0] = color.r();
	m_color[1] = color.g();
	m_color[2] = color.b();
	m_addr = (base && m_addr < 16) ? ((m_addr + 1) & 15) : uint8_t(m_addr + 1);
}

uint8_t bt481_device::overlay_address_r()
{
	LOGMASKED(LOG_ACCESS, "overlay_address_r: %02x\n", m_addr);

	return m_addr;
}

void bt481_device::overlay_address_w(uint8_t data)
{
	LOGMASKED(LOG_ACCESS, "overlay_address_w: %02x\n", data);

	m_addr = data;
	m_addr_rgb = 0;
}

uint8_t bt481_device::overlay_r()
{
	LOGMASKED(LOG_ACCESS, "overlay_r (index %02x)\n", m_addr);

	uint8_t const result = eight_bit() ? m_color[m_addr_rgb] : (m_color[m_addr_rgb] >> 2);
	if (!machine().side_effects_disabled())
	{
		m_addr_rgb = (m_addr_rgb + 1) % 3;
		if (!m_addr_rgb) palette_read_load(256);
	}
	return result;
}

uint8_t bt482_device::overlay_r()
{
	if (BIT(m_cursor, 3))
	{
		LOGMASKED(LOG_ACCESS, "overlay_r (index %02x)\n", m_addr);
		return m_cram[m_addr];
	}
	else
	{
		return bt481_device::overlay_r();
	}
}

void bt481_device::overlay_w(uint8_t data)
{
	LOGMASKED(LOG_ACCESS, "overlay_w: %02x (index %02x)\n", data, m_addr);

	if (m_addr >= 20)
	{
		LOG("Overlay index out of range: %02x\n", m_addr);
		return;
	}

	m_color[m_addr_rgb] = eight_bit() ? data : (data << 2);

	if (m_addr_rgb == 2)
	{
		// we have all components, update palette and increment index
		set_pen_color(256 + m_addr, rgb_t(m_color[0], m_color[1], m_color[2]));

		if (m_addr < 16)
			// while accessing the overlay color registers the 4 high bits are ignored
			m_addr = (m_addr + 1) & 0x0f;
		else
			// accessing cursor color
			m_addr++;
	}

	m_addr_rgb = (m_addr_rgb + 1) % 3;
}

void bt482_device::overlay_w(uint8_t data)
{
	if (BIT(m_cursor, 3))
	{
		LOGMASKED(LOG_ACCESS, "overlay_w: %02x (index %02x)\n", data, m_addr);
		m_cram[m_addr++] = data;
	}
	else
	{
		bt481_device::overlay_w(data);
	}
}

void bt481_device::command_w(uint8_t data)
{
	LOGMASKED(LOG_ACCESS, "command_w: %02x\n", data);

	m_cra = data;

	if (BIT(m_cra, 7))
	{
		switch ((m_cra >> 4) & 0x07)
		{
			case 0: LOG("Color mode: 5:5:5 dual-edge (33K colors)\n"); break;
			case 1: LOG("Color mode: 8:8:8:OL dual-edge (16.8M colors)\n"); break;
			case 2: LOG("Color mode: 5:5:5 single-edge (33K colors)\n"); break;
			case 4: LOG("Color mode: 5:6:5 dual-edge (65K colors)\n"); break;
			case 6: LOG("Color mode: 5:6:5 single-edge (65K colors)\n"); break;
			case 7: LOG("Color mode: 8:8:8 single-edge (16.8M colors)\n"); break;

			default: LOG("Invalid color mode selected!\n");
		}
	}
	else
	{
		LOG("Color mode: Pseudo color (256 colors)\n");
	}

	LOG("Extended register set %sabled\n", BIT(m_cra, 0) ? "en": "dis");
}

void bt481_device::overlay_address_read_w(uint8_t data)
{
	LOGMASKED(LOG_ACCESS, "overlay_address_read_w: %02x\n", data);

	m_addr = data;
	m_addr_rgb = 0;
	// Bt482 cursor RAM uses a byte address, without RGB palette prefetch.
	if (!BIT(m_cursor, 3)) palette_read_load(256);
}


//**************************************************************************
//  CURSOR RENDERING
//**************************************************************************

uint32_t bt482_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	// only the 3 color cursor is supported
	if ((m_cursor & 0x03) == CURSOR_3_COLOR)
	{
		// cursor area in screen coordinates
		rectangle cursor(m_cx - 32, m_cx - 1, m_cy - 32, m_cy - 1);

		// intersect with cliprect
		cursor &= bitmap.cliprect();

		if (!cursor.empty())
		{
			for (int y = 0; y < 32; y++)
			{
				const int ypos = m_cy - 32 + y;

				for (int x = 0; x < 32; x++)
				{
					const int xpos = m_cx - 32 + x;

					if (cursor.contains(xpos, ypos))
					{
						// fetch color data from plane 0 and 1
						uint8_t p0 = BIT(m_cram[0x00 + (y * 4) + (x / 8)], 7 - (x % 8));
						uint8_t p1 = BIT(m_cram[0x80 + (y * 4) + (x / 8)], 7 - (x % 8));

						uint8_t color = (p1 << 1) | p0;

						// if we have a color draw it
						if (color)
							bitmap.pix(ypos, xpos) = pen_color(256 + 16 + color);
					}
				}
			}
		}
	}

	return 0;
}

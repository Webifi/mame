// license:BSD-3-Clause
/*
 * Oak OTI-087X VGA chip used by Sequential Systems Second Sight.
 *
 * Sources: OTI-087 databook (printed page numbers below), OTI-087X addendum,
 * OTI-068 frequency table p7, Brooktree L481A_C, ROM_BEEF mode tables,
 * and the Oak Windows driver STD8_HC.DRV.
 *
 * Exact DRAM/FIFO/cache arbitration and IOCHRDY release timing are unknown.
 * The parameterized DRAM timeline in oti087_memory.h supplies IOCHRDY,
 * separate from U15 Z180 memory decoder's overlapping fixed waits.
 * No register I/O waits. Oak's Windows driver establishes the cursor layout,
 * visibility and AND/XOR pixel codes. Addressed latch-byte selection uses
 * CPU A2:A0 when ext22 bit4=0 and bit0=1.
 * The ext03 test bits follow an assumed convention where the data sheet is silent.
 */

#include "emu.h"
#include "pc_vga_oti087.h"

#define LOG_DIAGNOSTIC (1U << 1)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(OTI087, oak_oti087_vga_device, "oti087_vga", "Oak OTI-087X VGA chip")

namespace
{
constexpr u32 clocks[16] = {25'175'000, 28'322'000, 65'000'000, 44'900'000, 28'322'000, 36'000'000, 40'000'000, 36'000'000, 25'175'000,
	28'322'000, 78'000'000, 65'000'000, 63'000'000, 72'000'000, 40'000'000, 50'000'000};
enum class cursor_op : u8
{
	color0,
	color1,
	transparent,
	invert
};
rectangle output_span(rectangle area, int x, int y)
{
	area.offset(x, y);
	return area;
}

// The monitors the card drives are CRTs. Each places the picture by the
// signal's timing. The owner sets the position and size controls for the best
// picture: the whole unblanked signal, the active picture and its border,
// shows and is centered. Where the monitor's own window is larger, the rest
// of the window is blanked signal.
//
// VGA, a multisync monitor: it keeps its raster a fixed part of the line and
// of the frame. At 31.5 kHz the sync polarities select the IBM vertical size
// for 350, 400 or 480 lines (IBM PS/2 Video Subsystem, Oct 1990, Fig. 103);
// the windows are 656 of 800 dots (Fig. 108) and 362 of 449, 414 of 449 and
// 496 of 525 lines (Figs. 104-106). It locks from the IBM 31.469 kHz and
// 50 Hz (Fig. 6) to the fastest VESA DMT 1.10 timings the card's 78 MHz clock
// makes: 1024 x 768 at 75 Hz (60.023 kHz) and 1024 x 768 interlaced
// (86.96 Hz fields). The card's own VGA record $61 runs 1.0% below the IBM
// rates, so each limit has that tolerance.
constexpr double vga_line = 25'175'000.0 / 800, vga_tolerance = 0.01;
constexpr double vga_window_part = 656.0 / 800.0;
constexpr double vga_line_min = vga_line * (1 - vga_tolerance), vga_line_max = 78'750'000.0 / 1312 * (1 + vga_tolerance);
constexpr double vga_field_min = 50.0 * (1 - vga_tolerance), vga_field_max = 44'900'000.0 / 1264 / 817 * 2 * (1 + vga_tolerance);
// AppleColor RGB Monitor (Apple TIL01930): its sweep is set at the factory so
// that the IIgs 640 x 200 picture fills 200 x 150 mm of the 11.5-inch
// viewable face. The IIgs line is 65 counts of 14 cycles of the 14.318 MHz
// clock plus 2 for the long count, and 40 counts show (Apple IIe Technical
// Reference, video display), so the face spans that time x 292.1 / 250.
constexpr double applecolor_face = 11.5 * 25.4 / 250.0;
constexpr double applecolor_window_time = 40 * 14 / 14'318'180.0 * applecolor_face;
constexpr double applecolor_window_frame = 200 * (65 * 14 + 2) / 14'318'180.0 * applecolor_face;
// Apple specifies 15.734 kHz and 50 or 60 Hz. The AppleColor RGB records
// Sequential shipped for the card run from 13.31 kHz (M640X200.18M) to
// 18.75 kHz (M80X25.1575M) and up to 74.5 Hz (M560X192); the monitor shows
// nothing at 31 kHz.
constexpr double applecolor_line_min = 28'322'000.0 / 2 / 1064, applecolor_line_max = 72'000'000.0 / 4 / 960;
constexpr double applecolor_field_min = 50.0, applecolor_field_max = 28'322'000.0 / 2 / 856 / 222;

struct monitor_window
{
	unsigned width, height;
	bool locked;
};

// The signal's line, field and pixel rates, its line in pixels, its frame in
// lines (both fields when interlaced), its unblanked pixels and lines, fields
// per frame and sync polarities.
monitor_window card_monitor(bool vga, double line_hz, double field_hz, double pixel_hz, unsigned ht, unsigned vt,
	unsigned uw, unsigned uh, unsigned fields, bool h_negative, bool v_negative)
{
	monitor_window window;
	if (vga)
	{
		double part = 496.0 / 525.0;
		if (std::abs(line_hz / vga_line - 1.0) <= vga_tolerance && h_negative != v_negative)
			part = (v_negative ? 362.0 : 414.0) / 449.0;
		window = { unsigned(ht * vga_window_part + 0.5), unsigned(vt * part + 0.5),
			line_hz >= vga_line_min && line_hz <= vga_line_max && field_hz >= vga_field_min && field_hz <= vga_field_max };
	}
	else
		window = { unsigned(applecolor_window_time * pixel_hz + 0.5), unsigned(applecolor_window_frame * line_hz * fields + 0.5),
			line_hz >= applecolor_line_min && line_hz <= applecolor_line_max &&
			field_hz >= applecolor_field_min && field_hz <= applecolor_field_max };
	window.width = std::max(window.width, uw);
	window.height = std::max(window.height, uh);
	return window;
}

// Floor of half a signed difference, so both directions round the same way.
int floor_half(int value)
{
	return value >= 0 ? value / 2 : -((1 - value) / 2);
}

// The unblanked part of one axis, in pixels or lines from the start of the
// active display: blanking starts at blank_first and lasts blank_length of
// total. start is negative when the unblanked part begins before the active
// display, in the border of the preceding line or frame.
struct unblanked_span
{
	int start, length;
};

unblanked_span unblanked(int total, int active, int blank_first, int blank_length)
{
	int const length = total - std::clamp(blank_length, 0, total);
	int const end = (blank_first + blank_length) % total;
	return { end < active ? end : end - total, length };
}

// Offset of the active display in a window that is centered on the unblanked
// part. With no unblanked part, center the window on the active display.
int active_offset(int window, int active, unblanked_span span)
{
	if (span.length <= 0)
		return floor_half(window - active);
	return floor_half(window - span.length) - span.start;
}

// Oak STD8_HC.DRV SetCursor (ordinal 102, segment 1:4BE3):
// plane A is the Windows AND mask, plane B is XOR. This primary driver
// evidence supersedes the conflicting secondary OAK.TXT code table.
constexpr cursor_op cursor_codes[4] = {
	cursor_op::color0,		// A=0 B=0
	cursor_op::color1,		// A=0 B=1
	cursor_op::transparent, // A=1 B=0
	cursor_op::invert		// A=1 B=1
};
}

oak_oti087_vga_device::oak_oti087_vga_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: svga_device(mconfig, OTI087, tag, owner, clock)
	, m_screen(*this, "screen")
	, m_ramdac(*this, "ramdac")
	, m_eeprom(*this, "eeprom")
	, m_monitor(*this, "MONITOR")
	, m_iochrdy_cb(*this)
	, m_scan_test_cb(*this)
	, m_eeprom_di_cb(*this)
	, m_eeprom_cs_cb(*this)
	, m_eeprom_clock_cb(*this)
{
	set_vram_size(0x100000);
	set_screen(m_screen);
	m_atc_space_config = address_space_config(
		"attribute_regs", ENDIANNESS_LITTLE, 8, 8, 0, address_map_constructor(FUNC(oak_oti087_vga_device::attribute_map), this));
}

void oak_oti087_vga_device::device_add_mconfig(machine_config &config)
{
	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	// The OTI-087X has no sync input: its raster is independent of the IIgs.
	m_screen->set_video_attributes(VIDEO_INDEPENDENT_FRAME);
	m_screen->set_physical_aspect(4, 3);
	m_screen->set_raw(25'175'000, 800, 0, 640, 525, 0, 480);
	m_screen->set_screen_update(FUNC(oak_oti087_vga_device::screen_update));
	BT481(config, m_ramdac, 0);
	// Prototype netlist: U31 Bt481 color chip (RAMDAC) pin 2 is tied to VAA.
	// Together with CR B1=1 at
	// power-up this selects the firmware's eight-bit RGB palette interface.
	m_ramdac->set_6_8_pin(true);
	// SYNC* is tied high; CR B's power-up sync-on-RGB bits add no sync signal.
	// Board wiring: OL0..3 are grounded. The renderers supply overlay=0.
	EEPROM_93C46_16BIT(config, m_eeprom).default_value(0xffff);
	m_eeprom->do_callback().set(FUNC(oak_oti087_vga_device::eeprom_do_w));
}

static INPUT_PORTS_START(oti087)
	PORT_START("VGA_SENSE")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_START("MONITOR")
	PORT_CONFNAME(1, 1, "Second Sight monitor")
	PORT_CONFSETTING(1, "VGA")
	PORT_CONFSETTING(0, "AppleColor RGB")
INPUT_PORTS_END

ioport_constructor oak_oti087_vga_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(oti087);
}

void oak_oti087_vga_device::device_start()
{
	svga_device::device_start();
	m_ready_timer = timer_alloc(FUNC(oak_oti087_vga_device::ready_tick), this);
	m_ready_listener = !m_iochrdy_cb.isunset();
	m_eeprom_ready_timer = timer_alloc(FUNC(oak_oti087_vga_device::eeprom_ready_tick), this);
	// Keep the VGA core's display-start latch semantics, with an Oak timer
	// that accounts for vertical overflow and the two interlaced fields.
	m_vblank_timer = timer_alloc(FUNC(oak_oti087_vga_device::display_latch_tick), this);
	m_display_end_timer = timer_alloc(FUNC(oak_oti087_vga_device::display_end_tick), this);
	m_character_map_timer = timer_alloc(FUNC(oak_oti087_vga_device::character_map_tick), this);
	m_pan_timer = timer_alloc(FUNC(oak_oti087_vga_device::pan_tick), this);
	save_item(NAME(m_character_map));
	save_item(NAME(m_row_history));
	save_item(NAME(m_row_history_frame));
	save_item(NAME(m_frame_preset));
	save_item(NAME(m_ext));
	save_item(NAME(m_active_display));
	save_item(NAME(m_display_origin));
	save_item(NAME(m_crtc_origin));
	save_item(NAME(m_origin_phase));
	save_item(NAME(m_out_of_range));
	save_item(NAME(m_lock_candidate));
	save_item(NAME(m_lock_since));
	save_item(NAME(m_lock_rates));
	save_item(NAME(m_system_data));
	save_item(NAME(m_latch));
	save_item(NAME(m_latch_diagnostics));
	save_item(NAME(m_index));
	save_item(NAME(m_setup));
	save_item(NAME(m_enable));
	save_item(NAME(m_ready));
	save_item(NAME(m_ready_until));
	save_item(NAME(m_output_divider));
	save_item(NAME(m_linear_address));
	save_item(NAME(m_monitor_override));
	save_item(NAME(m_memory_active));
	save_item(NAME(m_memory_epoch));
	save_item(NAME(m_memory_next_line));
	save_item(NAME(m_memory_raster));
	save_item(NAME(m_memory.busy));
	save_item(NAME(m_memory.line));
	save_item(NAME(m_memory.active_as));
	save_item(NAME(m_memory.word_as));
	save_item(NAME(m_memory.words));
	save_item(NAME(m_memory.served));
	save_item(NAME(m_memory.display_open));
	save_item(NAME(m_memory.refresh_pending));
	save_item(NAME(m_memory.refresh_open));
	save_item(NAME(m_memory.read_tag));
	save_item(NAME(m_memory.read_end));
	save_item(NAME(m_memory.write_tag));
	save_item(NAME(m_memory.write_mask));
	save_item(NAME(m_memory.write_start));
	save_item(NAME(m_memory.write_end));
	save_item(NAME(m_packed_layout_diagnostics));
	save_item(NAME(vga.crtc.horz_blank_end));
	machine().save().register_postload(save_prepost_delegate(FUNC(oak_oti087_vga_device::postload), this));
}

void oak_oti087_vga_device::device_reset()
{
	svga_device::device_reset();
	// Clear decoded VGA fields too: the base reset only clears some of them.
	zero();
	vga.crtc.maximum_scan_line = 1;
	vga.crtc.line_compare = 0x3ff;
	std::fill(std::begin(m_ext), std::end(m_ext), 0);
	m_scan_test_cb(0);
	m_screen->set_output_interlace(false);
	m_packed_layout_diagnostics = 0;
	m_row_history_frame = ~u64(0);
	m_frame_preset = 0;
	std::fill(std::begin(m_latch), std::end(m_latch), 0);
	m_latch_diagnostics = 0;
	// Explicit defaults, OTI pp33-46. Other zeroes are deterministic emulator
	// initialization, not claims about undocumented power-up state.
	m_ext[4] = 0x0f;
	m_ext[7] = m_config1;
	m_ext[8] = m_config2 & 0x7f; // 087X addendum: bit7 reserved; no feature connector
	m_ext[0x10] = m_dip;
	m_ext[0x19] = 4;
	m_ext[0x20] = 1; // p41 value; p38 disagrees for the mirrored bits
	m_ext[0x4c] = 0x10;
	m_index = m_setup = m_enable = 0;
	m_ioas = false;
	// These are external board inputs, not Oak resettable registers. The
	// The Z180 I/O and memory decoder GALs have no ARES input in schematic
	// sheet 2; retention is inferred.
	m_ready = true;
	m_ready_until = attotime::zero;
	m_memory_active = false;
	m_memory = {};
	// Convention: independent crystals start seven ns apart; never rephase
	// MCLK to a CPU access, mode change, or scanline (05-z180, IOCHRDY).
	m_memory_epoch = machine().time() + attotime::from_nsec(oti087_memory_timing::reset_phase_ns);
	m_memory_next_line = attotime::zero;
	m_memory_raster.fill(0);
	m_ready_timer->adjust(attotime::never);
	m_vblank_timer->adjust(attotime::never);
	m_display_end_timer->adjust(attotime::never);
	m_character_map_timer->adjust(attotime::never);
	m_pan_timer->adjust(attotime::never);
	m_character_map[0] = m_character_map[1] = 0;
	m_iochrdy_cb(1);
	eeprom_update();
}

void oak_oti087_vga_device::postload()
{
	m_scan_test_cb(scan_test());
	m_iochrdy_cb(iochrdy_r());
	m_vsync_cb(vga.crtc.irq_latch && !vga.crtc.irq_disable);
	eeprom_update();
	recompute_params();
}

bool oak_oti087_vga_device::enabled() const
{
	return BIT(m_enable, 0) && BIT(m_setup, 3) && !BIT(m_setup, 4);
}

void oak_oti087_vga_device::memory_line(attotime now)
{
	if (now < m_memory_next_line) return;
	using timing = oti087_memory_timing;
	// The background has no timers: reconstruct the current line when a
	// CPU request first needs it. Refresh position and FIFO prefetch at
	// HBLANK start are conventions (05-z180, IOCHRDY), not recovered logic.
	int const width = m_screen->width(), height = m_screen->height();
	unsigned const dots = BIT(vga.sequencer.data[1], 0) ? 8 : 9;
	unsigned const hblank = (vga.crtc.horz_blank_start * dots * horizontal_multiplier() / pixel_divisor()) % width;
	unsigned y = (crtc_vpos() + (crtc_hpos() >= hblank)) % height;
	m_memory_next_line = machine().time() + time_until_crtc_pos(y, hblank);
	if (m_memory_next_line <= now)
	{
		m_memory_next_line += m_screen->scan_period();
		y = (y + 1) % height;
	}
	attotime const line_start = m_memory_next_line - m_screen->scan_period();
	attotime const active = line_start + m_screen->scan_period() - m_screen->pixel_period() * hblank;
	s64 const edge = timing::edge(line_start - m_memory_epoch) + 1;
	s64 const active_as = (active - m_memory_epoch - timing::time(edge)).as_attoseconds();
	unsigned const field_y = y % (height / (get_interlace_mode() ? 2 : 1));
	bool const display = (vga.sequencer.data[0] & 3) == 3 && !BIT(vga.sequencer.data[1], 5) &&
		field_y < (vga.crtc.vert_disp_end + 1) * vertical_divisor();
	unsigned const characters = vga.crtc.horz_disp_end + 1;
	// Packed pixels consume their actual bytes, standard graphics one word
	// per character (all four maps). Text fetches character/attribute and
	// font separately; two words per character is a timing convention.
	unsigned const bytes = packed() ? m_active_display[2] * m_ramdac->bytes_per_pixel() :
		characters * (vga.gc.alpha_dis ? 4 : 8);
	unsigned const words = (bytes + 3) / 4;
	s64 const word_as = words ? (m_screen->pixel_period() * m_active_display[2]).as_attoseconds() / words : timing::clock_as * 8;
	if (edge != m_memory.line)
		m_memory.begin_line(edge, active_as, word_as, display ? words : 0,
			timing::refreshes(vga.crtc.data[0x11], m_ext[0x21]));
	else
	{
		// Register changes within a line do not manufacture another refresh.
		m_memory.active_as = active_as;
		m_memory.word_as = std::max(timing::clock_as * timing::page_cycle, word_as);
		m_memory.words = display ? words : 0;
		m_memory.served = std::min(m_memory.served, m_memory.words ? m_memory.words + timing::fifo_words : 0);
	}
}

unsigned oak_oti087_vga_device::memory_wait(offs_t address, bool write, attotime phi)
{
	if (machine().side_effects_disabled())
		return 0;
	u32 offset;
	if (!decode_memory(address, write, offset))
		return 0; // the board still applies U15 Z180 memory decoder's floor
	m_memory_active = true;
	attotime const now = machine().time();
	using timing = oti087_memory_timing;
	// Z180 memory command at T1 falling; /WAIT is sampled at T2 falling
	// and at each Tw falling edge (UM0050 pp27-28). The IOCHRDY deadline
	// belongs to this MC, not a new pulse after its data transfer. The
	// independent U15 Z180 memory decoder floor overlaps these waits.
	attotime const command = now + phi / 2 + attotime::from_nsec(timing::command_delay_ns);
	s64 const arrival = timing::accept(command - m_memory_epoch);
	memory_line(m_memory_epoch + timing::time(std::max(arrival, m_memory.write_end)));
	// Cache tags are physical four-byte DRAM groups. Aperture/bank aliases
	// must hit the same line; planar offsets select all four maps together.
	bool const chain4 = BIT(vga.sequencer.data[4], 3);
	bool const half_store = write && packed() && chain4 && (BIT(m_ext[0x30], 3) || (m_ext[0x30] & 3) == 1);
	u32 const map_offset = chain4 ? chain4_address(offset) :
		(write ? !BIT(vga.sequencer.data[4], 2) : vga.gc.host_oe) ? odd_even_address(offset) : offset;
	u32 const tag = half_store ? ((offset & ~7) / timing::cache_bytes) + (offset & 1) :
		packed() ? offset / timing::cache_bytes : map_offset & m_plane_address_mask;
	// An eight-byte graphics-latch load/expanded store needs two words on
	// the card's 32-bit DRAM bus. Selected page sequence: one random cycle
	// followed by one page cycle. The chain4 read duplicates one word.
	unsigned const transfer_words = !chain4 && (write ? packed() &&
		(BIT(m_ext[0x30], 3) || (m_ext[0x30] & 3) == 1) : BIT(m_ext[0x22], 4)) ? 2 : 1;
	u8 const lanes = half_store || transfer_words == 2 ? 15 : !packed() ? vga.sequencer.map_mask : 0;
	s64 const release = m_memory.access(arrival, tag, offset & 3, write,
		BIT(m_ext[0x22], 2), BIT(m_ext[0x22], 3), BIT(m_ext[3], 4), m_ext[0x20] & 15, transfer_words, lanes);
	m_ready_until = m_memory_epoch + timing::time(release);
	m_ready = m_ready_until <= now;
	if (m_ready_listener)
	{
		m_iochrdy_cb(m_ready);
		m_ready_timer->adjust(m_ready ? attotime::never : m_ready_until - now);
	}
	return timing::waits(now + phi + phi / 2, phi, timing::sampled_ready(m_ready_until));
}

void oak_oti087_vga_device::access_complete()
{
	if (!machine().side_effects_disabled())
		m_memory_active = false;
	// The data edge ends the MC's access, not the start of another ready
	// pulse. Debugger/fixture reads that bypass a CPU generate no bus timing.
}

TIMER_CALLBACK_MEMBER(oak_oti087_vga_device::ready_tick)
{
	m_ready = true;
	m_iochrdy_cb(1);
}

TIMER_CALLBACK_MEMBER(oak_oti087_vga_device::display_latch_tick)
{
	flush_display();
	vga.crtc.start_addr = latch_start_addr();
	m_frame_preset = vga.crtc.preset_row_scan;
	m_character_map[0] = vga.sequencer.char_sel.B;
	m_character_map[1] = vga.sequencer.char_sel.A;
	m_character_map_timer->adjust(attotime::never);
	m_row_history_frame = ~u64(0);
	// A monitor locks to timing that holds for a whole frame. The states that
	// pass while a mode is programmed never reach the screen.
	if (m_lock_candidate == m_out_of_range && machine().time() - m_lock_since >= m_screen->frame_period())
	{
		m_out_of_range = !m_lock_candidate;
		if (m_out_of_range)
			popmessage("Second Sight %s monitor: out of range (%.2f kHz, %.2f Hz)",
				m_lock_rates[2] ? "VGA" : "AppleColor RGB", m_lock_rates[0] / 1000.0, m_lock_rates[1]);
	}
	schedule_display_latch(1);
}

TIMER_CALLBACK_MEMBER(oak_oti087_vga_device::character_map_tick)
{
	flush_display();
	m_character_map[0] = vga.sequencer.char_sel.B;
	m_character_map[1] = vga.sequencer.char_sel.A;
}

TIMER_CALLBACK_MEMBER(oak_oti087_vga_device::pan_tick)
{
	flush_display();
	vga.attribute.pel_shift = vga.attribute.pel_shift_latch;
}

void oak_oti087_vga_device::schedule_pan()
{
	// IBM 2-67 latches only the start address; 2-95 gives the pan no latch,
	// and Abrash Listing 23.1 writes it after vertical sync. The line under
	// the beam keeps its pan. Outside active display no line shows it.
	unsigned const vd = vertical_divisor();
	unsigned const field_total = (vga.crtc.vert_total + 2) * vd;
	if ((crtc_vpos() % field_total) / vd > vga.crtc.vert_disp_end)
	{
		m_pan_timer->adjust(attotime::never);
		vga.attribute.pel_shift = vga.attribute.pel_shift_latch;
		return;
	}
	m_pan_timer->adjust(time_until_crtc_pos((crtc_vpos() + 1) % m_screen->height()));
}

void oak_oti087_vga_device::schedule_character_map()
{
	// IBM 2-97: character map writes take effect on the next whole character
	// line. Only writes arm this timer; the ordinary renderer needs no test.
	unsigned const vd = vertical_divisor();
	unsigned const field_total = (vga.crtc.vert_total + 2) * vd;
	unsigned const field = crtc_vpos() / field_total;
	unsigned const line = (crtc_vpos() % field_total) / vd;
	if ((vga.sequencer.data[0] & 3) != 3 || BIT(vga.sequencer.data[1], 5) || vga.gc.alpha_dis || line > vga.crtc.vert_disp_end)
	{
		m_character_map_timer->adjust(attotime::never);
		m_character_map[0] = vga.sequencer.char_sel.B;
		m_character_map[1] = vga.sequencer.char_sel.A;
		return;
	}
	unsigned const interlace = BIT(m_ext[0x14], 7) ? 2 : 1;
	unsigned const physical = crtc_vpos() % field_total;
	row_state const row = display_row(physical * interlace + field);
	unsigned remaining = ((((vga.crtc.maximum_scan_line - 1 - row.scan) & 31) + 1) *
		(vga.crtc.scan_doubling + 1)) - (vga.crtc.scan_doubling && row.repeat ? 1 : 0);
	unsigned const split = (vga.crtc.line_compare + 1) * vd;
	if (split > physical)
		remaining = std::min(remaining, split - physical);
	unsigned const next = physical + remaining + field * field_total;
	m_character_map_timer->adjust(time_until_crtc_pos(next % m_screen->height()));
}

TIMER_CALLBACK_MEMBER(oak_oti087_vga_device::display_end_tick)
{
	// IBM 2-70: end of active display, not blanking or retrace start.
	if (!vga.crtc.irq_latch && !vga.crtc.irq_disable && vga.crtc.irq_clear)
	{
		vga.crtc.irq_latch = 1;
		m_vsync_cb(1);
	}
	schedule_display_latch(2);
}

void oak_oti087_vga_device::schedule_display_latch(unsigned timers)
{
	unsigned const vd = vertical_divisor();
	unsigned const field_total = (vga.crtc.vert_total + 2) * vd;
	unsigned const retrace = (vga.crtc.vert_retrace_start * vd) % field_total;
	unsigned const end = ((vga.crtc.vert_disp_end + 1) * vd) % field_total;
	attotime next = time_until_crtc_pos(retrace);
	attotime irq = time_until_crtc_pos(end);
	if (get_interlace_mode())
	{
		unsigned const dots = BIT(vga.sequencer.data[1], 0) ? 8 : 9;
		unsigned const phase = (m_ext[0x15] & 0x7f) * dots * horizontal_multiplier() / pixel_divisor();
		next = std::min(next, time_until_crtc_pos(retrace + field_total, phase % m_screen->width()));
		irq = std::min(irq, time_until_crtc_pos(end + field_total));
	}
	// When both edges coincide, neither callback may cancel the other.
	if (timers & 1)
		m_vblank_timer->adjust(next);
	if (timers & 2)
		m_display_end_timer->adjust(irq);
}

void oak_oti087_vga_device::flush_display()
{
	// Include partial scanlines and blanked spans before a screen-on write.
	// screen_device rejects repeated updates at an unchanged beam position.
	if (m_ready_timer && enabled() && vga.crtc.horz_total && vga.crtc.vert_disp_end)
		m_screen->update_now();
}

u64 oak_oti087_vga_device::pack_row(row_state const &row)
{
	return (u64(row.address) << 8) | row.scan | (unsigned(row.repeat) << 5) | (unsigned(row.split) << 6);
}

oak_oti087_vga_device::row_state oak_oti087_vga_device::unpack_row(u64 row)
{
	return {u32(row >> 8), u8(row & 31), bool(BIT(row, 5)), bool(BIT(row, 6))};
}

oak_oti087_vga_device::row_state oak_oti087_vga_device::untracked_row(unsigned y) const
{
	unsigned const interlace = BIT(m_ext[0x14], 7) ? 2 : 1;
	unsigned const line = y / interlace, field = y % interlace;
	unsigned const split_line = (vga.crtc.line_compare + 1) * vertical_divisor();
	bool const split = line >= split_line;
	unsigned const scans = split ? line - split_line : line;
	unsigned const preset = split ? 0 : m_frame_preset;
	unsigned const clocks = scans / (vga.crtc.scan_doubling + 1);
	// IBM 2-63/64: the five-bit row counter starts at Preset, not at
	// Preset divided by the double-scan rate. If Preset exceeds Maximum,
	// it wraps through 31 before the equality comparator can reset it.
	unsigned const height = vga.crtc.maximum_scan_line;
	unsigned const first = ((height - 1 - preset) & 31) + 1;
	unsigned const rows = clocks < first ? 0 : 1 + (clocks - first) / height;
	unsigned const scan = clocks < first ? (preset + clocks) & 31 : (clocks - first) % height;
	// HRS divides only the vertical timing counter (IBM 2-76). Row scan
	// still advances at horizontal retrace, subject only to double scan.
	return {(split ? 0 : vga.crtc.start_addr) + (rows * interlace + field) * vga.crtc.offset * 2,
		u8(scan), bool(vga.crtc.scan_doubling && (scans & 1)), split};
}

oak_oti087_vga_device::row_state oak_oti087_vga_device::display_row(unsigned y) const
{
	return m_row_history_frame == m_screen->frame_number() && y < m_row_history.size() ?
		unpack_row(m_row_history[y]) : untracked_row(y);
}

void oak_oti087_vga_device::advance_row(row_state &row, unsigned line, unsigned field) const
{
	unsigned const vd = vertical_divisor(), interlace = BIT(m_ext[0x14], 7) ? 2 : 1;
	if ((line + 1) % vd == 0 && line / vd == vga.crtc.line_compare)
	{
		// IBM 2-77/103: a comparison clears the line address generator;
		// it is an event, not a comparison reapplied to all future rows.
		row = {field * vga.crtc.offset * 2, 0, false, true};
	}
	else if (!vga.crtc.scan_doubling || row.repeat)
	{
		row.repeat = false;
		if (row.scan == vga.crtc.maximum_scan_line - 1)
		{
			row.scan = 0;
			row.address += vga.crtc.offset * 2 * interlace;
		}
		else
			row.scan = (row.scan + 1) & 31;
	}
	else
		row.repeat = true;
}

int oak_oti087_vga_device::begin_row_change()
{
	if ((vga.sequencer.data[0] & 3) != 3 || BIT(vga.sequencer.data[1], 5))
		return -1;
	unsigned const vd = vertical_divisor(), interlace = BIT(m_ext[0x14], 7) ? 2 : 1;
	unsigned const total = (vga.crtc.vert_total + 2) * vd;
	unsigned const line = crtc_vpos() % total, field = crtc_vpos() / total;
	unsigned const beam = line * interlace + field;
	if (line >= (vga.crtc.vert_disp_end + 1) * vd || beam >= m_row_history.size())
		return -1;
	if (m_row_history_frame != m_screen->frame_number())
	{
		for (unsigned y = 0; y != m_row_history.size(); ++y)
			m_row_history[y] = pack_row(untracked_row(y));
		m_row_history_frame = m_screen->frame_number();
	}
	return beam;
}

void oak_oti087_vga_device::end_row_change(int beam)
{
	if (beam < 0)
		return;
	unsigned const interlace = BIT(m_ext[0x14], 7) ? 2 : 1, field = beam % interlace;
	row_state row = unpack_row(m_row_history[beam]);
	// Only a register write runs this scan-line recurrence. Ordinary frames
	// use the closed form above, and renderers obtain one state per row.
	for (unsigned y = beam; y + interlace < m_row_history.size(); y += interlace)
	{
		advance_row(row, y / interlace, field);
		m_row_history[y + interlace] = pack_row(row);
	}
	if (m_character_map_timer->enabled())
		schedule_character_map();
}

u32 oak_oti087_vga_device::display_address(u32 counter, unsigned row_scan) const
{
	// IBM Figure 2-65, p2-75. Extension bits stay above the VGA mux.
	u32 address = counter;
	if (vga.crtc.dw)
		address = (counter & ~u32(0xffff)) | ((counter << 2) & 0xfffc) | ((counter >> 12) & 3);
	else if (!vga.crtc.word_mode)
		address = (counter & ~u32(0xffff)) | ((counter << 1) & 0xfffe) | BIT(counter, vga.crtc.aw ? 15 : 13);
	if (!vga.crtc.map13)
		address = (address & ~u32(0x2000)) | (BIT(row_scan, 0) << 13);
	if (!vga.crtc.map14)
		address = (address & ~u32(0x4000)) | (BIT(row_scan, 1) << 14);
	return address & m_plane_address_mask;
}

unsigned oak_oti087_vga_device::serializer_period() const
{
	// IBM 2-49/50: SH4 takes precedence over SL. The "Extended Graphics"
	// exception refers to XGA's operating mode (3-7), not sequencer EM.
	return BIT(vga.sequencer.data[1], 4) ? 4 : BIT(vga.sequencer.data[1], 2) ? 2 : 1;
}

unsigned oak_oti087_vga_device::display_skew() const
{
	unsigned const dots = BIT(vga.sequencer.data[1], 0) ? 8 : 9;
	return vga.crtc.disp_enable_skew * dots * horizontal_multiplier() / pixel_divisor();
}

void oak_oti087_vga_device::monitor_type_w(int applecolor)
{
	m_monitor_override = !applecolor;
}

void oak_oti087_vga_device::output_divider_w(int divide_by_two)
{
	m_output_divider = bool(divide_by_two);
	recompute_params();
}

u32 oak_oti087_vga_device::vclk_hz() const
{
	return clocks[csel_pins()];
}

u8 oak_oti087_vga_device::csel_pins() const
{
	// p40: ext18 redirects the physical CSEL pins, not the ext06 storage.
	// The OTI-068 sees those same pins, including DO from the EEPROM.
	return BIT(m_ext[0x18], 2) ? (m_ext[6] & 8) | (BIT(m_ext[0x18], 3) << 2) | (BIT(m_ext[0x18], 0) << 1) | m_eeprom->do_read() :
								 m_ext[6] & 15;
}

bool oak_oti087_vga_device::vertical_retrace()
{
	int const ht = m_screen->width(), vt = (vga.crtc.vert_total + 2) * vertical_divisor();
	int const field_y = crtc_vpos() % vt;
	bool const odd = get_interlace_mode() && crtc_vpos() >= vt;
	unsigned const dots = BIT(vga.sequencer.data[1], 0) ? 8 : 9;
	unsigned const char_width = dots * horizontal_multiplier();
	// OTI p40: odd-field vertical retrace starts at the character
	// position in index15, rather than invariably half a scan line.
	int const phase = odd ? (m_ext[0x15] & 0x7f) * char_width / pixel_divisor() : 0;
	int const length = (vga.crtc.vert_retrace_end - vga.crtc.vert_retrace_start) & 15;
	int const position = field_y * ht + crtc_hpos();
	int const sync_start = vga.crtc.vert_retrace_start * vertical_divisor() * ht + phase;
	return (position - sync_start + vt * ht * 2) % (vt * ht) < (length ? length : 16) * vertical_divisor() * ht;
}

int oak_oti087_vga_device::vsync_r()
{
	// OTI-087 databook p33: either transition of ext03 bit5 toggles VSYNC.
	// Bit7 qualifies only the global modes in bits2:0. The saved register
	// bit supplies the test phase; repeated writes leave the pin unchanged.
	// Assumed ext03 convention: only the pin changes; counters, Input Status 1
	// and the CRTC interrupt retain the ordinary retrace timing.
	return (vga.crtc.sync_en && vertical_retrace()) ^ BIT(vga.miscellaneous_output, 7) ^ BIT(m_ext[3], 5);
}

void oak_oti087_vga_device::system_data_w(u8 data)
{
	if (m_system_data == data)
		return;
	if (scan_test())
		flush_display();
	// Assumed ext03 convention: SD7:0 holds the last Z180 bus byte; open
	// SD15:8 supplies FF. Keep the low byte through reset, as an external input.
	m_system_data = data;
}

int oak_oti087_vga_device::hsync_r()
{
	// IBM 2-60/61: retrace skew delays the pulse by character clocks.
	unsigned const dots = BIT(vga.sequencer.data[1], 0) ? 8 : 9;
	int const total = vga.crtc.horz_total + 5;
	int const character = crtc_hpos() * pixel_divisor() / (dots * horizontal_multiplier());
	int const start = (vga.crtc.horz_retrace_start + vga.crtc.horz_retrace_skew) % total;
	int const width = (vga.crtc.horz_retrace_end - vga.crtc.horz_retrace_start) & 31;
	bool const active = (character - start + total) % total < (width ? width : 32);
	return (vga.crtc.sync_en && active) ^ BIT(vga.miscellaneous_output, 6);
}

u8 oak_oti087_vga_device::io_r(offs_t port, bool *driven)
{
	port &= 0xffff;
	if (driven)
		*driven = true;
	auto undriven = [&]() -> u8
	{
		if (driven)
			*driven = false;
		return 0xff;
	};
	if (port == 0x102)
		return BIT(m_setup, 4) ? m_enable : undriven();
	if (!enabled())
		return undriven();
	if (port == 0x3de)
		return m_index;
	if (port == 0x3df)
		return extended_r();
	if (port >= 0x3c6 && port <= 0x3c9)
		return dac_r(port & 3);
	// OTI p41: this range is in addition to the standard palette addresses.
	if (m_ext[0x19] != 15 && (port & 0xfff0) == (0x300 | (m_ext[0x19] << 4)))
		return dac_r(port & 3); // board schematic: RS2 is grounded
	switch (port)
	{
	case 0x3c0:
		return atc_address_r(0);
	case 0x3c1:
		return atc_data_r(0);
	case 0x3c2:
		return 0x60 | (((m_monitor_override < 0 ? m_monitor->read() : m_monitor_override) & 1) << 4) | (vga.crtc.irq_latch << 7);
	case 0x3c4:
		return sequencer_address_r(0);
	case 0x3c5:
		return sequencer_data_r(0);
	case 0x3ca:
		return feature_control_r(0);
	case 0x3cc:
		return miscellaneous_output_r(0);
	case 0x3ce:
		return gc_address_r(0);
	case 0x3cf:
		return gc_data_r(0);
	}
	if ((port & 0xfff0) == get_crtc_port())
	{
		switch (port & 15)
		{
		case 4:
			return crtc_address_r(0);
		case 5:
			return crtc_data_r(0);
		case 10:
		{
			// Read side effects must not occur for debugger inspections.
			if (!machine().side_effects_disabled())
				vga.attribute.state = 0;
			int const vt = vga.crtc.vert_total + 2;
			int const field_y = (crtc_vpos() / vertical_divisor()) % vt;
			unsigned const dots = BIT(vga.sequencer.data[1], 0) ? 8 : 9;
			unsigned const char_width = dots * horizontal_multiplier();
			int const hchar = crtc_hpos() * pixel_divisor() / char_width;
			int const hb_len = (vga.crtc.horz_blank_end - vga.crtc.horz_blank_start) & 63;
			int const vb_len = vertical_blank_length();
			bool const hb =
				(hchar - vga.crtc.horz_blank_start + vga.crtc.horz_total + 5) % (vga.crtc.horz_total + 5) < (hb_len ? hb_len : 64);
			bool const vb = (field_y - vga.crtc.vert_blank_start + vt * 2) % vt < vb_len;
			bool const active = hchar >= vga.crtc.disp_enable_skew &&
				hchar <= vga.crtc.horz_disp_end + vga.crtc.disp_enable_skew && field_y <= vga.crtc.vert_disp_end;
			// IBM 2-74: CRTC $17 bit 7 = 0 forces the retrace signals inactive,
			// so bit 3 stays 0. Display enable (bit 0) does not depend on it.
			return (hb || vb || !active ? 1 : 0) | (vga.crtc.sync_en && vertical_retrace() ? 8 : 0);
		}
		}
	}
	return undriven();
}

void oak_oti087_vga_device::io_w(offs_t port, u8 data)
{
	port &= 0xffff;
	// OTI pp29-30: only setup state exposes 0102; 46e8 is write-only.
	if (port == 0x46e8)
	{
		m_setup = data & 0x18;
		return;
	}
	if (port == 0x102)
	{
		if (BIT(m_setup, 4))
			m_enable = data & 1;
		return;
	}
	if (!enabled())
		return;
	if (port == 0x3de)
	{
		m_index = data;
		return;
	}
	if (port == 0x3df)
	{
		extended_w(data);
		return;
	}
	if (port >= 0x3c6 && port <= 0x3c9)
	{
		dac_w(port & 3, data);
		return;
	}
	if (m_ext[0x19] != 15 && (port & 0xfff0) == (0x300 | (m_ext[0x19] << 4)))
	{
		dac_w(port & 3, data);
		return;
	}
	switch (port)
	{
	case 0x3c0:
		flush_display();
		atc_address_data_w(0, data);
		recompute_params();
		return;
	case 0x3c2:
		flush_display();
		m_ext[6] = (m_ext[6] & 12) | ((data >> 2) & 3);
		eeprom_update();
		miscellaneous_output_w(0, data);
		return;
	case 0x3c4:
		sequencer_address_w(0, data);
		return;
	case 0x3c5:
		if (vga.sequencer.index != 2 && vga.sequencer.data[vga.sequencer.index] != data)
			flush_display();
		sequencer_data_w(0, data);
		if (vga.sequencer.index == 3)
			schedule_character_map();
		if (vga.sequencer.index == 0 && (data & 3) != 3)
			m_row_history_frame = ~u64(0);
		recompute_params();
		return;
	case 0x3ce:
		gc_address_w(0, data);
		return;
	case 0x3cf:
		if (vga.gc.index == 5 || vga.gc.index == 6)
			flush_display();
		gc_data_w(0, data);
		recompute_params();
		return;
	}
	if ((port & 0xfff0) == get_crtc_port())
	{
		switch (port & 15)
		{
		case 4:
			crtc_address_w(0, data);
			break;
		case 5:
			crtc_data_w(0, data);
			break;
		case 10:
			feature_control_w(0, data);
			break;
		}
	}
}

u8 oak_oti087_vga_device::extended_r()
{
	// Oak 87X DOS INSTALL.EXE $2B8B/$2BB1-$2C02 identifies revision E:
	// product 01, revision high nibble 2. No software reads the low nibble;
	// choose zero. Ext02 reset zero/bit0 AT-bus zero remain model choices.
	switch (m_index)
	{
	case 0:
		return 0x01;
	case 1:
		return 0x20;
	case 2:
		return m_ext[2] & 6;
	case 0x0d:
		return (m_ext[0x20] & 7) | ((m_ext[0x21] & 12) << 1) | ((m_ext[6] & 4) << 3);
	case 0x11:
		return (m_ext[0x23] & 15) | ((m_ext[0x24] & 15) << 4);
	case 0x14:
		return (m_ext[0x14] & 0x87) | ((m_ext[0x17] & 1) << 3);
	case 0x18:
		if (!machine().side_effects_disabled())
			recompute_params(); // READY/BUSY can change between clocks
		return (m_ext[0x18] & 14) | BIT(csel_pins(), 0);
	case 0x25:
		return m_ext[0x24];
	case 0x36:
	{
		u8 const data = m_latch[m_ext[0x35]];
		if (!machine().side_effects_disabled())
			m_ext[0x35] = (m_ext[0x35] + 1) & 7;
		return data;
	}
	case 0x0c:
	case 0x0e:
	case 0x0f:
	case 0x12:
	case 0x16:
	case 0x46:
		return 0xff;
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
	case 10:
	case 11:
	case 0x10:
	case 0x13:
	case 0x15:
	case 0x17:
	case 0x19:
	case 0x20:
	case 0x21:
	case 0x22:
	case 0x23:
	case 0x24:
	case 0x30:
	case 0x31:
	case 0x32:
	case 0x33:
	case 0x34:
	case 0x35:
	case 0x40:
	case 0x41:
	case 0x42:
	case 0x43:
	case 0x44:
	case 0x45:
	case 0x47:
	case 0x48:
	case 0x49:
	case 0x4a:
	case 0x4b:
	case 0x4c:
		return m_ext[m_index];
	default:
		return 0xff;
	}
}

void oak_oti087_vga_device::extended_w(u8 data)
{
	bool const old_scan_test = scan_test();
	if (m_index == 3 && ((data ^ m_ext[3]) & 0x87))
		flush_display();
	if (m_index == 6 || m_index == 0x0d || m_index == 0x14 || m_index == 0x15 || m_index == 0x21)
		flush_display();
	if (m_index >= 0x40 && m_index <= 0x4c)
	{
		flush_display();
		LOGMASKED(LOG_DIAGNOSTIC, "Oak OTI-087X VGA chip: cursor register %02x write %02x\n", m_index, data);
	}
	u8 mask = 0;
	switch (m_index)
	{
	case 2:
		mask = 6;
		break; // accepted BIOS evidence; OAK.TXT/Ferraro bit0 read-only
	case 3:
		// p33: bits4/6 are independent of global-test enable. Bit4 flushes
		// each posted write immediately; it does not clear write-cache bit3.
		// Bit6 adds no I/O waits.
		// Assumed ext03 convention: retain bit3 and reserved selectors, all
		// eight bits read back. Undescribed CRTC/waveform/reserved tests leave normal
		// operation running; report every such write, including repeats.
		mask = 0xff;
		if (BIT(data, 7) && (data & 7) != 5)
			logerror("Oak OTI-087X VGA chip: ext03=%02x test mode %u: normal operation by convention (effect undocumented)\n", data, data & 7);
		break;
	case 4:
		mask = 0x1f;
		break; // pp33-34, local bus only
	case 5:
		mask = 0xff;
		break; // pp34-35
	case 6:
		mask = 15;
		break; // p35
	case 9:
	case 10:
	case 11:
		mask = 0xff;
		break; // p37, scratch
	case 0x0d: // p38, aliases share storage
		m_ext[0x20] = (m_ext[0x20] & 8) | (data & 7);
		m_ext[0x21] = (m_ext[0x21] & ~12) | ((data >> 1) & 12);
		m_ext[6] = (m_ext[6] & ~4) | ((data >> 3) & 4);
		eeprom_update();
		recompute_params();
		return;
	case 0x11:
		m_ext[0x23] = (m_ext[0x23] & 16) | (data & 15);
		m_ext[0x24] = (m_ext[0x24] & 16) | (data >> 4);
		return;
	case 0x13:
		mask = 0xf8;
		break; // p39
	case 0x14:
		m_ext[0x17] = (m_ext[0x17] & 6) | BIT(data, 3);
		mask = 0x87;
		break; // pp39-40; bit3 lives in index17
	case 0x15:
		mask = 0xff;
		break; // p40 (odd-field sync phase)
	case 0x17:
		mask = 7;
		break;
	case 0x18:
		mask = 15;
		break;
	case 0x19:
	case 0x20:
		mask = 15;
		break; // p41
	case 0x21:
		mask = 0x8f;
		break;
	case 0x22:
		mask = 0x3d;
		break; // p42
	case 0x23:
	case 0x24:
		mask = 31;
		break;
	case 0x25:
		m_ext[0x23] = m_ext[0x24] = data & 31;
		return;
	case 0x30:
		mask = 31;
		break; // p43
	case 0x31:
	case 0x32:
	case 0x33:
	case 0x34:
		mask = 0xff;
		break;
	case 0x35:
		mask = 7;
		break; // p44
	case 0x36:
		m_latch[m_ext[0x35]] = data;
		// The indexed port addresses the same graphics latch, not a second
		// disconnected copy. In 32-bit mode the low half is the VGA latch.
		if (m_ext[0x35] < 4)
			vga.gc.latch[m_ext[0x35]] = data;
		m_ext[0x35] = (m_ext[0x35] + 1) & 7;
		return;
	case 0x40:
	case 0x41:
	case 0x42:
	case 0x43:
	case 0x44:
	case 0x45:
	case 0x47:
	case 0x48:
	case 0x49:
	case 0x4a:
	case 0x4b:
		mask = 0xff;
		break;
	case 0x4c:
		mask = 0x3f;
		break; // pp44-46
	default:
		return; // read-only straps, unused and undocumented registers
	}
	m_ext[m_index] = data & mask;
	if (m_index == 3 && old_scan_test != scan_test())
		m_scan_test_cb(scan_test());
	if (m_index == 0x21 && (data & 8) && !BIT(m_packed_layout_diagnostics, 2))
	{
		m_packed_layout_diagnostics |= 4;
		LOGMASKED(LOG_DIAGNOSTIC, "Oak OTI-087X VGA chip: reserved ext21 mode %02x uses the VGA path (model choice)\n", data & 12);
	}
	if (m_index == 6)
		vga.miscellaneous_output = (vga.miscellaneous_output & ~12) | ((data & 3) << 2);
	if (m_index == 0x18 || m_index == 6)
		eeprom_update();
	recompute_params();
}

void oak_oti087_vga_device::eeprom_update()
{
	// p40. EEP enable multiplexes CSEL0/1/2 as DO/DI/SK.
	bool const enabled = BIT(m_ext[0x18], 2);
	int const cs = BIT(m_ext[0x18], 1);
	int const di = enabled ? BIT(m_ext[0x18], 0) : BIT(m_ext[6], 1);
	int const sk = enabled ? BIT(m_ext[0x18], 3) : BIT(m_ext[6], 2);
	m_eeprom->di_write(di);
	m_eeprom->cs_write(cs);
	m_eeprom->clk_write(sk);
	// DO also changes when a write/erase completes with CS held high, even
	// without another CPU access. Retiming uses the EEPROM model's deadline.
	if (m_eeprom_ready_timer)
		m_eeprom_ready_timer->adjust(m_eeprom->ready() ? attotime::never : m_eeprom->time_until_ready());
	// Retained as observation callbacks; the EEPROM now belongs to this device.
	m_eeprom_di_cb(di);
	m_eeprom_cs_cb(cs);
	m_eeprom_clock_cb(sk);
}

void oak_oti087_vga_device::eeprom_do_w(int state)
{
	if (BIT(m_ext[0x18], 2))
		recompute_params();
}

TIMER_CALLBACK_MEMBER(oak_oti087_vga_device::eeprom_ready_tick)
{
	if (BIT(m_ext[0x18], 2))
		recompute_params();
}

void oak_oti087_vga_device::attribute_map(address_map &map)
{
	vga_device::attribute_map(map);
	// VGA mode control bits6 (8-bit pixels) and7 (palette high bits) are used
	// by these mode tables; the generic VGA map currently masks them off.
	map(0x10, 0x10).mirror(0x20).lrw8(NAME([this] { return vga.attribute.data[0x10]; }), NAME([this] (u8 data) {
		vga.attribute.data[0x10] = data;
	}));
	// IBM VGA/XGA Technical Reference (May 1992), p2-94, Figure 2-85:
	// overscan selects the DAC color with all eight bits. OTI-087 p1 specifies
	// IBM VGA register compatibility; the generic VGA map drops bits 7-6.
	map(0x11, 0x11).mirror(0x20).lrw8(NAME([this] { return vga.attribute.data[0x11]; }), NAME([this] (u8 data) {
		vga.attribute.data[0x11] = data;
	}));
	map(0x13, 0x13).mirror(0x20).lrw8(NAME([this] { return vga.attribute.data[0x13]; }), NAME([this] (u8 data) {
		vga.attribute.data[0x13] = vga.attribute.pel_shift_latch = data & 15;
		schedule_pan();
	}));
	map(0x14, 0x14).mirror(0x20).lrw8(NAME([this] { return vga.attribute.data[0x14]; }), NAME([this] (u8 data) {
		vga.attribute.data[0x14] = data & 15;
	}));
}

u8 oak_oti087_vga_device::crtc_data_r(offs_t offset)
{
	// Assumed convention: CRTC10/11 read
	// their programmed fields regardless of CRTC03 bit7. The VGA map below
	// supplies those fields, never an EGA light-pen or live address counter.
	// IBM VGA/XGA TRM 2-73: End Vertical Blanking compares all eight bits.
	return vga.crtc.index == 0x16 ? vga.crtc.vert_blank_end : vga_device::crtc_data_r(offset);
}

void oak_oti087_vga_device::crtc_data_w(offs_t offset, u8 data)
{
	flush_display();
	unsigned const index = vga.crtc.index;
	bool const row_control = index == 7 || index == 9 || index == 0x13 || index == 0x18;
	int const beam = row_control && data != vga_device::crtc_data_r(offset) ? begin_row_change() : -1;
	vga_device::crtc_data_w(offset, data);
	end_row_change(beam);
	// IBM 2-73: all eight end-blank bits are significant. Keep this fix
	// local; the shared VGA register map currently retains only seven.
	if (vga.crtc.index == 0x16)
		vga.crtc.vert_blank_end = data;
	m_vsync_cb(vga.crtc.irq_latch && !vga.crtc.irq_disable);
	recompute_params();
}

u32 oak_oti087_vga_device::latch_start_addr()
{
	return (vga.crtc.start_addr_latch & 0xffff) | ((m_ext[0x17] & 7) << 16);
}

unsigned oak_oti087_vga_device::horizontal_multiplier() const
{
	// p41. Setting both bits is not documented.
	return BIT(m_ext[0x21], 1) ? 3 : BIT(m_ext[0x21], 0) ? 2 : 1;
}

unsigned oak_oti087_vga_device::pixel_divisor() const
{
	unsigned const dac = m_ramdac->pixels_per_clock_divisor();
	return dac * (BIT(vga.attribute.data[0x10], 6) ? 2 : 1);
}

unsigned oak_oti087_vga_device::vertical_blank_length() const
{
	// IBM VGA/XGA TRM 2-73: End = Start + Width - 1 (eight low bits).
	// Unlike horizontal blanking (2-59/60), the last encoded line counts.
	return ((vga.crtc.vert_blank_end - vga.crtc.vert_blank_start) & 255) + 1;
}

void oak_oti087_vga_device::recompute_params()
{
	// IBM 2-54: EM enables 256KB rather than 64KB, i.e. 64KB rather
	// than 16KB per map. Selected decode: disable in-map A15:A14 when
	// clear, retaining the independent Oak segment bits above A15 (p42).
	// IBM does not specify the disabled address pins' electrical values.
	m_plane_address_mask = BIT(vga.sequencer.data[4], 1) ? ~u32(0) : ~u32(0xc000);
	// Overflow bits must survive subsequent standard VGA CRTC writes (p39).
	vga.crtc.vert_total = (vga.crtc.vert_total & 0x3ff) | (BIT(m_ext[0x14], 0) << 10);
	vga.crtc.vert_blank_start = (vga.crtc.vert_blank_start & 0x3ff) | (BIT(m_ext[0x14], 1) << 10);
	vga.crtc.vert_retrace_start = (vga.crtc.vert_retrace_start & 0x3ff) | (BIT(m_ext[0x14], 2) << 10);
	if (!m_ready_timer || !vga.crtc.horz_total || !vga.crtc.horz_disp_end || !vga.crtc.vert_disp_end)
		return;
	unsigned const dots = BIT(vga.sequencer.data[1], 0) ? 8 : 9;
	unsigned const mult = horizontal_multiplier();
	unsigned const div = pixel_divisor();
	unsigned const interlace = get_interlace_mode() ? 2 : 1;
	unsigned const ht = (vga.crtc.horz_total + 5) * dots * mult / div;
	unsigned const hd = (vga.crtc.horz_disp_end + 1) * dots * mult / div;
	unsigned const vt = (vga.crtc.vert_total + 2) * interlace * vertical_divisor();
	unsigned const vd = (vga.crtc.vert_disp_end + 1) * interlace * vertical_divisor();
	if (!hd || hd >= ht || !vd || vd >= vt)
		return; // programming in progress
	u32 const dotclock = vclk_hz() / (BIT(vga.sequencer.data[1], 3) ? 2 : 1) / (m_output_divider ? 2 : 1);
	// Type 0 straps pass PCLK; do not apply a second Oak DAC clock divider.
	attotime const frame = attotime::from_ticks(u64(ht) * vt * div, dotclock);
	// The monitor window holds the whole unblanked signal, centered, as the
	// owner's position and size controls would set it. Keep the pixel and
	// line counters and the U12 AppleColor timing GAL divider unchanged.
	unsigned const blank_chars = (vga.crtc.horz_blank_end - vga.crtc.horz_blank_start) & 63;
	unsigned const line_rows = interlace * vertical_divisor();
	unblanked_span const hspan = unblanked(ht, hd, vga.crtc.horz_blank_start * dots * mult / div % ht,
		std::min(ht, (blank_chars ? blank_chars : 64) * dots * mult / div));
	unblanked_span const vspan = unblanked(vt, vd, vga.crtc.vert_blank_start * line_rows % vt,
		std::min(vt, vertical_blank_length() * line_rows));
	double const line_hz = double(dotclock) / (double(ht) * div);
	bool const vga_monitor = (m_monitor_override < 0 ? m_monitor->read() : m_monitor_override) & 1;
	monitor_window window = card_monitor(vga_monitor, line_hz, line_hz * interlace / vt, double(dotclock) / div, ht, vt,
		hspan.length, vspan.length, interlace, BIT(vga.miscellaneous_output, 6), BIT(vga.miscellaneous_output, 7));
	if (!window.locked)
	{
		// No picture, as on a monitor that cannot lock. Keep the screen
		// allocation so that the guest-visible timing does not change.
		window.width = hd;
		window.height = vd;
	}
	// The lock follows at the vertical retrace once this timing has held for
	// a whole frame (display_latch_tick).
	if (window.locked != m_lock_candidate)
	{
		m_lock_candidate = window.locked;
		m_lock_since = machine().time();
	}
	m_lock_rates = { line_hz, line_hz * interlace / vt, double(vga_monitor) };
	unsigned const width = std::clamp(window.width, 1U, ht), height = std::clamp(window.height, 1U, vt);
	int const left = window.locked ? active_offset(width, hd, hspan) : floor_half(int(width) - int(hd));
	int const top = window.locked ? active_offset(height, vd, vspan) : floor_half(int(height) - int(vd));
	// A window that starts left of counter zero starts in the preceding
	// CRTC line. A linear pixel offset retains that carry when status and
	// timers cross X=0.
	unsigned const carry = left > 0 ? 1 : 0;
	unsigned const hx = (ht - left) % ht;
	unsigned const vy = (vt - top) % vt;
	rectangle const visible(0, width - 1, 0, height - 1);
	m_active_display = { s32(left), s32(top), s32(hd), s32(vd) };
	m_display_origin = { s32(hx), s32(vy) };
	m_crtc_origin = ((vy + vt - carry) % vt) * ht + hx;
	m_screen->set_output_interlace(interlace == 2, m_crtc_origin);
	// screen_device anchors time at the end of its visible area. Rebase
	// that anchor by the coordinate change, preserving subpixel time; a
	// monitor crop must not change any guest-visible H/V or interrupt edge.
	attoseconds_t const period = frame.as_attoseconds();
	int const rows = (visible.height() - int(vd) + int(vy) - int(carry) + int(vt) * 2) % int(vt);
	attotime const phase(0, rows * (period / vt) + hx * (period / (ht * vt)));
	if (m_screen->width() != ht || m_screen->height() != vt ||
		m_screen->visible_area() != visible || m_screen->frame_period() != frame || phase != m_origin_phase)
	{
		m_origin_phase = phase;
		m_screen->configure(ht, vt, visible, period, phase);
	}
	std::array<u64, 8> const memory_raster{hd, vd, vga.crtc.horz_blank_start,
		u64(m_screen->scan_period().as_attoseconds()), u64((vga.sequencer.data[0] & 3) == 3 && !BIT(vga.sequencer.data[1], 5)),
		u64(packed() ? m_ramdac->bytes_per_pixel() : vga.gc.alpha_dis ? 0 : 4),
		oti087_memory_timing::refreshes(vga.crtc.data[0x11], m_ext[0x21]), u64(phase.as_attoseconds())};
	if (memory_raster != m_memory_raster)
	{
		m_memory_raster = memory_raster;
		m_memory_next_line = attotime::zero;
	}
	schedule_display_latch();
}

u8 oak_oti087_vga_device::dac_r(u8 rs)
{
	return m_ramdac->vga_r(rs);
}

void oak_oti087_vga_device::dac_w(u8 rs, u8 data)
{
	if (rs == 2 || rs == 1)
		flush_display();
	m_ramdac->vga_w(rs, data);
	vga.dac.dirty = 1;
	recompute_params();
}

void oak_oti087_vga_device::palette_update()
{
	for (unsigned i = 0; i != 256; ++i)
		set_pen_color(i, m_ramdac->pixel_select(i));
}

u32 oak_oti087_vga_device::card_address(offs_t address, bool half) const
{
	// Firmware-derived board wiring. The full Oak bus decoder
	// below also works independently of this Second Sight convenience entry.
	return (address & 0x7ffff) | (half ? 0x80000 : 0) | (m_linear_address ? 0x100000 : 0);
}

u8 oak_oti087_vga_device::memory_r(offs_t address, bool half)
{
	if (!machine().side_effects_disabled() && m_linear_address != BIT(m_ext[5], 0))
		LOGMASKED(
			LOG_DIAGNOSTIC, "Second Sight VGA read with AA/AB A20=%u, Oak OTI-087X VGA chip ext05=%02x at Z180 %05x\n", m_linear_address, m_ext[5], address);
	return bus_memory_r(card_address(address, half));
}

void oak_oti087_vga_device::memory_w(offs_t address, bool half, u8 data)
{
	if (m_linear_address != BIT(m_ext[5], 0))
		LOGMASKED(
			LOG_DIAGNOSTIC, "Second Sight VGA write with AA/AB A20=%u, Oak OTI-087X VGA chip ext05=%02x at Z180 %05x\n", m_linear_address, m_ext[5], address);
	bus_memory_w(card_address(address, half), data);
}

bool oak_oti087_vga_device::decode_memory(offs_t address, bool write, u32 &offset) const
{
	if (!enabled() || !BIT(vga.miscellaneous_output, 1))
		return false;
	if (BIT(m_ext[5], 0) && (m_ext[5] & 0xf0))
	{
		u32 const size = 0x40000U << ((m_ext[5] >> 2) & 3);
		u32 const base = (u32(m_ext[5] & 0xf0) << 16) & ~(size - 1);
		if (address < base || address - base >= size)
			return false;
		offset = address - base;
		return true;
	}
	// p35: base zero retains the VGA aperture, including when bit0 is set.
	static constexpr u32 bases[] = {0xa0000, 0xa0000, 0xb0000, 0xb8000};
	static constexpr u32 sizes[] = {0x20000, 0x10000, 0x8000, 0x8000};
	unsigned const map = vga.gc.memory_map_sel & 3;
	if (address < bases[map] || address - bases[map] >= sizes[map])
		return false;
	offset = address - bases[map];
	if (address < 0xb0000)
		offset += u32(m_ext[write ? 0x24 : 0x23]) << 16;
	return true;
}

u32 oak_oti087_vga_device::storage_address(u32 linear) const
{
	// Internal representation only: retain the generic VGA core's four
	// 64K plane arrays per 256K block. All packed and linear paths translate
	// through the same mapping, so changing modes never copies/loses VRAM.
	return (linear & 0xc0000) | ((linear & 3) << 16) | ((linear >> 2) & 0xffff);
}

u8 oak_oti087_vga_device::linear_byte(u32 address) const
{
	// OTI p22 and the Second Sight netlist: U28 OTI-087X VGA chip
	// RASHn (pin125) is open; all eight DRAMs use RASLn (pin120).
	// Unpopulated addresses do not alias RAM. Fixed FF is a model choice
	// for undefined read/display data (15-oak-drawing, memory past 1MB).
	return address < vga.svga_intf.vram_size ? vga.memory[storage_address(address)] : 0xff;
}

void oak_oti087_vga_device::linear_byte_w(u32 address, u8 data)
{
	if (address < vga.svga_intf.vram_size)
		vga.memory[storage_address(address)] = data;
}

u8 oak_oti087_vga_device::mem_linear_r(offs_t offset)
{
	return linear_byte(offset);
}

void oak_oti087_vga_device::mem_linear_w(offs_t offset, u8 data)
{
	linear_byte_w(offset, data);
}

u8 oak_oti087_vga_device::bus_memory_r(offs_t address, bool *driven)
{
	u32 offset;
	bool const selected = decode_memory(address, false, offset);
	if (driven)
		*driven = selected;
	if (!selected)
		return 0xff;
	access_complete();
	if (!packed())
		return planar_r(offset);
	u8 latch[4];
	for (unsigned i = 0; i != 4; ++i)
		latch[i] = linear_byte((offset & ~3) + i);
	if (!machine().side_effects_disabled())
	{
		std::copy(std::begin(latch), std::end(latch), std::begin(vga.gc.latch));
		if (BIT(m_ext[0x22], 4) && BIT(vga.sequencer.data[4], 3))
		{
			// Inferred from ROM_BEEF's SHR lookup and checked against its stores.
			// An 8-bit chain-4 read loads the addressed dword into BOTH halves.
			for (unsigned i = 0; i != 4; ++i)
				m_latch[i] = m_latch[4 + i] = vga.gc.latch[i] = linear_byte((offset & ~3) + i);
		}
		else
		{
			// The firmware packed modes enable chain4. Neither
			// the databook nor period software specifies this other layout;
			// retain the aligned eight-byte load as a logged convention.
			if (BIT(m_ext[0x22], 4))
				latch_diagnostic(3, "packed latch with chain4 off: aligned eight-byte load convention", offset);
			for (unsigned i = 0; i != 8; ++i)
				m_latch[i] = linear_byte((offset & ~7) + i);
		}
	}
	if (!vga.gc.read_mode)
		return latch[BIT(vga.sequencer.data[4], 3) ? offset & 3 : vga.gc.read_map_sel & 3];
	u8 result = 0xff;
	for (unsigned p = 0; p != 4; ++p)
		if (BIT(vga.gc.color_dont_care, p))
			result &= latch[p] ^ (BIT(vga.gc.color_compare, p) ? 0 : 0xff);
	return result;
}

void oak_oti087_vga_device::bus_memory_w(offs_t address, u8 data)
{
	u32 offset;
	if (!decode_memory(address, true, offset))
		return;
	access_complete();
	if (packed())
		packed_w(offset, data);
	else
		planar_w(offset, data);
}

u32 oak_oti087_vga_device::chain4_address(u32 address) const
{
	// Keep Oak packed/wide-latch accesses on their existing byte layout.
	if (packed())
		return address >> 2;
	// Inferred from IBM VGA/XGA TRM pp2-38,2-54,2-72,2-74,2-75:
	// CH4 selects map A1:A0; within the 64K VGA aperture the map address is
	// (A & FFFC) | (A >> 14). CPU A15:A14 replace A1:A0, matching the CRTC
	// doubleword mux's recirculated MA13:MA12. IBM does not specify this
	// CPU-side map address explicitly. Retain extended bank bits above A15.
	return (address & ~u32(3)) | ((address >> 14) & 3);
}

u32 oak_oti087_vga_device::odd_even_address(u32 address) const
{
	// IBM 2-54, 2-84, 2-86: only an odd/even access uses GC Misc's A0
	// replacement. Sequential map accesses retain A0, including font loads.
	// The read/write callers select odd/even independently; it does not divide
	// the address.
	// The replacement bit is not named there. A16 is the model choice that
	// matches the IBM word-address mux with AW=1, retaining Oak bank bits.
	return vga.gc.chain_oe ? (address & ~u32(1)) | BIT(address, 16) : address;
}

u8 oak_oti087_vga_device::planar_r(u32 address)
{
	unsigned plane = vga.gc.read_map_sel & 3;
	if (BIT(vga.sequencer.data[4], 3))
	{
		plane = address & 3;
		address = chain4_address(address);
	}
	else if (vga.gc.host_oe)
	{
		plane = (plane & 2) | (address & 1);
		address = odd_even_address(address);
	}
	address &= m_plane_address_mask;
	u8 latch[4];
	for (unsigned p = 0; p != 4; ++p)
		latch[p] = linear_byte((address << 2) | p);
	if (!machine().side_effects_disabled())
	{
		std::copy(std::begin(latch), std::end(latch), std::begin(vga.gc.latch));
		for (unsigned i = 0; i != 8; ++i)
			m_latch[i] = linear_byte(((address & ~1) << 2) + i);
	}
	if (!vga.gc.read_mode)
		return latch[plane];
	u8 result = 0xff;
	for (unsigned p = 0; p != 4; ++p)
		if (BIT(vga.gc.color_dont_care, p))
			result &= latch[p] ^ (BIT(vga.gc.color_compare, p) ? 0 : 0xff);
	return result;
}

void oak_oti087_vga_device::planar_w(u32 address, u8 data)
{
	// Selected latch-address model: with bit4 clear, bit0 selects a byte from
	// CPU A2:A0, as index35 does. It changes the write-mode-1 latch source.
	bool const addressed_latch = (m_ext[0x22] & 0x11) == 1 && vga.gc.write_mode == 1;
	u8 const latch_byte = m_latch[address & 7];
	u8 mask = vga.sequencer.map_mask;
	if (BIT(vga.sequencer.data[4], 3))
	{
		mask &= 1 << (address & 3);
		address = chain4_address(address);
	}
	else if (!BIT(vga.sequencer.data[4], 2))
	{
		mask &= (address & 1) ? 10 : 5;
		address = odd_even_address(address);
	}
	address &= m_plane_address_mask;
	if (BIT(m_ext[0x30], 3))
	{
		// Oak p44: two map masks, not an eight-bit graphics bit mask.
		// p43 bit4 reverses mask bit order within the eight-lane character.
		u8 const ordered = BIT(m_ext[0x30], 4) ? bitswap<8>(m_ext[0x34], 0,1,2,3,4,5,6,7) : m_ext[0x34];
		mask = ordered >> ((address & 1) ? 0 : 4);
	}
	for (unsigned p = 0; p != 4; ++p)
	{
		if (!BIT(mask, p))
			continue;
		u8 const latch = BIT(m_ext[0x22], 4) ? m_latch[p + 4 * ((address & 1) ^ BIT(m_ext[0x22], 5))] : vga.gc.latch[p];
		u8 value = addressed_latch ? latch_byte : graphics_write(p, data, latch);
		if ((m_ext[0x30] & 3) == 3)
		{
			u8 const pattern = BIT(m_ext[0x30], 2) ? m_ext[0x33] : data;
			u8 const expanded = (BIT(m_ext[0x31], p) ? pattern : 0) | (BIT(m_ext[0x32], p) ? ~pattern : 0);
			value = graphics_write(p, expanded, latch);
		}
		linear_byte_w((address << 2) | p, value);
	}
}

u8 oak_oti087_vga_device::graphics_write(unsigned plane, u8 data, u8 latch) const
{
	// IBM 2-79..88, identical equations to the shared VGA core, with an
	// explicit latch input for Oak's eight-byte/swap paths. Writes never
	// reload the latch. The common unmasked CPU store returns immediately.
	if (!vga.gc.write_mode && !vga.gc.rotate_count && !vga.gc.logical_op &&
		!vga.gc.enable_set_reset && vga.gc.bit_mask == 0xff)
		return data;
	if (vga.gc.write_mode == 1)
		return latch;
	u8 const rotated = (data >> vga.gc.rotate_count) | (data << ((8 - vga.gc.rotate_count) & 7));
	u8 mask = vga.gc.bit_mask;
	u8 value = rotated;
	if (vga.gc.write_mode == 3)
	{
		mask &= rotated;
		value = BIT(vga.gc.set_reset, plane) ? 0xff : 0;
	}
	else if (vga.gc.write_mode == 2)
		value = BIT(data, plane) ? 0xff : 0;
	else if (BIT(vga.gc.enable_set_reset, plane))
		value = BIT(vga.gc.set_reset, plane) ? 0xff : 0;
	switch (vga.gc.logical_op)
	{
	case 1: value &= latch; break;
	case 2: value |= latch; break;
	case 3: value ^= latch; break;
	}
	return (value & mask) | (latch & ~mask);
}

void oak_oti087_vga_device::latch_diagnostic(unsigned bit, char const *reason, u32 address)
{
	// Report each unusual latch path once per reset, without hot-path spam.
	if (!BIT(m_latch_diagnostics, bit))
	{
		m_latch_diagnostics |= 1 << bit;
		LOGMASKED(LOG_DIAGNOSTIC, "Oak OTI-087X VGA chip: latch access: %s address=%06x mode=%u ext22=%02x ext30=%02x time=%s\n", reason, address,
			vga.gc.write_mode, m_ext[0x22], m_ext[0x30], machine().time().as_string());
	}
}

void oak_oti087_vga_device::packed_w(u32 address, u8 data)
{
	bool const pixel_mask = BIT(m_ext[0x30], 3);
	bool const expand = (m_ext[0x30] & 3) == 1;
	bool const wide_chain4 = BIT(m_ext[0x22], 4) && BIT(vga.sequencer.data[4], 3);
	if (wide_chain4 && !pixel_mask && !expand)
	{
		if (vga.gc.write_mode == 1)
			latch_diagnostic(1, "mode1 without pixel mask uses standard VGA path", address);
		// Mode0 table construction must retain CPU byte addressing even with
		// ext22=1c. ROM_BEEF sub_AE92 uses the normal VGA write path.
		planar_w(address, data);
		return;
	}
	// Ext22 bit4 widens the VGA graphics latch; it does not select it as
	// the write source. VGA write mode 1 does that. ROM_BEEF sub_AE92 builds
	// the SHR lookup table with ordinary mode-0 byte stores while bit4 is
	// already set, then selects mode1 and ext30 pixel masking for the blit.
	bool const addressed_latch = (m_ext[0x22] & 0x11) == 1;
	bool const latch = vga.gc.write_mode == 1;
	// OTI p43: pixel masking disables packed-pixel address/map masking.
	// With ordinary masking, a byte store still addresses only one byte.
	// ROM_BEEF SHR lookup and XFree86 3.3.6 oti87_fill.c imply A0 selects
	// one dword of the character; A1/A2 do not select a byte lane. Ext24's
	// segment is already in address. This also fits render_lores_linepair;
	// XFree86 3.3.6 oti87_fill.c writes offsets 0/1.
	// 15-oak-drawing, "What no Oak document defines": LIN8_SC.DRV
	// $1AFD-$1B96 fills with mode0 plus pixel mask. Probable 8-bit rule:
	// A0 selects four pixels, just as mode1/expansion do; A1:A2 ignored.
	bool const half_store = BIT(vga.sequencer.data[4], 3) && (expand || pixel_mask);
	u32 const base = half_store ? (address & ~7) + 4 * (address & 1) : (expand || pixel_mask) ? (address & ~7) : address;
	unsigned const count = half_store ? 4 : (expand || pixel_mask) ? 8 : 1;
	u8 const pattern = BIT(m_ext[0x30], 2) ? m_ext[0x33] : data;
	for (unsigned i = 0; i != count; ++i)
	{
		unsigned const pos = (base + i) & 7;
		if (pixel_mask)
		{
			// p43: bit4 set means mask bit7 is pixel0; clear means bit0 is
			// pixel0 (the default order described on p44).
			if (!BIT(m_ext[0x34], BIT(m_ext[0x30], 4) ? 7 - pos : pos))
				continue;
		}
		else if (!BIT(vga.sequencer.map_mask, (base + i) & 3))
			continue;
		u8 value = expand ? m_ext[BIT(pattern, 7 - pos) ? 0x31 : 0x32] : data;
		u8 const source = BIT(m_ext[0x22], 4) ? m_latch[pos ^ (BIT(m_ext[0x22], 5) ? 4 : 0)] : vga.gc.latch[(base + i) & 3];
		if (latch)
			value = addressed_latch ? m_latch[address & 7] : source;
		else
			// LIN8_SC.DRV $2736-$2773 proves XOR. Applying the other
			// IBM logic ops, SR and bit mask per latch byte is probable:
			// 15-oak-drawing, "What no Oak document defines".
			value = graphics_write((base + i) & 3, value, source);
		linear_byte_w(base + i, value);
	}
}

void oak_oti087_vga_device::draw_packed(bitmap_rgb32 &bitmap, rectangle const &cliprect, int ox, int oy)
{
	unsigned const bytes = m_ramdac->bytes_per_pixel();
	unsigned const skew = display_skew();
	// Oak packed modes use two eight-byte fetches per offset unit. Inferred:
	// packed mode bypasses the legacy VGA word/dword address mux.
	// Oak's BIOS uses byte mode here; Second Sight ROM1.4 mode13 combines
	// packed mode with VGA dword/PEL-width settings. GView's period README
	// reports that 320x200x256 fails. Do not halve the pitch to repair it.
	u32 const pitch = vga.crtc.offset * 16;
	u32 const start = vga.crtc.start_addr * 8;
	if ((vga.crtc.dw || !vga.crtc.word_mode) && !BIT(m_packed_layout_diagnostics, 0))
	{
		m_packed_layout_diagnostics |= 1;
		LOGMASKED(LOG_DIAGNOSTIC,
			"Oak OTI-087X VGA chip: packed address inference: ext21=%02x CRTC14=%02x CRTC17=%02x offset=%03x; bypass VGA word/dword shift, pitch=%u start=%u time=%s\n",
			m_ext[0x21], vga.crtc.data[0x14], vga.crtc.data[0x17], vga.crtc.offset, pitch, start, machine().time().as_string());
	}
	if (BIT(vga.attribute.data[0x10], 6) && !BIT(m_packed_layout_diagnostics, 1))
	{
		m_packed_layout_diagnostics |= 2;
		LOGMASKED(LOG_DIAGNOSTIC,
			"Oak OTI-087X VGA chip: packed PEL-width inference: ext21=%02x attribute10=%02x; retain bit6 divide-by-two pixel clock/width, direct packed bytes (nibble pairing undocumented) time=%s\n",
			m_ext[0x21], vga.attribute.data[0x10], machine().time().as_string());
	}
	// The screen callback is inside the card's execution barrier. Palette,
	// mask and command state cannot change during it; keep the exact DAC
	// result for each 8-bit input only until this callback returns.
	std::array<u32, 256> colors;
	if (bytes == 1)
		for (unsigned i = 0; i != colors.size(); ++i)
			colors[i] = m_ramdac->direct_color(i);
	u32 previous_address = 0;
	for (int y = cliprect.min_y; y <= cliprect.max_y; ++y)
	{
		row_state const row = display_row(y);
		bool const split = row.split;
		// Oak does not specify packed panning units. Model choice: byte-pan
		// advances an eight-byte fetch; pel-pan advances a complete DAC pixel.
		unsigned const pan = split && BIT(vga.attribute.data[0x10], 5) ? 0 : vga.attribute.pel_shift & 7;
		u32 const address = row.address * 8 + vga.crtc.byte_panning * 8 + (pan - skew) * bytes;
		// Repeated scanlines fetch the identical pixels. VRAM and DAC state
		// stay fixed inside this callback's barrier; copy only this clip's
		// completed span, before the cursor/overscan layers are applied.
		if (y != cliprect.min_y && address == previous_address)
		{
			std::copy_n(&bitmap.pix(y - 1 + oy, cliprect.min_x + ox), cliprect.width(), &bitmap.pix(y + oy, cliprect.min_x + ox));
			continue;
		}
		previous_address = address;
		if (bytes == 1)
		{
			int x = cliprect.min_x;
			u32 const first = address + x;
			int const populated_end = first < vga.svga_intf.vram_size
				? std::min(cliprect.max_x, x + int(vga.svga_intf.vram_size - first) - 1) : x - 1;
			for (; x <= populated_end && ((address + x) & 3); ++x)
				bitmap.pix(y + oy, x + ox) = colors[linear_byte(address + x)];
			// Four consecutive linear bytes occupy the same offset in the
			// four plane arrays. Decode that shared offset once; retain the
			// original pixel order and 256K block selection. RASHn is open
			// beyond the populated megabyte; never use the wrapped storage index.
			for (; x + 3 <= populated_end; x += 4)
			{
				u32 const base = storage_address(address + x);
				auto *const dest = &bitmap.pix(y + oy, x + ox);
				dest[0] = colors[vga.memory[base]];
				dest[1] = colors[vga.memory[base + 0x10000]];
				dest[2] = colors[vga.memory[base + 0x20000]];
				dest[3] = colors[vga.memory[base + 0x30000]];
			}
			for (; x <= populated_end; ++x)
				bitmap.pix(y + oy, x + ox) = colors[linear_byte(address + x)];
			if (x <= cliprect.max_x)
				std::fill_n(&bitmap.pix(y + oy, x + ox), cliprect.max_x - x + 1, colors[0xff]);
			continue;
		}
		for (int x = cliprect.min_x; x <= cliprect.max_x; ++x)
		{
			// Oak 87X 8ETRU640.DRV $C7CA-$C806/$1D226-$1D228:
			// the Bt481 receives R,G,B at increasing addresses (CR A=F0).
			u32 pixel = 0;
			for (unsigned b = 0; b != bytes; ++b)
				pixel |= u32(linear_byte(address + x * bytes + b)) << (8 * b);
			bitmap.pix(y + oy, x + ox) = m_ramdac->direct_color(pixel);
		}
	}
}

rgb_t oak_oti087_vga_device::attribute_color(u8 index) const
{
	index &= vga.attribute.data[0x12] & 15;
	u8 palette = vga.attribute.data[index];
	if (BIT(vga.attribute.data[0x10], 7))
		palette = (palette & 15) | ((vga.attribute.data[0x14] & 3) << 4);
	palette |= (vga.attribute.data[0x14] & 12) << 4;
	return m_ramdac->pixel_select(palette);
}

template <bool ScanTest>
void oak_oti087_vga_device::draw_text(bitmap_rgb32 &bitmap, rectangle const &cliprect, int ox, int oy)
{
	// Standard VGA character/attribute planes 0/1 and font plane2. Use the
	// same physical VRAM translation as CPU accesses, including odd/even mode.
	unsigned const dots = BIT(vga.sequencer.data[1], 0) ? 8 : 9;
	unsigned const pan = dots == 9 ? (vga.attribute.pel_shift + 1) % 9 : vga.attribute.pel_shift & 7;
	bool const blink = BIT(vga.attribute.data[0x10], 3);
	auto const frame = m_screen->frame_number();
	bool const blink_off = BIT(frame, 5);
	bool const cursor_frame = BIT(frame, 4);
	bool const extended = BIT(vga.sequencer.data[4], 1);
	bool const two_fonts = extended && m_character_map[0] != m_character_map[1];
	unsigned const load = serializer_period();
	unsigned const count_divisor = vga.crtc.div4 ? 4 : vga.crtc.div2 ? 2 : 1;
	unsigned const skew = display_skew();
	std::array<u32, 16> colors;
	for (unsigned i = 0; i != colors.size(); ++i)
		colors[i] = attribute_color(i);
	for (int y = cliprect.min_y; y <= cliprect.max_y; ++y)
	{
		row_state const row = display_row(y);
		bool const split = row.split;
		unsigned const glyph_row = row.scan;
		u32 const start = row.address + vga.crtc.byte_panning;
		unsigned const row_pan = (split && BIT(vga.attribute.data[0x10], 5)) ? 0 : pan;
		for (int x = cliprect.min_x; x <= cliprect.max_x;)
		{
			unsigned const dx = x - skew + row_pan;
			u32 const cell = start + dx / (dots * count_divisor);
			u32 const address = display_address(cell, glyph_row) * 4;
			u8 const ch = linear_byte(address), attr = ScanTest ? m_system_data : linear_byte(address + 1);
			unsigned const bank = !extended ? 0 : m_character_map[BIT(attr, 3)];
			u32 const font = bank * 0x2000 + ch * 32 + (glyph_row & 31);
			u8 const glyph = ScanTest ? 0xff : linear_byte(font * 4 + 2);
			unsigned column = dx % dots;
			// The character, font and attribute are identical across this
			// cell's clipped span. Keep the ninth dot, blink and cursor rules.
			unsigned pattern = (unsigned(glyph) << 1) | ((ch & 0xe0) == 0xc0 && BIT(vga.attribute.data[0x10], 2) && BIT(glyph, 0));
			// IBM monochrome attributes and 2-99: underline keeps the ninth
			// dot clear except on enabled C0..DF line-graphics characters.
			if ((attr & 0x77) == 1 && glyph_row == vga.crtc.underline_loc)
				pattern = 0x1fe | (((ch & 0xe0) == 0xc0) && BIT(vga.attribute.data[0x10], 2));
			if ((dx / dots) % load)
				pattern = 0; // exhausted eight-bit serializer between reloads
			if (blink && blink_off && BIT(attr, 7))
				pattern = 0;
			if (vga.crtc.cursor_enable && cursor_frame && cell == vga.crtc.cursor_addr + vga.crtc.cursor_skew && glyph_row >= vga.crtc.cursor_scan_start &&
				glyph_row <= vga.crtc.cursor_scan_end)
				pattern = 0x1ff;
			u32 const foreground = colors[attr & (two_fonts ? 7 : 15)], background = colors[(attr >> 4) & (blink ? 7 : 15)];
			int const end = x + std::min(dots - column, unsigned(cliprect.max_x - x + 1));
			for (; x < end; ++x, ++column)
				bitmap.pix(y + oy, x + ox) = BIT(pattern, 8 - column) ? foreground : background;
		}
	}
}

void oak_oti087_vga_device::draw_scan_test(bitmap_rgb32 &bitmap, rectangle const &cliprect, int ox, int oy)
{
	// OTI-087 p33 mode101 replaces the attribute controller's inputs, not
	// VRAM or the GC latches. Assumed ext03 convention: unconnected SD15:8 is
	// FF; text SD7:0 is the saved Z180 bus byte. Dispatch once per clip, so no
	// test-mode condition is added to ordinary per-pixel rendering.
	if (packed())
		bitmap.fill(m_ramdac->direct_color(0xffffffff), output_span(cliprect, ox, oy));
	else if (!vga.gc.alpha_dis)
		draw_text<true>(bitmap, cliprect, ox, oy);
	else if (BIT(vga.attribute.data[0x10], 6))
	{
		u8 const nibble = vga.attribute.data[0x12] & 15;
		bitmap.fill(m_ramdac->pixel_select((nibble << 4) | nibble), output_span(cliprect, ox, oy));
	}
	else
		bitmap.fill(attribute_color(15), output_span(cliprect, ox, oy));
}

void oak_oti087_vga_device::draw_planar(bitmap_rgb32 &bitmap, rectangle const &cliprect, int ox, int oy)
{
	draw_graphics(bitmap, cliprect, ox, oy);
}

void oak_oti087_vga_device::draw_vga256(bitmap_rgb32 &bitmap, rectangle const &cliprect, int ox, int oy)
{
	draw_graphics(bitmap, cliprect, ox, oy);
}

void oak_oti087_vga_device::draw_graphics(bitmap_rgb32 &bitmap, rectangle const &cliprect, int ox, int oy)
{
	// IBM 2-84: all serializers feed the same four attribute input wires.
	// PW pairs consecutive samples into one byte; without PW each sample
	// goes through the sixteen palette registers and Color Select.
	bool const pair = BIT(vga.attribute.data[0x10], 6);
	unsigned const count_divisor = vga.crtc.div4 ? 4 : vga.crtc.div2 ? 2 : 1;
	unsigned const load = serializer_period();
	unsigned const skew = display_skew();
	u8 const planes = vga.attribute.data[0x12] & 15;
	bool const nine = !BIT(vga.sequencer.data[1], 0);
	std::array<u32, 16> colors;
	for (unsigned i = 0; i != 16; ++i)
		colors[i] = attribute_color(i);
	for (int y = cliprect.min_y; y <= cliprect.max_y; ++y)
	{
		row_state const row = display_row(y);
		bool const split = row.split;
		unsigned const row_scan = row.scan;
		u32 const start = row.address + vga.crtc.byte_panning;
		unsigned const pan = split && BIT(vga.attribute.data[0x10], 5) ? 0 :
			pair ? vga.attribute.pel_shift & 6 : vga.attribute.pel_shift & 7;
		auto sample = [&](unsigned dot) -> u8 {
			unsigned const character = nine ? dot / 9 : dot / 8;
			if (character % load)
				return 0; // no reload; shifted-out stages are zero (model choice)
			u32 const counter = start + character / count_divisor;
			u32 const address = display_address(counter, row_scan) * 4;
			unsigned const bit = nine ? dot % 9 : dot & 7;
			if (bit == 8) return 0; // graphics in 9-dot timing: no ninth serializer bit
			u8 color = 0;
			if (vga.gc.shift256)
			{
				// Each map supplies its high nibble followed by its low nibble.
				// This also defines the previously missing C256 + PW=0 case.
				color = (linear_byte(address + bit / 2) >> ((bit & 1) ? 0 : 4)) & 15;
			}
			else if (vga.gc.shift_reg)
			{
				// CGA interleave: four two-bit pixels from maps 0/2, then
				// four from maps 1/3. Maps 2/3 provide attribute bits 3:2.
				unsigned const map = bit / 4, shift = 6 - 2 * (bit & 3);
				color = ((linear_byte(address + map) >> shift) & 3) |
					(((linear_byte(address + map + 2) >> shift) & 3) << 2);
			}
			else
				for (unsigned p = 0; p != 4; ++p)
					color |= BIT(linear_byte(address + p), 7 - bit) << p;
			return color & planes;
		};
		// Keep the ordinary 8-bit VGA path at one byte fetch per pixel.
		// Unused nibble/9-dot/plane-disable features add no per-pixel work.
		if (vga.gc.shift256 && pair && planes == 15 && load == 1 && !nine)
		{
			for (int x = cliprect.min_x; x <= cliprect.max_x; ++x)
			{
				unsigned const dx = x - skew + pan / 2;
				u32 const address = display_address(start + dx / (4 * count_divisor), row_scan);
				bitmap.pix(y + oy, x + ox) = m_ramdac->pixel_select(linear_byte((address << 2) | (dx & 3)));
			}
			continue;
		}
		for (int x = cliprect.min_x; x <= cliprect.max_x; ++x)
		{
			unsigned const dot = (x - skew) * (pair ? 2 : 1) + pan;
			bitmap.pix(y + oy, x + ox) = pair ? u32(m_ramdac->pixel_select((sample(dot) << 4) | sample(dot + 1))) : colors[sample(dot)];
		}
	}
}

bool oak_oti087_vga_device::blanked(int x, int y) const
{
	int const ht = vga.crtc.horz_total + 5, vt = vga.crtc.vert_total + 2;
	unsigned const dots = BIT(vga.sequencer.data[1], 0) ? 8 : 9;
	int const hx = x * pixel_divisor() / (dots * horizontal_multiplier());
	int const vy = y / ((BIT(m_ext[0x14], 7) ? 2 : 1) * vertical_divisor());
	int const hb = (vga.crtc.horz_blank_end - vga.crtc.horz_blank_start) & 63;
	int const vb = vertical_blank_length();
	return (hx - vga.crtc.horz_blank_start + ht * 2) % ht < (hb ? hb : 64) ||
		(vy - vga.crtc.vert_blank_start + vt * 2) % vt < vb;
}

u8 oak_oti087_vga_device::cursor_background_index(row_state const &row, int x) const
{
	// Resolve only inverted cursor pixels, before the Bt481 palette/mask.
	// Reuse the background's row history and address mux; the ordinary
	// rendering loops need no index buffer or per-pixel cursor condition.
	bool const reset_pan = row.split && BIT(vga.attribute.data[0x10], 5);
	unsigned const dx = x - display_skew();
	if (packed())
	{
		unsigned const pan = reset_pan ? 0 : vga.attribute.pel_shift & 7;
		return linear_byte(row.address * 8 + vga.crtc.byte_panning * 8 + dx + pan);
	}
	// Standard VGA C256 + paired PELs: the same two serializer nibbles
	// as draw_graphics, including plane enables, shift-load and nine dots.
	u32 const start = row.address + vga.crtc.byte_panning;
	unsigned const count_divisor = vga.crtc.div4 ? 4 : vga.crtc.div2 ? 2 : 1;
	unsigned const load = serializer_period();
	bool const nine = !BIT(vga.sequencer.data[1], 0);
	unsigned const dot = dx * 2 + (reset_pan ? 0 : vga.attribute.pel_shift & 6);
	auto sample = [&](unsigned position) -> u8 {
		unsigned const character = nine ? position / 9 : position / 8;
		unsigned const bit = nine ? position % 9 : position & 7;
		if (character % load || bit == 8)
			return 0;
		u32 const address = display_address(start + character / count_divisor, row.scan) * 4;
		return (linear_byte(address + bit / 2) >> ((bit & 1) ? 0 : 4)) & vga.attribute.data[0x12] & 15;
	};
	return (sample(dot) << 4) | sample(dot + 1);
}

template <bool ScanTest>
void oak_oti087_vga_device::draw_cursor(bitmap_rgb32 &bitmap, rectangle const &cliprect, int ox, int oy)
{
	u8 const control = m_ext[0x4c];
	// WHATVGA 2.00 reports real Oak hardware: cursor only in 256-color
	// modes. Ext4A/4B are palette indices (OTI p46), through the Bt481.
	bool const color256 = m_ramdac->bytes_per_pixel() == 1 &&
		(packed() || (vga.gc.alpha_dis && vga.gc.shift256 && BIT(vga.attribute.data[0x10], 6)));
	if (!BIT(control, 0) || !color256 || m_ramdac->asleep())
		return;
	if (BIT(control, 3) && BIT(m_screen->frame_number(), 2 + ((control >> 4) & 3)))
		return;
	int const px = m_ext[0x44], py = m_ext[0x45];
	if (px > 63 || py > 63)
		return; // p45: presets end at 63, never wrap
	// Oak's 087X Windows driver STD8_HC.DRV (SHA-256 523f60c8...):
	// $56AD-$56C6 subtract the hot spot; $5537-$557A write signed X/Y.
	// Y is top-1 only if that is nonnegative; top<=0 is written unchanged.
	// Inferred chip rule: first row is register Y+1, with signed clipping.
	// Convention for X not divisible by4 and X=-14: draw the register
	// position exactly. WHATVGA and Oak driver restrictions suggest real
	// faults there but do not specify their pixels (06-video, Hardware cursor).
	int const x0 = s16((m_ext[0x40] << 8) | m_ext[0x41]);
	int const y0 = s16((m_ext[0x42] << 8) | m_ext[0x43]);
	// Eight-byte units also reconcile Oak's driver: 1fe80*8 + 1f*16 + 20/8
	// (address + Y/X presets) = ff5f4, its bank0f:f5f4 destination.
	u32 const address = ((m_ext[0x47] << 16) | (m_ext[0x48] << 8) | m_ext[0x49]) << 3;
	// Probable: compare Y to the per-field count, repeating each pattern
	// row in both interlaced fields (06-video, Hardware cursor).
	int const repeat = get_interlace_mode() ? 2 : 1;
	int const first_y = (y0 + 1) * repeat;
	rectangle area(x0, x0 + 63 - px, first_y, first_y + (64 - py) * repeat - 1);
	area &= cliprect;

	// p46 bit1 permits the cursor over overscan, but never sync/blanking.
	if (!BIT(control, 1))
		area &= active_raster();
	int index_y = -1;
	row_state index_row{};
	for (int y = area.min_y; y <= area.max_y; ++y)
		for (int x = area.min_x; x <= area.max_x; ++x)
		{
			if (blanked(x, y))
				continue;
			unsigned const sx = x - x0 + px, sy = (y - first_y) / repeat + py;
			// Oak SetCursor uses the first eight bytes of each row for plane A
			// and the next eight for B. Its per-byte nibble swap reorders pixels,
			// never planes. Bit2 retains p46's bit0/bit7 ordering; the driver
			// establishes which Windows mask belongs to each plane.
			u8 const a = linear_byte(address + sy * 16 + sx / 8);
			u8 const b = linear_byte(address + sy * 16 + 8 + sx / 8);
			unsigned const bit = BIT(control, 2) ? 7 - (sx & 7) : (sx & 7);
			switch (cursor_codes[(BIT(a, bit) << 1) | BIT(b, bit)])
			{
			// The cursor colors use the same DAC palette and pixel mask.
			// Oak Feb 1993 p21: P[7:0] feeds the external palette chip;
			// p46's cursor colors are eight-bit indices. Code11 from the
			// reference's Hardware cursor plane table complements that bus.
			case cursor_op::color0:
				bitmap.pix(y + oy, x + ox) = m_ramdac->pixel_select(m_ext[0x4a]);
				break;
			case cursor_op::color1:
				bitmap.pix(y + oy, x + ox) = m_ramdac->pixel_select(m_ext[0x4b]);
				break;
			case cursor_op::transparent:
				break;
			case cursor_op::invert:
			{
				u8 index = vga.attribute.data[0x11]; // overscan is also indexed
				if (active_raster().contains(x, y) && x >= int(display_skew()))
				{
					if constexpr (ScanTest)
					{
						u8 const nibble = vga.attribute.data[0x12] & 15;
						index = packed() ? 0xff : (nibble << 4) | nibble;
					}
					else
					{
						if (index_y != y)
						{
							index_y = y;
							index_row = display_row(y);
						}
						index = cursor_background_index(index_row, x);
					}
				}
				bitmap.pix(y + oy, x + ox) = m_ramdac->pixel_select(index ^ 0xff);
				break;
			}
			}
		}
}

u32 oak_oti087_vga_device::draw_raster(bitmap_rgb32 &bitmap, rectangle const &cliprect, int ox, int oy)
{
	rectangle const bounds = cliprect;

	// CRTC17 bit7 only gates sync (IBM 2-73), not the pixel pipeline.
	if (!enabled() || (vga.sequencer.data[0] & 3) != 3 || BIT(vga.sequencer.data[1], 5))
	{
		bitmap.fill(rgb_t::black(), output_span(bounds, ox, oy));
		return 0;
	}
	// Overscan also supplies the left and top border before counter zero.
	bitmap.fill(m_ramdac->pixel_select(vga.attribute.data[0x11]), output_span(bounds, ox, oy));
	if (!BIT(vga.attribute.index, 5))
		return 0;
	rectangle active = bounds & active_raster();
	unsigned const skew = display_skew();
	if (skew && !active.empty())
	{
		bitmap.fill(m_ramdac->pixel_select(vga.attribute.data[0x11]), output_span(active, ox, oy));
		active.min_x = std::max(active.min_x, int(skew));
	}
	bool const test = scan_test();
	if (!active.empty())
	{
		if (test)
			draw_scan_test(bitmap, active, ox, oy);
		else if (packed())
			draw_packed(bitmap, active, ox, oy);
		else if (!vga.gc.alpha_dis)
			draw_text(bitmap, active, ox, oy);
		else if (vga.gc.shift256)
			draw_vga256(bitmap, active, ox, oy);
		else if (!vga.gc.shift256 && !vga.gc.shift_reg)
			draw_planar(bitmap, active, ox, oy);
		else
			draw_graphics(bitmap, active, ox, oy);
	}
	if (test)
		draw_cursor<true>(bitmap, bounds, ox, oy);
	else
		draw_cursor(bitmap, bounds, ox, oy);
	// Apply the programmed blanking intervals, including intervals moved
	// inside active display. Ordinary modes have no horizontal intersection;
	// do not add a blanking test to their pixel loops.
	unsigned const dots = BIT(vga.sequencer.data[1], 0) ? 8 : 9;
	int const ht = vga.crtc.horz_total + 5, vt = vga.crtc.vert_total + 2;
	int const unit = dots * horizontal_multiplier();
	int const hb = vga.crtc.horz_blank_start % ht;
	int const hw = (vga.crtc.horz_blank_end - vga.crtc.horz_blank_start) & 63;
	int const he = hb + (hw ? hw : 64);
	int const vb = vga.crtc.vert_blank_start % vt;
	int const vw = vertical_blank_length();
	int const repeat = vertical_divisor() * (get_interlace_mode() ? 2 : 1);
	auto black_span = [&](int y, int left, int right) {
		left = std::max(left * unit / int(pixel_divisor()), bounds.min_x);
		right = std::min(right * unit / int(pixel_divisor()) - 1, bounds.max_x);
		if (left <= right) std::fill_n(&bitmap.pix(y + oy, left + ox), right - left + 1, rgb_t::black());
	};
	for (int y = bounds.min_y; y <= bounds.max_y; ++y)
	{
		if ((y / repeat - vb + vt * 2) % vt < vw)
			std::fill_n(&bitmap.pix(y + oy, bounds.min_x + ox), bounds.width(), rgb_t::black());
		else
		{
			black_span(y, hb, std::min(he, ht));
			if (he > ht) black_span(y, 0, he - ht);
		}
	}
	return 0;
}

// Convert only at the screen boundary. Renderers, row history, line compare
// and cursor coordinates remain in CRTC space. Splitting at the two wraps
// gives at most four spans, each with a constant destination offset.
u32 oak_oti087_vga_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, rectangle const &cliprect)
{
	rectangle const bounds = cliprect & bitmap.cliprect();
	if (m_out_of_range)
	{
		bitmap.fill(rgb_t::black(), bounds);
		return 0;
	}
	int const w = screen.width(), h = screen.height();
	for (int y = bounds.min_y; y <= bounds.max_y;)
	{
		int const cy = (y + m_display_origin[1]) % h;
		int const ny = std::min(bounds.max_y + 1, y + h - cy);
		for (int x = bounds.min_x; x <= bounds.max_x;)
		{
			int const cx = (x + m_display_origin[0]) % w;
			int const nx = std::min(bounds.max_x + 1, x + w - cx);
			draw_raster(bitmap, rectangle(cx, cx + nx - x - 1, cy, cy + ny - y - 1), x - cx, y - cy);
			x = nx;
		}
		y = ny;
	}
	return 0;
}

int oak_oti087_vga_device::crtc_hpos() const
{
	return (m_screen->vpos() * m_screen->width() + m_screen->hpos() + m_crtc_origin) % m_screen->width();
}

int oak_oti087_vga_device::crtc_vpos() const
{
	return ((m_screen->vpos() * m_screen->width() + m_screen->hpos() + m_crtc_origin) /
		m_screen->width()) % m_screen->height();
}

attotime oak_oti087_vga_device::time_until_crtc_pos(int y, int x) const
{
	int const width = m_screen->width(), total = width * m_screen->height();
	int const position = (y * width + x - m_crtc_origin + total) % total;
	return m_screen->time_until_pos(position / width, position % width);
}

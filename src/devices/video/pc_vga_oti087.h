// license:BSD-3-Clause
#ifndef MAME_VIDEO_PC_VGA_OTI087_H
#define MAME_VIDEO_PC_VGA_OTI087_H

#pragma once

#include "pc_vga.h"
#include "bt48x.h"
#include "machine/eepromser.h"
#include "oti087_memory.h"
#include "oti087_lcd.h"

// Second Sight's Oak OTI-087X VGA chip, Bt481 color chip (RAMDAC) and
// OTI-068 clock generator. Addresses passed to io_r/w are full port numbers,
// including the AT add-on setup ports.
// memory_r/w (without a half argument) and bus_memory_r/w use the Oak's
// 24-bit address. The half-argument overloads are an optional card adapter:
// Z180 A18..0, separately latched A19, and the independent AA/AB A20 input.
// The board owns these latches and its A8/A9 strobes; it drives the input
// setters below. Oak ext05 selects aperture decoding, never the board's A20.
class oak_oti087_vga_device : public svga_device
{
  public:
	oak_oti087_vga_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock = 0);

	u8 io_r(offs_t port, bool *driven = nullptr);
	void io_w(offs_t port, u8 data);

	u8 memory_r(offs_t address, bool *driven = nullptr) { return bus_memory_r(address, driven); }

	void memory_w(offs_t address, u8 data) { bus_memory_w(address, data); }

	u8 memory_r(offs_t address, bool half);
	void memory_w(offs_t address, bool half, u8 data);
	u8 bus_memory_r(offs_t address, bool *driven = nullptr);
	void bus_memory_w(offs_t address, u8 data);

	virtual u8 mem_r(offs_t offset) override { return bus_memory_r(0xa0000 + offset); }

	virtual void mem_w(offs_t offset, u8 data) override { bus_memory_w(0xa0000 + offset, data); }

	virtual u8 mem_linear_r(offs_t offset) override;
	virtual void mem_linear_w(offs_t offset, u8 data) override;

	auto iochrdy_cb() { return m_iochrdy_cb.bind(); }

	int iochrdy_r() const { return m_ready_listener ? m_ready : machine().time() >= m_ready_until; } // high = ready (OTI p17)

	bool at_zero_wait_enabled() const { return BIT(m_ext[0x13], 3); }

	// Call once at the beginning of the CPU/DMA memory MC, not after its
	// data transfer. Its saved wait count remains inside that MC, so BUSREQ
	// cannot skip IOCHRDY. The board combines it with U15 Z180 memory
	// decoder's fixed floor; the Oak service timing is an approximation.
	unsigned memory_wait(offs_t address, bool write, attotime phi);

	void output_divider_w(int divide_by_two);

	void linear_address_w(int state) { m_linear_address = bool(state); } // AA=0, AB=1

	void monitor_type_w(int applecolor); // sense only; firmware chooses the mode

	void set_straps(u8 config1, u8 config2, u8 dip)
	{
		m_config1 = config1;
		m_config2 = config2;
		m_dip = dip;
	}

	// Optional observation taps; the serial EEPROM (93C46), child "eeprom",
	// owns DO and persistence.
	auto eeprom_di_cb() { return m_eeprom_di_cb.bind(); }

	auto eeprom_cs_cb() { return m_eeprom_cs_cb.bind(); }

	auto eeprom_clock_cb() { return m_eeprom_clock_cb.bind(); }

	u32 vclk_hz() const;

	static constexpr u32 mclk_hz() { return 40'000'000; }

	// Physical VSYNC (pin 34), including polarity and the ext03 test input.
	// The inherited vsync_cb is the CRTC interrupt output (CINT, pin 90).
	int vsync_r();
	int hsync_r();

	// SD7:0 is the Z180 data bus on Second Sight. The board need only
	// deliver changes while this callback requests scan-test observation.
	auto scan_test_cb() { return m_scan_test_cb.bind(); }
	void system_data_w(u8 data);

	screen_device &video_screen() { return *m_screen; }

	// Emulator/capture coordinates, never guest-readable beam registers.
	// Saved m_active_display is {left, top, width, height} in screen pixels.
	rectangle active_display() const { return rectangle(m_active_display[0], m_active_display[0] + m_active_display[2] - 1,
		m_active_display[1], m_active_display[1] + m_active_display[3] - 1); }
	int crtc_hpos() const;
	int crtc_vpos() const;
	attotime time_until_crtc_pos(int y, int x = 0) const;

	virtual u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, rectangle const &cliprect) override;

  protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void attribute_map(address_map &map) override ATTR_COLD;
	virtual void recompute_params() override;
	virtual u8 crtc_data_r(offs_t offset) override;
	virtual void crtc_data_w(offs_t offset, u8 data) override;
	virtual u32 latch_start_addr() override;

	virtual bool get_interlace_mode() override { return BIT(m_ext[0x14], 7); }

	virtual void palette_update() override;

  private:
	bool enabled() const;
	bool decode_memory(offs_t address, bool write, u32 &offset) const;
	u32 card_address(offs_t address, bool half) const;
	u8 extended_r();
	void extended_w(u8 data);
	u8 dac_r(u8 rs);
	void dac_w(u8 rs, u8 data);
	void access_complete();
	void memory_line(attotime now);
	void postload();
	void eeprom_update();
	void eeprom_do_w(int state);
	u8 csel_pins() const;
	bool vertical_retrace();
	void schedule_display_latch(unsigned timers = 3);
	void flush_display();
	int begin_row_change();
	void end_row_change(int beam);
	void schedule_character_map();
	void schedule_pan();
	struct row_state
	{
		u32 address;
		u8 scan;
		bool repeat;
		bool split;
	};
	row_state display_row(unsigned y) const;
	row_state untracked_row(unsigned y) const;
	void advance_row(row_state &row, unsigned line, unsigned field) const;
	static u64 pack_row(row_state const &row);
	static row_state unpack_row(u64 row);
	u32 display_address(u32 counter, unsigned row_scan) const;
	u32 odd_even_address(u32 address) const;
	unsigned vertical_divisor() const { return vga.crtc.sldiv ? 2 : 1; }
	unsigned serializer_period() const;
	unsigned display_skew() const;
	TIMER_CALLBACK_MEMBER(ready_tick);
	TIMER_CALLBACK_MEMBER(eeprom_ready_tick);
	TIMER_CALLBACK_MEMBER(display_latch_tick);
	TIMER_CALLBACK_MEMBER(display_end_tick);
	TIMER_CALLBACK_MEMBER(character_map_tick);
	TIMER_CALLBACK_MEMBER(pan_tick);

	bool packed() const { return (m_ext[0x21] & 0x0c) == 4; }

	u32 storage_address(u32 linear) const;
	u8 linear_byte(u32 address) const;
	void linear_byte_w(u32 address, u8 data);
	u32 chain4_address(u32 address) const;
	u8 planar_r(u32 address);
	u8 graphics_write(unsigned plane, u8 data, u8 latch) const;
	void planar_w(u32 address, u8 data);
	void packed_w(u32 address, u8 data);
	void latch_diagnostic(unsigned bit, char const *reason, u32 address);
	unsigned pixel_divisor() const;
	unsigned horizontal_multiplier() const;
	unsigned horizontal_blank_first() const;
	unsigned horizontal_blank_length() const;
	unsigned vertical_blank_first() const;
	unsigned vertical_blank_length() const;
	bool iigs_frame();
	bool vga_lcd() const;
	// Characters or lines of a CRTC retrace pulse. It starts where the counter
	// equals start and ends where the low bits (mask) of the counter equal
	// end; the counter wraps at total first (IBM VGA/XGA TRM pp. 2-60, 2-71).
	// 0 when the counter never matches end, so the pulse never ends.
	static int retrace_length(int start, int end, int mask, int total)
	{
		for (int n = 1; n < total; ++n)
			if ((((start + n) % total) & mask) == end)
				return n;
		return 0;
	}
	int connector_hsync();
	bool sync_present();
	bool lcd_measure(oti087_lcd::Timing &timing, int &hsync, int &vsync);
	std::array<s32, 11> lcd_shape(int hsync, int vsync);
	bool scan_test() const { return (m_ext[3] & 0x87) == 0x85; }
	void draw_scan_test(bitmap_rgb32 &bitmap, rectangle const &cliprect, int ox, int oy);
	void draw_packed(bitmap_rgb32 &bitmap, rectangle const &cliprect, int ox, int oy);
	template <bool ScanTest = false> void draw_text(bitmap_rgb32 &bitmap, rectangle const &cliprect, int ox, int oy);
	void draw_planar(bitmap_rgb32 &bitmap, rectangle const &cliprect, int ox, int oy);
	void draw_vga256(bitmap_rgb32 &bitmap, rectangle const &cliprect, int ox, int oy);
	void draw_graphics(bitmap_rgb32 &bitmap, rectangle const &cliprect, int ox, int oy);
	template <bool ScanTest = false> void draw_cursor(bitmap_rgb32 &bitmap, rectangle const &cliprect, int ox, int oy);
	u8 cursor_background_index(row_state const &row, int x) const;
	bool blanked(int x, int y) const;
	rgb_t attribute_color(u8 index) const;
	u32 draw_raster(bitmap_rgb32 &bitmap, rectangle const &cliprect, int ox, int oy);
	rectangle active_raster() const { return rectangle(0, m_active_display[2] - 1, 0, m_active_display[3] - 1); }

	required_device<screen_device> m_screen;
	required_device<bt481_device> m_ramdac;
	required_device<eeprom_serial_93c46_16bit_device> m_eeprom;
	required_ioport m_monitor;
	devcb_write_line m_iochrdy_cb;
	devcb_write_line m_scan_test_cb;
	devcb_write_line m_eeprom_di_cb, m_eeprom_cs_cb, m_eeprom_clock_cb;
	emu_timer *m_ready_timer = nullptr;
	attotime m_ready_until = attotime::zero;
	bool m_ready_listener = false;
	emu_timer *m_eeprom_ready_timer = nullptr;
	emu_timer *m_display_end_timer = nullptr;
	emu_timer *m_character_map_timer = nullptr;
	emu_timer *m_pan_timer = nullptr;
	u8 m_character_map[2]{};
	// Only live row-control changes need a history. Each entry packs the
	// address counter, five-bit row scan, double-scan phase and split latch.
	std::array<u64, 8200> m_row_history{};
	u64 m_row_history_frame = ~u64(0);
	u8 m_frame_preset = 0;
	u32 m_plane_address_mask = ~u32(0xc000);
	bool m_memory_active = false;
	oti087_memory_timing m_memory;
	attotime m_memory_epoch = attotime::zero;
	attotime m_memory_next_line = attotime::zero;
	std::array<u64, 8> m_memory_raster{};
	u8 m_ext[0x80]{};
	std::array<s32, 4> m_active_display{0, 0, 640, 480};
	std::array<s32, 2> m_display_origin{0, 0};
	s32 m_crtc_origin = 0;
	attotime m_origin_phase = attotime::zero;
	bool m_out_of_range = false, m_lock_candidate = true;
	attotime m_lock_since = attotime::zero;
	// The out-of-range message and when it was last shown.
	std::string m_monitor_message;
	attotime m_message_time = attotime::zero;
	std::array<double, 5> m_lock_rates{};
	// A mode that needs a back-panel control turned gets one short notice:
	// its line and field rates and which controls, 0 for no such mode.
	std::array<double, 5> m_hold_rates{};
	attotime m_hold_since = attotime::zero;
	bool m_hold_shown = false;
	// The VGA LCD monitor's controller, and whether it is attached with the
	// picture shape of its last Auto. Both follow from the registers.
	oti087_lcd::Monitor m_lcd;
	bool m_lcd_attached = false;
	std::array<s32, 11> m_lcd_shape{};
	u8 m_system_data = 0xff;
	u8 m_packed_layout_diagnostics = 0;
	u8 m_latch[8]{};
	u8 m_latch_diagnostics = 0;
	u8 m_index = 0, m_setup = 0, m_enable = 0;
	u8 m_config1 = 0x56, m_config2 = 0x01, m_dip = 0xff;
	bool m_ready = true, m_output_divider = false, m_linear_address = false;
	int m_monitor_override = -1;
};

DECLARE_DEVICE_TYPE(OTI087, oak_oti087_vga_device)

#endif // MAME_VIDEO_PC_VGA_OTI087_H

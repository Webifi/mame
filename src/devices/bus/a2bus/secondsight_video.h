// license:BSD-3-Clause
#ifndef MAME_BUS_A2BUS_SECONDSIGHT_VIDEO_H
#define MAME_BUS_A2BUS_SECONDSIGHT_VIDEO_H
#pragma once

#include "video/pc_vga_oti087.h"

// Small board/chip adapter. Addresses are already resolved by the board:
// LA20=linear&HA19, LA19=half&HA19; only the HA19-high video window reaches
// this interface. Lower address bits come from the Z180. Full I/O
// ports include $3xx and the separate $0102/$46E8 add-on enable strobes.
class secondsight_video_device : public device_t
{
  public:
	secondsight_video_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock = 0);

	u8 io_r(offs_t port, bool *driven = nullptr) { return m_oak->io_r(port, driven); }

	void io_w(offs_t port, u8 data) { m_oak->io_w(port, data); }

	u8 memory_r(offs_t address, bool *driven = nullptr);
	void memory_w(offs_t address, u8 data);

	void output_divider_w(bool divided) { m_oak->output_divider_w(divided); }
	void system_data_w(u8 data) { m_oak->system_data_w(data); }
	auto scan_test_cb() { return m_scan_test_cb.bind(); }

	// The VGA connector and U12 AppleColor RGB timing GAL share these
	// physical sync pins, including ext03's VSYNC toggle, not the CINT IRQ.
	int vsync_r() { return m_oak->vsync_r(); }
	int hsync_r() { return m_oak->hsync_r(); }

	auto iochrdy_cb() { return m_iochrdy_cb.bind(); }

	auto cint_cb() { return m_cint_cb.bind(); }

	unsigned memory_wait(offs_t address, bool write, attotime phi) { return m_oak->memory_wait(address, write, phi); }

  protected:
	virtual void device_config_complete() override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

  private:
	required_device<oak_oti087_vga_device> m_oak;
	devcb_write_line m_iochrdy_cb;
	devcb_write_line m_cint_cb;
	devcb_write_line m_scan_test_cb;
	u32 m_last_address = 0; // diagnostic observation at the board/chip boundary
};
DECLARE_DEVICE_TYPE(SECONDSIGHT_VIDEO, secondsight_video_device)
#endif

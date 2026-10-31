// license:BSD-3-Clause
#include "emu.h"
#include "secondsight_video.h"

DEFINE_DEVICE_TYPE(SECONDSIGHT_VIDEO, secondsight_video_device, "ssvideo", "Second Sight video interface")

secondsight_video_device::secondsight_video_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, SECONDSIGHT_VIDEO, tag, owner, clock)
	, m_oak(*this, "oak")
	, m_iochrdy_cb(*this)
	, m_cint_cb(*this)
	, m_scan_test_cb(*this)
{
}

void secondsight_video_device::device_add_mconfig(machine_config &config)
{
	OTI087(config, m_oak);
	m_oak->vsync_cb().set([this](int state) { m_cint_cb(state); });
	m_oak->scan_test_cb().set([this](int state) { m_scan_test_cb(state); });
}

void secondsight_video_device::device_config_complete()
{
	// The board queries IOCHRDY at the start of each MC. Do not turn an unconnected
	// output into a listener that schedules a machine-wide timer per byte.
	if (!m_iochrdy_cb.isunset())
		m_oak->iochrdy_cb().set([this](int state) { m_iochrdy_cb(state); });
}

void secondsight_video_device::device_start()
{
	save_item(NAME(m_last_address));
}

u8 secondsight_video_device::memory_r(offs_t address, bool *driven)
{
	if (!machine().side_effects_disabled())
		m_last_address = address;
	return m_oak->memory_r(address, driven);
}

void secondsight_video_device::memory_w(offs_t address, u8 data)
{
	m_last_address = address;
	m_oak->memory_w(address, data);
}

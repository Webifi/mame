// license:BSD-3-Clause
// copyright-holders:R. Belmont
#ifndef MAME_SOUND_ES5503_H
#define MAME_SOUND_ES5503_H

#pragma once

#include "dirom.h"
#include "dac_filter.h"

class es5503_device : public device_t,
		public device_sound_interface,
		public device_rom_interface<17>
{
public:
	es5503_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);
	void set_channels(int channels) { m_output_channels = channels; }
	// Optional external mono reconstruction network, before output sampling.
	void set_output_filter(const dac_filter::modes &poles, const dac_filter::modes &gains, unsigned count = 3)
	{
		m_filter_poles = poles;
		m_filter_gains = gains;
		m_filter_mode_count = count;
		m_filter_enabled = true;
	}
	void set_output_filter_enabled(bool enabled) { m_filter_enabled = enabled && m_filter_mode_count; }
	// Select the optional fitted WVREF diode model; false uses linear volume.
	void set_volume_knee(bool knee) { m_volume_knee = knee; }
	auto irq_func() { return m_irq_func.bind(); }
	auto adc_func() { return m_adc_func.bind(); }
	// Complete externally scheduled host transfers before advancing the chip.
	void set_host_sync(std::function<void (attotime)> sync) { m_host_sync = std::move(sync); }
	void synchronize_to(attotime time);
	u8 read_at(offs_t offset, attotime time);
	void write_at(offs_t offset, u8 data, attotime time);
	u8 read(offs_t offset);
	void write(offs_t offset, u8 data);
	uint8_t get_channel_strobe() { return m_channel_strobe; }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_clock_changed() override;
	virtual void sound_stream_update(sound_stream &stream) override;
	virtual void rom_bank_pre_change() override;

private:
	enum : uint8_t { FREE, ONESHOT, SYNCAM, SWAP };
	enum : uint8_t { ADDRESS, CHANNEL, SAMPLE };
	enum : uint8_t { HOST_NONE, HOST_WRITE, HOST_ACK, HOST_ADC };
	struct oscillator
	{
		uint16_t freq = 0;
		uint8_t control = 1;
		uint8_t vol = 0;
		uint8_t data = 0x80;
		uint32_t wavetblpointer = 0;
		uint8_t wavetblsize = 0;
		uint8_t resolution = 0;
		uint32_t accumulator = 0;
		uint8_t irqpend = 0;
	};
	oscillator m_oscillators[32];

	std::function<void (attotime)> m_host_sync;
	sound_stream *m_stream = nullptr;
	emu_timer *m_timer = nullptr;
	devcb_write_line m_irq_func;
	devcb_read8 m_adc_func;
	int m_output_channels = 1;
	uint32_t m_input_clock = 0;
	uint8_t m_oscsenabled = 1;
	uint8_t m_scan_enabled = 1;
	uint8_t m_slot = 0;
	uint8_t m_phase = ADDRESS;
	uint64_t m_tick = 0;
	uint64_t m_next_tick = 4;

	// The address and oscillator controls are latched for the E-low phase.
	uint32_t m_fetch_address = 0;
	uint32_t m_fetch_accumulator = 0;
	uint8_t m_fetch_control = 1;
	uint8_t m_fetch_volume = 0;
	bool m_fetch_active = false;
	bool m_fetch_carry = false;
	uint8_t m_channel_strobe = 15;
	bool m_cstrb = false;

	uint8_t m_host_operation = HOST_NONE;
	uint8_t m_host_address = 0;
	uint8_t m_host_data = 0;
	uint8_t m_irq_queue[32]{};
	uint8_t m_irq_count = 0;
	uint8_t m_irq_last = 0xff;
	bool m_irq_level = false;
	uint8_t m_adc_result = 0;
	uint64_t m_adc_tick = 0;

	int32_t m_dac_sample = 0;
	uint8_t m_dac_channel = 0;
	int64_t m_audio_sum[16]{};
	uint64_t m_audio_clocks = 0;
	// Like the sound core's output buffers, retain one second of samples.
	// A host bus transfer can advance the chip ahead of the stream's time.
	static constexpr unsigned AUDIO_RATE = 48000;
	std::vector<float> m_audio_buffer;
	uint64_t m_audio_index = 0;
	uint64_t m_audio_end = 0;
	dac_filter m_output_filter;
	dac_filter::modes m_filter_poles{}, m_filter_gains{};
	unsigned m_filter_mode_count = 0;
	bool m_filter_enabled = false;
	double m_filter_state[6]{};
	double m_filter_sum = 0;
	uint64_t m_filter_tick = 0;
	int32_t m_filter_sample = 0;

	// Effective volume of each register value, with VOLUME_BITS fraction bits.
	// Index 0 is the linear data sheet law; index 1 is the optional diode fit
	// described by fill_knee_gain(), not a calibration for every IIgs board.
	static constexpr unsigned VOLUME_BITS = 14;
	static constexpr double VOLUME_UNIT = 1 << VOLUME_BITS;
	int32_t m_volume_gain[2][256]{};
	bool m_volume_knee = false;

	TIMER_CALLBACK_MEMBER(wakeup);
	void synchronize();
	void advance_to(attotime time);
	void integrate_to(uint64_t tick);
	void filter_to(uint64_t tick);
	void address_phase();
	void sample_phase();
	void complete_oscillator(uint8_t osc, bool zero);
	void schedule_wakeup();
	void queue_irq(uint8_t osc);
	void update_irq();
	u8 register_read(uint8_t address) const;
	void register_write(uint8_t address, uint8_t data);
	void commit_host();
};

DECLARE_DEVICE_TYPE(ES5503, es5503_device)

#endif // MAME_SOUND_ES5503_H

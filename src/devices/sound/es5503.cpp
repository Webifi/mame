// license:BSD-3-Clause
// copyright-holders:R. Belmont
/*
 * Ensoniq ES5503 Digital Oscillator Chip
 *
 * References: Ensoniq 5503 specification pp. 9-16, 19, 22; ICS1261
 * specification pp. 2-5, 11-12; Apple DOC ERS (25 June 1986) pp. 2, 4,
 * 6-9; Apple IIgs Technical Note 11; Cortland Sound ERS pp. 5-6.
 *
 * One oscillator occupies eight input clocks. E-high is the host phase;
 * E-low presents a waveform address, with data sampled at E rising.
 * CA changes halfway through E-low. Two refresh cycles follow the last
 * enabled oscillator. Register accesses and audio share this clock line.
 *
 * The register API represents a host access during E-high; writes and
 * read side effects are latched at the following E falling edge. It does
 * not expose CS/WE pins or insert wait states into the calling processor.
 * A caller making an E-low access must supply its own bus synchronization.
 *
 * The sheets do not define internal same-edge arbitration, the reset
 * latency of a CPU halt, duplicate interrupt depth, or the table-end
 * detector when resolution changes with a nonzero accumulator. Choices
 * for those cases are identified below, rather than treated as pin specs.
 */

#include "emu.h"
#include "es5503.h"

DEFINE_DEVICE_TYPE(ES5503, es5503_device, "es5503", "Ensoniq ES5503")

namespace {

// Optional model of the IIgs WVREF path: volume op-amp output V = 5 V * D / 256
// drives a 1N914 in series with 1 kOhm to ground. The modeled waveform-DAC
// reference is the resistor voltage, so small D lose more than large D.
// Is and n are fitted to recordings of one IIgs, not measured part parameters
// or a calibration applicable to every board. Rs is zero
// and Vt is kT/q at 300 K. gain[D] is the volume that a linear law needs for the
// same output, 256 * WVREF / 5 V, times unit. The linear law gives D * unit.
void fill_knee_gain(int32_t (&gain)[256], double unit)
{
	constexpr double IS = 2.5e-9, N = 1.75, VT = 0.025852, R = 1000.0, VFULL = 5.0;
	gain[0] = 0;
	for (int d = 1; d < 256; ++d)
	{
		// I * R + N * VT * ln(1 + I / IS) = V rises with I, so bisect for the current.
		const double v = VFULL * d / 256;
		double low = 0.0, high = v / R;
		for (int step = 0; step < 64; ++step)
		{
			const double mid = (low + high) / 2;
			if (mid * R + N * VT * std::log1p(mid / IS) > v)
				high = mid;
			else
				low = mid;
		}
		gain[d] = int32_t(std::lround(256 * ((low + high) / 2) * R / VFULL * unit));
	}
}

} // anonymous namespace

es5503_device::es5503_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, ES5503, tag, owner, clock),
	device_sound_interface(mconfig, *this),
	device_rom_interface(mconfig, *this),
	m_irq_func(*this),
	m_adc_func(*this, 0)
{
}

void es5503_device::device_start()
{
	assert(m_output_channels > 0 && m_output_channels <= 16 && !(m_output_channels & (m_output_channels - 1)));
	m_input_clock = clock();
	// Audio reconstruction is independent of the oscillator-enable register.
	m_stream = stream_alloc(0, m_output_channels, AUDIO_RATE);
	m_audio_buffer.resize(AUDIO_RATE * m_output_channels);
	if (m_filter_mode_count)
		m_output_filter.configure(m_input_clock, AUDIO_RATE, m_filter_poles, m_filter_gains, m_filter_mode_count);
	m_timer = timer_alloc(FUNC(es5503_device::wakeup), this);
	for (int d = 0; d < 256; ++d)
		m_volume_gain[0][d] = d << VOLUME_BITS;
	fill_knee_gain(m_volume_gain[1], VOLUME_UNIT);

	save_pointer(STRUCT_MEMBER(m_oscillators, freq), 32);
	save_pointer(STRUCT_MEMBER(m_oscillators, control), 32);
	save_pointer(STRUCT_MEMBER(m_oscillators, vol), 32);
	save_pointer(STRUCT_MEMBER(m_oscillators, data), 32);
	save_pointer(STRUCT_MEMBER(m_oscillators, wavetblpointer), 32);
	save_pointer(STRUCT_MEMBER(m_oscillators, wavetblsize), 32);
	save_pointer(STRUCT_MEMBER(m_oscillators, resolution), 32);
	save_pointer(STRUCT_MEMBER(m_oscillators, accumulator), 32);
	save_pointer(STRUCT_MEMBER(m_oscillators, irqpend), 32);
	save_item(NAME(m_input_clock));
	save_item(NAME(m_oscsenabled));
	save_item(NAME(m_scan_enabled));
	save_item(NAME(m_slot));
	save_item(NAME(m_phase));
	save_item(NAME(m_tick));
	save_item(NAME(m_next_tick));
	save_item(NAME(m_fetch_address));
	save_item(NAME(m_fetch_accumulator));
	save_item(NAME(m_fetch_control));
	save_item(NAME(m_fetch_volume));
	save_item(NAME(m_fetch_active));
	save_item(NAME(m_fetch_carry));
	save_item(NAME(m_channel_strobe));
	save_item(NAME(m_cstrb));
	save_item(NAME(m_host_operation));
	save_item(NAME(m_host_address));
	save_item(NAME(m_host_data));
	save_item(NAME(m_irq_queue));
	save_item(NAME(m_irq_count));
	save_item(NAME(m_irq_last));
	save_item(NAME(m_irq_level));
	save_item(NAME(m_adc_result));
	save_item(NAME(m_adc_tick));
	save_item(NAME(m_dac_sample));
	save_item(NAME(m_dac_channel));
	save_item(NAME(m_audio_sum));
	save_item(NAME(m_audio_clocks));
	save_item(NAME(m_audio_buffer));
	save_item(NAME(m_audio_index));
	save_item(NAME(m_audio_end));
	save_item(NAME(m_filter_enabled));
	save_item(NAME(m_filter_state));
	save_item(NAME(m_filter_sum));
	save_item(NAME(m_filter_tick));
	save_item(NAME(m_filter_sample));
}

void es5503_device::device_reset()
{
	// Dynamic-register decay on a held reset is documented, but its time
	// constant is not. This API reset uses a deterministic quiet initial
	// state; it is not an emulation of a timed assertion of the RES pin.
	for (auto &o : m_oscillators)
		o = oscillator{};
	m_input_clock = clock();
	m_tick = machine().time().as_ticks(m_input_clock);
	m_next_tick = m_tick + 4;
	m_slot = 0;
	m_phase = ADDRESS;
	m_oscsenabled = m_scan_enabled = 1;
	m_fetch_active = m_fetch_carry = false;
	m_fetch_address = m_fetch_accumulator = 0;
	m_fetch_control = 1;
	m_fetch_volume = 0;
	m_channel_strobe = 15;
	m_cstrb = false;
	m_host_operation = HOST_NONE;
	m_host_address = m_host_data = 0;
	std::fill_n(m_irq_queue, 32, 0);
	m_irq_count = 0;
	m_irq_last = 0xff;
	m_irq_level = false;
	m_irq_func(0);
	m_adc_tick = 0;
	m_adc_result = 0;
	m_dac_sample = 0;
	m_dac_channel = 0;
	std::fill_n(m_audio_sum, 16, 0);
	m_audio_clocks = 0;
	m_audio_index = m_stream->start_index();
	m_audio_end = m_stream->sample_to_time(m_audio_index).as_ticks(m_input_clock);
	std::fill_n(m_filter_state, 6, 0);
	m_filter_sum = 0;
	m_filter_tick = m_tick;
	m_filter_sample = 0;
	schedule_wakeup();
}

void es5503_device::device_clock_changed()
{
	if (!m_stream)
		return;
	synchronize();
	if (m_filter_enabled)
		filter_to(m_tick);
	const uint64_t left = m_next_tick - m_tick;
	const uint64_t adc_left = m_adc_tick ? m_adc_tick - m_tick : 0;
	m_input_clock = clock();
	m_tick = machine().time().as_ticks(m_input_clock);
	m_next_tick = m_tick + left;
	m_adc_tick = adc_left ? m_tick + adc_left : 0;
	m_audio_end = m_stream->sample_to_time(m_audio_index).as_ticks(m_input_clock);
	m_filter_tick = m_tick;
	if (m_filter_mode_count)
		m_output_filter.configure(m_input_clock, AUDIO_RATE, m_filter_poles, m_filter_gains, m_filter_mode_count);
	schedule_wakeup();
}

void es5503_device::synchronize()
{
	m_stream->update();
	advance_to(machine().time());
}

void es5503_device::synchronize_to(attotime time)
{
	// Keep the normal stream flush when it cannot pass the host event.
	// A late event is drained by advance_to before the stream catches up.
	if (machine().time() <= time)
		m_stream->update();
	assert(time.as_ticks(m_input_clock) >= m_tick);
	advance_to(time);
}

void es5503_device::rom_bank_pre_change()
{
	synchronize();
	schedule_wakeup();
}

TIMER_CALLBACK_MEMBER(es5503_device::wakeup)
{
	synchronize();
	schedule_wakeup();
}

void es5503_device::filter_to(uint64_t tick)
{
	if (tick > m_filter_tick)
	{
		m_filter_sum += m_output_filter.step(m_filter_state, m_filter_sample, tick - m_filter_tick);
		m_filter_tick = tick;
	}
}

void es5503_device::integrate_to(uint64_t tick)
{
	// The external circuit sees the single multiplexed DAC. Accumulate an
	// unchanged hold lazily; only a new value or output boundary needs work.
	if (m_filter_enabled && m_filter_sample != m_dac_sample)
	{
		filter_to(m_tick);
		m_filter_sample = m_dac_sample;
	}
	// Split the DAC hold at output boundaries even when a future bus event
	// advances the chip before the stream consumes these samples. Averaging
	// only in sound_stream_update would merge them and then emit zeroes.
	while (m_audio_end <= tick)
	{
		assert(m_audio_index < m_stream->start_index() + AUDIO_RATE);
		// A reset may discard the fraction of a sample before the reset edge.
		const uint64_t clocks = m_audio_end > m_tick ? m_audio_end - m_tick : 0;
		if (m_filter_enabled)
			filter_to(m_audio_end);
		else
			m_audio_sum[m_dac_channel] += int64_t(m_dac_sample) * int64_t(clocks);
		m_audio_clocks += clocks;
		float *const output = &m_audio_buffer[(m_audio_index % AUDIO_RATE) * m_output_channels];
		for (int channel = 0; channel < m_output_channels; ++channel)
		{
			// Preserve the established 32-oscillator calibration (34 / 8).
			// The samples carry VOLUME_BITS fraction bits of volume.
			const double sum = m_filter_enabled ? (channel ? 0 : m_filter_sum) : double(m_audio_sum[channel]);
			output[channel] = m_audio_clocks ? sum * 17.0 / (4.0 * 32768.0 * VOLUME_UNIT * m_audio_clocks) : 0;
			m_audio_sum[channel] = 0;
		}
		m_audio_clocks = 0;
		m_filter_sum = 0;
		m_tick = std::max(m_tick, m_audio_end);
		++m_audio_index;
		// This is sample_to_time(index).as_ticks(clock), without attotime
		// arithmetic. The stream rounds sample times up to an attosecond.
		m_audio_end = (m_audio_index / AUDIO_RATE) * m_input_clock +
			((m_audio_index % AUDIO_RATE) * m_input_clock) / AUDIO_RATE;
	}
	const uint64_t clocks = tick - m_tick;
	if (!m_filter_enabled)
		m_audio_sum[m_dac_channel] += int64_t(m_dac_sample) * int64_t(clocks);
	m_audio_clocks += clocks;
	m_tick = tick;
}

void es5503_device::advance_to(attotime time)
{
	// A timer can be dispatched after a CPU instruction has crossed its
	// deadline. Drain the host interface first, using the original event
	// times, so audio and slot timers cannot overtake those transfers.
	if (m_host_sync)
		m_host_sync(time);
	const uint64_t target = time.as_ticks(m_input_clock);
	if (target <= m_tick)
		return;
	while (m_next_tick <= target || (m_adc_tick && m_adc_tick <= target))
	{
		if (m_adc_tick && m_adc_tick <= m_next_tick)
		{
			integrate_to(m_adc_tick);
			m_adc_tick = 0;
			m_adc_result = m_adc_func();
			continue;
		}
		// Complete an uninterrupted oscillator slot in one iteration. Nothing
		// can observe the channel midpoint before target, and the DAC holds
		// the same integer value across both halves of E-low.
		if (m_phase == ADDRESS && m_next_tick + 4 <= target &&
			(!m_adc_tick || m_adc_tick > m_next_tick + 4))
		{
			if (m_host_operation == HOST_NONE &&
				m_slot < m_scan_enabled && (m_oscillators[m_slot].control & 1))
			{
				// A halted oscillator performs no waveform fetch. Its service
				// still resets M0+H and contributes center level on its channel.
				// Integrate the preceding DAC hold through this sample edge.
				oscillator &o = m_oscillators[m_slot];
				integrate_to(m_next_tick + 4);
				m_fetch_control = o.control;
				m_fetch_volume = o.vol;
				m_fetch_active = false;
				if ((o.control & 3) == 3)
					o.accumulator = 0;
				m_cstrb = true;
				m_channel_strobe = o.control >> 4;
				m_dac_channel = m_channel_strobe & (m_output_channels - 1);
				m_dac_sample = 0;
				++m_slot; // refresh slots follow, so this cannot wrap the scan
				m_next_tick += 8;
				continue;
			}
			integrate_to(m_next_tick);
			m_cstrb = false;
			commit_host();
			address_phase();
			m_phase = CHANNEL;
			m_channel_strobe = (m_slot < m_scan_enabled) ? m_fetch_control >> 4 : 15;
			integrate_to(m_next_tick + 4);
			m_phase = SAMPLE;
			sample_phase();
			if (++m_slot == m_scan_enabled + 2)
			{
				m_slot = 0;
				m_scan_enabled = m_oscsenabled;
			}
			m_phase = ADDRESS;
			m_next_tick += 8;
			continue;
		}
		integrate_to(m_next_tick);
		switch (m_phase)
		{
		case ADDRESS:
			m_cstrb = false;
			commit_host();
			address_phase();
			m_phase = CHANNEL;
			m_next_tick += 2;
			break;
		case CHANNEL:
			m_channel_strobe = (m_slot < m_scan_enabled) ? m_fetch_control >> 4 : 15;
			m_phase = SAMPLE;
			m_next_tick += 2;
			break;
		case SAMPLE:
			sample_phase();
			if (++m_slot == m_scan_enabled + 2)
			{
				m_slot = 0;
				// Mid-scan enable changes are unspecified. Keep both refresh
				// slots, and apply the new limit to the following scan.
				m_scan_enabled = m_oscsenabled;
			}
			m_phase = ADDRESS;
			m_next_tick += 4;
			break;
		}
	}
	integrate_to(target);
}

void es5503_device::address_phase()
{
	m_fetch_active = false;
	if (m_slot >= m_scan_enabled)
		return;
	oscillator &o = m_oscillators[m_slot];
	m_fetch_control = o.control;
	m_fetch_volume = o.vol;
	// CPU halt reset is performed at this oscillator's own service. The
	// documentation gives M0*H, but not the CPU-write-to-reset latency.
	if ((o.control & 3) == 3)
		o.accumulator = 0;
	if (o.control & 1)
	{
		return;
	}

	m_fetch_active = true;
	const uint32_t index_mask = (1U << (8 + o.wavetblsize)) - 1;
	const unsigned shift = 9 + o.resolution - o.wavetblsize;
	const uint32_t cycle_mask = (1U << (17 + o.resolution)) - 1;
	// Carry out of the selected accumulator window marks a cycle. The
	// sheets do not specify a level test of previously set upper bits.
	m_fetch_carry = ((o.accumulator & cycle_mask) + o.freq) > cycle_mask;
	m_fetch_accumulator = (o.accumulator + o.freq) & 0xffffff;
	m_fetch_address = (o.wavetblpointer & ~index_mask)
		| ((m_fetch_accumulator >> shift) & index_mask);
}

void es5503_device::sample_phase()
{
	if (m_slot >= m_scan_enabled)
	{
		// The last DAC value is sustained through both refresh slots.
		// CA and CSTRB are inactive, independently of that analog hold.
		m_channel_strobe = 15;
		m_cstrb = false;
		return;
	}
	m_cstrb = true;
	m_dac_channel = (m_fetch_control >> 4) & (m_output_channels - 1);
	m_dac_sample = 0;
	if (!m_fetch_active)
	{
		return;
	}
	oscillator &o = m_oscillators[m_slot];
	o.accumulator = m_fetch_accumulator;
	o.data = read_byte(m_fetch_address);
	const uint8_t mode = (m_fetch_control >> 1) & 3;
	if (!o.data || m_fetch_carry)
		complete_oscillator(m_slot, !o.data);
	// A halted voice contributes center level. In particular, do not play
	// the wrapped first sample after a one-shot has reached its end.
	if (!o.data || (o.control & 1))
	{
		return;
	}
	if (mode == SYNCAM && (m_slot & 1))
	{
		return;
	}
	uint8_t volume = m_fetch_volume;
	if (m_slot && !(m_slot & 1) && ((m_oscillators[m_slot - 1].control >> 1) & 3) == SYNCAM)
		volume = m_oscillators[m_slot - 1].data;
	m_dac_sample = (int(o.data) - 128) * m_volume_gain[m_volume_knee][volume];
}

void es5503_device::complete_oscillator(uint8_t osc, bool zero)
{
	oscillator &o = m_oscillators[osc];
	const uint8_t mode = (m_fetch_control >> 1) & 3;
	if (mode == SYNCAM && !(osc & 1))
	{
		o.accumulator = 0;
		m_oscillators[osc + 1].accumulator = 0;
	}
	if (zero || mode == ONESHOT || mode == SWAP)
		o.control |= 1;
	if ((o.control & 3) == 3)
		o.accumulator = 0;
	if (mode == SWAP)
		m_oscillators[osc ^ 1].control &= ~1;
	// Completion is remembered even while interrupt delivery is masked.
	o.irqpend |= 1;
	queue_irq(osc);
}

void es5503_device::queue_irq(uint8_t osc)
{
	oscillator &o = m_oscillators[osc];
	// The FIFO order is explicit in Apple's ERS. One pending entry per
	// oscillator is a modeling choice; repeated-event depth is unspecified.
	if ((o.control & 8) && (o.irqpend & 1) && !(o.irqpend & 2))
	{
		assert(m_irq_count < 32);
		m_irq_queue[m_irq_count++] = osc;
		o.irqpend |= 2;
		update_irq();
	}
}

void es5503_device::update_irq()
{
	const bool level = m_irq_count != 0;
	if (level != m_irq_level)
	{
		m_irq_level = level;
		m_irq_func(level);
	}
}

u8 es5503_device::register_read(uint8_t address) const
{
	if (address < 0xe0)
	{
		const oscillator &o = m_oscillators[address & 31];
		switch (address & 0xe0)
		{
		case 0x00: return o.freq & 0xff;
		case 0x20: return o.freq >> 8;
		case 0x40: return o.vol;
		case 0x60: return o.data;
		case 0x80: return (o.wavetblpointer >> 8) & 0xff;
		case 0xa0: return o.control;
		case 0xc0: return 0x80 | ((o.wavetblpointer >> 10) & 0x40) | (o.wavetblsize << 3) | o.resolution;
		}
	}
	switch (address)
	{
	case 0xe0: return m_irq_count ? (m_irq_queue[0] << 1) | 0x41 : m_irq_last | 0x80;
	case 0xe1: return ((m_oscsenabled - 1) << 1) | 0xc1;
	case 0xe2: return m_adc_result;
	default: return 0xff;
	}
}

void es5503_device::register_write(uint8_t address, uint8_t data)
{
	if (address < 0xe0)
	{
		const uint8_t osc = address & 31;
		oscillator &o = m_oscillators[osc];
		switch (address & 0xe0)
		{
		case 0x00: o.freq = (o.freq & 0xff00) | data; break;
		case 0x20: o.freq = (o.freq & 0x00ff) | (data << 8); break;
		case 0x40: o.vol = data; break;
		case 0x60: break; // current sample is read-only
		case 0x80: o.wavetblpointer = (o.wavetblpointer & 0x10000) | (data << 8); break;
		case 0xa0:
			o.control = data;
			queue_irq(osc);
			break;
		case 0xc0:
			o.wavetblpointer = (o.wavetblpointer & 0xffff) | ((data & 0x40) << 10);
			o.wavetblsize = (data >> 3) & 7;
			o.resolution = data & 7;
			break;
		}
	}
	else if (address == 0xe1)
		m_oscsenabled = ((data >> 1) & 31) + 1;
}

void es5503_device::commit_host()
{
	switch (m_host_operation)
	{
	case HOST_WRITE:
		register_write(m_host_address, m_host_data);
		break;
	case HOST_ACK:
		if (m_irq_count && m_irq_queue[0] == m_host_data)
		{
			m_irq_last = (m_host_data << 1) | 0x41;
			m_oscillators[m_host_data].irqpend = 0;
			--m_irq_count;
			std::move(m_irq_queue + 1, m_irq_queue + 1 + m_irq_count, m_irq_queue);
			update_irq();
		}
		break;
	case HOST_ADC:
		m_adc_tick = m_tick + 26 * 8;
		break;
	}
	m_host_operation = HOST_NONE;
}

u8 es5503_device::read(offs_t offset)
{
	if (machine().side_effects_disabled())
		return register_read(offset);
	synchronize();
	return read_at(offset, machine().time());
}

u8 es5503_device::read_at(offs_t offset, attotime time)
{
	synchronize_to(time);
	const uint8_t value = register_read(offset);
	if ((offset & 0xff) == 0xe0 && m_irq_count)
	{
		m_host_operation = HOST_ACK;
		m_host_data = m_irq_queue[0];
	}
	else if ((offset & 0xff) == 0xe2)
		m_host_operation = HOST_ADC;
	schedule_wakeup();
	return value;
}

void es5503_device::write(offs_t offset, u8 data)
{
	synchronize();
	write_at(offset, data, machine().time());
}

void es5503_device::write_at(offs_t offset, u8 data, attotime time)
{
	synchronize_to(time);
	m_host_operation = HOST_WRITE;
	m_host_address = offset;
	m_host_data = data;
	schedule_wakeup();
}

void es5503_device::schedule_wakeup()
{
	if (!m_input_clock)
	{
		m_timer->adjust(attotime::never);
		return;
	}
	uint64_t due = m_adc_tick ? m_adc_tick : ~uint64_t(0);
	if (m_host_operation != HOST_NONE)
	{
		const uint64_t host = m_next_tick + (m_phase == CHANNEL ? 6 : m_phase == SAMPLE ? 4 : 0);
		due = std::min(due, host);
	}
	const uint64_t sample = m_next_tick + (m_phase == ADDRESS ? 4 : m_phase == CHANNEL ? 2 : 0);
	const int slots = m_scan_enabled + 2;
	for (int osc = 0; osc < m_scan_enabled; ++osc)
		if ((m_oscillators[osc].control & 9) == 8 ||
			(osc == m_slot && m_phase != ADDRESS && m_fetch_active && (m_fetch_control & 8)))
			due = std::min(due, sample + ((osc + slots - m_slot) % slots) * 8);
	if (m_scan_enabled != m_oscsenabled)
		due = std::min(due, sample + (slots - m_slot - 1) * 8);
	if (due == ~uint64_t(0))
		m_timer->adjust(attotime::never);
	else
	{
		// Avoid rounding a clock edge backwards when converting to attotime.
		const attotime when = attotime::from_ticks(due, m_input_clock) + attotime::from_nsec(1);
		m_timer->adjust(std::max(attotime::zero, when - machine().time()));
	}
}

void es5503_device::sound_stream_update(sound_stream &stream)
{
	for (int sample = 0; sample < stream.samples(); ++sample)
	{
		const uint64_t index = stream.start_index() + sample;
		advance_to(stream.sample_to_time(index));
		integrate_to(m_tick); // also materialize the initial, zero-length sample
		assert(index < m_audio_index && m_audio_index - index <= AUDIO_RATE);
		const float *const output = &m_audio_buffer[(index % AUDIO_RATE) * m_output_channels];
		for (int channel = 0; channel < m_output_channels; ++channel)
			stream.put(channel, sample, output[channel]);
	}
}

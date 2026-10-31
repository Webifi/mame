// license:BSD-3-Clause
// Documented AppleSqueezer interfaces, independent of MAME scheduling.
// AppleSqueezer GS API pp. 1, 6-7; designer correspondence, 2026-10-04.
// Queue service uses the host IIgs bus model; CPU-local memory has no cache misses.
// Those timing choices are approximations, not recovered FPGA logic.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace applesqueezer
{

constexpr uint32_t presence_register = 0xe2000a;
constexpr uint32_t version_register = 0xe2000c;
constexpr uint32_t dram_main_bank_register = 0xe30000;
constexpr uint32_t dram_low_register = 0xe30002;
constexpr uint32_t dram_high_register = 0xe30004;
constexpr uint32_t dram_bank_register = 0xe30006;
constexpr uint32_t dram_data_register = 0xe30008;
constexpr uint32_t speed_register = 0xe50000;
constexpr uint32_t extended_ram_bytes = 0x0f000000;

// These are approximate performance labels, not oscillator periods. The API
// permits other values but does not publish their transfer function.
struct speed_option
{
	uint8_t value;
	unsigned nominal_mhz;
};

constexpr std::array<speed_option, 5> speed_options{{{255, 14}, {251, 10}, {247, 8}, {239, 5}, {223, 3}}};

constexpr unsigned nominal_speed(uint8_t value)
{
	for (auto option : speed_options)
		if (option.value == value)
			return option.nominal_mhz;
	return 0; // unspecified; callers must not turn this into a zero-Hz clock
}

// The memory owner supplies storage. Readback of the address latches is not
// documented, so this class offers only their published write interface.
struct dram_port
{
	uint8_t main_bank = 0;
	uint32_t offset = 0;
	bool at_bank_end = false;

	bool set_address_byte(uint32_t reg, uint8_t value)
	{
		switch (reg)
		{
		case dram_main_bank_register:
			main_bank = value;
			break;
		case dram_low_register:
			offset = (offset & 0xffff00) | value;
			break;
		case dram_high_register:
			offset = (offset & 0xff00ff) | (uint32_t(value) << 8);
			break;
		case dram_bank_register:
			offset = (offset & 0x00ffff) | (uint32_t(value) << 16);
			break;
		default:
			return false;
		}
		at_bank_end = false;
		return true;
	}

	bool address(uint32_t &out) const
	{
		if (main_bank > 0x0e || at_bank_end)
			return false;
		out = (uint32_t(main_bank) << 24) | offset;
		return true;
	}

	void advance()
	{
		// The public equates promise increment "to end of main bank". The
		// next access after its final byte has no established wrap/carry rule.
		// Reject further transfers until software writes an address latch.
		if (offset == 0xffffff)
			at_bank_end = true;
		else
			++offset;
	}

	template <class Reader> bool read(uint8_t &value, Reader &&reader, bool side_effects = true)
	{
		uint32_t a;
		if (!address(a))
			return false;
		value = reader(a);
		if (side_effects)
			advance();
		return true;
	}

	template <class Writer> bool write(uint8_t value, Writer &&writer)
	{
		uint32_t a;
		if (!address(a))
			return false;
		writer(a, value);
		advance();
		return true;
	}
};

struct write_entry
{
	uint32_t address = 0;
	uint8_t data = 0;
	uint64_t start = 0, end = 0;
	uint32_t slow_address = 0xffffffff;
	int16_t palette = -1;
	bool io = false, mapping = false;
};

// A caller qualifies motherboard writes and supplies retirement events. No
// clock, service time, read barrier, forwarding rule or address decode is
// inferred from the documented 31-entry capacity and FIFO ordering alone.
struct write_fifo
{
	static constexpr std::size_t capacity = 31;
	std::array<write_entry, capacity> entries{};
	uint32_t head = 0, count = 0;

	bool push(write_entry value)
	{
		if (count == capacity)
			return false; // caller retains the unaccepted write; no overwrite
		entries[(head + count) % capacity] = value;
		++count;
		return true;
	}

	bool retire(write_entry &value)
	{
		if (!count)
			return false;
		value = entries[head];
		head = (head + 1) % capacity;
		--count;
		return true;
	}
};

} // namespace applesqueezer

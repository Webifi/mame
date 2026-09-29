// license:BSD-3-Clause
// copyright-holders:CFFA3000 contributors

// Apple firmware executes on the host CPU; the AVR is currently an HLE.
// Protocol references below are byte offsets in CFFA.bin V3.1.1.  Board
// configuration and virtual-drive behavior are described in the CFFA3000
// Reference Manual v1.3 and the V3.1 firmware release notes.
// TODO: AVR/CPLD execution and arbitration, CF/USB media and FAT assignments,
//       menu/CDA initialization, virtual Disk II, measured media timing.

#include "emu.h"
#include "a2cffa3000.h"

#include "imagedev/harddriv.h"

namespace {

class a2bus_cffa3000_device : public device_t, public device_a2bus_card_interface
{
public:
	a2bus_cffa3000_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void reset_from_bus() override;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual uint8_t read_cnxx(uint8_t offset) override;
	virtual void write_cnxx(uint8_t offset, uint8_t data) override;
	virtual uint8_t read_c800(uint16_t offset) override;
	virtual void write_c800(uint16_t offset, uint8_t data) override;
	virtual uint8_t read_c0nx(uint8_t offset) override;
	virtual void write_c0nx(uint8_t offset, uint8_t data) override;
	virtual bool take_c800() const override { return true; }

private:
	enum : uint8_t { IDLE, COMMAND, READ, WRITE, COMPLETE };
	TIMER_CALLBACK_MEMBER(service);
	TIMER_CALLBACK_MEMBER(prefetch);
	void schedule(uint8_t state, attotime delay);
	attotime media_time() const;
	void cache_store(unsigned unit, uint32_t block, const uint8_t *data);
	void read_ahead(unsigned unit, uint32_t block);
	std::error_condition media_load(device_image_interface &image);
	void media_unload(device_image_interface &image);
	void command(uint8_t data);
	void finish(uint8_t error);
	void transfer(uint32_t source, uint32_t destination, uint16_t length);
	void transfer_next();
	uint32_t blocks(unsigned unit);
	uint32_t get(unsigned offset, unsigned length) const;
	void put(unsigned offset, uint32_t data, unsigned length);

	required_region_ptr<uint8_t> m_flash;
	required_device_array<harddisk_image_device, 16> m_disk;
	required_ioport m_dips;
	required_ioport m_timing;
	required_ioport m_units;
	bool m_changed[16]{};
	emu_timer *m_service_timer = nullptr;
	emu_timer *m_prefetch_timer = nullptr;
	uint8_t m_service = IDLE;
	bool m_waiting = false;
	uint32_t m_buffer = 0;
	uint8_t m_cache_data[8][512]{};
	uint32_t m_cache_block[8]{};
	uint8_t m_cache_unit[8]{};
	bool m_cache_valid[8]{};
	uint8_t m_cache_next = 0;
	uint8_t m_prefetch_unit = 0;
	uint32_t m_prefetch_block = 0;
	uint8_t m_ram[0x1000]{};
	uint8_t m_semaphore[8]{};
	uint8_t m_phase = 0;
	uint8_t m_command = 0;
	uint8_t m_unit = 0;
	uint32_t m_block = 0;
	uint32_t m_source = 0;
	uint32_t m_destination = 0;
	uint16_t m_remaining = 0;
	bool m_native = true;
	bool m_smart = false;
};

ROM_START(cffa3000)
	ROM_REGION(0x20000, "flash", ROMREGION_ERASEFF)
	ROM_LOAD_OPTIONAL("cffa3000.bin", 0, 0x1c566, CRC(ecb0e959) SHA1(2414ed9abac224d04bcf8d1d2567c3841237dac8))
ROM_END

static INPUT_PORTS_START(cffa3000)
	PORT_START("dips")
	PORT_DIPNAME(0x08, 0x00, "Apple III (DIP 4)")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x08, DEF_STR(On))
	PORT_DIPNAME(0x40, 0x40, "Apple IIgs (DIP 7)")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x40, DEF_STR(On))

	// These are HLE media delays, not measured CF/USB device characteristics.
	PORT_START("timing")
	PORT_CONFNAME(0x03, 0x00, "Media access time (approximate)")
	PORT_CONFSETTING(0x00, "250 us")
	PORT_CONFSETTING(0x01, "1 ms")
	PORT_CONFSETTING(0x02, "10 ms")
	PORT_CONFSETTING(0x03, "100 ms")
	PORT_CONFNAME(0x04, 0x04, "Read ahead (CPLD v6)")
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFSETTING(0x04, DEF_STR(On))

	// V3.1.1 setting 12 defaults to six ($40cc); unused assignments stay offline.
	PORT_START("units")
	PORT_CONFNAME(0x1f, 6, "SmartPort devices")
	PORT_CONFSETTING(1, "1")
	PORT_CONFSETTING(2, "2")
	PORT_CONFSETTING(3, "3")
	PORT_CONFSETTING(4, "4")
	PORT_CONFSETTING(5, "5")
	PORT_CONFSETTING(6, "6")
	PORT_CONFSETTING(7, "7")
	PORT_CONFSETTING(8, "8")
	PORT_CONFSETTING(9, "9")
	PORT_CONFSETTING(10, "10")
	PORT_CONFSETTING(11, "11")
	PORT_CONFSETTING(12, "12")
	PORT_CONFSETTING(13, "13")
	PORT_CONFSETTING(14, "14")
	PORT_CONFSETTING(15, "15")
	PORT_CONFSETTING(16, "16")
INPUT_PORTS_END

a2bus_cffa3000_device::a2bus_cffa3000_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, A2BUS_CFFA3000, tag, owner, clock),
	device_a2bus_card_interface(mconfig, *this),
	m_flash(*this, "flash"),
	m_disk(*this, "disk%u", 0U),
	m_dips(*this, "dips"),
	m_timing(*this, "timing"),
	m_units(*this, "units")
{
}

void a2bus_cffa3000_device::device_add_mconfig(machine_config &config)
{
	for (auto &disk : m_disk)
	{
		HARDDISK(config, disk, 0);
		disk->set_device_load(FUNC(a2bus_cffa3000_device::media_load));
		disk->set_device_unload(FUNC(a2bus_cffa3000_device::media_unload));
	}
}

std::error_condition a2bus_cffa3000_device::media_load(device_image_interface &image)
{
	media_unload(image);
	return std::error_condition();
}

void a2bus_cffa3000_device::media_unload(device_image_interface &image)
{
	for (unsigned unit = 0; unit < m_disk.size(); unit++)
		if (&image == m_disk[unit].target())
			m_changed[unit] = true;
	std::fill(std::begin(m_cache_valid), std::end(m_cache_valid), false);
	if (m_prefetch_timer)
		m_prefetch_timer->adjust(attotime::never);
}

const tiny_rom_entry *a2bus_cffa3000_device::device_rom_region() const
{
	return ROM_NAME(cffa3000);
}

ioport_constructor a2bus_cffa3000_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(cffa3000);
}

void a2bus_cffa3000_device::device_start()
{
	m_service_timer = timer_alloc(FUNC(a2bus_cffa3000_device::service), this);
	m_prefetch_timer = timer_alloc(FUNC(a2bus_cffa3000_device::prefetch), this);
	save_item(NAME(m_changed));
	save_item(NAME(m_service));
	save_item(NAME(m_waiting));
	save_item(NAME(m_buffer));
	save_item(NAME(m_cache_data));
	save_item(NAME(m_cache_block));
	save_item(NAME(m_cache_unit));
	save_item(NAME(m_cache_valid));
	save_item(NAME(m_cache_next));
	save_item(NAME(m_prefetch_unit));
	save_item(NAME(m_prefetch_block));
	save_item(NAME(m_ram));
	save_item(NAME(m_semaphore));
	save_item(NAME(m_phase));
	save_item(NAME(m_command));
	save_item(NAME(m_unit));
	save_item(NAME(m_block));
	save_item(NAME(m_source));
	save_item(NAME(m_destination));
	save_item(NAME(m_remaining));
	save_item(NAME(m_native));
	save_item(NAME(m_smart));
}

void a2bus_cffa3000_device::device_reset()
{
	if (std::all_of(&m_flash[0], &m_flash[0] + 0x20000, [](uint8_t data) { return data == 0xff; }))
	{
		std::fill(std::begin(m_ram), std::end(m_ram), 0xff);
		reset_from_bus();
		osd_printf_warning("CFFA3000 (partial) requires cffa3000.bin; card firmware unavailable\n");
		return;
	}
	// V3.1 release notes: Apple III uses 6502 code even with DIP 7 on.
	m_native = BIT(m_dips->read(), 6) && !BIT(m_dips->read(), 3);
	std::fill(std::begin(m_ram), std::end(m_ram), 0xc8);
	unsigned const base = m_native ? 0x204e : 0x1a4e;
	std::copy_n(&m_flash[base + 0x200], 0x400, &m_ram[0xc00]);
	// The AVR installs the slot page using the relocation list at $CnE0.
	std::copy_n(&m_flash[base + 0x100], 0x100, &m_ram[slotno() << 8]);
	unsigned const page = slotno() << 8;
	if (m_ram[page + 0xe0] == 0x42)
		for (unsigned i = 0xe1; i < 0xff && m_ram[page + i]; i++)
			m_ram[page + m_ram[page + i]] = 0xc0 + slotno();
	// Retain the firmware defaults at $CFC0..CFC2, including the boot-key delay.
	// Menu/CDA service itself is not yet implemented by the AVR HLE.
	reset_from_bus();
}

void a2bus_cffa3000_device::reset_from_bus()
{
	std::fill(std::begin(m_semaphore), std::end(m_semaphore), 0);
	m_ram[0xfc4] = 0;
	m_ram[0xfd3] = 0;
	m_ram[0xfeb] = 0;
	m_service_timer->adjust(attotime::never);
	m_prefetch_timer->adjust(attotime::never);
	m_service = IDLE;
	m_waiting = false;
	std::fill(std::begin(m_cache_valid), std::end(m_cache_valid), false);
	m_cache_next = 0;
	std::fill(std::begin(m_changed), std::end(m_changed), true);
	m_phase = 0;
	m_command = 0;
	m_remaining = 0;
}

uint8_t a2bus_cffa3000_device::read_cnxx(uint8_t offset)
{
	return m_ram[(slotno() << 8) | offset];
}

void a2bus_cffa3000_device::write_cnxx(uint8_t offset, uint8_t data)
{
	m_ram[(slotno() << 8) | offset] = data;
}

uint8_t a2bus_cffa3000_device::read_c800(uint16_t offset)
{
	if (offset >= 0x7f0 && offset < 0x7f8)
		return 0; // AVR mailbox updates are atomic; host ownership blocks service().
	return m_ram[0x800 + offset];
}

void a2bus_cffa3000_device::write_c800(uint16_t offset, uint8_t data)
{
	if (offset >= 0x7f0 && offset < 0x7f8)
	{
		m_semaphore[offset & 7] = !BIT(data, 0);
		if (offset == 0x7f0 && BIT(data, 0) && m_waiting)
		{
			m_waiting = false;
			m_service_timer->adjust(attotime::zero);
		}
		else if (offset == 0x7f0 && BIT(data, 0) && m_service == IDLE)
		{
			if (m_ram[0xfd3])
				schedule(COMMAND, attotime::from_usec(4));
			else if (m_phase && !m_ram[0xfc4])
				schedule(COMPLETE, attotime::from_usec(4));
		}
		return;
	}
	m_ram[0x800 + offset] = data;
}

attotime a2bus_cffa3000_device::media_time() const
{
	static constexpr unsigned usec[] = { 250, 1000, 10000, 100000 };
	return attotime::from_usec(usec[m_timing->read() & 3]);
}

void a2bus_cffa3000_device::schedule(uint8_t state, attotime delay)
{
	m_service = state;
	m_service_timer->adjust(delay);
}

void a2bus_cffa3000_device::cache_store(unsigned unit, uint32_t block, const uint8_t *data)
{
	unsigned const entry = m_cache_next;
	std::copy_n(data, 512, m_cache_data[entry]);
	m_cache_unit[entry] = unit;
	m_cache_block[entry] = block;
	m_cache_valid[entry] = true;
	m_cache_next = (entry + 1) & 7;
}

void a2bus_cffa3000_device::read_ahead(unsigned unit, uint32_t block)
{
	if (!BIT(m_timing->read(), 2) || block >= blocks(unit))
		return;
	if (!m_prefetch_timer->expire().is_never() && m_prefetch_unit == unit && m_prefetch_block == block)
		return;
	for (unsigned i = 0; i < std::size(m_cache_valid); i++)
		if (m_cache_valid[i] && m_cache_unit[i] == unit && m_cache_block[i] == block)
			return;
	m_prefetch_unit = unit;
	m_prefetch_block = block;
	m_prefetch_timer->adjust(media_time());
}

TIMER_CALLBACK_MEMBER(a2bus_cffa3000_device::prefetch)
{
	// AVR $edfc queues one block ahead while $f084 waits for the host copy.
	// Its eight cache descriptors start at AVR SRAM $0501.
	uint8_t data[512];
	if (m_disk[m_prefetch_unit]->exists() && m_disk[m_prefetch_unit]->read(m_prefetch_block, data))
		cache_store(m_prefetch_unit, m_prefetch_block, data);
}

TIMER_CALLBACK_MEMBER(a2bus_cffa3000_device::service)
{
	// Never process a request as a side effect of a host/debugger read.  The
	// original host code spins in $CC23 and remains interruptible throughout.
	// Four microseconds is the AVR timer tick ($42ac/$4b2e), used here as an
	// HLE polling quantum, not a cycle-exact AVR dispatch time.
	// Resume a blocked AVR transaction on release, avoiding scheduler phase
	// locking against the host's acquire/release polling loop.
	if (m_semaphore[0])
	{
		m_waiting = true;
		return;
	}
	if (!m_ram[0xfeb])
	{
		m_service_timer->adjust(attotime::from_usec(4));
		return;
	}
	uint8_t const state = m_service;
	m_service = IDLE;
	switch (state)
	{
	case COMMAND:
		m_command = m_ram[0xfd3];
		m_ram[0xfd3] = 0;
		command(m_command);
		m_command = 0;
		break;

	case READ:
		if (!m_disk[m_unit]->exists() || !m_disk[m_unit]->read(m_block, &m_ram[0x800]))
		{
			finish(0x27);
			break;
		}
		if (BIT(m_timing->read(), 2))
			cache_store(m_unit, m_block, &m_ram[0x800]);
		transfer(0xc800, m_buffer, 512);
		read_ahead(m_unit, m_block + 1);
		break;

	case COMPLETE:
		if (m_remaining)
			transfer_next();
		else if (m_phase == 2)
			schedule(WRITE, media_time());
		else
		{
			m_phase = 0;
			finish(0);
		}
		break;

	case WRITE:
		{
			uint8_t error = 0;
			if (m_disk[m_unit]->is_readonly())
				error = 0x2b;
			else if (!m_disk[m_unit]->write(m_block, &m_ram[0x800]))
				error = 0x27;
			m_phase = 0;
			finish(error);
		}
		break;
	}
}

uint8_t a2bus_cffa3000_device::read_c0nx(uint8_t offset)
{
	return 0xff;
}

void a2bus_cffa3000_device::write_c0nx(uint8_t offset, uint8_t data)
{
}

uint32_t a2bus_cffa3000_device::get(unsigned offset, unsigned length) const
{
	uint32_t result = 0;
	for (unsigned i = 0; i < length; i++)
		result |= uint32_t(m_ram[offset + i]) << (8 * i);
	return result;
}

void a2bus_cffa3000_device::put(unsigned offset, uint32_t data, unsigned length)
{
	for (unsigned i = 0; i < length; i++)
		m_ram[offset + i] = data >> (8 * i);
}

uint32_t a2bus_cffa3000_device::blocks(unsigned unit)
{
	if (unit >= m_disk.size() || !m_disk[unit]->exists())
		return 0;
	auto const &info = m_disk[unit]->get_info();
	return info.cylinders * info.heads * info.sectors;
}

void a2bus_cffa3000_device::finish(uint8_t error)
{
	// V3.1.1 $7374 returns $0200 even for status and error responses.
	if (m_smart)
		put(0xfc6, 512, 2);
	m_ram[0xfc5] = error;
	m_ram[0xfc8] = (m_ram[0xfc8] & 0xfe) | (error ? 1 : 0);
	m_ram[0xfc4] = 2;
}

void a2bus_cffa3000_device::transfer(uint32_t source, uint32_t destination, uint16_t length)
{
	m_source = source;
	m_destination = destination;
	m_remaining = length;
	m_phase = 1;
	transfer_next();
}

void a2bus_cffa3000_device::transfer_next()
{
	uint32_t const source = m_source;
	uint32_t const destination = m_destination;
	unsigned const length = m_native ? std::min({unsigned(m_remaining), 0x10000U - (source & 0xffff), 0x10000U - (destination & 0xffff)}) : m_remaining;
	m_remaining -= length;
	m_source += length;
	m_destination += length;
	// Reproduce the AVR's parameterization of its host-side copy templates.
	if (m_native)
	{
		std::copy_n(&m_flash[0x2868], 0x22, &m_ram[0xa40]);
		m_ram[0xa43] = source >> 16;
		m_ram[0xa47] = destination >> 16;
		put(0xa50, length - 1, 2);
		put(0xa53, source, 2);
		put(0xa56, destination, 2);
	}
	else if (length == 512)
	{
		std::copy_n(&m_flash[0x2837], 0x12, &m_ram[0xa40]);
		put(0xa43, source, 2);
		put(0xa46, destination, 2);
		put(0xa49, source + 256, 2);
		put(0xa4c, destination + 256, 2);
	}
	else
	{
		std::copy_n(&m_flash[0x2849], 0xf, &m_ram[0xa40]);
		m_ram[0xa41] = length;
		put(0xa46, source, 2);
		put(0xa49, destination, 2);
	}
	put(0xfc9, 0xca40, 2);
	m_ram[0xfc4] = 0x0a;
}

void a2bus_cffa3000_device::command(uint8_t data)
{
	m_smart = data == 0xa8;
	logerror("command %02x ProDOS %02x unit %02x block %04x SmartPort %02x\n", data, m_ram[0xfd4], m_ram[0xfd5], get(0xfd8, 2), m_ram[0xfda]);
	if (data == 0xa0 || data == 0xa5)
	{
		finish(0);
		return;
	}
	if (data != 0xa6 && data != 0xa8)
	{
		logerror("unimplemented command %02x\n", data);
		finish(1);
		return;
	}

	bool const smart = data == 0xa8;
	bool const extended = smart && BIT(m_ram[0xfda], 6);
	unsigned const cmd = smart ? m_ram[0xfda] & 0xbf : m_ram[0xfd4];
	// $6c86..6cb4 maps a different ProDOS slot number to assignments 3/4.
	unsigned const unit = smart ? unsigned(m_ram[0xfe0]) - 1
		: BIT(m_ram[0xfd5], 7) + (((m_ram[0xfd5] >> 4) & 7) == slotno() ? 0 : 2);
	uint32_t const buffer = smart ? get(0xfe1, extended ? 4 : 2) : get(0xfd6, 2);
	// The non-extended parser at $50aa..50c6 ORs byte 2 into byte 0.
	uint32_t const block = smart ? (extended ? get(0xfe5, 4) : (get(0xfe3, 2) | m_ram[0xfe5])) : get(0xfd8, 2);
	uint32_t const size = blocks(unit);

	if (smart && cmd <= 9 && m_ram[0xfdf] != m_flash[0x7fa + cmd])
	{
		finish(4);
		return;
	}

	// The dispatch order and error precedence follow $7128..7394.
	if (smart && cmd > 9)
	{
		finish(1);
		return;
	}
	if (smart && cmd == 5)
	{
		finish(m_ram[0xfe0] == 0 ? 0 : 0x11);
		return;
	}
	if (smart && (cmd == 6 || cmd == 7 || cmd == 8 || cmd == 9))
	{
		finish(cmd < 8 ? 1 : 0x27);
		return;
	}
	if (cmd == 0 && smart && m_ram[0xfe0] == 0)
	{
		if (m_ram[extended ? 0xfe5 : 0xfe3] != 0)
		{
			finish(0x21);
			return;
		}
		std::fill_n(&m_ram[0x800], 8, 0);
		m_ram[0x800] = m_units->read();
		m_ram[0x801] = 0x40;
		m_ram[0x803] = 0xcc;
		m_ram[0x805] = 0x30;
		transfer(0xc800, buffer, 8);
		return;
	}
	if (smart && unit >= m_units->read())
	{
		finish(0x11);
		return;
	}
	if (cmd == 3)
	{
		// Format is a no-op, including for offline/write-protected media.
		// Extended format acknowledges the changed-media latch ($72ea).
		bool const changed = smart && extended && m_changed[unit];
		if (smart && extended)
			m_changed[unit] = false;
		finish(changed ? 0x2e : 0);
		return;
	}
	if (smart && cmd == 4)
	{
		unsigned const code = m_ram[extended ? 0xfe5 : 0xfe3];
		// $734a falls through: both codes 2 and 3 return $1f.
		finish(code < 2 || code == 4 ? 0 : code < 4 ? 0x1f : 0x21);
		return;
	}
	if (cmd == 0)
	{
		if (!smart)
		{
			put(0xfc6, std::min(size, uint32_t(0xffff)), 2);
			finish(!size ? 0x2f : m_disk[unit]->is_readonly() ? 0x2b : 0);
		}
		else
		{
			unsigned const code = m_ram[extended ? 0xfe5 : 0xfe3];
			if (code != 0 && code != 1 && code != 3)
			{
				finish(0x21);
				return;
			}
			if (code == 1)
			{
				// $70a2: two-byte device control block.
				put(0x800, 1, 2);
				transfer(0xc800, buffer, 2);
				return;
			}
			std::fill_n(&m_ram[0x800], 32, 0);
			// Preserve the V3.1.1 flag polarity at $6f60..6f9c.
			m_ram[0x800] = !size ? 0xe8 : m_disk[unit]->is_readonly() ? 0xfc : 0xf8;
			if (extended && m_changed[unit])
				m_ram[0x800] |= 1;
			uint32_t reported = size;
			if (!extended)
			{
				reported = std::min(reported, uint32_t(0xffffff));
				if (reported && !(reported & 0xffff))
					reported--;
			}
			put(0x801, reported, extended ? 4 : 3);
			unsigned length = extended ? 5 : 4;
			if (code == 3)
			{
				m_ram[0x800 + length] = 16;
				std::copy_n("CFFA3000        ", 16, &m_ram[0x801 + length]);
				m_ram[0x811 + length] = 2;
				m_ram[0x812 + length] = m_native ? 0xc0 : 0;
				m_ram[0x814 + length] = 0x30;
				length += 21;
			}
			put(0xfc6, length, 2);
			transfer(0xc800, buffer, length);
		}
		return;
	}
	if (cmd != 1 && cmd != 2)
	{
		finish(1);
		return;
	}
	if (!size)
	{
		finish(0x2f);
		return;
	}
	if (smart && extended && m_changed[unit])
	{
		m_changed[unit] = false;
		finish(0x2e);
		return;
	}
	if (block >= size)
	{
		finish(0x2d);
		return;
	}
	m_unit = unit;
	m_block = block;
	if (cmd == 1)
	{
		m_buffer = buffer;
		if (BIT(m_timing->read(), 2))
			for (unsigned i = 0; i < std::size(m_cache_valid); i++)
				if (m_cache_valid[i] && m_cache_unit[i] == unit && m_cache_block[i] == block)
				{
					std::copy_n(m_cache_data[i], 512, &m_ram[0x800]);
					transfer(0xc800, buffer, 512);
					read_ahead(unit, block + 1);
					return;
				}
		// A read already in flight need only wait for its remaining media time.
		attotime const delay = !m_prefetch_timer->expire().is_never() && m_prefetch_unit == unit && m_prefetch_block == block
				? m_prefetch_timer->remaining() : media_time();
		m_prefetch_timer->adjust(attotime::never);
		schedule(READ, delay);
	}
	else
	{
		m_prefetch_timer->adjust(attotime::never);
		std::fill(std::begin(m_cache_valid), std::end(m_cache_valid), false);
		transfer(buffer, 0xc800, 512);
		m_phase = 2;
	}
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(A2BUS_CFFA3000, device_a2bus_card_interface, a2bus_cffa3000_device, "cffa3000", "CFFA3000 CompactFlash/USB interface (partial)")

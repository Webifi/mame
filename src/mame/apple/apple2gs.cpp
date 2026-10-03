// license:BSD-3-Clause
// copyright-holders:R. Belmont
/***************************************************************************

    apple2gs.cpp - Apple IIgs

    Next generation driver written June 2018 by R. Belmont.
    Thanks to the original Apple IIgs driver's authors: Nathan Woods and R. Belmont
    Thanks also to the Apple II Documentation Project/Antoine Vignau, Peter Ferrie, and Olivier Galibert.

    Unique hardware configurations:
    - ROM 00/01: original motherboard, 256K of RAM (banks 00/01/E0/E1 only), FPI chip manages fast/slow side
    - ROM 03: revised motherboard, 1M of RAM (banks 00/01/->0F/E0/E1), CYA chip replaces FPI
    - Expanded IIe: ROM 00/01 motherboard in a IIe case with a IIe keyboard on J13
    - "Mark Twain" prototype: ROM 3 hardware, SWIM1 instead of IWM, built-in floppy, integrated High-Speed SCSI Card
      and internal SCSI HDD, 2 SIMM slots for RAM expansion instead of the proprietary memory slot of the previous IIgs.
      Only 5 slots: slots 5 and 7 are missing (5 for the SuperDrive, 7 for the SCSI).

    Timing in terms of the 14M 14.3181818 MHz clock (1/2 of the 28.6363636 master clock):
    - 1 2.8 MHz 65816 cycle is 5 14M clocks.
    - The Mega II 1 MHz side runs for 64 cycles at 14 14M clocks and every 65th is stretched to 16 14M clocks.
      This allows 8-bit Apple II raster demos to work.  Each scanline is (64*14) + 16 = 912 14M clocks.
      Due to this stretch, which does not occur on the fast side, the fast and 1 Mhz sides drift from each other
      and sync up every 22800 14M clocks (25 scan lines).
    - Accesses to the 1 MHz side incur a side-sync penalty (waiting for the start of the next 1 MHz cycle).
    - Every 50 14M clocks (10 65816 cycles) DRAM refresh occurs for 5 14M clocks
      * During this time, CPU accesses to ROM, Mega II side I/O, or banks E0/E1 are not penalized (but a side-sync penalty is incurred for the 1 MHz side)
      * Accesses to banks 00-7F are penalized except for I/O in banks 0/1.
    - ROM accesses always run at full speed.

    One video line is: 6 cycles of right border, 13 cycles of hblank, 6 cycles of left border, and 40 cycles of active video

    ((6*14)*2) + 560 = 728 (total for A2 modes)  htotal = 910 (65 * 14)
    ((6*16)*2) + 640 = 832 (total for SHR)       htotal = 1040 (65 * 16)

    FF6ACF is speed test in ROM
    Diags:
    A138 = scanline interrupt test (raster is too long to pass this)
    A179 = pass
    A17C = fail 1
    A0F1 = fail 2

    ZipGS notes:
    $C059 is the GS settings register
    bit 3: CPS Follow
    bit 4: Counter Delay
    bit 5: AppleTalk Delay
    bit 6: Joystick Delay
    bit 7: C/D cache disable

    $C05D is the speed percentage:
    $F0 = 6%, $E0 = 12%, $D0 = 18%, $C0 = 25%, $B0 = 31%, $A0 = 37%, $90 = 43%, $80 = 50%,
    $70 = 56%, $60 = 62%, $50 = 68%, $40 = 75%, $30 = 81%, $20 = 87%, $10 = 93%, $00 = 100%

***************************************************************************/

#include "emu.h"
#include "twgs_timing.h"
#include "twgs_clock.h"
#include "osdcore.h"

#include "apple2video.h"

#include "apple2common.h"
// #include "machine/apple2host.h"
#include "macadb.h"
#include "macrtc.h"

#include "bus/a2bus/a2bus.h"
#include "bus/a2bus/cards.h"
#include "bus/a2gameio/gameio.h"
#include "bus/rs232/rs232.h"
#include "cpu/g65816/g65816.h"
#include "cpu/m6502/m5074x.h"
#include "machine/bankdev.h"
#include "machine/eepromser.h"
#include "machine/ram.h"
#include "machine/timer.h"
#include "machine/z80scc.h"
#include "sound/es5503.h"
#include "sound/spkrdev.h"

#include "machine/applefdintf.h"
#include "machine/iwm.h"
#include "machine/swim1.h"

#include "screen.h"
#include "softlist_dev.h"
#include "speaker.h"

#include "utf8.h"

#include <bit>

#define LOG_GLU (1U << 1)
#define LOG_GLUTRACE (1U << 2)
#define VERBOSE (LOG_GLU | LOG_GLUTRACE)
#include "logmacro.h"

namespace {

// various timing standards
#define A2GS_MASTER_CLOCK (XTAL(28'636'363))
#define A2GS_14M    (A2GS_MASTER_CLOCK/2)
#define A2GS_7M     (A2GS_MASTER_CLOCK/4)
#define A2GS_2_8M   (A2GS_MASTER_CLOCK/10)
#define A2GS_1M     (XTAL(1021800))

#define A2GS_UPPERBANK_TAG "inhbank"
#define A2GS_AUXUPPER_TAG "inhaux"
#define A2GS_00UPPER_TAG "inh00"
#define A2GS_01UPPER_TAG "inh01"

#define A2GS_C300_TAG "c3bank"
#define A2GS_LCBANK_TAG "lcbank"
#define A2GS_LCAUX_TAG "lcaux"
#define A2GS_LC00_TAG "lc00"
#define A2GS_LC01_TAG "lc01"
#define A2GS_B0CXXX_TAG "bnk0atc"
#define A2GS_B1CXXX_TAG "bnk1atc"
#define A2GS_B00000_TAG "b0r00bank"
#define A2GS_B00200_TAG "b0r02bank"
#define A2GS_B00400_TAG "b0r04bank"
#define A2GS_B00800_TAG "b0r08bank"
#define A2GS_B02000_TAG "b0r20bank"
#define A2GS_B04000_TAG "b0r40bank"

#define A2GS_KBD_SPEC_TAG "keyb_special"

class apple2gs_state : public driver_device
{
public:
	apple2gs_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		  m_maincpu(*this, "maincpu"),
		  m_screen(*this, "screen"),
		  m_scantimer(*this, "scantimer"),
		  m_vgctimer(*this, "vgctimer"),
		  m_acceltimer(*this, "acceltimer"),
		  m_adbmicro(*this, "adbmicro"),
		  m_macadb(*this, "macadb"),
		  m_ram(*this, "ram"),
		  m_twgs_nvram(*this, "twgs_nvram"),
		  m_rom(*this, "maincpu"),
		  m_docram(*this, "docram"),
		  m_video(*this, "a2video"),
		  m_rtc(*this, "rtc"),
		  m_a2bus(*this, "a2bus"),
		  m_a2common(*this, "a2common"),
		  //      m_a2host(*this, "a2host"),
		  m_gameio(*this, "gameio"),
		  m_speaker(*this, "speaker_sound"),
		  m_sndmono(*this, "sndmono"),
		  m_sndleft(*this, "sndleft"),
		  m_sndright(*this, "sndright"),
		  m_upperbank(*this, A2GS_UPPERBANK_TAG),
		  m_upperaux(*this, A2GS_AUXUPPER_TAG),
		  m_upper00(*this, A2GS_00UPPER_TAG),
		  m_upper01(*this, A2GS_01UPPER_TAG),
		  m_c300bank(*this, A2GS_C300_TAG),
		  m_b0_0000bank(*this, A2GS_B00000_TAG),
		  m_b0_0200bank(*this, A2GS_B00200_TAG),
		  m_b0_0400bank(*this, A2GS_B00400_TAG),
		  m_b0_0800bank(*this, A2GS_B00800_TAG),
		  m_b0_2000bank(*this, A2GS_B02000_TAG),
		  m_b0_4000bank(*this, A2GS_B04000_TAG),
		  m_e0_0000bank(*this, "e0_0000_bank"),
		  m_e0_0200bank(*this, "e0_0200_bank"),
		  m_e0_0400bank(*this, "e0_0400_bank"),
		  m_e0_0800bank(*this, "e0_0800_bank"),
		  m_e0_2000bank(*this, "e0_2000_bank"),
		  m_e0_4000bank(*this, "e0_4000_bank"),
		  m_lcbank(*this, A2GS_LCBANK_TAG),
		  m_lcaux(*this, A2GS_LCAUX_TAG),
		  m_lc00(*this, A2GS_LC00_TAG),
		  m_lc01(*this, A2GS_LC01_TAG),
		  m_bank0_atc(*this, A2GS_B0CXXX_TAG),
		  m_bank1_atc(*this, A2GS_B1CXXX_TAG),
		  m_scc(*this, "scc"),
		  m_doc(*this, "doc"),
		  m_iwm(*this, "fdc"),
		  m_floppy(*this, "fdc:%d", 0U),
		  m_sysconfig(*this, "a2_config"),
		  m_btconfig(*this, "bus_timing"),
		  m_j13_config_port(*this, "j13_config"),
		  m_j13_keys(*this, "j13_y%u", 0U),
		  m_j13_special(*this, "j13_special")

	{
		m_cur_floppy = nullptr;
		m_devsel = 0;
		m_diskreg = 0;
	}

	void apple2gs(machine_config &config);
	void apple2gsr1(machine_config &config);
	void apple2gsmt(machine_config &config);

	void j13_config_postload();
	DECLARE_INPUT_CHANGED_MEMBER(j13_config_changed);
	int adb_keyboard_connected() { return !BIT(m_j13_config, 1); }
	int j13_enabled() { return BIT(m_j13_config, 0); }

	void rom1_init() { m_is_rom3 = false; }
	void rom3_init() { m_is_rom3 = true; }

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	required_device<g65816_device> m_maincpu;
	required_device<screen_device> m_screen;
	required_device<timer_device> m_scantimer, m_vgctimer, m_acceltimer;
	required_device<m5074x_device> m_adbmicro;
	required_device<macadb_device> m_macadb;
	required_device<ram_device> m_ram;
	required_device<eeprom_serial_x24c44_16bit_device> m_twgs_nvram;
	required_region_ptr<u8> m_rom;
	required_shared_ptr<u8> m_docram;
	required_device<a2_video_device> m_video;
	required_device<rtc3430042_device> m_rtc;
	required_device<a2bus_device> m_a2bus;
	required_device<apple2_common_device> m_a2common;
//  required_device<apple2_host_device> m_a2host;
	required_device<apple2_gameio_device> m_gameio;
	required_device<speaker_sound_device> m_speaker;
	required_device<speaker_device> m_sndmono, m_sndleft, m_sndright;
	memory_view m_upperbank, m_upperaux, m_upper00, m_upper01;
	required_device<address_map_bank_device> m_c300bank;
	memory_view m_b0_0000bank, m_b0_0200bank, m_b0_0400bank, m_b0_0800bank, m_b0_2000bank, m_b0_4000bank;
	memory_view m_e0_0000bank, m_e0_0200bank, m_e0_0400bank, m_e0_0800bank, m_e0_2000bank, m_e0_4000bank;
	memory_view m_lcbank, m_lcaux, m_lc00, m_lc01, m_bank0_atc, m_bank1_atc;
	required_device<z80scc_device> m_scc;
	required_device<es5503_device> m_doc;
	required_device<applefdintf_device> m_iwm;
	required_device_array<floppy_connector, 4> m_floppy;
	required_ioport m_sysconfig;
	required_ioport m_btconfig;
	optional_ioport m_j13_config_port;
	optional_ioport_array<10> m_j13_keys;
	optional_ioport m_j13_special;
	u8 m_j13_config = 0;

	static constexpr int CNXX_UNCLAIMED = -1;

	enum glu_reg_names
	{
		// these are the MCU-visible registers
		GLU_KEY_DATA = 0,   // MCU W
		GLU_COMMAND,        // MCU R
		GLU_MOUSEX,         // MCU W
		GLU_MOUSEY,         // MCU W
		GLU_KG_STATUS,      // MCU R
		GLU_ANY_KEY_DOWN,   // MCU W
		GLU_KEYMOD,         // MCU W
		GLU_DATA,           // MCU W

		GLU_C000,       // 816 R
		GLU_C010,       // 816 RW
		GLU_SYSSTAT     // 816 R/(limited) W
	};

	static constexpr u8 KGS_ANY_KEY_DOWN = 0x01;
	static constexpr u8 KGS_KEYSTROBE    = 0x10;
	static constexpr u8 KGS_DATA_FULL    = 0x20;
	static constexpr u8 KGS_COMMAND_FULL = 0x40;
	static constexpr u8 KGS_MOUSEX_FULL  = 0x80;

	static constexpr u8 GLU_STATUS_CMDFULL  = 0x01;
	static constexpr u8 GLU_STATUS_MOUSEXY  = 0x02;
	static constexpr u8 GLU_STATUS_KEYDATIRQEN = 0x04;
	static constexpr u8 GLU_STATUS_KEYDATIRQ = 0x08;
	static constexpr u8 GLU_STATUS_DATAIRQEN = 0x10;
	static constexpr u8 GLU_STATUS_DATAIRQ  = 0x20;
	static constexpr u8 GLU_STATUS_MOUSEIRQEN = 0x40;
	static constexpr u8 GLU_STATUS_MOUSEIRQ = 0x080;

	static constexpr u8 SHAD_IOLC       = 0x40; // I/O and language card inhibit for banks 00/01
	static constexpr u8 SHAD_TXTPG2     = 0x20; // inhibits text-page 2 shadowing in both banks (ROM 03 h/w only)
	static constexpr u8 SHAD_AUXHIRES   = 0x10; // inhibits bank 01 hi-res region shadowing
	static constexpr u8 SHAD_SUPERHIRES = 0x08; // inhibits bank 01 super-hi-res region shadowing
	static constexpr u8 SHAD_HIRESPG2   = 0x04; // inhibits hi-res page 2 shadowing in both banks
	static constexpr u8 SHAD_HIRESPG1   = 0x02; // inhibits hi-res page 1 shadowing in both banks
	static constexpr u8 SHAD_TXTPG1     = 0x01; // inhibits text-page 1 shadowing in both banks

	static constexpr u8 SPEED_HIGH      = 0x80; // full 2.8 MHz speed when set, Apple II 1 MHz when clear
	static constexpr u8 SPEED_POWERON = 0x40; // ROM 03 power-on latch, cleared by firmware
	static constexpr u8 SPEED_ALLBANKS  = 0x10; // enables bank 0/1 video shadowing in all RAM banks
	[[maybe_unused]] static constexpr u8 SPEED_DISKIISL7 = 0x08; // enable Disk II motor on detect for slot 7
	[[maybe_unused]] static constexpr u8 SPEED_DISKIISL6 = 0x04; // enable Disk II motor on detect for slot 6
	[[maybe_unused]] static constexpr u8 SPEED_DISKIISL5 = 0x02; // enable Disk II motor on detect for slot 5
	[[maybe_unused]] static constexpr u8 SPEED_DISKIISL4 = 0x01; // enable Disk II motor on detect for slot 4

	static constexpr u8 DISKREG_HDSEL = 7; // select signal for 3.5" Sony drives
	static constexpr u8 DISKREG_35SEL = 6; // 1 to enable 3.5" drives, 0 to chain through to 5.25"

	enum irq_sources
	{
		IRQS_DOC        = 0,
		IRQS_SCAN       = 1,
		IRQS_ADB        = 2,
		IRQS_VBL        = 3,
		IRQS_SECOND     = 4,
		IRQS_QTRSEC     = 5,
		IRQS_SLOT       = 6,
		IRQS_SCC        = 7
	};

	static constexpr u8 INTFLAG_IRQASSERTED = 0x01;
	[[maybe_unused]] static constexpr u8 INTFLAG_M2MOUSEMOVE = 0x02;
	[[maybe_unused]] static constexpr u8 INTFLAG_M2MOUSESW   = 0x04;
	static constexpr u8 INTFLAG_VBL         = 0x08;
	static constexpr u8 INTFLAG_QUARTER     = 0x10;
	static constexpr u8 INTFLAG_AN3         = 0x20;
	[[maybe_unused]] static constexpr u8 INTFLAG_MOUSEDOWNLAST = 0x40;
	[[maybe_unused]] static constexpr u8 INTFLAG_MOUSEDOWN   = 0x80;

	[[maybe_unused]]  static constexpr u8 VGCINT_EXTERNALEN = 0x01;
	static constexpr u8 VGCINT_SCANLINEEN   = 0x02;
	static constexpr u8 VGCINT_SECONDENABLE = 0x04;
	[[maybe_unused]] static constexpr u8 VGCINT_EXTERNAL     = 0x10;
	static constexpr u8 VGCINT_SCANLINE     = 0x20;
	static constexpr u8 VGCINT_SECOND       = 0x40;
	static constexpr u8 VGCINT_ANYVGCINT    = 0x80;

	enum adbstate_t
	{
		ADBSTATE_IDLE,
		ADBSTATE_INCOMMAND,
		ADBSTATE_INRESPONSE
	};

	bool m_adb_line = false;

	// align timing to match observed hardware behavior
	static constexpr int ALIGN_VBL = 4;
	static constexpr int ALIGN_CNT = 2;
	static constexpr int ALIGN_RFB = 1;
	static constexpr int HPOS_VBL = BORDER_LEFT + ((40 - ALIGN_VBL) * 16);
	// hardware testing shows SCB IRQs fire 8 video cycles after VBL
	static constexpr int HPOS_VGC = HPOS_VBL + (8 * 16);
	TIMER_DEVICE_CALLBACK_MEMBER(apple2_interrupt);
	TIMER_DEVICE_CALLBACK_MEMBER(apple2_vgc);
	TIMER_DEVICE_CALLBACK_MEMBER(accel_timer);

	void palette_init(palette_device &palette);

	void apple2gs_map(address_map &map) ATTR_COLD;
	void vectors_map(address_map &map) ATTR_COLD;
	void a2gs_es5503_map(address_map &map) ATTR_COLD;
	void c300bank_map(address_map &map) ATTR_COLD;

	void phases_w(uint8_t phases);
	void sel35_w(int sel35);
	void devsel_w(uint8_t devsel);
	void hdsel_w(int hdsel);

	floppy_image_device *m_cur_floppy = nullptr;
	int m_devsel = 0;
	u8 m_diskreg = 0;

	u8 auxram0000_r(offs_t offset);
	void auxram0000_w(offs_t offset, u8 data);
	u8 b0ram0000_r(offs_t offset);
	void b0ram0000_w(offs_t offset, u8 data);
	u8 b0ram0200_r(offs_t offset);
	void b0ram0200_w(offs_t offset, u8 data);
	u8 b0ram0400_r(offs_t offset);
	void b0ram0400_w(offs_t offset, u8 data);
	u8 b0ram0800_r(offs_t offset);
	void b0ram0800_w(offs_t offset, u8 data);
	u8 b0ram2000_r(offs_t offset);
	void b0ram2000_w(offs_t offset, u8 data);
	u8 b0ram4000_r(offs_t offset);
	void b0ram4000_w(offs_t offset, u8 data);
	u8 b1ram0000_r(offs_t offset);
	void b1ram0000_w(offs_t offset, u8 data);
	u8 b1ram0200_r(offs_t offset);
	void b1ram0200_w(offs_t offset, u8 data);
	u8 b1ram0400_r(offs_t offset);
	void b1ram0400_w(offs_t offset, u8 data);
	u8 b1ram0800_r(offs_t offset);
	void b1ram0800_w(offs_t offset, u8 data);
	u8 b1ram2000_r(offs_t offset);
	void b1ram2000_w(offs_t offset, u8 data);
	u8 b1ram4000_r(offs_t offset);
	void b1ram4000_w(offs_t offset, u8 data);

	template <int Addr> u8 e0ram_r(offs_t offset);
	template <int Addr> void e0ram_w(offs_t offset, u8 data);
	template <int Addr> u8 e1ram_r(offs_t offset);
	template <int Addr> void e1ram_w(offs_t offset, u8 data);

	u8 c000_r(offs_t offset);
	void c000_w(offs_t offset, u8 data);
	u8 c080_r(offs_t offset);
	void c080_w(offs_t offset, u8 data);
	u8 c100_r(offs_t offset);
	void c100_w(offs_t offset, u8 data);
	u8 c300_r(offs_t offset);
	u8 c300_int_r(offs_t offset);
	void c300_w(offs_t offset, u8 data);
	u8 c400_r(offs_t offset);
	void c400_w(offs_t offset, u8 data);
	u8 c800_r(offs_t offset);
	void c800_w(offs_t offset, u8 data);
	u8 inh_r(offs_t offset);
	void inh_w(offs_t offset, u8 data);
	u8 lc_r(offs_t offset);
	void lc_w(offs_t offset, u8 data);
	u8 lc_aux_r(offs_t offset);
	void lc_aux_w(offs_t offset, u8 data);
	u8 lc_00_r(offs_t offset);
	void lc_00_w(offs_t offset, u8 data);
	u8 lc_01_r(offs_t offset);
	void lc_01_w(offs_t offset, u8 data);
	u8 bank0_c000_r(offs_t offset);
	void bank0_c000_w(offs_t offset, u8 data);
	u8 bank1_0000_r(offs_t offset);
	void bank1_0000_sh_w(offs_t offset, u8 data);
	u8 bank1_c000_r(offs_t offset);
	void bank1_c000_w(offs_t offset, u8 data);
	u8 floatingbank_r(offs_t offset);
	u8 ghostram_r(offs_t offset);
	void ghostram_w(offs_t offset, u8 data);
	void expansion_ram_w(offs_t offset, u8 data);
	void expansion_shadow_w(offs_t offset, u8 data);
	void a2bus_irq_w(int state);
	void a2bus_nmi_w(int state);
	void a2bus_inh_w(int state);
	void doc_irq_w(int state);
	void scc_irq_w(int state);
	u8 doc_adc_read();
	u8 apple2gs_read_vector(offs_t offset);

	u8 keyglu_mcu_read(u8 offset);
	void keyglu_mcu_write(u8 offset, u8 data);
	u8 keyglu_816_read(u8 offset);
	void keyglu_816_write(u8 offset, u8 data);

	u8 m_adb_p2_last, m_adb_p3_last;
	int m_adb_reset_freeze = 0;
	bool m_ram_initialized = false;
	void keyglu_regen_irqs();

	u8 adbmicro_p0_in();
	u8 adbmicro_p1_in();
	u8 adbmicro_p2_in();
	u8 adbmicro_p3_in();
	void adbmicro_p0_out(u8 data);
	void adbmicro_p1_out(u8 data);
	void adbmicro_p2_out(u8 data);
	void adbmicro_p3_out(u8 data);
	void set_adb_line(int linestate);

	offs_t dasm_trampoline(std::ostream &stream, offs_t pc, const util::disasm_interface::data_buffer &opcodes, const util::disasm_interface::data_buffer &params);
	void wdm_trampoline(offs_t offset, u8 data) { }; //m_a2host->wdm_w(space, offset, data); }

	bool m_is_rom3 = false;
	int m_speaker_state = 0;

	double m_joystick_x1_time = 0, m_joystick_y1_time = 0, m_joystick_x2_time = 0, m_joystick_y2_time = 0;

	int m_inh_slot = 0, m_cnxx_slot = 0;
	int m_motoroff_time = 0;

	bool m_an0 = false, m_an1 = false, m_an2 = false, m_an3 = false;

	bool m_vbl = false;

	int m_irqmask = 0;

	bool m_intcxrom = false;
	bool m_slotc3rom = false;
	bool m_intc8rom = false;
	bool m_altzp = false;
	bool m_ramrd = false, m_ramwrt = false;
	bool m_lcram = false, m_lcram2 = false, m_lcprewrite = false, m_lcwriteenable = false;
	bool m_rombank = false;

	u8 m_shadow = 0, m_speed = 0;
	u8 m_motors_active = 0, m_slotromsel = 0, m_intflag = 0, m_vgcint = 0, m_inten = 0;

	bool m_last_speed = false;

	// Sound GLU. The DOC host port is free on one phase of the oscillator slot
	// (8 clocks of the 7.159 MHz DOC clock). A CPU transfer that misses that
	// phase is latched and completed on the following slot, so the port is
	// busy for between one and two slots. Register writes do not stall: a
	// second write replaces the byte still in the latch. Register reads and
	// sound-RAM accesses hold the CPU until the DOC port finishes. The 16-step
	// VCA sits after the DOC/speaker sum.
	u8 m_sndglu_ctrl = 0x0f;
	u16 m_sndglu_addr = 0;
	int m_sndglu_dummy_read = 0;
	attotime m_sndglu_busy_until;
	bool m_snd_demux = false;
	bool m_sndglu_pending = false;
	bool m_sndglu_pend_read = false;
	bool m_sndglu_pend_ram = false;
	bool m_sndglu_pend_auto = false;
	u16 m_sndglu_pend_addr = 0;
	u8 m_sndglu_pend_data = 0;
	emu_timer *m_sndglu_timer = nullptr;
	attotime m_glu_log_at;
	u32 m_glu_ram = 0;
	u32 m_glu_land[32][7]{};
	u32 m_glu_repl[32][7]{};
	static const char *const s_glu_cls[7];
	attotime sndglu_now() const;
	attotime bt_at(u64 t) const;
	bool sndglu_service();
	bool sndglu_service(attotime time);
	void sndglu_ctrl_write(u8 data);
	void sndglu_arm(bool is_read, u8 data);
	void sndglu_reg_write(u8 data);
	void sndglu_commit();
	// $C03D register write judged at the bus time bt_access computes (the
	// CPU core calls the handler first). m_snd_force is that time.
	bool m_glu_hold = false;
	u16 m_glu_hold_reg = 0;
	void sndglu_addr_write(u16 reg, u8 data);
	u8 m_glu_hold_data = 0;
	attotime m_snd_force;
	bool m_snd_force_on = false;
	void sndglu_tally(u16 addr, bool land);
	void sndglu_log_table();
	void sndglu_apply_vca();
	float m_snd_gain_mono = -1.0f, m_snd_gain_side = -1.0f, m_snd_vca = -1.0f;
	TIMER_CALLBACK_MEMBER(sndglu_commit_cb);

	// Optional Sound GLU diagnostics use the transfer model's bus timestamps
	// and latch state. Keep logging passive: it must not schedule transfers
	// or change their deadlines. m_glu_log_enabled gates diagnostic work.
	bool m_glu_log_enabled = false;
	struct glu_log_context
	{
		attotime time;
		int64_t gap = -1;
		u32 pc = 0, cpu_hz = 0, accelerator_hz = 0;
		u8 bank = 0;
		const char *accelerator = "none";
	};
	glu_log_context m_glu_log_context;
	u32 m_glu_log_instruction_pc = 0;
	u8 m_glu_log_access_bank = 0;
	u16 m_glu_log_intended_address = 0;
	u8 m_glu_log_intended_control = 15;
	int64_t m_glu_log_last_cycle = -1;
	u64 m_glu_log_counts[6]{};
	attotime m_glu_log_summary_at;
	void sndglu_log_note(u32 address, int type);
	bool sndglu_log_memory(u32 address, int type, u8 &data);
	std::string sndglu_log_prefix(const glu_log_context &event) const;
	std::string sndglu_log_cpu_state();
	std::string sndglu_log_stack();
	void sndglu_log_event(unsigned kind, std::string details);
	void sndglu_log_access(unsigned port, bool read, u8 data, u8 result = 0);
	void sndglu_log_summary();

	// Key GLU variables
	u8 m_glu_regs[12]{}, m_glu_bus = 0;
	bool m_glu_mcu_read_kgs = false, m_glu_816_read_dstat = false, m_glu_mouse_read_stat = false;
	int m_glu_kbd_y = 0;

	u8 *m_ram_ptr = nullptr;
	unsigned m_ram_size = 0, m_motherboard_ram = 0, m_ghost_mask = 0;
	u8 m_megaii_ram[0x20000]{};  // 128K of "slow RAM" at $E0/0000 FIXME: turn into memory_share_creator

	int m_inh_bank = 0;

	bool m_slot_irq = false;

	double m_x_calibration = 0, m_y_calibration = 0;

	device_a2bus_card_interface *m_slotdevice[8]{};

	u32 m_slow_counter = 0;

	// clock/BRAM
	u8 m_clkdata = 0, m_clock_control = 0;
	void rtc_vgc(int cko);

	void lcrom_update();
	void do_io(int offset);
	u8 read_floatingbus();
	void update_slotrom_banks();
	void lc_update(int offset, bool writing);
	u8 read_slot_rom(int slotbias, int offset);
	void write_slot_rom(int slotbias, int offset, u8 data);
	u8 read_int_rom(int slotbias, int offset);
	void auxbank_update();
	void raise_irq(int irq);
	void lower_irq(int irq);
	void update_speed();
	TIMER_CALLBACK_MEMBER(apply_speed);
	int get_vpos();
	void process_clock();
	void clear_vgcint(u8 data);

	// ZipGS stuff
	bool m_accel_unlocked = false;
	bool m_accel_fast = false;
	bool m_accel_present = false;
	bool m_accel_temp_slowdown = false;
	bool m_accel_warm_boot = false;
	int m_accel_stage = 0;
	u32 m_accel_speed = 0;
	u8 m_accel_slotspk = 0, m_accel_gsxsettings = 0, m_accel_percent = 0;

	// TransWarp GS: the accelerator of the CPU type setting when the
	// "Accelerator" setting is TransWarp GS
	bool m_twgs = false, m_twgs_sel = false, m_twgs_irq_at_reset = true;
	bool m_twgs_real = false;
	u16 m_twgs_arg = 0, m_twgs_result = 0;
	u8 m_twgs_fw[0x200];
	void twgs_build_firmware();
	void twgs_reset_config();
	void twgs_speed_index(int index);
	int twgs_cur_index();
	u16 twgs_index_freq(int index);
	int twgs_freq_index(u16 khz);
	u8 twgs_rom_r(offs_t offset);
	void twgs_rom_w(offs_t offset, u8 data);
	u8 m_twgs_config = 0;       // $BC0000: bit 1 data cache on, bit 2 TransWarp on, bit 3 IRQ slowdown off
	u8 *m_twgs_rom = nullptr;
	u8 m_twgs_cache_data[0x8000]{};
	u32 m_twgs_tag[0x8000]{};
	u8 m_twgs_serial = 0;
	u32 m_twgs_readback = 0;
	bool m_twgs_boot = false;
	// Functional CPU-side mapping state; the XC2064 bitstream is not decoded.
	// The motherboard changes only when its bus transfer completes.
	u32 m_twgs_waddr[4]{};
	u8 m_twgs_wdata[4]{};
	u64 m_twgs_wend[4]{};
	bool m_twgs_wdefer[4]{};
	u8 m_twgs_wcount = 0;
	bool m_twgs_post = false, m_twgs_access_io = false;
	emu_timer *m_twgs_write_timer = nullptr;
	bool twgs_memory(u32 address, int type, u8 &data);
	bool twgs_shadowed(u32 address) const;
	void twgs_drain(unsigned count);
	void twgs_schedule_write();
	void twgs_flush_due();
	TIMER_CALLBACK_MEMBER(twgs_write_complete);
	void twgs_set_config(u8 data);
	u8 twgs_bus(u32 address, int type, u8 data);
	u8 twgs_r(offs_t offset);
	void twgs_w(offs_t offset, u8 data);
	void floatingbank_w(offs_t offset, u8 data);
	memory_passthrough_handler m_accel_tap;

	void accel_reset();
	void accel_temp_delay(int ms, bool condition);
	void accel_stop_delay();
	void accel_slot(int slot);

	// Bus timing model: the FPI fast cycle with DRAM refresh, the Mega II
	// 1 MHz sync, and the ZipGS cache. Time is in units of 1/1056 of a 14M
	// clock, so the fast, Mega II and ZipGS cycles are all whole numbers.
	static constexpr u64 BUS_READ_RESUME = 1; // US 4,794,523 DL1/DL2 synchronizer: retain the one-card-cycle read resume.
	static constexpr int ZIP_WRITE_DEPTH = 1; // US 4,794,523 Table I has one first-stage write latch.
	static constexpr int TWGS_WRITE_LATCHES = 4; // Body schematic and GAL1 decode use all four write-latch columns.
	static constexpr u32 REPSEP_SLOW_CYCLES = 1; // GAL3 OP_SLOW decodes REP/SEP; retain the model's one motherboard cycle.
	static constexpr u64 BT_CLK = 1056;
	static constexpr u64 BT_PER_SEC = 15'120'000'000; // 14.318181 MHz * 1056
	static constexpr u64 BT_FAST = 5 * BT_CLK;
	static constexpr u64 BT_REFRESH_PERIOD = 50 * BT_CLK;
	static constexpr u64 BT_REFRESH = 5 * BT_CLK;
	static constexpr u64 BT_MEGA = 14 * BT_CLK;
	static constexpr u64 BT_MEGA_LAST = 16 * BT_CLK;
	static constexpr u64 BT_LINE = 912 * BT_CLK;
	static constexpr u64 BT_IDLE = 64; // more cycles than one instruction: the CPU waited (WAI)
	static constexpr u64 BT_PAGE_UNIT = 66; // per-page stall counters are in 1/16 of a 14M clock
	enum { BT_OFF = 0, BT_STOCK, BT_ZIP, BT_TWGS, BT_NATIVE };
	enum { BC_RAM = 0, BC_ROM, BC_FASTIO, BC_MEGA };

	// profile region ":zipprof", read by Lua
	static constexpr u32 BTP_COUNT = 0x20;     // u64 counters
	static constexpr u32 BTP_PAGES = 0x100;    // 65536 data pages x u64 reads, bus reads, writes, stall
	// code: 16-byte blocks in banks $00-$05, then 256-byte pages; u64 stall, bus reads, time
	static constexpr u32 BTP_PCS = 0x200100;
	static constexpr u32 BTP_PC_ENTRIES = 0x6000 + 0x10000;
	static constexpr u32 BTP_SIZE = BTP_PCS + BTP_PC_ENTRIES * 24;
	enum { BTC_INSTR = 0, BTC_READS, BTC_HITS, BTC_MISSES, BTC_UNCACHED, BTC_WRITES, BTC_MEGA, BTC_FAST,
		BTC_STALL_READ, BTC_STALL_WRITE, BTC_STALL_INTERNAL, BTC_REFRESH, BTC_TIME,
		BTC_MISS_OPCODE, BTC_MISS_OPERAND, BTC_MISS_DATA, BTC_WBUF_FULL, BTC_COUNT };

	int m_bt_mode = BT_OFF;
	bool m_native_ram = false; // "Native-speed fast RAM" setting, read at reset
	bool m_bt_resync = true, m_bt_slow = false;
	u64 m_bt_cycle = BT_FAST;
	u64 m_bt_istart = 0, m_bt_istall = 0, m_bt_frac = 0, m_bt_cprev = 0;
	u64 m_bt_charged = 0, m_bt_idx = 0;
	u32 m_bt_ipc = 0;       // the code block of the instruction (BTP_PCS entry)
	u64 m_bt_fanchor = 0, m_bt_busfree = 0, m_bt_slot = 0;
	bool m_bt_slot_ok = false;
	u64 m_bt_wbuf[4] = { 0, 0, 0, 0 };
	int m_bt_wcount = 0;
	std::unique_ptr<u32[]> m_zip_tag;
	std::unique_ptr<u8[]> m_zip_data;
	bool m_zip_tag_update = false, m_zip_trash_tag = false;
	// Accelerator-side mapping latches, independent of the FPI and video device.
	bool m_zip_ramrd = false, m_zip_ramwrt = false, m_zip_altzp = false;
	bool m_zip_store80 = false, m_zip_page2 = false, m_zip_hires = false;
	bool m_zip_lcram = false, m_zip_lcram2 = true;
	bool m_zip_iolc = false, m_zip_intcxrom = false;
	bool m_zip_c8_slot = false; // this $C800-$CFFF access is the selected card
	bool m_zip_hit = false;
	bool zip_iolc(u32 address) const;
	void zip_snoop(u32 address, bool write, u8 data);
	bool zip_cacheable(u32 address) const;
	u32 zip_taddr(u32 address, bool write) const;
	u8 zip_access(u32 address, int type, u8 data);
	u32 m_zip_mask = 0x3fff;
	u8 m_zip_size = 1;
	bool m_zip_bank_ok[256];
	u32 m_twgs_mask = 0x1fff;
	bool m_bt_twslow = false;    // this instruction runs at the GS speed (TransWarp GS IRQ slowdown)
	std::unique_ptr<u8[]> m_twgs_data;
	bool m_twgs_80store = false, m_twgs_page2 = false, m_twgs_hires = false;
	bool m_twgs_ramrd = false, m_twgs_ramwrt = false, m_twgs_altzp = false;
	bool m_twgs_lcram = false, m_twgs_lcram2 = true;
	bool m_twgs_lcprewrite = false, m_twgs_lcwrite = true, m_twgs_rombank = false;
	bool m_twgs_iolc = false, m_twgs_linear = false;
	u32 m_twgs_access_tag = 0;
	bool m_twgs_access_cacheable = false, m_twgs_access_hit = false;
	void twgs_reset_switches();
	bool twgs_io_bank(u32 address) const;
	void twgs_snoop(u32 address, bool write, u8 data);
	u32 twgs_taddr(u32 address, int type) const;
	u8 twdc_access(u32 offset, int type, u8 data);
	bool twgs_cached_access(u32 address, int type, u8 &data, int &cycles);
	// The physical card shares one controller/SRAM state between data and
	// timing. Legacy synthetic firmware mode retains its separate interface.
	// This factor represents every supported oscillator's four subclocks
	// exactly (including 50/55/64 MHz), without per-edge attotime arithmetic.
	static constexpr u64 TWGS_TIME_SCALE = 220;
	using twgs_time = twgs_decoded::time_base<BT_PER_SEC * TWGS_TIME_SCALE>;
	static u64 twgs_ticks(attotime const &t) { return twgs_time::ticks(t.seconds(), t.attoseconds()); }
	static attotime twgs_at(u64 t) { return attotime(t / (BT_PER_SEC * TWGS_TIME_SCALE), twgs_time::fraction(t)); }
	twgs_decoded::timing_scaled<TWGS_TIME_SCALE> m_twgs_kernel;
	bool m_twgs_kernel_on = false, m_twgs_kernel_started = false;
	u64 m_twgs_transaction = 0, m_twgs_retired = 0, m_twgs_visible = 0;
	u64 m_twgs_unemitted_cycles = 0;
	s64 m_twgs_cycle_charge = 0;
	struct twgs_board
	{
		apple2gs_state &owner;
		void read(twgs_decoded::transaction &w, u64 begin, u64 end) { w.data = owner.twgs_board_read(w, begin, end); }
		void write(twgs_decoded::transaction const &w, u64 begin, u64 end) { owner.twgs_board_write(w, begin, end); }
	};
	u8 twgs_board_read(twgs_decoded::transaction const &w, u64 begin, u64 end);
	void twgs_board_write(twgs_decoded::transaction const &w, u64 begin, u64 end);
	void twgs_kernel_cycle(u32 address, int type, u8 &data);
	int twgs_kernel_access(u32 address, int type, u8 &data);
	int twgs_internal(u32 address, int type, u8 data);
	u64 twgs_kernel_instruction(u32 pc);
	void twgs_kernel_schedule();
	u8 *m_btprof = nullptr;
	u64 *m_btc = nullptr;
	u64 *m_btpage = nullptr, *m_btpc = nullptr;

	int bt_access(u32 address, int type, u8 data);
	u64 bt_instruction(u32 pc);
	u64 bt_fast(u64 t, bool refresh);
	u64 bt_mega_start(u64 t) const;
	u64 bt_mega(u64 t);
	u64 bt_bus(u64 t, int cls);
	bool bt_aux(u32 a16, bool write);
	void bt_config();
	void bt_setup(u32 speed);
	void bt_control();
};

// FF6ACF is speed test routine in ROM 3

// slow_cycle() - take a 1 MHz cycle.  Theory: a 2.8 MHz cycle is 14M / 5.
// 1 MHz is 14M / 14.  14/5 = 2.8 * 65536 (16.16 fixed point) = 0x2cccd.
#define slow_cycle() \
{   \
	if (m_bt_mode != BT_OFF) \
	{\
		if (!machine().side_effects_disabled()) \
			m_bt_slow = true; \
	}\
	else if (m_last_speed && !machine().side_effects_disabled()) \
	{\
		m_slow_counter += 0x0002cccd; \
		int cycles = (m_slow_counter >> 16) & 0xffff; \
		m_slow_counter &= 0xffff; \
		m_maincpu->adjust_icount(-cycles); \
	} \
}

offs_t apple2gs_state::dasm_trampoline(std::ostream &stream, offs_t pc, const util::disasm_interface::data_buffer &opcodes, const util::disasm_interface::data_buffer &params)
{
	return m_a2common->dasm_override_GS(stream, pc, opcodes, params);
}

void apple2gs_state::a2bus_irq_w(int state)
{
	if (state == ASSERT_LINE)
	{
		raise_irq(IRQS_SLOT);
		m_slot_irq = true;
	}
	else
	{
		lower_irq(IRQS_SLOT);
	}
}

void apple2gs_state::a2bus_nmi_w(int state)
{
	m_maincpu->set_input_line(INPUT_LINE_NMI, state);
}

// TODO: this assumes /INH only on ROM, needs expansion to support e.g. phantom-slotting cards and etc.
void apple2gs_state::a2bus_inh_w(int state)
{
	if (state == ASSERT_LINE)
	{
		// assume no cards are pulling /INH
		m_inh_slot = -1;

		// scan the slots to figure out which card(s) are INHibiting stuff
		for (int i = 0; i <= 7; i++)
		{
			if (m_slotdevice[i])
			{
				// this driver only can inhibit from 0xd000-0xffff
				if ((m_slotdevice[i]->inh_start() == 0xd000) &&
					(m_slotdevice[i]->inh_end() == 0xffff))
				{
					if ((m_slotdevice[i]->inh_type() & INH_READ) == INH_READ)
					{
						if (m_inh_bank != 1)
						{
							m_upperbank.select(1);
							m_upperaux.select(1);
							m_upper00.select(1);
							m_upper01.select(1);
							m_inh_bank = 1;
						}
					}
					else
					{
						if (m_inh_bank != 0)
						{
							m_upperbank.select(0);
							m_upperaux.select(0);
							m_upper00.select(0);
							m_upper01.select(0);
							m_inh_bank = 0;
						}
					}

					m_inh_slot = i;
					break;
				}
			}
		}

		// if no slots are inhibiting, make sure ROM is fully switched in
		if ((m_inh_slot == -1) && (m_inh_bank != 0))
		{
			m_upperbank.select(0);
			m_upperaux.select(0);
			m_upper00.select(0);
			m_upper01.select(0);
			m_inh_bank = 0;
		}
	}
}

// FPI/CYA chip is connected to the VPB output of the 65816.
// this facilitates the documented behavior from the Firmware Reference.
u8 apple2gs_state::apple2gs_read_vector(offs_t offset)
{
	if (m_twgs_real && m_twgs && m_twgs_boot && ((offset & 0x1e) == 0x1c))
		return m_twgs_rom[0x7fe0 | offset];

	// when IOLC shadowing is enabled, vector fetches always go to ROM,
	// regardless of the language card config.
	if (!(m_shadow & SHAD_IOLC))
	{
		if (m_inh_slot != -1 && (m_slotdevice[m_inh_slot]->inh_type() & INH_READ) == INH_READ)
			return m_slotdevice[m_inh_slot]->read_inh_rom(offset | 0xFFE0);
		else
			return m_maincpu->space(AS_PROGRAM).read_byte(offset | 0xFFFFE0);
	}
	else    // else vector fetches from bank 0 RAM
	{
		return m_maincpu->space(AS_PROGRAM).read_byte((offset & 0xffff) | 0xFFE0);
	}
}

/***************************************************************************
    START/RESET
***************************************************************************/

void apple2gs_state::machine_start()
{
	m_ram_ptr = m_ram->pointer();
	m_ram_size = m_ram->size();

	m_zip_tag = std::make_unique<u32[]>(0x10000);
	m_zip_data = std::make_unique<u8[]>(0x10000);
	m_twgs_data = std::make_unique<u8[]>(0x10000);
	m_twgs_write_timer = timer_alloc(FUNC(apple2gs_state::twgs_write_complete), this);
	m_sndglu_timer = timer_alloc(FUNC(apple2gs_state::sndglu_commit_cb), this);
	m_glu_log_enabled = (VERBOSE & (LOG_GLU | LOG_GLUTRACE)) && machine().allow_logging();
	m_doc->set_host_sync([this](attotime time) { sndglu_service(time); });
	m_a2bus->set_dma_sync([this]() { twgs_flush_due(); });
	twgs_build_firmware();
	m_twgs_rom = memregion("twgs")->base();
	memory_region *prof = machine().memory().region_alloc(":zipprof", BTP_SIZE, 1, ENDIANNESS_LITTLE);
	m_btprof = prof->base();
	std::fill_n(m_btprof, BTP_SIZE, 0);
	m_btc = reinterpret_cast<u64 *>(m_btprof + BTP_COUNT);
	m_btpage = reinterpret_cast<u64 *>(m_btprof + BTP_PAGES);
	m_btpc = reinterpret_cast<u64 *>(m_btprof + BTP_PCS);
	m_maincpu->set_data_hook(g65816_device::data_hook_delegate(&apple2gs_state::twdc_access, this));
	machine().add_notifier(MACHINE_NOTIFY_EXIT, machine_notify_delegate([this]() {
		sndglu_log_table();
	}));
	m_maincpu->set_bus_hook(g65816_device::bus_hook_delegate(&apple2gs_state::bt_access, this));
	m_speaker_state = 0;
	m_speaker->level_w(m_speaker_state);
	m_upperbank.select(0);
	m_upperaux.select(0);
	m_upper00.select(0);
	m_upper01.select(0);
	m_lcbank.select(0);
	m_lcaux.select(0);
	m_lc00.select(0);
	m_lc01.select(0);
	m_b0_0000bank.select(0);
	m_e0_0000bank.select(0);
	m_b0_0200bank.select(0);
	m_e0_0200bank.select(0);
	m_b0_0400bank.select(0);
	m_e0_0400bank.select(0);
	m_b0_0800bank.select(0);
	m_e0_0800bank.select(0);
	m_b0_2000bank.select(0);
	m_e0_2000bank.select(0);
	m_b0_4000bank.select(0);
	m_e0_4000bank.select(0);
	m_inh_bank = 0;
	std::fill(std::begin(m_megaii_ram), std::end(m_megaii_ram), 0);

	std::fill(std::begin(m_glu_regs), std::end(m_glu_regs), 0);

	// The one-bit speaker is full scale into the summer. The $C03C nibble is the
	// VCA after that sum, applied in sndglu_apply_vca().
	static const double click[2] = { 0.0, 1.0 };
	m_speaker->set_levels(2, click);

	// precalculate joystick time constants
	m_x_calibration = attotime::from_nsec(10800).as_double();
	m_y_calibration = attotime::from_nsec(10800).as_double();

	// cache slot devices
	for (int i = 0; i <= 7; i++)
	{
		m_slotdevice[i] = m_a2bus->get_a2bus_card(i);
	}

	// setup video pointers
	m_video->set_ram_pointers(m_megaii_ram, &m_megaii_ram[0x10000]);
	m_video->set_char_pointer(memregion("gfx1")->base(), memregion("gfx1")->bytes());
	m_video->setup_GS_graphics();

	m_video->set_GS_textcol(0xf2);

	// Mega II events occur near the rightmost edge of active video
	m_scantimer->adjust(m_screen->time_until_pos(0, HPOS_VBL));
	// VGC IRQs occur slightly later during HBL
	m_vgctimer->adjust(m_screen->time_until_pos(0, HPOS_VGC));

	m_inh_slot = -1;
	m_cnxx_slot = CNXX_UNCLAIMED;

	// install ROM
	address_space &space = m_maincpu->space(AS_PROGRAM);
	if (m_is_rom3)
		space.install_rom(0xfc0000, 0xffffff, m_rom);

	// adjust RAM size
	if (!m_is_rom3 && (m_ram_size <= 1280 * 1024 || m_ram_size == 4352 * 1024))
	{
		m_ram_size -= 0x020000; // subtract 128k so requested RAM size matches exactly
	}
	// otherwise, RAM sizes for both classes of machine no longer include the Mega II RAM

	// install "fast" RAM beyond banks 0, 1
	m_motherboard_ram = m_is_rom3 ? 0x100000 : 0x020000;
	if (m_ram_size > 0x020000)
	{
		space.install_ram(0x020000, m_ram_size - 1, m_ram_ptr + 0x020000);
		space.install_write_handler(0x020000, m_ram_size - 1, write8sm_delegate(*this, FUNC(apple2gs_state::expansion_ram_w)));

		if (m_ram_size > m_motherboard_ram)
		{
			// unmap empty RAM banks in case of non-power-of-two expansion
			if (m_is_rom3 && m_ram_size < 0x800000)
				space.nop_read(m_ram_size, 0x7fffff);

			// expansion RAM ghosts power-of-two banks up through 7f
			m_ghost_mask = (1 << std::bit_width(m_ram_size - m_motherboard_ram - 1)) - 1;
			const int ghost_start = m_motherboard_ram + m_ghost_mask + 1;
			if (ghost_start < 0x800000)
			{
				space.install_readwrite_handler(ghost_start, 0x7fffff,
					 read8sm_delegate(*this, FUNC(apple2gs_state::ghostram_r)),
					write8sm_delegate(*this, FUNC(apple2gs_state::ghostram_w)));
			}

			// unmap empty ROM banks
			if (m_is_rom3) // ROM1 reads floating bus
				space.nop_read(0xf00000, 0xfbffff);
		}
	}

	// setup save states
	save_item(NAME(m_speaker_state));
	save_item(NAME(m_joystick_x1_time));
	save_item(NAME(m_joystick_y1_time));
	save_item(NAME(m_joystick_x2_time));
	save_item(NAME(m_joystick_y2_time));
	save_item(NAME(m_inh_slot));
	save_item(NAME(m_inh_bank));
	save_item(NAME(m_cnxx_slot));
	save_item(NAME(m_an0));
	save_item(NAME(m_an1));
	save_item(NAME(m_an2));
	save_item(NAME(m_an3));
	save_item(NAME(m_intcxrom));
	save_item(NAME(m_slotc3rom));
	save_item(NAME(m_intc8rom));
	save_item(NAME(m_altzp));
	save_item(NAME(m_ramrd));
	save_item(NAME(m_ramwrt));
	save_item(NAME(m_vbl));
	save_item(NAME(m_irqmask));
	save_item(NAME(m_lcram));
	save_item(NAME(m_lcram2));
	save_item(NAME(m_lcprewrite));
	save_item(NAME(m_lcwriteenable));
	save_item(NAME(m_rombank));
	save_item(NAME(m_shadow));
	save_item(NAME(m_speed));
	save_item(NAME(m_clock_control));
	save_item(NAME(m_clkdata));
	save_item(NAME(m_motors_active));
	save_item(NAME(m_slotromsel));
	save_item(NAME(m_diskreg));
	save_item(NAME(m_sndglu_ctrl));
	save_item(NAME(m_sndglu_addr));
	save_item(NAME(m_sndglu_dummy_read));
	save_item(NAME(m_sndglu_busy_until));
	save_item(NAME(m_sndglu_pending));
	save_item(NAME(m_sndglu_pend_read));
	save_item(NAME(m_sndglu_pend_ram));
	save_item(NAME(m_sndglu_pend_auto));
	save_item(NAME(m_sndglu_pend_addr));
	save_item(NAME(m_sndglu_pend_data));
	save_item(NAME(m_last_speed));
	save_item(NAME(m_glu_regs));
	save_item(NAME(m_glu_bus));
	save_item(NAME(m_glu_mcu_read_kgs));
	save_item(NAME(m_glu_816_read_dstat));
	save_item(NAME(m_glu_mouse_read_stat));
	save_item(NAME(m_glu_kbd_y));
	save_item(NAME(m_j13_config));
	machine().save().register_postload(save_prepost_delegate(FUNC(apple2gs_state::j13_config_postload), this));
	save_item(NAME(m_intflag));
	save_item(NAME(m_vgcint));
	save_item(NAME(m_inten));
	save_item(NAME(m_slot_irq));
	save_item(NAME(m_slow_counter));
	save_item(NAME(m_megaii_ram));
	save_item(m_clkdata, "CLKDATA");
	save_item(m_clock_control, "CLKCTRL");
	save_item(NAME(m_adb_p2_last));
	save_item(NAME(m_adb_p3_last));
	save_item(NAME(m_adb_reset_freeze));
	save_item(NAME(m_ram_initialized));
	save_item(NAME(m_accel_unlocked));
	save_item(NAME(m_accel_stage));
	save_item(NAME(m_accel_fast));
	save_item(NAME(m_accel_present));
	save_item(NAME(m_accel_slotspk));
	save_item(NAME(m_accel_gsxsettings));
	save_item(NAME(m_accel_percent));
	save_item(NAME(m_accel_temp_slowdown));
	save_item(NAME(m_accel_warm_boot));
	save_item(NAME(m_accel_speed));
	save_item(NAME(m_motoroff_time));

	// the bus timing model: a run resumed from a state has the same cache, write buffer, time
	// base and counters as the run that saved it
	save_item(NAME(m_bt_mode));
	save_item(NAME(m_native_ram));
	save_item(NAME(m_bt_resync));
	save_item(NAME(m_bt_slow));
	save_item(NAME(m_bt_cycle));
	save_item(NAME(m_bt_istart));
	save_item(NAME(m_bt_istall));
	save_item(NAME(m_bt_frac));
	save_item(NAME(m_bt_cprev));
	save_item(NAME(m_bt_charged));
	save_item(NAME(m_bt_idx));
	save_item(NAME(m_bt_ipc));
	save_item(NAME(m_bt_fanchor));
	save_item(NAME(m_bt_busfree));
	save_item(NAME(m_bt_wbuf));
	save_item(NAME(m_bt_wcount));
	save_item(NAME(m_bt_twslow));
	save_item(NAME(m_zip_mask));
	save_item(NAME(m_zip_size));
	save_item(NAME(m_zip_bank_ok));
	// Decoded card state: every physical register, tag and holding latch.
	save_item(NAME(m_twgs_kernel_on));
	save_item(NAME(m_twgs_kernel_started));
	save_item(NAME(m_twgs_transaction));
	save_item(NAME(m_twgs_retired));
	save_item(NAME(m_twgs_visible));
	save_item(NAME(m_twgs_unemitted_cycles));
	save_item(NAME(m_twgs_cycle_charge));
	save_item(NAME(m_twgs_kernel.now));
	save_item(NAME(m_twgs_kernel.early));
	save_item(NAME(m_twgs_kernel.late));
	save_item(NAME(m_twgs_kernel.gs_event));
	save_item(NAME(m_twgs_kernel.gs_start));
	save_item(NAME(m_twgs_kernel.x4));
	save_item(NAME(m_twgs_kernel.slot_start));
	save_item(NAME(m_twgs_kernel.slot_end));
	save_item(NAME(m_twgs_kernel.access_slot_start));
	save_item(NAME(m_twgs_kernel.access_slot_end));
	save_item(NAME(m_twgs_kernel.board_cycles));
	save_item(NAME(m_twgs_kernel.mega_cycles));
	save_item(NAME(m_twgs_kernel.refresh_cycles));
	save_item(NAME(m_twgs_kernel.speed));
	save_item(NAME(m_twgs_kernel.shadow));
	save_item(NAME(m_twgs_kernel.next_gs_level));
	save_item(NAME(m_twgs_kernel.had_bus));
	save_item(NAME(m_twgs_kernel.fast_hits));
	save_item(NAME(m_twgs_kernel.c.ctl.q1));
	save_item(NAME(m_twgs_kernel.c.ctl.q2));
	save_item(NAME(m_twgs_kernel.c.ctl.gs));
	save_item(NAME(m_twgs_kernel.c.ctl.early));
	save_item(NAME(m_twgs_kernel.c.ctl.delayed));
	save_item(NAME(m_twgs_kernel.c.ctl.service));
	save_item(NAME(m_twgs_kernel.c.ctl.pause_sample));
	save_item(NAME(m_twgs_kernel.c.ctl.sync));
	save_item(NAME(m_twgs_kernel.c.ctl.r16_sample));
	save_item(NAME(m_twgs_kernel.c.ctl.r17_sample));
	save_item(NAME(m_twgs_kernel.c.ctl.read));
	save_item(NAME(m_twgs_kernel.c.ctl.miss));
	save_item(NAME(m_twgs_kernel.c.ctl.eligible));
	save_item(NAME(m_twgs_kernel.c.ctl.slow));
	save_item(NAME(m_twgs_kernel.c.ctl.pause));
	save_item(NAME(m_twgs_kernel.c.ctl.use_gal2a));
	save_item(NAME(m_twgs_kernel.c.ctl.pin7_card_phase));
	save_item(NAME(m_twgs_kernel.c.decode.g3));
	save_item(NAME(m_twgs_kernel.c.decode.g4));
	save_item(NAME(m_twgs_kernel.c.decode.gal3_revision));
	save_item(NAME(m_twgs_kernel.c.decode.gal4_b));
	save_item(NAME(m_twgs_kernel.c.decode.gal5_permuted));
	save_item(NAME(m_twgs_kernel.c.decode.use_tables));
	save_item(NAME(m_twgs_kernel.c.fpga.q));
	save_item(NAME(m_twgs_kernel.c.tags));
	save_item(NAME(m_twgs_kernel.c.bytes));
	save_item(NAME(m_twgs_kernel.c.valid));
	save_item(NAME(m_twgs_kernel.c.active_column));
	save_item(NAME(m_twgs_kernel.c.bank_column));
	save_item(NAME(m_twgs_kernel.c.mask));
	save_item(NAME(m_twgs_kernel.c.fp));
	save_item(NAME(m_twgs_kernel.c.comparator_enable));
	save_item(NAME(m_twgs_kernel.c.cpu));
	save_item(NAME(m_twgs_kernel.c.we));
	save_item(NAME(m_twgs_kernel.c.forced_write));
	save_item(NAME(m_twgs_kernel.c.bank_valid));
	save_item(NAME(m_twgs_kernel.c.completions));
	save_item(NAME(m_twgs_kernel.c.deliveries));
	save_item(NAME(m_twgs_kernel.c.strobe_changes));
	save_item(NAME(m_twgs_kernel.c.bank_errors));
	save_item(NAME(m_twgs_kernel.c.high_errors));
	save_item(NAME(m_twgs_kernel.c.completed));
	save_item(NAME(m_twgs_kernel.c.delivered));
	save_item(NAME(m_twgs_kernel.c.from_sram));
	save_item(NAME(m_twgs_kernel.c.external_advance));
	save_item(NAME(m_twgs_kernel.c.receipt));
	// A load can resume a held-bus completion before setword recomputes these.
	save_item(NAME(m_twgs_kernel.c.sample_row));
	save_item(NAME(m_twgs_kernel.c.raw_match));
	save_item(NAME(m_twgs_kernel.c.pins.tag));
	save_item(NAME(m_twgs_kernel.c.pins.eligible));
	save_item(NAME(m_twgs_kernel.c.pins.bank_io));
	save_item(NAME(m_twgs_kernel.c.pins.irq));
	save_item(NAME(m_twgs_kernel.c.pins.fast_instruction));
	save_item(NAME(m_twgs_kernel.c.pins.gal3_pins));
	save_item(NAME(m_twgs_kernel.c.word.address));
	save_item(NAME(m_twgs_kernel.c.word.id));
	save_item(NAME(m_twgs_kernel.c.word.data));
	save_item(NAME(m_twgs_kernel.c.word.type));
	save_item(NAME(m_twgs_kernel.c.word.seed));
	save_item(NAME(m_twgs_kernel.c.active.bus.address));
	save_item(NAME(m_twgs_kernel.c.active.bus.id));
	save_item(NAME(m_twgs_kernel.c.active.bus.data));
	save_item(NAME(m_twgs_kernel.c.active.bus.type));
	save_item(NAME(m_twgs_kernel.c.active.bus.seed));
	save_item(NAME(m_twgs_kernel.c.bank_candidate.bus.address));
	save_item(NAME(m_twgs_kernel.c.bank_candidate.bus.id));
	save_item(NAME(m_twgs_kernel.c.bank_candidate.bus.data));
	save_item(NAME(m_twgs_kernel.c.bank_candidate.bus.type));
	save_item(NAME(m_twgs_kernel.c.bank_candidate.bus.seed));
	save_item(NAME(m_twgs_kernel.c.retired.bus.address));
	save_item(NAME(m_twgs_kernel.c.retired.bus.id));
	save_item(NAME(m_twgs_kernel.c.retired.bus.data));
	save_item(NAME(m_twgs_kernel.c.retired.bus.type));
	save_item(NAME(m_twgs_kernel.c.retired.bus.seed));
	save_item(NAME(m_twgs_kernel.c.active.byte));
	save_item(NAME(m_twgs_kernel.c.active.valid));
	save_item(NAME(m_twgs_kernel.c.bank_candidate.byte));
	save_item(NAME(m_twgs_kernel.c.bank_candidate.valid));
	save_item(NAME(m_twgs_kernel.c.retired.byte));
	save_item(NAME(m_twgs_kernel.c.retired.valid));
	for (unsigned i=0;i<4;++i)
	{
		save_item(NAME(m_twgs_kernel.c.latches[i].byte), i);
		save_item(NAME(m_twgs_kernel.c.latches[i].valid), i);
		save_item(NAME(m_twgs_kernel.c.latches[i].bus.address), i);
		save_item(NAME(m_twgs_kernel.c.latches[i].bus.id), i);
		save_item(NAME(m_twgs_kernel.c.latches[i].bus.data), i);
		save_item(NAME(m_twgs_kernel.c.latches[i].bus.type), i);
		save_item(NAME(m_twgs_kernel.c.latches[i].bus.seed), i);
	}
	save_item(NAME(m_twgs_mask));
	save_item(NAME(m_twgs));
	save_item(NAME(m_twgs_sel));
	save_item(NAME(m_twgs_config));
	save_item(NAME(m_twgs_real));
	save_item(NAME(m_twgs_irq_at_reset));
	save_item(NAME(m_twgs_arg));
	save_item(NAME(m_twgs_result));
	save_item(NAME(m_twgs_cache_data));
	save_item(NAME(m_twgs_tag));
	save_item(NAME(m_twgs_serial));
	save_item(NAME(m_twgs_readback));
	save_item(NAME(m_twgs_boot));
	save_pointer(NAME(m_zip_tag), 0x10000);
	save_item(NAME(m_zip_tag_update));
	save_item(NAME(m_zip_trash_tag));
	save_pointer(NAME(m_twgs_data), 0x10000);
	save_item(NAME(m_twgs_80store));
	save_item(NAME(m_twgs_page2));
	save_item(NAME(m_twgs_hires));
	save_item(NAME(m_twgs_waddr));
	save_item(NAME(m_twgs_wdata));
	save_item(NAME(m_twgs_wend));
	save_item(NAME(m_twgs_wdefer));
	save_item(NAME(m_twgs_wcount));
	save_item(NAME(m_twgs_post));
	save_item(NAME(m_twgs_access_io));

	save_pointer(NAME(m_zip_data), 0x10000);
	save_item(NAME(m_zip_ramrd));
	save_item(NAME(m_zip_ramwrt));
	save_item(NAME(m_zip_altzp));
	save_item(NAME(m_zip_store80));
	save_item(NAME(m_zip_page2));
	save_item(NAME(m_zip_hires));
	save_item(NAME(m_zip_lcram));
	save_item(NAME(m_zip_lcram2));
	save_item(NAME(m_zip_iolc));
	save_item(NAME(m_zip_intcxrom));
	save_item(NAME(m_zip_c8_slot));
	save_item(NAME(m_zip_hit));

	save_item(NAME(m_twgs_ramrd));
	save_item(NAME(m_twgs_ramwrt));
	save_item(NAME(m_twgs_altzp));
	save_item(NAME(m_twgs_lcram));
	save_item(NAME(m_twgs_lcram2));
	save_item(NAME(m_twgs_lcprewrite));
	save_item(NAME(m_twgs_lcwrite));
	save_item(NAME(m_twgs_rombank));
	save_item(NAME(m_twgs_iolc));
	save_item(NAME(m_twgs_linear));
	save_item(NAME(m_twgs_access_tag));
	save_item(NAME(m_twgs_access_cacheable));
	save_item(NAME(m_twgs_access_hit));
	save_pointer(NAME(m_btprof), BTP_SIZE);
}

void apple2gs_state::machine_reset()
{
	m_j13_config = m_j13_config_port.read_safe(0);
	m_zip_tag_update = m_zip_trash_tag = false;
	bool const cold_start = !m_ram_initialized;
	// Configuration is loaded after machine_start.  Initialise physical RAM only
	// on power-on; RESET retains its contents.  DRAM does not have a reset input.
	// The default approximates address/bit-lane bias with some unstable cells.
	// Apple II and C64 drivers also use address-dependent startup contents;
	// these parameters are a generic approximation, not a measured IIgs dump.
	if (!m_ram_initialized)
	{
		unsigned const mode = ioport("ram_start")->read();
		u32 state = 1; // Fixed seed preserves the DRAM power-on pattern across runs.
		auto next = [&state] ()
		{
			state ^= state << 13;
			state ^= state >> 17;
			state ^= state << 5;
			return state;
		};
		auto fill = [&] (u8 *base, u32 size)
		{
			u8 bias = 0, row = 0, column = 0;
			for (u32 i = 0; i < size; i++)
			{
				if (!(i & 0xffff))
				{
					bias = u8(next());
					row = u8(next());
					column = u8(next());
				}
				u32 const bits = next();
				base[i] = bias ^ (BIT(i, 7) ? row : 0) ^ (BIT(i, 0) ? column : 0) ^ u8(bits & (bits >> 8) & (bits >> 16));
			}
		};
		if (mode)
		{
			fill(m_ram_ptr, m_ram->size());
			fill(m_megaii_ram, sizeof(m_megaii_ram));
			// The sound bank is also DRAM (two 64K x 4 parts on both boards).
			// It has no power-on clear. Apply the same selected approximation;
			// firmware fills only the addresses its SoundBootInit loop visits.
			fill(&m_docram[0], m_docram.bytes());
		}
		m_ram_initialized = true;
	}

	m_adb_p2_last = m_adb_p3_last = 0;
	m_adb_reset_freeze = 0;
	m_video->scr_w(0);
	m_video->set_GS_border(0x02);
	m_video->set_GS_textcol(0xf2);
	m_an0 = m_an1 = m_an2 = m_an3 = false;
	m_gameio->an0_w(0);
	m_gameio->an1_w(0);
	m_gameio->an2_w(0);
	m_gameio->an3_w(0);
	m_vbl = false;
	m_irqmask = 0;
	m_intcxrom = false;
	m_slotc3rom = false;
	m_intc8rom = false;
	m_video->a80store_w(false);
	m_altzp = false;
	m_ramrd = false;
	m_ramwrt = false;
	m_rombank = false;
	m_video->set_newvideo(0x01); // verified on ROM03 hardware
	m_slot_irq = false;
	m_clkdata = 0;
	m_clock_control = 0;

	m_shadow = 0x00;
	// ROM 03 tests C036 bit 6 at FF:7FF6 and clears it at FF:801A.
	// A keyboard reset must not assert the power-on latch again.
	m_speed = SPEED_HIGH | ((cold_start && m_is_rom3) ? SPEED_POWERON : 0);
	m_motors_active = 0;
	m_diskreg = 0;
	m_intflag = 0;
	m_vgcint = 0;
	m_inten = 0;

	m_motoroff_time = 0;

	m_slow_counter = 0;

	// always assert full speed on reset
	m_maincpu->set_unscaled_clock(A2GS_2_8M);
	m_last_speed = true;
	m_zip_ramrd = m_zip_ramwrt = m_zip_altzp = false;
	m_zip_store80 = m_zip_page2 = m_zip_hires = false;
	m_zip_lcram = false;
	m_zip_lcram2 = true;
	m_zip_iolc = m_zip_intcxrom = m_zip_c8_slot = false;
	m_zip_hit = false;
	std::fill_n(m_zip_data.get(), 0x10000, 0);
	bt_config();
	bt_setup(A2GS_2_8M.value());

	m_sndglu_ctrl = 0x0f; // power-on VCA step; firmware may replace it
	m_sndglu_addr = 0;
	m_sndglu_dummy_read = 0;
	m_sndglu_busy_until = attotime::zero;
	m_sndglu_pending = false;
	m_sndglu_pend_read = false;
	m_sndglu_pend_ram = false;
	m_sndglu_pend_auto = false;
	m_sndglu_pend_addr = 0;
	m_sndglu_pend_data = 0;
	m_sndglu_timer->adjust(attotime::never);
	m_glu_log_at = attotime::from_seconds(10);
	m_glu_ram = 0;
	std::fill(&m_glu_land[0][0], &m_glu_land[0][0] + 32 * 7, 0);
	std::fill(&m_glu_repl[0][0], &m_glu_repl[0][0] + 32 * 7, 0);
	m_snd_demux = ioport("snd_demux")->read() != 0;
	m_doc->set_output_filter_enabled(!m_snd_demux);
	m_doc->set_volume_knee(ioport("doc_volume_law")->read() != 0);
	m_snd_vca = -1.0f;
	sndglu_apply_vca();

	m_b0_0000bank.select(0);
	m_e0_0000bank.select(0);
	m_b0_0200bank.select(0);
	m_e0_0200bank.select(0);
	m_b0_0400bank.select(0);
	m_e0_0400bank.select(0);
	m_b0_0800bank.select(0);
	m_e0_0800bank.select(0);
	m_b0_2000bank.select(0);
	m_e0_2000bank.select(0);
	m_b0_4000bank.select(0);
	m_e0_4000bank.select(0);
	m_bank0_atc.select(1);
	m_bank1_atc.select(1);

	// LC default state: read ROM, write enabled, Dxxx bank 2
	m_lcram = false;
	m_lcram2 = true;
	m_lcprewrite = false;
	m_lcwriteenable = true;

	// sync up the banking with the variables.
	// RESEARCH: how does RESET affect LC state and aux banking states?
	auxbank_update();
	update_slotrom_banks();

	// Apple-specific initial state
	m_scc->ctsa_w(0);
	m_scc->dcda_w(0);

	// The real ROM takes over reset after the accelerator is configured.
	m_twgs_boot = false;
	m_twgs_iolc = m_twgs_linear = false;
	m_twgs_80store = m_twgs_page2 = m_twgs_hires = false;
	m_twgs_wcount = 0;
	m_twgs_post = false;
	m_twgs_write_timer->adjust(attotime::never);
	m_maincpu->set_memory_hook(g65816_device::memory_hook_delegate());
	m_maincpu->set_cached_access(g65816_device::cached_access_delegate());
	m_maincpu->set_internal_hook(g65816_device::internal_hook_delegate());
	m_twgs_kernel_on = false;
	m_maincpu->set_data_hook(g65816_device::data_hook_delegate(&apple2gs_state::twdc_access, this));
	if (!m_twgs_real)
		m_maincpu->reset();

	// Setup ZipGS
	m_accel_present = false;
	m_accel_unlocked = false;
	m_accel_temp_slowdown = false;
	m_accel_fast = false;
	m_accel_warm_boot = false;

	// is Zip enabled?
	if (m_sysconfig->read() & 0x01)
	{
		static const int speeds[4] = { 7000000, 8000000, 12000000, 16000000 };
		m_accel_present = true;
		int idxSpeed = (m_sysconfig->read() >> 1);
		m_accel_speed = speeds[idxSpeed];
		// the clocks of upgraded cards (oscillator / 4)
		static const u32 clocks[8] = { 0, 12500000, 13750000, 14000000, 15000000, 10000000, 0, 0 };
		const u32 clock = clocks[(m_btconfig->read() >> 16) & 7];
		if (clock)
			m_accel_speed = clock;
		accel_reset();
	}

	// TransWarp GS: the same speeds, no Zip registers and no Zip delays
	m_twgs = m_accel_present && m_twgs_sel && (!m_twgs_real || !memcmp(m_twgs_rom + 0x7f00, "TWGS", 4));
	if (m_accel_present && m_twgs_sel && !m_twgs)
	{
		m_accel_present = false;
		m_accel_fast = false;
		update_speed();
		osd_printf_warning("TransWarp GS (partial) requires twgs_1.8s.bin; accelerator disabled\n");
	}
	std::fill_n(m_twgs_data.get(), 0x10000, 0);
	m_twgs_iolc = m_twgs_linear = false;
	twgs_reset_switches();
	if (m_twgs)
	{
		m_accel_unlocked = false;
		m_accel_gsxsettings = 0;
		m_accel_slotspk = 0;
		m_accel_percent = 0;
		if (m_twgs_real)
		{
			std::fill(std::begin(m_twgs_tag), std::end(m_twgs_tag), 0);
			m_twgs_serial = 0;
			m_twgs_nvram->cs_write(0);
			m_twgs_boot = true;
			twgs_set_config(0x06);
		}
		else
			twgs_reset_config();
	}
	if (m_twgs_real)
	{
		m_twgs_kernel_on = m_twgs;
		if (cold_start)
		{
			m_twgs_kernel = twgs_decoded::timing_scaled<TWGS_TIME_SCALE>{};
			m_twgs_kernel_started = false;
			m_twgs_transaction = m_twgs_retired = m_twgs_visible = 0;
			m_twgs_unemitted_cycles = 0;
			m_twgs_cycle_charge = 0;
		}
		m_maincpu->set_memory_hook(m_twgs ? g65816_device::memory_hook_delegate(&apple2gs_state::twgs_memory, this) : g65816_device::memory_hook_delegate());
		m_maincpu->set_cached_access(m_twgs ? g65816_device::cached_access_delegate(&apple2gs_state::twgs_cached_access, this) : g65816_device::cached_access_delegate());
		m_maincpu->set_internal_hook(m_twgs_kernel_on ? g65816_device::internal_hook_delegate(&apple2gs_state::twgs_internal, this) : g65816_device::internal_hook_delegate());
		m_maincpu->set_data_hook(g65816_device::data_hook_delegate(&apple2gs_state::twdc_access, this));
		update_speed();
		m_maincpu->reset();
	}
	if (!m_accel_present)
		m_maincpu->set_data_hook(g65816_device::data_hook_delegate());
	if (m_glu_log_enabled)
	{
		m_glu_log_context = glu_log_context{};
		m_glu_log_instruction_pc = 0;
		m_glu_log_access_bank = 0;
		m_glu_log_intended_address = m_sndglu_addr;
		m_glu_log_intended_control = m_sndglu_ctrl;
		m_glu_log_last_cycle = -1;
		std::fill(std::begin(m_glu_log_counts), std::end(m_glu_log_counts), 0);
		m_glu_log_summary_at = machine().time() + attotime::from_seconds(10);
		m_maincpu->set_memory_hook(g65816_device::memory_hook_delegate(&apple2gs_state::sndglu_log_memory, this));
	}
}

void apple2gs_state::raise_irq(int irq)
{
	if (!(m_irqmask & (1 << irq)))
	{
		m_irqmask |= (1 << irq);

		if (!(m_intflag & INTFLAG_IRQASSERTED))
		{
			//printf("raise IRQ %d (mask %x)\n", irq, m_irqmask);
			// Zip AppleTalk delay
			accel_temp_delay(5, BIT(m_accel_gsxsettings, 5));

			m_intflag |= INTFLAG_IRQASSERTED;
			m_maincpu->set_input_line(G65816_LINE_IRQ, ASSERT_LINE);
		}
	}
}

void apple2gs_state::lower_irq(int irq)
{
	if (m_irqmask & (1 << irq))
	{
		m_irqmask &= ~(1 << irq);

		//printf("lower IRQ %d (mask %x)\n", irq, m_irqmask);

		if (!m_irqmask)
		{
			m_intflag &= ~INTFLAG_IRQASSERTED;
			m_maincpu->set_input_line(G65816_LINE_IRQ, CLEAR_LINE);
		}
	}
}

void apple2gs_state::update_speed()
{
	if (m_twgs_kernel_on)
	{
		// Keep the accounting clock fixed at the card oscillator / 4. Actual
		// slow/fast selection and transition latency come from the GALs.
		m_last_speed = true;
		if (m_maincpu->unscaled_clock() != m_accel_speed)
			m_maincpu->set_unscaled_clock(m_accel_speed);
		if (m_bt_mode != BT_TWGS || m_bt_cycle != BT_PER_SEC / m_accel_speed)
			bt_setup(m_accel_speed);
		return;
	}
	const bool isfast = (m_speed & SPEED_HIGH) && !(m_speed & m_motors_active);
	const bool noaccel = !m_accel_fast || m_accel_temp_slowdown;
	u32 new_speed = m_accel_speed;

	if (isfast && noaccel)
	{
		new_speed = A2GS_2_8M.value();
	}
	else if (!isfast && (noaccel || BIT(m_accel_gsxsettings, 3) || m_twgs))
	{
		new_speed = A2GS_1M.value();
	}

	// Account for the current slice at its original clock before changing domains.
	if (m_maincpu->executing() && (new_speed != m_maincpu->unscaled_clock()))
	{
		machine().scheduler().synchronize(timer_expired_delegate(FUNC(apple2gs_state::apply_speed), this));
		return;
	}

	m_last_speed = (new_speed != A2GS_1M.value());
	m_maincpu->set_unscaled_clock(new_speed, !m_last_speed); // re-align with PH0
	bt_setup(new_speed);
}

TIMER_CALLBACK_MEMBER(apple2gs_state::apply_speed)
{
	// A later access in the same instruction may have changed the requested speed.
	update_speed();
}

void apple2gs_state::accel_reset()
{
	if (!m_accel_warm_boot)
	{
		m_accel_unlocked = false;
		m_accel_stage = 0;
		m_accel_gsxsettings = 0x59; // paddle and counter slow, CPS, GS
		m_accel_slotspk = 0x45; // slots 6, 2, and speaker slow
		m_accel_percent = 0; // 100% speed
		m_accel_fast = true;
		update_speed();
	}

	// C059 bit 2 shows boot status
	m_accel_gsxsettings = (m_accel_gsxsettings & ~0x04) | (m_accel_warm_boot ? 0x04 : 0);
	m_accel_warm_boot = true;
}

void apple2gs_state::accel_temp_delay(int ms, bool condition)
{
	// C05B status bit toggles even when accelerator is disabled
	if ((m_accel_present) && (condition))
	{
		m_accel_temp_slowdown = true;
		m_acceltimer->adjust(attotime::from_msec(ms));
		update_speed();
	}
}

void apple2gs_state::accel_stop_delay()
{
	m_accel_temp_slowdown = false;
	m_acceltimer->adjust(attotime::never);
	update_speed();
}

void apple2gs_state::accel_slot(int slot)
{
	if (!machine().side_effects_disabled())
		accel_temp_delay(52, BIT(m_accel_slotspk | (m_speed << 4), slot));
}

TIMER_DEVICE_CALLBACK_MEMBER(apple2gs_state::accel_timer)
{
	accel_stop_delay();
}

// ---------------------------------------------------------------------------
// TransWarp GS firmware interface (Applied Engineering, TransWarp GS
// Programmer's Reference, Appendix B): "TWGS" at $BC/FF00 and a jump table
// of JSL calls from $BC/FF08. This is not the AE ROM: each call is a short
// routine at $BC/FE00 that uses the registers at $BC/FD00 below. The
// configuration register at $BC/0000 is the card's (TWGS ROM 1.8s
// disassembly, D. Brock et al.): bit 1 data cache on, bit 2 TransWarp on,
// bit 3 IRQ slowdown off. Speed index 0 is the GS slow speed, 1 the GS fast
// speed with the TransWarp off, 2 the TransWarp speed (the ROM's three).
// ---------------------------------------------------------------------------

void apple2gs_state::twgs_build_firmware()
{
	std::fill_n(m_twgs_fw, 0x200, 0);
	static const u8 id[8] = { 'T', 'W', 'G', 'S', 'S', 'M', 'J', 'S' };
	std::copy_n(id, 8, m_twgs_fw + 0x100);

	// 65816 code, native mode: LDA/STA long to a $BC/FDxx register
	enum { LDA, STA, TAX, TXA, RTL };
	struct op { int code; u8 reg; };
	static const std::vector<std::vector<op>> calls = {
		{ { LDA, 0x02 }, { TAX, 0 }, { LDA, 0x00 }, { RTL, 0 } },   // $FF08 GetTWInfo
		{ { STA, 0x20 }, { RTL, 0 } },                              // $FF0C ResetTW
		{ { LDA, 0x04 }, { RTL, 0 } },                              // $FF10 GetMaxSpeed
		{ { LDA, 0x06 }, { TAX, 0 }, { RTL, 0 } },                  // $FF14 GetNumISpeed
		{ { STA, 0x10 }, { LDA, 0x10 }, { TAX, 0 }, { RTL, 0 } },   // $FF18 Freq2Index
		{ { TXA, 0 }, { STA, 0x12 }, { LDA, 0x12 }, { RTL, 0 } },   // $FF1C Index2Freq
		{ { LDA, 0x08 }, { RTL, 0 } },                              // $FF20 GetCurSpeed
		{ { STA, 0x14 }, { LDA, 0x14 }, { RTL, 0 } },               // $FF24 SetCurSpeed
		{ { LDA, 0x0a }, { TAX, 0 }, { RTL, 0 } },                  // $FF28 GetCurISpeed
		{ { TXA, 0 }, { STA, 0x16 }, { RTL, 0 } },                  // $FF2C SetCurISpeed
		{ { STA, 0x18 }, { RTL, 0 } },                              // $FF30 FlushCache
		{ { STA, 0x1a }, { RTL, 0 } },                              // $FF34 DisableIRQLogic
		{ { STA, 0x1c }, { RTL, 0 } },                              // $FF38 EnableIRQLogic
		{ { LDA, 0x0e }, { RTL, 0 } },                              // $FF3C GetTWConfig
		{ { STA, 0x1e }, { RTL, 0 } },                              // $FF40 SetTWConfig
		{ { LDA, 0x0c }, { RTL, 0 } },                              // $FF44 GetCacheSize
		{ { STA, 0x22 }, { RTL, 0 } },                              // $FF48 EnableDataCache
		{ { STA, 0x24 }, { RTL, 0 } } };                            // $FF4C DisableDataCache
	u32 pc = 0;
	for (int i = 0; i < int(calls.size()); i++)
	{
		const u32 entry = 0x108 + i * 4;
		m_twgs_fw[entry] = 0x5c; // JML $BCFExx
		m_twgs_fw[entry + 1] = pc & 0xff;
		m_twgs_fw[entry + 2] = 0xfe;
		m_twgs_fw[entry + 3] = 0xbc;
		for (const op &o : calls[i])
		{
			switch (o.code)
			{
				case LDA: case STA:
					m_twgs_fw[pc++] = (o.code == LDA) ? 0xaf : 0x8f;
					m_twgs_fw[pc++] = o.reg;
					m_twgs_fw[pc++] = 0xfd;
					m_twgs_fw[pc++] = 0xbc;
					break;
				case TAX: m_twgs_fw[pc++] = 0xaa; break;
				case TXA: m_twgs_fw[pc++] = 0x8a; break;
				case RTL: m_twgs_fw[pc++] = 0x6b; break;
			}
		}
	}
}

// ResetTW: the settings of the TWGS control panel (here: the machine configuration)
void apple2gs_state::twgs_reset_config()
{
	twgs_set_config(0x06 | (m_twgs_irq_at_reset ? 0x00 : 0x08));
}

void apple2gs_state::twgs_speed_index(int index)
{
	m_speed = (index == 0) ? (m_speed & ~SPEED_HIGH) : (m_speed | SPEED_HIGH);
	twgs_set_config((index == 2) ? (m_twgs_config | 0x04) : (m_twgs_config & ~0x04));
}

int apple2gs_state::twgs_cur_index()
{
	if (!(m_speed & SPEED_HIGH))
		return 0;
	return BIT(m_twgs_config, 2) ? 2 : 1;
}

u16 apple2gs_state::twgs_index_freq(int index)
{
	switch (index)
	{
		case 0: return 1024;
		case 1: return 2600;
		case 2: return m_accel_speed / 1000;
		default: return 0;
	}
}

int apple2gs_state::twgs_freq_index(u16 khz)
{
	for (int i = 0; i < 3; i++)
		if (twgs_index_freq(i) >= khz)
			return i;
	return 2;
}

u8 apple2gs_state::twgs_r(offs_t offset)
{
	if (m_twgs_real)
		return twgs_rom_r(offset);

	if (offset == 0x0000)
		return m_twgs_config;
	if (offset >= 0xfe00)
		return m_twgs_fw[offset - 0xfe00];
	if ((offset & 0xff00) == 0xfd00)
	{
		u16 v = 0;
		switch (offset & 0xfe)
		{
			case 0x00: v = 0x0000; break;                                  // features: no hardware flush, fixed clock
			case 0x02: v = 0x0108; break;                                  // version 1.8
			case 0x04: v = twgs_index_freq(2); break;                      // the maximum speed, KHz
			case 0x06: v = 3; break;                                       // number of speeds
			case 0x08: v = twgs_index_freq(twgs_cur_index()); break;       // the current speed, KHz
			case 0x0a: v = twgs_cur_index(); break;
			case 0x0c: v = (m_twgs_mask + 1) >> 10; break;                 // cache size, KB
			case 0x0e: v = m_twgs_config; break;
			case 0x10: case 0x12: case 0x14: v = m_twgs_result; break;
		}
		return BIT(offset, 0) ? (v >> 8) : (v & 0xff);
	}
	return 0x00;
}

void apple2gs_state::twgs_w(offs_t offset, u8 data)
{
	if (m_twgs_real)
	{
		twgs_rom_w(offset, data);
		return;
	}

	if (offset == 0x0000)
	{
		twgs_set_config(data);
		return;
	}
	if ((offset & 0xff00) != 0xfd00)
		return;
	if (!BIT(offset, 0))
	{
		m_twgs_arg = data;
		return;
	}
	// the high byte of a 16-bit store: the call
	const u16 arg = m_twgs_arg | (u16(data) << 8);
	switch (offset & 0xfe)
	{
		case 0x10: m_twgs_result = twgs_freq_index(arg); break;
		case 0x12: m_twgs_result = twgs_index_freq(arg); break;
		case 0x14:
		{
			const int index = twgs_freq_index(arg);
			twgs_speed_index(index);
			m_twgs_result = twgs_index_freq(index);
			break;
		}
		case 0x16: if (arg < 3) twgs_speed_index(arg); break;
		case 0x18: std::fill_n(m_zip_tag.get(), 0x10000, 0); break;
		case 0x1a: twgs_set_config(m_twgs_config | 0x08); break;
		case 0x1c: twgs_set_config(m_twgs_config & ~0x08); break;
		case 0x1e: twgs_set_config((arg & ~0x02) | (m_twgs_config & 0x02)); break; // keeps the data cache bit
		case 0x20: twgs_reset_config(); break;
		case 0x22: twgs_set_config(m_twgs_config | 0x02); break;
		case 0x24: twgs_set_config(m_twgs_config & ~0x02); break;
	}
}

void apple2gs_state::twgs_set_config(u8 data)
{
	if (m_twgs_real && ((m_twgs_config ^ data) & 0x02))
		std::fill(std::begin(m_twgs_tag), std::end(m_twgs_tag), 0);
	m_twgs_config = data & 0x0e;
	m_accel_fast = BIT(m_twgs_config, 2);
	update_speed();
}

// The serial port clocks the X2444 and the XC2064 readback chain on access.
u8 apple2gs_state::twgs_rom_r(offs_t offset)
{
	if (offset >= 0x8000)
		return m_twgs_rom[offset & 0x7fff];
	if (offset < 0x4000)
	{
		// GAL6/P12 and the FPGA select this whole range. Resolve the driven
		// nibble from old FPGA state before the CPU-fall clock, including CH
		// on D0; keep the existing undriven-bus value on the other lanes.
		return m_twgs_kernel.c.fpga.config_read(0xbc0000 | offset, true, true, offset ? 0xbc : 0);
	}
	if (offset == 0x4000)
	{
		if (BIT(m_twgs_serial, 7))
		{
			const u8 result = m_twgs_nvram->do_read() << 7;
			if (!machine().side_effects_disabled())
			{
				m_twgs_nvram->clk_write(0);
				m_twgs_nvram->clk_write(1);
			}
			return result;
		}
		if (BIT(m_twgs_serial, 0))
		{
			const u32 pos = m_twgs_readback;
			if (!machine().side_effects_disabled())
				m_twgs_readback++;
			// Three leading clocks, then a start bit and 72 bits per frame.
			if (pos >= 3 && ((pos - 3) % 73) != 0)
			{
				const u32 bit = 41 + ((pos - 3) / 73) * 75 + ((pos - 3) % 73) - 1;
				if ((bit >> 3) < 0x600)
					return BIT(~m_twgs_rom[bit >> 3], bit & 7) << 7;
			}
		}
		return 0;
	}
	return 0xbc;
}

void apple2gs_state::twgs_rom_w(offs_t offset, u8 data)
{
	if (offset == 0)
		twgs_set_config(data);
	else if (offset == 0x4001)
	{
		if (!BIT(m_twgs_serial, 0) && BIT(data, 0) && !BIT(data, 7))
			m_twgs_readback = 0;
		m_twgs_serial = data;
		m_twgs_nvram->cs_write(BIT(data, 7));
	}
	else if (offset == 0x4000 && BIT(m_twgs_serial, 7))
	{
		m_twgs_nvram->clk_write(0);
		m_twgs_nvram->di_write(BIT(data, 7));
		m_twgs_nvram->clk_write(1);
	}
}

bool apple2gs_state::twgs_shadowed(u32 address) const
{
	const u16 low = address;
	const u8 bank = address >> 16;
	if (bank == 0xe0 || bank == 0xe1)
		return true;
	if (bank >= 2 && (!BIT(m_speed, 4) || bank >= 0xe0))
		return false;
	const bool aux = BIT(twgs_taddr(address, g65816_device::BUS_WRITE), 16);
	// Text page 1 is a slow Mega II write only while shadowing is enabled.
	// The inhibit bit leaves the cycle on fast RAM. Reads are never slowed here.
	if (low >= 0x0400 && low < 0x0800)
		return !(m_shadow & SHAD_TXTPG1);
	if (low >= 0x0800 && low < 0x0c00)
		return m_is_rom3 && !(m_shadow & SHAD_TXTPG2);
	if (low >= 0x2000 && low < 0x4000)
		return (!(m_shadow & SHAD_HIRESPG1) && (!aux || !(m_shadow & SHAD_AUXHIRES))) || (aux && !(m_shadow & SHAD_SUPERHIRES));
	if (low >= 0x4000 && low < 0x6000)
		return (!(m_shadow & SHAD_HIRESPG2) && (!aux || !(m_shadow & SHAD_AUXHIRES))) || (aux && !(m_shadow & SHAD_SUPERHIRES));
	return low >= 0x6000 && low < 0xa000 && aux && !(m_shadow & SHAD_SUPERHIRES);
}

u8 apple2gs_state::twgs_board_read(twgs_decoded::transaction const &w, u64 begin, u64 end)
{
	if (m_glu_log_enabled)
		sndglu_log_note(w.address, w.type);
	begin /= TWGS_TIME_SCALE;
	end /= TWGS_TIME_SCALE;
	const bool slow = m_bt_slow;
	m_bt_busfree = end;
	// Keep the existing GLU strobe convention within the actual final Mega II
	// cycle. An isolated access can include a preceding synchronization wait.
	const bool mega = m_twgs_kernel.bus_class(w) == BC_MEGA;
	m_bt_slot = mega ? end - ((end % BT_LINE) ? BT_MEGA : 16 * BT_CLK) : begin;
	m_bt_slot_ok = true;
	m_snd_force = bt_at(m_bt_slot);
	m_snd_force_on = true;
	u8 data;
	if (w.type == g65816_device::BUS_VECTOR)
		data = m_maincpu->space(g65816_device::AS_VECTORS).read_byte(w.address & 0x1f);
	else if (m_twgs_boot && (w.address >> 16) == 0 && (w.address & 0xffff) >= 0x8000)
		data = m_twgs_rom[w.address & 0x7fff];
	else if ((w.address >> 16) == 0xbc)
	{
		m_twgs_boot = false;
		data = twgs_rom_r(w.address & 0xffff);
	}
	else
		data = m_maincpu->space(AS_PROGRAM).read_byte(w.address);
	m_snd_force_on = false;
	m_bt_slow = slow;
	return data;
}

void apple2gs_state::twgs_board_write(twgs_decoded::transaction const &w, u64 begin, u64 end)
{
	if (m_glu_log_enabled)
		sndglu_log_note(w.address, w.type);
	begin /= TWGS_TIME_SCALE;
	end /= TWGS_TIME_SCALE;
	m_twgs_retired = std::max(m_twgs_retired, w.id);
	// BC belongs to the card. FPGA/register capture happens at CPU completion;
	// it is never converted into a motherboard write by a barrier or drain.
	if ((w.address >> 16) == 0xbc)
		return;
	const bool slow = m_bt_slow;
	m_bt_busfree = end;
	const bool mega = m_twgs_kernel.bus_class(w) == BC_MEGA;
	m_bt_slot = mega ? end - ((end % BT_LINE) ? BT_MEGA : 16 * BT_CLK) : begin;
	m_bt_slot_ok = true;
	m_snd_force = bt_at(m_bt_slot);
	m_snd_force_on = true;
	m_maincpu->space(AS_PROGRAM).write_byte(w.address, w.data);
	m_snd_force_on = false;
	m_bt_slow = slow;
}

void apple2gs_state::twgs_kernel_schedule()
{
	if (m_twgs_retired >= m_twgs_visible)
	{
		if (m_twgs_write_timer->enabled())
			m_twgs_write_timer->adjust(attotime::never);
		return;
	}
	const attotime now = machine().time();
	const attotime when = std::max(now, twgs_at(m_twgs_kernel.gs_event));
	if (!m_twgs_write_timer->enabled() || m_twgs_write_timer->expire() != when)
		m_twgs_write_timer->adjust(when - now);
}

void apple2gs_state::twgs_kernel_cycle(u32 address, int type, u8 &data)
{
	auto &t = m_twgs_kernel;
	t.speed = (m_speed & m_motors_active) ? (m_speed & ~SPEED_HIGH) : m_speed;
	t.shadow = m_shadow;
	if (!m_twgs_kernel_started)
	{
		t.x4 = (BT_PER_SEC * TWGS_TIME_SCALE) / (u64(m_accel_speed) * 4);
		t.c.mask = m_twgs_mask;
		t.start(twgs_ticks(m_maincpu->local_time()));
		m_twgs_kernel_started = true;
	}
	const u64 start = t.now, old_mega = t.mega_cycles, old_board = t.board_cycles, old_refresh = t.refresh_cycles;
	twgs_decoded::transaction w{address, ++m_twgs_transaction, data, u8(type), false};
	t.access(w, twgs_board{*this});
	const u64 duration = t.now - start;
	const u64 charge = duration > t.x4 * 4 ? duration - t.x4 * 4 : 0;
	const u64 stall = charge / TWGS_TIME_SCALE;
	m_bt_idx++;
	m_bt_istall += stall;
	// Retain even the first partial clock's credit. Clamping each transfer
	// would let machine time creep ahead of the decoded CPU/GS edge stream.
	m_twgs_cycle_charge += s64(duration) - s64(t.x4 * 4);
	m_btc[BTC_MEGA] += t.mega_cycles - old_mega;
	m_btc[BTC_FAST] += t.board_cycles - old_board - (t.mega_cycles - old_mega);
	m_btc[BTC_REFRESH] += t.refresh_cycles - old_refresh;
	u64 *const page = &m_btpage[(address >> 8) * 4];
	const bool internal = type >= g65816_device::BUS_INTERNAL;
	if (w.read())
	{
		data = t.c.receipt;
		m_btc[BTC_READS]++;
		page[0]++;
		if (t.c.from_sram && !t.had_bus)
			m_btc[BTC_HITS]++;
		else
		{
			page[1]++;
			m_btpc[m_bt_ipc * 3 + 1]++;
			m_btc[t.c.pins.eligible ? BTC_MISSES : BTC_UNCACHED]++;
			if (t.c.pins.eligible)
				m_btc[type == g65816_device::BUS_OPCODE ? BTC_MISS_OPCODE : type == g65816_device::BUS_OPERAND ? BTC_MISS_OPERAND : BTC_MISS_DATA]++;
		}
	}
	else
	{
		m_btc[BTC_WRITES]++;
		page[2]++;
		if ((address >> 16) == 0xbc)
			twgs_rom_w(address & 0xffff, data);
		else if ((address >> 16) >= 0x80 || t.bus_class(w) != BC_RAM)
			m_twgs_visible = w.id;
	}
	m_btc[internal ? BTC_STALL_INTERNAL : w.read() ? BTC_STALL_READ : BTC_STALL_WRITE] += stall;
	page[3] += stall / BT_PAGE_UNIT;
	m_btpc[m_bt_ipc * 3] += stall / BT_PAGE_UNIT;
	if (m_twgs_visible > m_twgs_retired || m_twgs_write_timer->enabled())
		twgs_kernel_schedule();
}

u64 apple2gs_state::twgs_kernel_instruction(u32 pc)
{
	const u64 now = m_maincpu->total_cycles();
	if (!m_bt_resync)
	{
		const u64 base = now - m_bt_cprev - m_bt_charged;
		if (base > BT_IDLE)
		{
			// WAI/STP expose their held read bus while the CPU is suspended.
			const u64 deadline = twgs_ticks(m_maincpu->local_time());
			m_twgs_kernel.advance_until(deadline, twgs_board{*this});
		}
		else if (base != m_bt_idx)
		{
			++m_twgs_unemitted_cycles;
			fatalerror("TWGS CPU bus coverage at %06x: base=%llu emitted=%llu\n", pc,
				(unsigned long long)base, (unsigned long long)m_bt_idx);
		}
		const u64 duration = m_twgs_kernel.now / TWGS_TIME_SCALE - m_bt_istart;
		m_btc[BTC_TIME] += duration;
		m_btpc[m_bt_ipc * 3 + 2] += duration / BT_PAGE_UNIT;
	}
	m_bt_resync = false;
	m_bt_istart = m_twgs_kernel.now / TWGS_TIME_SCALE;
	m_btc[BTC_INSTR]++;
	m_bt_cprev = now;
	m_bt_charged = m_bt_idx = m_bt_istall = 0;
	m_bt_ipc = pc < 0x060000 ? pc >> 4 : 0x6000 + (pc >> 8);
	return 0;
}

int apple2gs_state::twgs_kernel_access(u32 address, int type, u8 &data)
{
	if (type == g65816_device::BUS_OPCODE)
		twgs_kernel_instruction(address);
	twgs_kernel_cycle(address, type, data);
	const u64 cycle = m_twgs_kernel.x4 * 4;
	const u64 n = m_twgs_cycle_charge >= s64(cycle) ? u64(m_twgs_cycle_charge) / cycle : 0;
	m_twgs_cycle_charge -= n * cycle;
	m_bt_charged += n;
	return int(n);
}

int apple2gs_state::twgs_internal(u32 address, int type, u8 data)
{
	return twgs_kernel_access(address, type, data);
}

void apple2gs_state::twgs_drain(unsigned count)
{
	// The bus latches retain the CPU bank byte. FPI performs shadowing when
	// the transaction reaches the motherboard, using its then-current switches.
	const bool slow = m_bt_slow;
	while (count-- && m_twgs_wcount)
	{
		m_maincpu->space(AS_PROGRAM).write_byte(m_twgs_waddr[0], m_twgs_wdata[0]);
		--m_twgs_wcount;
		for (unsigned i = 0; i < m_twgs_wcount; ++i)
		{
			m_twgs_waddr[i] = m_twgs_waddr[i + 1];
			m_twgs_wdata[i] = m_twgs_wdata[i + 1];
			m_twgs_wend[i] = m_twgs_wend[i + 1];
			m_twgs_wdefer[i] = m_twgs_wdefer[i + 1];
		}
	}
	m_bt_slow = slow;
	twgs_schedule_write();
}

void apple2gs_state::twgs_schedule_write()
{
	// Unshadowed RAM has no autonomous reader. Cache misses and mapping/IO
	// accesses drain it before use; slot DMA synchronizes at its bus access.
	// Keep a timer whenever any queued write can affect a peripheral/video.
	// Bus slot reservation and full-buffer CPU waits are unchanged.
	bool observable = false;
	for (unsigned i = 0; i < m_twgs_wcount; ++i)
		observable |= !m_twgs_wdefer[i];
	if (!observable)
	{
		if (m_twgs_write_timer->expire() != attotime::never)
			m_twgs_write_timer->adjust(attotime::never);
	}
	else
	{
		const attotime end = attotime::from_double(double(m_twgs_wend[0]) / double(BT_PER_SEC));
		const attotime now = machine().time();
		const attotime when = std::max(now, end);
		if (!m_twgs_write_timer->enabled() || m_twgs_write_timer->expire() != when)
			m_twgs_write_timer->adjust(when - now);
	}
}

void apple2gs_state::twgs_flush_due()
{
	if (m_twgs_kernel_on)
	{
		if (m_twgs_kernel_started)
			m_twgs_kernel.advance_until(twgs_ticks(machine().time()), twgs_board{*this});
		twgs_kernel_schedule();
		return;
	}
	// Materialize only completed bus cycles before an external observation.
	if (!m_twgs_wcount)
		return;
	const u64 now = u64(machine().time().as_double() * double(BT_PER_SEC));
	unsigned count = 0;
	while (count < m_twgs_wcount && m_twgs_wend[count] && m_twgs_wend[count] <= now)
		++count;
	if (count)
		twgs_drain(count);
}

TIMER_CALLBACK_MEMBER(apple2gs_state::twgs_write_complete)
{
	if (m_twgs_kernel_on)
	{
		twgs_flush_due();
		return;
	}
	twgs_drain(1);
}

bool apple2gs_state::twgs_cached_access(u32 address, int type, u8 &data, int &cycles)
{
	if (m_glu_log_enabled)
		sndglu_log_note(address, type);
	if (m_twgs_kernel_on)
	{
		cycles = twgs_kernel_access(address, type, data);
		return true;
	}
	// Combine the memory, data and timing callbacks for ordinary SRAM hits.
	// Special bank and bootstrap reads retain the full motherboard path.
	if (m_twgs_boot || m_bt_mode != BT_TWGS || (address >> 16) == 0xbc ||
		(address >> 16) == 0xbf || (twgs_io_bank(address) && (address & 0xf000) == 0xc000) ||
		!BIT(m_twgs_config, 2) || !BIT(m_speed, 7) ||
		(!BIT(m_twgs_config, 3) && m_maincpu->irq_masked()) ||
		(type == g65816_device::BUS_READ && !BIT(m_twgs_config, 1)))
		return false;
	const u32 tag = (twgs_io_bank(address) ? twgs_taddr(address, type) : address) + 1;
	const u32 slot = address & m_twgs_mask;
	if (type == g65816_device::BUS_WRITE)
	{
		// The enabled card captures ordinary stores in its four latches and
		// updates SRAM. Device, ROM and bootstrap stores retain
		// the general path. No motherboard handler runs at capture time.
		m_twgs_access_io = false;
		m_twgs_access_cacheable = true;
		m_twgs_access_tag = tag;
		m_twgs_access_hit = m_twgs_tag[slot] == tag;
		if (m_twgs_wcount >= TWGS_WRITE_LATCHES)
			twgs_drain(1);
		m_bt_slow = twgs_shadowed(address);
		m_twgs_waddr[m_twgs_wcount] = address;
		m_twgs_wdata[m_twgs_wcount] = data;
		m_twgs_wdefer[m_twgs_wcount] = (address >> 16) < 0x80 && !m_bt_slow && ((address >> 16) >= 2 || (address & 0xf000) != 0xc000);
		m_twgs_wend[m_twgs_wcount++] = 0;
		m_twgs_post = true;
		m_twgs_tag[slot] = tag;
		m_twgs_cache_data[slot] = data;
		cycles = bt_access(address, type, data);
		return true;
	}
	if (m_twgs_tag[slot] != tag)
		return false;
	data = m_twgs_cache_data[slot];
	const bool opcode = type == g65816_device::BUS_OPCODE;
	const bool stretched_opcode = opcode && REPSEP_SLOW_CYCLES && (data == 0xc2 || data == 0xe2);
	if (!m_bt_twslow && !m_twgs_post && !stretched_opcode)
	{
		// An SRAM hit has no motherboard transaction. Only an opcode needs
		// to close the previous instruction and account its internal cycles.
		m_bt_slow = m_bt_slot_ok = false;
		if (opcode)
			m_bt_frac += bt_instruction(address);
		m_bt_idx++;
		m_btc[BTC_READS]++;
		m_btc[BTC_HITS]++;
		m_btpage[(address >> 8) * 4]++;
		cycles = 0;
		if (m_bt_frac >= m_bt_cycle)
		{
			const u64 n = m_bt_frac / m_bt_cycle;
			m_bt_frac -= n * m_bt_cycle;
			m_bt_charged += n;
			cycles = int(n);
		}
		return true;
	}
	// These are inputs to the general hook, not state of the SRAM. A hit
	// completed above needs no descriptor for a motherboard transaction.
	m_twgs_access_io = false;
	m_twgs_access_cacheable = true;
	m_twgs_access_tag = tag;
	m_twgs_access_hit = true;
	cycles = bt_access(address, type, data);
	return true;
}

bool apple2gs_state::twgs_memory(u32 address, int type, u8 &data)
{
	const u8 bank = address >> 16;
	const u16 low = address;
	const bool write = type == g65816_device::BUS_WRITE;
	const bool code = type == g65816_device::BUS_OPCODE || type == g65816_device::BUS_OPERAND;
	const bool io = twgs_io_bank(address) && low >= 0xc000 && low < 0xd000;
	m_twgs_access_io = io;
	m_twgs_access_cacheable = !io && type != g65816_device::BUS_VECTOR;
	m_twgs_access_tag = twgs_taddr(address, type) + 1;
	m_twgs_access_hit = m_twgs_tag[address & m_twgs_mask] == m_twgs_access_tag;
	const bool enabled = BIT(m_twgs_config, 2) && BIT(m_speed, 7) && (BIT(m_twgs_config, 3) || !m_maincpu->irq_masked());

	// Device accesses, mapping changes and read misses wait for the pending
	// motherboard writes. Cache hits can proceed while those writes drain.
	if (io || bank == 0xbc || type == g65816_device::BUS_VECTOR || !enabled)
	{
		twgs_drain(m_twgs_wcount);
		twgs_snoop(address, write, data);
		return false;
	}
	if (!write)
	{
		const u32 slot = address & m_twgs_mask;
		if (bank == 0xbf || ((code || BIT(m_twgs_config, 1)) && m_twgs_access_hit))
		{
			data = m_twgs_cache_data[slot];
			return true;
		}
		twgs_drain(m_twgs_wcount);
		return false;
	}
	if (m_bt_mode != BT_TWGS)
		return false;
	if (m_twgs_wcount >= TWGS_WRITE_LATCHES)
		twgs_drain(1);
	m_bt_slow = twgs_shadowed(address);
	m_twgs_waddr[m_twgs_wcount] = address;
	m_twgs_wdata[m_twgs_wcount] = data;
	m_twgs_wdefer[m_twgs_wcount] = bank < 0x80 && !m_bt_slow && (bank >= 2 || (address & 0xf000) != 0xc000);
	m_twgs_wend[m_twgs_wcount++] = 0;
	m_twgs_post = true;
	return true;
}

u8 apple2gs_state::twgs_bus(u32 address, int type, u8 data)
{
	const u8 bank = address >> 16;
	const u16 low = address;
	const bool write = type == g65816_device::BUS_WRITE;
	const bool code = type == g65816_device::BUS_OPCODE || type == g65816_device::BUS_OPERAND;
	if (m_twgs_boot && !write)
	{
		if (bank == 0 && low >= 0x8000)
			return m_twgs_rom[low & 0x7fff];
		if (bank == 0xbc)
			m_twgs_boot = false;
	}
	if (bank == 0xbc && low < 0x8000)
		return data;
	if (m_twgs_access_io)
		return data;
	const u32 slot = address & m_twgs_mask;
	if (bank == 0xbf)
	{
		if (write)
		{
			m_twgs_tag[slot] = address + 1;
			m_twgs_cache_data[slot] = data;
		}
		return m_twgs_cache_data[slot];
	}
	const u32 tag = m_twgs_access_tag - 1;
	if (type == g65816_device::BUS_VECTOR)
		return data;
	const bool enabled = BIT(m_twgs_config, 2) && BIT(m_speed, 7) && (BIT(m_twgs_config, 3) || !m_maincpu->irq_masked());
	// Cache enable controls the CPU read source. A motherboard read still
	// updates SRAM through the GAL2 SEL_GS_PH2 write-enable term.
	if (!write && enabled && (code || BIT(m_twgs_config, 1)) && m_twgs_access_hit)
		return m_twgs_cache_data[slot];
	// Writes to the EPROM fill the cache with ROM data, as used by FlushCache.
	if (write && bank == 0xbc)
		data = m_twgs_rom[low & 0x7fff];
	m_twgs_tag[slot] = tag + 1;
	m_twgs_cache_data[slot] = data;
	return data;
}

/***************************************************************************
    VIDEO
***************************************************************************/

// ---------------------------------------------------------------------------
// Bus timing model
//
// IIgs (header of this file, IIgs Hardware Reference): a fast cycle is 5 14M
// clocks; DRAM refresh takes 5 clocks of every 50 and delays fast RAM, not ROM
// or I/O; a Mega II (1 MHz) access waits for the next Mega II cycle, 14 clocks,
// with every 65th cycle 16 clocks (912 clocks a line).
//
// ZipGS (patent US 4,794,523; ZipGSX manual; D. Empson, comp.sys.apple2):
// direct mapped, one tag per data byte (1-byte lines, no read-ahead); a read
// miss runs one motherboard cycle and fills that byte; writes go through a
// write buffer to the motherboard and also write the cache. The patent's
// first-stage latch is one deep: the byte is enabled onto the Apple bus on
// the next phase 0, and a second write before that phase 0 stops the CPU
// until the latch can take it (so the new byte occupies the following
// phase 0). Private mapping latches select the auxiliary and language-card
// tag bits. $C000-$C0FF is never cached. $C100-$C7FF is one page per slot,
// so the tag names it. $C800-$CFFF is the shared /IOSTROBE window and is not
// read from cache when the motherboard serves the selected card (MENABB: the
// tag cannot name the slot, and the card can change the bytes). Motherboard
// ROM in that window stays cacheable. A 16 KB cache caches fast RAM banks
// $00-$2F, $E0, $E1 and $FC-$FF (measured). Read hits return the cached byte.
//
// TransWarp GS (AE manual and Programmer's Reference; G. Body schematic,
// 2016; ROM 1.8s disassembly): direct mapped, 8 or 32 KB, a 16-bit tag for
// each data byte (1-byte lines); all memory is cached except I/O
// $C000-$CFFF; a store also fills the cache (the ROM's cache size test reads
// back stores to bank $BF); stores go to the GS through 4 latched byte
// writes (schematic: 4 sets of address, bank and data latches). GAL2 enables
// each latch onto the motherboard with the socket's PH2, so a latched write
// takes the bus cycle the FPI gives its address (a Mega II cycle for $C03x).
// With the data cache off, data reads miss and
// code fetches still hit; while the CPU has interrupts off and the IRQ
// logic is on, the card runs at the GS speed.
// ---------------------------------------------------------------------------

void apple2gs_state::bt_config()
{
	const ioport_value cfg = m_btconfig->read();
	m_zip_size = (cfg >> 1) & 3;
	m_zip_mask = (0x2000 << m_zip_size) - 1;
	std::fill_n(m_zip_tag.get(), 0x10000, 0);

	static const u32 fastbanks[4] = { 0x10, 0x30, 0x70, 0x80 };
	// ZipGSX cache-size decode; 16K bank limit measured by David Empson.
	const u32 limit = fastbanks[m_zip_size];
	for (int bank = 0; bank < 256; bank++)
		m_zip_bank_ok[bank] = (bank < limit) || (bank == 0xe0) || (bank == 0xe1) || (bank >= 0xfc);

	// the native-speed setting replaces the accelerator of the CPU type
	m_native_ram = BIT(cfg, 13);
	m_twgs_sel = BIT(cfg, 10) && !m_native_ram;
	m_twgs_real = BIT(ioport("twgs_rom")->read(), 0);
	m_twgs_irq_at_reset = !BIT(cfg, 12);
	m_twgs_mask = BIT(cfg, 11) ? 0x7fff : 0x1fff;
	// GAL1/GAL2 write service adds no whole motherboard clocks.
	m_bt_wcount = 0;
	m_bt_busfree = 0;
	m_bt_fanchor = 0;
}

void apple2gs_state::bt_setup(u32 speed)
{
	int mode = BT_OFF;
	if (m_last_speed)
		mode = (speed == A2GS_2_8M.value()) ? BT_STOCK : (m_twgs ? BT_TWGS : (m_native_ram ? BT_NATIVE : BT_ZIP));
	m_bt_mode = mode;
	m_bt_cycle = ((mode == BT_ZIP) || (mode == BT_TWGS) || (mode == BT_NATIVE)) ? (BT_PER_SEC / speed) : BT_FAST;
	m_bt_twslow = false;
	m_bt_resync = true;
	m_bt_slow = false;
	m_bt_wcount = 0;
}

void apple2gs_state::bt_control()
{
	u32 *const header = reinterpret_cast<u32 *>(m_btprof);
	if (header[4] == 1)
	{
		std::fill(m_btprof + BTP_COUNT, m_btprof + BTP_SIZE, 0);
		header[4] = 0;
	}
	header[0] = 0x4353475a; // "ZGSC"
	header[1] = 4;
	header[2] = m_bt_mode;
	header[3] = (m_bt_mode == BT_ZIP) ? (m_zip_mask + 1) : (m_bt_mode == BT_TWGS) ? (m_twgs_mask + 1) : 0;
	header[5] = u32(m_bt_cycle);
	header[6] = u32(BT_CLK);
	header[7] = (m_bt_mode == BT_NATIVE) ? 0 : (((m_bt_mode == BT_TWGS) ? TWGS_WRITE_LATCHES : ZIP_WRITE_DEPTH) | (u32(BUS_READ_RESUME) << 8));
}

u64 apple2gs_state::bt_fast(u64 t, bool refresh)
{
	u64 b = (t <= m_bt_fanchor) ? m_bt_fanchor : m_bt_fanchor + ((t - m_bt_fanchor + BT_FAST - 1) / BT_FAST) * BT_FAST;
	if (refresh)
	{
		const u64 r = (b / BT_REFRESH_PERIOD) * BT_REFRESH_PERIOD;
		u64 wait = 0;
		if (b < r + BT_REFRESH)
			wait = r + BT_REFRESH;
		else if (b + BT_FAST > r + BT_REFRESH_PERIOD)
			wait = r + BT_REFRESH_PERIOD + BT_REFRESH;
		if (wait)
		{
			m_btc[BTC_REFRESH]++;
			b = wait;
			m_bt_fanchor = b;
		}
	}
	m_btc[BTC_FAST]++;
	m_bt_slot = b;
	return b + BT_FAST;
}

// A slow access presented at t runs in the first Mega II cycle that starts at
// or after t (Lowry FPI notes p.23). Cycle 64 of each line is the long one.
u64 apple2gs_state::bt_mega_start(u64 t) const
{
	const u64 line = (t / BT_LINE) * BT_LINE;
	const u64 pos = t - line;
	return (pos <= 64 * BT_MEGA) ? line + ((pos + BT_MEGA - 1) / BT_MEGA) * BT_MEGA : line + BT_LINE;
}

u64 apple2gs_state::bt_mega(u64 t)
{
	const u64 b = bt_mega_start(t);
	const u64 len = ((b % BT_LINE) == 64 * BT_MEGA) ? BT_MEGA_LAST : BT_MEGA;
	m_btc[BTC_MEGA]++;
	m_bt_fanchor = b + len;
	m_bt_slot = b;
	return b + len;
}

// does this bank $00/$E0 access go to the aux bank ($01/$E1)? The rules of auxbank_update()
bool apple2gs_state::bt_aux(u32 a16, bool write)
{
	if (a16 < 0x0200)
		return m_altzp;
	if (a16 >= 0xc000)
		return (a16 >= 0xd000) && m_altzp;
	if (m_video->get_80store())
	{
		if ((a16 >= 0x0400) && (a16 < 0x0800))
			return m_video->get_page2();
		if ((a16 >= 0x2000) && (a16 < 0x4000) && m_video->get_hires())
			return m_video->get_page2();
	}
	return write ? m_ramwrt : m_ramrd;
}

u64 apple2gs_state::bt_bus(u64 t, int cls)
{
	u64 end;
	switch (cls)
	{
		case BC_MEGA: end = bt_mega(t); break;
		case BC_RAM: end = bt_fast(t, true); break;
		default: end = bt_fast(t, false); break;
	}
	m_bt_slot_ok = true;
	return end;
}

u64 apple2gs_state::bt_instruction(u32 pc)
{
	u64 charge = 0;
	const u64 now = m_maincpu->total_cycles();
	if (m_bt_resync)
	{
		m_bt_resync = false;
		m_bt_istart = u64(m_maincpu->local_time().as_double() * double(BT_PER_SEC));
		m_bt_frac = 0;
	}
	else
	{
		// the cycles of the last instruction, as the CPU core counts them
		const u64 base = now - m_bt_cprev - m_bt_charged;
		if ((m_bt_mode == BT_STOCK) && (base <= BT_IDLE))
		{
			// its internal cycles are also bus cycles at 2.8 MHz (refresh)
			while (m_bt_idx < base)
			{
				const u64 t = m_bt_istart + m_bt_idx * BT_FAST + m_bt_istall;
				const u64 stall = bt_fast(t, true) - t - BT_FAST;
				m_bt_istall += stall;
				charge += stall;
				m_bt_idx++;
			}
			if (charge)
			{
				m_btc[BTC_STALL_INTERNAL] += charge;
				m_btpc[m_bt_ipc * 3] += charge / BT_PAGE_UNIT;
			}
		}
		else if (m_bt_twslow && (base <= BT_IDLE))
		{
			// TransWarp GS IRQ slowdown: its internal cycles are GS fast cycles too
			while (m_bt_idx < base)
			{
				const u64 t = m_bt_istart + m_bt_idx * m_bt_cycle + m_bt_istall;
				const u64 stall = bt_fast(t, true) - t - m_bt_cycle;
				m_bt_istall += stall;
				charge += stall;
				m_bt_idx++;
			}
			if (charge)
			{
				m_btc[BTC_STALL_INTERNAL] += charge;
				m_btpc[m_bt_ipc * 3] += charge / BT_PAGE_UNIT;
			}
		}
		const u64 duration = base * m_bt_cycle + m_bt_istall;
		m_bt_istart += duration;
		m_btc[BTC_TIME] += duration;
		m_btpc[m_bt_ipc * 3 + 2] += duration / BT_PAGE_UNIT;
	}
	m_btc[BTC_INSTR]++;
	m_bt_cprev = now;
	m_bt_charged = 0;
	m_bt_idx = 0;
	m_bt_istall = 0;
	m_bt_ipc = (pc < 0x060000) ? (pc >> 4) : (0x6000 + (pc >> 8));
	if (m_bt_mode == BT_TWGS)
		m_bt_twslow = !BIT(m_twgs_config, 3) && m_maincpu->irq_masked();
	return charge;
}

// The FPGA observes CPU cycles, independently of the motherboard handlers.
// GAL4 qualifies E0/E1 independently of O36. XC2064 HE.Q drives O36:
// qualified C035 writes latch D6; the other SHADOW bits are not stored here.
bool apple2gs_state::twgs_io_bank(u32 address) const
{
	const u8 bank = address >> 16;
	return bank == 0xe0 || bank == 0xe1 || (bank < 2 && !m_twgs_iolc);
}

void apple2gs_state::twgs_reset_switches()
{
	m_twgs_80store = m_twgs_page2 = m_twgs_hires = false;
	m_twgs_ramrd = m_twgs_ramwrt = m_twgs_altzp = false;
	m_twgs_lcram = m_twgs_lcprewrite = m_twgs_rombank = false;
	m_twgs_lcram2 = true;
	m_twgs_lcwrite = false;
	std::fill_n(m_zip_tag.get(), 0x10000, 0);
}

void apple2gs_state::twgs_snoop(u32 address, bool write, u8 data)
{
	if (!twgs_io_bank(address) || (address & 0xff00) != 0xc000)
		return;
	const u8 low = address;
	if (write)
	{
		switch (low)
		{
		case 0x00: case 0x01: m_twgs_80store = BIT(low, 0); break;
		case 0x02: case 0x03: m_twgs_ramrd = BIT(low, 0); break;
		case 0x04: case 0x05: m_twgs_ramwrt = BIT(low, 0); break;
		case 0x08: case 0x09: m_twgs_altzp = BIT(low, 0); break;
		case 0x28: m_twgs_rombank = !m_twgs_rombank; break; // DA
		case 0x29: m_twgs_linear = BIT(data, 6); break;    // HB
		case 0x35: m_twgs_iolc = BIT(data, 6); break;     // HE
		case 0x68:
			m_twgs_altzp = BIT(data, 7);
			m_twgs_page2 = BIT(data, 6);
			m_twgs_ramrd = BIT(data, 5);
			m_twgs_ramwrt = BIT(data, 4);
			m_twgs_lcram = !BIT(data, 3);
			m_twgs_lcram2 = BIT(data, 2);
			m_twgs_rombank = BIT(data, 1);
			break;
		}
	}
	switch (low)
	{
	case 0x54: case 0x55: m_twgs_page2 = BIT(low, 0); break;
	case 0x56: case 0x57: m_twgs_hires = BIT(low, 0); break;
	}
	// CD.Y decodes C080-C083 and C088-C08B. CC and CB clock on reads
	// and writes alike: CC captures A0 and CB captures A0 & old CC.Q.
	if ((low & 0xf4) == 0x80)
	{
		m_twgs_lcwrite = BIT(low, 0) && m_twgs_lcprewrite;
		m_twgs_lcprewrite = BIT(low, 0);
		m_twgs_lcram = (low & 3) == 0 || (low & 3) == 3;
		m_twgs_lcram2 = !BIT(low, 3);
	}
}

// Functional mapping of BL0X and the LC/ROM tag states. GAL5 F/G/H are
// interpreted as A8/A10/A9, consistent with zero/stack and the firmware text
// page test. The printed A10/A9/A8 labels do not satisfy that test.
// Shadow inhibits do not redirect CPU data to the slow-memory video copy.
u32 apple2gs_state::twgs_taddr(u32 address, int type) const
{
	const bool write = type == g65816_device::BUS_WRITE;
	const u16 low = address;
	u32 tag = address;
	if (twgs_io_bank(address))
	{
		bool aux = false;
		if (low < 0x0200 || low >= 0xd000)
			aux = m_twgs_altzp;
		else if (low < 0xc000)
		{
			if (m_twgs_80store && ((low >= 0x0400 && low < 0x0800) ||
				(m_twgs_hires && low >= 0x2000 && low < 0x4000)))
				aux = m_twgs_page2;
			else
				aux = write ? m_twgs_ramwrt : m_twgs_ramrd;
		}
		if (aux)
			tag |= 0x010000;
		// GAL4 AD0 includes the C029.D6 latch (HB.Q/O32). Its low-memory
		// terms distinguish linearized mappings in addition to the bank tag.
		if (m_twgs_linear && ((low >= 0x4000 && low < 0x8000) ||
			(BIT(address, 16) && ((low >= 0x2000 && low < 0x4000) ||
				(low >= 0x8000 && low < 0xa000)))))
			tag |= 0x1000000;
		if (low >= 0xd000)
		{
			if (!write && (!m_twgs_lcram || type == g65816_device::BUS_VECTOR))
				tag |= 0x1000000 | (m_twgs_rombank ? 0x4000000 : 0);
			else if (low < 0xe000 && m_twgs_lcram2)
				tag |= 0x2000000;
		}
	}
	return tag;
}

// ZipGS manual v1.04, SW1/1: Cxxx/Dxxx shadow/non-shadow aliasing.
// Infer a private IOLC latch from the IIgs map; it qualifies cycles, not tags.
// The GS register list (12 October 1990, C05D) confirms bank observation.
bool apple2gs_state::zip_iolc(u32 address) const
{
	const u32 bank = address >> 16;
	return (bank == 0xe0) || (bank == 0xe1) || ((bank <= 1) && !m_zip_iolc);
}

// Observe every CPU cycle, including fast FPI registers and both word bytes.
// US 4,794,523, col. 4: RWN qualifies write-only mapping switches.
void apple2gs_state::zip_snoop(u32 address, bool write, u8 data)
{
	if (!zip_iolc(address))
		return;
	const u16 a = address;
	if (write)
	{
		switch (a)
		{
		case 0xc000: case 0xc001: m_zip_store80 = BIT(a, 0); break;
		case 0xc002: case 0xc003: m_zip_ramrd = BIT(a, 0); break;
		case 0xc004: case 0xc005: m_zip_ramwrt = BIT(a, 0); break;
		case 0xc006: case 0xc007: m_zip_intcxrom = BIT(a, 0); break;
		case 0xc008: case 0xc009: m_zip_altzp = BIT(a, 0); break;
		case 0xc035: m_zip_iolc = BIT(data, 6); break;
		case 0xc068:
			m_zip_altzp = BIT(data, 7);
			m_zip_page2 = BIT(data, 6);
			m_zip_ramrd = BIT(data, 5);
			m_zip_ramwrt = BIT(data, 4);
			m_zip_lcram = !BIT(data, 3);
			m_zip_lcram2 = BIT(data, 2);
			m_zip_intcxrom = BIT(data, 0);
			break;
		}
	}
	switch (a)
	{
	case 0xc054: case 0xc055: m_zip_page2 = BIT(a, 0); break;
	case 0xc056: case 0xc057: m_zip_hires = BIT(a, 0); break;
	}
	if ((a & 0xfff0) == 0xc080)
	{
		m_zip_lcram = ((a & 3) == 0) || ((a & 3) == 3);
		m_zip_lcram2 = !BIT(a, 3);
	}
}

bool apple2gs_state::zip_cacheable(u32 address) const
{
	const u32 bank = address >> 16;
	const u16 a = address;
	const bool mapped = (bank <= 1) || (bank == 0xe0) || (bank == 0xe1);
	// SW1/1 disables Cxxx/Dxxx in both views, not the Exxx/Fxxx region.
	// With it clear, $C0xx stays uncached and $C1xx-$C7xx can cache. The
	// /IOSTROBE window is uncached only when this access is the selected card
	// (m_zip_c8_slot, set by c800_r/c800_w from the pre-clear owner). $C059.7
	// still bypasses $C000-$DFFF in the four mapped banks.
	const bool slot_window = zip_iolc(address) && (a >= 0xc800) && (a <= 0xcfff) && m_zip_c8_slot;
	return m_zip_bank_ok[bank]
		&& !(zip_iolc(address) && (a >= 0xc000) && (a < 0xc100))
		&& !slot_window
		&& !(mapped && BIT(m_accel_gsxsettings, 7) && (a >= 0xc000) && (a < 0xe000));
}

u32 apple2gs_state::zip_taddr(u32 address, bool write) const
{
	const u32 bank = address >> 16;
	const u16 a = address;
	u32 tag = address;
	const bool lc = zip_iolc(address) && (a >= 0xd000);
	if (((bank == 0x00) || (bank == 0xe0)) && ((a < 0xc000) || lc))
	{
		bool aux;
		if ((a < 0x0200) || lc)
			aux = m_zip_altzp;
		else if (m_zip_store80 && (((a >= 0x0400) && (a < 0x0800))
			|| (m_zip_hires && (a >= 0x2000) && (a < 0x4000))))
			aux = m_zip_page2;
		else
			aux = write ? m_zip_ramwrt : m_zip_ramrd;
		if (aux)
			tag |= 0x010000; // ZipGS mapping latches tag the physical auxiliary bank (US 4,794,523).
	}
	if (lc)
	{
		if (!write && !m_zip_lcram)
		{
			tag |= 0x1000000;
			// The original FPI selects an alternate ROM at bank 00 D000-FFFF.
			// Keep both ROM mappings distinct in the cache identity.
			if (bank == 0 && m_rombank && !m_is_rom3)
				tag |= 0x8000000;
		}
		else if ((a < 0xe000) && m_zip_lcram2)
			tag |= 0x2000000;
	}
	return tag;
}

u8 apple2gs_state::zip_access(u32 address, int type, u8 data)
{
	const bool write = type == g65816_device::BUS_WRITE;
	const bool cacheable = zip_cacheable(address);
	zip_snoop(address, write, data);
	m_zip_hit = false;
	if (!cacheable)
		return data;
	const u32 slot = address & m_zip_mask;
	const u32 tag = zip_taddr(address, write) + 1;
	// Even while slowed or disabled, CPU stores must update the cache.
	if (!write && (m_bt_mode != BT_ZIP))
		return data;
	m_zip_hit = !write && (m_zip_tag[slot] == tag);
	if (m_zip_hit)
	{
		return m_zip_data[slot];
	}
	m_zip_data[slot] = data;
	// C05E invalidates the next allocated write tag; C05F clears status.
	m_zip_tag[slot] = (write && m_zip_trash_tag) ? 0 : tag;
	if (write)
		m_zip_trash_tag = false;
	m_zip_tag_update = true;
	return data;
}

// Only CPU transactions reach this hook. Debugger and Lua inspection must not
// replace a cache byte without its tag. Capture the pre-access decode before a
// switch write (notably C035) changes the qualification of its own address.
u8 apple2gs_state::twdc_access(u32 offset, int type, u8 data)
{
	if (!m_twgs)
	{
		// C05D samples the CPU bank, not the program bank (long reads may
		// address a different bank). Only CPU cycles reach this hook.
		const u8 bank = offset >> 16;
		if (m_accel_unlocked && type != g65816_device::BUS_WRITE &&
			(offset & 0xffff) == 0xc05d &&
			(bank == 0xe0 || bank == 0xe1 || (bank < 2 && !(m_shadow & SHAD_IOLC))))
			return bank;
		if (m_accel_present && !m_native_ram)
			return zip_access(offset, type, data);
		return data;
	}
	if (m_twgs_real)
		return twgs_bus(offset, type, data);
	const bool write = type == g65816_device::BUS_WRITE;
	m_twgs_access_cacheable = type != g65816_device::BUS_VECTOR && (offset >> 16) != 0xbc &&
		!(twgs_io_bank(offset) && (offset & 0xf000) == 0xc000);
	m_twgs_access_tag = twgs_taddr(offset, type) + 1;
	m_twgs_access_hit = m_zip_tag[offset & m_twgs_mask] == m_twgs_access_tag;
	twgs_snoop(offset, write, data);
	if (!m_twgs_access_cacheable)
		return data;
	const u32 slot = offset & m_twgs_mask;
	if (write)
	{
		m_twgs_data[slot] = data;
		// The timing hook maintains tags at card speed. Writes still maintain
		// coherence when acceleration or the motherboard fast mode is disabled.
		if (m_bt_mode != BT_TWGS)
			m_zip_tag[slot] = m_twgs_access_tag;
		return data;
	}
	const bool code = type == g65816_device::BUS_OPCODE || type == g65816_device::BUS_OPERAND;
	const bool slow = type == g65816_device::BUS_OPCODE
		? (!BIT(m_twgs_config, 3) && m_maincpu->irq_masked()) : m_bt_twslow;
	if (m_bt_mode == BT_TWGS && !slow && (code || BIT(m_twgs_config, 1)) && m_twgs_access_hit)
	{
		return m_twgs_data[slot];
	}
	// Motherboard cycles fill SRAM even when the CPU cannot read the cache.
	m_twgs_data[slot] = data;
	m_zip_tag[slot] = m_twgs_access_tag;
	return data;
}

int apple2gs_state::bt_access(u32 address, int type, u8 data)
{
	const bool slow = m_bt_slow;
	m_bt_slow = false;
	m_bt_slot_ok = false;
	if (m_bt_mode == BT_OFF)
		return 0;

	u64 charge = 0;
	if (type == g65816_device::BUS_OPCODE)
	{
		charge = bt_instruction(address);
		if (REPSEP_SLOW_CYCLES && ((m_bt_mode == BT_ZIP) || (m_bt_mode == BT_TWGS)))
		{
			// The card decodes the opcode present on the CPU data pins,
			// including a cache/EPROM hit. Do not perform another RAM read.
			if ((data == 0xc2) || (data == 0xe2))
			{
				// REP and SEP: N of the instruction's cycles run at the motherboard speed (fast
				// cycles with refresh) in place of N card cycles, then the card clock again
				const u64 t0 = m_bt_istart + m_bt_idx * m_bt_cycle + m_bt_istall;
				u64 end = std::max(t0, m_bt_busfree);
				for (u32 i = 0; i < REPSEP_SLOW_CYCLES; i++)
					end = bt_fast(end, true);
				end = ((end + m_bt_cycle - 1) / m_bt_cycle) * m_bt_cycle;
				const u64 extra = (end > t0 + REPSEP_SLOW_CYCLES * m_bt_cycle) ? (end - t0 - REPSEP_SLOW_CYCLES * m_bt_cycle) : 0;
				m_bt_istall += extra;
				charge += extra;
			}
		}
	}

	const u64 t = m_bt_istart + m_bt_idx * m_bt_cycle + m_bt_istall;
	m_bt_idx++;

	// A cache hit has no motherboard transaction. Keep its counters and
	// instruction charge, without decoding or scheduling an unused bus slot.
	if (m_bt_mode == BT_TWGS && !m_bt_twslow && !m_twgs_post &&
		type != g65816_device::BUS_WRITE && (address >> 16) != 0xbc &&
		m_twgs_access_cacheable && m_twgs_access_hit &&
		(type == g65816_device::BUS_OPCODE || type == g65816_device::BUS_OPERAND || BIT(m_twgs_config, 1)))
	{
		m_btc[BTC_READS]++;
		m_btc[BTC_HITS]++;
		m_btpage[(address >> 8) * 4]++;
		m_bt_frac += charge;
		if (m_bt_frac < m_bt_cycle)
			return 0;
		const u64 n = m_bt_frac / m_bt_cycle;
		m_bt_frac -= n * m_bt_cycle;
		m_bt_charged += n;
		return int(n);
	}

	const u32 bank = address >> 16;
	const u32 a16 = address & 0xffff;
	const bool write = (type == g65816_device::BUS_WRITE);
	// banks with I/O and a language card at $C000-$FFFF
	const bool iolc = (bank >= 0xe0) ? (bank <= 0xe1) : ((bank <= 0x01) && !(m_shadow & SHAD_IOLC));
	const bool io = (m_twgs_real && m_twgs) ? m_twgs_access_io : (iolc && ((a16 & 0xf000) == 0xc000));
	int cls = BC_RAM;
	if (slow)
		cls = BC_MEGA;
	else if (io)
		cls = BC_FASTIO;
	else if ((bank >= 0xf0) || (iolc && (a16 >= 0xd000) && !m_lcram && !write))
		cls = BC_ROM;

	u64 *const page = &m_btpage[(address >> 8) * 4];
	u64 stall = 0;
	bool busread = false;
	if (write)
	{
		m_btc[BTC_WRITES]++;
		page[2]++;
	}
	else
	{
		m_btc[BTC_READS]++;
		page[0]++;
	}

	if (m_bt_mode == BT_STOCK)
	{
		stall = bt_bus(t, cls) - t - BT_FAST;
		if ((cls == BC_MEGA) && !write)
		{
			page[1]++;
			busread = true;
		}
	}
	else if (m_bt_mode == BT_ZIP)
	{
		const bool cacheable = zip_cacheable(address);
		if (write)
		{
			while (m_bt_wcount && (m_bt_wbuf[0] <= t))
			{
				for (int i = 1; i < m_bt_wcount; i++)
					m_bt_wbuf[i - 1] = m_bt_wbuf[i];
				m_bt_wcount--;
			}
			u64 accept = t;
			if (m_bt_wcount >= ZIP_WRITE_DEPTH)
			{
				// US 4,794,523 Table I: the CPU stops until the previous byte's
				// cycle ends, and CLR can latch the new byte only after DL1/DL2
				// see that end (two ZipGS clocks). The byte misses the PH2 cycle
				// that starts there, so a slow write takes the Mega II cycle after.
				accept = ((m_bt_wbuf[0] + m_bt_cycle - 1) / m_bt_cycle + 1) * m_bt_cycle;
				for (int i = 1; i < m_bt_wcount; i++)
					m_bt_wbuf[i - 1] = m_bt_wbuf[i];
				m_bt_wcount--;
				m_btc[BTC_WBUF_FULL]++;
			}
			const u64 end = bt_bus(std::max(accept, m_bt_busfree), cls);
			m_bt_busfree = end;
			m_bt_wbuf[m_bt_wcount++] = end;
			stall = accept - t;

		}
		else if (m_zip_hit)
		{
			m_btc[BTC_HITS]++;
		}
		else
		{
			// a motherboard read, after the buffered writes; then the ZipGS clock again
			const u64 end = bt_bus(std::max(t, m_bt_busfree), cls);
			m_bt_busfree = end;
			m_bt_wcount = 0;
			const u64 resume = ((end + m_bt_cycle - 1) / m_bt_cycle) * m_bt_cycle + BUS_READ_RESUME * m_bt_cycle;
			stall = resume - t - m_bt_cycle;
			page[1]++;
			busread = true;
			if (cacheable)
			{
				m_btc[BTC_MISSES]++;
				if (type == g65816_device::BUS_OPCODE)
					m_btc[BTC_MISS_OPCODE]++;
				else if (type == g65816_device::BUS_OPERAND)
					m_btc[BTC_MISS_OPERAND]++;
				else
					m_btc[BTC_MISS_DATA]++;
			}
			else
				m_btc[BTC_UNCACHED]++;
		}
	}
	else if (m_bt_mode == BT_NATIVE)
	{
		// Not a real card. Fast RAM, ROM and the FPI registers take one CPU cycle each, with
		// no refresh, cache or write buffer. An access to the Mega II side waits for its 1 MHz
		// cycle. Then the CPU continues at its next clock edge.
		if (cls == BC_MEGA)
		{
			const u64 end = bt_bus(t, cls);
			stall = ((end + m_bt_cycle - 1) / m_bt_cycle) * m_bt_cycle - t - m_bt_cycle;
			if (!write)
			{
				page[1]++;
				busread = true;
			}
		}
	}
	else
	{
		// TransWarp GS
		const bool cacheable = m_twgs_access_cacheable;
		const bool code = (type == g65816_device::BUS_OPCODE) || (type == g65816_device::BUS_OPERAND);
		u32 &tag = m_zip_tag[address & m_twgs_mask];
		// the tag is the memory used (G. Body schematic; GAL equations, A2DP): in
		// the I/O banks GAL5 gives the tag bank bit 0 of the aux memory (BL0X,
		// by RAMRD, RAMWRT, ALTZP, 80STORE/PAGE2/HIRES), and GAL4 adds the
		// language card state for $D000-$FFFF (AD0, AD1)
		const u32 taddr = m_twgs_access_tag - 1;
		if (bank == 0xbc && !m_twgs_real)
		{
			// The firmware interface does not use the motherboard bus.
		}
		else if (m_bt_twslow)
		{
			// interrupts off with the IRQ logic on: GS speed, one bus cycle for each access
			const u64 end = bt_bus(std::max(t, m_bt_busfree), cls);
			m_bt_busfree = end;
			m_bt_wcount = 0;
			stall = end - t - m_bt_cycle;
				tag = taddr + 1;
			if (!write)
			{
				m_btc[BTC_UNCACHED]++;
				page[1]++;
				busread = true;
			}
		}
		else if (bank == 0xbc)
		{
			// The ROM is also the source used to refill every cache slot on a flush.
			if (a16 >= 0x8000)
				tag = address + 1;
		}
		else if (write)
		{
				tag = taddr + 1;
			while (m_bt_wcount && (m_bt_wbuf[0] <= t))
			{
				for (int i = 1; i < m_bt_wcount; i++)
					m_bt_wbuf[i - 1] = m_bt_wbuf[i];
				m_bt_wcount--;
			}
			u64 accept = t;
			if (m_bt_wcount >= TWGS_WRITE_LATCHES)
			{
				// the write latches are full: wait for the oldest write
				accept = m_bt_wbuf[0];
				for (int i = 1; i < m_bt_wcount; i++)
					m_bt_wbuf[i - 1] = m_bt_wbuf[i];
				m_bt_wcount--;
				m_btc[BTC_WBUF_FULL]++;
			}
			const u64 end = bt_bus(std::max(accept, m_bt_busfree), cls);
			m_bt_busfree = end;
			m_bt_wbuf[m_bt_wcount++] = end;
			stall = accept - t;
		}
		else if (cacheable && (code || BIT(m_twgs_config, 1)) && m_twgs_access_hit)
		{
			m_btc[BTC_HITS]++;
		}
		else
		{
			// a GS read, after the latched writes; then the TransWarp clock again
			const u64 end = bt_bus(std::max(t, m_bt_busfree), cls);
			m_bt_busfree = end;
			m_bt_wcount = 0;
			const u64 resume = ((end + m_bt_cycle - 1) / m_bt_cycle) * m_bt_cycle + BUS_READ_RESUME * m_bt_cycle;
			stall = resume - t - m_bt_cycle;
			page[1]++;
			busread = true;
			if (cacheable)
				tag = taddr + 1;
			if (cacheable && (code || BIT(m_twgs_config, 1)))
			{
				m_btc[BTC_MISSES]++;
				if (type == g65816_device::BUS_OPCODE)
					m_btc[BTC_MISS_OPCODE]++;
				else if (type == g65816_device::BUS_OPERAND)
					m_btc[BTC_MISS_OPERAND]++;
				else
					m_btc[BTC_MISS_DATA]++;
			}
			else
				m_btc[BTC_UNCACHED]++;
		}
	}

	if (m_twgs_real && m_twgs_post)
	{
		m_twgs_post = false;
		m_twgs_wend[m_twgs_wcount - 1] = m_bt_busfree;
		twgs_schedule_write();
	}

	// $C03D was held in the device handler, which runs before this hook.
	// The byte reaches the GLU at m_bt_slot: the bus cycle that carries it.
	if (m_glu_hold && write && a16 == m_glu_hold_reg)
	{
		if (m_bt_slot_ok)
		{
			m_snd_force = bt_at(m_bt_slot);
			m_snd_force_on = true;
		}
		if (m_glu_hold_reg == 0xc03c)
			sndglu_ctrl_write(m_glu_hold_data);
		else if (m_glu_hold_reg == 0xc03d)
			sndglu_reg_write(m_glu_hold_data);
		else
			sndglu_addr_write(m_glu_hold_reg, m_glu_hold_data);
		m_snd_force_on = false;
		m_glu_hold = false;
	}

	if (stall)
	{
		m_bt_istall += stall;
		page[3] += stall / BT_PAGE_UNIT;
		m_btpc[m_bt_ipc * 3] += stall / BT_PAGE_UNIT;
		m_btc[write ? BTC_STALL_WRITE : BTC_STALL_READ] += stall;
	}
	if (busread)
		m_btpc[m_bt_ipc * 3 + 1]++;

	charge += stall;
	m_bt_frac += charge;
	const u64 n = m_bt_frac / m_bt_cycle;
	m_bt_frac -= n * m_bt_cycle;
	m_bt_charged += n;
	return int(n);
}

TIMER_DEVICE_CALLBACK_MEMBER(apple2gs_state::apple2_interrupt)
{
	// timer fires near the end of active video; handle events for the next line
	int scanline = param + 1;

	if (scanline == BORDER_TOP)
	{
		m_vbl = false;
	}
	else if (scanline == (192+BORDER_TOP))
	{
		m_vbl = true;

		// Mega II status flags are set even when the interrupt is disabled
		m_intflag |= INTFLAG_VBL;
		if (m_inten & 0x08)
		{
			raise_irq(IRQS_VBL);
		}

		m_adbmicro->set_input_line(m5074x_device::M5074X_INT1_LINE, ASSERT_LINE);

		bt_control();

		// 3.5 motor off timeout
		if (m_motoroff_time > 0)
		{
			m_motoroff_time--;
			if (m_motoroff_time == 0)
			{
				if (m_floppy[2]->get_device()) m_floppy[2]->get_device()->tfsel_w(0);
				if (m_floppy[3]->get_device()) m_floppy[3]->get_device()->tfsel_w(0);
			}
		}
	}
	else if (scanline == (192+BORDER_TOP+1))
	{
		m_adbmicro->set_input_line(m5074x_device::M5074X_INT1_LINE, CLEAR_LINE);
	}
	else if (scanline == ((256+BORDER_TOP) % m_screen->height()))
	{
		// LANGSEL is latched when wrapping scanline 256
		const int height = m_video->is_pal_video_mode() ? 312 : 262;
		if (height != m_screen->height())
		{
			// toggle 50/60 Hz, both use 65 1M cycles per scanline
			m_screen->configure(m_screen->width(), height, m_screen->visible_area(), HZ_TO_ATTOSECONDS(A2GS_1M.dvalue() / (65 * height)));
			if (scanline >= height)
				scanline -= height;
		}

		// "quarter" second IRQ occurs every 16 frames, ~3.75 Hz
		if ((m_screen->frame_number() & 0xf) == 0)
		{
			m_intflag |= INTFLAG_QUARTER;
			if (m_inten & 0x10)
			{
				raise_irq(IRQS_QTRSEC);
			}
		}
	}
	else if (scanline == m_screen->height())
		scanline = 0;

	m_scantimer->adjust(m_screen->time_until_pos(scanline, HPOS_VBL), scanline);
}

TIMER_DEVICE_CALLBACK_MEMBER(apple2gs_state::apple2_vgc)
{
	twgs_flush_due();
	// flush the previous scanline to ensure SCB or palette changes are visible
	m_screen->update_now();

	// timer fires during HBL past right border; handle events for the next line
	int scanline = param + 1;
	if (scanline >= m_screen->height())
		scanline -= m_screen->height(); // catch 50Hz toggle

	// scanline interrupts occur during HBL of each super hi-res line
	if ((m_video->get_newvideo() & 0x80) && (scanline >= BORDER_TOP) && (scanline < (BORDER_TOP + 200)))
	{
		u8 scb;
		const int shrline = scanline - BORDER_TOP;

		if (shrline & 1)
		{
			scb = m_megaii_ram[0x19e80 + (shrline >> 1)];
		}
		else
		{
			scb = m_megaii_ram[0x15e80 + (shrline >> 1)];
		}

		// latch scb on this cycle for subsequent palette lookups
		m_video->set_SHR_scb(shrline, scb);

		if (scb & 0x40)
		{
			// VGC status flags are set even when the interrupt is disabled
			m_vgcint |= VGCINT_SCANLINE;

			// trigger the interrupt if enabled
			if (m_vgcint & VGCINT_SCANLINEEN)
			{
				m_vgcint |= VGCINT_ANYVGCINT;
				raise_irq(IRQS_SCAN);
			}
		}
	}

	m_vgctimer->adjust(m_screen->time_until_pos(scanline, HPOS_VGC), scanline);
}

void apple2gs_state::rtc_vgc(int cko)
{
	if (cko) // one second IRQ
	{
		m_vgcint |= VGCINT_SECOND;
		if (m_vgcint & VGCINT_SECONDENABLE)
		{
			m_vgcint |= VGCINT_ANYVGCINT;
			raise_irq(IRQS_SECOND);
		}
	}
}

void apple2gs_state::palette_init(palette_device &palette)
{
	static const unsigned char apple2gs_palette[] =
	{
		0x0, 0x0, 0x0,  /* Black         $0              $0000 */
		0xd, 0x0, 0x3,  /* Deep Red      $1              $0D03 */
		0x0, 0x0, 0x9,  /* Dark Blue     $2              $0009 */
		0xd, 0x2, 0xd,  /* Purple        $3              $0D2D */
		0x0, 0x7, 0x2,  /* Dark Green    $4              $0072 */
		0x5, 0x5, 0x5,  /* Dark Gray     $5              $0555 */
		0x2, 0x2, 0xf,  /* Medium Blue   $6              $022F */
		0x6, 0xa, 0xf,  /* Light Blue    $7              $06AF */
		0x8, 0x5, 0x0,  /* Brown         $8              $0850 */
		0xf, 0x6, 0x0,  /* Orange        $9              $0F60 */
		0xa, 0xa, 0xa,  /* Light Gray    $A              $0AAA */
		0xf, 0x9, 0x8,  /* Pink          $B              $0F98 */
		0x1, 0xd, 0x0,  /* Light Green   $C              $01D0 */
		0xf, 0xf, 0x0,  /* Yellow        $D              $0FF0 */
		0x4, 0xf, 0x9,  /* Aquamarine    $E              $04F9 */
		0xf, 0xf, 0xf   /* White         $F              $0FFF */
	};

	for (int i = 0; i < 16; i++)
	{
		palette.set_pen_color(i,
			apple2gs_palette[(3*i)]*17,
			apple2gs_palette[(3*i)+1]*17,
			apple2gs_palette[(3*i)+2]*17);

		m_video->set_GS_border_color(i, rgb_t(apple2gs_palette[(3*i)]*17, apple2gs_palette[(3*i)+1]*17, apple2gs_palette[(3*i)+2]*17));
	}
}

/***************************************************************************
    I/O
***************************************************************************/
void apple2gs_state::auxbank_update()
{
	int ramwr = (m_ramrd ? 1 : 0) | (m_ramwrt ? 2 : 0);

	m_b0_0000bank.select(m_altzp ? 1 : 0);
	m_e0_0000bank.select(m_altzp ? 1 : 0);
	m_b0_0200bank.select(ramwr);
	m_e0_0200bank.select(ramwr);

	if (m_video->get_80store())
	{
		if (m_video->get_page2())
		{
			m_b0_0400bank.select(3);
			m_e0_0400bank.select(3);
		}
		else
		{
			m_b0_0400bank.select(0);
			m_e0_0400bank.select(0);
		}
	}
	else
	{
		m_b0_0400bank.select(ramwr);
		m_e0_0400bank.select(ramwr);
	}

	m_b0_0800bank.select(ramwr);
	m_e0_0800bank.select(ramwr);

	if ((m_video->get_80store()) && (m_video->get_hires()))
	{
		if (m_video->get_page2())
		{
			m_b0_2000bank.select(3);
			m_e0_2000bank.select(3);
		}
		else
		{
			m_b0_2000bank.select(0);
			m_e0_2000bank.select(0);
		}
	}
	else
	{
		m_b0_2000bank.select(ramwr);
		m_e0_2000bank.select(ramwr);
	}

	m_b0_4000bank.select(ramwr);
	m_e0_4000bank.select(ramwr);
}

void apple2gs_state::update_slotrom_banks()
{
	//printf("update_slotrom_banks: intcxrom %d slotc3rom %d\n", m_intcxrom, m_slotc3rom);

	if ((m_intcxrom) || (!m_slotc3rom))
	{
		m_c300bank->set_bank(1);
	}
	else
	{
		m_c300bank->set_bank(0);
	}
}

void apple2gs_state::lc_update(int offset, bool writing)
{
	bool old_lcram = m_lcram;

	// any even access disables pre-write and writing
	if ((offset & 1) == 0)
	{
		m_lcprewrite = false;
		m_lcwriteenable = false;
	}

	// any write disables pre-write
	// has no effect on write-enable if writing was enabled already
	if (writing == true)
	{
		m_lcprewrite = false;
	}
	// first odd read enables pre-write, second one enables writing
	else if ((offset & 1) == 1)
	{
		if (m_lcprewrite == false)
		{
			m_lcprewrite = true;
		}
		else
		{
			m_lcwriteenable = true;
		}
	}

	switch (offset & 3)
	{
		case 0:
		case 3:
		{
			m_lcram = true;
			break;
		}

		case 1:
		case 2:
		{
			m_lcram = false;
			break;
		}
	}

	m_lcram2 = false;

	if (!(offset & 8))
	{
		m_lcram2 = true;
	}

	if (m_lcram != old_lcram)
	{
		lcrom_update();
	}

	#if 0
	printf("LC: new state %c%c dxxx=%04x altzp=%d\n",
			m_lcram ? 'R' : 'x',
			m_lcwriteenable ? 'W' : 'x',
			m_lcram2 ? 0x1000 : 0x0000,
			m_altzp);
	#endif
}

void apple2gs_state::lcrom_update()
{
	if (m_lcram)
	{
		m_lcbank.select(1);
		m_lcaux.select(1);
		m_lc00.select(1 + ((m_rombank && !m_is_rom3) ? 2 : 0));
		m_lc01.select(1);
	}
	else
	{
		m_lcbank.select(0);
		m_lcaux.select(0);
		m_lc00.select(0 + ((m_rombank && !m_is_rom3) ? 2 : 0));
		m_lc01.select(0);
	}
}

// most softswitches don't care about read vs write, so handle them here
const char *const apple2gs_state::s_glu_cls[7] = { "flo", "fhi", "vol", "ptr", "ctl", "siz", "oth" };

// The $C03C nibble is the amplifier after the DOC/speaker sum.
// Step 0 is mute. No resistor values for a log taper were found, so
// steps 1..15 are the linear fraction n/15 of full scale.
static float sndglu_vca_gain(u8 step)
{
	return float(step & 0x0f) / 15.0f;
}

void apple2gs_state::sndglu_apply_vca()
{
	// The output choice changes only at reset. Changing a route gain while running drops output
	// samples, so the VCA is a gain on the two sources instead.
	const float mono = m_snd_demux ? 0.0f : 1.0f;
	const float side = m_snd_demux ? 1.0f : 0.0f;
	if ((mono != m_snd_gain_mono) || (side != m_snd_gain_side))
	{
		m_snd_gain_mono = mono;
		m_snd_gain_side = side;
		m_doc->set_route_gain(0, m_sndmono, 0, mono);
		m_doc->set_route_gain(1, m_sndmono, 0, mono);
		m_doc->set_route_gain(0, m_sndright, 0, side);
		m_doc->set_route_gain(1, m_sndleft, 0, side);
		m_speaker->set_route_gain(0, m_sndmono, 0, mono);
		m_speaker->set_route_gain(0, m_sndleft, 0, side);
		m_speaker->set_route_gain(0, m_sndright, 0, side);
	}
	const float g = sndglu_vca_gain(m_sndglu_ctrl);
	if (g != m_snd_vca)
	{
		m_snd_vca = g;
		m_doc->set_output_gain(ALL_OUTPUTS, g);
		m_speaker->set_output_gain(ALL_OUTPUTS, g);
	}
}

// Bus-model ticks (1/1056 of a 14M clock) as an absolute machine time.
attotime apple2gs_state::bt_at(u64 t) const
{
	const u64 secs = t / BT_PER_SEC;
	const u64 rem = t % BT_PER_SEC;
	return attotime(secs, attoseconds_t(double(rem) * (double(ATTOSECONDS_PER_SECOND) / double(BT_PER_SEC))));
}

// A GLU access reaches the GLU when its own bus cycle does. Zip and TransWarp
// register writes set m_snd_force to that cycle.
// Anything else still sees the bus once earlier buffered writes have finished.
attotime apple2gs_state::sndglu_now() const
{
	if (m_snd_force_on)
		return m_snd_force;
	if (m_bt_mode == BT_ZIP || m_bt_mode == BT_NATIVE)
	{
		// ZipGS writes arrive forced, so this is a $C03C-$C03F read. The GLU
		// select is a PH0 strobe: the read gets there only in the Mega II
		// cycle that bt_access will charge for it, after any posted write.
		const u64 t = m_bt_istart + m_bt_idx * m_bt_cycle + m_bt_istall;
		return bt_at(bt_mega_start(std::max(t, m_bt_busfree)));
	}
	const attotime now = machine().time();
	if (m_bt_mode != BT_TWGS)
		return now;
	const attotime bus = bt_at(m_bt_busfree);
	return (bus > now) ? bus : now;
}

bool apple2gs_state::sndglu_service()
{
	return sndglu_service(sndglu_now());
}

bool apple2gs_state::sndglu_service(attotime time)
{
	if (!m_sndglu_pending)
		return false;
	if (machine().side_effects_disabled())
		return time < m_sndglu_busy_until;
	// Service by event time, not timer-dispatch time. This also runs before
	// the DOC advances, so neither its scan nor its audio can pass a due
	// transfer. The latch is cleared before entering the DOC again.
	while (m_sndglu_pending && m_sndglu_busy_until <= time)
		sndglu_commit();
	return m_sndglu_pending;
}

// Capture the instruction PC and access bank at the CPU bus hook. A later
// deferred GLU write uses this context; the CPU may have advanced by then.
void apple2gs_state::sndglu_log_note(u32 address, int type)
{
	if (type == g65816_device::BUS_OPCODE)
		m_glu_log_instruction_pc = address;
	if ((address & 0xfffc) == 0xc03c)
		m_glu_log_access_bank = address >> 16;
}

bool apple2gs_state::sndglu_log_memory(u32 address, int type, u8 &data)
{
	sndglu_log_note(address, type);
	return m_twgs_real && m_twgs ? twgs_memory(address, type, data) : false;
}

std::string apple2gs_state::sndglu_log_prefix(const glu_log_context &event) const
{
	const std::string gap = event.gap < 0 ? "first" : util::string_format("%lld", (long long)event.gap);
	return util::string_format("Sound GLU t=%.9f PC=%02X:%04X bank=%02X gap=%s bus cycles CPU=%.3fMHz accelerator=%s/%.3fMHz: ",
		event.time.as_double(), event.pc >> 16, event.pc & 0xffff, event.bank, gap,
		event.cpu_hz / 1.0e6, event.accelerator, event.accelerator_hz / 1.0e6);
}

std::string apple2gs_state::sndglu_log_cpu_state()
{
	const unsigned p = m_maincpu->state_int(g65816_device::G65816_P);
	return util::string_format("A=%04X X=%04X Y=%04X S=%04X D=%04X DBR=%02X P=%02X (m=%u x=%u e=%u)",
		unsigned(m_maincpu->state_int(g65816_device::G65816_A)),
		unsigned(m_maincpu->state_int(g65816_device::G65816_X)),
		unsigned(m_maincpu->state_int(g65816_device::G65816_Y)),
		unsigned(m_maincpu->state_int(g65816_device::G65816_S)),
		unsigned(m_maincpu->state_int(g65816_device::G65816_D)),
		unsigned(m_maincpu->state_int(g65816_device::G65816_DB)), p,
		BIT(p, 5), BIT(p, 4), unsigned(m_maincpu->state_int(g65816_device::G65816_E)));
}

std::string apple2gs_state::sndglu_log_stack()
{
	// Pushed data can resemble return addresses. Verify a preceding call
	// opcode when possible, scan upward from S, and never execute CPU hooks.
	auto const disable = machine().disable_side_effects();
	auto &space = m_maincpu->space(AS_PROGRAM);
	const u16 s = m_maincpu->state_int(g65816_device::G65816_S);
	const bool emulation = m_maincpu->state_int(g65816_device::G65816_E);
	auto stack_byte = [&space, s, emulation](unsigned offset)
	{
		const u16 address = emulation ? 0x100 | ((s + offset) & 0xff) : u16(s + offset);
		return space.read_byte(address);
	};
	unsigned bank = m_glu_log_context.pc >> 16, found = 0;
	std::string text;
	for (unsigned offset = 1; offset <= 62 && found < 3; ++offset)
	{
		const u16 saved = stack_byte(offset) | (u16(stack_byte(offset + 1)) << 8);
		const unsigned long_bank = stack_byte(offset + 2);
		const bool jsl = space.read_byte((long_bank << 16) | u16(saved - 3)) == 0x22;
		const u8 short_opcode = space.read_byte((bank << 16) | u16(saved - 2));
		const bool jsr = short_opcode == 0x20 || short_opcode == 0xfc;
		if (!jsl && !jsr)
			continue;
		if (jsl)
			bank = long_bank;
		if (found++)
			text += ", ";
		text += util::string_format("%02X:%04X (%s at S+%u)", bank, u16(saved + 1), jsl ? "JSL" : "JSR", offset);
		offset += jsl ? 2 : 1;
	}
	return text.empty() ? "no plausible return addresses in 64 bytes" : text;
}

void apple2gs_state::sndglu_log_event(unsigned kind, std::string details)
{
	++m_glu_log_counts[kind];
	const std::string prefix = sndglu_log_prefix(m_glu_log_context);
	std::string state;
	if (VERBOSE & LOG_GLUTRACE)
		state = "; " + sndglu_log_cpu_state();
	LOGMASKED(LOG_GLU, "%s%s%s\n", prefix, details, state);
	static const char *const kinds[] = { "busy data write", "busy control write", "busy address-low write", "busy address-high write", "busy data read", "data setup change" };
	LOGMASKED(LOG_GLUTRACE, "%sCPU/stack for %s%s; call trace (best guess; pushed data may look like returns): %s\n",
		prefix, kinds[kind], state, sndglu_log_stack());
}

void apple2gs_state::sndglu_log_summary()
{
	if (std::none_of(std::begin(m_glu_log_counts), std::end(m_glu_log_counts), [](u64 count) { return count != 0; }))
		return;
	LOGMASKED(LOG_GLU | LOG_GLUTRACE, "%slast interval: busy data writes=%llu, busy control writes=%llu, busy address-low writes=%llu, busy address-high writes=%llu, busy data reads=%llu, setup changes=%llu\n",
		sndglu_log_prefix(m_glu_log_context), (unsigned long long)m_glu_log_counts[0], (unsigned long long)m_glu_log_counts[1],
		(unsigned long long)m_glu_log_counts[2], (unsigned long long)m_glu_log_counts[3],
		(unsigned long long)m_glu_log_counts[4], (unsigned long long)m_glu_log_counts[5]);
	std::fill(std::begin(m_glu_log_counts), std::end(m_glu_log_counts), 0);
}

void apple2gs_state::sndglu_log_access(unsigned port, bool read, u8 data, u8 result)
{
	if (machine().side_effects_disabled())
		return;
	const attotime now = sndglu_now();
	const u64 ticks = now.as_ticks(A2GS_14M);
	const int64_t cycle = (ticks / 912) * 65 + std::min<u64>((ticks % 912) / 14, 64);
	m_glu_log_context = { now, m_glu_log_last_cycle < 0 ? -1 : cycle - m_glu_log_last_cycle,
		m_glu_log_instruction_pc, u32(m_bt_twslow ? BT_PER_SEC / BT_FAST : (m_bt_mode == BT_OFF ? m_maincpu->clock() : BT_PER_SEC / m_bt_cycle)),
		m_accel_present ? m_accel_speed : 0, m_glu_log_access_bank, m_accel_present ? (m_twgs ? "TransWarp GS" : "ZipGS") : "none" };
	m_glu_log_last_cycle = cycle;
	if (now >= m_glu_log_summary_at)
	{
		sndglu_log_summary();
		m_glu_log_summary_at = now + attotime::from_seconds(10);
	}
	auto destination = [](u16 address, bool ram)
	{
		return util::string_format("%s $%04X", ram ? "sound RAM" : "DOC register", ram ? address : address & 0xff);
	};
	const bool busy = m_sndglu_pending && now < m_sndglu_busy_until;
	if (busy && !read && port != 13)
	{
		const u16 next = port == 12 ? (BIT(data, 6) ? m_sndglu_addr : m_sndglu_addr & 0xff)
			: port == 14 ? (m_sndglu_addr & 0xff00) | data : (m_sndglu_addr & 0xff) | (u16(data) << 8);
		sndglu_log_event(port == 12 ? 1 : port - 12,
			util::string_format("$C03%X write $%02X while busy; local register changes; pending %s will use %s (was %s)",
				port, data, m_sndglu_pend_read ? "read" : "write", destination(next, m_sndglu_pend_ram), destination(m_sndglu_addr, m_sndglu_pend_ram)));
	}
	if (busy && port == 13)
	{
		const std::string requested = destination(m_glu_log_intended_address, BIT(m_glu_log_intended_control, 6));
		const std::string target = destination(m_sndglu_addr, BIT(m_sndglu_ctrl, 6));
		if (read)
			sndglu_log_event(4, util::string_format("$C03D read while busy returned $%02X; pending transfer becomes a read of %s at the existing deadline; program requested %s", result, target, requested));
		else
		{
			const std::string old = m_sndglu_pend_read ? "pending read" : util::string_format("pending byte $%02X", m_sndglu_pend_data);
			sndglu_log_event(0, util::string_format("$C03D write $%02X while busy replaces %s; transfer goes to %s at the existing deadline; program requested %s", data, old, target, requested));
			if (!m_sndglu_pend_read && !m_sndglu_pend_ram && data != m_sndglu_pend_data && m_sndglu_busy_until - now < attotime::from_nsec(150))
				sndglu_log_event(5, util::string_format("DOC data changed $%02X -> $%02X inside 150ns setup; %.1fns before transfer to %s", m_sndglu_pend_data, data, (m_sndglu_busy_until - now).as_double() * 1.0e9, target));
		}
	}
	if (port == 13 && BIT(m_glu_log_intended_control, 5))
		++m_glu_log_intended_address;
	else if (!read && port == 12)
		m_glu_log_intended_control = data & 0x7f;
	else if (!read && port == 14)
		m_glu_log_intended_address = (m_glu_log_intended_address & 0xff00) | data;
	else if (!read && port == 15)
		m_glu_log_intended_address = (m_glu_log_intended_address & 0xff) | (u16(data) << 8);
}

void apple2gs_state::sndglu_ctrl_write(u8 data)
{
	sndglu_service();
	if (m_glu_log_enabled)
		sndglu_log_access(12, false, data);
	m_sndglu_ctrl = data & 0x7f; // bit 7 (busy) is read-only
	if (!(m_sndglu_ctrl & 0x40)) // clear hi byte of address pointer on DOC access
		m_sndglu_addr &= 0xff;
	sndglu_apply_vca();
}

void apple2gs_state::sndglu_arm(bool is_read, u8 data)
{
	const attotime now = sndglu_now();
	const u64 ticks = now.as_ticks(A2GS_7M);
	// Time to the next free phase, plus three DOC clocks for the strobe.
	// Missing the phase waits from three clocks to a slot plus three.
	const u64 commit_tick = (ticks / 8 + 1) * 8 + 3;
	m_sndglu_pend_read = is_read;
	m_sndglu_pend_ram = (m_sndglu_ctrl & 0x40) != 0;
	m_sndglu_pend_auto = (m_sndglu_ctrl & 0x20) != 0;
	m_sndglu_pend_addr = m_sndglu_addr;
	m_sndglu_pend_data = data;
	m_sndglu_busy_until = attotime::from_ticks(commit_tick, A2GS_7M);
	m_sndglu_pending = true;
	// Absolute commit time. now may be a future bus cycle; scheduling the
	// interval from now would fire early and drop the collision.
	const attotime left = m_sndglu_busy_until - machine().time();
	m_sndglu_timer->adjust((left < attotime::zero) ? attotime::zero : left);
}

// Register write of $C03D. A write that arrives while the latch is still
// waiting replaces the byte. The commit time does not move.
void apple2gs_state::sndglu_addr_write(u16 reg, u8 data)
{
	sndglu_service();
	if (m_glu_log_enabled)
		sndglu_log_access(reg & 15, false, data);
	if (reg == 0xc03e)
		m_sndglu_addr = (m_sndglu_addr & 0xff00) | data;
	else
		m_sndglu_addr = (m_sndglu_addr & 0x00ff) | (data << 8);
}

void apple2gs_state::sndglu_reg_write(u8 data)
{
	const bool busy = sndglu_service();
	if (m_glu_log_enabled)
		sndglu_log_access(13, false, data);
	if (busy)
	{
		logerror("Sound GLU busy: write %02x to %04x dropped\n", m_sndglu_pend_data, m_sndglu_pend_addr);
		sndglu_tally(m_sndglu_pend_addr, false);
		m_sndglu_pend_read = false;
		m_sndglu_pend_ram = (m_sndglu_ctrl & 0x40) != 0;
		m_sndglu_pend_auto = (m_sndglu_ctrl & 0x20) != 0;
		m_sndglu_pend_addr = m_sndglu_addr;
		m_sndglu_pend_data = data;
		return;
	}
	sndglu_arm(false, data);
}

void apple2gs_state::sndglu_tally(u16 addr, bool land)
{
	const int osc = addr & 0x1f;
	int cls;
	switch ((addr >> 5) & 7)
	{
		case 0: cls = 0; break;
		case 1: cls = 1; break;
		case 2: cls = 2; break;
		case 4: cls = 3; break;
		case 5: cls = 4; break;
		case 6: cls = 5; break;
		default: cls = 6; break;
	}
	if (land)
		m_glu_land[osc][cls]++;
	else
		m_glu_repl[osc][cls]++;
}

void apple2gs_state::sndglu_log_table()
{
	logerror("Sound GLU vca %02x\n", m_sndglu_ctrl & 0x0f);
	logerror("Sound GLU ram bytes %u\n", m_glu_ram);
	for (int osc = 0; osc < 32; osc++)
	{
		bool any = false;
		for (int c = 0; c < 7; c++)
			any = any || m_glu_land[osc][c] || m_glu_repl[osc][c];
		if (!any)
			continue;
		logerror("Sound GLU osc %02d", osc);
		for (int c = 0; c < 7; c++)
			logerror(" %s %u/%u", s_glu_cls[c], m_glu_land[osc][c], m_glu_repl[osc][c]);
		logerror("\n");
	}
}

void apple2gs_state::sndglu_commit()
{
	if (!m_sndglu_pending)
		return;
	m_sndglu_pending = false;
	m_sndglu_timer->adjust(attotime::never);
	// The GLU drives the pointer on SND-LA at the sample point, so a pointer
	// store that arrives while a byte waits redirects that byte.
	const u16 addr = m_sndglu_addr;
	if (!m_sndglu_pend_read && (addr != m_sndglu_pend_addr))
		logerror("Sound GLU redirect: %02x for %04x went to %04x\n", m_sndglu_pend_data, m_sndglu_pend_addr, addr);
	if (m_sndglu_pend_read)
	{
		if (m_sndglu_pend_ram)
		{
			m_doc->synchronize_to(m_sndglu_busy_until);
			m_sndglu_dummy_read = m_docram[addr];
		}
		else
			m_sndglu_dummy_read = m_doc->read_at(addr, m_sndglu_busy_until);
	}
	else if (m_sndglu_pend_ram)
	{
		m_doc->synchronize_to(m_sndglu_busy_until);
		m_docram[addr] = m_sndglu_pend_data;
		m_glu_ram++;
	}
	else
	{
		m_doc->write_at(addr, m_sndglu_pend_data, m_sndglu_busy_until);
		sndglu_tally(addr, true);
	}
	// The pointer increments only if the CPU has not stored a new address
	// since this transfer was accepted. A store to $C03E/$C03F replaces
	// the pointer; the completing cycle must not add one to that new value.
	if (m_sndglu_pend_auto && m_sndglu_addr == m_sndglu_pend_addr)
		m_sndglu_addr = (addr + 1) & 0xffff;
	if (machine().time() >= m_glu_log_at)
	{
		m_glu_log_at += attotime::from_seconds(10);
		sndglu_log_table();
	}
}

TIMER_CALLBACK_MEMBER(apple2gs_state::sndglu_commit_cb)
{
	sndglu_service(machine().time());
}

void apple2gs_state::do_io(int offset)
{
	if(machine().side_effects_disabled()) return;

	switch (offset)
	{
		case 0x28:  // ROMSWITCH (original FPI; ROM 03 decode not established)
			if (!m_is_rom3)
			{
				// C068.1 reads/writes the same ROM bank latch.
				m_rombank = !m_rombank;
				lcrom_update();
			}
			break;

		case 0x2a:  // 16-bit access to NEWVIDEO
			break;

		case 0x2e:  // VERTCNT
		case 0x7e:
			// Zip counter delay
			accel_temp_delay(5, BIT(m_accel_gsxsettings, 4));
			break;

		case 0x30:  // SPKR
			m_speaker_state ^= 1;
			m_speaker->level_w(m_speaker_state);

			accel_temp_delay(5, (m_accel_slotspk & 1));
			break;

		case 0x37:  // DMAREG, 16-bit access to CYAREG
			break;

		case 0x47:  // CLRVBLINT
			m_intflag &= ~(INTFLAG_VBL | INTFLAG_QUARTER);
			lower_irq(IRQS_VBL);
			lower_irq(IRQS_QTRSEC);
			break;

		case 0x50:  // graphics mode
			m_video->txt_w(0);
			break;

		case 0x51:  // text mode
			m_video->txt_w(1);
			break;

		case 0x52:  // no mix
			m_video->mix_w(0);
			break;

		case 0x53:  // mixed mode
			m_video->mix_w(1);
			break;

		case 0x54:  // set page 1
			m_video->scr_w(0);
			auxbank_update();
			break;

		case 0x55:  // set page 2
			m_video->scr_w(1);
			auxbank_update();
			break;

		case 0x56: // select lo-res
			m_video->res_w(0);
			auxbank_update();
			break;

		case 0x57: // select hi-res
			m_video->res_w(1);
			auxbank_update();
			break;

		case 0x58: // AN0 off
			m_an0 = false;
			m_gameio->an0_w(0);
			break;

		case 0x59: // AN0 on
			m_an0 = true;
			m_gameio->an0_w(1);
			break;

		case 0x5a: // AN1 off
			m_an1 = false;
			m_gameio->an1_w(0);
			break;

		case 0x5b: // AN1 on
			m_an1 = true;
			m_gameio->an1_w(1);
			break;

		case 0x5c: // AN2 off
			m_an2 = false;
			m_gameio->an2_w(0);
			break;

		case 0x5d: // AN2 on
			m_an2 = true;
			m_gameio->an2_w(1);
			break;

		case 0x5e: // AN3 off, DHIRESON
			m_an3 = false;
			m_gameio->an3_w(0);
			m_video->an3_w(0);
			break;

		case 0x5f: // AN3 on, DHIRESOFF
			m_an3 = true;
			m_gameio->an3_w(1);
			m_video->an3_w(1);
			break;

		case 0x69:  // 16-bit access to STATEREG
			break;

		case 0x70:  // PTRIG triggers paddles on read or write
			// 558 monostable one-shot timers; a running timer cannot be restarted
			if (machine().time().as_double() >= m_joystick_x1_time)
			{
				m_joystick_x1_time = machine().time().as_double() + m_x_calibration * m_gameio->pdl0_r();
			}
			if (machine().time().as_double() >= m_joystick_y1_time)
			{
				m_joystick_y1_time = machine().time().as_double() + m_y_calibration * m_gameio->pdl1_r();
			}
			if (machine().time().as_double() >= m_joystick_x2_time)
			{
				m_joystick_x2_time = machine().time().as_double() + m_x_calibration * m_gameio->pdl2_r();
			}
			if (machine().time().as_double() >= m_joystick_y2_time)
			{
				m_joystick_y2_time = machine().time().as_double() + m_y_calibration * m_gameio->pdl3_r();
			}
			[[fallthrough]];
		case 0x20:
			// Zip paddle delay
			accel_temp_delay(5, BIT(m_accel_gsxsettings, 6));
			break;

		default:
			logerror("do_io: unknown switch $C0%02X\n", offset);
			break;
	}
}

// return the correct vertical counter per IIgs Tech Note #39
int apple2gs_state::get_vpos()
{
	int vpos = m_screen->vpos();
	vpos += (m_screen->hpos() >= (BORDER_LEFT + (40 - ALIGN_CNT) * 16)); // adjust for set_raw

	if (vpos < BORDER_TOP)
		vpos += m_screen->height(); // remap top border to bottom of VBL
	vpos += 256 - BORDER_TOP;
	if (vpos > 511)
		vpos -= m_screen->height(); // wrap scanline 256 to 250 (60 Hz) or 200 (50 Hz)

	return vpos;
}

void apple2gs_state::process_clock()
{
	for (int i = 0; i < 8; i++)
	{
		m_rtc->clk_w(ASSERT_LINE);
		if (!BIT(m_clock_control, 6))
		{
			m_rtc->data_w(BIT(m_clkdata, 7-i));
			m_rtc->clk_w(CLEAR_LINE);
		}
		else
		{
			m_rtc->clk_w(CLEAR_LINE);
			m_clkdata <<= 1;
			m_clkdata |= m_rtc->data_r() & 1;
		}
	}
}

void apple2gs_state::clear_vgcint(u8 data)
{
	if (!(data & VGCINT_SECOND))
	{
		m_vgcint &= ~VGCINT_SECOND;
		lower_irq(IRQS_SECOND);
	}
	if (!(data & VGCINT_SCANLINE))
	{
		m_vgcint &= ~VGCINT_SCANLINE;
		lower_irq(IRQS_SCAN);
	}
	if (!(m_vgcint & (VGCINT_SECOND | VGCINT_SCANLINE)))
	{
		m_vgcint &= ~VGCINT_ANYVGCINT;
	}
}

u8 apple2gs_state::c000_r(offs_t offset)
{
	u8 ret;

	if (offset < 0x71)
	{
		switch (offset)
		{
			// C07x ROM and FPI registers are fast. $C03C-$C03F are a Mega II
			// phase 0 (Tech Note #68: /M2SEL for internal I/O; the GLU
			// select is the phase-0 strobe C038-3F*).
			case 0x2d: case 0x35: case 0x36: case 0x37: case 0x68:
				break;
			default:
				slow_cycle();
		}
	}

	if (!machine().side_effects_disabled())
	{
		switch (offset)
		{
			case 0x38:  // SCCBREG
				return m_scc->cb_r(0);

			case 0x39:  // SCCAREG
				return m_scc->ca_r(0);

			case 0x3a:  // SCCBDATA
				return m_scc->db_r(0);

			case 0x3b:  // SCCADATA
				return m_scc->da_r(0);
		}
	}

	const u8 uFloatingBus = read_floatingbus(); // video side-effects latch after reading
	const u8 uFloatingBus7 = uFloatingBus & 0x7f;
	const u8 uKeyboard = keyglu_816_read(GLU_C000);

	switch (offset)
	{
		// keyboard latch
		case 0x00: case 0x01: case 0x02: case 0x03: case 0x04: case 0x05: case 0x06:
		case 0x07: case 0x08: case 0x09: case 0x0a: case 0x0b: case 0x0c: case 0x0d:
		case 0x0e: case 0x0f:
			return uKeyboard;

		case 0x10:  // read any key down, reset keyboard strobe
			if (!machine().side_effects_disabled())
				keyglu_816_write(GLU_C010, 0);
			return keyglu_816_read(GLU_C010) | (uKeyboard & 0x7f);

		// Mega II status replaces D7; D0-D6 retain the keyboard latch,
		// as in the IIe soft-switch readback (apple2e_state::c000_r).
		case 0x11:  // read LCRAM2 (LC Dxxx bank)
			return (m_lcram2 ? 0x80 : 0x00) | (uKeyboard & 0x7f);

		case 0x12:  // read LCRAM (is LC readable?)
			return (m_lcram ? 0x80 : 0x00) | (uKeyboard & 0x7f);

		case 0x13:  // read RAMRD
			return (m_ramrd ? 0x80 : 0x00) | (uKeyboard & 0x7f);

		case 0x14:  // read RAMWRT
			return (m_ramwrt ? 0x80 : 0x00) | (uKeyboard & 0x7f);

		case 0x15:  // read INTCXROM
			return (m_intcxrom ? 0x80 : 0x00) | (uKeyboard & 0x7f);

		case 0x16:  // read ALTZP
			return (m_altzp ? 0x80 : 0x00) | (uKeyboard & 0x7f);

		case 0x17:  // read SLOTC3ROM
			return (m_slotc3rom ? 0x80 : 0x00) | (uKeyboard & 0x7f);

		case 0x18:  // read 80STORE
			return (m_video->get_80store() ? 0x80 : 0x00) | (uKeyboard & 0x7f);

		case 0x19:  // read VBL (not VBLBAR, see Apple IIGS Technical Note #40)
			return (m_vbl ? 0x80 : 0x00) | (uKeyboard & 0x7f);

		case 0x1a:  // read TEXT
			return (m_video->get_graphics() ? 0x00 : 0x80) | (uKeyboard & 0x7f);

		case 0x1b:  // read MIXED
			return (m_video->get_mix() ? 0x80 : 0x00) | (uKeyboard & 0x7f);

		case 0x1c:  // read PAGE2
			return (m_video->get_page2() ? 0x80 : 0x00) | (uKeyboard & 0x7f);

		case 0x1d:  // read HIRES
			return (m_video->get_hires() ? 0x80 : 0x00) | (uKeyboard & 0x7f);

		case 0x1e:  // read ALTCHARSET
			return (m_video->get_altcharset() ? 0x80 : 0x00) | (uKeyboard & 0x7f);

		case 0x1f:  // read 80COL
			return (m_video->get_80col() ? 0x80 : 0x00) | (uKeyboard & 0x7f);

		case 0x22:  // TEXTCOL
			return m_video->get_GS_textcol();

		case 0x23:  // VGCINT
			return m_vgcint;

		case 0x24:  // MOUSEDATA
			return keyglu_816_read(GLU_MOUSEX);

		case 0x25:  // KEYMODREG
			return keyglu_816_read(GLU_KEYMOD);

		case 0x26:  // DATAREG
			return keyglu_816_read(GLU_DATA);

		case 0x27:  // KMSTATUS
			return keyglu_816_read(GLU_SYSSTAT);

		case 0x29:  // NEWVIDEO
			return m_video->get_newvideo();

		case 0x2b:  // LANGSEL
			return m_video->get_GS_langsel();

		case 0x2d:  // SLOTROMSEL
			return m_slotromsel;

		case 0x2e:  // VERTCNT
			do_io(offset); // Zip delay

			// reading VERTCNT clears SCB status
			if (!machine().side_effects_disabled())
				clear_vgcint(~VGCINT_SCANLINE);

			return get_vpos() >> 1;

		case 0x2f:  // HORIZCNT
			// reading HORIZCNT clears SCB status
			if (!machine().side_effects_disabled())
				clear_vgcint(~VGCINT_SCANLINE);

			ret = (m_screen->hpos() - BORDER_LEFT) / 16 + (25 + ALIGN_CNT); // adjust for set_raw
			if (ret >= 65)
				ret -= 65;
			if (ret > 0)
			{
				ret += 0x3f; // HBL [0, 0x40...57] active [0x58...7f]
			}

			if (get_vpos() & 1)
			{
				ret |= 0x80;
			}
			return ret;

		case 0x31:  // DISKREG
			return m_diskreg;

		case 0x32: // VGCINTCLEAR
			return uFloatingBus;

		case 0x33: // CLOCKDATA
			return m_clkdata;

		case 0x34:  // BORDERCOL
			return (m_clock_control & 0xf0) | (m_video->get_GS_border() & 0xf);

		case 0x35:  // SHADOW
			return m_shadow;

		case 0x36:  // SPEED/CYAREG
			return m_speed;

		case 0x37:  // DMAREG. Slot /DMA has no bank pins; the FPI/CYA supplies this byte.
			return m_a2bus->dma_bank();

		case 0x3c:  // SOUNDCTL
			// Bit 7 is the DOC port. Bit 4 is reserved and reads as 1. Bits 3-0
			// are the VCA, returned as stored (not forced on).
			if (!machine().side_effects_disabled())
			{
				sndglu_service();
				if (m_glu_log_enabled)
					sndglu_log_access(12, true, 0);
			}
			return (m_sndglu_ctrl & 0x7f) | 0x10 | ((m_sndglu_pending && sndglu_now() < m_sndglu_busy_until) ? 0x80 : 0x00);

		case 0x3d:  // SOUNDDATA
		{
			if (machine().side_effects_disabled())
				return m_sndglu_dummy_read;
			// The GLU has no ready line, so it cannot hold the CPU. The read returns
			// the byte already in the latch and posts a fetch; while busy, the new
			// request takes the place of the waiting one.
			// Finish a transfer whose deadline has passed before sampling the
			// CPU read latch. The scheduler may not have dispatched its timer
			// yet while this CPU instruction is still executing.
			const bool busy = sndglu_service();
			ret = m_sndglu_dummy_read;
			if (m_glu_log_enabled)
				sndglu_log_access(13, true, 0, ret);
			if (busy)
			{
				m_sndglu_pend_read = true;
				m_sndglu_pend_ram = (m_sndglu_ctrl & 0x40) != 0;
				m_sndglu_pend_auto = (m_sndglu_ctrl & 0x20) != 0;
				m_sndglu_pend_addr = m_sndglu_addr;
			}
			else
				sndglu_arm(true, 0);
			return ret;
		}

		case 0x3e:  // SOUNDADRL
			sndglu_service();
			if (m_glu_log_enabled)
				sndglu_log_access(0xe, true, 0);
			return m_sndglu_addr & 0xff;

		case 0x3f:  // SOUNDADRH
			sndglu_service();
			if (m_glu_log_enabled)
				sndglu_log_access(0xf, true, 0);
			return (m_sndglu_addr >> 8) & 0xff;

		case 0x41:  // INTEN
			return m_inten;

		case 0x44: // MMDELTAX
		case 0x45: // MMDELTAY
			return 0; // read by AppleTalk to detect SCC activity

		case 0x46:  // INTFLAG
			return (m_an3 ? INTFLAG_AN3 : 0x00) | m_intflag;

		case 0x60: // button 3 on IIgs, inverted
			return (m_gameio->sw3_r() ? 0 : 0x80) | uFloatingBus7;

		case 0x61: // button 0 or Open Apple
			// HACK/TODO: the 65816 loses a race to the microcontroller on reset
			if (m_adb_reset_freeze > 0 && !machine().side_effects_disabled())
				m_adb_reset_freeze--;
			return (((m_gameio->has_sw0() && m_gameio->sw0_r()) || (m_adb_p3_last & 0x20)) ? 0x80 : 0) | uFloatingBus7;

		case 0x62: // button 1 or Option
			return (((m_gameio->has_sw1() && m_gameio->sw1_r()) || (m_adb_p3_last & 0x10)) ? 0x80 : 0) | uFloatingBus7;

		case 0x63: // button 2 (no shift key mod)
			return (m_gameio->sw2_r() ? 0x80 : 0) | uFloatingBus7;

		case 0x64:  // joy 1 X axis
			if (!m_gameio->is_device_connected()) return 0x80 | uFloatingBus7;
			return ((machine().time().as_double() < m_joystick_x1_time) ? 0x80 : 0) | uFloatingBus7;

		case 0x65:  // joy 1 Y axis
			if (!m_gameio->is_device_connected()) return 0x80 | uFloatingBus7;
			return ((machine().time().as_double() < m_joystick_y1_time) ? 0x80 : 0) | uFloatingBus7;

		case 0x66: // joy 2 X axis
			if (!m_gameio->is_device_connected()) return 0x80 | uFloatingBus7;
			return ((machine().time().as_double() < m_joystick_x2_time) ? 0x80 : 0) | uFloatingBus7;

		case 0x67: // joy 2 Y axis
			if (!m_gameio->is_device_connected()) return 0x80 | uFloatingBus7;
			return ((machine().time().as_double() < m_joystick_y2_time) ? 0x80 : 0) | uFloatingBus7;

		case 0x68: // STATEREG, synthesizes all the IIe state regs
			return  (m_altzp ? 0x80 : 0x00) |
					(m_video->get_page2() ? 0x40 : 0x00) |
					(m_ramrd ? 0x20 : 0x00) |
					(m_ramwrt ? 0x10 : 0x00) |
					(m_lcram ? 0x00 : 0x08) |
					(m_lcram2 ? 0x04 : 0x00) |
					(m_rombank ? 0x02 : 0x00) |
					(m_intcxrom ? 0x01 : 0x00);

		// The ROM IRQ vectors point here (0x70 returns floating bus)
		case 0x7e:
			do_io(offset); // Zip delay
			[[fallthrough]];
		case 0x71: case 0x72: case 0x73: case 0x74: case 0x75: case 0x76: case 0x77:
		case 0x78: case 0x79: case 0x7a: case 0x7b: case 0x7c: case 0x7d: case 0x7f:
			return m_rom[offset + 0x3c000];

		default:
			/*
			When a ZipGS is present, side-effects to annunciators are skipped
			IF the accelerator is unlocked AND the current instruction executes
			from cache.  Cache is not emulated here, but as an approximation,
			m_accel_fast differentiates all-cached and not-cached.
			*/
			if (((offset & 0xf8) != 0x58) || !m_accel_unlocked || !m_accel_fast)
			{
				do_io(offset);
			}

			if (m_accel_unlocked) switch(offset)
			{
				case 0x58: return 0xff; // undocumented, not floating bus
				case 0x5d: return 0x00; // "bank"
				case 0x5e:
					if (!machine().side_effects_disabled())
						m_zip_trash_tag = true;
					return 0x00; // TODO: physical ZipGS tag byte encoding
				case 0x5f:
					if (!machine().side_effects_disabled())
						m_zip_tag_update = false;
					return 0x00; // TODO: physical ZipGS tag byte encoding

				case 0x59:
					return m_accel_gsxsettings;

				case 0x5a:
					return m_accel_percent | 0x0f;

				case 0x5b: // status flags
				{
					// bits 0-1 are cache size: [8, 16, 32, 64]kB
					const u8 b01 = m_zip_size;
					// bit 3 is set if a temporary delay is active due to slot or softswitch access
					const u8 b3 = m_accel_temp_slowdown ? 0x08 : 0x00;
					// bit 4 is set if the Zip is disabled
					const u8 b4 = m_accel_fast ? 0x00 : 0x10;
					// bit 5 is set if LC caching is disabled
					const u8 b5 = BIT(m_accel_gsxsettings, 7) ? 0x20 : 0x00;
					// bit 7 is a tap on the PH0 clock divided by 1024; edge every 512 cycles
					const int time = machine().time().as_ticks(1021800 / 512.0F);
					const u8 b7 = (time & 1) ? 0x80 : 0x00;
					return b7 | (m_zip_tag_update ? 0x40 : 0x00) | b5 | b4 | b3 | (m_rombank ? 0x04 : 0x00) | b01;
				}

				case 0x5c:
					// C036 motor detection overrides slots 4-7
					return m_accel_slotspk | (m_speed << 4);
			}
			break;
	}

	return uFloatingBus;
}

void apple2gs_state::c000_w(offs_t offset, u8 data)
{
	if(machine().side_effects_disabled()) return;

	switch (offset)
	{
		// FPI registers are fast. $C03C-$C03F stay a synchronized 1 MHz cycle.
		case 0x35: case 0x36: case 0x37:
			break;
		default:
			slow_cycle();
	}

	switch (offset)
	{
		case 0x00:  // 80STOREOFF
			m_video->a80store_w(false);
			auxbank_update();
			break;

		case 0x01:  // 80STOREON
			m_video->a80store_w(true);
			auxbank_update();
			break;

		case 0x02:  // RAMRDOFF
			m_ramrd = false;
			auxbank_update();
			break;

		case 0x03:  // RAMRDON
			m_ramrd = true;
			auxbank_update();
			break;

		case 0x04:  // RAMWRTOFF
			m_ramwrt = false;
			auxbank_update();
			break;

		case 0x05:  // RAMWRTON
			m_ramwrt = true;
			auxbank_update();
			break;

		case 0x06:  // INTCXROMOFF
			m_intcxrom = false;
			update_slotrom_banks();
			break;

		case 0x07:  // INTCXROMON
			m_intcxrom = true;
			update_slotrom_banks();
			break;

		case 0x08:  // ALTZPOFF
			m_altzp = false;
			auxbank_update();
			break;

		case 0x09:  // ALTZPON
			m_altzp = true;
			auxbank_update();
			break;

		case 0x0a:  // SETINTC3ROM
			m_slotc3rom = false;
			update_slotrom_banks();
			break;

		case 0x0b:  // SETSLOTC3ROM
			m_slotc3rom = true;
			update_slotrom_banks();
			break;

		case 0x0c:  // 80COLOFF
			m_video->a80col_w(false);
			break;

		case 0x0d:  // 80COLON
			m_video->a80col_w(true);
			break;

		case 0x0e:  // ALTCHARSETOFF
			m_video->altcharset_w(false);
			break;

		case 0x0f:  // ALTCHARSETON
			m_video->altcharset_w(true);
			break;

		case 0x10: case 0x11: case 0x12: case 0x13:
		case 0x14: case 0x15: case 0x16: case 0x17:
		case 0x18: case 0x19: case 0x1a: case 0x1b:
		case 0x1c: case 0x1d: case 0x1e: case 0x1f: // IIe/Mega II keyboard strobe aliases
			keyglu_816_write(GLU_C010, data);
			break;

		case 0x21:  // MONOCHROME
			m_video->set_GS_monochrome(data);
			break;

		case 0x22:  // TEXTCOL
			m_video->set_GS_textcol(data);
			break;

		case 0x23:  // VGCINT
			// clearing enable bits may clear VGCINT_ANYVGCINT
			// but it does not clear VGCINT status bits or lower IRQs
			if (!((m_vgcint &   (VGCINT_SECOND | VGCINT_SCANLINE))
				& (data & (VGCINT_SECONDENABLE | VGCINT_SCANLINEEN))))
			{
				m_vgcint &= ~(VGCINT_ANYVGCINT);
			}

			m_vgcint &= 0xf0;
			m_vgcint |= (data & 0x07);
			//printf("%02x to VGCINT, now %02x\n", data, m_vgcint);
			break;

		case 0x26:  // DATAREG
			// allow ADBuC sync if the Zip is on
			accel_temp_delay(30, true);
			keyglu_816_write(GLU_COMMAND, data);
			break;

		case 0x27:  // KMSTATUS
			keyglu_816_write(GLU_SYSSTAT, data);
			break;

		case 0x29:  // NEWVIDEO
			m_video->set_newvideo(data & 0xe1);
			break;

		case 0x2b:  // LANGSEL
			m_video->set_GS_langsel(data & 0xf8);
			break;

		case 0x2d:  // SLOTROMSEL
			m_slotromsel = data & 0xf6;
			break;

		case 0x31:  // DISKREG
			if (!BIT(data, DISKREG_35SEL))
			{
				m_motoroff_time = 30;
			}

			if ((m_cur_floppy) && (BIT(data, DISKREG_35SEL)))
			{
				m_cur_floppy->ss_w(BIT(data, DISKREG_HDSEL));
			}
			m_diskreg = data & 0xc0;
			break;

		case 0x32:  // VGCINTCLEAR
			//printf("%02x to VGCINTCLEAR\n", data);
			clear_vgcint(data);
			break;

		case 0x33:  // CLOCKDATA
			m_clkdata = data;
			break;

		case 0x34:  // CLOCKCTL
			if ((data & 0xf) != (m_video->get_GS_border() & 0xf))
			{
				m_screen->update_now();
			}
			m_clock_control = data & 0x6f;
			m_video->set_GS_border(data & 0xf);
			m_rtc->ce_w(BIT(data, 7) ^ 1);
			if (data & 0x80)
			{
				process_clock();
			}
			break;

		case 0x35:  // SHADOW
			m_shadow = data; // bits 5,7 are not forced to zero

			// handle I/O and language card inhibit bits here
			if (m_shadow & SHAD_IOLC)
			{
				m_bank0_atc.select(0);
				m_bank1_atc.select(0);
			}
			else
			{
				m_bank0_atc.select(1);
				m_bank1_atc.select(1);
			}
			break;

		case 0x36:  // SPEED
			m_speed = data; // bits 5-6 are not forced to zero
			update_speed();
			break;

		case 0x37:  // DMAREG
			m_a2bus->set_dma_bank(data);
			break;

		case 0x38:  // SCCBREG
			m_scc->cb_w(0, data);
			break;

		case 0x39:  // SCCAREG
			m_scc->ca_w(0, data);
			break;

		case 0x3a:  // SCCBDATA
			m_scc->db_w(0, data);
			break;

		case 0x3b:  // SCCADATA
			m_scc->da_w(0, data);
			break;

		case 0x3c:  // SOUNDCTL
			if (!m_snd_force_on && ((m_bt_mode == BT_ZIP) || (m_bt_mode == BT_TWGS) || (m_bt_mode == BT_NATIVE)))
			{
				m_glu_hold = true;
				m_glu_hold_reg = 0xc03c;
				m_glu_hold_data = data;
				break;
			}
			sndglu_ctrl_write(data);
			break;

		case 0x3d:  // SOUNDDATA
			// The CPU core writes the device before the bus hook. On a Zip or
			// a TransWarp the byte is not on the GLU until that hook's cycle
			// (its own bus cycle). Judge the collision there.
			if (!m_snd_force_on && (m_bt_mode == BT_ZIP || m_bt_mode == BT_TWGS || m_bt_mode == BT_NATIVE))
			{
				m_glu_hold = true;
				m_glu_hold_reg = 0xc03d;
				m_glu_hold_data = data;
				break;
			}
			sndglu_reg_write(data);
			break;

		case 0x3e:  // SOUNDADRL
		case 0x3f:  // SOUNDADRH
			// On a Zip or a TransWarp the store reaches the GLU at its own bus cycle.
			if (!m_snd_force_on && (m_bt_mode == BT_ZIP || m_bt_mode == BT_TWGS || m_bt_mode == BT_NATIVE))
			{
				m_glu_hold = true;
				m_glu_hold_reg = 0xc000 | offset;
				m_glu_hold_data = data;
				break;
			}
			sndglu_addr_write(0xc000 | offset, data);
			break;

		case 0x41:  // INTEN
			m_inten = data & 0x1f;
			// clearing enable bits may clear INTFLAG_IRQASSERTED
			// but it does not clear INTFLAG status bits
			if (!(data & 0x10))
			{
				lower_irq(IRQS_QTRSEC);
			}
			if (!(data & 0x08))
			{
				lower_irq(IRQS_VBL);
			}
			//printf("%02x to INTEN, now %02x\n", data, m_vgcint);
			break;

		case 0x59: // Zip GS-specific settings
			accel_stop_delay();
			if (m_accel_unlocked)
			{
				m_accel_gsxsettings = (data & 0xf8) | (m_accel_gsxsettings & 0x07);
				update_speed(); // CPS Follow
			}
			do_io(offset);
			break;

		case 0x5a: // Zip accelerator unlock
			accel_stop_delay();
			if ((m_sysconfig->read() & 0x01) && !m_twgs)
			{
				if ((data & 0xf0) == 0x50)
				{
					if (m_accel_stage == 0)
					{
						m_accel_stage = 1;
						m_accel_tap = m_maincpu->space(AS_PROGRAM).install_write_tap(
							0x000000, 0xffffff, "zip_unlock",
							[this](offs_t offset, u8 &data, u8 mem_mask)
						{
							if(!machine().side_effects_disabled())
							{
								bool iobank = false;

								switch(offset >> 16)
								{
									case 0x00: case 0x01:
										if (m_shadow & SHAD_IOLC) break;
										[[fallthrough]];
									case 0xe0: case 0xe1:
										iobank = true;
								}
								offset &= 0xFFFF;

								if ((offset != 0xc05a) || ((data & 0xf0) != 0x50) || !iobank)
								{
									// any other write instruction between the first
									// four 5x writes will invalidate the unlock
									m_accel_stage = 0;
									m_accel_tap.remove();
								}
								else if (++m_accel_stage == 4)
								{
									// at least four writes of 5x in succession
									m_accel_unlocked = true;
									m_accel_tap.remove();
								}
							}
						}, &m_accel_tap);
					}
				}
				else if ((data & 0xf0) == 0xa0)
				{
					// lock
					m_accel_unlocked = false;
					m_accel_stage = 0;
				}
				else if (m_accel_unlocked)
				{
					// disable acceleration
					m_accel_fast = false;
					update_speed();
				}
			}
			do_io(offset);
			break;

		case 0x5b: // enable acceleration
			accel_stop_delay();
			if (m_accel_unlocked)
			{
				m_accel_fast = true;
				update_speed();
			}
			do_io(offset);
			break;

		case 0x5c: // Zip slot/speaker flags
			accel_stop_delay();
			if (m_accel_unlocked)
			{
				m_accel_slotspk = data;
			}
			do_io(offset);
			break;

		case 0x5d: // Zip speed percentage
			accel_stop_delay();
			if (m_accel_unlocked)
			{
				m_accel_percent = data;
			}
			do_io(offset);
			break;

		case 0x58: // Zip cold boot
			m_accel_warm_boot = false;
			[[fallthrough]];
		case 0x5e:
		case 0x5f:
			accel_stop_delay();
			do_io(offset);
			break;

		case 0x68: // STATEREG
			m_altzp = (data & 0x80);
			m_video->scr_w(data & 0x40);
			m_ramrd = (data & 0x20);
			m_ramwrt = (data & 0x10);
			m_lcram = (data & 0x08) ? false : true;
			m_lcram2 = (data & 0x04);
			m_rombank = (data & 0x02);
			m_intcxrom = (data & 0x01);

			// update the aux state
			auxbank_update();

			// Use the same mapping updates as the individual switches. In
			// particular, retain ROMBANK and update the C3 slot ROM view.
			lcrom_update();
			update_slotrom_banks();
			break;

		default:
			do_io(offset);
			break;
		}
}

u8 apple2gs_state::c080_r(offs_t offset)
{
	if (!machine().side_effects_disabled())
	{
		int slot;

		slow_cycle();

		offset &= 0x7f;
		slot = offset / 0x10;

		if (slot == 0)
		{
			lc_update(offset & 0xf, false);
		}
		else
		{
			accel_slot(slot);

			if (slot >= 4)
			{
				if ((offset & 0xf) == 0x9)
				{
					m_motors_active |= (1 << (slot - 4));
				}
				else if ((offset & 0xf) == 8)
				{
					m_motors_active &= ~(1 << (slot - 4));
				}

				update_speed();
			}

			// slot 3 always has I/O go to the external card
			if ((slot != 3) && ((m_slotromsel & (1 << slot)) == 0))
			{
				if (slot == 6)
				{
					return m_iwm->read(offset & 0xf);
				}
			}
			else
			{
				if (m_slotdevice[slot] != nullptr)
				{
					return m_slotdevice[slot]->read_c0nx(offset % 0x10);
				}
			}
		}
	}

	return read_floatingbus();
}

void apple2gs_state::c080_w(offs_t offset, u8 data)
{
	int slot;

	slow_cycle();

	offset &= 0x7f;
	slot = offset / 0x10;

	if (slot == 0)
	{
		lc_update(offset & 0xf, true);
	}
	else
	{
		accel_slot(slot);

		if (slot >= 4)
		{
			if ((offset & 0xf) == 0x9)
			{
				m_motors_active |= (1 << (slot - 4));
			}
			else if ((offset & 0xf) == 8)
			{
				m_motors_active &= ~(1 << (slot - 4));
			}

			update_speed();
		}

		// slot 3 always has I/O go to the external card
		if ((slot != 3) && ((m_slotromsel & (1 << slot)) == 0))
		{
			if (slot == 6)
			{
				m_iwm->write(offset & 0xf, data);
			}
		}
		else
		{
			if (m_slotdevice[slot] != nullptr)
			{
				m_slotdevice[slot]->write_c0nx(offset % 0x10, data);
			}
		}
	}
}

u8 apple2gs_state::read_slot_rom(int slotbias, int offset)
{
	const int slotnum = ((offset>>8) & 0xf) + slotbias;

//  printf("read_slot_rom: sl %d offs %x, cnxx_slot %d\n", slotnum, offset, m_cnxx_slot);
	slow_cycle();

	if (m_slotdevice[slotnum] != nullptr)
	{
		// a bus fight here is resolved as "first-one-wins"
		if ((m_cnxx_slot == CNXX_UNCLAIMED) && (m_slotdevice[slotnum]->take_c800()) && (!machine().side_effects_disabled()))
		{
			m_cnxx_slot = slotnum;
		}

		return m_slotdevice[slotnum]->read_cnxx(offset&0xff);
	}

	return read_floatingbus();
}

void apple2gs_state::write_slot_rom(int slotbias, int offset, u8 data)
{
	const int slotnum = ((offset>>8) & 0xf) + slotbias;

	slow_cycle();

	if (m_slotdevice[slotnum] != nullptr)
	{
		if ((m_cnxx_slot == CNXX_UNCLAIMED) && (m_slotdevice[slotnum]->take_c800()) && !machine().side_effects_disabled())
		{
			m_cnxx_slot = slotnum;
		}

		m_slotdevice[slotnum]->write_cnxx(offset&0xff, data);
	}
}

u8 apple2gs_state::read_int_rom(int slotbias, int offset) { return m_rom[slotbias + offset]; }

u8 apple2gs_state::c100_r(offs_t offset)
{
	const int slot = ((offset>>8) & 0xf) + 1;

	if (m_intcxrom || !BIT(m_slotromsel, slot))
	{
		return read_int_rom(0x3c100, offset);
	}

	return read_slot_rom(1, offset);
}

void apple2gs_state::c100_w(offs_t offset, u8 data)
{
	const int slot = ((offset>>8) & 0xf) + 1;

	if (!m_intcxrom && BIT(m_slotromsel, slot))
	{
		write_slot_rom(1, offset, data);
	}
}

u8 apple2gs_state::c300_int_r(offs_t offset)
{
	if ((!m_slotc3rom) && !machine().side_effects_disabled())
	{
		m_intc8rom = true;
	}

	return read_int_rom(0x3c300, offset);
}

u8 apple2gs_state::c300_r(offs_t offset)  { return read_slot_rom(3, offset); }
void apple2gs_state::c300_w(offs_t offset, u8 data) { write_slot_rom(3, offset, data); }

u8 apple2gs_state::c400_r(offs_t offset)
{
	const int slot = ((offset>>8) & 0xf) + 4;

	if (m_intcxrom || !BIT(m_slotromsel, slot))
	{
		return read_int_rom(0x3c400, offset);
	}

	return read_slot_rom(4, offset);
}

void apple2gs_state::c400_w(offs_t offset, u8 data)
{
	const int slot = ((offset>>8) & 0xf) + 4;

	if (!m_intcxrom && BIT(m_slotromsel, slot))
	{
		write_slot_rom(4, offset, data);
	}
}

u8 apple2gs_state::c800_r(offs_t offset)
{
	const int slot = m_cnxx_slot;
	const int internal = (m_intcxrom) || (m_intc8rom);
	// Sample the owner before $CFFF clears it. The Zip data hook runs after
	// this handler. Motherboard ROM is not this flag.
	m_zip_c8_slot = !internal && (slot > 0) && (m_slotdevice[slot] != nullptr);

	if ((offset == 0x7ff) && !machine().side_effects_disabled())
	{
		m_cnxx_slot = CNXX_UNCLAIMED;
		m_intc8rom = false;
	}

	if (internal)
	{
		return m_rom[offset + 0x3c800];
	}

	if ((slot > 0) && (m_slotdevice[slot] != nullptr))
	{
		slow_cycle();
		return m_slotdevice[slot]->read_c800(offset&0xfff);
	}

	return read_floatingbus();
}

void apple2gs_state::c800_w(offs_t offset, u8 data)
{
	m_zip_c8_slot = (m_cnxx_slot > 0) && (m_slotdevice[m_cnxx_slot] != nullptr);
	if (m_zip_c8_slot)
	{
		slow_cycle();
		m_slotdevice[m_cnxx_slot]->write_c800(offset&0xfff, data);
	}

	if (offset == 0x7ff)
	{
		m_cnxx_slot = CNXX_UNCLAIMED;
		m_intc8rom = false;
		return;
	}
}

// 65816 bank register is left on floating bus
u8 apple2gs_state::floatingbank_r(offs_t offset)
{
	if (m_twgs && ((offset >> 16) == 0xbc))
		return twgs_r(offset & 0xffff);

	// When the memory expansion slot is empty, this is the behavior
	// for every bank not populated by motherboard RAM or ROM.
	return offset >> 16;
}

void apple2gs_state::floatingbank_w(offs_t offset, u8 data)
{
	if (m_twgs && ((offset >> 16) == 0xbc) && !machine().side_effects_disabled())
		twgs_w(offset & 0xffff, data);
}

// mirror expansion RAM banks per Hardware Reference, Figure 3-9
u8 apple2gs_state::ghostram_r(offs_t offset)
{
	offs_t ghost_offset = (offset & m_ghost_mask) + m_motherboard_ram;

	if (ghost_offset < m_ram_size)
		return m_ram_ptr[ghost_offset];
	else // unpopulated non-power-of-two bank
		return m_is_rom3 ? 0xff : ((offset + m_motherboard_ram + m_ghost_mask + 1) >> 16);
}

void apple2gs_state::ghostram_w(offs_t offset, u8 data)
{
	offs_t ghost_offset = (offset & m_ghost_mask) + m_motherboard_ram;

	if (ghost_offset < m_ram_size)
	{
		m_ram_ptr[ghost_offset] = data;
		expansion_shadow_w(ghost_offset, data);
	}
}

void apple2gs_state::expansion_ram_w(offs_t offset, u8 data)
{
	m_ram_ptr[offset + 0x020000] = data;
	expansion_shadow_w(offset + 0x020000, data);
}

void apple2gs_state::expansion_shadow_w(offs_t offset, u8 data)
{
	if (!(m_speed & SPEED_ALLBANKS))
		return;

	const u16 address = offset & 0xffff;
	const bool aux = BIT(offset, 16);
	bool shadow = false;

	if (address >= 0x0400 && address < 0x0800)
		shadow = !(m_shadow & SHAD_TXTPG1);
	else if (address >= 0x0800 && address < 0x0c00)
		shadow = m_is_rom3 && !(m_shadow & SHAD_TXTPG2);
	else if (address >= 0x2000 && address < 0x4000)
		shadow = (!(m_shadow & SHAD_HIRESPG1) && (!aux || !(m_shadow & SHAD_AUXHIRES))) || (aux && !(m_shadow & SHAD_SUPERHIRES));
	else if (address >= 0x4000 && address < 0x6000)
		shadow = (!(m_shadow & SHAD_HIRESPG2) && (!aux || !(m_shadow & SHAD_AUXHIRES))) || (aux && !(m_shadow & SHAD_SUPERHIRES));
	else if (address >= 0x6000 && address < 0xa000)
		shadow = aux && !(m_shadow & SHAD_SUPERHIRES);

	if (shadow)
	{
		if (aux && address >= 0x2000)
			auxram0000_w(address, data);
		else
		{
			slow_cycle();
			m_megaii_ram[offset & 0x1ffff] = data;
		}
	}
}

u8 apple2gs_state::inh_r(offs_t offset)
{
	if (m_inh_slot != -1)
	{
		slow_cycle();
		return m_slotdevice[m_inh_slot]->read_inh_rom(offset + 0xd000);
	}

	assert(0);  // hitting inh_r with invalid m_inh_slot should not be possible
	return read_floatingbus();
}

void apple2gs_state::inh_w(offs_t offset, u8 data)
{
	if (m_inh_slot != -1)
	{
		slow_cycle();
		m_slotdevice[m_inh_slot]->write_inh_rom(offset + 0xd000, data);
	}
}

u8 apple2gs_state::lc_r(offs_t offset)
{
	slow_cycle();
	if (m_altzp)
	{
		if (offset < 0x1000)
		{
			if (m_lcram2)
			{
				return m_megaii_ram[(offset & 0xfff) + 0x1c000];
			}
			else
			{
				return m_megaii_ram[(offset & 0xfff) + 0x1d000];
			}
		}

		return m_megaii_ram[(offset & 0x1fff) + 0x1e000];
	}
	else
	{
		if (offset < 0x1000)
		{
			if (m_lcram2)
			{
				return m_megaii_ram[(offset & 0xfff) + 0xc000];
			}
			else
			{
				return m_megaii_ram[(offset & 0xfff) + 0xd000];
			}
		}

		return m_megaii_ram[(offset & 0x1fff) + 0xe000];
	}
}

void apple2gs_state::lc_w(offs_t offset, u8 data)
{
	slow_cycle();
	if (!m_lcwriteenable)
	{
		return;
	}

	if (m_altzp)
	{
		if (offset < 0x1000)
		{
			if (m_lcram2)
			{
				m_megaii_ram[(offset & 0xfff) + 0x1c000] = data;
			}
			else
			{
				m_megaii_ram[(offset & 0xfff) + 0x1d000] = data;
			}
			return;
		}

		m_megaii_ram[(offset & 0x1fff) + 0x1e000] = data;
	}
	else
	{
		if (offset < 0x1000)
		{
			if (m_lcram2)
			{
				m_megaii_ram[(offset & 0xfff) + 0xc000] = data;
			}
			else
			{
				m_megaii_ram[(offset & 0xfff) + 0xd000] = data;
			}
			return;
		}

		m_megaii_ram[(offset & 0x1fff) + 0xe000] = data;
	}
}

u8 apple2gs_state::lc_aux_r(offs_t offset)
{
	slow_cycle();
	if (offset < 0x1000)
	{
		if (m_lcram2)
		{
			return m_megaii_ram[(offset & 0xfff) + 0x1c000];
		}
		else
		{
			return m_megaii_ram[(offset & 0xfff) + 0x1d000];
		}
	}

	return m_megaii_ram[(offset & 0x1fff) + 0x1e000];
}

void apple2gs_state::lc_aux_w(offs_t offset, u8 data)
{
	slow_cycle();
	if (!m_lcwriteenable)
	{
		return;
	}

	if (offset < 0x1000)
	{
		if (m_lcram2)
		{
			m_megaii_ram[(offset & 0xfff) + 0x1c000] = data;
		}
		else
		{
			m_megaii_ram[(offset & 0xfff) + 0x1d000] = data;
		}
		return;
	}

	m_megaii_ram[(offset & 0x1fff) + 0x1e000] = data;
}

u8 apple2gs_state::lc_00_r(offs_t offset)
{
	if (m_altzp)
	{
		if (offset < 0x1000)
		{
			if (m_lcram2)
			{
				return m_ram_ptr[(offset & 0xfff) + 0x1c000];
			}
			else
			{
				return m_ram_ptr[(offset & 0xfff) + 0x1d000];
			}
		}

		return m_ram_ptr[(offset & 0x1fff) + 0x1e000];
	}
	else
	{
		if (offset < 0x1000)
		{
			if (m_lcram2)
			{
				return m_ram_ptr[(offset & 0xfff) + 0xc000];
			}
			else
			{
				return m_ram_ptr[(offset & 0xfff) + 0xd000];
			}
		}

		return m_ram_ptr[(offset & 0x1fff) + 0xe000];
	}
}

void apple2gs_state::lc_00_w(offs_t offset, u8 data)
{
	if (!m_lcwriteenable)
	{
		return;
	}

	if (m_altzp)
	{
		if (offset < 0x1000)
		{
			if (m_lcram2)
			{
				m_ram_ptr[(offset & 0xfff) + 0x1c000] = data;
			}
			else
			{
				m_ram_ptr[(offset & 0xfff) + 0x1d000] = data;
			}
			return;
		}

		m_ram_ptr[(offset & 0x1fff) + 0x1e000] = data;
	}
	else
	{
		if (offset < 0x1000)
		{
			if (m_lcram2)
			{
				m_ram_ptr[(offset & 0xfff) + 0xc000] = data;
			}
			else
			{
				m_ram_ptr[(offset & 0xfff) + 0xd000] = data;
			}
			return;
		}

		m_ram_ptr[(offset & 0x1fff) + 0xe000] = data;
	}
}

u8 apple2gs_state::lc_01_r(offs_t offset)
{
	if (offset < 0x1000)
	{
		if (m_lcram2)
		{
			return m_ram_ptr[(offset & 0xfff) + 0x1c000];
		}
		else
		{
			return m_ram_ptr[(offset & 0xfff) + 0x1d000];
		}
	}

	return m_ram_ptr[(offset & 0x1fff) + 0x1e000];
}

void apple2gs_state::lc_01_w(offs_t offset, u8 data)
{
	if (!m_lcwriteenable)
	{
		return;
	}

	if (offset < 0x1000)
	{
		if (m_lcram2)
		{
			m_ram_ptr[(offset & 0xfff) + 0x1c000] = data;
		}
		else
		{
			m_ram_ptr[(offset & 0xfff) + 0x1d000] = data;
		}
		return;
	}

	m_ram_ptr[(offset & 0x1fff) + 0x1e000] = data;
}

u8 apple2gs_state::read_floatingbus()
{
	int h_clock = (m_screen->hpos() - BORDER_LEFT) / 16 + (25 + ALIGN_RFB); // adjust for set_raw
	if (h_clock >= 65)
		h_clock -= 65;

	int v_clock = m_screen->vpos();
	if (v_clock < BORDER_TOP)
		v_clock += m_screen->height(); // remap top border to bottom of VBL
	v_clock -= BORDER_TOP;
	v_clock += (h_clock < (25 - ALIGN_RFB)); // adjust for hpos wrap
	if (v_clock >= m_screen->height())
		v_clock -= m_screen->height();

	/*
	Unlike 8-bit Apples, the IIgs does not redundantly scan video memory during
	blanking.  The first five HBL cycles are used for Mega II "RAM refresh":
	during these PH1 cycles (and everywhere after scanline 199) nothing is put
	onto the data bus.  The floating value is instead the last byte fetched
	during PH0, of the instruction which invoked this read.  For example:
	    LDA C050 returns C0 (typical softswitch)
	    LDA C250 returns C2 (ROM of a Your Card slot with no card)
	    LDA 50   returns 50 (with direct page C000)
	*/
	static bool recurse = false; // no recursion, single thread
	if (!recurse && ((h_clock < 5) || (v_clock > 199)))
	{
		// approximate (for non-flow control instructions) by peeking at PC
		offs_t pc = m_maincpu->pc();
		// previous byte, wrapping at bank boundary
		pc = (pc & 0xff0000) | ((pc - 1) & 0xffff);
		// prevent recursion via slot firmware or Mega II C07x
		recurse = true;
		u8 res = m_maincpu->space(AS_PROGRAM).read_byte(pc);
		recurse = false;
		return res;
	}
	else
	{
		const u32 address = m_video->scanner_address_GS(h_clock, v_clock);
		return m_megaii_ram[address];
	}
}

/***************************************************************************
    ADDRESS MAP
***************************************************************************/

template <int Addr>
u8 apple2gs_state::e0ram_r(offs_t offset) { slow_cycle(); return m_megaii_ram[offset + Addr]; }

template <int Addr>
void apple2gs_state::e0ram_w(offs_t offset, u8 data) { slow_cycle(); m_megaii_ram[offset + Addr] = data; }

template <int Addr>
u8 apple2gs_state::e1ram_r(offs_t offset) { slow_cycle(); return m_megaii_ram[offset + Addr + 0x10000]; }

template <int Addr>
void apple2gs_state::e1ram_w(offs_t offset, u8 data) { slow_cycle(); m_megaii_ram[offset + Addr + 0x10000] = data; }

template u8 apple2gs_state::e0ram_r<0x0000>(offs_t offset);
template u8 apple2gs_state::e0ram_r<0x0200>(offs_t offset);
template u8 apple2gs_state::e0ram_r<0x0400>(offs_t offset);
template u8 apple2gs_state::e0ram_r<0x0800>(offs_t offset);
template u8 apple2gs_state::e0ram_r<0x2000>(offs_t offset);
template u8 apple2gs_state::e0ram_r<0x4000>(offs_t offset);

template u8 apple2gs_state::e1ram_r<0x0000>(offs_t offset);
template u8 apple2gs_state::e1ram_r<0x0200>(offs_t offset);
template u8 apple2gs_state::e1ram_r<0x0400>(offs_t offset);
template u8 apple2gs_state::e1ram_r<0x0800>(offs_t offset);
template u8 apple2gs_state::e1ram_r<0x2000>(offs_t offset);
template u8 apple2gs_state::e1ram_r<0x4000>(offs_t offset);

template void apple2gs_state::e0ram_w<0x0000>(offs_t offset, u8 data);
template void apple2gs_state::e0ram_w<0x0200>(offs_t offset, u8 data);
template void apple2gs_state::e0ram_w<0x0400>(offs_t offset, u8 data);
template void apple2gs_state::e0ram_w<0x0800>(offs_t offset, u8 data);
template void apple2gs_state::e0ram_w<0x2000>(offs_t offset, u8 data);
template void apple2gs_state::e0ram_w<0x4000>(offs_t offset, u8 data);

template void apple2gs_state::e1ram_w<0x0000>(offs_t offset, u8 data);
template void apple2gs_state::e1ram_w<0x0200>(offs_t offset, u8 data);
template void apple2gs_state::e1ram_w<0x0400>(offs_t offset, u8 data);
template void apple2gs_state::e1ram_w<0x0800>(offs_t offset, u8 data);
template void apple2gs_state::e1ram_w<0x2000>(offs_t offset, u8 data);
template void apple2gs_state::e1ram_w<0x4000>(offs_t offset, u8 data);

u8 apple2gs_state::auxram0000_r(offs_t offset)
{
	slow_cycle();
	if ((offset >= 0x2000) && (offset < 0xa000) && ((m_video->get_newvideo() & 0xc0) != 0))
	{
		if (offset & 1)
		{
			offset = ((offset - 0x2000) >> 1) + 0x6000;
		}
		else
		{
			offset = ((offset - 0x2000) >> 1) + 0x2000;
		}
	}
	return m_megaii_ram[offset+0x10000];
}

void apple2gs_state::auxram0000_w(offs_t offset, u8 data)
{
	u16 orig_addr = offset;

	slow_cycle();

	if ((offset >= 0x2000) && (offset < 0xa000) && ((m_video->get_newvideo() & 0xc0) != 0))
	{
		if (offset & 1)
		{
			offset = ((offset - 0x2000) >> 1) + 0x6000;
		}
		else
		{
			offset = ((offset - 0x2000) >> 1) + 0x2000;
		}
	}

	m_megaii_ram[offset+0x10000] = data;

	if ((orig_addr >= 0x9e00) && (orig_addr <= 0x9fff))
	{
		int color = (orig_addr - 0x9e00) >> 1;
		m_video->set_SHR_color(color, rgb_t(
			((m_megaii_ram[0x19f00 + color] >> 0) & 0x0f) * 17,
			((m_megaii_ram[0x15f00 + color] >> 4) & 0x0f) * 17,
			((m_megaii_ram[0x15f00 + color] >> 0) & 0x0f) * 17));
	}
}

u8 apple2gs_state::b0ram0000_r(offs_t offset)  { return m_ram_ptr[offset]; }
void apple2gs_state::b0ram0000_w(offs_t offset, u8 data) { m_ram_ptr[offset] = data; }
u8 apple2gs_state::b0ram0200_r(offs_t offset)  { return m_ram_ptr[offset+0x200]; }
void apple2gs_state::b0ram0200_w(offs_t offset, u8 data) { m_ram_ptr[offset+0x200] = data; }
// Bank $00 text page 1, for a CPU-socket cycle (the 65C816 or the TransWarp GS
// in that socket) and for slot DMA whose $C037 bank is $00. A read is always
// fast RAM. A write is copied to Mega II only while text-page-1 shadowing is
// enabled; that copy is what asserts /M2SEL. Shadow inhibited: neither
// direction touches $E0, so the old text-page bytes stay on the video side.
u8 apple2gs_state::b0ram0400_r(offs_t offset)  { return m_ram_ptr[offset+0x400]; }
void apple2gs_state::b0ram0400_w(offs_t offset, u8 data)
{
	m_ram_ptr[offset+0x400] = data;
	if (!(m_shadow & SHAD_TXTPG1))
	{
		slow_cycle();
		m_megaii_ram[offset+0x0400] = data;
	}
}
u8 apple2gs_state::b0ram0800_r(offs_t offset)  { return m_ram_ptr[offset+0x800]; }
void apple2gs_state::b0ram0800_w(offs_t offset, u8 data)
{
	m_ram_ptr[offset+0x800] = data;
	if (offset < 0x400)
	{
		if (!(m_shadow & SHAD_TXTPG2) && m_is_rom3)
		{
			slow_cycle();
			m_megaii_ram[offset+0x800] = data;
		}
	}
}
u8 apple2gs_state::b0ram2000_r(offs_t offset)  { return m_ram_ptr[offset+0x2000]; }
void apple2gs_state::b0ram2000_w(offs_t offset, u8 data)
{
	m_ram_ptr[offset+0x2000] = data;
	if (!(m_shadow & SHAD_HIRESPG1))
	{
		slow_cycle();
		m_megaii_ram[offset+0x2000] = data;
	}
}
u8 apple2gs_state::b0ram4000_r(offs_t offset)  { return m_ram_ptr[offset+0x4000]; }
void apple2gs_state::b0ram4000_w(offs_t offset, u8 data)
{
	m_ram_ptr[offset+0x4000] = data;
	if (offset < 0x2000)
	{
		if (!(m_shadow & SHAD_HIRESPG2))
		{
			slow_cycle();
			m_megaii_ram[offset+0x4000] = data;
		}
	}
}

u8 apple2gs_state::b1ram0000_r(offs_t offset)  { return m_ram_ptr[offset+0x10000]; }
void apple2gs_state::b1ram0000_w(offs_t offset, u8 data) { m_ram_ptr[offset+0x10000] = data; }
u8 apple2gs_state::b1ram0200_r(offs_t offset)  { return m_ram_ptr[offset+0x10200]; }
void apple2gs_state::b1ram0200_w(offs_t offset, u8 data) { m_ram_ptr[offset+0x10200] = data; }
u8 apple2gs_state::b1ram0400_r(offs_t offset)  { return m_ram_ptr[offset+0x10400]; }
void apple2gs_state::b1ram0400_w(offs_t offset, u8 data)
{
	m_ram_ptr[offset+0x10400] = data;
	if (!(m_shadow & SHAD_TXTPG1))
	{
		slow_cycle();
		m_megaii_ram[offset+0x10400] = data;
	}
}
u8 apple2gs_state::b1ram0800_r(offs_t offset) { return m_ram_ptr[offset+0x10800]; }
void apple2gs_state::b1ram0800_w(offs_t offset, u8 data)
{
	m_ram_ptr[offset+0x10800] = data;
	if (offset < 0x400)
	{
		if (!(m_shadow & SHAD_TXTPG2) && m_is_rom3)
		{
			slow_cycle();
			m_megaii_ram[offset+0x10800] = data;
		}
	}
}
u8 apple2gs_state::b1ram2000_r(offs_t offset)  { return m_ram_ptr[offset+0x12000]; }
void apple2gs_state::b1ram2000_w(offs_t offset, u8 data)
{
	m_ram_ptr[offset+0x12000] = data;
	if ((!(m_shadow & SHAD_HIRESPG1) && !(m_shadow & SHAD_AUXHIRES)) || !(m_shadow & SHAD_SUPERHIRES))
	{
		auxram0000_w(offset+0x2000, data);
	}
}
u8 apple2gs_state::b1ram4000_r(offs_t offset)  { return m_ram_ptr[offset+0x14000]; }
void apple2gs_state::b1ram4000_w(offs_t offset, u8 data)
{
	m_ram_ptr[offset+0x14000] = data;
	if (offset < 0x2000)
	{
		if ((!(m_shadow & SHAD_HIRESPG2) && !(m_shadow & SHAD_AUXHIRES)) || !(m_shadow & SHAD_SUPERHIRES))
		{
			auxram0000_w(offset+0x4000, data);
		}
	}
	else if ((offset >= 0x2000) && (offset <= 0x5fff))
	{
		if (!(m_shadow & SHAD_SUPERHIRES))
		{
			auxram0000_w(offset+0x4000, data);
		}
	}
}

u8 apple2gs_state::bank0_c000_r(offs_t offset)
{
	if (offset & 0x2000)
	{
		offset ^= 0x1000;
	}

	if (m_ramrd)
	{
		return m_ram_ptr[offset + 0x1c000];
	}

	return m_ram_ptr[offset + 0xc000];
}

void apple2gs_state::bank0_c000_w(offs_t offset, u8 data)
{
	if (offset & 0x2000)
	{
		offset ^= 0x1000;
	}

	if (m_ramwrt)
	{
		m_ram_ptr[offset + 0x1c000] = data;
		return;
	}

	m_ram_ptr[offset + 0xc000] = data;
}

u8 apple2gs_state::bank1_0000_r(offs_t offset) { return m_ram_ptr[offset + 0x10000]; }
u8 apple2gs_state::bank1_c000_r(offs_t offset) { if (offset & 0x2000) offset ^= 0x1000; return m_ram_ptr[offset + 0x1c000]; }
void apple2gs_state::bank1_c000_w(offs_t offset, u8 data) { if (offset & 0x2000) offset ^= 0x1000; m_ram_ptr[offset + 0x1c000] = data; }
void apple2gs_state::bank1_0000_sh_w(offs_t offset, u8 data)
{
	m_ram_ptr[offset + 0x10000] = data;

	switch (offset>>10)
	{
		case 0x01:  // text page 1
			if (!(m_shadow & SHAD_TXTPG1))
			{
				slow_cycle();
				m_megaii_ram[offset + 0x10000] = data;
			}
			break;

		case 0x02:  // text page 2 (only shadowable on ROM 03)
			if ((!(m_shadow & SHAD_TXTPG2)) && (m_is_rom3))
			{
				slow_cycle();
				m_megaii_ram[offset + 0x10000] = data;
			}
			break;

			// hi-res page 1
		case 0x08: case 0x09: case 0x0a: case 0x0b: case 0x0c: case 0x0d: case 0x0e: case 0x0f:
			if ((!(m_shadow & SHAD_HIRESPG1) && !(m_shadow & SHAD_AUXHIRES)) || !(m_shadow & SHAD_SUPERHIRES))
			{
				auxram0000_w(offset, data);
			}
			break;

			// hi-res page 2
		case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17:
			if ((!(m_shadow & SHAD_HIRESPG2) && !(m_shadow & SHAD_AUXHIRES)) || !(m_shadow & SHAD_SUPERHIRES))
			{
				auxram0000_w(offset, data);
			}
			break;

		default:
			if ((offset >= 0x6000) && (offset <= 0x9fff))
			{
				if (!(m_shadow & SHAD_SUPERHIRES))
				{
					auxram0000_w(offset, data);
				}
			}
			break;
	}
}

void apple2gs_state::apple2gs_map(address_map &map)
{
	// default behavior for unpopulated banks (affected by memory expansion slot)
	map(0x000000, 0xffffff).rw(FUNC(apple2gs_state::floatingbank_r), FUNC(apple2gs_state::floatingbank_w));
	map.unmap_value_high(); // with expansion slot, unpopulated banks return ff on ROM3

	// "fast side" - runs 2.8 MHz minus RAM refresh, banks 00 and 01 usually have writes shadowed to E0/E1 where I/O lives
	// Banks 00 and 01 also have their own independent language cards which are NOT shadowed.
	map(0x000000, 0x0001ff).view(m_b0_0000bank);
	m_b0_0000bank[0](0x0000, 0x01ff).rw(FUNC(apple2gs_state::b0ram0000_r), FUNC(apple2gs_state::b0ram0000_w));
	m_b0_0000bank[1](0x0000, 0x01ff).rw(FUNC(apple2gs_state::b1ram0000_r), FUNC(apple2gs_state::b1ram0000_w));

	map(0x000200, 0x0003ff).view(m_b0_0200bank);
	m_b0_0200bank[0](0x0200, 0x03ff).rw(FUNC(apple2gs_state::b0ram0200_r), FUNC(apple2gs_state::b0ram0200_w)); // wr 0 rd 0
	m_b0_0200bank[1](0x0200, 0x03ff).rw(FUNC(apple2gs_state::b1ram0200_r), FUNC(apple2gs_state::b0ram0200_w)); // wr 0 rd 1
	m_b0_0200bank[2](0x0200, 0x03ff).rw(FUNC(apple2gs_state::b0ram0200_r), FUNC(apple2gs_state::b1ram0200_w)); // wr 1 rd 0
	m_b0_0200bank[3](0x0200, 0x03ff).rw(FUNC(apple2gs_state::b1ram0200_r), FUNC(apple2gs_state::b1ram0200_w)); // wr 1 rd 1

	map(0x000400, 0x0007ff).view(m_b0_0400bank);
	m_b0_0400bank[0](0x0400, 0x07ff).rw(FUNC(apple2gs_state::b0ram0400_r), FUNC(apple2gs_state::b0ram0400_w)); // wr 0 rd 0
	m_b0_0400bank[1](0x0400, 0x07ff).rw(FUNC(apple2gs_state::b1ram0400_r), FUNC(apple2gs_state::b0ram0400_w)); // wr 0 rd 1
	m_b0_0400bank[2](0x0400, 0x07ff).rw(FUNC(apple2gs_state::b0ram0400_r), FUNC(apple2gs_state::b1ram0400_w)); // wr 1 rd 0
	m_b0_0400bank[3](0x0400, 0x07ff).rw(FUNC(apple2gs_state::b1ram0400_r), FUNC(apple2gs_state::b1ram0400_w)); // wr 1 rd 1

	map(0x000800, 0x001fff).view(m_b0_0800bank);
	m_b0_0800bank[0](0x0800, 0x1fff).rw(FUNC(apple2gs_state::b0ram0800_r), FUNC(apple2gs_state::b0ram0800_w));
	m_b0_0800bank[1](0x0800, 0x1fff).rw(FUNC(apple2gs_state::b1ram0800_r), FUNC(apple2gs_state::b0ram0800_w));
	m_b0_0800bank[2](0x0800, 0x1fff).rw(FUNC(apple2gs_state::b0ram0800_r), FUNC(apple2gs_state::b1ram0800_w));
	m_b0_0800bank[3](0x0800, 0x1fff).rw(FUNC(apple2gs_state::b1ram0800_r), FUNC(apple2gs_state::b1ram0800_w));

	map(0x002000, 0x003fff).view(m_b0_2000bank);
	m_b0_2000bank[0](0x2000, 0x3fff).rw(FUNC(apple2gs_state::b0ram2000_r), FUNC(apple2gs_state::b0ram2000_w));
	m_b0_2000bank[1](0x2000, 0x3fff).rw(FUNC(apple2gs_state::b1ram2000_r), FUNC(apple2gs_state::b0ram2000_w));
	m_b0_2000bank[2](0x2000, 0x3fff).rw(FUNC(apple2gs_state::b0ram2000_r), FUNC(apple2gs_state::b1ram2000_w));
	m_b0_2000bank[3](0x2000, 0x3fff).rw(FUNC(apple2gs_state::b1ram2000_r), FUNC(apple2gs_state::b1ram2000_w));

	map(0x004000, 0x00bfff).view(m_b0_4000bank);
	m_b0_4000bank[0](0x4000, 0xbfff).rw(FUNC(apple2gs_state::b0ram4000_r), FUNC(apple2gs_state::b0ram4000_w));
	m_b0_4000bank[1](0x4000, 0xbfff).rw(FUNC(apple2gs_state::b1ram4000_r), FUNC(apple2gs_state::b0ram4000_w));
	m_b0_4000bank[2](0x4000, 0xbfff).rw(FUNC(apple2gs_state::b0ram4000_r), FUNC(apple2gs_state::b1ram4000_w));
	m_b0_4000bank[3](0x4000, 0xbfff).rw(FUNC(apple2gs_state::b1ram4000_r), FUNC(apple2gs_state::b1ram4000_w));

	map(0x00c000, 0x00ffff).view(m_bank0_atc);
	m_bank0_atc[0](0xc000, 0xffff).rw(FUNC(apple2gs_state::bank0_c000_r), FUNC(apple2gs_state::bank0_c000_w));
	m_bank0_atc[1](0xc000, 0xc07f).rw(FUNC(apple2gs_state::c000_r), FUNC(apple2gs_state::c000_w));
	m_bank0_atc[1](0xc080, 0xc0ff).rw(FUNC(apple2gs_state::c080_r), FUNC(apple2gs_state::c080_w));
	m_bank0_atc[1](0xc100, 0xc2ff).rw(FUNC(apple2gs_state::c100_r), FUNC(apple2gs_state::c100_w));
	m_bank0_atc[1](0xc300, 0xc3ff).m(m_c300bank, FUNC(address_map_bank_device::amap8));
	m_bank0_atc[1](0xc400, 0xc7ff).rw(FUNC(apple2gs_state::c400_r), FUNC(apple2gs_state::c400_w));
	m_bank0_atc[1](0xc800, 0xcfff).rw(FUNC(apple2gs_state::c800_r), FUNC(apple2gs_state::c800_w));

	m_bank0_atc[1](0xd000, 0xffff).view(m_upper00);
	m_upper00[0](0xd000, 0xffff).view(m_lc00);
	m_upper00[1](0xd000, 0xffff).rw(FUNC(apple2gs_state::inh_r), FUNC(apple2gs_state::inh_w));

	m_lc00[0](0xd000, 0xffff).rom().region("maincpu", 0x3d000).w(FUNC(apple2gs_state::lc_00_w));
	m_lc00[1](0xd000, 0xffff).rw(FUNC(apple2gs_state::lc_00_r), FUNC(apple2gs_state::lc_00_w));
	m_lc00[2](0xd000, 0xffff).rom().region("maincpu", 0x39000).w(FUNC(apple2gs_state::lc_00_w));
	m_lc00[3](0xd000, 0xffff).rw(FUNC(apple2gs_state::lc_00_r), FUNC(apple2gs_state::lc_00_w));

	map(0x010000, 0x01bfff).rw(FUNC(apple2gs_state::bank1_0000_r), FUNC(apple2gs_state::bank1_0000_sh_w));

	map(0x01c000, 0x01ffff).view(m_bank1_atc);
	m_bank1_atc[0](0x1c000, 0x1ffff).rw(FUNC(apple2gs_state::bank1_c000_r), FUNC(apple2gs_state::bank1_c000_w));
	m_bank1_atc[1](0x1c000, 0x1c07f).rw(FUNC(apple2gs_state::c000_r), FUNC(apple2gs_state::c000_w));
	m_bank1_atc[1](0x1c080, 0x1c0ff).rw(FUNC(apple2gs_state::c080_r), FUNC(apple2gs_state::c080_w));
	m_bank1_atc[1](0x1c100, 0x1c2ff).rw(FUNC(apple2gs_state::c100_r), FUNC(apple2gs_state::c100_w));
	m_bank1_atc[1](0x1c300, 0x1c3ff).m(m_c300bank, FUNC(address_map_bank_device::amap8));
	m_bank1_atc[1](0x1c400, 0x1c7ff).rw(FUNC(apple2gs_state::c400_r), FUNC(apple2gs_state::c400_w));
	m_bank1_atc[1](0x1c800, 0x1cfff).rw(FUNC(apple2gs_state::c800_r), FUNC(apple2gs_state::c800_w));

	m_bank1_atc[1](0x1d000, 0x1ffff).view(m_upper01);
	m_upper01[0](0x1d000, 0x1ffff).view(m_lc01);
	m_upper01[1](0x1d000, 0x1ffff).rw(FUNC(apple2gs_state::inh_r), FUNC(apple2gs_state::inh_w));

	m_lc01[0](0x1d000, 0x1ffff).rom().region("maincpu", 0x3d000).w(FUNC(apple2gs_state::lc_01_w));
	m_lc01[1](0x1d000, 0x1ffff).rw(FUNC(apple2gs_state::lc_01_r), FUNC(apple2gs_state::lc_01_w));

	// "Mega II side" - this is basically a 128K IIe on a chip that runs merrily at 1 MHz
	// Unfortunately all I/O happens here, including new IIgs-specific stuff
	map(0xe00000, 0xe001ff).view(m_e0_0000bank);
	m_e0_0000bank[0](0xe00000, 0xe001ff).rw(FUNC(apple2gs_state::e0ram_r<0x0000>), FUNC(apple2gs_state::e0ram_w<0x0000>));
	m_e0_0000bank[1](0xe00000, 0xe001ff).rw(FUNC(apple2gs_state::e1ram_r<0x0000>), FUNC(apple2gs_state::e1ram_w<0x0000>));

	map(0xe00200, 0xe003ff).view(m_e0_0200bank);
	m_e0_0200bank[0](0xe00200, 0xe003ff).rw(FUNC(apple2gs_state::e0ram_r<0x0200>), FUNC(apple2gs_state::e0ram_w<0x0200>)); // wr 0 rd 0
	m_e0_0200bank[1](0xe00200, 0xe003ff).rw(FUNC(apple2gs_state::e1ram_r<0x0200>), FUNC(apple2gs_state::e0ram_w<0x0200>)); // wr 0 rd 1
	m_e0_0200bank[2](0xe00200, 0xe003ff).rw(FUNC(apple2gs_state::e0ram_r<0x0200>), FUNC(apple2gs_state::e1ram_w<0x0200>)); // wr 1 rd 0
	m_e0_0200bank[3](0xe00200, 0xe003ff).rw(FUNC(apple2gs_state::e1ram_r<0x0200>), FUNC(apple2gs_state::e1ram_w<0x0200>)); // wr 1 rd 1

	map(0xe00400, 0xe007ff).view(m_e0_0400bank);
	m_e0_0400bank[0](0xe00400, 0xe007ff).rw(FUNC(apple2gs_state::e0ram_r<0x0400>), FUNC(apple2gs_state::e0ram_w<0x0400>)); // wr 0 rd 0
	m_e0_0400bank[1](0xe00400, 0xe007ff).rw(FUNC(apple2gs_state::e1ram_r<0x0400>), FUNC(apple2gs_state::e0ram_w<0x0400>)); // wr 0 rd 1
	m_e0_0400bank[2](0xe00400, 0xe007ff).rw(FUNC(apple2gs_state::e0ram_r<0x0400>), FUNC(apple2gs_state::e1ram_w<0x0400>)); // wr 1 rd 0
	m_e0_0400bank[3](0xe00400, 0xe007ff).rw(FUNC(apple2gs_state::e1ram_r<0x0400>), FUNC(apple2gs_state::e1ram_w<0x0400>)); // wr 1 rd 1

	map(0xe00800, 0xe01fff).view(m_e0_0800bank);
	m_e0_0800bank[0](0xe00800, 0xe01fff).rw(FUNC(apple2gs_state::e0ram_r<0x0800>), FUNC(apple2gs_state::e0ram_w<0x0800>));
	m_e0_0800bank[1](0xe00800, 0xe01fff).rw(FUNC(apple2gs_state::e1ram_r<0x0800>), FUNC(apple2gs_state::e0ram_w<0x0800>));
	m_e0_0800bank[2](0xe00800, 0xe01fff).rw(FUNC(apple2gs_state::e0ram_r<0x0800>), FUNC(apple2gs_state::e1ram_w<0x0800>));
	m_e0_0800bank[3](0xe00800, 0xe01fff).rw(FUNC(apple2gs_state::e1ram_r<0x0800>), FUNC(apple2gs_state::e1ram_w<0x0800>));

	map(0xe02000, 0xe03fff).view(m_e0_2000bank);
	m_e0_2000bank[0](0xe02000, 0xe03fff).rw(FUNC(apple2gs_state::e0ram_r<0x2000>), FUNC(apple2gs_state::e0ram_w<0x2000>));
	m_e0_2000bank[1](0xe02000, 0xe03fff).rw(FUNC(apple2gs_state::e1ram_r<0x2000>), FUNC(apple2gs_state::e0ram_w<0x2000>));
	m_e0_2000bank[2](0xe02000, 0xe03fff).rw(FUNC(apple2gs_state::e0ram_r<0x2000>), FUNC(apple2gs_state::e1ram_w<0x2000>));
	m_e0_2000bank[3](0xe02000, 0xe03fff).rw(FUNC(apple2gs_state::e1ram_r<0x2000>), FUNC(apple2gs_state::e1ram_w<0x2000>));

	map(0xe04000, 0xe0bfff).view(m_e0_4000bank);
	m_e0_4000bank[0](0xe04000, 0xe0bfff).rw(FUNC(apple2gs_state::e0ram_r<0x4000>), FUNC(apple2gs_state::e0ram_w<0x4000>));
	m_e0_4000bank[1](0xe04000, 0xe0bfff).rw(FUNC(apple2gs_state::e1ram_r<0x4000>), FUNC(apple2gs_state::e0ram_w<0x4000>));
	m_e0_4000bank[2](0xe04000, 0xe0bfff).rw(FUNC(apple2gs_state::e0ram_r<0x4000>), FUNC(apple2gs_state::e1ram_w<0x4000>));
	m_e0_4000bank[3](0xe04000, 0xe0bfff).rw(FUNC(apple2gs_state::e1ram_r<0x4000>), FUNC(apple2gs_state::e1ram_w<0x4000>));

	map(0xe0c000, 0xe0c07f).rw(FUNC(apple2gs_state::c000_r), FUNC(apple2gs_state::c000_w));
	map(0xe0c080, 0xe0c0ff).rw(FUNC(apple2gs_state::c080_r), FUNC(apple2gs_state::c080_w));
	map(0xe0c100, 0xe0c2ff).rw(FUNC(apple2gs_state::c100_r), FUNC(apple2gs_state::c100_w));
	map(0xe0c300, 0xe0c3ff).m(m_c300bank, FUNC(address_map_bank_device::amap8));
	map(0xe0c400, 0xe0c7ff).rw(FUNC(apple2gs_state::c400_r), FUNC(apple2gs_state::c400_w));
	map(0xe0c800, 0xe0cfff).rw(FUNC(apple2gs_state::c800_r), FUNC(apple2gs_state::c800_w));

	map(0xe0d000, 0xe0ffff).view(m_upperbank);
	m_upperbank[0](0xe0d000, 0xe0ffff).view(m_lcbank);
	m_upperbank[1](0xe0d000, 0xe0ffff).rw(FUNC(apple2gs_state::inh_r), FUNC(apple2gs_state::inh_w));

	m_lcbank[0](0xe0d000, 0xe0ffff).rom().region("maincpu", 0x3d000).w(FUNC(apple2gs_state::lc_w));
	m_lcbank[1](0xe0d000, 0xe0ffff).rw(FUNC(apple2gs_state::lc_r), FUNC(apple2gs_state::lc_w));

	map(0xe10000, 0xe1bfff).rw(FUNC(apple2gs_state::auxram0000_r), FUNC(apple2gs_state::auxram0000_w));
	map(0xe1c000, 0xe1c07f).rw(FUNC(apple2gs_state::c000_r), FUNC(apple2gs_state::c000_w));
	map(0xe1c080, 0xe1c0ff).rw(FUNC(apple2gs_state::c080_r), FUNC(apple2gs_state::c080_w));
	map(0xe1c100, 0xe1c2ff).rw(FUNC(apple2gs_state::c100_r), FUNC(apple2gs_state::c100_w));
	map(0xe1c300, 0xe1c3ff).m(m_c300bank, FUNC(address_map_bank_device::amap8));
	map(0xe1c400, 0xe1c7ff).rw(FUNC(apple2gs_state::c400_r), FUNC(apple2gs_state::c400_w));
	map(0xe1c800, 0xe1cfff).rw(FUNC(apple2gs_state::c800_r), FUNC(apple2gs_state::c800_w));

	map(0xe1d000, 0xe1ffff).view(m_upperaux);
	m_upperaux[0](0xe1d000, 0xe1ffff).view(m_lcaux);
	m_upperaux[1](0xe1d000, 0xe1ffff).rw(FUNC(apple2gs_state::inh_r), FUNC(apple2gs_state::inh_w));

	m_lcaux[0](0xe1d000, 0xe1ffff).rom().region("maincpu", 0x3d000).w(FUNC(apple2gs_state::lc_aux_w));
	m_lcaux[1](0xe1d000, 0xe1ffff).rw(FUNC(apple2gs_state::lc_aux_r), FUNC(apple2gs_state::lc_aux_w));

	map(0xfe0000, 0xffffff).rom().region("maincpu", 0x20000);
}

void apple2gs_state::vectors_map(address_map &map)
{
	map(0x00, 0x1f).r(FUNC(apple2gs_state::apple2gs_read_vector));
}

void apple2gs_state::c300bank_map(address_map &map)
{
	map(0x0000, 0x00ff).rw(FUNC(apple2gs_state::c300_r), FUNC(apple2gs_state::c300_w));
	map(0x0100, 0x01ff).r(FUNC(apple2gs_state::c300_int_r)).nopw();
}

void apple2gs_state::a2gs_es5503_map(address_map &map)
{
	map(0x00000, 0x0ffff).mirror(0x10000).readonly().share("docram"); // IIgs only has 64K, top bank mirrors lower bank
}

/***************************************************************************
    ADB microcontroller + KEYGLU emulation

    Huge thanks to Neil Parker's writeup on the ADB microcontroller!
    http://llx.com/Neil/a2/adb.html
***************************************************************************/

void apple2gs_state::j13_config_postload()
{
	if (!m_j13_config_port)
		return;

	// Keep the configuration UI and the next cfg write consistent with the state.
	for (ioport_value mask : { 0x01, 0x02 })
	{
		ioport_field *const field = m_j13_config_port->field(mask);
		ioport_field::user_settings settings;
		field->get_user_settings(settings);
		settings.value = m_j13_config & mask;
		field->set_user_settings(settings);
	}
}

INPUT_CHANGED_MEMBER(apple2gs_state::j13_config_changed)
{
	m_j13_config = m_j13_config_port->read();
}

u8 apple2gs_state::adbmicro_p0_in()
{
	return m_glu_bus;
}

u8 apple2gs_state::adbmicro_p1_in()
{
	if (!m_is_rom3)
	{
		// ROM01 sheet 5: GLU SC0-SC9 -> Y0-Y9, J13 X0-X7 -> P10-P17.
		if (BIT(m_j13_config, 0) && m_glu_kbd_y < 10)
		{
			// Apple 699-0076-C sheet 4 is a passive switch matrix.  The
			// 10 March 1986 FDB microcontroller ERS, p. 2, explicitly
			// retains its ghost/phantom keys.  Follow all closed-switch
			// paths from the selected low scan line, including paths back
			// through other rows; there are no per-switch isolation diodes.
			u8 switches[10];
			for (unsigned row = 0; row < 10; ++row)
				switches[row] = ~m_j13_keys[row]->read();
			u8 columns = switches[m_glu_kbd_y];
			u8 previous;
			do
			{
				previous = columns;
				for (u8 row : switches)
					if (row & columns)
						columns |= row;
			} while (columns != previous);
			return ~columns;
		}
		return 0xff;
	}
	else
	{
		return 0x06;
	}
}

u8 apple2gs_state::adbmicro_p2_in()
{
	u8 rv = 0;

	if (!m_is_rom3)
	{
		// J13 /KRESET -> P26. The IIe keyboard's normal solder link
		// returns Reset through Control (699-0076-C sheet 4).
		rv |= (BIT(m_j13_config, 0) && !(m_j13_special->read() & 0x42)) ? 0 : 0x40;
	}
	rv |= (m_adb_line) ? 0x00 : 0x80;

	return rv;
}

u8 apple2gs_state::adbmicro_p3_in()
{
	if (m_is_rom3)
	{
		// Check the jumper to remove the Control Panel from the Control-Open Apple-Esc CDA menu
		if (m_sysconfig->read() & 0x08)
		{
			return 0x40;
		}
		return 0x00;
	}
	else
	{
		// Shift/Control/Caps are active low; KSW0/KSW1 active high.
		if (BIT(m_j13_config, 0))
		{
			u8 const keys = m_j13_special->read();
			return (keys & 0x07) | (BIT(keys, 4) << 7) | (BIT(keys, 5) << 6);
		}
		return 0x07;
	}
}

void apple2gs_state::adbmicro_p0_out(u8 data)
{
	m_glu_bus = data;
}

void apple2gs_state::adbmicro_p1_out(u8 data)
{
}

void apple2gs_state::adbmicro_p2_out(u8 data)
{
	if (!BIT(data, 5) && BIT(m_adb_p2_last, 5))
	{
		m_adb_reset_freeze = 2;
		m_a2bus->reset_bus();
		m_maincpu->reset();
		if (m_twgs)
			twgs_reset_switches();
		if (m_accel_present)
			accel_reset();
		m_video->set_newvideo(0x41);

		m_lcram = false;
		m_lcram2 = true;
		m_lcprewrite = false;
		m_lcwriteenable = true;
		m_intcxrom = false;
		m_slotc3rom = false;
		m_intc8rom = false;
		m_video->a80store_w(false);
		m_altzp = false;
		m_ramrd = false;
		m_ramwrt = false;
		m_altzp = false;
		m_video->scr_w(0);
		m_video->res_w(0);
		m_irqmask = 0;

		lcrom_update();
		auxbank_update();
		update_slotrom_banks();
	}

	if (!(data & 0x10))
	{
		if (m_adbmicro->are_port_bits_output(0, 0xff))
		{
			keyglu_mcu_write(data & 7, m_glu_bus);
		}
		else    // read GLU
		{
			m_glu_bus = keyglu_mcu_read(data & 7);
		}
	}
	else
	{
		m_glu_kbd_y = data & 0xf;
	}

	m_adb_p2_last = data;
}

void apple2gs_state::adbmicro_p3_out(u8 data)
{
	if (((data & 0x08) == 0x08) != m_adb_line)
	{
		m_adb_line = (data & 0x8) ? true : false;
		m_macadb->adb_linechange_w(!m_adb_line);
	}

	if (m_adb_reset_freeze == 0)
	{
		m_adb_p3_last = data;
	}
}

void apple2gs_state::set_adb_line(int linestate)
{
	m_adb_line = (linestate == CLEAR_LINE);
}

u8 apple2gs_state::keyglu_mcu_read(u8 offset)
{
	u8 rv = m_glu_regs[offset];

//  printf("MCU reads reg %x\n", offset);

	// the command full flag is cleared by the MCU reading
	// first the KGS register and then the command register
	if ((offset == GLU_COMMAND) && (m_glu_mcu_read_kgs))
	{
		m_glu_regs[GLU_KG_STATUS] &= ~KGS_COMMAND_FULL;
		m_glu_mcu_read_kgs = false;
	}

	// prime for the next command register read to clear the command full flag
	if (offset == GLU_KG_STATUS)
	{
		m_glu_mcu_read_kgs = true;
	}

	return rv;
}

void  apple2gs_state::keyglu_mcu_write(u8 offset, u8 data)
{
	m_glu_regs[offset] = data;

	switch (offset)
	{
		case GLU_KEY_DATA:
			// if this is the first key pressed within a certain time frame, don't raise the strobe
			// so that the emulated system doesn't see the keypress used to start the emulation.
			// Retain the legacy input-start workaround with J13 off only.
			// Hardware has no CPU-cycle qualification: Hardware Reference,
			// second edition, table 6-4, and FDB MCU ERS p. 3.
			if (!j13_enabled() && (m_sysconfig->read() & 0x01))
			{ // bump the cycle count way up for a 16 Mhz ZipGS
				if (m_maincpu->total_cycles() < 700000)
				{
					return;
				}
			}
			else if (!j13_enabled())
			{
				if (m_maincpu->total_cycles() < 25000)
				{
					return;
				}
			}

			m_glu_regs[GLU_KG_STATUS] |= KGS_KEYSTROBE;
			m_glu_regs[GLU_SYSSTAT] |= GLU_STATUS_KEYDATIRQ;
			keyglu_regen_irqs();
			break;

		case GLU_MOUSEX:
			m_glu_regs[GLU_KG_STATUS] |= KGS_MOUSEX_FULL;
			m_glu_regs[GLU_SYSSTAT] |= GLU_STATUS_MOUSEIRQ;
			keyglu_regen_irqs();
			m_glu_mouse_read_stat = false;  // signal next read will be mouse X
			break;

		case GLU_MOUSEY:
			break;

		case GLU_ANY_KEY_DOWN:  // bit 7 is the actual flag here
			if (data & 0x80)
			{
				m_glu_regs[GLU_KG_STATUS] |= KGS_ANY_KEY_DOWN;
			}
			break;

		case GLU_DATA:
			m_glu_regs[GLU_DATA] = data;
			m_glu_regs[GLU_KG_STATUS] |= KGS_DATA_FULL;
			m_glu_regs[GLU_SYSSTAT] |= GLU_STATUS_DATAIRQ;
			keyglu_regen_irqs();
			m_glu_816_read_dstat = false;
			break;
	}
}

/*
   Keyglu registers map as follows on the 816:

   C000           = key data + strobe (not cleared by this read)
   C010           = clears keystrobe
   C024 MOUSEDATA = reads GLU mouseX and mouseY
   C025 KEYMODREG = reads GLU keymod register
   C026 DATAREG   = writes from the 816 go to COMMAND, reads from DATA
   C027 KMSTATUS  = GLU system status register
*/

u8 apple2gs_state::keyglu_816_read(u8 offset)
{
	switch (offset)
	{
		case GLU_C000:
			if (m_glu_regs[GLU_KG_STATUS] & KGS_KEYSTROBE)
			{
				return m_glu_regs[GLU_KEY_DATA] | 0x80;
			}
			return m_glu_regs[GLU_KEY_DATA];

		case GLU_C010:
			return (m_glu_regs[GLU_ANY_KEY_DOWN] & 0x80);

		case GLU_KEYMOD:
			return m_glu_regs[GLU_KEYMOD];

		case GLU_MOUSEX:
		case GLU_MOUSEY:
			if (!m_glu_mouse_read_stat)
			{
				if (!machine().side_effects_disabled())
					m_glu_mouse_read_stat = true;
				return m_glu_regs[GLU_MOUSEX];
			}
			if (!machine().side_effects_disabled())
			{
				m_glu_mouse_read_stat = false;
				m_glu_regs[GLU_KG_STATUS] &= ~KGS_MOUSEX_FULL;
				m_glu_regs[GLU_SYSSTAT] &= ~GLU_STATUS_MOUSEIRQ;
				keyglu_regen_irqs();
			}
			return m_glu_regs[GLU_MOUSEY];

		case GLU_SYSSTAT:
			{
				// regenerate sysstat bits
				u8 sysstat = m_glu_regs[GLU_SYSSTAT] & ~0xab;
				if (m_glu_regs[GLU_KG_STATUS] & KGS_COMMAND_FULL)
				{
					sysstat |= GLU_STATUS_CMDFULL;
				}
				if (m_glu_mouse_read_stat)
				{
					sysstat |= GLU_STATUS_MOUSEXY;
				}
				if (m_glu_regs[GLU_KG_STATUS] & KGS_KEYSTROBE)
				{
					sysstat |= GLU_STATUS_KEYDATIRQ;
				}
				if (m_glu_regs[GLU_KG_STATUS] & KGS_DATA_FULL)
				{
					sysstat |= GLU_STATUS_DATAIRQ;
				}
				if (m_glu_regs[GLU_KG_STATUS] & KGS_MOUSEX_FULL)
				{
					sysstat |= GLU_STATUS_MOUSEIRQ;
				}
				if (!machine().side_effects_disabled())
					m_glu_816_read_dstat = true;
				//printf("Read SYSSTAT %02x (PC=%x)\n", sysstat, m_maincpu->pc());
				return sysstat;
			}

		case GLU_DATA:
			if (m_glu_816_read_dstat && !machine().side_effects_disabled())
			{
				m_glu_816_read_dstat = false;
				m_glu_regs[GLU_KG_STATUS] &= ~KGS_DATA_FULL;
				keyglu_regen_irqs();
			}
			return m_glu_regs[GLU_DATA];

		default:
			return 0xff;
			break;
	}

	return 0xff;
}

void apple2gs_state::keyglu_816_write(u8 offset, u8 data)
{
	if (offset < GLU_C000)
	{
		m_glu_regs[offset&7] = data;
	}

	switch (offset)
	{
		case GLU_C010:
			m_glu_regs[GLU_SYSSTAT] &= ~GLU_STATUS_KEYDATIRQ;
			m_glu_regs[GLU_KG_STATUS] &= ~KGS_KEYSTROBE;
			keyglu_regen_irqs();
			break;

		case GLU_COMMAND:
			m_glu_regs[GLU_KG_STATUS] |= KGS_COMMAND_FULL;
			break;

		case GLU_SYSSTAT:
			m_glu_regs[GLU_SYSSTAT] &= 0xab;  // clear the non-read-only fields
			m_glu_regs[GLU_SYSSTAT] |= (data & ~0xab);
			// Table 6-7: IRQ is enabled AND data full, including when an
			// enable changes while data is already pending.  Preserve the
			// legacy default path when the optional J13 model is disabled.
			if (j13_enabled())
				keyglu_regen_irqs();
			break;
	}
}

void apple2gs_state::keyglu_regen_irqs()
{
	bool bIRQ = false;

	if ((m_glu_regs[GLU_KG_STATUS] & KGS_DATA_FULL) && (m_glu_regs[GLU_SYSSTAT] & GLU_STATUS_DATAIRQEN))
	{
		bIRQ = true;
	}

	// Unresolved silicon detail: Hardware Reference table 6-7 specifies a
	// separate keyboard IRQ latch, but the 10/24/86 Firmware Reference,
	// appendix E p. E-4, warns that C027 bits 3 and 2 do not work.  Keep the
	// legacy approximation until the actual KEYGLU behavior is established.
	if ((m_glu_regs[GLU_KG_STATUS] & KGS_KEYSTROBE) && (m_glu_regs[GLU_SYSSTAT] & GLU_STATUS_KEYDATIRQEN))
	{
		bIRQ = true;
	}

	if ((m_glu_regs[GLU_KG_STATUS] & KGS_MOUSEX_FULL) && (m_glu_regs[GLU_SYSSTAT] & GLU_STATUS_MOUSEIRQEN))
	{
		bIRQ = true;
	}

	if (bIRQ)
	{
		raise_irq(IRQS_ADB);
	}
	else
	{
		lower_irq(IRQS_ADB);
	}
}

void apple2gs_state::scc_irq_w(int state)
{
	if (state)
	{
		raise_irq(IRQS_SCC);
	}
	else
	{
		lower_irq(IRQS_SCC);
	}
}

/* Sound - DOC */
void apple2gs_state::doc_irq_w(int state)
{
	if (state)
	{
		raise_irq(IRQS_DOC);
	}
	else
	{
		lower_irq(IRQS_DOC);
	}
}

u8 apple2gs_state::doc_adc_read()
{
	return 0x80;
}

void apple2gs_state::phases_w(uint8_t phases)
{
	if (m_cur_floppy)
	{
		m_cur_floppy->seek_phase_w(phases);
	}
}

void apple2gs_state::devsel_w(uint8_t devsel)
{
	m_devsel = devsel;
	if (m_devsel == 1)
	{
		if (!BIT(m_diskreg, DISKREG_35SEL))
		{
			m_cur_floppy = m_floppy[0]->get_device();
		}
		else
		{
			m_cur_floppy = m_floppy[2]->get_device();
			m_motoroff_time = 0;
		}
	}
	else if (m_devsel == 2)
	{
		if (!BIT(m_diskreg, DISKREG_35SEL))
		{
			m_cur_floppy = m_floppy[1]->get_device();
		}
		else
		{
			m_cur_floppy = m_floppy[3]->get_device();
			m_motoroff_time = 0;
		}
	}
	else
	{
		m_cur_floppy = nullptr;
	}
	m_iwm->set_floppy(m_cur_floppy);

	if ((m_cur_floppy) && (BIT(m_diskreg, DISKREG_35SEL)))
	{
		m_cur_floppy->ss_w(BIT(m_diskreg, DISKREG_HDSEL));
	}
}

void apple2gs_state::sel35_w(int sel35)
{
}

/***************************************************************************
    INPUT PORTS
***************************************************************************/

INPUT_PORTS_START( apple2gs )
	PORT_START("twgs_rom")
	PORT_CONFNAME(0x01, 0x00, "TransWarp GS real ROM (partial)")
	PORT_CONFSETTING(0x00, DEF_STR( Off ))
	PORT_CONFSETTING(0x01, DEF_STR( On ))

	PORT_START("snd_demux")
	PORT_CONFNAME(0x01, 0x00, "DOC stereo demultiplexer")
	PORT_CONFSETTING(0x00, "Off (stock mono)")
	PORT_CONFSETTING(0x01, "On (even right, odd left)")

	PORT_START("doc_volume_law")
	PORT_CONFNAME(0x01, 0x00, "DOC volume law")
	PORT_CONFSETTING(0x00, "Linear (data sheet)")
	PORT_CONFSETTING(0x01, "Knee fitted to one real IIgs (test)")

	PORT_START("ram_start")
	PORT_CONFNAME(0x07, 0x05, "Power-on RAM contents")
	PORT_CONFSETTING(0x00, "All zeroes")
	PORT_CONFSETTING(0x05, "DRAM power-on pattern")

	PORT_START("a2_config")
	PORT_CONFNAME(0x07, 0x00, "CPU type")
	PORT_CONFSETTING(0x00, "Standard")
	PORT_CONFSETTING(0x01, "7 MHz ZipGS")
	PORT_CONFSETTING(0x03, "8 MHz ZipGS")
	PORT_CONFSETTING(0x05, "12 MHz ZipGS")
	PORT_CONFSETTING(0x07, "16 MHz ZipGS")

	PORT_START("bus_timing")
	PORT_CONFNAME(0x006, 0x006, "ZipGS cache size")
	PORT_CONFSETTING(0x000, "8 KB")
	PORT_CONFSETTING(0x002, "16 KB")
	PORT_CONFSETTING(0x004, "32 KB")
	PORT_CONFSETTING(0x006, "64 KB")
	PORT_CONFNAME(0x400, 0x000, "Accelerator of the CPU type")
	PORT_CONFSETTING(0x000, "ZipGS")
	PORT_CONFSETTING(0x400, "TransWarp GS")
	PORT_CONFNAME(0x800, 0x000, "TransWarp GS cache size")
	PORT_CONFSETTING(0x000, "8 KB")
	PORT_CONFSETTING(0x800, "32 KB")
	PORT_CONFNAME(0x1000, 0x000, "TransWarp GS AppleTalk/IRQ slowdown")
	PORT_CONFSETTING(0x000, "On")
	PORT_CONFSETTING(0x1000, "Off")
	PORT_CONFNAME(0x70000, 0x00000, "Accelerator clock")
	PORT_CONFSETTING(0x00000, "The speed of the CPU type")
	PORT_CONFSETTING(0x50000, "10 MHz")
	PORT_CONFSETTING(0x10000, "12.5 MHz")
	PORT_CONFSETTING(0x20000, "13.75 MHz")
	PORT_CONFSETTING(0x30000, "14 MHz")
	PORT_CONFSETTING(0x40000, "15 MHz")
	PORT_CONFNAME(0x2000, 0x0000, "Native-speed fast RAM (not a real card)")
	PORT_CONFSETTING(0x0000, DEF_STR( Off ))
	PORT_CONFSETTING(0x2000, DEF_STR( On ))
INPUT_PORTS_END

INPUT_PORTS_START( apple2gsj13 )
	PORT_INCLUDE( apple2gs )

	PORT_START("j13_config")
	PORT_CONFNAME(0x01, 0x00, "Built-in keyboard (J13)") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(apple2gs_state::j13_config_changed), 0)
	PORT_CONFSETTING(0x00, "None (stock)")
	PORT_CONFSETTING(0x01, "Apple IIe keyboard")
	PORT_CONFNAME(0x02, 0x00, "ADB keyboard") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(apple2gs_state::j13_config_changed), 0)
	PORT_CONFSETTING(0x00, "Connected (stock)")
	PORT_CONFSETTING(0x02, "None")

	// Apple drawing 699-0076-C, sheet 4: IIe matrix, indexed by Y then X.
	// The external numeric pad (X4-X7, Y0-Y5) is not connected.
	PORT_START("j13_y0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Escape") PORT_CODE(KEYCODE_ESC) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Tab") PORT_CODE(KEYCODE_TAB) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 A") PORT_CODE(KEYCODE_A) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Z") PORT_CODE(KEYCODE_Z) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_START("j13_y1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 1") PORT_CODE(KEYCODE_1) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Q") PORT_CODE(KEYCODE_Q) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 D") PORT_CODE(KEYCODE_D) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 X") PORT_CODE(KEYCODE_X) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_START("j13_y2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 2") PORT_CODE(KEYCODE_2) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 W") PORT_CODE(KEYCODE_W) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 S") PORT_CODE(KEYCODE_S) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 C") PORT_CODE(KEYCODE_C) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_START("j13_y3")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 3") PORT_CODE(KEYCODE_3) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 E") PORT_CODE(KEYCODE_E) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 H") PORT_CODE(KEYCODE_H) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 V") PORT_CODE(KEYCODE_V) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_START("j13_y4")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 4") PORT_CODE(KEYCODE_4) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 R") PORT_CODE(KEYCODE_R) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 F") PORT_CODE(KEYCODE_F) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 B") PORT_CODE(KEYCODE_B) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_START("j13_y5")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 6") PORT_CODE(KEYCODE_6) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Y") PORT_CODE(KEYCODE_Y) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 G") PORT_CODE(KEYCODE_G) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 N") PORT_CODE(KEYCODE_N) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_START("j13_y6")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 5") PORT_CODE(KEYCODE_5) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 T") PORT_CODE(KEYCODE_T) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 J") PORT_CODE(KEYCODE_J) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 M") PORT_CODE(KEYCODE_M) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Backslash") PORT_CODE(KEYCODE_BACKSLASH) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Grave") PORT_CODE(KEYCODE_TILDE) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Return") PORT_CODE(KEYCODE_ENTER) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Delete") PORT_CODE(KEYCODE_BACKSPACE) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_START("j13_y7")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 7") PORT_CODE(KEYCODE_7) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 U") PORT_CODE(KEYCODE_U) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 K") PORT_CODE(KEYCODE_K) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Comma") PORT_CODE(KEYCODE_COMMA) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Equals") PORT_CODE(KEYCODE_EQUALS) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 P") PORT_CODE(KEYCODE_P) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Up") PORT_CODE(KEYCODE_UP) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Down") PORT_CODE(KEYCODE_DOWN) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_START("j13_y8")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 8") PORT_CODE(KEYCODE_8) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 I") PORT_CODE(KEYCODE_I) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Semicolon") PORT_CODE(KEYCODE_COLON) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Period") PORT_CODE(KEYCODE_STOP) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 0") PORT_CODE(KEYCODE_0) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Left bracket") PORT_CODE(KEYCODE_OPENBRACE) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Space") PORT_CODE(KEYCODE_SPACE) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Left") PORT_CODE(KEYCODE_LEFT) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_START("j13_y9")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 9") PORT_CODE(KEYCODE_9) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 O") PORT_CODE(KEYCODE_O) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 L") PORT_CODE(KEYCODE_L) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Slash") PORT_CODE(KEYCODE_SLASH) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Minus") PORT_CODE(KEYCODE_MINUS) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Right bracket") PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Quote") PORT_CODE(KEYCODE_QUOTE) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Right") PORT_CODE(KEYCODE_RIGHT) PORT_CONDITION("j13_config", 0x01, EQUALS, 0x01)

	PORT_START("j13_special")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Shift") PORT_CODE(KEYCODE_LSHIFT) PORT_CODE(KEYCODE_RSHIFT)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Control") PORT_CODE(KEYCODE_LCONTROL) PORT_CODE(KEYCODE_RCONTROL)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Caps Lock") PORT_CODE(KEYCODE_CAPSLOCK) PORT_TOGGLE
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("J13 Open Apple") PORT_CODE(KEYCODE_LALT)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("J13 Closed Apple") PORT_CODE(KEYCODE_RALT)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J13 Reset") PORT_CODE(KEYCODE_F12)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_UNUSED)
INPUT_PORTS_END

INPUT_PORTS_START( apple2gsrom3 )
	PORT_INCLUDE( apple2gs )

	PORT_MODIFY("a2_config")
	PORT_CONFNAME(0x08, 0x00, "Disable CDA Control Panel")
	PORT_CONFSETTING(0x00, DEF_STR(No))
	PORT_CONFSETTING(0x08, DEF_STR(Yes))
INPUT_PORTS_END

void apple2gs_state::apple2gs(machine_config &config)
{
	/* basic machine hardware */
	G65816(config, m_maincpu, A2GS_2_8M);
	m_maincpu->set_addrmap(AS_PROGRAM, &apple2gs_state::apple2gs_map);
	m_maincpu->set_addrmap(g65816_device::AS_VECTORS, &apple2gs_state::vectors_map);
	m_maincpu->set_dasm_override(FUNC(apple2gs_state::dasm_trampoline));
	m_maincpu->wdm_handler().set(FUNC(apple2gs_state::wdm_trampoline));
	TIMER(config, m_scantimer).configure_generic(FUNC(apple2gs_state::apple2_interrupt));
	TIMER(config, m_vgctimer).configure_generic(FUNC(apple2gs_state::apple2_vgc));
	TIMER(config, m_acceltimer).configure_generic(FUNC(apple2gs_state::accel_timer));

	config.set_maximum_quantum(attotime::from_hz(60));

	M50741(config, m_adbmicro, A2GS_MASTER_CLOCK/8);
	m_adbmicro->set_pullups<2>(0x20);
	m_adbmicro->read_p<0>().set(FUNC(apple2gs_state::adbmicro_p0_in));
	m_adbmicro->write_p<0>().set(FUNC(apple2gs_state::adbmicro_p0_out));
	m_adbmicro->read_p<1>().set(FUNC(apple2gs_state::adbmicro_p1_in));
	m_adbmicro->write_p<1>().set(FUNC(apple2gs_state::adbmicro_p1_out));
	m_adbmicro->read_p<2>().set(FUNC(apple2gs_state::adbmicro_p2_in));
	m_adbmicro->write_p<2>().set(FUNC(apple2gs_state::adbmicro_p2_out));
	m_adbmicro->read_p<3>().set(FUNC(apple2gs_state::adbmicro_p3_in));
	m_adbmicro->write_p<3>().set(FUNC(apple2gs_state::adbmicro_p3_out));

	MACADB(config, m_macadb, A2GS_MASTER_CLOCK/8);
	m_macadb->adb_data_callback().set(FUNC(apple2gs_state::set_adb_line));

	RTC3430042(config, m_rtc, XTAL(32'768));
	m_rtc->cko_cb().set(FUNC(apple2gs_state::rtc_vgc));

	APPLE2_VIDEO(config, m_video, A2GS_14M).set_screen(m_screen);
	m_video->set_base_model(a2_video_device::model::IIGS);

	APPLE2_COMMON(config, m_a2common, A2GS_14M).set_GS_cputag(m_maincpu);

//  APPLE2_HOST(config, m_a2host, A2GS_14M);
//  m_a2host->set_cputag(m_maincpu);
//  m_a2host->set_space(m_maincpu, AS_PROGRAM);

	APPLE2_GAMEIO(config, m_gameio, apple2_gameio_device::default_options, nullptr);

	// HBL is positioned to the right of active video here, but to the left on hardware.
	// the borders are counted as active video here, but as HBL on hardware.
	// these must be compensated for in any use of hpos/vpos/hblank/vblank.
	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	m_screen->set_raw(A2GS_1M.value() * 16, 65 * 16, 0, BORDER_LEFT + 40 * 16 + BORDER_RIGHT, 262, 0, 231);
	m_screen->set_screen_update(m_video, NAME((&a2_video_device::screen_update_GS)));

	PALETTE(config, "palette", FUNC(apple2gs_state::palette_init), 256);

	/* sound hardware */
	// One mono sum (speaker, headphone, video pin) and the molex demultiplexer
	// (even DOC channels right, odd channels left). sndglu_apply_vca() mutes
	// the path that is not selected and applies the 16-step VCA to the other.
	SPEAKER(config, "sndmono").front_center();
	SPEAKER(config, "sndleft").front_left();
	SPEAKER(config, "sndright").front_right();
	SPEAKER_SOUND(config, m_speaker);
	m_speaker->add_route(ALL_OUTPUTS, "sndmono", 1.00);
	m_speaker->add_route(ALL_OUTPUTS, "sndleft", 1.00);
	m_speaker->add_route(ALL_OUTPUTS, "sndright", 1.00);

	ES5503(config, m_doc, A2GS_7M);
	m_doc->set_channels(2);
	// ROM03 sound sheet 8: SR51/SR2/SR15=15k, SC24=1.5nF, SC15=3.9nF,
	// SC14=220pF; SR16/SR43=10k, SC17=3.9nF, SC16=560pF. Including
	// the loading between stages, H(s)=1/((1+3.24e-5*s+5.346e-10*s^2+
	// 4.343625e-15*s^3)*(1+1.12e-5*s+2.184e-10*s^2)). These are its
	// poles and steady-state residues, not a fit to an audio recording.
	m_doc->set_output_filter(
		{{{-62671.010935312021, 0}, {-30202.956070805525, 52547.97163933207}, {-25641.025641025644, 62620.223433255531}}},
		{{{0.83295036787945753, -0}, {0.85237541426630326, -1.9216565533963892}, {-0.76885059820603185, 1.0994546201934139}}}, 3);
	m_doc->set_addrmap(0, &apple2gs_state::a2gs_es5503_map);
	m_doc->irq_func().set(FUNC(apple2gs_state::doc_irq_w));
	m_doc->adc_func().set(FUNC(apple2gs_state::doc_adc_read));
	m_doc->add_route(0, "sndmono", 1.0);
	m_doc->add_route(1, "sndmono", 1.0);
	m_doc->add_route(0, "sndright", 1.0);
	m_doc->add_route(1, "sndleft", 1.0);

	/* RAM */
	EEPROM_X24C44_16BIT(config, m_twgs_nvram);

	RAM(config, m_ram).set_default_size("2M").set_extra_options("1M,3M,4M,5M,6M,7M,8M").set_default_value(0x00);

	/* C300 banking */
	ADDRESS_MAP_BANK(config, A2GS_C300_TAG).set_map(&apple2gs_state::c300bank_map).set_options(ENDIANNESS_LITTLE, 8, 32, 0x100);

	/* serial */
	SCC85C30(config, m_scc, A2GS_7M);
	m_scc->configure_channels(3'686'400, 3'686'400, 3'686'400, 3'686'400);
	m_scc->out_int_callback().set(FUNC(apple2gs_state::scc_irq_w));
	m_scc->out_txda_callback().set("printer", FUNC(rs232_port_device::write_txd));
	m_scc->out_txdb_callback().set("modem", FUNC(rs232_port_device::write_txd));

	rs232_port_device &rs232a(RS232_PORT(config, "printer", default_rs232_devices, nullptr));
	rs232a.rxd_handler().set(m_scc, FUNC(z80scc_device::rxa_w));
	rs232a.dcd_handler().set(m_scc, FUNC(z80scc_device::dcda_w));
	rs232a.cts_handler().set(m_scc, FUNC(z80scc_device::ctsa_w));

	rs232_port_device &rs232b(RS232_PORT(config, "modem", default_rs232_devices, nullptr));
	rs232b.rxd_handler().set(m_scc, FUNC(z80scc_device::rxb_w));
	rs232b.dcd_handler().set(m_scc, FUNC(z80scc_device::dcdb_w));
	rs232b.cts_handler().set(m_scc, FUNC(z80scc_device::ctsb_w));

	/* slot devices */
	A2BUS(config, m_a2bus);
	m_a2bus->set_space(m_maincpu, AS_PROGRAM);
	m_a2bus->irq_w().set(FUNC(apple2gs_state::a2bus_irq_w));
	m_a2bus->nmi_w().set(FUNC(apple2gs_state::a2bus_nmi_w));
	m_a2bus->inh_w().set(FUNC(apple2gs_state::a2bus_inh_w));
	m_a2bus->dma_w().set_inputline(m_maincpu, INPUT_LINE_HALT);
	A2BUS_SLOT(config, "sl1", A2GS_7M, m_a2bus, apple2gs_cards, nullptr);
	A2BUS_SLOT(config, "sl2", A2GS_7M, m_a2bus, apple2gs_cards, nullptr);
	A2BUS_SLOT(config, "sl3", A2GS_7M, m_a2bus, apple2gs_cards, nullptr);
	A2BUS_SLOT(config, "sl4", A2GS_7M, m_a2bus, apple2gs_cards, nullptr);
	A2BUS_SLOT(config, "sl5", A2GS_7M, m_a2bus, apple2gs_cards, nullptr);
	A2BUS_SLOT(config, "sl6", A2GS_7M, m_a2bus, apple2gs_cards, nullptr);
	A2BUS_SLOT(config, "sl7", A2GS_7M, m_a2bus, apple2gs_cards, nullptr);

	IWM(config, m_iwm, A2GS_7M, A2GS_1M*2);
	m_iwm->phases_cb().set(FUNC(apple2gs_state::phases_w));
	m_iwm->sel35_cb().set(FUNC(apple2gs_state::sel35_w));
	m_iwm->devsel_cb().set(FUNC(apple2gs_state::devsel_w));

	applefdintf_device::add_525(config, m_floppy[0]);
	applefdintf_device::add_525(config, m_floppy[1]);
	applefdintf_device::add_35(config, m_floppy[2]);
	applefdintf_device::add_35(config, m_floppy[3]);

	SOFTWARE_LIST(config, "flop_gs_clean").set_original("apple2gs_flop_clcracked"); // GS-specific cleanly cracked disks
	SOFTWARE_LIST(config, "flop_gs_orig").set_compatible("apple2gs_flop_orig"); // Original disks for GS
	SOFTWARE_LIST(config, "flop_gs_misc").set_compatible("apple2gs_flop_misc"); // Legacy software list pre-June 2021 and defaced cracks
	SOFTWARE_LIST(config, "flop_a2_clean").set_compatible("apple2_flop_clcracked"); // Apple II series cleanly cracked
	SOFTWARE_LIST(config, "flop_a2_orig").set_compatible("apple2_flop_orig").set_filter("A2GS");  // Filter list to compatible disks for this machine.
	SOFTWARE_LIST(config, "flop_a2_misc").set_compatible("apple2_flop_misc");
}

void apple2gs_state::apple2gsr1(machine_config &config)
{
	apple2gs(config);
	m_macadb->keyboard_connected_callback().set(FUNC(apple2gs_state::adb_keyboard_connected));
	m_macadb->standard_timing_callback().set(FUNC(apple2gs_state::j13_enabled));

	// ROM01 sound sheet 8 omits ROM03 SR51/SR2/SC24. Its first follower
	// is a single SR15(15k)/SC14(220pF) pole, followed by the same second pair.
	m_doc->set_output_filter(
		{{{-303030.30303030304, 0}, {-25641.025641025644, 62620.223433255531}, {0, 0}}},
		{{{0.056621431913898006, -0}, {0.47168928404305094, -0.33014257581209661}, {0, 0}}}, 2);

	// 256K on board + 1M in the expansion slot was common for ROM 01
	m_ram->set_default_size("1280K").set_extra_options("256K,512K,768K,1M,2M,3M,4M,4224K,4352K,5M,6M,7M,8M").set_default_value(0x00);

	M50740(config.replace(), m_adbmicro, A2GS_MASTER_CLOCK/8);
	m_adbmicro->set_pullups<2>(0x20);
	m_adbmicro->read_p<0>().set(FUNC(apple2gs_state::adbmicro_p0_in));
	m_adbmicro->write_p<0>().set(FUNC(apple2gs_state::adbmicro_p0_out));
	m_adbmicro->read_p<1>().set(FUNC(apple2gs_state::adbmicro_p1_in));
	m_adbmicro->write_p<1>().set(FUNC(apple2gs_state::adbmicro_p1_out));
	m_adbmicro->read_p<2>().set(FUNC(apple2gs_state::adbmicro_p2_in));
	m_adbmicro->write_p<2>().set(FUNC(apple2gs_state::adbmicro_p2_out));
	m_adbmicro->read_p<3>().set(FUNC(apple2gs_state::adbmicro_p3_in));
	m_adbmicro->write_p<3>().set(FUNC(apple2gs_state::adbmicro_p3_out));
}

void apple2gs_state::apple2gsmt(machine_config &config)
{
	apple2gs(config);

	// SWIM1 344S0061 confirmed on two different Mark Twain boards, with 15.6672 MHz oscillator
	SWIM1(config.replace(), m_iwm, 15.6672_MHz_XTAL);
	m_iwm->phases_cb().set(FUNC(apple2gs_state::phases_w));
	m_iwm->sel35_cb().set(FUNC(apple2gs_state::sel35_w));
	m_iwm->devsel_cb().set(FUNC(apple2gs_state::devsel_w));

	applefdintf_device::add_525(config, m_floppy[0]);
	applefdintf_device::add_525(config, m_floppy[1]);
	applefdintf_device::add_35_hd(config, m_floppy[2]);
	applefdintf_device::add_35_hd(config, m_floppy[3]);
}

/***************************************************************************

  Game driver(s)

***************************************************************************/
ROM_START(apple2gs)
	ROM_REGION(0x8000, "twgs", ROMREGION_ERASEFF)
	ROM_LOAD_OPTIONAL("twgs_1.8s.bin", 0, 0x8000, CRC(f42c1e38) SHA1(65fa112b68cd804a63081aa8ff5bf7a600d5cbba))
	// M50740/50741 ADB MCU inside the IIgs system unit
	ROM_REGION(0x1000, "adbmicro", 0)
	ROM_LOAD( "341s0632-2.bin", 0x000000, 0x001000, CRC(e1c11fb0) SHA1(141d18c36a617ab9dce668445440d34354be0672) )

	ROM_REGION(0x4000,"gfx1",0)
	ROM_LOAD("344s0047.bin", 0x000000, 0x004000, CRC(2d541944) SHA1(5a5a77c8ec45632aea1a57cd9c9257f7f6e44668)) /* Mega II: 344S0047.  Dumped from the test registers, verified identical on both ROM 1 and 3 */

	ROM_REGION(0x80000,"maincpu",0)
	// 341-0728 is the MASK rom version while 341-0737 is the EPROM version - SAME data.
	ROM_LOAD("341-0728", 0x00000, 0x20000, CRC(8d410067) SHA1(c0f4704233ead14cb8e1e8a68fbd7063c56afd27) ) /* 341-0728: IIgs ROM03 FC-FD */
	// 341-0748 is the MASK rom version while 341-0749 is the EPROM version - SAME data.
	ROM_LOAD("341-0748", 0x30000, 0x10000, CRC(18190283) SHA1(c70576869deec92ca82c78438b1d5c686eac7480) ) /* 341-0748: IIgs ROM03 FE-FF */
	ROM_CONTINUE ( 0x20000, 0x10000) /* high address line is inverted on PCB? */
ROM_END

ROM_START(apple2gsr3p)
	ROM_REGION(0x8000, "twgs", ROMREGION_ERASEFF)
	ROM_LOAD_OPTIONAL("twgs_1.8s.bin", 0, 0x8000, CRC(f42c1e38) SHA1(65fa112b68cd804a63081aa8ff5bf7a600d5cbba))
	ROM_REGION(0x1000, "adbmicro", 0)
	ROM_LOAD( "341s0632-2.bin", 0x000000, 0x001000, CRC(e1c11fb0) SHA1(141d18c36a617ab9dce668445440d34354be0672) )

	ROM_REGION(0x4000,"gfx1",0)
	ROM_LOAD("344s0047.bin", 0x000000, 0x004000, CRC(2d541944) SHA1(5a5a77c8ec45632aea1a57cd9c9257f7f6e44668)) /* Mega II: 344S0047.  Dumped from the test registers, verified identical on both ROM 1 and 3 */

	ROM_REGION(0x80000,"maincpu",0)
	ROM_LOAD("341-0728", 0x00000, 0x20000, CRC(8d410067) SHA1(c0f4704233ead14cb8e1e8a68fbd7063c56afd27) ) /* 341-0728: IIgs ROM03 prototype FC-FD - 28 pin MASK rom */
	ROM_LOAD("341-0729", 0x20000, 0x20000, NO_DUMP) /* 341-0729: IIgs ROM03 prototype FE-FF */
ROM_END

ROM_START(apple2gsr1)
	ROM_REGION(0x8000, "twgs", ROMREGION_ERASEFF)
	ROM_LOAD_OPTIONAL("twgs_1.8s.bin", 0, 0x8000, CRC(f42c1e38) SHA1(65fa112b68cd804a63081aa8ff5bf7a600d5cbba))
	ROM_REGION(0xc00, "adbmicro", 0)
	ROM_LOAD( "341s0345.bin", 0x000000, 0x000c00, CRC(48cd5779) SHA1(97e421f5247c00a0ca34cd08b6209df573101480) )

	ROM_REGION(0x4000,"gfx1",0)
	ROM_LOAD("344s0047.bin", 0x000000, 0x004000, CRC(2d541944) SHA1(5a5a77c8ec45632aea1a57cd9c9257f7f6e44668)) /* Mega II: 344S0047.  Dumped from the test registers, verified identical on both ROM 1 and 3 */

	ROM_REGION(0x80000,"maincpu",0)
	ROM_LOAD("342-0077-b", 0x20000, 0x20000, CRC(42f124b0) SHA1(e4fc7560b69d062cb2da5b1ffbe11cd1ca03cc37)) /* 342-0077-B: IIgs ROM01 */
ROM_END

ROM_START(apple2gsr0)
	ROM_REGION(0x8000, "twgs", ROMREGION_ERASEFF)
	ROM_LOAD_OPTIONAL("twgs_1.8s.bin", 0, 0x8000, CRC(f42c1e38) SHA1(65fa112b68cd804a63081aa8ff5bf7a600d5cbba))
	ROM_REGION(0xc00, "adbmicro", 0)
	ROM_LOAD( "341s0345.bin", 0x000000, 0x000c00, CRC(48cd5779) SHA1(97e421f5247c00a0ca34cd08b6209df573101480) )

	ROM_REGION(0x4000,"gfx1",0)
	ROM_LOAD("344s0047.bin", 0x000000, 0x004000, CRC(2d541944) SHA1(5a5a77c8ec45632aea1a57cd9c9257f7f6e44668)) /* Mega II: 344S0047.  Dumped from the test registers, verified identical on both ROM 1 and 3 */

	ROM_REGION(0x80000,"maincpu",0)
	ROM_LOAD("342-0077-a", 0x20000, 0x20000, CRC(dfbdd97b) SHA1(ff0c245dd0732ec4413a934fd80efc2defd8a8e3) ) /* 342-0077-A: IIgs ROM00 */
ROM_END

ROM_START(apple2gsr0p)  // 6/19/1986 Cortland prototype

	ROM_REGION(0x8000, "twgs", ROMREGION_ERASEFF)
	ROM_LOAD_OPTIONAL("twgs_1.8s.bin", 0, 0x8000, CRC(f42c1e38) SHA1(65fa112b68cd804a63081aa8ff5bf7a600d5cbba))
	ROM_REGION(0xc00, "adbmicro", 0)
	ROM_LOAD( "341s0345.bin", 0x000000, 0x000c00, CRC(48cd5779) SHA1(97e421f5247c00a0ca34cd08b6209df573101480) )

	ROM_REGION(0x4000,"gfx1",0)
	ROM_LOAD("344s0047.bin", 0x000000, 0x004000, CRC(2d541944) SHA1(5a5a77c8ec45632aea1a57cd9c9257f7f6e44668)) /* Mega II: 344S0047.  Dumped from the test registers, verified identical on both ROM 1 and 3 */

	ROM_REGION(0x80000,"maincpu",0)
	ROM_LOAD( "rombf.bin",    0x020000, 0x020000, CRC(ab04fedf) SHA1(977589a17553956d583a21020080a39dd396df5c) )
ROM_END

ROM_START(apple2gsr0p2)  // 3/10/1986 Cortland prototype, boots as "Apple //'ing - Alpha 2.0"

	ROM_REGION(0x8000, "twgs", ROMREGION_ERASEFF)
	ROM_LOAD_OPTIONAL("twgs_1.8s.bin", 0, 0x8000, CRC(f42c1e38) SHA1(65fa112b68cd804a63081aa8ff5bf7a600d5cbba))
	ROM_REGION(0xc00, "adbmicro", 0)
	ROM_LOAD( "341s0345.bin", 0x000000, 0x000c00, CRC(48cd5779) SHA1(97e421f5247c00a0ca34cd08b6209df573101480) )

	ROM_REGION(0x4000,"gfx1",0)
	ROM_LOAD("344s0047.bin", 0x000000, 0x004000, CRC(2d541944) SHA1(5a5a77c8ec45632aea1a57cd9c9257f7f6e44668)) /* Mega II: 344S0047.  Dumped from the test registers, verified identical on both ROM 1 and 3 */

	ROM_REGION(0x80000,"maincpu",0)
	ROM_LOAD( "apple iigs alpha rom 2.0 19860310.bin", 0x020000, 0x020000, CRC(a47d275f) SHA1(c5836adcfc8be69c7351b84afa94c814e8d92b81) )
ROM_END

ROM_START(apple2gsmt)
	ROM_REGION(0x8000, "twgs", ROMREGION_ERASEFF)
	ROM_LOAD_OPTIONAL("twgs_1.8s.bin", 0, 0x8000, CRC(f42c1e38) SHA1(65fa112b68cd804a63081aa8ff5bf7a600d5cbba))
	// 50741 ADB MCU inside the IIgs system unit
	ROM_REGION(0x1000, "adbmicro", 0)
	ROM_LOAD( "341s0632-2.bin", 0x000000, 0x001000, CRC(e1c11fb0) SHA1(141d18c36a617ab9dce668445440d34354be0672) )

	ROM_REGION(0x4000, "gfx1", 0)
	ROM_LOAD("344s0047.bin", 0x000000, 0x004000, CRC(2d541944) SHA1(5a5a77c8ec45632aea1a57cd9c9257f7f6e44668)) /* Mega II: 344S0047.  Dumped from the test registers, verified identical on both ROM 1 and 3 */

	ROM_REGION(0x80000, "maincpu", 0)
	// The Mark Twain ROM is 512K, with address bit 16 inverted (same as ROM 3).
	// The first 256K is filled with 64K of 0xF0, then 0xF1, 0xF2, and 0xF3.  I'm guessing this was meant to be
	// a small ROM disk at $F00000, like the Mac Classic has.  The second 256K is the firmware we all know from
	// the "System 6.0.1" source leak.
	ROM_LOAD( "mtrom.bin", 0x040000, 0x040000, CRC(d75414c5) SHA1(7054915f5e5f9f3bb2cbecf6830d4f80793694a6) )
	ROM_CONTINUE(0x10000, 0x10000)
	ROM_CONTINUE(0x00000, 0x10000)
	ROM_CONTINUE(0x30000, 0x10000)
	ROM_CONTINUE(0x20000, 0x10000)

	// The firmware for the built-in High-Speed SCSI Card
	ROM_REGION(0x8000, "hsscsi", 0)
	ROM_LOAD( "mtscsi.bin",   0x000000, 0x008000, CRC(7426c880) SHA1(1c16310e5c180701a05089d69c6e72e9dc7434f6) )
ROM_END

} // Anonymous namespace

/*    YEAR  NAME          PARENT    COMPAT  MACHINE     INPUT         CLASS           INIT       COMPANY           FULLNAME */
COMP( 1989, apple2gs,     0,        apple2, apple2gs,   apple2gsrom3, apple2gs_state, rom3_init, "Apple Computer", "Apple IIgs (ROM03)", MACHINE_SUPPORTS_SAVE )
COMP( 198?, apple2gsr3p,  apple2gs, 0,      apple2gs,   apple2gsrom3, apple2gs_state, rom3_init, "Apple Computer", "Apple IIgs (ROM03 prototype)", MACHINE_NOT_WORKING )
COMP( 1987, apple2gsr1,   apple2gs, 0,      apple2gsr1, apple2gsj13,  apple2gs_state, rom1_init, "Apple Computer", "Apple IIgs (ROM01)", MACHINE_SUPPORTS_SAVE )
COMP( 1986, apple2gsr0,   apple2gs, 0,      apple2gsr1, apple2gsj13,  apple2gs_state, rom1_init, "Apple Computer", "Apple IIgs (ROM00)", MACHINE_SUPPORTS_SAVE )
COMP( 1986, apple2gsr0p,  apple2gs, 0,      apple2gsr1, apple2gsj13,  apple2gs_state, rom1_init, "Apple Computer", "Apple IIgs (ROM00 prototype 6/19/1986)", MACHINE_SUPPORTS_SAVE )
COMP( 1986, apple2gsr0p2, apple2gs, 0,      apple2gsr1, apple2gsj13,  apple2gs_state, rom1_init, "Apple Computer", "Apple IIgs (ROM00 prototype 3/10/1986)", MACHINE_SUPPORTS_SAVE )
COMP( 1991, apple2gsmt,   apple2gs, 0,      apple2gsmt, apple2gsrom3, apple2gs_state, rom3_init, "Apple Computer", "Apple IIgs (1991 Mark Twain prototype)", MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )

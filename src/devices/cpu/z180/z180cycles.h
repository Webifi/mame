// license:BSD-3-Clause
// Private state of the resumable Z180 machine-cycle engine.
// Included inside z180_device, not a separate device or CPU implementation.
enum class cycle_kind : u8
{
	fetch,
	operand,
	read,
	write,
	input,
	output,
	internal,
	acknowledge,
	nmi,
	refresh,
	halt,
	release
};
enum class cycle_owner : u8
{
	cpu,
	dma,
	refresh
};

struct cycle_yield
{
};

// A scheduler suspension discards only trivial ISA/engine stack locals.
// Architectural state and the unfinished edge are already in the journal.
// This jump target lives only during execute_machine_cycles; never save it.
std::jmp_buf m_cycle_resume;

struct cycle_record
{
	cycle_kind kind = cycle_kind::internal;
	u32 address = 0;
	u8 data = 0;
	u8 nominal = 0;
};

struct cycle_job
{
	cycle_kind kind = cycle_kind::internal;
	cycle_owner owner = cycle_owner::cpu;
	u32 address = 0;
	u8 data = 0;
	u8 phase = 0;
	u16 waits = 0;
	u16 elapsed = 0;
	u8 nominal = 3;
	u8 trailing = 0;
	u8 io_select_edge = 0;
	bool active = false;
	bool sampled = false;
	bool ttp = false;
	bool no_arbitration = false;
	bool internal_io = false;
	bool ack_selected = false;
};

struct instruction_state
{
	u32 regs[12]{};
	u8 control[9]{};
};

bool m_cycle_engine = false;
bool m_cycle_reference = false;
bool m_cycle_grants = false;
bool m_cycle_executing = false;
bool m_instruction_fast = false;
bool m_instruction_whole = false;
// Derived validity of the provisional ISA state at the suspended bus job.
// Debugger imports and save/load invalidate it; it is not hardware state.
bool m_replay_pristine = false;
bool m_fast_irq_valid = false;
int m_fast_interrupt = -1;
bool m_instruction_active = false;
bool m_wait_line = false;
bool m_bus_acknowledged = false;
bool m_sampled_bus_request = false;
bool m_boundary_allowed = true;
bool m_dma_allowed = false;
bool m_dma_start_pending = false;
bool m_refresh_pending = false;
bool m_refresh_after_sleep = false;
bool m_dma_edge[2]{};
u32 m_sampled_iol = 0;
u64 m_dma_edge_generation[2]{}, m_dma_edge_accepted[2]{}, m_dma_edge_sampled[2]{};
u8 m_phi_half = 0;
u8 m_refresh_address = 0;
u16 m_refresh_phase = 0;
u16 m_advance_left = 0;
u64 m_half_phi = 0;
// Saved oscillator-stop offset: service phase is derived only on internal
// data I/O, never maintained by a per-PHI hot-path update.
u64 m_io_service_stopped = 0;
u32 m_power_wake_left = 0;
bool m_power_bus_only = false;
bool cycle_halt();
bool cycle_low_power();
unsigned cycle_internal_io_wait(offs_t address, bool write) const;
// Complete certified reads/Ti may compose these peripheral clocks. This is
// flushed before any callback, phase-engine entry or scheduler exit, never
// saved as hardware state. The scheduler budget and bus time stay current.
unsigned m_quiet_phi = 0;
u8 m_quiet_record = 0;
bool m_quiet_no_arbitration = false;
u64 m_retired = 0;
u64 m_fast_instructions = 0;
u64 m_resumptions = 0;
cycle_job m_cycle;
instruction_state m_instruction_start;
cycle_record m_history[40]{};
u8 m_history_count = 0;
unsigned m_cursor = 0, m_bus_number = 0, m_nominal = 0;
unsigned m_family = 0, m_opcode = 0;
u32 m_prelude_schedule = 0;
bool m_trapping = false, m_third_trap = false;
bool m_reti_notified = false;
bool m_trap_started = false;
bool m_interrupt_started = false;
int m_sampled_interrupt = -1;
bool m_interrupt_recovery = false;
bool m_dma_nmi = false;
int m_instruction_interrupt = -1;
int m_ack_vector = -1;
u8 m_dma_channel = 0xff;
u8 m_dma_stage = 0;
u8 m_dma_continue = 0xff;
u8 m_dma_byte = 0;
u32 m_dma_source = 0, m_dma_destination = 0;
bool m_dma_source_io = false, m_dma_destination_io = false;
u8 m_dma_source_mode = 0, m_dma_destination_mode = 0;
std::function<unsigned(offs_t, bool, bool)> m_bus_wait;
std::function<void(offs_t, bool, u8, bool)> m_io_strobe;
std::function<void(bool)> m_irq0_ack_strobe;
std::function<void(u64, u8, u8, offs_t, u8)> m_cycle_observer;
std::function<void(bool)> m_busack_cb;
std::array<u8 const *, 16> m_direct_read{};
// Derived composition of the MMU, multiplexed A18 and certified physical
// read views. These pointers cache address decoding only, never read data.
std::array<u8 const *, 16> m_direct_logical{};
u8 *m_direct_read_bus = nullptr;
bool m_direct_reads_enabled = false;
void cycle_update_direct_reads();

u8 const *cycle_direct_read(offs_t address) const
{
	if (m_cycle_reference || !m_direct_reads_enabled)
		return nullptr;
	address = a18_tout_address(address);
	u8 const *const page = m_direct_read[(address >> 16) & 15];
	return page ? page + (address & 0xffff) : nullptr;
}

void execute_legacy();
void execute_machine_cycles();
[[noreturn]] void cycle_suspend();
ATTR_FORCE_INLINE bool cycle_next_grant();
bool cycle_continue_grant();
void cycle_save_state();
void cycle_restore_state();
void cycle_register_save();
void cycle_reset();
void cycle_decode(unsigned family, unsigned opcode);
unsigned cycle_prelude(cycle_kind kind) const;
unsigned cycle_prelude_reference(cycle_kind kind) const;
ATTR_FORCE_INLINE u8 cycle_access(cycle_kind kind, offs_t address, u8 data = 0);
u8 cycle_access_slow(cycle_kind kind, offs_t address, u8 data, unsigned prelude);
ATTR_FORCE_INLINE u8 cycle_whole_access(cycle_kind kind, offs_t address, u8 data, unsigned prelude);
ATTR_FORCE_INLINE u8 cycle_certified_read(cycle_kind kind, offs_t address, u8 const *direct);
u8 cycle_mapped_access(cycle_kind kind, offs_t address, offs_t physical, u8 data);
void cycle_whole_internal(unsigned count, bool no_arbitration = false);
void cycle_defer_clocks(unsigned phi);
void cycle_flush_quiet();
void cycle_sample_stable_inputs();
void cycle_resume_admission();
void cycle_advance_quiet(unsigned ticks);
// A counting PRT is a timed source only when it can raise an interrupt
// (TIE0, TIE1) or drive TOUT (TOC1, TOC0). Otherwise its count and flags are
// seen only through its registers, and every register access first brings
// the deferred clocks up to date.
bool cycle_prt_timed() const { return ((m_tcr & 0x01) && (m_tcr & 0x10)) || ((m_tcr & 0x02) && (m_tcr & 0x2c)); }
void cycle_quiet_timers(unsigned phi);
u8 cycle_transaction(cycle_kind kind, offs_t address, u8 data, bool no_arbitration = false);
void cycle_begin(cycle_owner owner, cycle_kind kind, offs_t address, u8 data = 0, bool no_arbitration = false);
void cycle_run();
bool cycle_run_unobserved();
bool cycle_run_io_unobserved();
bool cycle_run_partial();
unsigned cycle_resume_quiet();
void cycle_sample_opcode();
void cycle_sample_data();
void cycle_advance(unsigned half_phi);
void cycle_arbitrate();
void cycle_dma_begin(unsigned channel);
void cycle_dma_run();
bool cycle_dma_ready(unsigned channel) const;
void cycle_finish_cpu(int nominal);
void cycle_emit(u8 edge);
int cycle_interrupt();
int cycle_choose_interrupt();
void cycle_sample_interrupts();
void cycle_reti();

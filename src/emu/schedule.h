// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    schedule.h

    Core device execution and scheduling engine.

***************************************************************************/

#pragma once

#ifndef __EMU_H__
#error Dont include this file directly; include emu.h instead.
#endif

#ifndef MAME_EMU_SCHEDULE_H
#define MAME_EMU_SCHEDULE_H


//**************************************************************************
//  MACROS
//**************************************************************************

#define TIMER_CALLBACK_MEMBER(name)     void name(s32 param)


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// timer callbacks look like this
typedef named_delegate<void (s32)> timer_expired_delegate;

// ======================> emu_timer

class emu_timer
{
public:
	// getters
	bool enabled() const noexcept { return m_enabled; }
	void set_owner(device_t &owner) { m_owner = &owner; }
	s32 param() const noexcept { return m_param; }

	// setters
	bool enable(bool enable = true) noexcept;
	void set_param(s32 param) noexcept { m_param = param; }

	// control
	void reset(const attotime &duration = attotime::never) noexcept { adjust(duration, m_param, m_period); }
	void adjust(attotime start_delay, s32 param = 0, const attotime &periodicity = attotime::never) noexcept;
	// A bus edge calculated within an atomic instruction can precede that
	// CPU's precharged local time while remaining ahead of scheduler time.
	// Preserve that edge; this API never moves the global timeline back.
	void adjust_absolute(attotime when, s32 param = 0) noexcept;

	// timing queries
	attotime elapsed() const noexcept;
	attotime remaining() const noexcept;
	attotime start() const noexcept { return m_start; }
	attotime expire() const noexcept { return m_expire; }
	attotime period() const noexcept { return m_period; }

private:
	// construction/destruction
	emu_timer() noexcept;
	~emu_timer();

	// allocation and re-use
	emu_timer &init(
			device_scheduler &scheduler,
			timer_expired_delegate &&callback,
			attotime start_delay,
			s32 param,
			bool temporary);

	// internal helpers
	void register_save(save_manager &manager) ATTR_COLD;
	void schedule_next_period() noexcept;
	void dump() const;

	// internal state
	device_scheduler *  m_scheduler;    // reference to the owning machine
	device_t *          m_owner = nullptr;
	u64                 m_adjustment = 0; // detects changes while a callback catches up a peer
	emu_timer *         m_next;         // next timer in order in the list
	emu_timer *         m_prev;         // previous timer in order in the list
	timer_expired_delegate m_callback;  // callback function
	s32                 m_param;        // integer parameter
	bool                m_enabled;      // is the timer enabled?
	bool                m_temporary;    // is the timer temporary?
	attotime            m_period;       // the repeat frequency of the timer
	attotime            m_start;        // time when the timer was started
	attotime            m_expire;       // time when the timer will expire
	u32                 m_index;        // needed to restore timers scheduled at the same time in correct order

	friend class device_scheduler;
	friend class fixed_allocator<emu_timer>;
	friend class simple_list<emu_timer>; // FIXME: fixed_allocator requires this
};


// ======================> device_scheduler

class device_scheduler
{
	friend class device_execute_interface;
	friend class emu_timer;

public:
	// construction/destruction
	device_scheduler(running_machine &machine);
	device_scheduler(running_machine &machine, device_t &domain);
	~device_scheduler();

	// A detached device domain owns its executor and timer list.  Only the
	// thread holding this scope may run it; the machine scheduler is untouched.
	class domain_scope
	{
	public:
		explicit domain_scope(device_scheduler &scheduler) noexcept;
		~domain_scope();
		domain_scope(domain_scope const &) = delete;
		domain_scope &operator=(domain_scope const &) = delete;
	private:
		device_scheduler *m_previous;
	};
	static device_scheduler &for_thread(device_scheduler &fallback) noexcept;
	static device_scheduler const &for_thread(device_scheduler const &fallback) noexcept;
	static u32 &side_effects_count(running_machine const &machine, u32 &fallback) noexcept;
	static u32 side_effects_count(running_machine const &machine, u32 const &fallback) noexcept;
	bool detached() const noexcept { return m_domain != nullptr; }
	void detach(device_execute_interface &exec, device_t &root);
	attotime advance_domain(device_execute_interface &exec, attotime when, bool bus_edge, bool include_equal = true);
	struct domain_target { attotime when; bool bus_edge, include_equal; };
	void advance_domain_stream(device_execute_interface &exec, domain_target target,
		std::function<bool (attotime, domain_target &)> const &next);
	bool continue_domain_grant(device_execute_interface &exec);
	void interrupt_domain(attotime when);
	void finish_domain_edge(attotime when);
	void set_domain_barrier(std::function<void (attotime)> barrier) { m_domain_barrier = std::move(barrier); }
	void domain_barrier(attotime when) { if (m_domain_barrier) m_domain_barrier(when); }

	// getters
	running_machine &machine() const noexcept { return m_machine; }
	attotime time() const noexcept;
	attotime base_time() const noexcept { return m_basetime; }
	bool in_callback() const noexcept { return m_callback_timer != nullptr; }
	emu_timer *first_timer() const noexcept { return m_timer_list; }
	device_execute_interface *currently_executing() const noexcept { return m_executing_device; }
	bool can_save() const;

	// execution
	void timeslice();
	// From a timer callback, finish bus edges before its timestamp on a peer.
	// False means a newly discovered earlier timer must run first.
	bool synchronize_before(device_execute_interface &exec, attotime when);
	// Nested catch-up from a bus participant. An earlier timer is a barrier;
	// the caller must yield and retry instead of exposing future state.
	bool synchronize_device(device_execute_interface &exec, attotime when);
	void synchronize_domain(device_execute_interface &exec, attotime when);
	void abort_timeslice() noexcept;
	void trigger(int trigid, const attotime &after = attotime::zero);
	void add_quantum(const attotime &quantum, const attotime &duration);
	void perfect_quantum(const attotime &duration);
	void suspend_resume_changed() { m_suspend_changes_pending = true; }

	// timers, specified by callback/name
	emu_timer *timer_alloc(timer_expired_delegate callback);
	[[deprecated("timer_set is deprecated; please avoid anonymous timers. Use an allocated emu_timer instead.")]]
	void timer_set(const attotime &duration, timer_expired_delegate callback, s32 param = 0);
	void synchronize(timer_expired_delegate callback = timer_expired_delegate(), s32 param = 0, device_t *owner = nullptr);

	// debugging
	void dump_timers() const;

	// for emergencies only!
	void eat_all_cycles();

private:
	device_scheduler(running_machine &machine, device_t *domain);
	static thread_local device_scheduler *s_thread_scheduler;
	device_t *m_domain;
	struct domain_stream;
	domain_stream *m_domain_stream = nullptr;
	std::function<void (attotime)> m_domain_barrier;
	attotime m_domain_interrupt = attotime::never;
	u32 m_domain_side_effects = 0;
	// callbacks
	void timed_trigger(s32 param);
	void presave();
	void postload();

	// scheduling helpers
	void compute_perfect_interleave();
	void rebuild_execute_list();
	void apply_suspend_changes();
	static void account_execution(device_execute_interface &exec, int cycles);

	// timer helpers
	template <bool CheckIndex = false> emu_timer &timer_list_insert(emu_timer &timer);
	emu_timer &timer_list_remove(emu_timer &timer);
	void execute_timers();
	void execute_timer(emu_timer &timer);

	// internal state
	running_machine &           m_machine;                  // reference to our machine
	device_execute_interface *  m_executing_device;         // pointer to currently executing device
	device_execute_interface *  m_execute_list;             // list of devices to be executed
	std::vector<device_execute_interface *> m_execute_order;
	bool m_execute_order_final = false;
	attotime                    m_basetime;                 // global basetime; everything moves forward from here

	// list of active timers
	emu_timer *                 m_timer_list;               // head of the active list
	emu_timer *                 m_inactive_timers;          // head of the inactive timer list
	fixed_allocator<emu_timer>  m_timer_allocator;          // allocator for timers

	// other internal states
	emu_timer *                 m_callback_timer;           // pointer to the current callback timer
	bool                        m_callback_timer_modified;  // true if the current callback timer was modified
	attotime                    m_callback_timer_expire_time; // the original expiration time
	bool                        m_suspend_changes_pending;  // suspend/resume changes are pending
	bool                        m_synchronized_deadline = false;

	// scheduling quanta
	class quantum_slot
	{
		friend class simple_list<quantum_slot>;

	public:
		quantum_slot *next() const { return m_next; }

		quantum_slot *          m_next;
		attoseconds_t           m_actual;                   // actual duration of the quantum
		attoseconds_t           m_requested;                // duration of the requested quantum
		attotime                m_expire;                   // absolute expiration time of this quantum
	};
	simple_list<quantum_slot>   m_quantum_list;             // list of active quanta
	fixed_allocator<quantum_slot> m_quantum_allocator;      // allocator for quanta
	attoseconds_t               m_quantum_minimum;          // duration of minimum quantum
};


#endif  // MAME_EMU_SCHEDULE_H

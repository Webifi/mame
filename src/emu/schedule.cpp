// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    schedule.cpp

    Core device execution and scheduling engine.

***************************************************************************/

#include "emu.h"
#include "debugger.h"

//**************************************************************************
//  DEBUGGING
//**************************************************************************

#define VERBOSE 0

#define LOG(...)  do { if (VERBOSE) machine().logerror(__VA_ARGS__); } while (0)
#define PRECISION



//**************************************************************************
//  CONSTANTS
//**************************************************************************

// internal trigger IDs
enum
{
	TRIGGER_INT         = -2000,
	TRIGGER_YIELDTIME   = -3000,
	TRIGGER_SUSPENDTIME = -4000
};



//**************************************************************************
//  EMU TIMER
//**************************************************************************

//-------------------------------------------------
//  emu_timer - constructor
//-------------------------------------------------

inline emu_timer::emu_timer() noexcept :
	m_scheduler(nullptr),
	m_next(nullptr),
	m_prev(nullptr),
	m_param(0),
	m_enabled(false),
	m_temporary(false),
	m_period(attotime::zero),
	m_start(attotime::zero),
	m_expire(attotime::never)
{
}


//-------------------------------------------------
//  ~emu_timer - destructor
//-------------------------------------------------

emu_timer::~emu_timer()
{
}


//-------------------------------------------------
//  init - completely initialize the state when
//  re-allocated as a non-device timer
//-------------------------------------------------

inline emu_timer &emu_timer::init(
		device_scheduler &scheduler,
		timer_expired_delegate &&callback,
		attotime start_delay,
		s32 param,
		bool temporary)
{
	// ensure the entire timer state is clean
	m_scheduler = &scheduler;
	m_owner = nullptr;
	m_adjustment = 0;
	m_next = nullptr;
	m_prev = nullptr;
	m_callback = std::move(callback);
	m_param = param;
	m_temporary = temporary;
	m_period = attotime::never;

	m_start = m_scheduler->time();
	m_expire = m_start + start_delay;
	m_enabled = !m_expire.is_never();

	// if we're not temporary, register ourselves with the save state system
	if (!m_temporary)
		register_save(scheduler.machine().save());

	// insert into the list
	m_scheduler->timer_list_insert(*this);
	if (this == m_scheduler->first_timer())
		m_scheduler->abort_timeslice();

	return *this;
}


//-------------------------------------------------
//  enable - enable/disable a timer
//-------------------------------------------------

bool emu_timer::enable(bool enable) noexcept
{
	assert(m_scheduler);

	// reschedule only if the state has changed
	const bool old = m_enabled;
	if (old != enable)
	{
		// set the enable flag
		m_enabled = enable;

		// remove the timer and insert back into the list
		m_scheduler->timer_list_remove(*this);
		m_scheduler->timer_list_insert(*this);
	}
	return old;
}


//-------------------------------------------------
//  adjust - adjust the time when this timer will
//  fire and specify a period for subsequent
//  firings
//-------------------------------------------------

void emu_timer::adjust(attotime start_delay, s32 param, const attotime &period) noexcept
{
	assert(m_scheduler);
	const attotime old_deadline = m_scheduler->first_timer()->expire();

	// if this is the callback timer, mark it modified
	++m_adjustment;
	if (m_scheduler->m_callback_timer == this)
		m_scheduler->m_callback_timer_modified = true;

	// compute the time of the next firing and insert into the list
	m_param = param;
	m_enabled = true;

	// clamp negative times to 0
	if (start_delay.seconds() < 0)
		start_delay = attotime::zero;

	// set the start and expire times
	m_start = m_scheduler->time();
	m_expire = m_start + start_delay;
	m_period = period;

	// remove and re-insert the timer in its new order
	m_scheduler->timer_list_remove(*this);
	m_scheduler->timer_list_insert(*this);

	// The executing slice is already bounded by the old first timer. Only
	// an earlier deadline needs to shorten it. Keep the list reinsertion:
	// callbacks sharing a timestamp must retain their original ordering.
	if (this == m_scheduler->first_timer() && m_expire < old_deadline)
		m_scheduler->abort_timeslice();
}


//-------------------------------------------------
//  adjust_absolute - preserve a physical bus edge
//  discovered by a CPU whose local clock leads it
//-------------------------------------------------

void emu_timer::adjust_absolute(attotime when, s32 param) noexcept
{
	assert(m_scheduler);
	if (when < m_scheduler->m_basetime)
		fatalerror("Timer edge %s precedes scheduler time %s", when.as_string(), m_scheduler->m_basetime.as_string());
	const attotime old_deadline = m_scheduler->first_timer()->expire();
	++m_adjustment;
	if (m_scheduler->m_callback_timer == this)
		m_scheduler->m_callback_timer_modified = true;
	m_param = param;
	m_enabled = true;
	m_start = m_scheduler->m_basetime;
	m_expire = when;
	m_period = attotime::never;
	m_scheduler->timer_list_remove(*this);
	m_scheduler->timer_list_insert(*this);
	if (this == m_scheduler->first_timer() && m_expire < old_deadline)
		m_scheduler->abort_timeslice();
}

//-------------------------------------------------
//  elapsed - return time since the timer was started
//-------------------------------------------------

attotime emu_timer::elapsed() const noexcept
{
	assert(m_scheduler);

	return m_scheduler->time() - m_start;
}


//-------------------------------------------------
//  remaining - return the amount of time
//  remaining until the timer expires
//-------------------------------------------------

attotime emu_timer::remaining() const noexcept
{
	assert(m_scheduler);

	const attotime curtime = m_scheduler->time();
	if (curtime >= m_expire)
		return attotime::zero;
	return m_expire - curtime;
}


//-------------------------------------------------
//  register_save - register ourself with the save
//  state system
//-------------------------------------------------

void emu_timer::register_save(save_manager &manager)
{
	// determine our instance number - timers are indexed based on the callback function name
	int index = 0;
	std::string name = m_callback.name() ? m_callback.name() : "unnamed";
	for (const emu_timer *curtimer = m_scheduler->first_timer(); curtimer; curtimer = curtimer->m_next)
	{
		if (!curtimer->m_temporary)
		{
			if (curtimer->m_callback.name() && m_callback.name() && !strcmp(curtimer->m_callback.name(), m_callback.name()))
				index++;
			else if (!curtimer->m_callback.name() && !m_callback.name())
				index++;
		}
	}
	for (const emu_timer *curtimer = m_scheduler->m_inactive_timers; curtimer; curtimer = curtimer->m_next)
	{
		assert(!curtimer->m_temporary);

		if (curtimer->m_callback.name() && m_callback.name() && !strcmp(curtimer->m_callback.name(), m_callback.name()))
			index++;
		else if (!curtimer->m_callback.name() && !m_callback.name())
			index++;
	}

	// save the bits
	manager.save_item(nullptr, "timer", name.c_str(), index, NAME(m_param));
	manager.save_item(nullptr, "timer", name.c_str(), index, NAME(m_enabled));
	manager.save_item(nullptr, "timer", name.c_str(), index, NAME(m_period));
	manager.save_item(nullptr, "timer", name.c_str(), index, NAME(m_start));
	manager.save_item(nullptr, "timer", name.c_str(), index, NAME(m_expire));
	manager.save_item(nullptr, "timer", name.c_str(), index, NAME(m_index));
}


//-------------------------------------------------
//  schedule_next_period - schedule the next
//  period
//-------------------------------------------------

inline void emu_timer::schedule_next_period() noexcept
{
	assert(m_scheduler);

	// advance by one period
	m_start = m_expire;
	m_expire += m_period;

	// remove and re-insert us
	m_scheduler->timer_list_remove(*this);
	m_scheduler->timer_list_insert(*this);
}


//-------------------------------------------------
//  dump - dump internal state to a single output
//  line in the error log
//-------------------------------------------------

void emu_timer::dump() const
{
	assert(m_scheduler);

	m_scheduler->machine().logerror("%p: en=%d temp=%d exp=%15s start=%15s per=%15s param=%d", this, m_enabled, m_temporary, m_expire.as_string(PRECISION), m_start.as_string(PRECISION), m_period.as_string(PRECISION), m_param);
	if (!m_callback.name())
		m_scheduler->machine().logerror(" cb=NULL\n");
	else
		m_scheduler->machine().logerror(" cb=%s\n", m_callback.name());
}



//**************************************************************************
//  DEVICE SCHEDULER
//**************************************************************************

//-------------------------------------------------
//  device_scheduler - constructor
//-------------------------------------------------

thread_local device_scheduler *device_scheduler::s_thread_scheduler = nullptr;

device_scheduler::domain_scope::domain_scope(device_scheduler &scheduler) noexcept
	: m_previous(std::exchange(s_thread_scheduler, &scheduler))
{
}

device_scheduler::domain_scope::~domain_scope()
{
	s_thread_scheduler = m_previous;
}

device_scheduler &device_scheduler::for_thread(device_scheduler &fallback) noexcept
{
	return s_thread_scheduler && &s_thread_scheduler->machine() == &fallback.machine() ? *s_thread_scheduler : fallback;
}

device_scheduler const &device_scheduler::for_thread(device_scheduler const &fallback) noexcept
{
	return for_thread(const_cast<device_scheduler &>(fallback));
}

u32 &device_scheduler::side_effects_count(running_machine const &machine, u32 &fallback) noexcept
{
	return s_thread_scheduler && &s_thread_scheduler->machine() == &machine && s_thread_scheduler->detached()
		? s_thread_scheduler->m_domain_side_effects : fallback;
}

u32 device_scheduler::side_effects_count(running_machine const &machine, u32 const &fallback) noexcept
{
	return side_effects_count(machine, const_cast<u32 &>(fallback));
}

device_scheduler::device_scheduler(running_machine &machine) : device_scheduler(machine, nullptr) { }
device_scheduler::device_scheduler(running_machine &machine, device_t &domain) : device_scheduler(machine, &domain) { }

device_scheduler::device_scheduler(running_machine &machine, device_t *domain) :
	m_domain(domain),
	m_machine(machine),
	m_executing_device(nullptr),
	m_execute_list(nullptr),
	m_basetime(attotime::zero),
	m_timer_list(nullptr),
	m_inactive_timers(nullptr),
	m_callback_timer(nullptr),
	m_callback_timer_modified(false),
	m_callback_timer_expire_time(attotime::zero),
	m_suspend_changes_pending(true),
	m_quantum_minimum(ATTOSECONDS_IN_NSEC(1) / 1000)
{
	// append a single never-expiring timer so there is always one in the list
	// need to subvert it because it would naturally be inserted in the inactive list
	m_timer_list = &timer_list_remove(m_timer_allocator.alloc()->init(*this, timer_expired_delegate(), attotime::never, 0, true));

	assert(m_timer_list);
	assert(!m_timer_list->m_prev);
	assert(!m_timer_list->m_next);
	assert(!m_inactive_timers);

	// register global states
	if (domain)
		machine.save().save_item(domain, "scheduler", domain->tag(), 0, NAME(m_basetime));
	else
		machine.save().save_item(NAME(m_basetime));
	machine.save().register_presave(save_prepost_delegate(FUNC(device_scheduler::presave), this));
	machine.save().register_postload(save_prepost_delegate(FUNC(device_scheduler::postload), this));
}


//-------------------------------------------------
//  device_scheduler - destructor
//-------------------------------------------------

device_scheduler::~device_scheduler()
{
	// remove all timers
	while (m_inactive_timers)
		m_timer_allocator.reclaim(timer_list_remove(*m_inactive_timers));
	while (m_timer_list)
		m_timer_allocator.reclaim(timer_list_remove(*m_timer_list));
}

void device_scheduler::detach(device_execute_interface &exec, device_t &root)
{
	auto &source = exec.scheduler();
	assert((m_domain == &root || (!m_domain && source.m_domain == &root)) && !m_executing_device);
	assert(&source != this && !source.m_executing_device);
	auto owns = [&](emu_timer const &timer) {
		for (auto *owner = timer.m_owner; owner; owner = owner->owner())
			if (owner == &root) return true;
		if (&timer == exec.m_timedint_timer || &timer == exec.m_spin_end_timer) return true;
		for (auto *pulse : exec.m_pulse_end_timers) if (&timer == pulse) return true;
		return false;
	};
	// Traversal order is retained, including timers with equal deadlines.
	for (auto *first : { source.m_timer_list, source.m_inactive_timers })
		for (auto *timer = first; timer; )
		{
			auto *next = timer->m_next;
			if (owns(*timer))
			{
				source.timer_list_remove(*timer);
				timer->m_scheduler = this;
				timer_list_insert(*timer);
			}
			timer = next;
		}
	if (m_domain) m_basetime = source.m_basetime;
	else assert(source.m_basetime == m_basetime);
	if (!source.m_domain) machine().private_domain_change(true);
	else if (!m_domain) machine().private_domain_change(false);
	exec.m_scheduler = this;
	source.m_execute_order_final = m_execute_order_final = false;
	source.rebuild_execute_list();
	rebuild_execute_list();
}

void device_scheduler::interrupt_domain(attotime when)
{
	m_domain_interrupt = std::min(m_domain_interrupt, when);
	abort_timeslice();
}

// Detached-domain contract:
//
// * One caller owns a domain at a time. domain_scope routes machine APIs on
//   that thread only, and only for the same running_machine. Nested scopes
//   restore the previous route on return or exception. Side-effect disablers
//   must be destroyed in the domain in which they were entered. A scope is
//   routing, not a mutex: the owner must fence worker access before detach,
//   reset, save/load or teardown. detach moves the executor and its timers
//   only while both schedulers are idle, preserving equal-time timer order.
//
// * advance_domain grants a monotonic target, not permission to run beyond
//   arbitrary future events. Plain grants use complete cycles (floor);
//   bus-edge preparation uses cycles whose starts precede the edge (ceiling).
//   Their CPU-local times may therefore straddle the same target differently.
//   A suspended non-eating executor advances neither cycles nor local time;
//   an eating executor accounts the grant without calling its run method.
//
// * A running grant publishes cycles_running and icount, clears cycles_stolen,
//   and hides the outer callback while the executor owns scheduler.time(). On
//   return, budget - icount - cycles_stolen is accounted exactly once. Abort
//   removes unexecuted clocks; it does not retire an instruction or bus edge.
//   Restore the caller/callback before delivering any timer or next target.
//   An exception restores routing/execution context and propagates; it does
//   not commit the failed grant or silently continue emulation.
//
// * Timers already pending before a target precede it (equal deadlines only
//   when include_equal). An earlier timer discovered during CPU preparation
//   forces reselection before the target is exposed. An equal timer discovered
//   during bus-edge preparation follows that edge via finish_domain_edge.
//   Callback rearming is tracked by adjustment count in floor synchronization;
//   edge preparation deliberately retains synchronize_before's own ordering.
//   interrupt_domain aborts the current executor and bounds a plain advance;
//   it does not discard or substitute an incoming bus edge.
//
// * advance_domain_stream has the same boundaries. continue_domain_grant may
//   retain a CPU's host stack only at an exhausted budget, after closing that
//   grant and running intervening callbacks with no executor marked active.
//   The next-target callback is synchronous and must outlive the stream. False
//   ends it; debug hooks use ordinary advance_domain instead. Saved hardware
//   state includes local/base times and timers, never this live stream/frame.
//   The owner quiesces it before save/load; postload rebuilds timer/executor
//   ordering before a new grant.

void device_scheduler::account_execution(device_execute_interface &exec, int cycles)
{
	assert(cycles >= 0);
	exec.m_totalcycles += cycles;
	exec.m_localtime += attotime(0, exec.m_attoseconds_per_cycle * cycles);
}

attotime device_scheduler::advance_domain(device_execute_interface &exec, attotime when, bool bus_edge, bool include_equal)
{
	assert(m_domain && &exec.scheduler() == this && !m_executing_device);
	assert(when >= m_basetime);
	if (m_suspend_changes_pending)
		apply_suspend_changes();
	m_domain_interrupt = attotime::never;
	// A timer already pending at this deadline precedes the incoming edge.
	// A timer discovered while preparing that edge follows the same ordering
	// as synchronize_before: only an earlier deadline forces a retry.
	while (true)
	{
		if (!bus_edge && m_domain_interrupt <= when)
		{
			m_basetime = m_domain_interrupt;
			return m_basetime;
		}
		if (m_timer_list->expire() < when || (include_equal && m_timer_list->expire() == when))
		{
			auto *next = m_timer_list;
			m_callback_timer = next;
			m_callback_timer_expire_time = next->expire();
			const bool ready = synchronize_device(exec, next->expire());
			m_callback_timer = nullptr;
			if (!ready || m_timer_list != next) continue;
			m_basetime = next->expire();
			execute_timer(*next);
			m_callback_timer = nullptr;
			continue;
		}
		if (bus_edge)
		{
			m_callback_timer = m_timer_list; // callback context, never dispatched
			m_callback_timer_expire_time = when;
			const bool ready = synchronize_before(exec, when);
			m_callback_timer = nullptr;
			if (!ready) continue;
		}
		else
		{
			if (!synchronize_device(exec, when) || (include_equal && m_timer_list->expire() == when)) continue;
		}
		if (!bus_edge && m_domain_interrupt <= when) continue;
		m_basetime = when;
		return when;
	}
}

// The detached-domain driver retains the same grant boundaries as
// advance_domain/synchronize_device/synchronize_before. A CPU may ask for
// the next grant while its ISA stack is still live; every intervening timer
// and bus callback runs after accounting the previous grant, with no CPU
// marked as executing. A fence or an empty input queue ends the stream.
struct device_scheduler::domain_stream
{
 device_scheduler &s;
 device_execute_interface &exec;
 domain_target target;
 std::function<bool (attotime, domain_target &)> const &next;
 enum class stage { start, select, synchronize, done } state=stage::start;
 enum class purpose { timer, edge, plain } why=purpose::plain;
 emu_timer *expected=nullptr, *callback=nullptr;
 attotime deadline, old_deadline;
 u64 adjustment=0;
 int budget=0;
 bool in_grant=false;

 void complete(attotime when)
 {
  s.m_basetime=when;
  state=next(when,target)?stage::start:stage::done;
 }
 void finish_sync(bool ready)
 {
  if(why!=purpose::edge && s.m_timer_list->m_expire<old_deadline)
   s.m_synchronized_deadline=true;
  state=stage::select;
  if(why==purpose::timer) {
   s.m_callback_timer=nullptr;
   if(!ready || s.m_timer_list!=expected)return;
   s.m_basetime=expected->expire();
   s.execute_timer(*expected);
   s.m_callback_timer=nullptr;
  } else if(why==purpose::edge) {
   s.m_callback_timer=nullptr;
   if(ready)complete(target.when);
  } else if(ready && !(target.include_equal && s.m_timer_list->expire()==target.when) &&
   s.m_domain_interrupt>target.when)complete(target.when);
 }
 void finish_floor()
 {
  const bool quiescent=!exec.m_attoseconds_per_cycle || (exec.m_nextsuspend&&!exec.m_nexteatcycles);
  finish_sync((quiescent || exec.m_localtime+attotime(0,exec.m_attoseconds_per_cycle)>deadline) &&
   s.m_timer_list->m_expire>=deadline);
 }
 void finish_grant()
 {
  assert(in_grant && s.m_executing_device==&exec);
  const int ran=budget-*exec.m_icountptr-exec.m_cycles_stolen;
  s.m_executing_device=nullptr;
  s.m_callback_timer=callback;
  if(why!=purpose::edge)
   s.m_callback_timer_modified |= callback && callback->m_adjustment!=adjustment;
  in_grant=false;
  s.account_execution(exec, ran);
  if(why==purpose::edge) {
   if(!ran)finish_sync(false);
  } else if(!ran || !s.m_domain_interrupt.is_never())finish_floor();
 }
 bool next_grant()
 {
  assert(!in_grant && !s.m_executing_device);
  while(state!=stage::done) {
   if(state==stage::start) {
    assert(target.when>=s.m_basetime);
    if(s.m_suspend_changes_pending)s.apply_suspend_changes();
    s.m_domain_interrupt=attotime::never;
    state=stage::select;
   }
   if(state==stage::select) {
    if(!target.bus_edge && s.m_domain_interrupt<=target.when) {
     complete(s.m_domain_interrupt);continue;
    }
    if(s.m_timer_list->expire()<target.when || (target.include_equal&&s.m_timer_list->expire()==target.when)) {
     why=purpose::timer;expected=s.m_timer_list;deadline=expected->expire();
     s.m_callback_timer=expected;s.m_callback_timer_expire_time=deadline;
    } else {
     why=target.bus_edge?purpose::edge:purpose::plain;deadline=target.when;
     if(target.bus_edge) {
      s.m_callback_timer=s.m_timer_list;
      s.m_callback_timer_expire_time=deadline;
     }
    }
    old_deadline=s.m_timer_list->m_expire;
    state=stage::synchronize;
   }
   if(why==purpose::edge) {
    if(exec.m_localtime>=deadline) {finish_sync(s.m_timer_list->m_expire>=deadline);continue;}
    if(s.m_timer_list->m_expire<deadline) {finish_sync(false);continue;}
    if(!exec.m_attoseconds_per_cycle || (exec.m_nextsuspend&&!exec.m_nexteatcycles)) {
     finish_sync(s.m_timer_list->m_expire>=deadline);continue;
    }
    const attoseconds_t delta=(deadline-exec.m_localtime).as_attoseconds();
    budget=(delta+exec.m_attoseconds_per_cycle-1)/exec.m_attoseconds_per_cycle;
   } else {
    if(exec.m_localtime+attotime(0,exec.m_attoseconds_per_cycle)>deadline) {finish_floor();continue;}
    const attotime limit=std::min(deadline,s.m_timer_list->m_expire);
    if(limit<=exec.m_localtime || !exec.m_attoseconds_per_cycle || (exec.m_nextsuspend&&!exec.m_nexteatcycles)) {
     finish_floor();continue;
    }
    const attoseconds_t delta=(limit-exec.m_localtime).as_attoseconds();
    budget=delta/exec.m_attoseconds_per_cycle;
    if(!budget) {finish_floor();continue;}
   }
   exec.m_cycles_running=budget;
   if(exec.m_nextsuspend) {
    s.account_execution(exec, budget);
    if(why!=purpose::edge&&!s.m_domain_interrupt.is_never())finish_floor();
    continue;
   }
   exec.m_cycles_stolen=0;
   s.m_executing_device=&exec;
   callback=std::exchange(s.m_callback_timer,nullptr);
   adjustment=callback?callback->m_adjustment:0;
   *exec.m_icountptr=budget;
   in_grant=true;
   return true;
  }
  return false;
 }
};

void device_scheduler::advance_domain_stream(device_execute_interface &exec, domain_target target,
 std::function<bool (attotime, domain_target &)> const &next)
{
 assert(m_domain && &exec.scheduler()==this && !m_executing_device && !m_domain_stream);
 // Debugger hooks retain the ordinary driver and its per-grant start/stop.
 if(machine().debug_flags&(DEBUG_FLAG_ENABLED|DEBUG_FLAG_CALL_HOOK)) {
  do {target.when=advance_domain(exec,target.when,target.bus_edge,target.include_equal);}
  while(next(target.when,target));
  return;
 }
 domain_stream stream{*this,exec,target,next};
 m_domain_stream=&stream;
 try {
  while(stream.next_grant()) {
   auto profile=g_profiler.start(exec.m_profiler);
   exec.run();
   if(stream.in_grant)stream.finish_grant();
  }
 } catch(...) {
  if(stream.in_grant) {
   m_executing_device=nullptr;m_callback_timer=stream.callback;
   if(stream.why!=domain_stream::purpose::edge)
    m_callback_timer_modified |= stream.callback && stream.callback->m_adjustment!=stream.adjustment;
  }
  m_domain_stream=nullptr;
  throw;
 }
 m_domain_stream=nullptr;
}

bool device_scheduler::continue_domain_grant(device_execute_interface &exec)
{
 if(!m_domain_stream || &m_domain_stream->exec!=&exec || !m_domain_stream->in_grant ||
  *exec.m_icountptr>0)return false;
 m_domain_stream->finish_grant();
 return m_domain_stream->next_grant();
}


void device_scheduler::finish_domain_edge(attotime when)
{
	assert(m_domain && m_basetime == when);
	execute_timers();
}


//-------------------------------------------------
//  time - return the current time
//-------------------------------------------------

attotime device_scheduler::time() const noexcept
{
	// if we're currently in a callback, use the timer's expiration time as a base
	if (m_callback_timer != nullptr)
		return m_callback_timer_expire_time;

	// if we're executing as a particular CPU, use its local time as a base
	// otherwise, return the global base time
	return (m_executing_device != nullptr) ? m_executing_device->local_time() : m_basetime;
}


//-------------------------------------------------
//  can_save - return true if it's safe to save
//  (i.e., no temporary timers outstanding)
//-------------------------------------------------

bool device_scheduler::can_save() const
{
	if (!m_domain)
		for (auto &exec : execute_interface_enumerator(machine().root_device()))
			if (&exec.scheduler() != this)
			{
				exec.scheduler().domain_barrier(m_basetime);
				if (exec.scheduler().base_time() > m_basetime) return false;
				if (!exec.scheduler().can_save()) return false;
			}
	// if any live temporary timers exit, fail
	for (emu_timer *timer = m_timer_list; timer; timer = timer->m_next)
	{
		if (timer->m_temporary && !timer->expire().is_never())
		{
			machine().logerror("Failed save state attempt due to anonymous timers:\n");
			dump_timers();
			return false;
		}
	}

	// otherwise, we're good
	return true;
}


//-------------------------------------------------
//  apply_suspend_changes - applies suspend/resume
//  changes to all device_execute_interfaces
//-------------------------------------------------

inline void device_scheduler::apply_suspend_changes()
{
	bool order_changed = false;
	u32 reason_changed = 0;
	m_suspend_changes_pending = false;
	for (device_execute_interface *exec = m_execute_list; exec != nullptr; exec = exec->m_nextexec)
	{
		// Ordering depends on active versus suspended, not on which reason
		// holds a suspended device. A one-slice yield still needs the next
		// boundary to clear it; ordinary applied changes need no second pass.
		order_changed |= (exec->m_suspend == 0) != (exec->m_nextsuspend == 0);
		reason_changed |= exec->m_suspend ^ exec->m_nextsuspend;
		exec->m_suspend = exec->m_nextsuspend;
		exec->m_nextsuspend &= ~SUSPEND_REASON_TIMESLICE;
		exec->m_eatcycles = exec->m_nexteatcycles;
		m_suspend_changes_pending |= exec->m_suspend != exec->m_nextsuspend;
	}

	// Preserve the existing repeated-TIMESLICE behavior: a reasserted
	// identical mask did not request an additional apply boundary.
	m_suspend_changes_pending &= reason_changed != 0;
	if (order_changed)
		rebuild_execute_list();
}


//-------------------------------------------------
//  timeslice - execute all devices for a single
//  timeslice
//-------------------------------------------------

void device_scheduler::timeslice()
{
	bool call_debugger = ((machine().debug_flags & DEBUG_FLAG_ENABLED) != 0);

	// build the execution list if we don't have one yet
	if (UNEXPECTED(m_execute_list == nullptr))
		rebuild_execute_list();

	// if the current quantum has expired, find a new one
	while (m_basetime >= m_quantum_list.first()->m_expire)
		m_quantum_allocator.reclaim(m_quantum_list.detach_head());

	// loop until we hit the next timer
	while (m_basetime < m_timer_list->m_expire)
	{
		// by default, assume our target is the end of the next quantum
		attotime target(m_basetime + attotime(0, m_quantum_list.first()->m_actual));

		// however, if the next timer is going to fire before then, override
		if (m_timer_list->m_expire < target)
			target = m_timer_list->m_expire;

		LOG("------------------\n");
		LOG("cpu_timeslice: target = %s\n", target.as_string(PRECISION));

		// do we have pending suspension changes?
		if (m_suspend_changes_pending)
			apply_suspend_changes();

		// loop over all CPUs
		for (device_execute_interface *exec = m_execute_list; exec != nullptr; exec = exec->m_nextexec)
		{
			if (exec->m_deferred_execution) continue;
			// only process if this CPU is executing or truly halted (not yielding)
			// and if our target is later than the CPU's current time (coarse check)
			if (EXPECTED((exec->m_suspend == 0 || exec->m_eatcycles) && target.seconds() >= exec->m_localtime.seconds()))
			{
				// compute how many attoseconds to execute this CPU
				attoseconds_t delta = target.attoseconds() - exec->m_localtime.attoseconds();
				if (delta < 0 && target.seconds() > exec->m_localtime.seconds())
					delta += ATTOSECONDS_PER_SECOND;
				assert(delta == (target - exec->m_localtime).as_attoseconds());

				if (exec->m_attoseconds_per_cycle == 0)
				{
					exec->m_localtime = target;
				}
				// if we have enough for at least 1 cycle, do the math
				else if (delta >= exec->m_attoseconds_per_cycle || (exec->m_cycle_start_execution && !exec->m_suspend && delta > 0))
				{
					// compute how many cycles we want to execute
					const bool cycle_start = exec->m_cycle_start_execution && !exec->m_suspend;
					int ran = cycle_start
						? delta / exec->m_attoseconds_per_cycle + (delta % exec->m_attoseconds_per_cycle != 0)
						: divu_64x32(u64(delta) >> exec->m_divshift, exec->m_divisor);
					exec->m_cycles_running = ran;
					LOG("  cpu '%s': %d (%d cycles)\n", exec->device().tag(), delta, exec->m_cycles_running);

					// if we're not suspended, actually execute
					if (exec->m_suspend == 0)
					{
						auto profile = g_profiler.start(exec->m_profiler);

						// note that this global variable cycles_stolen can be modified
						// via the call to cpu_execute
						exec->m_cycles_stolen = 0;
						m_executing_device = exec;
						*exec->m_icountptr = exec->m_cycles_running;
						if (!call_debugger)
							exec->run();
						else
						{
							exec->debugger_start_cpu_hook(target);
							exec->run();
							exec->debugger_stop_cpu_hook();
						}

						// adjust for any cycles we took back
						assert(ran >= *exec->m_icountptr);
						ran -= *exec->m_icountptr;
						assert(ran >= exec->m_cycles_stolen);
						ran -= exec->m_cycles_stolen;
					}

					// account for these cycles
					exec->m_totalcycles += ran;

					// update the local time for this CPU
					attotime deltatime;
					if (EXPECTED(ran < exec->m_cycles_per_second))
						deltatime = attotime(0, exec->m_attoseconds_per_cycle * ran);
					else
						deltatime = exec->cycles_to_attotime(ran);
					assert(deltatime >= attotime::zero);
					exec->m_localtime += deltatime;
					LOG("         %d ran, %d total, time = %s\n", ran, s32(exec->m_totalcycles), exec->m_localtime.as_string(PRECISION));

					// if the new local CPU time is less than our target, move the target up, but not before the base
					if (exec->m_localtime < target)
					{
						target = std::max(exec->m_localtime, m_basetime);
						LOG("         (new target)\n");
					}
					// A cycle-start core may lead target by one clock. An edge
					// discovered inside that clock must still run before the
					// next device or the global timeline crosses its deadline.
					if (cycle_start)
						target = std::min(target, std::max(m_basetime, m_timer_list->expire()));
				}
			}
			if (m_synchronized_deadline)
			{
				target = std::min(target, std::max(m_basetime, m_timer_list->m_expire));
				m_synchronized_deadline = false;
			}
		}
		m_executing_device = nullptr;

		// update the base time
		m_basetime = target;
	}

	// execute timers
	execute_timers();
}


// Complete the peer's cycle whose start precedes the callback. The normal
// timeslice budget rounds down, leaving that cycle unexecuted when the two
// devices' clocks differ. A shared-register read must not overlook its edge.
// Operations discovered here still publish through their timestamped timers;
// return to execute_timers before completing the caller if an earlier one appears.
bool device_scheduler::synchronize_before(device_execute_interface &exec, attotime when)
{
	assert(!m_executing_device && m_callback_timer);
	while (exec.m_localtime < when)
	{
		if (m_timer_list->m_expire < when) return false;
		if (!exec.m_attoseconds_per_cycle) break;
		if (exec.m_nextsuspend && !exec.m_nexteatcycles) break;
		const attoseconds_t delta = (when - exec.m_localtime).as_attoseconds();
		int ran = exec.m_cycles_running = (delta + exec.m_attoseconds_per_cycle - 1) / exec.m_attoseconds_per_cycle;
		if (!exec.m_nextsuspend)
		{
			exec.m_cycles_stolen = 0;
			m_executing_device = &exec;
			emu_timer *const callback = std::exchange(m_callback_timer, nullptr);
			*exec.m_icountptr = ran;
			try
			{
				auto profile = g_profiler.start(exec.m_profiler);
				if (machine().debug_flags & DEBUG_FLAG_ENABLED) exec.debugger_start_cpu_hook(when);
				exec.run();
				if (machine().debug_flags & DEBUG_FLAG_ENABLED) exec.debugger_stop_cpu_hook();
			}
			catch (...)
			{
				m_executing_device = nullptr;
				m_callback_timer = callback;
				throw;
			}
			ran -= *exec.m_icountptr + exec.m_cycles_stolen;
			m_executing_device = nullptr;
			m_callback_timer = callback;
		}
		account_execution(exec, ran);
		if (!ran) return false;
	}
	return m_timer_list->m_expire >= when;
}


bool device_scheduler::synchronize_device(device_execute_interface &exec, attotime when)
{
	assert(m_executing_device != &exec);
	auto *const caller = m_executing_device;
	const attotime old_deadline = m_timer_list->m_expire;
	while (exec.m_localtime + attotime(0, exec.m_attoseconds_per_cycle) <= when)
	{
		const attotime limit = std::min(when, m_timer_list->m_expire);
		if (limit <= exec.m_localtime) break;
		if (!exec.m_attoseconds_per_cycle) break;
		if (exec.m_nextsuspend && !exec.m_nexteatcycles) break;
		const attoseconds_t delta = (limit - exec.m_localtime).as_attoseconds();
		int ran = exec.m_cycles_running = delta / exec.m_attoseconds_per_cycle;
		if (!ran) break;
		if (!exec.m_nextsuspend)
		{
			exec.m_cycles_stolen = 0;
			m_executing_device = &exec;
			emu_timer *const callback = std::exchange(m_callback_timer, nullptr);
			const u64 adjustment = callback ? callback->m_adjustment : 0;
			*exec.m_icountptr = ran;
			try
			{
				auto profile = g_profiler.start(exec.m_profiler);
				if (machine().debug_flags & DEBUG_FLAG_ENABLED) exec.debugger_start_cpu_hook(when);
				exec.run();
				if (machine().debug_flags & DEBUG_FLAG_ENABLED) exec.debugger_stop_cpu_hook();
			}
			catch (...)
			{
				m_executing_device = caller;
				m_callback_timer = callback;
				m_callback_timer_modified |= callback && callback->m_adjustment != adjustment;
				throw;
			}
			ran -= *exec.m_icountptr + exec.m_cycles_stolen;
			m_executing_device = caller;
			m_callback_timer = callback;
			m_callback_timer_modified |= callback && callback->m_adjustment != adjustment;
		}
		account_execution(exec, ran);
		if (!ran) break;
		if (m_domain && !m_domain_interrupt.is_never()) break;
	}
	if (m_timer_list->m_expire < old_deadline) m_synchronized_deadline = true;
	const bool quiescent = !exec.m_attoseconds_per_cycle || (exec.m_nextsuspend && !exec.m_nexteatcycles);
	return (quiescent || exec.m_localtime + attotime(0, exec.m_attoseconds_per_cycle) > when) && m_timer_list->m_expire >= when;
}

// Materialize an isolated clock domain from a synchronous bus callback. Its
// private timers may still be in the past of the caller: execute those at their
// own timestamps, preserving the caller and the machine's global base time.
void device_scheduler::synchronize_domain(device_execute_interface &exec, attotime when)
{
	if (&exec.scheduler() != this)
	{
		exec.scheduler().domain_barrier(when);
		return;
	}
	if (m_executing_device == &exec) return;
	auto *const caller = m_executing_device;
	auto *const callback = m_callback_timer;
	const bool modified = m_callback_timer_modified;
	const u64 adjustment = callback ? callback->m_adjustment : 0;
	const attotime callback_time = m_callback_timer_expire_time;
	auto restore = [&]() {
		m_executing_device = caller;
		m_callback_timer = callback;
		m_callback_timer_modified = modified || (callback && callback->m_adjustment != adjustment);
		m_callback_timer_expire_time = callback_time;
	};
	try
	{
		while (true)
		{
			emu_timer *edge = nullptr;
			for (auto *timer = m_timer_list; timer && timer->m_expire <= when; timer = timer->m_next)
			{
				// The caller is already dispatching at this timestamp. Callbacks
				// at the same time must follow it, even if it rearms itself.
				if (callback && timer->m_expire >= callback_time) break;
				if (timer != callback && timer->m_owner && timer->m_owner->execution_barrier() == &exec)
				{ edge = timer; break; }
			}
			const attotime limit = edge ? edge->m_expire : when;
			if (exec.m_attoseconds_per_cycle && !(exec.m_nextsuspend && !exec.m_nexteatcycles) &&
				exec.m_localtime + attotime(0, exec.m_attoseconds_per_cycle) <= limit)
			{
				const auto delta = (limit - exec.m_localtime).as_attoseconds();
				int ran = exec.m_cycles_running = delta / exec.m_attoseconds_per_cycle;
				if (!exec.m_nextsuspend)
				{
					m_executing_device = &exec; m_callback_timer = nullptr;
					exec.m_cycles_stolen = 0; *exec.m_icountptr = ran;
					auto profile = g_profiler.start(exec.m_profiler);
					if (machine().debug_flags & DEBUG_FLAG_ENABLED) exec.debugger_start_cpu_hook(limit);
					exec.run();
					if (machine().debug_flags & DEBUG_FLAG_ENABLED) exec.debugger_stop_cpu_hook();
					ran -= *exec.m_icountptr + exec.m_cycles_stolen;
				}
				account_execution(exec, ran);
				if (!ran) break;
				continue; // reselect: execution may have armed an earlier edge
			}
			if (!edge) break;
			m_executing_device = nullptr;
			execute_timer(*edge);
		}
	}
	catch (...) { restore(); throw; }
	restore();
	if (caller && m_timer_list->m_expire < when)
	{
		m_synchronized_deadline = true;
		caller->abort_timeslice();
	}
}


//-------------------------------------------------
//  abort_timeslice - abort execution for the
//  current timeslice
//-------------------------------------------------

void device_scheduler::abort_timeslice() noexcept
{
	if (m_executing_device != nullptr)
		m_executing_device->abort_timeslice();
}


//-------------------------------------------------
//  trigger - generate a global trigger
//-------------------------------------------------

void device_scheduler::trigger(int trigid, const attotime &after)
{
	// ensure we have a list of executing devices
	if (m_execute_list == nullptr)
		rebuild_execute_list();

	if (after != attotime::zero)
	{
		// if we have a non-zero time, schedule a timer
		timer_set(after, timer_expired_delegate(FUNC(device_scheduler::timed_trigger), this), trigid);
	}
	else
	{
		// send the trigger to everyone who cares
		for (device_execute_interface *exec = m_execute_list; exec; exec = exec->m_nextexec)
			exec->trigger(trigid);
	}
}


//-------------------------------------------------
//  add_quantum - add a scheduling quantum;
//  the smallest active one is the one that is in use
//-------------------------------------------------

void device_scheduler::add_quantum(const attotime &quantum, const attotime &duration)
{
	assert(quantum.seconds() == 0);

	attotime curtime = time();
	attotime expire = curtime + duration;
	const attoseconds_t quantum_attos = quantum.attoseconds();

	// figure out where to insert ourselves, expiring any quanta that are out-of-date
	quantum_slot *insert_after = nullptr;
	quantum_slot *next;
	for (quantum_slot *quant = m_quantum_list.first(); quant != nullptr; quant = next)
	{
		// if this quantum is expired, nuke it
		next = quant->next();
		if (curtime >= quant->m_expire)
			m_quantum_allocator.reclaim(m_quantum_list.detach(*quant));

		// if this quantum is shorter than us, we need to be inserted afterwards
		else if (quant->m_requested <= quantum_attos)
			insert_after = quant;
	}

	// if we found an exact match, just take the maximum expiry time
	if (insert_after != nullptr && insert_after->m_requested == quantum_attos)
		insert_after->m_expire = std::max(insert_after->m_expire, expire);

	// otherwise, allocate a new quantum and insert it after the one we picked
	else
	{
		quantum_slot &quant = *m_quantum_allocator.alloc();
		quant.m_requested = quantum_attos;
		quant.m_actual = std::max(quantum_attos, m_quantum_minimum);
		quant.m_expire = expire;
		m_quantum_list.insert_after(quant, insert_after);
	}
}


//-------------------------------------------------
//  perfect_quantum - add a (temporary) minimum
//  scheduling quantum to boost the interleave
//-------------------------------------------------

void device_scheduler::perfect_quantum(const attotime &duration)
{
	add_quantum(attotime::zero, duration);
}


//-------------------------------------------------
//  timer_alloc - allocate a global non-device
//  timer and return a pointer
//-------------------------------------------------

emu_timer *device_scheduler::timer_alloc(timer_expired_delegate callback)
{
	return &m_timer_allocator.alloc()->init(*this, std::move(callback), attotime::never, 0, false);
}


//-------------------------------------------------
//  timer_set - allocate an anonymous non-device
//  timer and set it to go off after the given
//  amount of time
//-------------------------------------------------

void device_scheduler::timer_set(const attotime &duration, timer_expired_delegate callback, s32 param)
{
	[[maybe_unused]] emu_timer &timer = m_timer_allocator.alloc()->init(
			*this,
			std::move(callback),
			duration,
			param,
			true);
	assert(!timer.m_expire.is_never()); // this is not handled
}


//-------------------------------------------------
//  synchronize - allocate an anonymous non-device
//  timer and set it to go off as soon as possible
//-------------------------------------------------

void device_scheduler::synchronize(timer_expired_delegate callback, s32 param, device_t *owner)
{
	auto &timer = m_timer_allocator.alloc()->init(
			*this,
			std::move(callback),
			attotime::zero,
			param,
			true);
	if (owner) timer.set_owner(*owner);
}


//-------------------------------------------------
//  eat_all_cycles - eat a ton of cycles on all
//  CPUs to force a quick exit
//-------------------------------------------------

void device_scheduler::eat_all_cycles()
{
	for (device_execute_interface *exec = m_execute_list; exec != nullptr; exec = exec->m_nextexec)
		exec->eat_cycles(1000000000);
}


//-------------------------------------------------
//  timed_trigger - generate a trigger after a
//  given amount of time
//-------------------------------------------------

void device_scheduler::timed_trigger(s32 param)
{
	trigger(param);
}


//-------------------------------------------------
//  presave - before creating a save state
//-------------------------------------------------

void device_scheduler::presave()
{
	if (m_domain_barrier) m_domain_barrier(machine().time());
	// report the timer state after a log
	LOG("Prior to saving state:\n");
	u32 index = 0;
	for (emu_timer *timer = m_timer_list; timer; timer = timer->m_next)
		timer->m_index = index++;
#if VERBOSE
	dump_timers();
#endif
}


//-------------------------------------------------
//  postload - after loading a save state
//-------------------------------------------------

void device_scheduler::postload()
{
	// remove all timers and make a private list of permanent ones
	emu_timer *private_list = nullptr;
	while (m_inactive_timers)
	{
		emu_timer &timer = *m_inactive_timers;
		assert(!timer.m_temporary);

		timer_list_remove(timer).m_next = private_list;
		private_list = &timer;
	}
	while (m_timer_list->m_next)
	{
		emu_timer &timer = *m_timer_list;

		if (timer.m_temporary)
		{
			assert(!timer.expire().is_never());

			// temporary timers go away entirely (except our special never-expiring one)
			timer.m_callback.reset();
			m_timer_allocator.reclaim(timer_list_remove(timer));
		}
		else
		{
			// permanent ones get added to our private list
			timer_list_remove(timer).m_next = private_list;
			private_list = &timer;
		}
	}

	// special dummy timer
	assert(!m_timer_list->m_enabled);
	assert(m_timer_list->m_temporary);
	assert(m_timer_list->m_expire.is_never());

	// now re-insert them; this effectively re-sorts them by time
	while (private_list)
	{
		emu_timer &timer = *private_list;
		private_list = timer.m_next;
		timer_list_insert<true>(timer);
	}

	m_suspend_changes_pending = true;
	rebuild_execute_list();

	// report the timer state after a log
	LOG("After resetting/reordering timers:\n");
#if VERBOSE
	dump_timers();
#endif
}


//-------------------------------------------------
//  compute_perfect_interleave - compute the
//  "perfect" interleave interval
//-------------------------------------------------

void device_scheduler::compute_perfect_interleave()
{
	// ensure we have a list of executing devices
	if (m_execute_list == nullptr)
		rebuild_execute_list();

	// start with the first one
	device_execute_interface *first = m_execute_list;
	if (first != nullptr)
	{
		// start with a huge time factor and find the 2nd smallest cycle time
		attoseconds_t smallest = first->minimum_quantum();
		attoseconds_t perfect = ATTOSECONDS_PER_SECOND - 1;
		for (device_execute_interface *exec = first->m_nextexec; exec != nullptr; exec = exec->m_nextexec)
		{
			// find the 2nd smallest cycle interval
			attoseconds_t curquantum = exec->minimum_quantum();
			if (curquantum < smallest)
			{
				perfect = smallest;
				smallest = curquantum;
			}
			else if (curquantum < perfect)
				perfect = curquantum;
		}

		// if this is a new minimum quantum, apply it
		if (m_quantum_minimum != perfect)
		{
			// adjust all the actuals; this doesn't affect the current
			m_quantum_minimum = perfect;
			for (quantum_slot &quant : m_quantum_list)
				quant.m_actual = std::max(quant.m_requested, m_quantum_minimum);
		}
	}
}


//-------------------------------------------------
//  rebuild_execute_list - rebuild the list of
//  executing CPUs, moving suspended CPUs to the
//  end
//-------------------------------------------------

void device_scheduler::rebuild_execute_list()
{
	// Device topology is fixed once execution begins. Suspension changes
	// reorder this same list; they do not require walking every bus, screen,
	// memory device and ROM dependency again at each PH0 rendezvous.
	if (!m_execute_order_final)
	{
		m_execute_order.clear();
		for (auto &exec : execute_interface_enumerator(machine().root_device()))
			if (exec.m_scheduler == this || (!m_domain && !exec.m_scheduler)) m_execute_order.push_back(&exec);
		m_execute_order_final = machine().phase() == machine_phase::RUNNING;
	}
	// if we haven't yet set a scheduling quantum, do it now
	if (m_quantum_list.empty())
	{
		// set the core scheduling quantum, ensuring it's no longer than 60Hz
		attotime min_quantum = machine().config().maximum_quantum(attotime::from_hz(60));

		// if the configuration specifies a device to make perfect, pick that as the minimum
		device_execute_interface *const exec(machine().config().perfect_quantum_device());
		if (exec)
			min_quantum = (std::min)(attotime(0, exec->minimum_quantum()), min_quantum);

		// inform the timer system of our decision
		add_quantum(min_quantum, attotime::never);
	}

	// start with an empty list
	device_execute_interface **active_tailptr = &m_execute_list;
	*active_tailptr = nullptr;

	// also make an empty list of suspended devices
	device_execute_interface *suspend_list = nullptr;
	device_execute_interface **suspend_tailptr = &suspend_list;

	// iterate over all devices
	for (device_execute_interface *device : m_execute_order)
	{
		auto &exec = *device;
		// append to the appropriate list
		exec.m_nextexec = nullptr;
		if (exec.m_suspend == 0)
		{
			*active_tailptr = &exec;
			active_tailptr = &exec.m_nextexec;
		}
		else
		{
			*suspend_tailptr = &exec;
			suspend_tailptr = &exec.m_nextexec;
		}
	}

	// append the suspend list to the end of the active list
	*active_tailptr = suspend_list;
}


//-------------------------------------------------
//  timer_list_insert - insert a new timer into
//  the list at the appropriate location
//-------------------------------------------------

template <bool CheckIndex>
inline emu_timer &device_scheduler::timer_list_insert(emu_timer &timer)
{
	// disabled timers never expire
	if (!timer.m_expire.is_never() && timer.m_enabled)
	{
		// loop over the timer list
		emu_timer *prevtimer = nullptr;
		for (emu_timer *curtimer = m_timer_list; curtimer; prevtimer = curtimer, curtimer = curtimer->m_next)
		{
			// if the current list entry expires after us, we should be inserted before it
			bool const here =
					(curtimer->m_expire > timer.m_expire) ||
					(CheckIndex && !(curtimer->m_expire < timer.m_expire) && (curtimer->m_index > timer.m_index));
			if (here)
			{
				// link the new guy in before the current list entry
				timer.m_prev = prevtimer;
				timer.m_next = curtimer;

				if (prevtimer)
					prevtimer->m_next = &timer;
				else
					m_timer_list = &timer;

				curtimer->m_prev = &timer;
				return timer;
			}
		}

		// need to insert after the last one
		if (prevtimer)
			prevtimer->m_next = &timer;
		else
			m_timer_list = &timer;

		timer.m_prev = prevtimer;
		timer.m_next = nullptr;
	}
	else
	{
		// keep inactive timers in a separate list
		if (m_inactive_timers)
			m_inactive_timers->m_prev = &timer;

		timer.m_next = m_inactive_timers;
		timer.m_prev = nullptr;

		m_inactive_timers = &timer;
	}
	return timer;
}


//-------------------------------------------------
//  timer_list_remove - remove a timer from the
//  linked list
//-------------------------------------------------

inline emu_timer &device_scheduler::timer_list_remove(emu_timer &timer)
{
	// remove it from the list
	if (timer.m_prev)
	{
		timer.m_prev->m_next = timer.m_next;
	}
	else if (&timer == m_timer_list)
	{
		m_timer_list = timer.m_next;
	}
	else
	{
		assert(&timer == m_inactive_timers);
		m_inactive_timers = timer.m_next;
	}

	if (timer.m_next)
		timer.m_next->m_prev = timer.m_prev;

	return timer;
}


//-------------------------------------------------
//  execute_timers - execute timers that are due
//-------------------------------------------------

inline void device_scheduler::execute_timers()
{
	LOG("execute_timers: new=%s head->expire=%s\n", m_basetime.as_string(PRECISION), m_timer_list->m_expire.as_string(PRECISION));

	// now process any timers that are overdue
	while (m_timer_list->m_expire <= m_basetime)
	{
		// Advancing a deferred CPU may discover an earlier peripheral edge.
		// Dispatch that edge first, then retry this timer. No global quantum
		// and no guessed extra bus clocks are required for the rendezvous.
		emu_timer *const next = m_timer_list;
		if (auto *exec = next->m_owner ? next->m_owner->execution_barrier() : nullptr;
			exec && exec->deferred_execution() && exec->m_localtime < next->m_expire)
		{
			m_callback_timer = next;
			m_callback_timer_expire_time = next->m_expire;
			const bool ready = synchronize_device(*exec, next->m_expire);
			m_callback_timer = nullptr;
			if (!ready || m_timer_list != next) continue;
		}
		// CPU execution can cancel or postpone the callback we were preparing.
		// Reselect the head and dispatch only if it is still due.
		if (m_timer_list->m_expire <= m_basetime) execute_timer(*m_timer_list);
	}

	// clear the callback timer global
	m_callback_timer = nullptr;
}

void device_scheduler::execute_timer(emu_timer &timer)
{
		// if this is a one-shot timer, disable it now
		bool was_enabled = timer.m_enabled;
		if (timer.m_period.is_zero() || timer.m_period.is_never())
			timer.m_enabled = false;

		// set the global state of which callback we're in
		m_callback_timer_modified = false;
		m_callback_timer = &timer;
		m_callback_timer_expire_time = timer.m_expire;

		// call the callback
		if (was_enabled)
		{
			auto profile = g_profiler.start(PROFILER_TIMER_CALLBACK);

			if (!timer.m_callback.isnull())
			{
				LOG("execute_timers: timer callback %s\n", timer.m_callback.name());
				timer.m_callback(timer.m_param);
			}
		}

		// reset or remove the timer, but only if it wasn't modified during the callback
		if (!m_callback_timer_modified)
		{
			if (!timer.m_temporary)
			{
				// if the timer is not temporary, reschedule it
				timer.schedule_next_period();
			}
			else
			{
				// otherwise, remove it now
				timer.m_callback.reset();
				m_timer_allocator.reclaim(timer_list_remove(timer));
			}
		}
}


//-------------------------------------------------
//  dump_timers - dump the current timer state
//-------------------------------------------------

void device_scheduler::dump_timers() const
{
	machine().logerror("=============================================\n");
	machine().logerror("Timer Dump: Time = %15s\n", time().as_string(PRECISION));
	for (emu_timer *timer = m_timer_list; timer; timer = timer->m_next)
		timer->dump();
	for (emu_timer *timer = m_inactive_timers; timer; timer = timer->m_next)
		timer->dump();
	machine().logerror("=============================================\n");
}

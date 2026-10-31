// license:BSD-3-Clause
#pragma once
#include "twgs_timing.h"
#include "twgs_ready.h"
#include "twgs_slot_clock.h"
#include "twgs_idle_hit.h"

namespace twgs_decoded
{
// Convert the elapsed-time remainder to local CPU clocks. Most transfers
// leave less than one clock: their quotient is known without a divide.
// A timestamped bus returns negative credit immediately (floor division);
// the card-absent path retains it until the accumulated charge is positive.
inline int64_t local_cycle_charge(int64_t charge, uint64_t cycle, bool timestamped) noexcept
{
	if (charge >= 0)
		return uint64_t(charge) < cycle ? 0 : charge / int64_t(cycle);
	if (!timestamped)
		return 0;
	return charge >= -int64_t(cycle) ? -1 : -1 - int64_t(uint64_t(-(charge + 1)) / cycle);
}

// Resumable version of the same TWGS edge stream. An owner stops at a
// scheduler deadline, then continues this transaction without supplying a
// new CPU address or completing a provisional cache fill.
template <uint64_t Scale> struct slot_timing : timing_scaled<Scale>
{
	using base = timing_scaled<Scale>;
	using clock = slot_clock<Scale>;
	using base::CLK;
	using base::MEGA;
	using base::LINE;
	using base::now;
	using base::early;
	using base::late;
	using base::gs_event;
	using base::gs_start;
	using base::x4;
	using base::next_gs_level;
	using base::slot_start;
	using base::slot_end;
	using base::c;
	slot_ready ready;
	uint64_t phase_event = 0;
	transaction motherboard{}, falling_bus{};
	bool motherboard_valid = false, motherboard_mega = false, retry = false;
	bool falling_valid = false, pending = false;

	void start(uint64_t t, unsigned late_phase = Scale) noexcept
	{
		base::start(t, late_phase);
		phase_event = (t / CLK + 1) * CLK;
	}

	[[gnu::always_inline]] void begin(transaction const &w) noexcept
	{
		c.external_advance = true;
		c.template setword_inline<true>(w);
		c.completed = false;
		base::had_bus = false;
		pending = true;
	}

	uint64_t next_event() const noexcept { return std::min(phase_event, std::min(gs_event, std::min(early, late))); }

	bool phase_read(uint64_t time) const noexcept
	{
		if (!motherboard_valid || !motherboard_mega || !motherboard.read())
			return false;
		const uint64_t start = slot_end - (slot_end % LINE ? MEGA : 16 * CLK);
		return time >= start + 2 * CLK && time < slot_end;
	}

	// Candidate pins at a pending phase, including a tied GS edge. Data can
	// still be supplied by finish_bus; the neutral-read certificate depends
	// only on the address and direction. A non-selected phase is the idle bus.
	transaction phase_candidate(uint64_t time) const noexcept
	{
		if (gs_event == time)
		{
			if (!next_gs_level)
				return motherboard_valid && motherboard_mega ? motherboard : transaction{};
			// GS can select a Mega II transaction before its aligned PH0 cycle
			// starts. Its address is not exposed to the slot until that cycle's
			// bank interval, exactly as step_reference's selected predicate.
			auto selected = [&](transaction const &word)
			{
				const uint64_t line = gs_start / LINE * LINE, pos = gs_start - line, k = (pos + MEGA - 1) / MEGA;
				const uint64_t end = pos <= 64 * MEGA ? line + k * MEGA + (k == 64 ? 16 * CLK : MEGA) : line + LINE + MEGA;
				const uint64_t start = end - (end % LINE ? MEGA : 16 * CLK);
				return time >= start + 2 * CLK && time < end ? word : transaction{};
			};
			if (retry)
				return motherboard_mega ? selected(motherboard) : transaction{};
			if (!((c.ctl.service && c.ctl.sync) || !c.ctl.sync))
				return {};
			const unsigned column = unsigned(controller::bit(c.ctl.q1, 16)) | (unsigned(controller::bit(c.ctl.q1, 17)) << 1);
			payload socket = c.latches[column];
			if (!controller::bit(c.ctl.q2, 15) && c.ctl.capture_column() == column)
				socket = {c.word, 0, true};
			payload active = c.active;
			if ((c.ctl.service && c.ctl.sync) || (!c.ctl.sync && !c.ctl.read))
				active = socket;
			auto word = active.valid ? active.bus : (socket.valid ? socket.bus : c.word);
			if (!active.valid || active.bus.read())
				word.type = c.word.type;
			if ((active.valid || socket.valid) && (word.address >> 16) != 0xbc && base::bus_class(word) == 3)
				return selected(word);
			return {};
		}
		const auto p = clock::at(time);
		const uint64_t start = slot_end - (slot_end % LINE ? MEGA : 16 * CLK);
		if (motherboard_valid && motherboard_mega && time >= start + 2 * CLK && time < slot_end)
			return motherboard;
		return falling_valid && time == p.start ? falling_bus : transaction{};
	}

	[[gnu::always_inline]] bool phase_idle(uint64_t time) const noexcept
	{
		if (falling_valid || (motherboard_valid && motherboard_mega))
			return false;
		// GS rising samples the output column selected by GAL1. The only
		// pre-edge latch mutation is transparent capture of the current word;
		// service/sync themselves change only at GS falling. Predict exactly
		// that candidate instead of considering unrelated historical columns.
		if (gs_event == time && next_gs_level && ((c.ctl.service && c.ctl.sync) || !c.ctl.sync))
		{
			const unsigned column = unsigned(controller::bit(c.ctl.q1, 16)) | (unsigned(controller::bit(c.ctl.q1, 17)) << 1);
			payload socket = c.latches[column];
			if (!controller::bit(c.ctl.q2, 15) && c.ctl.capture_column() == column)
				socket = {c.word, 0, true};
			payload active = c.active;
			if ((c.ctl.service && c.ctl.sync) || (!c.ctl.sync && !c.ctl.read))
				active = socket;
			auto word = active.valid ? active.bus : (socket.valid ? socket.bus : c.word);
			if (!active.valid || active.bus.read())
				word.type = c.word.type;
			if ((active.valid || socket.valid) && (word.address >> 16) != 0xbc && base::bus_class(word) == 3)
				return false;
		}
		return true;
	}

	template <class Next> [[gnu::always_inline]] void gs_edge(bool level, Next &&next) noexcept
	{
		if (!base::fast_hits || std::endian::native != std::endian::little || !c.external_advance)
		{
			ready.gs_edge(c, level, next);
			return;
		}
		// The same GS memo as the card-absent engine, with U23's direct PAUSE
		// input and the held-write retirement gate. U30 still changes only on
		// LATE, and GAL2 sees its previous value on that edge.
		c.ctl.pause = ready.gs_ok();
		uint64_t key;
		std::memcpy(&key, &c.ctl.r16_sample, 8);
		key &= 0xffffffff00ff0000ULL;
		const uint64_t fixed = (uint64_t(c.ctl.pin7_card_phase) << 4) | (uint64_t(cache_decode::pad(c.fp, 53)) << 6);
		uint64_t a;
		std::memcpy(&a, &c.ctl.q1, 8);
		const uint64_t b = key | uint64_t(c.ctl.r16_sample) | (uint64_t(c.ctl.r17_sample) << 8) |
			((base::card_byte(fixed) | (uint64_t(level) << 7)) << 24);
		auto const &e = gs_cache.get(a & gs_memo::MASK_A, b);
		const uint32_t eff = e.eff;
		c.completed = c.delivered = false;
		if (eff & gs_memo::PRE)
			base::capture(eff & 3, eff & 4);
		if (!level)
		{
			if (ready.gs_ok() && c.active.valid && !c.active.bus.read())
			{
				if (!(c.active == c.latches[c.active_column]))
					++c.high_errors;
				c.retired = c.active;
				c.delivered = true;
				++c.deliveries;
			}
			c.active.valid = false;
		}
		a = (a & ~gs_memo::CHANGED_A) | e.new_a;
		std::memcpy(&c.ctl.q1, &a, 8);
		c.ctl.r16_sample = e.new_b & 1;
		c.ctl.r17_sample = (e.new_b >> 8) & 1;
		c.ctl.miss = e.miss;
		std::memcpy(&c.comparator_enable, &e.card4, 4);
		if (eff & gs_memo::SELECT)
		{
			const unsigned col = (eff >> 13) & 3;
			if (!level)
			{
				c.bank_column = col;
				c.bank_candidate = c.latches[col];
				c.bank_valid = true;
			}
			else
			{
				c.active_column = col;
				c.active = c.latches[col];
				if (c.ctl.sync && c.active.valid && !c.active.bus.read() &&
					(!c.bank_valid || c.bank_column != c.active_column || c.bank_candidate.bus.address >> 16 != c.active.bus.address >> 16))
					++c.bank_errors;
			}
		}
		if (eff & gs_memo::SRAM_BEFORE)
			c.write_sram();
		if (eff & gs_memo::COMPLETE)
			base::complete_transfer(eff & gs_memo::Q2_13, (fixed >> 6) & 1);
		if (eff & gs_memo::POST)
			base::capture((eff >> 4) & 3, eff & 64);
		if (eff & gs_memo::SRAM_AFTER)
			c.write_sram();
		c.strobe_changes += eff >> 16;
		c.ctl.pause = ready.u30;
	}

	template <class Bus> [[gnu::always_inline]] bool step_reference(Bus &&bus) noexcept
	{
		// Rendezvous before any event at a slot-phase timestamp, including a
		// tied GS fall that samples data before PH0 falls. Rejection must leave
		// the edge untouched. Approval may be requested again for the same tick
		// until its phase is consumed, so the owner keeps that approval valid.
		if constexpr (requires { bus.prepare_phase(phase_event); })
			if (next_event() == phase_event && !bus.prepare_phase(phase_event))
				return false;
		auto hold = [](transaction const &w, uint8_t) { return w; };
		// As in timing_scaled, GS precedes X4 on a tie. At PH0 fall the
		// motherboard byte is sampled before its slot driver turns off.
		if (gs_event <= phase_event && gs_event <= early && gs_event <= late)
		{
			now = gs_event;
			if (next_gs_level)
			{
				gs_edge(true, hold);
				base::socket = c.latches[c.ctl.output_column()];
				const bool external = retry || (c.ctl.service && c.ctl.sync) || !c.ctl.sync;
				unsigned cls = 3;
				if (retry)
					motherboard_valid = true;
				else
				{
					motherboard = c.active.valid ? c.active.bus : (base::socket.valid ? base::socket.bus : c.word);
					cls = external ? base::bus_class(motherboard) : 0;
					motherboard_valid = ((c.ctl.service && c.ctl.sync) || !c.ctl.sync) && (c.active.valid || base::socket.valid);
					// A latch retains address/data, not the access type of a later CPU
					// read using it. Use the motherboard's live R/W/VP inputs; otherwise
					// a read through a former write's latch invents another Apple write.
					if (!c.active.valid || c.active.bus.read())
					{
						motherboard.type = c.word.type;
						motherboard.id = c.word.id;
					}
					// Bank BC is the TWGS's own FPGA/configuration/ROM decode. Its
					// ordering cycles do not put that write on the motherboard.
					// Keep the timing class, but do not expose a private access to slots.
					motherboard_mega = (motherboard.address >> 16) != 0xbc && base::bus_class(motherboard) == 3;
				}
				slot_start = gs_start;
				slot_end = base::end_of_cycle(gs_start, cls);
				gs_event = slot_end;
				++base::board_cycles;
				if (external && !c.ctl.sync)
				{
					base::had_bus = true;
					base::access_slot_start = slot_start;
					base::access_slot_end = slot_end;
				}
			}
			else
			{
				if (!base::finish_bus(bus, ready.gs_ok()))
					return false;
				if (!c.ctl.sync && c.word.read() && base::socket.valid)
				{
					transaction r = base::socket.bus;
					r.type = c.word.type;
					r.id = c.word.id;
					// The slot's falling-edge latches see the returned motherboard byte,
					// not the data retained in an earlier address-latch payload.
					if (motherboard.read() && motherboard.address == r.address)
						motherboard.data = c.word.data;
				}
				falling_bus = motherboard;
				falling_valid = motherboard_valid && motherboard_mega;
				retry = motherboard_valid && !ready.gs_ok();
				gs_edge(false, hold);
				gs_start = now;
				gs_event = now + 2 * CLK;
				motherboard_valid = false;
			}
			next_gs_level = !next_gs_level;
		}
		else if (phase_event <= early && phase_event <= late)
		{
			now = phase_event;
			phase_event += CLK;
			const auto p = clock::at(now);
			// The existing observer contract supplies only /M2SEL-low transfers
			// only. Between them, advance clocks on an inactive read decode;
			// this does not claim to recover the unspecified fast-side bus waveform.
			const uint64_t start = slot_end - (slot_end % LINE ? MEGA : 16 * CLK);
			const bool selected = motherboard_valid && motherboard_mega && now >= start + 2 * CLK && now < slot_end;
			transaction const idle{};
			const transaction &w = selected ? motherboard : (falling_valid && now == p.start ? falling_bus : idle);
			const int data = bus.phase(w, now, p.ph0, p.m7, selected, p.bank);
			if (data >= 0 && selected && w.read() && !c.ctl.sync)
				c.word.data = uint8_t(data);
			if (p.m7)
				ready.slot_7m_rise(p.ph0, p.q3, bus.rdy());
			falling_valid = false;
		}
		else if (early <= late)
		{
			now = early;
			early += x4;
			c.early_edge();
		}
		else
		{
			now = late;
			late += x4;
			ready.late_edge(c, hold);
		}
		if (pending && c.completed)
			pending = false;
		return true;
	}

	// Compose EARLY/LATE recurrences only while PAUSE's two domains agree.
	// The memo is populated by the original GAL edge equations and retains
	// every latch/SRAM strobe and CPU fall. Stop before any external edge.
	[[gnu::always_inline]] bool x4_run(uint64_t bound) noexcept
	{
		if (!base::fast_hits || std::endian::native != std::endian::little || !c.external_advance || ready.u30 != ready.gs_ok() ||
			c.ctl.pause != ready.u30)
			return false;
		if (late >= bound || (early <= late ? late - early >= x4 : early - late > x4))
			return false;
		uint64_t key;
		std::memcpy(&key, &c.ctl.r16_sample, 8);
		key &= 0xffffffff00ff0000ULL;
		const uint64_t fixed = (uint64_t(c.ctl.pin7_card_phase) << 4) | (uint64_t(cache_decode::pad(c.fp, 53)) << 6);
		if (early <= late)
		{
			if (late >= bound || late - early >= x4)
				return false;
			base::template x4_span<true>(key, fixed, bound);
		}
		else
		{
			if (late >= bound || early - late > x4)
				return false;
			base::template x4_span<false>(key, fixed, bound);
		}
		if (pending && c.completed)
			pending = false;
		return true;
	}

	template <class Bus> [[gnu::always_inline]] bool step(Bus &&bus, uint64_t deadline = ~uint64_t(0)) noexcept
	{
		const uint64_t after = deadline == ~uint64_t(0) ? deadline : deadline + 1;
		if constexpr (requires {
						  bus.idle_phases();
						  bus.idle_phases(now, now);
					  })
		{
			const bool candidate = base::fast_hits && phase_event < gs_event && ready.h && ready.dma && ready.u30;
			bool neutral = candidate && phase_idle(phase_event) && bus.idle_phases();
			// read_phases certifies unchanged inputs and an undriven data bus.
			if constexpr (requires { bus.read_phases(motherboard); })
				if (candidate && !neutral && phase_read(phase_event))
					neutral = bus.read_phases(motherboard);
			if (neutral)
			{
				uint64_t bound = std::min(gs_event, after);
				if constexpr (requires { bus.idle_phase_limit(); })
					bound = std::min(bound, bus.idle_phase_limit());
				if (phase_event >= bound)
					return step_reference(bus);
				// This PH0/7M stream cannot change host outputs. Advance its cursor
				// only through the GAL work actually completed, so an early CPU fall
				// retains the identical phase count and time for the next transfer.
				if (std::min(early, late) < bound)
				{
					if (!x4_run(bound))
					{
						auto hold = [](transaction const &w, uint8_t) { return w; };
						if (early <= late)
						{
							now = early;
							early += x4;
							c.early_edge();
						}
						else
						{
							now = late;
							late += x4;
							ready.late_edge(c, hold);
						}
						if (pending && c.completed)
							pending = false;
					}
				}
				else
					now = (bound - 1) / CLK * CLK;
				if (phase_event <= now)
				{
					const uint64_t last = now / CLK * CLK;
					bus.idle_phases(phase_event, last);
					phase_event = last + CLK;
					falling_valid = false;
				}
				return true;
			}
		}
		const uint64_t bound = std::min(after, std::min(gs_event, phase_event));
		if (x4_run(bound))
			return true;
		return step_reference(bus);
	}

	template <class Bus> bool resume(uint64_t deadline, Bus &&bus) noexcept
	{
		while (pending && next_event() <= deadline)
			if (!step(bus, deadline))
				return false;
		// The card's falling PH0 edge shares the completed socket boundary.
		// Deliver it before waking the CPU, including its CATCH/INT1 effects.
		if (!pending && phase_event == now && phase_event <= deadline && !step_reference(bus))
			return false;
		return !pending;
	}

	template <class Bus> bool idle_hit(Bus &&bus) noexcept
	{
		if constexpr (!requires {
						  bus.idle_phases();
						  bus.idle_phases(now, now);
						  bus.idle_phase_limit();
					  })
			return false;
		else
		{
			if (!base::fast_hits || std::endian::native != std::endian::little || !pending || !ready.h || !ready.dma || !ready.u30 ||
				motherboard_valid || falling_valid || retry || base::bus_stage || !c.ctl.read || !c.ctl.eligible || c.ctl.slow ||
				c.ctl.miss || !c.ctl.pause || !c.ctl.pause_sample || !c.ctl.sync || c.ctl.service || (c.ctl.q1 & 5) || c.active.valid ||
				c.forced_write || c.we || c.ctl.q2 != 0xbf || !c.raw_match || !c.comparator_enable || !cache_decode::pad(c.fp, 53) ||
				early < gs_start || late < gs_start || gs_event < gs_start || !bus.idle_phases())
				return false;
			auto const &chain = idle_reads.states[c.ctl.q1];
			// Only the oscillator cursors enter this closed-form interval. GS
			// edges still sample their actual prefix, including socket/falling
			// payloads, U23/U45, refresh, and both U60 samples. No external event
			// may intervene; otherwise use the general memo/edge path below.
			const uint64_t end = late + 3 * x4;
			if (chain.valid && !c.cpu && (early <= late ? late - early < x4 : early - late <= x4) &&
				x4 <= (~uint64_t(0) - std::max(early, late)) / 4 && end < bus.idle_phase_limit())
			{
				const unsigned ne = early <= late ? 4 : 3;
				const uint64_t last_early = early + (ne - 1) * x4;
				bool sample = c.ctl.gs, previous = c.ctl.gs;
				auto prefix = [&](unsigned n)
				{
					auto const &s = chain.s[n];
					c.ctl.q1 = s.q1;
					c.ctl.q2 = s.q2;
					for (unsigned mask = s.data | s.bank; mask; mask &= mask - 1)
					{
						const unsigned col = __builtin_ctz(mask);
						base::capture(col, (s.data >> col) & 1);
					}
				};
				while (gs_event <= end)
				{
					const uint64_t boundary = gs_event;
					const unsigned n = (boundary > late) + (boundary > late + x4) + (boundary > late + 2 * x4);
					prefix(n);
					if (boundary <= last_early)
						sample = next_gs_level;
					if (boundary <= last_early - x4)
						previous = next_gs_level;
					c.ctl.gs_edge(next_gs_level);
					if (next_gs_level)
					{
						base::socket = c.latches[c.ctl.output_column()];
						motherboard = base::socket.valid ? base::socket.bus : c.word;
						motherboard.type = c.word.type;
						motherboard.id = c.word.id;
						motherboard_mega = (motherboard.address >> 16) != 0xbc && base::bus_class(motherboard) == 3;
						slot_start = gs_start;
						slot_end = base::end_of_cycle(gs_start, 0);
						gs_event = slot_end;
						++base::board_cycles;
					}
					else
					{
						falling_bus = motherboard;
						gs_start = boundary;
						gs_event = boundary + 2 * CLK;
					}
					next_gs_level = !next_gs_level;
				}
				prefix(4);
				c.ctl.early = sample;
				c.ctl.delayed = previous;
				early += ne * x4;
				late += 4 * x4;
				now = end;
				c.cpu = false;
				c.completed = c.delivered = false;
				base::complete_transfer(true, true);
				if (phase_event <= now)
				{
					const uint64_t last = now / CLK * CLK;
					bus.idle_phases(phase_event, last);
					phase_event = last + CLK;
				}
				pending = false;
				return true;
			}
			const uint64_t origin = gs_start;
			uint64_t a, b;
			std::memcpy(&a, &c.ctl.q1, 8);
			std::memcpy(&b, &c.ctl.r16_sample, 8);
			const uint64_t fixed = (uint64_t(c.ctl.pin7_card_phase) << 4) | (uint64_t(cache_decode::pad(c.fp, 53)) << 6);
			b = (b & 0xffffffff00ffffffULL) | (base::card_byte(fixed) << 24);
			auto const &e = idle_hits<Scale>.get({a, b, early - origin, late - origin, gs_event - origin, x4, origin % (50 * CLK)});
			if (!e.valid || origin + e.end >= bus.idle_phase_limit())
				return false;

			auto payload_at = [&](unsigned source)
			{ return source < 4 ? c.latches[source] : payload{c.word, source == 4 ? c.word.data : uint8_t(c.word.address >> 16), true}; };
			auto word_at = [&](unsigned source)
			{
				const auto p = payload_at(source);
				auto w = p.valid ? p.bus : c.word;
				w.type = c.word.type;
				w.id = c.word.id;
				return w;
			};
			if (e.fall)
				falling_bus = e.falling == 6 ? motherboard : word_at(e.falling);
			if (e.rise)
			{
				base::socket = payload_at(e.socket);
				motherboard = word_at(e.motherboard);
				motherboard_mega = (motherboard.address >> 16) != 0xbc && base::bus_class(motherboard) == 3;
				slot_start = origin + e.slot_start;
				slot_end = origin + e.slot_end;
			}
			for (unsigned i = 0; i < 4; ++i)
				if (e.columns[i] >= 4)
					base::capture(i, e.columns[i] == 4);
			c.ctl = e.ctl;
			c.cpu = false;
			c.completed = c.delivered = false;
			base::complete_transfer(e.q2_13, true);
			early = origin + e.early;
			late = origin + e.late;
			gs_event = origin + e.gs;
			gs_start = origin + e.start;
			next_gs_level = e.next_gs;
			now = origin + e.end;
			base::board_cycles += e.boards;
			base::refresh_cycles += e.refreshes;
			if (phase_event <= now)
			{
				const uint64_t last = now / CLK * CLK;
				bus.idle_phases(phase_event, last);
				phase_event = last + CLK;
			}
			pending = false;
			return true;
		}
	}

	template <class Bus> [[gnu::always_inline]] bool access_or_wait(Bus &&bus) noexcept
	{
		if (idle_hit(bus))
			return true;
		return access_wait_edges(bus);
	}

	template <class Bus> [[gnu::noinline]] bool access_wait_edges(Bus &&bus) noexcept
	{
		// Ordinary CPU transfers keep the existing synchronous bus contract.
		// A shared-latch rendezvous pauses this edge without elapsed clocks;
		// an external ready hold waits for its physical release. Suspending at
		// every accelerator edge would let the posted-write timer clock the old
		// CPU word before the core can present its next address.
		while (pending)
		{
			if (!step(bus))
				return false;
			if (pending && !bus.rdy())
				return false;
		}
		if (phase_event == now && !step_reference(bus))
			return false;
		return true;
	}
};
} // namespace twgs_decoded

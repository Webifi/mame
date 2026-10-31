// license:BSD-3-Clause
#ifndef MAME_VIDEO_OTI087_MEMORY_H
#define MAME_VIDEO_OTI087_MEMORY_H
#pragma once

// Shared DRAM service, evaluated only on a CPU/DMA request. OTI-087 p56:
// 40MHz MCLK, random cycle 7, RAS 4, precharge 3, page cycle 2, CBR
// setup 1. This is NOT a recovered Oak arbiter: the parameters below
// name its undocumented parts, separately from the specified DRAM edges.
struct oti087_memory_timing
{
	static constexpr unsigned mclk_hz = 40'000'000;
	static constexpr s64 clock_as = ATTOSECONDS_PER_SECOND / mclk_hz;
	static constexpr unsigned random_cycle = 7, ras_data = 4, precharge = 3, page_cycle = 2;
	static constexpr unsigned refresh_setup = 1, read_pipeline = 1;
	// Conventions (05-z180, same section): acceptance at the second edge;
	// one 32-bit read-cache line and one four-byte write-cache page. The
	// 32-bit DRAM bus motivates these minimum geometries, not a cache spec.
	static constexpr unsigned accept_edges = 2, cache_bytes = 4;
	// Convention: sixteen 32-bit FIFO entries (one more than ext20's largest
	// minimum), prefetched in HBLANK; refill consumed words with page cycles.
	// Convention (07-glue-logic, "Copy timing budget"): refresh starts at
	// HBLANK, but a waiting CPU wins at the next refresh-cycle boundary.
	// Remaining refreshes resume after that access; none are discarded.
	// Fill the display FIFO before starting refresh (same convention): this
	// avoids stacking a refresh and an empty-FIFO refill ahead of a request.
	// A CPU can preempt display after its current word and ext20 minimum.
	static constexpr unsigned fifo_words = 16;
	static constexpr unsigned reset_phase_ns = 7;
	// s20 R32 IOCHRDY pull-up resistor=2K: 14pF pin load, unknown trace
	// taken as zero; -10 U15
	// Z180 memory decoder, combinational output. Named approximations,
	// not the former fixed five-wait upper bound. UM0050 tWS = 15ns.
	static constexpr unsigned command_delay_ns = 35, rise_ns = 14, decoder_ns = 10, setup_ns = 15;

	// All service timestamps are edges of the independently running MCLK.
	s64 busy = 0, line = 0, active_as = 0, word_as = clock_as * 8;
	s64 write_start = 0, write_end = 0, read_end = 0;
	u32 words = 0, served = 0, read_tag = ~u32(0), write_tag = ~u32(0);
	u32 refresh_pending = 0;
	u8 write_mask = 0;
	bool display_open = false, refresh_open = false;

	static s64 ceil_clock(s64 atto) { return atto / clock_as + (atto % clock_as > 0); }
	static s64 edge(attotime t) { return s64(t.seconds()) * mclk_hz + t.attoseconds() / clock_as; }
	static attotime time(s64 t) { return t < 0 ? attotime::zero - attotime::from_ticks(-t, mclk_hz) : attotime::from_ticks(t, mclk_hz); }
	static s64 accept(attotime command) { return edge(command) + accept_edges; }
	static unsigned refreshes(u8 crtc11, u8 ext21) { return BIT(ext21, 7) ? 1 : BIT(crtc11, 6) ? 5 : 3; }
	static unsigned waits(attotime first_sample, attotime phi, attotime ready)
	{
		if (ready <= first_sample) return 0;
		s64 const remaining = (ready - first_sample).as_attoseconds();
		return (remaining + phi.as_attoseconds() - 1) / phi.as_attoseconds();
	}
	static attotime sampled_ready(attotime release)
	{
		return release + attotime::from_nsec(rise_ns + decoder_ns + setup_ns);
	}

	// Called at a new raster line, lazily. Completed idle lines have no
	// outstanding work: skip them arithmetically. An unfinished CPU/cache
	// transfer retains its reservation and delays this line's refresh.
	void begin_line(s64 start, s64 active_delay_as, s64 word_period_as, u32 visible_words, unsigned refresh)
	{
		// Retire the preceding line's work before changing its raster inputs.
		// A cycle already in flight, or an outstanding refresh, survives.
		advance(start);
		line = start;
		active_as = active_delay_as;
		word_as = std::max(clock_as * page_cycle, word_period_as);
		words = visible_words;
		served = 0;
		busy = std::max(busy, start) + (display_open ? precharge : 0);
		refresh_pending += refresh;
		display_open = false;
	}
	u32 drained(s64 t) const
	{
		s64 const elapsed = (t - line) * clock_as - active_as;
		return elapsed <= 0 ? 0 : std::min<s64>(words, elapsed / word_as);
	}
	u32 demand(s64 t) const { return words ? fifo_words + drained(t) : 0; }
	s64 due(u32 i) const
	{
		return i < fifo_words ? line : line + ceil_clock(active_as + (i - fifo_words + 1) * word_as);
	}

	// Arithmetic equivalent of issuing one replacement read per drained
	// word. For period >= 2 MCLK, the last completion is the maximum of
	// the backlog's page-service finish and the last arrival + two clocks.
	// It also retains a read already in flight when the CPU arrives.
	void advance(s64 now)
	{
		if (refresh_pending && words && served < fifo_words)
		{
			advance_display(now, fifo_words);
			if (served < fifo_words || busy >= now) return;
		}
		if (refresh_pending && busy < now)
		{
			// Issue only cycles that started BEFORE the request. At a tie
			// the CPU wins. CAS can stay low across consecutive CBR cycles;
			// restarting after another owner pays the setup clock again.
			s64 const start = busy + (display_open ? precharge : 0);
			if (start < now)
			{
				unsigned const setup = refresh_open ? 0 : refresh_setup;
				u32 const issued = std::min<s64>(refresh_pending,
					std::max<s64>(1, (now - start - setup + random_cycle - 1) / random_cycle));
				busy = start + setup + random_cycle * issued;
				refresh_pending -= issued;
				refresh_open = true;
				display_open = false;
			}
		}
		if (!refresh_pending) advance_display(now, words + fifo_words);
	}
	void advance_display(s64 now, u32 limit)
	{
		if (!words || busy >= now) return;
		u32 const pending = std::min(demand(now), limit) - served;
		if (!pending) return;
		s64 const first = std::max(busy, due(served)) + (display_open ? page_cycle : ras_data);
		if (first <= now)
		{
			u32 const completed = std::min<s64>((now - first) / page_cycle + 1,
				std::max<s64>(0, s64(std::min(demand(now - page_cycle), limit)) - served));
			if (completed)
			{
				busy = std::max(first + page_cycle * (completed - 1), due(served + completed - 1) + page_cycle);
				served += completed;
				display_open = true;
			}
		}
		if (served < std::min(demand(now), limit) && std::max(busy, due(served)) < now)
		{
			busy = std::max(busy, due(served)) + (display_open ? page_cycle : ras_data);
			++served;
			display_open = true;
		}
		if (display_open) refresh_open = false;
	}

	// Return the CPU's RAS start. A display word already started finishes;
	// refill to ext20, then change owner with the specified precharge.
	s64 arbitrate(s64 arrival, unsigned minimum)
	{
		advance(arrival);
		s64 start = std::max(arrival, busy);
		if (words && s64(served) - drained(start + (display_open ? precharge : 0)) < minimum)
		{
			// Solve n + served - floor((a + 2n) / word_period) >= minimum.
			// Clipping at active end also handles the full 80MB/s page rate.
			s64 const extra = display_open ? 0 : ras_data - page_cycle;
			s64 const a = (start + extra + precharge - line) * clock_as - active_as;
			s64 const rhs = a - (s64(served) - minimum + 1) * word_as;
			s64 const gap = word_as - page_cycle * clock_as;
			s64 n = std::max<s64>(1, s64(minimum) + drained(start) - served);
			if (rhs >= 0) n = std::max(n, gap ? rhs / gap + 1 : s64(words) + minimum - served);
			n = std::min(n, s64(words) + minimum - served);
			start += extra + page_cycle * n;
			served += n;
			display_open = true;
		}
		if (display_open) start += precharge;
		display_open = false;
		refresh_open = false;
		return start;
	}

	// Cache tags denote DRAM words, not CPU aperture addresses. Cache state
	// affects timing only; the normal VGA path still loads its latches and
	// performs every ROP/write. Any write invalidates the read-cache line
	// (convention, 15-oak-drawing "Caches").
	s64 access(s64 arrival, u32 tag, unsigned lane, bool write, bool read_cache, bool write_cache,
		bool flush_write, unsigned minimum, unsigned transfer_words = 1, u8 lanes = 0)
	{
		if (transfer_words > 1) read_cache = false;
		if (!read_cache) read_tag = ~u32(0);
		if (write)
		{
			u8 const mask = lanes ? lanes : 1 << lane;
			read_tag = ~u32(0);
			if (write_cache && transfer_words == 1 && !flush_write && arrival < write_start && tag == write_tag && !(write_mask & mask))
			{
				write_mask |= mask;
				return arrival; // still-buffered same-page bytes combine
			}
			s64 const accepted = std::max(arrival, write_end);
			write_start = arbitrate(accepted, minimum);
			busy = write_end = write_start + random_cycle + (transfer_words - 1) * page_cycle;
			write_tag = tag;
			write_mask = mask;
			// Bit4 flush starts draining immediately; it does not disable
			// bit3's posted CPU completion. Ordinary entries also drain at
			// the first free DRAM slot here (named eager-drain convention).
			return !write_cache ? write_end : transfer_words == 1 ? accepted : write_end - page_cycle;
		}
		if (read_cache && tag == read_tag && arrival >= read_end)
			return arrival;
		s64 const start = arbitrate(std::max(arrival, write_end), minimum);
		busy = start + random_cycle + (transfer_words - 1) * page_cycle;
		read_end = start + ras_data + read_pipeline + (transfer_words - 1) * page_cycle;
		read_tag = read_cache ? tag : ~u32(0);
		return read_end;
	}
};
#endif

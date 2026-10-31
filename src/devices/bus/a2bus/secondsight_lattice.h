// license:BSD-3-Clause
// Generated from the decoded original Lattice chip (BE8C) fuse map.
// No gate delays or equations for undumped GALs are asserted here:
// the Z180 I/O decoder, Z180 memory decoder, copy controller and
// AppleColor RGB timing GAL.
// Schematic sheet 1: pin 35 is unused Y1, not an external reset.
// The 15 registers clear only at power-up; Apple reset preserves them.
#ifndef MAME_BUS_A2BUS_SECONDSIGHT_LATTICE_H
#define MAME_BUS_A2BUS_SECONDSIGHT_LATTICE_H
#pragma once
#include <array>
#include <cstdint>
struct secondsight_lattice
{
 enum signal : unsigned {
  HIRES = 0,
  ADOUT_ACTIVE = 1,
  G00_Q2 = 2,
  ADIN_ACTIVE = 3,
  EVENT_A3 = 4,
  HD3_DRIVE = 5,
  DIRTY_9000 = 6,
  DIRTY_5000 = 7,
  COPY_INHIBIT = 8,
  CPU_DELAY = 9,
  BANK_WINDOW = 10,
  COPY_INHIBIT_ALIAS = 11,
  G03_Q0 = 12,
  C0_VALID = 13,
  HOST_DATA_OE = 14,
  CAPTURE = 15,
  RANGE_2000_5FFF = 16,
  RANGE_0400_0BFF = 17,
  RANGE_C000_C0FF = 18,
  RANGE_2000_9FFF = 19,
  BANK_HELD_ALIAS = 20,
  BANK_HELD = 21,
  CATCH_ACTIVE = 22,
  G05_Q3 = 23,
  EVENT_A1 = 24,
  DIRTY_7000 = 25,
  DIRTY_3000 = 26,
  HD1_DRIVE = 27,
  FLAGS_ACCESS = 28,
  ACK_A0 = 29,
  MODE_D3 = 30,
  FLAGS_ACCESS_ALIAS = 31,
  G10_Q0 = 32,
  G10_Q1 = 33,
  BANK_Q = 34,
  G10_Q3 = 35,
  RUN = 36,
  IRQ_PENDING = 37,
  STALL = 38,
  EVENT_LATCH = 39,
  EVENT_A5 = 40,
  EVENT_A4 = 41,
  EVENT_A7 = 42,
  EVENT_A6 = 43,
  EVENT_A2 = 44,
  DIRTY_8000 = 45,
  DIRTY_4000 = 46,
  HD2_DRIVE = 47,
  WRITE_2000_3FFF = 48,
  WRITE_8000_9FFF = 49,
  WRITE_6000_7FFF = 50,
  WRITE_4000_5FFF = 51,
  G15_Q0 = 52,
  PAGE2 = 53,
  RAMWRT = 54,
  STORE80 = 55,
  DIRTY_6000 = 56,
  EVENT_A0 = 57,
  DIRTY_2000 = 58,
  HD0_DRIVE = 59,
  LOAD_PAGE2 = 60,
  EVENT_DECODE = 61,
  LOAD_RAMWRT = 62,
  LOAD_STORE80 = 63,
 };
 struct inputs {
  std::uint16_t address = 0;
  std::uint8_t data = 0, port = 0, host_data = 0;
  bool CPU = false, M2B0 = false, M2SEL_n = true, AX = false, Y0 = true, Y2 = true;
  bool read = true;
  bool operator==(inputs const &) const = default;
 };
 std::array<bool, 64> q{};
 std::array<bool, 64> clocks{};
 // Registered outputs clear at power-up (Lattice 1992 Data Book p4-7).
 // Feedback startup is a convention; firmware initializes those latches.
 void power_on() { q.fill(false); clocks.fill(false); q[RUN] = true; }
 // Delta-cycle Boolean evaluation, not a gate-delay simulation.
 static bool state_equal(std::array<bool, 64> const &a, std::array<bool, 64> const &b)
 {
  // No early exit: the compiler can compare all bits in parallel on hits.
  unsigned different = 0;
  for (unsigned i = 0; i < 64; ++i) different |= unsigned(a[i]) ^ unsigned(b[i]);
  return !different;
 }
 struct cached_edge {
  std::uint64_t pins = 0;
  std::array<bool, 64> before{}, clocks_before{}, after{}, clocks_after{};
  bool valid = false;
 };
 std::array<cached_edge, 1024> edges{};
 bool settle(inputs const &p)
 {
  // Memoize only input bits that can affect this decoded transition.
  // Equations still use p; all output and clock-history bits stay in the key.
  auto key = p;
  key.data &= 0x50;
  key.port &= 0x07;
  key.host_data &= 0x0c;
  // ~Y0 is constant during settle, so these registers can clock only
  // on the first delta. Use the incoming LOAD and clock history even after
  // a state restore; a later decoded LOAD cannot sample AD4/AD6 on this edge.
  key.data &= (!p.Y0 && (p.address & 0x20))
   ? ((q[LOAD_PAGE2] && !clocks[PAGE2] ? 0x40 : 0)
    | (q[LOAD_RAMWRT] && !clocks[RAMWRT] ? 0x10 : 0)) : 0;
  // Every HD2/HD3 product term requires AX and HA2:0 = 1.
  if (!p.AX || (p.port & 7) != 1) key.host_data = 0;
  // The generator proves these zero nodes stay zero outside C0xx,
  // and then proves every next output/clock independent of AA0-7. Checking
  // incoming feedback also preserves unsettled and restored transitions.
  // Keep the four adjacent feedback nodes together for a word comparison.
  if ((p.address & 0xff00) != 0xc000
   && !(q[RANGE_C000_C0FF] | q[C0_VALID] | q[EVENT_LATCH])
   && std::array<bool, 4>{q[LOAD_PAGE2], q[EVENT_DECODE], q[LOAD_RAMWRT], q[LOAD_STORE80]} == std::array<bool, 4>{}) key.address &= 0xff00;
  // A lossless 47-bit encoding of the canonical input fields. Raw pins
  // still reach the equations; no padding bytes enter the comparison.
  const std::uint64_t memo_pins = std::uint64_t(key.address)
   | (std::uint64_t(key.data) << 16) | (std::uint64_t(key.port) << 24)
   | (std::uint64_t(key.host_data) << 32) | (std::uint64_t(key.CPU) << 40)
   | (std::uint64_t(key.M2B0) << 41) | (std::uint64_t(key.M2SEL_n) << 42)
   | (std::uint64_t(key.AX) << 43) | (std::uint64_t(key.Y0) << 44)
   | (std::uint64_t(key.Y2) << 45) | (std::uint64_t(key.read) << 46);
  // Separate the incoming 7M clock histories even when pins and q agree.
  // AX/read also distinguish the host and Apple bus phases. Every input,
  // output and clock bit is still compared before accepting a hit.
  const unsigned slot = (key.address ^ (key.address >> 6) ^ (key.port << 2)
   ^ (unsigned(key.CPU) << 3) ^ (unsigned(key.Y0) << 4) ^ (unsigned(key.Y2) << 5)
   ^ (unsigned(q[CPU_DELAY]) << 6) ^ (unsigned(q[BANK_WINDOW]) << 7)
   ^ unsigned(clocks[HIRES]) ^ (unsigned(key.AX) << 8) ^ (unsigned(key.read) << 9)) & 1023;
  auto &e = edges[slot];
  if (e.valid && e.pins == memo_pins && state_equal(e.before, q) && state_equal(e.clocks_before, clocks))
  {
   q = e.after; clocks = e.clocks_after;
   return true;
  }
  e.valid = false; e.pins = memo_pins; e.before = q; e.clocks_before = clocks;
  return settle_miss(p, e);
 }
 bool settle_miss(inputs const &p, cached_edge &e)
 {
  for (unsigned delta = 0; delta < 128; ++delta)
  {
   auto next = q;
   auto next_clocks = clocks;
   // G00.Q0 REG
   next_clocks[HIRES] = (!p.Y0);
   if (next_clocks[HIRES] && !clocks[HIRES]) next[HIRES] = (((((((((bool((p.address >> 3) & 1) & q[HIRES]) | (bool((p.address >> 5) & 1) & q[HIRES])) | (bool((p.address >> 7) & 1) & q[HIRES])) | (q[HIRES] & (!bool((p.address >> 1) & 1)))) | (q[HIRES] & (!bool((p.address >> 2) & 1)))) | (q[HIRES] & (!bool((p.address >> 4) & 1)))) | (q[HIRES] & (!bool((p.address >> 6) & 1)))) | (q[HIRES] & (!q[C0_VALID]))) | ((((((((bool((p.address >> 0) & 1) & bool((p.address >> 1) & 1)) & bool((p.address >> 2) & 1)) & bool((p.address >> 4) & 1)) & bool((p.address >> 6) & 1)) & q[C0_VALID]) & (!bool((p.address >> 3) & 1))) & (!bool((p.address >> 5) & 1))) & (!bool((p.address >> 7) & 1))));
   // G00.Q1 COMB
   next[ADOUT_ACTIVE] = ((((((bool((p.address >> 4) & 1) & bool((p.address >> 5) & 1)) & bool((p.address >> 7) & 1)) & p.read) & q[C0_VALID]) & p.CPU) & (!bool((p.address >> 6) & 1)));
   // G00.Q2 COMB
   next[G00_Q2] = false;
   // G00.Q3 COMB
   next[ADIN_ACTIVE] = (((q[CAPTURE] & p.CPU) & q[RUN]) | ((p.CPU & q[EVENT_LATCH]) & (!p.read)));
   // G01.Q0 REG
   next_clocks[EVENT_A3] = q[EVENT_LATCH];
   if (next_clocks[EVENT_A3] && !clocks[EVENT_A3]) next[EVENT_A3] = bool((p.address >> 3) & 1);
   // G01.Q1 COMB
   next[HD3_DRIVE] = (((((q[FLAGS_ACCESS] & q[HD3_DRIVE]) | ((q[EVENT_A7] & bool((p.port >> 0) & 1)) & bool((p.port >> 1) & 1))) | ((q[DIRTY_9000] & bool((p.port >> 0) & 1)) & (!bool((p.port >> 1) & 1)))) | ((q[EVENT_A3] & bool((p.port >> 1) & 1)) & (!bool((p.port >> 0) & 1)))) | ((q[DIRTY_5000] & (!bool((p.port >> 0) & 1))) & (!bool((p.port >> 1) & 1))));
   // G01.Q2 COMB
   next[DIRTY_9000] = (((bool((p.address >> 12) & 1) & q[WRITE_8000_9FFF]) | (q[DIRTY_9000] & (!q[FLAGS_ACCESS]))) | (q[DIRTY_9000] & (!bool((p.port >> 0) & 1))));
   // G01.Q3 COMB
   next[DIRTY_5000] = (((bool((p.address >> 12) & 1) & q[WRITE_4000_5FFF]) | (q[DIRTY_5000] & bool((p.port >> 0) & 1))) | (q[DIRTY_5000] & (!q[FLAGS_ACCESS])));
   // G02.Q0 COMB
   next[COPY_INHIBIT] = (((((q[COPY_INHIBIT] & bool((p.port >> 1) & 1)) | (q[COPY_INHIBIT] & bool((p.port >> 2) & 1))) | (q[COPY_INHIBIT] & (!p.AX))) | (q[COPY_INHIBIT] & (!bool((p.port >> 0) & 1)))) | ((((p.AX & bool((p.port >> 0) & 1)) & bool((p.host_data >> 2) & 1)) & (!bool((p.port >> 1) & 1))) & (!bool((p.port >> 2) & 1))));
   // G02.Q1 REG
   next_clocks[CPU_DELAY] = (!p.Y2);
   if (next_clocks[CPU_DELAY] && !clocks[CPU_DELAY]) next[CPU_DELAY] = p.CPU;
   // G02.Q2 REG
   next_clocks[BANK_WINDOW] = (!p.Y2);
   if (next_clocks[BANK_WINDOW] && !clocks[BANK_WINDOW]) next[BANK_WINDOW] = (q[CPU_DELAY] & (!p.CPU));
   // G02.Q3 COMB
   next[COPY_INHIBIT_ALIAS] = (((((q[COPY_INHIBIT] & bool((p.port >> 1) & 1)) | (q[COPY_INHIBIT] & bool((p.port >> 2) & 1))) | (q[COPY_INHIBIT] & (!p.AX))) | (q[COPY_INHIBIT] & (!bool((p.port >> 0) & 1)))) | ((((p.AX & bool((p.port >> 0) & 1)) & bool((p.host_data >> 2) & 1)) & (!bool((p.port >> 1) & 1))) & (!bool((p.port >> 2) & 1))));
   // G03.Q0 COMB
   next[G03_Q0] = false;
   // G03.Q1 COMB
   next[C0_VALID] = ((q[RANGE_C000_C0FF] & (!p.M2SEL_n)) | (q[RANGE_C000_C0FF] & (!q[MODE_D3])));
   // G03.Q2 COMB
   next[HOST_DATA_OE] = (p.AX & bool((p.port >> 2) & 1));
   // G03.Q3 COMB
   next[CAPTURE] = ((((((((q[RANGE_0400_0BFF] & (!p.read)) & (!q[COPY_INHIBIT_ALIAS])) & (!p.M2SEL_n)) | (((q[RANGE_0400_0BFF] & (!p.read)) & (!q[COPY_INHIBIT_ALIAS])) & (!q[MODE_D3]))) | (((q[RANGE_2000_5FFF] & (!p.read)) & (!q[COPY_INHIBIT_ALIAS])) & (!p.M2SEL_n))) | (((q[RANGE_2000_5FFF] & (!p.read)) & (!q[COPY_INHIBIT_ALIAS])) & (!q[MODE_D3]))) | ((((((q[BANK_HELD] & q[MODE_D3]) & q[RANGE_2000_9FFF]) & (!p.read)) & (!q[COPY_INHIBIT_ALIAS])) & (!p.M2SEL_n)) & (!q[RAMWRT]))) | ((((((q[MODE_D3] & q[RAMWRT]) & q[RANGE_2000_9FFF]) & (!p.read)) & (!q[BANK_HELD])) & (!q[COPY_INHIBIT_ALIAS])) & (!p.M2SEL_n)));
   // G04.Q0 COMB
   next[RANGE_2000_5FFF] = (((bool((p.address >> 13) & 1) & (!bool((p.address >> 14) & 1))) & (!bool((p.address >> 15) & 1))) | ((bool((p.address >> 14) & 1) & (!bool((p.address >> 13) & 1))) & (!bool((p.address >> 15) & 1))));
   // G04.Q1 COMB
   next[RANGE_0400_0BFF] = ((((((bool((p.address >> 10) & 1) & (!bool((p.address >> 11) & 1))) & (!bool((p.address >> 12) & 1))) & (!bool((p.address >> 13) & 1))) & (!bool((p.address >> 14) & 1))) & (!bool((p.address >> 15) & 1))) | (((((bool((p.address >> 11) & 1) & (!bool((p.address >> 10) & 1))) & (!bool((p.address >> 12) & 1))) & (!bool((p.address >> 13) & 1))) & (!bool((p.address >> 14) & 1))) & (!bool((p.address >> 15) & 1))));
   // G04.Q2 COMB
   next[RANGE_C000_C0FF] = (((((((bool((p.address >> 14) & 1) & bool((p.address >> 15) & 1)) & (!bool((p.address >> 10) & 1))) & (!bool((p.address >> 11) & 1))) & (!bool((p.address >> 12) & 1))) & (!bool((p.address >> 13) & 1))) & (!bool((p.address >> 8) & 1))) & (!bool((p.address >> 9) & 1)));
   // G04.Q3 COMB
   next[RANGE_2000_9FFF] = (((bool((p.address >> 13) & 1) & (!bool((p.address >> 15) & 1))) | (bool((p.address >> 14) & 1) & (!bool((p.address >> 15) & 1)))) | ((bool((p.address >> 15) & 1) & (!bool((p.address >> 13) & 1))) & (!bool((p.address >> 14) & 1))));
   // G05.Q0 COMB
   next[BANK_HELD_ALIAS] = ((((q[BANK_HELD] & p.M2B0) & q[MODE_D3]) | ((q[BANK_HELD] & q[MODE_D3]) & (!q[BANK_WINDOW]))) | (((q[BANK_WINDOW] & p.M2B0) & q[MODE_D3]) & (!p.M2SEL_n)));
   // G05.Q1 COMB
   next[BANK_HELD] = ((((q[BANK_HELD] & p.M2B0) & q[MODE_D3]) | ((q[BANK_HELD] & q[MODE_D3]) & (!q[BANK_WINDOW]))) | (((q[BANK_WINDOW] & p.M2B0) & q[MODE_D3]) & (!p.M2SEL_n)));
   // G05.Q2 COMB
   next[CATCH_ACTIVE] = ((q[CAPTURE] & p.CPU) & (!p.M2B0));
   // G05.Q3 COMB
   next[G05_Q3] = false;
   // G06.Q0 REG
   next_clocks[EVENT_A1] = q[EVENT_LATCH];
   if (next_clocks[EVENT_A1] && !clocks[EVENT_A1]) next[EVENT_A1] = bool((p.address >> 1) & 1);
   // G06.Q1 COMB
   next[DIRTY_7000] = (((bool((p.address >> 12) & 1) & q[WRITE_6000_7FFF]) | (q[DIRTY_7000] & (!q[FLAGS_ACCESS_ALIAS]))) | (q[DIRTY_7000] & (!bool((p.port >> 0) & 1))));
   // G06.Q2 COMB
   next[DIRTY_3000] = (((bool((p.address >> 12) & 1) & q[WRITE_2000_3FFF]) | (q[DIRTY_3000] & bool((p.port >> 0) & 1))) | (q[DIRTY_3000] & (!q[FLAGS_ACCESS_ALIAS])));
   // G06.Q3 COMB
   next[HD1_DRIVE] = (((((q[FLAGS_ACCESS_ALIAS] & q[HD1_DRIVE]) | ((q[EVENT_A5] & bool((p.port >> 0) & 1)) & bool((p.port >> 1) & 1))) | ((q[DIRTY_7000] & bool((p.port >> 0) & 1)) & (!bool((p.port >> 1) & 1)))) | ((q[EVENT_A1] & bool((p.port >> 1) & 1)) & (!bool((p.port >> 0) & 1)))) | ((q[DIRTY_3000] & (!bool((p.port >> 0) & 1))) & (!bool((p.port >> 1) & 1))));
   // G07.Q0 COMB
   next[FLAGS_ACCESS] = ((p.AX & bool((p.port >> 2) & 1)) & (!bool((p.port >> 1) & 1)));
   // G07.Q1 COMB
   next[ACK_A0] = (((p.AX & (!bool((p.port >> 0) & 1))) & (!bool((p.port >> 1) & 1))) & (!bool((p.port >> 2) & 1)));
   // G07.Q2 COMB
   next[MODE_D3] = (((((bool((p.port >> 1) & 1) & q[MODE_D3]) | (bool((p.port >> 2) & 1) & q[MODE_D3])) | (q[MODE_D3] & (!p.AX))) | (q[MODE_D3] & (!bool((p.port >> 0) & 1)))) | ((((p.AX & bool((p.port >> 0) & 1)) & bool((p.host_data >> 3) & 1)) & (!bool((p.port >> 1) & 1))) & (!bool((p.port >> 2) & 1))));
   // G07.Q3 COMB
   next[FLAGS_ACCESS_ALIAS] = ((p.AX & bool((p.port >> 2) & 1)) & (!bool((p.port >> 1) & 1)));
   // G10.Q0 COMB
   next[G10_Q0] = false;
   // G10.Q1 COMB
   next[G10_Q1] = false;
   // G10.Q2 REG
   next_clocks[BANK_Q] = (!(q[CAPTURE] & p.CPU));
   if (next_clocks[BANK_Q] && !clocks[BANK_Q]) next[BANK_Q] = (((((q[BANK_HELD_ALIAS] & q[MODE_D3]) & (!q[RAMWRT])) | ((q[PAGE2] & q[STORE80]) & (!q[BANK_HELD_ALIAS]))) | ((q[RAMWRT] & (!q[BANK_HELD_ALIAS])) & (!q[STORE80]))) | (((q[MODE_D3] & q[RAMWRT]) & (!q[BANK_HELD_ALIAS])) & (!q[HIRES])));
   // G10.Q3 COMB
   next[G10_Q3] = false;
   // G11.Q0 COMB
   next[RUN] = ((((q[RUN] & (!q[IRQ_PENDING])) | ((q[RUN] & (!q[CAPTURE])) & (!p.CPU))) | ((q[RUN] & (!q[CAPTURE])) & (!q[EVENT_DECODE]))) | (((!q[ACK_A0]) & (!p.CPU)) & (!q[IRQ_PENDING])));
   // G11.Q1 COMB
   next[IRQ_PENDING] = ((q[IRQ_PENDING] & (!q[ACK_A0])) | ((q[EVENT_LATCH] & (!q[ACK_A0])) & (!p.CPU)));
   // G11.Q2 COMB
   next[STALL] = ((!q[RUN]) | (q[STALL] & (!p.CPU)));
   // G11.Q3 COMB
   next[EVENT_LATCH] = ((((q[EVENT_LATCH] & (!q[ACK_A0])) & (!p.CPU)) | ((q[EVENT_LATCH] & (!q[ACK_A0])) & (!q[IRQ_PENDING]))) | (((p.CPU & q[EVENT_DECODE]) & q[RUN]) & (!q[IRQ_PENDING])));
   // G12.Q0 REG
   next_clocks[EVENT_A5] = q[EVENT_LATCH];
   if (next_clocks[EVENT_A5] && !clocks[EVENT_A5]) next[EVENT_A5] = bool((p.address >> 5) & 1);
   // G12.Q1 REG
   next_clocks[EVENT_A4] = q[EVENT_LATCH];
   if (next_clocks[EVENT_A4] && !clocks[EVENT_A4]) next[EVENT_A4] = bool((p.address >> 4) & 1);
   // G12.Q2 REG
   next_clocks[EVENT_A7] = q[EVENT_LATCH];
   if (next_clocks[EVENT_A7] && !clocks[EVENT_A7]) next[EVENT_A7] = bool((p.address >> 7) & 1);
   // G12.Q3 REG
   next_clocks[EVENT_A6] = q[EVENT_LATCH];
   if (next_clocks[EVENT_A6] && !clocks[EVENT_A6]) next[EVENT_A6] = bool((p.address >> 6) & 1);
   // G13.Q0 REG
   next_clocks[EVENT_A2] = q[EVENT_LATCH];
   if (next_clocks[EVENT_A2] && !clocks[EVENT_A2]) next[EVENT_A2] = bool((p.address >> 2) & 1);
   // G13.Q1 COMB
   next[DIRTY_8000] = (((q[DIRTY_8000] & (!q[FLAGS_ACCESS])) | (q[DIRTY_8000] & (!bool((p.port >> 0) & 1)))) | (q[WRITE_8000_9FFF] & (!bool((p.address >> 12) & 1))));
   // G13.Q2 COMB
   next[DIRTY_4000] = (((q[DIRTY_4000] & bool((p.port >> 0) & 1)) | (q[DIRTY_4000] & (!q[FLAGS_ACCESS]))) | (q[WRITE_4000_5FFF] & (!bool((p.address >> 12) & 1))));
   // G13.Q3 COMB
   next[HD2_DRIVE] = (((((q[FLAGS_ACCESS] & q[HD2_DRIVE]) | ((q[EVENT_A6] & bool((p.port >> 0) & 1)) & bool((p.port >> 1) & 1))) | ((q[DIRTY_8000] & bool((p.port >> 0) & 1)) & (!bool((p.port >> 1) & 1)))) | ((q[EVENT_A2] & bool((p.port >> 1) & 1)) & (!bool((p.port >> 0) & 1)))) | ((q[DIRTY_4000] & (!bool((p.port >> 0) & 1))) & (!bool((p.port >> 1) & 1))));
   // G14.Q0 COMB
   next[WRITE_2000_3FFF] = ((((bool((p.address >> 13) & 1) & q[CAPTURE]) & p.CPU) & (!bool((p.address >> 14) & 1))) & (!bool((p.address >> 15) & 1)));
   // G14.Q1 COMB
   next[WRITE_8000_9FFF] = ((((bool((p.address >> 15) & 1) & q[CAPTURE]) & p.CPU) & (!bool((p.address >> 13) & 1))) & (!bool((p.address >> 14) & 1)));
   // G14.Q2 COMB
   next[WRITE_6000_7FFF] = ((((bool((p.address >> 13) & 1) & bool((p.address >> 14) & 1)) & q[CAPTURE]) & p.CPU) & (!bool((p.address >> 15) & 1)));
   // G14.Q3 COMB
   next[WRITE_4000_5FFF] = ((((bool((p.address >> 14) & 1) & q[CAPTURE]) & p.CPU) & (!bool((p.address >> 13) & 1))) & (!bool((p.address >> 15) & 1)));
   // G15.Q0 COMB
   next[G15_Q0] = false;
   // G15.Q1 REG
   next_clocks[PAGE2] = (!p.Y0);
   if (next_clocks[PAGE2] && !clocks[PAGE2]) next[PAGE2] = (((q[PAGE2] & (!q[LOAD_PAGE2])) | ((bool((p.address >> 5) & 1) & bool((p.data >> 6) & 1)) & q[LOAD_PAGE2])) | ((bool((p.address >> 0) & 1) & q[LOAD_PAGE2]) & (!bool((p.address >> 5) & 1))));
   // G15.Q2 REG
   next_clocks[RAMWRT] = (!p.Y0);
   if (next_clocks[RAMWRT] && !clocks[RAMWRT]) next[RAMWRT] = (((q[RAMWRT] & (!q[LOAD_RAMWRT])) | ((bool((p.address >> 5) & 1) & bool((p.data >> 4) & 1)) & q[LOAD_RAMWRT])) | ((bool((p.address >> 0) & 1) & q[LOAD_RAMWRT]) & (!bool((p.address >> 5) & 1))));
   // G15.Q3 REG
   next_clocks[STORE80] = (!p.Y0);
   if (next_clocks[STORE80] && !clocks[STORE80]) next[STORE80] = ((bool((p.address >> 0) & 1) & q[LOAD_STORE80]) | (q[STORE80] & (!q[LOAD_STORE80])));
   // G16.Q0 COMB
   next[DIRTY_6000] = (((q[DIRTY_6000] & (!q[FLAGS_ACCESS_ALIAS])) | (q[DIRTY_6000] & (!bool((p.port >> 0) & 1)))) | (q[WRITE_6000_7FFF] & (!bool((p.address >> 12) & 1))));
   // G16.Q1 REG
   next_clocks[EVENT_A0] = q[EVENT_LATCH];
   if (next_clocks[EVENT_A0] && !clocks[EVENT_A0]) next[EVENT_A0] = bool((p.address >> 0) & 1);
   // G16.Q2 COMB
   next[DIRTY_2000] = (((q[DIRTY_2000] & bool((p.port >> 0) & 1)) | (q[DIRTY_2000] & (!q[FLAGS_ACCESS_ALIAS]))) | (q[WRITE_2000_3FFF] & (!bool((p.address >> 12) & 1))));
   // G16.Q3 COMB
   next[HD0_DRIVE] = (((((q[FLAGS_ACCESS_ALIAS] & q[HD0_DRIVE]) | ((q[EVENT_A4] & bool((p.port >> 0) & 1)) & bool((p.port >> 1) & 1))) | ((q[DIRTY_6000] & bool((p.port >> 0) & 1)) & (!bool((p.port >> 1) & 1)))) | ((q[EVENT_A0] & bool((p.port >> 1) & 1)) & (!bool((p.port >> 0) & 1)))) | ((q[DIRTY_2000] & (!bool((p.port >> 0) & 1))) & (!bool((p.port >> 1) & 1))));
   // G17.Q0 COMB
   next[LOAD_PAGE2] = (((((((((bool((p.address >> 2) & 1) & bool((p.address >> 4) & 1)) & bool((p.address >> 6) & 1)) & q[C0_VALID]) & p.CPU) & (!bool((p.address >> 1) & 1))) & (!bool((p.address >> 3) & 1))) & (!bool((p.address >> 5) & 1))) & (!bool((p.address >> 7) & 1))) | (((((((((bool((p.address >> 3) & 1) & bool((p.address >> 5) & 1)) & bool((p.address >> 6) & 1)) & q[C0_VALID]) & p.CPU) & (!bool((p.address >> 0) & 1))) & (!bool((p.address >> 1) & 1))) & (!bool((p.address >> 2) & 1))) & (!bool((p.address >> 4) & 1))) & (!bool((p.address >> 7) & 1))));
   // G17.Q1 COMB
   next[EVENT_DECODE] = ((((((((((((bool((p.address >> 4) & 1) & bool((p.address >> 5) & 1)) & bool((p.address >> 7) & 1)) & q[C0_VALID]) & (!bool((p.address >> 3) & 1))) & (!bool((p.address >> 6) & 1))) | ((((((bool((p.address >> 1) & 1) & bool((p.address >> 2) & 1)) & bool((p.address >> 4) & 1)) & bool((p.address >> 6) & 1)) & q[C0_VALID]) & (!bool((p.address >> 5) & 1))) & (!bool((p.address >> 7) & 1)))) | (((((((bool((p.address >> 1) & 1) & bool((p.address >> 2) & 1)) & bool((p.address >> 3) & 1)) & bool((p.address >> 4) & 1)) & bool((p.address >> 6) & 1)) & q[C0_VALID]) & (!bool((p.address >> 7) & 1))) & (!p.read))) | ((((((bool((p.address >> 4) & 1) & bool((p.address >> 6) & 1)) & q[C0_VALID]) & (!bool((p.address >> 2) & 1))) & (!bool((p.address >> 3) & 1))) & (!bool((p.address >> 5) & 1))) & (!bool((p.address >> 7) & 1)))) | ((((((bool((p.address >> 4) & 1) & bool((p.address >> 6) & 1)) & q[C0_VALID]) & (!bool((p.address >> 3) & 1))) & (!bool((p.address >> 5) & 1))) & (!bool((p.address >> 7) & 1))) & (!q[STORE80]))) | (((((((bool((p.address >> 2) & 1) & bool((p.address >> 3) & 1)) & q[C0_VALID]) & (!bool((p.address >> 4) & 1))) & (!bool((p.address >> 5) & 1))) & (!bool((p.address >> 6) & 1))) & (!bool((p.address >> 7) & 1))) & (!p.read))) | (((((((((bool((p.address >> 0) & 1) & bool((p.address >> 3) & 1)) & bool((p.address >> 5) & 1)) & q[C0_VALID]) & q[MODE_D3]) & (!bool((p.address >> 2) & 1))) & (!bool((p.address >> 4) & 1))) & (!bool((p.address >> 6) & 1))) & (!bool((p.address >> 7) & 1))) & (!p.read))) | ((((((((((bool((p.address >> 1) & 1) & bool((p.address >> 5) & 1)) & q[C0_VALID]) & q[MODE_D3]) & (!bool((p.address >> 0) & 1))) & (!bool((p.address >> 2) & 1))) & (!bool((p.address >> 3) & 1))) & (!bool((p.address >> 4) & 1))) & (!bool((p.address >> 6) & 1))) & (!bool((p.address >> 7) & 1))) & (!p.read)));
   // G17.Q2 COMB
   next[LOAD_RAMWRT] = ((((((((((bool((p.address >> 3) & 1) & bool((p.address >> 5) & 1)) & bool((p.address >> 6) & 1)) & q[C0_VALID]) & p.CPU) & (!bool((p.address >> 0) & 1))) & (!bool((p.address >> 1) & 1))) & (!bool((p.address >> 2) & 1))) & (!bool((p.address >> 4) & 1))) & (!bool((p.address >> 7) & 1))) | (((((((((bool((p.address >> 2) & 1) & q[C0_VALID]) & p.CPU) & (!bool((p.address >> 1) & 1))) & (!bool((p.address >> 3) & 1))) & (!bool((p.address >> 4) & 1))) & (!bool((p.address >> 5) & 1))) & (!bool((p.address >> 6) & 1))) & (!bool((p.address >> 7) & 1))) & (!p.read)));
   // G17.Q3 COMB
   next[LOAD_STORE80] = (((((((((q[C0_VALID] & p.CPU) & (!bool((p.address >> 1) & 1))) & (!bool((p.address >> 2) & 1))) & (!bool((p.address >> 3) & 1))) & (!bool((p.address >> 4) & 1))) & (!bool((p.address >> 5) & 1))) & (!bool((p.address >> 6) & 1))) & (!bool((p.address >> 7) & 1))) & (!p.read));
   if (next == q && next_clocks == clocks)
   {
    e.after = q; e.clocks_after = clocks; e.valid = true;
    return true;
   }
   q = next; clocks = next_clocks;
  }
  return false;
 }
};
#endif

// license:BSD-3-Clause
#pragma once
#include <cstdint>
namespace twgs_decoded {
// A scheduler deadline rounds up to the first representable attosecond;
// elapsed scheduler time rounds down to completed controller ticks. This
// pair cannot repeatedly schedule an event just before its own deadline.
template<uint64_t Frequency> struct time_base {
 static constexpr uint64_t ATTO = 1000000000000000000ULL;
 static uint64_t fraction(uint64_t ticks) noexcept {
  return (__uint128_t(ticks % Frequency) * ATTO + Frequency - 1) / Frequency;
 }
 static uint64_t ticks(uint64_t seconds,uint64_t attoseconds) noexcept {
  return seconds * Frequency + __uint128_t(attoseconds) * Frequency / ATTO;
 }
};
}

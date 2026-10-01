// license:BSD-3-Clause
#pragma once
#include <cstdint>
#include <limits>
#include <numeric>
namespace twgs_decoded {
// A scheduler deadline rounds up to the first representable attosecond;
// elapsed scheduler time rounds down to completed controller ticks. This
// pair cannot repeatedly schedule an event just before its own deadline.
template<uint64_t Frequency> struct time_base {
 static constexpr uint64_t ATTO = 1000000000000000000ULL;
 static constexpr uint64_t GCD = std::gcd(Frequency, ATTO);
 static constexpr uint64_t NUM = ATTO / GCD, DEN = Frequency / GCD;
 static_assert(NUM <= std::numeric_limits<uint64_t>::max() / DEN);
 static uint64_t fraction(uint64_t ticks) noexcept {
  const uint64_t t=ticks%Frequency, r=t%DEN;
  // Split before multiplication. This is the same rounded rational value,
  // but the reduced numerator/remainder fit in 64 bits (no 128-bit divide).
  return (t/DEN)*NUM + (r ? 1+(r*NUM-1)/DEN : 0);
 }
 static uint64_t ticks(uint64_t seconds,uint64_t attoseconds) noexcept {
  return seconds*Frequency + (attoseconds/NUM)*DEN + (attoseconds%NUM)*DEN/NUM;
 }
};
}

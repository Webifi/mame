// license:BSD-3-Clause
#pragma once
#include "twgs_logic.h"
#include <array>
namespace twgs_decoded {
// Physical pin levels. GAL register updates are simultaneous from old state.
// CPU_PH2 as GAL1.pin7 is the source audit's explicit board-binding hypothesis.
// No invented propagation time is embedded here: caller supplies ordered edges.
struct transition_tables {
 std::array<uint8_t,65536> g1{}, g2a{}, g2b{};
 transition_tables() noexcept {
  for (unsigned state=0; state<256; ++state)
   for (unsigned input=0; input<256; ++input) {
    const unsigned index=(state<<8)|input;
    g1[index]=gal1(state,input);g2a[index]=gal2a(state,input);g2b[index]=gal2b(state,input);
   }
 }
};
inline const transition_tables transitions;
struct controller {
 uint8_t q1=0xff, q2=0xff;
 bool gs=false, early=false, delayed=false, service=false, pause_sample=true, sync=true;
 bool r16_sample=false, r17_sample=false;
 bool read=true, miss=false, eligible=true, slow=false, pause=true;
 bool use_gal2a=false, pin7_card_phase=false;
 static bool bit(uint8_t q, unsigned pin) noexcept { return (q >> (pin-12)) & 1; }
 bool cpu() const noexcept { return bit(q2,16) ? bit(q2,18) : gs; }
 unsigned capture_column() const noexcept { return bit(q1,19) | (unsigned(bit(q1,18)) << 1); }
 unsigned output_column() const noexcept {
  return (!gs && service) ? (!r17_sample | (unsigned(r16_sample)<<1)) : (bit(q1,16) | (unsigned(bit(q1,17))<<1));
 }
 void gs_edge(bool level) noexcept {
  if (gs && !level) {
   if (pause) { service=bit(q1,12); sync=bit(q2,19); }
   pause_sample=pause;
  }
  gs=level;
  if (level) { r16_sample=bit(q1,16); r17_sample=bit(q1,17); }
 }
 void early_edge() noexcept { delayed=early; early=gs; }
 void late_edge() noexcept {
  const bool qualifier=pin7_card_phase ? bit(q2,18) : cpu();
  const uint8_t i1=unsigned(bit(q2,19)) | (unsigned(read)<<1) | (unsigned(!early && delayed)<<2) | (unsigned(service)<<3) | (unsigned(pause_sample)<<4) | (unsigned(qualifier)<<5) | (unsigned(bit(q2,13))<<6) | (unsigned(bit(q2,12))<<7);
  const uint8_t i2=unsigned(miss) | (unsigned(eligible)<<1) | (unsigned(read)<<2) | (unsigned(bit(q1,15))<<3) | (unsigned(bit(q1,14))<<4) | (unsigned(pause)<<5) | (unsigned(slow)<<6) | (unsigned(early)<<7);
  const uint8_t n1=transitions.g1[(unsigned(q1)<<8)|i1];
  const uint8_t n2=(use_gal2a ? transitions.g2a : transitions.g2b)[(unsigned(q2)<<8)|i2];
  q1=n1; q2=n2;
 }
};
}

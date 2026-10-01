// license:BSD-3-Clause
#pragma once
#include "twgs_card.h"
#include <cstddef>
#include <cstring>
// Table sizes (log2 entries). A table at half load is cleared and refilled;
// the tests build with small sizes to exercise that path.
#ifndef TWGS_X4_MEMO_BITS
#define TWGS_X4_MEMO_BITS 15
#endif
#ifndef TWGS_GS_MEMO_BITS
#define TWGS_GS_MEMO_BITS 14
#endif
namespace twgs_decoded {
// The packed keys below use this byte layout. Keep layout changes from
// silently altering the memo's inputs; non-little-endian hosts use the oracle.
static_assert(sizeof(bool)==1 && sizeof(controller)==17);
static_assert(offsetof(controller,q1)==0 && offsetof(controller,r16_sample)==8 && offsetof(controller,use_gal2a)==15);
static_assert(offsetof(card,cpu)==offsetof(card,comparator_enable)+1);
static_assert(offsetof(card,we)==offsetof(card,comparator_enable)+2);
static_assert(offsetof(card,forced_write)==offsetof(card,comparator_enable)+3);
// Between two GS edges only the card's X4 EARLY/LATE edges run, and the
// transfer word is fixed (the owner supplies the next address). Their
// recurrence is then a pure function of a few controller and card bits; the
// SRAM enters only as "this word's tag matches" (raw) and every SRAM write
// stores the same word. x4_memo caches, per such state, the chain of up to K
// LATE edges (each but the first preceded by its EARLY edge): the state after
// each, and the latches, SRAM writes, WE changes and CPU completion so far.
// x4_model repeats card::late_edge/observe_cpu/memory_settle/transparent
// step by step on that state, so each cached chain is the per-edge result.
// gs_memo does the same for the discrete part of card::gs_edge.
struct x4_model {
 controller ctl;
 bool cpu=false,we=false,forced_write=false,ce=false,raw=false,ce_next=false;
 uint8_t dmask=0,bmask=0;
 bool before=false,after=false,done=false,q2_13=false;
 unsigned strobe=0;
 void settle() noexcept {
  const bool n=forced_write || (controller::bit(ctl.q2,14) && ctl.cpu() && ((controller::bit(ctl.q2,19)&&!ctl.read)||!controller::bit(ctl.q2,16)));
  if(n){raw=true;(done?after:before)=true;}
  if(n!=we)++strobe;
  we=n;ctl.miss=!(ce&&raw);
 }
 void transparent() noexcept {
  if(!controller::bit(ctl.q2,15)){const uint8_t m=uint8_t(1u<<ctl.capture_column());dmask&=~m;bmask&=~m;(cpu?dmask:bmask)|=m;}
 }
 void late() noexcept {
  settle();transparent();ctl.late_edge();
  settle();const bool n=ctl.cpu();
  if(cpu&&!n){done=true;q2_13=controller::bit(ctl.q2,13);ce=ce_next;}
  cpu=n;settle();transparent();settle();
 }
 void early() noexcept {ctl.early_edge();}
};
// Keys. Word A is the controller's first eight bytes (q1, q2, gs, early,
// delayed, service, pause_sample, sync); word B its next eight (r16, r17,
// read, miss, eligible, slow, pause, use_gal2a) with the miss byte replaced
// by the card bits: comparator enable, cpu, we, forced write (bits 0-3, the
// card's byte order in memory), pin7 (4), raw tag match (5), next
// comparator enable (6), GS level (7, GS edges only). Each memo masks out the
// bytes its edges neither read nor write.
struct memo_key {
 static uint64_t card_bits(uint32_t card4) noexcept {return ((card4*0x01020408u)>>24)&15;}
 static void load(x4_model &m,uint64_t a,uint64_t b) noexcept {
  uint8_t x[16];std::memcpy(x,&a,8);std::memcpy(x+8,&b,8);
  controller &c=m.ctl;
  c.q1=x[0];c.q2=x[1];c.gs=x[2];c.early=x[3];c.delayed=x[4];c.service=x[5];c.pause_sample=x[6];c.sync=x[7];
  c.r16_sample=x[8];c.r17_sample=x[9];c.read=x[10];c.miss=false;c.eligible=x[12];c.slow=x[13];c.pause=x[14];c.use_gal2a=x[15];
  const unsigned k=x[11];
  c.pin7_card_phase=(k>>4)&1;
  m.ce=k&1;m.cpu=(k>>1)&1;m.we=(k>>2)&1;m.forced_write=(k>>3)&1;m.raw=(k>>5)&1;m.ce_next=(k>>6)&1;
 }
 // The table index is the top bits; one multiply mixes both words.
 static uint64_t hash(uint64_t a,uint64_t b) noexcept {return (a^(b>>13)^(b<<23))*0x9E3779B97F4A7C15ULL;}
 static uint32_t pack_card(x4_model const&m) noexcept {return uint32_t(m.ce)|(uint32_t(m.cpu)<<8)|(uint32_t(m.we)<<16)|(uint32_t(m.forced_write)<<24);}
};
struct x4_memo {
 static constexpr unsigned K=8,BITS=TWGS_X4_MEMO_BITS,SIZE=1u<<BITS,LIMIT=SIZE/2;
 // A: q1..pause_sample (sync is not read). B: read, card bits, eligible,
 // slow, pause, use_gal2a (r16/r17 are not read by EARLY/LATE edges).
 static constexpr uint64_t MASK_A=0x00ffffffffffffffULL, MASK_B=0xffffffff7fff0000ULL;
 // Controller bytes an X4 edge can change: q1, q2, early, delayed.
 static constexpr uint64_t CHANGED_A=0x000000ffff00ffffULL;
 enum : uint32_t {SRAM_BEFORE=1u<<8,SRAM_AFTER=1u<<9,COMPLETE=1u<<10,Q2_13=1u<<11};
 // abits = q1 | q2<<8 | early<<16 | delayed<<24; card4 = the four card bytes;
 // eff: bits 0-3 columns captured with the data byte, 4-7 with the bank
 // byte, flags above, WE changes from bit 16.
 struct step {uint32_t abits, card4, eff; uint8_t miss, raw;};
 // One cache line per entry: the key, the chain's length, its last
 // (completing or fixed-point) step, and its full step list in chains.
 struct alignas(64) entry {uint64_t a=~0ULL,b=0; uint8_t n=0,complete=0,fixed=0; uint32_t chain=0; step last{};};
 struct chain_steps {step s[K];};
 static uint64_t expand(uint32_t abits) noexcept {return (abits&0xffffu)|(uint64_t(abits>>16)<<24);}
 entry table[SIZE];
 chain_steps chains[LIMIT];
 unsigned used=0;
 uint64_t built=0;
 void build(entry &e,uint64_t a,uint64_t b) noexcept {
  ++built;
  x4_model m;memo_key::load(m,a,b);
  e.a=a;e.b=b;e.n=e.complete=e.fixed=0;
  step *steps=chains[e.chain].s;
  for(unsigned j=0;j<K;++j){
   const unsigned strobe=m.strobe;
   if(j)m.early();
   m.late();
   step &s=steps[j];
   s.abits=uint32_t(m.ctl.q1)|(uint32_t(m.ctl.q2)<<8)|(uint32_t(m.ctl.early)<<16)|(uint32_t(m.ctl.delayed)<<24);
   s.card4=memo_key::pack_card(m);s.miss=m.ctl.miss;s.raw=m.raw;
   s.eff=uint32_t(m.dmask)|(uint32_t(m.bmask)<<4)|(m.before?SRAM_BEFORE:0)|(m.after?SRAM_AFTER:0)|(m.done?COMPLETE:0)|(m.q2_13?Q2_13:0)|(m.strobe<<16);
   e.n=uint8_t(j+1);e.last=s;
   if(m.done){e.complete=uint8_t(j+1);return;}
   // Fixed point: the step left every input of the next EARLY/LATE pair
   // unchanged and toggled no WE, so all later pairs repeat it exactly.
   if(j&&s.abits==steps[j-1].abits&&s.card4==steps[j-1].card4&&s.raw==steps[j-1].raw&&s.miss==steps[j-1].miss&&m.strobe==strobe){e.fixed=uint8_t(j+1);return;}
  }
 }
 // Step j (1-based, j<=n) of the entry's chain.
 step const& at(entry const&e,unsigned j) const noexcept {return j==e.n?e.last:chains[e.chain].s[j-1];}
 // Open addressing, linear probing. Entries are never evicted one by one;
 // a full table is cleared, which only costs rebuilding the chains in use.
 TWGS_COLD entry const& insert(unsigned i,uint64_t a,uint64_t b) noexcept {
  if(used>=LIMIT){for(auto &x:table)x.a=~0ULL;used=0;return get(a,b);}
  entry &e=table[i];e.chain=used++;build(e,a,b);return e;
 }
 entry const& get(uint64_t a,uint64_t b) noexcept {
  for(unsigned i=unsigned(memo_key::hash(a,b)>>(64-BITS));;i=(i+1)&(SIZE-1)){
   entry const&e=table[i];
   if(e.a==a&&e.b==b)return e;
   if(e.a==~0ULL)return insert(i,a,b);
  }
 }
};
// The discrete part of card::gs_edge: everything but the payload moves of
// the active/bank-candidate latches, the delivery checks and the bus. The
// latch reads happen between the first and the second transparent capture.
struct gs_memo {
 static constexpr unsigned BITS=TWGS_GS_MEMO_BITS,SIZE=1u<<BITS,LIMIT=SIZE/2;
 // A without early/delayed (GS edges neither read nor change them).
 static constexpr uint64_t MASK_A=0xffffff0000ffffffULL, MASK_B=~0ULL;
 // Controller bytes a GS edge can change: gs, service, pause_sample, sync
 // in A; r16, r17 in B.
 static constexpr uint64_t CHANGED_A=0xffffff0000ff0000ULL, CHANGED_B=0x000000000000ffffULL;
 enum : uint32_t {PRE=1u<<3,POST=1u<<7,SRAM_BEFORE=1u<<8,SRAM_AFTER=1u<<9,COMPLETE=1u<<10,Q2_13=1u<<11,SELECT=1u<<12};
 // eff: bits 0-1 first capture column, bit 2 its kind (1=data), PRE; bits
 // 4-6 likewise for the second capture, POST; bits 13-14 selected column;
 // WE changes from bit 16.
 struct alignas(64) entry {uint64_t a=~0ULL,b=0,new_a=0,new_b=0; uint32_t card4=0,eff=0; uint8_t miss=0,raw=0;};
 entry table[SIZE];
 unsigned used=0;
 uint64_t built=0;
 void build(entry &e,uint64_t a,uint64_t b) noexcept {
  ++built;
  x4_model m;memo_key::load(m,a,b);
  const bool level=(b>>31)&1;
  uint32_t eff=0;
  auto capture=[&](unsigned shift){if(!controller::bit(m.ctl.q2,15))eff|=(m.ctl.capture_column()|(unsigned(m.cpu)<<2)|8u)<<shift;};
  m.settle();capture(0);
  if(!level){m.ctl.gs_edge(false);if(m.ctl.service)eff|=SELECT|(m.ctl.output_column()<<13);}
  else{m.ctl.gs_edge(true);if((m.ctl.service&&m.ctl.sync)||(!m.ctl.sync&&!m.ctl.read))eff|=SELECT|(m.ctl.output_column()<<13);}
  m.settle();const bool n=m.ctl.cpu();
  if(m.cpu&&!n){m.done=true;m.q2_13=controller::bit(m.ctl.q2,13);m.ce=m.ce_next;}
  m.cpu=n;m.settle();capture(4);m.settle();
  eff|=(m.before?SRAM_BEFORE:0)|(m.after?SRAM_AFTER:0)|(m.done?COMPLETE:0)|(m.q2_13?Q2_13:0)|(m.strobe<<16);
  uint64_t na,nb;std::memcpy(&na,&m.ctl.q1,8);std::memcpy(&nb,&m.ctl.r16_sample,8);
  e.a=a;e.b=b;e.new_a=na&CHANGED_A;e.new_b=nb&CHANGED_B;
  e.card4=memo_key::pack_card(m);e.eff=eff;e.miss=m.ctl.miss;e.raw=m.raw;
 }
 TWGS_COLD entry const& insert(unsigned i,uint64_t a,uint64_t b) noexcept {
  if(used>=LIMIT){for(auto &x:table)x.a=~0ULL;used=0;return get(a,b);}
  ++used;entry &e=table[i];build(e,a,b);return e;
 }
 entry const& get(uint64_t a,uint64_t b) noexcept {
  for(unsigned i=unsigned(memo_key::hash(a,b)>>(64-BITS));;i=(i+1)&(SIZE-1)){
   entry const&e=table[i];
   if(e.a==a&&e.b==b)return e;
   if(e.a==~0ULL)return insert(i,a,b);
  }
 }
};
// Process-wide caches. They hold only values derived from their keys, so
// they need no saving and survive any state load. The kernel is used from
// one thread.
inline x4_memo x4_cache;
inline gs_memo gs_cache;
}

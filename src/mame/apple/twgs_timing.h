// license:BSD-3-Clause
#pragma once
#include "twgs_card.h"
#include "twgs_edge_table.h"
#include <cstring>
#include <algorithm>
#include <cstdint>
#include <bit>
namespace twgs_decoded {
struct hit_jump {
 struct entry {uint8_t q1=0,bank=0,data=0;bool valid=false;};
 std::array<entry,1024> states{};
 hit_jump() noexcept {
  for(unsigned state=0;state<1024;++state){
   controller c;c.q1=state&255;c.q2=0xbf;c.gs=c.early=c.delayed=state&256;c.service=state&512;
   c.read=true;c.miss=false;c.eligible=true;c.slow=false;c.pause=true;
   entry e;bool cpu=false,ok=true;unsigned falls=0;
   auto capture=[&](){if(!controller::bit(c.q2,15)){const uint8_t m=1<<c.capture_column();e.bank&=~m;e.data&=~m;(cpu?e.data:e.bank)|=m;}};
   for(unsigned j=0;j<4;++j){capture();c.late_edge();const bool n=c.cpu();if(cpu&&!n){++falls;ok&=j==3;}
    ok&=controller::bit(c.q2,16);cpu=n;capture();
   }
   e.q1=c.q1;e.valid=ok&&falls==1&&c.q2==0xbf;states[state]=e;
  }
 }
};
inline const hit_jump composed_hit;
// Motherboard timing inputs follow Apple IIgs TN68. The card's clock-source
// selection and CPU completion are exclusively the GAL recurrence in card.h.
// Units are the existing motherboard model's 1/15,120,000,000 second ticks.
template<uint64_t Scale> struct timing_scaled {
 static constexpr uint64_t CLK=1056*Scale, FAST=5*CLK, MEGA=14*CLK, LINE=912*CLK;
 uint64_t now=0, early=0, late=Scale, gs_event=2*CLK, gs_start=0, x4=270*Scale;
 uint64_t slot_start=0, slot_end=0, access_slot_start=0, access_slot_end=0;
 uint64_t board_cycles=0, mega_cycles=0, refresh_cycles=0;
 uint8_t speed=128,shadow=0;
 bool next_gs_level=true;
 bool had_bus=false;
 bool fast_hits=true;
 struct observed_bus {
  uint8_t data;
  void read(transaction &w,uint64_t,uint64_t) const noexcept {w.data=data;}
  void write(transaction const&,uint64_t,uint64_t) const noexcept {}
 };
 // The unspecified propagation interval is an explicit phase parameter.
 // Default EARLY then LATE at the same ideal instant, ordered by one tick.
 void start(uint64_t t,unsigned late_phase=Scale) noexcept {
  now=t;early=(t/x4+1)*x4;late=early+late_phase;
  gs_start=(t/FAST)*FAST;gs_event=gs_start+2*CLK;
  if(gs_event<=t){c.ctl.gs=true;next_gs_level=false;gs_event=gs_start+FAST;}
 }
 unsigned bus_class(transaction const&w) const noexcept {
  const auto a=w.address;const unsigned b=a>>16,l=a&65535;const bool wr=!w.read();
  if(!(speed&128))return 3;
  const bool io_bank=b==0xe0||b==0xe1||(b<2&&!cache_decode::pad(c.fp,36));
  if(b==0xe0||b==0xe1)return 3;
  if(io_bank&&l>=0xc000&&l<0xd000){
   if(l==0xc035||l==0xc036||l==0xc037||(w.read()&&(l==0xc02d||l==0xc068)))return 2;
   return 3;
  }
  if(wr&&(b<2||((speed&16)&&b<0xe0))){
   // The FPI performs shadowing on the original CPU bank, not an E1 tag.
   const bool aux=(b&1)||!bool(c.pins.tag&1);
   if(l>=0x400&&l<0x800&&!(shadow&1))return 3;
   if(l>=0x800&&l<0xc00&&!(shadow&32))return 3;
   if(l>=0x2000&&l<0x4000&&((!(shadow&2)&&(!aux||!(shadow&16)))||(aux&&!(shadow&8))))return 3;
   if(l>=0x4000&&l<0x6000&&((!(shadow&4)&&(!aux||!(shadow&16)))||(aux&&!(shadow&8))))return 3;
   if(l>=0x6000&&l<0xa000&&aux&&!(shadow&8))return 3;
  }
  if(b>=0xf0||(io_bank&&l>=0xd000&&w.read()&&!cache_decode::pad(c.fp,17)))return 1;
  return 0;
 }
 uint64_t end_of_cycle(uint64_t t,unsigned cls) noexcept {
  if(cls==3){
   const uint64_t line=t/LINE*LINE,pos=t-line,k=(pos+MEGA-1)/MEGA;
   ++mega_cycles;
   if(pos<=64*MEGA)return line+k*MEGA+(k==64?16*CLK:MEGA);
   return line+LINE+MEGA;
  }
  uint64_t end=t+FAST;
  if(cls==0){const uint64_t r=t/(50*CLK)*(50*CLK);if(t<r+FAST){end+=r+FAST-t;++refresh_cycles;}else if(end>r+50*CLK){end=r+50*CLK+2*FAST;++refresh_cycles;}}
  return end;
 }
 template<class Next,class Bus> void edge(Next&&next,Bus&&bus) noexcept {
  // GS wins equal-time ties, then EARLY, then LATE, as the reference model.
  if(gs_event<=early&&gs_event<=late){
   now=gs_event;
   if(next_gs_level){
    c.gs_edge(true,next);
    const transaction &w=c.active.valid?c.active.bus:c.word;
    const bool external=(c.ctl.service&&c.ctl.sync)||!c.ctl.sync;
    const unsigned cls=external?bus_class(w):0;
    slot_start=gs_start;slot_end=end_of_cycle(gs_start,cls);gs_event=slot_end;++board_cycles;
    if(external&&!c.ctl.sync){had_bus=true;access_slot_start=slot_start;access_slot_end=slot_end;}
   }else{
    // Data is sampled at the actual motherboard falling edge. The SRAM
    // write window and registered CPU-side logic see that byte before the
    // same edge can complete the CPU transfer. Buffered stores reach the
    // motherboard only when their physical output latch finishes service.
    if(c.active.valid&&!c.active.bus.read())bus.write(c.active.bus,slot_start,now);
    if(!c.ctl.sync&&c.word.read())bus.read(c.word,slot_start,now);
    c.gs_edge(false,next);gs_start=now;gs_event=now+2*CLK;
   }
   next_gs_level=!next_gs_level;
  }else if(early<=late){now=early;early+=x4;c.early_edge();}
  else{now=late;late+=x4;c.late_edge(next);}
 }
 template<class Next> void edge(Next&&next) noexcept {edge(next,observed_bus{c.word.data});}
 // ceil(d/x4) for d>0 without a divide: x4_rcp is floor((2^64-1)/x4), so
 // the product quotient is low by at most one.
 uint64_t x4_rcp=0,x4_rcp_for=0;
 TWGS_COLD uint64_t x4_periods(uint64_t d) noexcept {
  if(x4!=x4_rcp_for){x4_rcp=~0ULL/x4;x4_rcp_for=x4;}
  const uint64_t n=d-1;uint64_t q=uint64_t((__uint128_t(n)*x4_rcp)>>64);
  if((q+1)*x4<=n)++q;
  return q+1;
 }
 // Card bits of the memo keys (see memo_key): CE, cpu, we, forced write,
 // the word's raw tag match, plus the per-transfer pin7 and next-CE bits.
 uint64_t card_byte(uint64_t fixed) const noexcept {
  uint32_t card4;std::memcpy(&card4,&c.comparator_enable,4);
  return memo_key::card_bits(card4)|(uint64_t(c.raw_match)<<5)|fixed;
 }
 void capture(unsigned col,bool data) noexcept {
  c.latches[col]={c.word,data?c.word.data:uint8_t(c.word.address>>16),true};
 }
 TWGS_COLD void clear_tags_8k() noexcept {
  for(unsigned i=0;i<=c.mask;++i)c.tags[i]&=255;
  const unsigned idx=c.word.address&c.mask;c.raw_match=c.valid[idx]&&c.tags[idx]==c.pins.tag;
 }
 TWGS_COLD void clock_fpga_full() noexcept {c.fpga.clock_full(c.word.address,c.word.read(),c.receipt,c.pins.bank_io,c.pins.irq);}
 TWGS_COLD void sample_equations() noexcept {c.decode.sample_data(c.pins,c.receipt);}
 // card::observe_cpu at the CPU's falling edge, with the external advance:
 // receipt, FPGA and GAL3 data registers, completion count.
 [[gnu::always_inline]] void complete_transfer(bool q2_13,bool ce_next) noexcept {
  const unsigned idx=c.word.address&c.mask;
  const bool read=c.word.read();
  c.receipt=c.word.data;c.from_sram=read&&q2_13&&c.valid[idx];
  if(c.from_sram)c.receipt=c.bytes[idx];
  // The comparator enable is already ce_next: the memo's card bytes carry it.
  if(!ce_next&&c.mask==8191)clear_tags_8k();
  const uint32_t ad=c.word.address;
  if((ad&0xff8000)!=0xbc0000&&(!c.pins.bank_io||(ad&0xff00)!=0xc000))c.fpga.q|=1u<<5;
  else clock_fpga_full();
  auto &d=c.decode;
  if(d.use_tables){
   const uint32_t reg=(15u<<17)|(d.gal3_revision==2?(1u<<22):0);
   d.g3=(d.g3&~reg)|(uint32_t(g3_transitions.data[d.gal3_revision][c.sample_row|c.receipt])<<15);
  }else sample_equations();
  c.completed=true;++c.completions;
 }
 // Applies a cached chain of m LATE edges (and the m-1 EARLY edges between
 // them, plus a leading EARLY edge if Early), all before bound: the next GS
 // edge, or the first tick after advance_until's deadline. key_b holds this
 // transfer's bytes of key word B, fixed its pin7 and next-CE card bits.
 // Returns true on CPU completion.
 template<bool Early> [[gnu::always_inline]] bool x4_span(uint64_t key_b,uint64_t fixed,uint64_t bound) noexcept {
  uint64_t a;std::memcpy(&a,&c.ctl.q1,8);
  if(Early){
   // EARLY edge: delayed=early, early=gs (bytes 4, 3, 2 of the word).
   a=(a&~(0xffffULL<<24))|(((a>>16)&0xffff)<<24);
   early+=x4;
  }
  auto const&e=x4_cache.get(a&x4_memo::MASK_A,key_b|(card_byte(fixed)<<24));
  const uint64_t d=bound-late;
  uint64_t m;x4_memo::step const*s;
  if(e.complete&&uint64_t(e.complete-1)*x4<d){m=e.complete;s=&e.last;}
  else{
   // LATE edges before the GS edge: ceil(d/x4), counted for short distances.
   uint64_t nl;
   if(d<=x4_memo::K*x4){nl=1;for(uint64_t t=x4;t<d;t+=x4)++nl;}
   else nl=x4_periods(d);
   if(e.fixed){m=nl;s=&x4_cache.at(e,unsigned(nl<e.fixed?nl:e.fixed));}
   else{m=nl<e.n?nl:e.n;s=&x4_cache.at(e,unsigned(m));}
  }
  a=(a&~x4_memo::CHANGED_A)|x4_memo::expand(s->abits);std::memcpy(&c.ctl.q1,&a,8);
  c.ctl.miss=s->miss;std::memcpy(&c.comparator_enable,&s->card4,4);
  const uint32_t eff=s->eff;
  for(uint32_t cap=(eff|(eff>>4))&15;cap;cap&=cap-1){const unsigned col=__builtin_ctz(cap);capture(col,(eff>>col)&1);}
  if(eff&x4_memo::SRAM_BEFORE)c.write_sram();
  c.strobe_changes+=eff>>16;
  now=late+(m-1)*x4;late+=m*x4;early+=(m-1)*x4;c.delivered=false;
  if(!(eff&x4_memo::COMPLETE)){c.completed=false;return false;}
  complete_transfer(eff&x4_memo::Q2_13,(fixed>>6)&1);
  if(eff&x4_memo::SRAM_AFTER)c.write_sram();
  return true;
 }
 // One GS edge, as edge() with card::gs_edge; its discrete part from gs_memo.
 template<class Bus> void gs_step(Bus&&bus,uint64_t key_b,uint64_t fixed) noexcept {
  now=gs_event;
  const bool level=next_gs_level;
  if(!level){
   if(c.active.valid&&!c.active.bus.read())bus.write(c.active.bus,slot_start,now);
   if(!c.ctl.sync&&c.word.read())bus.read(c.word,slot_start,now);
  }
  uint64_t a;std::memcpy(&a,&c.ctl.q1,8);
  const uint64_t b=key_b|uint64_t(c.ctl.r16_sample)|(uint64_t(c.ctl.r17_sample)<<8)|((card_byte(fixed)|(uint64_t(level)<<7))<<24);
  auto const&e=gs_cache.get(a&gs_memo::MASK_A,b);
  const uint32_t eff=e.eff;
  c.completed=c.delivered=false;
  if(eff&gs_memo::PRE)capture(eff&3,eff&4);
  if(!level){
   if(c.active.valid&&!c.active.bus.read()){
    if(!(c.active==c.latches[c.active_column]))++c.high_errors;
    c.retired=c.active;c.delivered=true;++c.deliveries;
   }
   c.active.valid=false;
  }
  a=(a&~gs_memo::CHANGED_A)|e.new_a;std::memcpy(&c.ctl.q1,&a,8);
  c.ctl.r16_sample=e.new_b&1;c.ctl.r17_sample=(e.new_b>>8)&1;
  c.ctl.miss=e.miss;std::memcpy(&c.comparator_enable,&e.card4,4);
  if(eff&gs_memo::SELECT){
   const unsigned col=(eff>>13)&3;
   if(!level){c.bank_column=col;c.bank_candidate=c.latches[col];c.bank_valid=true;}
   else{
    c.active_column=col;c.active=c.latches[col];
    if(c.ctl.sync&&c.active.valid&&!c.active.bus.read()&&(!c.bank_valid||c.bank_column!=c.active_column||c.bank_candidate.bus.address>>16!=c.active.bus.address>>16))++c.bank_errors;
   }
  }
  if(eff&gs_memo::SRAM_BEFORE)c.write_sram();
  if(eff&gs_memo::COMPLETE)complete_transfer(eff&gs_memo::Q2_13,(fixed>>6)&1);
  if(eff&gs_memo::POST)capture((eff>>4)&3,eff&64);
  if(eff&gs_memo::SRAM_AFTER)c.write_sram();
  c.strobe_changes+=eff>>16;
  if(level){
   const transaction &w=c.active.valid?c.active.bus:c.word;
   const bool external=(c.ctl.service&&c.ctl.sync)||!c.ctl.sync;
   const unsigned cls=external?bus_class(w):0;
   slot_start=gs_start;slot_end=end_of_cycle(gs_start,cls);gs_event=slot_end;++board_cycles;
   if(external&&!c.ctl.sync){had_bus=true;access_slot_start=slot_start;access_slot_end=slot_end;}
  }else{gs_start=now;gs_event=now+2*CLK;}
  next_gs_level=!level;
 }
 // Same edges, same order and the same results as access_reference: GS
 // edges and each run of EARLY/LATE edges before the next GS edge are
 // applied from gs_memo and x4_memo, which the per-edge recurrence fills.
 template<class Bus> uint64_t access(transaction const&w,Bus&&bus) noexcept {
  if(!fast_hits||std::endian::native!=std::endian::little)return access_reference(w,bus);
  c.external_advance=true;c.setword_inline<true>(w);c.completed=false;had_bus=false;
  // Word B bytes of this transfer, from the fields just stored byte-wise.
  const uint64_t key_b=(uint64_t(c.ctl.read)<<16)|(uint64_t(c.ctl.eligible)<<32)|(uint64_t(c.ctl.slow)<<40)|(uint64_t(c.ctl.pause)<<48)|(uint64_t(c.ctl.use_gal2a)<<56);
  const uint64_t fixed=(uint64_t(c.ctl.pin7_card_phase)<<4)|(uint64_t(cache_decode::pad(c.fp,53))<<6);
  for(;;){
   if(gs_event<=early&&gs_event<=late){gs_step(bus,key_b,fixed);if(c.completed)return now;}
   else if(early<=late){
    // The cached chains assume each EARLY edge falls within one X4 period
    // before its LATE edge, as start() sets up; other phases go edge by edge.
    if(late<gs_event&&late-early<x4){if(x4_span<true>(key_b,fixed,gs_event))return now;}
    else{now=early;early+=x4;c.early_edge();}
   }
   else if(early-late>x4){auto hold=[](transaction const&a,uint8_t){return a;};now=late;late+=x4;c.late_edge(hold);if(c.completed)return now;}
   else if(x4_span<false>(key_b,fixed,gs_event))return now;
  }
 }
 // Reference path through the card's edge methods. With fast_hits clear,
 // access and advance_until use individual edges; with it set, this path
 // can also compose cache-hit transitions and skip fixed points below.
 // Memoized execution must retain the same card state and bus events.
 template<class Bus> uint64_t access_reference(transaction w,Bus&&bus) noexcept {
  c.external_advance=true;c.setword_reference(w);c.completed=false;had_bus=false;
  // A cached transfer gets its driven data from the physical SRAM, not
  // from a second motherboard lookup. Supplied trace data is merely input
  // for the observed_bus adapter used by the standalone reference tests.
  if(w.read()&&c.match()){c.word.data=c.bytes[w.address&c.mask];w.data=c.word.data;}
  auto hold=[](transaction const&a,uint8_t){return a;};
  // No GS edge or U60 change can occur in this interval. The four exact
  // GAL transitions below can therefore be composed without scheduling
  // eight individual clock callbacks. Keep their order and intermediate
  // latch/SRAM effects equivalent to edge-by-edge execution.
  const bool idle_board=!c.ctl.service&&!(c.ctl.q1&5)&&!c.active.valid;
  if(fast_hits&&w.read()&&c.pins.eligible&&!c.ctl.slow&&!c.ctl.miss&&
    c.ctl.pause&&c.ctl.pause_sample&&c.ctl.sync&&!c.forced_write&&c.ctl.q2==0xbf&&
    ((c.ctl.early==c.ctl.gs&&c.ctl.delayed==c.ctl.gs&&gs_event>late+3*x4)||idle_board)){
   const auto &jump=composed_hit.states[c.ctl.q1|(unsigned(c.ctl.gs)<<8)|(unsigned(c.ctl.service)<<9)];
   if(jump.valid){
    const uint64_t end=late+3*x4;
    // With no service, active write, or SERVICE/H flag, GS changes cannot
    // change this qualified read's four GAL transitions. Keep the sampled
    // U60 history and U23/U45 state at their real deadlines nevertheless.
    const uint64_t last_early=early<=end?early+((end-early)/x4)*x4:early-x4;
    bool sample=c.ctl.gs,previous=last_early>=early+x4?c.ctl.gs:c.ctl.early;
    while(gs_event<=end){
     const uint64_t boundary=gs_event;
     if(boundary<=last_early)sample=next_gs_level;
     if(last_early>=early+x4&&boundary<=last_early-x4)previous=next_gs_level;
     if(next_gs_level){c.ctl.gs_edge(true);slot_start=gs_start;slot_end=end_of_cycle(gs_start,0);gs_event=slot_end;++board_cycles;}
     else{c.ctl.gs_edge(false);gs_start=gs_event;gs_event+=2*CLK;}
     next_gs_level=!next_gs_level;
    }
    if(early<=end){
     c.ctl.early=sample;
     c.ctl.delayed=previous;
    }
    c.ctl.q1=jump.q1;
    for(unsigned col=0;col<4;++col){if(jump.data&(1<<col))c.latches[col]={w,w.data,true};else if(jump.bank&(1<<col))c.latches[col]={w,uint8_t(w.address>>16),true};}
    now=end;late+=4*x4;if(early<=now)early+=((now-early)/x4+1)*x4;
    c.cpu=true;c.observe_cpu(hold);return now;
   }
  }
  do {
   const bool was_late=late<early&&late<gs_event;
   const uint8_t old_q1=c.ctl.q1,old_q2=c.ctl.q2;
   edge(hold,bus);
   // After a LATE edge reaches a fixed point, with both U60 samples equal
   // to GS, further EARLY/LATE edges have identical inputs and outputs.
   // SRAM and transparent latches have already settled on that edge. No
   // CPU completion or motherboard event is removed: stop strictly before
   // the next GS transition and retain both clock phases at that instant.
   if(fast_hits&&was_late&&!c.completed&&c.ctl.q1==old_q1&&c.ctl.q2==old_q2&&
      c.ctl.early==c.ctl.gs&&c.ctl.delayed==c.ctl.gs){
    if(early<gs_event){const uint64_t last=early+(gs_event-1-early)/x4*x4;now=std::max(now,last);early=last+x4;}
    if(late<gs_event){const uint64_t last=late+(gs_event-1-late)/x4*x4;now=std::max(now,last);late=last+x4;}
   }
  }while(!c.completed);
  return now;
 }
 uint64_t access(transaction w) noexcept {return access(w,observed_bus{w.data});}
 // The held bus after a transfer: the same edges as edge() up to and
 // including the deadline, through the same caches as access. CPU
 // completions of the held word are repeated exactly as observe_cpu does.
 template<class Bus> void advance_until(uint64_t deadline,Bus&&bus) noexcept {
  auto hold=[](transaction const&a,uint8_t){return a;};
  if(!fast_hits||!c.external_advance||std::endian::native!=std::endian::little){
   while(std::min(gs_event,std::min(early,late))<=deadline)edge(hold,bus);
   return;
  }
  {const unsigned i=c.word.address&c.mask;c.raw_match=c.valid[i]&&c.tags[i]==c.pins.tag;}
  const uint64_t key_b=(uint64_t(c.ctl.read)<<16)|(uint64_t(c.ctl.eligible)<<32)|(uint64_t(c.ctl.slow)<<40)|(uint64_t(c.ctl.pause)<<48)|(uint64_t(c.ctl.use_gal2a)<<56);
  const uint64_t fixed=(uint64_t(c.ctl.pin7_card_phase)<<4)|(uint64_t(cache_decode::pad(c.fp,53))<<6);
  const uint64_t after=deadline==~0ULL?~0ULL:deadline+1;
  while(std::min(gs_event,std::min(early,late))<=deadline){
   const uint64_t bound=std::min(gs_event,after);
   if(gs_event<=early&&gs_event<=late)gs_step(bus,key_b,fixed);
   else if(early<=late){
    if(late<bound&&late-early<x4)x4_span<true>(key_b,fixed,bound);
    else{now=early;early+=x4;c.early_edge();}
   }
   else if(early-late>x4){now=late;late+=x4;c.late_edge(hold);}
   else x4_span<false>(key_b,fixed,bound);
  }
 }
 // Last, so that the timing fields above sit at short offsets.
 card c;
};
using timing=timing_scaled<1>;
}

// license:BSD-3-Clause
#pragma once
#include "twgs_card.h"
#include <algorithm>
#include <cstdint>
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
 card c;
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
 template<class Bus> uint64_t access(transaction w,Bus&&bus) noexcept {
  c.external_advance=true;c.setword(w);c.completed=false;had_bus=false;
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
  do{edge(hold,bus);}while(!c.completed);
  return now;
 }
 uint64_t access(transaction w) noexcept {return access(w,observed_bus{w.data});}
 template<class Bus> void advance_until(uint64_t deadline,Bus&&bus) noexcept {
  auto hold=[](transaction const&a,uint8_t){return a;};
  while(std::min(gs_event,std::min(early,late))<=deadline)edge(hold,bus);
 }
};
using timing=timing_scaled<1>;
}

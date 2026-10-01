// license:BSD-3-Clause
#pragma once
#include "twgs_controller.h"
#include "twgs_cache.h"
#include <array>
#include <cstdint>
namespace twgs_decoded {
struct transaction {
 uint32_t address=0;
 uint64_t id=0;
 uint8_t data=0, type=2;
 bool seed=false;
 bool read() const noexcept {return type!=3&&type!=6;}
};
struct payload {
 transaction bus{};
 uint8_t byte=0;
 bool valid=false;
 bool operator==(payload const &b) const noexcept {return valid==b.valid && (!valid || (bus.id==b.bus.id && bus.address==b.bus.address && byte==b.byte));}
};
// Edge kernel, independent of the emulator and of the motherboard clock source.
// The owner supplies GS, EARLY, LATE in time order and the data observed on reads.
// This version covers the normal memory circuit audited by CacheExperiment:
// external PAUSE and GAL8 forced-write inputs remain explicit.
struct card {
 controller ctl;
 cache_decode decode;
 fpga_copy fpga;
 std::array<uint16_t,32768> tags{};
 std::array<uint8_t,32768> bytes{};
 std::array<bool,32768> valid{};
 std::array<payload,4> latches{};
 payload active{},bank_candidate{},retired{};
 unsigned active_column=0,bank_column=0;
 uint32_t mask=32767;
 transaction word{};
 decoded_access pins{};
 uint64_t fp=0;
 bool comparator_enable=true,cpu=true,we=false,forced_write=false,bank_valid=false;
 uint64_t completions=0,deliveries=0,strobe_changes=0,bank_errors=0,high_errors=0;
 bool completed=false,delivered=false,from_sram=false;
 bool external_advance=false; // owner supplies next address before the next edge
 uint8_t receipt=0;
 static bool bit(uint8_t q,unsigned pin) noexcept {return controller::bit(q,pin);}
 bool match() const noexcept {return comparator_enable && valid[word.address&mask] && tags[word.address&mask]==pins.tag;}
 void setword(transaction w) noexcept {
  word=w;ctl.read=w.read();
  fp=fpga.pads(cache_decode::bit(decode.g3,18));
  pins=decode.access(w.address,w.read(),w.type!=1&&w.type!=5&&w.type!=6,w.type<2||w.type==7,w.type!=4,fp,mask==32767);
  // IRQ permission is GAL3.18, not the core's I flag. The pads' slow output
  // is the only projected combinational output dependent on that pin.
  fp=fpga.pads(pins.irq);
  ctl.slow=cache_decode::pad(fp,49)||!pins.fast_instruction;
  ctl.eligible=pins.eligible;
  if(w.seed)write_sram();
  ctl.miss=!match();
 }
 void write_sram() noexcept {const unsigned i=word.address&mask;valid[i]=true;tags[i]=pins.tag;bytes[i]=word.data;}
 void memory_settle() noexcept {
  const bool n=forced_write || (bit(ctl.q2,14) && ctl.cpu() && ((bit(ctl.q2,19)&&!ctl.read)||!bit(ctl.q2,16)));
  if(n)write_sram();
  if(n!=we)++strobe_changes;
  we=n;ctl.miss=!match();
 }
 void transparent() noexcept {
  if(!bit(ctl.q2,15))latches[ctl.capture_column()]={word,uint8_t(cpu?word.data:word.address>>16),true};
 }
 template<class Next> void observe_cpu(Next &&next) noexcept {
  memory_settle();const bool n=ctl.cpu();
  if(cpu&&!n) {
   receipt=word.data;from_sram=word.read()&&bit(ctl.q2,13)&&valid[word.address&mask];
   if(from_sram)receipt=bytes[word.address&mask];
   comparator_enable=cache_decode::pad(fp,53);
   if(!comparator_enable&&mask==8191)for(unsigned i=0;i<=mask;++i)tags[i]&=255;
   fpga.clock(word.address,word.read(),receipt,pins.bank_io,pins.irq);
   decode.sample_data(pins,receipt);
   completed=true;++completions;
   if(!external_advance)setword(next(word,receipt));
  }
  cpu=n;memory_settle();
 }
 template<class Next> void gs_edge(bool level,Next &&next) noexcept {
  completed=delivered=false;memory_settle();transparent();
  if(!level){
   if(active.valid&&!active.bus.read()){
    if(!(active==latches[active_column]))++high_errors;
    retired=active;delivered=true;++deliveries;
   }
   active.valid=false;
   ctl.gs_edge(false);
   if(ctl.service){bank_column=ctl.output_column();bank_candidate=latches[bank_column];bank_valid=true;}
  }else{
   ctl.gs_edge(true);
   if((ctl.service&&ctl.sync)||(!ctl.sync&&!ctl.read)){
    active_column=ctl.output_column();active=latches[active_column];
    if(ctl.sync&&active.valid&&!active.bus.read()&&(!bank_valid||bank_column!=active_column||bank_candidate.bus.address>>16!=active.bus.address>>16))++bank_errors;
   }
  }
  observe_cpu(next);transparent();memory_settle();
 }
 void early_edge() noexcept {completed=delivered=false;ctl.early_edge();}
 template<class Next> void late_edge(Next &&next) noexcept {
  completed=delivered=false;memory_settle();transparent();ctl.late_edge();observe_cpu(next);transparent();memory_settle();
 }
};
}

// license:BSD-3-Clause
#pragma once
#include "twgs_controller.h"
#include "twgs_cache.h"
#include <array>
#include <cstdint>
namespace twgs_decoded {
// One CPU bus cycle. type uses the g65816 BUS_* numbering: opcode, operand,
// read, write, vector, internal read, internal write, interrupt. id follows
// the cycle through the latches so a buffered write can be identified later.
struct transaction {
 uint32_t address=0;
 uint64_t id=0;
 uint8_t data=0, type=2;
 bool seed=false;
 bool read() const noexcept {return type!=3&&type!=6;}
};
// A transparent latch can hold either the bank byte or the data byte of
// its transaction. Keep that captured byte separate from bus.data.
struct payload {
 transaction bus{};
 uint8_t byte=0;
 bool valid=false;
 bool operator==(payload const &b) const noexcept {return valid==b.valid && (!valid || (bus.id==b.bus.id && bus.address==b.bus.address && byte==b.byte));}
};
// Card state advanced by ordered GS, EARLY and LATE clock edges. The caller
// supplies motherboard read data and the external PAUSE/forced-write inputs.
// CPU completion and buffered-write delivery are separate events: observe_cpu
// completes the CPU cycle, while gs_edge retires the motherboard write latch.
struct card {
 controller ctl;
 cache_decode decode;
 fpga_copy fpga;
 std::array<payload,4> latches{};
 // active is the motherboard cycle in progress; bank_candidate is its
 // preceding bank-address phase. A finished write is exposed as retired.
 payload active{},bank_candidate{},retired{};
 unsigned active_column=0,bank_column=0;
 uint32_t mask=32767;
 transaction word{};
 decoded_access pins{};
 uint64_t fp=0;
 bool comparator_enable=true,cpu=true,we=false,forced_write=false,bank_valid=false;
 uint64_t completions=0,deliveries=0,strobe_changes=0,bank_errors=0,high_errors=0;
 // completed/delivered are cleared by each edge handler. CPU completion
 // can precede delivery for a buffered write; the counters above accumulate.
 bool completed=false,delivered=false,from_sram=false;
 bool external_advance=false; // owner supplies next address before the next edge
 uint8_t receipt=0;
 // FPGA output pads for both IRQ levels of the current register state. The
 // pads are a function of fpga.q and IRQ only; q changes only at completion.
 uint64_t pads_q=~0ULL; // never equal to a 32-bit q: empty
 uint64_t pads_v[2]{};
 TWGS_COLD void refresh_pads() noexcept {pads_v[0]=fpga.pads(false);pads_v[1]=fpga.pads(true);pads_q=fpga.q;}
 uint64_t pads(bool irq) noexcept {
  if(fpga.q!=pads_q)refresh_pads();
  return pads_v[irq];
 }
 static bool bit(uint8_t q,unsigned pin) noexcept {return controller::bit(q,pin);}
 bool match() const noexcept {return comparator_enable && valid[word.address&mask] && tags[word.address&mask]==pins.tag;}
 // Decode through the GAL equations. setword_inline falls back here for
 // transfer types above 7 or when decode.use_tables is false; otherwise
 // it must produce the same pins, register state and cache-match result.
 TWGS_COLD void setword_reference(transaction w) noexcept {
  word=w;ctl.read=w.read();
  fp=fpga.pads(cache_decode::bit(decode.g3,18));
  pins=decode.access(w.address,w.read(),w.type!=1&&w.type!=5&&w.type!=6,w.type<2||w.type==7,w.type!=4,fp,mask==32767);
  // IRQ permission is GAL3.18, not the core's I flag. The pads' slow output
  // is the only projected combinational output dependent on that pin.
  fp=fpga.pads(pins.irq);
  ctl.slow=cache_decode::pad(fp,49)||!pins.fast_instruction;
  ctl.eligible=pins.eligible;
  if(w.seed)write_sram();
  const unsigned i=word.address&mask;
  raw_match=valid[i]&&tags[i]==pins.tag;
  ctl.miss=!(comparator_enable&&raw_match);
  const uint32_t p=pins.gal3_pins;
  sample_row=((((p>>17)&15)|((p>>18)&16))<<11)|((((p>>2)&3)|((p>>12)&4))<<8);
 }
 // Row of the GAL3 data-register table this transfer samples at completion
 // (cache_decode::sample_data's state and class bits of pins.gal3_pins).
 uint32_t sample_row=0;
 // Transfer type -> read, VDA, VPA, VPB (bits 0-3), as cache_decode::access
 // receives them from setword_reference. Types above 7 take the reference.
 static constexpr uint8_t type_bits[8]={0xf,0xd,0xb,0xa,0x3,0x9,0x8,0xf};
 // setword with the GAL3 table decode written out, the ordinary-bank decode
 // inline and the I/O-bank GAL4/5/7 decode through io_cache; the equation
 // GAL3 mode takes setword_reference.
 void setword(transaction w) noexcept {setword_inline<false>(w);}
 // Access: also replace the word's data by the SRAM byte when the read hits
 // (timing::access does this right after setword).
 template<bool Access=false> [[gnu::always_inline]] void setword_inline(transaction const&w) noexcept {
  const uint32_t g3=decode.g3;
  const uint64_t fp1=pads((g3>>18)&1);
  const unsigned bank=w.address>>16,type=w.type;
  if(type>7||!decode.use_tables){
   setword_reference(w);
   if(Access&&ctl.read&&comparator_enable&&raw_match)word.data=bytes[word.address&mask];
   return;
  }
  word=w;
  const unsigned tb=type_bits[type];
  const bool read=tb&1,vpb=(tb>>3)&1;
  const unsigned cls=tb>>1;
  ctl.read=read;
  const unsigned state=((g3>>17)&15)|((g3>>18)&16);
  const unsigned gn=g3_transitions.bank[decode.gal3_revision][(state<<11)|(cls<<8)|bank];
  decode.g3=gn<<15;
  const unsigned a=w.address;
  const bool irq=(gn>>3)&1,fast=(gn>>7)&1;
  const uint32_t i3=g3_inputs.pins[(cls<<8)|bank]|(gn<<15);
  uint16_t tag;bool eligible;
  if(bank==0xe0||bank==0xe1||(bank<2&&!cache_decode::pad(fp1,36))){
   pins=decode.access_io_cached(a,read,vpb,fp1,mask==32767,i3);
   tag=pins.tag;eligible=pins.eligible;
  }else{
   const bool h=(gn>>6)&1,i=(gn>>1)&1,j=gn&1;
   const bool reject=!vpb||(!read&&!i&&j)||(!cache_decode::pad(fp1,46)&&i&&!j);
   eligible=!(reject||(!h&&i&&j));
   const unsigned tag2=(mask==32767?((a>>15)&1)|6:((a>>15)&1)|((a>>13)&2)|((a>>11)&4))|0xd8|(unsigned(!reject)<<5);
   decode.g4|=(1u<<15)|(1u<<18)|(1u<<22);
   tag=uint16_t((bank^1)|(tag2<<8));
   pins={tag,eligible,false,irq,fast,i3};
  }
  fp=pads(irq);
  ctl.slow=cache_decode::pad(fp,49)||!fast;
  ctl.eligible=eligible;
  if(w.seed)write_sram();
  const unsigned k=a&mask;
  const bool raw=valid[k]&&tags[k]==tag;
  raw_match=raw;
  const bool ce=comparator_enable;
  ctl.miss=!(ce&&raw);
  sample_row=((((gn>>2)&15)|((gn>>3)&16))<<11)|(cls<<8);
  if(Access&&read&&ce&&raw)word.data=bytes[k];
 }
 // This word's SRAM cell holds a valid matching tag (match() without the
 // comparator enable). Set by setword and write_sram, the only writers of
 // the word's cell during a transfer apart from the 8 KB tag clear below.
 bool raw_match=false;
 void write_sram() noexcept {const unsigned i=word.address&mask;valid[i]=true;tags[i]=pins.tag;bytes[i]=word.data;raw_match=true;}
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
   if(!comparator_enable&&mask==8191){for(unsigned i=0;i<=mask;++i)tags[i]&=255;raw_match=valid[word.address&mask]&&tags[word.address&mask]==pins.tag;}
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
 // The 128 KB of SRAM arrays come last, so the controller and transfer
 // fields above sit at short offsets from the card's base address.
 std::array<uint16_t,32768> tags{};
 std::array<uint8_t,32768> bytes{};
 std::array<bool,32768> valid{};
};
}

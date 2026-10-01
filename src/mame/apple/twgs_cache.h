// license:BSD-3-Clause
#pragma once
#include "twgs_cache_logic.h"
#include <array>
#include <initializer_list>
// Rare paths kept out of the hot functions' register allocation: with
// preserve_most the caller saves nothing around the call.
#ifndef TWGS_COLD
#if defined(__clang__)
#define TWGS_COLD [[gnu::noinline,gnu::cold]] __attribute__((preserve_most))
#else
#define TWGS_COLD [[gnu::noinline,gnu::cold]]
#endif
#endif
namespace twgs_decoded {
struct gal3_tables {
 std::array<std::array<uint8_t,65536>,3> bank{},data{};
 gal3_tables() noexcept {
  constexpr unsigned map[8]={10,11,8,9,6,7,4,5};
  for(unsigned rev=0;rev<3;++rev)for(unsigned key=0;key<65536;++key){
   const unsigned old=key>>11,cls=(key>>8)&7,byte=key&255;
   uint32_t pins=0xffffff;
   auto put=[&](unsigned p,bool v){pins=(pins&~(1u<<p))|(uint32_t(v)<<p);};
   put(2,cls&1);put(3,cls&2);put(13,false);put(14,cls&4);
   for(unsigned b=0;b<8;++b)put(map[b],(byte>>b)&1);
   for(unsigned b=0;b<4;++b)put(17+b,(old>>b)&1);put(22,old&16);
   auto eval=[&](uint32_t p){return rev==0?gal3b(p):rev==1?gal3c(p):gal3e(p);};
   constexpr uint32_t out=0x7f8000;
   const uint32_t reg=(15u<<17)|(rev==2?(1u<<22):0),saved=pins&reg;
   data[rev][key]=(eval(pins)&reg)>>15;
   for(unsigned j=0;j<8;++j){const uint32_t n=(eval(pins)&(out&~reg))|saved;if(n==(pins&out))break;pins=(pins&~out)|n;}
   bank[rev][key]=(pins&out)>>15;
  }
 }
};
inline const gal3_tables g3_transitions;
// GAL3 input pins of one access before its registered outputs are merged:
// VDA, VPA, VPB and the bank byte, as cache_decode::access places them.
struct gal3_input_table {
 std::array<uint32_t,2048> pins{};
 gal3_input_table() noexcept {
  constexpr unsigned map[8]={10,11,8,9,6,7,4,5};
  for(unsigned cls=0;cls<8;++cls)for(unsigned bank=0;bank<256;++bank){
   uint32_t p=0xffffff;
   auto put=[&](unsigned n,bool v){p=(p&~(1u<<n))|(uint32_t(v)<<n);};
   put(2,cls&1);put(3,cls&2);put(13,false);put(14,cls&4);
   for(unsigned b=0;b<8;++b)put(map[b],(bank>>b)&1);
   pins[(cls<<8)|bank]=p&~0x7f8000u;
  }
 }
};
inline const gal3_input_table g3_inputs;
struct decoded_access {
 uint16_t tag;
 bool eligible, bank_io, irq, fast_instruction;
 uint32_t gal3_pins;
};
// Results of cache_decode::access_io (GAL4/GAL5/GAL7 settling) per input:
// the FPGA pads word and the address bits, read, VPB, cache size, GAL4
// registered outputs, GAL3 H/I/J and the GAL4/GAL5 programming variants.
// The settled GAL4 pin word is stored too: it is the next call's state.
struct io_memo {
 static constexpr unsigned BITS=12,SIZE=1u<<BITS,LIMIT=SIZE/2;
 struct entry {uint64_t fp=0,key=~0ULL; uint32_t g4=0; uint16_t tag=0; bool eligible=false,bank_io=false;};
 entry table[SIZE];
 unsigned used=0;
 uint64_t built=0;
};
inline io_memo io_cache;
struct cache_decode {
 uint32_t g3=0x7f8000, g4=0xffffff;
 unsigned gal3_revision=2; // 0=B, 1=C, 2=E; hardware programming revision, not speed grade.
 bool gal4_b=true, gal5_permuted=true;
 bool use_tables=true;
 static constexpr unsigned bank_pin[8]={10,11,8,9,6,7,4,5};
 static uint32_t put(uint32_t p,unsigned bit,bool value) noexcept {return (p&~(1u<<bit))|(uint32_t(value)<<bit);}
 static bool bit(uint32_t p,unsigned n) noexcept {return (p>>n)&1;}
 static bool pad(uint64_t p,unsigned n) noexcept {return (p>>n)&1;}
 uint32_t eval3(uint32_t p) const noexcept {return gal3_revision==0 ? gal3b(p) : gal3_revision==1 ? gal3c(p) : gal3e(p);}
 [[gnu::always_inline]] decoded_access access(uint32_t address, bool read, bool vda, bool vpa, bool vpb, uint64_t fp, bool cache32) noexcept {
  const unsigned bank=address>>16;
  constexpr uint32_t g3out=0x7f8000;
  const unsigned cls=unsigned(vda)|(unsigned(vpa)<<1)|(unsigned(vpb)<<2);
  uint32_t i3=g3_inputs.pins[(cls<<8)|bank];
  if(use_tables){
   const unsigned state=((g3>>17)&15)|((g3>>18)&16);
   g3=uint32_t(g3_transitions.bank[gal3_revision][(state<<11)|(cls<<8)|bank])<<15;
   i3|=g3;
  }else{
   const uint32_t g3reg=(15u<<17)|(gal3_revision==2 ? (1u<<22) : 0);
   i3|=g3&g3out;
   for(unsigned j=0;j<8;++j){const auto n=(eval3(i3)&(g3out&~g3reg))|(g3&g3reg);if(n==g3)break;g3=n;i3=(i3&~g3out)|g3;}
  }
  const bool io_bank = bank==0xe0 || bank==0xe1 || (bank<2 && !pad(fp,36));
  if (!io_bank) {
   // Fuse reduction: every GAL4 AD0/AD1 rejection product requires BANK_IO=0.
   // Every GAL5 BL0X product except bank bit 0 also requires BANK_IO=0.
   const bool h=bit(g3,21), i=bit(g3,16), j=bit(g3,15);
   const bool reject=!vpb || (!read && !i && j) || (!pad(fp,46) && i && !j);
   const bool eligible=!(reject || (!h && i && j));
   const unsigned tag2=((address>>15)&1)|(unsigned(cache32||((address>>14)&1))<<1)|(unsigned(cache32||((address>>13)&1))<<2)|0xd8|(unsigned(!reject)<<5);
   g4 |= (1u<<15)|(1u<<18)|(1u<<22);
   return {uint16_t((bank^1)|(tag2<<8)),eligible,false,bit(g3,18),bit(g3,22),i3};
  }
  return access_io(address,read,vpb,fp,cache32,i3);
 }
 // access_io through io_cache: identical results and the same g4.
 decoded_access access_io_cached(uint32_t address, bool read, bool vpb, uint64_t fp, bool cache32, uint32_t i3) noexcept {
  const uint64_t key=(address>>8)|(uint64_t(read)<<16)|(uint64_t(vpb)<<17)|(uint64_t(cache32)<<18)|(uint64_t(bit(g4,15))<<19)|(uint64_t(bit(g4,18))<<20)|(uint64_t(bit(g4,22))<<21)
   |(uint64_t(bit(g3,15))<<22)|(uint64_t(bit(g3,16))<<23)|(uint64_t(bit(g3,21))<<24)|(uint64_t(gal4_b)<<25)|(uint64_t(gal5_permuted)<<26);
  const uint64_t h=(fp*0x9E3779B97F4A7C15ULL)^(key*0xC2B2AE3D27D4EB4FULL);
  for(unsigned i=unsigned(h>>(64-io_memo::BITS));;i=(i+1)&(io_memo::SIZE-1)){
   auto &e=io_cache.table[i];
   if(e.key==key&&e.fp==fp){g4=e.g4;return {e.tag,e.eligible,e.bank_io,bit(g3,18),bit(g3,22),i3};}
   if(e.key==~0ULL)return access_io_insert(i,key,address,read,vpb,fp,cache32,i3);
  }
 }
 TWGS_COLD decoded_access access_io_insert(unsigned i,uint64_t key,uint32_t address, bool read, bool vpb, uint64_t fp, bool cache32, uint32_t i3) noexcept {
  if(io_cache.used>=io_memo::LIMIT){for(auto &x:io_cache.table)x.key=~0ULL;io_cache.used=0;i=unsigned(((fp*0x9E3779B97F4A7C15ULL)^(key*0xC2B2AE3D27D4EB4FULL))>>(64-io_memo::BITS));}
  const decoded_access d=access_io(address,read,vpb,fp,cache32,i3);
  ++io_cache.used;++io_cache.built;
  io_cache.table[i]={fp,key,g4,d.tag,d.eligible,d.bank_io};
  return d;
 }
 // I/O banks (E0, E1, and 00/01 while the FPGA maps them): GAL4, GAL5, GAL7.
 [[gnu::noinline]] decoded_access access_io(uint32_t address, bool read, bool vpb, uint64_t fp, bool cache32, uint32_t i3) noexcept {
  const unsigned bank=address>>16;
  uint32_t i4=0xffffff;
  for(unsigned b=2;b<8;++b)i4=put(i4,bank_pin[b],(bank>>b)&1);
  i4=put(i4,10,(bank>>1)&1);i4=put(i4,16,bank&1);
  for(unsigned n: {1u,2u,20u,21u})i4=put(i4,n,(address>>(n==1?15:n==2?14:n==20?12:13))&1);
  i4=put(i4,3,pad(fp,17));i4=put(i4,11,pad(fp,36));i4=put(i4,13,pad(fp,39));i4=put(i4,14,pad(fp,57));i4=put(i4,17,pad(fp,32));i4=put(i4,19,read);
  constexpr uint32_t g4out=(1u<<15)|(1u<<18)|(1u<<22);
  i4=(i4&~g4out)|(g4&g4out);
  // Transparent bank latch in CPU low, held in high. Consensus products settle.
  for(unsigned low=0;low<2;++low){i4=put(i4,23,!low);for(unsigned j=0;j<4;++j){const auto n=(i4&~g4out)|((gal4_b?gal4b(i4):gal4a(i4))&g4out);if(n==i4)break;i4=n;}}
  g4=i4;
  uint32_t i5=0xffffff;
  for(unsigned p=1;p<=5;++p)i5=put(i5,p,(address>>(16-p))&1);
  i5=put(i5,6,(address>>(gal5_permuted?8:10))&1);i5=put(i5,7,(address>>(gal5_permuted?10:9))&1);i5=put(i5,8,(address>>(gal5_permuted?9:8))&1);
  i5=put(i5,9,pad(fp,37));i5=put(i5,10,pad(fp,34));i5=put(i5,11,pad(fp,33));i5=put(i5,13,pad(fp,13));i5=put(i5,14,bit(g4,18));i5=put(i5,16,bank&1);i5=put(i5,19,read);i5=put(i5,20,pad(fp,40));i5=put(i5,23,pad(fp,19));
  constexpr uint32_t g5out=(1u<<15)|(1u<<17)|(1u<<18)|(1u<<22);
  for(unsigned j=0;j<8;++j){const auto n=(i5&~g5out)|(gal5a(i5)&g5out);if(n==i5)break;i5=n;}
  const uint32_t g5=i5;
  uint32_t i7=0xffffff;
  i7=put(i7,1,bit(g5,22));i7=put(i7,2,pad(fp,21));i7=put(i7,3,read);i7=put(i7,4,pad(fp,46));i7=put(i7,5,bit(g4,18));i7=put(i7,6,pad(fp,23));i7=put(i7,7,vpb);i7=put(i7,8,bit(g3,21));i7=put(i7,9,bit(g3,16));i7=put(i7,11,bit(g3,15));
  for(unsigned p=14;p<=17;++p)i7=put(i7,p,(address>>(p-2))&1);
  i7=put(i7,18,false);const uint32_t g7=gal7a(i7);
  const unsigned tag1=(bank&254)|bit(g5,15);
  const unsigned tag2=((address>>15)&1)|(unsigned(cache32||((address>>14)&1))<<1)|(unsigned(cache32||((address>>13)&1))<<2)|(unsigned(bit(g4,15))<<3)|(unsigned(bit(g4,22))<<4)|(unsigned(bit(g7,12))<<5)|0xc0;
  return {uint16_t(tag1|(tag2<<8)),bit(g7,19),!bit(g4,18),bit(g3,18),bit(g3,22),i3};
 }
 void sample_data(decoded_access const &a,uint8_t data) noexcept {
  uint32_t pins=a.gal3_pins;
  const uint32_t reg=(15u<<17)|(gal3_revision==2 ? (1u<<22) : 0);
  if(use_tables){
   const unsigned state=((pins>>17)&15)|((pins>>18)&16),cls=((pins>>2)&3)|((pins>>12)&4);
   g3=(g3&~reg)|(uint32_t(g3_transitions.data[gal3_revision][(state<<11)|(cls<<8)|data])<<15);
  }else{
   for(unsigned b=0;b<8;++b)pins=put(pins,bank_pin[b],(data>>b)&1);
   g3=(g3&~reg)|(eval3(pins)&reg);
  }
 }
};
}

// license:BSD-3-Clause
#pragma once
#include "twgs_cache_logic.h"
#include <array>
#include <initializer_list>
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
struct decoded_access {
 uint16_t tag;
 bool eligible, bank_io, irq, fast_instruction;
 uint32_t gal3_pins;
};
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
 decoded_access access(uint32_t address, bool read, bool vda, bool vpa, bool vpb, uint64_t fp, bool cache32) noexcept {
  const unsigned bank=address>>16;
  uint32_t i3=0xffffff;
  i3=put(i3,2,vda);i3=put(i3,3,vpa);i3=put(i3,13,false);i3=put(i3,14,vpb);
  for(unsigned b=0;b<8;++b)i3=put(i3,bank_pin[b],(bank>>b)&1);
  constexpr uint32_t g3out=0x7f8000;
  const uint32_t g3reg=(15u<<17)|(gal3_revision==2 ? (1u<<22) : 0);
  i3=(i3&~g3out)|(g3&g3out);
  if(use_tables){
   const unsigned state=((g3>>17)&15)|((g3>>18)&16),cls=unsigned(vda)|(unsigned(vpa)<<1)|(unsigned(vpb)<<2);
   g3=uint32_t(g3_transitions.bank[gal3_revision][(state<<11)|(cls<<8)|bank])<<15;
   i3=(i3&~g3out)|g3;
  }else for(unsigned j=0;j<8;++j){const auto n=(eval3(i3)&(g3out&~g3reg))|(g3&g3reg);if(n==g3)break;g3=n;i3=(i3&~g3out)|g3;}
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

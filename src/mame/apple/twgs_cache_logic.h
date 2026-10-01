// license:BSD-3-Clause
// TransWarp GS cache GAL equations and FPGA configuration logic decoded
// from JED fuse maps and ROM truth tables. GAL masks use bit n for pin n;
// fpga_copy keeps register state in q and returns physical pad levels.
// These equations are the reference for the shortcuts in twgs_cache.h.
#pragma once
#include <cstdint>
namespace twgs_decoded {
inline uint32_t gal3b(uint32_t pins) noexcept {
 uint32_t out=0;
 out |= uint32_t(!(((pins & 0xa000cU)==0x0U) || ((pins & 0x40000cU)==0xcU) || ((pins & 0xc00000U)==0x800000U) || ((pins & 0x1a000cU)==0xa0000U) || ((pins & 0x5a000cU)==0xa0000U) || ((pins & 0x5a000cU)==0xa0004U))) << 22;
 out |= uint32_t(!(((pins & 0x800ffcU)==0x800bfcU) || ((pins & 0x8000b0U)==0x800030U) || ((pins & 0x80002cU)==0x800004U) || ((pins & 0x8000bcU)==0x800024U) || ((pins & 0x8003f0U)==0x8002e0U) || ((pins & 0x800ff0U)==0x800be0U) || ((pins & 0xa00000U)==0x0U))) << 21;
 out |= uint32_t(!(((pins & 0xffcU)==0x25cU) || ((pins & 0xffcU)==0x83cU) || ((pins & 0xa000cU)==0x8U) || ((pins & 0xffcU)==0xe5cU) || ((pins & 0xffcU)==0xe8cU) || ((pins & 0x1a000cU)==0xa0000U) || ((pins & 0x1a000cU)==0xa0004U))) << 20;
 out |= uint32_t(!(((pins & 0xf7cU)==0x25cU) || ((pins & 0xf7cU)==0x83cU) || ((pins & 0xa000cU)==0x8U))) << 19;
 out |= uint32_t(!(((pins & 0x1a400cU)==0x24000U) || ((pins & 0xa410cU)==0x84004U) || ((pins & 0xe410cU)==0x4008U) || ((pins & 0x1a410cU)==0x4108U) || ((pins & 0x4400cU)==0x400cU) || ((pins & 0xe4000U)==0xa4000U) || ((pins & 0x6400cU)==0x4000U))) << 18;
 out |= uint32_t(!(((pins & 0xf7cU)==0x83cU) || ((pins & 0xffcU)==0x1cU) || ((pins & 0xffcU)==0x28cU) || ((pins & 0xa000cU)==0x80000U) || ((pins & 0xa000cU)==0x8U))) << 17;
 out |= uint32_t(!(((pins & 0x8000f0U)==0x8000f0U) || ((pins & 0x8002f0U)==0x8002b0U) || ((pins & 0x8003f0U)==0x8001b0U) || ((pins & 0x800bf0U)==0x8008b0U) || ((pins & 0x8000f0U)==0x8000a0U) || ((pins & 0x8002f0U)==0x8000e0U) || ((pins & 0x810000U)==0x0U))) << 16;
 out |= uint32_t(!(((pins & 0x800ff0U)==0x8003e0U) || ((pins & 0x80002cU)==0x800004U) || ((pins & 0x8000bcU)==0x800024U) || ((pins & 0x800ff0U)==0x800be0U) || ((pins & 0x808000U)==0x0U))) << 15;
 return out;
}
inline uint32_t gal3c(uint32_t pins) noexcept {
 uint32_t out=0;
 out |= uint32_t(!(((pins & 0xa000cU)==0x0U) || ((pins & 0x40000cU)==0xcU) || ((pins & 0xc00000U)==0x800000U) || ((pins & 0x1a000cU)==0xa0000U) || ((pins & 0x5a000cU)==0xa0000U) || ((pins & 0x5a000cU)==0xa0004U))) << 22;
 out |= uint32_t(!(((pins & 0x800ffcU)==0x800bfcU) || ((pins & 0x8000b0U)==0x800030U) || ((pins & 0x80002cU)==0x800004U) || ((pins & 0x8000bcU)==0x800024U) || ((pins & 0x8003f0U)==0x8002e0U) || ((pins & 0x800ff0U)==0x800be0U) || ((pins & 0xa00000U)==0x0U))) << 21;
 out |= uint32_t(!(((pins & 0xffcU)==0x25cU) || ((pins & 0xffcU)==0x83cU) || ((pins & 0xa000cU)==0x8U) || ((pins & 0xffcU)==0xe5cU) || ((pins & 0xffcU)==0xe8cU) || ((pins & 0x1a000cU)==0xa0000U) || ((pins & 0x1a000cU)==0xa0004U))) << 20;
 out |= uint32_t(!(((pins & 0xf7cU)==0x25cU) || ((pins & 0xf7cU)==0x83cU) || ((pins & 0xa000cU)==0x8U))) << 19;
 out |= uint32_t(!(((pins & 0x1a400cU)==0x24000U) || ((pins & 0xa410cU)==0x84004U) || ((pins & 0xe410cU)==0x4008U) || ((pins & 0x1a410cU)==0x4108U) || ((pins & 0x4400cU)==0x400cU) || ((pins & 0xe4000U)==0xa4000U) || ((pins & 0x6400cU)==0x4000U))) << 18;
 out |= uint32_t(!(((pins & 0xf7cU)==0x83cU) || ((pins & 0xffcU)==0x1cU) || ((pins & 0xffcU)==0x28cU) || ((pins & 0xa000cU)==0x80000U) || ((pins & 0xa000cU)==0x8U))) << 17;
 out |= uint32_t(!(((pins & 0x8000f0U)==0x8000f0U) || ((pins & 0x8002f0U)==0x8002b0U) || ((pins & 0x8003f0U)==0x8001b0U) || ((pins & 0x800bf0U)==0x8008b0U) || ((pins & 0x8000f0U)==0x8000a0U) || ((pins & 0x8002f0U)==0x8000e0U) || ((pins & 0x810000U)==0x0U))) << 16;
 out |= uint32_t(!(((pins & 0x800ff0U)==0x8003e0U) || ((pins & 0x80002cU)==0x800004U) || ((pins & 0x8000bcU)==0x800024U) || ((pins & 0x800ff0U)==0x800be0U) || ((pins & 0x808000U)==0x0U))) << 15;
 return out;
}
inline uint32_t gal3e(uint32_t pins) noexcept {
 uint32_t out=0;
 out |= uint32_t(!(((pins & 0xa000cU)==0x8U) || ((pins & 0x40000cU)==0x0U) || ((pins & 0xfecU)==0xe4cU) || ((pins & 0xffcU)==0xe8cU) || ((pins & 0x5a000cU)==0xa0000U) || ((pins & 0x5a000cU)==0xa0004U))) << 22;
 out |= uint32_t(!(((pins & 0x800ffcU)==0x800bfcU) || ((pins & 0x80002cU)==0x800004U) || ((pins & 0x800ff0U)==0x800be0U) || ((pins & 0xa00000U)==0x0U) || ((pins & 0x200ffcU)==0xbfcU) || ((pins & 0x20002cU)==0x4U) || ((pins & 0x200ff0U)==0xbe0U))) << 21;
 out |= uint32_t(!(((pins & 0xffcU)==0x25cU) || ((pins & 0xffcU)==0x83cU) || ((pins & 0xa000cU)==0x8U) || ((pins & 0xffcU)==0xe5cU) || ((pins & 0xffcU)==0xe8cU) || ((pins & 0x1a000cU)==0xa0000U) || ((pins & 0x1a000cU)==0xa0004U))) << 20;
 out |= uint32_t(!(((pins & 0xf7cU)==0x25cU) || ((pins & 0xf7cU)==0x83cU) || ((pins & 0xa000cU)==0x8U))) << 19;
 out |= uint32_t(!(((pins & 0x1a400cU)==0x24000U) || ((pins & 0xa410cU)==0x84004U) || ((pins & 0xe410cU)==0x4008U) || ((pins & 0x1a410cU)==0x4108U) || ((pins & 0x4400cU)==0x400cU) || ((pins & 0xe4000U)==0xa4000U) || ((pins & 0x6400cU)==0x4000U))) << 18;
 out |= uint32_t(!(((pins & 0xf7cU)==0x83cU) || ((pins & 0xffcU)==0x1cU) || ((pins & 0xffcU)==0x28cU) || ((pins & 0xa000cU)==0x80000U) || ((pins & 0xa000cU)==0x8U))) << 17;
 out |= uint32_t(!(((pins & 0x8000f0U)==0x8000f0U) || ((pins & 0x810000U)==0x0U) || ((pins & 0x100f0U)==0xf0U))) << 16;
 out |= uint32_t(!(((pins & 0x800ff0U)==0x8003e0U) || ((pins & 0x800ff0U)==0x800be0U) || ((pins & 0x80002cU)==0x800004U) || ((pins & 0x808000U)==0x0U) || ((pins & 0x8ff0U)==0x3e0U) || ((pins & 0x8ff0U)==0xbe0U) || ((pins & 0x802cU)==0x4U))) << 15;
 return out;
}
inline uint32_t gal4a(uint32_t pins) noexcept {
 uint32_t out=0;
 out |= uint32_t(!(((pins & 0xc200eU)==0x8000eU) || ((pins & 0x3c6006U)==0x100006U) || ((pins & 0x346006U)==0x102006U))) << 22;
 out |= uint32_t(!(false)) << 21;
 out |= uint32_t(!(false)) << 20;
 out |= uint32_t(!(false)) << 19;
 out |= uint32_t(!(((pins & 0x800ff0U)==0x800000U) || ((pins & 0x8007f0U)==0x8000b0U) || ((pins & 0x840000U)==0x0U))) << 18;
 out |= uint32_t(!(false)) << 17;
 out |= uint32_t(!(false)) << 16;
 out |= uint32_t(!(((pins & 0xc2006U)==0x80006U) || ((pins & 0x270006U)==0x230000U) || ((pins & 0x60006U)==0x20004U) || ((pins & 0x270006U)==0x30002U))) << 15;
 return out;
}
inline uint32_t gal4b(uint32_t pins) noexcept {
 uint32_t out=0;
 out |= uint32_t(!(((pins & 0xc200eU)==0x8000eU) || ((pins & 0x3c6006U)==0x100006U) || ((pins & 0x346006U)==0x102006U))) << 22;
 out |= uint32_t(!(false)) << 21;
 out |= uint32_t(!(false)) << 20;
 out |= uint32_t(!(false)) << 19;
 out |= uint32_t(!(((pins & 0x800ff0U)==0x800000U) || ((pins & 0x8007f0U)==0x8000b0U) || ((pins & 0x840000U)==0x0U) || ((pins & 0x40ff0U)==0x0U) || ((pins & 0x407f0U)==0xb0U))) << 18;
 out |= uint32_t(!(false)) << 17;
 out |= uint32_t(!(false)) << 16;
 out |= uint32_t(!(((pins & 0xc2006U)==0x80006U) || ((pins & 0x270006U)==0x230000U) || ((pins & 0x60006U)==0x20004U) || ((pins & 0x270006U)==0x30002U))) << 15;
 return out;
}
inline uint32_t gal5a(uint32_t pins) noexcept {
 uint32_t out=0;
 out |= uint32_t(!(((pins & 0x1feU)==0x6U))) << 22;
 out |= uint32_t(!(false)) << 21;
 out |= uint32_t(!(false)) << 20;
 out |= uint32_t(!(false)) << 19;
 out |= uint32_t(!(((pins & 0x1eU)==0x16U) || ((pins & 0xeU)==0xeU) || ((pins & 0x1beU)==0x0U) || ((pins & 0x8004beU)==0x800480U) || ((pins & 0x80240eU)==0x802408U))) << 18;
 out |= uint32_t(!(((pins & 0x1eU)==0x16U) || ((pins & 0xeU)==0xeU) || ((pins & 0x1beU)==0x0U) || ((pins & 0x8004beU)==0x800080U) || ((pins & 0x80240eU)==0x802008U) || ((pins & 0x1eU)==0x6U))) << 17;
 out |= uint32_t(!(false)) << 16;
 out |= uint32_t(!(((pins & 0xe4200U)==0xe0200U) || ((pins & 0x1e4000U)==0x160000U) || ((pins & 0x64800U)==0x800U) || ((pins & 0x64000U)==0x20000U) || ((pins & 0x10000U)==0x10000U))) << 15;
 return out;
}
inline uint32_t gal7a(uint32_t pins) noexcept {
 uint32_t out=0;
 out |= uint32_t(!(((pins & 0x3c068U)==0x38000U) || ((pins & 0x34068U)==0x34000U) || ((pins & 0x3c020U)==0x30000U) || ((pins & 0x80U)==0x0U) || ((pins & 0xa08U)==0x800U) || ((pins & 0xa10U)==0x200U) || ((pins & 0xb00U)==0xa00U))) << 19;
 out |= uint32_t(!(false)) << 18;
 out |= uint32_t(!(false)) << 17;
 out |= uint32_t(!(false)) << 16;
 out |= uint32_t(!(false)) << 15;
 out |= uint32_t(!(false)) << 14;
 out |= uint32_t(!(((pins & 0x1c000U)==0x0U))) << 13;
 out |= uint32_t(!(((pins & 0x3c068U)==0x38000U) || ((pins & 0x34068U)==0x34000U) || ((pins & 0x3c020U)==0x30000U) || ((pins & 0x80U)==0x0U) || ((pins & 0xa08U)==0x800U) || ((pins & 0xa10U)==0x200U) || ((pins & 0x40000U)==0x40000U))) << 12;
 return out;
}
struct fpga_copy {
 uint32_t q=0;
 bool bit(unsigned n) const noexcept { return (q>>n)&1; }
 void clock_full(uint32_t address, bool read, uint8_t data, bool bank_io, bool /*irq*/=true) noexcept {
 const bool AD_G=(0x11001100U >> ((unsigned(((address>>2)&1))<<0) | (unsigned(((address>>1)&1))<<1) | (unsigned(((address>>3)&1))<<3)))&1;
 const bool BB_G=(0x22002200U >> ((unsigned(AD_G)<<0) | (unsigned(!(bank_io && (address&0xff70)==0xc050))<<1) | (unsigned(((address>>7)&1))<<3)))&1;
 const bool AC_F=(0xcfcfc0c0U >> ((unsigned(((address>>0)&1))<<1) | (unsigned(BB_G)<<2) | (unsigned(bit(0))<<4)))&1;
 const bool AD_F=(0x00880088U >> ((unsigned(((address>>2)&1))<<0) | (unsigned(((address>>1)&1))<<1) | (unsigned(((address>>3)&1))<<3)))&1;
 const bool BB_F=(0x00330033U >> ((unsigned(!(bank_io && (address&0xff70)==0xc050))<<1) | (unsigned(((address>>7)&1))<<3)))&1;
 const bool BA_F=(0xf7f78080U >> ((unsigned(AD_F)<<0) | (unsigned(BB_F)<<1) | (unsigned(((address>>0)&1))<<2) | (unsigned(bit(1))<<4)))&1;
 const bool GA_G=(0x0a0a0a0aU >> ((unsigned(AD_G)<<0) | (unsigned(read)<<2)))&1;
 const bool HA_F=(0x50505050U >> ((unsigned(((address>>0)&1))<<0) | (unsigned(GA_G)<<2)))&1;
 const bool DB_F=(0x04040404U >> ((unsigned(((address>>7)&1))<<0) | (unsigned(HA_F)<<1) | (unsigned(!(bank_io && (address&0xff70)==0xc060))<<2)))&1;
 const bool CD_F=(0x00440044U >> ((unsigned(((address>>2)&1))<<0) | (unsigned(((address>>7)&1))<<1) | (unsigned(!(bank_io && (address&0xff70)==0xc000))<<3)))&1;
 const bool DF_G=(0xffaaffaaU >> ((unsigned(DB_F)<<0) | (unsigned(CD_F)<<3)))&1;
 const bool BH_M=(0x3bfb08c8U >> ((unsigned(((address>>3)&1))<<0) | (unsigned(DF_G)<<1) | (unsigned(DB_F)<<2) | (unsigned(((data>>2)&1))<<3) | (unsigned(bit(2))<<4)))&1;
 const bool CB_F=(0x8f8f8080U >> ((unsigned(((address>>0)&1))<<0) | (unsigned(bit(4))<<1) | (unsigned(CD_F)<<2) | (unsigned(bit(3))<<4)))&1;
 const bool CC_F=(0xafafa0a0U >> ((unsigned(((address>>0)&1))<<0) | (unsigned(CD_F)<<2) | (unsigned(bit(4))<<4)))&1;
 const bool AG_G=(0x00550055U >> ((unsigned(((address>>14)&1))<<0) | (unsigned(((address&0xff8000)!=0xbc0000))<<3)))&1;
 const bool CG_G=(0x0a0a0a0aU >> ((unsigned(AG_G)<<0) | (unsigned(read)<<2)))&1;
 const bool CH_F=(0xbbbbbbbbU >> ((unsigned(((data>>0)&1))<<0) | (unsigned(CG_G)<<1)))&1;
 const bool GC_F=(0x00330033U >> ((unsigned(((address>>7)&1))<<1) | (unsigned(!(bank_io && (address&0xff70)==0xc020))<<3)))&1;
 const bool CA_F=(0xfcf0fcf0U >> ((unsigned(GC_F)<<1) | (unsigned(DB_F)<<2) | (unsigned(HA_F)<<3)))&1;
 const bool DA_M=(0xb3b3c4c4U >> ((unsigned(DB_F)<<0) | (unsigned(CA_F)<<1) | (unsigned(((data>>1)&1))<<2) | (unsigned(bit(6))<<4)))&1;
 const bool CE_G=(0x20202020U >> ((unsigned(((address>>3)&1))<<0) | (unsigned(((address>>1)&1))<<1) | (unsigned(((address>>2)&1))<<2)))&1;
 const bool CF_F=(0x00800080U >> ((unsigned(((address>>0)&1))<<0) | (unsigned(CE_G)<<1) | (unsigned(GC_F)<<2) | (unsigned(read)<<3)))&1;
 const bool DF_F=(0xf3f3c0c0U >> ((unsigned(CF_F)<<1) | (unsigned(((data>>6)&1))<<2) | (unsigned(bit(7))<<4)))&1;
 const bool DG_F=(0xdddd8888U >> ((unsigned(CG_G)<<0) | (unsigned(((data>>2)&1))<<1) | (unsigned(bit(8))<<4)))&1;
 const bool DH_F=(0xdddd8888U >> ((unsigned(CG_G)<<0) | (unsigned(((data>>1)&1))<<1) | (unsigned(bit(9))<<4)))&1;
 const bool BE_F=(0xfefefefeU >> ((unsigned(((address>>3)&1))<<0) | (unsigned(((address>>2)&1))<<1) | (unsigned(((address>>1)&1))<<2)))&1;
 const bool CD_G=(0x00330033U >> ((unsigned(((address>>7)&1))<<1) | (unsigned(!(bank_io && (address&0xff70)==0xc000))<<3)))&1;
 const bool DC_F=(0x03000300U >> ((unsigned(read)<<1) | (unsigned(BE_F)<<2) | (unsigned(CD_G)<<3)))&1;
 const bool EA_F=(0xdddd8888U >> ((unsigned(DC_F)<<0) | (unsigned(((address>>0)&1))<<1) | (unsigned(bit(10))<<4)))&1;
 const bool DB_G=(0x0a000a00U >> ((unsigned(((address>>7)&1))<<0) | (unsigned(!(bank_io && (address&0xff70)==0xc060))<<2) | (unsigned(AD_G)<<3)))&1;
 const bool EB_F=(0xdddd8888U >> ((unsigned(DB_G)<<0) | (unsigned(((address>>0)&1))<<1) | (unsigned(bit(11))<<4)))&1;
 const bool GC_G=(0x03030303U >> ((unsigned(((address>>7)&1))<<1) | (unsigned(!(bank_io && (address&0xff70)==0xc030))<<2)))&1;
 const bool GA_F=(0x0c0c0c0cU >> ((unsigned(AD_F)<<1) | (unsigned(read)<<2)))&1;
 const bool FD_F=(0x50005000U >> ((unsigned(((address>>0)&1))<<0) | (unsigned(GC_G)<<2) | (unsigned(GA_F)<<3)))&1;
 const bool ED_F=(0xcfcfc0c0U >> ((unsigned(((data>>7)&1))<<1) | (unsigned(FD_F)<<2) | (unsigned(bit(12))<<4)))&1;
 const bool EE_F=(0xbbbb8888U >> ((unsigned(((data>>1)&1))<<0) | (unsigned(FD_F)<<1) | (unsigned(bit(13))<<4)))&1;
 const bool EF_F=(0xbbbb8888U >> ((unsigned(((data>>2)&1))<<0) | (unsigned(FD_F)<<1) | (unsigned(bit(14))<<4)))&1;
 const bool EH_F=(0xf3f3c0c0U >> ((unsigned(CG_G)<<1) | (unsigned(((data>>3)&1))<<2) | (unsigned(bit(15))<<4)))&1;
 const bool GB_G=(0xfaaafaaaU >> ((unsigned(DB_F)<<0) | (unsigned(GA_F)<<2) | (unsigned(CD_G)<<3)))&1;
 const bool FA_M=(0x3f770c44U >> ((unsigned(((address>>0)&1))<<0) | (unsigned(GB_G)<<1) | (unsigned(((data>>0)&1))<<2) | (unsigned(DB_F)<<3) | (unsigned(bit(16))<<4)))&1;
 const bool FB_G=(0x08080808U >> ((unsigned(((address>>7)&1))<<0) | (unsigned(AD_G)<<1) | (unsigned(!(bank_io && (address&0xff70)==0xc070))<<2)))&1;
 const bool FC_F=(0xbbbb8888U >> ((unsigned(((address>>0)&1))<<0) | (unsigned(FB_G)<<1) | (unsigned(bit(17))<<4)))&1;
 const bool FB_F=(0x00880088U >> ((unsigned(((address>>7)&1))<<0) | (unsigned(AD_G)<<1) | (unsigned(!(bank_io && (address&0xff70)==0xc040))<<3)))&1;
 const bool FE_F=(0xbbbb8888U >> ((unsigned(((address>>0)&1))<<0) | (unsigned(FB_F)<<1) | (unsigned(bit(18))<<4)))&1;
 const bool FG_F=(0xf3f3c0c0U >> ((unsigned(FD_F)<<1) | (unsigned(((data>>3)&1))<<2) | (unsigned(bit(19))<<4)))&1;
 const bool CE_F=(0x04040404U >> ((unsigned(((address>>3)&1))<<0) | (unsigned(((address>>1)&1))<<1) | (unsigned(((address>>2)&1))<<2)))&1;
 const bool DD_F=(0xaeaaaeaaU >> ((unsigned(DB_F)<<0) | (unsigned(CD_G)<<1) | (unsigned(read)<<2) | (unsigned(CE_F)<<3)))&1;
 const bool GE_M=(0xbfb38c80U >> ((unsigned(((data>>5)&1))<<0) | (unsigned(DD_F)<<1) | (unsigned(DB_F)<<2) | (unsigned(((address>>0)&1))<<3) | (unsigned(bit(20))<<4)))&1;
 const bool GG_F=(0xdddd8888U >> ((unsigned(FD_F)<<0) | (unsigned(((data>>0)&1))<<1) | (unsigned(bit(21))<<4)))&1;
 const bool HA_G=(0xa000a000U >> ((unsigned(((address>>0)&1))<<0) | (unsigned(GA_G)<<2) | (unsigned(GC_F)<<3)))&1;
 const bool HB_F=(0xf3f3c0c0U >> ((unsigned(HA_G)<<1) | (unsigned(((data>>6)&1))<<2) | (unsigned(bit(22))<<4)))&1;
 const bool GB_F=(0xeeaaeeaaU >> ((unsigned(DB_F)<<0) | (unsigned(GA_G)<<1) | (unsigned(CD_G)<<3)))&1;
 const bool HC_M=(0xfb3bc808U >> ((unsigned(((address>>0)&1))<<0) | (unsigned(GB_F)<<1) | (unsigned(DB_F)<<2) | (unsigned(((data>>7)&1))<<3) | (unsigned(bit(23))<<4)))&1;
 const bool BE_G=(0x04040404U >> ((unsigned(((address>>3)&1))<<0) | (unsigned(((address>>2)&1))<<1) | (unsigned(((address>>1)&1))<<2)))&1;
 const bool BC_G=(0xeeaaeeaaU >> ((unsigned(DB_F)<<0) | (unsigned(BB_F)<<1) | (unsigned(BE_G)<<3)))&1;
 const bool HD_M=(0xfb3bc808U >> ((unsigned(((address>>0)&1))<<0) | (unsigned(BC_G)<<1) | (unsigned(DB_F)<<2) | (unsigned(((data>>6)&1))<<3) | (unsigned(bit(24))<<4)))&1;
 const bool BC_F=(0x0f000f00U >> ((unsigned(read)<<2) | (unsigned(BE_G)<<3)))&1;
 const bool FD_G=(0x80808080U >> ((unsigned(((address>>0)&1))<<0) | (unsigned(BC_F)<<1) | (unsigned(GC_G)<<2)))&1;
 const bool HE_F=(0xdddd8888U >> ((unsigned(FD_G)<<0) | (unsigned(((data>>6)&1))<<1) | (unsigned(bit(25))<<4)))&1;
 const bool FE_G=(0xa5a5a5a5U >> ((unsigned(((address>>0)&1))<<0) | (unsigned(((address>>1)&1))<<2)))&1;
 const bool HF_M=(0x3bfb08c8U >> ((unsigned(FE_G)<<0) | (unsigned(DF_G)<<1) | (unsigned(DB_F)<<2) | (unsigned(((data>>3)&1))<<3) | (unsigned(bit(26))<<4)))&1;
 const bool GD_F=(0xececececU >> ((unsigned(CD_G)<<0) | (unsigned(DB_F)<<1) | (unsigned(BC_F)<<2)))&1;
 const bool HG_M=(0xfb3bc808U >> ((unsigned(((address>>0)&1))<<0) | (unsigned(GD_F)<<1) | (unsigned(DB_F)<<2) | (unsigned(((data>>4)&1))<<3) | (unsigned(bit(27))<<4)))&1;
 q = (uint32_t(AC_F)<<0) | (uint32_t(BA_F)<<1) | (uint32_t(BH_M)<<2) | (uint32_t(CB_F)<<3) | (uint32_t(CC_F)<<4) | (uint32_t(CH_F)<<5) | (uint32_t(DA_M)<<6) | (uint32_t(DF_F)<<7) | (uint32_t(DG_F)<<8) | (uint32_t(DH_F)<<9) | (uint32_t(EA_F)<<10) | (uint32_t(EB_F)<<11) | (uint32_t(ED_F)<<12) | (uint32_t(EE_F)<<13) | (uint32_t(EF_F)<<14) | (uint32_t(EH_F)<<15) | (uint32_t(FA_M)<<16) | (uint32_t(FC_F)<<17) | (uint32_t(FE_F)<<18) | (uint32_t(FG_F)<<19) | (uint32_t(GE_M)<<20) | (uint32_t(GG_F)<<21) | (uint32_t(HB_F)<<22) | (uint32_t(HC_M)<<23) | (uint32_t(HD_M)<<24) | (uint32_t(HE_F)<<25) | (uint32_t(HF_M)<<26) | (uint32_t(HG_M)<<27);
 }
 uint64_t pads(bool irq=true) const noexcept {
 const bool EG_F=(0x22ff22ffU >> ((unsigned(bit(7))<<0) | (unsigned(bit(14))<<1) | (unsigned(bit(11))<<3)))&1;
 const bool FG_G=(0x5555ffffU >> ((unsigned(bit(17))<<0) | (unsigned(bit(19))<<4)))&1;
 const bool GH_F=(0x2a3f2a3fU >> ((unsigned(bit(15))<<0) | (unsigned(bit(21))<<1) | (unsigned(bit(18))<<2) | (unsigned(irq)<<3)))&1;
 const bool DE_F=(0x2a002a00U >> ((unsigned(bit(8))<<0) | (unsigned(bit(0))<<1) | (unsigned(bit(13))<<2) | (unsigned(bit(12))<<3)))&1;
 const bool FH_F=(0x7fff7fffU >> ((unsigned(EG_F)<<0) | (unsigned(FG_G)<<1) | (unsigned(GH_F)<<2) | (unsigned(DE_F)<<3)))&1;
 return (uint64_t(bit(1))<<13) | (uint64_t(bit(6))<<17) | (uint64_t(bit(10))<<19) | (uint64_t(bit(16))<<21) | (uint64_t(bit(3))<<23) | (uint64_t(bit(22))<<32) | (uint64_t(bit(23))<<33) | (uint64_t(bit(24))<<34) | (uint64_t(bit(25))<<36) | (uint64_t(bit(20))<<37) | (uint64_t(bit(26))<<39) | (uint64_t(bit(27))<<40) | (uint64_t(bit(9))<<46) | (uint64_t(FH_F)<<49) | (uint64_t(bit(5))<<53) | (uint64_t(bit(2))<<57);
 }
 uint8_t config_read(uint32_t address, bool read, bool cpu_phi2, uint8_t bus) const noexcept {
 const bool AG_G=(0x00550055U >> ((unsigned(((address>>14)&1))<<0) | (unsigned(((address&0xff8000)!=0xbc0000))<<3)))&1;
 const bool CG_F=(0x7f7f7f7fU >> ((unsigned(AG_G)<<0) | (unsigned(cpu_phi2)<<1) | (unsigned(read)<<2)))&1;
 const uint8_t mask = (uint8_t(!CG_F)<<0) | (uint8_t(!CG_F)<<1) | (uint8_t(!CG_F)<<2) | (uint8_t(!CG_F)<<3);
 const uint8_t driven = (uint8_t(bit(5))<<0) | (uint8_t(bit(9))<<1) | (uint8_t(bit(8))<<2) | (uint8_t(bit(15))<<3);
 return (bus & uint8_t(~mask)) | (driven & mask);
 }
 void clock(uint32_t address, bool read, uint8_t data, bool bank_io, bool irq=true) noexcept {
  // Outside the configuration ROM and mapped C0xx I/O, clock_full leaves
  // every register unchanged except CH_F (q bit 5), which is set. Preserve
  // the full equations above when changing this address qualification.
  if ((address&0xff8000)!=0xbc0000 && (!bank_io || (address&0xff00)!=0xc000)) {
   q |= (1u<<5); return;
  }
  clock_full(address,read,data,bank_io,irq);
 }
};
} // namespace twgs_decoded

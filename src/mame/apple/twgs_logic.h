// license:BSD-3-Clause
// TransWarp GS controller equations decoded from the GAL JED fuse maps
// identified below. Input bits 0-7 are pins 2-9; state/result bits 0-7 are
// registered output pins 12-19. Evaluate against the old state, then latch
// the returned byte on the controller edge (see twgs_controller.h).
#pragma once
#include <cstdint>
namespace twgs_decoded {
// Source JED: TransWarp_GS_-_TWGS1A.I_-_GAL16V8.jed
// SHA256: a31a5c4c471a323b7dcd2d51e03b6076022bd03111b937102843d5b22be39bec
inline uint8_t gal1(uint8_t state, uint8_t input) noexcept
{
 const uint32_t pins = (uint32_t(state) << 12) | (uint32_t(input) << 2);
 uint8_t next = 0;
 next |= uint8_t(!(((pins & 0x4004U) == 0x4U) || ((pins & 0xc004U) == 0x8000U) || ((pins & 0x40U) == 0x0U))) << 0;
 next |= uint8_t(!(false)) << 1;
 next |= uint8_t(!(((pins & 0xf0000U) == 0x20000U) || ((pins & 0xf0000U) == 0x80000U) || ((pins & 0xf0000U) == 0xd0000U) || ((pins & 0xf0000U) == 0x70000U) || ((pins & 0xe4000U) == 0x0U) || ((pins & 0xd4000U) == 0x90000U) || ((pins & 0xe4000U) == 0xe0000U) || ((pins & 0xd4000U) == 0x40000U))) << 2;
 next |= uint8_t(!(((pins & 0xf0000U) == 0x10000U) || ((pins & 0x60000U) == 0x20000U) || ((pins & 0xf0000U) == 0x80000U) || ((pins & 0xf0000U) == 0xe0000U) || ((pins & 0x60000U) == 0x40000U) || ((pins & 0xf0000U) == 0x70000U))) << 3;
 next |= uint8_t(!(((pins & 0x10010U) == 0x0U) || ((pins & 0x10020U) == 0x0U) || ((pins & 0x10040U) == 0x0U) || ((pins & 0x20070U) == 0x20070U))) << 4;
 next |= uint8_t(!(((pins & 0x20010U) == 0x0U) || ((pins & 0x20020U) == 0x0U) || ((pins & 0x20040U) == 0x0U) || ((pins & 0x10070U) == 0x70U))) << 5;
 next |= uint8_t(!(((pins & 0x40200U) == 0x200U) || ((pins & 0x40080U) == 0x80U) || ((pins & 0x80280U) == 0x0U))) << 6;
 next |= uint8_t(!(((pins & 0x80200U) == 0x200U) || ((pins & 0x80080U) == 0x80U) || ((pins & 0x40280U) == 0x40000U))) << 7;
 return next;
}
// Source JED: TransWarp_GS_-_TWGS2A-1_-_GAL16V8.jed
// SHA256: 9a997b6fafa93e61947c080dfa32793c5496ba522f32e0eb57b22468acfdc200
inline uint8_t gal2a(uint8_t state, uint8_t input) noexcept
{
 const uint32_t pins = (uint32_t(state) << 12) | (uint32_t(input) << 2);
 uint8_t next = 0;
 next |= uint8_t(!(((pins & 0xe0010U) == 0xe0000U) || ((pins & 0xe619cU) == 0x66098U))) << 0;
 next |= uint8_t(!(((pins & 0x60014U) == 0x60014U) || ((pins & 0x60100U) == 0x60100U) || ((pins & 0x60008U) == 0x60000U) || ((pins & 0x60010U) == 0x60000U) || ((pins & 0x42000U) == 0x40000U))) << 1;
 next |= uint8_t(!(((pins & 0x40080U) == 0x40000U) || ((pins & 0x4080U) == 0x0U))) << 2;
 next |= uint8_t(!(((pins & 0x64020U) == 0x44000U) || ((pins & 0x6c020U) == 0x6c000U) || ((pins & 0xf4000U) == 0x64000U) || ((pins & 0xcc000U) == 0x4000U) || ((pins & 0xec000U) == 0x44000U))) << 3;
 next |= uint8_t(!(((pins & 0x66260U) == 0x64220U) || ((pins & 0x54000U) == 0x44000U))) << 4;
 next |= uint8_t(!(((pins & 0x40000U) == 0x0U))) << 5;
 next |= uint8_t(!(((pins & 0x60000U) == 0x20000U) || ((pins & 0x6619cU) == 0x66098U) || ((pins & 0x6c1b8U) == 0x64088U) || ((pins & 0xf62a0U) == 0x640a0U))) << 6;
 next |= uint8_t(!(((pins & 0x62260U) == 0x60200U) || ((pins & 0xe2000U) == 0x20000U) || ((pins & 0xa0000U) == 0x0U) || ((pins & 0xc0000U) == 0x40000U))) << 7;
 return next;
}
// Source JED: TransWarp_GS_-_TWGS2B.I_-_GAL16V8.jed
// SHA256: c39327b67da4361b226acdb9e8449ab4e025ab7908ee7d6e3923cad8a494709a
inline uint8_t gal2b(uint8_t state, uint8_t input) noexcept
{
 const uint32_t pins = (uint32_t(state) << 12) | (uint32_t(input) << 2);
 uint8_t next = 0;
 next |= uint8_t(!(((pins & 0xe0010U) == 0xe0000U) || ((pins & 0xe619cU) == 0x66098U))) << 0;
 next |= uint8_t(!(((pins & 0x60014U) == 0x60014U) || ((pins & 0x60100U) == 0x60100U) || ((pins & 0x60008U) == 0x60000U) || ((pins & 0x60010U) == 0x60000U) || ((pins & 0x42000U) == 0x40000U))) << 1;
 next |= uint8_t(!(((pins & 0x40080U) == 0x40000U) || ((pins & 0x4080U) == 0x0U))) << 2;
 next |= uint8_t(!(((pins & 0x64020U) == 0x44000U) || ((pins & 0x6c020U) == 0x6c000U) || ((pins & 0xf4000U) == 0x64000U) || ((pins & 0xcc000U) == 0x4000U) || ((pins & 0xec000U) == 0x44000U))) << 3;
 next |= uint8_t(!(((pins & 0x662e0U) == 0x642a0U) || ((pins & 0x54080U) == 0x44080U))) << 4;
 next |= uint8_t(!(((pins & 0x40000U) == 0x0U))) << 5;
 next |= uint8_t(!(((pins & 0x60000U) == 0x20000U) || ((pins & 0x6619cU) == 0x66098U) || ((pins & 0x6c1b8U) == 0x64088U) || ((pins & 0xf62a0U) == 0x640a0U))) << 6;
 next |= uint8_t(!(((pins & 0x62260U) == 0x60200U) || ((pins & 0xe2000U) == 0x20000U) || ((pins & 0xa0000U) == 0x0U) || ((pins & 0xc0000U) == 0x40000U))) << 7;
 return next;
}
} // namespace twgs_decoded

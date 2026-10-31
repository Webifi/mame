# Sequential Systems Second Sight

Second Sight is a VGA card for the Apple IIgs. It has a Zilog Z8S180 CPU, an
Oak OTI-087X VGA controller and a Brooktree Bt481 RAMDAC. MAME emulates it as
the slot card `secondsight` on the Apple IIgs systems (`apple2gs`,
`apple2gsr1` and the other IIgs sets).

## Build

A full MAME build includes the card. For a small build that has only the IIgs:

```sh
make SUBTARGET=iigs SOURCES=src/mame/apple/apple2gs.cpp,src/mame/apple/apple2.cpp
```

This makes the executable `iigs`.

## ROMs

You need the usual IIgs system ROMs, for example the `apple2gs` set. The card
firmware is in `roms/a2secondsight`:

| File | Firmware | BIOS name |
|---|---|---|
| `rom_beef.bin` | 1.4-BEEF, the original EPROM. Not included. | `14` (default) |
| `rom_cbfc.bin` | 1.5-CBFC. Included. | `15` |
| `rom_3277.bin` | 2.0-3277. Included. Needs the v2.0 Lattice chip. | `20` |

If you do not have `rom_beef.bin`, select firmware 1.5 or 2.0 with `bios=`.

## Run

```sh
iigs apple2gs -ramsize 4M -sl3 secondsight,bios=15 -view Left-to-Right
```

The card has its own screen. `-view Left-to-Right` shows the IIgs screen and
the card screen side by side in one window. `-numscreens 2` shows each screen
in its own window.

## Machine Configuration menu

- **Second Sight monitor**: `VGA` (default) or `AppleColor RGB`. This sets the
  monitor sense bit. The firmware reads it when it starts, so restart the
  machine after a change.
- **Lattice chip (power cycle)**: `Original (BE8C)` (default) or
  `v2.0 (BBB1)`. Firmware 2.0 needs `v2.0 (BBB1)`.

## CRT display (optional)

The BGFX renderer has a CRT shader chain for each screen:

```sh
iigs apple2gs -sl3 secondsight,bios=15 -view Left-to-Right -video bgfx -bgfx_screen_chains crt-geom,crt-geom
```

For the card screen, `crt-geom-fields` shows interlaced modes one field at a
time. The shaders change only the displayed picture. Snapshots and emulated
timing do not change.

## Debugging and test scripts

| Tag | Part |
|---|---|
| `:sl3:secondsight:z180` | Z8S180 CPU |
| `:sl3:secondsight:video:oak` | OTI-087X VGA controller |
| `:sl3:secondsight:video:oak:screen` | card screen |

In Lua, `screen:pixels()` on the card screen returns the visible monitor area,
with the border. The OTI-087X device saves the active picture as
`m_active_display`: left, top, width and height, in screen pixels.

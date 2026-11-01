# VP2 depth of field (field post effect)

What the game computes for its depth of field, so the native renderer can draw it from the game's own data: the
packets the game builds for it (Z read as a texture, a T8 + CLUT channel shuffle, blur passes, a CT16 shuffle into
alpha, a final blend) are not drawn or read (no GS anywhere). Addresses are mod001 unless stated. [V] verified in the
MIPS, [I] inferred.

## The object

- A post-effect object of 0x13210 bytes, created at 0x222324 (ctor 0x2DAAA0), at `*(u32 *)0x0035DDF8 + 0x4448`. [V]
- Per-frame method 0x2D92A0 (vtable slot at main 0x159D90): allocates its work buffers, then calls the DOF manager
  0x2D21A0. [V]
- 0x2D21A0: auto-focus (byte `+0x130D0 == 1`) through 0x2D1820, else the parameter blocks `+0xF45C` (flag bit 1 of
  `+0xEF0C`) and `+0xF330 + i * 0x3C` (first word non-zero); each block goes to 0x2CCF60, which builds the packets. [V]

## Parameters (a 0x3C-byte block)

| Offset | Meaning |
|---|---|
| +0 | enable |
| +4..+0x14 | p0..p4: depth bytes (0..255) delimiting the blur ramps |
| +0x18 (s16) | blur level (0x80 / 0x5B / 0x40) |
| +0x1C (u8) | brightness factor c of the blurred copy (C = B x 2c / 128) |
| +0x1D..+0x23 | mode flags (+0x1F, +0x21: other table builders) |
| +0x24 / 28 / 2C / 30 | blur radii |
| +0x34 | strength 0..1 |

Depth scale `+0x131C4` by ZBUF PSM: Z32 16777216, Z24 65536, else 256: the table index is Z's top byte. [V]
Camera inputs (0x209A20, from the camera object `obj->0x94`): flags `+0xEF0C` from camera bytes +0x44B..+0x44F, mode
`+0x130D0` from +0x44D, focus `+0x130D4 / D8 / DC` from floats +0x450 / 454 / 458; projection constants `+0xF49C..F4BC`,
`+0x131C8` set by 0x2CB8D0 from camera +0x230 / 260 / 270 / 2A4 / 2B0. [V]

## The depth table (what to read)

0x2CC110 fills 256 entries at `this + 0xEF10`, one per depth byte: `a << 24 | v << 16 | v << 8 | v`, v the blurred
picture's weight (0..128, 128 = fully blurred), a an opacity ramp. [V arithmetic]

```
for i in 0..255:
  v = 0                                         if i > p0 or i < p4
    = 0.5 + 128 r (1 - (i - p1) / (p0 - p1))    if p1 < i <= p0
    = 0.5 + 128 table[i] / (|X(p2)| K)          if p2 < i <= p1     (table = *(float **)(this + 0x131B8))
    = 128                                       if p3 < i <= p2 (or p3 == p4)
    = 0.5 + 128 (i - p4) / (p3 - p4)            if p4 <= i <= p3
```

(X, K, r from the projection constants, see the research notes.) The native pass reads this finished table each frame:
it is the game's own data, built by its own code.

## The picture the game makes from it

Blur the frame at decreasing sizes (128x88 -> 64x44 -> 32x22, bilinear), brighten the blurred copy by 2c/128, then mix
it over the frame by the table's weight for each pixel's depth: `(blurred - frame) * w + frame`. Native: depth from the
native renderer's depth buffer, a blur of the native frame, one full-screen pass.

## What the title and the first field ask for (VP2S_POST_LOG, 2026-11-01)

Object at 0x5F18B0, flags `+0xEF0C` 0x1D, no auto-focus; the same blocks from the title to the field:

| Block | p0..p4 | +0x18 level | +0x1C c | +0x1D..+0x1F | radii | strength |
|---|---|---|---|---|---|---|
| +0xF45C | 0 | 0x80 | 0 | 00 80 01 | 0, 0 | 1.0 |
| +0xF330 | 0 | 0x80 / 0x80 | 0x40 (C = B) | 01 80 00 | 5.5, 0 | 1.0 |
| +0xF36C | 0 | 0x80 / 0x80 | 0x40 | 01 80 00 (+0x21: 1) | 5.5, 0 | 1.0 |
| +0xF3A8 | 0 | 0x80 / 0x40 | 0xC8 (C = 3.1 B) | 01 80 00 (+0x21: 2) | 5.5, 2.3 | 1.0 |

With every p 0 the table weights only the farthest depth (Z top byte 0: the sky), so the sky gets the blurred copy
brightened up to ~3x: the original's bright title sky (drawn at vertex colour 64) and, likely, the field's glow. [I]

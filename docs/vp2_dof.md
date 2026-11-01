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

## What the title and the first field ask for (VP2S_POST_LOG and EE dumps, 2026-11-01)

Object at 0x5F18B0, flags `+0xEF0C` 0x1D, no auto-focus, the same from the title to the field. Corrections to the
table above: a block's words are +0x24 / +0x28 radii, **+0x2C / +0x30 blur gains (1.0), +0x34 s (0.0)**, +0x38 the
composite's Z; table entries are `a | v << 8 | v << 16 | a << 24` (R = A = a, G = B = v), a = 0x80 when s = 0. [V]
Before use c becomes `c + (128 - c) s` (for +0x21 != 0 only when flag bit 2 is set). [V]

| Block | +0x1C c | +0x18 / +0x1A | +0x1D..+0x23 | radii +0x24 / +0x28 |
|---|---|---|---|---|
| +0xF45C | 0 | 0x80 | +0x1F 1: not drawn (flag bit 1 clear) | 0 / 0 |
| +0xF330 | 0x40 | 0x80 / 0x80 | +0x20 1, +0x21 0, +0x22 3 | 5.5 / 0 |
| +0xF36C | 0x40 | 0x80 / 0x80 | +0x20 1, +0x21 1, +0x22 3 | 5.5 / 0 |
| +0xF3A8 | 0xC8 | 0x80 / 0x40 | +0x20 1, +0x21 2 | 5.5 / 2.3 |
| +0xF3E4 | 0xB6 | 0x80 / 0x40 | +0x20 2, +0x21 3, +0x23 1 | 5.5 / 2.6 |
| +0xF420 | 0x80 | 0x80 / 0x40 | +0x20 2, +0x21 4, +0x23 1 | 5.5 / 2.6 |

## The picture of a block with +0x1F = 0 (0x2CCF60, every block above)

Buffers: the frame F (W x H = +0xA8 x +0xAC, 512 x 480); Z (PSMZ24); work maps of w x h = +0x131AC x +0x131B0
(128 x 89); C, the frame shrunk to w x h (made by another object, `*(0x35DDF8) + 0x444C`). Sizes: L18 / L1A the s16
+0x18 / +0x1A, h18 = L18 h / w. All sprites cover their rectangles exactly (pixel edges). [V]

1. Once a frame: Z point-sampled to w x 2h; index = min(255, Z24 >> (16 - n)), n = +0x131D0 (0 when +0xF4AC == 0). [V/I]
2. The block's table (2CC110 for +0x21 == 0, else 2CBBF0) as a CLUT; weight map S = table[index] (nearest, 2:1 in
   height), w x h. [V]
3. When L1A != w: S shrunk bilinear to L1A x h1A. When +0x28 > 0: blur 1 of that map, Gaussian sigma = 0.555 x
   (+0x28), gain +0x30. [V]
4. Colour: C stretched bilinear to L18 x h18 times g (RGBAQ from g = +0x131E0, 2.0 seen: min(255, C g)), then blur 2:
   Gaussian sigma = 0.555 x (+0x24) (5.5 -> 3.05 px at 128 wide), gain +0x2C. [V]
5. Alpha: the weight map's G scaled by 2c / 128 (c 0x40 = x1, 0xC8 = x3.1) to L18 x h18, clamped to 128. When c is
   0x40 and L1A == L18 that step is skipped and the alpha is taken from the blurred colour's green (a glow keyed by
   brightness; check at run time before relying on it). [V code, I meaning]
6. Composite over F (stretched bilinear to W x H): `out = F + (col - F) A / 128`, A times +0x1E / 128, only where the
   frame's Z <= +0x38 when (+0x20 & 3) == 1 (Z test GEQUAL with the sprite at Z +0x38 = 0: the cleared background,
   the sky). [V]

So on the title and the field the sky gets a blurred, ~2x brightened copy of the frame (the original's bright title
sky, drawn at vertex colour 64), and the blocks with +0x20 == 2 (no Z test) a brightness-keyed glow over everything.
Open: the VU1 blur microcode's handling of alpha, C's producer, the runtime values of +0x131C4..+0x131E0. [I]

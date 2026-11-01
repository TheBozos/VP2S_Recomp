# VP2 2D layer (HUD, text, menus): what the native renderer draws from

VP2's interface does not use VU1: each 2D element builds its own GIF packets on the EE, in GS context 2, queued on
the DMA list when its layer is drawn. The native port draws the same items from the game's descriptions instead (no
GIF packet read). Addresses are mod001 unless stated. [V] verified in the MIPS or a dump, [I] inferred.

## Framework

- UI manager G2 = *(u32 *)0x35E170 (vtable 0x15A160): four font objects at +0x14/+0x54/+0x94/+0xD4; sheets 'SHI\0' 0x41 /
  0x3D at +0x818 / +0x81C; scale +0x840 / +0x844 (16.0, 14.9333) and offset +0x848 / +0x84C (27648, 29184): GS 12.4
  X = offset + scale * vx on a virtual 640 x 480 screen (GS pixel x = 1728 + vx, y = 1824 + 0.9333 vy); text shadow
  offset +0x850 / +0x854; registered layers +0x85C (head, linked by +0x98 / +0x9C). [V]
- Layers draw in the scene pass after the 3D (pass byte +0x65 = 1), in registration order; every layer's draw goes to
  0x2EC8F0(layer). Layer: +0x08 child list (next = element + 8), +0x40 / +0x44 position, +0xE0 alpha (0..128). [V]
- Element header: +0x18..+0x24 x, y, w, h (layer-relative), +0x28 alpha, +0x2C z, +0x39 visible, +0x3B built (its
  packet cached), +0x3C dirty, +0x3D enabled, +0x6C font, +0x90 colour multiplier (/128); vt +0x20 build. An element is
  drawn when +0x39 && +0x3D && +0x3B (the game's own chain rule). [V]

## Items (the build functions, one call per item)

- Glyph 0x2E7DD0(element, item): item (0x30 bytes) +0x00..0x0C box (x0, y0, x1, y1), +0x10 / +0x14 scale, +0x18 font,
  +0x20 colour, +0x24 shadow mode, +0x28 shadow colour, +0x2E glyph index. Pixels: 4 bits a pixel at font+0x10 +
  index * font+0x20, font+0x24 x font+0x28 (32 x 28); palette part (CSA) 1 for the glyph, 1 or 2 for its shadow (the
  'SHI\0' 0x41 sheet's palette). Quad grown by the border font+0x2C * scale; colour x element+0x90 / 128; alpha =
  layer+0xE0 * element+0x28 / 128; shadow offset G2+0x850 / 0x854 (mode 1). [V]
- Fixed-font string 0x2E75E0: 16 x 16 cells of 'SHI\0' 0x41, cell of c = ((c - 1) & 15, (c - 1) >> 4), 12-pixel advance.
- Sprite quad 0x2EB2D0 (f12 layer alpha, a0 sprite = element + 0x50, a2 cursor, a3 layer position, t0 z, t1 colour
  multiplier); quad list 0x2EAC40 (same arguments). Sprite: +0x00 its sheet's 'FIS' file (until uploaded), +0x04 its
  parts' 'DTT' file, +0x08 parts (16 bytes: u0, v0, u1, v1 in 4.12 of the sheet (s = u / 4096), w, h, +0xC CSA),
  +0x0C quads (24 bytes: part, x0, y0, x1, y1, z; x1 / y1 <= 0: the part's size), +0x14 / +0x18 position, +0x1C /
  +0x20 pivot, +0x2C / +0x30 scale, +0x3C rotation (degrees), +0x40..+0x4C colour (+0x4C multiplies the single quad's
  RGB only), +0x50 alpha, +0x54 part (single quad), +0x58 quad count, +0x5A..+0x61 TEX0 (TBP, CBP, TBW, PSM, TW, TH),
  +0x64 CSA (negative: the part's). [V]
- Sheet upload 0x2EBB10(a0 sprite, a1 cursor, a2 TEX0 destination or 0): the texture manager (0x29D8F0) gives the
  sprite's FIS a VRAM block, the TBP goes to the sprite's TEX0 or to a2. [V]
- Texture manager *0x35DE38 (scratchpad): resident list from the byte at +0x2008 (next +0x2008 + e), entry e key
  +0x1008 + 16e tag / +0x8 + 16e index, TEX0 +0x1010 + 16e (TBP low 14 bits). The native port finds a sprite's sheet
  there by its TBP (the save portraits). [V]
- Fixed UI slots outside the manager: TBP 0 'SHI\0' 0x41 (CBP 0x3FFC) and 0x400 'SHI\0' 0x3D (CBP 0x3FF8), set by
  0x2EC190, uploaded by 0x2F32F0(a0 image: +0xC4 data to DBP +0xD0 when +0xE4, +0xD4 data to DBP +0xE0; the data
  lies in its FIS file, so the port keys each DBP from it); TBP 0x800 / 0x1000 PSMT4HH 512 x 256 sheets of the menus, TEX0 records built by mod003 (0x4643C0 and
  callers of 0x30C5E0: TBP 0x800, CBP from the menu context +0x98, DTTs from its +0x90): their sheet is not keyed yet
  (the field HUD's 0x800 is 'JOHN' 0x276F). 0x2EC130(a0 sprite, a1 DTT, a2 TEX0 record) gives a sprite parts + TEX0. [V]
- Window background 0x2E9DD0(a0 window): rectangle +0x18 (or +0xB8 when +0xE8) plus its layer's (+0x14) position; one
  gouraud strip (0x44211B, 0x3F1B17, 0x513A24; darker set when +0xF5) at 0.95 x alpha, then 8 strips from the table at
  0x357900 ({x, y, colour} x 4 over the corners x0, x0 + m, x1 - m, x1; m = size / 4, at most 24) shading the edges
  (black at half the alpha to nothing). [V]
- Windows 0x2E9580 (gradient 0x2E9DD0, border 0x2E9C20), scissor 0x2EF440, rectangles 0x460EA0 (mod003), 0x30ED70,
  flat rectangles main 0x120A00 / 0x120870, screen fade mod003 0x3D36C0 (type +0xF3, colours +0x98..+0xA0). [V]
- Sheets are FIS files with their own keys (FIS +0x14 tag, +0x18 index): 'SHI\0' (tag 0x00494853) 0x41 and 0x3D
  (icons, fixed font, glyph palettes), the field HUD sheet 'JOHN' (0x4E484F4A) 0x276F. [V]

## Text encoding

Byte = ASCII - 0x1F; b0 & 0x80 starts a two-byte code (b0 & 0x7F) | b1 << 7, >= 0x4000 a control (0x4000 newline,
0x4001 end, 0x4002 wait, 0x4019 button icon). [V]

## Native plan

Capture each item at build (the game caches the packet and frees the items), keyed by its element; draw an element's
items when its layer is drawn (0x2EC8F0) and the element is shown, in the layer's order, after the 3D.

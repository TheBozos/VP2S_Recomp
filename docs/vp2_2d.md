# VP2 2D layer (HUD, text, menus): what the native renderer draws from

VP2's interface does not use VU1: each 2D element builds its own GIF packets on the EE, in GS context 2, queued on
the DMA list when its layer is drawn. The native port draws the same items from the game's descriptions instead (no
GIF packet read). Addresses are mod001 unless stated. [V] verified in the MIPS or a dump, [I] inferred.

## Framework

### Bozo solid screen rectangle (2026-11-07)

Mod002 `0x372180` now submits the mode-0 screen object (`0x15bc00`) as a native
rectangle from colour `+0x9c`, endpoint bounds `+0xac..+0xb8`, depth `+0xbc` plus
`+0xf1`, and the original source dimensions. Its compiled scalar writer flush and
real software completion remain. Additional modes/content reject until qualified.
64 producer/flush/completion CPU/RAM cases check both passes and native vertex
endpoints. Startup reaches the next unported UI layer; visual parity remains open.

### Bozo fixed-image ownership (2026-11-07)

The independent Bozo runner now replaces the physical enqueue in `0x2f36f0`
with publication of the offline FIS asset under its game image object, followed
by the original compiled scene completion. The image type, current scene pass
and pending-object lifetime are checked. Its destructor releases the association;
copied native assets survive source-FIS overwrite and image-object rebinding.
The native association uses image objects, with no hardware-address lookup map.
The shared helper `0x2ef390` and unported UI layer drawing remain guarded.
This does not qualify a rendered UI or the Logun route in the Bozo runner.

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

- Glyph 0x2E7DD0(element, item): item (0x30 bytes) +0x00..0x0C box (x0, y0, x1, y1), +0x10 / +0x14 x / y scale, +0x18 font,
  +0x1C (int) the glyph's width (its UVs: 2 border + that width, 2 border + font +0x14 high; the rest of the cell unused),
  +0x20 colour, +0x24 shadow mode, +0x28 shadow colour, +0x2E glyph index. Pixels: 4 bits a pixel at font+0x10 +
  index * font+0x20, font+0x24 x font+0x28 (32 x 28); palette part (CSA) 1 for the glyph, 1 or 2 for its shadow (the
  'SHI\0' 0x41 sheet's palette). Quad grown by the border font+0x2C * scale; colour x element+0x90 / 128; alpha =
  layer+0xE0 * element+0x28 / 128; shadow offset G2+0x850 / 0x854 (mode 1). [V]
- Fixed-font string 0x2E75E0: 16 x 16 cells of 'SHI\0' 0x41, cell of c = ((c - 1) & 15, (c - 1) >> 4), 12-pixel advance.
- Sprite quad 0x2EB2D0 (f12 layer alpha, a0 sprite = element + 0x50, a1 cursor, a2 position (float x, y), a3 z, t0 colour
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
  0x2EC190, set up by 0x2F3630(a0 image, a1 its FIS, a2 PSM, a3 DBP of the pixels, t0 of the palette; uploaded by
  0x2F32F0), so the port keys each DBP from the FIS; TBP 0x800 / 0x1000 PSMT4HH 512 x 256 sheets of the menus, TEX0 records built by mod003 (0x4643C0 and
  callers of 0x30C5E0: TBP 0x800, CBP from the menu context +0x98, DTTs from its +0x90): their upload is not found
  yet; compared with the original's load menu, 0x800 holds 'SHI\0' 0x5E (frame ornaments) and 0x1000 'SHI\0' 0x3E
  (stone), loaded with the menu (the field HUD's 0x800 is 'JOHN' 0x276F). 0x2EC130(a0 sprite, a1 DTT, a2 TEX0 record) gives a sprite parts + TEX0. [V]
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

## Field HUD (2026-11-01)

- Button icons in text (control 0x4019): sprites of the text element (+0x100 -> +0x24, 0x70 bytes each, +0x28 of
  them, shown when +0x66), drawn by 0x2EFCD0 from the text build 0x2EFA30 with direct calls; placed from the layer's
  position plus the element's (+0x18 / +0x1C) less its scroll (+0x78 / +0x7C). The port captures them after 0x2EFA30.
- Prompt backing bar: class 0x165BB0 (mod003 build 0x448AF0), embedded at owner + 0x230 (the prompt layer): a cap (part
  22) and a body to the right edge (part 23) of the field HUD sheet (TBP 0x800 PSMT8H, CBP 0x3FF4: 'JOHN' 0x276F),
  parts at *(*(*(0x48DB48) + 0x5C) + 0x224); left at layer x + e+0x58 + 5, top at layer y + e+0x5C + 2, h + 4 high
  (h part 22's), cap h GS pixels wide (x 0.75 when bit 4 of *(*(0x48DB4C) + 3)). Drawn each frame from live data.
- TBP 0x800 is shared: the menus' sheets use CBP 0x3FC8 / 0x3FCC, the field HUD's CBP 0x3FF4.
- The minimap is another element, not found yet.

## Field minimap (2026-11-01)

The area map object M = *(*(*(0x48DB48) + 8) + 0xC8) (mod003 class 0x165470; per-frame update 0x440FA0, vtable +0x14)
builds the minimap each frame (0x457BE0) as a small 3D scene through the generic polygon batcher (flush 0x200DE0),
plus an EE-built background (X = *(M+0x1640), class 0x167180). Its own 2D frame: origin X+0x94 / +0x98 (1792, 1808),
scissor 24..179 x 24..139 (*(M+0x1650) +0xA4..). The native port draws it from M's data (vp2s_minimap.cpp): the
background and border, the player's layer of rooms (records *(M+0x1644), 0xF0 bytes, textured with the floor image
*(M+0x29C)) through the scene object (*(M+0x1630) +0x90) and the map camera (*(M+0x1638)), after the 3D and before
the effect passes (it is part of the scene: doubled and scattered like it). Not yet: visited rooms only, doors,
icons, the player's marker (a green spinning kite), the camera footprint, the full map.

## Location announcement (2026-11-04, USA addresses)

The bottom-left "Solde / Port of Logun" announcement is a separate custom element, class `0x167820`,
build `mod003 0x4650D0`, layout `0x465150`; its layer owner is class `0x167850`.
It does not call the generic sprite emitters. Class `0x165E00`, already handled by the native UI,
is the backing of the top-of-screen interaction prompt, not this announcement.

The native draw reads the original layout inputs: layer position `+0x40/+0x44`, alpha `+0xE0`,
text bounds at `(layer+0xF0)+0xEC`, integer width `+0x27C`, separator inset `+0x2A4`, and flags `+0x2AC`
(bit 2 suppresses the whole decoration, bit 0 selects the reversed/short layout).
The normal two-line layout uses parts 11, 20 and 21 from the field HUD's DTT at
`*(*(*(0x48E048)+0x5C)+0x224)`, sheet `JOHN 0x272B`. Part 11 supplies the upper body;
part 20 the lower body; part 21 caps both rows. The tail width is `part21.w * 0.85`,
with the game's additional 0.75 factor in its narrow mode. The separator has a dark offset
and a light line, drawn from the same layer/text measurements as the original.

The emblem uses the live part at layer `+0x268`, resource `+0x26C`, enable `+0x270`,
texture-table index `+0x284`, and alpha multiplier `+0x298`. Each 24-byte entry at
`resource+0x10A8` retains its FIS pointer at `+0x10`; these direct uploads do not appear
in the general texture cache. Its sheet is resolved from that FIS, rather than a fixed emblem key; Solde's loop-shaped emblem
is in `JOHN 0x272D`. The original normal-layout horizontal size is 0.85 of its part width.
The layer remains underneath the pause's frozen scene copy, so it is not redrawn at full brightness.
The pause now requests `Renderer::capturePreviousFrame()`: copy the completed native target before
drawing the next frame. `captureFrame()` copies the current frame at its end; using that request
for pause captured the first paused frame after hiding the location layer, losing the announcement.
The separate previous-frame API preserves the existing current-frame API for other callers.

Optional `VP2S_LOCATION_DUMP=<file>` writes EE memory once when the layer reaches the screen,
and logs its layer, element, parts and resource pointers for offline layout verification.

Verification: USA build passed, followed by the guarded Logun-to-inn replay. Normal banner:
`build_usa/label_pause_verified/native_392.png`; paused banner: `native_396.png`; inn: `native_518.png`.
The saved PCSX2 reference is `SLUS-21452_20261102201340.png` (paused). At normalized 640x480,
the strongest separator edge is row 407 in both native captures and PCSX2. The backdrop zone
`(120,423)-(205,433)` averages luminance 98.53 before and 76.29 after in native gameplay;
the paused reference is darker overall, so it is used for geometry and pause behavior rather than
an unpaused brightness target. Local evidence: `build_usa/label_final/comparison.png`,
`pause_comparison.png`, `comparison.json`. Emblem selection was checked against the live FIS
pointer at resource entry `+0x10`, which points to `JOHN 0x272D` in the Solde run.

The minimized launcher stalled presentation in two runs before the field. The successful final
run used an unminimized window placed at (-30000,-30000), with its bounds and unchanged foreground
window checked by `build/agent_tools/offscreen_test.ps1`; host input/audio stayed disabled and
the existing start/stop helpers restored the memory cards. No UI changes were inferred from
the stalled pre-field runs.

## Field NPC dialogue (2026-11-04, USA addresses)

The dialogue layer (owner class `0x15FD30`) has two custom elements whose builds bypass
the generic window/sprite emitters: backing class `0x167B50` (`mod003 0x468620`) and
speaker separator class `0x167B20` (`0x4683A0`). Both were absent from the native draw.

The backing reads the element's layer-relative rectangle `+0x18..24`; nine 12-byte
records at `0x48DD50` select four corners from the rectangle's 4x4 grid, expanded by
eight virtual units at each edge. Each vertex selects transparent black or black with
alpha `layer[+0xE0] * element[+0x28] / 256`. The native renderer draws these strips directly.
The separator uses its own rectangle, RGB `(189,81,43)`, opacity `layerAlpha * elementAlpha / 128`,
with a peak at one quarter of the width and 0.2 of that opacity at both ends.

The continuation arrow is class `0x15A4E0`, build `mod001 0x2F1230`. That build directly
calls `0x2EB3A0`, so the generic sprite-element capture did not see it. The native fallback
reads its live sprite at `element+0x50` when `+0xBC` and the normal element visibility/build
flags allow it, using the layer position plus element `+0x18/+0x1C`. Sprite alpha already
contains the element alpha: multiply it by the layer alpha once. Existing captured sprites
retain their previous alpha path. This preserves the game's waiting/animation state.

Separating evidence: read-only EE RAM snapshots from the user's current PCSX2 and native
sessions (`build/agent_tools/npc_pcsx2.ee`, `npc_native.ee`) show the same rectangles:
backing `(20,344,600,92)`, line `(48,374,544,3)`, arrow position `(604,436)` with pivot `(12,12)`
and part size `(24,24)`. Offline inspection of PCSX2's cached draw data gives line vertices
`x=47.96875,183.98437,591.95312`, `y=373.94529..376.99217` in the game's virtual UI coordinates,
within 0.06 units of the native layout. No draw packets are replayed or interpreted at runtime.

Verification: isolated `vp2s_npc.exe` built from the same Ninja graph while the manual game
remained open, with separate input file and memory-card directories, no host input/focus, and
muted audio. A 250 ms left-stick movement followed by CROSS from the initial Waiting Area
reproduces the Girl's pirate dialogue. `build_usa/npc_probe/shots/native_353.png` has the
backing, separator and arrow; `native_506.png` checks the next page; `native_582.png` checks
that all three disappear on closing. The original manual game exited independently before
the final standard USA pipeline build, which passed (21 s).

Local comparison: `build_usa/npc_probe/comparison.png` and `comparison.json`, using the user's
reference screenshots. Normalized crops contain 512 brown separator pixels / 124 white arrow
pixels in PCSX2, 0 / 0 before, and 509 / 121 after. These are presence/shape checks;
the supplied crops have different framing and are not registered full-viewport screenshots.
The character's world positions also differ by about 86 game units between the two supplied states.

Computer-use capture of the open PCSX2 window failed (unavailable geometry / capture timeout,
and F8 produced no file), so reference analysis used the supplied images and read-only game RAM.
Neither running session was reset or advanced by the analysis. The isolated test process stopped
after its guard timeout; its saves/input paths never pointed at the user's session.

## Armory shop (2026-11-05, USA)

The Solde Armory exposed several custom builders that bypass the captured generic
sprite/window calls. The native draw now uses their live layout and FIS/DTT records;
it never reads or replays their cached draw packets. Original MIPS and offline packet
inspection were used to check the formulas against PCSX2.

| Element class | Original builder | Native inputs and behavior |
|---|---|---|
| `0x15A420` | `0x2EF5B0` | Scissor rectangle `+0x18..24`; `+0x51` adds the layer position, `+0x50` restores the screen. Applies to subsequent siblings, including text, sprites and windows. Restore the previous clip when leaving the layer. |
| `0x15AC60` | `0x310480` | Item icon consists of **two** sprites at `+0x50` and `+0xB8`, enabled by each sprite's `+0x66`. The old capture keyed the second sprite as another element and lost its glyph. Read both live and apply layer alpha once. |
| `0x15AC30` | `0x30EE00` | Black row-selection strip, 16 virtual units fading at either end, alpha `layerAlpha * elementAlpha / 512`. |
| `0x16D040` | mod023 `0x4A4520` | Category icons: DTT `layer+0x464`, resource `+0x468`, slot `+0x470`, count byte `+0x47A`. Parts 9 through 14, with part 16 replacing the last when byte `+0x47B != 1`. Source pixels: start `(layerX+60)*512/640`, `(layerY+4)*512.9599609375/480-2`; 30x30 sprites, 72-pixel X step. |
| `0x16D010` | mod023 `0x4A4BB0` | Cyan category-selection strip with 16-unit transparent ends. |
| `0x167CF0` | mod003 `0x469EB0` | Three-piece stone header; DTT `layer+0xF4`, resource `+0x150`, slot `+0x154`, middle width/height `+0x158/+0x15C`. RGB 104. |
| shop header owner `0x16D210` | mod023 `0x4A3500` | Adds the portrait from part `+0x1660` and selected resource slot `+0x1670`, after delay `+0x1674`. Right/bottom anchors `(layerX+296)*512/640`, `(layerY+126)*512.9599609375/480`; part size scales `.9*.88*narrow`, `1.2*.88`, with `narrow=.75` for bit 4 of `*0x48E04C+3`. Alicia resolves to FIS key `0x30327076/0x8F`. |
| `0x167DF0` | mod003 `0x46A600` | Buy/Sell button resource and DTT from `*(element+0x54)+0x8C0..8C8`. Parts 3/4 cap a 96-source-pixel middle; part 8 is its base, part 6 fades over it by `element+0x58`. Height 26.716665 source pixels; middle inset 1 pixel vertically. |
| `0x167EF0` | mod003 `0x46BB60` | Top help backing: part 5 from DTT `layer+0xF4`, resource/slot `+0x388/+0x38C`, RGB 96. Overlay RGB 48 with alpha ramping from zero at the left to layer alpha at the right. |
| `0x167C00` | mod003 `0x469810` | Black footer plus warm separator, strongest at one quarter of the width. Shop owner `0x16D2E0` adds the two OTH emblems (mod023 `0x4A24C0`): part 15 of `layer+0x5BC`, resource/slot `*0x48EEE0+0x167E4/+0x168B8`, X offsets 208 and 436 source pixels. |

Custom builders use source-frame pixels, unlike the generic text framework's virtual
coordinates. Convert through `G2+0x840/+0x844` divided by 16, truncating source vertices
to 1/16 pixel first. The shop source frame is 512x512, with scale `.8,1.0666667`.
Scissors clamp to `G2+0x838/+0x83C` and preserve inclusive source-pixel bounds when
converted to the native material's GS-window coordinate convention.

Verification used a private portable PCSX2 reference and isolated native new-game
runs, with independent memory cards/input files, no native host input or focus,
off-screen windows and muted audio. The original manual native session remained
running. The normal USA pipeline build passed; the alternate probe uses the same
objects and libraries when the manual executable is open.

Local evidence is under `build_usa/armory_bug` and
`build_usa/attacks_probe/armory_verified/shots`. Captures: All Items `native_231.png`,
selected Long Sword/Performance `native_248.png`, Resistance `native_274.png`,
Effect `native_300.png`, Description `native_406.png`, weapons filter `native_511.png`,
and quantity 1 / subtotal 500 OTH `native_555.png`. The earlier `armory_final`
run's `native_382.png` checks scrolling to Cloak through Leather Boots. Question-mark
merchandise names are also present in PCSX2 and are intentional locked items.

`comparison.png` / `comparison.json` normalize UI captures to 640x480 (the user's
before image has a 14-pixel left border removed). First-item glyph bright pixels:
before **0**, PCSX2 **50**, fixed **48**. All six category zones have no icon edges
before and populated edges after. The footer's warm separator peaks at row **446**
in PCSX2 and **445** native. A blank stone zone averages luminance **10.86** before,
**81.45** PCSX2, **83.44** fixed. These are presence/geometry checks, not exact pixel
parity: capture resolutions and texture filtering differ, and the 3D camera positions
are not registered.

Starting route (Windows, no WSL):

```powershell
python tools/attacks_menu_probe.py --tag armory_check --exe build_usa/port/vp2s.exe --script-file tools/pad_scripts/armory_usa.txt --duration 600
```

Inspect captures before interpreting a result: entry timings shift under heavy CPU
load. At the Armory doorway, a live `UP/1200` enters; inside, `LEFT/800` approaches
the shopkeeper. Use `CROSS/150` to talk, then again after each transition to open
Buy, All Items, and the item list. Append LF-terminated commands to that probe's
`pad.txt`; never use the manual session's input file. Triangle cycles Performance,
Resistance, Effect and Description. A 900 ms Down hold scrolls the list; Circle
returns to category selection, Right selects weapons, Cross enters the filtered list.
Right/Left in the list changes quantity; Circle backs out. First shop exit starts a
multi-page explanation of locked merchandise; advance it before judging field controls.

Final standard-build replay: `armory_cache_final/native_195.png` reaches All Items
using the tracked route without intervention; `native_209.png` verifies Performance
and the help icons. A proposed text-icon cache reset was tested in this replay,
did not fix the separate badge below, and was reverted from the delivered change.

Open follow-up: a tiny cyan control badge remains near the top right during the
first-visit post-shop dialogue (`armory_verified/native_602.png` and
`armory_cache_final/native_229.png`); PCSX2's `20261105002355.png` has no badge.
It disappears when the dialogue ends (`armory_verified/native_754.png`). Both
snapshots' field-prompt text elements have zero primitives in their original cached
draw data. Clearing captured text icons at text-layout start had no effect, so that
cache-reset hypothesis is rejected. Possible causes are another captured draw type
retained across element reuse or a draw-rule mismatch. Separating evidence needed:
log the native element/class and capture type that submits the badge during this
dialogue, then compare its original draw condition. Do not hide a screen region.

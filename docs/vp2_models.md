# VP2 models: what the native renderer draws from

VP2's draw code does not build vertices: for each visible mesh it puts a DMA REF to a prebuilt VIF packet inside the
model file (and one to a 16-qword material block) into the VIF1 chain. The geometry and the materials exist only as
those packets, so they are converted offline from the user's disc (rule decision 2026-10-31, as for the textures) and
loaded by the game's own pointer; everything else is read from the game's EE structures. Addresses are mod001. [V]
verified in the MIPS or the assets, [I] inferred.

## Town face comparison (2026-11-05, USA)

The manual pass's close view of Alicia's angular profile was compared with a
private PCSX2 slot-1 reference in the Waiting Area. Native object `0x9960C0`,
model `0xBD0C90`, and reference object `0x996040`, model `0xBD0C10`, each have
146 meshes. Offline conversion of both live snapshots gives maximum absolute
position and normal differences of **0.0** across all those meshes. The working
model pointer is zero in both. The world orientations differ by about 0.65 degrees;
positions and idle animation phases differ, so this is not a registered pixel test.

The field model's texture includes FIS key `0x33317076/0x4C`, a 128x128 atlas
with the face in its upper-left area. PCSX2 also shows the angular profile at this
angle; its close-up cinematic face is a separate reference and does not establish
the town model's expected detail. No face geometry or texture change was made
based on the report. This does not close the existing lighting/brightness work.

Evidence: `build_usa/face_native.ee`, `face_perf/pcs_field.ee`,
`face_perf/mesh_reports.json`, `face_perf/face_comparison.png`. The comparison
enlarges a crop of PCSX2's `20261105203331.png` and the user's supplied native
capture. Their original capture resolutions and filtering differ.

See [native presentation performance](vp2_native_performance.md) for the separate
GPU texture-cache fix and correction of the FPS overlay's measurement source.

## Objects

- Model objects: ctor 0x23F0A0 (base class vtable 0x154650, ~18 subclasses); vtable +0xFC draw 0x23A8D0, +0xF4
  0x23E820 (buffers). Per-frame update 0x23C640: 0x23E420 (flags), 0x23E050, 0x23CB20 (textures), 0x23A0F0 (cull and
  sort), then draw. [V]
- Fields: +0x90 world matrix (4 rows, translation in .w of rows 0..2); +0x220 the object's VU1 block (row r at
  +0x230 + 16 r: 9..12 the world matrix, 15.w fog, 16.z cull sign, 17 / 18 FBA / TEST); +0x644 resource (+0x8 data);
  +0x648 skeleton; +0x734 material palette (256 bytes an entry); +0x77C visible list; +0x7A4..7A8 group masks; +0x7BC
  model. [V]
- Model = the 'FPS\0' chunk + 0x10: +0x08 texture parts (FIS), +0x34 mesh count, +0x38 material table {count,
  qword offsets}, +0x57 lighting mode 0..3. First mesh at model + 0x60, next = cur + s16 at cur + 4. [V]
- Visible list (0x23A0F0, VU0 micro 0x1F0 / 0x188 culling): 12-byte entries {mesh, material block, depth}, opaque
  first, then blended meshes (strip tag ABE) by depth. Block = mesh + 0x40 when the mesh's flag bit 0 is set (or
  palette[idx - 1] when block + 0x5C is non-zero), else the previous mesh's. [V]
- Camera: camera + 0x650 / 0x690 / 0x6D0 = V / P / C (VU rows 48..59); 0x23A0F0 culls with camera + 0x2F0. [V packet,
  I roles]
- Skeleton S = obj[0x648]: bone count S[0x34][4], bone nodes S[0x3C][i] (world matrix at +0x90), per-bone matrices
  S[0x40] (bind inverse [I]), palette S[0x4C] + 0x40 + buf * n * 64 (double-buffered, buf = S[0x50]) filled by
  0x237020 (VU0 micro 0x898). [V layout]

## Meshes in the files (rmac, dnal, lfvw, lctp...: 24157 models, 335398 meshes)

One VIF packet a mesh, e.g.: UNPACK V4-16 x7 @0 (header), S-8 @0x16 (strip vertex counts), STCYCL 4/1, then per vertex
(4 qwords): +0 normal V3-32, +1 UV V2-16 (12-bit fixed, 4096 = 1), +2 colour V4-8 (128 = 1), +3 position V3-32.
Variants: V3-16 normal and position (set-up routine 8); constant UV / colour through STMASK / STROW; rigid skinning
with position V4-16, .w = VU bone address 0x290 + 4 * bone. Header: q0 (size, vertex base, strip count, strip list),
q1.xyz bounding centre, q2.x flags (bit 0 carries a material block, bit 3 forces the material path), q2.y vertex
count, q4 / q5 set-up routine data, q6.y group mask. [V]

Material block (16 qwords, VU rows 7..21): ALPHA_1, TEX0_1 templates, strip tags (PRIM, e.g. 0x5C tri-strip IIP TME
ABE), flags (.x cull off, .z texture index), untextured alternates. The texture is chosen by the table index at
block + 0x98 through 0x23CB20. [V]

## Native plan

- Offline (tools, output in build/, never committed): every mesh and material block of every model, keyed by where it
  is in the file (the asset part and offset).
- Runtime: the per-mesh loop of 0x23A8D0 (or the visible list of 0x23A0F0) gives the mesh and block pointers; where the
  game loaded that data gives the file and offset (its own disc reads and decompression, see port/vp2s_textures.cpp);
  the object's world matrix, the camera, the bones, the lights and the texture choice come from the structures above.
- Still to write natively from VU0 microcode: the culling (0x1F0 / 0x188) and the skinning product (0x898).

## Drawing from the game's data (the runtime spec)

[V] unless marked. The hook is the draw method 0x23A8D0: a0 the object, a2 the camera (the scene pass 0x2211E0 picks
`view = obj[0x80] ? obj[0x80] : scene[0x4080]`, camera = view[0x94]), a3 the render context. The object draws only when
`obj[0x7C3] & (1 << obj[0x7C5])`.

- Screen position: `s = M330 * V * O * (x, y, z, 1)` with O = obj+0x90 (4 rows, translation in .w of rows 0..2,
  applied M * p), V = cam+0x2F0, M330 = cam+0x330 (P transposed); X = s.x / s.w, Y = s.y / s.w (GS 12.4 space, 2048 the
  centre), Z = s.z / s.w (larger nearer). Clip: `c = M370 * V * O * p`, inside when |c.x|, |c.y|, |c.z| <= c.w. Fog
  (object row 15.w = obj+0x32C on): clamp(cam[0x798] + cam[0x79C] / s.w, 0, 255).
- Meshes: every mesh is in the visible list (nothing culled): with obj[0x7C2] != 0 and obj[0x77C] != 0, pointer i of
  obj[0x77C] + 12 n (n = model[0x34]) -> {mesh, block, depth}, opaque meshes first in file order, then blended ones
  far to near; else the meshes in file order. A mesh without its own block keeps the previous one's.
- Group masks (model[0x59] != 0), group g = u16 at mesh+0x36: g & obj[0x7A4] hides the mesh; g & obj[0x7A6] tints it
  by (obj[0x7E0], obj[0x7E4], obj[0x7E8], obj[0x7EC] * obj[0x7C4] / 128); g & obj[0x7A8] calls vtable +0xF8 for GS state.
- Material block rows (row r at blk + 0x10 + 16 (r - 7)): ALPHA blk+0x20, strip tag PRIM = (u32 blk+0x74 >> 15) &
  0x7FF (ABE bit 0x200000 of that word), cull flag blk+0x90, texture slot blk+0x98, CSA blk+0x9C; palette index
  blk+0x5C (1-based: the model's material table entry). Object fade (obj[0x210] != 0): ALPHA = *(0x314060 + 8 obj[0x210])
  with ABE set, back faces drawn.
- Texture by the game's key, not TEX0: G = *(u32 *)0x35DE38, e = *(s16 *)(G + 0x2670 + 2 slot), key = (tag *(u32 *)(G +
  0x1008 + 16 e), index *(u32 *)(G + 0x8 + 16 e)); written by 0x29D8F0 / 0x23DA80 during 0x23CB20, right before the draw.
- Object rows (row r at obj + 0x230 + 16 r): 1..4 light directions, 5..8 light colours, 13 object colour (= obj+0x7E0),
  14 lighting flags, 19 ambient; point and spot lights from the render context (ctx+0x210.., ctx+0x290..).
- Bones: n = *(u32 *)(S[0x34] + 4), matrix b at S[0x4C] + 0x40 + 64 (n * (u8)S[0x50] + b), row-vector form (p' = x R0 +
  y R1 + z R2 + R3; normals w = 0), giving object space. Routine 2: b = (position.w - 0x290) / 4; routine 4 entries the same.

## Mode 3 lighting (2026-11-04)

The native model draw now applies standard table-D mode 3 for objects whose fourth light colour is zero
(including Alicia in the Waiting Area). Previously these materials kept their stored RGB unchanged.
Material row 12 (`block+0x60..68`) weighs ambient, diffuse and specular; row 9.w (`block+0x3C`) gives
the number of specular squarings. The normal and position are skinned first; the eye and directional lights
are transformed into object space. The specular dot is squared without first clamping it at zero, as in
the game's calculation. The object alpha continues to apply, including zero-alpha weapons outside battle.

PCSX2 slot 2 EE memory: Alicia object `0x996040`, model `0xBD0C10`, meshes 115..117 use mode 3,
weights `(1, 0.3, 0.553340971)`, power 2. Offline evaluation from this state's bone/light/material data
produces RGB values spanning approximately 34..109, rather than the stored constant 64.
The USA build and the guarded Logun-to-inn route passed after the change.

Still unsupported here: table-A mode 3 (point/spot lighting), and mode 3 with a nonzero fourth light
whose treatment differs between library builds. These retain the existing stored-colour fallback.
The broader Waiting Area brightness discrepancy remains open; this change implements a missing material
calculation and does not introduce a global colour or exposure correction.

## Cloak back faces (2026-11-04)

The model draw honours material row 15.x == 0: reject back-facing triangles after skinning,
using the signed area in the game's screen coordinates (Y down), with alternating strip winding.
Positive area and positive zero are rejected, as in the original mesh calculation. Materials 1/2
retain both sides. Object fade (the byte at `+0x210 != 0`, loaded with LBU by the game) retains both sides, and shadow-volume collection
keeps every caster face. The guard-volume rejection described below runs before back-face culling.

Separating evidence for the hard grey triangles on Alicia's left shoulder in the opening:
the defect remains with shadow volumes disabled and with mode 3 disabled. Flat mesh colours,
with post effects disabled, identify the upper patch as event asset `00a9771da0_000+0xD8E40`
(model `+0xD5E10`, mesh 15), the lower patch as `+0xD84A0` (mesh 12). Both have cull flag 0,
mode 1, and two-bone skinning. Nearby outer cloth `+0xDB850` also has cull flag 0.
The native draw previously emitted both sides unconditionally; PCSX2's close-ups have smooth
outer cloth instead of the dark lining faces.

Local diagnostic evidence (build output, not distributed game data):
`build_usa/cloak_probe/no_shadow/native_151.png`,
`build_usa/cloak_probe/mode3_off/native_151.png`,
`build_usa/cloak_probe/exact_ids2/native_147.png` and `exact_ids2.out`.

Verification: USA port build passed. With default lighting/post effects and shadow volumes enabled,
`build_usa/cloak_fixed2/native_148.png` removes both patches. The same 6x6 screen regions have
luminance 65.40 -> 156.67 (upper) and 86.25 -> 160.68 (lower); nearby outer cloth is 174.42 -> 184.77.
The poses differ slightly (face-region RGB mean absolute difference 9.32), so these numbers verify
removal of the hard dark faces, not a matched-frame global brightness correction. The before image
had shadow volumes disabled. Comparison crop and measurements: `cloak_fixed2/comparison.png` and
`comparison.json`. PCSX2 reference torso capture: `cloak_probe/pcsx2/ref_2.png`.
The existing `logun_usa.sh` / `guardrun.sh` route also reached the Waiting Area and inn;
native captures `build_usa/cloak_cull_route/native_39.png` and `native_48.png` retain their
scenery and characters. Both runs used no host input/focus and the existing card snapshot/restore.

## Guard-volume rejection: Soul Street green polygons (2026-11-04)

The mesh program rejects an entire triangle when any of its three vertices lies outside
the camera's separate guard volume. Native model drawing now applies this rule to the
skinned positions: `C = camera[0x370] * camera[0x2F0] * object[0x90]`; a vertex is outside
when any `abs(C.xyz) > abs(C.w)`. Boundaries are inclusive. This is deliberately wider
than viewport clipping. Shadow collection retains every caster face.

PCSX2 slot 1's `vu1MicroMem.bin` provides the reference: `clipw` at 0xC78 / 0xD50,
`fcand vi03,0x3FFFF` at 0xCA8, then the rejected-vertex/ADC path at 0xD80. The existing
mesh prototype also implements this three-vertex rejection; native EE model drawing
previously omitted it and relied on GPU polygon clipping instead.

The user's defect is at Solde's **Soul Street**, beside the staircase. Asset
`006e2785b8_000.rmac`, model `+0x2F2140`, mesh `+0x2F21A0`, has 16 vertices in five
strips (six triangles), untextured PRIM 0x2C, stored RGBA (77,114,77,128). In the saved
native frame it is object `0x13846A0`, mesh `0xFB7C20`, camera `0x1195740`. Its screen
W spans -97.12..203.97. Every triangle contains an outside vertex: six submitted
previously, zero after the game's guard test. GPU clipping had displayed large green
pieces across the left side and foreground.

Verified with an isolated new-game walk left from the Waiting Area. The diagnostic
switch `VP2S_MODELS_GUARD_CLIP=0` reproduces the defect; the default enables rejection.
Before: `build_usa/attacks_probe/stairs_before/shots/native_111.png`; after:
`stairs_after/shots/native_142.png`. A green-colour mask counts 85,862 -> 37 pixels,
the left 180-pixel zone 50,273 -> 0, and the foreground zone 32,555 -> 7. Camera poses
differ slightly; these measurements establish removal of the large polygons, not
pixel parity with a matched PCSX2 screenshot. The saved PCSX2 microcode supplies the
clipping reference. Evidence and measurements: `build_usa/manual_visual_bug/`.

Repeat the route using `tools/attacks_menu_probe.py --script-file <timed pad script>`;
append `135:LEFT/12000` after the normal opening-skip presses. The probe uses its own
card tree and no host input. `PS2X_WINDOW_OFFSCREEN=1` creates its window hidden,
disables focus-on-show, moves it off-screen, then reveals it without activation.
This avoids the startup focus incident caught by the probe's foreground guard.

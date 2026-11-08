# VP2 models: what the native renderer draws from

VP2's draw code does not build vertices: for each visible mesh it puts a DMA REF to a prebuilt VIF packet inside the
model file (and one to a 16-qword material block) into the VIF1 chain. The geometry and the materials exist only as
those packets, so they are converted offline from the user's disc (rule decision 2026-10-31, as for the textures) and
loaded by the game's own pointer; everything else is read from the game's EE structures. Addresses are mod001. [V]
verified in the MIPS or the assets, [I] inferred.

## Bozo native model submission (2026-11-07)

Bozo procedural feedback geometry (2026-11-08) is captured from scalar CPU
records in `0x2476b0`: fixed destination, UV, packed colour and scalar triangle
rejection. `native_procedural_draws.cpp` supports class `152d50`, strips/fans and
compact texture key `4674404c/1`, backed by the successfully published native
feedback image of the current frame. It reads no output packet or live texture
register words. The original allocation establishes modulate with texture alpha
disabled; original scalar flags provide depth/alpha behavior. Immutable batches
submit before enqueue, followed by actual compiled scene completion. X/Y fixed
units divide by 16; Z retains raw depth units. Twelve hidden GPU cases measure
576 colour/coverage/depth samples. Completed field/Logun and matched PCSX2
parity, including sampler edges, remain unqualified.

Bozo particle quads (2026-11-08) use `native_particle_draws.cpp`, independently
of model mesh conversion. CPU points (48 bytes) and template kinds (80 bytes)
supply geometry, UVs and colour; camera/view/object records supply transforms.
Fixed asset template material words are converted **offline** to `VPP1` named
semantics by `tools/vp2_bozo_particle_materials.py`; live template material words
are deliberately poisoned in tests. Runtime FIS identity comes from the compact
game descriptor and must match the selected texture-manager slot. The adapter
requires complete decoded-part template provenance, completed buffer counts,
class `154180`/mode 3/group 1, no shared/fade variant and unit texture Q. It
submits before enqueue and executes the original scene completion. Unsupported
variants reject; no VIF/DMA worker is run. Six hidden GPU cases measure quad
coverage and partial-alpha colours; real Logun/PCSX2 parity remains open.

Zero-mesh FPS models are now admitted as empty native submissions (2026-11-08).
The live object `0x12a6d50`, class `0x1546d0`, uses model `0x100de90`, with decoded
part provenance `a94b8b44/0+348010`. The disc's FPS chunk at `+348000` independently
has mesh count zero. The four original mesh-loop branches in `0x23a950` test
count > 0 and converge at `0x23bfe0` without reading a mesh for that count.
Admission requires scene/camera ownership, an aligned FPS signature and complete,
contiguous decoded-part provenance across the entire 0x70-byte chunk/model header.
Unknown holes, mixed origins and unfinished writes reject. No geometry, material,
skeleton or lighting state is consumed for an empty submission; the original
scene completion still follows. 384 existing captures plus 96 empty cases and
33,554,432 per-byte provenance comparisons remain green. This advances the actual
route to particle lighting at `0x22f300`; field/Logun is still unqualified.

The optional working-model probe at `object+0x630` first qualifies an aligned,
complete RAM header through the admitted game RAM aliases (2026-11-08). The
slot is not uniformly a pointer: original getter `0x22cfb0` also exposes an
inline record at that offset. The first field capture, object `0x9a2060`, class
`0x1546d0`, had `0x3f7ffffe` there; the old probe attempted `+0x34` as an address.
Its actual seven-mesh model has completed part provenance `e405ef34/0+41340`.
Non-model values now leave the optional probe inactive; valid editable copies
still require the existing count/first-mesh header match and use direct live
vertex sources. No arbitrary value is wrapped into RAM.

All 384 native model capture cases retain geometry/skin/colour/RAM checks while
covering genuine copies through five RAM alias forms, non-pointer values,
truncated/unaligned headers, original-model identity and mismatched headers.
Model and startup suites pass. The actual route passes this submission and
stops at `0x248570`. Its captured partial frame is entirely black (RGB means
zero), despite 155 queued batches / 4,683 vertices; this is not visible field
parity or Logun qualification.

The scalar bone-palette loop `0x2371dc..0x2372d0` also builds native matrix
records directly (2026-11-08). Its root inverse, bone transforms and bind matrices
feed two ordered compositions and a transpose at the original output stores.
The original loop and both buffer selections remain. Pipelined source/output
overlaps reject; 512 whole CPU/RAM loop comparisons against independent integer
arithmetic plus mutation-free rejection cases pass. This advances actual New Game
to `0x1fedd4`; it does not yet qualify rendered field lighting.

The Bozo runner replaces the model enqueue at `0x239fec` with native batch
submission followed by the original scene completion. `bozo/native_models.cpp`
reads game records and immutable semantic VPN1 assets converted offline from
VPM3 by `tools/vp2_bozo_meshes.py`. No runtime VIF/GS parsing or PS2Recomp service
is used. The partial title route draws real models; unsupported algorithms and
light tables still reject. This does not qualify field lighting or Logun yet.

## Editable vertex sources without runtime VIF interpretation (2026-11-05)

The old `vertexSources()` helper reconstructed VIF writes at runtime to find the
raw float positions/normals in a model's facial-animation work copy. That was a
runtime hardware interpretation path, despite the surrounding native renderer.
It is removed. `tools/vp2_meshes.py` now tracks source offsets during offline
conversion and appends them to each mesh in **VPM3** packs. The renderer reads
the game's working vertices directly through this metadata. Constants, replicated
scalars and packed coordinates use an unavailable-offset sentinel; packed format
routines 6/8 retain their existing converted-vertex handling.

VPM2 packs are rejected with a regeneration message, with no parser fallback:

```powershell
python tools/vp2_meshes.py --assets build_usa/extract/assets --out build_usa/vp2_meshes
python -m unittest discover -s tools -p test_vp2_meshes.py
```

The complete USA conversion was checked offline against both the previous VPM2
records and the removed C++ helper: **5,139 packs, 1,348,830 meshes, 39,404,365
vertices and 48,017,159 available raw xyz sources**, with zero geometry/material
record or source-offset differences. The 717,841 meshes using format routines
6/8 retain the existing exclusion from live float reads. Five synthetic tests
cover relative offsets, an edited work copy, mask constants/retention, scalar
replication and packed coordinates. Local verifier inputs/results are in
`build_usa/native_mesh_offsets/` (game data, never committed).

The USA build passed, and a private-card/off-screen replay reached the native town
sequence after skipping the opening movie through its private input file. The
offset path was used for 90 positions in three meshes of object `0x11A6EC0`
(work model `0x11A9980`, asset `00a94b8b44_000`). An offline decode of the captured
working model agrees with all 90 direct reads. Those positions still matched the
disc at the captured instant, so this is work-copy path coverage, not proof of
facial-animation parity; the synthetic edited-copy test covers changed values.
There were no rejected mesh packs, and user-card hashes were unchanged. Captures,
the EE snapshot and `live_vertices.json` are in the same local evidence directory.

This removes the editable-vertex command interpreter only. It does **not** remove
the VIF1 music chains, VIF0/VU0 execution core, IOP device environment or other
hardware transfer dependencies. Those remain separate native-replacement work.

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
capture. Their original capture resolutions and filtering differ. This geometry
check did **not** verify the reported colour: the follow-up below identified and
fixed an incorrect shadow applied to her face.

See [native presentation performance](vp2_native_performance.md) for the separate
GPU texture-cache fix and correction of the FPS overlay's measurement source.

## Character colour and shadow receivers (2026-11-05)

The native shadow volume darkened Alicia's face even though the game excludes
her surfaces from its shadow-receiver mask. The coarse caster hull then produced
a flat brown patch across the face, hiding the skin shading. Disabling shadows
restored the colour and separated this from geometry, texture selection and lighting.

PCSX2's Waiting Area GS dump `20261105204518_(2).gs.zst` provides the rule:
matched face vertices have alpha **127**, while the shadow pass uses `TEST=0x5C00D`
(`DATE=1`, `DATM=1`) and darkening alpha **63**. The shadow applies only to receiver
surfaces whose stored alpha has its high bit set; opaque character surfaces with
127 are excluded. The native renderer had forced opaque alpha to full scale.

Native model materials now opt into `Material::writeSourceAlpha`: RGB blending
is unchanged, but their source alpha is retained in the off-screen scene, including
for opaque draws. The shadow material opts into `shadowReceiverAlpha`. The renderer
snapshots the scene colour/alpha before the shadow pass and restricts darkening
to its full-scale receiver alpha (128 maps to 1; 127 remains below it). Sampling
uses a separate texture to avoid an attachment feedback loop. The volume count and
depth tests still produce the ground shadow. Both options default off for other
materials and ports. Darkened receiver alpha is also lowered below the receiver
threshold, preventing a second caster from darkening the same pixel again.
No GS state is read or replayed by this path.

Local evidence: `build_usa/attacks_probe/face_shadow_off/shots/native_46.png`,
`face_receiver_fixed/shots/native_46.png`, and `build_usa/face_perf/` analysis files.
The fixed run retains normal shadows. In the identified 4x4 face region
`(193,190)-(197,194)`, luminance is **62.38** before, **107.79** with shadows disabled,
and **107.45** fixed. The ground strip `(168,348)-(184,352)` remains darker than
the shadows-disabled run by mean RGB **(18.09,18.03,14.14)**, confirming that shadows
were not simply removed. Captures have slightly different idle phases; these are
local defect-removal measurements, not registered whole-frame parity.

The normal USA pipeline build passed. The final replay using the existing Logun
script's movement sequence reached the inn with normal lighting, floor blends and
characters (`build_usa/attacks_probe/shadow_receiver_inn/shots/native_45.png` and
`native_58.png`). It used independent cards, no host input/focus, and muted audio;
there was no missing guest target. Periodic screenshot capture still introduces
measurement hitches, so this replay is a visual regression check.

A separate offline check also found that the current mode-1 calculation clamps
the fourth light where the saved field microcode keeps it signed. Allowing the
signed term closely reproduces the captured vertex colours, but the relevant
library variants still need verification before changing that calculation.
This finding was not mixed into the receiver-mask fix.

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

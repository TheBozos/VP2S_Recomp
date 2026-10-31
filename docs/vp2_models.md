# VP2 models: what the native renderer draws from

VP2's draw code does not build vertices: for each visible mesh it puts a DMA REF to a prebuilt VIF packet inside the
model file (and one to a 16-qword material block) into the VIF1 chain. The geometry and the materials exist only as
those packets, so they are converted offline from the user's disc (rule decision 2026-10-31, as for the textures) and
loaded by the game's own pointer; everything else is read from the game's EE structures. Addresses are mod001. [V]
verified in the MIPS or the assets, [I] inferred.

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

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

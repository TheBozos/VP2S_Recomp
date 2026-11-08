# BozoEngine migration: initial native bring-up

Started 2026-11-06 after VU0 collision-point increment `d99b892`. The existing
PS2Recomp launcher remains the playable baseline. **The Bozo runner is not
playable: it renders native movies, title models and the skip dialogue, but its
gameplay and Logun route remain unqualified.** It is an independent compiled
target, not a replacement DLL loaded into PS2Recomp.

User clarification (2026-11-08): the recent unstable-sky and camera-dependent
lighting reports concern DC, not VP2S. They are not VP2S migration follow-ups.
The earlier VP2S face/shading and general brightness comparisons remain open.

## Build and evidence

### Native model/global parameter combination (2026-11-08)

The original scalar branch block `0x23c240..0x23c2d0` retains its `+0x518`
priority over `+0x517`. Native operations either multiply `+0x310` by the
model's `+0x7e0` record, or multiply `+0x3c0` / `+0x390` by that model record
then add `+0x310` with the original truncated reduction. Inputs are captured
before the original stack output, including aliases. Producer ABI, numerical
domain and product/sum bounds remain guarded.

512 whole CPU/RAM cases pass independent integer arithmetic across all flag
combinations and four modes, including cancellation, tiny products, overlapping
inputs/output and mutation-free rejection. Bone/record, source-guard and startup
suites pass. Actual New Game now passes this calculation and reaches native
model submission, where an optional working-model probe reads a non-RAM value
as an address. That resource probe is under investigation; Logun remains open.

### Native camera matrix scale (2026-11-08)

The two `0x20f240` / `0x20f280` blocks read their own ordinary source matrix
(`+0xd0` or `+0x150`) and scale record through `fp`. Native arithmetic scales
each complete column by its corresponding coefficient into `+0x1d0`.
All 16 source components and four coefficients are captured before publication,
including scale/source/output aliases; no inherited vector state supplies data.
Original scalar source selection, branches and continuations remain compiled.

1,024 whole CPU/RAM block comparisons pass independent integer products, both
branches, all four modes, signed zeros, tiny products, ordinary/scratch records
and overlapping coefficients. Invalid producer pointers, NaN/subnormal inputs
and overflow reject without mutation. Actual New Game advances to model/global
parameter composition at `0x23c2bc`. All 58 rebuilt Bozo suites pass, including
the expanded visibility snapshot and all downstream consumers. Logun and
rendered field parity remain open.

### Native four-plane angle classification (2026-11-08)

The remaining `0x220c30` branch consumes four further frustum origins/normals
(`6..9`) and the sphere captured before the original virtual callback. Native
code computes the original ordered plane distances minus radius, then returns
the caller's `0x00`/`0x80` classification directly at `0x220c74`. It does not
construct a vector register, status bank, sticky state or timing result. The
original scalar draw-list construction and visibility-byte store remain.

The input sphere is now an immutable part of the per-invocation snapshot. Both
the source sphere and live camera records may change without changing that
classification. Duplicate calculation, result consumption before publication,
wrong object/source and consumption after release reject. Original caller and
the complete offline angle algorithm range are fingerprinted.

1,620 independent axis-plane geometry cases plus signed-zero cases pass across
four modes. All 256 original scalar CPU/RAM comparisons now complete, including
150 angle branches previously stopped before the unported call. Existing 5,488
sphere geometry cases remain. Lifetime, source mutation, arithmetic and startup
checks pass. Actual New Game advances to camera matrix scale `0x20f288`.
Logun and rendered field parity remain open.

### Native model colour/alpha product (2026-11-08)

Mod003 `0x43d280` multiplies its ordinary `+0xd0` / `+0xe0` four-component
records into the original stack temporary. The original scalar loop retains
its optional flag update: type `0x110` consumes W at `+0x874`; other nodes
receive the complete record at `+0x560` before their original virtual update.
Native calculation snapshots both inputs and preserves truncated products,
signed zeros and tiny-result reduction. Nonfinite/subnormal inputs, overflow
and a mismatched producer ABI reject before CPU/RAM mutation.

512 complete original caller CPU/RAM comparisons pass against independent
integer products, covering empty/populated lists, both consumer branches,
retained/replaced flags, output/source overlap, tiny products, stack locations
and four host modes. Iterator and virtual methods are bounded software test
contracts; this does not qualify their independent implementations or rendering.
The real route retains those original methods and advances to the separate
four-plane visibility test at `0x220c30`. Arithmetic, source-guard and startup
suites pass. Logun and rendered field parity remain open.

### Enclosing-object distance caller (2026-11-08)

Mod003 `0x37f994..0x37f9b4` uses the qualified XYZ length operation with its
`t2` input / `t1` output ABI. Original scalar subtraction, temporary copies,
radius addition and maximum comparison remain. 256 extra whole CPU/RAM cases
exercise all four rounding modes and both stack locations, including both
comparison outcomes; total distance coverage is 1,280 cases. The fixture needs
enough scratch-stack space for its original `SP+0x100` store. Source-guard,
arithmetic and startup suites pass. Actual New Game now reaches component
product `0x43d2a0`. Logun and rendered field parity remain open.

### Native box-to-sphere builder (2026-11-08)

Mod003 `0x37fa30` keeps its original scalar setup, midpoint/extent stores and
return ABI. Its vector block now derives the sphere directly from the box's two
corner records. XYZ midpoint uses the original truncated sum/product; radius
retains `squaredLength * (1 / sqrt(squaredLength))`, including the zero-size
case. This differs numerically from replacing the radius with a direct square
root. Corner W payloads are not consumed. The scalar extent writes after native
sphere publication remain compiled with their original rounding behavior.

512 complete CPU/RAM builder cases pass an independent integer product/signed-sum,
root and reciprocal oracle across four modes, signed zeros, identical/reversed
corners and ordinary/scratch records/stacks. Wrong output/coefficient, NaN and
overflow reject before CPU/RAM mutation. The overflow negative case found that
round-toward-zero can return finite `FLT_MAX`; explicit pre-product/sum bounds
are required, rather than `isfinite` alone. All targeted arithmetic, source-guard
and startup suites pass after that correction. Actual New Game advances to
distance block `0x37f994`. Logun and rendered field parity remain open.

### Setup-object distance caller (2026-11-08)

Mod003 `0x382020..0x382040` now uses the qualified XYZ length arithmetic
with its own `a0` input / `v1` output record ABI. Its original scalar position
lookup, subtraction, stack construction and object distance-field store remain.
256 additional whole CPU/RAM caller comparisons pass across four rounding modes
and both stack locations; total field-distance coverage is 1,024 cases. Source
mutations across the new caller reject, and field-distance, signature and startup
suites pass. Actual New Game advances to bounding-sphere builder `0x37faa0`.
Logun and rendered field parity remain open.

### Original bounded string comparison restored (2026-11-08)

The export metadata omitted main `0x134b60..0x134d18`, reached by the field
loader at mod003 `0x43cd0c`. A source-guarded function extent now compiles this
original CPU routine unchanged. There is no new library override or engine
compatibility path. A private standalone fixture verifies all 16-by-16 byte
alignment pairs with 64 bound/terminator/difference patterns: 16,384 cases,
including high bytes and the exact unsigned-byte difference. No device/library
call or vector-unit operation is allowed. Actual New Game now passes that call
and reaches another position-distance block at mod003 `0x382020`.
Logun remains open.

### Native position-pair distance (2026-11-08)

`0x1fedd4..0x1fedfc` now reads its two position records directly, applies the
original truncated subtraction, XYZ squares/sum and root, and writes the original
temporary record. Y/Z squared intermediates and the first record's untouched W
payload remain. The scalar record lookup, index advance and final distance-field
store remain compiled. No vector or status state is read or produced.

512 additional whole CPU/RAM cases compare the retained scalar lookup/store
against independent integer arithmetic, including cancellation, signed zero,
opaque W payloads, input/output aliases, both stack locations and four rounding
modes. Combined bone/distance, source-guard and startup checks pass. Actual New
Game advances to frame 1486 and a missing compiled bounded-string comparison
at `0x134b60`, called by mod003 `0x43cd0c`. Logun remains open.

### Native model bone matrix records (2026-11-08)

The original scalar loop `0x2371dc..0x2372d0` now produces bone matrices from
the root inverse on its stack, current bone records, and immutable bind matrices.
The native operation performs the original two ordered compositions, preserves
the untranslated fourth column, then transposes with the original zero/one
arithmetic. Publication occurs at the original output stores. The scalar loop,
buffer selector and buffer cursors remain compiled; no VU register bank or
previous vector result supplies an input. Source/output overlaps reject because
the original loop pipelines a later input before the preceding output store.

512 complete scalar-loop CPU/RAM comparisons pass against an independent integer
product/signed-sum oracle across 1–11 bones, both buffers, four rounding modes,
signed zeros and normal/scratch stacks. Invalid index/buffer, source aliases and
nonfinite input reject without CPU/RAM mutation. Source guards cover the original
caller and both offline-read math program ranges, including the second MPG header
offset; no such bytes are parsed at runtime. Actual New Game now passes this
dependency and stops at distance calculation `0x1fedd4`. All 57 rebuilt Bozo
checks pass, including startup and the resource lifetime increment. Logun remains open.

### Bespoke FIS resource publication (2026-11-08)

The type-3 producer was mod003 `0x447af0`, class `0x165b40`: a texture
resource, not geometry. Actual completed decoded part 0 has disc provenance
`e408e4a0`, FIS identity `JOHN:272b`; its 256-square atlas already exists in
the offline conversion. Native publication reads only the ordinary FIS identity
and completed file provenance, then owns the immutable asset by game object.
The original decoder, allocation/free, descriptor-building CPU code and two
software waits remain. Both obsolete upload calls now invoke the real
`0x4479e0`/`0x21c9a0` callback after successful publication. No runtime
descriptor input, device job, status response or IRQ is constructed.

Loader entry and destructor entry release the live image association. Failed
reloads cannot retain a previous image; held native assets survive source frees.
64 complete CPU/RAM loader comparisons cover success/failure, prior allocation
release, both source identities, stack locations, nested resumes, retained waits
and original draw/scene completion. The fixture uses the actual privately decoded
FIS for scalar construction with synthetic native texture identities; decoder
and heap dependencies are bounded contracts, not a new decoder qualification.
Additional checks reject missing/wrong-part provenance, unknown keys and wrong
queue ownership before CPU/RAM mutation. Resource, existing texture publication
and source-signature suites pass. The actual New Game route now passes the old
`0x123220` boundary and stops at unported math `0x2371e8`. Logun remains open.

### Native flat scene fades (2026-11-08)

Class `0x161230`, mode 0, now captures the original `0x3d3660` builder's scalar
rectangle arguments at `0x3d38d4`, before packing. Its source dimensions,
colour and depth become an immutable native rectangle, using the separately
verified ordinary alpha blend (offline table entry 2). Other modes/flags reject.
Destruction releases the live association and rejects an in-flight publication.

The exact 96-byte original writer range stays owned through compiled flush
bookkeeping; extra content rejects. Compiled `0x3d34e0`/`0x21c9a0` completion
then runs, with verified epilogue padding represented in static metadata.
No packet input, device job or invented completion is used.

64 new whole CPU/RAM cases retain the entire original builder, original solid
rectangle helper, flush and real completion. They verify native vertices,
colour/alpha/depth, source sizes, both passes, nested resumes, unsupported modes,
extra content and destruction/reuse. Combined writer coverage is 304 cases;
font/writer, rectangle, signature and startup checks pass. Actual New Game
passes the fade and next rejects graphics service `0x123220`, called while
processing a pending type-3 queue entry. Its producer is being traced.
Logun remains open.

### Empty child-writer lifetime and real completion (2026-11-08)

The empty class `0x1653d0` builder emits one state entry and no geometry. Its
draw now owns a guarded 32-byte writer range through the original CPU flush;
extra content or populated children reject. Native materials own that state.
The original writer cursor stores remain, followed by compiled `0x207580`
and `0x21c9a0` completion. Only the transport enqueue retires; no completion
bit is invented. Relocated epilogue instructions use verified padding and the
static function extent explicitly includes them.

Another 64 full CPU/RAM flush/completion comparisons exercise both scene
passes, nested resumed notifications, rejection and released leases. The test
driver now resumes outer compiled callers after a paused nested completion.
The numerical rounding fix materializes four volatile scalar components:
MSVC cannot directly return a volatile `__m128`. The MSVC port build passes;
the independent native rounding comparisons are retained. All 55 rebuilt Bozo
checks pass, including the shared normalization and complete writer lifetime.

Actual New Game reaches a distinct flat fade writer (class `0x161230`, alpha
`0x80`, 512-square bounds) at `0x3d3648`. Logun remains open.

### Empty child-projection setup (2026-11-08)

Actual common `0x2076a0` was called for class `0x1653d0` with child-list next
equal to its own head (`object+0x90`). Its scalar loop invokes no child and
stores no result from the vector matrix setup. Only this guarded empty path
retires its four vector loads and microcall; no vector environment is recreated.
Any populated list or other class still rejects. Scalar matrix preparation,
writer construction and the empty loop remain compiled.

64 cases check complete CPU/RAM preservation at all five sites, and rejection
of a populated list. Math, signature and actual startup suites pass. Actual New
Game passes this setup and next rejects its writer flush from `0x2075c8`.
That state-only writer's lifetime/completion is the next task; Logun is open.

### Field debug-font state-only publication (2026-11-07)

Mod003 `0x3d93f4` now attaches its debug-font flush to the same typed atlas,
writer range and lifetime checks as the existing menu caller. Only the already
accepted constructor's state prefix is admitted: appended glyphs/background
still reject. Original scalar flush/bookkeeping remain, with no device job or
fabricated completion. The initializer/flush CPU/RAM fixture now exercises both
caller register conventions; font, signature and startup suites pass.

Actual New Game passes this prefix-only writer and next rejects a separate
child-projection matrix setup at common `0x207720`. Logun remains open.

### Native camera-position distance (2026-11-07)

Mod003 `0x37b500..0x37b578` retains its scalar subtraction and publishes the
same audited displacement-length record at `0x37b568`. The original scalar
store to camera field `+0x33c` remains. Another 256 complete original/adapted
CPU/RAM consumers pass in four rounding modes, including zero, W payloads,
scratchpad stacks and the actual field store. Combined distance coverage is
768 cases, with 256 inline rotations and 512 independent row-dot callers.
Actual New Game next rejects a debug-font writer flush from `0x3d9400`;
Logun remains open.

### Native field row-dot transform (2026-11-07)

Mod003 `0x410df0` now reads its complete matrix/vector CPU records and publishes
the four row-dot results at the original destination. Reduction order is
`XYZW`, `YXZW`, `ZYXW`, `WYZX` respectively; a generic matrix multiply would
change rounding order. Inputs are copied before output, preserving aliases.
No inherited matrix/vector state is supplied to the adjacent unported helper.

512 whole CPU/RAM callers compare with independent integer significand products
and signed aligned sums in all four rounding modes, with cancellation, signed
zero, scratchpad matrices and output overlapping each input. The combined suite
also retains 512 distance consumers and 256 inline rotation callers. Actual New
Game passes and next rejects another distance producer at `0x37b548`.
Logun remains open.

### Native field-object inline rotation (2026-11-07)

Mod003 `0x379228..0x37928c` retains its selected-quaternion copy to object
`+0x190` and scalar identity fourth-row construction at `+0x1f0`. The already
qualified quaternion-matrix calculation now publishes the first three rows
directly to `+0x1c0`; its unused control read leaves CPU state untouched.

256 additional whole CPU/RAM caller comparisons cover all four rounding modes,
scratchpad stacks, nonunit inputs and signed zero, alongside the existing
512 distance consumers. This qualifies the caller boundary using the separately
qualified numerical API. Actual New Game passes and next rejects the matrix
record/vector helper at mod003 `0x410df0`. Logun remains open.

### Native movement displacement distance (2026-11-07)

The next reached mod003 site `0x3c5d58..0x3c5d78` uses the same audited distance
equations, with a different scalar displacement builder. It now publishes
directly from that builder's CPU temporary. Its original zero/nonzero comparison
and update of movement flag `+0xbc` remain compiled.

The combined distance fixture now passes 512 full CPU/RAM cases, including
another 256 complete movement consumers, exact zero, nonzero displacements,
other-flag-bit preservation, W payload bits and all four rounding modes. The
oracle preserves its own host FP state during binary search through subnormal
candidates; outgoing CPU FP state is still compared exactly. Actual New Game
passes this site and next rejects inline quaternion-to-matrix `0x379234`.
Logun remains open.

### Disabled frame-filter lifetime (2026-11-07)

The original `0x25aa40` builder branches directly to its epilogue when `+0xaf`
is zero, before reading format, source or reduction mode. Native capture now
invalidates an older enabled snapshot and publishes an owned empty result for
that branch. Submission still checks pending scene ownership and executes the
real original completion; no frame copy or reductions are drawn. Re-enabling
requires a newly completed builder and cannot reuse the empty result.

256 enabled plus 256 disabled snapshot/lifetime cases pass without CPU/RAM
writes, including ignored inactive mode/source fields and stale-state rejection.
Actual New Game reaches the next vector distance use at mod003 `0x3c5d58`.
Logun remains open.

### Scaled source coordinates and display fields (2026-11-07)

The native composition guard incorrectly required scaled source size to equal
display size. The game's source-scale setter `0x117b00` changes source scales
independently; `0x116780..0x1167bc` records one or two field heights according
to video mode. The guard now checks each display view against that unscaled
base and field factor, while native Screen retains the scaled coordinate span.
The renderer already maps that span directly to its output viewport. No field
copy or hardware surface is reconstructed.

128 original/adapted composition CPU/RAM cases cover both field factors,
identity/gameplay scales, pair selection, resumed waits and lifetime; invalid
dimensions/field pairs still reject. Composition, signature and actual startup
suites pass after correcting an initial assumption caught by the startup test.
Actual New Game passes and next rejects frame-filter object `0x155d50` with
its enable flag zero. This is not a rendered field or Logun parity claim.

### Native field-loop pacing ownership (2026-11-07)

Mod003's field loop now retires only timer0 HBLANK-mode setup at `0x3d9238`
and the paired counter reset at `0x3d95b0`. The loop has no timer-register
read; its actual pacing/query calls remain through already ported `0x11f5a0`
and `0x11f590`. Event waits, UI work and the field update remain compiled.
No counter or completion value is provided. Guard mutations, native frame-event
fixtures, field distance and startup checks pass (four targeted suites).

Actual New Game passes the timer setup and reaches a distinct source-frame
layout guard: logical source `512 x 512.95996`, display fields `640 x 224`.
Scaled source-to-display composition is being traced; Logun remains open.

### Native field position distance (2026-11-07)

Mod003 `0x378908..0x3789c8` retains its scalar position subtraction, temporary
copies and object-field consumers. The vector distance tail publishes directly
from the completed CPU displacement record at `0x3789a0`. The native calculation
uses the game's truncated product/sum/root order; Y/Z squares and the untouched
W bits are preserved in the temporary without any vector state.

256 full CPU/RAM cases compare the original scalar builder and consumers with
an independent exact-product/sum and binary-search-root oracle, in all four
rounding modes and with scratchpad stacks. Actual New Game passes this routine
and next rejects a timer-mode write at mod003 `0x3d91f0`. Logun remains open.

### Native axis-angle quaternion normalization (2026-11-07)

Common `0x215640` retains its scalar sine/cosine call and XYZ products. The
completed quaternion at the original destination is normalized directly at
`0x2156b0`; the unused control read supplies no invented status. No vector
register state or microprogram is used.

An independent scalar oracle caught a one-ULP error: GCC placed the final
multiply after restoring MXCSR. Materializing the result inside the rounding
guard fixes that ordering; strict rounding compilation is enabled for the
native math translation unit. 256 complete CPU/RAM cases cover four rounding
modes, signed zero, nonunit axes, source/destination aliasing and scratchpad
records/stacks. Sine/cosine is a shared bounded dependency in this fixture and
has its own original-builder qualification. The fresh actual route passes this
helper and next rejects mod003 distance calculation at `0x378980`.
All 54 rebuilt Bozo checks pass after the shared normalization change.

### Retired physical field-buffer fill transport (2026-11-07)

Mod003 `0x3d5670` fills a 256 KiB EE buffer and uploads it repeatedly to physical
graphics memory. Native rendering has no such memory destination: native frame
and auxiliary targets initialize their own colour/depth/stencil. The specific
upload and its two sync calls retire; allocation/failure, CPU fill, descriptor
construction, visibility flushes, game cache reset and free remain compiled.
No image owner, transfer acknowledgement or completion bit is invented.

32 original CPU cases check allocation failure, all 262,144 filled bytes,
exact descriptor/visibility counts and reset-before-free lifetime. Physical
upload/sync and device I/O reject in the fixture. Actual New Game passes the
routine and next rejects common math at `0x2156a0`; Logun remains open.

### Native internal-record placement (2026-11-07)

Common `0x30ce70` mixed timer0 with the original scalar RNG only to select one
of sixteen internal payload offsets. At `0x30ce84`, native per-run entropy now
publishes the final payload pointer. The original pointer store at `+0x188`,
0x78-byte clear and field initialization remain compiled; the payload fits at
all offsets before that pointer field. No counter value is supplied. The
original RNG call and its sequence updates remain.

Another 1,024 deterministic placements check every allowed offset, both source
alignments, complete CPU preservation except the pointer result, bounds and
per-run isolation. Combined allocation/progress checks cover 2,048 entropy
choices and 256 schedules, without RAM/device writes. Actual New Game passes
this routine and next rejects the physical graphics-memory fill upload at
mod003 `0x3d579c`. Logun remains open.

### Retired timer-only record-initialization delay (2026-11-07)

Mod003 `0x3d7718..0x3d7764` delayed between 36 calls to the synchronous record
initializer `0x2f8770`. Each initializer returned before the timer-only loop;
the loop checked elapsed timer0 counts and called FlushCache, with no job/status
completion test. Only that delay retires. All 36 initializers, one visibility
flush per record and the original UI predicate/update/render calls remain.
No counter value, ready bit or fictitious completion is provided.

A compiled original-loop fragment with bounded synthetic record callbacks passes
32 cases: 1,152 constructor, visibility and UI-predicate calls, 576 UI updates,
the original loop state and whole RAM match independent expected work. Device
I/O rejects in the fixture. Actual New Game passes the loop and next rejects a
different timer read at `0x30ce84`; Logun remains open.

### Native allocation entropy and loading UI schedule (2026-11-07)

Common initializer `0x306950` used timer0 in two unrelated ways. At `0x306a10`,
the timer mixed into an allocation placement whose only consumer is a four-bit
index, shifted by four before adding it to the real allocation. The allocator
overwrites the old incidental a2 input. This now receives a bounded per-run
entropy choice from `SeedOwner`; all original scalar RNG updates, allocations,
file loading, decoding, pointer publication and frees remain compiled. No timer
count or counter result is supplied.

The second use gated loading UI callbacks while the original `0x103870` request
poll reports pending. `NativeLoadProgress` owns a schedule per original stack,
worker and destination. It allows UI updates at native field cadence, checks
clock/record lifetime, and releases only after the actual poll returns completed.
The original poll, callbacks and completion branch are unchanged. Counter loads
and their dead arithmetic retire; no file completion, ready bit or IRQ is made.
This is a native UI cadence, not an implementation of the original timer rate.

256 schedule lifetimes and 1,024 deterministic allocation choices pass without
RAM/device writes; invalid/repeated requests, changed ownership, reversed clocks,
overflow and premature completion reject. The signature and 1,380-frame startup
checks also pass. Actual New Game now returns from this initializer and next
rejects a separate timer busy-wait at mod003 `0x3d7708`. Logun remains open.

### Compiled field/menu overlay (2026-11-07)

The default pipeline now includes mod003 alongside main/mod001/mod002. Its
4,200 primary functions and 21,229 metadata entries are compiled from the
user's extracted ELF, with original load bytes unchanged and a distinct resident
identity. No runtime instruction decoding or device/VU fallback is introduced;
the image retains 33 explicit untranslated VRNEXT boundaries.

Actual New Game now passes the former `0x48df80` initializer stop and next
rejects a timer-register read in common code at `0x306a10`. The caller uses it
in an allocation/initialization path; its consumers require analysis before
choosing a native replacement. Logun is not reached.
All 51 project checks pass with mod003 included.

### Native selection-highlight publication (2026-11-07)

Class `0x15ac30`, builder `0x30ee00`, captures its six scalar integer coordinates,
alpha and depth before packing. Its native snapshot draws three untextured
gradient strips in original order, retains cached geometry and releases it at
destruction. No geometry is recovered from packet output. Another 64 complete
builder CPU/RAM comparisons check coordinates, per-corner alpha, cached rebuild
and immutable lifetime, with a shared bounded integer-conversion callback.
Invalid, repeated and incomplete captures reject without changing RAM.

Actual New Game completes 1,380 frames and captures the complete skip dialogue:
81 batches / 486 vertices at 640x448. Confirming Yes reaches frame 1446, then
requires compiled mod003 code at `0x48df80`; the initial migration build still
contains only main/mod001/mod002. This is visible native dialogue, not Logun
or PCSX2 pixel-parity qualification.
The window/selection builder, signature and scripted startup checks pass.

### Native short sine (2026-11-07)

The next story/skip dialogue leaf, `0x2144c0`, now evaluates its degree-nine
sine polynomial directly from the game's coefficients. It reduces the original
angle by integer multiples of pi; `0x35dabc` receives the consumed quadrant and
`0x35da8c` receives the scalar result. Original parity branches and scalar return
remain compiled. The temporary scalar sign store at `0x214540` retires because
the original leaf's final store restored the old coefficient there. No inherited
vector scratch bytes or register state are reproduced.

768 additional complete CPU/RAM cases cover signed zero, quadrant boundaries,
adjacent floats and four host rounding modes. The combined trig check passes
1,920 complete CPU/RAM cases and 196,608 independent polynomial comparisons over
76 sites. Actual New Game passes the leaf, captures the window and three text
elements, then rejects selection-element class `0x15ac30`. Dialogue presentation
and Logun remain open.

### Native dialogue window snapshots (2026-11-07)

Class `0x15a1b0` now captures its original scalar `0x2e9640` builder: the
`0x2e9e90` background's rounded grid and colours, eight edge-shading strips from
the game's ordinary topology table, then the `0x2e9ce0` / `0x2ead10` border's
scalar corners, depth, per-part palette choice and integer UV coordinates.
Captures occur before packet packing; packet output is never read. Original
CPU builders and their built flag remain intact. Scroll controls still reject.

Actual separating evidence: null FIS, live DTT `0x67e200`, nine border parts,
and an existing constructor-owned native atlas lease, button `0x27` / `SHI:3D`.
The border reuses that lease and validates its live DTT/parts and quad-list
ownership. No hardware-address mapping or draw-time source guess was added.
Immutable window snapshots retain their pictures through cache rebuild and
destruction. The layer admits only a completed snapshot; the destructor releases
the association. Native submission preserves background/edge/border order.

64 complete original/native builder cases leave CPU/RAM identical and check
both background palettes, alpha, size thresholds, alternate rectangles, all
background/border enable combinations, integer geometry/UVs, cached rebuild,
null source pointers and retained picture lifetime. Actual New Game passes the
window and next rejects `0x2144d0`, the short-sine routine. Window presentation
and PCSX2 parity remain unqualified until that subsequent producer completes.
The full run passed its other 50 checks; after updating the explicit native-site
allowlist, the remaining signature check also passes (51 checks qualified).

### Scalar UI transition and story layer (2026-11-07)

The analysis export ended `0x2ea350` at its computed jump, omitting branch
`0x2ea380` and seven following branches. Its nine-target table at `0x357960`
and complete scalar body through `0x2ea6f0` are now signature-checked offline;
metadata exposes the whole function to static compilation. Load bytes are intact.

The story/skip dialogue layer's exact `0x15bca0` / `0x15bcbc` / `0x15bd38`
tuple now uses the existing native child submission. Its original build, draw
and completion methods are the base methods via mod002 adjustment thunks
`0x371fe0`, `0x371ff0`, `0x372000`. The vtable is signature-checked. Unknown
child types and incomplete snapshots still reject; mixed type tuples reject.
The 64 complete layer CPU/RAM cases now cover both base and story tuples.

Actual New Game passes the switch and layer, then rejects uncaptured window
element class `0x15a1b0`. The visible partial framebuffer there is the opening
movie landscape; the dialogue itself is not yet drawn. Logun remains open.

### Direct native streamed-music publication (2026-11-07)

`bozo/native_music.cpp` owns each decoded channel per run, using the already
qualified direct coefficient/window/synthesis algorithms and offline numeric
tables. It intercepts class `0x151d30` before the device queue; other jobs retain
their resident compiled producer. Original `0x106000` posts software message 5
through `0x1131c0`, retaining its scalar stack and cooperative resume points.
Worker `0x105b80` publishes PCM/history, validates unchanged player/source/output
ownership, then decodes the next channel. No VIF input, hardware status or DMA
completion is used. `0x1060a0` advances the original buffer halves and marker
cache directly. Both completed/cancelled paths explicitly return to their caller.

An optional private typed recording check covers 512 stereo pairs: all 1,048,576
PCM samples and updated histories match exactly. It also checks deferred RAM
visibility, rejected address changes, duplicate submission, per-run isolation,
cancellation, return PCs, marker caching, two-frame continuation, half selection
and all four host rounding modes. Generated reference VU kernels are not linked.
The ordinary pipeline generates only the direct codec and converted tables.

The actual hidden movie-skip route completes 1,000 logical frames with native
title models/filter/UI and direct music (256 stereo frames logged). A successful
native framebuffer capture contains 47 batches / 2,694 vertices. Selecting New
Game next exposes omitted scalar UI jump-table code at `0x2ea380`, frame 1303.
This remains a bring-up boundary, not Logun or PCSX2 visual/audio qualification.
All 50 root checks pass, including the actual 1,000-frame startup route.

### Native compiled-library disc files (2026-11-07)

BozoEngine PR 74 merged as `dbeac945068bb31f458f3b6ddd821908f53fa68a`;
branch, PR and postmerge CI passed on Windows and Linux. The VP2S host opts into
`useNativeDiscFiles()`. Reads use the shared ISO reader and copy actual bytes
before owned native callback delivery; sync/error describe these requests.
No EE drive-time fields, hardware-ready bank or synthetic interrupt is used.
47 engine native suites cover real bytes, missing files, EOF, host short reads,
buffer preservation on failure, callback exclusion/reentry, reset and isolation.

The actual scripted route now passes Csd `cdvdman:11` and reaches music upload
`0x105f60` at frame 670. That upload remains rejected; the next work is to port
the existing direct coefficient-to-PCM decoder and software job ownership.
This is title/disc bring-up, not a playable launch or Logun qualification.
All 49 root checks pass at this pin, including the scripted title route reaching
that explicit music-upload rejection.

### Native two-pass title image element (2026-11-07)

Class `0x15a690` retains its original `0x2f2df0` builder. Native capture keeps both
scalar sprite outputs in order: first the additive pass, then ordinary alpha.
The original corner/colour calculations, cached-build flag and layer completion
remain compiled. The native layer admits only a finished snapshot of this class;
its destructor releases both sprite associations and the element snapshot.

The actual source FIS pointer is null at draw. Separating evidence showed that
the already-qualified constructor hook retained `SHI:41`, id `0x21`, for sprite
`0x997a2c`, with the matching DTT/parts. Draw-time binding now reuses this typed
native lease. No source-asset guess, packet lookup or hardware-address mapping
was added. Temporary setup observers were removed.

64 complete original/native builder CPU/RAM cases add null-FIS atlas leases,
additive/alpha order, independently calculated corners/colours, scratch CPU
stacks, cached rebuilds and immutable lifetime through destruction to the older
128 button/icon cases. The full suite's other 48 checks and the updated startup
check pass. Actual title layer `0x997530` submits one native image element;
the next block is compiled Csd import `cdvdman:11` (`sceCdSync`). No Logun claim.

### Native short cosine (2026-11-07)

The actual title camera next reached `0x214300`. Its degree-nine cosine
polynomial now uses native scalar math and the original coefficient table.
Only the quadrant word and scalar result are published. The temporary scalar
sign store is retired: it was used only by the same numerical calculation and
overwritten by the leaf's old coefficient/sign lanes at its final store.
No inherited vector scratch lanes or register state are reconstructed.

768 complete generated CPU/RAM cases across all four host rounding modes and
65536 independent polynomial comparisons cover signs, quadrant boundaries and
FP controls. Subnormal/nonfinite input remains explicitly unqualified. The
actual skip passes this leaf and reaches a separate UI sprite caller guard.
All 49 suites pass. The next actual caller is `0x2f2efc`, sprite `0x997a2c`,
with no colour multiplier. The runner still has not reached Logun.

### Native title meshes and frame filter (2026-11-07)

`0x239fec` now submits copied native model batches before real compiled scene
completion. The physical mesh enqueue is retired. `NativeModels` resolves the
actual decoded asset-part provenance to offline semantic mesh packs, then reads
the game's transforms, camera, lighting, bone palette, editable positions/normals,
visible mesh list, masks and texture keys. No transport is decoded or replayed.
Unsupported mesh algorithms/light tables and missing data reject.

`tools/vp2_bozo_meshes.py` converts existing VPM3 packs offline into VPN1 records:
explicit material booleans, lighting/texture functions and blend factors replace
raw material words. Runtime code does not interpret those words. 5139 USA packs
convert. Six invented-record conversion tests cover truncation, malformed sources
and algorithm payload preservation. Another 384 native captures compare independent
rigid/two-bone/morph positions, strip order, UV and colours, whole RAM preservation,
hidden meshes, pending ownership and immutable lifetime after source changes.

The next actual producer, class `0x155d50`, is a framebuffer filter. Its original
CPU builder remains: native observers capture the level count at `0x25adbc`
before that register is reused, and final scalar colours at `0x25bad0`.
Native GPU copies/reductions and composites consume these semantic snapshots;
`0x25aa2c` then runs actual scene completion instead of enqueuing DMA. Alternate
formats, depth modes and level selections outside 1..4 remain guarded. 256
snapshot/ownership/lifetime cases preserve whole CPU RAM.

The actual skip route accepts multiple title models and the five-level filter,
then stops at unported scalar cosine leaf `0x214310`. An explicitly labelled
partial-boundary GPU capture contains the title logo/menu (59 batches, 2766
vertices). `VP2S_BOZO_CAPTURE_BOUNDARY=1` with `--capture` is diagnostic only:
the run still fails and the image is not a completed frame or boot/parity proof.
No PCSX2 image qualification or Logun completion is claimed.

All 49 suites pass, including the original native texture publication/allocation
lifetime and movie reset/presentation/teardown checks. Test runs remain headless,
with memory-only cards and no host input. Both native producer bodies and the
complete original filter builder reject source-word mutations offline.

### Inline model light colour and mesh boundary (2026-11-07)

The four-instruction component product at `0x23c17c` now uses the already
qualified native component-product algorithm. All four colour lanes, input/output
aliases, signed zeros, tiny values and host FP restoration follow that contract.
262144 independent vectors and 256 whole CPU/RAM alias cases exercise the new
actual consumer fragment. Nonfinite inputs reject before publication.

All 46 suites pass. The actual movie-skip route reaches `0x239f70` at frame 667,
before any device write. This remains an explicit rejecting boundary, now reporting
the actual model provenance: object `0x9a46c0`, five meshes from disc chain
`0x21551000`, part 1, first mesh offset `0x424c0`. Native model submission and
Logun are not yet qualified. No transfer is replayed or acknowledged here.

### Native six-plane sphere visibility (2026-11-07)

The `0x220b74` caller now snapshots its actual sphere and uses the invocation's
captured clip matrix and immutable camera frustum. The original `0x207cc0`
producer establishes the semantic order: six plane origins, six normals,
then four extra origins/normals. Native math projects the center and tests
signed plane distances against the radius, returning ordinary outside/intersect
predicates and projected/near/far records. It uses no vector memory, flags,
registers, microprogram execution or transfer protocol. Original CPU virtual
callbacks, output stores, culling branches and draw-list construction remain.

Source and the 296-byte numerical program shape are guarded offline. The two
predicate boundaries were separated using the actual source sequence and
[PCSX2's four-cycle arithmetic publication](https://github.com/PCSX2/pcsx2/blob/master/pcsx2/VUops.cpp):
outside is the six distance-plus-radius signs; near/far intersection uses the
last two distance-minus-radius signs. Runtime code computes these predicates
directly, without reconstructing status bits or instruction timing. Signed-zero
side policy is retained. Zero projection depth and negative/nonfinite radius
remain unqualified and reject.

5488 independent axis-aligned geometry cases cover four host rounding modes,
plane boundaries, projected coordinates and near/far depths. Another 256 actual
scalar consumer CPU/RAM comparisons exercise culling, clipped objects, draw-list
entries, far-fade stores, scratch CPU stacks, aliased/changed source data and
snapshot lifetime. Cases reaching the separate `0xc88` angle test stop before
that unqualified operation in both fixtures. The reference consumes ordinary
semantic predicates rather than emulating vector flags. Actual opening skip
passes this sphere test and reaches object update `0x23c17c`; angle culling,
native model draws and Logun remain open.

### Native populated sphere getter (2026-11-07)

The remaining `0x239ce0` getter reached after skipping the opening now computes
its scale and center from game records without VU state. The radius keeps its
original scalar CPU multiply and the center uses the original MMI-transposed
GPR snapshot, preserving aliases and radius W. Source helper and numeric program
shape are fingerprinted offline. 384 whole CPU/RAM cases and 8192 independent
scale cases pass; all 46 local suites pass. See [VU0 notes](vp2_vu0.md) for the
numeric/aliasing contract. The actual skip route now reaches the distinct
visibility-sphere guard at `0x220b74`. Native scene rendering and Logun remain open.

### Native movie presentation, clock and teardown (2026-11-07)

The old `0x2b9560` consumer waited on retired decoder-input byte counts before
reading Timer0, disabling hardware interrupts and decoding into the old upload
queue. Its actual `0x2be220` call now uses a typed native picture consumer. It
waits on real decoded pictures and their sequence frame rate, presents immutable
native RGBA images, and publishes actual application dimensions/picture counts.
It never fills old input/output queues or returns invented hardware counts.
The sequence frame-rate code supplies the rational presentation period on the
native scheduler clock; this disc's opening is 640x448 at 30000/1001.

The audio consumer's `0x2bd5d8` branch now asks whether native presentation has
actually begun. Its original software audio ring/copy/command path remains.
Native command alias handling below lets that path submit real source bytes.
End-of-sequence or an actual PSS program-end code flushes delayed decoder frames;
the reference-empty query remains false until those pictures and the last
presentation interval are drained. No generic TMB MPEG private state is used.

Explicit skip/stop, successful decoder reset and buffer release clear the held
native image exactly once. Reset allocation failure preserves the image and
decoder. At `0x2bc218`, the input-buffer destructor now jumps over physical DMA/
interrupt shutdown to its original software cleanup. Native semaphore deletion,
conditional heap free, CPU return and stack restoration remain compiled. Another
64 complete destructor CPU/RAM cases use this native backend contract. Sixteen
movie-consumer cases cover rational deadlines, no early last-frame release,
actual black/white YUV conversion, immutable held images, EOF flush, reference
query, skip and reset lifetime. Tests also preserve CPU/RAM on blocked input.

The mod002 derived text destructor at `0x373ed8` writes vtable `0x15bd70` before
calling the text base destructor. That exact typed caller now releases its base
native snapshot; other callers still reject. 32 derived/base teardown pairs
preserve CPU state and held snapshots, and reject a wrong caller. This does not
admit the derived class's rendering callbacks. Both source/vtable spans are
fingerprinted.

Hidden native GPU capture `build_usa/bozo/captures/native-movie-600.png` shows an
actual opening frame. The same source image's decoded RGB mean rises from
0.156 at picture 60 to 170.665 at picture 180; the initial black capture was
source content, not absent presentation. A bounded 1800-frame run reaches
815 presentations without a device fault. Skip reaches the remaining sphere
getter at `0x239cfc` after native teardown. PCSX2 movie colour/placement parity,
device audio and the 3D/Logun route remain unqualified. No visible interactive
launcher is delivered by this increment.

### Native command input aliases (2026-11-07)

Actual movie audio submission reached command 2 with source `0x2056e9c0`,
8192 bytes, through the real `0x13a558` sender. The CPU sound producer explicitly
uses the uncached spelling of its ordinary EE heap allocation. The native
command API now resolves only the `0x20000000..0x21ffffff` data alias before
checking its canonical RAM span. It copies the actual allocation's bytes into
the native service; no scratch/device domain, DMA replay or status bank is added.
Both packet and extra-data sources use the same bounded rule, and packet header
writes preserve the caller's ABI. CPU sound-producer source is fingerprinted.

32 command cases cover both aliases independently, all supported modes and
extra/no-extra payloads. Scratch, hardware, other mirrors and end-of-RAM crossing
reject before command submission. Module/identity suites pass; ongoing native
movie-consumer work can now progress beyond 201 presented pictures to 815 in
1800 logical frames. This is not audio-device, movie parity or Logun qualification.

### Original PSS parser and native coded-video input (2026-11-07)

The exact original compiled ring, pack/system-header and PES parsers now run
under a per-stack native lease. Source signatures cover those helpers, their
constant lookup and the actual `0x2bdf80` caller. Admission validates the bounded
original 0x90000-byte ring, input span, work allocation and captured stream table.
Actual parser consumption controls the original file worker; release, stream
mutation, reset and reentry during parsing reject. No hardware parser is used.

The video callback `0x2b90a0` is an explicit native API replacement. It copies
actual PES payload bytes, including ring wrap, and PTS/DTS metadata into native
storage after validating callback GP/data and the original descriptor. A bounded
elementary-stream framer groups sequence/GOP headers and whole coded pictures
for the real native FFmpeg decoder. Native queue capacity supplies actual
backpressure; rejected payloads are not consumed. The old decoder-input copy,
tags and DMA submission are bypassed. The original two audio callbacks and their
`0x2b9f00` ring-copy helper remain compiled CPU software with bounded ownership
checks; this does not qualify audio playback.

64 parser/video handoff and 128 complete parser/audio CPU/RAM comparisons cover
both audio channels, input/output wrap, exact payload descriptors and GP/data.
Thirty elementary-stream split positions verify framing across fragmented start
codes, exact decoder packets, sequence flush, native YUV output and backpressure.
The reference parser uses the same explicit native video-sink contract; the old
physical video callback is not a reference executable. Actual scripted boot now
finishes 300 logical frames, decodes eight real pictures, and pauses its producer
at native queue capacity. Those pictures are not yet presented: the original
consumer still consults retired input-buffer counts. Presentation, movie pacing,
audio playback and Logun remain open. All 46 local suites pass.

### Native decoder reset and original input-buffer reset (2026-11-07)

The actual `0x2b9ba0` call to `sceMpegReset` now retains its original compiled
software reset. A fresh native decoder is allocated before that routine runs;
allocation failure preserves the old decoder and CPU/RAM. The helper retains
its software-state writes while retiring physical decoder/interrupt setup and
control-register accesses. At the original return, validated software records
allow the native decoder swap. No hardware status or completion is supplied.

The immediately paired `0x2b9bb4` input-buffer reset reuses the same native
owner. Its original timestamp/tag CPU builders remain, without consuming those
tags as DMA. Callback and stream registrations survive both resets. Duplicate
reset, premature release and mismatched helper ownership reject. 32 decoder
reset and 16 paired input-reset whole CPU/RAM comparisons pass, together with
allocation-failure and lifetime checks. All 46 local suites pass. Actual scripted
boot now stops at PSS demux `0x1f4590`, caller `0x2bdf80`, before any unowned SDK
body. PSS delivery, native movie presentation/pacing and Logun remain open.

### Original CPU video/audio stream registration (2026-11-07)

The original `sceMpegAddStrCallback` CPU routine now runs for the three actual
constructor calls: video type 0/channel 0 and audio type 4/channels 0/1.
Admission checks the real wrapper's saved caller, function, userdata, created
decoder and the bounded software-handler pool. Its original filter-table lookup,
replacement, counts and prior-function return remain unchanged. The completed
pattern/mask/function/data/GP records are copied into the native owner; these are
PSS media filters, not graphics/device state. No handler is invoked by registration.

96 whole CPU/RAM registration/replacement cases use the actual private constant
table, check both audio channels/video, GP/data and rejects before bad-channel
writes. All 46 local suites pass. Scripted boot completes the three registrations
and stops cleanly at `sceMpegReset` `0x1f0438`, caller `0x2b9ba0`, before any
hardware body/shared wrapper. PSS payload parsing, decoded frames, actual pacing
and Logun remain unqualified.

### Original CPU MPEG callback registration (2026-11-07)

All six actual `sceMpegAddCallback` calls now run the original pure CPU table
routine after native stream/caller/type/function/data admission. Its ABI returns
the previous function pointer, not a registration counter. Function, userdata
and GP are copied from the completed original records into native callback
ownership; no callback is invoked or completion fabricated by registration.
Unknown callers/functions/owners reject before the original table writes.

192 whole CPU/RAM registration/replacement comparisons check all six types,
retained actual GP/data and prior-function returns. All 46 local suites pass.
Scripted startup completes all six registrations, then rejects stream-handler
registration at `0x1f4990`, caller `0x2ba3c8`, before a generic SDK/ABI path.
The three initial stream handlers are video type 0/channel 0 and audio type
4/channels 0/1 in this image. Their source table/callback setup is the next audit;
PSS parsing, decoded-picture delivery, actual movie pacing and Logun remain open.

### Original software MPEG stream construction (2026-11-07)

`sceMpegCreate` now runs its original compiled software-record builder under
native decoder ownership. The explicit API entry validates the prepared movie,
MP/work allocation and real `0x2ba524` caller, then enters the generated CPU
function directly. Only calls `0x1ef864` (physical decoder layout) and
`0x1ef86c` (physical reset path) are retired. Original work clearing, alignment,
pool allocation, callback tables, reference-record construction, error paths and
return remain. No transport descriptor is consumed, hardware status provided or
TMB private ABI written. The real native decoder already exists.

A per-stack creation owner survives original memory-clear callback suspension.
At the actual `0x1ef914` return it validates the CPU-published MP/work records,
then associates successful creation with that decoder; failure leaves it
uncreated. Release while creation is pending rejects. The USA result stays
1/0, with no substituted pointer return. Another 64 whole CPU/RAM cases compare
this original software builder under the same native backend contract, including
small work areas, misalignment, resumed clearing and retained allocation life.

All 46 suites are qualified after rerunning the affected media, source-signature
and scripted-startup tests. Actual startup passes creation and now rejects the
next unowned callback registration at `0x1f0510`, caller `0x2ba544`, without a
device fault. Native PSS/video delivery and the Logun route remain open.

### Native USA MPEG initializer (2026-11-07)

The explicit overlay binding now owns `sceMpegInit` at `0x1f02a0` for the real
`0x2be6dc` movie-constructor caller. It requires the typed `0x159bf8` parent,
its prepared native decoder and a configured native field-duration source.
Initialization changes only the host media lifecycle and returns the actual USA
result 1; it writes no SDK-private globals, work records or device registers.
Other callers/repeated initialization reject. The shared TMB-specific wrapper
remains unused. Sixty-four ABI/whole-RAM checks verify the return, caller guard,
unchanged CPU state outside the result/return PC and unchanged RAM.

All 46 suites pass. Temporary-script startup now gets through the native
initializer and rejects stream creation at `0x1ef6f8`, caller `0x2ba524`, before
either its SDK body or generic wrapper runs. The startup assertion was advanced
to this distinct guard and requires both native buffer/init markers with no
device fault. Source audit also confirms USA `sceMpegCreate` returns 1 on
success, while the TMB wrapper returns a work pointer; its private layout is
different. Stream creation/PSS/decoded frames and Logun remain open.

### Native media clock source and overlay ownership (2026-11-07)

The derived `0x2ba600` constructor configures Timer0 at `0x2ba670`. Its
replacement records the native scheduler's actual field duration in the prepared
decoder owner, after checking the original `0x159ac0` record initialization.
It supplies no timer register/count/status. The later `0x2b99ac`/`0x2b9ab8`
minimum-frame busy wait is a real timing consumer and remains guarded until its
native presentation scheduler is implemented; it has not been discarded as
profiling. Another 64 complete-constructor CPU/RAM cases preserve the original
semaphore/allocation/callback boundaries under bounded fixture leaves.

The next scripted stop exposed a separate dispatch gap: overlay SDK externals
were emitted as stubs but not bound when their image was registered. Calls
therefore reached their compiled SDK bodies; the device guard caught the first
`sceMpegInit` hardware read at 0x1000f520. Image registration now declares every
overlay external to the host explicitly and installs its runtime binding.
Unknown overlay providers reject before generic library delegation. All seven
current MPEG entries remain explicit VP2S provider guards, so neither the SDK
hardware body nor the shared TMB-specific ABI wrapper executes.

All 46 suites pass. The startup test now sends Square through a temporary native
script and proves real buffer/clock-source ownership, then the expected clean
MPEG-provider guard at `0x1f02a0`, caller `0x2be6dc`, with no device access.
`sceMpegInit` itself returns 1 in this USA image; its generic wrapper's 0 is not
the source contract. The next work is the native VP2S media API/PSS provider.
Native video delivery, visible interactive launch, 3D and Logun remain open.

### Native media-buffer construction (2026-11-07)

The `0x2bc330` constructor creates a software semaphore and initializes the
typed `0x159af0` input-buffer record before calling `0x2bbfc0`. Native ownership
now allocates a real host MPEG-2 decoder through `makeVideoDecoder` at that
entry. The original CPU reset, timestamp records and descriptor writes remain;
their transport contents are never read or replayed. Two scoped patches branch
over only the subsequent physical DMA/interrupt setup to the original return.
No transfer job is created and no completion/status is supplied. The native
prepared-buffer association publishes only after its original CPU reset has
finished. Original destructor entry releases it; retained owners keep resources
alive. Other reset callers still reject until their state transitions are audited.

Sixty-four whole CPU/RAM comparisons cover the original reset prefix under this
explicit native transport contract (both fixtures omit the physical tail).
Decoder allocation/failure, repeated initialization, incomplete reset, release
and retained-resource lifetime checks pass. All 46 local suites pass. Actual
scripted startup now passes `0x1000b420`, confirms native prepared ownership,
and stops at `0x10000010` in the derived movie initializer (`0x2ba670`).

The pinned engine's legacy `libmpeg` wrapper writes TMB-specific private globals
and callback words, and `getPictureOn` sets an IPU-demand flag. VP2S explicitly
rejects those MPEG SDK entries before delegation; none has executed in this
bring-up. TMB independently confirms `VideoDecoder` is clean while PSS demux is
still internal to that ABI wrapper. Native PSS/callback ownership, the movie
pacing consumer and playback remain open; this allocation work does not qualify
video delivery or permit a status-bank fallback. Logun remains open.

### Original scalar button-sprite geometry (2026-11-07)

The three missing screen-size prompts now render natively. Capture observes the
real `0x2efe40` text-icon builder and complete `0x2eb3a0` scalar sprite routine.
The original routine computes colour/alpha and four rounded 12.4 coordinate
pairs; native code copies these scalar results and normalized UVs before their
stores, never reading transport buffers. Its original callbacks/instructions
remain compiled. Each text-builder/sprite-builder stack owns its capture, and
the final text snapshot is replaced only after the builder's real return.
Held glyph snapshots and finished icon lists remain immutable through reset,
item release and later rebuilds. Native submission draws the actual icon list
after the glyphs. A layer with a button array but no completed icon capture now
rejects explicitly, closing the earlier missing-prompt gap.

Sixty-four additional whole text-icon/sprite CPU/RAM cases cover both bound
atlases, disabled and unrevealed icons, positions, colour/alpha, depth and UVs.
Invocation, corner order, premature completion, reset-during-capture and held
snapshot checks reject invalid lifetimes. Initial support admits the real text
caller, no extra colour pointer and zero rotation; other callers/rotation still
reject. This does not qualify scrolling text, generic sprites or window classes.

Hidden GPU capture at 600 logical frames now has 346 batches/2,076 vertices.
Against the prior capture, exactly 1,028 pixels changed, bounded by
(221,115)-(360,180), covering the three button prompts. This is native
before/after evidence, not a PCSX2 parity claim. All 45 local suites pass,
including the optional GPU alpha suite. Movie-ring setup at `0x2bbfc0`
remains the next reached guard after scripted Square; Logun is still open.
Its actual caller is the `0x2bc330` buffer constructor (return `0x2bc3e0`),
before any native MPEG instance/decoder has been created. The write guard's
diagnostic confirms active=0/mp=0/decoder=0; retiring it on an assumed active
codec would be incorrect. Native media-buffer ownership is the next audit.

### Button-sheet ownership (2026-11-07)

The first page's missing prompts are three enabled `0x4019` button sprites in
text control record `+0x100`, not the emitter's `0x4010` callback. The original
`0x2efe40` builds them separately after the glyph emitter. A bounded diagnostic
confirms their DTT/part records and item addresses before geometry capture.

`NativeUiSprites` now observes the original `0x2ec260` button-sheet setup at
`0x2ec308`. Its original signed button-ID branch selects an actual game image
object from UI manager `+0x828` or `+0x82c`; type/FIS identity checks resolve the
native `SHI` 0x41/0x3d asset. It reads no TEX0 fields or hardware-address map.
Original DTT lookup and scalar assignments remain compiled. The sprite's original
reset `0x2ec320` releases the native association; held immutable assets survive.
Unknown manager/image identity, escaped part lists and invalid IDs reject before
changing an existing binding. The USA constructor writes manager type `0x15a1e0`;
the older `vp2_2d.md` type value was stale and has been corrected from source.

Sixty-four whole setup/reset CPU/RAM cases cover both ID branches, lookup resume,
identity/part rejection and lifetime. All 45 suites are qualified after rerunning
the affected fixture, signatures and startup tests. Actual startup again reaches
600 logical frames. This only binds assets; scalar sprite geometry and drawing
remain open, and the three prompt gaps are still present in the prior capture.

### Native frame publication and first capture (2026-11-07)

An actual ordering trace separated target setup from drawing: at logical frame
598, `0x114550` composition returned with zero batches, then scene/UI code drew
171 glyphs. Publishing at its return was therefore wrong. `NativeFramePublisher`
now publishes the preceding native list at `0x1145bc`, after the next original
wait has really completed and before resetting the list for the next target.
The first setup only begins a list; it publishes no placeholder frame. Tests
verify that publication keeps the old target and copied geometry, survives later
source changes and rejects invalid next dimensions without resetting the list.
The original composition/wait CPU/RAM comparisons remain unchanged.

The optional GPU build adds `--capture OUTPUT.png`: after the bounded headless
run, it presents the latest completed nonempty native frame in a hidden SDL
window and reads back PNG pixels. It opens no host input provider and accesses
no saves when `--card-dir` is omitted. At 600 frames the first capture is 640x448,
serial 541, 343 batches/2,058 vertices. Measured lit pixels: 1,503 in the heading
zone (200,80)-(450,140), 5,332 in the green-instruction zone (90,150)-(550,225),
zero below y=240. The button-prompt gap is visible and needs a separate source
audit; this is not a PCSX2 parity claim. All 44 local suites pass.

`--pad-script FILE` uses the engine's existing frame-keyed native input source.
With `200:SQUARE/6`, native controller sampling reports button 0x8000 at frames
200-202, then boot advances to a new explicit device-write guard: 0x1000b420
inside `0x2bbfc0` (actual store `0x2bc124`). That producer is not retired yet.
The runner remains headless; a visible interactive launcher and Logun are open.

### Native text-layer submission (2026-11-07)

The first layer is typed `0x152060`/`0x15ba00`, with scene object `layer+0x20`
typed `0x15ba1c` and job `0x15ba98`. Actual boot confirms that object is the
pending pass-1 owner. `0x2ec9c0` now accepts a native draw list copied from
finished text snapshots, then calls the real compiled `0x21c9a0` software
completion. It creates no console transfer job. The original busy store and
completion clear/notification execute unchanged; failure before native acceptance
leaves the completion uncalled.

Child traversal follows the original visible/enabled/built rule. Unknown active
classes, inline images, missing/unfinished snapshots, broken parent links/cycles,
and changed pending ownership reject before drawing. Inactive unknown classes
remain inactive. Glyphs submit shadow then foreground with original scalar
positions/colour/depth and immutable offline native pictures. Native draw-list
ownership retains snapshots through source destruction.

The text fixture now includes 64 native-contract producer/real-completion whole
CPU/RAM comparisons in addition to 64 emitter and 64 glyph cases. Its reference
uses a scalar adjusted-object delay slot and the same explicit native acceptance
contract; it does not execute or qualify a console enqueue. Failure atomicity,
visibility, secondary notification, held snapshots and rejecting admissions are
checked separately. All 44 local suites pass after updating the expected source
patch list/startup assertion. Actual headless boot reaches 600 logical frames
with native solid-screen and 171-glyph layer submissions. Presentation, input
route, 3D rendering and Logun are still unqualified.

### Original scalar glyph placement (2026-11-07)

The native text snapshot now also observes `0x2e83e4` in the fully compiled
`0x2e7e90` glyph routine. At that point the original CPU algorithm has completed
its FPU rounding, fixed-point positions, layer/element alpha, RGB multiplier,
shadow offset and depth calculation. Native code copies those scalar results;
it does not decode the routine's transport buffers or replace the rounding with
an approximation. Geometry uses the original half-texel inset and each converted
cell's padded dimensions. Unknown UV extents, depth and coordinate ranges reject.

An additional 64 whole original/adapted glyph CPU/RAM comparisons exercise the
complete scalar routine, all three shadow modes and varied boxes. Independent
numeric checks verify position, colour, depth, shadow offset and UVs. All 44 local
suites pass. Actual boot accepts placement for all 171 initial glyphs and still
stops at the guarded layer producer. No layer submission or presentation yet.

### Text-item allocation lifetime (2026-11-07)

The original `0x2efba0` text builder frees its layout-item array after calling
`0x2e74f0`. Native capture now observes that emitter's real setup/item/final
callbacks and copies glyph descriptions and immutable offline pictures before
the array is released. Setup mutations are visible; later glyph-callback
mutations do not alter the captured item. Control items remain skipped by the
original CPU code; inline-image items are recorded explicitly for subsequent
draw admission. No packet payload is read or replayed.

Each emitter stack has a bounded invocation owner. Finished snapshots replace
the element association, and the original text destructor releases that
association without invalidating snapshots already held by native consumers.
The original emitter instructions, virtual callbacks, allocation and destruction
continue to execute. Sixty-four whole CPU/RAM/callback comparisons cover empty,
negative-count, glyph/control/inline items and resumed callbacks. Additional
checks reject repeated/escaped lifetimes and verify replacement ownership.
This does not submit text to the renderer: the UI layer remains the boot blocker.
All 44 local suites pass, including the optional hidden GPU alpha test. An actual
headless boot captures 171 glyphs from the initial 174-item array (three controls,
no inline images), then stops at the same guarded layer submission.

### Offline glyph assets and provenance (2026-11-07)

The first active UI layer contains 174 text items. Its font pixel origin is the
actual decompressed mcps chain `0x21551000`, part 0, byte offset `0x1500`.
`tools/vp2_glyphs.py` now converts mcps font pixels and both glyph palette parts
offline into padded native RGBA cells, retaining 0x80-scale alpha. The USA scan
indexes 223 font origins into 178 deduplicated packs (59,817,624 bytes); 361
fontless bundles produce no entry. Packs/manifests stay in the ignored build tree.

`NativeGlyphs` selects these assets through `NativeAssets` decompressed-file
provenance and checks the game's numeric font metrics against the converted
header. Runtime code reads no font pixels or palette bytes. Glyph pictures are
immutable copied cells; held pictures survive source invalidation, while later
lookups reject invalidated or unknown origins. Pack/manifest dimensions, sizes,
palette parts, glyph indices and paths reject explicitly. Caches are per host.

Synthetic tests check nibble ordering, both offline palette parts, padding,
raw alpha, malformed inputs and fontless bundles, plus C++ loader/provenance,
numeric mismatch, source poisoning/invalidation, immutable pictures and host
separation. All 43 local suites pass, including optional GPU alpha readback.
This is glyph-resource groundwork. Text capture was added separately above;
native UI layer drawing remains open, so boot still stops at the guarded
`0x2ec9c0` layer submission.

### Partial-alpha texture publication (2026-11-07)

Glyph preparation exposed a native texture publication error: VP2S expanded the
offline texture's alpha to 255 before both engine presenters expanded it again.
On black, source alpha 32 produced RGB 127/128 instead of 64; source alpha 64
became fully opaque. An end-to-end hidden SDL/OpenGL readback reproduced 16 failed
cases out of 24 through the actual `NativeTextureAsset::picture` method.

The port now preserves the game's 0x80-scale alpha in the native texture object.
All 24 direct/indexed texture cases, with six coverage values and separate frame
and overlay paths, pass; all 41 local port suites are green with this optional
GPU check enabled. The shared renderer remains unchanged. TMB and DC source
audits independently describe this same input convention; the engine Texture.h
comment says otherwise, so that comment was not treated as sufficient evidence.
This is synthetic GPU qualification, not PCSX2/game-scene parity.

The optional `VP2S_BOZO_GPU_TESTS=ON` CMake switch adds this regression test with
an available SDL3 package (`SDL3_DIR`). It creates only a hidden 32x32 window,
initializes no input, and reads no game files or saves. Default builds remain
headless. Current UI metadata traces also identify the first active text layer
and the exact decompressed-file provenance of its font pixels; offline glyph
conversion and native text-layer submission are still pending.

### Native solid screen rectangle (2026-11-07)

The next mod002 writer was measured from its game object: type `0x15bc00`,
mode 0, black `0x80000000`, bounds `(0,0,640,448)` and a 224-byte writer extent.
The native adapter now copies the colour, depth, bounds and original source-frame
dimensions from EE records into an ordinary renderer draw. Mode/extent/type,
pending scene object and invocation lifetime are checked. Other modes remain
guarded. The right/bottom fields are absolute endpoints as the original
`0x120a80` computes them; odd source sizes retain its integer half-size rule.

The full original scalar `0x120eb0` writer flush still appends its terminator and
advances both cursors. Only this producer's proven enqueue retires. Its return
tail-calls the original compiled `0x21c9a0` completion after native input ownership
and scalar flush finish. The per-invocation proof survives the flush and is
released before that real completion, which may itself pause/resume. Generic
nonempty writers still reject. No packet payload is decoded and no hardware
status/completion is supplied.

64 generated producer/flush/completion cases preserve full CPU/RAM except the
removed enqueue's exact call charge and the void secondary draw's dead return.
They cover both scene passes, scratchpad/EE stacks, nested resume, copied colour,
independent vertex endpoint checks and odd source sizes. Unsupported modes,
additional writer contents, duplicate entry, premature completion and renderer
failure reject. Both fixtures use this native rectangle submission contract;
neither executes a graphics consumer. Startup now checks an actual native
rectangle submission before reaching the next UI layer (`0x2ec9c0`). No rendered
scene or playable Logun route is qualified yet.
All 40 port suites pass on this tree.

### Fixed UI image publication and lifetime (2026-11-07)

A caller trace separates the shared enqueue helper `0x2ef390` from the reached
fixed-image producer `0x2f36f0`. That producer now publishes its typed image
object's FIS key to an immutable offline texture, then calls the original compiled
software completion. The source payload and packet pointers are never read by the
adapter. Identity lives under the game image object; there is no DBP/VRAM lookup
table. The destructor entry `0x2ea9a0` releases that association while previously
held assets remain valid. Rebinding an image replaces only its current association.
Complete producer/setup/destructor spans and the image vtable are fingerprinted.

The actual reached upload is in scene pass 1 (`0x2210c0`), outside the pass-0
canonical texture-manager invocation. Admission checks the image's pass byte,
matching current pending object and its real pending flag. Both complete scene
owners are guarded. The retained compiled completion performs the actual flag
updates and event/secondary notification; no transfer-completion value is served.

64 generated producer/completion CPU/RAM cases cover both passes, EE/scratchpad
stacks and nested callback resume. Both fixtures use this native publication
contract; neither executes an upload. Poisoned payload pointers, source overwrite,
destructor release, immutable held assets, rebind, missing-asset atomicity and
per-host separation are checked. Texture fixture: 160 whole cases. Headless boot
advances to the unowned graphics writer from mod002 `0x3721a8`. Native rendering
and the Logun route remain open.

### Native brightness draw producer (2026-11-07)

BozoEngine [PR 72](https://github.com/TheBozos/BozoEngine/pull/72) merged as
`2c91d30` after branch/PR Windows and Linux CI passed. Its merge tree matches the
tested feature tree. All 47 local engine suites pass, including hidden OpenGL
readbacks for native colour levels; postmerge CI is green. The operation is an
ordinary ordered frame material: clamp/subtract/gain, with copied parameters,
explicit admission and digest coverage. It reads no console packets or devices.

The `0x2cb470` brightness draw producer now snapshots its own typed EE object,
requires the actual scene's pending object and active canonical texture-manager
frame, and submits mode 1 as a full-screen native correction. Disabled game
objects draw nothing; enabled tone-band modes 0/2/3 reject pending their own
implementation. RGB writes preserve the frame's alpha. The original physical
enqueue is replaced by the original compiled `0x21c9a0` software completion,
which runs only after the renderer has copied the semantic draw. No physical job,
busy flag, device counter or transfer-completion response is created. Native
submission completes CPU input ownership; it does not assert GPU presentation.

64 additional whole producer/completion CPU/RAM cases use this same native
submission contract in both compiled fixtures; the reference independently reads
the exposure bytes. This is not an execution of the removed VIF transport.
Enabled/disabled effects, scratchpad/EE stacks, nested callback resume and copied
input lifetime pass. Renderer rejection leaves CPU/RAM and pending completion
unchanged; unsupported modes and invalid gains reject. Graphics fixture: 704
cases. Boot advances to the guarded generic DMA selector `0x123220`, return
`0x112fe0`. Native scene rendering and Logun remain open.

All 39 port suites pass with this engine pin and the port's strict host. Generic device
reads/writes, BC0 and VU microprograms still throw rather than delegating to SDK
compatibility paths; no `transfersAlwaysDone` option is enabled. Static counter
consumer admission now rejects both COP0 register 24 and 25. The source inventory
finds neither in the three current images; this does not qualify later images.
New engine raw-syscall services exposed an older fixture's dependence on generic
engine rejection. VP2S now explicitly preserves its existing raw admission:
native stack/heap setup (60/61) only; other raw calls reject before state changes.
Actual image alarm/firmware entry guards remain tested through the registry.

### Scene profiling pair (2026-11-07)

The scene start sample at `0x221698` feeds two elapsed-time calculations in
the real completion callback `0x21c9a0`. A direct literal-access audit of all
generated game images finds no reader of their `scene+0x40a4` result; this is
not a whole-program alias proof. Only those ten profiling instructions retire.
Both existing timestamp fields stay untouched. Pending-object stores, completion
flags, the event-queue callback and the secondary completion notification remain
original compiled game code. Complete source owners are fingerprinted; the start
requires the active canonical texture-manager frame and completion requires its
actual current scene. No clock sample or completion value is supplied.

64 additional whole completion-callback CPU/RAM cases cover both layers,
matching/mismatching pending objects, optional secondary notification and callback
pause/resume. Direct producer checks preserve CPU/RAM and reject a changed scene.
Graphics fixture: 640 cases. Headless boot advances to VIF1 write `0x10009020`
at `0x2cb400`; native drawing and the Logun route remain open.

### Native draw-camera selection (2026-11-07)

The `0x2088d0` camera draw producer now selects a fresh immutable native camera
from its EE matrices, viewport, fog and clear colour. The embedded draw producer
type `0x153120` and existing camera-state owner are checked before admission.
Only its enqueue retires, before a VIF job exists; original scalar setup/return
remain. `0x208870` and the complete producer are fingerprinted. Generic DMA
channel selection now explicitly rejects before the shared SDK could return
a hardware-channel address; generic draw/data submission stays guarded.

64 generated CPU/RAM producer cases compare an observed original enqueue with
native publication and independently check selected camera bytes/immutability.
No VIF job or hardware sample executes in either fixture. The enqueue's dead
return and removed call charge are the only CPU comparison adjustments.
Graphics fixture: 576 cases. Boot passes camera selection and reaches a T0
read after the `0x221650` return point. Drawing and the Logun route remain open.

### Canonical EE texture-manager frame (2026-11-07)

`0x221260` now keeps the game's canonical EE texture-manager record selected
throughout drawing. Its source and working destination are the same record;
no scratch copy, transfer payload, channel state or completion is implemented.
The complete frame, manager initializer and both removed transfer producers
are fingerprinted. A per-invocation lifetime checks the scene, canonical pointer,
entry/exit order and absence of outstanding legacy work before admission.
Overlapping scene/manager records and changed pointers reject.

Only this frame's two physical transfer calls and their paired completion polls
and channel cleanup calls retire. Other device/COP0 conditions still reject.
The original scalar texture-residency code, arguments, dirty-record stores,
callbacks and global-pointer restore remain compiled and act on the EE record.

64 whole empty-scene frame CPU/RAM cases qualify both compiled fixtures in this
in-place mode. Identical bounded camera/queue callbacks mutate the canonical
record and pause/resume; exact byte checks observe those changes and unchanged
poisoned scratch bytes. This does not compare against executing scratch DMA or
qualify populated-scene native drawing. Graphics fixture: 512 cases. Boot passes
the former scratch transfer and reaches camera VIF1 wait `0x208888`
(`0x10009000`). Native rendering and the Logun route remain open.

### Retired render profiling samples (2026-11-07)

The reached render task's T0 sample at `0x29e43c` fed only its private `+0x14`
timestamp store at `0x29e444`. Its audited class methods/direct global references
show no reader; this is not a whole-program alias proof. The next compiled helper
overwrites the CPU temporary. Both producer and store now retire, leaving the
existing task field untouched and returning no counter sample. Admission uses
the live task type `0x1591f0`; a diagnostic separated it from the earlier base
construction type `0x1591e0`. Constructor/initializer and complete owner spans
are fingerprinted. All calls and real software completion checks remain.

The scene-wait sample at `0x22106c` is locally dead before its first completion
flag load. Only that load retires; polling and final scene record resets remain
compiled. 64 complete render-job cases preserve callback order, arguments,
pause/resume, scratchpad stack and arbitrary preexisting timestamp bytes; 64
scene-wait/query cases preserve CPU/RAM. Both fixtures retire profiling without
executing or answering device reads. Graphics fixture: 448 cases. Boot now
reaches the device write at `0x29d75c` (`0x1000d410`); drawing/Logun remain open.

### Native visibility-input lifetime (2026-11-07)

The full `0x2205c0` visibility owner is fingerprinted. A native invocation keyed
by its actual game stack now captures the camera clip and view matrices at their
distinct original read points, retaining the matching immutable frustum. Its
13 transport/basis-preparation sites retire; scalar iteration/callbacks stay
compiled. Entry/return annotations begin and release the invocation, including
the no-camera branch. Missing/duplicate owners or mismatched camera inputs
reject. No generic register storage or hardware matrix bank exists.

64 generated CPU/RAM cases cover empty-scene/no-camera paths, scratchpad stack,
callback pause/resume, source mutation between captures and actual owner return.
The earlier clip stays intact while the later view sees callback changes.
Independent byte checks validate snapshots/lifetimes; both CPU fixtures retire
transport. This does not qualify populated-scene sphere/angle culling, which
remains guarded. Graphics fixture total: 320 cases. Current boot passes these
camera reads and reaches a device read at `0x29e438` (address `0x10000000`).
Native drawing and the Logun route remain open.

### Native frustum publication (2026-11-07)

The fingerprinted `0x207cc0` routine now publishes an immutable semantic camera
frustum: ten world points and ten normals copied from the game's camera records
at `+0x460/+0x500`. Its 41 loads/vector-memory stores retire without any vector
memory, address map, bank or transfer completion. Scalar pointer calculations
and the original return remain compiled. Interior entry is rejected. Remaining
culling algorithms must consume this native publication and still reject until
ported; no original vector-memory consumer is admitted by the publication.

64 generated whole CPU/RAM publication cases check alternating camera objects,
unchanged vector sentinels, owner isolation, exact point/normal bytes and retained
snapshots after source overwrite/republication. Both compiled CPU fixtures retire
the transport; independent byte checks qualify the new native snapshot, without
executing a vector-memory reference. The graphics fixture totals 256 cases.
Boot passes publication and reaches `0x220624`; native drawing/Logun remain open.

### Native final light-colour product (2026-11-07)

Four vector sites at the end of fingerprinted `0x20f440` now multiply the
four-component records at object `+0x320/+0x330` directly and publish `+0x340`.
Caller FP mode and signed-zero/tiny-result handling remain; nonfinite input or
overflow rejects. The preceding scalar light collection/interpolation stays
compiled. 256 CPU-tail/RAM cases compare independent scalar products, saved
register/FPU restoration, scratchpad stack, all FP modes and FTZ/DAZ. This tail
fixture does not newly qualify preceding light branches. Combined: 2,112 cases
/ 371 sites. Boot passes the product and reaches `0x207cc8`, the camera frustum
publication routine. Native drawing and the Logun route remain open.

### Native camera target-distance record (2026-11-07)

Nine vector sites in fingerprinted `0x208360` now compute the scalar-built
target displacement's length directly. The temporary retains squared Y/Z and
the input W; the original scalar copy publishes X to camera `+0x2a4`. Optional
target/enable branches and original object/camera update calls stay compiled.
256 new CPU/RAM cases cover missing/disabled targets, normal/tiny displacement,
scratchpad stack, FP modes and resumed object-update calls. The fixture bounds
the preceding object-update callee with identical callbacks; it does not newly
qualify that callee. Combined fixture: 1,856 cases / 367 sites. Boot passes this
owner and stops at `0x20f8fc`; native scene rendering and Logun remain open.

### Native camera frustum geometry (2026-11-07)

The fingerprinted `0x207d70` owner retains its scalar stack construction/cursor
loop. Its 181 vector sites now transform ten point records and construct ten
normal records directly from the camera world matrix and the scalar-built
stack data. Basis transposition retains its zero arithmetic; the subsequent
inline products, cross products and normalizations retain caller FP mode.
Inputs/results outside the qualified finite domain reject.

The first six normal Ws originate in actual stack point records 4..9, loaded
earlier by the original owner. The second four inherit the first four Ws.
Native publication preserves these bytes, including NaN payloads, from those
records without using a vector bank. It snapshots inputs before output writes.

The camera fixture now retains the actual compiled frustum callee. An additional
256 whole CPU/RAM cases cover normal/zero/tiny/degenerate bases, scratchpad
stacks, four FP modes, FTZ/DAZ and opaque W payloads. Independent scalar geometry
equations match, for 1,600 combined cases and 358 exercised sites. This is
source/numeric qualification, not PCSX2 visual qualification. Boot passes this
callee and stops at `0x208430`; native drawing and the Logun route remain open.

### Native camera projection records (2026-11-07)

The fingerprinted `0x208cd0` owner now derives its projected-depth point from
the screen matrix at camera `+0x330` and the original stack point. It needs no
matrix/vector state across the intervening compiled scalar projection builder.
The transposition's zero arithmetic, truncating weighted sums and reciprocal W
remain explicit record mathematics. Invalid inputs/divisors reject. Its final
view composition separately uses caller-mode ordered products, with both inputs
snapped before writing the camera output. 37 vector sites retire in total.

256 new complete CPU/RAM cases exercise the ordinary dirty-camera route, both
depth-scale choices, scratchpad stacks, in-place final composition, all FP
modes/FTZ/DAZ and resumed scalar callbacks. Independent scalar equations qualify
the new numeric results. Identical virtual/tangent/final-update fixture callbacks
isolate this owner; recursive target tracking and the final camera-update callee
are not newly qualified by that fixture. Total: 1,344 cases / 177 retired sites.
Production boot passes the owner and reaches `0x207d78` in the actual final
camera-update callee. Native scene rendering and Logun remain open.

### Native matrix-column normalization (2026-11-07)

The fingerprinted `0x216960` angle-extraction owner now normalizes its matrix's
XYZ columns directly. Ten transport/call/synchronization sites retire; the
ordinary integer zero clobber remains explicit. The lifted algorithm keeps
truncating sums and reciprocal-square-root order, including zero-length
handling. Translation Ws and the final matrix row survive bit-for-bit. Active
nonfinite/subnormal input and squared-length overflow reject. Scalar angle
branches and the production compiled libm callees remain unchanged.

256 new whole CPU/RAM cases match independent scalar normalization equations
across all FP modes and FTZ/DAZ, zero/tiny columns, both singular branches,
in-place/stack/scratchpad aliases and resumed scalar calls. The fixture uses
identical host scalar angle consumers on both sides to isolate normalization;
it does not independently qualify production libm. NaN payloads in preserved
Ws/last row remain opaque. Total fixture: 1,088 cases and 140 sites. Boot passes
this owner and stops at `0x208f64`; scene rendering and Logun remain open.

### Native frame-builder matrix composition (2026-11-07)

The final matrix product in fingerprinted `0x213b50` now reads completed game
records directly. Its 28 vector transport/calculation sites retire without a
register bank. Both matrices are snapped before output; ordered products/sums
retain caller FP mode and tiny-result handling. The scalar direction/basis
builder and calls to the qualified sine/cosine and rigid-inverse owners remain
compiled. This is not the truncating-sum matrix microprogram.

256 additional complete builder CPU/RAM comparisons pass, including scratchpad,
stack-input/output aliases, both direction branches, all FP modes and FTZ/DAZ.
The fixture totals 832 cases and 130 retired sites; its new product oracle uses
independent scalar equations, while retained trig calls reuse their previously
qualified native owner. Boot passes this composition and rejects at `0x216970`.
No rendered scene or Logun qualification yet.

### Native rigid-inverse translation bridge (2026-11-07)

Eight remaining transport/calculation sites in fingerprinted `0x216e40` now
use the already lifted rigid-inverse translation algorithm on EE basis and
temporary records. The original scalar transpose, identity initialization,
translation negations and final output copies remain compiled. The temporary's
fourth component is preserved, and all active source values are read before
the native result is written. No vector register/flag/accumulator is used.

The world/inverse fixture adds 256 complete CPU/RAM cases, bringing its total
to 576. Independent scalar products, all four rounding modes, FTZ/DAZ, tiny
terms, in-place output, stack/scratchpad input and temporary overlap match.
Unused input last-row NaN payloads remain unconsumed. The fixture now exercises
72 retired sites across world and inverse owners. Boot passes this bridge and
reaches the matrix composition at `0x213e1c` in the `0x213b50` builder;
rendering/Logun remain open.

### Native game sine/cosine polynomial (2026-11-07)

The `0x213fd0` leaf now evaluates the game's quadrant reduction and degree-13
polynomial from its own EE coefficient records. Both the full leaf and original
coefficient initializer are fingerprinted. This is not a host sin/cos substitute.
Thirty vector transport/calculation sites retire; the original integer parity,
sign records and scalar CPU flow remain. The output retains its two coefficient
lanes and computed cosine/sine lanes. FP arithmetic follows the caller's mode;
active nonfinite/subnormal inputs and out-of-range reductions reject.

The original scratch quad store carried meaningful quadrant integers only in
Z/W, plus inherited vector payload in X/Y. Native publication writes the two
integers and preserves the existing EE X/Y bytes instead of inventing a vector
shadow. The inspected generated images show this scratch producer and the two
Z/W scalar consumers; this is not an all-path alias proof. The fixture keeps
poisoned X/Y bytes intact and applies no RAM comparison mask.

Qualification is 384 complete generated CPU-flow/RAM cases plus 65,536
independent scalar polynomial comparisons. Four rounding modes, FTZ/DAZ,
quadrant boundaries, negative zero, scratchpad/global-output aliases and all
30 retired sites are covered. A loose mathematical sin/cos bound is an extra
sanity check, not the native result definition. A near-zero fixture initially
misclassified a subnormal under DAZ; stored-bit classification separates that
rejected input from the qualified minimum-normal boundary without changing
runtime arithmetic. Boot passes this leaf and reaches inverse translation at
`0x216ee0`. No rendered scene or Logun result yet.

### Native object world transforms (2026-11-07)

The full `0x248fc0` owner is fingerprinted. Its 64 vector/microprogram transport
sites now operate on quaternion, position, scale, parent rows and attachment
records directly. The original parent callback, dirty flags and scalar CPU
flow remain compiled. Native calculation snapshots every input at the first
result store, after callbacks and before either output can alias a source.
Local/attachment matrices are written to the original object records. No VU
register, accumulator or flags are touched or used as hidden temporaries.

Quaternion and parent calculations keep the game's truncating sum policy.
Optional parent-column normalization keeps translation intact; the subsequent
inline scaling and attachment products use the caller's FP mode. The original
vi0 CPU clobber is explicit. Active nonfinite/subnormal inputs and arithmetic
overflow remain unqualified and reject; unused position/scale fourth-component
payloads are ignored. Tiny intermediate products retain the native flush policy.

The generated CPU-flow fixture supplies independent scalar numeric equations,
not a vector-machine executor. All 320 CPU/RAM/matrix cases match across four
rounding modes, FTZ/DAZ settings, normal/tiny terms, signed zeros, parent/self /
output/attachment aliases and resumed parent callbacks. Every one of the 64
retired sites executes in the fixture; host FP controls and vector sentinels
match. An initially invalid alias fixture placed a vtable pointer over scale.x;
that poisoned input was separated from valid alias coverage without changing
runtime arithmetic. Boot passes this owner and reaches a separate trigonometric
routine at `0x213fe0`. No rendered-scene or Logun qualification yet.

### Unused profiling-counter reset (2026-11-07)

The reached `0x2bef20` object-update owner wrote zero to five COP0 performance
selectors before its software update loop. Offline inspection finds no reads
of that counter in the currently generated main/mod001/mod002 images. Generation
now rejects an image introducing such a read, pending a separate native
consumer audit. Only the five write-only resets retire; the complete source
owner is fingerprinted. No counter value, register bank or completion is served.

Sixty-four generated comparisons observe the original writes without executing
a device operation. All CPU/RAM and retained callback arguments/stack locations
match, covering empty/disabled children, preallocated/dynamic lists, allocation
failure, child roots and scratchpad callback branches. Original allocations,
cleanup and virtual object updates remain compiled. Boot passes the reset and
reaches the next unported object-update algorithm at `0x249044`. Native scene
rendering and Logun remain unfinished.

### Constants-only global render setup (2026-11-07)

The complete `0x29c990` source is fingerprinted and contains only eleven fixed
texture/filter/blend/test/clamp defaults, with no vertices or image transfer.
Native draws carry explicit materials, so this standalone device-state prefix
has no native draw to execute. Its source-owned invocation now retires only the
enqueue inside the original compiled CPU flush. Writer bounds, terminator and
cursor updates remain compiled; neither the writer nor graphics queues are
reset/cleared to bypass pending work. Unknown writers retain the rejecting guard.

The source captures its exact software range before the flush; publication
requires that range, writer identity and bounds to remain intact. Duplicate /
changed invocations reject. Sixty-four additional generated full-function / CPU
flush comparisons retain all RAM and architectural state, except the void
enqueue result and the exact removed-call accounting. The existing font/setup
suite now has 176 comparisons, including resumed initialization and raw aliases.
Boot now reaches logical frame 58, then rejects COP0 performance-counter reset
at `0x2bef34`. This increment supplies no hardware-counter value or completion.
There are still no rendered scene or Logun-route results.

### State-only debug-font setup ownership (2026-11-07)

The next producer is the ANKF debug-font object in the `mod002` loop. The
fingerprinted `0x29af80` initializer now captures an immutable native atlas and
glyph metrics per object/invocation. Its original CPU construction remains.
The reached publication contains only that initializer's five-qword material
prefix, with no glyph or background. Native materials need no standalone state
submission, so this exact prefix has no draw to publish. This is not a native
glyph renderer or permission to discard text.

The high-level caller at `0x362328` accepts only the captured writer's untouched
begin/end range and no background. A durable invocation owns its original CPU
flush; that flush keeps bounds checks, terminator writes and cursor updates.
Only its enqueue retires, and a rejecting native guard now occupies that site.
Every other nonempty writer rejects before any queue/DMA submission; the
generic SDK guard remains. Appended glyphs, changed ranges, backgrounds and
unowned initializers reject. No packet byte is read, no buffer is reset or
cleared to bypass work, and no graphics job/completion is invented.

The compiled fixture has 112 initializer/flush CPU/RAM comparisons, including
accelerated RAM aliases, allocation failure, optional-background rejection and
pauses at every initializer out-call. The void enqueue result and exact removed
call accounting are explicit differences. Unowned publications preserve CPU/RAM
on rejection, and atlas pixels are checked from the offline fixture. The real
boot passes the state-only font publication and stops at the separate global
graphics-setup writer `0x35df10`, from `0x29cb08`. Rendering and Logun remain open.

### Native full-height source target (2026-11-07)

The ordinary `0x114550` composition path now begins a native Bozo renderer frame
from the game's logical dimensions and scales, before field division/alignment.
The reached target is 640x448. Source and live records establish that the two
enabled startup entries are complementary field views of that same source;
they are not independent scene layers. Their geometry/flags must match the
admitted contract. Secondary copies, extra layers, alternate direct-display or
alignment modes reject. Compatibility is checked again after the retained wait.

Only that path's physical environment/field-copy enqueues, display submission
and two physical display stores retire. Software buffer selection, field-view
state and descriptor construction remain compiled. The generic draw/DMA and
display SDK consumers still reject without a producer owner. Source-target
construction and the complete composer are fingerprinted. This begins a native
target; it does not publish an empty image or claim to have drawn the scene.

Sixty-four generated original/adapted comparisons preserve all RAM and CPU
state except the void producer's dead enqueue result, exact removed-call
cycle/checkpoint accounting and the host slice deadline. Both field phases,
copy-enabled/disabled, visible views and resumed waits are covered; unsupported
layouts, stack reuse and immutable target lifetime are checked separately.
Boot passes this composer and reaches another unowned producer, enqueued by
`0x120eb0` at return `0x120f50`, from `mod002 0x362334`. Native scene/draw integration and Logun are
still unfinished, with no rendered-scene claim.

### Native asset provenance and allocation lifetime (2026-11-07)

Per-host byte provenance now follows successful native `sceCdRead`, actual
`memcpy` and `memset`, and the original compiled decompression producers
`0x102650`, `0x105a30` / worker `0x105940`. Native observations at entry/return
preserve every original CPU instruction. Invocation records survive out-call
unwinds; accepted chain identity is immutable across source-buffer rebinding.
No runtime texture/mesh layout decoding or transfer inspection is introduced.

The interval map preserves unaffected tails, propagates copies across multiple
source intervals, snapshots overlapping source mappings, and invalidates unknown
overwrites. Async jobs reserve their destination: no native consumer sees a
partially completed asset, even when the raw worker calls memcpy. Only the
original compiled worker's successful completion publishes its part identity.
Conflicting writes/job reuse reject instead of guessing. The original heap-free
producer `0x118230` invalidates the payload before coalescing changes its software
header. Immutable published texture assets retain no pointer into those blocks.

Qualification: 33,554,432 per-byte comparisons against an independent map,
plus 204 generated original/adapted CPU/RAM cases for raw synchronous and sliced
decompression, failure, resumed returns and allocation freeing. Only the host
slice-limit refresh on synthetic resume is exempt; architectural context and
RAM match. Compressed-format algorithms remain original compiled code; their
individual completion paths and every other memory-writing API are not proven
by the raw-copy fixture. This observer is groundwork for native asset selection,
not blanket proof of all model-loader/overlay paths.

The bounded boot still rejects the unowned frame-composition submission from
`0x114550`. Native renderer integration and the Logun route remain unfinished;
no scene image or visual-parity claim is made.

### Native camera records and remaining frame composition (2026-11-07)

The camera-only enqueue in `0x208900` now publishes an immutable native camera:
view/screen/guard matrices, viewport/centre, fog coefficients and clear colour,
all from the camera's game records. Its original scalar body and state writes
remain. The enqueue retires before a job exists; its empty completion method
needs no callback. Admission checks the live job type `0x153140`, whose execute /
completion entries are `0x2097c0` / `0x209810`. The full constructor, publisher,
transport/completion leaf and main-image vtable are fingerprinted. Initial
inspection found the base destructor vtable `0x1590b0`; the live type guard
rejected it, and the constructor/static table supplied the separating evidence.

Sixty-four generated camera comparisons preserve all RAM and architectural CPU
fields except the void enqueue's dead result. Exact removed-call accounting is
checked; the original out-call's host slice deadline is separately exempt.
Enabled/disabled publication, matrix/fog contents and immutable record lifetime
are covered. Together with graphics/math installs, that fixture has 192 cases.

A diagnostic boot previously reached logical frame 58 after this camera change,
then stopped at COP0 performance setup. That run exposed another earlier gap:
shared DMA-send SDK stubs also retire data without a VP2S owner. This port now
rejects Send/SendN/SendI/Sync before delegating. The honest current boundary is
`sceDmaSendN 0x1235b8`, returning to `0x113048`, for a frame-composition producer
in `0x114550` (first enqueue return `0x1145fc`). No completion was invented.
The resident enqueue diagnostic now observes an inline CPU entry, so it also
sees direct compiled calls; it preserves the original scalar instruction and
does not install a global override. Native frame composition, draws and scene /
Logun qualification remain unfinished.

### Quaternion/Euler builder temporaries (2026-11-07)

The fingerprinted `0x215870` builder now creates its normalized temporary and
quaternion matrix directly from ordinary EE records, using the already lifted
native math algorithms. Its scalar branch/asin/atan2 calculation remains
compiled, including boundary/gimbal-lock cases. Its two original vi0 CPU
clobbers remain explicit; no vector/flag bank supplies or receives results.
The normalized stack temporary is preserved even though the matrix uses the
original input rather than that normalized copy.

Sixty-four generated original-consumer/adapted-wrapper comparisons include
scratchpad stacks/sources and aliased output. Complete outgoing CPU context,
all RAM and host FP effects match. The math suite now has 384 matrix/quaternion/
Euler cases plus 1,048,576 independently calculated scalar component products.
An initial one-bit normalization mismatch was isolated to compiler folding of
the loop-built test input outside its rounding guard; an opaque, non-inlined
reference input fixes the fixture without a runtime arithmetic change.

Startup passes this builder and next reaches the original scene transport at
`0x2097c0` (VIF1 DMA data/length submission). This is a rendering producer and
requires native scene/draw ownership, not a fabricated channel completion.
No rendered-scene or Logun-route qualification yet.

### Unconditional default math-code installation (2026-11-07)

The reached channel setup was dispatched by the unconditional program selector
`0x29e150`, a sibling of the already retired conditional selector `0x29e180`.
For the validated default math mode only, its physical enqueue and associated
busy store now retire together. The original mode record and scalar CPU body
remain. Admission requires the expected object type and no outstanding legacy
job; other modes reject. The fingerprinted default resource only installs math
code/constants and executes no draw or data computation. No job is created or
completed, and the DMA leaf remains unported.

Sixty-four generated producer comparisons check the exact absent busy write,
all other RAM, all architectural CPU fields and the removed call's cycle /
checkpoint delta. The fixture separately allows the host slice-limit refresh
at the original out-call. Nondefault-mode rejection is tested. Startup passes
this producer and reaches `0x215870`, whose inline normalization/quaternion
matrix temporaries feed its retained scalar Euler-angle calculation.

### Static scene palette publication (2026-11-07)

The next producer `0x29cb20` is reached from the fingerprinted scene setup
`0x2b8f90`, with the fixed palette at `0x314200`. The entire palette image is
fingerprinted and converted to linear index order offline, into the same native
conversion format. Runtime admits this exact source identity, publishes the
immutable palette by the game's software slot, then frees its unused descriptor
through the compiled heap routine. Other dynamic sources remain unsupported.
No runtime palette-layout conversion, upload inspection or graphics-memory
mapping was added. The offline manifest now includes 4,872 unambiguous game
keys plus this separately named native scene palette.

Thirty-two additional generated CPU/RAM/ownership comparisons cover the fixed
palette producer, allocation failure and resumed allocation/free boundaries.
The fixture poisons the corresponding EE source bytes: publication still comes
only from the offline file, and every native output texel is checked. Startup
gets through both texture sets and now reaches the separate math-code transport
leaf `0x29e0c0`. Scene rendering and the Logun route remain incomplete.

### Native FIS publication and descriptor ownership (2026-11-07)

The `mod001 0x29cc30` producer now captures the FIS key into a per-host active
invocation and loads its immutable offline conversion. Its original software
palette/pixel slot allocators and CPU descriptor builders remain compiled. At
the two original enqueue sites, the native owner publishes the immutable asset
by the game's slot identity, then the original compiled heap-free routine
releases that descriptor. No graphics job is enqueued, so no completion, IRQ,
busy flag or generic SDK acknowledgement is supplied. The generic image-upload
consumer remains guarded. The full producer body is fingerprinted; native
annotations are module-scoped and never global cached-overlay overrides.

Pending invocations survive scheduler unwinds and disappear on every original
return, including first/second allocation failure. They retain no descriptor or
EE pixel pointer. Reusing an active stack or publishing a part twice rejects;
completed stack reuse is valid. Already published images keep their immutable
identity after slot rebinding. Native picture keys also distinguish palette
parts and different per-host asset lifetimes.

The generated original/adapted fixture passes 64 cases covering allocation
failure, descriptor release counts/outstanding allocations, and pauses at
allocation/free return boundaries. Every RAM byte matches the reference
ownership observer. CPU comparisons explicitly allow the changed a0/a2 call
arguments and refreshed host slice limit after a synthetic resume; other fields,
including cycle/checkpoint totals, FPU/vector/flag sentinels and return values,
match. All 256 output texels of the selected palette part are checked. Separate
cases cover concurrent invocation stacks, duplicate publication, immutable slot
rebinding and reuse. This is not a pixel comparison against PCSX2 or a proof
that the renderer integration is complete.

`vp2_bozo_texture_manifest.py` indexes existing offline conversions; 4,872 keys
have identical converted content in every known disc copy. Missing/conflicting
copies remain rejected until load provenance selects one. Runtime reads only
FIS metadata and linear native conversion files, never transfer packets,
swizzled pixels, GS memory or a register shadow. The first real `ANKF:0` asset
publishes palette/pixels and both descriptors are freed by compiled code.
Startup next reaches the separate static palette producer `0x29cb20`.

### Menu physical-memory fill (2026-11-07)

The next guarded image caller, `mod002 0x360dd0`, fills a temporary EE buffer
with a byte and repeats uploads across physical graphics memory. Its complete
body is fingerprinted; only the upload producer at `0x360ef4` retires. The native
renderer has no physical graphics-memory image to fill. Allocation and failure,
the byte fill, descriptor construction, synchronization calls and free remain.
Sixty-four generated original/adapted comparisons cover normal and failed
allocation, both sizes and varied fill bytes. All architectural CPU fields and
RAM match; the exact removed-call scheduling delta is checked separately.

Startup now rejects a real queued image upload at `0x122bf8`, returning to
`0x112fac`. An opt-in enqueue trace identifies its producer precisely:
`mod001 0x29cc30`, enqueue returning to `0x29cd88`. That routine reads its own
image records, allocates a palette slot and pixel slot, and enqueues palette
and pixel descriptors separately (`0x29cd88` and `0x29cf20`). The first reached
descriptor has no callback owner. Its queue flags still own descriptor freeing;
discarding the enqueue without handling that allocation would leak memory.
The next work is this high-level asset producer and native texture ownership,
including direct palette/pixel identity and allocation lifetime. The resident
enqueue tracer invokes the generated original; it does not replace processing.
No queue indices or completion flags were cleared, no DMA interrupt
was fabricated, and no packet payload was parsed. Native scene/Logun parity is
still unqualified.

### Display selection and unowned image guard (2026-11-07)

The whole display selector `0x114240..0x114544` is fingerprinted. Only its ten
physical DISPFB/DISPLAY stores retire. All software buffer selection, record
updates and helper calls remain compiled. The generated original/adapted fixture
passes 128 full-CPU/full-RAM comparisons, covers all ten stores, and preserves
32 software/movie callbacks. Its reference only observes the nominated writes;
it does not execute a GS device or supply register values.

A diagnostic startup then reached a graphics-upload queue barrier at `0x112bc0`:
capacity 1024, active index 0, producer index 5, active and processing flags set,
and four subsequent type-1 descriptors. The queue must not be cleared or given
a synthetic DMA completion. Source inspection exposed an earlier problem:
Bozo's shared `sceGsExecLoadImage` retirement assumes its port already owns the
texture, but VP2S's native texture owner has not been installed. VP2S now rejects
that SDK boundary explicitly, before the engine's no-op. The first caller was
`0x100710`, inside a sixteen-slice zero-fill of all 4 MiB of physical graphics
memory. Its fingerprinted upload producer now retires: native rendering has no
physical graphics memory. The EE buffer zero-fill, descriptor construction,
sixteen frame waits and buffer free remain. Eight generated comparisons verify
all architectural CPU fields and all RAM, plus the exact scheduling difference
of sixteen retired call checkpoints / 128 generated cycles. All 128 descriptor
builds and waits remain. This retirement is scoped to this boot-only producer;
later real assets still require direct texture ownership from records/files.
Startup now rejects the distinct menu image upload returning to `0x360efc`.

`VP2S_BOZO_TRACE_QUEUES=1` prints only audited queue metadata and descriptor
type/source/count, with checked bounds. It never prints descriptor payloads,
bootstrap material or arbitrary CPU-referenced memory. The production boot test
now requires the unowned-image guard. Display selection remains fixture-qualified;
no rendered scene or playable Logun-route completion is claimed.

### Menu frame-clock physical stores (2026-11-07)

The menu module repeats the boot code's HBLANK mode setup and resets its counter
at each frame-loop iteration. The native frame clock and game wait already own
these transitions. Its whole `0x3617f0..0x362764` body is fingerprinted; only the
two physical stores at `0x36224c` and `0x362760` retire. Both are delay slots:
their adjacent call/branch, game-state writes, logical frame wait and checkpoints
remain unchanged. No counter/status value or completion is fabricated.

Targeted clock, frame-event, source-signature and native-startup checks pass.
Startup reaches the distinct display-buffer selector (`0x114240`), stopping on
its DISPFB2 store. That routine also updates software/movie records, so it must
not be replaced by a dummy return; physical display stores are the next audit.
There is still no rendered scene or playable-route qualification.

### Native component product and constructor constants (2026-11-07)

The reached leaf `0x211da0` multiplies the four components of records at
object `+0x560` and `+0x570` into `+0x580`. It now computes directly from those
records, including its result store in the return delay slot. The whole body
is fingerprinted. No vector bank, MAC flags or emulated instruction is supplied.
The finite-input native path uses chop rounding, signed-zero/subnormal handling
and restores host FP state; nonfinite inputs explicitly remain unqualified.
The local PCSX2 reference config uses VU0 round mode 3 and denormals-as-zero;
[PCSX2's arithmetic source](https://github.com/PCSX2/pcsx2/blob/master/pcsx2/VUops.cpp)
was inspected for reference, with no code copied. This is not a new pixel-parity
claim or a blanket qualification of exceptional floating-point inputs.

An independent integer/rational oracle checks 1,048,576 finite scalar products,
with complete generated CPU-context preservation and sampled full-RAM checks.
The previous 320 matrix/quaternion/context cases still pass. The source read
and native result are snapshotted before writes; invalid inputs do not change
the destination.

Another omitted UI type getter exposed a constant-propagation bug: unrelated
ADDIU instructions invalidated all tracked constants. The offline scanner now
tracks zero-source immediates and only invalidates actual CPU destinations;
coprocessor moves/arithmetic and syscall boundaries have explicit rules. It
continues to stop after control-flow delay slots. Nine metadata suites cover
unrelated writes, true clobbers, null slots and overlay provenance. All 28 port
suites pass. Current vtable additions are 8/1197/238 for main/mod001/mod002.
Startup now reaches the menu's HBLANK control store at `0x36224c`, before its
frame loop. No rendered scene or playable Logun-route qualification yet.

### Native object-update transform (2026-11-07)

`0x249270` repeats the constructor's audited quaternion/position and scale
calculation with different ordinary CPU argument registers. Its full body is
fingerprinted; its inline sites now share the typed constructor algorithm.
The retained CPU code still builds its fourth row, scale record and return ABI.
Sixty-four additional generated update/tail comparisons cover varied positions,
quaternions, signed scales and scratchpad stacks. All 320 math cases preserve
complete CPU context and RAM against their independently populated original
CPU consumers, without writing VU registers or flags.

Targeted math, source-signature and headless-startup checks pass. Startup gets
past object updates and reaches the separate component-product helper
`0x211da0` (first guarded load `0x211dac`), whose four-field result still needs
native lowering. This is not normalization; source inspection separates the
operation. No native scene or playable-route completion is claimed.

### Native initialization entropy and complete static linkage (2026-11-07)

The next timer access was initialization jitter in `0x2c3c10`: it repeatedly
advanced the scalar game RNG while waiting for a counter change, then derived a
discard count from the first sample. The full routine is fingerprinted. Native
code retires that hardware-dependent warmup and supplies one bounded per-run
entropy sample to the retained discard-count/RNG/shuffle code. A fixed
`VP2S_BOZO_SEED` gives a reproducible stream; default runs use the host seed.
No timer count, elapsed-time register or second fake sample is returned. This
does not claim the same random history as PCSX2's nondeterministic jitter.
Three-host/256-draw checks cover determinism, seed independence, the original
sample domain and complete CPU-context preservation except the input result.

The subsequent UI stop identified two additional offline metadata cases:
constructor/destructor vptr stores in JAL delay slots, and isolated null virtual
slots before later methods. Constants now remain valid through the delay and
expire afterwards; single null slots do not truncate the table, while a new
two-zero header still stops it. Literal direct-call linking also converges
between main/common code and follows each later overlay's own calls, preserving
overlay provenance. Nine metadata tests cover these rules, including alternating
common-image call chains and overlapping overlays with different local targets.

All 28 port suites pass; the math bridge tests now compare entire CPU contexts
in all 256 cases. Current vtable additions are 6/1176/232 for main/mod001/mod002,
with another 48/9/0 literal-call entries. All original load bytes are unchanged.
Boot passes the timer warmup and UI setup, and stops at the object-update
transform `0x2492ac`. Rendering/Logun remains unqualified. C2C/F2F checks/posts
resumed after the user supplied the new endpoint and explicitly requested resume.

### Static vtable method metadata (2026-11-07)

The camera stop at `0x1e8080` was a genuine omitted setter: its original body
stores f12/f13/f14 and the homogeneous component, with no device operation.
Its camera vtable and related setter/getter source spans are fingerprinted.
Admitting that family exposed another omitted virtual leaf, `0x1ffc90`.

Offline preparation now follows literal vtable pointers stored by admitted
constructor code, including nonzero base-class offsets. Tables require the
original two-zero RTTI header and two method addresses in explicit input code.
Constant tracking stops at control flow/clobbers. Only original file bytes
are used; no runtime RAM, instruction discovery or interpreter fallback exists.
The existing bounded static entry lifter supplies original method bodies.
Constructor provenance is kept per overlay: overlapping overlays cannot nominate
methods for each other. Main/common-module entries can use shared references
because those address ranges do not overlap the later overlays.

The current main/mod001/mod002 build admits 10/540/116 additional vtable-linked
entries (some become interior labels), retaining every original load byte.
Seven metadata tests include clobbers, malformed tables, ignored raw words,
scope isolation and byte-preserving preparation. All 28 port suites pass.
Boot passes the omitted camera/scene setters and reaches the distinct random
warmup's raw timer read (`0x2c3cb4`; the last call marker reports `0x2c3c60`).
Native rendering and the playable Logun route remain unqualified.

### Native object-constructor transform (2026-11-07)

The reached constructor (`0x2494c0`) uses the separate quaternion entry `0x728`.
Source inspection shows why it cannot use the common conversion unchanged:
the supplied position becomes the W components of its first three rows. The
retained CPU code builds the fourth row and scale record. Native code now writes
those three rows directly, then computes the original scale/rotation product
from ordinary records. The entire constructor and admitted source math image
are fingerprinted; no inherited vector register or arithmetic flag is restored.

All 28 suites pass. Sixty-four additional generated constructor-tail comparisons
use varied quaternion, signed scale and position records, complete RAM/GPR/FPR
comparison, working stacks and untouched vector sentinels (256 total math ABI
cases). Boot gets past this calculation and stops on the camera vtable setter
`0x1e8080`, absent from the offline function map. Its original body stores the
three float arguments and homogeneous component in the camera record. This is
an omitted code entry, not a rendered-scene or graphics-parity result.

### Native default math selection and record bridges (2026-11-07)

The default VIF0 selection at `0x29e180` installs 453 math instruction pairs and
fixed constants, without executing a program. Its full source resource and
selection/uploader bodies are fingerprinted. For mode zero, native code retires
the busy-store and enqueue before a job exists, retaining the original mode
store, branch and CPU instructions. The inline admission requires the expected
game object type and no outstanding legacy job. Other modes still reject before
their mode store; no VIF stream, vector bank or DMA completion is supplied.

The common matrix product (`0x217550`) and quaternion matrix (`0x216730`) now use
the previously verified native record algorithms through Bozo's memory ABI.
The original CPU stack/result-copy bodies remain compiled and run; only vector
transport/calculation retires. Inputs are loaded before any output write, and
ordinary CPU clobbers remain, including the upper half of t1 surviving its
low-half zero. EE RAM aliases and the 16 KiB working stack are checked directly.
No legacy runtime library is linked.

All 28 suites pass, including 192 generated CPU-consumer/native-helper full-RAM
and GPR comparisons, scratchpad stacks, in-place outputs and unchanged vector
sentinels. The existing independent offline game-arithmetic check was rerun:
65,552 matrix and 131,096 quaternion cases have zero differences. These numeric
checks do not qualify visual output. Boot reaches the object constructor's
separate quaternion/position and scale calculation at `0x249618`; it remains
guarded pending its own lowering. No scene or Logun route has rendered yet.

### Retire the initial graphics-code installation job (2026-11-07)

The hidden native boot reproduced the VIF1 write at `0x29e240`, frame 56,
from the software queue's start callback (`ra=0x202094`, task `0x59cfe0`).
Offline inspection separates this from a mesh or texture upload: its single
REFE source installs 220 graphics instruction pairs and fixed transfer defaults;
there is no UNPACK, DIRECT or MSCAL. Its completion method at `0x29e230` is empty.
The producer's sole audited caller at `0x249950` ignores its return value.

The native build retires only that producer's enqueue call (`0x29e2fc`). Object
initialization and the call's delay-slot load remain. No job is submitted, no
interrupt/status/busy flag is fabricated, and all later transfers still reject.
Full source fingerprints include the producer, uploader, empty completion and
complete offline resource bytes. Sixty-four original/adapted producer comparisons
preserve full RAM and GPRs apart from the unused enqueue result; the original
leaf is observed as a software enqueue without executing a DMA consumer.

All 27 port suites pass. The longer boot now reaches the distinct VIF0
program-selection vtable entry `0x29e0c0`, which was absent from the offline map.
No rendered scene or playable route is claimed. The build driver now builds all
configured targets before CTest, including newly added checks, avoiding stale or
missing test executables. C2C/F2F remains paused at the user's request.

### Native procedural stream initialization (2026-11-06)

The random manager's constructor now seeds ordinary per-run procedural-stream data from the game's
original EE random result. The following unused vector read is retired under its complete constructor
fingerprint. No VU vector/random register or pipeline is maintained. The stream's 23-bit polynomial
policy was checked against [PCSX2's reference](https://github.com/PCSX2/pcsx2/blob/master/pcsx2/VUops.cpp);
the team-authored integer implementation is checked by an independent basis-transition calculation
over 262,144 steps. Sixty-four seed-hook cases verify complete context preservation, independent
owners and unchanged guarded-consumer behavior. The 66 `VRNEXT` sites in `0x2439f0/0x2961a0` still
reject until their actual scalar consumers are qualified; this is initialization coverage only.

The next missing vtable method is admitted as original static code (`0x29e240..0x29e274`) with its
complete source guard. It only starts the old VIF1 program-install transfer. The real boot now stops
on its QWC write (`0x10009020`). Native graphics resource preparation and completion must replace
that producer/consumer pair before continuing. Still no native rendered scene or Logun qualification.

### Native graphics prelude (2026-11-06)

Overlay startup's DMA-control enable store and the channel-nine object's physical routing RMW are
retired under complete setup/constructor fingerprints. These do not submit application data. All
allocations, object/vtable fields, native callback registration and initialization remain compiled;
no control/status value is supplied. Mutation tests cover every word of both bodies. The real boot
now gets past that setup and reaches `0x29e20c`: initialization of the game's random stream through
the old VU random instructions. That stream and its consumers need a native algorithm; it remains
an explicit unsupported boundary. Native rendering and the Logun route are still unqualified.

### Native startup arithmetic and overlay linking (2026-11-06)

The initial VIF0 upload installs only the already ported startup derivation algorithm. Bozo now reuses
the pure native `port/vp2s_bootstrap.h` algorithm, extracting its tables offline from the guarded USA
image. Static patches retire that upload/poll/ack and replace its EE/VU bridge with a native hook on
the real Csi response record. No VIF chain or VU execution core runs. The original offline checker
passes 65,537 seeds with zero differences using the newly generated tables; 64 Bozo ABI cases compare
the private captured key, register effects, host rounding and independent owners. Private values are
never printed or committed.

The longer boot reaches overlay initialization after 56 logical boundaries. The offline map omitted
15 of mod001's 27 original constructor entries. Preparation now consumes explicit `.ctors` metadata,
validates executable-section targets and compiles those bodies rather than skipping initialization.
The mod001 code/list fingerprints have mutation coverage. Mod002 is now included. Cross-image static
J/JAL references also expose 48 main entries missed by isolated analysis, including object constructors
and accessors. New bodies are bounded by declared code and return/tail transfers after forward paths;
covered interiors become labels. Synthetic tests cover recursive references, forward return paths,
interiors, bounds, ignored unadmitted bytes, idempotence and unchanged load segments. No runtime
instruction discovery is added. The longer native run now passes these gaps and explicitly stops on
overlay graphics setup's DMA-control write (`0x1000e000`, block `0x249824`). There is still no native
rendered scene or Logun-route qualification.

### First game-frame notification (2026-11-06)

The post-sound watchdog was missing the game's frame-end notification: traces show the original
frame-event counters advancing and the compiled wait completing. A guarded native hook at its NOP
`0x11f62c` now publishes that completed boundary without changing game RAM, CPU, counters or return
parity. The query-only branch bypasses this site. The complete wait-body fingerprint rejects mutation.
The frame-event suite adds 256 complete-context/RAM preservation checks for publication. A one-boundary
headless run exits normally (`frames=1`); a longer 600-boundary request explicitly stops at a PS2 DMA
control write `0x1000e000`, instruction `0x10087c`. This is logical startup progress, not native rendered
frames or visual/Logun qualification. Physical display timing is not generated by the notification.

### Ready native audio completions (2026-11-06)

The host now opts into delivery of actual completed libsd jobs at linked native function returns.
The real startup gets past Csm's busy flag: its compiled completion callback clears the flag after
the actual 4,096-byte upload. No game flag/status is manufactured, no clock is advanced and the work
budget is unchanged. The engine suite reproduces starvation under the unchanged default policy and
checks actual mixer bytes, once-only/reentrant callbacks, exclusion, reset and host isolation.
The next production boundary is `timrman:20` at `0x45790`. Still zero frames; no playable route yet.

The next timer increment supplies native callback registration/configuration/start/stop contracts.
All 25 engine suites pass. The actual boot now passes Csm's callback and clock configuration, stopping
at `vblank:4` (`0x457c4`) before timer start. Timer deadlines, next-period returns, zero cancellation,
self-stop/free, GP, clock divisors, reset and two hosts are tested independently. Gated modes still
reject. No device timer state is modeled. The production frame wait needs a native scheduler contract;
still zero rendered frames.

The following increment maps Csm's frame listener to actual native game-frame completions. Advancing
time cannot manufacture these events. Start listeners preserve callback GP/argument, priority/FIFO
order, zero removal, cancellation and exclusion. The native host publishes them from `onFrame`.
Csm's one-time physical-display alignment call at `0x449f0` is retired under its complete USA image
fingerprint and exact call/delay-slot guards. The original timer creation, handler/configuration,
start, listener registration, errors and all game data remain compiled. Ninety-six original/native
comparisons cover complete RAM, CPU and ordered SDK calls across success and every setup-error stage,
excluding only the retired display wait. Actual startup now passes Csm start/listener setup and reaches
Csd `sifcmd:8` at `0x559d4`. The frame listener has isolated native-event tests but the production renderer
still produces zero frames. This does not qualify audiovisual timing or the Logun route.

Engine PR21 (`30761f5`), PR22 (`9bc4acf`) and PR23 have green Linux/Windows CI. The final frame-listener
increment passes all 26 engine and 23 VP2S headless suites. Runs use no window/input and fresh in-memory
cards. The next Csd request is `sceSifSetCmdBuffer` for five entries at `0x56e78`, followed by user command
registrations 0..3 (`sifcmd:10`). The next implementation must provide a real bounded native command
registry and dispatch, not acknowledge registration while dropping its messages. System commands,
transport protocols and device interrupts are not required for that native user-message service.

The next engine increment implements that bounded user-command registry and real queued native
dispatch. Owned message bytes, linked callbacks/GP, table records, removal, exact-once dispatch,
exclusion, invalid IDs/sizes, reset and independent hosts pass in 27 engine suites. Csd's five-entry
table and four registrations now complete in the actual boot. The next failure is an EE worker's
indirect target `0x107e10` (`ra=0x11977c`, `a0=0x5605e4`): no generated code or override is present.
The separating check is whether this is omitted static coverage or a corrupted target. EE message
senders/extra-buffer transfers are still unqualified and must be connected before actual user commands
can flow from compiled EE code. No frames or playable route yet.

The indirect target is confirmed as a retail C++ adjustor thunk (vtable `0x151d70`, slot +16), not a
corrupt pointer. Both omitted thunks `0x107e10/20` subtract four from `this` and tail-call existing
compiled bodies. Metadata now admits exactly those two eight-byte entries under their complete
32-byte source fingerprint; every guarded word has mutation coverage. Load bytes remain unchanged.
The actual boot passes these entries and stops on a real 4,096-byte upload to native sound-data
address `0x15b140`. The opaque module-file allocator owns only `0x40000000+` handles, so that transfer
must use the sound host's checked allocation gateway rather than aliasing an opaque handle into RAM.

The allocation gateway now checks every span against one live native sound allocation, including
interior alignment; images, stacks, freed blocks, opaque handles, adjacent-block crossings and overflow
reject. Transfer batches validate/snapshot both destination domains before any copy. Mixed-domain
atomicity and actual bytes pass; all 28 engine suites pass. The real 4-KiB upload to `0x15b140` succeeds.

Its following `0x44` transport descriptor is not accepted. Instead the complete guarded SDK sender
`0x13a3f0..0x13a524` is replaced by direct native user-message submission. It retains the game's header
fields and optional checked extra-data copy, then queues its registered compiled handler. Eight mode/
extra-data cases check actual bytes, header/option, canaries and returned job IDs; invalid spans preserve
the packet and submit no job. There are now 1,782 rejecting SDK interior guards. Actual startup receives
the EE command in Csd and advances to its reverse reply API, `sifcmd:12` (`0x559e4`). That reply still
rejects: a native library-to-EE queued callback path is required. Zero frames and no Logun qualification.

The following reply increment replaces compiled `sifcmd:12/13` with a native application provider.
The port consumes the game's retained software SetCmdBuffer/Add/Remove records (layout guarded by
their complete USA source signature), checks the linked EE handler, copies message/extra bytes and
queues the actual compiled callback with its original GP/argument. Its native allocation is released
when the callback returns. Engine tests cover bytes/header/job IDs, provider rejection, reset and
two hosts; the port test executes a linked callback and verifies data, GP, exactly once, extra bytes,
invalid targets and allocation cleanup. The real boot now completes the sound handshake and reaches
the main game loop, which stalls at `0x11f608` in frame-wait routine `0x11f5a0`. The watchdog reports
its currently running old frame callback `0x1200f0`. Still zero frames: native pacing and rendering
are not integrated, and no playable/Logun result is claimed.

### Native libsd provider (2026-11-06)

The USA `Sound_Device_Library` image is admitted through a full SHA-256 guard and uses Bozo's native
libsd/mixer provider, following DC's published provider pattern. Game note/pitch arithmetic remains
strictly compiled; the hardware initializer does not execute. The isolated probe sets/reads parameters
for 24 voices on each core and verifies all 64 uploaded bytes in actual mixer storage, with repeat,
reset and independent hosts. This is an API/data qualification, not soundtrack or timing parity.
Production now starts libsd as service 5 and stops at `Csm_.irx`; no frame is rendered yet.

The following Csm increment admits its full USA fingerprint and strictly compiles 72 routines.
Its original initialization entry performs native export registration, configures native callback
sources and initializes the game callback record. It passes repeat/reset/two-host checks, without
executing device code. Production assigns service 6 and reaches the distinct `Csd_.irx` boundary.
This qualifies Csm initialization only; streaming/voice/timer services are still checked on use.

The following Csd increment admits its USA image and 75 strictly compiled routines. Its original
entry creates a native cooperative task, retains the compiled Csm dependency call and leaves the
worker sleeping at the game's `SleepThread`. Repeat/reset/two-host and retained-sleep checks pass.
Production assigns service 7 and reaches `Csi_.irx`. Playback/file/command paths are not yet qualified.

Csi's isolated initializer reproduced missing `sysclib:14` (`memset`), implemented by engine PR16
with bounded native byte filling and 19 passing headless suites/green Windows/Linux CI. Csi's original
compiled entry now registers service `0x1234` and parks its native RPC task. Registration, parked worker,
repeat/reset and independent-server ownership pass. Production reaches `mcman.irx` after service 8.
An explicit request-14 probe still fails on `thbase:42`; RPC worker identity/priority is the next native
SDK contract, not an invented priority response. Game startup still has zero frames.

Engine PR17 now retains each native RPC server's owner task and implements `thbase:42` using its
native priority record. Callback stacks remain separate; identity/status/priority refer to the registered
worker, which remains parked. All 20 engine suites and Windows/Linux CI pass. Csi request 14 passes
the complete 20-byte status/seed reply against the guarded private image, both hosts and output canaries;
no private word is printed. Other RPC requests are still unqualified.

The standard mcman/mcserv modules now use the installed native EE memory-card APIs, as in DC's provider
pattern. Exact private image admission, API readiness, empty arguments and stable/reset service IDs are
checked. Tests use fresh in-memory cards. All ten boot modules initialize; the real boot next rejects
DBC `intrman:17` before any callback-protected transfer. No callback guard or transfer result is faked.

Engine PR18 implements scoped native callback exclusion (`intrman:17/18`), including nesting, deferred
native timer/event/task dispatch, retained pending time, invalid release/return rejection and reset.
There is no processor status/register bank. All 21 engine suites and Windows/Linux CI pass. The real
game now reaches DBC's `sifman:7` native bulk-copy boundary, with 21 port suites passing. The copy
must validate every source/destination span, perform the actual data movement and queue a real native
completion; no dummy transfer ID or callback acknowledgement is admitted.

Engine PR19 (`f824703`) now performs checked native library-data-to-EE copies and queues actual
native completion callbacks. All 22 engine suites and Windows/Linux CI pass. Failed batches preserve
all EE output and consume no job ID; device/VU destinations, tag attributes and unknown callback entries
reject. The real boot gets past the DBC copy and reaches a GS CSR read in the old frame-event callback
`0x11fb40` (caller `0x1200f0`), not a draw submission. Its game counters, input countdowns and message
calls must be preserved while removing hardware field/flip behavior. Native rendering is still absent;
zero frames and no Logun-route qualification. C2C/F2F checks remain paused at the user's request.

### Native frame-event input (2026-11-06)

Following TMB/DC's published native adapters, the old frame-event callback now consumes logical parity
from the native scheduler directly. The GS CSR load is an inline native producer and the paired >>13
extraction is retired with a full-body USA signature/mutation guard. No CSR value or register bank is
constructed. The rest of the callback remains compiled game code. A scalar reference begins after the
retired hardware producer; 256 cases compare complete RAM, registers and ordered display/message/input
leaf-call arguments, across both phases, counter wrap, flags, sizes and queue outcomes. No GS read is
answered. Leaf internals and display writes are not covered by that test and remain guarded at runtime.
The live game advances to `_sceSDC` at `0x137a4c`, a PS2 cache-tag scan. Still zero rendered frames.

### Native cache visibility (2026-11-06)

The guarded `_sceSDC` entry now supplies a host memory fence. Its original body only scans PS2 cache
tags/drains cache lines; native program/job buffers already share real host memory. No TagLo/CP0 value,
cache-line bank or scan is supplied. All interior entries remain rejecting guards (1,706 total); mutation
checks cover the original full function. The kernel contract suite verifies context/RAM/kernel/deadline
preservation across 48 cases. The live boot now reaches Csd's priority semaphore request, which explicitly
rejects until the native priority-wait contract is implemented. No frames or playable route yet.

Engine PR20 adds priority semaphore grants using current native task priorities, FIFO ties and dynamic
waiting-task priority changes, with 23 headless suites and green Windows/Linux CI. The real Csi init
request advances to Csm's poll at `0x43fb0/43fb8`. Its real native voice upload copied 4,096 bytes and
queued the linked completion `0x44a40`, which clears the game's own busy flag. The root RPC currently
cannot deliver that completion until it returns, so the poll reaches 1,000,007 work/99,762 native calls.
The work limit is unchanged. The next fix is actual ready-completion delivery at safe native call
boundaries, honoring callback exclusion; it must not fabricate the flag, status or a successful reply.

### Native controller record boundary (2026-11-06)

Engine PR15 (`1b6f1ed`) supplies per-host native algorithm bindings, with 18 headless engine suites
and green Windows/Linux CI. The port's native controller record builder passes 1,536 comparisons
against the USA `ds2o_d` scalar record routine: complete 128-byte output, connected/disconnected,
analog/pressure values, arbitrary buttons/sticks/pressures, and sequence wrap/submission slot.
The fixture supplies no SIO2 transaction and never executes the hardware worker. Capabilities come
from the private relocated game image. Native DBC two-port descriptor registration also passes
repeat/reset/two-host checks. These tests qualify record construction and registration, not the
controller's complete open/configuration/periodic delivery path.

The next increment binds `ds2o_d` startup and worker/resume entries to a per-run native input producer.
It retains compiled DBC descriptor registration and open/submission consumers. Two cooperative tasks
sample the configured `PadInput`, build the qualified game record and delay via a native service; the
original SIO2/VBlank worker never executes. With no controller recipient configured yet, DBC retains
its source behavior and performs no EE delivery. Scripted connected/absent input, record inspection,
delay/resume and reset pass in isolation. The real boot now reaches `Sound_Device_Library.irx` after
native DBC/controller initialization, with 16 port suites passing. Later EE recipient configuration,
rumble/close/reconnect and full route input delivery remain unqualified and reject missing services.

Native Windows PowerShell, existing user-owned USA extraction and function/SDK
maps in `build_usa`, UCRT64 CMake/Ninja/G++, and native FFmpeg:

```powershell
python tools/vp2_bozo.py --stage build
python tools/vp2_bozo.py --stage run
```

The first command builds main + mod001 and runs the port tests. The second
currently returns failure at the audited timer-initialization boundary below. Neither opens
a window nor reads host input. No card directory is passed: cards remain fresh
in memory and user saves are not opened. All generated game bytes and logs stay
in ignored `build_usa/bozo` and `build_usa/bozo_port` directories.

The engine submodule is now pinned to published `4a8f70a`, including native
graphics-services PR5 and compiler PR6/7/8. PR7's local/remote feature branch was
deleted after green CI and merge. VP2S independently builds the new pin and passes
8 port suites plus all 12 headless/default-tools engine suites. The startup stop
and complete flat-memory hash match the prior `e5ddc96` pin exactly
(`97ea3633aa2d1eb6` at the game-owned timing store). This qualifies this bounded
bootstrap path only; it does not qualify VP2S drawing/audio or the route yet.
The engine's old GS state/privileged bank/runtime packet readers are removed in
this published pin. Per-image guarded adaptations are used only when explicitly
audited below; original ELF/runtime load bytes and overlay identities stay intact.
TMB and DC supplied concrete migration and overlay-registration guidance on F2F.

`bozo/CMakeLists.txt` links BozoEngine only. `tools/vp2_bozo_prepare.py` appends
ELF symbols/section metadata and verifies every load segment's original bytes,
addresses, sizes and flags. It merges overlapping function extents, retains
interior labels, and emits an explicit entry map. Retail executable sections
contain embedded data/VU programs: only mapped EE extents are marked executable.
Unmapped targets remain unsupported; zero unknown words in those mapped spans
does not prove complete game coverage. Cross-section function spans are rejected.

Overlay functions have distinct native symbol prefixes. Runtime registration
checks the resident module's 64-byte identity before selecting statically compiled
code. Synthetic tests execute primary/interior indirect calls, interior resumes,
and overlapping module identity selection, including no-match rejection.
Four Python metadata tests verify load preservation, aliases and invalid inputs.
The independent engine headless/default-tools build passes all 11 suites,
including explicit-entry CLI rejection and generated/executed combined
`--entries`/`--patches` tests. DC's review caught a partial-code-word edge case;
entries now require all four bytes inside one executable range, with synthetic
short-range fixtures testing it. No new PCSX2 visual parity is claimed.
Without `--entries`, all 12 generated files for main + mod001 (four chunks each)
are byte-identical against the published `149bf90` recompiler.

## Current boundaries

The first bring-up stopped at syscall 116 (`RFU116_SetSyscall`, instruction
`0x141550`, wrapper `0x14154c`, clock 7342664). The 2026-11-06 follow-up retires
the SDK firmware-discovery installer at `0x141448` using the already-running
native kernel. The next increment retires `InitAlarm`'s firmware installer at
`0x141a38`, including its original patch-payload entry points. The next increment
implements timer record initialization at `0x141bb0` for the audited startup
argument 2. Startup now reaches the separate start-timer routine `0x141df0`
then starts the native SDK clock and binds normalized time queries. Startup now
passes native Get/SetOsdConfigParam and the event-firmware installer retirement.
It reaches game-owned timing setup in `0x100230`, rejecting the write to
`0x10000010` at logged PC `0x1002c0`. It still produces no frames.
IRQ/deadline processing and other timer lifecycle operations remain unported.
See [the bootstrap audit](#sdk-syscall-bootstrap-audit-2026-11-06).

Main contains eight explicit untranslated TLB operations. Mod001 contains 68
untranslated VU random operations (66 VRNEXT, one VRGET, one VRINIT). Those counts
are emitter diagnostics, not evidence of execution. Remaining COP2/VU register
operations are declared as native instruction boundaries and reject execution
until their game algorithms are ported; the engine's macro-mode compatibility
code is not accepted as a fallback (25 guarded sites in main, 9,614 in mod001).
Device I/O, completion polling and VU
microprogram calls also fail explicitly. No GS packet reader or VIF replay is
connected to this runner.

Only main and mod001 are compiled in the verified initial target. `--inputs all`
preparation currently reaches a missing `build_usa/sdk/mod028.toml` and fails;
there is no silent assumption that absent SDK metadata means no imports.
Mapped out-of-code rows are counted in the metadata report, not promoted to code.

## Next increments

1. Continue through native OSD/settings and the remaining SDK bootstrap, then
   bind the existing game-specific native producers and services. Complete timer
   lifecycle/deadline processing when its actual consumers are reached; the IRQ
   callback remains guarded. Upgrade the newer published engine pin in a
   separately verified integration increment. The user expanded the objective:
   continue until Bozo reaches the prior Logun route, not just one blocker.
2. Finish overlay SDK/entry coverage and native bindings. Port the existing native
   VU0 algorithms through the Bozo ABI; remaining algorithms stay explicit faults.
3. Move the game-record model, 2D, fog/lighting and postprocess producers to the
   Bozo native renderer. Preserve PCSX2 image-zone measurements and existing
   capture checks. Do not reconstruct output by consuming GS/VIF chains.
4. Recompile tri-Ace sound libraries and use native sound services/mixer. TMB's
   989SND implementation does not establish VP2S audio compatibility. Move movie
   decode/presentation and abort/release lifetimes separately.
5. Qualify hidden isolated-save scripted routes, menus/battle/audio and reference
   captures before switching the user's normal launcher or retiring PS2Recomp.

## SDK syscall-bootstrap audit, 2026-11-06

`0x1001e8` calls SDK initialization at `0x141558`. It creates two semaphores
(`0x141308`) through native `CreateSema`, then calls `0x141448` before `InitAlarm`.
The offending SetSyscall arguments were index `0x83`, handler `0x1413c0`, return
address `0x141488`. The installer then registers index `0x5a` with `0x141388`.
These are temporary word-search and word-copy routines. It invokes syscall
`0x83` to scan PS2 kernel memory `0x80000000..0x80080000` for both handler
addresses, subtracts `0x20c`/`0x168` from their locations, and writes the common
dispatch-table address to `0x1492a8`. The only mapped main/mod001 read of that
global is in `0x141410`, which copies a firmware table slot to its stack.
This is evidence of SDK firmware discovery, not a generic game callback contract.

The native ABI has no firmware dispatch-table address. The port therefore retires
only `0x141448`, verifies a booted kernel and native GetThreadId/CreateSema/
FlushCache bindings, then returns to its caller without touching RAM or kernel
state. It neither manufactures `0x1492a8` nor registers emulated syscalls. The
caller's result is unused. Other SetSyscall sites remain unsupported. All 75
interior/query addresses in the retired installer and `0x141410` reject explicitly;
non-control instructions also have rejecting native guards for resumed bodies.

`tools/vp2_bozo_kernel.py` validates full SHA-256 signatures of the installer,
search/copy/query helpers, enclosing SDK caller and discovery records before
emitting per-main-image bindings. Every word in these guarded spans is tested
with a mutation. No overlay inherits these addresses. No instruction/load bytes
are patched, and there is no firmware image, address table, hardware-register
shadow, or general SetSyscall acceptance added to the engine.

The port now passes five CTest suites with the private USA ISO available: metadata,
ELF preparation, native kernel boundary, signatures, and bounded headless startup.
The syscall increment originally covered 16 whole-context cases, full flat-memory
equality, preserved semaphore state, every missing readiness binding, 75 rejecting
entries and continued generic SetSyscall rejection. The alarm-installer increment
below extends these tests. They establish the native ABI, not original
firmware-state equivalence; the bounded startup test is not gameplay qualification.

## InitAlarm firmware-installer retirement, 2026-11-06

Reproduced the previous stop with the headless startup fixture before changing
code. `InitAlarm` (`0x141a38..0x141b0c`) checks timer 3's mode at `0x10001810`.
If not already installed, it installs a temporary copy service, copies `0x740`
bytes from `0x149a90` into PS2 kernel address `0x80076000`, copies a 40-byte table
to `0x82000`, and installs six private syscall handlers. The source mapping is:

| Syscall | Relocated handler | Original payload address |
|---|---|---|
| `0xfc`, `0xfe` | `0x80076440` | `0x149ed0` |
| `0xfd`, `0xff` | `0x800762a0` | `0x149d30` |
| `0x12c` | `0x80076488` | `0x149f18` |
| `8` | `0x800766c0` | `0x14a150` |

This is firmware installation, not a game-side alarm queue initializer. Its
startup caller at `0x141558` ignores its result. The port now requires a booted
native kernel/clock and native FlushCache binding, then returns to that caller
without installing PS2 code, configuring a timer, or changing RAM/context/kernel
state. This retires obsolete firmware work; it does **not** supply an alarm API.
The engine's SetAlarm/iSetAlarm/ReleaseAlarm/iReleaseAlarm names are placeholders
whose dispatcher cases remain unsupported. DC and TMB both confirmed on F2F
that their native frame/vblank callbacks are not a generic SDK alarm scheduler.

Additional SHA-256 guards cover the whole installer, its install/copy helpers,
the full payload, copied table and install records. Every guarded source word is
mutation-tested. All entry addresses in the old payload and installer reject,
except the primary installer admission/return. Native instruction guards cover
mapped non-control instructions, including resume paths; non-code payload data
is never declared as executable. General SetSyscall and all six private syscall
requests remain hard failures, as do the four placeholder alarm kernel calls.

The five port suites pass. They now include 32 whole-context cases, full flat-RAM
equality, preserved semaphore state, native callback-queue sequence and next-event
time, missing readiness rejection, and all 591 rejecting entries. The headless
integration fixture requires both retirement messages and the new explicit
`0x10001010` failure. No window/input or persistent card directory is used.
The engine pin remains the previously verified published `e5ddc96`; no shared
engine runtime change was needed for this increment.

## Native timer records and handler lifecycle, 2026-11-06

Reproduced the `0x141c80` timer-mode read before editing. The next initializer
(`0x141bb0..0x141d04`, startup argument 2) is not disposable firmware work:

| Records | Required behavior |
|---|---|
| State at `0x14a238` | If signed handler ID at +8 is nonnegative, return `0x80008001` without changes. Otherwise clear the first 8 bytes and +16, set +20 to the primary pool, and store the allocated handler ID at +8. Preserve other fields. |
| 128 records at `0x1e4180`, stride 64 | Clear all 8 KiB, link each record to its successor, terminate the last with zero. |
| 64 callback records at `0x1e6180`, stride 16 | Set head `0x1e6580`, rewrite only each record's first-word link, terminate the last with zero. Preserve all callback payload bytes. |
| Native handler | Register cause 11, callback `0x142150`, next/argument zero, and the original GP/registration-stack metadata. Enable the logical native kernel cause; no hardware counter/mode/compare access. |

`bozo/native_timer.cpp` implements this data algorithm and lifecycle with native
kernel operations. Only the audited clock argument 2 is admitted for a fresh
initialization; unsupported choices reject before writes or handler allocation.
The already-initialized check precedes clock admission, matching the original.
Return values use the original low-64-bit signed convention; other native ABI
registers, caller stack and interrupt status are retained. No simulated timer
mode, counter or deadline service is created. The callback and initializer's
interior entries reject until separately ported; total rejecting SDK entries
now number 844. Full source/helper/state signatures are checked before generation.

The new `vp2s-bozo-timer-records` suite executes an isolated **original statically
recompiled** initializer, its actual callback-pool helper and critical-section
helper. Native memset and AddIntcHandler are supplied, but its first device read
throws immediately: no register value is returned and no device state is modeled.
Fixture metadata preserves original load bytes, with private generated code in
its own build directory, separate from normal game code generation.

Across 32 dirty-memory cases (16 fresh negative IDs, 16 existing IDs), original
and native flat RAM match after excluding only the original prefix's 96-byte ABI
stack scratch. Native stack/context preservation is checked separately. This
detects accidental clearing of callback payloads as well as missed record writes.
Both initial interrupt states and four prior handler-allocation histories are
covered. Sixteen additional repeat calls leave RAM unchanged and do not duplicate
the native IRQ handler. Native cause delivery queues the expected callback/GP;
the callback then rejects before its old device-dependent body can run.
Unsupported clock admission also leaves RAM and handler allocation untouched.
This is data/lifecycle evidence, not original hardware timing or CPU-cycle parity.

All six port suites pass, including the updated bounded startup check: both
firmware installers return, native timer records are initialized, then
`0x141df0` explicitly fails at its first mode read. There is still no playable
Bozo frame or newly claimed PCSX2 visual parity. TMB/DC were kept informed on F2F.

## Native start/time domain, 2026-11-06

`0x141df0` starts time, reads normalized system time and rearms pending work.
Its initial empty-queue path changes no game data. `0x1423f8` normalizes time
into bus-clock units; startup clock choice 2 has 256-unit API resolution,
one quantum per 512 EE cycles. The source's normalization shifts by eight;
the audited timer divisor is 512 EE cycles (read-only reference cross-check).

`TimerClock` is owned by the concrete host for each run. It stores a logical
epoch/base and lifecycle state; no PS2 mode, counter, overflow register, compare
register or poll side effects exist. Starting returns 0 once, 1 on repeat without
resetting the epoch. Queries return elapsed SDK time at the source API resolution.
Nonempty pending queues remain an explicit unsupported deadline boundary. The
original rearm body and interior entry paths reject; total rejecting entries
are now 997. No process-global map or state leak between Runtime objects is used.

Seven suites pass. The added clock suite checks units/resolution, the old 16-bit
wrap boundary, a duration beyond 32 bits, repeat start, RAM/CPU ABI preservation,
interrupt-status preservation, clock-regression rejection and two-host isolation.
Startup now passes start-timer and stops at the next native settings service,
GetOsdConfigParam, before any frame. Work continues toward the user's route goal.

The following settings increment provides per-host native preferences, with
English/4:3 defaults and ABI version 1 encoded into checked four-byte SDK buffers.
Get/Set at `0x137290`/`0x137280` round-trip writable preferences, preserve native
platform version, reject incomplete buffers without partial mutation, and accept
unaligned memory safely. No firmware memory/config bank is retained. The original
OSD version probe now succeeds and skips its legacy firmware patch. The settings
suite checks defaults, sentinels, write/read/probe behavior, bounds and two-host
isolation. Eight port suites pass to the next event-SDK installer boundary.

The event-SDK installer `0x140b70` copies 0x330 bytes from `0x148c58` to kernel
address `0x80075000`, replaces syscall slots 0x54..0x59, and retains a kernel
object address in `0x148fd0`. That address has three mapped reader functions:
`0x141780`, `0x141910`, `0x141980`. The installer is now retired after native
kernel readiness checks, without manufacturing an address. All three readers,
the original payload and interior entries reject. Source/helpers/install records
have full fingerprints and mutation coverage. The extended kernel suite checks
48 whole-context retirement cases and 1,356 rejecting entries, plus continued
rejection of the private runtime event/flag syscalls. Eight suites pass through
SDK initialization into the game's own main initialization. No generic event
service was claimed by retiring its firmware installer.

## Worker startup and native sleep, 2026-11-06

Engine PR 9 is merged at `32b13ce` after Windows/Linux CI: SDK sleep/wakeup and
cancel operations now use native per-thread credits and ready queues. Thirteen
headless engine suites pass, including two synthetic threads that block, wake,
preempt and resume at exact PCs/results. The merged feature branches are deleted.

The game's two HBLANK setup stores (`0x1002f4`, `0x100300`) are retired by guarded
offline adaptations. The adjacent magic store to `0x1ff0000`, call/checkpoint and
delay-slot structure remain. Thirty-two original/adapted complete-flat-RAM prefix
comparisons pass, stopping the original at its first device request and the
adaptation at the next game call; neither request is answered. Every guarded
source word is mutation-tested. Original ELF load bytes/identities remain intact.
Unported time readers still reject; a native frame adapter remains necessary.

The virtual worker dispatch from `0x119750` reaches `0x11d4e0`, whose prologue/body
were absent from Ghidra's export although its tail labels were present. A fully
fingerprinted supplemental extent through `0x11d7c0` restores ordinary static
compilation without a stub. The existing tail entries remain interior labels.

Timer rearm `0x141f00` only reconfigures a compare device for an empty list and
does nothing during callback dispatch. Those two cases now preserve native game
records. Three original/native full-RAM prefix comparisons (excluding the
original 64-byte ABI stack scratch) and six native state/admission cases pass.
A nonempty list outside dispatch still rejects before mutation. Nine VP2S suites
pass through native worker startup to that pending-deadline boundary. This is
still zero rendered frames; reaching the previous playable Logun route remains
the active task. No hardware counter, alarm callback or visual parity is claimed.

## Native SDK callback deadlines, 2026-11-06

Engine PR 10 is merged at `4f6555d`, with Windows/Linux CI green and feature
branches deleted. Its native keyed deadlines participate in the scheduler's
running checkpoints and idle decisions. Fourteen headless engine suites pass.
Pending replacement/cancellation, ordering, payloads and per-Kernel isolation
are covered. Cancellation does not retract an already queued callback.

VP2S rearm maps its real record deadline into the per-host SDK time domain and
schedules the native callback producer at `0x142150`. A per-host sequence rejects
stale queued invocations. The producer lifts the SDK list/handle/argument and
callback-return rules: zero disables, -1 frees, positive values repeat with the
source's minimum increment. Game callbacks execute their compiled code with the
record's GP and arguments. No timer mode/counter/compare bank or overflow polling
exists; the original device IRQ body remains guarded at all interior entries.

The scalar source slice `0x142230..0x1422f8` and its original list helpers are
compiled separately, without the hardware prefix. Across 768 cases, every flat
RAM byte and all five callback arguments match the native record transition,
including list tails, multiple return values, free/disable/reinsert and 64-bit
times. Fingerprints/mutation tests include both list helpers. Ten port suites
pass, including a real scheduled native timer invocation during startup.

The next reproduced blocker is an original CD callback worker at `0x128f38`
repeatedly calling WaitSema(-1), after native sceCdInit skipped its private SDK
semaphore installer. The 120-field no-frame guard remains active. This is a
native/original SDK ownership mismatch to replace, not a timer wait to bypass.
There are still zero rendered frames and no playable-route or visual claim.

## CD callback ownership and boot RNG input, 2026-11-06

Engine PR 11 is merged at `63ac504`, Windows/Linux CI green and branches deleted.
Fifteen headless engine suites pass. Native sceCdInitEeCB configures checked
callback stack ownership; sceCdCallback owns function/GP and busy replacement
rules. Native read completion delivers reason 1 exactly once after data and
the existing read duration are ready. A private generated ISO exercises actual
dispatcher callback ordering. Dedicated original-worker priority parity and
other CD command callbacks are outside this increment's qualification.

VP2S no longer creates the broken private callback worker or waits on semaphore
-1. Its next HBLANK read at `0x1003c4` only supplies boot RNG entropy. That guarded
inline producer now uses a per-run host seed, while the original game constant
mix, RNG initializer and checkpoint remain compiled normally. No load address,
timer value or register bank is served. Default entropy comes from the host;
`VP2S_BOZO_SEED=<unsigned decimal uint32>` makes diagnostic runs reproducible.
The seed suite verifies full-context ABI preservation, unsigned bounds and two
host instances; all eight words of the source mix are mutation-tested.

Eleven port suites pass. Startup now reaches the original CD status SCMD/RPC
precheck, attempting to bind `0x80000593` without a native CD status service.
The bounded bootstrap fixture identifies that exact no-frame boundary and
ensures the former semaphore -1 loop does not return. Zero rendered frames;
the previous playable route remains unfinished and work continues.

## Native read state and module buffers, 2026-11-06

Engine PR 13 is merged at `cca5653` with all Windows/Linux checks green, feature
branches deleted. Sixteen headless engine suites pass, including TMB's merged
80-byte reference-record reader regression. Native CD status now follows native
initialization/read/stream/error state and documented SDK enums. GetReadPos
reports bytes of an active native copy, zero outside its read-command lifetime;
it never substitutes a sector number or invented partial drive counter.

The next original RPC bind was separated by its saved caller: `0x13fbe0`, the
buffer-load frontend. That ABI has four arguments and leaves t0 stale; the
five-argument path loader uses t0 as a mode. Both source spans and the buffer
free/transfer wrappers are fingerprinted and interior entries guarded. Native
frontends validate path/argument/result spans and pass the real arguments and
start result to a per-run service owner. Async path-load modes remain unsupported.

Module allocations are sparse host-owned vectors behind opaque handles outside
EE RAM. Copies validate every source, destination and descriptor before any
buffer changes. Completed job handles and native completion notification replace
the original SIF transport; no `io.stat`, IOP RAM/device bank or CPU interpreter
is used. The previous generic SifSetDma had no payload destination without a
provider, and wrote a compatibility status bank; this port's native entry bypasses
that code. Reset/free/copy ownership and invalid-batch atomicity are tested.

The first copied module matches the complete extracted `sio2man.irx` bytes (only
its allocation padding lies outside the identity). Its transport dependency is
owned by native pad/card services; no original module executes. All other module
identities currently reject explicitly, including the next `sio2d.irx`. This does
not claim the later custom Dbc_Manager controller protocol or Tri-Ace sound is
implemented. Native module-argument policy is empty-only until each provider's
configuration contract is audited. Twelve port suites pass, with 1,666 remaining
rejecting entry guards. Startup still has zero frames; work toward Logun continues.

The next transport module, `sio2d` 3.03, imports sio2man/secrman and provides the
controller/card transport export. It now uses the same explicit native service
ownership, with native pad/card API readiness required before admission. Provider
identity/argument/readiness/duplicate/reset tests use the actual private source
images. Thirteen port suites pass. Startup reaches the next distinct custom
`Dbc_Manager` 3.16 module; its custom protocol is still rejected and must be
implemented separately. No custom controller data or sound result is fabricated.

## Compiled Dbc_Manager initialization, 2026-11-06

BozoEngine PR 14 is merged at `28f9708`, all Windows/Linux CI green, branches
deleted. The first isolated DBC native entry failed at `thsemap:4`. The engine
now supplies native FIFO semaphore create/delete/signal/iSignal/wait/poll with
cooperative task queues. Seventeen headless suites pass, including mixed wait
reason/credit preservation, delete wakeup, SDK errors, reset and ownership.
Priority semaphore attributes are deliberately unsupported.

The USA Dbc_Manager 3.16 image has a full SHA-256 admission guard. Offline strict
translation compiles all 78 functions, with zero rejected functions; generated
code/data remain in the ignored private build tree. No interpreter, COP0 use or
device backend is involved in the reached initializer. Its original compiled
entry initializes records and a real native semaphore through nine compiled
calls. Repeat/reset/two-host checks pass. Production load now invokes this same
per-run native initializer, then assigns a distinct service handle after success.
It does not acknowledge custom controller requests with fabricated data.

Fourteen port suites pass. The game loads sio2man, sio2d and Dbc_Manager, then
stops explicitly at `ds2o_d.irx`, whose controller-data producer is the next
native boundary. DBC RPC/transfer paths beyond its initializer are not qualified;
the later driver and Tri-Ace sound remain unsupported. All screenshots/playable
Logun-route qualification still await native startup/rendering completion.

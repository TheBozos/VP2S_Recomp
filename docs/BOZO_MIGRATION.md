# BozoEngine migration: initial native bring-up

Started 2026-11-06 after VU0 collision-point increment `d99b892`. The existing
PS2Recomp launcher remains the playable baseline. **The Bozo runner is not
playable: it reaches a logical game-frame boundary but has no native rendered scene.** It is an independent compiled
target, not a replacement DLL loaded into PS2Recomp.

## Build and evidence

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

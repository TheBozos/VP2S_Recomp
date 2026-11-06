# Native replacements for VU0 callers (USA)

VU0 execution, VIF0 uploads and their hardware environment still exist for many
game routines. Lifting code into an AOT fragment that takes a VU execution object
does not remove that dependency. Replacements here take game records directly
and return ordinary native values or update the original EE records.

## Startup key derivation, 2026-11-06

The default startup path no longer uploads the short main-ELF VU0 program or
executes it. The game still obtains its four-word seed through the original
Csi service request 14. `port/vp2s_bootstrap.cpp` reads that completed EE seed
record, computes four affine float expressions, and writes the result into the
same record and EE return register. The existing main code stores the result
at `0x1D59B0` and continues initialization.

There is no VU register file, VU data memory, instruction dispatcher, pipeline,
VIF reader, DMA request or device completion in the native calculation. It uses
ordinary SIMD multiplication and the same native truncating-sum arithmetic as
the music decoder. Host rounding state is restored after calculation.
The native implementation never reads `build/sle_key.txt` or embeds a key.

`tools/vp2_bootstrap.py` validates the USA algorithm's instruction shape **offline**
and extracts its eight float constants to `native_bootstrap/tables.bin` (`VPBK`).
These constants and all game-derived checker files stay in the build tree.
The ordinary port pipeline runs the converter automatically.

The original initializer overlaps three generated C++ entries: main
`0x100230`, carved `0x100800`, and carved `0x1008B8`. The converter lowers all
three. On the default path it skips the upload/wait at `0x10087C..0x1008D0`,
omits its DMA acknowledgement at `0x1008E4`, and replaces the input/output VU
bridge at `0x100908..0x10092C` with a direct native call. The seed-producing RPC
and later initialization remain game code. Source labels and resume targets
must be retained: patching only a function-table entry would miss an inlined
copy inside the full initializer. The lowering is idempotent across port builds
and re-applied after EE source regeneration.

`VP2S_NATIVE_BOOTSTRAP=0` explicitly selects the old comparison path. Missing
native tables fail instead of selecting it automatically.
`VP2S_BOOTSTRAP_TABLES=<file>` selects a converted table file.

## Verification

`VP2S_BOOTSTRAP_RECORD=<file>` saves the single typed seed/result pair privately
for offline checks. It contains sensitive game-derived data and must never be
committed or printed. The optional `vp2s_bootstrap_check` target compares the
native result against the original lifted game algorithm, using a captured seed
and deterministic finite synthetic inputs. The original reference program and
its VU interface belong only to this offline executable, never to the native
startup bridge.

Validation: **65,537 four-word seeds / 262,148 scalar outputs**, zero differences
against the original game calculation under its truncating arithmetic policy.
The captured live result also matches the original EE key at `0x1D59B0`
byte-for-byte; the comparison prints only a boolean. The hidden/offscreen
200-second town/inn replay completed with native music still playing and the
user's memory-card hashes unchanged. No PCSX2 visual or audio parity claim is
made from these arithmetic and initialization checks.
Final-build startup checks also cover both native and explicit comparison modes
for 60 seconds each. Their EE keys match exactly, both complete without a fatal
error, and both preserve the user-card hashes. A subsequent incremental port
build changes no lowered EE sources, confirming converter idempotence.

```powershell
cmake --build build_usa/port --target vp2s_bootstrap_check
build_usa/port/vp2s_bootstrap_check.exe build_usa/native_bootstrap/tables.bin <private-record.bin>
```

## Shared matrix product, 2026-11-06

The default mod001 helper `0x217550` now reads two ordinary 4x4 EE matrix
records and writes their composition directly. Its ABI is `a0 = destination`,
`a1 = first matrix`, `a2 = second matrix`. Columns follow the game's convention:
the result is `second * first`. The native operation retains the original
multiply/add order and truncating float arithmetic, without fused operations.
It reads all inputs before any output write, including in-place input/output
aliases. The wrapper also preserves the ordinary EE stack temporary and GPR
clobbers; it never reads or updates VU state in the default path.

This replaces the shared wrapper's VU0 program at entry `0x250`, rather than
patching individual camera/model callers. The caller audit includes camera
composition at `0x2076E4`, object transforms and mod003 callers. Subsequent VU
operations load their inputs again or use other explicitly loaded records.
This does not replace their remaining algorithms.

An entry offset alone is not an algorithm identity: mod001 also contains a
separate skinning code image with different instructions at `0x250`. The native
helper implements the fixed matrix-record API; it does not inspect uploaded
code or introduce code-image dispatch. `tools/vp2_math.py` validates the common
math implementation offline and generates its original algorithm solely for
the `vp2s_math_check` target. The native product itself needs no converted asset,
VU memory or VIF/DMA input.

`VP2S_NATIVE_MATRIX=0` explicitly retains the original wrapper for comparison.
`VP2S_MATRIX_CHECK=1` is a diagnostic mode: each native result is checked against
the original wrapper on the same live inputs, and a mismatch fails immediately.
It is off by default and is never selected as a fallback.
`VP2S_MATRIX_RECORD=<file>` captures up to 4,096 typed input/result records in
bounded host memory, then persists them on a host worker. No per-call disk flush
runs on the game thread. A force kill before a full capture loses that partial
capture; an orderly shutdown saves it.

```powershell
cmake --build build_usa/port --target vp2s_math_check
build_usa/port/vp2s_math_check.exe --synthetic
build_usa/port/vp2s_math_check.exe <matrix-records.bin>
```

The initial checks cover 65,552 finite synthetic products (including self
composition) and 4,096 live products from the diagnostic replay. Both native
and captured results match the original algorithm exactly. The live capture
contains 129 distinct input pairs. The 200-second diagnostic town/inn replay
completed with music playing and the user-card hashes unchanged.

A separate default-path replay, with original comparison disabled, also completed
200 seconds with music playing and user-card hashes unchanged. Its 4,096 captured
products match the original offline algorithm exactly. Combined with the synthetic
and diagnostic captures, this is **73,744 cases / 1,179,904 scalar outputs**, zero
differences. Static inn zones against the pre-change original-wrapper run have RGB
mean absolute errors of 0.071 (roof), 0.017 (back wall), and 0.000 (right floor)
on the 0..255 scale. Comparing the diagnostic replay's differently aligned camera
initially produced about 8 levels of error; the old-wrapper control separates
that run-to-run camera alignment difference from the native calculation.

## Quaternion normalization, 2026-11-06

The 16 mod001 call sites for common-math entry `0x670` now normalize ordinary
four-float quaternion records directly. This covers seven shared routines,
including the in-place helper `0x2156D0` and the axis-angle conversion helper.
The converter validates the USA instructions and lowers their input/call/sync/
output bundles while preserving EE instructions, branch labels and delay slots.
The default bundles neither load VU registers nor execute a microprogram.

`normalizeQuaternion` retains the original squared-length addition order,
truncating arithmetic, reciprocal-square-root behavior and signed zero results.
It needs no uploaded code, VIF chain, VU data memory or register-state input.
`VP2S_NATIVE_NORMALIZE=0` explicitly selects the original path;
`VP2S_NORMALIZE_CHECK=1` executes it only for strict live result comparison.
Neither is an automatic fallback.

One diagnostic run exposed a real input-storage requirement: routine `0x2159C0`
temporarily puts its EE stack in the game's 16 KiB working area. The failing
record was `0x70003D60`, not a numerical mismatch. Native record access now
accepts bounded, aligned spans of that existing storage, as the native model
path already does. This reads game data directly; it adds no scratch DMA,
hardware timing or VU transport. Other users of that storage remain separate
replacement work.

The offline checker validates **65,560 finite synthetic cases / 262,240 scalar
outputs**, including zero, signed-zero, axis and large/small finite inputs,
with zero differences against the original algorithm. A 200-second strict
comparison replay passed at least **638,976 live normalizations**. Its bounded
64-record capture and an independent native-only 4,096-record capture also
match exactly: **69,720 captured/synthetic cases / 278,880 scalar outputs**
in total. Both replays preserved the user's memory-card hashes.
A further 300-second native-only town/inn replay passed more than one million
normalizations, with streamed music still playing at 290 seconds and the
user-card hashes unchanged.
Walking replays produced differently positioned inn cameras, so they were not
used to infer a shading change. A separate stationary-town original/native pair
has static-zone RGB mean absolute errors of 0.006 (central wall), 0.419 (left
wall) and 0.368 (ground) on the 0..255 scale at capture 42. Whole-frame error
also includes independently timed NPCs, birds and effects. This is a native
regression comparison, not a new PCSX2 parity claim.

`VP2S_NORMALIZE_RECORD=<file>` uses bounded host-memory capture and background
persistence. `VP2S_MATH_CAPTURE_LIMIT=1..4096` controls both math capture limits.
The optional offline checker accepts these typed records:

```powershell
build_usa/port/vp2s_math_check.exe --normalize-synthetic
build_usa/port/vp2s_math_check.exe <quaternion-records.bin>
```

## Shared quaternion-to-matrix helper, 2026-11-06

The mod001 helper `0x216730` now converts the quaternion record at `a1` directly
to the four-column matrix record at `a0`. This replaces its common-math VU0
entry `0x720`. The game does **not** normalize this input inside the conversion;
the native calculation preserves that behavior, operation order, truncating
sums and positive-zero fourth components. Its final column is `(0,0,0,1)`.
Inputs are read before output writes, including an in-place quaternion/matrix
record. The helper retains its ordinary EE temporary stack record and `v1`/`t1`
clobbers, without reading or writing VU registers in the default path.

The direct-caller audit finds 16 distinct mod003 call instructions. Callers
consume the matrix record; subsequent inverse/matrix operations reload their
own matrix inputs. The later scalar-length macro sequence uses separately
loaded registers 24/25. The conversion's temporary vector registers are not a
return interface. Separate inline `0x720` bundles were outside this shared-helper
replacement; the following step covers those and the mod028 bundles.

`VP2S_NATIVE_QUAT_MATRIX=0` explicitly selects the old helper.
`VP2S_QUAT_MATRIX_CHECK=1` compares against it on the same live input and fails
on any output mismatch; it is never an automatic fallback. The native
calculation itself needs only a quaternion, without VIF packets, uploaded code,
VU memory or execution state. The original 26-pair USA algorithm is validated
offline and emitted only for the optional checker.

```powershell
build_usa/port/vp2s_math_check.exe --quat-matrix-synthetic
build_usa/port/vp2s_math_check.exe <quaternion-matrix-records.bin>
```

Synthetic validation covers **131,096 cases / 2,097,536 scalar outputs**, zero
differences against the original algorithm. These include 65,536 finite
general inputs, their 65,536 normalized counterparts, and 24 signed-zero/axis/
small/large cases. The previous matrix-product and normalization suites also
pass unchanged. `VP2S_QUAT_MATRIX_RECORD=<file>` captures typed input/result
records with the existing bounded-memory/background-write mechanism.

Two independent live captures (strict comparison and native-only) each contain
4,096 conversions with zero reference or captured-output differences. Combined
with the synthetic suite this is **139,288 cases / 2,228,608 scalar outputs**.
At stationary town capture 39, native versus strict-comparison RGB mean absolute
errors are 0.014 (central wall), 0.045 (left wall), and 0.035 (ground), on the
0..255 scale. A bird crosses the left-wall region at capture 40, so that frame's
whole-region difference does not measure static geometry. These are regression
checks against the existing port, not new PCSX2 parity evidence.
The 200-second strict-comparison town/inn run passed at least 16,384 helper
calls. The independent 300-second native-only route also completed, with music
still playing at 290 seconds. Both preserved the user's memory-card hashes.
A separate 135-second native capture run checked the first live result exactly
and likewise preserved the card hashes; its one record is additional to the
139,288-case total above.

## Inline quaternion-to-matrix conversions, 2026-11-06

All **56 remaining inline `0x720` sites** in mod001, mod003 and mod028 now use
native quaternion/matrix records by default. `tools/vp2_inline_math.py` validates
their input load, instruction word, intervening EE effects and output stores,
then lowers the full functions and their overlapping carved entries. The
generated-source audit finds 105 guarded inline microprogram-call copies; the
only remaining unguarded generated `0x720` call is the shared helper already
bypassed by its native hook. Repeating conversion changes no EE sources.

At 53 sites, the calculation writes three rotation columns (four in the two
mod001 routines). The intervening instructions only establish pointers, sync
the old calculation, or initialize the translation column; the converter
symbolically validates that data flow before moving the native record write
to the calculation point. Translation stores remain ordinary EE effects.
Native synchronization writes the documented GPR value directly, and constant
identity-column stores no longer read a VU register.

Three mod028 sites (`0x48426C`, `0x485724`, `0x48C9C4`) keep a rotation across an
object callback, then compose it with the object's attachment matrix. Replacing
only their microcall would leave a VU register dependency. Instead, the rotation
uses the routine's existing final stack matrix buffer (`SP+0x50`, `SP+0xB0`,
`SP+0x90` respectively); the callback remains in its original position. The
subsequent four-column multiply/add and record stores are native too. No host
register shadow, VU context, instruction replay or extra game-stack allocation
is introduced. The native attachment product preserves the existing EE
multiply/add arithmetic, separately from the common VU matrix product's
truncating-sum policy. An offline comparison caught that distinction before
the replacements were committed.

`VP2S_NATIVE_INLINE_QUAT_MATRIX=0` explicitly selects the old inline sequences;
`VP2S_INLINE_QUAT_MATRIX_CHECK=1` executes them only for strict result comparison.
`VP2S_INLINE_QUAT_MATRIX_RECORD=<file>` captures bounded typed input/result
records. `VP2S_INLINE_QUAT_FAILURE_RECORD=<file>` persists the first failing
input/original result, then aborts; there is no fallback.

The first strict run exposed a shared numerical edge after 110,592 exact calls:
site `0x39F324`, quaternion in the game's working stack at `0x70002FA0`. The
captured original output matches the offline reference, while the initial
native conversion made two diagonal entries one ULP below 1. Squared tiny
components became subnormal terms which the original calculation reduces to
signed zero. The native conversion now applies that reduction to its inputs
and intermediate terms. This also corrects the shared helper's conversion.
The minimal regression input `(0,0,1e-20,1)` and the actual captured input both
match exactly; no addressing or runtime execution-state workaround was needed.

Offline checks now cover:

- 131,096 general/normalized/axis conversion cases;
- 1,296 signed-zero and fixed-axis component combinations;
- 65,537 small-component cases spanning finite component exponents 0..127;
- two independent 4,096-record final live captures and the captured regression;
- 196,608 rotation-plus-attachment compositions, using the original conversion
  and EE macro arithmetic as independent references, checking all three stack
  layouts, buffer guards, and round-to-nearest/toward-zero host modes.

That is **402,730 cases / 6,443,680 scalar outputs**, zero differences.

```powershell
build_usa/port/vp2s_math_check.exe --quat-matrix-synthetic
build_usa/port/vp2s_math_check.exe --quat-matrix-zeros
build_usa/port/vp2s_math_check.exe --quat-matrix-small
build_usa/port/vp2s_math_check.exe --attachment-synthetic
```

Live route coverage is five sites: `0x2158C4`, `0x37923C`, `0x380124`, `0x39F324`,
`0x396A94`. The remaining sites are validated statically and through their shared
offline calculations; in particular the mod028 callback paths have not been
exercised by the town/inn route. A battle/effect scene is still needed for that
live integration coverage. These checks do not establish new PCSX2 parity.

The final strict-comparison and native-only town/inn runs both completed 300
seconds with user-card hashes unchanged. The strict run passed at least 262,144
inline conversions; the native-only run passed at least 245,760 and still had
music playing at 290 seconds. The diagnostic run's old-execution path lost its
music input later in the run, so it is evidence for math comparison, not an
audio pass. The independent native runs retain continuous late music playback.
At final town capture 39, native versus diagnostic static-zone RGB mean absolute
errors are 0.015 (central wall), 0.053 (left wall), and 0.173 (ground), on 0..255.

## Rigid-transform inverse translation, 2026-11-06

The shared rigid-transform inverse helper `0x216E40` no longer needs VU0
registers or macro execution by default. Its only VU block (`0x216EE0` through
`0x216EFC`) multiplied three matrix rows by the xyz components of a temporary
translation record. The native operation reads those ordinary game records,
performs the same ordered three products/two sums, and writes xyz back while
preserving the temporary's fourth component bit-for-bit. The scalar transpose,
negation, stack handling and EE control flow remain the original recompiled
game code. This is the game's rigid-transform inverse, not the separate general
matrix-inverse helper `0x217310` / microprogram `0x2D0`.

The lowerer validates the exact eight-instruction USA block against the ELF
and rewrites both the full function and its overlapping `0x216EB8` continuation.
Repeating the converter changes no generated EE files. All four vector loads,
three arithmetic macro instructions and the old result store are gated to an
explicit original-comparison path. The native call takes a three-row matrix
record and the game's existing temporary vector, never VU memory, instructions,
register inputs or packet data. It introduces no timing or hardware transport.

The 11 direct caller instructions include camera/projection builders, object
transforms and model helpers. They consume the returned matrix record; later
vector operations reload their operands or use different explicitly loaded
registers. No temporary VU register result is retained as a return interface.

`VP2S_NATIVE_RIGID_INVERSE=0` explicitly selects the original macro block.
`VP2S_RIGID_INVERSE_CHECK=1` checks each native temporary against the original
four-component result and fails immediately on a mismatch, without fallback.
`VP2S_RIGID_INVERSE_RECORD=<file>` captures bounded typed input/result records;
`VP2S_RIGID_INVERSE_FAILURE_RECORD=<file>` writes a failing record before aborting.
The records also retain the host rounding/denormal controls for offline replay.
The native arithmetic follows the caller's host mode, preserving this EE
sequence's small-result reduction without using the microprogram product's
different truncating-sum policy.

The offline original-EE-macro comparison passes **131,216 cases / 524,864 scalar
outputs**. It covers round-to-nearest and toward-zero, each with denormal controls
on and off, general finite inputs, tiny components, signed-zero/axis cases and
arbitrary fourth-component payload bits. Two independent 4,096-record live
captures (three distinct input records each) also match exactly. Together these
are **139,408 cases / 557,632 scalar outputs**, zero differences. Live captures
use round-to-nearest with denormal controls off.

Both the strict-comparison and independent native-only town/inn routes completed
300 seconds with streamed music still playing at 290 seconds and user memory-card
hashes unchanged. Each passed at least 200,704 calls; the native-only run passed
208,896. At town capture 40, native versus comparison static-zone RGB mean
absolute errors are 0.003 (central wall), 0.247 (left wall), and 0.080 (ground)
on the 0..255 scale. These are native regression checks, not new PCSX2 parity
evidence. Other inverse/projection helpers and VU0 operations remain separate
replacement work.

```powershell
build_usa/port/vp2s_math_check.exe --rigid-inverse-synthetic
build_usa/port/vp2s_math_check.exe <rigid-inverse-records.bin>
```

## Projection-builder matrix products, 2026-11-06

The two projection builders `0x2178B0` and `0x2179E0` now compose their local
projection and inverse-view matrix records natively. Their scalar projection
setup and ordinary EE register/stack behavior remain the original recompiled
game code. Together with the preceding rigid-inverse replacement, both builders
are free of VU state access in the default path. This does not replace other
projection/camera helpers elsewhere in the game.

The USA ELF contains two identical 28-instruction product blocks, validated
before lowering: eight matrix loads, sixteen ordered multiply/add macro
instructions, and four result stores. `tools/vp2_math.py` replaces both blocks
with a typed record product at the first result store. All inputs are copied
before output writes; output may alias an input. No VU register, accumulator,
flag, microcode, VIF/DMA input or instruction pipeline supplies the native
calculation. Repeating conversion changes no generated EE source.

The native product follows the EE sequence's host rounding/denormal controls,
operation order and reduction of tiny intermediate terms. It deliberately
does not reuse the common microprogram product's different truncating-sum
policy. `VP2S_NATIVE_PROJECTION_PRODUCT=0` explicitly selects the original macro
blocks; `VP2S_PROJECTION_PRODUCT_CHECK=1` compares native results with them and
fails on any mismatch. Neither is an automatic fallback.
`VP2S_PROJECTION_PRODUCT_RECORD=<file>` captures bounded typed records with host
floating-point controls; `VP2S_PROJECTION_PRODUCT_FAILURE_RECORD=<file>` saves
a failed original result before aborting.

Arithmetic validation passes **131,584 cases / 2,105,344 scalar outputs**, zero
differences against the original EE macro sequence. It covers general finite
and tiny inputs under four host control modes, plus both builders' distinct
projection layouts.

The town/inn route did not execute either builder. To qualify the unexercised
integration, the offline checker additionally runs the **complete original and
rewritten game functions**, including their rigid-inverse call, against private
RAM buffers and CPU records. It uses the exact production native bridges from
`port/vp2s_math.cpp`. The converter emits these original/reference functions
only into the ignored offline checker source, never into the game. Their one
direct helper call becomes an ordinary C++ call, and bounded EE loops finish
without scheduler checkpoints; there is no runtime instance or device/timing
model in this harness.

All **2,048 full-function cases** pass: both builders, four host control modes,
128 camera/input records and separate/aliased output. Checks cover the entire
private RAM buffer (including stack temporaries), all general and scalar FP
registers, return PC and FP status. Native execution also preserves the initial
VU vector/accumulator/flag state. The camera-update caller consumes the matrix
records; its callers have no subsequent VU-result dependency.

Two hidden, input-isolated, private-card 300-second town/inn runs also completed
with music playing at 290 seconds and user-card hashes unchanged. These are
general regression checks, **not live coverage of either projection builder**.
Live scene coverage of these paths and new PCSX2 parity remain unverified.

```powershell
build_usa/port/vp2s_math_check.exe --projection-product-synthetic
build_usa/port/vp2s_math_check.exe --projection-functions
build_usa/port/vp2s_math_check.exe <projection-product-records.bin>
```

## Column-layout rigid-transform inverse, 2026-11-06

The counterpart helper `0x216CB0` now reads ordinary matrix/vector records for
its translation calculation too. Its scalar code first clears and transposes
the output basis; the native operation reads that basis and the source's fourth
vector, computes the ordered translation product, negates xyz and preserves the
source vector's fourth component bit-for-bit. It reuses the verified native
rigid-inverse arithmetic and adds only the final xyz sign change.

The exact eleven-word USA tail (`0x216D34` through `0x216D5C`) is validated
against the ELF. Four vector loads, the constant -1 construction and four
arithmetic macro operations are now gated to explicit comparison. The `JR` is
retained, and the native record write replaces only its delay-slot result store.
The full function and overlapping `0x216D28` continuation are lowered; repeated
conversion changes neither. Default execution has no VU register, accumulator,
flag or pipeline dependency. The direct caller at `0x2485F8` consumes the output
record and performs no later VU operation before returning.

This extends the existing rigid-inverse controls: `VP2S_NATIVE_RIGID_INVERSE=0`
selects the old block, and `VP2S_RIGID_INVERSE_CHECK=1` compares the entire native
four-component result against the original. A mismatch fails without fallback.
`VP2S_RIGID_INVERSE_COLUMN_RECORD=<file>` captures bounded typed records and
host floating-point controls; `VP2S_RIGID_INVERSE_COLUMN_FAILURE_RECORD=<file>`
saves the first failing original result before aborting. Column-layout records
have a separate type tag from the non-negated row-layout calculation.

The arithmetic checker passes **131,216 cases / 524,864 scalar outputs** across
four host floating-point control modes, general/tiny values, signed zeros and
arbitrary untouched fourth-component payloads. Independent strict-comparison
and native-only live captures add 4,096 records each (2,831 and 2,951 distinct
inputs respectively), all exact. Together this is **139,408 arithmetic/captured
cases / 557,632 scalar outputs**, zero differences.

The offline full-game-function harness also passes **1,024 cases** using the
exact production bridge: four control modes, 128 nonzero column-layout camera
transforms and separate/aliased input/output records. Whole private RAM, general
and scalar FP registers, FP status, return PC, branch PC and delay-slot completion
agree with the original function. Native VU vector/accumulator/flag state remains
untouched. The original aliased-record behavior is preserved, including the
scalar destination initialization that precedes source reads. The prior 2,048
projection-function checks still pass after extending the harness.

Both 300-second hidden town/inn runs completed, each passing at least 200,704
calls of this helper. The strict comparison found no mismatch; the native-only
run used no VU state for the operation. Music was still playing at 290 seconds
in both runs, and the user's memory-card hashes were unchanged. At town capture
40, native versus comparison static-zone RGB mean absolute errors are 0.004
(central wall), 0.850 (left wall) and 0.485 (ground), on the 0..255 scale. These
are port regression checks; no new PCSX2 parity claim is made.

```powershell
build_usa/port/vp2s_math_check.exe --rigid-column-synthetic
build_usa/port/vp2s_math_check.exe --rigid-column-functions
build_usa/port/vp2s_math_check.exe <rigid-column-records.bin>
```

## Point/plane offset dot product, 2026-11-06

The shared helper `0x214590` now evaluates `(point - origin) dot axis` from its
three ordinary vector records, without VU state in the default path. The result
replaces x in the game's temporary point record; its original y/z/w bits are
preserved. The existing scalar load returns x in `f0`, and the EE stack/register
setup and return remain unchanged. No axis normalization is added.

The converter validates the exact nine-word USA block (`0x214598` through
`0x2145B8`) against the ELF before lowering it. All three vector loads, the xyz
subtract/product, construction of the unit vector, accumulator addition and
final multiply/add are gated to explicit comparison. Default execution writes
the native temporary record at the original result-store point, with no VU
register, accumulator, flag, code image, data memory, packet or timing input.
All three input records are copied before the result is written, including when
the point aliases the stack temporary. The lowerer is repeatable without edits
to already converted sources.

The direct caller at `0x20DBF8` consumes `f0` for a signed plane test, adding its
plane coefficient before branching. Subsequent VU operations load their own
records and do not consume this helper's temporary vector results.

`VP2S_NATIVE_OFFSET_DOT=0` explicitly selects the old block.
`VP2S_OFFSET_DOT_CHECK=1` compares all four temporary components against it and
fails on any mismatch, without fallback. `VP2S_OFFSET_DOT_RECORD=<file>` captures
bounded typed input/result records with host floating-point controls;
`VP2S_OFFSET_DOT_FAILURE_RECORD=<file>` persists a failing original result before
aborting. The native arithmetic retains the ordered xyz difference, products,
`x+y` then `+z` sum, caller host controls and reduction of tiny results.

The original-EE-arithmetic comparison passes **131,216 cases / 524,864 scalar
outputs**, including four host control modes, general finite/tiny components,
signed-zero/axis cases and opaque unused fourth-component payloads. The offline
complete original/native function comparison adds **1,024 cases**, covering
whole private RAM, general/FP registers, FP status, return/branch PC and delay-slot
completion, with separate or stack-aliased point records. The exact production
bridge is used; native VU vector/accumulator/flag state remains untouched.

The strict-comparison and independent native-only town/inn runs both reached
this helper and completed 300 seconds without a mismatch or runtime error.
Music was still playing at 290 seconds and user-card hashes were unchanged.
This is sparse live coverage: neither run filled the default 4,096-record
capture, so no large live-call total is claimed. A separate bounded one-record
native capture completed 135 seconds with unchanged save hashes; its actual
live input/result matches the original offline arithmetic exactly. Including
that sample gives **131,217 arithmetic/captured cases / 524,868 scalar outputs**.
The extended harness also retains passing results for the preceding 1,024
column-inverse and 2,048 projection-function checks. No new PCSX2 parity is
claimed from these regression and arithmetic checks.
At town capture 40, native versus comparison static-zone RGB mean absolute
errors are 0.020 (central wall), 0.734 (left wall), and 0.394 (ground), on 0..255.

```powershell
build_usa/port/vp2s_math_check.exe --offset-dot-synthetic
build_usa/port/vp2s_math_check.exe --offset-dot-functions
build_usa/port/vp2s_math_check.exe <offset-dot-records.bin>
```

## Inverse with an intervening integer transpose, 2026-11-06

The helper `0x216D60` is now free of VU state in its default path. Its translation
calculation originally ran before an integer transpose, but stored its result
after that transpose had written the output matrix. Reading the original source
again at the result store would be wrong for partially overlapping records.
The native boundary therefore reads the newly transposed **output** matrix and
the existing translation temporary, reconstructs the three original basis rows
with ordinary SIMD shuffles, and reuses the verified native translation product.
The temporary's fourth component is preserved; the original scalar negations
and remaining output stores stay as recompiled EE game code.

The converter validates the full 25-word USA interval `0x216DA4..0x216E04`,
including the intervening integer loads, rearrangements and stores. It gates
the four VU loads and three arithmetic macro operations to explicit comparison,
keeps the integer transpose intact, and replaces the final VU result store
with a typed record call. No input/result register shadow, suspended native
execution context, VIF packet, VU memory or timing model is introduced. Repeating
conversion changes no generated EE source. Six direct callers consume matrix
records and reload the operands for subsequent VU work.

This uses the existing `VP2S_NATIVE_RIGID_INVERSE=0` comparison-only legacy
selection and `VP2S_RIGID_INVERSE_CHECK=1` strict original-result check. A mismatch
fails without fallback. `VP2S_TRANSPOSED_RIGID_RECORD=<file>` captures bounded
typed matrix/vector/result records and host floating-point controls;
`VP2S_TRANSPOSED_RIGID_FAILURE_RECORD=<file>` saves the first failed original
result before aborting.

The independent arithmetic oracle reconstructs basis rows with scalar indexing
and applies the original EE operations. It passes **131,216 cases / 524,864
scalar outputs**, including four host control modes, general/tiny values,
signed-zero/axis cases and opaque fourth-component bits. Two 64-record live
captures add 49 distinct inputs each and match exactly. Combined this is
**131,344 arithmetic/captured cases / 525,376 scalar outputs**, zero differences.

The complete original/native game-function harness additionally passes **2,048
cases**: four host control modes, 128 transforms and four record relationships
(disjoint, identical, and partial overlaps in both directions). Whole private
RAM, all general/FP registers, FP status, return/branch PC and delay-slot completion
match. The native function uses the exact production bridge and leaves its
initial VU vector/accumulator/flag state untouched. The input/output overlap
behavior follows the original game code, including its early destination clear.

Both hidden, input-isolated town/inn runs completed 300 seconds and exercised
at least **581,632 calls each**. The strict comparison found no mismatch; the
independent native run used no VU state for this helper. Streamed music was still
playing at 290 seconds, and user-card hashes remained unchanged. The earlier
1,024 offset-dot, 1,024 column-inverse and 2,048 projection-function regression
checks still pass after extending the overlap harness.

At town capture 40, native versus comparison RGB mean absolute errors are 0.021
(central wall), 0.249 (ground), and 0.023 in an unobstructed lower-left stone
patch `(10,150)..(60,180)`, on 0..255. The usual upper-left wall region is crossed
by the animated bird at different positions and gives 16.909; it is not a static
geometry comparison in that frame. These are port regression checks, not new
PCSX2 parity evidence.

```powershell
build_usa/port/vp2s_math_check.exe --transposed-rigid-synthetic
build_usa/port/vp2s_math_check.exe --transposed-rigid-functions
build_usa/port/vp2s_math_check.exe <transposed-rigid-records.bin>
```

## Object-update linear vector products, 2026-11-06

Four terminal blocks in object helpers `0x1FB120`, `0x1FB240`, `0x1FB330`
and `0x1FB530` now compute a typed three-basis-vector product directly from
ordinary EE records. The result stores are `0x1FB200`, `0x1FB318`, `0x1FB448`
and `0x1FB640`. Their identical eight-word USA instruction sequence is validated
before lowering all four generated functions; repeating conversion changes no
generated EE source. The default path has no VU register, accumulator, flag,
microcode, transport or timing dependency for these blocks.

The operation is `basis[0]*vector.x + basis[1]*vector.y + basis[2]*vector.z`,
with ordered multiply/adds under the caller's host FP mode and signed-zero
reduction of tiny intermediate results. Input `vector.w` is ignored, including
arbitrary payload bits. All four basis components participate: output `w` is
computed, rather than forced to one or preserved as in the rigid-inverse helper.
The bridge copies every input before writing the destination. The original EE
control flow, callbacks, integer transposes, copies and epilogues remain intact.
The earlier four-column products in `0x1FB330` and `0x1FB530` remain separate VU
replacement work; these functions are not claimed wholly VU-free.

`VP2S_NATIVE_LINEAR_VECTOR=0` explicitly selects the original diagnostic path.
`VP2S_LINEAR_VECTOR_CHECK=1` runs the original block and strictly checks its
result; a mismatch aborts without fallback. `VP2S_LINEAR_VECTOR_RECORD=<file>`
captures bounded typed basis/vector/result records plus host FP controls.
`VP2S_LINEAR_VECTOR_FAILURE_RECORD=<file>` saves a mismatching original result.
There is no default read or write of VU state.

The independent original-operation arithmetic oracle passes **131,216 cases /
524,864 scalar outputs** over four host FP control modes, ordinary and tiny
terms, signed zeros, arbitrary vector-w payloads and nontrivial result-w values.
Two 64-record live captures each contain 23 distinct inputs and replay exactly,
giving **131,344 arithmetic/captured cases / 525,376 outputs** in total.
All four exact generated blocks additionally pass **2,048 original/native
cases**, including separate output, output overlapping the basis at offsets
zero and 16, and output aliasing the vector. Whole private RAM, CPU records and
the final instruction PC match; native VU vector/accumulator/flags are untouched.

An additional **1,024 real callback-continuation cases** enter `0x1FB240` at
`0x1FB25C`, after its virtual callback. They check both the dirty-flag skip and
calculation paths, the actual integer transpose and object output, register
restores, return PC and delay slot across four host modes. Full private RAM and
CPU state match, with native VU state unchanged. Entering the callback in this
offline fixture throws a test error: the callback itself is not fabricated or
claimed tested by this fixture. The prior 2,048 complete transposed-inverse
function checks also remain passing.

Both hidden, input-isolated town/inn runs completed 300 seconds, reached all
four sites and preserved user-card hashes. Strict comparison found no mismatch.
Each run logged at least 49,152 calls at `0x1FB448` and 86,016 at `0x1FB640`;
the other two sites were observed but did not reach the 4,096-call logging
threshold. Music input was still playing with nonzero PCM peaks at 290 seconds.
This establishes route coverage, not coverage of every scene or every caller.

At Waiting Area capture 40, native-versus-strict-comparison RGB mean absolute
errors are 0.015 (central wall), 0.082 (upper-left wall), 0.302 (ground) and 0.017
(unobstructed lower-left stone), on 0..255. These are port regression checks,
not new PCSX2 parity evidence or a performance benchmark.

```powershell
build_usa/port/vp2s_math_check.exe --linear-vector-synthetic
build_usa/port/vp2s_math_check.exe --linear-vector-blocks
build_usa/port/vp2s_math_check.exe --linear-vector-functions
build_usa/port/vp2s_math_check.exe <linear-vector-records.bin>
```

## Full object matrix/vector products, 2026-11-06

The next native boundary replaces the full four-weight transforms in object
helpers `0x1FB330`, `0x1FB530` and `0x1FB7C0`. Their result stores are
`0x1FB418`, `0x1FB610` and `0x1FB8A0`. All three identical ten-word USA blocks
are validated before lowering. Every input is loaded from the game's ordinary
EE matrix/vector records before any destination write; no VU register shadow,
accumulator, flags, memory, packet replay or timing model is used by default.

Unlike the preceding three-weight operation, **vector.w participates**:
`matrix[0]*x + matrix[1]*y + matrix[2]*z + matrix[3]*w`. All four output
components are computed. Ordered products/sums retain the caller's host FP
mode and reduction of tiny intermediate results to signed zero. This is
separate from the common matrix microprogram's truncating-sum policy. Existing
integer transposes, dirty flags, callbacks, stores and return code are retained.
The subsequent three-weight blocks reload their inputs and remain native.

`VP2S_NATIVE_MATRIX_VECTOR=0` explicitly selects original diagnostic execution;
`VP2S_MATRIX_VECTOR_CHECK=1` strictly compares the original result before native
storage and aborts on mismatch, without fallback. `VP2S_MATRIX_VECTOR_RECORD`
captures bounded typed matrix/vector/result records plus host FP controls;
`VP2S_MATRIX_VECTOR_FAILURE_RECORD` saves the first failed original result.

The original-operation oracle passes **131,264 cases / 525,056 scalar outputs**
over four host FP control modes. Cases include ordinary/tiny values, signed
zeros, every contributing axis including w, and nontrivial fourth matrix
components. **1,536 exact generated-block cases** cover all three sites with
separate output, overlapping matrix output at offsets zero and 16, and vector
output aliasing. Whole private RAM, CPU records and terminal PC match; native
VU vector/accumulator/flags remain unchanged.

The real continuation harness additionally passes **2,560 cases**. For
`0x1FB330` it enters `0x1FB3E4`, after the existing object helper; for `0x1FB530`
and `0x1FB7C0` it enters the actual post-callback PCs `0x1FB54C` and `0x1FB7DC`.
The latter two cover both dirty-flag skip and calculation paths, including their
integer transpose. The first two include their subsequent native three-weight
product. Whole private RAM, GPR/FPR/status, return/branch PC and delay slot match,
with native VU state untouched. Reaching a preceding callback/helper in these
offline fixtures throws: those callees are not fabricated or claimed tested by
this harness. The previous 2,048 three-weight block and 1,024 continuation checks
still pass after extending the fixtures.

Two 64-record live captures each contain 22 distinct inputs and replay exactly
against the original operations. Combined with the synthetic oracle this gives
**131,392 arithmetic/captured cases / 525,568 scalar outputs**, zero differences.
Repeating conversion changes no generated EE file. At Waiting Area capture 40,
native-versus-strict-comparison RGB mean absolute errors are 0.000 (central
wall), 0.177 (upper-left wall), 0.100 (ground) and 0.008 (lower-left stone), on
0..255. These are port regression checks, not new PCSX2 parity evidence or an
FPS benchmark.

At inn capture 54 the corresponding RGB mean absolute errors are 0.376 (roof),
0.163 (back wall) and 0.001 (floor), on 0..255, with the same camera view.

Both isolated routes completed 300 seconds and preserved user-card hashes.
All three sites have live strict-comparison and native-only coverage; no
transform mismatch occurred. Each logged at least 49,152 calls at `0x1FB418`,
86,016 at `0x1FB610` and 12,288 at `0x1FB8A0`.
The native-only route retained playing music input
at 290 seconds with nonzero PCM peaks. The diagnostic route instead lost music
around 210 seconds: decoding stopped at 7,680 blocks, with 657 input halves.
It counts as arithmetic/integration evidence, **not an audio pass**. The retained
pre-change `inline_quat_final_checked` log has the same 7,680-block stop and
silent late input (658 halves), while its native counterpart continues to
11,904 blocks and playing music. Thus the diagnostic failure predates this
increment; its cause remains unresolved and no audio/hardware workaround was
introduced. These route checks do not establish whole-game RAM equivalence or
coverage of every scene/caller.

A further isolated 300-second control used the retained pre-change executable
from `linear_native_20261106`. It preserved user-card hashes and continued
decoding/playing music through 290 seconds (973 input halves, nonzero peaks),
as does the new default build. That control did not reproduce the diagnostic
audio failure; the earlier matching failure log establishes its prior existence.

```powershell
build_usa/port/vp2s_math_check.exe --matrix-vector-synthetic
build_usa/port/vp2s_math_check.exe --matrix-vector-blocks
build_usa/port/vp2s_math_check.exe --matrix-vector-functions
build_usa/port/vp2s_math_check.exe <matrix-vector-records.bin>
```

## Paired point/direction transform, 2026-11-06

Object helper `0x1FB6A0` now transforms its cached point and direction through
ordinary EE records. The point uses all four matrix weights; the direction
uses only xyz weights, while both results compute all four components. The
native calculation reuses the verified `matrixVectorProduct` and
`linearVectorProduct` algorithms. It snapshots the matrix and both vectors,
computes both results, then writes point followed by direction in place.
Computing the second result after writing the first would break aliases.

The converter validates the exact 15-word USA block `0x1FB768..0x1FB7A0`,
gates its six VU loads/seven arithmetic operations to explicit comparison and
replaces the first result store with the paired native record call. The second
legacy store is inactive by default. Callback dispatch, dirty flags, the integer
transpose, preceding record copies and epilogue remain ordinary recompiled game
code. There is no default VU register/ACC/flag, packet, memory or timing input
to the replacement, and no native input/result shadow carried between stores.

`VP2S_NATIVE_POINT_DIRECTION=0` explicitly selects the original diagnostic path;
`VP2S_POINT_DIRECTION_CHECK=1` compares both original results before native
storage, aborting without fallback on mismatch. `VP2S_POINT_DIRECTION_RECORD`
captures bounded typed matrix/two-vector/two-result records plus host FP controls;
`VP2S_POINT_DIRECTION_FAILURE_RECORD` saves the first failed original result.

The independent original-operation oracle passes **131,264 pairs / 1,050,112
scalar outputs** over four host FP control modes, ordinary/tiny terms, signed
zeros and each contributing axis. Direction.w is tested with arbitrary bits
where ignored. **1,280 exact generated-block cases** cover separate records,
both vectors overlapping the matrix, point/direction sharing the same record,
direction overlapping the final matrix row, and point overlapping another
matrix row. Whole private RAM, CPU records and terminal PC match; native VU
vectors/ACC/flags remain unchanged.

Fixture qualification matters: arbitrary direction.w payloads were initially
also written into aliased, active matrix/point components. Their NaN or enormous
values caused nonfinite/overflow differences in the fourth result component.
Those cases were outside the bounded numerical-input contract, not evidence of
store-order failure. Active shared numerical lanes are now finite and bounded;
opaque payload bits remain tested wherever genuinely ignored. The native game
algorithm does not reproduce PS2 NaN/overflow semantics. No runtime arithmetic
or hardware-state workaround was added to make those invalid fixtures pass.

An additional **1,024 real continuation cases** enter `0x1FB6BC` after its
virtual callback. Both dirty-flag skip and calculation paths cover the actual
integer transpose, cached-record copies, both transformed outputs, register
restores, return/branch PC and delay slot. Whole private RAM and CPU state match,
and native VU state is untouched. Entering the preceding callback in the offline
fixture throws; that callback is not fabricated or claimed tested here. The
earlier 1,536 full matrix/vector block and 2,560 continuation checks still pass.

Two 64-record live captures contain 26 distinct inputs in native-only mode and
24 in strict comparison, and replay exactly against the original operations.
Together with the synthetic cases this is **131,392 pairs / 1,051,136 scalar
outputs**, zero differences within the numerical contract above. Repeating the
converter changes no generated EE file. At Waiting Area capture 40, native
versus strict-comparison RGB mean absolute errors are 0.017 (central wall),
0.040 (upper-left wall), 0.114 (ground) and 0.005 (lower-left stone), on 0..255.
These are port regression checks, not new PCSX2 parity or FPS evidence.

Both hidden, input-isolated town/inn routes completed 300 seconds and logged
at least **53,248 paired transforms each**. Strict original-result comparison
found no mismatch. Music input remained playing with nonzero PCM peaks at
290 seconds in both runs, and user-card hashes were unchanged. This is live
route coverage, not whole-game RAM equivalence or coverage of every caller.

```powershell
build_usa/port/vp2s_math_check.exe --point-direction-synthetic
build_usa/port/vp2s_math_check.exe --point-direction-blocks
build_usa/port/vp2s_math_check.exe --point-direction-functions
build_usa/port/vp2s_math_check.exe <point-direction-records.bin>
```

## Collision point transform, 2026-11-06

The inline matrix/vector calculation at `0x1FBDCC..0x1FBDF0` in collision helper
`0x1FBD60` now uses the existing typed native four-weight transform. It reads
the matrix through v1, the point through a1 and writes the temporary through v0.
Earlier collision code can write a projected point through a2 before this block
reloads its input. The native bridge therefore reads current records at the
original result-store boundary, rather than retaining a function-entry snapshot.
It copies every input before writing the result, including when the temporary
overlaps an input record.

The converter validates the exact ten USA instruction words and lowers only
this audited block. An identical block at `0x1FBE6C` belongs to a different
helper and remains separate work. Earlier point/plane/projection VU operations
in `0x1FBD60` are also retained; the complete helper is **not** VU-free. The
replacement introduces no VU register/ACC/flag state, packet replay, VU memory
or timing dependency. Later scalar x/z bounds tests and return behavior remain
the original game code. Repeating conversion changes no generated EE source.

The existing `VP2S_NATIVE_MATRIX_VECTOR=0` diagnostic selection and
`VP2S_MATRIX_VECTOR_CHECK=1` strict comparison cover this additional site.
`VP2S_COLLISION_POINT_RECORD=<file>` provides a separate bounded capture for
this site so hotter object transforms do not consume its capture allowance.
The existing `VP2S_MATRIX_VECTOR_FAILURE_RECORD` saves a mismatching original
matrix/vector/result record; a mismatch aborts without fallback.

The shared arithmetic oracle remains exact on **131,264 cases / 525,056 scalar
outputs** within the previously documented numerical contract. **2,560 exact
original/generated block cases** cover separate output, output overlapping the
matrix at offsets zero and 16, output aliasing the point and point overlapping
the matrix. Whole private RAM, GPR/FPR/status, terminal PC and native VU-state
preservation match.

Another **2,560 complete original/native helper cases** compare whole private
RAM and CPU state across four host FP control modes, including earlier projected
output aliasing the input point or a matrix row, point/temporary stack aliasing,
and a point inside the matrix. They exercise **640 accepted / 1,920 rejected**
collisions and match return/branch PC and delay-slot completion. These complete
fixtures deliberately retain the earlier VU calculations in both variants;
VU-state preservation is asserted for the replaced isolated block, not claimed
for the whole collision helper. No callback or result is fabricated.

Two 64-record live captures each contain 64 distinct inputs and replay exactly
against the original operations. Together with the shared arithmetic cases this
gives **131,392 cases / 525,568 scalar outputs**, zero differences within the
documented numerical contract. At Waiting Area capture 40, native versus strict
comparison RGB mean absolute errors are 0.010 (central wall), 0.237 (ground) and
0.048 (unobstructed lower-left stone), on 0..255. The upper-left ROI is crossed
by the animated bird at different positions and gives 15.076; it is not a static
geometry comparison in those frames. These are port regression checks, not new
PCSX2 parity or FPS evidence.

Both hidden, input-isolated town/inn routes completed 300 seconds and logged
at least **12,288 calls at the new site each**, with strict comparisons exact.
Music remained playing with nonzero PCM peaks at 290 seconds in both modes,
and user-card hashes were unchanged. This qualifies the tested route, not
every collision scene or whole-game state equivalence.

```powershell
build_usa/port/vp2s_math_check.exe --collision-point-blocks
build_usa/port/vp2s_math_check.exe --collision-point-functions
build_usa/port/vp2s_math_check.exe <collision-point-records.bin>
```

## Still open

The Csi seed service still runs through the IOP environment. Other main/module
VU0 programs, macro operations, VIF0 setup and unrelated DMA remain separate
replacement work. The replacements above remove their documented calculations'
hardware dependencies; they do not remove the complete VU0 core.

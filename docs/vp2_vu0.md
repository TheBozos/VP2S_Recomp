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

## Still open

The Csi seed service still runs through the IOP environment. Other main/module
VU0 programs, macro operations, VIF0 setup and unrelated DMA remain separate
replacement work. The replacements above remove their documented calculations'
hardware dependencies; they do not remove the complete VU0 core.

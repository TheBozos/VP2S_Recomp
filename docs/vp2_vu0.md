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

## Still open

The Csi seed service still runs through the IOP environment. Other main/module
VU0 programs, macro operations, VIF0 setup and unrelated DMA remain separate
replacement work. This change removes the startup calculation's hardware
transport and execution dependency; it does not remove the complete VU0 core.

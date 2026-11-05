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

## Still open

The Csi seed service still runs through the IOP environment. Other main/module
VU0 programs, macro operations, VIF0 setup and unrelated DMA remain separate
replacement work. This change removes the startup calculation's hardware
transport and execution dependency; it does not remove the complete VU0 core.

# BozoEngine migration: initial native bring-up

Started 2026-11-06 after VU0 collision-point increment `d99b892`. The existing
PS2Recomp launcher remains the playable baseline. **The Bozo runner is not
playable: it currently produces zero frames.** It is an independent compiled
target, not a replacement DLL loaded into PS2Recomp.

## Build and evidence

Native Windows PowerShell, existing user-owned USA extraction and function/SDK
maps in `build_usa`, UCRT64 CMake/Ninja/G++, and native FFmpeg:

```powershell
python tools/vp2_bozo.py --stage build
python tools/vp2_bozo.py --stage run
```

The first command builds main + mod001 and runs the port tests. The second
currently returns failure at the audited InitAlarm boundary below. Neither opens
a window nor reads host input. No card directory is passed: cards remain fresh
in memory and user saves are not opened. All generated game bytes and logs stay
in ignored `build_usa/bozo` and `build_usa/bozo_port` directories.

The engine submodule is pinned to published `e5ddc96`, merging the isolated
offline-entry contribution in [engine PR7](https://github.com/TheBozos/BozoEngine/pull/7)
on top of compiler PR6 (`9d24c9d`). All Windows/Linux CI checks passed; the
merged source tree matches tested head `eda6885`. PR7's remote/local feature
branch was deleted. Graphics-services PR5 remains a separate unqualified candidate
here. No `--patches` instruction adaptations are used by this bring-up pipeline.
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
native kernel. Startup now reaches `InitAlarm` and rejects its timer-register
read at `0x10001810` (entry `0x141a38`). It still produces no frames.
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

1. Audit InitAlarm's native requirements and replace its SDK timer/firmware
   initialization using native alarm services; do not supply timer-register state.
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
The native boundary test covers 16 whole-context cases, full flat-memory equality,
preserved semaphore state, every missing readiness binding, all 75 rejecting
entries, and continued generic SetSyscall rejection. These are tests of the new
native ABI, not original firmware-state equivalence. The startup test explicitly
expects the next InitAlarm failure; it is not a gameplay qualification. It has no
window, host input, or persistent card directory.

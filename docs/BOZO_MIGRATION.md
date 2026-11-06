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
currently returns failure at the audited startup boundary below. Neither opens
a window nor reads host input. No card directory is passed: cards remain fresh
in memory and user saves are not opened. All generated game bytes and logs stay
in ignored `build_usa/bozo` and `build_usa/bozo_port` directories.

The engine submodule is based on published `149bf90`, plus the isolated explicit
offline-entry contribution `84cbdc3` on `codex/explicit-entry-points`
([engine draft PR7](https://github.com/TheBozos/BozoEngine/pull/7)). Graphics-services
PR5 and instruction-adaptation PR6 are separate, unqualified candidates here.
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
The independent engine headless/default-tools build passes all nine suites,
including the new explicit-entry CLI test. No new PCSX2 visual parity is claimed.

## Current boundaries

Startup stops at syscall 116 (`RFU116_SetSyscall`, instruction `0x141550`, wrapper
`0x14154c`, clock 7342664), before any library call or frame. This needs caller,
argument and subsequent-consumer analysis to establish a native service contract;
it must not become a blanket no-op or firmware patch emulator.
The captured arguments are index `0x83`, target `0x1413c0`, and caller return
address `0x141488`; these do not yet establish the registered routine's semantics.

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

1. Audit SetSyscall callers/arguments/consumers and implement the required native
   service or game-specific replacement with evidence, independently of rendering.
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

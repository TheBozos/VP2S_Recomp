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

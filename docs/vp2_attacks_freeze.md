# Attacks exit freeze (USA, 2026-11-04)

## Reproduction and cause

Open camp with SELECT, select Attacks, select Alicia, then cancel the attack list.
The user's running session and a separate native reproduction both stopped EE
thread 1 at unregistered return address `0x49BE34`. Read-only RAM inspection
confirmed the code at `0x49BCE0..0x49BE50` matched mod018's ELF.

The exported blocks split at `0x49BE30`, the delay slot of a JAL at `0x49BE2C`.
Its return address (`call + 8`) is inside the next standalone `entry_` block.
The analyzer kept only continuations inside the caller and rejected standalone
blocks as external owners. Thus neither the registration table nor the next
block's entry switch contained `0x49BE34`.

The recompiler now queues out-of-block call continuations as external entries.
Both analysis and ownership resolution allow standalone blocks, while preferring
a real function when it overlaps an entry wrapper. The generated next block
registers `0x49BE34` and enters directly at that label, skipping the call's delay
slot. This changes static C++ generation, with no runtime interpretation or
game-specific return-address shim.

## Verification

- A synthetic ELF regression covers both JAL and JALR at a block boundary, with
  a side-effecting delay slot in the next block. It checks registration and the
  owner's exact resume label, after that delay slot.
- The new regression fails on the original recompiler and passes after the fix.
  The full `ps2x_tests` run changes from 600/612 to 601/612. Its 11 existing
  failures are unchanged (one MPEG callback test and ten VU helper tests).
- The normal USA executable builds successfully. The fixed native run in
  `build_usa/attacks_probe/final2` enters Alicia's attack list twice and returns
  to camp after each exit (`native_156`, `166`, `177`, `186`). It then closes
  camp and moves in town. No missing-target error is logged. The earlier
  `after2` probe independently verified one attack-list exit and town movement.
- Ignored local evidence: `build_usa/attacks_freeze/manual.ee`, `manual.err`,
  `before.err`, `before/native_151.png`, `tests_before.log`, `tests_after.log` and
  `test_results.json`. The original and isolated failures have identical return
  targets and cleanup call traces.

Fork fix: `cbec364a` (`native/gs-backend`). All USA modules were regenerated;
mod018's fix changes its registration table and continuation block only.

## Repeatable native probe

```powershell
python tools/attacks_menu_probe.py --tag check1 --exe build_usa/port/vp2s.exe
```

The probe creates its own card copies, hard-links read-only disc files, disables
host input, mutes audio and places its window off-screen without activation. It
starts a new game, skips the opening, enters/exits Alicia's Attacks twice, closes
camp and moves right. Inspect captures in `build_usa/attacks_probe/check1/shots`;
an empty missing-target count alone is insufficient proof of navigation.
Each tag must be new. Card copies and evidence remain for inspection.

# Waiting Area lighting investigation, 2026-11-04

Rendering remains unchanged. Do not calibrate a global colour gain from the current screenshots.

## Evidence

- The first 60 entries of `VP2S_MODELS_MATLOG` previously belonged to title asset
  `0021551000_001`. They did **not** establish the town's lighting modes.
- The logger now records each object/asset/mesh once. `VP2S_MODELS_MATLOG_PART` filters
  by asset substring; `VP2S_MODELS_MATLOG_LIMIT` sets a 1..10000 limit (default 60).
- USA build passed. The existing `build/agent_tools/logun_usa.sh` route was run through
  **native Windows Git Bash**, `C:\Program Files\Git\bin\bash.exe`, with the guard wrapper.
  Log: `build/agent_tools/run_lighting_meshes.out`.
- Alicia's asset `00e405ef34_000` uses lighting mode 1, with some mode 3 materials.
  Town asset `006e569c20_000` contains many mode 0 materials and some mode 1 materials.
  Mode 0 results from the title log cannot be generalized to Alicia.
- PCSX2 slot 2 saved EE memory (`eeMemory.bin` inside its .p2s) identifies Alicia at
  `0x996040`: light colours 7 and 8 and directions 3 and 4 are zero. A different
  fourth-light clamp cannot explain Alicia's discrepancy in this state.
- That PCSX2 state's brightness object is `0x5EE420` (owner `0x5A1220`, +0x4444):
  mode 1, high `0xFFFFFF`, low 0, RGB gain `2.394957304`.
- Native exposure log `build/agent_tools/run_exposure.out` reports mode 1,
  high `0xFFFFFF`, low 0, gain 2.0. Its timed START sequence ended paused in the
  Waiting Area (`native_exposure/native_35.png`). This is not a matched exposure
  comparison: the camera/scene state must be matched before drawing conclusions.

## Measurement limits and next separating evidence

Earlier normalized screenshot regions had native road/Alicia mean luminance roughly
14..22 levels above the PCSX2 image, while sky was lower. Images were not registered
to the same frame, pose and exposure; this supports investigation, not a gain fix.

Next identify the specific ground mesh and compare its native vertex colours against
the PCSX2 dump's colours, with camera/exposure matched. For Alicia compare the live
light rows and skinned normal for a matched vertex. This separates asset colour,
lighting and post-processing differences without changing all three at once.

Runs use the existing no-host-input/no-focus launcher and snapshot/restore helpers.
All launched VP2S processes were stopped. Both restored USA memory-card directories
and their snapshots were empty. No WSL is needed for these scripts.

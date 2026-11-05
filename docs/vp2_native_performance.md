# Native presentation performance (2026-11-05)

The manual town pass reported an odd-looking Alicia face and low FPS. These were
investigated separately, using the USA game, native captures, the sampling profiler
and a private PCSX2 reference. No render interpretation or emulation was added.

## Native GPU texture cache

The direct native draw overload supplies a `TextureCache::Texture` with an immutable
asset key. Its materials generally leave TEX0/TEXA at zero. The GPU cache had been
indexed only by `material.tex0 ^ hash(material.texa)`, so unrelated native assets
all occupied one slot. Every material switch unloaded and uploaded that slot again.
This was visible as repeated Raylib texture loads, often reusing texture ID 10.

Application textures now occupy a separate GPU-cache namespace keyed by their own
`Texture::key`. The retained source pointer still detects replacement versions.
The existing GS-context cache lookup is unchanged. This is a native resource-cache
fix, independent of the game's geometry, texture conversion or timing.

## FPS overlay

VP2S's overlay was counting old latched GS textures/readback hashes even when the
whole picture came from the native renderer. It could report single-digit FPS
while native presentation statistics showed roughly 58 distinct pictures/s.

The native renderer now exposes `presentedFrameCount()`, incremented when it selects
a newer application frame for display. VP2S counts these presentation changes in
its normal native path. The comparison path retains its prior counter. Host redraws
of the same native frame do not increment it. No frame limiter, game clock, guest
timing or audio timing was changed.

## Verification and limits

Isolated Windows runs use `tools/attacks_menu_probe.py` with private memory cards,
no host input/focus, muted audio and an off-screen window. An opening-only route
leaves Alicia idle in the Waiting Area. `--capture-interval 10` reduces screenshot
frequency for the measurements. Enable `PS2X_NATIVE_STATS=1` to report host presents,
distinct native frames and longest repeated-picture intervals every five seconds.
The native counter is the relevant measurement; the old FPS overlay is not.

Local evidence:

- `build_usa/attacks_probe/face_perf_baseline`: 1 kHz host stack sampling. The field
  has the EE thread as its busiest sampled thread; the main thread also spends
  time uploading textures.
- `build_usa/attacks_probe/perf_frames_before` and `perf_frames_after`: same route,
  200 seconds each, native target 545x409, captures every ten seconds.
- `build_usa/face_perf/performance.json`, `render_comparison.png`: analysis output.

| Measurement | Before | After |
|---|---:|---:|
| Texture upload messages, whole run | 1,205,797 | 2,841 |
| Texture upload messages, warm field section 130–190 s | 413,640 | 0 |
| Mean distinct native frames/s, 13 field windows | 57.24 | 57.60 |
| Mean host presents/s in those windows | 59.19 | 59.26 |

The upload reduction is established. The small average FPS difference is **not**
an established speedup: the scene is near the host's 60 Hz presentation limit,
the user's manual game exited partway through the baseline, and background load
was not identical. Both runs' periodic PNG captures also cause approximately
120–140 ms hitches; those are capture overhead, not evidence of normal-play
frame pacing. Windows without captures generally have a longest interval near
33–35 ms. Do not claim that normal gameplay hitches were fixed from these runs.

Static stone crops have mean absolute RGB differences of 0.16 and 0.03 on a 0–255
scale between the captured before/after pictures. Moving NPCs, animations, shadows
and the ship area differ between the captures; they are not registered reference
frames for whole-image pixel parity. Both isolated runs have no missing guest target.

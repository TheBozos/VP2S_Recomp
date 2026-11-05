# Native streamed-music DSP (USA, 2026-11-05)

The default streamed-music decoder no longer processes VIF1 chains or executes
through a VU environment. `port/vp2s_music.cpp` takes the game's coefficient
records and per-channel synthesis history directly, calls three compiled game
algorithms, and writes history and PCM back to the EE buffers the sound service
already consumes. The IOP/Csd service that produces the coefficients and receives
the PCM is **still present**. This is not removal of all audio/runtime emulation.

## Offline compilation and tables

`tools/vp2_music.py` reads the fixed music upload block in the user's USA main
ELF **offline** and statically compiles the dequantization and synthesis algorithms
to typed C++ arrays. The window stage is expressed directly as its bounded matrix
operation. Numeric tables are converted to `native_music/tables.bin` (`VPMU`).
Neither generated code nor tables are committed. The port build invokes the tool
automatically; unchanged outputs retain their timestamps.

The generated decoder includes only `vp2s_music_math.h`: no VU context/register
file, code-image dispatch, memory-space adapter, instruction execution, packet
reader, pipeline, cycle counter or device callback. Its temporary SIMD variables
are compiled game code. The original algorithm's truncating sums and host
round-toward-zero arithmetic are retained as pure arithmetic; the host rounding
mode is restored after each channel decode. Ordinary round-to-nearest changed a
small number of PCM words by one level in the separating offline test.

The offline compiler rejects unmapped memory accesses and residual hardware
dependencies. It emits separate original lifted reference programs for the
**offline checker target only**. Those reference files are not linked into
`vp2s.exe`.

## Game interface

- Each channel occupies a 2 KiB coefficient record. The first four reserved
  bytes are ignored. The decoder reads four signed header values, 28 selectors
  (one scale selector and 27 coded bands), and 27 x 32 signed coefficients.
  Unused lookahead slots are native zero padding.
- A stereo frame produces two consecutive 1,024-sample signed-16-bit blocks,
  matching the original EE `PPACH` copy order. Each channel also updates 4 KiB of
  synthesis history. The algorithm's intermediate integer output is checked
  before the lossless signed-16-bit packing.
- The native hook intercepts music jobs at mod001 `0x201F20` (class `0x151D30`).
  It bypasses the hardware queue and computes one channel into host-owned buffers.
  Completion is posted through the game's ordinary worker-message sender
  (`0x1131C0`, message 5). The native worker callback at `0x105B80` commits PCM and
  history, then submits the next channel/frame. No DMA, interrupt or cycle event
  drives this path. Main `0x1060A0` is its direct producer continuation.
  The producer's counters, input/output half selection and ready flags are preserved.
- A native return continuation registered at unused main-ELF padding `0x106038`
  restores the caller after message submission, including cooperative EE yields.
  This is a compiled ABI bridge; no MIPS instructions execute at that address.
- History comes from player `+0xF0/+0xF4`; input from `+0x1C`; PCM destination
  from job `+8`. Optional analysis rings are copied with the original room/wrap
  rules. The existing one-packet-in-flight SIF acknowledgement fix is unchanged.
- The producer's one-byte frame-marker cache (`+0xC8` -> `+0xCC`) is preserved.
  These markers later populate the SIF PCM header; they are game metadata, not
  VIF commands. The consumed marker byte is cleared without writing a VIF word.

The three exceptions in `keptVif1Chain()` are disabled by default. The explicit
`VP2S_NATIVE_MUSIC=0` comparison switch retains the old path; failures in native
decoding never select it automatically. Missing tables, missing hook targets and
invalid native music buffers/counts fail explicitly.

## Validation

The initial offline fixtures cover 28 stage results and nine complete decodes:
**28,672 intermediate/output words, zero differences**, including exact PCM and
updated history through the complete three-stage native pipeline. The fixtures
come from the existing game recordings; this is a numeric algorithm comparison,
not a new PCSX2 listening comparison.

The first isolated live run processed thousands of stereo frames. Its first
**1,024 channel blocks / 1,048,576 PCM samples** matched the original algorithms
bit-for-bit, with zero PCM or history block differences, including startup
history initialization. Native mixer input was playing with raw peak 5,002, also
observed in the baseline. The user-card hashes were unchanged.

The initial town/inn replay exposed a dequantizer lookup at index 512. Capturing
the typed failing input separated it from buffer corruption: valid maximum
magnitude 511/512 makes the original kernel prefetch the next lookup entry even
though its interpolation weight is zero. The native kernel now supplies the
mathematical closed endpoint (2.0), without implementing a VU address wrap. The
captured failing block then matches the original PCM and history byte-for-byte.
The marker-cache omission found during the same audit was fixed separately.

Final numerical checks cover five 1,024-block live captures, 180 synthetic cases
and the captured lookup-endpoint failure: **5,428,224 PCM samples**, with zero
native/original PCM differences, zero history differences and zero captured-output
differences. These include a late interval beginning after 14,000 channel decodes.

The integration preserves asynchronous per-channel completion: the caller performs
bookkeeping after submission. Numerical checks alone do not validate this contract.
Late diagnostic recording also exposed a separate timing-sensitive stream-end
failure around 210 seconds. Moving from inline to queued completion did not remove
it. Both a capture-free run and an outgoing-buffer capture run continued playing
past that point. The typed recorder had been flushing about 16 KiB per channel on
the game thread. It now reserves bounded host storage before the game starts and
writes the completed capture on a host worker, without per-channel disk flushes.
The same late-recording replay then completed 300 seconds with audio still
playing at 290 seconds (973 mixer input halves, raw peak 15,563), matching the
baseline's continuing feed. A separate capture-free 300-second replay also
passed. Both used hidden/offscreen windows, scripted input with host devices
disabled and private copied memory cards; the user's card hashes were unchanged.

To capture typed inputs/outputs, set `VP2S_NATIVE_MUSIC_RECORD=<file>`; at most
1,024 channel blocks are written. `VP2S_NATIVE_MUSIC_RECORD_SKIP=<channel blocks>`
selects a later interval. Records contain coefficients, prior history, output
PCM words and updated history; no hardware state or packets. A complete capture
is persisted after block 1,024. An orderly shutdown also saves a partial capture;
force-killing the process before block 1,024 does not.

```powershell
# Build the optional offline checker after the normal port build:
cmake --build build_usa/port --target vp2s_music_check
build_usa/port/vp2s_music_check.exe build_usa/native_music/tables.bin <records.bin>
build_usa/port/vp2s_music_check.exe build_usa/native_music/tables.bin --synthetic
```

The checker compares both newly computed native output and captured live output
against the original algorithms. Synthetic inputs exercise zero bands, legal
quantizer widths and signed extrema, including the closed lookup endpoint.
An optional third path writes original-algorithm records for a captured edge
case; those game-derived records stay in the build tree. Native failures can be
captured with `VP2S_NATIVE_MUSIC_FAILURE_RECORD=<file>`; this never enables a
fallback.

`VP2S_MUSIC_PACKET_RECORD=<file>` optionally records outgoing game buffers
(64-byte header plus 4 KiB of PCM) for offline transport comparison. It is a
diagnostic hook, not input to the decoder. In the first 2,200 aligned outgoing
stereo buffers, 1,044 of 4,505,600 samples differ from the old running runtime
path, each by exactly one signed-16-bit level. Many buffers match byte-for-byte.
This live old-path comparison is distinct from the exact checks against the
offline original algorithms under the retained truncating arithmetic policy.
It is not evidence of a new PCSX2 listening or hardware-audio comparison.

Local evidence is in `build_usa/native_music/` and
`build_usa/native_mesh_offsets/music_final/` and `music_no_capture/` (never committed).

## Still open

The tri-Ace IOP services, controller/device environment, VIF0/VU0 core and other
hardware transfers remain separate native-replacement tasks. Removing music's
transport dependency does not make the complete VP2S runtime hardware-free.

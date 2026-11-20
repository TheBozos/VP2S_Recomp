#pragma once
#include "bozo/runtime.h"
#include "bozo/render/renderer.h"
#include <array>
#include <cstdint>
#include <optional>
#include <vector>
namespace vp2s {
// Battle-entry capture (mod001 class 15a850, built by the encounter transition
// 15d5d0's update 375870 after event-script op 0x38). Its build 3094d0 reads the
// frame back into +100 slices of 64x416 PSMCT16 (readback group 15a7a0, +a4);
// its update 308c60 rotates the 2N+1 column boundaries about the screen centre
// (atan2/sin/cos; +b0 twist, +b8 scale) into +a0 (16 bytes per boundary: x, y
// on top, x, y at the bottom, 12.4 as floats); 3078d0 writes the slices and a
// textured strip per half slice; its job (+90, vtable 15a8cc) goes on the VIF1
// queue *(35dde0) through 308c30 and sends that chain to the GS (kick 308bd0).
//
// Natively build state 1 claims a kept engine image (the capture fade's slots)
// for the object and its group; the group's submit 307170, where the PS2 reads
// the frame back (its scene slot, after the field's draws), copies the frame
// drawn so far into it; the job's enqueue draws the strips from that image, and
// the patched call runs the job's own completion 308c10 (scene completion 21c9a0).
struct NativeEntryCaptureDraw {
    uint32_t object=0,colour=0;
    uint8_t image=0;
    bool blend=false;          // PRIM ABE: +c8 non-zero
    int32_t frameWidth=0,frameHeight=0;
    std::vector<bozo::render::Vertex> vertices;   // triangles
    // The grid pass after the strips: the displayed frame (TBP = display FBP), 10 x 7
    // CT24 tiles of 64 x 32 over the whole screen, i.e. the previous presentation
    // redrawn 1:1 (a trail). RGB 0x88 modulate; alpha (TEXA 0x60 * vertex alpha) >> 7;
    // ALPHA 0x44; TEX1 nearest. Empty while its alpha is 0.
    std::vector<bozo::render::Vertex> grid;
};
// The strips 3078d0 writes, from the object's fields (no packet is read):
// half h of slice k spans boundaries c = 2k + h and c + 1; it is skipped when
// any x or y exceeds 65536, or both x (or both y) of either boundary are <= 0.
// UV: U = (k*64 + h*32) + 0.75 .. + 31.25, V = c4 .. 416 - c4 (c4 = +c4 as
// int); XY = the boundary floats truncated (12.4); Z = *(35df00) - 1 - +e0.
// Colour 0x7f7f7f with alpha 0x80, or 128 * (1 - min(+c8, +d0) / +d0) while
// +c8 > 0. Grid: vertex alpha cvt(160 * +c0) (+f9 selects a zoomed 0x848484
// variant, unported).
// UVs are in the snapshot's size (the frame at the capture: 512 x 512 in the field,
// while the battle-entry frame it is drawn into is 576 x 416); texels past the
// snapshot clamp to its edge.
NativeEntryCaptureDraw nativeEntryCaptureDraw(Runtime *,uint32_t object,int32_t snapshotWidth,int32_t snapshotHeight);
class EntryCaptureOwner {
public:
    virtual ~EntryCaptureOwner()=default;
    // The capture fade's snapshot slots (images 9..15): claimed at build state 1 for
    // the object and its readback group, copied at the group's submit.
    virtual uint8_t claimNativeEntryFrame(uint32_t object,uint32_t group)=0;
    virtual void copyNativeEntryFrame(uint32_t group)=0;
    virtual std::optional<uint8_t> nativeEntryFrame(uint32_t object)=0;
    virtual std::array<int32_t,2> nativeEntryFrameSize(uint32_t object)=0;
    virtual void drawNativeEntryCapture(const NativeEntryCaptureDraw &)=0;
};
// Build state 1, after the readback group is queued (3096bc): the claim. The
// group's submit 307170 (site 307188, before its patched completion call): the copy.
// The enqueue 308c30 before its (patched) call 308c4c (site 308c48): the native draw; the patch
// calls the job's completion 308c10(object) instead of the VIF1 queue 201f20.
std::optional<bool> nativeEntryCaptureInstruction(R5900Context *,Runtime *,uint32_t pc);
// The slices' readback nodes (class 159010, owned by the group: +65) reach the
// main graphics queue's readback entry, which enqueues them with 29a460 for the
// readback thread (GS CSR, VIF1 FIFO). A recorded colour request (PSMCT16, 64 x
// 416 from the 576-wide frame) is served by the snapshot instead: the node runs
// its own notify 299c00 (slot flag, scene completion) at once; its EE buffer is
// not written (its only reader, the slice upload, is replaced). Any other node
// runs the original 29a460.
void installNativeEntryCapture(Runtime &);
}

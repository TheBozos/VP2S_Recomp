#pragma once
#include "bozo/runtime.h"
#include <array>
#include <cstdint>
#include <optional>
namespace vp2s {
// Occlusion culling (mod001). The scene pass 2200d0 asks 222850 for the occluders in front of its camera: for each
// occluder object (at most 26) the leaf 222b10 runs on VU0 with the clip rows camera +3f0 in vf24..vf27 (loaded at
// 22293c) and writes one 9-qword record into VU0 data memory at vi1 (8 + 9k):
// - p0..p3: the four corners (object +220..+250) through the world rows +90, as program 0x1f0's transpose leaves them
//   (diagonal moved raw, W row 1 x, the rest 0 + x): p = ((T0 c.x + T1 c.y) + T2 c.z) + T3;
// - n0..n4: unit normals of the planes through the camera position e and each edge, n_k = (p_k+1 - p_k) x (p_k - e),
//   and of the face, n4 = (p0 - p1) x (p2 - p0); each scaled by RSQRT(|n|^2), negated when the clip-space face normal
//   (c1 - c0) x (c2 - c0) has z > 0 (then a single-sided occluder, +21c set, is rejected before its normals are stored).
// The leaf also stores to its stack the clip corners c_k = ((R0 p.x + R1 p.y) + R2 p.z) + R3, the face normal, and per
// corner (c.x Q, c.y Q, c.z, c.w) with Q = 1 / c.z, which the compiled code turns into outcodes.
// Then for each object sphere s (centre, radius w) and each occluder no nearer than it, program 0xd58 computes the five
// distances (s - p_k) . n_k (k < 4) and (s - p0) . n4 and reports (VI2, FSAND of the sticky sign) whether any distance
// minus the radius is negative; when none is, the object is occluded (+6a |= 0x10).
// All arithmetic is VU0's, on PS2 float words; sums in the listings' order.
using OcclusionWords=std::array<uint32_t,4>;
struct NativeOccluder {
    std::array<OcclusionWords,4> corners{},clip{};
    std::array<OcclusionWords,5> normals{};   // unit, before the facing negation
    OcclusionWords face{};                     // (c1 - c0) x (c2 - c0), xyz (w kept by the caller)
};
// world: rows +90..+c0; local: the corners +220..+250; eye: the camera position; rows: the clip rows vf24..vf27.
NativeOccluder nativeOccluder(const std::array<OcclusionWords,4> &world,const std::array<OcclusionWords,4> &local,const OcclusionWords &eye,
                              const std::array<OcclusionWords,4> &rows);
// The per-corner projection the leaf stores: (c.x Q, c.y Q, c.z, c.w), Q = DIV(1, c.z).
OcclusionWords nativeOccluderProjection(const OcclusionWords &clip);
// Program 0xd58 on one record (p0..p3, n0..n4 as stored): true when the sphere is occluded (no sign set).
bool nativeOccluded(const OcclusionWords &sphere,const std::array<OcclusionWords,9> &record);
std::optional<bool> nativeOcclusionInstruction(R5900Context *,Runtime *,uint32_t pc);
// An occlusion pass is open from the clip-row loads (22293c) to 220294: a save waits for it.
bool nativeOcclusionOpen(const Runtime *);
}

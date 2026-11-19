#pragma once
#include "native_scene.h"
#include "bozo/render/renderer.h"
#include <array>
#include <vector>
namespace vp2s {
// The Lost Forest haze (class 15a020, built by mod001 2dfeb8..2e31f8). The game
// draws it as GS passes over 64x128 tiles: the depth buffer through its 256-entry
// exponential ramp (+134c4), times a per-vertex scattering colour on a 32-pixel
// grid (VU1), added to the scene scaled by the ramp's alpha. Natively: one
// depth-colour pass over the frame with that grid, from the game's records.
struct NativeHaze {
    uint32_t object=0;
    std::shared_ptr<const NativeFrameTarget> frame;
    // 64 RGBA entries indexed by depth >> 14 (R, G, B /255, alpha /128).
    std::array<float,256> colours{};
    // Screen triangles (GS coordinates of the native frame), Gouraud scattering colour.
    std::vector<bozo::render::Vertex> vertices;
};
// The ramp index the GS packet builds from a 24-bit depth (blue = Z >> 16 doubled
// five times with saturation, plus 0x10 * dest alpha / 128 where alpha holds Z
// bits 15 and 14 when bit 15 is set): a pure function of Z >> 14.
unsigned hazeRampIndex(uint32_t z);
// The VU1 program's vertex colour for a grid point (configuration: flags 8.x = 0,
// 11.y = 0, 11.z = 1), from the 25-qword constant block. Returns R | G << 8 | B << 16 | 0x80 << 24.
uint32_t hazeVertexColour(const std::array<std::array<float,4>,25> &constants,const std::array<float,4> &point);
NativeHaze nativeHaze(Runtime *,const NativeScene &,uint32_t object);
void drawHaze(const NativeHaze &);
class HazeOwner {
public:
    virtual ~HazeOwner()=default;
    virtual void drawNativeHaze(const NativeHaze &)=0;
};
std::optional<bool> nativeHazeInstruction(R5900Context *,Runtime *,uint32_t pc);
}

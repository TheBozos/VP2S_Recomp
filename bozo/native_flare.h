#pragma once
#include "bozo/runtime.h"
#include "bozo/render/renderer.h"
#include "native_trig.h"
#include <array>
#include <map>
#include <optional>
#include <vector>
namespace vp2s {
// Lens flare (mod001 class 155b50; its build 255b60 runs each frame, its draw 255b30 enqueues the job whose start
// 255ad0 kicks the built GIF packet on VIF1). The packet sets ALPHA_1 0x48 (Cs As + Cd), TEST_1 0x30000 (depth
// test always) and ZBUF_1 with ZMSK, then:
// - when +23e and the light's intensity f21 > 0 (256404): the flash, screen-width / 64 SPRITEs of 64 x height pixels
//   from (0x800 - width / 2, 0x800 - height / 2), colour +230 with alpha FTOI(128 f21) (recorded at 256408);
// - per element of the list +21c, its +c with a0 = element, a2 = the owner's colour +230, f12 / f13 = the light's
//   window position, f14 = scale, f15 = angle offset, f16 = intensity, f17 / f18 = x / y stretch:
//   2550d0 (disc: a fan and two ring strips), 2549a0 (four ring strips), 253770 (rays of triangles).
// Natively the build's element calls and flash record this geometry (in the writers' EE float order, ps2f), the
// enqueue (255b4c, native site its delay slot 255b50) becomes the job's completion 255b10 (vp2_bozo_kernel.py) and draws the recorded triangles.
// The packet is never decoded.
struct NativeFlareCall {
    uint32_t colour=0;                      // a2: the owner's +230 (RGB)
    std::array<uint32_t,7> f{};             // f12..f18 (PS2 words)
};
// The writers' geometry: triangles in GS window coordinates (pixels; XYZ2 12.4 / 16), RGBA as the packet's RGBAQ.
// element: the element's first 0x48 bytes; rays: 253770's ray table (+44, 0x44 bytes per ray) or nullptr.
std::vector<bozo::render::Vertex> nativeFlareDisc(const uint8_t *element,const NativeFlareCall &,const TrigCoefficients &);
std::vector<bozo::render::Vertex> nativeFlareRings(const uint8_t *element,const NativeFlareCall &,const TrigCoefficients &);
std::vector<bozo::render::Vertex> nativeFlareRays(const uint8_t *element,const uint8_t *rays,const NativeFlareCall &,const TrigCoefficients &);
std::vector<bozo::render::Vertex> nativeFlareFlash(uint32_t colour,uint32_t intensity,int16_t width,int16_t height);
// The game's colour scale: (c * o * 0x80808081 >> 32) >> 7, at most 255.
uint32_t nativeFlareChannel(uint32_t c,uint32_t o);
class NativeFlares {
    std::map<uint32_t,std::vector<bozo::render::Vertex>> builds;   // per owner, since its build began
public:
    void begin(uint32_t owner){builds[owner].clear();}
    void add(uint32_t owner,const std::vector<bozo::render::Vertex> &);
    std::vector<bozo::render::Vertex> take(uint32_t owner);
};
class FlareOwner {
public:
    virtual ~FlareOwner()=default;
    virtual NativeFlares &nativeFlares()=0;
    virtual void drawNativeFlare(const std::vector<bozo::render::Vertex> &)=0;
};
std::optional<bool> nativeFlareInstruction(R5900Context *,Runtime *,uint32_t pc);
}

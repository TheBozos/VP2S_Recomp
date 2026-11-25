#pragma once
#include "native_scene.h"
#include "native_trig.h"
#include "bozo/render/renderer.h"
namespace vp2s {
struct NativeScreenRect {
    uint32_t object=0,colour=0,depth=0;
    int32_t x=0,y=0,right=0,bottom=0;
    int32_t sourceWidth=0,sourceHeight=0;
    // The ALPHA_2 the builder sets (3d37cc: the table at 3140e0 by +f4; 0x44 normal, 0x148 additive).
    uint64_t alpha=0x44;
    // Fade modes 4 (+f3 = 4, a slanted gradient wipe, 12 vertices) and 1 (a
    // two-colour gradient quad, 4 vertices): drawn as this Gouraud strip.
    bool wipe=false;uint8_t stripVertices=12;
    std::array<bozo::render::Vertex,12> strip{};
    // Fade mode 2 (+f3 = 2, a circle): PRIM 24d, a Gouraud fan of the centre and 17 ring vertices.
    bool fan=false;std::array<bozo::render::Vertex,18> fanVertices{};
};
// Fade 161230 mode 2, builder 3d4178..3d44a4: vertex 0 is the centre (+c8, +cc, floats) in colour `inner`; ring
// vertex k (1..17) at angle a = (k-1) x 0.3927 (accumulated with add.s) has
// x = c8 + float(trunc(cos(a) (0.9375 float(+b8) + |c8|)) + 0x800), y = cc + float(trunc(-sin(a) (0.9375 float(+bc) + |cc|)) + 0x800),
// colour `outer`; 214210 / 2143e0 are the full cosine / sine. XYZ2 keeps trunc(16 v) (centre: (trunc(c) + 0x800) << 4).
struct NativeFadeCircleInputs {uint32_t centreX=0,centreY=0;int32_t width=0,height=0;uint32_t inner=0,outer=0,depth=0;};
std::array<bozo::render::Vertex,18> nativeFadeCircle(const NativeFadeCircleInputs &,const TrigCoefficients &);
std::array<bozo::render::Vertex,6> nativeScreenRectangleVertices(const NativeScreenRect &);
// Fade 161230 mode 4, builder 3d3ad0..3d4170: progress p = +a4 / +a8 moves a band
// of 6 edge pairs across the screen (+9c's alpha byte 0x80 moves it left from
// the right edge, otherwise right from the left); edges 115 apart, the last pair
// 2w / w further, top edges offset by the height, all clamped to [0, 4095]. Its
// 5 ramp pairs take colour `from`'s RGB with alpha trunc(ramp[i] * (a_to - a_from)),
// the last pair `to`. Each pair is (x_bottom, bottom), (x_top, top) in GS pixels
// (x truncated to 1/16), z = depth.
struct NativeFadeWipeInputs {uint32_t from=0,to=0,time=0,duration=0;int32_t width=0,height=0;uint32_t depth=0;std::array<uint32_t,5> ramp{};};
std::array<bozo::render::Vertex,12> nativeFadeWipe(const NativeFadeWipeInputs &);
class NativeScreenRects {
    struct Lease {NativeScreenRect rect;uint32_t writer,begin,end;bool retired=false,flushed=false;};
    std::map<uint32_t,Lease> calls;
    std::map<uint32_t,NativeScreenRect> fades;
    std::map<uint32_t,uint32_t> skipped; // stack -> fade object whose builder exited before writing
public:
    NativeScreenRect begin(Runtime *,uint32_t stack,uint32_t object);
    void captureFade(Runtime *,const R5900Context *);
    NativeScreenRect beginFade(Runtime *,uint32_t stack,uint32_t object);
    // The builder 3d3660 left the fade unbuilt (+63 = 0: no writer, skip, pause
    // gate or hidden in +f5): its submit flushes an untouched (empty) writer,
    // which 120eb0 leaves alone, then completes. True if this is such a fade.
    bool skipFade(Runtime *,uint32_t stack,uint32_t object);
    void releaseFade(uint32_t object);
    void cancel(uint32_t stack){calls.erase(stack);}
    bool active(uint32_t stack)const{return calls.contains(stack);}
    void retire(Runtime *,R5900Context *);
    void flushed(Runtime *,uint32_t stack);
    uint32_t complete(uint32_t stack);
    size_t skippedFades()const{return skipped.size();}
    size_t outstanding()const{return calls.size();}
};
class ScreenRectOwner {
public:
    virtual ~ScreenRectOwner()=default;
    virtual NativeScreenRects &nativeScreenRects()=0;
    virtual void drawNativeScreenRect(const NativeScreenRect &)=0;
};
std::optional<bool> nativeScreenRectInstruction(R5900Context *,Runtime *,uint32_t pc);
}

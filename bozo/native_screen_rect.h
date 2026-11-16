#pragma once
#include "native_scene.h"
#include "bozo/render/renderer.h"
namespace vp2s {
struct NativeScreenRect {
    uint32_t object=0,colour=0,depth=0;
    int32_t x=0,y=0,right=0,bottom=0;
    int32_t sourceWidth=0,sourceHeight=0;
    // Fade mode 4 (+f3 = 4): a slanted gradient wipe, drawn as this Gouraud strip.
    bool wipe=false;
    std::array<bozo::render::Vertex,12> strip{};
};
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

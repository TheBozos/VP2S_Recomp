#pragma once
#include "native_scene.h"
#include "bozo/render/renderer.h"
#include <map>
#include <optional>
#include <vector>
namespace vp2s {
// mod003 class 15d420 (draw 374be0, builder 374c20; sites 374c04 and 374c10): the
// last displayed image (the 640-wide display buffer at FRAME's FBP), 64x32 page
// by page, drawn into the current target (512x512 for event cameras) zoomed about the centre
// (flag c0 bit 0, scale obj+a4) or shifted (obj+b0/b4), alpha-blended with
// texel alpha TEXA 0x60 times (1 - obj+98) * 160, nearest sampling, depth
// test always and depth 0xfffe06 written (ZBUF_2 loaded without ZMSK).
struct NativeScreenZoomStrip {float x0,y0,x1,y1;uint16_t column,row;};
struct NativeScreenZoom {
    uint32_t object=0;
    int32_t width=0,height=0;   // draw target (display globals +3c6c/+3c6e)
    int32_t sourceWidth=0,sourceHeight=0;   // the display buffer the pages come from
    uint8_t alpha=0;            // GS alpha after TEXA modulation
    std::vector<NativeScreenZoomStrip> strips;
};
struct NativeScreenZoomInputs {
    uint32_t visibility=0,zoom=0,shiftX=0,shiftY=0;   // obj+98, a4, b0, b4 (PS2 words)
    bool zoomed=false,fields=false;                   // obj+c0 bit 0, byte 1d5984
    int16_t width=0,height=0,baseWidth=0,baseHeight=0; // globals +3c6c, +3c6e, +3c68, +3c6a
    uint16_t sourcePages=10;                          // FRAME FBW of globals +670
};
// The builder's geometry in its own PS2 float order (positions are its 12.4
// fixed-point words divided by 16).
NativeScreenZoom nativeScreenZoomGeometry(const NativeScreenZoomInputs &);
std::array<bozo::render::Vertex,6> nativeScreenZoomVertices(const NativeScreenZoom &,const NativeScreenZoomStrip &);
class NativeScreenZooms {
    struct Lease {NativeScreenZoom zoom;uint32_t writer,begin,end;bool retired=false,flushed=false;};
    std::map<uint32_t,Lease> calls;
public:
    std::optional<NativeScreenZoom> begin(Runtime *,uint32_t stack,uint32_t object);
    void cancel(uint32_t stack){calls.erase(stack);}
    bool active(uint32_t stack)const{return calls.contains(stack);}
    void retire(Runtime *,R5900Context *);
    void flushed(Runtime *,uint32_t stack);
    void complete(uint32_t stack);
    size_t outstanding()const{return calls.size();}
};
class ScreenZoomOwner {
public:
    virtual ~ScreenZoomOwner()=default;
    virtual NativeScreenZooms &nativeScreenZooms()=0;
    virtual void drawNativeScreenZoom(const NativeScreenZoom &)=0;
};
std::optional<bool> nativeScreenZoomInstruction(R5900Context *,Runtime *,uint32_t pc);
}

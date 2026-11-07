#pragma once
#include "native_scene.h"
#include "bozo/render/renderer.h"
namespace vp2s {
struct NativeScreenRect {
    uint32_t object=0,colour=0,depth=0;
    int32_t x=0,y=0,right=0,bottom=0;
    int32_t sourceWidth=0,sourceHeight=0;
};
std::array<bozo::render::Vertex,6> nativeScreenRectangleVertices(const NativeScreenRect &);
class NativeScreenRects {
    struct Lease {NativeScreenRect rect;uint32_t writer,begin,end;bool retired=false,flushed=false;};
    std::map<uint32_t,Lease> calls;
public:
    NativeScreenRect begin(Runtime *,uint32_t stack,uint32_t object);
    void cancel(uint32_t stack){calls.erase(stack);}
    bool active(uint32_t stack)const{return calls.contains(stack);}
    void retire(Runtime *,R5900Context *);
    void flushed(Runtime *,uint32_t stack);
    uint32_t complete(uint32_t stack);
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

// Field map panel 167200 as native triangles (see native_map_panel.h).
#include "native_map_panel.h"
#include "bozo/ee/dispatcher.h"
#include <bit>
#include <cmath>
#include <cstring>
#include <format>
namespace vp2s {
namespace {
template<class T>T value(Runtime *rt,uint32_t at){
    if(!rt || at<0x100000 || uint64_t(at)+sizeof(T)>0x2000000)throw std::runtime_error("native map panel record outside EE RAM");
    T v;std::memcpy(&v,rt->memory.hostPointer(at,sizeof(T)),sizeof(T));return v;
}
// cvt.w.s: truncation toward zero; the builder's 2^31 branch keeps the low byte the same.
uint32_t truncated(float v){
    if(!std::isfinite(v) || v<0 || v>=4294967296.f)throw std::runtime_error("unqualified native map panel value");
    return v>=2147483648.f?uint32_t(int64_t(v)):uint32_t(int32_t(v));
}
}
NativeMapPanel nativeMapPanel(Runtime *rt,uint32_t object){
    if(object<0x100000 || value<uint32_t>(rt,object)!=0x167200 || value<uint32_t>(rt,object+0x90)!=0x16727c)
        throw std::runtime_error("native map panel escaped its class");
    NativeMapPanel panel;panel.object=object;
    // 4541d0: opacity +b0; zero draws nothing (+63 stays clear).
    const float opacity=value<float>(rt,object+0xb0);
    if(opacity==0.f)return panel;
    if(!value<uint8_t>(rt,object+0x63))throw std::runtime_error("native map panel enqueued without its build");
    const uint32_t alpha=truncated(opacity)&0xff;
    uint32_t border;{volatile float scaled=0.75f*opacity;border=truncated(scaled)&0xff;}
    // Alpha above 0x80 (GS 1.0) has no native blend form.
    if(alpha>0x80 || border>0x80)throw std::runtime_error(std::format("unported native map panel alpha {:x}/{:x}",alpha,border));
    // Rectangle: 24..180 x 24..140, grown by +b4 when (+a4)->+1683 bit 2 is set,
    // then offset by +94 / +98 (the frame's GS origin).
    int32_t x0=24,y0=24,x1=180,y1=140;
    const auto owner=value<uint32_t>(rt,object+0xa4);
    if((value<uint8_t>(rt,owner+0x1683)>>2)&1){
        const float t=value<float>(rt,object+0xb4);if(!std::isfinite(t))throw std::runtime_error("unqualified native map panel growth");
        volatile float a=-24.f*t;a=24.f+a;volatile float b=332.f*t;b=180.f+b;volatile float c=std::bit_cast<float>(0x43ba7ae0u)*t;c=140.f+c;
        for(float v:{float(a),float(b),float(c)})if(!(std::abs(v)<1e6f))throw std::runtime_error("unqualified native map panel growth");
        x0=y0=int32_t(float(a));x1=int32_t(float(b));y1=int32_t(float(c));
    }
    const auto ox=value<int32_t>(rt,object+0x94),oy=value<int32_t>(rt,object+0x98);
    if(ox<0 || ox>4095 || oy<0 || oy>4095)throw std::runtime_error("unqualified native map panel origin");
    x0+=ox;x1+=ox;y0+=oy;y1+=oy;
    if(x1<x0 || y1<y0)throw std::runtime_error("native map panel rectangle reversed");
    const auto vertex=[](int32_t x,int32_t y,uint32_t rgb,uint32_t a){bozo::render::Vertex v;v.x=float(x);v.y=float(y);v.rgba=rgb|a<<24;return v;};
    const auto strip=[&](const bozo::render::Vertex (&q)[4]){for(unsigned i:{0u,1u,2u,1u,3u,2u})panel.vertices.push_back(q[i]);};
    // Gradient: R b7 G 81 B 4d at the top, R 7a G 54 B 26 at the bottom.
    const uint32_t top=0x4d81b7,bottom=0x26547a,dark=0x101028;
    strip({vertex(x0,y0,top,alpha),vertex(x1,y0,top,alpha),vertex(x0,y1,bottom,alpha),vertex(x1,y1,bottom,alpha)});
    // Border: four strips between the rectangle and the one 2 pixels outside it.
    strip({vertex(x0-2,y0-2,dark,border),vertex(x0,y0,dark,border),vertex(x1+2,y0-2,dark,border),vertex(x1,y0,dark,border)});
    strip({vertex(x0-2,y0-2,dark,border),vertex(x0,y0,dark,border),vertex(x0-2,y1+2,dark,border),vertex(x0,y1,dark,border)});
    strip({vertex(x0-2,y1+2,dark,border),vertex(x0,y1,dark,border),vertex(x1+2,y1+2,dark,border),vertex(x1,y1,dark,border)});
    strip({vertex(x1+2,y0-2,dark,border),vertex(x1,y0,dark,border),vertex(x1+2,y1+2,dark,border),vertex(x1,y1,dark,border)});
    return panel;
}
void drawMapPanel(const NativeMapPanel &panel){
    if(panel.vertices.empty())return;
    using Material=bozo::render::Material;
    Material m;m.textured=false;m.directColour=true;m.depthTest=false;m.depthWrite=false;m.blend=Material::Blend::Alpha;
    const float id[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    bozo::render::Renderer::instance().draw(m,id,panel.vertices.data(),uint32_t(panel.vertices.size()));
}
std::optional<bool> nativeMapPanelInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x4541b8)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<MapPanelOwner *>(&d->host()):nullptr;
    if(!c || !owner)throw std::runtime_error("native map panel requires its renderer owner");
    // The enqueue 454190 (a1 = a0 + 0x90): its call becomes the completion 454170.
    if(c->pc!=0x454190 || c->r[5].uw[0]<0x100090 || c->r[4].uw[0]!=c->r[5].uw[0]-0x90)
        throw std::runtime_error("native map panel requires its original pre-enqueue producer");
    const auto object=c->r[4].uw[0];owner->drawNativeMapPanel(nativeMapPanel(rt,object));
    c->r[4].sd[0]=int32_t(object);return true;
}
}

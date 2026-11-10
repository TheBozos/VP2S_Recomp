#include "native_screen_zoom.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/ops_fpu.h"
#include <bit>
#include <cstring>
#include <format>
#include <stdexcept>
namespace vp2s {
namespace {
template<class T>T value(Runtime *rt,uint32_t at){
    if(!rt || at<0x100000 || at>0x2000000-sizeof(T))throw std::runtime_error("native screen zoom record outside EE RAM");
    T v;std::memcpy(&v,rt->memory.hostPointer(at,sizeof(T)),sizeof(T));return v;
}
// The builder's EE FPU (engine ps2f): add/sub truncate, mul/div round to nearest.
uint32_t add(uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::add(a,b,f);}
uint32_t sub(uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::sub(a,b,f);}
uint32_t mul(uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);}
uint32_t divide(uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::div(a,b,f);}
constexpr uint32_t kOne=0x3f800000u,kHalf=0x3f000000u,kSixteen=0x41800000u,kThirtyTwo=0x42000000u,kSixtyFour=0x42800000u,
    k160=0x43200000u,k224=0x43600000u,k640=0x44200000u,kTwo31=0x4f000000u;
constexpr uint32_t kClass=0x15d420,kDepth=0xfffe06;
int32_t half(int16_t v){return v<0?(v+1)>>1:v>>1;}
}
NativeScreenZoom nativeScreenZoomGeometry(const NativeScreenZoomInputs &in){
    // The pages come from the 640-wide display buffer (FRAME FBW 10, CT24): ten
    // 64x32 pages per row, as the builder's x loop walks them.
    if(in.sourcePages!=10 || in.width<1 || in.width>2048 || in.height<1 || in.height>2048 || in.baseWidth<=0 || in.baseHeight<=0)
        throw std::runtime_error(std::format("native screen zoom outside the admitted frame layout: {}x{} over {}x{}, {} source pages",in.width,in.height,in.baseWidth,in.baseHeight,in.sourcePages));
    NativeScreenZoom out;out.width=in.width;out.height=in.height;out.sourceWidth=640;out.sourceHeight=in.fields?448:224;
    // Alpha (374ce4..374d50): (1 - v) * 160 by cvt.w.s, in the RGBAQ top byte;
    // the PSMCT24 pages read TEXA's 0x60, so the GS modulates 0x60 * A >> 7.
    const auto scaled=mul(k160,sub(kOne,in.visibility));
    const uint32_t vertexAlpha=(bozo::ps2f::le(kTwo31,scaled)?bozo::ps2f::toInt(sub(scaled,kTwo31))|0x80000000u:bozo::ps2f::toInt(scaled))&0xff;
    out.alpha=uint8_t(std::min<uint32_t>(0xff,(0x60*vertexAlpha)>>7));
    // 112860: sx = W/W0, sy = (H/H0) * (1 + fields); the builder's f9 = 2 or 1.
    const auto sx=divide(bozo::ps2f::fromInt(uint32_t(int32_t(in.width))),bozo::ps2f::fromInt(uint32_t(int32_t(in.baseWidth))));
    const auto sy=mul(divide(bozo::ps2f::fromInt(uint32_t(int32_t(in.height))),bozo::ps2f::fromInt(uint32_t(int32_t(in.baseHeight)))),add(kOne,in.fields?kOne:0));
    const uint32_t f9=in.fields?0x40000000u:kOne;
    const auto z=sub(in.zoom,kOne);                         // f22 (zoom path only)
    const auto cy=mul(kHalf,mul(k224,z)),cx=mul(kHalf,mul(k640,z));   // f21, f20
    const auto ox=bozo::ps2f::fromInt(uint32_t(0x800-half(in.width))),oy=bozo::ps2f::fromInt(uint32_t(0x800-half(in.height)));
    const auto limit=mul(k224,f9);
    const auto fixed=[](uint32_t v){return float(int32_t(bozo::ps2f::toInt(mul(kSixteen,v))))/16.f;};
    uint16_t row=0;
    for(uint32_t y=0;bozo::ps2f::lt(y,limit);y=add(y,kThirtyTwo),++row){
        uint32_t top,bottom;
        if(in.zoomed){
            const auto edge=[&](uint32_t v){return add(divide(mul(sy,add(v,sub(mul(v,z),cy))),f9),oy);};
            top=edge(y);bottom=edge(add(kThirtyTwo,y));
        }else{
            bottom=add(divide(mul(sy,add(add(kThirtyTwo,y),in.shiftY)),f9),oy);
            top=add(divide(mul(sy,add(y,in.shiftY)),f9),oy);
        }
        uint16_t column=0;
        for(uint32_t x=0;bozo::ps2f::lt(x,k640);x=add(x,kSixtyFour),++column){
            uint32_t left,right;
            if(in.zoomed){
                const auto edge=[&](uint32_t v){return add(mul(sx,add(v,sub(mul(v,z),cx))),ox);};
                left=edge(x);right=edge(add(kSixtyFour,x));
            }else{
                right=add(mul(sx,add(add(kSixtyFour,x),in.shiftX)),ox);
                left=add(mul(sx,add(x,in.shiftX)),ox);
            }
            out.strips.push_back({fixed(left),fixed(top),fixed(right),fixed(bottom),column,row});
        }
    }
    return out;
}
std::array<bozo::render::Vertex,6> nativeScreenZoomVertices(const NativeScreenZoom &zoom,const NativeScreenZoomStrip &s){
    const bool right[6]={false,true,false,true,true,false},bottom[6]={false,false,true,false,true,true};
    const float u0=64.f*s.column/zoom.sourceWidth,u1=64.f*(s.column+1)/zoom.sourceWidth,v0=32.f*s.row/zoom.sourceHeight,v1=32.f*(s.row+1)/zoom.sourceHeight;
    std::array<bozo::render::Vertex,6> vertices;
    for(unsigned i=0;i<6;++i){auto &v=vertices[i];
        v.x=right[i]?s.x1:s.x0;v.y=bottom[i]?s.y1:s.y0;v.z=float(kDepth);v.u=right[i]?u1:u0;v.v=bottom[i]?v1:v0;
        v.rgba=0x00808080u|uint32_t(zoom.alpha)<<24;}
    return vertices;
}
std::optional<NativeScreenZoom> NativeScreenZooms::begin(Runtime *rt,uint32_t stack,uint32_t object){
    if(calls.contains(stack))throw std::runtime_error("native screen zoom invocation repeats");
    if(value<uint32_t>(rt,object)!=kClass || value<uint8_t>(rt,object+0x64)!=1)throw std::runtime_error("unported native screen zoom type/state");
    const auto writer=value<uint32_t>(rt,object+0x94);
    if(!writer)return std::nullopt;
    const auto base=value<uint32_t>(rt,writer),begin=value<uint32_t>(rt,writer+12),end=value<uint32_t>(rt,writer+8);
    if(begin==end)return std::nullopt;   // skipped this frame: nothing reaches the flush
    if(!value<uint8_t>(rt,object+0x63))throw std::runtime_error("native screen zoom writer filled without its builder");
    const auto globals=value<uint32_t>(rt,0x35de60);
    NativeScreenZoomInputs in{.visibility=value<uint32_t>(rt,object+0x98),.zoom=value<uint32_t>(rt,object+0xa4),
        .shiftX=value<uint32_t>(rt,object+0xb0),.shiftY=value<uint32_t>(rt,object+0xb4),
        .zoomed=(value<uint8_t>(rt,object+0xc0)&1)!=0,.fields=value<uint8_t>(rt,0x1d5984)!=0,
        .width=value<int16_t>(rt,globals+0x3c6c),.height=value<int16_t>(rt,globals+0x3c6e),
        .baseWidth=value<int16_t>(rt,globals+0x3c68),.baseHeight=value<int16_t>(rt,globals+0x3c6a)};
    // The builder's TEX0 pages start at FRAME's FBP (globals +670); PSMCT24 only.
    const auto frame=value<uint64_t>(rt,globals+0x670);in.sourcePages=uint16_t((frame>>16)&0x3f);
    if(((frame>>24)&0x3f)!=1)throw std::runtime_error("native screen zoom source is not the CT24 display buffer");
    auto zoom=nativeScreenZoomGeometry(in);zoom.object=object;
    // Exact writer extent: ALPHA_2 (2 qwords), the 8-qword state block and
    // four qwords per page strip. Contents are not decoded; the original
    // scalar flush keeps their bookkeeping.
    const uint64_t expected=0x20+0x80+uint64_t(zoom.strips.size())*0x40;
    if(uint64_t(base)+0x40!=begin || uint64_t(begin)+expected!=end || uint64_t(end)+16>uint64_t(base)+uint64_t(value<uint32_t>(rt,writer+16))*8)
        throw std::runtime_error("native screen zoom writer has additional/unowned content");
    calls.emplace(stack,Lease{zoom,writer,begin,end});return zoom;
}
void NativeScreenZooms::retire(Runtime *rt,R5900Context *c){
    auto &lease=calls.at(c->r[29].uw[0]+0x30);
    if(lease.retired || c->r[17].uw[0]!=lease.writer || c->r[9].uw[0]!=lease.zoom.object+0x90 ||
       c->r[16].uw[0]!=lease.end+16 || c->r[2].uw[0]!=lease.begin || c->r[5].uw[0]!=2 || c->r[6].uw[0] || c->r[8].uw[0]!=(lease.end+16-lease.begin)/16 ||
       value<uint32_t>(rt,lease.writer+8)!=lease.end || value<uint32_t>(rt,lease.writer+12)!=lease.begin)
        throw std::runtime_error("native screen zoom changed during original flush");
    lease.retired=true;
}
void NativeScreenZooms::flushed(Runtime *rt,uint32_t stack){
    auto &lease=calls.at(stack);
    if(!lease.retired || lease.flushed || value<uint32_t>(rt,lease.writer+8)!=lease.end+16 || value<uint32_t>(rt,lease.writer+12)!=lease.end+16)
        throw std::runtime_error("native screen zoom did not finish its scalar flush");
    lease.flushed=true;
}
void NativeScreenZooms::complete(uint32_t stack){
    auto it=calls.find(stack);if(it==calls.end())return;   // empty writer: nothing was leased
    if(!it->second.flushed)throw std::runtime_error("native screen zoom completed before its flush");
    calls.erase(it);
}
std::optional<bool> nativeScreenZoomInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x374c04 && pc!=0x374c10 && pc!=0x120f4c && pc!=0x120f58)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<ScreenZoomOwner *>(&d->host()):nullptr;
    if(!owner || !c){if(pc==0x120f4c || pc==0x120f58)return std::nullopt;throw std::runtime_error("native screen zoom requires an owner");}
    auto &zooms=owner->nativeScreenZooms();const auto sp=c->r[29].uw[0];
    if(pc==0x374c04){
        // The flush call's delay slot, before its writer load: a0 = object, a1 = object+90.
        if((c->pc!=0x374be0 && c->pc!=0x374c00) || c->r[5].uw[0]!=c->r[4].uw[0]+0x90)throw std::runtime_error("native screen zoom requires its complete draw method");
        if(const auto zoom=zooms.begin(rt,sp,c->r[4].uw[0])){
            try{owner->drawNativeScreenZoom(*zoom);}catch(...){zooms.cancel(sp);throw;}
        }
    }else if(pc==0x374c10){
        if(c->pc!=0x374c08)throw std::runtime_error("native screen zoom completion escaped its return");
        zooms.complete(sp);
    }else{
        if(!zooms.active(sp+0x30))return std::nullopt;
        if(pc==0x120f4c)zooms.retire(rt,c);else zooms.flushed(rt,sp+0x30);
    }
    return false; // Preserve the original scalar instruction, including both delay slots.
}
}

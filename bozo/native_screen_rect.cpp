#include "native_screen_rect.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <format>
#include <stdexcept>
namespace vp2s {
namespace {
template<class T>T value(Runtime *rt,uint32_t at){
    if(!rt || at<0x100000 || at>0x2000000-sizeof(T))throw std::runtime_error("native screen rectangle outside EE RAM");
    T v;std::memcpy(&v,rt->memory.hostPointer(at,sizeof(T)),sizeof(T));return v;
}
}
// Flag byte +0xf5 of class 0x161230: the constructor (3d4b90) writes bits 0..4 only.
// The builder (3d3660) reads bits 2..4 (skip, pause gate, hidden); bits 0..1 steer the
// timer update and auto-destroy (3d46c0..3d47e4), and no class method reads bits 5..7.
constexpr uint8_t kFadeDrawFlags=0x1c;
void NativeScreenRects::captureFade(Runtime *rt,const R5900Context *c){
    const auto object=c->r[17].uw[0];
    if(value<uint32_t>(rt,object)!=0x161230 || value<uint32_t>(rt,object+0x90)!=0x1612ac ||
       value<uint8_t>(rt,object+0xf2) || value<uint8_t>(rt,object+0xf3) || value<uint8_t>(rt,object+0xf4) || (value<uint8_t>(rt,object+0xf5)&kFadeDrawFlags))
        throw std::runtime_error(std::format("unported native fade type/mode (object {:x} class {:x}/{:x} f2..f5 {:x} {:x} {:x} {:x})",object,
            value<uint32_t>(rt,object),value<uint32_t>(rt,object+0x90),value<uint8_t>(rt,object+0xf2),value<uint8_t>(rt,object+0xf3),
            value<uint8_t>(rt,object+0xf4),value<uint8_t>(rt,object+0xf5)));
    const auto source=value<uint32_t>(rt,0x35de60);
    NativeScreenRect rect{.object=object,.colour=c->r[10].uw[0],.depth=c->r[16].uw[0],
        .x=c->r[5].sw[0],.y=c->r[6].sw[0],.right=c->r[7].sw[0],.bottom=c->r[8].sw[0],
        .sourceWidth=value<int16_t>(rt,source+0x3c6c),.sourceHeight=value<int16_t>(rt,source+0x3c6e)};
    if(c->r[4].uw[0]!=value<uint32_t>(rt,object+0xc4) || rect.colour!=value<uint32_t>(rt,object+0xa0) ||
       rect.depth!=value<uint32_t>(rt,object+0xc0)+value<uint8_t>(rt,object+0xf1) || rect.depth>0xffffff ||
       rect.x || rect.y || rect.right!=rect.sourceWidth || rect.bottom!=rect.sourceHeight ||
       rect.sourceWidth<1 || rect.sourceWidth>2048 || rect.sourceHeight<1 || rect.sourceHeight>2048)
        throw std::runtime_error("native flat fade escaped original rectangle producer");
    fades[object]=rect;
}
NativeScreenRect NativeScreenRects::beginFade(Runtime *rt,uint32_t stack,uint32_t object){
    if(calls.contains(stack))throw std::runtime_error("native fade publication stack reused");
    auto it=fades.find(object);
    if(it==fades.end() || value<uint32_t>(rt,object)!=0x161230 || !value<uint8_t>(rt,object+0x63) || value<uint8_t>(rt,object+0x64)!=1 ||
       value<uint8_t>(rt,object+0xf2) || value<uint8_t>(rt,object+0xf3) || value<uint8_t>(rt,object+0xf4) || (value<uint8_t>(rt,object+0xf5)&kFadeDrawFlags))
        throw std::runtime_error("native fade has no completed flat builder");
    const auto scene=value<uint32_t>(rt,0x35ddf8);const auto layer=value<uint8_t>(rt,object+0x65);
    if(layer>1 || value<uint32_t>(rt,scene+0x40a8+layer*4)!=object || value<uint8_t>(rt,scene+0x40dc+layer)!=1)
        throw std::runtime_error("native fade has no pending scene owner");
    const auto buffer=value<uint32_t>(rt,object+0xc4),base=value<uint32_t>(rt,buffer),begin=value<uint32_t>(rt,buffer+12),end=value<uint32_t>(rt,buffer+8);
    if(uint64_t(base)+0x40!=begin || uint64_t(begin)+96!=end || uint64_t(end)+16>uint64_t(base)+uint64_t(value<uint32_t>(rt,buffer+16))*8)
        throw std::runtime_error("native fade writer has additional/unowned content");
    calls.emplace(stack,Lease{it->second,buffer,begin,end});return it->second;
}
void NativeScreenRects::releaseFade(uint32_t object){
    for(const auto &[stack,call]:calls)if(call.rect.object==object)throw std::runtime_error("native fade destroyed during publication");
    fades.erase(object);
}
NativeScreenRect NativeScreenRects::begin(Runtime *rt,uint32_t stack,uint32_t object){
    if(calls.contains(stack))throw std::runtime_error("native screen rectangle invocation repeats");
    if(value<uint32_t>(rt,object)!=0x15bc00 || value<uint32_t>(rt,object+0x90)!=0x15bc7c ||
       value<uint8_t>(rt,object+0xf3) || value<uint8_t>(rt,object+0xf4))
        throw std::runtime_error("unported native screen rectangle mode/type");
    const auto scene=value<uint32_t>(rt,0x35ddf8);
    const auto layer=value<uint8_t>(rt,object+0x65);
    if(layer>1 || value<uint32_t>(rt,scene+0x40a8+layer*4)!=object || value<uint8_t>(rt,scene+0x40dc+layer)!=1)
        throw std::runtime_error("native screen rectangle has no pending scene object");
    NativeScreenRect rect{.object=object,.colour=value<uint32_t>(rt,object+0x9c),
        .depth=value<uint32_t>(rt,object+0xbc)+value<uint8_t>(rt,object+0xf1),
        .x=value<int32_t>(rt,object+0xac),.y=value<int32_t>(rt,object+0xb0),
        .right=value<int32_t>(rt,object+0xb4),.bottom=value<int32_t>(rt,object+0xb8)};
    const auto source=value<uint32_t>(rt,0x35de60);
    rect.sourceWidth=value<int16_t>(rt,source+0x3c6c);rect.sourceHeight=value<int16_t>(rt,source+0x3c6e);
    if(rect.right<rect.x || rect.bottom<rect.y || rect.right>8192 || rect.bottom>8192 || rect.x < -8192 || rect.x>8192 || rect.y < -8192 || rect.y>8192 || rect.depth>0xffffff)
        throw std::runtime_error("native screen rectangle dimensions/depth outside admitted range");
    if(rect.sourceWidth<1 || rect.sourceWidth>2048 || rect.sourceHeight<1 || rect.sourceHeight>2048)
        throw std::runtime_error("native screen rectangle source dimensions outside admitted range");
    const auto writer=value<uint32_t>(rt,object+0xc0),base=value<uint32_t>(rt,writer),
        begin=value<uint32_t>(rt,writer+12),end=value<uint32_t>(rt,writer+8);
    // Mode 0's exact writer extent: native defaults, blend selection, rectangle.
    // Contents are not decoded. Original scalar flush keeps their bookkeeping.
    if(uint64_t(base)+0x40!=begin || uint64_t(begin)+224!=end || uint64_t(end)+16>uint64_t(base)+uint64_t(value<uint32_t>(rt,writer+16))*8)
        throw std::runtime_error("native screen rectangle writer has additional/unowned content");
    calls.emplace(stack,Lease{rect,writer,begin,end});return rect;
}
std::array<bozo::render::Vertex,6> nativeScreenRectangleVertices(const NativeScreenRect &rect){
    const bool right[6]={false,true,false,true,true,false},bottom[6]={false,false,true,false,true,true};
    std::array<bozo::render::Vertex,6> vertices;
    for(unsigned i=0;i<6;++i){
        vertices[i].x=float(2048-rect.sourceWidth/2+(right[i]?rect.right:rect.x));
        vertices[i].y=float(2048-rect.sourceHeight/2+(bottom[i]?rect.bottom:rect.y));
        vertices[i].z=float(rect.depth);vertices[i].rgba=rect.colour;
    }
    return vertices;
}
void NativeScreenRects::retire(Runtime *rt,R5900Context *c){
    auto &lease=calls.at(c->r[29].uw[0]+0x30);
    if(lease.retired || c->r[17].uw[0]!=lease.writer || c->r[9].uw[0]!=lease.rect.object+0x90 ||
       c->r[16].uw[0]!=lease.end+16 || c->r[2].uw[0]!=lease.begin || c->r[5].uw[0]!=2 || c->r[6].uw[0] || c->r[8].uw[0]!=(lease.end+16-lease.begin)/16 ||
       value<uint32_t>(rt,lease.writer+8)!=lease.end || value<uint32_t>(rt,lease.writer+12)!=lease.begin)
        throw std::runtime_error("native screen rectangle changed during original flush");
    lease.retired=true;
}
void NativeScreenRects::flushed(Runtime *rt,uint32_t stack){
    auto &lease=calls.at(stack);
    if(!lease.retired || lease.flushed || value<uint32_t>(rt,lease.writer+8)!=lease.end+16 || value<uint32_t>(rt,lease.writer+12)!=lease.end+16)
        throw std::runtime_error("native screen rectangle did not finish its scalar flush");
    lease.flushed=true;
}
uint32_t NativeScreenRects::complete(uint32_t stack){
    const auto lease=calls.at(stack);if(!lease.flushed)throw std::runtime_error("native screen rectangle completed before its flush");
    calls.erase(stack);return lease.rect.object;
}
std::optional<bool> nativeScreenRectInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x3721a4 && pc!=0x3721b0 && pc!=0x3d38d4 && pc!=0x3d363c && pc!=0x3d364c && pc!=0x3d49b0 && pc!=0x120f4c && pc!=0x120f58)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<ScreenRectOwner *>(&d->host()):nullptr;
    if(!owner || !c){if(pc==0x120f4c || pc==0x120f58)return std::nullopt;throw std::runtime_error("native screen rectangle requires an owner");}
    auto &rects=owner->nativeScreenRects();const auto sp=c->r[29].uw[0];
    if(pc==0x3d38d4)rects.captureFade(rt,c);
    else if(pc==0x3d49b0)rects.releaseFade(c->r[4].uw[0]);
    else if(pc==0x3d363c){
        const auto rect=rects.beginFade(rt,sp,c->r[4].uw[0]);
        try{owner->drawNativeScreenRect(rect);}catch(...){rects.cancel(sp);throw;}
    }else if(pc==0x3d364c)c->r[4].sd[0]=int32_t(rects.complete(sp));
    else if(pc==0x3721a4){
        if(c->pc!=0x372180 || c->r[5].uw[0]!=c->r[4].uw[0]+0x90)throw std::runtime_error("native screen rectangle requires its complete producer");
        const auto rect=rects.begin(rt,sp,c->r[4].uw[0]);
        try{owner->drawNativeScreenRect(rect);}catch(...){rects.cancel(sp);throw;}
    }else if(pc==0x3721b0){
        if(c->pc!=0x3721a8)throw std::runtime_error("native screen rectangle completion escaped its return");
        c->r[4].sd[0]=static_cast<int32_t>(rects.complete(sp));
    }else{
        if(!rects.active(sp+0x30))return std::nullopt;
        if(pc==0x120f4c)rects.retire(rt,c);else rects.flushed(rt,sp+0x30);
    }
    return false; // Preserve the original scalar instruction, including both delay slots.
}
}

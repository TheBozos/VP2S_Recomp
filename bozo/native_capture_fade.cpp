#include "native_capture_fade.h"
#include "bozo/ee/dispatcher.h"
#include <bit>
#include <cstring>
#include <format>
#include <stdexcept>
namespace vp2s {
namespace {
template<class T>T value(Runtime *rt,uint32_t at){
    if(!rt || at<0x100000 || at>0x2000000-sizeof(T))throw std::runtime_error("native capture fade outside EE RAM");
    T v;std::memcpy(&v,rt->memory.hostPointer(at,sizeof(T)),sizeof(T));return v;
}
constexpr uint32_t kClass=0x1610f0;
// The screen the strips cover: 35de60 +3c6c/+3c6e, as 3d2630 reads them.
std::array<int32_t,2> screen(Runtime *rt){
    const auto source=value<uint32_t>(rt,0x35de60);
    return {value<int16_t>(rt,source+0x3c6c),value<int16_t>(rt,source+0x3c6e)};
}
}
NativeCaptureFades::Snapshot *NativeCaptureFades::find(uint32_t object){
    for(auto &image:images)if(image.object==object)return &image;
    return nullptr;
}
uint8_t NativeCaptureFades::capture(uint32_t object){
    auto *slot=find(object);
    if(!slot){slot=&images[0];for(auto &image:images)if(image.used<slot->used)slot=&image;}
    built.erase(slot->object);
    *slot=Snapshot{object,++clock,false};
    return uint8_t(kFirstImage+(slot-images.data()));
}
std::optional<uint8_t> NativeCaptureFades::pendingCopy(uint32_t object){
    auto *slot=find(object);if(!slot || slot->copied)return std::nullopt;
    slot->copied=true;slot->used=++clock;
    return uint8_t(kFirstImage+(slot-images.data()));
}
std::optional<uint8_t> NativeCaptureFades::image(uint32_t object){
    auto *slot=find(object);if(!slot || !slot->copied)return std::nullopt;
    return uint8_t(kFirstImage+(slot-images.data()));
}
void NativeCaptureFades::build(const NativeCaptureFadeDraw &draw){
    auto *slot=find(draw.object);
    if(!slot || !slot->copied || kFirstImage+(slot-images.data())!=draw.image)throw std::runtime_error("native capture fade has no copied snapshot");
    slot->used=++clock;built[draw.object]=draw;
}
std::optional<NativeCaptureFadeDraw> NativeCaptureFades::submit(uint32_t object){
    const auto it=built.find(object);if(it==built.end())return std::nullopt;
    const auto draw=it->second;built.erase(it);
    if(!find(object))throw std::runtime_error("native capture fade snapshot recycled before its draw");
    return draw;
}
std::array<bozo::render::Vertex,6> nativeCaptureFadeVertices(const NativeCaptureFadeDraw &draw){
    // 3d2630: x from 0x800 - width/2 in 64-pixel strips, y from 0x800 - height/2,
    // each texel at its own pixel (UV +0.5 on pixel corners).
    const bool right[6]={false,true,false,true,true,false},bottom[6]={false,false,true,false,true,true};
    const int32_t x=0x800-draw.width/2,y=0x800-draw.height/2;
    std::array<bozo::render::Vertex,6> vertices;
    for(unsigned i=0;i<6;++i){
        auto &v=vertices[i];v.x=float(x+(right[i]?draw.width:0));v.y=float(y+(bottom[i]?draw.height:0));
        v.z=float(draw.depth);v.u=right[i]?1.f:0.f;v.v=bottom[i]?1.f:0.f;v.rgba=draw.colour;
    }
    return vertices;
}
std::optional<bool> nativeCaptureFadeInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x3d2d7c && pc!=0x3d2630 && pc!=0x3d2698 && pc!=0x3d2520)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<CaptureFadeOwner *>(&d->host()):nullptr;
    if(!c || !owner)throw std::runtime_error("native capture fade requires an owner");
    auto &fades=owner->nativeCaptureFades();
    if(pc==0x3d2d7c){
        // 3d2cd0 state 0, every capture condition met (delay slot of the patched
        // branch 3d2d78): the branch goes to
        // its tail (+a0 = +98, state 1) without readback nodes or strip headers.
        const auto object=c->r[21].uw[0];
        const auto strips=value<int32_t>(rt,object+0xaa0);
        if(value<uint32_t>(rt,object)!=kClass || value<uint8_t>(rt,object+0x9d4) || !value<uint32_t>(rt,object+0x9c4) ||
           !(std::bit_cast<float>(value<uint32_t>(rt,object+0xa4))<0.f) || (value<uint32_t>(rt,value<uint32_t>(rt,0x48e048)+0x8c)&1) ||
           strips<1 || strips>16)
            throw std::runtime_error("native capture fade escaped its capture condition");
        for(int32_t i=0;i<strips;++i)if(value<uint32_t>(rt,object+0x984+4*i))throw std::runtime_error("native capture fade already has readback nodes");
        fades.capture(object);
        return false; // The patched branch runs.
    }
    if(pc==0x3d2630){
        const auto object=c->r[4].uw[0];
        if(value<uint32_t>(rt,object)!=kClass)throw std::runtime_error("native capture fade builder outside its class");
        if(const auto image=fades.pendingCopy(object))owner->copyNativeFrameToImage(*image);
        return false;
    }
    if(pc==0x3d2698){
        // Builder after its state (>= 2) and buffer checks: the original strip
        // uploads and sprites are replaced by one native draw of the snapshot.
        const auto object=c->r[16].uw[0];const auto size=screen(rt);
        if(value<uint32_t>(rt,object)!=kClass || value<int8_t>(rt,object+0x9d4)<2 || !value<uint32_t>(rt,object+0xc4) || !value<uint32_t>(rt,object+0x9c4))
            throw std::runtime_error("native capture fade builder escaped its draw path");
        if(size[0]<64 || size[0]>2048 || size[1]<1 || size[1]>2048 || value<int32_t>(rt,object+0xaa0)*64!=size[0])
            throw std::runtime_error(std::format("unported native capture fade strips ({} for {}x{})",value<int32_t>(rt,object+0xaa0),size[0],size[1]));
        NativeCaptureFadeDraw draw{.object=object,.colour=value<uint32_t>(rt,object+0xa0),.depth=value<uint32_t>(rt,object+0xc0),.width=size[0],.height=size[1]};
        // The image claimed at capture and copied at the fade's first builder.
        const auto image=fades.image(object);
        if(!image)throw std::runtime_error("native capture fade drawn before its copy");
        draw.image=*image;fades.build(draw);
        return false;
    }
    // 3d2520 submit: strip count 0, so its only entry is the builder's own packet
    // (register header, 3d3500 defaults, terminator: 0x11 qwords, no draw). The
    // patched tail calls the real completion 21c9a0(object) instead of queueing it.
    const auto object=c->r[4].uw[0];
    if(value<uint32_t>(rt,object)!=kClass || value<int32_t>(rt,object+0xa98)!=0)
        throw std::runtime_error("native capture fade submit has original packets");
    if(const auto draw=fades.submit(object)){
        const auto writer=value<uint32_t>(rt,object+0xc4),begin=value<uint32_t>(rt,writer+12),end=value<uint32_t>(rt,writer+8);
        if(value<uint32_t>(rt,object+0x9d8)!=begin || value<uint32_t>(rt,object+0x9dc)!=0x11 || uint64_t(begin)+0x110!=end)
            throw std::runtime_error(std::format("native capture fade packet has additional content ({:x}+{:x} writer {:x}..{:x})",
                value<uint32_t>(rt,object+0x9d8),value<uint32_t>(rt,object+0x9dc),begin,end));
        owner->drawNativeCaptureFade(*draw);
    }
    return false;
}
}

#include "native_frame_copy.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/render/renderer.h"
#include <cstring>
#include <format>
#include <immintrin.h>
#include <cmath>
namespace vp2s {
namespace {
struct PreserveHostFp {unsigned saved=_mm_getcsr();~PreserveHostFp(){_mm_setcsr(saved);}};
template<class T>T value(Runtime *rt,uint32_t at){
    if(!rt || at<0x100000 || uint64_t(at)+sizeof(T)>0x2000000)throw std::runtime_error("native frame-copy metadata outside RAM");
    T out;std::memcpy(&out,rt->memory.hostPointer(at,sizeof(T)),sizeof(T));return out;
}
uint32_t owner(Runtime *rt,uint32_t object){
    if(value<uint32_t>(rt,object)!=0x159ee0 || value<uint32_t>(rt,object+0x90)!=0x159f5c)
        throw std::runtime_error("unowned field frame-copy object");
    const auto scene=value<uint32_t>(rt,0x35ddf8);
    if(value<uint32_t>(rt,scene+0x444c)!=object)throw std::runtime_error("field frame copy escaped scene lifetime");
    return scene;
}
}
void NativeFrameCopies::begin(Runtime *rt,uint32_t stack,uint32_t object){
    owner(rt,object);if(builds.contains(stack))throw std::runtime_error("active frame-copy stack reused");
    for(const auto &[at,build]:builds)if(build.object==object)throw std::runtime_error("frame-copy object already has a builder");
    ready.erase(object);images.erase(object);builds.emplace(stack,Build{object});
}
void NativeFrameCopies::activate(uint32_t stack,uint32_t object,uint32_t source){
    const auto it=builds.find(stack);
    if(it==builds.end() || it->second.object!=object || it->second.active)throw std::runtime_error("frame-copy activation escaped original builder");
    it->second.active=true;it->second.source=source;
}
void NativeFrameCopies::finish(Runtime *rt,const NativeScene &scene,uint32_t stack,uint32_t object){
    PreserveHostFp arithmetic; // Fractional logical-to-pixel validation must not add host flags.
    const auto it=builds.find(stack);
    if(it==builds.end() || it->second.object!=object)throw std::runtime_error("frame copy finished without original builder");
    owner(rt,object);
    if(it->second.active && value<uint8_t>(rt,object+0x63)){
        const auto frame=scene.frameTarget();
        if(!frame || frame->object!=it->second.source || value<uint32_t>(rt,object+0x94)!=frame->object ||
           value<uint32_t>(rt,object+0xa8)!=uint32_t(frame->width) || value<uint32_t>(rt,object+0xac)!=uint32_t(frame->height) ||
           value<uint32_t>(rt,object+0xa8)!=uint32_t(value<int16_t>(rt,frame->object+0x3c6c)) || value<uint32_t>(rt,object+0xac)!=uint32_t(value<int16_t>(rt,frame->object+0x3c6e)))
            throw std::runtime_error(std::format("field frame copy has no matching native source: frame={:x} {}x{} observed={:x} objectSource={:x} {}x{}",frame?frame->object:0,frame?frame->width:0,frame?frame->height:0,it->second.source,value<uint32_t>(rt,object+0x94),value<uint32_t>(rt,object+0xa8),value<uint32_t>(rt,object+0xac)));
        const auto width=value<uint32_t>(rt,object+0x1a0),height=value<uint32_t>(rt,object+0x1a4);
        if(value<uint32_t>(rt,object+0x198)!=128 || value<uint32_t>(rt,object+0x19c)!=128 ||
           !width || !height || width>128 || height>128 || value<uint32_t>(rt,object+0xec)!=width || value<uint32_t>(rt,object+0xf0)!=height)
            throw std::runtime_error("unqualified field frame-copy dimensions");
        ready[object]={object,width,height,frame};
    }
    builds.erase(it);
}
NativeFrameCopy NativeFrameCopies::submit(Runtime *rt,const NativeScene &scene,uint32_t object)const{
    const auto current=owner(rt,object);const auto layer=value<uint8_t>(rt,object+0x65);const auto it=ready.find(object);
    if(it==ready.end() || it->second.frame!=scene.frameTarget() || layer>1 ||
       value<uint32_t>(rt,current+0x40a8+4*layer)!=object || !value<uint8_t>(rt,current+0x40dc+layer) ||
       value<uint32_t>(rt,object+0x94)!=it->second.frame->object || value<uint32_t>(rt,object+0x1a0)!=it->second.width || value<uint32_t>(rt,object+0x1a4)!=it->second.height ||
       value<uint32_t>(rt,object+0xec)!=it->second.width || value<uint32_t>(rt,object+0xf0)!=it->second.height || value<uint32_t>(rt,object+0x198)!=128 || value<uint32_t>(rt,object+0x19c)!=128)
        throw std::runtime_error("field frame copy submitted outside completed builder/frame");
    return it->second;
}
void NativeFrameCopies::published(uint32_t object){
    const auto it=ready.find(object);if(it==ready.end())throw std::runtime_error("field frame copy published twice");
    images[object]=it->second;ready.erase(it);
}
void NativeFrameCopies::release(uint32_t object){
    for(const auto &[stack,build]:builds)if(build.object==object)throw std::runtime_error("field frame copy destroyed during builder");
    ready.erase(object);images.erase(object);
}
std::optional<NativeFrameCopy> NativeFrameCopies::image(const NativeScene &scene,uint32_t object)const{
    const auto it=images.find(object);return it==images.end() || it->second.frame!=scene.frameTarget()?std::nullopt:std::optional(it->second);
}
void drawFrameCopy(const NativeFrameCopy &copy){
    PreserveHostFp arithmetic;
    if(!copy.frame || !copy.width || !copy.height || copy.width>128 || copy.height>128 ||
       !std::isfinite(copy.frame->width) || !std::isfinite(copy.frame->height) || copy.frame->width<1 || copy.frame->height<1 || copy.frame->width>2048 || copy.frame->height>2048)
        throw std::runtime_error("invalid native frame-copy draw");
    auto &renderer=bozo::render::Renderer::instance();renderer.setTargetSize(7,uint16_t(copy.width),uint16_t(copy.height));
    bozo::render::Material material;material.textured=false;material.depthTest=false;material.depthWrite=false;
    material.alphaBlend=false;material.blend=bozo::render::Material::Blend::Opaque;material.fromFrame=1;material.target=6;
    material.clampU=material.clampV=true;material.nearest=true;
    const float xy[6][2]={{0,0},{1,0},{0,1},{1,0},{1,1},{0,1}},identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    bozo::render::Vertex vertices[6];for(unsigned i=0;i<6;++i){auto &v=vertices[i];v.x=2048-copy.frame->width/2+xy[i][0]*copy.frame->width;
        v.y=2048-copy.frame->height/2+xy[i][1]*copy.frame->height;v.u=xy[i][0];v.v=xy[i][1];v.rgba=0x80808080;}
    // The engine's linear main-frame sampler deliberately adds a mip LOD bias.
    // Preserve a one-to-one native snapshot first, then the game's bilinear
    // reduction from that independent image, where there is no extra mip blur.
    renderer.setTargetSize(6,uint16_t(copy.frame->width),uint16_t(copy.frame->height));
    renderer.draw(material,identity,vertices,6);
    material.source=6;material.target=7;material.nearest=false;renderer.draw(material,identity,vertices,6);
}
std::optional<bool> nativeFrameCopyInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x2dbde0 && pc!=0x2dbeb4 && pc!=0x2dc6c8 && pc!=0x2dc9a4 && pc!=0x2dc700)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *copies=d?dynamic_cast<FrameCopyOwner *>(&d->host()):nullptr;auto *scene=d?dynamic_cast<SceneOwner *>(&d->host()):nullptr;
    if(!c || !copies || !scene)throw std::runtime_error("native field frame copy requires scene/renderer owner");
    const auto stack=c->r[29].uw[0];auto &manager=copies->nativeFrameCopies();
    if(pc==0x2dbde0){if(c->pc!=pc)throw std::runtime_error("frame copy requires full builder");manager.begin(rt,stack-0xc0,c->r[4].uw[0]);}
    else if(pc==0x2dbeb4)manager.activate(stack,c->r[18].uw[0],c->r[2].uw[0]);
    else if(pc==0x2dc6c8)manager.finish(rt,scene->nativeScene(),stack,c->r[18].uw[0]);
    else if(pc==0x2dc700)manager.release(c->r[4].uw[0]);
    else{
        if(c->pc!=0x2dc9a0 || c->r[4].uw[0]<0x100090)throw std::runtime_error("frame copy requires original submission thunk");
        const auto object=c->r[4].uw[0]-0x90;const auto copy=manager.submit(rt,scene->nativeScene(),object);
        copies->drawNativeFrameCopy(copy);manager.published(object);
    }
    return false; // Original scalar producer and real software completion execute.
}
}

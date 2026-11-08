#include "native_procedural.h"
#include "native_assets.h"
#include "bozo/ee/dispatcher.h"
#include <bit>
#include <cstring>
namespace vp2s {
namespace {
uint32_t address(uint32_t at){return at>=0x70000000 && at<0x70004000?at:NativeAssets::address(at);}
uint8_t *record(Runtime *rt,uint32_t address,uint32_t bytes){
    const auto at=vp2s::address(address);
    if(!rt || (at&15) || !((at>=0x100000 && uint64_t(at)+bytes<=0x2000000) || (at>=0x70000000 && uint64_t(at)+bytes<=0x70004000)))throw std::runtime_error("procedural projection outside owned memory");
    auto *p=rt->memory.hostPointer(address,bytes);if(!p)throw std::runtime_error("procedural projection missing memory");return p;
}
uint32_t word(const uint8_t *source){uint32_t out;std::memcpy(&out,source,4);return out;}
void producer(const R5900Context *c){
    if(!c || c->pc!=0x2481d0 || c->r[31].uw[0]!=0x2481d0 || c->r[4].uw[0]!=c->r[29].uw[0]+0x130)throw std::runtime_error("procedural projection escaped original producer");
}
}
void NativeProceduralTransforms::publish(Runtime *rt,const R5900Context *c,const NativeScene &scene){
    producer(c);const auto stack=c->r[29].uw[0],object=c->r[17].uw[0],camera=c->r[16].uw[0];auto *frame=record(rt,stack,0x170);
    const auto caller=word(frame+0x20);if(caller!=0x202308 && caller!=0x202bc8 && caller!=0x203d28)throw std::runtime_error("procedural projection has unknown scalar caller");
    record(rt,object,0x250);const auto view=scene.drawingCamera();if(!view || view->object!=camera)throw std::runtime_error("procedural projection has no selected camera");
    NativeProceduralProjection input{object,camera,address(stack+0x170)};
    for(unsigned row=0;row<4;++row)for(unsigned column=0;column<4;++column){
        const auto bits=word(frame+0x130+16*column+4*row),magnitude=bits&0x7fffffff;
        if(magnitude>=0x7f800000 || (magnitude && magnitude<0x00800000))throw std::runtime_error("invalid procedural projection coefficient");
        input.screenFromLocal[4*row+column]=std::bit_cast<float>(bits);
    }
    if(!draws.emplace(input.frame,input).second)throw std::runtime_error("procedural projection overlaps an active draw");
}
const NativeProceduralProjection &NativeProceduralTransforms::projection(uint32_t frame,uint32_t object)const{
    const auto found=draws.find(address(frame));if(found==draws.end() || found->second.object!=object)throw std::runtime_error("procedural point has no matching draw projection");return found->second;
}
void NativeProceduralTransforms::finish(uint32_t frame,uint32_t object){projection(frame,object);draws.erase(address(frame));}
std::optional<bool> nativeProceduralInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool publish=pc>=0x248260 && pc<=0x24826c && !(pc&3);if(!publish && pc!=0x247ec0)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;auto *owner=d?dynamic_cast<ProceduralOwner *>(&d->host()):nullptr;auto *scene=d?dynamic_cast<SceneOwner *>(&d->host()):nullptr;
    if(!c || !owner || !scene)throw std::runtime_error("procedural projection lacks semantic owner");
    if(publish){producer(c);if(pc==0x24826c)owner->nativeProceduralTransforms().publish(rt,c,scene->nativeScene());return true;}
    if(c->pc!=0x247ec0 || (c->r[31].uw[0]!=0x202960 && c->r[31].uw[0]!=0x2038d0 && c->r[31].uw[0]!=0x2053f8))throw std::runtime_error("procedural draw finish has unknown caller");
    owner->nativeProceduralTransforms().finish(c->r[29].uw[0],c->r[4].uw[0]);return false;
}
}

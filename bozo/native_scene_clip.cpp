// Scene scissor element 1597e0 as a native renderer clip (see native_scene_clip.h).
#include "native_scene_clip.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <format>
namespace vp2s {
namespace {
template<class T>T value(Runtime *rt,uint32_t at){
    if(!rt || at<0x100000 || uint64_t(at)+sizeof(T)>0x2000000)throw std::runtime_error("native scene clip record outside EE RAM");
    T v;std::memcpy(&v,rt->memory.hostPointer(at,sizeof(T)),sizeof(T));return v;
}
}
NativeSceneClip nativeSceneClip(Runtime *rt,const NativeScene &scene,uint32_t object){
    if(object<0x100000 || value<uint32_t>(rt,object)!=0x1597e0 || value<uint32_t>(rt,object+0x90)!=0x15985c)
        throw std::runtime_error("native scene clip escaped its class");
    const auto current=value<uint32_t>(rt,0x35ddf8);const unsigned layer=value<uint8_t>(rt,object+0x65);
    if(current<0x100000 || layer>1 || value<uint32_t>(rt,current+0x40a8+4*layer)!=object || !value<uint8_t>(rt,current+0x40dc+layer))
        throw std::runtime_error("native scene clip outside its scene slot");
    // 2b5b60 writes the packet only when built (+63); with a source object
    // (+94) the rectangle comes from 112960(source, 3) instead: not ported.
    if(!value<uint8_t>(rt,object+0x63))throw std::runtime_error("native scene clip enqueued without its build");
    if(value<uint32_t>(rt,object+0x94))throw std::runtime_error("unported native scene clip from a source object (112960)");
    const auto frame=scene.frameTarget();if(!frame)throw std::runtime_error("native scene clip has no frame");
    // SCISSOR = +a4 | +ac << 16 | +a8 << 32 | +b0 << 48: SCX0, SCX1, SCY0, SCY1.
    NativeSceneClip clip{object,{value<int32_t>(rt,object+0xa4),value<int32_t>(rt,object+0xa8),value<int32_t>(rt,object+0xac),value<int32_t>(rt,object+0xb0)}};
    const auto &r=clip.rect;
    if(r[0]<0 || r[1]<0 || r[2]<r[0] || r[3]<r[1] || r[2]>=int32_t(frame->width) || r[3]>=int32_t(frame->height))
        throw std::runtime_error(std::format("native scene clip {},{}..{},{} outside the frame",r[0],r[1],r[2],r[3]));
    return clip;
}
std::optional<bool> nativeSceneClipInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x2b5b50)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<SceneClipOwner *>(&d->host()):nullptr;auto *scene=d?dynamic_cast<SceneOwner *>(&d->host()):nullptr;
    if(!c || !owner || !scene)throw std::runtime_error("native scene clip requires its scene/renderer owner");
    // The enqueue 2b5b30 (a1 = a0 + 0x90): its call becomes the completion 2b5b10.
    if(c->pc!=0x2b5b30 || c->r[5].uw[0]<0x100090 || c->r[4].uw[0]!=c->r[5].uw[0]-0x90)
        throw std::runtime_error("native scene clip requires its original pre-enqueue producer");
    const auto object=c->r[4].uw[0];owner->setNativeSceneClip(nativeSceneClip(rt,scene->nativeScene(),object));
    c->r[4].sd[0]=int32_t(object);return true;
}
}

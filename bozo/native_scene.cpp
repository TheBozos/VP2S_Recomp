#include "native_scene.h"
#include "bozo/ee/dispatcher.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <format>
namespace vp2s {
namespace {
uint8_t *record(Runtime *rt,uint32_t at,uint32_t bytes){
    if(!rt || at<0x100000 || at>=0x2000000 || bytes>0x2000000-at)
        throw std::runtime_error("native camera metadata outside EE RAM");
    return rt->memory.hostPointer(at,bytes);
}
uint32_t word(Runtime *rt,uint32_t at){uint32_t v;std::memcpy(&v,record(rt,at,4),4);return v;}
}
void NativeScene::publishCamera(Runtime *rt,uint32_t object){
    auto *source=record(rt,object,0x8e4);const uint32_t job=word(rt,object+0x8e0);
    if(word(rt,job)!=0x153140)throw std::runtime_error(std::format("unowned camera transport type {:x} job={:x} camera={:x}",word(rt,job),job,object));
    auto view=std::make_shared<NativeCamera>();view->object=object;
    view->clearColour=word(rt,object+0x5a0)&0xffffff;
    std::memcpy(view->view.data(),source+0x2f0,64);std::memcpy(view->screen.data(),source+0x330,64);
    std::memcpy(view->guard.data(),source+0x370,64);std::memcpy(view->viewport.data(),source+0x260,16);
    std::memcpy(view->centre.data(),source+0x244,8);std::memcpy(view->fog.data(),source+0x5a8,8);
    cameras[object]=std::move(view);++cameraPublications;
    if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:scene] native camera=%x clear=%06x\n",object,cameras[object]->clearColour);
}
std::shared_ptr<const NativeCamera> NativeScene::camera(uint32_t object)const{
    auto it=cameras.find(object);return it==cameras.end()?nullptr:it->second;
}
bool nativeSceneInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x208a94)return false;
    auto *dispatcher=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=dispatcher?dynamic_cast<SceneOwner *>(&dispatcher->host()):nullptr;
    if(!owner || !c)throw std::runtime_error("native camera publication requires a scene owner");
    const uint32_t camera=c->r[17].uw[0];owner->nativeScene().publishCamera(rt,camera);
    // Retain the original scalar delay-slot load. The camera-only transport
    // enqueue was retired before a job exists; its completion method is empty.
    c->r[5].sd[0]=static_cast<int32_t>(word(rt,camera+0x8e0));return true;
}
}

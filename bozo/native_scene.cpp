#include "native_scene.h"
#include "bozo/ee/dispatcher.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <format>
#include <cmath>
namespace vp2s {
namespace {
uint8_t *record(Runtime *rt,uint32_t at,uint32_t bytes){
    if(!rt || at<0x100000 || at>=0x2000000 || bytes>0x2000000-at)
        throw std::runtime_error("native camera metadata outside EE RAM");
    return rt->memory.hostPointer(at,bytes);
}
uint32_t word(Runtime *rt,uint32_t at){uint32_t v;std::memcpy(&v,record(rt,at,4),4);return v;}
template<class T>T value(const uint8_t *r,uint32_t at){T v;std::memcpy(&v,r+at,sizeof(v));return v;}
}
namespace {
NativeFrameTarget sourceTarget(Runtime *rt,uint32_t object){
    const auto *r=record(rt,object,0x3e88);
    // Initial ordinary source-target path only. Secondary copies, extra layers
    // and the alternate direct-display path need their own native producers.
    if(!r[0x3c76] || value<uint32_t>(r,0x3dd8) || r[0x3e48] || value<int16_t>(r,0x3e78))
        throw std::runtime_error("unported secondary native frame composition");
    // Startup configures two complementary field views of the same source,
    // not two scene layers. Their source/size contract is checked below.
    if(r[0x31d0] || r[0x3c40] || r[0x1cf0]!=1 || r[0x2760]!=1 ||
       r[0x1cf2]>1 || r[0x2762]!=(r[0x1cf2]^1))
        throw std::runtime_error("unported native display layer composition");
    if(*record(rt,0x1c8d00+0xcc84,1)==2)throw std::runtime_error("unported alternate native frame alignment");
    // Logical game dimensions/scales before field division and physical
    // buffer alignment in the original source-target constructor.
    const float width=value<int16_t>(r,0x3c68)*value<float>(r,0x3c60);
    const float height=value<int16_t>(r,0x3c6a)*value<float>(r,0x3c64);
    if(!std::isfinite(width) || !std::isfinite(height) || width<1 || width>2048 || height<1 || height>2048)
        throw std::runtime_error(std::format("invalid native source frame dimensions {}x{}",width,height));
    for(unsigned i=0;i<2;++i){const auto *layer=r+i*0xa70;
        if(layer[0x1cf3] || layer[0x1cf4] || layer[0x1cf5]!=1 ||
           value<int16_t>(layer,0x1cfa)!=width || value<int16_t>(layer,0x1cfc)*2!=height)
            throw std::runtime_error(std::format("unported native source field view: layer={} target={}x{} field={}x{}",i,width,height,value<int16_t>(layer,0x1cfa),value<int16_t>(layer,0x1cfc)));
    }
    return {object,width,height};
}
}
NativeFrameTarget NativeScene::beginComposition(Runtime *rt,uint32_t stack,uint32_t object){
    if(!compositions.empty())throw std::runtime_error("native frame composition already active");
    const NativeFrameTarget layout=sourceTarget(rt,object);compositions.emplace(stack,layout);
    target=std::make_shared<NativeFrameTarget>(layout);
    if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:scene] native full-height target=%x %.0fx%.0f\n",object,layout.width,layout.height);
    return layout;
}
const NativeFrameTarget &NativeScene::composition(uint32_t stack)const{
    auto it=compositions.find(stack);if(it==compositions.end())throw std::runtime_error("native frame composition has no invocation");return it->second;
}
void NativeScene::finishComposition(uint32_t stack){
    if(compositions.erase(stack)!=1)throw std::runtime_error("native frame return has no invocation");
}
std::optional<bool> nativeCompositionInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x114550 && pc!=0x1145bc && pc!=0x114da0 && pc!=0x1145f8 && pc!=0x114634 && pc!=0x114714 && pc!=0x114a78 && pc!=0x114774 && pc!=0x1147ac)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<SceneOwner *>(&d->host()):nullptr;
    if(!owner || !c)throw std::runtime_error("native composition requires a scene owner");
    auto &scene=owner->nativeScene();const uint32_t sp=c->r[29].uw[0];
    if(pc==0x114550){scene.beginComposition(rt,sp-0xa0,c->r[4].uw[0]);return false;}
    const uint32_t object=scene.composition(sp).object;
    const auto current=sourceTarget(rt,object);
    if(current.width!=scene.composition(sp).width || current.height!=scene.composition(sp).height)
        throw std::runtime_error("native frame dimensions changed during composition");
    const uint32_t expectedObject=object+(pc==0x114da0?4*0xa70:0);
    if(c->r[22].uw[0]!=expectedObject)throw std::runtime_error("native frame source object changed during composition");
    // The previous frame's original wait must finish before resetting the
    // native renderer's building list. This point is reached once on resume.
    if(pc==0x1145bc){owner->beginNativeFrame(scene.composition(sp));return false;}
    if(pc==0x114da0){scene.finishComposition(sp);return false;}
    if(pc==0x1145f8 || pc==0x114a78){
        if(c->r[5].uw[0]!=4 || c->r[7].uw[0]!=object+(pc==0x1145f8?0x660:0x150))
            throw std::runtime_error("unowned native source-target environment");
    }else if(pc==0x114634){
        if(c->r[5].uw[0]!=2)throw std::runtime_error("unowned native field-copy producer");
    }else if(pc==0x114774 || pc==0x1147ac){
        const uint32_t layer=c->r[17].uw[0];
        if(layer!=object && layer!=object+0xa70)throw std::runtime_error("unowned native field view");
        if(c->r[5].uw[0]!=(pc==0x114774?4:2))throw std::runtime_error("unowned native field-view copy");
    }
    return false; // Original scalar delay-slot argument/load remains compiled.
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
    const bool frustumSite=(!(pc&3) && pc>=0x207cc8 && pc<=0x207d64) || pc==0x207d6c;
    if(pc!=0x208a94 && !frustumSite)return false;
    auto *dispatcher=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=dispatcher?dynamic_cast<SceneOwner *>(&dispatcher->host()):nullptr;
    if(!owner || !c)throw std::runtime_error("native camera publication requires a scene owner");
    if(frustumSite){
        if(c->pc!=0x207cc0)throw std::runtime_error("native frustum publication requires its complete owner");
        if(pc==0x207cc8){
            const auto camera=c->r[4].uw[0];
            if(c->r[5].uw[0]!=camera+0x460 || c->r[3].uw[0]!=camera+0x500)
                throw std::runtime_error("native frustum source changed during publication");
            owner->nativeScene().publishFrustum(rt,camera);
        }
        return true;
    }
    const uint32_t camera=c->r[17].uw[0];owner->nativeScene().publishCamera(rt,camera);
    // Retain the original scalar delay-slot load. The camera-only transport
    // enqueue was retired before a job exists; its completion method is empty.
    c->r[5].sd[0]=static_cast<int32_t>(word(rt,camera+0x8e0));return true;
}
void NativeScene::publishFrustum(Runtime *rt,uint32_t object){
    const auto *source=record(rt,object,0x5a0);
    auto view=std::make_shared<NativeCameraFrustum>();view->object=object;
    std::memcpy(view->points.data(),source+0x460,160);
    std::memcpy(view->normals.data(),source+0x500,160);
    activeFrustum=std::move(view);++frustumPublications;
}
void NativeScene::beginVisibility(uint32_t stack,uint32_t object){
    if(!visibilityInputs.emplace(stack,NativeVisibilityInputs{.scene=object}).second)
        throw std::runtime_error("native visibility invocation already active");
}
const NativeVisibilityInputs &NativeScene::visibility(uint32_t stack)const{
    auto it=visibilityInputs.find(stack);if(it==visibilityInputs.end())throw std::runtime_error("native visibility has no invocation");return it->second;
}
void NativeScene::visibilityClip(Runtime *rt,uint32_t stack,uint32_t camera){
    auto next=visibility(stack);
    if(next.camera || !activeFrustum || activeFrustum->object!=camera)
        throw std::runtime_error("native visibility clip has no matching frustum or repeats");
    std::memcpy(next.clip.data(),record(rt,camera,0x430)+0x3f0,64);
    next.camera=camera;next.frustum=activeFrustum;visibilityInputs.at(stack)=std::move(next);
}
void NativeScene::visibilityView(Runtime *rt,uint32_t stack,uint32_t camera){
    auto next=visibility(stack);
    if(next.camera!=camera || next.viewReady)throw std::runtime_error("native visibility view has no matching clip or repeats");
    std::memcpy(next.view.data(),record(rt,camera,0x330)+0x2f0,64);next.viewReady=true;visibilityInputs.at(stack)=std::move(next);
}
void NativeScene::finishVisibility(uint32_t stack,uint32_t object){
    if(visibility(stack).scene!=object)throw std::runtime_error("native visibility scene changed during invocation");
    visibilityInputs.erase(stack);
}
std::optional<bool> nativeVisibilityInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool clip=pc==0x220624 || pc==0x220628 || pc==0x22062c || pc==0x220630;
    const bool view=pc==0x2206b8 || pc==0x2206bc || pc==0x2206c0 || pc==0x2206c4 ||
                    pc==0x2206d0 || pc==0x2206d4 || pc==0x2206d8 || pc==0x2206dc || pc==0x2206e0;
    if(pc!=0x2205c0 && pc!=0x220ce0 && !clip && !view)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<SceneOwner *>(&d->host()):nullptr;
    if(!owner || !c)throw std::runtime_error("native visibility requires a scene owner");
    auto &scene=owner->nativeScene();const auto sp=c->r[29].uw[0];
    if(pc==0x2205c0){
        if((sp&15) || sp<0x130 || !rt->memory.hostPointer(sp-0x130,0x130))throw std::runtime_error("native visibility stack outside owned memory");
        scene.beginVisibility(sp-0x130,c->r[4].uw[0]);return false;
    }
    if(pc==0x220ce0){scene.finishVisibility(sp,c->r[21].uw[0]);return false;}
    if(scene.visibility(sp).scene!=c->r[21].uw[0])throw std::runtime_error("native visibility scene changed before capture");
    if(pc==0x220624)scene.visibilityClip(rt,sp,c->r[20].uw[0]);
    if(pc==0x2206b8)scene.visibilityView(rt,sp,c->r[20].uw[0]);
    return true;
}
}

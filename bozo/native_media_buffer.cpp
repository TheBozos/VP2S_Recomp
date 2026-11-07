#include "native_media_buffer.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
namespace vp2s {
namespace {
uint32_t word(Runtime *rt,uint32_t at){
    if(!rt || at<0x100000 || at>0x2000000-4)throw std::runtime_error("native media buffer metadata outside EE RAM");
    uint32_t v;std::memcpy(&v,rt->memory.hostPointer(at,4),4);return v;
}
void validate(Runtime *rt,uint32_t object){
    if((object&15) || object<0x100000 || object>0x2000000-0xce4f0 || word(rt,object+0xce4ec)!=0x159af0 ||
       word(rt,object+0xc9990)!=0x190 || word(rt,object+0xc99a0)!=0xc8000 || word(rt,object+0xce4e0)!=0x320 ||
       int32_t(word(rt,object+0xc99c8))<=0 || word(rt,object+0xc99cc)!=0x158ff8)
        throw std::runtime_error("unported native media buffer constructor/shape");
}
}
void NativeMediaBuffers::begin(Runtime *rt,uint32_t stack,uint32_t object){
    validate(rt,object);
    if(pending.contains(stack) || prepared.contains(object))throw std::runtime_error("native media buffer initialization repeats");
    for(const auto &[at,entry]:pending)if(entry->object==object)throw std::runtime_error("native media buffer has an active initializer");
    auto decoder=factory();if(!decoder)throw std::runtime_error("native media buffer has no native video decoder");
    auto entry=std::make_shared<NativeMediaBuffer>();entry->object=object;entry->capacity=0xc8000;
    entry->semaphore=word(rt,object+0xc99c8);entry->decoder=std::move(decoder);pending.emplace(stack,std::move(entry));
}
void NativeMediaBuffers::finish(Runtime *rt,uint32_t stack){
    const auto entry=pending.at(stack);validate(rt,entry->object);
    if(word(rt,entry->object+0xc99c8)!=entry->semaphore || word(rt,entry->object+0xc9994) || word(rt,entry->object+0xc9998) ||
       word(rt,entry->object+0xc999c) || word(rt,entry->object+0xc99d0)!=1 || word(rt,entry->object+0xce4e4) || word(rt,entry->object+0xce4e8))
        throw std::runtime_error("native media buffer did not complete its CPU reset");
    prepared.emplace(entry->object,entry);pending.erase(stack);
}
void NativeMediaBuffers::release(uint32_t object){
    for(const auto &[at,entry]:pending)if(entry->object==object)throw std::runtime_error("native media buffer released during initialization");
    prepared.erase(object);
}
void NativeMediaBuffers::preparePacing(Runtime *rt,uint32_t object,uint64_t fieldCycles){
    const auto entry=prepared.at(object);
    if(!fieldCycles || entry->pacingFieldCycles || word(rt,object+0xce4ec)!=0x159ac0 || word(rt,object+0x15e600) ||
       word(rt,object+0x15e604) || word(rt,object+0x15e608)!=0x90000)
        throw std::runtime_error("unported native movie pacing initialization");
    // A native field-duration contract for the later presentation scheduler.
    // No timer register, count or status is stored or returned to game code.
    entry->pacingFieldCycles=fieldCycles;
}
void NativeMediaBuffers::initialize(Runtime *rt,uint32_t parent){
    if(parent<0x100000 || parent>0x2000000-0x15e780 || word(rt,parent+0x15e77c)!=0x159bf8)
        throw std::runtime_error("native MPEG initialization has no typed movie constructor");
    auto entry=prepared.at(parent+0xc0);
    if(!entry->decoder || !entry->pacingFieldCycles || entry->initialized)throw std::runtime_error("native MPEG initialization escaped its prepared decoder lifetime");
    entry->initialized=true;
}
std::shared_ptr<const NativeMediaBuffer> NativeMediaBuffers::buffer(uint32_t object)const{
    const auto it=prepared.find(object);return it==prepared.end()?nullptr:it->second;
}
std::optional<bool> nativeMediaBufferInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x2bbfc0 && pc!=0x2bc1d0 && pc!=0x2bc1e0 && pc!=0x2ba670)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<MediaBufferOwner *>(&d->host()):nullptr;
    if(!owner || !c)throw std::runtime_error("native movie input requires a media-buffer owner");
    const auto sp=c->r[29].uw[0];
    if(pc==0x2ba670){
        if(c->pc!=0x2ba614 || c->r[2].uw[0]!=0x83)throw std::runtime_error("native movie pacing requires original initializer continuation");
        owner->nativeMediaBuffers().preparePacing(rt,c->r[16].uw[0],d->kernel().field());return true;
    }else if(pc==0x2bbfc0){
        if(c->pc!=pc || c->r[31].uw[0]!=0x2bc3e0 || sp<0x20 || (sp&15))throw std::runtime_error("unported native media-buffer reset caller");
        owner->nativeMediaBuffers().begin(rt,sp-0x20,c->r[4].uw[0]);
    }else if(pc==0x2bc1d0){
        owner->nativeMediaBuffers().finish(rt,sp);
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:media] native decoder owns prepared movie input\n");
    }
    else{
        if(c->pc!=pc)throw std::runtime_error("native media buffer release requires original destructor entry");
        owner->nativeMediaBuffers().release(c->r[4].uw[0]);
    }
    return false; // Original CPU initialization/return remains; no transport job exists.
}
bool nativeMediaLibrary(R5900Context *c,Runtime *rt,uint32_t target){
    if(target!=0x1f02a0)return false;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<MediaBufferOwner *>(&d->host()):nullptr;
    if(!owner || !c || c->pc!=target || c->r[31].uw[0]!=0x2be6dc)throw std::runtime_error("unported native MPEG initialization caller");
    owner->nativeMediaBuffers().initialize(rt,c->r[19].uw[0]);
    if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:media] native MPEG initializer ready\n");
    // This USA SDK initializer returns 1. The native decoder was actually
    // constructed by the input-buffer owner; no SDK/private/device state runs.
    c->r[2].sd[0]=1;c->pc=c->r[31].uw[0];return true;
}
}

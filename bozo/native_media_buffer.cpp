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
void validate(Runtime *rt,uint32_t object,uint32_t type=0x159af0){
    if((object&15) || object<0x100000 || object>0x2000000-0xce4f0 || word(rt,object+0xce4ec)!=type ||
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
    const auto entry=pending.at(stack);const auto existing=prepared.find(entry->object);
    validate(rt,entry->object,existing==prepared.end()?0x159af0:0x159ac0);
    if(existing!=prepared.end() && (existing->second!=entry || !entry->inputResetPending))throw std::runtime_error("native media input reset lost its decoder transition");
    if(word(rt,entry->object+0xc99c8)!=entry->semaphore || word(rt,entry->object+0xc9994) || word(rt,entry->object+0xc9998) ||
       word(rt,entry->object+0xc999c) || word(rt,entry->object+0xc99d0)!=1 || word(rt,entry->object+0xce4e4) || word(rt,entry->object+0xce4e8))
        throw std::runtime_error("native media buffer did not complete its CPU reset");
    prepared.emplace(entry->object,entry);entry->inputResetPending=false;pending.erase(stack);
}
void NativeMediaBuffers::beginInputReset(Runtime *rt,uint32_t stack,uint32_t object){
    validate(rt,object,0x159ac0);auto entry=prepared.at(object);
    if(!entry->created || !entry->decoder || !entry->inputResetPending || pending.contains(stack))throw std::runtime_error("native movie input reset has no completed decoder reset");
    for(const auto &[at,active]:pending)if(active==entry)throw std::runtime_error("native movie input reset repeats");
    pending.emplace(stack,std::move(entry));
}
void NativeMediaBuffers::release(uint32_t object){
    for(const auto &[at,entry]:pending)if(entry->object==object)throw std::runtime_error("native media buffer released during initialization");
    for(const auto &[at,entry]:creations)if(entry.owner->object==object)throw std::runtime_error("native media buffer released during stream creation");
    for(const auto &[at,entry]:resets)if(entry.owner->object==object)throw std::runtime_error("native media buffer released during decoder reset");
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
void NativeMediaBuffers::beginCreation(Runtime *rt,uint32_t stack,uint32_t object,uint32_t mp,uint32_t work,uint32_t bytes){
    auto entry=prepared.at(object);
    if(!entry->initialized || entry->created || creations.contains(stack) || mp!=object+0xce500 || work<0x100000 ||
       work>0x2000000 || bytes>0x2000000-work || mp>0x2000000-0x44 || word(rt,object+0xce4ec)!=0x159ac0)
        throw std::runtime_error("native MPEG creation escaped its decoder/work allocation");
    for(const auto &[at,creation]:creations)if(creation.owner==entry)throw std::runtime_error("native MPEG decoder has an active stream constructor");
    creations.emplace(stack,Creation{std::move(entry),mp,work,bytes});
}
void NativeMediaBuffers::creationEntry(uint32_t stack,uint32_t mp,uint32_t work,uint32_t bytes)const{
    const auto &entry=creations.at(stack);
    if(entry.mp!=mp || entry.work!=work || entry.bytes!=bytes)throw std::runtime_error("native MPEG constructor input changed before original entry");
}
void NativeMediaBuffers::finishCreation(Runtime *rt,uint32_t stack,uint32_t result){
    const auto entry=creations.at(stack);
    if(result>1)throw std::runtime_error("native MPEG constructor returned an unknown result");
    if(result){
        const auto aligned=(entry.work+3)&~3u;
        if(aligned<entry.work || aligned-entry.work>entry.bytes || word(rt,entry.mp+0x40)!=aligned ||
           word(rt,entry.mp) || word(rt,entry.mp+4) || word(rt,entry.mp+8) || (aligned&63))
            throw std::runtime_error("native MPEG constructor did not publish its original CPU records");
        entry.owner->mp=entry.mp;entry.owner->work=aligned;entry.owner->workBytes=entry.bytes-(aligned-entry.work);entry.owner->created=true;
    }
    creations.erase(stack);
}
std::shared_ptr<NativeMediaBuffer> NativeMediaBuffers::stream(Runtime *rt,uint32_t mp)const{
    if(mp<0xce500)throw std::runtime_error("native MPEG stream pointer outside owner range");
    auto entry=prepared.at(mp-0xce500);
    if(!entry->initialized || !entry->created || entry->inputResetPending || entry->mp!=mp || word(rt,mp+0x40)!=entry->work)
        throw std::runtime_error("native MPEG stream has no live original work record");
    for(const auto &[at,reset]:resets)if(reset.owner==entry)throw std::runtime_error("native MPEG stream is resetting");
    return entry;
}
void NativeMediaBuffers::validateCallback(Runtime *rt,uint32_t mp,uint32_t type,uint32_t data)const{
    const auto entry=stream(rt,mp);
    if(type>=entry->callbacks.size() || data!=entry->object)throw std::runtime_error("unported native MPEG callback owner/type");
}
void NativeMediaBuffers::captureCallback(Runtime *rt,uint32_t mp,uint32_t type){
    auto entry=stream(rt,mp);
    if(type>=entry->callbacks.size())throw std::runtime_error("native MPEG callback type outside range");
    const auto at=entry->work+type*12+0xc;
    entry->callbacks[type]={word(rt,at),word(rt,at+4),word(rt,at+8),true};
}
void NativeMediaBuffers::validateStreamRegistration(Runtime *rt,uint32_t mp)const{
    const auto entry=stream(rt,mp);const auto pool=word(rt,entry->work+0x60),count=word(rt,entry->work+0x64);
    if(count>=64 || pool<entry->work || uint64_t(pool)+0x800>uint64_t(entry->work)+entry->workBytes)
        throw std::runtime_error("native MPEG stream-handler pool escaped its original allocation");
}
void NativeMediaBuffers::captureStreams(Runtime *rt,uint32_t mp){
    auto entry=stream(rt,mp);const auto pool=word(rt,entry->work+0x60),count=word(rt,entry->work+0x64);
    if(count>64 || pool<entry->work || uint64_t(pool)+uint64_t(count)*32>uint64_t(entry->work)+entry->workBytes)
        throw std::runtime_error("native MPEG stream-handler results exceed original allocation");
    std::vector<NativeMediaBuffer::StreamCallback> result;
    for(uint32_t i=0;i<count;++i){const auto at=pool+i*32;
        const auto wide=[&](uint32_t address){return uint64_t(word(rt,address))|(uint64_t(word(rt,address+4))<<32);};
        result.push_back({wide(at),wide(at+8),{word(rt,at+0x10),word(rt,at+0x14),word(rt,at+0x18),true}});
    }
    entry->streams=std::move(result);
}
void NativeMediaBuffers::beginReset(Runtime *rt,uint32_t stack,uint32_t mp){
    auto entry=stream(rt,mp);
    if(resets.contains(stack))throw std::runtime_error("native MPEG reset stack repeats");
    auto decoder=factory();if(!decoder)throw std::runtime_error("native MPEG reset has no new decoder");
    resets.emplace(stack,Reset{std::move(entry),std::move(decoder)});
}
void NativeMediaBuffers::resetEntry(uint32_t stack,uint32_t mp)const{
    if(resets.at(stack).owner->mp!=mp)throw std::runtime_error("native MPEG reset entry changed its stream");
}
void NativeMediaBuffers::resetHelper(uint32_t stack,uint32_t work)const{
    if(resets.at(stack).owner->work!=work)throw std::runtime_error("native MPEG reset helper changed its work record");
}
void NativeMediaBuffers::finishReset(Runtime *rt,uint32_t stack){
    const auto &reset=resets.at(stack);const auto entry=reset.owner;
    for(uint32_t offset:{0u,4u,8u,0xc0u,0x128u,0x858u,0x878u,0x1c0u})
        if(word(rt,entry->work+offset))throw std::runtime_error("native MPEG reset left an original CPU state record active");
    if(word(rt,entry->mp+8) || word(rt,entry->work+0x98)!=0xffffffff || word(rt,entry->work+0x828)!=1)
        throw std::runtime_error("native MPEG reset did not finish its original software initialization");
    entry->decoder=std::move(resets.at(stack).decoder);entry->inputResetPending=true;resets.erase(stack);
}
std::shared_ptr<const NativeMediaBuffer> NativeMediaBuffers::buffer(uint32_t object)const{
    const auto it=prepared.find(object);return it==prepared.end()?nullptr:it->second;
}
std::optional<bool> nativeMediaBufferInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x2bbfc0 && pc!=0x2bc1d0 && pc!=0x2bc1e0 && pc!=0x2ba670 && pc!=0x1ef6f8 && pc!=0x1ef914 && pc!=0x1f0438 && pc!=0x1f04a0 && pc!=0x1efa30)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<MediaBufferOwner *>(&d->host()):nullptr;
    if(!owner || !c)throw std::runtime_error("native movie input requires a media-buffer owner");
    const auto sp=c->r[29].uw[0];
    if(pc==0x1f0438){
        if(c->pc!=pc || sp<0x20 || (sp&15))throw std::runtime_error("native MPEG reset requires complete original entry");
        owner->nativeMediaBuffers().resetEntry(sp-0x20,c->r[4].uw[0]);
    }else if(pc==0x1efa30){
        if(c->r[31].uw[0]!=0x1f0474)throw std::runtime_error("unported native MPEG reset helper caller");
        owner->nativeMediaBuffers().resetHelper(sp,c->r[4].uw[0]);
    }else if(pc==0x1f04a0){
        if(c->r[2].uw[0]!=1)throw std::runtime_error("native MPEG reset returned an unknown result");
        owner->nativeMediaBuffers().finishReset(rt,sp);
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:media] native decoder reset completed\n");
    }else if(pc==0x1ef6f8){
        if(c->pc!=pc || sp<0x50 || (sp&15))throw std::runtime_error("native MPEG creation requires complete original entry");
        owner->nativeMediaBuffers().creationEntry(sp-0x50,c->r[4].uw[0],c->r[5].uw[0],c->r[6].uw[0]);
    }else if(pc==0x1ef914){
        owner->nativeMediaBuffers().finishCreation(rt,sp,c->r[2].uw[0]);
        if(c->r[2].uw[0] && std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:media] native decoder owns original MPEG software records\n");
    }
    else if(pc==0x2ba670){
        if(c->pc!=0x2ba614 || c->r[2].uw[0]!=0x83)throw std::runtime_error("native movie pacing requires original initializer continuation");
        owner->nativeMediaBuffers().preparePacing(rt,c->r[16].uw[0],d->kernel().field());return true;
    }else if(pc==0x2bbfc0){
        if(c->pc!=pc || sp<0x20 || (sp&15))throw std::runtime_error("unported native media-buffer reset caller");
        if(c->r[31].uw[0]==0x2bc3e0)owner->nativeMediaBuffers().begin(rt,sp-0x20,c->r[4].uw[0]);
        else if(c->r[31].uw[0]==0x2b9bb4)owner->nativeMediaBuffers().beginInputReset(rt,sp-0x20,c->r[4].uw[0]);
        else throw std::runtime_error("unported native media-buffer reset caller");
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
    if(target!=0x1f02a0 && target!=0x1ef6f8 && target!=0x1f0510 && target!=0x1f4990 && target!=0x1f0438)return false;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<MediaBufferOwner *>(&d->host()):nullptr;
    if(!owner || !c || c->pc!=target)throw std::runtime_error("native MPEG requires its declared API entry");
    if(target==0x1ef6f8){
        const auto sp=c->r[29].uw[0];auto original=rt->functions.generated(target);
        if(c->r[31].uw[0]!=0x2ba524 || sp<0x50 || (sp&15) || !original)throw std::runtime_error("unported native MPEG creation caller");
        owner->nativeMediaBuffers().beginCreation(rt,sp-0x50,c->r[17].uw[0],c->r[4].uw[0],c->r[5].uw[0],c->r[6].uw[0]);
        original(rt->memory.flat(),c,rt);return true;
    }
    if(target==0x1f0438){
        const auto sp=c->r[29].uw[0];auto original=rt->functions.generated(target);
        if(c->r[31].uw[0]!=0x2b9ba0 || sp<0x20 || (sp&15) || !original)throw std::runtime_error("unported native MPEG reset caller");
        owner->nativeMediaBuffers().beginReset(rt,sp-0x20,c->r[4].uw[0]);original(rt->memory.flat(),c,rt);return true;
    }
    if(target==0x1f0510){
        const auto mp=c->r[4].uw[0],type=c->r[5].uw[0];
        constexpr uint32_t returns[6]={0x2ba544,0x2ba564,0x2ba584,0x2ba5a4,0x2ba5e4,0x2ba5c4};
        constexpr uint32_t functions[6]={0x2b9550,0x2b9330,0x2b9300,0x2b92e0,0x2b9320,0x2b92a0};
        auto original=rt->functions.generated(target);
        if(type>=6 || c->r[31].uw[0]!=returns[type] || c->r[6].uw[0]!=functions[type] || !original)
            throw std::runtime_error("unported native MPEG callback-registration caller/function");
        owner->nativeMediaBuffers().validateCallback(rt,mp,type,c->r[7].uw[0]);
        original(rt->memory.flat(),c,rt);
        if(c->pc!=returns[type])throw std::runtime_error("native MPEG scalar callback registration did not return");
        owner->nativeMediaBuffers().captureCallback(rt,mp,type);
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:media] native callback type=%u function=%x\n",type,functions[type]);
        return true;
    }
    if(target==0x1f4990){
        const auto mp=c->r[4].uw[0],type=c->r[5].uw[0],channel=c->r[6].uw[0];
        if(mp<0xce500 || c->r[31].uw[0]!=0x2ba3c8)throw std::runtime_error("unported native MPEG stream wrapper");
        const auto object=mp-0xce500;const auto sp=c->r[29].uw[0];const auto *saved=rt->memory.hostPointer(sp,8);uint64_t caller=0;
        if((sp&15) || !saved)throw std::runtime_error("native MPEG stream wrapper has no bounded original stack");std::memcpy(&caller,saved,8);
        unsigned slot;
        if(type==0 && channel==0)slot=0;else if(type==4 && channel<=1)slot=1+channel;
        else throw std::runtime_error("unported native MPEG stream type/channel");
        constexpr uint32_t callers[3]={0x2be738,0x2be75c,0x2be780},functions[3]={0x2b90a0,0x2bdac0,0x2bd9b0};
        auto original=rt->functions.generated(target);
        if(caller!=callers[slot] || c->r[7].uw[0]!=functions[slot] || c->r[8].uw[0]!=object+(slot?0x15e680:0) || !original)
            throw std::runtime_error("unported native MPEG stream callback provenance");
        owner->nativeMediaBuffers().validateStreamRegistration(rt,mp);original(rt->memory.flat(),c,rt);
        if(c->pc!=0x2ba3c8)throw std::runtime_error("native MPEG scalar stream registration did not return");
        owner->nativeMediaBuffers().captureStreams(rt,mp);
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:media] native stream type=%u channel=%u\n",type,channel);
        return true;
    }
    if(c->r[31].uw[0]!=0x2be6dc)throw std::runtime_error("unported native MPEG initialization caller");
    owner->nativeMediaBuffers().initialize(rt,c->r[19].uw[0]);
    if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:media] native MPEG initializer ready\n");
    // This USA SDK initializer returns 1. The native decoder was actually
    // constructed by the input-buffer owner; no SDK/private/device state runs.
    c->r[2].sd[0]=1;c->pc=c->r[31].uw[0];return true;
}
}

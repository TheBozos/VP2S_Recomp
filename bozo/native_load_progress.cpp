#include "native_load_progress.h"
#include "native_seed.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <limits>
#include <stdexcept>
namespace vp2s {
namespace {
uint32_t word(Runtime *rt,uint32_t at){
    if(!rt || at<0x100000 || uint64_t(at)+4>0x2000000)throw std::runtime_error("native loading metadata outside EE RAM");
    uint32_t v;std::memcpy(&v,rt->memory.hostPointer(at,4),4);return v;
}
void validate(Runtime *rt,uint32_t stack,uint32_t worker,uint32_t destination){
    word(rt,stack);word(rt,worker);word(rt,destination);
    if((stack&15) || (destination&127))throw std::runtime_error("native loading request has unaligned owned buffers");
}
}
void NativeLoadProgress::begin(Runtime *rt,uint32_t stack,uint32_t worker,uint32_t destination,uint64_t now,uint64_t period){
    validate(rt,stack,worker,destination);
    if(pending.contains(stack) || !period || now>std::numeric_limits<uint64_t>::max()-period)throw std::runtime_error("native loading schedule is repeated or out of range");
    pending.emplace(stack,Pending{worker,destination,now,now+period,period});
}
bool NativeLoadProgress::updateDue(Runtime *rt,uint32_t stack,uint32_t worker,uint32_t destination,uint64_t now){
    validate(rt,stack,worker,destination);auto &p=pending.at(stack);
    if(p.worker!=worker || p.destination!=destination || now<p.last || now>std::numeric_limits<uint64_t>::max()-p.period)
        throw std::runtime_error("native loading schedule lost request/clock ownership");
    p.last=now;if(now<p.next)return false;p.next=now+p.period;return true;
}
void NativeLoadProgress::finish(uint32_t stack,uint32_t worker,uint32_t destination,bool requestCompleted){
    const auto &p=pending.at(stack);
    if(!requestCompleted || p.worker!=worker || p.destination!=destination)throw std::runtime_error("native loading request has not actually completed");
    pending.erase(stack);
}
std::optional<bool> nativeLoadProgressInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x306a10 && pc!=0x306b84 && pc!=0x306bc4 && pc!=0x306c58 && pc!=0x30ce84)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<LoadProgressOwner *>(&d->host()):nullptr;
    auto *seed=d?dynamic_cast<SeedOwner *>(&d->host()):nullptr;
    if(!c || !owner || !seed)throw std::runtime_error("native loading requires per-run request/entropy owners");
    if(pc==0x306a10){
        if(word(rt,0x35da00)!=c->r[16].uw[0])throw std::runtime_error("native allocation entropy escaped original scalar generator");
        // The old timer mixed only into the sixteen-way allocation placement.
        // Publish that semantic choice directly; never manufacture a count.
        c->r[17].sd[0]=int32_t(seed->initializationJitter()&15u);return true;
    }
    if(pc==0x30ce84){
        const auto base=c->r[16].uw[0];
        if(base<0x100000 || (base&3) || uint64_t(base)+0x18c>0x2000000)throw std::runtime_error("native record placement outside its owned object");
        // A 0x78-byte payload fits at any of the sixteen original offsets,
        // before the object's pointer field at +0x188. The CPU stores/clears it.
        c->r[2].sd[0]=int32_t(base+16*(seed->initializationJitter()&15u));return true;
    }
    const auto stack=c->r[29].uw[0],worker=word(rt,0x35dd90),destination=c->r[16].uw[0];
    auto &progress=owner->nativeLoadProgress();
    if(pc==0x306b84)progress.begin(rt,stack,worker,destination,d->now(c),d->kernel().field());
    else if(pc==0x306bc4){
        // A software UI cadence on the engine's native clock, at most once per
        // field. The original file-request poll and UI callbacks remain compiled.
        c->r[1].ud[0]=progress.updateDue(rt,stack,worker,destination,d->now(c))?0:1;
    }else progress.finish(stack,worker,destination,c->r[2].uw[0]==0);
    return false;
}
}

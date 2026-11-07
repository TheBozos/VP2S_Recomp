#include "native_state_setup.h"
#include "native_assets.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <stdexcept>
namespace vp2s {
namespace {
uint32_t word(Runtime *rt,uint32_t at){
    at=NativeAssets::address(at);
    if(at>0x2000000-4)throw std::runtime_error("native setup metadata outside EE RAM");
    uint32_t v;std::memcpy(&v,rt->memory.hostPointer(at,4),4);return v;
}
constexpr uint32_t writer=0x35df10;
}
void NativeStateSetups::beginEmptyChildren(Runtime *rt,uint32_t stack,uint32_t object){
    if(active(stack))throw std::runtime_error("empty child publication stack reused");
    if(word(rt,object)!=0x1653d0 || word(rt,object+0x98)!=object+0x90)
        throw std::runtime_error("populated/unowned child writer requires native geometry");
    const auto *p=rt->memory.hostPointer(object,0x118);if(!p || !p[0x63] || p[0x64]!=1 || p[0x65]>1)
        throw std::runtime_error("empty child writer was not built/pending");
    const uint32_t scene=word(rt,0x35ddf8),layer=p[0x65];
    if(word(rt,scene+0x40a8+4*layer)!=object || !*rt->memory.hostPointer(scene+0x40dc+layer,1))
        throw std::runtime_error("empty child writer has no pending scene owner");
    const auto buffer=word(rt,object+0x114),base=word(rt,buffer),begin=word(rt,buffer+12),end=word(rt,buffer+8);
    // The empty builder writes exactly one state entry through 0x1206b0.
    // Native materials already own it. No packet words are read.
    if(uint64_t(base)+0x40!=begin || uint64_t(begin)+32!=end || uint64_t(end)+16>uint64_t(base)+uint64_t(word(rt,buffer+16))*8)
        throw std::runtime_error("empty child writer has additional/unowned contents");
    invocations.emplace(stack,Proof{begin,end,buffer,object});
}
void NativeStateSetups::beginDefaults(Runtime *rt,uint32_t stack){
    if(active(stack))throw std::runtime_error("active native defaults invocation stack reused");
    const uint32_t base=word(rt,writer),begin=word(rt,writer+12),end=word(rt,writer+8);
    const uint32_t physical=NativeAssets::address(base);
    const uint64_t capacity=uint64_t(base)+uint64_t(word(rt,writer+16))*8;
    if(physical<0x100000 || physical>0x2000000-0x110 ||
       uint64_t(base)+0x40!=begin || uint64_t(begin)+0xc0!=end || uint64_t(end)+16>capacity)
        throw std::runtime_error("native defaults writer has unowned contents");
    invocations.emplace(stack,Proof{begin,end});
}
void NativeStateSetups::retire(Runtime *rt,R5900Context *c){
    auto it=invocations.find(c->r[29].uw[0]);
    if(it==invocations.end() || it->second.retired)throw std::runtime_error("native defaults publication has no invocation");
    const auto &p=it->second;
    if(c->r[17].uw[0]!=p.writer || c->r[16].uw[0]!=p.end+16 || c->r[2].uw[0]!=p.begin ||
       c->r[5].uw[0]!=2 || c->r[6].uw[0] || c->r[8].uw[0]!=(p.end+16-p.begin)/16 ||
       (p.object && c->r[9].uw[0]!=p.object+0x108) ||
       word(rt,p.writer+8)!=p.end || word(rt,p.writer+12)!=p.begin)
        throw std::runtime_error("native defaults publication changed during CPU flush");
    it->second.retired=true;
}
void NativeStateSetups::returned(Runtime *rt,uint32_t stack){
    auto it=invocations.find(stack);if(it==invocations.end() || !it->second.retired)throw std::runtime_error("native state writer returned without retirement");
    auto &p=it->second;if(!p.object){finish(stack);return;}
    if(p.flushed || word(rt,p.writer+8)!=p.end+16 || word(rt,p.writer+12)!=p.end+16)
        throw std::runtime_error("empty child writer did not finish scalar bookkeeping");
    p.flushed=true;
}
uint32_t NativeStateSetups::completeEmptyChildren(Runtime *rt,uint32_t stack){
    auto it=invocations.find(stack);if(it==invocations.end() || !it->second.object || !it->second.flushed)
        throw std::runtime_error("empty child completion before its flush");
    const auto object=it->second.object;
    if(word(rt,object)!=0x1653d0 || word(rt,object+0x98)!=object+0x90 || word(rt,object+0x114)!=it->second.writer)
        throw std::runtime_error("empty child publication changed during flush");
    invocations.erase(it);return object;
}
void NativeStateSetups::finish(uint32_t stack){
    auto it=invocations.find(stack);
    if(it==invocations.end() || !it->second.retired)throw std::runtime_error("native defaults return without producer retirement");
    invocations.erase(it);
}
std::optional<bool> nativeStateSetupInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x29caf8 && pc!=0x2075bc && pc!=0x2075cc && pc!=0x120f4c && pc!=0x120f58)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<StateSetupOwner *>(&d->host()):nullptr;
    if(!owner || !c){if(pc==0x120f4c || pc==0x120f58)return std::nullopt;throw std::runtime_error("native state publication requires an owner");}
    auto &setups=owner->nativeStateSetups();const uint32_t sp=c->r[29].uw[0];
    if(pc==0x29caf8)setups.beginDefaults(rt,sp-0x30);
    else if(pc==0x2075bc)setups.beginEmptyChildren(rt,sp-0x30,c->r[4].uw[0]);
    else if(pc==0x2075cc)c->r[4].sd[0]=int32_t(setups.completeEmptyChildren(rt,sp-0x30));
    else if(!setups.active(sp))return std::nullopt;
    else if(pc==0x120f4c)setups.retire(rt,c);
    else setups.returned(rt,sp);
    return false; // Preserve the compiled scalar instruction, not a transport.
}
}

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
    if(c->r[17].uw[0]!=writer || c->r[16].uw[0]!=p.end+16 || c->r[2].uw[0]!=p.begin ||
       c->r[5].uw[0]!=2 || c->r[6].uw[0] || c->r[8].uw[0]!=13 ||
       word(rt,writer+8)!=p.end || word(rt,writer+12)!=p.begin)
        throw std::runtime_error("native defaults publication changed during CPU flush");
    it->second.retired=true;
}
void NativeStateSetups::finish(uint32_t stack){
    auto it=invocations.find(stack);
    if(it==invocations.end() || !it->second.retired)throw std::runtime_error("native defaults return without producer retirement");
    invocations.erase(it);
}
std::optional<bool> nativeStateSetupInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x29caf8 && pc!=0x120f4c && pc!=0x120f58)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<StateSetupOwner *>(&d->host()):nullptr;
    if(!owner || !c){if(pc!=0x29caf8)return std::nullopt;throw std::runtime_error("native defaults require an owner");}
    auto &setups=owner->nativeStateSetups();const uint32_t sp=c->r[29].uw[0];
    if(pc==0x29caf8)setups.beginDefaults(rt,sp-0x30);
    else if(!setups.active(sp))return std::nullopt;
    else if(pc==0x120f4c)setups.retire(rt,c);
    else setups.finish(sp);
    return false; // Preserve the compiled scalar instruction, not a transport.
}
}

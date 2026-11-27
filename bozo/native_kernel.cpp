#include "native_cast.h"
#include "native_kernel.h"
#include "native_assets.h"
#include "bozo/ee/dispatcher.h"
#include <cstdio>
#include <cstring>
#include <format>
#include <stdexcept>
#include <atomic>

namespace vp2s {
std::optional<NativeWorkStackSite> nativeWorkStackSite(uint32_t pc){
    static constexpr NativeWorkStackSite sites[]={{0x2bf040,0x2bf058,17},{0x47ddb8,0x47ddc4,17},{0x3d740c,0x3d7410,16},{0x3fc3bc,0x3fc3c0,16},
        {0x3fcacc,0x3fcad0,16},{0x3fd09c,0x3fd0a0,16},{0x42499c,0x4249a0,16},{0x424eac,0x424eb0,16},
        {0x5342fc,0x534300,16}};
    for(const auto &s:sites)if(pc==s.entry || pc==s.back)return s;
    return std::nullopt;
}
void nativeWorkStack(R5900Context *c,Runtime *rt,uint32_t pc){
    auto *d=rt?vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services):nullptr;
    auto *t=d?d->kernel().thread(d->kernel().running()):nullptr;
    if(!c || !t || t->pseudo || &t->top().ctx!=c)
        throw std::runtime_error("native work-stack handoff requires the running game frame");
    // The game's scratchpad work stack: saved = sp; t0 = 70003ff0; `move sp, t0` (entry); the
    // call; `move sp, saved` (return). mod001 2bf040/2bf058 (s1); mod028 47ddb8/47ddc4 (s1,
    // around the battle's map pass 47cdf0) and six with s0 whose entry is the call's delay slot
    // (3d740c, 3fc3bc, 3fcacc, 3fd09c, 42499c, 424eac, and mod164's 5342fc for battles 0xba / 0xbb, each returning at
    // the next word). The
    // mod028 words are checked (other overlays share the addresses).
    const auto site=nativeWorkStackSite(pc);
    if(!site)throw std::runtime_error("unqualified native work-stack site");
    const bool entry=pc==site->entry;const unsigned saved=site->saved;
    if(site->entry!=0x2bf040){uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
        if(word!=(entry?0x0100e821u:(saved==17?0x0220e821u:0x0200e821u))){
            if(site->entry==0x5342fc)return;   // another module in slot 0x532200: not this switch
            throw std::runtime_error("native work-stack site differs from the USA battle one");}}
    if(entry){
        if(c->r[8].uw[0]!=0x70003ff0 || c->r[saved].uw[0]!=c->r[29].uw[0])
            throw std::runtime_error("native work-stack entry does not match original saved SP");
        d->kernel().parkThreadStack(c->r[29].uw[0]);
    }else{
        if(c->r[29].uw[0]<0x70000000 || c->r[29].uw[0]>0x70003ff0)
            throw std::runtime_error("native work-stack return is outside its work frame");
        d->kernel().resumeThreadStack(c->r[saved].uw[0]);
    }
}
// libc memmove (132d38): the bytes move as the original moves them (overlap allowed), and
// their asset provenance moves with them, as memcpy's (132c88) does. The battle copies a
// FIS file with it before uploading it (mod028 4f2f70).
void nativeMemmove(uint8_t *,R5900Context *ctx,Runtime *rt){
    auto *d=rt?vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services):nullptr;
    auto *assets=d?vp2s::nativeCast<AssetOwner>(&d->host()):nullptr;
    if(!ctx || !assets || ctx->pc!=0x132d38u)throw std::runtime_error("native memmove requires its asset owner");
    const uint32_t to=ctx->r[4].uw[0],from=ctx->r[5].uw[0],bytes=ctx->r[6].uw[0];
    if(bytes){
        if(bytes>0x2000000u)throw std::runtime_error(std::format("native memmove of {:x} bytes (ra {:x})",bytes,ctx->r[31].uw[0]));
        auto *destination=rt->memory.hostPointer(to,bytes);const auto *source=rt->memory.hostPointer(from,bytes);
        if(!destination || !source)throw std::runtime_error(std::format("native memmove outside guest memory ({:x} <- {:x}, {:x})",to,from,bytes));
        assets->nativeAssets().copy(to,from,bytes,0);std::memmove(destination,source,bytes);
    }
    ctx->r[2].sd[0]=int32_t(to);ctx->pc=ctx->r[31].uw[0];
}
void validateNativeKernelCall(const R5900Context *ctx,uint32_t kind){
    if(kind!=3)return; // Bound SDK calls keep their image-specific registry guards.
    if(!ctx || (ctx->r[3].uw[0]!=60 && ctx->r[3].uw[0]!=61))
        throw std::runtime_error("unqualified raw VP2S syscall requires a native owner");
}
void nativeCacheSync(uint8_t *,R5900Context *ctx,Runtime *rt){
    auto *d=rt?vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services):nullptr;
    if(!ctx || ctx->pc!=0x137a30 || !d || d->kernel().running()<=0)
        throw std::runtime_error("native cache visibility requires a running native context");
    // _sceSDC only scans physical PS2 cache tags and drains cache lines. Game
    // buffers already live in shared host memory; native job copies use those
    // bytes directly. Supply host visibility, without tags/cache/CP0 state.
    std::atomic_thread_fence(std::memory_order_seq_cst);
    ctx->pc=ctx->r[31].uw[0];
}
namespace {
void rejectFirmwareEntry(uint8_t *,R5900Context *ctx,Runtime *rt) {
    uint32_t active=0,dispatching=0;
    if(rt) { std::memcpy(&active,rt->memory.ram()+0x14a250,4);std::memcpy(&dispatching,rt->memory.ram()+0x14a254,4); }
    throw std::runtime_error(std::format("unported SDK kernel/timer entry at {:x} ra={:x} active={:x} dispatching={:x}; native replacement required",ctx->pc,ctx->r[31].uw[0],active,dispatching));
}
}
void initNativeSyscallServices(uint8_t *,R5900Context *ctx,Runtime *rt) {
    auto *dispatcher=rt ? vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services) : nullptr;
    if(!dispatcher || !ctx || ctx->pc!=0x141448u ||
       dispatcher->kernel().running()<=0 ||
       !dispatcher->isKernelCall(0x1370d0u) || // GetThreadId
       !dispatcher->isKernelCall(0x1371e0u) || // CreateSema
       !dispatcher->isKernelCall(0x137460u))   // FlushCache
        throw std::runtime_error("native syscall bootstrap requires a booted native kernel and bound SDK services");
    // The original routine's sole persistent output is a firmware dispatch-table
    // address. No such address exists in the native ABI. Its lookup consumers
    // reject explicitly; retain RAM, registers, clocks and native kernel state.
    std::fprintf(stderr,"[vp2s:bozo] native syscall services ready; retired SDK firmware discovery\n");
    ctx->pc=ctx->r[31].uw[0];
}
void retireAlarmFirmwareInstaller(uint8_t *,R5900Context *ctx,Runtime *rt) {
    auto *dispatcher=rt ? vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services) : nullptr;
    if(!dispatcher || !ctx || ctx->pc!=0x141a38u ||
       dispatcher->kernel().running()<=0 || !dispatcher->kernel().field() ||
       !dispatcher->isKernelCall(0x137460u))
        throw std::runtime_error("SDK alarm installer retirement requires a booted native kernel/clock and FlushCache binding");
    // The original only installs a PS2 kernel patch and configures timer 3.
    // No game-memory output is needed by this startup caller. Do not pretend
    // that the engine's placeholder alarm enums implement a deadline service:
    // runtime requests and the original patch payload still reject explicitly.
    std::fprintf(stderr,"[vp2s:bozo] retired SDK alarm firmware installer; runtime alarm APIs remain unsupported\n");
    ctx->pc=ctx->r[31].uw[0];
}
void installNativeKernelBindings(bozo::port::Registry &registry) {
    for(uint32_t pc=0x13a3f0u;pc<0x13a524u;pc+=4) registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x137a30u;pc<0x137ad4u;pc+=4) registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x141410u;pc<0x141444u;pc+=4)
        registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x141448u;pc<0x141544u;pc+=4)
        registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x141a38u;pc<0x141b0cu;pc+=4)
        registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x149a90u;pc<0x14a1d0u;pc+=4)
        registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x141bb0u;pc<0x141d04u;pc+=4)
        registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x142150u;pc<0x1423f4u;pc+=4)
        registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x141df0u;pc<0x141e80u;pc+=4) registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x1423f8u;pc<0x142444u;pc+=4) registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x141f00u;pc<0x142090u;pc+=4) registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x140b70u;pc<0x140c34u;pc+=4) registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x148c58u;pc<0x148f88u;pc+=4) registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x141780u;pc<0x141898u;pc+=4) registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x141910u;pc<0x141958u;pc+=4) registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x141980u;pc<0x1419ccu;pc+=4) registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x1401b8u;pc<0x1403dcu;pc+=4) registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x13fba8u;pc<0x13fdb0u;pc+=4) registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x13f608u;pc<0x13f69cu;pc+=4) registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x137580u;pc<0x1375c0u;pc+=4) registry.registerFunction(pc,&rejectFirmwareEntry);
    registry.registerFunction(0x141448u,&initNativeSyscallServices);
    registry.registerFunction(0x132d38u,&nativeMemmove);
    for(uint32_t pc=0x132d3cu;pc<0x132e40u;pc+=4) registry.registerFunction(pc,&rejectFirmwareEntry);
    registry.registerFunction(0x141a38u,&retireAlarmFirmwareInstaller);
    registry.registerFunction(0x141bb0u,&initNativeTimer);
    registry.registerFunction(0x141df0u,&startNativeTimer);
    registry.registerFunction(0x1423f8u,&queryNativeTimer);
    registry.registerFunction(0x141f00u,&rearmNativeTimer);
    registry.registerFunction(0x142150u,&dispatchNativeTimer);
    registry.registerFunction(0x1401b8u,&loadNativeModule);
    registry.registerFunction(0x13fba8u,&loadNativeModule);
    registry.registerFunction(0x1375a0u,&nativeModuleTransfer);
    registry.registerFunction(0x1375b0u,&nativeModuleTransfer);
    registry.registerFunction(0x137580u,&nativeModuleTransferStatus);
    registry.registerFunction(0x137590u,&nativeModuleTransferStatus);
    registry.registerFunction(0x13f608u,&freeNativeModuleBuffer);
    registry.registerFunction(0x13f680u,&freeNativeModuleBuffer);
    registry.registerFunction(0x137280u,&nativeOsdSettings);
    registry.registerFunction(0x137290u,&nativeOsdSettings);
    registry.registerFunction(0x140b70u,&retireEventFirmwareInstaller);
    registry.registerFunction(0x137a30u,&nativeCacheSync);
    registry.registerFunction(0x13a3f0u,&sendNativeCommand);
}
void retireEventFirmwareInstaller(uint8_t *,R5900Context *ctx,Runtime *rt) {
    auto *d=rt ? vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services) : nullptr;
    if(!d || !ctx || ctx->pc!=0x140b70 || d->kernel().running()<=0)
        throw std::runtime_error("event SDK firmware retirement requires a booted native kernel");
    // The installer copies kernel code, replaces private syscall slots and
    // retains a kernel-object address. Native callers must use kernel services;
    // the original address-query consumers are rejecting boundaries above.
    std::fprintf(stderr,"[vp2s:bozo] retired SDK event firmware installer; kernel-address consumers remain unsupported\n");
    ctx->pc=ctx->r[31].uw[0];
}
}

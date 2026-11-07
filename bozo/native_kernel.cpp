#include "native_kernel.h"
#include "bozo/ee/dispatcher.h"
#include <cstdio>
#include <cstring>
#include <format>
#include <stdexcept>
#include <atomic>

namespace vp2s {
void validateNativeKernelCall(const R5900Context *ctx,uint32_t kind){
    if(kind!=3)return; // Bound SDK calls keep their image-specific registry guards.
    if(!ctx || (ctx->r[3].uw[0]!=60 && ctx->r[3].uw[0]!=61))
        throw std::runtime_error("unqualified raw VP2S syscall requires a native owner");
}
void nativeCacheSync(uint8_t *,R5900Context *ctx,Runtime *rt){
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
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
    auto *dispatcher=rt ? dynamic_cast<bozo::ee::Dispatcher *>(rt->services) : nullptr;
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
    auto *dispatcher=rt ? dynamic_cast<bozo::ee::Dispatcher *>(rt->services) : nullptr;
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
    auto *d=rt ? dynamic_cast<bozo::ee::Dispatcher *>(rt->services) : nullptr;
    if(!d || !ctx || ctx->pc!=0x140b70 || d->kernel().running()<=0)
        throw std::runtime_error("event SDK firmware retirement requires a booted native kernel");
    // The installer copies kernel code, replaces private syscall slots and
    // retains a kernel-object address. Native callers must use kernel services;
    // the original address-query consumers are rejecting boundaries above.
    std::fprintf(stderr,"[vp2s:bozo] retired SDK event firmware installer; kernel-address consumers remain unsupported\n");
    ctx->pc=ctx->r[31].uw[0];
}
}

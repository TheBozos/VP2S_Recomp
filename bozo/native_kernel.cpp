#include "native_kernel.h"
#include "bozo/ee/dispatcher.h"
#include <cstdio>
#include <cstring>
#include <format>
#include <stdexcept>

namespace vp2s {
namespace {
void rejectFirmwareEntry(uint8_t *,R5900Context *ctx,Runtime *) {
    throw std::runtime_error(std::format("unported SDK kernel/timer entry at {:x}; native replacement required",ctx->pc));
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
    registry.registerFunction(0x141448u,&initNativeSyscallServices);
    registry.registerFunction(0x141a38u,&retireAlarmFirmwareInstaller);
    registry.registerFunction(0x141bb0u,&initNativeTimer);
    registry.registerFunction(0x141df0u,&startNativeTimer);
    registry.registerFunction(0x1423f8u,&queryNativeTimer);
    registry.registerFunction(0x137280u,&nativeOsdSettings);
    registry.registerFunction(0x137290u,&nativeOsdSettings);
}
}

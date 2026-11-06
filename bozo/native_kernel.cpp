#include "native_kernel.h"
#include "bozo/ee/dispatcher.h"
#include <cstdio>
#include <format>
#include <stdexcept>

namespace vp2s {
namespace {
void rejectFirmwareEntry(uint8_t *,R5900Context *ctx,Runtime *) {
    throw std::runtime_error(std::format("unported SDK firmware-address query/interior entry at {:x}; native callers required",ctx->pc));
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
void installNativeKernelBindings(bozo::port::Registry &registry) {
    for(uint32_t pc=0x141410u;pc<0x141444u;pc+=4)
        registry.registerFunction(pc,&rejectFirmwareEntry);
    for(uint32_t pc=0x141448u;pc<0x141544u;pc+=4)
        registry.registerFunction(pc,&rejectFirmwareEntry);
    registry.registerFunction(0x141448u,&initNativeSyscallServices);
}
}

// SDK timer records/lifecycle from the USA initializer. No device state or polling.
#include "native_kernel.h"
#include "native_timer.h"
#include "bozo/ee/dispatcher.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace vp2s {
void initNativeTimer(uint8_t *,R5900Context *ctx,Runtime *rt) {
    auto *d=rt ? dynamic_cast<bozo::ee::Dispatcher *>(rt->services) : nullptr;
    if(!d || !ctx || ctx->pc!=0x141bb0 || d->kernel().running()<=0)
        throw std::runtime_error("native timer initialization requires a booted kernel");
    auto *ram=rt->memory.ram();
    auto get=[&](uint32_t address) { uint32_t value; std::memcpy(&value,ram+address,4); return value; };
    auto put=[&](uint32_t address,uint32_t value) { std::memcpy(ram+address,&value,4); };
    auto result=[&](uint32_t value) { ctx->r[2].sd[0]=static_cast<int32_t>(value); ctx->pc=ctx->r[31].uw[0]; };
    constexpr uint32_t state=0x14a238, pool=0x1e4180, callbacks=0x1e6180;
    if(static_cast<int32_t>(get(state+8))>=0) { result(0x80008001); return; }
    // Only the audited startup's clock choice is admitted. This is an API
    // admission guard, not a PS2 mode register retained anywhere in the port.
    if(ctx->r[4].uw[0]!=2)
        throw std::runtime_error("unported SDK timer initialization clock choice");
    put(state,0); put(state+4,0); put(state+16,0); put(state+20,pool);
    std::memset(ram+pool,0,128*64);
    for(uint32_t i=0;i<128;++i) put(pool+i*64,i==127?0:pool+(i+1)*64);
    put(0x1e6580,callbacks);
    // The original callback-pool helper only writes links. Payloads stay intact.
    for(uint32_t i=0;i<64;++i) put(callbacks+i*16,i==63?0:callbacks+(i+1)*16);
    auto &kernel=d->kernel();
    const auto id=kernel.addHandler(false,11,0x142150,0,0,ctx->r[28].uw[0],ctx->r[29].uw[0]-0x60);
    if(id<0) { result(0x80009021); return; }
    put(state+8,static_cast<uint32_t>(id));
    kernel.causeEnable(false,11,true);
    if(auto *owner=dynamic_cast<TimerOwner *>(&d->host())) {
        owner->timerClock()=TimerClock{};
        owner->timerClock().initialized=true;
    }
    // The callback address is registered to a rejecting native boundary until
    // deadline processing is ported; no original timer IRQ body can execute.
    std::fprintf(stderr,"[vp2s:bozo] native timer records initialized; runtime timer callback remains unsupported\n");
    result(0);
}
namespace {
TimerClock &clockFor(Runtime *rt,R5900Context *ctx,bozo::ee::Dispatcher *&d) {
    d=rt ? dynamic_cast<bozo::ee::Dispatcher *>(rt->services) : nullptr;
    auto *owner=d ? dynamic_cast<TimerOwner *>(&d->host()) : nullptr;
    if(!ctx || !owner || d->kernel().running()<=0 || !owner->timerClock().initialized)
        throw std::runtime_error("native SDK time requires a booted per-run clock owner");
    return owner->timerClock();
}
}
void startNativeTimer(uint8_t *,R5900Context *ctx,Runtime *rt) {
    bozo::ee::Dispatcher *d=nullptr;
    auto &clock=clockFor(rt,ctx,d);
    if(clock.running) { ctx->r[2].sd[0]=1; ctx->pc=ctx->r[31].uw[0]; return; }
    uint32_t active;
    std::memcpy(&active,rt->memory.ram()+0x14a250,4);
    if(active) throw std::runtime_error("native SDK timer deadlines are not ported yet");
    clock.epoch=d->now(ctx); clock.running=true;
    ctx->r[2].sd[0]=0; ctx->pc=ctx->r[31].uw[0];
    std::fprintf(stderr,"[vp2s:bozo] native SDK timer started\n");
}
void queryNativeTimer(uint8_t *,R5900Context *ctx,Runtime *rt) {
    bozo::ee::Dispatcher *d=nullptr;
    const auto &clock=clockFor(rt,ctx,d);
    ctx->r[2].ud[0]=clock.value(d->now(ctx)); ctx->pc=ctx->r[31].uw[0];
}
void rearmNativeTimer(uint8_t *,R5900Context *ctx,Runtime *rt) {
    bozo::ee::Dispatcher *d=nullptr;
    clockFor(rt,ctx,d);
    uint32_t active,dispatching;
    std::memcpy(&active,rt->memory.ram()+0x14a250,4);
    std::memcpy(&dispatching,rt->memory.ram()+0x14a254,4);
    // While a callback is being dispatched the original does nothing. An empty
    // list only disables its compare interrupt. Native time has no such device.
    // Do not accept a pending list until its real deadlines/callbacks are ported.
    if(static_cast<int32_t>(dispatching)<0 && active)
        throw std::runtime_error("native SDK timer rearm requires pending deadline support");
    ctx->pc=ctx->r[31].uw[0];
}
}

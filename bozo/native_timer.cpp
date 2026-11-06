// SDK timer records/lifecycle from the USA initializer. No device state or polling.
#include "native_kernel.h"
#include "native_timer.h"
#include "bozo/ee/dispatcher.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <limits>

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
    // Its callback entry is a native record producer; no original device IRQ
    // body can execute through this binding.
    std::fprintf(stderr,"[vp2s:bozo] native timer records initialized\n");
    result(0);
}
namespace {
TimerClock &clockFor(Runtime *rt,R5900Context *ctx,bozo::ee::Dispatcher *&d) {
    d=rt ? dynamic_cast<bozo::ee::Dispatcher *>(rt->services) : nullptr;
    auto *owner=d ? dynamic_cast<TimerOwner *>(&d->host()) : nullptr;
    if(!ctx || !owner || d->kernel().running()==0 || !owner->timerClock().initialized)
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
    auto &clock=dynamic_cast<TimerOwner &>(d->host()).timerClock();
    if(static_cast<int32_t>(dispatching)<0) {
        uint64_t dueCycle=0;
        if(active) {
            if(active<0x1e4180 || active>=0x1e6180 || (active-0x1e4180)%64)
                throw std::runtime_error("invalid SDK timer record");
            uint64_t start,elapsed,period;
            std::memcpy(&start,rt->memory.ram()+active+16,8);
            std::memcpy(&elapsed,rt->memory.ram()+active+24,8);
            std::memcpy(&period,rt->memory.ram()+active+32,8);
            const uint64_t deadline=start+period-elapsed;
            const uint64_t now=d->now(ctx);
            if(!clock.running) throw std::runtime_error("SDK timer deadline requires running native clock");
            if(deadline<=clock.value(now)) dueCycle=now;
            else {
                const uint64_t delta=deadline-clock.base;
                const uint64_t quanta=delta/256+(delta%256!=0);
                if(clock.epoch==~0ull || quanta>(std::numeric_limits<uint64_t>::max()-1-clock.epoch)/512)
                    throw std::runtime_error("SDK timer deadline exceeds native clock domain");
                dueCycle=clock.epoch+quanta*512;
            }
        }
        if(clock.deadlineSerial==~0ull) throw std::runtime_error("SDK timer callback sequence exhausted");
        ++clock.deadlineSerial;
        constexpr uint32_t key=0x56503254; // VP2T: private callback owner in this kernel.
        if(active) {
            bozo::ee::QueuedHandler h;
            h.kind=bozo::ee::kInvokeAlarm;h.pc=0x142150;h.tag=clock.deadlineSerial;
            h.gp=ctx->r[28].uw[0];
            d->kernel().scheduleDeadline(key,dueCycle,h);
        } else d->kernel().cancelDeadline(key);
    }
    ctx->pc=ctx->r[31].uw[0];
}

namespace {
struct TimerRecords {
    uint8_t *ram;
    void node(uint32_t p) const {
        if(p<0x1e4180 || p>=0x1e6180 || (p-0x1e4180)%64)
            throw std::runtime_error("invalid SDK timer list link");
    }
    uint32_t u32(uint32_t p) const { uint32_t v;std::memcpy(&v,ram+p,4);return v; }
    uint64_t u64(uint32_t p) const { uint64_t v;std::memcpy(&v,ram+p,8);return v; }
    void put(uint32_t p,uint32_t v) { std::memcpy(ram+p,&v,4); }
    void put64(uint32_t p,uint64_t v) { std::memcpy(ram+p,&v,8); }
    uint64_t deadline(uint32_t p) const { return u64(p+16)+u64(p+32)-u64(p+24); }
    uint32_t unlink(uint32_t p) {
        node(p);const auto prev=u32(p+4),next=u32(p);
        if(prev) node(prev);if(next) node(next);
        put(prev?prev:0x14a250,next);
        if(next)put(next+4,prev);
        put(p+4,0);return next;
    }
    void insert(uint32_t p) {
        uint32_t prev=0,next=u32(0x14a250);unsigned visited=0;
        while(next) {
            node(next);
            if(++visited>128 || next==p) throw std::runtime_error("cyclic SDK timer list");
            if(deadline(p)<deadline(next)) break;
            prev=next;next=u32(next);
        }
        put(p+4,prev);put(p,next);
        if(next)put(next+4,p);
        put(prev?prev:0x14a250,p);
    }
};
}
uint32_t processTimerRecord(uint8_t *,R5900Context *ctx,Runtime *rt,uint32_t record,uint64_t now) {
    TimerRecords r{rt->memory.ram()};r.node(record);
    const uint32_t next=r.unlink(record);
    const uint32_t handle=(record<<4)|r.u32(record+8);
    r.put(0x14a254,handle);
    R5900Context callback=*ctx;
    callback.pc=r.u32(record+40);callback.r[31].ud[0]=0;
    callback.r[28].sd[0]=static_cast<int32_t>(r.u32(record+44));
    callback.r[4].sd[0]=static_cast<int32_t>(handle);
    callback.r[5].ud[0]=r.u64(record+32);
    callback.r[6].ud[0]=now+r.u64(record+24)-r.u64(record+16);
    callback.r[7].sd[0]=static_cast<int32_t>(r.u32(record+48));
    callback.r[8].ud[0]=ctx->r[6].ud[0];
    bozo_call_indirect(rt->memory.flat(),&callback,rt,callback.pc);
    if(callback.pc!=0) throw std::runtime_error("native SDK timer callback did not return");
    ctx->cycles=callback.cycles;ctx->checkpoints=callback.checkpoints;
    const uint64_t reply=callback.r[2].ud[0];
    if(reply==0) r.put(record+12,r.u32(record+12)&~2u);
    else if(reply==~0ull) {
        r.put(record+8,0);r.put(record+12,0);
        r.put(record,r.u32(0x14a24c));r.put(0x14a24c,record);
        r.put(0x14a248,r.u32(0x14a248)-1);
    } else {
        r.put64(record+32,r.u64(record+32)+(reply>0x3998?reply:0x3999));
        r.insert(record);
    }
    return next;
}
void dispatchNativeTimer(uint8_t *,R5900Context *ctx,Runtime *rt) {
    bozo::ee::Dispatcher *d=nullptr;auto &clock=clockFor(rt,ctx,d);
    if(!d->currentFrame() || d->currentDepth()==0)
        throw std::runtime_error("native SDK timer dispatch requires a scheduled callback");
    if(d->currentFrame()->tag!=clock.deadlineSerial) { ctx->pc=ctx->r[31].uw[0];return; }
    TimerRecords records{rt->memory.ram()};
    uint32_t record=records.u32(0x14a250);unsigned visited=0;
    while(record) {
        records.node(record);
        if(++visited>128) throw std::runtime_error("cyclic SDK timer dispatch list");
        const auto now=clock.value(d->now(ctx));
        if(now<records.deadline(record))break;
        record=processTimerRecord(nullptr,ctx,rt,record,now);
    }
    records.put(0x14a254,0xffffffff);
    rearmNativeTimer(nullptr,ctx,rt);
    ctx->r[2].sd[0]=0;
}
}

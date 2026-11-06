// Native SDK boundary tests: no game instructions, firmware image or device model.
#include "native_kernel.h"
#include "bozo/ee/dispatcher.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace {
void require(bool condition) { if(!condition) throw std::runtime_error("native kernel boundary mismatch"); }
template<class Fn> void rejected(Fn fn) {
    bool failed=false;
    try { fn(); } catch(const std::runtime_error &) { failed=true; }
    require(failed);
}
class Host final : public bozo::ee::Host {
public:
    unsigned callbacks=0;
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t) override { ++callbacks; throw std::runtime_error("unexpected library call"); }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t) override { ++callbacks; throw std::runtime_error("unexpected VU call"); }
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2]) override { ++callbacks; throw std::runtime_error("unexpected device read"); }
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2]) override { ++callbacks; throw std::runtime_error("unexpected device write"); }
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &what) override { ++callbacks; throw std::runtime_error(what); }
};
void bind(bozo::ee::Dispatcher &d,unsigned mask) {
    if(mask&1) d.bindKernelCall(0x1370d0,bozo::ee::KernelCall::GetThreadId);
    if(mask&2) d.bindKernelCall(0x1371e0,bozo::ee::KernelCall::CreateSema);
    if(mask&4) d.bindKernelCall(0x137460,bozo::ee::KernelCall::FlushCache);
}
}
int main() {
    try {
        Host host;
        bozo::ee::Dispatcher d(host);
        auto &rt=d.runtime();
        R5900Context ctx;
        ctx.pc=0x141448; ctx.r[31].uw[0]=0x141570;
        rejected([&]{vp2s::initNativeSyscallServices(nullptr,&ctx,&rt);});
        bind(d,7);
        rejected([&]{vp2s::initNativeSyscallServices(nullptr,&ctx,&rt);}); // no boot
        ctx.pc=0x141a38;
        rejected([&]{vp2s::retireAlarmFirmwareInstaller(nullptr,&ctx,&rt);});
        d.boot(0x100000,0x1fffff0);
        {
            Host h; bozo::ee::Dispatcher missing(h);
            bind(missing,3); missing.boot(0x100000,0x1fffff0);
            rejected([&]{vp2s::retireAlarmFirmwareInstaller(nullptr,&ctx,&missing.runtime());});
        }
        ctx.pc=0x141448;
        for(unsigned mask=0;mask<7;++mask) {
            Host h; bozo::ee::Dispatcher missing(h);
            bind(missing,mask); missing.boot(0x100000,0x1fffff0);
            rejected([&]{vp2s::initNativeSyscallServices(nullptr,&ctx,&missing.runtime());});
        }
        auto &k=d.kernel();
        const auto first=k.createSema(1,2,0,0);
        require(first>0);
        bozo::ee::QueuedHandler pending;
        pending.kind=bozo::ee::kInvokeLibraryCall; pending.pc=0x155000;
        k.queue(pending);
        const auto sequence=k.sequence();
        const auto event=k.nextEvent();
        std::memset(rt.memory.flat(),0xa7,bozo::kFlatBytes);
        std::vector<uint8_t> before(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
        for(unsigned n=0;n<16;++n) {
            std::memset(&ctx,0x20+n,sizeof(ctx));
            ctx.pc=0x141448; ctx.r[31].uw[0]=0x141570+n*4;
            R5900Context expected;
            std::memcpy(&expected,&ctx,sizeof(ctx)); expected.pc=ctx.r[31].uw[0];
            vp2s::initNativeSyscallServices(nullptr,&ctx,&rt);
            require(!std::memcmp(&ctx,&expected,sizeof(ctx)));
            ctx.pc=0x141a38;
            std::memcpy(&expected,&ctx,sizeof(ctx)); expected.pc=ctx.r[31].uw[0];
            vp2s::retireAlarmFirmwareInstaller(nullptr,&ctx,&rt);
            require(!std::memcmp(&ctx,&expected,sizeof(ctx)));
        }
        require(!std::memcmp(before.data(),rt.memory.flat(),bozo::kFlatBytes));
        require(k.running()==1 && k.createSema(0,1,0,0)==first+1);
        require(k.handlerQueued() && k.sequence()==sequence && k.nextEvent()==event);
        require(k.pollSema(first)>=0 && k.pollSema(first)<0);
        require(host.callbacks==0);
        bozo::port::Registry registry;
        vp2s::installNativeKernelBindings(registry);
        require(registry.find(0x141448)==&vp2s::initNativeSyscallServices);
        require(registry.find(0x141a38)==&vp2s::retireAlarmFirmwareInstaller);
        unsigned guarded=0;
        for(const auto &entry:registry.functions()) {
            if(entry.address==0x141448 || entry.address==0x141a38 || entry.address==0x141bb0) continue;
            ctx.pc=entry.address;
            rejected([&]{entry.function(nullptr,&ctx,&rt);});
            ++guarded;
        }
        require(guarded==844 && !registry.find(0x141548)); // SetSyscall is not bypassed
        for(uint32_t number:{116u,0xfcu,0xfeu,0xfdu,0xffu,0x12cu,8u}) {
            ctx.pc=0x14154c; ctx.r[3].uw[0]=number;
            ctx.r[4].uw[0]=0x83; ctx.r[5].uw[0]=0x1413c0;
            rejected([&]{d.syscall(nullptr,&ctx,&rt,0);});
        }
        // Names/enums are not evidence of working native alarm services.
        for(auto call:{bozo::ee::KernelCall::SetAlarm,bozo::ee::KernelCall::iSetAlarm,
                       bozo::ee::KernelCall::ReleaseAlarm,bozo::ee::KernelCall::iReleaseAlarm}) {
            d.bindKernelCall(0x180000,call);
            rejected([&]{bozo_call_indirect(rt.memory.flat(),&ctx,&rt,0x180000);});
        }
        require(host.callbacks==11 && !std::memcmp(before.data(),rt.memory.flat(),bozo::kFlatBytes));
        require(k.handlerQueued() && k.sequence()==sequence && k.nextEvent()==event);
        std::puts("native installer readiness, 32 context cases, RAM/kernel preservation, 844 guards and unsupported alarm/syscall rejection pass");
        return 0;
    } catch(const std::exception &e) { std::fprintf(stderr,"%s\n",e.what()); return 1; }
}

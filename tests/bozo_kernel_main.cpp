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
        d.boot(0x100000,0x1fffff0);
        for(unsigned mask=0;mask<7;++mask) {
            Host h; bozo::ee::Dispatcher missing(h);
            bind(missing,mask); missing.boot(0x100000,0x1fffff0);
            rejected([&]{vp2s::initNativeSyscallServices(nullptr,&ctx,&missing.runtime());});
        }
        auto &k=d.kernel();
        const auto first=k.createSema(1,2,0,0);
        require(first>0);
        std::memset(rt.memory.flat(),0xa7,bozo::kFlatBytes);
        std::vector<uint8_t> before(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
        for(unsigned n=0;n<16;++n) {
            std::memset(&ctx,0x20+n,sizeof(ctx));
            ctx.pc=0x141448; ctx.r[31].uw[0]=0x141570+n*4;
            R5900Context expected;
            std::memcpy(&expected,&ctx,sizeof(ctx)); expected.pc=ctx.r[31].uw[0];
            vp2s::initNativeSyscallServices(nullptr,&ctx,&rt);
            require(!std::memcmp(&ctx,&expected,sizeof(ctx)));
        }
        require(!std::memcmp(before.data(),rt.memory.flat(),bozo::kFlatBytes));
        require(k.running()==1 && k.createSema(0,1,0,0)==first+1);
        require(k.pollSema(first)>=0 && k.pollSema(first)<0);
        require(host.callbacks==0);
        bozo::port::Registry registry;
        vp2s::installNativeKernelBindings(registry);
        require(registry.find(0x141448)==&vp2s::initNativeSyscallServices);
        unsigned guarded=0;
        for(const auto &entry:registry.functions()) {
            if(entry.address==0x141448) continue;
            ctx.pc=entry.address;
            rejected([&]{entry.function(nullptr,&ctx,&rt);});
            ++guarded;
        }
        require(guarded==75 && !registry.find(0x141548)); // SetSyscall is not bypassed
        ctx.pc=0x14154c; ctx.r[3].uw[0]=116; ctx.r[4].uw[0]=0x83; ctx.r[5].uw[0]=0x1413c0;
        rejected([&]{d.syscall(nullptr,&ctx,&rt,0);});
        require(host.callbacks==1 && !std::memcmp(before.data(),rt.memory.flat(),bozo::kFlatBytes));
        std::puts("native kernel readiness, context/RAM preservation, 75 rejecting entries and SetSyscall rejection pass");
        return 0;
    } catch(const std::exception &e) { std::fprintf(stderr,"%s\n",e.what()); return 1; }
}

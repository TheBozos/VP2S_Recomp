#include "native_kernel.h"
#include "native_timer.h"
#include "bozo/ee/dispatcher.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>
namespace {
void check(bool b) { if(!b) throw std::runtime_error("native SDK clock mismatch"); }
class Host final:public bozo::ee::Host,public vp2s::TimerOwner {
public:
    vp2s::TimerClock clock;
    vp2s::TimerClock &timerClock() override { return clock; }
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t) override { throw std::runtime_error("unexpected library"); }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t) override { throw std::runtime_error("unexpected VU"); }
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2]) override { throw std::runtime_error("unexpected I/O"); }
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2]) override { throw std::runtime_error("unexpected I/O"); }
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s) override { throw std::runtime_error(s); }
};
}
int main() {
    try {
        Host h,other; bozo::ee::Dispatcher d(h),second(other);
        d.boot(0x100000,0x1fffff0); second.boot(0x100000,0x1fffff0);
        auto &rt=d.runtime(); uint32_t id=0xffffffffu;
        std::memcpy(rt.memory.ram()+0x14a240,&id,4);
        R5900Context ctx; ctx.pc=0x141bb0;ctx.r[4].uw[0]=2;ctx.r[29].uw[0]=0x1fffff0;ctx.r[31].uw[0]=0x150000;
        vp2s::initNativeTimer(nullptr,&ctx,&rt);
        check(h.clock.initialized && !other.clock.initialized);
        std::vector<uint8_t> before(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
        d.setNow(&ctx,500);ctx.pc=0x141df0;
        rt.cop0Status=0x10001;
        vp2s::startNativeTimer(nullptr,&ctx,&rt);
        check(ctx.r[2].sd[0]==0 && ctx.pc==0x150000 && h.clock.epoch==500);
        d.setNow(&ctx,700);ctx.pc=0x141df0;
        vp2s::startNativeTimer(nullptr,&ctx,&rt);
        check(ctx.r[2].sd[0]==1 && h.clock.epoch==500);
        const uint64_t samples[][2]={{0,0},{1,0},{511,0},{512,256},{513,256},{1023,256},{1024,512},
            {33553920,0xffff00},{33554432,0x1000000},{33554944,0x1000100},{1ull<<41,1ull<<40}};
        for(const auto &sample:samples) {
            d.setNow(&ctx,500+sample[0]);ctx.pc=0x1423f8;
            ctx.r[2].ud[1]=0xabcdef1234567890ull;
            R5900Context expected;std::memcpy(&expected,&ctx,sizeof(ctx));
            expected.r[2].ud[0]=sample[1];expected.pc=0x150000;
            vp2s::queryNativeTimer(nullptr,&ctx,&rt);
            check(!std::memcmp(&ctx,&expected,sizeof(ctx)));
        }
        check(!std::memcmp(before.data(),rt.memory.flat(),bozo::kFlatBytes) && rt.cop0Status==0x10001);
        bool rejected=false;
        try { vp2s::startNativeTimer(nullptr,&ctx,&second.runtime()); } catch(const std::runtime_error &) { rejected=true; }
        check(rejected && !other.clock.initialized);
        rejected=false;
        try { h.clock.value(499); } catch(const std::runtime_error &) { rejected=true; }
        check(rejected);
        h.clock.running=false; uint32_t active=0x180000;
        std::memcpy(rt.memory.ram()+0x14a250,&active,4);
        rejected=false;
        try { vp2s::startNativeTimer(nullptr,&ctx,&rt); } catch(const std::runtime_error &) { rejected=true; }
        check(rejected && !h.clock.running);
        std::puts("native SDK clock: units, resolution, 16-bit boundary, 64-bit duration, repeat start, RAM/ABI and host isolation pass");
        return 0;
    } catch(const std::exception &e) { std::fprintf(stderr,"%s\n",e.what());return 1; }
}

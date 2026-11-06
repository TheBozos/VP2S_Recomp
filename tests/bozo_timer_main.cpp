// Differential check against original statically recompiled code, stopped at
// its FIRST device request. No device value, firmware image or interpreter.
#include "native_kernel.h"
#include "native_timer.h"
#include "bozo/ee/dispatcher.h"
#include "timer_ref_functions.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace {
void check(bool value) { if(!value) throw std::runtime_error("timer record/ABI comparison failed"); }
struct FirstDevice {};
class Host final : public bozo::ee::Host,public vp2s::TimerOwner {
public:
    vp2s::TimerClock clock;
    vp2s::TimerClock &timerClock() override { return clock; }
    unsigned reads=0;
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *ctx,uint32_t target) override {
        check(target==0x132e40);
        auto *out=d.runtime().memory.hostPointer(ctx->r[4].uw[0],ctx->r[6].uw[0]);
        check(out!=nullptr);
        std::memset(out,ctx->r[5].uw[0]&255,ctx->r[6].uw[0]);
        ctx->r[2].sd[0]=static_cast<int32_t>(ctx->r[4].uw[0]);
        ctx->pc=ctx->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t) override { throw std::runtime_error("unexpected VU request"); }
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t address,uint32_t size,uint64_t[2]) override {
        check(address==0x10001010 && size==4); ++reads; throw FirstDevice{};
    }
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2]) override { throw std::runtime_error("unexpected device write"); }
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &what) override { throw std::runtime_error(what); }
};
void word(uint8_t *ram,uint32_t at,uint32_t value) { std::memcpy(ram+at,&value,4); }
uint32_t word(const uint8_t *ram,uint32_t at) { uint32_t value; std::memcpy(&value,ram+at,4); return value; }
}
int main() {
    try {
        constexpr uint32_t state=0x14a238, sp=0x1f00000;
        for(unsigned test=0;test<32;++test) {
            Host originalHost,nativeHost;
            bozo::ee::Dispatcher original(originalHost),native(nativeHost);
            auto &ref=original.runtime(); auto &actual=native.runtime();
            original.setFunctions(timer_ref_function_table,timer_ref_function_count,timer_ref_resume_table,timer_ref_resume_count);
            original.bindLibrary(0x132e40);
            original.bindKernelCall(0x136ed0,bozo::ee::KernelCall::AddIntcHandler);
            original.boot(0x100000,sp); native.boot(0x100000,sp);
            const uint32_t initialStatus=(test&1)?0x10001u:0u;
            ref.cop0Status=actual.cop0Status=initialStatus;
            for(unsigned i=0;i<test%4;++i) {
                original.kernel().addHandler(false,7,0x150000,0,0,0,0);
                native.kernel().addHandler(false,7,0x150000,0,0,0,0);
            }
            // Deliberately dirty callback payloads; clearing them is a regression.
            std::memset(ref.memory.flat(),0x81+test,bozo::kFlatBytes);
            word(ref.memory.ram(),state+8,test<16?0xffffffffu-test:17u+test);
            std::memcpy(actual.memory.flat(),ref.memory.flat(),bozo::kFlatBytes);
            std::vector<uint8_t> stack(ref.memory.ram()+sp-0x60,ref.memory.ram()+sp);
            R5900Context reference,ctx;
            reference.r[4].uw[0]=2; reference.r[28].uw[0]=0x1d0b70;
            reference.r[29].uw[0]=sp; reference.r[31].uw[0]=0x200000;
            reference.r[2].ud[1]=0xabcdef0123456789ull;
            ctx=reference; ctx.pc=0x141bb0;
            R5900Context expected;
            std::memcpy(&expected,&ctx,sizeof(ctx));
            bool trapped=false;
            try { ref.functions.generated(0x141bb0)(ref.memory.flat(),&reference,&ref); }
            catch(const FirstDevice &) { trapped=true; }
            check(trapped==(test<16) && originalHost.reads==(test<16?1u:0u));
            vp2s::initNativeTimer(nullptr,&ctx,&actual);
            expected.pc=0x200000; expected.r[2].sd[0]=test<16?0:static_cast<int32_t>(0x80008001u);
            check(!std::memcmp(&ctx,&expected,sizeof(ctx)));
            // Original prefix uses its own ABI stack. Compare ALL other flat
            // memory, including link payload bytes and every untouched record.
            std::memcpy(ref.memory.ram()+sp-0x60,stack.data(),stack.size());
            check(!std::memcmp(ref.memory.flat(),actual.memory.flat(),bozo::kFlatBytes));
            check(nativeHost.reads==0 && actual.cop0Status==initialStatus);
            if(test>=16) check(reference.pc==ctx.pc && reference.r[2].sd[0]==ctx.r[2].sd[0]);
            if(test<16) {
                const uint32_t id=word(actual.memory.ram(),state+8);
                check(id==1+test%4);
                std::vector<uint8_t> before(actual.memory.flat(),actual.memory.flat()+bozo::kFlatBytes);
                ctx.pc=0x141bb0; ctx.r[4].uw[0]=3; // repeat branch precedes clock admission
                vp2s::initNativeTimer(nullptr,&ctx,&actual);
                check(ctx.r[2].sd[0]==static_cast<int32_t>(0x80008001u));
                check(!std::memcmp(before.data(),actual.memory.flat(),bozo::kFlatBytes));
                // Native cause/handler lifecycle is real; the unported IRQ body
                // rejects before any original device-dependent code can run.
                native.kernel().irq(false,11);
                check(native.kernel().handlerQueued());
                auto &frame=native.kernel().stackHandler(*native.kernel().thread(1));
                check(frame.ctx.pc==0x142150 && frame.a[0]==11 && frame.gp==0x1d0b70);
                check(!native.kernel().handlerQueued()); // repeat init did not register twice
                bozo::port::Registry registry; vp2s::installNativeKernelBindings(registry);
                bool rejected=false;
                try { registry.find(0x142150)(nullptr,&frame.ctx,&actual); }
                catch(const std::runtime_error &) { rejected=true; }
                check(rejected && nativeHost.reads==0);
            }
        }
        // Original empty rearm has no game-data output before its FIRST device
        // read; dispatch-in-progress exits before any device access. Stack ABI
        // scratch is the only excluded region, as in the initializer fixture.
        for(int32_t dispatching:{-1,0,7}) {
            Host aHost,bHost; bHost.clock.initialized=true;
            bozo::ee::Dispatcher a(aHost),b(bHost);
            a.boot(0x100000,sp);b.boot(0x100000,sp);
            a.setFunctions(timer_ref_function_table,timer_ref_function_count,timer_ref_resume_table,timer_ref_resume_count);
            auto &x=a.runtime();auto &y=b.runtime();
            std::memset(x.memory.flat(),0x76,bozo::kFlatBytes);
            word(x.memory.ram(),state+24,0);word(x.memory.ram(),state+28,dispatching);
            std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
            std::vector<uint8_t> stack(x.memory.ram()+sp-0x40,x.memory.ram()+sp);
            R5900Context original,native; original.pc=0x141f00;
            original.r[29].uw[0]=sp;original.r[31].uw[0]=0x200000;native=original;
            bool stopped=false;
            try { x.functions.generated(0x141f00)(x.memory.flat(),&original,&x); } catch(const FirstDevice &) { stopped=true; }
            check(stopped==(dispatching<0));
            vp2s::rearmNativeTimer(nullptr,&native,&y);
            check(native.pc==0x200000 && bHost.reads==0);
            std::memcpy(x.memory.ram()+sp-0x40,stack.data(),stack.size());
            check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        }
        Host h; bozo::ee::Dispatcher d(h); d.boot(0x100000,sp);
        auto &rt=d.runtime(); word(rt.memory.ram(),state+8,0xffffffff);
        std::vector<uint8_t> before(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
        R5900Context ctx; ctx.pc=0x141bb0; ctx.r[4].uw[0]=3;
        bool rejected=false;
        try { vp2s::initNativeTimer(nullptr,&ctx,&rt); } catch(const std::runtime_error &) { rejected=true; }
        check(rejected && !std::memcmp(before.data(),rt.memory.flat(),bozo::kFlatBytes) && !d.kernel().handlerQueued());
        check(d.kernel().addHandler(false,7,0x150000,0,0,0,0)==1); // no partial allocation
        Host coldHost; bozo::ee::Dispatcher cold(coldHost);
        ctx.r[4].uw[0]=2;
        rejected=false;
        try { vp2s::initNativeTimer(nullptr,&ctx,&cold.runtime()); } catch(const std::runtime_error &) { rejected=true; }
        check(rejected && cold.kernel().running()==0 && coldHost.reads==0);
        std::puts("32 original/native full-record comparisons, 16 repeat calls, IRQ lifecycle/rejection and unsupported-clock atomicity pass");
        return 0;
    } catch(const std::exception &e) { std::fprintf(stderr,"%s\n",e.what()); return 1; }
}

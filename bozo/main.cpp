// Initial BozoEngine bring-up. Unsupported boundaries fail explicitly.
#include "bozo/ee/dispatcher.h"
#include "bozo/host/free_host.h"
#include "bozo/port.h"
#include "vp2_bozo_images.h"
#include "native_kernel.h"
#include "native_timer.h"
#include "native_settings.h"
#include <cstdio>
#include <cstdlib>
#include <format>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace {
class StrictHost final : public bozo::ee::Host, public vp2s::TimerOwner, public vp2s::SettingsOwner {
public:
    bozo::host::FreeHost native;
    vp2s::TimerClock sdkClock;
    vp2s::TimerClock &timerClock() override { return sdkClock; }
    vp2s::Settings preferences;
    vp2s::Settings &settings() override { return preferences; }
    explicit StrictHost(const bozo::host::FreeOptions &options) : native(options) {}
    void boundaryHash(bozo::ee::Dispatcher &d) {
        if(!std::getenv("VP2S_BOZO_BOUNDARY_HASH")) return;
        uint64_t hash=14695981039346656037ull;
        const auto *ram=d.runtime().memory.flat();
        for(uint32_t i=0;i<bozo::kFlatBytes;++i) { hash^=ram[i];hash*=1099511628211ull; }
        std::fprintf(stderr,"[vp2s:bozo] boundary RAM hash=%016llx\n",(unsigned long long)hash);
    }
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t target) override {
        if(std::getenv("VP2S_BOZO_TRACE_CALLS"))
            std::fprintf(stderr,"[vp2s:call] pc=%x ra=%x a0=%x a1=%x\n",target,c->r[31].uw[0],c->r[4].uw[0],c->r[5].uw[0]);
        native.libraryCall(d,c,target);
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *c,uint32_t address) override {
        throw std::runtime_error(std::format("unported VP2S VU0 algorithm {:x} at pc={:x}",address,c->pc));
    }
    void ioRead(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t address,uint32_t,uint64_t[2]) override {
        boundaryHash(d);
        throw std::runtime_error(std::format("unported device read {:x} at pc={:x}",address,c->pc));
    }
    void ioWrite(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t address,uint32_t,const uint64_t[2]) override {
        boundaryHash(d);
        throw std::runtime_error(std::format("unported device write {:x} at pc={:x}",address,c->pc));
    }
    bool cop0Condition(bozo::ee::Dispatcher &,R5900Context *c) override {
        throw std::runtime_error(std::format("unported device completion condition at pc={:x}",c->pc));
    }
    void fault(bozo::ee::Dispatcher &d,R5900Context *c,const std::string &what) override {
        std::fprintf(stderr,"[vp2s:bozo] fault args a0=%x a1=%x a2=%x a3=%x ra=%x\n",
                     c->r[4].uw[0],c->r[5].uw[0],c->r[6].uw[0],c->r[7].uw[0],c->r[31].uw[0]);
        native.fault(d,c,what);
    }
    void idle(bozo::ee::Dispatcher &d,uint64_t from,uint64_t to) override { native.idle(d,from,to); }
    void resuming(bozo::ee::Dispatcher &d,bozo::ee::Thread &t,bozo::ee::Frame &f,uint32_t depth) override { native.resuming(d,t,f,depth); }
    unsigned traceKernel=0;
    void kernelCallEnd(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t target,bool stop) override {
        if(std::getenv("VP2S_BOZO_TRACE_CALLS") && traceKernel++<160)
            std::fprintf(stderr,"[vp2s:kernel] pc=%x a0=%x result=%lld stop=%d\n",target,c->r[4].uw[0],(long long)c->r[2].sd[0],stop);
        native.kernelCallEnd(d,c,target,stop);
    }
    void kernelCallBegin(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t target,uint32_t kind) override { native.kernelCallBegin(d,c,target,kind); }
    void waitCompleted(bozo::ee::Dispatcher &d,bozo::ee::Thread &t) override { native.waitCompleted(d,t); }
    void handlerCompleted(bozo::ee::Dispatcher &d,bozo::ee::Thread &t,const bozo::ee::Frame &f) override { native.handlerCompleted(d,t,f); }
};
bool noNativeInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc) {
    throw std::runtime_error(std::format("unported native VP2S algorithm at pc={:x}",pc));
}
}

int main(int argc,char **argv) {
    bozo::host::FreeOptions options;
    options.elf=std::string(VP2S_BOZO_IMAGES)+"/main.elf";
    options.frames=1; options.stallFields=120; options.printGameOutput=true;
    for(int i=1;i<argc;++i) {
        const std::string arg=argv[i];
        if(arg=="--iso" && i+1<argc)options.iso=argv[++i];
        else if(arg=="--card-dir" && i+1<argc)options.cardDir=argv[++i];
        else if(arg=="--frames" && i+1<argc)options.frames=std::stoul(argv[++i]);
        else { std::fprintf(stderr,"usage: vp2s-bozo-headless --iso FILE [--card-dir PRIVATE_DIR] [--frames N]\n");return 2; }
    }
    if(options.iso.empty()) {std::fprintf(stderr,"user-owned USA ISO required\n");return 2;}
    try {
        StrictHost host(options);
        bozo::ee::Dispatcher dispatcher(host);
        dispatcher.setFunctions(vp2_main_function_table,vp2_main_function_count,vp2_main_resume_table,vp2_main_resume_count);
        bozo::port::Registry registry;
        vp2s::installNativeKernelBindings(registry);
        registry.nativeGraphics(&noNativeInstruction); // guard retired graphics; no compatibility fallback
        std::string error;
        if(!bozo::port::undeclared(registry,vp2_main_external_table,vp2_main_external_count).empty())
            throw std::runtime_error("native kernel bindings missing from generated externals");
        if(!host.native.setUp(dispatcher,vp2_main_external_table,vp2_main_external_count,&registry,error))
            throw std::runtime_error(error);
        vp2sBozoRegisterImages(dispatcher.runtime(),VP2S_BOZO_IMAGES);
        const auto frames=host.native.run();
        const auto &stats=host.native.stats();
        std::fprintf(stderr,"[vp2s:bozo] frames=%llu libraries=%llu end=%s\n",
                     (unsigned long long)frames,(unsigned long long)stats.libraryCalls,host.native.endReason().c_str());
        return frames>=options.frames?0:1;
    } catch(const std::exception &e) {
        std::fprintf(stderr,"[vp2s:bozo] native boundary: %s\n",e.what());return 1;
    }
}

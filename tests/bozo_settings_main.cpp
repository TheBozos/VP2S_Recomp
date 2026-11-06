#include "native_settings.h"
#include "native_kernel.h"
#include "bozo/ee/dispatcher.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
namespace {
void check(bool value) { if(!value) throw std::runtime_error("native settings mismatch"); }
class Host final: public bozo::ee::Host,public vp2s::SettingsOwner {
public:
    vp2s::Settings prefs;
    vp2s::Settings &settings() override { return prefs; }
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t) override { throw std::runtime_error("unexpected call"); }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t) override { throw std::runtime_error("unexpected VU"); }
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2]) override { throw std::runtime_error("unexpected I/O"); }
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2]) override { throw std::runtime_error("unexpected I/O"); }
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s) override { throw std::runtime_error(s); }
};
}
int main() {
    try {
        Host host,other; bozo::ee::Dispatcher d(host),separate(other);
        auto &rt=d.runtime();
        R5900Context ctx; ctx.pc=0x137290;ctx.r[4].uw[0]=0x180001;ctx.r[31].uw[0]=0x200000;
        auto *buffer=rt.memory.ram()+0x180000;
        std::memset(buffer,0xa5,8);
        vp2s::nativeOsdSettings(nullptr,&ctx,&rt);
        uint32_t value;std::memcpy(&value,buffer+1,4);
        check(value==0x12011 && ctx.r[2].sd[0]==0 && ctx.pc==0x200000);
        check(buffer[0]==0xa5 && buffer[5]==0xa5);
        for(uint32_t requested:{0u,0xffffffffu,0x10011u,0x30015u}) {
            std::memcpy(buffer+1,&requested,4);
            ctx.pc=0x137280;vp2s::nativeOsdSettings(nullptr,&ctx,&rt);
            ctx.pc=0x137290;vp2s::nativeOsdSettings(nullptr,&ctx,&rt);
            std::memcpy(&value,buffer+1,4);
            check(value==((requested&~0xe000u)|0x2000u));
            check(other.prefs.packed()==0x12011);
        }
        const auto before=host.prefs.packed();
        for(uint32_t pc:{0x137280u,0x137290u}) {
            ctx.pc=pc;ctx.r[4].uw[0]=0x1ffffff;
            vp2s::nativeOsdSettings(nullptr,&ctx,&rt);
            check(ctx.r[2].sd[0]==-1 && host.prefs.packed()==before);
        }
        std::puts("native settings: defaults, unaligned buffer/sentinels, write/read/version probe, bounds and host isolation pass");
        return 0;
    } catch(const std::exception &e) { std::fprintf(stderr,"%s\n",e.what());return 1; }
}

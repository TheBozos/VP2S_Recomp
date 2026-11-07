#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "native_math.h"
#include "install_original_functions.h"
#include "install_adapted_functions.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <source_location>
namespace {
void check(bool value,std::source_location at=std::source_location::current()){
    if(!value)throw std::runtime_error("graphics install retirement assertion failed at "+std::to_string(at.line()));
}
struct Host:bozo::ee::Host,bozo::port::Host {
    unsigned enqueues=0;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{return vp2s::nativeMathInstruction(r,c,rt,pc);}
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *c,uint32_t pc)override{
        check(pc==0x201f20 && c->r[4].uw[0]==0x180000 && c->r[5].uw[0]==0x190000);
        ++enqueues;c->r[2].sd[0]=1;c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("unexpected VU execution");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("unexpected device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("unexpected device write");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host oldHost,nativeHost;bozo::ee::Dispatcher old(oldHost),native(nativeHost);
    old.setFunctions(install_original_function_table,install_original_function_count,install_original_resume_table,install_original_resume_count);
    native.setFunctions(install_adapted_function_table,install_adapted_function_count,install_adapted_resume_table,install_adapted_resume_count);
    old.runtime().port=&oldHost;native.runtime().port=&nativeHost;
    old.boot(0x29e2e0,0x1fff000);native.boot(0x29e2e0,0x1fff000);old.bindLibrary(0x201f20);
    for(unsigned seed=0;seed<64;++seed){
        auto *a=old.runtime().memory.flat(),*b=native.runtime().memory.flat();
        std::memset(a,seed+1,bozo::kFlatBytes);uint32_t queue=0x180000;std::memcpy(a+0x35dde0,&queue,4);
        std::memcpy(b,a,bozo::kFlatBytes);
        R5900Context x{},y{};
        for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x1234000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        x.r[4].ud[0]=0x190000;x.r[29].ud[0]=0x1fff000;x.r[31].ud[0]=0x170000;y=x;
        old.runtime().functions.generated(0x29e2e0)(a,&x,&old.runtime());
        native.runtime().functions.generated(0x29e2e0)(b,&y,&native.runtime());
        check(x.pc==0x170000 && y.pc==x.pc && oldHost.enqueues==seed+1 && nativeHost.enqueues==0);
        check(b[0x190004]==0 && b[0x190005]==255);
        check(!std::memcmp(a,b,bozo::kFlatBytes));
        // The void producer's enqueue result and transient call ra are dead in
        // its only audited caller. No completion callback/state is synthesized.
        x.r[2]=y.r[2];check(!std::memcmp(x.r,y.r,sizeof(x.r)));
    }
    for(unsigned seed=0;seed<64;++seed){
        auto *a=old.runtime().memory.flat(),*b=native.runtime().memory.flat();std::memset(a,seed+1,bozo::kFlatBytes);
        uint32_t queue=0x180000,type=0x1591d0;std::memcpy(a+0x35ddd8,&queue,4);std::memcpy(a+0x190000,&type,4);
        a[0x190050]=0;a[0x190051]=seed;std::memcpy(b,a,bozo::kFlatBytes);
        R5900Context x{},y{};x.r[4].ud[0]=0x190000;x.r[5].ud[0]=0;x.r[29].ud[0]=0x1fff000;x.r[31].ud[0]=0x170000;y=x;
        old.runtime().functions.generated(0x29e150)(a,&x,&old.runtime());native.runtime().functions.generated(0x29e150)(b,&y,&native.runtime());
        check(a[0x190050]==1 && b[0x190050]==0 && a[0x190051]==0 && b[0x190051]==0);
        // No native job exists, so its legacy busy store is deliberately absent.
        a[0x190050]=0;check(!std::memcmp(a,b,bozo::kFlatBytes));
        check(x.cycles==y.cycles+8 && x.checkpoints==y.checkpoints+1);x.cycles-=8;--x.checkpoints;
        // Only the original out-call asks the dispatcher for its host slice
        // deadline. This is scheduling metadata, not a program result.
        check(x.cycleLimit==y.cycleLimit || y.cycleLimit==~0ull);x.cycleLimit=y.cycleLimit;
        check(!std::memcmp(&x,&y,sizeof(x)));
    }
    for(unsigned mode:{1u,128u,255u}){
        auto *b=native.runtime().memory.ram();R5900Context c{};c.r[4].ud[0]=0x190000;c.r[5].ud[0]=mode;
        b[0x190050]=0;bool rejected=false;try{vp2s::nativeMathInstruction(b,&c,&native.runtime(),0x29e160);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && b[0x190050]==0);
    }
    std::puts("64 graphics + 64 math-code producer CPU/RAM comparisons; busy store never created and nondefault modes reject; no queue/device completion synthesized");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

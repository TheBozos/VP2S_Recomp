// Real scalar SDK record slice, statically compiled. No device read is answered
// and no VU or timer state is supplied. Compare its callback record transitions.
#include "native_timer.h"
#include "bozo/ee/dispatcher.h"
#include "timer_record_functions.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
namespace {
void check(bool v) { if(!v)throw std::runtime_error("timer callback record mismatch"); }
struct Done {};
struct Host final:bozo::ee::Host {
    uint64_t reply=0,args[5]={};unsigned calls=0;
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *c,uint32_t pc) override {
        if(pc==0x1421b0 || pc==0x1422f8)throw Done{};
        check(pc==0x155000);++calls;
        for(unsigned i=0;i<5;++i)args[i]=c->r[4+i].ud[0];
        check(c->r[28].uw[0]==0x1d0b70);
        c->r[2].ud[0]=reply;c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t) override {throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2]) override {throw std::runtime_error("IO");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2]) override {throw std::runtime_error("IO");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s) override {throw std::runtime_error(s);}
};
void put(uint8_t *r,uint32_t a,uint32_t v){std::memcpy(r+a,&v,4);}
void put64(uint8_t *r,uint32_t a,uint64_t v){std::memcpy(r+a,&v,8);}
}
int main() {
 try {
    for(unsigned seed=0;seed<128;++seed)for(uint64_t reply:{0ull,~0ull,1ull,0x3998ull,0x3999ull,100000ull}) {
        Host refHost,nativeHost;refHost.reply=nativeHost.reply=reply;
        bozo::ee::Dispatcher ref(refHost),actual(nativeHost);
        ref.setFunctions(timer_record_function_table,timer_record_function_count,timer_record_resume_table,timer_record_resume_count);
        for(auto *d:{&ref,&actual}) {d->boot(0x100000,0x1f00000);d->bindLibrary(0x155000);}
        ref.bindLibrary(0x1421b0);ref.bindLibrary(0x1422f8);
        auto &x=ref.runtime();auto &y=actual.runtime();
        auto *ram=x.memory.ram();std::memset(x.memory.flat(),0x30+seed%64,bozo::kFlatBytes);
        constexpr uint32_t node=0x1e4180,next=0x1e41c0;
        const bool tail=seed&1;const uint64_t now=0x100000000ull+seed*512;
        put(ram,0x14a250,node);put(ram,0x14a24c,0x1e4200);put(ram,0x14a248,2);
        put(ram,node,tail?0:next);put(ram,node+4,0);put(ram,node+8,1+seed);put(ram,node+12,3);
        put64(ram,node+16,now-0x10000);put64(ram,node+24,seed*256);put64(ram,node+32,0x8000);
        put(ram,node+40,0x155000);put(ram,node+44,0x1d0b70);put(ram,node+48,seed);
        put(ram,next,0);put(ram,next+4,node);put64(ram,next+16,now);put64(ram,next+24,0);put64(ram,next+32,0x20000);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context a,b;a.pc=0x142230;a.r[16].uw[0]=node;a.r[17].ud[0]=now;a.r[18].uw[0]=0x14a238;
        a.r[20].uw[0]=0x150000;a.r[22].ud[0]=0x12345678;a.r[23].ud[0]=0x3999;
        a.r[29].uw[0]=0x1f00000;b=a;b.r[6].ud[0]=a.r[22].ud[0];
        bool stopped=false;
        try{x.functions.generated(0x142230)(x.memory.flat(),&a,&x);}catch(const Done &){stopped=true;}
        check(stopped);
        const auto following=vp2s::processTimerRecord(nullptr,&b,&y,node,now);
        check(following==(tail?0u:next));
        check(refHost.calls==1 && nativeHost.calls==1);
        check(!std::memcmp(refHost.args,nativeHost.args,sizeof(refHost.args)));
        check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
    }
    std::puts("768 original/native full-RAM callback-record transitions and callback arguments passed");return 0;
 }catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}
}

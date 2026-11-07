#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "display_original_functions.h"
#include "display_adapted_functions.h"
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <set>
#include <source_location>
#include <stdexcept>
#include <vector>
namespace {
void check(bool value,std::source_location source=std::source_location::current()){
    if(!value)throw std::runtime_error("display selection retirement assertion failed at line "+std::to_string(source.line()));
}
constexpr std::array<uint32_t,10> stores{0x1142c0,0x1142d0,0x1142e0,0x1142ec,0x114328,0x114334,0x1144c0,0x1144d0,0x114520,0x11452c};
struct Call{uint32_t pc;std::array<uint32_t,6> arguments;bool operator==(const Call &)const=default;};
struct ClearBoundary{};
struct Host:bozo::ee::Host,bozo::port::Host {
    bool observe=false;unsigned uploads=0,waits=0,descriptors=0;std::set<uint32_t> seen;std::vector<Call> calls;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("unexpected library request");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc)override{
        check(observe && std::find(stores.begin(),stores.end(),pc)!=stores.end());seen.insert(pc);return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("unexpected frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        if(pc==0x122bf8){
            check(observe && c->r[5].uw[0]==0x240000);++uploads;
            // Observe only: no physical graphics memory or completion exists.
            c->r[2].sd[0]=0;c->pc=c->r[31].uw[0];return;
        }
        Call call{pc,{}};for(unsigned i=0;i<6;++i)call.arguments[i]=c->r[4+i].uw[0];calls.push_back(call);
        if(pc==0x100b30)throw ClearBoundary{};
        if(pc==0x100af0){check(c->r[4].uw[0]==0x40000);c->r[2].sd[0]=0x240000;}
        else if(pc==0x132e40){
            check(c->r[4].uw[0]==0x240000 && c->r[5].uw[0]==0 && c->r[6].uw[0]==0x40000);
            std::memset(d.runtime().memory.ram()+0x240000,0,0x40000);c->r[2].sd[0]=0x240000;
        }else if(pc==0x1228d0){
            check(c->r[5].uw[0]==descriptors*1024 && c->r[6].uw[0]==4 &&
                  c->r[7].uw[0]==0 && c->r[8].uw[0]==0 && c->r[9].uw[0]==0 &&
                  c->r[10].uw[0]==256 && c->r[11].uw[0]==256);
            ++descriptors;
            // A typed descriptor-builder leaf, identical on both sides. Its
            // scratch output is still written even though its upload retires.
            std::memcpy(d.runtime().memory.ram()+c->r[4].uw[0],&call.arguments,sizeof(call.arguments));
            c->r[2].sd[0]=0;
        }else if(pc==0x11f4b0){++waits;c->r[2].sd[0]=0;}
        else if(pc==0x1010e0)c->r[2].sd[0]=0x1a0000;
        else if(pc==0x137460)c->r[2].sd[0]=0;
        else if(pc==0x10d0f0)c->r[2].sd[0]=0x1a0000;
        else if(pc==0x112a40 || pc==0x1131c0 || pc==0x122160)c->r[2].sd[0]=0;
        else if(pc==0x114dd0){
            // A software/movie helper that changes its owner's records must
            // still be called, in order, even though display writes retire.
            const uint32_t value=0x13570000|uint32_t(calls.size());
            std::memcpy(d.runtime().memory.ram()+c->r[4].uw[0]+0x3e20,&value,4);c->r[2].sd[0]=0;
        }else throw std::runtime_error("unexpected display leaf");
        c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("unexpected VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("unexpected device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("a physical display write executed");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
void word(uint8_t *r,uint32_t at,uint32_t value){std::memcpy(r+at,&value,4);}
}
int main(){try{
    Host refHost,nativeHost;refHost.observe=true;bozo::ee::Dispatcher before(refHost),after(nativeHost);
    before.setFunctions(display_original_function_table,display_original_function_count,display_original_resume_table,display_original_resume_count);
    after.setFunctions(display_adapted_function_table,display_adapted_function_count,display_adapted_resume_table,display_adapted_resume_count);
    before.runtime().port=&refHost;after.runtime().port=&nativeHost;
    for(auto *d:{&before,&after})for(uint32_t pc:{0x10d0f0u,0x112a40u,0x1131c0u,0x114dd0u,0x122160u,
        0x100af0u,0x132e40u,0x1228d0u,0x137460u,0x122bf8u,0x1010e0u,0x11f4b0u,0x100b30u})d->bindLibrary(pc);
    constexpr uint32_t object=0x180000,control=0x1a0000,picture=0x1b0000,stack=0x1fff000;
    unsigned cases=0,movieCalls=0;
    for(unsigned seed=0;seed<128;++seed){
        auto *a=before.runtime().memory.flat(),*b=after.runtime().memory.flat();std::memset(a,0,bozo::kFlatBytes);
        a[control+0xcc84]=seed&1;a[object+0x3c76]=(seed>>1)&1;
        a[object+0x3e66]=(seed>>2)&1;a[object+0x3e68]=(seed>>3)&1;a[object+0x3e64]=(seed>>4)&1;
        word(a,object+0x3e60,50);word(a,object+0x3e54,seed%4);word(a,object+0x3e58,(seed+1)%4);
        for(unsigned i=0;i<4;++i){const uint32_t base=object+i*0xa70;
            a[base+0x1cf0]=a[base+0x1cf1]=a[base+0x1cf4]=a[base+0x1cf5]=1;
            a[base+0x1cf2]=i&1;a[base+0x1cf7]=(seed>>5)&1;
        }
        a[object+0x3e6b]=1;
        if(seed&64){
            a[object+0x3c76]=1;a[object+0x3e6b]=0;a[object+0x3e6a]=1;
            for(unsigned i=0;i<4;++i)a[object+i*0xa70+0x1cf0]=0;
            word(a,object+0x3c84,picture);a[picture+0x10]=1;a[object+0x3e48]=1;a[object+0x3e30]=2;
        }
        std::memcpy(b,a,bozo::kFlatBytes);R5900Context x{},y{};
        for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x1234567800000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0xd3,sizeof(x.vu0_vf));x.r[4].ud[0]=object;x.r[5].ud[0]=(seed>>1)&1;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;y=x;
        refHost.calls.clear();nativeHost.calls.clear();
        before.runtime().functions.generated(0x114240)(a,&x,&before.runtime());
        after.runtime().functions.generated(0x114240)(b,&y,&after.runtime());
        check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)) && refHost.calls==nativeHost.calls);
        check(!std::memcmp(a,b,bozo::kFlatBytes));
        for(auto &call:refHost.calls)movieCalls+=call.pc==0x114dd0;++cases;
    }
    check(refHost.seen==std::set<uint32_t>(stores.begin(),stores.end()) && movieCalls>0 && nativeHost.seen.empty());
    std::printf("%u full-CPU/RAM display selector comparisons, all 10 retired stores and %u retained software/movie callbacks passed; no GS access\n",cases,movieCalls);
    for(unsigned seed=0;seed<8;++seed){
        auto *a=before.runtime().memory.flat(),*b=after.runtime().memory.flat();
        std::memset(a,0x31+seed,bozo::kFlatBytes);std::memcpy(b,a,bozo::kFlatBytes);
        R5900Context x{},y{};
        for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x1234000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0xd3,sizeof(x.vu0_vf));x.r[29].ud[0]=stack;x.pc=0x100694;y=x;
        refHost.calls.clear();nativeHost.calls.clear();
        refHost.uploads=nativeHost.uploads=refHost.waits=nativeHost.waits=refHost.descriptors=nativeHost.descriptors=0;
        bool originalEnded=false,adaptedEnded=false;
        try{before.runtime().functions.generated(0x100694)(a,&x,&before.runtime());}catch(const ClearBoundary &){originalEnded=true;}
        try{after.runtime().functions.generated(0x100694)(b,&y,&after.runtime());}catch(const ClearBoundary &){adaptedEnded=true;}
        check(originalEnded && adaptedEnded && refHost.uploads==16 && nativeHost.uploads==0 &&
              refHost.waits==16 && nativeHost.waits==16 && refHost.descriptors==16 && nativeHost.descriptors==16);
        // Sixteen removed JAL checkpoints cost eight generated cycles each.
        // Qualify that exact scheduling difference separately; all other CPU
        // fields (including vector/flag state) and every RAM byte must match.
        check(x.cycles==y.cycles+128 && x.checkpoints==y.checkpoints+16);
        auto expected=x;expected.cycles-=128;expected.checkpoints-=16;
        check(!std::memcmp(&expected,&y,sizeof(y)) && !std::memcmp(a,b,bozo::kFlatBytes) && refHost.calls==nativeHost.calls);
    }
    std::puts("8 architectural CPU/full-RAM startup-clear comparisons retain all 128 descriptor builds and waits; exact retired-checkpoint delta checked; no physical memory/completions");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

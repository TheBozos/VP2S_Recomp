#include "native_load_progress.h"
#include "native_seed.h"
#include "bozo/ee/dispatcher.h"
#include <cstdio>
#include <cstring>
#include <limits>
#include <source_location>
#include <vector>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("loading assertion at "+std::to_string(at.line()));}
template<class F>void rejects(F f){bool caught=false;try{f();}catch(const std::exception &){caught=true;}check(caught);}
struct Host:bozo::ee::Host,vp2s::SeedOwner,vp2s::LoadProgressOwner {
    vp2s::NativeLoadProgress progress;uint32_t seed=1;
    uint32_t bootSeed()const override{return seed;}
    vp2s::NativeLoadProgress &nativeLoadProgress()override{return progress;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("library");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("I/O");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("I/O");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &)override{throw std::runtime_error("fault");}
};
}
int main(){try{
    Host host,peer;bozo::ee::Dispatcher d(host),other(peer);auto &rt=d.runtime();std::memset(rt.memory.flat(),0,bozo::kFlatBytes);
    const auto unchanged=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
    constexpr uint32_t stack=0x1f0000,worker=0x190000,destination=0x200000;
    for(unsigned i=0;i<256;++i){
        const uint64_t now=1000ull*i,period=1+i;auto &p=host.progress;
        p.begin(&rt,stack,worker,destination,now,period);check(p.pendingCount()==1 && peer.progress.pendingCount()==0);
        rejects([&]{p.begin(&rt,stack,worker,destination,now,period);});
        check(!p.updateDue(&rt,stack,worker,destination,now+period-1));
        rejects([&]{p.updateDue(&rt,stack,worker+4,destination,now+period);});
        rejects([&]{p.updateDue(&rt,stack,worker,destination+128,now+period);});
        check(p.updateDue(&rt,stack,worker,destination,now+period));check(!p.updateDue(&rt,stack,worker,destination,now+period));
        rejects([&]{p.updateDue(&rt,stack,worker,destination,now);});
        rejects([&]{p.finish(stack,worker,destination,false);});check(p.pendingCount()==1);
        rejects([&]{p.finish(stack,worker+4,destination,true);});check(p.pendingCount()==1);
        check(p.updateDue(&rt,stack,worker,destination,now+2*period));p.finish(stack,worker,destination,true);check(p.pendingCount()==0);
    }
    check(!std::memcmp(unchanged.data(),rt.memory.flat(),unchanged.size()));
    rejects([&]{host.progress.begin(&rt,stack,worker,destination,0,0);});
    rejects([&]{host.progress.begin(&rt,stack,worker,destination+4,0,10);});
    rejects([&]{host.progress.begin(&rt,stack,worker,destination,std::numeric_limits<uint64_t>::max()-1,10);});
    const uint32_t gameState=0x12345678;std::memcpy(rt.memory.ram()+0x35da00,&gameState,4);std::memcpy(other.runtime().memory.ram()+0x35da00,&gameState,4);
    uint32_t choices=0;
    for(unsigned i=0;i<1024;++i){R5900Context a{},b{};std::memset(a.r,0x71,sizeof(a.r));a.r[16].uw[0]=gameState;b=a;
        const auto before=a;check(vp2s::nativeLoadProgressInstruction(&a,&rt,0x306a10)==true);
        check(vp2s::nativeLoadProgressInstruction(&b,&other.runtime(),0x306a10)==true);check(a.r[17].ud[0]==b.r[17].ud[0] && a.r[17].ud[0]<16);
        choices|=1u<<a.r[17].uw[0];a.r[17]=before.r[17];check(!std::memcmp(&a,&before,sizeof(a)));
    }
    check(choices==0xffff);R5900Context c{};rejects([&]{vp2s::nativeLoadProgressInstruction(&c,&rt,0x306a10);});
    std::puts("256 native loading schedule lifetimes and1024 deterministic allocation choices: no RAM/device writes, actual-completion requirement, cadence, ownership, isolation and invalid states passed");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

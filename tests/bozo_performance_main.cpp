#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "performance_original_functions.h"
#include "performance_adapted_functions.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <source_location>
#include <vector>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){
    if(!v)throw std::runtime_error("performance retirement assertion at "+std::to_string(at.line()));
}
struct Call {uint32_t pc,a0,a1,sp;bool operator==(const Call &)const=default;};
template<class T>void put(uint8_t *r,uint32_t at,T value){std::memcpy(r+at,&value,sizeof(value));}
struct Host:bozo::ee::Host,bozo::port::Host {
    std::vector<Call> calls;unsigned writes=0;bool allocationFailure=false;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *,R5900Context *c,Runtime *,uint32_t pc)override{
        check(pc>=0x2bef34 && pc<=0x2bef44 && !((pc-0x2bef34)%4) && c->r[0].ud[0]==0);++writes;return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        calls.push_back({pc,c->r[4].uw[0],c->r[5].uw[0],c->r[29].uw[0]});auto *r=d.runtime().memory.flat();
        if(pc==0x2bf190)c->r[2].sd[0]=2;
        else if(pc==0x100af0)c->r[2].sd[0]=allocationFailure?0:0x1b0000;
        else if(pc==0x2bf2e0){uint32_t n;std::memcpy(&n,r+0x180060,4);for(unsigned i=0;i<n;++i)put<uint32_t>(r,c->r[5].uw[0]+i*4,0x190000+i*0x100);}
        else if(pc==0x170100){put<uint32_t>(r,c->r[4].uw[0]+0x80,0x1234);}
        else check(pc==0x2bf0e0 || pc==0x100be0);
        c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host old,now;bozo::ee::Dispatcher a(old),b(now);
    a.setFunctions(performance_original_function_table,performance_original_function_count,performance_original_resume_table,performance_original_resume_count);
    b.setFunctions(performance_adapted_function_table,performance_adapted_function_count,performance_adapted_resume_table,performance_adapted_resume_count);
    a.runtime().port=&old;b.runtime().port=&now;
    for(auto *d:{&a,&b})for(uint32_t pc:{0x2bf190u,0x100af0u,0x2bf2e0u,0x2bf0e0u,0x100be0u,0x170100u})d->bindLibrary(pc);
    for(unsigned seed=0;seed<64;++seed){
        auto *x=a.runtime().memory.flat(),*y=b.runtime().memory.flat();std::memset(x,0x66,bozo::kFlatBytes);
        old.calls.clear();now.calls.clear();old.writes=now.writes=0;old.allocationFailure=now.allocationFailure=seed%5==0;
        put<uint32_t>(x,0x18000c,seed%5);put<uint32_t>(x,0x180068,seed&1?0x1b0000:0);put<uint32_t>(x,0x180070,seed&2?0x1c0000:0);
        for(unsigned i=0;i<8;++i){const uint32_t child=0x190000+i*0x100;put<uint32_t>(x,child,0x1a0000);x[child+14]=(seed+i)%4!=0;x[child+15]=(seed+i)%8;}
        put<uint32_t>(x,0x1a0014,0x170100);std::memcpy(y,x,bozo::kFlatBytes);
        R5900Context p{},q{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=0x1234000000000000ull+seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        p.r[4].ud[0]=0x180000;p.r[29].ud[0]=0x1fff000;p.r[31].ud[0]=0x170000;q=p;
        a.runtime().functions.generated(0x2bef20)(x,&p,&a.runtime());b.runtime().functions.generated(0x2bef20)(y,&q,&b.runtime());
        check(old.writes==5 && now.writes==0 && old.calls==now.calls);
        check(!std::memcmp(x,y,bozo::kFlatBytes) && !std::memcmp(&p,&q,sizeof(p)));
    }
    std::puts("64 generated object-update CPU/RAM/callback comparisons; five unused write-only profiling resets retired, no counter bank");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

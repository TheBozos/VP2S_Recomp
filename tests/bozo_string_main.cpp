#include "bozo/ee/dispatcher.h"
#include "string_functions.h"
#include <cstdio>
#include <cstring>
#include <source_location>
#include <stdexcept>
namespace {
void check(bool ok,std::source_location at=std::source_location::current()){
    if(!ok)throw std::runtime_error("compiled bounded string assertion at "+std::to_string(at.line()));
}
struct Host:bozo::ee::Host{
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("library override");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device write");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host h;bozo::ee::Dispatcher d(h);d.setFunctions(string_function_table,string_function_count,string_resume_table,string_resume_count);
    auto &r=d.runtime();unsigned cases=0;
    for(unsigned left=0;left<16;++left)for(unsigned right=0;right<16;++right)for(unsigned seed=0;seed<64;++seed){
        const uint32_t a=0x180000+left,b=0x190000+right,n=seed;
        auto *x=r.memory.hostPointer(a,96),*y=r.memory.hostPointer(b,96);
        for(unsigned i=0;i<96;++i)x[i]=y[i]=uint8_t(1+(i*47+seed)%255);
        const auto zero=seed%3==0?seed/2:80u;x[zero]=y[zero]=0;
        if(seed&1)y[seed%48]=uint8_t(x[seed%48]+1);
        if(seed%5==0)y[seed%48]=0xff;
        unsigned i=0;while(i<n && x[i] && x[i]==y[i])++i;
        const int expected=i==n?0:int(x[i])-int(y[i]);
        uint8_t beforeX[96],beforeY[96];std::memcpy(beforeX,x,96);std::memcpy(beforeY,y,96);
        R5900Context c{};for(unsigned k=1;k<32;++k){c.r[k].ud[0]=seed*32+k;c.r[k].ud[1]=~c.r[k].ud[0];}
        std::memset(c.vu0_vf,0x91,sizeof(c.vu0_vf));c.r[4].ud[0]=a;c.r[5].ud[0]=b;c.r[6].ud[0]=n;c.r[29].ud[0]=0x1fff000;c.r[31].ud[0]=0x170000;const auto saved=c;
        r.functions.generated(0x134b60)(r.memory.flat(),&c,&r);
        check(c.pc==0x170000 && int32_t(c.r[2].uw[0])==expected);
        check(!std::memcmp(x,beforeX,96) && !std::memcmp(y,beforeY,96));
        for(unsigned k=16;k<32;++k)check(!std::memcmp(&c.r[k],&saved.r[k],sizeof(c.r[k])));
        check(!std::memcmp(c.vu0_vf,saved.vu0_vf,sizeof(c.vu0_vf)));++cases;
    }
    std::printf("%u original compiled bounded-string cases: all byte alignments, zero bounds, terminators, high bytes and exact unsigned difference; no overrides\n",cases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

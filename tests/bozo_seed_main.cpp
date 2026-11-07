#include "native_seed.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <cstdio>
#include <stdexcept>
namespace {
void check(bool ok){if(!ok)throw std::runtime_error("native seed ABI mismatch");}
struct Host final:bozo::ee::Host, vp2s::SeedOwner {
    uint32_t seed=0;uint32_t bootSeed() const override{return seed;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t) override {throw std::runtime_error("library");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t) override {throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2]) override {throw std::runtime_error("IO");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2]) override {throw std::runtime_error("IO");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s) override {throw std::runtime_error(s);}
};
}
int main(){try{
    Host a,b;bozo::ee::Dispatcher x(a),y(b);a.seed=123;b.seed=456;
    for(uint32_t seed:{0u,1u,0x7fffffffu,0x80000000u,0xffffffffu}) {
        a.seed=seed;R5900Context c;std::memset(&c,0x5a,sizeof(c));
        R5900Context expected=c;expected.r[3].sd[0]=static_cast<int32_t>(seed);
        check(vp2s::nativeBootSeed(nullptr,&c,&x.runtime(),0x1003c4));
        check(!std::memcmp(&c,&expected,sizeof(c)));
        check(vp2s::nativeBootSeed(nullptr,&c,&y.runtime(),0x1003c4) && c.r[3].sd[0]==456);
        expected=c;
        check(!vp2s::nativeBootSeed(nullptr,&c,&x.runtime(),0x1003c8));
        check(!std::memcmp(&c,&expected,sizeof(c)));
    }
    Host first,second,other;first.seed=second.seed=789;other.seed=790;
    bozo::ee::Dispatcher one(first),two(second),three(other);
    bool differs=false;
    for(unsigned draw=0;draw<256;++draw){
        R5900Context c;std::memset(&c,0x4a,sizeof(c));const auto saved=c;
        check(vp2s::nativeBootSeed(nullptr,&c,&one.runtime(),0x2c3cb4));
        const auto sample=c.r[2].ud[0];check(sample<=65535);
        auto expected=saved;expected.r[2].ud[0]=sample;check(!std::memcmp(&c,&expected,sizeof(c)));
        check(vp2s::nativeBootSeed(nullptr,&c,&two.runtime(),0x2c3cb4) && c.r[2].ud[0]==sample);
        check(vp2s::nativeBootSeed(nullptr,&c,&three.runtime(),0x2c3cb4));differs|=sample!=c.r[2].ud[0];
    }
    check(differs);
    bool rejected=false;try{vp2s::nativeBootSeed(nullptr,nullptr,nullptr,0x1003c4);}catch(const std::runtime_error &){rejected=true;}
    check(rejected);std::puts("native boot seed ABI, scope, bounds and per-host ownership passed");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

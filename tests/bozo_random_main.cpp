#include "native_random.h"
#include "bozo/ee/dispatcher.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
namespace {
void check(bool ok){if(!ok)throw std::runtime_error("native procedural random assertion failed");}
uint32_t polynomialStep(uint32_t state){
    uint32_t result=0;
    for(unsigned bit=0;bit<23;++bit)if(state&(1u<<bit)){
        if(bit<22)result^=1u<<(bit+1);
        if(bit==4 || bit==22)result^=1;
    }
    return result;
}
struct Host:bozo::ee::Host,vp2s::RandomOwner {
    vp2s::ProceduralRandom random;
    vp2s::ProceduralRandom &proceduralRandom()override{return random;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("library");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    uint32_t input=1;unsigned steps=0;
    for(unsigned seed=0;seed<1024;++seed){input=input*1664525u+1013904223u;
        vp2s::ProceduralRandom stream;stream.seed(input);uint32_t state=input&0x7fffff;
        for(unsigned step=0;step<256;++step){
            check(std::bit_cast<uint32_t>(stream.current())==(0x3f800000u|state));
            state=polynomialStep(state);const auto value=stream.next();
            check(std::bit_cast<uint32_t>(value)==(0x3f800000u|state) && value>=1.f && value<2.f);++steps;
        }
    }
    Host first,second;bozo::ee::Dispatcher a(first),b(second);
    for(unsigned seed=0;seed<64;++seed){R5900Context ctx{};
        for(unsigned reg=1;reg<32;++reg){ctx.r[reg].ud[0]=0x1234567800000000ull+seed*32+reg;ctx.r[reg].ud[1]=~ctx.r[reg].ud[0];}
        ctx.vu0_vf[31].u[0]=0xdeadbeef;const auto expected=ctx;
        check(vp2s::nativeRandomSeed(nullptr,&ctx,&a.runtime(),0x29e20c));
        check(std::bit_cast<uint32_t>(first.random.current())==(0x3f800000u|(ctx.r[2].uw[0]&0x7fffffu)));
        check(std::bit_cast<uint32_t>(second.random.current())==0x3f800000u);
        for(uint32_t pc:{0x29e210u,0x29e214u})check(vp2s::nativeRandomSeed(nullptr,&ctx,&a.runtime(),pc));
        check(!std::memcmp(&ctx,&expected,sizeof(ctx)));
        check(!vp2s::nativeRandomSeed(nullptr,&ctx,&a.runtime(),0x243f2c)); // Consumers stay guarded.
    }
    second.random.seed(0xffffffffu);first.random.seed(0);check(first.random.next()==1.f && second.random.current()>1.f);
    {   // An unseeded stream (a state older than the random field) refuses to draw.
        vp2s::ProceduralRandom unknown;bool refused=false;try{unknown.next();}catch(const std::runtime_error &){refused=true;}check(refused && !unknown.seeded());}
    {   // mod028 4a45d8 RNEXT.x vf31 / 4a45dc QMFC2.I v1: v1 low word = the next value, other lanes 0.
        Host host;bozo::ee::Dispatcher d(host);auto &rt=d.runtime();
        const uint32_t next=0x4b1f043c,move=0x4823f801;std::memcpy(rt.memory.hostPointer(0x4a45d8,4),&next,4);std::memcpy(rt.memory.hostPointer(0x4a45dc,4),&move,4);
        host.random.seed(0x39d672);vp2s::ProceduralRandom mirror;mirror.seed(0x39d672);R5900Context ctx{};
        {bool refused=false;try{vp2s::nativeRandomNext(nullptr,&ctx,&rt,0x4a45dc);}catch(const std::runtime_error &){refused=true;}check(refused);}   // a move before its draw
        for(unsigned k=0;k<8;++k){ctx.r[3].ud[0]=ctx.r[3].ud[1]=~0ull;
            check(vp2s::nativeRandomNext(nullptr,&ctx,&rt,0x4a45d8) && vp2s::nativeRandomNext(nullptr,&ctx,&rt,0x4a45dc));
            check(ctx.r[3].uw[0]==std::bit_cast<uint32_t>(mirror.next()) && !ctx.r[3].uw[1] && !ctx.r[3].uw[2] && !ctx.r[3].uw[3]);}
        const uint32_t other=0;std::memcpy(rt.memory.hostPointer(0x4a45d8,4),&other,4);check(!vp2s::nativeRandomNext(nullptr,&ctx,&rt,0x4a45d8));   // another overlay
        // mod028 4a741c / 4a7420 QMFC2.I a0 (class 173010's constructor): the same stream into a0.
        const uint32_t moveA0=0x4824f801;std::memcpy(rt.memory.hostPointer(0x4a741c,4),&next,4);std::memcpy(rt.memory.hostPointer(0x4a7420,4),&moveA0,4);
        ctx.r[4].ud[0]=ctx.r[4].ud[1]=~0ull;check(vp2s::nativeRandomNext(nullptr,&ctx,&rt,0x4a741c) && vp2s::nativeRandomNext(nullptr,&ctx,&rt,0x4a7420));
        check(ctx.r[4].uw[0]==std::bit_cast<uint32_t>(mirror.next()) && !ctx.r[4].uw[1] && !ctx.r[4].uw[2] && !ctx.r[4].uw[3]);
    }
    std::printf("%u independent-polynomial random steps and 64 seed ABI/isolation cases passed; no VU state used\n",steps);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

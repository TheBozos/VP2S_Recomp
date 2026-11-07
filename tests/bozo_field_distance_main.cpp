#include "native_math.h"
#include "../port/vp2s_math.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "distance_original_functions.h"
#include "distance_adapted_functions.h"
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <immintrin.h>
#include <source_location>
#include <stdexcept>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){
    if(!v)throw std::runtime_error("field distance assertion at "+std::to_string(at.line()));
}
uint32_t square(uint32_t b){
    const unsigned e=(b>>23)&255;if(!e)return 0;
    uint64_t p=uint64_t((b&0x7fffff)|0x800000)*((b&0x7fffff)|0x800000);
    int exponent=int(e)*2-127;unsigned shift=23;
    if(p&(1ull<<47)){++exponent;++shift;}
    if(exponent<=0)return 0;check(exponent<255);
    return uint32_t(exponent<<23)|uint32_t((p>>shift)&0x7fffff);
}
uint32_t sum(uint32_t a,uint32_t b){
    if(a<b)std::swap(a,b);if(!b)return a;unsigned e=a>>23;const unsigned distance=e-(b>>23);
    uint32_t m=(a&0x7fffff)|0x800000;if(distance<=23)m+=((b&0x7fffff)|0x800000)>>distance;
    if(m&0x1000000){m>>=1;++e;}return (e<<23)|(m&0x7fffff);
}
uint32_t root(uint32_t bits){
    const double target=std::bit_cast<float>(bits);uint32_t low=0,high=0x7f7fffff;
    while(low<high){const auto mid=low+(high-low+1)/2;const double value=std::bit_cast<float>(mid);
        if(value*value<=target)low=mid;else high=mid-1;}
    return low;
}
struct Host:bozo::ee::Host,bozo::port::Host {
    bool reference=false;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("library");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(reference && pc==0x379280){
            // The numerical quaternion API is separately qualified. This
            // comparison isolates this caller's CPU builder/output ABI.
            __m128 input;std::memcpy(&input,rt->memory.hostPointer(c->r[19].uw[0]+0x190,16),16);
            const auto result=vp2s::math::quaternionMatrix(input);
            std::memcpy(rt->memory.hostPointer(c->r[2].uw[0],48),result.data(),48);return true;
        }
        if(reference){const auto saved=_mm_getcsr();check(pc==0x3789a0 || pc==0x3c5d78);uint32_t bits[4];std::memcpy(bits,rt->memory.hostPointer(c->r[5].uw[0],16),16);
            const auto x=square(bits[0]),y=square(bits[1]),z=square(bits[2]);
            bits[0]=root(sum(sum(x,y),z));bits[1]=y;bits[2]=z;
            // Searching down to an exact zero traverses subnormal candidates;
            // numerical oracle work must not add host FP flags to the CPU ABI.
            std::memcpy(rt->memory.hostPointer(c->r[4].uw[0],16),bits,16);_mm_setcsr(saved);return true;}
        check(vp2s::nativeMathInstruction(r,c,rt,pc));return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("call");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host host;bozo::ee::Dispatcher original(host),adapted(host);
    original.setFunctions(distance_original_function_table,distance_original_function_count,distance_original_resume_table,distance_original_resume_count);
    adapted.setFunctions(distance_adapted_function_table,distance_adapted_function_count,distance_adapted_resume_table,distance_adapted_resume_count);
    original.runtime().port=adapted.runtime().port=&host;unsigned cases=0;
    for(bool movement:{false,true})for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<64;++seed){
        auto &a=original.runtime();auto &b=adapted.runtime();std::memset(a.memory.flat(),seed+1,bozo::kFlatBytes);
        const uint32_t object=0x180000,source=0x190000,owner=0x1a0000,position=0x1b0000,flags=0x1c0000;
        const uint32_t stack=seed&1?0x70003f00:0x1fff000;
        std::memcpy(a.memory.ram()+0x48e048,&owner,4);std::memcpy(a.memory.ram()+owner+0x14,&position,4);std::memcpy(a.memory.ram()+object,&flags,4);
        float point[4]={float(int(seed)-32)/8.f,float(seed%11)*-0.75f,float(seed%7)*1.25f,std::bit_cast<float>(0x7fc01234u)};
        const float other[3]={float(seed%3),float(seed%5),float(seed%7)};
        if(seed%8==0)std::memcpy(point,other,12);
        std::memcpy(a.memory.ram()+object+0x10,point,16);std::memcpy(a.memory.ram()+position+0x250,other,12);
        if(movement){std::memcpy(a.memory.ram()+object+0x30,point,16);std::memcpy(a.memory.ram()+object+0x10,other,12);a.memory.ram()[object+0xbc]=uint8_t(seed*17);}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0xaa,sizeof(x.vu0_vf));x.vu0_acc.u[1]=0xabcd;x.vu0_status=0x7654;
        x.r[16].ud[0]=movement?object:source;x.r[17].ud[0]=object;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        const auto entry=movement?0x3c5d08:0x378908;
        host.reference=true;a.functions.generated(entry)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(entry)(b.memory.flat(),&y,&b);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)));
        check(!std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));++cases;
        if(movement)check((b.memory.ram()[object+0xbc]&1)==(seed%8!=0) && (b.memory.ram()[object+0xbc]&254)==((seed*17)&254));
    }
    unsigned rotations=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<64;++seed){
        auto &a=original.runtime();auto &b=adapted.runtime();const uint32_t object=0x180000,stack=seed&1?0x70003d00:0x1fff000;
        std::memset(a.memory.flat(),seed+1,bozo::kFlatBytes);
        float q[4]={float(int(seed%9)-4)/8.f,float(int(seed%11)-5)/8.f,float(int(seed%7)-3)/8.f,float(seed%13)/8.f};
        if(!seed){q[0]=-0.f;q[1]=q[2]=0.f;q[3]=1.f;}
        std::memcpy(a.memory.hostPointer(stack+0x160,16),q,16);std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0xcc,sizeof(x.vu0_vf));x.vu0_acc.u[0]=0x9876;x.vu0_status=0x5432;
        x.r[19].ud[0]=object;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        host.reference=true;a.functions.generated(0x379228)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(0x379228)(b.memory.flat(),&y,&b);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)));
        check(!std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        float row[4];std::memcpy(row,b.memory.ram()+object+0x1f0,16);check(row[0]==0 && row[1]==0 && row[2]==0 && row[3]==1);++rotations;
    }
    std::printf("%u whole CPU/RAM field-distance consumers and%u inline quaternion matrix callers; four rounding modes, scratchpad stacks, no VU/device state\n",cases,rotations);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

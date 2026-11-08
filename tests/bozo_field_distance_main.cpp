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
uint32_t product(uint32_t a,uint32_t b){
    const auto sign=(a^b)&0x80000000u;const unsigned ea=(a>>23)&255,eb=(b>>23)&255;if(!ea || !eb)return sign;
    uint64_t p=uint64_t((a&0x7fffff)|0x800000)*((b&0x7fffff)|0x800000);int exponent=int(ea)+int(eb)-127;unsigned shift=23;
    if(p&(1ull<<47)){++exponent;++shift;}if(exponent<=0)return sign;check(exponent<255);
    return sign|uint32_t(exponent<<23)|uint32_t((p>>shift)&0x7fffff);
}
uint32_t signedSum(uint32_t a,uint32_t b){
    if((a&0x7fffffff)<(b&0x7fffffff))std::swap(a,b);
    if(!(b&0x7fffffff))return (a&0x7fffffff)?a:(a&b&0x80000000u);
    int exponent=int((a>>23)&255);const auto distance=unsigned(exponent)-((b>>23)&255);
    const int64_t large=(a&0x7fffff)|0x800000,small=distance<=23?(((b&0x7fffff)|0x800000)>>distance):0;
    int64_t value=(a>>31?-large:large)+(b>>31?-small:small);if(!value)return 0;
    const auto sign=value<0?0x80000000u:0u;uint64_t magnitude=value<0?-value:value;
    while(magnitude>=0x1000000){magnitude>>=1;++exponent;}
    while(magnitude<0x800000){magnitude<<=1;--exponent;}
    if(exponent<=0)return sign;check(exponent<255);return sign|uint32_t(exponent<<23)|uint32_t(magnitude&0x7fffff);
}
uint32_t reciprocal(uint32_t bits){
    if(!(bits&0x7fffffff))return 0x7f7fffff;
    const double denominator=std::bit_cast<float>(bits);uint32_t lo=0,hi=0x7f7fffff;
    while(lo<hi){const auto mid=lo+(hi-lo+1)/2;const double value=std::bit_cast<float>(mid);
        if(value*denominator<=1.)lo=mid;else hi=mid-1;}return lo;
}
struct Host:bozo::ee::Host,bozo::port::Host {
    bool reference=false;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("library");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(reference && pc==0x37fad8){
            const auto saved=_mm_getcsr();uint32_t corners[8],out[4],squares[3];
            std::memcpy(corners,rt->memory.hostPointer(c->r[4].uw[0],32),32);
            for(unsigned i=0;i<3;++i){out[i]=product(signedSum(corners[i+4],corners[i]),0x3f000000);
                const auto extent=product(signedSum(corners[i+4],corners[i]^0x80000000u),0x3f000000);squares[i]=product(extent,extent);}
            const auto squared=signedSum(signedSum(squares[0],squares[1]),squares[2]);
            out[3]=product(squared,reciprocal(root(squared)));
            std::memcpy(rt->memory.hostPointer(c->r[7].uw[0],16),out,16);_mm_setcsr(saved);return true;
        }
        if(reference && pc==0x410e4c){
            uint32_t matrix[4][4],vector[4],result[4];std::memcpy(matrix,rt->memory.hostPointer(c->r[4].uw[0],64),64);std::memcpy(vector,rt->memory.hostPointer(c->r[5].uw[0],16),16);
            const unsigned order[4][4]={{0,1,2,3},{1,0,2,3},{2,1,0,3},{3,1,2,0}};
            for(unsigned row=0;row<4;++row){uint32_t p[4];for(unsigned i=0;i<4;++i)p[i]=product(matrix[row][i],vector[i]);
                auto s=signedSum(p[order[row][0]],p[order[row][1]]);s=signedSum(s,p[order[row][2]]);result[row]=signedSum(s,p[order[row][3]]);}
            std::memcpy(rt->memory.hostPointer(c->r[6].uw[0],16),result,16);return true;
        }
        if(reference && pc==0x379280){
            // The numerical quaternion API is separately qualified. This
            // comparison isolates this caller's CPU builder/output ABI.
            __m128 input;std::memcpy(&input,rt->memory.hostPointer(c->r[19].uw[0]+0x190,16),16);
            const auto result=vp2s::math::quaternionMatrix(input);
            std::memcpy(rt->memory.hostPointer(c->r[2].uw[0],48),result.data(),48);return true;
        }
        if(reference){const auto saved=_mm_getcsr();check(pc==0x3789a0 || pc==0x3c5d78 || pc==0x37b568 || pc==0x382040);uint32_t bits[4];std::memcpy(bits,rt->memory.hostPointer(c->r[pc==0x382040?4:5].uw[0],16),16);
            const auto x=square(bits[0]),y=square(bits[1]),z=square(bits[2]);
            bits[0]=root(sum(sum(x,y),z));bits[1]=y;bits[2]=z;
            // Searching down to an exact zero traverses subnormal candidates;
            // numerical oracle work must not add host FP flags to the CPU ABI.
            std::memcpy(rt->memory.hostPointer(c->r[pc==0x382040?3:4].uw[0],16),bits,16);_mm_setcsr(saved);return true;}
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
    for(unsigned profile=0;profile<4;++profile)for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<64;++seed){
        const bool movement=profile==1,camera=profile==2,setup=profile==3;
        auto &a=original.runtime();auto &b=adapted.runtime();std::memset(a.memory.flat(),seed+1,bozo::kFlatBytes);
        const uint32_t object=0x180000,source=0x190000,owner=0x1a0000,position=0x1b0000,flags=0x1c0000;
        const uint32_t stack=seed&1?0x70003f00:0x1fff000;
        std::memcpy(a.memory.ram()+0x48e048,&owner,4);std::memcpy(a.memory.ram()+owner+0x14,&position,4);std::memcpy(a.memory.ram()+object,&flags,4);
        float point[4]={float(int(seed)-32)/8.f,float(seed%11)*-0.75f,float(seed%7)*1.25f,std::bit_cast<float>(0x7fc01234u)};
        const float other[3]={float(seed%3),float(seed%5),float(seed%7)};
        if(seed%8==0)std::memcpy(point,other,12);
        std::memcpy(a.memory.ram()+object+0x10,point,16);std::memcpy(a.memory.ram()+position+0x250,other,12);
        if(movement){std::memcpy(a.memory.ram()+object+0x30,point,16);std::memcpy(a.memory.ram()+object+0x10,other,12);a.memory.ram()[object+0xbc]=uint8_t(seed*17);}
        if(camera){std::memcpy(a.memory.ram()+object+0x40,point,16);std::memcpy(a.memory.ram()+object+0x250,other,12);}
        if(setup){std::memcpy(a.memory.ram()+object+0xc,&position,4);std::memcpy(a.memory.hostPointer(stack+0x90,12),other,12);
            for(unsigned i=0;i<3;++i)std::memcpy(a.memory.ram()+position+0xdc+i*16,point+i,4);}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0xaa,sizeof(x.vu0_vf));x.vu0_acc.u[1]=0xabcd;x.vu0_status=0x7654;
        x.r[16].ud[0]=profile?object:source;x.r[17].ud[0]=object;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        const auto entry=setup?0x381fb8:camera?0x37b500:movement?0x3c5d08:0x378908;
        host.reference=true;a.functions.generated(entry)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(entry)(b.memory.flat(),&y,&b);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)));
        check(!std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));++cases;
        if(movement)check((b.memory.ram()[object+0xbc]&1)==(seed%8!=0) && (b.memory.ram()[object+0xbc]&254)==((seed*17)&254));
        if(camera)check(!std::memcmp(b.memory.ram()+object+0x33c,b.memory.hostPointer(stack+0x20,4),4));
        if(setup)check(!std::memcmp(b.memory.ram()+object+0x58,b.memory.hostPointer(stack+0x50,4),4));
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
    unsigned dots=0;uint32_t random=71;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=original.runtime();auto &b=adapted.runtime();std::memset(a.memory.flat(),seed+1,bozo::kFlatBytes);
        const uint32_t matrix=seed&1?0x70000100:0x180000,vector=0x190000;
        const uint32_t destination=seed%3==0?matrix+(seed%4)*16:seed%3==1?vector:0x1a0000;
        uint32_t values[20];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((96+random%56)<<23);}
        if(seed%8==0){for(unsigned i=0;i<20;++i)values[i]=std::bit_cast<uint32_t>(i>=16?1.f:(i%2?-1.f:1.f));}
        if(seed%8==1){for(unsigned i=16;i<20;++i)values[i]=(i&1)?0x80000000u:0;}
        std::memcpy(a.memory.hostPointer(matrix,64),values,64);std::memcpy(a.memory.hostPointer(vector,16),values+16,16);std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0xf3,sizeof(x.vu0_vf));x.vu0_acc.u[0]=0x4321;x.vu0_status=0x1234;
        x.r[4].ud[0]=matrix;x.r[5].ud[0]=vector;x.r[6].ud[0]=destination;x.r[31].ud[0]=0x170000;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        host.reference=true;a.functions.generated(0x410df0)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(0x410df0)(b.memory.flat(),&y,&b);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)));
        check(!std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));++dots;
    }
    unsigned boxes=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=original.runtime(),&b=adapted.runtime();std::memset(a.memory.flat(),0x91,bozo::kFlatBytes);
        const uint32_t box=seed&1?0x70000100:0x180000,stack=seed&2?0x70003e00:0x1fff000;
        uint32_t values[8];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((100+random%30)<<23);}
        if(seed%8==0)for(unsigned i=0;i<3;++i)values[i+4]=values[i];
        if(seed%8==1)for(unsigned i=0;i<8;++i)values[i]=(i&1)?0x80000000u:0;
        values[3]=0x7fc01234;values[7]=0xffc05678;
        std::memcpy(a.memory.hostPointer(box,32),values,32);std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0x41,sizeof(x.vu0_vf));x.vu0_status=0x6543;
        x.r[4].ud[0]=box;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        host.reference=true;a.functions.generated(0x37fa30)(a.memory.flat(),&x,&a);const auto expected=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(0x37fa30)(b.memory.flat(),&y,&b);check(_mm_getcsr()==expected);_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)));
        check(!std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));++boxes;
    }
    {auto &r=adapted.runtime();R5900Context c{};c.r[4].ud[0]=0x180000;c.r[8].ud[0]=0x180010;c.r[7].ud[0]=0x180020;c.r[9].ud[0]=0x190000;
        std::memset(r.memory.hostPointer(0x180000,64),0,64);float half=0.5f;std::memcpy(r.memory.hostPointer(0x190000,4),&half,4);
        auto reject=[&](const char *label){auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x37fad8);}catch(const std::runtime_error &){bad=true;}
            if(!bad)std::fprintf(stderr,"sphere rejection unexpectedly accepted: %s\n",label);
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[7].ud[0]+=16;reject("output");c.r[7].ud[0]-=16;
        half=0.25f;std::memcpy(r.memory.hostPointer(0x190000,4),&half,4);reject("coefficient");half=0.5f;std::memcpy(r.memory.hostPointer(0x190000,4),&half,4);
        uint32_t value=0x7fc00000;std::memcpy(r.memory.hostPointer(0x180000,4),&value,4);reject("NaN");
        value=0x7f7fffff;std::memcpy(r.memory.hostPointer(0x180000,4),&value,4);reject("squared overflow");
    }
    std::printf("%u distances,%u rotations,%u row dots,%u box/sphere whole CPU/RAM cases; integer arithmetic, four rounding modes and mutation-free sphere rejection; no VU/device state\n",cases,rotations,dots,boxes);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

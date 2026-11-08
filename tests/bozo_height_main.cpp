// Complete original scalar height-table loop, optional update and bound stores.
#include "native_math.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "height_reference_functions.h"
#include "height_adapted_functions.h"
#include <bit>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <immintrin.h>
#include <source_location>
#include <stdexcept>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){
    if(!v)throw std::runtime_error("height bounds assertion at "+std::to_string(at.line()));
}
template<class T>void put(Runtime &r,uint32_t at,T v){auto *p=r.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(p,&v,sizeof(v));}
uint32_t word(Runtime &r,uint32_t at){uint32_t v;std::memcpy(&v,r.memory.hostPointer(at,4),4);return v;}
int16_t half(Runtime &r,uint32_t at){int16_t v;std::memcpy(&v,r.memory.hostPointer(at,2),2);return v;}
uint32_t product(uint32_t a){unsigned e=(a>>23)&255;if(!e)return 0;uint64_t m=uint64_t((a&0x7fffff)|0x800000)*((a&0x7fffff)|0x800000);int exponent=2*int(e)-127;unsigned shift=23;
    if(m&(1ull<<47)){++exponent;++shift;}return exponent<=0?0:uint32_t(exponent<<23)|uint32_t((m>>shift)&0x7fffff);}
uint32_t add(uint32_t a,uint32_t b){if(a<b)std::swap(a,b);if(!b)return a;int e=(a>>23)&255;unsigned d=unsigned(e)-((b>>23)&255);
    uint64_t m=((a&0x7fffff)|0x800000)+(d<=23?(((b&0x7fffff)|0x800000)>>d):0);if(m>=0x1000000){m>>=1;++e;}return uint32_t(e<<23)|uint32_t(m&0x7fffff);}
uint32_t scale(Runtime &r,uint32_t object){uint32_t sum=0;for(unsigned i=0;i<3;++i)sum=add(sum,product(word(r,object+0x94+i*16)));
    if(!sum)return 0; // DAZ must not turn subnormal search candidates into zero roots.
    const double squared=std::bit_cast<float>(sum);uint32_t lo=0,hi=0x7f7fffff;
    while(lo<hi){auto mid=lo+(hi-lo+1)/2;const double v=std::bit_cast<float>(mid);if(v*v<=squared)lo=mid;else hi=mid-1;}return lo;}
struct Host final:bozo::ee::Host,bozo::port::Host{
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("unexpected native library ownership");}
    bool reference=false;unsigned callbacks=0;
    bool nativeInstruction(uint8_t *ram,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(reference){check(pc==0x3e5cc0);put(*rt,c->r[29].uw[0]+0x54,scale(*rt,c->r[18].uw[0]));return true;}
        check(vp2s::nativeMathInstruction(ram,c,rt,pc));return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        check(pc==0x172000);const auto object=c->r[4].uw[0];
        // Real indirect update ABI, with a bounded matrix/translation producer.
        for(unsigned i=0;i<3;++i)put(d.runtime(),object+0x94+i*16,std::bit_cast<uint32_t>(i==1?2.f:0.f));
        put(d.runtime(),object+0xec,0x41200000u);++callbacks;c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("IO");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("IO");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host host;bozo::ee::Dispatcher a(host),b(host);
    a.setFunctions(height_reference_function_table,height_reference_function_count,height_reference_resume_table,height_reference_resume_count);
    b.setFunctions(height_adapted_function_table,height_adapted_function_count,height_adapted_resume_table,height_adapted_resume_count);
    for(auto *d:{&a,&b}){d->runtime().port=&host;d->bindLibrary(0x172000);}
    unsigned cases=0;uint32_t random=91;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x53,bozo::kFlatBytes);
        const uint32_t object=seed%3==0?0x70000100:0x180000,table=0x190000,out=seed%5==0?object+0x180:0x1a0004;
        const uint32_t stack=seed&1?0x70003f00:0x1fff000,count=seed%17;
        put(x,object,0x1b0000u);put(x,0x1b0064,0x172000u);put(x,object+0x250,table);put<uint8_t>(x,object+0x61,seed%4?1:0);
        put(x,table+8,count);put(x,table+12,1u);
        for(unsigned i=0;i<count;++i)put<int16_t>(x,table+0x20+i*2,i%5==0?0x7fff:int16_t(int(i)*19-int(seed)*2));
        for(unsigned row=0;row<3;++row)for(unsigned lane=0;lane<4;++lane){random=random*1664525u+1013904223u;
            uint32_t value=lane==1?std::bit_cast<uint32_t>(float(int(random%400)-200)/64.f):0x7f805678u;
            if(seed%8==0 && lane==1)value=row==1?0x3f800000u:0;
            if(seed%8==1 && lane==1)value=0x80000000u;
            if(seed%8==2 && lane==1)value=std::bit_cast<uint32_t>(1.e-30f);
            if(seed%8==3 && lane==1)value=std::bit_cast<uint32_t>(row==0?3.f:row==1?-4.f:0.f);
            put(x,object+0x90+row*16+lane*4,value);}
        put(x,object+0xdc,0x7f801234u);put(x,object+0xec,std::bit_cast<uint32_t>(float(int(seed)-64)));put(x,object+0xfc,0x7fc04567u);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}std::memset(p.vu0_vf,0x47,sizeof(p.vu0_vf));
        p.r[4].ud[0]=out;p.r[5].ud[0]=object;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040u))|mode|(seed&1?0x8040u:0u);_mm_setcsr(control);
        host.reference=true;x.functions.generated(0x3e5bc0)(x.memory.flat(),&p,&x);const auto expected=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;y.functions.generated(0x3e5bc0)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expected);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;
        if(std::memcmp(&p,&q,sizeof(p))){const auto *u=reinterpret_cast<const uint8_t *>(&p),*v=reinterpret_cast<const uint8_t *>(&q);
            for(unsigned i=0;i<sizeof(p);++i)if(u[i]!=v[i]){std::fprintf(stderr,"seed=%u mode=%x CPU byte=%u old=%02x new=%02x pc=%x/%x\n",seed,mode,i,u[i],v[i],p.pc,q.pc);break;}}
        check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));++cases;
        if(seed==8)check(half(y,out+4)==244 && half(y,out+6)==16); // update publishes scale 2, Y 10
        if(seed==1)check(half(y,out+4)==-63 && half(y,out+6)==-63);
    }
    // Rejections must leave every CPU/RAM byte unchanged at the numeric boundary.
    auto &r=b.runtime();R5900Context c{};c.r[18].ud[0]=0x180000;c.r[4].ud[0]=0x180090;c.r[19].ud[0]=0x190000;c.r[29].ud[0]=0x1fff000;c.r[3].ud[0]=0x1fff050;
    for(unsigned i=0;i<3;++i)put(r,0x180094+i*16,0u);
    const auto reject=[&]{const auto before=c;std::vector<uint8_t> memory(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
        try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x3e5cc0);}catch(const std::runtime_error &){bad=true;}
        check(bad && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(memory.data(),r.memory.flat(),memory.size()));};
    c.r[4].ud[0]+=16;reject();c.r[4].ud[0]-=16;c.r[3].ud[0]+=16;reject();c.r[3].ud[0]-=16;
    for(auto bits:{0x7fc00000u,1u,0x7f000000u}){put(r,0x180094,bits);reject();}put(r,0x180094,0u);
    c.r[19].ud[0]=0x1fff050;reject();c.r[19].ud[0]=0x190000;c.r[18].ud[0]=0x1fff000;c.r[4].ud[0]=0x1fff090;reject();
    std::printf("%u complete height-bound callers, %u bounded update calls: four modes, full CPU/RAM, opaque unused lanes and rejection preservation\n",cases,host.callbacks);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

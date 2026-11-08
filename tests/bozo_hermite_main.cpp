#include "native_math.h"
#include "native_float_oracle.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "hermite_original_functions.h"
#include "hermite_adapted_functions.h"
#include <bit>
#include <cstring>
#include <cstdio>
#include <span>
#include <source_location>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("Hermite vector assertion at "+std::to_string(at.line()));}
template<class T>void put(Runtime &rt,uint32_t at,T v){std::memcpy(rt.memory.hostPointer(at,sizeof(T)),&v,sizeof(T));}
void reference(Runtime *rt,R5900Context *c){
    uint32_t points[4][4],weights[4],out[4];std::memcpy(points,rt->memory.hostPointer(c->r[16].uw[0],64),64);std::memcpy(weights,rt->memory.hostPointer(0x35dfd0,16),16);
    for(unsigned lane=0;lane<4;++lane){auto value=vp2s::test::product(points[0][lane],weights[0]);
        for(unsigned i=1;i<4;++i)value=vp2s::test::sum(value,vp2s::test::product(points[i][lane],weights[i]));out[lane]=value;}
    std::memcpy(rt->memory.hostPointer(c->r[29].uw[0]+0x30,16),out,16);
}
struct Host:bozo::ee::Host,bozo::port::Host {
    bool old=false;unsigned publications=0;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("library");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(old){check(pc==0x2a68fc);reference(rt,c);}else check(vp2s::nativeMathInstruction(r,c,rt,pc));
        if(pc==0x2a68fc)++publications;return true;
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
    Host old,now;old.old=true;bozo::ee::Dispatcher a(old),b(now);
    a.setFunctions(hermite_original_function_table,hermite_original_function_count,hermite_original_resume_table,hermite_original_resume_count);
    b.setFunctions(hermite_adapted_function_table,hermite_adapted_function_count,hermite_adapted_resume_table,hermite_adapted_resume_count);
    a.runtime().port=&old;b.runtime().port=&now;
    const auto saved=_mm_getcsr();unsigned callers=0,records=0,endpoints=0,extrapolated=0;uint32_t random=0x2a68a0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime();auto &y=b.runtime();std::memset(x.memory.flat(),0x65,bozo::kFlatBytes);
        const uint32_t top=seed&1?0x70003e00:0x1fff000,stack=top-0x40;
        const uint32_t curve=seed%4==0?0x180000:seed%4==1?0x70000100:seed%4==2?stack+0x30:0x35dfc0;
        const uint32_t destination=seed%5==0?0x190000:seed%5==1?curve:seed%5==2?curve+0x10:seed%5==3?0x35dfd0:stack+0x30;
        float points[4][4];for(auto &point:points)for(auto &value:point){random=random*1664525u+1013904223u;value=std::bit_cast<float>((random&0x807fffff)|((120+random%9)<<23));}
        std::memcpy(x.memory.hostPointer(curve,64),points,64);
        const float start=float(int(seed%7)-3)/4,duration=float(seed%5+1)/2;
        const float t[]={0,1,0.5f,-0.25f,1.25f,0.125f,0.75f,2};const auto time=seed%8==7?start+float(seed+1)/37:start+duration*t[seed%8];
        put(x,curve+0x40,start);put(x,curve+0x48,duration);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];p.f[i]=float(i)/8;}
        std::memset(p.vu0_vf,0x6c,sizeof(p.vu0_vf));p.vu0_status=0x9876;p.r[4].ud[0]=destination;p.r[5].ud[0]=curve;p.r[29].ud[0]=top;p.r[31].ud[0]=0x170000;p.f[12]=time;auto q=p;
        old.publications=now.publications=0;const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);_mm_setcsr(control);
        x.functions.generated(0x2a68a0)(x.memory.flat(),&p,&x);const auto after=_mm_getcsr();_mm_setcsr(control);
        try{y.functions.generated(0x2a68a0)(y.memory.flat(),&q,&y);}catch(const std::exception &e){std::fprintf(stderr,"Hermite seed=%u mode=%x: %s\n",seed,mode,e.what());throw;}
        if(_mm_getcsr()!=after || std::memcmp(&p,&q,sizeof(p)) || std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes)){
            unsigned cpu=0,ram=0;while(cpu<sizeof(p) && reinterpret_cast<uint8_t *>(&p)[cpu]==reinterpret_cast<uint8_t *>(&q)[cpu])++cpu;
            while(ram<bozo::kFlatBytes && x.memory.flat()[ram]==y.memory.flat()[ram])++ram;
            std::fprintf(stderr,"Hermite mismatch seed=%u mode=%x FP=%x/%x CPU=%x RAM=%x\n",seed,mode,after,_mm_getcsr(),cpu,ram);check(false);}
        check(p.pc==0x170000 && old.publications==1 && now.publications==1);
        if(seed%8<2 && curve!=0x35dfc0){float out[4];std::memcpy(out,y.memory.hostPointer(destination,16),16);for(unsigned i=0;i<4;++i)check(out[i]==points[seed%8][i]);++endpoints;}
        if(seed%8==3 || seed%8==4)++extrapolated;++callers;
    }
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<256;++seed){
        auto &x=a.runtime();auto &y=b.runtime();std::memset(x.memory.flat(),0x65,bozo::kFlatBytes);const uint32_t stack=seed&1?0x70003000:0x1ffe000,curve=seed%3==0?stack+0x30:seed%3==1?0x180000:0x35dfc0;
        uint32_t points[16],weights[4];for(auto &v:points){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((104+random%25)<<23);}
        for(auto &v:weights){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((104+random%25)<<23);}
        if(seed%16==0){std::memset(points,0,64);std::memset(weights,0,16);points[0]=weights[0]=0x3f800000;}
        std::memcpy(x.memory.hostPointer(curve,64),points,64);std::memcpy(x.memory.hostPointer(0x35dfd0,16),weights,16);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};p.pc=0x2a68c8;p.r[16].ud[0]=curve;p.r[29].ud[0]=stack;p.r[4].ud[0]=0x35dfd0;p.r[3].ud[0]=stack+0x30;auto q=p;
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);_mm_setcsr(control);reference(&x,&p);
        check(vp2s::nativeMathInstruction(y.memory.flat(),&q,&y,0x2a68fc));check(_mm_getcsr()==control && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));++records;
    }
    _mm_setcsr(saved);
    {auto &rt=b.runtime();R5900Context c{};c.pc=0x2a68c8;c.r[16].uw[0]=0x180000;c.r[29].uw[0]=0x1ffe000;c.r[4].uw[0]=0x35dfd0;c.r[3].uw[0]=0x1ffe030;
        uint32_t points[16]={},weights[4]={0x3f800000,0,0,0};std::memcpy(rt.memory.hostPointer(0x180000,64),points,64);std::memcpy(rt.memory.hostPointer(0x35dfd0,16),weights,16);
        auto reject=[&]{const auto before=c;std::vector<uint8_t> ram(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2a68fc);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));};
        c.pc=0x170000;reject();c.pc=0x2a68c8;c.r[4].uw[0]+=16;reject();c.r[4].uw[0]-=16;c.r[3].uw[0]+=16;reject();c.r[3].uw[0]-=16;
        for(uint32_t address:{0x180000u,0x35dfd0u}){
            for(uint32_t bad:{1u,0x7fc01234u,0x7f7fffffu}){put(rt,address,bad);reject();}put(rt,address,uint32_t(0));}
    }
    std::printf("%u complete Hermite callers (%u endpoints,%u extrapolated) + %u full XYZW records,10 sites; CPU/RAM/FP, four modes/FTZ/DAZ, original scalar weights and aliases passed\n",callers,endpoints,extrapolated,records);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

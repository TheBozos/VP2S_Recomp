#include "native_math.h"
#include "native_float_oracle.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "particle_lighting_functions.h"
#include <bit>
#include <cstdio>
#include <cstring>
#include <set>
#include <source_location>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("particle lighting assertion at "+std::to_string(at.line()));}
template<class T>void write(Runtime &rt,uint32_t at,const T &v){auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(p,&v,sizeof(v));}
struct Host:bozo::ee::Host,bozo::port::Host {
    bool reference=false;std::set<uint32_t> sites;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("library");}bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        sites.insert(pc);if(!reference){check(vp2s::nativeMathInstruction(r,c,rt,pc));return true;}
        if(pc==0x22f30c){uint32_t colour[4],light[4],out[4];std::memcpy(colour,rt->memory.hostPointer(c->r[3].uw[0],16),16);std::memcpy(light,rt->memory.hostPointer(c->r[4].uw[0],16),16);
            for(unsigned i=0;i<4;++i)out[i]=vp2s::test::product(colour[i],light[i]);write(*rt,c->r[2].uw[0],out);}return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("call");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host old,now;old.reference=true;bozo::ee::Dispatcher a(old),b(now);
    for(auto *d:{&a,&b}){d->runtime().port=d==&a?&old:&now;d->setFunctions(particle_lighting_function_table,particle_lighting_function_count,particle_lighting_resume_table,particle_lighting_resume_count);}
    const auto saved=_mm_getcsr();unsigned callers=0,known=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x51,bozo::kFlatBytes);
        const uint32_t aliases[]={0,0x20000000,0x30000000,0x80000000,0xa0000000},object=aliases[(seed/8)%5]|0x180000,lighting=aliases[(seed/16)%5]|0x190000;
        uint32_t stack=seed&2?0x70003e00:0x1ffe000;if(seed&32)stack=object+0x4d0;if(seed&64)stack=lighting+0x290;
        std::array<uint32_t,4> colour,light;for(unsigned i=0;i<4;++i){colour[i]=uint32_t((seed*7919+i*3121)&0x807fffff)|((95+(seed+i)%48)<<23);light[i]=uint32_t((seed*65537+i*19937)&0x807fffff)|((95+(seed+i*7)%48)<<23);}
        const bool identity=seed%8==0;if(identity)light.fill(0x3f800000);
        if(seed%8==1){colour={0x00800000,0x80800000,0,0x80000000};light.fill(0x3f000000);}
        write(x,object+0x580,colour);write(x,lighting+0x340,light);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];p.f[i]=float(i)/8;}
        p.r[20].ud[0]=object;p.r[18].ud[0]=lighting;p.r[4].ud[0]=lighting+0x340;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;
        std::memset(p.vu0_vf,0x65,sizeof(p.vu0_vf));std::memset(&p.vu0_acc,0x54,sizeof(p.vu0_acc));p.vu0_status=0x1234;auto q=p;
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&2?0x8040:0);_mm_setcsr(control);
        x.functions.generated(0x22f2f8)(x.memory.flat(),&p,&x);const auto fp=_mm_getcsr();_mm_setcsr(control);
        y.functions.generated(0x22f2f8)(y.memory.flat(),&q,&y);check(_mm_getcsr()==fp);_mm_setcsr(saved);p.cycleLimit=q.cycleLimit;
        check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        if(identity){check(!std::memcmp(colour.data(),y.memory.hostPointer(stack+0xb0,16),16));++known;}++callers;
    }
    auto &rt=b.runtime();for(unsigned which=0;which<9;++which){constexpr uint32_t object=0x180000,lighting=0x190000,stack=0x1ffe000;
        const std::array<float,4> unit{1,1,1,1};write(rt,object+0x580,unit);write(rt,lighting+0x340,unit);
        R5900Context c{};c.pc=0x22f214;c.r[20].uw[0]=object;c.r[18].uw[0]=lighting;c.r[3].uw[0]=object+0x580;c.r[4].uw[0]=lighting+0x340;c.r[2].uw[0]=stack+0xb0;c.r[29].uw[0]=stack;
        if(which<3){const uint32_t bad[]={1,0x7fa01234,0x71800000};write(rt,object+0x58c,bad[which]);}
        else if(which==3)c.pc=0x170000;else if(which==4)c.r[3].uw[0]+=16;else if(which==5)c.r[4].uw[0]+=16;else if(which==6)c.r[2].uw[0]+=16;else if(which==7)c.r[18].uw[0]=0;else c.r[3].uw[0]+=4;
        const auto before=c;const auto fp=_mm_getcsr();std::vector<uint8_t> ram(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x22f30c);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && _mm_getcsr()==fp && !std::memcmp(&c,&before,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));
    }
    _mm_setcsr(saved);check(old.sites==now.sites && now.sites.size()==4);
    std::printf("%u bounded particle colour products,%u known RGBA identities,4 sites; integer oracle, full CPU/RAM/FP, alpha/underflow/signed zero/four modes/aliases/overlap/rejection passed\n",callers,known);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

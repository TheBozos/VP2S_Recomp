#include "native_math.h"
#include "native_float_oracle.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "update_point_functions.h"
#include <bit>
#include <cstdio>
#include <cstring>
#include <set>
#include <source_location>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("update point assertion at "+std::to_string(at.line()));}
struct Host:bozo::ee::Host,bozo::port::Host {
    bool reference=false;std::set<uint32_t> sites;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("library");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        sites.insert(pc);if(!reference){check(vp2s::nativeMathInstruction(r,c,rt,pc));return true;}
        if(pc==0x227d9c || pc==0x227e0c){
            const auto stack=c->r[29].uw[0],matrix=pc==0x227d9c?stack+0x450:c->r[19].uw[0];
            uint32_t m[4][4],p[4],out[4];std::memcpy(m,rt->memory.hostPointer(matrix,64),64);std::memcpy(p,rt->memory.hostPointer(stack+0x390,16),16);
            for(unsigned lane=0;lane<4;++lane){auto v=vp2s::test::product(m[0][lane],p[0]);
                for(unsigned row=1;row<4;++row)v=vp2s::test::sum(v,vp2s::test::product(m[row][lane],p[row]));out[lane]=v;}
            std::memcpy(rt->memory.hostPointer(stack+0x440,16),out,16);
        }return true;
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
    Host old,now;old.reference=true;bozo::ee::Dispatcher a(old),b(now);a.runtime().port=&old;b.runtime().port=&now;
    for(auto *d:{&a,&b})d->setFunctions(update_point_function_table,update_point_function_count,update_point_resume_table,update_point_resume_count);
    const auto saved=_mm_getcsr();unsigned callers=0,known=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime();auto &y=b.runtime();std::memset(x.memory.flat(),0,bozo::kFlatBytes);
        const auto stack=seed&2?0x70003000u:0x1ffe000u;const bool offset=seed&1;
        const uint32_t aliases[5]={0,0x20000000,0x30000000,0x80000000,0xa0000000};
        auto matrix=offset?stack+0x450:aliases[(seed/8)%5]|0x180000u;
        if(!offset && (seed&4))matrix=stack+0x440;
        float m[4][4],point[4]={float(int(seed%17)-8),float(int(seed%11)-5),float(int(seed%7)-3),std::bit_cast<float>(0x7fc01234u)};
        const bool identity=seed%8<2;
        for(unsigned row=0;row<4;++row)for(unsigned lane=0;lane<4;++lane)m[row][lane]=identity?float(row==lane):float(int((seed+row*17+lane*11)%41)-20)/16;
        std::memcpy(x.memory.hostPointer(matrix,64),m,64);std::memcpy(x.memory.hostPointer(stack+0x390,16),point,16);
        const uint32_t delta=offset?0x190000u:0;std::memcpy(x.memory.hostPointer(stack+0x128,4),&delta,4);
        const float displacement[4]={2,-3,4,std::bit_cast<float>(0x7fc00001u)};std::memcpy(x.memory.hostPointer(0x190000,16),displacement,16);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        p.r[19].ud[0]=matrix;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;std::memset(p.vu0_vf,0x6a,sizeof(p.vu0_vf));p.vu0_status=0x9876;auto q=p;
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&2?0x8040:0);_mm_setcsr(control);
        x.functions.generated(0x227d58)(x.memory.flat(),&p,&x);const auto after=_mm_getcsr();_mm_setcsr(control);
        y.functions.generated(0x227d58)(y.memory.flat(),&q,&y);
        if(_mm_getcsr()!=after || std::memcmp(&p,&q,sizeof(p)) || std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes)){
            size_t at=0;while(at<bozo::kFlatBytes && x.memory.flat()[at]==y.memory.flat()[at])++at;
            std::fprintf(stderr,"point seed%u mode%x FP%x/%x RAM%zx\n",seed,mode,after,_mm_getcsr(),at);check(false);}
        check(p.pc==0x170000);++callers;
        if(identity){float out[4];std::memcpy(out,y.memory.hostPointer(stack+0x440,16),16);
            for(unsigned i=0;i<3;++i)check(out[i]==point[i]+(offset?displacement[i]:0));check(out[3]==1);++known;}
    }
    auto &rt=b.runtime();constexpr uint32_t stack=0x1ffe000,matrix=0x180000;float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},point[4]={1,2,3,1};
    for(unsigned which=0;which<8;++which){std::memset(rt.memory.flat(),0,bozo::kFlatBytes);std::memcpy(rt.memory.hostPointer(matrix,64),identity,64);std::memcpy(rt.memory.hostPointer(stack+0x390,16),point,16);
        R5900Context c{};c.pc=0x226568;c.r[19].ud[0]=matrix;c.r[29].ud[0]=stack;c.r[2].ud[0]=stack+0x440;c.r[3].ud[0]=stack+0x390;
        if(which<3){const uint32_t bad[3]={0x7fc00000,1,0x71800000};std::memcpy(rt.memory.hostPointer(matrix,4),bad+which,4);}
        else if(which==3)c.pc=0x216f50;else if(which==4)c.r[2].ud[0]+=16;else if(which==5)c.r[3].ud[0]+=16;
        else if(which==6){const float bad=2;std::memcpy(rt.memory.hostPointer(stack+0x39c,4),&bad,4);}else c.r[19].ud[0]+=4;
        const auto before=c;const auto bytes=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        const auto fp=_mm_getcsr();try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x227e0c);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && _mm_getcsr()==fp && !std::memcmp(&c,&before,sizeof(c)) && !std::memcmp(bytes.data(),rt.memory.flat(),bytes.size()));
    }
    _mm_setcsr(saved);check(old.sites==now.sites && now.sites.size()==20);
    std::printf("%u complete bounded point-transform branches, %u known identities, 20 sites; scalar W/XYZ offset consumers, integer oracle, CPU/RAM/FP, four modes, aliases and mutation-free guards passed\n",callers,known);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

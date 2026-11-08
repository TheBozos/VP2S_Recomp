#include "native_math.h"
#include "native_float_oracle.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "query_point_functions.h"
#include <bit>
#include <cstdio>
#include <cstring>
#include <set>
#include <source_location>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("query point assertion at "+std::to_string(at.line()));}
struct Host:bozo::ee::Host,bozo::port::Host {
    bool reference=false,pause=false;unsigned seed=0,calls=0;std::set<uint32_t> sites;std::array<uint32_t,4> observed{};
    struct Pause{};
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("library");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        sites.insert(pc);if(!reference){check(vp2s::nativeMathInstruction(r,c,rt,pc));return true;}
        if(pc==0x227f2c){const auto stack=c->r[29].uw[0];uint32_t m[4][4],p[4],out[4];
            std::memcpy(m,rt->memory.hostPointer(c->r[18].uw[0],64),64);std::memcpy(p,rt->memory.hostPointer(stack+0x440,16),16);
            for(unsigned lane=0;lane<4;++lane){auto v=vp2s::test::product(m[0][lane],p[0]);
                for(unsigned row=1;row<4;++row)v=vp2s::test::sum(v,vp2s::test::product(m[row][lane],p[row]));out[lane]=v;}
            std::memcpy(rt->memory.hostPointer(stack+0x2f0,16),out,16);
        }return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        const auto stack=c->r[29].uw[0];check(pc==0x22d5f0 && ++calls==1 && c->r[4].uw[0]==0x1b0000 &&
            c->r[5].uw[0]==stack+0x2f0 && c->r[6].uw[0]==stack+0x5e8 && c->r[7].uw[0]==stack+0x5ec);
        std::memcpy(observed.data(),d.runtime().memory.hostPointer(c->r[5].uw[0],16),16);
        const uint32_t first=0x12340000|seed,second=0x76540000|seed;
        std::memcpy(d.runtime().memory.hostPointer(c->r[6].uw[0],4),&first,4);std::memcpy(d.runtime().memory.hostPointer(c->r[7].uw[0],4),&second,4);
        c->r[2].sd[0]=seed%4?int(seed):-1;c->pc=c->r[31].uw[0];if(pause)throw Pause{};
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
void run(bozo::ee::Dispatcher &d,R5900Context &c){auto &rt=d.runtime();auto fn=rt.functions.generated(0x227f00);unsigned resumed=0;
    while(true){try{fn(rt.memory.flat(),&c,&rt);if(c.pc==0x170000)break;}catch(const Host::Pause &){}
        check(++resumed<3);const auto entry=rt.functions.resumeEntry(c.pc);check(entry);fn=entry->resume;}}
}
int main(){try{
    Host old,now;old.reference=true;bozo::ee::Dispatcher a(old),b(now);a.runtime().port=&old;b.runtime().port=&now;
    for(auto *d:{&a,&b}){d->setFunctions(query_point_function_table,query_point_function_count,query_point_resume_table,query_point_resume_count);d->bindLibrary(0x22d5f0);}
    const auto saved=_mm_getcsr();unsigned callers=0,known=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime();auto &y=b.runtime();std::memset(x.memory.flat(),0,bozo::kFlatBytes);old.calls=now.calls=0;old.seed=now.seed=seed;old.pause=now.pause=seed&16;
        const auto stack=seed&2?0x70003000u:0x1ffe000u;const uint32_t aliases[5]={0,0x20000000,0x30000000,0x80000000,0xa0000000};
        auto matrix=aliases[(seed/8)%5]|0x180000u;if(seed&4)matrix=stack+0x440;else if(seed&8)matrix=stack+0x2f0;
        float m[4][4],point[4]={float(int(seed%17)-8),float(int(seed%11)-5),float(int(seed%7)-3),float(int(seed%5)-2)};
        const bool identity=seed%8==0,translated=seed%8==1;
        for(unsigned row=0;row<4;++row)for(unsigned lane=0;lane<4;++lane)m[row][lane]=(identity || translated)?float(row==lane):float(int((seed+row*17+lane*11)%41)-20)/16;
        if(translated){m[3][0]=2;m[3][1]=-3;m[3][2]=4;}
        std::memcpy(x.memory.hostPointer(matrix,64),m,64);std::memcpy(x.memory.hostPointer(stack+0x440,16),point,16);
        const uint32_t child=0x1b0000;const uint64_t outer=0x170000;std::memcpy(x.memory.hostPointer(0x1a03e4,4),&child,4);std::memcpy(x.memory.hostPointer(stack+0x600,8),&outer,8);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        p.r[18].ud[0]=matrix;p.r[20].ud[0]=0x1a0000;p.r[29].ud[0]=stack;p.r[31].ud[0]=outer;std::memset(p.vu0_vf,0x6a,sizeof(p.vu0_vf));p.vu0_status=0x9876;auto q=p;
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&2?0x8040:0);_mm_setcsr(control);run(a,p);const auto after=_mm_getcsr();_mm_setcsr(control);run(b,q);
        if(_mm_getcsr()!=after || std::memcmp(&p,&q,sizeof(p)) || std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"query point seed%u mode%x FP%x/%x\n",seed,mode,after,_mm_getcsr());check(false);}
        check(old.calls==1 && now.calls==1 && old.observed==now.observed && q.r[21].sd[0]==(seed%4?int(seed):-1));++callers;
        if(identity || translated){float observed[4];std::memcpy(observed,now.observed.data(),16);
            for(unsigned i=0;i<3;++i){const float shift=translated?(i==0?2.f:i==1?-3.f:4.f):0;check(observed[i]==point[i]+shift*point[3]);}
            check(observed[3]==point[3]);++known;}
    }
    auto &rt=b.runtime();constexpr uint32_t stack=0x1ffe000,matrix=0x180000;
    const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},point[4]={1,2,3,2};
    for(unsigned which=0;which<8;++which){std::memset(rt.memory.flat(),0,bozo::kFlatBytes);std::memcpy(rt.memory.hostPointer(matrix,64),identity,64);std::memcpy(rt.memory.hostPointer(stack+0x440,16),point,16);
        R5900Context c{};c.pc=0x226568;c.r[18].ud[0]=matrix;c.r[29].ud[0]=stack;c.r[2].ud[0]=stack+0x2f0;c.r[3].ud[0]=stack+0x440;
        if(which<2 || which==7){const uint32_t bad=which==0?0x7fc00000u:which==1?1u:0x71800000u;std::memcpy(rt.memory.hostPointer(stack+0x44c,4),&bad,4);}
        else if(which==2)c.r[18].ud[0]+=4;else if(which==3)c.r[2].ud[0]+=16;else if(which==4)c.pc=0x216f50;
        else if(which==5)c.r[3].ud[0]+=16;else c.r[18].ud[0]=0x1fffff0;
        const auto before=c;const auto bytes=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        const auto fp=_mm_getcsr();try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x227f2c);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && _mm_getcsr()==fp && !std::memcmp(&c,&before,sizeof(c)) && !std::memcmp(bytes.data(),rt.memory.flat(),bytes.size()));
    }
    _mm_setcsr(saved);check(old.sites==now.sites && now.sites.size()==10);
    std::printf("%u bounded query-point callers, %u known homogeneous transforms, 10 sites; full W, scalar query arguments/results, CPU/RAM/FP, integer oracle, aliases and paused resumes passed\n",callers,known);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

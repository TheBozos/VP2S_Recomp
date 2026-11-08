#include "native_math.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "dof_matrix_original_functions.h"
#include "dof_matrix_adapted_functions.h"
#include <bit>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <source_location>
#include <span>
#include <immintrin.h>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("DOF matrix assertion at "+std::to_string(at.line()));}
template<class T>void put(Runtime &rt,uint32_t at,T v){std::memcpy(rt.memory.hostPointer(at,sizeof(T)),&v,sizeof(T));}
uint64_t rightJam(uint64_t v,unsigned shift){if(!shift)return v;if(shift>=64)return v!=0;return (v>>shift)|((v&((uint64_t(1)<<shift)-1))!=0);}
uint32_t rounded(uint32_t sign,int e,uint64_t m){
    if(!m)return sign;while(m>=(uint64_t(1)<<56)){m=rightJam(m,1);++e;}while(m<(uint64_t(1)<<55)){m<<=1;--e;}
    if(e<=0){m=rightJam(m,unsigned(1-e));e=1;}
    uint32_t value=uint32_t(m>>32),fraction=uint32_t(m);const auto mode=_mm_getcsr()&_MM_ROUND_MASK;
    const bool increment=mode==_MM_ROUND_NEAREST?(fraction>0x80000000u || (fraction==0x80000000u && (value&1))):mode==_MM_ROUND_DOWN?(sign && fraction):mode==_MM_ROUND_UP?(!sign && fraction):false;
    if(increment && ++value==0x1000000){value>>=1;++e;}check(e<255);if(value<0x800000)return sign;return sign|uint32_t(e<<23)|(value&0x7fffff);
}
uint32_t product(uint32_t a,uint32_t b){const auto sign=(a^b)&0x80000000u;const int ea=(a>>23)&255,eb=(b>>23)&255;
    if(!ea || !eb)return sign;return rounded(sign,ea+eb-127,uint64_t((a&0x7fffff)|0x800000)*((b&0x7fffff)|0x800000)<<9);}
uint32_t sum(uint32_t a,uint32_t b){
    if((a&0x7fffffff)<(b&0x7fffffff))std::swap(a,b);
    if(!(b&0x7fffffff)){if(a&0x7fffffff)return a;return ((a&b)&0x80000000u)|((_mm_getcsr()&_MM_ROUND_MASK)==_MM_ROUND_DOWN?((a|b)&0x80000000u):0u);}
    int e=(a>>23)&255;const auto distance=unsigned(e)-((b>>23)&255);
    const uint64_t large=uint64_t((a&0x7fffff)|0x800000)<<32,small=rightJam(uint64_t((b&0x7fffff)|0x800000)<<32,distance);
    const auto magnitude=((a^b)&0x80000000u)?large-small:large+small;
    if(!magnitude)return (_mm_getcsr()&_MM_ROUND_MASK)==_MM_ROUND_DOWN?0x80000000u:0u;return rounded(a&0x80000000u,e,magnitude);
}
void reference(Runtime *rt,R5900Context *c){
    uint32_t local[4][4],parent[4][4],out[4][4];std::memcpy(local,rt->memory.hostPointer(c->r[4].uw[0],64),64);std::memcpy(parent,rt->memory.hostPointer(c->r[3].uw[0],64),64);
    for(unsigned column=0;column<4;++column)for(unsigned lane=0;lane<4;++lane){auto v=product(parent[0][lane],local[column][0]);
        for(unsigned i=1;i<4;++i)v=sum(v,product(parent[i][lane],local[column][i]));out[column][lane]=v;}
    std::memcpy(rt->memory.hostPointer(c->r[5].uw[0],64),out,64);
}
struct Host:bozo::ee::Host,bozo::port::Host {
    bool old=false;unsigned publications=0;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("library");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(pc==0x2d7c64 || pc==0x2d7ce0 || pc==0x2d7d5c)++publications;
        if(old){reference(rt,c);return true;}check(vp2s::nativeMathInstruction(r,c,rt,pc));return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *c,uint32_t pc)override{
        if(pc==0x12dbc0)c->f[0]=std::cos(c->f[12]);else if(pc==0x12df60)c->f[0]=std::sin(c->f[12]);else throw std::runtime_error("call");c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host old,now;old.old=true;bozo::ee::Dispatcher a(old),b(now);
    a.setFunctions(dof_matrix_original_function_table,dof_matrix_original_function_count,dof_matrix_original_resume_table,dof_matrix_original_resume_count);
    b.setFunctions(dof_matrix_adapted_function_table,dof_matrix_adapted_function_count,dof_matrix_adapted_resume_table,dof_matrix_adapted_resume_count);
    a.runtime().port=&old;b.runtime().port=&now;
    for(auto *d:{&a,&b})for(uint32_t pc:{0x12dbc0u,0x12df60u})d->bindLibrary(pc);
    const auto saved=_mm_getcsr();unsigned callers=0,records=0;uint32_t random=0x2d7a30;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime();auto &y=b.runtime();std::memset(x.memory.flat(),0x65,bozo::kFlatBytes);
        const uint32_t object=0x180000,cursor=0x1a0000,buffer=0x200000,top=seed&1?0x70003f00:0x1fff000;
        put(x,cursor,buffer);put(x,object+0x131ac,uint32_t(128));put(x,object+0x131b0,uint32_t(seed&2?89:128));
        put(x,object+0x131a4,uint32_t(512));put(x,object+0x131a8,uint32_t(512));put(x,object+0xa8,uint32_t(512));put(x,object+0xac,uint32_t(512));
        put(x,object+0x98,uint32_t(1728));put(x,object+0x9c,uint32_t(1824));std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];p.f[i]=float(i)/8;}
        std::memset(p.vu0_vf,0x6b,sizeof(p.vu0_vf));p.vu0_status=0x7654;p.r[4].ud[0]=object;p.r[5].ud[0]=cursor;p.r[6].ud[0]=16+seed%3*16;p.r[7].ud[0]=8+seed%4*8;
        p.r[29].ud[0]=top;p.r[31].ud[0]=0x170000;p.f[12]=float(int(seed%17)-8)/8;p.f[13]=0.5f+float(seed%5)/4;auto q=p;
        old.publications=now.publications=0;const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);_mm_setcsr(control);
        x.functions.generated(0x2d7a30)(x.memory.flat(),&p,&x);const auto after=_mm_getcsr();_mm_setcsr(control);
        try{y.functions.generated(0x2d7a30)(y.memory.flat(),&q,&y);}catch(const std::exception &e){std::fprintf(stderr,"DOF writer seed=%u mode=%x: %s\n",seed,mode,e.what());throw;}
        if(_mm_getcsr()!=after || std::memcmp(&p,&q,sizeof(p)) || std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes)){
            unsigned cpu=0,ram=0;while(cpu<sizeof(p) && reinterpret_cast<uint8_t *>(&p)[cpu]==reinterpret_cast<uint8_t *>(&q)[cpu])++cpu;
            while(ram<bozo::kFlatBytes && x.memory.flat()[ram]==y.memory.flat()[ram])++ram;
            std::fprintf(stderr,"DOF writer mismatch seed=%u mode=%x FP=%x/%x CPU=%x RAM=%x\n",seed,mode,after,_mm_getcsr(),cpu,ram);check(false);}
        check(old.publications==3 && now.publications==3 && p.pc==0x170000);uint32_t end;std::memcpy(&end,y.memory.ram()+cursor,4);check(end>buffer && end<buffer+0x40000);++callers;
    }
    // Direct complete XYZW records qualify non-affine matrices and all three
    // in-place publications beyond the sparse matrices in the real builder.
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed)for(unsigned stage=0;stage<3;++stage){
        auto &x=a.runtime();auto &y=b.runtime();std::memset(x.memory.flat(),0x65,bozo::kFlatBytes);const uint32_t stack=seed&1?0x70003000:0x1ffe000;
        const uint32_t local=stack+(stage==0?0x140:stage==1?0x180:0xc0),site=stage==0?0x2d7c64:stage==1?0x2d7ce0:0x2d7d5c;
        uint32_t first[16],parent[16];for(auto *matrix:{first,parent})for(auto &v:std::span(matrix,16)){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((104+random%25)<<23);}
        if(seed%16==0){std::memset(first,0,64);std::memset(parent,0,64);for(unsigned i=0;i<4;++i)first[i*5]=parent[i*5]=0x3f800000;}
        std::memcpy(x.memory.hostPointer(local,64),first,64);std::memcpy(x.memory.hostPointer(stack+0x100,64),parent,64);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};p.pc=0x2d7be0;p.r[29].ud[0]=stack;p.r[4].ud[0]=local;p.r[3].ud[0]=p.r[5].ud[0]=stack+0x100;auto q=p;
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);_mm_setcsr(control);reference(&x,&p);
        check(vp2s::nativeMathInstruction(y.memory.flat(),&q,&y,site));check(_mm_getcsr()==control && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));++records;
    }
    _mm_setcsr(saved);
    {auto &rt=b.runtime();R5900Context c{};c.pc=0x2d7be0;c.r[29].uw[0]=0x1ffe000;c.r[4].uw[0]=0x1ffe140;c.r[3].uw[0]=c.r[5].uw[0]=0x1ffe100;
        uint32_t matrix[16]={};for(unsigned i=0;i<4;++i)matrix[i*5]=0x3f800000;std::memcpy(rt.memory.hostPointer(0x1ffe140,64),matrix,64);std::memcpy(rt.memory.hostPointer(0x1ffe100,64),matrix,64);
        auto reject=[&]{const auto before=c;std::vector<uint8_t> ram(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2d7c64);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));};
        c.pc=0x170000;reject();c.pc=0x2d7be0;c.r[4].uw[0]+=16;reject();c.r[4].uw[0]-=16;
        for(uint32_t invalid:{1u,0x7fc01234u,0x7f7fffffu}){std::memcpy(rt.memory.hostPointer(0x1ffe140,4),&invalid,4);reject();}
    }
    std::printf("%u complete DOF coordinate/writer callers + %u full matrix records, 84 sites; CPU/RAM/FP, four modes/FTZ/DAZ, in-place compositions and original scalar consumers passed\n",callers,records);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

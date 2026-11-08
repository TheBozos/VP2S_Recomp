#include "native_math.h"
#include "native_float_oracle.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "update_linear_functions.h"
#include <bit>
#include <cstdio>
#include <cstring>
#include <set>
#include <source_location>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("update linear assertion at "+std::to_string(at.line()));}
struct Host:bozo::ee::Host,bozo::port::Host {
    bool reference=false;std::set<uint32_t> sites;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("library");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        sites.insert(pc);if(!reference){check(vp2s::nativeMathInstruction(r,c,rt,pc));return true;}
        if(pc==0x229180 || pc==0x229248 || pc==0x229314 || pc==0x2295f0 || pc==0x22961c || pc==0x229644){
            const bool first=pc==0x229180 || pc==0x2295f0,second=pc==0x229248 || pc==0x22961c,change=pc>=0x2295d4;
            const auto stack=c->r[29].uw[0],temporary=change?0x420u:0x430u,source=change?0x210u:0x2d0u;
            const auto matrix=c->r[first?19:18].uw[0],vector=stack+(second?temporary:source),destination=first?stack+temporary:c->r[16].uw[0]+(change?0x40:0x30);
            uint32_t m[3][4],p[4],out[4];std::memcpy(m,rt->memory.hostPointer(matrix,48),48);std::memcpy(p,rt->memory.hostPointer(vector,16),16);
            for(unsigned lane=0;lane<4;++lane){auto value=vp2s::test::product(m[0][lane],p[0]);
                for(unsigned row=1;row<3;++row)value=vp2s::test::sum(value,vp2s::test::product(m[row][lane],p[row]));out[lane]=value;}
            std::memcpy(rt->memory.hostPointer(destination,16),out,16);
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
    for(auto *d:{&a,&b})d->setFunctions(update_linear_function_table,update_linear_function_count,update_linear_resume_table,update_linear_resume_count);
    const auto saved=_mm_getcsr();unsigned callers=0,known=0;
    for(bool change:{false,true})for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime();auto &y=b.runtime();std::memset(x.memory.flat(),0,bozo::kFlatBytes);
        const auto stack=seed&8?0x70003000u:0x1ffe000u;constexpr uint32_t object=0x1a0000,particle=0x1b0000;
        const uint32_t aliases[5]={0,0x20000000,0x30000000,0x80000000,0xa0000000},alias=aliases[(seed/8)%5];
        const auto temporary=change?0x420u:0x430u,source=change?0x210u:0x2d0u,field=change?0x40u:0x30u;
        const auto matrixA=seed&16?stack+temporary:alias|0x180000u,matrixB=seed&32?particle+field:alias|0x190000u;
        const uint32_t opaque[4]={0x7fc01234,0x7f800001,0x7f800000,1};
        float point[4]={float(int(seed%17)-8),float(int(seed%11)-5),float(int(seed%7)-3),std::bit_cast<float>(opaque[seed%4])};
        const float initial[4]={10,20,30,std::bit_cast<float>(opaque[(seed+1)%4])};
        std::memcpy(x.memory.hostPointer(stack+source,16),point,16);std::memcpy(x.memory.hostPointer(stack+temporary,16),initial,16);
        const bool identity=(seed/8)%4==0;float first[3][4],second[3][4];
        for(unsigned row=0;row<3;++row)for(unsigned lane=0;lane<4;++lane){
            first[row][lane]=identity?float(row==lane):float(int((seed+row*17+lane*11)%41)-20)/16;
            second[row][lane]=identity?float(row==lane):float(int((seed+row*13+lane*7)%37)-18)/16;}
        std::memcpy(x.memory.hostPointer(matrixA,48),first,48);std::memcpy(x.memory.hostPointer(matrixB,48),second,48);
        // Fourth basis records and input W are deliberately not numerical inputs.
        for(auto at:{matrixA+48,matrixB+48})std::memcpy(x.memory.hostPointer(at,16),opaque,16);
        const uint8_t flags=change?uint8_t(seed&1?4:0):uint8_t((seed&1?2:0)|(seed&2?0x10:0)),extra=seed&4?1:0;
        std::memcpy(x.memory.hostPointer(object+0x4ab,1),&flags,1);std::memcpy(x.memory.hostPointer(object+0x4ad,1),&extra,1);
        const float acceleration[3]={2,-4,6},offset[3]={3,5,-7},time=0.5f;
        std::memcpy(x.memory.hostPointer(object+0x450,12),acceleration,12);std::memcpy(x.memory.hostPointer(object+0x480,12),offset,12);std::memcpy(x.memory.hostPointer(object+0x3fc,4),&time,4);
        const uint32_t displacement=extra?0x1c0000u:0;std::memcpy(x.memory.hostPointer(stack+0x120,4),&displacement,4);std::memcpy(x.memory.hostPointer(0x1c0000,12),offset,12);
        const uint32_t defaults[4]={0x7fc00001,0x7f800001,0xff800000,1};std::memcpy(x.memory.hostPointer(0x35da10,16),defaults,16);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        p.r[16].ud[0]=particle;p.r[18].ud[0]=matrixB;p.r[19].ud[0]=matrixA;p.r[20].ud[0]=object;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;
        std::memset(p.vu0_vf,0x6a,sizeof(p.vu0_vf));p.vu0_status=0x9876;auto q=p;
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&8?0x8040:0);_mm_setcsr(control);
        const auto entry=change?0x229578u:0x229148u;
        x.functions.generated(entry)(x.memory.flat(),&p,&x);const auto after=_mm_getcsr();_mm_setcsr(control);
        y.functions.generated(entry)(y.memory.flat(),&q,&y);
        if(_mm_getcsr()!=after || std::memcmp(&p,&q,sizeof(p)) || std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes)){
            size_t at=0;while(at<bozo::kFlatBytes && x.memory.flat()[at]==y.memory.flat()[at])++at;
            std::fprintf(stderr,"linear change%u seed%u mode%x FP%x/%x RAM%zx\n",change,seed,mode,after,_mm_getcsr(),at);check(false);}
        check(p.pc==0x170000);++callers;
        if(change)check(!std::memcmp(y.memory.hostPointer(particle+0x50,16),defaults,16));
        if(identity){float out[4];std::memcpy(out,y.memory.hostPointer(particle+field,16),16);
            for(unsigned lane=0;lane<3;++lane){float value=point[lane];
                if(change){if(extra)value+=offset[lane];}
                else{if(flags&0x10)value+=acceleration[lane]*time;if(!(flags&2) && extra)value+=offset[lane];}check(out[lane]==value);}
            check(out[3]==0);++known;}
    }
    auto &rt=b.runtime();constexpr uint32_t stack=0x1ffe000,matrix=0x180000;
    const float basis[12]={1,0,0,0,0,1,0,0,0,0,1,0},point[4]={1,2,3,std::bit_cast<float>(0x7f800001u)};
    for(bool change:{false,true})for(unsigned which=0;which<8;++which){const auto source=change?0x210u:0x2d0u,temporary=change?0x420u:0x430u;
        std::memset(rt.memory.flat(),0,bozo::kFlatBytes);std::memcpy(rt.memory.hostPointer(matrix,48),basis,48);std::memcpy(rt.memory.hostPointer(stack+source,16),point,16);
        R5900Context c{};c.pc=0x2284ec;c.r[19].ud[0]=matrix;c.r[29].ud[0]=stack;c.r[2].ud[0]=stack+temporary;c.r[3].ud[0]=stack+source;
        if(which<3){const uint32_t bad[3]={0x7fc00000,1,0x71800000};std::memcpy(rt.memory.hostPointer(matrix,4),bad+which,4);}
        else if(which==3 || which==4){const uint32_t bad=which==3?0x7fc00000u:1u;std::memcpy(rt.memory.hostPointer(stack+source,4),&bad,4);}
        else if(which==5)c.pc=0x216f50;else if(which==6)c.r[2].ud[0]+=16;else c.r[3].ud[0]+=16;
        const auto before=c;const auto bytes=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        const auto fp=_mm_getcsr();try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,change?0x2295f0:0x229180);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && _mm_getcsr()==fp && !std::memcmp(&c,&before,sizeof(c)) && !std::memcmp(bytes.data(),rt.memory.flat(),bytes.size()));
    }
    _mm_setcsr(saved);check(old.sites==now.sites && now.sites.size()==48);
    std::printf("%u bounded velocity/change callers, %u known identities, 48 sites; opaque input W/full output W, scalar offsets/default record consumers, integer oracle, CPU/RAM/FP, aliases and rejection guards passed\n",callers,known);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

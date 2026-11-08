#include "native_math.h"
#include "native_float_oracle.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "particle_callback_functions.h"
#include <bit>
#include <cstdio>
#include <cstring>
#include <set>
#include <source_location>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("particle callback assertion at "+std::to_string(at.line()));}
template<class T>void write(Runtime &rt,uint32_t at,const T &v){auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(p,&v,sizeof(v));}
struct Host:bozo::ee::Host,bozo::port::Host {
    bool reference=false;std::set<uint32_t> sites;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("library");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        sites.insert(pc);if(!reference){check(vp2s::nativeMathInstruction(r,c,rt,pc));return true;}
        if(pc==0x410684 || pc==0x4106cc || pc==0x410714){
            const bool point=pc==0x410684;uint32_t basis[4][4],input[4],out[4];
            std::memcpy(basis,rt->memory.hostPointer(c->r[4].uw[0]+0x80,64),64);
            std::memcpy(input,rt->memory.hostPointer(c->r[point?5:pc==0x4106cc?6:7].uw[0],16),16);
            for(unsigned lane=0;lane<4;++lane){auto v=vp2s::test::product(basis[0][lane],input[0]);
                for(unsigned row=1;row<(point?4u:3u);++row)v=vp2s::test::sum(v,vp2s::test::product(basis[row][lane],row==3?0x3f800000:input[row]));out[lane]=v;}
            write(*rt,c->r[2].uw[0],out);
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
    Host old,now;old.reference=true;bozo::ee::Dispatcher a(old),b(now);
    for(auto *d:{&a,&b}){d->runtime().port=d==&a?&old:&now;d->setFunctions(particle_callback_function_table,particle_callback_function_count,particle_callback_resume_table,particle_callback_resume_count);}
    const auto saved=_mm_getcsr();unsigned callers=0,known=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x55,bozo::kFlatBytes);
        const uint32_t aliases[]={0,0x20000000,0x30000000,0x80000000,0xa0000000};
        const uint32_t emitter=aliases[(seed/8)%5]|0x180000,outer=seed&16?0x70003f00:0x1fff000,stack=outer-0x80;
        const unsigned flags=seed&7;write(x,emitter+0xd2,uint8_t(flags));
        float basis[4][4],inputs[3][4];const bool identity=seed%16<8;
        for(unsigned row=0;row<4;++row)for(unsigned lane=0;lane<4;++lane)basis[row][lane]=identity?float(row==lane):float(int((seed+row*17+lane*11)%41)-20)/16;
        for(unsigned j=0;j<3;++j){for(unsigned lane=0;lane<3;++lane)inputs[j][lane]=float(int((seed+j*7+lane*3)%29)-14)/8;
            const uint32_t opaque[]={0x7fc01234,0x7fa01234,0x7f800000,1};inputs[j][3]=std::bit_cast<float>(opaque[(seed/8)%4]);}
        write(x,emitter+0x80,basis);
        uint32_t addresses[3]={0x190000,0x190010,0x190020};if(seed&32)addresses[1]=addresses[0];if(seed&64)addresses[2]=emitter+0x80;
        for(unsigned j=0;j<3;++j)if(addresses[j]!=emitter+0x80)write(x,addresses[j],inputs[j]);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        p.r[4].ud[0]=emitter;p.r[5].ud[0]=addresses[0];p.r[6].ud[0]=addresses[1];p.r[7].ud[0]=addresses[2];p.r[29].ud[0]=outer;p.r[31].ud[0]=0x170000;
        std::memset(p.vu0_vf,0x6a,sizeof(p.vu0_vf));std::memset(&p.vu0_acc,0x65,sizeof(p.vu0_acc));p.vu0_status=0x9876;auto q=p;
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&8?0x8040:0);_mm_setcsr(control);
        x.functions.generated(0x410640)(x.memory.flat(),&p,&x);const auto fp=_mm_getcsr();_mm_setcsr(control);
        y.functions.generated(0x410640)(y.memory.flat(),&q,&y);check(_mm_getcsr()==fp);_mm_setcsr(saved);p.cycleLimit=q.cycleLimit;
        check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        if(identity && !(seed&64)){for(unsigned j=0;j<3;++j)if(flags&(1<<j)){float out[4],input[4];std::memcpy(out,y.memory.hostPointer(stack+0x70-j*16,16),16);std::memcpy(input,x.memory.hostPointer(addresses[j],16),16);
                for(unsigned lane=0;lane<3;++lane)check(out[lane]==input[lane]);check(out[3]==(j==0?1.f:0.f));++known;}}
        check(q.r[10].uw[0]==(flags&1?stack+0x70:0) && q.r[5].uw[0]==(flags&2?stack+0x60:0) && q.r[3].uw[0]==(flags&4?stack+0x50:0));++callers;
    }
    _mm_setcsr(saved);check(old.sites==now.sites && now.sites.size()==26);
    auto &rt=b.runtime();for(unsigned which=0;which<8;++which){
        std::memset(rt.memory.flat(),0,bozo::kFlatBytes);const float basis[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},input[4]={1,2,3,0};write(rt,0x180080,basis);write(rt,0x190000,input);write(rt,0x1800d2,uint8_t(1));
        R5900Context c{};c.pc=0x410640;c.r[4].uw[0]=0x180000;c.r[5].uw[0]=0x190000;c.r[29].uw[0]=0x1fff000;c.r[2].uw[0]=0x1fff070;c.r[3].uw[0]=0x180080;
        if(which<3){const uint32_t bad[]={0x7fa01234,1,0x71800000};write(rt,0x180080,bad[which]);}
        else if(which==3)c.pc=0x410790;else if(which==4)c.r[2].uw[0]+=16;else if(which==5)c.r[3].uw[0]+=16;else if(which==6)write(rt,0x1800d2,uint8_t(0));else c.r[5].uw[0]+=4;
        const auto before=c;const auto fp=_mm_getcsr();std::vector<uint8_t> ram(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x410684);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && _mm_getcsr()==fp && !std::memcmp(&c,&before,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));
    }
    std::printf("%u bounded callback argument callers,%u known identity outputs,26 sites; original pointer selection, integer oracle, full CPU/RAM/FP, opaque input W, four modes and rejection guards passed\n",callers,known);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

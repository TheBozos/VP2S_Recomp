#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "precache_functions.h"
#include <cstdio>
#include <cstring>
#include <source_location>
#include <immintrin.h>
namespace {
constexpr uint32_t object=0x180000,root=0x190000,world=0x200000,manager=0x210000,ui=0x220000,
    scene=0x230000,models=0x240000,header=0x241000,rows=0x250000,animation=0x280000;
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("precache assertion at "+std::to_string(at.line()));}
template<class T>void put(Runtime &rt,uint32_t at,T v){std::memcpy(rt.memory.hostPointer(at,sizeof(v)),&v,sizeof(v));}
uint32_t actor(unsigned i){return 0x260000+i*0x500;}uint32_t child(unsigned i){return 0x270000+i*0x100;}
struct Host:bozo::ee::Host,bozo::port::Host{
    using Event=std::pair<uint32_t,uint32_t>;std::vector<Event> events;bool pause=false;unsigned samples=0;int last=-1;
    struct Pause{};
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t)override{throw std::runtime_error("unexpected native operation");}
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        const auto arg=c->r[4].uw[0];
        if(pc==0x2a8440){check(arg==animation && c->r[5].uw[0]==2 && c->f[12]>=0 && c->f[12]<=127);
            last=int(c->f[12]);++samples;put(d.runtime(),animation+0x10,c->f[12]);events.emplace_back(pc,uint32_t(last));}
        else {events.emplace_back(pc,arg);
            if(pc==0x21e7b0)check(arg==scene && last%4==0);
            else if(pc==0x22cfd0)check(last>=60 && c->r[5].uw[0]==2);
            else if(pc==0x170130)put(d.runtime(),object+0x24,uint32_t(0x12345678));
            else if(pc==0x3edda0)check(arg==manager && c->f[12]==30);
            else if(pc!=0x170100 && pc!=0x170110 && pc!=0x170120 && pc!=0x2bedb0)throw std::runtime_error("unqualified callback");
        }
        c->pc=c->r[31].uw[0];if(pause && events.size()%17==0)throw Pause{};
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("hardware read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("hardware write");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host host;bozo::ee::Dispatcher d(host);d.runtime().port=&host;
    d.setFunctions(precache_function_table,precache_function_count,precache_resume_table,precache_resume_count);
    for(uint32_t pc:{0x2a8440u,0x21e7b0u,0x22cfd0u,0x3edda0u,0x2bedb0u,0x170100u,0x170110u,0x170120u,0x170130u})d.bindLibrary(pc);
    const auto saved=_mm_getcsr();unsigned callers=0,samples=0,callbacks=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<64;++seed){
        auto &rt=d.runtime();std::memset(rt.memory.flat(),0,bozo::kFlatBytes);host.events.clear();host.samples=0;host.last=-1;host.pause=seed&8;
        const unsigned kind=seed%8,count=(seed/8)%7,startValues[6]={0,1,59,60,127,128},start=startValues[(seed/8)%6];
        const auto flags=uint8_t((seed&16?2:0)|(seed&32?1:0)|(kind==5?4:0));const float timer=kind==0?2.f:0.f;
        const auto top=seed&1?0x70003f00u:0x1fff000u;
        put(rt,0x48e048,root);put(rt,0x35ddf8,scene);put(rt,root+8,kind==1?0u:world);put(rt,root+0x20,ui);put(rt,root+0x28,manager);put(rt,root+0xd8,uint32_t(seed%14));
        put(rt,world+0xc4,kind==2?0u:world+0x200);put(rt,world+0x304,kind==3?0u:models);put(rt,world+0x374,world+0x600);put(rt,world+0x68c,kind==4?0u:animation);
        put(rt,models+0x20,header);put(rt,models+0x18,rows);put(rt,header+4,count);put(rt,ui+0x1c,ui+0x14);
        put(rt,object,uint32_t(0x290000));put(rt,0x290010,uint32_t(0x170130));put(rt,0x291014,uint32_t(0x170110));
        put(rt,0x292014,uint32_t(0x170120));put(rt,0x292064,uint32_t(0x170100));
        put(rt,object+0x14,timer);put(rt,object+0x18,start);put(rt,object+0x1c,flags);put(rt,manager+0x538,uint8_t(0xaf));put(rt,world+0xd5,uint8_t(0x5a));
        std::vector<bool> eligible(count),hasChild(count);
        for(unsigned i=0;i<count;++i){const auto variant=(seed+i)%4;eligible[i]=variant>=2;hasChild[i]=variant==3;
            put(rt,rows+i*16,uint8_t(variant==0));put(rt,rows+i*16+8,actor(i));put(rt,rows+i*16+12,0x293000+i*32);
            put(rt,0x29300f+i*32,uint8_t(variant==1?11:10));put(rt,actor(i),uint32_t(0x291000));put(rt,actor(i)+0x3e4,hasChild[i]?child(i):0u);
            put(rt,child(i),uint32_t(0x292000));put(rt,child(i)+0x6a,uint16_t(seed&8?0x10:0));}
        R5900Context c{};for(unsigned i=1;i<32;++i){c.r[i].ud[0]=seed*32+i;c.r[i].ud[1]=~c.r[i].ud[0];}
        c.r[4].ud[0]=object;c.r[29].ud[0]=top;c.r[31].ud[0]=0x160000;std::memset(c.vu0_vf,0x6a,sizeof(c.vu0_vf));const auto before=c;
        auto expected=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
        const auto expect=[&]<class T>(uint32_t at,T v){std::memcpy(expected.data()+at,&v,sizeof(v));};
        // ABI spills are original game code, including scratch-stack aliases.
        const auto stackOffset=rt.memory.hostPointer(top-0x90,0x90)-rt.memory.flat();
        for(unsigned i=16;i<=23;++i)std::memcpy(expected.data()+stackOffset+(i-16)*16,&before.r[i],16);
        std::memcpy(expected.data()+stackOffset+0x80,&before.r[31].ud[0],8);
        expect(manager+0x14,1.f);expect(object+0x14,timer-1);
        std::vector<Host::Event> events;const bool active=kind>=5;
        if(kind==1 || kind==2)expect(object+0x1c,uint8_t(flags|4));
        if(active && kind!=5){
            for(unsigned i=0;i<count;++i)if(eligible[i] && hasChild[i])events.emplace_back(0x170100,child(i));
            for(unsigned sample=start;sample<128;++sample){events.emplace_back(0x2a8440,sample);if(sample%4==0)events.emplace_back(0x21e7b0,scene);
                for(unsigned i=0;i<count;++i)if(eligible[i]){
                    if(hasChild[i] && (seed&8) && sample>=60)events.emplace_back(0x22cfd0,child(i));
                    else{events.emplace_back(0x170110,actor(i));if(hasChild[i])events.emplace_back(0x170120,child(i));}}}
            expect(object+0x1c,uint8_t(flags|4));if(start<128)expect(animation+0x10,127.f);
        }
        if(active){if(flags&2)events.emplace_back(0x3edda0,manager);events.emplace_back(0x2bedb0,object);events.emplace_back(0x170130,object);
            expect(manager+0x538,uint8_t((0xaf&~1)|(flags&1)));expect(world+0xd5,uint8_t(0x7a));expect(root+0xd8,uint32_t(std::max(10u,seed%14)));expect(object+0x24,uint32_t(0x12345678));}
        _mm_setcsr((saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0));
        auto fn=rt.functions.generated(0x420ca0);unsigned resumed=0;
        while(true){try{fn(rt.memory.flat(),&c,&rt);if(c.pc==0x160000)break;}catch(const Host::Pause &){}
            check(++resumed<1000);const auto entry=rt.functions.resumeEntry(c.pc);check(entry);fn=entry->resume;}
        if(host.events!=events || std::memcmp(rt.memory.flat(),expected.data(),expected.size())){
            size_t at=0;while(at<expected.size() && expected[at]==rt.memory.flat()[at])++at;
            std::fprintf(stderr,"precache seed%u mode%x events%zu/%zu RAM%zx\n",seed,mode,host.events.size(),events.size(),at);check(false);}
        for(unsigned i=16;i<=23;++i)check(!std::memcmp(&c.r[i],&before.r[i],16));check(c.r[29].uw[0]==top);
        check(!std::memcmp(c.vu0_vf,before.vu0_vf,sizeof(c.vu0_vf)));++callers;samples+=host.samples;callbacks+=host.events.size();
    }
    _mm_setcsr(saved);std::printf("%u complete bounded precache owners, %u animation samples/%u ordered callbacks; whole RAM, ABI, four modes, pauses, zero device reads passed\n",callers,samples,callbacks);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

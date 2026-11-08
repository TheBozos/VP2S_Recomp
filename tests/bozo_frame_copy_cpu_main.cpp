#include "native_frame_copy.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "copy_original_functions.h"
#include "copy_adapted_functions.h"
#include "copy_helpers_original_functions.h"
#include "copy_helpers_adapted_functions.h"
#include <cstdio>
#include <algorithm>
#include <cstring>
#include <source_location>
#include <immintrin.h>
namespace {
constexpr uint32_t object=0x180000,target=0x190000,current=0x1a0000,queue=0x1b0000,post=0x200000,manager=0x230000,writer=0x240000,buffer=0x250000;
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("frame-copy CPU assertion at "+std::to_string(at.line()));}
template<class T>void put(Runtime &rt,uint32_t at,T v){std::memcpy(rt.memory.hostPointer(at,sizeof(T)),&v,sizeof(T));}
template<class T>T get(Runtime &rt,uint32_t at){T v;std::memcpy(&v,rt.memory.hostPointer(at,sizeof(T)),sizeof(T));return v;}
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::SceneOwner,vp2s::FrameCopyOwner {
    bool reference=false,fail=false,pause=false;unsigned allocations=0,draws=0,callbacks=0;
    vp2s::NativeScene scene;vp2s::NativeFrameCopies copies;std::optional<vp2s::NativeFrameCopy> submitted;
    struct Pause{};
    vp2s::NativeScene &nativeScene()override{return scene;}
    vp2s::NativeFrameCopies &nativeFrameCopies()override{return copies;}
    void drawNativeFrameCopy(const vp2s::NativeFrameCopy &copy)override{check(copy.object==object);submitted=copy;++draws;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("library");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(reference && pc==0x2dc9a4){check(c->r[4].uw[0]==object+0x90);++draws;return false;}
        if(auto handled=vp2s::nativeFrameCopyInstruction(c,rt,pc))return *handled;
        if(vp2s::nativeSceneInstruction(r,c,rt,pc))return true;
        throw std::runtime_error("unexpected frame-copy hook");
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        if(pc==0x112960){check(c->r[4].uw[0]==target && c->r[5].uw[0]<=4);c->r[2].ud[0]=0x80000+c->r[5].uw[0]*64;}
        else if(pc==0x29d9b0){check(c->r[4].uw[0]==manager);c->r[2].ud[0]=fail?0:allocations%2?0x3400:0x2800;++allocations;}
        else if(pc==0x23da20){check(c->r[4].uw[0]==manager);c->r[2].ud[0]=0x4000;}
        else if(pc==0x29d7c0){check(c->r[4].uw[0]==manager);c->r[2].ud[0]=0;}
        else if(pc==0x1130f0){check(draws && c->r[4].uw[0]==queue);++callbacks;++*d.runtime().memory.hostPointer(queue,1);}
        else throw std::runtime_error("unexpected frame-copy leaf");
        c->pc=c->r[31].uw[0];if(pause && pc==0x1130f0)throw Pause{};
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
void run(bozo::ee::Dispatcher &d,R5900Context &c,uint32_t pc){
    auto fn=d.runtime().functions.generated(pc);unsigned attempts=0;
    while(true){try{fn(d.runtime().memory.flat(),&c,&d.runtime());if(c.pc==0x170000)break;}catch(const Host::Pause &){}
        check(++attempts<4);auto entry=d.runtime().functions.resumeEntry(c.pc);check(entry);fn=entry->resume;}
}
}
int main(){try{
    Host old,now;old.reference=true;bozo::ee::Dispatcher a(old),b(now);
    std::vector<BozoFunctionEntry> af,bf;std::vector<BozoResumeEntry> ar,br;
#define ADD(to,res,prefix) to.insert(to.end(),prefix##_function_table,prefix##_function_table+prefix##_function_count);res.insert(res.end(),prefix##_resume_table,prefix##_resume_table+prefix##_resume_count)
    ADD(af,ar,copy_original);ADD(af,ar,copy_helpers_original);ADD(bf,br,copy_adapted);ADD(bf,br,copy_helpers_adapted);
#undef ADD
    for(auto *table:{&af,&bf})std::sort(table->begin(),table->end(),[](auto x,auto y){return x.address<y.address;});
    for(auto *table:{&ar,&br})std::sort(table->begin(),table->end(),[](auto x,auto y){return x.start<y.start;});
    a.setFunctions(af.data(),af.size(),ar.data(),ar.size());b.setFunctions(bf.data(),bf.size(),br.data(),br.size());
    a.runtime().port=&old;b.runtime().port=&now;
    for(auto *d:{&a,&b})for(uint32_t pc:{0x112960u,0x29d9b0u,0x23da20u,0x29d7c0u,0x1130f0u})d->bindLibrary(pc);
    const auto saved=_mm_getcsr();unsigned builders=0,submissions=0,disabled=0,failed=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<32;++seed){
        auto &x=a.runtime();auto &y=b.runtime();std::memset(x.memory.flat(),0,bozo::kFlatBytes);
        old.allocations=now.allocations=0;old.draws=now.draws=old.callbacks=now.callbacks=0;old.fail=now.fail=seed%8==3;
        old.pause=now.pause=seed&4;now.submitted.reset();old.copies.release(object);now.copies.release(object);
        const bool enabled=seed%8!=0;const auto top=seed&1?0x70003f00u:0x1fff000u;
        put(x,0x35ddf8,current);put(x,0x35dd78,queue);put(x,0x35de38,manager);put(x,current+0x444c,object);put(x,current+0x4448,post);
        put(x,object,uint32_t(0x159ee0));put(x,object+0x90,uint32_t(0x159f5c));put(x,object+0xc4,writer);put(x,writer,buffer);put(x,writer+0x10,uint32_t(8192));
        put(x,post+0x94,target);put(x,post+0xdfa4,uint8_t(enabled));put(x,post+0xdfa5,uint8_t(seed&1));put(x,post+0xdfa8,seed&2?0.75f:1.42f);
        put(x,post+0xef0c,uint8_t(1));put(x,post+0xf328,uint32_t(1));
        put(x,target+0x3c76,uint8_t(1));put(x,target+0x3c60,1.f);put(x,target+0x3c64,seed&2?1.001875f:1.f);
        for(auto at:{0x3c68u,0x3c6au,0x3c6cu,0x3c6eu})put(x,target+at,int16_t(512));
        for(unsigned i=0;i<2;++i){const auto at=target+0xa70*i;put(x,at+0x1cf0,uint8_t(1));put(x,at+0x1cf2,uint8_t(i));put(x,at+0x1cf5,uint8_t(1));put(x,at+0x1cfa,int16_t(512));put(x,at+0x1cfc,int16_t(512));}
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        for(unsigned repeat=0;repeat<2;++repeat){
            for(auto *d:{&a,&b})put(d->runtime(),object+0x63,uint8_t(0));
            old.scene.beginComposition(&x,top,target);now.scene.beginComposition(&y,top,target);
            R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
            std::memset(p.vu0_vf,0x6a,sizeof(p.vu0_vf));p.vu0_status=0x9876;p.r[4].ud[0]=object;p.r[29].ud[0]=top;p.r[31].ud[0]=0x170000;auto q=p;
            const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);_mm_setcsr(control);
            run(a,p,0x2dbde0);const auto after=_mm_getcsr();_mm_setcsr(control);
            try{run(b,q,0x2dbde0);}catch(const std::exception &e){std::fprintf(stderr,"copy builder seed=%u repeat=%u mode=%x: %s\n",seed,repeat,mode,e.what());throw;}
            if(_mm_getcsr()!=after || std::memcmp(&p,&q,sizeof(p)) || std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes)){
                unsigned cpu=0,ram=0;while(cpu<sizeof(p) && reinterpret_cast<uint8_t *>(&p)[cpu]==reinterpret_cast<uint8_t *>(&q)[cpu])++cpu;
                while(ram<bozo::kFlatBytes && x.memory.flat()[ram]==y.memory.flat()[ram])++ram;
                std::fprintf(stderr,"copy mismatch seed=%u repeat=%u mode=%x FP=%x/%x CPUbyte=%x RAM=%x\n",seed,repeat,mode,after,_mm_getcsr(),cpu,ram);check(false);}
            ++builders;
            const bool built=get<uint8_t>(x,object+0x63);check(built==(enabled && !old.fail));
            if(built){
                const uint8_t layer=seed&1;for(auto *d:{&a,&b}){auto &rt=d->runtime();put(rt,object+0x65,layer);put(rt,current+0x40a8+4*layer,object);put(rt,current+0x40dc+layer,uint8_t(1));}
                p={};p.r[4].ud[0]=object+0x90;p.r[29].ud[0]=top;p.r[31].ud[0]=0x170000;q=p;
                run(a,p,0x2dc9a0);run(b,q,0x2dc9a0);
                check(!std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
                check(now.submitted && now.submitted->width==get<uint32_t>(x,object+0x1a0) && now.submitted->height==get<uint32_t>(x,object+0x1a4));
                check(get<uint8_t>(y,object+0x62)==1 && get<uint8_t>(y,object+0x64)==0 && get<uint8_t>(y,current+0x40dc+layer)==0);
                check(old.draws==now.draws && old.callbacks==now.callbacks);++submissions;
            }else if(!enabled)++disabled;else ++failed;
            old.scene.finishComposition(top);now.scene.finishComposition(top);
        }
    }
    _mm_setcsr(saved);check(submissions && disabled && failed);
    std::printf("%u complete scalar copy builders, %u native submissions/real completions, %u disabled/%u failed allocations; CPU/RAM/FP, four modes, real writer helpers, reuse and paused callback passed\n",builders,submissions,disabled,failed);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

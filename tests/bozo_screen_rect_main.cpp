#include "native_screen_rect.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "rect_screen_original_functions.h"
#include "rect_screen_adapted_functions.h"
#include "rect_flush_original_functions.h"
#include "rect_flush_adapted_functions.h"
#include "rect_completion_original_functions.h"
#include "rect_completion_adapted_functions.h"
#include <cstdio>
#include <cstring>
#include <source_location>
#include <stdexcept>
#include <vector>
#include <algorithm>
namespace {
void check(bool ok,std::source_location loc=std::source_location::current()){
    if(!ok)throw std::runtime_error("native screen rectangle assertion at "+std::to_string(loc.line()));
}
template<class T>void put(Runtime &rt,uint32_t at,T value){std::memcpy(rt.memory.hostPointer(at,sizeof(T)),&value,sizeof(T));}
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::ScreenRectOwner,vp2s::SceneOwner {
    struct Pause{};bool pause=false,reject=false;unsigned enqueues=0,completions=0;
    vp2s::NativeScreenRects rects;vp2s::NativeScene scene;std::vector<vp2s::NativeScreenRect> draws;
    vp2s::NativeScreenRects &nativeScreenRects()override{return rects;}
    vp2s::NativeScene &nativeScene()override{return scene;}
    void drawNativeScreenRect(const vp2s::NativeScreenRect &rect)override{
        if(reject)throw std::runtime_error("fixture renderer rejected rectangle");draws.push_back(rect);
    }
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("library");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(auto known=vp2s::nativeScreenRectInstruction(c,rt,pc))return *known;
        if(vp2s::nativeSceneInstruction(r,c,rt,pc))return true;
        throw std::runtime_error("unowned screen fixture instruction");
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        if(pc==0x1131c0){rects.retire(&d.runtime(),c);++enqueues;c->r[2].ud[0]=7;}
        else if(pc==0x1130f0){check(c->r[4].uw[0]==0x1b0000);++completions;++*d.runtime().memory.hostPointer(0x1b0000,1);c->r[2].ud[0]=0;}
        else throw std::runtime_error("unowned screen fixture leaf");
        c->pc=c->r[31].uw[0];if(pc==0x1130f0 && pause)throw Pause{};
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device write");}
    bool cop0Condition(bozo::ee::Dispatcher &,R5900Context *)override{throw std::runtime_error("device completion");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &why)override{throw std::runtime_error(why);}
};
void run(bozo::ee::Dispatcher &d,R5900Context &c){
    auto fn=d.runtime().functions.generated(0x372180);check(fn);unsigned attempts=0;
    while(true){try{fn(d.runtime().memory.flat(),&c,&d.runtime());if(c.pc==0x170000)break;}catch(const Host::Pause &){}
        check(++attempts<6);auto next=d.runtime().functions.resumeEntry(c.pc);check(next);fn=next->resume;}
}
}
int main(){try{
    Host oldHost,nativeHost;bozo::ee::Dispatcher old(oldHost),native(nativeHost);
    std::vector<BozoFunctionEntry> before,after;std::vector<BozoResumeEntry> beforeResume,afterResume;
#define ADD_TABLE(to, res, prefix) to.insert(to.end(),prefix##_function_table,prefix##_function_table+prefix##_function_count); res.insert(res.end(),prefix##_resume_table,prefix##_resume_table+prefix##_resume_count)
    ADD_TABLE(before,beforeResume,rect_screen_original);ADD_TABLE(before,beforeResume,rect_flush_original);ADD_TABLE(before,beforeResume,rect_completion_original);
    ADD_TABLE(after,afterResume,rect_screen_adapted);ADD_TABLE(after,afterResume,rect_flush_adapted);ADD_TABLE(after,afterResume,rect_completion_adapted);
#undef ADD_TABLE
    for(auto *table:{&before,&after})std::sort(table->begin(),table->end(),[](const auto &a,const auto &b){return a.address<b.address;});
    for(auto *table:{&beforeResume,&afterResume})std::sort(table->begin(),table->end(),[](const auto &a,const auto &b){return a.start<b.start;});
    old.setFunctions(before.data(),before.size(),beforeResume.data(),beforeResume.size());native.setFunctions(after.data(),after.size(),afterResume.data(),afterResume.size());
    old.runtime().port=&oldHost;native.runtime().port=&nativeHost;
    for(auto *d:{&old,&native}){d->boot(0x372180,0x1fff000);d->bindLibrary(0x1130f0);}old.bindLibrary(0x1131c0);
    for(unsigned seed=0;seed<64;++seed){
        auto &a=old.runtime(),&b=native.runtime();std::memset(a.memory.flat(),0,bozo::kFlatBytes);
        constexpr uint32_t object=0x190000,writer=0x1a0000,base=0x280000,scene=0x1d0000,source=0x1e0000;
        put(a,object,0x15bc00u);put(a,object+0x90,0x15bc7cu);put(a,object+0xc0,writer);
        put(a,object+0x9c,0x40002010u+seed);put(a,object+0xbc,0xffff00u);
        put(a,object+0xac,int32_t(seed)-16);put(a,object+0xb0,int32_t(seed)-8);put(a,object+0xb4,640u);put(a,object+0xb8,448u);
        put(a,object+0x65,uint8_t(seed&1));put(a,object+0xf1,uint8_t(seed));
        put(a,0x35ddf8,scene);put(a,0x35dd78,0x1b0000u);put(a,0x35de60,source);
        put(a,source+0x3c6c,int16_t(640+(seed&1)));put(a,source+0x3c6e,int16_t(448+(seed&1)));
        put(a,scene+0x40a8+(seed&1)*4,object);put(a,scene+0x40dc+(seed&1),uint8_t(1));put(a,scene+0x4088,0x1c0000u);
        put(a,writer,base);put(a,writer+8,base+0x40+224);put(a,writer+12,base+0x40);put(a,writer+16,512u);
        std::memset(a.memory.flat()+base,0xa5,0x200);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{},y{};x.r[4].ud[0]=object;x.r[29].ud[0]=seed&2?0x70003f00:0x1fff000;x.r[31].ud[0]=0x170000;y=x;
        nativeHost.pause=seed&32;run(old,x);run(native,y);
        check(x.cycles==y.cycles+8 && x.checkpoints==y.checkpoints+1);x.cycles-=8;--x.checkpoints;x.cycleLimit=y.cycleLimit;
        if(seed&1)x.r[2]=y.r[2]; // Void scene draw's dead enqueue result.
        check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        check(!oldHost.rects.outstanding() && !nativeHost.rects.outstanding() && nativeHost.draws.size()==seed+1);
        const auto held=nativeHost.draws.back();check(held.object==object && held.colour==0x40002010u+seed && held.x==int32_t(seed)-16 && held.y==int32_t(seed)-8 && held.right==640 && held.bottom==448 && held.depth==0xffff00+seed);
        check(held.sourceWidth==640+int(seed&1) && held.sourceHeight==448+int(seed&1));
        const auto vertices=vp2s::nativeScreenRectangleVertices(held);
        check(vertices[0].x==float(2048-(640+int(seed&1))/2+int(seed)-16) && vertices[0].y==float(2048-(448+int(seed&1))/2+int(seed)-8));
        check(vertices[1].x==float(2048-(640+int(seed&1))/2+640) && vertices[2].y==float(2048-(448+int(seed&1))/2+448));
        check(vertices[4].rgba==held.colour && vertices[4].z==float(0xffff00+seed));
        std::memset(b.memory.flat()+object+0x9c,0,4);check(nativeHost.draws.back().colour==held.colour);
    }
    {auto &b=native.runtime();constexpr uint32_t object=0x190000,writer=0x1a0000,base=0x280000,stack=0x1ffef00;
        put(b,0x1d0000+0x40dd,uint8_t(1));put(b,writer+8,base+0x40+224);put(b,writer+12,base+0x40);
        const auto rejects=[&](auto fn){bool failed=false;try{fn();}catch(const std::runtime_error &){failed=true;}check(failed);};
        for(uint8_t mode:{1,2}){put(b,object+0xf3,mode);rejects([&]{nativeHost.rects.begin(&b,stack,object);});check(!nativeHost.rects.outstanding());}
        put(b,object+0xf3,uint8_t(0));put(b,writer+8,base+0x40+240);
        rejects([&]{nativeHost.rects.begin(&b,stack,object);});check(!nativeHost.rects.outstanding());put(b,writer+8,base+0x40+224);
        R5900Context c{};c.pc=0x372180;c.r[4].ud[0]=object;c.r[5].ud[0]=object+0x90;c.r[29].ud[0]=stack;const auto saved=c;
        std::vector<uint8_t> before(b.memory.flat(),b.memory.flat()+bozo::kFlatBytes);nativeHost.reject=true;
        rejects([&]{vp2s::nativeScreenRectInstruction(&c,&b,0x3721a4);});nativeHost.reject=false;
        check(!nativeHost.rects.outstanding() && !std::memcmp(&c,&saved,sizeof(c)) && !std::memcmp(before.data(),b.memory.flat(),before.size()));
        nativeHost.rects.begin(&b,stack,object);rejects([&]{nativeHost.rects.begin(&b,stack,object);});
        rejects([&]{nativeHost.rects.complete(stack);});check(nativeHost.rects.outstanding()==1);nativeHost.rects.cancel(stack);
    }
    check(oldHost.enqueues==64 && nativeHost.enqueues==0 && oldHost.completions==32 && nativeHost.completions==32);
    std::puts("64 native solid-screen submission/scalar-flush/completion CPU/RAM cases, both scene passes and nested callback resume; copied EE geometry/colour, no packet consumer");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "native_math.h"
#include "native_scene.h"
#include "install_original_functions.h"
#include "install_adapted_functions.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <source_location>
namespace {
void check(bool value,std::source_location at=std::source_location::current()){
    if(!value)throw std::runtime_error("graphics install retirement assertion failed at "+std::to_string(at.line()));
}
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::SceneOwner {
    unsigned enqueues=0;
    bool pause=false;
    struct Pause{};
    vp2s::NativeScene scene;
    std::optional<vp2s::NativeVisibilityInputs> lastVisibility;
    vp2s::NativeScene &nativeScene()override{return scene;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(auto handled=vp2s::nativeVisibilityInstruction(c,rt,pc)){
            if(pc==0x2206b8)lastVisibility=scene.visibility(c->r[29].uw[0]);return *handled;
        }
        if(vp2s::nativeSceneInstruction(r,c,rt,pc) || vp2s::nativeMathInstruction(r,c,rt,pc))return true;
        throw std::runtime_error("unported graphics fixture algorithm");
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        if(pc==0x2082f0){
            const float first=1,second=2,clip=999,view=888;
            std::memcpy(d.runtime().memory.hostPointer(c->r[5].uw[0],4),&first,4);
            std::memcpy(d.runtime().memory.hostPointer(c->r[6].uw[0],4),&second,4);
            std::memcpy(d.runtime().memory.hostPointer(c->r[4].uw[0]+0x3f0,4),&clip,4);
            std::memcpy(d.runtime().memory.hostPointer(c->r[4].uw[0]+0x2f0,4),&view,4);
            c->pc=c->r[31].uw[0];if(pause)throw Pause{};return;
        }else if(pc==0x201f20){
            check(c->r[4].uw[0]==0x180000);
            if(c->r[31].uw[0]==0x208a98){check(c->r[5].uw[0]==0x1b0000);scene.publishCamera(&d.runtime(),c->r[17].uw[0]);}
            else check(c->r[5].uw[0]==0x190000);
            ++enqueues;c->r[2].sd[0]=1;
        }else if(pc==0x170100)c->r[2].sd[0]=0x1c0000;
        else if(pc==0x2138f0){auto *to=d.runtime().memory.hostPointer(c->r[4].uw[0],16);auto *from=d.runtime().memory.hostPointer(c->r[5].uw[0],16);check(to && from);std::memcpy(to,from,16);}
        else if(pc==0x215870){const float angles[4]={0.1f,0.2f,0.3f,0};auto *to=d.runtime().memory.hostPointer(c->r[4].uw[0],16);check(to);std::memcpy(to,angles,16);}
        else throw std::runtime_error("unexpected native scene leaf");
        c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("unexpected VU execution");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("unexpected device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("unexpected device write");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host oldHost,nativeHost;bozo::ee::Dispatcher old(oldHost),native(nativeHost);
    old.setFunctions(install_original_function_table,install_original_function_count,install_original_resume_table,install_original_resume_count);
    native.setFunctions(install_adapted_function_table,install_adapted_function_count,install_adapted_resume_table,install_adapted_resume_count);
    old.runtime().port=&oldHost;native.runtime().port=&nativeHost;
    old.boot(0x29e2e0,0x1fff000);native.boot(0x29e2e0,0x1fff000);old.bindLibrary(0x201f20);
    for(auto *d:{&old,&native})for(uint32_t pc:{0x170100u,0x2138f0u,0x215870u,0x2082f0u})d->bindLibrary(pc);
    for(unsigned seed=0;seed<64;++seed){
        auto *a=old.runtime().memory.flat(),*b=native.runtime().memory.flat();
        std::memset(a,seed+1,bozo::kFlatBytes);uint32_t queue=0x180000;std::memcpy(a+0x35dde0,&queue,4);
        std::memcpy(b,a,bozo::kFlatBytes);
        R5900Context x{},y{};
        for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x1234000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        x.r[4].ud[0]=0x190000;x.r[29].ud[0]=0x1fff000;x.r[31].ud[0]=0x170000;y=x;
        old.runtime().functions.generated(0x29e2e0)(a,&x,&old.runtime());
        native.runtime().functions.generated(0x29e2e0)(b,&y,&native.runtime());
        check(x.pc==0x170000 && y.pc==x.pc && oldHost.enqueues==seed+1 && nativeHost.enqueues==0);
        check(b[0x190004]==0 && b[0x190005]==255);
        check(!std::memcmp(a,b,bozo::kFlatBytes));
        // The void producer's enqueue result and transient call ra are dead in
        // its only audited caller. No completion callback/state is synthesized.
        x.r[2]=y.r[2];check(!std::memcmp(x.r,y.r,sizeof(x.r)));
    }
    for(unsigned seed=0;seed<64;++seed){
        auto *a=old.runtime().memory.flat(),*b=native.runtime().memory.flat();std::memset(a,seed+1,bozo::kFlatBytes);
        uint32_t queue=0x180000,type=0x1591d0;std::memcpy(a+0x35ddd8,&queue,4);std::memcpy(a+0x190000,&type,4);
        a[0x190050]=0;a[0x190051]=seed;std::memcpy(b,a,bozo::kFlatBytes);
        R5900Context x{},y{};x.r[4].ud[0]=0x190000;x.r[5].ud[0]=0;x.r[29].ud[0]=0x1fff000;x.r[31].ud[0]=0x170000;y=x;
        old.runtime().functions.generated(0x29e150)(a,&x,&old.runtime());native.runtime().functions.generated(0x29e150)(b,&y,&native.runtime());
        check(a[0x190050]==1 && b[0x190050]==0 && a[0x190051]==0 && b[0x190051]==0);
        // No native job exists, so its legacy busy store is deliberately absent.
        a[0x190050]=0;check(!std::memcmp(a,b,bozo::kFlatBytes));
        check(x.cycles==y.cycles+8 && x.checkpoints==y.checkpoints+1);x.cycles-=8;--x.checkpoints;
        // Only the original out-call asks the dispatcher for its host slice
        // deadline. This is scheduling metadata, not a program result.
        check(x.cycleLimit==y.cycleLimit || y.cycleLimit==~0ull);x.cycleLimit=y.cycleLimit;
        check(!std::memcmp(&x,&y,sizeof(x)));
    }
    for(unsigned mode:{1u,128u,255u}){
        auto *b=native.runtime().memory.ram();R5900Context c{};c.r[4].ud[0]=0x190000;c.r[5].ud[0]=mode;
        b[0x190050]=0;bool rejected=false;try{vp2s::nativeMathInstruction(b,&c,&native.runtime(),0x29e160);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && b[0x190050]==0);
    }
    for(unsigned seed=0;seed<64;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0,bozo::kFlatBytes);
        const auto word=[&](uint32_t at,uint32_t value){std::memcpy(a.memory.ram()+at,&value,4);};
        word(0x35dde0,0x180000);word(0x190000,0x1a0000);word(0x1a0068,0x170100);
        word(0x1908e0,0x1b0000);word(0x1b0000,0x153140);word(0x1905c8,0x1d0000);word(0x1905a0,0x123400+seed);
        a.memory.ram()[0x190445]=seed&1;
        for(unsigned i=0;i<48;++i){const float value=float(i+seed)/128.f;std::memcpy(a.memory.ram()+0x1902f0+i*4,&value,4);}
        const float fog[2]={127.f,-1024.f};std::memcpy(a.memory.ram()+0x1905a8,fog,8);std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{},y{};x.r[4].ud[0]=0x190000;x.r[29].ud[0]=0x1fff000;x.r[31].ud[0]=0x170000;y=x;
        const auto count=nativeHost.scene.publicationCount();
        a.functions.generated(0x208900)(a.memory.flat(),&x,&a);b.functions.generated(0x208900)(b.memory.flat(),&y,&b);
        const unsigned published=!(seed&1);check(nativeHost.scene.publicationCount()==count+published);
        check(!std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        check(x.cycles==y.cycles+published*8 && x.checkpoints==y.checkpoints+published);
        x.cycles-=published*8;x.checkpoints-=published;x.cycleLimit=y.cycleLimit;
        // The void camera method's enqueue result is dead. Everything else,
        // including source records, CPU temporaries and vector/flag state matches.
        x.r[2]=y.r[2];check(!std::memcmp(&x,&y,sizeof(x)));
        if(published){auto view=nativeHost.scene.camera(0x190000);check(view && view->clearColour==0x123400+seed);
            check(!std::memcmp(view->view.data(),a.memory.ram()+0x1902f0,64) && !std::memcmp(view->screen.data(),a.memory.ram()+0x190330,64) && !std::memcmp(view->guard.data(),a.memory.ram()+0x190370,64));
            check(view->fog[0]==127 && view->fog[1]==-1024);
            const auto colour=view->clearColour;word(0x1905a0,0);check(view->clearColour==colour);
        }
    }
    for(unsigned seed=0;seed<64;++seed){
        auto &a=old.runtime(),&b=native.runtime();std::memset(a.memory.flat(),seed+1,bozo::kFlatBytes);
        const uint32_t camera=seed&1?0x180000:0x190000;
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{},y{};x.r[4].ud[0]=camera;x.r[31].ud[0]=0x170000;
        std::memset(x.vu0_vf,0x6a,sizeof(x.vu0_vf));std::memset(x.vi,0x7b,sizeof(x.vi));y=x;
        auto previous=nativeHost.scene.frustum();const auto count=nativeHost.scene.frustumPublicationCount();
        a.functions.generated(0x207cc0)(a.memory.flat(),&x,&a);b.functions.generated(0x207cc0)(b.memory.flat(),&y,&b);
        check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        check(nativeHost.scene.frustumPublicationCount()==count+1);
        auto view=nativeHost.scene.frustum();check(view && view->object==camera);
        check(!std::memcmp(view->points.data(),a.memory.flat()+camera+0x460,160) && !std::memcmp(view->normals.data(),a.memory.flat()+camera+0x500,160));
        if(previous)check(previous!=view && std::bit_cast<uint32_t>(previous->points[0][0])==0x01010101u*seed);
        std::memset(b.memory.flat()+camera+0x460,0,320);check(std::bit_cast<uint32_t>(view->points[0][0])==0x01010101u*(seed+1));
    }
    bool invalid=false;R5900Context bad{};bad.pc=0x207ce0;
    try{vp2s::nativeSceneInstruction(nullptr,&bad,&native.runtime(),0x207ce0);}catch(const std::runtime_error &){invalid=true;}check(invalid);
    for(unsigned seed=0;seed<64;++seed){
        auto &a=old.runtime(),&b=native.runtime();std::memset(a.memory.flat(),0,bozo::kFlatBytes);
        constexpr uint32_t object=0x180000,camera=0x190000;
        const auto word=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.flat()+at,&v,4);};
        word(object+8,object);word(object+0x4080,0x1a0000);word(0x1a0094,seed%4==0?0:camera);
        a.memory.flat()[camera+0x5a4]=(seed&2)!=0;
        for(unsigned i=0;i<16;++i){const float v=float(seed+i)/32;std::memcpy(a.memory.flat()+camera+0x3f0+i*4,&v,4);std::memcpy(a.memory.flat()+camera+0x2f0+i*4,&v,4);}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);oldHost.lastVisibility.reset();nativeHost.lastVisibility.reset();
        R5900Context x{},y{};x.r[4].ud[0]=object;x.r[29].ud[0]=seed&1?0x70003f00:0x1fff000;x.r[31].ud[0]=0x170000;
        std::memset(x.vu0_vf,0x6a,sizeof(x.vu0_vf));y=x;
        oldHost.pause=false;nativeHost.pause=(seed&32)!=0;
        a.functions.generated(0x2205c0)(a.memory.flat(),&x,&a);
        auto fn=b.functions.generated(0x2205c0);unsigned attempts=0;
        while(true){try{fn(b.memory.flat(),&y,&b);break;}catch(const Host::Pause &){}
            check(++attempts<4);auto entry=b.functions.resumeEntry(y.pc);check(entry);fn=entry->resume;}
        x.cycleLimit=y.cycleLimit;
        check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        const bool expected=seed%4!=0;check(nativeHost.lastVisibility.has_value()==expected);
        if(expected){const auto &v=*nativeHost.lastVisibility;check(v.scene==object && v.camera==camera && v.viewReady && v.frustum && v.frustum->object==camera);
            check(v.clip[0]==float(seed)/32 && !std::memcmp(v.clip.data()+1,a.memory.flat()+camera+0x3f4,60) && !std::memcmp(v.view.data(),a.memory.flat()+camera+0x2f0,64));
            std::memset(b.memory.flat()+camera,0,0x600);check(v.clip[0]==float(seed)/32);}
        bool released=false;try{nativeHost.scene.visibility((seed&1?0x70003f00:0x1fff000)-0x130);}catch(const std::runtime_error &){released=true;}check(released);
    }
    nativeHost.scene.beginVisibility(0x1fff000,0x180000);bool duplicate=false;
    try{nativeHost.scene.beginVisibility(0x1fff000,0x180000);}catch(const std::runtime_error &){duplicate=true;}check(duplicate);
    nativeHost.scene.finishVisibility(0x1fff000,0x180000);
    std::puts("320 graphics/math/camera/frustum/visibility CPU/RAM cases; immutable snapshots, invocation lifetimes and no transport state");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

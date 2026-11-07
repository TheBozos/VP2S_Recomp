#include "native_scene.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "composition_original_functions.h"
#include "composition_adapted_functions.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <source_location>
#include <limits>
#include <vector>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){
    if(!v)throw std::runtime_error("native composition assertion at "+std::to_string(at.line()));
}
struct Pause{};
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::SceneOwner {
    vp2s::NativeScene scene;unsigned targets=0,enqueues=0,displays=0,physicalStores=0,waits=0;bool adapted=false,pause=false;
    vp2s::NativeScene &nativeScene()override{return scene;}
    void beginNativeFrame(const vp2s::NativeFrameTarget &t)override{check(t.object==0x180000 && t.width==640 && t.height==448);++targets;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(!adapted){check(pc==0x114ab4 || pc==0x114ac0);++physicalStores;return true;}
        auto result=vp2s::nativeCompositionInstruction(r,c,rt,pc);check(result.has_value());return *result;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *c,uint32_t pc)override{
        bool stop=false;
        if(pc==0x1010e0)c->r[2].sd[0]=0x170000;
        else if(pc==0x11f5a0){if(adapted)check(targets==waits);c->r[2].sd[0]=waits++%2;stop=pause;}
        else if(pc==0x10d0f0)c->r[2].sd[0]=0x1c8d00;
        else if(pc==0x1131c0){check(!adapted && (c->r[5].uw[0]==2 || c->r[5].uw[0]==4));++enqueues;c->r[2].sd[0]=7;}
        else if(pc==0x122160){check(!adapted && c->r[4].uw[0]==0x180000);++displays;c->r[2].sd[0]=0;}
        else throw std::runtime_error("unexpected secondary composition leaf");
        c->pc=c->r[31].uw[0];if(stop)throw Pause{};
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
template<class T>void value(uint8_t *r,uint32_t at,T v){std::memcpy(r+at,&v,sizeof(v));}
void run(bozo::ee::Dispatcher &d,R5900Context &c){
    auto fn=d.runtime().functions.generated(0x114550);unsigned attempts=0;
    while(true){try{fn(d.runtime().memory.flat(),&c,&d.runtime());break;}catch(const Pause &){}
        check(++attempts<4);auto entry=d.runtime().functions.resumeEntry(c.pc);check(entry);fn=entry->resume;
    }
}
}
int main(){try{
    Host original,adapted;adapted.adapted=true;bozo::ee::Dispatcher a(original),b(adapted);
    a.setFunctions(composition_original_function_table,composition_original_function_count,composition_original_resume_table,composition_original_resume_count);
    b.setFunctions(composition_adapted_function_table,composition_adapted_function_count,composition_adapted_resume_table,composition_adapted_resume_count);
    a.runtime().port=&original;b.runtime().port=&adapted;
    for(auto *d:{&a,&b})for(uint32_t pc:{0x1010e0u,0x11f5a0u,0x10d0f0u,0x1131c0u,0x122160u})d->bindLibrary(pc);
    for(unsigned seed=0;seed<64;++seed){
        auto *x=a.runtime().memory.flat(),*y=b.runtime().memory.flat();std::memset(x,0,bozo::kFlatBytes);
        const uint32_t object=0x180000;value<int16_t>(x,object+0x3c68,640);value<int16_t>(x,object+0x3c6a,448);
        value<float>(x,object+0x3c60,1);value<float>(x,object+0x3c64,1);x[object+0x3c76]=1;x[object+0x3e67]=(seed>>1)&1;
        value<uint32_t>(x,object+0x3dd4,seed%2);
        for(unsigned i=0;i<2;++i){auto *layer=x+object+i*0xa70;layer[0x1cf0]=1;layer[0x1cf1]=(seed>>2)&1;
            layer[0x1cf2]=i^(seed&1);layer[0x1cf5]=1;value<int16_t>(layer,0x1cfa,640);value<int16_t>(layer,0x1cfc,224);}
        std::memcpy(y,x,bozo::kFlatBytes);
        R5900Context old{},now{};for(unsigned i=1;i<32;++i){old.r[i].ud[0]=0x1234000000000000ull+seed*32+i;old.r[i].ud[1]=~old.r[i].ud[0];}
        old.r[4].ud[0]=object;old.r[5].ud[0]=seed%2;old.r[6].ud[0]=0x190000;old.r[29].ud[0]=0x1fff000;old.r[31].ud[0]=0x170000;now=old;
        original.enqueues=original.displays=original.physicalStores=0;original.pause=adapted.pause=(seed&8)!=0;
        run(a,old);run(b,now);check(!std::memcmp(x,y,bozo::kFlatBytes));
        const unsigned removed=original.enqueues+original.displays;
        check(old.cycles==now.cycles+removed*8 && old.checkpoints==now.checkpoints+removed);
        old.cycles-=removed*8;old.checkpoints-=removed;old.cycleLimit=now.cycleLimit;
        // This void producer's final enqueue result is dead in its callers.
        old.r[2]=now.r[2];check(!std::memcmp(&old,&now,sizeof(old)));
        check(adapted.enqueues==0 && adapted.displays==0 && adapted.physicalStores==0 && adapted.targets==seed+1);
        check(original.physicalStores==(((seed>>2)&1)?2:0));
        const auto frame=adapted.scene.frameTarget();check(frame && frame->width==640 && frame->height==448);
        bool finished=false;try{adapted.scene.composition(0x1fff000-0xa0);}catch(const std::runtime_error &){finished=true;}check(finished);
    }
    auto *ram=b.runtime().memory.flat();const uint32_t object=0x180000;
    const auto preserved=adapted.scene.frameTarget();
    const auto rejects=[&](auto mutate){
        std::vector<uint8_t> snapshot(ram+object,ram+object+0x3e88);mutate();bool rejected=false;
        try{adapted.scene.beginComposition(&b.runtime(),0x1fff000,object);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && adapted.scene.frameTarget()==preserved);std::memcpy(ram+object,snapshot.data(),snapshot.size());
    };
    rejects([&]{ram[object+0x3c76]=0;});
    rejects([&]{value<uint32_t>(ram,object+0x3dd8,1);});
    rejects([&]{ram[object+0x3e48]=1;});
    rejects([&]{value<int16_t>(ram,object+0x3e78,1);});
    rejects([&]{ram[object+0x31d0]=1;});
    rejects([&]{ram[object+0x2762]=ram[object+0x1cf2];});
    rejects([&]{value<float>(ram,object+0x3c60,std::numeric_limits<float>::quiet_NaN());});
    rejects([&]{value<int16_t>(ram,object+0x1cfc,200);});
    rejects([&]{ram[object+0x1cf4]=1;});
    auto target=adapted.scene.beginComposition(&b.runtime(),0x1fff000,object);check(target.height==448);
    bool reused=false;try{adapted.scene.beginComposition(&b.runtime(),0x1fff000,object);}catch(const std::runtime_error &){reused=true;}check(reused);
    adapted.scene.finishComposition(0x1fff000);check(preserved->width==640 && preserved->height==448);
    std::puts("64 generated source-target composition CPU/RAM comparisons, field pair selection and resumed waits passed; no transfer consumer");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

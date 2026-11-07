#include "native_ui_sprites.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "button_original_functions.h"
#include "button_adapted_functions.h"
#include <cstdio>
#include <cstring>
#include <source_location>
#include <stdexcept>
namespace {
void check(bool ok,std::source_location loc=std::source_location::current()){
    if(!ok)throw std::runtime_error("native button assertion at "+std::to_string(loc.line()));
}
template<class T>void put(Runtime &rt,uint32_t at,T v){std::memcpy(rt.memory.hostPointer(at,sizeof(T)),&v,sizeof(T));}
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::UiSpriteOwner,vp2s::TextureOwner {
    vp2s::NativeUiSprites sprites;vp2s::NativeTextures textures;bool pause=false;struct Pause{};
    explicit Host(const std::filesystem::path &path):textures(path,path/"manifest.txt"){}
    vp2s::NativeUiSprites &nativeUiSprites()override{return sprites;}
    vp2s::NativeTextures &nativeTextures()override{return textures;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc)override{
        auto result=vp2s::nativeUiSpriteInstruction(c,rt,pc);check(result.has_value());return *result;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *c,uint32_t pc)override{
        check(pc==0x2f2800 && c->r[4].uw[0]==0x1b3000);c->r[2].ud[0]=0x1b4000;c->pc=c->r[31].uw[0];if(pause)throw Pause{};
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device write");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &why)override{throw std::runtime_error(why);}
};
void run(bozo::ee::Dispatcher &d,R5900Context &c,uint32_t entry){
    auto fn=d.runtime().functions.generated(entry);unsigned attempts=0;
    while(true){try{fn(d.runtime().memory.flat(),&c,&d.runtime());if(c.pc==0x170000)break;}catch(const Host::Pause &){}
        check(++attempts<4);const auto resume=d.runtime().functions.resumeEntry(c.pc);check(resume);fn=resume->resume;}
}
}
int main(int argc,char **argv){try{
    check(argc==2);Host before(argv[1]),after(argv[1]);bozo::ee::Dispatcher old(before),native(after);
    old.setFunctions(button_original_function_table,button_original_function_count,button_original_resume_table,button_original_resume_count);
    native.setFunctions(button_adapted_function_table,button_adapted_function_count,button_adapted_resume_table,button_adapted_resume_count);
    for(auto *d:{&old,&native}){d->boot(0x2ec260,0x1fff000);d->bindLibrary(0x2f2800);}
    old.runtime().port=&before;native.runtime().port=&after;
    constexpr uint32_t sprite=0x190000,manager=0x1b0000,dtt=0x1b4000;
    for(unsigned seed=0;seed<64;++seed){
        auto &a=old.runtime(),&b=native.runtime();std::memset(a.memory.flat(),0,bozo::kFlatBytes);
        put(a,0x35e170,manager);put(a,manager,0x15a1e0u);put(a,manager+0x820,0x1b3000u);put(a,dtt+0x10,3u);
        for(unsigned i=0;i<2;++i){const uint32_t image=0x1c0000+i*0x100,fis=0x1d0000+i*0x100;
            put(a,manager+0x828+i*4,image);put(a,image,0x15a6f0u);put(a,image+0x90,0x15a76cu);put(a,image+0xc0,fis);
            put(a,fis,0x534946u);put(a,fis+0x14,0x00494853u);put(a,fis+0x18,i?0x3du:0x41u);
        }
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);R5900Context x{},y{};
        x.r[4].ud[0]=sprite;x.r[5].ud[0]=seed;x.r[29].ud[0]=seed&2?0x70003f00:0x1fff000;x.r[31].ud[0]=0x170000;y=x;after.pause=seed&1;
        run(old,x,0x2ec260);run(native,y,0x2ec260);x.cycleLimit=y.cycleLimit;
        check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        auto held=after.sprites.button(sprite);check(held && held->dtt==dtt && held->parts==dtt+0x14 && held->count==3 && held->button==seed);
        check(held->asset->key==vp2s::TextureKey{0x00494853,seed<0x26?0x41u:0x3du});auto picture=held->asset->picture();
        check(picture->rgba[0]==(0x80402000u|(seed<0x26?0x41u:0x3du)));
        auto rejects=[&](auto action){bool caught=false;try{action();}catch(const std::exception &){caught=true;}check(caught && after.sprites.button(sprite)==held);};
        const uint32_t fis=seed<0x26?0x1d0000:0x1d0100;
        put(b,fis+0x18,0x42u);rejects([&]{after.sprites.bindButton(&b,after.textures,sprite,seed);});put(b,fis+0x18,seed<0x26?0x41u:0x3du);
        put(b,sprite+8,dtt+0x18);rejects([&]{after.sprites.bindButton(&b,after.textures,sprite,seed);});put(b,sprite+8,dtt+0x14);
        rejects([&]{after.sprites.bindButton(&b,after.textures,sprite,-1);});
        // Original reset invalidates association; retained native pictures survive.
        x={};x.r[4].ud[0]=sprite;x.r[31].ud[0]=0x170000;y=x;run(old,x,0x2ec320);run(native,y,0x2ec320);x.cycleLimit=y.cycleLimit;
        check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes) && !after.sprites.button(sprite));
        check(picture->rgba[0]==(0x80402000u|(seed<0x26?0x41u:0x3du)));
    }
    std::puts("64 full button-sheet/reset CPU/RAM cases, both original ID branches, real lookup resume, typed image identity and immutable native asset lifetime passed; no sprite geometry/draw yet");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

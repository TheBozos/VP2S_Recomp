#include "native_ui_text.h"
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
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::UiSpriteOwner,vp2s::TextureOwner,vp2s::UiTextOwner {
    vp2s::NativeUiSprites sprites;vp2s::NativeTextures textures;bool pause=false;struct Pause{};
    vp2s::NativeUiTexts texts;vp2s::NativeGlyphs glyphs;
    explicit Host(const std::filesystem::path &path):textures(path,path/"manifest.txt"){}
    vp2s::NativeUiSprites &nativeUiSprites()override{return sprites;}
    vp2s::NativeTextures &nativeTextures()override{return textures;}
    vp2s::NativeUiTexts &nativeUiTexts()override{return texts;}
    vp2s::NativeGlyphs &nativeGlyphs()override{return glyphs;}
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
        constexpr uint32_t text=0x1f0000,layer=0x1f1000,control=0x1f2000,item=0x1a0000;
        put(a,text,0x15a450u);put(a,text+0x14,layer);put(a,text+0x18,2.25f);put(a,text+0x1c,3.5f);
        put(a,text+0x78,1.f);put(a,text+0x7c,2.f);put(a,text+0x28,128.f);put(a,text+0x2c,4u);
        put(a,text+0x34,0x1f3000u);put(a,text+0x100,control);put(a,text+0x74,seed&4?item:item+0x30);
        put(a,control+0x24,sprite);put(a,control+0x28,1u);put(a,control+0x2c,1u);put(a,control+0x1c,0x1f4000u);
        put(a,layer+0x40,10.f);put(a,layer+0x44,20.f);put(a,layer+0xe0,128.f);put(a,layer+0xc4,0xff0000u);
        put(a,manager+0x840,16.f);put(a,manager+0x844,16.f);put(a,manager+0x848,27648.f);put(a,manager+0x84c,29184.f);
        put(a,sprite+0x66,uint8_t(!(seed&8)));put(a,sprite+0x68,item);put(a,sprite+0x64,int8_t(-1));
        put(a,item,10.f+float(seed)/8);put(a,item+4,20.f+float(seed)/4);put(a,item+0x1c,0.5f);put(a,item+0x20,0.75f);
        for(uint32_t offset:{0x40u,0x44u,0x48u,0x4cu})put(a,sprite+offset,128.f);put(a,sprite+0x50,64.f);
        put(a,dtt+0x14,uint16_t(0x400));put(a,dtt+0x16,uint16_t(0x800));put(a,dtt+0x18,uint16_t(0xc00));put(a,dtt+0x1a,uint16_t(0x1000));
        put(a,dtt+0x1c,uint16_t(16));put(a,dtt+0x1e,uint16_t(24));put(a,dtt+0x20,uint8_t(0));
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        after.texts.enter(&b,0x1ff0000,text,0,0);after.texts.activate(0x1ff0000,text,0,0);after.texts.finish(0x1ff0000,text);
        const auto glyphSnapshot=after.texts.text(text);
        x={};x.r[4].ud[0]=text;x.r[29].ud[0]=seed&2?0x70003f00:0x1fff000;x.r[31].ud[0]=0x170000;y=x;
        run(old,x,0x2efe40);run(native,y,0x2efe40);x.cycleLimit=y.cycleLimit;
        check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        const auto captured=after.texts.text(text);check(captured->iconsCaptured && !glyphSnapshot->iconsCaptured && !after.sprites.outstanding());
        check(captured->icons.size()==((seed&12)?0:1));
        if(!captured->icons.empty()){
            const auto &icon=captured->icons[0];check(icon.colour==0x40808080 && icon.depth==0xff0004);
            const int32_t left=27988+seed*2,top=29848+seed*4;
            check(icon.fixedCorners==std::array<std::array<int32_t,2>,4>{{{left,top},{left+128,top},{left,top+288},{left+128,top+288}}});
            check(icon.uv==std::array<float,4>{0.25f,0.5f,0.75f,1.f});const auto vertices=vp2s::nativeUiSpriteVertices(icon);
            check(vertices[0].x==float(left)/16 && vertices[0].y==float(top)/16 && vertices[4].u==0.75f && vertices[4].v==1.f);
        }
        after.texts.release(text);check(captured->iconsCaptured);
        if(!captured->icons.empty()){
            const auto preserved=captured->icons[0].fixedCorners;
            std::memset(b.memory.flat()+item,0xee,0x30);check(captured->icons[0].fixedCorners==preserved);
            std::memcpy(b.memory.flat()+item,a.memory.flat()+item,0x30);
        }
        // Original reset invalidates association; retained native pictures survive.
        x={};x.r[4].ud[0]=sprite;x.r[31].ud[0]=0x170000;y=x;run(old,x,0x2ec320);run(native,y,0x2ec320);x.cycleLimit=y.cycleLimit;
        check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes) && !after.sprites.button(sprite));
        check(picture->rgba[0]==(0x80402000u|(seed<0x26?0x41u:0x3du)));
    }
    auto &rt=native.runtime();constexpr uint32_t text=0x1f0000,stack=0x1fff000;
    auto rejects=[](auto action){bool rejected=false;try{action();}catch(const std::exception &){rejected=true;}check(rejected);};
    after.sprites.beginText(stack,text);check(after.sprites.buildingText(text));
    rejects([&]{after.sprites.beginText(stack,text);});
    rejects([&]{after.sprites.finishText(stack,text+4);});
    put(rt,sprite+4,dtt);put(rt,sprite+8,dtt+0x14);put(rt,sprite+0x56,uint16_t(3));
    after.sprites.bindButton(&rt,after.textures,sprite,0x41);
    after.sprites.beginSprite(&rt,stack,sprite,0xff0000);
    rejects([&]{after.sprites.reset(sprite);});
    rejects([&]{after.sprites.finishText(stack,text);});
    rejects([&]{after.sprites.corner(stack-0xd0,1,100,100,{0,0,1,1});});
    rejects([&]{after.sprites.finishSprite(stack-0xd0);});
    after.sprites.colour(stack-0xd0,0x80808080);
    rejects([&]{after.sprites.colour(stack-0xd0,0);});
    for(unsigned i=0;i<4;++i)after.sprites.corner(stack-0xd0,i,100+i,200+i,{0,0,1,1});
    after.sprites.finishSprite(stack-0xd0);auto retained=after.sprites.finishText(stack,text);after.sprites.reset(sprite);
    check(retained.size()==1 && retained[0].picture && !after.sprites.outstanding() && !after.sprites.buildingText(text));
    std::puts("64 full button-sheet/reset plus64 original text-icon/scalar-sprite CPU/RAM cases: real lookup resume, both atlases, enable/reveal, colour/depth/UV/positions and immutable lifetime passed");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

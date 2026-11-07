#include "native_ui_text.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "ui_text_original_functions.h"
#include "ui_text_adapted_functions.h"
#include <cstring>
#include <cstdio>
#include <source_location>
#include <stdexcept>
namespace {
void check(bool ok,std::source_location loc=std::source_location::current()){
    if(!ok)throw std::runtime_error("native UI text assertion at "+std::to_string(loc.line()));
}
template<class T>void put(Runtime &rt,uint32_t address,T v){std::memcpy(rt.memory.hostPointer(address,sizeof(T)),&v,sizeof(T));}
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::UiTextOwner,vp2s::AssetOwner {
    struct Pause{};bool pause=false;std::vector<uint32_t> calls;
    vp2s::NativeUiTexts texts;vp2s::NativeGlyphs glyphs;vp2s::NativeAssets assets;
    explicit Host(const char *path):glyphs(path){}
    vp2s::NativeUiTexts &nativeUiTexts()override{return texts;}
    vp2s::NativeGlyphs &nativeGlyphs()override{return glyphs;}
    vp2s::NativeAssets &nativeAssets()override{return assets;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc)override{
        auto result=vp2s::nativeUiTextInstruction(c,rt,pc);check(result.has_value());return *result;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        calls.push_back(pc);auto &rt=d.runtime();check(c->r[4].uw[0]==0x190000);
        if(pc==0x170100)put(rt,0x1a0000,999.f);
        else if(pc==0x2e7e90)put(rt,c->r[5].uw[0],888.f);
        else if(pc!=0x170110 && pc!=0x2e76a0)throw std::runtime_error("unexpected UI fixture callback");
        c->r[2].ud[0]=pc;c->pc=c->r[31].uw[0];if(pause)throw Pause{};
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device write");}
    bool cop0Condition(bozo::ee::Dispatcher &,R5900Context *)override{throw std::runtime_error("device completion");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &why)override{throw std::runtime_error(why);}
};
void run(bozo::ee::Dispatcher &d,R5900Context &c){
    auto fn=d.runtime().functions.generated(0x2e74f0);unsigned attempts=0;
    while(true){try{fn(d.runtime().memory.flat(),&c,&d.runtime());if(c.pc==0x170000)break;}catch(const Host::Pause &){}
        check(++attempts<20);auto entry=d.runtime().functions.resumeEntry(c.pc);check(entry);fn=entry->resume;}
}
}
int main(int argc,char **argv){try{
    check(argc==2);Host before(argv[1]),after(argv[1]);bozo::ee::Dispatcher old(before),native(after);
    old.setFunctions(ui_text_original_function_table,ui_text_original_function_count,ui_text_original_resume_table,ui_text_original_resume_count);
    native.setFunctions(ui_text_adapted_function_table,ui_text_adapted_function_count,ui_text_adapted_resume_table,ui_text_adapted_resume_count);
    for(auto *d:{&old,&native}){d->boot(0x2e74f0,0x1fff000);for(uint32_t pc:{0x170100u,0x170110u,0x2e7e90u,0x2e76a0u})d->bindLibrary(pc);}
    old.runtime().port=&before;native.runtime().port=&after;
    for(unsigned seed=0;seed<64;++seed){
        auto &a=old.runtime(),&b=native.runtime();std::memset(a.memory.flat(),0,bozo::kFlatBytes);
        constexpr uint32_t object=0x190000,items=0x1a0000,font=0x1b0000,source=0x180000;
        put(a,object,0x15a450u);put(a,0x15a478,0x170100u);put(a,0x15a47c,0x2e7e90u);put(a,0x15a480,0x2e76a0u);put(a,0x15a484,0x170110u);
        put(a,font,source);put(a,font+0x10,source+0x80);put(a,font+0x14,2u);put(a,font+0x18,2u);put(a,font+0x20,3u);put(a,font+0x24,3u);put(a,font+0x28,2u);put(a,font+0x2c,1u);
        const int32_t count=seed&16?-1:int32_t(seed%6);unsigned expected=0;bool images=false;
        for(unsigned i=0;i<5;++i){const uint32_t item=items+i*0x30;
            for(unsigned j=0;j<4;++j)put(a,item+j*4,float(seed+i+j)/8);
            put(a,item+0x10,1.f);put(a,item+0x14,0.5f);put(a,item+0x18,font);put(a,item+0x1c,3u);
            put(a,item+0x20,0x80203040u+seed);put(a,item+0x24,seed%3);put(a,item+0x28,0x40010203u);
            const uint16_t code=(seed&4 && i==1)?0x4010:(seed&8 && i==2)?0x4000:0x21+i;
            put(a,item+0x2c,code);put(a,item+0x2e,uint16_t(i&1));
            if(int32_t(i)<count){if(code==0x4010)images=true;else if(!(code&0x4000))++expected;}
        }
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        after.assets={};after.assets.discRead(0x170000,1,0x21551000/2048);after.assets.begin(1,0x170000,source,0);after.assets.finish(1,true,0x90);
        R5900Context x{},y{};x.r[4].ud[0]=object;x.r[5].ud[0]=items;x.r[6].sd[0]=count;
        x.r[29].ud[0]=seed&2?0x70003f00:0x1fff000;x.r[31].ud[0]=0x170000;y=x;
        before.calls.clear();after.calls.clear();after.pause=seed&32;run(old,x);run(native,y);x.cycleLimit=y.cycleLimit;
        check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes) && before.calls==after.calls);
        auto held=after.texts.text(object);check(held && held->glyphs.size()==expected && held->inlineImages==images && !after.texts.outstanding());
        if(expected){check(held->glyphs[0].box[0]==999.f && held->glyphs[0].scale[0]==1.f && held->glyphs[0].picture->rgba[0]==0x20000011);}
        std::memset(b.memory.flat()+items,0xee,5*0x30);after.assets.overwrite(source,0x90);
        if(expected)check(held->glyphs[0].box[0]==999.f && held->glyphs[0].picture->rgba[0]==0x20000011);
        R5900Context dying{};dying.pc=0x2f0de0;dying.r[4].ud[0]=object;const auto saved=dying;
        check(vp2s::nativeUiTextInstruction(&dying,&b,0x2f0de0)==false && !after.texts.text(object));
        check(!std::memcmp(&dying,&saved,sizeof(dying)) && held->glyphs.size()==expected);
    }
    auto rejects=[](auto action){bool rejected=false;try{action();}catch(const std::exception &){rejected=true;}check(rejected);};
    auto &rt=native.runtime();constexpr uint32_t object=0x190000,items=0x1a0000,stack=0x1ff0000;
    after.texts.enter(&rt,stack,object,items,1);
    rejects([&]{after.texts.enter(&rt,stack,object,items,1);});
    rejects([&]{after.texts.finish(stack,object);});
    rejects([&]{after.texts.release(object);});
    rejects([&]{after.texts.activate(stack,object,items+0x30,1);});
    after.texts.activate(stack,object,items,1);
    rejects([&]{after.texts.activate(stack,object,items,1);});
    rejects([&]{after.texts.capture(&rt,after.glyphs,after.assets,stack,object,items+0x30,1,true);});
    put(rt,items+0x2c,uint16_t(0x4010));
    after.texts.capture(&rt,after.glyphs,after.assets,stack,object,items,0,true);
    rejects([&]{after.texts.capture(&rt,after.glyphs,after.assets,stack,object,items,0,true);});
    after.texts.finish(stack,object);auto held=after.texts.text(object);
    after.texts.enter(&rt,stack,object,0,0);after.texts.activate(stack,object,0,0);after.texts.finish(stack,object);
    check(held->inlineImages && !after.texts.text(object)->inlineImages && !after.texts.outstanding());
    after.texts.release(object);put(rt,object,0x15a690u);
    rejects([&]{after.texts.enter(&rt,stack,object,0,0);});
    check(!after.texts.outstanding());
    std::puts("64 full text-emitter CPU/RAM/callback cases: empty/control/inline/glyph items, setup mutation, nested resume, source release and immutable text lifetime; no native layer draw yet");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

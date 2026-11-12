#include "native_ui_text.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "ui_text_original_functions.h"
#include "ui_text_adapted_functions.h"
#include "ui_glyph_original_functions.h"
#include "ui_glyph_adapted_functions.h"
#include "ui_layer_original_functions.h"
#include "ui_layer_adapted_functions.h"
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
    bool referenceLayer=false,failDraw=false;std::vector<vp2s::NativeUiLayer> submissions;
    vp2s::NativeUiTexts texts;vp2s::NativeGlyphs glyphs;vp2s::NativeAssets assets;
    explicit Host(const char *path):glyphs(path){}
    vp2s::NativeUiTexts &nativeUiTexts()override{return texts;}
    vp2s::NativeGlyphs &nativeGlyphs()override{return glyphs;}
    vp2s::NativeAssets &nativeAssets()override{return assets;}
    void drawNativeUiLayer(const vp2s::NativeUiLayer &layer)override{
        if(failDraw)throw std::runtime_error("test draw rejection");submissions.push_back(layer);
    }
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(pc==0x2ec9e4 && referenceLayer){drawNativeUiLayer(texts.layer(rt,c->r[4].uw[0]));return false;}
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
void run(bozo::ee::Dispatcher &d,R5900Context &c,uint32_t entry=0x2e74f0){
    auto fn=d.runtime().functions.generated(entry);unsigned attempts=0;
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
        R5900Context dying{};dying.pc=0x2f0de0;dying.r[4].ud[0]=object;
        if(seed&1){
            put(b,object,0x15bd70u);dying.r[31].uw[0]=0x373ee0;
            auto invalid=dying;invalid.r[31].uw[0]+=4;bool rejected=false;
            try{vp2s::nativeUiTextInstruction(&invalid,&b,0x2f0de0);}catch(const std::exception &){rejected=true;}
            check(rejected && after.texts.text(object)==held);
        }
        const auto saved=dying;
        check(vp2s::nativeUiTextInstruction(&dying,&b,0x2f0de0)==false && !after.texts.text(object));
        check(!std::memcmp(&dying,&saved,sizeof(dying)) && held->glyphs.size()==expected);
    }
    auto rejects=[](auto action){bool rejected=false;try{action();}catch(const std::exception &){rejected=true;}check(rejected);};
    auto &rt=native.runtime();constexpr uint32_t object=0x190000,items=0x1a0000,stack=0x1ff0000;
    put(rt,object,0x15a450u);
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
    const auto geometry=(std::filesystem::path(argv[1])/"geometry").string();Host glyphBefore(geometry.c_str()),glyphAfter(geometry.c_str());
    bozo::ee::Dispatcher originalGlyph(glyphBefore),nativeGlyph(glyphAfter);
    originalGlyph.setFunctions(ui_glyph_original_function_table,ui_glyph_original_function_count,ui_glyph_original_resume_table,ui_glyph_original_resume_count);
    nativeGlyph.setFunctions(ui_glyph_adapted_function_table,ui_glyph_adapted_function_count,ui_glyph_adapted_resume_table,ui_glyph_adapted_resume_count);
    for(auto *d:{&originalGlyph,&nativeGlyph})d->boot(0x2e7e90,0x1fff000);
    originalGlyph.runtime().port=&glyphBefore;nativeGlyph.runtime().port=&glyphAfter;
    Host layerBefore(geometry.c_str()),layerAfter(geometry.c_str());layerBefore.referenceLayer=true;
    bozo::ee::Dispatcher originalLayer(layerBefore),nativeLayer(layerAfter);
    originalLayer.setFunctions(ui_layer_original_function_table,ui_layer_original_function_count,ui_layer_original_resume_table,ui_layer_original_resume_count);
    nativeLayer.setFunctions(ui_layer_adapted_function_table,ui_layer_adapted_function_count,ui_layer_adapted_resume_table,ui_layer_adapted_resume_count);
    for(auto *d:{&originalLayer,&nativeLayer})d->boot(0x2ec9c0,0x1fff000);
    originalLayer.runtime().port=&layerBefore;nativeLayer.runtime().port=&layerAfter;
    for(unsigned seed=0;seed<64;++seed){
        auto &a=originalGlyph.runtime(),&b=nativeGlyph.runtime();std::memset(a.memory.flat(),0,bozo::kFlatBytes);
        constexpr uint32_t layer=0x1b1000,font=0x1b0000,source=0x180000,manager=0x1e0000,glyphStack=0x1fff000;
        put(a,object,0x15a450u);put(a,object+0x14,layer);put(a,object+0x18,2.25f);put(a,object+0x1c,3.5f);
        put(a,object+0x78,1.f);put(a,object+0x7c,2.f);put(a,object+0x28,64.f);put(a,object+0x90,64.f);
        put(a,object+0x2c,10u);put(a,object+0x58,0x1c0000u);put(a,object+0x60,0x1d0000u);put(a,object+0x5c,0x1d0000u);
        put(a,layer+0x40,10.f);put(a,layer+0x44,20.f);put(a,layer+0xe0,128.f);put(a,layer+0xc4,0xffff00u);
        put(a,0x35e170,manager);put(a,manager+0x840,16.f);put(a,manager+0x844,8.f);
        put(a,manager+0x848,27648.f);put(a,manager+0x84c,29184.f);put(a,manager+0x850,16u);put(a,manager+0x854,32u);
        put(a,font,source);put(a,font+0x10,source+0x80);put(a,font+0x14,4u);put(a,font+0x18,2u);
        put(a,font+0x1c,6u);put(a,font+0x20,18u);put(a,font+0x24,6u);put(a,font+0x28,6u);put(a,font+0x2c,1u);
        put(a,items,float(seed)/8);put(a,items+4,float(seed)/4);put(a,items+8,float(seed)/8+3);put(a,items+12,float(seed)/4+4);
        put(a,items+0x10,0.5f);put(a,items+0x14,1.f);put(a,items+0x18,font);put(a,items+0x1c,3u);
        put(a,items+0x20,0x80203040u);put(a,items+0x24,seed%3);put(a,items+0x28,0x80102030u);put(a,items+0x2c,uint16_t(0x21));put(a,items+0x2e,uint16_t(seed&1));
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        glyphAfter.assets={};glyphAfter.assets.discRead(0x170000,1,0x21551000/2048);glyphAfter.assets.begin(1,0x170000,source,0);glyphAfter.assets.finish(1,true,0xb0);
        auto &texts=glyphAfter.texts;texts.enter(&b,glyphStack,object,items,1);texts.activate(glyphStack,object,items,1);
        texts.capture(&b,glyphAfter.glyphs,glyphAfter.assets,glyphStack,object,items,0,false);
        R5900Context x{},y{};x.r[4].ud[0]=object;x.r[5].ud[0]=items;x.r[29].ud[0]=glyphStack;x.r[31].ud[0]=0x170000;y=x;
        run(originalGlyph,x,0x2e7e90);run(nativeGlyph,y,0x2e7e90);x.cycleLimit=y.cycleLimit;
        check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        texts.finish(glyphStack,object);auto held=texts.text(object);const auto &g=held->glyphs.at(0);const auto &p=g.placement;
        if(!g.placed || p.fixedBox!=std::array<int32_t,4>{27820+int(seed)*2,29348+int(seed)*2,27884+int(seed)*2,29396+int(seed)*2})
            std::fprintf(stderr,"seed=%u placed=%u box=%d,%d,%d,%d\n",seed,g.placed,p.fixedBox[0],p.fixedBox[1],p.fixedBox[2],p.fixedBox[3]);
        check(g.placed && p.fixedBox==std::array<int32_t,4>{27820+int(seed)*2,29348+int(seed)*2,27884+int(seed)*2,29396+int(seed)*2});
        if(p.depth!=0xffff0a || p.colour!=0x40101820 || p.shadowColour!=(seed%3?0x40081018u:0u))
            std::fprintf(stderr,"seed=%u depth=%x colour=%08x shadow=%08x\n",seed,p.depth,p.colour,p.shadowColour);
        check(p.depth==0xffff0a && p.colour==0x40101820 && p.shadowColour==(seed%3?0x40081018u:0u));
        check(p.fixedShadowOffset==std::array<int32_t,2>{seed%3==1?16:0,seed%3==1?32:0});
        const auto vertices=vp2s::nativeUiGlyphVertices(g,false);
        check(vertices[0].x==float(27820+seed*2)/16 && vertices[0].y==float(29348+seed*2)/16 && vertices[4].z==float(0xffff0a));
        check(vertices[0].u==0.5f/8 && vertices[0].v==0.5f/8 && vertices[4].u==4.5f/8 && vertices[4].v==5.5f/8);
        if(g.shadow){const auto shadow=vp2s::nativeUiGlyphVertices(g,true);check(shadow[0].z==float(0xffff09) && shadow[0].rgba==p.shadowColour && shadow[0].x==vertices[0].x+(seed%3==1?1.f:0.f));}
        std::memset(b.memory.flat()+items,0xee,0x30);check(vp2s::nativeUiGlyphVertices(g,false)[0].x==vertices[0].x);
        constexpr uint32_t current=0x200000,secondary=0x210000,inactive=0x1b2000;
        const bool story=seed&32;
        put(b,layer,0x152060u);put(b,layer+0x10,story?0x15bca0u:0x15ba00u);put(b,layer+0x20,story?0x15bcbcu:0x15ba1cu);put(b,layer+0xb0,story?0x15bd38u:0x15ba98u);
        put(b,layer+0x83,uint8_t(1));put(b,layer+0x85,uint8_t(1));put(b,layer+0xca,uint8_t(2));put(b,layer+8,object);
        put(b,object+8,inactive);put(b,inactive+8,layer);put(b,inactive+0x14,layer);put(b,inactive,0x15a690u);
        put(b,object+0x39,uint8_t(!(seed&4)));put(b,object+0x3b,uint8_t(!(seed&8)));put(b,object+0x3d,uint8_t(!(seed&16)));
        put(b,0x35ddf8,current);put(b,current+0x40ac,layer+0x20);put(b,current+0x40dd,uint8_t(1));put(b,current+0x4088,seed&1?secondary:0u);
        auto &lr=originalLayer.runtime(),&ln=nativeLayer.runtime();
        std::memcpy(lr.memory.flat(),b.memory.flat(),bozo::kFlatBytes);std::memcpy(ln.memory.flat(),b.memory.flat(),bozo::kFlatBytes);
        layerBefore.texts=texts;layerAfter.texts=texts;layerBefore.submissions.clear();layerAfter.submissions.clear();
        R5900Context u{},v{};u.r[4].ud[0]=layer;u.r[29].ud[0]=glyphStack;u.r[31].ud[0]=0x170000;v=u;
        run(originalLayer,u,0x2ec9c0);run(nativeLayer,v,0x2ec9c0);u.cycleLimit=v.cycleLimit;
        check(!std::memcmp(&u,&v,sizeof(u)) && !std::memcmp(lr.memory.flat(),ln.memory.flat(),bozo::kFlatBytes));
        const size_t visible=(seed&28)?0:1;
        check(layerBefore.submissions.size()==1 && layerAfter.submissions.size()==1 && layerAfter.submissions[0].texts.size()==visible);
        check(*ln.memory.hostPointer(layer+0x84,1)==0 && *ln.memory.hostPointer(layer+0x82,1)==1 && *ln.memory.hostPointer(current+0x40dd,1)==0);
        if(seed&1)check(*ln.memory.hostPointer(secondary+0x3e6a,1)==1);
        const auto submitted=layerAfter.submissions[0];
        texts.release(object);layerAfter.texts.release(object);
        if(visible)check(submitted.texts.at(0)->glyphs.at(0).placement.colour==0x40101820);
    }
    auto &layerRam=nativeLayer.runtime();constexpr uint32_t layer=0x1b1000,current=0x200000,inactive=0x1b2000;
    layerAfter.texts=layerBefore.texts;put(layerRam,current+0x40dd,uint8_t(1));
    for(auto at:{0x39u,0x3bu,0x3du})put(layerRam,object+at,uint8_t(1));
    R5900Context rejected{};rejected.pc=0x2ec9c0;rejected.r[4].ud[0]=layer;rejected.r[5].ud[0]=layer+0xb0;
    const auto unchanged=rejected;std::vector<uint8_t> ramBefore(layerRam.memory.flat(),layerRam.memory.flat()+bozo::kFlatBytes);
    layerAfter.failDraw=true;
    rejects([&]{vp2s::nativeUiTextInstruction(&rejected,&layerRam,0x2ec9e4);});
    check(!std::memcmp(&rejected,&unchanged,sizeof(rejected)) && !std::memcmp(ramBefore.data(),layerRam.memory.flat(),ramBefore.size()));
    layerAfter.failDraw=false;
    put(layerRam,layer+0x10,0x15ba00u); // A mixed base/derived tuple is invalid.
    rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,layer+0x10,0x15bca0u);
    {   // mod003 event layer 167850: admitted by its vtables and thunk words.
        const auto types=[&](uint32_t l,uint32_t o,uint32_t j){put(layerRam,layer+0x10,l);put(layerRam,layer+0x20,o);put(layerRam,layer+0xb0,j);};
        put(layerRam,0x16786cu+0x18,0x3b3150u);put(layerRam,0x1678e8u+0x0c,0x3b31c0u);put(layerRam,0x1678e8u+0x14,0x2eca00u);put(layerRam,0x16786cu+0x10,0x466ef0u);
        put(layerRam,0x3b3150u,0x08000000u|(0x2ec9c0u>>2));put(layerRam,0x3b3154u,0x2484ffe0u);put(layerRam,0x3b31c0u,0x08000000u|(0x2ec9a0u>>2));put(layerRam,0x3b31c4u,0x2484ff50u);
        put(layerRam,0x466ef0u,0x08000000u|(0x466c10u>>2));put(layerRam,0x466ef4u,0x2484ffe0u);put(layerRam,0x466c20u,0x0c000000u|(0x2ef0f0u>>2));
        types(0x167850u,0x16786cu,0x1678e8u);layerAfter.texts.layer(&layerRam,layer);
        put(layerRam,0x1678e8u+0x14,0x447680u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x1678e8u+0x14,0x2eca00u);   // another completion
        put(layerRam,0x466c20u,0u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x466c20u,0x0c000000u|(0x2ef0f0u>>2));    // teardown not the base
        put(layerRam,0x3b3154u,0x2484ffd0u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x3b3154u,0x2484ffe0u);            // another adjustment
        types(0x15bca0u,0x15bcbcu,0x15bd38u);}
    for(auto at:{0x39u,0x3bu,0x3du})put(layerRam,inactive+at,uint8_t(1));
    rejects([&]{layerAfter.texts.layer(&layerRam,layer);});
    put(layerRam,inactive+0x3b,uint8_t(0));put(layerRam,inactive+8,object);
    rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,inactive+8,layer);
    put(layerRam,current+0x40ac,layer+0x40);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,current+0x40ac,layer+0x20);
    put(layerRam,inactive+0x14,layer+0x10);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,inactive+0x14,layer);
    layerAfter.texts.release(object);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});
    std::puts("64 full text-emitter,64 original scalar glyph and64 native-contract layer/real-completion CPU/RAM cases; immutable lifetime, fixed-point geometry, colour/alpha, shadow, half-texel UV, visibility and scene completion passed");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

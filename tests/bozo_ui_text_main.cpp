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
#include <bit>
#include <cmath>
#include "bozo/ops_fpu.h"
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
    after.texts.release(object);
    // 15a600 (15a2f0 with layer request hooks): the same emitter lifetime; another class rejects.
    put(rt,object,0x15a600u);after.texts.enter(&rt,stack,object,0,0);after.texts.activate(stack,object,0,0);after.texts.finish(stack,object);
    check(after.texts.text(object)!=nullptr);after.texts.release(object);
    // 16ed70 (mod028's battle copy of 15a600): the same lifetime.
    put(rt,object,0x16ed70u);after.texts.enter(&rt,stack,object,0,0);after.texts.activate(stack,object,0,0);after.texts.finish(stack,object);
    check(after.texts.text(object)!=nullptr);after.texts.release(object);
    put(rt,object,0x15a610u);rejects([&]{after.texts.enter(&rt,stack,object,0,0);});
    {   // 15a350 (sibling text): same emitter lifetime; released at its own destructor 2eeb80.
        put(rt,object,0x15a350u);after.texts.enter(&rt,stack,object,0,0);after.texts.activate(stack,object,0,0);after.texts.finish(stack,object);
        check(after.texts.text(object)!=nullptr);
        R5900Context dying{};dying.pc=0x2eeb80;dying.r[4].ud[0]=object;
        put(rt,object,0x15a450u);rejects([&]{vp2s::nativeUiTextInstruction(&dying,&rt,0x2eeb80);});check(after.texts.text(object)!=nullptr);
        put(rt,object,0x15a350u);auto early=dying;early.pc=0x2eeb84;rejects([&]{vp2s::nativeUiTextInstruction(&early,&rt,0x2eeb80);});
        check(vp2s::nativeUiTextInstruction(&dying,&rt,0x2eeb80)==false && !after.texts.text(object));
        R5900Context none{};none.pc=0x2eeb80;check(vp2s::nativeUiTextInstruction(&none,&rt,0x2eeb80)==false);
    }
    put(rt,object,0x15a690u);
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
        if(seed==0){   // Battle text 1748d0 (mod028): 4c8fb0's upload of the item's font cell and its own sprite, read back.
            constexpr uint32_t battle=0x194000,battleItems=0x1a4000,uploads=0x1c4000,spritesAt=0x1d4000,battleStack=0x1ffe000;
            std::memcpy(b.memory.flat()+battleItems,b.memory.flat()+items,0x30);put(b,battleItems+0x2e,uint16_t(1));put(b,battle,0x1748d0u);
            put(b,battle+0x58,uploads|0x30000000u);put(b,battle+0x60,spritesAt|0x30000000u);   // the game's uncached aliases
            texts.enter(&b,battleStack,battle,battleItems,1);texts.activate(battleStack,battle,battleItems,1);
            rejects([&]{texts.battleGlyph(&b,glyphAfter.glyphs,glyphAfter.assets,battle,spritesAt);});      // a commit without its item
            texts.beginBattleGlyph(&b,battleStack,battle,battleItems,0);
            const uint64_t block[16]={0x10000006,0,0x1000000000000004,0xe,0x2410000000000000,0x50,uint64_t(0x20)<<32,0x51,uint64_t(6)<<32|6,0x52,0,0x53,
                0x0800000000008002,0,uint64_t(source+0x80+18)<<32|0x30000002,0};   // glyph 1: font +10 + 1 * stride 18
            std::memcpy(b.memory.flat()+uploads,block,sizeof(block));put(b,battle+0x58,(uploads+0x80)|0x30000000u);
            const uint64_t sprite[8]={0x6400000000008001,0x535317,uint64_t(0x24)<<20|uint64_t(1)<<56,0x55808080,
                uint64_t(0x20*16+8)|uint64_t(8)<<16,0x7000|uint64_t(0x7100)<<16|uint64_t(0xfff4dd)<<32,
                uint64_t(0x20*16+88)|uint64_t(88)<<16,0x7100|uint64_t(0x7200)<<16|uint64_t(0xfff4dd)<<32};
            std::memcpy(b.memory.flat()+spritesAt,sprite,sizeof(sprite));
            texts.battleGlyph(&b,glyphAfter.glyphs,glyphAfter.assets,battle,(spritesAt+0x40)|0x30000000u);texts.finish(battleStack,battle);
            const auto bt=texts.text(battle);check(bt && bt->glyphs.empty() && bt->icons.size()==1);const auto &icon=bt->icons[0];
            check(icon.colour==0x55808080u && icon.depth==0xfff4dd && icon.fixedCorners[3]==std::array<int32_t,2>{0x7100,0x7200});
            check(icon.uv==std::array<float,4>{0.5f/8,0.5f/8,5.5f/8,5.5f/8} && icon.picture==held->glyphs[0].font->picture(1,1));
            // Another glyph's address is not the item's cell.
            put(b,battle+0x58,uploads|0x30000000u);put(b,battle+0x60,spritesAt|0x30000000u);
            texts.enter(&b,battleStack,battle,battleItems,1);texts.activate(battleStack,battle,battleItems,1);texts.beginBattleGlyph(&b,battleStack,battle,battleItems,0);
            put(b,battle+0x58,(uploads+0x80)|0x30000000u);
            put(b,uploads+0x74,source+0x80);rejects([&]{texts.battleGlyph(&b,glyphAfter.glyphs,glyphAfter.assets,battle,(spritesAt+0x40)|0x30000000u);});
            texts.finish(battleStack,battle);texts.release(battle);check(!texts.outstanding());
        }
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
        {   // The field pause layer 166fe0: the event layer's job entries, base teardown.
            // Layers are recognised by these entries (every layer/object/job slot
            // but the destructors), not by their type addresses: the camp menu's
            // four layer types carry the same set.
            const auto entries=[&](uint32_t l,uint32_t o,uint32_t j){
                for(auto [at,v]:{std::pair{0xcu,0x2c00a0u},{0x10u,0x2c0070u},{0x14u,0x2c0040u},{0x18u,0x2bffe0u}})put(layerRam,l+at,v);
                for(auto [at,v]:{std::pair{0xcu,0x35edd0u},{0x14u,0x35ee90u},{0x1cu,0x374990u},{0x18u,0x3b3150u},{0x10u,0x2ef0f0u}})put(layerRam,o+at,v);
                for(auto [at,v]:{std::pair{0x8u,0x3748e0u},{0x18u,0x2ec9c0u},{0x1cu,0x2ec9a0u},{0xcu,0x3b31c0u},{0x10u,0x3b31d0u},{0x14u,0x2eca00u}})put(layerRam,j+at,v);};
            // The helpers' code (mod003 35edd0 returns 3, 35ee90/3748e0 empty, 374990 the colour setter).
            const auto words=[&](uint32_t at,std::initializer_list<uint32_t> w){for(auto x:w){put(layerRam,at,x);at+=4;}};
            const auto helpers=[&](uint32_t three,uint32_t none,uint32_t colour,uint32_t jobNone){
                words(three,{0x03e00008u,0x24020003u});words(none,{0x03e00008u,0u});words(jobNone,{0x03e00008u,0u});
                words(colour,{0x24030001u,0xa0830050u,0xe48c0020u,0x3c033f80u,0xe48d0024u,0xe48e0028u,0x03e00008u,0xac83002cu});};
            helpers(0x35edd0u,0x35ee90u,0x374990u,0x3748e0u);
            entries(0x166fe0u,0x166ffcu,0x167078u);entries(0x1682f0u,0x16830cu,0x168388u);
            put(layerRam,0x3b31d0u,0x08000000u|(0x2eca00u>>2));put(layerRam,0x3b31d4u,0x2484ffe0u);
            types(0x166fe0u,0x166ffcu,0x167078u);layerAfter.texts.layer(&layerRam,layer);
            put(layerRam,0x167078u+0x14,0x44a540u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x167078u+0x14,0x2eca00u);   // another completion
            put(layerRam,0x166ffcu+0x10,0x466ef0u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x166ffcu+0x10,0x2ef0f0u);   // another teardown
            put(layerRam,0x3b31d4u,0x2484ffd0u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x3b31d4u,0x2484ffe0u);            // another adjustment
            types(0x166fe0u,0x16786cu,0x167078u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});                                          // mixed tuple
            entries(0x166f20u,0x166f3cu,0x166fb8u);
            types(0x166f20u,0x166f3cu,0x166fb8u);layerAfter.texts.layer(&layerRam,layer);                                                       // the sibling
            types(0x1682f0u,0x16830cu,0x168388u);layerAfter.texts.layer(&layerRam,layer);                                                       // a camp menu layer
            put(layerRam,0x1682f0u+0x10,0x2c0071u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x1682f0u+0x10,0x2c0070u); // another layer entry
            put(layerRam,0x168388u+0x1c,0u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x168388u+0x1c,0x2ec9a0u);       // another job entry
            // The town exit prompt 15fc60: the same entries but a producer 3afca0 (object +18
            // thunk 3b31b0) that calls FlushCache(0) and then the base producer 2ec9c0.
            entries(0x15fc60u,0x15fc7cu,0x15fcf8u);put(layerRam,0x15fc7cu+0x18,0x3b31b0u);put(layerRam,0x15fcf8u+0x18,0x3afca0u);
            put(layerRam,0x3b31b0u,0x080ebf28u);put(layerRam,0x3b31b4u,0x2484ffe0u);
            const auto producer=[&](uint32_t at){words(at,{0x27bdffe0u,0xffbf0010u,0x7fb00000u,0x0080802du,0x0c04dd18u,0x0000202du,0x0c0bb270u,0x0200202du});};
            producer(0x3afca0u);
            types(0x15fc60u,0x15fc7cu,0x15fcf8u);layerAfter.texts.layer(&layerRam,layer);
            for(auto [at,bad]:{std::pair{0x3afcb0u,0x0c04dd1cu},{0x3afcb4u,0x0000282du},{0x3afcb8u,0x0c0bb268u},{0x3b31b0u,0x080ebf2cu}}){
                uint32_t good;std::memcpy(&good,layerRam.memory.hostPointer(at,4),4);
                put(layerRam,at,bad);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,at,good);}                       // another call, argument, producer or thunk
            put(layerRam,0x15fcf8u+0x18,0x2ec9c0u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x15fcf8u+0x18,0x3afca0u); // half the pair
            for(auto [at,bad]:{std::pair{0x3afca0u,0x27bdffd0u},{0x3afcacu,0x0080282du},{0x3afcbcu,0x0200282du}}){
                uint32_t good;std::memcpy(&good,layerRam.memory.hostPointer(at,4),4);
                put(layerRam,at,bad);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,at,good);}                       // another frame or argument move
            // Map 446's layer 15fd30: a second copy of that producer, 3b2fd0 behind thunk 3b3180, admitted by its words.
            entries(0x15fd30u,0x15fd4cu,0x15fdc8u);put(layerRam,0x15fd4cu+0x18,0x3b3180u);put(layerRam,0x15fdc8u+0x18,0x3b2fd0u);
            put(layerRam,0x3b3180u,0x080ecbf4u);put(layerRam,0x3b3184u,0x2484ffe0u);producer(0x3b2fd0u);
            types(0x15fd30u,0x15fd4cu,0x15fdc8u);layerAfter.texts.layer(&layerRam,layer);
            put(layerRam,0x3b2fe8u,0x0c0bb268u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x3b2fe8u,0x0c0bb270u);     // another base producer
            // ... and its own deferred completion 467f30 (job +10 = thunk 3b3190): the base call first, admitted by its prologue.
            put(layerRam,0x15fdc8u+0x10,0x3b3190u);put(layerRam,0x15fdc8u+0x14,0x467f30u);put(layerRam,0x3b3190u,0x08119fccu);put(layerRam,0x3b3194u,0x2484ffe0u);
            words(0x467f30u,{0x27bdffe0u,0xffbf0010u,0x7fb00000u,0x0c0bb280u,0x0080802du});layerAfter.texts.layer(&layerRam,layer);
            put(layerRam,0x467f3cu,0x0c0bb284u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x467f3cu,0x0c0bb280u);     // another first call
            // Maps 329-433's layer 168010: the pause's entries with the bracketed completion 44a540 (job +10 = thunk 44a710),
            // whose base call is checked.
            entries(0x168010u,0x16802cu,0x1680a8u);put(layerRam,0x1680a8u+0x10,0x44a710u);put(layerRam,0x1680a8u+0x14,0x44a540u);
            put(layerRam,0x44a710u,0x08112950u);put(layerRam,0x44a714u,0x2484ffe0u);put(layerRam,0x44a58cu,0x0c000000u|(0x2eca00u>>2));
            types(0x168010u,0x16802cu,0x1680a8u);layerAfter.texts.layer(&layerRam,layer);
            put(layerRam,0x44a58cu,0u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x44a58cu,0x0c000000u|(0x2eca00u>>2));   // no base call
            put(layerRam,0x44a714u,0x2484ff50u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x44a714u,0x2484ffe0u);       // another adjustment
            // Map 2's overlay layer 168b70: object +10 = thunk 490ee0 to a wrapper that only calls the base teardown 2ef0f0.
            entries(0x168b70u,0x168b8cu,0x168c08u);put(layerRam,0x168b8cu+0x10,0x490ee0u);put(layerRam,0x490ee0u,0x08124160u);put(layerRam,0x490ee4u,0x2484ffe0u);
            words(0x490580u,{0x27bdfff0u,0xffbf0000u,0x0c0bbc3cu,0x24840020u,0xdfbf0000u,0x03e00008u});
            types(0x168b70u,0x168b8cu,0x168c08u);layerAfter.texts.layer(&layerRam,layer);
            put(layerRam,0x49058cu,0x24840010u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x49058cu,0x24840020u);       // another object
            // mod003 layer 15fef0 (map 79): these entries with the mod001 completion 2f2390
            // (base 2eca00 first, then deferred actions; job +10 = thunk 3b3130).
            entries(0x15fef0u,0x15ff0cu,0x15ff88u);put(layerRam,0x15ff88u+0x10,0x3b3130u);put(layerRam,0x15ff88u+0x14,0x2f2390u);
            put(layerRam,0x3b3130u,0x08000000u|(0x2f2390u>>2));put(layerRam,0x3b3134u,0x2484ffe0u);
            words(0x2f2390u,{0x27bdffe0u,0xffbf0010u,0x7fb00000u,0x0c0bb280u,0x0080802du});
            types(0x15fef0u,0x15ff0cu,0x15ff88u);layerAfter.texts.layer(&layerRam,layer);
            for(auto [at,bad]:{std::pair{0x2f239cu,0x0c0bb284u},{0x3b3130u,0x08000000u|(0x2eca00u>>2)},{0x15ff88u+0x14,0x2f2394u}}){
                uint32_t good;std::memcpy(&good,layerRam.memory.hostPointer(at,4),4);
                put(layerRam,at,bad);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,at,good);}                       // no base call, another thunk or completion
            // mod028's battle layer 16ec80 (16ec9c/16ed18): the same entries with its own copy of
            // that completion, 3830d0, behind thunk 44d570.
            entries(0x16ec80u,0x16ec9cu,0x16ed18u);put(layerRam,0x16ed18u+0x10,0x44d570u);put(layerRam,0x16ed18u+0x14,0x3830d0u);
            words(0x44d570u,{0x08000000u|(0x3830d0u>>2),0x2484ffe0u});
            words(0x3830d0u,{0x27bdffe0u,0xffbf0010u,0x7fb00000u,0x0c0bb280u,0x0080802du});
            types(0x16ec80u,0x16ec9cu,0x16ed18u);layerAfter.texts.layer(&layerRam,layer);
            put(layerRam,0x3830dcu,0u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x3830dcu,0x0c0bb280u);              // no base call
            // mod010's layer 1693d0: its own copies of the helpers (494c10, 494c60, 494d40, 494c90)
            // and thunks (495140 -> 2ec9c0, 495150 -> 2ec9a0, 495160 -> 2eca00), admitted by their code.
            {constexpr uint32_t l=0x1693d0u,o=0x1693ecu,j=0x169468u;
                for(auto [at,v]:{std::pair{0xcu,0x2c00a0u},{0x10u,0x2c0070u},{0x14u,0x2c0040u},{0x18u,0x2bffe0u}})put(layerRam,l+at,v);
                for(auto [at,v]:{std::pair{0xcu,0x494c10u},{0x14u,0x494c60u},{0x1cu,0x494d40u},{0x18u,0x495140u},{0x10u,0x2ef0f0u}})put(layerRam,o+at,v);
                for(auto [at,v]:{std::pair{0x8u,0x494c90u},{0x18u,0x2ec9c0u},{0x1cu,0x2ec9a0u},{0xcu,0x495150u},{0x10u,0x495160u},{0x14u,0x2eca00u}})put(layerRam,j+at,v);
                helpers(0x494c10u,0x494c60u,0x494d40u,0x494c90u);
                words(0x495140u,{0x08000000u|(0x2ec9c0u>>2),0x2484ffe0u});words(0x495150u,{0x08000000u|(0x2ec9a0u>>2),0x2484ff50u});
                words(0x495160u,{0x08000000u|(0x2eca00u>>2),0x2484ffe0u});
                types(l,o,j);layerAfter.texts.layer(&layerRam,layer);
                for(auto [at,bad]:{std::pair{0x494c14u,0x24020004u},{0x494d48u,0u},{0x494c94u,0x24020001u},{0x495140u,0x08000000u|(0x2ec9a0u>>2)},{0x495154u,0x2484ffe0u}}){
                    uint32_t good;std::memcpy(&good,layerRam.memory.hostPointer(at,4),4);
                    put(layerRam,at,bad);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,at,good);}                // another helper or thunk
            }
            types(0x166fe0u,0x166ffcu,0x167078u);
        }
        // mod003 layer 165d30: base teardown, own bracketing completion 44a540.
        put(layerRam,0x165d4cu+0x18,0x3b3150u);put(layerRam,0x165dc8u+0x0c,0x3b31c0u);put(layerRam,0x165dc8u+0x14,0x44a540u);put(layerRam,0x165d4cu+0x10,0x2ef0f0u);
        put(layerRam,0x44a58cu,0x0c000000u|(0x2eca00u>>2));
        types(0x165d30u,0x165d4cu,0x165dc8u);layerAfter.texts.layer(&layerRam,layer);
        {   // The sibling 165c60 with the same entries.
            put(layerRam,0x165c7cu+0x18,0x3b3150u);put(layerRam,0x165cf8u+0x0c,0x3b31c0u);put(layerRam,0x165cf8u+0x14,0x44a540u);put(layerRam,0x165c7cu+0x10,0x2ef0f0u);
            types(0x165c60u,0x165c7cu,0x165cf8u);layerAfter.texts.layer(&layerRam,layer);
            put(layerRam,0x165cf8u+0x14,0x2eca00u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x165cf8u+0x14,0x44a540u);
            types(0x165c60u,0x165d4cu,0x165dc8u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});   // mixed tuple
            types(0x165d30u,0x165d4cu,0x165dc8u);}
        put(layerRam,0x44a58cu,0u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x44a58cu,0x0c000000u|(0x2eca00u>>2));       // completion skips the base
        put(layerRam,0x165d4cu+0x10,0x466ef0u);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,0x165d4cu+0x10,0x2ef0f0u); // another teardown
        types(0x15bca0u,0x15bcbcu,0x15bd38u);}
    for(auto at:{0x39u,0x3bu,0x3du})put(layerRam,inactive+at,uint8_t(1));
    rejects([&]{layerAfter.texts.layer(&layerRam,layer);});
    put(layerRam,inactive+0x3b,uint8_t(0));put(layerRam,inactive+8,object);
    rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,inactive+8,layer);
    put(layerRam,current+0x40ac,layer+0x40);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,current+0x40ac,layer+0x20);
    put(layerRam,inactive+0x14,layer+0x10);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});put(layerRam,inactive+0x14,layer);
    layerAfter.texts.release(object);rejects([&]{layerAfter.texts.layer(&layerRam,layer);});
    {   // Event name plate (465150 path A) against the route's built packet (element 13268dc):
        // layer +40 -266, +44 379, +27c 240, +2a4 75.0, frame 512x512, half-width flag set.
        namespace F=bozo::ps2f;uint32_t fl=0;const auto W=[](float v){return std::bit_cast<uint32_t>(v);};
        vp2s::NativeNamePlateInputs in;
        in.x=F::add(F::fromInt(1792),F::div(F::mul(W(512.f),W(-266.f),fl),W(640.f),fl),fl);
        in.y=F::add(F::fromInt(1792),F::div(F::mul(0x44003d70u,W(379.f),fl),W(480.f),fl),fl);
        in.length=F::div(F::mul(W(512.f),F::fromInt(240),fl),W(640.f),fl);
        in.colour=0x80808080u;in.depth=0xfffeceu;in.tailOffset=W(75.f);in.narrowCap=true;
        in.rects={std::array<uint16_t,6>{0x0008,0x0db8,0x0e88,0x0f98,0x00e9,0x001f},{0x0008,0x0db8,0x0e88,0x0fc8,0x00e9,0x0022},{0x0e98,0x0db8,0x0fd8,0x0fc8,0x0015,0x0022}};
        const auto plate=vp2s::nativeNamePlate(in,nullptr);
        // XYZ2 (12.4) and UV of the four sprites, as the PS2 packet holds them.
        const int32_t xyz[4][4]={{25267,35088,28595,35584},{28595,35088,28809,35632},{25053,35584,28381,36128},{28381,35584,28595,36128}};
        const int32_t uvs[4][4]={{0x0008,0x0db8,0x0e88,0x0f98},{0x0e98,0x0db8,0x0fd8,0x0fc8},{0x0008,0x0db8,0x0e88,0x0fc8},{0x0e98,0x0db8,0x0fd8,0x0fc8}};
        check(plate.icons.size()==4);
        for(unsigned k=0;k<4;++k){const auto &s=plate.icons[k];
            check(s.fixedCorners[0][0]==xyz[k][0] && s.fixedCorners[0][1]==xyz[k][1] && s.fixedCorners[3][0]==xyz[k][2] && s.fixedCorners[3][1]==xyz[k][3]);
            check(s.fixedCorners[1][0]==xyz[k][2] && s.fixedCorners[1][1]==xyz[k][1] && s.fixedCorners[2][0]==xyz[k][0] && s.fixedCorners[2][1]==xyz[k][3]);
            for(unsigned i=0;i<4;++i)check(s.uv[i]==float(uvs[k][i])/4096.f);
            check(s.colour==0x80808080u && s.depth==0xfffeceu);}
        // The strips: dark (1654,2227)-(1774,2229) then light (1652,2225)-(1772,2227).
        check(plate.alphaTriangles.size()==12);
        const auto &a=plate.alphaTriangles;
        check(a[0].x==1654 && a[0].y==2227 && a[4].x==1774 && a[4].y==2229 && a[0].rgba==0x80010101u);
        check(a[6].x==1652 && a[6].y==2225 && a[10].x==1772 && a[10].y==2227 && a[6].rgba==0x80f8f8f8u);
        check(plate.lateIcons.empty());
        // Path B (465b38): X 100, Y 50, L 20 (exact floats), rect +140 h 0x22, rect +150 h 0x22 -> cap
        // 0.85*34 = 28.9 (x0.75 = 21.675). Rows 16*46 .. 16*80; body [X, X+L], cap after it;
        // mirrored: body reversed and the cap before X. No strips.
        {auto b=in;b.pathB=true;b.x=W(100.f);b.y=W(50.f);b.length=W(20.f);
            for(const bool mirror:{false,true}){b.mirror=mirror;const auto p=vp2s::nativeNamePlate(b,nullptr);
                check(p.icons.size()==2 && p.alphaTriangles.empty() && p.lateIcons.empty());
                const auto &body=p.icons[0],&cap=p.icons[1];
                check(body.fixedCorners[0][1]==736 && body.fixedCorners[3][1]==1280 && cap.fixedCorners[0][1]==736 && cap.fixedCorners[3][1]==1280);
                const int32_t near=mirror?1920:1600,far=mirror?1600:1920,capEnd=mirror?int32_t(16*(100-21.675f)):int32_t(16*(120+21.675f));
                check(body.fixedCorners[0][0]==near && body.fixedCorners[3][0]==far && cap.fixedCorners[0][0]==far && std::abs(cap.fixedCorners[3][0]-capEnd)<=1);
                check(body.uv[0]==float(0x0008)/4096.f && body.uv[3]==float(0x0fc8)/4096.f && cap.uv[0]==float(0x0e98)/4096.f);}
            // Mirrored tail: w 32 x0.75 = 24, h 32, s0 20: x (120 - 40) .. 104, y (50 + 10 - 16) .. +32.
            b.mirror=true;b.icon=true;b.textSize=20;b.alpha=W(128.f);b.alphaScale=W(1.f);b.iconRect={0x0008,0x0008,0x0208,0x0208,32,32};
            b.iconLog2Width=7;b.iconLog2Height=7;const auto t=vp2s::nativeNamePlate(b,nullptr,nullptr);
            check(t.icons.size()==2 && t.lateIcons.size()==1);const auto &s=t.lateIcons[0];
            check(s.fixedCorners[0][0]==16*80 && s.fixedCorners[3][0]==16*104 && s.fixedCorners[0][1]==16*44 && s.fixedCorners[3][1]==16*76);
        }
        // The uploader-image tail: 16+X .. +0.85*w (x0.75 with the flag), y from Y + s0/2 - 0.85*h/2 - 8, height h.
        {auto tail=in;tail.icon=true;tail.textSize=41;tail.alpha=W(128.f);tail.alphaScale=W(0.5f);
            tail.iconRect={0x0008,0x0008,0x0208,0x0208,32,32};tail.iconLog2Width=7;tail.iconLog2Height=7;
            const auto t=vp2s::nativeNamePlate(tail,nullptr,nullptr);check(t.lateIcons.size()==1);const auto &s=t.lateIcons[0];
            const auto X=std::bit_cast<float>(in.x),Y=std::bit_cast<float>(in.y);
            check(s.colour==0x40808080u && s.depth==0xfffeceu);
            check(std::abs(s.fixedCorners[0][0]/16.f-(16+X))<0.07f && std::abs((s.fixedCorners[3][0]-s.fixedCorners[0][0])/16.f-0.85f*32*0.75f)<0.13f);
            check(std::abs(s.fixedCorners[0][1]/16.f-(Y+20-0.85f*16-8))<0.13f && std::abs((s.fixedCorners[3][1]-s.fixedCorners[0][1])/16.f-32)<0.13f);
            check(s.uv[0]==4.f/2048 && s.uv[3]==260.f/2048);}   // 410c30 scales table texels by (1<<7)/256
        {   // Event title ornament (449580) against the route's built packet (element 58dd00):
            // left = 1792 + 512*244.5/640, right 2108.375, top = 1792 + 512.96*63.6307/480 (1860).
            vp2s::NativeOrnamentInputs o;
            o.left=F::add(F::fromInt(1792),F::div(F::mul(W(512.f),W(244.5f),fl),W(640.f),fl),fl);o.right=W(2108.375f);o.top=W(1860.f);
            o.colour=0x0c808080u;o.depth=0xffff32u;
            o.rects={std::array<uint16_t,6>{0x0598,0x0008,0x0908,0x0208,0x0038,0x0021},{0x02b8,0x0008,0x0578,0x0208,0x002d,0x0021}};
            const auto t=vp2s::nativeOrnament(o,nullptr);check(t.icons.size()==4);
            // XYZ2 (12.4) x0,x1 and UV u0,u1 of the four sprites as packed: y 1854..1892, v 0.5..32.5.
            const int32_t xs[4][2]={{31801,32767},{32767,33734},{31081,31801},{33734,34454}};
            const int32_t us[4][2]={{0x0598,0x0908},{0x0908,0x0598},{0x02b8,0x0578},{0x0578,0x02b8}};
            for(unsigned k=0;k<4;++k){const auto &s=t.icons[k];
                check(s.fixedCorners[0][0]==xs[k][0] && s.fixedCorners[3][0]==xs[k][1] && s.fixedCorners[0][1]==1854*16 && s.fixedCorners[3][1]==1892*16);
                check(s.uv[0]==float(us[k][0])/4096.f && s.uv[2]==float(us[k][1])/4096.f && s.uv[1]==8.f/4096 && s.uv[3]==520.f/4096);
                check(s.colour==0x0c808080u && s.depth==0xffff32u);}
        }
        {   // Town prompt bar (4489b0) against the route's built packet (element 58cf40):
            // x = -16+542, y = -8+394 scaled; cap 0.75*33 (flag set); right edge 2304.
            namespace G=bozo::ps2f;uint32_t g=0;const auto A=[&](uint32_t a,uint32_t b){return G::add(a,b,g);};
            const auto Sb=[&](uint32_t a,uint32_t b){return G::sub(a,b,g);};const auto Mu=[&](uint32_t a,uint32_t b){return G::mul(a,b,g);};
            const auto Di=[&](uint32_t a,uint32_t b){return G::div(a,b,g);};
            vp2s::NativeBarInputs b;
            const auto x=A(W(-16.f),W(542.f)),y=A(W(-8.f),W(394.f));
            b.x1=A(W(6.f),A(G::fromInt(1792),Sb(Di(Mu(W(512.f),x),W(640.f)),W(2.f))));
            b.y0=A(W(4.f),A(G::fromInt(1792),Sb(Di(Mu(0x44003d70u,y),W(480.f)),W(2.f))));
            b.y1=A(W(4.f),A(b.y0,G::fromInt(0x21)));b.cap=Mu(W(0.75f),G::fromInt(0x21));b.right=G::fromInt(2304);
            b.colour=0x0c808080u;b.depth=0xffff32u;
            b.rects={std::array<uint16_t,6>{0x0998,0x0458,0x0cd8,0x0658,0x0035,0x0021},{0x0ce8,0x0458,0x0fe8,0x0658,0x0031,0x0021}};
            const auto t=vp2s::nativeBar(b,nullptr);check(t.icons.size()==2);
            const int32_t xs[2][2]={{35072,35468},{35468,36864}};const int32_t us[2][2]={{0x0998,0x0cd8},{0x0ce8,0x0fe8}};
            for(unsigned k=0;k<2;++k){const auto &s=t.icons[k];
                check(s.fixedCorners[0][0]==xs[k][0] && s.fixedCorners[3][0]==xs[k][1] && s.fixedCorners[0][1]==35304 && s.fixedCorners[3][1]==35896);
                check(s.uv[0]==float(us[k][0])/4096.f && s.uv[2]==float(us[k][1])/4096.f && s.uv[1]==float(0x0458)/4096.f && s.uv[3]==float(0x0658)/4096.f);
                check(s.colour==0x0c808080u && s.depth==0xffff32u);}
        }
        {   // Field pause logo (4514f0): layer at (0, 0), rect +30 40x20 at u 0x100..0x300.
            vp2s::NativePauseLogoInputs l;l.x=W(512.f*73.f/640.f+1792.f);l.y=W(512.96f*79.f/480.f+1792.f);l.colour=0x40808080u;l.depth=0xffff10u;
            l.rect={0x100,0x200,0x300,0x400,40,20};
            const auto t=vp2s::nativePauseLogo(l,nullptr);check(t.icons.size()==1 && t.alphaTriangles.empty());const auto &s=t.icons[0];
            const float x=std::bit_cast<float>(l.x),y=std::bit_cast<float>(l.y);
            check(s.fixedCorners[0][0]==int32_t(16*x) && s.fixedCorners[0][1]==int32_t(16*y) && s.fixedCorners[3][0]==int32_t(16*(x+40)) && s.fixedCorners[3][1]==int32_t(16*(y+20)));
            check(s.uv[0]==0x100/4096.f && s.uv[1]==0x200/4096.f && s.uv[2]==0x300/4096.f && s.uv[3]==0x400/4096.f && s.colour==0x40808080u && s.depth==0xffff10u);
        }
        {   // Countdown timer HUD (168e10, mod008 48fea0): a REGLIST sprite from its stored words over a 256 x 64 texture
            // (UV 10.4: U 0x400 = texel 64 = 0.25; XYZ2 12.4 with Z above), and its six-vertex Gouraud strip.
            const uint64_t uv0=0x0100u|(uint64_t(0x0200u)<<16),uv1=0x0400u|(uint64_t(0x0300u)<<16);
            const uint64_t xyz0=0x7100u|(uint64_t(0x7d00u)<<16)|(uint64_t(0xffff40u)<<32),xyz1=0x7300u|(uint64_t(0x7e00u)<<16)|(uint64_t(0xffff40u)<<32);
            const auto s=vp2s::nativeTimerSprite(0x7f808080u,uv0,xyz0,uv1,xyz1,256,64);
            check(s.fixedCorners[0][0]==0x7100 && s.fixedCorners[0][1]==0x7d00 && s.fixedCorners[1][0]==0x7300 && s.fixedCorners[2][1]==0x7e00 && s.fixedCorners[3][0]==0x7300 && s.fixedCorners[3][1]==0x7e00);
            check(s.uv[0]==0x100/4096.f && s.uv[1]==0x200/1024.f && s.uv[2]==0x400/4096.f && s.uv[3]==0x300/1024.f && s.colour==0x7f808080u && s.depth==0xffff40u);
            const auto strip=vp2s::nativeTimerStrip({0x7f2b1243u,0x7f2b51bdu,0x7f2b1243u},{0x7000u|(uint64_t(0x7f00u)<<16),0x7000u|(uint64_t(0x7f40u)<<16),0x7a00u|(uint64_t(0x7f00u)<<16),
                0x7a00u|(uint64_t(0x7f40u)<<16),0x8400u|(uint64_t(0x7f00u)<<16),0x8400u|(uint64_t(0x7f40u)<<16)});
            check(strip.size()==12 && strip[0].x==0x700 && strip[0].y==0x7f0 && strip[0].rgba==0x7f2b1243u && strip[2].rgba==0x7f2b51bdu && strip[11].x==0x840 && strip[11].rgba==0x7f2b1243u);
        }
        {   // Field pause bar (451a60): x 0, y 384 (scaled), origin (1792, 1792).
            vp2s::NativePauseBarInputs b;b.x=W(0.f);b.y=W(384.f);b.depth=0xffff20u;b.originX=1792;b.originY=1792;
            const auto t=vp2s::nativePauseBar(b);check(t.icons.empty() && t.alphaTriangles.size()==6+12);
            const auto &v=t.alphaTriangles;
            check(v[0].x==1792 && v[0].y==2176 && v[4].x==2304 && v[4].y==2304 && v[0].rgba==0x80000000u && v[0].z==float(0xffff20u));
            // Strip D(1792,2176) D(1792,2180) L(1952,..) L D(2688 = 512 + Y, the game's own end) D at depth ffffaa.
            check(v[6].x==1792 && v[6].y==2176 && v[7].y==2180 && v[8].x==1952 && v[8].rgba==0x802b51bdu && v[6].rgba==0x802b1243u && v[6].z==float(0xffffaa));
            check(v[15].x==1952 && v[16].x==2688 && v[16].y==2176 && v[17].x==2688 && v[17].y==2180 && v[17].rgba==0x802b1243u);
            // The rectangle takes int(y); the strip keeps the fraction in 1/16 pixels.
            b.y=W(384.5f);const auto h=vp2s::nativePauseBar(b).alphaTriangles;check(h[0].y==2176 && h[2].y==2304 && h[6].y==2176.5f && h[7].y==2180.5f);
        }
        // Without the half-width flag the cap is 0.85*w wide.
        in.narrowCap=false;const auto wide=vp2s::nativeNamePlate(in,nullptr);
        check(wide.icons[1].fixedCorners[3][0]-wide.icons[1].fixedCorners[0][0]>plate.icons[1].fixedCorners[3][0]-plate.icons[1].fixedCorners[0][0]);
    }
    std::puts("64 full text-emitter,64 original scalar glyph and64 native-contract layer/real-completion CPU/RAM cases; immutable lifetime, fixed-point geometry, colour/alpha, shadow, half-texel UV, visibility and scene completion passed");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

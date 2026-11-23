#include "native_cast.h"
#include "native_conditional_sites.h"
#include "native_ui_sprites.h"
#include "native_ui_text.h"
#include "native_ui_windows.h"
#include "native_assets.h"
#include "bozo/ee/dispatcher.h"
#include <format>
#include <cstring>
#include <stdexcept>
#include <bit>
namespace vp2s {
namespace {
template<class T>T value(Runtime *rt,uint32_t at){
    if(!rt || at<0x100000 || at>0x2000000-sizeof(T))throw std::runtime_error(std::format("native sprite metadata outside EE RAM: address={:x} size={}",at,sizeof(T)));
    T v;std::memcpy(&v,rt->memory.hostPointer(at,sizeof(T)),sizeof(T));return v;
}
}
void NativeUiSprites::bindButton(Runtime *rt,NativeTextures &textures,uint32_t sprite,int32_t button){
    if(button<0 || button>255)throw std::runtime_error("native button identifier outside admitted range");
    const auto manager=value<uint32_t>(rt,0x35e170);
    if(value<uint32_t>(rt,manager)!=0x15a1e0)throw std::runtime_error("native button has no typed UI manager");
    // These are the constructor's actual image-object slots, selected by the
    // original button-id branch. No TEX0 bits or physical addresses are read.
    const auto image=value<uint32_t>(rt,manager+(button<0x26?0x828:0x82c));
    if(value<uint32_t>(rt,image)!=0x15a6f0 || value<uint32_t>(rt,image+0x90)!=0x15a76c)
        throw std::runtime_error("native button sheet has no typed image object");
    const auto fis=value<uint32_t>(rt,image+0xc0);
    const TextureKey key{value<uint32_t>(rt,fis+0x14),value<uint32_t>(rt,fis+0x18)};
    if(value<uint32_t>(rt,fis)!=0x534946 || key!=TextureKey{0x00494853u,button<0x26?0x41u:0x3du})
        throw std::runtime_error("native button image identity differs from UI setup");
    const auto dtt=value<uint32_t>(rt,sprite+4),parts=value<uint32_t>(rt,sprite+8);
    const auto count=value<uint16_t>(rt,sprite+0x56);
    if(dtt<0x100000 || dtt>0x2000000-0x14 || parts!=dtt+0x14 || count!=value<uint16_t>(rt,dtt+0x10) ||
       !count || uint64_t(parts)+uint64_t(count)*16>0x2000000)
        throw std::runtime_error("native button DTT parts escaped the original setup");
    auto result=std::make_shared<NativeButtonSheet>(NativeButtonSheet{sprite,dtt,parts,count,uint32_t(button),textures.load(key)});
    sheets[sprite]=std::move(result);
}
void NativeUiSprites::serialize(bozo::state::Archive &a,NativeTextures &textures){
    if(outstanding())throw std::runtime_error("native UI sprite state saved with a build in flight");
    state::map(a,sheets,[&](std::shared_ptr<const NativeButtonSheet> &p){
        NativeButtonSheet s=p?*p:NativeButtonSheet{};
        bozo::state::fields(a,s.sprite,s.dtt,s.parts,s.count,s.button);stateAsset(a,textures,s.asset);
        if(a.loading())p=std::make_shared<NativeButtonSheet>(std::move(s));});
}
std::shared_ptr<const NativeButtonSheet> NativeUiSprites::button(uint32_t sprite)const{
    auto it=sheets.find(sprite);return it==sheets.end()?nullptr:it->second;
}
void NativeUiSprites::reset(uint32_t sprite){
    for(const auto &[stack,build]:spriteBuilds)if(build.sprite.sprite==sprite)throw std::runtime_error("native button reset during scalar capture");
    sheets.erase(sprite);
}
bool NativeUiSprites::buildingText(uint32_t element)const{
    for(const auto &[stack,build]:textBuilds)if(build.element==element)return true;return false;
}
void NativeUiSprites::beginText(uint32_t stack,uint32_t element){
    if(textBuilds.contains(stack))throw std::runtime_error("native text-icon invocation repeats");
    textBuilds.emplace(stack,TextBuild{element});
}
std::vector<NativeUiSprite> NativeUiSprites::finishText(uint32_t stack,uint32_t element){
    for(const auto &[at,build]:spriteBuilds)if(build.textStack==stack)throw std::runtime_error("native icon builder returned with unfinished sprites");
    auto it=textBuilds.find(stack);if(it==textBuilds.end() || it->second.element!=element)throw std::runtime_error("native text-icon return escaped its invocation");
    auto result=std::move(it->second.sprites);textBuilds.erase(it);return result;
}
void NativeUiSprites::beginSprite(Runtime *rt,uint32_t stack,uint32_t sprite,uint32_t depth,bool additive){
    if(stack<0xd0 || spriteBuilds.contains(stack-0xd0) || !textBuilds.contains(stack))throw std::runtime_error("native button draw has no original text invocation");
    const auto sheet=button(sprite);
    if(!sheet || value<uint32_t>(rt,sprite+4)!=sheet->dtt || value<uint32_t>(rt,sprite+8)!=sheet->parts ||
       value<int16_t>(rt,sprite+0x54)<0 || value<uint16_t>(rt,sprite+0x54)>=sheet->count || depth>0xffffff || (value<uint32_t>(rt,sprite+0x3c)&0x7fffffff))
        throw std::runtime_error("unported native button parts/depth/rotation");
    const auto part=sheet->parts+value<uint16_t>(rt,sprite+0x54)*16;
    const auto selected=value<int8_t>(rt,sprite+0x64);const auto csa=selected<0?value<uint8_t>(rt,part+12):uint32_t(selected);
    auto picture=sheet->asset->picture(csa);
    spriteBuilds.emplace(stack-0xd0,SpriteBuild{stack,NativeUiSprite{.sprite=sprite,.depth=depth,.picture=std::move(picture)}});
    spriteBuilds.at(stack-0xd0).sprite.additive=additive;
}
void NativeUiSprites::colour(uint32_t stack,uint32_t colour){
    auto &build=spriteBuilds.at(stack);if(build.coloured)throw std::runtime_error("native button colour repeats");
    build.sprite.colour=colour;build.coloured=true;
}
void NativeUiSprites::corner(uint32_t stack,unsigned index,int32_t x,int32_t y,const std::array<float,4> &uv){
    auto &build=spriteBuilds.at(stack);
    if(index>3 || build.mask!=(1u<<index)-1 || x<0 || x>65535 || y<0 || y>65535)throw std::runtime_error("native button corner escaped its scalar routine");
    if(!index){for(float f:uv)if((std::bit_cast<uint32_t>(f)&0x7f800000)==0x7f800000)throw std::runtime_error("nonfinite native button UV");build.sprite.uv=uv;}
    build.sprite.fixedCorners[index]={x,y};build.mask|=1u<<index;
}
void NativeUiSprites::finishSprite(uint32_t stack){
    auto it=spriteBuilds.find(stack);if(it==spriteBuilds.end() || it->second.mask!=15 || !it->second.coloured)throw std::runtime_error("native button draw has unfinished scalar outputs");
    auto &text=textBuilds.at(it->second.textStack);if(text.sprites.size()>=8192)throw std::runtime_error("native button count outside admitted range");
    text.sprites.push_back(std::move(it->second.sprite));spriteBuilds.erase(it);
}
// The tutorial window's button element: 15a4e0, or mod028's battle copy 16edd0 (its +20 build is
// the base 2f1230; its +1c only gates the base update 2f1310 on the battle's pause flags).
bool nativeButtonElementClass(Runtime *rt,uint32_t type){
    return type==0x15a4e0 || (type==0x16edd0 && value<uint32_t>(rt,0x16edf0)==0x2f1230);
}
std::optional<bool> nativeUiSpriteInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(!nativeUiSpriteSite(pc))return std::nullopt;   // its sites: native_conditional_sites.h
    if(pc!=0x2eab28 && pc!=0x2eaa20 && pc!=0x462a60 && pc!=0x2f1050 && pc!=0x2f12cc && pc!=0x2f11c0 && pc!=0x3835e0 && pc!=0x47a1c0 && pc!=0x2ec308 && pc!=0x2ec320 && pc!=0x2efe40 && pc!=0x2effe8 && pc!=0x2eb3a0 && pc!=0x2eb5c4 && pc!=0x2ebac4 && pc!=0x2ebb0c && pc!=0x2ebb58 && pc!=0x2ebb9c && pc!=0x2ebba8 && pc!=0x2f2df0 && pc!=0x2f2fb8 && pc!=0x2f2d70 && pc!=0x310480 && pc!=0x310584 && pc!=0x3102f0)return std::nullopt;
    auto *d=rt?vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services):nullptr;
    auto *owner=d?vp2s::nativeCast<UiSpriteOwner>(&d->host()):nullptr;
    auto *textures=d?vp2s::nativeCast<TextureOwner>(&d->host()):nullptr;
    if(!c || !owner || !textures)throw std::runtime_error("native button sheet requires image/sprite owners");
    auto &sprites=owner->nativeUiSprites();const auto sp=c->r[29].uw[0];
    if(pc==0x2f2df0 || pc==0x2f2fb8 || pc==0x2f2d70){
        auto *texts=vp2s::nativeCast<UiTextOwner>(&d->host());if(!texts)throw std::runtime_error("native image element has no layer owner");
        const auto element=c->r[pc==0x2f2fb8?18:4].uw[0];
        if(pc==0x2f2d70 && !element)return false;
        if(value<uint32_t>(rt,element)!=0x15a690)throw std::runtime_error("unported native image element class");
        if(pc==0x2f2df0){
            if(c->pc!=pc || sp<0x50 || (sp&15))throw std::runtime_error("native image element requires complete original builder");
            if(!value<uint8_t>(rt,element+0x3b))sprites.beginText(sp-0x50,element);
        }else if(pc==0x2f2fb8)texts->nativeUiTexts().spriteElement(element,sprites.finishText(sp,element));
        else{
            if(c->pc!=pc || sprites.buildingText(element))throw std::runtime_error("native image element destroyed during capture");
            sprites.reset(element+0x50);sprites.reset(element+0xb8);texts->nativeUiTexts().release(element);
        }
    }else if(pc==0x2efe40 || pc==0x2effe8){
        auto *texts=vp2s::nativeCast<UiTextOwner>(&d->host());if(!texts)throw std::runtime_error("native button capture has no text owner");
        const auto element=c->r[pc==0x2efe40?4:21].uw[0];
        const auto textClass=value<uint32_t>(rt,element);   // 15a450 and its subclasses (see native_ui_text.cpp)
        if(textClass==0x15a350 || !nativeUiTextClass(textClass) || !texts->nativeUiTexts().text(element))throw std::runtime_error("unported native button text class/lifetime");
        if(pc==0x2efe40){if(c->pc!=pc || sp<0xa0 || (sp&15))throw std::runtime_error("native text icons require original entry");sprites.beginText(sp-0xa0,element);}
        else texts->nativeUiTexts().icons(element,sprites.finishText(sp,element));
    }else if(pc==0x2eab28 || pc==0x2eaa20 || pc==0x462a60 || pc==0x2f1050 || pc==0x47a1c0 || pc==0x2f12cc || pc==0x2f11c0 || pc==0x3835e0){
        // mod028's battle button element 16edd0: destructor 3835e0 sets its class, then inlines
        // 2f11c0's body (another overlay's words there are not it).
        if(pc==0x3835e0 && (value<uint32_t>(rt,0x3835e0)!=0x27bdffd0u || value<uint32_t>(rt,0x38361c)!=0x0c0bbd34u))return std::nullopt;
        // Sprite element 15a200 (build 2eaa90 calls the sprite writer 2eb3a0
        // once with its sprite +50 and returns to 2eab28; destructor 2eaa20).
        auto *windows=vp2s::nativeCast<UiWindowOwner>(&d->host());if(!windows)throw std::runtime_error("native sprite element has no window owner");
        // Its mod003 subclass 168410 (destructor 462a60 inlines the base one),
        // and the FIS sprite element 168440 (base 15a4b0: build 2f0fa0 returns
        // from the writer to 2f1050; destructor 47a1c0).
        // The button sprite element 15a4e0 (build 2f1230 returns from the writer to
        // 2f12cc; destructor 2f11c0): the same capture and release.
        const bool done=pc==0x2eab28 || pc==0x2f1050 || pc==0x2f12cc;
        const auto element=c->r[done?16:4].uw[0];
        if(!done && !element)return false;
        const auto cls=value<uint32_t>(rt,element);
        const bool fisElement=pc==0x2f1050 || pc==0x47a1c0;
        const bool button=pc==0x2f12cc || pc==0x2f11c0 || pc==0x3835e0;
        const bool known=button?nativeButtonElementClass(rt,cls):fisElement?cls==0x168440:pc==0x462a60?cls==0x168410:(cls==0x15a200 || cls==0x168410);
        if(!known)
            throw std::runtime_error("native sprite element has another class");
        if(!done){if(c->pc!=pc || sprites.buildingText(element))throw std::runtime_error("native sprite element destroyed during capture");
            windows->nativeUiWindows().release(element);sprites.reset(element+0x50);}
        else windows->nativeUiWindows().spriteElement(element,sprites.finishText(sp,element));
    }else if(pc==0x310480 || pc==0x310584 || pc==0x3102f0){
        // Two-sprite element 15ac60 (the camp menu Items page's icons): build 310480 draws each of its sprites +50
        // and +b8 whose +66 is set, returning from the writer to 310548, when +11e or +b6 is set and it is not
        // built (+3b); it marks itself built at 310584. Destructor 3102f0 (its sprite array resets through 2ec320).
        auto *windows=vp2s::nativeCast<UiWindowOwner>(&d->host());if(!windows)throw std::runtime_error("native two-sprite element has no window owner");
        const auto element=c->r[pc==0x310584?20:4].uw[0];
        if(pc==0x3102f0 && !element)return false;
        if(value<uint32_t>(rt,element)!=0x15ac60)throw std::runtime_error("native two-sprite element has another class");
        if(pc==0x310480){
            if(c->pc!=pc || sp<0x80 || (sp&15))throw std::runtime_error("native two-sprite element requires complete original builder");
            if((value<uint8_t>(rt,element+0x11e) || value<uint8_t>(rt,element+0xb6)) && !value<uint8_t>(rt,element+0x3b))sprites.beginText(sp-0x80,element);
        }else if(pc==0x310584)windows->nativeUiWindows().spriteElements(element,sprites.finishText(sp,element));
        else{
            if(c->pc!=pc || sprites.buildingText(element))throw std::runtime_error("native two-sprite element destroyed during capture");
            windows->nativeUiWindows().release(element);
        }
    }else if(pc==0x2eb3a0){
        const auto caller=c->r[31].uw[0];const bool image=caller==0x2f2efc || caller==0x2f2f8c;
        if(caller==0x310548){
            // 15ac60's sprites (above): bound by 2ec200 to their own one-part DTT (+4) and texture record (+5a page,
            // +5c palette page, +5f format, +60/+61 log2 size); the picture is the camp menu image under that handle.
            const auto element=c->r[20].uw[0],sprite=c->r[4].uw[0],layer=value<uint32_t>(rt,element+0x14);
            if(c->pc!=pc || value<uint32_t>(rt,element)!=0x15ac60 || (sprite!=element+0x50 && sprite!=element+0xb8) ||
               value<uint32_t>(rt,layer)!=0x152060 || !sprites.buildingText(element))
                throw std::runtime_error("native two-sprite element's sprite escaped its builder");
            if(!sprites.button(sprite)){
                const auto dtt=value<uint32_t>(rt,sprite+4),parts=value<uint32_t>(rt,sprite+8),entries=uint32_t(value<uint16_t>(rt,sprite+0x56));
                if(value<uint32_t>(rt,dtt)!=0x545444 || parts!=dtt+0x14 || !entries || entries!=value<uint16_t>(rt,dtt+0x10))
                    throw std::runtime_error("native two-sprite element lost its DTT binding");
                const auto page=value<uint16_t>(rt,sprite+0x5a),palette=value<uint16_t>(rt,sprite+0x5c);
                const auto format=value<uint8_t>(rt,sprite+0x5f),tw=value<uint8_t>(rt,sprite+0x60),th=value<uint8_t>(rt,sprite+0x61);
                auto asset=textures->nativeTextures().menuPage(page,format,palette);
                if(!asset)throw std::runtime_error(std::format("native two-sprite element names no published menu image (page {:x} format {:x} palette {:x})",page,format,palette));
                if(tw>10 || th>10 || asset->width!=1u<<tw || asset->height!=1u<<th)throw std::runtime_error("native two-sprite element image size differs from its record");
                sprites.bindSheet(std::make_shared<NativeButtonSheet>(NativeButtonSheet{sprite,dtt,parts,entries,0,std::move(asset)}));
            }
        }
        if(caller==0x2f1050){
            // FIS sprite element 168440: 2ebbe0 bound its sprite to the element's
            // own FIS file (+b8, read through the tracked loader into +cc), with
            // the texture cache's bookkeeping and an upload packet in the
            // element's buffer that native drawing never submits. The picture
            // is that file under its key; the DTT (+bc) gives the parts.
            const auto element=c->r[16].uw[0],sprite=c->r[4].uw[0],fis=value<uint32_t>(rt,element+0xb8),dtt=value<uint32_t>(rt,element+0xbc);
            const auto parts=value<uint32_t>(rt,sprite+8),entries=uint32_t(value<uint16_t>(rt,sprite+0x56));
            if(c->pc!=pc || value<uint32_t>(rt,element)!=0x168440 || sprite!=element+0x50 || value<uint32_t>(rt,sprite)!=fis || value<uint32_t>(rt,fis)!=0x534946 ||
               value<uint32_t>(rt,sprite+4)!=dtt || value<uint32_t>(rt,dtt)!=0x545444 || parts!=dtt+0x14 || !entries || entries!=value<uint16_t>(rt,dtt+0x10))
                throw std::runtime_error("native FIS sprite element lost its file/DTT binding");
            auto *assets=vp2s::nativeCast<AssetOwner>(&d->host());
            if(!assets || !assets->nativeAssets().origin(fis))throw std::runtime_error("native FIS sprite element has no decoded file owner");
            auto asset=textures->nativeTextures().load({value<uint32_t>(rt,fis+0x14),value<uint32_t>(rt,fis+0x18)});
            const auto tw=value<uint8_t>(rt,sprite+0x60),th=value<uint8_t>(rt,sprite+0x61);
            if(tw>10 || th>10 || asset->width!=1u<<tw || asset->height!=1u<<th)throw std::runtime_error(std::format("native FIS sprite {}x{} differs from its record ({}, {})",asset->width,asset->height,tw,th));
            sprites.bindSheet(std::make_shared<NativeButtonSheet>(NativeButtonSheet{sprite,dtt,parts,entries,0,std::move(asset)}));
            sprites.beginText(sp,element);
        }
        if(caller==0x2eab28){
            // Sprite element 15a200 in a camp menu layer. With a DTT (+b8) its
            // sprite was bound by 2ec200 to the element's texture record +bc
            // (page, palette page, width, format, log2 size): the picture is the
            // camp menu image published under that handle. Without one it is a
            // button sheet bound at 2ec308.
            const auto element=c->r[16].uw[0],sprite=c->r[4].uw[0],layer=value<uint32_t>(rt,element+0x14);
            const auto family=value<uint32_t>(rt,layer+0x10);
            if(c->pc!=pc || (value<uint32_t>(rt,element)!=0x15a200 && value<uint32_t>(rt,element)!=0x168410) || sprite!=element+0x50 || value<uint32_t>(rt,layer)!=0x152060)
                throw std::runtime_error(std::format("unported native sprite element: element={:x} class={:x} layer type={:x}",element,value<uint32_t>(rt,element),family));
            if(const auto dtt=value<uint32_t>(rt,element+0xb8)){
                const auto parts=value<uint32_t>(rt,sprite+8),entries=uint32_t(value<uint16_t>(rt,sprite+0x56));
                if(value<uint32_t>(rt,sprite+4)!=dtt || value<uint32_t>(rt,dtt)!=0x545444 || parts!=dtt+0x14 || !entries || entries!=value<uint16_t>(rt,dtt+0x10))
                    throw std::runtime_error("native sprite element lost its DTT binding");
                const auto page=value<uint16_t>(rt,sprite+0x5a),palette=value<uint16_t>(rt,sprite+0x5c);
                const auto format=value<uint8_t>(rt,sprite+0x5f),tw=value<uint8_t>(rt,sprite+0x60),th=value<uint8_t>(rt,sprite+0x61);
                auto asset=textures->nativeTextures().menuPage(page,format,palette);
                if(!asset)throw std::runtime_error(std::format("native sprite element names no published menu image (page {:x} format {:x} palette {:x})",page,format,palette));
                if(tw>10 || th>10 || asset->width!=1u<<tw || asset->height!=1u<<th)throw std::runtime_error("native sprite element image size differs from its record");
                sprites.bindSheet(std::make_shared<NativeButtonSheet>(NativeButtonSheet{sprite,dtt,parts,entries,0,std::move(asset)}));
            }else if(!sprites.button(sprite))throw std::runtime_error("native sprite element has no bound sheet");
            sprites.beginText(sp,element);
        }
        if(caller==0x2f12cc){
            // Button sprite element 15a4e0 (the tutorial window's button prompt): shown
            // by +bc, its update 2f1310 binds sheet entry 0x25 at 2ec260 and animates
            // it (2ec090); the build passes the layer origin plus +18/+1c and colour 0.
            const auto element=c->r[16].uw[0],sprite=c->r[4].uw[0],layer=value<uint32_t>(rt,element+0x14);
            if(c->pc!=pc || !nativeButtonElementClass(rt,value<uint32_t>(rt,element)) || sprite!=element+0x50 || value<uint32_t>(rt,layer)!=0x152060)
                throw std::runtime_error(std::format("unported native button sprite element: element={:x} class={:x}",element,value<uint32_t>(rt,element)));
            if(!sprites.button(sprite))throw std::runtime_error("native button sprite element has no bound sheet");
            sprites.beginText(sp,element);
        }
        if(c->pc!=pc || (caller!=0x2eff74 && caller!=0x2eab28 && caller!=0x2f1050 && caller!=0x2f12cc && caller!=0x310548 && !image) || c->r[8].uw[0])throw std::runtime_error(std::format("unported native sprite caller/colour multiplier: caller={:x} sprite={:x} colour={:x} x={:x} y={:x} depth={:x}",caller,c->r[4].uw[0],c->r[8].uw[0],c->r[5].uw[0],c->r[6].uw[0],c->r[7].uw[0]));
        if(image){const auto element=c->r[18].uw[0];
            if(value<uint32_t>(rt,element)!=0x15a690 || c->r[4].uw[0]!=element+(caller==0x2f2efc?0xb8:0x50) || !sprites.buildingText(element))
                throw std::runtime_error("native image sprite escaped its original element builder");
            if(!sprites.button(c->r[4].uw[0]))throw std::runtime_error("native image sprite requires original typed atlas binding");
        }
        sprites.beginSprite(rt,sp,c->r[4].uw[0],c->r[7].uw[0],caller==0x2f2efc);
    }else if(pc==0x2eb5c4)sprites.colour(sp,c->r[3].uw[0]);
    else if(pc==0x2ebba8)sprites.finishSprite(sp);
    else if(pc==0x2ebac4 || pc==0x2ebb0c || pc==0x2ebb58 || pc==0x2ebb9c){
        const unsigned index=pc==0x2ebac4?0:pc==0x2ebb0c?1:pc==0x2ebb58?2:3;
        sprites.corner(sp,index,std::bit_cast<int32_t>(c->fu[1]),std::bit_cast<int32_t>(c->fu[0]),{c->f[6],c->f[5],c->f[4],c->f[2]});
    }else if(pc==0x2ec320){
        if(c->pc!=pc)throw std::runtime_error("native sprite reset requires original entry");
        sprites.reset(c->r[4].uw[0]);
    }else sprites.bindButton(rt,textures->nativeTextures(),c->r[16].uw[0],c->r[17].sw[0]);
    return false; // Original lookup, scalar assignment/reset and callbacks remain.
}
}

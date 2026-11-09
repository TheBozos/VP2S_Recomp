#include "native_ui_sprites.h"
#include "native_ui_text.h"
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
std::optional<bool> nativeUiSpriteInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x2ec308 && pc!=0x2ec320 && pc!=0x2efe40 && pc!=0x2effe8 && pc!=0x2eb3a0 && pc!=0x2eb5c4 && pc!=0x2ebac4 && pc!=0x2ebb0c && pc!=0x2ebb58 && pc!=0x2ebb9c && pc!=0x2ebba8 && pc!=0x2f2df0 && pc!=0x2f2fb8 && pc!=0x2f2d70)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<UiSpriteOwner *>(&d->host()):nullptr;
    auto *textures=d?dynamic_cast<TextureOwner *>(&d->host()):nullptr;
    if(!c || !owner || !textures)throw std::runtime_error("native button sheet requires image/sprite owners");
    auto &sprites=owner->nativeUiSprites();const auto sp=c->r[29].uw[0];
    if(pc==0x2f2df0 || pc==0x2f2fb8 || pc==0x2f2d70){
        auto *texts=dynamic_cast<UiTextOwner *>(&d->host());if(!texts)throw std::runtime_error("native image element has no layer owner");
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
        auto *texts=dynamic_cast<UiTextOwner *>(&d->host());if(!texts)throw std::runtime_error("native button capture has no text owner");
        const auto element=c->r[pc==0x2efe40?4:21].uw[0];
        const auto textClass=value<uint32_t>(rt,element);   // 165ae0: the 15a450 subclass (see native_ui_text.cpp)
        if((textClass!=0x15a450 && textClass!=0x165ae0) || !texts->nativeUiTexts().text(element))throw std::runtime_error("unported native button text class/lifetime");
        if(pc==0x2efe40){if(c->pc!=pc || sp<0xa0 || (sp&15))throw std::runtime_error("native text icons require original entry");sprites.beginText(sp-0xa0,element);}
        else texts->nativeUiTexts().icons(element,sprites.finishText(sp,element));
    }else if(pc==0x2eb3a0){
        const auto caller=c->r[31].uw[0];const bool image=caller==0x2f2efc || caller==0x2f2f8c;
        if(c->pc!=pc || (caller!=0x2eff74 && !image) || c->r[8].uw[0])throw std::runtime_error(std::format("unported native sprite caller/colour multiplier: caller={:x} sprite={:x} colour={:x} x={:x} y={:x} depth={:x}",caller,c->r[4].uw[0],c->r[8].uw[0],c->r[5].uw[0],c->r[6].uw[0],c->r[7].uw[0]));
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

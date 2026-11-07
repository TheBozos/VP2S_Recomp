#include "native_ui_sprites.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <stdexcept>
namespace vp2s {
namespace {
template<class T>T value(Runtime *rt,uint32_t at){
    if(!rt || at<0x100000 || at>0x2000000-sizeof(T))throw std::runtime_error("native button metadata outside EE RAM");
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
std::optional<bool> nativeUiSpriteInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x2ec308 && pc!=0x2ec320)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<UiSpriteOwner *>(&d->host()):nullptr;
    auto *textures=d?dynamic_cast<TextureOwner *>(&d->host()):nullptr;
    if(!c || !owner || !textures)throw std::runtime_error("native button sheet requires image/sprite owners");
    if(pc==0x2ec320){
        if(c->pc!=pc)throw std::runtime_error("native sprite reset requires original entry");
        owner->nativeUiSprites().reset(c->r[4].uw[0]);
    }else owner->nativeUiSprites().bindButton(rt,textures->nativeTextures(),c->r[16].uw[0],c->r[17].sw[0]);
    return false; // Original lookup, scalar assignment/reset and callbacks remain.
}
}

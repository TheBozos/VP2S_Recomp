#pragma once
#include "native_textures.h"
namespace vp2s {
struct NativeButtonSheet {
    uint32_t sprite=0,dtt=0,parts=0,count=0,button=0;
    std::shared_ptr<const NativeTextureAsset> asset;
};
class NativeUiSprites {
    std::map<uint32_t,std::shared_ptr<const NativeButtonSheet>> sheets;
public:
    void bindButton(Runtime *,NativeTextures &,uint32_t sprite,int32_t button);
    void reset(uint32_t sprite){sheets.erase(sprite);}
    std::shared_ptr<const NativeButtonSheet> button(uint32_t sprite)const;
};
class UiSpriteOwner {
public:
    virtual ~UiSpriteOwner()=default;
    virtual NativeUiSprites &nativeUiSprites()=0;
};
std::optional<bool> nativeUiSpriteInstruction(R5900Context *,Runtime *,uint32_t pc);
}

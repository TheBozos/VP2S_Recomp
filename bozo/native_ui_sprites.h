#pragma once
#include "native_textures.h"
namespace vp2s {
struct NativeButtonSheet {
    uint32_t sprite=0,dtt=0,parts=0,count=0,button=0;
    std::shared_ptr<const NativeTextureAsset> asset;
};
struct NativeUiSprite {
    uint32_t sprite=0,colour=0,depth=0;
    std::array<std::array<int32_t,2>,4> fixedCorners{};
    std::array<float,4> uv{};
    std::shared_ptr<const bozo::render::Texture> picture;
    bool additive=false;
};
class NativeUiSprites {
    std::map<uint32_t,std::shared_ptr<const NativeButtonSheet>> sheets;
    struct TextBuild {uint32_t element;std::vector<NativeUiSprite> sprites;};
    struct SpriteBuild {uint32_t textStack;NativeUiSprite sprite;uint32_t mask=0;bool coloured=false;};
    std::map<uint32_t,TextBuild> textBuilds;
    std::map<uint32_t,SpriteBuild> spriteBuilds;
public:
    void bindButton(Runtime *,NativeTextures &,uint32_t sprite,int32_t button);
    void reset(uint32_t sprite);
    std::shared_ptr<const NativeButtonSheet> button(uint32_t sprite)const;
    void beginText(uint32_t stack,uint32_t element);
    std::vector<NativeUiSprite> finishText(uint32_t stack,uint32_t element);
    void beginSprite(Runtime *,uint32_t stack,uint32_t sprite,uint32_t depth,bool additive=false);
    void colour(uint32_t stack,uint32_t colour);
    void corner(uint32_t stack,unsigned index,int32_t x,int32_t y,const std::array<float,4> &uv);
    void finishSprite(uint32_t stack);
    size_t outstanding()const{return textBuilds.size()+spriteBuilds.size();}
    bool buildingText(uint32_t element)const;
};
class UiSpriteOwner {
public:
    virtual ~UiSpriteOwner()=default;
    virtual NativeUiSprites &nativeUiSprites()=0;
};
std::optional<bool> nativeUiSpriteInstruction(R5900Context *,Runtime *,uint32_t pc);
}

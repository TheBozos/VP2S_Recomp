#pragma once
#include "native_glyphs.h"
#include <array>
namespace vp2s {
struct NativeUiGlyph {
    std::array<float,4> box{};
    std::array<float,2> scale{};
    int32_t cellWidth=0;
    uint32_t index=0,colour=0,shadowMode=0,shadowColour=0;
    std::shared_ptr<const NativeGlyphFont> font;
    std::shared_ptr<const bozo::render::Texture> picture,shadow;
};
struct NativeUiText {uint32_t element=0;bool inlineImages=false;std::vector<NativeUiGlyph> glyphs;};
class NativeUiTexts {
    struct Build {
        NativeUiText text;
        uint32_t items=0;int32_t count=0;
        uint32_t last=0;bool active=false,hasLast=false;
    };
    std::map<uint32_t,Build> builds;
    std::map<uint32_t,std::shared_ptr<const NativeUiText>> texts;
public:
    void enter(Runtime *,uint32_t stack,uint32_t element,uint32_t items,int32_t count);
    void activate(uint32_t stack,uint32_t element,uint32_t items,int32_t count);
    void capture(Runtime *,NativeGlyphs &,const NativeAssets &,uint32_t stack,uint32_t element,uint32_t item,uint32_t index,bool image);
    void finish(uint32_t stack,uint32_t element);
    void release(uint32_t element);
    std::shared_ptr<const NativeUiText> text(uint32_t element)const;
    size_t outstanding()const{return builds.size();}
};
class UiTextOwner {
public:
    virtual ~UiTextOwner()=default;
    virtual NativeUiTexts &nativeUiTexts()=0;
    virtual NativeGlyphs &nativeGlyphs()=0;
};
std::optional<bool> nativeUiTextInstruction(R5900Context *,Runtime *,uint32_t pc);
}

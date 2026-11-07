#pragma once
#include "native_glyphs.h"
#include "bozo/render/renderer.h"
#include <array>
namespace vp2s {
struct NativeUiGlyphPlacement {
    std::array<int32_t,4> fixedBox{};
    std::array<int32_t,2> fixedShadowOffset{};
    uint32_t depth=0,colour=0,shadowColour=0;
};
struct NativeUiGlyph {
    std::array<float,4> box{};
    std::array<float,2> scale{};
    int32_t cellWidth=0;
    uint32_t index=0,colour=0,shadowMode=0,shadowColour=0;
    std::shared_ptr<const NativeGlyphFont> font;
    std::shared_ptr<const bozo::render::Texture> picture,shadow;
    NativeUiGlyphPlacement placement;
    bool placed=false;
};
struct NativeUiText {uint32_t element=0;bool inlineImages=false;std::vector<NativeUiGlyph> glyphs;};
struct NativeUiLayer {uint32_t layer=0,object=0;std::vector<std::shared_ptr<const NativeUiText>> texts;};
class NativeUiTexts {
    struct Build {
        NativeUiText text;
        uint32_t items=0;int32_t count=0;
        uint32_t last=0;bool active=false,hasLast=false,lastIsGlyph=false;
    };
    std::map<uint32_t,Build> builds;
    std::map<uint32_t,std::shared_ptr<const NativeUiText>> texts;
public:
    void enter(Runtime *,uint32_t stack,uint32_t element,uint32_t items,int32_t count);
    void activate(uint32_t stack,uint32_t element,uint32_t items,int32_t count);
    void capture(Runtime *,NativeGlyphs &,const NativeAssets &,uint32_t stack,uint32_t element,uint32_t item,uint32_t index,bool image);
    void finish(uint32_t stack,uint32_t element);
    void place(uint32_t stack,uint32_t element,const NativeUiGlyphPlacement &);
    void release(uint32_t element);
    std::shared_ptr<const NativeUiText> text(uint32_t element)const;
    NativeUiLayer layer(Runtime *,uint32_t layer)const;
    size_t outstanding()const{return builds.size();}
};
std::array<bozo::render::Vertex,6> nativeUiGlyphVertices(const NativeUiGlyph &,bool shadow);
class UiTextOwner {
public:
    virtual ~UiTextOwner()=default;
    virtual NativeUiTexts &nativeUiTexts()=0;
    virtual NativeGlyphs &nativeGlyphs()=0;
    virtual void drawNativeUiLayer(const NativeUiLayer &){throw std::runtime_error("native UI layer has no draw owner");}
};
std::optional<bool> nativeUiTextInstruction(R5900Context *,Runtime *,uint32_t pc);
}

#pragma once
#include "native_glyphs.h"
#include "native_ui_sprites.h"
#include "bozo/render/renderer.h"
#include <array>
namespace vp2s {
struct NativeUiWindow;
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
struct NativeUiText {uint32_t element=0;bool inlineImages=false;std::vector<NativeUiGlyph> glyphs;std::vector<NativeUiSprite> icons;bool iconsCaptured=false;std::shared_ptr<const NativeUiWindow> window;
    std::vector<bozo::render::Vertex> additiveTriangles;  // untextured Cs*As+Cd geometry (chapter title bar)
    std::vector<bozo::render::Vertex> alphaTriangles;     // untextured (Cs-Cd)As+Cd geometry, drawn after the icons (name plate)
    std::vector<NativeUiSprite> lateIcons;};               // textured, drawn after alphaTriangles (name plate event image)
std::vector<bozo::render::Vertex> nativeChapterBar(Runtime *,uint32_t element);
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
    void icons(uint32_t element,std::vector<NativeUiSprite>);
    void spriteElement(uint32_t element,std::vector<NativeUiSprite>);
    std::shared_ptr<const NativeUiText> text(uint32_t element)const;
    // Event name plate (element class 167820): built once by 465150, path A.
    std::map<uint32_t,std::shared_ptr<const NativeUiText>> plates;
    std::shared_ptr<const NativeUiText> plate(uint32_t element)const{auto it=plates.find(element);return it==plates.end()?nullptr:it->second;}
    NativeUiLayer layer(Runtime *,uint32_t layer)const;
    size_t outstanding()const{return builds.size();}
};
std::array<bozo::render::Vertex,6> nativeUiGlyphVertices(const NativeUiGlyph &,bool shadow);
std::array<bozo::render::Vertex,6> nativeUiSpriteVertices(const NativeUiSprite &);
class UiTextOwner {
public:
    virtual ~UiTextOwner()=default;
    virtual NativeUiTexts &nativeUiTexts()=0;
    virtual NativeGlyphs &nativeGlyphs()=0;
    virtual void drawNativeUiLayer(const NativeUiLayer &){throw std::runtime_error("native UI layer has no draw owner");}
};
std::optional<bool> nativeUiTextInstruction(R5900Context *,Runtime *,uint32_t pc);
std::optional<bool> nativeNamePlateInstruction(R5900Context *,Runtime *,uint32_t pc);
// Event title ornament (element class 165e00, builder = layer method 449580):
// left (f21), right (f20) and top (f22) as the builder computed them after its
// mode adjustments (site 449790), the colour (s3), depth (layer +c4) and the
// resource table's rects +a0 (centre) and +90 (wings).
struct NativeOrnamentInputs {
    uint32_t left=0,right=0,top=0,colour=0,depth=0;
    std::array<std::array<uint16_t,6>,2> rects{};   // +a0, +90
    unsigned log2Width=8,log2Height=8;
};
NativeUiText nativeOrnament(const NativeOrnamentInputs &,std::shared_ptr<const bozo::render::Texture> picture);
// Town prompt bar (element class 165c30, build 4489b0): at the join 448d70 the
// builder holds x1 (f23), y0 (f21), y1 (f20), the cap width (f24) and the
// screen's right edge (f22); the colour (s2), depth (layer +c4) and the
// resource table's rects +160 (cap) and +170 (body, stretched to the edge).
struct NativeBarInputs {
    uint32_t x1=0,y0=0,y1=0,cap=0,right=0,colour=0,depth=0;
    std::array<std::array<uint16_t,6>,2> rects{};   // +160, +170
    unsigned log2Width=8,log2Height=8;
};
NativeUiText nativeBar(const NativeBarInputs &,std::shared_ptr<const bozo::render::Texture> picture);
// Name-plate geometry from the scalars builder 465150 computed when it enters
// path A (465e3e8): X (f24), Y (f23), length L (f28), all PS2 float words; the
// colour (s6), depth (fp), the frame size and the resource image's sub-image
// table (u16 u0,v0,u1,v1,w,h at +b0/+140/+150) and its log2 sizes; the plate
// tail offset (layer +2a4); the global half-width flag (48e04c byte 3 bit 4).
struct NativeNamePlateInputs {
    uint32_t x=0,y=0,length=0,colour=0,depth=0,tailOffset=0;
    int32_t frameWidth=512,frameHeight=512;
    std::array<std::array<uint16_t,6>,3> rects{};   // +b0, +140, +150
    unsigned log2Width=8,log2Height=8;
    bool narrowCap=false;
    // Tail (465f08..4662f8, path A): the event texture uploader's image. Drawn when
    // layer +270 is set and the uploader (+26c) reports its upload done.
    bool icon=false;
    int32_t textSize=0;                              // s0: int(512*text size/640)
    uint32_t alpha=0,alphaScale=0;                   // layer +e0 and +298 (float words)
    std::array<uint16_t,6> iconRect{};               // layer +268 record: u0,v0,u1,v1,w,h
    unsigned iconLog2Width=0,iconLog2Height=0;       // the entry TEX0's TW/TH
};
NativeUiText nativeNamePlate(const NativeNamePlateInputs &,std::shared_ptr<const bozo::render::Texture> picture,
                             std::shared_ptr<const bozo::render::Texture> iconPicture=nullptr);
}

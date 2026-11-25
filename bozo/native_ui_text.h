#pragma once
#include "native_glyphs.h"
#include "native_ui_sprites.h"
#include "bozo/render/renderer.h"
#include <array>
namespace vp2s {
struct PictureContext;
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
// The emitter-built text classes the native UI admits (15a450 and its subclasses, 15a350).
bool nativeUiTextClass(uint32_t type);
class NativeUiTexts {
    struct Build {
        NativeUiText text;
        uint32_t items=0;int32_t count=0;
        uint32_t last=0;bool active=false,hasLast=false,lastIsGlyph=false;
    };
    std::map<uint32_t,Build> builds;
    std::map<uint32_t,std::shared_ptr<const NativeUiText>> texts;
    struct BattleGlyph {uint32_t stack=0,item=0,index=0,uploads=0,sprites=0;};
    std::map<uint32_t,BattleGlyph> battleGlyphs;   // by element: the item 4c8fb0 is drawing
public:
    void enter(Runtime *,uint32_t stack,uint32_t element,uint32_t items,int32_t count);
    void activate(uint32_t stack,uint32_t element,uint32_t items,int32_t count);
    void capture(Runtime *,NativeGlyphs &,const NativeAssets &,uint32_t stack,uint32_t element,uint32_t item,uint32_t index,bool image);
    void finish(uint32_t stack,uint32_t element);
    void place(uint32_t stack,uint32_t element,const NativeUiGlyphPlacement &);
    // Battle text 1748d0 (mod028): its item callback 4c8fb0 uploads the item's glyph cell to a GS
    // cache (list +58) and writes its own sprites (list +60). The item is noted at the emitter's
    // call (2e757c) and read back from both lists when 4c8fb0 commits its cursor (4c9f78).
    void beginBattleGlyph(Runtime *,uint32_t stack,uint32_t element,uint32_t item,uint32_t index);
    void battleGlyph(Runtime *,NativeGlyphs &,const NativeAssets &,uint32_t element,uint32_t end);
    void release(uint32_t element);
    void icons(uint32_t element,std::vector<NativeUiSprite>);
    void spriteElement(uint32_t element,std::vector<NativeUiSprite>);
    std::shared_ptr<const NativeUiText> text(uint32_t element)const;
    // Event name plate (element class 167820): built once by 465150, path A.
    std::map<uint32_t,std::shared_ptr<const NativeUiText>> plates;
    std::shared_ptr<const NativeUiText> plate(uint32_t element)const{auto it=plates.find(element);return it==plates.end()?nullptr:it->second;}
    NativeUiLayer layer(Runtime *,uint32_t layer)const;
    size_t outstanding()const{return builds.size()+battleGlyphs.size();}
    // Save states (native_state_ui.cpp): texts and plates; a build in flight makes a save wait.
    void serialize(bozo::state::Archive &,const PictureContext &);
    std::string unsaveable()const{return builds.empty()?"":"a native UI text build is in flight";}
};
std::array<bozo::render::Vertex,6> nativeUiGlyphVertices(const NativeUiGlyph &,bool shadow);
std::array<bozo::render::Vertex,6> nativeUiSpriteVertices(const NativeUiSprite &);
// Countdown timer HUD (class 168e10, mod008 build 48fea0): one REGLIST sprite from the words its builder stores (RGBAQ,
// UV, XYZ2, UV, XYZ2) over a width x height texture, and its six-vertex Gouraud strip (three RGBAQ, two XYZ2 each).
NativeUiSprite nativeTimerSprite(uint32_t colour,uint64_t uv0,uint64_t xyz0,uint64_t uv1,uint64_t xyz1,uint32_t width,uint32_t height);
std::vector<bozo::render::Vertex> nativeTimerStrip(const std::array<uint32_t,3> &colours,const std::array<uint64_t,6> &xyz);
std::optional<bool> nativeTimerInstruction(R5900Context *,Runtime *,uint32_t pc);
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
// Field pause logo (element class 166ef0, build 4514f0): at 4516f0 the builder
// holds x (f21) = 512 (73 + layer x)/640 + originX and y (f20) = 512.96 (79 +
// layer y)/480 + originY, the colour (s4: 808080, alpha int(layer +e0)), depth
// (layer +c4) and the field HUD sheet's rect +30.
struct NativePauseLogoInputs {
    uint32_t x=0,y=0,colour=0,depth=0;
    std::array<uint16_t,6> rect{};
    unsigned log2Width=8,log2Height=8;
};
NativeUiText nativePauseLogo(const NativePauseLogoInputs &,std::shared_ptr<const bozo::render::Texture> picture);
// Field pause bottom bar (element class 1670a0, build 451a60): at 451b64 the
// builder holds x (f21) = 512 layer x/640 and y (f20) = 512.96 layer y/480,
// before the screen origin (s2, s1); depth layer +c4.
struct NativePauseBarInputs {
    uint32_t x=0,y=0,depth=0;
    int32_t originX=0x700,originY=0x700;
};
NativeUiText nativePauseBar(const NativePauseBarInputs &);
// Picture element 167b80 (build 4689f0, built once per +3b clear): one SPRITE (PRIM 356, context 2) over the field
// resource image (TEX0 as the pause logo's), colour 808080 with alpha (element +28 x layer +e0 / 128, unsigned
// conversion), depth layer +c4. Its rect (u0, v0, u1, v1, w, h) is the resource table's +1f0, UVs as 410c30. With
// x = layer +40 + element +18, y = layer +44 + element +1c, w, h scaled by element +54, and the origin
// (0x800 - frame width / 2, 0x800 - frame height / 2): X = 16 (ox + 0.8 x), Y = 16 (oy + 1.0687 y), up to
// x + w, y + h. All inputs are PS2 float words except the frame size, depth and rect.
struct NativePictureInputs {
    uint32_t layerX=0,layerY=0,x=0,y=0,scale=0,alpha=0,layerAlpha=0,depth=0;
    int16_t frameWidth=640,frameHeight=448;
    std::array<uint16_t,6> rect{};
    unsigned log2Width=8,log2Height=8;
};
NativeUiText nativePicture(const NativePictureInputs &,std::shared_ptr<const bozo::render::Texture> picture);
// Its UVs as 410c30 writes them (u0, v0, u1, v1): (1 << log2) / 256 times the rect, truncated.
std::array<int32_t,4> nativePictureUv(const std::array<uint16_t,6> &rect,unsigned log2Width,unsigned log2Height);
// Shadow frame 167b50 (build 468620, built once per +3b clear): nine Gouraud quads (PRIM 24c, REGLIST strips of four)
// in black, around the rectangle x = layer +40 + element +18, y = layer +44 + element +1c, w = +20, h = +24, 8 units
// wide; the frame mapping of the display record [35e170] (+840 / +844 scale, +848 / +84c offset) truncated to 12.4.
// The outer edge has alpha 0, the inner one alpha int(0.5 (layer +e0 x element +28) / 128). Depth layer +c4.
// Name tab of layer family 168010 (maps 329, 333, 432, 433): its ornament element 165e00's build 4494f0 calls the
// family's writer 46cb50, which (layer +365 set, path 46cff0) writes two SPRITEs (PRIM 356, context 2, ALPHA_2 0x44)
// over the field resource image, colour 808080 with alpha (layer +e0, unsigned conversion), depth layer +224:
// L = ox + 512 x / 640, R = ox + 512 (+288 + (x + W)) / 640, Y = 512.96 y / 480 + oy (W the width of the layer's text
// +108, as 2ef860 returns it at +8). Both span 16 (Y - 2) .. that + 16 h of rect +200; sprite A has rect +200, B +210.
// Mode 3 (+367 & f, mirrored): A from 16 (R + w) to 16 R, B from 16 R to 16 L; otherwise A from 16 (L - w) to 16 L,
// B from 16 L to 16 (128 + R). UVs as 410c30. Inputs are PS2 float words except the ints and rects.
struct NativeNameTabInputs {
    uint32_t layerX=0,layerY=0,extra=0,width=0,alpha=0,depth=0;
    int16_t frameWidth=640,frameHeight=448;
    bool mirrored=false;
    std::array<uint16_t,6> cap{},body{};   // rects +200, +210
    unsigned log2Width=8,log2Height=8;
};
struct NativeNameTabEdges {uint32_t left=0,right=0;};   // L and R (f23, f21) as PS2 words
// Path A (layer +365 clear): the base ornament 449580, then one Gouraud strip (PRIM 24c) of eight vertices at depth
// layer +224, colour 2b41ad: (16 (L - 24), 384 + that, 16 R, 16 (24 + R)) by (16 (24 + Y), 48 + that), alpha 0 at the
// two outer columns and the layer's alpha (unsigned conversion of +e0) at the inner two.
std::vector<bozo::render::Vertex> nativeNameTabStrip(const NativeNameTabInputs &);
NativeNameTabEdges nativeNameTabEdges(const NativeNameTabInputs &);
NativeUiText nativeNameTab(const NativeNameTabInputs &,std::shared_ptr<const bozo::render::Texture> picture);
// Symbol sprite 167fe0 (build 46c3a0, built once per +3b clear; type +64 = 5 is never built): one SPRITE (PRIM 356,
// context 2) of the field resource image (element +58 clear; rect +40, or +50 / +1d0 / +1e0 for types 2 / 3 / 4) at
// depth layer +c4. Its size is element +20 / +24, or the rect's w / h where those are 0. With x = layer +40 + element +18,
// y = layer +44 + element +1c and the scales (sx, sy) = (1, 0.9333) when bit 5 of byte [*48e048] +de is set (3d4fe0 with
// mask 8), else (0.8, 1.0687): X = 16 (ox + sx x) .. 16 (ox + sx (x + w)), Y = 16 (oy + sy y) .. 16 (oy + sy (y + h)), the
// rows swapped for type 1. Colour 804040 (type 1), 404080 (type 0) or 808080, alpha (element +28 x layer +e0) / 128 converted
// unsigned. Inputs are PS2 float words except the ints and the rect.
struct NativeSymbolInputs {
    uint32_t layerX=0,layerY=0,x=0,y=0,width=0,height=0,alpha=0,layerAlpha=0,depth=0;
    int8_t type=0;bool wide=false;
    int16_t frameWidth=640,frameHeight=448;
    std::array<uint16_t,6> rect{};
    unsigned log2Width=8,log2Height=8;
};
struct NativeSymbolEdges {uint32_t scaleX=0,x0=0,x1=0,ya=0,yb=0;};   // the build's f8, f6, f5, f3, f2 (16 x, as PS2 words)
NativeSymbolEdges nativeSymbolEdges(const NativeSymbolInputs &);
NativeUiText nativeSymbol(const NativeSymbolInputs &,std::shared_ptr<const bozo::render::Texture> picture);
// Highlight bar 167b20 (build 4683a0, once per +3b clear): one Gouraud strip (PRIM 24c) of six vertices in colour 2b51bd,
// at depth layer +c4 + element +2c, mapped by the display record [35e170] (+840 / +844 scale, +848 / +84c offset) and
// truncated: columns x, x + w / 4, x + w (x = layer +40 + element +18, w = +20) by rows y + h, y (y = layer +44 + element
// +1c, h = +24); the quarter column has alpha a = int((layer +e0 x element +28) / 128), the outer ones int(0.2 a).
struct NativeHighlightInputs {
    uint32_t layerX=0,layerY=0,x=0,y=0,width=0,height=0,alpha=0,layerAlpha=0,depth=0;
    uint32_t scaleX=0,scaleY=0,offsetX=0,offsetY=0;
};
struct NativeHighlight {std::array<int32_t,3> x{};std::array<int32_t,2> y{};uint32_t outer=0,inner=0;};   // y: rows y + h, y
NativeHighlight nativeHighlightEdges(const NativeHighlightInputs &);
std::vector<bozo::render::Vertex> nativeHighlight(const NativeHighlightInputs &);
struct NativeShadowFrameInputs {
    uint32_t layerX=0,layerY=0,x=0,y=0,width=0,height=0,alpha=0,layerAlpha=0,depth=0;
    uint32_t scaleX=0,scaleY=0,offsetX=0,offsetY=0;
};
struct NativeShadowFrame {std::array<int32_t,4> x{},y{};uint32_t alpha=0;};
NativeShadowFrame nativeShadowFrameEdges(const NativeShadowFrameInputs &);
NativeUiText nativeShadowFrame(const NativeShadowFrameInputs &);
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
    // Path B (465b38): short plates (int(512*size/640) < 29) or mirrored ones (layer +2ac bit 0).
    bool pathB=false,mirror=false;
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

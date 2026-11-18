#pragma once
#include "native_ui_text.h"
#include <optional>
namespace vp2s {
struct PictureContext;
struct NativeUiStrip {
    std::array<std::array<int32_t,2>,4> corners{};
    std::array<uint32_t,4> colours{};
    uint32_t depth=0;
};
struct NativeUiWindow {uint32_t element=0;std::vector<NativeUiStrip> background;std::vector<NativeUiSprite> border;
    // Scissor element 15a420: the drawing rectangle in frame pixels (x0, y0, x1, y1 inclusive) from here on.
    std::optional<std::array<int32_t,4>> scissor;};
class NativeUiWindows {
    struct Build {NativeUiWindow window;bool background=false,border=false;};
    struct Background {uint32_t parentStack=0,element=0;bool captured=false;};
    struct Quads {
        uint32_t parentStack=0,sprite=0,count=0,list=0,colour=0;
        std::shared_ptr<const NativeButtonSheet> sheet;
        std::optional<NativeUiSprite> quad;
        unsigned uvMask=0;bool coloured=false;
        uint32_t element=0;bool image=false; // Camp menu image list (15aa20): its own capture, no window.
    };
    std::map<uint32_t,NativeUiWindow> imageBuilds; // Per quad-list stack.
    std::vector<NativeUiSprite> &quadTarget(const Quads &);
    std::map<uint32_t,Build> builds;
    std::map<uint32_t,Background> backgrounds;
    std::map<uint32_t,Quads> quads;
    std::map<uint32_t,std::shared_ptr<const NativeUiWindow>> ready;
    struct Selection {uint32_t element=0,alpha=0,depth=0;std::array<int32_t,6> coordinates{};uint32_t mask=0;bool coloured=false;};
    std::map<uint32_t,Selection> selections;
    unsigned captures=0;
public:
    void begin(Runtime *,uint32_t stack,uint32_t element);
    void beginBackground(uint32_t stack,uint32_t element);
    void background(Runtime *,uint32_t stack,const std::array<uint32_t,3> &colours,uint32_t edge,uint32_t depth);
    void endBackground(uint32_t stack);
    void beginQuads(Runtime *,NativeUiSprites &,uint32_t stack,uint32_t sprite,uint32_t caller);
    void beginImageQuads(Runtime *,const NativeTextures &,uint32_t stack,uint32_t sprite,uint32_t element);
    void colour(uint32_t stack,uint32_t colour);
    void quad(Runtime *,uint32_t stack,uint32_t part,uint32_t csa,uint32_t list);
    void corners(uint32_t stack,int32_t x0,int32_t y0,int32_t x1,int32_t y1,uint32_t depth);
    void uv(uint32_t stack,unsigned index,int32_t coordinate);
    void finishQuads(uint32_t stack);
    void finish(Runtime *,uint32_t stack,uint32_t element);
    std::shared_ptr<const NativeUiWindow> window(uint32_t element)const;
    void release(uint32_t element);
    void beginSelection(Runtime *,uint32_t stack,uint32_t element);
    void selectionColour(uint32_t stack,uint32_t alpha,uint32_t depth);
    void selectionCoordinate(uint32_t stack,unsigned index,int32_t coordinate);
    void finishSelection(uint32_t stack,uint32_t element);
    void panel(Runtime *,uint32_t element,uint64_t lowerRight,uint64_t upperLeft,uint32_t alpha);
    void spriteElement(uint32_t element,std::vector<NativeUiSprite> sprites);
    void scissor(Runtime *,uint32_t element,uint64_t registerValue);
    // Save states (native_state_ui.cpp): the finished windows; captures in flight make a save wait.
    void serialize(bozo::state::Archive &,const PictureContext &);
    std::string unsaveable()const{return imageBuilds.empty() && builds.empty() && backgrounds.empty() && quads.empty() && selections.empty()?"":"a native UI window capture is in flight";}
};
class UiWindowOwner {
public:
    virtual ~UiWindowOwner()=default;
    virtual NativeUiWindows &nativeUiWindows()=0;
};
std::optional<bool> nativeUiWindowInstruction(R5900Context *,Runtime *,uint32_t);
NativeUiLayer nativeMenuBackdrop(Runtime *,const NativeTextures &,uint32_t object);
std::shared_ptr<const bozo::render::Texture> nativeMenuBackdropPicture(const NativeTextureAsset &top,const NativeTextureAsset &bottom);
std::array<bozo::render::Vertex,6> nativeUiStripVertices(const NativeUiStrip &);
// The material for an untextured UI primitive (window backgrounds, panels,
// selection bars, flat and additive triangles): GS PRIM TME=0 takes the vertex
// RGB as display bytes (0x20 is 0x20, not 0x40), alpha still 0x80 = 1.0.
bozo::render::Material nativeUiFlatMaterial(bozo::render::Material::Blend);
}

#pragma once
#include "native_textures.h"
#include <optional>
namespace vp2s {
struct NativeDebugFont {
    uint32_t object=0,writer=0,begin=0,end=0,background=0;
    uint16_t glyphWidth=0,glyphHeight=0;
    std::shared_ptr<const NativeTextureAsset> atlas;
};
class NativeDebugFonts {
    struct Initializer {uint32_t object,background;std::shared_ptr<const NativeTextureAsset> atlas;};
    struct Publication {std::shared_ptr<const NativeDebugFont> font;bool retired=false;};
    std::map<uint32_t,Initializer> initializers;
    std::map<uint32_t,std::shared_ptr<const NativeDebugFont>> fonts;
    std::map<uint32_t,Publication> publications;
public:
    void begin(Runtime *,NativeTextures &,uint32_t stack,uint32_t object,uint32_t background);
    void finish(Runtime *,uint32_t stack);
    void publish(Runtime *,uint32_t stack,uint32_t object);
    void retire(Runtime *,R5900Context *);
    void returnFromFlush(uint32_t stack);
    std::shared_ptr<const NativeDebugFont> font(uint32_t object)const;
    size_t outstanding()const{return initializers.size()+publications.size();}
};
class DebugFontOwner {
public:
    virtual ~DebugFontOwner()=default;
    virtual NativeDebugFonts &nativeDebugFonts()=0;
};
std::optional<bool> nativeDebugFontInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
}

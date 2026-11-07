#pragma once
#include "bozo/runtime.h"
#include "bozo/render/texture.h"
#include <array>
#include <filesystem>
#include <map>
#include <memory>
#include <vector>
namespace vp2s {
struct TextureKey {
    uint32_t tag=0,index=0;
    auto operator<=>(const TextureKey &)const=default;
};
struct NativeTextureAsset {
    TextureKey key;
    uint64_t identity=0;
    uint32_t width=0,height=0,format=0;
    std::vector<uint8_t> texels; // Offline linear indices or RGBA8, never console layout.
    std::array<uint32_t,256> palette{}; // Offline index order.
    std::shared_ptr<const bozo::render::Texture> picture(unsigned csa=0)const;
};
class NativeTextures {
    struct Invocation {
        std::shared_ptr<const NativeTextureAsset> asset;
        bool palette=false,pixels=false;
    };
    std::filesystem::path root;
    std::map<TextureKey,std::string> files;
    std::map<TextureKey,std::shared_ptr<const NativeTextureAsset>> loaded;
    std::map<uint32_t,Invocation> pending; // Per-host active stack frame, erased on every return.
    std::map<uint16_t,std::shared_ptr<const NativeTextureAsset>> palettes,pixels;
    uint64_t publications=0;
public:
    explicit NativeTextures(std::filesystem::path directory={},std::filesystem::path manifest={});
    std::shared_ptr<const NativeTextureAsset> load(TextureKey);
    void begin(uint32_t stack,TextureKey);
    void publish(uint32_t stack,uint16_t slot,bool palette);
    void finish(uint32_t stack);
    std::shared_ptr<const NativeTextureAsset> pixelAsset(uint16_t slot)const;
    std::shared_ptr<const NativeTextureAsset> paletteAsset(uint16_t slot)const;
    size_t outstanding()const{return pending.size();}
    uint64_t publicationCount()const{return publications;}
};
class TextureOwner {
public:
    virtual ~TextureOwner()=default;
    virtual NativeTextures &nativeTextures()=0;
};
bool nativeTextureInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
}

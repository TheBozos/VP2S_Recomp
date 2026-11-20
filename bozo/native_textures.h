#pragma once
#include "native_state.h"
#include <mutex>
#include "bozo/runtime.h"
#include "bozo/render/texture.h"
#include <array>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <vector>
namespace vp2s {
struct TextureKey {
    uint32_t tag=0,index=0;
    auto operator<=>(const TextureKey &)const=default;
};
inline constexpr TextureKey kStaticScenePalette{0x50325056,0x314200};
inline void io(bozo::state::Archive &a,TextureKey &k){bozo::state::fields(a,k.tag,k.index);}
struct NativeTextureAsset {
    TextureKey key;
    uint64_t identity=0;
    uint32_t width=0,height=0,format=0;
    std::vector<uint8_t> texels; // Offline linear indices or RGBA8, never console layout.
    std::array<uint32_t,256> palette{}; // Offline index order.
    // Preserve 0x80-scale alpha: each engine presenter expands it once.
    std::shared_ptr<const bozo::render::Texture> picture(unsigned csa=0)const;
    // Expanded pictures per palette part: the asset is immutable once loaded,
    // so each is built once (it used to be rebuilt per mesh per frame).
    mutable std::array<std::shared_ptr<const bozo::render::Texture>,16> pictures{};
    mutable std::mutex picturesLock;
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
    std::map<uint32_t,std::shared_ptr<const NativeTextureAsset>> images; // Game image objects, never hardware addresses.
    struct MenuPage {std::shared_ptr<const NativeTextureAsset> asset;uint16_t palette=0;};
    std::map<uint32_t,MenuPage> menuPages;
    // The battle's images, named as its loader 36b4a0 names them: by the graphics-memory block
    // its layout table (*(36d880)+0x68) assigns, pixels and palettes apart.
    std::map<uint32_t,std::shared_ptr<const NativeTextureAsset>> blockPixels,blockPalettes;
public:
    struct MenuLoad {uint32_t page=0,format=0,palette=0;};
private:
    std::map<uint32_t,MenuLoad> menuLoads; // Camp menu loader calls in flight (entry 45e870 to publication 45ea18). // The camp menu's own (page, format) handles; see nativeMenuImageInstruction.
    uint64_t publications=0;
public:
    explicit NativeTextures(std::filesystem::path directory={},std::filesystem::path manifest={});
    std::shared_ptr<const NativeTextureAsset> load(TextureKey);
    void begin(uint32_t stack,TextureKey);
    void publish(uint32_t stack,uint16_t slot,bool palette);
    void publishStaticPalette(uint16_t slot);
    void publishImage(uint32_t object,TextureKey);
    void releaseImage(uint32_t object);
    std::shared_ptr<const NativeTextureAsset> imageAsset(uint32_t object)const;
    void beginMenuLoad(uint32_t object,uint32_t page,uint32_t format,uint32_t palette);
    MenuLoad takeMenuLoad(uint32_t object);
    void publishMenuPage(uint16_t page,uint8_t format,uint16_t palette,TextureKey);
    std::shared_ptr<const NativeTextureAsset> menuPage(uint16_t page,uint8_t format,uint16_t palette)const;
    void publishBlock(uint32_t block,bool palette,TextureKey);
    std::shared_ptr<const NativeTextureAsset> blockAsset(uint32_t block,bool palette)const;
    // Saved apart, at the end of VP2S's section: states older than these records end before them.
    void serializeBlocks(bozo::state::Archive &);
    void finish(uint32_t stack);
    std::shared_ptr<const NativeTextureAsset> pixelAsset(uint16_t slot)const;
    std::shared_ptr<const NativeTextureAsset> paletteAsset(uint16_t slot)const;
    // Save states: every published asset as its key (assets are offline files, loaded again by
    // key); the decoded-file cache is dropped on load.
    void serialize(bozo::state::Archive &);
    size_t outstanding()const{return pending.size();}
    uint64_t publicationCount()const{return publications;}
};
class TextureOwner {
public:
    virtual ~TextureOwner()=default;
    virtual NativeTextures &nativeTextures()=0;
};
bool nativeTextureInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
std::optional<bool> nativeFixedImageInstruction(R5900Context *,Runtime *,uint32_t pc);
std::optional<bool> nativeTextureUploaderInstruction(R5900Context *,Runtime *,uint32_t pc);
std::optional<bool> nativeMenuImageInstruction(R5900Context *,Runtime *,uint32_t pc);
std::optional<bool> nativeBattleImageInstruction(R5900Context *,Runtime *,uint32_t pc);
// The game's deferred free (1205b0): a 0x400-entry ring of pointers released later.
void guestDeferredFree(Runtime *,uint32_t list,uint32_t pointer);
// A held asset saved as its key and loaded again through `textures`.
void stateAsset(bozo::state::Archive &,NativeTextures &,std::shared_ptr<const NativeTextureAsset> &);
}

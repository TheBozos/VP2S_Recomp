#pragma once
#include "native_state.h"
#include "native_assets.h"
#include <mutex>
#include "bozo/runtime.h"
#include "bozo/render/texture.h"
#include <array>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <tuple>
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
    // Keys whose disc copies differ, by the decoded part and texture-header offset of each copy:
    // the copy's synthetic key {tag, 0x80000000 | variant << 16 | index} (variants numbered by
    // distinct conversion in manifest order: stable for save states), listed in `files` too.
    std::map<std::tuple<uint64_t,uint32_t,uint32_t>,TextureKey> provenance;
    std::map<TextureKey,std::shared_ptr<const NativeTextureAsset>> loaded;
    std::map<uint32_t,Invocation> pending; // Per-host active stack frame, erased on every return.
    std::map<uint16_t,std::shared_ptr<const NativeTextureAsset>> palettes,pixels;
    std::map<uint32_t,std::shared_ptr<const NativeTextureAsset>> images; // Game image objects, never hardware addresses.
    struct MenuPage {std::shared_ptr<const NativeTextureAsset> asset;uint16_t palette=0;};
    std::map<uint32_t,MenuPage> menuPages;
    // The battle's images, named as its loader 36b4a0 names them: by the graphics-memory block
    // its layout table (*(36d880)+0x68) assigns, pixels and palettes apart.
    std::map<uint32_t,std::shared_ptr<const NativeTextureAsset>> blockPixels,blockPalettes;
    std::map<std::pair<uint64_t,uint64_t>,std::shared_ptr<const NativeTextureAsset>> repalettes;   // derived, not saved
public:
    struct MenuLoad {uint32_t page=0,format=0,palette=0;};
private:
    std::map<uint32_t,MenuLoad> menuLoads; // Camp menu loader calls in flight (entry 45e870 to publication 45ea18). // The camp menu's own (page, format) handles; see nativeMenuImageInstruction.
    uint64_t publications=0;
public:
    explicit NativeTextures(std::filesystem::path directory={},std::filesystem::path manifest={});
    std::shared_ptr<const NativeTextureAsset> load(TextureKey);
    // The key to load for a file the game loaded: `key` itself when its copies all convert
    // alike, else the copy at `header` (the origin of the file's texture header).
    TextureKey resolve(TextureKey key,const AssetOrigin &header)const;
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
    // A battle image the game composes in RAM (its FIS header copied from one file, its descriptor
    // and pixels its own, e.g. the party portrait sheet): the indices as the game wrote them.
    // Saved with its texels (key tag kDerivedTag); its palette is published apart.
    static constexpr uint32_t kDerivedTag=0x56524544u;   // "DERV"
    void publishDerivedBlock(uint32_t block,uint32_t width,uint32_t height,uint32_t format,std::vector<uint8_t> texels);
    std::shared_ptr<const NativeTextureAsset> blockAsset(uint32_t block,bool palette)const;
    // The image a TEX0 names by its blocks: the file published under the pixel block with the
    // palette published under the palette block. When the palette comes from another file (a
    // palette-only colour variant), the pixels' indices with that palette (built once).
    std::shared_ptr<const NativeTextureAsset> blockImage(uint32_t pixels,uint32_t palette);
    // A model's texture, named by the texture manager's key and TEX0. A key converted alike in
    // every disc copy loads as itself. A key with differing copies is the copy in the drawn mesh's
    // own decoded part (`meshPart`: disc, part), when that part holds one; else the battle image the
    // game published at TEX0's pixel block (TBP0), with the palette at its CBP, when it is this key's.
    std::shared_ptr<const NativeTextureAsset> modelImage(TextureKey key,uint64_t tex0,std::optional<std::pair<uint64_t,uint32_t>> meshPart={});
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
std::optional<bool> nativeBattleTextureSetInstruction(R5900Context *,Runtime *,uint32_t pc);
// The game's deferred free (1205b0): a 0x400-entry ring of pointers released later.
void guestDeferredFree(Runtime *,uint32_t list,uint32_t pointer);
// A held asset saved as its key and loaded again through `textures`.
void stateAsset(bozo::state::Archive &,NativeTextures &,std::shared_ptr<const NativeTextureAsset> &);
}

#pragma once
#include "native_ui_sprites.h"
#include <functional>
namespace vp2s {
// Battle 2D sprites (mod028). The battle UI classes (174e30, 175450, 174300, ...) build a draw list in one of two
// writers (+90/+94, chosen by +bb) in their build method (174e30: 4d4770): ALPHA_2 = 0x44 (120860), then
// texture selections (method vt+a0 = 3cd330) and sprites from the shared sprite writer 42db50
// (one REGLIST sprite: PRIM 0x256, RGBAQ, ST/XYZ2 twice). Its method 3cd0c0 later flushes the
// writer chosen by +bc through 120eb0 (from 3cd214; the base flush 42d4c0 from 42d4ec). Native: each texture selection and sprite
// is recorded from the producers' own values (the game's image record and texture-cache entry,
// the sprite writer's computed corners), and the flush draws that list instead of the packet.
// A texture selection names the image by its graphics-memory blocks (pixels, palette) and the
// palette part. The build runs before the same frame's upload publishes the image (3cd0c0
// uploads, then flushes the other writer), so pictures are resolved when the list is flushed.
// `bytes`: what the selection wrote to the writer (3cd330's 8-bit path: one TEX0_2, 32;
// 4f0e90's UI sheet: a palette-load TEX0_2 and the TEX0_2, 64).
struct NativeBattleTexture {uint32_t pixels=0,palette=0,csa=0,bytes=32;};
struct NativeBattleSpriteBuild {
    uint32_t object=0,begin=0;
    std::vector<NativeBattleTexture> textures;
    std::vector<uint32_t> spriteTextures;   // per sprite: its selection
    std::vector<NativeUiSprite> sprites;    // pictures set at the flush
};
// The texel path of 42db50 (42de78..42df50): ST = 0x397fffd5 * float(texel) per coordinate,
// from the image record's four halfwords (u0, v0, u1, v1), EE single-precision multiply.
std::array<float,4> nativeBattleSpriteTexels(const std::array<uint16_t,4> &texels);
class NativeBattleSprites {
    struct Lease {uint32_t object,writer,begin,end;bool retired=false,flushed=false;};
    std::map<uint32_t,NativeBattleSpriteBuild> builds;   // by writer
    std::map<uint32_t,Lease> leases;                     // by the flushing caller's stack
public:
    void begin(Runtime *,uint32_t object,uint32_t writer);
    void texture(uint32_t writer,const NativeBattleTexture &);
    void sprite(Runtime *,const R5900Context *);
    // `image(block, palette)`: the asset published under a graphics-memory block (NativeTextures::blockAsset).
    using ImageLookup=std::function<std::shared_ptr<const NativeTextureAsset>(uint32_t,bool)>;
    NativeBattleSpriteBuild flush(Runtime *,uint32_t stack,uint32_t object,uint32_t writer,const ImageLookup &image);
    bool active(uint32_t stack)const{return leases.contains(stack);}
    void retire(Runtime *,const R5900Context *);
    void flushed(Runtime *,uint32_t stack);
    void complete(uint32_t stack);
    void cancel(uint32_t stack){leases.erase(stack);}
    const NativeBattleSpriteBuild *build(uint32_t writer)const{auto it=builds.find(writer);return it==builds.end()?nullptr:&it->second;}
    size_t outstanding()const{return leases.size();}
    // Save states: the held builds (a writer is flushed a frame after it is built); their
    // sprites hold no pictures until the flush.
    void serialize(bozo::state::Archive &);
};
class BattleSpriteOwner {
public:
    virtual ~BattleSpriteOwner()=default;
    virtual NativeBattleSprites &nativeBattleSprites()=0;
    virtual void drawNativeBattleSprites(const NativeBattleSpriteBuild &)=0;
};
std::optional<bool> nativeBattleSpriteInstruction(R5900Context *,Runtime *,uint32_t pc);
}

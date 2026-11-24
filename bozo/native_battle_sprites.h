#pragma once
#include "native_ui_sprites.h"
#include <functional>
#include <set>
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
    std::vector<uint32_t> spriteTextures;   // per sprite: its selection (kUntextured: an untextured polygon)
    std::vector<NativeUiSprite> sprites;    // pictures set at the flush
    uint32_t polygonBytes=0,polygons=0;     // the recorded polygons (42df80, among the sprites) and what they wrote
    uint32_t blendWrites=0;                 // ALPHA_2 writes after the opening one (32 bytes each)
    bool additive=false;                    // the current ALPHA_2: 0x44 blending, 0x148 additive (Cs As + Cd)
    // The bytes the recorded content wrote after the writer's header.
    uint64_t accountedBytes()const{uint64_t n=32+uint64_t(polygonBytes)+32ull*blendWrites;for(const auto &t:textures)n+=t.bytes;return n+64ull*(sprites.size()-polygons);}
    static constexpr uint32_t kUntextured=~0u;
    // Sampling the frame presented last (the display buffer the GS reads at the record's +670 page): screen lenses.
    static constexpr uint32_t kPreviousFrame=~1u;
};
// The texel path of 42db50 (42de78..42df50): ST = 0x397fffd5 * float(texel) per coordinate,
// from the image record's four halfwords (u0, v0, u1, v1), EE single-precision multiply.
std::array<float,4> nativeBattleSpriteTexels(const std::array<uint16_t,4> &texels);
class NativeBattleSprites {
public:
    using ImageLookup=std::function<std::shared_ptr<const NativeTextureAsset>(uint32_t,uint32_t)>;
private:
    struct Lease {uint32_t object,writer,begin,end;bool retired=false,flushed=false;};
    std::map<uint32_t,NativeBattleSpriteBuild> builds;   // by writer
    std::map<uint32_t,Lease> leases;                     // by the flushing caller's stack
    std::map<uint32_t,std::pair<uint32_t,std::vector<NativeUiSprite>>> shadeChecks;   // VP2S_BOZO_VERIFY_SHADE: packet start, strips
    std::set<uint32_t> directHeaders;   // writers whose VIF DIRECT quadword was just reserved (within one build call)
    NativeBattleSpriteBuild resolve(const NativeBattleSpriteBuild &,const ImageLookup &)const;
public:
    // 120860 with ALPHA_2 = 0x44 on a reset writer: a build opens (false: not a reset writer).
    // With 0x148 (Cs As + Cd, e.g. the gauge bars of class 16f0a0) the build starts additive.
    bool begin(Runtime *,uint32_t writer,uint64_t alpha=0x44);
    void serializeBlends(bozo::state::Archive &);
    void serializeUvs(bozo::state::Archive &);   // appended after the blends: per-corner UVs
    // 120860 on a writer whose build is open at exactly its recorded content: a later ALPHA_2
    // (0x44 or 0x148) for the sprites after it (false: not an open build's writer).
    bool blend(Runtime *,uint32_t writer,uint64_t alpha);
    NativeBattleSpriteBuild &bind(Runtime *,uint32_t object,uint32_t writer);
    void texture(uint32_t writer,const NativeBattleTexture &);
    void sprite(Runtime *,const R5900Context *);
    // Polygon writer 42df80 after its unrotated corner loop (42e144): a 4-corner strip, flat or Gouraud, textured or not.
    void polygon(Runtime *,const R5900Context *);
    void rotatedPolygon(Runtime *,const R5900Context *);
    // Band writer 383cb0 before it closes the writer (3843a0): three Gouraud strips read from
    // the packet it wrote.
    void band(Runtime *,const R5900Context *);
    // The engine's flat rectangle writer main 120a80 (writer a0) where it commits its cursor
    // (120c30, t4 the new end): one flat strip read back from the packet, into the writer's
    // open battle build only (other writers are not battle UI builds and are left alone).
    void flatQuad(Runtime *,uint32_t writer,uint32_t end);
    // Rectangle writer 42ed50 where its unrotated paths join (42efe0): the same strip forms.
    void rectangle(Runtime *,const R5900Context *);
    // Corner shade, class 175600 (build 4f7a80): vt+a0 4f7a40 opens the build over its static
    // register block, vt+a8 4f72d0 draws (false: its mode draws nothing), vt+a4 4f7a00 closes it.
    void cornerShadeOpen(Runtime *,uint32_t object,uint32_t writer);
    bool cornerShade(Runtime *,uint32_t object,uint32_t writer);
    void cornerShadeClose(Runtime *,uint32_t object,uint32_t writer);
    // Screen lens, class 16e9e0 (mod028 build 377180, the victory's): its producers write raw packets that
    // sample the displayed frame; at their commits the vertices they wrote are read back under exact checks.
    // 3764e0's fans (at 376f68), 375db0's frame strips (at 376490), 375c80's restore block (at 375d88), each before its commit.
    void lensFans(Runtime *,uint32_t object,uint32_t writer,uint32_t end);
    void lensStrips(Runtime *,uint32_t object,uint32_t writer,uint32_t end);
    void lensRestore(Runtime *,uint32_t writer,uint32_t end);
    // VIF1-direct battle UI, class 174530: its header vt+7c 42d3e0 reserves one quadword for a VIF
    // DIRECT code on the reset writer, so the ALPHA_2 that follows opens the build one quadword in.
    // Its enqueue 42d340 (patched to complete the job at once) draws the +bc writer's build.
    void directHeader(Runtime *,uint32_t object,uint32_t writer);
    NativeBattleSpriteBuild submitDirect(Runtime *,uint32_t object,const ImageLookup &image);
    struct Strip {uint32_t object=0,desc=0,mode=0,data=0,z=0,colours=0,st=0,image=0;std::array<std::array<int32_t,2>,4> corners{};bool triangle=false;};
    void strip(Runtime *,const Strip &);
    // `image(pixels, palette)`: the image a TEX0 names by its blocks (NativeTextures::blockImage).
    NativeBattleSpriteBuild flush(Runtime *,uint32_t stack,uint32_t object,uint32_t writer,const ImageLookup &image);
    bool active(uint32_t stack)const{return leases.contains(stack);}
    void retire(Runtime *,const R5900Context *);
    void flushed(Runtime *,uint32_t stack);
    uint32_t complete(uint32_t stack);   // the flushed object
    void cancel(uint32_t stack){leases.erase(stack);}
    const NativeBattleSpriteBuild *build(uint32_t writer)const{auto it=builds.find(writer);return it==builds.end()?nullptr:&it->second;}
    size_t outstanding()const{return leases.size();}
    // Save states: the held builds (a writer is flushed a frame after it is built); their
    // sprites hold no pictures until the flush.
    void serialize(bozo::state::Archive &);
};
// Screen shockwave, class 1746d0 (mod028; battles 0x2c, 0x2d, 0x2f, 0x38): its draw 4c0e50 queued the chain of the
// +90[+bc] writer as a type-3 job (120c50, owner +c0). The chain redraws the frame presented last (the display
// buffer, as the screen lens) displaced: a fan around the target and a ring of 5-vertex strips. Native: the kernel
// makes the enqueue complete at once (42d4a0 -> 21c9a0) and site 4c0e80 reads the chain back into these.
struct NativeShockwave {
    uint32_t object=0;
    std::vector<NativeUiSprite> fan,strips;   // triangles as quads (the 4th corner repeats the 3rd); free UVs
    bool linear=true;                         // TEX1_2 0x61 (else 1: nearest)
    bool fanDepthWrite=false,stripDepthWrite=false;   // ZBUF_2 without ZMSK
};
NativeShockwave nativeShockwave(Runtime *,uint32_t object);
class BattleSpriteOwner {
public:
    virtual ~BattleSpriteOwner()=default;
    virtual NativeBattleSprites &nativeBattleSprites()=0;
    virtual void drawNativeBattleSprites(const NativeBattleSpriteBuild &)=0;
    virtual void drawNativeShockwave(const NativeShockwave &){throw std::runtime_error("native screen shockwave has no renderer");}
};
std::optional<bool> nativeBattleSpriteInstruction(R5900Context *,Runtime *,uint32_t pc);
}

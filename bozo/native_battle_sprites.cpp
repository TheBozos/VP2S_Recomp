#include "native_battle_sprites.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/hash.h"
#include "bozo/ops_fpu.h"
#include <algorithm>
#include <bit>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
#include <stdexcept>
namespace vp2s {
namespace {
const uint8_t *record(Runtime *rt,uint32_t at,uint32_t bytes){
    at&=0x1ffffff;   // writer pointers are uncached (0x30000000) mirrors
    if(!rt || at<0x100000 || at>=0x2000000 || bytes>0x2000000-at)throw std::runtime_error("native battle sprite record outside EE RAM");
    return rt->memory.hostPointer(at,bytes);
}
template<class T>T value(Runtime *rt,uint32_t at){T v;std::memcpy(&v,record(rt,at,sizeof(v)),sizeof(v));return v;}
uint32_t selections=0,lastSelection=0,lastBuild=0;   // diagnostics for the boundary messages
bool code(Runtime *rt,uint32_t lo,uint32_t hi,uint64_t expected){return bozo::hash::fnv1a64(record(rt,lo,hi-lo),hi-lo)==expected;}
// The battle UI family: the base flush 42d4c0 (vt+18; e.g. class 174300) or the texture set's
// 3cd0c0 (with its texture selection vt+a0 = 3cd330: 174e30, 175450, ...); each class has its
// own build method.
bool family(Runtime *rt,uint32_t object){const auto vt=value<uint32_t>(rt,object);if(vt<0x100000)return false;
    const auto flush=value<uint32_t>(rt,vt+0x18);
    return flush==0x42d4c0 || (flush==0x3cd0c0 && value<uint32_t>(rt,vt+0xa0)==0x3cd330);}
// The build methods' writer reset (27 USA sites, tools/vp2_bozo_kernel.py): lbu v1,0xbb(obj);
// li v0,1; li a1,0x44; sll v1,v1,2; addu v1,v1,obj; lw a0,0x90(v1); lw v1,(a0); addiu v1,v1,0x40;
// sw v1,8(a0); lw v1,8(a0); sw v1,0xc(a0); jal 120860; sb v0,0x15(a0) <- this delay slot.
// The object register (s0..s7) is read from the addu; where it is a0 (overwritten by the lw),
// the object is the method's own copy, its nearest `move sN,a0` back to the prologue. The
// object found is then checked by begin(): its +90 + 4 * +bb must be the writer reset here.
std::optional<unsigned> buildObjectRegister(Runtime *rt,uint32_t pc){
    static constexpr uint32_t tail[8]={0x8c640090,0x8c830000,0x24630040,0xac830008,0x8c830008,0xac83000c,0x0c048218,0xa0820015};
    for(unsigned i=0;i<8;++i)if(value<uint32_t>(rt,pc-0x1c+4*i)!=tail[i])return std::nullopt;
    const auto addu=value<uint32_t>(rt,pc-0x20);const unsigned reg=(addu>>16)&31;
    if(value<uint32_t>(rt,pc-0x24)!=0x00031880 || (addu&0xffe0ffffu)!=0x00601821)return std::nullopt;
    if(reg>=16 && reg<=23)return reg;
    if(reg!=4)return std::nullopt;
    for(uint32_t at=pc-0x28;at+0x200>pc;at-=4){const auto w=value<uint32_t>(rt,at);
        if((w&0xffff07ffu)==0x0080002d && ((w>>11)&31)>=16 && ((w>>11)&31)<=23)return (w>>11)&31;
        if((w>>16)==0x27bd && (w&0x8000))break;}   // addiu sp,sp,-n: the prologue
    return std::nullopt;
}
// The build writer: +90 + 4 * +bb (42db50 at 42db74, 430340, 4d4790).
uint32_t buildWriter(Runtime *rt,uint32_t object){return value<uint32_t>(rt,object+0x90+4u*value<uint8_t>(rt,object+0xbb));}
}
std::array<float,4> nativeBattleSpriteTexels(const std::array<uint16_t,4> &texels){
    std::array<float,4> st{};uint32_t flags=0;
    for(unsigned i=0;i<4;++i)st[i]=std::bit_cast<float>(bozo::ps2f::mul(0x397fffd5u,bozo::ps2f::fromInt(texels[i]),flags));
    return st;
}
void NativeBattleSprites::begin(Runtime *rt,uint32_t object,uint32_t writer){
    for(const auto &[stack,lease]:leases)if(lease.writer==writer)throw std::runtime_error("native battle sprite writer rebuilt during its flush");
    const auto base=value<uint32_t>(rt,writer);
    if(!family(rt,object) || buildWriter(rt,object)!=writer ||
       value<uint32_t>(rt,writer+8)!=base+0x40 || value<uint32_t>(rt,writer+12)!=base+0x40)
        throw std::runtime_error("native battle sprite build escaped its writer reset");
    builds[writer]=NativeBattleSpriteBuild{.object=object,.begin=base+0x40};lastBuild=writer;
}
void NativeBattleSprites::texture(uint32_t writer,const NativeBattleTexture &texture){
    auto it=builds.find(writer);if(it==builds.end())throw std::runtime_error(std::format("native battle texture selection has no sprite build (writer {:x})",writer));
    if(!texture.pixels || texture.pixels>=0x4000 || !texture.palette || texture.palette>=0x4000 || texture.csa>=16 || (texture.bytes!=32 && texture.bytes!=64) || it->second.textures.size()>=256)
        throw std::runtime_error("native battle texture selection outside admitted range");
    it->second.textures.push_back(texture);++selections;lastSelection=writer;
}
void NativeBattleSprites::sprite(Runtime *rt,const R5900Context *c){
    // 42ddd8 (both corner paths joined): s3 object, s2 descriptor, s0 writer, v1 = descriptor +20
    // (image record or 0); first corner (a2, t2), second (a1, t1), 12.4 fixed; z = a3.
    const auto object=c->r[19].uw[0],desc=c->r[18].uw[0],writer=c->r[16].uw[0],image=value<uint32_t>(rt,desc+0x20);
    auto it=builds.find(writer);
    if(it==builds.end() || it->second.object!=object || buildWriter(rt,object)!=writer || c->r[3].uw[0]!=image)
        throw std::runtime_error(std::format("native battle sprite has no sprite build (object {:x} writer {:x})",object,writer));
    auto &build=it->second;
    if(build.textures.empty())throw std::runtime_error(std::format("native battle sprite drawn before a texture selection (writer {:x}, {} selections so far, last in {:x}, last build {:x})",
        writer,selections,lastSelection,lastBuild));
    if(build.sprites.size()>=4096)throw std::runtime_error("native battle sprite count outside admitted range");
    const int32_t x0=c->r[6].sw[0],y0=c->r[10].sw[0],x1=c->r[5].sw[0],y1=c->r[9].sw[0];const uint32_t z=c->r[7].uw[0];
    // XYZ2 packs x | y << 16 from sign-extended words: only 16-bit corners reach the GS intact.
    for(int32_t v:{x0,y0,x1,y1})if(v<0 || v>0xffff)throw std::runtime_error("native battle sprite corner outside the GS range");
    if(z>0xffffff)throw std::runtime_error("native battle sprite depth outside 24 bits");
    std::array<float,4> uv{};
    if((value<uint8_t>(rt,desc+0x40)&2) || !image)for(unsigned i=0;i<4;++i)uv[i]=value<float>(rt,desc+0x30+4*i);
    else uv=nativeBattleSpriteTexels({value<uint16_t>(rt,image),value<uint16_t>(rt,image+2),value<uint16_t>(rt,image+4),value<uint16_t>(rt,image+6)});
    for(float f:uv)if((std::bit_cast<uint32_t>(f)&0x7f800000)==0x7f800000)throw std::runtime_error("nonfinite native battle sprite ST");
    NativeUiSprite s{.sprite=desc,.colour=value<uint32_t>(rt,desc+0x24),.depth=z,
        .fixedCorners={std::array<int32_t,2>{x0,y0},{x1,y0},{x0,y1},{x1,y1}},.uv=uv};
    build.sprites.push_back(std::move(s));build.spriteTextures.push_back(uint32_t(build.textures.size()-1));
}
NativeBattleSpriteBuild NativeBattleSprites::flush(Runtime *rt,uint32_t stack,uint32_t object,uint32_t writer,const ImageLookup &image){
    if(leases.contains(stack))throw std::runtime_error("native battle sprite flush stack reused");
    auto it=builds.find(writer);
    if(it==builds.end() || it->second.object!=object || !family(rt,object))
        throw std::runtime_error(std::format("native battle sprite flush has no build (object {:x} writer {:x})",object,writer));
    const auto &build=it->second;
    const auto base=value<uint32_t>(rt,writer),begin=value<uint32_t>(rt,writer+12),end=value<uint32_t>(rt,writer+8);
    // The whole writer is ALPHA_2 (32 bytes), each selection's TEX0_2 writes and one sprite
    // packet per recorded sprite (64): anything else in it has another, unported producer.
    uint64_t expected=32+64ull*build.sprites.size();for(const auto &t:build.textures)expected+=t.bytes;
    if(begin!=build.begin || end<begin || end-begin!=expected || uint64_t(end)+16>uint64_t(base)+uint64_t(value<uint32_t>(rt,writer+16))*8)
        throw std::runtime_error(std::format("native battle sprite writer has unowned content (writer {:x}: {} bytes, {} textures and {} sprites recorded)",
            writer,int64_t(end)-int64_t(begin),build.textures.size(),build.sprites.size()));
    // Each selection's image, as published by then (the texture set's upload).
    NativeBattleSpriteBuild resolved=build;std::vector<std::shared_ptr<const bozo::render::Texture>> pictures;
    for(const auto &t:build.textures){auto asset=image(t.pixels,false);
        if(!asset || image(t.palette,true)!=asset)
            throw std::runtime_error(std::format("native battle sprites name no published image (pixels {:x} palette {:x})",t.pixels,t.palette));
        pictures.push_back(asset->picture(t.csa));}
    for(size_t i=0;i<resolved.sprites.size();++i)resolved.sprites[i].picture=pictures.at(resolved.spriteTextures.at(i));
    leases.emplace(stack,Lease{object,writer,begin,end});return resolved;
}
void NativeBattleSprites::retire(Runtime *rt,const R5900Context *c){
    auto &lease=leases.at(c->r[29].uw[0]+0x30);
    if(lease.retired || c->r[17].uw[0]!=lease.writer || c->r[9].uw[0]!=lease.object+0xc0 ||
       c->r[16].uw[0]!=lease.end+16 || c->r[2].uw[0]!=lease.begin || c->r[5].uw[0]!=2 || c->r[6].uw[0] || c->r[8].uw[0]!=(lease.end+16-lease.begin)/16 ||
       value<uint32_t>(rt,lease.writer+8)!=lease.end || value<uint32_t>(rt,lease.writer+12)!=lease.begin)
        throw std::runtime_error("native battle sprites changed during the original flush");
    lease.retired=true;
}
void NativeBattleSprites::flushed(Runtime *rt,uint32_t stack){
    auto &lease=leases.at(stack);
    if(!lease.retired || lease.flushed || value<uint32_t>(rt,lease.writer+8)!=lease.end+16 || value<uint32_t>(rt,lease.writer+12)!=lease.end+16)
        throw std::runtime_error("native battle sprites did not finish their scalar flush");
    lease.flushed=true;
}
void NativeBattleSprites::complete(uint32_t stack){
    auto it=leases.find(stack);if(it==leases.end() || !it->second.flushed)throw std::runtime_error("native battle sprite flush returned before it finished");
    leases.erase(it);
}
void NativeBattleSprites::serialize(bozo::state::Archive &a){
    if(!leases.empty())throw std::runtime_error("native battle sprite state saved during a flush");
    state::map(a,builds,[&](NativeBattleSpriteBuild &b){
        bozo::state::fields(a,b.object,b.begin);
        auto n=bozo::state::count(a,b.textures.size());if(a.loading())b.textures.resize(n);
        for(auto &t:b.textures)bozo::state::fields(a,t.pixels,t.palette,t.csa,t.bytes);
        n=bozo::state::count(a,b.spriteTextures.size());if(a.loading())b.spriteTextures.resize(n);
        for(auto &t:b.spriteTextures)bozo::state::io(a,t);
        n=bozo::state::count(a,b.sprites.size());if(a.loading())b.sprites.resize(n);
        for(auto &s:b.sprites)bozo::state::fields(a,s.sprite,s.colour,s.depth,s.fixedCorners,s.uv);});
}
std::optional<bool> nativeBattleSpriteInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    static constexpr uint32_t buildSites[]={0x372a64,0x3752c8,0x3799a0,0x38b7e4,0x44f558,0x454778,0x455738,0x456594,0x45a2a8,0x45a834,0x466d34,0x468244,0x46ae24,
        0x46dd34,0x477424,0x477744,0x4bc3f4,0x4bebe4,0x4d47d8,0x4d6b4c,0x4d87a4,0x4da7b4,0x4dc258,0x4ee7a4,0x4f16d4,0x4f623c,0x4fb8a4};
    const bool buildSite=std::find(std::begin(buildSites),std::end(buildSites),pc)!=std::end(buildSites);
    if(!buildSite && pc!=0x4f0f20 && pc!=0x3cd468 && pc!=0x3cd3c8 && pc!=0x3cd4a0 && pc!=0x42ddd8 && pc!=0x3cd218 && pc!=0x3cd21c && pc!=0x42d4f0 && pc!=0x42d4f4 && pc!=0x120f4c && pc!=0x120f58)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<BattleSpriteOwner *>(&d->host()):nullptr;
    if(!owner || !c){if(pc==0x120f4c || pc==0x120f58)return std::nullopt;throw std::runtime_error("native battle sprites require an owner");}
    auto &sprites=owner->nativeBattleSprites();const auto sp=c->r[29].uw[0];
    if(pc==0x120f4c || pc==0x120f58){
        if(!sprites.active(sp+0x30))return std::nullopt;
        if(pc==0x120f4c)sprites.retire(rt,c);else sprites.flushed(rt,sp+0x30);
        return false;
    }
    if(buildSite){
        // A build method's writer reset (delay slot of its 120860 call): begin = end = base + 0x40;
        // the call writes ALPHA_2 = 0x44 (standard alpha blending).
        const auto reg=buildObjectRegister(rt,pc);
        if(!reg || c->r[5].ud[0]!=0x44)throw std::runtime_error(std::format("native battle sprite build {:x} differs from the USA writer reset",pc));
        sprites.begin(rt,c->r[*reg].uw[0],c->r[4].uw[0]);
    }else if(pc==0x4f0f20){
        // Class 175450's helper 4f0e90 selects the battle's UI sheet itself (delay slot of its second
        // 120740 call, s1 = writer): a palette-load TEX0_2 and a 512x256 PSMT4HH TEX0_2, blocks from
        // the battle layout table *(36d880) + 0x68 (+fc pixels, +100 palette), as the battle image
        // loader publishes them; palette part 0.
        if(!code(rt,0x4f0e90,0x4f1008,0x5833ee607231dd30ull))throw std::runtime_error("native battle UI sheet selection differs from the USA helper");
        const auto table=value<uint32_t>(rt,value<uint32_t>(rt,0x36d880)+0x68);
        const uint32_t pixels=value<uint16_t>(rt,table+0xfc),palette=value<uint16_t>(rt,table+0x100);
        if(c->r[19].uw[0]!=pixels)throw std::runtime_error("native battle UI sheet selection lost its layout table");
        sprites.texture(c->r[17].uw[0],{pixels,palette,0,64});
    }else if(pc==0x3cd468 || pc==0x3cd3c8 || pc==0x3cd4a0){
        // Texture selection 3cd330 (vt+a0): s3 writer, s0 = the texture cache's TEX0 for the
        // image (*(35de38) + 0x1010 + 16 * slot), s1 = the image's record (+2 = palette block).
        if(!code(rt,0x3cd330,0x3cd510,0x11afb988b5340bbcull))throw std::runtime_error("native battle texture selection differs from the USA method");
        const uint64_t tex=c->r[16].ud[0];
        const auto rec=c->r[17].uw[0];const uint64_t palette=value<uint16_t>(rt,rec+2);
        if(pc==0x3cd468){
            // 8-bit (3cd468..3cd494): one TEX0_2 = s0 | CBP << 37 | TCC | CLD 4 | PSM 13 bits.
            const uint64_t tex0=tex|palette<<37;
            if(((tex>>20)&0x3f)!=0x13)throw std::runtime_error("native battle texture selection lost its 8-bit format");
            sprites.texture(c->r[19].uw[0],{uint32_t(tex0&0x3fff),uint32_t(tex0>>37)&0x3fff,uint32_t(tex0>>56)&0x1f,32});
        }else if(pc==0x3cd3c8){
            // 4-bit (3cd3c8..3cd458): a palette-load TEX0_2, then s0 | the record's fields
            // (+0 TBP, +4 TBW/2, +5 PSM, +6 TW, +7 TH, +2 CBP), TCC, and CSA = t0 (s2) << 56.
            const uint64_t tex0=tex|uint64_t(c->r[18].sw[0])<<56|palette<<37|4ull<<32|uint64_t(value<uint8_t>(rt,rec+7))<<30|uint64_t(value<uint8_t>(rt,rec+6))<<26|
                uint64_t(value<uint8_t>(rt,rec+5))<<20|uint64_t(value<uint8_t>(rt,rec+4))<<15|value<uint16_t>(rt,rec);
            if(((tex0>>20)&0x3f)!=0x14)throw std::runtime_error("native battle texture selection lost its 4-bit format");
            sprites.texture(c->r[19].uw[0],{uint32_t(tex0&0x3fff),uint32_t(tex0>>37)&0x3fff,uint32_t(tex0>>56)&0x1f,64});
        }else throw std::runtime_error(std::format("unported native battle texture format {:x} (writer {:x})",(tex>>20)&0x3f,c->r[19].uw[0]));
    }else if(pc==0x42ddd8){
        // Shared sprite writer 42db50; its REGLIST tag comes from 50d450.
        if(!code(rt,0x42db50,0x42df80,0x2b4a98f8aa26ef0full) || value<uint64_t>(rt,0x50d450)!=0x6400000000000001ull || value<uint64_t>(rt,0x50d458)!=0x525210ull)
            throw std::runtime_error("native battle sprite writer differs from the USA one");
        sprites.sprite(rt,c);
    }else if(pc==0x3cd218 || pc==0x42d4f0){
        // Delay slot of the flushes' 120eb0 call (3cd0c0 at 3cd214, object s5; the base 42d4c0 at
        // 42d4ec, object a0): the writer +90 + 4 * +bc (v0 = object + 4 * +bc) with owner +c0.
        const auto object=c->r[pc==0x3cd218?21:4].uw[0];
        if(value<uint32_t>(rt,pc-4)!=0x0c0483ac || value<uint32_t>(rt,pc)!=0x8c440090 || c->r[2].uw[0]!=object+4u*value<uint8_t>(rt,object+0xbc) || c->r[5].uw[0]!=object+0xc0 ||
           (pc==0x42d4f0 && !code(rt,0x42d4c0,0x42d500,0xd1f5f3664cbc85a7ull)))
            throw std::runtime_error("native battle sprite flush differs from the USA submit");
        auto *textures=dynamic_cast<TextureOwner *>(&d->host());if(!textures)throw std::runtime_error("native battle sprites require a texture owner");
        const auto build=sprites.flush(rt,sp,object,value<uint32_t>(rt,c->r[2].uw[0]+0x90),
            [&](uint32_t block,bool palette){return textures->nativeTextures().blockAsset(block,palette);});
        try{owner->drawNativeBattleSprites(build);}catch(...){sprites.cancel(sp);throw;}
        static unsigned traced=0;
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES") && ++traced<=4)
            std::fprintf(stderr,"[vp2s:battle] native sprites object=%x sprites=%zu textures=%u\n",object,build.sprites.size(),unsigned(build.textures.size()));
    }else sprites.complete(sp);   // 3cd21c / 42d4f4, the flush's return
    return false; // The original instructions and the writer's scalar flush remain.
}
}

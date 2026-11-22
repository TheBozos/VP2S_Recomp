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
// The build writer: +90 + 4 * +bb (42db50 at 42db74, 430340, 4d4790).
uint32_t buildWriter(Runtime *rt,uint32_t object){return value<uint32_t>(rt,object+0x90+4u*value<uint8_t>(rt,object+0xbb));}
}
std::array<float,4> nativeBattleSpriteTexels(const std::array<uint16_t,4> &texels){
    std::array<float,4> st{};uint32_t flags=0;
    for(unsigned i=0;i<4;++i)st[i]=std::bit_cast<float>(bozo::ps2f::mul(0x397fffd5u,bozo::ps2f::fromInt(texels[i]),flags));
    return st;
}
bool NativeBattleSprites::begin(Runtime *rt,uint32_t writer){
    // Writers elsewhere in the game may live in the scratchpad (0x70000000): only EE RAM ones,
    // cached or uncached, can be battle UI builds.
    const auto seg=writer>>28,at=writer&0x1ffffff;
    if((seg!=0 && seg!=2 && seg!=3) || at<0x100000 || at>0x1fffff0)return false;
    for(const auto &[stack,lease]:leases)if(lease.writer==writer)throw std::runtime_error("native battle sprite writer rebuilt during its flush");
    const auto base=value<uint32_t>(rt,writer);
    if(value<uint32_t>(rt,writer+8)!=base+0x40 || value<uint32_t>(rt,writer+12)!=base+0x40)return false;   // not a reset writer
    builds[writer]=NativeBattleSpriteBuild{.begin=base+0x40};lastBuild=writer;return true;
}
bool NativeBattleSprites::blend(Runtime *rt,uint32_t writer,uint64_t alpha){
    const auto it=builds.find(writer);if(it==builds.end())return false;
    auto &build=it->second;
    if(value<uint32_t>(rt,writer+12)!=build.begin || uint64_t(value<uint32_t>(rt,writer+8))!=build.begin+build.accountedBytes())return false;   // not this build's open content
    for(const auto &[stack,lease]:leases)if(lease.writer==writer)throw std::runtime_error("native battle sprite blend changed during its flush");
    if(alpha!=0x44 && alpha!=0x148)throw std::runtime_error(std::format("native battle sprite build {:x} has an unported ALPHA_2 {:x}",writer,alpha));
    build.additive=alpha==0x148;++build.blendWrites;return true;
}
NativeBattleSpriteBuild &NativeBattleSprites::bind(Runtime *rt,uint32_t object,uint32_t writer){
    // The build's object: the first sprite, polygon or flush names it; it must be of the battle
    // UI family and own this writer as its +bb build writer (or, at the flush, its +bc one).
    auto it=builds.find(writer);
    if(it==builds.end() || (it->second.object && it->second.object!=object) || !family(rt,object))
        throw std::runtime_error(std::format("native battle sprite writer has no build for its object (object {:x} writer {:x})",object,writer));
    it->second.object=object;return it->second;
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
    if(buildWriter(rt,object)!=writer || c->r[3].uw[0]!=image)
        throw std::runtime_error(std::format("native battle sprite has no sprite build (object {:x} writer {:x})",object,writer));
    auto &build=bind(rt,object,writer);
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
    s.additive=build.additive;build.sprites.push_back(std::move(s));build.spriteTextures.push_back(uint32_t(build.textures.size()-1));
}
void NativeBattleSprites::strip(Runtime *rt,const Strip &p){
    // A 4-corner strip after the header 42ec70 chose from the same mode bits, at the writer's
    // open end (alpha-blended triangle strips): bit 4 = Gouraud (each corner's colour from
    // the descriptor's colours + 4 * corner), bit 8 = textured (ST with Q = 1, the writer's
    // current texture selection).
    //   flat     tag 50d440: PRIM 244, RGBAQ, XYZ2 x4                      64 bytes
    //   Gouraud  tag 50d470: PRIM 24c, (RGBAQ, XYZ2) x4, NOP              96
    //   textured tag 50d460: PRIM 254, RGBAQ, (ST, XYZ2) x4                96
    //   both     tag 50d480: PRIM 25c, (RGBAQ, ST, XYZ2) x4, NOP          128
    // Triangles come from 42fec0 (3 corners, its own header, untextured only):
    //   flat     tag 50d420: PRIM 243, RGBAQ, XYZ2 x3, NOP                 64
    //   Gouraud  tag 50d430: PRIM 24b, (RGBAQ, XYZ2) x3, NOP               80
    // drawn as a strip whose fourth corner repeats the third (an empty second triangle).
    const auto writer=buildWriter(rt,p.object);
    auto &build=bind(rt,p.object,writer);
    const bool gouraud=p.mode&4,textured=p.mode&8;
    if(p.triangle && textured)throw std::runtime_error(std::format("unported native battle textured triangle (object {:x})",p.object));
    static constexpr uint64_t tags[4][3]={{0x6400000000000001ull,0x555510ull,0x244},{0xa400000000000001ull,0xf515151510ull,0x24c},
                                          {0xa400000000000001ull,0x5252525210ull,0x254},{0xe400000000000001ull,0xf5215215215210ull,0x25c}};
    static constexpr uint64_t triangleTags[2][3]={{0x6400000000000001ull,0xf55510ull,0x243},{0x8400000000000001ull,0xf5151510ull,0x24b}};
    static constexpr uint32_t sizes[4]={64,96,96,128},triangleSizes[2]={64,80};
    const auto &tag=p.triangle?triangleTags[gouraud]:tags[gouraud+2*textured];
    if(p.data-0x18!=value<uint32_t>(rt,writer+8) || value<uint64_t>(rt,p.data-0x18)!=tag[0] || value<uint64_t>(rt,p.data-0x10)!=tag[1] || value<uint64_t>(rt,p.data-8)!=tag[2])
        throw std::runtime_error("native battle polygon header differs from its strip mode");
    if(textured && build.textures.empty())throw std::runtime_error(std::format("native battle polygon textured before a texture selection (writer {:x})",writer));
    if(build.sprites.size()>=4096)throw std::runtime_error("native battle sprite count outside admitted range");
    if(p.z>0xffffff)throw std::runtime_error("native battle polygon depth outside 24 bits");
    NativeUiSprite s{.sprite=p.desc,.colour=value<uint32_t>(rt,p.colours),.depth=p.z,.gouraud=gouraud};
    // A triangle repeats its third corner as the fourth: the strip's second triangle is empty.
    if(gouraud)for(unsigned i=0;i<4;++i)s.cornerColours[i]=value<uint32_t>(rt,p.colours+4*(p.triangle && i==3?2:i));
    if(textured){
        // ST: from the image record (corners (u0,v0), (u1,v0), (u0,v1), (u1,v1) of its four
        // halfwords, 0x397fffd5 * texel) unless bit 2 is set or there is none: then the
        // descriptor's corner floats (s, t per corner), admitted as an axis-aligned rectangle.
        if(p.image && !(p.mode&2))s.uv=nativeBattleSpriteTexels({value<uint16_t>(rt,p.image),value<uint16_t>(rt,p.image+2),value<uint16_t>(rt,p.image+4),value<uint16_t>(rt,p.image+6)});
        else{float st[4][2];for(unsigned i=0;i<4;++i)for(unsigned j=0;j<2;++j)st[i][j]=value<float>(rt,p.st+8*i+4*j);
            if(st[0][0]!=st[2][0] || st[1][0]!=st[3][0] || st[0][1]!=st[1][1] || st[2][1]!=st[3][1])
                throw std::runtime_error(std::format("unported native battle polygon with free corner ST (object {:x})",p.object));
            s.uv={st[0][0],st[0][1],st[3][0],st[3][1]};}
        for(float f:s.uv)if((std::bit_cast<uint32_t>(f)&0x7f800000)==0x7f800000)throw std::runtime_error("nonfinite native battle polygon ST");
    }
    // XYZ2 = x | y << 16 from sign-extended words: only 16-bit fixed corners reach the GS intact.
    for(const auto &corner:p.corners)for(int32_t v:corner)if(v<0 || v>0xffff)throw std::runtime_error("native battle polygon corner outside the GS range");
    s.fixedCorners=p.corners;   // strip order: triangles (0,1,2) and (1,2,3)
    s.additive=build.additive;build.sprites.push_back(std::move(s));
    build.spriteTextures.push_back(textured?uint32_t(build.textures.size()-1):NativeBattleSpriteBuild::kUntextured);
    build.polygonBytes+=p.triangle?triangleSizes[gouraud]:sizes[gouraud+2*textured];++build.polygons;
}
void NativeBattleSprites::polygon(Runtime *rt,const R5900Context *c){
    // 42e144, after 42df80's unrotated corner loop: s6 object, s5 descriptor, s4 corner count,
    // s0 = descriptor +78 (mode), s3 = the packet after the header, the corners at sp+e0 as
    // screen pixels (int x, y); depth +74, colours +40, image record +70, corner ST +50.
    const auto desc=c->r[21].uw[0],count=c->r[20].uw[0],mode=c->r[16].uw[0]&0xff;
    Strip p{.object=c->r[22].uw[0],.desc=desc,.mode=mode,.data=c->r[19].uw[0],.z=value<uint32_t>(rt,desc+0x74),
            .colours=desc+0x40,.st=desc+0x50,.image=value<uint32_t>(rt,desc+0x70)};
    if(value<uint8_t>(rt,desc+0x78)!=mode)throw std::runtime_error(std::format("native battle polygon mode differs from its descriptor (object {:x})",p.object));
    // 3 corners: the triangle writer 42fec0 (its header 50d420 / 50d430, see strip); the zero
    // word after the corners (42e1c8 / 42e23c) is its header's NOP.
    if(count!=4 && count!=3)throw std::runtime_error(std::format("unported native battle polygon with {} corners (object {:x})",count,p.object));
    p.triangle=count==3;
    const auto sp=c->r[29].uw[0];
    for(unsigned i=0;i<count;++i){const auto x=value<int32_t>(rt,sp+0xe0+8*i),y=value<int32_t>(rt,sp+0xe4+8*i);
        if(x<0 || x>0xfff || y<0 || y>0xfff)throw std::runtime_error("native battle polygon corner outside the GS range");
        p.corners[i]={x*16,y*16};}
    if(p.triangle)p.corners[3]=p.corners[2];
    strip(rt,p);
}
void NativeBattleSprites::band(Runtime *rt,const R5900Context *c){
    // 383cb0(object, index) draws a band of two rows of four points as three alpha-blended
    // Gouraud strips, after one reservation (120fa0) at the writer's open end: each is the
    // object's GIF tag (+d0/+d8: REGLIST, PRIM, (RGBAQ, XYZ2) x 4, NOP), PRIM 24c, then per
    // corner RGBA (only the low word is stored: Q keeps the buffer's old bytes, unused without
    // a texture) and XYZ2 = 16 x | 16 y << 16 | fff4dd << 32, then the NOP. At 3843a0 (s4
    // object, s3 writer, v0 the packet's end) the strips are read back as the game wrote them.
    const auto object=c->r[20].uw[0],writer=c->r[19].uw[0],end=c->r[2].uw[0];
    if(writer!=buildWriter(rt,object))throw std::runtime_error(std::format("native battle band writer is not its object's (object {:x})",object));
    auto &build=bind(rt,object,writer);
    constexpr uint32_t kStrip=96;const auto start=value<uint32_t>(rt,writer+8);
    if(end!=start+3*kStrip)throw std::runtime_error(std::format("native battle band packet is not three strips (object {:x})",object));
    if(value<uint64_t>(rt,object+0xd0)!=0xa400000000000001ull || value<uint64_t>(rt,object+0xd8)!=0xf515151510ull)
        throw std::runtime_error(std::format("native battle band tag differs from the Gouraud strip (object {:x})",object));
    if(build.sprites.size()+3>4096)throw std::runtime_error("native battle sprite count outside admitted range");
    std::array<NativeUiSprite,3> strips{};   // all three are checked before any is recorded
    for(unsigned k=0;k<3;++k){const auto at=start+kStrip*k;
        if(value<uint64_t>(rt,at)!=0xa400000000000001ull || value<uint64_t>(rt,at+8)!=0xf515151510ull || value<uint64_t>(rt,at+16)!=0x24cull || value<uint64_t>(rt,at+88)!=0)
            throw std::runtime_error(std::format("native battle band strip {} differs from its tag (object {:x})",k,object));
        auto &s=strips[k];s=NativeUiSprite{.sprite=object,.colour=value<uint32_t>(rt,at+24),.depth=0xfff4dd,.gouraud=true};
        for(unsigned i=0;i<4;++i){const auto xyz=value<uint64_t>(rt,at+32+16*i);
            if((xyz>>32)!=0xfff4dd)throw std::runtime_error("native battle band depth differs from its writer's");
            s.cornerColours[i]=value<uint32_t>(rt,at+24+16*i);s.fixedCorners[i]={int32_t(xyz&0xffff),int32_t((xyz>>16)&0xffff)};}}
    for(auto &s:strips){s.additive=build.additive;build.sprites.push_back(std::move(s));build.spriteTextures.push_back(NativeBattleSpriteBuild::kUntextured);
        build.polygonBytes+=kStrip;++build.polygons;}
}
void NativeBattleSprites::flatQuad(Runtime *rt,uint32_t writer,uint32_t end){
    // 120a80(writer, x0, y0, x1, y1, z t0, ..., colour t2) builds the tag bit by bit (REGLIST,
    // NLOOP 1, 6 registers 555510: PRIM, RGBAQ, XYZ2 x4; the same as 50d440), PRIM 0x204 or
    // 0x244 with writer +15 (alpha blending), stores only the RGBA word (Q keeps stale bytes),
    // and four XYZ2 = (0x800 - frame size / 2 + coordinate) << 4 with z << 32. The battle's
    // gauge bars (class 16f0a0's build 38d5e0) draw through it.
    auto it=builds.find(writer);if(it==builds.end())return;
    auto &build=it->second;const auto at=value<uint32_t>(rt,writer+8);
    if(end!=at+64)throw std::runtime_error(std::format("native battle flat rectangle is not one strip (writer {:x})",writer));
    if(value<uint64_t>(rt,at)!=0x6400000000000001ull || value<uint64_t>(rt,at+8)!=0x555510ull || value<uint64_t>(rt,at+16)!=0x244ull)
        throw std::runtime_error(std::format("unported native battle flat rectangle form (writer {:x}, PRIM {:x})",writer,value<uint64_t>(rt,at+16)));
    if(build.sprites.size()>=4096)throw std::runtime_error("native battle sprite count outside admitted range");
    const auto z=uint32_t(value<uint64_t>(rt,at+32)>>32);if(z>0xffffff)throw std::runtime_error("native battle flat rectangle depth outside 24 bits");
    NativeUiSprite s{.sprite=writer,.colour=value<uint32_t>(rt,at+24),.depth=z};
    for(unsigned i=0;i<4;++i){const auto xyz=value<uint64_t>(rt,at+32+8*i);
        if(uint32_t(xyz>>32)!=z)throw std::runtime_error("native battle flat rectangle corners differ in depth");
        s.fixedCorners[i]={int32_t(xyz&0xffff),int32_t((xyz>>16)&0xffff)};}
    s.additive=build.additive;build.sprites.push_back(std::move(s));build.spriteTextures.push_back(NativeBattleSpriteBuild::kUntextured);
    build.polygonBytes+=64;++build.polygons;
}
void NativeBattleSprites::rectangle(Runtime *rt,const R5900Context *c){
    // 42efe0, where 42ed50's two unrotated paths join: s7 object, s5 descriptor, s0 =
    // descriptor +6c (mode), s4 = the packet after the header, s3 = depth (descriptor +8 as
    // an integer), s2 = image record (+68); 12.4 edges x0 = v1, y0 = t0, x1 = v0, y1 = a0.
    // Colours +30, corner ST +40.
    const auto desc=c->r[21].uw[0],mode=c->r[16].uw[0]&0xff;
    Strip p{.object=c->r[23].uw[0],.desc=desc,.mode=mode,.data=c->r[20].uw[0],.z=c->r[19].uw[0],
            .colours=desc+0x30,.st=desc+0x40,.image=c->r[18].uw[0]};
    if(value<uint8_t>(rt,desc+0x6c)!=mode || value<uint32_t>(rt,desc+0x68)!=p.image)
        throw std::runtime_error(std::format("native battle rectangle differs from its descriptor (object {:x})",p.object));
    const int32_t x0=c->r[3].sw[0],y0=c->r[8].sw[0],x1=c->r[2].sw[0],y1=c->r[4].sw[0];
    p.corners={std::array<int32_t,2>{x0,y0},{x1,y0},{x0,y1},{x1,y1}};
    strip(rt,p);
}
NativeBattleSpriteBuild NativeBattleSprites::flush(Runtime *rt,uint32_t stack,uint32_t object,uint32_t writer,const ImageLookup &image){
    if(leases.contains(stack))throw std::runtime_error("native battle sprite flush stack reused");
    if(!builds.contains(writer))throw std::runtime_error(std::format("native battle sprite flush has no build (object {:x} writer {:x})",object,writer));
    const auto &build=bind(rt,object,writer);
    const auto base=value<uint32_t>(rt,writer),begin=value<uint32_t>(rt,writer+12),end=value<uint32_t>(rt,writer+8);
    // The whole writer is ALPHA_2 (32 bytes), each selection's TEX0_2 writes, one sprite packet
    // per recorded sprite (64) and the polygons' packets: anything else has an unported producer.
    const uint64_t expected=build.accountedBytes();
    if(begin!=build.begin || end<begin || end-begin!=expected || uint64_t(end)+16>uint64_t(base)+uint64_t(value<uint32_t>(rt,writer+16))*8)
        throw std::runtime_error(std::format("native battle sprite writer has unowned content (writer {:x}: {} bytes, {} textures and {} sprites recorded)",
            writer,int64_t(end)-int64_t(begin),build.textures.size(),build.sprites.size()));
    // Each selection's image, as published by then (the texture set's upload).
    NativeBattleSpriteBuild resolved=build;std::vector<std::shared_ptr<const bozo::render::Texture>> pictures;
    for(const auto &t:build.textures){auto asset=image(t.pixels,t.palette);
        if(!asset)
            throw std::runtime_error(std::format("native battle sprites name no published image (pixels {:x} palette {:x})",t.pixels,t.palette));
        pictures.push_back(asset->picture(t.csa));}
    for(size_t i=0;i<resolved.sprites.size();++i)if(const auto t=resolved.spriteTextures.at(i);t!=NativeBattleSpriteBuild::kUntextured)
        resolved.sprites[i].picture=pictures.at(t);
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
uint32_t NativeBattleSprites::complete(uint32_t stack){
    auto it=leases.find(stack);if(it==leases.end() || !it->second.flushed)throw std::runtime_error("native battle sprite flush returned before it finished");
    const auto object=it->second.object;leases.erase(it);return object;
}
void NativeBattleSprites::serialize(bozo::state::Archive &a){
    if(!leases.empty())throw std::runtime_error("native battle sprite state saved during a flush");
    state::map(a,builds,[&](NativeBattleSpriteBuild &b){
        bozo::state::fields(a,b.object,b.begin,b.polygonBytes,b.polygons);
        auto n=bozo::state::count(a,b.textures.size());if(a.loading())b.textures.resize(n);
        for(auto &t:b.textures)bozo::state::fields(a,t.pixels,t.palette,t.csa,t.bytes);
        n=bozo::state::count(a,b.spriteTextures.size());if(a.loading())b.spriteTextures.resize(n);
        for(auto &t:b.spriteTextures)bozo::state::io(a,t);
        n=bozo::state::count(a,b.sprites.size());if(a.loading())b.sprites.resize(n);
        for(auto &s:b.sprites)bozo::state::fields(a,s.sprite,s.colour,s.depth,s.fixedCorners,s.uv,s.gouraud,s.cornerColours);});
}
void NativeBattleSprites::serializeBlends(bozo::state::Archive &a){
    // Added after the "random" field: per build (in the same order as serialize), the later
    // ALPHA_2 writes, the current blend and each sprite's. Older states end before it: no build
    // in them had changed its blend (that was a boundary), so the defaults hold.
    for(auto &[writer,b]:builds){bozo::state::fields(a,b.blendWrites,b.additive);for(auto &s:b.sprites)bozo::state::io(a,s.additive);}
}
std::optional<bool> nativeBattleSpriteInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    static constexpr uint32_t pairSites[]={0x4bcd40,0x4bde08,0x4d0c34,0x4d1734,0x4d99ac,0x4ecf0c,0x4ef498,0x4f0f20,0x4f268c,0x4f2874,0x4fa19c,0x4fdcc0,0x503d50};
    const bool pairSite=std::find(std::begin(pairSites),std::end(pairSites),pc)!=std::end(pairSites);
    if(pc!=0x120860 && pc!=0x120c30 && !pairSite && pc!=0x3cd468 && pc!=0x3cd3c8 && pc!=0x3cd4a0 && pc!=0x42ddd8 && pc!=0x42e144 && pc!=0x42efe0 && pc!=0x3843a0 && pc!=0x42d4f0 && pc!=0x42d4f4 && pc!=0x120f4c && pc!=0x120f58)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<BattleSpriteOwner *>(&d->host()):nullptr;
    if(!owner || !c){if(pc==0x120860 || pc==0x120c30 || pc==0x120f4c || pc==0x120f58)return std::nullopt;throw std::runtime_error("native battle sprites require an owner");}
    auto &sprites=owner->nativeBattleSprites();const auto sp=c->r[29].uw[0];
    if(pc==0x120f4c || pc==0x120f58){
        if(!sprites.active(sp+0x30))return std::nullopt;
        if(pc==0x120f4c)sprites.retire(rt,c);else sprites.flushed(rt,sp+0x30);
        return false;
    }
    if(pc==0x120c30){
        // main 120a80's cursor commit (sw t4, 8(a0)): record its strip when the writer is a battle build.
        if(value<uint32_t>(rt,0x120c30)!=0xac8c0008u || value<uint32_t>(rt,0x120a80)!=0x27bdfff0u)return std::nullopt;
        sprites.flatQuad(rt,c->r[4].uw[0],c->r[12].uw[0]);
        return false;
    }
    if(pc==0x120860){
        // ALPHA_2 writer 120860 (writer a0, value a1), called game-wide. The battle UI build
        // methods reset their writer (+8 = +c = base + 0x40) and call it with 0x44 (standard
        // blending) before their selections and sprites: that call opens a build.
        // A later call on an open build (cursor at its recorded content) changes the blending
        // of the sprites that follow (0x148: additive, the attack trail's fading copies).
        if(!(c->r[5].ud[0]==0x44 && sprites.begin(rt,c->r[4].uw[0])))sprites.blend(rt,c->r[4].uw[0],c->r[5].ud[0]);
        return false;
    }else if(pairSite){
        // Selection pairs outside 3cd330 (13 USA sites, e.g. 175450's UI sheet helper 4f0e90 at
        // 4f0f20): a palette-load TEX0_2 (lui 130 ...; jal 120740; or a1,sX,v0) and then the
        // image's TEX0_2 (jal 120740; or a1,rs,rt <- this delay slot, a0 = writer): the battle's
        // 512x256 PSMT4HH UI sheet or an 8-bit PSMT8H image. The TEX0 is the delay slot's OR,
        // read from its two registers: pixels and palette blocks (as the battle image loader
        // publishes them), palette part.
        const auto ds=value<uint32_t>(rt,pc);
        bool first=false;for(uint32_t at=pc-0x80;at<pc-4;at+=4)if(value<uint32_t>(rt,at)==0x0c0481d0 && (value<uint32_t>(rt,at+4)&0xfc00ffffu)==0x00002825){first=true;break;}
        if(value<uint32_t>(rt,pc-4)!=0x0c0481d0 || (ds&0xfc00ffffu)!=0x00002825 || !first)
            throw std::runtime_error(std::format("native battle selection pair {:x} differs from the USA idiom",pc));
        const uint64_t tex0=c->r[(ds>>21)&31].ud[0]|c->r[(ds>>16)&31].ud[0];
        if(const auto psm=(tex0>>20)&0x3f;psm!=0x2c && psm!=0x1b)throw std::runtime_error(std::format("native battle selection pair {:x} has an unported format {:x}",pc,psm));
        sprites.texture(c->r[4].uw[0],{uint32_t(tex0&0x3fff),uint32_t(tex0>>37)&0x3fff,uint32_t(tex0>>56)&0x1f,64});
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
    }else if(pc==0x42e144 || pc==0x42efe0){
        // Triangles: 42fec0 writes their header (50d420 / 50d430) and calls 42df80 with 3 corners.
        if(pc==0x42e144 && c->r[20].uw[0]==3 && (!code(rt,0x42fec0,0x42ffe8,0xee6b3c0b773a0526ull) ||
           value<uint64_t>(rt,0x50d420)!=0x6400000000000001ull || value<uint64_t>(rt,0x50d428)!=0xf55510ull ||
           value<uint64_t>(rt,0x50d430)!=0x8400000000000001ull || value<uint64_t>(rt,0x50d438)!=0xf5151510ull))
            throw std::runtime_error("native battle triangle writer differs from the USA one");
        // Shared polygon writer 42df80 (callers pass the corner count) and rectangle writer
        // 42ed50, both after the header 42ec70 (tags 50d440..50d480).
        if(!code(rt,0x42df80,0x42ec20,0xf33b4cdd2a689d4eull) || !code(rt,0x42ec20,0x42ed50,0x6716509c287ca7c7ull) ||
           (pc==0x42efe0 && !code(rt,0x42ed50,0x42fec0,0x9d0d4c3128b2e310ull)) ||
           value<uint64_t>(rt,0x50d440)!=0x6400000000000001ull || value<uint64_t>(rt,0x50d448)!=0x555510ull ||
           value<uint64_t>(rt,0x50d470)!=0xa400000000000001ull || value<uint64_t>(rt,0x50d478)!=0xf515151510ull ||
           value<uint64_t>(rt,0x50d460)!=0xa400000000000001ull || value<uint64_t>(rt,0x50d468)!=0x5252525210ull ||
           value<uint64_t>(rt,0x50d480)!=0xe400000000000001ull || value<uint64_t>(rt,0x50d488)!=0xf5215215215210ull)
            throw std::runtime_error("native battle polygon writer differs from the USA one");
        if(pc==0x42e144)sprites.polygon(rt,c);else sprites.rectangle(rt,c);
    }else if(pc==0x3843a0){
        if(!code(rt,0x383cb0,0x3843f8,0x23a73ec964306dc5ull))throw std::runtime_error("native battle band writer differs from the USA one");
        sprites.band(rt,c);
    }else if(pc==0x42d4f0){
        // Delay slot of the base flush's 120eb0 call (42d4c0 at 42d4ec, object a0): the writer +90 +
        // 4 * +bc (v0 = object + 4 * +bc) with owner +c0. 3cd0c0's own flush (3cd214, +e8 set) is
        // patched into a call of 42d4c0(object), so both flushes arrive here.
        const auto object=c->r[4].uw[0];
        if(value<uint32_t>(rt,pc-4)!=0x0c0483ac || value<uint32_t>(rt,pc)!=0x8c440090 || c->r[2].uw[0]!=object+4u*value<uint8_t>(rt,object+0xbc) || c->r[5].uw[0]!=object+0xc0 ||
           (pc==0x42d4f0 && !code(rt,0x42d4c0,0x42d500,0xd1f5f3664cbc85a7ull)))
            throw std::runtime_error("native battle sprite flush differs from the USA submit");
        auto *textures=dynamic_cast<TextureOwner *>(&d->host());if(!textures)throw std::runtime_error("native battle sprites require a texture owner");
        const auto build=sprites.flush(rt,sp,object,value<uint32_t>(rt,c->r[2].uw[0]+0x90),
            [&](uint32_t pixels,uint32_t palette){return textures->nativeTextures().blockImage(pixels,palette);});
        try{owner->drawNativeBattleSprites(build);}catch(...){sprites.cancel(sp);throw;}
        static unsigned traced=0;
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES") && ++traced<=64)
            std::fprintf(stderr,"[vp2s:battle] native sprites object=%x sprites=%zu textures=%u\n",object,build.sprites.size(),unsigned(build.textures.size()));
    }else{   // 42d4f4, the flush's return
        const auto object=sprites.complete(sp);
        // The base flush's job (owner +c0) has no native queue to complete it: 42d4c0's return
        // (42d4f8) is patched to its completion 42d4a0 (owner thunk 44d5a0's target; the scene's
        // 21c9a0), entered with the object. (Without +e8, the 3cd0c0 family completes through its
        // texture set's patched enqueue, 3cc900; with it, through this flush.)
        if(value<uint32_t>(rt,0x42d4f8)!=0x03e00008u)throw std::runtime_error("native battle base flush return differs (the kernel patches its jr ra to j 42d4a0)");
        c->r[4].sd[0]=int32_t(object);
    }
    return false; // The original instructions and the writer's scalar flush remain.
}
}

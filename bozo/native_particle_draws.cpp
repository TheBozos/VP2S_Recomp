// Explicit point/kind records to native PC quads. No live command/header
// decoding: immutable material semantics are converted offline from the disc.
#include "native_cast.h"
#include "native_particle_draws.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/ops_vu0.h"
#include "bozo/runtime_api.h"
#include "../port/vp2s_music_math.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <format>
#include <fstream>
#include <limits>
namespace vp2s {
namespace {
uint8_t *record(Runtime *rt,uint32_t at,uint32_t bytes){
    if(!rt || (at&15) || at<0x100000 || at>=0x2000000 || uint64_t(at)+bytes>0x2000000)throw std::runtime_error("native particle draw record outside EE RAM");
    auto *p=rt->memory.hostPointer(at,bytes);if(!p)throw std::runtime_error("native particle draw record outside owned memory");return p;
}
template<class T>T value(const uint8_t *p,unsigned offset){T v;std::memcpy(&v,p+offset,sizeof(v));return v;}
uint32_t word(Runtime *rt,uint32_t at){
    if(!rt || at<0x100000 || uint64_t(at)+4>0x2000000)throw std::runtime_error("native particle draw pointer outside EE RAM");
    return value<uint32_t>(rt->memory.hostPointer(at,4),0);
}
float number(float v){const auto b=std::bit_cast<uint32_t>(v),m=b&0x7fffffff;
    if(m>=0x7f800000 || (m && m<0x00800000) || std::abs(v)>1.e8f)throw std::runtime_error(std::format("unqualified native particle draw input {:08x}",b));return v;
}
using Mat=std::array<float,16>;
Mat product(const Mat &a,const Mat &b){Mat out{};for(unsigned row=0;row<4;++row)for(unsigned column=0;column<4;++column)for(unsigned k=0;k<4;++k)out[row*4+column]+=a[row*4+k]*b[k*4+column];return out;}
std::array<float,4> transform(const Mat &m,const std::array<float,4> &p){std::array<float,4> out{};
    for(unsigned row=0;row<4;++row)for(unsigned i=0;i<4;++i)out[row]+=m[row*4+i]*p[i];return out;
}
}
const NativeParticleDraws::Material &NativeParticleDraws::material(const AssetOrigin &origin){
    if(origin.kind!=AssetOrigin::Kind::Part)throw std::runtime_error("native particle material lacks decoded part ownership");
    const auto key=std::pair{origin.disc,origin.part};auto found=materials.find(key);
    if(found==materials.end()){
        const auto file=root/std::format("{:010x}_{:03}.pmat",origin.disc,origin.part);std::ifstream in(file,std::ios::binary);
        const auto take=[&](){uint32_t v;if(!in.read(reinterpret_cast<char *>(&v),4))throw std::runtime_error("missing/truncated native particle material conversion: "+file.string());return v;};
        if(take()!=0x33505056)throw std::runtime_error("native particle material version mismatch");const auto count=take();if(count>65536)throw std::runtime_error("native particle material count outside bounds");
        std::map<uint32_t,Material> pack;
        for(unsigned i=0;i<count;++i){const auto offset=take();Material m;m.kinds=take();m.extent=take();m.textureAlpha=take();m.function=take();m.passes=take();m.second=take();m.stride=take();
            for(auto &blend:m.blends)for(auto &factor:blend)factor=take();
            if(!m.kinds || m.kinds>51 || m.extent!=0xc0+80*m.kinds || m.textureAlpha>1 || m.function>3 || !m.passes || m.passes>4 || !m.stride || m.stride>51)throw std::runtime_error("invalid native particle material semantics");
            for(const auto &b:m.blends){for(unsigned j=0;j<4;++j)if(b[j]>2)throw std::runtime_error("invalid native particle blend factor");if(b[4]>255)throw std::runtime_error("invalid native particle blend constant");}
            if(!pack.emplace(offset,m).second)throw std::runtime_error("duplicate native particle material offset");
        }
        if(in.peek()!=std::char_traits<char>::eof())throw std::runtime_error("trailing native particle material bytes");found=materials.emplace(key,std::move(pack)).first;
    }
    // kinds / stride up to 51: the template's kind UNPACK count is 5 * kinds in an 8-bit field.
    const auto record=found->second.find(origin.offset);if(record==found->second.end())throw std::runtime_error(std::format("native particle material not converted: {:x}/{}+{:x}",origin.disc,origin.part,origin.offset));return record->second;
}
NativeModelSubmission NativeParticleDraws::capture(Runtime *rt,NativeAssets &assets,NativeTextures &textures,const NativeScene &scene,const NativeParticles &particles,uint32_t object){
    music::CodecRounding arithmetic;particles.validateDraw(object);auto *owner=record(rt,object,0xb10);
    const auto camera=scene.drawingCamera();const auto target=scene.frameTarget();const auto current=word(rt,0x35ddf8);const unsigned layer=owner[0x65];
    if(!camera || !target || layer>1 || word(rt,current+0x40a8+4*layer)!=object || *rt->memory.hostPointer(current+0x40dc+layer,1)!=1 || !owner[0x63] || !particleClass(value<uint32_t>(owner,0)) || (owner[0xaff]>4 && owner[0xaff]!=6) || owner[0xb04] || owner[0xaf9]>1)
        throw std::runtime_error("native particle draw has no admitted scene/point owner");
    const auto first=NativeAssets::address(value<uint32_t>(owner,0xaa0)),second=NativeAssets::address(value<uint32_t>(owner,0xaa4));
    if(second<=first || second-first<0x20 || second-first>0x1000000)throw std::runtime_error("native particle draw buffer halves invalid");
    const auto span=second-first,index=owner[0xaf9]^1u,base=index?second:first,total=value<uint32_t>(owner,0xabc),chunks=value<uint32_t>(owner,0xac0+4*index);
    record(rt,first,span*2);if(total>65536 || chunks>65536 || bool(total)!=bool(chunks))throw std::runtime_error("native particle draw counts invalid");
    NativeModelSubmission submission{object,camera->object};if(!total)return submission;
    // Mode 2 selects VU1 image 5 (mode + 3): image 7's ribbons without a texture. It reads no kind template (+628 = 0)
    // and no FIS; its GS state is the owner's row block (+624): row 96 the A+D tag, row 97 ALPHA_1, row 102 the strip tag.
    // Mode 0 selects image 3: GS points of the same untextured kind (tag 0x2030400000008000: PRIM 0x60, FGE, ABE), or
    // without fog (0x2020400000008000: PRIM 0x40, ABE; field maps 61-93, 189-193: ALPHA 0x148, additive).
    // Mode 1 selects image 4: Gouraud line strips of the same kind (tag 0x2025400000008000: PRIM 0x4a, ABE).
    const bool points=owner[0xaff]==0,lineStrips=owner[0xaff]==1,plain=owner[0xaff]==2 || points || lineStrips;NativeModelBatch batch;auto &mat=batch.material;Material m{};const uint8_t *kindData=nullptr;
    if(plain){
        const auto block=value<uint32_t>(owner,0x624);if(!block)throw std::runtime_error("native particle draw has no GS row block (+624)");
        auto *rows=record(rt,block,0x100);
        // Battle 0x1c: tag 0x2036400000008000 / 0x41 (PRE, PRIM 0x6c: Gouraud strip, FGE, ABE, context 1; RGBAQ, XYZF2);
        // map 559 (story-gated arrival): 0x20264000, PRIM 0x4c without FGE: the GS never fogs it (its fog variant still runs).
        // The strip's F is 0xff (no fog); the program writes the tag's NLOOP / EOP itself.
        const auto tagHigh=value<uint32_t>(rows,0x74);
        // Line strips: 0x20254000 (PRIM 0x4a) or with FGE 0x20354000 (PRIM 0x6a; field maps 62, 288 with the fog variant), or
        // with FGE and without ABE 0x20154000 (PRIM 0x2a; field maps 46, 48): written opaque, as the points without ABE.
        const bool tagKnown=points?(tagHigh==0x20304000u || tagHigh==0x20204000u || tagHigh==0x20104000u):
                            lineStrips?(tagHigh==0x20254000u || tagHigh==0x20354000u || tagHigh==0x20154000u):(tagHigh==0x20364000u || tagHigh==0x20264000u);
        if(value<uint32_t>(rows,0xc)!=0x6c0f0060 || value<uint32_t>(rows,0x28)!=0x42 || !tagKnown || value<uint64_t>(rows,0x78)!=0x41)
            throw std::runtime_error(std::format("unported native particle image 3/5 GS state (ALPHA reg {:x}, tag {:08x}:{:x})",value<uint32_t>(rows,0x28),value<uint32_t>(rows,0x74),value<uint64_t>(rows,0x78)));
        const auto alpha=value<uint64_t>(rows,0x20);
        // ALPHA_1 from the block (battle 0x1c: 0x44, normal alpha; user 2026-11-24: "A: normal alpha").
        for(unsigned i=0;i<4;++i)if(((alpha>>(2*i))&3)>2)throw std::runtime_error("native particle image 5 ALPHA factor 3");
        // Points and line strips may also come without ABE (0x2010400000008000: PRIM 0x20, FGE only; field maps 182-188;
        // 0x20154000, PRIM 0x2a): written opaque.
        const bool abe=(tagHigh>>21)&1;
        mat.textured=false;mat.alphaBlend=abe;mat.blend=abe?bozo::render::Material::Blend::Alpha:bozo::render::Material::Blend::Opaque;mat.gsBlend=abe;
        mat.blendA=alpha&3;mat.blendB=(alpha>>2)&3;mat.blendC=(alpha>>4)&3;mat.blendD=(alpha>>6)&3;mat.blendFix=(alpha>>32)&0xff;
        mat.depthTest=true;mat.depthWrite=false;mat.alphaTest=false;
    }else{
    const auto kinds=value<uint32_t>(owner,0x628);const auto origin=assets.origin(kinds);if(!origin)throw std::runtime_error("native particle kinds lack allocation provenance");
    m=material(*origin);kindData=record(rt,kinds,m.extent);
    if(assets.origin(kinds,m.extent)!=origin)throw std::runtime_error("native particle kinds extend beyond their converted template");
    // +210 is not a draw input: 22fa50 turns it into draw bucket +60 = 2, and the
    // bucket lists (21ea80) only order the 22f140 calls; this draw never reads it.
    if(unsigned(kindData[0])*kindData[1]!=m.kinds || value<uint16_t>(owner,0x634)!=0)
        throw std::runtime_error(std::format("unported native particle template/material variant (grid {}x{} for {} kinds, +634={:x}, object {:x}, kinds {:x})",
            kindData[0],kindData[1],m.kinds,value<uint16_t>(owner,0x634),object,kinds));
    const auto fis=value<uint32_t>(owner,0x658);auto *descriptor=record(rt,fis,16);const TextureKey key{value<uint32_t>(descriptor,4),value<uint32_t>(descriptor,8)};
    const auto manager=word(rt,0x35de38);const auto slot=int16_t(word(rt,manager+0x2670)&0xffff);
    if(slot<0 || slot>=256 || word(rt,manager+0x1008+16*slot)!=key.tag || word(rt,manager+8+16*slot)!=key.index)throw std::runtime_error("native particle FIS identity differs from selected game texture");
    // A key whose disc copies differ resolves by the origin of the texture header the info
    // block names (info + *(info+c), as the offline index locates it).
    const auto header=fis+value<uint32_t>(descriptor,0xc);const auto headerOrigin=assets.origin(header,0x10);
    const auto resolvedKey=headerOrigin?textures.resolve(key,*headerOrigin):key;
    const auto texture=textures.load(resolvedKey);const auto palette=texture->format==0x14?kindData[2]:0;
    // The draw uploads this copy to the slot's TEX0 blocks, and other draws name it there: battle 0x2f's model f1b800
    // samples the manager slot the particle registered (key 36317076:3ac, whose disc copies differ, at block 3480).
    {const uint64_t tex0=word(rt,manager+0x1010+16*slot)|uint64_t(word(rt,manager+0x1014+16*slot))<<32;
     const auto pixelBlock=uint32_t(tex0)&0x3fff,paletteBlock=uint32_t(tex0>>37)&0x3fff;
     if(pixelBlock)textures.publishBlock(pixelBlock,false,resolvedKey);
     if(paletteBlock && (texture->format==0x13 || texture->format==0x14))textures.publishBlock(paletteBlock,true,resolvedKey);}
    batch.texture=texture->picture(palette);
    // The template carries no CLAMP: U clamps and V repeats as the earlier
    // single-pass draws did (the game's own CLAMP for these kicks is unverified).
    mat.textured=true;mat.textureAlpha=m.textureAlpha;mat.textureFunction=static_cast<bozo::render::Material::TextureFunction>(m.function);mat.clampU=true;mat.clampV=false;
    mat.alphaBlend=true;mat.blend=bozo::render::Material::Blend::Alpha;mat.gsBlend=true;
    mat.depthTest=true;mat.depthWrite=false;mat.alphaTest=false;
    }
    // Screen depth coefficients are in the game's projected depth domain and can
    // exceed the spatial-input bound. They are coefficients, not positions.
    const auto world=value<Mat>(owner,0x90);for(const auto &matrix:{world,camera->view,camera->screen,camera->guard})for(float v:matrix){
        const auto bits=std::bit_cast<uint32_t>(v)&0x7fffffff;
        if(bits>=0x7f800000 || (bits && bits<0x00800000) || std::abs(v)>1.e12f)throw std::runtime_error("unqualified native particle matrix coefficient");}
    for(float centre:camera->centre)number(centre);
    const auto toView=product(camera->view,world);const auto width=2*(2048.f-camera->viewport[0]),height=2*(2048.f-camera->viewport[1]);
    if(width<=16 || height<=16 || width>=4096 || height>=4096)throw std::runtime_error("native particle camera viewport invalid");
    for(unsigned column=0;column<4;++column){batch.matrix[4*column]=camera->screen[column]*target->width/width+(1-target->width/width)*2048.f*camera->screen[12+column];
        batch.matrix[4*column+1]=camera->screen[4+column]*target->height/height+(1-target->height/height)*2048.f*camera->screen[12+column];batch.matrix[4*column+2]=camera->screen[8+column];batch.matrix[4*column+3]=camera->screen[12+column];}
    for(float coefficient:batch.matrix)if(!std::isfinite(coefficient))throw std::runtime_error("native particle screen transform overflow");
    auto colour=value<std::array<float,4>>(owner,0x300);const auto limit=value<std::array<float,4>>(owner,0x230);auto identity=value<uint32_t>(owner,0x310);
    if(identity>1)throw std::runtime_error("unported native particle lighting mode");
    // Images 6 and 7 first call an ambient step (1830 / 1aa0, at 0738): when the low 16 bits of VU row 97.y are set, row
    // 13.xyz = (((r5 s + r6 s) + r7 s) + r8 s) + r19 with s = row 100.y, and row 14.x (+310, the "no lighting" flag) is
    // cleared. Rows 96..110 are the owner's GS block (+624), uploaded by the draw itself (UNPACK 15 to 0x60 before MSCAL
    // 6e0): row 97 is its ALPHA entry, y = ALPHA.FIX (emitter +afa, 22dac0), row 100.y = +af0. Image 9 has no such step.
    if(owner[0xaff]!=6){
        const auto block=value<uint32_t>(owner,0x624);
        if(!block)throw std::runtime_error("native particle draw has no GS row block (+624)");
        auto *rows=record(rt,block,0x100);
        if(value<uint32_t>(rows,0xc)!=0x6c0f0060)throw std::runtime_error("native particle GS row block is not the UNPACK of rows 96..110");
        if(value<uint32_t>(rows,0x24)&0xffff){
            const auto s=number(value<float>(rows,0x54));std::array<float,4> acc{};
            for(unsigned r=5;r<=8;++r){const auto row=value<std::array<float,4>>(owner,0x230+16*r);for(unsigned i=0;i<3;++i)acc[i]+=number(row[i])*s;}
            const auto ambient=value<std::array<float,4>>(owner,0x230+16*19);for(unsigned i=0;i<3;++i)colour[i]=acc[i]+number(ambient[i]);
            identity=0;}
    }
    if(!identity)for(unsigned i=0;i<4;++i){number(colour[i]);number(limit[i]);if(colour[i]<0 || limit[i]<0 || limit[i]>255)throw std::runtime_error("native particle colour/limit invalid");}
    // Image 7 with row 14.w = 2 (+31c) lights per point (1a50): a buffer starts at row 13 for every point, row 14.y (+314)
    // adds three point lights (1b28): L = rows 20..22, colour rows 23..25, k = row 27.xyz, d = L - p (p the point record's
    // position); buf.xyz = ((c1 k1/|d1| + (c0 k0) /|d0|) + buf) + c2 k2/|d2| (ERLENG for light 0, RSQRT k/sqrt((x2 + y2) + 1 z2)
    // for 1 and 2). The apply (18f0) is then colour * buf clamped to [0, row 0], whatever row 14.x says. Row 14.z (1c78) is
    // not ported. Battles 0x37 / 0x38: one light (rows 24, 25 zero).
    const bool perPointLight=(owner[0xaff]==4 || plain) && (value<uint32_t>(owner,0x31c)&0xffff)==2;
    std::array<std::array<float,4>,3> lightAt{},lightColour{};std::array<float,3> lightK{};bool pointLights=false;
    if(perPointLight){
        if(value<uint32_t>(owner,0x318)&0xffff)throw std::runtime_error("unported native particle ribbon lighting pass 1c78 (VU row 14.z)");
        pointLights=(value<uint32_t>(owner,0x314)&0xffff)!=0;
        const auto flush=[](float f){return std::fpclassify(f)==FP_SUBNORMAL?0.f:f;};   // VU reads denormals as zero
        for(unsigned k=0;k<3;++k){lightAt[k]=value<std::array<float,4>>(owner,0x230+16*(20+k));lightColour[k]=value<std::array<float,4>>(owner,0x230+16*(23+k));
            for(auto &f:lightAt[k])f=flush(f);for(auto &f:lightColour[k])f=flush(f);lightK[k]=flush(value<float>(owner,0x230+16*27+4*k));}
        for(unsigned i=0;i<4;++i){number(colour[i]);number(limit[i]);}
        identity=0;
    }
    const auto litColour=[&](const uint8_t *point){
        if(!perPointLight)return colour;
        auto buf=colour;if(!pointLights)return buf;
        const auto p=value<std::array<float,4>>(point,0);float term[3][3]{};
        for(unsigned k=0;k<3;++k){
            if(lightColour[k][0]==0 && lightColour[k][1]==0 && lightColour[k][2]==0)continue;   // 0 * anything is 0 on the VU
            float d[3];for(unsigned i=0;i<3;++i)d[i]=number(lightAt[k][i])-number(p[i]);
            const float length2=(d[0]*d[0]+d[1]*d[1])+d[2]*d[2];
            if(!(length2>=std::numeric_limits<float>::min()))throw std::runtime_error("native particle point light at the point");
            const float r=1/std::sqrt(length2);
            for(unsigned i=0;i<3;++i)term[k][i]=k==0?(number(lightColour[0][i])*lightK[0])*r:number(lightColour[k][i])*(lightK[k]*r);}
        for(unsigned i=0;i<3;++i){buf[i]=(term[1][i]+term[0][i])+buf[i];buf[i]=buf[i]+term[2][i];number(buf[i]);}
        return buf;
    };
    // Mode 6 selects VU1 image 9 (table 3141a0 entry 9 via 22ddf0: mode + 3) instead of 6:
    // the point record reads (ax, ay, scale, kind); see the corner loop. Its angle offsets
    // (VU row 37 = header +260/+264, the camera angles when emitter +afb / +afc) are not
    // ported: with either flag set the draw refuses.
    const bool tilted=owner[0xaff]==6,ribbon=owner[0xaff]==4 || plain;bool fogged=false;
    // Image 9 adds row 37's xy to the point's two angles before its sin/cos (0908: ADD.xy vf27, vf27, vf16). The draw
    // (22f250..22f2ec) fills row 37: x = -camera +5d4 when emitter +afb, y = -camera +5d0 when +afc, else 0; the camera
    // is the draw's own (a1), brought up to date first through its +64 method when +61 is clear (battle 0x37).
    float angleOffset[2]={0,0};
    if(tilted && (owner[0xafb] || owner[0xafc])){
        // (nativeParticleDrawInstruction has run the camera's +64 update first when +61 was clear, as 22f268 does.)
        if(owner[0xafb])angleOffset[0]=-number(value<float>(record(rt,camera->object+0x5d4,4),0));
        if(owner[0xafc])angleOffset[1]=-number(value<float>(record(rt,camera->object+0x5d0,4),0));}
    // VU1 particle program (0x6e0, image c8e40f07): MUL tint*colour (0b60), MINIi 255
    // (0b80), FTOI0 (0ba8); no clamp at zero. FTOI0 truncates toward zero (saturating),
    // and GIF PACKED RGBAQ keeps each word's low byte: a channel of -1 draws as 0xff.
    // The lighting pass before it (subroutine 1400 in image 6, 1670 in image 7, the same
    // words) multiplies by VU row 13 (+300) and clamps to [0, row 0 (+230)] unless +310 is set.
    // Image 5 has no tint and no MINI 255: FTOI0 of the lit colour, low byte per channel.
    // A negative point colour (a particle fading past zero: maps 136-142, 299, 304) lit by that pass clamps to 0; unlit it
    // wraps in the GIF byte (-1 -> 0xff). Both checked against images 3, 4, 5 and 7 in the ribbon test.
    const auto pointColour=[&](const uint8_t *point,const uint8_t *shape){
        const auto light=litColour(point);
        if(plain){uint32_t rgba=0;for(unsigned i=0;i<4;++i){auto channel=number(value<float>(point,16+4*i));
                if(!identity)channel=std::clamp(channel*light[i],0.f,limit[i]);
                const int32_t fixed=channel<=-2147483648.f?INT32_MIN:channel>=2147483648.f?INT32_MAX:int32_t(channel);rgba|=(uint32_t(fixed)&0xffu)<<(8*i);}
            return rgba;}
        uint32_t rgba=0;for(unsigned i=0;i<4;++i){auto channel=number(value<float>(point,16+4*i));const auto tint=number(value<float>(shape,64+4*i));
            if(tint<0)throw std::runtime_error("negative native particle tint");
            if(!identity)channel=std::clamp(channel*light[i],0.f,limit[i]);channel=std::min(channel*tint,255.f);
            const int32_t fixed=channel<=-2147483648.f?INT32_MIN:int32_t(channel);rgba|=(uint32_t(fixed)&0xffu)<<(8*i);}
        return rgba;
    };
    const auto outsideGuard=[&](const bozo::render::Vertex &v){const auto clip=transform(camera->guard,{v.x,v.y,v.z,1});
        return std::abs(clip[0])>std::abs(clip[3]) || std::abs(clip[1])>std::abs(clip[3]) || std::abs(clip[2])>std::abs(clip[3]);};
    if(ribbon){
        // Mode 4 selects VU1 image 7 (table 3141a0 entry 7, mod001 325240): ribbons. The
        // owner's VU rows are owner +230 + 16 row (the +220 REF opens with UNPACK 0x2f
        // qwords to row 0). Row 15.w = 0 runs 0868..0f28; 15.w set selects the fog variant
        // (0f38..1660: XYZF2, fog from row 68, below). Row 14.w = 2 (pass 1a50) is
        // litColour's per-point buffer.
        // Image 5's fog variant (row 15.w, 0ee8..15f8): the same strips, each vertex's F = clamp(row68.z + row68.w Q, 0, 255)
        // (MULAz.w by vf00.w, MADDq.w, MINIi 255, MAXx), Q = 1 / its screen w; FTOI4 keeps the integer F. Row 68 is the
        // camera's +5a0 qword (z, w = the fog pair at +5a8, battle 0x1c: 186, 69000); FOGCOL is the camera's clear colour.
        // Image 7's fog variant (0f38..1660) forms the same F per point: MULAz.w / MADDq.w (Q = 1 / its first vertex's
        // w, 1198 / 1360 / 1538), MAXx, MINIi 255, FTOI4; its second vertex keeps that F (FTOI4.xyz vf23, vf21 at 1270;
        // the middle points' ADDi.w 2048 only adds the ADC bit 0x8000 to the w word). With screen w independent of x / y
        // (checked below) both vertices have that w. Its guard transforms finish before the fog's ACC.w write.
        if(value<uint32_t>(owner,0x32c)){
            fogged=true;mat.fog=!plain || ((value<uint32_t>(record(rt,value<uint32_t>(owner,0x624),0x100),0x74)>>20)&1);
            mat.fogColour=camera->clearColour;for(float f:camera->fog)number(f);}
        // Image 7 divides a point's second vertex by the first one's Q (0b58, 0d08, 0ea8):
        // that equals a per-vertex projection only while the screen w row ignores x and y.
        if(!plain && (camera->screen[12]!=0 || camera->screen[13]!=0))throw std::runtime_error("unported native particle ribbon projection (screen w depends on x/y)");
    }
    // VU1 0x6e0 kicks every point once per pass: pass p draws kind + p * stride with ALPHA variant p,
    // all points of a pass before the next pass. Image 7 never reads the pass header (VU 2e0):
    // one pass with ALPHA variant 0 (VU 2e5 = template +70).
    if(lineStrips){
        // Image 4 (0868..0a20; fog variant 0a30..): the same projection as image 3 (vf01..vf04 = local-to-screen left by
        // the set-up 238, guard rows 85..88) for every record, one GS LINE STRIP vertex each, coloured FTOI0 of the lit
        // record colour. The header's z (ilw.z 0(vi14)) is the points per strip: at each strip's start FCSET 0xffffff
        // marks the previous judgements outside, and FCAND 0x000fff (this vertex's CLIPW and the one before) sets ADC.
        // So a segment joins two consecutive points of one strip when neither is outside the guard; strips never join.
        // Its fog variant (row 15.w, 0a30..) is image 3's: each vertex's F = clamp(fog.x + fog.y q, 0, 255) from its own
        // projection, and a chunk's first vertex coloured from vf18 before its load (row 96's FTOI0 in the first chunk,
        // colour 0 after: VU memory this draw never wrote). The GS fogs only with the tag's FGE.
        const bool variant=value<uint32_t>(owner,0x32c)!=0,fge=(value<uint32_t>(record(rt,value<uint32_t>(owner,0x624),0x100),0x74)>>20)&1;
        const bool fog=variant && fge;if(fog)for(float f:camera->fog)number(f);
        uint32_t staleColour=0;
        {const auto *row96=record(rt,value<uint32_t>(owner,0x624)+0x10,16);
         for(unsigned i=0;i<4;++i)staleColour|=(bozo::ps2f::toFixed(value<uint32_t>(row96,4*i),0)&0xffu)<<(8*i);}
        mat.lines=true;mat.fog=fog;mat.fogColour=camera->clearColour;
        auto cursor=base+0x20;uint32_t seen=0;
        for(unsigned chunk=0;chunk<chunks;++chunk){
            if(cursor>base+span-0x20)throw std::runtime_error("native particle chunk outside completed buffer");auto *header=record(rt,cursor,0x20);
            const auto count=value<uint32_t>(header,0x10),extent=value<uint32_t>(header,0x14),group=value<uint32_t>(header,0x18);
            // The program subtracts whole strips from the count (isub vi09, vi06 at 09a0) until it reaches 0.
            if(!count || count>48 || extent!=count*3+2 || !group || group>48 || count%group || count>total-seen || uint64_t(cursor)+32+48*count>uint64_t(base)+span)
                throw std::runtime_error(std::format("unported/incomplete native particle line-strip group (count {} extent {} group {})",count,extent,group));
            auto *records=record(rt,cursor+0x20,count*48);
            std::array<bozo::render::Vertex,48> strip;std::array<bool,48> outside;
            for(unsigned p=0;p<count;++p){const auto *point=records+48*p;
                std::array<float,4> position=value<std::array<float,4>>(point,0);for(unsigned j=0;j<3;++j)number(position[j]);position[3]=1;
                const auto eye=transform(toView,position);auto &v=strip[p];v.x=eye[0];v.y=eye[1];v.z=eye[2];
                v.rgba=variant && !p?(chunk?0u:staleColour):pointColour(point,nullptr);v.fog=1;
                if(fog){const auto clip=transform(camera->screen,eye);if(!(clip[3]!=0))throw std::runtime_error("native particle strip vertex at w = 0");
                    float f=camera->fog[0]+camera->fog[1]*(1/clip[3]);if(!(f<=255.f))f=255.f;if(!(f>=0.f))f=0.f;v.fog=std::floor(f)/255.f;}
                for(float f:{v.x,v.y,v.z})if(!std::isfinite(f))throw std::runtime_error("native particle vertex overflow");
                outside[p]=outsideGuard(v);}
            // The fog variant judges each vertex's guard one vertex ahead (CLIPW 0b08), so the FCSET 0xffffff at a strip's
            // end (0b30) replaces the judgement already taken for the next strip's first vertex: in every strip after a
            // chunk's first, the start and the vertex after it both carry ADC (the segment start..start+1 is not drawn).
            for(unsigned first=0;first<count;first+=group)for(unsigned i=first+1;i<first+group;++i){
                if(variant && first && i==first+1)continue;
                if(!outside[i-1] && !outside[i]){batch.vertices.push_back(strip[i-1]);batch.vertices.push_back(strip[i]);}}
            seen+=count;cursor+=32+48*count;
        }
        if(seen!=total)throw std::runtime_error("native particle draw point count mismatch");
        if(!batch.vertices.empty())submission.batches.push_back(batch);
        return submission;
    }
    if(points){
        // Image 3 (0868..09c0; fog variant 09d0..0af8): one GS point per particle. The header's set-up program (MSCAL 238)
        // leaves vf01..vf04 = rows 89..92 as it stored them at 0308, the owner's local-to-screen matrix (screen x view x
        // world); image 3 never loads them and projects each record position through them with its own divide. Rows
        // 85..88 (guard x view x world) judge it: CLIPW, FCAND 0x3f, ADC when outside. The colour is FTOI0 of the lit
        // record colour (image 5's rule). A point covers one GS pixel: drawn as a 1 x 1 quad in GS pixel space.
        // Its fog variant (row 15.w, 09d0) colours each chunk's first point with vf18 as it was (FTOI0 at 0a10 runs before
        // the record's load at 0a18). In the first chunk that is row 96, the GS block's A+D tag (loaded at 0788 by the
        // 6e0 prologue): FTOI0 of its words. In later chunks (MSCAL 818) it is the previous chunk's load one past its
        // records, VU memory this draw never wrote: colour 0, invisible (user 2026-11-24: "Colour 0, invisible").
        // The GS fogs only with the tag's PRIM.FGE (bit 20 of its high word); the variant's first-point colour stays.
        const bool variant=value<uint32_t>(owner,0x32c)!=0,fge=(value<uint32_t>(record(rt,value<uint32_t>(owner,0x624),0x100),0x74)>>20)&1;
        const bool fog=variant && fge;if(fog)for(float f:camera->fog)number(f);
        uint32_t staleColour=0;
        {const auto *row96=record(rt,value<uint32_t>(owner,0x624)+0x10,16);
         for(unsigned i=0;i<4;++i){uint32_t flags=0;staleColour|=(bozo::ps2f::toFixed(value<uint32_t>(row96,4*i),0)&0xffu)<<(8*i);}}
        NativeModelBatch dots;dots.material=mat;dots.material.fog=fog;dots.material.fogColour=camera->clearColour;
        const float width=2*(2048.f-camera->viewport[0]),height=2*(2048.f-camera->viewport[1]);
        dots.matrix={target->width/width,0,0,0,0,target->height/height,0,0,0,0,1,0,(1-target->width/width)*2048.f,(1-target->height/height)*2048.f,0,1};
        auto cursor=base+0x20;uint32_t seen=0;
        for(unsigned chunk=0;chunk<chunks;++chunk){
            if(cursor>base+span-0x20)throw std::runtime_error("native particle chunk outside completed buffer");auto *header=record(rt,cursor,0x20);
            const auto count=value<uint32_t>(header,0x10),extent=value<uint32_t>(header,0x14);
            // Image 3 reads only the header's x (the count) and the records (vi13 += 3).
            if(!count || count>48 || extent!=count*3+2 || count>total-seen || uint64_t(cursor)+32+48*count>uint64_t(base)+span)
                throw std::runtime_error(std::format("unported/incomplete native particle point group (count {} extent {})",count,extent));
            auto *records=record(rt,cursor+0x20,count*48);
            for(unsigned p=0;p<count;++p){const auto *point=records+48*p;
                std::array<float,4> position=value<std::array<float,4>>(point,0);for(unsigned j=0;j<3;++j)number(position[j]);position[3]=1;
                const auto eye=transform(toView,position);
                bozo::render::Vertex centre;centre.x=eye[0];centre.y=eye[1];centre.z=eye[2];if(outsideGuard(centre))continue;   // ADC
                const auto clip=transform(camera->screen,eye);if(!(clip[3]!=0))throw std::runtime_error("native particle point at w = 0");
                const float q=1/clip[3],x=clip[0]*q,y=clip[1]*q,z=clip[2]*q;
                if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || x<0 || x>=4096 || y<0 || y>=4096)throw std::runtime_error("native particle point outside the GS coordinate range");
                float f=1;if(fog){f=camera->fog[0]+camera->fog[1]*q;if(!(f<=255.f))f=255.f;if(!(f>=0.f))f=0.f;f=std::floor(f)/255.f;}
                const auto rgba=variant && !p?(chunk?0u:staleColour):pointColour(point,nullptr);const float gz=std::trunc(z*16)/16;
                bozo::render::Vertex corner[4];for(unsigned k=0;k<4;++k){auto &v=corner[k];v.x=x+float(k&1);v.y=y+float(k>>1);v.z=gz;v.rgba=rgba;v.fog=f;}
                for(unsigned k:{0u,1u,2u,1u,3u,2u})dots.vertices.push_back(corner[k]);}
            seen+=count;cursor+=32+48*count;
        }
        if(seen!=total)throw std::runtime_error("native particle draw point count mismatch");
        if(!dots.vertices.empty())submission.batches.push_back(std::move(dots));
        return submission;
    }
    for(unsigned pass=0;pass<(ribbon?1u:m.passes);++pass){
    if(!plain){mat.blendA=m.blends[pass][0];mat.blendB=m.blends[pass][1];mat.blendC=m.blends[pass][2];mat.blendD=m.blends[pass][3];mat.blendFix=m.blends[pass][4];}batch.vertices.clear();
    auto cursor=base+0x20;uint32_t seen=0;
    for(unsigned chunk=0;chunk<chunks;++chunk){
        if(cursor>base+span-0x20)throw std::runtime_error("native particle chunk outside completed buffer");auto *header=record(rt,cursor,0x20);
        const auto count=value<uint32_t>(header,0x10),extent=value<uint32_t>(header,0x14),group=value<uint32_t>(header,0x18);
        if(ribbon){
            // Image 7 (0868, xtop qword x = count, z = points per ribbon): one GIF packet per chunk,
            // a triangle strip of two vertices per point. A group below 2 never ends its VU loop.
            if(!count || count>48 || extent!=count*3+2 || group<2 || group>48 || count%group || count>total-seen || uint64_t(cursor)+32+48*count>uint64_t(base)+span)
                throw std::runtime_error("unported/incomplete native particle ribbon group");
            auto *points=record(rt,cursor+0x20,count*48);
            for(unsigned first=0;first<count;first+=group){
                const auto *start=points+48*first;
                // The ribbon's kind comes from its first point (ILW.w record +2 at 0970); kind rows
                // 2e9 + 5 kind: +2 (u0, v0, q0), +3 (u1, v1, q1), +4 tint.
                const uint8_t *shape=nullptr;float u0=0,v0=0,u1=0,v1=0;
                if(!plain){const auto kind=value<uint32_t>(start,44);if(kind>=m.kinds)throw std::runtime_error("native particle ribbon kind outside template");
                    shape=kindData+0xc0+80*kind;
                    if(value<uint32_t>(shape,40)!=0x3f800000 || value<uint32_t>(shape,56)!=0x3f800000)throw std::runtime_error("unported native particle projective texture coordinates");
                    u0=number(value<float>(shape,32));v0=number(value<float>(shape,36));u1=number(value<float>(shape,48));v1=number(value<float>(shape,52));}
                // T runs from v0 by (v1 - v0) / (group - 1) per point (DIV 0980, ADDw.y 0b70 / 0d50).
                const float step=plain?0.f:(v1-v0)*(1.f/float(group-1));
                std::array<std::array<float,4>,48> eye;
                for(unsigned i=0;i<group;++i){std::array<float,4> position=value<std::array<float,4>>(start+48*i,0);for(unsigned j=0;j<3;++j)number(position[j]);position[3]=1;eye[i]=transform(toView,position);}
                std::array<bozo::render::Vertex,96> strip;std::array<bool,96> outside;float t=v0;
                for(unsigned i=0;i<group;++i){const auto *point=start+48*i;const auto &e=eye[i];
                    // Direction in eye xy (z = 0): first - next at the start (0a20), last - ... at the end
                    // (prev - last, 0d88), 0.5 (prev - cur) + 0.5 (cur - next) between (0bb0..0bd8).
                    float dx,dy;
                    if(!i){dx=e[0]-eye[1][0];dy=e[1]-eye[1][1];}
                    else if(i+1==group){dx=eye[i-1][0]-e[0];dy=eye[i-1][1]-e[1];}
                    else{dx=.5f*(eye[i-1][0]-e[0])+.5f*(e[0]-eye[i+1][0]);dy=.5f*(eye[i-1][1]-e[1])+.5f*(e[1]-eye[i+1][1]);}
                    // ERLENG then width (record +2 .x): d = dir * width * 1/|dir|. A zero direction (points
                    // that coincide) gives ERLENG +max and 0 * max = 0 on the VU: no offset.
                    const float width=number(value<float>(point,32)),length=dx*dx+dy*dy;float ox=0,oy=0;
                    if(length>=std::numeric_limits<float>::min()){const float r=1/std::sqrt(length);ox=dx*width*r;oy=dy*width*r;}
                    else if(dx!=0 || dy!=0)throw std::runtime_error("unqualified native particle ribbon direction");
                    const auto rgba=pointColour(point,shape);
                    // Vertices (0a98..0ab0): p.x + d.y, p.y - d.x and p.x - d.y, p.y + d.x; S u0 / u1, both T t.
                    auto &a=strip[2*i],&b=strip[2*i+1];
                    // Image 5's loop (0b48..0cf0) moves the next point into vf19 (0b80) before its ADDy / SUBx (0bb8):
                    // a middle point's vertices take the next point's x, y around its own z (image 7 reads vf19 in
                    // the same pair as that move, so it keeps its own).
                    const auto &centre=plain && i && i+1<group?eye[i+1]:e;
                    a.x=centre[0]+oy;a.y=centre[1]-ox;b.x=centre[0]-oy;b.y=centre[1]+ox;a.z=b.z=e[2];a.u=u0;b.u=u1;a.v=b.v=t;a.rgba=b.rgba=rgba;t+=step;
                    for(const auto *v:{&a,&b})for(float f:{v->x,v->y,v->z,v->u,v->v})if(!std::isfinite(f))throw std::runtime_error("native particle vertex overflow");
                    outside[2*i]=outsideGuard(a);outside[2*i+1]=outsideGuard(b);
                    // The fog loop (1290..12c0) sets ACC.w to fog.z (MULAz.w at 1298, for vertex a's F) in the middle of
                    // vertex b's guard transform: a middle point's b is judged against w = ((a + g13 y) + g14 z) + g15.
                    if(fogged && plain && i && i+1<group){const auto clip=transform(camera->guard,{b.x,b.y,b.z,1});
                        const float w=((camera->fog[0]+camera->guard[13]*b.y)+camera->guard[14]*b.z)+camera->guard[15];
                        outside[2*i+1]=std::abs(clip[0])>std::abs(w) || std::abs(clip[1])>std::abs(w) || std::abs(clip[2])>std::abs(w);}
                    if(fogged)for(auto *v:{&a,&b}){
                        // w as the program forms it (row 52..55's w lane by MULAx, MADDAy, MADDAz, MADDw).
                        namespace V=bozo::vu0;uint32_t flags=0;const auto bits=[](float f){return std::bit_cast<uint32_t>(f);};
                        const auto &sc=camera->screen;
                        const auto w=V::addf(V::addf(V::addf(V::mulf(bits(sc[12]),bits(v->x),flags),V::mulf(bits(sc[13]),bits(v->y),flags),flags),
                            V::mulf(bits(sc[14]),bits(v->z),flags),flags),bits(sc[15]),flags);
                        const auto q=bozo::ps2f::div(0x3f800000u,w,flags);
                        auto f=std::bit_cast<float>(V::addf(bits(camera->fog[0]),V::mulf(bits(camera->fog[1]),q,flags),flags));
                        if(!(f<=255.f))f=255.f;if(!(f>=0.f))f=0.f;v->fog=std::floor(f)/255.f;}
                }
                // The first point's two vertices always carry ADC (isw.w vi06): no triangle bridges two
                // ribbons. Every later vertex gets ADC when FCAND 0x03ffff sees any of the last three
                // CLIPW judgements (its triangle) outside the guard.
                for(unsigned j=2;j<2*group;++j)if(!outside[j-2] && !outside[j-1] && !outside[j])for(unsigned k=j-2;k<=j;++k)batch.vertices.push_back(strip[k]);
            }
            seen+=count;cursor+=32+48*count;continue;
        }
        // Image 6 (mode 3) reads only the header's x, the point count (ilw.x 0(vi14) at 0990, 0f48 and in its lighting
        // pass 1400); it never reads z, the producer's group (battle 0x1b: 16 points in groups of 8), so any group draws
        // the same. Image 9 (mode 6) is unchecked: it keeps group 1.
        if(!count || count>48 || extent!=count*3+2 || (tilted?group!=1:(!group || group>48)) || count>total-seen || uint64_t(cursor)+32+48*count>uint64_t(base)+span)throw std::runtime_error(std::format("unported/incomplete native particle quad group (count {} extent {} group {} seen {} of {}, mode {})",count,extent,group,seen,total,int(owner[0xaff])));
        auto *points=record(rt,cursor+0x20,count*48);
        for(unsigned p=0;p<count;++p){const auto *point=points+48*p;const auto kind=uint64_t(value<uint32_t>(point,44))+uint64_t(pass)*m.stride;if(kind>=m.kinds)throw std::runtime_error("native particle sprite kind outside template");auto *shape=kindData+0xc0+80*kind;
            std::array<float,4> position=value<std::array<float,4>>(point,0);for(unsigned i=0;i<3;++i)number(position[i]);position[3]=1;const auto eye=transform(toView,position);
            const auto angle=number(value<float>(point,32))+angleOffset[0],sx=number(value<float>(point,36))+angleOffset[1],sy=number(value<float>(point,40)),c=std::cos(angle),s=std::sin(angle);
            const float x[2]={number(value<float>(shape,0)),number(value<float>(shape,16))},y[2]={number(value<float>(shape,4)),number(value<float>(shape,20))};
            if(value<uint32_t>(shape,40)!=0x3f800000 || value<uint32_t>(shape,56)!=0x3f800000)throw std::runtime_error("unported native particle projective texture coordinates");
            const float u[2]={number(value<float>(shape,32)),number(value<float>(shape,48))},v[2]={number(value<float>(shape,36)),number(value<float>(shape,52))};
            const auto rgba=pointColour(point,shape);
            bozo::render::Vertex corners[4];bool outside[4];
            // Image 9 (mode 6): +32 ax, +36 ay, +40 a uniform scale. Each corner (x, y, z),
            // z from the shape rows (+8 / +24), turns by ax in x/y (scaled), then by ay:
            // y' = cos(ay) y, z' = sin(ay) y + cos(ay) z (the program's row for z in y is
            // vf00.y = 0), all added to the eye-space point (VU 0b18..0c88).
            const float z[2]={tilted?number(value<float>(shape,8)):0.f,tilted?number(value<float>(shape,24)):0.f};
            const float ca=std::cos(sx),sa=std::sin(sx);
            for(unsigned i=0;i<4;++i){auto &out=corners[i];
                if(tilted){const float px=sy*(c*x[i&1]+s*y[i>>1]),py=sy*(-s*x[i&1]+c*y[i>>1]);
                    out.x=eye[0]+px;out.y=eye[1]+ca*py;out.z=eye[2]+sa*py+ca*z[i>>1];}
                else{out.x=eye[0]+c*sx*x[i&1]+s*sy*y[i>>1];out.y=eye[1]-s*sx*x[i&1]+c*sy*y[i>>1];out.z=eye[2];}
                out.u=u[i&1];out.v=v[i>>1];out.rgba=rgba;
                for(float f:{out.x,out.y,out.z,out.u,out.v})if(!std::isfinite(f))throw std::runtime_error("native particle vertex overflow");
                outside[i]=outsideGuard(out);}
            for(const auto triangle:{std::array<unsigned,3>{0,1,2},std::array<unsigned,3>{1,3,2}})if(!outside[triangle[0]] && !outside[triangle[1]] && !outside[triangle[2]])for(auto i:triangle)batch.vertices.push_back(corners[i]);
        }
        seen+=count;cursor+=32+48*count;
    }
    if(seen!=total)throw std::runtime_error("native particle draw point count mismatch");if(!batch.vertices.empty())submission.batches.push_back(batch);
    }
    return submission;
}
std::optional<bool> nativeParticleDrawInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x22f160)return std::nullopt;auto *d=rt?vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services):nullptr;
    auto *draw=d?vp2s::nativeCast<ParticleDrawOwner>(&d->host()):nullptr;auto *models=d?vp2s::nativeCast<ModelOwner>(&d->host()):nullptr;auto *scene=d?vp2s::nativeCast<SceneOwner>(&d->host()):nullptr;
    auto *assets=d?vp2s::nativeCast<AssetOwner>(&d->host()):nullptr;auto *textures=d?vp2s::nativeCast<TextureOwner>(&d->host()):nullptr;auto *particles=d?vp2s::nativeCast<ParticleOwner>(&d->host()):nullptr;
    if(!c || !draw || !models || !scene || !assets || !textures || !particles || (c->pc!=0x22f140 && c->pc!=0x22f158) || c->r[31].uw[0]!=0x22f164 || c->r[5].uw[0]<0x1005b0)throw std::runtime_error("native particle draw lacks original pre-enqueue producer");
    const auto object=c->r[5].uw[0]-0x5b0;if(c->r[4].uw[0]!=object)throw std::runtime_error("native particle draw object changed before publication");
    // Mode 6 with angle offsets (+afb / +afc) reads the drawing camera's +5d0 / +5d4. The queued draw 22f170 brings that
    // camera up to date first through its own +64 method when +61 is clear (22f268..22f27c); the native capture runs at
    // the pre-enqueue, so it makes that same compiled call here, with the CPU state kept as it was. This assumes the
    // camera's inputs are final once objects queue their draws (the first queued draw that needs it does the same).
    if(auto *owner=rt->memory.hostPointer(object,0xb10);owner && owner[0xaff]==6 && (owner[0xafb] || owner[0xafc]))
        if(const auto camera=scene->nativeScene().drawingCamera();camera && !*rt->memory.hostPointer(camera->object+0x61,1)){
            uint32_t vtable,method;std::memcpy(&vtable,rt->memory.hostPointer(camera->object,4),4);std::memcpy(&method,rt->memory.hostPointer(vtable+0x64,4),4);
            const R5900Context saved=*c;c->r[4].ud[0]=camera->object;c->r[31].ud[0]=0x22f164;
            bozo_call_indirect(rt->memory.flat(),c,rt,method);
            const auto cycles=c->cycleLimit;*c=saved;c->cycleLimit=cycles;}   // as 22f288, the angles are read whatever +61 says now
    const auto submission=draw->nativeParticleDraws().capture(rt,assets->nativeAssets(),textures->nativeTextures(),scene->nativeScene(),particles->nativeParticles(),object);
    models->drawNativeModel(submission);c->r[4].sd[0]=int32_t(object);return true;
}
}

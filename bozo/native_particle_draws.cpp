// Explicit point/kind records to native PC quads. No live command/header
// decoding: immutable material semantics are converted offline from the disc.
#include "native_particle_draws.h"
#include "bozo/ee/dispatcher.h"
#include "../port/vp2s_music_math.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <format>
#include <fstream>
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
        if(take()!=0x31505056)throw std::runtime_error("native particle material version mismatch");const auto count=take();if(count>65536)throw std::runtime_error("native particle material count outside bounds");
        std::map<uint32_t,Material> pack;
        for(unsigned i=0;i<count;++i){const auto offset=take();Material m;m.kinds=take();m.extent=take();m.textureAlpha=take();m.function=take();m.wrapU=take();m.wrapV=take();
            for(auto &blend:m.blends)for(auto &factor:blend)factor=take();
            if(!m.kinds || m.kinds>16 || m.extent!=0xc0+80*m.kinds || m.textureAlpha>1 || m.function>3 || m.wrapU>3 || m.wrapV>3)throw std::runtime_error("invalid native particle material semantics");
            for(const auto &b:m.blends){for(unsigned j=0;j<4;++j)if(b[j]>2)throw std::runtime_error("invalid native particle blend factor");if(b[4]>255)throw std::runtime_error("invalid native particle blend constant");}
            if(!pack.emplace(offset,m).second)throw std::runtime_error("duplicate native particle material offset");
        }
        if(in.peek()!=std::char_traits<char>::eof())throw std::runtime_error("trailing native particle material bytes");found=materials.emplace(key,std::move(pack)).first;
    }
    const auto record=found->second.find(origin.offset);if(record==found->second.end())throw std::runtime_error(std::format("native particle material not converted: {:x}/{}+{:x}",origin.disc,origin.part,origin.offset));return record->second;
}
NativeModelSubmission NativeParticleDraws::capture(Runtime *rt,NativeAssets &assets,NativeTextures &textures,const NativeScene &scene,const NativeParticles &particles,uint32_t object){
    music::CodecRounding arithmetic;particles.validateDraw(object);auto *owner=record(rt,object,0xb10);
    const auto camera=scene.drawingCamera();const auto target=scene.frameTarget();const auto current=word(rt,0x35ddf8);const unsigned layer=owner[0x65];
    if(!camera || !target || layer>1 || word(rt,current+0x40a8+4*layer)!=object || *rt->memory.hostPointer(current+0x40dc+layer,1)!=1 || !owner[0x63] || value<uint32_t>(owner,0)!=0x154180 || owner[0xaff]!=3 || owner[0xb04] || owner[0xaf9]>1)
        throw std::runtime_error("native particle draw has no admitted scene/point owner");
    const auto first=NativeAssets::address(value<uint32_t>(owner,0xaa0)),second=NativeAssets::address(value<uint32_t>(owner,0xaa4));
    if(second<=first || second-first<0x20 || second-first>0x1000000)throw std::runtime_error("native particle draw buffer halves invalid");
    const auto span=second-first,index=owner[0xaf9]^1u,base=index?second:first,total=value<uint32_t>(owner,0xabc),chunks=value<uint32_t>(owner,0xac0+4*index);
    record(rt,first,span*2);if(total>65536 || chunks>65536 || bool(total)!=bool(chunks))throw std::runtime_error("native particle draw counts invalid");
    NativeModelSubmission submission{object,camera->object};if(!total)return submission;
    const auto kinds=value<uint32_t>(owner,0x628);const auto origin=assets.origin(kinds);if(!origin)throw std::runtime_error("native particle kinds lack allocation provenance");
    const auto &m=material(*origin);auto *kindData=record(rt,kinds,m.extent);
    if(assets.origin(kinds,m.extent)!=origin || unsigned(kindData[0])*kindData[1]!=m.kinds || owner[0x210] || value<uint16_t>(owner,0x634)!=0 || m.wrapU>1 || m.wrapV>1)
        throw std::runtime_error("unported native particle template/material variant");
    const auto fis=value<uint32_t>(owner,0x658);auto *descriptor=record(rt,fis,16);const TextureKey key{value<uint32_t>(descriptor,4),value<uint32_t>(descriptor,8)};
    const auto manager=word(rt,0x35de38);const auto slot=int16_t(word(rt,manager+0x2670)&0xffff);
    if(slot<0 || slot>=256 || word(rt,manager+0x1008+16*slot)!=key.tag || word(rt,manager+8+16*slot)!=key.index)throw std::runtime_error("native particle FIS identity differs from selected game texture");
    const auto texture=textures.load(key);const auto palette=texture->format==0x14?kindData[2]:0;
    NativeModelBatch batch;batch.texture=texture->picture(palette);auto &mat=batch.material;
    mat.textured=true;mat.textureAlpha=m.textureAlpha;mat.textureFunction=static_cast<bozo::render::Material::TextureFunction>(m.function);mat.clampU=m.wrapU;mat.clampV=m.wrapV;
    mat.alphaBlend=true;mat.blend=bozo::render::Material::Blend::Alpha;mat.gsBlend=true;mat.blendA=m.blends[0][0];mat.blendB=m.blends[0][1];mat.blendC=m.blends[0][2];mat.blendD=m.blends[0][3];mat.blendFix=m.blends[0][4];
    mat.depthTest=true;mat.depthWrite=false;mat.alphaTest=false;
    // Screen depth coefficients are in the game's projected depth domain and can
    // exceed the spatial-input bound. They are coefficients, not positions.
    const auto world=value<Mat>(owner,0x90);for(const auto &matrix:{world,camera->view,camera->screen,camera->guard})for(float v:matrix){
        const auto bits=std::bit_cast<uint32_t>(v)&0x7fffffff;
        if(bits>=0x7f800000 || (bits && bits<0x00800000) || std::abs(v)>1.e12f)throw std::runtime_error("unqualified native particle matrix coefficient");}
    for(float centre:camera->centre)number(centre);
    const auto toView=product(camera->view,world);const auto width=2*(camera->centre[0]-camera->viewport[0]),height=2*(camera->centre[1]-camera->viewport[1]);
    if(width<=16 || height<=16 || width>=4096 || height>=4096)throw std::runtime_error("native particle camera viewport invalid");
    for(unsigned column=0;column<4;++column){batch.matrix[4*column]=camera->screen[column]*target->width/width+(1-target->width/width)*camera->centre[0]*camera->screen[12+column];
        batch.matrix[4*column+1]=camera->screen[4+column]*target->height/height+(1-target->height/height)*camera->centre[1]*camera->screen[12+column];batch.matrix[4*column+2]=camera->screen[8+column];batch.matrix[4*column+3]=camera->screen[12+column];}
    for(float coefficient:batch.matrix)if(!std::isfinite(coefficient))throw std::runtime_error("native particle screen transform overflow");
    const auto colour=value<std::array<float,4>>(owner,0x300),limit=value<std::array<float,4>>(owner,0x230);const auto identity=value<uint32_t>(owner,0x310);
    if(identity>1)throw std::runtime_error("unported native particle lighting mode");if(!identity)for(unsigned i=0;i<4;++i){number(colour[i]);number(limit[i]);if(colour[i]<0 || limit[i]<0 || limit[i]>255)throw std::runtime_error("native particle colour/limit invalid");}
    auto cursor=base+0x20;uint32_t seen=0;
    for(unsigned chunk=0;chunk<chunks;++chunk){
        if(cursor>base+span-0x20)throw std::runtime_error("native particle chunk outside completed buffer");auto *header=record(rt,cursor,0x20);
        const auto count=value<uint32_t>(header,0x10),extent=value<uint32_t>(header,0x14),group=value<uint32_t>(header,0x18);
        if(!count || count>48 || extent!=count*3+2 || group!=1 || count>total-seen || uint64_t(cursor)+32+48*count>uint64_t(base)+span)throw std::runtime_error("unported/incomplete native particle quad group");
        auto *points=record(rt,cursor+0x20,count*48);
        for(unsigned p=0;p<count;++p){const auto *point=points+48*p;const auto kind=value<uint32_t>(point,44);if(kind>=m.kinds)throw std::runtime_error("native particle sprite kind outside template");auto *shape=kindData+0xc0+80*kind;
            std::array<float,4> position=value<std::array<float,4>>(point,0);for(unsigned i=0;i<3;++i)number(position[i]);position[3]=1;const auto eye=transform(toView,position);
            const auto angle=number(value<float>(point,32)),sx=number(value<float>(point,36)),sy=number(value<float>(point,40)),c=std::cos(angle),s=std::sin(angle);
            const float x[2]={number(value<float>(shape,0)),number(value<float>(shape,16))},y[2]={number(value<float>(shape,4)),number(value<float>(shape,20))};
            if(value<uint32_t>(shape,40)!=0x3f800000 || value<uint32_t>(shape,56)!=0x3f800000)throw std::runtime_error("unported native particle projective texture coordinates");
            const float u[2]={number(value<float>(shape,32)),number(value<float>(shape,48))},v[2]={number(value<float>(shape,36)),number(value<float>(shape,52))};
            uint32_t rgba=0;for(unsigned i=0;i<4;++i){auto channel=number(value<float>(point,16+4*i));const auto tint=number(value<float>(shape,64+4*i));if(channel<0 || tint<0)throw std::runtime_error("negative native particle colour");
                if(!identity)channel=std::clamp(channel*colour[i],0.f,limit[i]);channel=std::min(channel*tint,255.f);rgba|=uint32_t(channel)<<(8*i);}
            bozo::render::Vertex corners[4];bool outside[4];
            for(unsigned i=0;i<4;++i){auto &out=corners[i];out.x=eye[0]+c*sx*x[i&1]+s*sy*y[i>>1];out.y=eye[1]-s*sx*x[i&1]+c*sy*y[i>>1];out.z=eye[2];out.u=u[i&1];out.v=v[i>>1];out.rgba=rgba;
                for(float f:{out.x,out.y,out.z,out.u,out.v})if(!std::isfinite(f))throw std::runtime_error("native particle vertex overflow");
                const auto clip=transform(camera->guard,{out.x,out.y,out.z,1});outside[i]=std::abs(clip[0])>std::abs(clip[3]) || std::abs(clip[1])>std::abs(clip[3]) || std::abs(clip[2])>std::abs(clip[3]);}
            for(const auto triangle:{std::array<unsigned,3>{0,1,2},std::array<unsigned,3>{1,3,2}})if(!outside[triangle[0]] && !outside[triangle[1]] && !outside[triangle[2]])for(auto i:triangle)batch.vertices.push_back(corners[i]);
        }
        seen+=count;cursor+=32+48*count;
    }
    if(seen!=total)throw std::runtime_error("native particle draw point count mismatch");if(!batch.vertices.empty())submission.batches.push_back(std::move(batch));return submission;
}
std::optional<bool> nativeParticleDrawInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x22f160)return std::nullopt;auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *draw=d?dynamic_cast<ParticleDrawOwner *>(&d->host()):nullptr;auto *models=d?dynamic_cast<ModelOwner *>(&d->host()):nullptr;auto *scene=d?dynamic_cast<SceneOwner *>(&d->host()):nullptr;
    auto *assets=d?dynamic_cast<AssetOwner *>(&d->host()):nullptr;auto *textures=d?dynamic_cast<TextureOwner *>(&d->host()):nullptr;auto *particles=d?dynamic_cast<ParticleOwner *>(&d->host()):nullptr;
    if(!c || !draw || !models || !scene || !assets || !textures || !particles || (c->pc!=0x22f140 && c->pc!=0x22f158) || c->r[31].uw[0]!=0x22f164 || c->r[5].uw[0]<0x1005b0)throw std::runtime_error("native particle draw lacks original pre-enqueue producer");
    const auto object=c->r[5].uw[0]-0x5b0;if(c->r[4].uw[0]!=object)throw std::runtime_error("native particle draw object changed before publication");
    const auto submission=draw->nativeParticleDraws().capture(rt,assets->nativeAssets(),textures->nativeTextures(),scene->nativeScene(),particles->nativeParticles(),object);
    models->drawNativeModel(submission);c->r[4].sd[0]=int32_t(object);return true;
}
}

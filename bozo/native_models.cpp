// Native mesh assets, game object records and scalar rendering mathematics.
// No runtime packet decoding, transport replay or vector-machine state.
#include "native_models.h"
#include "native_math.h"
#include "bozo/ee/dispatcher.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <format>
#include <iterator>
#include <set>
#include <source_location>
namespace vp2s {
struct NativeModels::Mesh {
    struct Vertex {float p[4],n[3],uv[2];uint8_t colour[4];};
    struct Bone {int32_t a;float wa;int32_t b;float wb;};
    struct Source {uint32_t normal,position;};
    uint32_t group=0,sourceSize=0;
    std::array<int32_t,4> algorithms{};
    std::array<uint32_t,11> material{};
    bool hasMaterial=false;
    int32_t scroll=-1;
    std::vector<uint16_t> strips;
    std::vector<Vertex> vertices;
    std::vector<Bone> bones;
    std::vector<Source> sources;
    bool has(int n)const{return std::find(algorithms.begin(),algorithms.end(),n)!=algorithms.end();}
};
namespace {
struct Reader {
    std::vector<uint8_t> data;size_t at=0;
    template<class T>T take(){T v;read(&v,sizeof(v));return v;}
    void read(void *to,size_t n){if(n>data.size()-at)throw std::runtime_error("truncated semantic native mesh pack");std::memcpy(to,data.data()+at,n);at+=n;}
    template<class T>void array(std::vector<T> &v,size_t n){if(n>0x100000 || n> (data.size()-at)/sizeof(T))throw std::runtime_error("native mesh count outside asset");v.resize(n);read(v.data(),n*sizeof(T));}
};
template<class T>T value(Runtime *rt,uint32_t at,std::source_location source=std::source_location::current()){
    if(!rt || at<0x100000 || uint64_t(at)+sizeof(T)>0x2000000)throw std::runtime_error(std::format("native model record outside EE RAM: address={:08x}, bytes={}, source_line={}",at,sizeof(T),source.line()));
    const auto *p=rt->memory.hostPointer(at,sizeof(T));if(!p)throw std::runtime_error("unmapped native model record");
    T v;std::memcpy(&v,p,sizeof(T));return v;
}
using Mat=std::array<float,16>; // Row-major game matrices.
Mat product(const Mat &a,const Mat &b){Mat out{};for(unsigned r=0;r<4;++r)for(unsigned c=0;c<4;++c)
    for(unsigned k=0;k<4;++k)out[4*r+c]+=a[4*r+k]*b[4*k+c];return out;}
std::array<float,4> apply(const Mat &m,const std::array<float,4> &p){std::array<float,4> out{};
    for(unsigned r=0;r<4;++r)for(unsigned c=0;c<4;++c)out[r]+=m[4*r+c]*p[c];return out;}
float dot(const std::array<float,3> &a,const std::array<float,3> &b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
void finite(float f){if(!std::isfinite(f))throw std::runtime_error("nonfinite native model input/result");}
}
std::shared_ptr<const NativeModels::Pack> NativeModels::load(const AssetOrigin &origin){
    if(origin.kind!=AssetOrigin::Kind::Part)throw std::runtime_error("native mesh has no decoded asset-part provenance");
    const auto key=std::pair{origin.disc,origin.part};if(auto it=packs.find(key);it!=packs.end())return it->second;
    const auto file=root/std::format("{:010x}_{:03}.mesh",origin.disc,origin.part);
    std::ifstream in(file,std::ios::binary);if(!in)throw std::runtime_error("missing semantic native mesh pack: "+file.string());
    Reader r{{std::istreambuf_iterator<char>(in),{}}};
    if(r.take<uint32_t>()!=0x314e5056)throw std::runtime_error("semantic native mesh version mismatch");
    const auto count=r.take<uint32_t>();if(count>65536)throw std::runtime_error("native mesh pack too large");
    auto pack=std::make_shared<Pack>();
    for(unsigned i=0;i<count;++i){auto m=std::make_shared<Mesh>();const auto offset=r.take<uint32_t>(),nv=r.take<uint32_t>(),ns=r.take<uint32_t>();
        m->group=r.take<uint32_t>();m->algorithms=r.take<std::array<int32_t,4>>();
        const auto has=r.take<uint32_t>();if(has>1)throw std::runtime_error("invalid native material presence");m->hasMaterial=has;
        m->material=r.take<std::array<uint32_t,11>>();
        const auto &s=m->material;
        if(s[0]>1 || s[1]>1 || s[2]>1 || s[3]>3 || s[4]>3 || s[5]>1 || s[6]>1 || s[7]>2 || s[8]>2 || s[9]>2 || s[10]>2)
            throw std::runtime_error("invalid semantic native material");
        r.array(m->strips,ns);r.array(m->vertices,nv);if(m->has(4))r.array(m->bones,nv);
        if(m->has(48))m->scroll=r.take<int32_t>();
        m->sourceSize=r.take<uint32_t>();r.array(m->sources,nv);
        uint64_t total=0;for(auto n:m->strips)total+=n;if(total!=nv)throw std::runtime_error("native strip extent mismatch");
        for(const auto source:m->sources)for(auto at:{source.normal,source.position})if(at!=UINT32_MAX && uint64_t(at)+12>m->sourceSize)
            throw std::runtime_error("native editable vertex outside asset");
        if(!pack->emplace(offset,m).second)throw std::runtime_error("duplicate native mesh offset");
    }
    if(r.at!=r.data.size())throw std::runtime_error("trailing semantic mesh bytes");
    packs[key]=pack;return pack;
}
NativeModelSubmission NativeModels::capture(Runtime *rt,NativeAssets &assets,NativeTextures &textures,const NativeScene &scene,uint32_t object){
    const auto word=[&](uint32_t at,std::source_location source=std::source_location::current()){return value<uint32_t>(rt,at,source);};
    const auto byte=[&](uint32_t at){return value<uint8_t>(rt,at);};
    const auto real=[&](uint32_t at){const auto v=value<float>(rt,at);finite(v);return v;};
    const auto row=[&](uint32_t n){auto v=value<std::array<float,4>>(rt,object+0x230+16*n);for(auto f:v)finite(f);return v;};
    const auto camera=scene.drawingCamera();const auto target=scene.frameTarget();
    if(!camera || !target)throw std::runtime_error("native model has no active camera/frame");
    const auto current=word(0x35ddf8);const auto layer=byte(object+0x65);
    if(layer>1 || word(current+0x40a8+4*layer)!=object || byte(current+0x40dc+layer)!=1)
        throw std::runtime_error("native model has no pending scene ownership");
    const auto model=word(object+0x7bc),count=word(model+0x34),list=word(object+0x77c);
    if(count>4096)throw std::runtime_error(std::format("native model mesh count outside bounds: model={:08x} count={}",model,count));
    if(!count){
        if((model&15) || model<0x100010)throw std::runtime_error("empty native model lacks aligned FPS header");
        const auto header=assets.origin(model-0x10,0x70);
        if(!header || header->kind!=AssetOrigin::Kind::Part || word(model-0x10)!=0x00535046)
            throw std::runtime_error("empty native model lacks complete decoded FPS header");
        // All four original mesh loops test count > 0 before reading a mesh.
        // No vertex, material, skeleton or lighting input is consumed here.
        return NativeModelSubmission{object,camera->object};
    }
    const auto world=value<Mat>(rt,object+0x90);for(auto f:world)finite(f);
    const auto viewObject=product(camera->view,world),screen=product(camera->screen,viewObject),guard=product(camera->guard,viewObject);
    NativeModelSubmission out{object,camera->object};
    Mat matrix{};const float width=2*(camera->centre[0]-camera->viewport[0]),height=2*(camera->centre[1]-camera->viewport[1]);
    for(const auto &m:{camera->view,camera->screen,camera->guard,viewObject,screen,guard})for(auto f:m)finite(f);
    finite(width);finite(height);
    if(width<=16 || height<=16 || width>=4096 || height>=4096)throw std::runtime_error("native model camera viewport outside bounds");
    for(unsigned c=0;c<4;++c){matrix[4*c]=screen[c]*target->width/width+(1-target->width/width)*camera->centre[0]*screen[12+c];
        matrix[4*c+1]=screen[4+c]*target->height/height+(1-target->height/height)*camera->centre[1]*screen[12+c];
        matrix[4*c+2]=screen[8+c];matrix[4*c+3]=screen[12+c];}
    std::array<std::array<float,3>,4> directions{};std::array<std::array<float,4>,4> colours{};
    float rotation[3][3];for(unsigned r=0;r<3;++r){const float length=std::sqrt(world[4*r]*world[4*r]+world[4*r+1]*world[4*r+1]+world[4*r+2]*world[4*r+2]);
        for(unsigned c=0;c<3;++c)rotation[r][c]=length?world[4*r+c]/length:0;}
    for(unsigned i=0;i<4;++i){const auto d=row(i+1);colours[i]=row(i+5);for(unsigned c=0;c<3;++c)directions[i][c]=rotation[0][c]*d[0]+rotation[1][c]*d[1]+rotation[2][c]*d[2];}
    const auto ambient=row(19),objectColour=row(13),limit=row(0);
    const bool tableA=word(object+0x31c)!=0,plain=word(object+0x310)!=0;
    // The light-table word mem[14] is object+310: x plain, y table B (point
    // lights), z table C (spot lights), w table A. Table A alone changes only
    // mode 0 (the object colour is kept). With A, B and C add lights to the
    // factor after it, in object space as the VU1 object set-up leaves them
    // (vp2s_vu1_hle runObjectSetup/runLighting):
    // - positions mem[20..22] (B) and mem[28..29] (C) through O's rigid
    //   inverse (R^T, -R^T t times w); spot directions mem[30..31] by Rn^T;
    // - B: three point lights, colours mem[23..25], strengths mem[27].xyz;
    //   each adds colour * max(0, n . d * s / |d|^2) squared header 9.w times;
    // - C: one or two spot lights (mem[14].z = 1 or 2), colour mem[32 + i],
    //   cone mem[34 + i] (.x cos^2 offset, .y scale, .z strength over distance);
    //   mode 3 adds a specular half-vector term (header 11.y/.z weights).
    const auto pointLights=tableA?word(object+0x314):0u,spotLights=tableA?word(object+0x318):0u;
    if(spotLights>2)throw std::runtime_error(std::format("unqualified native spot light count {}",spotLights));
    const auto rigidInverse=[&](const std::array<float,4> &x){std::array<float,3> out{};
        for(unsigned j=0;j<3;++j)for(unsigned r=0;r<3;++r)out[j]+=world[4*r+j]*(x[r]-world[4*r+3]*x[3]);return out;};
    struct PointLight{std::array<float,3> position,colour;float strength;};std::array<PointLight,3> points{};
    if(pointLights){const auto strength=row(27);for(unsigned i=0;i<3;++i){const auto c=row(23+i);points[i]={rigidInverse(row(20+i)),{c[0],c[1],c[2]},strength[i]};}}
    struct SpotLight{std::array<float,3> position,direction,colour;std::array<float,4> cone;};std::vector<SpotLight> spots;
    for(unsigned i=0;i<spotLights;++i){const auto d=row(30+i),c=row(32+i);SpotLight spot{rigidInverse(row(28+i)),{},{c[0],c[1],c[2]},row(34+i)};
        for(unsigned c2=0;c2<3;++c2)spot.direction[c2]=rotation[0][c2]*d[0]+rotation[1][c2]*d[1]+rotation[2][c2]*d[2];spots.push_back(spot);}
    std::array<float,3> eye{};for(unsigned c=0;c<3;++c)eye[c]=-(viewObject[c]*viewObject[3]+viewObject[4+c]*viewObject[7]+viewObject[8+c]*viewObject[11]);
    uint32_t boneCount=0,palette=0;const auto skeleton=word(object+0x648);
    if(skeleton){boneCount=word(word(skeleton+0x34)+4);if(boneCount>4096)throw std::runtime_error("native bone count outside bounds");
        palette=word(skeleton+0x4c)+0x40+64*boneCount*(byte(skeleton+0x50)&1);}
    // This optional working-copy probe inherited a legacy heuristic. The slot
    // also holds ordinary record data, so qualify a complete RAM header before
    // dereferencing it; never wrap an arbitrary value into the address space.
    const auto candidate=word(object+0x630),segment=candidate&0xf0000000u,offset=candidate&0x0fffffffu;
    const bool ramAlias=segment==0 || segment==0x20000000u || segment==0x30000000u || segment==0x80000000u || segment==0xa0000000u;
    const auto work=ramAlias && !(offset&15) && offset>=0x100000 && offset<=0x2000000-0x68?NativeAssets::address(candidate):0;
    const bool working=work && work!=model && word(work+0x34)==count && word(work+0x60)==word(model+0x60) && word(work+0x64)==word(model+0x64);
    const auto manager=word(0x35de38);uint32_t cursor=model+0x60,block=0;
    std::array<uint32_t,11> material{0,1,1,0,0,0,0,0,1,0,1};
    for(unsigned i=0;i<count;++i){uint32_t mesh;
        if(list && byte(object+0x7c2)){const auto entry=word(list+12*count+4*i);mesh=word(entry);block=word(entry+4);}
        else{mesh=cursor;const auto size=value<int16_t>(rt,mesh+4);if(size<=0)throw std::runtime_error("invalid native mesh source stride");cursor+=size;
            if(word(mesh+0x14)&1){block=mesh+0x40;const auto entry=word(block+0x5c);if(entry){if(entry>4096)throw std::runtime_error("native material palette index outside bounds");block=word(object+0x734)+256*(entry-1);}}}
        if(!mesh)continue;
        if(byte(model+0x59) && (value<uint16_t>(rt,mesh+0x36)&value<uint16_t>(rt,object+0x7a4)))continue;
        const auto origin=assets.origin(mesh);if(!origin)throw std::runtime_error("native mesh missing allocation provenance");
        const auto pack=load(*origin);const auto found=pack->find(origin->offset);
        if(found==pack->end())throw std::runtime_error(std::format("native mesh not converted: {:x}/{}+{:x}",origin->disc,origin->part,origin->offset));
        const auto &m=*found->second;
        for(auto algorithm:m.algorithms)if(algorithm!=-1 && algorithm!=0 && algorithm!=2 && algorithm!=4 && algorithm!=6 && algorithm!=8 && algorithm!=12 && algorithm!=14 && algorithm!=18 && algorithm!=28 && algorithm!=32 && algorithm!=36 && algorithm!=44 && algorithm!=46 && algorithm!=48)
            throw std::runtime_error(std::format("unported native mesh algorithm {}",algorithm));
        if(m.hasMaterial)material=m.material;
        else if(block>=0x40 && block-0x40!=mesh){const auto owner=assets.origin(block-0x40);if(owner && owner->disc==origin->disc && owner->part==origin->part){
            const auto other=pack->find(owner->offset);if(other!=pack->end() && other->second->hasMaterial)material=other->second->material;}}
        NativeModelBatch batch;batch.matrix=matrix;auto &mat=batch.material;
        mat.alphaBlend=material[0];mat.blend=mat.alphaBlend?bozo::render::Material::Blend::Alpha:bozo::render::Material::Blend::Opaque;
        if(mat.alphaBlend){mat.gsBlend=true;mat.blendA=material[7];mat.blendB=material[8];mat.blendC=material[9];mat.blendD=material[10];mat.blendFix=0x80;}
        mat.alphaTest=true;mat.depthWrite=!mat.alphaBlend || palette;mat.textureAlpha=material[2];
        mat.textureFunction=static_cast<bozo::render::Material::TextureFunction>(material[3]);
        // PRIM.FGE is set by the game in the vertices' GIF tag in the block in EE
        // RAM (row 13 at +70: EOP|nloop 8000, PRIM at +74 bits 15.., REGS 412 =
        // ST RGBAQ XYZF2); the converted (disc) material has it clear. This is the
        // town/harbour distance haze (PCSX2: almost every scene prim has FGE).
        const bool liveTag=block>=0x100000 && word(block+0x70)==0x8000 && word(block+0x78)==0x412;
        const bool fogEnabled=liveTag?((word(block+0x74)>>15)&0x20)!=0:material[6]!=0;
        mat.fog=fogEnabled && camera->fog[1]!=0;mat.fogColour=camera->clearColour;
        if(material[1]){if(!block || !manager)throw std::runtime_error("native textured mesh missing material/manager");
            const auto slot=value<int32_t>(rt,block+0x98);if(slot<0 || slot>=256)throw std::runtime_error("native mesh texture slot outside range");
            const auto index=value<int16_t>(rt,manager+0x2670+2*slot);if(index<0)throw std::runtime_error("native mesh texture not registered");
            const TextureKey key{word(manager+0x1008+16*index),word(manager+8+16*index)};batch.texture=textures.load(key)->picture(word(block+0x9c));}
        mat.textured=bool(batch.texture);
        const bool editable=working && mesh>=model && !m.has(6) && !m.has(8);
        const auto source=editable?work+mesh-model:0;
        std::array<float,2> scroll{};if(m.has(48)){if(m.scroll<0 || m.scroll>=16)throw std::runtime_error("native UV scroll index outside bounds");const auto q=row(38+m.scroll);scroll={q[2],q[3]};}
        const auto skinned=[&](size_t k,std::array<float,3> p,float w){
            const auto transform=[&](int32_t address){if(address<0x290 || ((address-0x290)&3) || uint32_t((address-0x290)/4)>=boneCount)throw std::runtime_error("native vertex bone outside palette");
                const auto mtx=value<Mat>(rt,palette+64*((address-0x290)/4));std::array<float,3> result{};
                // Bone palette is three row vectors followed by translation.
                for(unsigned c=0;c<3;++c)result[c]=mtx[c]*p[0]+mtx[4+c]*p[1]+mtx[8+c]*p[2]+mtx[12+c]*w;return result;};
            if(palette && m.has(2))return transform(int32_t(m.vertices[k].p[3]));
            if(palette && m.has(4)){const auto b=m.bones[k];std::array<float,3> result{};bool any=false;
                for(auto [address,weight]:{std::pair{b.a,b.wa},std::pair{b.b,b.wb}}){finite(weight);if(weight!=0){const auto q=transform(address);for(unsigned c=0;c<3;++c)result[c]+=q[c]*weight;any=true;}}
                if(any)return result;}
            return p;};
        std::vector<bozo::render::Vertex> vertices(m.vertices.size());std::vector<std::array<float,4>> projected;std::vector<bool> outside;
        for(size_t k=0;k<m.vertices.size();++k){const auto &v=m.vertices[k];std::array<float,3> p{v.p[0],v.p[1],v.p[2]},n{v.n[0],v.n[1],v.n[2]};
            if(editable){if(m.sources[k].position!=UINT32_MAX)for(unsigned c=0;c<3;++c)p[c]=real(source+m.sources[k].position+4*c);
                if(m.sources[k].normal!=UINT32_MAX)for(unsigned c=0;c<3;++c)n[c]=real(source+m.sources[k].normal+4*c);}
            p=skinned(k,p,1);n=skinned(k,n,0);auto &vout=vertices[k];vout.x=p[0];vout.y=p[1];vout.z=p[2];vout.u=v.uv[0]+scroll[0];vout.v=v.uv[1]+scroll[1];
            const auto clip=apply(screen,{p[0],p[1],p[2],1}),g=apply(guard,{p[0],p[1],p[2],1});projected.push_back(clip);
            outside.push_back(std::fabs(g[0])>std::fabs(g[3]) || std::fabs(g[1])>std::fabs(g[3]) || std::fabs(g[2])>std::fabs(g[3]));
            if(mat.fog && clip[3]>0)vout.fog=std::clamp(camera->fog[0]+camera->fog[1]/clip[3],0.0f,255.0f)/255;
            std::array<float,3> e{eye[0]-p[0],eye[1]-p[1],eye[2]-p[2]};const auto length=std::sqrt(dot(e,e));
            if(m.has(36)){const float d=dot(n,e),inv=length?1/length:0;std::array<float,3> reflection{2*d*n[0]-e[0],2*d*n[1]-e[1],2*d*n[2]-e[2]},view{};
                for(unsigned r=0;r<3;++r)view[r]=(viewObject[4*r]*reflection[0]+viewObject[4*r+1]*reflection[1]+viewObject[4*r+2]*reflection[2])*inv;
                const float q=std::sqrt(std::max(1-view[2],0.1f));vout.u=(-view[0]+1.41421f*q)/(2.82843f*q);vout.v=(view[1]+1.41421f*q)/(2.82843f*q);}
            std::array<float,4> factor{1,1,1,objectColour[3]};
            if(material[4]==0){factor=objectColour;if(!tableA && !plain)factor[0]=factor[1]=factor[2]=0;}
            else if(material[4]==1){for(unsigned c=0;c<3;++c){factor[c]=ambient[c];for(unsigned l=0;l<4;++l)factor[c]+=colours[l][c]*std::max(0.0f,dot(directions[l],n));}}
            else if(material[4]==3){
                if(!block)throw std::runtime_error("native specular light table without a mesh block");
                // The fourth light depends on the VU1 program set that builder 23a950
                // installs: with the global light byte +517 (sets b-d, 11-13) it is
                // weighted by header 12.x, not clamped and has no specular; without
                // it (sets 0-2, e-10) it is lit like the other three. The byte is
                // recorded per model by its setup 23c0d0 (vp2s_vu1_hle mode 3).
                const bool fourth=colours[3][0]!=0 || colours[3][1]!=0 || colours[3][2]!=0;
                bool plainFourth=false;
                if(fourth){const auto variant=nativeModelLightVariant(rt,object);
                    if(!variant)throw std::runtime_error(std::format("native specular fourth light without its model setup (block {:x})",block));plainFourth=*variant;}
                const auto wa=real(block+0x60),wd=real(block+0x64),ws=real(block+0x68);const auto power=value<int32_t>(rt,block+0x3c);
                if(power<0 || power>16)throw std::runtime_error("native specular power outside bounds");
                float diffuse[4],specular[4];for(unsigned l=0;l<4;++l){const bool plain=l==3 && plainFourth;
                    diffuse[l]=plain?wa*dot(directions[l],n):std::max(0.0f,wd*dot(directions[l],n));
                    std::array<float,3> h{};for(unsigned c=0;c<3;++c)h[c]=directions[l][c]*ws+e[c]*ws*(length?1/length:0);
                    specular[l]=dot(h,n);for(int j=0;j<power;++j)specular[l]*=specular[l];if(plain)specular[l]=0;}
                for(unsigned c=0;c<3;++c){factor[c]=ambient[c]*wa;for(unsigned l=0;l<4;++l)factor[c]+=colours[l][c]*diffuse[l];
                    factor[c]+=colours[0][c]*specular[0];factor[c]+=colours[3][c]*specular[3];factor[c]+=colours[1][c]*specular[1];factor[c]+=colours[2][c]*specular[2];}}
            if(pointLights || !spots.empty()){
                int32_t power=0;std::array<float,4> weights{};
                if(material[4]==3 || pointLights){if(!block)throw std::runtime_error("native point/spot lights without a mesh block");
                    power=value<int32_t>(rt,block+0x3c);if(power<0 || power>16)throw std::runtime_error("native light power outside bounds");}
                if(material[4]==3){if(pointLights)throw std::runtime_error("unported native point lights in specular mode 3");
                    if(power<=0)throw std::runtime_error("unported native spot light with specular power 0");weights=value<std::array<float,4>>(rt,block+0x50);}
                else if(material[4]!=0 && material[4]!=1)throw std::runtime_error(std::format("unported native point/spot lights in mode {}",material[4]));
                for(unsigned i=0;pointLights && i<3;++i){const auto &l=points[i];std::array<float,3> d{l.position[0]-p[0],l.position[1]-p[1],l.position[2]-p[2]};
                    const float squared=dot(d,d),q=squared!=0?l.strength/squared:0;float lit=std::max(0.0f,(n[0]*d[0]+n[1]*d[1]+n[2]*d[2])*q);
                    for(int32_t r=0;r<power;++r)lit*=lit;for(unsigned c=0;c<3;++c)factor[c]+=l.colour[c]*lit;}
                for(const auto &l:spots){std::array<float,3> d{l.position[0]-p[0],l.position[1]-p[1],l.position[2]-p[2]};
                    const float dl=std::sqrt(dot(d,d)),inverse=dl!=0?1/dl:0;std::array<float,3> unit{d[0]*inverse,d[1]*inverse,d[2]*inverse};
                    const float cosine=dot(l.direction,unit),cone=cosine>0?std::clamp((cosine*cosine-l.cone[0])*l.cone[1],0.0f,1.0f):0.0f;
                    const float strength=l.cone[2]*inverse;float diffuse=std::max(0.0f,(unit[0]*n[0]+unit[1]*n[1]+unit[2]*n[2])*strength*cone);
                    if(material[4]!=3){for(unsigned c=0;c<3;++c)factor[c]+=l.colour[c]*diffuse;continue;}
                    const float el=std::sqrt(dot(e,e)),q=el!=0?1/el:std::numeric_limits<float>::max();
                    std::array<float,3> half{};for(unsigned c=0;c<3;++c)half[c]=(e[c]*q+unit[c])*weights[2];
                    float sp=std::max(0.0f,dot(half,n));for(int32_t r=0;r<power;++r)sp*=sp;const float specular=sp*cone*strength;diffuse*=weights[1];
                    for(unsigned c=0;c<3;++c)factor[c]+=l.colour[c]*diffuse+l.colour[c]*specular;}
            }
            vout.rgba=0;for(unsigned c=0;c<4;++c){finite(factor[c]);const auto lit=uint32_t(std::clamp(v.colour[c]*factor[c],0.0f,limit[c]>0?std::min(limit[c],255.0f):255.0f));vout.rgba|=lit<<(8*c);}
            for(auto f:{vout.x,vout.y,vout.z,vout.u,vout.v,vout.fog})finite(f);
        }
        const bool cull=(block?word(block+0x90)==0:bool(material[5])) && !byte(object+0x210);size_t first=0;
        for(auto length:m.strips){for(unsigned k=2;k<length;++k){const auto a=first+k-2,b=first+k-1,c=first+k;
            if(outside[a] || outside[b] || outside[c])continue;
            if(cull && projected[a][3]>0 && projected[b][3]>0 && projected[c][3]>0){const auto &p=projected[a],&q=projected[b],&r=projected[c];
                const float area=(p[0]/p[3]-q[0]/q[3])*(q[1]/q[3]-r[1]/r[3])-(p[1]/p[3]-q[1]/q[3])*(q[0]/q[3]-r[0]/r[3]);
                if(!std::signbit(area*((k&1)?-1.0f:1.0f)))continue;}
            batch.vertices.push_back(vertices[a]);batch.vertices.push_back(vertices[(k&1)?c:b]);batch.vertices.push_back(vertices[(k&1)?b:c]);}first+=length;}
        if(!batch.vertices.empty())out.batches.push_back(std::move(batch));
    }
    return out;
}
std::optional<bool> nativeModelInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x239ff0)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<ModelOwner *>(&d->host()):nullptr;auto *scene=d?dynamic_cast<SceneOwner *>(&d->host()):nullptr;
    auto *assets=d?dynamic_cast<AssetOwner *>(&d->host()):nullptr;auto *textures=d?dynamic_cast<TextureOwner *>(&d->host()):nullptr;
    if(!c || !owner || !scene || !assets || !textures || c->pc!=0x239fd0 || c->r[5].uw[0]<0x100650)
        throw std::runtime_error("native model submission requires its complete scene producer");
    const auto object=c->r[5].uw[0]-0x650;
    NativeModelSubmission submission;
    try{submission=owner->nativeModels().capture(rt,assets->nativeAssets(),textures->nativeTextures(),scene->nativeScene(),object);}
    catch(const std::runtime_error &e){throw std::runtime_error(std::format("native model object={:08x}: {}",object,e.what()));}
    owner->drawNativeModel(submission);c->r[4].sd[0]=int32_t(object);
    return true; // Original compiled scene completion follows accepted native draws.
}
}

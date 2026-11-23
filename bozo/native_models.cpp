// Native mesh assets, game object records and scalar rendering mathematics.
// No runtime packet decoding, transport replay or vector-machine state.
#include "native_models.h"
#include "native_math.h"
#include "bozo/ee/dispatcher.h"
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <format>
#include <iterator>
#include <limits>
#include <set>
#include <source_location>
namespace vp2s {
namespace {bool traceModels=false;}
void setNativeModelTrace(bool on){traceModels=on;}
struct NativeModels::Mesh {
    struct Vertex {float p[4],n[3],uv[2];uint8_t colour[4];};
    struct Bone {int32_t a;float wa;int32_t b;float wb;};
    struct Source {uint32_t normal,position;};
    uint32_t group=0,sourceSize=0;
    std::array<int32_t,4> algorithms{};
    std::array<uint32_t,11> material{};
    bool hasMaterial=false;
    int32_t scroll=-1;
    // Routine 26's data row: alpha base (.x), alpha on (.y), alpha scale (.w).
    struct Reflection {float base=0;int32_t on=0;float scale=0;} reflection;
    ScreenProjection screen; // routine 40's data rows
    std::vector<uint16_t> strips;
    std::vector<Vertex> vertices;
    std::vector<Bone> bones;
    std::vector<Source> sources;
    bool has(int n)const{return std::find(algorithms.begin(),algorithms.end(),n)!=algorithms.end();}
};
void NativeModels::serialize(bozo::state::Archive &a){
    namespace st=bozo::state;
    const auto mesh=[&](Mesh &m){
        st::fields(a,m.group,m.sourceSize,m.algorithms,m.material,m.hasMaterial,m.scroll,m.strips);
        st::pod(a,m.reflection); // (routine 40 data is never saved: run-time meshes reject routine 40)
        const auto podVector=[&](auto &v){auto n=st::count(a,v.size());if(a.loading())v.resize(n);for(auto &e:v)st::pod(a,e);};
        podVector(m.vertices);podVector(m.bones);podVector(m.sources);};
    auto n=st::count(a,generated.size());
    if(a.loading()){generated.clear();
        for(uint64_t i=0;i<n;++i){uint32_t at=0;st::io(a,at);auto m=std::make_shared<Mesh>();mesh(*m);auto &g=generated[at];g.mesh=std::move(m);st::io(a,g.bytes);}}
    else for(auto &[at,g]:generated){uint32_t key=at;st::io(a,key);Mesh copy=*g.mesh;mesh(copy);st::io(a,g.bytes);}
}
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
// r = 2(n.e)n - e; v = Rn r; q = sqrt(8 - 8 v.y); (S, T, Q) = (v.x + q/2,
// v.z + q/2, q) as 12-bit fixed point (FTOI12 truncates). The GS divides S
// and T by Q: u = v.x/q + 1/2, the classic sphere map around the world Y axis.
std::array<int32_t,3> sphereMapTexture(const std::array<float,3> &n,const std::array<float,3> &eye,const float (&rotation)[3][3]){
    const float d=(n[0]*eye[0]+n[1]*eye[1])+n[2]*eye[2];
    std::array<float,3> r{},v{};for(unsigned c=0;c<3;++c)r[c]=(n[c]*d+n[c]*d)-eye[c];
    for(unsigned j=0;j<3;++j)v[j]=(rotation[j][0]*r[0]+rotation[j][1]*r[1])+rotation[j][2]*r[2];
    const float q=std::sqrt(std::fabs(8-8*v[1]));   // VU SQRT takes |x|
    const auto fixed=[](float x){const double s=std::trunc(double(x)*4096);return int32_t(std::clamp(s,-2147483648.0,2147483647.0));};
    return {fixed(v[0]+.5f*q),fixed(v[2]+.5f*q),fixed(q)};
}
ReflectionTexture reflectionTexture(const std::array<float,3> &n,const std::array<float,3> &p,const std::array<float,3> &eye,
                                    const float (&rotation)[3][3],float base,int32_t on,float scale){
    const std::array<float,3> d{eye[0]-p[0],eye[1]-p[1],eye[2]-p[2]};
    const float length=std::sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]),inv=length!=0?1/length:0;   // ERLENG
    const float c=(n[0]*d[0]+n[1]*d[1])+n[2]*d[2];
    std::array<float,3> r{},v{};for(unsigned k=0;k<3;++k)r[k]=(n[k]*c+n[k]*c)-d[k];
    for(unsigned j=0;j<3;++j)v[j]=(rotation[j][0]*r[0]+rotation[j][1]*r[1])+rotation[j][2]*r[2];v[1]=-v[1];
    const float x=v[0]*inv,y=1+v[1]*inv,z=v[2]*inv,q=std::sqrt(std::fabs(y));   // VU SQRT takes |x|
    const auto fixed=[](float f){const double s=std::trunc(double(f)*4096);return int32_t(std::clamp(s,-2147483648.0,2147483647.0));};
    ReflectionTexture out{{fixed(x+1.41421f*q),fixed(z+1.41421f*q),fixed(2.82843f*q)},std::nullopt};
    if(on){const float f=1-c*inv,a=base+scale*(f*f);out.alpha=int32_t(std::clamp(std::trunc(double(a)),-2147483648.0,2147483647.0));}   // FTOI0
    return out;
}
// The VU's order: ACC chains (n push) + p and ((g0 x + g1 y) + g2 z) + g3;
// DIV Q = 1/w; (c s) + (o w), times Q; FTOI12 truncates and saturates.
std::array<int32_t,2> screenTexture(const std::array<float,3> &n,const std::array<float,3> &p,const ScreenProjection &data,
                                    const std::array<float,2> &row38,const std::array<float,2> &row39,const std::array<float,16> &guard){
    std::array<float,3> q{};for(unsigned c=0;c<3;++c)q[c]=n[c]*data.push+p[c];
    std::array<float,4> clip{};for(unsigned r=0;r<4;++r)clip[r]=((guard[4*r]*q[0]+guard[4*r+1]*q[1])+guard[4*r+2]*q[2])+guard[4*r+3];
    const float w=clip[3],inverse=w!=0?1/w:std::copysign(std::numeric_limits<float>::max(),w);
    const auto fixed=[](float f){if(std::isnan(f))throw std::runtime_error("nonfinite native screen-projected texture");
        const double s=std::trunc(double(f)*4096);return int32_t(std::clamp(s,-2147483648.0,2147483647.0));};
    std::array<int32_t,2> out{};
    for(unsigned c=0;c<2;++c){const float s=row38[c]*data.scale[c],o=row39[c]+data.offset[c];out[c]=fixed((clip[c]*s+o*w)*inverse);}
    return out;
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
        if(m->has(26))m->reflection=r.take<Mesh::Reflection>();
        if(m->has(40)){m->screen=r.take<ScreenProjection>();for(auto f:{m->screen.offset[0],m->screen.offset[1],m->screen.push,m->screen.scale[0],m->screen.scale[1]})finite(f);}
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
// The sprite batcher's mesh, from its sections (see native_models.h). Header
// payload: V4-16 rows 0..6 as the writer's halfword stores leave them (row 0:
// size, vertex row, strip count, length row; row 2: flags, vertex count; row 5:
// routines; row 6.y: group). Lanes, each behind the VIF word the writer composes:
// strip lengths (S-8 unsigned), then with STCYCL 4/1 normals (V4-32, row v),
// UVs (V2-16 signed, v+1), colours (V4-8 unsigned, v+2), positions (V4-32, v+3)
// and the closing STCYCL 1/1. Values become vertices exactly as tools/vp2_meshes.py
// converts disc meshes (routines 6/8 fixed point, UV /4096); materials as
// tools/vp2_bozo_meshes.py names them.
void NativeModels::publishGeneratedMesh(Runtime *rt,const std::vector<std::pair<uint32_t,uint32_t>> &input){
    if(input.size()!=5 && input.size()!=6 && input.size()!=7)throw std::runtime_error("native generated mesh has an unknown section count");
    std::vector<std::pair<uint32_t,uint32_t>> s;for(const auto &[at,q]:input)s.push_back({NativeAssets::address(at),q});
    uint64_t total=0;for(size_t i=0;i<s.size();++i){if(i && s[i].first!=s[i-1].first+16*s[i-1].second)throw std::runtime_error("native generated mesh sections are not contiguous");total+=16ull*s[i].second;}
    const auto mesh=s[0].first;
    if(s[0].second!=4 || value<uint32_t>(rt,mesh)!=0x6d078000u)throw std::runtime_error("native generated mesh header escaped the writer's layout");
    const auto half=[&](unsigned i){return value<int16_t>(rt,mesh+4+2*i);};
    const unsigned size=uint16_t(half(0));const int32_t vertexRow=half(1),strips=half(2),lengthRow=half(3),flags=half(8),count=half(9),group=half(25);
    const bool material=flags&1;
    if(size!=total || s.size()<5u+material || s.size()>6u+material || count<1 || count>1024 || strips<1 || strips>count || vertexRow<0 || vertexRow+4*count>1024 || lengthRow<0 || lengthRow>=1024)
        throw std::runtime_error(std::format("native generated mesh header outside its sections (size {} of {}, {} vertices, {} strips)",size,total,count,strips));
    auto m=std::make_shared<Mesh>();m->group=uint32_t(group);m->sourceSize=size;
    for(unsigned r=0;r<4;++r)m->algorithms[r]=half(20+r);
    bool fixed16=false,fixed4096=false;
    for(auto r:m->algorithms){
        if(r==8)fixed16=fixed4096=true;else if(r==6)fixed4096=true;
        else if(r!=0 && r!=12 && r!=14 && r!=16 && r!=18 && r!=22 && r!=24 && r!=28 && r!=32 && r!=34 && r!=38 && r!=42 && r!=46 && r!=50 && r!=52 && r!=54)
            throw std::runtime_error(std::format("unported generated mesh routine {}",r));
    }
    size_t k=1;
    const auto lane=[&](uint32_t code,uint32_t qwords){
        if(k>=s.size() || s[k].second!=qwords || value<uint32_t>(rt,s[k].first+12)!=code)
            throw std::runtime_error(std::format("native generated mesh lane {} escaped the writer's layout",k));
        return s[k++].first+16;};
    if(material){
        const auto rows=lane(0x6c0f8007u,16);const auto row=[&](unsigned r,unsigned c){return value<uint32_t>(rt,rows+16*(r-7)+4*c);};
        const auto alpha=row(8,0),mode=row(8,1),tex=row(9,1),prim=(row(13,1)>>15)&0x7ff,cull=row(15,0);
        m->hasMaterial=true;// material[6]: bit 0 PRIM.FGE; bits 1 / 2 a LINE STRIP / LINE primitive (PRIM & 7 = 2 / 1), drawn as segments.
        m->material={uint32_t(bool(prim&64)),uint32_t(bool(prim&16)),uint32_t(bool(tex&4)),(tex>>3)&3,mode,uint32_t(cull==0),
            uint32_t(bool(prim&32))|((prim&7)==2?2u:0u)|((prim&7)==1?4u:0u),
            alpha&3,(alpha>>2)&3,(alpha>>4)&3,(alpha>>6)&3};
        const auto &x=m->material;
        if(x[4]>3 || x[7]>2 || x[8]>2 || x[9]>2 || x[10]>2)throw std::runtime_error("invalid generated native material");
    }
    const auto n=uint32_t(count);
    const auto lengths=lane(0x62000000u|uint32_t(strips)<<16|0xc000u|uint32_t(lengthRow),(uint32_t(strips)+15)/16+1);
    if(value<uint32_t>(rt,lengths-16+8)!=0)throw std::runtime_error("native generated mesh strip lane escaped the writer's layout");
    // Normals: absent when the battle writer subclass 174f90 has +d1 set (mod028 4d7858 skips
    // the lane; 4d7a28 puts the STCYCL 4/1 before the UV lane instead). The VU rows keep
    // whatever an earlier mesh left there, so they are NaN here and the draw refuses every
    // path that reads a normal (routines 20/26/36/40, lighting modes 1/3, point/spot lights).
    // Otherwise V4-32, or V4-16 signed (the battle writer subclass 174f90 with +d0 set packs
    // FTOI12 halves, mod028 4d71cc): VIF sign-extends them to the same 32-bit integers.
    const bool packedNormals=k<s.size() && value<uint32_t>(rt,s[k].first+12)==(0x6d000000u|n<<16|0x8000u|uint32_t(vertexRow));
    if(packedNormals && !fixed4096)throw std::runtime_error("native generated mesh packs normals its routines read as floats");
    const uint32_t uvCode=0x75000000u|n<<16|0x8000u|uint32_t(vertexRow+1);
    const bool hasNormals=!(k<s.size() && value<uint32_t>(rt,s[k].first+12)==uvCode);
    if(s.size()!=5u+material+hasNormals)throw std::runtime_error("native generated mesh section count differs from its lanes");
    if(!hasNormals && (m->has(2) || m->has(4) || m->has(20) || m->has(26) || m->has(36) || m->has(40) || (material && m->material[4]!=0)))
        throw std::runtime_error("native generated mesh without normals runs a routine or lighting that reads them");
    const auto normals=!hasNormals?0u:packedNormals?lane(0x6d000000u|n<<16|0x8000u|uint32_t(vertexRow),(n*8+15)/16+1):lane(0x6c000000u|n<<16|0x8000u|uint32_t(vertexRow),n+1);
    if(value<uint32_t>(rt,(hasNormals?normals:s[k].first+16)-16+8)!=0x01000104u)throw std::runtime_error("native generated mesh stride escaped the writer's layout");
    const auto uvs=lane(0x75000000u|n<<16|0x8000u|uint32_t(vertexRow+1),(n+3)/4+1);
    const auto colours=lane(0x6e000000u|n<<16|0xc000u|uint32_t(vertexRow+2),(n+3)/4+1);
    // Positions: V4-32, or V3-16 signed (the battle writer subclass with +d0 = 2 packs FTOI4
    // halves, mod028 4d7260): sign-extended to the same integers; W is not unpacked, so only
    // routines that never read it (no skinning by index, 2) take that lane.
    const bool packedPositions=k<s.size() && value<uint32_t>(rt,s[k].first+12)==(0x69000000u|n<<16|0x8000u|uint32_t(vertexRow+3));
    if(packedPositions && (!fixed16 || m->has(2)))
        throw std::runtime_error("native generated mesh packs positions its routines read as floats or by W");
    const uint32_t positionBytes=packedPositions?(n*6+15)/16*16:16*n;
    const auto positions=packedPositions?lane(0x69000000u|n<<16|0x8000u|uint32_t(vertexRow+3),positionBytes/16+2):lane(0x6c000000u|n<<16|0x8000u|uint32_t(vertexRow+3),n+2);
    if(k!=s.size() || value<uint32_t>(rt,positions+positionBytes)!=0x01000101u)throw std::runtime_error("native generated mesh does not close as the writer's");
    uint32_t sum=0;for(int32_t i=0;i<strips;++i){const auto l=value<uint8_t>(rt,lengths+i);m->strips.push_back(l);sum+=l;}
    if(sum!=n)throw std::runtime_error("native generated mesh strip extent mismatch");
    for(uint32_t i=0;i<n;++i){Mesh::Vertex v{};
        for(unsigned c=0;c<3;++c){const auto pw=packedPositions?uint32_t(int32_t(value<int16_t>(rt,positions+6*i+2*c))):value<uint32_t>(rt,positions+16*i+4*c);
            const auto nw=!hasNormals?0x7fc00000u:packedNormals?uint32_t(int32_t(value<int16_t>(rt,normals+8*i+2*c))):value<uint32_t>(rt,normals+16*i+4*c);
            v.p[c]=fixed16?float(int32_t(pw))/16.f:std::bit_cast<float>(pw);v.n[c]=!hasNormals?std::bit_cast<float>(nw):fixed4096?float(int32_t(nw))/4096.f:std::bit_cast<float>(nw);}
        v.p[3]=packedPositions?0.f:float(value<int32_t>(rt,positions+16*i+12));
        for(unsigned c=0;c<2;++c)v.uv[c]=float(value<int16_t>(rt,uvs+4*i+2*c))/4096.f;
        for(unsigned c=0;c<4;++c)v.colour[c]=value<uint8_t>(rt,colours+4*i+c);
        for(float f:{v.p[0],v.p[1],v.p[2]})finite(f);
        if(hasNormals)for(float f:{v.n[0],v.n[1],v.n[2]})finite(f);
        m->vertices.push_back(v);m->sources.push_back({UINT32_MAX,UINT32_MAX});}
    for(auto it=generated.begin();it!=generated.end();)
        if(it->first<mesh+size && mesh<it->first+it->second.bytes.size())it=generated.erase(it);else ++it;
    const auto *bytes=rt->memory.hostPointer(mesh,size);if(!bytes)throw std::runtime_error("native generated mesh unmapped");
    generated[mesh]=Generated{std::move(m),std::vector<uint8_t>(bytes,bytes+size)};
}
// The scene's pending object owns a model it is, or one its composite draw
// (class 1599a0, method 2b7ab0) reaches: for slot 2..0, each list child's slot
// +8c0+4i, then its own; then each list child (list +8d4, next +8, ending at
// the owner's +8cc). Children of that class compose the same way; their draws
// run inside the owner's, and their completions (21c9a0) leave the scene slot.
bool composedBy(Runtime *rt,uint32_t owner,uint32_t object){
    constexpr uint32_t kComposite=0x1599a0;
    std::vector<uint32_t> stack{owner},seen;
    while(!stack.empty()){
        const auto node=stack.back();stack.pop_back();
        if(node==object)return true;
        if(node<0x100000 || node>0x2000000-0x8d8 || std::find(seen.begin(),seen.end(),node)!=seen.end())continue;
        seen.push_back(node);if(seen.size()>512)throw std::runtime_error("native composite model tree outside bounds");
        if(value<uint32_t>(rt,node)!=kComposite)continue;
        for(unsigned i=0;i<3;++i)if(const auto slot=value<uint32_t>(rt,node+0x8c0+4*i))stack.push_back(slot);
        unsigned steps=0;
        for(auto child=value<uint32_t>(rt,node+0x8d4);child!=node+0x8cc;child=value<uint32_t>(rt,child+8)){
            if(child<0x100000 || child>0x2000000-0x8d8 || ++steps>256)throw std::runtime_error("native composite model list outside bounds");
            stack.push_back(child);for(unsigned i=0;i<3;++i)if(const auto slot=value<uint32_t>(rt,child+0x8c0+4*i))stack.push_back(slot);
        }
    }
    return false;
}
void nativeShadowPrisms(const std::vector<std::array<float,3>> &positions,const std::vector<uint16_t> &strips,const std::array<float,3> &light,
                        const std::array<float,16> &top,const std::array<float,16> &flat,
                        std::vector<bozo::render::Vertex> &increment,std::vector<bozo::render::Vertex> &decrement){
    size_t total=0;for(auto length:strips)total+=length;
    if(total!=positions.size())throw std::runtime_error("native shadow strips differ from its vertices");
    using P=std::array<float,3>;
    const auto sub=[](const P &a,const P &b){return P{a[0]-b[0],a[1]-b[1],a[2]-b[2]};};
    const auto project=[](const std::array<float,16> &m,const P &p,P &s){
        float c[4];for(unsigned r=0;r<4;++r)c[r]=m[r]*p[0]+m[4+r]*p[1]+m[8+r]*p[2]+m[12+r];
        if(!(c[3]>0) || !std::isfinite(c[0]/c[3]) || !std::isfinite(c[1]/c[3]) || !std::isfinite(c[2]/c[3]))return false;
        s={c[0]/c[3],c[1]/c[3],c[2]/c[3]};return true;};
    size_t first=0;
    for(auto length:strips){
        for(unsigned k=2;k<length;++k){
            P a=positions[first+k-2],b=positions[first+k-1];const P c=positions[first+k];
            if((k-2)&1)std::swap(a,b);
            const P e1=sub(b,a),e2=sub(c,a),n{e1[1]*e2[2]-e1[2]*e2[1],e1[2]*e2[0]-e1[0]*e2[2],e1[0]*e2[1]-e1[1]*e2[0]};
            if(light[0]*n[0]+light[1]*n[1]+light[2]*n[2]<0)continue;
            P s[6];if(!project(top,a,s[0]) || !project(top,b,s[1]) || !project(top,c,s[2]) || !project(flat,a,s[3]) || !project(flat,b,s[4]) || !project(flat,c,s[5]))continue;
            const unsigned order[10]={0,1,2,5,0,3,1,4,5,3};   // a b c c' a a' b b' c' a'
            for(unsigned i=0;i<8;++i){
                const P &p0=s[order[i]],&p1=s[order[i+1]],&p2=s[order[i+2]];
                float area=(p1[0]-p0[0])*(p2[1]-p0[1])-(p1[1]-p0[1])*(p2[0]-p0[0]);if(i&1)area=-area;
                if(area==0)continue;
                auto &out=area>0?increment:decrement;
                // Stored in the strip's own winding (odd triangles' last two swapped): increments
                // have a positive screen area, decrements a negative one.
                for(const P *q:{&p0,(i&1)?&p2:&p1,(i&1)?&p1:&p2}){bozo::render::Vertex v;v.x=(*q)[0];v.y=(*q)[1];v.z=(*q)[2];v.rgba=0;out.push_back(v);}
            }
        }
        first+=length;
    }
}
std::vector<NativeShadowDraw> nativeShadowPass(const NativeModelSubmission &s,float width,float height){
    std::vector<NativeShadowDraw> draws;
    if(!s.shadow || (s.shadowIncrement.empty() && s.shadowDecrement.empty()))return draws;
    if(!(width>=1 && height>=1 && width<=2048 && height<=2048))throw std::runtime_error("native shadow pass target outside bounds");
    bozo::render::Material volume;volume.textured=false;volume.colourMask=0;volume.depthTest=true;volume.depthGreater=true;volume.depthWrite=false;
    volume.blend=bozo::render::Material::Blend::Opaque;volume.alphaBlend=false;volume.alphaTest=false;volume.stencilClear=true;
    for(const auto *faces:{&s.shadowIncrement,&s.shadowDecrement}){
        if(faces->empty())continue;
        volume.stencilOp=faces==&s.shadowIncrement?bozo::render::Material::StencilOp::IncrementWrap:bozo::render::Material::StencilOp::DecrementWrap;
        draws.push_back({volume,*faces});volume.stencilClear=false;
    }
    bozo::render::Material dark;dark.textured=false;dark.depthTest=false;dark.depthWrite=false;dark.alphaTest=false;dark.stencilNotZero=true;
    dark.blend=bozo::render::Material::Blend::Alpha;dark.gsBlend=true;dark.blendA=0;dark.blendB=1;dark.blendC=0;dark.blendD=1;dark.blendFix=0x80;
    const float xy[6][2]={{0,0},{1,0},{0,1},{1,0},{1,1},{0,1}};std::vector<bozo::render::Vertex> quad(6);
    for(unsigned i=0;i<6;++i){quad[i].x=2048-width/2+xy[i][0]*width;quad[i].y=2048-height/2+xy[i][1]*height;quad[i].rgba=0x3f000000u;}
    draws.push_back({dark,quad});
    return draws;
}
NativeModelSubmission NativeModels::capture(Runtime *rt,NativeAssets &assets,NativeTextures &textures,const NativeScene &scene,uint32_t object,const NativeFeedbackCopies *feedback){
    const auto word=[&](uint32_t at,std::source_location source=std::source_location::current()){return value<uint32_t>(rt,at,source);};
    const auto byte=[&](uint32_t at){return value<uint8_t>(rt,at);};
    const auto real=[&](uint32_t at){const auto v=value<float>(rt,at);finite(v);return v;};
    const auto row=[&](uint32_t n){auto v=value<std::array<float,4>>(rt,object+0x230+16*n);for(auto f:v)finite(f);return v;};
    const auto camera=scene.drawingCamera();const auto target=scene.frameTarget();
    if(!camera || !target)throw std::runtime_error("native model has no active camera/frame");
    const auto current=word(0x35ddf8);const auto layer=byte(object+0x65);
    if(layer>1 || byte(current+0x40dc+layer)!=1 || !composedBy(rt,word(current+0x40a8+4*layer),object))
        throw std::runtime_error("native model has no pending scene ownership");
    // Shadow classes 159880 and 1599a0: their slot +60 (2b86f0, where ordinary
    // models have 23c6c0) builds the object's own model packet with 2b6640
    // (VU1 set 0xa, entry 0x6f0) as a shadow volume. On the GS it writes only
    // the frame's alpha (a stencil count), never colour; it becomes stencil
    // faces here (nativeShadowPrisms). Drawn as a lit mesh it once laid a black
    // alpha-64 film over the character.
    const auto cls=word(object);const bool shadowClass=cls==0x159880 || cls==0x1599a0;
    if(shadowClass && word(cls+0x60)!=0x2b86f0)throw std::runtime_error(std::format("native shadow class {:x} without its 2b86f0 set-up",cls));
    const auto owner=word(current+0x40a8+4*layer);
    const auto deferUntil=[&]()->uint32_t{if(shadowClass || owner==object)return 0;const auto c=word(owner);return c==0x159880 || c==0x1599a0?owner:0;}();
    const auto model=word(object+0x7bc),count=word(model+0x34),list=word(object+0x77c);
    if(count>4096)throw std::runtime_error(std::format("native model mesh count outside bounds: model={:08x} count={}",model,count));
    if(!count){
        if((model&15) || model<0x100010)throw std::runtime_error("empty native model lacks aligned FPS header");
        const auto header=assets.origin(model-0x10,0x70);
        if(!header || header->kind!=AssetOrigin::Kind::Part || word(model-0x10)!=0x00535046)
            throw std::runtime_error("empty native model lacks complete decoded FPS header");
        // All four original mesh loops test count > 0 before reading a mesh.
        // No vertex, material, skeleton or lighting input is consumed here.
        NativeModelSubmission empty{object,camera->object};empty.shadow=shadowClass;empty.deferUntil=deferUntil;return empty;
    }
    const auto world=value<Mat>(rt,object+0x90);for(auto f:world)finite(f);
    const auto viewObject=product(camera->view,world),screen=product(camera->screen,viewObject),guard=product(camera->guard,viewObject);
    NativeModelSubmission out{object,camera->object};out.shadow=shadowClass;out.deferUntil=deferUntil;
    // The camera draws into the frame whose GS origin is its viewport (+260): the frame is
    // 2*(2048 - origin) wide, scaled to the native target about the frame centre 2048.
    // An off-centre camera (the forest minimap: centre 1894,1907) keeps its own centre
    // inside that frame; its centre is not the frame's.
    Mat matrix{};const float width=2*(2048.f-camera->viewport[0]),height=2*(2048.f-camera->viewport[1]);
    for(const auto &m:{camera->view,camera->screen,camera->guard,viewObject,screen,guard})for(auto f:m)finite(f);
    finite(width);finite(height);
    if(width<=16 || height<=16 || width>=4096 || height>=4096)throw std::runtime_error("native model camera viewport outside bounds");
    for(unsigned c=0;c<4;++c){matrix[4*c]=screen[c]*target->width/width+(1-target->width/width)*2048.f*screen[12+c];
        matrix[4*c+1]=screen[4+c]*target->height/height+(1-target->height/height)*2048.f*screen[12+c];
        matrix[4*c+2]=screen[8+c];matrix[4*c+3]=screen[12+c];}
    std::array<std::array<float,3>,4> directions{};std::array<std::array<float,4>,4> colours{};
    float rotation[3][3];for(unsigned r=0;r<3;++r){const float length=std::sqrt(world[4*r]*world[4*r]+world[4*r+1]*world[4*r+1]+world[4*r+2]*world[4*r+2]);
        for(unsigned c=0;c<3;++c)rotation[r][c]=length?world[4*r+c]/length:0;}
    for(unsigned i=0;i<4;++i){const auto d=row(i+1);colours[i]=row(i+5);for(unsigned c=0;c<3;++c)directions[i][c]=rotation[0][c]*d[0]+rotation[1][c]*d[1]+rotation[2][c]*d[2];}
    const auto ambient=row(19),objectColour=row(13),limit=row(0);
    const bool tableA=word(object+0x31c)!=0;
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
    // VU row 72: the eye's direction in object space (object set-up, see vp2s_vu1_hle runObjectSetup).
    std::array<float,3> eyeDirection{};{const float length=std::sqrt(dot(eye,eye));if(length)for(unsigned c=0;c<3;++c)eyeDirection[c]=eye[c]/length;}
    uint32_t boneCount=0,palette=0;const auto skeleton=word(object+0x648);
    if(skeleton){boneCount=word(word(skeleton+0x34)+4);if(boneCount>4096)throw std::runtime_error("native bone count outside bounds");
        palette=word(skeleton+0x4c)+0x40+64*boneCount*(byte(skeleton+0x50)&1);}
    // Shadow inputs as 2b82f0 leaves them in the object's VU rows: mem[37] the
    // world light (object set-up 0x238 turns a direction by Rn^T), mem[14].w 0
    // for a directional light, mem[5..8] (rows, as the world matrix at +90) the
    // object-to-world matrix flattened along the light onto the ground. The VU
    // set-up projects it as it does the world matrix: P^T V F (checked against
    // 23 recorded shadow runs, relative error < 1e-7).
    std::array<float,3> shadowLight{};Mat shadowFlat{};
    if(shadowClass){
        if(word(object+0x31c))throw std::runtime_error("unported native point-light shadow (mem[14].w)");
        const auto l=row(37);for(unsigned c=0;c<3;++c)shadowLight[c]=rotation[0][c]*l[0]+rotation[1][c]*l[1]+rotation[2][c]*l[2];
        Mat ground{};for(unsigned r=0;r<4;++r){const auto line=row(5+r);for(unsigned c=0;c<4;++c)ground[4*r+c]=line[c];}
        const auto flat=product(camera->screen,product(camera->view,ground));
        for(unsigned c=0;c<4;++c){shadowFlat[4*c]=flat[c]*target->width/width+(1-target->width/width)*2048.f*flat[12+c];
            shadowFlat[4*c+1]=flat[4+c]*target->height/height+(1-target->height/height)*2048.f*flat[12+c];
            shadowFlat[4*c+2]=flat[8+c];shadowFlat[4*c+3]=flat[12+c];}
    }
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
        const auto origin=assets.origin(mesh);std::shared_ptr<const Pack> pack;std::shared_ptr<const Mesh> held;
        if(origin){pack=load(*origin);const auto found=pack->find(origin->offset);
            if(found==pack->end())throw std::runtime_error(std::format("native mesh not converted: {:x}/{}+{:x}",origin->disc,origin->part,origin->offset));
            held=found->second;}
        else{
            // A run-time mesh: only one the sprite batcher published, with every
            // byte as it was published.
            const auto g=generated.find(mesh);if(g==generated.end())throw std::runtime_error("native mesh missing allocation provenance");
            const auto *now=rt->memory.hostPointer(mesh,uint32_t(g->second.bytes.size()));
            if(!now)throw std::runtime_error("native generated mesh unmapped");
            // Except the material block's +5c (mesh +9c), the palette entry the game
            // assigns after publication and this path reads live (below).
            std::string detail;unsigned shown=0;
            // And PRIM.FGE in the material's GIF tags (rows 13 and 14, mesh +b4 / +c4): the
            // battle switches fog on after publication; the draw reads row 13's tag live.
            for(size_t at=0;at+4<=g->second.bytes.size();at+=4){
                if(at==0x9c && g->second.mesh->hasMaterial)continue;
                uint32_t was,is;std::memcpy(&was,g->second.bytes.data()+at,4);std::memcpy(&is,now+at,4);
                if((at==0xb4 || at==0xc4) && g->second.mesh->hasMaterial && (was^is)==0x00100000u)continue;
                if(was!=is && shown++<6)detail+=std::format(" +{:x}: {:08x}->{:08x}",at,was,is);}
            if(shown)throw std::runtime_error(std::format("native generated mesh {:x} changed after publication:{}",mesh,detail));
            held=g->second.mesh;}
        const auto &m=*held;
        // 22: the library's set-up table entry is a bare JR vi10 (both layouts, 0x499 / 0x483, every copy in EE RAM
        // at the battle's frame 72200), so the routine does nothing, as 46 (and as publishGeneratedMesh admits it).
        for(auto algorithm:m.algorithms)if(algorithm!=-1 && algorithm!=0 && algorithm!=2 && algorithm!=4 && algorithm!=6 && algorithm!=8 && algorithm!=12 && algorithm!=14 && algorithm!=18 && algorithm!=22 && algorithm!=28 && algorithm!=32 && algorithm!=36 && algorithm!=20 && algorithm!=26 && algorithm!=40 && algorithm!=44 && algorithm!=46 && algorithm!=48)
            throw std::runtime_error(std::format("unported native mesh algorithm {}",algorithm));
        if(m.hasMaterial)material=m.material;
        else if(origin && block>=0x40 && block-0x40!=mesh){const auto owner=assets.origin(block-0x40);if(owner && owner->disc==origin->disc && owner->part==origin->part){
            const auto other=pack->find(owner->offset);if(other!=pack->end() && other->second->hasMaterial)material=other->second->material;}}
        NativeModelBatch batch;batch.matrix=matrix;auto &mat=batch.material;bool feedbackTexture=false;
        mat.alphaBlend=material[0];mat.blend=mat.alphaBlend?bozo::render::Material::Blend::Alpha:bozo::render::Material::Blend::Opaque;
        if(mat.alphaBlend){mat.gsBlend=true;mat.blendA=material[7];mat.blendB=material[8];mat.blendC=material[9];mat.blendD=material[10];mat.blendFix=0x80;}
        mat.alphaTest=true;mat.depthWrite=!mat.alphaBlend || palette;mat.textureAlpha=material[2];
        mat.textureFunction=static_cast<bozo::render::Material::TextureFunction>(material[3]);
        // PRIM.FGE is set by the game in the vertices' GIF tag in the block in EE
        // RAM (row 13 at +70: EOP|nloop 8000, PRIM at +74 bits 15.., REGS 412 =
        // ST RGBAQ XYZF2); the converted (disc) material has it clear. This is the
        // town/harbour distance haze (PCSX2: almost every scene prim has FGE).
        const bool liveTag=block>=0x100000 && word(block+0x70)==0x8000 && word(block+0x78)==0x412;
        const bool fogEnabled=liveTag?((word(block+0x74)>>15)&0x20)!=0:(material[6]&1)!=0;
        mat.fog=fogEnabled && camera->fog[1]!=0;mat.fogColour=camera->clearColour;
        if(material[1]){if(!block || !manager)throw std::runtime_error("native textured mesh missing material/manager");
            const auto slot=value<int32_t>(rt,block+0x98);if(slot<0 || slot>=256)throw std::runtime_error("native mesh texture slot outside range");
            const auto index=value<int16_t>(rt,manager+0x2670+2*slot);if(index<0)throw std::runtime_error("native mesh texture not registered");
            const TextureKey key{word(manager+0x1008+16*index),word(manager+8+16*index)};
            if(key.tag==0x4674404c && key.index==1){
                // 'L@tF' 1 is the screen-feedback picture the game builds each frame (class 152e80,
                // registered with TEX0 at manager+1010: 512 x 512, TW = TH = 9); mode 1 filters the
                // frame into that 512 x 512 image, native image 5, so mesh UVs map onto it as they are.
                const auto image=feedback?feedback->image(scene):std::nullopt;
                const auto lo=word(manager+0x1010+16*index),hi=word(manager+0x1014+16*index);
                if(!image || image->mode!=1 || image->width!=512 || image->height!=512 || ((lo>>26)&15)!=9 || ((lo>>30)|(hi&3)<<2)!=9 || word(block+0x9c))
                    throw std::runtime_error("native mesh samples the screen feedback picture without its current 512 x 512 image");
                mat.fromFrame=1;mat.source=5;feedbackTexture=true;}
            else batch.texture=textures.load(key)->picture(word(block+0x9c));}
        mat.textured=bool(batch.texture);
        // The GS draws an untextured vertex colour as it is (0..255), not as 0x80 = 1
        // modulation: the inn's save-point light column (3e/3f/61) was twice as bright.
        mat.directColour=!mat.textured && !feedbackTexture;
        const bool editable=working && mesh>=model && !m.has(6) && !m.has(8);
        const auto source=editable?work+mesh-model:0;
        std::array<std::array<float,4>,2> screenRows{};if(m.has(40))screenRows={row(38),row(39)};
        std::array<float,2> scroll{};if(m.has(48)){if(m.scroll<0 || m.scroll>=16)throw std::runtime_error("native UV scroll index outside bounds");const auto q=row(38+m.scroll);scroll={q[2],q[3]};}
        // A generated mesh without normals (NaN, see publishGeneratedMesh) may only take paths
        // that never read one; the object's lights are known only here.
        if(!m.vertices.empty() && std::isnan(m.vertices[0].n[0]) && (material[4]!=0 || pointLights || !spots.empty() ||
           m.has(2) || m.has(4) || m.has(20) || m.has(26) || m.has(36) || m.has(40)))
            throw std::runtime_error("native generated mesh without normals drawn with a path that reads them");
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
            if(m.has(20)){
                if(m.has(2) || m.has(4) || m.has(36))throw std::runtime_error("unqualified native sphere map with skinning or routine 36");
                // Routine 20 writes S, T, Q; a later routine 48 adds its offset to S
                // and T (12-bit); the GS then divides by Q.
                auto stq=sphereMapTexture(n,eyeDirection,rotation);
                if(m.has(48)){const auto scrollAfter=std::find(m.algorithms.begin(),m.algorithms.end(),48)>std::find(m.algorithms.begin(),m.algorithms.end(),20);
                    if(scrollAfter)for(unsigned c=0;c<2;++c)stq[c]=int32_t(std::trunc((double(stq[c])/4096+scroll[c])*4096));}
                vout.u=stq[2]?float(stq[0])/float(stq[2]):.5f;vout.v=stq[2]?float(stq[1])/float(stq[2]):.5f;}
            float alphaRow=v.colour[3];
            if(m.has(26)){
                // Routine 26 runs after the skinning it follows (it reads the skinned
                // normal and position); with 20, 36 or 48 it would share the UV row.
                const auto at=std::find(m.algorithms.begin(),m.algorithms.end(),26);
                if(m.has(20) || m.has(36) || m.has(48) || std::find(at,m.algorithms.end(),2)!=m.algorithms.end() || std::find(at,m.algorithms.end(),4)!=m.algorithms.end())
                    throw std::runtime_error("unqualified native routine 26 combination");
                const auto t=reflectionTexture(n,p,eye,rotation,m.reflection.base,m.reflection.on,m.reflection.scale);
                vout.u=t.stq[2]?float(t.stq[0])/float(t.stq[2]):.5f;vout.v=t.stq[2]?float(t.stq[1])/float(t.stq[2]):.5f;
                if(t.alpha)alphaRow=float(*t.alpha);}
            if(m.has(40)){
                // Routine 40 reads the converted normal and position; every disc mesh
                // runs it after 6 or 8 (or alone), never with skinning or another UV routine.
                if(m.has(2) || m.has(4) || m.has(20) || m.has(26) || m.has(36) || m.has(48))throw std::runtime_error("unqualified native routine 40 combination");
                const auto st=screenTexture(n,p,m.screen,{screenRows[0][0],screenRows[0][1]},{screenRows[1][0],screenRows[1][1]},guard);
                vout.u=float(st[0])/4096;vout.v=float(st[1])/4096;}
            std::array<float,4> factor{1,1,1,objectColour[3]};
            // Mode 0 keeps the object colour (row 13) with or without table A. Without A
            // and with plain (mem[14].x) clear, the model program's table-D entry
            // (image 02a71b3e: 2b10 "b 2b50" with "ibeq vi2,vi0 -> 2b68" in its delay
            // slot) runs the 2b68 loop: ITOF0 colour, MUL by mem[13], MINI by mem[0].
            // The inn's save-point light column (object colour 3e/3f/61, alpha 2b) is
            // that case; it was drawn black.
            if(material[4]==0)factor=objectColour;
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
                if(material[4]==3){if(power<=0)throw std::runtime_error("unported native point/spot light with specular power 0");weights=value<std::array<float,4>>(rt,block+0x50);}
                else if(material[4]!=0 && material[4]!=1)throw std::runtime_error(std::format("unported native point/spot lights in mode {}",material[4]));
                if(pointLights && material[4]==3){
                    // Mode 3's table B (code image 02a71b3e, routine 0x3350, pipelined one vertex
                    // behind): per light d = L - p scaled by S / |d|^2 (DIV, then MULQ); diffuse
                    // max(0, n . dq) * header 11.y (unpowered); specular max(0, n . h) squared
                    // header 9.w times, h = (dq + (eye - p) / |eye - p|) * header 11.z. The factor
                    // gains C0 D0 + C1 D1 + C2 D2 + C0 S0 + C1 S1 + C2 S2, summed in that order.
                    const float el=std::sqrt(dot(e,e)),q=el!=0?1/el:0;float diffuse[3],specular[3];
                    for(unsigned i=0;i<3;++i){const auto &l=points[i];std::array<float,3> d{l.position[0]-p[0],l.position[1]-p[1],l.position[2]-p[2]};
                        const float squared=dot(d,d),s=squared!=0?l.strength/squared:0;std::array<float,3> dq{d[0]*s,d[1]*s,d[2]*s},h{};
                        diffuse[i]=std::max(0.0f,dot(n,dq))*weights[1];
                        for(unsigned c=0;c<3;++c)h[c]=(dq[c]+e[c]*q)*weights[2];
                        specular[i]=std::max(0.0f,dot(h,n));for(int32_t r=0;r<power;++r)specular[i]*=specular[i];}
                    for(unsigned c=0;c<3;++c)factor[c]+=points[0].colour[c]*diffuse[0]+points[1].colour[c]*diffuse[1]+points[2].colour[c]*diffuse[2]+
                        points[0].colour[c]*specular[0]+points[1].colour[c]*specular[1]+points[2].colour[c]*specular[2];}
                for(unsigned i=0;pointLights && material[4]!=3 && i<3;++i){const auto &l=points[i];std::array<float,3> d{l.position[0]-p[0],l.position[1]-p[1],l.position[2]-p[2]};
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
            vout.rgba=0;for(unsigned c=0;c<4;++c){finite(factor[c]);const auto lit=uint32_t(std::clamp((c==3?alphaRow:float(v.colour[c]))*factor[c],0.0f,limit[c]>0?std::min(limit[c],255.0f):255.0f));vout.rgba|=lit<<(8*c);}
            for(auto f:{vout.x,vout.y,vout.z,vout.u,vout.v,vout.fog})finite(f);
        }
        if(shadowClass){
            std::vector<std::array<float,3>> positions(vertices.size());for(size_t k=0;k<vertices.size();++k)positions[k]={vertices[k].x,vertices[k].y,vertices[k].z};
            nativeShadowPrisms(positions,m.strips,shadowLight,matrix,shadowFlat,out.shadowIncrement,out.shadowDecrement);continue;}
        const bool cull=(block?word(block+0x90)==0:bool(material[5])) && !byte(object+0x210);size_t first=0;unsigned guarded=0,culled=0;
        if(material[6]&6){
            // The GS's LINE STRIP (each vertex continues the line) or LINE (pairs), one pixel wide.
            mat.lines=true;const bool strip=material[6]&2;
            for(auto length:m.strips){for(unsigned k=1;k<length;k+=strip?1:2){const auto a=first+k-1,b=first+k;
                if(outside[a] || outside[b]){++guarded;continue;}batch.vertices.push_back(vertices[a]);batch.vertices.push_back(vertices[b]);}first+=length;}
        }else
        for(auto length:m.strips){for(unsigned k=2;k<length;++k){const auto a=first+k-2,b=first+k-1,c=first+k;
            if(outside[a] || outside[b] || outside[c]){++guarded;continue;}
            if(cull && projected[a][3]>0 && projected[b][3]>0 && projected[c][3]>0){const auto &p=projected[a],&q=projected[b],&r=projected[c];
                const float area=(p[0]/p[3]-q[0]/q[3])*(q[1]/q[3]-r[1]/r[3])-(p[1]/p[3]-q[1]/q[3])*(q[0]/q[3]-r[0]/r[3]);
                if(!std::signbit(area*((k&1)?-1.0f:1.0f))){++culled;continue;}}
            batch.vertices.push_back(vertices[a]);batch.vertices.push_back(vertices[(k&1)?c:b]);batch.vertices.push_back(vertices[(k&1)?b:c]);}first+=length;}
        if(traceModels && batch.vertices.empty())
            std::fprintf(stderr,"[vp2s:model-empty] object=%x mesh=%u/%x vertices=%zu guarded=%u culled=%u textured=%u blend=%u/%u%u%u%u class=%x\n",
                object,i,mesh,m.vertices.size(),guarded,culled,material[1],material[0],material[7],material[8],material[9],material[10],word(object));
        if(traceModels && !batch.vertices.empty()){
            double tex[3]{},col[4]{};float x0=1e9,y0=1e9,x1=-1e9,y1=-1e9;
            if(batch.texture){for(auto v:batch.texture->rgba)for(unsigned c=0;c<3;++c)tex[c]+=(v>>(8*c))&255;for(auto &t:tex)t/=double(batch.texture->rgba.size());}
            for(const auto &v:batch.vertices){for(unsigned c=0;c<4;++c)col[c]+=(v.rgba>>(8*c))&255;
                const auto &M=batch.matrix;const float w=M[3]*v.x+M[7]*v.y+M[11]*v.z+M[15];if(w<=0)continue;
                const float sx=(M[0]*v.x+M[4]*v.y+M[8]*v.z+M[12])/w,sy=(M[1]*v.x+M[5]*v.y+M[9]*v.z+M[13])/w;
                x0=std::min(x0,sx);y0=std::min(y0,sy);x1=std::max(x1,sx);y1=std::max(y1,sy);}
            for(auto &c:col)c/=double(batch.vertices.size());
            uint32_t tag=0,index=0,slot=0,part=0;
            if(material[1] && block){slot=uint32_t(value<int32_t>(rt,block+0x98));const auto ix=value<int16_t>(rt,manager+0x2670+2*slot);tag=word(manager+0x1008+16*ix);index=word(manager+8+16*ix);part=word(block+0x9c);}
            std::fprintf(stderr,"[vp2s:model] object=%x mesh=%u/%x block=%x mode=%u tfx=%u tex=%08x:%x part=%u texmean=%.0f,%.0f,%.0f rgba=%.0f,%.0f,%.0f,%.0f box=%.0f,%.0f,%.0f,%.0f tris=%zu algs=",
                object,i,mesh,block,material[4],material[3],tag,index,part,tex[0],tex[1],tex[2],col[0],col[1],col[2],col[3],x0,y0,x1,y1,batch.vertices.size()/3);
            for(auto a:m.algorithms)std::fprintf(stderr,"%d.",a);
            std::fprintf(stderr," obj=%.2f,%.2f,%.2f amb=%.2f,%.2f,%.2f class=%x o66=%u m3f=%u m57=%u blend=%u/%u%u%u%u\n",objectColour[0],objectColour[1],objectColour[2],ambient[0],ambient[1],ambient[2],
                word(object),byte(object+0x66),byte(model+0x3f),byte(model+0x57),material[0],material[7],material[8],material[9],material[10]);}
        if(!batch.vertices.empty())out.batches.push_back(std::move(batch));
    }
    if(traceModels && shadowClass){
        std::fprintf(stderr,"[vp2s:shadow] object=%x light=%.4f,%.4f,%.4f faces=%zu/%zu\n",object,shadowLight[0],shadowLight[1],shadowLight[2],out.shadowIncrement.size()/3,out.shadowDecrement.size()/3);
        for(unsigned c=0;c<4;++c)std::fprintf(stderr,"[vp2s:shadow]  top col%u %.3f %.3f %.3f %.3f flat col%u %.3f %.3f %.3f %.3f\n",c,matrix[4*c],matrix[4*c+1],matrix[4*c+2],matrix[4*c+3],
            c,shadowFlat[4*c],shadowFlat[4*c+1],shadowFlat[4*c+2],shadowFlat[4*c+3]);
        for(size_t k=0;k<std::min<size_t>(out.shadowIncrement.size(),6);++k)
            std::fprintf(stderr,"[vp2s:shadow]  inc %.2f %.2f %.1f\n",out.shadowIncrement[k].x,out.shadowIncrement[k].y,out.shadowIncrement[k].z);
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
    auto *feedback=dynamic_cast<FeedbackOwner *>(&d->host());
    try{submission=owner->nativeModels().capture(rt,assets->nativeAssets(),textures->nativeTextures(),scene->nativeScene(),object,feedback?&feedback->nativeFeedbackCopies():nullptr);}
    catch(const std::runtime_error &e){throw std::runtime_error(std::format("native model object={:08x}: {}",object,e.what()));}
    owner->drawNativeModel(submission);c->r[4].sd[0]=int32_t(object);
    return true; // Original compiled scene completion follows accepted native draws.
}
}

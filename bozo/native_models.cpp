// Native mesh assets, game object records and scalar rendering mathematics.
// No runtime packet decoding, transport replay or vector-machine state.
#include "native_cast.h"
#include "native_models.h"
#include "native_math.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/ops_fpu.h"
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
    int32_t secondScroll=-1;   // routine 50's offset row index (table B: the second pass's UV scroll)
    std::array<std::array<uint32_t,8>,2> shell{};   // routines 12 / 14 (table B shells): data rows 0, 1
    std::vector<std::array<int32_t,2>> listedUv;   // routine 18: the second pass's UVs, 12-bit fixed point
    // Routine 26's data row: alpha base (.x), alpha on (.y), alpha scale (.w).
    struct Reflection {float base=0;int32_t on=0;float scale=0;} reflection,secondReflection;   // 26, 28 (table B)
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
void finite(float f,std::source_location at=std::source_location::current()){
    if(!std::isfinite(f))throw std::runtime_error(std::format("nonfinite native model input/result (native_models.cpp:{})",at.line()));}
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
    if(r.take<uint32_t>()!=0x334e5056)throw std::runtime_error("semantic native mesh version mismatch (VPN3: rebuild the mesh packs)");
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
        if(m->has(50))m->secondScroll=r.take<int32_t>();
        for(unsigned s=0;s<2;++s)if(m->has(s?14:12))m->shell[s]=r.take<std::array<uint32_t,8>>();
        if(m->has(18))r.array(m->listedUv,nv);
        if(m->has(28))m->secondReflection=r.take<Mesh::Reflection>();
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
// Where each vertex's UV pair sits in a mesh packet: its VIF codes run as tools/vp2_meshes.py runs them (STCYCL
// addressing, STMASK / STROW, every unpack format), tracking the byte that fills each 16-bit lane. Vertex k's UV is VU
// row (vertices + 4k + 1), lanes x and y, vertices = row 0.y. Each pair must come from one V2-16 unpack (u then v).
std::vector<NativeMeshUv> nativeMeshUvSources(const uint8_t *bytes,uint32_t size,size_t count){
    struct Lane{int64_t value=0;uint32_t source=UINT32_MAX;bool unsignedLane=false;};
    std::vector<std::array<Lane,4>> mem(1024);
    unsigned cl=1,wl=1;uint32_t mask=0;std::array<uint32_t,4> row{};
    const auto u32=[&](uint32_t at){if(at+4>size)throw std::runtime_error("native mesh VIF code outside its packet");uint32_t v;std::memcpy(&v,bytes+at,4);return v;};
    for(uint32_t p=0;p+4<=size;){
        const uint32_t w=u32(p),cmd=(w>>24)&0x7f,num=(w>>16)&0xff,imm=w&0xffff;p+=4;
        if((cmd&0x60)==0x60){
            const unsigned vn=(cmd>>2)&3,vl=cmd&3,n=num?num:256,comps=vn+1;const bool uns=(imm>>14)&1,masked=(cmd>>4)&1;const unsigned addr=imm&0x3ff;
            const uint32_t nbytes=vl==3?2*n:comps*(32>>vl)/8*n;if(p+nbytes>size)throw std::runtime_error("native mesh unpack outside its packet");
            for(unsigned k=0;k<n;++k){
                const unsigned qw=(addr+(k/wl)*cl+k%wl)%1024;std::array<Lane,4> values{};
                if(vl==3){uint16_t v;std::memcpy(&v,bytes+p+2*k,2);values[0].value=(v&0x1f)<<3;values[1].value=((v>>5)&0x1f)<<3;values[2].value=((v>>10)&0x1f)<<3;values[3].value=((v>>15)&1)<<7;}
                else{
                    const unsigned bits=32>>vl;
                    for(unsigned c=0;c<comps;++c){const uint32_t at=p+(k*comps+c)*bits/8;int64_t raw=0;
                        if(bits==32)raw=u32(at);else if(bits==16){uint16_t v;std::memcpy(&v,bytes+at,2);raw=v;}else raw=bytes[at];
                        if(bits<32 && !uns && (raw>>(bits-1)))raw-=int64_t(1)<<bits;
                        values[c].value=raw;values[c].source=bits==16?at:UINT32_MAX;values[c].unsignedLane=uns;}
                    if(comps==1)for(unsigned c=1;c<4;++c)values[c]=values[0];
                    else for(unsigned c=comps;c<4;++c)values[c]=mem[qw][c];
                }
                for(unsigned c=0;c<4;++c){
                    if(masked){const unsigned m=(mask>>(2*c+8*std::min(k%wl,3u)))&3;
                        if(m==1){mem[qw][c]=Lane{int64_t(row[c]),UINT32_MAX,false};continue;}
                        if(m==2 || m==3)continue;}
                    mem[qw][c]=values[c];}
            }
            p+=(nbytes+3)/4*4;continue;
        }
        if(cmd==0x01){cl=imm&0xff;wl=(imm>>8)&0xff;if(!wl)wl=1;}
        else if(cmd==0x20){mask=u32(p);p+=4;}
        else if(cmd==0x30){for(unsigned c=0;c<4;++c)row[c]=u32(p+4*c);p+=16;}
        else if(cmd==0x31)p+=16;
        else if(cmd!=0x00 && cmd!=0x02 && cmd!=0x03 && cmd!=0x04 && cmd!=0x05 && cmd!=0x06 && cmd!=0x07 && cmd!=0x10 && cmd!=0x11 && cmd!=0x13)
            throw std::runtime_error(std::format("native mesh VIF code {:02x} unknown",cmd));
    }
    const auto vertices=int32_t(mem[0][1].value);std::vector<NativeMeshUv> out(count);
    for(size_t k=0;k<count;++k){const auto &lanes=mem[(uint32_t(vertices)+4*k+1)%1024];
        if(lanes[0].source==UINT32_MAX || lanes[1].source!=lanes[0].source+2 || lanes[0].unsignedLane!=lanes[1].unsignedLane)
            throw std::runtime_error(std::format("native mesh vertex {} UV not from one V2-16 unpack",k));
        out[k]={lanes[0].source,lanes[0].unsignedLane};}
    return out;
}
// The scene's pending object owns a model it is, or one its composite draw
// (class 1599a0, method 2b7ab0) reaches: for slot 2..0, each list child's slot
// +8c0+4i, then its own; then each list child (list +8d4, next +8, ending at
// the owner's +8cc). Children of that class compose the same way; their draws
// run inside the owner's, and their completions (21c9a0) leave the scene slot.
// A level-of-detail model (any class whose +60 is 246f50: 154dc0 and its levels 154ca0) owns the levels in its slots
// +87c, +880, +884: 246f50 creates them (class 154ca0, 247234..247268) and, while the owner is the scene's pending
// object, runs each level's +64 and then its draw +11c (247318..247378) before its own.
bool composedBy(Runtime *rt,uint32_t owner,uint32_t object){
    constexpr uint32_t kComposite=0x1599a0;
    std::vector<uint32_t> stack{owner},seen;
    while(!stack.empty()){
        const auto node=stack.back();stack.pop_back();
        if(node==object)return true;
        if(node<0x100000 || node>0x2000000-0x8d8 || std::find(seen.begin(),seen.end(),node)!=seen.end())continue;
        seen.push_back(node);if(seen.size()>512)throw std::runtime_error("native composite model tree outside bounds");
        const auto type=value<uint32_t>(rt,node);
        if(type>=0x100000 && type<0x2000000-0x64 && value<uint32_t>(rt,type+0x60)==0x246f50){
            for(unsigned i=0;i<3;++i)if(const auto level=value<uint32_t>(rt,node+0x87c+4*i))stack.push_back(level);
            continue;}
        if(type!=kComposite)continue;
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
                        std::vector<bozo::render::Vertex> &increment,std::vector<bozo::render::Vertex> &decrement,bool point){
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
            // A point light (program 0x6f0 with mem[14].w: vf01 = vf21 - mem[37]): the direction from the light to the
            // triangle; any of its vertices gives the same sign.
            const P d=point?sub(a,light):light;
            if(d[0]*n[0]+d[1]*n[1]+d[2]*n[2]<0)continue;
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
ProjectedTexture projectedTexture(const std::array<float,3> &p,const std::array<float,3> &n,const std::array<float,4> &origin,
                                  const std::array<float,4> (&cols)[4],uint32_t mask){
    std::array<float,3> q{};for(unsigned c=0;c<3;++c)q[c]=((cols[3][c]*1.0f+cols[0][c]*p[0])+cols[1][c]*p[1])+cols[2][c]*p[2];
    std::array<float,3> t{};for(unsigned c=0;c<3;++c)t[c]=(origin[c]-p[c])*n[c];
    const float f=(t[2]+t[1])+t[0];const uint32_t mac=(f==0?0x02u:0u)|(std::signbit(f) && f!=0?0x20u:0u);   // z lane: Z, S
    const auto fixed=[](float x){return int32_t(std::clamp(std::trunc(double(x)*4096),-2147483648.0,2147483647.0));};   // FTOI12
    return {{fixed(q[0]),fixed(q[1]),fixed(q[2])},(std::signbit(q[2]) && q[2]!=0) || (mac&mask)!=0};
}
std::vector<std::vector<NativeShellVertex>> nativeShells(int routine,const std::array<uint32_t,8> &data,std::vector<NativeShellVertex> v){
    const float push=std::bit_cast<float>(data[0]),jitter=std::bit_cast<float>(data[2]);std::array<float,4> add{};for(unsigned c=0;c<4;++c)add[c]=std::bit_cast<float>(data[4+c]);
    int32_t counter=int32_t(data[1]);uint32_t seed=data[3],R=0x3f800000u;
    const auto rnext=[](uint32_t r){return (((r<<1)^((r>>4)&1u)^((r>>22)&1u))&0x007fffffu)|0x3f800000u;};
    const auto step=[&](bool jittered){
        for(auto &x:v){for(unsigned c=0;c<4;++c){const float f=std::min(std::max(float(int32_t(x.colour[c]))+add[c],0.0f),255.0f);x.colour[c]=uint32_t(int32_t(f));}   // FTOI0
            for(unsigned c=0;c<3;++c)x.p[c]=x.p[c]*1.0f+x.n[c]*push;}
        if(!jittered)return;
        std::array<float,2> pair{};const auto draw=[&]{R=rnext(R);pair[0]=std::bit_cast<float>(R);R=rnext(R);pair[1]=std::bit_cast<float>(R);};
        draw();for(auto &x:v){for(unsigned c=0;c<2;++c){const float j=(pair[c]-1.5f)*jitter;const double f=(double(int32_t(std::trunc(double(x.uv[c])*4096)))/4096+j)*4096;
                x.uv[c]=float(std::clamp(std::trunc(f),-2147483648.0,2147483647.0))/4096;}
            draw();}};   // the pair after the last vertex is drawn too (the loop draws one ahead)
    std::vector<std::vector<NativeShellVertex>> out;
    for(unsigned dispatch=0;dispatch<64 && counter>0;++dispatch){
        if(routine==12){step(false);--counter;
            if(counter>0){--counter;if(seed){R=0x3f800000u|(seed&0x007fffffu);seed=0;}step(true);}}
        else{--counter;if(seed){R=0x3f800000u|(seed&0x007fffffu);seed=0;}step(true);}
        out.push_back(v);}
    return out;
}
void nativeEmbossStrips(std::vector<std::array<int32_t,2>> &uv,const std::vector<std::array<float,3>> &p,const std::vector<uint16_t> &strips,
                        const std::array<float,3> &light,bool point,float scale){
    if(uv.size()!=p.size())throw std::runtime_error("native UV shift: positions and UVs differ in count");
    uint32_t fl=0;
    const auto W=[](float x){return std::bit_cast<uint32_t>(x);};
    const auto M=[&](uint32_t a,uint32_t b){return bozo::ps2f::mul(a,b,fl);};
    const auto A=[&](uint32_t a,uint32_t b){return bozo::ps2f::addNearest(a,b,fl);};
    const auto S=[&](uint32_t a,uint32_t b){return bozo::ps2f::subNearest(a,b,fl);};
    const auto one=W(1.f);const auto uvw=[&](int32_t x){return W(float(x)/4096.f);};   // ITOF12
    const auto length2=[&](const std::array<uint32_t,3> &x){return A(A(M(x[0],x[0]),M(x[1],x[1])),M(x[2],x[2]));};
    size_t first=0;
    for(const auto length:strips){
        if(length<3 || first+length>uv.size())throw std::runtime_error("native UV shift strip outside its mesh");
        std::vector<std::array<uint32_t,2>> offset(length-2);
        for(size_t t=0;t+2<length;++t){
            const auto a=first+t,b=a+1,c=a+2;
            const uint32_t du1=S(uvw(uv[b][0]),uvw(uv[a][0])),dv1=S(uvw(uv[b][1]),uvw(uv[a][1])),du2=S(uvw(uv[c][0]),uvw(uv[a][0])),dv2=S(uvw(uv[c][1]),uvw(uv[a][1]));
            std::array<uint32_t,3> T{},B{};
            for(unsigned l=0;l<3;++l){
                const uint32_t p1=S(W(p[b][l]),W(p[a][l])),p2=S(W(p[c][l]),W(p[a][l]));
                const uint32_t x=S(M(du1,dv2),M(dv1,du2)),y=S(M(dv1,p2),M(p1,dv2)),z=S(M(p1,du2),M(du1,p2));
                const uint32_t q=bozo::ps2f::div(one,x,fl);T[l]=M(y,q);B[l]=M(z,q);}
            const uint32_t tl=bozo::ps2f::rsqrt(one,length2(T),fl),bl=bozo::ps2f::rsqrt(one,length2(B),fl);
            std::array<uint32_t,3> L{W(light[0]),W(light[1]),W(light[2])};
            if(point){std::array<uint32_t,3> d{};for(unsigned l=0;l<3;++l)d[l]=S(W(p[c][l]),L[l]);
                const uint32_t r=bozo::ps2f::rsqrt(one,length2(d),fl);for(unsigned l=0;l<3;++l)L[l]=M(d[l],r);}
            for(auto &x:L)x=M(x,W(scale));
            offset[t]={A(A(M(M(T[0],tl),L[0]),M(M(T[1],tl),L[1])),M(M(T[2],tl),L[2])),A(A(M(M(B[0],bl),L[0]),M(M(B[1],bl),L[1])),M(M(B[2],bl),L[2]))};}
        for(size_t k=0;k<length;++k){const auto &o=offset[std::min<size_t>(k,length-3)];
            for(unsigned c=0;c<2;++c){const double f=double(std::bit_cast<float>(S(uvw(uv[first+k][c]),o[c])))*4096;   // FTOI12
                uv[first+k][c]=int32_t(std::clamp(std::trunc(f),-2147483648.0,2147483647.0));}}
        first+=length;}
}
std::optional<NativeSecondPass> nativeSecondPass(const NativeMaterialRows &rows){
    const auto row=[&](unsigned r,unsigned c){return rows[r-7][c];};
    const auto prim=[&](unsigned r){return (row(r,1)>>15)&0x7ff;};
    constexpr uint32_t primBits=0x7ffu<<15;
    for(unsigned r:{17u,18u})if((row(r,2)&0xff)!=6)return std::nullopt;   // TEX0_1 writes, as rows 9 / 10
    if(row(21,2)>=256 || row(21,3)>15)return std::nullopt;
    // The same tags, PRIM with ABE set (pass 1 may already blend: maps 65, 124); or (map 109) exactly pass 1's tags with ABE
    // clear on both: an opaque redraw of the same triangles with pass 2's texture.
    const bool opaque=prim(19)==prim(13) && prim(20)==prim(14) && !(prim(13)&0x40) && !(prim(14)&0x40);
    for(unsigned r:{13u,14u})if(row(r+6,0)!=row(r,0) || row(r+6,2)!=row(r,2) || (row(r+6,1)&~primBits)!=(row(r,1)&~primBits) || (!opaque && prim(r+6)!=(prim(r)|0x40)))
        return std::nullopt;
    const auto alpha=row(16,0);
    // ALPHA_1; .w picks row 65 or 66 for the register between (pass 1's row 8.w does the same; both seen). Pass 2 redraws
    // pass 1's triangles at the same depths, so that register's depth write changes nothing drawn. The high word's low byte
    // is FIX; the GS ignores the rest (maps 459-461 keep routine 34's UV-shift scale there, 0.01).
    if((row(16,2)&0xff)!=0x42 || row(16,3)>1 || alpha>0xff)return std::nullopt;
    for(unsigned f=0;f<4;++f)if(((alpha>>(2*f))&3)==3)return std::nullopt;
    return NativeSecondPass{alpha,row(21,2),row(21,3),row(18,0),row(16,1)&0xff,row(21,1)!=0,!opaque};
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
    std::array<float,3> shadowLight{};Mat shadowFlat{};bool shadowPoint=false;
    if(shadowClass){
        // A point or spot light (2b82f0: the light's +2c8 & 3 set): mem[14].w = 1, its world position (w 1) in
        // mem[20] and the flattening from 2148a0 (a projection through that point) in mem[5..8]. The object set-up
        // 0x238 takes mem[20] through O's rigid inverse (table A), set 0xa's set-up (0xf38, 10d0..10e0) copies it
        // to mem[37], and the shadow program culls with vertex - mem[37].
        shadowPoint=word(object+0x31c)!=0;
        if(shadowPoint){const auto p=row(20);if(p[3]!=1)throw std::runtime_error("native point-light shadow position without w 1");shadowLight=rigidInverse(p);}
        else{const auto l=row(37);for(unsigned c=0;c<3;++c)shadowLight[c]=rotation[0][c]*l[0]+rotation[1][c]*l[1]+rotation[2][c]*l[2];}
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
        // A mesh of one of the object's two working copies (+630 and +634, byte copies of the model whose UVs the game
        // animates, used in turn; maps 264, 270, 271, 535): the list names it at one of the model's own mesh offsets. Its
        // source is the disc mesh at that offset; its positions, normals and UVs are read live from the copy (below, as for
        // an editable mesh); every other word of it must still be the disc mesh's.
        uint32_t live=0,lookup=mesh;std::vector<std::array<float,2>> liveUv;
        if(!assets.origin(mesh))for(const uint32_t slot:{0x630u,0x634u}){
            const auto c=word(object+slot),cs=c&0xf0000000u,co=c&0x0fffffffu;
            if(!(cs==0 || cs==0x20000000u || cs==0x30000000u || cs==0x80000000u || cs==0xa0000000u) || (co&15) || co<0x100000 || co>0x2000000-0x68)continue;
            const auto copy=NativeAssets::address(c);
            if(copy==model || mesh<copy || word(copy+0x34)!=count || word(copy+0x60)!=word(model+0x60) || word(copy+0x64)!=word(model+0x64))continue;
            for(uint32_t at=model+0x60,k=0;k<count;++k){if(mesh-copy==at-model){live=mesh;lookup=at;break;}
                const auto size=value<int16_t>(rt,at+4);if(size<=0)break;at+=uint32_t(size);}
            if(live)break;}
        const auto origin=assets.origin(lookup);std::shared_ptr<const Pack> pack;std::shared_ptr<const Mesh> held;
        if(origin){pack=load(*origin);const auto found=pack->find(origin->offset);
            if(found==pack->end())throw std::runtime_error(std::format("native mesh not converted: {:x}/{}+{:x}",origin->disc,origin->part,origin->offset));
            held=found->second;
            if(live){
                // The game animates the copy's UVs (maps 264..535: only V2-16 words differ): read them live.
                const auto size=uint32_t(value<int16_t>(rt,lookup+4));std::vector<bool> moving(size/4+1,false);
                for(const auto &s:held->sources)for(auto at:{s.normal,s.position})if(at!=UINT32_MAX)for(unsigned c=0;c<3;++c)if(at/4+c<moving.size())moving[at/4+c]=true;
                const auto *disc=rt->memory.hostPointer(lookup,size);if(!disc)throw std::runtime_error("native working-copy disc mesh unmapped");
                const auto uvs=nativeMeshUvSources(disc,size,held->vertices.size());liveUv.resize(uvs.size());
                for(size_t k=0;k<uvs.size();++k){for(uint32_t b:{uvs[k].offset,uvs[k].offset+3})if(b/4<moving.size())moving[b/4]=true;
                    for(unsigned c=0;c<2;++c){const auto raw=value<uint16_t>(rt,live+uvs[k].offset+2*c);liveUv[k][c]=float(uvs[k].unsignedLane?int32_t(raw):int32_t(int16_t(raw)))/4096.f;}}
                for(uint32_t at=0;at+4<=size;at+=4)if(!moving[at/4] && word(live+at)!=word(lookup+at))
                    throw std::runtime_error(std::format("native working-copy mesh {:x} differs from its disc mesh {:x} outside its vertices (+{:x}: {:08x} / {:08x})",live,lookup,at,word(live+at),word(lookup+at)));}}
        else{
            // A run-time mesh: only one the sprite batcher published, with every
            // byte as it was published.
            const auto g=generated.find(mesh);if(g==generated.end())throw std::runtime_error(std::format("native mesh missing allocation provenance (mesh {:x} model {:x} words {:08x} {:08x} {:08x} {:08x})",mesh,model,word(mesh),word(mesh+4),word(mesh+0x14),word(mesh+0x18)));
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
        // A skinned mesh (2 / 4) whose bone palette the game never built (map 487: the buffer still holds a file read's
        // bytes): not drawn (user decision 2026-11-25).
        if(palette && (m.has(2) || m.has(4)) && !nativeBonePaletteBuilt(rt,palette))continue;
        // 22: the library's set-up table entry is a bare JR vi10 (both layouts, 0x499 / 0x483, every copy in EE RAM
        // at the battle's frame 72200), so the routine does nothing, as 46 (and as publishGeneratedMesh admits it).
        // 50, 52, 54 likewise: bare JR vi10 in all eight table copies in EE RAM (battle 0x1c and 0x18, the forest), found as
        // the tables whose entry 40 branches to the screen-UV routine (0x4000076f) and whose 22 and 46 return.
        for(auto algorithm:m.algorithms)if(algorithm!=-1 && algorithm!=0 && algorithm!=2 && algorithm!=4 && algorithm!=6 && algorithm!=8 && algorithm!=12 && algorithm!=14 && algorithm!=16 && algorithm!=18 && algorithm!=22 && algorithm!=28 && algorithm!=32 && algorithm!=34 && algorithm!=36 && algorithm!=20 && algorithm!=26 && algorithm!=40 && algorithm!=44 && algorithm!=46 && algorithm!=48 && algorithm!=50 && algorithm!=52 && algorithm!=54)
            throw std::runtime_error(std::format("unported native mesh algorithm {}",algorithm));
        // 16: table A's entry is a bare JR vi10 in every copy (0x483 / 0x4ab / 0x499 / 0x42c / 0x454 / 0x442, map 107's
        // EE RAM); table B's, which the mesh program runs after the mesh's kick (0f28: base A + 0x32), is a second
        // pass (set 0x00's 0x1720): rows 19 and 20 become the GIF tags 13 and 14, the register packet is row 7's tag
        // with rows 16, 65 or 66 (mem[15].y | row 16.w), 17 and 18 (pass 1 sends 8, 65/66, 9, 10), and the vertex loop
        // (0918) draws the mesh again. Every such mesh in the all-maps sweep (12 maps) has pass 2 = pass 1 with PRIM.ABE
        // set and ALPHA_1 from row 16 (0x48, 0x44 or 0x42): drawn natively as a second, blended copy of the batch.
        // Its texture: row 21's slot and part (pass 1's are row 15's, block +98 / +9c); map 124's pass 2 samples another
        // image (manager slot 1, TBP 3a40) than pass 1 (slot 0, TBP 3380), map 107's the same one.
        std::optional<NativeSecondPass> secondPass;
        // Diagnostic (VP2S_BOZO_TRACE_TABLE_B): each mesh whose algorithms have a table-B routine (run after the kick,
        // each gated: 16/18/22/28/50 by row 21.y, 32/52/54 by row 17.w, 12/14 by their data's .y, 46 by mem[36].z), once.
        if(static const bool traceB=std::getenv("VP2S_BOZO_TRACE_TABLE_B")!=nullptr;traceB){
            static std::set<std::pair<uint32_t,uint32_t>> seen;bool any=false;for(auto a:m.algorithms)any|=a==12||a==14||a==18||a==22||a==28||a==32||a==46||a==50||a==52||a==54;
            if(any && seen.insert({mesh,block}).second){
                const bool live=block>=0x100000 && word(block+0x70)==0x8000 && word(block+0x78)==0x412;
                std::fprintf(stderr,"[vp2s:table-b] object=%x class=%x mesh=%x block=%x algs=%d.%d.%d.%d live=%d row21y=%u row17w=%u row36z=%u\n",object,word(object),mesh,block,
                    m.algorithms[0],m.algorithms[1],m.algorithms[2],m.algorithms[3],int(live),live?word(block+0xf4):0u,live?word(block+0xbc):0u,word(object+0x230+16*36+8));}}
        // ... and for 46 (VP2S_BOZO_TRACE_TABLE_B=46): row 21.x, and the projector rows 16, 36, 40..46 of the object.
        if(static const bool trace46=[]{const char *v=std::getenv("VP2S_BOZO_TRACE_TABLE_B");return v && std::string(v)=="46";}();trace46 && m.has(46)){
            static std::set<uint32_t> shown;const bool live=block>=0x100000 && word(block+0x70)==0x8000 && word(block+0x78)==0x412;
            if(shown.insert(object).second){std::string rows;for(unsigned r:{16u,36u,40u,41u,42u,43u,44u,45u,46u})rows+=std::format(" r{}={:08x},{:08x},{:08x},{:08x}",r,word(object+0x230+16*r),word(object+0x234+16*r),word(object+0x238+16*r),word(object+0x23c+16*r));
                if(live)for(unsigned r:{13u,14u,19u,20u})rows+=std::format(" m{}={:08x},{:08x},{:08x},{:08x}",r,word(block+0x10+16*(r-7)),word(block+0x14+16*(r-7)),word(block+0x18+16*(r-7)),word(block+0x1c+16*(r-7)));
                std::fprintf(stderr,"[vp2s:table-b46] object=%x class=%x mesh=%x row21x=%u live=%d%s\n",object,word(object),mesh,live?word(block+0xf0):0u,int(live),rows.c_str());}}
        // 22 (table B 0f90: routine 20's sphere map, bal 0fd0, then 1720), 50 (2348: the UVs plus object row 38 + data.x
        // .zw, bal 2398, then 1720), 18 (1870: its data's per-vertex UV pairs copied in, then 1748) and 28 (1138: routine 26's
        // reflection map with 28's data row, bal 1178, then 1720) are the same pass with other UVs. Each tests and clears
        // row 21.y, so the first of them in the mesh's routine order draws it.
        // 32, 34, 52, 54 (table B 1bb8, 1ba8, 1e70, 1e60 in set 0; the same code in every set that has them; each entry's
        // delay slot loads the gate, row 17.w). 52 / 54 are not ported (no live one with the gate set). 32 and 34 (below) are
        // the UV-shift redraw: 32's register packet is pass 1's rows 8 / 9 / 10 (1b00), 34's rows 16 / 17 / 18 (1a90), each
        // with ALPHA 0x48; table A's 32 and 34 are bare JR vi10. Field maps 459-461 draw meshes with them and the gate set
        // (row 16.y 0.01: the shift's scale).
        if((m.has(52) || m.has(54)) && block>=0x100000 && word(block+0x70)==0x8000 && word(block+0x78)==0x412 && word(block+0xbc))
            {std::string shown;for(unsigned r=7;r<=21;++r)shown+=std::format(" r{}={:08x},{:08x},{:08x},{:08x}",r,word(block+0x10+16*(r-7)),word(block+0x14+16*(r-7)),word(block+0x18+16*(r-7)),word(block+0x1c+16*(r-7)));
             throw std::runtime_error(std::format("unported native table-B pass (routine 32/52/54): mesh {:x} block {:x} routines {}.{}.{}.{} r40w {:x}{}",mesh,block,
                m.algorithms[0],m.algorithms[1],m.algorithms[2],m.algorithms[3],word(object+0x230+16*40+12),shown));}
        const int embossRoutine=block>=0x100000 && word(block+0x70)==0x8000 && word(block+0x78)==0x412 && word(block+0xbc)?(m.has(34)?34:m.has(32)?32:0):0;
        if(embossRoutine && m.has(32) && m.has(34))throw std::runtime_error("native UV-shift routines 32 and 34 together");
        int passRoutine=0;for(auto a:m.algorithms)if(a==16 || a==18 || a==22 || a==28 || a==50){passRoutine=a;break;}
        std::array<float,2> passScroll{};
        if(passRoutine==50){
            if(m.has(20) || m.has(26) || m.has(36) || m.has(40))throw std::runtime_error("native second pass (routine 50) over UVs that are not the mesh's own");
            if(m.secondScroll<0 || m.secondScroll>=16)throw std::runtime_error("native second-pass UV scroll index outside bounds");
            const auto q=row(38+m.secondScroll);passScroll={q[2],q[3]};}
        if(passRoutine){
            if(block<0x100000 || word(block+0x70)!=0x8000 || word(block+0x78)!=0x412)throw std::runtime_error("native mesh second pass without its live material rows");
            NativeMaterialRows rows;for(unsigned r=0;r<16;++r)for(unsigned c=0;c<4;++c)rows[r][c]=word(block+0x10+16*r+4*c);
            secondPass=nativeSecondPass(rows);
            if(!secondPass){std::string shown;for(unsigned r:{13u,14u,16u,17u,18u,19u,20u,21u})shown+=std::format(" r{}={:08x},{:08x},{:08x},{:08x}",r,rows[r-7][0],rows[r-7][1],rows[r-7][2],rows[r-7][3]);
                throw std::runtime_error(std::format("unported native mesh second pass (routine {}): mesh {:x} block {:x}{}",passRoutine,mesh,block,shown));}
            if(!secondPass->on)secondPass.reset();}   // row 21.y clear: the routine skips its pass
        // 32 with its gate set: pass 1 (its own tags 13 / 14 and rows 8 / 9 / 10, e.g. ALPHA 0x42: Cd - Cs As) drawn again with
        // the shifted UVs and ALPHA 0x48; no pass routine (16, 18, 22, 28, 50) may run before it (they rewrite the tags).
        // 34 with its gate (row 17.w) set: after the 16 pass (which left rows 19 / 20, PRIM with ABE, in the tags 13 / 14 and
        // ALPHA from row 16, e.g. 0x42: Cd - Cs As) the mesh is drawn again (1808 -> the vertex loop 0918) with the same
        // texture (rows 17 / 18), the UVs shifted along the light in each triangle's tangent frame (nativeEmbossStrips) and
        // ALPHA 0x48 (Cs As + Cd): emboss bump mapping. Light: mem[37] as the object set-up 0x238 leaves it: row 37 through
        // O's rigid inverse with table A (mem[14].w) and mem[40].w, else turned by Rn^T; mem[40].w (vi08) makes the routine
        // take it as a point (maps 459-461: a point light). Scale: row 16.y. The UVs it shifts are the vertices' own (VU row
        // +1): no UV routine may have rewritten them.
        std::vector<std::array<float,2>> embossUv;int embossSlot=-1;
        if(embossRoutine){
            const auto at16=std::find(m.algorithms.begin(),m.algorithms.end(),16),at34=std::find(m.algorithms.begin(),m.algorithms.end(),embossRoutine);
            const bool point=word(object+0x230+16*40+12)!=0;
            const bool order=embossRoutine==34?passRoutine==16 && secondPass && at16<at34:!passRoutine && ((word(block+0x74)>>21)&1);   // 32: pass 1 already blends (row 13 PRIM ABE)
            if(!order || m.has(20) || m.has(26) || m.has(36) || m.has(40) || m.has(48) || m.has(46) || m.has(12) || m.has(14) || (material[6]&6))
                throw std::runtime_error(std::format("unqualified native UV-shift pass (routine {}): mesh {:x} routines {}.{}.{}.{} second pass {} point {} tag {:08x}",embossRoutine,mesh,
                    m.algorithms[0],m.algorithms[1],m.algorithms[2],m.algorithms[3],bool(secondPass),point,word(block+0x74)));
            const auto l=row(37);std::array<float,3> light{};
            if(tableA && point)light=rigidInverse(l);
            else for(unsigned c=0;c<3;++c)light[c]=rotation[0][c]*l[0]+rotation[1][c]*l[1]+rotation[2][c]*l[2];
            std::vector<std::array<int32_t,2>> uv(m.vertices.size());std::vector<std::array<float,3>> p(m.vertices.size());
            for(size_t k=0;k<m.vertices.size();++k){const auto &v=m.vertices[k];for(unsigned c=0;c<3;++c)p[k][c]=v.p[c];
                for(unsigned c=0;c<2;++c)uv[k][c]=int32_t(std::lround(double(liveUv.empty()?v.uv[c]:liveUv[k][c])*4096));}
            nativeEmbossStrips(uv,p,m.strips,light,point,real(block+0xa4));
            embossUv.resize(uv.size());for(size_t k=0;k<uv.size();++k){embossUv[k]={float(uv[k][0])/4096,float(uv[k][1])/4096};for(auto f:embossUv[k])finite(f);}
            embossSlot=int(at34-m.algorithms.begin());}
        // 46 (table B 21e0): a texture projected onto the mesh (a darkening pattern in world space: ALPHA 0x46, Cd (1 - As)).
        // Gated by mem[36].z and row 21.x (which it clears; the 16/22/50 pass tests .y, so both may draw). Per vertex,
        // in the object's VU rows: q = ((r46 + r43 x) + r44 y) + r45 z; S, T, Q = FTOI12(q.xyz) to the UV row; the colour
        // row's .w = 0 when q.z < 0 (vf14's S z) or when the MAC flags of f = ((t.z + t.y) + t.x), t = (r42 - p) * n, meet
        // the mask r40.z, else FTOI0(r36.w). Rows 16.x (ALPHA_1), 19.y (= r16.y, the tag's PRIM) and 17/18.xy (= r40/r41,
        // TEX0) are set, then 16's tail draws (1748). Lifted with the fork's vu1_lift (timed): the flags are this vertex's.
        struct Projector{uint32_t alpha=0,prim=0,mask=0;float level=0;std::array<float,4> origin{},cols[4]{};std::shared_ptr<const bozo::render::Texture> texture;bool textureAlpha=false;uint32_t textureFunction=0;};
        std::optional<Projector> projector;int projectorSlot=-1,passSlot=-1;
        for(unsigned a=0;a<4;++a){if(m.algorithms[a]==46 && projectorSlot<0)projectorSlot=int(a);if(passRoutine && m.algorithms[a]==passRoutine && passSlot<0)passSlot=int(a);}
        const auto rw=[&](unsigned r,unsigned c){return word(object+0x230+16*r+4*c);};
        if(m.has(46) && rw(36,2)){
            if(block<0x100000 || word(block+0x70)!=0x8000 || word(block+0x78)!=0x412)throw std::runtime_error("native projected texture (routine 46) without its live material rows");
            if(word(block+0xf0)){
                Projector pr;pr.alpha=rw(16,0);pr.prim=(rw(16,1)>>15)&0x7ff;pr.mask=rw(40,2);
                constexpr uint32_t primBits=0x7ffu<<15;const auto row13y=word(block+0x74);
                bool ok=pr.alpha<=0xff && (pr.prim&7)==((row13y>>15)&7) && (pr.prim&16) && (pr.prim&64) && (rw(16,1)&~primBits)==(row13y&~primBits) && !(pr.mask&~0x22u);
                for(unsigned f=0;f<4;++f)ok=ok && ((pr.alpha>>(2*f))&3)!=3;
                if(!ok)throw std::runtime_error(std::format("unported native projected texture (routine 46): ALPHA {:x} tag {:08x} mask {:x}",pr.alpha,rw(16,1),pr.mask));
                const float level=row(36)[3];pr.level=float(int32_t(std::clamp(std::trunc(double(level)),-2147483648.0,2147483647.0)));   // FTOI0
                pr.origin=row(42);for(unsigned c=0;c<4;++c)pr.cols[c]=row(43+c);
                // The texture: the manager entry whose TEX0 is the one r41 writes (r40 is the CLUT load before it).
                const auto lo=rw(41,0),hi=rw(41,1);int found=-1;
                if(manager)for(int e=0;e<256;++e)if((word(manager+0x1008+16*e) || word(manager+8+16*e)) && word(manager+0x1010+16*e)==lo){found=e;break;}
                if(found<0)throw std::runtime_error(std::format("native projected texture: no texture-manager entry holds TEX0 {:08x}",lo));
                const TextureKey key{word(manager+0x1008+16*found),word(manager+8+16*found)};
                if(key.tag==0x4674404c && key.index==1)throw std::runtime_error("native projected texture over the feedback picture");
                const uint64_t tex0=lo|uint64_t(word(manager+0x1014+16*found))<<32;
                pr.texture=textures.modelImage(key,tex0,std::nullopt)->picture((hi>>24)&0xf);   // CSA
                pr.textureAlpha=(hi>>2)&1;pr.textureFunction=(hi>>3)&3;   // TCC, TFX
                projector=pr;}}
        // 12 and 14 (table B 1488 and 1590, lifted with vu1_lift): shell redraws. Each step, over the vertices in order:
        // colour = FTOI0(min(max(ITOF0(colour) + data row 1, 0), 255)); position = position + normal * data0.x; and in 14's part
        // UV = FTOI12(ITOF12(uv) + (R - 1.5) * data0.z), R two RNEXTs a vertex (2 (n + 1) a step: the pipeline draws one pair
        // ahead), RINIT from data0.w when set (then cleared). The counter data0.y falls by one a step. 14: if counter, one
        // step. 12: if counter, a step without the UV part, then (its code runs into 14's) a 14 step if the counter is still
        // set. Each dispatch that stepped sets vi12, and 0f58 redraws the mesh from 08b8 (past table A) with its own material.
        const int shellRoutine=m.has(12)?12:m.has(14)?14:0;std::array<uint32_t,8> shellData{};
        if(shellRoutine){shellData=m.shell[shellRoutine==12?0:1];
            if(m.has(12) && m.has(14))throw std::runtime_error("native shell routines 12 and 14 together");
            if(int32_t(shellData[1])>0 && (passRoutine || projector || m.has(20) || m.has(26) || m.has(36) || m.has(40) || (material[6]&6)))
                throw std::runtime_error(std::format("unqualified native shell redraw (routine {}) combination: pass {} projector {} routines {}.{}.{}.{} lines {}",shellRoutine,passRoutine,bool(projector),m.algorithms[0],m.algorithms[1],m.algorithms[2],m.algorithms[3],material[6]&6));
            if(int32_t(shellData[1])>64)throw std::runtime_error("native shell count outside bounds");
            if(std::bit_cast<float>(shellData[2])!=0 && !shellData[3])throw std::runtime_error("native shell jitter without its seed (the VU's R register before it)");}
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
            else if(const auto *rendered=scene.targetTexture(word(manager+0x1010+16*index)&0x3fff);rendered &&
                    !textures.hasModelImage(key,word(manager+0x1010+16*index)|uint64_t(word(manager+0x1014+16*index))<<32,
                        origin && origin->kind==AssetOrigin::Kind::Part?std::optional(std::pair{origin->disc,origin->part}):std::nullopt)){
                // A manager entry with no loaded copy at an offscreen target's block (153880, e.g. 'sPlM' 1 at 3ae0; a
                // loaded texture may share that block): the target's engine image, TEX0's buffer width, format and
                // power-of-two size as the target's.
                const auto lo=word(manager+0x1010+16*index),hi=word(manager+0x1014+16*index);
                const auto log2=[](int32_t v){uint32_t n=0;while((1<<n)<v)++n;return (1<<n)==v?n:99u;};
                if(((lo>>14)&0x3f)!=rendered->bufferWidth() || ((lo>>20)&0x3f)!=rendered->format() || ((lo>>26)&15)!=log2(rendered->width) ||
                   ((lo>>30)|(hi&3)<<2)!=log2(rendered->height) || word(block+0x9c) || scene.offscreenTarget())
                    throw std::runtime_error(std::format("native mesh samples offscreen target {:x} with another layout (TEX0 {:08x}{:08x})",rendered->block(),hi,lo));
                mat.fromFrame=1;mat.source=rendered->image;feedbackTexture=true;
                if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:models] object=%x samples target %x image %u tcc=%u tfx=%u blend=%u test=%u\n",object,rendered->block(),
                    unsigned(rendered->image),unsigned(material[2]),unsigned(material[3]),unsigned(material[0]),unsigned(mat.alphaTest));}
            else{
                // A texture whose disc copies differ ('vp18' 1a5, battle frame 74880): the copy in the mesh's
                // own pack, else the one the battle loader published at the entry's TEX0 blocks.
                const uint64_t tex0=word(manager+0x1010+16*index)|uint64_t(word(manager+0x1014+16*index))<<32;
                const auto meshPart=origin && origin->kind==AssetOrigin::Kind::Part?std::optional(std::pair{origin->disc,origin->part}):std::nullopt;
                batch.texture=textures.modelImage(key,tex0,meshPart)->picture(word(block+0x9c));}}
        mat.textured=bool(batch.texture);
        // The GS draws an untextured vertex colour as it is (0..255), not as 0x80 = 1
        // modulation: the inn's save-point light column (3e/3f/61) was twice as bright.
        mat.directColour=!mat.textured && !feedbackTexture;
        const bool editable=(live || (working && mesh>=model)) && !m.has(6) && !m.has(8);
        if(live && !editable)throw std::runtime_error("native working-copy mesh with fixed-point vertices (routines 6 / 8)");
        const auto source=editable?(live?live:work+mesh-model):0;
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
        // Per-mesh scratch kept between calls (the draw runs on the game's thread only): no allocation per mesh.
        thread_local std::vector<bozo::render::Vertex> vertices;thread_local std::vector<std::array<float,4>> projected;thread_local std::vector<uint8_t> outside;
        vertices.assign(m.vertices.size(),bozo::render::Vertex{});
        thread_local std::vector<std::array<float,2>> passUv,projectedUv;thread_local std::vector<uint32_t> emitted,projectedRgba,passRgba;emitted.clear();
        if(secondPass){passUv.assign(m.vertices.size(),{});passRgba.assign(m.vertices.size(),0);}
        thread_local std::vector<std::array<float,3>> shellP,shellN;thread_local std::vector<std::array<float,4>> shellFactor;
        if(shellRoutine){shellP.assign(m.vertices.size(),{});shellN.assign(m.vertices.size(),{});shellFactor.assign(m.vertices.size(),{});}
        if(projector){projectedUv.assign(m.vertices.size(),{});projectedRgba.assign(m.vertices.size(),0);}projected.clear();projected.reserve(m.vertices.size());outside.clear();outside.reserve(m.vertices.size());
        // The vertex loop's lighting factor at position p, normal n (its own e = eye - p): the shell redraws (12 / 14) light
        // their moved vertices the same way, as the VU's redraw (08b8) runs the lighting again.
        // The VU saturates an overflow at +-max instead of infinity: the specular term squared power times
        // (x^(2^power), power up to 16) overflows for |n . h| > 1 (map 487), and the lit colour then clamps at 255.
        const auto vuSaturate=[](float v){return std::isinf(v)?std::copysign(std::numeric_limits<float>::max(),v):v;};
        const auto lightFactor=[&](const std::array<float,3> &p,const std::array<float,3> &n){
            std::array<float,3> e{eye[0]-p[0],eye[1]-p[1],eye[2]-p[2]};const auto length=std::sqrt(dot(e,e));
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
                    specular[l]=dot(h,n);for(int j=0;j<power;++j)specular[l]=vuSaturate(specular[l]*specular[l]);if(plain)specular[l]=0;}
                for(unsigned c=0;c<3;++c){factor[c]=ambient[c]*wa;for(unsigned l=0;l<4;++l)factor[c]+=colours[l][c]*diffuse[l];
                    for(const unsigned l:{0u,3u,1u,2u})factor[c]=vuSaturate(factor[c]+vuSaturate(colours[l][c]*specular[l]));}}
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
            return factor;};
        for(size_t k=0;k<m.vertices.size();++k){const auto &v=m.vertices[k];std::array<float,3> p{v.p[0],v.p[1],v.p[2]},n{v.n[0],v.n[1],v.n[2]};
            if(editable){if(m.sources[k].position!=UINT32_MAX)for(unsigned c=0;c<3;++c)p[c]=real(source+m.sources[k].position+4*c);
                if(m.sources[k].normal!=UINT32_MAX)for(unsigned c=0;c<3;++c)n[c]=real(source+m.sources[k].normal+4*c);}
            const auto &uvk=liveUv.empty()?std::array<float,2>{v.uv[0],v.uv[1]}:liveUv[k];
            p=skinned(k,p,1);n=skinned(k,n,0);auto &vout=vertices[k];vout.x=p[0];vout.y=p[1];vout.z=p[2];vout.u=uvk[0]+scroll[0];vout.v=uvk[1]+scroll[1];
            const auto clip=apply(screen,{p[0],p[1],p[2],1}),g=apply(guard,{p[0],p[1],p[2],1});projected.push_back(clip);
            outside.push_back(std::fabs(g[0])>std::fabs(g[3]) || std::fabs(g[1])>std::fabs(g[3]) || std::fabs(g[2])>std::fabs(g[3]));
            if(mat.fog && clip[3]>0)vout.fog=std::clamp(camera->fog[0]+camera->fog[1]/clip[3],0.0f,255.0f)/255;
            std::array<float,3> e{eye[0]-p[0],eye[1]-p[1],eye[2]-p[2]};const auto length=std::sqrt(dot(e,e));
            if(m.has(36)){const float d=dot(n,e),inv=length?1/length:0;std::array<float,3> reflection{2*d*n[0]-e[0],2*d*n[1]-e[1],2*d*n[2]-e[2]},view{};
                for(unsigned r=0;r<3;++r)view[r]=(viewObject[4*r]*reflection[0]+viewObject[4*r+1]*reflection[1]+viewObject[4*r+2]*reflection[2])*inv;
                const float q=std::sqrt(std::max(1-view[2],0.1f));vout.u=(-view[0]+1.41421f*q)/(2.82843f*q);vout.v=(view[1]+1.41421f*q)/(2.82843f*q);}
            if(m.has(20)){
                // Skinning (2, 4) before 20 gives it the skinned normal, as computed above (battle 0x1a: routines 8 2 20; the
                // palette's output is object space, drawn through the same screen matrix as unskinned meshes).
                const auto at20=std::find(m.algorithms.begin(),m.algorithms.end(),20);
                const bool skinAfter=std::find(at20,m.algorithms.end(),2)!=m.algorithms.end() || std::find(at20,m.algorithms.end(),4)!=m.algorithms.end();
                if(skinAfter || m.has(36)){std::string order;for(const auto r:m.algorithms)order+=std::format(" {}",int(r));
                    throw std::runtime_error("unqualified native sphere map with routine 36 or skinning after it (routines"+order+")");}
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
            const auto factor=lightFactor(p,n);
            for(unsigned c=0;c<4;++c)if(!std::isfinite(factor[c]))throw std::runtime_error(std::format("nonfinite native model light factor {} {} {} {} (mode {}, point lights {}, spots {}, normal {} {} {}, position {} {} {}; vertex {} of mesh {:x}, routines {}.{}.{}.{}, palette {:x}, editable {}, source {:x})",factor[0],factor[1],factor[2],factor[3],material[4],pointLights,spots.size(),n[0],n[1],n[2],p[0],p[1],p[2],k,mesh,m.algorithms[0],m.algorithms[1],m.algorithms[2],m.algorithms[3],palette,editable,source));
            vout.rgba=0;for(unsigned c=0;c<4;++c){finite(factor[c]);const auto lit=uint32_t(std::clamp((c==3?alphaRow:float(v.colour[c]))*factor[c],0.0f,limit[c]>0?std::min(limit[c],255.0f):255.0f));vout.rgba|=lit<<(8*c);}
            {const float lanes[6]={vout.x,vout.y,vout.z,vout.u,vout.v,vout.fog};
             for(unsigned i=0;i<6;++i)if(!std::isfinite(lanes[i])){std::string order;for(const auto r:m.algorithms)order+=std::format(" {}",int(r));
                 throw std::runtime_error(std::format("nonfinite native model vertex {} (vertex {}, position {} {} {}; routines{})","x\0y\0z\0u\0v\0fog"+2*i,k,p[0],p[1],p[2],order));}}
            if(secondPass)passRgba[k]=vout.rgba;
            if(shellRoutine){shellP[k]=p;shellN[k]=n;shellFactor[k]=factor;}
            if(secondPass){auto &uv=passUv[k];uv={vout.u,vout.v};
                if(passRoutine==22){const auto stq=sphereMapTexture(n,eyeDirection,rotation);uv={stq[2]?float(stq[0])/float(stq[2]):.5f,stq[2]?float(stq[1])/float(stq[2]):.5f};}
                else if(passRoutine==50){uv[0]+=passScroll[0];uv[1]+=passScroll[1];}
                else if(passRoutine==18){if(m.listedUv.size()!=m.vertices.size())throw std::runtime_error("native second pass (routine 18) without its UV list");
                    uv={float(m.listedUv[k][0])/4096,float(m.listedUv[k][1])/4096};}
                else if(passRoutine==28){const auto &r=m.secondReflection;const auto t=reflectionTexture(n,p,eye,rotation,r.base,r.on,r.scale);
                    uv={t.stq[2]?float(t.stq[0])/float(t.stq[2]):.5f,t.stq[2]?float(t.stq[1])/float(t.stq[2]):.5f};
                    if(t.alpha){const auto lit=uint32_t(std::clamp(float(*t.alpha)*factor[3],0.0f,limit[3]>0?std::min(limit[3],255.0f):255.0f));passRgba[k]=(vout.rgba&0xffffffu)|lit<<24;}}
                for(auto f:uv)finite(f);}
            if(projector){const auto &pr=*projector;
                const auto t=projectedTexture(p,n,pr.origin,pr.cols,pr.mask);const int32_t S=t.stq[0],T=t.stq[1],Q=t.stq[2];
                projectedUv[k]={Q?float(S)/float(Q):.5f,Q?float(T)/float(Q):.5f};for(auto x:projectedUv[k])finite(x);
                const auto lit=uint32_t(std::clamp((t.off?0.0f:pr.level)*factor[3],0.0f,limit[3]>0?std::min(limit[3],255.0f):255.0f));
                projectedRgba[k]=(vout.rgba&0xffffffu)|lit<<24;}
        }
        if(shadowClass){
            std::vector<std::array<float,3>> positions(vertices.size());for(size_t k=0;k<vertices.size();++k)positions[k]={vertices[k].x,vertices[k].y,vertices[k].z};
            nativeShadowPrisms(positions,m.strips,shadowLight,matrix,shadowFlat,out.shadowIncrement,out.shadowDecrement,shadowPoint);continue;}
        const bool cull=(block?word(block+0x90)==0:bool(material[5])) && !byte(object+0x210);size_t first=0;unsigned guarded=0,culled=0;
        batch.vertices.reserve(m.vertices.size()*3);
        if(material[6]&6){
            // The GS's LINE STRIP (each vertex continues the line) or LINE (pairs), one pixel wide.
            mat.lines=true;const bool strip=material[6]&2;
            for(auto length:m.strips){for(unsigned k=1;k<length;k+=strip?1:2){const auto a=first+k-1,b=first+k;
                if(outside[a] || outside[b]){++guarded;continue;}batch.vertices.push_back(vertices[a]);batch.vertices.push_back(vertices[b]);
                if(secondPass || projector || embossRoutine){emitted.push_back(uint32_t(a));emitted.push_back(uint32_t(b));}}first+=length;}
        }else
        for(auto length:m.strips){for(unsigned k=2;k<length;++k){const auto a=first+k-2,b=first+k-1,c=first+k;
            if(outside[a] || outside[b] || outside[c]){++guarded;continue;}
            if(cull && projected[a][3]>0 && projected[b][3]>0 && projected[c][3]>0){const auto &p=projected[a],&q=projected[b],&r=projected[c];
                const float area=(p[0]/p[3]-q[0]/q[3])*(q[1]/q[3]-r[1]/r[3])-(p[1]/p[3]-q[1]/q[3])*(q[0]/q[3]-r[0]/r[3]);
                if(!std::signbit(area*((k&1)?-1.0f:1.0f))){++culled;continue;}}
            batch.vertices.push_back(vertices[a]);batch.vertices.push_back(vertices[(k&1)?c:b]);batch.vertices.push_back(vertices[(k&1)?b:c]);
            if(secondPass || projector || embossRoutine){emitted.push_back(uint32_t(a));emitted.push_back(uint32_t((k&1)?c:b));emitted.push_back(uint32_t((k&1)?b:c));}}first+=length;}
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
        std::vector<std::pair<int,NativeModelBatch>> extra;
        if(secondPass && !batch.vertices.empty()){
            // The second pass (routine 16, 22 or 50): the same triangles with its UVs and texture, blended with its ALPHA_1, after pass 1.
            auto second=batch;if(emitted.size()!=second.vertices.size())throw std::runtime_error("native second pass lost its vertex order");
            for(size_t e=0;e<emitted.size();++e){second.vertices[e].u=passUv[emitted[e]][0];second.vertices[e].v=passUv[emitted[e]][1];second.vertices[e].rgba=passRgba[emitted[e]];}auto &m2=second.material;
            if(secondPass->blended){m2.alphaBlend=true;m2.blend=bozo::render::Material::Blend::Alpha;m2.gsBlend=true;
            const auto a=secondPass->alpha;m2.blendA=a&3;m2.blendB=(a>>2)&3;m2.blendC=(a>>4)&3;m2.blendD=(a>>6)&3;m2.blendFix=m2.blendC==2?secondPass->fix:0x80;}
            if(material[1]){
                if(!manager)throw std::runtime_error("native mesh second pass without the texture manager");
                const auto index=value<int16_t>(rt,manager+0x2670+2*secondPass->slot);if(index<0)throw std::runtime_error("native mesh second-pass texture not registered");
                const TextureKey key{word(manager+0x1008+16*index),word(manager+8+16*index)};
                const auto lo=word(manager+0x1010+16*index);const bool feedbackKey=key.tag==0x4674404c && key.index==1;
                if(lo!=secondPass->tex0 || (feedbackKey && !feedbackTexture))
                    throw std::runtime_error(std::format("native mesh second pass: slot {} holds TEX0 {:08x}, row 18 writes {:08x}",secondPass->slot,lo,secondPass->tex0));
                // Pass 1 over the screen-feedback picture (maps 103, 445): pass 2 keeps that source when its slot is the same
                // picture ('L@tF' 1), else samples its own texture.
                if(feedbackKey)goto secondTextured;
                m2.fromFrame=0;m2.source=0;
                const uint64_t tex0=lo|uint64_t(word(manager+0x1014+16*index))<<32;
                const auto meshPart=origin && origin->kind==AssetOrigin::Kind::Part?std::optional(std::pair{origin->disc,origin->part}):std::nullopt;
                second.texture=textures.modelImage(key,tex0,meshPart)->picture(secondPass->part);m2.textured=true;}
            secondTextured:
            m2.depthWrite=bool(palette);
            if(!embossUv.empty()){
                auto third=second;for(size_t e=0;e<emitted.size();++e){third.vertices[e].u=embossUv[emitted[e]][0];third.vertices[e].v=embossUv[emitted[e]][1];}
                auto &m3=third.material;m3.blendA=0;m3.blendB=2;m3.blendC=0;m3.blendD=1;   // ALPHA 0x48: Cs As + Cd
                extra.push_back({passSlot,std::move(second)});extra.push_back({embossSlot,std::move(third)});}
            else extra.push_back({passSlot,std::move(second)});}
        if(embossRoutine==32 && !batch.vertices.empty()){
            auto again=batch;if(emitted.size()!=again.vertices.size())throw std::runtime_error("native UV-shift pass lost its vertex order");
            for(size_t e=0;e<emitted.size();++e){again.vertices[e].u=embossUv[emitted[e]][0];again.vertices[e].v=embossUv[emitted[e]][1];}
            auto &ma=again.material;ma.alphaBlend=true;ma.blend=bozo::render::Material::Blend::Alpha;ma.gsBlend=true;ma.blendA=0;ma.blendB=2;ma.blendC=0;ma.blendD=1;   // ALPHA 0x48
            ma.depthWrite=false;extra.push_back({embossSlot,std::move(again)});}
        if(projector && !batch.vertices.empty()){
            // Routine 46's pass: the same triangles with the projected S/T and alpha, the projector's texture, blended with r16's
            // ALPHA. Repeating: Logun's meshes span S, T over several units (-5..5) and each object's r46 offsets the
            // projection so the pattern stays continuous in world space (the CLAMP register it draws with is not in these rows).
            auto third=batch;if(emitted.size()!=third.vertices.size())throw std::runtime_error("native projected texture lost its vertex order");
            for(size_t e=0;e<emitted.size();++e){auto &v=third.vertices[e];v.u=projectedUv[emitted[e]][0];v.v=projectedUv[emitted[e]][1];v.rgba=projectedRgba[emitted[e]];}
            auto &m3=third.material;const auto &pr=*projector;m3.alphaBlend=true;m3.blend=bozo::render::Material::Blend::Alpha;m3.gsBlend=true;
            m3.blendA=pr.alpha&3;m3.blendB=(pr.alpha>>2)&3;m3.blendC=(pr.alpha>>4)&3;m3.blendD=(pr.alpha>>6)&3;m3.blendFix=0x80;
            third.texture=pr.texture;m3.textured=true;m3.directColour=false;m3.fromFrame=0;m3.textureAlpha=pr.textureAlpha;
            m3.textureFunction=static_cast<bozo::render::Material::TextureFunction>(pr.textureFunction);m3.clampU=m3.clampV=false;
            m3.fog=(pr.prim&32) && camera->fog[1]!=0;m3.depthWrite=false;m3.lines=false;
            if(traceModels){unsigned lit=0;float u0=1e9,u1=-1e9,v0=1e9,v1=-1e9;for(const auto &v:third.vertices){lit+=(v.rgba>>24)!=0;u0=std::min(u0,v.u);u1=std::max(u1,v.u);v0=std::min(v0,v.v);v1=std::max(v1,v.v);}
                std::fprintf(stderr,"[vp2s:projected] object=%x mesh=%x vertices=%zu lit=%u u=%.3f..%.3f v=%.3f..%.3f level=%.0f texture=%ux%u\n",object,mesh,third.vertices.size(),lit,u0,u1,v0,v1,pr.level,
                    pr.texture?pr.texture->width:0,pr.texture?pr.texture->height:0);}
            extra.push_back({projectorSlot,std::move(third)});}
        if(shellRoutine && int32_t(shellData[1])>0 && !batch.vertices.empty()){
            const auto shellMaterial=batch.material;const auto shellTexture=batch.texture;const auto shellMatrix=batch.matrix;
            std::vector<NativeShellVertex> start(m.vertices.size());
            for(size_t k=0;k<m.vertices.size();++k){for(unsigned c=0;c<4;++c)start[k].colour[c]=m.vertices[k].colour[c];start[k].p=shellP[k];start[k].n=shellN[k];start[k].uv={vertices[k].u,vertices[k].v};}
            const auto redraw=[&](const std::vector<NativeShellVertex> &state){
                NativeModelBatch shell;shell.matrix=shellMatrix;shell.material=shellMaterial;shell.texture=shellTexture;
                std::vector<bozo::render::Vertex> verts(m.vertices.size());std::vector<std::array<float,4>> clip(m.vertices.size());std::vector<uint8_t> away(m.vertices.size());
                for(size_t k=0;k<m.vertices.size();++k){auto &o=verts[k];const auto &q=state[k].p;const auto relit=lightFactor(q,state[k].n);for(auto f:relit)finite(f);o.x=q[0];o.y=q[1];o.z=q[2];o.u=state[k].uv[0];o.v=state[k].uv[1];
                    clip[k]=apply(screen,{q[0],q[1],q[2],1});const auto g=apply(guard,{q[0],q[1],q[2],1});
                    away[k]=std::fabs(g[0])>std::fabs(g[3]) || std::fabs(g[1])>std::fabs(g[3]) || std::fabs(g[2])>std::fabs(g[3]);
                    if(shellMaterial.fog && clip[k][3]>0)o.fog=std::clamp(camera->fog[0]+camera->fog[1]/clip[k][3],0.0f,255.0f)/255;
                    o.rgba=0;for(unsigned c=0;c<4;++c){const auto lit=uint32_t(std::clamp(float(int32_t(state[k].colour[c]))*relit[c],0.0f,limit[c]>0?std::min(limit[c],255.0f):255.0f));o.rgba|=lit<<(8*c);}
                    for(auto f:{o.x,o.y,o.z,o.u,o.v,o.fog})finite(f);}
                size_t at=0;for(auto length:m.strips){for(unsigned k=2;k<length;++k){const auto a=at+k-2,b=at+k-1,c=at+k;
                    if(away[a] || away[b] || away[c])continue;
                    if(cull && clip[a][3]>0 && clip[b][3]>0 && clip[c][3]>0){const auto &p0=clip[a],&p1=clip[b],&p2=clip[c];
                        const float area=(p0[0]/p0[3]-p1[0]/p1[3])*(p1[1]/p1[3]-p2[1]/p2[3])-(p0[1]/p0[3]-p1[1]/p1[3])*(p1[0]/p1[3]-p2[0]/p2[3]);
                        if(!std::signbit(area*((k&1)?-1.0f:1.0f)))continue;}
                    shell.vertices.push_back(verts[a]);shell.vertices.push_back(verts[(k&1)?c:b]);shell.vertices.push_back(verts[(k&1)?b:c]);}at+=length;}
                if(!shell.vertices.empty())extra.push_back({4,std::move(shell)});};
            for(const auto &state:nativeShells(shellRoutine,shellData,std::move(start)))redraw(state);}
        if(!batch.vertices.empty()){out.batches.push_back(std::move(batch));
            std::stable_sort(extra.begin(),extra.end(),[](const auto &x,const auto &y){return x.first<y.first;});   // table B runs in routine order
            for(auto &[slot,b]:extra)out.batches.push_back(std::move(b));}
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
    auto *d=rt?vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services):nullptr;
    auto *owner=d?vp2s::nativeCast<ModelOwner>(&d->host()):nullptr;auto *scene=d?vp2s::nativeCast<SceneOwner>(&d->host()):nullptr;
    auto *assets=d?vp2s::nativeCast<AssetOwner>(&d->host()):nullptr;auto *textures=d?vp2s::nativeCast<TextureOwner>(&d->host()):nullptr;
    if(!c || !owner || !scene || !assets || !textures || c->pc!=0x239fd0 || c->r[5].uw[0]<0x100650)
        throw std::runtime_error("native model submission requires its complete scene producer");
    const auto object=c->r[5].uw[0]-0x650;
    NativeModelSubmission submission;
    auto *feedback=vp2s::nativeCast<FeedbackOwner>(&d->host());
    try{submission=owner->nativeModels().capture(rt,assets->nativeAssets(),textures->nativeTextures(),scene->nativeScene(),object,feedback?&feedback->nativeFeedbackCopies():nullptr);}
    catch(const std::runtime_error &e){throw std::runtime_error(std::format("native model object={:08x}: {}",object,e.what()));}
    owner->drawNativeModel(submission);c->r[4].sd[0]=int32_t(object);
    return true; // Original compiled scene completion follows accepted native draws.
}
}

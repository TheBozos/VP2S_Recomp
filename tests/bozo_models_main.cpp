#include "native_models.h"
#include "bozo/ops_fpu.h"
#include <bit>
#include <fstream>
#include <cstring>
#include <source_location>
#include <cstdio>
#include <cmath>
#include <immintrin.h>
namespace {
void check(bool yes,std::source_location at=std::source_location::current()){if(!yes)throw std::runtime_error("native model assertion at "+std::to_string(at.line()));}
template<class T>void put(Runtime &rt,uint32_t at,T v){std::memcpy(rt.memory.hostPointer(at,sizeof(T)),&v,sizeof(T));}
template<class T>void write(std::ofstream &out,T v){out.write(reinterpret_cast<const char *>(&v),sizeof(T));}
template<class F>void rejects(F f){bool yes=false;try{f();}catch(const std::runtime_error &){yes=true;}check(yes);}
constexpr uint32_t object=0x160000,model=0x150000,camera=0x170000,target=0x180000,current=0x190000;
using Mat=std::array<float,16>;
const Mat identity{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
void fixture(const std::filesystem::path &dir,int algorithm){
    std::filesystem::create_directories(dir);std::ofstream f(dir/"0000004000_001.mesh",std::ios::binary);
    for(uint32_t n:{0x314e5056,1,0x60,4,1,1})write(f,n);
    write(f,std::array<int32_t,4>{algorithm,48,0,0});write(f,uint32_t(1));
    write(f,std::array<uint32_t,11>{0,0,0,0,2,0,0,0,1,0,1});write(f,uint16_t(4));
    const float xy[4][2]={{-.5f,-.5f},{.5f,-.5f},{-.5f,.5f},{.5f,.5f}};
    for(unsigned k=0;k<4;++k){write(f,std::array<float,4>{xy[k][0],xy[k][1],.5f,656});write(f,std::array<float,3>{0,0,1});
        write(f,std::array<float,2>{float(k&1),float(k>>1)});write(f,uint32_t(0x80402010));}
    if(algorithm==4)for(unsigned k=0;k<4;++k){write(f,int32_t(656));write(f,.25f);write(f,int32_t(660));write(f,.75f);}
    write(f,int32_t(0));write(f,uint32_t(0x200));for(unsigned k=0;k<4;++k){write(f,UINT32_MAX);write(f,uint32_t(0xc0+12*k));}
}
}
int main(int argc,char **argv){try{
    check(argc==2);const std::filesystem::path dir=argv[1];
    unsigned cases=0,emptyCases=0;
    for(int algorithm:{0,2,4}){
        fixture(dir,algorithm);vp2s::NativeModels models(dir);Runtime rt;vp2s::NativeScene scene;vp2s::NativeAssets assets;vp2s::NativeTextures textures;
        std::memset(rt.memory.flat(),0,bozo::kFlatBytes);
        // This fixture uses the constructor's two-field base height (448).
        put(rt,0x1c8d00+0xcc84,uint8_t(1));
        put(rt,0x35ddf8,current);put(rt,current+0x40a8,object);put(rt,current+0x40dc,uint8_t(1));
        put(rt,target+0x3c76,uint8_t(1));put(rt,target+0x3c60,1.f);put(rt,target+0x3c64,1.f);
        put(rt,target+0x3c68,int16_t(640));put(rt,target+0x3c6a,int16_t(448));
        for(unsigned i=0;i<2;++i){const auto base=target+0xa70*i;put(rt,base+0x1cf0,uint8_t(1));put(rt,base+0x1cf2,uint8_t(i));put(rt,base+0x1cf5,uint8_t(1));
            put(rt,base+0x1cfa,int16_t(640));put(rt,base+0x1cfc,int16_t(224));}
        scene.beginComposition(&rt,0x1f0000,target);
        put(rt,camera+0x220,uint32_t(0x153120));put(rt,camera+0x8e0,uint32_t(0x1a0000));put(rt,0x1a0000,uint32_t(0x153140));
        put(rt,camera+0x2f0,identity);put(rt,camera+0x370,identity);auto screen=identity;screen[0]=100;screen[5]=100;screen[3]=2048;screen[7]=2048;
        put(rt,camera+0x330,screen);put(rt,camera+0x244,2048.f);put(rt,camera+0x248,2048.f);put(rt,camera+0x260,1728.f);put(rt,camera+0x264,1824.f);
        put(rt,camera+0x5a8,0.f);put(rt,camera+0x5ac,1000.f);   // fog F = a + b/w (only for FGE meshes)
        scene.selectCamera(&rt,camera);
        put(rt,object+0x90,identity);put(rt,object+0x7bc,model);put(rt,model+0x34,uint32_t(1));put(rt,model+0x64,int16_t(0x200));
        put(rt,model+0x74,uint32_t(1));put(rt,model+0x60+0x40+0x90,uint32_t(1));
        put(rt,object+0x300,std::array<float,4>{1,1,1,1}); // Object colour row13.
        put(rt,object+0x310,uint32_t(1));
        if(algorithm){put(rt,object+0x648,uint32_t(0x1b0000));put(rt,0x1b0034,uint32_t(0x1b0100));put(rt,0x1b0104,uint32_t(2));put(rt,0x1b004c,uint32_t(0x1c0000));
            auto a=identity,b=identity;a[12]=.1f;b[13]=.2f;put(rt,0x1c0040,a);put(rt,0x1c0080,b);}
        assets.discRead(0x140000,1,8);assets.begin(0x1f1000,0x140000,model,1);assets.finish(0x1f1000,true,0x1000);
        std::shared_ptr<const bozo::render::Texture> unused;
        for(unsigned seed=0;seed<128;++seed){
            const float alpha=float(seed)/128;put(rt,object+0x30c,alpha);put(rt,object+0x230+16*38,std::array<float,4>{0,0,.125f,.25f});
            const bool morph=seed&1;if(morph){const uint32_t aliases[5]={0,0x20000000u,0x30000000u,0x80000000u,0xa0000000u};
                put(rt,object+0x630,uint32_t(0x1d0000)|aliases[(seed/2)%5]);std::memcpy(rt.memory.hostPointer(0x1d0000,0x260),rt.memory.hostPointer(model,0x260),0x260);
                for(unsigned k=0;k<4;++k)put(rt,0x1d0060+0xc0+12*k,std::array<float,3>{(k&1)?.4f:-.4f,(k>>1)?.4f:-.4f,.5f});}
            else {const uint32_t nonModels[10]={0,0x3f7ffffeu,0x10003000u,0xffffffffu,0x01fffff0u,0x00100001u,0x80000000u,model,0x1e0000u,0x01ffff90u};
                put(rt,object+0x630,nonModels[(seed/2)%10]);put(rt,0x1e0034,uint32_t(2));}
            const auto before=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
            const auto result=models.capture(&rt,assets,textures,scene,object);check(result.batches.size()==1);
            const auto &batch=result.batches[0];check(batch.vertices.size()==6 && !batch.texture && !batch.material.textured && batch.material.depthWrite);
            check(batch.matrix[0]==100 && batch.matrix[5]==100 && batch.matrix[12]==2048 && batch.matrix[13]==2048);
            const unsigned indices[6]={0,1,2,1,3,2};const float extent=morph?.4f:.5f;
            for(unsigned j=0;j<6;++j){const auto k=indices[j];const auto &v=batch.vertices[j];
                const float x=(k&1?extent:-extent)+(algorithm==2?.1f:algorithm==4?.025f:0);
                const float y=(k>>1?extent:-extent)+(algorithm==4?.15f:0);
                check(std::fabs(v.x-x)<1e-6f && std::fabs(v.y-y)<1e-6f && v.z==.5f);
                check(v.u==float(k&1)+.125f && v.v==float(k>>1)+.25f);
                check(v.rgba==((seed<<24)|0x402010));}
            check(!std::memcmp(before.data(),rt.memory.flat(),before.size()));
            put(rt,object+0x30c,0.f);check(batch.vertices[0].rgba==((seed<<24)|0x402010)); // immutable captured data
            // Light table A (mem[14].w = +31c) alone keeps the factor; its point (B, +314)
            // and spot (C, +318) tables are not ported and reject explicitly.
            // PRIM.FGE is read live from the block's GIF tag in EE RAM (row 13: +70 EOP|nloop,
            // +74 PRIM<<15, +78 REGS 412); a tag that is not this one falls back to the pack.
            if(seed%8==5){const uint32_t block=model+0x60+0x40;check(!batch.material.fog);
                put(rt,block+0x70,uint32_t(0x8000));put(rt,block+0x78,uint32_t(0x412));
                put(rt,block+0x74,uint32_t(0x34u<<15));{const auto r=models.capture(&rt,assets,textures,scene,object);check(r.batches.size()==1 && r.batches[0].material.fog);
                    for(const auto &v:r.batches[0].vertices)check(v.fog>0 && v.fog<=1);}
                put(rt,block+0x74,uint32_t(0x14u<<15));check(!models.capture(&rt,assets,textures,scene,object).batches[0].material.fog);
                put(rt,block+0x74,uint32_t(0x34u<<15));put(rt,block+0x78,uint32_t(0x413));check(!models.capture(&rt,assets,textures,scene,object).batches[0].material.fog);
                for(uint32_t at:{0x70u,0x74u,0x78u})put(rt,block+at,uint32_t(0));}
            if(seed%8==3){put(rt,object+0x31c,uint32_t(1));
                {const auto lit=models.capture(&rt,assets,textures,scene,object);check(lit.batches.size()==1 && lit.batches[0].vertices[0].rgba==0x402010u);} // alpha cleared above
                put(rt,object+0x314,uint32_t(1));rejects([&]{models.capture(&rt,assets,textures,scene,object);});put(rt,object+0x314,uint32_t(0));
                put(rt,object+0x318,uint32_t(2));rejects([&]{models.capture(&rt,assets,textures,scene,object);});put(rt,object+0x318,uint32_t(0));
                put(rt,object+0x31c,uint32_t(0));}
            put(rt,model+0x59,uint8_t(1));put(rt,model+0x96,uint16_t(1));put(rt,object+0x7a4,uint16_t(1));
            check(models.capture(&rt,assets,textures,scene,object).batches.empty());put(rt,object+0x7a4,uint16_t(0));
            put(rt,current+0x40dc,uint8_t(0));rejects([&]{models.capture(&rt,assets,textures,scene,object);});put(rt,current+0x40dc,uint8_t(1));
            ++cases;
        }
        {constexpr uint32_t empty=0x1e1000;std::vector<uint8_t> saved(rt.memory.flat()+object,rt.memory.flat()+object+0x800);
            std::memset(rt.memory.hostPointer(empty-0x10,0x70),0x7f,0x70);put(rt,empty-0x10,uint32_t(0x00535046));put(rt,empty+0x34,uint32_t(0));put(rt,object+0x7bc,empty);
            const auto provenance=[&]{assets.begin(0x1f2000,0x140000,empty-0x10,1);assets.finish(0x1f2000,true,0x70);};provenance();
            for(unsigned seed=0;seed<32;++seed){
                std::array<uint32_t,16> unused;unused.fill(seed&1?0x7fa01234:0xffffffff);put(rt,object+0x90,unused);
                put(rt,object+0x648,uint32_t(0xdeadbeef));put(rt,object+0x77c,uint32_t(0xdeadbeef));put(rt,object+0x630,uint32_t(seed*0x1020304));
                put(rt,object+0x230,unused);const auto fp=_mm_getcsr();const auto before=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
                const auto result=models.capture(&rt,assets,textures,scene,object);check(result.object==object && result.camera==camera && result.batches.empty());
                check(_mm_getcsr()==fp && !std::memcmp(before.data(),rt.memory.flat(),before.size()));++emptyCases;
            }
            put(rt,current+0x40dc,uint8_t(0));rejects([&]{models.capture(&rt,assets,textures,scene,object);});put(rt,current+0x40dc,uint8_t(1));
            // Composite draws (class 1599a0, 2b7ab0): the scene's pending object owns
            // the models its child slots and child list reach (route: 999540 -> 9a2920).
            {constexpr uint32_t owner=0x1e4000,child=0x1e6000,nested=0x1e8000,kComposite=0x1599a0;
                const auto composite=[&](uint32_t at){std::memset(rt.memory.hostPointer(at,0x900),0,0x900);put(rt,at,kComposite);put(rt,at+0x8d4,at+0x8cc);};
                const auto admitted=[&]{return !models.capture(&rt,assets,textures,scene,object).batches.size();};
                composite(owner);composite(child);composite(nested);put(rt,current+0x40a8,owner);
                rejects([&]{models.capture(&rt,assets,textures,scene,object);});++emptyCases;            // unreachable
                put(rt,owner+0x8c4,object);check(admitted());put(rt,owner+0x8c4,uint32_t(0));++emptyCases;   // owner slot
                put(rt,owner+0x8d4,child);put(rt,child+8,owner+0x8cc);put(rt,child+0x8c0,object);check(admitted());++emptyCases; // list child's slot
                put(rt,child+0x8c0,uint32_t(0));put(rt,child+0x8d4,nested);put(rt,nested+8,child+0x8cc);put(rt,nested+0x8c8,object);check(admitted());++emptyCases; // nested
                put(rt,nested+0x8c8,uint32_t(0));uint32_t link;std::memcpy(&link,rt.memory.hostPointer(object+8,4),4);
                put(rt,child+0x8d4,object);put(rt,object+8,child+0x8cc);check(admitted());put(rt,object+8,link);++emptyCases; // list child itself
                put(rt,child+0x8d4,child+0x8cc);put(rt,owner,uint32_t(0x153700));put(rt,owner+0x8c4,object);
                rejects([&]{models.capture(&rt,assets,textures,scene,object);});put(rt,owner,kComposite);++emptyCases;   // not a composite
                put(rt,owner+0x8c4,uint32_t(0));put(rt,owner+0x8d4,child);put(rt,child+8,child);   // a list that never ends
                rejects([&]{models.capture(&rt,assets,textures,scene,object);});++emptyCases;
                put(rt,current+0x40a8,object);}
            put(rt,empty-0x10,uint32_t(0));rejects([&]{models.capture(&rt,assets,textures,scene,object);});put(rt,empty-0x10,uint32_t(0x00535046));
            assets.overwrite(empty+0x5f,1);rejects([&]{models.capture(&rt,assets,textures,scene,object);});provenance();
            assets.overwrite(empty+0x10,1);rejects([&]{models.capture(&rt,assets,textures,scene,object);});provenance();
            assets.begin(0x1f2000,0x140000,empty+0x5f,2);assets.finish(0x1f2000,true,1);rejects([&]{models.capture(&rt,assets,textures,scene,object);});provenance();
            assets.discRead(empty-0x10,1,8);rejects([&]{models.capture(&rt,assets,textures,scene,object);});provenance();
            put(rt,empty+0x34,uint32_t(4097));rejects([&]{models.capture(&rt,assets,textures,scene,object);});put(rt,empty+0x34,uint32_t(0));
            assets.overwrite(empty-0x10,0x70);rejects([&]{models.capture(&rt,assets,textures,scene,object);});
            std::memcpy(rt.memory.flat()+object,saved.data(),saved.size());
        }
        assets.overwrite(model,0x1000);rejects([&]{models.capture(&rt,assets,textures,scene,object);});
    }
    // VU1 set-up routine 20 (code 0x1078) against its listing in PS2 words:
    // MUL/ADDAy/MADDz n.e; MULAx/MADDAx/MSUBw r; rows 73..75 with .y times -8,
    // MULAx/MADDAy/MADDAi.y(I=8)/MADDz; SQRT Q of .y; S,T = (x, z) + 0.5 Q, Q; FTOI12.
    unsigned sphereCases=0;
    {const auto M=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
     const auto A=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
     const auto S=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::subNearest(a,b,f);};
     const auto W=[](float x){return std::bit_cast<uint32_t>(x);};const auto F=[](uint32_t x){return std::bit_cast<float>(x);};
     uint32_t state=12345;const auto rnd=[&](){state=state*1664525u+1013904223u;return float(state>>8)/float(1u<<24)*2-1;};
     for(unsigned seed=0;seed<4096;++seed){
        std::array<float,3> n{rnd(),rnd(),rnd()},eye{rnd(),rnd(),rnd()};const float el=std::sqrt(eye[0]*eye[0]+eye[1]*eye[1]+eye[2]*eye[2]);for(auto &x:eye)x/=el;
        if(seed%3==0){const float nl=std::sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);for(auto &x:n)x/=nl;}
        // An orthonormal rotation from a random unit quaternion.
        float qv[4]{rnd(),rnd(),rnd(),rnd()};const float ql=std::sqrt(qv[0]*qv[0]+qv[1]*qv[1]+qv[2]*qv[2]+qv[3]*qv[3]);for(auto &x:qv)x/=ql;
        const float x=qv[0],y=qv[1],z=qv[2],w=qv[3];
        float rot[3][3]={{1-2*(y*y+z*z),2*(x*y+z*w),2*(x*z-y*w)},{2*(x*y-z*w),1-2*(x*x+z*z),2*(y*z+x*w)},{2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)}};
        const auto got=vp2s::sphereMapTexture(n,eye,rot);
        uint32_t e[3],nn[3];for(unsigned c=0;c<3;++c){e[c]=W(eye[c]);nn[c]=W(n[c]);}
        const uint32_t d=A(A(M(nn[0],e[0]),M(nn[1],e[1])),M(W(1),M(nn[2],e[2])));
        uint32_t r[3];for(unsigned c=0;c<3;++c)r[c]=S(A(M(nn[c],d),M(nn[c],d)),M(e[c],W(1)));
        uint32_t row[3][3];for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j){row[i][j]=W(rot[j][i]);if(j==1)row[i][j]=M(row[i][j],W(-8));}
        uint32_t v[3];for(unsigned j=0;j<3;++j){uint32_t acc=A(M(row[0][j],r[0]),M(row[1][j],r[1]));if(j==1)acc=A(acc,M(W(1),W(8)));v[j]=A(acc,M(row[2][j],r[2]));}
        const auto saved=_mm_getcsr();uint32_t f=0;const uint32_t q=bozo::ps2f::sqrt(v[1],f);_mm_setcsr(saved);
        const auto fixed=[&](uint32_t a){return int32_t(std::trunc(double(F(a))*4096));};
        const int32_t expect[3]={fixed(A(M(v[0],W(1)),M(W(.5f),q))),fixed(A(M(v[2],W(1)),M(W(.5f),q))),fixed(A(0,q))};
        for(unsigned c=0;c<3;++c)if(std::abs(got[c]-expect[c])>2){std::fprintf(stderr,"sphere map seed%u lane%u %d / %d\n",seed,c,got[c],expect[c]);check(false);}
        // The GS's S/Q, T/Q stay on the unit disc's square: |u - 1/2|, |v - 1/2| <= 1/2.
        // (For unit normals: then |r| = 1 and (v.x/q)^2 = (1 + v.y)/8 <= 1/4.)
        if(seed%3==0 && got[2]>64)for(unsigned c=0;c<2;++c)check(std::fabs(float(got[c])/float(got[2])-.5f)<=.5f+4.f/float(got[2]));
        ++sphereCases;}}
    std::printf("%u VU1 routine-20 sphere-map vertices match the listing; %u native model captures +%u qualified empty FPS models: independent positions/skinning/UV/colour/strip ordering, unused-input isolation, whole RAM/FP preservation and snapshot/asset lifetime\n",sphereCases,cases,emptyCases);
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

#include <format>
#include "native_models.h"
#include "native_math.h"
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
void fixture(const std::filesystem::path &dir,int algorithm,uint32_t mode=2,uint32_t textured=0){
    std::filesystem::create_directories(dir);std::ofstream f(dir/"0000004000_001.mesh",std::ios::binary);
    for(uint32_t n:{0x334e5056,1,0x60,4,1,1})write(f,n);
    write(f,std::array<int32_t,4>{algorithm,48,0,0});write(f,uint32_t(1));
    write(f,std::array<uint32_t,11>{0,textured,0,0,mode,0,0,0,1,0,1});write(f,uint16_t(4));
    const float xy[4][2]={{-.5f,-.5f},{.5f,-.5f},{-.5f,.5f},{.5f,.5f}};
    for(unsigned k=0;k<4;++k){write(f,std::array<float,4>{xy[k][0],xy[k][1],.5f,656});write(f,std::array<float,3>{0,0,1});
        write(f,std::array<float,2>{float(k&1),float(k>>1)});write(f,uint32_t(0x80402010));}
    if(algorithm==4)for(unsigned k=0;k<4;++k){write(f,int32_t(656));write(f,.25f);write(f,int32_t(660));write(f,.75f);}
    write(f,int32_t(0));write(f,uint32_t(0x200));for(unsigned k=0;k<4;++k){write(f,UINT32_MAX);write(f,uint32_t(0xc0+12*k));}
}
}
int main(int argc,char **argv){try{
    check(argc==2);const std::filesystem::path dir=argv[1];
    {   // Scene ownership: a level-of-detail owner (class 154dc0, +60 = 246f50) composes its levels in +87c..+884, and a
        // level of the same family its own; another class's +87c is not followed.
        Runtime rt;std::memset(rt.memory.flat(),0,bozo::kFlatBytes);
        constexpr uint32_t lod=0x154dc0,level=0x154ca0,other=0x150000,owner=0x400000,a=0x410000,b=0x420000,c=0x430000,stranger=0x440000;
        put(rt,lod+0x60,0x246f50u);put(rt,level+0x60,0x246f50u);put(rt,other+0x60,0x23c6c0u);
        put(rt,owner,lod);put(rt,owner+0x880,a);put(rt,a,level);put(rt,a+0x884,b);put(rt,b,level);put(rt,stranger,other);put(rt,stranger+0x87c,c);
        check(vp2s::composedBy(&rt,owner,owner) && vp2s::composedBy(&rt,owner,a) && vp2s::composedBy(&rt,owner,b));
        check(!vp2s::composedBy(&rt,owner,c) && !vp2s::composedBy(&rt,stranger,c) && !vp2s::composedBy(&rt,a,owner));
    }
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
        // A skinned mesh draws only once the game built its palette (skeleton +4c + 40, 2 bones; map 487): nothing before.
        if(algorithm==2)check(models.capture(&rt,assets,textures,scene,object).batches.empty());   // (one runtime address serves both passes)
        if(algorithm)vp2s::nativeBonePaletteWritten(&rt,0x1c0040);
        // The disc mesh's own VIF codes at +100 (the rest of its RAM is NOPs here): row 0 = (0, 8, 0, 0) (vertices at row
        // 8), STCYCL 4/1, one signed V2-16 unpack of the four UVs to rows 9, 13, 17, 21 (the pack's UVs).
        {const uint32_t codes[11]={0x6c010000u,0,8,0,0,0x01000104u,0x65040009u,0x00000000u,0x00001000u,0x10000000u,0x10001000u};
         for(unsigned k=0;k<11;++k)put(rt,model+0x60+0x100+4*k,codes[k]);
         const uint8_t *disc=rt.memory.hostPointer(model+0x60,0x200);const auto uvs=vp2s::nativeMeshUvSources(disc,0x200,4);
         for(unsigned k=0;k<4;++k)check(uvs[k].offset==0x11c+4*k && !uvs[k].unsignedLane);
         rejects([&]{vp2s::nativeMeshUvSources(disc,0x200,5);});}   // a fifth vertex has no UV unpack
        std::shared_ptr<const bozo::render::Texture> unused;
        for(unsigned seed=0;seed<128;++seed){
            const float alpha=float(seed)/128;put(rt,object+0x30c,alpha);put(rt,object+0x230+16*38,std::array<float,4>{0,0,.125f,.25f});
            const bool morph=seed&1;if(morph){const uint32_t aliases[5]={0,0x20000000u,0x30000000u,0x80000000u,0xa0000000u};
                put(rt,object+0x630,uint32_t(0x1d0000)|aliases[(seed/2)%5]);std::memcpy(rt.memory.hostPointer(0x1d0000,0x260),rt.memory.hostPointer(model,0x260),0x260);
                for(unsigned k=0;k<4;++k)put(rt,0x1d0060+0xc0+12*k,std::array<float,3>{(k&1)?.4f:-.4f,(k>>1)?.4f:-.4f,.5f});}
            else {const uint32_t nonModels[10]={0,0x3f7ffffeu,0x10003000u,0xffffffffu,0x01fffff0u,0x00100001u,0x80000000u,model,0x1e0000u,0x01ffff90u};
                put(rt,object+0x630,nonModels[(seed/2)%10]);put(rt,0x1e0034,uint32_t(2));}
            // Every other morph seed names the working copy's own mesh through the object's list (+77c, +7c2), as maps 264,
            // 270, 271 and 535 do: the disc mesh at the same offset, its vertices read from the copy.
            const bool listed=morph && seed%8==7;constexpr uint32_t list=0x1d8000,entry=0x1d8100;   // seed%8 == 7: clear of the seed%8 == 3 / 5 sections, which edit the disc block
            put(rt,object+0x77c,listed?list:0u);put(rt,object+0x7c2,uint8_t(listed?1:0));put(rt,list+12,entry);put(rt,entry,uint32_t(0x1d0060));put(rt,entry+4,model+0xa0);
            // Every other listed seed names the copy only through the second slot +634 (the copies are used in turn).
            put(rt,object+0x634,0u);if(listed && seed%16==15){put(rt,object+0x634,uint32_t(0x1d0000));put(rt,object+0x630,uint32_t(0x01ffff90u));}
            // The game animates the copy's UVs: (0.25 or 0.75, 0 or 0.5) here, read live.
            if(listed)for(unsigned k=0;k<4;++k)put(rt,0x1d0060+0x11c+4*k,uint32_t((k&1)?0x0c00u:0x0400u)|uint32_t((k>>1)?0x0800u:0u)<<16);
            if(listed){const auto saved=*reinterpret_cast<uint32_t *>(rt.memory.hostPointer(0x1d0060+0x10,4));put(rt,0x1d0060+0x10,saved^1u);
                rejects([&]{models.capture(&rt,assets,textures,scene,object);});put(rt,0x1d0060+0x10,saved);}
            const auto before=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
            const auto result=[&]{try{return models.capture(&rt,assets,textures,scene,object);}
                catch(const std::runtime_error &e){throw std::runtime_error(std::format("algorithm {} seed {}: {}",algorithm,seed,e.what()));}}();
            check(result.batches.size()==1);
            const auto &batch=result.batches[0];check(batch.vertices.size()==6 && !batch.texture && !batch.material.textured && batch.material.directColour && batch.material.depthWrite);
            check(batch.matrix[0]==100 && batch.matrix[5]==100 && batch.matrix[12]==2048 && batch.matrix[13]==2048);
            const unsigned indices[6]={0,1,2,1,3,2};const float extent=morph?.4f:.5f;
            for(unsigned j=0;j<6;++j){const auto k=indices[j];const auto &v=batch.vertices[j];
                const float x=(k&1?extent:-extent)+(algorithm==2?.1f:algorithm==4?.025f:0);
                const float y=(k>>1?extent:-extent)+(algorithm==4?.15f:0);
                check(std::fabs(v.x-x)<1e-6f && std::fabs(v.y-y)<1e-6f && v.z==.5f);
                if(listed)check(v.u==((k&1)?.75f:.25f)+.125f && v.v==((k>>1)?.5f:0.f)+.25f);
                else check(v.u==float(k&1)+.125f && v.v==float(k>>1)+.25f);
                check(v.rgba==((seed<<24)|0x402010));}
            check(!std::memcmp(before.data(),rt.memory.flat(),before.size()));
            put(rt,object+0x30c,0.f);check(batch.vertices[0].rgba==((seed<<24)|0x402010)); // immutable captured data
            // Light table A (mem[14].w = +31c) alone keeps the factor; its point (B, +314)
            // and spot (C, +318) tables reject in this fixture's mode 2 (mode 3 below).
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
                put(rt,object+0x31c,uint32_t(0));
                // Mode 0 keeps the object colour (row 13) with plain clear and no table A
                // (VU1 2b68: colour * mem[13], min mem[0]), with plain set, and with table A.
                if(algorithm==0){fixture(dir/"mode0",0,0);vp2s::NativeModels flat(dir/"mode0");
                    put(rt,object+0x300,std::array<float,4>{.5f,.25f,1,0});
                    for(uint32_t plainWord:{0u,1u})for(uint32_t tableWord:{0u,1u}){put(rt,object+0x310,plainWord);put(rt,object+0x31c,tableWord);
                        const auto lit=flat.capture(&rt,assets,textures,scene,object);check(lit.batches.size()==1 && lit.batches[0].vertices[0].rgba==0x400808u);}
                    put(rt,object+0x300,std::array<float,4>{1,1,1,0});put(rt,object+0x310,uint32_t(1));put(rt,object+0x31c,uint32_t(0));}
                // Mode 3's point lights (VU1 routine 0x3350): light 0 at (0,0,2), strength 4,
                // colour (.25,.5,.1); the eye at the origin; header 11 = (0,1,1,0), power 1.
                // Vertex 0 at (-x,-x,.5) (x = the morph's .4 on these odd seeds), normal +z:
                // dq = d*4/|d|^2, D = n.dq, h = dq + e/|e|, S = (n.h)^2; factor = colour * (D + S)
                // (directional weights all 0).
                if(algorithm==0){fixture(dir/"mode3",0,3);vp2s::NativeModels shiny(dir/"mode3");const uint32_t block=model+0x60+0x40;
                    put(rt,object+0x31c,uint32_t(1));put(rt,object+0x314,uint32_t(1));
                    put(rt,object+0x230+16*20,std::array<float,4>{0,0,2,1});put(rt,object+0x230+16*21,std::array<float,4>{9,0,0,1});put(rt,object+0x230+16*22,std::array<float,4>{0,9,0,1});
                    put(rt,object+0x230+16*23,std::array<float,4>{.25f,.5f,.1f,0});put(rt,object+0x230+16*27,std::array<float,4>{4,1,1,0});
                    put(rt,block+0x50,std::array<float,4>{0,1,1,0});put(rt,block+0x60,std::array<float,4>{0,0,0,0});put(rt,block+0x3c,int32_t(1));
                    const double x=extent,dq=4/(2*x*x+2.25),D=1.5*dq,inv=1/std::sqrt(2*x*x+.25),h=1.5*dq-.5*inv,S=h*h;
                    const auto lit=shiny.capture(&rt,assets,textures,scene,object);check(lit.batches.size()==1);const auto rgba=lit.batches[0].vertices[0].rgba;
                    const double want[3]={16*.25*(D+S),32*.5*(D+S),64*.1*(D+S)};
                    for(unsigned c=0;c<3;++c)check(std::fabs(double((rgba>>(8*c))&255)-std::trunc(std::min(want[c],255.0)))<=1);
                    put(rt,block+0x3c,int32_t(0));rejects([&]{shiny.capture(&rt,assets,textures,scene,object);});   // power 0 (the VU loop wraps)
                    for(unsigned r:{20u,21u,22u,23u,27u})put(rt,object+0x230+16*r,std::array<float,4>{0,0,0,0});
                    for(uint32_t at:{0x3cu})put(rt,block+at,uint32_t(0));put(rt,block+0x50,std::array<float,4>{});
                    put(rt,object+0x31c,uint32_t(0));put(rt,object+0x314,uint32_t(0));}
                // A mesh sampling 'L@tF' 1 (manager slot 0 -> entry 1, TEX0 512 x 512) takes the
                // current mode-1 screen-feedback image 5; without one, or another size, it rejects.
                if(algorithm==0){fixture(dir/"feedback",0,0,1);vp2s::NativeModels fed(dir/"feedback");constexpr uint32_t manager=0x1a8000;const uint32_t block=model+0x60+0x40;
                    put(rt,0x35de38,manager);put(rt,block+0x98,int32_t(0));put(rt,manager+0x2670,int16_t(1));put(rt,manager+0x1008+16,uint32_t(0x4674404c));put(rt,manager+8+16,uint32_t(1));
                    put(rt,manager+0x1010+16,uint32_t(0x64222ec0));put(rt,manager+0x1014+16,uint32_t(2));
                    rejects([&]{fed.capture(&rt,assets,textures,scene,object);});
                    vp2s::NativeFeedbackCopies copies;vp2s::NativeFeedbackCopy image{};image.frame=scene.frameTarget();image.mode=1;image.width=image.height=512;copies.published(image);
                    {const auto r=fed.capture(&rt,assets,textures,scene,object,&copies);check(r.batches.size()==1);const auto &mt=r.batches[0].material;
                        check(mt.fromFrame==1 && mt.source==5 && !mt.textured && !mt.directColour && !r.batches[0].texture);}
                    put(rt,manager+0x1010+16,uint32_t(0x60222ec0));rejects([&]{fed.capture(&rt,assets,textures,scene,object,&copies);});put(rt,manager+0x1010+16,uint32_t(0x64222ec0));   // TW 8
                    image.mode=0;copies.published(image);rejects([&]{fed.capture(&rt,assets,textures,scene,object,&copies);});
                    put(rt,0x35de38,uint32_t(0));put(rt,block+0x98,int32_t(0));for(uint32_t at:{0x2670u,0x1018u,0x18u,0x1020u,0x1024u})put(rt,manager+at,uint32_t(0));}}
            put(rt,model+0x59,uint8_t(1));put(rt,model+0x96,uint16_t(1));put(rt,object+0x7a4,uint16_t(1));
            check(models.capture(&rt,assets,textures,scene,object).batches.empty());put(rt,object+0x7a4,uint16_t(0));
            put(rt,current+0x40dc,uint8_t(0));rejects([&]{models.capture(&rt,assets,textures,scene,object);});put(rt,current+0x40dc,uint8_t(1));
            ++cases;
        }
        put(rt,object+0x7c2,uint8_t(0));put(rt,object+0x77c,0u);put(rt,object+0x634,0u);
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
            // Shadow classes (slot +60 = 2b86f0): their own model is a shadow volume
            // that writes only frame alpha, so it submits no colour batches.
            // (Its faces: nativeShadowPrisms, checked below; here an empty FPS model.)
            for(uint32_t cls:{0x159880u,0x1599a0u}){
                put(rt,object,cls);put(rt,cls+0x60,uint32_t(0x2b86f0));
                {const auto r=models.capture(&rt,assets,textures,scene,object);check(r.object==object && r.camera==camera && r.batches.empty() && r.shadow && !r.deferUntil &&
                    r.shadowIncrement.empty() && r.shadowDecrement.empty());++emptyCases;}
                put(rt,cls+0x60,uint32_t(0x23c6c0));rejects([&]{models.capture(&rt,assets,textures,scene,object);});++emptyCases;
                put(rt,cls+0x60,uint32_t(0));put(rt,object,uint32_t(0));}
            put(rt,object,uint32_t(0x1546d0));put(rt,0x1546d0+0x60,uint32_t(0x23c6c0));
            {const auto r=models.capture(&rt,assets,textures,scene,object);check(!r.shadow && !r.deferUntil);++emptyCases;}
            put(rt,0x1546d0+0x60,uint32_t(0));put(rt,object,uint32_t(0));
            // A model composed by a shadow-class owner waits for the owner's shadow pass.
            {constexpr uint32_t owner=0x1e4000;std::memset(rt.memory.hostPointer(owner,0x900),0,0x900);put(rt,owner,uint32_t(0x1599a0));put(rt,owner+0x8d4,owner+0x8cc);
                put(rt,owner+0x8c4,object);put(rt,current+0x40a8,owner);
                check(models.capture(&rt,assets,textures,scene,object).deferUntil==owner);++emptyCases;
                put(rt,owner,uint32_t(0x153700));rejects([&]{models.capture(&rt,assets,textures,scene,object);});   // not a composite
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
    // Routine 26 against its own microcode, lifted and run as a test-only oracle
    // (tests/bozo_reflection_oracle.inc): S, T, Q within 2/4096, alpha exactly.
    unsigned reflectionCases=0;
    {struct Case{std::array<float,3> n,p,eye;float c,s;std::array<int32_t,4> out;int32_t on;};
     static const Case cases[]={
#include "bozo_reflection_oracle.inc"
     };
     for(const auto &k:cases){
        const float R[3][3]={{k.c,0,k.s},{0,1,0},{-k.s,0,k.c}};float rot[3][3];
        for(unsigned j=0;j<3;++j)for(unsigned c=0;c<3;++c)rot[j][c]=R[c][j];
        const auto got=vp2s::reflectionTexture(k.n,k.p,k.eye,rot,40,k.on,60);
        for(unsigned c=0;c<3;++c)if(std::abs(got.stq[c]-k.out[c])>2){std::fprintf(stderr,"routine 26 case%u lane%u %d / %d\n",reflectionCases,c,got.stq[c],k.out[c]);check(false);}
        if(k.on){if(!got.alpha || *got.alpha!=k.out[3]){std::fprintf(stderr,"routine 26 case%u alpha %d / %d\n",reflectionCases,got.alpha?*got.alpha:-999,k.out[3]);check(false);}}
        else check(!got.alpha && k.out[3]==-9);
        ++reflectionCases;}
     check(reflectionCases==40);}
    std::printf("%u VU1 routine-26 reflection vertices match its lifted microcode\n",reflectionCases);
    // Routine 40 against its own microcode, lifted and run as a test-only oracle
    // (tests/bozo_screen_projection_oracle.inc): S, T within 2/4096, Q 4096, w untouched.
    unsigned screenCases=0;
    {struct Case{std::array<float,3> n,p,data;std::array<float,2> scale,row38,row39;std::array<float,16> rows;std::array<int32_t,4> out;};
     static const Case cases[]={
#include "bozo_screen_projection_oracle.inc"
     };
     for(const auto &k:cases){
        // The oracle's rows 85..88 are the guard matrix's columns: c = r85 x + r86 y + r87 z + r88.
        std::array<float,16> guard{};for(unsigned r=0;r<4;++r)for(unsigned c=0;c<4;++c)guard[4*r+c]=k.rows[4*c+r];
        vp2s::ScreenProjection data;data.offset[0]=k.data[0];data.offset[1]=k.data[1];data.push=k.data[2];data.scale[0]=k.scale[0];data.scale[1]=k.scale[1];
        const auto got=vp2s::screenTexture(k.n,k.p,data,k.row38,k.row39,guard);
        for(unsigned c=0;c<2;++c)if(std::abs(got[c]-k.out[c])>2){std::fprintf(stderr,"routine 40 case%u lane%u %d / %d\n",screenCases,c,got[c],k.out[c]);check(false);}
        check(k.out[2]==4096 && k.out[3]==-5);
        ++screenCases;}
     check(screenCases==48);}
    std::printf("%u VU1 routine-40 screen-projected vertices match its lifted microcode\n",screenCases);
    // Routine 32/34's UV shift against its own microcode, lifted and run as a test-only oracle (tests/bozo_emboss_oracle.inc):
    // every UV within 1/4096, directional and point lights, strips with repeated vertices (degenerate triangles).
    unsigned embossCases=0,embossVertices=0;
    {struct Case{unsigned point;float scale;std::array<float,3> light;std::array<uint16_t,3> counts;std::vector<std::array<float,3>> p;std::vector<std::array<int32_t,2>> in,out;};
     static const Case cases[]={
#include "bozo_emboss_oracle.inc"
     };
     for(const auto &k:cases){
        std::vector<uint16_t> strips;for(auto n:k.counts)if(n)strips.push_back(n);
        auto uv=k.in;vp2s::nativeEmbossStrips(uv,k.p,strips,k.light,k.point!=0,k.scale);
        for(size_t v=0;v<uv.size();++v)for(unsigned c=0;c<2;++c)if(std::abs(uv[v][c]-k.out[v][c])>1){
            std::fprintf(stderr,"routine 34 case%u vertex%zu lane%u %d / %d\n",embossCases,v,c,uv[v][c],k.out[v][c]);check(false);}
        embossVertices+=unsigned(uv.size());++embossCases;}
     check(embossCases==48);
     // A strip shorter than a triangle, or one past the mesh, is refused.
     {std::vector<std::array<int32_t,2>> uv(4);std::vector<std::array<float,3>> p(4);bool refused=false;
      try{vp2s::nativeEmbossStrips(uv,p,{2,2},{0,0,1},false,1);}catch(const std::runtime_error &){refused=true;}check(refused);refused=false;
      try{vp2s::nativeEmbossStrips(uv,p,{5},{0,0,1},false,1);}catch(const std::runtime_error &){refused=true;}check(refused);}}
    std::printf("%u VU1 routine-34 UV-shift strips (%u vertices) match its lifted microcode\n",embossCases,embossVertices);
    // Shadow program 0x6f0 faces (nativeShadowPrisms). top: identity; flat: y -> 0, x += y/2.
    unsigned shadowCases=0;
    {using P=std::array<float,3>;using V=bozo::render::Vertex;
        std::array<float,16> top{},flat{};top[0]=top[5]=top[10]=top[15]=1;flat[0]=flat[10]=flat[15]=1;flat[4]=.5f;
        const auto area=[](const std::vector<V> &t){double s=0;for(size_t i=0;i+2<t.size();i+=3)s+=(t[i+1].x-t[i].x)*(t[i+2].y-t[i].y)-(t[i+1].y-t[i].y)*(t[i+2].x-t[i].x);return s;};
        const auto has=[](const std::vector<V> &t,const P &a,const P &b,const P &c){
            for(size_t i=0;i+2<t.size();i+=3){unsigned hit=0;for(const P *p:{&a,&b,&c})for(size_t j=0;j<3;++j)if(t[i+j].x==(*p)[0] && t[i+j].y==(*p)[1] && t[i+j].z==(*p)[2]){++hit;break;}
                if(hit==3)return true;}return false;};
        // A planar strip of 4 (two triangles, the second swapped back to the same winding), normal (0,0,1) in the plane z=2.
        const std::vector<P> quad{{0,4,2},{4,4,2},{0,8,2},{4,8,2}};const std::vector<uint16_t> strip{4};
        for(int sign:{1,-1}){std::vector<V> inc,dec;vp2s::nativeShadowPrisms(quad,strip,{0,0,float(sign)},top,flat,inc,dec);
            // (b-a)x(c-a) of (0,4)(4,4)(0,8) is +z: the light along +z keeps both triangles, along -z none.
            if(sign>0){check(!inc.empty() && !dec.empty() && inc.size()%3==0 && dec.size()%3==0);
                // Each prism is closed: its screen faces' signed areas cancel, increments against decrements.
                check(std::fabs(area(inc)+area(dec))<1e-3 && area(inc)>1 && area(dec)<-1);
                // The first cap (a, b, c) and its flattened copy (a', b', c') are faces; flattening y -> 0, x += y/2.
                check(has(inc,{0,4,2},{4,4,2},{0,8,2}) || has(dec,{0,4,2},{4,4,2},{0,8,2}));
                for(const auto *list:{&inc,&dec})for(const auto &v:*list)check(v.rgba==0 && v.z==2);}
            else check(inc.empty() && dec.empty());
            ++shadowCases;}
        // Without the odd-triangle swap the second triangle would face away: both prisms are drawn, 2 x (up to 8) faces.
        {std::vector<V> inc,dec;vp2s::nativeShadowPrisms(quad,strip,{0,0,1},top,flat,inc,dec);
            std::vector<V> one,two;vp2s::nativeShadowPrisms({quad[0],quad[1],quad[2]},std::vector<uint16_t>{3},{0,0,1},top,flat,one,two);
            check(inc.size()+dec.size()>one.size()+two.size());++shadowCases;}
        // A point light (mem[14].w): the direction is a - light per triangle. Any light with z < 2 gives a - P a
        // positive z, so both triangles are kept as with the directional (0,0,1), whatever its x, y; one with z > 2
        // culls both.
        {std::vector<V> inc,dec,ref,refDec;vp2s::nativeShadowPrisms(quad,strip,{0,0,1},top,flat,ref,refDec);
            for(const P &light:{P{0,0,0},P{100,-50,-3},P{-7,9,1.99f}}){inc.clear();dec.clear();vp2s::nativeShadowPrisms(quad,strip,light,top,flat,inc,dec,true);
                check(inc.size()==ref.size() && dec.size()==refDec.size());++shadowCases;}
            for(const P &light:{P{0,0,5},P{30,30,2.01f}}){inc.clear();dec.clear();vp2s::nativeShadowPrisms(quad,strip,light,top,flat,inc,dec,true);
                check(inc.empty() && dec.empty());++shadowCases;}
            // Directional semantics are unchanged by the flag's default: (0,0,5) as a direction keeps both.
            inc.clear();dec.clear();vp2s::nativeShadowPrisms(quad,strip,{0,0,5},top,flat,inc,dec);check(inc.size()==ref.size());++shadowCases;}
        // Mesh algorithm 16's second pass, from map 107's mesh rows 7..22 (EE RAM, object d40690): ALPHA_1 0x48, slot 0,
        // part 0, TEX0 0x01400000; map 124's block 148eb70 samples slot 1 (TBP 3a40).
        {vp2s::NativeMaterialRows rows{{{0x8004,0x10000000,0xeeeeeeee,0},{0,0,0x42,0},{0x01400000,6,6,0},{0x01400000,6,6,0xa35},
            {0x3f800000,0,0,1},{0x3f800000,0,0,0},{0x8000,0x300e4000,0x412,0},{0x8000,0x300ec000,0x412,0},{0,1,0,0},
            {0x48,0,0x42,1},{0x01400000,6,6,0},{0x01400000,6,6,0},{0x8000,0x302e4000,0x412,0},{0x8000,0x302ec000,0x412,0},{1,1,0,0},{0,0,0,0x6202c016}}};
         const auto pass=vp2s::nativeSecondPass(rows);check(pass && pass->alpha==0x48 && pass->slot==0 && pass->part==0 && pass->tex0==0x01400000u && pass->on);++shadowCases;
         {auto off=rows;off[14][1]=0;const auto p0=vp2s::nativeSecondPass(off);check(p0 && !p0->on);++shadowCases;}   // row 21.y clear: no pass
         auto other=rows;
         {auto glow=rows;glow[2]={0x21300000,0x4007b406,6,0};glow[3]={0x65423380,0x0007b406,6,0xa35};glow[10]={0x21300000,0x4007b786,6,0};glow[11]={0x21413a40,0x0007b786,6,0};
          glow[14]={1,1,1,0};glow[9][0]=0x44;const auto p2=vp2s::nativeSecondPass(glow);check(p2 && p2->alpha==0x44 && p2->slot==1 && p2->part==0 && p2->tex0==0x21413a40u);++shadowCases;}
         other=rows;other[12][1]=0x30ae4000;check(!vp2s::nativeSecondPass(other));             // PRIM differs beyond ABE
         other=rows;other[9][3]=0;check(vp2s::nativeSecondPass(other).has_value());            // row 65 selected (map 124)
         other=rows;other[9][3]=2;check(!vp2s::nativeSecondPass(other));                       // .w neither 0 nor 1
         other=rows;other[9][0]=0xc8;check(!vp2s::nativeSecondPass(other));                    // a reserved blend field (3)
         other=rows;other[11][2]=0x14;check(!vp2s::nativeSecondPass(other));                   // row 18 not a TEX0_1 write
         other=rows;other[6][1]=0x302e4000;other[12][1]=0x302e4000;check(vp2s::nativeSecondPass(other).has_value());   // pass 1 already blended (map 65)
         other[12][1]=0x30ae4000;check(!vp2s::nativeSecondPass(other));shadowCases+=6;   // ... and pass 2 changing another PRIM bit
         // Map 109 (object 10e3170): pass 2 is exactly pass 1's tags, both without ABE (PRIM 3c / 3d), ALPHA 0, its own TEX0:
         // an opaque redraw. Only one of the two tags matching that way is still refused.
         other=rows;other[6]={0x8000,0x301e4000,0x412,0};other[7]={0x8000,0x301ec000,0x412,0};other[12]=other[6];other[13]=other[7];other[9][0]=0;
         other[10]={0x21300000,0x4007b786,6,0};other[11]={0x213137e0,0x0007b786,6,0};other[14]={1,1,4,0};
         {const auto p3=vp2s::nativeSecondPass(other);check(p3 && !p3->blended && p3->tex0==0x213137e0u && p3->slot==4 && p3->part==0 && p3->on && pass->blended);}
         other[13][1]=0x302ec000;check(!vp2s::nativeSecondPass(other));shadowCases+=2;}
        // Routine 46's projection, Logun's projector rows (object d23280): S = trunc(4096 (-1.7 + -0.0028573 x)), Q = 1;
        // the mask 0x20 turns a vertex facing away from the origin (dot((origin - p), n) < 0) off, and q.z < 0 too.
        {const std::array<float,4> cols[4]{{std::bit_cast<float>(0xbb3b3ee7u),0,0,0},{0,std::bit_cast<float>(0x3aced65cu),0,std::bit_cast<float>(0x3f55691eu)},
             {0,std::bit_cast<float>(0xbb1c1839u),0,std::bit_cast<float>(0x3f0d6489u)},{-1.7f,-1.25f,1,0}};
         const std::array<float,4> origin{-1500,-2867,-2683,1};
         const auto t=vp2s::projectedTexture({100,-40,200},{0,-1,0},origin,cols,0x20);
         const float s=((-1.7f*1.0f+cols[0][0]*100.0f)+cols[1][0]*-40.0f)+cols[2][0]*200.0f;
         check(t.stq[0]==int32_t(std::trunc(double(s)*4096)) && t.stq[2]==4096 && !t.off);++shadowCases;          // facing up to the origin above
         check(vp2s::projectedTexture({100,-40,200},{0,1,0},origin,cols,0x20).off);++shadowCases;                  // facing away: off
         check(!vp2s::projectedTexture({100,-40,200},{0,1,0},origin,cols,0).off);++shadowCases;                    // ... unless the mask is 0
         std::array<float,4> behind[4]{cols[0],cols[1],cols[2],cols[3]};behind[3][2]=-1;check(vp2s::projectedTexture({100,-40,200},{0,-1,0},origin,behind,0).off);++shadowCases;}   // q.z < 0
        // Shell redraws (table B 12 / 14). 12, counter 3, push 1, alpha step -8.5: dispatch 1 steps twice (12, then 14's part:
        // the counter 3 -> 1), dispatch 2 once (-> 0): two redraws, pushed 2 and 3, alpha 120 -> 111 -> 102, then 93.
        {const auto f=[](float x){return std::bit_cast<uint32_t>(x);};
         vp2s::NativeShellVertex v;v.colour={10,20,30,120};v.p={1,2,3};v.n={0,1,0};v.uv={0.5f,0.25f};
         const auto s12=vp2s::nativeShells(12,{f(1),3,0,0,0,0,0,f(-8.5f)},{v});
         check(s12.size()==2 && s12[0][0].p[1]==4 && s12[1][0].p[1]==5 && s12[0][0].colour[3]==102 && s12[1][0].colour[3]==93 && s12[1][0].colour[0]==10 && s12[1][0].uv[0]==0.5f);++shadowCases;
         // Colours clamp to 0..255; a zero counter draws nothing.
         const auto bright=vp2s::nativeShells(14,{0,1,0,0,f(300),f(-300),0,0},{v});check(bright.size()==1 && bright[0][0].colour[0]==255 && bright[0][0].colour[1]==0);++shadowCases;
         check(vp2s::nativeShells(14,{f(1),0,0,0,0,0,0,0},{v}).empty());++shadowCases;
         // 14's jitter: RINIT from the seed, then two RNEXTs a vertex: u += (R1 - 1.5) * scale, v += (R2 - 1.5) * scale (12-bit).
         const uint32_t seed=0x2bc22;const auto next=[](uint32_t r){return (((r<<1)^((r>>4)&1u)^((r>>22)&1u))&0x7fffffu)|0x3f800000u;};
         const uint32_t r1=next(0x3f800000u|seed),r2=next(r1);const float scale=0.005f;
         const auto j=vp2s::nativeShells(14,{0,1,f(scale),seed,0,0,0,0},{v});
         const auto expect=[&](float uv,uint32_t r){return float(std::trunc((double(int32_t(std::trunc(double(uv)*4096)))/4096+(std::bit_cast<float>(r)-1.5f)*scale)*4096))/4096;};
         check(j.size()==1 && j[0][0].uv[0]==expect(0.5f,r1) && j[0][0].uv[1]==expect(0.25f,r2) && j[0][0].uv[0]!=0.5f);++shadowCases;}
        // A vertex at w <= 0 drops its triangle; strips that do not cover the vertices reject.
        {auto behind=top;behind[15]=-1;std::vector<V> inc,dec;vp2s::nativeShadowPrisms(quad,strip,{0,0,1},behind,flat,inc,dec);check(inc.empty() && dec.empty());
            bool bad=false;try{vp2s::nativeShadowPrisms(quad,std::vector<uint16_t>{3},{0,0,1},top,flat,inc,dec);}catch(const std::runtime_error &){bad=true;}check(bad);shadowCases+=2;}
        // The pass: stencil cleared by the first counting draw, depth GREATER, no colour or depth writes;
        // then the darkening quad (black, alpha 0x3f, (Cs - Cd) As + Cd) where the count is not 0.
        {vp2s::NativeModelSubmission s;s.shadow=true;check(vp2s::nativeShadowPass(s,640,448).empty());
            vp2s::nativeShadowPrisms(quad,strip,{0,0,1},top,flat,s.shadowIncrement,s.shadowDecrement);
            const auto d=vp2s::nativeShadowPass(s,640,448);check(d.size()==3);
            using Op=bozo::render::Material::StencilOp;
            check(d[0].material.stencilClear && d[0].material.stencilOp==Op::IncrementWrap && d[0].vertices.size()==s.shadowIncrement.size());
            check(!d[1].material.stencilClear && d[1].material.stencilOp==Op::DecrementWrap && d[1].vertices.size()==s.shadowDecrement.size());
            for(unsigned i=0;i<2;++i)check(d[i].material.colourMask==0 && d[i].material.depthTest && d[i].material.depthGreater && !d[i].material.depthWrite && !d[i].material.textured);
            const auto &m=d[2].material;check(m.stencilNotZero && m.stencilOp==Op::None && !m.depthTest && !m.depthWrite && m.gsBlend && m.blendA==0 && m.blendB==1 && m.blendC==0 && m.blendD==1);
            check(d[2].vertices.size()==6);for(const auto &v:d[2].vertices)check(v.rgba==0x3f000000u && (v.x==2048-320 || v.x==2048+320) && (v.y==2048-224 || v.y==2048+224));
            s.shadowIncrement.clear();const auto only=vp2s::nativeShadowPass(s,640,448);check(only.size()==2 && only[0].material.stencilClear && only[0].material.stencilOp==Op::DecrementWrap);
            s.shadow=false;check(vp2s::nativeShadowPass(s,640,448).empty());shadowCases+=4;}
    }
    std::printf("%u shadow volume cases (program 0x6f0 faces, the counting/darkening pass)\n",shadowCases);
    std::printf("%u VU1 routine-20 sphere-map vertices match the listing; %u native model captures +%u qualified empty FPS models: independent positions/skinning/UV/colour/strip ordering, unused-input isolation, whole RAM/FP preservation and snapshot/asset lifetime\n",sphereCases,cases,emptyCases);
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

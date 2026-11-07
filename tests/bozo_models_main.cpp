#include "native_models.h"
#include <fstream>
#include <cstring>
#include <source_location>
#include <cstdio>
#include <cmath>
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
    unsigned cases=0;
    for(int algorithm:{0,2,4}){
        fixture(dir,algorithm);vp2s::NativeModels models(dir);Runtime rt;vp2s::NativeScene scene;vp2s::NativeAssets assets;vp2s::NativeTextures textures;
        std::memset(rt.memory.flat(),0,bozo::kFlatBytes);
        put(rt,0x35ddf8,current);put(rt,current+0x40a8,object);put(rt,current+0x40dc,uint8_t(1));
        put(rt,target+0x3c76,uint8_t(1));put(rt,target+0x3c60,1.f);put(rt,target+0x3c64,1.f);
        put(rt,target+0x3c68,int16_t(640));put(rt,target+0x3c6a,int16_t(448));
        for(unsigned i=0;i<2;++i){const auto base=target+0xa70*i;put(rt,base+0x1cf0,uint8_t(1));put(rt,base+0x1cf2,uint8_t(i));put(rt,base+0x1cf5,uint8_t(1));
            put(rt,base+0x1cfa,int16_t(640));put(rt,base+0x1cfc,int16_t(224));}
        scene.beginComposition(&rt,0x1f0000,target);
        put(rt,camera+0x220,uint32_t(0x153120));put(rt,camera+0x8e0,uint32_t(0x1a0000));put(rt,0x1a0000,uint32_t(0x153140));
        put(rt,camera+0x2f0,identity);put(rt,camera+0x370,identity);auto screen=identity;screen[0]=100;screen[5]=100;screen[3]=2048;screen[7]=2048;
        put(rt,camera+0x330,screen);put(rt,camera+0x244,2048.f);put(rt,camera+0x248,2048.f);put(rt,camera+0x260,1728.f);put(rt,camera+0x264,1824.f);
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
            const bool morph=seed&1;if(morph){put(rt,object+0x630,uint32_t(0x1d0000));std::memcpy(rt.memory.hostPointer(0x1d0000,0x260),rt.memory.hostPointer(model,0x260),0x260);
                for(unsigned k=0;k<4;++k)put(rt,0x1d0060+0xc0+12*k,std::array<float,3>{(k&1)?.4f:-.4f,(k>>1)?.4f:-.4f,.5f});}
            else put(rt,object+0x630,uint32_t(0));
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
            put(rt,model+0x59,uint8_t(1));put(rt,model+0x96,uint16_t(1));put(rt,object+0x7a4,uint16_t(1));
            check(models.capture(&rt,assets,textures,scene,object).batches.empty());put(rt,object+0x7a4,uint16_t(0));
            put(rt,current+0x40dc,uint8_t(0));rejects([&]{models.capture(&rt,assets,textures,scene,object);});put(rt,current+0x40dc,uint8_t(1));
            ++cases;
        }
        assets.overwrite(model,0x1000);rejects([&]{models.capture(&rt,assets,textures,scene,object);});
    }
    std::printf("%u native model captures: independent positions/skinning/UV/colour/strip ordering, whole RAM preservation and snapshot/asset lifetime\n",cases);
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

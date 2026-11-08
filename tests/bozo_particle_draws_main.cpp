#include "native_particle_draws.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "pdraw_original_functions.h"
#include "pdraw_adapted_functions.h"
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <format>
#include <immintrin.h>
#include <source_location>
#ifdef VP2S_PARTICLE_GPU
#include "bozo/render/renderer.h"
#include "bozo/render/gl.h"
#include <SDL3/SDL.h>
#endif
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("particle draw assertion at "+std::to_string(at.line()));}
template<class T>void put(Runtime &rt,uint32_t at,const T &v){auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(p,&v,sizeof(v));}
template<class T>void write(std::ofstream &out,const T &v){out.write(reinterpret_cast<const char *>(&v),sizeof(v));}
constexpr uint32_t object=0x160000,camera=0x170000,target=0x180000,current=0x190000,kinds=0x1e1020,fis=0x1d1000,manager=0x1b0000;
using Mat=std::array<float,16>;
const Mat identity{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
void files(const std::filesystem::path &dir){
    std::filesystem::create_directories(dir);
    {std::ofstream f(dir/"0000004000_001.pmat",std::ios::binary);write(f,uint32_t(0x31505056));write(f,uint32_t(1));
        const std::array<uint32_t,27> fields{0x20,1,0x110,1,0,1,0,0,1,0,1,0,2,2,0,1,0,2,2,0,1,0,2,2,0,1,0};write(f,fields);}
    {std::ofstream f(dir/"manifest.txt");f<<"VPNT1\n11223344 5 c_0000000000000000.tex\n";}
    {std::ofstream f(dir/"c_0000000000000000.tex",std::ios::binary);write(f,std::array<uint32_t,3>{2,2,0});write(f,std::array<uint32_t,4>{0x800000ff,0x8000ff00,0x80ff0000,0x80ffffff});write(f,std::array<uint32_t,256>{});}
}
void initialize(Runtime &rt,vp2s::NativeScene &scene,vp2s::NativeAssets &assets,unsigned seed){
    std::memset(rt.memory.flat(),0,bozo::kFlatBytes);scene=vp2s::NativeScene{};assets=vp2s::NativeAssets{};
    put(rt,0x1c8d00+0xcc84,uint8_t(1));put(rt,0x35ddf8,current);put(rt,current+0x40a8+4*(seed&1),object);put(rt,current+0x40dc+(seed&1),uint8_t(1));
    put(rt,target+0x3c76,uint8_t(1));put(rt,target+0x3c60,1.f);put(rt,target+0x3c64,1.f);put(rt,target+0x3c68,int16_t(640));put(rt,target+0x3c6a,int16_t(448));
    for(unsigned i=0;i<2;++i){const auto base=target+0xa70*i;put(rt,base+0x1cf0,uint8_t(1));put(rt,base+0x1cf2,uint8_t(i));put(rt,base+0x1cf5,uint8_t(1));put(rt,base+0x1cfa,int16_t(640));put(rt,base+0x1cfc,int16_t(224));}
    scene.beginComposition(&rt,0x1f8000,target);put(rt,camera+0x220,uint32_t(0x153120));put(rt,camera+0x8e0,uint32_t(0x1a0000));put(rt,0x1a0000,uint32_t(0x153140));
    put(rt,camera+0x2f0,identity);put(rt,camera+0x370,identity);auto screen=identity;screen[0]=100;screen[5]=100;screen[3]=screen[7]=2048;
    screen[11]=std::bit_cast<float>(0x4d200821u);put(rt,camera+0x330,screen);put(rt,camera+0x244,2048.f);put(rt,camera+0x248,2048.f);put(rt,camera+0x260,1728.f);put(rt,camera+0x264,1824.f);scene.selectCamera(&rt,camera);
    put(rt,object,uint32_t(0x154180));put(rt,object+0x63,uint8_t(1));put(rt,object+0x65,uint8_t(seed&1));put(rt,object+0x90,identity);put(rt,object+0xaff,uint8_t(3));put(rt,object+0xaf9,uint8_t((seed&1)^1));
    const uint32_t aliases[]={0,0x20000000,0x30000000,0x80000000,0xa0000000};put(rt,object+0xaa0,uint32_t(0x200000)|aliases[(seed/8)%5]);put(rt,object+0xaa4,uint32_t(0x210000)|aliases[(seed/8)%5]);
    const unsigned count=seed%5;put(rt,object+0xabc,uint32_t(count));put(rt,object+0xac0+4*(seed&1),uint32_t(count?1:0));put(rt,object+0x628,kinds);put(rt,object+0x658,fis);
    put(rt,fis+4,uint32_t(0x11223344));put(rt,fis+8,uint32_t(5));put(rt,0x35de38,manager);put(rt,manager+0x2670,int16_t(1));put(rt,manager+0x1008+16,uint32_t(0x11223344));put(rt,manager+8+16,uint32_t(5));
    put(rt,object+0x230,std::array<float,4>{255,255,255,128});put(rt,object+0x300,std::array<float,4>{.5f,1,.25f,.5f});put(rt,object+0x310,uint32_t(seed&4?1:0));
    if(seed&4){std::array<uint32_t,4> opaque{0x7fa01234,0x7f800000,1,0xffffffff};put(rt,object+0x230,opaque);put(rt,object+0x300,opaque);}
    // These words resemble transport/material data intentionally. Runtime
    // drawing must ignore all of them in favour of the offline semantic pack.
    std::memset(rt.memory.hostPointer(kinds,0x110),0xcd,0x110);put(rt,kinds,uint8_t(1));put(rt,kinds+1,uint8_t(1));put(rt,kinds+2,uint8_t(0));
    const std::array<float,20> shape{-.25f,-.25f,0,1,.25f,.25f,0,1,0,0,1,1,1,1,1,1,1,.5f,1,1};put(rt,kinds+0xc0,shape);
    assets.discRead(0x140000,1,8);assets.begin(0x1f0000,0x140000,kinds-0x20,1);assets.finish(0x1f0000,true,0x130);
    const auto chunk=(seed&1?0x210000u:0x200000u)+0x20;std::memset(rt.memory.hostPointer(chunk,32+48*count),0xa5,32+48*count);
    put(rt,chunk+0x10,uint32_t(count));put(rt,chunk+0x14,uint32_t(count*3+2));put(rt,chunk+0x18,uint32_t(1));
    for(unsigned i=0;i<count;++i){const auto at=chunk+32+48*i;put(rt,at,std::array<float,4>{float(i)/20,0,.5f,std::bit_cast<float>(0x7fa01234u)});
        put(rt,at+16,std::array<float,4>{16,32,64,128});put(rt,at+32,std::array<float,3>{seed&2?1.5707963267948966f:0.f,.5f,1});put(rt,at+44,uint32_t(0));}
}
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::ParticleDrawOwner,vp2s::ParticleOwner,vp2s::ModelOwner,vp2s::SceneOwner,vp2s::AssetOwner,vp2s::TextureOwner{
    vp2s::NativeScene scene;vp2s::NativeAssets assets;vp2s::NativeParticles particles;
    vp2s::NativeParticleDraws draws;vp2s::NativeTextures textures;vp2s::NativeModels models;
    std::vector<vp2s::NativeModelSubmission> submitted;unsigned callbacks=0,enqueues=0;bool reject=false;
    explicit Host(const std::filesystem::path &dir):draws(dir),textures(dir,dir/"manifest.txt"),models(dir){}
    vp2s::NativeParticleDraws &nativeParticleDraws()override{return draws;}
    vp2s::NativeParticles &nativeParticles()override{return particles;}
    vp2s::NativeScene &nativeScene()override{return scene;}
    vp2s::NativeAssets &nativeAssets()override{return assets;}
    vp2s::NativeTextures &nativeTextures()override{return textures;}
    vp2s::NativeModels &nativeModels()override{return models;}
    void drawNativeModel(const vp2s::NativeModelSubmission &s)override{if(reject)throw std::runtime_error("renderer rejection");submitted.push_back(s);}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(auto answer=vp2s::nativeParticleDrawInstruction(c,rt,pc))return *answer;
        if(vp2s::nativeSceneInstruction(r,c,rt,pc))return true;
        throw std::runtime_error("unexpected particle fixture native site");
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        if(pc==0x201f20){
            // A bounded software queue contract, not a hardware worker: consume
            // the original object argument and run its actual compiled callback.
            check(c->r[4].uw[0]==0x1d0000 && c->r[5].uw[0]==object+0x5b0);++enqueues;
            c->r[4].sd[0]=object;d.runtime().functions.generated(0x22f120)(d.runtime().memory.flat(),c,&d.runtime());return;
        }
        check(pc==0x1130f0 && c->r[4].uw[0]==0x1d2000);++callbacks;
        ++*d.runtime().memory.hostPointer(0x1d2000,1);c->r[2].ud[0]=0x1234;c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device write");}
    bool cop0Condition(bozo::ee::Dispatcher &,R5900Context *)override{throw std::runtime_error("device completion");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
void completionCases(const std::filesystem::path &dir){
    Host oldHost(dir),nativeHost(dir);bozo::ee::Dispatcher old(oldHost),native(nativeHost);
    old.setFunctions(pdraw_original_function_table,pdraw_original_function_count,pdraw_original_resume_table,pdraw_original_resume_count);
    native.setFunctions(pdraw_adapted_function_table,pdraw_adapted_function_count,pdraw_adapted_resume_table,pdraw_adapted_resume_count);
    old.runtime().port=&oldHost;native.runtime().port=&nativeHost;
    old.boot(0x22f140,0x1fff000);native.boot(0x22f140,0x1fff000);old.bindLibrary(0x201f20);old.bindLibrary(0x1130f0);native.bindLibrary(0x1130f0);
    const auto saved=_mm_getcsr();
    for(unsigned seed=0;seed<128;++seed){
        for(auto *d:{&old,&native}){auto &h=d==&old?oldHost:nativeHost;initialize(d->runtime(),h.scene,h.assets,seed);
            put(d->runtime(),0x35dde0,uint32_t(0x1d0000));put(d->runtime(),0x35dd78,uint32_t(0x1d2000));put(d->runtime(),current+0x4088,uint32_t(0x1d3000));
            put(d->runtime(),object+0x64,uint8_t(1));h.submitted.clear();h.callbacks=0;}
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x1234000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        x.r[4].ud[0]=object;x.r[29].ud[0]=0x1fff000;x.r[31].ud[0]=0x180000;
        std::memset(x.vu0_vf,0xa5,sizeof(x.vu0_vf));x.vu0_status=0x12345678;auto y=x;
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|((seed%4)<<13)|(seed&8?0x8040:0);_mm_setcsr(control);
        old.runtime().functions.generated(0x22f140)(old.runtime().memory.flat(),&x,&old.runtime());check(_mm_getcsr()==control);
        native.runtime().functions.generated(0x22f140)(native.runtime().memory.flat(),&y,&native.runtime());check(_mm_getcsr()==control);_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;check(x.pc==0x180000 && !std::memcmp(&x,&y,sizeof(x)));
        check(!std::memcmp(old.runtime().memory.flat(),native.runtime().memory.flat(),bozo::kFlatBytes));
        check(nativeHost.submitted.size()==1 && nativeHost.submitted[0].object==object && nativeHost.callbacks==oldHost.callbacks && nativeHost.callbacks==(seed&1?0:1));
        const auto *ram=native.runtime().memory.flat();check(!ram[object+0x64] && ram[object+0x62]==1 && !ram[current+0x40dc+(seed&1)]);
        if(seed&1)check(ram[0x1d3000+0x3e6a]==1);else check(!*reinterpret_cast<const uint32_t *>(ram+current+0x40a8));
    }
    // Renderer rejection cannot execute any completion or mutate CPU/RAM at
    // the publication site. The original wrapper's prologue is outside it.
    initialize(native.runtime(),nativeHost.scene,nativeHost.assets,1);nativeHost.reject=true;nativeHost.submitted.clear();nativeHost.callbacks=0;
    R5900Context c{};c.pc=0x22f140;c.r[31].uw[0]=0x22f164;c.r[4].uw[0]=object;c.r[5].uw[0]=object+0x5b0;
    const auto before=c;const auto ram=std::vector<uint8_t>(native.runtime().memory.flat(),native.runtime().memory.flat()+bozo::kFlatBytes);bool bad=false;
    try{vp2s::nativeParticleDrawInstruction(&c,&native.runtime(),0x22f160);}catch(const std::runtime_error &){bad=true;}
    check(bad && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),native.runtime().memory.flat(),ram.size()) && nativeHost.submitted.empty() && !nativeHost.callbacks);
    std::printf("128 original enqueue/native draw pairs: real compiled completion, whole CPU/RAM/FP, both layers and renderer rejection passed\n");
}
#ifdef VP2S_PARTICLE_GPU
void gpuCases(const std::filesystem::path &dir){
    using namespace bozo::render;
    check(SDL_Init(SDL_INIT_VIDEO));SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,3);SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,3);SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,SDL_GL_CONTEXT_PROFILE_CORE);
    auto *window=SDL_CreateWindow("VP2S particle tests",64,48,SDL_WINDOW_OPENGL|SDL_WINDOW_HIDDEN);check(window);const auto context=SDL_GL_CreateContext(window);
    check(context && gl::load([](const char *name){return reinterpret_cast<void *>(SDL_GL_GetProcAddress(name));}));
    auto &renderer=Renderer::instance();renderer.setEnabled(true);renderer.setPresentsAlone(true);renderer.setClearColour(0);
    Runtime rt;vp2s::NativeScene scene;vp2s::NativeAssets assets;vp2s::NativeParticles particles;vp2s::NativeParticleDraws draws(dir);
    {std::fstream f(dir/"c_0000000000000000.tex",std::ios::in|std::ios::out|std::ios::binary);f.seekp(12);const std::array<uint32_t,4> white{0x80ffffff,0x80ffffff,0x80ffffff,0x80ffffff};f.write(reinterpret_cast<const char *>(white.data()),sizeof(white));}
    vp2s::NativeTextures textures(dir,dir/"manifest.txt");unsigned samples=0;
    for(bool rotate:{false,true})for(unsigned alpha:{32u,64u,128u}){
        initialize(rt,scene,assets,1);put(rt,object+0x310,uint32_t(1));
        auto screen=identity;screen[0]=160;screen[5]=112;screen[3]=screen[7]=2048;put(rt,camera+0x330,screen);
        auto guard=identity;guard[0]=guard[5]=.01f;put(rt,camera+0x370,guard);scene.selectCamera(&rt,camera);
        const auto point=0x210040u;put(rt,point,std::array<float,4>{0,0,.5f,1});put(rt,point+16,std::array<float,4>{128,128,128,float(alpha)});
        put(rt,point+32,std::array<float,3>{rotate?1.5707963267948966f:0.f,.5f,1});
        put(rt,kinds+0xc0,std::array<float,4>{-1,-1,0,1});put(rt,kinds+0xd0,std::array<float,4>{1,1,0,1});put(rt,kinds+0x100,std::array<float,4>{1,1,1,1});
        const auto submission=draws.capture(&rt,assets,textures,scene,particles,object);check(submission.batches.size()==1);
        renderer.beginFrame({1728,1824,640,448});for(const auto &b:submission.batches)renderer.draw(b.material,b.matrix.data(),b.vertices.data(),b.vertices.size(),b.texture);
        renderer.endFrame();gl::api().Viewport(0,0,64,48);renderer.present(0,0,64,48);std::array<uint8_t,64*48*4> pixels{};gl::api().ReadPixels(0,0,64,48,gl::kRgba,gl::kUnsignedByte,pixels.data());
        for(unsigned y:{4u,21u,26u,43u})for(unsigned x:{4u,29u,34u,59u}){
            const int expected=x>4 && x<59 && y>4 && y<43?int((alpha*255+64)/128):0;
            for(unsigned c=0;c<3;++c){const auto actual=pixels[(y*64+x)*4+c];if(std::abs(int(actual)-expected)>2)throw std::runtime_error(std::format("particle GPU angle{} alpha{} zone{},{} channel{} actual{} expected{}",rotate,alpha,x,y,c,actual,expected));++samples;}}
    }
    renderer.releaseGpu();SDL_GL_DestroyContext(context);SDL_DestroyWindow(window);SDL_Quit();std::printf("6 native particle GPU cases, %u measured RGB samples passed\n",samples);
}
#endif
}
int main(int argc,char **argv){try{
    check(argc==2 || argc==3);const std::filesystem::path dir=argv[1];files(dir);
#ifdef VP2S_PARTICLE_GPU
    if(argc==3){gpuCases(dir);return 0;}
#endif
    Runtime rt;vp2s::NativeScene scene;vp2s::NativeAssets assets;vp2s::NativeParticles particles;
    vp2s::NativeParticleDraws draws(dir);vp2s::NativeTextures textures(dir,dir/"manifest.txt");const auto saved=_mm_getcsr();unsigned cases=0,vertices=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        initialize(rt,scene,assets,seed);const auto before=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&8?0x8040:0);_mm_setcsr(control);
        const auto result=draws.capture(&rt,assets,textures,scene,particles,object);check(_mm_getcsr()==control);_mm_setcsr(saved);
        check(result.object==object && result.camera==camera && !std::memcmp(before.data(),rt.memory.flat(),before.size()));const unsigned count=seed%5;
        check(result.batches.size()==(count?1:0));if(count){const auto &batch=result.batches[0];check(batch.vertices.size()==count*6);const auto &m=batch.material;
            check(m.textured && m.textureAlpha && m.alphaBlend && m.gsBlend && m.depthTest && !m.depthWrite && !m.alphaTest && m.clampU && !m.clampV);
            check(m.blendA==0 && m.blendB==1 && m.blendC==0 && m.blendD==1);check(batch.texture && batch.texture->rgba[0]==0x800000ff);
            check(batch.matrix[0]==100 && batch.matrix[5]==100 && batch.matrix[12]==2048 && batch.matrix[13]==2048 && std::bit_cast<uint32_t>(batch.matrix[14])==0x4d200821u);
            const unsigned strip[]={0,1,2,1,3,2};for(unsigned i=0;i<count;++i)for(unsigned j=0;j<6;++j){const auto k=strip[j];const auto &v=batch.vertices[i*6+j];const float x=k&1?.25f:-.25f,y=k>>1?.25f:-.25f;
                const float expectedX=float(i)/20+(seed&2?y:.5f*x),expectedY=seed&2?-.5f*x:y;
                check(std::abs(v.x-expectedX)<1.e-6f && std::abs(v.y-expectedY)<1.e-6f && v.z==.5f);check(v.u==float(k&1) && v.v==float(k>>1));
                check(v.rgba==(seed&4?0x80401010u:0x40101008u));}
            const auto first=batch.vertices[0];put(rt,(seed&1?0x210000u:0x200000u)+0x40,99.f);check(batch.vertices[0].x==first.x && batch.vertices[0].rgba==first.rgba);vertices+=batch.vertices.size();}
        ++cases;
    }
    for(unsigned which=0;which<28;++which){initialize(rt,scene,assets,1);particles=vp2s::NativeParticles{};constexpr uint32_t chunk=0x210020,point=chunk+32;
        if(which==0)put(rt,object,uint32_t(0));else if(which==1)put(rt,object+0xaff,uint8_t(2));else if(which==2)put(rt,object+0xb04,uint8_t(1));
        else if(which==3)put(rt,current+0x40dd,uint8_t(0));else if(which==4)put(rt,object+0xabc,uint32_t(2));else if(which==5)put(rt,chunk+0x10,uint32_t(49));else if(which==6)put(rt,chunk+0x18,uint32_t(2));
        else if(which==7)put(rt,chunk+0x14,uint32_t(6));else if(which==8)put(rt,point+44,uint32_t(1));else if(which==9)put(rt,kinds+0xc0+40,.5f);
        else if(which==10)put(rt,point+28,uint32_t(0x7fa01234));else if(which==11)put(rt,point+32,uint32_t(0x7fa01234));else if(which==12)put(rt,point+36,uint32_t(1));
        else if(which==13)put(rt,object+0x210,uint8_t(1));else if(which==14)put(rt,object+0x634,uint16_t(1));else if(which==15)put(rt,fis+4,uint32_t(0));
        else if(which==16)assets.overwrite(kinds+0x20,1);else if(which==17)put(rt,manager+0x2670,int16_t(256));else if(which==18)put(rt,object+0xaa4,uint32_t(0x200000));
        else if(which==19)put(rt,object+0xaf9,uint8_t(2));else if(which==20)put(rt,object+0x30c,-1.f);else if(which==21)put(rt,kinds+0xc0+64,-1.f);else if(which==22)put(rt,object+0x310,uint32_t(2));
        else if(which==23){R5900Context c{};c.pc=0x22eadc;c.r[21].uw[0]=object;c.r[29].uw[0]=0x1ffe000;c.r[30].uw[0]=0;put(rt,0x1ffe0e0,uint32_t(0x200020));put(rt,object+0xa90,uint32_t(0x1d0000));particles.begin(&rt,&c);}
        else if(which==24)put(rt,object+0x63,uint8_t(0));
        else {put(rt,camera+0x330,which==25?uint32_t(0x7fa01234):which==26?uint32_t(1):uint32_t(0x7f7fffff));scene.selectCamera(&rt,camera);}
        const auto fp=_mm_getcsr();const auto before=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);const auto active=particles.outstanding();bool rejected=false;
        try{draws.capture(&rt,assets,textures,scene,particles,object);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && _mm_getcsr()==fp && active==particles.outstanding() && !std::memcmp(before.data(),rt.memory.flat(),before.size()));
    }
    _mm_setcsr(saved);completionCases(dir);std::printf("%u native particle captures,%u known vertices; axis geometry/UV/two-stage colour/material, opaque transport words, whole RAM/FP, immutable output and28 rejection cases passed\n",cases,vertices);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

#include "native_procedural_draws.h"
#include <bit>
#include <cstdio>
#include <cstring>
#include <immintrin.h>
#include <source_location>
#ifdef VP2S_PROCEDURAL_GPU
#include "bozo/render/renderer.h"
#include "bozo/render/gl.h"
#include <SDL3/SDL.h>
#endif
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("procedural draw assertion at "+std::to_string(at.line()));}
template<class T>void put(Runtime &rt,uint32_t at,const T &v){auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(p,&v,sizeof(v));}
constexpr uint32_t object=0x180000,camera=0x190000,target=0x1a0000,sceneObject=0x1b0000,manager=0x1c0000,parent=0x1fff000,stack=parent-0xb0;
const std::array<float,16> identity{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
void initialize(Runtime &rt,vp2s::NativeScene &scene,vp2s::NativeProceduralTransforms &projections,unsigned seed){
    std::memset(rt.memory.flat(),0,bozo::kFlatBytes);scene=vp2s::NativeScene{};projections=vp2s::NativeProceduralTransforms{};
    put(rt,0x1c8d00+0xcc84,uint8_t(1));put(rt,0x35ddf8,sceneObject);put(rt,sceneObject+0x40a8+4*(seed&1),object);put(rt,sceneObject+0x40dc+(seed&1),uint8_t(1));
    put(rt,target+0x3c76,uint8_t(1));put(rt,target+0x3c60,1.f);put(rt,target+0x3c64,1.f);put(rt,target+0x3c68,int16_t(640));put(rt,target+0x3c6a,int16_t(448));
    for(unsigned i=0;i<2;++i){const auto base=target+0xa70*i;put(rt,base+0x1cf0,uint8_t(1));put(rt,base+0x1cf2,uint8_t(i));put(rt,base+0x1cf5,uint8_t(1));put(rt,base+0x1cfa,int16_t(640));put(rt,base+0x1cfc,int16_t(224));}
    scene.beginComposition(&rt,0x1e0000,target);put(rt,camera+0x220,uint32_t(0x153120));put(rt,camera+0x8e0,uint32_t(0x1d0000));put(rt,0x1d0000,uint32_t(0x153140));
    put(rt,camera+0x2f0,identity);put(rt,camera+0x330,identity);put(rt,camera+0x370,identity);put(rt,camera+0x244,2048.f);put(rt,camera+0x248,2048.f);put(rt,camera+0x260,1728.f);put(rt,camera+0x264,1824.f);scene.selectCamera(&rt,camera);
    put(rt,object,uint32_t(0x152d50));put(rt,object+0x63,uint8_t(1));put(rt,object+0x65,uint8_t(seed&1));put(rt,object+0x244,uint8_t((seed>>1)&1));put(rt,object+0x245,uint8_t((seed>>2)&1));
    put(rt,0x35de38,manager);put(rt,manager+0x2670,int16_t(1));put(rt,manager+0x1018,uint32_t(0x4674404c));put(rt,manager+0x18,uint32_t(1));
    put(rt,manager+0x1020,uint64_t(0xffffffffffffffff));put(rt,object+0x234,uint32_t(0xdeadbeef));put(rt,object+0x240,uint32_t(0xdeadbeef));
    const auto prepare=parent-0x170;put(rt,prepare+0x20,uint32_t(0x202bc8));put(rt,prepare+0x130,identity);
    R5900Context c{};c.pc=0x2481d0;c.r[31].uw[0]=0x2481d0;c.r[29].uw[0]=prepare;c.r[4].uw[0]=prepare+0x130;c.r[16].uw[0]=camera;c.r[17].uw[0]=object;projections.publish(&rt,&c,scene);
}
R5900Context meshContext(unsigned seed,unsigned count){R5900Context c{};c.pc=0x247700;c.r[31].uw[0]=0x247700;c.r[29].uw[0]=stack;c.r[21].uw[0]=object;c.r[20].uw[0]=count;c.r[22].uw[0]=(seed>>3)&1;c.r[16].uw[0]=(seed>>4)&1;c.r[17].uw[0]=0x1d1000;return c;}
void vertexInput(Runtime &rt,R5900Context &c,unsigned index,bool clip=false,uint32_t colour=0x80808080){
    const bool fan=c.r[22].uw[0];const unsigned order[4]={0,1,fan?3u:2u,fan?2u:3u};const auto corner=order[index];c.r[5].uw[0]=index;
    const std::array<int32_t,4> position{int32_t(16*(2048+(corner&1?160:-160))),int32_t(16*(2048+(corner&2?112:-112))),1000,clip?0x8000:16};
    put(rt,stack+0x80,position);put(rt,stack+0xa8,std::array<float,2>{float((corner>>1)&1),float(corner&1)});put(rt,0x1d1000+4*index,colour);
}
void end(vp2s::NativeProceduralDraws &draws,R5900Context &c,vp2s::NativeProceduralTransforms &projections){
    c.pc=c.r[31].uw[0]=0x247ac0;draws.endMesh(&c);c.pc=0x247ec0;c.r[31].uw[0]=0x2038d0;c.r[29].uw[0]=parent;c.r[4].uw[0]=object;draws.finish(&c,projections);projections.finish(parent,object);
}
#ifdef VP2S_PROCEDURAL_GPU
void gpu(){
    using namespace bozo::render;check(SDL_Init(SDL_INIT_VIDEO));SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,3);SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,3);SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,SDL_GL_CONTEXT_PROFILE_CORE);
    auto *window=SDL_CreateWindow("VP2S procedural tests",64,48,SDL_WINDOW_OPENGL|SDL_WINDOW_HIDDEN);check(window);const auto context=SDL_GL_CreateContext(window);
    check(context && gl::load([](const char *name){return reinterpret_cast<void *>(SDL_GL_GetProcAddress(name));}));auto &renderer=Renderer::instance();renderer.setEnabled(true);renderer.setPresentsAlone(true);renderer.setClearColour(0);
    Runtime rt;vp2s::NativeScene scene;vp2s::NativeProceduralTransforms projections;unsigned samples=0;
    for(bool fan:{false,true})for(unsigned alpha:{32u,64u,128u})for(bool occluded:{false,true}){
        const unsigned seed=16+4+(fan?8:0);initialize(rt,scene,projections,seed);vp2s::NativeProceduralDraws draws;vp2s::NativeFeedbackCopies copies;auto c=meshContext(seed,4);
        draws.beginMesh(&rt,&c,scene,projections);for(unsigned i=0;i<4;++i){vertexInput(rt,c,i,false,0x00808080|(alpha<<24));draws.vertex(&rt,&c);}end(draws,c,projections);
        const vp2s::NativeFeedbackCopy copy{0x1d2000,target,0x1d3000,640,448,scene.frameTarget(),0,640,448};copies.published(copy);const auto submission=draws.submission(&rt,scene,copies,object);
        renderer.beginFrame({1728,1824,640,448});auto fill=[&](uint32_t colour,float z,bool write){
            Material m;m.textured=false;m.depthTest=false;m.depthWrite=write;m.alphaBlend=false;m.blend=Material::Blend::Opaque;
            const float xy[6][2]={{0,0},{1,0},{0,1},{1,0},{1,1},{0,1}};Vertex vertices[6];for(unsigned i=0;i<6;++i){vertices[i].x=1728+640*xy[i][0];vertices[i].y=1824+448*xy[i][1];vertices[i].z=z;vertices[i].rgba=colour;}
            renderer.draw(m,identity.data(),vertices,6);
        };
        // The feedback image has alpha zero. Its colour must still contribute:
        // this material uses the vertex's alpha, proved from the allocation.
        fill(0x00808080,0,false);vp2s::drawFeedbackCopy(copy);fill(0x80000000,occluded?2000.f:500.f,true);
        for(const auto &batch:submission.batches)renderer.draw(batch.material,batch.matrix.data(),batch.vertices.data(),batch.vertices.size(),batch.texture);
        renderer.endFrame();gl::api().Viewport(0,0,64,48);renderer.present(0,0,64,48);std::array<uint8_t,64*48*4> pixels{};gl::api().ReadPixels(0,0,64,48,gl::kRgba,gl::kUnsignedByte,pixels.data());
        for(unsigned y:{4u,18u,30u,43u})for(unsigned x:{4u,24u,40u,59u}){
            const int expected=!occluded && x>4 && x<59 && y>4 && y<43?int((alpha*255+64)/128):0;
            for(unsigned lane=0;lane<3;++lane){const auto actual=pixels[(y*64+x)*4+lane];if(std::abs(int(actual)-expected)>2){std::fprintf(stderr,"procedural GPU fan%u alpha%u occluded%u pixel%u,%u lane%u actual%u expected%d\n",fan,alpha,occluded,x,y,lane,actual,expected);check(false);}++samples;}}
    }
    renderer.releaseGpu();SDL_GL_DestroyContext(context);SDL_DestroyWindow(window);SDL_Quit();std::printf("12 native procedural GPU cases,%u measured RGB samples: strips/fans, source-alpha ignored, vertex alpha, raw depth occlusion and coverage passed\n",samples);
}
#endif
}
int main(int argc,char **argv){try{
#ifdef VP2S_PROCEDURAL_GPU
    if(argc>1){gpu();return 0;}
#endif
    Runtime rt;vp2s::NativeScene scene;vp2s::NativeProceduralTransforms projections;const auto saved=_mm_getcsr();unsigned cases=0;
    for(unsigned seed=0;seed<128;++seed){initialize(rt,scene,projections,seed);vp2s::NativeProceduralDraws draws;vp2s::NativeFeedbackCopies copies;
        const unsigned count=seed%5;auto c=meshContext(seed,count);const auto before=c;const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|((seed%4)<<13)|(seed&8?0x8040:0);_mm_setcsr(control);
        draws.beginMesh(&rt,&c,scene,projections);check(!std::memcmp(&before,&c,sizeof(c)) && _mm_getcsr()==control);
        for(unsigned i=0;i<count;++i){vertexInput(rt,c,i,seed&32 && i==2,0x80402010+seed);const auto cpu=c;const auto ram=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);draws.vertex(&rt,&c);check(!std::memcmp(&cpu,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()) && _mm_getcsr()==control);}
        end(draws,c,projections);check(_mm_getcsr()==control);_mm_setcsr(saved);
        bool rejected=false;try{draws.submission(&rt,scene,copies,object);}catch(const std::runtime_error &){rejected=true;}check(rejected && draws.pending()==1);
        copies.published({0x1d2000,target,0x1d3000,640,448,scene.frameTarget(),0,640,448});const auto submission=draws.submission(&rt,scene,copies,object);
        const unsigned triangles=count>=3?(count-2)-unsigned(bool(seed&32)):0;check(submission.object==object && submission.camera==camera && submission.batches.size()==unsigned(triangles!=0));
        if(triangles){const auto &batch=submission.batches[0];check(batch.vertices.size()==triangles*3 && batch.matrix==identity);const auto &m=batch.material;
            check(m.fromFrame && m.source==5 && !m.textureAlpha && m.textureFunction==bozo::render::Material::TextureFunction::Modulate && m.alphaBlend==bool(seed&16) && m.depthTest==!bool(seed&2) && m.depthWrite==!bool(seed&4));
            for(const auto &v:batch.vertices){check(v.x==1888 || v.x==2208);check(v.y==1936 || v.y==2160);check(v.z==1000 && v.u==(v.x==2208?1.f:0.f) && v.v==(v.y==2160?1.f:0.f) && v.rgba==0x80402010+seed);}}
        draws.consumed(object);check(!draws.pending());rejected=false;try{draws.consumed(object);}catch(const std::runtime_error &){rejected=true;}check(rejected);++cases;
    }
    for(unsigned which=0;which<21;++which){initialize(rt,scene,projections,0);vp2s::NativeProceduralDraws draws;vp2s::NativeFeedbackCopies copies;auto c=meshContext(0,4);
        if(which==0)put(rt,object,uint32_t(0));else if(which==1)put(rt,manager+0x1018,uint32_t(0));else if(which==2)c.r[22].uw[0]=2;else if(which==3)c.pc=0;else if(which==16)c.r[16].uw[0]=2;
        else {draws.beginMesh(&rt,&c,scene,projections);
            if(which>=10 && which<=14){for(unsigned i=0;i<4;++i){vertexInput(rt,c,i);draws.vertex(&rt,&c);}end(draws,c,projections);
                if(which!=10)copies.published({0x1d2000,target,0x1d3000,640,448,scene.frameTarget(),0,640,448});
                if(which==11)put(rt,sceneObject+0x40dc,uint8_t(0));else if(which==12)put(rt,object+0x65,uint8_t(2));else if(which==13){scene.finishComposition(0x1e0000);scene.beginComposition(&rt,0x1e0000,target);}else if(which==14)put(rt,object,uint32_t(0));
            }else if(which==5 || which==6 || which==15 || which==17){vertexInput(rt,c,0);if(which==5)c.r[5].uw[0]=1;else if(which==6)put(rt,stack+0xa8,uint32_t(0x7fa01234));else if(which==15)c.r[21].uw[0]+=16;else draws.vertex(&rt,&c);
            }else if(which==7 || which==18){c.pc=c.r[31].uw[0]=0x247ac0;if(which==18)c.r[31].uw[0]=0;
            }else if(which==8 || which==19){c.pc=0x247ec0;c.r[31].uw[0]=which==19?0:0x2038d0;c.r[29].uw[0]=parent;c.r[4].uw[0]=object;}}
        const auto before=c;const auto active=draws.pending(),pointCount=projections.pendingPoints();const auto fp=_mm_getcsr();const auto ram=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        try{if(which<=4 || which==16)draws.beginMesh(&rt,&c,scene,projections);else if(which==5 || which==6 || which==15 || which==17)draws.vertex(&rt,&c);
            else if(which==7 || which==18)draws.endMesh(&c);else if(which==8 || which==19)draws.finish(&c,projections);else if(which==20)draws.consumed(object);else draws.submission(&rt,scene,copies,object);
        }catch(const std::runtime_error &){rejected=true;}
        check(rejected && draws.pending()==active && projections.pendingPoints()==pointCount && _mm_getcsr()==fp && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));
    }
    _mm_setcsr(saved);std::printf("%u native procedural captures: known strips/fans/ADC triangles, UV/colour/depth units, opaque packet words, CPU/RAM/FP, feedback publication/consumption and21 readonly failures passed\n",cases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

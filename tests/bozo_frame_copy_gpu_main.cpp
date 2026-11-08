// Real native render-to-texture readback; hidden video-only window, no input.
#include "native_frame_copy.h"
#include "bozo/render/renderer.h"
#include "bozo/render/gl.h"
#include <SDL3/SDL.h>
#include <array>
#include <cstdio>
#include <cmath>
using namespace bozo::render;
int main(){
    if(!SDL_Init(SDL_INIT_VIDEO))return 1;
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,3);SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,SDL_GL_CONTEXT_PROFILE_CORE);
    auto *window=SDL_CreateWindow("VP2S frame-copy tests",64,48,SDL_WINDOW_OPENGL|SDL_WINDOW_HIDDEN);if(!window)return 1;
    const auto context=SDL_GL_CreateContext(window);
    if(!context || !gl::load([](const char *name){return reinterpret_cast<void *>(SDL_GL_GetProcAddress(name));}))return 1;
    auto &r=Renderer::instance();r.setEnabled(true);r.setPresentsAlone(true);r.setClearColour(0);
    const Screen screen{2016,2024,64,48};const float id[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    auto quad=[&](float x,float y,float w,float h,uint32_t colour,Material material){
        Vertex v[6];const float xy[6][2]={{0,0},{1,0},{0,1},{1,0},{1,1},{0,1}};
        for(unsigned i=0;i<6;++i){v[i].x=2016+x+xy[i][0]*w;v[i].y=2024+y+xy[i][1]*h;v[i].u=xy[i][0];v[i].v=xy[i][1];v[i].rgba=colour;}r.draw(material,id,v,6);
    };
    std::array<uint8_t,64*48*4> baseline{},copied{};unsigned cases=0,failures=0,compared=0;
    auto frame=std::make_shared<vp2s::NativeFrameTarget>(vp2s::NativeFrameTarget{0x180000,64,48});
    for(unsigned seed=0;seed<4;++seed)for(unsigned size:{8u,16u,32u}){
        for(bool copy:{false,true}){
            r.beginFrame(screen);Material m;m.textured=false;m.depthTest=false;m.depthWrite=false;m.alphaBlend=false;m.blend=Material::Blend::Opaque;
            const uint32_t colours[4]={0x80000060u+seed*4,0x80006000u+seed*0x400,0x80600000u+seed*0x40000,0x80606060u-seed*0x30303};
            for(unsigned i=0;i<4;++i)quad(float(i%2)*32,float(i/2)*24,32,24,colours[i],m);
            if(copy){vp2s::drawFrameCopy({0x190000,size,size*3/4,frame});m.fromFrame=1;m.source=7;m.nearest=false;m.clampU=m.clampV=true;quad(0,0,64,48,0x80808080,m);}
            r.endFrame();gl::api().Viewport(0,0,64,48);r.present(0,0,64,48);
            gl::api().ReadPixels(0,0,64,48,gl::kRgba,gl::kUnsignedByte,(copy?copied:baseline).data());
        }
        // Compare sixteen interior zones in all quadrants; exclude only the
        // intentional bilinear transition along their shared boundaries.
        for(unsigned y:{6u,12u,36u,42u})for(unsigned x:{8u,16u,48u,56u})for(unsigned lane=0;lane<4;++lane){
            const auto at=(y*64+x)*4+lane;++compared;if(std::abs(int(baseline[at])-int(copied[at]))>2){
                if(failures<8)std::printf("copy GPU seed=%u size=%u pixel=%u,%u lane=%u baseline=%u copy=%u\n",seed,size,x,y,lane,baseline[at],copied[at]);++failures;}
        }
        ++cases;
    }
    r.releaseGpu();SDL_GL_DestroyContext(context);SDL_DestroyWindow(window);SDL_Quit();
    std::printf("%u native frame-copy GPU cases, %u interior RGBA samples, %u failures\n",cases,compared,failures);return failures?1:0;
}

// Synthetic native texture admission to both presenters. Hidden window, no input or game files.
#include "native_textures.h"
#include "bozo/render/renderer.h"
#include "bozo/render/gl.h"
#include <SDL3/SDL.h>
#include <cstdio>
#include <cstring>
#include <cmath>
using namespace bozo::render;
int main(){
    if(!SDL_Init(SDL_INIT_VIDEO))return 1;
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,3);SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,SDL_GL_CONTEXT_PROFILE_CORE);
    auto *window=SDL_CreateWindow("VP2S native alpha tests",32,32,SDL_WINDOW_OPENGL|SDL_WINDOW_HIDDEN);
    if(!window)return 1;const auto context=SDL_GL_CreateContext(window);
    if(!context || !gl::load([](const char *name){return reinterpret_cast<void *>(SDL_GL_GetProcAddress(name));}))return 1;
    gl::api().Viewport(0,0,32,32);auto &r=Renderer::instance();r.setEnabled(true);r.setPresentsAlone(true);r.setClearColour(0);
    const Screen screen{0,0,16,16};const float id[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    Vertex vertices[6];const float xy[6][2]={{0,0},{16,0},{0,16},{16,0},{16,16},{0,16}};
    for(unsigned i=0;i<6;++i){vertices[i].x=xy[i][0];vertices[i].y=xy[i][1];vertices[i].rgba=0x80808080;}
    Material material;material.depthTest=false;material.depthWrite=false;material.blend=Material::Blend::Alpha;
    unsigned failures=0,cases=0;
    for(bool indexed:{false,true})for(unsigned alpha:{0u,16u,32u,64u,96u,128u})for(bool overlay:{false,true}){
        vp2s::NativeTextureAsset asset;asset.width=asset.height=1;asset.identity=100+alpha+unsigned(indexed)*256;
        const uint32_t colour=0xffffffu|(alpha<<24);
        if(indexed){asset.format=0x14;asset.texels={3};asset.palette[19]=colour;}
        else{asset.format=0;asset.texels.resize(4);std::memcpy(asset.texels.data(),&colour,4);}
        auto texture=asset.picture(indexed?1:0);
        r.publishOverlay(screen);r.beginFrame(screen);
        if(!overlay)r.draw(material,id,vertices,6,texture);r.endFrame();
        if(overlay){r.drawOverlay(material,vertices,6,texture);r.publishOverlay(screen);}
        r.present(0,0,32,32);uint8_t pixel[4]{};gl::api().ReadPixels(16,16,1,1,gl::kRgba,gl::kUnsignedByte,pixel);
        const int expected=std::min(255u,(alpha*255+64)/128);
        if(std::abs(int(pixel[0])-expected)>2 || pixel[0]!=pixel[1] || pixel[1]!=pixel[2]){
            std::printf("FAIL native texture alpha=%u indexed=%u overlay=%u RGB=%u,%u,%u expected=%d\n",alpha,indexed,overlay,pixel[0],pixel[1],pixel[2],expected);++failures;
        }
        ++cases;
    }
    r.releaseGpu();SDL_GL_DestroyContext(context);SDL_DestroyWindow(window);SDL_Quit();
    std::printf("%u native partial-alpha CPU-asset/GPU-presenter cases: %s\n",cases,failures?"FAILED":"passed");return failures?1:0;
}

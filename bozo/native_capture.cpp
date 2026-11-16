// Read back the completed native frame, with no device input or visible window.
#include "native_capture.h"
#include "bozo/render/renderer.h"
#include "bozo/render/gl.h"
#include "bozo/app/png.h"
#include <SDL3/SDL.h>
#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <vector>
#include <cstdio>
namespace vp2s {
namespace {
// One hidden window and context for every capture of the run, so the renderer's
// GPU state (kept images 9..15, the previous presentation) survives between
// captures as it does between presented frames in a window. With captures every
// frame (VP2S_BOZO_CAPTURE_EVERY=1) frame-to-frame effects show as in play.
struct CaptureContext {
    SDL_Window *window=nullptr;SDL_GLContext context=nullptr;uint32_t width=0,height=0;
    ~CaptureContext(){
        if(context){bozo::render::Renderer::instance().releaseGpu();SDL_GL_DestroyContext(context);}
        if(window)SDL_DestroyWindow(window);
        if(window || context)SDL_Quit();
    }
    void open(uint32_t w,uint32_t h){
        if(context && w==width && h==height){SDL_GL_MakeCurrent(window,context);return;}
        if(context){bozo::render::Renderer::instance().releaseGpu();SDL_GL_DestroyContext(context);context=nullptr;}
        if(window){SDL_DestroyWindow(window);window=nullptr;}
        if(!SDL_WasInit(SDL_INIT_VIDEO) && !SDL_Init(SDL_INIT_VIDEO))throw std::runtime_error(SDL_GetError());
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,3);SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,SDL_GL_CONTEXT_PROFILE_CORE);
        window=SDL_CreateWindow("VP2S native capture",w,h,SDL_WINDOW_OPENGL|SDL_WINDOW_HIDDEN);
        if(!window)throw std::runtime_error("hidden native capture window failed");
        context=SDL_GL_CreateContext(window);
        if(!context || !bozo::render::gl::load([](const char *name){return reinterpret_cast<void *>(SDL_GL_GetProcAddress(name));}))
            throw std::runtime_error("native capture OpenGL context failed");
        width=w;height=h;
    }
};
CaptureContext &captureContext(){static CaptureContext context;return context;}
}
void captureNativeFrame(const std::string &path){
    auto &renderer=bozo::render::Renderer::instance();const auto published=renderer.published(),backdrop=renderer.backdrop();
    const auto frame=published && !published->batches.empty()?published:backdrop;
    if(!frame || frame->batches.empty())throw std::runtime_error("native capture has no completed nonempty frame or movie");
    const auto width=uint32_t(frame->screen.width),height=uint32_t(frame->screen.height);
    if(!width || !height || width>2048 || height>2048)throw std::runtime_error("native capture dimensions outside bounds");
    captureContext().open(width,height);
    bozo::render::gl::api().Viewport(0,0,width,height);renderer.present(0,0,width,height);
    std::vector<uint8_t> bottomUp(width*height*4),topDown(bottomUp.size());
    bozo::render::gl::api().ReadPixels(0,0,width,height,bozo::render::gl::kRgba,bozo::render::gl::kUnsignedByte,bottomUp.data());
    for(uint32_t y=0;y<height;++y)std::copy_n(bottomUp.data()+(height-1-y)*width*4,width*4,topDown.data()+y*width*4);
    const auto output=std::filesystem::absolute(path);std::filesystem::create_directories(output.parent_path());
    if(!bozo::app::writePng(output.string(),topDown.data(),width,height))throw std::runtime_error("native capture PNG write failed");
    std::fprintf(stderr,"[vp2s:capture] native frame serial=%llu batches=%zu vertices=%zu size=%ux%u saved\n",
        static_cast<unsigned long long>(frame->serial),frame->batches.size(),frame->vertices.size(),width,height);
}
}

#include "native_capture_fade.h"
#include <cstdio>
#include <functional>
#include <stdexcept>
namespace {
void check(bool ok,const char *what){if(!ok)throw std::runtime_error(what);}
bool rejects(const std::function<void()> &f){try{f();}catch(const std::runtime_error &){return true;}return false;}
}
int main(){try{
    unsigned checks=0;
    // The route's capture fade: object 6a2040 over the 512x512 town frame,
    // eight 64-pixel strips (+aa0 = 8).
    constexpr uint32_t object=0x6a2040,other=0x6a3040;
    {vp2s::NativeCaptureFades fades;
     const auto image=fades.capture(object);check(image>=9 && image<=15,"capture claims a kept image (9..15)");++checks;
     check(!fades.image(object),"no snapshot before the copy");++checks;
     check(fades.pendingCopy(object)==image,"first builder copies");++checks;
     check(!fades.pendingCopy(object),"copied once");++checks;
     check(fades.image(object)==image,"the copied snapshot");++checks;
     check(!fades.submit(object),"no draw before its builder");++checks;
     vp2s::NativeCaptureFadeDraw draw{.object=object,.colour=0x40808080,.depth=0x100,.image=image,.width=512,.height=512};
     fades.build(draw);const auto drawn=fades.submit(object);
     check(drawn && drawn->colour==0x40808080 && drawn->image==image,"submit draws what the builder recorded");++checks;
     check(!fades.submit(object),"one draw per build");++checks;
     check(fades.capture(object)==image && !fades.image(object),"a new capture reuses the image and waits for a new copy");++checks;
     auto wrong=draw;wrong.object=other;check(rejects([&]{fades.build(wrong);}),"no build without a snapshot");++checks;}
    // Seven kept images; an eighth capture recycles the least recently used
    // (a build counts as a use). The recycled fade has no snapshot any more:
    // its pending draw is dropped and its next build rejects.
    {vp2s::NativeCaptureFades fades;std::array<uint8_t,7> images{};
     for(uint32_t i=0;i<7;++i){images[i]=fades.capture(object+i*0x1000);fades.pendingCopy(object+i*0x1000);}
     for(uint32_t i=0;i<7;++i)fades.build({.object=object+i*0x1000,.image=images[i],.width=512,.height=512});
     check(fades.capture(other+0x100000)==images[0],"eighth capture recycles the least recently used image");++checks;
     check(!fades.submit(object) && !fades.image(object),"the recycled fade has no draw and no snapshot");++checks;
     check(fades.submit(object+0x1000) && fades.image(object+0x1000)==images[1],"other fades keep theirs");++checks;
     check(rejects([&]{fades.build({.object=object,.image=images[0],.width=512,.height=512});}),"the recycled fade's next build rejects");++checks;}
    // 3d2630's sprites: x from 0x800 - w/2 over eight 64-wide strips, y from
    // 0x800 - h/2; one texel per pixel.
    {const auto v=vp2s::nativeCaptureFadeVertices({.object=object,.colour=0x80406080,.depth=0x2a,.image=9,.width=512,.height=512});
     check(v[0].x==1792 && v[0].y==1792 && v[4].x==2304 && v[4].y==2304,"full frame at the strips' position");++checks;
     check(v[0].u==0 && v[0].v==0 && v[4].u==1 && v[4].v==1,"whole snapshot, 1:1");++checks;
     check(v[3].rgba==0x80406080 && v[5].z==0x2a,"fade colour and depth on every vertex");++checks;
     const auto odd=vp2s::nativeCaptureFadeVertices({.width=640,.height=447});
     check(odd[0].x==1728 && odd[0].y==1825 && odd[4].y==2272,"odd height rounds toward zero as 3d2630");++checks;}
    std::printf("capture fade: %u checks passed\n",checks);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"FAIL %s\n",e.what());return 1;}}

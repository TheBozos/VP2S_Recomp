#include "native_publication.h"
#include "bozo/render/renderer.h"
#include <cmath>
namespace vp2s {
void NativeFramePublisher::next(const NativeFrameTarget &target){
    if(!std::isfinite(target.width) || !std::isfinite(target.height) || target.width<1 || target.height<1 || target.width>2048 || target.height>2048)
        throw std::runtime_error("native frame publication dimensions outside bounds");
    auto &renderer=bozo::render::Renderer::instance();renderer.setEnabled(true);renderer.setPresentsAlone(true);
    // 0x114550 sets up a target, returns, then scene/UI code draws into it.
    // The next completed wait admits publication before that list is reset.
    if(building)renderer.endFrame();
    bozo::render::Screen screen;screen.width=target.width;screen.height=target.height;
    screen.originX=2048-target.width/2;screen.originY=2048-target.height/2;
    renderer.beginFrame(screen);building=true;
}
}

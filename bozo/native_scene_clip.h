#pragma once
#include "native_scene.h"
#include <array>
namespace vp2s {
// Scene scissor element (class 1597e0): its packet (builder 2b5b60) is only a
// SCISSOR write, which clips every later draw of the frame (the forest minimap:
// x 24..179, y 24..139). Natively: the renderer's clip from the same fields.
struct NativeSceneClip {uint32_t object=0;std::array<int32_t,4> rect{};};  // x0, y0, x1, y1 (inclusive frame pixels)
NativeSceneClip nativeSceneClip(Runtime *,const NativeScene &,uint32_t object);
class SceneClipOwner {
public:
    virtual ~SceneClipOwner()=default;
    virtual void setNativeSceneClip(const NativeSceneClip &)=0;
};
std::optional<bool> nativeSceneClipInstruction(R5900Context *,Runtime *,uint32_t pc);
}

#pragma once
#include "native_scene.h"
#include "bozo/render/renderer.h"
#include <array>
#include <map>
#include <optional>
namespace vp2s {
// Capture fade (mod003 class 0x1610f0, a fade 0x161230 subclass): the frame as
// it was when the fade began, redrawn over the new scene under the fade colour.
// The original reads the GS frame back into EE strips (3d2cd0) and uploads them
// again (3d2630). Natively the previous presentation is copied once into a kept
// engine image, then drawn 1:1: TEX0.TCC 0 (vertex alpha), modulate, ALPHA 0x44,
// depth test always, depth write masked.
struct NativeCaptureFadeDraw {
    uint32_t object=0,colour=0,depth=0;
    uint8_t image=0;
    int32_t width=0,height=0;
};
std::array<bozo::render::Vertex,6> nativeCaptureFadeVertices(const NativeCaptureFadeDraw &);
class NativeCaptureFades {
    struct Snapshot {uint32_t object=0;uint64_t used=0;bool copied=false;};
    std::array<Snapshot,7> images{};
    uint64_t clock=0;
    std::map<uint32_t,NativeCaptureFadeDraw> built;
    Snapshot *find(uint32_t object);
public:
    static constexpr uint8_t kFirstImage=9; // engine images 9..15
    uint8_t capture(uint32_t object);
    // The image still waiting for its copy (made in the frame the fade's builder draws into).
    std::optional<uint8_t> pendingCopy(uint32_t object);
    std::optional<uint8_t> image(uint32_t object); // the copied snapshot
    void build(const NativeCaptureFadeDraw &);
    std::optional<NativeCaptureFadeDraw> submit(uint32_t object);
};
class CaptureFadeOwner {
public:
    virtual ~CaptureFadeOwner()=default;
    virtual NativeCaptureFades &nativeCaptureFades()=0;
    virtual void copyNativeFrameToImage(uint8_t image)=0;
    virtual void drawNativeCaptureFade(const NativeCaptureFadeDraw &)=0;
};
std::optional<bool> nativeCaptureFadeInstruction(R5900Context *,Runtime *,uint32_t pc);
}

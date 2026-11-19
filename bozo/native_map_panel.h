#pragma once
#include "native_scene.h"
#include "bozo/render/renderer.h"
#include <vector>
namespace vp2s {
// Field map panel (mod003 class 167200, builder 4541d0): the minimap's backdrop,
// a vertical gradient rectangle with a 2-pixel dark border, alpha-blended over
// the frame. Natively: the same triangles from the object's records.
struct NativeMapPanel {
    uint32_t object=0;
    // Screen triangles (GS coordinates), direct colour; empty when the panel's opacity is zero.
    std::vector<bozo::render::Vertex> vertices;
};
NativeMapPanel nativeMapPanel(Runtime *,uint32_t object);
class MapPanelOwner {
public:
    virtual ~MapPanelOwner()=default;
    virtual void drawNativeMapPanel(const NativeMapPanel &)=0;
};
void drawMapPanel(const NativeMapPanel &);
std::optional<bool> nativeMapPanelInstruction(R5900Context *,Runtime *,uint32_t pc);
}

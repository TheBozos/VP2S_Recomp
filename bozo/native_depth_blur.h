#pragma once
#include "native_scene.h"
#include "bozo/render/renderer.h"
#include <map>
#include <optional>
#include <vector>
namespace vp2s {
// Main 1152c0: the multi-copy depth blur (graphics context = *(35de60)), enabled
// by mod001 set-blur 2098f0. The compiled builder still runs; its two GIF writers
// are observed at their entries for the scalar arguments it computed:
//  116070 plane (a2 w, a3 h, t0 alpha, t1 Z): alpha only, Z test GEQUAL, no Z write;
//  115830 copy  (frame texture, offset dx/16, (dy/2)/16 px, vertex alpha, AREF,
//               copy Z): alpha test GEQUAL AREF, Z test GEQUAL, ALPHA 0x44, RGBA written.
// Published at mod001 draw 1ffe70 (delay slot 1ffe88 of its enqueue call); no queue entry or DMA.
struct NativeDepthBlurPass {
    bool copy=false;
    uint32_t z=0;            // GS Z as the writer received it (PSMZ24 keeps 24 bits)
    uint8_t alpha=0;         // plane: written alpha; copy: vertex alpha (0x80 = 1.0)
    uint8_t reference=0;     // copy: TEST AREF
    int32_t dx=0,dy=0;       // copy: 12.4 x offset; y offset before the writer's halving
    bool last=false;         // copy: the writer appended the end-of-packet tag
};
struct NativeDepthBlur {
    uint32_t context=0;
    int32_t width=0,height=0;
    bool bilinear=false;
    std::vector<NativeDepthBlurPass> passes;
};
// Qwords each writer emits for a frame of width w (checked against the writers'
// own cursor advance while building, and against the published count).
uint32_t nativeDepthBlurPlaneQwords(int32_t width);
uint32_t nativeDepthBlurCopyQwords(int32_t width,bool last);
// GS-space geometry (pixels; u/v normalized to the frame image).
std::vector<bozo::render::Vertex> nativeDepthBlurPlaneVertices(const NativeDepthBlur &,const NativeDepthBlurPass &);
std::vector<bozo::render::Vertex> nativeDepthBlurCopyVertices(const NativeDepthBlur &,const NativeDepthBlurPass &);
class NativeDepthBlurs {
    struct Build {NativeDepthBlur blur;uint32_t base=0,end=0;};
    std::map<uint32_t,Build> builds;   // by packet buffer base
public:
    void plane(Runtime *,const R5900Context *);
    void copy(Runtime *,const R5900Context *);
    // At 1ffe88: the build the enqueue would send, or nullopt when it sends nothing.
    // The caller draws it, then retire() clears the packet start as the enqueue would.
    std::optional<NativeDepthBlur> publish(Runtime *,const R5900Context *);
    void retire(Runtime *,uint32_t context);
    size_t pending()const{return builds.size();}
};
class DepthBlurOwner {
public:
    virtual ~DepthBlurOwner()=default;
    virtual NativeDepthBlurs &nativeDepthBlurs()=0;
    virtual void drawNativeDepthBlur(const NativeDepthBlur &)=0;
};
std::optional<bool> nativeDepthBlurInstruction(R5900Context *,Runtime *,uint32_t pc);
}

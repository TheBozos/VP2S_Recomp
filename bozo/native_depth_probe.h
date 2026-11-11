#pragma once
#include "native_scene.h"
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <vector>
namespace vp2s {
// Readback node class 159010 (constructor 29a720, 0x210 bytes). Its request
// 29a520(object, destination, sbp, sbw, psm, x, y, w, h) flips slot +95, builds a
// 7-qword StoreImage packet at +e0+70*slot (main 122ab8, sceGsSetDefStoreImage)
// and keeps the destination at +200+4*slot. Promotion 29a5d0 copies the slot to
// +96. On the PS2 its enqueue 29a4b0 queues +90 on the DMA list; the list wakes
// thread 299c30, which reads the rectangle from GS memory through the VIF1 FIFO
// and calls notify 299c00 (slot flag +97+slot, scene completion 21c9a0).
//
// Natively, the engine's sceGsSetDefStoreImage builds no packet, so the request is
// recorded at that call, and the enqueue (29a4dc, delay slot of the patched call
// to 299c00) fills the destination from the renderer's depth and lets the real
// notify run. Only depth formats are owned: PSMZ32 (4 bytes) and PSMZ24 (3 packed
// bytes), rows top-down, which is the layout the consumers read (253360 counts the
// samples below its light's Z).
struct NativeDepthProbeRequest {
    uint32_t packet=0;                 // physical address of the slot's packet
    int32_t sbp=0,sbw=0,psm=0,x=0,y=0,width=0,height=0;
};
// A depth sample at a GS screen point (pixel centre), as GS Z; false when the
// renderer has none (headless, not read back yet).
using NativeDepthSampler=std::function<bool(float gsX,float gsY,uint32_t &gsZ)>;
struct NativeDepthProbeFrame {int32_t width=0,height=0;};
// The destination bytes for a request. Samples outside the frame, and samples the
// renderer cannot give, read as the format's nearest Z (all ones): they never
// count as visible. (The PS2 would read whatever GS memory lies beyond the
// buffer; this native rule is deterministic.)
std::vector<uint8_t> nativeDepthProbeBytes(const NativeDepthProbeRequest &,const NativeDepthProbeFrame &,const NativeDepthSampler &);
class NativeDepthProbes {
    std::map<uint32_t,NativeDepthProbeRequest> requests;   // by packet address
public:
    // At main 122ab8 called from 29a57c (return 29a584): a0 packet, a1 sbp, a2 sbw,
    // a3 psm, t0 x, t1 y, t2 w, t3 h.
    void request(const R5900Context *);
    // At 29a4dc: fills the slot's destination; returns the node object for a0.
    uint32_t complete(Runtime *,const R5900Context *,const NativeDepthProbeFrame &,const NativeDepthSampler &);
    size_t pending()const{return requests.size();}
};
class DepthProbeOwner {
public:
    virtual ~DepthProbeOwner()=default;
    virtual NativeDepthProbes &nativeDepthProbes()=0;
    virtual NativeDepthProbeFrame nativeDepthProbeFrame()=0;
    virtual bool nativeDepthSample(float gsX,float gsY,uint32_t &gsZ)=0;
};
std::optional<bool> nativeDepthProbeInstruction(R5900Context *,Runtime *,uint32_t pc);
}

#pragma once
#include "native_depth_probe.h"
#include <array>
#include <cstdint>
#include <map>
#include <optional>
namespace vp2s {
// Battle column capture (mod028 class, vtable 175290). Its state +1ea: 2 = capture
// (4de080 calls 4de910, then sets 3), 3 = per column k (0..8) a build 4de130 and the
// flush 4dde60 of its writer pairs (+bc draw / +128 upload, 12 bytes each), 1 =
// countdown +1e0 and teardown.
// 4de910 lays out 9 image records at align128(+94) + k * (+1e4 & ~15): +0 width 64,
// +2 height, +4 size, +c psm (+1e8), +10..7f an image upload packet, pixels at +80;
// and asks readback node +98+4k (29a520) for the frame's column k: SBP = the scene
// frame (+1d0 +160, FBP * 32), SBW 9, psm +1e8, x = 64k, y = 0, 64 x height, into
// record k + 80.
// 4de130 (column k) then resets its draw writer (ALPHA_2 0x44, then TEX1_2 1, PRIM 356
// = sprite/TME/ABE/FST, RGBAQ +1c0, TEST_2 ZTE/always, ZBUF_2 masked), uploads the
// record into a transient texture-cache allocation ('kuri', 29d9b0 / 29c5e0: a type-3
// chain in the upload writer), selects its TEX0 and writes one sprite: UV (0.5, 0.5)
// to (+0, +2) of the record, corners (x0, y0) = (sp+c0, sp+d0), (x1, y1) = (s3, v1)
// from the placement +1a0.. (screen pixels), z = 125090(+1a8). 4dde60 enqueues each
// upload writer (386300, a type-3 job without owner) and flushes each draw writer
// through 120eb0 (the last with owner +90).
//
// Natively 4de910's entry registers the effect: its nine nodes, records and a kept
// engine image (the capture fade's slots). The readback completion (29a4dc) of a
// registered node copies the frame drawn so far (the PS2 reads the frame at the node's
// place in the scene's list, which is where its enqueue runs) into that image, once per
// capture; the records' pixels in EE RAM stay unwritten: only the GS read them, through
// the upload, whose enqueue (4ddea0) is retired. Each column's sprite is recorded where
// 4de130 computed it (4de718, end at 4de888) and drawn from column k of the image at
// its draw writer's flush (4dded0 / 4ddef4); the scalar flush keeps its bookkeeping. The
// last column's job (owner +90) has no native queue to complete it: 4dde60's return is
// patched to its completion 4ddb70 (21c9a0, the scene's), entered with the object.
struct NativeColumnSprite {
    uint32_t writer=0,begin=0,end=0,colour=0,z=0;
    int32_t x0=0,y0=0,x1=0,y1=0;
    float u0=0,v0=0,u1=0,v1=0;   // texels of the record (GS UV)
    bool built=false;
};
struct NativeColumnCapture {
    uint32_t object=0,records=0,stride=0;int32_t psm=0,height=0;   // height: the requests' (record +2)
    std::array<uint32_t,9> nodes{};
    uint8_t image=0;bool copied=false;
    int32_t frameWidth=0,frameHeight=0;   // the copied frame's size (the image's)
    std::array<NativeColumnSprite,9> sprites{};
};
class NativeColumnCaptures {
    std::map<uint32_t,NativeColumnCapture> captures;   // by object
    struct Lease {uint32_t object,column,writer,owner,begin,end;bool retired=false,flushed=false;};
    std::map<uint32_t,Lease> leases;                    // by the flushing caller's stack
    std::map<uint32_t,uint32_t> finished;               // 4dde60's stack -> object, after its last column
public:
    // 4de910's entry (a0 = the object): returns the capture (image to claim by the host).
    NativeColumnCapture &begin(Runtime *,uint32_t object);
    // A colour readback request at the completion: the capture and column it serves, or
    // nothing when the node is not a registered column node.
    std::optional<std::pair<NativeColumnCapture *,unsigned>> column(uint32_t node,const NativeDepthProbeRequest &,uint32_t destination);
    const NativeColumnCapture *capture(uint32_t object)const{auto it=captures.find(object);return it==captures.end()?nullptr:&it->second;}
    // 4de718: the sprite 4de130 computed (texture: the current cache slot's TEX0, checked).
    void sprite(Runtime *,const R5900Context *);
    // 4de888: the build's end (the draw writer's +8).
    void built(Runtime *,const R5900Context *);
    // 4dded0 / 4ddef4 (delay slot of the 120eb0 call): the column to draw, leased.
    std::pair<const NativeColumnCapture *,unsigned> flush(Runtime *,const R5900Context *,uint32_t pc);
    bool active(uint32_t stack)const{return leases.contains(stack);}
    void retire(Runtime *,const R5900Context *);
    void flushed(Runtime *,uint32_t stack);
    void complete(uint32_t stack);
    // 4ddf24 (delay slot of 4dde60's return, patched to j 4ddb70): the object whose last column
    // flushed, for its completion (4ddb70, the job +90's thunk 5074c0 target; scene completion).
    uint32_t completion(uint32_t stack);
    void cancel(uint32_t stack){leases.erase(stack);}
};
class ColumnCaptureOwner {
public:
    virtual ~ColumnCaptureOwner()=default;
    virtual NativeColumnCaptures &nativeColumnCaptures()=0;
    virtual uint8_t claimNativeColumnImage(uint32_t object)=0;
    // Copies the main frame drawn so far into the capture's image (at the frame's size).
    virtual void copyNativeColumnFrame(NativeColumnCapture &)=0;
    virtual void drawNativeColumn(const NativeColumnCapture &,unsigned column)=0;
};
std::optional<bool> nativeColumnCaptureInstruction(R5900Context *,Runtime *,uint32_t pc);
}

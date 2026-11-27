#pragma once
#include <optional>
#include "bozo/runtime.h"
#include "bozo/state.h"
#include <array>
#include <map>
namespace vp2s {
// One game particle's pending curve integration and ordered record publication.
// Fields are semantic point data; no VU register, flag or accumulator layout.
class NativeParticleUpdates {
    struct Update {uint32_t emitter,point,output,index;std::array<float,4> position,rate,attributes,change,drift;float timestep;unsigned phase=0;};
    struct Completed {uint32_t emitter,point,index;};
    struct Callback {uint32_t emitter,point,index,object,returnAddress;unsigned phase=1;};
    std::map<uint32_t,Update> updates;
    std::map<uint32_t,Completed> completed;
    std::map<uint32_t,Callback> callbacks;
    friend bool nativeParticlePhysicsInstruction(R5900Context *,Runtime *,uint32_t);
    friend bool nativeParticleCallbackInstruction(R5900Context *,Runtime *,uint32_t);
public:
    size_t outstanding()const{return updates.size()+completed.size()+callbacks.size();}
};
class ParticleUpdateOwner {
public:
    virtual ~ParticleUpdateOwner()=default;
    virtual NativeParticleUpdates &nativeParticleUpdates()=0;
};
// Ownership of three explicit quaternion out-parameter publications. Only
// addresses and call order survive a helper return; no arithmetic register bank.
class NativeSquads {
    struct Call {uint32_t output;std::array<uint32_t,4> inputs;unsigned phase=0;bool published=false;};
    std::map<uint32_t,Call> calls;
    friend bool nativeSquadInstruction(R5900Context *,Runtime *,uint32_t);
    friend void qualifySquadSines(Runtime *,R5900Context *,uint32_t);
public:
    size_t outstanding()const{return calls.size();}
};
class SquadOwner {
public:
    virtual ~SquadOwner()=default;
    virtual NativeSquads &nativeSquads()=0;
};
void qualifySquadSines(Runtime *,R5900Context *,uint32_t leafFrame);
// A suspended game motion update, owned by its scalar caller frame. This is
// one semantic position result, not a vector-register or accumulator bank.
class NativeMotion {
    struct Update {uint32_t object;std::array<uint32_t,4> position;};
    std::map<uint32_t,Update> updates;
public:
    void prepare(uint32_t frame,uint32_t object,std::array<uint32_t,4> position);
    std::array<uint32_t,4> finish(uint32_t frame,uint32_t object);
    size_t outstanding()const{return updates.size();}
};
class MotionOwner {
public:
    virtual ~MotionOwner()=default;
    virtual NativeMotion &nativeMotion()=0;
    virtual bool qualifiedMotionCallback(Runtime *,uint32_t)const{return false;}
};
// Game record algorithms shared with the existing native port, through Bozo's
// memory/CPU ABI. No vector bank, microprogram or transport is provided.
bool nativeMathInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
// The chain update 1fd110 (mod001) adds the push SP+4a0 after each collider's +8c, and the force field 1fbad0 writes it only
// in range; 1fd110 never initialises it. User decision 2026-11-25 (maps 32, 47): no push until a field writes one. At
// 1fd114, right after the frame is allocated, the push SP+4a0..4af is zeroed; the original instruction then runs.
std::optional<bool> nativeChainPushInstruction(R5900Context *,Runtime *,uint32_t pc);
// mod028 48f1f0's box (a - b + p, a + b + p) to s2+40 / 50 (native_math.cpp).
bool nativeBattleBox(R5900Context *,Runtime *,uint32_t pc);
// mod028 4905b0's triangle projection and 490630's separating-axis triangle test (native_math.cpp).
bool nativeTriangleProject(R5900Context *,Runtime *,uint32_t pc);
bool nativeTriangleOverlap(R5900Context *,Runtime *,uint32_t pc);
// mod001 1fc180's sphere / capsule pair (native_math.cpp).
bool nativeSpherePair(R5900Context *,Runtime *,uint32_t pc);
// mod001 224590's disc hit test (native_math.cpp).
bool nativeDiscHit(R5900Context *,Runtime *,uint32_t pc);
// mod028 solver sites whose VU registers cross a call or a loop (native_math.cpp).
bool nativeSolverRegisters(R5900Context *,Runtime *,uint32_t pc);
// mod147 533700's colour blend and mod159 533b4c's orthonormal frame (slot 0x532200; native_math.cpp).
bool nativeSlotBlend(R5900Context *,Runtime *,uint32_t pc);
// mod028 490380's ray / sphere terms and 490480's closest points of two segments (native_math.cpp).
bool nativeSegmentPair(R5900Context *,Runtime *,uint32_t pc);
// Bone palettes (skeleton +4c buffer, +50 selector) the game's palette builder 237284 / 2372bc has written completely
// (its last bone) in this run. User decision 2026-11-25 (map 487): a skinned mesh whose palette was never built is not
// drawn. A save state does not carry the set: a palette counts once its builder runs again.
bool nativeBonePaletteBuilt(const Runtime *,uint32_t palette);
void nativeBonePaletteWritten(const Runtime *,uint32_t palette);
// The sub-handler that claimed the site of the last nativeMathInstruction call that returned true (null when
// an inline block of nativeMathInstruction did): the dispatcher's per-site fast path calls it directly.
using NativeMathSite=bool(*)(R5900Context *,Runtime *,uint32_t);
NativeMathSite nativeMathClaimer();
// Native math state that lives between guest instructions (save states).
void serializeMathState(bozo::state::Archive &,const Runtime *);
// Box pair separating-axis test mod001 1f82c0 (stubbed): served at its call from
// the shape-pair dispatch with the vf1..vf9 that object-pair block 1f8aa0 loaded.
bool nativeBoxPairLibrary(R5900Context *,Runtime *,uint32_t target);
// The test itself: rows 0..3 object A, 4..7 object B, scale (A, B), centres and
// half extents (a0, a1, a2, a3). Returns 1 when no axis separates the boxes.
uint32_t nativeBoxPairTest(const std::array<std::array<uint32_t,4>,8> &rows,const std::array<uint32_t,4> &scale,const std::array<uint32_t,4> &ca,
    const std::array<uint32_t,4> &ea,const std::array<uint32_t,4> &cb,const std::array<uint32_t,4> &eb);
// The global light byte +517 seen by model setup 23c0d0 for this model in
// this frame (the VU1 program set choice in builder 23a950). nullopt: unseen.
std::optional<bool> nativeModelLightVariant(const Runtime *,uint32_t model);
// mod001 212950 box/frustum test: the 38 VU stores in listing order for the
// box record (a1, 5 qwords) and frustum record (a0, 5 qwords); defined = the
// lanes the function writes or loads (only those are published).
struct BoxFrustumStores{std::array<std::array<uint32_t,4>,38> words{};std::array<unsigned,38> defined{};};
BoxFrustumStores boxFrustumStores(const std::array<std::array<uint32_t,4>,5> &box,const std::array<std::array<uint32_t,4>,5> &frustum);
}

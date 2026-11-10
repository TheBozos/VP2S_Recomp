#pragma once
#include <optional>
#include "bozo/runtime.h"
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
// The global light byte +517 seen by model setup 23c0d0 for this model in
// this frame (the VU1 program set choice in builder 23a950). nullopt: unseen.
std::optional<bool> nativeModelLightVariant(const Runtime *,uint32_t model);
// mod001 212950 box/frustum test: the 38 VU stores in listing order for the
// box record (a1, 5 qwords) and frustum record (a0, 5 qwords); defined = the
// lanes the function writes or loads (only those are published).
struct BoxFrustumStores{std::array<std::array<uint32_t,4>,38> words{};std::array<unsigned,38> defined{};};
BoxFrustumStores boxFrustumStores(const std::array<std::array<uint32_t,4>,5> &box,const std::array<std::array<uint32_t,4>,5> &frustum);
}

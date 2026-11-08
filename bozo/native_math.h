#pragma once
#include "bozo/runtime.h"
#include <array>
#include <map>
namespace vp2s {
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
}

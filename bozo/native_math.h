#pragma once
#include "bozo/runtime.h"
#include <array>
#include <map>
namespace vp2s {
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

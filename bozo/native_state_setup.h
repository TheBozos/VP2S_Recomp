#pragma once
#include "bozo/runtime.h"
#include <map>
#include <optional>
namespace vp2s {
// Audited constants-only render setup. Native draws carry their own material;
// no device/register state or packet payload is retained here.
class NativeStateSetups {
    struct Proof {uint32_t begin,end,writer=0x35df10,object=0;bool retired=false,flushed=false;};
    std::map<uint32_t,Proof> invocations;
public:
    void beginDefaults(Runtime *,uint32_t stack);
    void beginEmptyChildren(Runtime *,uint32_t stack,uint32_t object);
    void returned(Runtime *,uint32_t stack);
    uint32_t completeEmptyChildren(Runtime *,uint32_t stack);
    void retire(Runtime *,R5900Context *);
    void finish(uint32_t stack);
    bool active(uint32_t stack)const{return invocations.contains(stack);}
    size_t outstanding()const{return invocations.size();}
};
class StateSetupOwner {
public:
    virtual ~StateSetupOwner()=default;
    virtual NativeStateSetups &nativeStateSetups()=0;
};
std::optional<bool> nativeStateSetupInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
}

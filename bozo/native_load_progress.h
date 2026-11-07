#pragma once
#include "bozo/runtime.h"
#include <map>
#include <optional>
namespace vp2s {
// Scheduling UI work for a real pending game-file request. This owns no file
// completion, device counter, timer mode, status bits or interrupt.
class NativeLoadProgress {
    struct Pending {uint32_t worker,destination;uint64_t last,next,period;};
    std::map<uint32_t,Pending> pending;
public:
    void begin(Runtime *,uint32_t stack,uint32_t worker,uint32_t destination,uint64_t now,uint64_t period);
    bool updateDue(Runtime *,uint32_t stack,uint32_t worker,uint32_t destination,uint64_t now);
    void finish(uint32_t stack,uint32_t worker,uint32_t destination,bool requestCompleted);
    size_t pendingCount()const{return pending.size();}
};
class LoadProgressOwner {
public:
    virtual ~LoadProgressOwner()=default;
    virtual NativeLoadProgress &nativeLoadProgress()=0;
};
std::optional<bool> nativeLoadProgressInstruction(R5900Context *,Runtime *,uint32_t pc);
}

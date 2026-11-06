#pragma once
#include <cstdint>
#include <stdexcept>
struct R5900Context;
struct Runtime;
namespace vp2s {
// Per-run SDK time, in bus-clock units with the audited API's 256-unit
// resolution. This is an elapsed-time domain, not a PS2 counter/register bank.
struct TimerClock {
    bool initialized=false, running=false;
    uint64_t epoch=0, base=0;
    uint64_t deadlineSerial=0;
    uint64_t value(uint64_t eeCycle) const {
        if(!initialized) throw std::runtime_error("native SDK timer is not initialized");
        if(running && eeCycle<epoch) throw std::runtime_error("native SDK clock regressed");
        return base+(running?((eeCycle-epoch)/512)*256:0);
    }
};
// One lifted SDK record transition, shared with the original-code fixture.
uint32_t processTimerRecord(uint8_t *,R5900Context *,Runtime *,uint32_t,uint64_t);
class TimerOwner {
public:
    virtual ~TimerOwner()=default;
    virtual TimerClock &timerClock()=0;
};
}

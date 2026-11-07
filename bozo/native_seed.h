#pragma once
#include "bozo/runtime.h"
namespace vp2s {
class SeedOwner {
    uint64_t initializationDraw_=0;
public:
    virtual ~SeedOwner()=default;
    virtual uint32_t bootSeed() const=0;
    // Native initialization jitter, deterministic when the run has a fixed
    // seed. This is an entropy sample, not a counter, clock or device value.
    uint32_t initializationJitter(){
        uint64_t value=uint64_t(bootSeed())+(++initializationDraw_)*0x9e3779b97f4a7c15ull;
        value=(value^(value>>30))*0xbf58476d1ce4e5b9ull;
        value=(value^(value>>27))*0x94d049bb133111ebull;
        return uint32_t((value^(value>>31))>>48);
    }
};
// Audited boot/init entropy producers. They have no timer address/count semantics.
bool nativeBootSeed(uint8_t *,R5900Context *,Runtime *,uint32_t);
}

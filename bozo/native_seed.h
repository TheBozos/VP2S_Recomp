#pragma once
#include "bozo/runtime.h"
namespace vp2s {
class SeedOwner {
public:
    virtual ~SeedOwner()=default;
    virtual uint32_t bootSeed() const=0;
};
// Audited boot RNG input producer. It has no timer address/count semantics.
bool nativeBootSeed(uint8_t *,R5900Context *,Runtime *,uint32_t);
}

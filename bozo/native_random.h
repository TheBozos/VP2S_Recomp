#pragma once
#include "bozo/context.h"
#include <bit>
namespace vp2s {
// The game's procedural random sequence, as ordinary per-run application data.
class ProceduralRandom {
    uint32_t state=0;
public:
    void seed(uint32_t bits){state=bits&0x7fffffu;}
    float current()const{return std::bit_cast<float>(0x3f800000u|state);}
    float next(){
        const auto feedback=std::popcount(state&0x400010u)&1u;
        state=(state*2u+feedback)&0x7fffffu;return current();
    }
};
class RandomOwner {
public:
    virtual ~RandomOwner()=default;
    virtual ProceduralRandom &proceduralRandom()=0;
};
bool nativeRandomSeed(uint8_t *,R5900Context *,Runtime *,uint32_t);
}

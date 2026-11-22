#pragma once
#include "bozo/context.h"
#include "native_state.h"
#include <bit>
#include <stdexcept>
namespace vp2s {
// The game's procedural random sequence, as ordinary per-run application data. It is
// seeded once at boot (mod001 29e1d0 from the global set-up 249914); save states carry it
// (the vp2s section's "random" field). A state older than that field leaves it unknown and
// a draw refuses (VP2S_STATE_RANDOM_SEED upgrades such a state explicitly).
class ProceduralRandom {
    uint32_t state=0;bool known=false;
public:
    void seed(uint32_t bits){state=bits&0x7fffffu;known=true;}
    bool seeded()const{return known;}
    uint32_t bits()const{return state;}
    template<class A>void serialize(A &a){bozo::state::fields(a,state,known);}
    float current()const{return std::bit_cast<float>(0x3f800000u|state);}
    float next(){
        if(!known)throw std::runtime_error("native procedural random stream was not seeded in this run or state");
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
// A consumer of the stream: RNEXT.x vf31 then QMFC2 of vf31 into a GPR at a listed site.
bool nativeRandomNext(uint8_t *,R5900Context *,Runtime *,uint32_t);
}

#pragma once
#include "bozo/runtime.h"
#include <array>
namespace vp2s {
struct TrigCoefficients {
    float pi=0,inversePi=0,halfPi=0;
    std::array<float,6> sine; // x^13, x^11, x^9, x^7, x^5, x^3
};
struct TrigReduction {std::array<int32_t,2> quadrants;std::array<float,2> residual;};
TrigReduction reduceSineCosine(float angle,const TrigCoefficients &);
std::array<float,4> sineCosine(float angle,const TrigCoefficients &);
struct NativeCosine {int32_t quadrant;float value;};
NativeCosine shortCosine(float angle,const TrigCoefficients &);
bool nativeTrigInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
}

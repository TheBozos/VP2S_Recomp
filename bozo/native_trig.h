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
NativeCosine shortSine(float angle,const TrigCoefficients &);
NativeCosine fullSine(float angle,const TrigCoefficients &);
NativeCosine fullCosine(float angle,const TrigCoefficients &);
struct InverseCosineCoefficients {std::array<float,15> values;};
float inverseCosineRoot(float angle,const InverseCosineCoefficients &);
float inverseCosine(float angle,const InverseCosineCoefficients &);
struct ThreeSineReduction {std::array<int32_t,3> quadrants;std::array<float,3> scaled,residual;};
ThreeSineReduction reduceThreeSines(const std::array<float,3> &,const TrigCoefficients &);
std::array<float,3> threeSines(const std::array<float,3> &,const TrigCoefficients &);
std::array<float,3> nativeThreeSines(Runtime *,const std::array<float,3> &);
bool nativeTrigInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
}

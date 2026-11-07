#pragma once
#include "../port/vp2s_math.h"
#include "bozo/runtime.h"
namespace vp2s {
struct ObjectWorldInput {
    __m128 quaternion,position,scale;
    math::RigidBasis parent;
    math::Matrix attachment;
    bool hasParent=false,normalizeParent=false;
};
struct ObjectWorldResult {math::Matrix local,world;};
ObjectWorldResult objectWorld(const ObjectWorldInput &);
bool nativeObjectMathInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
}

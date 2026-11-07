#pragma once
#include "../port/vp2s_math.h"
#include "bozo/runtime.h"
namespace vp2s {
using FrustumRecords=std::array<__m128,10>;
struct FrustumResult {FrustumRecords points,normals;};
FrustumResult cameraFrustum(math::Matrix world,const FrustumRecords &points);
bool nativeFrustumInstruction(R5900Context *,Runtime *,uint32_t);
}

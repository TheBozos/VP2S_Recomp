#pragma once
#include "../port/vp2s_math.h"
#include "bozo/runtime.h"
namespace vp2s {
using FrustumRecords=std::array<__m128,10>;
struct FrustumResult {FrustumRecords points,normals;};
struct NativeSphereVisibility;
NativeSphereVisibility sphereVisibility(math::Matrix clip,const FrustumResult &,__m128 sphere);
uint8_t sphereAngleVisibility(const FrustumResult &,__m128 sphere);
std::array<float,4> objectFacing(const std::array<float,16> &view,const std::array<float,16> &world,const std::array<float,4> &position,const std::array<float,4> &direction);
FrustumResult cameraFrustum(math::Matrix world,const FrustumRecords &points);
bool nativeFrustumInstruction(R5900Context *,Runtime *,uint32_t);
}

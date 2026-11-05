#pragma once
#include "vp2s_music_math.h"

namespace vp2s::math {
using Matrix = std::array<__m128,4>;
// Two column-major inputs: the game's a1/a2 convention produces a2 * a1.
inline Matrix compose(const Matrix &local,const Matrix &parent) {
    music::CodecRounding rounding;
    Matrix result;
    for(unsigned column=0;column<4;++column) {
        const auto v=local[column];
        auto acc=_mm_mul_ps(parent[0],music::broadcast<0>(v));
        acc=music::codecSum(acc,_mm_mul_ps(parent[1],music::broadcast<1>(v)));
        acc=music::codecSum(acc,_mm_mul_ps(parent[2],music::broadcast<2>(v)));
        result[column]=music::codecSum(acc,_mm_mul_ps(parent[3],music::broadcast<3>(v)));
    }
    return result;
}
}
class PS2Runtime;
void vp2sInstallMath(PS2Runtime &runtime);

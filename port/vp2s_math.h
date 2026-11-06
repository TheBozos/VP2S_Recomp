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
inline __m128 normalizeQuaternion(__m128 value) {
    music::CodecRounding rounding;
    const auto squares=_mm_mul_ps(value,value);
    float length2=music::codecSum(music::laneValue(squares,0),music::laneValue(squares,1));
    length2=music::codecSum(length2,music::laneValue(squares,2));
    length2=music::codecSum(length2,music::laneValue(squares,3));
    const float length=std::sqrt(length2);
    const float reciprocal=length!=0.f ? 1.f/length : std::numeric_limits<float>::max();
    return _mm_mul_ps(value,_mm_set1_ps(reciprocal));
}
}
class PS2Runtime;
struct R5900Context;
void vp2sInstallMath(PS2Runtime &runtime);
bool vp2sNormalizeEnabled();
bool vp2sNormalizeReferenceEnabled();
void vp2sNormalizeRecord(uint8_t*,R5900Context*,uint32_t source,uint32_t destination);

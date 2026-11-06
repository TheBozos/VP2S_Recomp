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
// The game accepts the supplied quaternion as-is; normalization is a separate
// operation. Preserve its product and subtraction order, including signed zero.
inline Matrix quaternionMatrix(__m128 value) {
    music::CodecRounding rounding;
    const auto squares=_mm_mul_ps(value,value);
    const auto twice=music::codecSum(value,value);
    const auto diagonal=music::codecSum(squares,squares);
    const auto x=_mm_mul_ps(value,music::broadcast<0>(twice));
    const auto w=_mm_mul_ps(value,music::broadcast<3>(twice));
    const auto y=_mm_mul_ps(value,music::broadcast<1>(twice));
    const auto z=_mm_mul_ps(value,music::broadcast<2>(twice));
    auto lane=[](__m128 v,int n) {return music::laneValue(v,n);};
    auto sum=[](float a,float b) {return music::codecSum(a,b);};
    return {
        _mm_setr_ps(sum(sum(1.f,-lane(diagonal,1)),-lane(diagonal,2)),
                    sum(lane(x,1),lane(w,2)),sum(lane(x,2),-lane(w,1)),0.f),
        _mm_setr_ps(sum(lane(y,0),-lane(w,2)),
                    sum(sum(1.f,-lane(diagonal,0)),-lane(diagonal,2)),sum(lane(y,2),lane(x,3)),0.f),
        _mm_setr_ps(sum(lane(z,0),lane(w,1)),sum(lane(z,1),-lane(x,3)),
                    sum(sum(1.f,-lane(diagonal,0)),-lane(diagonal,1)),0.f),
        _mm_setr_ps(0.f,0.f,0.f,1.f)
    };
}
}
class PS2Runtime;
struct R5900Context;
void vp2sInstallMath(PS2Runtime &runtime);
bool vp2sNormalizeEnabled();
bool vp2sNormalizeReferenceEnabled();
void vp2sNormalizeRecord(uint8_t*,R5900Context*,uint32_t source,uint32_t destination);

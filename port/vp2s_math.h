#pragma once
#include "vp2s_music_math.h"

namespace vp2s::math {
using Matrix = std::array<__m128,4>;
using RigidBasis = std::array<__m128,3>;
// Translation product used by the game's rigid inverse. The fourth component
// belongs to the caller's temporary record and must survive bit-for-bit.
inline __m128 rigidInverseTranslation(const RigidBasis &basis,__m128 translation) {
    auto normal=[](__m128 v) {
        const auto bits=_mm_castps_si128(v);
        const auto tiny=_mm_cmpeq_epi32(_mm_and_si128(bits,_mm_set1_epi32(0x7f800000)),_mm_setzero_si128());
        return _mm_castsi128_ps(_mm_andnot_si128(_mm_and_si128(tiny,_mm_set1_epi32(0x7fffffff)),bits));
    };
    auto product=[&](__m128 row,__m128 scale) {
        return normal(_mm_mul_ps(_mm_blend_ps(row,_mm_setzero_ps(),8),scale));
    };
    auto result=product(basis[0],music::broadcast<0>(translation));
    result=normal(_mm_add_ps(result,product(basis[1],music::broadcast<1>(translation))));
    result=normal(_mm_add_ps(result,product(basis[2],music::broadcast<2>(translation))));
    return _mm_blend_ps(result,translation,8);
}
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
// The attachment routines' EE calculation uses ordinary multiply/adds under
// the caller's floating-point mode, unlike the common matrix microprogram.
// Keep this separate from compose's explicit truncating-sum policy.
inline Matrix composeAttachment(const Matrix &local,const Matrix &parent) {
    Matrix result;
    for(unsigned column=0;column<4;++column) {
        const auto v=local[column];
        auto acc=_mm_mul_ps(parent[0],music::broadcast<0>(v));
        acc=_mm_add_ps(acc,_mm_mul_ps(parent[1],music::broadcast<1>(v)));
        acc=_mm_add_ps(acc,_mm_mul_ps(parent[2],music::broadcast<2>(v)));
        result[column]=_mm_add_ps(acc,_mm_mul_ps(parent[3],music::broadcast<3>(v)));
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
    // Tiny components/products in this game calculation reduce to signed zero.
    // Retaining a subnormal squared term makes 1 - term round one ULP below 1.
    auto normal=[](__m128 v) {
        const auto bits=_mm_castps_si128(v);
        const auto tiny=_mm_cmpeq_epi32(_mm_and_si128(bits,_mm_set1_epi32(0x7f800000)),_mm_setzero_si128());
        return _mm_castsi128_ps(_mm_andnot_si128(_mm_and_si128(tiny,_mm_set1_epi32(0x7fffffff)),bits));
    };
    auto product=[&](__m128 a,__m128 b) {return normal(_mm_mul_ps(a,b));};
    value=normal(value);
    const auto squares=product(value,value);
    const auto twice=normal(music::codecSum(value,value));
    const auto diagonal=normal(music::codecSum(squares,squares));
    const auto x=product(value,music::broadcast<0>(twice));
    const auto w=product(value,music::broadcast<3>(twice));
    const auto y=product(value,music::broadcast<1>(twice));
    const auto z=product(value,music::broadcast<2>(twice));
    auto lane=[](__m128 v,int n) {return music::laneValue(v,n);};
    auto sum=[&](float a,float b) {return _mm_cvtss_f32(normal(music::codecSum(_mm_set1_ps(a),_mm_set1_ps(b))));};
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
bool vp2sInlineQuaternionEnabled();
bool vp2sInlineQuaternionReferenceEnabled();
void vp2sInlineQuaternionMatrix(uint8_t*,R5900Context*,uint32_t source,uint32_t destination,unsigned columns,uint32_t site);
void vp2sInlineQuaternionCompose(uint8_t*,R5900Context*,uint32_t source,uint32_t destination);
bool vp2sRigidInverseEnabled();
bool vp2sRigidInverseReferenceEnabled();
void vp2sRigidInverseTranslation(uint8_t*,R5900Context*,uint32_t basis,uint32_t translation);

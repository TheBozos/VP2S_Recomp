#pragma once
#include "vp2s_music_math.h"
#include <algorithm>

namespace vp2s::math {
using Matrix = std::array<__m128,4>;
using RigidBasis = std::array<__m128,3>;
// The sphere getter's original scale calculation: largest column length of
// the three-row basis, with the game's truncating products/sums and square root.
inline float largestAxisScale(RigidBasis basis){
    for(auto row:basis){uint32_t bits[4];std::memcpy(bits,&row,16);
        for(unsigned i=0;i<3;++i){const auto e=bits[i]&0x7f800000;
            if(e==0x7f800000 || (!e && (bits[i]&0x7fffff)))throw std::runtime_error("unqualified native sphere basis");}}
    const auto normal=[](__m128 v){const auto b=_mm_castps_si128(v);
        const auto tiny=_mm_cmpeq_epi32(_mm_and_si128(b,_mm_set1_epi32(0x7f800000)),_mm_setzero_si128());
        return _mm_castsi128_ps(_mm_andnot_si128(_mm_and_si128(tiny,_mm_set1_epi32(0x7fffffff)),b));};
    music::CodecRounding rounding;
    for(auto &row:basis)row=_mm_blend_ps(row,_mm_setzero_ps(),8);
    auto squared=normal(_mm_mul_ps(basis[0],basis[0]));
    squared=normal(music::codecSum(squared,normal(_mm_mul_ps(basis[1],basis[1]))));
    squared=normal(music::codecSum(squared,normal(_mm_mul_ps(basis[2],basis[2]))));
    float values[4];std::memcpy(values,&squared,16);const auto maximum=std::max(values[0],std::max(values[1],values[2]));
    if(!std::isfinite(maximum))throw std::runtime_error("native sphere scale overflow");
    volatile float length=std::sqrt(maximum);volatile float result=0.f+length;return result;
}
// Remove per-column XYZ scale while preserving the record's translation Ws
// and final row. This is the game's three-column normalization algorithm.
inline Matrix normalizedMatrixColumns(Matrix value) {
    for(unsigned row=0;row<3;++row){
        uint32_t bits[4];std::memcpy(bits,&value[row],16);
        for(unsigned lane=0;lane<3;++lane){const auto exponent=bits[lane]&0x7f800000;
            if(exponent==0x7f800000 || (!exponent && (bits[lane]&0x7fffff)))
                throw std::runtime_error("unqualified matrix-column normalization input");}
    }
    auto normal=[](__m128 v){
        const auto bits=_mm_castps_si128(v);
        const auto tiny=_mm_cmpeq_epi32(_mm_and_si128(bits,_mm_set1_epi32(0x7f800000)),_mm_setzero_si128());
        return _mm_castsi128_ps(_mm_andnot_si128(_mm_and_si128(tiny,_mm_set1_epi32(0x7fffffff)),bits));
    };
    music::CodecRounding rounding;
    Matrix xyz=value;for(unsigned i=0;i<3;++i)xyz[i]=_mm_blend_ps(xyz[i],_mm_setzero_ps(),8);
    auto lengths=normal(_mm_mul_ps(xyz[0],xyz[0]));
    lengths=normal(music::codecSum(lengths,normal(_mm_mul_ps(xyz[1],xyz[1]))));
    lengths=normal(music::codecSum(lengths,normal(_mm_mul_ps(xyz[2],xyz[2]))));
    const auto exponent=_mm_and_si128(_mm_castps_si128(lengths),_mm_set1_epi32(0x7f800000));
    if(_mm_movemask_epi8(_mm_cmpeq_epi32(exponent,_mm_set1_epi32(0x7f800000))))
        throw std::runtime_error("unqualified matrix-column normalization overflow");
    float factors[4]={0,0,0,1};
    for(unsigned i=0;i<3;++i){const float length=std::sqrt(music::laneValue(lengths,i));
        factors[i]=length==0?std::numeric_limits<float>::max():1.f/length;}
    __m128 reciprocal;std::memcpy(&reciprocal,factors,16);
    for(unsigned i=0;i<3;++i)value[i]=_mm_blend_ps(normal(_mm_mul_ps(xyz[i],reciprocal)),value[i],8);
    return value;
}
// The camera builder projects an ordinary point through its transposed screen
// matrix. The result record keeps reciprocal W, as its scalar consumers expect.
inline __m128 cameraProjectedPoint(Matrix screen,__m128 point) {
    auto qualified=[](__m128 value){uint32_t bits[4];std::memcpy(bits,&value,16);
        for(auto b:bits){const auto exponent=b&0x7f800000;
            if(exponent==0x7f800000 || (!exponent && (b&0x7fffff)))
                throw std::runtime_error("unqualified native camera projection input");}};
    for(auto column:screen)qualified(column);qualified(point);
    auto normal=[](__m128 v){
        const auto bits=_mm_castps_si128(v);
        const auto tiny=_mm_cmpeq_epi32(_mm_and_si128(bits,_mm_set1_epi32(0x7f800000)),_mm_setzero_si128());
        return _mm_castsi128_ps(_mm_andnot_si128(_mm_and_si128(tiny,_mm_set1_epi32(0x7fffffff)),bits));
    };
    music::CodecRounding rounding;
    _MM_TRANSPOSE4_PS(screen[0],screen[1],screen[2],screen[3]);
    // The original basis construction adds zero to off-diagonal XYZ entries,
    // multiplies translation W by one, and moves its four diagonal entries.
    for(unsigned i=0;i<4;++i){
        const auto adjusted=normal(_mm_add_ps(_mm_setzero_ps(),screen[i]));
        switch(i){case 0:screen[i]=_mm_blend_ps(adjusted,screen[i],9);break;
                  case 1:screen[i]=_mm_blend_ps(adjusted,screen[i],10);break;
                  case 2:screen[i]=_mm_blend_ps(adjusted,screen[i],12);break;
                  case 3:screen[i]=_mm_blend_ps(adjusted,screen[i],8);break;}
        if(i<3)screen[i]=_mm_blend_ps(screen[i],normal(_mm_mul_ps(screen[i],_mm_set1_ps(1))),8);
    }
    auto result=normal(_mm_mul_ps(screen[0],music::broadcast<0>(point)));
    result=normal(music::codecSum(result,normal(_mm_mul_ps(screen[1],music::broadcast<1>(point)))));
    result=normal(music::codecSum(result,normal(_mm_mul_ps(screen[2],music::broadcast<2>(point)))));
    result=normal(music::codecSum(result,normal(_mm_mul_ps(screen[3],music::broadcast<3>(point)))));
    qualified(result);
    const float w=music::laneValue(result,3);
    if(w==0 || !std::isfinite(w))throw std::runtime_error("unqualified native camera projection divisor");
    volatile float reciprocal=1.f/w;
    return normal(_mm_mul_ps(_mm_blend_ps(result,_mm_set1_ps(1),8),_mm_set1_ps(reciprocal)));
}
// Full record transform: all four input weights and output components matter.
inline __m128 matrixVectorProduct(const Matrix &matrix,__m128 vector) {
    auto normal=[](__m128 v) {
        const auto bits=_mm_castps_si128(v);
        const auto tiny=_mm_cmpeq_epi32(_mm_and_si128(bits,_mm_set1_epi32(0x7f800000)),_mm_setzero_si128());
        return _mm_castsi128_ps(_mm_andnot_si128(_mm_and_si128(tiny,_mm_set1_epi32(0x7fffffff)),bits));
    };
    auto acc=normal(_mm_mul_ps(matrix[0],music::broadcast<0>(vector)));
    acc=normal(_mm_add_ps(acc,normal(_mm_mul_ps(matrix[1],music::broadcast<1>(vector)))));
    acc=normal(_mm_add_ps(acc,normal(_mm_mul_ps(matrix[2],music::broadcast<2>(vector)))));
    return normal(_mm_add_ps(acc,normal(_mm_mul_ps(matrix[3],music::broadcast<3>(vector)))));
}
// Object-update linear transform: xyz weights, all four basis components.
inline __m128 linearVectorProduct(const RigidBasis &basis,__m128 vector) {
    auto normal=[](__m128 v) {
        const auto bits=_mm_castps_si128(v);
        const auto tiny=_mm_cmpeq_epi32(_mm_and_si128(bits,_mm_set1_epi32(0x7f800000)),_mm_setzero_si128());
        return _mm_castsi128_ps(_mm_andnot_si128(_mm_and_si128(tiny,_mm_set1_epi32(0x7fffffff)),bits));
    };
    auto acc=normal(_mm_mul_ps(basis[0],music::broadcast<0>(vector)));
    acc=normal(_mm_add_ps(acc,normal(_mm_mul_ps(basis[1],music::broadcast<1>(vector)))));
    return normal(_mm_add_ps(acc,normal(_mm_mul_ps(basis[2],music::broadcast<2>(vector)))));
}
using PointDirection = std::array<__m128,2>;
inline PointDirection transformPointDirection(const Matrix &matrix,const PointDirection &vectors) {
    return {matrixVectorProduct(matrix,vectors[0]),
            linearVectorProduct(RigidBasis{matrix[0],matrix[1],matrix[2]},vectors[1])};
}
inline __m128 offsetDotRecord(__m128 point,__m128 axis,__m128 origin) {
    auto normal=[](__m128 v) {
        const auto bits=_mm_castps_si128(v);
        const auto tiny=_mm_cmpeq_epi32(_mm_and_si128(bits,_mm_set1_epi32(0x7f800000)),_mm_setzero_si128());
        return _mm_castsi128_ps(_mm_andnot_si128(_mm_and_si128(tiny,_mm_set1_epi32(0x7fffffff)),bits));
    };
    const auto zero=_mm_setzero_ps();
    const auto delta=normal(_mm_sub_ps(_mm_blend_ps(point,zero,8),_mm_blend_ps(origin,zero,8)));
    const auto terms=normal(_mm_mul_ps(delta,_mm_blend_ps(axis,zero,8)));
    const auto xy=normal(_mm_add_ss(terms,music::broadcast<1>(terms)));
    const auto result=normal(_mm_add_ss(xy,music::broadcast<2>(terms)));
    return _mm_move_ss(point,result);
}
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
inline __m128 rigidInverseColumnTranslation(const RigidBasis &basis,__m128 translation) {
    const auto result=rigidInverseTranslation(basis,translation);
    return _mm_xor_ps(result,_mm_castsi128_ps(_mm_setr_epi32(int(0x80000000u),int(0x80000000u),int(0x80000000u),0)));
}
inline __m128 transposedRigidInverseTranslation(Matrix transposed,__m128 translation) {
    _MM_TRANSPOSE4_PS(transposed[0],transposed[1],transposed[2],transposed[3]);
    return rigidInverseTranslation(RigidBasis{transposed[0],transposed[1],transposed[2]},translation);
}
// Ordered record composition for the EE projection builders. Keep their host
// arithmetic and reduction of tiny intermediate results, separately from the
// common matrix microprogram's truncating-sum policy.
inline Matrix projectionProduct(const Matrix &local,const Matrix &view) {
    auto normal=[](__m128 v) {
        const auto bits=_mm_castps_si128(v);
        const auto tiny=_mm_cmpeq_epi32(_mm_and_si128(bits,_mm_set1_epi32(0x7f800000)),_mm_setzero_si128());
        return _mm_castsi128_ps(_mm_andnot_si128(_mm_and_si128(tiny,_mm_set1_epi32(0x7fffffff)),bits));
    };
    Matrix result;
    for(unsigned column=0;column<4;++column) {
        const auto v=local[column];
        auto acc=normal(_mm_mul_ps(view[0],music::broadcast<0>(v)));
        acc=normal(_mm_add_ps(acc,normal(_mm_mul_ps(view[1],music::broadcast<1>(v)))));
        acc=normal(_mm_add_ps(acc,normal(_mm_mul_ps(view[2],music::broadcast<2>(v)))));
        result[column]=normal(_mm_add_ps(acc,normal(_mm_mul_ps(view[3],music::broadcast<3>(v)))));
    }
    return result;
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
    // Materialize before the rounding guard restores the caller's mode.
    // Optimizers can otherwise sink this intrinsic past the MXCSR restore.
    const auto scaled=_mm_mul_ps(value,_mm_set1_ps(reciprocal));
    // Scalar volatile components also work with MSVC's special __m128 type.
    volatile float result[4];
    for(unsigned i=0;i<4;++i)result[i]=music::laneValue(scaled,i);
    return _mm_setr_ps(result[0],result[1],result[2],result[3]);
}
// The game accepts the supplied quaternion as-is; normalization is a separate
// operation. Preserve its product and subtraction order, including signed zero.
inline Matrix quaternionMatrix(__m128 value) {
    // VU0 macro arithmetic: sums and products round to nearest even.
    music::NearestRounding rounding;
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
    const auto twice=normal(_mm_add_ps(value,value));
    const auto diagonal=normal(_mm_add_ps(squares,squares));
    const auto x=product(value,music::broadcast<0>(twice));
    const auto w=product(value,music::broadcast<3>(twice));
    const auto y=product(value,music::broadcast<1>(twice));
    const auto z=product(value,music::broadcast<2>(twice));
    auto lane=[](__m128 v,int n) {return music::laneValue(v,n);};
    auto sum=[&](float a,float b) {return _mm_cvtss_f32(normal(_mm_add_ss(_mm_set_ss(a),_mm_set_ss(b))));};
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
void vp2sRigidInverseColumnTranslation(uint8_t*,R5900Context*,uint32_t basis,uint32_t translation,uint32_t destination);
void vp2sTransposedRigidInverseTranslation(uint8_t*,R5900Context*,uint32_t matrix,uint32_t translation);
bool vp2sOffsetDotEnabled();
bool vp2sOffsetDotReferenceEnabled();
void vp2sOffsetDotRecord(uint8_t*,R5900Context*,uint32_t point,uint32_t axis,uint32_t origin,uint32_t destination);
bool vp2sProjectionProductEnabled();
bool vp2sProjectionProductReferenceEnabled();
void vp2sProjectionProduct(uint8_t*,R5900Context*,uint32_t local,uint32_t view,uint32_t destination,uint32_t site);
bool vp2sLinearVectorEnabled();
bool vp2sLinearVectorReferenceEnabled();
void vp2sLinearVectorProduct(uint8_t*,R5900Context*,uint32_t basis,uint32_t vector,uint32_t destination,uint32_t site);
bool vp2sMatrixVectorEnabled();
bool vp2sMatrixVectorReferenceEnabled();
void vp2sMatrixVectorProduct(uint8_t*,R5900Context*,uint32_t matrix,uint32_t vector,uint32_t destination,uint32_t site);
bool vp2sPointDirectionEnabled();
bool vp2sPointDirectionReferenceEnabled();
void vp2sTransformPointDirection(uint8_t*,R5900Context*,uint32_t matrix,uint32_t point,uint32_t direction);

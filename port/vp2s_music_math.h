// Native music codec: ordinary vectors, coefficient arrays and synthesis history.
// No hardware state, instruction execution, packet input or address-space adapter.
#pragma once
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <immintrin.h>
namespace vp2s::music {
using Vector = __m128;
using Block = std::array<Vector, 256>;
struct CodecRounding {
    unsigned saved = _mm_getcsr();
    CodecRounding() { _mm_setcsr((saved & ~_MM_ROUND_MASK) | _MM_ROUND_TOWARD_ZERO); }
    ~CodecRounding() { _mm_setcsr(saved); }
};
// VU0 vector add/sub/mul round the exact result to nearest even. Restores the
// caller's whole MXCSR, flags included, so inexact results never leak out.
struct NearestRounding {
    unsigned saved = _mm_getcsr();
    NearestRounding() { _mm_setcsr((saved & ~_MM_ROUND_MASK) | _MM_ROUND_NEAREST); }
    ~NearestRounding() { _mm_setcsr(saved); }
};
// The game algorithm truncates the smaller mantissa before each sum. Keep this
// numerical policy without an execution core, register file or pipeline state.
inline Vector codecSum(Vector a, Vector b) {
    __m128i ia=_mm_castps_si128(a), ib=_mm_castps_si128(b), zero=_mm_setzero_si128();
    __m128i ones=_mm_cmpeq_epi32(zero,zero);
    __m128i ea=_mm_srli_epi32(_mm_slli_epi32(ia,1),24), eb=_mm_srli_epi32(_mm_slli_epi32(ib,1),24);
    __m128i da=_mm_sub_epi32(eb,ea), db=_mm_sub_epi32(ea,eb);
    __m128i keep=_mm_or_si128(_mm_cmpeq_epi32(_mm_min_epi32(ea,eb),zero),_mm_set1_epi32(int(0x80000000u)));
    auto aligned=[&](__m128i value,__m128i distance) {
        __m128i mask=_mm_or_si128(_mm_andnot_si128(_mm_cmpgt_epi32(distance,_mm_set1_epi32(23)),
                                   _mm_sllv_epi32(ones,_mm_max_epi32(distance,zero))),keep);
        return _mm_castsi128_ps(_mm_and_si128(value,mask));
    };
    return _mm_add_ps(aligned(ia,da),aligned(ib,db));
}
inline float codecSum(float a,float b) { return _mm_cvtss_f32(codecSum(_mm_set1_ps(a),_mm_set1_ps(b))); }
struct Quantized {
    std::array<int32_t, 4> header;
    std::array<int32_t, 28> bands;
    std::array<std::array<int32_t, 4>, 224> coefficients;
};
struct Tables {
    std::array<Vector, 16> rotations;
    std::array<Vector, 512> inverseQuant;
    std::array<Vector, 256> windowMatrix;
    std::array<Vector, 136> synthesis;
};
inline Vector quantizerRow(const Tables &t,int index,const char *site,bool neighbor=false) {
    // The scale deliberately caps valid magnitudes at 511/512. The interpolation
    // kernel still prefetches the next entry for that exact endpoint, with zero
    // weight. Supply the mathematical endpoint, not a wrapped hardware address.
    if(neighbor && index==512) return _mm_setr_ps(2.f,0.f,0.f,0.f);
    if(index<0 || size_t(index)>=t.inverseQuant.size())
        throw std::out_of_range("music quantizer index "+std::to_string(index)+" at "+site);
    return t.inverseQuant[size_t(index)];
}
template<int mask> Vector lanes(Vector previous, Vector value) {
    const auto bits = _mm_castsi128_ps(_mm_setr_epi32(mask&8?-1:0, mask&4?-1:0, mask&2?-1:0, mask&1?-1:0));
    return _mm_or_ps(_mm_and_ps(bits,value), _mm_andnot_ps(bits,previous));
}
inline uint32_t signsAndZeros(Vector v) {
    float values[4]; _mm_storeu_ps(values, v);
    uint32_t result = 0;
    for (int k=0; k<4; ++k) {
        if (values[k] == 0) result |= 8u >> k;
        if (std::signbit(values[k])) result |= 128u >> k;
    }
    return result;
}
template<int mask> Vector sum(Vector old, Vector a, Vector b, uint32_t *classification=nullptr) {
    Vector v = codecSum(a,b); if(classification) *classification=signsAndZeros(v); return lanes<mask>(old,v);
}
template<int mask> Vector difference(Vector old, Vector a, Vector b, uint32_t *classification=nullptr) {
    Vector v = codecSum(a,_mm_xor_ps(b,_mm_castsi128_ps(_mm_set1_epi32(int(0x80000000u))))); if(classification) *classification=signsAndZeros(v); return lanes<mask>(old,v);
}
template<int mask> Vector product(Vector old, Vector a, Vector b, uint32_t *classification=nullptr) {
    Vector v = _mm_mul_ps(a,b); if(classification) *classification=signsAndZeros(v); return lanes<mask>(old,v);
}
template<int mask> Vector accumulate(Vector old, Vector a, Vector b, Vector acc, uint32_t *classification=nullptr) {
    return sum<mask>(old, acc, _mm_mul_ps(a,b), classification);
}
template<int mask> Vector subtractProduct(Vector old, Vector a, Vector b, Vector acc, uint32_t *classification=nullptr) {
    return difference<mask>(old, acc, _mm_mul_ps(a,b), classification);
}
template<int lane> Vector broadcast(Vector value) { return _mm_shuffle_ps(value,value,_MM_SHUFFLE(lane,lane,lane,lane)); }
inline Vector floats(__m128i v) { return _mm_castsi128_ps(v); }
inline Vector rotate(Vector v) { return _mm_shuffle_ps(v,v,_MM_SHUFFLE(0,3,2,1)); }
inline uint32_t laneBits(Vector v, int c) { uint32_t a[4]; _mm_storeu_si128(reinterpret_cast<__m128i*>(a), _mm_castps_si128(v)); return a[c]; }
inline float laneValue(Vector v, int c) { float a[4]; _mm_storeu_ps(a,v); return a[c]; }
template<int shift,int mask> Vector itof(Vector old, Vector v) {
    return lanes<mask>(old,_mm_mul_ps(_mm_cvtepi32_ps(_mm_castps_si128(v)),_mm_set1_ps(std::ldexp(1.f,-shift))));
}
template<int shift,int mask> Vector ftoi(Vector old, Vector v) {
    return lanes<mask>(old,_mm_castsi128_ps(_mm_cvttps_epi32(_mm_mul_ps(v,_mm_set1_ps(std::ldexp(1.f,shift))))));
}
inline float ratio(float a, float b) {
    return b ? a / b : std::copysign(std::numeric_limits<float>::max(), a * b);
}
void dequantize(const Quantized &input, const Tables &tables, Block &spectrum);
void window(const Block &spectrum, const Tables &tables, Block &result);
void synthesize(const Tables &tables, Block &history, Block &result, float gain);
} // namespace vp2s::music

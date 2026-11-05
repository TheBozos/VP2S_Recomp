#pragma once
#include "vp2s_music_math.h"

namespace vp2s::bootstrap {
using Words = std::array<uint32_t,4>;
struct Tables { Words scale, bias; };
inline Words derive(const Words &seed,const Tables &tables) {
    music::CodecRounding rounding;
    const auto load=[](const Words &v) {
        return _mm_castsi128_ps(_mm_loadu_si128(reinterpret_cast<const __m128i*>(v.data())));
    };
    const __m128 value=music::codecSum(_mm_mul_ps(load(seed),load(tables.scale)),load(tables.bias));
    Words result;
    _mm_storeu_si128(reinterpret_cast<__m128i*>(result.data()),_mm_castps_si128(value));
    return result;
}
}

struct R5900Context;
bool vp2sBootstrapEnabled();
void vp2sBootstrapKey(uint8_t *ram,R5900Context *ctx);
void vp2sInstallBootstrap();

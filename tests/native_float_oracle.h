#pragma once
// Test-only finite float32 rational rounding. No runtime arithmetic, CPU/VU
// execution model or game register bank is used by these numerical oracles.
#include <cstdint>
#include <utility>
#include <stdexcept>
#include <immintrin.h>
namespace vp2s::test {
inline uint64_t rightJam(uint64_t v,unsigned shift){if(!shift)return v;if(shift>=64)return v!=0;return (v>>shift)|((v&((uint64_t(1)<<shift)-1))!=0);}
inline uint32_t rounded(uint32_t sign,int e,uint64_t m){
    if(!m)return sign;while(m>=(uint64_t(1)<<56)){m=rightJam(m,1);++e;}while(m<(uint64_t(1)<<55)){m<<=1;--e;}
    if(e<=0){m=rightJam(m,unsigned(1-e));e=1;}
    uint32_t value=uint32_t(m>>32),fraction=uint32_t(m);const auto mode=_mm_getcsr()&_MM_ROUND_MASK;
    const bool increment=mode==_MM_ROUND_NEAREST?(fraction>0x80000000u || (fraction==0x80000000u && (value&1))):mode==_MM_ROUND_DOWN?(sign && fraction):mode==_MM_ROUND_UP?(!sign && fraction):false;
    if(increment && ++value==0x1000000){value>>=1;++e;}if(e>=255)throw std::runtime_error("finite numerical oracle overflow");if(value<0x800000)return sign;return sign|uint32_t(e<<23)|(value&0x7fffff);
}
inline uint32_t product(uint32_t a,uint32_t b){const auto sign=(a^b)&0x80000000u;const int ea=(a>>23)&255,eb=(b>>23)&255;
    if(!ea || !eb)return sign;return rounded(sign,ea+eb-127,uint64_t((a&0x7fffff)|0x800000)*((b&0x7fffff)|0x800000)<<9);}
inline uint32_t sum(uint32_t a,uint32_t b){
    if((a&0x7fffffff)<(b&0x7fffffff))std::swap(a,b);
    if(!(b&0x7fffffff)){if(a&0x7fffffff)return a;return ((a&b)&0x80000000u)|((_mm_getcsr()&_MM_ROUND_MASK)==_MM_ROUND_DOWN?((a|b)&0x80000000u):0u);}
    int e=(a>>23)&255;const auto distance=unsigned(e)-((b>>23)&255);
    const uint64_t large=uint64_t((a&0x7fffff)|0x800000)<<32,small=rightJam(uint64_t((b&0x7fffff)|0x800000)<<32,distance);
    const auto magnitude=((a^b)&0x80000000u)?large-small:large+small;
    if(!magnitude)return (_mm_getcsr()&_MM_ROUND_MASK)==_MM_ROUND_DOWN?0x80000000u:0u;return rounded(a&0x80000000u,e,magnitude);
}
}

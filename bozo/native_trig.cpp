#include "native_trig.h"
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <immintrin.h>
#include <stdexcept>
namespace vp2s {
namespace {
float normal(float v){const auto b=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((b&0x7f800000)?b:b&0x80000000);}
float product(float a,float b){return normal(_mm_cvtss_f32(_mm_mul_ss(_mm_set_ss(a),_mm_set_ss(b))));}
float sum(float a,float b){return normal(_mm_cvtss_f32(_mm_add_ss(_mm_set_ss(a),_mm_set_ss(b))));}
float difference(float a,float b){return normal(_mm_cvtss_f32(_mm_sub_ss(_mm_set_ss(a),_mm_set_ss(b))));}
void qualified(float v){const auto bits=std::bit_cast<uint32_t>(v),e=bits&0x7f800000;
    if(e==0x7f800000 || (!e && (bits&0x7fffff)))throw std::runtime_error("unqualified native trigonometric input/coefficient");}
uint8_t *record(Runtime *rt,uint32_t at,uint32_t bytes){
    if(!rt || (at&15))throw std::runtime_error("unaligned native trigonometric record");
    const bool scratch=at>=bozo::kScratchpadBase && at<bozo::kScratchpadBase+bozo::kScratchpadBytes;
    if(!scratch && ((at&0x1ffffff)<0x100000 || (at>>28)==7))throw std::runtime_error("invalid native trigonometric record");
    auto *p=rt->memory.hostPointer(at,bytes);if(!p)throw std::runtime_error("native trigonometric record outside owned memory");return p;
}
TrigCoefficients coefficients(Runtime *rt){
    float table[16];std::memcpy(table,record(rt,0x35da70,64),64);TrigCoefficients c;
    c.pi=table[0];c.inversePi=table[1];c.halfPi=table[10];
    c.sine={table[13],table[12],table[5],table[4],table[3],table[2]};return c;
}
constexpr std::array<uint32_t,30> sites{
    0x213fe0,0x213fe4,0x213fe8,0x213fec,0x213ff8,0x213ffc,0x214000,0x214004,
    0x214008,0x21400c,0x214010,0x214014,0x214018,0x214058,0x2140b0,0x2140b4,
    0x2140b8,0x2140bc,0x2140c0,0x2140c4,0x2140c8,0x2140cc,0x2140d0,0x2140d4,
    0x2140d8,0x2140dc,0x2140e0,0x2140e4,0x2140e8,0x2140f0};
}
TrigReduction reduceSineCosine(float angle,const TrigCoefficients &c){
    qualified(angle);qualified(c.pi);qualified(c.inversePi);qualified(c.halfPi);
    TrigReduction result;const float arguments[2]={difference(c.halfPi,angle),angle};
    for(unsigned i=0;i<2;++i){
        const float scaled=product(arguments[i],c.inversePi);
        if(!std::isfinite(scaled) || scaled>=2147483648.f || scaled< -2147483648.f)
            throw std::runtime_error("unqualified native trigonometric reduction range");
        const int32_t n=_mm_cvttss_si32(_mm_set_ss(scaled));result.quadrants[i]=n;
        const float integer=_mm_cvtss_f32(_mm_cvtsi32_ss(_mm_setzero_ps(),n));
        result.residual[i]=difference(arguments[i],product(integer,c.pi));
    }return result;
}
std::array<float,4> sineCosine(float angle,const TrigCoefficients &c){
    for(float v:c.sine)qualified(v);const auto reduced=reduceSineCosine(angle,c);
    std::array<float,4> out{c.sine[3],c.sine[2],0,0};
    for(unsigned i=0;i<2;++i){
        const float r=reduced.residual[i],square=product(r,r);
        float polynomial=sum(product(square,c.sine[0]),c.sine[1]);
        for(unsigned k=2;k<6;++k)polynomial=sum(product(polynomial,square),c.sine[k]);
        polynomial=sum(product(polynomial,square),1.f);
        const float sign=(uint32_t(reduced.quadrants[i])&1)?-1.f:1.f;
        out[i+2]=product(polynomial,product(r,sign));
        if(!std::isfinite(out[i+2]))throw std::runtime_error("unqualified native trigonometric polynomial overflow");
    }return out;
}
bool nativeTrigInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc){
    if(!std::binary_search(sites.begin(),sites.end(),pc))return false;
    // This complete leaf has no calls/checkpoints. Interior carved entries
    // cannot supply an inherited vector calculation as a native invocation.
    if(!c || c->pc!=0x213fd0)throw std::runtime_error("unported interior trigonometric invocation");
    if(pc==0x214014){
        const auto reduced=reduceSineCosine(c->f[12],coefficients(rt));
        auto *indices=record(rt,0x35db00,16);
        // Only Z/W are CPU quadrant results. The original full vector store's
        // inherited X/Y scratch payload has no producer/consumer in this leaf;
        // retain the EE bytes rather than invent or shadow vector state.
        std::memcpy(indices+8,reduced.quadrants.data(),8);
    }else if(pc==0x2140f0){
        const auto result=sineCosine(c->f[12],coefficients(rt));
        std::memcpy(record(rt,c->r[4].uw[0],16),result.data(),16);
    }return true;
}
}

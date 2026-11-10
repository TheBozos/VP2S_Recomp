#include "native_object_math.h"
#include <algorithm>
#include <array>
#include <cstring>
#include <format>
#include <limits>
#include <stdexcept>
namespace vp2s {
namespace {
__m128 normal(__m128 v){
    const auto bits=_mm_castps_si128(v);
    const auto tiny=_mm_cmpeq_epi32(_mm_and_si128(bits,_mm_set1_epi32(0x7f800000)),_mm_setzero_si128());
    return _mm_castsi128_ps(_mm_andnot_si128(_mm_and_si128(tiny,_mm_set1_epi32(0x7fffffff)),bits));
}
__m128 product(__m128 a,__m128 b){return normal(_mm_mul_ps(a,b));}
__m128 sum(__m128 a,__m128 b){return normal(music::codecSum(a,b));}
void qualified(__m128 v,const char *field,unsigned mask=15){
    alignas(16) uint32_t bits[4];std::memcpy(bits,&v,16);
    for(unsigned i=0;i<4;++i)if(mask&(1u<<i)){
        const auto exponent=bits[i]&0x7f800000;
        if(exponent==0x7f800000)throw std::runtime_error(std::format(
            "unqualified exponent-255 object-transform input {} lane {} ({:08x} {:08x} {:08x} {:08x})",field,i,bits[0],bits[1],bits[2],bits[3]));
    }
}
void finite(__m128 v){
    const auto exponent=_mm_and_si128(_mm_castps_si128(v),_mm_set1_epi32(0x7f800000));
    if(_mm_movemask_epi8(_mm_cmpeq_epi32(exponent,_mm_set1_epi32(0x7f800000))))throw std::runtime_error("unqualified object-transform arithmetic overflow");
}
uint8_t *record(Runtime *rt,uint32_t at,uint32_t bytes){
    if(!rt || (at&15))throw std::runtime_error("unaligned native object transform record");
    const bool scratch=at>=bozo::kScratchpadBase && at<bozo::kScratchpadBase+bozo::kScratchpadBytes;
    if(!scratch && ((at&0x1ffffff)<0x100000 || (at>>28)==7))throw std::runtime_error("invalid native object transform record");
    auto *p=rt->memory.hostPointer(at,bytes);if(!p)throw std::runtime_error("native object transform outside owned memory");return p;
}
constexpr std::array<uint32_t,64> sites{
    0x249044,0x24904c,0x249054,0x24905c,0x2490c0,0x2490c4,0x2490c8,0x2490d4,
    0x249114,0x24911c,0x249120,0x249124,0x249128,0x24913c,0x249140,0x249144,
    0x249148,0x249150,0x249154,0x249158,0x24915c,0x249160,0x249164,0x249168,
    0x24916c,0x249170,0x249174,0x249178,0x24917c,0x249180,0x249184,0x249188,
    0x24918c,0x249190,0x249194,0x249198,0x2491a0,0x2491d0,0x2491d4,0x2491d8,
    0x2491e8,0x2491ec,0x2491f0,0x2491f4,0x2491fc,0x249200,0x249204,0x249208,
    0x24920c,0x249210,0x249214,0x249218,0x24921c,0x249220,0x249224,0x249228,
    0x24922c,0x249230,0x249234,0x249238,0x24923c,0x249240,0x249244,0x249248};
}
// VU0 reads a denormal operand as signed zero, so denormal input words are
// flushed before any host arithmetic (mod001 249180 met an authored
// near-identity quaternion {-9e-22, 1e-21, 80000554, 1}). Exponent-255 words
// are PS2 numbers the host cannot represent and still reject.
ObjectWorldResult objectWorld(const ObjectWorldInput &original){
    ObjectWorldInput in=original;
    in.quaternion=normal(in.quaternion);in.position=normal(in.position);in.scale=normal(in.scale);
    for(auto &v:in.attachment)v=normal(v);
    for(auto &v:in.parent)v=normal(v);
    qualified(in.quaternion,"quaternion");qualified(in.position,"position",7);qualified(in.scale,"scale",7);
    for(auto v:in.attachment)qualified(v,"attachment row");
    if(in.hasParent)for(auto v:in.parent)qualified(v,"parent row");
    auto rotation=math::quaternionMatrix(in.quaternion);
    for(unsigned i=0;i<3;++i)rotation[i]=_mm_blend_ps(rotation[i],_mm_set1_ps(music::laneValue(in.position,i)),8);
    const auto scale=_mm_blend_ps(in.scale,_mm_set1_ps(1),8);
    ObjectWorldResult out{};
    if(in.hasParent){
        {
            music::CodecRounding rounding;
            for(unsigned i=0;i<3;++i){
                auto row=product(rotation[0],music::broadcast<0>(in.parent[i]));
                row=sum(row,product(rotation[1],music::broadcast<1>(in.parent[i])));
                row=sum(row,product(rotation[2],music::broadcast<2>(in.parent[i])));
                row=sum(row,product(rotation[3],music::broadcast<3>(in.parent[i])));
                out.local[i]=product(scale,row);finite(out.local[i]);
            }
        }
        if(in.normalizeParent){
            {
                music::CodecRounding rounding;
                const auto a=_mm_blend_ps(out.local[0],_mm_setzero_ps(),8);
                const auto b=_mm_blend_ps(out.local[1],_mm_setzero_ps(),8);
                const auto c=_mm_blend_ps(out.local[2],_mm_setzero_ps(),8);
                auto length2=product(a,a);
                length2=sum(length2,product(b,b));
                length2=sum(length2,product(c,c));finite(length2);
                float reciprocal[4]={0,0,0,1};
                for(unsigned i=0;i<3;++i){const float length=std::sqrt(music::laneValue(length2,i));
                    reciprocal[i]=length==0?std::numeric_limits<float>::max():1.f/length;}
                __m128 factors;std::memcpy(&factors,reciprocal,16);
                for(unsigned i=0;i<3;++i){const auto xyz=product(out.local[i],factors);
                    out.local[i]=_mm_blend_ps(xyz,out.local[i],8);}
            }
            // These products are inline game arithmetic after the normalization
            // microprogram returned, under the caller's FP mode.
            for(unsigned i=0;i<3;++i)out.local[i]=product(out.local[i],scale);
        }
    }else for(unsigned i=0;i<3;++i)out.local[i]=product(rotation[i],scale);
    out.local[3]=_mm_setr_ps(0,0,0,1);
    for(unsigned i=0;i<3;++i){finite(out.local[i]);out.world[i]=math::matrixVectorProduct(in.attachment,out.local[i]);finite(out.world[i]);}
    out.world[3]=in.attachment[3];return out;
}
bool nativeObjectMathInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc){
    if(!std::binary_search(sites.begin(),sites.end(),pc))return false;
    if(pc==0x249114){c->r[9].ud[0]=0;return true;}
    if(pc!=0x249180 && pc!=0x24922c)return true;
    const uint32_t object=c->r[16].uw[0],parent=c->r[17].uw[0];
    if(bool(parent)!=(pc==0x249180))throw std::runtime_error("native object transform parent branch mismatch");
    auto *source=record(rt,object,0x210);ObjectWorldInput in;
    std::memcpy(&in.position,source+0x20,16);std::memcpy(&in.quaternion,source+0x30,16);std::memcpy(&in.scale,source+0x40,16);
    std::memcpy(in.attachment.data(),source+0x110,64);in.hasParent=parent!=0;
    uint16_t flags;std::memcpy(&flags,source+0x20c,2);in.normalizeParent=(flags&0x1c0)==0;
    if(parent)std::memcpy(in.parent.data(),record(rt,parent+0xd0,48),48);
    // Snapshot every input before writing either output; self-parent and
    // attachment/output aliases cannot observe partially written matrices.
    const auto out=objectWorld(in);
    std::memcpy(source+0xd0,out.local.data(),64);std::memcpy(source+0x90,out.world.data(),64);return true;
}
}

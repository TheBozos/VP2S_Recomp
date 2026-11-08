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
constexpr std::array<uint32_t,24> cosineSites{
    0x214310,0x214314,0x214318,0x214324,0x214328,0x21432c,0x214330,0x214334,
    0x214338,0x21433c,0x214360,0x214388,0x214390,0x214394,0x214398,0x2143a4,
    0x2143a8,0x2143ac,0x2143b0,0x2143b4,0x2143b8,0x2143bc,0x2143c0,0x2143c4};
constexpr std::array<uint32_t,22> shortSineSites{
    0x2144d0,0x2144d4,0x2144e0,0x2144e4,0x2144e8,0x2144ec,0x2144f0,0x2144f4,
    0x214518,0x214540,0x214548,0x21454c,0x214550,0x21455c,0x214560,0x214564,
    0x214568,0x21456c,0x214570,0x214574,0x214578,0x21457c};
constexpr std::array<uint32_t,27> fullSineSites{
    0x2143f0,0x2143f4,0x2143f8,0x214404,0x214408,0x21440c,0x214410,0x214414,0x214418,
    0x21443c,0x214460,0x214468,0x21446c,0x214470,0x21447c,0x214480,0x214484,0x214488,
    0x21448c,0x214490,0x214494,0x214498,0x21449c,0x2144a0,0x2144a4,0x2144a8,0x2144ac};
constexpr std::array<uint32_t,29> fullCosineSites{
    0x214220,0x214224,0x214228,0x21422c,0x214238,0x21423c,0x214240,0x214244,0x214248,0x21424c,
    0x214250,0x214274,0x214298,0x2142a0,0x2142a4,0x2142a8,0x2142b4,0x2142b8,0x2142bc,0x2142c0,
    0x2142c4,0x2142c8,0x2142cc,0x2142d0,0x2142d4,0x2142d8,0x2142dc,0x2142e0,0x2142e4};
bool inverseCosineSite(uint32_t pc){
    return !(pc&3) && ((pc>=0x214ea8 && pc<=0x214eb4) || (pc>=0x214ef0 && pc<=0x214f5c) ||
        (pc>=0x214f7c && pc<=0x214ff8) || (pc>=0x215010 && pc<=0x215060) || pc==0x215074 || pc==0x215078 ||
        pc==0x215094 || (pc>=0x2150a0 && pc<=0x2150dc));
}
void qualifiedInverseCosine(const InverseCosineCoefficients &c){
    for(float value:c.values){qualified(value);if(std::abs(value)>4.f)throw std::runtime_error("unsupported inverse-cosine coefficient domain");}
    if(std::bit_cast<uint32_t>(c.values[13])!=0x3f000000 || std::bit_cast<uint32_t>(c.values[14])!=0x40000000)
        throw std::runtime_error("inverse-cosine literal coefficients mismatch");
}
float inverseCosineRatio(float z,const InverseCosineCoefficients &c){
    const auto &v=c.values;float numerator=sum(product(z,v[8]),v[7]),denominator=sum(product(z,v[12]),v[11]);
    numerator=sum(product(numerator,z),v[6]);denominator=sum(product(denominator,z),v[10]);
    numerator=sum(product(numerator,z),v[5]);denominator=sum(product(denominator,z),v[9]);
    numerator=sum(product(numerator,z),v[4]);denominator=sum(product(denominator,z),1.f);
    numerator=sum(product(numerator,z),v[3]);numerator=product(numerator,z);
    if(denominator==0)throw std::runtime_error("unqualified inverse-cosine polynomial divisor");
    volatile float ratio=numerator/denominator;return normal(ratio);
}
float inverseCosineSquare(float angle,const InverseCosineCoefficients &c){
    return std::signbit(angle)?sum(c.values[13],product(angle,c.values[13])):difference(c.values[13],product(angle,c.values[13]));
}
bool threeSineSite(uint32_t pc){
    return pc==0x215104 || pc==0x21510c || pc==0x215114 || pc==0x215118 || pc==0x215120 || pc==0x2151b8 || pc==0x2151bc || pc==0x2151c4;
}
void qualifiedThreeSines(const TrigCoefficients &c){
    constexpr std::array<uint32_t,6> expected{0x2f2ec9d3,0xb2d72f34,0x3638ef1b,0xb9500d01,0x3c088889,0xbe2aaaab};
    if(std::bit_cast<uint32_t>(c.pi)!=0x40490fdb || std::bit_cast<uint32_t>(c.inversePi)!=0x3ea2f983)
        throw std::runtime_error("three-sine reduction coefficients mismatch");
    for(unsigned i=0;i<6;++i)if(std::bit_cast<uint32_t>(c.sine[i])!=expected[i])throw std::runtime_error("three-sine polynomial coefficients mismatch");
}
}
ThreeSineReduction reduceThreeSines(const std::array<float,3> &angles,const TrigCoefficients &c){
    qualifiedThreeSines(c);ThreeSineReduction result;
    for(unsigned i=0;i<3;++i){qualified(angles[i]);const auto scaled=product(angles[i],c.inversePi);
        if(!std::isfinite(scaled) || scaled>=2147483648.f || scaled< -2147483648.f)throw std::runtime_error("unqualified three-sine reduction range");
        const auto quadrant=_mm_cvttss_si32(_mm_set_ss(scaled));const auto integer=_mm_cvtss_f32(_mm_cvtsi32_ss(_mm_setzero_ps(),quadrant));
        const auto residual=difference(sum(0.f,angles[i]),product(integer,c.pi));
        if(!std::isfinite(residual) || std::abs(residual)>4.f)throw std::runtime_error("unqualified three-sine residual range");
        result.quadrants[i]=quadrant;result.scaled[i]=scaled;result.residual[i]=residual;
    }return result;
}
std::array<float,3> threeSines(const std::array<float,3> &angles,const TrigCoefficients &c){
    const auto reduced=reduceThreeSines(angles,c);std::array<float,3> result;
    for(unsigned i=0;i<3;++i){const auto square=product(reduced.residual[i],reduced.residual[i]);
        // The audited game program starts with its pre-reduction scaled
        // argument here, rather than the usual square of the residual.
        auto polynomial=sum(product(reduced.scaled[i],c.sine[0]),c.sine[1]);
        for(unsigned k=2;k<6;++k)polynomial=sum(product(polynomial,square),c.sine[k]);
        const auto signedResidual=product(reduced.residual[i],(uint32_t(reduced.quadrants[i])&1)?-1.f:1.f);
        polynomial=sum(product(polynomial,square),1.f);result[i]=product(polynomial,signedResidual);
    }return result;
}
std::array<float,3> nativeThreeSines(Runtime *rt,const std::array<float,3> &angles){return threeSines(angles,coefficients(rt));}
float inverseCosineRoot(float angle,const InverseCosineCoefficients &c){
    const auto bits=std::bit_cast<uint32_t>(angle);
    if(bits<0x3f000000 || bits>=0x3f800000)throw std::runtime_error("unqualified positive inverse-cosine root input");
    qualifiedInverseCosine(c);const auto z=inverseCosineSquare(angle,c);
    volatile float root=static_cast<float>(std::sqrt(std::abs(double(z))));return normal(root);
}
float inverseCosine(float angle,const InverseCosineCoefficients &c){
    const auto bits=std::bit_cast<uint32_t>(angle),magnitude=bits&0x7fffffff;
    // These integer-domain endpoint/tiny decisions are the game's scalar code,
    // including its signed out-of-range bit patterns. No library acos replaces it.
    if(magnitude>=0x3f800000)return bits>>31?std::bit_cast<float>(0x40490fdbu):0.f;
    if(magnitude<0x23000001)return std::bit_cast<float>(0x3fc90fdbu);
    qualified(angle);qualifiedInverseCosine(c);const auto &v=c.values;
    if(magnitude<0x3f000000){const auto ratio=inverseCosineRatio(product(angle,angle),c);
        return difference(v[1],difference(angle,difference(v[2],product(angle,ratio))));}
    const auto z=inverseCosineSquare(angle,c),ratio=inverseCosineRatio(z,c);
    volatile float root=static_cast<float>(std::sqrt(std::abs(double(z))));const auto s=normal(root);
    if(bits>>31){const auto correction=difference(product(s,ratio),v[2]);return difference(v[0],product(sum(correction,s),v[14]));}
    const auto split=std::bit_cast<float>(std::bit_cast<uint32_t>(s)&0xfffff000u),divisor=sum(s,split);
    if(divisor==0)throw std::runtime_error("unqualified inverse-cosine split divisor");
    volatile float correction=difference(z,product(split,split))/divisor;
    const auto residual=sum(product(s,ratio),normal(correction));return sum(product(v[14],split),product(v[14],residual));
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
NativeCosine shortCosine(float angle,const TrigCoefficients &c){
    qualified(angle);qualified(c.pi);qualified(c.inversePi);qualified(c.halfPi);
    for(unsigned i=2;i<6;++i)qualified(c.sine[i]);
    const auto argument=difference(c.halfPi,angle),scaled=product(argument,c.inversePi);
    if(!std::isfinite(scaled) || scaled>=2147483648.f || scaled< -2147483648.f)
        throw std::runtime_error("unqualified native short cosine reduction range");
    const auto quadrant=_mm_cvttss_si32(_mm_set_ss(scaled));
    const auto integer=_mm_cvtss_f32(_mm_cvtsi32_ss(_mm_setzero_ps(),quadrant));
    const auto r=difference(argument,product(integer,c.pi)),square=product(r,r);
    float p=sum(product(square,c.sine[2]),c.sine[3]);
    p=sum(product(p,square),c.sine[4]);p=sum(product(p,square),c.sine[5]);
    p=sum(product(p,square),1.f);
    const auto result=product(p,product(r,(uint32_t(quadrant)&1)?-1.f:1.f));
    if(!std::isfinite(result))throw std::runtime_error("unqualified native short cosine polynomial overflow");
    return {quadrant,result};
}
NativeCosine shortSine(float angle,const TrigCoefficients &c){
    qualified(angle);qualified(c.pi);qualified(c.inversePi);
    for(unsigned i=2;i<6;++i)qualified(c.sine[i]);
    const auto scaled=product(angle,c.inversePi);
    if(!std::isfinite(scaled) || scaled>=2147483648.f || scaled< -2147483648.f)
        throw std::runtime_error("unqualified native short sine reduction range");
    const auto quadrant=_mm_cvttss_si32(_mm_set_ss(scaled));
    const auto integer=_mm_cvtss_f32(_mm_cvtsi32_ss(_mm_setzero_ps(),quadrant));
    const auto r=difference(angle,product(integer,c.pi)),square=product(r,r);
    float p=sum(product(square,c.sine[2]),c.sine[3]);
    p=sum(product(p,square),c.sine[4]);p=sum(product(p,square),c.sine[5]);
    p=sum(product(p,square),1.f);
    const auto result=product(p,product(r,(uint32_t(quadrant)&1)?-1.f:1.f));
    if(!std::isfinite(result))throw std::runtime_error("unqualified native short sine polynomial overflow");
    return {quadrant,result};
}
NativeCosine fullSine(float angle,const TrigCoefficients &c){
    qualified(angle);qualified(c.pi);qualified(c.inversePi);for(float v:c.sine)qualified(v);
    const auto scaled=product(angle,c.inversePi);
    if(!std::isfinite(scaled) || scaled>=2147483648.f || scaled< -2147483648.f)
        throw std::runtime_error("unqualified native full sine reduction range");
    const auto quadrant=_mm_cvttss_si32(_mm_set_ss(scaled));
    const auto integer=_mm_cvtss_f32(_mm_cvtsi32_ss(_mm_setzero_ps(),quadrant));
    const auto r=difference(angle,product(integer,c.pi)),square=product(r,r);
    float p=sum(product(square,c.sine[0]),c.sine[1]);
    for(unsigned i=2;i<6;++i)p=sum(product(p,square),c.sine[i]);
    p=sum(product(p,square),1.f);
    const auto result=product(p,product(r,(uint32_t(quadrant)&1)?-1.f:1.f));
    if(!std::isfinite(result))throw std::runtime_error("unqualified native full sine polynomial overflow");
    return {quadrant,result};
}
NativeCosine fullCosine(float angle,const TrigCoefficients &c){
    qualified(angle);qualified(c.halfPi);qualified(c.pi);qualified(c.inversePi);for(float v:c.sine)qualified(v);
    return fullSine(difference(c.halfPi,angle),c);
}
bool nativeTrigInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc){
    if(std::binary_search(fullCosineSites.begin(),fullCosineSites.end(),pc)){
        if(!c || c->pc!=0x214210)throw std::runtime_error("unported interior full cosine invocation");
        if(pc==0x21424c || pc==0x2142e4){auto table=coefficients(rt);std::memcpy(&table.halfPi,record(rt,0x35da90,16)+12,4);
            const auto result=fullCosine(c->f[12],table);
            if(pc==0x21424c)std::memcpy(record(rt,0x35dae0,16)+12,&result.quadrant,4);
            else std::memcpy(record(rt,0x35da80,16)+12,&result.value,4);}
        return true;
    }
    if(std::binary_search(fullSineSites.begin(),fullSineSites.end(),pc)){
        if(!c || c->pc!=0x2143e0)throw std::runtime_error("unported interior full sine invocation");
        if(pc==0x214414 || pc==0x2144ac){const auto result=fullSine(c->f[12],coefficients(rt));
            if(pc==0x214414)std::memcpy(record(rt,0x35dac0,16)+12,&result.quadrant,4);
            else std::memcpy(record(rt,0x35da80,16)+12,&result.value,4);}
        // The temporary scalar sign store is overwritten by the final original
        // record. Its XYZ coefficients remain untouched; only W is published.
        return true;
    }
    if(threeSineSite(pc)){
        if(!c || c->pc!=0x2150f0 || (c->r[31].uw[0]!=0x2152ec && c->r[31].uw[0]!=0x215324))
            throw std::runtime_error("unported independent three-sine return contract");
        auto *frame=record(rt,c->r[29].uw[0],0x20);
        const auto separate=[&](uint8_t *p,size_t bytes){const auto a=reinterpret_cast<uintptr_t>(frame),b=reinterpret_cast<uintptr_t>(p);
            if(a<b+bytes && b<a+0x20)throw std::runtime_error("three-sine frame aliases caller operands/coefficients");};
        for(unsigned reg:{16u,17u,18u})separate(record(rt,c->r[reg].uw[0],16),16);
        separate(record(rt,0x35da70,64),64);
        if(pc==0x215118){const std::array<float,3> angles{c->f[12],c->f[13],c->f[14]};const auto result=reduceThreeSines(angles,coefficients(rt));
            if(c->r[4].uw[0]!=c->r[29].uw[0])throw std::runtime_error("three-sine quadrant destination mismatch");
            std::memcpy(frame,result.quadrants.data(),12);
        }
        // Preserve original scalar quadrant/parity/sign stores. Their unused
        // W and control-read writes retire. The caller derives sine weights
        // directly from the scalar arguments that this helper preserves.
        return true;
    }
    if(inverseCosineSite(pc)){
        if(!c || c->pc!=0x214e40)throw std::runtime_error("unported interior inverse-cosine invocation");
        const auto frame=reinterpret_cast<uintptr_t>(record(rt,c->r[29].uw[0],16)),tableAddress=reinterpret_cast<uintptr_t>(record(rt,0x35da30,64));
        if(frame<tableAddress+64 && tableAddress<frame+16)throw std::runtime_error("inverse-cosine caller frame aliases its coefficient record");
        if(pc==0x214f5c || pc==0x214ff8 || pc==0x215050 || pc==0x2150dc){
            const auto bits=std::bit_cast<uint32_t>(c->f[12]),magnitude=bits&0x7fffffff;
            const bool small=magnitude<0x3f000000,negative=bits>>31;
            if(magnitude>=0x3f800000 || magnitude<0x23000001 || (pc==0x214f5c?!small:small || (pc==0x214ff8?!negative:negative)))
                throw std::runtime_error("inverse-cosine scalar branch mismatch");
            InverseCosineCoefficients table;std::memcpy(table.values.data(),record(rt,0x35da30,64),60);
            float result;
            if(pc==0x215050)result=inverseCosineRoot(c->f[12],table);
            else {
                if(pc==0x2150dc){const auto root=inverseCosineRoot(c->f[12],table);uint32_t split;
                    std::memcpy(&split,record(rt,0x35da60,16)+12,4);
                    if(split!=(std::bit_cast<uint32_t>(root)&0xfffff000u))throw std::runtime_error("inverse-cosine split-root publication mismatch");}
                result=inverseCosine(c->f[12],table);
            }
            // XYZ are unchanged coefficient payloads in every full source
            // store. Only W publishes the root or polynomial result.
            std::memcpy(record(rt,0x35da60,16)+12,&result,4);
        }return true;
    }
    if(std::binary_search(shortSineSites.begin(),shortSineSites.end(),pc)){
        if(!c || c->pc!=0x2144c0)throw std::runtime_error("unported interior short sine invocation");
        if(pc==0x2144f0 || pc==0x21457c){
            const auto result=shortSine(c->f[12],coefficients(rt));
            if(pc==0x2144f0)std::memcpy(record(rt,0x35dab0,16)+12,&result.quadrant,4);
            else std::memcpy(record(rt,0x35da80,16)+12,&result.value,4);
        }
        // Retire the temporary scalar sign store too: the final original store
        // restored the old Z coefficient. The native polynomial owns the sign.
        return true;
    }
    if(std::binary_search(cosineSites.begin(),cosineSites.end(),pc)){
        if(!c || c->pc!=0x214300)throw std::runtime_error("unported interior short cosine invocation");
        if(pc==0x214338 || pc==0x2143c4){
            auto table=coefficients(rt);std::memcpy(&table.halfPi,record(rt,0x35da90,16)+12,4);
            const auto result=shortCosine(c->f[12],table);
            if(pc==0x214338)std::memcpy(record(rt,0x35dad0,16)+12,&result.quadrant,4);
            else std::memcpy(record(rt,0x35da80,16)+12,&result.value,4);
        }
        // The temporary scalar sign store at 214388 is consumed solely by
        // this leaf's numerical calculation, then overwritten by the original
        // old coefficient/sign lanes at its final store. Keep those EE bytes;
        // native math computes the sign directly from the quadrant instead.
        return true;
    }
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

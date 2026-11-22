#include "native_math.h"
#include "../port/vp2s_math.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "math_original_functions.h"
#include "math_adapted_functions.h"
#include "bozo/ops_fpu.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <bit>
#include <cmath>
#include <source_location>
#include <vector>
namespace {
void check(bool value,std::source_location at=std::source_location::current()){
    if(!value)throw std::runtime_error("native math ABI assertion failed at "+std::to_string(at.line()));
}
// Keep the test input opaque across the rounding guard. An inlined expression
// built from loop constants can be folded under the compiler's default mode.
[[gnu::noinline,gnu::noclone]] __m128 referenceNormalization(const uint8_t *source){
    __m128 input;std::memcpy(&input,source,16);return vp2s::math::normalizeQuaternion(input);
}
// Independent exact rational multiplication and truncation of finite float32
// operands. Test oracle only: no runtime arithmetic/CPU/VU implementation used.
uint32_t exactProduct(uint32_t a,uint32_t b){
    const auto sign=(a^b)&0x80000000u;const unsigned ea=(a>>23)&255,eb=(b>>23)&255;
    if(!ea || !eb)return sign;
    uint64_t product=uint64_t((a&0x7fffff)|0x800000)*((b&0x7fffff)|0x800000);
    int exponent=int(ea)+int(eb)-127;unsigned shift=23;
    if(product&(1ull<<47)){++exponent;++shift;}
    if(exponent<=0)return sign;
    if(exponent>=255)return sign|0x7f7fffffu;
    return sign|uint32_t(exponent<<23)|uint32_t((product>>shift)&0x7fffff);
}
uint32_t positiveTruncatedSum(uint32_t a,uint32_t b){
    if(a<b)std::swap(a,b);if(!b)return a;
    unsigned exponent=a>>23;const auto distance=exponent-(b>>23);
    uint32_t sum=(a&0x7fffff)|0x800000;
    if(distance<=23)sum+=((b&0x7fffff)|0x800000)>>distance;
    if(sum&0x1000000){sum>>=1;++exponent;}
    return (exponent<<23)|(sum&0x7fffff);
}
float independentAxisScale(const vp2s::math::RigidBasis &basis){
    uint32_t rows[3][4];std::memcpy(rows,basis.data(),48);uint32_t largest=0;
    for(unsigned lane=0;lane<3;++lane){
        auto squared=exactProduct(rows[0][lane],rows[0][lane]);
        squared=positiveTruncatedSum(squared,exactProduct(rows[1][lane],rows[1][lane]));
        squared=positiveTruncatedSum(squared,exactProduct(rows[2][lane],rows[2][lane]));largest=std::max(largest,squared);
    }
    // Each candidate square has at most48 significant bits, exactly
    // representable in double. Binary search gives the exact float32 floor.
    const double target=std::bit_cast<float>(largest);uint32_t low=0,high=0x7f7fffff;
    while(low<high){const auto middle=low+(high-low+1)/2;const auto exponent=middle>>23;
        const auto mantissa=(middle&0x7fffff)|(exponent?0x800000:0);
        const auto value=std::ldexp(double(mantissa),exponent?int(exponent)-150:-149);
        if(value*value<=target)low=middle;else high=middle-1;
    }
    return std::bit_cast<float>(low);
}
[[gnu::noinline]] __m128 independentSphereCenter(const vp2s::math::Matrix &matrix,__m128 sphere){
    float cols[4][4],point[4],out[4];std::memcpy(cols,matrix.data(),64);std::memcpy(point,&sphere,16);std::memcpy(out,point,16);
    const auto normal=[](float v){const auto b=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((b&0x7f800000)?b:(b&0x80000000));};
    for(unsigned lane=0;lane<3;++lane){
        volatile float first=cols[0][lane]*point[0],second=cols[1][lane]*point[1],third=cols[2][lane]*point[2];
        volatile float xy=normal(first)+normal(second);volatile float xyz=normal(xy)+normal(third);
        volatile float result=normal(xyz)+cols[3][lane];out[lane]=normal(result);
    }
    __m128 result;std::memcpy(&result,out,16);return result;
}
struct Host:bozo::ee::Host,bozo::port::Host {
    bool referenceSphere=false;
    bool referenceAxis=false;
    bool referenceOverlap=false;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("unexpected library request");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *t,uint32_t pc)override{
        if(referenceAxis && pc==0x2156b0){
            // Independent finite scalar equations: exact rational products,
            // truncated positive sums and binary-search rounded roots/ratios.
            auto floorFloat=[](double value){uint32_t low=0,high=0x7f7fffff;
                while(low<high){const auto middle=low+(high-low+1)/2;
                    if(double(std::bit_cast<float>(middle))<=value)low=middle;else high=middle-1;}
                return low;};
            auto *p=t->memory.hostPointer(c->r[17].uw[0],16);uint32_t input[4],output[4];std::memcpy(input,p,16);
            auto sum=exactProduct(input[0],input[0]);
            for(unsigned i=1;i<4;++i)sum=positiveTruncatedSum(sum,exactProduct(input[i],input[i]));
            const auto root=floorFloat(std::sqrt(double(std::bit_cast<float>(sum))));
            const auto reciprocal=root?floorFloat(1.0/double(std::bit_cast<float>(root))):0x7f7fffffu;
            for(unsigned i=0;i<4;++i)output[i]=exactProduct(input[i],reciprocal);
            std::memcpy(p,output,16);return true;
        }
        if(referenceOverlap && (pc==0x1f9da4 || pc==0x1f9e28 || pc==0x2a4994 || pc==0x2a49a0 || pc==0x2a418c || pc==0x2a4258 || pc==0x2a4478 || pc==0x2a44a8 || pc==0x2a4608 || pc==0x2a4708 || pc==0x2a47c8)){
            const unsigned laneReg=pc==0x2a49a0 || pc==0x2a44a8?13:10,destinationReg=pc==0x2a4994?4:pc==0x2a44a8?2:3;
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto sub=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::subNearest(a,b,f);};
            uint32_t out[4],flags=0;for(unsigned i=0;i<3;++i)out[i]=bozo::ps2f::fromFixed(c->r[laneReg].uw[i],15);
            auto rest=0x3f800000u;for(unsigned i=0;i<3;++i)rest=sub(rest,mul(mul(out[i],out[i]),0x3f800000));
            const auto saved=_mm_getcsr();out[3]=mul(0x3f800000,bozo::ps2f::sqrt(rest,flags));_mm_setcsr(saved);
            std::memcpy(t->memory.hostPointer(c->r[destinationReg].uw[0],16),out,16);return true;
        }
        if(referenceOverlap && (pc==0x24699c || pc==0x2469c8 || pc==0x246a30 || pc==0x246a5c || pc==0x246e7c)){
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
            const auto load=[&](uint32_t at){std::array<uint32_t,4> v;std::memcpy(v.data(),t->memory.hostPointer(at,16),16);return v;};
            const auto stack=c->r[29].uw[0];const uint32_t one=0x3f800000;
            if(pc==0x246e7c){const auto target=c->r[21].uw[0]+0x90;std::array<uint32_t,4> rows[4],m[4];uint32_t out[16];
                for(unsigned k=0;k<4;++k){rows[k]=load(stack+0xe0+16*k);m[k]=load(target+16*k);}
                for(unsigned k=0;k<4;++k)for(unsigned i=0;i<4;++i)out[4*k+i]=add(add(add(mul(m[0][i],rows[k][0]),mul(m[1][i],rows[k][1])),mul(m[2][i],rows[k][2])),mul(m[3][i],rows[k][3]));
                std::memcpy(t->memory.hostPointer(target,64),out,64);return true;}
            const bool divide=pc==0x2469c8 || pc==0x246a5c;const uint32_t out=pc==0x24699c || pc==0x2469c8?0x190:0x180;
            uint32_t result[4];
            if(divide){auto v=load(stack+out);uint32_t flags=0;const auto saved=_mm_getcsr();const auto q=bozo::ps2f::div(one,v[3],flags);_mm_setcsr(saved);
                for(unsigned i=0;i<4;++i)result[i]=mul(v[i],q);}
            else{std::array<uint32_t,4> m[4];for(unsigned k=0;k<4;++k)m[k]=load(stack+0x130+16*k);const auto v=load(stack+(pc==0x24699c?0x1b0:0x1a0));
                constexpr unsigned order[4][4]={{0,1,2,3},{1,0,2,3},{2,1,0,3},{3,1,2,0}};
                for(unsigned k=0;k<4;++k){uint32_t p[4];for(unsigned i=0;i<4;++i)p[i]=mul(m[k][i],v[i]);
                    result[k]=add(add(add(p[order[k][0]],p[order[k][1]]),mul(one,p[order[k][2]])),mul(one,p[order[k][3]]));}}
            std::memcpy(t->memory.hostPointer(stack+out,16),result,16);return true;
        }
        if(referenceOverlap && (pc==0x248f18 || pc==0x248f84)){
            // mod001 248e40 world position from the mode-0 programs' listings (ps2f).
            const auto M=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto A=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
            const auto S=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::subNearest(a,b,f);};
            using V=std::array<uint32_t,4>;const auto load=[&](uint32_t at){V v;std::memcpy(v.data(),t->memory.hostPointer(at,16),16);return v;};
            const uint32_t one=0x3f800000;const auto obj=c->r[3].uw[0]-0x110;V vf[32]{};
            vf[0]={0,0,0,one};vf[5]=load(obj+0x30);vf[16]=load(obj+0x20);vf[17]=load(obj+0x40);
            // 0x728
            for(unsigned i=0;i<4;++i){vf[6][i]=M(vf[5][i],vf[5][i]);vf[7][i]=A(vf[5][i],vf[5][i]);}vf[11]=vf[0];
            vf[8][0]=A(vf[0][0],vf[0][3]);vf[9][1]=A(vf[0][1],vf[0][3]);vf[17][3]=vf[0][3];
            for(unsigned i=0;i<4;++i)vf[6][i]=A(vf[6][i],vf[6][i]);
            for(unsigned i=0;i<4;++i){vf[12][i]=M(vf[5][i],vf[7][0]);vf[13][i]=M(vf[5][i],vf[7][3]);vf[14][i]=M(vf[5][i],vf[7][1]);}
            vf[8][0]=S(vf[8][0],vf[6][1]);for(unsigned i=0;i<4;++i)vf[15][i]=M(vf[5][i],vf[7][2]);
            vf[8][1]=A(vf[12][1],vf[13][2]);vf[8][2]=S(vf[12][2],vf[13][1]);vf[8][0]=S(vf[8][0],vf[6][2]);vf[9][1]=S(vf[9][1],vf[6][0]);
            vf[10][2]=A(vf[0][2],vf[0][3]);vf[9][0]=S(vf[14][0],vf[13][2]);vf[9][2]=A(vf[14][2],vf[12][3]);vf[9][1]=S(vf[9][1],vf[6][2]);
            vf[10][2]=S(vf[10][2],vf[6][0]);vf[10][0]=A(vf[15][0],vf[13][1]);vf[10][1]=S(vf[15][1],vf[12][3]);vf[8][3]=M(vf[0][3],vf[16][0]);
            vf[10][2]=S(vf[10][2],vf[6][1]);vf[9][3]=M(vf[0][3],vf[16][1]);vf[10][3]=M(vf[0][3],vf[16][2]);
            const auto rowsTimes=[&](const V &x,const V &y,const V &z,const V &w,const V &v){V o;for(unsigned i=0;i<4;++i)o[i]=A(A(A(M(x[i],v[0]),M(y[i],v[1])),M(z[i],v[2])),M(w[i],v[3]));return o;};
            if(pc==0x248f18){const auto par=c->r[6].uw[0];for(unsigned k=0;k<3;++k)vf[1+k]=load(par+16*k);
                // 0xae8
                for(unsigned k=1;k<=3;++k)vf[k]=rowsTimes(vf[8],vf[9],vf[10],vf[11],vf[k]);
                for(unsigned k=1;k<=3;++k)for(unsigned i=0;i<4;++i)vf[k][i]=M(vf[17][i],vf[k][i]);
                uint16_t fl;std::memcpy(&fl,t->memory.hostPointer(obj+0x20c,2),2);
                if(!(fl&0x1c0)){
                    // 0x830
                    for(unsigned i=0;i<4;++i)vf[5][i]=A(A(M(vf[1][i],vf[1][i]),M(vf[2][i],vf[2][i])),M(vf[3][i],vf[3][i]));
                    for(unsigned i=0;i<3;++i){uint32_t f=0;const auto saved=_mm_getcsr();const auto q=bozo::ps2f::rsqrt(one,vf[5][i],f);_mm_setcsr(saved);
                        for(unsigned k=1;k<=3;++k)vf[k][i]=M(vf[k][i],q);}
                    for(unsigned k=1;k<=3;++k)for(unsigned i=0;i<4;++i)vf[k][i]=M(vf[k][i],vf[17][i]);}
                vf[4]=load(obj+0x140);
            }else{for(unsigned k=0;k<3;++k)for(unsigned i=0;i<4;++i)vf[1+k][i]=M(vf[17][i],vf[8+k][i]);vf[4]=load(obj+0x140);}
            const auto r0=load(obj+0x110),r1=load(obj+0x120),r2=load(obj+0x130);V out[4];
            for(unsigned k=0;k<3;++k)out[k]=rowsTimes(r0,r1,r2,vf[4],vf[1+k]);out[3]=vf[4];
            std::memcpy(t->memory.hostPointer(c->r[29].uw[0],64),out,64);return true;
        }
        if(referenceOverlap && (pc==0x20dc34 || pc==0x20dd48 || (pc>=0x20e4c0 && pc<=0x20e4cc))){
            // mod001 20d9d0 sphere sweep, engine ps2f arithmetic.
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto sub=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::subNearest(a,b,f);};
            uint32_t o[4];
            if(pc>=0x20e4c0){const unsigned k=(pc-0x20e4c0)/4;uint32_t s[4];const auto row=c->r[21].uw[0]+0x240+16*k;
                std::memcpy(s,t->memory.hostPointer(c->r[20].uw[0],16),16);std::memcpy(o,t->memory.hostPointer(row,16),16);
                for(unsigned i=0;i<4;++i)o[i]=mul(o[i],s[k]);std::memcpy(t->memory.hostPointer(row,16),o,16);return true;}
            uint32_t a[4],b[4];std::memcpy(a,t->memory.hostPointer(c->r[22].uw[0],16),16);std::memcpy(b,t->memory.hostPointer(c->r[18].uw[0]+0x240,16),16);
            for(unsigned i=0;i<4;++i){const auto d=sub(b[i],a[i]);o[i]=mul(d,d);}
            std::memcpy(t->memory.hostPointer(c->r[29].uw[0]+(pc==0x20dc34?0xd0:0xc0),16),o,16);return true;
        }
        if(referenceOverlap && pc==0x2145b8){
            // mod001 214590 plane distance, engine ps2f arithmetic; Y/Z/W of VF16 = the point.
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
            const auto sub=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::subNearest(a,b,f);};
            uint32_t p[4],n[4],o[4],m[3];std::memcpy(p,t->memory.hostPointer(c->r[4].uw[0],16),16);std::memcpy(n,t->memory.hostPointer(c->r[5].uw[0],16),16);std::memcpy(o,t->memory.hostPointer(c->r[6].uw[0],16),16);
            for(unsigned i=0;i<3;++i)m[i]=mul(sub(p[i],o[i]),n[i]);
            p[0]=add(add(m[0],m[1]),mul(0x3f800000,m[2]));std::memcpy(t->memory.hostPointer(c->r[29].uw[0],16),p,16);return true;
        }
        if(referenceOverlap && ((pc>=0x23e21c && pc<=0x23e228) || pc==0x23e358)){
            // mod001 23e0d0 hit frame, engine ps2f arithmetic.
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
            const auto stack=c->r[29].uw[0];uint32_t o[4];
            if(pc!=0x23e358){const unsigned k=(pc-0x23e21c)/4;uint32_t a[4],b[4][4];
                std::memcpy(a,t->memory.hostPointer(c->r[16].uw[0]+0x270+16*k,16),16);std::memcpy(b,t->memory.hostPointer(c->r[19].uw[0]+0x90,64),64);
                for(unsigned i=0;i<4;++i)o[i]=add(add(add(mul(b[0][i],a[0]),mul(b[1][i],a[1])),mul(b[2][i],a[2])),mul(b[3][i],a[3]));
                std::memcpy(t->memory.hostPointer(stack+0xe0+16*k,16),o,16);return true;}
            uint32_t R[4][4],v[4];std::memcpy(R,t->memory.hostPointer(stack+0xe0,64),64);std::memcpy(v,t->memory.hostPointer(stack+0xd0,16),16);
            constexpr unsigned order[4][4]={{0,1,2,3},{1,0,2,3},{2,1,0,3},{3,1,2,0}};
            for(unsigned k=0;k<4;++k){uint32_t p[4];for(unsigned i=0;i<4;++i)p[i]=mul(R[k][i],v[i]);
                o[k]=add(add(add(p[order[k][0]],p[order[k][1]]),mul(0x3f800000,p[order[k][2]])),mul(0x3f800000,p[order[k][3]]));}
            std::memcpy(t->memory.hostPointer(c->r[18].uw[0]+0x2b0,16),o,16);return true;
        }
        if(referenceOverlap && pc==0x29f698){
            // mod001 29f5e0 node transform, engine ps2f arithmetic.
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
            const auto stack=c->r[29].uw[0];uint32_t object,R[4][4],p[4],o[4];std::memcpy(&object,t->memory.hostPointer(c->r[16].uw[0]+0x230,4),4);
            std::memcpy(R,t->memory.hostPointer(object+0x190,64),64);std::memcpy(p,t->memory.hostPointer(stack+0x30,16),16);
            for(unsigned i=0;i<4;++i)o[i]=add(add(add(mul(R[0][i],p[0]),mul(R[1][i],p[1])),mul(R[2][i],p[2])),mul(R[3][i],p[3]));
            std::memcpy(t->memory.hostPointer(stack+0x20,16),o,16);return true;
        }
        if(referenceOverlap && pc>=0x2179a8 && pc<=0x2179b4){
            // mod001 2178c0 composition: row k at its own store, engine ps2f arithmetic.
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
            const auto stack=c->r[29].uw[0];const unsigned k=(pc-0x2179a8)/4;uint32_t a[4],b[4][4],o[4];
            std::memcpy(a,t->memory.hostPointer(stack+0x40+16*k,16),16);std::memcpy(b,t->memory.hostPointer(stack+0x80,64),64);
            for(unsigned i=0;i<4;++i)o[i]=add(add(add(mul(b[0][i],a[0]),mul(b[1][i],a[1])),mul(b[2][i],a[2])),mul(b[3][i],a[3]));
            std::memcpy(t->memory.hostPointer(c->r[16].uw[0]+16*k,16),o,16);return true;
        }
        if(referenceOverlap && ((pc>=0x220440 && pc<=0x22044c) || pc==0x2204c8)){
            // mod001 2202d0 cull transform: composition row k at its own store, then the point; engine ps2f arithmetic.
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
            if(pc!=0x2204c8){const unsigned k=(pc-0x220440)/4;uint32_t a[4],b[4][4],o[4];
                std::memcpy(a,t->memory.hostPointer(c->r[3].uw[0]+16*k,16),16);std::memcpy(b,t->memory.hostPointer(c->r[2].uw[0],64),64);
                for(unsigned i=0;i<4;++i)o[i]=add(add(add(mul(b[0][i],a[0]),mul(b[1][i],a[1])),mul(b[2][i],a[2])),mul(b[3][i],a[3]));
                std::memcpy(t->memory.hostPointer(c->r[4].uw[0]+16*k,16),o,16);}
            else{uint32_t R[4][4],p[4],o[4];std::memcpy(R,t->memory.hostPointer(c->r[3].uw[0],64),64);std::memcpy(p,t->memory.hostPointer(c->r[4].uw[0],16),16);
                for(unsigned i=0;i<4;++i)o[i]=add(add(add(mul(R[0][i],p[0]),mul(R[1][i],p[1])),mul(R[2][i],p[2])),mul(R[3][i],p[3]));
                std::memcpy(t->memory.hostPointer(c->r[2].uw[0],16),o,16);}
            return true;
        }
        if(referenceOverlap && (pc==0x1fbb24 || pc==0x1fbb54 || pc==0x1fbb9c)){
            // mod001 1fbad0 range test, engine ps2f arithmetic.
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
            using V=std::array<uint32_t,4>;
            const auto load=[&](uint32_t at){V v;std::memcpy(v.data(),t->memory.hostPointer(at,16),16);return v;};
            const auto stack=c->r[29].uw[0];V o;
            if(pc==0x1fbb24){V R[4];for(unsigned k=0;k<4;++k)R[k]=load(c->r[4].uw[0]+0x220+16*k);const auto p=load(c->r[5].uw[0]);
                for(unsigned i=0;i<4;++i)o[i]=add(add(add(mul(R[0][i],p[0]),mul(R[1][i],p[1])),mul(R[2][i],p[2])),mul(R[3][i],p[3]));
                std::memcpy(t->memory.hostPointer(stack+0x30,16),o.data(),16);return true;}
            const auto v=load(pc==0x1fbb54?stack+0x20:stack+0x30);o=v;uint32_t sq[3];for(unsigned i=0;i<3;++i)sq[i]=mul(v[i],v[i]);
            o[0]=add(add(sq[0],sq[1]),mul(0x3f800000,sq[2]));o[1]=sq[1];o[2]=sq[2];
            std::memcpy(t->memory.hostPointer(pc==0x1fbb54?stack+0x10:stack,16),o.data(),16);return true;
        }
        if(referenceOverlap && (pc==0x1fbda0 || pc==0x1fbdc0)){
            // mod001 1fbd60 plane contact, engine ps2f arithmetic.
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
            const auto sub=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::subNearest(a,b,f);};
            using V=std::array<uint32_t,4>;
            const auto load=[&](uint32_t at){V v;std::memcpy(v.data(),t->memory.hostPointer(at,16),16);return v;};
            const auto stack=c->r[29].uw[0],plane=c->r[4].uw[0];const auto p=load(c->r[5].uw[0]),n=load(plane+0x220);const uint32_t one=0x3f800000;
            if(pc==0x1fbda0){const auto o=load(plane+0x230);uint32_t m[3];for(unsigned i=0;i<3;++i)m[i]=mul(sub(p[i],o[i]),n[i]);
                const auto d=sub(add(add(m[0],m[1]),mul(one,m[2])),std::bit_cast<uint32_t>(c->f[12]));std::memcpy(t->memory.hostPointer(stack+0x10,4),&d,4);return true;}
            V o=p;
            {uint32_t d;std::memcpy(&d,t->memory.hostPointer(stack+0x10,4),4);for(unsigned i=0;i<3;++i)o[i]=sub(add(p[i],0),mul(n[i],d));
                std::memcpy(t->memory.hostPointer(c->r[6].uw[0],16),o.data(),16);return true;}
        }
        if(referenceOverlap && (pc==0x1fc3c0 || pc==0x1fc3c4 || pc==0x1fc478)){
            // mod001 1fc300 capsule pair, engine ps2f arithmetic.
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
            const auto sub=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::subNearest(a,b,f);};
            using V=std::array<uint32_t,4>;
            const auto load=[&](uint32_t at){V v;std::memcpy(v.data(),t->memory.hostPointer(at,16),16);return v;};
            const auto word=[&](uint32_t at){uint32_t v;std::memcpy(&v,t->memory.hostPointer(at,4),4);return v;};
            const auto stack=c->r[29].uw[0],A=c->r[20].uw[0],B=c->r[19].uw[0];const uint32_t one=0x3f800000;
            if(pc==0x1fc478){
                const auto p0=load(B+0x230),p1=load(B+0x240),p=load(stack+0x60),d=load(stack+0x70);const auto tb=word(stack+0x88),ta=word(stack+0x8c);
                uint32_t delta[3],sq[3],out[3],flags=0;
                for(unsigned i=0;i<3;++i){delta[i]=sub(sub(add(add(p0[i],0),mul(p1[i],tb)),mul(p[i],one)),mul(d[i],ta));sq[i]=mul(delta[i],delta[i]);}
                const auto saved=_mm_getcsr();const auto q=bozo::ps2f::rsqrt(one,add(add(sq[0],sq[1]),mul(one,sq[2])),flags);_mm_setcsr(saved);
                for(unsigned i=0;i<3;++i)out[i]=mul(delta[i],q);std::memcpy(t->memory.hostPointer(c->r[18].uw[0],12),out,12);return true;
            }
            V R[4];const auto m=word(word(stack+0x20)+4)+0x190;for(unsigned k=0;k<4;++k)R[k]=load(m+16*k);V o;
            if(pc==0x1fc3c4){const auto d=load(A+0x260);for(unsigned i=0;i<4;++i)o[i]=add(add(mul(R[0][i],d[0]),mul(R[1][i],d[1])),mul(R[2][i],d[2]));
                std::memcpy(t->memory.hostPointer(stack+0x70,16),o.data(),16);return true;}
            const auto p=load(A+0x250),w=load(c->r[6].uw[0]);const auto ra=word(A+0x220),rb=word(B+0x220),sum=add(rb,ra);
            for(unsigned i=0;i<4;++i)o[i]=add(add(add(mul(R[0][i],p[0]),mul(R[1][i],p[1])),mul(R[2][i],p[2])),mul(R[3][i],one));
            for(unsigned i=0;i<3;++i){uint32_t flags=0;const auto saved=_mm_getcsr();const auto q=bozo::ps2f::div(sum,add(add(0,ra),mul(w[i],rb)),flags);_mm_setcsr(saved);o[i]=mul(o[i],q);}
            std::memcpy(t->memory.hostPointer(stack+0x60,16),o.data(),16);return true;
        }
        if(referenceOverlap && (pc==0x1fda70 || pc==0x1fda7c || pc==0x1fda84 || pc==0x1fdaec || pc==0x1fdaf0 || pc==0x1fdb74 || pc==0x1fdc68 ||
                                pc==0x1fdee4 || pc==0x1fdee8 || pc==0x1fe7e8 || pc==0x1fe63c || pc==0x1fe680 || pc==0x1fe6c0)){
            // mod001 1fd110 chain blocks and bone aim, engine ps2f arithmetic
            // in the original instruction order (ACC chains, OPMULA/OPMSUB).
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
            const auto sub=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::subNearest(a,b,f);};
            using V=std::array<uint32_t,4>;
            const auto load=[&](uint32_t at){V v;std::memcpy(v.data(),t->memory.hostPointer(at,16),16);return v;};
            const auto word=[&](uint32_t at){uint32_t v;std::memcpy(&v,t->memory.hostPointer(at,4),4);return v;};
            const auto store=[&](uint32_t at,const V &v){std::memcpy(t->memory.hostPointer(at,16),v.data(),16);};
            const auto stack=c->r[29].uw[0],part=c->r[17].uw[0];const uint32_t one=0x3f800000;
            const auto transform=[&](const V *R,const V &v){V o;for(unsigned i=0;i<4;++i)o[i]=add(add(add(mul(R[0][i],v[0]),mul(R[1][i],v[1])),mul(R[2][i],v[2])),mul(R[3][i],one));return o;};
            if(pc==0x1fda70){store(stack+0x460,load(part));return true;}
            if(pc==0x1fda7c){store(stack+0x460,V{0,0,0,one});return true;}
            if(pc==0x1fda84){V R[4];const auto m=word(c->r[18].uw[0]+4)+0x190;for(unsigned k=0;k<4;++k)R[k]=load(m+16*k);
                auto o=transform(R,load(word(part+0x38)+0x30));
                if(word(part+0x20)&4){const auto s=load(part);for(unsigned i=0;i<3;++i)o[i]=add(o[i],s[i]);}
                store(stack+0x220,o);return true;}
            if(pc==0x1fdaec || pc==0x1fdaf0){const auto v=load(stack+0x220);const auto s=pc==0x1fdaec?word(part+0x30):std::bit_cast<uint32_t>(c->f[22]);
                V o;for(unsigned i=0;i<4;++i)o[i]=mul(v[i],s);store(pc==0x1fdaec?stack+0x460:stack+0x470,o);return true;}
            if(pc==0x1fdb74 || pc==0x1fdc68){V R[3];const auto m=word(c->r[18].uw[0]+4)+0x150;for(unsigned k=0;k<3;++k)R[k]=load(m+16*k);
                const auto v=load(stack+0x210);V o;for(unsigned i=0;i<4;++i)o[i]=add(add(mul(R[0][i],v[0]),mul(R[1][i],v[1])),mul(R[2][i],v[2]));
                store(stack+0x210,o);return true;}
            if(pc==0x1fdee4){auto o=load(stack+0x460);const auto d=load(stack+0x220);const auto a=word(part+0x30);
                for(unsigned i=0;i<3;++i)o[i]=add(add(o[i],0),mul(d[i],a));store(stack+0x200,o);return true;}
            if(pc==0x1fdee8){auto o=load(stack+0x220);const auto s=std::bit_cast<uint32_t>(c->f[22]);
                for(unsigned i=0;i<3;++i)o[i]=sub(add(o[i],0),mul(o[i],s));store(stack+0x1f0,o);return true;}
            if(pc==0x1fe7e8){V R[4],o;for(unsigned k=0;k<4;++k)R[k]=load(stack+0x120+16*k);const auto v=load(c->r[16].uw[0]+0x30);
                constexpr unsigned order[4][4]={{0,1,2,3},{1,0,2,3},{2,1,0,3},{3,1,2,0}};
                for(unsigned k=0;k<4;++k){uint32_t p[4];for(unsigned i=0;i<4;++i)p[i]=mul(R[k][i],v[i]);
                    o[k]=add(add(add(p[order[k][0]],p[order[k][1]]),mul(one,p[order[k][2]])),mul(one,p[order[k][3]]));}
                store(stack+0x110,o);return true;}
            if(pc==0x1fe63c || pc==0x1fe680){
                V R[4];for(unsigned k=0;k<4;++k)R[k]=load(stack+0x180+16*k);
                const auto unit=[&](const V &p){const auto v=transform(R,p);uint32_t s[3],flags=0;for(unsigned i=0;i<3;++i)s[i]=mul(v[i],v[i]);
                    const auto saved=_mm_getcsr();const auto q=bozo::ps2f::rsqrt(one,add(add(s[0],s[1]),mul(one,s[2])),flags);_mm_setcsr(saved);
                    V u{};for(unsigned i=0;i<3;++i)u[i]=mul(v[i],q);return u;};
                const auto a=unit(load(stack+0x1c0)),b=unit(load(word(c->r[30].uw[0]+0x38)+0x30));
                const auto dot=add(add(mul(a[0],b[0]),mul(a[1],b[1])),mul(a[2],b[2]));
                if(pc==0x1fe63c){std::memcpy(t->memory.hostPointer(stack+0x170,4),&dot,4);return true;}
                const uint32_t cross[3]={sub(mul(b[1],a[2]),mul(a[1],b[2])),sub(mul(b[2],a[0]),mul(a[2],b[0])),sub(mul(b[0],a[1]),mul(a[0],b[1]))};
                uint32_t flags=0;const auto saved=_mm_getcsr();
                const auto q=mul(one,bozo::ps2f::sqrt(add(add(0,0x40000000),mul(dot,0x40000000)),flags));const auto r=bozo::ps2f::div(one,q,flags);_mm_setcsr(saved);
                V o;for(unsigned i=0;i<3;++i)o[i]=mul(cross[i],r);o[3]=mul(q,0x3f000000);store(stack+0x160,o);return true;}
            // 1fe6c0: q1 (SP+160) * q2 (s1+30)
            const auto q1=load(stack+0x160),q2=load(c->r[17].uw[0]+0x30);V o,p;for(unsigned i=0;i<4;++i)p[i]=mul(q1[i],q2[i]);
            const uint32_t cross[3]={sub(mul(q1[1],q2[2]),mul(q2[1],q1[2])),sub(mul(q1[2],q2[0]),mul(q2[2],q1[0])),sub(mul(q1[0],q2[1]),mul(q2[0],q1[1]))};
            for(unsigned i=0;i<3;++i)o[i]=add(add(mul(q2[i],q1[3]),mul(q1[i],q2[3])),mul(cross[i],one));
            o[3]=sub(sub(sub(p[3],p[2]),mul(one,p[1])),mul(one,p[0]));store(stack+0x160,o);return true;
        }
        if(referenceOverlap && (pc==0x1f67fc || pc==0x1f6800 || pc==0x1f69ac || pc==0x1f69b0 || pc==0x1f69c0 || pc==0x1f69f4)){
            // Segment against the hit box, engine ps2f arithmetic.
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
            const auto load=[&](uint32_t at){std::array<uint32_t,4> v;std::memcpy(v.data(),t->memory.hostPointer(at,16),16);return v;};
            uint32_t object,query;std::memcpy(&object,t->memory.hostPointer(0x358cd8,4),4);std::memcpy(&query,t->memory.hostPointer(0x358d54,4),4);
            const auto p=load(query),d=load(query+0x10);const uint32_t one=0x3f800000;const auto stack=c->r[29].uw[0];
            if(pc==0x1f67fc || pc==0x1f6800){
                std::array<uint32_t,4> R[4];for(unsigned i=0;i<4;++i)R[i]=load(object+0x150+16*i);const auto cc=load(c->r[4].uw[0]);
                uint32_t world[4];for(unsigned i=0;i<4;++i)world[i]=add(add(add(mul(R[0][i],cc[0]),mul(R[1][i],cc[1])),mul(R[2][i],cc[2])),mul(R[3][i],one));
                uint32_t v[3];if(pc==0x1f67fc){for(unsigned i=0;i<3;++i)v[i]=add(p[i],world[i]^0x80000000u);}else for(unsigned i=0;i<3;++i)v[i]=d[i];
                const auto dot=[&](const std::array<uint32_t,4> &r,unsigned i,unsigned j,unsigned k){return add(add(mul(v[i],r[i]),mul(v[j],r[j])),mul(one,mul(v[k],r[k])));};
                const uint32_t out[3]={dot(R[0],0,1,2),dot(R[1],1,0,2),dot(R[2],2,0,1)};
                std::memcpy(t->memory.hostPointer(pc==0x1f67fc?stack+0x90:stack+0x80,12),out,12);return true;
            }
            uint32_t clip[2];std::memcpy(clip,t->memory.hostPointer(stack+0xa0,8),8);const auto tt=clip[pc==0x1f69b0 || pc==0x1f69f4?1:0];
            uint32_t out[3];for(unsigned i=0;i<3;++i)out[i]=add(mul(p[i],one),mul(d[i],tt));
            std::memcpy(t->memory.hostPointer(c->r[20].uw[0]+(pc==0x1f69b0?0x10:0),12),out,12);return true;
        }
        if(referenceOverlap && ((pc>=0x1f6c54 && pc<=0x1f6de0) || pc==0x1f6e74)){
            // Collision leaf tests from their records, engine ps2f arithmetic.
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
            const auto sub=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::subNearest(a,b,f);};
            const auto load=[&](uint32_t at){std::array<uint32_t,4> v;std::memcpy(v.data(),t->memory.hostPointer(at,16),16);return v;};
            uint32_t object;std::memcpy(&object,t->memory.hostPointer(0x358cd8,4),4);
            std::array<uint32_t,4> R[4];for(unsigned i=0;i<4;++i)R[i]=load(object+0x150+16*i);uint32_t scale;std::memcpy(&scale,t->memory.hostPointer(object+0x204,4),4);
            const auto transform=[&](const std::array<uint32_t,4> &v){std::array<uint32_t,4> o;
                for(unsigned i=0;i<4;++i)o[i]=add(add(add(mul(R[0][i],v[0]),mul(R[1][i],v[1])),mul(R[2][i],v[2])),mul(R[3][i],0x3f800000));return o;};
            const uint32_t one=0x3f800000;auto *frame=t->memory.hostPointer(c->r[29].uw[0],32);
            if(pc==0x1f6e74){
                const auto s=load(c->r[4].uw[0]),p=load(c->r[5].uw[0]),d=load(c->r[5].uw[0]+0x10);const auto centre=transform(s);
                const auto r=mul(s[3],scale),r2=mul(r,r);uint32_t delta[3],prod[3];for(unsigned i=0;i<3;++i){delta[i]=sub(centre[i],p[i]);prod[i]=mul(delta[i],d[i]);}
                auto along=add(add(mul(one,prod[0]),mul(one,prod[1])),mul(one,prod[2]));along=bozo::ps2f::max(along,0);
                uint32_t sq[3];for(unsigned i=0;i<3;++i){const auto off=sub(delta[i],mul(d[i],along));sq[i]=mul(off,off);}
                auto out=s;out[3]=sub(add(add(mul(one,sq[0]),mul(one,sq[1])),mul(one,sq[2])),r2);std::memcpy(frame,out.data(),16);return true;
            }
            const auto box=load(c->r[4].uw[0]),half=load(c->r[5].uw[0]),q=load(c->r[6].uw[0]),e=load(c->r[6].uw[0]+0x10);const auto centre=transform(box);
            const auto absb=[](uint32_t v){return v&0x7fffffffu;};
            uint32_t tt[3];for(unsigned i=0;i<3;++i)tt[i]=sub(q[i],centre[i]);
            const auto dot=[&](const uint32_t *a,const std::array<uint32_t,4> &r,unsigned i,unsigned j,unsigned k){return add(add(mul(a[i],r[i]),mul(a[j],r[j])),mul(one,mul(a[k],r[k])));};
            const unsigned stage=pc==0x1f6c54||pc==0x1f6c58?0:pc==0x1f6cb4||pc==0x1f6cb8?1:pc==0x1f6d10||pc==0x1f6d14?2:pc==0x1f6d68?3:pc==0x1f6da8?4:5;
            const uint32_t ev[3]={e[0],e[1],e[2]};uint32_t v10[3],v11[3],v12[3],v13[3],v15[3],v16[3],v24[3],v25[3];bool d24[3]{},d25[3]{};
            v10[0]=dot(ev,R[0],0,1,2);v12[0]=dot(tt,R[0],0,1,2);v10[1]=dot(ev,R[1],1,0,2);v12[1]=dot(tt,R[1],1,0,2);
            v11[0]=absb(v10[0]);v13[0]=absb(v12[0]);v25[0]=mul(v12[0],v10[0]);v24[0]=sub(v13[0],half[0]);d24[0]=d25[0]=true;
            if(stage>=1){v11[1]=absb(v10[1]);v13[1]=absb(v12[1]);v25[1]=mul(v12[1],v10[1]);v24[1]=sub(v13[1],half[1]);v10[2]=dot(ev,R[2],2,0,1);v12[2]=dot(tt,R[2],2,0,1);d24[1]=d25[1]=true;}
            if(stage>=2){v11[2]=absb(v10[2]);v13[2]=absb(v12[2]);
                v15[0]=sub(mul(ev[1],tt[2]),mul(tt[1],ev[2]));v15[1]=sub(mul(ev[2],tt[0]),mul(tt[2],ev[0]));v15[2]=sub(mul(ev[0],tt[1]),mul(tt[0],ev[1]));
                v25[2]=mul(v12[2],v10[2]);v24[2]=sub(v13[2],half[2]);d24[2]=d25[2]=true;}
            if(stage>=3){v16[0]=absb(dot(v15,R[0],0,1,2));v24[1]=sub(add(mul(v11[1],half[2]),mul(half[1],v11[2])),mul(one,v16[0]));}
            if(stage>=4){v16[1]=absb(dot(v15,R[1],1,0,2));v24[0]=sub(add(mul(v11[0],half[2]),mul(half[0],v11[2])),mul(one,v16[1]));}
            if(stage>=5){v16[2]=absb(dot(v15,R[2],2,0,1));v24[0]=sub(add(mul(v11[0],half[1]),mul(half[0],v11[1])),mul(one,v16[2]));}
            const bool second=pc==0x1f6c58 || pc==0x1f6cb8 || pc==0x1f6d14;
            for(unsigned i=0;i<3;++i){if(!second && d24[i])std::memcpy(frame+4*i,&v24[i],4);if(second && d25[i])std::memcpy(frame+0x10+4*i,&v25[i],4);}
            return true;
        }
        if(referenceOverlap && pc==0x1fc570){
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
            uint32_t a[4],d[4],p[4],delta[3],sq[3],out[3],flags=0;const auto segment=c->r[19].uw[0]+0x230;
            std::memcpy(a,t->memory.hostPointer(segment,16),16);std::memcpy(d,t->memory.hostPointer(segment+0x10,16),16);std::memcpy(p,t->memory.hostPointer(c->r[20].uw[0]+0x230,16),16);
            const uint32_t s=c->r[2].uw[0];
            for(unsigned i=0;i<3;++i){delta[i]=add(add(add(0,a[i]),mul(d[i],s)),mul(p[i],0x3f800000)^0x80000000u);sq[i]=mul(delta[i],delta[i]);}
            const auto saved=_mm_getcsr();const auto q=bozo::ps2f::rsqrt(0x3f800000,add(add(sq[0],sq[1]),mul(0x3f800000,sq[2])),flags);_mm_setcsr(saved);
            for(unsigned i=0;i<3;++i)out[i]=mul(delta[i],q);std::memcpy(t->memory.hostPointer(c->r[18].uw[0]&~15u,12),out,12);return true;
        }
        if(referenceOverlap && (pc==0x209c94 || pc==0x209d14 || pc==0x20a084)){
            // Common-math 0x1f0 column load then 0x0 projection, engine ps2f arithmetic.
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
            const auto point=c->r[29].uw[0]+(pc==0x209c94?0x80:pc==0x209d14?0x70:0x60);
            uint32_t m[4][4],v[4],p[4],out[4],flags=0;std::memcpy(m,t->memory.hostPointer(c->r[18].uw[0]+0x330,64),64);std::memcpy(v,t->memory.hostPointer(point,16),16);
            for(unsigned i=0;i<4;++i){uint32_t column[4];
                for(unsigned k=0;k<4;++k)column[k]=k==i?m[i][k]:i==3?mul(0x3f800000,m[3][k]):add(0,m[i][k]);
                p[i]=add(add(add(mul(column[0],v[0]),mul(column[1],v[1])),mul(column[2],v[2])),mul(column[3],v[3]));}
            const auto saved=_mm_getcsr();const auto q=bozo::ps2f::div(0x3f800000,p[3],flags);_mm_setcsr(saved);
            for(unsigned i=0;i<3;++i)out[i]=mul(p[i],q);out[3]=mul(0x3f800000,q);
            std::memcpy(t->memory.hostPointer(point,16),out,16);return true;
        }
        if(referenceOverlap && (pc==0x1f8d1c || pc==0x1f8d94 || pc==0x1f8e58)){
            // The engine's VU0 arithmetic definitions (ps2f, nearest even).
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
            const auto saved=_mm_getcsr();const auto stack=c->r[29].uw[0];
            if(pc!=0x1f8e58){
                // Common-math 0x7f0: per-lane sqrt of the basis column's squared length.
                uint32_t rows[3][4],out[3],flags=0;std::memcpy(rows,t->memory.hostPointer(c->r[5].uw[0],48),48);
                for(unsigned i=0;i<3;++i)out[i]=add(0,bozo::ps2f::sqrt(add(add(mul(rows[0][i],rows[0][i]),mul(rows[1][i],rows[1][i])),mul(rows[2][i],rows[2][i])),flags));
                std::memcpy(t->memory.hostPointer(c->r[2].uw[0],12),out,12);
            }else{
                uint32_t scale[4],a[4],b[4],m1[4][4],m2[4][4],t1[4],t2[4],out[4];
                std::memcpy(scale,t->memory.hostPointer(stack+0x90,16),16);std::memcpy(a,t->memory.hostPointer(c->r[4].uw[0],16),16);std::memcpy(b,t->memory.hostPointer(c->r[3].uw[0],16),16);
                std::memcpy(m1,t->memory.hostPointer(c->r[6].uw[0],64),64);std::memcpy(m2,t->memory.hostPointer(c->r[5].uw[0],64),64);
                for(unsigned i=0;i<4;++i){t1[i]=add(add(add(mul(m1[0][i],a[0]),mul(m1[1][i],a[1])),mul(m1[2][i],a[2])),mul(m1[3][i],0x3f800000));
                    t2[i]=add(add(add(mul(m2[0][i],b[0]),mul(m2[1][i],b[1])),mul(m2[2][i],b[2])),mul(m2[3][i],0x3f800000));}
                const auto radius=add(mul(a[3],scale[0]),mul(b[3],scale[1]));const auto radius2=mul(radius,radius);uint32_t sq[3];
                for(unsigned i=0;i<3;++i){const auto d=add(t1[i],t2[i]^0x80000000u);sq[i]=mul(d,d);}
                out[0]=add(add(add(mul(0x3f800000,sq[0]),mul(0x3f800000,sq[1])),mul(0x3f800000,radius2)^0x80000000u),mul(0x3f800000,sq[2]));
                out[1]=sq[1];out[2]=sq[2];out[3]=t1[3];std::memcpy(t->memory.hostPointer(stack+0x80,16),out,16);
            }
            _mm_setcsr(saved);return true;
        }
        if(referenceSphere && pc==0x239d98){
            vp2s::math::RigidBasis basis;std::memcpy(basis.data(),t->memory.hostPointer(c->r[4].uw[0]+0x90,48),48);
            const auto scale=independentAxisScale(basis);std::memcpy(t->memory.hostPointer(c->r[2].uw[0],4),&scale,4);return true;
        }
        if(referenceSphere && pc==0x239dec){
            vp2s::math::Matrix basis;for(unsigned i=0;i<4;++i)std::memcpy(&basis[i],&c->r[std::array<unsigned,4>{24,11,10,8}[i]],16);
            __m128 sphere;std::memcpy(&sphere,t->memory.hostPointer(c->r[3].uw[0],16),16);
            const auto result=independentSphereCenter(basis,sphere);std::memcpy(t->memory.hostPointer(c->r[2].uw[0],16),&result,16);return true;
        }
        if(!vp2s::nativeMathInstruction(r,c,t,pc))throw std::runtime_error("unported math site");return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("unexpected frame end");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        if(pc==0x12e1d8)c->f[0]=std::asin(c->f[12]);
        else if(pc==0x12e2d8)c->f[0]=std::atan2(c->f[12],c->f[13]);
        else if(pc==0x213fd0){
            // Shared bounded sine/cosine dependency, already qualified in its
            // own suite. This fixture isolates the original scalar builder.
            const float values[4]={0,0,std::cos(c->f[12]),std::sin(c->f[12])};
            std::memcpy(d.runtime().memory.hostPointer(c->r[4].uw[0],16),values,16);
        }
        else throw std::runtime_error("unexpected call");
        c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(int argc,char **argv){try{
    Host host;bozo::ee::Dispatcher old(host),native(host);
    old.setFunctions(math_original_function_table,math_original_function_count,math_original_resume_table,math_original_resume_count);
    native.setFunctions(math_adapted_function_table,math_adapted_function_count,math_adapted_resume_table,math_adapted_resume_count);
    old.runtime().port=native.runtime().port=&host;
    for(auto *d:{&old,&native})for(uint32_t pc:{0x12e1d8u,0x12e2d8u,0x213fd0u})d->bindLibrary(pc);
    unsigned cases=0;uint32_t random=17;
    for(bool quaternion:{false,true})for(unsigned seed=0;seed<96;++seed){
        auto &a=old.runtime();auto &b=native.runtime();
        std::memset(a.memory.flat(),seed+1,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003ff0:0x1fff000;
        const uint32_t source=seed%3==0?0x70000100:0x180000,parent=0x180100;
        const uint32_t destination=seed%3==0?source:seed%3==1?parent:0x190000;
        vp2s::math::Matrix local,view;
        for(auto *m:{&local,&view})for(auto &v:*m){alignas(16) float lanes[4];
            for(float &f:lanes){random=random*1664525u+1013904223u;f=float(int32_t(random)%10000)/1024.f;}std::memcpy(&v,lanes,16);}
        std::memcpy(a.memory.hostPointer(source,64),local.data(),64);std::memcpy(a.memory.ram()+parent,view.data(),64);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        const auto result=quaternion?vp2s::math::quaternionMatrix(local[0]):vp2s::math::compose(local,view);
        R5900Context x{},y{};
        for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x1234567800000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0xac,sizeof(x.vu0_vf));x.vu0_acc.u[1]=0x123abc;
        x.r[4].ud[0]=destination;x.r[5].ud[0]=source;x.r[6].ud[0]=parent;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;y=x;
        x.r[29].ud[0]=stack-64;x.r[9].ud[0]=0;std::memcpy(a.memory.hostPointer(stack-64,64),result.data(),64);
        const unsigned control=_mm_getcsr();
        a.functions.generated(quaternion?0x216758:0x217598)(a.memory.flat(),&x,&a);
        const unsigned expectedControl=_mm_getcsr();_mm_setcsr(control);
        b.functions.generated(quaternion?0x216730:0x217550)(b.memory.flat(),&y,&b);
        check(_mm_getcsr()==expectedControl);check(x.pc==0x170000 && y.pc==x.pc);
        check(!std::memcmp(&x,&y,sizeof(x)));
        check(!std::memcmp(x.r,y.r,sizeof(x.r)) && !std::memcmp(x.vu0_vf,y.vu0_vf,sizeof(x.vu0_vf)));
        check(!std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));++cases;
    }
    for(bool updating:{false,true})for(unsigned seed=0;seed<64;++seed){
        auto &a=old.runtime();auto &b=native.runtime();constexpr uint32_t object=0x180000;
        const uint32_t stack=seed&1?0x70003f00:0x1ffe000;
        std::memset(a.memory.flat(),seed+1,bozo::kFlatBytes);
        const auto quaternion=_mm_setr_ps(float(seed%7)/16.f,-float(seed%11)/16.f,float(seed%13)/16.f,0.5f);
        const auto position=_mm_setr_ps(float(seed)+0.5f,-float(seed)*2.f,3.75f,-123.f);
        const float scales[3]={float(seed%4)+0.5f,-2.f,1.5f};
        std::memcpy(a.memory.ram()+object+0x1e0,&quaternion,16);
        std::memcpy(a.memory.ram()+object+0x1d0,&position,16);
        std::memcpy(a.memory.ram()+object+0x1f0,scales,12);
        vp2s::math::Matrix scale{_mm_setr_ps(scales[0],0,0,0),_mm_setr_ps(0,scales[1],0,0),_mm_setr_ps(0,0,scales[2],0),_mm_setr_ps(0,0,0,1)};
        std::memcpy(a.memory.hostPointer(stack+0x60,64),scale.data(),64);
        R5900Context x{},y{};
        for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x1234567800000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        x.r[16].ud[0]=object;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;
        std::memset(x.vu0_vf,0xce,sizeof(x.vu0_vf));x.vu0_acc.u[3]=0xbabeface;
        std::memcpy(a.memory.hostPointer(stack,16),&x.r[16],16);std::memcpy(a.memory.hostPointer(stack+0x10,8),&x.r[31].ud[0],8);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);y=x;
        auto rotation=vp2s::math::quaternionMatrix(quaternion);
        rotation[0]=_mm_blend_ps(rotation[0],vp2s::music::broadcast<0>(position),8);
        rotation[1]=_mm_blend_ps(rotation[1],vp2s::music::broadcast<1>(position),8);
        rotation[2]=_mm_blend_ps(rotation[2],vp2s::music::broadcast<2>(position),8);
        std::memcpy(a.memory.hostPointer(stack+0x20,64),rotation.data(),64);
        vp2s::math::Matrix output;
        for(unsigned col=0;col<4;++col)output[col]=vp2s::math::matrixVectorProduct(scale,rotation[col]);
        std::memcpy(a.memory.ram()+object+0x110,output.data(),64);
        x.r[9].ud[0]=0;x.fu[0]=x.fu[1]=0;
        if(updating){x.r[3].ud[0]=stack+0x60;x.r[4].ud[0]=stack+0x20;x.r[5].ud[0]=object+0x110;}
        a.functions.generated(updating?0x24937c:0x2496e8)(a.memory.flat(),&x,&a);
        b.functions.generated(updating?0x249288:0x2495f4)(b.memory.flat(),&y,&b);
        check(x.pc==0x170000 && y.pc==x.pc && !std::memcmp(x.r,y.r,sizeof(x.r)));
        check(!std::memcmp(&x,&y,sizeof(x)));
        check(!std::memcmp(x.fu,y.fu,sizeof(x.fu)) && !std::memcmp(x.vu0_vf,y.vu0_vf,sizeof(x.vu0_vf)));
        check(!std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));++cases;
    }
    auto &r=native.runtime();constexpr uint32_t object=0x180000;auto *p=r.memory.ram()+object;
    std::memset(old.runtime().memory.flat(),0x79,bozo::kFlatBytes);
    std::memcpy(r.memory.flat(),old.runtime().memory.flat(),bozo::kFlatBytes);
    unsigned products=0;
    for(unsigned sample=0;sample<262144;++sample){
        uint32_t a[4],b[4],result[4];
        for(unsigned lane=0;lane<4;++lane){
            random=random*1664525u+1013904223u;
            a[lane]=(random&0x807fffff)|(((random>>23)%255)<<23);
            random=random*1664525u+1013904223u;
            b[lane]=(random&0x807fffff)|(((random>>23)%255)<<23);
            result[lane]=exactProduct(a[lane],b[lane]);++products;
        }
        for(auto *ram:{r.memory.ram(),old.runtime().memory.ram()}){
            std::memcpy(ram+object+0x560,a,16);std::memcpy(ram+object+0x570,b,16);
        }
        std::memcpy(old.runtime().memory.ram()+object+0x580,result,16);
        R5900Context c{};std::memset(c.vu0_vf,0xe2,sizeof(c.vu0_vf));c.vu0_status=0xabc;
        c.r[4].uw[0]=object;c.r[31].uw[0]=0x170000;
        c.r[3].ud[1]=c.r[5].ud[1]=c.r[6].ud[1]=0xabcdef1234567890ull;
        auto expected=c;expected.r[3].ud[0]=object+0x580;expected.r[5].ud[0]=object+0x570;expected.r[6].ud[0]=object+0x560;expected.pc=0x170000;
        const auto fp=_mm_getcsr();r.functions.generated(0x211da0)(r.memory.flat(),&c,&r);
        check(_mm_getcsr()==fp && !std::memcmp(&c,&expected,sizeof(c)));
        check(!std::memcmp(p+0x580,result,16));
        c=expected;c.pc=0x23c17c;c.r[4].ud[0]=object+0x560;expected=c;expected.pc=0x170000;
        std::memset(p+0x580,0xa4,16);r.functions.generated(0x23c17c)(r.memory.flat(),&c,&r);
        check(_mm_getcsr()==fp && !std::memcmp(&c,&expected,sizeof(c)) && !std::memcmp(p+0x580,result,16));
        if(sample%8192==0)check(!std::memcmp(r.memory.flat(),old.runtime().memory.flat(),bozo::kFlatBytes));
    }
    for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<64;++seed){
        auto &reference=old.runtime();std::memset(reference.memory.flat(),seed+1,bozo::kFlatBytes);
        const uint32_t left=seed&1?0x70000100:0x180000,right=left+(seed%4?16:0),dest=seed%3==0?left:seed%3==1?right:0x1a0000;
        uint32_t values[8];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|(((random>>23)%220)<<23);}
        std::memcpy(reference.memory.hostPointer(left,16),values,16);std::memcpy(reference.memory.hostPointer(right,16),values+4,16);
        uint32_t a[4],b[4],result[4];std::memcpy(a,reference.memory.hostPointer(left,16),16);std::memcpy(b,reference.memory.hostPointer(right,16),16);
        for(unsigned i=0;i<4;++i)result[i]=exactProduct(a[i],b[i]);
        std::memcpy(r.memory.flat(),reference.memory.flat(),bozo::kFlatBytes);std::memcpy(reference.memory.hostPointer(dest,16),result,16);
        R5900Context c{};for(unsigned i=1;i<32;++i){c.r[i].ud[0]=seed*32+i;c.r[i].ud[1]=~c.r[i].ud[0];}
        std::memset(c.vu0_vf,0xc3,sizeof(c.vu0_vf));c.vu0_acc.u[1]=0xbeef;c.r[3].ud[0]=dest;c.r[4].ud[0]=left;c.r[5].ud[0]=right;c.r[31].ud[0]=0x170000;
        auto expected=c;expected.pc=0x170000;const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|rounding;_mm_setcsr(control);
        r.functions.generated(0x23c17c)(r.memory.flat(),&c,&r);check(_mm_getcsr()==control);_mm_setcsr(saved);
        check(!std::memcmp(&c,&expected,sizeof(c)) && !std::memcmp(r.memory.flat(),reference.memory.flat(),bozo::kFlatBytes));
    }
    R5900Context invalid{};invalid.r[6].uw[0]=object+0x560;invalid.r[5].uw[0]=object+0x570;invalid.r[3].uw[0]=object+0x580;
    const uint32_t nan=0x7fc00000;std::memcpy(p+0x560,&nan,4);uint32_t prior[4];std::memcpy(prior,p+0x580,16);
    bool nonfiniteRejected=false;try{vp2s::nativeMathInstruction(r.memory.flat(),&invalid,&r,0x211dbc);}catch(const std::runtime_error &){nonfiniteRejected=true;}
    check(nonfiniteRejected && !std::memcmp(prior,p+0x580,16));
    invalid.r[4].uw[0]=object+0x560;nonfiniteRejected=false;
    try{vp2s::nativeMathInstruction(r.memory.flat(),&invalid,&r,0x23c188);}catch(const std::runtime_error &){nonfiniteRejected=true;}
    check(nonfiniteRejected && !std::memcmp(prior,p+0x580,16));
    std::memset(p,0,0x54);uint32_t type=0x1591d0;std::memcpy(p,&type,4);
    R5900Context c{};c.r[4].uw[0]=object;const auto saved=c;
    check(vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x29e1a0));check(!std::memcmp(&saved,&c,sizeof(c)) && p[0x50]==0);
    // Mode 1 (fingerprinted second code-only program) is admitted at the
    // conditional selector only; other modes and the unconditional one reject.
    c.r[5].uw[0]=1;check(vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x29e1a0));check(p[0x50]==0);
    {bool rejected=false;try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x29e160);}catch(const std::runtime_error &){rejected=true;}check(rejected);}
    for(unsigned mode:{2u,128u,255u}){c.r[5].uw[0]=mode;bool rejected=false;try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x29e1a0);}catch(const std::runtime_error &){rejected=true;}check(rejected);}
    c.r[5].uw[0]=0;p[0x50]=1;bool rejected=false;try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x29e1a0);}catch(const std::runtime_error &){rejected=true;}check(rejected && p[0x50]==1);
    for(unsigned seed=0;seed<64;++seed){
        auto &a=old.runtime();auto &b=native.runtime();const uint32_t stack=seed&1?0x70003f00:0x1fff000,frame=stack-0xc0;
        const uint32_t source=seed%3?0x180000:0x70000100,destination=seed%4?0x190000:source;
        std::memset(a.memory.flat(),seed+1,bozo::kFlatBytes);
        const auto input=_mm_setr_ps(float(int(seed%9)-4)/8.f,float(int(seed%13)-6)/8.f,float(int(seed%7)-3)/8.f,float(seed%11)/8.f);
        std::memcpy(a.memory.hostPointer(source,16),&input,16);std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{},y{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x1234000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0xad,sizeof(x.vu0_vf));x.r[4].ud[0]=destination;x.r[5].ud[0]=source;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;y=x;
        std::memcpy(a.memory.hostPointer(frame,16),&x.r[16],16);std::memcpy(a.memory.hostPointer(frame+0x10,8),&x.r[31].ud[0],8);
        const auto normalized=referenceNormalization(a.memory.hostPointer(source,16));const auto rotation=vp2s::math::quaternionMatrix(input);
        std::memcpy(a.memory.hostPointer(frame+0xa0,16),&normalized,16);std::memcpy(a.memory.hostPointer(frame+0x20,64),rotation.data(),64);
        alignas(16) uint32_t lanes[4];std::memcpy(lanes,&input,16);for(unsigned i=0;i<4;++i)x.fu[i]=lanes[3-i];
        x.r[16].ud[0]=destination;x.r[29].ud[0]=frame;x.r[2].ud[0]=frame+0x20;x.r[9].ud[0]=0;
        const auto control=_mm_getcsr();a.functions.generated(0x2158e0)(a.memory.flat(),&x,&a);const auto expected=_mm_getcsr();_mm_setcsr(control);
        b.functions.generated(0x215870)(b.memory.flat(),&y,&b);check(_mm_getcsr()==expected);
        check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));++cases;
    }
    unsigned sphereCases=0;
    for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<96;++seed){
        auto &a=old.runtime(),&b=native.runtime();constexpr uint32_t shape=0x1b0000,object=0x180000;
        const uint32_t stack=seed&1?0x70003f00:0x1fff000,frame=stack-0x20;
        const uint32_t sphere=seed%4==0?object+0x7e4:seed%4==1?frame-0xc:seed%4==2?object+0x84:0x1c0000;
        std::memset(a.memory.flat(),seed+1,bozo::kFlatBytes);
        vp2s::math::Matrix matrix;
        for(auto &row:matrix){float values[4];for(auto &v:values){random=random*1664525u+1013904223u;v=float(int32_t(random)%100)/16.f;}std::memcpy(&row,values,16);}
        std::memcpy(a.memory.hostPointer(object+0x90,64),matrix.data(),64);
        const float center[4]={float(int(seed)-24)/8.f,float(seed%11)/4.f,-float(seed%7),float(seed%17)/2.f};
        std::memcpy(a.memory.hostPointer(sphere+0xc,16),center,16);
        const uint32_t shapePointer=seed%9?shape:0,spherePointer=seed%7?sphere:0,child=seed%3?0x1d0000:0;
        std::memcpy(a.memory.hostPointer(object+0x644,4),&shapePointer,4);std::memcpy(a.memory.hostPointer(object+0x7bc,4),&spherePointer,4);
        std::memcpy(a.memory.hostPointer(shape+8,4),&child,4);a.memory.ram()[shape+0x1c]=seed&1;
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0xaa,sizeof(x.vu0_vf));x.vu0_acc.u[0]=0xbeef;x.vu0_status=0x1234;
        x.r[4].ud[0]=object;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|rounding;_mm_setcsr(control);
        host.referenceSphere=true;a.functions.generated(0x239ce0)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceSphere=false;b.functions.generated(0x239ce0)(b.memory.flat(),&y,&b);
        check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);x.cycleLimit=y.cycleLimit;
        check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));++sphereCases;
    }
    for(unsigned sample=0;sample<8192;++sample){
        vp2s::math::RigidBasis basis;for(auto &row:basis){uint32_t bits[4];for(auto &b:bits){random=random*1664525u+1013904223u;b=(random&0x807fffff)|((80+(random%80))<<23);}std::memcpy(&row,bits,16);}
        const auto expected=independentAxisScale(basis);const auto control=_mm_getcsr();const auto scale=vp2s::math::largestAxisScale(basis);
        check(_mm_getcsr()==control && std::bit_cast<uint32_t>(expected)==std::bit_cast<uint32_t>(scale));
    }
    vp2s::math::RigidBasis invalidBasis{_mm_setr_ps(1,0,0,0),_mm_setr_ps(0,1,0,0),_mm_setr_ps(0,0,1,0)};
    for(auto &row:invalidBasis)row=_mm_blend_ps(row,_mm_castsi128_ps(_mm_set1_epi32(0x7fc00000)),8);
    check(vp2s::math::largestAxisScale(invalidBasis)==1.f);
    invalidBasis[0]=_mm_castsi128_ps(_mm_setr_epi32(0x7fc00000,0,0,0));bool invalidRejected=false;
    try{vp2s::math::largestAxisScale(invalidBasis);}catch(const std::exception &){invalidRejected=true;}check(invalidRejected);
    unsigned axisCases=0;
    for(unsigned seed=0;seed<64;++seed){
        auto &rt=native.runtime();const uint32_t empty=0x180000,type=0x1653d0,head=empty+0x90;
        std::memset(rt.memory.flat(),seed+1,bozo::kFlatBytes);std::memcpy(rt.memory.ram()+empty,&type,4);std::memcpy(rt.memory.ram()+head+8,&head,4);
        const auto before=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
        R5900Context c{};std::memset(c.vu0_vf,seed,sizeof(c.vu0_vf));c.vu0_status=seed*31;c.r[17].ud[0]=empty;const auto saved=c;
        for(uint32_t pc:{0x207720u,0x207724u,0x207728u,0x20772cu,0x207738u}){
            check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));check(!std::memcmp(&c,&saved,sizeof(c)));
        }
        check(!std::memcmp(before.data(),rt.memory.flat(),bozo::kFlatBytes));
        const uint32_t child=0x190000;std::memcpy(rt.memory.ram()+head+8,&child,4);
        for(uint32_t pc:{0x207720u,0x207724u,0x207728u,0x20772cu,0x207738u}){
            bool rejected=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc);}catch(const std::runtime_error &){rejected=true;}
            check(rejected && !std::memcmp(&c,&saved,sizeof(c)));
        }
    }
    for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<64;++seed){
        auto &a=old.runtime();auto &b=native.runtime();
        const uint32_t stack=seed&1?0x70003f00:0x1fff000,source=seed%3?0x180000:0x70000100;
        const uint32_t destination=seed%4?0x190000:source;
        std::memset(a.memory.flat(),seed+1,bozo::kFlatBytes);
        float axis[4]={float(int(seed%9)-4)/8.f,float(int(seed%13)-6)/8.f,float(int(seed%7)-3)/8.f,float(int(seed)-32)/16.f};
        if(seed==0){axis[0]=-0.f;axis[1]=0.f;axis[2]=-0.f;axis[3]=0.f;}
        std::memcpy(a.memory.hostPointer(source,16),axis,16);std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0xa3,sizeof(x.vu0_vf));x.vu0_acc.u[0]=0x1234;x.vu0_status=0xbeef;
        x.r[4].ud[0]=destination;x.r[5].ud[0]=source;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|rounding;_mm_setcsr(control);
        host.referenceAxis=true;a.functions.generated(0x215640)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceAxis=false;b.functions.generated(0x215640)(b.memory.flat(),&y,&b);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)));
        if(std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            uint32_t expected[4],actual[4];std::memcpy(expected,a.memory.hostPointer(destination,16),16);std::memcpy(actual,b.memory.hostPointer(destination,16),16);
            std::fprintf(stderr,"axis seed%u rounding%x: %08x %08x %08x %08x / %08x %08x %08x %08x\n",seed,rounding,expected[0],expected[1],expected[2],expected[3],actual[0],actual[1],actual[2],actual[3]);check(false);
        }++axisCases;
    }
    unsigned overlapCases=0,touching=0;
    for(unsigned part=0;part<3;++part)for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<96;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x41+seed,bozo::kFlatBytes);
        const uint32_t aliases[]={0,0x20000000,0x30000000,0x80000000,0xa0000000};
        const uint32_t stack=seed&1?0x70002e00:0x1ffe000|aliases[(seed/2)%5],first=0x180000|aliases[(seed/3)%5],second=0x184000,pointA=0x188000,pointB=0x70000400;
        uint32_t state=seed*2654435761u+part*97+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;
            return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        for(uint32_t object:{first,second}){
            for(unsigned i=0;i<12;++i){uint32_t v=next(120,12);if(seed%16==3)v=0;std::memcpy(a.memory.hostPointer(object+0x90+4*i,4),&v,4);}
            for(unsigned i=0;i<16;++i){uint32_t v=i%4==3?(i==15?0x3f800000u:0u):next(118,14);std::memcpy(a.memory.hostPointer(object+0x150+4*i,4),&v,4);}
            const uint32_t scale=next(124,6)&0x7fffffff;std::memcpy(a.memory.hostPointer(object+0x204,4),&scale,4);
            uint8_t flags=uint8_t(0x41+seed);flags&=~0x40;if(object==first && seed%5==4)flags|=0x40;std::memcpy(a.memory.hostPointer(object+0x20e,1),&flags,1);
        }
        for(uint32_t point:{pointA,pointB})for(unsigned i=0;i<4;++i){uint32_t v=i==3?next(122,6)&0x7fffffff:next(118,12);if(seed%16==5 && i<3)v=0;std::memcpy(a.memory.hostPointer(point+0x10+4*i,4),&v,4);}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x2468000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0x6b,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xb6,sizeof(x.vu0_acc));x.vu0_status=0x1928;x.vu0_q_bits=0x3f800001;x.vi[0]=0x1928;
        x.r[18].ud[0]=first;x.r[17].ud[0]=second;x.r[20].ud[0]=pointA;x.r[19].ud[0]=pointB;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;
        const uint32_t entry=part==0?0x1f8b30:part==1?0x1f8cc8:0x1f8dbc;if(part==2)x.r[4].ud[0]=pointA+0x10;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(entry)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(entry)(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"overlap part%u rounding%x seed%u pc%x/%x fp%08x/%08x\n",part,rounding,seed,x.pc,y.pc,expectedFp,actualFp);
            for(unsigned i=0;i<sizeof(x);++i)if(reinterpret_cast<const uint8_t *>(&x)[i]!=reinterpret_cast<const uint8_t *>(&y)[i]){std::fprintf(stderr,"CPU byte%x old%02x native%02x\n",i,reinterpret_cast<const uint8_t *>(&x)[i],reinterpret_cast<const uint8_t *>(&y)[i]);break;}
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        if(part==2){float value;std::memcpy(&value,b.memory.hostPointer(stack+0x80,4),4);touching+=value<=0;}
        ++overlapCases;
    }
    {auto &rt=native.runtime();for(unsigned which=0;which<6;++which){constexpr uint32_t stack=0x1ffe000,first=0x180000,second=0x184000;
        R5900Context c{};c.r[29].uw[0]=stack;c.r[18].uw[0]=first;c.r[17].uw[0]=second;c.r[20].uw[0]=0x188000;c.r[19].uw[0]=0x70000400;uint32_t pc=0x1f8b34;
        c.r[2].uw[0]=stack+0xa0;c.r[5].uw[0]=first+0x90;
        if(which==0)c.r[2].uw[0]=stack+0xb0;else if(which==1){pc=0x1f8d1c;c.r[5].uw[0]=second+0x90;}else if(which==2){pc=0x1f8d1c;c.r[2].uw[0]=stack+0x60;}
        else if(which==3){pc=0x1f8e58;c.r[2].uw[0]=stack+0x80;c.r[4].uw[0]=0x188010;c.r[3].uw[0]=0x70000410;c.r[6].uw[0]=first+0x150;c.r[5].uw[0]=second+0x140;}
        else{pc=0x1f8e58;c.r[2].uw[0]=stack+0x80;c.r[4].uw[0]=0x188010;c.r[3].uw[0]=0x70000410;c.r[6].uw[0]=first+0x150;c.r[5].uw[0]=second+0x150;
            const uint32_t bad=which==4?0x7f800000:0x7fc00001;std::memcpy(rt.memory.hostPointer(first+0x150+4,4),&bad,4);}
        const auto before=c;const auto fp=_mm_getcsr();const auto ram=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && _mm_getcsr()==fp && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));
    }}
    std::printf("%u object overlap cases (spill, basis scales, %u touching spheres) and6 readonly rejections passed\n",overlapCases,touching);
    {   // Object pair set-up 1f9130 (mod003 398324): its two 0x7f0 basis-length sites
        // against the ps2f reference (nearest even), the register loads, and rejections.
        auto &rt=native.runtime();constexpr uint32_t stack=0x70003870,first=0x180000,second=0x184000;unsigned setupCases=0;
        const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
        const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
        for(unsigned seed=0;seed<64;++seed){
            uint32_t state=seed*2654435761u+3;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;
                return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
            for(uint32_t object:{first,second})for(unsigned i=0;i<12;++i){const uint32_t v=next(118,14);std::memcpy(rt.memory.hostPointer(object+0x90+4*i,4),&v,4);}
            R5900Context c{};c.r[29].uw[0]=stack;c.r[17].uw[0]=first;c.r[16].uw[0]=second;c.r[9].ud[0]=0x1234;
            for(unsigned pass=0;pass<2;++pass){const uint32_t object=pass?second:first,out=stack+(pass?0x30:0x40);
                c.r[6].uw[0]=object+0x90;c.r[3].uw[0]=out;
                for(uint32_t pc:pass?std::array{0x1f9368u,0x1f936cu,0x1f9370u,0x1f9378u,0x1f937cu,0x1f9384u}:std::array{0x1f92f0u,0x1f92f4u,0x1f92f8u,0x1f9300u,0x1f9304u,0x1f930cu})
                    check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
                check(c.r[9].ud[0]==0);
                uint32_t rows[3][4],expected[3],actual[3],flags=0;std::memcpy(rows,rt.memory.hostPointer(object+0x90,48),48);std::memcpy(actual,rt.memory.hostPointer(out,12),12);
                const auto saved=_mm_getcsr();
                for(unsigned i=0;i<3;++i)expected[i]=add(0,bozo::ps2f::sqrt(add(add(mul(rows[0][i],rows[0][i]),mul(rows[1][i],rows[1][i])),mul(rows[2][i],rows[2][i])),flags));
                _mm_setcsr(saved);check(!std::memcmp(expected,actual,12));
            }
            c.r[5].uw[0]=first+0x150;c.r[4].uw[0]=second+0x150;c.r[3].uw[0]=stack+0x50;
            for(uint32_t pc=0x1f93c4;pc<=0x1f93e4;pc+=4)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            ++setupCases;
        }
        {   // mod003 397780 keeps these registers across its virtual calls: spill to sp+3a0,
            // restore, vf8 from sp+460; a second spill shows the restored set with the new row.
            for(unsigned i=0;i<9;++i){const uint32_t s=0xf8600000u|((i+1)<<16)|i*0x10,l=0xd8400000u|((i+1)<<16)|i*0x10;
                std::memcpy(rt.memory.hostPointer(0x3984f4+4*i,4),&s,4);std::memcpy(rt.memory.hostPointer(0x3985e0+4*i,4),&l,4);}
            const uint32_t moveWord=0xd8480000u;std::memcpy(rt.memory.hostPointer(0x398608,4),&moveWord,4);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[3].uw[0]=stack+0x3a0;c.r[2].uw[0]=stack+0x3a0;
            for(uint32_t pc=0x3984f4;pc<=0x398514;pc+=4)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            check(!std::memcmp(rt.memory.hostPointer(stack+0x3a0,64),rt.memory.hostPointer(first+0x150,64),64));
            check(!std::memcmp(rt.memory.hostPointer(stack+0x3e0,64),rt.memory.hostPointer(second+0x150,64),64));
            check(!std::memcmp(rt.memory.hostPointer(stack+0x420,16),rt.memory.hostPointer(stack+0x50,16),16));
            const std::array<uint32_t,4> moved{0x41200000,0x41a00000,0x41f00000,0x3f800000};std::memcpy(rt.memory.hostPointer(stack+0x460,16),moved.data(),16);
            for(uint32_t pc=0x3985e0;pc<=0x398600;pc+=4)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            c.r[2].uw[0]=stack+0x460;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x398608));
            std::vector<uint8_t> earlier(rt.memory.hostPointer(stack+0x3a0,0x90),rt.memory.hostPointer(stack+0x3a0,0x90)+0x90);std::memcpy(earlier.data()+0x70,moved.data(),16);
            for(uint32_t pc=0x3984f4;pc<=0x398514;pc+=4)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            check(!std::memcmp(earlier.data(),rt.memory.hostPointer(stack+0x3a0,0x90),0x90));
            c.r[3].uw[0]=stack+0x3b0;bool rejected=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x3984f4);}catch(const std::runtime_error &){rejected=true;}check(rejected);
            const uint32_t other=0;std::memcpy(rt.memory.hostPointer(0x3984f4,4),&other,4);check(!vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x3984f4));   // another overlay
        }
        for(unsigned which=0;which<4;++which){R5900Context c{};c.r[29].uw[0]=stack;c.r[17].uw[0]=first;c.r[16].uw[0]=second;
            uint32_t pc=0x1f930c;c.r[6].uw[0]=first+0x90;c.r[3].uw[0]=stack+0x40;
            if(which==0)c.r[6].uw[0]=second+0x90;else if(which==1)c.r[3].uw[0]=stack+0x30;
            else{pc=0x1f93e4;c.r[5].uw[0]=first+0x150;c.r[4].uw[0]=second+0x150;c.r[3].uw[0]=stack+0x50;if(which==2)c.r[5].uw[0]=second+0x150;else c.r[3].uw[0]=stack+0x40;}
            const auto before=c;const auto ram=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
            try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc);}catch(const std::runtime_error &){rejected=true;}
            check(rejected && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));
        }
        std::printf("%u object pair set-ups (1f9130: both basis lengths, register loads), the 397780 spill/restore round trip and 6 rejections passed\n",setupCases);
        {   // SQC2 vf0 (mod027 35fe28): the hard-wired (0, 0, 0, 1) at base + offset, whatever was there.
            const uint32_t word=0xf8400030u;std::memcpy(rt.memory.hostPointer(0x35fe28,4),&word,4);   // SQC2 vf0,0x30(v0)
            std::memset(rt.memory.hostPointer(0x1a0030,16),0x5a,16);R5900Context c{};c.r[2].uw[0]=0x1a0000;
            check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x35fe28));
            const std::array<uint32_t,4> want{0,0,0,0x3f800000u};check(!std::memcmp(rt.memory.hostPointer(0x1a0030,16),want.data(),16));
        }
        {   // mod028 43c120..43c13c: object basis lengths (0x7f0) to sp+70; the W word is untouched.
            constexpr uint32_t words[8]={0xd88c0000,0xd88d0010,0xd88e0020,0,0x4a003fb8,0x48490001,0x27a30070,0xf8660000};
            std::memcpy(rt.memory.hostPointer(0x43c120,32),words,32);
            constexpr uint32_t object=0x1a0000,stack=0x1fff000;
            const float rows[12]={3,0,0,9, 4,2,0,9, 0,0,-5,9};std::memcpy(rt.memory.hostPointer(object+0x90,48),rows,48);
            std::memset(rt.memory.hostPointer(stack+0x70,16),0x5a,16);
            R5900Context c{};c.r[16].uw[0]=object;c.r[4].uw[0]=object+0x90;c.r[29].uw[0]=stack;c.r[3].uw[0]=stack+0x70;c.r[9].ud[0]=77;
            for(uint32_t pc=0x43c120;pc<=0x43c13c;pc+=4)if(pc!=0x43c138)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            float out[4];std::memcpy(out,rt.memory.hostPointer(stack+0x70,16),16);
            check(out[0]==5 && out[1]==2 && out[2]==5 && std::bit_cast<uint32_t>(out[3])==0x5a5a5a5au && c.r[9].ud[0]==0);
            c.r[3].uw[0]+=16;bool bad=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x43c13c);}catch(const std::runtime_error &){bad=true;}check(bad);
            c.r[3].uw[0]-=16;c.r[4].uw[0]+=16;bad=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x43c120);}catch(const std::runtime_error &){bad=true;}check(bad);
            const uint32_t other=0x12345678;std::memcpy(rt.memory.hostPointer(0x43c13c,4),&other,4);
            check(!vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x43c13c));   // another overlay's word: not this site
            // mod001 239b54: the same block, object a0 (base v0), output sp+0.
            constexpr uint32_t second[8]={0xd84c0000,0xd84d0010,0xd84e0020,0,0x4a003fb8,0x48490001,0x27a20000,0xf8460000};
            std::memcpy(rt.memory.hostPointer(0x239b54,32),second,32);std::memset(rt.memory.hostPointer(stack,16),0x5a,16);
            R5900Context d{};d.r[4].uw[0]=object;d.r[2].uw[0]=object+0x90;d.r[29].uw[0]=stack;
            for(uint32_t pc=0x239b54;pc<=0x239b68;pc+=4)check(vp2s::nativeMathInstruction(rt.memory.flat(),&d,&rt,pc));
            check(!vp2s::nativeMathInstruction(rt.memory.flat(),&d,&rt,0x239b6c));   // addiu v0, sp, 0: compiled
            d.r[2].uw[0]=stack;check(vp2s::nativeMathInstruction(rt.memory.flat(),&d,&rt,0x239b70));
            std::memcpy(out,rt.memory.hostPointer(stack,16),16);check(out[0]==5 && out[1]==2 && out[2]==5 && std::bit_cast<uint32_t>(out[3])==0x5a5a5a5au);
            // mod028 4a52a8: 4a50b0's node path, object s1 (base a1), output sp+70.
            constexpr uint32_t third[8]={0xd8ac0000,0xd8ad0010,0xd8ae0020,0,0x4a003fb8,0x48490001,0x27a20070,0xf8460000};
            std::memcpy(rt.memory.hostPointer(0x4a52a8,32),third,32);std::memset(rt.memory.hostPointer(stack+0x70,16),0x5a,16);
            R5900Context e{};e.r[17].uw[0]=object;e.r[5].uw[0]=object+0x90;e.r[29].uw[0]=stack;e.r[2].uw[0]=stack+0x70;
            for(uint32_t pc=0x4a52a8;pc<=0x4a52c4;pc+=4)if(pc!=0x4a52c0)check(vp2s::nativeMathInstruction(rt.memory.flat(),&e,&rt,pc));
            std::memcpy(out,rt.memory.hostPointer(stack+0x70,16),16);check(out[0]==5 && out[1]==2 && out[2]==5 && std::bit_cast<uint32_t>(out[3])==0x5a5a5a5au);
        }
        {   // Battle shape tests (nativeBattleShapes): 4f5630's set-up records the pair registers,
            // 1f8840 tests two spheres on them; 216cb0's translation is -(R^T t).
            const std::pair<uint32_t,uint32_t> words[]={{0x4f57a4,0xfa0c0040},{0x4f580c,0x4be8033c},{0x1f889c,0xf84b0000},{0x216d5c,0xf8bf0030}};
            for(auto [pc,word]:words)std::memcpy(rt.memory.hostPointer(pc,4),&word,4);
            constexpr uint32_t object=0x1a0000,shape=0x1b0000,list=0x1c0000,stack=0x1fff000;
            const float identity[16]={1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};std::memcpy(rt.memory.hostPointer(object+0x150,64),identity,64);
            const float sphere[4]={3,4,0,1},scales[4]={2,1,0,0},centre[4]={0,0,0,1};
            std::memcpy(rt.memory.hostPointer(list+0x10,16),sphere,16);std::memcpy(rt.memory.hostPointer(shape+0x40,16),scales,16);
            std::memcpy(rt.memory.hostPointer(shape,16),centre,16);std::memcpy(rt.memory.hostPointer(shape+0x10,48),identity,48);
            R5900Context c{};c.r[20].uw[0]=object;c.r[16].uw[0]=shape;c.r[17].uw[0]=list;c.r[4].uw[0]=object+0x150;c.r[3].uw[0]=list+0x10;
            check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4f57a4) && vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4f580c));
            float o[4];std::memcpy(o,rt.memory.hostPointer(shape+0x40,16),16);check(o[0]==25-9 && o[3]==1);   // |(3,4,0)|^2 - (1*2 + 1)^2
            // Two spheres 5 apart with radii 1*2 and 1*1: (25) - 9 > 0, no overlap.
            const float a[4]={3,4,0,1},b[4]={0,0,0,1};std::memcpy(rt.memory.hostPointer(0x1d0000,16),a,16);std::memcpy(rt.memory.hostPointer(0x1d0010,16),b,16);
            R5900Context d{};d.r[4].uw[0]=0x1d0000;d.r[5].uw[0]=0x1d0010;d.r[29].uw[0]=stack;d.r[2].uw[0]=stack;
            check(vp2s::nativeMathInstruction(rt.memory.flat(),&d,&rt,0x1f889c));std::memcpy(o,rt.memory.hostPointer(stack,16),16);check(o[0]==16 && o[3]==1);
            // Rigid inverse: rows already transposed in a1; translation (1, 2, 3) through identity, negated.
            const float in[16]={1,0,0,0, 0,1,0,0, 0,0,1,0, 1,2,3,1};std::memcpy(rt.memory.hostPointer(0x1e0000,64),in,64);std::memcpy(rt.memory.hostPointer(0x1e0040,64),identity,64);
            R5900Context e{};e.r[4].uw[0]=0x1e0000;e.r[5].uw[0]=0x1e0040;check(vp2s::nativeMathInstruction(rt.memory.flat(),&e,&rt,0x216d5c));
            std::memcpy(o,rt.memory.hostPointer(0x1e0070,16),16);check(o[0]==-1 && o[1]==-2 && o[2]==-3 && o[3]==1);
        }
        {   // mod028 3fd3f8 point distance: a0 = (|a1 - v1|, dy^2, dz^2, a1.w).
            constexpr uint32_t words[]={0xd8b80000,0xd8790000,0x4bd9c62c,0x4bd8c62a,0x4bc00643,0x4b18c03d,0x4b18ce0a,0x4a1803bd,0x4a0003bf,0x4b000620,0xf8980000};
            std::memcpy(rt.memory.hostPointer(0x3fd3f8,sizeof(words)),words,sizeof(words));
            constexpr uint32_t a=0x1a0000,b=0x1a0010,out=0x1fff000;const float pa[4]={4,6,0,7},pb[4]={1,2,0,9};
            std::memcpy(rt.memory.hostPointer(a,16),pa,16);std::memcpy(rt.memory.hostPointer(b,16),pb,16);
            R5900Context c{};c.r[5].uw[0]=a;c.r[3].uw[0]=b;c.r[4].uw[0]=out;
            for(uint32_t pc=0x3fd3f8;pc<=0x3fd420;pc+=4)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            float o[4];std::memcpy(o,rt.memory.hostPointer(out,16),16);check(o[0]==5 && o[1]==16 && o[2]==0 && o[3]==7);
            c.r[3].uw[0]=0;bool bad=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x3fd3f8);}catch(const std::runtime_error &){bad=true;}check(bad);
            // mod028 381b84 (381ae0): the same block with A = a3, B = v1, OUT = a2.
            uint32_t copy[11];std::memcpy(copy,words,sizeof(copy));copy[0]=0xd8f80000;copy[10]=0xf8d80000;
            std::memcpy(rt.memory.hostPointer(0x381b84,sizeof(copy)),copy,sizeof(copy));
            R5900Context d{};d.r[7].uw[0]=a;d.r[3].uw[0]=b;d.r[6].uw[0]=out+16;
            for(uint32_t pc=0x381b84;pc<=0x381bac;pc+=4)check(vp2s::nativeMathInstruction(rt.memory.flat(),&d,&rt,pc));
            std::memcpy(o,rt.memory.hostPointer(out+16,16),16);check(o[0]==5 && o[1]==16 && o[2]==0 && o[3]==7);
            // mod068 527cdc: A = a0, B = v0, OUT = v1.
            copy[0]=0xd8980000;copy[1]=0xd8590000;copy[10]=0xf8780000;std::memcpy(rt.memory.hostPointer(0x527cdc,sizeof(copy)),copy,sizeof(copy));
            R5900Context e{};e.r[4].uw[0]=a;e.r[2].uw[0]=b;e.r[3].uw[0]=out+32;
            for(uint32_t pc=0x527cdc;pc<=0x527d04;pc+=4)check(vp2s::nativeMathInstruction(rt.memory.flat(),&e,&rt,pc));
            std::memcpy(o,rt.memory.hostPointer(out+32,16),16);check(o[0]==5 && o[1]==16 && o[2]==0 && o[3]==7);
        }
        {   // mod028 macro idioms: normalize25, rows3, xzlength (QMFC2 v0), xzdistance.
            constexpr uint32_t stack=0x1fff000,code=0x4e6600;
            const auto run=[&](const std::vector<uint32_t> &words,R5900Context &c){std::memcpy(rt.memory.hostPointer(code,4*words.size()),words.data(),4*words.size());
                for(size_t i=0;i<words.size();++i){const bool cpu=(words[i]>>26)!=0x12 && (words[i]>>26)!=0x36 && (words[i]>>26)!=0x3e;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,code+4*uint32_t(i))!=cpu);}};
            const auto putv=[&](uint32_t at,std::array<float,4> v){std::memcpy(rt.memory.hostPointer(at,16),v.data(),16);};
            const auto getv=[&](uint32_t at){std::array<float,4> v;std::memcpy(v.data(),rt.memory.hostPointer(at,16),16);return v;};
            R5900Context c{};c.r[29].uw[0]=stack;c.r[2].uw[0]=stack+0xa0;c.r[3].uw[0]=stack+0x60;c.r[4].uw[0]=stack+0xa0;
            putv(stack+0xa0,{0,3,4,7});run({0xd8590000,0x4bc007c3,0x27a200a0,0x4bd9ceaa,0x4b1ad03d,0x4b1afe8a,0x4a7a03be,0x4a0003bf,0x4bc0ce5c,0xf8590000},c);
            {const auto v=getv(stack+0xa0);check(v[0]==0 && v[1]==0.6f && v[2]==0.8f && v[3]==7);}
            putv(stack+0x60,{1,0,0,0});putv(stack+0x70,{0,2,0,0});putv(stack+0x80,{0,0,4,1});putv(stack+0xa0,{1,2,3,9});
            run({0xd8840000,0xd8610000,0xd8620010,0xd8630020,0x4be409bc,0x4be410bd,0x4be4190a,0xf8440000},c);
            {const auto v=getv(stack+0xa0);check(v[0]==1 && v[1]==4 && v[2]==12 && v[3]==3);}
            putv(stack+0xa0,{3,5,4,9});run({0xd8580000,0x4bd8c62a,0x4b18c602,0x4a1803bd,0x4a0003bf,0x4b000620,0x4822c000},c);
            check(c.r[2].uw[0]==std::bit_cast<uint32_t>(5.f) && c.r[2].uw[1]==std::bit_cast<uint32_t>(25.f) && c.r[2].uw[3]==std::bit_cast<uint32_t>(9.f));
            c.r[2].uw[0]=stack+0xa0;putv(stack+0xa0,{4,1,6,2});putv(stack+0xb0,{1,1,2,0});c.r[5].uw[0]=stack+0xb0;
            run({0xd8580000,0xd8b90000,0x4bd9c62c,0x4bd8c62a,0x4b18c602,0x4a1803bd,0x4a0003bf,0x4b000620,0x4823c000},c);   // QMFC2 v1
            check(c.r[3].uw[0]==std::bit_cast<uint32_t>(5.f) && c.r[3].uw[1]==0 && c.r[3].uw[2]==std::bit_cast<uint32_t>(16.f) && c.r[3].uw[3]==std::bit_cast<uint32_t>(2.f));
            // direction26 (4a4094's form): a = (a0), b = (v1) -> (b - a) normalized to (v0), w from a.
            c.r[4].uw[0]=stack+0xc0;c.r[3].uw[0]=stack+0xb0;c.r[2].uw[0]=stack+0xa0;putv(stack+0xc0,{1,1,1,5});putv(stack+0xb0,{1,4,5,8});
            run({0xd89a0000,0xd87b0000,0x4bc007c3,0x27a200a0,0x4bdadeac,0x4bdad6ea,0x4b1bd83d,0x4b1bfeca,0x4a7b03be,0x4a0003bf,0x4bc0d69c,0xf85a0000},c);
            {const auto v=getv(stack+0xa0);check(v[0]==0 && v[1]==0.6f && v[2]==0.8f && v[3]==5);}
            // xzdirection (3dedb0's form): a = (a1), b = (v0) -> xz of (b - a) normalized, y 0, w 1, to (v0).
            c.r[5].uw[0]=stack+0xd0;c.r[2].uw[0]=stack+0x10;putv(stack+0xd0,{1,9,1,1});putv(stack+0x10,{4,-2,5,3});
            run({0xd8b90000,0xd85a0000,0x27a20010,0x4b59d6ec,0x4b5bdf2a,0x4b1ce702,0x4a7c03be,0x4abb033c,0x4a0003bf,0x4bc0dedc,0xf85b0000},c);
            {const auto v=getv(stack+0x10);check(v[0]==0.6f && v[1]==0 && v[2]==0.8f && v[3]==1);}
            {bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,code+40);}catch(const std::runtime_error &){failed=true;}check(failed);}   // the store without its loads
        }
        {   // mod028 4d10a0 battle unit projection: rows kept across two points, program 0.
            constexpr uint32_t words[]={0xd8410000,0xd8420010,0xd8430020,0xd8440030};std::memcpy(rt.memory.hostPointer(0x4d1140,16),words,16);
            const uint32_t loop[3]={0xd8850000,0,0};std::memcpy(rt.memory.hostPointer(0x4d11e0,4),loop,4);
            const uint32_t call[2]={0x4a000038,0x48490001},store=0xf8460000;std::memcpy(rt.memory.hostPointer(0x4d11ec,8),call,8);std::memcpy(rt.memory.hostPointer(0x4d11f8,4),&store,4);
            constexpr uint32_t stack=0x1fff000,rows=0x1a0000;const float m[16]={1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};std::memcpy(rt.memory.hostPointer(rows,64),m,64);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[2].uw[0]=rows;c.r[4].uw[0]=stack+0x80;
            for(uint32_t pc=0x4d1140;pc<=0x4d114c;pc+=4)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            c.r[2].uw[0]=stack+0x70;
            for(float w:{2.f,4.f}){const float v[4]={2,4,6,w};std::memcpy(rt.memory.hostPointer(stack+0x80,16),v,16);
                for(uint32_t pc:{0x4d11e0u,0x4d11ecu,0x4d11f0u,0x4d11f8u})check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
                float o[4];std::memcpy(o,rt.memory.hostPointer(stack+0x70,16),16);check(o[0]==2/w && o[1]==4/w && o[2]==6/w && o[3]==1/w && c.r[9].ud[0]==0);}
        }
        {   // mod028 3e9470 battle side axes: n = normalize(P - P0); a x n, b x n (xyz only), twice.
            constexpr uint32_t b1[]={0xd8410000,0xd8420010,0xc7a001a4,0x27a301a0,0x27a20160,0xe7a00164,0xd87c0000,0xd85b0000,0x4bc00643,0x4bdcdeac,0x4bdad6ea,
                0x4b1bd83d,0x4b1bceca,0x4a7b03be,0x27a20100,0x4a0003bf,0x4bc0d69c,0x4bda0afe,0x4bc1d2ae,0x4bda12fe,0x4bc2d2ee,0xf84a0000,0xf84b0010};
            constexpr uint32_t b2[]={0xd85b0000,0x4bdcdeac,0x4bdad6ea,0x4b1bd83d,0x4b1bceca,0x4a7b03be,0x7ba20160,0x461adec0,0x27a300c0,0x7fa200e0,
                0x4a0003bf,0x4bc0d69c,0x4bda0afe,0x4bc1d2ae,0x4bda12fe,0x4bc2d2ee,0xf86a0000,0xf86b0010};
            std::memcpy(rt.memory.hostPointer(0x3e9764,sizeof(b1)),b1,sizeof(b1));std::memcpy(rt.memory.hostPointer(0x3e989c,sizeof(b2)),b2,sizeof(b2));
            constexpr uint32_t stack=0x1fff000;
            const auto putv=[&](uint32_t at,std::array<float,4> v){std::memcpy(rt.memory.hostPointer(at,16),v.data(),16);};
            const auto getv=[&](uint32_t at){std::array<float,4> v;std::memcpy(v.data(),rt.memory.hostPointer(at,16),16);return v;};
            putv(stack+0x170,{1,0,0,0});putv(stack+0x180,{0,1,0,0});putv(stack+0x1a0,{1,2,3,0});putv(stack+0x160,{1,2,5,0});   // n = (0,0,1)
            putv(stack+0x100,{9,9,9,7});putv(stack+0x110,{9,9,9,8});putv(stack+0xe0,{3,2,3,0});putv(stack+0xc0,{9,9,9,5});putv(stack+0xd0,{9,9,9,6});
            R5900Context c{};c.r[29].uw[0]=stack;
            const auto step=[&](uint32_t pc,uint32_t w){const auto op=w>>26;const bool cop2=op==0x12 || op==0x36 || op==0x3e;
                check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc)==cop2);};
            {   R5900Context fresh{};fresh.r[29].uw[0]=stack+0x400;fresh.r[3].uw[0]=stack+0xc0;bool failed=false;
                try{vp2s::nativeMathInstruction(rt.memory.flat(),&fresh,&rt,0x3e98dc);}catch(const std::runtime_error &){failed=true;}check(failed); }   // block 2 alone
            c.r[2].uw[0]=stack+0x170;
            for(unsigned i=0;i<std::size(b1);++i){const uint32_t pc=0x3e9764+4*i;
                if(pc==0x3e977c){c.r[3].uw[0]=stack+0x1a0;c.r[2].uw[0]=stack+0x160;}
                if(pc==0x3e97b8)c.r[2].uw[0]=stack+0x100;
                step(pc,b1[i]);}
            {const auto x=getv(stack+0x100),y=getv(stack+0x110);   // (1,0,0) x (0,0,1) = (0,-1,0); (0,1,0) x (0,0,1) = (1,0,0)
             check(x[0]==0 && x[1]==-1 && x[2]==0 && x[3]==7 && y[0]==1 && y[1]==0 && y[2]==0 && y[3]==8);}
            c.r[2].uw[0]=stack+0xe0;c.r[3].uw[0]=stack+0xc0;                                                   // P = (3,2,3): n = (1,0,0)
            for(unsigned i=0;i<std::size(b2);++i)step(0x3e989c+4*i,b2[i]);
            {const auto x=getv(stack+0xc0),y=getv(stack+0xd0);   // (1,0,0) x (1,0,0) = 0; (0,1,0) x (1,0,0) = (0,0,-1)
             check(x[0]==0 && x[1]==0 && x[2]==0 && x[3]==5 && y[0]==0 && y[1]==0 && y[2]==-1 && y[3]==6);}
        }
        {   // mod028 481260 corner crosses: axes from the object once, per corner n = normalize(P - sp+50); a x n, b x n to scratchpad (xyz only).
            constexpr uint32_t axes[]={0xd8410000,0xd8420010};
            constexpr uint32_t corner[]={0xd89a0000,0xd91b0000,0x4bc007c3,0x4bdadeac,0x4bdad6ea,0x4b1bd83d,0x4b1bfeca,0x4a7b03be,0x4a0003bf,0x4bc0d69c,
                0x4bda0afe,0x4bc1d2ae,0x4bda12fe,0x4bc2d2ee,0xf86a0000,0xf86b0010};
            std::memcpy(rt.memory.hostPointer(0x4813f4,sizeof(axes)),axes,sizeof(axes));std::memcpy(rt.memory.hostPointer(0x481434,sizeof(corner)),corner,sizeof(corner));
            constexpr uint32_t stack=0x1fff000,obj=0x1b0000,spr=0x70003fe0;
            const auto putv=[&](uint32_t at,std::array<float,4> v){std::memcpy(rt.memory.hostPointer(at,16),v.data(),16);};
            const auto getv=[&](uint32_t at){std::array<float,4> v;std::memcpy(v.data(),rt.memory.hostPointer(at,16),16);return v;};
            putv(obj+0xa20,{1,0,0,0});putv(obj+0xa30,{0,1,0,0});putv(stack+0x50,{1,2,3,0});putv(spr,{9,9,9,7});putv(spr+16,{9,9,9,8});
            R5900Context c{};c.r[29].uw[0]=stack;c.r[2].uw[0]=obj+0xa20;
            for(unsigned i=0;i<2;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4813f4+4*i));
            c.r[4].uw[0]=stack+0x50;c.r[3].uw[0]=spr;
            const auto run=[&](uint32_t point){c.r[8].uw[0]=point;for(unsigned i=0;i<std::size(corner);++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x481434+4*i));};
            putv(stack,{1,2,5,0});run(stack);   // n = (0,0,1): (1,0,0) x n = (0,-1,0); (0,1,0) x n = (1,0,0)
            {const auto x=getv(spr),y=getv(spr+16);check(x[0]==0 && x[1]==-1 && x[2]==0 && x[3]==7 && y[0]==1 && y[1]==0 && y[2]==0 && y[3]==8);}
            putv(stack+16,{3,2,3,0});run(stack+16);   // n = (1,0,0): 0 and (0,0,-1)
            {const auto x=getv(spr),y=getv(spr+16);check(x[0]==0 && x[1]==0 && x[2]==0 && x[3]==7 && y[0]==0 && y[1]==0 && y[2]==-1 && y[3]==8);}
            {R5900Context fresh{};fresh.r[29].uw[0]=stack+0x400;fresh.r[3].uw[0]=spr;bool failed=false;
             try{vp2s::nativeMathInstruction(rt.memory.flat(),&fresh,&rt,0x48146c);}catch(const std::runtime_error &){failed=true;}check(failed);}   // no axes
        }
        {   // mod028 387330 battle camera projection (one of four in-place copies): rows via v1, point s0+120, store sp+30.
            constexpr uint32_t words[]={0xd8610000,0xd8620010,0xd8630020,0xd8640030,0x26030120,0xd8650000,0,0,0x4a000038,0x48490001,0x27a30030,0xf8660000};
            std::memcpy(rt.memory.hostPointer(0x387330,sizeof(words)),words,sizeof(words));
            constexpr uint32_t stack=0x1fff000,rows=0x1a0000,obj=0x1b0000;const float m[16]={2,0,0,0, 0,2,0,0, 0,0,2,0, 0,0,0,1};std::memcpy(rt.memory.hostPointer(rows,64),m,64);
            const float v[4]={1,2,3,4};std::memcpy(rt.memory.hostPointer(obj+0x120,16),v,16);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[3].uw[0]=rows;c.r[9].ud[0]=7;
            for(unsigned i=0;i<std::size(words);++i){const uint32_t pc=0x387330+4*i;
                if(pc==0x387344)c.r[3].uw[0]=obj+0x120;if(pc==0x38735c)c.r[3].uw[0]=stack+0x30;
                const bool cop2=i<4 || i==5 || i==8 || i==9 || i==11;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc)==cop2);}
            float o[4];std::memcpy(o,rt.memory.hostPointer(stack+0x30,16),16);check(o[0]==0.5f && o[1]==1 && o[2]==1.5f && o[3]==0.25f && c.r[9].ud[0]==0);
            {   bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x38735c);}catch(const std::runtime_error &){failed=true;}check(failed); }   // stored twice
        }
        {   // mod028 4981b4 battle ground plane: P0 (0,1,0), P1 (2,1,0), P2 (0,1,-2): n = x * -z = (0,1,0),
            // w = -1 (y = 1); with P2 (0,1,2) n = (0,-1,0) is flipped by the CPU; slope k = -1/n.y.
            struct W{uint32_t pc,word;};
            constexpr W block[]={{0x4981f0,0xda410000},{0x498228,0xda220000},{0x498260,0xda030000},
                {0x498264,0x4bc112ac},{0x498268,0x4bc007c3},{0x49826c,0x4bc11aec},{0x498270,0x4bca552a},{0x498274,0x4b14a03d},{0x498278,0x4b14fd0a},{0x49827c,0x4a7403be},
                {0x498280,0x4bcb5d6a},{0x498284,0x4b15a83d},{0x498288,0x4b15fd4a},{0x49828c,0x4a0003bf},{0x498290,0x4bc0529c},{0x498294,0x4a7503be},{0x498298,0x4b000787},
                {0x49829c,0x4a800781},{0x4982a8,0x4a0003bf},{0x4982ac,0x4bc05adc},{0x4982b0,0x4a3e0798},{0x4982b4,0x4bcb52fe},{0x4982b8,0x4bca5d2e},{0x4982bc,0x4bc1a66a},
                {0x4982c0,0x4b19c83d},{0x4982c4,0x4b19fe4a},{0x4982c8,0x4a39f518},{0x4982d8,0xf8540000},{0x4982f0,0x4bfea518},{0x4982f4,0xf8540000},
                {0x49834c,0x48a80800},{0x498350,0x4b61a798},{0x498354,0xf85e0000}};
            for(const auto &w:block)std::memcpy(rt.memory.hostPointer(w.pc,4),&w.word,4);
            constexpr uint32_t stack=0x1fff000,obj=0x1b0000;
            const auto putv=[&](uint32_t a,std::array<float,4> v){std::memcpy(rt.memory.hostPointer(a,16),v.data(),16);};
            const auto getv=[&](uint32_t a){std::array<float,4> v;std::memcpy(v.data(),rt.memory.hostPointer(a,16),16);return v;};
            const auto run=[&](float z2,bool flip){
                putv(obj+0x8f0,{0,1,0,1});putv(obj+0x900,{2,1,0,1});putv(obj+0x910,{0,1,z2,1});
                R5900Context c{};c.r[29].uw[0]=stack;c.r[18].uw[0]=obj+0x8f0;c.r[17].uw[0]=obj+0x900;c.r[16].uw[0]=obj+0x910;c.r[2].uw[0]=stack+0x80;
                for(const auto &w:block){
                    if((w.pc==0x4982f0 || w.pc==0x4982f4) && !flip)continue;
                    if(w.pc==0x49834c){const float y=getv(stack+0x80)[1];c.r[8].uw[0]=std::bit_cast<uint32_t>(-1.f/y);c.r[2].uw[0]=obj+0x8c0;}
                    check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,w.pc));}
                const auto n=getv(stack+0x80),slope=getv(obj+0x8c0);
                check(n[0]==0 && n[1]==1 && n[2]==0 && n[3]==-1 && slope[0]==0 && slope[1]==0 && slope[2]==0 && slope[3]==1);};
            run(-2,false);run(2,true);
            {   R5900Context fresh{};fresh.r[29].uw[0]=stack+0x400;bool failed=false;
                try{vp2s::nativeMathInstruction(rt.memory.flat(),&fresh,&rt,0x4982d8);}catch(const std::runtime_error &){failed=true;}check(failed); }
        }
        {   // mod028 47e8c0 battle point scale: vf30.x = 0.4f once (47ec2c), then each block scales a point in place (xyz).
            constexpr uint32_t blocks[]={0x47ecc4,0x47ed9c,0x47ef64,0x47f0ec},mulOffset[]={0x14,0x14,0x14,0x14},storeOffset[]={0x24,0x24,0x34,0x34};
            constexpr uint32_t k=0x48a8f000;std::memcpy(rt.memory.hostPointer(0x47ec2c,4),&k,4);
            for(unsigned b=0;b<4;++b){const uint32_t l=0xd8410000,m=0x4bde0858,st=0xf8410000;std::memcpy(rt.memory.hostPointer(blocks[b],4),&l,4);
                std::memcpy(rt.memory.hostPointer(blocks[b]+mulOffset[b],4),&m,4);std::memcpy(rt.memory.hostPointer(blocks[b]+storeOffset[b],4),&st,4);}
            constexpr uint32_t stack=0x1fff000;
            R5900Context c{};c.r[29].uw[0]=stack+0x700;c.r[2].uw[0]=stack+0x220;
            {bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x47ecc4);}catch(const std::runtime_error &){failed=true;}check(failed);}   // no scale yet
            c.r[29].uw[0]=stack;c.r[8].uw[0]=0x3ecccccd;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x47ec2c));
            for(unsigned b=0;b<4;++b){const float v[4]={10,-20,2.5f,7};std::memcpy(rt.memory.hostPointer(stack+0x220,16),v,16);
                check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,blocks[b]) && vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,blocks[b]+mulOffset[b]) &&
                      vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,blocks[b]+storeOffset[b]));
                uint32_t o[4];std::memcpy(o,rt.memory.hostPointer(stack+0x220,16),16);uint32_t f=0;
                for(unsigned i=0;i<3;++i)check(o[i]==bozo::ps2f::mul(std::bit_cast<uint32_t>(v[i]),0x3ecccccdu,f));check(o[3]==std::bit_cast<uint32_t>(7.f));}
            vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x47ecc4);c.r[2].uw[0]=stack+0x210;
            {bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x47ece8);}catch(const std::runtime_error &){failed=true;}check(failed);}   // stored elsewhere
        }
        {   // mod028 49f310 battle half vectors (both copies): (s4), (s3) times vf3.x = 0.5 into the new object +0 / +10, w kept.
            constexpr uint32_t one[]={0x48a81800,0xda810000,0xda620000,0x24a20010,0x4bc30858,0x4bc31098,0xf8a10000,0xf8420000};
            constexpr uint32_t two[]={0x48a81800,0xda810000,0xda620000,0x26220010,0x4bc30858,0x4bc31098,0xfa210000,0xf8420000};
            std::memcpy(rt.memory.hostPointer(0x49f418,sizeof(one)),one,sizeof(one));std::memcpy(rt.memory.hostPointer(0x49f4e0,sizeof(two)),two,sizeof(two));
            constexpr uint32_t stack=0x1fff000,src=0x1b0000,obj=0x1b1000;
            for(unsigned copy=0;copy<2;++copy){const uint32_t base=copy?0x49f4e0:0x49f418;const auto *w=copy?two:one;
                const float a[4]={3,-5,0.25f,9},b[4]={1e6f,2,-8,-1};std::memcpy(rt.memory.hostPointer(src,16),a,16);std::memcpy(rt.memory.hostPointer(src+16,16),b,16);
                R5900Context c{};c.r[29].uw[0]=stack;c.r[20].uw[0]=src;c.r[19].uw[0]=src+16;c.r[8].uw[0]=0x3f000000;c.r[copy?17:5].uw[0]=obj;c.r[2].uw[0]=obj+16;
                if(!copy){bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,base+24);}catch(const std::runtime_error &){failed=true;}check(failed);}   // a store first
                for(unsigned i=0;i<8;++i){const auto op=w[i]>>26;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,base+4*i)==(op==0x12 || op==0x36 || op==0x3e));}
                float o[8];std::memcpy(o,rt.memory.hostPointer(obj,32),32);
                check(o[0]==1.5f && o[1]==-2.5f && o[2]==0.125f && o[3]==9 && o[4]==5e5f && o[5]==1 && o[6]==-4 && o[7]==-1);std::memset(rt.memory.hostPointer(obj,32),0,32);}
        }
        {   // mod028 48b260 battle segment points: A t + B u per record, t from 1 and u from 0 in steps of .25.
            constexpr struct {uint32_t pc,word;} words[]={{0x48b280,0xd8e10000},{0x48b284,0xd8c20000},{0x48b2d0,0x48a81800},{0x48b2d4,0x48a92000},
                {0x48b30c,0x4be309bc},{0x48b310,0x4be410c8},{0x48b314,0xf8a30000}};
            for(const auto &w:words)std::memcpy(rt.memory.hostPointer(w.pc,4),&w.word,4);
            constexpr uint32_t stack=0x1fff000,seg=0x1b0000,rec=0x1b1000;
            const float a[4]={0,10,-4,1},b[4]={8,2,4,1};std::memcpy(rt.memory.hostPointer(seg,16),a,16);std::memcpy(rt.memory.hostPointer(seg+16,16),b,16);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[7].uw[0]=seg;c.r[6].uw[0]=seg+16;
            check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x48b280) && vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x48b284));
            c.r[5].uw[0]=rec;{bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x48b314);}catch(const std::runtime_error &){failed=true;}check(failed);}   // no weights
            for(unsigned k=0;k<5;++k){const float t=1-0.25f*k,uu=0.25f*k;c.r[8].uw[0]=std::bit_cast<uint32_t>(t);c.r[9].uw[0]=std::bit_cast<uint32_t>(uu);c.r[5].uw[0]=rec+0x40*k;
                for(uint32_t pc:{0x48b2d0u,0x48b2d4u,0x48b30cu,0x48b310u,0x48b314u})check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
                float o[4];std::memcpy(o,rt.memory.hostPointer(rec+0x40*k,16),16);
                check(o[0]==8*uu && o[1]==10*t+2*uu && o[2]==-4*t+4*uu && o[3]==1);}
        }
        {   // mod028 48ad10 battle record shift: the saved +10 quadword fills later records even after the source changes.
            const uint32_t lq=0xd8610000,sq=0xf8610000;std::memcpy(rt.memory.hostPointer(0x48adc0,4),&lq,4);std::memcpy(rt.memory.hostPointer(0x48ae0c,4),&sq,4);
            constexpr uint32_t stack=0x1fff000,rec=0x1b0000;const uint32_t v[4]={0xdeadbeef,0x7f800001,0,0x80000000};std::memcpy(rt.memory.hostPointer(rec+0x10,16),v,16);
            R5900Context c{};c.r[29].uw[0]=stack+0x40;c.r[3].uw[0]=rec+0x50;
            {bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x48ae0c);}catch(const std::runtime_error &){failed=true;}check(failed);}   // no save
            c.r[29].uw[0]=stack;c.r[3].uw[0]=rec+0x10;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x48adc0));
            std::memset(rt.memory.hostPointer(rec+0x10,16),0x11,16);   // the CPU shift overwrites the source
            for(unsigned k=1;k<4;++k){c.r[3].uw[0]=rec+0x10+0x40*k;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x48ae0c));
                check(!std::memcmp(rt.memory.hostPointer(rec+0x10+0x40*k,16),v,16));}
        }
        {   // mod028 49c3b0 battle advance: b += c * (a.x k); a.x += a.y k (k = 0.5).
            constexpr struct {uint32_t pc,word;} words[]={{0x49c7a4,0x48a82000},{0x49c7a8,0xdaa10000},{0x49c7ac,0xda420000},{0x49c7b0,0xda230000},
                {0x49c7d0,0x4bc40958},{0x49c7d4,0x4bc011bf},{0x49c7d8,0x4bc51888},{0x49c7dc,0x4b050841},{0x49c7e0,0xfa420000},{0x49c7e8,0xfaa10000}};
            for(const auto &w:words)std::memcpy(rt.memory.hostPointer(w.pc,4),&w.word,4);
            constexpr uint32_t stack=0x1fff000,A=0x1b0000,B=0x1b0010,C=0x1b0020;
            const float a[4]={4,6,7,8},b[4]={1,2,3,9},cc[4]={2,-1,0.5f,5};
            std::memcpy(rt.memory.hostPointer(A,16),a,16);std::memcpy(rt.memory.hostPointer(B,16),b,16);std::memcpy(rt.memory.hostPointer(C,16),cc,16);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[21].uw[0]=A;c.r[18].uw[0]=B;c.r[17].uw[0]=C;c.r[8].uw[0]=0x3f000000;
            {bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x49c7e0);}catch(const std::runtime_error &){failed=true;}check(failed);}   // a store first
            for(const auto &w:words)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,w.pc));
            float oa[4],ob[4];std::memcpy(oa,rt.memory.hostPointer(A,16),16);std::memcpy(ob,rt.memory.hostPointer(B,16),16);
            // a.x k = 2: b = (1 + 4, 2 - 2, 3 + 1, 9); a.x = 4 + 3 = 7
            check(ob[0]==5 && ob[1]==0 && ob[2]==4 && ob[3]==9 && oa[0]==7 && oa[1]==6 && oa[2]==7 && oa[3]==8);
        }
        {   // mod028 4a50b0 battle nearest point: rows (translation (1,2,2)) times an entry point,
            // its distance to the reference in a3's low word, the transformed point stored by SQC2.
            constexpr struct {uint32_t pc,word;} words[]={{0x4a52f4,0xd86b0000},{0x4a52f8,0xd86c0010},{0x4a52fc,0xd86d0020},{0x4a5300,0xd86e0030},{0x4a5304,0xd84f0000},{0x4a5308,0x4bc007c3},
                {0x4a5354,0xd84a0000},{0x4a5358,0x4bea59bc},{0x4a535c,0x4bea60bd},{0x4a5360,0x4bea68be},{0x4a5364,0x4be0728b},{0x4a5370,0x4bcf562c},{0x4a5374,0x4bd8c62a},{0x4a5378,0x4b18c03d},
                {0x4a537c,0x4b18fe0a},{0x4a5380,0x4a1803bd},{0x4a53ac,0x4a0003bf},{0x4a53b0,0x4b000620},{0x4a53b4,0x4827c000},{0x4a5488,0xfa8a0000}};
            for(const auto &w:words)std::memcpy(rt.memory.hostPointer(w.pc,4),&w.word,4);
            constexpr uint32_t stack=0x1ffd000,rows=0x1b0000,ref=0x1b0100,entry=0x1b0200,out=0x1b0300;
            const float m[16]={1,0,0,0, 0,1,0,0, 0,0,1,0, 1,2,2,1},r[4]={0,0,0,1},e[4]={1,2,2,1};
            std::memcpy(rt.memory.hostPointer(rows,64),m,64);std::memcpy(rt.memory.hostPointer(ref,16),r,16);std::memcpy(rt.memory.hostPointer(entry+0x10,16),e,16);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[3].uw[0]=rows;c.r[2].uw[0]=ref;c.r[20].uw[0]=out;
            {bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4a5354);}catch(const std::runtime_error &){failed=true;}check(failed);}   // an entry before the rows
            for(unsigned i=0;i<6;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,words[i].pc));
            c.r[2].uw[0]=entry+0x10;for(unsigned i=6;i<20;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,words[i].pc));
            float d;std::memcpy(&d,&c.r[7].uw[0],4);float o[4];std::memcpy(o,rt.memory.hostPointer(out,16),16);
            check(d==6 && o[0]==2 && o[1]==4 && o[2]==4 && o[3]==1);   // (2,4,4): length 6
            {bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4a53b4);}catch(const std::runtime_error &){failed=true;}check(failed);}   // QMFC2 twice
        }
        {   // mod028 4a55e0: the picked object's rows (*(5136c8)+150) and QMTC2 t0 -> vf5 become the
            // segment test's registers; rows of another object are refused.
            constexpr struct {uint32_t pc,word;} words[]={{0x4a562c,0xd8610000},{0x4a5630,0xd8620010},{0x4a5634,0xd8630020},{0x4a5638,0xd8640030},{0x4a563c,0x48a82800}};
            for(const auto &w:words)std::memcpy(rt.memory.hostPointer(w.pc,4),&w.word,4);
            constexpr uint32_t picked=0x1b4000;std::memcpy(rt.memory.hostPointer(0x5136c8,4),&picked,4);
            R5900Context c{};c.r[29].uw[0]=0x1ffc000;c.r[3].uw[0]=picked+0x150;c.r[8].uw[0]=0x3fc00000;
            for(const auto &w:words)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,w.pc));
            c.r[3].uw[0]=picked+0x160;bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4a563c);}catch(const std::runtime_error &){failed=true;}check(failed);
        }
        {   // main 1f7b80 contact point: centres through each object's rows (+ translation), extents
            // through the 3x3 part with ABS.xyz (rows 1: x -> -y, y -> x; rows 2: identity + (5,0,0)).
            constexpr uint32_t words[]={0xd8cb0000,0xd8ac0000,0xd9090000,0xd8810000,0xd8820010,0xd8830020,0xd8840030,0xd84a0000,0xd8650000,0xd8660010,0xd8670020,0xd8680030,
                0x4beb09bc,0x4beb10bd,0x4beb1aca,0x4bec29bc,0x4bec30bd,0x4bec3b0a,0x4bcb59fd,0x4be909bc,0x4be910bd,0x4bcc61fd,0x4be918be,0x4be0224b,0x4bea29bc,0x4bea30bd,0x4bea38be,0x4be0428b};
            std::memcpy(rt.memory.hostPointer(0x1f7f20,sizeof(words)),words,sizeof(words));
            constexpr uint32_t stores[]={0xf8a90000,0xf88a0000,0xf86b0000,0xf84c0000};std::memcpy(rt.memory.hostPointer(0x1f7fa0,16),stores,16);
            constexpr uint32_t stack=0x1ffb000,r1=0x1b5000,r2=0x1b5100,c1=0x1b5200,c2=0x1b5300;
            const float m1[16]={0,-1,0,0, 1,0,0,0, 0,0,1,0, 0,0,0,1},m2[16]={1,0,0,0, 0,1,0,0, 0,0,1,0, 5,0,0,1};
            const float a[4]={2,3,4,0},b[4]={1,1,1,0},p1[4]={1,2,3,1},p2[4]={1,1,1,1};
            std::memcpy(rt.memory.hostPointer(r1,64),m1,64);std::memcpy(rt.memory.hostPointer(r2,64),m2,64);std::memcpy(rt.memory.hostPointer(stack+0xa0,16),a,16);
            std::memcpy(rt.memory.hostPointer(stack+0x90,16),b,16);std::memcpy(rt.memory.hostPointer(c1,16),p1,16);std::memcpy(rt.memory.hostPointer(c2,16),p2,16);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[6].uw[0]=stack+0xa0;c.r[5].uw[0]=stack+0x90;c.r[8].uw[0]=c1;c.r[4].uw[0]=r1;c.r[2].uw[0]=c2;c.r[3].uw[0]=r2;
            {bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x1f7fa0);}catch(const std::runtime_error &){failed=true;}check(failed);}   // a store first
            for(unsigned i=0;i<28;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x1f7f20+4*i));
            c.r[5].uw[0]=stack+0x80;c.r[4].uw[0]=stack+0x70;c.r[3].uw[0]=stack+0x60;c.r[2].uw[0]=stack+0x50;
            for(unsigned i=0;i<4;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x1f7fa0+4*i));
            float o[4][4];for(unsigned i=0;i<4;++i)std::memcpy(o[i],rt.memory.hostPointer(stack+0x80-0x10*i,16),16);
            check(o[0][0]==2 && o[0][1]==-1 && o[0][2]==3 && o[0][3]==1);   // R1 (1,2,3) = (2,-1,3)
            check(o[1][0]==6 && o[1][1]==1 && o[1][2]==1 && o[1][3]==1);    // (1,1,1) + (5,0,0)
            check(o[2][0]==3 && o[2][1]==2 && o[2][2]==4);                    // |(3,-2,4)|
            check(o[3][0]==1 && o[3][1]==1 && o[3][2]==1);
        }
        {   // mod001 248a00 chain aim: identity rows, P on x, T on y -> dot 0 (x lane only), axis (0, 0, -1);
            // then the identity rotation times q2 gives q2.
            constexpr uint32_t sites[]={0x248b84,0x248b88,0x248b8c,0x248b90,0x248b94,0x248b98,0x248c24,0x248c38,0x248c50,0x248c78,0x248c7c,0x248cac};
            constexpr uint32_t words[]={0xd8610000,0xd8620010,0xd8630020,0xd8640030,0xd8450000,0xda860000,0xf8530000,0x4bd4a2aa,0xf84c0000,0xd8410000,0xd8620000,0xf8410000};
            for(unsigned i=0;i<12;++i)std::memcpy(rt.memory.hostPointer(sites[i],4),&words[i],4);
            constexpr uint32_t stack=0x1ff8000,target=0x1b8000,link=0x1b8100;
            const float m[16]={1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1},p[4]={1,0,0,1},t[4]={0,1,0,7},keep[4]={9,9,9,9};
            std::memcpy(rt.memory.hostPointer(stack+0xe0,64),m,64);std::memcpy(rt.memory.hostPointer(stack+0x140,16),p,16);std::memcpy(rt.memory.hostPointer(target,16),t,16);
            std::memcpy(rt.memory.hostPointer(stack+0xd0,16),keep,16);std::memcpy(rt.memory.hostPointer(stack+0xc0,16),keep,16);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[20].uw[0]=target;c.r[2].uw[0]=stack+0xd0;
            for(unsigned i=0;i<7;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,sites[i]));
            float o[4];std::memcpy(o,rt.memory.hostPointer(stack+0xd0,16),16);check(o[0]==0 && o[1]==9 && o[3]==9);
            c.r[2].uw[0]=stack+0xc0;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x248c38) && vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x248c50));
            std::memcpy(o,rt.memory.hostPointer(stack+0xc0,16),16);check(o[0]==0 && o[1]==0 && o[2]==-1 && o[3]==9);
            {bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x248c50);}catch(const std::runtime_error &){failed=true;}check(failed);}   // an axis without its aim
            const float q1[4]={0,0,0,1},q2[4]={1,2,3,4};std::memcpy(rt.memory.hostPointer(stack+0xb0,16),q1,16);std::memcpy(rt.memory.hostPointer(link+0x30,16),q2,16);
            c.r[2].uw[0]=stack+0xb0;c.r[3].uw[0]=link+0x30;c.r[18].uw[0]=link;
            for(uint32_t pc:{0x248c78u,0x248c7cu,0x248cacu})check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            std::memcpy(o,rt.memory.hostPointer(stack+0xb0,16),16);check(o[0]==1 && o[1]==2 && o[2]==3 && o[3]==4);
        }
        {   // mod028 4ab9a0 vector turn: n = +z, v = +x; dot 0 to v0; with (cos, sin) = (0, 1) the vector
            // becomes (v x (v x n)) = (0, 0, -1), w 1.
            constexpr uint32_t sites[]={0x4ab9c0,0x4ab9fc,0x4aba00,0x4aba10,0x4aba5c,0x4abab4};
            constexpr uint32_t words[]={0xd8ac0000,0xfa0c0000,0xda300000,0x48228000,0xda0e0000,0xfa4c0000};
            for(unsigned i=0;i<6;++i)std::memcpy(rt.memory.hostPointer(sites[i],4),&words[i],4);
            constexpr uint32_t stack=0x1ff7000,from=0x1b9000,to=0x1b9010,vec=0x1b9020;
            const float f[4]={0,0,0,1},t[4]={0,0,2,1},v[4]={1,0,0,0};
            std::memcpy(rt.memory.hostPointer(from,16),f,16);std::memcpy(rt.memory.hostPointer(to,16),t,16);std::memcpy(rt.memory.hostPointer(stack+0x50,16),v,16);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[4].uw[0]=from;c.r[5].uw[0]=to;c.r[16].uw[0]=stack+0x60;c.r[17].uw[0]=stack+0x50;c.r[18].uw[0]=vec;
            for(unsigned i=0;i<4;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,sites[i]));
            float o[4];std::memcpy(o,rt.memory.hostPointer(stack+0x60,16),16);check(o[0]==0 && o[1]==0 && o[2]==1 && o[3]==1);
            check(c.r[2].uw[0]==0);
            const float sc[4]={9,9,0,1};std::memcpy(rt.memory.hostPointer(vec,16),sc,16);   // 214100's (k4, k5, cos, sin)
            check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4aba5c) && vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4abab4));
            std::memcpy(o,rt.memory.hostPointer(vec,16),16);check(o[0]==0 && o[1]==0 && o[2]==-1 && o[3]==1);
        }
        {   // mod028 500df0 palette fade: (t, 1 - t, 255, 0) = (0.25, 0.75, 255, 0), weights 1 (lane 2: 4 -> min 255):
            // A 200 / B 40 -> 80; lane 2: 0.25 * 100 + min(4 * 100, 255) * 0.75 = 216.25 -> 216.
            constexpr uint32_t sites[]={0x500ec0,0x500ec4,0x500ee8,0x500eec,0x500f10};constexpr uint32_t words[]={0xd8610000,0xd8820000,0x48a81800,0x48a92000,0x48281800};
            for(unsigned i=0;i<5;++i)std::memcpy(rt.memory.hostPointer(sites[i],4),&words[i],4);
            constexpr uint32_t stack=0x1ff6000,weights=0x1ba000;const float fade[4]={0.25f,0.75f,255,0},w[4]={1,1,4,1};
            std::memcpy(rt.memory.hostPointer(stack,16),fade,16);std::memcpy(rt.memory.hostPointer(weights,16),w,16);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[3].uw[0]=stack;c.r[4].uw[0]=weights;
            {bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x500f10);}catch(const std::runtime_error &){failed=true;}check(failed);}
            check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x500ec0) && vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x500ec4));
            const uint32_t a[4]={200,0,100,255},b[4]={40,0,100,0};for(unsigned i=0;i<4;++i){c.r[8].uw[i]=a[i];c.r[9].uw[i]=b[i];}
            for(uint32_t pc:{0x500ee8u,0x500eecu,0x500f10u})check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            check(c.r[8].uw[0]==80 && c.r[8].uw[1]==0 && c.r[8].uw[2]==216 && c.r[8].uw[3]==63);   // alpha: 255 * 0.25 = 63.75 -> 63 (the EE code then keeps B's alpha)
        }
        {   // mod028 4a41b0 effect scatter: direction (0, 0, 4) normalized (Q = 1/4), length 2, position
            // (1, 2, 3, 7) -> (1, 2, 5, 7) at sp+d0; the store before the length refuses.
            constexpr uint32_t sites[]={0x4a43d8,0x4a43dc,0x4a43e0,0x4a43e4,0x4a43e8,0x4a43ec,0x4a43f0,0x4a4424,0x4a442c,0x4a4430,0x4a4434,0x4a4438,0x4a443c};
            constexpr uint32_t words[]={0xd8590000,0xdab80000,0x4bc007c3,0x4bd9ceaa,0x4b1ad03d,0x4b1afe8a,0x4a7a03be,0x48a8d800,0x4a0003bf,0x4bc0ce5c,0x4bc0c1bf,0x4bdbce08,0xf8580000};
            for(unsigned i=0;i<13;++i)std::memcpy(rt.memory.hostPointer(sites[i],4),&words[i],4);
            constexpr uint32_t stack=0x1ff7000,position=0x1bb000;const float d[4]={0,0,4,9},p[4]={1,2,3,7};
            std::memcpy(rt.memory.hostPointer(stack+0xe0,16),d,16);std::memcpy(rt.memory.hostPointer(position,16),p,16);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[2].uw[0]=stack+0xe0;c.r[21].uw[0]=position;
            for(unsigned i=0;i<7;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,sites[i]));
            c.r[2].uw[0]=stack+0xd0;{bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4a443c);}catch(const std::runtime_error &){failed=true;}check(failed);}
            c.r[2].uw[0]=stack+0xe0;for(unsigned i=0;i<7;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,sites[i]));
            c.r[8].uw[0]=0x40000000;c.r[2].uw[0]=stack+0xd0;for(unsigned i=7;i<13;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,sites[i]));
            float o[4];std::memcpy(o,rt.memory.hostPointer(stack+0xd0,16),16);check(o[0]==1 && o[1]==2 && o[2]==5 && o[3]==7);
        }
        {   // mod028 4fcae0 label projection: rows loaded once, program 0 in its own blocks and in
            // 4fc750's block (frame 0x70 below); a block outside the loading frame refuses.
            const uint32_t rowsWords[4]={0xd8410000,0xd8420010,0xd8430020,0xd8440030},block[7]={0xd8450000,0,0,0x4a000038,0x48490001,0x27a200e0,0xf8460000};
            std::memcpy(rt.memory.hostPointer(0x4fcb3c,16),rowsWords,16);std::memcpy(rt.memory.hostPointer(0x4fcd14,28),block,28);std::memcpy(rt.memory.hostPointer(0x4fc884,28),block,28);
            constexpr uint32_t stack=0x1ff9000,rows=0x1b7000,point=0x1b7100,out=0x1b7200;
            const float m[16]={2,0,0,0, 0,2,0,0, 0,0,1,0, 0,0,0,1},p[4]={1,2,3,1};   // w' = 1: (2, 4, 3, 1)
            std::memcpy(rt.memory.hostPointer(rows,64),m,64);std::memcpy(rt.memory.hostPointer(point,16),p,16);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[2].uw[0]=rows;
            for(unsigned k=0;k<4;++k)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4fcb3c+4*k));
            for(uint32_t base:{0x4fcd14u,0x4fc884u}){c.r[29].uw[0]=base==0x4fc884?stack-0x70:stack;
                c.r[2].uw[0]=point;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,base));
                check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,base+0xc) && vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,base+0x10));
                c.r[2].uw[0]=out;std::memset(rt.memory.hostPointer(out,16),0,16);check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,base+0x18));
                float o[4];std::memcpy(o,rt.memory.hostPointer(out,16),16);check(o[0]==2 && o[1]==4 && o[2]==3 && o[3]==1);}
            c.r[29].uw[0]=stack-0x40;c.r[2].uw[0]=point;bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4fc884);}catch(const std::runtime_error &){failed=true;}check(failed);
        }
        {   // mod028 4ab680 nearest of points: per point QMFC2 v1 = (distance, dy^2, dz^2, point.w).
            constexpr struct {uint32_t pc,word;} words[]={{0x4ab698,0xd89e0000},{0x4ab69c,0x4bc00643},{0x4ab6b8,0xd8bf0000},{0x4ab6bc,0x4bdeffec},{0x4ab6c4,0x4bdfffea},{0x4ab6c8,0x4b1ff83d},
                {0x4ab6cc,0x4b1fcfca},{0x4ab6d0,0x4a1f03bd},{0x4ab6d4,0x4a0003bf},{0x4ab6d8,0x4b0007e0},{0x4ab6dc,0x4823f800}};
            for(const auto &w:words)std::memcpy(rt.memory.hostPointer(w.pc,4),&w.word,4);
            constexpr uint32_t stack=0x1ffa000,centre=0x1b6000,points=0x1b6100;
            const float ctr[4]={1,1,1,0},pts[2][4]={{4,5,1,7},{1,1,3,9}};
            std::memcpy(rt.memory.hostPointer(centre,16),ctr,16);std::memcpy(rt.memory.hostPointer(points,32),pts,32);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[4].uw[0]=centre;
            {bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4ab6b8);}catch(const std::runtime_error &){failed=true;}check(failed);}   // no centre
            check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4ab698) && vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4ab69c));
            const float want[2][4]={{5,16,0,7},{2,0,4,9}};
            for(unsigned k=0;k<2;++k){c.r[5].uw[0]=points+16*k;for(unsigned i=2;i<11;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,words[i].pc));
                float got[4];std::memcpy(got,c.r[3].uw,16);check(got[0]==want[k][0] && got[1]==want[k][1] && got[2]==want[k][2] && got[3]==want[k][3]);}
        }
        {   // mod028 4a6c40 battle object overlap: the vf1..vf9 spill writes nothing; the block gives
            // ((dx^2 + dy^2) - (r1 a.w + r2 b.w)^2) + dz^2 between two translated centres.
            constexpr uint32_t words[]={0xd8c90000,0xd8aa0000,0xd88b0000,0xd8610000,0xd8620010,0x4a295298,0x4a295ad9,0xd8630020,0xd8640030,0xd8450000,0xd8460010,0xd8470020,0xd8480030,
                0x4bea09bc,0x4bea10bd,0x4bea18be,0x4be0230b,0x4beb29bc,0x4beb30bd,0x4beb38be,0x4be0434b,0x4bcd632c,0x4a2b52a8,0x4bcc632a,0x4b000343,0x4a2a52aa,0x4b0c69bc,0x4b0c68bd,
                0x4b0a68ff,0x4b0c6b0a,0xf8cc0000};
            std::memcpy(rt.memory.hostPointer(0x4a6fa4,sizeof(words)),words,sizeof(words));
            for(unsigned i=0;i<9;++i){const uint32_t w=0xf8400000u|(i+1)<<16|i*16;std::memcpy(rt.memory.hostPointer(0x4a6cf4+4*i,4),&w,4);}
            constexpr uint32_t stack=0x1ffe000,first=0x1b0000,second=0x1b1000,ca=0x1b2000,cb=0x1b3000;
            std::memset(rt.memory.hostPointer(stack+0xd0,0x90),0x77,0x90);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[2].uw[0]=stack+0xd0;
            for(unsigned i=0;i<9;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4a6cf4+4*i));
            check(rt.memory.hostPointer(stack+0xd0,1)[0]==0x77 && rt.memory.hostPointer(stack+0xd0+0x8f,1)[0]==0x77);   // no register words written
            const auto putm=[&](uint32_t at,float tx,float ty,float tz){const float m[16]={1,0,0,0, 0,1,0,0, 0,0,1,0, tx,ty,tz,1};std::memcpy(rt.memory.hostPointer(at,64),m,64);};
            putm(first+0x150,1,0,0);putm(second+0x150,0,2,0);
            const float pa[4]={2,0,0,0.5f},pb[4]={0,1,3,0.25f},radii[4]={2,4,0,0};
            std::memcpy(rt.memory.hostPointer(ca+0x10,16),pa,16);std::memcpy(rt.memory.hostPointer(cb+0x10,16),pb,16);std::memcpy(rt.memory.hostPointer(stack+0xc0,16),radii,16);
            c.r[19].uw[0]=first;c.r[18].uw[0]=second;c.r[21].uw[0]=ca;c.r[20].uw[0]=cb;c.r[6].uw[0]=stack+0xc0;c.r[5].uw[0]=ca+0x10;c.r[4].uw[0]=cb+0x10;c.r[3].uw[0]=first+0x150;c.r[2].uw[0]=second+0x150;
            for(unsigned i=0;i<std::size(words);++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4a6fa4+4*i));
            // t1 = (3, 0, 0, 1), t2 = (0, 3, 3): d = (3, -3, -3); r = 0.5*2 + 0.25*4 = 2: (9 + 9 - 4) + 9 = 23
            float o[4];std::memcpy(o,rt.memory.hostPointer(stack+0xc0,16),16);check(o[0]==23 && o[1]==9 && o[2]==9 && o[3]==1);
            c.r[5].uw[0]=ca;bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4a701c);}catch(const std::runtime_error &){failed=true;}check(failed);   // not a1 = s5+10
        }
        {   // mod001 1f7490 sphere-tree walk (frame 71840): the same overlap of node a0+10 through
            // object s0's +150 and point a1 through matrix a2, scale sp+80 = (+204, f12). It recurses:
            // a child's spill keeps its parent's object-pair registers and its restore gives them back,
            // which a parent shape test (1f8840 at 1f889c) then reads.
            constexpr uint32_t words[]={0xd8490000,0xd86a0000,0xda4b0000,0xd8810000,0xd8820010,0x4a295298,0x4a295ad9,0xd8830020,0xd8840030,0xda250000,0xda260010,0xda270020,0xda280030,
                0x4bea09bc,0x4bea10bd,0x4bea18be,0x4be0230b,0x4beb29bc,0x4beb30bd,0x4beb38be,0x4be0434b,0x4bcd632c,0x4a2b52a8,0x4bcc632a,0x4b000343,0x4a2a52aa,0x4b0c69bc,0x4b0c68bd,
                0x4b0a68ff,0x4b0c6b0a,0x27a20070,0xf84c0000};
            std::memcpy(rt.memory.hostPointer(0x1f7660,sizeof(words)),words,sizeof(words));
            for(unsigned i=0;i<9;++i){const uint32_t sp=0xf8400000u|(i+1)<<16|i*16,rs=0xd8400000u|(i+1)<<16|i*16;
                std::memcpy(rt.memory.hostPointer(0x1f74f4+4*i,4),&sp,4);for(uint32_t b:{0x1f76f4u,0x1f77dcu,0x1f785cu})std::memcpy(rt.memory.hostPointer(b+4*i,4),&rs,4);}
            {const uint32_t w=0xf84b0000;std::memcpy(rt.memory.hostPointer(0x1f889c,4),&w,4);}
            constexpr uint32_t parentSp=0x1ffe000,childSp=0x1ffde00,node=0x1b4000,child=0x1b4100,object=0x1b5000,childObject=0x1b6000,point=0x1b7000,matrix=0x1b7100,identityAt=0x1b7200;
            const auto putm=[&](uint32_t at,float tx,float ty,float tz){const float m[16]={1,0,0,0, 0,1,0,0, 0,0,1,0, tx,ty,tz,1};std::memcpy(rt.memory.hostPointer(at,64),m,64);};
            putm(object+0x150,1,0,0);putm(matrix,0,2,0);putm(childObject+0x150,1,0,0);putm(identityAt,0,0,0);
            std::memcpy(rt.memory.hostPointer(node+4,4),&object,4);std::memcpy(rt.memory.hostPointer(child+4,4),&childObject,4);
            const float pa[4]={2,0,0,0.5f},pb[4]={0,1,3,0.25f},scale[4]={2,4,0,0};
            for(uint32_t n:{node,child})std::memcpy(rt.memory.hostPointer(n+0x10,16),pa,16);std::memcpy(rt.memory.hostPointer(point,16),pb,16);
            for(uint32_t sp:{parentSp,childSp})std::memcpy(rt.memory.hostPointer(sp+0x80,16),scale,16);
            const auto frame=[&](uint32_t sp,uint32_t n,uint32_t obj,uint32_t m){R5900Context c{};c.r[29].uw[0]=sp;c.r[19].uw[0]=n;c.r[16].uw[0]=obj;c.r[18].uw[0]=point;c.r[17].uw[0]=m;
                c.r[3].uw[0]=n+0x10;c.r[4].uw[0]=obj+0x150;c.r[2].uw[0]=sp+0x90;return c;};
            const auto spillAll=[&](R5900Context &c){c.r[2].uw[0]=c.r[29].uw[0]+0x90;for(unsigned i=0;i<9;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x1f74f4+4*i));};
            const auto overlap=[&](R5900Context &c){c.r[2].uw[0]=c.r[29].uw[0]+0x80;for(unsigned i=0;i<std::size(words);++i){if(i==30){c.r[2].uw[0]=c.r[29].uw[0]+0x70;continue;}
                check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x1f7660+4*i));}};
            auto parent=frame(parentSp,node,object,matrix);spillAll(parent);overlap(parent);
            // t1 = (3, 0, 0, 1), t2 = (0, 3, 3): (9 + 9 - 2^2) + 9 = 23
            float o[4];std::memcpy(o,rt.memory.hostPointer(parentSp+0x70,16),16);check(o[0]==23 && o[1]==9 && o[2]==9 && o[3]==1);
            auto kid=frame(childSp,child,childObject,identityAt);spillAll(kid);overlap(kid);
            {auto bad=kid;bad.r[2].uw[0]=childSp+0x80;bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&bad,&rt,0x1f785c);}catch(const std::runtime_error &){failed=true;}check(failed);}   // restore outside sp+90
            kid.r[2].uw[0]=childSp+0x90;for(unsigned i=0;i<9;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&kid,&rt,0x1f785c+4*i));
            // The parent's registers again: spheres (0,0,0,.5) -> (1,0,0) and (0,0,0,.25) -> (0,2,0): (1 + 4 - 2^2) + 0 = 1
            // (the child's identity matrix would give 1 - 4 = -3).
            const float sa[4]={0,0,0,0.5f},sb[4]={0,0,0,0.25f};std::memcpy(rt.memory.hostPointer(0x1d0000,16),sa,16);std::memcpy(rt.memory.hostPointer(0x1d0010,16),sb,16);
            R5900Context d{};d.r[4].uw[0]=0x1d0000;d.r[5].uw[0]=0x1d0010;d.r[29].uw[0]=0x1fff000;d.r[2].uw[0]=0x1fff000;
            check(vp2s::nativeMathInstruction(rt.memory.flat(),&d,&rt,0x1f889c));std::memcpy(o,rt.memory.hostPointer(0x1fff000,16),16);check(o[0]==1);
            {auto bad=parent;bad.r[3].uw[0]=node;bad.r[2].uw[0]=parentSp+0x80;bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&bad,&rt,0x1f7664);}catch(const std::runtime_error &){failed=true;}check(failed);}   // not a0+10
            {R5900Context lone=frame(0x1ffdc00,node,object,matrix);bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&lone,&rt,0x1f76f4+0x20);}catch(const std::runtime_error &){failed=true;}check(failed);}   // a restore with no spill
            parent.r[2].uw[0]=parentSp+0x90;for(unsigned i=0;i<9;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&parent,&rt,0x1f76f4+4*i));
        }
        {   // mod028 4a6470 battle shape tree: loads fill the object-pair rows; a nested call's spill/restore gives the parent's back.
            for(unsigned i=0;i<5;++i){const uint32_t sp=0xf8400000u|(i+1)<<16|i*16,rs=0xd8400000u|(i+1)<<16|i*16;
                std::memcpy(rt.memory.hostPointer(0x4a64e4+4*i,4),&sp,4);std::memcpy(rt.memory.hostPointer(0x4a6770+4*i,4),&rs,4);std::memcpy(rt.memory.hostPointer(0x4a67a0+4*i,4),&rs,4);}
            {const uint32_t w[5]={0xd8650000,0xd8410000,0xd8420010,0xd8430020,0xd8440030};std::memcpy(rt.memory.hostPointer(0x4a664c,20),w,20);}
            {   // mod001 1f5c30 (the same walk): spill sp+70, vf5 = sp+60 (v0), rows from v1 = s0+150, restore 1f5e1c.
                for(unsigned i=0;i<5;++i){const uint32_t sp=0xf8400000u|(i+1)<<16|i*16,rs=0xd8400000u|(i+1)<<16|i*16;
                    std::memcpy(rt.memory.hostPointer(0x1f5c8c+4*i,4),&sp,4);std::memcpy(rt.memory.hostPointer(0x1f5e1c+4*i,4),&rs,4);}
                const uint32_t w[5]={0xd8450000,0xd8610000,0xd8620010,0xd8630020,0xd8640030};std::memcpy(rt.memory.hostPointer(0x1f5df0,20),w,20);
                constexpr uint32_t sp=0x1ffd000,node=0x1b2000;R5900Context c{};c.r[29].uw[0]=sp;c.r[16].uw[0]=node;
                c.r[2].uw[0]=sp+0x70;for(unsigned i=0;i<5;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x1f5c8c+4*i));
                c.r[2].uw[0]=sp+0x60;c.r[3].uw[0]=node+0x150;for(unsigned i=0;i<5;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x1f5df0+4*i));
                c.r[3].uw[0]=node+0x160;bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x1f5df4);}catch(const std::runtime_error &){failed=true;}check(failed);   // rows elsewhere
                c.r[2].uw[0]=sp+0x70;for(unsigned i=0;i<5;++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x1f5e1c+4*i));
            }
            constexpr uint32_t parentSp=0x1ffe000,childSp=0x1ffdf00,nodeA=0x1b0000,nodeB=0x1b1000;
            const auto fill=[&](uint32_t node,float base){float m[16];for(unsigned i=0;i<16;++i)m[i]=base+i;std::memcpy(rt.memory.hostPointer(node+0x150,64),m,64);};
            fill(nodeA,100);fill(nodeB,200);
            const auto run=[&](uint32_t sp,uint32_t node,float scale,const std::vector<uint32_t> &pcs){R5900Context c{};c.r[29].uw[0]=sp;c.r[16].uw[0]=node;
                const float s4[4]={scale,1,0,0};std::memcpy(rt.memory.hostPointer(sp+0x80,16),s4,16);
                for(uint32_t pc:pcs){c.r[2].uw[0]=pc>=0x4a6650 && pc<=0x4a665c?node+0x150:sp+0x90;c.r[3].uw[0]=sp+0x80;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));}};
            std::vector<uint32_t> spill,load,restore;for(unsigned i=0;i<5;++i){spill.push_back(0x4a64e4+4*i);load.push_back(0x4a664c+4*i);restore.push_back(0x4a6770+4*i);}
            run(parentSp,nodeA,3,spill);run(parentSp,nodeA,3,load);
            run(childSp,nodeB,5,spill);run(childSp,nodeB,5,load);run(childSp,nodeB,5,restore);   // the child returns
            run(parentSp,nodeA,3,restore);
            {R5900Context c{};c.r[29].uw[0]=childSp;c.r[2].uw[0]=childSp+0x90;bool failed=false;
             try{for(uint32_t pc:restore)vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc);}catch(const std::runtime_error &){failed=true;}check(failed);}   // no spill left
        }
        {   // mod001 22b734 rows4 idiom: vf5 = ((r1 x + r2 y) + r3 z) + r4 w, w from the point.
            constexpr uint32_t words[]={0xd8650000,0xda410000,0xda420010,0xda430020,0xda440030,0x4be509bc,0x4be510bd,0x4be518be,0x4be5214b,0xf8450000};
            std::memcpy(rt.memory.hostPointer(0x22b734,sizeof(words)),words,sizeof(words));
            constexpr uint32_t pt=0x1b0000,rows=0x1b1000,out=0x1b2000;
            const float p[4]={1,2,3,0.5f},m[16]={1,0,0,0, 0,2,0,0, 0,0,4,0, 10,20,30,2};
            std::memcpy(rt.memory.hostPointer(pt,16),p,16);std::memcpy(rt.memory.hostPointer(rows,64),m,64);
            R5900Context c{};c.r[29].uw[0]=0x1fff000;c.r[3].uw[0]=pt;c.r[18].uw[0]=rows;c.r[2].uw[0]=out;
            for(unsigned i=0;i<std::size(words);++i)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x22b734+4*i));
            float o[4];std::memcpy(o,rt.memory.hostPointer(out,16),16);check(o[0]==6 && o[1]==14 && o[2]==27 && o[3]==1);   // (1+5, 4+10, 12+15, 0+1)
        }
        {   // mod028 47ae40 battle colour pair: (128, 64, 255, 128) and (0, 1, 2, 3) widened by the CPU.
            constexpr uint32_t words[]={0x48a80800,0x48a91000,0x48aa1800,0x4be1093c,0x4be2113c,0x248309a0,0x4bc30858,0x4bc31098,0xf8610000,0xf8620010};
            std::memcpy(rt.memory.hostPointer(0x47ae78,sizeof(words)),words,sizeof(words));
            constexpr uint32_t stack=0x1fff000,obj=0x1b0000;
            R5900Context c{};c.r[29].uw[0]=stack;c.r[3].uw[0]=obj+0x9a0;
            const uint32_t a[4]={128,64,255,128},b[4]={0,1,2,3};for(unsigned i=0;i<4;++i){c.r[8].uw[i]=a[i];c.r[9].uw[i]=b[i];}c.r[10].uw[0]=0x3f000000;
            for(unsigned i=0;i<std::size(words);++i){const uint32_t pc=0x47ae78+4*i;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc)==(i!=5));}
            float o[8];std::memcpy(o,rt.memory.hostPointer(obj+0x9a0,32),32);
            check(o[0]==64 && o[1]==32 && o[2]==127.5f && o[3]==128 && o[4]==0 && o[5]==0.5f && o[6]==1 && o[7]==3);
        }
        {   // mod028 4d7060 FTOI4 of a1 to t2 = sp: (1.5, -2.25, 100.03, 0) -> (24, -36, 1600, 0) (truncated).
            constexpr uint32_t words[]={0xd8a10000,0x4be1097d,0xf9410000};std::memcpy(rt.memory.hostPointer(0x4d7068,sizeof(words)),words,sizeof(words));
            constexpr uint32_t stack=0x1fff000,src=0x1b0000;const float v[4]={1.5f,-2.25f,100.03f,0};std::memcpy(rt.memory.hostPointer(src,16),v,16);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[5].uw[0]=src;c.r[10].uw[0]=stack;
            for(uint32_t pc:{0x4d7068u,0x4d706cu,0x4d7070u})check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            int32_t o[4];std::memcpy(o,rt.memory.hostPointer(stack,16),16);check(o[0]==24 && o[1]==-36 && o[2]==1600 && o[3]==0);
            c.r[10].uw[0]=stack+16;bool failed=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4d7070);}catch(const std::runtime_error &){failed=true;}check(failed);
        }
        {   // mod028 3e8a34 guarded normalise: (2,2,2,2) -> QMFC2 t0 = (16,4,4,4), store (0.5,0.5,0.5,0.5).
            constexpr uint32_t words[14]={0xd8590000,0x4bc007c3,0x27a20150,0x4bf9ceaa,0x4b1ad03d,0x4b1af8be,0x4b1afe8b,0x4828d000,0x11000006,0,0x4a7a03be,0x4a0003bf,0x4be0ce5c,0xf8590000};
            std::memcpy(rt.memory.hostPointer(0x3e8a34,sizeof(words)),words,sizeof(words));
            constexpr uint32_t stack=0x1fff000;const float v[4]={2,2,2,2};std::memcpy(rt.memory.hostPointer(stack+0x150,16),v,16);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[2].uw[0]=stack+0x150;
            for(uint32_t pc=0x3e8a34;pc<=0x3e8a68;pc+=4){const bool cpu=pc==0x3e8a3c || pc==0x3e8a54 || pc==0x3e8a58;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc)!=cpu);}
            check(c.r[8].uw[0]==std::bit_cast<uint32_t>(16.f) && c.r[8].uw[1]==std::bit_cast<uint32_t>(4.f) && c.r[8].uw[3]==std::bit_cast<uint32_t>(4.f));
            float o[4];std::memcpy(o,rt.memory.hostPointer(stack+0x150,16),16);check(o[0]==0.5f && o[1]==0.5f && o[2]==0.5f && o[3]==0.5f);
        }
        {   // mod028 3fd5f0 box sphere: a0+20 = ((max+min)/2, |(max-min)/2|).
            constexpr uint32_t words[]={0xd93b0000,0xd89c0000,0xd91d0000,0x4bdcefe8,0x4bdcefac,0x4bdbffd8,0x4bdbf798,0x4bdef72a,
                0x4a3c01bc,0x4a3c00bd,0x4a3c074a,0x4bfd03be,0x4a0003bf,0x4a20efdc,0xf8ff0000};
            std::memcpy(rt.memory.hostPointer(0x3fd660,sizeof(words)),words,sizeof(words));
            constexpr uint32_t box=0x1a0000,stack=0x1fff000;const float half=0.5f,low[4]={-1,0,2,0},high[4]={5,8,2,0};
            std::memcpy(rt.memory.hostPointer(stack,4),&half,4);std::memcpy(rt.memory.hostPointer(box,16),low,16);std::memcpy(rt.memory.hostPointer(box+0x10,16),high,16);
            R5900Context c{};c.r[4].uw[0]=box;c.r[29].uw[0]=stack;c.r[9].uw[0]=stack;c.r[8].uw[0]=box+0x10;c.r[7].uw[0]=box+0x20;
            for(uint32_t pc=0x3fd660;pc<=0x3fd698;pc+=4)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            float o[4];std::memcpy(o,rt.memory.hostPointer(box+0x20,16),16);check(o[0]==2 && o[1]==4 && o[2]==2 && o[3]==5);   // half (3, 4, 0): radius 5
        }
        {   // mod028 4d7150 vertex pack: FTOI12 of a2 / FTOI4 of a1 to sp+0 (truncate, saturate, denormal 0).
            const std::pair<uint32_t,uint32_t> words[]={{0x4d71cc,0xd8c10000},{0x4d71d0,0x4be1097e},{0x4d71d4,0xf8610000},{0x4d7260,0xd8a10000},{0x4d7264,0x4be1097d},{0x4d7268,0xf8c10000}};
            for(auto [pc,word]:words)std::memcpy(rt.memory.hostPointer(pc,4),&word,4);
            constexpr uint32_t stack=0x1fff000,a1=0x1a0000,a2=0x1a0010;
            const uint32_t vec[4]={std::bit_cast<uint32_t>(0.5f),std::bit_cast<uint32_t>(-1.25f),1u,0x7f800000u};
            const uint32_t pos[4]={std::bit_cast<uint32_t>(10.99f),std::bit_cast<uint32_t>(-3.5f),std::bit_cast<uint32_t>(3.e9f),0xff7fffffu};
            std::memcpy(rt.memory.hostPointer(a2,16),vec,16);std::memcpy(rt.memory.hostPointer(a1,16),pos,16);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[6].uw[0]=a2;c.r[5].uw[0]=a1;c.r[3].uw[0]=stack;
            for(uint32_t pc:{0x4d71ccu,0x4d71d0u,0x4d71d4u})check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            int32_t o[4];std::memcpy(o,rt.memory.hostPointer(stack,16),16);check(o[0]==2048 && o[1]==-5120 && o[2]==0 && o[3]==0x7fffffff);
            c.r[6].uw[0]=stack;for(uint32_t pc:{0x4d7260u,0x4d7264u,0x4d7268u})check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            std::memcpy(o,rt.memory.hostPointer(stack,16),16);check(o[0]==175 && o[1]==-56 && o[2]==0x7fffffff && o[3]==int32_t(0x80000000u));
        }
        {   // mod028 48ee30: shape origin s5+1b0 = (a1+10).xyz [+ (s4+c, s4+1c, s4+2c)], w = 1.
            const std::pair<uint32_t,uint32_t> words[]={{0x48ee74,0xd8610000},{0x48ee88,0x4a20005b},{0x48ef84,0xd8820000},{0x48ef88,0x4bc20868},{0x48ef94,0xf8610000}};
            for(auto [pc,word]:words)std::memcpy(rt.memory.hostPointer(pc,4),&word,4);
            constexpr uint32_t shape=0x1a0000,source=0x1b0000,stack=0x1fff000;
            for(bool offset:{false,true}){
                const float in[4]={1.5f,-2,3,7};std::memcpy(rt.memory.hostPointer(source+0x10,16),in,16);
                const float add[4]={0.25f,4,-3,-99};std::memcpy(rt.memory.hostPointer(stack+0x90,16),add,16);
                std::memset(rt.memory.hostPointer(shape+0x1b0,16),0x5a,16);
                R5900Context c{};c.r[21].uw[0]=shape;c.r[5].uw[0]=source;c.r[3].uw[0]=source+0x10;c.r[29].uw[0]=stack;c.r[20].uw[0]=offset?0x1c0000:0;
                check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x48ee74) && vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x48ee88));
                if(offset){c.r[4].uw[0]=stack+0x90;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x48ef84) && vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x48ef88));}
                c.r[3].uw[0]=shape+0x1b0;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x48ef94));
                float out[4];std::memcpy(out,rt.memory.hostPointer(shape+0x1b0,16),16);
                check(offset?(out[0]==1.75f && out[1]==2 && out[2]==0):(out[0]==1.5f && out[1]==-2 && out[2]==3));check(out[3]==1);
                bool bad=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x48ef94);}catch(const std::runtime_error &){bad=true;}check(bad);   // stored once per load
            }
        }
        {   // mod028 48edc0..48ee18: shape world point a0+1c0 = rows(a1) . (a0+1b0).
            constexpr uint32_t words[]={0xd8cf0000,0xd8a10000,0xd8a20010,0xd8a30020,0xd8a40030,0x4bc00183,0x4bef0aea,0x4bef132a,0x4bef1b6a,0x4bef23aa,
                0x4b0b583d,0x4b0b30be,0x4b0b33cb,0x4a8c603c,0x4a8c30be,0x4a8c33cb,0x4a4d683d,0x4a4d30bc,0x4a4d33cb,0x4a2e703d,0x4a2e00be,0x4a2e03c8,0xf86f0000};
            std::memcpy(rt.memory.hostPointer(0x48edc0,sizeof(words)),words,sizeof(words));
            constexpr uint32_t shape=0x1a0000,matrix=0x1b0000;
            const float rows[16]={1,0,0,10, 0,2,0,20, 0,0,1,-5, 0,0,0,1},point[4]={3,4,5,1};
            std::memcpy(rt.memory.hostPointer(matrix,64),rows,64);std::memcpy(rt.memory.hostPointer(shape+0x1b0,16),point,16);
            R5900Context c{};c.r[4].uw[0]=shape;c.r[5].uw[0]=matrix;c.r[6].uw[0]=shape+0x1b0;c.r[3].uw[0]=shape+0x1c0;
            for(uint32_t pc=0x48edc0;pc<=0x48ee18;pc+=4)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            float out[4];std::memcpy(out,rt.memory.hostPointer(shape+0x1c0,16),16);check(out[0]==13 && out[1]==28 && out[2]==0 && out[3]==1);
            c.r[3].uw[0]+=16;bool bad=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x48ee18);}catch(const std::runtime_error &){bad=true;}check(bad);
        }
        {   // mod028 48f0d0: box centre copy, eight corners, optional in-place transform.
            constexpr uint32_t build[]={0xd8e10000,0xd8aa0000,0xf8610000,0x4a21033c,0x4be20b3c,0x4be30b3c,0x4be40b3c,0x4be50b3c,0x4be60b3c,0x4be70b3c,0x4be80b3c,0x24830020,
                0x4b4a0868,0x4b0a1080,0x4bca18e8,0x4b8a2128,0x4a4a2942,0x4aca39e8,0x4a8a4201,0x4a8a0845,0x4aca10ac,0x4a4a2106,0x4b8a296c,0x4bca31ac,0x4b0a39c4,0x4b4a422c,
                0xf8610000,0xf8620010,0xf8630020,0xf8640030,0xf8650040,0xf8660050,0xf8670060,0x10c00021,0xf8680070};
            constexpr uint32_t loop[]={0xd8cb0000,0xd8cc0010,0xd8cd0020,0xd8ce0030,0x4bc007c3,0x24850020,0x10000015,0x0000202d,0x4be15d6a,0x4be165aa,0x4be16dea,0x4be1762a,
                0xd8a10010,0x4b15a83d,0x4b15f8be,0x4b15f94b,0x4a96b03c,0x4a96f8be,0x4a96f94b,0x4a57b83d,0x4a57f8bc,0x4a57f94b,0x4a38c03d,0x4a3800be,0x4a380148,0xf8a50000};
            std::memcpy(rt.memory.hostPointer(0x48f0dc,sizeof(build)),build,sizeof(build));std::memcpy(rt.memory.hostPointer(0x48f168,sizeof(loop)),loop,sizeof(loop));
            constexpr uint32_t box=0x1a0000,source=0x1b0000,matrix=0x1c0000;
            const float extent[4]={1,2,3,0},centre[4]={10,20,30,std::bit_cast<float>(0x5a5a5a5au)};
            std::memcpy(rt.memory.hostPointer(source,16),extent,16);std::memcpy(rt.memory.hostPointer(source+0x10,16),centre,16);
            const float rows[16]={1,0,0,100, 0,1,0,0, 0,0,1,0, 0,0,0,1};std::memcpy(rt.memory.hostPointer(matrix,64),rows,64);
            R5900Context c{};c.r[4].uw[0]=box;c.r[5].uw[0]=source;c.r[7].uw[0]=source+0x10;c.r[3].uw[0]=box+0x180;c.r[6].uw[0]=matrix;
            check(!vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x48f108));   // addiu: compiled
            for(uint32_t pc=0x48f0dc;pc<=0x48f164;pc+=4)if(pc!=0x48f108 && pc!=0x48f160)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            check(!std::memcmp(rt.memory.hostPointer(box+0x180,16),centre,16));
            constexpr float want[8][3]={{11,18,33},{11,18,27},{11,22,33},{11,22,27},{9,18,33},{9,18,27},{9,22,33},{9,22,27}};
            for(unsigned k=0;k<8;++k){float v[4];std::memcpy(v,rt.memory.hostPointer(box+0x20+k*16,16),16);check(v[0]==want[k][0] && v[1]==want[k][1] && v[2]==want[k][2] && v[3]==1);}
            for(unsigned k=0;k<8;++k){c.r[4].uw[0]=k;c.r[5].uw[0]=box+0x20+k*16;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x48f1cc));
                float v[4];std::memcpy(v,rt.memory.hostPointer(box+0x20+k*16,16),16);check(v[0]==want[k][0]+100 && v[1]==want[k][1] && v[3]==1);}
            // 48efd0: the same box to world through a1's rows: corners to +a0.., centre to +190.
            const uint32_t load[]={0xd8a10000,0xd8a20010,0xd8a30020,0xd8a40030,0x4bc007c3};std::memcpy(rt.memory.hostPointer(0x48efdc,20),load,20);
            uint32_t loopBlock[18]={0xd8ef0000,0x4bef0aea,0x4bef132a,0x4bef1b6a,0x4bef23aa,0x4b0b583d,0x4b0bf8be,0x4b0bfbcb,0x4a8c603c,0x4a8cf8be,0x4a8cfbcb,
                0x4a4d683d,0x4a4df8bc,0x4a4dfbcb,0x4a2e703d,0x4a2e00be,0x4a2e03c8,0xf8cf0000};
            std::memcpy(rt.memory.hostPointer(0x48f000,72),loopBlock,72);loopBlock[0]=0xd8af0000;loopBlock[17]=0xf86f0000;std::memcpy(rt.memory.hostPointer(0x48f06c,72),loopBlock,72);
            const float shift[16]={1,0,0,0, 0,1,0,-20, 0,0,1,0, 0,0,0,1};std::memcpy(rt.memory.hostPointer(matrix,64),shift,64);
            const float point[4]={1,2,3,1};std::memcpy(rt.memory.hostPointer(box+0x180,16),point,16);
            R5900Context w{};w.r[4].uw[0]=box;w.r[5].uw[0]=matrix;
            for(uint32_t pc=0x48efdc;pc<=0x48efec;pc+=4)check(vp2s::nativeMathInstruction(rt.memory.flat(),&w,&rt,pc));
            for(unsigned k=0;k<8;++k){w.r[5].uw[0]=k;w.r[7].uw[0]=box+0x20+k*16;w.r[6].uw[0]=box+0xa0+k*16;
                for(uint32_t pc=0x48f000;pc<=0x48f044;pc+=4)check(vp2s::nativeMathInstruction(rt.memory.flat(),&w,&rt,pc));
                float v[4];std::memcpy(v,rt.memory.hostPointer(box+0xa0+k*16,16),16);check(v[0]==want[k][0]+100 && v[1]==want[k][1]-20 && v[2]==want[k][2] && v[3]==1);}
            w.r[5].uw[0]=box+0x180;w.r[3].uw[0]=box+0x190;
            for(uint32_t pc=0x48f06c;pc<=0x48f0b0;pc+=4)check(vp2s::nativeMathInstruction(rt.memory.flat(),&w,&rt,pc));
            float v[4];std::memcpy(v,rt.memory.hostPointer(box+0x190,16),16);check(v[0]==1 && v[1]==-18 && v[2]==3 && v[3]==1);
            bool bad=false;try{vp2s::nativeMathInstruction(rt.memory.flat(),&w,&rt,0x48f0b0);}catch(const std::runtime_error &){bad=true;}check(bad);   // matrix released
        }
        {   // mod001 284050 particle integrator (nativeParticleStep's site table): pos + vel * dt to the output and back, w kept.
            const std::pair<uint32_t,uint32_t> words[]={{0x2840d0,0x48a31800},{0x28414c,0xd9410000},{0x284150,0xd8620000},{0x284178,0x4bc00abc},{0x28417c,0x4bc31048},{0x284188,0xf8610000},{0x28418c,0xf9410000}};
            for(auto [pc,word]:words)std::memcpy(rt.memory.hostPointer(pc,4),&word,4);
            constexpr uint32_t rec=0x1a0080,out=0x1b0000;const float dt=0.5f;
            const float pos[4]={1,2,3,std::bit_cast<float>(0x12345678u)},vel[4]={4,-6,8,99};
            std::memcpy(rt.memory.hostPointer(rec,16),pos,16);std::memcpy(rt.memory.hostPointer(rec+0x10,16),vel,16);
            R5900Context c{};c.r[3].uw[0]=std::bit_cast<uint32_t>(dt);c.r[10].uw[0]=rec;
            check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2840d0) && vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x28414c));
            c.r[3].uw[0]=rec+0x10;for(uint32_t pc:{0x284150u,0x284178u,0x28417cu})check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            c.r[3].uw[0]=out;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x284188) && vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x28418c));
            float o[4],p[4];std::memcpy(o,rt.memory.hostPointer(out,16),16);std::memcpy(p,rt.memory.hostPointer(rec,16),16);
            check(o[0]==3 && o[1]==-1 && o[2]==7 && !std::memcmp(o,p,16) && std::bit_cast<uint32_t>(p[3])==0x12345678u);
            // 28a8a0, the copy of 409fb0: its dt site 40a030 - 17f710.
            const uint32_t q=0x48a31800;std::memcpy(rt.memory.hostPointer(0x28a920,4),&q,4);c.r[3].uw[0]=std::bit_cast<uint32_t>(dt);
            check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x28a920));
        }
    }
    unsigned projectionCases=0,projectedPoints=0;
    for(unsigned part=0;part<2;++part)for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<96;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x29+seed,bozo::kFlatBytes);
        const uint32_t aliases[]={0,0x20000000,0x30000000,0x80000000,0xa0000000};
        const uint32_t stack=seed&1?0x70002f00:0x1ffd800|aliases[(seed/2)%5],camera=0x1b0000|aliases[(seed/3)%5],owner=0x1a8000,table=0x1c0000;
        uint32_t state=seed*40503u+part*7+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;
            return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        for(unsigned row=0;row<4;++row)for(unsigned k=0;k<4;++k){uint32_t v=row<3?next(118,12):k==3?(next(127,2)&0x7fffffff):next(110,8);
            if(seed%16==2 && row<3 && k!=row)v=k%2?0x80000000u:0;std::memcpy(a.memory.hostPointer(camera+0x330+16*row+4*k,4),&v,4);}
        for(uint32_t at:{0x270u,0x274u,0x278u,0x2a4u}){const uint32_t v=next(122,8);std::memcpy(a.memory.hostPointer(camera+at,4),&v,4);}
        std::memcpy(a.memory.hostPointer(owner+0x8c,4),&table,4);const uint8_t count=uint8_t(seed%6),both=uint8_t(seed&2);
        std::memcpy(a.memory.hostPointer(owner+0x9a,1),&count,1);std::memcpy(a.memory.hostPointer(owner+0x99,1),&both,1);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x1357000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(int(i)-16)/16;}
        std::memset(x.vu0_vf,0x7d,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xd7,sizeof(x.vu0_acc));x.vu0_status=0x2b2b;x.vu0_q_bits=0x40000001;x.vi[0]=0x2b2b;
        x.r[16].ud[0]=owner;x.r[18].ud[0]=camera;x.r[2].ud[0]=camera+0x330;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;
        const uint32_t entry=part?0x20a00c:0x209bd0;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(entry)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(entry)(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"projection part%u rounding%x seed%u pc%x/%x fp%08x/%08x\n",part,rounding,seed,x.pc,y.pc,expectedFp,actualFp);
            for(unsigned i=0;i<sizeof(x);++i)if(reinterpret_cast<const uint8_t *>(&x)[i]!=reinterpret_cast<const uint8_t *>(&y)[i]){std::fprintf(stderr,"CPU byte%x old%02x native%02x\n",i,reinterpret_cast<const uint8_t *>(&x)[i],reinterpret_cast<const uint8_t *>(&y)[i]);break;}
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        projectedPoints+=part?1:count*(both?2:1);++projectionCases;
    }
    {auto &rt=native.runtime();for(unsigned which=0;which<5;++which){constexpr uint32_t stack=0x1ffd800,camera=0x1b0000,owner=0x1a8000;
        R5900Context c{};c.r[29].uw[0]=stack;c.r[18].uw[0]=camera;c.r[16].uw[0]=owner;c.r[2].uw[0]=stack+0x60;uint32_t pc=0x20a084;
        const uint32_t table=0x1c0000;std::memcpy(rt.memory.hostPointer(owner+0x8c,4),&table,4);
        const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},point[4]={1,2,3,1};std::memcpy(rt.memory.hostPointer(camera+0x330,64),identity,64);std::memcpy(rt.memory.hostPointer(stack+0x60,16),point,16);
        if(which==0){pc=0x209bd0;}else if(which==1)c.r[2].uw[0]=stack+0x70;else if(which==2){const float flat[4]={0,0,0,0};std::memcpy(rt.memory.hostPointer(camera+0x360,16),flat,16);}
        else if(which==3){pc=0x209c94;c.r[11].uw[0]=stack+0x80;c.r[18].uw[0]=table+0x3e8c-0x330;}else c.r[18].uw[0]=0;
        const auto before=c;const auto fp=_mm_getcsr();const auto ram=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && _mm_getcsr()==fp && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));
    }}
    std::printf("%u camera projection runs (%u projected points) and5 readonly rejections passed\n",projectionCases,projectedPoints);
    // Flare light projection (flare owner block 256068..256188): rows SP+360 as
    // program 0's column registers, light SP+340, result SP+2c0; ps2f oracle.
    unsigned flareCases=0;
    {auto &rt=native.runtime();
        const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
        const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
        uint64_t state=0x9e3779b97f4a7c15ull;const auto next=[&](){state^=state<<13;state^=state>>7;state^=state<<17;return state;};
        const auto value=[&](float range){return float(int64_t(next()%2000001)-1000000)/1000000.f*range;};
        const auto run=[&](uint32_t stack,const R5900Context &start,std::array<uint32_t,4> &out){
            R5900Context c=start;
            for(uint32_t pc:{0x256068u,0x25606cu,0x256070u,0x256074u}){c.r[3].uw[0]=stack+0x360;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));}
            c.r[3].uw[0]=stack+0x340;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x256170));
            check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x25617c));c.r[9].ud[0]=0x1234;
            check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x256180));check(c.r[9].ud[0]==0);
            c.r[3].uw[0]=stack+0x2c0;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x256188));
            std::memcpy(out.data(),rt.memory.hostPointer(stack+0x2c0,16),16);
        };
        for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
            const uint32_t stack=seed&1?0x70002c00:0x1ffd000;R5900Context c{};c.r[29].uw[0]=stack;
            float rows[16],light[4];for(float &x:rows)x=value(4.f);for(unsigned i=0;i<3;++i)light[i]=value(5000.f);light[3]=1.f;
            rows[15]=std::fabs(rows[15])+2.f;   // keep p.w away from zero
            if(seed%8==3)rows[1]=std::bit_cast<float>(0x00000123u);   // a denormal input reads as zero
            std::memcpy(rt.memory.hostPointer(stack+0x360,64),rows,64);std::memcpy(rt.memory.hostPointer(stack+0x340,16),light,16);
            uint32_t m[4][4],v[4];std::memcpy(m,rows,64);std::memcpy(v,light,16);if(seed%8==3)m[0][1]=0;
            uint32_t p[4],expected[4],flags=0;
            for(unsigned i=0;i<4;++i)p[i]=add(add(add(mul(m[0][i],v[0]),mul(m[1][i],v[1])),mul(m[2][i],v[2])),mul(m[3][i],v[3]));
            const auto savedCsr=_mm_getcsr();const auto q=bozo::ps2f::div(0x3f800000,p[3],flags);_mm_setcsr(savedCsr);
            for(unsigned i=0;i<3;++i)expected[i]=mul(p[i],q);expected[3]=mul(0x3f800000,q);
            _MM_SET_ROUNDING_MODE(rounding);const auto csr=_mm_getcsr();std::array<uint32_t,4> out{};run(stack,c,out);
            check(_mm_getcsr()==csr);_MM_SET_ROUNDING_MODE(_MM_ROUND_NEAREST);
            check(!std::memcmp(out.data(),expected,16));++flareCases;
        }
        // Rejections: wrong sources/destination, store without loads, zero depth.
        const uint32_t stack=0x1ffd000;
        for(unsigned which=0;which<5;++which){R5900Context c{};c.r[29].uw[0]=stack;bool rejected=false;
            try{
                if(which==0){c.r[3].uw[0]=stack+0x350;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x256068);}
                else if(which==1){c.r[3].uw[0]=stack+0x330;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x256170);}
                else if(which==2){c.r[3].uw[0]=stack+0x2c0;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x256188);}
                else{
                    float rows[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,which==3?0.f:1.f},light[4]={1,2,3,1};
                    std::memcpy(rt.memory.hostPointer(stack+0x360,64),rows,64);std::memcpy(rt.memory.hostPointer(stack+0x340,16),light,16);
                    for(uint32_t pc:{0x256068u,0x25606cu,0x256070u,0x256074u}){c.r[3].uw[0]=stack+0x360;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc);}
                    c.r[3].uw[0]=stack+0x340;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x256170);
                    c.r[3].uw[0]=which==3?stack+0x2c0:stack+0x2d0;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x256188);
                }
            }catch(const std::runtime_error &){rejected=true;}
            check(rejected);
        }
    }
    std::printf("%u flare projection cases and 5 rejections passed\n",flareCases);
    // Two-point projection (mod001 2dfeb8, block 2e0184): rows SP+120 loaded once
    // as program 0's column registers, then two projections of SP+160 to SP+170
    // with the point rewritten in between (FPU code); ps2f oracle.
    unsigned pairCases=0;
    {auto &rt=native.runtime();
        const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
        const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
        uint64_t state=0x2545f4914f6cdd1dull;const auto next=[&](){state^=state<<13;state^=state>>7;state^=state<<17;return state;};
        const auto value=[&](float range){return float(int64_t(next()%2000001)-1000000)/1000000.f*range;};
        const auto oracle=[&](const uint32_t (&m)[4][4],const uint32_t (&v)[4],uint32_t (&expected)[4]){
            uint32_t p[4],flags=0;
            for(unsigned i=0;i<4;++i)p[i]=add(add(add(mul(m[0][i],v[0]),mul(m[1][i],v[1])),mul(m[2][i],v[2])),mul(m[3][i],v[3]));
            const auto savedCsr=_mm_getcsr();const auto q=bozo::ps2f::div(0x3f800000,p[3],flags);_mm_setcsr(savedCsr);
            for(unsigned i=0;i<3;++i)expected[i]=mul(p[i],q);expected[3]=mul(0x3f800000,q);
        };
        const auto project=[&](R5900Context &c,uint32_t stack,uint32_t load,uint32_t call,uint32_t status,uint32_t store){
            c.r[3].uw[0]=stack+0x160;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,load));
            check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,call));c.r[9].ud[0]=0x1234;
            check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,status));check(c.r[9].ud[0]==0);
            c.r[3].uw[0]=stack+0x170;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,store));
        };
        for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
            const uint32_t stack=seed&1?0x70002c00:0x1ffd000;R5900Context c{};c.r[29].uw[0]=stack;
            float rows[16],points[2][4];for(float &x:rows)x=value(4.f);rows[15]=std::fabs(rows[15])+2.f;
            for(unsigned k=0;k<3;++k)rows[4*k+3]=value(1.e-4f);   // keep p.w away from zero
            for(auto &point:points){for(unsigned i=0;i<3;++i)point[i]=value(3000.f);point[3]=1.f;}
            if(seed%8==5)rows[6]=std::bit_cast<float>(0x80000077u);   // a denormal input reads as zero
            std::memcpy(rt.memory.hostPointer(stack+0x120,64),rows,64);
            uint32_t m[4][4];std::memcpy(m,rows,64);if(seed%8==5)m[1][2]=0;
            _MM_SET_ROUNDING_MODE(rounding);const auto csr=_mm_getcsr();
            for(uint32_t pc:{0x2e0234u,0x2e0238u,0x2e023cu,0x2e0240u}){c.r[3].uw[0]=stack+0x120;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));}
            // The rows are registers after the loads: later RAM writes must not reach them.
            std::memset(rt.memory.hostPointer(stack+0x120,64),0xa5,64);
            for(unsigned k=0;k<2;++k){
                std::memcpy(rt.memory.hostPointer(stack+0x160,16),points[k],16);uint32_t v[4],expected[4];std::memcpy(v,points[k],16);oracle(m,v,expected);
                if(k==0)project(c,stack,0x2e0248,0x2e0254,0x2e0258,0x2e0260);else project(c,stack,0x2e02e8,0x2e02f4,0x2e02f8,0x2e0300);
                check(!std::memcmp(rt.memory.hostPointer(stack+0x170,16),expected,16));++pairCases;
            }
            check(_mm_getcsr()==csr);_MM_SET_ROUNDING_MODE(_MM_ROUND_NEAREST);
        }
        // Rejections: wrong row/point source, wrong destination, a store without a
        // fresh point, a store without the rows, zero depth.
        const uint32_t stack=0x1ffd000;
        for(unsigned which=0;which<6;++which){R5900Context c{};c.r[29].uw[0]=stack;bool rejected=false;
            float rows[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,which==5?0.f:1.f},point[4]={1,2,3,1};
            std::memcpy(rt.memory.hostPointer(stack+0x120,64),rows,64);std::memcpy(rt.memory.hostPointer(stack+0x160,16),point,16);
            const auto rowsLoad=[&](){for(uint32_t pc:{0x2e0234u,0x2e0238u,0x2e023cu,0x2e0240u}){c.r[3].uw[0]=stack+0x120;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc);}};
            try{
                if(which==0){c.r[3].uw[0]=stack+0x130;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2e0234);}
                else if(which==1){rowsLoad();c.r[3].uw[0]=stack+0x150;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2e0248);}
                else if(which==2){rowsLoad();c.r[3].uw[0]=stack+0x160;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2e0248);
                    c.r[3].uw[0]=stack+0x180;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2e0260);}
                else if(which==3){rowsLoad();c.r[3].uw[0]=stack+0x160;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2e0248);
                    c.r[3].uw[0]=stack+0x170;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2e0260);vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2e0300);}
                else if(which==4){c.r[3].uw[0]=stack+0x120;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2e0234);
                    c.r[3].uw[0]=stack+0x160;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2e0248);
                    c.r[3].uw[0]=stack+0x170;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2e0260);}
                else{rowsLoad();c.r[3].uw[0]=stack+0x160;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2e0248);
                    c.r[3].uw[0]=stack+0x170;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2e0260);}
            }catch(const std::runtime_error &){rejected=true;}
            check(rejected);
        }
    }
    std::printf("%u pair projection cases and 6 rejections passed\n",pairCases);
    // Exponential ramp (mod001 2df060, block 2df0e8): 256 entries of
    // FTOI0(min(vf24*(1-1/e(x)).xyz | vf24.w/e(x.w), 255)) with the 13-term series
    // of 3578b0; independent ps2f oracle, all inputs read from RAM at 2df384.
    unsigned rampEntries=0;
    {auto &rt=native.runtime();
        namespace F=bozo::ps2f;
        uint64_t state=0x5851f42d4c957f2dull;const auto next=[&](){state^=state<<13;state^=state>>7;state^=state<<17;return state;};
        const auto value=[&](float lo,float hi){return lo+float(next()%1000001)/1000000.f*(hi-lo);};
        const auto bits=[](float v){return std::bit_cast<uint32_t>(v);};
        constexpr uint32_t object=0x1500000,table=0x1600000,coefficients=0x3578b0;
        const auto put=[&](uint32_t at,const float (&v)[4]){std::memcpy(rt.memory.hostPointer(at,16),v,16);};
        for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<16;++seed){
            const uint32_t stack=seed&1?0x70002c00:0x1ffd000;R5900Context c{};c.r[29].uw[0]=stack;c.r[17].uw[0]=object;c.r[16].uw[0]=table;
            float k[12];double factorial=1;for(unsigned n=2;n<14;++n){factorial*=n;k[n-2]=float((seed&2?1.:.98)/factorial);}
            const float one[4]={1,1,1,1},scale[4]={255,255,255,128},base[4]={value(0,.5f),value(0,.5f),value(0,.5f),value(-.2f,.2f)};
            const float params[4]={value(.001f,.05f),value(.05f,1),value(0,.02f),1},floorValue[4]={seed&4?value(0,.05f):0.f,99,99,99};
            put(stack+0x50,one);put(stack+0x40,scale);put(object+0x13470,base);put(stack+0x60,floorValue);put(stack+0x30,params);
            for(unsigned r=0;r<3;++r){const float row[4]={k[4*r],k[4*r+1],k[4*r+2],k[4*r+3]};put(coefficients+16*r,row);}
            _MM_SET_ROUNDING_MODE(rounding);const auto csr=_mm_getcsr();
            const std::pair<uint32_t,unsigned> loads[]={{0x2df210,6},{0x2df214,3},{0x2df228,4},{0x2df22c,3},{0x2df238,3},{0x2df23c,3},{0x2df240,3},{0x2df2b0,5}};
            const uint32_t sources[]={stack+0x50,stack+0x40,object+0x13470,stack+0x60,coefficients,coefficients,coefficients,stack+0x30};
            for(unsigned i=0;i<8;++i){c.r[loads[i].second].uw[0]=sources[i];check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,loads[i].first));}
            for(uint32_t pc=0x2df2b4;pc<0x2df384;pc+=4)if(pc!=0x2df2b8 && pc!=0x2df2bc)check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            for(unsigned index=0;index<256;++index){
                c.r[5].uw[0]=index;c.r[4].ud[0]=c.r[4].ud[1]=0x5555555555555555ull;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2df384));
                uint32_t f=0;const auto denominator=F::addNearest(F::mul(F::fromInt(index),bits(params[2]),f),bits(params[1]),f);
                const auto saved=_mm_getcsr();const auto q=F::max(F::addNearest(0,F::div(bits(params[0]),denominator,f),f),bits(floorValue[0]));_mm_setcsr(saved);
                for(unsigned lane=0;lane<4;++lane){
                    const auto x=F::mul(bits(base[lane]),q,f);uint32_t p2=F::mul(x,x,f);const auto p4=F::mul(p2,p2,f);uint32_t p3=F::mul(p2,x,f),p5=F::mul(p4,x,f),p4b=p4;
                    uint32_t acc=F::addNearest(x,0x3f800000,f);
                    for(unsigned r=0;r<3;++r){if(r){p2=F::mul(p2,p4,f);p3=F::mul(p3,p4,f);p4b=F::mul(p4b,p4,f);p5=F::mul(p5,p4,f);}
                        acc=F::addNearest(acc,F::mul(p2,bits(k[4*r]),f),f);acc=F::addNearest(acc,F::mul(p3,bits(k[4*r+1]),f),f);
                        acc=F::addNearest(acc,F::mul(p4b,bits(k[4*r+2]),f),f);acc=F::addNearest(acc,F::mul(p5,bits(k[4*r+3]),f),f);}
                    const auto s2=_mm_getcsr();const auto reciprocal=F::div(0x3f800000,acc,f);_mm_setcsr(s2);
                    const auto v=lane<3?F::subNearest(0x3f800000,F::addNearest(0,reciprocal,f),f):F::mul(0x3f800000,reciprocal,f);
                    const auto expected=F::toInt(F::min(F::mul(bits(scale[lane]),v,f),bits(255.f)));
                    check(c.r[4].uw[lane]==expected);
                }
                ++rampEntries;
            }
            check(_mm_getcsr()==csr);_MM_SET_ROUNDING_MODE(_MM_ROUND_NEAREST);
        }
        // Rejections: a moved input, the index past the loop, an output table over the
        // inputs, a zero step denominator.
        const uint32_t stack=0x1ffd000;
        for(unsigned which=0;which<4;++which){R5900Context c{};c.r[29].uw[0]=stack;c.r[17].uw[0]=object;c.r[16].uw[0]=which==2?stack:table;c.r[5].uw[0]=which==1?256:0;
            const float params[4]={1,0,0,1};if(which==3)put(stack+0x30,params);
            bool rejected=false;
            try{if(which==0){c.r[6].uw[0]=stack+0x40;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2df210);}
                else vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2df384);}catch(const std::runtime_error &){rejected=true;}
            check(rejected);
        }
    }
    std::printf("%u exponential ramp entries and 4 rejections passed\n",rampEntries);
    // Grid point (mod001 2e1a7c): basis rows loaded once before the loop, then per
    // pass ((origin + vf0) + u*p.x) + v*p.y to SP+210; independent ps2f oracle.
    unsigned gridPoints=0;
    {auto &rt=native.runtime();namespace F=bozo::ps2f;
        uint64_t state=0x14057b7ef767814full;const auto next=[&](){state^=state<<13;state^=state>>7;state^=state<<17;return state;};
        const auto value=[&](float range){return float(int64_t(next()%2000001)-1000000)/1000000.f*range;};
        const auto bits=[](float v){return std::bit_cast<uint32_t>(v);};
        constexpr uint32_t s5=0x1400000;
        for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<32;++seed){
            const uint32_t stack=seed&1?0x70002000:0x1ffd000;R5900Context c{};c.r[29].uw[0]=stack;c.r[21].uw[0]=s5;
            float u[4],v[4],origin[4];for(float &x:u)x=value(64.f);for(float &x:v)x=value(64.f);for(float &x:origin)x=value(4096.f);
            if(seed%8==7)u[2]=std::bit_cast<float>(0x00000042u);   // denormal reads as zero
            std::memcpy(rt.memory.hostPointer(stack+0x200,16),u,16);std::memcpy(rt.memory.hostPointer(stack+0x1f0,16),v,16);std::memcpy(rt.memory.hostPointer(s5+0x188f0,16),origin,16);
            if(seed%8==7)u[2]=0;
            _MM_SET_ROUNDING_MODE(rounding);const auto csr=_mm_getcsr();
            c.r[10].uw[0]=stack+0x200;c.r[9].uw[0]=stack+0x1f0;c.r[8].uw[0]=s5+0x188f0;
            for(uint32_t pc:{0x2e215cu,0x2e2160u,0x2e2164u})check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
            for(unsigned pass=0;pass<8;++pass){
                const float point[4]={value(32.f),value(32.f),std::bit_cast<float>(0x7fc00000u),std::bit_cast<float>(0x7f800000u)};   // z/w unused
                std::memcpy(rt.memory.hostPointer(stack+0x1e0,16),point,16);
                c.r[2].uw[0]=stack+0x1e0;for(uint32_t pc:{0x2e2500u,0x2e2504u,0x2e2508u,0x2e250cu})check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
                c.r[2].uw[0]=stack+0x210;check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2e2514));
                uint32_t expected[4],f=0;
                for(unsigned i=0;i<4;++i)expected[i]=F::addNearest(F::addNearest(F::addNearest(bits(origin[i]),i==3?0x3f800000u:0u,f),F::mul(bits(u[i]),bits(point[0]),f),f),F::mul(bits(v[i]),bits(point[1]),f),f);
                check(!std::memcmp(rt.memory.hostPointer(stack+0x210,16),expected,16));++gridPoints;
            }
            check(_mm_getcsr()==csr);_MM_SET_ROUNDING_MODE(_MM_ROUND_NEAREST);
        }
        // Rejections: moved basis, moved point, moved destination, origin over the frame.
        const uint32_t stack=0x1ffd000;
        for(unsigned which=0;which<4;++which){R5900Context c{};c.r[29].uw[0]=stack;c.r[21].uw[0]=which==3?stack+0x1e0-0x188f0:s5;bool rejected=false;
            try{if(which==0){c.r[9].uw[0]=stack+0x200;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2e2160);}
                else if(which==1){c.r[2].uw[0]=stack+0x1f0;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2e2500);}
                else{c.r[2].uw[0]=which==2?stack+0x220:stack+0x210;vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x2e2514);}
            }catch(const std::runtime_error &){rejected=true;}
            check(rejected);
        }
    }
    std::printf("%u grid points and 4 rejections passed\n",gridPoints);
    unsigned contactCases=0,contacts=0;
    for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x33+seed,bozo::kFlatBytes);
        const uint32_t aliases[]={0,0x20000000,0x30000000,0x80000000,0xa0000000};
        const uint32_t stack=seed&1?0x70003000:0x1ffd400|aliases[(seed/2)%5],segment=0x1d0000|aliases[(seed/3)%5],sphere=0x1d4000,outputs=0x1d8000;
        uint32_t state=seed*22695477u+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        for(unsigned i=0;i<4;++i){uint32_t v=next(120,10);std::memcpy(a.memory.hostPointer(segment+0x230+4*i,4),&v,4);v=next(116,10);std::memcpy(a.memory.hostPointer(segment+0x240+4*i,4),&v,4);
            v=next(120,10);if(seed%16==2)std::memcpy(&v,a.memory.hostPointer(segment+0x230+4*i,4),4),v^=1;std::memcpy(a.memory.hostPointer(sphere+0x230+4*i,4),&v,4);}
        for(uint32_t at:{segment+0x220,sphere+0x220}){const uint32_t v=next(124,4)&0x7fffffff;std::memcpy(a.memory.hostPointer(at,4),&v,4);}
        const uint32_t t=seed%16==5?0:next(118,8)&0x7fffffff;std::memcpy(a.memory.hostPointer(stack+0x6c,4),&t,4);
        const uint64_t ret=0x170000;std::memcpy(a.memory.hostPointer(stack+0x50,8),&ret,8);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x0246000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/64;}
        std::memset(x.vu0_vf,0x2d,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xd2,sizeof(x.vu0_acc));x.vu0_status=0x3c3c;x.vu0_q_bits=0x3f800003;x.vi[0]=0x3c3c;
        x.f[0]=seed%7==6?100.f:float(seed%5)/64;   // squared distance from the parameter search
        x.r[19].ud[0]=segment;x.r[20].ud[0]=sphere;x.r[16].ud[0]=outputs;x.r[17].ud[0]=outputs+4;x.r[18].ud[0]=outputs+0x20+(seed%3==2?4:0);
        x.r[29].ud[0]=stack;x.r[31].ud[0]=0x1fc4f0;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(0x1fc4f0)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(0x1fc4f0)(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"contact rounding%x seed%u pc%x/%x fp%08x/%08x\n",rounding,seed,x.pc,y.pc,expectedFp,actualFp);
            for(unsigned i=0;i<sizeof(x);++i)if(reinterpret_cast<const uint8_t *>(&x)[i]!=reinterpret_cast<const uint8_t *>(&y)[i]){std::fprintf(stderr,"CPU byte%x old%02x native%02x\n",i,reinterpret_cast<const uint8_t *>(&x)[i],reinterpret_cast<const uint8_t *>(&y)[i]);break;}
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        contacts+=x.r[2].uw[0]==1;++contactCases;
    }
    {auto &rt=native.runtime();for(unsigned which=0;which<5;++which){constexpr uint32_t segment=0x1d0000,sphere=0x1d4000,outputs=0x1d8000;
        R5900Context c{};c.r[19].uw[0]=segment;c.r[20].uw[0]=sphere;c.r[4].uw[0]=segment+0x230;c.r[3].uw[0]=sphere+0x230;c.r[16].uw[0]=outputs;c.r[17].uw[0]=outputs+4;c.r[18].uw[0]=outputs+0x20;
        c.r[2].uw[0]=0x3f000000;uint32_t pc=0x1fc570;const float a0[4]={0,0,0,1},d0[4]={1,0,0,0},p0[4]={2,0,0,1};
        std::memcpy(rt.memory.hostPointer(segment+0x230,16),a0,16);std::memcpy(rt.memory.hostPointer(segment+0x240,16),d0,16);std::memcpy(rt.memory.hostPointer(sphere+0x230,16),p0,16);
        if(which==0)c.r[3].uw[0]+=16;else if(which==1)c.r[16].uw[0]=segment+0x234;else if(which==2){const float same[4]={0.5f,0,0,1};std::memcpy(rt.memory.hostPointer(sphere+0x230,16),same,16);}
        else if(which==3)c.r[2].uw[0]=0x7f800000;else{pc=0x1fc520;c.r[2].uw[0]=segment+0x220;}
        const auto before=c;const auto fp=_mm_getcsr();const auto ram=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc);}catch(const std::runtime_error &){rejected=true;}
        if(which==2 || which==3){
            // PS2 words: a zero-length difference gives RSQRT +max (normal 0*max = 0), and
            // exponent 255 is an ordinary number; both publish without touching host state.
            uint32_t out[3];std::memcpy(out,rt.memory.hostPointer(outputs+0x20,12),12);
            check(!rejected && _mm_getcsr()==fp && !std::memcmp(&before,&c,sizeof(c)) && (which==3 || (!out[0] && !out[1] && !out[2])));continue;}
        check(rejected && _mm_getcsr()==fp && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));
    }}
    std::printf("%u segment/sphere contact runs (%u contacts), 3 readonly rejections and 2 PS2-word cases passed\n",contactCases,contacts);
    unsigned packedCases=0;
    for(unsigned block=0;block<9;++block)for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x5b+seed,bozo::kFlatBytes);std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003100:0x1ffd600|(seed&2?0x20000000u:0u);
        uint32_t state=seed*1103515245u+block*977+rounding;const auto next=[&](){state=state*1664525u+1013904223u;return int32_t(int16_t(state>>12));};
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x3579000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/16;}
        std::memset(x.vu0_vf,0x4c,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xc4,sizeof(x.vu0_acc));x.vu0_status=0x0c0c;x.vu0_q_bits=0x3f000003;x.vi[0]=0x0c0c;
        int32_t lanes[3]={next(),next(),next()};if(seed%16==1){lanes[0]=32767;lanes[1]=lanes[2]=0;}if(seed%16==2)lanes[0]=lanes[1]=lanes[2]=0;if(seed%16==3){lanes[0]=-32768;lanes[1]=-32768;lanes[2]=12345;}
        for(unsigned i=0;i<3;++i)x.r[10].sw[i]=lanes[i];x.r[10].sw[3]=lanes[2]<0?-1:0;
        for(unsigned i=0;i<3;++i)x.r[13].sw[i]=next();x.r[13].sw[3]=x.r[13].sw[2]<0?-1:0;
        const uint32_t keys=0x1e0000;x.r[16].ud[0]=keys;
        // 2a40e0's pair block reads two key-record pointers through a1 (+24/+28).
        const uint32_t records=0x1e1000;std::memcpy(a.memory.hostPointer(records+0x24,4),&keys,4);std::memcpy(a.memory.hostPointer(records+0x28,4),&keys,4);
        std::memcpy(b.memory.hostPointer(records+0x24,4),&keys,4);std::memcpy(b.memory.hostPointer(records+0x28,4),&keys,4);x.r[5].ud[0]=records;
        x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;if(block)x.r[3].ud[0]=stack;auto y=x;
        const uint32_t entries[9]={0x1f9d74,0x1f9df8,0x2a492c,0x2a415c,0x2a4228,0x2a440c,0x2a45d8,0x2a46d8,0x2a4798},entry=entries[block];
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(entry)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(entry)(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"packed quaternion block%u rounding%x seed%u pc%x/%x fp%08x/%08x\n",block,rounding,seed,x.pc,y.pc,expectedFp,actualFp);
            for(unsigned i=0;i<sizeof(x);++i)if(reinterpret_cast<const uint8_t *>(&x)[i]!=reinterpret_cast<const uint8_t *>(&y)[i]){std::fprintf(stderr,"CPU byte%x old%02x native%02x\n",i,reinterpret_cast<const uint8_t *>(&x)[i],reinterpret_cast<const uint8_t *>(&y)[i]);break;}
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        ++packedCases;
    }
    std::printf("%u packed unit-quaternion keys match the engine VU0 oracle\n",packedCases);
    unsigned leafCases=0,leafHits[2]{};
    for(unsigned kind=0;kind<2;++kind)for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<192;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x2f+seed,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003200:0x1ffd700|(seed&2?0x20000000u:0u),object=0x1e4000,records=0x1e5000,query=0x1e6000;
        uint32_t state=seed*69069u+kind*31+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        put(0x358cd8,object);
        for(unsigned i=0;i<16;++i)put(object+0x150+4*i,i%4==3?(i==15?0x3f800000u:0u):(i/4==i%4?(next(126,2)&0x7fffffff):next(118,6)));
        put(object+0x204,next(126,3)&0x7fffffff);
        for(unsigned i=0;i<12;++i)put(records+4*i,i%4==3?(next(124,4)&0x7fffffff):next(122,6));
        for(unsigned i=0;i<4;++i){put(records+4*i+(kind?0:0),i==3?0x3f800000u:next(122,6));put(records+0x10+4*i,i==3?(next(124,4)&0x7fffffff):(kind?next(122,6):next(124,3)&0x7fffffff));}
        for(unsigned i=0;i<8;++i)put(query+4*i,i==3||i==7?0x3f800000u:(i>=4?(kind?next(118,6):next(124,3)&0x7fffffff):next(122,6)));
        if(seed%16==5)for(unsigned i=0;i<3;++i){uint32_t v;std::memcpy(&v,a.memory.hostPointer(records+0x10+4*i,4),4);put(query+4*i,v);}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x4680000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0x1d,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xd1,sizeof(x.vu0_acc));x.vu0_status=0x0d0d;x.vu0_q_bits=0x3f000007;x.vi[0]=0x0d0d;
        x.r[16].ud[0]=records;x.r[17].ud[0]=query;x.r[4].ud[0]=records+0x10;x.r[29].ud[0]=stack+0x20;
        if(kind==0){x.r[5].ud[0]=records;x.r[6].ud[0]=query;x.r[31].ud[0]=0x1f7238;}else{x.r[5].ud[0]=query;x.r[31].ud[0]=0x1f724c;}
        auto y=x;const uint32_t entry=kind?0x1f6e10:0x1f6be0;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(entry)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(entry)(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=(kind?0x1f724cu:0x1f7238u) || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"leaf kind%u rounding%x seed%u pc%x/%x fp%08x/%08x r2 %llx/%llx\n",kind,rounding,seed,x.pc,y.pc,expectedFp,actualFp,(unsigned long long)x.r[2].ud[0],(unsigned long long)y.r[2].ud[0]);
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        leafHits[kind]+=x.r[2].uw[0]==1;++leafCases;
    }
    std::printf("%u collision leaf tests (%u box hits, %u sphere hits) match the engine VU0 oracle\n",leafCases,leafHits[0],leafHits[1]);
    unsigned segmentCases=0,segmentPoints[3]{};
    for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<256;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x6d+seed,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003400:0x1ffd900|(seed&2?0x20000000u:0u),object=0x1e4000,box=0x1e5000,query=0x1e6000,output=0x1e7000;
        uint32_t state=seed*134775813u+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        put(0x358cd8,object);put(0x358d54,query);
        for(unsigned i=0;i<16;++i)put(object+0x150+4*i,i%4==3?(i==15?0x3f800000u:0u):(i/4==i%4?(next(126,2)&0x7fffffff):next(118,6)));
        for(unsigned i=0;i<4;++i){put(box+4*i,i==3?0x3f800000u:next(122,6));put(box+0x10+4*i,i==3?0u:next(124,4)&0x7fffffff);}
        for(unsigned i=0;i<4;++i){put(query+4*i,i==3?0x3f800000u:next(122,7));put(query+0x10+4*i,i==3?0u:(seed%16==3 && i==1?0u:next(120,8)));}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x5791000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0x2e,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xe2,sizeof(x.vu0_acc));x.vu0_status=0x0e0e;x.vu0_q_bits=0x3f000009;x.vi[0]=0x0e0e;
        x.r[4].ud[0]=box;x.r[5].ud[0]=box+0x10;x.r[6].ud[0]=query;x.r[7].ud[0]=output;x.r[29].ud[0]=stack+0x100;x.r[31].ud[0]=0x3e75b4;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(0x1f6740)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(0x1f6740)(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x3e75b4 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"segment box rounding%x seed%u pc%x/%x fp%08x/%08x r2 %llx/%llx\n",rounding,seed,x.pc,y.pc,expectedFp,actualFp,(unsigned long long)x.r[2].ud[0],(unsigned long long)y.r[2].ud[0]);
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        ++segmentPoints[std::min<uint32_t>(x.r[2].uw[0],2)];++segmentCases;
    }
    std::printf("%u segment/box clips (%u miss, %u one point, %u two points) match the engine VU0 oracle\n",segmentCases,segmentPoints[0],segmentPoints[1],segmentPoints[2]);
    unsigned projectionBlockCases=0;
    for(unsigned block=0;block<5;++block)for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<96;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x19+seed,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1ffd000|(seed&2?0x20000000u:0u),owner=0x1e8000;
        uint32_t state=seed*2891336453u+block*101+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        for(uint32_t at:{stack+0x130u,stack+0xe0u,owner+0x90u})for(unsigned i=0;i<16;++i)put(at+4*i,next(118,12));
        for(uint32_t at:{stack+0x1b0u,stack+0x1a0u,stack+0x190u,stack+0x180u})for(unsigned i=0;i<4;++i)put(at+4*i,i==3?(next(124,6)&0x7fffffff):next(118,12));
        if(seed%16==2)for(unsigned i=0;i<4;++i)put(stack+0x1b0+4*i,0);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x6802000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0x3a,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xa3,sizeof(x.vu0_acc));x.vu0_status=0x0a0a;x.vu0_q_bits=0x3f00000b;x.vi[0]=0x0a0a;
        x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;x.r[21].ud[0]=owner;
        const uint32_t entries[5]={0x246944,0x2469b8,0x2469d8,0x246a4c,0x246e1c};
        if(block==0){x.r[4].ud[0]=stack+0x130;x.r[3].ud[0]=stack+0x1b0;x.r[2].ud[0]=stack+0x190;}
        if(block==1){x.r[3].ud[0]=stack+0x190;x.r[2].ud[0]=stack+0x190;}
        if(block==2){x.r[4].ud[0]=stack+0x130;x.r[3].ud[0]=stack+0x1a0;x.r[2].ud[0]=stack+0x180;}
        if(block==3){x.r[3].ud[0]=stack+0x180;x.r[2].ud[0]=stack+0x180;}
        if(block==4){x.r[3].ud[0]=stack+0xe0;x.r[2].ud[0]=owner+0x90;x.r[4].ud[0]=owner+0x90;}
        auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(entries[block])(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(entries[block])(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"projection block%u rounding%x seed%u pc%x/%x fp%08x/%08x\n",block,rounding,seed,x.pc,y.pc,expectedFp,actualFp);
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        ++projectionBlockCases;
    }
    std::printf("%u mod001 246760 product/divide/composition blocks match the engine VU0 oracle\n",projectionBlockCases);
    unsigned chainCases=0,chainOffsets=0;
    for(unsigned block=0;block<8;++block)for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<64;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x27+seed,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1ffd000|(seed&2?0x20000000u:0u),object=0x1e4000,owner=0x1e5000,part=0x1e6000,other=0x1e7000,frame=0x1e8000,partner=0x1e9000,bone=0x1ea000;
        uint32_t state=seed*747796405u+block*977+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        for(unsigned i=0;i<0x380/4;++i)put(stack+0x110+4*i,next(118,12));
        for(unsigned i=0;i<32;++i)put(object+0x150+4*i,next(118,12));
        for(unsigned i=0;i<16;++i){put(part+4*i,next(118,12));put(bone+0x30+4*i,next(118,12));}
        for(unsigned i=0;i<4;++i){put(other+0x30+4*i,next(118,12));put(partner+0x30+4*i,next(118,12));}
        const bool offset=seed&4;put(part+0x20,0x12345670u|(offset?4u:0u)|(seed&16?2u:0u));put(part+0x38,other);
        put(owner+4,object);put(frame+0x38,partner);chainOffsets+=block==0 && offset;
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x7913000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0x4b,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xb4,sizeof(x.vu0_acc));x.vu0_status=0x0b0b;x.vu0_q_bits=0x3f00000d;x.vi[0]=0x0b0b;
        x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;x.r[17].ud[0]=block==7?bone:part;x.r[18].ud[0]=owner;x.r[16].ud[0]=bone;x.r[30].ud[0]=frame;
        x.f[22]=std::bit_cast<float>(next(118,12));
        const uint32_t entries[8]={0x1fda08,0x1fdacc,0x1fdb54,0x1fdc48,0x1fdeb0,0x1fe790,0x1fe598,0x1fe68c};
        if(block==0)x.r[2].ud[0]=other+0x30;
        if(block==2){x.r[3].ud[0]=stack+0x210;x.r[2].ud[0]=object+0x150;}
        if(block==3){x.r[2].ud[0]=stack+0x210;x.r[3].ud[0]=object+0x150;}
        if(block==4){x.r[2].ud[0]=stack+0x220;std::memcpy(&x.f[2],a.memory.hostPointer(part+0x30,4),4);}
        if(block==5){x.r[4].ud[0]=stack+0x120;x.r[3].ud[0]=bone+0x30;x.r[2].ud[0]=stack+0x110;}
        if(block==6){x.r[5].ud[0]=stack+0x180;x.r[4].ud[0]=stack+0x1c0;x.r[3].ud[0]=partner+0x30;}
        if(block==7){x.r[2].ud[0]=stack+0x160;x.r[3].ud[0]=bone+0x30;}
        auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(entries[block])(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(entries[block])(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"chain block%u rounding%x seed%u pc%x/%x fp%08x/%08x\n",block,rounding,seed,x.pc,y.pc,expectedFp,actualFp);
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        ++chainCases;
    }
    std::printf("%u mod001 1fd110 chain/aim blocks (%u offset placements) match the engine VU0 oracle\n",chainCases,chainOffsets);
    unsigned capsuleCases=0;
    for(unsigned block=0;block<2;++block)for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x35+seed,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1ffd000|(seed&2?0x20000000u:0u),object=0x1e4000,owner=0x1e5000,A=0x1e6000,B=0x1e7000,weights=0x1e8000,out=0x1e9000,scalars=0x1ea000;
        uint32_t state=seed*2654435761u+block*313+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        for(unsigned i=0;i<16;++i)put(object+0x190+4*i,next(118,12));
        for(unsigned i=0;i<4;++i){put(A+0x250+4*i,next(118,12));put(A+0x260+4*i,next(118,12));put(B+0x230+4*i,next(118,12));put(B+0x240+4*i,next(118,12));
            put(weights+4*i,next(120,10)&0x7fffffff);put(stack+0x60+4*i,next(118,12));put(stack+0x70+4*i,next(118,12));put(out+4*i,next(118,12));}
        put(A+0x220,next(120,8)&0x7fffffff);put(B+0x220,next(120,8)&0x7fffffff);put(stack+0x88,next(118,9));put(stack+0x8c,next(118,9));
        put(stack+0x20,owner);put(owner+4,object);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x8a24000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0x5c,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xc5,sizeof(x.vu0_acc));x.vu0_status=0x0c0c;x.vu0_q_bits=0x3f00000f;x.vi[0]=0x0c0c;
        x.r[29].ud[0]=stack;x.r[31].ud[0]=0x1fdc2c;x.r[20].ud[0]=A;x.r[19].ud[0]=B;x.r[18].ud[0]=out;x.r[17].ud[0]=scalars;x.r[16].ud[0]=scalars+0x10;
        x.r[6].ud[0]=weights;x.r[2].ud[0]=A+0x250;x.f[0]=std::bit_cast<float>(next(118,9)&0x7fffffff);
        if(block)x.r[2].ud[0]=B+0x230;   // set by the branch delay slot at 1fc3f4
        auto y=x;const uint32_t entry=block?0x1fc3f8:0x1fc334;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(entry)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(entry)(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x1fdc2c || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"capsule block%u rounding%x seed%u pc%x/%x fp%08x/%08x\n",block,rounding,seed,x.pc,y.pc,expectedFp,actualFp);
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        ++capsuleCases;
    }
    std::printf("%u mod001 1fc300 capsule placement/separation blocks match the engine VU0 oracle\n",capsuleCases);
    unsigned planeCases=0,planeInside=0;
    for(unsigned block=0;block<2;++block)for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x43+seed,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1ffd000|(seed&2?0x20000000u:0u),plane=0x1e4000,point=0x1e5030,out=seed&4?0x1ffd1e0u:0x1e6000u;
        uint32_t state=seed*1103515245u+block*577+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        for(unsigned i=0;i<16;++i)put(plane+0x190+4*i,next(118,12));
        for(unsigned i=0;i<4;++i){put(plane+0x220+4*i,next(118,9));put(plane+0x230+4*i,next(118,12));put(point+4*i,next(118,12));put(stack+0x10+4*i,next(118,12));put(stack+4*i,next(118,12));put(out+4*i,next(118,12));}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x9b35000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0x6d,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xd6,sizeof(x.vu0_acc));x.vu0_status=0x0d0d;x.vu0_q_bits=0x3f000011;x.vi[0]=0x0d0d;
        x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;x.r[4].ud[0]=plane;x.r[5].ud[0]=point;x.r[6].ud[0]=out;x.f[12]=std::bit_cast<float>(next(118,10)&0x7fffffff);
        auto y=x;const uint32_t entry=block?0x1fbdb8:0x1fbd64;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(entry)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(entry)(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"plane block%u rounding%x seed%u pc%x/%x fp%08x/%08x\n",block,rounding,seed,x.pc,y.pc,expectedFp,actualFp);
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        if(!block){float d;std::memcpy(&d,a.memory.hostPointer(stack+0x10,4),4);planeInside+=d<=0;}++planeCases;
    }
    std::printf("%u mod001 1fbd60 plane contact blocks (%u inside) match the engine VU0 oracle\n",planeCases,planeInside);
    unsigned rangeCases=0;
    for(unsigned block=0;block<2;++block)for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x51+seed,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1ffd000|(seed&2?0x20000000u:0u),object=0x1e4000,point=0x1e5030;
        uint32_t state=seed*22695477u+block*911+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        for(unsigned i=0;i<16;++i)put(object+0x220+4*i,next(118,12));
        for(unsigned i=0;i<4;++i){put(point+4*i,next(118,12));for(unsigned k=0;k<4;++k)put(stack+0x10*k+4*i,next(118,12));}
        if(seed%16==7)put(point+12,0);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0xac46000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0x7e,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xe7,sizeof(x.vu0_acc));x.vu0_status=0x0e0e;x.vu0_q_bits=0x3f000013;x.vi[0]=0x0e0e;
        x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;x.r[4].ud[0]=object;x.r[5].ud[0]=point;
        auto y=x;const uint32_t entry=block?0x1fbb80:0x1fbaf8;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(entry)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(entry)(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"range block%u rounding%x seed%u pc%x/%x fp%08x/%08x\n",block,rounding,seed,x.pc,y.pc,expectedFp,actualFp);
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        ++rangeCases;
    }
    std::printf("%u mod001 1fbad0 range transform/length blocks match the engine VU0 oracle\n",rangeCases);
    unsigned buildCases=0;
    for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x5f+seed,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1ffd000|(seed&2?0x20000000u:0u),out=seed&4?0x1e6000u:0x1ffd800u;
        uint32_t state=seed*3266489917u+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        for(unsigned i=0;i<32;++i)put(stack+0x40+4*i,seed%16==9 && i%5==0?0:next(116,14));
        for(unsigned i=0;i<16;++i)put(out+4*i,next(118,12));
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0xbd57000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0x8f,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xf8,sizeof(x.vu0_acc));x.vu0_status=0x0f0f;x.vu0_q_bits=0x3f000015;x.vi[0]=0x0f0f;
        x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;x.r[5].ud[0]=stack+0x40;x.r[3].ud[0]=stack+0x80;x.r[16].ud[0]=out;
        auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(0x217948)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(0x217948)(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"matrix build rounding%x seed%u pc%x/%x fp%08x/%08x\n",rounding,seed,x.pc,y.pc,expectedFp,actualFp);
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        ++buildCases;
    }
    std::printf("%u mod001 2178c0 matrix compositions match the engine VU0 oracle\n",buildCases);
    unsigned cullCases=0;
    for(unsigned block=0;block<2;++block)for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x3d+seed,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1ffd000|(seed&2?0x20000000u:0u),owner=0x1e2000,matrix=seed&4?0x1e6000u:0x1e7040u;
        uint32_t state=seed*2654435761u+rounding+block;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        for(unsigned i=0;i<16;++i){put(owner+0x2f0+4*i,seed%16==9 && i%5==0?0:next(116,14));put(matrix+4*i,next(118,12));put(stack+0xa0+4*i,next(118,12));}
        for(unsigned i=0;i<4;++i){put(stack+0xe0+4*i,i==3?0x3f800000u:next(118,12));put(stack+0x90+4*i,next(118,12));}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0xbd58000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0x8f,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xf8,sizeof(x.vu0_acc));x.vu0_status=0x0f0f;x.vu0_q_bits=0x3f000015;x.vi[0]=0x0f0f;
        x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;x.r[20].ud[0]=owner;
        if(!block){x.r[3].ud[0]=owner+0x2f0;x.r[2].ud[0]=matrix;x.r[4].ud[0]=stack+0xa0;}
        else{x.r[4].ud[0]=stack+0xe0;x.r[3].ud[0]=stack+0xa0;x.r[2].ud[0]=stack+0x90;}
        auto y=x;const uint32_t entry=block?0x2204a4:0x2203e0;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(entry)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(entry)(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"cull block%u rounding%x seed%u pc%x/%x fp%08x/%08x\n",block,rounding,seed,x.pc,y.pc,expectedFp,actualFp);
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        ++cullCases;
    }
    std::printf("%u mod001 2202d0 cull compositions and centre transforms match the engine VU0 oracle\n",cullCases);
    {
        // mod001 212950: all 38 stores against the listing evaluated symbolically
        // in double (bound: the same expression on magnitudes), lane masks exact.
        struct Case{uint32_t in[40];double expected[38*4],bound[38*4];unsigned mask[38];};
        static const Case cases[]={
#include "bozo_box_frustum_oracle.inc"
        };
        unsigned lanes=0;double worst=0;
        for(const auto &k:cases){
            std::array<std::array<uint32_t,4>,5> box,frustum;
            for(unsigned r=0;r<5;++r)for(unsigned i=0;i<4;++i){box[r][i]=k.in[4*r+i];frustum[r][i]=k.in[20+4*r+i];}
            const auto stores=vp2s::boxFrustumStores(box,frustum);
            for(unsigned s=0;s<38;++s){
                if(stores.defined[s]!=k.mask[s]){std::fprintf(stderr,"box/frustum store %u mask %x expected %x\n",s,stores.defined[s],k.mask[s]);check(false);}
                for(unsigned i=0;i<4;++i)if(k.mask[s]>>i&1){
                    const double got=std::bit_cast<float>(stores.words[s][i]),want=k.expected[4*s+i],err=std::abs(got-want),tol=4e-6*k.bound[4*s+i]+1e-30;
                    worst=std::max(worst,err/(k.bound[4*s+i]+1e-30));
                    if(!(err<=tol)){std::fprintf(stderr,"box/frustum store %u lane %u got %.9g expected %.9g bound %.9g\n",s,i,got,want,k.bound[4*s+i]);check(false);}
                    ++lanes;}
            }
        }
        std::printf("%u mod001 212950 box/frustum store lanes match the listing (worst %.2g of magnitude)\n",lanes,worst);
    }
    unsigned listCases=0;
    for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x6b+seed,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1ffd000|(seed&2?0x20000000u:0u),node=0x1e4000,object=0x1e5000;
        uint32_t state=seed*2246822519u+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        put(node+0x230,object);for(unsigned i=0;i<16;++i)put(object+0x190+4*i,next(118,12));
        for(unsigned i=0;i<4;++i){put(stack+0x30+4*i,i==3?0x3f800000u:next(118,12));put(stack+0x20+4*i,next(118,12));}
        // Stale sphere cache seen on the route (first frame of an object): finite, huge Y.
        if(seed%16==11){put(stack+0x30,0xbfa12848);put(stack+0x34,0xe3b96faf);put(stack+0x38,0xb5bac313);}
        if(seed%16==13)put(stack+0x34,next(200,40));
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0xce68000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0x91,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0x19,sizeof(x.vu0_acc));x.vu0_status=0x0101;x.vu0_q_bits=0x3f000017;x.vi[0]=0x0101;
        x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;x.r[16].ud[0]=node;x.r[9].ud[0]=stack+0x30;x.r[10].ud[0]=object+0x190;x.r[8].ud[0]=stack+0x20;
        auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(0x29f674)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(0x29f674)(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"list box rounding%x seed%u pc%x/%x fp%08x/%08x\n",rounding,seed,x.pc,y.pc,expectedFp,actualFp);
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        ++listCases;
    }
    std::printf("%u mod001 29f5e0 list box node transforms match the engine VU0 oracle\n",listCases);
    unsigned hitFrameCases=0;
    for(unsigned block=0;block<2;++block)for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x79+seed,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1ffd000|(seed&2?0x20000000u:0u),node=0x1e4000,owner=0x1e5000,self=0x1e6000;
        uint32_t state=seed*1597334677u+block*1013+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        for(unsigned i=0;i<16;++i){put(node+0x270+4*i,next(116,14));put(self+0x90+4*i,next(116,14));put(stack+0xe0+4*i,next(116,14));}
        for(unsigned i=0;i<4;++i){put(stack+0xd0+4*i,next(118,12));put(owner+0x2b0+4*i,next(118,12));}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0xdf79000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0xa2,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0x2a,sizeof(x.vu0_acc));x.vu0_status=0x0202;x.vu0_q_bits=0x3f000019;x.vi[0]=0x0202;
        x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;x.r[16].ud[0]=node;x.r[18].ud[0]=owner;x.r[19].ud[0]=self;
        if(block==0){x.r[4].ud[0]=node+0x270;x.r[3].ud[0]=self+0x90;x.r[2].ud[0]=stack+0xe0;}
        else{x.r[7].ud[0]=stack+0xd0;x.r[8].ud[0]=stack+0xe0;x.r[6].ud[0]=owner+0x2b0;}
        auto y=x;const uint32_t entry=block?0x23e300:0x23e1bc;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(entry)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(entry)(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"hit frame block%u rounding%x seed%u pc%x/%x fp%08x/%08x\n",block,rounding,seed,x.pc,y.pc,expectedFp,actualFp);
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        ++hitFrameCases;
    }
    std::printf("%u mod001 23e0d0 hit frame composition/product blocks match the engine VU0 oracle\n",hitFrameCases);
    unsigned distanceCases=0;
    for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x8d+seed,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003010:0x1ffd010|(seed&2?0x20000000u:0u),point=0x1e47f0,normal=0x1e5240,origin=seed&4?0x1e6000u:point;
        uint32_t state=seed*2654435769u+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        for(unsigned i=0;i<4;++i){put(point+4*i,next(118,12));put(normal+4*i,next(118,9));put(0x1e6000+4*i,next(118,12));put(stack-0x10+4*i,next(118,12));}
        if(seed%16==11){put(point,0xbfa12848);put(point+4,0xe3b96faf);put(point+8,0xb5bac313);put(point+12,0x9c5ce0e7);}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0xe08a000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0xb3,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0x3b,sizeof(x.vu0_acc));x.vu0_status=0x0303;x.vu0_q_bits=0x3f00001b;x.vi[0]=0x0303;
        x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;x.r[4].ud[0]=point;x.r[5].ud[0]=normal;x.r[6].ud[0]=origin;
        auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(0x214590)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(0x214590)(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"plane distance rounding%x seed%u pc%x/%x fp%08x/%08x\n",rounding,seed,x.pc,y.pc,expectedFp,actualFp);
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        ++distanceCases;
    }
    std::printf("%u mod001 214590 plane distances match the engine VU0 oracle\n",distanceCases);
    unsigned sweepCases=0;
    for(unsigned block=0;block<3;++block)for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<96;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0x9b+seed,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1ffd000|(seed&2?0x20000000u:0u),self=0x1e4000,node=0x1e5000,owner=0x1e6000;
        uint32_t state=seed*1640531527u+block*1231+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        for(unsigned i=0;i<4;++i){put(self+0x7f0+4*i,next(118,12));put(self+0x7e0+4*i,next(118,10));put(node+0x240+4*i,next(118,12));put(stack+0xc0+4*i,next(118,12));put(stack+0xd0+4*i,next(118,12));}
        for(unsigned i=0;i<16;++i)put(owner+0x240+4*i,next(116,14));
        if(seed%16==11){put(self+0x7f0,0xbfa12848);put(self+0x7f4,0xe3b96faf);put(self+0x7f8,0xb5bac313);put(self+0x7fc,0x9c5ce0e7);}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0xf19b000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0xc4,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0x4c,sizeof(x.vu0_acc));x.vu0_status=0x0404;x.vu0_q_bits=0x3f00001d;x.vi[0]=0x0404;
        x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;x.r[22].ud[0]=self+0x7f0;x.r[20].ud[0]=self+0x7e0;x.r[18].ud[0]=node;x.r[21].ud[0]=owner;
        if(block==0)x.r[30].ud[0]=node+0x240;
        if(block==1)x.r[3].ud[0]=node+0x240;
        if(block==2){x.r[4].ud[0]=owner+0x240;x.r[3].ud[0]=owner+0x240;}
        auto y=x;const uint32_t entries[3]={0x20dc20,0x20dd34,0x20e49c};
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(entries[block])(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(entries[block])(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"sphere sweep block%u rounding%x seed%u pc%x/%x fp%08x/%08x\n",block,rounding,seed,x.pc,y.pc,expectedFp,actualFp);
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        ++sweepCases;
    }
    std::printf("%u mod001 20d9d0 sphere sweep blocks match the engine VU0 oracle\n",sweepCases);
    unsigned worldCases=0,worldPaths[3]{};
    for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<192;++seed){
        auto &a=old.runtime();auto &b=native.runtime();std::memset(a.memory.flat(),0xa7+seed,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1ffd000|(seed&2?0x20000000u:0u),object=0x1e4000,parent=0x1e5000,out=0x1e6000;
        uint32_t state=seed*3735928559u+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        for(unsigned i=0;i<4;++i){put(object+0x20+4*i,next(118,12));put(object+0x30+4*i,next(122,5));put(object+0x40+4*i,next(124,4)&0x7fffffff);put(out+4*i,next(118,12));}
        for(unsigned i=0;i<16;++i){put(object+0x110+4*i,next(116,14));put(parent+0xd0+4*i,next(116,14));put(stack+4*i,next(118,12));}
        const unsigned path=seed%3;put(object+0x208,path?parent:0);
        {uint16_t fl=path==2?0x0040:0x8001;std::memcpy(a.memory.hostPointer(object+0x20c,2),&fl,2);}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0xa1b2000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0xd5,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0x5d,sizeof(x.vu0_acc));x.vu0_status=0x0505;x.vu0_q_bits=0x3f00001f;x.vi[0]=0x0505;
        x.r[29].ud[0]=stack+0x40;x.r[31].ud[0]=0x170000;x.r[4].ud[0]=object;x.r[5].ud[0]=out;
        auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(0x248e40)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(0x248e40)(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"world position path%u rounding%x seed%u pc%x/%x fp%08x/%08x\n",path,rounding,seed,x.pc,y.pc,expectedFp,actualFp);
            for(unsigned i=0;i<sizeof(x);++i)if(reinterpret_cast<const uint8_t *>(&x)[i]!=reinterpret_cast<const uint8_t *>(&y)[i]){std::fprintf(stderr,"CPU byte%x old%02x native%02x\n",i,reinterpret_cast<const uint8_t *>(&x)[i],reinterpret_cast<const uint8_t *>(&y)[i]);break;}
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        ++worldPaths[path];++worldCases;
    }
    std::printf("%u mod001 248e40 world positions (%u unparented, %u normalized, %u unnormalized) match the engine VU0 oracle\n",worldCases,worldPaths[0],worldPaths[1],worldPaths[2]);
    // main 10da70 camera target: the native site stepped through its COP2 words
    // (EE code between the blocks adjusts SP+88), against a ps2f oracle; no
    // other RAM, CPU state or MXCSR changes.
    unsigned targetCases=0;
    for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &r=native.runtime();std::memset(r.memory.flat(),0xb9+seed,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1ffd000|(seed&2?0x20000000u:0u),owner=0x1e4000,target=0x1e5000,camera=0x1e6000;
        uint32_t state=seed*2166136261u+rounding;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(r.memory.hostPointer(at,4),&v,4);};
        const auto get=[&](uint32_t at){uint32_t v;std::memcpy(&v,r.memory.hostPointer(at,4),4);return v;};
        put(camera+0x1390,owner);for(unsigned i=0;i<16;++i)put(owner+0x190+4*i,next(116,14));for(unsigned i=0;i<4;++i)put(target+4*i,next(118,12));
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0xc3d4000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        x.r[29].ud[0]=stack;x.r[17].ud[0]=camera;x.r[16].ud[0]=target;x.r[3].ud[0]=owner+0x190;x.r[2].ud[0]=stack+0x80;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
        const auto add=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
        uint32_t expect[4],m[4][4],p[4];std::memcpy(m,r.memory.hostPointer(owner+0x190,64),64);std::memcpy(p,r.memory.hostPointer(target,16),16);
        for(unsigned i=0;i<4;++i)expect[i]=add(add(add(mul(m[0][i],p[0]),mul(m[1][i],p[1])),mul(m[2][i],p[2])),mul(m[3][i],0x3f800000));
        const std::vector<uint8_t> before(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);const auto cpu=x;
        for(uint32_t pc=0x10dc44;pc<=0x10dc68;pc+=4)check(vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,pc));
        bool same=!std::memcmp(&cpu,&x,sizeof(x));for(unsigned i=0;i<4;++i)same=same && get(stack+0x80+4*i)==expect[i];
        const auto z=next(118,12);put(stack+0x88,z);expect[2]=z;x.r[3].ud[0]=stack+0x80;x.r[2].ud[0]=stack+0x50;
        for(uint32_t pc=0x10dc84;pc<=0x10dc98;pc+=4)check(vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,pc));
        uint32_t sq[3];for(unsigned i=0;i<3;++i)sq[i]=mul(expect[i],expect[i]);
        const uint32_t length[4]={add(add(sq[0],sq[1]),mul(0x3f800000,sq[2])),sq[1],sq[2],expect[3]};
        for(unsigned i=0;i<4;++i)same=same && get(stack+0x50+4*i)==length[i];
        const auto fp=_mm_getcsr();_mm_setcsr(saved);same=same && fp==control;
        for(size_t i=0;i<bozo::kFlatBytes && same;++i){const auto at=r.memory.flat()+i;const bool written=at>=r.memory.hostPointer(stack+0x50,16) && at<r.memory.hostPointer(stack+0x50,16)+16;
            const bool row=at>=r.memory.hostPointer(stack+0x80,16) && at<r.memory.hostPointer(stack+0x80,16)+16;if(!written && !row)same=r.memory.flat()[i]==before[i];}
        if(!same){std::fprintf(stderr,"camera target rounding%x seed%u\n",rounding,seed);check(false);}
        ++targetCases;
    }
    std::printf("%u main 10da70 camera-target block pairs match the engine VU0 oracle\n",targetCases);
    // mod001 220d20 visibility: VI2 after program 0xb60 transcribed from its
    // listing (VU0 data qwords 8..19 = camera+460 points, camera+500 normals),
    // sticky sign from the flag writes landed by the FSAND; load sites change nothing.
    unsigned visibilityCases=0,visible=0;
    for(unsigned seed=0;seed<512;++seed){
        auto &r=native.runtime();std::memset(r.memory.flat(),0x3c+seed,bozo::kFlatBytes);
        const uint32_t sphere=seed&1?0x1e47f0:0x35db40,s1cam=0x1e5000,s2cam=0x1e6000;
        uint32_t state=seed*2246822507u;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(r.memory.hostPointer(at,4),&v,4);};
        for(uint32_t cam:{s1cam,s2cam})for(unsigned i=0;i<12;++i)for(unsigned k=0;k<4;++k)put(cam+(i<6?0x460+16*i:0x500+16*(i-6))+4*k,i<6?next(118,8):next(120,7));
        for(unsigned k=0;k<4;++k)put(sphere+4*k,k==3?(next(118,10)&0x7fffffff):next(116,10));
        if(seed%32==7){put(sphere,0xbfa12848);put(sphere+4,0xe3b96faf);put(sphere+8,0xb5bac313);put(sphere+12,0x9c5ce0e7);}
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0xe5f6000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        x.r[2].ud[0]=sphere;x.r[17].ud[0]=s1cam;x.r[18].ud[0]=s2cam;x.r[16].ud[0]=seed&2?1:0x100;   // s0 is andi'd to a byte
        const auto cam=(seed&2)?s1cam:s2cam;
        const auto M=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
        const auto A=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
        const auto S=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::subNearest(a,b,f);};
        using V=std::array<uint32_t,4>;const auto load=[&](uint32_t at){V v;std::memcpy(v.data(),r.memory.hostPointer(at,16),16);return v;};
        V vf[32]{},mem[20];vf[5]=load(sphere);vf[31]={0x3f800000,0x3f800000,0x3f800000,0x3f800000};
        for(unsigned i=0;i<6;++i){mem[8+i]=load(cam+0x460+16*i);mem[14+i]=load(cam+0x500+16*i);}
        const unsigned rows[6]={17,18,19,28,29,30};
        for(unsigned i=0;i<6;++i)for(unsigned k=0;k<3;++k)vf[rows[i]][k]=M(S(vf[5][k],mem[8+i][k]),mem[14+i][k]);
        V d17{},d18{};
        d17[0]=A(A(vf[17][0],vf[17][1]),M(vf[31][0],vf[17][2]));d17[1]=A(A(vf[18][1],vf[18][0]),M(vf[31][1],vf[18][2]));
        d17[2]=A(A(vf[19][2],vf[19][1]),M(vf[31][2],vf[19][0]));d18[0]=A(A(vf[28][0],vf[28][1]),M(vf[31][0],vf[28][2]));
        d18[1]=A(A(vf[29][1],vf[29][0]),M(vf[31][1],vf[29][2]));d18[2]=A(A(vf[30][2],vf[30][1]),M(vf[31][2],vf[30][0]));
        uint32_t sticky=0;     // FSSET clears; the ADDW.xyz writes at c58/c60 land before FSAND
        for(unsigned k=0;k<3;++k){if(A(d17[k],vf[5][3])>>31)sticky|=2;if(A(d18[k],vf[5][3])>>31)sticky|=2;}
        const uint32_t vi2=(sticky<<6)&0x80;
        const std::vector<uint8_t> before(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);auto y=x;const auto fp=_mm_getcsr();
        for(uint32_t pc:{0x220d70u,0x220dd8u,0x220df0u,0x220df4u,0x220f04u,0x220f1cu,0x220f40u,0x220f64u,0x220fa0u,0x220fb8u,0x220fc4u})check(vp2s::nativeMathInstruction(r.memory.flat(),&y,&r,pc));
        check(!std::memcmp(&x,&y,sizeof(x)));
        check(vp2s::nativeMathInstruction(r.memory.flat(),&y,&r,0x220fd0));
        if(y.r[3].ud[0]!=vi2){std::fprintf(stderr,"visibility seed%u vi2 %llx/%x\n",seed,(unsigned long long)y.r[3].ud[0],vi2);check(false);}
        y.r[3]=x.r[3];check(!std::memcmp(&x,&y,sizeof(x)) && _mm_getcsr()==fp && !std::memcmp(before.data(),r.memory.flat(),before.size()));
        visible+=vi2==0;++visibilityCases;
    }
    std::printf("%u mod001 220d20 visibility tests (%u inside) match the 0xb60 listing\n",visibilityCases,visible);
    // mod010 48fa10 marker projection: camera rows from their loads, the stored
    // half of program 0xb60 (vf6 = (p.x/p.z, p.y/p.z, p.z, v.w), p = R*(v,1)) to
    // SP+170; then the s0+100 length to SP+90 (the length idiom). The code comes from the
    // private mod010 image (argument); without them, or with one byte changed,
    // the sites are not claimed (another overlay at the same addresses).
    unsigned markerCases=0;
    {
        if(argc<2)throw std::runtime_error("usage: vp2s-bozo-math-check <mod010.elf>");
        std::vector<uint8_t> elf;{FILE *f=std::fopen(argv[1],"rb");if(!f)throw std::runtime_error("mod010 image missing");
            uint8_t buf[4096];size_t n;while((n=std::fread(buf,1,sizeof buf,f))>0)elf.insert(elf.end(),buf,buf+n);std::fclose(f);}
        const auto u32=[&](size_t at){uint32_t v;std::memcpy(&v,elf.data()+at,4);return v;};
        const auto u16=[&](size_t at){uint16_t v;std::memcpy(&v,elf.data()+at,2);return v;};
        const auto block=[&](Runtime &r,uint32_t lo,uint32_t hi){
            for(unsigned i=0;i<u16(0x2c);++i){const size_t ph=u32(0x1c)+i*u16(0x2a);
                if(u32(ph)==1 && u32(ph+8)<=lo && hi<=u32(ph+8)+u32(ph+16)){std::memcpy(r.memory.hostPointer(lo,hi-lo),elf.data()+u32(ph+4)+lo-u32(ph+8),hi-lo);return;}}
            throw std::runtime_error("mod010 image lacks the marker code");};
        const auto M=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
        const auto A=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
        using V=std::array<uint32_t,4>;
        for(unsigned seed=0;seed<256;++seed){
            auto &r=native.runtime();std::memset(r.memory.flat(),0x21+seed,bozo::kFlatBytes);
            block(r,0x491f40,0x4923a0);block(r,0x492f80,0x492fb0);
            const uint32_t camera=seed&1?0x1e5000:0x1c4c40,stack=0x1fff000,points=0x1e7000;
            uint32_t state=seed*2654435761u+7;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
            const auto put=[&](uint32_t at,uint32_t v){std::memcpy(r.memory.hostPointer(at,4),&v,4);};
            const auto get=[&](uint32_t at){uint32_t v;std::memcpy(&v,r.memory.hostPointer(at,4),4);return v;};
            for(unsigned i=0;i<16;++i){put(camera+0x2f0+4*i,next(120,10));put(camera+0x3f0+4*i,next(120,10));}
            const unsigned count=1+seed%4;for(unsigned i=0;i<count;++i)for(unsigned k=0;k<4;++k)put(points+0x40*i+4*k,next(118,12));
            if(seed%16==5)put(points+8,0);   // p.z from the rows alone
            R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0xa1b2000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
            x.r[16].ud[0]=camera;x.r[2].ud[0]=camera+0x3f0;x.r[3].ud[0]=camera+0x2f0;x.r[29].ud[0]=stack;
            V R[4];for(unsigned k=0;k<4;++k)std::memcpy(R[k].data(),r.memory.hostPointer(camera+0x3f0+16*k,16),16);
            std::vector<uint8_t> before(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);auto y=x;
            for(uint32_t pc=0x491f44;pc<=0x491f50;pc+=4)check(vp2s::nativeMathInstruction(r.memory.flat(),&y,&r,pc));
            for(uint32_t pc:{0x491f90u,0x491f94u,0x491f98u,0x491f9cu,0x491fa8u,0x491facu,0x491fb0u,0x491fb4u,0x491fb8u})check(vp2s::nativeMathInstruction(r.memory.flat(),&y,&r,pc));
            check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(before.data(),r.memory.flat(),before.size()));
            // A changed camera record after the loads does not change the rows (they are registers).
            for(unsigned i=0;i<16;++i)put(camera+0x3f0+4*i,next(126,1));
            for(unsigned i=0;i<count;++i){
                y.r[16].ud[0]=i;y.r[17].ud[0]=points+0x40*i;y.r[3].ud[0]=stack+0x170;
                V v;std::memcpy(v.data(),r.memory.hostPointer(points+0x40*i,16),16);V p;
                for(unsigned l=0;l<4;++l)p[l]=A(A(A(M(R[0][l],v[0]),M(R[1][l],v[1])),M(R[2][l],v[2])),M(R[3][l],0x3f800000u));
                uint32_t f=0;const auto q=bozo::ps2f::div(0x3f800000u,p[2],f);const V expected{M(p[0],q),M(p[1],q),p[2],v[3]};
                for(uint32_t pc:{0x492380u,0x49238cu,0x492390u,0x492398u})check(vp2s::nativeMathInstruction(r.memory.flat(),&y,&r,pc));
                check(y.r[9].ud[0]==0 && y.r[9].ud[1]==x.r[9].ud[1]);
                for(unsigned l=0;l<4;++l)if(get(stack+0x170+4*l)!=expected[l]){std::fprintf(stderr,"marker seed%u item%u lane%u %08x/%08x\n",seed,i,l,get(stack+0x170+4*l),expected[l]);check(false);}
                ++markerCases;
            }
            // The length after the loop: (sqrt((x2+y2)+1*z2), y2, z2, raw w) of s0+100 at SP+90.
            const uint32_t object=0x1d0000+0x40*seed;for(unsigned k=0;k<4;++k)put(object+0x100+4*k,next(115,15));
            y.r[16].ud[0]=object;y.r[17].ud[0]=object+0x100;y.r[3].ud[0]=stack+0x90;
            V s;std::memcpy(s.data(),r.memory.hostPointer(object+0x100,16),16);V sq{M(s[0],s[0]),M(s[1],s[1]),M(s[2],s[2]),s[3]};
            uint32_t f=0;const V length{A(0,bozo::ps2f::sqrt(A(A(sq[0],sq[1]),M(0x3f800000u,sq[2])),f)),sq[1],sq[2],s[3]};
            for(uint32_t pc=0x492f88;pc<=0x492fa8;pc+=4)check(vp2s::nativeMathInstruction(r.memory.flat(),&y,&r,pc));
            for(unsigned l=0;l<4;++l)check(get(stack+0x90+4*l)==length[l]);
            // Another overlay's code at these addresses: nothing is claimed.
            const uint32_t changed=0x491f40+4*(seed%280);put(changed,get(changed)^0x100);y=x;
            check(!vp2s::nativeMathInstruction(r.memory.flat(),&y,&r,0x491f44));
        }
        // A runtime that never loaded the rows (and has no mod010 code) claims no projection site.
        auto &o=old.runtime();std::memset(o.memory.flat(),0,bozo::kFlatBytes);R5900Context z{};
        for(uint32_t pc:{0x491f48u,0x491f90u,0x492380u,0x49238cu,0x492390u,0x492398u})check(!vp2s::nativeMathInstruction(o.memory.flat(),&z,&o,pc));
    }
    std::printf("%u mod010 48fa10 marker projections match the 0xb60 listing (rows kept from their loads)\n",markerCases);
    // mod001 2156d0: program 0x670 from its listing, in place; CFC2 reads VI0.
    unsigned quaternionLeafCases=0;
    for(unsigned seed=0;seed<512;++seed){
        auto &r=native.runtime();std::memset(r.memory.flat(),0x5d+1+seed,bozo::kFlatBytes);const uint32_t at=seed&1?0x70003040:0x1e4030;
        uint32_t state=seed*1103515245u;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        uint32_t q[4];for(auto &v:q)v=next(110,24);if(seed%32==5)for(auto &v:q)v=0;if(seed%32==6)q[1]=0x7fffffffu;
        std::memcpy(r.memory.hostPointer(at,16),q,16);
        const auto M=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
        const auto A=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
        uint32_t v6[4];for(unsigned i=0;i<4;++i)v6[i]=M(q[i],q[i]);const uint32_t one=0x3f800000;
        const auto sum=A(A(A(v6[0],v6[1]),M(one,v6[2])),M(one,v6[3]));
        uint32_t f=0;const auto saved=_mm_getcsr();const auto Q=bozo::ps2f::rsqrt(one,sum,f);_mm_setcsr(saved);
        uint32_t expect[4];for(unsigned i=0;i<4;++i)expect[i]=M(q[i],Q);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0xf7a8000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}x.r[4].ud[0]=at;
        auto y=x;const auto fp=_mm_getcsr();
        for(uint32_t pc:{0x2156d0u,0x2156d8u,0x2156dcu,0x2156e4u})check(vp2s::nativeMathInstruction(r.memory.flat(),&y,&r,pc));
        uint32_t got[4];std::memcpy(got,r.memory.hostPointer(at,16),16);
        x.r[9].ud[0]=0;check(!std::memcmp(&x,&y,sizeof(x)) && _mm_getcsr()==fp && !std::memcmp(got,expect,16));
        ++quaternionLeafCases;
    }
    std::printf("%u mod001 2156d0 quaternion normalizations match the 0x670 listing\n",quaternionLeafCases);
    // mod001 237320 bone palette: programs 0x898 + 0x1f8.. run instruction by
    // instruction on a VF register file (the B to 0x1f8 skips the 0x1f0 pair).
    unsigned paletteCases=0;
    for(unsigned seed=0;seed<256;++seed){
        auto &r=native.runtime();std::memset(r.memory.flat(),0x6f+seed,bozo::kFlatBytes);
        const uint32_t stack=0x1ffd000,list=0x1e4000,skeleton=0x1e5000,counts=0x1e5800,binds=0x1e6000,bones=0x1e8000;const unsigned n=1+seed%5;
        uint32_t state=seed*69069u+7;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(r.memory.hostPointer(at,4),&v,4);};
        put(skeleton+0x34,counts);put(counts+4,n);put(skeleton+0x40,binds);
        for(unsigned j=0;j<n;++j){put(list+4*j,bones+0x100*j);for(unsigned i=0;i<16;++i){put(bones+0x100*j+0x90+4*i,seed%16==3 && i==5?0x80000000u:next(116,14));put(binds+64*j+4*i,next(116,14));}}
        for(unsigned i=0;i<16;++i)put(stack+0x60+4*i,next(116,14));
        using V=std::array<uint32_t,4>;const auto load=[&](uint32_t at){V v;std::memcpy(v.data(),r.memory.hostPointer(at,16),16);return v;};
        const auto M=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
        const auto A=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
        for(unsigned j=0;j<n;++j){
            V vf[32]{};vf[0]={0,0,0,0x3f800000};for(unsigned k=0;k<4;++k){vf[9+k]=load(bones+0x100*j+0x90+16*k);vf[13+k]=load(binds+64*j+16*k);vf[28+k]=load(stack+0x60+16*k);}
            vf[1]=vf[2]=vf[3]=V{0x11111111,0x22222222,0x33333333,0x44444444};   // garbage before the program
            V acc{};const auto chain=[&](unsigned d,const V &a,const V &b,const V &c2,const V &e,const V &s){
                for(unsigned i=0;i<4;++i){acc[i]=M(a[i],s[0]);acc[i]=A(acc[i],M(b[i],s[1]));acc[i]=A(acc[i],M(c2[i],s[2]));vf[d][i]=A(acc[i],M(e[i],s[3]));}};
            chain(1,vf[13],vf[14],vf[15],vf[16],vf[9]);chain(2,vf[13],vf[14],vf[15],vf[16],vf[10]);chain(3,vf[13],vf[14],vf[15],vf[16],vf[11]);vf[4]=vf[12];
            chain(5,vf[1],vf[2],vf[3],vf[4],vf[28]);chain(6,vf[1],vf[2],vf[3],vf[4],vf[29]);vf[8]=vf[4];chain(7,vf[1],vf[2],vf[3],vf[4],vf[30]);
            // 0x1f0 .. 0x248 (the B at 0x948 targets pc+8+imm*8 = 0x1f0)
            vf[1][1]=A(vf[0][1],vf[6][0]);vf[3][2]=vf[7][2];
            vf[1][2]=A(vf[0][2],vf[7][0]);vf[4][3]=vf[8][3];vf[1][3]=M(vf[0][3],vf[8][0]);vf[2][0]=A(vf[0][0],vf[5][1]);
            vf[2][2]=A(vf[0][2],vf[7][1]);vf[1][0]=vf[5][0];vf[2][3]=M(vf[0][3],vf[8][1]);vf[3][0]=A(vf[0][0],vf[5][2]);
            vf[3][1]=A(vf[0][1],vf[6][2]);vf[2][1]=vf[6][1];vf[3][3]=M(vf[0][3],vf[8][2]);vf[4][0]=A(vf[0][0],vf[5][3]);
            vf[4][1]=A(vf[0][1],vf[6][3]);vf[4][2]=A(vf[0][2],vf[7][3]);
            R5900Context x{};for(unsigned i=1;i<32;++i)x.r[i].ud[0]=0x1357000000000000ull+i;
            x.r[17].ud[0]=0x70000010+64*j;x.r[18].ud[0]=list;x.r[20].ud[0]=skeleton;x.r[29].ud[0]=stack;auto y=x;
            const uint32_t pc=j+1<n || seed&1?0x2374ec:0x237584;
            check(vp2s::nativeMathInstruction(r.memory.flat(),&y,&r,pc));check(!std::memcmp(&x,&y,sizeof(x)));
            for(unsigned k=0;k<4;++k){const auto got=load(0x70000010+64*j+16*k);
                if(got!=vf[1+k]){std::fprintf(stderr,"palette seed%u bone%u row%u %08x %08x %08x %08x / %08x %08x %08x %08x\n",seed,j,k,got[0],got[1],got[2],got[3],vf[1+k][0],vf[1+k][1],vf[1+k][2],vf[1+k][3]);check(false);}}
            ++paletteCases;}
    }
    std::printf("%u mod001 237320 bone palette entries match the 0x898/0x1f8 listing\n",paletteCases);
    // The inlined length idiom, recognised by its nine words anywhere in code.
    unsigned idiomCases=0;
    for(unsigned seed=0;seed<512;++seed){
        auto &r=native.runtime();std::memset(r.memory.flat(),0x33+seed,bozo::kFlatBytes);
        uint32_t state=seed*2246822519u+3;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
        const unsigned rs=1+rnd()%25,rd=1+rnd()%25;const int16_t so=int16_t(16*(rnd()%64)),dof=int16_t(16*(rnd()%64))-0x200;
        const uint32_t code=0x1c0000+16*(rnd()%256),srcBase=0x1d0000,dstBase=0x1e8000;
        const uint32_t words[9]={0xd8000000u|rs<<21|24u<<16|uint16_t(so),0x4bd8c62a,0x4bc00643,0x4b18c03d,0x4b18ce0a,0x4a1803bd,0x4a0003bf,0x4b000620,0xf8000000u|rd<<21|24u<<16|uint16_t(dof)};
        std::memcpy(r.memory.hostPointer(code,36),words,36);
        uint32_t v[4];for(auto &x:v)x=(rnd()&0x807fffffu)|((110+rnd()%30)<<23);if(seed%16==3)v[0]=v[1]=v[2]=0;
        std::memcpy(r.memory.hostPointer(srcBase+so,16),v,16);
        R5900Context x{};for(unsigned i=1;i<32;++i)x.r[i].ud[0]=0x1357900000000000ull+i;x.r[rs].ud[0]=srcBase;x.r[rd].ud[0]=dstBase;
        if(rs==rd)continue;
        const auto before=x;const auto fp=_mm_getcsr();
        for(unsigned k=0;k<9;++k)check(vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+4*k));
        check(_mm_getcsr()==fp && !std::memcmp(&before,&x,sizeof(x)));
        const auto M=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
        const auto A=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
        uint32_t expect[4]={M(v[0],v[0]),M(v[1],v[1]),M(v[2],v[2]),v[3]},flags=0;
        const auto saved=_mm_getcsr();expect[0]=A(0,bozo::ps2f::sqrt(A(A(expect[0],expect[1]),M(0x3f800000,expect[2])),flags));_mm_setcsr(saved);
        uint32_t got[4];std::memcpy(got,r.memory.hostPointer(dstBase+dof,16),16);
        if(std::memcmp(got,expect,16)){std::fprintf(stderr,"length idiom seed%u %08x %08x %08x %08x / %08x %08x %08x %08x\n",seed,got[0],got[1],got[2],got[3],expect[0],expect[1],expect[2],expect[3]);check(false);}
        // One changed word makes it unrecognised (the caller then rejects the site).
        const uint32_t broken=0x4b18ce0b;std::memcpy(r.memory.hostPointer(code+16,4),&broken,4);
        check(!vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+32));
        ++idiomCases;
    }
    std::printf("%u inlined vector-length blocks match the VU0 oracle\n",idiomCases);
    // Negate / offset-dot / squared-difference idioms; the store's base register
    // is a load's base register rewritten in between (as in mod001 20e5b8).
    unsigned macroCases=0;
    for(unsigned seed=0;seed<384;++seed){
        auto &r=native.runtime();std::memset(r.memory.flat(),0x21+seed,bozo::kFlatBytes);
        uint32_t state=seed*747796405u+11;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
        const unsigned kind=seed%6,loads=kind==0 || kind==3 || kind==4?1:kind==1?3:2;const uint32_t code=0x1c4000+64*(rnd()%64);
        const auto lq=[](unsigned base,unsigned vf,int16_t off){return 0xd8000000u|base<<21|vf<<16|uint16_t(off);};
        const auto sq=[](unsigned base,unsigned vf,int16_t off){return 0xf8000000u|base<<21|vf<<16|uint16_t(off);};
        const unsigned regs[3]={4,5,3};const uint32_t bases[3]={0x1d0000,0x1d4000,0x1d8000},outBase=0x1e8000;
        std::vector<uint32_t> words;uint32_t in[3][4];
        const unsigned vfs[3]={kind==0 || kind==5?1u:kind==4?24u:20u,kind==5?2u:21u,22u};
        for(unsigned l=0;l<loads;++l){for(auto &w:in[l])w=(rnd()&0x807fffffu)|((112+rnd()%30)<<23);
            const int16_t off=int16_t(16*(rnd()%32));std::memcpy(r.memory.hostPointer(bases[l]+off,16),in[l],16);words.push_back(lq(regs[l],vfs[l],off));
            if(rnd()%2)words.push_back(0);}   // a plain EE word between COP2 words
        // v1 (the last load's base) becomes the store's base, as in the game.
        words.push_back(0x24030000u|uint16_t(outBase-0x1e0000));   // addiu v1,zero,... (placeholder value)
        words.back()=0x3c030000u|uint16_t(outBase>>16);             // lui v1,outBase>>16
        if(kind==0){words.push_back(0x4bc1006c);words.push_back(sq(3,1,0));}
        else if(kind==1){for(uint32_t w:{0x4bf4adecu,0x4b000603u,0x4bf7b5eau,0x4b17b83du,0x4b17c5cau})words.push_back(w);words.push_back(sq(3,23,0));}
        else if(kind==2){words.push_back(0x4bf4adac);words.push_back(0x4bf6b5aa);words.push_back(sq(3,22,0));}
        else if(kind==5){for(uint32_t w:{0x4bc208eau,0x4b031841u,0x4b030842u})words.push_back(w);words.push_back(sq(3,1,0));}
        else if(kind==4){for(uint32_t w:{0x4bd8c62au,0x4bc00643u,0x4b18c03du,0x4b18ce0au})words.push_back(w);words.push_back(sq(3,24,0));}
        else{for(uint32_t w:{0x4bd4a5aau,0x4b000543u,0x4b16b03du,0x4b16ad8au,0x4a7603beu,0u,0u,0u,0x4a0003bfu,0x4bc0a51cu})words.push_back(w);words.push_back(sq(3,20,0));}
        std::memcpy(r.memory.hostPointer(code,4*words.size()),words.data(),4*words.size());
        R5900Context x{};for(unsigned i=1;i<32;++i)x.r[i].ud[0]=0x2468a00000000000ull+i;for(unsigned l=0;l<3;++l)x.r[regs[l]].ud[0]=bases[l];
        for(unsigned i=0;i<words.size();++i){const uint32_t pc=code+4*i,w=words[i];
            if(w==(0x3c030000u|uint16_t(outBase>>16))){x.r[3].ud[0]=outBase;continue;}   // the EE word itself
            const bool isCop2=(w>>26)==0x12 || (w>>26)==0x36 || (w>>26)==0x3e;if(!isCop2)continue;
            if(!vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,pc)){std::fprintf(stderr,"macro idiom kind%u seed%u word %u (%08x) unclaimed\n",kind,seed,i,w);check(false);}}
        const auto M=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
        const auto A=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
        const auto S=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::subNearest(a,b,f);};
        uint32_t expect[4];
        if(kind==0){for(unsigned i=0;i<3;++i)expect[i]=S(0,in[0][i]);expect[3]=in[0][3];}
        else if(kind==1){for(unsigned i=0;i<4;++i)expect[i]=M(in[2][i],S(in[1][i],in[0][i]));expect[0]=A(A(expect[0],expect[1]),M(0x3f800000,expect[2]));}
        else if(kind==2)for(unsigned i=0;i<4;++i){const auto d=S(in[1][i],in[0][i]);expect[i]=M(d,d);}
        else if(kind==5){std::memcpy(expect,in[0],16);expect[0]=A(A(M(in[0][0],in[1][0]),M(in[0][1],in[1][1])),M(in[0][2],in[1][2]));}
        else if(kind==4){for(unsigned i=0;i<3;++i)expect[i]=M(in[0][i],in[0][i]);expect[0]=A(A(expect[0],expect[1]),M(0x3f800000,expect[2]));expect[3]=in[0][3];}
        else{uint32_t f=0,sqv[3];for(unsigned i=0;i<3;++i)sqv[i]=M(in[0][i],in[0][i]);const auto saved=_mm_getcsr();
            const auto q=bozo::ps2f::rsqrt(0x3f800000,A(A(sqv[0],sqv[1]),M(0x3f800000,sqv[2])),f);_mm_setcsr(saved);for(unsigned i=0;i<3;++i)expect[i]=M(in[0][i],q);expect[3]=in[0][3];}
        uint32_t got[4];std::memcpy(got,r.memory.hostPointer(outBase,16),16);
        if(std::memcmp(got,expect,16)){std::fprintf(stderr,"macro idiom kind%u seed%u %08x %08x %08x %08x / %08x %08x %08x %08x\n",kind,seed,got[0],got[1],got[2],got[3],expect[0],expect[1],expect[2],expect[3]);check(false);}
        // A store reached without its loads rejects.
        R5900Context y=x;bool rejected=false;try{vp2s::nativeMathInstruction(r.memory.flat(),&y,&r,code+4*unsigned(words.size()-1));}catch(const std::runtime_error &){rejected=true;}check(rejected);
        ++macroCases;
    }
    std::printf("%u inlined negate/dot/square/normalize/squared-length/dot3 blocks match the VU0 oracle\n",macroCases);
    // Component product (mod003 366d10): the game's words, its store in the
    // delay slot of jr ra. Any other control word there leaves it unclaimed.
    unsigned productCases=0;
    for(unsigned seed=0;seed<128;++seed){
        auto &r=native.runtime();std::memset(r.memory.flat(),0x33+seed,bozo::kFlatBytes);
        uint32_t state=seed*2246822519u+3;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
        const uint32_t code=0x1c4000+64*(rnd()%64),object=0x1d0000+16*(rnd()%256);
        const uint32_t words[8]={0x24860560,0x24850570,0x24830580,0xd8cc0000,0xd8ad0000,0x4bed632a,0x03e00008,0xf86c0000};
        std::memcpy(r.memory.hostPointer(code,32),words,32);
        uint32_t a[4],b[4];for(unsigned i=0;i<4;++i){a[i]=(rnd()&0x807fffffu)|((100+rnd()%50)<<23);b[i]=(rnd()&0x807fffffu)|((100+rnd()%50)<<23);}
        if(seed%16==3)b[2]=0;   // a zero lane
        std::memcpy(r.memory.hostPointer(object+0x560,16),a,16);std::memcpy(r.memory.hostPointer(object+0x570,16),b,16);
        R5900Context x{};x.r[4].ud[0]=object;x.r[6].ud[0]=object+0x560;x.r[5].ud[0]=object+0x570;x.r[3].ud[0]=object+0x580;
        for(const unsigned i:{3u,4u,5u,7u})check(vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+4*i));
        uint32_t got[4],expect[4];std::memcpy(got,r.memory.hostPointer(object+0x580,16),16);
        for(unsigned i=0;i<4;++i){uint32_t f=0;expect[i]=bozo::ps2f::mul(a[i],b[i],f);}
        check(!std::memcmp(got,expect,16));
        const uint32_t branch=0x14400003;std::memcpy(r.memory.hostPointer(code+24,4),&branch,4);
        check(!vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+28));
        ++productCases;
    }
    std::printf("%u component-product blocks (store in a return delay slot) match the VU0 oracle\n",productCases);
    // Scaled sum (mod003 433c60..433c8c), the game's words: the scale comes from a
    // GPR through QMTC2 (2.0 there; other values and garbage upper lanes here).
    unsigned sumCases=0;
    for(unsigned seed=0;seed<128;++seed){
        auto &r=native.runtime();std::memset(r.memory.flat(),0x44+seed,bozo::kFlatBytes);
        uint32_t state=seed*3266489917u+5;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
        const uint32_t code=0x1c4000+64*(rnd()%64),object=0x1d0000+16*(rnd()%256),stackAt=0x1e0000+16*(rnd()%256);
        const uint32_t words[12]={0x268501f0,0x44820000,0x27a40150,0x27a30150,0xd8a10000,0xd8820000,0x44020000,0,0x48a21800,0x4bc011bf,0x4bc30848,0xf8610000};
        std::memcpy(r.memory.hostPointer(code,48),words,48);
        uint32_t a[4],b[4];for(unsigned i=0;i<4;++i){a[i]=(rnd()&0x807fffffu)|((100+rnd()%50)<<23);b[i]=(rnd()&0x807fffffu)|((100+rnd()%50)<<23);}
        const uint32_t scale=seed%4?0x40000000u:(rnd()&0x807fffffu)|((110+rnd()%30)<<23);
        std::memcpy(r.memory.hostPointer(object+0x1f0,16),a,16);std::memcpy(r.memory.hostPointer(stackAt+0x150,16),b,16);
        R5900Context x{};x.r[20].ud[0]=object;x.r[29].ud[0]=stackAt;x.r[5].ud[0]=object+0x1f0;x.r[4].ud[0]=stackAt+0x150;x.r[3].ud[0]=stackAt+0x150;
        x.r[2].uw[0]=scale;x.r[2].uw[1]=rnd();x.r[2].uw[2]=rnd();x.r[2].uw[3]=rnd();
        for(const unsigned i:{4u,5u,8u,9u,10u,11u})if(!vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+4*i)){std::fprintf(stderr,"scaled sum word %u unclaimed\n",i);check(false);}
        uint32_t got[4],expect[4];std::memcpy(got,r.memory.hostPointer(stackAt+0x150,16),16);
        for(unsigned i=0;i<3;++i){uint32_t f=0;expect[i]=bozo::ps2f::addNearest(bozo::ps2f::mul(b[i],0x3f800000,f),bozo::ps2f::mul(a[i],scale,f),f);}expect[3]=a[3];
        if(std::memcmp(got,expect,16)){std::fprintf(stderr,"scaled sum seed%u %08x %08x %08x %08x / %08x %08x %08x %08x\n",seed,got[0],got[1],got[2],got[3],expect[0],expect[1],expect[2],expect[3]);check(false);}
        ++sumCases;
    }
    // The same with the scale from v1 (mod003 38cf0c..38cf28, the game's words):
    // LQC2 vf1 (v1), vf2 (a3); MFC1 v1 = f0; QMTC2 v1; MULAw; MADDx; SQC2 vf1 -> a3.
    for(unsigned seed=0;seed<128;++seed){
        auto &r=native.runtime();std::memset(r.memory.flat(),0x47+seed,bozo::kFlatBytes);
        uint32_t state=seed*2246822519u+3;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
        const uint32_t code=0x1c4000+64*(rnd()%64),aAt=0x1d0000+16*(rnd()%256),bAt=0x1e0000+16*(rnd()%256);
        const uint32_t words[8]={0xd8610000,0xd8e20000,0x44030000,0,0x48a31800,0x4bc011bf,0x4bc30848,0xf8e10000};
        std::memcpy(r.memory.hostPointer(code,32),words,32);
        uint32_t a[4],b[4];for(unsigned i=0;i<4;++i){a[i]=(rnd()&0x807fffffu)|((100+rnd()%50)<<23);b[i]=(rnd()&0x807fffffu)|((100+rnd()%50)<<23);}
        const uint32_t scale=(rnd()&0x807fffffu)|((110+rnd()%30)<<23);
        std::memcpy(r.memory.hostPointer(aAt,16),a,16);std::memcpy(r.memory.hostPointer(bAt,16),b,16);
        R5900Context x{};x.r[3].ud[0]=aAt;x.r[7].ud[0]=bAt;
        for(const unsigned i:{0u,1u})if(!vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+4*i)){std::fprintf(stderr,"scaled sum v1 word %u unclaimed\n",i);check(false);}
        x.r[3].uw[0]=scale;x.r[3].uw[1]=rnd();x.r[3].uw[2]=rnd();x.r[3].uw[3]=rnd();   // MFC1 v1 = f0
        for(const unsigned i:{4u,5u,6u,7u})if(!vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+4*i)){std::fprintf(stderr,"scaled sum v1 word %u unclaimed\n",i);check(false);}
        uint32_t got[4],expect[4];std::memcpy(got,r.memory.hostPointer(bAt,16),16);
        for(unsigned i=0;i<3;++i){uint32_t f=0;expect[i]=bozo::ps2f::addNearest(bozo::ps2f::mul(b[i],0x3f800000,f),bozo::ps2f::mul(a[i],scale,f),f);}expect[3]=a[3];
        if(std::memcmp(got,expect,16)){std::fprintf(stderr,"scaled sum v1 seed%u %08x %08x %08x %08x / %08x %08x %08x %08x\n",seed,got[0],got[1],got[2],got[3],expect[0],expect[1],expect[2],expect[3]);check(false);}
        ++sumCases;
    }
    // Point advanced along a direction (mod028 47c108..47c128, the game's words): LQC2 vf1 (v1 =
    // sp+c0), vf2 (v0 = sp+d0); MFC1 t0 = f0; QMTC2 t0; addiu v0 = sp+c0; MULAw vf1; MADDx; SQC2 vf1 -> v0.
    for(unsigned seed=0;seed<128;++seed){
        auto &r=native.runtime();std::memset(r.memory.flat(),0x49+seed,bozo::kFlatBytes);
        uint32_t state=seed*2654435761u+11;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
        const uint32_t code=0x1c4000+64*(rnd()%64),stackAt=0x1e0000+16*(rnd()%256);
        const uint32_t words[9]={0xd8610000,0xd8420000,0x44080000,0,0x48a81800,0x27a200c0,0x4bc009bf,0x4bc31048,0xf8410000};
        std::memcpy(r.memory.hostPointer(code,36),words,36);
        uint32_t p[4],d[4];for(unsigned i=0;i<4;++i){p[i]=(rnd()&0x807fffffu)|((100+rnd()%50)<<23);d[i]=(rnd()&0x807fffffu)|((100+rnd()%50)<<23);}
        const uint32_t scale=(rnd()&0x807fffffu)|((110+rnd()%30)<<23);
        std::memcpy(r.memory.hostPointer(stackAt+0xc0,16),p,16);std::memcpy(r.memory.hostPointer(stackAt+0xd0,16),d,16);
        R5900Context x{};x.r[29].ud[0]=stackAt;x.r[3].ud[0]=stackAt+0xc0;x.r[2].ud[0]=stackAt+0xd0;
        for(const unsigned i:{0u,1u})if(!vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+4*i)){std::fprintf(stderr,"advance word %u unclaimed\n",i);check(false);}
        x.r[8].uw[0]=scale;x.r[8].uw[1]=rnd();x.r[8].uw[2]=rnd();x.r[8].uw[3]=rnd();   // MFC1 t0 = f0
        check(vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+16));
        x.r[2].ud[0]=stackAt+0xc0;   // addiu v0, sp, 0xc0
        for(const unsigned i:{6u,7u,8u})if(!vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+4*i)){std::fprintf(stderr,"advance word %u unclaimed\n",i);check(false);}
        uint32_t got[4],expect[4];std::memcpy(got,r.memory.hostPointer(stackAt+0xc0,16),16);
        for(unsigned i=0;i<3;++i){uint32_t f=0;expect[i]=bozo::ps2f::addNearest(bozo::ps2f::mul(p[i],0x3f800000,f),bozo::ps2f::mul(d[i],scale,f),f);}expect[3]=p[3];
        if(std::memcmp(got,expect,16)){std::fprintf(stderr,"advance seed%u %08x %08x %08x %08x / %08x %08x %08x %08x\n",seed,got[0],got[1],got[2],got[3],expect[0],expect[1],expect[2],expect[3]);check(false);}
        ++sumCases;
    }
    // Colour scale (mod028 47c280..47c29c, the game's words): LQC2 vf1 (v1); MFC1 t0 = f0; QMTC2 t0
    // -> vf2; addiu v0 = sp+a0; MULx.xyz vf1 *= vf2.x; MULx.w vf1 *= vf0.x; SQC2 vf1 -> v0.
    for(unsigned seed=0;seed<128;++seed){
        auto &r=native.runtime();std::memset(r.memory.flat(),0x4b+seed,bozo::kFlatBytes);
        uint32_t state=seed*2246822519u+13;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
        const uint32_t code=0x1c4000+64*(rnd()%64),colourAt=0x1d0000+16*(rnd()%256),stackAt=0x1e0000+16*(rnd()%256);
        const uint32_t words[8]={0xd8610000,0x44080000,0,0x48a81000,0x27a200a0,0x4bc20858,0x4a200858,0xf8410000};
        std::memcpy(r.memory.hostPointer(code,32),words,32);
        uint32_t v[4];for(unsigned i=0;i<4;++i)v[i]=(rnd()&0x807fffffu)|((100+rnd()%50)<<23);
        const uint32_t scale=seed%2?0x3c000000u:(rnd()&0x807fffffu)|((110+rnd()%30)<<23);
        std::memcpy(r.memory.hostPointer(colourAt,16),v,16);
        R5900Context x{};x.r[29].ud[0]=stackAt;x.r[3].ud[0]=colourAt;
        check(vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code));
        x.r[8].uw[0]=scale;x.r[8].uw[1]=rnd();x.r[8].uw[2]=rnd();x.r[8].uw[3]=rnd();   // MFC1 t0 = f0
        check(vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+12));
        x.r[2].ud[0]=stackAt+0xa0;   // addiu v0, sp, 0xa0
        for(const unsigned i:{5u,6u,7u})if(!vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+4*i)){std::fprintf(stderr,"colour scale word %u unclaimed\n",i);check(false);}
        uint32_t got[4],expect[4];std::memcpy(got,r.memory.hostPointer(stackAt+0xa0,16),16);
        for(unsigned i=0;i<3;++i){uint32_t f=0;expect[i]=bozo::ps2f::mul(v[i],scale,f);}{uint32_t f=0;expect[3]=bozo::ps2f::mul(v[3],0,f);}
        if(std::memcmp(got,expect,16)){std::fprintf(stderr,"colour scale seed%u %08x %08x %08x %08x / %08x %08x %08x %08x\n",seed,got[0],got[1],got[2],got[3],expect[0],expect[1],expect[2],expect[3]);check(false);}
        ++sumCases;
    }
    std::printf("%u scaled-sum blocks (QMTC2 scale from v0, v1 or t0; colour scale) match the VU0 oracle\n",sumCases);
    // Particle draw setup (mod001 22f3a4..22f3bc and 22f404..22f410), the game's words:
    // a*b + c with its store in the delay slot of b 22f418, and the plain product.
    unsigned setupCases=0;
    for(unsigned seed=0;seed<128;++seed){
        auto &r=native.runtime();std::memset(r.memory.flat(),0x66+seed,bozo::kFlatBytes);
        uint32_t state=seed*1103515245u+7;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
        const bool sum=seed&1;const uint32_t code=0x1c4000+64*(rnd()%64),s2=0x1d0000+16*(rnd()%64),sp=0x1e0000+16*(rnd()%64);
        const std::vector<uint32_t> words=sum?std::vector<uint32_t>{0x264503c0,0x27a400b0,0x27a30090,0x26420310,0xd8a10000,0xd8820000,0xd8430000,0x4be20abe,0x4be0184b,0x10000017,0xf8610000}
                                             :std::vector<uint32_t>{0x26440310,0x27a300b0,0x27a20090,0xd8810000,0xd8620000,0x4be2086a,0xf8410000};
        std::memcpy(r.memory.hostPointer(code,4*words.size()),words.data(),4*words.size());
        uint32_t a[4],b[4],k[4];for(unsigned i=0;i<4;++i){a[i]=(rnd()&0x807fffffu)|((100+rnd()%50)<<23);b[i]=(rnd()&0x807fffffu)|((100+rnd()%50)<<23);k[i]=(rnd()&0x807fffffu)|((100+rnd()%50)<<23);}
        R5900Context x{};
        if(sum){std::memcpy(r.memory.hostPointer(s2+0x3c0,16),a,16);std::memcpy(r.memory.hostPointer(sp+0xb0,16),b,16);std::memcpy(r.memory.hostPointer(s2+0x310,16),k,16);
            x.r[5].ud[0]=s2+0x3c0;x.r[4].ud[0]=sp+0xb0;x.r[3].ud[0]=sp+0x90;x.r[2].ud[0]=s2+0x310;
            for(const unsigned i:{4u,5u,6u,7u,8u,10u})if(!vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+4*i)){std::fprintf(stderr,"muladd word %u unclaimed\n",i);check(false);}}
        else{std::memcpy(r.memory.hostPointer(s2+0x310,16),a,16);std::memcpy(r.memory.hostPointer(sp+0xb0,16),b,16);
            x.r[4].ud[0]=s2+0x310;x.r[3].ud[0]=sp+0xb0;x.r[2].ud[0]=sp+0x90;
            for(const unsigned i:{3u,4u,5u,6u})if(!vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+4*i)){std::fprintf(stderr,"product2 word %u unclaimed\n",i);check(false);}}
        uint32_t got[4],expect[4];std::memcpy(got,r.memory.hostPointer(sp+0x90,16),16);
        for(unsigned i=0;i<4;++i){uint32_t f=0;expect[i]=bozo::ps2f::mul(a[i],b[i],f);if(sum)expect[i]=bozo::ps2f::addNearest(expect[i],bozo::ps2f::mul(k[i],0x3f800000,f),f);}
        check(!std::memcmp(got,expect,16));
        if(sum){const uint32_t conditional=0x14400017;std::memcpy(r.memory.hostPointer(code+36,4),&conditional,4);check(!vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+40));}
        ++setupCases;
    }
    std::printf("%u particle draw-setup blocks (a*b+c after b, a*b) match the VU0 oracle\n",setupCases);
    // Particle step (mod003 3652d0 and its copies 38d900, 44b8f0): the game's site words at their addresses; four
    // loop variants, dt from v1 or s1, two stores per particle, out-of-order rejects.
    unsigned particleCases=0;
    {struct Variant{uint32_t q,p,v,a,m,s1,s2;};
     const Variant variants[4]={{0x365350,0x3653c4,0x3653c8,0x3653e8,0x3653ec,0x3653f8,0x3653fc},{0x365350,0x3654ac,0x3654b0,0x3654d0,0x3654d4,0x3654e0,0x3654e4},
                                {0x36554c,0x3655e8,0x3655f0,0x3656f4,0x3656f8,0x365704,0x36570c},{0x36554c,0x3657e4,0x3657e8,0x3658fc,0x365900,0x36590c,0x365914}};
     const std::pair<uint32_t,uint32_t> words[]={{0x365350,0x48a31800},{0x36554c,0x48b11800},
        {0x3653c4,0xda010000},{0x3653c8,0xd8620000},{0x3653e8,0x4bc00abc},{0x3653ec,0x4bc31048},{0x3653f8,0xf8610000},{0x3653fc,0xfa010000},
        {0x3654ac,0xd8610000},{0x3654b0,0xd9220000},{0x3654d0,0x4bc00abc},{0x3654d4,0x4bc31048},{0x3654e0,0xf9210000},{0x3654e4,0xf8610000},
        {0x3655e8,0xda610000},{0x3655f0,0xdaa20000},{0x3656f4,0x4bc00abc},{0x3656f8,0x4bc31048},{0x365704,0xfa410000},{0x36570c,0xfa610000},
        {0x3657e4,0xda610000},{0x3657e8,0xda020000},{0x3658fc,0x4bc00abc},{0x365900,0x4bc31048},{0x36590c,0xfa010000},{0x365914,0xfa610000}};
     // 409700 (a third class): its own layout, sites listed exactly.
     const Variant variants409700[4]={{0x409780,0x4097fc,0x409800,0x409828,0x40982c,0x409838,0x40983c},{0x409780,0x4098ec,0x4098f0,0x40991c,0x409920,0x40992c,0x409930},
                                      {0x4099a8,0x409a58,0x409a60,0x409bbc,0x409bc0,0x409bcc,0x409bd4},{0x4099a8,0x409ca8,0x409cac,0x409e1c,0x409e20,0x409e2c,0x409e34}};
     const std::pair<uint32_t,uint32_t> words409700[]={{0x409780,0x48a31800},{0x4099a8,0x48b01800},
        {0x4097fc,0xd9410000},{0x409800,0xd8620000},{0x409828,0x4bc00abc},{0x40982c,0x4bc31048},{0x409838,0xf8610000},{0x40983c,0xf9410000},
        {0x4098ec,0xd8610000},{0x4098f0,0xd9220000},{0x40991c,0x4bc00abc},{0x409920,0x4bc31048},{0x40992c,0xf9210000},{0x409930,0xf8610000},
        {0x409a58,0xdac10000},{0x409a60,0xda020000},{0x409bbc,0x4bc00abc},{0x409bc0,0x4bc31048},{0x409bcc,0xfa010000},{0x409bd4,0xfac10000},
        {0x409ca8,0xdaa10000},{0x409cac,0xda020000},{0x409e1c,0x4bc00abc},{0x409e20,0x4bc31048},{0x409e2c,0xfa010000},{0x409e34,0xfaa10000}};
     // 409fb0 (a fourth class, map 79): the same words at its own sites.
     const Variant variants409fb0[4]={{0x40a030,0x40a0a4,0x40a0a8,0x40a0c8,0x40a0cc,0x40a0d8,0x40a0dc},{0x40a030,0x40a184,0x40a188,0x40a1ac,0x40a1b0,0x40a1bc,0x40a1c0},
                                      {0x40a238,0x40a2e0,0x40a2e8,0x40a43c,0x40a440,0x40a44c,0x40a454},{0x40a238,0x40a520,0x40a524,0x40a68c,0x40a690,0x40a69c,0x40a6a4}};
     const std::pair<uint32_t,uint32_t> words409fb0[]={{0x40a030,0x48a31800},{0x40a238,0x48b01800},
        {0x40a0a4,0xd9410000},{0x40a0a8,0xd8620000},{0x40a0c8,0x4bc00abc},{0x40a0cc,0x4bc31048},{0x40a0d8,0xf8610000},{0x40a0dc,0xf9410000},
        {0x40a184,0xd8610000},{0x40a188,0xd9220000},{0x40a1ac,0x4bc00abc},{0x40a1b0,0x4bc31048},{0x40a1bc,0xf9210000},{0x40a1c0,0xf8610000},
        {0x40a2e0,0xdac10000},{0x40a2e8,0xda020000},{0x40a43c,0x4bc00abc},{0x40a440,0x4bc31048},{0x40a44c,0xfa010000},{0x40a454,0xfac10000},
        {0x40a520,0xdaa10000},{0x40a524,0xda020000},{0x40a68c,0x4bc00abc},{0x40a690,0x4bc31048},{0x40a69c,0xfa010000},{0x40a6a4,0xfaa10000}};
     const auto field=[](uint32_t w,unsigned shift){return (w>>shift)&31;};
     for(unsigned seed=0;seed<256;++seed){
        const bool fourth=seed>=192,third=seed>=128 && !fourth;
        // The 38d900 copy (seeds 64..95) and the forest emitter's 44b8f0 copy (96..127): every site shifted.
        const uint32_t k=seed>=64 && seed<128?(seed>=96?0x44b8f0-0x3652d0:0x38d900-0x3652d0):0;
        auto &r=native.runtime();std::memset(r.memory.flat(),0x5c+seed,bozo::kFlatBytes);
        if(third){for(const auto &[pc,w]:words409700)std::memcpy(r.memory.hostPointer(pc,4),&w,4);}
        else if(fourth){for(const auto &[pc,w]:words409fb0)std::memcpy(r.memory.hostPointer(pc,4),&w,4);}
        else for(const auto &[pc,w]:words)std::memcpy(r.memory.hostPointer(pc+k,4),&w,4);
        uint32_t state=seed*2654435761u+9;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
        const auto value=[&](){return (rnd()&0x807fffffu)|((105+rnd()%40)<<23);};
        const auto var=[&]{auto v=fourth?variants409fb0[seed%4]:third?variants409700[seed%4]:variants[seed%4];for(auto *f:{&v.q,&v.p,&v.v,&v.a,&v.m,&v.s1,&v.s2})*f+=k;return v;}();const uint32_t dt=seed%8==1?0:value();
        R5900Context x{};const auto qw=[&](uint32_t pc){uint32_t w;std::memcpy(&w,r.memory.hostPointer(pc,4),4);return w;};
        x.r[field(qw(var.q),16)].uw[0]=dt;x.r[field(qw(var.q),16)].uw[3]=rnd();check(vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,var.q));
        for(unsigned n=0;n<3;++n){
            const uint32_t pAt=0x1d0000+0x100*n+16*(rnd()%8),vAt=0x1d8000+0x100*n+16*(rnd()%8),o1=0x1e0000+0x40*n,o2=0x1e8000+0x40*n;
            uint32_t p[4],v[4];for(auto &w:p)w=value();for(auto &w:v)w=value();if(seed%8==3)p[1]=0x80000000;
            std::memcpy(r.memory.hostPointer(pAt,16),p,16);std::memcpy(r.memory.hostPointer(vAt,16),v,16);
            x.r[field(qw(var.p),21)].uw[0]=pAt;check(vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,var.p));
            x.r[field(qw(var.v),21)].uw[0]=vAt;check(vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,var.v));
            check(vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,var.a) && vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,var.m));
            x.r[field(qw(var.s1),21)].uw[0]=o1;check(vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,var.s1));
            x.r[field(qw(var.s2),21)].uw[0]=o2;check(vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,var.s2));
            uint32_t expect[4],got1[4],got2[4];
            for(unsigned i=0;i<3;++i){uint32_t f=0;expect[i]=bozo::ps2f::addNearest(bozo::ps2f::addNearest(p[i],0,f),bozo::ps2f::mul(v[i],dt,f),f);}expect[3]=p[3];
            std::memcpy(got1,r.memory.hostPointer(o1,16),16);std::memcpy(got2,r.memory.hostPointer(o2,16),16);
            if(std::memcmp(got1,expect,16) || std::memcmp(got2,expect,16)){std::fprintf(stderr,"particle seed%u n%u %08x %08x %08x %08x / %08x %08x %08x %08x\n",seed,n,got1[0],got1[1],got1[2],got1[3],expect[0],expect[1],expect[2],expect[3]);check(false);}
            ++particleCases;
        }
        // A step or velocity without its position rejects; so does a store before the step.
        bool rejected=false;try{vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,var.q);vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,var.v);}catch(const std::runtime_error &){rejected=true;}check(rejected);
        rejected=false;try{vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,var.p);vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,var.s1);}catch(const std::runtime_error &){rejected=true;}check(rejected);
        // Different code at the same address (another overlay) is not claimed here.
        const uint32_t other=0;std::memcpy(r.memory.hostPointer(var.m,4),&other,4);check(!vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,var.m));
     }}
    std::printf("%u particle steps (four classes, four loops, two stores) match the VU0 oracle\n",particleCases);
    // Row point and homogeneous divide (mod001 2b802c / 2b80a0), the game's words.
    unsigned rowCases=0;
    for(unsigned seed=0;seed<256;++seed){
        auto &r=native.runtime();std::memset(r.memory.flat(),0x5a+seed,bozo::kFlatBytes);
        uint32_t state=seed*2891336453u+7;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
        const bool homogeneous=seed&1;const uint32_t code=0x1c4000+128*(rnd()%32),point=0x1d0000+16*(rnd()%64),matrix=0x1d4000+16*(rnd()%64),out=0x1e8000;
        const auto lq=[](unsigned base,unsigned vf,int16_t off){return 0xd8000000u|base<<21|vf<<16|uint16_t(off);};
        const auto sq=[](unsigned base,unsigned vf,int16_t off){return 0xf8000000u|base<<21|vf<<16|uint16_t(off);};
        const auto value=[&](){return (rnd()&0x807fffffu)|((110+rnd()%34)<<23);};
        uint32_t v[4],m[4][4];for(auto &w:v)w=value();for(auto &row:m)for(auto &w:row)w=value();
        if(homogeneous && seed%8==1)v[3]=0;   // division by zero follows ps2f (max magnitude)
        std::memcpy(r.memory.hostPointer(point,16),v,16);std::memcpy(r.memory.hostPointer(matrix,64),m,64);
        std::vector<uint32_t> words;
        if(homogeneous){words={lq(3,20,0),0x27a201b0u,0x4bf403bcu,0x4a0003bfu,0x4be0a51cu,sq(2,20,0)};}
        else{words={lq(3,15,0),lq(4,11,0),lq(4,12,0x10),lq(4,13,0x20),lq(4,14,0x30),0x4bc007c3u,0x4bef5aeau,0x4bef632au,0x4bef6b6au,0x4bef73aau,0x4b0b583du,0x4b0bf8beu,0x4b0bf94bu,
            0x4a8c603cu,0x4a8cf8beu,0x4a8cf94bu,0x4a4d683du,0x4a4df8bcu,0x4a4df94bu,0x4a2e703du,0x4a2e00beu,0x4a2e0148u,sq(2,5,0)};}
        std::memcpy(r.memory.hostPointer(code,4*words.size()),words.data(),4*words.size());
        R5900Context x{};x.r[3].ud[0]=point;x.r[4].ud[0]=matrix;x.r[2].ud[0]=out;
        for(unsigned i=0;i<words.size();++i){const uint32_t w=words[i];if(!((w>>26)==0x12 || (w>>26)==0x36 || (w>>26)==0x3e))continue;
            if(!vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+4*i)){std::fprintf(stderr,"row idiom seed%u word %u unclaimed\n",seed,i);check(false);}}
        const auto M=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
        const auto A=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
        uint32_t expect[4];
        if(homogeneous){uint32_t f=0;const auto saved=_mm_getcsr();const auto q=bozo::ps2f::div(0x3f800000,v[3],f);_mm_setcsr(saved);for(unsigned i=0;i<4;++i)expect[i]=M(v[i],q);}
        else{constexpr unsigned order[4][4]={{0,1,2,3},{1,0,2,3},{2,1,0,3},{3,1,2,0}};
            for(unsigned row=0;row<4;++row){uint32_t t[4];for(unsigned i=0;i<4;++i)t[i]=M(m[row][i],v[i]);
                expect[row]=A(A(A(t[order[row][0]],t[order[row][1]]),M(0x3f800000,t[order[row][2]])),M(0x3f800000,t[order[row][3]]));}}
        uint32_t got[4];std::memcpy(got,r.memory.hostPointer(out,16),16);
        if(std::memcmp(got,expect,16)){std::fprintf(stderr,"row idiom seed%u %08x %08x %08x %08x / %08x %08x %08x %08x\n",seed,got[0],got[1],got[2],got[3],expect[0],expect[1],expect[2],expect[3]);check(false);}
        R5900Context y=x;bool rejected=false;try{vp2s::nativeMathInstruction(r.memory.flat(),&y,&r,code+4*unsigned(words.size()-1));}catch(const std::runtime_error &){rejected=true;}check(rejected);
        // One changed arithmetic word leaves the block unrecognised.
        const uint32_t broken=words[homogeneous?4:10]^0x40;std::memcpy(r.memory.hostPointer(code+4*(homogeneous?4:10),4),&broken,4);
        check(!vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+4*unsigned(words.size()-1)));
        ++rowCases;
    }
    std::printf("%u inlined row-point/homogeneous-divide blocks match the VU0 oracle\n",rowCases);
    // Matrix product (mod001 2b84a0): A rows (vf1..vf4) at v1, B rows (vf5..vf8) at
    // v0, each output row stored over A; plain EE words between some COP2 words.
    unsigned composeCases=0;
    for(unsigned seed=0;seed<256;++seed){
        auto &r=native.runtime();std::memset(r.memory.flat(),0x3c+seed,bozo::kFlatBytes);
        uint32_t state=seed*3266489917u+3;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
        const uint32_t code=0x1c4000+256*(rnd()%16),a=0x1d0000+16*(rnd()%64),b=0x1d4000+16*(rnd()%64);
        const auto lq=[](unsigned base,unsigned vf,int16_t off){return 0xd8000000u|base<<21|vf<<16|uint16_t(off);};
        const auto sq=[](unsigned base,unsigned vf,int16_t off){return 0xf8000000u|base<<21|vf<<16|uint16_t(off);};
        uint32_t ma[4][4],mb[4][4];for(auto *m:{&ma,&mb})for(auto &row:*m)for(auto &w:row)w=(rnd()&0x807fffffu)|((112+rnd()%30)<<23);
        std::memcpy(r.memory.hostPointer(a,64),ma,64);std::memcpy(r.memory.hostPointer(b,64),mb,64);
        std::vector<uint32_t> words;
        for(unsigned k=0;k<4;++k)words.push_back(lq(3,1+k,int16_t(16*k)));
        for(unsigned k=0;k<4;++k){words.push_back(lq(2,5+k,int16_t(16*k)));if(rnd()%3==0)words.push_back(0);}
        for(uint32_t w:{0x4be129bcu,0x4be130bdu,0x4be138beu,0x4be1404bu,0x4be229bcu,0x4be230bdu,0x4be238beu,0x4be2408bu,
                        0x4be329bcu,0x4be330bdu,0x4be338beu,0x4be340cbu,0x4be429bcu,0x4be430bdu,0x4be438beu,0x4be4410bu})words.push_back(w);
        for(unsigned k=0;k<4;++k)words.push_back(sq(3,1+k,int16_t(16*k)));
        std::memcpy(r.memory.hostPointer(code,4*words.size()),words.data(),4*words.size());
        R5900Context x{};x.r[3].ud[0]=a;x.r[2].ud[0]=b;
        for(unsigned i=0;i<words.size();++i){const uint32_t w=words[i];if(!w)continue;
            if(!vp2s::nativeMathInstruction(r.memory.flat(),&x,&r,code+4*i)){std::fprintf(stderr,"compose idiom seed%u word %u unclaimed\n",seed,i);check(false);}}
        const auto M=[](uint32_t p,uint32_t q){uint32_t f=0;return bozo::ps2f::mul(p,q,f);};
        const auto A=[](uint32_t p,uint32_t q){uint32_t f=0;return bozo::ps2f::addNearest(p,q,f);};
        uint32_t expect[4][4],got[4][4];
        for(unsigned k=0;k<4;++k)for(unsigned i=0;i<4;++i)expect[k][i]=A(A(A(M(mb[0][i],ma[k][0]),M(mb[1][i],ma[k][1])),M(mb[2][i],ma[k][2])),M(mb[3][i],ma[k][3]));
        std::memcpy(got,r.memory.hostPointer(a,64),64);
        if(std::memcmp(got,expect,64)){std::fprintf(stderr,"compose idiom seed%u row0 %08x %08x / %08x %08x\n",seed,got[0][0],got[0][1],expect[0][0],expect[0][1]);check(false);}
        R5900Context y=x;bool rejected=false;try{vp2s::nativeMathInstruction(r.memory.flat(),&y,&r,code+4*unsigned(words.size()-1));}catch(const std::runtime_error &){rejected=true;}check(rejected);
        ++composeCases;
    }
    std::printf("%u inlined matrix-product blocks match the VU0 oracle\n",composeCases);
    // mod003 3e7158: SP+160 through the hit object's rows (358cd8 -> +150), in place.
    {unsigned hitCases=0;auto &r=native.runtime();
        const auto M=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
        const auto A=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
        uint64_t state=0x13198a2e03707344ull;const auto next=[&](){state^=state<<13;state^=state>>7;state^=state<<17;return state;};
        const auto real=[&](float range){return std::bit_cast<uint32_t>(float(int64_t(next()%2000001)-1000000)/1000000.f*range);};
        for(unsigned seed=0;seed<512;++seed){
            std::memset(r.memory.flat(),0x11+seed,bozo::kFlatBytes);
            const uint32_t object=0x1d0000+0x400*(seed%16),stack=seed&1?0x70003700:0x1ffd000;
            uint32_t rows[4][4],v[4];for(auto &row:rows)for(auto &w:row)w=real(seed%3?2.f:500.f);for(auto &w:v)w=real(1000.f);
            if(seed%32==7)rows[2][1]=0x00000077u;   // a denormal row lane reads as zero
            std::memcpy(r.memory.hostPointer(0x358cd0+8,4),&object,4);std::memcpy(r.memory.hostPointer(object+0x150,64),rows,64);
            std::memcpy(r.memory.hostPointer(stack+0x160,16),v,16);
            R5900Context c{};c.r[29].uw[0]=stack;c.r[3].uw[0]=stack+0x160;
            for(uint32_t pc=0x3e7158;pc<=0x3e716c;pc+=4)check(vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,pc));
            uint32_t expect[4],got[4];if(seed%32==7)rows[2][1]=0;
            for(unsigned i=0;i<4;++i)expect[i]=A(A(A(M(rows[0][i],v[0]),M(rows[1][i],v[1])),M(rows[2][i],v[2])),M(rows[3][i],0x3f800000u));
            std::memcpy(got,r.memory.hostPointer(stack+0x160,16),16);
            if(std::memcmp(got,expect,16)){std::fprintf(stderr,"hit point seed%u %08x/%08x\n",seed,got[0],expect[0]);check(false);}
            ++hitCases;
        }
        for(unsigned which=0;which<2;++which){R5900Context c{};c.r[29].uw[0]=0x1ffd000;c.r[3].uw[0]=which?0x1ffd170:0x1ffd160;bool rejected=false;
            if(!which){const uint32_t none=0;std::memcpy(r.memory.hostPointer(0x358cd0+8,4),&none,4);}
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x3e716c);}catch(const std::runtime_error &){rejected=true;}check(rejected);}
        std::printf("%u hit-point transforms match the VU0 oracle; 2 rejections\n",hitCases);}
    // Box pair separating-axis test (mod001 1f82c0) against the listing's
    // sequential evaluation (tests/bozo_box_pair_oracle.inc): the first compared
    // pair with separation > radius (FPU compare) returns 0, else 1.
    {unsigned boxCases=0,reached[16]{};
        const auto M=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
        const auto A=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
        const auto S=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::subNearest(a,b,f);};
        const auto ABS=[](uint32_t a){return a&0x7fffffffu;};
        const auto key=[](uint32_t x){const int64_t m=(x&0x7f800000u)?int64_t(x&0x7fffffffu):0;return (x&0x80000000u)?-m:m;};
        uint64_t state=0x243f6a8885a308d3ull;const auto next=[&](){state^=state<<13;state^=state>>7;state^=state<<17;return state;};
        const auto real=[&](float range){return std::bit_cast<uint32_t>(float(int64_t(next()%2000001)-1000000)/1000000.f*range);};
        for(unsigned seed=0;seed<8192;++seed){
            std::array<std::array<uint32_t,4>,8> R{};std::array<uint32_t,4> sc{},ca{},ea{},cb{},eb{};
            const float spread=seed%4==0?0.2f:seed%4==1?2.f:seed%4==2?6.f:20.f;   // centre distance vs extents
            for(unsigned r=0;r<8;++r)for(unsigned l=0;l<4;++l)R[r][l]=(r%4==3)?(l<3?real(spread):0x3f800000u):real(1.f);
            for(unsigned l=0;l<4;++l){ca[l]=real(spread);cb[l]=real(spread);ea[l]=std::bit_cast<uint32_t>(std::fabs(std::bit_cast<float>(real(4.f))));eb[l]=std::bit_cast<uint32_t>(std::fabs(std::bit_cast<float>(real(4.f))));}
            sc[0]=std::bit_cast<uint32_t>(0.5f+std::fabs(std::bit_cast<float>(real(1.f))));sc[1]=std::bit_cast<uint32_t>(0.5f+std::fabs(std::bit_cast<float>(real(1.f))));
            sc[2]=real(1e6f);sc[3]=real(1e6f);   // unused lanes
            if(seed%64==5)R[1][2]=0x00000042u;     // a denormal input
#include "bozo_box_pair_oracle.inc"
            unsigned axis=15;for(unsigned k=0;k<15;++k)if(!(key(pairs[k][0])<=key(pairs[k][1]))){axis=k;break;}
            const uint32_t expect=axis==15?1:0;
            const auto csr=_mm_getcsr();const auto got=vp2s::nativeBoxPairTest(R,sc,ca,ea,cb,eb);check(_mm_getcsr()==csr);
            if(got!=expect){std::fprintf(stderr,"box pair seed%u axis%u got%u\n",seed,axis,got);check(false);}
            ++reached[axis];++boxCases;
        }
        unsigned axes=0;for(unsigned k=0;k<16;++k)axes+=reached[k]!=0;
        std::printf("%u box-pair tests match the listing (%u of 16 outcomes reached:",boxCases,axes);for(unsigned k=0;k<16;++k)std::printf(" %u",reached[k]);std::printf(")\n");
        check(axes==16);
    }
    // 237320 tail: the palette is kept at the VIF0 kick (no transfer); 218220
    // then skins natively. The oracle is generated from the mode-1 listings by
    // sequential evaluation (tests/bozo_skin_oracle.inc), lane by lane.
    unsigned skinVertices=0,skinRejects=0;
    for(unsigned seed=0;seed<160;++seed){
        auto &r=native.runtime();std::memset(r.memory.flat(),0,bozo::kFlatBytes);
        uint32_t state=seed*2654435761u+12345;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
        const auto fl=[&](int lo,int span){return (rnd()&0x807fffffu)|uint32_t((lo+int(rnd()%unsigned(span)))<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(r.memory.hostPointer(at,4),&v,4);};
        const auto put8=[&](uint32_t at,uint8_t v){std::memcpy(r.memory.hostPointer(at,1),&v,1);};
        using V=std::array<uint32_t,4>;const auto load=[&](uint32_t at){V v;std::memcpy(v.data(),r.memory.hostPointer(at,16),16);return v;};
        const uint32_t skeleton=0x1e5000,counts=0x1e5800,manager=0x1e5c00,desc=0x1e6000,table=0x1e6400,positions=0x1e8000,normals=0x1ec000,packedAt=0x1f0000;
        const unsigned bones=1+seed%12;put(skeleton+0x34,counts);put(counts+4,bones);
        std::vector<V> rows(4*bones);
        for(unsigned i=0;i<4*bones;++i){for(auto &x:rows[i])x=fl(118,12);std::memcpy(r.memory.hostPointer(0x70000010+16*i,16),rows[i].data(),16);}
        put(0x7000000c,0x6c000010u|(4*bones)<<16);
        R5900Context u{};u.r[20].ud[0]=skeleton;u.r[6].ud[0]=4*bones+1;auto u0=u;
        for(uint32_t pc:{0x237568u,0x23756cu,0x237574u,0x23757cu,0x2375a4u})check(vp2s::nativeMathInstruction(r.memory.flat(),&u,&r,pc));
        check(!std::memcmp(&u,&u0,sizeof(u)));
        // Scratchpad reuse after the kick must not change the uploaded palette.
        for(unsigned i=0;i<4*bones;++i)std::memset(r.memory.hostPointer(0x70000010+16*i,16),0x55,16);
        put(0x35ddb8,manager);put8(manager+0x51,1);
        const unsigned vertices=1+rnd()%24;const bool packed=seed%3!=0,rigid=seed%5==1;const unsigned index=seed%2;
        put(desc,seed%17==4?0:vertices);put(desc+4,positions);put(desc+8,normals);put(desc+0x14,packed?packedAt:0);put(desc+0x1c+4*index,table);
        put8(table+6,rigid);put(table+8,0x100);put(table+0xc,0x400);
        const unsigned processed=seed%17==4?1:vertices;
        struct In{V p,n;int16_t t[4];unsigned count;uint8_t first;int8_t extra[3];uint32_t w[4];};std::vector<In> in(processed);
        uint32_t bonesAt=table+0x100,weightsAt=table+0x400,countsAt=table+0x10;
        for(auto &v:in){
            for(auto &x:v.p)x=fl(120,10);for(auto &x:v.n)x=fl(122,6);for(auto &x:v.t)x=int16_t(rnd()&0xffff);
            v.count=rigid?0:rnd()%5;v.first=uint8_t(16+4*(rnd()%bones));put8(bonesAt++,v.first);
            if(!rigid){put8(countsAt++,uint8_t(v.count));const unsigned extras=v.count==0?0:v.count>=3?3:v.count;
                for(unsigned j=0;j<extras;++j){v.extra[j]=int8_t(rnd()%bones);put8(bonesAt++,uint8_t(v.extra[j]));}
                const unsigned ws=extras+1;for(unsigned j=0;j<ws;++j){v.w[j]=fl(122,5);put(weightsAt,v.w[j]);weightsAt+=4;}}
        }
        for(unsigned k=0;k<processed;++k){std::memcpy(r.memory.hostPointer(positions+16*k,16),in[k].p.data(),16);std::memcpy(r.memory.hostPointer(normals+16*k,16),in[k].n.data(),16);
            if(packed)std::memcpy(r.memory.hostPointer(packedAt+8*k,8),in[k].t,8);}
        R5900Context x{};for(unsigned i=1;i<32;++i)x.r[i].ud[0]=0x2468000000000000ull+i;x.r[5].ud[0]=desc;x.r[6].ud[0]=index;auto y=x;
        check(vp2s::nativeMathInstruction(r.memory.flat(),&y,&r,0x21822c));
        x.r[3].sd[0]=int32_t(packed?packedAt:0);check(!std::memcmp(&x,&y,sizeof(x)));
        const auto M=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
        const auto A=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);};
        const auto I12=[](uint32_t v){volatile float f=float(int32_t(v))*(1.f/4096.f);return std::bit_cast<uint32_t>(f);};
        const auto F12=[](uint32_t v){if((v&0x7f800000)==0x7f800000)return v&0x80000000?0x80000000u:0x7fffffffu;
            const double d=double(std::bit_cast<float>((v&0x7f800000)?v:(v&0x80000000)))*4096.;
            return d>=2147483647.?0x7fffffffu:d<=-2147483648.?0x80000000u:uint32_t(int32_t(d));};
        const auto mem=[&](int q){check(q>=16 && q<16+int(4*bones));return rows[unsigned(q-16)];};
        for(unsigned k=0;k<processed;++k){
            const auto &v=in[k];V vf[12]{};vf[5]=v.p;vf[6]=v.n;for(unsigned i=0;i<4;++i)vf[11][i]=uint32_t(int32_t(v.t[i]));
            int vi[5]={0,v.first,v.extra[0],v.extra[1],v.extra[2]};for(unsigned j=0;j<4;++j)vf[7+j][0]=v.w[j];
            const unsigned program=(packed?0x340:0)+(rigid||v.count==0?0:v.count==1?(packed?0x78:0x58):v.count==2?(packed?0x170:0x108):(packed?0x2d0:0x200));
            V vf30{},vf31{},vf29{};
#include "bozo_skin_oracle.inc"
            const auto gotP=load(positions+16*k),gotN=load(normals+16*k);
            if(gotP!=vf30 || gotN!=vf31){std::fprintf(stderr,"skin seed%u vertex%u program %x: P %08x %08x %08x %08x / %08x %08x %08x %08x N %08x / %08x\n",seed,k,program,
                gotP[0],gotP[1],gotP[2],gotP[3],vf30[0],vf30[1],vf30[2],vf30[3],gotN[0],vf31[0]);check(false);}
            if(packed){int16_t h[4];std::memcpy(h,r.memory.hostPointer(packedAt+8*k,8),8);
                for(unsigned i=0;i<4;++i)if(uint16_t(h[i])!=uint16_t(vf29[i])){std::fprintf(stderr,"skin seed%u vertex%u program %x packed lane %u %04x / %04x\n",seed,k,program,i,uint16_t(h[i]),uint16_t(vf29[i]));check(false);}}
            ++skinVertices;
        }
        // A bone outside the uploaded palette rejects before any write.
        put8(table+0x100,uint8_t(16+4*bones));for(unsigned k=0;k<processed;++k)std::memcpy(r.memory.hostPointer(positions+16*k,16),in[k].p.data(),16);
        auto z=x;try{vp2s::nativeMathInstruction(r.memory.flat(),&z,&r,0x21822c);check(false);}catch(const std::runtime_error &){++skinRejects;}
        check(load(positions)==in[0].p);
    }
    std::printf("%u mod001 218220 skinned vertices match the mode-1 listings (%u out-of-palette rejects)\n",skinVertices,skinRejects);
    // Animated packet UVs. The original loader 219620 (with its mesh cursors
    // and element methods) builds the scratchpad description; a test-only
    // oracle reproduces the original pipeline from it (toSPR chain, callback
    // 29cfe0 + micro 0xc80 with ps2f arithmetic, fromSPR destination chain).
    // The native rewrite reads only the records and must produce the same packets.
    unsigned uvCases=0,uvChunks=0,uvWords=0,uvEmpty=0,skinMeshes=0,morphMeshes=0;
    for(unsigned seed=0;seed<96;++seed){
        auto &a=old.runtime(),&b=native.runtime();std::memset(a.memory.flat(),0,bozo::kFlatBytes);
        uint32_t state=seed*747796405u+2891336453u;const auto next=[&](){state=state*1664525u+1013904223u;return state>>8;};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        const auto put16=[&](uint32_t at,uint16_t v){std::memcpy(a.memory.hostPointer(at,2),&v,2);};
        const auto get=[&](Runtime &r,uint32_t at){uint32_t v;std::memcpy(&v,r.memory.hostPointer(at,4),4);return v;};
        constexpr uint32_t object=0x200000,model=0x201000,iterator=0x201800,maskAt=0x200800,vtable=0x1591b0,data=0x210000,packets=0x240000,stack=0x1ffc000;
        // Morph seeds: one element of class 1536a0 (attributes 143: positions,
        // normals, the +34 stream through bit 100; no element data).
        const bool morphed=seed%6==1;
        const unsigned meshes=1+seed%4,elements=morphed?1:1+seed%3;const bool unaligned=seed&1;
        // Skinned seeds: element 0 is a skeleton interface (attributes eb:
        // positions, normals, UVs, packed vectors, element data).
        const bool skinned=seed%3==2;const unsigned bones=1+seed%8;
        const uint32_t base=data;  // 128-aligned, so both iterator modes agree
        put(object+0x54,model);put(model+0x644,iterator);put(iterator+8,data);put(iterator+0x14,seed%5==4?0:packets);
        std::memcpy(a.memory.hostPointer(iterator+0x1c,1),&unaligned,1);
        put(base+0x10+0x34,meshes);put(object+0x48,maskAt);put(object+0x58,skinned?0xeb:morphed?0x143:0x28);
        put(object+0x5c,0x202000);put(object+0x60,0x203000);put(object+0x68,0x204000);put(object+0x6c,0x204400);put(object+0x70,0x205000);
        const uint8_t mode=uint8_t(seed%2),rebuild=2,flags8d=uint8_t(seed%7==3?4:0);
        std::memcpy(a.memory.hostPointer(object+0x8a,1),&mode,1);std::memcpy(a.memory.hostPointer(object+0x8c,1),&rebuild,1);std::memcpy(a.memory.hostPointer(object+0x8d,1),&flags8d,1);
        uint32_t maskBits=0;for(unsigned j=0;j<meshes;++j)if(seed%5==0 || next()%4)maskBits|=1u<<j;put(maskAt,maskBits);
        const uint32_t packetStart=seed%5==4?base+0x70:packets+0x60;
        uint32_t source=base+0x70,packet=packetStart;
        for(unsigned j=0;j<meshes;++j){
            const uint16_t n=uint16_t(skinned || morphed?1+next()%40:seed%9==8?1+next()%3:4+next()%(seed%3==0?1000:120));const uint16_t offset=0x40;
            uint32_t span=(0x40+((n+3)/4)*16+0x2f)&~0xfu;
            if(skinned || morphed){const uint32_t at=0x40+((n+3)/4)*16,normalsAt=at+16*n,packedAt=normalsAt+16*n;const bool packed=next()%4!=0;
                put16(source+0x18,uint16_t(at));put16(source+0x1a,uint16_t(normalsAt));put16(source+0x34,packed?uint16_t(packedAt):0);
                span=(packedAt+16*((n+1u)/2)+0x2f)&~0xfu;
                for(uint32_t w=at;w<packedAt+16*((n+1u)/2);w+=4){
                    const uint32_t v=w<packedAt?(next()&0x807fffffu)|((120+next()%10)<<23):next()*13u;put(source+w,v);if(packet!=source)put(packet+w,0xdeadbeef);}}
            put16(source+0x16,n);put16(source+0x1c,offset);put16(source+4,uint16_t(span));
            if(packet!=source){put16(packet+4,uint16_t(span));}
            for(unsigned w=0;w<((n+3u)&~3u);++w){put(source+offset+4*w,next()*7u);if(packet!=source)put(packet+offset+4*w,0xdeadbeef);}
            source+=span;packet+=span;
        }
        put16(source+4,0);if(packet!=source)put16(packet+4,0);  // terminators (never inside the count)
        put(vtable+0x10,0x29cfe0);put(vtable+0x14,0x29cfd0);put(vtable+0x18,0x29d0d0);
        uint32_t descriptorAt=0x230000;
        const uint32_t skeletonVtable=0x154690;put(skeletonVtable+0x10,0x218220);put(skeletonVtable+0x14,0x236ba0);put(skeletonVtable+0x18,0x2372f0);
        for(unsigned i=0;i<elements;++i){
            const uint32_t e=0x206000+0x100*i,bits=0x206800+0x10*i;put(e,vtable);put(e+8,i+1<elements?e+0x100:object);
            if(morphed){
                // Morph: one table per call (included, applicable, N != 0), in mesh order.
                const uint32_t morphVtable=0x1536a0;put(morphVtable+0x10,0x218600);put(morphVtable+0x14,0x21b440);put(morphVtable+0x18,0x1f9ab0);
                put(e,morphVtable);const uint8_t enabled=1,all=uint8_t(seed%4==1);
                std::memcpy(a.memory.hostPointer(e+0xd,1),&enabled,1);std::memcpy(a.memory.hostPointer(e+0x18,1),&enabled,1);std::memcpy(a.memory.hostPointer(e+0x19,1),&all,1);
                uint32_t elementBits=0;for(unsigned j=0;j<meshes;++j)if(next()%3)elementBits|=1u<<j;put(bits,elementBits);put(e+0x24,bits);
                put(e+0x40,(next()&0x807fffffu)|((122+next()%5)<<23));put(e+0x44,(next()&0x807fffffu)|((122+next()%5)<<23));
                uint32_t tableAt=0x238000;put(e+0x38,tableAt);uint32_t meshSource=base+0x70;
                for(unsigned j=0;j<meshes;++j){
                    const bool included=flags8d==4 || (maskBits>>j&1);const uint16_t n=uint16_t(get(a,meshSource+0x14)>>16);
                    meshSource+=uint16_t(get(a,meshSource+4));if(!included || !n || !(all || (elementBits>>j&1)))continue;
                    const unsigned count=1+next()%std::min<unsigned>(n,6),packedCount=next()%(n+1u);
                    std::vector<unsigned> order(n);for(unsigned k=0;k<n;++k)order[k]=k;for(unsigned k=n-1;k>0;--k)std::swap(order[k],order[next()%(k+1)]);
                    const uint32_t size=(0x10+0x20*count+8*packedCount+15)&~15u;put(tableAt,count);put(tableAt+8,size);put(tableAt+0xc,8*packedCount);
                    for(unsigned k=0;k<count;++k){const uint32_t entry=tableAt+0x10+0x20*k;
                        for(unsigned l=0;l<7;++l)put(entry+4*l,(next()&0x807fffffu)|((118+next()%8)<<23));put(entry+0x1c,order[k]);}
                    for(unsigned k=0;k<2*packedCount;++k)put(tableAt+0x10+0x20*count+4*k,next()*11u);
                    tableAt+=size;}
                continue;}
            if(skinned && i==0){
                // Skeleton: tables for every included mesh from the +44 cursor.
                put(e,skeletonVtable);const uint8_t enabled=1,all=uint8_t(seed%4==1);
                std::memcpy(a.memory.hostPointer(e+0xd,1),&enabled,1);std::memcpy(a.memory.hostPointer(e+0x18,1),&enabled,1);std::memcpy(a.memory.hostPointer(e+0x19,1),&all,1);
                uint32_t elementBits=0;for(unsigned j=0;j<meshes;++j)if(next()%3)elementBits|=1u<<j;put(bits,elementBits);put(e+0x24,bits);
                uint32_t tableAt=0x238000;put(e+0x44,tableAt);put(e+0x48,tableAt);
                uint32_t meshSource=base+0x70;
                for(unsigned j=0;j<meshes;++j){
                    const bool included=flags8d==4 || (maskBits>>j&1);const uint16_t n=uint16_t(get(a,meshSource+0x14)>>16);
                    meshSource+=uint16_t(get(a,meshSource+4));if(!included)continue;
                    const bool rigid=next()%4==0;const uint32_t bonesAt=0x10+n,weightsAt=(bonesAt+4*n+3)&~3u,size=(weightsAt+16*n+1)&~1u;
                    put16(tableAt,uint16_t(size));std::memcpy(a.memory.hostPointer(tableAt+6,1),&rigid,1);put(tableAt+8,bonesAt);put(tableAt+0xc,weightsAt);
                    uint32_t boneAt=tableAt+bonesAt,w=tableAt+weightsAt;
                    for(unsigned k=0;k<n;++k){const uint8_t extra=uint8_t(next()%5);if(!rigid)std::memcpy(a.memory.hostPointer(tableAt+0x10+k,1),&extra,1);
                        const uint8_t first=uint8_t(16+4*(next()%bones));std::memcpy(a.memory.hostPointer(boneAt++,1),&first,1);
                        if(rigid)continue;const unsigned extras=extra==0?0:extra>=3?3:extra;
                        for(unsigned m=0;m<extras;++m){const uint8_t bone=uint8_t(next()%bones);std::memcpy(a.memory.hostPointer(boneAt++,1),&bone,1);}
                        for(unsigned m=0;m<=extras;++m){put(w,(next()&0x807fffffu)|((122+next()%5)<<23));w+=4;}}
                    tableAt+=size;}
                continue;}
            const uint8_t enabled=1,all=uint8_t(seed%4==1 && i==0);std::memcpy(a.memory.hostPointer(e+0xd,1),&enabled,1);
            std::memcpy(a.memory.hostPointer(e+0x18,1),&enabled,1);std::memcpy(a.memory.hostPointer(e+0x19,1),&all,1);
            uint32_t elementBits=0;for(unsigned j=0;j<meshes;++j)if(next()%3)elementBits|=1u<<j;put(bits,elementBits);put(e+0x24,bits);
            put(e+0x34,descriptorAt);put(e+0x38,descriptorAt);
            const float scroll[2]={float(int(next()%2001)-1000)/1000.f,float(int(next()%2001)-1000)/4096.f};put(e+0x40,std::bit_cast<uint32_t>(scroll[0]));put(e+0x44,std::bit_cast<uint32_t>(scroll[1]));
            uint32_t meshSource=base+0x70;
            for(unsigned j=0;j<meshes;++j){
                const bool included=flags8d==4 || (maskBits>>j&1);const uint16_t n=uint16_t(get(a,meshSource+0x14)>>16);
                meshSource+=uint16_t(get(a,meshSource+4));
                if(!included || !(all || (elementBits>>j&1)))continue;
                const unsigned listed=1+next()%std::min<unsigned>(n,48);const uint32_t size=0x10+((listed+15)&~15u);
                put(descriptorAt,listed);put(descriptorAt+4,j);put(descriptorAt+8,size);
                for(unsigned k=0;k<std::max(listed,1u);++k){const uint8_t index=uint8_t(next()%n);std::memcpy(a.memory.hostPointer(descriptorAt+0x10+k,1),&index,1);}
                descriptorAt+=size;
            }
        }
        put(object+8,0x206000);
        constexpr uint32_t manager=0x207000,skeleton=0x207400,boneCounts=0x207800;
        if(morphed){put(0x35ddb8,manager);std::memcpy(a.memory.hostPointer(manager+0x51,1),"\1",1);}
        if(skinned){put(0x35ddb8,manager);std::memcpy(a.memory.hostPointer(manager+0x51,1),"\1",1);put(skeleton+0x34,boneCounts);put(boneCounts+4,bones);}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        if(skinned)for(auto *r:{&a,&b}){
            // The same palette uploaded through the native 237320 kick in each runtime.
            uint32_t rs=seed*97u+5;for(unsigned q=0;q<16*bones;++q){rs=rs*1664525u+1013904223u;const uint32_t v=(rs&0x807fffffu)|((118+(rs>>9)%12)<<23);std::memcpy(r->memory.hostPointer(0x70000010+4*q,4),&v,4);}
            const uint32_t tag=0x6c000010u|(4*bones)<<16;std::memcpy(r->memory.hostPointer(0x7000000c,4),&tag,4);
            R5900Context u{};u.r[20].ud[0]=skeleton;u.r[6].ud[0]=4*bones+1;check(vp2s::nativeMathInstruction(r->memory.flat(),&u,r,0x2375a4));}
        // Original loader.
        R5900Context x{};x.r[4].ud[0]=object;x.r[5].ud[0]=elements;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;
        a.functions.generated(0x219620)(a.memory.flat(),&x,&a);check(x.pc==0x170000);
        // Oracle: the original scratchpad pipeline from the loader's description.
        const auto mul=[](uint32_t p,uint32_t q){uint32_t f=0;return bozo::ps2f::mul(p,q,f);};(void)mul;
        const auto add=[](uint32_t p,uint32_t q){uint32_t f=0;return bozo::ps2f::addNearest(p,q,f);};
        const auto scrolled=[&](uint32_t half,uint32_t scroll){const auto sum=add(std::bit_cast<uint32_t>(float(half)/4096.f),scroll);
            const double fixed=double(std::bit_cast<float>(sum&0x7f800000?sum:sum&0x80000000))*4096.;
            return fixed>=2147483647.?0x7fffffff:fixed<=-2147483648.?int32_t(0x80000000u):int32_t(fixed);};
        const int16_t chunks=int16_t(get(a,object+0x84)&0xffff);check(chunks<16);if(chunks<=0)++uvEmpty;
        const uint32_t table=get(a,object+0x68+4*mode),groups=get(a,object+0x70);
        std::vector<uint8_t> spr(16384);unsigned group=0;
        uint32_t morphVu[32][4];for(auto &r:morphVu)for(auto &w:r)w=(next()&0x007fffffu)|((124+next()%6)<<23);
        for(int c=0;c<chunks;++c){
            const uint32_t count=get(a,table+16*c),chain=get(a,table+16*c+4),sadr=get(a,table+16*c+8);check(sadr>=0x70000000 && sadr<0x70004000);
            uint32_t at=chain,pos=sadr-0x70000000;
            for(unsigned guard=0;;++guard){check(guard<512);const auto lo=get(a,at),address=get(a,at+4);const unsigned qwc=lo&0xffff,id=(lo>>28)&7;
                check(pos+16*qwc<=spr.size());
                if(id==1){std::memcpy(spr.data()+pos,a.memory.hostPointer(at+16,16*qwc),16*qwc);pos+=16*qwc;at+=16+16*qwc;}
                else if(id==3){std::memcpy(spr.data()+pos,a.memory.hostPointer(address,16*qwc),16*qwc);pos+=16*qwc;at+=16;}
                else{check(id==7);std::memcpy(spr.data()+pos,a.memory.hostPointer(at+16,16*qwc),16*qwc);break;}}
            for(uint32_t g=0;g<count;++g,++group){const uint32_t rec=groups+0x5c*group;if(!get(a,rec))continue;
                const uint32_t dataAt=get(a,rec+0xc)-0x70000000;
                for(unsigned i=0;i<elements;++i){const uint32_t e=0x206000+0x100*i;const bool all=a.memory.hostPointer(e+0x19,1)[0]!=0;
                    if(!all && !(get(a,get(a,e+0x24)+4*(group>>5))>>(group&31)&1))continue;
                    if(morphed){
                        // 218600 with programs 0x7d8/0x848 transcribed pair by pair; the
                        // VU state persists across calls (stale lanes start random).
                        const uint32_t posAt=get(a,rec+4)-0x70000000,nrmAt=get(a,rec+8)-0x70000000,pk=get(a,rec+0x14),tbl=get(a,e+0x38);
                        const uint32_t wx=get(a,e+0x40),wy=get(a,e+0x44),one=0x3f800000;const unsigned count=get(a,tbl);
                        uint32_t in[5][4];unsigned idx=0,prev=0;
                        const auto load=[&](unsigned j){const uint32_t entry=tbl+0x10+0x20*j;std::memcpy(in[3],a.memory.hostPointer(entry,16),16);std::memcpy(in[4],a.memory.hostPointer(entry+16,16),16);
                            idx=in[4][3];check(idx<get(a,rec));std::memcpy(in[1],spr.data()+posAt+16*idx,16);std::memcpy(in[2],spr.data()+nrmAt+16*idx,16);};
                        const auto call=[&](){uint32_t flags=0;const auto saved=_mm_getcsr();const auto q=bozo::ps2f::rsqrt(one,morphVu[7][0],flags);_mm_setcsr(saved);
                            for(unsigned c2=0;c2<3;++c2){const auto n9=add(mul(in[2][c2],one),mul(in[4][c2],wx));morphVu[11][c2]=morphVu[9][c2];morphVu[9][c2]=n9;}
                            for(unsigned c2=0;c2<3;++c2){const auto n8=add(mul(in[1][c2],one),mul(in[3][c2],wx));morphVu[12][c2]=morphVu[8][c2];morphVu[8][c2]=n8;}
                            morphVu[7][0]=add(0,one);for(unsigned c2=0;c2<3;++c2)morphVu[6][c2]=mul(morphVu[9][c2],morphVu[9][c2]);
                            auto acc=mul(morphVu[7][0],morphVu[6][0]);acc=add(acc,mul(morphVu[7][0],morphVu[6][1]));morphVu[7][0]=add(acc,mul(morphVu[7][0],morphVu[6][2]));
                            morphVu[13][3]=in[2][3];for(unsigned c2=0;c2<3;++c2)morphVu[13][c2]=mul(morphVu[11][c2],q);};
                        // Positions keep their w: vf12.w is VU register history (see 218600 in native_math.cpp).
                        const auto store=[&](unsigned at){std::memcpy(spr.data()+posAt+16*at,morphVu[12],12);std::memcpy(spr.data()+nrmAt+16*at,morphVu[13],16);};
                        load(0);call();
                        if(count==1){call();store(idx);}
                        else{for(unsigned j=1;j<count;++j){prev=idx;load(j);call();store(prev);}call();store(idx);}
                        const uint32_t bytes=get(a,tbl+0xc);
                        if(pk && bytes){const uint32_t pkAt=pk-0x70000000,delta=tbl+0x10+0x20*count;
                            const auto i12=[](int16_t v){volatile float f=float(v)*(1.f/4096.f);return std::bit_cast<uint32_t>(f);};
                            for(uint32_t k=0;k<(bytes>>3);++k){int16_t v[4],d[4];std::memcpy(v,spr.data()+pkAt+8*k,8);std::memcpy(d,a.memory.hostPointer(delta+8*k,8),8);
                                for(unsigned l=0;l<4;++l){const auto r=add(mul(i12(v[l]),one),mul(i12(d[l]),wy));const auto b2=r&0x7f800000?r:r&0x80000000;
                                    const double fixed=(r&0x7f800000)==0x7f800000?(r&0x80000000?-3e9:3e9):double(std::bit_cast<float>(b2))*4096.;
                                    v[l]=int16_t(uint16_t(uint32_t(fixed>=2147483647.?0x7fffffff:fixed<=-2147483648.?int32_t(0x80000000u):int32_t(fixed))));}
                                std::memcpy(spr.data()+pkAt+8*k,v,8);}}
                        put(e+0x38,tbl+get(a,tbl+8));++morphMeshes;continue;}
                    if(skinned && i==0){
                        // 218220 on the scratchpad copy the loader described (desc = rec, index i).
                        std::memcpy(a.memory.hostPointer(0x70000000,spr.size()),spr.data(),spr.size());
                        R5900Context k{};k.r[5].ud[0]=rec;k.r[6].ud[0]=i;check(vp2s::nativeMathInstruction(a.memory.flat(),&k,&a,0x21822c));
                        std::memcpy(spr.data(),a.memory.hostPointer(0x70000000,spr.size()),spr.size());++skinMeshes;continue;}
                    const uint32_t d=get(a,rec+0x1c+4*i)-0x70000000;uint32_t listed;std::memcpy(&listed,spr.data()+d,4);
                    for(uint32_t k=0;k<std::max(listed,1u);++k){const auto index=spr[d+0x10+k];uint32_t w;std::memcpy(&w,spr.data()+dataAt+4*index,4);
                        w=(uint32_t(scrolled(w>>16,get(a,e+0x44)))<<16)|(uint32_t(scrolled(w&0xffff,get(a,e+0x40)))&0xffff);std::memcpy(spr.data()+dataAt+4*index,&w,4);}
                }
            }
            for(uint32_t p=sadr-0x70000000,guard=0;;++guard){check(guard<512);uint32_t lo,address;std::memcpy(&lo,spr.data()+p,4);std::memcpy(&address,spr.data()+p+4,4);
                const unsigned qwc=lo&0xffff,id=(lo>>28)&7;if(id==7)break;check(id==1);
                std::memcpy(a.memory.hostPointer(address,16*qwc),spr.data()+p+16,16*qwc);uvWords+=4*qwc;p+=16+16*qwc;}
            ++uvChunks;
        }
        // Native rewrite at 219210 (s7 = object, s5 = elements, SP+e0 list).
        R5900Context y{};y.r[23].ud[0]=object;y.r[21].ud[0]=elements;y.r[29].ud[0]=stack;
        std::memcpy(b.memory.hostPointer(object+0x84,2),a.memory.hostPointer(object+0x84,2),2);
        for(unsigned i=0;i<elements;++i){const uint32_t e=0x206000+0x100*i;std::memcpy(b.memory.hostPointer(stack+0xe0+4*i,4),&e,4);}
        const auto fp=_mm_getcsr();check(vp2s::nativeMathInstruction(b.memory.flat(),&y,&b,0x219210));check(_mm_getcsr()==fp && y.r[2].sd[0]==chunks);
        const uint32_t firstPacket=seed%5==4?base+0x70:packets+0x60;
        if(std::memcmp(a.memory.hostPointer(firstPacket,0x18000),b.memory.hostPointer(firstPacket,0x18000),0x18000)){
            for(uint32_t at=firstPacket;at<firstPacket+0x18000;at+=4)if(get(a,at)!=get(b,at)){std::fprintf(stderr,"uv seed%u at %x oracle %08x native %08x\n",seed,at,get(a,at),get(b,at));break;}check(false);}
        if(morphed)check(get(a,0x206000+0x38)==get(b,0x206000+0x38));   // the morph cursor advances alike
        ++uvCases;
    }
    std::printf("%u animated-UV models (%u loader chunks, %u packet words, %u without chunks) match the original scratchpad pipeline (%u skinned, %u morphed meshes)\n",uvCases,uvChunks,uvWords,uvEmpty,skinMeshes,morphMeshes);
    {   // Battle ribbon element (class 172b10, mod028 498940 + ground height 4ab140) on the
        // animator: one mesh of 12 vertices = two segments, the second past e+250 (stream
        // zeroed). Exact values: d = (1, 0), a = 4, b = 3, centre (10, 10) from O.
        std::string path=argv[1];const auto at=path.rfind("mod010");check(at!=std::string::npos);path.replace(at,6,"mod028");
        std::vector<uint8_t> elf;{FILE *f=std::fopen(path.c_str(),"rb");if(!f)throw std::runtime_error("mod028 image missing");
            std::fseek(f,0,SEEK_END);elf.resize(size_t(std::ftell(f)));std::fseek(f,0,SEEK_SET);check(std::fread(elf.data(),1,elf.size(),f)==elf.size());std::fclose(f);}
        auto &r=native.runtime();
        const auto put=[&](uint32_t a,uint32_t v){std::memcpy(r.memory.hostPointer(a,4),&v,4);};
        const auto putf=[&](uint32_t a,float v){put(a,std::bit_cast<uint32_t>(v));};
        const auto get=[&](uint32_t a){uint32_t v;std::memcpy(&v,r.memory.hostPointer(a,4),4);return v;};
        unsigned ribbonCases=0;
        for(unsigned ground=0;ground<2;++ground){
            std::memset(r.memory.flat(),0,bozo::kFlatBytes);
            {uint32_t ph,count;std::memcpy(&ph,elf.data()+0x1c,4);count=uint16_t(elf[0x2c]|elf[0x2d]<<8);
             for(uint32_t i=0;i<count;++i){uint32_t h[6];std::memcpy(h,elf.data()+ph+32*i,24);if(h[0]==1)std::memcpy(r.memory.hostPointer(h[2],h[4]),elf.data()+h[1],h[4]);}}
            constexpr uint32_t object=0x200000,model=0x201000,iterator=0x201800,maskAt=0x200800,data=0x210000,packets=0x240000,stack=0x1ffc000,e=0x206000,boundsAt=0x206800,vt=0x172b10;
            put(object+0x54,model);put(model+0x644,iterator);put(iterator+8,data);put(iterator+0x14,packets);
            put(data+0x10+0x34,1);put(object+0x48,maskAt);put(maskAt,1);put(object+0x58,5);put(object+0x84,1);put(object+8,e);
            const uint32_t source=data+0x70,packet=packets+0x60,span=0x200,posAt=0x40,streamAt=0x140;
            put(source+4,span);put(packet+4,span);put(source+0x14,12u<<16);put(source+0x18,posAt);put(source+0x1c,streamAt<<16);
            put(source+span+4,0);put(packet+span+4,0);
            for(unsigned v=0;v<12;++v){putf(source+posAt+16*v,-1);putf(source+posAt+16*v+4,-2);putf(source+posAt+16*v+8,-3);putf(source+posAt+16*v+12,1);put(source+streamAt+4*v,0x11110000u+v);}
            put(vt+0x10,0x498940);put(e,vt);put(e+8,object);r.memory.hostPointer(e+0x19,1)[0]=1;
            putf(e+0x40,100);putf(e+0x44,-50);putf(e+0x48,200);putf(e+0x50,1);putf(e+0x58,0);
            put(e+0x60,boundsAt);putf(e+0x64,1);putf(e+0x68,1);put(e+0x250,1);put(e+0x254,0);r.memory.hostPointer(e+0x258,1)[0]=uint8_t(ground);
            putf(boundsAt,1000);putf(boundsAt+4,-1000);putf(boundsAt+8,1000);putf(boundsAt+12,-1000);
            putf(e+0x70,110);putf(e+0x78,210);putf(e+0x80,118);putf(e+0x88,210);putf(e+0x90,2);
            // Ground: one tile x 100..200, z 200..300, 32-unit cells, height 1000 (h00 a hole,
            // filled from h01), scale 1/1024, offset -5: 995 everywhere it is asked.
            constexpr uint32_t control=0x1a0000,owner=0x1a1000,list=0x1a2000,tile=0x1a3000,grid=0x1a4000;
            put(0x36d880,control);r.memory.hostPointer(control+0x53,1)[0]=0x80;put(control+0x68,owner);put(owner+0xf0,list);put(list+0x118,1);put(list+0xd8,tile);
            putf(0x513318,-3000);putf(0x51331c,-3000);putf(0x513320,3000);putf(0x513324,3000);
            put(tile+0x270,100);put(tile+0x274,200);put(tile+0x278,200);put(tile+0x27c,300);put(tile+0x260,32);put(tile+0x264,32);put(tile+0x268,8);put(tile+0x250,grid);
            putf(tile+0x288,1.f/32);putf(tile+0x28c,1.f/32);putf(tile+0x284,1.f/1024);putf(tile+0x280,-5);
            for(unsigned k=0;k<64;++k){const uint16_t h=k==0?0x7fff:1000;std::memcpy(r.memory.hostPointer(grid+0x20+2*k,2),&h,2);}
            put(stack+0xe0,e);
            R5900Context y{};y.r[23].ud[0]=object;y.r[21].ud[0]=1;y.r[29].ud[0]=stack;
            check(vp2s::nativeMathInstruction(r.memory.flat(),&y,&r,0x219210) && y.r[2].sd[0]==1);
            const float xz[6][2]={{10,13},{10,7},{14,13},{14,7},{18,13},{18,7}};uint32_t f=0;
            const uint32_t lifted=bozo::ps2f::sub(std::bit_cast<uint32_t>(1045.f),0x3e4ccccdu,f);
            for(unsigned v=0;v<12;++v){const uint32_t p=packet+posAt+16*v;
                if(v<6)check(get(p)==std::bit_cast<uint32_t>(xz[v][0]) && get(p+8)==std::bit_cast<uint32_t>(xz[v][1]) && get(p+4)==(ground?lifted:std::bit_cast<uint32_t>(-2.f)));
                else check(get(p)==std::bit_cast<uint32_t>(-1.f) && get(p+4)==std::bit_cast<uint32_t>(-2.f) && get(p+8)==std::bit_cast<uint32_t>(-3.f));
                check(get(p+12)==std::bit_cast<uint32_t>(1.f) && get(packet+streamAt+4*v)==(v<6?0x11110000u+v:0u));}
            check(get(boundsAt)==std::bit_cast<uint32_t>(10.f) && get(boundsAt+4)==std::bit_cast<uint32_t>(18.f) && get(boundsAt+8)==std::bit_cast<uint32_t>(7.f) && get(boundsAt+12)==std::bit_cast<uint32_t>(13.f));
            check(get(e+0x254)==2);
            // Two tiles: the choice would read the frame's never-written y; refused.
            if(ground){put(list+0x118,2);put(list+0xdc,tile);put(e+0x254,0);bool refused=false;R5900Context z{};z.r[23].ud[0]=object;z.r[21].ud[0]=1;z.r[29].ud[0]=stack;
                try{vp2s::nativeMathInstruction(r.memory.flat(),&z,&r,0x219210);}catch(const std::runtime_error &){refused=true;}check(refused);}
            ++ribbonCases;
        }
        std::printf("%u battle ribbon meshes (flat and ground-following, 4ab140 one tile) and the several-tile refusal passed\n",ribbonCases);
    }
    {   // Battle colour element (class 171f20, mod028 47a910) on the animator, with the pair its task's
        // 47a9b0 loaded (47aa5c): one mesh of 6 vertices, three pairs. Phase .25, entries .5, 1, -.75:
        // t = .75 (u .25), 1.25 - 2 -> .75, .5 (u .5). Pair (64, 32, 127.5, 128) / (0, .5, 1, 3).
        std::string path=argv[1];const auto at=path.rfind("mod010");check(at!=std::string::npos);path.replace(at,6,"mod028");
        std::vector<uint8_t> elf;{FILE *f=std::fopen(path.c_str(),"rb");if(!f)throw std::runtime_error("mod028 image missing");
            std::fseek(f,0,SEEK_END);elf.resize(size_t(std::ftell(f)));std::fseek(f,0,SEEK_SET);check(std::fread(elf.data(),1,elf.size(),f)==elf.size());std::fclose(f);}
        auto &r=native.runtime();
        const auto put=[&](uint32_t a,uint32_t v){std::memcpy(r.memory.hostPointer(a,4),&v,4);};
        const auto putf=[&](uint32_t a,float v){put(a,std::bit_cast<uint32_t>(v));};
        const auto get=[&](uint32_t a){uint32_t v;std::memcpy(&v,r.memory.hostPointer(a,4),4);return v;};
        std::memset(r.memory.flat(),0,bozo::kFlatBytes);
        {uint32_t ph,count;std::memcpy(&ph,elf.data()+0x1c,4);count=uint16_t(elf[0x2c]|elf[0x2d]<<8);
         for(uint32_t i=0;i<count;++i){uint32_t h[6];std::memcpy(h,elf.data()+ph+32*i,24);if(h[0]==1)std::memcpy(r.memory.hostPointer(h[2],h[4]),elf.data()+h[1],h[4]);}}
        constexpr uint32_t object=0x200000,model=0x201000,iterator=0x201800,maskAt=0x200800,data=0x210000,packets=0x240000,stack=0x1ffc000,e=0x206000,table=0x207000,owner=0x208000,vt=0x171f20;
        put(object+0x54,model);put(model+0x644,iterator);put(iterator+8,data);put(iterator+0x14,packets);
        put(data+0x10+0x34,1);put(object+0x48,maskAt);put(maskAt,1);put(object+0x58,4);put(object+0x84,1);put(object+8,e);
        const uint32_t source=data+0x70,packet=packets+0x60,span=0x200,streamAt=0x140;
        put(source+4,span);put(packet+4,span);put(source+0x14,6u<<16);put(source+0x1c,streamAt<<16);put(source+span+4,0);put(packet+span+4,0);
        for(unsigned v=0;v<8;++v)put(source+streamAt+4*v,0x55555555u);
        put(vt+0x10,0x47a910);put(e,vt);put(e+8,object);r.memory.hostPointer(e+0x19,1)[0]=1;
        put(e+0x34,table);putf(e+0x40,0.25f);putf(table,0.5f);putf(table+4,1.f);putf(table+8,-0.75f);
        put(stack+0xe0,e);
        {R5900Context y{};y.r[23].ud[0]=object;y.r[21].ud[0]=1;y.r[29].ud[0]=stack;bool refused=false;   // no pair loaded yet
         try{vp2s::nativeMathInstruction(r.memory.flat(),&y,&r,0x219210);}catch(const std::runtime_error &){refused=true;}check(refused);}
        const float pair[8]={64,32,127.5f,128, 0,0.5f,1,3};std::memcpy(r.memory.hostPointer(owner+0x9a0,32),pair,32);
        {R5900Context l{};l.r[16].uw[0]=owner;l.r[17].uw[0]=e;l.r[2].uw[0]=owner+0x9b0;bool refused=false;   // not the +9a0 pair
         try{vp2s::nativeMathInstruction(r.memory.flat(),&l,&r,0x47aa5c);}catch(const std::runtime_error &){refused=true;}check(refused);
         l.r[2].uw[0]=owner+0x9a0;check(vp2s::nativeMathInstruction(r.memory.flat(),&l,&r,0x47aa5c) && vp2s::nativeMathInstruction(r.memory.flat(),&l,&r,0x47aa60));}
        for(unsigned i=0;i<8;++i)putf(owner+0x9a0+4*i,9);   // later object changes do not reach the task's pair
        R5900Context y{};y.r[23].ud[0]=object;y.r[21].ud[0]=1;y.r[29].ud[0]=stack;
        check(vp2s::nativeMathInstruction(r.memory.flat(),&y,&r,0x219210) && y.r[2].sd[0]==1);
        const uint32_t A=48u|24u<<8|95u<<16|96u<<24,B=32u|16u<<8|64u<<16|65u<<24,want[6]={A,A,A,A,B,B};
        for(unsigned v=0;v<6;++v)check(get(packet+streamAt+4*v)==want[v]);
        check(get(e+0x34)==table+8);   // the final cursor - 4, as 47a9a0 writes it
        std::puts("battle colour element: the task's own pair blended into the vertex stream (three pairs, wrap, cursor) and both refusals passed");
        // Battle flash element (class 172080, mod028 47ccf0) after its task's 47cdc0 (47cdd8: 255.0):
        // 6 vertices = two groups. Indices 0x000002 and 0x010000 into the factor table: 64 (k = .5)
        // and 128 (k = 1). Group colours (100, 50, 10, 80) and (200, 0, 255, 255):
        // (150, 75, 15, 80 + 175 * .5 = 167.5 -> 167) and (400 -> 0x90, 0, 510 -> 0xfe, 255).
        put(vt+0x10,0x47ccf0);put(e,0x172080);put(e+0x3c,5);put(e+0x34,table);put(e+0x38,0x209000);
        const uint8_t idx[11]={9,9,9,9,9,2,0,0,0,0,1};std::memcpy(r.memory.hostPointer(table,11),idx,11);
        r.memory.hostPointer(0x209000+2,1)[0]=64;r.memory.hostPointer(0x209000+0x10000,1)[0]=128;
        put(0x172080+0x10,0x47ccf0);
        const uint32_t g0=100u|50u<<8|10u<<16|80u<<24,g1=200u|0u<<8|255u<<16|255u<<24;
        for(unsigned v=0;v<8;++v)put(source+streamAt+4*v,v<4?(v?0x11111111u:g0):(v==4?g1:0x22222222u));
        {R5900Context y2{};y2.r[23].ud[0]=object;y2.r[21].ud[0]=1;y2.r[29].ud[0]=stack;bool refused=false;   // its 47cdc0 never ran
         try{vp2s::nativeMathInstruction(r.memory.flat(),&y2,&r,0x219210);}catch(const std::runtime_error &){refused=true;}check(refused);}
        {R5900Context l{};l.r[4].uw[0]=e;l.r[8].uw[0]=0x437f0000;check(vp2s::nativeMathInstruction(r.memory.flat(),&l,&r,0x47cdd8) && vp2s::nativeMathInstruction(r.memory.flat(),&l,&r,0x47cddc));}
        put(e+0x3c,5);
        {R5900Context y2{};y2.r[23].ud[0]=object;y2.r[21].ud[0]=1;y2.r[29].ud[0]=stack;check(vp2s::nativeMathInstruction(r.memory.flat(),&y2,&r,0x219210));}
        const uint32_t F0=150u|75u<<8|15u<<16|167u<<24,F1=0x90u|0u<<8|0xfeu<<16|255u<<24;
        for(unsigned v=0;v<8;++v)check(get(packet+streamAt+4*v)==(v<4?F0:F1));
        check(get(e+0x3c)==11);
        std::puts("battle flash element: the task's 255 and the factor table brighten each 4-vertex group; refusal without 47cdc0 passed");
        // Battle trail element (class 1730d0, mod028 4a83e0): positions from the two rings, newest first.
        // Entry 2 (+7c0 2, count 0), toggle set: A2, B2, A1, B1, A0, B0; count stored 3 - 1, toggle set again.
        {constexpr uint32_t posAt=0x40;put(object+0x58,1);put(source+0x18,posAt);put(e,0x1730d0);put(0x1730d0+0x10,0x4a83e0);
         put(e+0x7c0,2);put(e+0x808,0);put(e+0x80c,1);
         for(unsigned k=0;k<3;++k)for(unsigned l=0;l<4;++l){putf(e+0x40+16*k+4*l,float(100+10*k+l));putf(e+0x400+16*k+4*l,float(200+10*k+l));}
         R5900Context y3{};y3.r[23].ud[0]=object;y3.r[21].ud[0]=1;y3.r[29].ud[0]=stack;check(vp2s::nativeMathInstruction(r.memory.flat(),&y3,&r,0x219210));
         const unsigned ring[6]={100,200,100,200,100,200},entry[6]={2,2,1,1,0,0};
         for(unsigned v=0;v<6;++v)for(unsigned l=0;l<4;++l)check(get(packet+posAt+16*v+4*l)==std::bit_cast<uint32_t>(float(ring[v]+10*entry[v]+l)));
         check(get(e+0x808)==2 && (get(e+0x80c)&0xff)==1);
         std::puts("battle trail element: two rings interleaved into the positions, newest first, count and toggle stored back");}
    }
    std::printf("%u matrix/quaternion/Euler ABI cases, %u component products, %u whole CPU/RAM sphere getters,8192 independent scale cases and%u whole CPU/RAM axis-angle cases passed; no VU/device state\n",cases,products,sphereCases,axisCases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

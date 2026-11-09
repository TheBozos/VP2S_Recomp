#include "native_math.h"
#include "../port/vp2s_math.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "distance_original_functions.h"
#include "distance_adapted_functions.h"
#include "bozo/ops_fpu.h"
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <immintrin.h>
#include <source_location>
#include <stdexcept>
namespace {
// The engine's VU0 arithmetic definitions (ops_fpu.h, reference-capture
// settled): sums and products round the exact result to nearest even.
uint32_t vu0Product(uint32_t a,uint32_t b){uint32_t flags=0;return bozo::ps2f::mul(a,b,flags);}
uint32_t vu0Sum(uint32_t a,uint32_t b){uint32_t flags=0;return bozo::ps2f::addNearest(a,b,flags);}
std::array<uint32_t,12> movementRotationOracle(const uint32_t *raw){
    const auto product=vu0Product;const auto sum=vu0Sum;uint32_t q[4],twice[4],diagonal[4];
    for(unsigned i=0;i<4;++i){q[i]=(raw[i]&0x7f800000)?raw[i]:raw[i]&0x80000000;twice[i]=sum(q[i],q[i]);const auto square=product(q[i],q[i]);diagonal[i]=sum(square,square);}
    const auto minus=[](uint32_t b){return b^0x80000000;};uint32_t x[4],y[4],z[4],w[4];for(unsigned i=0;i<4;++i){x[i]=product(q[i],twice[0]);y[i]=product(q[i],twice[1]);z[i]=product(q[i],twice[2]);w[i]=product(q[i],twice[3]);}
    return {sum(sum(0x3f800000,minus(diagonal[1])),minus(diagonal[2])),sum(x[1],w[2]),sum(x[2],minus(w[1])),0,
            sum(y[0],minus(w[2])),sum(sum(0x3f800000,minus(diagonal[0])),minus(diagonal[2])),sum(y[2],x[3]),0,
            sum(z[0],w[1]),sum(z[1],minus(x[3])),sum(sum(0x3f800000,minus(diagonal[0])),minus(diagonal[1])),0};
}
void check(bool v,std::source_location at=std::source_location::current()){
    if(!v)throw std::runtime_error("field distance assertion at "+std::to_string(at.line()));
}
uint32_t square(uint32_t b){
    const unsigned e=(b>>23)&255;if(!e)return 0;
    uint64_t p=uint64_t((b&0x7fffff)|0x800000)*((b&0x7fffff)|0x800000);
    int exponent=int(e)*2-127;unsigned shift=23;
    if(p&(1ull<<47)){++exponent;++shift;}
    if(exponent<=0)return 0;check(exponent<255);
    return uint32_t(exponent<<23)|uint32_t((p>>shift)&0x7fffff);
}
uint32_t sum(uint32_t a,uint32_t b){
    if(a<b)std::swap(a,b);if(!b)return a;unsigned e=a>>23;const unsigned distance=e-(b>>23);
    uint32_t m=(a&0x7fffff)|0x800000;if(distance<=23)m+=((b&0x7fffff)|0x800000)>>distance;
    if(m&0x1000000){m>>=1;++e;}return (e<<23)|(m&0x7fffff);
}
uint32_t root(uint32_t bits){
    const double target=std::bit_cast<float>(bits);uint32_t low=0,high=0x7f7fffff;
    while(low<high){const auto mid=low+(high-low+1)/2;const double value=std::bit_cast<float>(mid);
        if(value*value<=target)low=mid;else high=mid-1;}
    return low;
}
uint32_t product(uint32_t a,uint32_t b){
    const auto sign=(a^b)&0x80000000u;const unsigned ea=(a>>23)&255,eb=(b>>23)&255;if(!ea || !eb)return sign;
    uint64_t p=uint64_t((a&0x7fffff)|0x800000)*((b&0x7fffff)|0x800000);int exponent=int(ea)+int(eb)-127;unsigned shift=23;
    if(p&(1ull<<47)){++exponent;++shift;}if(exponent<=0)return sign;check(exponent<255);
    return sign|uint32_t(exponent<<23)|uint32_t((p>>shift)&0x7fffff);
}
uint32_t signedSum(uint32_t a,uint32_t b){
    if((a&0x7fffffff)<(b&0x7fffffff))std::swap(a,b);
    if(!(b&0x7fffffff))return (a&0x7fffffff)?a:(a&b&0x80000000u);
    int exponent=int((a>>23)&255);const auto distance=unsigned(exponent)-((b>>23)&255);
    const int64_t large=(a&0x7fffff)|0x800000,small=distance<=23?(((b&0x7fffff)|0x800000)>>distance):0;
    int64_t value=(a>>31?-large:large)+(b>>31?-small:small);if(!value)return 0;
    const auto sign=value<0?0x80000000u:0u;uint64_t magnitude=value<0?-value:value;
    while(magnitude>=0x1000000){magnitude>>=1;++exponent;}
    while(magnitude<0x800000){magnitude<<=1;--exponent;}
    if(exponent<=0)return sign;check(exponent<255);return sign|uint32_t(exponent<<23)|uint32_t(magnitude&0x7fffff);
}
uint32_t reciprocal(uint32_t bits){
    if(!(bits&0x7fffffff))return 0x7f7fffff;
    const double denominator=std::bit_cast<float>(bits);uint32_t lo=0,hi=0x7f7fffff;
    while(lo<hi){const auto mid=lo+(hi-lo+1)/2;const double value=std::bit_cast<float>(mid);
        if(value*denominator<=1.)lo=mid;else hi=mid-1;}return lo;
}
uint32_t roundedPositive(uint64_t significant,unsigned shift,unsigned exponent,unsigned mode){
    const auto mask=(1ull<<shift)-1,remainder=significant&mask;auto mantissa=significant>>shift;
    if((mode==_MM_ROUND_UP && remainder) || (mode==_MM_ROUND_NEAREST &&
       (remainder>(1ull<<(shift-1)) || (remainder==(1ull<<(shift-1)) && (mantissa&1)))))++mantissa;
    if(mantissa>=0x1000000){mantissa>>=1;++exponent;}
    return (exponent<<23)|uint32_t(mantissa&0x7fffff);
}
uint32_t roundedSquare(uint32_t bits,unsigned mode){
    const auto e=(bits>>23)&255;if(!e)return 0;
    const uint64_t mantissa=(bits&0x7fffff)|0x800000,p=mantissa*mantissa;
    int exponent=int(e)*2-127;unsigned shift=23;if(p&(1ull<<47)){++exponent;++shift;}
    if(exponent<=0)return 0;check(exponent<255);return roundedPositive(p,shift,unsigned(exponent),mode);
}
uint32_t roundedSum(uint32_t a,uint32_t b,unsigned mode){
    if(a<b)std::swap(a,b);if(!b)return a;unsigned e=a>>23;const auto delta=e-(b>>23);
    uint64_t smaller=uint64_t((b&0x7fffff)|0x800000)<<32;
    if(delta>=64)smaller=1;else if(delta){const auto lost=smaller&((1ull<<delta)-1);smaller=(smaller>>delta)|(lost!=0);}
    auto total=(uint64_t((a&0x7fffff)|0x800000)<<32)+smaller;
    if(total&(1ull<<56)){total=(total>>1)|(total&1);++e;}
    return roundedPositive(total,32,e,mode);
}
double positiveValue(uint32_t bits){const auto e=bits>>23,m=(bits&0x7fffff)|(e?0x800000:0);return std::ldexp(double(m),e?int(e)-150:-149);}
uint32_t roundedRoot(uint32_t bits,unsigned mode){
    const auto target=positiveValue(bits);uint32_t lo=0,hi=0x7f7fffff;
    while(lo<hi){const auto mid=lo+(hi-lo+1)/2;const auto v=positiveValue(mid);if(v*v<=target)lo=mid;else hi=mid-1;}
    const auto v=positiveValue(lo);if(v*v==target)return lo;
    if(mode==_MM_ROUND_UP)return lo+1;
    if(mode==_MM_ROUND_NEAREST){const auto mid=(v+positiveValue(lo+1))*0.5;
        if(target>mid*mid || (target==mid*mid && (lo&1)))return lo+1;}
    return lo;
}
struct Host:bozo::ee::Host,bozo::port::Host {
    bool reference=false;std::vector<uint32_t> nodes;unsigned colourCalls=0;
    uint32_t actionObject=0,actionFlags=0;unsigned actionCalls=0,actionFinals=0;
    std::vector<uint32_t> movementNodes;unsigned movementCalls=0,movementPublications=0;
    std::array<float,4> movementDelta{};
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("library");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(reference && (pc==0x39f17c || pc==0x39f1e0 || pc==0x39f268 || pc==0x39f2cc)){
            const auto object=c->r[16].uw[0];const auto b=object+(pc>=0x39f268?0x250:0x210),target=object+(pc==0x39f1e0 || pc==0x39f2cc?0xd0:0x90);
            uint32_t m[4][4],rows[4][4],out[4][4];std::memcpy(m,rt->memory.hostPointer(b,64),64);std::memcpy(rows,rt->memory.hostPointer(target,64),64);
            for(unsigned k=0;k<4;++k)for(unsigned i=0;i<4;++i)
                out[k][i]=vu0Sum(vu0Sum(vu0Sum(vu0Product(m[0][i],rows[k][0]),vu0Product(m[1][i],rows[k][1])),vu0Product(m[2][i],rows[k][2])),vu0Product(m[3][i],rows[k][3]));
            std::memcpy(rt->memory.hostPointer(target,64),out,64);return true;
        }
        if(reference && pc==0x39f368){uint32_t q[4];std::memcpy(q,rt->memory.hostPointer(c->r[29].uw[0]+0x20,16),16);const auto rows=movementRotationOracle(q);
            std::memcpy(rt->memory.hostPointer(c->r[3].uw[0],48),rows.data(),48);c->r[9].ud[0]=0;return true;}
        if(reference && pc==0x38015c){uint32_t q[4];std::memcpy(q,rt->memory.hostPointer(c->r[29].uw[0]+0x290,16),16);const auto rows=movementRotationOracle(q);
            std::memcpy(rt->memory.hostPointer(c->r[2].uw[0],48),rows.data(),48);c->r[9].ud[0]=0;return true;}
        if(pc>=0x377d8c && pc<=0x377dac){
            if(!reference)return vp2s::nativeMathInstruction(r,c,rt,pc);
            if(pc==0x377dac){const auto saved=_mm_getcsr(),mode=saved&_MM_ROUND_MASK;uint32_t out[4];std::memcpy(out,rt->memory.hostPointer(c->r[7].uw[0],16),16);
                for(unsigned i=0;i<3;++i)out[i]=roundedSquare(out[i],mode);
                out[0]=roundedRoot(roundedSum(roundedSum(out[0],out[1],mode),out[2],mode),mode);
                std::memcpy(rt->memory.hostPointer(c->r[6].uw[0],16),out,16);_mm_setcsr(saved);
            }return true;
        }
        if(pc>=0x446724 && pc<=0x446744){
            if(pc==0x446744)++movementPublications;
            if(!reference)return vp2s::nativeMathInstruction(r,c,rt,pc);
            if(pc==0x446744){const auto saved=_mm_getcsr(),mode=saved&_MM_ROUND_MASK;uint32_t out[4];
                std::memcpy(out,rt->memory.hostPointer(c->r[29].uw[0]+0xb0,16),16);
                for(unsigned i=0;i<3;++i)out[i]=roundedSquare(out[i],mode);
                out[0]=roundedRoot(roundedSum(roundedSum(out[0],out[1],mode),out[2],mode),mode);
                std::memcpy(rt->memory.hostPointer(c->r[29].uw[0]+0x50,16),out,16);_mm_setcsr(saved);
            }return true;
        }
        if(reference && pc==0x4127ac){const auto saved=_mm_getcsr(),mode=saved&_MM_ROUND_MASK;uint32_t out[4];
            std::memcpy(out,rt->memory.hostPointer(c->r[16].uw[0]+0x220,16),16);
            for(unsigned i=0;i<3;++i)out[i]=roundedSquare(out[i],mode);
            out[0]=roundedRoot(roundedSum(roundedSum(out[0],out[1],mode),out[2],mode),mode);
            std::memcpy(rt->memory.hostPointer(c->r[29].uw[0]+0x20,16),out,16);_mm_setcsr(saved);return true;
        }
        if(reference && pc==0x43d2ac){uint32_t first[4],second[4],out[4];
            std::memcpy(first,rt->memory.hostPointer(c->r[6].uw[0],16),16);std::memcpy(second,rt->memory.hostPointer(c->r[4].uw[0],16),16);
            for(unsigned i=0;i<4;++i)out[i]=product(first[i],second[i]);std::memcpy(rt->memory.hostPointer(c->r[3].uw[0],16),out,16);return true;}
        if(reference && pc==0x37fad8){
            const auto saved=_mm_getcsr();uint32_t corners[8],out[4],squares[3];
            std::memcpy(corners,rt->memory.hostPointer(c->r[4].uw[0],32),32);
            for(unsigned i=0;i<3;++i){out[i]=product(signedSum(corners[i+4],corners[i]),0x3f000000);
                const auto extent=product(signedSum(corners[i+4],corners[i]^0x80000000u),0x3f000000);squares[i]=product(extent,extent);}
            const auto squared=signedSum(signedSum(squares[0],squares[1]),squares[2]);
            out[3]=product(squared,reciprocal(root(squared)));
            std::memcpy(rt->memory.hostPointer(c->r[7].uw[0],16),out,16);_mm_setcsr(saved);return true;
        }
        if(reference && pc==0x410e4c){
            uint32_t matrix[4][4],vector[4],result[4];std::memcpy(matrix,rt->memory.hostPointer(c->r[4].uw[0],64),64);std::memcpy(vector,rt->memory.hostPointer(c->r[5].uw[0],16),16);
            const unsigned order[4][4]={{0,1,2,3},{1,0,2,3},{2,1,0,3},{3,1,2,0}};
            // VU0 mul, then add/madd (product by 1 is exact) as the original 410df0 body.
            for(unsigned row=0;row<4;++row){uint32_t p[4];for(unsigned i=0;i<4;++i)p[i]=vu0Product(matrix[row][i],vector[i]);
                result[row]=vu0Sum(vu0Sum(vu0Sum(p[order[row][0]],p[order[row][1]]),p[order[row][2]]),p[order[row][3]]);}
            std::memcpy(rt->memory.hostPointer(c->r[6].uw[0],16),result,16);return true;
        }
        if(reference && pc==0x379280){
            // The numerical quaternion API is separately qualified. This
            // comparison isolates this caller's CPU builder/output ABI.
            __m128 input;std::memcpy(&input,rt->memory.hostPointer(c->r[19].uw[0]+0x190,16),16);
            const auto result=vp2s::math::quaternionMatrix(input);
            std::memcpy(rt->memory.hostPointer(c->r[2].uw[0],48),result.data(),48);return true;
        }
        if(reference && (pc==0x42d3b0 || pc==0x3d0534)){
            // mod003 42d280 stack length: (sqrt((x2+y2)+1*z2), y2, z2, raw W), ps2f.
            const auto stack=c->r[29].uw[0];std::array<uint32_t,4> v;std::memcpy(v.data(),rt->memory.hostPointer(pc==0x42d3b0?stack+0x100:c->r[20].uw[0]+0x20,16),16);
            for(unsigned i=0;i<3;++i)v[i]=vu0Product(v[i],v[i]);uint32_t f=0;const auto saved=_mm_getcsr();
            const auto q=bozo::ps2f::sqrt(vu0Sum(vu0Sum(v[0],v[1]),vu0Product(0x3f800000,v[2])),f);_mm_setcsr(saved);
            v[0]=vu0Sum(0,q);std::memcpy(rt->memory.hostPointer(stack+(pc==0x42d3b0?0x70:0xa0),16),v.data(),16);return true;
        }
        if(reference && (pc==0x3ef09c || pc==0x3ef0f0 || (pc>=0x3ef154 && pc<=0x3ef160) || pc==0x3ef184 || pc==0x3ef1c8)){
            // mod003 3ef010 segment approach, engine ps2f arithmetic in the original order.
            const auto sub=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::subNearest(a,b,f);};
            const auto load=[&](uint32_t at){std::array<uint32_t,4> v;std::memcpy(v.data(),rt->memory.hostPointer(at,16),16);return v;};
            const auto store=[&](uint32_t at,const std::array<uint32_t,4> &v,unsigned bytes){std::memcpy(rt->memory.hostPointer(at,bytes),v.data(),bytes);};
            const auto stack=c->r[29].uw[0];const uint32_t one=0x3f800000;
            if(pc==0x3ef09c){auto v=load(stack+0x90);for(unsigned i=0;i<3;++i)v[i]=vu0Product(v[i],v[i]);
                uint32_t f=0;const auto saved=_mm_getcsr();const auto q=bozo::ps2f::sqrt(vu0Sum(vu0Sum(v[0],v[1]),vu0Product(one,v[2])),f);_mm_setcsr(saved);
                v[0]=vu0Sum(0,q);store(stack+0x40,v,16);return true;}
            if(pc==0x3ef0f0){auto v=load(stack+0x90);const auto base=load(c->r[7].uw[0]);const auto s=std::bit_cast<uint32_t>(c->f[0]);
                for(unsigned i=0;i<3;++i)v[i]=vu0Sum(vu0Product(base[i],one),vu0Product(v[i],s));store(stack+0x90,v,16);return true;}
            if(pc==0x3ef184 || pc==0x3ef1c8){auto v=load(stack+0x70);const auto w=load(stack+(pc==0x3ef184?0x60:0x50));
                v[0]=vu0Sum(vu0Sum(vu0Product(v[0],w[0]),vu0Product(v[1],w[1])),vu0Product(v[2],w[2]));store(stack+(pc==0x3ef184?0x30:0x20),v,16);return true;}
            const auto owner=c->r[16].uw[0];const auto A=load(owner+0x280),B=load(owner+0x290),P=load(c->r[7].uw[0]);
            std::array<uint32_t,4> d{},e{},o{};uint32_t num,den;
            for(unsigned i=0;i<3;++i){d[i]=sub(B[i],A[i]);e[i]=sub(P[i],A[i]);}
            num=vu0Sum(vu0Sum(vu0Product(e[0],d[0]),vu0Product(e[1],d[1])),vu0Product(e[2],d[2]));
            den=vu0Sum(vu0Sum(vu0Product(d[1],d[1]),vu0Product(d[0],d[0])),vu0Product(d[2],d[2]));
            uint32_t f=0;const auto saved=_mm_getcsr();const auto t=bozo::ps2f::div(num,den,f);_mm_setcsr(saved);
            if(pc==0x3ef154){for(unsigned i=0;i<3;++i)o[i]=vu0Sum(A[i],vu0Product(d[i],t));store(stack+0x80,o,12);}
            if(pc==0x3ef158)store(stack+0x70,d,12);
            if(pc==0x3ef15c)store(stack+0x60,e,12);
            if(pc==0x3ef160){for(unsigned i=0;i<3;++i)o[i]=sub(P[i],B[i]);store(stack+0x50,o,12);}
            return true;
        }
        if(reference && (pc==0x439278 || pc==0x4398a0 || pc==0x439bc8 || pc==0x439bf8 || pc==0x439c68 || pc==0x3cdca4 || pc==0x42c6e8)){
            const unsigned source=pc==0x3cdca4?19:pc==0x439278 || pc==0x4398a0 || pc==0x42c6e8?5:pc==0x439bc8?4:pc==0x439bf8?3:4,destination=pc==0x3cdca4?2:pc==0x439278 || pc==0x4398a0 || pc==0x42c6e8?4:pc==0x439bc8?3:2;
            uint32_t bits[4],flags=0;std::memcpy(bits,rt->memory.hostPointer(c->r[source].uw[0],16),16);
            if(pc==0x439c68){uint32_t other[4];std::memcpy(other,rt->memory.hostPointer(c->r[3].uw[0],16),16);
                bits[0]=vu0Sum(vu0Sum(vu0Product(bits[0],other[0]),vu0Product(bits[1],other[1])),vu0Product(bits[2],other[2]));}
            else{for(unsigned i=0;i<3;++i)bits[i]=vu0Product(bits[i],bits[i]);
                bits[0]=vu0Sum(vu0Sum(bits[0],bits[1]),vu0Product(0x3f800000,bits[2]));
                const auto saved=_mm_getcsr();bits[0]=vu0Sum(0,bozo::ps2f::sqrt(bits[0],flags));_mm_setcsr(saved);}
            std::memcpy(rt->memory.hostPointer(c->r[destination].uw[0],16),bits,16);return true;
        }
        if(reference && (pc==0x380684 || pc==0x380864)){
            // Original VU0 sequence: VMUL.xyz squares, VADDA.x/VMADD.x sum
            // (product by 1 exact), VSQRT, VADD.x 0+Q; Y/Z squares and W kept.
            uint32_t bits[4],flags=0;std::memcpy(bits,rt->memory.hostPointer(c->r[4].uw[0],16),16);
            for(unsigned i=0;i<3;++i)bits[i]=vu0Product(bits[i],bits[i]);
            bits[0]=vu0Sum(vu0Sum(bits[0],bits[1]),vu0Product(0x3f800000,bits[2]));
            // ps2f::sqrt seeds from a host root; keep host FP flags out of the CPU ABI.
            {const auto saved=_mm_getcsr();bits[0]=vu0Sum(0,bozo::ps2f::sqrt(bits[0],flags));_mm_setcsr(saved);}
            std::memcpy(rt->memory.hostPointer(c->r[3].uw[0],16),bits,16);return true;
        }
        if(reference){const auto saved=_mm_getcsr();check(pc==0x3789a0 || pc==0x3c5d78 || pc==0x37b568 || pc==0x382040 || pc==0x37f9b4);uint32_t bits[4];std::memcpy(bits,rt->memory.hostPointer(c->r[pc==0x382040?4:pc==0x37f9b4?10:5].uw[0],16),16);
            const auto x=square(bits[0]),y=square(bits[1]),z=square(bits[2]);
            bits[0]=root(sum(sum(x,y),z));bits[1]=y;bits[2]=z;
            // Searching down to an exact zero traverses subnormal candidates;
            // numerical oracle work must not add host FP flags to the CPU ABI.
            std::memcpy(rt->memory.hostPointer(c->r[pc==0x382040?3:pc==0x37f9b4?9:4].uw[0],16),bits,16);_mm_setcsr(saved);return true;}
        check(vp2s::nativeMathInstruction(r,c,rt,pc));return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        // Bounded software iterator/setter contracts isolate this original
        // caller. They do not qualify the model manager or renderer itself.
        if(pc==0x248e00){std::memcpy(d.runtime().memory.hostPointer(c->r[4].uw[0],16),d.runtime().memory.hostPointer(c->r[5].uw[0],16),16);}
        else if(pc==0x172030){check(std::find(movementNodes.begin(),movementNodes.end(),c->r[4].uw[0])!=movementNodes.end() && c->r[8].uw[0]==0);
            const auto stack=c->r[29].uw[0];check(c->r[5].uw[0]==stack+0xd0 && c->r[6].uw[0]==stack+0xc0 && c->r[7].uw[0]==stack+0xa0);
            std::memcpy(d.runtime().memory.hostPointer(c->r[7].uw[0],16),movementDelta.data(),16);++movementCalls;
        }
        else if(pc==0x3d0b90){check(c->r[4].uw[0]==0x1e0000 && c->r[5].uw[0]==actionFlags && c->r[6].uw[0]==0);
            check(c->f[12]==0 && c->f[13]==0 && c->f[14]==8 && c->f[15]==1 && c->f[16]==25);++actionCalls;}
        else if(pc==0x172020){check(c->r[4].uw[0]==actionObject && c->r[5].uw[0]==255);*d.runtime().memory.hostPointer(actionObject+0x7c9,1)=255;}
        else if(pc==0x395ec0){check(c->r[4].uw[0]==actionObject);++actionFinals;c->r[2].ud[0]=0x4a;}
        else if(pc==0x267280){check(c->r[4].uw[0]==0x1e0000 && c->r[5].uw[0]==0);c->r[2].ud[0]=nodes.empty()?0:nodes.front();}
        else if(pc==0x266950){check(c->r[4].uw[0]==0x1e0000);auto i=std::find(nodes.begin(),nodes.end(),c->r[5].uw[0]);check(i!=nodes.end());c->r[2].ud[0]=++i==nodes.end()?0:*i;}
        else if(pc==0x172000){*d.runtime().memory.hostPointer(c->r[4].uw[0]+0x110,1)=uint8_t(c->r[5].uw[0]);c->r[2].ud[0]=0;++colourCalls;}
        else if(pc==0x172010){check(std::find(nodes.begin(),nodes.end(),c->r[4].uw[0])!=nodes.end());c->r[2].ud[0]=0;++colourCalls;}
        else throw std::runtime_error("call");
        c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host host;bozo::ee::Dispatcher original(host),adapted(host);
    original.setFunctions(distance_original_function_table,distance_original_function_count,distance_original_resume_table,distance_original_resume_count);
    adapted.setFunctions(distance_adapted_function_table,distance_adapted_function_count,distance_adapted_resume_table,distance_adapted_resume_count);
    original.runtime().port=adapted.runtime().port=&host;unsigned cases=0;
    for(auto *d:{&original,&adapted})for(uint32_t pc:{0x267280u,0x266950u,0x172000u,0x172010u,0x172020u,0x3d0b90u,0x395ec0u,0x248e00u,0x172030u,0x2a8760u})d->bindLibrary(pc);
    for(unsigned profile=0;profile<5;++profile)for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<64;++seed){
        const bool movement=profile==1,camera=profile==2,setup=profile==3,enclosing=profile==4;
        auto &a=original.runtime();auto &b=adapted.runtime();std::memset(a.memory.flat(),seed+1,bozo::kFlatBytes);
        const uint32_t object=0x180000,source=0x190000,owner=0x1a0000,position=0x1b0000,flags=0x1c0000;
        const uint32_t stack=seed&1?(enclosing?0x70003d00:0x70003f00):0x1fff000;
        std::memcpy(a.memory.ram()+0x48e048,&owner,4);std::memcpy(a.memory.ram()+owner+0x14,&position,4);std::memcpy(a.memory.ram()+object,&flags,4);
        float point[4]={float(int(seed)-32)/8.f,float(seed%11)*-0.75f,float(seed%7)*1.25f,std::bit_cast<float>(0x7fc01234u)};
        const float other[3]={float(seed%3),float(seed%5),float(seed%7)};
        if(seed%8==0)std::memcpy(point,other,12);
        std::memcpy(a.memory.ram()+object+0x10,point,16);std::memcpy(a.memory.ram()+position+0x250,other,12);
        if(movement){std::memcpy(a.memory.ram()+object+0x30,point,16);std::memcpy(a.memory.ram()+object+0x10,other,12);a.memory.ram()[object+0xbc]=uint8_t(seed*17);}
        if(camera){std::memcpy(a.memory.ram()+object+0x40,point,16);std::memcpy(a.memory.ram()+object+0x250,other,12);}
        if(setup){std::memcpy(a.memory.ram()+object+0xc,&position,4);std::memcpy(a.memory.hostPointer(stack+0x90,12),other,12);
            for(unsigned i=0;i<3;++i)std::memcpy(a.memory.ram()+position+0xdc+i*16,point+i,4);}
        if(enclosing){std::memcpy(a.memory.hostPointer(stack+0x40,16),point,16);const float radius=float(seed%7)/4.f;std::memcpy(a.memory.ram()+source,&radius,4);}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0xaa,sizeof(x.vu0_vf));x.vu0_acc.u[1]=0xabcd;x.vu0_status=0x7654;
        x.r[16].ud[0]=profile?object:source;x.r[17].ud[0]=object;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;auto y=x;
        if(enclosing){x.r[10].ud[0]=stack+0x40;x.r[9].ud[0]=stack+0x60;x.r[11].ud[0]=source;
            x.f[5]=other[0];x.f[3]=other[1];x.f[1]=other[2];x.f[0]=seed%3==0?0.f:seed%3==1?100.f:float(seed);y=x;}
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        const auto entry=enclosing?0x37f95c:setup?0x381fb8:camera?0x37b500:movement?0x3c5d08:0x378908;
        host.reference=true;a.functions.generated(entry)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(entry)(b.memory.flat(),&y,&b);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)));
        check(!std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));++cases;
        if(movement)check((b.memory.ram()[object+0xbc]&1)==(seed%8!=0) && (b.memory.ram()[object+0xbc]&254)==((seed*17)&254));
        if(camera)check(!std::memcmp(b.memory.ram()+object+0x33c,b.memory.hostPointer(stack+0x20,4),4));
        if(setup)check(!std::memcmp(b.memory.ram()+object+0x58,b.memory.hostPointer(stack+0x50,4),4));
    }
    unsigned automaticDistances=0,knownCameraDistances=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=original.runtime(),&b=adapted.runtime();std::memset(a.memory.flat(),0x51,bozo::kFlatBytes);
        constexpr uint32_t object=0x180000,manager=0x190000,camera=0x1a0000,flags=0x1b0000;
        const uint32_t top=seed&4?0x70003f00:0x1fff000,frame=top-0x30;
        std::memcpy(a.memory.ram()+object,&flags,4);std::memcpy(a.memory.ram()+0x48e048,&manager,4);std::memcpy(a.memory.ram()+manager+0x14,&camera,4);
        const uint32_t opaque[]={0x7fc01234,0x7fa01234,0x7f800000,1};
        float point[4]={float(int(seed%17)-8)/8,float(seed%11)/4,-float(seed%7)/2,std::bit_cast<float>(opaque[(seed/8)%4])};
        float other[3]={float(seed%3)/2,-float(seed%5)/4,float(seed%7)/8};
        if(seed%8==0){point[0]=3;point[1]=4;point[2]=0;other[0]=other[1]=other[2]=0;}
        std::memcpy(a.memory.ram()+object+0x10,point,16);std::memcpy(a.memory.ram()+camera+0x250,other,12);
        const float initial=8;std::memcpy(a.memory.ram()+object+0x1a8,&initial,4);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);R5900Context x{};
        for(unsigned i=1;i<32;++i){x.r[i].ud[0]=seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        x.r[4].ud[0]=object;x.r[29].ud[0]=top;x.r[31].ud[0]=0x170000;x.f[12]=seed&1?1+float(seed%7)/4:0;x.f[13]=seed&2?1.5f:0;
        std::memset(x.vu0_vf,0x54,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0x65,sizeof(x.vu0_acc));x.vu0_status=0x1234;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&4?0x8040:0);_mm_setcsr(control);
        host.reference=true;a.functions.generated(0x377d20)(a.memory.flat(),&x,&a);const auto fp=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(0x377d20)(b.memory.flat(),&y,&b);check(_mm_getcsr()==fp);_mm_setcsr(saved);x.cycleLimit=y.cycleLimit;
        check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        check(b.memory.ram()[flags+0x446]==(seed&1?0:1));
        if(seed%8==0){float value;std::memcpy(&value,b.memory.ram()+object+0x1ac,4);check(value==5);std::memcpy(&value,b.memory.ram()+object+0x1a8,4);check(value==5);++knownCameraDistances;}
        if(!(seed&1)){uint32_t tag;std::memcpy(&tag,b.memory.hostPointer(frame+0xc,4),4);check(tag==opaque[(seed/8)%4]);}
        ++automaticDistances;
    }
    {auto &rt=adapted.runtime();constexpr uint32_t frame=0x1fff000;const float input[4]={3,4,0,std::bit_cast<float>(0x7fa01234u)};
        for(unsigned which=0;which<7;++which){std::memcpy(rt.memory.hostPointer(frame+0x20,16),input,16);R5900Context c{};c.pc=0x377d20;c.r[29].uw[0]=frame;c.r[7].uw[0]=frame+0x20;c.r[6].uw[0]=frame;
            if(which<3){const uint32_t bad[]={1,0x7fa01234,0x7f7fffff};std::memcpy(rt.memory.hostPointer(frame+0x20,4),bad+which,4);}
            else if(which==3)c.pc=0x377dd0;else if(which==4)c.r[7].uw[0]+=16;else if(which==5)c.r[6].uw[0]+=16;else c.r[29].uw[0]+=4;
            const auto before=c;const auto fp=_mm_getcsr();std::vector<uint8_t> ram(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
            try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x377dac);}catch(const std::runtime_error &){rejected=true;}
            check(rejected && _mm_getcsr()==fp && !std::memcmp(&c,&before,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));
        }
    }
    unsigned animationMoves=0,animationNodeCalls=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=original.runtime(),&b=adapted.runtime();std::memset(a.memory.flat(),0x61,bozo::kFlatBytes);
        constexpr uint32_t object=0x180000,table=0x1d0000,pose=0x1c0000,head=0x358da0,callback=0x172030;
        const uint32_t top=seed&1?0x70003f00:0x1fff000,frame=top-0xe0,empty=0,mask=seed%3?1:0;
        std::memcpy(a.memory.ram()+object+0x114,&empty,4);std::memcpy(a.memory.ram()+object+0x118,&pose,4);std::memcpy(a.memory.ram()+object+0x11c,&mask,4);
        float params[3]={float(seed%16)*.75f,float(seed%5)*.25f,seed%5?20.f:0.f},step=seed%3?.5f:1.25f;
        std::memcpy(a.memory.ram()+object+0x120,params,12);std::memcpy(a.memory.ram()+0x35ded8,&step,4);
        a.memory.ram()[object+0x12c]=uint8_t(seed%3);a.memory.ram()[object+0x12d]=uint8_t(int(seed%4)-1);
        a.memory.ram()[object+0x12e]=uint8_t(seed%3);a.memory.ram()[object+0x12f]=uint8_t(seed%8);
        const std::array<float,4> position{3,-4,5,1};std::memcpy(a.memory.ram()+pose,position.data(),16);
        host.movementNodes.clear();unsigned expectedCalls=0;
        for(unsigned i=0;i<seed%4;++i)host.movementNodes.push_back(0x200000+i*0x1000);
        const auto first=host.movementNodes.empty()?head:host.movementNodes.front();std::memcpy(a.memory.ram()+head+8,&first,4);
        std::memcpy(a.memory.ram()+table+0x8c,&callback,4);
        for(unsigned i=0;i<host.movementNodes.size();++i){const auto node=host.movementNodes[i],next=i+1<host.movementNodes.size()?host.movementNodes[i+1]:head,flags=i&1;
            std::memcpy(a.memory.ram()+node,&table,4);std::memcpy(a.memory.ram()+node+8,&next,4);std::memcpy(a.memory.ram()+node+0x210,&flags,4);
            expectedCalls+=!(mask&flags);
        }
        const float lengths[]={0,.25f,.5f,.6f,3,40,200,.0001f};const auto length=lengths[(seed/4)%8];
        host.movementDelta={seed&4?-length:length,length*.5f,-length*.25f,std::bit_cast<float>(0x7fc01234u)};
        if(seed%17==0)host.movementDelta={std::bit_cast<float>(0x00800000u),0,0,0};
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);R5900Context x{};
        for(unsigned i=1;i<32;++i){x.r[i].ud[0]=seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0x54,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0x65,sizeof(x.vu0_acc));x.vu0_status=0x1234;
        x.r[4].ud[0]=object;x.r[29].ud[0]=top;x.r[31].ud[0]=0x170000;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);_mm_setcsr(control);
        host.movementCalls=host.movementPublications=0;host.reference=true;a.functions.generated(0x446620)(a.memory.flat(),&x,&a);
        const auto expectedFp=_mm_getcsr();check(host.movementCalls==expectedCalls && host.movementPublications==1);_mm_setcsr(control);
        host.movementCalls=host.movementPublications=0;host.reference=false;
        try{b.functions.generated(0x446620)(b.memory.flat(),&y,&b);}catch(const std::exception &e){std::fprintf(stderr,"animation movement seed=%u mode=%x: %s\n",seed,mode,e.what());throw;}
        check(_mm_getcsr()==expectedFp && host.movementCalls==expectedCalls && host.movementPublications==1);_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        uint32_t result[4];std::memcpy(result,b.memory.hostPointer(frame+0x50,16),16);check(result[3]==0);
        if(!(seed&2)){float rate;std::memcpy(&rate,b.memory.ram()+object+0x124,4);check(rate>=0 && rate<=2 && (b.memory.ram()[object+0x12f]&2));}
        animationNodeCalls+=expectedCalls;++animationMoves;
    }
    {auto &rt=adapted.runtime();R5900Context c{};c.pc=0x44665c;c.r[18].uw[0]=0x180000;c.r[29].uw[0]=0x1fff000;c.r[4].uw[0]=0x1fff0b0;c.r[3].uw[0]=0x1fff050;
        const uint32_t input[4]={0x3f800001,0xbf000001,0x00800000,0x7fc05678};std::memcpy(rt.memory.hostPointer(c.r[4].uw[0],16),input,16);
        auto reject=[&]{const auto before=c;const auto fp=_mm_getcsr();std::vector<uint8_t> ram(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool failed=false;
            try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x446744);}catch(const std::runtime_error &){failed=true;}
            check(failed && _mm_getcsr()==fp && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));};
        c.pc=0x170000;reject();c.pc=0x44665c;c.r[3].uw[0]+=16;reject();c.r[3].uw[0]-=16;c.r[4].uw[0]+=16;reject();c.r[4].uw[0]-=16;
        c.r[18].uw[0]=c.r[29].uw[0];reject();c.r[18].uw[0]=0x180000;
        for(uint32_t bad:{1u,0x7fc01234u,0x7f7fffffu}){std::memcpy(rt.memory.hostPointer(c.r[4].uw[0],4),&bad,4);reject();}
        std::memcpy(rt.memory.hostPointer(c.r[4].uw[0],16),input,16);check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x446744));
        uint32_t opaque;std::memcpy(&opaque,rt.memory.hostPointer(c.r[3].uw[0]+12,4),4);check(opaque==input[3]);
    }
    unsigned actions=0,activeActions=0,zeroActions=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=original.runtime();auto &b=adapted.runtime();std::memset(a.memory.flat(),seed+1,bozo::kFlatBytes);
        constexpr uint32_t object=0x180000,table=0x1d0000,model=0x1e0000,child=0x1f0000,setter=0x172020;
        const uint32_t top=seed&1?0x70003f00:0x1fff000,flag=0x1234;
        const int8_t state=int8_t(seed%40);a.memory.ram()[object+0x7c6]=uint8_t(state);
        const bool active=!(state>=15 && state<=35) && state!=10;
        uint32_t point[4]={0,0,0,0x7fc01234};
        if(seed%5){for(unsigned i=0;i<3;++i)point[i]=((seed+i)&1?0x80000000u:0u)|(0x3a000000u+((seed*77771+i*112345)&0x7fffffu));}
        if(seed%11==0 && seed%5)point[0]=0x3f800001; // Non-exact square/sum/root rounding.
        if(seed%17==0 && seed%5){point[0]=0x00800000;point[1]=point[2]=0;} // Squared terms flush to zero.
        const bool zero=seed%5==0 || seed%17==0;
        const uint32_t selectedChild=seed&2?child:0;
        for(auto pair:{std::pair{object,table},std::pair{table+0xac,setter},std::pair{object+0x488,model},std::pair{model+0x7c,selectedChild},std::pair{object+0x100,flag}})
            std::memcpy(a.memory.ram()+pair.first,&pair.second,4);
        std::memcpy(a.memory.ram()+object+0x220,point,16);std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0x8c,sizeof(x.vu0_vf));x.vu0_acc.u[2]=0x9988;x.vu0_status=0x7654;
        x.r[4].ud[0]=object;x.r[29].ud[0]=top;x.r[31].ud[0]=0x170000;auto y=x;
        host.actionObject=object;host.actionFlags=flag|(zero?0x310000:0x330000);
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);_mm_setcsr(control);
        host.actionCalls=host.actionFinals=0;host.reference=true;a.functions.generated(0x4126a0)(a.memory.flat(),&x,&a);
        const auto expectedFp=_mm_getcsr();check(host.actionCalls==unsigned(active) && host.actionFinals==1);_mm_setcsr(control);
        host.actionCalls=host.actionFinals=0;host.reference=false;
        try{b.functions.generated(0x4126a0)(b.memory.flat(),&y,&b);}catch(const std::exception &e){std::fprintf(stderr,"action seed=%u mode=%x: %s\n",seed,mode,e.what());throw;}
        check(_mm_getcsr()==expectedFp && host.actionCalls==unsigned(active) && host.actionFinals==1);_mm_setcsr(saved);
        check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        if(active){uint32_t output[4];std::memcpy(output,b.memory.hostPointer(top-0x10,16),16);check(output[3]==point[3]);
            check((output[0]==0)==zero);check(b.memory.ram()[model+0x90]==1 && b.memory.ram()[model+0x88]==0);
            if(selectedChild)check(b.memory.ram()[child+0x60]==1 && b.memory.ram()[child+0x48]==0);
            ++activeActions;if(zero)++zeroActions;}
        ++actions;
    }
    {auto &rt=adapted.runtime();R5900Context c{};c.pc=0x4126a0;c.r[16].uw[0]=0x180000;c.r[3].uw[0]=0x180220;c.r[29].uw[0]=0x1fff000;c.r[2].uw[0]=0x1fff020;
        const uint32_t point[4]={0,0,0,0x7fc01234};std::memcpy(rt.memory.ram()+0x180220,point,16);
        auto reject=[&]{const auto before=c;std::vector<uint8_t> ram(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x4127ac);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));};
        c.pc=0x170000;reject();c.pc=0x4126a0;c.r[3].uw[0]+=16;reject();c.r[3].uw[0]-=16;c.r[2].uw[0]+=16;reject();c.r[2].uw[0]-=16;
        for(uint32_t invalid:{1u,0x7fc05678u,0x7f7fffffu}){std::memcpy(rt.memory.ram()+0x180220,&invalid,4);reject();}
        c.r[29].uw[0]=0x180200;c.r[2].uw[0]=0x180220;reject();
    }
    unsigned rotations=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<64;++seed){
        auto &a=original.runtime();auto &b=adapted.runtime();const uint32_t object=0x180000,stack=seed&1?0x70003d00:0x1fff000;
        std::memset(a.memory.flat(),seed+1,bozo::kFlatBytes);
        float q[4]={float(int(seed%9)-4)/8.f,float(int(seed%11)-5)/8.f,float(int(seed%7)-3)/8.f,float(seed%13)/8.f};
        if(!seed){q[0]=-0.f;q[1]=q[2]=0.f;q[3]=1.f;}
        std::memcpy(a.memory.hostPointer(stack+0x160,16),q,16);std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0xcc,sizeof(x.vu0_vf));x.vu0_acc.u[0]=0x9876;x.vu0_status=0x5432;
        x.r[19].ud[0]=object;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        host.reference=true;a.functions.generated(0x379228)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(0x379228)(b.memory.flat(),&y,&b);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)));
        check(!std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        float row[4];std::memcpy(row,b.memory.ram()+object+0x1f0,16);check(row[0]==0 && row[1]==0 && row[2]==0 && row[3]==1);++rotations;
    }
    unsigned movementRotations=0,knownMovementRotations=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=original.runtime(),&b=adapted.runtime();std::memset(a.memory.flat(),0x51,bozo::kFlatBytes);const uint32_t aliases[]={0,0x20000000,0x30000000,0x80000000,0xa0000000};
        const auto stack=seed&1?0x70002d90u:0x1ffd000u|aliases[(seed/8)%5];uint32_t q[4];for(unsigned i=0;i<4;++i)q[i]=((seed*7919+i*3121)&0x807fffff)|((117+(seed+i)%10)<<23);
        const bool known=seed%8==0;if(known){q[0]=q[1]=q[2]=0;q[3]=0x3f800000;}
        if(seed%8==1){q[0]=1;q[1]=0x80000001;q[2]=0x00800000;q[3]=0x3f800000;}
        std::memcpy(a.memory.hostPointer(stack+0x290,16),q,16);const std::array<float,4> point{1,2,3,1};std::memcpy(a.memory.hostPointer(stack+0x2e0,16),point.data(),16);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x1234000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        std::memset(x.vu0_vf,0xa5,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0x5a,sizeof(x.vu0_acc));x.vu0_status=0x12345678;x.vu0_q_bits=0x87654321;x.vi[0]=0x1234;
        x.r[29].ud[0]=stack;x.r[31].ud[0]=0x380118;auto y=x;const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&8?0x8040:0);_mm_setcsr(control);
        host.reference=true;a.functions.generated(0x380118)(a.memory.flat(),&x,&a);const auto fp=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(0x380118)(b.memory.flat(),&y,&b);check(_mm_getcsr()==fp);_mm_setcsr(saved);x.cycleLimit=y.cycleLimit;
        if(x.pc!=0x380118 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"movement rotation mode%x seed%u PCs%x/%x\n",mode,seed,x.pc,y.pc);
            for(unsigned i=0;i<sizeof(x);++i)if(reinterpret_cast<const uint8_t *>(&x)[i]!=reinterpret_cast<const uint8_t *>(&y)[i]){std::fprintf(stderr,"CPU byte%x old%02x native%02x\n",i,reinterpret_cast<const uint8_t *>(&x)[i],reinterpret_cast<const uint8_t *>(&y)[i]);break;}
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        const auto rows=movementRotationOracle(q);check(!std::memcmp(rows.data(),b.memory.hostPointer(stack+0x2a0,48),48));
        if(seed%8==1){uint32_t outputY;std::memcpy(&outputY,b.memory.hostPointer(stack+0x2e4,4),4);check(outputY==0x40000000u);} // 2 plus a tiny normal term rounds to nearest 2
        const std::array<float,4> last{0,0,0,1};check(!std::memcmp(last.data(),b.memory.hostPointer(stack+0x2d0,16),16));if(known){check(!std::memcmp(point.data(),b.memory.hostPointer(stack+0x2e0,16),16));++knownMovementRotations;}++movementRotations;
    }
    {auto &rt=adapted.runtime();for(unsigned which=0;which<8;++which){constexpr uint32_t stack=0x1ffd000;const std::array<float,4> identity{0,0,0,1};std::memcpy(rt.memory.hostPointer(stack+0x290,16),identity.data(),16);
        R5900Context c{};c.pc=0x380118;c.r[31].uw[0]=0x380118;c.r[29].uw[0]=stack;c.r[2].uw[0]=stack+0x2a0;uint32_t pc=0x38015c;
        if(which==0)c.pc=0;else if(which==1)c.r[31].uw[0]=0;else if(which==2)c.r[2].uw[0]+=16;else if(which==3){pc=0x38011c;c.r[2].uw[0]=stack+0x2a0;}else if(which==4){pc=0x380124;c.r[2].uw[0]=stack+0x2a0;}
        else {const uint32_t bad=which==5?0x7fa01234:which==6?0x7f800000:0x71800000;std::memcpy(rt.memory.hostPointer(stack+0x290,4),&bad,4);}
        const auto before=c;const auto fp=_mm_getcsr();const auto ram=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc);}catch(const std::runtime_error &){rejected=true;}check(rejected && _mm_getcsr()==fp && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));
    }}
    std::printf("%u bounded movement rotation consumers,%u identity cases, integer quaternion/row-product oracle and8 readonly rejections passed\n",movementRotations,knownMovementRotations);
    unsigned vectorBlocks=0;
    struct VectorBlock{uint32_t first;unsigned sourceReg;bool stack;uint32_t sourceOffset;unsigned destinationReg;uint32_t destinationOffset;bool dot;};
    const VectorBlock vectorCases[]={{0x439258,5,true,0x100,4,0x90,false},{0x439880,5,true,0x100,4,0x80,false},
                                     {0x439ba8,4,false,0x1f0,3,0x70,false},{0x439bd8,3,false,0x230,2,0x60,false},{0x439c54,4,false,0x1f0,2,0x50,true},{0x3cdc84,19,false,0x1f0,2,0xa0,false},{0x42c6c8,5,true,0x60,4,0x30,false}};
    for(const auto &block:vectorCases)for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<64;++seed){
        auto &a=original.runtime(),&b=adapted.runtime();std::memset(a.memory.flat(),0x71+seed,bozo::kFlatBytes);const uint32_t aliases[]={0,0x20000000,0x30000000,0x80000000,0xa0000000};
        const uint32_t owner=0x1b4000|aliases[(seed/4)%5],stack=seed&1?0x70002e00u:0x1ffd400u|aliases[(seed/8)%5];
        uint32_t state=seed*2246822519u+block.first+mode;const auto next=[&](){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((112+(state>>4)%30)<<23);};
        const uint32_t source=(block.stack?stack:owner)+block.sourceOffset;
        for(uint32_t at:{source,owner+0x1f0,owner+0x230})for(unsigned i=0;i<4;++i){uint32_t v=next();if(seed%16==2 && i<3)v=0;if(seed%16==3 && i==1)v=1;std::memcpy(a.memory.hostPointer(at+4*i,4),&v,4);}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x2468000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/4;}
        std::memset(x.vu0_vf,0x3e,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xe3,sizeof(x.vu0_acc));x.vu0_status=0x1111;x.vu0_q_bits=0x3f800005;x.vi[0]=0x1111;
        x.r[17].ud[0]=owner;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;x.r[block.sourceReg].ud[0]=source;x.r[block.destinationReg].ud[0]=stack+block.destinationOffset;
        if(block.dot)x.r[3].ud[0]=owner+0x230;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&8?0x8040:0);_mm_setcsr(control);
        host.reference=true;a.functions.generated(block.first)(a.memory.flat(),&x,&a);const auto fp=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(block.first)(b.memory.flat(),&y,&b);const auto blockFp=_mm_getcsr();_mm_setcsr(saved);
        if(blockFp!=fp)std::fprintf(stderr,"vector block %x mode%x seed%u mxcsr old%08x native%08x\n",block.first,mode,seed,fp,blockFp);check(blockFp==fp);x.cycleLimit=y.cycleLimit;
        if(x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"vector block %x mode%x seed%u PCs%x/%x\n",block.first,mode,seed,x.pc,y.pc);
            for(unsigned i=0;i<sizeof(x);++i)if(reinterpret_cast<const uint8_t *>(&x)[i]!=reinterpret_cast<const uint8_t *>(&y)[i]){std::fprintf(stderr,"CPU byte%x old%02x native%02x\n",i,reinterpret_cast<const uint8_t *>(&x)[i],reinterpret_cast<const uint8_t *>(&y)[i]);break;}
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        ++vectorBlocks;
    }
    std::printf("%u mod003 439180 vector-length/dot blocks match the engine VU0 oracle\n",vectorBlocks);
    unsigned approachBlocks=0;
    const uint32_t approachEntries[5]={0x3ef07c,0x3ef0c8,0x3ef100,0x3ef170,0x3ef1b4};
    for(unsigned block=0;block<5;++block)for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<64;++seed){
        auto &a=original.runtime(),&b=adapted.runtime();std::memset(a.memory.flat(),0x35+seed,bozo::kFlatBytes);
        const uint32_t owner=0x1b4000,camera=0x1b6000,stack=seed&1?0x70002e00u:0x1ffd400u|(seed&2?0x20000000u:0u);
        uint32_t state=seed*2654435761u+block*97+mode;const auto next=[&](){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((116+(state>>4)%16)<<23);};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        for(uint32_t at:{owner+0x280u,owner+0x290u,camera+0x40u,camera+0x250u})for(unsigned i=0;i<4;++i)put(at+4*i,next());
        for(unsigned i=0;i<0x80/4;++i)put(stack+0x20+4*i,next());
        if(seed%16==5)for(unsigned i=0;i<4;++i)put(owner+0x290+4*i,i<3?0:next());     // degenerate segment: 0/0
        if(seed%16==6)for(unsigned i=0;i<3;++i){uint32_t v;std::memcpy(&v,a.memory.hostPointer(owner+0x280+4*i,4),4);put(owner+0x290+4*i,v);} // zero length, nonzero bits
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x3579000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/4;}
        std::memset(x.vu0_vf,0x4f,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xf4,sizeof(x.vu0_acc));x.vu0_status=0x2222;x.vu0_q_bits=0x3f800007;x.vi[0]=0x2222;
        x.r[16].ud[0]=owner;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;x.r[7].ud[0]=seed&4?camera+0x40:camera+0x250;
        if(block==0){x.r[4].ud[0]=stack+0x90;x.r[3].ud[0]=stack+0x40;}
        if(block==1){x.r[7].ud[0]=camera+0x40;x.f[0]=300.f;x.f[1]=std::bit_cast<float>(next()&0x7fffffff)+300.f;}
        if(block==2){x.r[3].ud[0]=owner+0x280;x.r[2].ud[0]=owner+0x290;}
        if(block==3){x.r[4].ud[0]=stack+0x70;x.r[3].ud[0]=stack+0x60;x.r[2].ud[0]=stack+0x30;}
        if(block==4){x.r[4].ud[0]=stack+0x70;x.r[3].ud[0]=stack+0x50;x.r[2].ud[0]=stack+0x20;}
        auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&8?0x8040:0);_mm_setcsr(control);
        host.reference=true;a.functions.generated(approachEntries[block])(a.memory.flat(),&x,&a);const auto fp=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(approachEntries[block])(b.memory.flat(),&y,&b);const auto blockFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(blockFp!=fp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"approach block%u mode%x seed%u PCs%x/%x fp%08x/%08x\n",block,mode,seed,x.pc,y.pc,fp,blockFp);
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        ++approachBlocks;
    }
    std::printf("%u mod003 3ef010 segment approach blocks match the engine VU0 oracle\n",approachBlocks);
    unsigned lengthBlocks=0;
    for(unsigned site=0;site<2;++site)for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<64;++seed){
        auto &a=original.runtime(),&b=adapted.runtime();std::memset(a.memory.flat(),0x47+seed,bozo::kFlatBytes);const uint32_t record=0x1b8000;
        const uint32_t stack=seed&1?0x70002e00u:0x1ffd400u|(seed&2?0x20000000u:0u);
        uint32_t state=seed*40503u+mode;const auto next=[&](){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((110+(state>>4)%36)<<23);};
        const uint32_t source=site?record+0x20:stack+0x100,target=stack+(site?0xa0:0x70);
        for(unsigned i=0;i<4;++i){uint32_t v=next();if(seed%16==4 && i<3)v=0;std::memcpy(a.memory.hostPointer(source+4*i,4),&v,4);v=next();std::memcpy(a.memory.hostPointer(target+4*i,4),&v,4);}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x4680000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/4;}
        std::memset(x.vu0_vf,0x5a,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xa5,sizeof(x.vu0_acc));x.vu0_status=0x3333;x.vu0_q_bits=0x3f800009;x.vi[0]=0x3333;
        x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;x.r[20].ud[0]=record;
        if(site){x.r[4].ud[0]=source;x.r[3].ud[0]=target;}else{x.r[3].ud[0]=source;x.r[2].ud[0]=target;}auto y=x;const uint32_t entry=site?0x3d0514:0x42d390;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&8?0x8040:0);_mm_setcsr(control);
        host.reference=true;a.functions.generated(entry)(a.memory.flat(),&x,&a);const auto fp=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(entry)(b.memory.flat(),&y,&b);const auto blockFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(blockFp!=fp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"stack length mode%x seed%u PCs%x/%x fp%08x/%08x\n",mode,seed,x.pc,y.pc,fp,blockFp);check(false);}
        ++lengthBlocks;
    }
    std::printf("%u mod003 42d280/3cfde0 vector lengths match the engine VU0 oracle\n",lengthBlocks);
    unsigned compositions=0,composedProducts=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=original.runtime(),&b=adapted.runtime();std::memset(a.memory.flat(),0x5d+seed,bozo::kFlatBytes);const uint32_t aliases[]={0,0x20000000,0x30000000,0x80000000,0xa0000000};
        const uint32_t object=0x1a0000|aliases[(seed/4)%5],stack=seed&1?0x70002d00u:0x1ffd200u|aliases[(seed/8)%5];
        uint32_t state=seed*69069u+mode;const auto next=[&](int lo,int span){state=state*1664525u+1013904223u;return ((state>>8)&0x807fffffu)|uint32_t((lo+int((state>>4)%unsigned(span)))<<23);};
        for(uint32_t at:{0x90u,0xd0u,0x210u,0x250u})for(unsigned i=0;i<16;++i){uint32_t v=next(118,14);if(seed%16==3 && i%5)v=i%2?0x80000000u:0;std::memcpy(a.memory.hostPointer(object+at+4*i,4),&v,4);}
        const uint8_t flags=uint8_t((0x5d+seed)&~3u)|uint8_t(seed%4),state51=seed%16==7?2:1;std::memcpy(a.memory.hostPointer(object+0x290,1),&flags,1);std::memcpy(a.memory.hostPointer(object+0x51,1),&state51,1);
        const uint64_t ret=0x170000;std::memcpy(a.memory.hostPointer(stack+0x20,8),&ret,8);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x7531000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/32;}
        std::memset(x.vu0_vf,0x4e,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xe4,sizeof(x.vu0_acc));x.vu0_status=0x0f0f;x.vu0_q_bits=0x3f000001;x.vi[0]=0x0f0f;
        x.r[16].ud[0]=object;x.r[17].ud[0]=0x100;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x39f0ec;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&8?0x8040:0);_mm_setcsr(control);
        host.reference=true;a.functions.generated(0x39f0ec)(a.memory.flat(),&x,&a);const auto fp=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(0x39f0ec)(b.memory.flat(),&y,&b);const auto composeFp=_mm_getcsr();_mm_setcsr(saved);check(composeFp==fp);x.cycleLimit=y.cycleLimit;
        if(x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"object compose mode%x seed%u PCs%x/%x\n",mode,seed,x.pc,y.pc);
            for(unsigned i=0;i<sizeof(x);++i)if(reinterpret_cast<const uint8_t *>(&x)[i]!=reinterpret_cast<const uint8_t *>(&y)[i]){std::fprintf(stderr,"CPU byte%x old%02x native%02x\n",i,reinterpret_cast<const uint8_t *>(&x)[i],reinterpret_cast<const uint8_t *>(&y)[i]);break;}
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        if(state51==1)composedProducts+=2*((seed&1)+((seed>>1)&1));++compositions;
    }
    {auto &rt=adapted.runtime();for(unsigned which=0;which<5;++which){constexpr uint32_t object=0x1a0000;
        R5900Context c{};c.r[16].uw[0]=object;c.r[4].uw[0]=object+0x90;c.r[3].uw[0]=object+0x210;uint32_t pc=0x39f17c;
        const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};for(uint32_t at:{0x90u,0xd0u,0x210u,0x250u})std::memcpy(rt.memory.hostPointer(object+at,64),identity,64);
        if(which==0)c.r[3].uw[0]=object+0x250;else if(which==1){pc=0x39f1e0;c.r[3].uw[0]=object+0x90;}else if(which==2)c.r[16].uw[0]=0;
        else{const uint32_t bad=which==3?0x7f800000:0x7fc00001;std::memcpy(rt.memory.hostPointer(object+0x214,4),&bad,4);}
        const auto before=c;const auto fp=_mm_getcsr();const auto ram=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc);}catch(const std::runtime_error &){rejected=true;}check(rejected && _mm_getcsr()==fp && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));
    }}
    std::printf("%u object compositions (%u matrix products) and5 readonly rejections passed\n",compositions,composedProducts);
    unsigned objectRotations=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=original.runtime(),&b=adapted.runtime();std::memset(a.memory.flat(),0x63+seed,bozo::kFlatBytes);const uint32_t aliases[]={0,0x20000000,0x30000000,0x80000000,0xa0000000};
        const uint32_t object=0x190000|aliases[(seed/4)%5],stack=seed&1?0x70002d60u:0x1ffd100u|aliases[(seed/8)%5];
        uint32_t q[4];for(unsigned i=0;i<4;++i)q[i]=((seed*6007+i*2741)&0x807fffff)|((117+(seed+i)%10)<<23);
        if(seed%8==0){q[0]=q[1]=q[2]=0;q[3]=0x3f800000;}if(seed%8==1){q[0]=1;q[1]=0x80000001;q[2]=0x00800000;q[3]=0x3f800000;}
        std::memcpy(a.memory.hostPointer(stack+0x20,16),q,16);const uint64_t ret=0x170000;std::memcpy(a.memory.hostPointer(stack+0x10,8),&ret,8);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x5678000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/16;}
        std::memset(x.vu0_vf,0x5c,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xc5,sizeof(x.vu0_acc));x.vu0_status=0x1357;x.vu0_q_bits=0x2468ace0;x.vi[0]=0x5678;
        x.r[16].ud[0]=object;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x39f318;auto y=x;const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&8?0x8040:0);_mm_setcsr(control);
        host.reference=true;a.functions.generated(0x39f318)(a.memory.flat(),&x,&a);const auto fp=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(0x39f318)(b.memory.flat(),&y,&b);const auto rotationFp=_mm_getcsr();_mm_setcsr(saved);check(rotationFp==fp);x.cycleLimit=y.cycleLimit;
        if(x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"object rotation mode%x seed%u PCs%x/%x\n",mode,seed,x.pc,y.pc);
            for(unsigned i=0;i<sizeof(x);++i)if(reinterpret_cast<const uint8_t *>(&x)[i]!=reinterpret_cast<const uint8_t *>(&y)[i]){std::fprintf(stderr,"CPU byte%x old%02x native%02x\n",i,reinterpret_cast<const uint8_t *>(&x)[i],reinterpret_cast<const uint8_t *>(&y)[i]);break;}
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        const auto rows=movementRotationOracle(q);check(!std::memcmp(rows.data(),b.memory.hostPointer(object+0x250,48),48));
        const std::array<float,4> last{0,0,0,1};check(!std::memcmp(last.data(),b.memory.hostPointer(object+0x280,16),16));++objectRotations;
    }
    {auto &rt=adapted.runtime();for(unsigned which=0;which<7;++which){constexpr uint32_t stack=0x1ffd100,object=0x190000;const std::array<float,4> identity{0,0,0,1};std::memcpy(rt.memory.hostPointer(stack+0x20,16),identity.data(),16);
        R5900Context c{};c.pc=0x39f318;c.r[31].uw[0]=0x39f318;c.r[29].uw[0]=stack;c.r[16].uw[0]=object;c.r[3].uw[0]=object+0x250;uint32_t pc=0x39f368;
        if(which==0)c.pc=0;else if(which==1)c.r[31].uw[0]=0;else if(which==2)c.r[3].uw[0]+=16;else if(which==3){pc=0x39f31c;}else if(which==4){pc=0x39f324;}
        else {const uint32_t bad=which==5?0x7fa01234:0x7f800000;std::memcpy(rt.memory.hostPointer(stack+0x24,4),&bad,4);}
        const auto before=c;const auto fp=_mm_getcsr();const auto ram=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc);}catch(const std::runtime_error &){rejected=true;}check(rejected && _mm_getcsr()==fp && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));
    }}
    std::printf("%u object rotations (quaternion getter output), engine VU0 oracle and7 readonly rejections passed\n",objectRotations);
    // mod003 396c00: quaternion at s0+250, program 0x720, rows to SP+40.
    unsigned facingRotations=0,facingRejects=0;
    for(unsigned seed=0;seed<256;++seed){
        auto &rt=adapted.runtime();std::memset(rt.memory.flat(),0x41+seed,bozo::kFlatBytes);const uint32_t aliases[]={0,0x20000000,0x30000000,0x80000000,0xa0000000};
        const uint32_t object=0x190000|aliases[(seed/4)%5],stack=seed&1?0x70002d60u:0x1ffd100u|aliases[(seed/8)%5];
        uint32_t q[4];for(unsigned i=0;i<4;++i)q[i]=((seed*7919+i*2741)&0x807fffff)|((117+(seed+i)%10)<<23);
        if(seed%8==0){q[0]=q[1]=q[2]=0;q[3]=0x3f800000;}
        std::memcpy(rt.memory.hostPointer(object+0x250,16),q,16);
        const auto ram=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
        R5900Context c{};for(unsigned i=1;i<32;++i)c.r[i].ud[0]=0x6789000000000000ull+seed*32+i;c.r[16].ud[0]=object;c.r[29].ud[0]=stack;c.r[3].ud[0]=object+0x250;c.r[2].ud[0]=stack+0x40;
        const auto before=c;const auto fp=_mm_getcsr();
        for(uint32_t pc:{0x396c24u,0x396c2cu,0x396c5cu,0x396c64u,0x396c68u,0x396c6cu})check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc));
        check(_mm_getcsr()==fp);auto expect=before;expect.r[9].ud[0]=0;check(!std::memcmp(&expect,&c,sizeof(c)));
        const auto rows=movementRotationOracle(q);check(!std::memcmp(rows.data(),rt.memory.hostPointer(stack+0x40,48),48));
        if(stack<0x70000000){std::memcpy(rt.memory.hostPointer(stack+0x40,48),ram.data()+((stack+0x40)&0x1ffffff),48);check(!std::memcmp(ram.data(),rt.memory.flat(),ram.size()));}
        ++facingRotations;
        // Rejections leave everything unchanged.
        R5900Context bad=before;uint32_t pc=0x396c64;
        if(seed%3==0)bad.r[2].uw[0]+=16;else if(seed%3==1){pc=0x396c24;bad.r[3].uw[0]+=16;}
        else{const uint32_t inf=0x7f800000;std::memcpy(rt.memory.hostPointer(object+0x254,4),&inf,4);}
        const auto snapshot=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);const auto bad0=bad;bool rejected=false;
        try{vp2s::nativeMathInstruction(rt.memory.flat(),&bad,&rt,pc);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && !std::memcmp(&bad,&bad0,sizeof(bad)) && !std::memcmp(snapshot.data(),rt.memory.flat(),snapshot.size()));++facingRejects;
    }
    std::printf("%u mod003 396c00 facing rotations match the engine VU0 oracle (%u rejections)\n",facingRotations,facingRejects);
    unsigned steps=0,skippedSteps=0;
    for(unsigned pass=0;pass<2;++pass)for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=original.runtime(),&b=adapted.runtime();std::memset(a.memory.flat(),0x37+seed,bozo::kFlatBytes);const uint32_t aliases[]={0,0x20000000,0x30000000,0x80000000,0xa0000000};
        const uint32_t entry=pass?0x3807d0u:0x3805f0u,positionAt=pass?0x220u:0x280u,targetAt=pass?0x10u:0x20u,lengthAt=pass?0x70u:0xa0u;
        const uint32_t object=0x180000|aliases[(seed/4)%5],stack=seed&1?0x70002d90u:0x1ffd000u|aliases[(seed/8)%5];
        uint32_t position[4],target[4];for(unsigned i=0;i<4;++i){position[i]=((seed*7919+i*3121)&0x807fffff)|((120+(seed+i)%14)<<23);target[i]=((seed*104729+i*7477)&0x807fffff)|((120+(seed*3+i)%14)<<23);}
        if(seed%16==2){position[0]=target[0];position[1]=target[1];position[2]=target[2];}             // zero displacement: length +0
        if(seed%16==3){position[0]=0x40400000;position[1]=0x40800000;position[2]=0;target[0]=target[1]=target[2]=0;} // 3,4,0 -> 5
        if(seed%16==4){position[0]=0x00000001;position[1]=0x80000000;position[2]=0x00800001;target[0]=0;target[1]=0;target[2]=0x00800000;} // subnormal and tiny terms
        const uint32_t time=seed%16==5?0x41200000u:seed%16==6?0x41a00000u:(0x3c000000u|((seed*977)&0x7fffff));const bool skip=seed%16==5 || seed%16==6;
        std::memcpy(a.memory.hostPointer(stack+positionAt,16),position,16);std::memcpy(a.memory.hostPointer(object+targetAt,16),target,16);std::memcpy(a.memory.hostPointer(object+0x60,4),&time,4);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x4321000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/4;}
        std::memset(x.vu0_vf,0x3c,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0xc3,sizeof(x.vu0_acc));x.vu0_status=0x2468;x.vu0_q_bits=0x13579bdf;x.vi[0]=0x4321;
        x.r[20].ud[0]=object;x.r[29].ud[0]=stack;x.r[31].ud[0]=entry;auto y=x;const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&8?0x8040:0);_mm_setcsr(control);
        host.reference=true;a.functions.generated(entry)(a.memory.flat(),&x,&a);const auto fp=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(entry)(b.memory.flat(),&y,&b);const auto stepFp=_mm_getcsr();_mm_setcsr(saved);
        if(stepFp!=fp)std::fprintf(stderr,"movement step mode%x seed%u mxcsr old%08x native%08x control%08x\n",mode,seed,fp,stepFp,control);check(stepFp==fp);x.cycleLimit=y.cycleLimit;
        if(x.pc!=entry || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"movement step pass%u mode%x seed%u PCs%x/%x\n",pass,mode,seed,x.pc,y.pc);
            for(unsigned i=0;i<sizeof(x);++i)if(reinterpret_cast<const uint8_t *>(&x)[i]!=reinterpret_cast<const uint8_t *>(&y)[i]){std::fprintf(stderr,"CPU byte%x old%02x native%02x\n",i,reinterpret_cast<const uint8_t *>(&x)[i],reinterpret_cast<const uint8_t *>(&y)[i]);break;}
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(a.memory.flat()[i]!=b.memory.flat()[i]){std::fprintf(stderr,"RAM byte%x old%02x native%02x\n",i,a.memory.flat()[i],b.memory.flat()[i]);break;}check(false);
        }
        if(!skip){uint32_t length;std::memcpy(&length,b.memory.hostPointer(stack+lengthAt,4),4);
            if(seed%16==2)check(length==0);if(seed%16==3)check(length==0x40a00000u);}
        else{++skippedSteps;uint8_t untouched[16];std::memcpy(untouched,b.memory.hostPointer(stack+lengthAt,16),16);
            for(uint8_t v:untouched)check(v==uint8_t(0x37+seed));} // threshold path never reaches the length
        ++steps;
    }
    {auto &rt=adapted.runtime();for(unsigned which=0;which<6;++which){constexpr uint32_t stack=0x1ffd000;const std::array<float,4> displacement{1,2,3,4};std::memcpy(rt.memory.hostPointer(stack+0x270,16),displacement.data(),16);
        R5900Context c{};c.r[31].uw[0]=0x3805f0;c.r[29].uw[0]=stack;c.r[4].uw[0]=stack+0x270;c.r[3].uw[0]=stack+0xa0;uint32_t pc=0x380684;
        if(which==0)c.r[31].uw[0]=0x380118;else if(which==1)c.r[4].uw[0]+=16;else if(which==2)c.r[3].uw[0]=stack+0xb0;else if(which==3){pc=0x380664;c.r[4].uw[0]=0;}
        else {const uint32_t bad=which==4?0x7f800000:0x7fa01234;std::memcpy(rt.memory.hostPointer(stack+0x274,4),&bad,4);}
        const auto before=c;const auto fp=_mm_getcsr();const auto ram=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc);}catch(const std::runtime_error &){rejected=true;}check(rejected && _mm_getcsr()==fp && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));
    }}
    std::printf("%u movement step lengths (%u threshold skips), engine VU0 oracle and6 readonly rejections passed\n",steps,skippedSteps);
    unsigned dots=0;uint32_t random=71;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=original.runtime();auto &b=adapted.runtime();std::memset(a.memory.flat(),seed+1,bozo::kFlatBytes);
        const uint32_t matrix=seed&1?0x70000100:0x180000,vector=0x190000;
        const uint32_t destination=seed%3==0?matrix+(seed%4)*16:seed%3==1?vector:0x1a0000;
        uint32_t values[20];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((96+random%56)<<23);}
        if(seed%8==0){for(unsigned i=0;i<20;++i)values[i]=std::bit_cast<uint32_t>(i>=16?1.f:(i%2?-1.f:1.f));}
        if(seed%8==1){for(unsigned i=16;i<20;++i)values[i]=(i&1)?0x80000000u:0;}
        std::memcpy(a.memory.hostPointer(matrix,64),values,64);std::memcpy(a.memory.hostPointer(vector,16),values+16,16);std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0xf3,sizeof(x.vu0_vf));x.vu0_acc.u[0]=0x4321;x.vu0_status=0x1234;
        x.r[4].ud[0]=matrix;x.r[5].ud[0]=vector;x.r[6].ud[0]=destination;x.r[31].ud[0]=0x170000;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        host.reference=true;a.functions.generated(0x410df0)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(0x410df0)(b.memory.flat(),&y,&b);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)));
        check(!std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));++dots;
    }
    unsigned boxes=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=original.runtime(),&b=adapted.runtime();std::memset(a.memory.flat(),0x91,bozo::kFlatBytes);
        const uint32_t box=seed&1?0x70000100:0x180000,stack=seed&2?0x70003e00:0x1fff000;
        uint32_t values[8];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((100+random%30)<<23);}
        if(seed%8==0)for(unsigned i=0;i<3;++i)values[i+4]=values[i];
        if(seed%8==1)for(unsigned i=0;i<8;++i)values[i]=(i&1)?0x80000000u:0;
        values[3]=0x7fc01234;values[7]=0xffc05678;
        std::memcpy(a.memory.hostPointer(box,32),values,32);std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0x41,sizeof(x.vu0_vf));x.vu0_status=0x6543;
        x.r[4].ud[0]=box;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        host.reference=true;a.functions.generated(0x37fa30)(a.memory.flat(),&x,&a);const auto expected=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;b.functions.generated(0x37fa30)(b.memory.flat(),&y,&b);check(_mm_getcsr()==expected);_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)));
        check(!std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));++boxes;
    }
    {auto &r=adapted.runtime();R5900Context c{};c.r[4].ud[0]=0x180000;c.r[8].ud[0]=0x180010;c.r[7].ud[0]=0x180020;c.r[9].ud[0]=0x190000;
        std::memset(r.memory.hostPointer(0x180000,64),0,64);float half=0.5f;std::memcpy(r.memory.hostPointer(0x190000,4),&half,4);
        auto reject=[&](const char *label){auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x37fad8);}catch(const std::runtime_error &){bad=true;}
            if(!bad)std::fprintf(stderr,"sphere rejection unexpectedly accepted: %s\n",label);
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[7].ud[0]+=16;reject("output");c.r[7].ud[0]-=16;
        half=0.25f;std::memcpy(r.memory.hostPointer(0x190000,4),&half,4);reject("coefficient");half=0.5f;std::memcpy(r.memory.hostPointer(0x190000,4),&half,4);
        uint32_t value=0x7fc00000;std::memcpy(r.memory.hostPointer(0x180000,4),&value,4);reject("NaN");
        value=0x7f7fffff;std::memcpy(r.memory.hostPointer(0x180000,4),&value,4);reject("squared overflow");
    }
    unsigned colours=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &a=original.runtime(),&b=adapted.runtime();std::memset(a.memory.flat(),0x61,bozo::kFlatBytes);
        constexpr uint32_t object=0x180000,table=0x1f0000;const uint32_t stack=seed%4==0?object+0xe0:seed&1?0x70003e00:0x1fff000;
        uint32_t values[8];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((95+random%40)<<23);}
        if(seed%8==0)for(unsigned i=0;i<8;++i)values[i]=(i&1)?0x80000000u:0;
        if(seed%8==2)for(unsigned i=0;i<4;++i){values[i]=0x00800000u|(i&1?0x80000000u:0);values[i+4]=0x3f000000;}
        std::memcpy(a.memory.ram()+object+0xd0,values,32);
        const uint32_t manager=seed%5==0?0:0x1e0000,flag=seed%3==0?0xffffffffu:seed%4;
        std::memcpy(a.memory.ram()+object+0x104,&manager,4);a.memory.ram()[object+0x110]=0x7b;
        host.nodes.clear();unsigned expectedCalls=0;
        for(unsigned i=0;i<seed%4;++i){const uint32_t node=0x200000+i*0x1000;host.nodes.push_back(node);
            std::memcpy(a.memory.ram()+node,&table,4);const uint16_t type=(i+seed)&1?0x110:0x111;
            std::memcpy(a.memory.ram()+node+0x6c,&type,2);if(type!=0x110)expectedCalls+=2;}
        const uint32_t setter=0x172000,colour=0x172010;std::memcpy(a.memory.ram()+table+0x8c,&setter,4);std::memcpy(a.memory.ram()+table+0x94,&colour,4);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0x82,sizeof(x.vu0_vf));x.vu0_status=0x1254;x.r[4].ud[0]=object;x.r[5].sd[0]=int32_t(flag);x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;auto y=x;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        host.colourCalls=0;host.reference=true;a.functions.generated(0x43d280)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();check(host.colourCalls==(manager?expectedCalls:0));_mm_setcsr(control);
        host.colourCalls=0;host.reference=false;b.functions.generated(0x43d280)(b.memory.flat(),&y,&b);check(_mm_getcsr()==expectedFp && host.colourCalls==(manager?expectedCalls:0));_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)));check(!std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        check(b.memory.ram()[object+0x110]==(flag==0xffffffffu?0x7b:flag));++colours;
    }
    {auto &r=adapted.runtime();constexpr uint32_t object=0x180000;R5900Context c{};c.r[16].ud[0]=object;c.r[6].ud[0]=object+0xd0;c.r[4].ud[0]=object+0xe0;c.r[29].ud[0]=0x1fff000;c.r[3].ud[0]=0x1fff030;
        uint32_t values[8];for(auto &v:values)v=0x3f800000;std::memcpy(r.memory.ram()+object+0xd0,values,32);
        auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x43d2ac);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[3].ud[0]+=16;reject();c.r[3].ud[0]-=16;
        for(uint32_t bad:{0x7fc00000u,1u}){std::memcpy(r.memory.ram()+object+0xd0,&bad,4);reject();}
        values[0]=0x7f7fffff;values[4]=0x40000000;std::memcpy(r.memory.ram()+object+0xd0,values,32);reject();
    }
    std::printf("%u distances,%u automatic camera callers (%u known),%u animation movements (%u node callbacks),%u actions (%u active,%u zero),%u rotations,%u row dots,%u box/spheres,%u model-colour callers; CPU/RAM, integer arithmetic, four modes, aliasing and rejected input; no VU/device state\n",cases,automaticDistances,knownCameraDistances,animationMoves,animationNodeCalls,actions,activeActions,zeroActions,rotations,dots,boxes,colours);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

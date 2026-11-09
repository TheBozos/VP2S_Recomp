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
        if(referenceOverlap && (pc==0x1f9da4 || pc==0x1f9e28 || pc==0x2a4994 || pc==0x2a49a0 || pc==0x2a418c || pc==0x2a4258 || pc==0x2a4478 || pc==0x2a44a8)){
            const unsigned laneReg=pc==0x2a49a0 || pc==0x2a44a8?13:10,destinationReg=pc==0x2a4994?4:pc==0x2a44a8?2:3;
            const auto mul=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);};
            const auto sub=[](uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::subNearest(a,b,f);};
            uint32_t out[4],flags=0;for(unsigned i=0;i<3;++i)out[i]=bozo::ps2f::fromFixed(c->r[laneReg].uw[i],15);
            auto rest=0x3f800000u;for(unsigned i=0;i<3;++i)rest=sub(rest,mul(mul(out[i],out[i]),0x3f800000));
            const auto saved=_mm_getcsr();out[3]=mul(0x3f800000,bozo::ps2f::sqrt(rest,flags));_mm_setcsr(saved);
            std::memcpy(t->memory.hostPointer(c->r[destinationReg].uw[0],16),out,16);return true;
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
int main(){try{
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
        check(rejected && _mm_getcsr()==fp && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));
    }}
    std::printf("%u segment/sphere contact runs (%u contacts) and5 readonly rejections passed\n",contactCases,contacts);
    unsigned packedCases=0;
    for(unsigned block=0;block<6;++block)for(unsigned rounding:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
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
        const uint32_t entries[6]={0x1f9d74,0x1f9df8,0x2a492c,0x2a415c,0x2a4228,0x2a440c},entry=entries[block];
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|rounding|(seed&8?0x8040:0);_mm_setcsr(control);
        host.referenceOverlap=true;a.functions.generated(entry)(a.memory.flat(),&x,&a);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        host.referenceOverlap=false;b.functions.generated(entry)(b.memory.flat(),&y,&b);const auto actualFp=_mm_getcsr();_mm_setcsr(saved);
        x.cycleLimit=y.cycleLimit;
        if(actualFp!=expectedFp || x.pc!=0x170000 || std::memcmp(&x,&y,sizeof(x)) || std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"packed quaternion block%u rounding%x seed%u pc%x/%x fp%08x/%08x\n",block,rounding,seed,x.pc,y.pc,expectedFp,actualFp);
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
    // Animated packet UVs. The original loader 219620 (with its mesh cursors
    // and element methods) builds the scratchpad description; a test-only
    // oracle reproduces the original pipeline from it (toSPR chain, callback
    // 29cfe0 + micro 0xc80 with ps2f arithmetic, fromSPR destination chain).
    // The native rewrite reads only the records and must produce the same packets.
    unsigned uvCases=0,uvChunks=0,uvWords=0,uvEmpty=0;
    for(unsigned seed=0;seed<96;++seed){
        auto &a=old.runtime(),&b=native.runtime();std::memset(a.memory.flat(),0,bozo::kFlatBytes);
        uint32_t state=seed*747796405u+2891336453u;const auto next=[&](){state=state*1664525u+1013904223u;return state>>8;};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(a.memory.hostPointer(at,4),&v,4);};
        const auto put16=[&](uint32_t at,uint16_t v){std::memcpy(a.memory.hostPointer(at,2),&v,2);};
        const auto get=[&](Runtime &r,uint32_t at){uint32_t v;std::memcpy(&v,r.memory.hostPointer(at,4),4);return v;};
        constexpr uint32_t object=0x200000,model=0x201000,iterator=0x201800,maskAt=0x200800,vtable=0x1591b0,data=0x210000,packets=0x240000,stack=0x1ffc000;
        const unsigned meshes=1+seed%4,elements=1+seed%3;const bool unaligned=seed&1;
        const uint32_t base=data;  // 128-aligned, so both iterator modes agree
        put(object+0x54,model);put(model+0x644,iterator);put(iterator+8,data);put(iterator+0x14,seed%5==4?0:packets);
        std::memcpy(a.memory.hostPointer(iterator+0x1c,1),&unaligned,1);
        put(base+0x10+0x34,meshes);put(object+0x48,maskAt);put(object+0x58,0x28);
        put(object+0x5c,0x202000);put(object+0x60,0x203000);put(object+0x68,0x204000);put(object+0x6c,0x204400);put(object+0x70,0x205000);
        const uint8_t mode=uint8_t(seed%2),rebuild=2,flags8d=uint8_t(seed%7==3?4:0);
        std::memcpy(a.memory.hostPointer(object+0x8a,1),&mode,1);std::memcpy(a.memory.hostPointer(object+0x8c,1),&rebuild,1);std::memcpy(a.memory.hostPointer(object+0x8d,1),&flags8d,1);
        uint32_t maskBits=0;for(unsigned j=0;j<meshes;++j)if(seed%5==0 || next()%4)maskBits|=1u<<j;put(maskAt,maskBits);
        const uint32_t packetStart=seed%5==4?base+0x70:packets+0x60;
        uint32_t source=base+0x70,packet=packetStart;
        for(unsigned j=0;j<meshes;++j){
            const uint16_t n=uint16_t(seed%9==8?1+next()%3:4+next()%(seed%3==0?1000:120));const uint16_t offset=0x40;
            const uint32_t span=(0x40+((n+3)/4)*16+0x2f)&~0xfu;
            put16(source+0x16,n);put16(source+0x1c,offset);put16(source+4,uint16_t(span));
            if(packet!=source){put16(packet+4,uint16_t(span));}
            for(unsigned w=0;w<((n+3u)&~3u);++w){put(source+offset+4*w,next()*7u);if(packet!=source)put(packet+offset+4*w,0xdeadbeef);}
            source+=span;packet+=span;
        }
        put16(source+4,0);if(packet!=source)put16(packet+4,0);  // terminators (never inside the count)
        put(vtable+0x10,0x29cfe0);put(vtable+0x14,0x29cfd0);put(vtable+0x18,0x29d0d0);
        uint32_t descriptorAt=0x230000;
        for(unsigned i=0;i<elements;++i){
            const uint32_t e=0x206000+0x100*i,bits=0x206800+0x10*i;put(e,vtable);put(e+8,i+1<elements?e+0x100:object);
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
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
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
        ++uvCases;
    }
    std::printf("%u animated-UV models (%u loader chunks, %u packet words, %u without chunks) match the original scratchpad pipeline\n",uvCases,uvChunks,uvWords,uvEmpty);
    std::printf("%u matrix/quaternion/Euler ABI cases, %u component products, %u whole CPU/RAM sphere getters,8192 independent scale cases and%u whole CPU/RAM axis-angle cases passed; no VU/device state\n",cases,products,sphereCases,axisCases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

#include "native_math.h"
#include "../port/vp2s_math.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "math_original_functions.h"
#include "math_adapted_functions.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <bit>
#include <cmath>
#include <source_location>
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
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("unexpected library request");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *t,uint32_t pc)override{
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
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *c,uint32_t pc)override{
        if(pc==0x12e1d8)c->f[0]=std::asin(c->f[12]);
        else if(pc==0x12e2d8)c->f[0]=std::atan2(c->f[12],c->f[13]);
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
    for(auto *d:{&old,&native})for(uint32_t pc:{0x12e1d8u,0x12e2d8u})d->bindLibrary(pc);
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
        if(sample%8192==0)check(!std::memcmp(r.memory.flat(),old.runtime().memory.flat(),bozo::kFlatBytes));
    }
    R5900Context invalid{};invalid.r[6].uw[0]=object+0x560;invalid.r[5].uw[0]=object+0x570;invalid.r[3].uw[0]=object+0x580;
    const uint32_t nan=0x7fc00000;std::memcpy(p+0x560,&nan,4);uint32_t prior[4];std::memcpy(prior,p+0x580,16);
    bool nonfiniteRejected=false;try{vp2s::nativeMathInstruction(r.memory.flat(),&invalid,&r,0x211dbc);}catch(const std::runtime_error &){nonfiniteRejected=true;}
    check(nonfiniteRejected && !std::memcmp(prior,p+0x580,16));
    std::memset(p,0,0x54);uint32_t type=0x1591d0;std::memcpy(p,&type,4);
    R5900Context c{};c.r[4].uw[0]=object;const auto saved=c;
    check(vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x29e1a0));check(!std::memcmp(&saved,&c,sizeof(c)) && p[0x50]==0);
    for(unsigned mode:{1u,128u,255u}){c.r[5].uw[0]=mode;bool rejected=false;try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x29e1a0);}catch(const std::runtime_error &){rejected=true;}check(rejected);}
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
    std::printf("%u matrix/quaternion/Euler ABI cases, %u component products, %u whole CPU/RAM sphere getters and8192 independently rounded scale cases passed; no VU/device state\n",cases,products,sphereCases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

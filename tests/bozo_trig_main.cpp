#include "native_trig.h"
#include "native_math.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "trig_original_functions.h"
#include "trig_adapted_functions.h"
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <set>
#include <stdexcept>
#include <source_location>
#include <immintrin.h>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){
    if(!v)throw std::runtime_error("native trig assertion at "+std::to_string(at.line()));
}
float flush(float f){const auto b=std::bit_cast<uint32_t>(f);return std::bit_cast<float>((b&0x7f800000)?b:b&0x80000000);}
[[gnu::noinline,gnu::noclone]] float multiply(float a,float b){volatile float result=a*b;return flush(result);}
[[gnu::noinline,gnu::noclone]] float add(float a,float b){volatile float result=a+b;return flush(result);}
[[gnu::noinline,gnu::noclone]] float subtract(float a,float b){volatile float result=a-b;return flush(result);}
struct ScalarResult {std::array<int32_t,2> quadrants;std::array<float,4> value;};
// Independent finite float32 rational oracle for the four normalization
// publications. No CPU/VU execution or register bank participates.
uint32_t exactProduct(uint32_t a,uint32_t b){
    const auto sign=(a^b)&0x80000000u;const unsigned ea=(a>>23)&255,eb=(b>>23)&255;
    if(!ea || !eb)return sign;
    const uint64_t product=uint64_t((a&0x7fffff)|0x800000)*((b&0x7fffff)|0x800000);
    int exponent=int(ea)+int(eb)-127;unsigned shift=23;
    if(product&(1ull<<47)){++exponent;++shift;}
    if(exponent<=0)return sign;if(exponent>=255)return sign|0x7f7fffffu;
    return sign|uint32_t(exponent<<23)|uint32_t((product>>shift)&0x7fffff);
}
uint32_t positiveSum(uint32_t a,uint32_t b){
    if(a<b)std::swap(a,b);if(!b)return a;
    unsigned exponent=a>>23;const auto distance=exponent-(b>>23);
    uint32_t sum=(a&0x7fffff)|0x800000;
    if(distance<=23)sum+=((b&0x7fffff)|0x800000)>>distance;
    if(sum&0x1000000){sum>>=1;++exponent;}return (exponent<<23)|(sum&0x7fffff);
}
double finiteValue(uint32_t bits){
    const auto exponent=bits>>23,mantissa=(bits&0x7fffff)|(exponent?0x800000:0);
    return std::ldexp(double(mantissa),exponent?int(exponent)-150:-149);
}
std::array<uint32_t,4> referenceUnit(const uint32_t *input){
    auto squared=exactProduct(input[0],input[0]);
    for(unsigned i=1;i<4;++i)squared=positiveSum(squared,exactProduct(input[i],input[i]));
    uint32_t low=0,high=0x7f7fffff;
    while(low<high){const auto mid=low+(high-low+1)/2;const auto v=finiteValue(mid);
        if(v*v<=finiteValue(squared))low=mid;else high=mid-1;}
    const auto root=low;low=0;high=0x7f7fffff;
    if(root)while(low<high){const auto mid=low+(high-low+1)/2;
        if(finiteValue(mid)*finiteValue(root)<=1.0)low=mid;else high=mid-1;}
    else low=0x7f7fffff;
    std::array<uint32_t,4> out;for(unsigned i=0;i<4;++i)out[i]=exactProduct(input[i],low);return out;
}
[[gnu::noinline,gnu::noclone]] ScalarResult reference(float angle,const vp2s::TrigCoefficients &c){
    ScalarResult out;out.value={c.sine[3],c.sine[2],0,0};
    const float arguments[2]={subtract(c.halfPi,angle),angle};
    for(unsigned i=0;i<2;++i){
        volatile float turns=multiply(arguments[i],c.inversePi);const int32_t n=static_cast<int32_t>(turns);out.quadrants[i]=n;
        volatile float integer=static_cast<float>(n);const float residual=subtract(arguments[i],multiply(integer,c.pi));
        const float squared=multiply(residual,residual);
        float term=add(multiply(squared,c.sine[0]),c.sine[1]);
        for(unsigned j=2;j<6;++j)term=add(multiply(term,squared),c.sine[j]);
        term=add(multiply(term,squared),1);
        out.value[i+2]=multiply(term,multiply(residual,(n&1)?-1.f:1.f));
    }return out;
}
template<class T>T value(Runtime *rt,uint32_t at){T v;std::memcpy(&v,rt->memory.hostPointer(at,sizeof(v)),sizeof(v));return v;}
vp2s::TrigCoefficients read(Runtime *rt){
    vp2s::TrigCoefficients c;c.pi=value<float>(rt,0x35da70);c.inversePi=value<float>(rt,0x35da74);c.halfPi=value<float>(rt,0x35da98);
    c.sine={value<float>(rt,0x35daa4),value<float>(rt,0x35daa0),value<float>(rt,0x35da84),value<float>(rt,0x35da80),value<float>(rt,0x35da7c),value<float>(rt,0x35da78)};return c;
}
vp2s::InverseCosineCoefficients readInverse(Runtime *rt){
    vp2s::InverseCosineCoefficients c;std::memcpy(c.values.data(),rt->memory.hostPointer(0x35da30,60),60);return c;
}
[[gnu::noinline,gnu::noclone]] float referenceInverseRoot(float angle,const vp2s::InverseCosineCoefficients &c){
    const auto z=subtract(c.values[13],multiply(angle,c.values[13]));volatile float root=static_cast<float>(std::sqrt(std::abs(double(z))));return flush(root);
}
[[gnu::noinline,gnu::noclone]] float referenceInverse(float angle,const vp2s::InverseCosineCoefficients &c){
    const auto bits=std::bit_cast<uint32_t>(angle),magnitude=bits&0x7fffffff;
    if(magnitude>=0x3f800000)return bits&0x80000000?std::bit_cast<float>(0x40490fdbu):0.f;
    if(magnitude<0x23000001)return std::bit_cast<float>(0x3fc90fdbu);
    const auto &v=c.values;const bool small=magnitude<0x3f000000,negative=bits&0x80000000;
    const float z=small?multiply(angle,angle):negative?add(v[13],multiply(angle,v[13])):subtract(v[13],multiply(angle,v[13]));
    float numerator=v[8];for(int i=7;i>=3;--i)numerator=add(multiply(numerator,z),v[i]);numerator=multiply(numerator,z);
    float denominator=v[12];for(int i=11;i>=9;--i)denominator=add(multiply(denominator,z),v[i]);denominator=add(multiply(denominator,z),1.f);
    volatile float quotient=numerator/denominator;const auto ratio=flush(quotient);
    if(small)return subtract(v[1],subtract(angle,subtract(v[2],multiply(angle,ratio))));
    volatile float root=static_cast<float>(std::sqrt(std::abs(double(z))));const auto s=flush(root);
    if(negative)return subtract(v[0],multiply(add(subtract(multiply(s,ratio),v[2]),s),v[14]));
    const auto truncated=std::bit_cast<float>(std::bit_cast<uint32_t>(s)&0xfffff000u);
    const auto numeratorCorrection=subtract(z,multiply(truncated,truncated)),denominatorCorrection=add(s,truncated);
    volatile float correction=numeratorCorrection/denominatorCorrection;
    return add(multiply(v[14],truncated),multiply(v[14],add(multiply(s,ratio),flush(correction))));
}
vp2s::ThreeSineReduction referenceThreeReduction(const std::array<float,3> &angles,const vp2s::TrigCoefficients &c){
    vp2s::ThreeSineReduction result;
    for(unsigned i=0;i<3;++i){const auto scaled=multiply(angles[i],c.inversePi);const auto quadrant=static_cast<int32_t>(scaled);volatile float integer=static_cast<float>(quadrant);
        result.quadrants[i]=quadrant;result.scaled[i]=scaled;result.residual[i]=subtract(add(0.f,angles[i]),multiply(integer,c.pi));}return result;
}
std::array<float,3> referenceThreeSines(const std::array<float,3> &angles,const vp2s::TrigCoefficients &c){
    const auto reduced=referenceThreeReduction(angles,c);std::array<float,3> result;
    for(unsigned i=0;i<3;++i){const auto squared=multiply(reduced.residual[i],reduced.residual[i]);
        auto term=add(multiply(c.sine[0],reduced.scaled[i]),c.sine[1]);for(unsigned j=2;j<6;++j)term=add(multiply(term,squared),c.sine[j]);
        const auto signedResidual=multiply(reduced.residual[i],(uint32_t(reduced.quadrants[i])&1)?-1.f:1.f);
        result[i]=multiply(add(multiply(term,squared),1.f),signedResidual);}return result;
}
std::array<float,4> referenceLinearQuaternion(Runtime *rt,R5900Context *c){
    float a[4],b[4],weights[2],blend[4],squares[4];std::array<float,4> out;
    std::memcpy(a,rt->memory.hostPointer(c->r[17].uw[0],16),16);std::memcpy(b,rt->memory.hostPointer(c->r[16].uw[0],16),16);
    float scale=1.f;
    if(c->pc==0x2152ec || c->pc==0x215324){const auto sines=referenceThreeSines({c->f[12],c->f[13],c->f[14]},read(rt));weights[0]=sines[0];weights[1]=sines[1];
        volatile float reciprocal=sines[2]==0?std::copysign(std::numeric_limits<float>::max(),sines[2]):1.f/sines[2];scale=flush(reciprocal);
    }else std::memcpy(weights,rt->memory.hostPointer(c->r[29].uw[0]+0x60,8),8);
    for(unsigned i=0;i<4;++i){const auto first=multiply(a[i],weights[0]),second=multiply(b[i],weights[1]);
        blend[i]=multiply(c->r[19].ud[0]?subtract(second,first):add(second,first),scale);squares[i]=multiply(blend[i],blend[i]);}
    const auto squared=add(add(add(squares[0],squares[1]),multiply(1.f,squares[2])),multiply(1.f,squares[3]));
    volatile float root=static_cast<float>(std::sqrt(std::abs(double(squared))));volatile float reciprocal=root==0?std::numeric_limits<float>::max():static_cast<float>(1.0/double(root));
    for(unsigned i=0;i<4;++i)out[i]=multiply(blend[i],flush(reciprocal));return out;
}
[[gnu::noinline,gnu::noclone]] vp2s::NativeCosine referenceShort(float angle,const vp2s::TrigCoefficients &c,bool sine=false,bool full=false){
    const float argument=sine?angle:subtract(c.halfPi,angle);volatile float turns=multiply(argument,c.inversePi);
    const int32_t n=static_cast<int32_t>(turns);volatile float integer=static_cast<float>(n);
    const float r=subtract(argument,multiply(integer,c.pi)),square=multiply(r,r);
    const unsigned first=full?0:2;float p=add(multiply(square,c.sine[first]),c.sine[first+1]);
    for(unsigned i=first+2;i<6;++i)p=add(multiply(p,square),c.sine[i]);
    p=add(multiply(p,square),1);return {n,multiply(p,multiply(r,(uint32_t(n)&1)?-1.f:1.f))};
}
struct Host:bozo::ee::Host,bozo::port::Host {
    bool native=false;std::set<uint32_t> sites;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        sites.insert(pc);if(native){check(vp2s::nativeMathInstruction(r,c,rt,pc));return true;}
        if(pc==0x215a98 || pc==0x215afc || pc==0x215b60 || pc==0x215de4)c->r[9].ud[0]=0;
        if(pc==0x215aa0 || pc==0x215b04 || pc==0x215b68 || pc==0x215dec){
            const auto at=c->r[29].uw[0]+(pc==0x215aa0?0x140:pc==0x215b04?0x130:pc==0x215b68?0x120:0x110);
            uint32_t input[4];auto *p=rt->memory.hostPointer(at,16);std::memcpy(input,p,16);
            const auto result=referenceUnit(input);std::memcpy(p,result.data(),16);
        }
        if(pc==0x21639c || pc==0x216400 || pc==0x216464 || pc==0x2166e4)c->r[9].ud[0]=0;
        if(pc==0x2163a4 || pc==0x216408 || pc==0x21646c || pc==0x2166e8){
            const auto stack=c->r[29].uw[0],at=pc==0x2163a4?stack+0x130:pc==0x216408?stack+0x120:pc==0x21646c?stack+0x110:c->r[16].uw[0];
            uint32_t input[4];auto *p=rt->memory.hostPointer(at,16);std::memcpy(input,p,16);
            const auto result=referenceUnit(input);std::memcpy(p,result.data(),16);
        }
        if(pc==0x215118){const auto result=referenceThreeReduction({c->f[12],c->f[13],c->f[14]},read(rt));std::memcpy(rt->memory.hostPointer(c->r[29].uw[0],12),result.quadrants.data(),12);}
        if(pc==0x21536c){const auto result=referenceLinearQuaternion(rt,c);std::memcpy(rt->memory.hostPointer(c->r[18].uw[0],16),result.data(),16);}
        if(pc==0x214f5c || pc==0x214ff8 || pc==0x215050 || pc==0x2150dc){
            const auto table=readInverse(rt);const auto result=pc==0x215050?referenceInverseRoot(c->f[12],table):referenceInverse(c->f[12],table);
            std::memcpy(rt->memory.hostPointer(0x35da6c,4),&result,4);
        }
        if(pc==0x214014){const auto result=reference(c->f[12],read(rt));std::memcpy(rt->memory.hostPointer(0x35db08,8),result.quadrants.data(),8);}
        if(pc==0x2140f0){const auto result=reference(c->f[12],read(rt));std::memcpy(rt->memory.hostPointer(c->r[4].uw[0],16),result.value.data(),16);}
        if(pc==0x214338 || pc==0x2143c4){auto table=read(rt);table.halfPi=value<float>(rt,0x35da9c);const auto result=referenceShort(c->f[12],table);
            if(pc==0x214338)std::memcpy(rt->memory.hostPointer(0x35dadc,4),&result.quadrant,4);
            else std::memcpy(rt->memory.hostPointer(0x35da8c,4),&result.value,4);}
        if(pc==0x2144f0 || pc==0x21457c){const auto result=referenceShort(c->f[12],read(rt),true);
            if(pc==0x2144f0)std::memcpy(rt->memory.hostPointer(0x35dabc,4),&result.quadrant,4);
            else std::memcpy(rt->memory.hostPointer(0x35da8c,4),&result.value,4);}
        if(pc==0x214414 || pc==0x2144ac){const auto result=referenceShort(c->f[12],read(rt),true,true);
            if(pc==0x214414)std::memcpy(rt->memory.hostPointer(0x35dacc,4),&result.quadrant,4);
            else std::memcpy(rt->memory.hostPointer(0x35da8c,4),&result.value,4);}
        if(pc==0x21424c || pc==0x2142e4){auto table=read(rt);table.halfPi=value<float>(rt,0x35da9c);const auto result=referenceShort(c->f[12],table,false,true);
            if(pc==0x21424c)std::memcpy(rt->memory.hostPointer(0x35daec,4),&result.quadrant,4);
            else std::memcpy(rt->memory.hostPointer(0x35da8c,4),&result.value,4);}
        return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("library");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host old,now;now.native=true;bozo::ee::Dispatcher a(old),b(now);
    a.setFunctions(trig_original_function_table,trig_original_function_count,trig_original_resume_table,trig_original_resume_count);
    b.setFunctions(trig_adapted_function_table,trig_adapted_function_count,trig_adapted_resume_table,trig_adapted_resume_count);
    a.runtime().port=&old;b.runtime().port=&now;
    R5900Context init{};init.r[31].uw[0]=0x170000;
    a.runtime().functions.generated(0x358490)(a.runtime().memory.flat(),&init,&a.runtime());const auto originalCoefficients=read(&a.runtime());
    const unsigned saved=_mm_getcsr();unsigned cases=0;uint32_t random=0x213fd0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO}){
        for(unsigned seed=0;seed<96;++seed){
            auto *x=a.runtime().memory.flat(),*y=b.runtime().memory.flat();std::memset(x,0x65,bozo::kFlatBytes);
            init={};init.r[31].uw[0]=0x170000;a.runtime().functions.generated(0x358490)(x,&init,&a.runtime());
            std::memcpy(y,x,bozo::kFlatBytes);
            float angle=(float(int(seed%32)-16)*originalCoefficients.halfPi);
            if(seed>=32 && seed<64)angle=std::nextafter(angle,seed&1?std::numeric_limits<float>::infinity():-std::numeric_limits<float>::infinity());
            const uint32_t angleBits=std::bit_cast<uint32_t>(angle);
            // DAZ can make floating classification treat a subnormal as zero.
            // Choose the qualified boundary from its stored bits instead.
            if(!(angleBits&0x7f800000) && (angleBits&0x7fffff))angle=std::bit_cast<float>((angleBits&0x80000000)|0x00800000u);
            if(seed>=64){random=random*1664525u+1013904223u;angle=float(int32_t(random)%100000)/1024;}
            if(seed==0)angle=std::bit_cast<float>(0x80000000u);
            const uint32_t destinations[4]={0x180000,0x70003ff0,0x35da80,0x35db00};const auto to=destinations[seed%4];
            R5900Context p{},q{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=0x1234000000000000ull+seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
            std::memset(p.vu0_vf,0x4b,sizeof(p.vu0_vf));std::memset(&p.vu0_acc,0x5c,sizeof(p.vu0_acc));
            p.r[4].ud[0]=to;p.r[31].ud[0]=0x170000;p.f[12]=angle;q=p;
            const unsigned control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);
            _mm_setcsr(control);a.runtime().functions.generated(0x213fd0)(x,&p,&a.runtime());const unsigned expectedControl=_mm_getcsr();
            _mm_setcsr(control);
            try{b.runtime().functions.generated(0x213fd0)(y,&q,&b.runtime());}
            catch(const std::exception &e){std::fprintf(stderr,"trig fixture seed=%u mode=%x angle=%08x: %s\n",seed,mode,std::bit_cast<uint32_t>(angle),e.what());throw;}
            check(_mm_getcsr()==expectedControl);check(!std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x,y,bozo::kFlatBytes));
            if(to!=0x35db00){check(value<uint32_t>(&b.runtime(),0x35db00)==0x65656565 && value<uint32_t>(&b.runtime(),0x35db04)==0x65656565);}
            ++cases;
        }
        // Many arithmetic cases without repeatedly scanning whole RAM; this
        // compares independent scalar equations, not std::sin substitutions.
        for(unsigned i=0;i<16384;++i){random=random*1664525u+1013904223u;const float angle=float(int32_t(random)%1000000)/8192;
            const auto expected=reference(angle,originalCoefficients);const auto actual=vp2s::sineCosine(angle,originalCoefficients);
            check(!std::memcmp(expected.value.data(),actual.data(),16));
            check(std::abs(actual[2]-std::cos(angle))<0.0002f && std::abs(actual[3]-std::sin(angle))<0.0002f);
        }
    }
    _mm_setcsr(saved);check(old.sites==now.sites && now.sites.size()==30);
    unsigned shortCases=0;
    for(const uint32_t entry:{0x214300u,0x2144c0u,0x2143e0u,0x214210u}){
    const bool sine=entry!=0x214300 && entry!=0x214210,full=entry==0x2143e0 || entry==0x214210;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO}){
        for(unsigned seed=0;seed<192;++seed){
            auto *x=a.runtime().memory.flat(),*y=b.runtime().memory.flat();std::memset(x,0x65,bozo::kFlatBytes);
            R5900Context init{};init.r[31].uw[0]=0x170000;a.runtime().functions.generated(0x358490)(x,&init,&a.runtime());
            if(entry==0x214210 && seed%8==4){const float distinctHalfPi=.25f;std::memcpy(a.runtime().memory.hostPointer(0x35da9c,4),&distinctHalfPi,4);}
            const uint32_t oldSign=full && seed%8==5?0x7fc04567u:0u;std::memcpy(a.runtime().memory.hostPointer(0x35da88,4),&oldSign,4);
            std::memcpy(y,x,bozo::kFlatBytes);random=random*1664525u+1013904223u;
            float angle=float(int32_t(random)%100000)/1024;
            if(seed<64){angle=float(int(seed%16)-8)*originalCoefficients.pi;if(seed&16)angle=std::nextafter(angle,INFINITY);if(seed&32)angle=std::nextafter(angle,-INFINITY);}
            const auto bits=std::bit_cast<uint32_t>(angle);
            if((bits&0x7fffffff) && !(bits&0x7f800000))angle=std::bit_cast<float>((bits&0x80000000)|0x00800000u);
            if(seed==0)angle=std::bit_cast<float>(0x80000000u);
            R5900Context p{},q{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=0x1234000000000000ull+seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
            std::memset(p.vu0_vf,0x4b,sizeof(p.vu0_vf));std::memset(&p.vu0_acc,0x5c,sizeof(p.vu0_acc));p.r[31].uw[0]=0x170000;p.f[12]=angle;q=p;
            const unsigned control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);
            _mm_setcsr(control);a.runtime().functions.generated(entry)(x,&p,&a.runtime());const auto after=_mm_getcsr();
            _mm_setcsr(control);
            try{b.runtime().functions.generated(entry)(y,&q,&b.runtime());}
            catch(const std::exception &e){std::fprintf(stderr,"short trig entry=%x seed=%u mode=%x angle=%08x: %s\n",entry,seed,mode,std::bit_cast<uint32_t>(angle),e.what());throw;}
            check(_mm_getcsr()==after && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x,y,bozo::kFlatBytes));
            const uint32_t scratch=entry==0x214210?0x35dae0:full?0x35dac0:sine?0x35dab0:0x35dad0;
            check(value<uint32_t>(&b.runtime(),scratch)==0x65656565 && value<uint32_t>(&b.runtime(),scratch+4)==0x65656565 && value<uint32_t>(&b.runtime(),scratch+8)==0x65656565);
            check(value<uint32_t>(&b.runtime(),0x35da88)==oldSign);++shortCases;
        }
        for(unsigned i=0;i<16384;++i){random=random*1664525u+1013904223u;const float angle=float(int32_t(random)%1000000)/8192;
            const auto expected=referenceShort(angle,originalCoefficients,sine,full),actual=full?(sine?vp2s::fullSine(angle,originalCoefficients):vp2s::fullCosine(angle,originalCoefficients)):sine?vp2s::shortSine(angle,originalCoefficients):vp2s::shortCosine(angle,originalCoefficients);
            check(expected.quadrant==actual.quadrant && std::bit_cast<uint32_t>(expected.value)==std::bit_cast<uint32_t>(actual.value));
            check(std::abs(actual.value-(sine?std::sin(angle):std::cos(angle)))<0.008f);
        }
    }
    }
    _mm_setcsr(saved);check(old.sites==now.sites && now.sites.size()==132);
    unsigned inverseCases=0;double inverseError=0;
    const uint32_t edges[]={0,0x80000000,1,0x80000001,0x00800000,0x80800000,0x23000000,0xa3000000,0x23000001,0xa3000001,
        0x3effffff,0xbeffffff,0x3f000000,0xbf000000,0x3f000001,0xbf000001,0x3f7fffff,0xbf7fffff,
        0x3f800000,0xbf800000,0x3f800001,0xbf800001,0x7f7fffff,0xff7fffff,0x7f800000,0xff800000,0x7fc01234,0xffc01234,0x7f801234,0xff801234};
    vp2s::InverseCosineCoefficients inverseTable{};
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO}){
        for(unsigned seed=0;seed<256;++seed){
            auto *x=a.runtime().memory.flat(),*y=b.runtime().memory.flat();std::memset(x,0x65,bozo::kFlatBytes);
            init={};init.r[31].uw[0]=0x170000;a.runtime().functions.generated(0x358490)(x,&init,&a.runtime());inverseTable=readInverse(&a.runtime());
            random=random*1664525u+1013904223u;uint32_t bits;
            if(seed<std::size(edges))bits=edges[seed];else bits=(random&0x80000000u)|(0x23000001u+random%(0x3f800000u-0x23000001u));
            if(seed>=64 && seed<128)bits=(seed&1?0x80000000u:0u)|(0x3f000000u+(random%0x800000u));
            const auto angle=std::bit_cast<float>(bits);std::memcpy(y,x,bozo::kFlatBytes);
            R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=0x1234000000000000ull+seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
            std::memset(p.vu0_vf,0x4d,sizeof(p.vu0_vf));std::memset(&p.vu0_acc,0x5e,sizeof(p.vu0_acc));p.vu0_status=0x1234;p.r[31].uw[0]=0x170000;p.r[29].ud[0]=seed&2?0x70003fd0:0x1fff000;p.f[12]=angle;auto q=p;
            const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);_mm_setcsr(control);
            a.runtime().functions.generated(0x214e40)(x,&p,&a.runtime());const auto after=_mm_getcsr();_mm_setcsr(control);
            try{b.runtime().functions.generated(0x214e40)(y,&q,&b.runtime());}
            catch(const std::exception &e){std::fprintf(stderr,"inverse cosine seed=%u mode=%x angle=%08x: %s\n",seed,mode,bits,e.what());throw;}
            check(_mm_getcsr()==after && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x,y,bozo::kFlatBytes));
            check(!std::memcmp(b.runtime().memory.hostPointer(0x35da30,60),inverseTable.values.data(),60));
            if(bits>=0x3f000000 && bits<0x3f800000)check(value<uint32_t>(&b.runtime(),q.r[29].uw[0]-8)==(value<uint32_t>(&b.runtime(),q.r[29].uw[0]-4)&0xfffff000u));
            if((bits&0x7fffffff)<=0x3f800000){const double error=std::abs(double(q.f[0])-double(std::acos(angle)));inverseError=std::max(inverseError,error);check(error<0.000001);}
            ++inverseCases;
        }
        _mm_setcsr((saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode);
        for(unsigned i=0;i<16384;++i){random=random*1664525u+1013904223u;const float angle=float(int32_t(random)%1000000)/1000000.f;
            const auto expected=referenceInverse(angle,inverseTable),actual=vp2s::inverseCosine(angle,inverseTable);check(std::bit_cast<uint32_t>(expected)==std::bit_cast<uint32_t>(actual));
            const double error=std::abs(double(actual)-double(std::acos(angle)));inverseError=std::max(inverseError,error);check(error<0.000001);
        }
    }
    _mm_setcsr(saved);check(old.sites==now.sites && now.sites.size()==236);
    unsigned linearCallers=0,negativeCallers=0,linearRecords=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto *x=a.runtime().memory.flat(),*y=b.runtime().memory.flat();std::memset(x,0x65,bozo::kFlatBytes);
        init={};init.r[31].uw[0]=0x170000;a.runtime().functions.generated(0x358490)(x,&init,&a.runtime());
        const uint32_t stack=seed&1?0x70003e00:0x1fff000,first=seed&2?0x70000100:0x180000,second=0x190000;
        const uint32_t destination=seed%4==0?first:seed%4==1?second:seed%4==2?stack-0x10:0x1a0000;
        float aa[4],bb[4];const double offset=double(seed%16)*0.0001;
        const double aX=seed&8?0.2:0,aY=seed&8?0.1:0,aZ=seed&8?0.05:0;
        aa[0]=float(aX);aa[1]=float(aY);aa[2]=float(aZ);aa[3]=float(std::sqrt(1-aX*aX-aY*aY-aZ*aZ));
        const double bX=aX+offset,bY=aY+offset*0.5,bZ=aZ+offset*0.25;
        bb[0]=float(bX);bb[1]=float(bY);bb[2]=float(bZ);bb[3]=float(std::sqrt(1-bX*bX-bY*bY-bZ*bZ));
        if(seed&4)for(auto &v:bb)v=-v;
        std::memcpy(a.runtime().memory.hostPointer(first,16),aa,16);std::memcpy(a.runtime().memory.hostPointer(second,16),bb,16);std::memcpy(y,x,bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=0x1234000000000000ull+seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x4f,sizeof(p.vu0_vf));std::memset(&p.vu0_acc,0x60,sizeof(p.vu0_acc));p.vu0_status=0x1234;p.r[4].ud[0]=destination;p.r[5].ud[0]=first;p.r[6].ud[0]=second;p.r[29].ud[0]=stack;p.r[31].uw[0]=0x170000;
        const float parameters[]={0,1,0.5f,0.25f,-0.25f,1.25f,0.1f,0.33333334f};p.f[12]=parameters[seed%8];auto q=p;
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);_mm_setcsr(control);
        a.runtime().functions.generated(0x2151d0)(x,&p,&a.runtime());const auto after=_mm_getcsr();_mm_setcsr(control);
        try{b.runtime().functions.generated(0x2151d0)(y,&q,&b.runtime());}
        catch(const std::exception &e){std::fprintf(stderr,"linear quaternion seed=%u mode=%x: %s\n",seed,mode,e.what());throw;}
        check(_mm_getcsr()==after && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x,y,bozo::kFlatBytes));
        if(seed&4)++negativeCallers;++linearCallers;
    }
    check(negativeCallers==256 && old.sites==now.sites && now.sites.size()==255);
    unsigned nonlinearCallers=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto *x=a.runtime().memory.flat(),*y=b.runtime().memory.flat();std::memset(x,0x65,bozo::kFlatBytes);
        init={};init.r[31].uw[0]=0x170000;a.runtime().functions.generated(0x358490)(x,&init,&a.runtime());
        const uint32_t stack=seed&1?0x70003e00:0x1fff000,first=seed&2?0x70000100:0x180000,second=0x190000;
        const uint32_t destination=seed%4==0?first:seed%4==1?second:seed%4==2?stack-0x10:0x1a0000;
        float aa[4]={0,0,0,1},bb[4]={0.6f,0,0,0.8f};
        if(seed&8){aa[0]=0.2f;aa[1]=0.1f;aa[2]=0.05f;aa[3]=float(std::sqrt(1-.2*.2-.1*.1-.05*.05));bb[0]=-.3f;bb[1]=.4f;bb[2]=.1f;bb[3]=float(std::sqrt(.74));}
        if(seed%16==0){bb[0]=1;bb[1]=bb[2]=bb[3]=0;}
        if(seed&4)for(auto &v:bb)v=-v;
        std::memcpy(a.runtime().memory.hostPointer(first,16),aa,16);std::memcpy(a.runtime().memory.hostPointer(second,16),bb,16);std::memcpy(y,x,bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=0x1234000000000000ull+seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x53,sizeof(p.vu0_vf));std::memset(&p.vu0_acc,0x64,sizeof(p.vu0_acc));p.vu0_status=0x1234;p.r[4].ud[0]=destination;p.r[5].ud[0]=first;p.r[6].ud[0]=second;p.r[29].ud[0]=stack;p.r[31].uw[0]=0x170000;
        const float parameters[]={0,1,.5f,.25f,-.25f,1.25f,.1f,3.f};p.f[12]=parameters[seed%8];auto q=p;
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);_mm_setcsr(control);
        a.runtime().functions.generated(0x2151d0)(x,&p,&a.runtime());const auto after=_mm_getcsr();_mm_setcsr(control);
        try{b.runtime().functions.generated(0x2151d0)(y,&q,&b.runtime());}
        catch(const std::exception &e){std::fprintf(stderr,"nonlinear quaternion seed=%u mode=%x: %s\n",seed,mode,e.what());throw;}
        check(_mm_getcsr()==after && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x,y,bozo::kFlatBytes));++nonlinearCallers;
    }
    check(old.sites==now.sites && now.sites.size()==279);
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto *x=a.runtime().memory.flat(),*y=b.runtime().memory.flat();std::memset(x,0x65,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1ffe000,first=seed&2?0x70000100:0x180000,second=seed%8==3?first:0x190000;
        const uint32_t destination=seed%4==0?first:seed%4==1?second:seed%4==2?stack+0x60:0x1a0000;
        uint32_t aa[4],bb[4];for(auto *vector:{aa,bb})for(unsigned i=0;i<4;++i){random=random*1664525u+1013904223u;vector[i]=(random&0x807fffff)|((104+random%26)<<23);}
        if(seed%16==0)for(unsigned i=0;i<4;++i)aa[i]=bb[i]=0;
        if(seed%16==8)for(unsigned i=0;i<4;++i)aa[i]=bb[i]=0x00800000;
        std::memcpy(a.runtime().memory.hostPointer(first,16),aa,16);std::memcpy(a.runtime().memory.hostPointer(second,16),bb,16);
        const float weights[4]={0.5f,0.5f,std::bit_cast<float>(0xff801234u),std::bit_cast<float>(0x7fc05678u)};
        std::memcpy(a.runtime().memory.hostPointer(stack+0x60,16),weights,16);std::memcpy(y,x,bozo::kFlatBytes);
        R5900Context p{};std::memset(p.vu0_vf,0x51,sizeof(p.vu0_vf));std::memset(&p.vu0_acc,0x62,sizeof(p.vu0_acc));p.vu0_status=0x1234;
        p.pc=seed&4?0x215274:0x215260;p.r[16].ud[0]=second;p.r[17].ud[0]=first;p.r[18].ud[0]=destination;p.r[19].ud[0]=seed&4?1:0;p.r[29].ud[0]=stack;p.f[0]=weights[0];p.f[22]=weights[1];auto q=p;
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode;_mm_setcsr(control);
        check(old.nativeInstruction(x,&p,&a.runtime(),0x21536c));const auto after=_mm_getcsr();_mm_setcsr(control);
        check(now.nativeInstruction(y,&q,&b.runtime(),0x21536c));check(_mm_getcsr()==after);check(!std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x,y,bozo::kFlatBytes));
        if(seed%16==0)for(unsigned i=0;i<4;++i)check((value<uint32_t>(&b.runtime(),destination+i*4)&0x7fffffff)==0);++linearRecords;
    }
    _mm_setcsr(saved);
    {auto &rt=b.runtime();R5900Context c{};c.pc=0x215324;c.r[29].uw[0]=0x1ffe000;const auto before=c;std::vector<uint8_t> ram(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x21536c);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));
        c.pc=0x215260;c.r[16].ud[0]=0x190000;c.r[17].ud[0]=0x180000;c.r[18].ud[0]=0x1a0000;c.f[0]=c.f[22]=0.5f;
        float weights[4]={0.5f,0.5f,0,0},point[4]={0,0,0,1};std::memcpy(rt.memory.hostPointer(0x1ffe060,16),weights,16);
        std::memcpy(rt.memory.hostPointer(0x180000,16),point,16);std::memcpy(rt.memory.hostPointer(0x190000,16),point,16);
        auto rejectLinear=[&]{const auto savedContext=c;std::vector<uint8_t> savedRam(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x21536c);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&savedContext,&c,sizeof(c)) && !std::memcmp(savedRam.data(),rt.memory.flat(),savedRam.size()));};
        c.r[19].ud[0]=2;rejectLinear();c.r[19].ud[0]=0;c.f[0]=0.25f;rejectLinear();c.f[0]=0.5f;
        uint32_t invalid=0x7fc00000;std::memcpy(rt.memory.hostPointer(0x18000c,4),&invalid,4);rejectLinear();invalid=0x7f7fffff;std::memcpy(rt.memory.hostPointer(0x18000c,4),&invalid,4);rejectLinear();
    }
    {auto bad=inverseTable;bad.values[13]=0.25f;bool rejected=false;try{vp2s::inverseCosine(0.25f,bad);}catch(const std::runtime_error &){rejected=true;}check(rejected);
        bad=inverseTable;bad.values[8]=std::numeric_limits<float>::infinity();rejected=false;try{vp2s::inverseCosine(0.25f,bad);}catch(const std::runtime_error &){rejected=true;}check(rejected);
        bad=inverseTable;for(unsigned i=9;i<13;++i)bad.values[i]=0;bad.values[9]=-4;rejected=false;try{vp2s::inverseCosine(0.5f,bad);}catch(const std::runtime_error &){rejected=true;}check(rejected);
        R5900Context c{};c.pc=0x214e40;c.r[29].uw[0]=0x35da40;const auto before=c;std::vector<uint8_t> ram(b.runtime().memory.flat(),b.runtime().memory.flat()+bozo::kFlatBytes);
        rejected=false;try{vp2s::nativeTrigInstruction(b.runtime().memory.flat(),&c,&b.runtime(),0x214ea8);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),b.runtime().memory.flat(),ram.size()));
    }
    unsigned threeSineCallers=0;double threeSineError=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO}){
        for(unsigned seed=0;seed<96;++seed){
            auto *x=a.runtime().memory.flat(),*y=b.runtime().memory.flat();std::memset(x,0x65,bozo::kFlatBytes);
            init={};init.r[31].uw[0]=0x170000;a.runtime().functions.generated(0x358490)(x,&init,&a.runtime());
            const uint32_t stack=seed&1?0x70003e00:0x1fff000;std::array<float,3> angles;
            for(unsigned i=0;i<3;++i){random=random*1664525u+1013904223u;angles[i]=seed<64?float(int((seed+i)%16)-8)*originalCoefficients.pi:float(int32_t(random)%32000)/1024;
                if(seed&16)angles[i]=std::nextafter(angles[i],INFINITY);if(seed&32)angles[i]=std::nextafter(angles[i],-INFINITY);
                const auto bits=std::bit_cast<uint32_t>(angles[i]);if((bits&0x7fffffff) && !(bits&0x7f800000))angles[i]=std::bit_cast<float>((bits&0x80000000)|0x00800000u);}
            if(seed==0)angles[0]=std::bit_cast<float>(0x80000000u);std::memcpy(y,x,bozo::kFlatBytes);
            R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=0x1234000000000000ull+seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
            std::memset(p.vu0_vf,0x55,sizeof(p.vu0_vf));std::memset(&p.vu0_acc,0x66,sizeof(p.vu0_acc));p.vu0_status=0x1234;
            p.r[16].ud[0]=0x180000;p.r[17].ud[0]=0x190000;p.r[18].ud[0]=0x1a0000;p.r[29].ud[0]=stack;p.r[31].uw[0]=seed&1?0x2152ec:0x215324;
            for(unsigned i=0;i<3;++i)p.f[12+i]=angles[i];auto q=p;const auto originalR9=p.r[9];
            const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);_mm_setcsr(control);
            a.runtime().functions.generated(0x2150f0)(x,&p,&a.runtime());const auto after=_mm_getcsr();_mm_setcsr(control);
            b.runtime().functions.generated(0x2150f0)(y,&q,&b.runtime());check(_mm_getcsr()==after && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x,y,bozo::kFlatBytes));
            check(!std::memcmp(&q.r[9],&originalR9,sizeof(originalR9)) && value<uint32_t>(&b.runtime(),stack-0x14)==0x65656565 && value<uint32_t>(&b.runtime(),stack-4)==0x65656565);
            const auto reduction=referenceThreeReduction(angles,originalCoefficients);
            for(unsigned i=0;i<3;++i){check(value<uint32_t>(&b.runtime(),stack-0x20+i*4)==uint32_t(reduction.quadrants[i]));
                check(value<uint32_t>(&b.runtime(),stack-0x10+i*4)==((uint32_t(reduction.quadrants[i])&1)?0xbf800000u:0x3f800000u));}
            ++threeSineCallers;
        }
        _mm_setcsr((saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode);
        for(unsigned i=0;i<16384;++i){std::array<float,3> angles;for(auto &angle:angles){random=random*1664525u+1013904223u;angle=float(int32_t(random)%1000000)/1000000.f*originalCoefficients.halfPi;}
            const auto expected=referenceThreeSines(angles,originalCoefficients),actual=vp2s::threeSines(angles,originalCoefficients);check(!std::memcmp(expected.data(),actual.data(),12));
            for(unsigned j=0;j<3;++j){const auto error=std::abs(double(actual[j])-double(std::sin(angles[j])));threeSineError=std::max(threeSineError,error);check(error<0.000001);}
        }
    }
    _mm_setcsr(saved);check(old.sites==now.sites && now.sites.size()==279);
    unsigned eulerCallers=0;
    for(uint32_t entry:{0x2162c0u,0x2159c0u}){
    const bool positive=entry==0x2159c0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto *x=a.runtime().memory.flat(),*y=b.runtime().memory.flat();std::memset(x,0x65,bozo::kFlatBytes);
        init={};init.r[31].uw[0]=0x170000;a.runtime().functions.generated(0x358490)(x,&init,&a.runtime());
        const uint32_t top=seed&1?0x70003f00:0x1fff000,frame=top-(positive?0x180:0x170),source=0x180000;
        const uint32_t destination=seed%4==0?source:seed%4==1?frame+0x130:seed%4==2?frame+0x70:0x190000;
        float angles[4]={0,0,0,0};
        if(seed>=4)for(auto &angle:angles){random=random*1664525u+1013904223u;angle=float(int32_t(random)%32000)/8192;}
        else if(seed)angles[seed-1]=0.5f;
        std::memcpy(a.runtime().memory.hostPointer(source,16),angles,16);std::memcpy(y,x,bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=0x1234000000000000ull+seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];p.f[i]=float(i)/8;}
        std::memset(p.vu0_vf,0x56,sizeof(p.vu0_vf));std::memset(&p.vu0_acc,0x67,sizeof(p.vu0_acc));p.vu0_status=0x1234;
        p.r[4].ud[0]=destination;p.r[5].ud[0]=source;p.r[29].ud[0]=top;p.r[31].ud[0]=0x170000;auto q=p;
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);_mm_setcsr(control);
        a.runtime().functions.generated(entry)(x,&p,&a.runtime());const auto after=_mm_getcsr();_mm_setcsr(control);
        try{b.runtime().functions.generated(entry)(y,&q,&b.runtime());}
        catch(const std::exception &e){std::fprintf(stderr,"Euler quaternion entry=%x seed=%u mode=%x: %s\n",entry,seed,mode,e.what());throw;}
        check(_mm_getcsr()==after && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x,y,bozo::kFlatBytes));
        const auto result=value<std::array<float,4>>(&b.runtime(),destination);double norm=0;for(auto v:result)norm+=double(v)*v;check(std::abs(norm-1)<0.000002);
        if(seed<4){for(unsigned i=0;i<3;++i)if(i+1!=seed)check(result[i]==0);
            if(seed)check((!positive && seed<3?result[seed-1]<0:result[seed-1]>0));else check(result[3]==1);}
        ++eulerCallers;
    }
    }
    _mm_setcsr(saved);check(old.sites==now.sites && now.sites.size()==311);
    for(bool positive:{false,true}){auto &rt=b.runtime();R5900Context c{};const uint32_t entry=positive?0x215a58:0x21635c,site=positive?0x215aa0:0x2163a4,at=positive?0x1ffe140:0x1ffe130;
        c.pc=entry;c.r[29].uw[0]=0x1ffe000;c.r[2].uw[0]=at;
        const uint32_t point[4]={0,0,0,0x3f800000};std::memcpy(rt.memory.hostPointer(at,16),point,16);
        auto reject=[&]{const auto before=c;std::vector<uint8_t> ram(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool failure=false;
            try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,site);}catch(const std::runtime_error &){failure=true;}
            check(failure && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));};
        c.pc=0x170000;reject();c.pc=entry;c.r[2].uw[0]+=16;reject();c.r[2].uw[0]-=16;
        for(uint32_t bad:{1u,0x7fc01234u,0x7f7fffffu}){std::memcpy(rt.memory.hostPointer(at,4),&bad,4);reject();}
    }
    {auto bad=originalCoefficients;bad.sine[0]=0;bool rejected=false;try{vp2s::threeSines({0.1f,0.2f,0.3f},bad);}catch(const std::runtime_error &){rejected=true;}check(rejected);
        bad=originalCoefficients;bad.halfPi=std::numeric_limits<float>::quiet_NaN();const auto ignored=vp2s::threeSines({0.1f,0.2f,0.3f},bad),normal=vp2s::threeSines({0.1f,0.2f,0.3f},originalCoefficients);check(!std::memcmp(ignored.data(),normal.data(),12));
        for(float invalid:{std::numeric_limits<float>::infinity(),std::bit_cast<float>(1u),std::numeric_limits<float>::max()}){rejected=false;try{vp2s::threeSines({invalid,0,0},originalCoefficients);}catch(const std::runtime_error &){rejected=true;}check(rejected);}
        auto &rt=b.runtime();R5900Context c{};c.pc=0x2150f0;c.r[31].uw[0]=0x215324;c.r[29].ud[0]=0x1ffe000;c.r[16].ud[0]=0x180000;c.r[17].ud[0]=0x190000;c.r[18].ud[0]=0x1a0000;
        auto reject=[&]{const auto savedContext=c;std::vector<uint8_t> ram(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool failure=false;
            try{vp2s::nativeTrigInstruction(rt.memory.flat(),&c,&rt,0x215104);}catch(const std::runtime_error &){failure=true;}
            check(failure && !std::memcmp(&savedContext,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));};
        c.r[18].ud[0]=c.r[29].ud[0]+0x10;reject();c.r[18].ud[0]=0x1a0000;c.r[29].ud[0]=0x35da70;reject();c.r[29].ud[0]=0x1ffe000;c.r[31].uw[0]=0x170000;reject();
    }
    bool rejected=false;try{vp2s::sineCosine(std::numeric_limits<float>::infinity(),originalCoefficients);}catch(const std::runtime_error &){rejected=true;}check(rejected);
    rejected=false;try{vp2s::sineCosine(std::bit_cast<float>(1u),originalCoefficients);}catch(const std::runtime_error &){rejected=true;}check(rejected);
    rejected=false;try{vp2s::shortCosine(std::bit_cast<float>(1u),originalCoefficients);}catch(const std::runtime_error &){rejected=true;}check(rejected);
    rejected=false;try{vp2s::shortSine(std::bit_cast<float>(1u),originalCoefficients);}catch(const std::runtime_error &){rejected=true;}check(rejected);
    rejected=false;try{vp2s::fullSine(std::bit_cast<float>(1u),originalCoefficients);}catch(const std::runtime_error &){rejected=true;}check(rejected);
    rejected=false;try{vp2s::fullCosine(std::bit_cast<float>(1u),originalCoefficients);}catch(const std::runtime_error &){rejected=true;}check(rejected);
    {auto table=originalCoefficients;table.halfPi=std::bit_cast<float>(0x7fc05678u);check(vp2s::fullSine(0.f,table).value==0.f);
        table.sine[0]=std::numeric_limits<float>::infinity();rejected=false;try{vp2s::fullSine(.25f,table);}catch(const std::runtime_error &){rejected=true;}check(rejected);}
    std::printf("%u sine/cosine + %u scalar cosine/sines + %u inverse-cosine + %u linear-quaternion (%u negative-dot) + %u nonlinear-quaternion callers + %u blend records + %u three-sine + %u Euler-quaternion callers;458752 polynomial record comparisons,311 sites; inverse-cosine max error %.9g,three-sine max error %.9g; CPU/RAM/FP, signs, endpoints and split-root stores passed\n",cases,shortCases,inverseCases,linearCallers,negativeCallers,nonlinearCallers,linearRecords,threeSineCallers,eulerCallers,inverseError,threeSineError);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

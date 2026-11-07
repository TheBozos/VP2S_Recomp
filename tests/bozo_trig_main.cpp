#include "native_trig.h"
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
struct Host:bozo::ee::Host,bozo::port::Host {
    bool native=false;std::set<uint32_t> sites;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        sites.insert(pc);if(native){check(vp2s::nativeTrigInstruction(r,c,rt,pc));return true;}
        if(pc==0x214014){const auto result=reference(c->f[12],read(rt));std::memcpy(rt->memory.hostPointer(0x35db08,8),result.quadrants.data(),8);}
        if(pc==0x2140f0){const auto result=reference(c->f[12],read(rt));std::memcpy(rt->memory.hostPointer(c->r[4].uw[0],16),result.value.data(),16);}
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
    bool rejected=false;try{vp2s::sineCosine(std::numeric_limits<float>::infinity(),originalCoefficients);}catch(const std::runtime_error &){rejected=true;}check(rejected);
    rejected=false;try{vp2s::sineCosine(std::bit_cast<float>(1u),originalCoefficients);}catch(const std::runtime_error &){rejected=true;}check(rejected);
    std::printf("%u generated sine/cosine CPU/RAM cases and 65536 independent scalar polynomial comparisons;30 sites, FP controls, signs, quadrant edges and aliases passed\n",cases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

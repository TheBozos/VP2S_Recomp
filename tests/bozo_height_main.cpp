// Complete original scalar height-table loop, optional update and bound stores.
#include "native_math.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "height_reference_functions.h"
#include "height_adapted_functions.h"
#include "terrain_reference_functions.h"
#include "terrain_adapted_functions.h"
#include "query_reference_functions.h"
#include "query_adapted_functions.h"
#include <bit>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <immintrin.h>
#include <source_location>
#include <stdexcept>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){
    if(!v)throw std::runtime_error("height bounds assertion at "+std::to_string(at.line()));
}
template<class T>void put(Runtime &r,uint32_t at,T v){auto *p=r.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(p,&v,sizeof(v));}
uint32_t word(Runtime &r,uint32_t at){uint32_t v;std::memcpy(&v,r.memory.hostPointer(at,4),4);return v;}
int16_t half(Runtime &r,uint32_t at){int16_t v;std::memcpy(&v,r.memory.hostPointer(at,2),2);return v;}
uint32_t product(uint32_t a){unsigned e=(a>>23)&255;if(!e)return 0;uint64_t m=uint64_t((a&0x7fffff)|0x800000)*((a&0x7fffff)|0x800000);int exponent=2*int(e)-127;unsigned shift=23;
    if(m&(1ull<<47)){++exponent;++shift;}return exponent<=0?0:uint32_t(exponent<<23)|uint32_t((m>>shift)&0x7fffff);}
uint32_t add(uint32_t a,uint32_t b){if(a<b)std::swap(a,b);if(!b)return a;int e=(a>>23)&255;unsigned d=unsigned(e)-((b>>23)&255);
    uint64_t m=((a&0x7fffff)|0x800000)+(d<=23?(((b&0x7fffff)|0x800000)>>d):0);if(m>=0x1000000){m>>=1;++e;}return uint32_t(e<<23)|uint32_t(m&0x7fffff);}
uint32_t scale(Runtime &r,uint32_t object,unsigned axis=1){uint32_t sum=0;for(unsigned i=0;i<3;++i)sum=add(sum,product(word(r,object+0x90+axis*4+i*16)));
    if(!sum)return 0; // DAZ must not turn subnormal search candidates into zero roots.
    const double squared=std::bit_cast<float>(sum);uint32_t lo=0,hi=0x7f7fffff;
    while(lo<hi){auto mid=lo+(hi-lo+1)/2;const double v=std::bit_cast<float>(mid);if(v*v<=squared)lo=mid;else hi=mid-1;}return lo;}
uint32_t signedProduct(uint32_t a,uint32_t b){const auto sign=(a^b)&0x80000000u;unsigned ea=(a>>23)&255,eb=(b>>23)&255;if(!ea || !eb)return sign;
    uint64_t m=uint64_t((a&0x7fffff)|0x800000)*((b&0x7fffff)|0x800000);int e=int(ea)+int(eb)-127;unsigned shift=23;
    if(m&(1ull<<47)){++e;++shift;}check(e<255);return e<=0?sign:sign|uint32_t(e<<23)|uint32_t((m>>shift)&0x7fffff);}
uint32_t signedSum(uint32_t a,uint32_t b){if((a&0x7fffffff)<(b&0x7fffffff))std::swap(a,b);if(!(b&0x7fffffff))return (a&0x7fffffff)?a:(a&b&0x80000000u);
    int e=(a>>23)&255;unsigned d=unsigned(e)-((b>>23)&255);const int64_t large=(a&0x7fffff)|0x800000,small=d<=23?(((b&0x7fffff)|0x800000)>>d):0;
    const auto v=(a>>31?-large:large)+(b>>31?-small:small);if(!v)return 0;const auto sign=v<0?0x80000000u:0u;uint64_t m=v<0?-v:v;
    while(m>=0x1000000){m>>=1;++e;}while(m<0x800000){m<<=1;--e;}check(e<255);return e<=0?sign:sign|uint32_t(e<<23)|uint32_t(m&0x7fffff);}
void rowPointOracle(Runtime &r,uint32_t matrixAt,uint32_t pointAt,uint32_t destination){uint32_t matrix[4][4],point[4],out[4];std::memcpy(matrix,r.memory.hostPointer(matrixAt,64),64);std::memcpy(point,r.memory.hostPointer(pointAt,16),16);
    constexpr unsigned order[4][4]={{0,1,2,3},{1,0,2,3},{2,1,0,3},{3,1,2,0}};
    for(unsigned row=0;row<4;++row){uint32_t terms[4];for(unsigned i=0;i<4;++i)terms[i]=signedProduct(matrix[row][i],point[i]);
        out[row]=signedSum(signedSum(signedSum(terms[order[row][0]],terms[order[row][1]]),signedProduct(0x3f800000u,terms[order[row][2]])),signedProduct(0x3f800000u,terms[order[row][3]]));}
    std::memcpy(r.memory.hostPointer(destination,16),out,16);
}
// Independent finite float32 arithmetic for the inline query's caller rounding.
// Integer guard/sticky bits distinguish a tiny cancellation residual from zero.
uint64_t rightJam(uint64_t v,unsigned shift){if(!shift)return v;if(shift>=64)return v!=0;return (v>>shift)|((v&((uint64_t(1)<<shift)-1))!=0);}
uint32_t roundedValue(uint32_t sign,int e,uint64_t m){
    if(!m)return sign;
    while(m>=(uint64_t(1)<<56)){m=rightJam(m,1);++e;}
    while(m<(uint64_t(1)<<55)){m<<=1;--e;}
    if(e<=0)return sign;
    uint32_t value=uint32_t(m>>32),fraction=uint32_t(m);const auto mode=_mm_getcsr()&_MM_ROUND_MASK;
    const bool increment=mode==_MM_ROUND_NEAREST?(fraction>0x80000000u || (fraction==0x80000000u && (value&1))):
        mode==_MM_ROUND_DOWN?(sign && fraction):mode==_MM_ROUND_UP?(!sign && fraction):false;
    if(increment && ++value==0x1000000){value>>=1;++e;}check(e<255);return sign|uint32_t(e<<23)|(value&0x7fffff);
}
uint32_t roundedProduct(uint32_t a,uint32_t b){const auto sign=(a^b)&0x80000000u;const int ea=(a>>23)&255,eb=(b>>23)&255;
    if(!ea || !eb)return sign;return roundedValue(sign,ea+eb-127,uint64_t((a&0x7fffff)|0x800000)*((b&0x7fffff)|0x800000)<<9);}
uint32_t roundedSum(uint32_t a,uint32_t b){
    if((a&0x7fffffff)<(b&0x7fffffff))std::swap(a,b);
    if(!(b&0x7fffffff)){if(a&0x7fffffff)return a;return ((a&b)&0x80000000u)|((_mm_getcsr()&_MM_ROUND_MASK)==_MM_ROUND_DOWN?((a|b)&0x80000000u):0u);}
    int e=(a>>23)&255;const unsigned distance=unsigned(e)-((b>>23)&255);
    const uint64_t large=uint64_t((a&0x7fffff)|0x800000)<<32,small=rightJam(uint64_t((b&0x7fffff)|0x800000)<<32,distance);
    const auto magnitude=((a^b)&0x80000000u)?large-small:large+small;
    if(!magnitude)return (_mm_getcsr()&_MM_ROUND_MASK)==_MM_ROUND_DOWN?0x80000000u:0u;
    return roundedValue(a&0x80000000u,e,magnitude);
}
uint32_t roundedReciprocal(uint32_t denominator){if(!denominator)return 0x7f7fffff;uint32_t lo=0,hi=0x7f7fffff;const double d=std::bit_cast<float>(denominator);
    while(lo<hi){const auto mid=lo+(hi-lo+1)/2;const double v=std::bit_cast<float>(mid);if(v*d<=1.0)lo=mid;else hi=mid-1;}
    const auto mode=_mm_getcsr()&_MM_ROUND_MASK;const double lower=std::bit_cast<float>(lo);
    if(lower*d==1.0 || mode==_MM_ROUND_TOWARD_ZERO || mode==_MM_ROUND_DOWN)return lo;
    if(mode==_MM_ROUND_UP)return lo+1;
    const double middle=(lower+double(std::bit_cast<float>(lo+1)))*.5,comparison=middle*d;
    return lo+unsigned(comparison<1.0 || (comparison==1.0 && (lo&1)));
}
void queryDistanceOracle(Runtime &r,R5900Context *c){
    const auto object=c->r[16].uw[0],node=c->r[18].uw[0],query=c->r[17].uw[0];
    uint32_t center[4],origin[4],direction[4],matrix[4][4],difference[3],squares[3],dot[3];
    std::memcpy(center,r.memory.hostPointer(node+16,16),16);std::memcpy(origin,r.memory.hostPointer(query,16),16);
    std::memcpy(direction,r.memory.hostPointer(query+16,16),16);std::memcpy(matrix,r.memory.hostPointer(object+0x150,64),64);
    const auto sum3=[](const uint32_t v[3]){return roundedSum(roundedSum(roundedProduct(0x3f800000,v[0]),roundedProduct(0x3f800000,v[1])),roundedProduct(0x3f800000,v[2]));};
    for(unsigned i=0;i<3;++i){auto value=roundedProduct(matrix[0][i],center[0]);for(unsigned j=1;j<4;++j)value=roundedSum(value,roundedProduct(matrix[j][i],j==3?0x3f800000:center[j]));
        difference[i]=roundedSum(value,origin[i]^0x80000000u);squares[i]=roundedProduct(direction[i],direction[i]);dot[i]=roundedProduct(difference[i],direction[i]);}
    const auto denominator=sum3(squares),reciprocal=roundedReciprocal(denominator);
    const auto projection=sum3(dot),parameter=roundedProduct((projection&0x80000000u)?0u:projection,reciprocal);
    for(unsigned i=0;i<3;++i){const auto v=roundedSum(difference[i],roundedProduct(direction[i],parameter)^0x80000000u);squares[i]=roundedProduct(v,v);}
    const auto radius=roundedProduct(center[3],word(r,object+0x204));center[3]=roundedSum(sum3(squares),roundedProduct(radius,radius)^0x80000000u);
    std::memcpy(r.memory.hostPointer(c->r[29].uw[0]+0x50,16),center,16);
}
struct Host final:bozo::ee::Host,bozo::port::Host{
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("unexpected native library ownership");}
    bool reference=false;unsigned callbacks=0,getters=0;uint32_t matrices[2]{};
    bool nativeInstruction(uint8_t *ram,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(reference){if(pc==0x252f20){rowPointOracle(*rt,c->r[3].uw[0],c->r[17].uw[0],c->r[2].uw[0]);return true;}
            if(pc==0x1f717c){queryDistanceOracle(*rt,c);return true;}
            if(pc==0x1f70b4){for(unsigned axis=0;axis<3;++axis)put(*rt,c->r[29].uw[0]+0x40+axis*4,scale(*rt,c->r[16].uw[0],axis));return true;}
            if(pc==0x252ff8){put(*rt,c->r[29].uw[0]+0xb4,scale(*rt,c->r[16].uw[0]));return true;}
            if(pc==0x3e7f34 || pc==0x3e7ff8){rowPointOracle(*rt,c->r[2].uw[0],c->r[29].uw[0]+0x70,c->r[29].uw[0]+0x70);return true;}
            check(pc==0x3e5cc0);put(*rt,c->r[29].uw[0]+0x54,scale(*rt,c->r[18].uw[0]));return true;}
        check(vp2s::nativeMathInstruction(ram,c,rt,pc));return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        if(pc==0x174000){put<uint8_t>(d.runtime(),c->r[4].uw[0]+0x61,1);++callbacks;c->pc=c->r[31].uw[0];return;}
        if(pc==0x173000){check(c->r[31].uw[0]==(getters%2?0x3e7f98u:0x3e7ed8u));c->r[2].sd[0]=int32_t(matrices[getters++%2]);c->pc=c->r[31].uw[0];return;}
        check(pc==0x172000);const auto object=c->r[4].uw[0];
        // Real indirect update ABI, with a bounded matrix/translation producer.
        for(unsigned i=0;i<3;++i)put(d.runtime(),object+0x94+i*16,std::bit_cast<uint32_t>(i==1?2.f:0.f));
        put(d.runtime(),object+0xec,0x41200000u);++callbacks;c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("IO");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("IO");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host host;bozo::ee::Dispatcher a(host),b(host);
    a.setFunctions(height_reference_function_table,height_reference_function_count,height_reference_resume_table,height_reference_resume_count);
    b.setFunctions(height_adapted_function_table,height_adapted_function_count,height_adapted_resume_table,height_adapted_resume_count);
    for(auto *d:{&a,&b}){d->runtime().port=&host;d->bindLibrary(0x172000);d->bindLibrary(0x173000);}
    unsigned cases=0;uint32_t random=91;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x53,bozo::kFlatBytes);
        const uint32_t object=seed%3==0?0x70000100:0x180000,table=0x190000,out=seed%5==0?object+0x180:0x1a0004;
        const uint32_t stack=seed&1?0x70003f00:0x1fff000,count=seed%17;
        put(x,object,0x1b0000u);put(x,0x1b0064,0x172000u);put(x,object+0x250,table);put<uint8_t>(x,object+0x61,seed%4?1:0);
        put(x,table+8,count);put(x,table+12,1u);
        for(unsigned i=0;i<count;++i)put<int16_t>(x,table+0x20+i*2,i%5==0?0x7fff:int16_t(int(i)*19-int(seed)*2));
        for(unsigned row=0;row<3;++row)for(unsigned lane=0;lane<4;++lane){random=random*1664525u+1013904223u;
            uint32_t value=lane==1?std::bit_cast<uint32_t>(float(int(random%400)-200)/64.f):0x7f805678u;
            if(seed%8==0 && lane==1)value=row==1?0x3f800000u:0;
            if(seed%8==1 && lane==1)value=0x80000000u;
            if(seed%8==2 && lane==1)value=std::bit_cast<uint32_t>(1.e-30f);
            if(seed%8==3 && lane==1)value=std::bit_cast<uint32_t>(row==0?3.f:row==1?-4.f:0.f);
            put(x,object+0x90+row*16+lane*4,value);}
        put(x,object+0xdc,0x7f801234u);put(x,object+0xec,std::bit_cast<uint32_t>(float(int(seed)-64)));put(x,object+0xfc,0x7fc04567u);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}std::memset(p.vu0_vf,0x47,sizeof(p.vu0_vf));
        p.r[4].ud[0]=out;p.r[5].ud[0]=object;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040u))|mode|(seed&1?0x8040u:0u);_mm_setcsr(control);
        host.reference=true;x.functions.generated(0x3e5bc0)(x.memory.flat(),&p,&x);const auto expected=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;y.functions.generated(0x3e5bc0)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expected);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;
        if(std::memcmp(&p,&q,sizeof(p))){const auto *u=reinterpret_cast<const uint8_t *>(&p),*v=reinterpret_cast<const uint8_t *>(&q);
            for(unsigned i=0;i<sizeof(p);++i)if(u[i]!=v[i]){std::fprintf(stderr,"seed=%u mode=%x CPU byte=%u old=%02x new=%02x pc=%x/%x\n",seed,mode,i,u[i],v[i],p.pc,q.pc);break;}}
        check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));++cases;
        if(seed==8)check(half(y,out+4)==244 && half(y,out+6)==16); // update publishes scale 2, Y 10
        if(seed==1)check(half(y,out+4)==-63 && half(y,out+6)==-63);
    }
    // Rejections must leave every CPU/RAM byte unchanged at the numeric boundary.
    auto &r=b.runtime();R5900Context c{};c.r[18].ud[0]=0x180000;c.r[4].ud[0]=0x180090;c.r[19].ud[0]=0x190000;c.r[29].ud[0]=0x1fff000;c.r[3].ud[0]=0x1fff050;
    for(unsigned i=0;i<3;++i)put(r,0x180094+i*16,0u);
    const auto reject=[&]{const auto before=c;std::vector<uint8_t> memory(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
        try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x3e5cc0);}catch(const std::runtime_error &){bad=true;}
        check(bad && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(memory.data(),r.memory.flat(),memory.size()));};
    c.r[4].ud[0]+=16;reject();c.r[4].ud[0]-=16;c.r[3].ud[0]+=16;reject();c.r[3].ud[0]-=16;
    for(auto bits:{0x7fc00000u,1u,0x7f000000u}){put(r,0x180094,bits);reject();}put(r,0x180094,0u);
    c.r[19].ud[0]=0x1fff050;reject();c.r[19].ud[0]=0x190000;c.r[18].ud[0]=0x1fff000;c.r[4].ud[0]=0x1fff090;reject();
    unsigned corners=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x53,bozo::kFlatBytes);
        const uint32_t object=0x180000,descriptor=0x190000,stack=seed&1?0x70003e00:0x1fff000,index=seed%7;
        host.matrices[0]=seed%6==1?stack+0x70:object+0x90;
        host.matrices[1]=seed%6==2?stack+0x40:seed%3==0?host.matrices[0]:0x1c0000;
        if(seed==8)host.matrices[1]=host.matrices[0]; // exact example has no frame/source alias
        const uint32_t output=seed%9==0?host.matrices[0]:seed%9==1?stack+0x80:0x1d0000;
        for(auto matrix:host.matrices)for(unsigned row=0;row<4;++row)for(unsigned lane=0;lane<4;++lane){random=random*1664525u+1013904223u;
            float value=float(int(random%400)-200)/64.f;
            if(seed==8 || seed%11==0)value=row==lane?1.f:0.f;
            if(seed==8 && lane==3 && row<3)value=row==0?10.f:row==1?20.f:-5.f;
            if(seed%11==1)value=lane&1?-0.f:0.f;
            put(x,matrix+row*16+lane*4,std::bit_cast<uint32_t>(value));}
        put(x,object,0x1b0000u);put(x,0x1b0068,0x173000u);
        for(unsigned i=0;i<4;++i)put<int32_t>(x,descriptor+i*4,seed==8?std::array<int,4>{-3,7,4,-2}[i]:int(seed)*(i&1?-1:1)+int(i)*23-50);
        for(unsigned i=0;i<4;++i)put(x,stack+0x80+i*4,std::bit_cast<uint32_t>(i<2?-1000.f:1000.f));
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}std::memset(p.vu0_vf,0x47,sizeof(p.vu0_vf));
        p.r[16].ud[0]=descriptor;p.r[17].ud[0]=output-index*16;p.r[18].ud[0]=index;p.r[19].ud[0]=object;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040u))|mode|(seed&1?0x8040u:0u);_mm_setcsr(control);
        host.reference=true;host.getters=0;x.functions.generated(0x3e7e94)(x.memory.flat(),&p,&x);const auto expected=_mm_getcsr();check(host.getters==2);_mm_setcsr(control);
        host.reference=false;host.getters=0;y.functions.generated(0x3e7e94)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expected && host.getters==2);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        check(*y.memory.hostPointer(object+0x50,1)==1 && !*y.memory.hostPointer(object+0x61,1) && !*y.memory.hostPointer(object+0x51,1));
        if(seed==8){const float exact[4]={7,-7,14,2};for(unsigned i=0;i<4;++i){if(word(y,output+i*4)!=std::bit_cast<uint32_t>(exact[i]))std::fprintf(stderr,"exact corner lane=%u got=%g expected=%g\n",i,std::bit_cast<float>(word(y,output+i*4)),exact[i]);check(word(y,output+i*4)==std::bit_cast<uint32_t>(exact[i]));}}++corners;
    }
    {auto &r=b.runtime();R5900Context c{};c.r[29].ud[0]=0x1fff000;c.r[3].ud[0]=0x1fff070;c.r[2].ud[0]=0x180000;
        for(unsigned i=0;i<16;++i)put(r,0x180000+i*4,0u);for(unsigned i=0;i<4;++i)put(r,0x1fff070+i*4,0u);
        const auto reject=[&]{const auto before=c;std::vector<uint8_t> memory(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x3e7f34);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(memory.data(),r.memory.flat(),memory.size()));};
        c.r[3].ud[0]+=16;reject();c.r[3].ud[0]-=16;c.r[2].ud[0]+=4;reject();c.r[2].ud[0]-=4;
        for(auto at:{0x180000u,0x1fff070u})for(auto bits:{0x7fc00000u,1u,0x7f000000u}){put(r,at,bits);reject();put(r,at,0u);}
    }
    a.setFunctions(terrain_reference_function_table,terrain_reference_function_count,terrain_reference_resume_table,terrain_reference_resume_count);
    b.setFunctions(terrain_adapted_function_table,terrain_adapted_function_count,terrain_adapted_resume_table,terrain_adapted_resume_count);
    unsigned terrains=0,rejectedBounds=0,missingHeights=0,heights=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x53,bozo::kFlatBytes);
        const uint32_t object=0x180000,grid=0x190000,stack=seed&1?0x70003e00:0x1fff000,frame=stack-0xe0;
        const uint32_t point=seed%5==0?frame+0xd0:seed%5==1?object+0x210:0x1a0000;
        for(unsigned i=0;i<16;++i)put(x,object+0x210+i*4,std::bit_cast<uint32_t>(i%5?0.f:1.f));
        for(unsigned i=0;i<12;++i)put(x,object+0x90+i*4,(i%4==1)?std::bit_cast<uint32_t>(i/4==1?2.f:0.f):0x7f805678u);
        put(x,object+0xac,0x41200000u); // this caller uses the +0x90 matrix's Y translation
        put(x,object+0xdc,0x7fc01234u);put(x,object+0xec,0x41200000u);put(x,object+0xfc,0x7f805678u);
        put<uint8_t>(x,object+0x61,1);put(x,object+0x250,grid);
        for(unsigned i=0;i<8;++i)put<int32_t>(x,grid+i*4,std::array<int,8>{10,10,4,4,0,0,30,30}[i]);
        for(unsigned z=0;z<4;++z)for(unsigned xx=0;xx<4;++xx){int16_t value=int16_t(5+xx*2+z*3);
            if(seed%11==1 || (seed%11==2 && xx==0 && z==0) || (seed%11==3 && (xx+z)%2==0))value=0x7fff;
            put(x,grid+0x20+(z*4+xx)*2,value);}
        float sample[4]={float(5+(seed%3)*10),0,float(5+(seed%5)*5),1};
        if(seed%9==1)sample[0]=-1;else if(seed%9==2)sample[0]=30;else if(seed%9==3)sample[2]=30;
        if(seed==0){sample[0]=sample[2]=5;}
        std::memcpy(x.memory.hostPointer(point,16),sample,16);
        // Test source/output and inverse/source aliases without inventing an
        // inverse update. Cached matrices are ordinary records the routine owns.
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}std::memset(p.vu0_vf,0x47,sizeof(p.vu0_vf));
        p.r[4].ud[0]=object;p.r[5].ud[0]=point;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040u))|mode|(seed&1?0x8040u:0u);_mm_setcsr(control);
        host.reference=true;x.functions.generated(0x252e30)(x.memory.flat(),&p,&x);const auto expected=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;y.functions.generated(0x252e30)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expected);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        const bool miss=*y.memory.hostPointer(object+0x255,1)!=0;if(miss){if(q.fu[0]==0)++rejectedBounds;else ++missingHeights;}else ++heights;
        if(seed==0){if(miss || q.f[0]!=25.f)std::fprintf(stderr,"exact terrain got=%g missing=%u\n",q.f[0],unsigned(miss));check(!miss && q.f[0]==25.f);}++terrains;
    }
    check(rejectedBounds && missingHeights && heights);
    {auto &r=b.runtime();R5900Context c{};c.r[16].ud[0]=0x180000;c.r[17].ud[0]=0x1a0000;c.r[29].ud[0]=0x1fff000;c.r[2].ud[0]=0x1fff0b0;
        put(r,0x180250,0x190000u);put(r,0x190008,4u);put(r,0x19000c,4u);for(unsigned i=0;i<3;++i)put(r,0x180094+i*16,0u);
        const auto reject=[&]{const auto before=c;std::vector<uint8_t> memory(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x252ff8);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(memory.data(),r.memory.flat(),memory.size()));};
        c.r[2].ud[0]+=16;reject();c.r[2].ud[0]-=16;put(r,0x190008,0u);reject();put(r,0x190008,4u);
        for(auto bits:{0x7fc00000u,1u,0x7f000000u}){put(r,0x180094,bits);reject();}put(r,0x180094,0u);
        c.r[17].ud[0]=0x1fff0b0;reject();c.r[17].ud[0]=0x1a0000;put(r,0x180250,0x1fff090u);put(r,0x1fff098,4u);put(r,0x1fff09c,4u);reject();
    }
    a.setFunctions(query_reference_function_table,query_reference_function_count,query_reference_resume_table,query_reference_resume_count);
    b.setFunctions(query_adapted_function_table,query_adapted_function_count,query_adapted_resume_table,query_adapted_resume_count);
    for(auto *d:{&a,&b})d->bindLibrary(0x174000);
    unsigned queries=0,queryHits=0,queryMisses=0,querySkipped=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x53,bozo::kFlatBytes);
        const uint32_t header=0x180000,node=header+0x30,object=0x1a0000,query=0x1c0000,argument=0x170100,stack=seed&1?0x70003e00:0x1fff000,frame=stack-0x100;
        put(x,argument,header);put(x,header+12,0x190000u);put(x,node,0u);put(x,node+4,seed%13==12?0u:object);
        put<uint8_t>(x,node+9,seed%13==11?0:1);put<uint8_t>(x,0x314020,1);put<int16_t>(x,node+14,-1);
        put(x,object,0x1b0000u);put(x,0x1b0064,0x174000u);put<uint8_t>(x,object+0x61,seed%3?1:0);put<uint8_t>(x,object+0x20e,(seed%4)*0x10+(seed%4>=2?0x20:0));
        for(unsigned i=0;i<16;++i){const auto value=std::bit_cast<uint32_t>(i%5?0.f:1.f);put(x,object+0xd0+i*4,value);put(x,object+0x150+i*4,value);if(i<12)put(x,object+0x90+i*4,value);}
        put(x,object+0x204,0x3f800000u);
        float sphere[4]={0,0,0,1},origin[4]={-5,0,0,std::bit_cast<float>(0x7f805678u)},direction[4]={10,0,0,std::bit_cast<float>(0x7fc01234u)};
        switch(seed%7){case 1:sphere[1]=3;break;case 2:sphere[0]=20;break;case 3:sphere[0]=-20;break;case 4:direction[0]=0;break;case 5:direction[0]=.125f;sphere[1]=.5f;break;case 6:sphere[3]=0;break;}
        if(seed%7<3){origin[0]=-4;direction[0]=8;} // exactly representable projection examples
        std::memcpy(x.memory.hostPointer(node+16,16),sphere,16);std::memcpy(x.memory.hostPointer(query,16),origin,16);std::memcpy(x.memory.hostPointer(query+16,16),direction,16);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}std::memset(p.vu0_vf,0x47,sizeof(p.vu0_vf));
        const uint32_t caller=seed&1?0x3e753c:0x3e70bc;p.r[4].ud[0]=argument;p.r[5].ud[0]=query;p.r[29].ud[0]=stack;p.r[31].ud[0]=caller;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~(_MM_ROUND_MASK|0x8040u))|mode|(seed&1?0x8040u:0u);_mm_setcsr(control);
        host.reference=true;x.functions.generated(0x1f7370)(x.memory.flat(),&p,&x);const auto expected=_mm_getcsr();_mm_setcsr(control);
        host.reference=false;y.functions.generated(0x1f7370)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expected);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;
        if(std::memcmp(&p,&q,sizeof(p))){const auto *u=reinterpret_cast<const uint8_t *>(&p),*v=reinterpret_cast<const uint8_t *>(&q);for(unsigned i=0;i<sizeof(p);++i)if(u[i]!=v[i]){std::fprintf(stderr,"query seed=%u mode=%x CPU byte=%u old=%02x new=%02x pc=%x/%x\n",seed,mode,i,u[i],v[i],p.pc,q.pc);break;}}
        if(std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes))for(unsigned i=0;i<bozo::kFlatBytes;++i)if(x.memory.flat()[i]!=y.memory.flat()[i]){std::fprintf(stderr,"query seed=%u RAM=%x old=%02x new=%02x\n",seed,i,x.memory.flat()[i],y.memory.flat()[i]);break;}
        check(p.pc==caller && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));check(q.r[2].uw[0]==0);
        if(seed%13>=11)++querySkipped;else{const auto distance=std::bit_cast<float>(word(y,frame+0x5c));if(distance<=0)++queryHits;else ++queryMisses;
            if(seed%7==0 || seed%7==2)check(distance==-1.f);if(seed%7==1)check(distance==8.f);}
        ++queries;
    }
    check(queryHits && queryMisses && querySkipped);
    {auto &r=b.runtime();constexpr uint32_t node=0x180030,object=0x1a0000,query=0x1c0000,stack=0x1fff000;R5900Context c{};
        c.r[18].ud[0]=node;c.r[17].ud[0]=query;c.r[29].ud[0]=stack;c.r[31].ud[0]=0x1f7390;
        put(r,node,0u);put(r,node+4,object);put<int16_t>(r,node+14,-1);put(r,stack+0xf0,0x3e70bcu);
        const auto reject=[&]{const auto before=c;std::vector<uint8_t> memory(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1f6f94);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(memory.data(),r.memory.flat(),memory.size()));};
        // Populated nodes (a child, or a first triangle) are admitted now: their
        // compiled triangle loop calls the native leaf tests 1f6be0/1f6e10.
        for(int which=0;which<2;++which){if(which)put<int16_t>(r,node+14,0);else put(r,node,0x180050u);const auto before=c;
            check(vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1f6f94) && !std::memcmp(&before,&c,sizeof(c)));put(r,node,0u);put<int16_t>(r,node+14,-1);}
        put(r,stack+0xf0,0x123456u);reject();put(r,stack+0xf0,0x3e70bcu);c.r[17].ud[0]=stack+0x70;reject();
        c.r[17].ud[0]=query;c.r[2].ud[0]=stack+0x50;
        for(unsigned i=0;i<16;++i)put(r,object+0x150+i*4,std::bit_cast<uint32_t>(i%5?0.f:1.f));
        put(r,object+0x204,0x3f800000u);for(unsigned i=0;i<4;++i)put(r,node+16+i*4,i==3?0x3f800000u:0u);
        for(unsigned i=0;i<8;++i)put(r,query+i*4,i==4?0x3f800000u:0u);
        const auto rejectDistance=[&]{const auto before=c;std::vector<uint8_t> memory(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1f717c);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(memory.data(),r.memory.flat(),memory.size()));};
        c.r[2].ud[0]+=16;rejectDistance();c.r[2].ud[0]-=16;
        for(auto at:{query,object+0x150,object+0x204}){const auto saved=word(r,at);for(auto bits:{0x7fc00000u,1u,0x7f000000u}){put(r,at,bits);rejectDistance();}put(r,at,saved);}
    }
    std::printf("%u height/%u corner/%u terrain/%u leafless queries (%u broad hits/%u misses/%u skipped): complete CPU/RAM/FP callers, four modes, exact results and guarded populated trees\n",cases,corners,terrains,queries,queryHits,queryMisses,querySkipped);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

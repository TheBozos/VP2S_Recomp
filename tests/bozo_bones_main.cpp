#include "native_math.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "bones_reference_functions.h"
#include "bones_adapted_functions.h"
#include <bit>
#include <cstdio>
#include <cstring>
#include <immintrin.h>
#include <source_location>
#include <stdexcept>
namespace {
void check(bool ok,std::source_location at=std::source_location::current()){
    if(!ok)throw std::runtime_error("bone matrix assertion at "+std::to_string(at.line()));
}
uint32_t product(uint32_t a,uint32_t b){
    const auto sign=(a^b)&0x80000000u;unsigned ea=(a>>23)&255,eb=(b>>23)&255;if(!ea || !eb)return sign;
    uint64_t p=uint64_t((a&0x7fffff)|0x800000)*((b&0x7fffff)|0x800000);int e=int(ea)+int(eb)-127;unsigned shift=23;
    if(p&(1ull<<47)){++e;++shift;}if(e<=0)return sign;check(e<255);return sign|uint32_t(e<<23)|uint32_t((p>>shift)&0x7fffff);
}
uint32_t sum(uint32_t a,uint32_t b){
    if((a&0x7fffffff)<(b&0x7fffffff))std::swap(a,b);
    if(!(b&0x7fffffff))return (a&0x7fffffff)?a:(a&b&0x80000000u);
    int e=(a>>23)&255;unsigned distance=unsigned(e)-((b>>23)&255);
    const int64_t large=(a&0x7fffff)|0x800000,small=distance<=23?(((b&0x7fffff)|0x800000)>>distance):0;
    int64_t value=(a>>31?-large:large)+(b>>31?-small:small);if(!value)return 0;
    const auto sign=value<0?0x80000000u:0u;uint64_t m=value<0?-value:value;
    while(m>=0x1000000){m>>=1;++e;}while(m<0x800000){m<<=1;--e;}
    if(e<=0)return sign;check(e<255);return sign|uint32_t(e<<23)|uint32_t(m&0x7fffff);
}
uint32_t root(uint32_t bits){
    const double target=std::bit_cast<float>(bits);uint32_t low=0,high=0x7f7fffff;
    while(low<high){const auto mid=low+(high-low+1)/2;const double value=std::bit_cast<float>(mid);
        if(value*value<=target)low=mid;else high=mid-1;}return low;
}
template<class T>void put(Runtime &r,uint32_t p,T v){std::memcpy(r.memory.hostPointer(p,sizeof(v)),&v,sizeof(v));}
uint32_t word(Runtime &r,uint32_t p){uint32_t v;std::memcpy(&v,r.memory.hostPointer(p,4),4);return v;}
void oracle(Runtime &r,R5900Context *c){
    uint32_t base[4][4],bone[4][4],local[4][4],relative[4][4],result[4][4],output[4][4];
    const auto owner=c->r[20].uw[0],i=c->r[6].uw[0]-1;
    std::memcpy(base,r.memory.hostPointer(c->r[29].uw[0]+0x60,64),64);
    std::memcpy(bone,r.memory.hostPointer(word(r,c->r[17].uw[0]+i*4)+0x90,64),64);
    std::memcpy(local,r.memory.hostPointer(word(r,owner+0x40)+i*64,64),64);
    auto transform=[](uint32_t out[4],const uint32_t m[4][4],const uint32_t v[4]){
        for(unsigned lane=0;lane<4;++lane){auto s=product(m[0][lane],v[0]);
            for(unsigned j=1;j<4;++j)s=sum(s,product(m[j][lane],v[j]));out[lane]=s;}
    };
    for(unsigned j=0;j<3;++j)transform(relative[j],bone,base[j]);std::memcpy(relative[3],bone[3],16);
    for(unsigned j=0;j<3;++j)transform(result[j],relative,local[j]);std::memcpy(result[3],relative[3],16);
    for(unsigned row=0;row<4;++row)for(unsigned col=0;col<4;++col){
        const auto value=result[col][row];output[row][col]=col==row?value:col==3?product(value,0x3f800000):sum(0,value);
    }
    std::memcpy(r.memory.hostPointer(c->r[19].uw[0],64),output,64);
}
struct Host:bozo::ee::Host,bozo::port::Host{
    bool reference=false;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(reference && (pc==0x23c270 || pc==0x23c2a8 || pc==0x23c2c8)){
            const bool combined=pc!=0x23c2c8;uint32_t first[4],second[4],offset[4]{},out[4];
            std::memcpy(first,rt->memory.hostPointer(c->r[combined?6:5].uw[0],16),16);
            std::memcpy(second,rt->memory.hostPointer(c->r[combined?5:4].uw[0],16),16);
            if(combined)std::memcpy(offset,rt->memory.hostPointer(c->r[3].uw[0],16),16);
            for(unsigned i=0;i<4;++i){out[i]=product(first[i],second[i]);if(combined)out[i]=sum(out[i],offset[i]);}
            std::memcpy(rt->memory.hostPointer(c->r[combined?4:3].uw[0],16),out,16);return true;
        }
        if(reference && (pc==0x20f26c || pc==0x20f2ac)){
            uint32_t matrix[4][4],scale[4],out[4][4];
            std::memcpy(matrix,rt->memory.hostPointer(c->r[4].uw[0],64),64);std::memcpy(scale,rt->memory.hostPointer(c->r[30].uw[0],16),16);
            for(unsigned row=0;row<4;++row)for(unsigned lane=0;lane<4;++lane)out[row][lane]=product(matrix[row][lane],scale[row]);
            std::memcpy(rt->memory.hostPointer(c->r[3].uw[0],64),out,64);return true;
        }
        if(reference && pc==0x1fedfc){
            const auto saved=_mm_getcsr();uint32_t first[4],second[4],out[4];
            std::memcpy(first,rt->memory.hostPointer(c->r[5].uw[0],16),16);std::memcpy(second,rt->memory.hostPointer(c->r[2].uw[0],16),16);
            for(unsigned i=0;i<3;++i){const auto d=sum(first[i],second[i]^0x80000000u);out[i]=product(d,d);}
            out[0]=root(sum(sum(out[0],out[1]),out[2]));out[3]=first[3];
            std::memcpy(rt->memory.hostPointer(c->r[4].uw[0],16),out,16);_mm_setcsr(saved);return true;
        }
        if(reference){check(pc==0x237284 || pc==0x2372bc);oracle(*rt,c);return true;}
        check(vp2s::nativeMathInstruction(r,c,rt,pc));return true;
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
    Host h;bozo::ee::Dispatcher a(h),b(h);
    a.setFunctions(bones_reference_function_table,bones_reference_function_count,bones_reference_resume_table,bones_reference_resume_count);
    b.setFunctions(bones_adapted_function_table,bones_adapted_function_count,bones_adapted_resume_table,bones_adapted_resume_count);
    a.runtime().port=&h;b.runtime().port=&h;uint32_t random=51;unsigned cases=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        constexpr uint32_t owner=0x180000,list=0x190000,bind=0x1a0000,output=0x1b0000,metadata=0x1c0000;
        const uint32_t count=1+seed%11,stack=seed&1?0x70003e00:0x1fff000,selector=(seed>>1)&1;
        put(x,owner+0x34,metadata);put(x,metadata+4,count);put(x,owner+0x3c,list);put(x,owner+0x40,bind);put(x,owner+0x4c,output);put<uint8_t>(x,owner+0x50,selector);
        const auto matrix=[&](uint32_t at){uint32_t values[16];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((112+random%17)<<23);}
            if(seed%8==0)for(unsigned i=0;i<16;++i)values[i]=std::bit_cast<uint32_t>(i%5?0.f:1.f);
            if(seed%8==1)for(unsigned i=0;i<16;++i)values[i]=(i&1)?0x80000000u:0;
            std::memcpy(x.memory.hostPointer(at,64),values,64);};
        matrix(stack+0x60);
        for(unsigned i=0;i<count;++i){const auto bone=0x200000+i*0x100;put(x,list+i*4,bone);matrix(bone+0x90);matrix(bind+i*64);}
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0xd3,sizeof(p.vu0_vf));p.vu0_status=0x1245;p.r[17].ud[0]=list;p.r[20].ud[0]=owner;
        p.r[19].ud[0]=(output+0x40+selector*count*64)|0x30000000;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x2371dc)(x.memory.flat(),&p,&x);const auto expected=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x2371dc)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expected);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));
        check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));++cases;
    }
    auto &r=b.runtime();constexpr uint32_t owner=0x180000,list=0x190000,output=0x1b0000;
    const auto count=word(r,0x1c0004);const auto selector=*r.memory.hostPointer(owner+0x50,1);
    R5900Context c{};c.r[20].ud[0]=owner;c.r[17].ud[0]=list;c.r[6].ud[0]=count;c.r[7].ud[0]=count;
    c.r[19].ud[0]=output+0x40+selector*count*64+(count-1)*64;c.r[29].ud[0]=0x70003e00;
    auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
        try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x2372bc);}catch(const std::runtime_error &){bad=true;}
        check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
    c.r[6].ud[0]=0;reject();c.r[6].ud[0]=count;c.r[19].ud[0]+=16;reject();c.r[19].ud[0]-=16;
    const auto bind=word(r,owner+0x40);put(r,owner+0x40,output+0x40+selector*count*64);reject();put(r,owner+0x40,bind);
    const auto bone=word(r,list);put(r,list,output+0x40+selector*count*64-0x90);reject();put(r,list,bone);
    put(r,bind+(count-1)*64,0x7fc00000u);reject();
    unsigned distances=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x51,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003e00:0x1fff000,destination=stack+0x60;
        const uint32_t first=seed%3==0?destination:0x180000,second=seed%3==1?destination:0x190000;
        uint32_t values[8];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((100+random%40)<<23);}
        if(seed%8==0)for(unsigned i=0;i<3;++i)values[i+4]=values[i];
        if(seed%8==1)for(unsigned i=0;i<8;++i)values[i]=i&1?0x80000000u:0;
        values[3]=0x7fc01234u; // W is carried verbatim, never an XYZ input.
        std::memcpy(x.memory.hostPointer(first,16),values,16);std::memcpy(x.memory.hostPointer(second,16),values+4,16);
        constexpr uint32_t item=0x1a0000,table=0x1b0000;
        const auto index=seed%3;put(x,item+0xc,table);put(x,table+index*64+0x38,second-0x30);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x91,sizeof(p.vu0_vf));p.vu0_status=0x9876;
        p.r[4].ud[0]=destination;p.r[5].ud[0]=first;p.r[6].ud[0]=index;p.r[8].ud[0]=item;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x1fedc0)(x.memory.flat(),&p,&x);const auto expected=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x1fedc0)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expected);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));
        check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));++distances;
    }
    unsigned scales=0;
    for(unsigned branch=0;branch<2;++branch)for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t object=seed&1?0x70000100:0x180000,source=object+(branch?0x150:0xd0),destination=object+0x1d0;
        const uint32_t coefficient=seed%4==0?destination:seed%4==1?source+16:seed%4==2?0x190000:0x70000800;
        uint32_t values[20];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((104+random%30)<<23);}
        if(seed%8==0)for(unsigned i=0;i<20;++i)values[i]=(i&1)?0x80000000u:0;
        if(seed%8==2){for(unsigned i=0;i<16;++i)values[i]=0x00800000u|(i&1?0x80000000u:0);for(unsigned i=16;i<20;++i)values[i]=0x3f000000;}
        std::memcpy(x.memory.hostPointer(source,64),values,64);std::memcpy(x.memory.hostPointer(coefficient,16),values+16,16);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x81,sizeof(p.vu0_vf));p.vu0_status=0x9843;p.r[21].ud[0]=object;p.r[30].ud[0]=coefficient;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);const auto entry=branch?0x20f280:0x20f240;
        h.reference=true;x.functions.generated(entry)(x.memory.flat(),&p,&x);const auto expected=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(entry)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expected);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));++scales;
    }
    {auto &r=b.runtime();constexpr uint32_t object=0x180000;R5900Context c{};c.r[21].ud[0]=object;c.r[4].ud[0]=object+0x150;c.r[3].ud[0]=object+0x1d0;c.r[30].ud[0]=0x190000;
        uint32_t one=0x3f800000;for(unsigned i=0;i<16;++i)put(r,object+0x150+i*4,one);for(unsigned i=0;i<4;++i)put(r,0x190000+i*4,one);
        auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x20f2ac);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[4].ud[0]+=16;reject();c.r[4].ud[0]-=16;
        put(r,0x190000,0x7fc00000u);reject();put(r,0x190000,1u);reject();put(r,0x190000,0x40000000u);put(r,object+0x150,0x7f7fffffu);reject();
    }
    unsigned parameters=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x71,bozo::kFlatBytes);
        const uint32_t global=seed&1?0x70000100:0x180000,model=seed&2?0x70001000:0x190000,flags=seed%4;
        uint32_t values[16];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((104+random%30)<<23);}
        if(seed%8==0)for(unsigned i=0;i<16;++i)values[i]=i&1?0x80000000u:0;
        if(seed%8==1)for(unsigned i=0;i<4;++i){values[i]=std::bit_cast<uint32_t>(i&1?-1.f:1.f);values[i+4]=values[i+8]=values[i]^0x80000000u;values[i+12]=0x3f800000;}
        if(seed%8==2)for(unsigned i=0;i<12;++i){values[i]=0x00800000u|(i&1?0x80000000u:0);values[12+i%4]=0x3f000000;}
        std::memcpy(x.memory.hostPointer(global+0x310,16),values,16);std::memcpy(x.memory.hostPointer(global+0x390,16),values+4,16);
        std::memcpy(x.memory.hostPointer(global+0x3c0,16),values+8,16);std::memcpy(x.memory.hostPointer(model+0x7e0,16),values+12,16);
        *x.memory.hostPointer(global+0x518,1)=flags&1?7:0;*x.memory.hostPointer(global+0x517,1)=flags&2?9:0;
        const uint32_t coefficient=global+(flags&1?0x3c0:flags&2?0x390:0x310);
        const uint32_t stack=seed%4==0?global+0x310-0x50:seed%4==1?coefficient-0x50:seed%4==2?model+0x7e0-0x50:seed&4?0x70003e00:0x1fff000;
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x61,sizeof(p.vu0_vf));p.vu0_status=0x1254;p.r[18].ud[0]=global;p.r[19].ud[0]=model;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x23c240)(x.memory.flat(),&p,&x);const auto expected=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x23c240)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expected);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));++parameters;
    }
    {auto &r=b.runtime();constexpr uint32_t global=0x180000,model=0x190000;R5900Context c{};
        c.r[18].ud[0]=global;c.r[19].ud[0]=model;c.r[29].ud[0]=0x1fff000;c.r[6].ud[0]=global+0x3c0;c.r[5].ud[0]=model+0x7e0;c.r[3].ud[0]=global+0x310;c.r[4].ud[0]=0x1fff050;
        for(unsigned i=0;i<4;++i){put(r,global+0x310+i*4,0x3f800000u);put(r,global+0x3c0+i*4,0x3f800000u);put(r,model+0x7e0+i*4,0x3f800000u);}
        auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x23c270);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[4].ud[0]+=16;reject();c.r[4].ud[0]-=16;
        for(uint32_t bad:{0x7fc00000u,1u}){put(r,global+0x3c0,bad);reject();}
        put(r,global+0x3c0,0x7f7fffffu);put(r,model+0x7e0,0x40000000u);reject();
        put(r,model+0x7e0,0x3f800000u);put(r,global+0x310,0x7f7fffffu);reject();
    }
    std::printf("%u bone loops,%u pair distances,%u camera scales,%u model parameter branches: whole CPU/RAM, integer arithmetic, four modes, aliasing and rejected input; no VU/device state\n",cases,distances,scales,parameters);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

#include "native_object_math.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "world_original_functions.h"
#include "world_adapted_functions.h"
#include <bit>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <source_location>
#include <set>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){
    if(!v)throw std::runtime_error("native object world assertion at "+std::to_string(at.line()));
}
float normal(float x){auto b=std::bit_cast<uint32_t>(x);return std::bit_cast<float>((b&0x7f800000)?b:(b&0x80000000));}
[[gnu::noinline,gnu::noclone]] float mul(float a,float b){volatile float v=a*b;return normal(v);}
[[gnu::noinline,gnu::noclone]] float add(float a,float b){volatile float v=a+b;return normal(v);}
[[gnu::noinline,gnu::noclone]] float chop(float a,float b){
    auto x=std::bit_cast<uint32_t>(a),y=std::bit_cast<uint32_t>(b);int ea=(x>>23)&255,eb=(y>>23)&255;
    if(ea && eb){
        if(ea<eb){const int d=eb-ea;x&=0x80000000u|(d>23?0u:~0u<<d);}
        if(eb<ea){const int d=ea-eb;y&=0x80000000u|(d>23?0u:~0u<<d);}
    }
    volatile float v=std::bit_cast<float>(x)+std::bit_cast<float>(y);return normal(v);
}
struct Rounding {unsigned saved=_mm_getcsr();Rounding(){_mm_setcsr((saved&~_MM_ROUND_MASK)|_MM_ROUND_TOWARD_ZERO);}~Rounding(){_mm_setcsr(saved);}};
// Independent scalar equations read opaque records. This is neither a VU
// instruction executor nor a VU register/flag reference implementation.
[[gnu::noinline,gnu::noclone]] vp2s::ObjectWorldResult reference(const vp2s::ObjectWorldInput &in){
    float q[4],p[4],scale[4],parent[3][4],attachment[4][4],rotation[4][4]{},local[4][4]{},world[4][4]{};
    std::memcpy(q,&in.quaternion,16);std::memcpy(p,&in.position,16);std::memcpy(scale,&in.scale,16);scale[3]=1;
    std::memcpy(parent,in.parent.data(),48);std::memcpy(attachment,in.attachment.data(),64);
    {Rounding rounding;float square[4],twice[4],diagonal[4],x[4],w[4],y[4],z[4];
        for(unsigned i=0;i<4;++i){square[i]=mul(q[i],q[i]);twice[i]=chop(q[i],q[i]);diagonal[i]=chop(square[i],square[i]);}
        for(unsigned i=0;i<4;++i){x[i]=mul(q[i],twice[0]);w[i]=mul(q[i],twice[3]);y[i]=mul(q[i],twice[1]);z[i]=mul(q[i],twice[2]);}
        rotation[0][0]=chop(chop(1,-diagonal[1]),-diagonal[2]);rotation[0][1]=chop(x[1],w[2]);rotation[0][2]=chop(x[2],-w[1]);
        rotation[1][0]=chop(y[0],-w[2]);rotation[1][1]=chop(chop(1,-diagonal[0]),-diagonal[2]);rotation[1][2]=chop(y[2],x[3]);
        rotation[2][0]=chop(z[0],w[1]);rotation[2][1]=chop(z[1],-x[3]);rotation[2][2]=chop(chop(1,-diagonal[0]),-diagonal[1]);
    }
    for(unsigned i=0;i<3;++i)rotation[i][3]=p[i];rotation[3][3]=1;
    if(in.hasParent){
        {Rounding rounding;
            for(unsigned row=0;row<3;++row)for(unsigned lane=0;lane<4;++lane){
                float v=mul(rotation[0][lane],parent[row][0]);
                for(unsigned k=1;k<4;++k)v=chop(v,mul(rotation[k][lane],parent[row][k]));
                local[row][lane]=mul(scale[lane],v);
            }
        }
        if(in.normalizeParent){
            {Rounding rounding;
                for(unsigned lane=0;lane<3;++lane){
                    float length2=mul(local[0][lane],local[0][lane]);
                    length2=chop(length2,mul(local[1][lane],local[1][lane]));length2=chop(length2,mul(local[2][lane],local[2][lane]));
                    volatile float length=std::sqrt(length2);volatile float factor=length==0?std::numeric_limits<float>::max():1.f/length;
                    for(unsigned row=0;row<3;++row)local[row][lane]=mul(local[row][lane],factor);
                }
            }
            for(unsigned row=0;row<3;++row)for(unsigned lane=0;lane<4;++lane)local[row][lane]=mul(local[row][lane],scale[lane]);
        }
    }else for(unsigned row=0;row<3;++row)for(unsigned lane=0;lane<4;++lane)local[row][lane]=mul(rotation[row][lane],scale[lane]);
    local[3][3]=1;
    for(unsigned row=0;row<3;++row)for(unsigned lane=0;lane<4;++lane){
        float v=mul(attachment[0][lane],local[row][0]);for(unsigned k=1;k<4;++k)v=add(v,mul(attachment[k][lane],local[row][k]));world[row][lane]=v;
    }
    std::memcpy(world[3],attachment[3],16);vp2s::ObjectWorldResult out;
    std::memcpy(out.local.data(),local,64);std::memcpy(out.world.data(),world,64);return out;
}
struct Pause{};
struct Host:bozo::ee::Host,bozo::port::Host {
    std::set<uint32_t> sites;
    bool native=false,pause=false;unsigned callbacks=0,pauses=0;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        sites.insert(pc);
        if(native){check(vp2s::nativeObjectMathInstruction(r,c,rt,pc));return true;}
        if(pc==0x249114)c->r[9].ud[0]=0;
        if(pc==0x249180 || pc==0x24922c){
            auto *at=rt->memory.hostPointer(c->r[16].uw[0],0x210);check(at);vp2s::ObjectWorldInput in{};
            std::memcpy(&in.quaternion,at+0x30,16);std::memcpy(&in.position,at+0x20,16);std::memcpy(&in.scale,at+0x40,16);
            std::memcpy(in.attachment.data(),at+0x110,64);in.hasParent=c->r[17].uw[0]!=0;uint16_t flags;std::memcpy(&flags,at+0x20c,2);in.normalizeParent=!(flags&0x1c0);
            if(in.hasParent)std::memcpy(in.parent.data(),rt->memory.hostPointer(c->r[17].uw[0]+0xd0,48),48);
            auto out=reference(in);std::memcpy(at+0xd0,out.local.data(),64);std::memcpy(at+0x90,out.world.data(),64);
        }return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        check(pc==0x170100);++callbacks;const uint32_t parent=c->r[4].uw[0];
        auto *at=d.runtime().memory.hostPointer(parent,0x110);check(at);at[0x61]=1;const float translation=1.25f;std::memcpy(at+0xdc,&translation,4);
        c->pc=c->r[31].uw[0];if(pause){++pauses;throw Pause{};}
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
template<class T>void put(uint8_t *r,uint32_t at,T v){std::memcpy(r+at,&v,sizeof(v));}
void run(bozo::ee::Dispatcher &d,R5900Context &c){auto fn=d.runtime().functions.generated(0x248fc0);unsigned attempts=0;
    while(true){try{fn(d.runtime().memory.flat(),&c,&d.runtime());break;}catch(const Pause &){}
        check(++attempts<4);auto entry=d.runtime().functions.resumeEntry(c.pc);check(entry);fn=entry->resume;}
}
}
int main(){try{
    Host old,now;now.native=true;bozo::ee::Dispatcher a(old),b(now);
    a.setFunctions(world_original_function_table,world_original_function_count,world_original_resume_table,world_original_resume_count);
    b.setFunctions(world_adapted_function_table,world_adapted_function_count,world_adapted_resume_table,world_adapted_resume_count);
    a.runtime().port=&old;b.runtime().port=&now;a.bindLibrary(0x170100);b.bindLibrary(0x170100);
    const unsigned saved=_mm_getcsr();unsigned cases=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<80;++seed){
        auto *x=a.runtime().memory.flat(),*y=b.runtime().memory.flat();std::memset(x,0x22,bozo::kFlatBytes);
        constexpr uint32_t object=0x180000;
        const uint32_t parents[5]={0,0x190000,object,object+0x40,object-0x40};const uint32_t parent=parents[seed%5];
        const auto q=_mm_setr_ps(seed%17==0?1.e-21f:float(int(seed%9)-4)/16,-float(seed%7)/32,float(seed%11)/16,0.75f);
        const auto p=_mm_setr_ps(float(seed)+0.125f,-3.25f,float(seed%5)-2.f,std::bit_cast<float>(0x7fc01234u));
        const auto scale=_mm_setr_ps(seed%9==0?std::bit_cast<float>(seed&1?0x80000000u:0u):1.25f,-0.75f,1.5f,std::bit_cast<float>(0x7fc05678u));
        std::memcpy(x+object+0x20,&p,16);std::memcpy(x+object+0x30,&q,16);std::memcpy(x+object+0x40,&scale,16);
        for(unsigned i=0;i<16;++i)put<float>(x,object+0x110+i*4,float(int((seed+i)%13)-6)/8);
        if(parent){for(unsigned i=0;i<12;++i)put<float>(x,parent+0xd0+i*4,(seed%13==0?1.e-20f:1.f)*float(int((seed+i)%11)-5)/4);
            if(parent!=object+0x40)put<uint32_t>(x,parent,0x1a0000);
            // This alias's vtable would occupy the active child scale.x.
            // Keep the no-callback parent path rather than poison that input.
            x[parent+0x61]=parent==object+0x40?1:(seed>>2)&1;x[parent+0x51]=0;}
        put<uint32_t>(x,0x1a0064,0x170100);put<uint32_t>(x,object+0x208,parent);
        put<uint16_t>(x,object+0x20c,seed&16?0x1c0:0);x[object+0x50]=(seed&8)!=0;
        std::memcpy(y,x,bozo::kFlatBytes);old.callbacks=now.callbacks=old.pauses=now.pauses=0;now.pause=(seed&32)!=0;
        R5900Context pctx{},qctx{};for(unsigned i=1;i<32;++i){pctx.r[i].ud[0]=0x1234000000000000ull+seed*32+i;pctx.r[i].ud[1]=~pctx.r[i].ud[0];}
        std::memset(pctx.vu0_vf,0x6a,sizeof(pctx.vu0_vf));std::memset(&pctx.vu0_acc,0xd4,sizeof(pctx.vu0_acc));
        pctx.r[4].ud[0]=object;pctx.r[29].ud[0]=seed&1?0x70003f00:0x1fff000;pctx.r[31].ud[0]=0x170000;qctx=pctx;
        const unsigned control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);
        _mm_setcsr(control);run(a,pctx);const unsigned expectedControl=_mm_getcsr();_mm_setcsr(control);run(b,qctx);
        check(_mm_getcsr()==expectedControl);check(old.callbacks==now.callbacks);
        pctx.cycleLimit=qctx.cycleLimit;check(!std::memcmp(&pctx,&qctx,sizeof(pctx)));
        if(std::memcmp(x,y,bozo::kFlatBytes)){
            for(unsigned i=0;i<32;++i){uint32_t expected,actual;std::memcpy(&expected,x+object+0x90+i*4,4);std::memcpy(&actual,y+object+0x90+i*4,4);
                if(expected!=actual)std::fprintf(stderr,"seed=%u mode=%x matrix-word=%u expected=%08x actual=%08x\n",seed,mode,i,expected,actual);}
            check(false);
        }++cases;
    }
    _mm_setcsr(saved);
    check(old.sites==now.sites && now.sites.size()==64);
    vp2s::ObjectWorldInput input{};input.quaternion=_mm_setr_ps(0,0,0,1);input.position=_mm_setzero_ps();input.scale=_mm_set1_ps(1);
    input.attachment={_mm_setr_ps(1,0,0,0),_mm_setr_ps(0,1,0,0),_mm_setr_ps(0,0,1,0),_mm_setr_ps(0,0,0,1)};
    const auto identity=vp2s::objectWorld(input);check(!std::memcmp(identity.world.data(),input.attachment.data(),64));
    input.quaternion=_mm_setr_ps(std::bit_cast<float>(1u),0,0,1);bool rejected=false;
    try{vp2s::objectWorld(input);}catch(const std::runtime_error &){rejected=true;}check(rejected);
    std::printf("%u generated object-world CPU/RAM and independent scalar matrix comparisons; parent/self/output/attachment aliases, FP modes, tiny terms and resumed callbacks passed\n",cases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

#include "native_object_math.h"
#include "native_math.h"
#include "native_trig.h"
#include "../port/vp2s_math.h"
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
        if(native){check(vp2s::nativeMathInstruction(r,c,rt,pc));return true;}
        // The already qualified sine/cosine owner is a retained callee. Only
        // the new builder's matrix product uses the independent equations.
        if(vp2s::nativeTrigInstruction(r,c,rt,pc))return true;
        if(pc==0x209020)c->r[9].ud[0]=0;
        if(pc==0x209028){
            float screen[4][4],point[4],transposed[4][4],result[4];
            std::memcpy(screen,rt->memory.hostPointer(c->r[16].uw[0]+0x330,64),64);
            auto *destination=rt->memory.hostPointer(c->r[2].uw[0],16);std::memcpy(point,destination,16);
            {Rounding rounding;for(unsigned i=0;i<4;++i)for(unsigned lane=0;lane<4;++lane){
                const float v=screen[lane][i];transposed[i][lane]=i==lane?v:lane==3?mul(v,1):chop(0,v);
            }
            for(unsigned lane=0;lane<4;++lane){float v=mul(transposed[0][lane],point[0]);
                for(unsigned k=1;k<4;++k)v=chop(v,mul(transposed[k][lane],point[k]));result[lane]=v;}
            volatile float reciprocal=1.f/result[3];result[3]=1;for(float &v:result)v=mul(v,reciprocal);}
            std::memcpy(destination,result,16);
        }
        if(pc==0x216988)c->r[9].ud[0]=0;
        if(pc==0x216990){
            float result[4][4];std::memcpy(result,rt->memory.hostPointer(c->r[5].uw[0],64),64);
            {Rounding rounding;for(unsigned lane=0;lane<3;++lane){
                float length2=mul(result[0][lane],result[0][lane]);
                length2=chop(length2,mul(result[1][lane],result[1][lane]));length2=chop(length2,mul(result[2][lane],result[2][lane]));
                volatile float length=std::sqrt(length2);volatile float factor=length==0?std::numeric_limits<float>::max():1.f/length;
                for(unsigned row=0;row<3;++row)result[row][lane]=mul(result[row][lane],factor);
            }}
            std::memcpy(rt->memory.hostPointer(c->r[2].uw[0],64),result,64);
        }
        if(pc==0x213e7c || pc==0x209158){
            float local[4][4],basis[4][4],result[4][4];
            std::memcpy(local,rt->memory.hostPointer(c->r[pc==0x213e7c?8:5].uw[0],64),64);
            std::memcpy(basis,rt->memory.hostPointer(c->r[pc==0x213e7c?7:4].uw[0],64),64);
            for(unsigned row=0;row<4;++row)for(unsigned lane=0;lane<4;++lane){
                float v=mul(basis[0][lane],local[row][0]);
                for(unsigned k=1;k<4;++k)v=add(v,mul(basis[k][lane],local[row][k]));
                result[row][lane]=v;
            }
            std::memcpy(rt->memory.hostPointer(c->r[pc==0x213e7c?16:6].uw[0],64),result,64);
        }
        if(pc==0x216efc){
            float basis[3][4],translation[4],result[4];
            std::memcpy(basis,rt->memory.hostPointer(c->r[5].uw[0],48),48);auto *temporary=rt->memory.hostPointer(c->r[3].uw[0],16);
            std::memcpy(translation,temporary,16);std::memcpy(result,translation,16);
            for(unsigned lane=0;lane<3;++lane){float v=mul(basis[0][lane],translation[0]);v=add(v,mul(basis[1][lane],translation[1]));result[lane]=add(v,mul(basis[2][lane],translation[2]));}
            std::memcpy(temporary,result,16);
        }
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
        if(pc==0x170200 || pc==0x170210 || pc==0x207d70 || pc==0x12e050){
            ++callbacks;if(pc==0x170200)c->r[2].ud[0]=1;
            if(pc==0x170210)c->r[2].ud[0]=c->r[4].uw[0]+0x90;
            if(pc==0x12e050)c->f[0]=std::tan(c->f[12]);
            c->pc=c->r[31].uw[0];if(pause){++pauses;throw Pause{};}return;
        }
        // Identical scalar consumers on both sides isolate normalization. The
        // production runner retains its compiled game libm implementations.
        if(pc==0x12e1d8 || pc==0x12e2d8){++callbacks;c->f[0]=pc==0x12e1d8?std::asin(c->f[12]):std::atan2(c->f[12],c->f[13]);
            c->pc=c->r[31].uw[0];if(pause){++pauses;throw Pause{};}return;}
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
void run(bozo::ee::Dispatcher &d,R5900Context &c,uint32_t pc=0x248fc0){auto fn=d.runtime().functions.generated(pc);unsigned attempts=0;
    while(true){try{fn(d.runtime().memory.flat(),&c,&d.runtime());break;}catch(const Pause &){}
        check(++attempts<12);auto entry=d.runtime().functions.resumeEntry(c.pc);check(entry);fn=entry->resume;}
}
}
int main(){try{
    Host old,now;now.native=true;bozo::ee::Dispatcher a(old),b(now);
    a.setFunctions(world_original_function_table,world_original_function_count,world_original_resume_table,world_original_resume_count);
    b.setFunctions(world_adapted_function_table,world_adapted_function_count,world_adapted_resume_table,world_adapted_resume_count);
    a.runtime().port=&old;b.runtime().port=&now;a.bindLibrary(0x170100);b.bindLibrary(0x170100);
    for(auto *d:{&a,&b}){d->bindLibrary(0x12e1d8);d->bindLibrary(0x12e2d8);}
    for(auto *d:{&a,&b})for(uint32_t pc:{0x12e050,0x2138f0,0x207d70,0x170200,0x170210})d->bindLibrary(pc);
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
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<64;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x37,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003f00:0x1fff000;
        const uint32_t source=seed%3==0?0x70000100:seed%3==1?0x180000:stack-0x40;
        const uint32_t destination=seed%3==0?source:seed%3==1?0x190000:0x70000200;
        float matrix[16];for(unsigned i=0;i<12;++i)matrix[i]=(seed%11==0?1.e-20f:1.f)*float(int((seed+i)%17)-8)/4;
        for(unsigned i=12;i<16;++i)matrix[i]=std::bit_cast<float>(0x7fc01234u+i);
        std::memcpy(x.memory.hostPointer(source,64),matrix,64);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{},q{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=0x1234000000000000ull+seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x6a,sizeof(p.vu0_vf));p.r[4].ud[0]=destination;p.r[5].ud[0]=source;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;q=p;
        const unsigned control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);
        _mm_setcsr(control);run(a,p,0x216e40);const unsigned expectedControl=_mm_getcsr();_mm_setcsr(control);run(b,q,0x216e40);
        check(_mm_getcsr()==expectedControl && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));++cases;
    }
    _mm_setcsr(saved);check(old.sites==now.sites && now.sites.size()==72);
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<64;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x37,bozo::kFlatBytes);
        R5900Context init{};init.r[31].uw[0]=0x170000;run(a,init,0x358490);
        const uint32_t stack=seed&1?0x70003e00:0x1fff000;
        const uint32_t destination=seed%4==0?0x180000:seed%4==1?stack-0xa0:seed%4==2?stack-0x40:0x70000200;
        const float position[4]={float(seed)/4,-2.75f,seed%7==0?1.e-20f:3.125f,1};
        const float direction[4]={seed%4==0?0.f:0.375f,seed%4==0?1.f:-0.25f,seed%4==0?0.f:1.125f,0};
        std::memcpy(x.memory.hostPointer(0x190000,16),position,16);std::memcpy(x.memory.hostPointer(0x190010,16),direction,16);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{},q{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=0x1234000000000000ull+seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x6a,sizeof(p.vu0_vf));p.r[4].ud[0]=destination;p.r[5].ud[0]=0x190000;p.r[6].ud[0]=0x190010;
        p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;p.f[12]=float(int(seed)-32)/8;q=p;
        const unsigned control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);
        _mm_setcsr(control);run(a,p,0x213b50);const unsigned expectedControl=_mm_getcsr();_mm_setcsr(control);run(b,q,0x213b50);
        check(_mm_getcsr()==expectedControl && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));++cases;
    }
    _mm_setcsr(saved);check(old.sites==now.sites && now.sites.size()==130);
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<64;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x37,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003f00:0x1fff000;
        const uint32_t source=seed%3==0?0x70000100:seed%3==1?0x180000:stack-0x40;
        const uint32_t destination=seed%3==0?source:seed%3==1?0x190000:0x70000200;
        float matrix[16];for(unsigned i=0;i<16;++i)matrix[i]=float(int((seed+i)%17)-8)/4;
        if(seed%8<2){for(unsigned row=0;row<3;++row)matrix[row*4+2]=row==1?(seed%8==0?1.f:-1.f):0.f;}
        if(seed%8==2)for(unsigned row=0;row<3;++row)matrix[row*4]=0;
        if(seed%8==3)for(unsigned row=0;row<3;++row)matrix[row*4]*=1.e-20f;
        for(unsigned row=0;row<3;++row)matrix[row*4+3]=std::bit_cast<float>(0x7fc01234u+row);
        for(unsigned i=12;i<16;++i)matrix[i]=std::bit_cast<float>(0x7fc05678u+i);
        std::memcpy(x.memory.hostPointer(source,64),matrix,64);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        old.callbacks=now.callbacks=old.pauses=now.pauses=0;old.pause=false;now.pause=(seed&32)!=0;
        R5900Context p{},q{};p.r[4].ud[0]=destination;p.r[5].ud[0]=source;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;q=p;
        const unsigned control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);
        _mm_setcsr(control);run(a,p,0x216960);const unsigned expectedControl=_mm_getcsr();_mm_setcsr(control);run(b,q,0x216960);
        p.cycleLimit=q.cycleLimit;check(old.callbacks==now.callbacks);
        check(_mm_getcsr()==expectedControl && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));++cases;
    }
    _mm_setcsr(saved);check(old.sites==now.sites && now.sites.size()==140);
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<64;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x37,bozo::kFlatBytes);
        constexpr uint32_t object=0x180000;std::memset(x.memory.flat()+object,0,0x600);
        put<uint32_t>(x.memory.flat(),object,0x1a0000);put<uint32_t>(x.memory.flat(),0x1a0064,0x170200);put<uint32_t>(x.memory.flat(),0x1a0068,0x170210);
        for(unsigned i=0;i<4;++i)put<float>(x.memory.flat(),object+0x90+i*20,1);
        put<float>(x.memory.flat(),object+0x9c,float(seed)/8);put<float>(x.memory.flat(),object+0xac,-2.5f);put<float>(x.memory.flat(),object+0xbc,4.125f);
        for(unsigned offset:{0x224,0x228,0x22c,0x234,0x238,0x268})put<float>(x.memory.flat(),object+offset,offset==0x234?0.25f:1.f);
        put<int32_t>(x.memory.flat(),object+0x23c,0);put<int32_t>(x.memory.flat(),object+0x240,0xffffff);
        put<float>(x.memory.flat(),object+0x244,1728);put<float>(x.memory.flat(),object+0x248,1824);
        put<float>(x.memory.flat(),object+0x270,640+seed);put<float>(x.memory.flat(),object+0x274,448);put<float>(x.memory.flat(),object+0x278,1000);
        x.memory.flat()[object+0x280]=seed&1;x.memory.flat()[object+0x44a]=1;
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        old.callbacks=now.callbacks=old.pauses=now.pauses=0;old.pause=false;now.pause=(seed&32)!=0;
        R5900Context p{},q{};p.r[4].ud[0]=object;p.r[29].ud[0]=seed&1?0x70003f00:0x1fff000;p.r[31].ud[0]=0x170000;q=p;
        const unsigned control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);
        _mm_setcsr(control);run(a,p,0x208cd0);const unsigned expectedControl=_mm_getcsr();_mm_setcsr(control);run(b,q,0x208cd0);
        p.cycleLimit=q.cycleLimit;check(old.callbacks==now.callbacks);
        check(_mm_getcsr()==expectedControl && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));++cases;
    }
    _mm_setcsr(saved);check(old.sites==now.sites && now.sites.size()==177);
    bool badProjection=false;try{vp2s::math::cameraProjectedPoint(vp2s::math::Matrix{},_mm_set1_ps(1));}catch(const std::runtime_error &){badProjection=true;}check(badProjection);
    vp2s::math::Matrix invalid{};invalid[0]=_mm_setr_ps(std::bit_cast<float>(1u),0,0,0);bool badColumns=false;
    try{vp2s::math::normalizedMatrixColumns(invalid);}catch(const std::runtime_error &){badColumns=true;}check(badColumns);
    vp2s::ObjectWorldInput input{};input.quaternion=_mm_setr_ps(0,0,0,1);input.position=_mm_setzero_ps();input.scale=_mm_set1_ps(1);
    input.attachment={_mm_setr_ps(1,0,0,0),_mm_setr_ps(0,1,0,0),_mm_setr_ps(0,0,1,0),_mm_setr_ps(0,0,0,1)};
    const auto identity=vp2s::objectWorld(input);check(!std::memcmp(identity.world.data(),input.attachment.data(),64));
    input.quaternion=_mm_setr_ps(std::bit_cast<float>(1u),0,0,1);bool rejected=false;
    try{vp2s::objectWorld(input);}catch(const std::runtime_error &){rejected=true;}check(rejected);
    std::printf("%u generated world/inverse CPU/RAM and scalar matrix comparisons; aliases, FP modes, tiny terms and resumed callbacks passed\n",cases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

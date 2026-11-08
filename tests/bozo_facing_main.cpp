#include "native_scene.h"
#include "native_frustum.h"
#include "native_float_oracle.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "facing_functions.h"
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <set>
#include <source_location>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("facing assertion at "+std::to_string(at.line()));}
using Vec=std::array<uint32_t,4>;
using Matrix=std::array<float,16>;
template<class T>T read(Runtime &rt,uint32_t at){T v;auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(&v,p,sizeof(v));return v;}
template<class T>void write(Runtime &rt,uint32_t at,const T &v){auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(p,&v,sizeof(v));}
uint32_t root(uint32_t bits){
    const auto value=[](uint32_t v){return std::ldexp(double((v&0x7fffff)|((v>>23)?0x800000:0)),(v>>23)?int(v>>23)-150:-149);};
    const auto target=value(bits);uint32_t lo=0,hi=0x7f7fffff;
    while(lo<hi){const auto mid=lo+(hi-lo+1)/2;const auto v=value(mid);if(v*v<=target)lo=mid;else hi=mid-1;}return lo;
}
Vec expectedFacing(const Matrix &view,const Matrix &world,Vec position,Vec direction){
    struct Scope{unsigned fp=_mm_getcsr();~Scope(){_mm_setcsr(fp);}} scope;_mm_setcsr((scope.fp&~_MM_ROUND_MASK)|_MM_ROUND_TOWARD_ZERO);
    using vp2s::test::sum;using vp2s::test::product;
    const auto coefficient=[&](const Matrix &m,unsigned row,unsigned column){const auto v=std::bit_cast<uint32_t>(m[row*4+column]);return row==column?v:row==3?product(0x3f800000,v):sum(0,v);};
    const auto transform=[&](const Matrix &m,Vec input,bool translation){Vec out;
        for(unsigned row=0;row<4;++row){out[row]=product(coefficient(m,row,0),input[0]);for(unsigned column=1;column<3;++column)out[row]=sum(out[row],product(coefficient(m,row,column),input[column]));
            out[row]=sum(out[row],product(translation?(row==3?0x3f800000:coefficient(m,row,3)):(row==3?0x3f800000:0),input[3]));}return out;};
    position[3]=0x3f800000;const auto eye=transform(view,position,true);const auto normal=transform(view,transform(world,direction,false),false);
    const auto inverse=[&](Vec v){const auto a=product(v[0],v[0]),b=product(v[1],v[1]),c=product(v[2],v[2]);return vp2s::test::quotient(0x3f800000,root(sum(sum(a,b),product(0x3f800000,c))));};
    const auto eyeScale=inverse(eye),normalScale=inverse(normal);auto out=normal;
    for(unsigned i=0;i<3;++i)out[i]=product(product(normal[i],product(eye[i],eyeScale)),normalScale);
    out[0]=sum(sum(out[0],out[1]),product(0x3f800000,out[2]));return out;
}
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::SceneOwner {
    bool reference=false,mutate=false;vp2s::NativeScene scene;Matrix world{},view{};Vec result{};uint32_t source=0;std::set<uint32_t> sites;
    vp2s::NativeScene &nativeScene()override{return scene;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("library");}bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc)override{
        sites.insert(pc);if(pc==0x220a20){source=c->r[2].uw[0];if(reference)world=read<Matrix>(*rt,source);}
        if(pc==0x220a38 && mutate)for(unsigned row=0;row<4;++row)for(unsigned column=0;column<3;++column)write(*rt,source+row*16+column*4,uint32_t(0x7fa01234));
        if(!reference){const auto handled=vp2s::nativeVisibilityInstruction(c,rt,pc);check(handled.has_value());return *handled;}
        if(pc==0x220a74)result=expectedFacing(view,world,read<Vec>(*rt,c->r[29].uw[0]+0x100),read<Vec>(*rt,c->r[29].uw[0]+0x110));
        if(pc==0x220a8c)c->r[9].ud[0]=0;if(pc==0x220a94)write(*rt,c->r[3].uw[0],result);return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("call");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
void prepare(Host &host,Runtime &rt,uint32_t stack){
    host.scene=vp2s::NativeScene{};host.scene.publishFrustum(&rt,0x190000);host.scene.beginVisibility(stack,0x180000);host.scene.visibilityClip(&rt,stack,0x190000);host.scene.visibilityView(&rt,stack,0x190000);
}
}
int main(){try{
    Host old,now;old.reference=true;bozo::ee::Dispatcher a(old),b(now);
    for(auto *d:{&a,&b}){d->runtime().port=d==&a?&old:&now;d->setFunctions(facing_function_table,facing_function_count,facing_resume_table,facing_resume_count);}
    const auto saved=_mm_getcsr();unsigned callers=0,known=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x51,bozo::kFlatBytes);const uint32_t stack=seed&8?0x70003d00:0x1ffe000;
        const bool alias=(seed&32) && !(seed&8);const uint32_t source=alias?stack+0x100:0x1c0000;
        Matrix view{},world{};const bool identity=seed%16<4;
        for(unsigned row=0;row<4;++row)for(unsigned column=0;column<4;++column){view[row*4+column]=identity?float(row==column):float(int((seed+row*11+column*7)%17)-8)/8;
            world[row*4+column]=identity?float(row==column):float(int((seed+row*17+column*3)%19)-9)/8;}
        if(identity){world[3]=seed%4==2?4.f:0.f;world[7]=0;world[11]=seed%4==0?4.f:seed%4==1?-4.f:0.f;}
        const uint32_t opaque[]={0x7fc01234,0x7fa01234,0x7f800000,1};view[15]=world[15]=std::bit_cast<float>(opaque[(seed/16)%4]);
        write(x,source,world);write(x,0x1902f0,view);write(x,stack+0x110,std::array<float,4>{0,0,1,1});if(!alias)write(x,stack+0x10c,std::bit_cast<float>(opaque[(seed/16)%4]));
        write(x,0x1840b8,2.f);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);old.view=view;old.mutate=now.mutate=(seed&64) && !alias;
        prepare(old,x,stack);prepare(now,y,stack);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];p.f[i]=float(i)/8;}
        p.r[17].ud[0]=0x1a0000;p.r[21].ud[0]=0x180000;p.r[2].ud[0]=source;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;
        std::memset(p.vu0_vf,0x65,sizeof(p.vu0_vf));std::memset(&p.vu0_acc,0x54,sizeof(p.vu0_acc));p.vu0_status=0x1234;p.vi[0]=0x4567;auto q=p;
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&8?0x8040:0);_mm_setcsr(control);
        x.functions.generated(0x220a20)(x.memory.flat(),&p,&x);const auto fp=_mm_getcsr();_mm_setcsr(control);
        y.functions.generated(0x220a20)(y.memory.flat(),&q,&y);
        check(_mm_getcsr()==fp);_mm_setcsr(saved);p.cycleLimit=q.cycleLimit;
        if(std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes)){std::fprintf(stderr,"facing seed%u mode%x expected=%08x actual=%08x\n",seed,mode,old.result[0],read<Vec>(y,stack+0xd0)[0]);check(false);}
        check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)) && q.r[9].ud[0]==0 && q.vi[0]==0x4567);
        if(identity && !alias){const auto output=read<std::array<float,4>>(y,stack+0xd0);check(output[0]==(seed%4==0?1.f:seed%4==1?-1.f:0.f));check(output[3]==1);++known;}
        check(now.scene.facing(stack,0x1a0000).published);now.scene.finishVisibility(stack,0x180000);old.scene.finishVisibility(stack,0x180000);++callers;
    }
    _mm_setcsr(saved);check(old.sites==now.sites && now.sites.size()==10);
    {auto &rt=b.runtime();constexpr uint32_t stack=0x1ffe000,object=0x1a0000,source=0x1c0000;
        Matrix identity{};for(unsigned i=0;i<4;++i)identity[i*5]=1;write(rt,source,identity);write(rt,0x1902f0,identity);write(rt,stack+0x100,std::array<float,4>{0,0,4,1});write(rt,stack+0x110,std::array<float,4>{0,0,1,1});prepare(now,rt,stack);
        R5900Context c{};c.r[29].uw[0]=stack;c.r[17].uw[0]=object;c.r[21].uw[0]=0x180000;c.r[2].uw[0]=source;c.r[3].uw[0]=stack+0xd0;
        const auto invoke=[&](uint32_t pc){check(vp2s::nativeVisibilityInstruction(&c,&rt,pc)==true);};
        const auto reject=[&](auto fn){const auto before=c;const auto fp=_mm_getcsr();std::vector<uint8_t> ram(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{fn();}catch(const std::runtime_error &){bad=true;}check(bad && _mm_getcsr()==fp && !std::memcmp(&c,&before,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));};
        reject([&]{invoke(0x220a74);});reject([&]{invoke(0x220a94);});invoke(0x220a20);reject([&]{invoke(0x220a20);});reject([&]{now.scene.finishVisibility(stack,0x180000);});
        reject([&]{invoke(0x220a8c);});reject([&]{invoke(0x220a94);});
        c.r[2].uw[0]+=16;reject([&]{invoke(0x220a24);});c.r[2].uw[0]-=16;
        c.r[5].uw[0]=stack+0x110;reject([&]{invoke(0x220a60);});c.r[5].uw[0]=stack+0x100;invoke(0x220a60);
        c.r[3].uw[0]=stack+0x100;reject([&]{invoke(0x220a68);});c.r[3].uw[0]=stack+0x110;invoke(0x220a68);
        for(uint32_t bad:{1u,0x7fa01234u,0x71800000u}){write(rt,stack+0x100,bad);reject([&]{invoke(0x220a74);});check(!now.scene.facing(stack,object).ready);}write(rt,stack+0x100,0.f);
        invoke(0x220a74);reject([&]{invoke(0x220a74);});c.r[17].uw[0]+=16;reject([&]{invoke(0x220a8c);});c.r[17].uw[0]-=16;invoke(0x220a8c);
        c.r[3].uw[0]=stack+0xe0;reject([&]{invoke(0x220a94);});c.r[3].uw[0]=stack+0xd0;invoke(0x220a94);reject([&]{invoke(0x220a94);});now.scene.finishVisibility(stack,0x180000);reject([&]{invoke(0x220a94);});
        for(uint32_t bad:{1u,0x7fa01234u,0x71800000u}){auto invalid=identity;invalid[2]=std::bit_cast<float>(bad);reject([&]{vp2s::objectFacing(identity,invalid,{0,0,4,1},{0,0,1,1});});}
    }
    std::printf("%u bounded facing callers,%u known axis results,10 sites; integer sum/product/quotient and binary-search root oracle, full CPU/RAM/FP, snapshot mutation/aliases/opaque unused W/four modes passed\n",callers,known);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

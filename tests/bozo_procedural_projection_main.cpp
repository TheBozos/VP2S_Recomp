#include "native_procedural.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "projection_functions.h"
#include <bit>
#include <cstdio>
#include <cstring>
#include <immintrin.h>
#include <source_location>
#include <vector>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("procedural projection assertion at "+std::to_string(at.line()));}
template<class T>void put(Runtime &rt,uint32_t at,const T &v){auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(p,&v,sizeof(v));}
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::ProceduralOwner,vp2s::SceneOwner{
    vp2s::NativeProceduralTransforms projections;vp2s::NativeScene scene;
    vp2s::NativeProceduralTransforms &nativeProceduralTransforms()override{return projections;}
    vp2s::NativeScene &nativeScene()override{return scene;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc)override{if(auto answer=vp2s::nativeProceduralInstruction(c,rt,pc))return *answer;throw std::runtime_error("unexpected native site");}
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("library");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device write");}
    bool cop0Condition(bozo::ee::Dispatcher &,R5900Context *)override{throw std::runtime_error("device completion");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
constexpr uint32_t object=0x180000,camera=0x190000;
void cameraInput(Runtime &rt,Host &h){
    put(rt,camera+0x220,uint32_t(0x153120));put(rt,camera+0x8e0,uint32_t(0x1a0000));put(rt,0x1a0000,uint32_t(0x153140));h.scene.selectCamera(&rt,camera);
}
}
int main(){try{
    Host h;bozo::ee::Dispatcher d(h);d.setFunctions(projection_function_table,projection_function_count,projection_resume_table,projection_resume_count);
    d.runtime().port=&h;d.boot(0x2481d0,0x1fff000);auto &rt=d.runtime();const auto saved=_mm_getcsr();unsigned cases=0;
    const uint32_t aliases[]={0,0x20000000,0x30000000,0x80000000,0xa0000000},callers[]={0x202308,0x202bc8,0x203d28},finishes[]={0x202960,0x2038d0,0x2053f8};
    for(unsigned seed=0;seed<512;++seed){
        h.scene=vp2s::NativeScene{};h.projections=vp2s::NativeProceduralTransforms{};std::memset(rt.memory.flat(),seed+1,bozo::kFlatBytes);cameraInput(rt,h);
        const auto stack=seed&1?0x70003d00u:0x1ffe000u|aliases[(seed/2)%5];
        const auto parent=stack+0x170;put(rt,stack+0x20,callers[seed%3]);put(rt,camera+0x23c,uint32_t(seed*3));put(rt,camera+0x240,uint32_t(seed*7));
        std::array<uint32_t,16> matrix;for(unsigned i=0;i<16;++i)matrix[i]=std::bit_cast<uint32_t>(float(int(seed%17)-8)+float(i+1)/32);
        if(seed&4)matrix[11]=0x4d200821;put(rt,stack+0x30,matrix);
        R5900Context c{};for(unsigned i=1;i<32;++i){c.r[i].ud[0]=0x1234000000000000ull+seed*32+i;c.r[i].ud[1]=~c.r[i].ud[0];}
        std::memset(c.vu0_vf,0xa5,sizeof(c.vu0_vf));std::memset(&c.vu0_acc,0x5a,sizeof(c.vu0_acc));c.vu0_status=0x12345678;c.vu0_q_bits=0x87654321;c.vi[0]=0x7f00;
        c.r[16].ud[0]=camera;c.r[17].ud[0]=object;c.r[29].ud[0]=stack;c.r[31].ud[0]=0x2481d0;
        put(rt,stack,c.r[16]);put(rt,stack+0x10,c.r[17]);const auto before=c;
        std::vector<uint8_t> expected(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
        const auto patch=[&](uint32_t at,const void *source,unsigned bytes){auto *ptr=rt.memory.hostPointer(at,bytes);check(ptr);std::memcpy(expected.data()+(ptr-rt.memory.flat()),source,bytes);};
        patch(stack+0x70,matrix.data(),64);std::array<uint32_t,16> transposed;for(unsigned row=0;row<4;++row)for(unsigned col=0;col<4;++col)transposed[4*col+row]=matrix[4*row+col];patch(stack+0x130,transposed.data(),64);
        const uint32_t low=seed*3*16,high=seed*7*16;patch(object+0x238,&low,4);patch(object+0x23c,&high,4);
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|((seed%4)<<13)|(seed&8?0x8040:0);_mm_setcsr(control);
        rt.functions.generated(0x2481d0)(rt.memory.flat(),&c,&rt);check(_mm_getcsr()==control);_mm_setcsr(saved);
        check(c.pc==callers[seed%3] && c.r[29].uw[0]==parent && c.r[31].uw[0]==c.pc && !std::memcmp(expected.data(),rt.memory.flat(),expected.size()));
        check(!std::memcmp(c.vu0_vf,before.vu0_vf,sizeof(c.vu0_vf)) && !std::memcmp(&c.vu0_acc,&before.vu0_acc,sizeof(c.vu0_acc)) && c.vu0_status==before.vu0_status && c.vu0_q_bits==before.vu0_q_bits && c.vi[0]==before.vi[0]);
        const auto held=h.projections.projection(parent,object);check(h.projections.pending()==1 && held.object==object && held.camera==camera);
        for(unsigned i=0;i<16;++i)check(std::bit_cast<uint32_t>(held.screenFromLocal[i])==matrix[i]);
        std::memset(rt.memory.hostPointer(stack+0x130,64),0xff,64);check(h.projections.projection(parent,object).screenFromLocal==held.screenFromLocal);
        R5900Context finish{};finish.pc=0x247ec0;finish.r[31].uw[0]=finishes[seed%3];finish.r[29].uw[0]=parent;finish.r[4].uw[0]=object;
        check(vp2s::nativeProceduralInstruction(&finish,&rt,0x247ec0)==false && !h.projections.pending());
        bool rejected=false;try{h.projections.projection(parent,object);}catch(const std::runtime_error &){rejected=true;}check(rejected);++cases;
    }
    for(unsigned which=0;which<13;++which){
        h.scene=vp2s::NativeScene{};h.projections=vp2s::NativeProceduralTransforms{};std::memset(rt.memory.flat(),0,bozo::kFlatBytes);cameraInput(rt,h);
        constexpr uint32_t stack=0x1ffe000,parent=stack+0x170;put(rt,stack+0x20,uint32_t(0x202308));std::array<float,16> matrix{};matrix[0]=matrix[5]=matrix[10]=matrix[15]=1;put(rt,stack+0x130,matrix);
        R5900Context c{};c.pc=0x2481d0;c.r[31].uw[0]=0x2481d0;c.r[29].uw[0]=stack;c.r[4].uw[0]=stack+0x130;c.r[16].uw[0]=camera;c.r[17].uw[0]=object;
        if(which==0)c.pc=0;else if(which==1)c.r[31].uw[0]=0;else if(which==2)c.r[4].uw[0]+=16;else if(which==3)put(rt,stack+0x20,uint32_t(0));
        else if(which==4)c.r[16].uw[0]+=16;else if(which==5)h.scene=vp2s::NativeScene{};
        else if(which==6)put(rt,stack+0x130,uint32_t(0x7fa01234));else if(which==7)put(rt,stack+0x130,uint32_t(1));
        else if(which>=8)h.projections.publish(&rt,&c,h.scene);
        if(which>=9){c.pc=0x247ec0;c.r[31].uw[0]=0x202960;c.r[29].uw[0]=parent;c.r[4].uw[0]=object;
            if(which==9)c.r[31].uw[0]=0;else if(which==10)c.r[29].uw[0]+=16;else if(which==11)c.r[4].uw[0]+=16;else h.projections.finish(parent,object);}
        const auto before=c;const auto count=h.projections.pending();const auto control=_mm_getcsr();const auto ram=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        try{vp2s::nativeProceduralInstruction(&c,&rt,which>=9?0x247ec0:0x24826c);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()) && h.projections.pending()==count && _mm_getcsr()==control);
    }
    _mm_setcsr(saved);std::printf("%u compiled scalar projection publications: independent transpose/copy RAM oracle, immutable typed rows, opaque processor state, aliases/scratch/FP, finish lifetime and13 readonly rejections passed\n",cases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

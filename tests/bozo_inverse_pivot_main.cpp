#include "native_math.h"
#include "native_float_oracle.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "inverse_pivot_functions.h"
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <set>
#include <source_location>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("inverse pivot assertion at "+std::to_string(at.line()));}
struct Host:bozo::ee::Host,bozo::port::Host {
    bool reference=false;std::set<uint32_t> sites;
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("library");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        sites.insert(pc);if(!reference){check(vp2s::nativeMathInstruction(r,c,rt,pc));return true;}
        if(pc==0x216f54){
            std::array<std::array<uint32_t,4>,4> matrix,weights;uint32_t maximum[4]={};
            std::memcpy(matrix.data(),rt->memory.hostPointer(c->r[4].uw[0],64),64);
            for(unsigned row=0;row<4;++row)for(unsigned lane=0;lane<4;++lane){matrix[row][lane]&=0x7fffffff;maximum[lane]=std::max(maximum[lane],matrix[row][lane]);}
            const auto saved=_mm_getcsr();_mm_setcsr((saved&~_MM_ROUND_MASK)|_MM_ROUND_TOWARD_ZERO);
            for(unsigned row=0;row<4;++row){const auto scale=vp2s::test::quotient(0x3f800000,maximum[row]);
                for(unsigned lane=0;lane<4;++lane)weights[row][lane]=vp2s::test::product(matrix[row][lane],scale);}
            _mm_setcsr(saved);std::memcpy(rt->memory.hostPointer(c->r[29].uw[0],64),weights.data(),64);
        }
        return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("call");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host old,now;old.reference=true;bozo::ee::Dispatcher a(old),b(now);a.runtime().port=&old;b.runtime().port=&now;
    for(auto *d:{&a,&b})d->setFunctions(inverse_pivot_function_table,inverse_pivot_function_count,inverse_pivot_resume_table,inverse_pivot_resume_count);
    const auto saved=_mm_getcsr();unsigned callers=0,known=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime();auto &y=b.runtime();std::memset(x.memory.flat(),0,bozo::kFlatBytes);
        const uint32_t aliases[5]={0,0x20000000,0x30000000,0x80000000,0xa0000000},alias=aliases[(seed/16)%5];
        const auto input=seed&8?0x70002000u:alias|0x180000u;
        const auto output=(seed/8)%4==0?input:(seed/8)%4==1?input+16:(seed/8)%4==2?input+32:(seed&8?0x70002100u:alias|0x190000u);
        const auto top=seed&1?0x70003f00u:0x1fff000u;float matrix[4][4]{};const unsigned kind=seed%8;
        for(unsigned row=0;row<4;++row)matrix[row][row]=1;
        if(kind==0){matrix[0][1]=-0.f;matrix[2][0]=-0.f;}
        if(kind==1){matrix[0][0]=2;matrix[1][1]=4;matrix[2][2]=8;matrix[3][0]=float(int(seed%13)-6)*16;matrix[3][1]=24;matrix[3][2]=-32;}
        if(kind==2){std::memset(matrix,0,sizeof(matrix));for(unsigned row=0;row<4;++row)matrix[row][(row+1)%4]=float(1u<<row);}
        if(kind==3){for(unsigned row=0;row<4;++row)for(unsigned lane=0;lane<4;++lane)matrix[row][lane]=row==lane?8.f:float(int((seed+row*3+lane*5)%13)-6)/16;}
        if(kind==4)matrix[0][0]=0x1p-20f;
        if(kind==5)std::memset(matrix,0,sizeof(matrix));
        if(kind==6)for(auto &row:matrix)for(float &v:row)v=1;
        if(kind==7)for(unsigned row=0;row<4;++row)matrix[row][row]=row&1?-float(row+1):float(row+1);
        std::memcpy(x.memory.hostPointer(input,64),matrix,64);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        p.r[4].ud[0]=input;p.r[5].ud[0]=output;p.r[29].ud[0]=top;p.r[31].ud[0]=0x170000;
        std::memset(p.vu0_vf,0x6a,sizeof(p.vu0_vf));p.vu0_status=0x9876;auto q=p;
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);_mm_setcsr(control);
        x.functions.generated(0x216f50)(x.memory.flat(),&p,&x);const auto after=_mm_getcsr();_mm_setcsr(control);
        y.functions.generated(0x216f50)(y.memory.flat(),&q,&y);
        if(_mm_getcsr()!=after || std::memcmp(&p,&q,sizeof(p)) || std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes)){
            size_t at=0;while(at<bozo::kFlatBytes && x.memory.flat()[at]==y.memory.flat()[at])++at;
            std::fprintf(stderr,"inverse seed%u mode%x FP%x/%x RAM%zx\n",seed,mode,after,_mm_getcsr(),at);check(false);}
        check(p.pc==0x170000 && p.r[29].uw[0]==top);++callers;
        if(kind!=5 && kind!=6){float inverse[4][4];std::memcpy(inverse,y.memory.hostPointer(output,64),64);
            for(unsigned row=0;row<4;++row)for(unsigned col=0;col<4;++col){double sum=0;
                for(unsigned k=0;k<4;++k)sum+=double(matrix[row][k])*inverse[k][col];
                if(std::abs(sum-double(row==col))>0.0002){std::fprintf(stderr,"inverse residual seed%u %u,%u = %g\n",seed,row,col,sum);check(false);}}
            ++known;}
    }
    // All rejection checks happen before any temporary publication.
    auto &rt=b.runtime();const uint32_t input=0x180000,output=0x190000,stack=0x1ffe000;
    float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    for(unsigned which=0;which<8;++which){std::memset(rt.memory.flat(),0,bozo::kFlatBytes);std::memcpy(rt.memory.hostPointer(input,64),identity,64);
        R5900Context c{};c.pc=0x216f50;c.r[4].ud[0]=input;c.r[5].ud[0]=output;c.r[29].ud[0]=stack;
        if(which<3){const uint32_t bad[3]={0x7fc00000,1,0x71800000};std::memcpy(rt.memory.hostPointer(input,4),bad+which,4);}
        else if(which==3)c.pc=0x216f58;else if(which==4)c.r[4].ud[0]=stack;else if(which==5)c.r[5].ud[0]=stack+0x40;
        else if(which==6)c.r[5].ud[0]=output+4;else c.r[4].ud[0]=0x1fffff0;
        const auto before=c;const auto bytes=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        const auto fp=_mm_getcsr();try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x216f54);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && _mm_getcsr()==fp && !std::memcmp(&c,&before,sizeof(c)) && !std::memcmp(bytes.data(),rt.memory.flat(),bytes.size()));
    }
    _mm_setcsr(saved);check(old.sites==now.sites && now.sites.size()==35);
    std::printf("%u complete scalar inverses, %u independent inverse identities, 35 sites; CPU/RAM/FP, pivot rational oracle, four modes, aliases, in-place/partial outputs, singular inputs and mutation-free guards passed\n",callers,known);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

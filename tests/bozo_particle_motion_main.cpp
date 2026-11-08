#include "native_math.h"
#include "native_float_oracle.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "particle_motion_original_functions.h"
#include "particle_motion_adapted_functions.h"
#include <bit>
#include <cstdio>
#include <cstring>
#include <set>
#include <source_location>
#include <stdexcept>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("particle motion assertion at "+std::to_string(at.line()));}
template<class T>T read(Runtime &rt,uint32_t at){T v;auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(&v,p,sizeof(v));return v;}
template<class T>void write(Runtime &rt,uint32_t at,T v){auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(p,&v,sizeof(v));}
struct Host:bozo::ee::Host,bozo::port::Host {
    bool reference=false;std::set<uint32_t> sites;unsigned publications=0;std::array<unsigned,4> paths{};
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        sites.insert(pc);const bool first=pc==0x408fb8 || pc==0x40909c || pc==0x40932c || pc==0x40957c;
        const bool second=pc==0x408fbc || pc==0x4090a0 || pc==0x409334 || pc==0x409584;
        if(first){++publications;++paths[pc==0x408fb8?0:pc==0x40909c?1:pc==0x40932c?2:3];}
        if(!reference){check(vp2s::nativeMathInstruction(r,c,rt,pc));return true;}
        if(first || second){const auto object=c->r[4].uw[0],index=c->r[24].uw[0]-1,source=read<uint32_t>(*rt,object+0x14)+index*0x90+0x60;
            auto value=read<std::array<uint32_t,4>>(*rt,source);const auto velocity=read<std::array<uint32_t,4>>(*rt,source+16);const auto time=read<uint32_t>(*rt,0x35dee8);
            for(unsigned i=0;i<3;++i)value[i]=vp2s::test::sum(vp2s::test::sum(value[i],0),vp2s::test::product(velocity[i],time));
            const auto destination=first?c->r[6].uw[0]+0x20:source;write(*rt,destination,value);
        }return true;
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
    Host old,now;old.reference=true;bozo::ee::Dispatcher a(old),b(now);
    a.setFunctions(particle_motion_original_function_table,particle_motion_original_function_count,particle_motion_original_resume_table,particle_motion_original_resume_count);
    b.setFunctions(particle_motion_adapted_function_table,particle_motion_adapted_function_count,particle_motion_adapted_resume_table,particle_motion_adapted_resume_count);
    a.runtime().port=&old;b.runtime().port=&now;const auto saved=_mm_getcsr();unsigned callers=0,published=0;uint32_t random=0x408e90;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x51,bozo::kFlatBytes);
        constexpr uint32_t object=0x180000,pool=0x190000,maskOwner=0x1a0000,mask=0x1b0000,history=0x1c0000,script=0x1e0000,keys=0x1f0000;
        std::memset(x.memory.hostPointer(object,0xe4),0,0xe4);const auto profile=seed/32,group=profile&1?2+seed%3:1;
        write(x,object+0xc,uint32_t(130));write(x,object+0x10,uint32_t(group));write(x,object+0x14,pool);write(x,object+0x1c,maskOwner);write(x,maskOwner+8,mask);write(x,object+0x20,history);
        const uint32_t starts[]={0,63,64,127};write(x,object+0x48,starts[(seed/4)%4]);write(x,object+0x4c,float(seed%3));
        write(x,object+0xd8,uint8_t(profile<2?1:0));write(x,object+0xd0,uint8_t(seed%7==0?4:0));write(x,object+0xdc,script);write(x,object+0xe0,keys);
        write(x,script,uint8_t(4));write(x,script+3,uint8_t(seed%3));for(unsigned i=0;i<4;++i)write(x,keys+i*80+12,30.f+float(i)*30);
        const float steps[]={0,.125f,.25f,1,2,-.125f,.1f,.33333334f};write(x,0x35dee8,steps[seed%8]);
        for(unsigned i=0;i<3;++i){random=random*1664525u+1013904223u;uint64_t flags=seed%8==0?0:seed%8==1?UINT64_MAX:(uint64_t(random)<<32)|uint64_t(random^0x97ab5312u);write(x,mask+i*8,flags);}
        for(unsigned i=0;i<130;++i){const auto at=pool+i*0x90+0x60;std::array<uint32_t,8> values;
            for(unsigned j=0;j<8;++j){random=random*1664525u+1013904223u;values[j]=(random&0x807fffff)|((j<4?120u:110u)<<23);}
            if(seed%11==0)for(unsigned j=0;j<3;++j){values[j]=j&1?0x80000000u:0;values[j+4]=0x00800000;}
            if(seed==1){const float known[8]={3,4,5,0,2,-4,.5f,0};std::memcpy(values.data(),known,32);}
            values[3]=0x7fc01234;values[7]=0xffc05678;write(x,at,values);
            write(x,at+0x20,uint32_t(seed&1));write(x,at+0x24,uint32_t(i%4));write(x,at+0x28,.0625f);write(x,at+0x2c,uint32_t(seed%2));
        }
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);R5900Context p{};
        for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];p.f[i]=float(i)/8;}
        std::memset(p.vu0_vf,0x47,sizeof(p.vu0_vf));std::memset(&p.vu0_acc,0x58,sizeof(p.vu0_acc));p.vu0_status=0x1234;
        p.r[4].ud[0]=object;p.r[5].ud[0]=(seed%12)*group;p.r[6].ud[0]=seed&1?0x70000100:0x230000;p.r[7].ud[0]=seed&1;
        p.r[29].ud[0]=seed&2?0x70003f00:0x1fff000;p.r[31].ud[0]=0x170000;auto q=p;
        old.publications=now.publications=0;const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);_mm_setcsr(control);
        x.functions.generated(0x408e90)(x.memory.flat(),&p,&x);const auto fp=_mm_getcsr();_mm_setcsr(control);
        try{y.functions.generated(0x408e90)(y.memory.flat(),&q,&y);}catch(const std::exception &e){std::fprintf(stderr,"particle motion seed=%u mode=%x: %s\n",seed,mode,e.what());throw;}
        check(_mm_getcsr()==fp);_mm_setcsr(saved);p.cycleLimit=q.cycleLimit;
        check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        check(old.publications==now.publications);published+=now.publications;++callers;
        if(seed==1){const auto result=read<std::array<uint32_t,4>>(y,pool+0x60);const std::array<uint32_t,4> expected{0x40500000,0x40600000,0x40a20000,0x7fc01234};check(result==expected && now.publications==1 && q.r[24].uw[0]==1);}
    }
    check(old.sites==now.sites && now.sites.size()==26 && published);
    for(unsigned i=0;i<4;++i)check(now.paths[i] && now.paths[i]==old.paths[i]);
    {auto &rt=b.runtime();constexpr uint32_t object=0x180000,source=0x190060,output=0x70000120;
        write(rt,object+0xc,uint32_t(1));write(rt,object+0x10,uint32_t(1));write(rt,object+0x14,uint32_t(0x190000));write(rt,object+0xd8,uint8_t(1));write(rt,0x35dee8,.25f);
        const std::array<uint32_t,8> inputs{0x3f800000,0x40000000,0x40400000,0x7fc01234,0x3f000000,0xbe800000,0,0xffc05678};write(rt,source,inputs);
        R5900Context c{};c.pc=0x408e90;c.r[4].uw[0]=object;c.r[25].uw[0]=1;c.r[24].uw[0]=1;c.r[10].uw[0]=source;c.r[6].uw[0]=output-0x20;c.r[3].uw[0]=output;c.r[29].uw[0]=0x1ffe000;
        auto reject=[&]{const auto before=c;const auto fp=_mm_getcsr();std::vector<uint8_t> ram(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x408fb8);}catch(const std::runtime_error &){bad=true;}
            check(bad && _mm_getcsr()==fp && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));};
        c.pc=0x170000;reject();c.pc=0x408e90;c.r[10].uw[0]+=16;reject();c.r[10].uw[0]-=16;c.r[3].uw[0]+=16;reject();c.r[3].uw[0]-=16;
        c.r[24].uw[0]=0;reject();c.r[24].uw[0]=2;reject();c.r[24].uw[0]=1;write(rt,object+0xd8,uint8_t(0));reject();write(rt,object+0xd8,uint8_t(1));
        c.r[6].uw[0]=source-0x20;c.r[3].uw[0]=source;reject();c.r[6].uw[0]=c.r[29].uw[0]-0x20;c.r[3].uw[0]=c.r[29].uw[0];reject();c.r[6].uw[0]=output-0x20;c.r[3].uw[0]=output;
        for(uint32_t at:{source,source+16,0x35dee8u})for(uint32_t bad:{1u,0x7fc01234u,0x7f7fffffu}){const auto previous=read<uint32_t>(rt,at);write(rt,at,bad);reject();write(rt,at,previous);}
        check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x408fb8));check(read<uint32_t>(rt,output+12)==inputs[3]);
        const auto expected=read<std::array<uint32_t,4>>(rt,output);check(vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,0x408fbc));check(read<std::array<uint32_t,4>>(rt,source)==expected);
    }
    std::printf("%u complete particle motion/animation callers,%u paired position publications,26 sites; integer oracle, full CPU/RAM/FP, bitset boundaries and four modes/FTZ/DAZ\n",callers,published);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

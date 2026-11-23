#include "native_math.h"
#include "native_float_oracle.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "particle_motion2_original_functions.h"
#include "particle_motion2_adapted_functions.h"
#include <algorithm>
#include <bit>
#include <cstdio>
#include <cstring>
#include <set>
#include <source_location>
#include <stdexcept>
// The second emitter routine 408620 (mod001 copy 277290, battle 0x1b): its 26 vector sites native against an integer
// oracle. Points are 0x60 apart: position +0x30, velocity +0x40, animation state +0x50.
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("particle motion2 assertion at "+std::to_string(at.line()));}
template<class T>T read(Runtime &rt,uint32_t at){T v;auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(&v,p,sizeof(v));return v;}
template<class T>void write(Runtime &rt,uint32_t at,T v){auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(p,&v,sizeof(v));}
// Each role's register: t = QMTC2's GPR (the timestep), P / V = the loads' bases, 1 = the output store's, 2 = the point's.
struct Role{uint32_t pc;char role;unsigned reg;};
constexpr Role roles[]={{0x4086a0,'t',3},{0x408714,'P',10},{0x408718,'V',3},{0x408748,'1',3},{0x40874c,'2',10},{0x4087f4,'P',3},{0x4087f8,'V',9},{0x40882c,'1',9},{0x408830,'2',3},{0x4088a8,'t',16},{0x408950,'P',22},{0x408958,'V',16},{0x408abc,'1',16},{0x408ac4,'2',22},{0x408b90,'P',21},{0x408b94,'V',16},{0x408d0c,'1',16},{0x408d14,'2',21}};
struct Host:bozo::ee::Host,bozo::port::Host {
    bool reference=false;std::set<uint32_t> sites;unsigned publications=0;std::array<unsigned,4> paths{};
    uint32_t time=0,point=0;std::array<uint32_t,4> position{},velocity{},result{};
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        sites.insert(pc);const auto role=std::find_if(std::begin(roles),std::end(roles),[&](const Role &x){return x.pc==pc;});
        if(role!=std::end(roles) && role->role=='1'){++publications;++paths[pc<0x408800?0:pc<0x408900?1:pc<0x408b00?2:3];}
        if(!reference){check(vp2s::nativeMathInstruction(r,c,rt,pc));return true;}
        if(role==std::end(roles))return true;   // folded arithmetic
        const auto at=c->r[role->reg].uw[0];
        switch(role->role){
        case 't':time=at;check(time==read<uint32_t>(*rt,0x35dee8));break;
        case 'P':point=at;position=read<std::array<uint32_t,4>>(*rt,at);break;
        case 'V':velocity=read<std::array<uint32_t,4>>(*rt,at);check(at==point+16);result=position;
            for(unsigned i=0;i<3;++i)result[i]=vp2s::test::sum(vp2s::test::sum(position[i],0),vp2s::test::product(velocity[i],time));break;
        default:check(role->role=='1' || at==point);write(*rt,role->role=='1'?at:point,result);
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
    Host old,now;old.reference=true;bozo::ee::Dispatcher a(old),b(now);
    a.setFunctions(particle_motion2_original_function_table,particle_motion2_original_function_count,particle_motion2_original_resume_table,particle_motion2_original_resume_count);
    b.setFunctions(particle_motion2_adapted_function_table,particle_motion2_adapted_function_count,particle_motion2_adapted_resume_table,particle_motion2_adapted_resume_count);
    a.runtime().port=&old;b.runtime().port=&now;const auto saved=_mm_getcsr();unsigned callers=0,published=0;uint32_t random=0x408620;
    // The native sites check their instruction words (other overlays share these addresses): the fixture's own words.
    static constexpr uint32_t code[][2]={{0x4086a0,0x48a31800},{0x408714,0xd9410000},{0x408718,0xd8620000},{0x408738,0x4bc00abc},{0x40873c,0x4bc31048},{0x408748,0xf8610000},{0x40874c,0xf9410000},{0x4087f4,0xd8610000},{0x4087f8,0xd9220000},{0x40881c,0x4bc00abc},{0x408820,0x4bc31048},{0x40882c,0xf9210000},{0x408830,0xf8610000},{0x4088a8,0x48b01800},{0x408950,0xdac10000},{0x408958,0xda020000},{0x408aac,0x4bc00abc},{0x408ab0,0x4bc31048},{0x408abc,0xfa010000},{0x408ac4,0xfac10000},{0x408b90,0xdaa10000},{0x408b94,0xda020000},{0x408cfc,0x4bc00abc},{0x408d00,0x4bc31048},{0x408d0c,0xfa010000},{0x408d14,0xfaa10000}};
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x51,bozo::kFlatBytes);for(const auto &[at,word]:code)write(x,at,word);
        constexpr uint32_t object=0x180000,pool=0x190000,maskOwner=0x1a0000,mask=0x1b0000,history=0x1c0000,script=0x1e0000,keys=0x1f0000;
        std::memset(x.memory.hostPointer(object,0xf0),0,0xf0);const auto profile=seed/32,group=profile&1?2+seed%3:1;
        write(x,object+0xc,uint32_t(130));write(x,object+0x10,uint32_t(group));write(x,object+0x14,pool);write(x,object+0x1c,maskOwner);write(x,maskOwner+8,mask);write(x,object+0x20,history);
        const uint32_t starts[]={0,63,64,127};write(x,object+0x48,starts[(seed/4)%4]);write(x,object+0x4c,float(seed%3));
        write(x,object+0xd8,uint8_t(profile<2?1:0));write(x,object+0xd0,uint8_t(seed%7==0?4:0));write(x,object+0xdc,script);write(x,object+0xe0,keys);
        write(x,script,uint8_t(4));write(x,script+3,uint8_t(seed%3));for(unsigned i=0;i<4;++i)write(x,keys+i*80+12,30.f+float(i)*30);
        const float steps[]={0,.125f,.25f,1,2,-.125f,.1f,.33333334f};write(x,0x35dee8,steps[seed%8]);
        for(unsigned i=0;i<3;++i){random=random*1664525u+1013904223u;uint64_t flags=seed%8==0?0:seed%8==1?UINT64_MAX:(uint64_t(random)<<32)|uint64_t(random^0x97ab5312u);write(x,mask+i*8,flags);}
        for(unsigned i=0;i<130;++i){const auto at=pool+i*0x60+0x30;std::array<uint32_t,8> values;
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
        x.functions.generated(0x408620)(x.memory.flat(),&p,&x);const auto fp=_mm_getcsr();_mm_setcsr(control);
        try{y.functions.generated(0x408620)(y.memory.flat(),&q,&y);}catch(const std::exception &e){std::fprintf(stderr,"particle motion2 seed=%u mode=%x: %s\n",seed,mode,e.what());throw;}
        check(_mm_getcsr()==fp);_mm_setcsr(saved);p.cycleLimit=q.cycleLimit;
        if(std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes)){const auto *u=x.memory.flat(),*v=y.memory.flat();size_t i=0;while(u[i]==v[i])++i;
            std::fprintf(stderr,"seed %u mode %x: RAM differs first at %zx\n",seed,mode,i);}
        check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        check(old.publications==now.publications);published+=now.publications;++callers;
        if(seed==1){const auto result=read<std::array<uint32_t,4>>(y,pool+0x30);const std::array<uint32_t,4> expected{0x40500000,0x40600000,0x40a20000,0x7fc01234};check(result==expected && now.publications==1);}
    }
    check(old.sites==now.sites && now.sites.size()==26 && published);
    for(unsigned i=0;i<4;++i)check(now.paths[i] && now.paths[i]==old.paths[i]);
    std::printf("%u complete particle motion (408620) callers,%u paired position publications,26 sites; integer oracle, full CPU/RAM/FP, four modes/FTZ/DAZ\n",callers,published);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

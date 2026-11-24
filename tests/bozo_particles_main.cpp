#include "native_particles.h"
#include "native_math.h"
#include "native_float_oracle.h"
#include "bozo/ops_vu0.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "particles_original_functions.h"
#include "particles_adapted_functions.h"
#include <cstdio>
#include <bit>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <immintrin.h>
#include <set>
#include <source_location>
#include <stdexcept>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("particle assertion at "+std::to_string(at.line()));}
template<class T>T read(Runtime &rt,uint32_t at){T v;auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(&v,p,sizeof(v));return v;}
template<class T>void write(Runtime &rt,uint32_t at,T v){auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(p,&v,sizeof(v));}
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::ParticleOwner,vp2s::AssetOwner {
    bool reference=false,ready=true,dirty=true;vp2s::NativeParticles particles;vp2s::NativeAssets assets;std::set<uint32_t> sites;
    unsigned total=0,emitted=0,group=1,publications=0,points=0,finished=0;
    std::optional<vp2s::ParticleBounds> box;
    vp2s::NativeParticles &nativeParticles()override{return particles;}
    vp2s::NativeAssets &nativeAssets()override{return assets;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc)override{
        sites.insert(pc);
        if(pc==0x22ee88){++publications;points+=c->r[2].uw[0];}
        if(!reference){const auto handled=vp2s::nativeParticleInstruction(c,rt,pc);check(handled.has_value());return *handled;}
        if(pc==0x22eae0){box.reset();return true;}
        if(pc==0x22ec9c){box=vp2s::ParticleBounds{read<std::array<float,4>>(*rt,0x35dbc0),read<std::array<float,4>>(*rt,0x35dbe0)};return true;}
        if(pc==0x22eca0 || pc==0x22ee94 || pc==0x22ee9c || pc==0x22eea4 || pc==0x22eebc)return true;
        if(std::find(std::begin(vp2s::particleSphereSites),std::end(vp2s::particleSphereSites),pc)!=std::end(vp2s::particleSphereSites)){
            if(pc==0x22f018){check(box.has_value());const auto fp=_mm_getcsr();
                const auto bits=[](float v){return std::bit_cast<uint32_t>(v);};std::array<uint32_t,4> sphere;uint32_t square[3];
                const auto padding=read<uint32_t>(*rt,c->r[29].uw[0]+0xf0);
                for(unsigned i=0;i<3;++i){sphere[i]=vp2s::test::product(vp2s::test::sum(bits(box->minimum[i]),bits(box->maximum[i])),bits(box->maximum[3]));
                    const auto extent=vp2s::test::sum(vp2s::test::sum(bits(box->maximum[i]),sphere[i]^0x80000000u),padding);square[i]=vp2s::test::product(extent,extent);}
                const auto sum=vp2s::test::sum(vp2s::test::sum(square[0],square[1]),vp2s::test::product(0x3f800000,square[2]));
                volatile float root=static_cast<float>(std::sqrt(std::abs(double(std::bit_cast<float>(sum)))));sphere[3]=vp2s::test::product(0x3f800000,bits(root));
                const auto object=c->r[21].uw[0];write(*rt,object+0xa40,box->minimum);write(*rt,object+0xa50,box->maximum);write(*rt,object+0xa60,sphere);_mm_setcsr(fp);
            }return true;
        }
        if(pc==0x22f0a0)return false;
        check(pc==0x22ee88);const auto stack=c->r[29].uw[0],to=read<uint32_t>(*rt,stack+0xe0),source=c->r[16].uw[0];
        const auto count=read<uint32_t>(*rt,source+0x10);check(count==c->r[2].uw[0]);
        // Independent typed point-chunk publication. No device register or
        // packed transfer command is inspected by either test path.
        const auto bytes=32+count*48;std::vector<uint8_t> record(bytes);std::memcpy(record.data(),rt->memory.hostPointer(source,bytes),bytes);
        std::memcpy(rt->memory.hostPointer(to,bytes),record.data(),bytes);return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        auto &rt=d.runtime();const auto object=c->r[4].uw[0];
        // Bounded software emitter/pose callbacks. They isolate the original
        // complete buffer writer, not the emitter's animation/physics internals.
        if(pc==0x172000)c->r[2].ud[0]=ready; // ready
        else if(pc==0x172010 || pc==0x2a1230 || pc==0x137d90 || pc==0x137df8){}
        else if(pc==0x172020)c->r[2].ud[0]=dirty;
        else if(pc==0x172030)c->r[2].ud[0]=group;
        else if(pc==0x172040)emitted=0; // begin substep
        else if(pc==0x172050 || pc==0x172060){}
        else if(pc==0x172070){check(object==0x190000 && c->r[7].uw[0]==0 && c->r[6].uw[0]>=0x70000020 && c->r[6].uw[0]<=0x70000a20);
            const auto count=std::min(c->r[5].uw[0],total-emitted);check(count%group==0);
            for(unsigned i=0;i<count;++i){std::array<uint32_t,12> point;
                for(unsigned j=0;j<12;++j)point[j]=0x3f000000u+(emitted+i)*192+j*17;point[3]=0x3f800000;point[11]=(emitted+i)%16;
                std::memcpy(rt.memory.hostPointer(c->r[6].uw[0]+i*48,48),point.data(),48);
                std::array<float,4> position;std::memcpy(position.data(),point.data(),16);
                if(reference){if(box)for(unsigned j=0;j<3;++j){box->minimum[j]=std::min(box->minimum[j],position[j]);box->maximum[j]=std::max(box->maximum[j],position[j]);}}
                else particles.includePoint(c->r[29].uw[0],object,position);
            }
            emitted+=count;c->r[2].ud[0]=count;
        }
        else if(pc==0x172080){}
        else if(pc==0x172090)++finished;
        else if(pc==0x1720a0)c->f[0]=.125f;
        else throw std::runtime_error("unsupported particle software callback");
        c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device write");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
// cls: the owner class (154180, or battle 0x1b's 175920, a 1725d0 subclass with the same producer).
void initialize(Runtime &rt,unsigned index,uint32_t cls=0x154180){
    constexpr uint32_t object=0x180000,emitter=0x190000,table=0x1a0000;
    std::memset(rt.memory.flat(),0x51,bozo::kFlatBytes);std::memset(rt.memory.hostPointer(object,0xb10),0,0xb10);
    std::memset(rt.memory.hostPointer(emitter,0x100),0,0x100);std::memset(rt.memory.hostPointer(table,0x200),0,0x200);
    write(rt,object,cls);write(rt,cls+0x64,uint32_t(0x172010));
    write(rt,object+0xa90,emitter);write(rt,emitter,table);write(rt,object+0xaa0,uint32_t(0x200000));write(rt,object+0xaa4,uint32_t(0x220000));
    write(rt,object+0xaf9,uint8_t(index));write(rt,object+0xaff,uint8_t(3));write(rt,object+0xad0,1.f);
    write(rt,0x35ded8,1.f);write(rt,0x35dee8,.25f);write(rt,0x35ddc8,uint32_t(0x1b0000));
    write(rt,table+0xf0,uint32_t(0x172000));write(rt,table+0x2c,uint32_t(0x172020));write(rt,table+0x38,uint32_t(0x172030));
    write(rt,table+0xc,uint32_t(0x172040));write(rt,table+0x10,uint32_t(0x172050));write(rt,table+0x14,uint32_t(0x172060));
    write(rt,table+0x18,uint32_t(0x172070));write(rt,table+0xfc,uint32_t(0x172080));write(rt,table+0x28,uint32_t(0x172090));
    write(rt,table+0x48,uint32_t(0x1720a0));
    write(rt,0x35dbc0,std::array<uint32_t,4>{0x7f800000,0x7f800000,0x7f800000,0x7fa01234});
    write(rt,0x35dbe0,std::array<uint32_t,4>{0xff800000,0xff800000,0xff800000,0x3f000000});
}
}
int main(){try{
    Host old,now;old.reference=true;bozo::ee::Dispatcher a(old),b(now);
    a.setFunctions(particles_original_function_table,particles_original_function_count,particles_original_resume_table,particles_original_resume_count);
    b.setFunctions(particles_adapted_function_table,particles_adapted_function_count,particles_adapted_resume_table,particles_adapted_resume_count);
    a.runtime().port=&old;b.runtime().port=&now;
    for(auto *d:{&a,&b}){for(uint32_t pc=0x172000;pc<0x1720b0;pc+=0x10)d->bindLibrary(pc);for(uint32_t pc:{0x2a1230u,0x137d90u,0x137df8u})d->bindLibrary(pc);}
    const auto saved=_mm_getcsr();unsigned callers=0,chunks=0,points=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();initialize(x,seed&1,seed&8?0x175920u:0x154180u);
        write(x,0x190004,uint8_t(seed&4?1:0));write(x,0x180ab4,float(seed%3)/16);write(x,0x180ab0,float(seed%5)/8);write(x,0x180ab8,float(seed%7)/4);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        old.group=now.group=1+seed%3;old.total=now.total=(seed%20)*old.group;
        old.publications=now.publications=old.points=now.points=old.finished=now.finished=0;
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];p.f[i]=float(i)/8;}
        std::memset(p.vu0_vf,0x45,sizeof(p.vu0_vf));std::memset(&p.vu0_acc,0x56,sizeof(p.vu0_acc));p.vu0_status=0x1234;
        p.r[4].ud[0]=0x180000;p.r[29].ud[0]=seed&2?0x70003f00:0x1fff000;p.r[31].ud[0]=0x170000;auto q=p;
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);_mm_setcsr(control);
        x.functions.generated(0x22e650)(x.memory.flat(),&p,&x);const auto fp=_mm_getcsr();_mm_setcsr(control);
        try{y.functions.generated(0x22e650)(y.memory.flat(),&q,&y);}catch(const std::exception &e){std::fprintf(stderr,"particle seed=%u mode=%x: %s\n",seed,mode,e.what());throw;}
        check(_mm_getcsr()==fp);_mm_setcsr(saved);p.cycleLimit=q.cycleLimit;
        if(std::memcmp(&p,&q,sizeof(p)) || std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes)){
            std::fprintf(stderr,"particle pair seed=%u mode=%x\n",seed,mode);
            for(unsigned i=0;i<sizeof(p);++i)if(reinterpret_cast<uint8_t *>(&p)[i]!=reinterpret_cast<uint8_t *>(&q)[i]){std::fprintf(stderr,"CPU byte %x old=%02x new=%02x\n",i,reinterpret_cast<uint8_t *>(&p)[i],reinterpret_cast<uint8_t *>(&q)[i]);break;}
            for(unsigned i=0;i<bozo::kFlatBytes;++i)if(x.memory.flat()[i]!=y.memory.flat()[i]){std::fprintf(stderr,"RAM byte %x old=%02x new=%02x\n",i,x.memory.flat()[i],y.memory.flat()[i]);break;}
        }
        check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        check(now.particles.outstanding()==0 && old.publications==now.publications && now.points==now.total && old.points==now.points && now.finished==1);
        check(read<uint8_t>(y,0x180af9)==((seed&1)^1) && read<uint32_t>(y,0x180ac0+(seed&1)*4)==now.publications);
        if((seed&4) && now.total){check(read<float>(y,0x180a4c)==1.f && read<float>(y,0x180a5c)==.5f);
            for(unsigned j=0;j<3;++j){check(read<uint32_t>(y,0x180a40+j*4)==0x3f000000u+j*17);check(read<uint32_t>(y,0x180a50+j*4)==0x3f000000u+(now.total-1)*192+j*17);}
            check(read<float>(y,0x180a6c)>=read<float>(y,0x180ab8));}
        chunks+=now.publications;points+=now.points;++callers;
    }
    check(old.sites==now.sites && now.sites.size()==24);
    for(unsigned seed=0;seed<64;++seed){
        auto &x=a.runtime(),&y=b.runtime();initialize(x,seed&1,seed&8?0x175920u:0x154180u);old.ready=now.ready=seed%4!=0;old.dirty=now.dirty=seed%4!=1;
        if(seed%4==2)write(x,0x180a90,uint32_t(0));if(seed%4==3)write(x,0x35ded8,0.f);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);R5900Context p{};p.r[4].uw[0]=0x180000;p.r[29].uw[0]=0x1fff000;p.r[31].uw[0]=0x170000;auto q=p;
        old.publications=now.publications=0;x.functions.generated(0x22e650)(x.memory.flat(),&p,&x);y.functions.generated(0x22e650)(y.memory.flat(),&q,&y);
        p.cycleLimit=q.cycleLimit;check(!std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        check(!now.publications && !old.publications && now.particles.outstanding()==0);
    }
    // mod003 3649f0's box sites (the game's words): LQC2 vf1 from a2, VMINI, VMAX join
    // the point to the driver frame's bounds (the method's frame is the driver's - 0x40).
    {auto &rt=b.runtime();initialize(rt,0);now.particles=vp2s::NativeParticles{};now.assets=vp2s::NativeAssets{};
        R5900Context c{};c.pc=0x22eadc;c.r[21].uw[0]=0x180000;c.r[29].uw[0]=0x1fff000;write(rt,0x1fff0e0,uint32_t(0x200020));
        check(*vp2s::nativeParticleInstruction(&c,&rt,0x22eae0));const auto frame=c.r[29].uw[0];
        write(rt,0x35dbc0,std::array<float,4>{1,1,1,0});write(rt,0x35dbe0,std::array<float,4>{2,2,2,.5f});c.r[3].uw[0]=0x35dbc0;c.r[2].uw[0]=0x35dbe0;
        check(*vp2s::nativeParticleInstruction(&c,&rt,0x22ec9c));check(*vp2s::nativeParticleInstruction(&c,&rt,0x22eca0));
        for(const auto &[pc,w]:{std::pair{0x364a84u,0xd8c10000u},{0x364ac4u,0x4bc1d6afu},{0x364ac8u,0x4bc1deebu}})write(rt,pc,w);
        R5900Context m{};m.r[29].uw[0]=frame-0x40;m.r[4].uw[0]=0x190000;m.r[6].uw[0]=0x1d0000;write(rt,0x1d0000,std::array<float,4>{-4,7,1.5f,9});
        auto math=[&](uint32_t pc){return vp2s::nativeMathInstruction(rt.memory.flat(),&m,&rt,pc);};
        bool order=false;try{math(0x364ac4);}catch(const std::runtime_error &){order=true;}check(order);
        check(math(0x364a84));order=false;try{math(0x364ac8);}catch(const std::runtime_error &){order=true;}check(order);
        check(math(0x364ac4) && math(0x364ac8));
        const auto grown=now.particles.bounds(frame,0x190000);
        check(grown.minimum[0]==-4 && grown.maximum[0]==2 && grown.minimum[1]==1 && grown.maximum[1]==7 && grown.minimum[2]==1 && grown.maximum[2]==2 && grown.maximum[3]==.5f);
        m.r[4].uw[0]=0x190010;order=false;try{math(0x364a84);}catch(const std::runtime_error &){order=true;}check(order);
        write(rt,0x364ac4,uint32_t(0));check(!math(0x364ac4));
        // 38cea0, a leaf method: the same words at 38cfec/38d010/38d018, frame = its sp.
        for(const auto &[pc,w]:{std::pair{0x38cfecu,0xd8c10000u},{0x38d010u,0x4bc1d6afu},{0x38d018u,0x4bc1deebu}})write(rt,pc,w);
        m.r[29].uw[0]=frame;m.r[4].uw[0]=0x190000;write(rt,0x1d0000,std::array<float,4>{5,-3,2.5f,9});
        order=false;try{math(0x38d018);}catch(const std::runtime_error &){order=true;}check(order);
        check(math(0x38cfec) && math(0x38d010) && math(0x38d018));
        const auto leaf=now.particles.bounds(frame,0x190000);
        check(leaf.minimum[0]==-4 && leaf.maximum[0]==5 && leaf.minimum[1]==-3 && leaf.maximum[1]==7 && leaf.maximum[2]==2.5f);
        m.r[29].uw[0]=frame-0x40;order=false;try{math(0x38cfec);}catch(const std::runtime_error &){order=true;}check(order);   // wrong frame
        // 44aef0 (forest emitter class): the same words at 44af7c/44afa4/44afa8, frame 0x40 as 3649f0.
        for(const auto &[pc,w]:{std::pair{0x44af7cu,0xd8c10000u},{0x44afa4u,0x4bc1d6afu},{0x44afa8u,0x4bc1deebu}})write(rt,pc,w);
        m.r[29].uw[0]=frame-0x40;m.r[4].uw[0]=0x190000;write(rt,0x1d0000,std::array<float,4>{0,11,-1,9});
        check(math(0x44af7c) && math(0x44afa4) && math(0x44afa8));
        const auto third=now.particles.bounds(frame,0x190000);
        check(third.minimum[0]==-4 && third.maximum[1]==11 && third.minimum[2]==-1 && third.maximum[0]==5);
        // mod028 48a820 (a battle emitter's +18, frame 0xf0): the point at v0 (word d8410000), the
        // emitter in s6 (a0 reused), MINI/MAX at 48aa10/48aa14.
        for(const auto &[pc,w]:{std::pair{0x48a9ccu,0xd8410000u},{0x48aa10u,0x4bc1d6afu},{0x48aa14u,0x4bc1deebu}})write(rt,pc,w);
        R5900Context e{};e.r[29].uw[0]=frame-0xf0;e.r[22].uw[0]=0x190000;e.r[4].uw[0]=0x1a0000;e.r[2].uw[0]=0x1d0010;write(rt,0x1d0010,std::array<float,4>{-9,3,12,9});
        auto bmath=[&](uint32_t pc){return vp2s::nativeMathInstruction(rt.memory.flat(),&e,&rt,pc);};
        check(bmath(0x48a9cc) && bmath(0x48aa10) && bmath(0x48aa14));
        const auto fourth=now.particles.bounds(frame,0x190000);
        check(fourth.minimum[0]==-9 && fourth.maximum[2]==12 && fourth.maximum[1]==11);
        e.r[22].uw[0]=0x1a0000;order=false;try{bmath(0x48a9cc);}catch(const std::runtime_error &){order=true;}check(order);   // not the frame's emitter
        // mod001 2847e0 (frame 0x240, emitter in s5): ten triplets, the point at r3 (the output)
        // or r4 (the record); two of them here, one per source register.
        for(const auto &[pc,w]:{std::pair{0x284f14u,0xd8610000u},{0x284f58u,0x4bc1d6afu},{0x284f5cu,0x4bc1deebu},
                {0x285050u,0xd8810000u},{0x2850a8u,0x4bc1d6afu},{0x2850acu,0x4bc1deebu}})write(rt,pc,w);
        R5900Context f{};f.r[29].uw[0]=frame-0x240;f.r[21].uw[0]=0x190000;f.r[4].uw[0]=0x1d0030;f.r[3].uw[0]=0x1d0020;
        write(rt,0x1d0020,std::array<float,4>{-13,0,0,1});write(rt,0x1d0030,std::array<float,4>{0,14,-6,1});
        auto fmath=[&](uint32_t pc){return vp2s::nativeMathInstruction(rt.memory.flat(),&f,&rt,pc);};
        check(fmath(0x284f14) && fmath(0x284f58) && fmath(0x284f5c) && fmath(0x285050) && fmath(0x2850a8) && fmath(0x2850ac));
        const auto fifth=now.particles.bounds(frame,0x190000);
        check(fifth.minimum[0]==-13 && fifth.maximum[1]==14 && fifth.minimum[2]==-6 && fifth.maximum[2]==12);
        order=false;try{fmath(0x284f58);}catch(const std::runtime_error &){order=true;}check(order);   // MINI without its load
        // mod001 23fa40 (the general emitter's +14, same frame and emitter register): three of its 48
        // triplets, one per source register r3, r6, r7.
        for(const auto &[pc,w]:{std::pair{0x23fb94u,0xd8610000u},{0x23fbecu,0x4bc1d6afu},{0x23fbf0u,0x4bc1deebu},{0x241fbcu,0xd8c10000u},{0x242008u,0x4bc1d6afu},{0x24200cu,0x4bc1deebu},
                {0x2423b4u,0xd8e10000u},{0x2423d8u,0x4bc1d6afu},{0x2423dcu,0x4bc1deebu}})write(rt,pc,w);
        f.r[3].uw[0]=0x1d0040;f.r[6].uw[0]=0x1d0050;f.r[7].uw[0]=0x1d0060;
        write(rt,0x1d0040,std::array<float,4>{-20,0,0,1});write(rt,0x1d0050,std::array<float,4>{0,21,0,1});write(rt,0x1d0060,std::array<float,4>{0,0,22,1});
        check(fmath(0x23fb94) && fmath(0x23fbec) && fmath(0x23fbf0) && fmath(0x241fbc) && fmath(0x242008) && fmath(0x24200c) && fmath(0x2423b4) && fmath(0x2423d8) && fmath(0x2423dc));
        const auto sixth=now.particles.bounds(frame,0x190000);
        check(sixth.minimum[0]==-20 && sixth.maximum[1]==21 && sixth.maximum[2]==22 && sixth.minimum[2]==-6);
        order=false;try{fmath(0x2423dc);}catch(const std::runtime_error &){order=true;}check(order);   // MAX without its load
        // mod001 28e290, 23fa40 compiled again for class 1585f0 (battle 0x1c): its triplets map one to one (here the first and the r7 one).
        for(const auto &[pc,w]:{std::pair{0x28e3e4u,0xd8610000u},{0x28e43cu,0x4bc1d6afu},{0x28e440u,0x4bc1deebu},{0x290b7cu,0xd8e10000u},{0x290ba0u,0x4bc1d6afu},{0x290ba4u,0x4bc1deebu}})write(rt,pc,w);
        write(rt,0x1d0040,std::array<float,4>{-30,0,0,1});write(rt,0x1d0060,std::array<float,4>{0,0,33,1});
        check(fmath(0x28e3e4) && fmath(0x28e43c) && fmath(0x28e440) && fmath(0x290b7c) && fmath(0x290ba0) && fmath(0x290ba4));
        const auto seventh=now.particles.bounds(frame,0x190000);check(seventh.minimum[0]==-30 && seventh.maximum[2]==33);
        write(rt,0x28e3e4,uint32_t(0));check(!fmath(0x28e3e4));   // another overlay's word there
        // mod001 287c30, a fifth compilation (battles 0x1c, 0x32): the first and the r7 triplet.
        for(const auto &[pc,w]:{std::pair{0x287d84u,0xd8610000u},{0x287ddcu,0x4bc1d6afu},{0x287de0u,0x4bc1deebu},{0x28a51cu,0xd8e10000u},{0x28a540u,0x4bc1d6afu},{0x28a544u,0x4bc1deebu}})write(rt,pc,w);
        write(rt,0x1d0040,std::array<float,4>{-40,0,0,1});write(rt,0x1d0060,std::array<float,4>{0,0,44,1});
        check(fmath(0x287d84) && fmath(0x287ddc) && fmath(0x287de0) && fmath(0x28a51c) && fmath(0x28a540) && fmath(0x28a544));
        {const auto eighth=now.particles.bounds(frame,0x190000);check(eighth.minimum[0]==-40 && eighth.maximum[2]==44);}
        now.particles=vp2s::NativeParticles{};
    }
    // mod003 40b080, the trail emitter: every VU0 site with the game's word at its address. A
    // test-only oracle executes each word through the engine's VU0 reference ops (bozo::vu0) on
    // its own copy of RAM; the native semantic step must leave identical records, outputs,
    // stack publications and box, over all six mode branches, the three trail kinds and 0..2
    // curve rows, two particles per call.
    {auto &y=b.runtime(),&x=a.runtime();initialize(y,0);now.particles=vp2s::NativeParticles{};now.assets=vp2s::NativeAssets{};
        R5900Context c{};c.pc=0x22eadc;c.r[21].uw[0]=0x180000;c.r[29].uw[0]=0x1fff000;write(y,0x1fff0e0,uint32_t(0x200020));
        check(*vp2s::nativeParticleInstruction(&c,&y,0x22eae0));const auto frame=c.r[29].uw[0];
        write(y,0x35dbc0,std::array<float,4>{1e9f,1e9f,1e9f,0});write(y,0x35dbe0,std::array<float,4>{-1e9f,-1e9f,-1e9f,.5f});c.r[3].uw[0]=0x35dbc0;c.r[2].uw[0]=0x35dbe0;
        check(*vp2s::nativeParticleInstruction(&c,&y,0x22ec9c));check(*vp2s::nativeParticleInstruction(&c,&y,0x22eca0));
        // The game's words (from the mod003 listing) at their addresses.
        const std::pair<uint32_t,uint32_t> words[]={
            {0x40b124,0x48a22800},{0x40b12c,0xd8540000},{0x40b168,0x4be00798},{0x40b170,0xd85d0000},
            {0x40b258,0xda410000},{0x40b260,0xda220000},{0x40b268,0xd9040000},{0x40b270,0xdac30000},{0x40b278,0xda060000},{0x40b280,0xd8670000},{0x40b288,0xd8480000},
            {0x40b318,0xd88b0000},{0x40b324,0x48a46000},{0x40b328,0x4bc00abc},{0x40b32c,0x4bcc5848},
            {0x40b364,0x4bc00abc},{0x40b368,0x4bc0a0bf},{0x40b36c,0x4bc51048},{0x40b370,0x4bfe32bc},{0x40b374,0x4be53988},{0x40b378,0x4bc022bc},{0x40b37c,0x4bc51108},
            {0x40b380,0x4bc012bc},{0x40b384,0x4bc51888},{0x40b388,0x4a251084},{0x40b38c,0x4be03190},{0x40b390,0x4a252100},{0x40b394,0x4be039bf},{0x40b398,0x4be541c8},
            {0x40b39c,0x4bc1d6af},{0x40b3a0,0x4bfd3194},{0x40b3a4,0x4bc1deeb},{0x40b3a8,0xfae10000},{0x40b3ac,0xfae60010},
            {0x40b410,0xf9040000},{0x40b414,0xfa410000},{0x40b418,0xfa220000},{0x40b41c,0xfa060000},{0x40b420,0xf8670000},
            {0x40b488,0xf9040000},{0x40b48c,0xfa410000},{0x40b490,0xfa220000},{0x40b494,0xfa060000},{0x40b498,0xf8670000},
            {0x40b510,0x4bc007c3},{0x40b514,0xf9040000},{0x40b518,0x4bc4212a},{0x40b51c,0xfa410000},{0x40b520,0xfa220000},{0x40b524,0xfa060000},{0x40b528,0xf8670000},
            {0x40b52c,0x4b04203d},{0x40b530,0x4b04f90a},{0x40b53c,0xf8440000},
            {0x40b5ac,0xd8480000},{0x40b5b0,0xd8490010},{0x40b5b4,0xd84a0020},{0x40b5b8,0xd84b0030},{0x40b5bc,0x4be141bc},{0x40b5c0,0x4be148bd},{0x40b5c4,0x4be150be},{0x40b5c8,0x4be05c8b},
            {0x40b5cc,0xf9040000},{0x40b5d0,0xfa410000},{0x40b5d4,0xfa220000},{0x40b5d8,0xfa060000},{0x40b5dc,0xf8670000},{0x40b5e4,0xf8520000},
            {0x40b650,0xf9040000},{0x40b654,0xfa410000},{0x40b658,0xfa220000},{0x40b65c,0xfa060000},{0x40b660,0xf8670000},
            {0x40b6f0,0xf9040000},{0x40b6f4,0xfa410000},{0x40b6f8,0xfa220000},{0x40b6fc,0xfa060000},{0x40b700,0xf8670000},
            {0x40b738,0xd8810000},{0x40b748,0x48a21800},{0x40b74c,0x4be40b3c},{0x40b768,0xd8420000},{0x40b76c,0x4be111ac},{0x40b770,0x4be1133c},{0x40b774,0x4bc007c3},
            {0x40b778,0x4bc631ea},{0x40b77c,0x4b07383d},{0x40b780,0x4b07f9ca},{0x40b784,0x4a6703be},{0x40b788,0x4a0003bf},{0x40b78c,0x4bc0321c},{0x40b790,0x4bc022bc},
            {0x40b794,0x4bc030bf},{0x40b798,0x4bc34108},{0x40b79c,0xf8440000},{0x40b7a0,0x4bc4d6af},{0x40b7a4,0x4bc4deeb},
            {0x40b7d8,0xd8810000},{0x40b7e8,0x48a21800},{0x40b7ec,0x4be40b3c},{0x40b808,0xd8420000},{0x40b80c,0x4be111ac},{0x40b810,0x4be1133c},{0x40b814,0x4bc007c3},
            {0x40b818,0x4bc631ea},{0x40b81c,0x4b07383d},{0x40b820,0x4b07f9ca},{0x40b824,0x4a6703be},{0x40b828,0x4a0003bf},{0x40b82c,0x4bc0319c},{0x40b830,0x4bc022bc},
            {0x40b834,0x4bc33108},{0x40b838,0xf8440000},{0x40b83c,0x4bc4d6af},{0x40b840,0x4bc4deeb},
            {0x40b878,0xd8810000},{0x40b888,0x48a21800},{0x40b88c,0x4be40b3c},{0x40b8a8,0xd8420000},{0x40b8ac,0x4be111ac},{0x40b8b0,0x4be1133c},{0x40b8b4,0x4bc022bc},
            {0x40b8b8,0x4bc33108},{0x40b8bc,0xf8440000},{0x40b8c0,0x4bc4d6af},{0x40b8c4,0x4bc4deeb}};
        for(const auto &[pc,w]:words)write(y,pc,w);
        // Test-only oracle: one COP2 macro word through the engine's VU0 reference.
        R5900Context o{};uint32_t q=0;
        const auto oracle=[&](uint32_t w){
            namespace v=bozo::vu0;auto &vf=o.vu0_vf;auto &acc=o.vu0_acc;const unsigned op=w>>26;
            const auto ea=[&]{return o.r[(w>>21)&31].uw[0]+uint32_t(int32_t(int16_t(w&0xffff)));};
            if(op==0x36){const unsigned t=(w>>16)&31;if(t)std::memcpy(vf[t].u,x.memory.hostPointer(ea(),16),16);return;}
            if(op==0x3e){std::memcpy(x.memory.hostPointer(ea(),16),vf[(w>>16)&31].u,16);return;}
            check(op==0x12);
            if(!(w&(1u<<25))){check(((w>>21)&31)==5);const unsigned d=(w>>11)&31;if(d)std::memcpy(vf[d].u,o.r[(w>>16)&31].uw,16);return;}
            const unsigned dest=(w>>21)&15,ft=(w>>16)&31,fs=(w>>11)&31,fd=(w>>6)&31,f=w&63;
            if(f>=0x3c){const unsigned sub=((w>>6)&31)<<2|(f&3);
                if(sub<=3)v::add(&o,acc,vf[fs],v::bc(vf[ft],sub&3),dest);
                else if(sub>=8 && sub<=0xb)v::madd(&o,acc,acc,vf[fs],v::bc(vf[ft],sub&3),dest);
                else if(sub>=0x18 && sub<=0x1b)v::mul(&o,acc,vf[fs],v::bc(vf[ft],sub&3),dest);
                else if(sub==0x28)v::add(&o,acc,vf[fs],vf[ft],dest);
                else if(sub==0x30){auto d=vf[ft];v::move(d,vf[fs],dest);if(ft)vf[ft]=d;}
                else if(sub==0x3a)q=v::rsqrt(&o,vf[fs].u[(w>>21)&3],vf[ft].u[(w>>23)&3]);
                else check(sub==0x3b);
                return;}
            auto d=vf[fd];
            if(f<0x1c){const auto t=v::bc(vf[ft],f&3);
                switch(f>>2){case 0:v::add(&o,d,vf[fs],t,dest);break;case 1:v::sub(&o,d,vf[fs],t,dest);break;case 2:v::madd(&o,d,acc,vf[fs],t,dest);break;
                    case 4:v::max(d,vf[fs],t,dest);break;case 5:v::mini(d,vf[fs],t,dest);break;case 6:v::mul(&o,d,vf[fs],t,dest);break;default:check(false);}}
            else switch(f){case 0x1c:v::mul(&o,d,vf[fs],v::broadcast(q),dest);break;case 0x28:v::add(&o,d,vf[fs],vf[ft],dest);break;case 0x2a:v::mul(&o,d,vf[fs],vf[ft],dest);break;
                case 0x2b:v::max(d,vf[fs],vf[ft],dest);break;case 0x2c:v::sub(&o,d,vf[fs],vf[ft],dest);break;case 0x2f:v::mini(d,vf[fs],vf[ft],dest);break;default:check(false);}
            if(fd)vf[fd]=d;
        };
        R5900Context m{};
        const auto run=[&](uint32_t pc){
            std::memcpy(o.r,m.r,sizeof(m.r));std::memcpy(o.f,m.f,sizeof(m.f));oracle(read<uint32_t>(y,pc));
            check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,pc));};
        o.vu0_vf[0].u[3]=0x3f800000u;
        std::memcpy(o.vu0_vf[26].u,y.memory.hostPointer(0x35dbc0,16),16);std::memcpy(o.vu0_vf[27].u,y.memory.hostPointer(0x35dbe0,16),16);
        constexpr uint32_t E=0x190000,B=0x1c0000,T=0x1c8000,O=0x1d0000;const uint32_t S=frame-0x170;
        unsigned trailCases=0,points=0;
        for(unsigned seed=0;seed<72;++seed){
            uint32_t state=seed*2246822519u+5;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
            const auto value=[&](unsigned lo=105,unsigned span=40){return (rnd()&0x807fffffu)|((lo+rnd()%span)<<23);};
            const auto vec=[&](uint32_t at){for(unsigned i=0;i<4;++i)write(y,at+4*i,value());};
            const uint32_t dt=value(118,6)&0x7fffffffu;write(y,0x35dee8,dt);vec(E+0xc0);write(y,E+0x54,value(122,6));
            write(y,E+0x14,B);write(y,E+0x68,T);for(unsigned i=0;i<4;++i)vec(E+0x80+16*i);for(unsigned i=0;i<16;++i)vec(T+16*i);
            for(unsigned i=0;i<4;++i)write(y,S+0x160+4*i,0x437f0000u);
            for(uint32_t at=B;at<B+0xb0*10;at+=16)vec(at);for(uint32_t at=O;at<O+0x30*8;at+=16)vec(at);
            std::memcpy(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes);
            const auto random=[&](unsigned r,uint32_t low){m.r[r].uw[0]=low;for(unsigned i=1;i<4;++i)m.r[r].uw[i]=rnd();};
            m=R5900Context{};m.r[29].uw[0]=S;m.r[21].uw[0]=E;
            random(2,dt);run(0x40b124);m.r[2].uw[0]=E+0xc0;run(0x40b12c);run(0x40b168);m.r[2].uw[0]=S+0x160;run(0x40b170);
            const unsigned mode=std::array{1u,3u,2u,5u,6u,0u}[seed%6],trail=seed%4,rows=(seed/4)%3,n=2+seed%3;
            for(unsigned k=0;k<2;++k){
                const unsigned index=seed%5+3*k;const uint32_t rec=B+index*0xb0,out=O+k*0x30*4;
                m.r[20].uw[0]=index;m.r[18].uw[0]=rec+0x10;m.r[17].uw[0]=rec+0x30;m.r[8].uw[0]=rec+0x50;m.r[22].uw[0]=rec+0x40;m.r[16].uw[0]=rec+0x20;m.r[3].uw[0]=rec+0x60;m.r[2].uw[0]=rec+0x70;
                for(uint32_t pc=0x40b258;pc<=0x40b288;pc+=8)run(pc);
                for(unsigned j=0;j<rows;++j){const unsigned t1=j+seed%5;m.r[9].uw[0]=t1;m.r[4].uw[0]=T+16*t1;run(0x40b318);
                    const auto fraction=value(115,8)&0x7fffffffu;m.f[1]=std::bit_cast<float>(fraction);random(4,fraction);run(0x40b324);run(0x40b328);run(0x40b32c);}
                for(uint32_t pc=0x40b364;pc<=0x40b3a4;pc+=4)run(pc);
                m.r[23].uw[0]=out;run(0x40b3a8);run(0x40b3ac);
                const auto stores=[&](uint32_t first){for(unsigned i=0;i<5;++i)run(first+4*i);};
                if(mode==1)stores(0x40b410);else if(mode==3)stores(0x40b488);else if(mode==6)stores(0x40b650);else if(mode==0)stores(0x40b6f0);
                else if(mode==2){for(uint32_t pc:{0x40b510u,0x40b514u,0x40b518u,0x40b51cu,0x40b520u,0x40b524u,0x40b528u,0x40b52cu,0x40b530u})run(pc);m.r[2].uw[0]=S+0x150;run(0x40b53c);}
                else{m.r[2].uw[0]=E+0x80;for(uint32_t pc=0x40b5ac;pc<=0x40b5c8;pc+=4)run(pc);stores(0x40b5cc);m.r[2].uw[0]=S+0x140;run(0x40b5e4);}
                if(trail){
                    const uint32_t head=trail==2?0x40b738:trail==1?0x40b7d8:0x40b878,first=trail==2?0x40b768:trail==1?0x40b808:0x40b8a8,last=trail==2?0x40b7a4:trail==1?0x40b840:0x40b8c4;
                    m.r[19].uw[0]=n;m.r[23].uw[0]=out+n*0x30;m.r[4].uw[0]=out;run(head);random(2,read<uint32_t>(y,E+0x54));run(head+0x10);run(head+0x14);
                    for(unsigned i=1;i<n;++i){m.r[3].uw[0]=i;m.r[2].uw[0]=out+i*0x30;for(uint32_t pc=first;pc<=last;pc+=4)run(pc);++points;}
                }
            }
            // Identical records, outputs, publications and box.
            check(!std::memcmp(x.memory.hostPointer(B,0xb0*10),y.memory.hostPointer(B,0xb0*10),0xb0*10));
            check(!std::memcmp(x.memory.hostPointer(O,0x30*8),y.memory.hostPointer(O,0x30*8),0x30*8));
            check(!std::memcmp(x.memory.hostPointer(S+0x140,0x20),y.memory.hostPointer(S+0x140,0x20),0x20));
            const auto box=now.particles.bounds(frame,E);
            for(unsigned i=0;i<3;++i)check(std::bit_cast<uint32_t>(box.minimum[i])==o.vu0_vf[26].u[i] && std::bit_cast<uint32_t>(box.maximum[i])==o.vu0_vf[27].u[i]);
            trailCases+=2;
        }
        // Out of order (a main site before the loads), a wrong record address, and another word reject.
        m=R5900Context{};m.r[29].uw[0]=S;m.r[21].uw[0]=E;m.r[2].uw[0]=read<uint32_t>(y,0x35dee8);
        check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40b124));
        bool rejected=false;try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40b12c);}catch(const std::runtime_error &){rejected=true;}check(rejected);   // drift source not +c0
        m.r[2].uw[0]=read<uint32_t>(y,0x35dee8);check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40b124));
        m.r[2].uw[0]=E+0xc0;check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40b12c));
        rejected=false;try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40b364);}catch(const std::runtime_error &){rejected=true;}check(rejected);
        m.r[2].uw[0]=read<uint32_t>(y,0x35dee8);check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40b124));m.r[2].uw[0]=E+0xc0;
        check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40b12c) && vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40b168));
        m.r[2].uw[0]=S+0x160;check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40b170));
        m.r[20].uw[0]=1;m.r[18].uw[0]=B+0xb0+0x20;rejected=false;try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40b258);}catch(const std::runtime_error &){rejected=true;}check(rejected);
        write(y,0x40b364,uint32_t(0));check(!vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40b364));
        std::printf("%u trail emitter particles (%u trail points, six modes, three trail kinds, curves) match the VU0 reference\n",trailCases,points);
        // mod003 40b990, the orbit emitter: the trail step on 0x120-byte records with an
        // orbit (radius/angles advanced before the main step, their rates after; the
        // scalar offset at sp+1a0 joins P and A), output through *(sp+160), all modes,
        // trail kinds and curves, against the same VU0 reference.
        {const std::pair<uint32_t,uint32_t> orbit[]={
            {0x40ba48,0x48a22800},{0x40ba50,0xd8540000},{0x40ba8c,0x4be00798},{0x40ba94,0xd85d0000},{0x40bb68,0xda410000},{0x40bb80,0xd8490000},{0x40bb88,0xd84a0000},{0x40bb90,0xdaeb0000},
            {0x40bb98,0xdbcd0000},{0x40bba4,0x4bc04abc},{0x40bba8,0x4bc55a48},{0x40bbac,0x4bc052bc},{0x40bbb0,0x4bc56a88},{0x40bbb8,0xf8490000},{0x40bbc0,0xf84a0000},{0x40bc54,0xda220000},
            {0x40bc5c,0xd9040000},{0x40bc6c,0xd8430000},{0x40bc74,0xda060000},{0x40bc7c,0xd8670000},{0x40bc84,0xd8480000},{0x40bc8c,0xd84f0000},{0x40bc94,0xd84c0000},{0x40bc9c,0xd84e0000},
            {0x40bca0,0xdaeb0000},{0x40bca4,0xdbcd0000},{0x40bd30,0xd8900000},{0x40bd3c,0x48a48800},{0x40bd40,0x4bc00abc},{0x40bd44,0x4bd18048},{0x40bd7c,0x4bc00abc},{0x40bd80,0x4bc078bf},
            {0x40bd84,0x4bc0a0bf},{0x40bd88,0x4bc51048},{0x40bd8c,0x4bfe32bc},{0x40bd90,0x4be53988},{0x40bd94,0x4bc022bc},{0x40bd98,0x4bc078bf},{0x40bd9c,0x4bc51108},{0x40bda0,0x4bc012bc},
            {0x40bda4,0x4bc51888},{0x40bda8,0x4a251084},{0x40bdac,0x4be03190},{0x40bdb0,0x4a252100},{0x40bdb4,0x4be039bf},{0x40bdb8,0x4be541c8},{0x40bdbc,0x4bc1d6af},{0x40bdc0,0x4bfd3194},
            {0x40bdc4,0x4bc1deeb},{0x40bdc8,0x4bc05abc},{0x40bdcc,0x4bc562c8},{0x40bdd0,0x4bc06abc},{0x40bdd4,0x4bc57348},{0x40bddc,0xf8810000},{0x40bde4,0xf8860010},{0x40be50,0xf9040000},
            {0x40be54,0xfa410000},{0x40be58,0xfa220000},{0x40be5c,0xfa060000},{0x40be60,0xf8670000},{0x40be64,0xfaeb0000},{0x40be68,0xfbcd0000},{0x40bed8,0xf9040000},{0x40bedc,0xfa410000},
            {0x40bee0,0xfa220000},{0x40bee4,0xfa060000},{0x40bee8,0xf8670000},{0x40beec,0xfaeb0000},{0x40bef0,0xfbcd0000},{0x40bf68,0x4bc007c3},{0x40bf6c,0xf9040000},{0x40bf70,0x4bc4212a},
            {0x40bf74,0xfa410000},{0x40bf78,0xfa220000},{0x40bf7c,0xfa060000},{0x40bf80,0xf8670000},{0x40bf84,0xfaeb0000},{0x40bf88,0xfbcd0000},{0x40bf8c,0x4b04203d},{0x40bf90,0x4b04f90a},
            {0x40bf9c,0xf8440000},{0x40c010,0xfaeb0000},{0x40c014,0xfbcd0000},{0x40c01c,0xd8480000},{0x40c020,0xd8490010},{0x40c024,0xd84a0020},{0x40c028,0xd84b0030},{0x40c02c,0x4be141bc},
            {0x40c030,0x4be148bd},{0x40c034,0x4be150be},{0x40c038,0x4be05c8b},{0x40c03c,0xf9040000},{0x40c040,0xfa410000},{0x40c044,0xfa220000},{0x40c048,0xfa060000},{0x40c04c,0xf8670000},
            {0x40c054,0xf8520000},{0x40c0c8,0xf9040000},{0x40c0cc,0xfa410000},{0x40c0d0,0xfa220000},{0x40c0d4,0xfa060000},{0x40c0d8,0xf8670000},{0x40c0dc,0xfaeb0000},{0x40c0e0,0xfbcd0000},
            {0x40c178,0xf9040000},{0x40c17c,0xfa410000},{0x40c180,0xfa220000},{0x40c184,0xfa060000},{0x40c188,0xf8670000},{0x40c18c,0xfaeb0000},{0x40c190,0xfbcd0000},{0x40c1cc,0xd8810000},
            {0x40c1dc,0x48a21800},{0x40c1e0,0x4be40b3c},{0x40c200,0xd8420000},{0x40c204,0x4be111ac},{0x40c208,0x4be1133c},{0x40c20c,0x4bc007c3},{0x40c210,0x4bc631ea},{0x40c214,0x4b07383d},
            {0x40c218,0x4b07f9ca},{0x40c21c,0x4a6703be},{0x40c220,0x4a0003bf},{0x40c224,0x4bc0321c},{0x40c228,0x4bc022bc},{0x40c22c,0x4bc030bf},{0x40c230,0x4bc34108},{0x40c234,0xf8440000},
            {0x40c238,0x4bc4d6af},{0x40c23c,0x4bc4deeb},{0x40c274,0xd8810000},{0x40c284,0x48a21800},{0x40c288,0x4be40b3c},{0x40c2a8,0xd8420000},{0x40c2ac,0x4be111ac},{0x40c2b0,0x4be1133c},
            {0x40c2b4,0x4bc007c3},{0x40c2b8,0x4bc631ea},{0x40c2bc,0x4b07383d},{0x40c2c0,0x4b07f9ca},{0x40c2c4,0x4a6703be},{0x40c2c8,0x4a0003bf},{0x40c2cc,0x4bc0319c},{0x40c2d0,0x4bc022bc},
            {0x40c2d4,0x4bc33108},{0x40c2d8,0xf8440000},{0x40c2dc,0x4bc4d6af},{0x40c2e0,0x4bc4deeb},{0x40c31c,0xd8810000},{0x40c32c,0x48a21800},{0x40c330,0x4be40b3c},{0x40c350,0xd8420000},
            {0x40c354,0x4be111ac},{0x40c358,0x4be1133c},{0x40c35c,0x4bc022bc},{0x40c360,0x4bc33108},{0x40c364,0xf8440000},{0x40c368,0x4bc4d6af},{0x40c36c,0x4bc4deeb}};
        for(const auto &[pc,w]:orbit)write(y,pc,w);
        const uint32_t S2=frame-0x1e0;unsigned orbitCases=0,orbitPoints=0;
        const auto both=[&](uint32_t at,uint32_t v){write(y,at,v);write(x,at,v);};
        for(unsigned seed=0;seed<72;++seed){
            uint32_t state=seed*2654435761u+11;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
            const auto value=[&](unsigned lo=105,unsigned span=40){return (rnd()&0x807fffffu)|((lo+rnd()%span)<<23);};
            const auto vec=[&](uint32_t at){for(unsigned i=0;i<4;++i)write(y,at+4*i,value());};
            const uint32_t dt=value(118,6)&0x7fffffffu;write(y,0x35dee8,dt);vec(E+0xc0);write(y,E+0x54,value(122,6));
            write(y,E+0x14,B);write(y,E+0x68,T);for(unsigned i=0;i<4;++i)vec(E+0x80+16*i);for(unsigned i=0;i<16;++i)vec(T+16*i);
            for(unsigned i=0;i<4;++i)write(y,S2+0x1d0+4*i,0x437f0000u);
            for(uint32_t at=B;at<B+0x120*10;at+=16)vec(at);for(uint32_t at=O;at<O+0x30*8;at+=16)vec(at);
            std::memcpy(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes);
            const auto random=[&](unsigned r,uint32_t low){m.r[r].uw[0]=low;for(unsigned i=1;i<4;++i)m.r[r].uw[i]=rnd();};
            m=R5900Context{};m.r[29].uw[0]=S2;m.r[21].uw[0]=E;
            random(2,dt);run(0x40ba48);m.r[2].uw[0]=E+0xc0;run(0x40ba50);run(0x40ba8c);m.r[2].uw[0]=S2+0x1d0;run(0x40ba94);
            const unsigned mode=std::array{1u,3u,2u,5u,6u,0u}[seed%6],trail=seed%4,rows=(seed/4)%3,n=2+seed%3;
            for(unsigned k=0;k<2;++k){
                const unsigned index=seed%5+3*k;const uint32_t rec=B+index*0x120,out=O+k*0x30*4;
                m.r[20].uw[0]=index;m.r[18].uw[0]=rec+0x10;run(0x40bb68);
                m.r[2].uw[0]=rec+0xb0;run(0x40bb80);m.r[2].uw[0]=rec+0x80;run(0x40bb88);m.r[23].uw[0]=rec+0xc0;run(0x40bb90);m.r[30].uw[0]=rec+0x90;run(0x40bb98);
                for(uint32_t pc=0x40bba4;pc<=0x40bbb0;pc+=4)run(pc);m.r[2].uw[0]=rec+0xb0;run(0x40bbb8);m.r[2].uw[0]=rec+0x80;run(0x40bbc0);
                for(unsigned i=0;i<4;++i)both(S2+0x1a0+4*i,value());   // the scalar orbit offset
                m.r[17].uw[0]=rec+0x30;run(0x40bc54);m.r[8].uw[0]=rec+0x50;run(0x40bc5c);m.r[2].uw[0]=rec+0x40;run(0x40bc6c);m.r[16].uw[0]=rec+0x20;run(0x40bc74);
                m.r[3].uw[0]=rec+0x60;run(0x40bc7c);m.r[2].uw[0]=rec+0x70;run(0x40bc84);m.r[2].uw[0]=S2+0x1a0;run(0x40bc8c);m.r[2].uw[0]=rec+0xd0;run(0x40bc94);
                m.r[2].uw[0]=rec+0xa0;run(0x40bc9c);run(0x40bca0);run(0x40bca4);
                for(unsigned j=0;j<rows;++j){const unsigned a3=j+seed%5;m.r[7].uw[0]=a3;m.r[4].uw[0]=T+16*a3;run(0x40bd30);
                    const auto fraction=value(115,8)&0x7fffffffu;m.f[1]=std::bit_cast<float>(fraction);random(4,fraction);run(0x40bd3c);run(0x40bd40);run(0x40bd44);}
                for(uint32_t pc=0x40bd7c;pc<=0x40bdd4;pc+=4)run(pc);
                both(S2+0x160,out);m.r[4].uw[0]=out;run(0x40bddc);run(0x40bde4);both(S2+0x160,out+n*0x30);
                const auto stores=[&](uint32_t first){for(unsigned i=0;i<7;++i)run(first+4*i);};
                if(mode==1)stores(0x40be50);else if(mode==3)stores(0x40bed8);else if(mode==6)stores(0x40c0c8);else if(mode==0)stores(0x40c178);
                else if(mode==2){for(uint32_t pc=0x40bf68;pc<=0x40bf90;pc+=4)run(pc);m.r[2].uw[0]=S2+0x190;run(0x40bf9c);}
                else{run(0x40c010);run(0x40c014);m.r[2].uw[0]=E+0x80;for(uint32_t pc=0x40c01c;pc<=0x40c04c;pc+=4)run(pc);m.r[2].uw[0]=S2+0x180;run(0x40c054);}
                if(trail){
                    const uint32_t head=trail==2?0x40c1cc:trail==1?0x40c274:0x40c31c,first=trail==2?0x40c200:trail==1?0x40c2a8:0x40c350,last=trail==2?0x40c23c:trail==1?0x40c2e0:0x40c36c;
                    m.r[22].uw[0]=n;m.r[4].uw[0]=out;run(head);random(2,read<uint32_t>(y,E+0x54));run(head+0x10);run(head+0x14);
                    for(unsigned i=1;i<n;++i){m.r[3].uw[0]=i;m.r[2].uw[0]=out+i*0x30;for(uint32_t pc=first;pc<=last;pc+=4)run(pc);++orbitPoints;}
                }
            }
            check(!std::memcmp(x.memory.hostPointer(B,0x120*10),y.memory.hostPointer(B,0x120*10),0x120*10));
            check(!std::memcmp(x.memory.hostPointer(O,0x30*8),y.memory.hostPointer(O,0x30*8),0x30*8));
            check(!std::memcmp(x.memory.hostPointer(S2+0x180,0x20),y.memory.hostPointer(S2+0x180,0x20),0x20));
            const auto box=now.particles.bounds(frame,E);
            for(unsigned i=0;i<3;++i)check(std::bit_cast<uint32_t>(box.minimum[i])==o.vu0_vf[26].u[i] && std::bit_cast<uint32_t>(box.maximum[i])==o.vu0_vf[27].u[i]);
            orbitCases+=2;
        }
        // The orbit step must follow the position load; a store to the wrong record field rejects.
        m=R5900Context{};m.r[29].uw[0]=S2;m.r[21].uw[0]=E;m.r[2].uw[0]=read<uint32_t>(y,0x35dee8);check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40ba48));
        m.r[2].uw[0]=E+0xc0;check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40ba50) && vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40ba8c));
        m.r[2].uw[0]=S2+0x1d0;check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40ba94));
        m.r[2].uw[0]=B+0xb0;bool refused=false;try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40bb80);}catch(const std::runtime_error &){refused=true;}check(refused);
        m.r[2].uw[0]=read<uint32_t>(y,0x35dee8);check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40ba48));
        m.r[2].uw[0]=S2+0x160;refused=false;try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x40ba50);}catch(const std::runtime_error &){refused=true;}check(refused);   // drift not +c0
        std::printf("%u orbit emitter particles (%u trail points, six modes, three trail kinds, curves) match the VU0 reference\n",orbitCases,orbitPoints);}
        // mod001 2451d0 + 2439f0, the general emitter (class 154810): prepare's constant registers,
        // then per particle of 0x170 bytes the end-condition ops (|A|^2, row transforms with the
        // dt/drift reloads), offset, jitter, loads, parent push, curves, the main step, output,
        // collision, stores and the three trail kinds (kind 2's last direction becomes the next
        // particle's drift), against the same VU0 reference.
        {const std::pair<uint32_t,uint32_t> program[]={
            {0x2447e4,0xd8440000},{0x2447e8,0x4bc007c3},{0x2447ec,0x4bc4206a},{0x2447f0,0x4b01083d},{0x2447f4,0x4b01f84a},{0x2447fc,0xf8410000},{0x2448d8,0xda050000},{0x2448dc,0xd8610000},
            {0x2448e0,0xd8620010},{0x2448e4,0xd8630020},{0x2448e8,0xd8640030},{0x2448ec,0x4be509bc},{0x2448f0,0x4be510bd},{0x2448f4,0x4be518be},{0x2448f8,0x4be0214b},{0x2448fc,0xf8450000},
            {0x244940,0x48a22800},{0x24494c,0xd8480000},{0x244958,0xda050000},{0x24495c,0xd8610000},{0x244960,0xd8620010},{0x244964,0xd8630020},{0x244968,0xd8640030},{0x24496c,0x4be509bc},
            {0x244970,0x4be510bd},{0x244974,0x4be518be},{0x244978,0x4be0214b},{0x24497c,0xf8450000},{0x2449a8,0x48a22800},{0x2449b4,0xd8480000},{0x244b50,0x4bcf033c},{0x244c58,0xd8500000},
            {0x244ce0,0x4be00418},{0x244ce8,0xda010000},{0x244cec,0xdbc20000},{0x244cf4,0xd8430000},{0x244cfc,0xd8440000},{0x244d04,0xd8460000},{0x244d0c,0xdac70000},{0x244d14,0xdab40000},
            {0x244d44,0xd8510000},{0x244dd0,0xd8550000},{0x244ddc,0x48a2b000},{0x244de0,0x4bc00abc},{0x244de4,0x4bd6a848},{0x244e1c,0x4bc00abc},{0x244e20,0x4bc078bf},{0x244e24,0x4bc040bf},
            {0x244e28,0x4bc51048},{0x244e2c,0x4bfe32bc},{0x244e30,0x4be080bf},{0x244e34,0x4be53988},{0x244e38,0x4bc01abc},{0x244e3c,0x4bc590c8},{0x244e40,0x4bc022bc},{0x244e44,0x4bc078bf},
            {0x244e48,0x4bc51108},{0x244e4c,0x4bc012bc},{0x244e50,0x4bc588bc},{0x244e54,0x4bc51888},{0x244e58,0x4a251084},{0x244e5c,0x4be03190},{0x244e60,0x4a252100},{0x244e64,0x4be039bf},
            {0x244e68,0x4be5a1c8},{0x244e6c,0x4bc1d6af},{0x244e70,0x4bfd3194},{0x244e74,0x4bc1deeb},{0x244e7c,0xfae10000},{0x244e8c,0xfbc20000},{0x244ecc,0xd84b0000},{0x244ed4,0xd8410000},
            {0x244ee0,0x4bcb10a8},{0x244ee8,0xfae60010},{0x244ef0,0xf8440000},{0x244ef4,0xfa010000},{0x244ef8,0xfbc20000},{0x244f00,0xf8430000},{0x244f08,0xf8460000},{0x244f0c,0xfac70000},
            {0x244f68,0xd8810000},{0x244f78,0x48a21800},{0x244f7c,0x4be40b3c},{0x244f98,0xd8420000},{0x244f9c,0x4be111ac},{0x244fa0,0x4be1133c},{0x244fa4,0x4bc007c3},{0x244fa8,0x4bc631ea},
            {0x244fac,0x4b07383d},{0x244fb0,0x4b07f9ca},{0x244fb4,0x4a6703be},{0x244fb8,0x4a0003bf},{0x244fbc,0x4bc0321c},{0x244fc0,0x4bc022bc},{0x244fc4,0x4bc030bf},{0x244fc8,0x4bc34108},
            {0x244fcc,0xf8440000},{0x244fd0,0x4bc4d6af},{0x244fd4,0x4bc4deeb},{0x245008,0xd8810000},{0x245018,0x48a21800},{0x24501c,0x4be40b3c},{0x245038,0xd8420000},{0x24503c,0x4be111ac},
            {0x245040,0x4be1133c},{0x245044,0x4bc007c3},{0x245048,0x4bc631ea},{0x24504c,0x4b07383d},{0x245050,0x4b07f9ca},{0x245054,0x4a6703be},{0x245058,0x4a0003bf},{0x24505c,0x4bc0319c},
            {0x245060,0x4bc022bc},{0x245064,0x4bc33108},{0x245068,0xf8440000},{0x24506c,0x4bc4d6af},{0x245070,0x4bc4deeb},{0x2450ac,0xd8810000},{0x2450bc,0x48a21800},{0x2450c0,0x4be40b3c},
            {0x2450e0,0xd8420000},{0x2450e4,0x4be111ac},{0x2450e8,0x4be1133c},{0x2450ec,0x4bc022bc},{0x2450f0,0x4bc33108},{0x2450f4,0xf8440000},{0x2450f8,0x4bc4d6af},{0x2450fc,0x4bc4deeb},
            {0x245240,0xd8720000},{0x245248,0x4bd2033c},{0x245250,0x4bd1033c},{0x245264,0x48a32800},{0x24526c,0xd8680000},{0x2452a8,0x4be00798},{0x2452b0,0xd87d0000}
        };
        for(const auto &[pc,w]:program){write(y,pc,w);write(x,pc,w);}
        const uint32_t P0=frame-0x40,S3=frame-0x270,PARENT=0x1e0000;unsigned programCases=0,programPoints=0;
        const auto both=[&](uint32_t at,uint32_t v){write(y,at,v);write(x,at,v);};
        for(unsigned seed=0;seed<96;++seed){
            uint32_t state=seed*2246822519u+29;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
            const auto value=[&](unsigned lo=105,unsigned span=40){return (rnd()&0x807fffffu)|((lo+rnd()%span)<<23);};
            const auto vec=[&](uint32_t at){for(unsigned i=0;i<4;++i)both(at+4*i,value());};
            const uint32_t dt=value(118,6)&0x7fffffffu;both(0x35dee8,dt);vec(E+0xc0);both(E+0x54,value(122,6));
            both(E+0x14,B);both(E+0x68,T);both(E+0xf8,seed&1?PARENT:0);vec(PARENT+0x10);
            for(unsigned i=0;i<4;++i)vec(E+0x80+16*i);for(unsigned i=0;i<16;++i)vec(T+16*i);
            for(uint32_t at=B;at<B+0x170*10;at+=16)vec(at);for(uint32_t at=O;at<O+0x30*8;at+=16)vec(at);
            const auto random=[&](unsigned r,uint32_t low){m.r[r].uw[0]=low;for(unsigned i=1;i<4;++i)m.r[r].uw[i]=rnd();};
            m=R5900Context{};m.r[29].uw[0]=P0;m.r[16].uw[0]=E;
            if(seed&1){m.r[3].uw[0]=PARENT+0x10;run(0x245240);}else run(0x245248);
            run(0x245250);random(3,dt);run(0x245264);m.r[3].uw[0]=E+0xc0;run(0x24526c);run(0x2452a8);
            for(unsigned i=0;i<4;++i)both(P0+0x30+4*i,0x437f0000u);m.r[3].uw[0]=P0+0x30;run(0x2452b0);
            m=R5900Context{};m.r[29].uw[0]=S3;m.r[20].uw[0]=E;
            const unsigned trail=seed%4,rows=(seed/4)%3,n=2+seed%3;
            for(unsigned k=0;k<2;++k){
                const unsigned index=seed%5+3*k,ops=(seed/2+k)%4;const uint32_t rec=B+index*0x170,out=O+k*0x30*4;
                m.r[19].uw[0]=index;m.r[17].uw[0]=rec;m.r[16].uw[0]=rec+0x10;m.r[30].uw[0]=rec+0x30;
                if(ops==1){m.r[2].uw[0]=rec+0x50;for(uint32_t pc=0x2447e4;pc<=0x2447f4;pc+=4)run(pc);m.r[2].uw[0]=S3+0x210;run(0x2447fc);}
                if(ops==2 || ops==3){const uint32_t base=ops==2?0x2448d8:0x244958,store=ops==2?0x200:0x1f0,reload=ops==2?0x244940:0x2449a8;
                    m.r[3].uw[0]=E+0x80;for(uint32_t pc=base;pc<base+0x24;pc+=4)run(pc);m.r[2].uw[0]=S3+store;run(base+0x24);
                    random(2,dt);run(reload);m.r[2].uw[0]=E+0xc0;run(reload+0xc);}
                run(0x244b50);
                if(seed%3==0){vec(S3+0x1e0);m.r[2].uw[0]=S3+0x1e0;run(0x244c58);}else run(0x244ce0);
                run(0x244ce8);run(0x244cec);m.r[2].uw[0]=rec+0x40;run(0x244cf4);m.r[2].uw[0]=rec+0x50;run(0x244cfc);m.r[2].uw[0]=rec+0x20;run(0x244d04);
                m.r[22].uw[0]=rec+0x60;run(0x244d0c);m.r[21].uw[0]=rec+0x70;run(0x244d14);
                if(seed&2){vec(S3+0x260);m.r[2].uw[0]=S3+0x260;run(0x244d44);}
                for(unsigned j=0;j<rows;++j){const unsigned a1=j+seed%5;m.r[5].uw[0]=a1;m.r[2].uw[0]=T+16*a1;run(0x244dd0);
                    const auto fraction=value(115,8)&0x7fffffffu;m.f[1]=std::bit_cast<float>(fraction);random(2,fraction);run(0x244ddc);run(0x244de0);run(0x244de4);}
                for(uint32_t pc=0x244e1c;pc<=0x244e74;pc+=4)run(pc);
                m.r[23].uw[0]=out;run(0x244e7c);
                if(seed%4==3 || (seed%7==2 && k)){run(0x244e8c);vec(S3+0x260);vec(S3+0x250);m.r[2].uw[0]=S3+0x260;run(0x244ecc);m.r[2].uw[0]=S3+0x250;run(0x244ed4);run(0x244ee0);}
                run(0x244ee8);m.r[2].uw[0]=rec+0x50;run(0x244ef0);run(0x244ef4);run(0x244ef8);m.r[2].uw[0]=rec+0x40;run(0x244f00);m.r[2].uw[0]=rec+0x20;run(0x244f08);run(0x244f0c);
                if(trail){
                    const uint32_t head=trail==2?0x244f68:trail==1?0x245008:0x2450ac,first=trail==2?0x244f98:trail==1?0x245038:0x2450e0,last=trail==2?0x244fd4:trail==1?0x245070:0x2450fc;
                    m.r[4].uw[0]=out;run(head);random(2,read<uint32_t>(y,E+0x54));run(head+0x10);run(head+0x14);
                    for(unsigned i=1;i<n;++i){m.r[3].uw[0]=i;m.r[2].uw[0]=out+i*0x30;for(uint32_t pc=first;pc<=last;pc+=4)run(pc);++programPoints;}
                }
            }
            check(!std::memcmp(x.memory.hostPointer(B,0x170*10),y.memory.hostPointer(B,0x170*10),0x170*10));
            check(!std::memcmp(x.memory.hostPointer(O,0x30*8),y.memory.hostPointer(O,0x30*8),0x30*8));
            check(!std::memcmp(x.memory.hostPointer(S3+0x1f0,0x20),y.memory.hostPointer(S3+0x1f0,0x20),0x20));
            check(!std::memcmp(x.memory.hostPointer(S3+0x210,12),y.memory.hostPointer(S3+0x210,12),12));   // .w of the |A|^2 store is never read
            const auto box=now.particles.bounds(frame,E);
            for(unsigned i=0;i<3;++i)check(std::bit_cast<uint32_t>(box.minimum[i])==o.vu0_vf[26].u[i] && std::bit_cast<uint32_t>(box.maximum[i])==o.vu0_vf[27].u[i]);
            programCases+=2;
        }
        // A main site before the loads, a step without its prepare frame, a wrong record and another word reject.
        m=R5900Context{};m.r[29].uw[0]=S3;m.r[20].uw[0]=E;
        bool refused=false;try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x244e1c);}catch(const std::runtime_error &){refused=true;}check(refused);
        refused=false;try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x244b50);}catch(const std::runtime_error &){refused=true;}check(refused);   // state reset: no prepare
        m.r[29].uw[0]=P0;m.r[16].uw[0]=E;both(E+0xf8,0);check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x245248) && vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x245250));
        m.r[3].uw[0]=read<uint32_t>(y,0x35dee8);check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x245264));m.r[3].uw[0]=E+0xc0;
        check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x24526c) && vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2452a8));m.r[3].uw[0]=P0+0x30;
        check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2452b0));
        m.r[29].uw[0]=S3;m.r[20].uw[0]=E;check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x244b50) && vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x244ce0));
        m.r[19].uw[0]=2;m.r[17].uw[0]=B+0x170;m.r[16].uw[0]=B+0x180;refused=false;try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x244ce8);}catch(const std::runtime_error &){refused=true;}check(refused);
        write(y,0x244e1c,uint32_t(0));check(!vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x244e1c));
        // The same emitter compiled again for class 1585f0 (battle 0x1c): prepare 297970 (sites 527a0 higher), step 2961a0 (527a8 higher).
        for(const auto &[pc,w]:{std::pair{0x2979e8u,0x4bd2033cu},{0x2979f0u,0x4bd1033cu},{0x297a04u,0x48a32800u},{0x297a0cu,0xd8680000u},{0x297a48u,0x4be00798u},
                {0x297a50u,0xd87d0000u},{0x2972f8u,0x4bcf033cu},{0x297488u,0x4be00418u}})write(y,pc,w);
        m=R5900Context{};m.r[29].uw[0]=P0;m.r[16].uw[0]=E;both(E+0xf8,0);check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2979e8) && vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2979f0));
        m.r[3].uw[0]=read<uint32_t>(y,0x35dee8);check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x297a04));m.r[3].uw[0]=E+0xc0;
        check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x297a0c) && vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x297a48));m.r[3].uw[0]=P0+0x30;
        check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x297a50));
        m.r[29].uw[0]=S3;m.r[20].uw[0]=E;check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2972f8) && vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x297488));
        // The copy's records are 0x140 bytes (2962fc: (5i)64), not 0x170: its first particle load (297490) takes record 2 there.
        write(y,0x297490,0xda010000u);m.r[19].uw[0]=2;m.r[17].uw[0]=read<uint32_t>(y,E+0x14)+2*0x140;m.r[16].uw[0]=m.r[17].uw[0]+0x10;
        check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x297490));
        write(y,0x2972f8,uint32_t(0));check(!vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2972f8));
        std::printf("%u general emitter particles (%u trail points, end conditions, jitter, push, curves, collision, three trail kinds) match the VU0 reference\n",programCases,programPoints);}
        // mod001 295f20 + 295580, the second emitter class (vtables 1563f0 / 156500): prepare's constant registers,
        // then per particle of 0xb0 bytes the loads, parent push, curves, the main step, output, early velocity,
        // collision, the stores of each +d0 kind (2: |A|^2, 5: rows * P, whose row loads become the next particles'
        // gravity and push) and the three trail kinds, against the same VU0 reference.
        {const std::pair<uint32_t,uint32_t> words[]={
                {0x295f90,0xd8690000},{0x295f98,0x4bc9033c},{0x295fa4,0xd86d0000},{0x295fa8,0x4bca033c},{0x295fbc,0x48a32800},{0x295ff4,0x4be00798},{0x295ffc,0xd87d0000},
                {0x295708,0xda610000},{0x295714,0xda420000},{0x295720,0xdae40000},{0x29572c,0xda230000},{0x295738,0xda060000},{0x29574c,0xd8470000},{0x295760,0xd8480000},
                {0x29578c,0xd84a0000},{0x295818,0xd84b0000},{0x295824,0x48a26000},{0x295828,0x4bc00abc},{0x29582c,0x4bcc5848},
                {0x295868,0x4bc00abc},{0x29586c,0x4bc068bf},{0x295870,0x4bc51048},{0x295874,0x4bfe32bc},{0x295878,0x4be53988},{0x29587c,0x4bc01abc},{0x295880,0x4bc548c8},
                {0x295884,0x4bc022bc},{0x295888,0x4bc51108},{0x29588c,0x4bc012bc},{0x295890,0x4bc550bc},{0x295894,0x4bc51888},{0x295898,0x4a251084},{0x29589c,0x4be03190},
                {0x2958a0,0x4a252100},{0x2958a4,0x4be039bf},{0x2958a8,0x4be541c8},{0x2958ac,0x4bc1d6af},{0x2958b0,0x4bfd3194},{0x2958b4,0x4bc1deeb},
                {0x2958b8,0xfbc10000},{0x2958c8,0xfa420000},{0x2958f4,0xd84b0000},{0x2958fc,0xd8410000},{0x295908,0x4bcb10a8},{0x295910,0xfbc60010},
                {0x295a80,0x4bc007c3},{0x295a88,0x4bc4212a},{0x295aa4,0x4b04203d},{0x295aa8,0x4b04f90a},{0x295ab4,0xf8440000},
                {0x295b24,0xd8480000},{0x295b28,0xd8490010},{0x295b2c,0xd84a0020},{0x295b30,0xd84b0030},{0x295b34,0x4be141bc},{0x295b38,0x4be148bd},{0x295b3c,0x4be150be},
                {0x295b40,0x4be05c8b},{0x295b64,0xf8520000},
                {0x295cd0,0xd8810000},{0x295ce0,0x48a21800},{0x295ce4,0x4be40b3c},{0x295d00,0xd8420000},{0x295d04,0x4be111ac},{0x295d08,0x4be1133c},{0x295d0c,0x4bc007c3},
                {0x295d10,0x4bc631ea},{0x295d14,0x4b07383d},{0x295d18,0x4b07f9ca},{0x295d1c,0x4a6703be},{0x295d20,0x4a0003bf},{0x295d24,0x4bc0321c},{0x295d28,0x4bc022bc},
                {0x295d2c,0x4bc030bf},{0x295d30,0x4bc34108},{0x295d34,0xf8440000},{0x295d38,0x4bc4d6af},{0x295d3c,0x4bc4deeb},
                {0x295d70,0xd8810000},{0x295d80,0x48a21800},{0x295d84,0x4be40b3c},{0x295da0,0xd8420000},{0x295da4,0x4be111ac},{0x295da8,0x4be1133c},{0x295dac,0x4bc007c3},
                {0x295db0,0x4bc631ea},{0x295db4,0x4b07383d},{0x295db8,0x4b07f9ca},{0x295dbc,0x4a6703be},{0x295dc0,0x4a0003bf},{0x295dc4,0x4bc0319c},{0x295dc8,0x4bc022bc},
                {0x295dcc,0x4bc33108},{0x295dd0,0xf8440000},{0x295dd4,0x4bc4d6af},{0x295dd8,0x4bc4deeb},
                {0x295e10,0xd8810000},{0x295e20,0x48a21800},{0x295e24,0x4be40b3c},{0x295e40,0xd8420000},{0x295e44,0x4be111ac},{0x295e48,0x4be1133c},{0x295e4c,0x4bc022bc},
                {0x295e50,0x4bc33108},{0x295e54,0xf8440000},{0x295e58,0x4bc4d6af},{0x295e5c,0x4bc4deeb}};
            // The six record stores (A, P, V, acc, C, R) at each kind's base: 1, 3, 2 (with the squares between), 5, 6, default.
            constexpr uint32_t storeWords[6]={0xfae40000,0xfa610000,0xfa420000,0xfa230000,0xfa060000,0xf8470000};
            constexpr uint32_t kindStores[6][6]={{0x295970,0x295974,0x295978,0x29597c,0x295980,0x295988},{0x2959f0,0x2959f4,0x2959f8,0x2959fc,0x295a00,0x295a08},
                {0x295a84,0x295a8c,0x295a90,0x295a94,0x295a98,0x295aa0},{0x295b44,0x295b48,0x295b4c,0x295b50,0x295b54,0x295b5c},
                {0x295bd0,0x295bd4,0x295bd8,0x295bdc,0x295be0,0x295be8},{0x295c80,0x295c84,0x295c88,0x295c8c,0x295c90,0x295c98}};
            for(const auto &[pc,w]:words){write(y,pc,w);write(x,pc,w);}
            for(const auto &k:kindStores)for(unsigned i=0;i<6;++i){write(y,k[i],storeWords[i]);write(x,k[i],storeWords[i]);}
            const uint32_t P0=frame-0x40,S4=frame-0x1b0,PARENT=0x1e0000;unsigned simpleCases=0,simplePoints=0;
            const auto both=[&](uint32_t at,uint32_t v){write(y,at,v);write(x,at,v);};
            for(unsigned seed=0;seed<120;++seed){
                uint32_t state=seed*2654435761u+17;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
                const auto value=[&](unsigned lo=105,unsigned span=40){return (rnd()&0x807fffffu)|((lo+rnd()%span)<<23);};
                const auto vec=[&](uint32_t at){for(unsigned i=0;i<4;++i)both(at+4*i,value());};
                const uint32_t dt=value(118,6)&0x7fffffffu;both(0x35dee8,dt);vec(E+0xc0);both(E+0x54,value(122,6));
                both(E+0x14,B);both(E+0x68,T);both(E+0xf0,seed&1?PARENT:0);vec(PARENT+0x10);
                for(unsigned i=0;i<4;++i)vec(E+0x80+16*i);for(unsigned i=0;i<16;++i)vec(T+16*i);
                for(uint32_t at=B;at<B+0xb0*10;at+=16)vec(at);for(uint32_t at=O;at<O+0x30*8;at+=16)vec(at);
                const auto random=[&](unsigned r,uint32_t low){m.r[r].uw[0]=low;for(unsigned i=1;i<4;++i)m.r[r].uw[i]=rnd();};
                m=R5900Context{};m.r[29].uw[0]=P0;m.r[16].uw[0]=E;
                if(seed&1){m.r[3].uw[0]=PARENT+0x10;run(0x295f90);}else run(0x295f98);
                m.r[3].uw[0]=E+0xc0;run(0x295fa4);run(0x295fa8);random(3,dt);run(0x295fbc);run(0x295ff4);
                for(unsigned i=0;i<4;++i)both(P0+0x30+4*i,0x437f0000u);m.r[3].uw[0]=P0+0x30;run(0x295ffc);
                m=R5900Context{};m.r[29].uw[0]=S4;m.r[21].uw[0]=E;
                const unsigned kind=seed%6,trail=(seed/6)%4,rows=(seed/3)%3,n=2+seed%3;
                for(unsigned k=0;k<3;++k){
                    const unsigned index=seed%4+3*k;const uint32_t rec=B+index*0xb0,out=O+(k%2)*0x30*4;
                    m.r[20].uw[0]=index;m.r[19].uw[0]=rec+0x10;run(0x295708);m.r[18].uw[0]=rec+0x30;run(0x295714);m.r[23].uw[0]=rec+0x50;run(0x295720);
                    m.r[17].uw[0]=rec+0x40;run(0x29572c);m.r[16].uw[0]=rec+0x20;run(0x295738);m.r[2].uw[0]=rec+0x60;run(0x29574c);m.r[2].uw[0]=rec+0x70;run(0x295760);
                    if(seed&2){vec(S4+0x1a0);m.r[2].uw[0]=S4+0x1a0;run(0x29578c);}
                    for(unsigned j=0;j<rows;++j){const unsigned a1=j+seed%5;m.r[5].uw[0]=a1;m.r[2].uw[0]=T+16*a1;run(0x295818);
                        const auto fraction=value(115,8)&0x7fffffffu;m.f[1]=std::bit_cast<float>(fraction);random(2,fraction);run(0x295824);run(0x295828);run(0x29582c);}
                    for(uint32_t pc=0x295868;pc<=0x2958b4;pc+=4)run(pc);
                    m.r[30].uw[0]=out;run(0x2958b8);run(0x2958c8);
                    if(seed%4==3 || (seed%7==2 && k)){vec(S4+0x1a0);vec(S4+0x190);m.r[2].uw[0]=S4+0x1a0;run(0x2958f4);m.r[2].uw[0]=S4+0x190;run(0x2958fc);run(0x295908);}
                    run(0x295910);
                    const unsigned which=kind==0?0:kind==1?1:kind==2?2:kind==3?3:kind==4?4:5;const auto &st=kindStores[which];
                    if(which==2)run(0x295a80);
                    if(which==3){m.r[2].uw[0]=E+0x80;for(uint32_t pc=0x295b24;pc<=0x295b40;pc+=4)run(pc);}
                    for(unsigned i=0;i<6;++i){if(i==5)m.r[2].uw[0]=rec+0x60;run(st[i]);if(which==2 && i==0)run(0x295a88);}
                    if(which==2){run(0x295aa4);run(0x295aa8);m.r[2].uw[0]=S4+0x180;run(0x295ab4);}
                    if(which==3){m.r[2].uw[0]=S4+0x170;run(0x295b64);}
                    if(trail){
                        const uint32_t head=trail==2?0x295cd0:trail==1?0x295d70:0x295e10,first=trail==2?0x295d00:trail==1?0x295da0:0x295e40,last=trail==2?0x295d3c:trail==1?0x295dd8:0x295e5c;
                        m.r[4].uw[0]=out;run(head);random(2,read<uint32_t>(y,E+0x54));run(head+0x10);run(head+0x14);
                        for(unsigned i=1;i<n;++i){m.r[3].uw[0]=i;m.r[2].uw[0]=out+i*0x30;for(uint32_t pc=first;pc<=last;pc+=4)run(pc);++simplePoints;}
                    }
                }
                check(!std::memcmp(x.memory.hostPointer(B,0xb0*10),y.memory.hostPointer(B,0xb0*10),0xb0*10));
                check(!std::memcmp(x.memory.hostPointer(O,0x30*8),y.memory.hostPointer(O,0x30*8),0x30*8));
                check(!std::memcmp(x.memory.hostPointer(S4+0x170,0x20),y.memory.hostPointer(S4+0x170,0x20),0x20));
                const auto box=now.particles.bounds(frame,E);
                for(unsigned i=0;i<3;++i)check(std::bit_cast<uint32_t>(box.minimum[i])==o.vu0_vf[26].u[i] && std::bit_cast<uint32_t>(box.maximum[i])==o.vu0_vf[27].u[i]);
                simpleCases+=3;
            }
            // A main site before the loads, a store without its prepare, a wrong record and another word reject.
            m=R5900Context{};m.r[29].uw[0]=S4;m.r[21].uw[0]=E;
            bool refused=false;try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x295868);}catch(const std::runtime_error &){refused=true;}check(refused);
            refused=false;try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x295708);}catch(const std::runtime_error &){refused=true;}check(refused);   // state reset: no prepare
            m.r[29].uw[0]=P0;m.r[16].uw[0]=E;both(E+0xf0,0);check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x295f98));
            m.r[3].uw[0]=E+0xc0;check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x295fa4) && vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x295fa8));
            m.r[3].uw[0]=read<uint32_t>(y,0x35dee8);check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x295fbc) && vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x295ff4));
            m.r[3].uw[0]=P0+0x30;check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x295ffc));
            m.r[29].uw[0]=S4;m.r[21].uw[0]=E;m.r[20].uw[0]=2;m.r[19].uw[0]=B+0xb0+0x10;
            refused=false;try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x295708);}catch(const std::runtime_error &){refused=true;}check(refused);   // record of index 1, not 2
            write(y,0x295868,uint32_t(0));check(!vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x295868));
            std::printf("%u second-class emitter particles (%u trail points, six store kinds, push, curves, collision, three trail kinds) match the VU0 reference\n",simpleCases,simplePoints);}
        // mod001 2932f0, the row emitter (battle 0x1c's mode-0 emitter): its own constants, per particle of 0xf0 bytes the two row
        // steps and stores, the loads, curves, main step, output, the stores of each +d0 kind (2: |A|^2, 5: e, f, then rows * P)
        // and the three trail kinds, against the same VU0 reference.
        {const std::pair<uint32_t,uint32_t> words[]={
                {0x2933a8,0x48a22800},{0x2933b0,0xd8540000},{0x2933ec,0x4be00798},{0x2933f4,0xd85d0000},
                {0x2934c8,0xda410000},{0x2934e0,0xd8490000},{0x2934e8,0xd84a0000},{0x2934f0,0xdaeb0000},{0x2934f8,0xdbcd0000},
                {0x293504,0x4bc04abc},{0x293508,0x4bc55a48},{0x29350c,0x4bc052bc},{0x293510,0x4bc56a88},{0x293518,0xf8490000},{0x293520,0xf84a0000},
                {0x2935b4,0xda220000},{0x2935bc,0xd9040000},{0x2935cc,0xd8430000},{0x2935d4,0xda060000},{0x2935dc,0xd8670000},{0x2935e4,0xd8480000},
                {0x2935ec,0xd84f0000},{0x2935f4,0xd84c0000},{0x2935fc,0xd84e0000},{0x293600,0xdaeb0000},{0x293604,0xdbcd0000},
                {0x293690,0xd8900000},{0x29369c,0x48a48800},{0x2936a0,0x4bc00abc},{0x2936a4,0x4bd18048},
                {0x2936dc,0x4bc00abc},{0x2936e0,0x4bc078bf},{0x2936e4,0x4bc0a0bf},{0x2936e8,0x4bc51048},{0x2936ec,0x4bfe32bc},{0x2936f0,0x4be53988},
                {0x2936f4,0x4bc022bc},{0x2936f8,0x4bc078bf},{0x2936fc,0x4bc51108},{0x293700,0x4bc012bc},{0x293704,0x4bc51888},{0x293708,0x4a251084},
                {0x29370c,0x4be03190},{0x293710,0x4a252100},{0x293714,0x4be039bf},{0x293718,0x4be541c8},{0x29371c,0x4bc1d6af},{0x293720,0x4bfd3194},
                {0x293724,0x4bc1deeb},{0x293728,0x4bc05abc},{0x29372c,0x4bc562c8},{0x293730,0x4bc06abc},{0x293734,0x4bc57348},
                {0x29373c,0xf8810000},{0x293744,0xf8860010},
                {0x2938c8,0x4bc007c3},{0x2938d0,0x4bc4212a},{0x2938ec,0x4b04203d},{0x2938f0,0x4b04f90a},{0x2938fc,0xf8440000},
                {0x29397c,0xd8480000},{0x293980,0xd8490010},{0x293984,0xd84a0020},{0x293988,0xd84b0030},{0x29398c,0x4be141bc},{0x293990,0x4be148bd},
                {0x293994,0x4be150be},{0x293998,0x4be05c8b},{0x2939b4,0xf8520000},
                {0x293b2c,0xd8810000},{0x293b3c,0x48a21800},{0x293b40,0x4be40b3c},{0x293b60,0xd8420000},{0x293b64,0x4be111ac},{0x293b68,0x4be1133c},
                {0x293b6c,0x4bc007c3},{0x293b70,0x4bc631ea},{0x293b74,0x4b07383d},{0x293b78,0x4b07f9ca},{0x293b7c,0x4a6703be},{0x293b80,0x4a0003bf},
                {0x293b84,0x4bc0321c},{0x293b88,0x4bc022bc},{0x293b8c,0x4bc030bf},{0x293b90,0x4bc34108},{0x293b94,0xf8440000},{0x293b98,0x4bc4d6af},{0x293b9c,0x4bc4deeb},
                {0x293bd4,0xd8810000},{0x293be4,0x48a21800},{0x293be8,0x4be40b3c},{0x293c08,0xd8420000},{0x293c0c,0x4be111ac},{0x293c10,0x4be1133c},
                {0x293c14,0x4bc007c3},{0x293c18,0x4bc631ea},{0x293c1c,0x4b07383d},{0x293c20,0x4b07f9ca},{0x293c24,0x4a6703be},{0x293c28,0x4a0003bf},
                {0x293c2c,0x4bc0319c},{0x293c30,0x4bc022bc},{0x293c34,0x4bc33108},{0x293c38,0xf8440000},{0x293c3c,0x4bc4d6af},{0x293c40,0x4bc4deeb},
                {0x293c7c,0xd8810000},{0x293c8c,0x48a21800},{0x293c90,0x4be40b3c},{0x293cb0,0xd8420000},{0x293cb4,0x4be111ac},{0x293cb8,0x4be1133c},
                {0x293cbc,0x4bc022bc},{0x293cc0,0x4bc33108},{0x293cc4,0xf8440000},{0x293cc8,0x4bc4d6af},{0x293ccc,0x4bc4deeb}};
            // The seven record stores (A, P, V, C, R, e, f) at each kind's base: 1, 3, 2 (squares around), 6, default; 5 stores e, f first.
            constexpr uint32_t storeWords[7]={0xf9040000,0xfa410000,0xfa220000,0xfa060000,0xf8670000,0xfaeb0000,0xfbcd0000};
            constexpr uint32_t kindStores[5][7]={{0x2937b0,0x2937b4,0x2937b8,0x2937bc,0x2937c0,0x2937c4,0x2937c8},{0x293838,0x29383c,0x293840,0x293844,0x293848,0x29384c,0x293850},
                {0x2938cc,0x2938d4,0x2938d8,0x2938dc,0x2938e0,0x2938e4,0x2938e8},{0x293a28,0x293a2c,0x293a30,0x293a34,0x293a38,0x293a3c,0x293a40},
                {0x293ad8,0x293adc,0x293ae0,0x293ae4,0x293ae8,0x293aec,0x293af0}};
            constexpr uint32_t kind5Stores[7]={0x29399c,0x2939a0,0x2939a4,0x2939a8,0x2939ac,0x293970,0x293974};   // A, P, V, C, R, e, f
            for(const auto &[pc,w]:words){write(y,pc,w);write(x,pc,w);}
            for(const auto &k:kindStores)for(unsigned i=0;i<7;++i){write(y,k[i],storeWords[i]);write(x,k[i],storeWords[i]);}
            for(unsigned i=0;i<7;++i){write(y,kind5Stores[i],storeWords[i]);write(x,kind5Stores[i],storeWords[i]);}
            const uint32_t S5=frame-0x1e0;unsigned rowCases=0,rowPoints=0;
            const auto both=[&](uint32_t at,uint32_t v){write(y,at,v);write(x,at,v);};
            for(unsigned seed=0;seed<120;++seed){
                uint32_t state=seed*2654435761u+29;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
                const auto value=[&](unsigned lo=105,unsigned span=40){return (rnd()&0x807fffffu)|((lo+rnd()%span)<<23);};
                const auto vec=[&](uint32_t at){for(unsigned i=0;i<4;++i)both(at+4*i,value());};
                const uint32_t dt=value(118,6)&0x7fffffffu;both(0x35dee8,dt);vec(E+0xc0);both(E+0x54,value(122,6));
                both(E+0x14,B);both(E+0x68,T);
                for(unsigned i=0;i<4;++i)vec(E+0x80+16*i);for(unsigned i=0;i<16;++i)vec(T+16*i);
                for(uint32_t at=B;at<B+0xf0*10;at+=16)vec(at);for(uint32_t at=O;at<O+0x30*8;at+=16)vec(at);
                vec(S5+0x180);vec(S5+0x190);   // the kind 5 / kind 2 results, alike in both before the case
                const auto random=[&](unsigned r,uint32_t low){m.r[r].uw[0]=low;for(unsigned i=1;i<4;++i)m.r[r].uw[i]=rnd();};
                m=R5900Context{};m.r[29].uw[0]=S5;m.r[21].uw[0]=E;
                random(2,dt);run(0x2933a8);m.r[2].uw[0]=E+0xc0;run(0x2933b0);run(0x2933ec);
                for(unsigned i=0;i<4;++i)both(S5+0x1d0+4*i,0x437f0000u);m.r[2].uw[0]=S5+0x1d0;run(0x2933f4);
                const unsigned kind=seed%6,trail=(seed/6)%4,rows=(seed/3)%3,n=2+seed%3;
                for(unsigned k=0;k<3;++k){
                    const unsigned index=seed%4+3*k;const uint32_t rec=B+index*0xf0,out=O+(k%2)*0x30*4;
                    m.r[20].uw[0]=index;m.r[18].uw[0]=rec+0x10;run(0x2934c8);m.r[2].uw[0]=rec+0xb0;run(0x2934e0);m.r[2].uw[0]=rec+0x80;run(0x2934e8);
                    m.r[23].uw[0]=rec+0xc0;run(0x2934f0);m.r[30].uw[0]=rec+0x90;run(0x2934f8);
                    for(uint32_t pc=0x293504;pc<=0x293510;pc+=4)run(pc);
                    m.r[2].uw[0]=rec+0xb0;run(0x293518);m.r[2].uw[0]=rec+0x80;run(0x293520);
                    vec(S5+0x1a0);   // the scalar offset
                    m.r[17].uw[0]=rec+0x30;run(0x2935b4);m.r[8].uw[0]=rec+0x50;run(0x2935bc);m.r[2].uw[0]=rec+0x40;run(0x2935cc);m.r[16].uw[0]=rec+0x20;run(0x2935d4);
                    m.r[3].uw[0]=rec+0x60;run(0x2935dc);m.r[2].uw[0]=rec+0x70;run(0x2935e4);m.r[2].uw[0]=S5+0x1a0;run(0x2935ec);m.r[2].uw[0]=rec+0xd0;run(0x2935f4);
                    m.r[2].uw[0]=rec+0xa0;run(0x2935fc);run(0x293600);run(0x293604);
                    for(unsigned j=0;j<rows;++j){const unsigned a3=j+seed%5;m.r[7].uw[0]=a3;m.r[4].uw[0]=T+16*a3;run(0x293690);
                        const auto fraction=value(115,8)&0x7fffffffu;m.f[1]=std::bit_cast<float>(fraction);random(4,fraction);run(0x29369c);run(0x2936a0);run(0x2936a4);}
                    for(uint32_t pc=0x2936dc;pc<=0x293734;pc+=4)run(pc);
                    both(S5+0x160,out);m.r[4].uw[0]=out;run(0x29373c);run(0x293744);
                    const unsigned which=kind%6;   // 0..4: kinds 1, 3, 2, 6, default; 5: kind 5
                    if(which==5){m.r[23].uw[0]=rec+0xc0;m.r[30].uw[0]=rec+0x90;run(0x293970);run(0x293974);
                        m.r[2].uw[0]=E+0x80;for(uint32_t pc=0x29397c;pc<=0x293998;pc+=4)run(pc);
                        for(unsigned i=0;i<5;++i)run(kind5Stores[i]);m.r[2].uw[0]=S5+0x180;run(0x2939b4);}
                    else{const auto &st=kindStores[which];
                        if(which==2)run(0x2938c8);
                        for(unsigned i=0;i<7;++i){run(st[i]);if(which==2 && i==0)run(0x2938d0);}
                        if(which==2){run(0x2938ec);run(0x2938f0);m.r[2].uw[0]=S5+0x190;run(0x2938fc);}}
                    if(trail){
                        const uint32_t head=trail==2?0x293b2c:trail==1?0x293bd4:0x293c7c,first=trail==2?0x293b60:trail==1?0x293c08:0x293cb0,last=trail==2?0x293b9c:trail==1?0x293c40:0x293ccc;
                        m.r[4].uw[0]=out;run(head);random(2,read<uint32_t>(y,E+0x54));run(head+0x10);run(head+0x14);
                        for(unsigned i=1;i<n;++i){m.r[3].uw[0]=i;m.r[2].uw[0]=out+i*0x30;for(uint32_t pc=first;pc<=last;pc+=4)run(pc);++rowPoints;}
                    }
                }
                check(!std::memcmp(x.memory.hostPointer(B,0xf0*10),y.memory.hostPointer(B,0xf0*10),0xf0*10));
                check(!std::memcmp(x.memory.hostPointer(O,0x30*8),y.memory.hostPointer(O,0x30*8),0x30*8));
                if(std::memcmp(x.memory.hostPointer(S5+0x180,0x20),y.memory.hostPointer(S5+0x180,0x20),0x20)){
                    for(unsigned i=0;i<8;++i)std::fprintf(stderr,"seed %u kind %u sp+%x: ref %08x native %08x\n",seed,kind,0x180+4*i,read<uint32_t>(x,S5+0x180+4*i),read<uint32_t>(y,S5+0x180+4*i));check(false);}
                const auto box=now.particles.bounds(frame,E);
                for(unsigned i=0;i<3;++i)check(std::bit_cast<uint32_t>(box.minimum[i])==o.vu0_vf[26].u[i] && std::bit_cast<uint32_t>(box.maximum[i])==o.vu0_vf[27].u[i]);
                rowCases+=3;
            }
            // A main site without the step's constants, a wrong record and another word reject.
            bool refused=false;m=R5900Context{};m.r[29].uw[0]=S5+0x10;m.r[21].uw[0]=E;
            try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2936dc);}catch(const std::runtime_error &){refused=true;}check(refused);
            m.r[29].uw[0]=S5;m.r[2].uw[0]=read<uint32_t>(y,0x35dee8);check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2933a8));
            m.r[2].uw[0]=E+0xc0;check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2933b0) && vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2933ec));
            m.r[2].uw[0]=S5+0x1d0;check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2933f4));
            m.r[20].uw[0]=2;m.r[18].uw[0]=B+0xf0+0x10;refused=false;
            try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2934c8);}catch(const std::runtime_error &){refused=true;}check(refused);   // record of index 1, not 2
            write(y,0x2936dc,uint32_t(0));check(!vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2936dc));
            std::printf("%u row emitter particles (%u trail points, six store kinds, row steps, offset, curves, three trail kinds) match the VU0 reference\n",rowCases,rowPoints);}
        // mod001 2920e0, the plain emitter (battle 0x3e; the row emitter without rows, records of 0x80 bytes, frame 0x170, output s7):
        // its constants, the six loads, curves, the main step, output, the five stores of each +d0 kind and the three trails, against
        // the same VU0 reference.
        {const std::pair<uint32_t,uint32_t> words[]={
                {0x292184,0x48a22800},{0x29218c,0xd8540000},{0x2921c8,0x4be00798},{0x2921d0,0xd85d0000},{0x2922a8,0xda410000},{0x2922b0,0xda220000},
                {0x2922b8,0xd9040000},{0x2922c0,0xdac30000},{0x2922c8,0xda060000},{0x2922d0,0xd8670000},{0x2922d8,0xd8480000},{0x292368,0xd88b0000},
                {0x292374,0x48a46000},{0x292378,0x4bc00abc},{0x29237c,0x4bcc5848},{0x2923b4,0x4bc00abc},{0x2923b8,0x4bc0a0bf},{0x2923bc,0x4bc51048},
                {0x2923c0,0x4bfe32bc},{0x2923c4,0x4be53988},{0x2923c8,0x4bc022bc},{0x2923cc,0x4bc51108},{0x2923d0,0x4bc012bc},{0x2923d4,0x4bc51888},
                {0x2923d8,0x4a251084},{0x2923dc,0x4be03190},{0x2923e0,0x4a252100},{0x2923e4,0x4be039bf},{0x2923e8,0x4be541c8},{0x2923ec,0x4bc1d6af},
                {0x2923f0,0x4bfd3194},{0x2923f4,0x4bc1deeb},{0x2923f8,0xfae10000},{0x2923fc,0xfae60010},{0x292460,0xf9040000},{0x292464,0xfa410000},
                {0x292468,0xfa220000},{0x29246c,0xfa060000},{0x292470,0xf8670000},{0x2924d8,0xf9040000},{0x2924dc,0xfa410000},{0x2924e0,0xfa220000},
                {0x2924e4,0xfa060000},{0x2924e8,0xf8670000},{0x292560,0x4bc007c3},{0x292564,0xf9040000},{0x292568,0x4bc4212a},{0x29256c,0xfa410000},
                {0x292570,0xfa220000},{0x292574,0xfa060000},{0x292578,0xf8670000},{0x29257c,0x4b04203d},{0x292580,0x4b04f90a},{0x29258c,0xf8440000},
                {0x2925fc,0xd8480000},{0x292600,0xd8490010},{0x292604,0xd84a0020},{0x292608,0xd84b0030},{0x29260c,0x4be141bc},{0x292610,0x4be148bd},
                {0x292614,0x4be150be},{0x292618,0x4be05c8b},{0x29261c,0xf9040000},{0x292620,0xfa410000},{0x292624,0xfa220000},{0x292628,0xfa060000},
                {0x29262c,0xf8670000},{0x292634,0xf8520000},{0x2926a0,0xf9040000},{0x2926a4,0xfa410000},{0x2926a8,0xfa220000},{0x2926ac,0xfa060000},
                {0x2926b0,0xf8670000},{0x292740,0xf9040000},{0x292744,0xfa410000},{0x292748,0xfa220000},{0x29274c,0xfa060000},{0x292750,0xf8670000},
                {0x292788,0xd8810000},{0x292798,0x48a21800},{0x29279c,0x4be40b3c},{0x2927b8,0xd8420000},{0x2927bc,0x4be111ac},{0x2927c0,0x4be1133c},
                {0x2927c4,0x4bc007c3},{0x2927c8,0x4bc631ea},{0x2927cc,0x4b07383d},{0x2927d0,0x4b07f9ca},{0x2927d4,0x4a6703be},{0x2927d8,0x4a0003bf},
                {0x2927dc,0x4bc0321c},{0x2927e0,0x4bc022bc},{0x2927e4,0x4bc030bf},{0x2927e8,0x4bc34108},{0x2927ec,0xf8440000},{0x2927f0,0x4bc4d6af},
                {0x2927f4,0x4bc4deeb},{0x292828,0xd8810000},{0x292838,0x48a21800},{0x29283c,0x4be40b3c},{0x292858,0xd8420000},{0x29285c,0x4be111ac},
                {0x292860,0x4be1133c},{0x292864,0x4bc007c3},{0x292868,0x4bc631ea},{0x29286c,0x4b07383d},{0x292870,0x4b07f9ca},{0x292874,0x4a6703be},
                {0x292878,0x4a0003bf},{0x29287c,0x4bc0319c},{0x292880,0x4bc022bc},{0x292884,0x4bc33108},{0x292888,0xf8440000},{0x29288c,0x4bc4d6af},
                {0x292890,0x4bc4deeb},{0x2928c8,0xd8810000},{0x2928d8,0x48a21800},{0x2928dc,0x4be40b3c},{0x2928f8,0xd8420000},{0x2928fc,0x4be111ac},
                {0x292900,0x4be1133c},{0x292904,0x4bc022bc},{0x292908,0x4bc33108},{0x29290c,0xf8440000},{0x292910,0x4bc4d6af},{0x292914,0x4bc4deeb},
            };
            for(const auto &[pc,w]:words){write(y,pc,w);write(x,pc,w);}
            constexpr uint32_t kindStores[5][5]={{0x292460,0x292464,0x292468,0x29246c,0x292470},{0x2924d8,0x2924dc,0x2924e0,0x2924e4,0x2924e8},
                {0x292564,0x29256c,0x292570,0x292574,0x292578},{0x2926a0,0x2926a4,0x2926a8,0x2926ac,0x2926b0},{0x292740,0x292744,0x292748,0x29274c,0x292750}};
            constexpr uint32_t kind5Stores[5]={0x29261c,0x292620,0x292624,0x292628,0x29262c};
            const uint32_t S7=frame-0x170;unsigned plainCases=0,plainPoints=0;
            const auto both=[&](uint32_t at,uint32_t v){write(y,at,v);write(x,at,v);};
            for(unsigned seed=0;seed<120;++seed){
                uint32_t state=seed*2246822519u+31;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
                const auto value=[&](unsigned lo=105,unsigned span=40){return (rnd()&0x807fffffu)|((lo+rnd()%span)<<23);};
                const auto vec=[&](uint32_t at){for(unsigned i=0;i<4;++i)both(at+4*i,value());};
                const uint32_t dt=value(118,6)&0x7fffffffu;both(0x35dee8,dt);vec(E+0xc0);both(E+0x54,value(122,6));
                both(E+0x14,B);both(E+0x68,T);
                for(unsigned i=0;i<4;++i)vec(E+0x80+16*i);for(unsigned i=0;i<16;++i)vec(T+16*i);
                for(uint32_t at=B;at<B+0x80*12;at+=16)vec(at);for(uint32_t at=O;at<O+0x30*8;at+=16)vec(at);
                vec(S7+0x140);vec(S7+0x150);
                const auto random=[&](unsigned r,uint32_t low){m.r[r].uw[0]=low;for(unsigned i=1;i<4;++i)m.r[r].uw[i]=rnd();};
                m=R5900Context{};m.r[29].uw[0]=S7;m.r[21].uw[0]=E;
                random(2,dt);run(0x292184);m.r[2].uw[0]=E+0xc0;run(0x29218c);run(0x2921c8);
                for(unsigned i=0;i<4;++i)both(S7+0x160+4*i,0x437f0000u);m.r[2].uw[0]=S7+0x160;run(0x2921d0);
                const unsigned kind=seed%6,trail=(seed/6)%4,rows=(seed/3)%3,n=2+seed%3;
                for(unsigned k=0;k<3;++k){
                    const unsigned index=seed%4+3*k;const uint32_t rec=B+index*0x80,out=O+(k%2)*0x30*4;
                    m.r[20].uw[0]=index;m.r[18].uw[0]=rec+0x10;run(0x2922a8);m.r[17].uw[0]=rec+0x30;run(0x2922b0);m.r[8].uw[0]=rec+0x50;run(0x2922b8);
                    m.r[22].uw[0]=rec+0x40;run(0x2922c0);m.r[16].uw[0]=rec+0x20;run(0x2922c8);m.r[3].uw[0]=rec+0x60;run(0x2922d0);m.r[2].uw[0]=rec+0x70;run(0x2922d8);
                    for(unsigned j=0;j<rows;++j){const unsigned t1=j+seed%5;m.r[9].uw[0]=t1;m.r[4].uw[0]=T+16*t1;run(0x292368);
                        const auto fraction=value(115,8)&0x7fffffffu;m.f[1]=std::bit_cast<float>(fraction);random(4,fraction);run(0x292374);run(0x292378);run(0x29237c);}
                    for(uint32_t pc=0x2923b4;pc<=0x2923f4;pc+=4)run(pc);
                    m.r[23].uw[0]=out;run(0x2923f8);run(0x2923fc);
                    const unsigned which=kind%6;   // 0..4: kinds 1, 3, 2, 6, default; 5: kind 5
                    if(which==5){m.r[2].uw[0]=E+0x80;for(uint32_t pc=0x2925fc;pc<=0x292618;pc+=4)run(pc);
                        for(unsigned i=0;i<5;++i)run(kind5Stores[i]);m.r[2].uw[0]=S7+0x140;run(0x292634);}
                    else{const auto &st=kindStores[which];
                        if(which==2)run(0x292560);
                        for(unsigned i=0;i<5;++i){run(st[i]);if(which==2 && i==0)run(0x292568);}
                        if(which==2){run(0x29257c);run(0x292580);m.r[2].uw[0]=S7+0x150;run(0x29258c);}}
                    if(trail){
                        const uint32_t head=trail==2?0x292788:trail==1?0x292828:0x2928c8,first=trail==2?0x2927b8:trail==1?0x292858:0x2928f8,last=trail==2?0x2927f4:trail==1?0x292890:0x292914;
                        m.r[4].uw[0]=out;run(head);random(2,read<uint32_t>(y,E+0x54));run(head+0x10);run(head+0x14);
                        for(unsigned i=1;i<n;++i){m.r[3].uw[0]=i;m.r[2].uw[0]=out+i*0x30;for(uint32_t pc=first;pc<=last;pc+=4)run(pc);++plainPoints;}
                    }
                }
                check(!std::memcmp(x.memory.hostPointer(B,0x80*12),y.memory.hostPointer(B,0x80*12),0x80*12));
                check(!std::memcmp(x.memory.hostPointer(O,0x30*8),y.memory.hostPointer(O,0x30*8),0x30*8));
                check(!std::memcmp(x.memory.hostPointer(S7+0x140,0x20),y.memory.hostPointer(S7+0x140,0x20),0x20));
                const auto box=now.particles.bounds(frame,E);
                for(unsigned i=0;i<3;++i)check(std::bit_cast<uint32_t>(box.minimum[i])==o.vu0_vf[26].u[i] && std::bit_cast<uint32_t>(box.maximum[i])==o.vu0_vf[27].u[i]);
                plainCases+=3;
            }
            // A wrong record (index 2 at record 1) rejects; the copy 2929e0 shares the sites' roles 0x900 / its own addresses.
            bool refused=false;m=R5900Context{};m.r[29].uw[0]=S7;m.r[21].uw[0]=E;m.r[2].uw[0]=read<uint32_t>(y,0x35dee8);check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x292184));
            m.r[2].uw[0]=E+0xc0;check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x29218c) && vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2921c8));
            m.r[2].uw[0]=S7+0x160;check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2921d0));
            m.r[20].uw[0]=2;m.r[18].uw[0]=B+0x80+0x10;
            try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2922a8);}catch(const std::runtime_error &){refused=true;}check(refused);
            write(y,0x2923b4,uint32_t(0));check(!vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x2923b4));
            std::printf("%u plain emitter particles (%u trail points, six store kinds, curves, three trail kinds) match the VU0 reference\n",plainCases,plainPoints);}
        // mod001 23f3f0, the general emitter's frame clock: vf3.x = dt, then per particle clock +140
        // += rate +150 * dt (.xyz) to the output's +20 and back, in both branches, against the reference.
        {const std::pair<uint32_t,uint32_t> clock[]={{0x23f47c,0x48b01800},
            {0x23f528,0xda810000},{0x23f530,0xda020000},{0x23f644,0x4bc00abc},{0x23f648,0x4bc31048},{0x23f654,0xfa010000},{0x23f65c,0xfa810000},
            {0x23f730,0xda810000},{0x23f734,0xda020000},{0x23f854,0x4bc00abc},{0x23f858,0x4bc31048},{0x23f864,0xfa010000},{0x23f86c,0xfa810000}};
        for(const auto &[pc,w]:clock){write(y,pc,w);write(x,pc,w);}
        const auto both=[&](uint32_t at,uint32_t v){write(y,at,v);write(x,at,v);};unsigned clockCases=0;
        for(unsigned seed=0;seed<64;++seed){
            uint32_t state=seed*2654435761u+3;const auto rnd=[&](){state=state*1664525u+1013904223u;return state>>8;};
            const auto value=[&](unsigned lo=105,unsigned span=40){return (rnd()&0x807fffffu)|((lo+rnd()%span)<<23);};
            const uint32_t dt=value(118,6)&0x7fffffffu;both(0x35dee8,dt);both(E+0x14,B);
            for(uint32_t at=B;at<B+0x170*10;at+=4)both(at,value());for(uint32_t at=O;at<O+0x30*8;at+=4)both(at,value());
            m=R5900Context{};m.r[4].uw[0]=E;m.r[16].uw[0]=dt;for(unsigned i=1;i<4;++i)m.r[16].uw[i]=rnd();run(0x23f47c);
            const uint32_t base=seed&1?0x23f730:0x23f528,math=seed&1?0x23f854:0x23f644;
            for(unsigned k=0;k<3;++k){const unsigned index=seed%4+2*k;const uint32_t rec=B+index*0x170;
                m.r[15].uw[0]=index;m.r[20].uw[0]=rec+0x140;m.r[16].uw[0]=rec+0x150;run(base);run(base+(seed&1?4:8));
                m.r[15].uw[0]=index+1;m.r[6].uw[0]=O+k*0x30;m.r[16].uw[0]=O+k*0x30+0x20;run(math);run(math+4);run(math+0x10);run(math+0x18);++clockCases;}
            check(!std::memcmp(x.memory.hostPointer(B,0x170*10),y.memory.hostPointer(B,0x170*10),0x170*10));
            check(!std::memcmp(x.memory.hostPointer(O,0x30*8),y.memory.hostPointer(O,0x30*8),0x30*8));
        }
        m=R5900Context{};m.r[4].uw[0]=E;m.r[16].uw[0]=read<uint32_t>(y,0x35dee8)^1;bool refused=false;
        try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x23f47c);}catch(const std::runtime_error &){refused=true;}check(refused);
        refused=false;try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x23f644);}catch(const std::runtime_error &){refused=true;}check(refused);
        m.r[16].uw[0]=read<uint32_t>(y,0x35dee8);check(vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x23f47c));
        m.r[15].uw[0]=1;m.r[20].uw[0]=B+0x140;refused=false;try{vp2s::nativeMathInstruction(y.memory.flat(),&m,&y,0x23f528);}catch(const std::runtime_error &){refused=true;}check(refused);
        std::printf("%u general emitter clock particles (both branches) match the VU0 reference\n",clockCases);}
        now.particles=vp2s::NativeParticles{};
    }
    {auto &rt=b.runtime();initialize(rt,0);now.particles=vp2s::NativeParticles{};now.assets=vp2s::NativeAssets{};
        R5900Context c{};c.pc=0x22eadc;c.r[21].uw[0]=0x180000;c.r[29].uw[0]=0x1fff000;write(rt,0x1fff0e0,uint32_t(0x200020));
        auto invoke=[&](uint32_t pc){const auto result=vp2s::nativeParticleInstruction(&c,&rt,pc);check(result.has_value());return *result;};
        auto reject=[&](uint32_t pc){const auto before=c;const auto fp=_mm_getcsr();const auto count=now.particles.outstanding();std::vector<uint8_t> ram(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{invoke(pc);}catch(const std::runtime_error &){bad=true;}check(bad && _mm_getcsr()==fp && count==now.particles.outstanding() && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));};
        reject(0x22ee88);c.pc=0x170000;reject(0x22eae0);c.pc=0x22eadc;
        write(rt,0x180aff,uint8_t(5));reject(0x22eae0);write(rt,0x180aff,uint8_t(3));   // mode 5 (VU1 image 8) is unported; 2 is produced as 4
        write(rt,0x180aa4,uint32_t(0x200000));reject(0x22eae0);write(rt,0x180aa4,uint32_t(0x220000));
        write(rt,0x1fff0e0,uint32_t(0x200030));reject(0x22eae0);write(rt,0x1fff0e0,uint32_t(0x200020));
        check(invoke(0x22eae0));reject(0x22eae0);reject(0x22f0a0);
        {const auto context=c;const auto frame=c.r[29].uw[0],fp=_mm_getcsr();const std::array<float,4> earlier{10,20,30,0};
            now.particles.includePoint(frame,0x190000,earlier);bool missing=false;
            try{now.particles.bounds(frame,0x190000);}catch(const std::runtime_error &){missing=true;}check(missing && _mm_getcsr()==fp);
            const std::array<uint32_t,4> minimum{0x7f800000,0x7f800000,0x7f800000,0x7fa01234},maximum{0xff800000,0xff800000,0xff800000,0x3f000000};
            write(rt,0x35dbc0,minimum);write(rt,0x35dbe0,maximum);c.r[3].uw[0]=0x35dbc0;c.r[2].uw[0]=0x35dbe0;
            reject(0x22eca0);check(invoke(0x22ec9c));reject(0x22ec9c);check(invoke(0x22eca0));
            now.particles.includePoint(frame,0x190000,{std::bit_cast<float>(0x80000000u),1,2,0});now.particles.includePoint(frame,0x190000,{0,0,3,0});
            const auto box=now.particles.bounds(frame,0x190000);check(std::bit_cast<uint32_t>(box.minimum[0])==0x80000000 && std::bit_cast<uint32_t>(box.maximum[0])==0x80000000);
            check(box.minimum[1]==0 && box.maximum[1]==1 && box.minimum[2]==2 && box.maximum[2]==3 && std::bit_cast<uint32_t>(box.minimum[3])==0x7fa01234 && box.maximum[3]==.5f);
            bool invalid=false;try{now.particles.includePoint(frame,0x190000,{5,std::bit_cast<float>(0x7fc01234u),6,0});}catch(const std::runtime_error &){invalid=true;}
            const auto unchanged=now.particles.bounds(frame,0x190000);check(invalid && !std::memcmp(&box,&unchanged,sizeof(box)) && _mm_getcsr()==fp);c=context;
        }
        c.r[16].uw[0]=0x70000000;c.r[2].uw[0]=3;c.r[18].uw[0]=1;c.r[5].uw[0]=11;c.r[4].uw[0]=176;c.r[3].uw[0]=0x200020;c.r[19].uw[0]=1;
        for(unsigned i=0;i<44;++i)write(rt,0x70000000+i*4,uint32_t(0x12340000+i));
        write(rt,0x70000010,uint32_t(3));write(rt,0x70000014,uint32_t(11));write(rt,0x70000018,uint32_t(1));
        reject(0x22ee94);c.r[16].uw[0]+=0xa00;reject(0x22ee88);c.r[16].uw[0]-=0xa00;
        c.r[19].uw[0]=2;reject(0x22ee88);c.r[19].uw[0]=1;c.r[5].uw[0]=10;reject(0x22ee88);c.r[5].uw[0]=11;
        write(rt,0x180aa4,uint32_t(0x220010));reject(0x22ee88);write(rt,0x180aa4,uint32_t(0x220000));
        write(rt,0x70000010,uint32_t(4));reject(0x22ee88);write(rt,0x70000010,uint32_t(3));
        now.assets.discRead(0x200000,1,99);check(now.assets.origin(0x200020).has_value());check(invoke(0x22ee88));
        check(!now.assets.origin(0x200020).has_value() && now.assets.origin(0x200000).has_value());
        check(!std::memcmp(rt.memory.hostPointer(0x200020,176),rt.memory.hostPointer(0x70000000,176),176));
        reject(0x22ee88);reject(0x22ee9c);
        for(uint32_t pc:{0x22ee94u,0x22ee9cu,0x22eea4u})check(invoke(pc));reject(0x22eebc);
        write(rt,0x1fff0e0,uint32_t(0x2000d0));check(invoke(0x22eebc));reject(0x22eebc);
        write(rt,0x180ac0,uint32_t(1));write(rt,0x180abc,uint32_t(3));write(rt,0x190004,uint8_t(1));
        c.pc=0x22efc8;c.r[2].uw[0]=0x180a60;write(rt,0x1fff0f0,.25f);
        c.r[2].uw[0]+=16;reject(0x22f018);c.r[2].uw[0]-=16;
        // An exponent-255 padding is a PS2 number now (user 2026-11-24, "PS2 float rules"): no longer a refusal case.
        c.pc=0x170000;reject(0x22f018);c.pc=0x22efc8;
        check(invoke(0x22f018));reject(0x22f018);
        check(read<uint32_t>(rt,0x180a4c)==0x7fa01234);
        check(read<float>(rt,0x180a64)==.5f && read<float>(rt,0x180a68)==2.5f);
        check(read<float>(rt,0x180a6c)==float(std::sqrt(1.1875)));
        write(rt,0x180af9,uint8_t(1));
        check(!invoke(0x22f0a0) && now.particles.outstanding()==0);reject(0x22ee88);
        // Reusing the same frame with a fresh buffer selection is valid.
        c.pc=0x22eadc;c.r[30].uw[0]=1;write(rt,0x1fff0e0,uint32_t(0x220020));check(invoke(0x22eae0));
        now.particles=vp2s::NativeParticles{};
        c.r[30].uw[0]=0;write(rt,0x180af9,uint8_t(0));write(rt,0x180aa4,uint32_t(0x200040));write(rt,0x1fff0e0,uint32_t(0x200020));
        check(invoke(0x22eae0));c.r[3].uw[0]=0x200020;reject(0x22ee88);now.particles=vp2s::NativeParticles{};
    }
    std::printf("%u complete particle producers,%u chunks,%u points; full CPU/RAM/FP, both buffers, scratch/RAM frames, four modes/FTZ/DAZ; no device or VU execution\n",callers,chunks,points);
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

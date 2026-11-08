#include "native_particles.h"
#include "native_float_oracle.h"
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
void initialize(Runtime &rt,unsigned index){
    constexpr uint32_t object=0x180000,emitter=0x190000,table=0x1a0000;
    std::memset(rt.memory.flat(),0x51,bozo::kFlatBytes);std::memset(rt.memory.hostPointer(object,0xb10),0,0xb10);
    std::memset(rt.memory.hostPointer(emitter,0x100),0,0x100);std::memset(rt.memory.hostPointer(table,0x200),0,0x200);
    write(rt,object,uint32_t(0x154180));write(rt,0x1541e4,uint32_t(0x172010));
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
        auto &x=a.runtime(),&y=b.runtime();initialize(x,seed&1);
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
        auto &x=a.runtime(),&y=b.runtime();initialize(x,seed&1);old.ready=now.ready=seed%4!=0;old.dirty=now.dirty=seed%4!=1;
        if(seed%4==2)write(x,0x180a90,uint32_t(0));if(seed%4==3)write(x,0x35ded8,0.f);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);R5900Context p{};p.r[4].uw[0]=0x180000;p.r[29].uw[0]=0x1fff000;p.r[31].uw[0]=0x170000;auto q=p;
        old.publications=now.publications=0;x.functions.generated(0x22e650)(x.memory.flat(),&p,&x);y.functions.generated(0x22e650)(y.memory.flat(),&q,&y);
        p.cycleLimit=q.cycleLimit;check(!std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        check(!now.publications && !old.publications && now.particles.outstanding()==0);
    }
    {auto &rt=b.runtime();initialize(rt,0);now.particles=vp2s::NativeParticles{};now.assets=vp2s::NativeAssets{};
        R5900Context c{};c.pc=0x22eadc;c.r[21].uw[0]=0x180000;c.r[29].uw[0]=0x1fff000;write(rt,0x1fff0e0,uint32_t(0x200020));
        auto invoke=[&](uint32_t pc){const auto result=vp2s::nativeParticleInstruction(&c,&rt,pc);check(result.has_value());return *result;};
        auto reject=[&](uint32_t pc){const auto before=c;const auto fp=_mm_getcsr();const auto count=now.particles.outstanding();std::vector<uint8_t> ram(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{invoke(pc);}catch(const std::runtime_error &){bad=true;}check(bad && _mm_getcsr()==fp && count==now.particles.outstanding() && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));};
        reject(0x22ee88);c.pc=0x170000;reject(0x22eae0);c.pc=0x22eadc;
        write(rt,0x180aff,uint8_t(2));reject(0x22eae0);write(rt,0x180aff,uint8_t(3));
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
        write(rt,0x1fff0f0,std::bit_cast<float>(0x7fa01234u));reject(0x22f018);write(rt,0x1fff0f0,.25f);
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

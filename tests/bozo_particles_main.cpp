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
        now.particles=vp2s::NativeParticles{};
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

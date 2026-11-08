#include "native_math.h"
#include "native_particles.h"
#include "native_float_oracle.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "particle_physics_original_functions.h"
#include "particle_physics_adapted_functions.h"
#include <bit>
#include <cstdio>
#include <cstring>
#include <set>
#include <source_location>
#include <stdexcept>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("particle physics assertion at "+std::to_string(at.line()));}
template<class T>T read(Runtime &rt,uint32_t at){T v;auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(&v,p,sizeof(v));return v;}
template<class T>void write(Runtime &rt,uint32_t at,T v){auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(p,&v,sizeof(v));}
using Record=std::array<uint32_t,4>;
uint32_t sum(uint32_t a,uint32_t b){return vp2s::test::sum(a,b);}
uint32_t product(uint32_t a,uint32_t b){return vp2s::test::product(a,b);}
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::ParticleOwner,vp2s::AssetOwner,vp2s::ParticleUpdateOwner {
    bool reference=false;vp2s::NativeParticles particles;vp2s::NativeAssets assets;vp2s::NativeParticleUpdates updates;
    vp2s::ParticleBounds expectedBounds;std::set<uint32_t> sites;unsigned publications=0,segments=0,callbacks=0;
    struct Point {uint32_t source,output,time;Record position,rate,attributes,change,drift;};std::map<uint32_t,Point> points;
    vp2s::NativeParticles &nativeParticles()override{return particles;}vp2s::NativeAssets &nativeAssets()override{return assets;}
    vp2s::NativeParticleUpdates &nativeParticleUpdates()override{return updates;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        sites.insert(pc);if(pc==0x40ab5c)++publications;if(pc==0x40aaec)++segments;
        if(!reference){const auto handled=vp2s::nativeMathInstruction(r,c,rt,pc);check(handled || pc==0x40b040);return handled;}
        if(pc>=0x410640 && pc<0x4107c4){
            if(pc==0x410684 || pc==0x4106cc || pc==0x410714){const bool position=pc==0x410684;const auto emitter=c->r[4].uw[0];
                const auto matrix=read<std::array<uint32_t,16>>(*rt,emitter+0x80);const auto input=read<Record>(*rt,c->r[position?5:pc==0x4106cc?6:7].uw[0]);Record out;
                for(unsigned i=0;i<4;++i){out[i]=product(matrix[i],input[0]);for(unsigned j=1;j<(position?4u:3u);++j)out[i]=sum(out[i],product(matrix[j*4+i],j==3?0x3f800000:input[j]));}write(*rt,c->r[2].uw[0],out);
            }return true;
        }
        const auto frame=c->r[29].uw[0],emitter=c->r[21].uw[0];
        if(pc==0x40b040){check(points.empty());return false;}
        if(pc==0x40aa30){const auto source=c->r[6].uw[0];points.emplace(frame,Point{source,c->r[20].uw[0],read<uint32_t>(*rt,0x35dee8),read<Record>(*rt,source+0x10),read<Record>(*rt,source+0x30),read<Record>(*rt,source+0x50),read<Record>(*rt,source+0x40),read<Record>(*rt,emitter+0xc0)});return true;}
        if(pc==0x40aaec){auto &p=points.at(frame);const auto curve=read<Record>(*rt,read<uint32_t>(*rt,emitter+0x68)+c->r[5].uw[0]*16);const auto duration=std::bit_cast<uint32_t>(c->f[1]);
            for(unsigned i=0;i<3;++i)p.position[i]=sum(sum(p.position[i],0),product(curve[i],duration));return true;}
        if(pc==0x40ab5c){auto &p=points.at(frame);const auto prior=p;
            for(unsigned i=0;i<3;++i){p.position[i]=sum(sum(sum(prior.position[i],0),product(prior.drift[i],0x3f800000)),product(prior.rate[i],p.time));
                p.attributes[i]=sum(sum(prior.attributes[i],0),product(prior.rate[i],p.time));p.rate[i]=sum(sum(prior.rate[i],0),product(prior.change[i],p.time));
                const float value=std::bit_cast<float>(p.position[i]);if(value<expectedBounds.minimum[i])expectedBounds.minimum[i]=value;if(value>expectedBounds.maximum[i])expectedBounds.maximum[i]=value;}
            p.rate[3]=sum(prior.rate[3],p.time^0x80000000u);p.attributes[3]=sum(prior.attributes[3],p.time);write(*rt,p.output,p.position);return true;}
        const bool attribute=pc==0x40abb8 || pc==0x40ac2c || pc==0x40ace0 || pc==0x40ad58 || pc==0x40adf0;
        const bool position=pc==0x40abbc || pc==0x40ac34 || pc==0x40ace4 || pc==0x40ad5c || pc==0x40adf4;
        const bool rate=pc==0x40abc0 || pc==0x40ac38 || pc==0x40ace8 || pc==0x40ad60 || pc==0x40adf8;
        if(attribute || position || rate){const auto &p=points.at(frame);write(*rt,p.source+(attribute?0x50:position?0x10:0x30),attribute?p.attributes:position?p.position:p.rate);if(rate)points.erase(frame);return true;}
        if(pc==0x40ac4c){auto value=read<Record>(*rt,c->r[8].uw[0]);for(unsigned i=0;i<3;++i)value[i]=product(value[i],value[i]);value[0]=sum(sum(value[0],value[1]),product(0x3f800000,value[2]));write(*rt,frame+0x140,value);return true;}
        if(pc==0x40acec){const auto matrix=read<std::array<uint32_t,16>>(*rt,emitter+0x80);auto point=read<Record>(*rt,c->r[17].uw[0]);point[3]=0x3f800000;Record out;
            for(unsigned i=0;i<4;++i){out[i]=product(matrix[i],point[0]);for(unsigned j=1;j<4;++j)out[i]=sum(out[i],product(matrix[j*4+i],point[j]));}write(*rt,frame+0x130,out);return true;}
        const std::set<uint32_t> retired{0x40a950,0x40a958,0x40aa38,0x40aa40,0x40aa48,0x40aad8,0x40aae4,0x40aae8,
            0x40ab24,0x40ab28,0x40ab2c,0x40ab30,0x40ab34,0x40ab38,0x40ab3c,0x40ab40,0x40ab44,0x40ab54,0x40ab58,
            0x40ac28,0x40ac30,0x40ac3c,0x40ac40,0x40acbc,0x40acc0,0x40acc4,0x40acc8,0x40accc,0x40acd0,0x40acd4,0x40acd8};
        check(retired.contains(pc));return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        check(pc==0x2262b0);auto &rt=d.runtime();const auto stack=c->r[29].uw[0],emitter=c->r[21].uw[0],point=c->r[17].uw[0]-0x10;
        check(c->r[4].uw[0]==0x270000 && c->r[5].uw[0]==read<uint8_t>(rt,emitter+0xd3) && c->r[31].uw[0]==0x410790);
        const auto flags=read<uint8_t>(rt,emitter+0xd2);const auto matrix=read<std::array<uint32_t,16>>(rt,emitter+0x80);
        uint32_t digest=read<uint32_t>(rt,0x270100);
        for(unsigned j=0;j<3;++j){const auto pointer=c->r[6+j].uw[0];check(pointer==(flags&(1<<j)?stack+0x70-j*16:0));
            if(pointer){const auto input=read<Record>(rt,point+(j==0?0x10:j==1?0x30:0x40));Record expected;
                for(unsigned i=0;i<4;++i){expected[i]=product(matrix[i],input[0]);for(unsigned row=1;row<(j==0?4u:3u);++row)expected[i]=sum(expected[i],product(matrix[row*4+i],row==3?0x3f800000:input[row]));digest^=expected[i];}
                const auto actual=read<Record>(rt,pointer);check(actual==expected);}
        }
        check(!c->r[9].uw[0] && !c->r[10].uw[0] && !c->r[11].uw[0]);
        for(unsigned i=0;i<4;++i)check(read<uint64_t>(rt,stack+i*8)==0);
        // The retired numeric spills have no semantic consumer. Neither path
        // writes them, and the bounded callback cannot depend on their bytes.
        for(unsigned i=0;i<32;++i)check(read<uint8_t>(rt,stack+0x30+i)==0x51);
        write(rt,0x270100,digest);++callbacks;c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
void prepareBounds(Host &host,Runtime &rt,uint32_t parent,unsigned selector){
    host.particles=vp2s::NativeParticles{};host.updates=vp2s::NativeParticleUpdates{};host.points.clear();
    constexpr uint32_t object=0x180000,emitter=0x190000;
    std::memset(rt.memory.hostPointer(object,0xb10),0,0xb10);write(rt,object,uint32_t(0x154180));write(rt,object+0xa90,emitter);
    write(rt,object+0xaa0,uint32_t(0x220000));write(rt,object+0xaa4,uint32_t(0x240000));write(rt,object+0xaf9,uint8_t(selector));write(rt,object+0xaff,uint8_t(3));
    write(rt,parent+0xe0,uint32_t((selector?0x240000:0x220000)+0x20));
    Record minimum{0x7f800000,0x7f800000,0x7f800000,0x3f800000},maximum{0xff800000,0xff800000,0xff800000,0x3f000000};write(rt,0x35dbc0,minimum);write(rt,0x35dbe0,maximum);
    std::memcpy(host.expectedBounds.minimum.data(),minimum.data(),16);std::memcpy(host.expectedBounds.maximum.data(),maximum.data(),16);
    R5900Context c{};c.pc=0x22eadc;c.r[21].uw[0]=object;c.r[29].uw[0]=parent;c.r[30].uw[0]=selector;
    check(vp2s::nativeParticleInstruction(&c,&rt,0x22eae0)==true);c.r[3].uw[0]=0x35dbc0;c.r[2].uw[0]=0x35dbe0;
    check(vp2s::nativeParticleInstruction(&c,&rt,0x22ec9c)==true);check(vp2s::nativeParticleInstruction(&c,&rt,0x22eca0)==true);
}
}
int main(){try{
    Host old,now;old.reference=true;bozo::ee::Dispatcher a(old),b(now);
    a.setFunctions(particle_physics_original_function_table,particle_physics_original_function_count,particle_physics_original_resume_table,particle_physics_original_resume_count);
    b.setFunctions(particle_physics_adapted_function_table,particle_physics_adapted_function_count,particle_physics_adapted_resume_table,particle_physics_adapted_resume_count);
    a.runtime().port=&old;b.runtime().port=&now;for(auto *d:{&a,&b})for(uint32_t pc:{0x2262b0u,0x4107d0u})d->bindLibrary(pc);
    const auto saved=_mm_getcsr();uint32_t random=0x40a8b0;unsigned callers=0,published=0,segments=0,callbacks=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x51,bozo::kFlatBytes);
        constexpr uint32_t emitter=0x190000,pool=0x200000,maskOwner=0x1a0000,mask=0x1b0000,curve=0x260000;
        std::memset(x.memory.hostPointer(emitter,0xe0),0,0xe0);write(x,emitter,uint32_t(0x162ff0));write(x,emitter+0xc,uint32_t(130));write(x,emitter+0x10,uint32_t(1));write(x,emitter+0x14,pool);
        write(x,emitter+0x1c,maskOwner);write(x,maskOwner+8,mask);const uint32_t starts[]={0,63,64,127};write(x,emitter+0x48,starts[(seed/4)%4]);
        const float steps[]={0,.016666668f,.035f,.1f};write(x,0x35dee8,steps[seed%4]);write(x,emitter+0x4c,float(seed%4)*.25f);write(x,emitter+0x54,1.f);write(x,emitter+0x64,1.f);
        write(x,emitter+0x68,uint32_t(seed&4?curve:0));const uint8_t modes[]={0,1,2,5,6};write(x,emitter+0xd0,modes[(seed/8)%5]);write(x,0x35dbf0,uint32_t(0));
        const std::array<float,16> matrix{1,.25f,0,0,0,1,0,.125f,0,0,1,0,.25f,-.5f,.75f,1};write(x,emitter+0x80,matrix);
        write(x,emitter+0x6c,uint32_t(seed&32?0x270000:0));write(x,emitter+0xd2,uint8_t(seed&7));write(x,emitter+0xd3,uint8_t(seed));
        write(x,0x270000,uint32_t(0x280000));write(x,0x280098,uint32_t(0x2262b0));write(x,0x270100,uint32_t(0));
        const std::array<float,4> drift{.01f,-.02f,.03f,0};write(x,emitter+0xc0,drift);
        for(unsigned i=0;i<80;++i){const std::array<float,4> value{float(int(i%7)-3)*.01f,float(i%5)*.005f,-.015f,0};write(x,curve+i*16,value);}
        for(unsigned i=0;i<3;++i){random=random*1664525u+1013904223u;const uint64_t flags=seed%8==0?0:seed%8==1?UINT64_MAX:(uint64_t(random)<<32)|(random^0x97ab5312u);write(x,mask+i*8,flags);}
        for(unsigned i=0;i<130;++i){const auto at=pool+i*0x90;std::array<float,4> position{float(int(i%7)-3)*.125f,float(i%11)*.05f,-.25f,std::bit_cast<float>(0x7fc01234u)};
            const std::array<float,4> rate{.1f,-.05f,.025f,float(seed%5)*.05f},attributes{.5f,.75f,.25f,float(seed%6)*.025f},change{.01f,.02f,-.03f,std::bit_cast<float>(0xffc05678u)};
            write(x,at+0x10,position);write(x,at+0x20,Record{17,34,51,128});write(x,at+0x30,rate);write(x,at+0x40,change);write(x,at+0x50,attributes);
        }
        const auto parent=seed&1?0x70003d00u:0x1fff000u;prepareBounds(old,x,parent,seed&1);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);prepareBounds(now,y,parent,seed&1);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];p.f[i]=float(i)/8;}
        std::memset(p.vu0_vf,0x49,sizeof(p.vu0_vf));std::memset(&p.vu0_acc,0x5a,sizeof(p.vu0_acc));p.vu0_status=0x1234;
        p.r[4].ud[0]=emitter;p.r[5].ud[0]=seed%8;p.r[6].ud[0]=0x70000020;p.r[7].ud[0]=0;p.r[29].ud[0]=parent;p.r[31].ud[0]=0x22ed38;auto q=p;
        old.publications=now.publications=old.segments=now.segments=old.callbacks=now.callbacks=0;const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&1?0x8040:0);_mm_setcsr(control);
        x.functions.generated(0x40a8b0)(x.memory.flat(),&p,&x);const auto fp=_mm_getcsr();_mm_setcsr(control);
        try{y.functions.generated(0x40a8b0)(y.memory.flat(),&q,&y);}catch(const std::exception &e){std::fprintf(stderr,"particle physics seed=%u mode=%x: %s\n",seed,mode,e.what());throw;}
        check(_mm_getcsr()==fp);_mm_setcsr(saved);p.cycleLimit=q.cycleLimit;
        check(p.pc==0x22ed38 && !std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        const auto &actualBounds=now.particles.bounds(parent,emitter);check(!std::memcmp(&old.expectedBounds,&actualBounds,sizeof(actualBounds)));
        check(now.updates.outstanding()==0 && old.points.empty() && old.publications==now.publications && old.segments==now.segments && old.callbacks==now.callbacks);
        published+=now.publications;segments+=now.segments;callbacks+=now.callbacks;++callers;
    }
    check(old.sites==now.sites && published && segments && callbacks);
    for(uint32_t pc:{0x41074cu,0x410750u,0x410790u,0x410794u,0x4107a8u,0x4107b0u,0x4107b4u})check(now.sites.contains(pc));
    {auto &rt=b.runtime();constexpr uint32_t parent=0x1ffe000,frame=parent-0x150,emitter=0x190000,point=0x200000,output=0x70000020;
        prepareBounds(now,rt,parent,0);write(rt,emitter+0xc,uint32_t(1));write(rt,emitter+0x10,uint32_t(1));write(rt,emitter+0x14,point);write(rt,emitter+0x68,uint32_t(0x260000));write(rt,emitter+0xd0,uint8_t(0));write(rt,0x35dee8,.25f);
        const std::array<float,4> position{1,2,3,std::bit_cast<float>(0x7fc01234u)},rate{.1f,.2f,.3f,1},attributes{.5f,.5f,.5f,.125f},change{.01f,-.01f,.02f,0},drift{.01f,.02f,.03f,0};
        write(rt,point+0x10,position);write(rt,point+0x30,rate);write(rt,point+0x50,attributes);write(rt,point+0x40,change);write(rt,emitter+0xc0,drift);write(rt,0x260000,drift);
        write(rt,frame+0xa0,uint32_t(0x22ed38));R5900Context c{};c.pc=0x40a8b0;c.r[21].uw[0]=emitter;c.r[29].uw[0]=frame;c.r[18].uw[0]=1;c.r[19].uw[0]=0;c.r[6].uw[0]=point;c.r[17].uw[0]=point+0x10;c.r[20].uw[0]=output;
        auto invoke=[&](uint32_t pc){return vp2s::nativeMathInstruction(rt.memory.flat(),&c,&rt,pc);};
        auto reject=[&](uint32_t pc){const auto before=c;const auto fp=_mm_getcsr();const auto count=now.updates.outstanding();const auto box=now.particles.bounds(parent,emitter);
            std::vector<uint8_t> ram(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool bad=false;try{invoke(pc);}catch(const std::runtime_error &){bad=true;}
            check(bad && _mm_getcsr()==fp && now.updates.outstanding()==count && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));
            const auto after=now.particles.bounds(parent,emitter);check(!std::memcmp(&box,&after,sizeof(box)));};
        reject(0x40ab5c);write(rt,frame+0xa0,uint32_t(0x170000));reject(0x40aa30);write(rt,frame+0xa0,uint32_t(0x22ed38));
        c.r[6].uw[0]+=16;reject(0x40aa30);c.r[6].uw[0]-=16;c.r[19].uw[0]=1;reject(0x40aa30);c.r[19].uw[0]=0;
        c.r[20].uw[0]=point;reject(0x40aa30);c.r[20].uw[0]=output;
        for(uint32_t at:{point+0x10,point+0x30,point+0x3c,point+0x5c,point+0x40,emitter+0xc0,0x35dee8u})for(uint32_t bad:{1u,0x7fc01234u,0x7f7fffffu}){const auto previous=read<uint32_t>(rt,at);write(rt,at,bad);reject(0x40aa30);write(rt,at,previous);}
        check(invoke(0x40aa30));reject(0x40aa30);reject(0x40b040);
        c.r[3].uw[0]=0x260010;c.r[5].uw[0]=0;c.f[1]=.01f;reject(0x40aaec);c.r[3].uw[0]=0x260000;c.r[5].uw[0]=65536;reject(0x40aaec);c.r[5].uw[0]=0;
        for(uint32_t bad:{1u,0x7fc01234u,0x7f7fffffu}){c.f[1]=std::bit_cast<float>(bad);reject(0x40aaec);}c.f[1]=.01f;check(invoke(0x40aaec));
        c.r[20].uw[0]+=16;reject(0x40ab5c);c.r[20].uw[0]-=16;check(invoke(0x40ab5c));reject(0x40ab5c);
        c.r[8].uw[0]=point+0x50;c.r[16].uw[0]=point+0x30;reject(0x40adf4);c.r[8].uw[0]+=16;reject(0x40adf0);c.r[8].uw[0]-=16;
        check(invoke(0x40adf0));check(invoke(0x40adf4));check(invoke(0x40adf8));reject(0x40adf8);
        check(read<uint32_t>(rt,point+0x1c)==0x7fc01234 && read<uint32_t>(rt,output+12)==0x7fc01234);
        {const auto outer=c;const auto helper=frame-0x80;
            write(rt,emitter+0x6c,uint32_t(0x270000));write(rt,emitter+0xd2,uint8_t(0));write(rt,0x270000,uint32_t(0x280000));write(rt,0x280098,uint32_t(0x2262b0));write(rt,helper+0x20,uint32_t(0x40ac20));
            c.pc=0x410640;c.r[29].uw[0]=helper;c.r[4].uw[0]=emitter;c.r[2].uw[0]=helper+0x30;c.r[3].uw[0]=c.r[5].uw[0]=c.r[8].uw[0]=c.r[9].uw[0]=c.r[10].uw[0]=0;
            c.r[6].uw[0]=point+0x30;c.r[7].uw[0]=point+0x40;c.r[31].uw[0]=0x40ac20;
            reject(0x410750);write(rt,0x280098,uint32_t(0x2262b4));reject(0x41074c);write(rt,0x280098,uint32_t(0x2262b0));
            c.r[31].uw[0]=0x170000;reject(0x41074c);c.r[31].uw[0]=0x40ac20;c.r[6].uw[0]+=16;reject(0x41074c);c.r[6].uw[0]-=16;
            check(invoke(0x41074c));reject(0x41074c);reject(0x410790);
            {const auto helperContext=c;c=outer;reject(0x40aa30);reject(0x40b040);c=helperContext;}
            check(invoke(0x410750));
            c.pc=0x410790;c.r[31].uw[0]=0x170000;reject(0x410790);c.r[31].uw[0]=0x410790;check(invoke(0x410790));check(invoke(0x410794));
            c.r[3].uw[0]=0;reject(0x4107a8);c.r[3].uw[0]=read<uint32_t>(rt,0x35dee8);check(invoke(0x4107a8));
            c.r[3].uw[0]=helper+0x40;reject(0x4107b0);c.r[3].uw[0]=helper+0x30;check(invoke(0x4107b0));check(invoke(0x4107b4));reject(0x4107b4);c=outer;
        }
        check(now.updates.outstanding()==1 && !invoke(0x40b040) && now.updates.outstanding()==0);
    }
    std::printf("%u complete bounded particle physics callers,%u points,%u curve segments,%u object-update callbacks,%zu sites; full CPU/RAM/FP and enclosing bounds, integer oracle/four modes/FTZ/DAZ\n",callers,published,segments,callbacks,now.sites.size());return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

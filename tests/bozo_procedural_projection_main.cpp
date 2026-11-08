#include "native_procedural.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "projection_functions.h"
#include "native_float_oracle.h"
#include <bit>
#include <cstdio>
#include <cstring>
#include <immintrin.h>
#include <source_location>
#include <vector>
#include <set>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("procedural projection assertion at "+std::to_string(at.line()));}
template<class T>void put(Runtime &rt,uint32_t at,const T &v){auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(p,&v,sizeof(v));}
template<class T>T get(Runtime &rt,uint32_t at){T v;auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(&v,p,sizeof(v));return v;}
std::array<uint32_t,4> oracle(const std::array<float,16> &matrix,const std::array<uint32_t,4> &p){
    struct Scope{unsigned fp=_mm_getcsr();~Scope(){_mm_setcsr(fp);}} scope;_mm_setcsr((scope.fp&~_MM_ROUND_MASK)|_MM_ROUND_TOWARD_ZERO);
    std::array<uint32_t,4> out;for(unsigned row=0;row<4;++row){uint32_t sum=vp2s::test::product(std::bit_cast<uint32_t>(matrix[4*row]),p[0]);
        for(unsigned i=1;i<3;++i)sum=vp2s::test::sum(sum,vp2s::test::product(std::bit_cast<uint32_t>(matrix[4*row+i]),p[i]));
        out[row]=vp2s::test::sum(sum,vp2s::test::product(std::bit_cast<uint32_t>(matrix[4*row+3]),0x3f800000));}
    const auto q=vp2s::test::quotient(0x3f800000,out[3]);for(auto &v:out)v=vp2s::test::product(v,q);return out;
}
int32_t fixedOracle(uint32_t b){const auto m=b&0x7fffffff;const int exponent=int((m>>23)&255)-127+4;
    if(!m || exponent<0)return 0;if(exponent>=31)return b>>31?INT32_MIN:INT32_MAX;
    const uint32_t mantissa=(m&0x7fffff)|0x800000,integer=exponent>=23?mantissa<<(exponent-23):mantissa>>(23-exponent);
    return b>>31?-int32_t(integer):int32_t(integer);
}
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::ProceduralOwner,vp2s::SceneOwner{
    vp2s::NativeProceduralTransforms projections;vp2s::NativeScene scene;
    bool reference=false;std::set<uint32_t> sites;std::array<uint32_t,4> first{},second{},fixed{};unsigned commits=0;
    vp2s::NativeProceduralTransforms &nativeProceduralTransforms()override{return projections;}
    vp2s::NativeScene &nativeScene()override{return scene;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc)override{
        sites.insert(pc);if(reference && pc>=0x2478c8 && pc<=0x247a1c){
            const auto &matrix=projections.projection(c->r[29].uw[0]+0xb0,c->r[21].uw[0]).screenFromLocal;
            if(pc==0x2478c8)first=oracle(matrix,get<std::array<uint32_t,4>>(*rt,c->r[9].uw[0]));
            else if(pc==0x247968)second=get<std::array<uint32_t,4>>(*rt,c->r[9].uw[0]);
            else if(pc==0x24796c)put(*rt,c->r[4].uw[0],first);
            else if(pc==0x247974){const auto projected=oracle(matrix,second);for(unsigned i=0;i<4;++i)fixed[i]=uint32_t(fixedOracle(projected[i]));}
            else if(pc==0x247a18)c->r[9].ud[0]=0;
            else if(pc==0x247a1c)put(*rt,c->r[3].uw[0],fixed);return true;
        }
        if(auto answer=vp2s::nativeProceduralInstruction(c,rt,pc))return *answer;throw std::runtime_error("unexpected native site");}
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        check(c->r[4].uw[0]==0x1db000);
        if(pc==0x120fa0){check(c->r[5].uw[0]==0);c->r[2].ud[0]=get<uint32_t>(d.runtime(),0x1db008);}
        else {check(pc==0x120f70);put(d.runtime(),0x1db008,c->r[5].uw[0]);++commits;}
        c->pc=c->r[31].uw[0];}
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
void pointCases(){
    Host old,now;old.reference=true;bozo::ee::Dispatcher a(old),b(now);
    for(auto *d:{&a,&b}){d->runtime().port=d==&a?&old:&now;d->setFunctions(projection_function_table,projection_function_count,projection_resume_table,projection_resume_count);d->boot(0x2476b0,0x1fff000);d->bindLibrary(0x120fa0);d->bindLibrary(0x120f70);}
    const auto saved=_mm_getcsr();unsigned pairs=0,known=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        const uint32_t aliases[]={0,0x20000000,0x30000000,0x80000000,0xa0000000},parent=seed&1?0x70003d70:0x1ffe100|aliases[(seed/8)%5],prepare=parent-0x170,frame=parent-0xb0;
        const unsigned count=seed%5;const uint32_t first=0x1d8000|aliases[(seed/16)%5],second=seed%8==1 && count==1?frame+0x90:0x1d9000|aliases[(seed/32)%5];
        std::array<float,16> matrix{1,0,0,2048,0,1,0,2048,0,0,1,seed&4?1000.f:5000.f,0,0,0,seed&2?3.f:1.f};
        if(seed%8==0){matrix={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};++known;}else {matrix[1]=.125f;matrix[6]=-.25f;matrix[8]=.5f;}
        if(seed%8==2)matrix[0]=matrix[5]=4;
        if(seed%8==3){matrix={.5f,0,0,0,0,.5f,0,0,0,0,1,0,0,0,0,1};}
        for(auto *d:{&a,&b}){auto &rt=d->runtime();auto &h=d==&a?old:now;std::memset(rt.memory.flat(),0x51,bozo::kFlatBytes);h.scene=vp2s::NativeScene{};h.projections=vp2s::NativeProceduralTransforms{};h.commits=0;cameraInput(rt,h);
            put(rt,prepare+0x20,uint32_t(0x202bc8));std::array<float,16> transposed;for(unsigned row=0;row<4;++row)for(unsigned col=0;col<4;++col)transposed[col*4+row]=matrix[row*4+col];put(rt,prepare+0x130,transposed);
            R5900Context pub{};pub.pc=0x2481d0;pub.r[31].uw[0]=0x2481d0;pub.r[29].uw[0]=prepare;pub.r[4].uw[0]=prepare+0x130;pub.r[16].uw[0]=camera;pub.r[17].uw[0]=object;h.projections.publish(&rt,&pub,h.scene);
            put(rt,object+0x230,uint32_t(0x1db000));put(rt,0x1db008,uint32_t(0x1e0000));put(rt,0x35ddf8,uint32_t(0x1c0000));put(rt,0x1c4088,uint32_t(0x1b0000));
            put(rt,0x1b3c6c,int16_t(640));put(rt,0x1b3c6e,int16_t(448));put(rt,object+0x238,uint32_t(0));put(rt,object+0x23c,uint32_t(65535));put(rt,object+0x24c,1.f);put(rt,object+0x250,1.f);
            put(rt,0x35d988,1.f);put(rt,0x35d990,1.f);put(rt,0x35d998,uint8_t(seed&16?1:0));
            for(unsigned i=0;i<count;++i){std::array<float,4> p{float(int(seed%11)-5)+float(i)/8,float(i)-2.125f,float(seed%7)+.125f,std::bit_cast<float>(0x7fa01234u)};
                if(seed%8==3){p[0]=std::bit_cast<float>(0x00800000u);p[1]=std::bit_cast<float>(0x80800000u);}
                auto q=p;q[0]+=.375f;q[1]-=.125f;q[3]=std::bit_cast<float>(1u);if(seed%8==2){q[0]=8.e7f;q[1]=-8.e7f;}
                put(rt,first+16*i,p);put(rt,second+16*i,q);put(rt,0x1da000+4*i,uint32_t(0x80604020+seed));}
        }
        R5900Context x{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x1234000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];x.f[i]=float(i)/8;}
        x.r[4].ud[0]=object;x.r[5].ud[0]=count;x.r[6].ud[0]=first;x.r[7].ud[0]=second;x.r[8].ud[0]=0x1da000;x.r[9].ud[0]=seed&4?1:0;x.r[10].ud[0]=seed&8?1:0;x.r[29].ud[0]=parent;x.r[31].ud[0]=0x170000;
        std::memset(x.vu0_vf,0xa5,sizeof(x.vu0_vf));std::memset(&x.vu0_acc,0x5a,sizeof(x.vu0_acc));x.vu0_status=0x12345678;x.vu0_q_bits=0x87654321;x.vi[0]=0x1234;auto y=x;
        const auto control=(saved&~(_MM_ROUND_MASK|0x8040|0x3f))|mode|(seed&8?0x8040:0);_mm_setcsr(control);
        a.runtime().functions.generated(0x2476b0)(a.runtime().memory.flat(),&x,&a.runtime());const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        b.runtime().functions.generated(0x2476b0)(b.runtime().memory.flat(),&y,&b.runtime());check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);x.cycleLimit=y.cycleLimit;
        check(x.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.runtime().memory.flat(),b.runtime().memory.flat(),bozo::kFlatBytes) && old.commits==1 && now.commits==1);
        now.projections.finish(parent,object);old.projections.finish(parent,object);++pairs;
    }
    check(old.sites==now.sites && now.sites.size()==12);std::printf("%u complete paired-point builders,%u identity projections: integer arithmetic oracle, original scalar clipping/cursor commit, opaque W/bank, aliases/scratch/overlapping second read/four FP modes passed\n",pairs,known);
    for(unsigned which=0;which<17;++which){auto &rt=b.runtime();now.scene=vp2s::NativeScene{};now.projections=vp2s::NativeProceduralTransforms{};std::memset(rt.memory.flat(),0,bozo::kFlatBytes);cameraInput(rt,now);
        constexpr uint32_t frame=0x1ffe000,parent=frame+0xb0,prepare=parent-0x170;
        std::array<float,16> matrix{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};if(which==7)matrix[0]=1.e16f;if(which==8 || which==14){matrix[14]=1;matrix[15]=0;}
        std::array<float,16> transpose;for(unsigned row=0;row<4;++row)for(unsigned col=0;col<4;++col)transpose[4*col+row]=matrix[4*row+col];put(rt,prepare+0x130,transpose);put(rt,prepare+0x20,uint32_t(0x202bc8));
        R5900Context pub{};pub.pc=0x2481d0;pub.r[31].uw[0]=0x2481d0;pub.r[29].uw[0]=prepare;pub.r[4].uw[0]=prepare+0x130;pub.r[16].uw[0]=camera;pub.r[17].uw[0]=object;now.projections.publish(&rt,&pub,now.scene);
        put(rt,0x1d8000,std::array<float,4>{1,2,which==8?0.f:1.f,1});put(rt,0x1d9000,std::array<float,4>{1,2,which==14?0.f:1.f,1});
        R5900Context c{};c.pc=0x247700;c.r[31].uw[0]=0x247700;c.r[29].uw[0]=frame;c.r[21].uw[0]=object;c.r[20].uw[0]=1;c.r[19].uw[0]=0x1d8000;c.r[18].uw[0]=0x1d9000;c.r[9].uw[0]=0x1d8000;c.r[4].uw[0]=frame+0x90;c.r[3].uw[0]=frame+0x80;
        uint32_t site=0x2478c8;bool finish=false;
        if(which==0)c.pc=0;else if(which==1)c.r[31].uw[0]=0;else if(which==2)c.r[20].uw[0]=0;else if(which==3)c.r[5].uw[0]=1;else if(which==4)c.r[21].uw[0]+=16;else if(which==5)c.r[9].uw[0]+=16;else if(which==6)put(rt,0x1d8000,uint32_t(0x7fa01234));
        else if(which>=9){now.projections.point(&rt,&c,0x2478c8);
            if(which==9)site=0x2478c8;else if(which==10){site=0x24796c;}
            else {c.r[9].uw[0]=0x1d9000;now.projections.point(&rt,&c,0x247968);site=0x24796c;
                if(which==11)c.r[4].uw[0]+=16;
                else {now.projections.point(&rt,&c,0x24796c);site=0x247974;
                    if(which==12)site=0x247a18;
                    else if(which==13){site=0x247a1c;}
                    else if(which>=15){now.projections.point(&rt,&c,0x247974);now.projections.point(&rt,&c,0x247a18);site=0x247a1c;
                        if(which==15)c.r[3].uw[0]+=16;else finish=true;}}}}
        const auto before=c;const auto active=now.projections.pendingPoints(),count=now.projections.pending();const auto fp=_mm_getcsr();const auto ram=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);bool rejected=false;
        try{if(finish)now.projections.finish(parent,object);else now.projections.point(&rt,&c,site);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && now.projections.pendingPoints()==active && now.projections.pending()==count && _mm_getcsr()==fp && !std::memcmp(&before,&c,sizeof(c)) && !std::memcmp(ram.data(),rt.memory.flat(),ram.size()));
    }
    std::printf("17 procedural paired-point readonly input/order/lifetime rejections passed\n");
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
    _mm_setcsr(saved);pointCases();std::printf("%u compiled scalar projection publications: independent transpose/copy RAM oracle, immutable typed rows, opaque processor state, aliases/scratch/FP, finish lifetime and13 readonly rejections passed\n",cases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

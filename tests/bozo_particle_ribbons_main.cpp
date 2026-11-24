// Mode-4 particle ribbons (VU1 image 7) and mode-2 plain ribbons (image 5: no texture, no tint, ALPHA from the GS row
// block): NativeParticleDraws::capture against an oracle that executes each image's own words. The image comes from the game's mod001.elf at test time (table
// 3141a0 entry 7 -> a REF to a VIF packet of MPG uploads); nothing of it is stored here.
// The interpreter is sequential with pair semantics (both halves read the state before the
// pair): image 7 relies only on VU interlocks, and every FCAND it uses comes 4 or more
// instructions after the CLIPW it reads. Arithmetic: the engine's VU reference ops.
#include "native_particle_draws.h"
#include "bozo/ops_vu0.h"
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <format>
#include <fstream>
#include <random>
#include <source_location>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("particle ribbon assertion at "+std::to_string(at.line()));}
template<class T>void put(Runtime &rt,uint32_t at,const T &v){auto *p=rt.memory.hostPointer(at,sizeof(v));check(p);std::memcpy(p,&v,sizeof(v));}
template<class T>void write(std::ofstream &out,const T &v){out.write(reinterpret_cast<const char *>(&v),sizeof(v));}
using Mat=std::array<float,16>;
const Mat identity{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

// ---- image 7 from mod001.elf ----
std::vector<uint8_t> image(const std::filesystem::path &elfPath,unsigned which){
    std::ifstream in(elfPath,std::ios::binary);std::vector<uint8_t> elf((std::istreambuf_iterator<char>(in)),{});check(elf.size()>0x34);
    const auto u32=[&](size_t at){check(at+4<=elf.size());uint32_t v;std::memcpy(&v,elf.data()+at,4);return v;};
    const auto phoff=u32(0x1c);uint16_t phnum;std::memcpy(&phnum,elf.data()+0x2c,2);
    const auto read=[&](uint32_t address,uint32_t bytes)->const uint8_t *{
        for(unsigned i=0;i<phnum;++i){const auto h=phoff+32*i;if(u32(h)!=1)continue;const auto off=u32(h+4),va=u32(h+8),size=u32(h+16);
            if(address>=va && uint64_t(address)+bytes<=uint64_t(va)+size){check(off+(address-va)+bytes<=elf.size());return elf.data()+off+(address-va);}}
        throw std::runtime_error(std::format("mod001 address {:x} outside its image",address));};
    uint32_t entry,tag[2];std::memcpy(&entry,read(0x3141a0+4*which,4),4);std::memcpy(tag,read(entry,8),8);
    check(((tag[0]>>28)&7)==3);const auto qwc=tag[0]&0xffff;const auto *packet=read(tag[1],qwc*16);
    std::vector<uint8_t> code(16384);unsigned uploads=0;
    for(uint32_t i=0;i<qwc*16;){uint32_t vif;std::memcpy(&vif,packet+i,4);i+=4;const auto cmd=(vif>>24)&0x7f,num=(vif>>16)&0xff,imm=vif&0xffff;
        if(cmd==0x4a){const auto n=num?num:256;check(imm*8+n*8<=code.size() && i+n*8<=qwc*16);std::memcpy(code.data()+imm*8,packet+i,n*8);i+=n*8;++uploads;}
        else if(cmd==0x20)i+=4;else if(cmd==0x30 || cmd==0x31)i+=16;else if(cmd>=0x60)throw std::runtime_error("image 7 packet carries data");}
    check(uploads==(which==3?2u:3u));return code;
}

// ---- the oracle ----
struct Vu1 {
    using Vec=bozo::vu0::Vec;
    const std::vector<uint8_t> &code;std::array<uint8_t,16384> mem{};std::array<Vec,32> vf{};std::array<uint16_t,16> vi{};
    Vec acc{};uint32_t q=0x3f800000,i=0,p=0,clip=0,xtop=0;std::vector<uint32_t> kicks;
    explicit Vu1(const std::vector<uint8_t> &c):code(c){vf[0].u[3]=0x3f800000;}
    Vec &row(uint32_t r){check(r<1024);return *reinterpret_cast<Vec *>(mem.data()+16*r);}
    static bool on(uint32_t d,int f){return bozo::vu0::fieldOn(d,f);}
    static uint32_t add(uint32_t a,uint32_t b){uint32_t f=0;return bozo::vu0::addf(a,b,f);}
    static uint32_t sub(uint32_t a,uint32_t b){uint32_t f=0;return bozo::vu0::subf(a,b,f);}
    static uint32_t mul(uint32_t a,uint32_t b){uint32_t f=0;return bozo::vu0::mulf(a,b,f);}
    void upper(uint32_t w,std::array<Vec,32> &out,const std::array<Vec,32> &in,unsigned &written){
        const uint32_t d=(w>>21)&15,ft=(w>>16)&31,fs=(w>>11)&31,fd=(w>>6)&31,op=w&63;const Vec s=in[fs],t=in[ft];
        const auto store=[&](unsigned r,const uint32_t v[4]){if(!r)return;for(int f=0;f<4;++f)if(on(d,f))out[r].u[f]=v[f];written=r;};
        const auto binary=[&](unsigned r,auto fn,const Vec &a,const Vec &b){uint32_t v[4];for(int f=0;f<4;++f)v[f]=fn(a.u[f],b.u[f]);store(r,v);};
        const auto madd=[&](unsigned r,const Vec &a,const Vec &b,bool minus){uint32_t v[4];for(int f=0;f<4;++f){const auto pr=mul(a.u[f],b.u[f]);v[f]=minus?sub(acc.u[f],pr):add(acc.u[f],pr);}store(r,v);};
        const auto toAcc=[&](auto fn){for(int f=0;f<4;++f)if(on(d,f))acc.u[f]=fn(f);};
        if(op<0x1c){const auto b=bozo::vu0::bc(t,op&3);switch(op>>2){
            case 0:return binary(fd,add,s,b);case 1:return binary(fd,sub,s,b);case 2:return madd(fd,s,b,false);case 3:return madd(fd,s,b,true);
            case 4:return binary(fd,bozo::ps2f::max,s,b);case 5:return binary(fd,bozo::ps2f::min,s,b);default:return binary(fd,mul,s,b);}}
        const auto Q=bozo::vu0::broadcast(q),I=bozo::vu0::broadcast(i);
        switch(op){case 0x1c:return binary(fd,mul,s,Q);case 0x1d:return binary(fd,bozo::ps2f::max,s,I);case 0x1e:return binary(fd,mul,s,I);case 0x1f:return binary(fd,bozo::ps2f::min,s,I);
            case 0x20:return binary(fd,add,s,Q);case 0x21:return madd(fd,s,Q,false);case 0x22:return binary(fd,add,s,I);case 0x23:return madd(fd,s,I,false);
            case 0x24:return binary(fd,sub,s,Q);case 0x26:return binary(fd,sub,s,I);case 0x28:return binary(fd,add,s,t);case 0x29:return madd(fd,s,t,false);
            case 0x2a:return binary(fd,mul,s,t);case 0x2b:return binary(fd,bozo::ps2f::max,s,t);case 0x2c:return binary(fd,sub,s,t);case 0x2f:return binary(fd,bozo::ps2f::min,s,t);default:break;}
        check(op>=0x3c);const auto sp=(fd<<2)|(op&3);
        if(sp<0x10){const auto b=bozo::vu0::bc(t,sp&3);if((sp>>2)==2){toAcc([&](int f){return add(acc.u[f],mul(s.u[f],b.u[f]));});return;}
            if((sp>>2)==0){toAcc([&](int f){return add(s.u[f],b.u[f]);});return;}}
        if(sp>=0x18 && sp<=0x1b){const auto b=bozo::vu0::bc(t,sp&3);toAcc([&](int f){return mul(s.u[f],b.u[f]);});return;}
        uint32_t v[4]={};
        switch(sp){
            case 0x10:for(int f=0;f<4;++f)v[f]=bozo::ps2f::fromFixed(s.u[f],0);return store(ft,v);
            case 0x14:for(int f=0;f<4;++f)v[f]=bozo::ps2f::toFixed(s.u[f],0);return store(ft,v);
            case 0x15:for(int f=0;f<4;++f)v[f]=bozo::ps2f::toFixed(s.u[f],4);return store(ft,v);
            case 0x1c:toAcc([&](int f){return mul(s.u[f],q);});return;
            case 0x1e:toAcc([&](int f){return mul(s.u[f],i);});return;
            case 0x1f:{const auto w=t.u[3]&0x7fffffffu;uint32_t flags=0;
                for(int f=0;f<3;++f){if(bozo::ps2f::lt(w,s.u[f]))flags|=1u<<(2*f);if(bozo::ps2f::lt(s.u[f],w|0x80000000u))flags|=2u<<(2*f);}
                clip=((clip<<6)|flags)&0xffffff;return;}
            case 0x2a:toAcc([&](int f){return mul(s.u[f],t.u[f]);});return;
            case 0x2f:return;
            default:throw std::runtime_error(std::format("oracle: upper {:08x}",w));}
    }
    // Returns the branch target (or ~0) of the lower instruction.
    uint32_t lower(uint32_t w,uint32_t pc,const std::array<Vec,32> &in,unsigned upperWritten){
        const uint32_t op=(w>>25)&0x7f,d=(w>>21)&15,it=(w>>16)&31,is=(w>>11)&31;const int32_t imm11=int32_t(w<<21)>>21;
        const auto setVi=[&](unsigned r,uint32_t v){check(r<16);if(r)vi[r]=uint16_t(v);};
        const auto addr=[&](uint32_t base,int32_t off){return uint32_t(int32_t(vi[base&15])+off)&0x3ff;};
        const auto setVf=[&](unsigned r,const Vec &v){if(!r)return;check(r!=upperWritten);for(int f=0;f<4;++f)if(on(d,f))vf[r].u[f]=v.u[f];};
        const auto target=[&]{return (pc+8+imm11*8)&0x3fff;};
        switch(op){
            case 0x00:return setVf(it,row(addr(is,imm11))),~0u;
            case 0x01:{auto &r=row(addr(it,imm11));for(int f=0;f<4;++f)if(on(d,f))r.u[f]=in[is].u[f];return ~0u;}
            case 0x04:{const auto &r=row(addr(is,imm11));int f=0;while(!on(d,f))++f;setVi(it,r.u[f]&0xffff);return ~0u;}
            case 0x05:{auto &r=row(addr(is,imm11));for(int f=0;f<4;++f)if(on(d,f))r.u[f]=vi[it&15];return ~0u;}
            case 0x08:setVi(it,vi[is&15]+((d<<11)|(w&0x7ff)));return ~0u;
            case 0x09:setVi(it,vi[is&15]-((d<<11)|(w&0x7ff)));return ~0u;
            case 0x12:vi[1]=(clip&(w&0xffffff))?1:0;return ~0u;
            case 0x20:return target();
            case 0x21:setVi(it,(pc+16)/8);return target();
            case 0x24:return (uint32_t(vi[is&15])*8)&0x3fff;
            case 0x28:return vi[it&15]==vi[is&15]?target():~0u;
            case 0x29:return vi[it&15]!=vi[is&15]?target():~0u;
            case 0x2d:return int16_t(vi[is&15])>0?target():~0u;
            case 0x40:break;
            default:throw std::runtime_error(std::format("oracle: lower {:08x} at {:04x}",w,pc));}
        const auto funct=w&63;
        if(funct==0x30){setVi((w>>6)&31,vi[is&15]+vi[it&15]);return ~0u;}
        if(funct==0x31){setVi((w>>6)&31,vi[is&15]-vi[it&15]);return ~0u;}
        if(funct==0x34){setVi((w>>6)&31,vi[is&15]&vi[it&15]);return ~0u;}
        if(funct==0x35){setVi((w>>6)&31,vi[is&15]|vi[it&15]);return ~0u;}
        check(funct>=0x3c);const auto sp=(((w>>6)&31)<<2)|(funct&3);
        switch(sp){
            case 0x30:setVf(it,in[is]);return ~0u;
            case 0x34:setVf(it,row(vi[is&15]&0x3ff));setVi(is,vi[is&15]+1);return ~0u;
            case 0x35:{auto &r=row(vi[it&15]&0x3ff);for(int f=0;f<4;++f)if(on(d,f))r.u[f]=in[is].u[f];setVi(it,vi[it&15]+1);return ~0u;}
            case 0x38:{uint32_t f=0;q=bozo::ps2f::div(in[is].u[(w>>21)&3],in[it].u[(w>>23)&3],f);return ~0u;}
            // RSQRT Q = fs.fsf / sqrt(ft.ftf) (the light pass 1b28 of row 14.w = 2); WAITQ: Q is set at once here.
            case 0x3a:{uint32_t f=0;q=bozo::ps2f::rsqrt(in[is].u[(w>>21)&3],in[it].u[(w>>23)&3],f);return ~0u;}
            case 0x3b:return ~0u;
            case 0x3d:setVf(it,bozo::vu0::broadcast(uint32_t(int32_t(int16_t(vi[is&15])))));return ~0u;
            case 0x64:setVf(it,bozo::vu0::broadcast(p));return ~0u;
            case 0x68:setVi(it,xtop);return ~0u;
            case 0x6c:kicks.push_back(vi[is&15]);if(std::getenv("KICKPC"))std::fprintf(stderr,"kick at %04x vi09 %x vi13 %x\n",pc,vi[9],vi[13]);return ~0u;
            case 0x73:{const auto &s=in[is];const auto sum=add(add(mul(s.u[0],s.u[0]),mul(s.u[1],s.u[1])),mul(s.u[2],s.u[2]));uint32_t f=0;p=bozo::ps2f::rsqrt(0x3f800000,sum,f);return ~0u;}
            case 0x7b:return ~0u; // waitp
            case 0xbf:return ~0u; // nop
            default:throw std::runtime_error(std::format("oracle: lower {:08x} at {:04x}",w,pc));}
    }
    void run(uint32_t pc){
        uint32_t branch=~0u;bool ending=false;
        for(unsigned steps=0;;++steps){check(steps<200000 && pc+8<=code.size());uint32_t lo,up;std::memcpy(&lo,code.data()+pc,4);std::memcpy(&up,code.data()+pc+4,4);
            const auto in=vf;unsigned written=0;upper(up,vf,in,written);
            uint32_t next=~0u;if(up&0x80000000u)i=lo;else next=lower(lo,pc,in,written);
            vf[0]=Vec{};vf[0].u[3]=0x3f800000;
            if(ending)return;if(up&0x40000000u)ending=true;
            if(branch!=~0u){pc=branch;branch=next;check(next==~0u);continue;}
            branch=next;pc+=8;}
    }
};

// ---- fixture (as the mode 3/6 test's) ----
constexpr uint32_t object=0x160000,camera=0x170000,target=0x180000,current=0x190000,kinds=0x1e1020,fis=0x1d1000,manager=0x1b0000,chunks=0x210020,block=0x1d8000;
void files(const std::filesystem::path &dir){
    std::filesystem::create_directories(dir);
    // Two kinds, two passes (variant 1 differs): image 7 draws one pass with variant 0.
    {std::ofstream f(dir/"0000004000_001.pmat",std::ios::binary);write(f,uint32_t(0x33505056));write(f,uint32_t(1));
        const std::array<uint32_t,28> fields{0x20,2,0x160,1,0,2,1,1,0,2,0,1,0,2,2,0,1,0x40,2,2,0,1,0,2,2,0,1,0};write(f,fields);}
    {std::ofstream f(dir/"manifest.txt");f<<"VPNT1\n11223344 5 c_0000000000000000.tex\n";}
    {std::ofstream f(dir/"c_0000000000000000.tex",std::ios::binary);write(f,std::array<uint32_t,3>{2,2,0});write(f,std::array<uint32_t,4>{0x800000ff,0x8000ff00,0x80ff0000,0x80ffffff});write(f,std::array<uint32_t,256>{});}
}
struct Case {unsigned groups[4],perChunk,chunks;bool lit,coincide,tilt,ambient=false;unsigned lights=0;};
bool plain=false;   // image 5 (mode 2) instead of image 7 (mode 4)
bool dots=false;    // with plain: image 3 (mode 0), GS points
bool fogged=false;  // image 5 with row 15.w: its fog variant (camera fog pair -60, 200; FOGCOL 203040)
void initialize(Runtime &rt,vp2s::NativeScene &scene,vp2s::NativeAssets &assets,const Case &c,std::mt19937 &random){
    std::memset(rt.memory.flat(),0,bozo::kFlatBytes);scene=vp2s::NativeScene{};assets=vp2s::NativeAssets{};
    put(rt,0x1c8d00+0xcc84,uint8_t(1));put(rt,0x35ddf8,current);put(rt,current+0x40a8,object);put(rt,current+0x40dc,uint8_t(1));
    put(rt,target+0x3c76,uint8_t(1));put(rt,target+0x3c60,1.f);put(rt,target+0x3c64,1.f);put(rt,target+0x3c68,int16_t(640));put(rt,target+0x3c6a,int16_t(448));
    for(unsigned i=0;i<2;++i){const auto b=target+0xa70*i;put(rt,b+0x1cf0,uint8_t(1));put(rt,b+0x1cf2,uint8_t(i));put(rt,b+0x1cf5,uint8_t(1));put(rt,b+0x1cfa,int16_t(640));put(rt,b+0x1cfc,int16_t(224));}
    scene.beginComposition(&rt,0x1f8000,target);put(rt,camera+0x220,uint32_t(0x153120));put(rt,camera+0x8e0,uint32_t(0x1a0000));put(rt,0x1a0000,uint32_t(0x153140));
    // A perspective screen (w = z + .5, x/y-free) and a unit guard; a turned view.
    Mat view=identity;view[0]=view[5]=std::cos(.3f);view[1]=std::sin(.3f);view[4]=-view[1];view[11]=.1f;put(rt,camera+0x2f0,view);put(rt,camera+0x370,identity);
    auto screen=identity;screen[0]=100;screen[5]=100;screen[3]=screen[7]=0;screen[2]=screen[6]=2048*1.f;screen[11]=1000;screen[14]=1;screen[15]=.5f;
    if(c.tilt)screen[12]=.01f;put(rt,camera+0x330,screen);
    if(fogged){put(rt,camera+0x5a0,uint32_t(0x203040));put(rt,camera+0x5a8,-60.f);put(rt,camera+0x5ac,200.f);}
    put(rt,camera+0x244,2048.f);put(rt,camera+0x248,2048.f);put(rt,camera+0x260,1728.f);put(rt,camera+0x264,1824.f);scene.selectCamera(&rt,camera);
    put(rt,object,uint32_t(0x1725d0));put(rt,object+0x63,uint8_t(1));put(rt,object+0x65,uint8_t(0));Mat world=identity;world[3]=.05f;world[7]=-.05f;put(rt,object+0x90,world);
    put(rt,object+0xaff,uint8_t(dots?0:plain?2:4));put(rt,object+0xaf9,uint8_t(0));put(rt,object+0xaa0,uint32_t(0x200000));put(rt,object+0xaa4,uint32_t(0x210000));
    put(rt,object+0x32c,uint32_t(fogged?1:0));put(rt,object+0x628,plain?0u:kinds);put(rt,object+0x658,plain?0x55555555u:fis);
    put(rt,fis+4,uint32_t(0x11223344));put(rt,fis+8,uint32_t(5));put(rt,0x35de38,manager);put(rt,manager+0x2670,int16_t(1));put(rt,manager+0x1008+16,uint32_t(0x11223344));put(rt,manager+8+16,uint32_t(5));
    put(rt,object+0x230,std::array<float,4>{255,128,255,64});put(rt,object+0x300,std::array<float,4>{.5f,1,.25f,2});put(rt,object+0x310,uint32_t(c.lit?0:1));
    // The GS row block (+624, rows 96..110): row 97.y (ALPHA.FIX) switches image 7's ambient step 1aa0 on, row 100.y
    // scales rows 5..8 into row 13 (+ row 19). Lights (row 14.w = 2, 14.y = 1): positions 20..22, colours 23..25, k row 27.
    put(rt,object+0x624,block);put(rt,block+0xc,uint32_t(0x6c0f0060));put(rt,block+0x24,uint32_t(c.ambient?0x40:0));put(rt,block+0x54,.5f);
    // Image 5's GS state (battle 0x1c's block): row 97 ALPHA_1 = 0x44 (normal alpha), row 102 the Gouraud strip tag.
    put(rt,block+0x20,uint32_t(0x44));put(rt,block+0x28,uint32_t(0x42));put(rt,block+0x10,uint32_t(0x8004));put(rt,block+0x14,uint32_t(0x10000000));
    put(rt,block+0x18,uint32_t(0xeeeeeeee));put(rt,block+0x70,uint32_t(0x8000));put(rt,block+0x74,uint32_t(dots?0x20304000:0x20364000));put(rt,block+0x78,uint64_t(0x41));
    if(c.ambient){put(rt,object+0x230+16*5,std::array<float,4>{.25f,.5f,.125f,0});put(rt,object+0x230+16*6,std::array<float,4>{.25f,.25f,.25f,0});
        put(rt,object+0x230+16*8,std::array<float,4>{-.01f,-.06f,-.002f,0});put(rt,object+0x230+16*19,std::array<float,4>{.85f,.64f,.47f,1});}
    if(c.lights){put(rt,object+0x31c,uint32_t(2));put(rt,object+0x314,uint32_t(1));
        put(rt,object+0x230+16*20,std::array<float,4>{3.f,-4.f,5.f,1});put(rt,object+0x230+16*23,std::array<float,4>{.05f,.04f,.03f,0});put(rt,object+0x230+16*27,std::array<float,4>{152,40,9,0});
        put(rt,object+0x230+16*21,std::array<float,4>{1.7e-9f,1e-15f,-8e-11f,0});put(rt,object+0x230+16*22,std::array<float,4>{-2,1,-6,0});
        if(c.lights>1){put(rt,object+0x230+16*24,std::array<float,4>{.01f,.02f,.03f,0});put(rt,object+0x230+16*25,std::array<float,4>{.03f,.01f,.02f,0});}}
    std::memset(rt.memory.hostPointer(kinds,0x160),0,0x160);put(rt,kinds,uint8_t(2));put(rt,kinds+1,uint8_t(1));put(rt,kinds+2,uint8_t(0));
    const std::array<float,20> k0{-1,-1,0,1,1,1,0,1, .125f,.25f,1,1, .75f,.875f,1,1, 1,.5f,1,2};put(rt,kinds+0xc0,k0);
    const std::array<float,20> k1{-1,-1,0,1,1,1,0,1, 0,1,1,1, 1,0,1,1, .5f,1,.25f,1};put(rt,kinds+0xc0+80,k1);
    assets.discRead(0x140000,1,8);assets.begin(0x1f0000,0x140000,kinds-0x20,1);assets.finish(0x1f0000,true,0x180);
    std::uniform_real_distribution<float> spread(-1.3f,1.3f),depth(.1f,.9f),wide(.02f,.3f);std::uniform_int_distribution<int> colour(0,40);
    uint32_t cursor=chunks;unsigned total=0;
    for(unsigned k=0;k<c.chunks;++k){const auto group=c.groups[k],count=c.perChunk*group;std::memset(rt.memory.hostPointer(cursor,32+48*count),0xa5,32+48*count);
        put(rt,cursor+0x10,uint32_t(count));put(rt,cursor+0x14,uint32_t(count*3+2));put(rt,cursor+0x18,uint32_t(group));put(rt,cursor+0x1c,uint32_t(0));
        for(unsigned i=0;i<count;++i){const auto at=cursor+32+48*i;const bool same=c.coincide && i%group<2;
            put(rt,at,std::array<float,3>{same?.2f:spread(random),same?.1f:spread(random)*.8f,depth(random)});
            put(rt,at+16,std::array<float,4>{float(colour(random)*4),float(colour(random)*4),float(colour(random)*4),float(colour(random)*2)});
            put(rt,at+32,wide(random));put(rt,at+44,uint32_t((i/group)&1));}
        cursor+=32+48*count;total+=count;}
    put(rt,object+0xabc,uint32_t(total));put(rt,object+0xac4,uint32_t(c.chunks));
}
// One VU1 run per chunk: the 0818 entry (lighting subroutine 1670, then 0868), with the owner's
// rows, the camera rows and the template laid out as the draw header uploads them.
struct Strip {std::vector<std::array<uint32_t,12>> vertices;};
Strip oracle(const std::vector<uint8_t> &code,Runtime &rt,const vp2s::NativeCamera &cam,uint32_t chunk){
    Vu1 vu(code);const auto *owner=rt.memory.hostPointer(object,0xb10);
    for(unsigned r=0;r<47;++r)std::memcpy(&vu.row(r),owner+0x230+16*r,16);
    for(unsigned r=0;r<15;++r)std::memcpy(&vu.row(96+r),rt.memory.hostPointer(block+0x10+16*r,16),16);
    Mat world;std::memcpy(world.data(),owner+0x90,64);Mat toView{};for(unsigned a=0;a<4;++a)for(unsigned b=0;b<4;++b)for(unsigned k=0;k<4;++k)toView[a*4+b]+=cam.view[a*4+k]*world[k*4+b];
    const auto columns=[&](unsigned first,const Mat &m){for(unsigned col=0;col<4;++col)for(unsigned r=0;r<4;++r)vu.row(first+col).u[r]=std::bit_cast<uint32_t>(m[r*4+col]);};
    columns(52,cam.screen);columns(56,cam.guard);columns(77,toView);std::memcpy(&vu.row(68),rt.memory.hostPointer(camera+0x5a0,16),16);
    vu.row(81).u[3]=0x1a0;vu.row(36).u[0]=vu.row(36).u[1]=0x1a0;if(!plain){vu.row(102).u[1]=0x30000000u;vu.row(102).u[2]=0x512;}
    const auto *k=rt.memory.hostPointer(kinds,0x160);for(unsigned r=0;r<9;++r)std::memcpy(&vu.row(0x2e0+r),k+0x20+16*r,16);for(unsigned r=0;r<10;++r)std::memcpy(&vu.row(0x2e9+r),k+0xc0+16*r,16);
    const auto count=*reinterpret_cast<const uint32_t *>(rt.memory.hostPointer(chunk+0x10,4));
    vu.xtop=0x100;std::memcpy(&vu.row(0x100),rt.memory.hostPointer(chunk+0x10,16+48*count),16+48*count);
    // The draw's first MSCAL (6e0) calls the ambient step 1aa0 at 0738 before 1670; its row 13 / row 14.x stay in VU
    // memory for the following chunks (0818). Here 1aa0 runs on its own, returning (vi15) into an E-bit stub at 3ff0.
    if(dots){   // image 3: vf01..vf04 = local-to-screen and rows 85..88 = guard x view x world (the set-up 238 leaves them)
        Mat l2s{},g2v{};for(unsigned a=0;a<4;++a)for(unsigned b=0;b<4;++b)for(unsigned k=0;k<4;++k){l2s[a*4+b]+=cam.screen[a*4+k]*toView[k*4+b];g2v[a*4+b]+=cam.guard[a*4+k]*toView[k*4+b];}
        for(unsigned col=0;col<4;++col)for(unsigned r=0;r<4;++r){vu.vf[1+col].u[r]=std::bit_cast<uint32_t>(l2s[r*4+col]);vu.row(85+col).u[r]=std::bit_cast<uint32_t>(g2v[r*4+col]);}
        // The real first MSCAL (6e0): ambient 0f38, the GS-state kick of rows 96..99 at row 81.w, lighting 0b08, then vf18 =
        // row 96 (0788) and the points packet after the state (a fogged chunk's first point takes FTOI0 of that vf18).
        vu.run(0x6e0);if(vu.kicks.size()!=3 || vu.kicks[0]!=0x3b0 || vu.kicks[1]!=0x1a0)throw std::runtime_error(std::format("image 3 kicks {}",vu.kicks.size()));
        const unsigned at=vu.kicks[2];const auto tag=vu.row(at);check(tag.u[0]==0x8000+count && tag.u[1]==0x20304000u && tag.u[2]==0x41);
        Strip s;for(unsigned v=0;v<count;++v){std::array<uint32_t,12> out{};std::memcpy(out.data()+4,&vu.row(at+1+2*v),16);std::memcpy(out.data()+8,&vu.row(at+2+2*v),16);s.vertices.push_back(out);}
        return s;}
    vu.vi[15]=0x3ff0/8;vu.run(plain?0x1a38:0x1aa0);check(vu.kicks.empty());
    vu.run(0x818);check(vu.kicks.size()==1 && vu.kicks[0]==0x1a0);
    if(std::getenv("RIBBON_COLOURS"))for(unsigned i=0;i<count;++i){float in[4],out[4];std::memcpy(in,rt.memory.hostPointer(chunk+0x30+48*i,16),16);std::memcpy(out,&vu.row(0x101+3*i+1),16);
        std::fprintf(stderr,"point %u colour %g %g %g %g -> %g %g %g %g\n",i,in[0],in[1],in[2],in[3],out[0],out[1],out[2],out[3]);}
    if(plain){   // image 5: the tag at the kick, then RGBAQ and XYZF2 per vertex (the RGBAQ in the first four words)
        const auto tag=vu.row(0x1a0);check(tag.u[0]==0x8000+2*count && tag.u[1]==0x20364000u && tag.u[2]==0x41);
        Strip s;for(unsigned v=0;v<2*count;++v){std::array<uint32_t,12> out{};std::memcpy(out.data()+4,&vu.row(0x1a1+2*v),16);std::memcpy(out.data()+8,&vu.row(0x1a2+2*v),16);
            out[2]=0x3f800000;s.vertices.push_back(out);}
        return s;}
    const auto tag=vu.row(0x1a0+5);if(!(tag.u[0]==0x8000+2*count && tag.u[1]==0x30000000u))for(unsigned r=0;r<8;++r)std::fprintf(stderr,"%x: %08x %08x %08x %08x\n",0x1a0+r,vu.row(0x1a0+r).u[0],vu.row(0x1a0+r).u[1],vu.row(0x1a0+r).u[2],vu.row(0x1a0+r).u[3]);check(tag.u[0]==0x8000+2*count && tag.u[1]==0x30000000u);
    Strip s;for(unsigned v=0;v<2*count;++v){std::array<uint32_t,12> out;for(unsigned j=0;j<3;++j)std::memcpy(out.data()+4*j,&vu.row(0x1a0+6+3*v+j),16);s.vertices.push_back(out);}
    return s;
}
}
int main(int argc,char **argv){try{
    check(argc==3);const std::filesystem::path dir=argv[1];files(dir);
    // An E-bit NOP pair (and its delay pair) at 3ff0: the return point of a subroutine run on its own.
    auto images=std::array{image(argv[2],7),image(argv[2],5),image(argv[2],3)};
    for(auto &code:images){check(code.size()>=0x4000);for(uint32_t at:{0x3ff0u,0x3ff8u}){const uint32_t lower=0x8000033c,upper=at==0x3ff0?0x400002ffu:0x000002ffu;
        std::memcpy(code.data()+at,&lower,4);std::memcpy(code.data()+at+4,&upper,4);}}
    unsigned plainCases=0,fogSeen=0;
    Runtime rt;vp2s::NativeScene scene;vp2s::NativeAssets assets;vp2s::NativeParticles particles;vp2s::NativeParticleDraws draws(dir);vp2s::NativeTextures textures(dir,dir/"manifest.txt");
    std::mt19937 random(0x7a11);unsigned cases=0,triangles=0,culled=0;
    const Case table[]={{{2},1,1,false,false,false},{{3},2,1,false,false,false},{{5},3,1,true,false,false},{{2,4,6},2,3,false,false,false},
        {{22},1,1,false,true,false},{{8,3},2,2,true,true,false},{{16},3,1,false,false,false},{{48},1,1,true,false,false},{{4},12,1,false,false,false},
        {{5},3,1,false,false,false,true},{{6},2,1,true,false,false,true},{{4},3,1,false,false,false,false,1},{{7},2,1,true,false,false,true,1},{{3},4,1,false,false,false,true,2}};
    for(const bool five:{false,true})for(unsigned round=0;round<8;++round)for(const auto &c:table){
        plain=five;fogged=five && (round&1);const auto &code=images[five];plainCases+=five;
        initialize(rt,scene,assets,c,random);const auto cam=scene.drawingCamera();check(bool(cam));
        const auto result=draws.capture(&rt,assets,textures,scene,particles,object);check(result.batches.size()<=1);
        // Expected: per chunk, the GS strip: vertex j (j >= 2) completes (j-2, j-1, j) unless ADC.
        std::vector<std::array<uint32_t,12>> expect;uint32_t chunk=chunks;
        for(unsigned k=0;k<c.chunks;++k){const auto s=oracle(code,rt,*cam,chunk);if(std::getenv("RIBBON_DUMP") && cases==unsigned(std::atoi(std::getenv("RIBBON_DUMP"))))for(size_t n=0;n<s.vertices.size();++n){const auto *p=rt.memory.hostPointer(chunk+32+48*(n/2),12);float q[3];std::memcpy(q,p,12);std::fprintf(stderr,"strip %zu: %d %d %x  point %g %g %g\n",n,int32_t(s.vertices[n][8]),int32_t(s.vertices[n][9]),s.vertices[n][11],q[0],q[1],q[2]);}
            for(unsigned j=2;j<s.vertices.size();++j){if(s.vertices[j][11]&0x8000){++culled;continue;}for(unsigned v=j-2;v<=j;++v)expect.push_back(s.vertices[v]);++triangles;}
            chunk+=32+48*(c.perChunk*c.groups[k]);}
        const std::vector<bozo::render::Vertex> none;const auto &got=result.batches.empty()?none:result.batches[0].vertices;
        if(std::getenv("RIBBON_DUMP") && cases==unsigned(std::atoi(std::getenv("RIBBON_DUMP"))))for(size_t n=0;n<got.size();++n){const auto &m=result.batches[0].matrix;const auto &v=got[n];float cl[4];for(unsigned r=0;r<4;++r)cl[r]=m[r]*v.x+m[4+r]*v.y+m[8+r]*v.z+m[12+r];
            std::fprintf(stderr,"%zu native %.1f %.1f oracle %d %d adc %x\n",n,cl[0]/cl[3]*16,cl[1]/cl[3]*16,int32_t(expect[n][8]),int32_t(expect[n][9]),expect[n][11]);}
        if(got.size()!=expect.size())throw std::runtime_error(std::format("case {}: {} native vertices, oracle {}",cases,got.size(),expect.size()));
        if(!result.batches.empty()){const auto &b=result.batches[0];
            if(plain)check(!b.material.textured && b.material.blendA==0 && b.material.blendB==1 && b.material.blendC==0 && b.material.blendD==1 &&
                b.material.fog==fogged && (!fogged || b.material.fogColour==0x203040));
            else check(b.material.blendA==0 && b.material.blendB==2 && b.material.blendD==1);}
        for(unsigned n=0;n<got.size();++n){const auto &v=got[n];const auto &o=expect[n];const auto &m=result.batches[0].matrix;
            float clip[4];for(unsigned r=0;r<4;++r)clip[r]=m[r]*v.x+m[4+r]*v.y+m[8+r]*v.z+m[12+r];
            const auto f=[&](unsigned i){return std::bit_cast<float>(o[i]);};const float qv=f(2);
            const bool ok=std::abs(clip[0]/clip[3]*16-int32_t(o[8]))<=1.5f && std::abs(clip[1]/clip[3]*16-int32_t(o[9]))<=1.5f && std::abs(clip[2]/clip[3]*16-int32_t(o[10]))<=2.f &&
                (plain || std::abs(qv*clip[3]-1)<1.e-5f) &&std::abs(v.u-f(0)/qv)<=1.e-5f && std::abs(v.v-f(1)/qv)<=1.e-5f;
            uint32_t rgba=0;for(unsigned i=0;i<4;++i)rgba|=(o[4+i]&0xff)<<(8*i);
            const bool fogOk=!plain || std::abs(int(std::lround(v.fog*255))-int((o[11]>>4)&0xff))<=1;fogSeen+=plain && ((o[11]>>4)&0xff)<255;
            if(!ok || !fogOk || rgba!=v.rgba)throw std::runtime_error(std::format("case {} vertex {}: native ({} {} {} uv {} {} {:08x}) clip ({} {} {}), oracle XYZ ({} {} {}) STQ ({} {} {}) {:08x}",
                cases,n,v.x,v.y,v.z,v.u,v.v,v.rgba,clip[0]/clip[3]*16,clip[1]/clip[3]*16,clip[2]/clip[3]*16,int32_t(o[8]),int32_t(o[9]),int32_t(o[10]),f(0),f(1),qv,rgba));}
        ++cases;
    }
    check(triangles>500 && culled>50);
    // Refusals: the fog variant (row 15.w), lighting pass 1c78 (row 14.w = 2 with 14.z), a screen w row with an x term,
    // a ribbon of one point, a ribbon kind outside the template.
    check(fogSeen>100);
    // Image 3 (mode 0): one GS point per particle, drawn as a GS pixel; positions, colours, fog and ADC against its words.
    unsigned dotCases=0,dotsDrawn=0,dotsCulled=0,dotsFogged=0;dots=plain=true;
    unsigned laterChunkFirsts=0;
    for(unsigned round=0;round<8;++round)for(const auto &c:table){fogged=round&1;
        initialize(rt,scene,assets,c,random);const auto cam=scene.drawingCamera();check(bool(cam));
        const auto result=draws.capture(&rt,assets,textures,scene,particles,object);check(result.batches.size()<=1);
        std::vector<std::array<uint32_t,12>> expect;uint32_t chunk=chunks;
        for(unsigned k=0;k<c.chunks;++k){const auto s=oracle(images[2],rt,*cam,chunk);
            // The oracle runs every chunk from 6e0; a later chunk's first fogged point takes leftover VU memory on the PS2 (818),
            // defined as colour 0 (user 2026-11-24: "Colour 0, invisible").
            for(size_t n=0;n<s.vertices.size();++n){auto v=s.vertices[n];if(fogged && k && !n){for(unsigned i=4;i<8;++i)v[i]=0;++laterChunkFirsts;}
                if(v[11]&0x8000){++dotsCulled;continue;}expect.push_back(v);}chunk+=32+48*(c.perChunk*c.groups[k]);}
        const std::vector<bozo::render::Vertex> none;const auto &got=result.batches.empty()?none:result.batches[0].vertices;
        if(got.size()!=6*expect.size())throw std::runtime_error(std::format("point case {}: {} native vertices, oracle {} points",dotCases,got.size(),expect.size()));
        if(!result.batches.empty())check(!result.batches[0].material.textured && result.batches[0].material.fog==fogged);dotsFogged+=fogged;
        for(size_t n=0;n<expect.size();++n){const auto &v=got[6*n];const auto &o=expect[n];uint32_t rgba=0;for(unsigned i=0;i<4;++i)rgba|=(o[4+i]&0xff)<<(8*i);
            const bool ok=std::abs(v.x*16-int32_t(o[8]))<=1.5f && std::abs(v.y*16-int32_t(o[9]))<=1.5f && std::abs(v.z*16-int32_t(o[10]))<=2.f &&
                std::abs(int(std::lround(v.fog*255))-int((o[11]>>4)&0xff))<=1 && rgba==v.rgba;
            if(!ok)throw std::runtime_error(std::format("point case {} point {}: native ({} {} {} fog {} {:08x}), oracle ({} {} {} w {:x}) {:08x}",dotCases,n,v.x*16,v.y*16,v.z*16,v.fog*255,v.rgba,int32_t(o[8]),int32_t(o[9]),int32_t(o[10]),o[11],rgba));
            ++dotsDrawn;}
        ++dotCases;}
    check(dotsDrawn>500 && dotsCulled>50 && dotsFogged>40 && laterChunkFirsts>8);fogged=false;std::printf("%u point captures equal image 3's words: %u points drawn, %u ADC-culled\n",dotCases,dotsDrawn,dotsCulled);
    dots=plain=fogged=false;
    for(unsigned which=0;which<5;++which){Case c{{3},1,1,false,false,which==2};initialize(rt,scene,assets,c,random);
        if(which==0)put(rt,object+0x32c,uint32_t(1));else if(which==1){put(rt,object+0x31c,uint32_t(2));put(rt,object+0x318,uint32_t(1));}
        else if(which==3){put(rt,chunks+0x10,uint32_t(1));put(rt,chunks+0x14,uint32_t(5));put(rt,chunks+0x18,uint32_t(1));put(rt,object+0xabc,uint32_t(1));}
        else if(which==4)put(rt,chunks+0x20+44,uint32_t(2));
        bool refused=false;try{draws.capture(&rt,assets,textures,scene,particles,object);}catch(const std::runtime_error &){refused=true;}check(refused);}
    // Image 5 refusals: a GS block whose ALPHA register or strip tag differs.
    plain=true;
    for(unsigned which=0;which<2;++which){Case c{{3},1,1,false,false,false};initialize(rt,scene,assets,c,random);
        if(which==0)put(rt,block+0x28,uint32_t(0x43));else put(rt,block+0x74,uint32_t(0x20324000));
        bool refused=false;try{draws.capture(&rt,assets,textures,scene,particles,object);}catch(const std::runtime_error &){refused=true;}check(refused);}
    std::printf("%u ribbon captures equal images 7 and 5's words (%u plain): %u triangles drawn, %u ADC-culled; %u fogged vertices; 7 refusals passed\n",cases,plainCases,triangles,culled,fogSeen);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

#include "native_debug_font.h"
#include "native_assets.h"
#include "native_state_setup.h"
#include "native_screen_rect.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "font_init_original_functions.h"
#include "font_init_adapted_functions.h"
#include "font_flush_original_functions.h"
#include "font_flush_adapted_functions.h"
#include "font_defaults_original_functions.h"
#include "font_defaults_adapted_functions.h"
#include "font_empty_original_functions.h"
#include "font_empty_adapted_functions.h"
#include "font_fade_original_functions.h"
#include "font_fade_adapted_functions.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <source_location>
#include <algorithm>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){
    if(!v)throw std::runtime_error("native debug font assertion at "+std::to_string(at.line()));
}
struct Pause{};
template<class T>T value(Runtime &rt,uint32_t at){T v;std::memcpy(&v,rt.memory.hostPointer(vp2s::NativeAssets::address(at),sizeof(v)),sizeof(v));return v;}
template<class T>void put(Runtime &rt,uint32_t at,T v){std::memcpy(rt.memory.hostPointer(vp2s::NativeAssets::address(at),sizeof(v)),&v,sizeof(v));}
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::DebugFontOwner,vp2s::TextureOwner,vp2s::StateSetupOwner,vp2s::ScreenRectOwner {
    vp2s::NativeScreenRects rects;std::vector<vp2s::NativeScreenRect> draws;
    vp2s::NativeScreenRects &nativeScreenRects()override{return rects;}
    void drawNativeScreenRect(const vp2s::NativeScreenRect &r)override{draws.push_back(r);}
    vp2s::NativeStateSetups setups;
    vp2s::NativeStateSetups &nativeStateSetups()override{return setups;}
    vp2s::NativeDebugFonts fonts;vp2s::NativeTextures textures;
    bool adapted=false,pause=false,allocationFailure=false;unsigned pauses=0,enqueues=0;
    explicit Host(const std::filesystem::path &p):textures(p,p/"manifest.txt"){}
    vp2s::NativeDebugFonts &nativeDebugFonts()override{return fonts;}
    vp2s::NativeTextures &nativeTextures()override{return textures;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(!adapted && pc==0x2075cc)setups.returned(rt,c->r[29].uw[0]-0x30);
        if(!adapted && pc==0x3d364c)rects.flushed(rt,c->r[29].uw[0]);
        if(auto known=vp2s::nativeScreenRectInstruction(c,rt,pc))return *known;
        if(auto known=vp2s::nativeStateSetupInstruction(r,c,rt,pc))return *known;
        auto result=vp2s::nativeDebugFontInstruction(r,c,rt,pc);check(result.has_value());return *result;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        auto &rt=d.runtime();
        if(pc==0x112860){put(rt,c->r[5].uw[0],1.f);put(rt,c->r[6].uw[0],1.f);}
        else if(pc==0x120fa0)c->r[2].sd[0]=allocationFailure?0:static_cast<int32_t>(value<uint32_t>(rt,c->r[4].uw[0]+8));
        else if(pc==0x1134c0)c->r[2].sd[0]=0x1234;
        else if(pc==0x120f70)put(rt,c->r[4].uw[0]+8,(c->r[5].uw[0]+15)&~15u);
        else if(pc==0x1131c0){check(!adapted);if(rects.active(c->r[29].uw[0]+0x30))rects.retire(&rt,c);else if(setups.active(c->r[29].uw[0]))setups.retire(&rt,c);else fonts.retire(&rt,c);++enqueues;c->r[2].sd[0]=7;}
        else if(pc==0x1130f0)c->r[2].ud[0]=0;
        else throw std::runtime_error("unexpected font leaf");
        c->pc=c->r[31].uw[0];if(pause){++pauses;throw Pause{};}
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
void run(bozo::ee::Dispatcher &d,R5900Context &c,uint32_t pc){
    auto fn=d.runtime().functions.generated(pc);check(fn);unsigned tries=0;
    while(true){try{fn(d.runtime().memory.flat(),&c,&d.runtime());if(c.pc==0x170000)break;}catch(const Pause &){}
        check(++tries<16);auto entry=d.runtime().functions.resumeEntry(c.pc);check(entry);fn=entry->resume;
    }
}
}
int main(int argc,char **argv){try{
    check(argc==2);Host old(argv[1]),now(argv[1]);now.adapted=true;bozo::ee::Dispatcher a(old),b(now);
    std::vector<BozoFunctionEntry> oldFunctions(font_init_original_function_table,font_init_original_function_table+font_init_original_function_count);
    oldFunctions.insert(oldFunctions.end(),font_flush_original_function_table,font_flush_original_function_table+font_flush_original_function_count);
    oldFunctions.insert(oldFunctions.end(),font_defaults_original_function_table,font_defaults_original_function_table+font_defaults_original_function_count);
    oldFunctions.insert(oldFunctions.end(),font_empty_original_function_table,font_empty_original_function_table+font_empty_original_function_count);
    oldFunctions.insert(oldFunctions.end(),font_fade_original_function_table,font_fade_original_function_table+font_fade_original_function_count);
    std::vector<BozoResumeEntry> oldResume(font_init_original_resume_table,font_init_original_resume_table+font_init_original_resume_count);
    oldResume.insert(oldResume.end(),font_flush_original_resume_table,font_flush_original_resume_table+font_flush_original_resume_count);
    oldResume.insert(oldResume.end(),font_defaults_original_resume_table,font_defaults_original_resume_table+font_defaults_original_resume_count);
    oldResume.insert(oldResume.end(),font_empty_original_resume_table,font_empty_original_resume_table+font_empty_original_resume_count);
    oldResume.insert(oldResume.end(),font_fade_original_resume_table,font_fade_original_resume_table+font_fade_original_resume_count);
    std::vector<BozoFunctionEntry> newFunctions(font_init_adapted_function_table,font_init_adapted_function_table+font_init_adapted_function_count);
    newFunctions.insert(newFunctions.end(),font_flush_adapted_function_table,font_flush_adapted_function_table+font_flush_adapted_function_count);
    newFunctions.insert(newFunctions.end(),font_defaults_adapted_function_table,font_defaults_adapted_function_table+font_defaults_adapted_function_count);
    newFunctions.insert(newFunctions.end(),font_empty_adapted_function_table,font_empty_adapted_function_table+font_empty_adapted_function_count);
    newFunctions.insert(newFunctions.end(),font_fade_adapted_function_table,font_fade_adapted_function_table+font_fade_adapted_function_count);
    std::vector<BozoResumeEntry> newResume(font_init_adapted_resume_table,font_init_adapted_resume_table+font_init_adapted_resume_count);
    newResume.insert(newResume.end(),font_flush_adapted_resume_table,font_flush_adapted_resume_table+font_flush_adapted_resume_count);
    newResume.insert(newResume.end(),font_defaults_adapted_resume_table,font_defaults_adapted_resume_table+font_defaults_adapted_resume_count);
    newResume.insert(newResume.end(),font_empty_adapted_resume_table,font_empty_adapted_resume_table+font_empty_adapted_resume_count);
    newResume.insert(newResume.end(),font_fade_adapted_resume_table,font_fade_adapted_resume_table+font_fade_adapted_resume_count);
    for(auto *table:{&oldFunctions,&newFunctions})std::sort(table->begin(),table->end(),[](const auto &x,const auto &y){return x.address<y.address;});
    for(auto *table:{&oldResume,&newResume})std::sort(table->begin(),table->end(),[](const auto &x,const auto &y){return x.start<y.start;});
    a.setFunctions(oldFunctions.data(),oldFunctions.size(),oldResume.data(),oldResume.size());
    b.setFunctions(newFunctions.data(),newFunctions.size(),newResume.data(),newResume.size());a.runtime().port=&old;b.runtime().port=&now;
    for(auto *d:{&a,&b})for(uint32_t pc:{0x112860u,0x120fa0u,0x1134c0u,0x120f70u,0x1131c0u,0x1130f0u})d->bindLibrary(pc);
    unsigned cases=0;
    for(unsigned seed=0;seed<64;++seed){
        old.fonts={};now.fonts={};old.enqueues=now.enqueues=now.pauses=0;now.pause=(seed&4)!=0;
        old.allocationFailure=now.allocationFailure=seed%5==0;
        auto &x=a.runtime(),&y=b.runtime();auto *ram=x.memory.flat();std::memset(ram,0x61,bozo::kFlatBytes);
        constexpr uint32_t object=0x180000,buffer=0x190000,source=0x1a0000,base=0x301b0000;
        put<uint32_t>(x,object+0x38,0x159140);put<uint32_t>(x,object+0x34,buffer);put<uint32_t>(x,object+0x30,source);
        put<uint32_t>(x,source,0x534946);put<uint32_t>(x,source+0x14,0x464b4e41);put<uint32_t>(x,source+0x18,0);
        put<uint32_t>(x,buffer,base);put<uint32_t>(x,buffer+16,0x1000);put<uint32_t>(x,0x35de60,0x1c0000);
        put<int16_t>(x,0x1c3c6c,640);put<int16_t>(x,0x1c3c6e,448);put<uint32_t>(x,0x35dd80,0x1d0000);
        put<uint32_t>(x,0x1d0004,0x1e0000);put<uint32_t>(x,0x1e007c,0);
        std::memcpy(y.memory.flat(),ram,bozo::kFlatBytes);
        R5900Context p{},q{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=0x1234000000000000ull+seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        p.r[4].ud[0]=object;p.r[5].ud[0]=seed;p.r[6].ud[0]=seed;p.r[7].ud[0]=(seed%4==2)?0x80402010:0;
        p.r[29].ud[0]=0x1fff000;p.r[31].ud[0]=0x170000;q=p;
        old.fonts.begin(&x,old.textures,0x1ffefa0,object,p.r[7].uw[0]);run(a,p,0x29af80);old.fonts.finish(&x,0x1ffefa0);
        run(b,q,0x29af80);check(!std::memcmp(ram,y.memory.flat(),bozo::kFlatBytes));p.cycleLimit=q.cycleLimit;check(!std::memcmp(&p,&q,sizeof(p)));++cases;
        check(old.fonts.outstanding()==0 && now.fonts.outstanding()==0);
        const auto snapshot=now.fonts.font(object);check(snapshot && snapshot->atlas->key==vp2s::TextureKey{0x464b4e41,0});
        check(snapshot->atlas->picture()->rgba[255]==0x80332211);
        if(snapshot->background){bool rejected=false;try{now.fonts.publish(&y,1,object);}catch(const std::runtime_error &){rejected=true;}check(rejected);continue;}
        p.r[4].ud[0]=buffer;p.r[5].ud[0]=0;p.r[16].ud[0]=p.r[17].ud[0]=object;p.r[29].ud[0]=0x1fff000;p.r[31].ud[0]=0x170000;q=p;
        old.fonts.publish(&x,0x1ffefd0,object);
        check(vp2s::nativeDebugFontInstruction(y.memory.flat(),&q,&y,seed&8?0x3d93f4:0x362328)==false);
        run(a,p,0x120eb0);old.fonts.returnFromFlush(0x1ffefd0);run(b,q,0x120eb0);
        check(!std::memcmp(ram,y.memory.flat(),bozo::kFlatBytes));
        check(p.cycles==q.cycles+old.enqueues*8 && p.checkpoints==q.checkpoints+old.enqueues);
        p.cycles-=old.enqueues*8;p.checkpoints-=old.enqueues;p.cycleLimit=q.cycleLimit;p.r[2]=q.r[2];check(!std::memcmp(&p,&q,sizeof(p)));
        check(old.fonts.outstanding()==0 && now.fonts.outstanding()==0 && now.enqueues==0);++cases;
        const R5900Context before=q;const auto memory=std::vector<uint8_t>(y.memory.flat(),y.memory.flat()+bozo::kFlatBytes);
        bool unowned=false;try{vp2s::nativeDebugFontInstruction(y.memory.flat(),&q,&y,0x120f4c);}catch(const std::runtime_error &){unowned=true;}
        check(unowned && !std::memcmp(&before,&q,sizeof(q)) && !std::memcmp(memory.data(),y.memory.flat(),memory.size()));
        put<uint32_t>(y,buffer+8,snapshot->end+16);bool rejected=false;
        try{now.fonts.publish(&y,1,object);}catch(const std::runtime_error &){rejected=true;}check(rejected);
        check(snapshot->atlas->picture()->rgba[0]==0x80332211);
    }
    for(unsigned seed=0;seed<64;++seed){
        old.setups={};now.setups={};old.enqueues=now.enqueues=0;old.allocationFailure=now.allocationFailure=false;
        now.pause=(seed&1)!=0;
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x2b,bozo::kFlatBytes);
        put<uint32_t>(x,0x35df10,0x30180000);put<uint32_t>(x,0x35df20,0x1000);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{},q{};p.r[29].ud[0]=0x1fff000;p.r[31].ud[0]=0x170000;q=p;
        run(a,p,0x29c990);old.setups.finish(0x1fff000-0x20-0x30);run(b,q,0x29c990);
        check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        check(old.enqueues==1 && now.enqueues==0 && !old.setups.outstanding() && !now.setups.outstanding());
        check(p.cycles==q.cycles+8 && p.checkpoints==q.checkpoints+1);p.cycles-=8;--p.checkpoints;p.cycleLimit=q.cycleLimit;p.r[2]=q.r[2];
        check(!std::memcmp(&p,&q,sizeof(p)));++cases;
        // A different writer range cannot be admitted as the fixed defaults.
        put<uint32_t>(y,0x35df18,value<uint32_t>(y,0x35df18)+16);bool rejected=false;
        try{now.setups.beginDefaults(&y,1);}catch(const std::runtime_error &){rejected=true;}check(rejected);
    }
    for(unsigned seed=0;seed<64;++seed){
        old.setups={};now.setups={};old.enqueues=now.enqueues=0;old.pause=false;now.pause=seed&4;
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x53,bozo::kFlatBytes);
        constexpr uint32_t object=0x180000,buffer=0x190000,base=0x1a0000,scene=0x1b0000,secondary=0x1c0000;
        const uint32_t layer=seed&1;
        put(x,object,uint32_t(0x1653d0));put(x,object+0x98,object+0x90);put(x,object+0x114,buffer);
        put(x,object+0x63,uint8_t(1));put(x,object+0x64,uint8_t(0));put(x,object+0x65,uint8_t(layer));
        put(x,buffer,base);put(x,buffer+12,base+0x40);put(x,buffer+8,base+0x60);put(x,buffer+16,uint32_t(0x100));
        put(x,0x35ddf8,scene);put(x,scene+0x40a8+layer*4,object);put(x,scene+0x40dc+layer,uint8_t(1));
        put(x,scene+0x4088,seed&2?secondary:0u);put(x,secondary+0x3e6a,uint8_t(0));
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        p.r[4].ud[0]=object;p.r[29].ud[0]=0x1fff000;p.r[31].ud[0]=0x170000;auto q=p;
        run(a,p,0x2075a0);run(b,q,0x2075a0);
        check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        check(old.enqueues==1 && now.enqueues==0 && !old.setups.outstanding() && !now.setups.outstanding());
        check(*y.memory.hostPointer(object+0x64,1)==0 && *y.memory.hostPointer(object+0x62,1)==1 && *y.memory.hostPointer(scene+0x40dc+layer,1)==0);
        check(value<uint32_t>(y,buffer+8)==base+0x70 && value<uint32_t>(y,buffer+12)==base+0x70);
        check(p.cycles==q.cycles+8 && p.checkpoints==q.checkpoints+1);p.cycles-=8;--p.checkpoints;p.cycleLimit=q.cycleLimit;p.r[2]=q.r[2];
        if(std::memcmp(&p,&q,sizeof(p))){const auto *pb=reinterpret_cast<const uint8_t *>(&p),*qb=reinterpret_cast<const uint8_t *>(&q);
            for(unsigned i=0;i<sizeof(p);++i)if(pb[i]!=qb[i]){std::fprintf(stderr,"empty CPU seed%u diff byte%u=%02x/%02x pc%x/%x\n",seed,i,pb[i],qb[i],p.pc,q.pc);break;}}
        check(!std::memcmp(&p,&q,sizeof(p)));++cases;
        // Failed acceptance cannot clear the game's pending byte or flush.
        put(y,object+0x64,uint8_t(1));put(y,scene+0x40dc+layer,uint8_t(1));put(y,scene+0x40a8+layer*4,object);
        put(y,buffer+12,base+0x40);put(y,buffer+8,base+0x70);
        bool rejected=false;try{now.setups.beginEmptyChildren(&y,1,object);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && !now.setups.outstanding() && *y.memory.hostPointer(object+0x64,1)==1 && value<uint32_t>(y,buffer+8)==base+0x70);
        put(y,buffer+8,base+0x60);put(y,object+0x98,uint32_t(0x1d0000));rejected=false;
        try{now.setups.beginEmptyChildren(&y,1,object);}catch(const std::runtime_error &){rejected=true;}check(rejected && !now.setups.outstanding());
    }
    for(unsigned seed=0;seed<64;++seed){
        old.rects={};now.rects={};old.draws.clear();now.draws.clear();old.enqueues=now.enqueues=0;old.pause=false;now.pause=seed&4;
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x53,bozo::kFlatBytes);
        constexpr uint32_t object=0x180000,buffer=0x190000,base=0x1a0000,scene=0x1b0000,source=0x1d0000;
        const uint32_t layer=seed&1,width=std::array<uint32_t,3>{320,512,640}[seed%3],height=std::array<uint32_t,3>{224,448,512}[seed%3];
        const auto colour=((seed*0x070301u)&0xffffff)|(std::array<uint32_t,4>{0,1,64,128}[seed%4]<<24);
        put(x,object,uint32_t(0x161230));put(x,object+0x90,uint32_t(0x1612ac));put(x,object+0xc4,buffer);
        put(x,object+0xa0,colour);put(x,object+0xc0,uint32_t(0xffff00));put(x,object+0xf1,uint8_t(seed));put(x,object+0xf2,uint32_t(0));put(x,object+0x65,uint8_t(layer));
        // Bits 0..1 and 5..7 of +0xf5 do not reach the draw (0xe0 seen on the ship route).
        put(x,object+0xf5,uint8_t(seed%4==3?0xe0|((seed>>2)&3):(seed>>2)&3));
        put(x,buffer,base);put(x,buffer+16,uint32_t(0x100));put(x,0x35de60,source);put(x,source+0x3c6c,int16_t(width));put(x,source+0x3c6e,int16_t(height));
        // Actual offline-admitted table entry for the flat fade's mode0:
        // ordinary source-alpha blend. Only compiled CPU code reads it here.
        put(x,0x3140f0,uint64_t(0x44));put(x,0x35ddf8,scene);put(x,scene+0x40a8+layer*4,object);put(x,scene+0x40dc+layer,uint8_t(1));put(x,scene+0x4088,uint32_t(0));
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        p.r[4].ud[0]=object;p.r[29].ud[0]=0x1fff000;p.r[31].ud[0]=0x170000;auto q=p;
        run(a,p,0x3d3660);run(b,q,0x3d3660);p.cycleLimit=q.cycleLimit;
        check(!std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        p.r[4].ud[0]=object;p.r[31].ud[0]=0x170000;q=p;
        run(a,p,0x3d3620);run(b,q,0x3d3620);
        check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        check(old.enqueues==1 && !now.enqueues && !old.rects.outstanding() && !now.rects.outstanding() && old.draws.size()==1 && now.draws.size()==1);
        const auto held=now.draws.front();check(held.object==object && held.colour==colour && held.depth==0xffff00+seed && held.right==int32_t(width) && held.bottom==int32_t(height));
        const auto vertices=vp2s::nativeScreenRectangleVertices(held);
        check(vertices[0].x==2048-float(width)/2 && vertices[0].y==2048-float(height)/2 && vertices[4].x==2048+float(width)/2 && vertices[4].y==2048+float(height)/2 && vertices[0].rgba==colour);
        check(*y.memory.hostPointer(object+0x64,1)==0 && *y.memory.hostPointer(object+0x62,1)==1 && *y.memory.hostPointer(scene+0x40dc+layer,1)==0);
        check(value<uint32_t>(y,buffer+8)==base+0xb0 && value<uint32_t>(y,buffer+12)==base+0xb0);
        check(p.cycles==q.cycles+8 && p.checkpoints==q.checkpoints+1);p.cycles-=8;--p.checkpoints;p.cycleLimit=q.cycleLimit;p.r[2]=q.r[2];check(!std::memcmp(&p,&q,sizeof(p)));++cases;
        put(y,object+0x64,uint8_t(1));put(y,scene+0x40dc+layer,uint8_t(1));put(y,scene+0x40a8+layer*4,object);put(y,buffer+12,base+0x40);put(y,buffer+8,base+0xa0);
        const auto flags=*y.memory.hostPointer(object+0xf5,1);
        for(const auto [offset,bits]:std::array<std::pair<uint32_t,uint32_t>,5>{{{0xf2u,1u},{0xf3u,1u},{0xf5u,4u},{0xf5u,8u},{0xf5u,16u}}}){
            const auto saved=*y.memory.hostPointer(object+offset,1);put(y,object+offset,uint8_t(saved|bits));bool rejected=false;
            try{now.rects.beginFade(&y,1,object);}catch(const std::runtime_error &){rejected=true;}check(rejected && !now.rects.outstanding());put(y,object+offset,saved);
        }
        check(*y.memory.hostPointer(object+0xf5,1)==flags);
        put(y,buffer+8,base+0xb0);bool extra=false;
        try{now.rects.beginFade(&y,1,object);}catch(const std::runtime_error &){extra=true;}check(extra && !now.rects.outstanding());put(y,buffer+8,base+0xa0);
        now.rects.beginFade(&y,1,object);bool rejected=false;
        try{now.rects.releaseFade(object);}catch(const std::runtime_error &){rejected=true;}check(rejected);
        now.rects.cancel(1);now.rects.releaseFade(object);rejected=false;
        try{now.rects.beginFade(&y,1,object);}catch(const std::runtime_error &){rejected=true;}check(rejected && !now.rects.outstanding() && held.colour==colour);
    }
    // A fade its builder left unbuilt (+63 = 0: skip, pause gate or hidden in +f5): the submit
    // draws nothing, 120eb0 leaves the untouched writer alone, and the completion returns the object.
    {auto &u=b.runtime();vp2s::NativeScreenRects rects;constexpr uint32_t object=0x180000,buffer=0x190000;
        put(u,object,uint32_t(0x161230));put(u,object+0x63,uint8_t(0));put(u,object+0xc4,buffer);put(u,buffer+8,uint32_t(0x1a00b0));put(u,buffer+12,uint32_t(0x1a00b0));
        check(rects.skipFade(&u,7,object) && rects.skippedFades()==1 && !rects.outstanding() && rects.complete(7)==object && !rects.skippedFades());
        bool rejected=false;try{rects.skipFade(&u,7,object);rects.skipFade(&u,7,object);}catch(const std::runtime_error &){rejected=true;}check(rejected);rects.complete(7);
        put(u,buffer+8,uint32_t(0x1a00c0));rejected=false;try{rects.skipFade(&u,8,object);}catch(const std::runtime_error &){rejected=true;}check(rejected && !rects.skippedFades());
        put(u,buffer+8,uint32_t(0x1a00b0));put(u,object+0xc4,uint32_t(0));rejected=false;try{rects.skipFade(&u,8,object);}catch(const std::runtime_error &){rejected=true;}check(rejected);
        put(u,object+0xc4,buffer);put(u,object+0x63,uint8_t(1));check(!rects.skipFade(&u,8,object));
        put(u,object+0x63,uint8_t(0));put(u,object,uint32_t(0x15bc00));check(!rects.skipFade(&u,8,object));++cases;}
    // Fade mode 4 (wipe): the game's own packet (Logun camp menu, frame ~38600):
    // from 0, to 0x80000000, p = 0/20, 512x512, depth ffff64, ramp {0, .4, .7, .9, 1}.
    {vp2s::NativeFadeWipeInputs in{.from=0,.to=0x80000000u,.time=0,.duration=0x41a00000u,.width=512,.height=512,.depth=0xffff64,
        .ramp={0,0x3ecccccdu,0x3f333333u,0x3f666666u,0x3f800000u}};
        const auto s=vp2s::nativeFadeWipe(in);
        const uint32_t x[12]={0x9000,0xb000,0x9730,0xb730,0x9e60,0xbe60,0xa590,0xc590,0xacc0,0xccc0,0xecc0,0xecc0};
        const uint32_t alpha[5]={0,0x33,0x59,0x73,0x80};
        for(unsigned k=0;k<12;++k)check(s[k].x*16==float(x[k]) && s[k].y==float(k&1?0x7000/16:0x9000/16) && s[k].z==float(0xffff64));
        for(unsigned i=0;i<5;++i)check(s[2*i].rgba==alpha[i]<<24 && s[2*i+1].rgba==alpha[i]<<24);check(s[10].rgba==0x80000000u && s[11].rgba==0x80000000u);
        // The other direction (to's alpha byte not 0x80): from the left past w + 0x1cc, edges 115 to the left, colours swapped.
        in.from=0x80102030u;in.to=0x40000000u;in.time=0x41200000u;const auto r=vp2s::nativeFadeWipe(in);
        // travel = 10/20 * (512 + 460 + 512) = 742: a0 = 1792 + (972 - 742) = 2022, then -115 each; last pair -w / -2w.
        check(r[0].x==2022 && r[1].x==2534 && r[2].x==1907 && r[8].x==1562 && r[10].x==1050 && r[11].x==1050);
        check((r[0].rgba&0xffffff)==0 && (r[0].rgba>>24)==0 && r[10].rgba==0x80102030u && (r[2].rgba>>24)==uint32_t(int32_t(0.4f*0x40)));
        // Clamped to [0, 4095]: a travel far past the screen pins every edge at 0.
        in.time=0x447a0000u;const auto c=vp2s::nativeFadeWipe(in);for(const auto &v:c)check(v.x==0);++cases;}
    {auto &u=b.runtime();vp2s::NativeScreenRects rects;constexpr uint32_t object=0x180000,buffer=0x190000,base=0x1a0000,scene=0x1b0000,source=0x1d0000;
        put(u,object,uint32_t(0x161230));put(u,object+0x63,uint8_t(1));put(u,object+0x64,uint8_t(1));put(u,object+0x65,uint8_t(0));put(u,object+0xf1,uint8_t(0));
        put(u,object+0xf2,uint8_t(0));put(u,object+0xf3,uint8_t(4));put(u,object+0xf4,uint8_t(0));put(u,object+0xf5,uint8_t(1));
        put(u,object+0x98,uint32_t(0));put(u,object+0x9c,uint32_t(0x80000000u));put(u,object+0xa4,0.f);put(u,object+0xa8,20.f);
        put(u,object+0xb8,int32_t(512));put(u,object+0xbc,int32_t(512));put(u,object+0xc0,uint32_t(0xffff64));put(u,object+0xc4,buffer);
        put(u,buffer,base);put(u,buffer+12,base+0x40);put(u,buffer+8,base+0x140);put(u,buffer+16,uint32_t(0x100));
        put(u,0x35de60,source);put(u,source+0x3c6c,int16_t(512));put(u,source+0x3c6e,int16_t(512));put(u,0x3140f0,uint64_t(0x44));
        put(u,0x35ddf8,scene);put(u,scene+0x40a8,object);put(u,scene+0x40dc,uint8_t(1));
        const float ramp[5]={0,.4f,.7f,.9f,1};for(unsigned i=0;i<5;++i)put(u,0x488430+4*i,ramp[i]);
        const auto rect=rects.beginFade(&u,9,object);check(rect.wipe && rect.strip[0].x==2304 && rects.active(9));rects.cancel(9);
        const auto rejects=[&](uint32_t at,auto bad,auto good){put(u,at,bad);bool r=false;try{rects.beginFade(&u,9,object);}catch(const std::runtime_error &){r=true;}check(r && !rects.active(9));put(u,at,good);};
        rejects(buffer+8,uint32_t(base+0x150),uint32_t(base+0x140));   // unowned writer content
        rejects(object+0xf4,uint8_t(1),uint8_t(0));                    // another blend
        rejects(object+0xf5,uint8_t(0x10),uint8_t(1));                 // hidden
        rejects(object+0xb8,int32_t(500),int32_t(512));                // not the screen box
        rejects(scene+0x40a8,uint32_t(0),object);                      // not the pending owner
        check(rects.beginFade(&u,9,object).wipe);rects.cancel(9);++cases;
        // Mode 1 (3d38e0): a two-colour Gouraud quad from +d0..+ec (each +0x800), colours a/b
        // alternating (+f2 = 0: a = +a0, b = +98; else a = +9c, b = +a0), depth +c0 + +f1; 128 bytes.
        put(u,object+0xf3,uint8_t(1));put(u,buffer+8,base+0xc0);put(u,object+0xf1,uint8_t(3));
        put(u,object+0x98,uint32_t(0x11223344u));put(u,object+0x9c,uint32_t(0x55667788u));put(u,object+0xa0,uint32_t(0x80000000u));
        const int32_t xy[8]={-256,-224,256,-224,-256,224,256,224};for(unsigned i=0;i<8;++i)put(u,object+0xd0+4*i,xy[i]);
        {const auto q=rects.beginFade(&u,9,object);check(q.wipe && q.stripVertices==4 && rects.active(9));rects.cancel(9);
            // Strip order (+d0, +d8, +e8, +e0): (-256,-224) a, (256,-224) b, (256,224) a, (-256,224) b.
            const float ex[4]={1792,2304,2304,1792},ey[4]={1824,1824,2272,2272};
            for(unsigned i=0;i<4;++i)check(q.strip[i].x==ex[i] && q.strip[i].y==ey[i] && q.strip[i].z==float(0xffff67) && q.strip[i].rgba==((i&1)?0x11223344u:0x80000000u));}
        put(u,object+0xf2,uint8_t(1));
        {const auto q=rects.beginFade(&u,9,object);check(q.strip[0].rgba==0x55667788u && q.strip[1].rgba==0x80000000u);rects.cancel(9);put(u,object+0xf2,uint8_t(0));}
        rejects(buffer+8,uint32_t(base+0x140),uint32_t(base+0xc0));    // the wipe's length: unowned content
        rejects(object+0xf4,uint8_t(1),uint8_t(0));                    // another blend
        rejects(object+0xf5,uint8_t(0x10),uint8_t(1));                 // hidden
        rejects(object+0xd0,int32_t(0x800),int32_t(-256));             // past the GS range
        rejects(scene+0x40a8,uint32_t(0),object);                      // not the pending owner
        put(u,object+0xf3,uint8_t(4));put(u,buffer+8,base+0x140);put(u,object+0xf1,uint8_t(0));++cases;}
    auto &rt=b.runtime();put<uint32_t>(rt,0x35df10,0x30180000);put<uint32_t>(rt,0x35df20,0x1000);put<uint32_t>(rt,0x35df1c,0x30180040);put<uint32_t>(rt,0x35df18,0x30180100);
    now.setups.beginDefaults(&rt,1);bool duplicate=false;
    try{now.setups.beginDefaults(&rt,1);}catch(const std::runtime_error &){duplicate=true;}check(duplicate);
    R5900Context c{};c.r[29].uw[0]=1;c.r[17].uw[0]=0x35df10;c.r[16].uw[0]=0x30180110;c.r[2].uw[0]=0x30180040;c.r[5].uw[0]=2;c.r[8].uw[0]=13;
    put<uint32_t>(rt,0x35df18,0x30180110);bool changed=false;
    try{now.setups.retire(&rt,&c);}catch(const std::runtime_error &){changed=true;}check(changed);
    put<uint32_t>(rt,0x35df18,0x30180100);now.setups.retire(&rt,&c);now.setups.finish(1);check(!now.setups.outstanding());
    std::printf("%u generated font/default/empty/fade CPU-flush comparisons; whole scalar builders/completions, vertices, aliases, nested resumes and unsupported-draw/lifetime rejection passed\n",cases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

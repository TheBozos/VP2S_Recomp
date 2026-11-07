#include "native_debug_font.h"
#include "native_assets.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "font_init_original_functions.h"
#include "font_init_adapted_functions.h"
#include "font_flush_original_functions.h"
#include "font_flush_adapted_functions.h"
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
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::DebugFontOwner,vp2s::TextureOwner {
    vp2s::NativeDebugFonts fonts;vp2s::NativeTextures textures;
    bool adapted=false,pause=false,allocationFailure=false;unsigned pauses=0,enqueues=0;
    explicit Host(const std::filesystem::path &p):textures(p,p/"manifest.txt"){}
    vp2s::NativeDebugFonts &nativeDebugFonts()override{return fonts;}
    vp2s::NativeTextures &nativeTextures()override{return textures;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
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
        else if(pc==0x1131c0){check(!adapted);fonts.retire(&rt,c);++enqueues;c->r[2].sd[0]=7;}
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
    while(true){try{fn(d.runtime().memory.flat(),&c,&d.runtime());break;}catch(const Pause &){}
        check(++tries<8);auto entry=d.runtime().functions.resumeEntry(c.pc);check(entry);fn=entry->resume;
    }
}
}
int main(int argc,char **argv){try{
    check(argc==2);Host old(argv[1]),now(argv[1]);now.adapted=true;bozo::ee::Dispatcher a(old),b(now);
    std::vector<BozoFunctionEntry> oldFunctions(font_init_original_function_table,font_init_original_function_table+font_init_original_function_count);
    oldFunctions.insert(oldFunctions.end(),font_flush_original_function_table,font_flush_original_function_table+font_flush_original_function_count);
    std::vector<BozoResumeEntry> oldResume(font_init_original_resume_table,font_init_original_resume_table+font_init_original_resume_count);
    oldResume.insert(oldResume.end(),font_flush_original_resume_table,font_flush_original_resume_table+font_flush_original_resume_count);
    std::vector<BozoFunctionEntry> newFunctions(font_init_adapted_function_table,font_init_adapted_function_table+font_init_adapted_function_count);
    newFunctions.insert(newFunctions.end(),font_flush_adapted_function_table,font_flush_adapted_function_table+font_flush_adapted_function_count);
    std::vector<BozoResumeEntry> newResume(font_init_adapted_resume_table,font_init_adapted_resume_table+font_init_adapted_resume_count);
    newResume.insert(newResume.end(),font_flush_adapted_resume_table,font_flush_adapted_resume_table+font_flush_adapted_resume_count);
    for(auto *table:{&oldFunctions,&newFunctions})std::sort(table->begin(),table->end(),[](const auto &x,const auto &y){return x.address<y.address;});
    for(auto *table:{&oldResume,&newResume})std::sort(table->begin(),table->end(),[](const auto &x,const auto &y){return x.start<y.start;});
    a.setFunctions(oldFunctions.data(),oldFunctions.size(),oldResume.data(),oldResume.size());
    b.setFunctions(newFunctions.data(),newFunctions.size(),newResume.data(),newResume.size());a.runtime().port=&old;b.runtime().port=&now;
    for(auto *d:{&a,&b})for(uint32_t pc:{0x112860u,0x120fa0u,0x1134c0u,0x120f70u,0x1131c0u})d->bindLibrary(pc);
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
        check(snapshot->atlas->picture()->rgba[255]==0xff332211);
        if(snapshot->background){bool rejected=false;try{now.fonts.publish(&y,1,object);}catch(const std::runtime_error &){rejected=true;}check(rejected);continue;}
        p.r[4].ud[0]=buffer;p.r[5].ud[0]=0;p.r[17].ud[0]=object;p.r[29].ud[0]=0x1fff000;p.r[31].ud[0]=0x170000;q=p;
        old.fonts.publish(&x,0x1ffefd0,object);
        check(vp2s::nativeDebugFontInstruction(y.memory.flat(),&q,&y,0x362328)==false);
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
        check(snapshot->atlas->picture()->rgba[0]==0xff332211);
    }
    std::printf("%u generated font setup/CPU-flush comparisons; alias, failed allocation, resumed initialization and unsupported-draw rejection passed\n",cases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

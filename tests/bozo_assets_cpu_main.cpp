#include "native_assets.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "assets_original_functions.h"
#include "assets_adapted_functions.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <source_location>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){
    if(!v)throw std::runtime_error("asset CPU assertion at "+std::to_string(at.line()));
}
struct Pause{};
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::AssetOwner {
    vp2s::NativeAssets assets;bool track=false,pause=false;unsigned pauses=0;
    vp2s::NativeAssets &nativeAssets()override{return assets;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        const auto known=vp2s::nativeAssetInstruction(r,c,rt,pc);check(known.has_value());return *known;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        auto &memory=d.runtime().memory;
        if(pc==0x132bf0){
            auto *source=memory.hostPointer(c->r[4].uw[0],3);check(source);
            const uint32_t magic=c->r[5].uw[0];
            c->r[2].sd[0]=source[0]==0xfd?1:(magic==0x14f600 || magic==0x14f6d0?0:1);
        }else if(pc==0x132c88){
            const auto to=c->r[4].uw[0],from=c->r[5].uw[0],n=c->r[6].uw[0];
            std::memmove(memory.hostPointer(vp2s::NativeAssets::address(to),n),memory.hostPointer(vp2s::NativeAssets::address(from),n),n);
            if(track)assets.copy(to,from,n,c->r[31].uw[0]==0x1059a8?c->r[17].uw[0]:0);
            c->r[2].sd[0]=static_cast<int32_t>(to);
        }else throw std::runtime_error("fixture expected raw decode only");
        c->pc=c->r[31].uw[0];if(pause){++pauses;throw Pause{};}
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
void word(uint8_t *r,uint32_t at,uint32_t value){std::memcpy(r+at,&value,4);}
void run(bozo::ee::Dispatcher &d,R5900Context &c,uint32_t pc){
    auto fn=d.runtime().functions.generated(pc);check(fn);unsigned attempts=0;
    while(true){try{fn(d.runtime().memory.flat(),&c,&d.runtime());break;}catch(const Pause &){}
        check(++attempts<16);auto entry=d.runtime().functions.resumeEntry(c.pc);check(entry);fn=entry->resume;
    }
}
}
int main(){try{
    Host original,adapted;adapted.track=true;
    bozo::ee::Dispatcher a(original),b(adapted);
    a.setFunctions(assets_original_function_table,assets_original_function_count,assets_original_resume_table,assets_original_resume_count);
    b.setFunctions(assets_adapted_function_table,assets_adapted_function_count,assets_adapted_resume_table,assets_adapted_resume_count);
    a.runtime().port=&original;b.runtime().port=&adapted;
    for(auto *d:{&a,&b})for(uint32_t pc:{0x132bf0u,0x132c88u})d->bindLibrary(pc);
    unsigned cases=0;
    for(unsigned seed=0;seed<96;++seed){
        original.assets={};adapted.assets={};adapted.pause=(seed&4)!=0;adapted.pauses=0;
        auto *x=a.runtime().memory.flat(),*y=b.runtime().memory.flat();std::memset(x,0x7b,bozo::kFlatBytes);
        const uint32_t part=seed%3,size=64+(seed%8)*16,chunk=0x180000+part*0x400;
        const bool fail=(seed%5)==0,async=(seed&1)!=0;
        for(unsigned i=0;i<3;++i){word(x,0x180000+i*0x400,0x005a4946);word(x,0x180008+i*0x400,size);word(x,0x18000c+i*0x400,0x400);}
        if(fail){x[chunk]=0xfd;if(async)x[0x180000]=0xfd;}
        std::memcpy(y,x,bozo::kFlatBytes);adapted.assets.discRead(0x180000,2,100+seed);
        R5900Context old{},now{};
        for(unsigned i=1;i<32;++i){old.r[i].ud[0]=0x1234000000000000ull+seed*32+i;old.r[i].ud[1]=~old.r[i].ud[0];}
        old.r[4].ud[0]=0x190000;old.r[5].ud[0]=async?0x1b0000:0x180000;
        old.r[6].ud[0]=async?0x180000:0x301a0000;old.r[7].ud[0]=async?0x301a0000:part;old.r[8].ud[0]=part;
        old.r[29].ud[0]=0x1fff000;old.r[31].ud[0]=0x170000;now=old;
        const auto compare=[&]{check(!std::memcmp(x,y,bozo::kFlatBytes));
            check(old.cycleLimit==now.cycleLimit || adapted.pauses);old.cycleLimit=now.cycleLimit;
            check(!std::memcmp(&old,&now,sizeof(old)));check(adapted.assets.pendingCount()==0);++cases;};
        const uint32_t entry=async?0x105a30:0x102650;run(a,old,entry);run(b,now,entry);compare();
        check(old.r[2].uw[0]==unsigned(!fail));
        if(fail){check(!adapted.assets.origin(0x1a0000) && adapted.assets.jobCount()==0);continue;}
        if(async){check(!adapted.assets.origin(0x1a0000) && adapted.assets.jobCount()==1);
            // The actual raw worker performs two copies and reports completion
            // only on the second; observers retain ownership across every pause.
            for(unsigned slice=0;slice<2;++slice){
                old.r[4].ud[0]=0x190000;old.r[5].ud[0]=0x1b0000;old.r[6].ud[0]=size/2;
                old.r[29].ud[0]=0x1fff000;old.r[31].ud[0]=0x170000;now=old;
                run(a,old,0x105940);run(b,now,0x105940);compare();
                check(old.r[2].uw[0]==slice);
                if(!slice)check(!adapted.assets.origin(0x1a0000));
            }
        }
        const auto found=adapted.assets.origin(0x1a0000+size-1);
        check(found==vp2s::AssetOrigin{vp2s::AssetOrigin::Kind::Part,uint64_t(100+seed)*2048,part,size-1});
        check(adapted.assets.jobCount()==0);
    }
    for(unsigned seed=0;seed<32;++seed){
        original.assets={};adapted.assets={};adapted.pause=false;
        auto *x=a.runtime().memory.flat(),*y=b.runtime().memory.flat();std::memset(x,0,bozo::kFlatBytes);
        const uint32_t header=0x180000,size=64+seed*16,manager=0x190000,head=0x1a0000,next=0x1b0000,prev=0x1c0000;
        word(x,header,size);x[header+5]=1;word(x,header+8,next);word(x,header+12,prev);word(x,header+16,manager);
        x[next+5]=x[prev+5]=1;word(x,manager+0x20,head);word(x,head+0x14,head);word(x,head+0x18,head);
        std::memcpy(y,x,bozo::kFlatBytes);adapted.assets.discRead(header,1,77);
        R5900Context old{},now{};old.r[4].ud[0]=header+0x20;old.r[29].ud[0]=0x1fff000;old.r[31].ud[0]=0x170000;now=old;
        run(a,old,0x118230);run(b,now,0x118230);
        check(!std::memcmp(x,y,bozo::kFlatBytes) && !std::memcmp(&old,&now,sizeof(old)));
        check(adapted.assets.origin(header+0x1f) && !adapted.assets.origin(header+0x20));
        check(!adapted.assets.origin(header+size-1) && adapted.assets.origin(header+size));++cases;
    }
    std::printf("%u generated decompression/free CPU/RAM comparisons; failure, delayed slices and resumed native provenance passed\n",cases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

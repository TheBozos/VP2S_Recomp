#include "native_textures.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "texture_original_functions.h"
#include "texture_adapted_functions.h"
#include <cstdio>
#include <cstring>
#include <set>
#include <source_location>
#include <stdexcept>
namespace {
void check(bool b,std::source_location at=std::source_location::current()){
    if(!b)throw std::runtime_error("native texture ownership assertion failed at "+std::to_string(at.line()));
}
template<class F>void rejects(F f){bool failed=false;try{f();}catch(const std::runtime_error &){failed=true;}check(failed);}
void word(uint8_t *r,uint32_t p,uint32_t v){std::memcpy(r+p,&v,4);}
struct Pause{};
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::TextureOwner {
    vp2s::NativeTextures textures;bool original=false;unsigned allocations=0,failAt=0,pauses=0,freeCalls=0;
    std::set<uint32_t> owned;bool yieldCalls=false;
    explicit Host(const char *path):textures(path,std::filesystem::path(path)/"manifest.txt"){}
    vp2s::NativeTextures &nativeTextures()override{return textures;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{return vp2s::nativeTextureInstruction(r,c,rt,pc);}
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void release(uint32_t pointer){check(pointer && owned.erase(pointer)==1);++freeCalls;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        bool pause=false;
        if(pc==0x100ab0){
            check(c->r[4].uw[0]==0x70);++allocations;
            const uint32_t p=allocations==failAt?0:0x280000+allocations*0x80;
            if(p)check(owned.insert(p).second);c->r[2].sd[0]=p;pause=yieldCalls;
        }else if(pc==0x100b30){check(!original);release(c->r[4].uw[0]);c->r[2].sd[0]=0;pause=yieldCalls;}
        else if(pc==0x113630)c->r[2].sd[0]=7;
        else if(pc==0x1137f0)c->r[2].sd[0]=11;
        else if(pc==0x137460)c->r[2].sd[0]=0;
        else if(pc==0x1228d0){
            // Pure descriptor construction, observed equally on both sides.
            // Do not interpret the source or execute any upload.
            for(unsigned i=0;i<8;++i)word(d.runtime().memory.ram(),c->r[4].uw[0]+i*4,c->r[5+i].uw[0]);
            c->r[2].sd[0]=0;
        }else if(pc==0x1131c0){
            check(original && c->r[5].uw[0]==1 && c->r[6].uw[0]==1 && c->r[8].uw[0]==0 && c->r[9].uw[0]==0);
            const bool palette=c->r[31].uw[0]==0x29cd88;
            const bool fixed=c->r[31].uw[0]==0x29cc0c;
            check(palette || fixed || c->r[31].uw[0]==0x29cf20);
            if(fixed)textures.publishStaticPalette(7);
            else textures.publish(c->r[29].uw[0],palette?7:11,palette);
            // Reference ownership observer releases the transient descriptor;
            // there is no DMA engine, status response or completion interrupt.
            release(c->r[7].uw[0]);c->r[2].sd[0]=1;
        }else throw std::runtime_error("unexpected image leaf");
        c->pc=c->r[31].uw[0];if(pause){++pauses;throw Pause{};}
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device write");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
void run(bozo::ee::Dispatcher &d,R5900Context &c,uint32_t entry=0x29cc30){
    auto fn=d.runtime().functions.generated(entry);
    for(unsigned i=0;i<10;++i){
        try{fn(d.runtime().memory.flat(),&c,&d.runtime());check(c.pc==0x170000);return;}catch(const Pause &){}
        auto entry=d.runtime().functions.resumeEntry(c.pc);check(entry);fn=entry->resume;
    }
    throw std::runtime_error("image fixture did not return");
}
}
int main(int argc,char **argv){try{
    check(argc==2);Host oldHost(argv[1]),newHost(argv[1]);oldHost.original=true;
    bozo::ee::Dispatcher old(oldHost),now(newHost);
    old.setFunctions(texture_original_function_table,texture_original_function_count,texture_original_resume_table,texture_original_resume_count);
    now.setFunctions(texture_adapted_function_table,texture_adapted_function_count,texture_adapted_resume_table,texture_adapted_resume_count);
    old.runtime().port=&oldHost;now.runtime().port=&newHost;
    for(auto *d:{&old,&now})for(uint32_t pc:{0x100ab0u,0x100b30u,0x113630u,0x1137f0u,0x1228d0u,0x137460u,0x1131c0u,0x133f70u})d->bindLibrary(pc);
    for(unsigned seed=0;seed<64;++seed){
        const unsigned fail=seed%3,palette=(seed>>2)&1,index=1+(seed&1);
        oldHost.allocations=newHost.allocations=oldHost.freeCalls=newHost.freeCalls=0;oldHost.failAt=newHost.failAt=fail;
        newHost.yieldCalls=(seed&8)!=0;newHost.pauses=0;
        auto *a=old.runtime().memory.flat(),*b=now.runtime().memory.flat();std::memset(a,0,bozo::kFlatBytes);
        word(a,0x180000,0x534946);word(a,0x180010,palette?0x1009000:0x9000);word(a,0x180014,0x12345678);word(a,0x180018,index);word(a,0x18001c,0x100);
        word(a,0x180110,16|(16u<<16));word(a,0x180118,0x100);word(a,0x18011c,0x14);
        word(a,0x180020,1024);word(a,0x180024,0x200);word(a,0x180028,1);
        word(a,0x35dd80,0x190000);word(a,0x35dd78,0x1a0000);word(a,0x190008,0x1b0000);word(a,0x190004,0x1c0000);
        word(a,0x1b0000+7*4,0x1000);word(a,0x1c0000+11*4,0x2000);std::memcpy(b,a,bozo::kFlatBytes);
        R5900Context x{},y{};
        for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x1234000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        std::memset(x.vu0_vf,0xd3,sizeof(x.vu0_vf));x.r[4].ud[0]=0x180000;x.r[5].ud[0]=31;x.r[29].ud[0]=0x1fff000;x.r[31].ud[0]=0x170000;y=x;
        const auto publications=newHost.textures.publicationCount();run(old,x);run(now,y);
        check(oldHost.owned.empty() && newHost.owned.empty() && oldHost.freeCalls==newHost.freeCalls && oldHost.allocations==newHost.allocations);
        check(!oldHost.textures.outstanding() && !newHost.textures.outstanding() && !std::memcmp(a,b,bozo::kFlatBytes));
        // Original queue arguments a0/a2 become the compiled-free arguments.
        // Resume dispatch can refresh its host slice limit; cycles/checkpoint
        // totals, architectural fields, FPU and VU sentinels still match.
        x.r[4]=y.r[4];x.r[6]=y.r[6];
        check(x.cycleLimit==y.cycleLimit || newHost.pauses>0);x.cycleLimit=y.cycleLimit;
        check(!std::memcmp(&x,&y,sizeof(x)));
        check(newHost.textures.publicationCount()-publications==newHost.freeCalls);
        if(newHost.textures.publicationCount()>publications){
            auto asset=palette?newHost.textures.paletteAsset(7):newHost.textures.pixelAsset(11);check(asset && asset->key.index==index);
            auto picture=asset->picture(3);check(picture->rgba.size()==256);
            for(unsigned i=0;i<256;++i){const unsigned k=48+(i+index)%16;check(picture->rgba[i]==((k*7%256)|(index*30<<8)|(k<<16)|0xff000000));}
        }
    }
    for(unsigned seed=0;seed<32;++seed){
        oldHost.allocations=newHost.allocations=oldHost.freeCalls=newHost.freeCalls=0;oldHost.failAt=newHost.failAt=seed&1;
        newHost.yieldCalls=(seed&2)!=0;newHost.pauses=0;
        auto *a=old.runtime().memory.flat(),*b=now.runtime().memory.flat();std::memset(a,0x75,bozo::kFlatBytes);
        word(a,0x35dd80,0x190000);word(a,0x35dd78,0x1a0000);word(a,0x190008,0x1b0000);word(a,0x1b0000+7*4,0x1000);
        std::memcpy(b,a,bozo::kFlatBytes);R5900Context x{},y{};
        x.r[4].ud[0]=0x314200;x.r[5].ud[0]=30;x.r[29].ud[0]=0x1fff000;x.r[31].ud[0]=0x170000;y=x;
        auto count=newHost.textures.publicationCount();run(old,x,0x29cb20);run(now,y,0x29cb20);
        check(oldHost.owned.empty() && newHost.owned.empty() && oldHost.freeCalls==newHost.freeCalls && newHost.freeCalls==1-(seed&1));
        check(newHost.textures.publicationCount()-count==newHost.freeCalls && !std::memcmp(a,b,bozo::kFlatBytes));
        x.r[4]=y.r[4];x.r[6]=y.r[6];check(x.cycleLimit==y.cycleLimit || newHost.pauses>0);x.cycleLimit=y.cycleLimit;
        check(!std::memcmp(&x,&y,sizeof(x)));
        if(!(seed&1)){auto asset=newHost.textures.paletteAsset(7);check(asset && asset->key==vp2s::kStaticScenePalette);
            auto picture=asset->picture();for(unsigned i=0;i<256;++i)check(picture->rgba[i]==(0xff000000|i*0x010101));}
    }
    // Active invocation identity, stack reuse, immutable publication and slot
    // rebinding are independent of transient descriptor/game-heap lifetimes.
    auto &t=newHost.textures;t.begin(1,{0x12345678,1});t.begin(2,{0x12345678,2});t.publish(1,20,false);
    auto first=t.pixelAsset(20);t.publish(2,20,false);check(first->key.index==1 && t.pixelAsset(20)->key.index==2);
    rejects([&]{t.begin(1,{0x12345678,2});});rejects([&]{t.publish(1,20,false);});t.finish(1);t.finish(2);
    t.begin(1,{0x12345678,2});t.finish(1);rejects([&]{t.finish(1);});rejects([&]{t.begin(3,{0,0});});check(t.outstanding()==0);
    std::puts("64 generated image + 32 static palette CPU/RAM/ownership comparisons, allocation failures and resumed frees; immutable per-host textures and slot/stack reuse passed; no upload consumer");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

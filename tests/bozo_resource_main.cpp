#include "native_textures.h"
#include "native_assets.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "resource_resource_reference_functions.h"
#include "resource_resource_adapted_functions.h"
#include "resource_helpers_reference_functions.h"
#include "resource_helpers_adapted_functions.h"
#include <algorithm>
#include <functional>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <set>
#include <source_location>
#include <stdexcept>
namespace {
constexpr uint32_t object=0x190000,source=0x180000,buffer=0x280000,scene=0x1d0000,queue=0x1a0000;
void check(bool ok,std::source_location at=std::source_location::current()){
    if(!ok)throw std::runtime_error("resource assertion at "+std::to_string(at.line()));
}
template<class T>void put(Runtime &r,uint32_t at,T v){std::memcpy(r.memory.hostPointer(at,sizeof(v)),&v,sizeof(v));}
template<class T>T get(Runtime &r,uint32_t at){T v;std::memcpy(&v,r.memory.hostPointer(at,sizeof(v)),sizeof(v));return v;}
struct Pause{};
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::TextureOwner,vp2s::AssetOwner{
    vp2s::NativeTextures textures;vp2s::NativeAssets assets;
    std::vector<uint8_t> decoded;std::set<uint32_t> allocations;
    bool adapted=false,pause=false,success=true;unsigned waits=0,frees=0,notifications=0,index=1;
    Host(const char *path,const char *fis):textures(path,std::filesystem::path(path)/"manifest.txt"){
        std::ifstream f(fis,std::ios::binary);decoded.assign(std::istreambuf_iterator<char>(f),{});check(decoded.size()==67072);
    }
    vp2s::NativeTextures &nativeTextures()override{return textures;}
    vp2s::NativeAssets &nativeAssets()override{return assets;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(adapted){auto v=vp2s::nativeFixedImageInstruction(c,rt,pc);check(v.has_value());return *v;}
        // Reference observes the retained scalar producer and invokes the same
        // real software callback after immutable native publication. It never
        // supplies a device response or interprets a packed descriptor.
        if(pc==0x447af0)textures.releaseImage(object);
        else if(pc==0x447b84)textures.publishImage(object,{0x12345678,index});
        else if(pc==0x447d60 || pc==0x447a48)c->r[4].sd[0]=object;
        else throw std::runtime_error("unexpected resource observer");
        return false;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        auto &r=d.runtime();
        if(pc==0x100cb0)c->r[2].sd[0]=0x1b0000;
        else if(pc==0x118520){
            check(c->r[4].uw[0]==0x1b0000 && c->r[5].uw[0]==decoded.size() && c->r[6].uw[0]==128);
            check(allocations.insert(buffer).second);c->r[2].sd[0]=buffer;
        }else if(pc==0x100b30){
            const auto p=c->r[4].uw[0];check(allocations.erase(p)==1);++frees;
            assets.overwrite(p,decoded.size());c->r[2].sd[0]=0;
        }else if(pc==0x102650){
            check(c->r[4].uw[0]==0x1b0000 && c->r[5].uw[0]==source && c->r[6].uw[0]==buffer && c->r[7].uw[0]==0);
            // Decoder itself has separate original-CPU qualification. This
            // bounded contract copies its actual decoded fixture and records
            // provenance only on successful completion.
            assets.begin(c->r[29].uw[0],source,buffer,0);
            if(success){std::memcpy(r.memory.hostPointer(buffer,decoded.size()),decoded.data(),decoded.size());
                put(r,buffer+0x14,0x12345678u);put(r,buffer+0x18,index);}
            assets.finish(c->r[29].uw[0],success,decoded.size());c->r[2].sd[0]=success;
        }else if(pc==0x137460)c->r[2].sd[0]=0;
        else if(pc==0x112ba0){check(c->r[4].uw[0]==queue && c->r[5].uw[0]==0);++waits;c->r[2].sd[0]=0;}
        else if(pc==0x1130f0){check(c->r[4].uw[0]==queue);++notifications;c->r[2].sd[0]=0;}
        else throw std::runtime_error("unexpected resource leaf "+std::to_string(pc));
        c->pc=c->r[31].uw[0];if(pause)throw Pause{};
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device write");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
void run(bozo::ee::Dispatcher &d,R5900Context &c,uint32_t pc){
    auto fn=d.runtime().functions.generated(pc);check(fn);
    for(unsigned i=0;i<32;++i){try{fn(d.runtime().memory.flat(),&c,&d.runtime());if(c.pc==0x170000)return;}catch(const Pause &){}
        auto e=d.runtime().functions.resumeEntry(c.pc);check(e);fn=e->resume;}
    throw std::runtime_error("resource did not return");
}
template<class F>void rejects(F f){bool bad=false;try{f();}catch(const std::runtime_error &){bad=true;}check(bad);}
}
int main(int argc,char **argv){try{
    check(argc==3);Host old(argv[1],argv[2]),now(argv[1],argv[2]);now.adapted=true;
    bozo::ee::Dispatcher a(old),b(now);
    std::vector<BozoFunctionEntry> af(resource_resource_reference_function_table,resource_resource_reference_function_table+resource_resource_reference_function_count);
    af.insert(af.end(),resource_helpers_reference_function_table,resource_helpers_reference_function_table+resource_helpers_reference_function_count);
    std::vector<BozoResumeEntry> ar(resource_resource_reference_resume_table,resource_resource_reference_resume_table+resource_resource_reference_resume_count);
    ar.insert(ar.end(),resource_helpers_reference_resume_table,resource_helpers_reference_resume_table+resource_helpers_reference_resume_count);
    std::vector<BozoFunctionEntry> bf(resource_resource_adapted_function_table,resource_resource_adapted_function_table+resource_resource_adapted_function_count);
    bf.insert(bf.end(),resource_helpers_adapted_function_table,resource_helpers_adapted_function_table+resource_helpers_adapted_function_count);
    std::vector<BozoResumeEntry> br(resource_resource_adapted_resume_table,resource_resource_adapted_resume_table+resource_resource_adapted_resume_count);
    br.insert(br.end(),resource_helpers_adapted_resume_table,resource_helpers_adapted_resume_table+resource_helpers_adapted_resume_count);
    for(auto *v:{&af,&bf})std::sort(v->begin(),v->end(),[](auto x,auto y){return x.address<y.address;});
    for(auto *v:{&ar,&br})std::sort(v->begin(),v->end(),[](auto x,auto y){return x.start<y.start;});
    a.setFunctions(af.data(),af.size(),ar.data(),ar.size());b.setFunctions(bf.data(),bf.size(),br.data(),br.size());
    a.runtime().port=&old;b.runtime().port=&now;
    for(auto *d:{&a,&b})for(uint32_t pc:{0x100cb0u,0x118520u,0x100b30u,0x102650u,0x137460u,0x112ba0u,0x1130f0u})d->bindLibrary(pc);
    for(unsigned seed=0;seed<64;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0,bozo::kFlatBytes);
        for(auto *h:{&old,&now}){h->assets={};h->textures.releaseImage(object);h->allocations={buffer+0x20000};h->waits=h->frees=h->notifications=0;h->success=seed%4!=0;h->index=1+(seed&1);}
        now.pause=seed&8;
        put(x,object,0x165b40u);put(x,object+0x90,0x165bbcu);put(x,object+0x228,buffer+0x20000);
        put(x,source+8,uint32_t(old.decoded.size()));put(x,0x35dd98,0x1b0000u);put(x,0x35dd78,queue);put(x,0x35ddf8,scene);
        put(x,0x48c580,0x545444u);put(x,0x48c590,7u);put(x,scene+0x40a8,object);put<uint8_t>(x,scene+0x40dc,1);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        for(auto *h:{&old,&now}){h->assets.discRead(source,1,100);h->textures.publishImage(object,{0x12345678,2});}
        auto held=now.textures.imageAsset(object);
        R5900Context p{},q{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=0x1234000000000000ull+seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0xd3,sizeof(p.vu0_vf));p.r[4].ud[0]=object;p.r[5].ud[0]=source;
        p.r[29].ud[0]=seed&2?0x70003f00:0x1fff000;p.r[31].ud[0]=0x170000;q=p;
        run(a,p,0x447af0);run(b,q,0x447af0);p.cycleLimit=q.cycleLimit;
        check(!std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        check(old.allocations==now.allocations && now.allocations==std::set<uint32_t>{buffer} && now.frees==1);
        check(old.waits==now.waits && now.waits==(now.success?2:0));check(!now.assets.pendingCount());
        check(held->key.index==2 && held->texels[0]==2);
        if(now.success){
            auto image=now.textures.imageAsset(object);check(image && image->key.index==now.index);
            for(auto *r:{&x,&y}){put(*r,scene+0x40a8,object);put<uint8_t>(*r,scene+0x40dc,1);}
            p={};p.r[4].ud[0]=object;p.r[29].ud[0]=0x1fff000;p.r[31].ud[0]=0x170000;q=p;
            const auto notifications=now.notifications;run(a,p,0x447a10);run(b,q,0x447a10);p.cycleLimit=q.cycleLimit;
            check(!std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
            check(now.notifications==notifications+1 && get<uint8_t>(y,scene+0x40dc)==0);
            R5900Context dying{};dying.r[4].ud[0]=object;const auto saved=dying;
            check(vp2s::nativeFixedImageInstruction(&dying,&y,0x447f50)==false && !now.textures.imageAsset(object));
            check(!std::memcmp(&dying,&saved,sizeof(dying)) && image->texels[0]==now.index);
        }else check(!now.textures.imageAsset(object));
    }
    // Unknown origin, unfinished/offset file, key and queue ownership fail
    // before CPU/RAM mutation. Freed source bytes cannot authorize reuse.
    auto &r=b.runtime();put(r,object+0x228,buffer);put(r,buffer,0x534946u);put(r,buffer+0x14,0x12345678u);put(r,buffer+0x18,1u);
    R5900Context c{};c.r[17].ud[0]=object;c.r[16].ud[0]=buffer;
    now.assets.overwrite(buffer,now.decoded.size());
    auto rejectUnchanged=[&](uint32_t pc){auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);
        rejects([&]{vp2s::nativeFixedImageInstruction(&c,&r,pc);});check(!std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
    rejectUnchanged(0x447b84);now.assets.discRead(source,1,100);now.assets.begin(1,source,buffer,1);now.assets.finish(1,true,now.decoded.size());rejectUnchanged(0x447b84);
    now.assets.begin(1,source,buffer,0);now.assets.finish(1,true,now.decoded.size());put(r,buffer+0x18,999u);rejectUnchanged(0x447b84);
    put(r,buffer+0x18,1u);check(vp2s::nativeFixedImageInstruction(&c,&r,0x447b84)==false);
    c.r[9].ud[0]=object+0x90;c.r[4].ud[0]=queue+4;c.r[5].ud[0]=3;c.r[7].ud[0]=object+0xa0;rejectUnchanged(0x447a48);
    // Texture uploader 165630 (mod003 draw 4431c0, enqueue delay slot 443238):
    // every registered FIS image is published under its entry; a0 becomes the
    // uploader for the real completion 443190. Entry 0 holds the decoded file.
    {constexpr uint32_t uploader=0x1e0000;unsigned cases=0;
        put(r,uploader,0x165630u);put(r,uploader+0x90,0x1656acu);put(r,uploader+0x12a8,2u);
        put(r,uploader+0x1228,uploader+0xa0);put(r,uploader+0x122c,uploader+0xd0);put(r,uploader+0x10b8,buffer);put(r,uploader+0x10b8+0x18,0u);
        const auto submit=[&](int index){R5900Context u{};u.pc=0x4431c0;u.r[18].ud[0]=uploader;u.r[17].ud[0]=uint64_t(index);u.r[16].ud[0]=uploader+4*uint32_t(index);
            u.r[4].ud[0]=queue;u.r[5].ud[0]=3;u.r[6].ud[0]=0;u.r[7].ud[0]=get<uint32_t>(r,uploader+0x1228+4*uint32_t(index));u.r[8].ud[0]=0x55;
            u.r[9].ud[0]=index==1?uploader+0x90:0;return u;};
        for(int index:{0,1}){auto u=submit(index);if(index)u.pc=0x44323c;check(vp2s::nativeTextureUploaderInstruction(&u,&r,0x443238)==true);
            check(u.r[4].uw[0]==uploader && u.r[8].ud[0]==0);
            const auto image=now.textures.imageAsset(uploader+0x10a8);check(image && image->key.index==1 && !now.textures.imageAsset(uploader+0x10a8+0x18));++cases;}
        const auto rejectUploader=[&](const std::function<void(R5900Context &)> &spoil){auto u=submit(0);spoil(u);
            rejects([&]{vp2s::nativeTextureUploaderInstruction(&u,&r,0x443238);});++cases;};
        rejectUploader([](R5900Context &u){u.pc=0x443190;});                                   // not the draw method
        rejectUploader([](R5900Context &u){u.r[9].ud[0]=uploader+0x90;});                      // owner on a non-last chain
        rejectUploader([](R5900Context &u){u.r[7].ud[0]+=0x10;});                              // another chain
        rejectUploader([](R5900Context &u){u.r[5].ud[0]=2;});                                  // another job type
        rejectUploader([&](R5900Context &){put(r,uploader,0x165b40u);});put(r,uploader,0x165630u);   // another class
        rejectUploader([&](R5900Context &){put(r,buffer,0x12345u);});put(r,buffer,0x534946u);        // not a FIS file
        now.assets.overwrite(buffer,now.decoded.size());rejectUploader([](R5900Context &){});           // no decoded provenance
        check(vp2s::nativeTextureUploaderInstruction(nullptr,&r,0x443200)==std::nullopt);
        std::printf("%u texture-uploader publications and rejections\n",cases);}
    std::puts("64 FIS load/free/reload/failure + original draw/scene completion CPU/RAM cases, resumable contracts, immutable assets and provenance rejection; no device consumer");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

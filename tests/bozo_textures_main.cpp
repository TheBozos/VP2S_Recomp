#include "native_textures.h"
#include "native_scene.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "texture_original_functions.h"
#include "texture_adapted_functions.h"
#include <cstdio>
#include <filesystem>
#include <fstream>
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
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::TextureOwner,vp2s::SceneOwner {
    vp2s::NativeScene scene;unsigned completions=0;
    vp2s::NativeScene &nativeScene()override{return scene;}
    vp2s::NativeTextures textures;bool original=false;unsigned allocations=0,failAt=0,pauses=0,freeCalls=0;
    std::set<uint32_t> owned;bool yieldCalls=false;
    explicit Host(const char *path):textures(path,std::filesystem::path(path)/"manifest.txt"){}
    vp2s::NativeTextures &nativeTextures()override{return textures;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(auto handled=vp2s::nativeFixedImageInstruction(c,rt,pc))return *handled;
        return vp2s::nativeSceneInstruction(r,c,rt,pc) || vp2s::nativeTextureInstruction(r,c,rt,pc);
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void release(uint32_t pointer){check(pointer && owned.erase(pointer)==1);++freeCalls;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        bool pause=false;
        if(pc==0x1130f0){
            check(c->r[4].uw[0]==0x1a0000);++completions;
            ++*d.runtime().memory.hostPointer(0x1a0000,1);c->r[2].ud[0]=0;c->pc=c->r[31].uw[0];pause=yieldCalls;
        }else if(pc==0x100ab0){
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
        try{fn(d.runtime().memory.flat(),&c,&d.runtime());if(c.pc==0x170000)return;}catch(const Pause &){}
        auto entry=d.runtime().functions.resumeEntry(c.pc);check(entry);fn=entry->resume;
    }
    throw std::runtime_error("image fixture did not return");
}
}
int main(int argc,char **argv){try{
    check(argc==2);
    {   // Keys whose disc copies differ (each enemy pack's BM01:1) resolve by the origin of the loaded
        // file's texture header; copies with the same conversion share one synthetic key.
        namespace fs=std::filesystem;const auto dir=fs::temp_directory_path()/"vp2s_texture_provenance";fs::create_directories(dir);
        const auto tex=[&](const char *name,uint8_t fill){std::ofstream o(dir/name,std::ios::binary);const uint32_t h[3]={4,4,0x14};o.write(reinterpret_cast<const char *>(h),12);
            std::vector<char> body(16+1024,0);std::fill(body.begin(),body.begin()+16,char(fill));o.write(body.data(),std::streamsize(body.size()));};
        tex("c_00000000000000aa.tex",1);tex("c_00000000000000bb.tex",2);tex("c_00000000000000cc.tex",3);
        const auto manifest=[&](const std::string &body){std::ofstream(dir/"m.txt")<<"VPNT1\n"<<body;return dir/"m.txt";};
        vp2s::NativeTextures t(dir,manifest("11111111 2 c_00000000000000aa.tex\n31304d42 1 c_00000000000000bb.tex 00ed10d830_000 12dd00\n"
            "31304d42 1 c_00000000000000cc.tex 00ed1f0030_000 126c80\n31304d42 1 c_00000000000000bb.tex 00ed2fd830_000 110c80\n"));
        const auto at=[](uint64_t disc,uint32_t offset){return vp2s::AssetOrigin{vp2s::AssetOrigin::Kind::Part,disc,0,offset};};
        const vp2s::TextureKey bm{0x31304d42,1},plain{0x11111111,2};
        check(t.resolve(plain,at(1,2))==plain);
        const auto a=t.resolve(bm,at(0xed10d830,0x12dd00)),b=t.resolve(bm,at(0xed1f0030,0x126c80)),c=t.resolve(bm,at(0xed2fd830,0x110c80));
        check(a==vp2s::TextureKey{0x31304d42,0x80000001u} && b==vp2s::TextureKey{0x31304d42,0x80010001u} && c==a);
        check(t.load(a)->texels[0]==2 && t.load(b)->texels[0]==3 && t.load(a)->key==a);
        const auto refused=[&](auto f){try{f();}catch(const std::runtime_error &){return true;}return false;};
        check(refused([&]{t.resolve(bm,at(0xed10d830,0x12dd10));}) && refused([&]{t.load(bm);}));                    // another offset; the bare key
        check(refused([&]{t.resolve(bm,vp2s::AssetOrigin{vp2s::AssetOrigin::Kind::Disc,0xed10d830,0,0x12dd00});})); // not a decoded part
        check(refused([&]{t.resolve(vp2s::TextureKey{0x31304d42,2},at(0xed10d830,0x12dd00));}));                    // another key's copy there
        check(refused([&]{vp2s::NativeTextures bad(dir,manifest("31304d42 1 c_00000000000000aa.tex\n31304d42 1 c_00000000000000bb.tex 00ed10d830_000 12dd00\n"));}));
        check(refused([&]{vp2s::NativeTextures bad(dir,manifest("31304d42 1 c_00000000000000bb.tex 00ed10d830_00 12dd00\n"));}));
        // A model's texture (the manager's key and TEX0): a plain key loads as itself; a key whose copies
        // differ is the copy published at TEX0's pixel block, with the palette at its CBP.
        check(t.modelImage(plain,0)->key==plain);
        const uint64_t tex0=0x3a40|uint64_t(0x3a80)<<37;
        check(refused([&]{t.modelImage(bm,tex0);}));                                       // nothing at block 3a40
        check(t.modelImage(bm,tex0,std::pair{uint64_t(0xed1f0030),0u})->key==b);           // the copy in the mesh's own part
        check(t.modelImage(bm,tex0,std::pair{uint64_t(0xed10d830),0u})->key==a);
        check(refused([&]{t.modelImage(bm,tex0,std::pair{uint64_t(0xed10d831),0u});}));    // no copy there, nothing at the block
        t.publishBlock(0x3a40,false,b);t.publishBlock(0x3a80,true,a);
        check(refused([&]{t.modelImage(bm,0x3a40|uint64_t(0x3b00)<<37);}));               // no palette at its CBP
        check(refused([&]{t.modelImage(vp2s::TextureKey{0x31304d42,2},tex0);}));          // another key's copy there
        const auto image=t.modelImage(bm,tex0);
        check(image->key==b && image->texels[0]==3 && image->palette==t.load(a)->palette && t.modelImage(bm,tex0)==image);
        fs::remove_all(dir);
    }
    Host oldHost(argv[1]),newHost(argv[1]);oldHost.original=true;
    bozo::ee::Dispatcher old(oldHost),now(newHost);
    old.setFunctions(texture_original_function_table,texture_original_function_count,texture_original_resume_table,texture_original_resume_count);
    now.setFunctions(texture_adapted_function_table,texture_adapted_function_count,texture_adapted_resume_table,texture_adapted_resume_count);
    old.runtime().port=&oldHost;now.runtime().port=&newHost;
    for(auto *d:{&old,&now})for(uint32_t pc:{0x100ab0u,0x100b30u,0x113630u,0x1137f0u,0x1228d0u,0x137460u,0x1131c0u,0x133f70u,0x1130f0u})d->bindLibrary(pc);
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
            for(unsigned i=0;i<256;++i){const unsigned k=48+(i+index)%16;check(picture->rgba[i]==((k*7%256)|(index*30<<8)|(k<<16)|0x80000000));}
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
            auto picture=asset->picture();for(unsigned i=0;i<256;++i)check(picture->rgba[i]==(0x80000000|i*0x010101));}
    }
    // Active invocation identity, stack reuse, immutable publication and slot
    // rebinding are independent of transient descriptor/game-heap lifetimes.
    auto &t=newHost.textures;t.begin(1,{0x12345678,1});t.begin(2,{0x12345678,2});t.publish(1,20,false);
    auto first=t.pixelAsset(20);t.publish(2,20,false);check(first->key.index==1 && t.pixelAsset(20)->key.index==2);
    rejects([&]{t.begin(1,{0x12345678,2});});rejects([&]{t.publish(1,20,false);});t.finish(1);t.finish(2);
    t.begin(1,{0x12345678,2});t.finish(1);rejects([&]{t.finish(1);});rejects([&]{t.begin(3,{0,0});});check(t.outstanding()==0);
    for(unsigned seed=0;seed<64;++seed){
        auto &a=old.runtime(),&b=now.runtime();std::memset(a.memory.flat(),0,bozo::kFlatBytes);
        constexpr uint32_t scene=0x1d0000,image=0x190000,fis=0x180000;
        const auto layer=(seed>>2)&1;
        auto *ram=a.memory.flat();word(ram,image,0x15a6f0);word(ram,image+0x90,0x15a76c);word(ram,image+0xc0,fis);
        // Original transport inputs are poisoned; only the FIS identity is native input.
        word(ram,image+0xb8,0xdeadbeef);word(ram,image+0xc4,0xbaadf00d);word(ram,image+0xd4,0xfefefefe);
        word(ram,fis,0x534946);word(ram,fis+0x14,0x12345678);word(ram,fis+0x18,1+(seed&1));
        word(ram,0x35ddf8,scene);word(ram,0x35dd78,0x1a0000);
        word(ram,scene+0x40a8+layer*4,image);ram[scene+0x40dc+layer]=1;ram[image+0x65]=layer;
        std::memcpy(b.memory.flat(),ram,bozo::kFlatBytes);
        R5900Context x{},y{};x.r[4].ud[0]=image;x.r[29].ud[0]=seed&2?0x70003f00:0x1fff000;x.r[31].ud[0]=0x170000;y=x;
        oldHost.yieldCalls=false;newHost.yieldCalls=seed&32;const auto count=newHost.completions;
        run(old,x,0x2f36f0);run(now,y,0x2f36f0);x.cycleLimit=y.cycleLimit;
        check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(ram,b.memory.flat(),bozo::kFlatBytes));
        check(newHost.completions==count+1-layer && b.memory.flat()[image+0x64]==0 && b.memory.flat()[image+0x62]==1 && b.memory.flat()[scene+0x40dc+layer]==0);
        auto held=newHost.textures.imageAsset(image);check(held && held->key.index==1+(seed&1));
        std::memset(b.memory.flat()+fis,0x5a,0x100);check(held->texels[0]==1+(seed&1));
        R5900Context dying{};dying.pc=0x2ea9a0;dying.r[4].ud[0]=image;const auto saved=dying;
        check(vp2s::nativeFixedImageInstruction(&dying,&b,0x2ea9a0)==false && !newHost.textures.imageAsset(image));
        check(!std::memcmp(&dying,&saved,sizeof(dying)) && held->texels[0]==1+(seed&1));
    }
    {auto &b=now.runtime();constexpr uint32_t scene=0x1d0000,image=0x190000,fis=0x180000;
        auto *ram=b.memory.flat();word(ram,fis,0x534946);word(ram,fis+0x14,0x12345678);word(ram,fis+0x18,999);
        ram[scene+0x40dd]=1;ram[image+0x64]=1;
        R5900Context c{};c.pc=0x2f36f0;c.r[4].ud[0]=image;c.r[5].ud[0]=image+0x90;const auto saved=c;
        std::vector<uint8_t> before(ram,ram+bozo::kFlatBytes);
        rejects([&]{vp2s::nativeFixedImageInstruction(&c,&b,0x2f3714);});
        check(!std::memcmp(&c,&saved,sizeof(c)) && !std::memcmp(before.data(),ram,before.size()) && !newHost.textures.imageAsset(image));
        auto &t=newHost.textures;t.publishImage(image,{0x12345678,1});auto held=t.imageAsset(image);
        rejects([&]{t.publishImage(image,{0x12345678,999});});check(t.imageAsset(image)==held);
        t.publishImage(image,{0x12345678,2});check(held->key.index==1 && t.imageAsset(image)->key.index==2);
        t.releaseImage(image);check(!t.imageAsset(image) && held->texels[0]==1 && oldHost.textures.imageAsset(image)->key.index==2);
    }
    {   // Battle images by graphics-memory block, pixels and palettes apart.
        auto &t=newHost.textures;t.publishBlock(0x3a00,false,{0x12345678,1});t.publishBlock(0x3a00,true,{0x12345678,2});
        check(t.blockAsset(0x3a00,false)->key.index==1 && t.blockAsset(0x3a00,true)->key.index==2 && !t.blockAsset(0x3a04,false));
        rejects([&]{t.publishBlock(0,false,{0x12345678,1});});rejects([&]{t.publishBlock(0x4000,true,{0x12345678,1});});
        // A TEX0's image: one file under both blocks, or the pixels with another file's palette (built once).
        check(!t.blockImage(0x3a00,0x3a04));t.publishBlock(0x3b00,false,{0x12345678,2});t.publishBlock(0x3b04,true,{0x12345678,2});
        check(t.blockImage(0x3b00,0x3b04)==t.blockAsset(0x3b00,false));
        {   const auto pixels=t.blockAsset(0x3a00,false),colours=t.blockAsset(0x3a00,true);
            if(pixels->format==0x13 || pixels->format==0x14){const auto mixed=t.blockImage(0x3a00,0x3a00);
                check(mixed && mixed!=pixels && mixed->texels==pixels->texels && mixed->palette==colours->palette && mixed->identity!=pixels->identity &&
                      t.blockImage(0x3a00,0x3a00)==mixed);}
            else rejects([&]{t.blockImage(0x3a00,0x3a00);}); }
        // The game's deferred free (1205b0): ring of 0x400 pointers, write index +0, read index +4.
        auto &b=now.runtime();constexpr uint32_t list=0x1d3710;std::memset(b.memory.hostPointer(list,0x1008),0,0x1008);
        const auto word=[&](uint32_t at){uint32_t v;std::memcpy(&v,b.memory.hostPointer(at,4),4);return v;};
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(b.memory.hostPointer(at,4),&v,4);};
        vp2s::guestDeferredFree(&b,list,0x1a0000);check(word(list)==0);   // read == write: dropped, as the game drops it
        put(list+4,5);vp2s::guestDeferredFree(&b,list,0);check(word(list)==0);   // null: nothing
        vp2s::guestDeferredFree(&b,list,0x1a0000);check(word(list)==1 && word(list+8)==0x1a0000);
        put(list,0x3ff);vp2s::guestDeferredFree(&b,list,0x1a0100);check(word(list)==0 && word(list+8+4*0x3ff)==0x1a0100);   // wraps
        put(list,0x400);rejects([&]{vp2s::guestDeferredFree(&b,list,0x1a0200);});
    }
    std::puts("64 manager images + 32 palettes + 64 fixed-image producer/completion CPU/RAM cases; offline assets survive source release, image destructor invalidates association; no upload consumer");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

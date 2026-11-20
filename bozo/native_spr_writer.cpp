#include "native_spr_writer.h"
#include "native_models.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <format>
#include <stdexcept>
namespace vp2s {
namespace {
constexpr uint32_t kWriterClass=0x152ac0,kScratch=0x70000000,kScratchBytes=0x4000,kHalfBytes=0xc00;
template<class T>T value(Runtime *rt,uint32_t at){
    const bool scratch=at>=kScratch && uint64_t(at)+sizeof(T)<=uint64_t(kScratch)+kScratchBytes;
    if(!rt || (!scratch && ((at&0x1fffffff)<0x100000 || (at&0x1fffffff)+sizeof(T)>0x2000000)))throw std::runtime_error("native scratchpad writer input outside owned memory");
    const auto *p=rt->memory.hostPointer(at,sizeof(T));if(!p)throw std::runtime_error("native scratchpad writer input unmapped");
    T v;std::memcpy(&v,p,sizeof(T));return v;
}
uint32_t writer(Runtime *rt,uint32_t object){
    // 174f90: the battle's subclass (mod028), same fields; its +d0 = 1 kick is 4d75f0's.
    if(object<0x100000 || (value<uint32_t>(rt,object+4)!=kWriterClass && value<uint32_t>(rt,object+4)!=0x174f90))
        throw std::runtime_error("native scratchpad writer escaped class 152ac0");
    return object;
}
void publish(Runtime *rt,NativeAssets &assets,uint32_t destination,uint32_t source,uint32_t bytes){
    if(!bytes)return;
    const auto physical=NativeAssets::address(destination);
    if(physical<0x100000 || uint64_t(physical)+bytes>0x2000000)throw std::runtime_error("native scratchpad writer destination outside EE RAM");
    auto *to=rt->memory.hostPointer(physical,bytes);const auto *from=rt->memory.hostPointer(source,bytes);
    if(!to || !from)throw std::runtime_error("native scratchpad writer section unmapped");
    assets.overwrite(physical,bytes);std::memmove(to,from,bytes);
}
}
std::vector<SprWriterSection> nativeSprWriterSections(Runtime *rt,uint32_t spr,uint32_t limitLow,uint32_t limitHigh,uint32_t cursor){
    if(spr<kScratch || spr>=kScratch+kScratchBytes || (spr&15) || spr+kHalfBytes>kScratch+kScratchBytes)throw std::runtime_error("native scratchpad writer chunk outside its scratchpad half");
    // Zero qwords are empty tags: the writer zero-fills the unused rest of each fixed
    // lane slot (up to 40 qwords) and an absent texture block (17). Only the half bounds them.
    std::vector<SprWriterSection> sections;uint32_t at=spr,expected=0;bool started=false;
    for(;;){
        if(at+16>spr+kHalfBytes)throw std::runtime_error("native scratchpad writer chunk runs past its half");
        const auto tag=value<uint64_t>(rt,at);
        if(!tag){at+=16;continue;}
        const unsigned id=unsigned(tag>>28)&7,qwords=unsigned(tag&0xffff);const auto address=uint32_t(tag>>32)&0x7fffffff;
        if(id==7){
            if(qwords || !started || expected!=cursor)
                throw std::runtime_error(std::format("native scratchpad writer chunk ends at {:x}, cursor {:x}",expected,cursor));
            return sections;
        }
        if(id!=1 || !qwords || (started && address!=expected) || address<limitLow || uint64_t(address)+16ull*qwords>limitHigh ||
           uint64_t(at)+16+16ull*qwords>uint64_t(spr)+kHalfBytes)
            throw std::runtime_error(std::format("native scratchpad writer section escaped its layout (tag {:016x} at {:x}, expected {:x})",tag,at,expected));
        sections.push_back({at+16,address,qwords});started=true;expected=address+16*qwords;at+=16+16*qwords;
    }
}
std::optional<bool> nativeSprWriterInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    // mod028 4d75f0 (the subclass's kick when +d0 is set) ends with mod001 201488..2014d0's
    // poll and fromSPR start 2d6a20 higher, word for word: map it onto those sites.
    if(pc>=0x20148c+0x2d6a20 && pc<=0x2014d0+0x2d6a20 && rt){
        uint32_t here=0,there=0;const auto *a=rt->memory.hostPointer(pc,4),*b=rt->memory.hostPointer(pc-0x2d6a20,4);
        if(a && b){std::memcpy(&here,a,4);std::memcpy(&there,b,4);}
        if(here!=there)return std::nullopt;   // not mod028's copy
        pc-=0x2d6a20;
    }
    const bool kick=pc==0x20148c || pc==0x2014b8 || pc==0x2014c0 || pc==0x2014c8 || pc==0x2014d0;
    const bool table=pc==0x200954 || pc==0x20099c || pc==0x2009a8 || pc==0x2009c4 || pc==0x2009cc || pc==0x2009d4;
    if(!kick && !table)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<AssetOwner *>(&d->host()):nullptr;
    if(!c || !owner)throw std::runtime_error("native scratchpad writer requires its asset owner");
    const auto object=writer(rt,c->r[kick?4:17].uw[0]);
    // The poll waits for the previous fromSPR transfer: natively it completed
    // when it was published, so its D_STAT bit reads as set.
    if(pc==0x20148c){c->r[3].sd[0]=0x100;return true;}
    if(pc==0x200954){c->r[2].sd[0]=0x100;return true;}
    if(pc==0x2014d0){
        const auto spr=value<uint32_t>(rt,object+0x80),low=value<uint32_t>(rt,object+0x78),cursor=value<uint32_t>(rt,object+0x7c);
        const auto high=uint64_t(low)+16ull*value<uint32_t>(rt,object+0xb8);
        if(c->r[3].uw[0]!=0x105 || c->r[6].uw[0]!=spr || high>0xffffffffull || cursor>high)
            throw std::runtime_error("native scratchpad writer kick escaped its chunk");
        auto *models=dynamic_cast<ModelOwner *>(&d->host());
        if(!models)throw std::runtime_error("native scratchpad writer requires its model owner");
        const auto sections=nativeSprWriterSections(rt,spr,low,uint32_t(high),cursor);
        std::vector<std::pair<uint32_t,uint32_t>> mesh;
        for(const auto &s:sections){publish(rt,owner->nativeAssets(),s.destination,s.source,16*s.qwords);mesh.push_back({s.destination,s.qwords});}
        // Each chunk is one mesh of the game's model format, drawn later through
        // the model the writer registers (25cab0): give the model path its record.
        models->nativeModels().publishGeneratedMesh(rt,mesh);
        return true;
    }
    if(pc==0x2009d4){
        const auto count=value<uint32_t>(rt,object+0x30),destination=c->r[16].uw[0],qwords=c->r[5].uw[0];
        if(c->r[3].uw[0]!=0x101 || destination!=value<uint32_t>(rt,object+0x28)+0x30 || qwords!=count*3 || !count || count>64 ||
           0x1800+16ull*qwords>kScratchBytes)
            throw std::runtime_error("native scratchpad writer descriptor transfer escaped its table");
        publish(rt,owner->nativeAssets(),destination,kScratch+0x1800,16*qwords);
        return true;
    }
    return true; // SADR / MADR / QWC / D_STAT stores: no transfer is started.
}
bool nativeSprWriterLibrary(R5900Context *c,Runtime *rt,uint32_t target){
    if(target!=0x123220 && target!=0x1237f0)return false;
    if(!c || !rt)return false;
    const auto ra=c->r[31].uw[0];
    if(target==0x123220 && (ra==0x20097c || ra==0x201810)){
        const auto context=value<uint32_t>(rt,0x35ddc8);
        if(c->r[4].uw[0]!=8 || value<uint8_t>(rt,context+0x1010)!=8)throw std::runtime_error("native scratchpad writer channel is not fromSPR");
        c->r[2].sd[0]=0x1000d000;c->pc=ra;return true;
    }
    if(target==0x1237f0 && (ra==0x20098c || ra==0x201820)){
        if(c->r[4].uw[0]!=0x1000d000 || c->r[5].uw[0] || c->r[6].uw[0])throw std::runtime_error("native scratchpad writer sync escaped fromSPR");
        c->r[2].sd[0]=0;c->pc=ra;return true;
    }
    return false;
}
}

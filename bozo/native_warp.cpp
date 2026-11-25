#include "native_warp.h"
#include <array>
#include <climits>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <format>
#include <fstream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
namespace vp2s {
namespace {
std::mutex requests;   // the F2 panel arms from the window's thread; the game's thread takes the request
std::optional<WarpRequest> pending;
std::optional<BattleRequest> battlePending;
// After the battle words the snippet holds kResumeTag and the interpreter's own pc: the handler
// leaves the pc on kSnippet + 12, whose fetch restores it. Guest RAM, so save states keep it.
constexpr uint32_t kResumeTag=0x42325056u;   // "VP2B"
// Below the game's image (0x100000): the PS2 kernel's RAM, unused by the native kernel.
constexpr uint32_t kSnippet=0x80000;
}
BattleRequest parseBattle(const std::string &text){
    std::vector<uint32_t> v;std::stringstream in(text);std::string item;
    while(std::getline(in,item,',')){size_t used=0;unsigned long long n=0;
        try{n=std::stoull(item,&used,0);}catch(const std::exception &){used=0;}
        if(item.empty() || item[0]=='-' || used!=item.size() || n>UINT32_MAX)throw std::runtime_error("malformed --battle value: "+item);v.push_back(uint32_t(n));}
    if(v.empty() || v.size()>3)throw std::runtime_error("--battle takes ID[,stage[,flags]]");
    if(v[0]>0xfffe)throw std::runtime_error("--battle id outside 0..0xfffe");
    BattleRequest r;r.battle=v[0];
    if(v.size()>1){if(v[1]>0x7fff)throw std::runtime_error("--battle stage outside 0..0x7fff");r.stage=v[1];}
    if(v.size()>2){if((v[2]&0xff)==0xff)throw std::runtime_error("--battle flags 0xff need an inline formation");r.flags=v[2];}
    return r;
}
void armNativeBattle(const BattleRequest &request){std::lock_guard lock(requests);battlePending=request;}
bool nativeBattlePending(){std::lock_guard lock(requests);return battlePending.has_value();}
std::array<uint32_t,3> battleInstruction(const BattleRequest &r){return {0x3802u,(r.stage<<16)|r.battle,r.flags};}
WarpRequest parseWarp(const std::string &text,const std::string &table){
    std::vector<int32_t> v;std::stringstream in(text);std::string item;
    while(std::getline(in,item,',')){size_t used=0;long long n=0;
        try{n=std::stoll(item,&used,0);}catch(const std::exception &){used=0;}
        if(item.empty() || used!=item.size() || n<INT32_MIN || n>INT32_MAX)throw std::runtime_error("malformed --warp value: "+item);v.push_back(int32_t(n));}
    if(v.size()!=1 && (v.size()<4 || v.size()>6))throw std::runtime_error("--warp takes MAP, or map,x,y,z[,facing[,entry]]");
    if(v[0]<0 || v[0]>0xffff)throw std::runtime_error("--warp map outside 0..65535");
    WarpRequest r;r.map=v[0];
    if(v.size()==1){
        // The map's most used entry point in the game's own scripts (tools/vp2_warps.py, sorted by uses).
        std::ifstream file(table);if(!file)throw std::runtime_error("--warp MAP needs "+table+" (tools/vp2_warps.py)");
        std::string line;
        while(std::getline(file,line)){if(line.empty() || line[0]=='#')continue;std::stringstream row(line);
            int32_t map,entry,x,y,z,facing;if(!(row>>map>>entry>>x>>y>>z>>facing))throw std::runtime_error("malformed warp table line: "+line);
            if(map==r.map){r.entry=entry;r.x=x;r.y=y;r.z=z;r.facing=facing;return r;}}
        throw std::runtime_error(std::format("map {} has no entry point in {}",r.map,table));
    }
    r.x=v[1];r.y=v[2];r.z=v[3];if(v.size()>4)r.facing=v[4];if(v.size()>5)r.entry=v[5];
    return r;
}
void armNativeWarp(const WarpRequest &request){std::lock_guard lock(requests);pending=request;}
bool nativeWarpPending(){std::lock_guard lock(requests);return pending.has_value();}
std::vector<WarpRow> loadWarpTable(const std::string &table){
    std::vector<WarpRow> rows;std::ifstream file(table);
    for(std::string line;std::getline(file,line);){if(line.empty() || line[0]=='#')continue;std::stringstream in(line);WarpRow r;
        if(in>>r.map>>r.entry>>r.x>>r.y>>r.z>>r.facing>>r.uses)rows.push_back(r);}
    return rows;
}
std::array<uint32_t,8> warpInstruction(const WarpRequest &r){
    return {0x3a07u,uint32_t(r.map),uint32_t(r.entry),uint32_t(r.x),uint32_t(r.y),uint32_t(r.z),uint32_t(r.camera),uint32_t(r.facing)};
}
// Step 3ee150 at the fetch "lw s1,0(a1)": a1 = the instruction, interpreter
// +604 = a1 + 4 (the handler's argument pointer), +18 = the script pc, which
// the step advances by 1 + count after a handler that returns 1 (op 0x3a does).
std::optional<bool> nativeWarpInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x3ee288)return std::nullopt;
    // Diagnostic: VP2S_BOZO_TRACE_SCRIPT_OP=HEX prints every instruction of that op any interpreter fetches (100: all ops).
    static const int traced=[]{const char *v=std::getenv("VP2S_BOZO_TRACE_SCRIPT_OP");return v?int(std::strtol(v,nullptr,16)):-1;}();
    if(traced>=0 && rt)if(const auto *word=rt->memory.hostPointer(c->r[5].uw[0],4)){
        uint32_t w;std::memcpy(&w,word,4);
        if(int((w>>8)&0xff)==traced || traced==0x100){   // 100: every op
            std::string args;const unsigned count=w&0xff;
            for(unsigned i=0;i<count && i<8;++i)if(const auto *p=rt->memory.hostPointer(c->r[5].uw[0]+4+4*i,4)){uint32_t a;std::memcpy(&a,p,4);args+=std::format(" {:08x}",a);}
            uint32_t type=0;if(const auto *p=rt->memory.hostPointer(c->r[16].uw[0],4))std::memcpy(&type,p,4);
            std::fprintf(stderr,"[vp2s:script] interpreter %x (class %x) op %x at %x:%s\n",c->r[16].uw[0],type,unsigned((w>>8)&0xff),c->r[5].uw[0],args.c_str());
        }
    }
    if(c->r[5].uw[0]==kSnippet+12)if(auto *resume=rt->memory.hostPointer(kSnippet+12,8)){
        uint32_t tag,resumePc;std::memcpy(&tag,resume,4);std::memcpy(&resumePc,resume+4,4);
        if(tag==kResumeTag){
            // The battle handler advanced the injected pc and yielded; resume the script.
            auto *fields=rt->memory.hostPointer(c->r[16].uw[0],0x608);if(!fields)throw std::runtime_error("native battle outside owned RAM");
            uint32_t blob;std::memcpy(&blob,fields+0x600,4);
            const uint32_t at=blob+0x20+4*resumePc,args=at+4,cleared=0;
            std::memcpy(fields+0x18,&resumePc,4);std::memcpy(fields+0x604,&args,4);c->r[5].uw[0]=at;std::memcpy(resume,&cleared,4);
        }
    }
    std::unique_lock lock(requests);
    if(battlePending){
        const uint32_t object=c->r[16].uw[0],at=c->r[5].uw[0];
        auto *snippet=rt->memory.hostPointer(kSnippet,20);auto *fields=rt->memory.hostPointer(object,0x608);
        if(!snippet || !fields)throw std::runtime_error("native battle outside owned RAM");
        uint32_t args,scriptPc,blob;std::memcpy(&args,fields+0x604,4);std::memcpy(&scriptPc,fields+0x18,4);std::memcpy(&blob,fields+0x600,4);
        if(args!=at+4 || at!=blob+0x20+4*scriptPc)throw std::runtime_error("native battle: interpreter fetch escaped step 3ee150's layout");
        const auto words=battleInstruction(*battlePending);std::memcpy(snippet,words.data(),12);
        std::memcpy(snippet+12,&kResumeTag,4);std::memcpy(snippet+16,&scriptPc,4);
        // pc such that blob + 0x20 + 4*pc = the snippet (32-bit wrap, as the handler's addu).
        const uint32_t snippetPc=(kSnippet-blob-0x20)/4,newArgs=kSnippet+4;
        if(blob+0x20+4*snippetPc!=kSnippet)throw std::runtime_error("native battle: blob not word aligned");
        std::memcpy(fields+0x18,&snippetPc,4);std::memcpy(fields+0x604,&newArgs,4);c->r[5].uw[0]=kSnippet;
        std::fprintf(stderr,"[vp2s:battle] interpreter %x (script pc %x) starts battle %u stage %u flags %08x\n",
            object,scriptPc,battlePending->battle,battlePending->stage,battlePending->flags);
        battlePending.reset();
        return false; // The compiled fetch, dispatch and op 0x38 handler run as the game's own code.
    }
    if(!pending)return false;
    const uint32_t object=c->r[16].uw[0],at=c->r[5].uw[0];
    auto *snippet=rt->memory.hostPointer(kSnippet,32);auto *fields=rt->memory.hostPointer(object,0x608);
    if(!snippet || !fields)throw std::runtime_error("native warp outside owned RAM");
    uint32_t args,scriptPc;std::memcpy(&args,fields+0x604,4);std::memcpy(&scriptPc,fields+0x18,4);
    if(args!=at+4)throw std::runtime_error("native warp: interpreter fetch escaped step 3ee150's layout");
    const auto words=warpInstruction(*pending);std::memcpy(snippet,words.data(),32);
    const uint32_t newArgs=kSnippet+4,rewound=scriptPc-8;
    std::memcpy(fields+0x604,&newArgs,4);std::memcpy(fields+0x18,&rewound,4);c->r[5].uw[0]=kSnippet;
    std::fprintf(stderr,"[vp2s:warp] interpreter %x (script pc %x) runs map change %d at (%d,%d,%d) facing %d entry %d\n",
        object,scriptPc,pending->map,pending->x,pending->y,pending->z,pending->facing,pending->entry);
    // Debug: VP2S_BOZO_WARP_FLAGS=hex[,hex...] sets those story flags (script variables 2000..4fff, bit n - 2000) with the
    // warp, so an arrival gated on the story (common script area 61 reads 259a) runs. The bits live in the save block
    // whose id is 3 in the table at 1d3690 (16 pointers, ids at +40), at +4 + n / 8 (3ed6a0's 2000-4fff case).
    if(const char *list=std::getenv("VP2S_BOZO_WARP_FLAGS")){
        uint32_t block=0;
        for(unsigned i=0;i<16;++i){uint32_t id,p;std::memcpy(&id,rt->memory.hostPointer(0x1d3690+0x40+4*i,4),4);std::memcpy(&p,rt->memory.hostPointer(0x1d3690+4*i,4),4);if(id==3)block=p;}
        if(!block)throw std::runtime_error("native warp flags: no story flag block (id 3) at 1d3690");
        for(const char *s=list;*s;){char *end;const auto flag=uint32_t(std::strtoul(s,&end,16));if(end==s)break;
            if(flag<0x2000 || flag>0x4fff)throw std::runtime_error("native warp flags: outside 2000..4fff");
            const uint32_t bit=flag-0x2000;auto *b=rt->memory.hostPointer(block+4+bit/8,1);if(!b)throw std::runtime_error("native warp flags outside RAM");
            *b|=uint8_t(1u<<(bit&7));std::fprintf(stderr,"[vp2s:warp] story flag %x set\n",flag);s=*end==','?end+1:end;}
    }
    pending.reset();
    return false; // The compiled fetch, dispatch and op 0x3a handler run as the game's own code.
}
}

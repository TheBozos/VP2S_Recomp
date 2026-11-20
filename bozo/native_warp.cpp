#include "native_warp.h"
#include <array>
#include <climits>
#include <cstdio>
#include <cstring>
#include <format>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
namespace vp2s {
namespace {
std::optional<WarpRequest> pending;
// Below the game's image (0x100000): the PS2 kernel's RAM, unused by the native kernel.
constexpr uint32_t kSnippet=0x80000;
}
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
void armNativeWarp(const WarpRequest &request){pending=request;}
bool nativeWarpPending(){return pending.has_value();}
std::array<uint32_t,8> warpInstruction(const WarpRequest &r){
    return {0x3a07u,uint32_t(r.map),uint32_t(r.entry),uint32_t(r.x),uint32_t(r.y),uint32_t(r.z),uint32_t(r.camera),uint32_t(r.facing)};
}
// Step 3ee150 at the fetch "lw s1,0(a1)": a1 = the instruction, interpreter
// +604 = a1 + 4 (the handler's argument pointer), +18 = the script pc, which
// the step advances by 1 + count after a handler that returns 1 (op 0x3a does).
std::optional<bool> nativeWarpInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x3ee288)return std::nullopt;
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
    pending.reset();
    return false; // The compiled fetch, dispatch and op 0x3a handler run as the game's own code.
}
}

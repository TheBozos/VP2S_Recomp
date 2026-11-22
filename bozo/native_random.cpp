#include "native_random.h"
#include "bozo/ee/dispatcher.h"
#include <stdexcept>
#include <cstdio>
#include <cstring>
#include <map>
#include <optional>
namespace vp2s {
bool nativeRandomSeed(uint8_t *,R5900Context *ctx,Runtime *rt,uint32_t pc){
    if(pc!=0x29e20c && pc!=0x29e210 && pc!=0x29e214)return false;
    auto *dispatcher=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=dispatcher?dynamic_cast<RandomOwner *>(&dispatcher->host()):nullptr;
    if(!ctx || !owner)throw std::runtime_error("native procedural random initialization requires per-run ownership");
    if(pc==0x29e20c){owner->proceduralRandom().seed(ctx->r[2].uw[0]);std::fprintf(stderr,"[vp2s:random] seeded %06x\n",owner->proceduralRandom().bits());}
    // Following initialization/read instructions are retired. The read has no
    // game-data output; no VU vector, random register or pipeline is retained.
    // Every procedural consumer remains guarded until lowered to this stream.
    return true;
}
// RNEXT.x vf31 (4b1f043c) advances the stream and puts it in vf31.x; the QMFC2(.I) that
// follows moves vf31 into a GPR. Each listed site's compiled code reads only that GPR's low
// word (mtc1), the random float in [1, 2); the other lanes are written 0 (RNEXT writes x only
// and those lanes are never read there).
//   mod028 4a45d8 / 4a45dc (v1): the angle 2 pi (r - 1) of a battle effect's random turn.
bool nativeRandomNext(uint8_t *,R5900Context *ctx,Runtime *rt,uint32_t pc){
    struct Site{uint32_t next,move;unsigned reg;};
    constexpr Site sites[]={{0x4a45d8,0x4a45dc,3}};
    const Site *site=nullptr;bool move=false;
    for(const auto &s:sites){if(pc==s.next)site=&s;else if(pc==s.move){site=&s;move=true;}}
    if(!site || !rt)return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=(move?(0x4820f801u|site->reg<<16):0x4b1f043cu))return false;   // another overlay at these addresses
    auto *dispatcher=dynamic_cast<bozo::ee::Dispatcher *>(rt->services);
    auto *owner=dispatcher?dynamic_cast<RandomOwner *>(&dispatcher->host()):nullptr;
    if(!ctx || !owner)throw std::runtime_error("native procedural random draw requires per-run ownership");
    static std::map<const Runtime *,std::optional<float>> drawn;auto &value=drawn[rt];
    if(!move){value=owner->proceduralRandom().next();return true;}
    if(!value)throw std::runtime_error("native procedural random move without its draw");
    uint32_t bits;std::memcpy(&bits,&*value,4);ctx->r[site->reg].uw[0]=bits;ctx->r[site->reg].uw[1]=ctx->r[site->reg].uw[2]=ctx->r[site->reg].uw[3]=0;
    value.reset();return true;
}
}

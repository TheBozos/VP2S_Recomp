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
//   mod028 4a41b0 (the same effect spawn), nine more v1 pairs: its size 1 + (r - 1)/4, the
//   offset direction and length, and the turn vector (each v1 read only by an mtc1, checked offline).
bool nativeRandomNext(uint8_t *,R5900Context *ctx,Runtime *rt,uint32_t pc){
    struct Site{uint32_t next,move;unsigned reg;};
    // mod001 2439f0 (the general particle emitter's step, class 154810): its program opcodes 0x20..0x29
    // and the jitter (+132 = 1) draw this way; each QMFC2.I target (a0, a1 or v1) is read only by an
    // mtc1 before it is written again (checked offline over the whole function).
    // mod028 4a73b0 (constructor of battle class 173010, frame 72200): a0 = r, read only by the mtc1 at 4a7434;
    // +3d4 = 0.13865 + 0.17453 (r - 1). The branch at 4a7464 returns without reading a0.
    constexpr Site sites[]={{0x4a45d8,0x4a45dc,3},{0x4a741c,0x4a7420,4},
        {0x4a429c,0x4a42a0,3},{0x4a4350,0x4a4354,3},{0x4a437c,0x4a4380,3},{0x4a43a8,0x4a43ac,3},{0x4a43f4,0x4a43f8,3},{0x4a4470,0x4a4474,3},{0x4a449c,0x4a44a0,3},{0x4a44c8,0x4a44cc,3},{0x4a44f4,0x4a44f8,3},
        {0x243f2c,0x243f30,4},{0x243f6c,0x243f70,4},{0x243fb0,0x243fb4,4},{0x243ffc,0x244000,4},{0x24403c,0x244040,4},{0x244080,0x244084,4},{0x2440cc,0x2440d0,4},{0x24410c,0x244110,4},{0x244150,0x244154,4},{0x24419c,0x2441a0,4},{0x2441dc,0x2441e0,4},{0x244220,0x244224,4},{0x24426c,0x244270,4},{0x2442ac,0x2442b0,4},{0x2442f0,0x2442f4,4},{0x24433c,0x244340,4},{0x24437c,0x244380,4},{0x2443c0,0x2443c4,4},{0x24440c,0x244410,5},{0x24444c,0x244450,5},{0x244490,0x244494,5},{0x2444d4,0x2444d8,5},{0x244524,0x244528,5},{0x244564,0x244568,5},{0x2445a8,0x2445ac,5},{0x2445ec,0x2445f0,5},{0x24464c,0x244650,4},{0x2446a4,0x2446a8,4},{0x2446e4,0x2446e8,4},{0x244b70,0x244b74,3},{0x244ba8,0x244bac,3},{0x244be0,0x244be4,3},{0x244c18,0x244c1c,3}};
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

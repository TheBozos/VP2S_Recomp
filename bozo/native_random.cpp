#include "native_cast.h"
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
    auto *dispatcher=rt?vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services):nullptr;
    auto *owner=dispatcher?vp2s::nativeCast<RandomOwner>(&dispatcher->host()):nullptr;
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
    // +3d4 = 0.13865 + 0.17453 (r - 1). The branch at 4a7464 returns without reading a0. Four more
    // there: 4a75b4 (a0, mtc1 4a75e0, then a0 = s1): +a0 = 0.9 + 0.3 (r - 1); 4a77ec (v1, mtc1
    // 4a7800): sp+78 = 0.5 - (r - 1); 4a7894 / 4a78dc (v1, mtc1 4a78a8 / 4a78f0): +3d8 = 2 + 5 (r - 1),
    // +3dc = 0.5 + 3 (r - 1). No v1 reader follows any of them before it is written again.
    // mod028 4a7b0c (class 173010, frame 72240): a0 = r, mtc1 4a7be0 (then a0 = s3); int(-201.2 + 507.2 (r - 1)).
    // mod028 3e6b94's victory scatter (battle frame 75420): 3e6c90 / 3e6cc0 (v1, mtc1 3e6ca8 / 3e6cd4): sp+140 and
    // sp+140 again = 0.5 - (r - 1), then normalised with sp+144 = 0 (the normalize25 words); v1 is not read again.
    // mod028 48da30 (the victory's scatter, battle frame 75460): ten draws (a0, a1 or v1), each target read only by
    // the mtc1 after it before it is written again (checked over the function's listing).
    // mod028 48c380 (the victory's burst): 48c5d8 (a0, mtc1 48c5f4; a0 is written again at 48c648 before any read).
    static constexpr Site sites[]={{0x48c5d8,0x48c5dc,4},{0x48dad8,0x48dadc,4},{0x48dd14,0x48dd18,5},{0x48ddf8,0x48ddfc,4},{0x48de24,0x48de28,4},{0x48de54,0x48de58,4},
        {0x48dee0,0x48dee4,3},{0x48dfb8,0x48dfbc,3},{0x48e064,0x48e068,4},{0x48e084,0x48e088,4},{0x48e0a4,0x48e0a8,4},
        {0x3e6c90,0x3e6c94,3},{0x3e6cc0,0x3e6cc4,3},{0x4a45d8,0x4a45dc,3},{0x4a7bcc,0x4a7bd0,4},{0x4a741c,0x4a7420,4},{0x4a75b4,0x4a75b8,4},{0x4a77ec,0x4a77f0,3},{0x4a7894,0x4a7898,3},{0x4a78dc,0x4a78e0,3},
        {0x4a429c,0x4a42a0,3},{0x4a4350,0x4a4354,3},{0x4a437c,0x4a4380,3},{0x4a43a8,0x4a43ac,3},{0x4a43f4,0x4a43f8,3},{0x4a4470,0x4a4474,3},{0x4a449c,0x4a44a0,3},{0x4a44c8,0x4a44cc,3},{0x4a44f4,0x4a44f8,3},
        {0x243f2c,0x243f30,4},{0x243f6c,0x243f70,4},{0x243fb0,0x243fb4,4},{0x243ffc,0x244000,4},{0x24403c,0x244040,4},{0x244080,0x244084,4},{0x2440cc,0x2440d0,4},{0x24410c,0x244110,4},{0x244150,0x244154,4},{0x24419c,0x2441a0,4},{0x2441dc,0x2441e0,4},{0x244220,0x244224,4},{0x24426c,0x244270,4},{0x2442ac,0x2442b0,4},{0x2442f0,0x2442f4,4},{0x24433c,0x244340,4},{0x24437c,0x244380,4},{0x2443c0,0x2443c4,4},{0x24440c,0x244410,5},{0x24444c,0x244450,5},{0x244490,0x244494,5},{0x2444d4,0x2444d8,5},{0x244524,0x244528,5},{0x244564,0x244568,5},{0x2445a8,0x2445ac,5},{0x2445ec,0x2445f0,5},{0x24464c,0x244650,4},{0x2446a4,0x2446a8,4},{0x2446e4,0x2446e8,4},{0x244b70,0x244b74,3},{0x244ba8,0x244bac,3},{0x244be0,0x244be4,3},{0x244c18,0x244c1c,3}};
    const Site *site=nullptr;bool move=false;
    for(const auto &s:sites){if(pc==s.next)site=&s;else if(pc==s.move){site=&s;move=true;}}
    if(!site || !rt)return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=(move?(0x4820f801u|site->reg<<16):0x4b1f043cu))return false;   // another overlay at these addresses
    auto *dispatcher=vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services);
    auto *owner=dispatcher?vp2s::nativeCast<RandomOwner>(&dispatcher->host()):nullptr;
    if(!ctx || !owner)throw std::runtime_error("native procedural random draw requires per-run ownership");
    static std::map<const Runtime *,std::optional<float>> drawn;auto &value=drawn[rt];
    if(!move){value=owner->proceduralRandom().next();return true;}
    if(!value)throw std::runtime_error("native procedural random move without its draw");
    uint32_t bits;std::memcpy(&bits,&*value,4);ctx->r[site->reg].uw[0]=bits;ctx->r[site->reg].uw[1]=ctx->r[site->reg].uw[2]=ctx->r[site->reg].uw[3]=0;
    value.reset();return true;
}
}

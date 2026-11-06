#include "native_random.h"
#include "bozo/ee/dispatcher.h"
#include <stdexcept>
namespace vp2s {
bool nativeRandomSeed(uint8_t *,R5900Context *ctx,Runtime *rt,uint32_t pc){
    if(pc!=0x29e20c && pc!=0x29e210 && pc!=0x29e214)return false;
    auto *dispatcher=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=dispatcher?dynamic_cast<RandomOwner *>(&dispatcher->host()):nullptr;
    if(!ctx || !owner)throw std::runtime_error("native procedural random initialization requires per-run ownership");
    if(pc==0x29e20c)owner->proceduralRandom().seed(ctx->r[2].uw[0]);
    // Following initialization/read instructions are retired. The read has no
    // game-data output; no VU vector, random register or pipeline is retained.
    // Every procedural consumer remains guarded until lowered to this stream.
    return true;
}
}

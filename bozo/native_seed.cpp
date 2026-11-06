#include "native_seed.h"
#include "bozo/ee/dispatcher.h"
#include <stdexcept>
namespace vp2s {
bool nativeBootSeed(uint8_t *,R5900Context *ctx,Runtime *rt,uint32_t pc) {
    if(pc!=0x1003c4) return false;
    auto *d=rt ? dynamic_cast<bozo::ee::Dispatcher *>(rt->services) : nullptr;
    auto *owner=d ? dynamic_cast<SeedOwner *>(&d->host()) : nullptr;
    if(!ctx || !owner)throw std::runtime_error("native boot seed requires a per-run seed owner");
    // The next original instructions mix this input with the game's constants
    // and call its real RNG initializer. No load address is inspected or served.
    ctx->r[3].sd[0]=static_cast<int32_t>(owner->bootSeed());
    return true;
}
}

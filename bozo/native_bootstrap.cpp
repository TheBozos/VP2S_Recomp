#include "native_bootstrap.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <stdexcept>
namespace vp2s {
bool nativeBootstrapKey(uint8_t *,R5900Context *ctx,Runtime *rt,uint32_t pc){
    if(pc!=0x10090c)return false;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<BootstrapOwner *>(&d->host()):nullptr;
    if(!ctx || !owner)throw std::runtime_error("native startup derivation requires its per-run tables");
    const auto address=ctx->r[16].uw[0];
    auto *record=rt->memory.hostPointer(address,16);
    if(address<0x100000 || uint64_t(address)+16>0x2000000u || !record)
        throw std::runtime_error("native startup seed record is outside application RAM");
    bootstrap::Words seed;std::memcpy(seed.data(),record,16);
    const auto key=bootstrap::derive(seed,owner->bootstrapTables());
    std::memcpy(record,key.data(),16);std::memcpy(&ctx->r[2],key.data(),16);
    ctx->r[9].ud[0]=0; // Retired CFC2 result; upper half is retained.
    return true;
}
}

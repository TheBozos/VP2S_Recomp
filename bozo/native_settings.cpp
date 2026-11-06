#include "native_settings.h"
#include "native_kernel.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <stdexcept>
namespace vp2s {
void nativeOsdSettings(uint8_t *,R5900Context *ctx,Runtime *rt) {
    auto *d=rt ? dynamic_cast<bozo::ee::Dispatcher *>(rt->services) : nullptr;
    auto *owner=d ? dynamic_cast<SettingsOwner *>(&d->host()) : nullptr;
    if(!owner || !ctx) throw std::runtime_error("native settings owner is missing");
    if(ctx->pc!=0x137280 && ctx->pc!=0x137290) throw std::runtime_error("unsupported native settings entry");
    auto *buffer=rt->memory.hostPointer(ctx->r[4].uw[0],4);
    if(!buffer) ctx->r[2].sd[0]=-1;
    else {
        if(ctx->pc==0x137290) {
            const uint32_t packed=owner->settings().packed();
            std::memcpy(buffer,&packed,4);
        } else {
            uint32_t packed; std::memcpy(&packed,buffer,4);
            owner->settings().assign(packed);
        }
        ctx->r[2].sd[0]=0;
    }
    ctx->pc=ctx->r[31].uw[0];
}
}

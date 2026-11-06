#include "native_frame.h"
#include "bozo/ee/dispatcher.h"
#include <stdexcept>
namespace vp2s {
bool nativeFrameInput(uint8_t *,R5900Context *ctx,Runtime *rt,uint32_t pc){
    if(pc!=0x11fb50)return false;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    if(!ctx || !d || !d->kernel().field())throw std::runtime_error("native frame phase requires a booted clock");
    // Logical parity from the native scheduler, directly. The original GS
    // load and >>13 extraction are gone; no CSR value/address is supplied.
    ctx->r[2].ud[0]=d->kernel().vblankCount()&1u;
    return true;
}
}

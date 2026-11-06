#include "native_frame.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <cstring>
namespace vp2s {
bool nativeFrameInput(uint8_t *,R5900Context *ctx,Runtime *rt,uint32_t pc){
    if(pc==0x11f62c){
        if(!ctx || !rt || !rt->port)throw std::runtime_error("native game frame boundary requires its port host");
        // The original wait has completed and produced its return value.
        // Its query-only branch bypasses this site. Preserve all game state.
        bozo::port::frameEnd(rt,ctx);return true;
    }
    if(pc!=0x11fb50)return false;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    if(!ctx || !d || !d->kernel().field())throw std::runtime_error("native frame phase requires a booted clock");
    if(std::getenv("VP2S_BOZO_TRACE_FRAME")){
        static unsigned traced=0;
        if(traced++<12){const auto object=ctx->r[4].uw[0];uint32_t counters[3]={};
            for(unsigned i=0;i<3;++i)if(const auto *p=rt->memory.hostPointer(object+(i==2?0x300:0xc+i*4),4))std::memcpy(&counters[i],p,4);
            std::fprintf(stderr,"[vp2s:frame-input] object=%x wait=%u total=%u callbacks=%u native=%u\n",object,counters[0],counters[1],counters[2],d->kernel().vblankCount());
        }
    }
    // Logical parity from the native scheduler, directly. The original GS
    // load and >>13 extraction are gone; no CSR value/address is supplied.
    ctx->r[2].ud[0]=d->kernel().vblankCount()&1u;
    return true;
}
}

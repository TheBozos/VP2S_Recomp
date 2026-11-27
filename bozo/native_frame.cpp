#include "native_cast.h"
#include "native_frame.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <cstring>
namespace vp2s {
// Battle timestep (mod027 36c9e8..36ca88): count = 11f590(the frame object) = vblanks since the last battle frame,
// clamped to 1..4 after `cvt.s.w f2, f1` at 36ca00, then dt = count * speed / 60 to 35dee8. Diagnostics only, off by
// default: VP2S_BOZO_TRACE_BATTLE_VSYNCS=1 prints each count; VP2S_BOZO_DEBUG_BATTLE_VSYNCS=n replaces it (a pacing
// experiment). The original conversion always runs after.
void nativeBattleVsyncs(R5900Context *ctx,Runtime *rt){
    uint32_t word=0;if(const auto *p=rt?rt->memory.hostPointer(0x36ca00,4):nullptr)std::memcpy(&word,p,4);
    if(!ctx || word!=0x468008a0u)return;
    static const char *trace=std::getenv("VP2S_BOZO_TRACE_BATTLE_VSYNCS");static const char *force=std::getenv("VP2S_BOZO_DEBUG_BATTLE_VSYNCS");
    if(trace && *trace=='1')std::fprintf(stderr,"[vp2s:battle-vsyncs] %d\n",int32_t(ctx->fu[1]));
    if(force && *force)ctx->fu[1]=uint32_t(std::atoi(force));
}
bool nativeFrameInput(uint8_t *,R5900Context *ctx,Runtime *rt,uint32_t pc){
    if(pc==0x11f62c){
        if(!ctx || !rt || !rt->port)throw std::runtime_error("native game frame boundary requires its port host");
        // The original wait has completed and produced its return value.
        // Its query-only branch bypasses this site. Preserve all game state.
        bozo::port::frameEnd(rt,ctx);return true;
    }
    if(pc!=0x11fb50)return false;
    auto *d=rt?vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services):nullptr;
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

#include "native_math.h"
#include "../port/vp2s_math.h"
#include <cstring>
#include <stdexcept>
namespace vp2s {
namespace {
uint8_t *record(Runtime *rt,uint32_t address,uint32_t bytes){
    const bool scratch=address>=bozo::kScratchpadBase && address<bozo::kScratchpadBase+bozo::kScratchpadBytes;
    const auto offset=address&0x1ffffffu;
    if(!rt || (address&15) || (!scratch && (offset<0x100000 || (address>>28)==7)))
        throw std::runtime_error("invalid native math record");
    auto *result=rt->memory.hostPointer(address,bytes);
    if(!result)throw std::runtime_error("native math record extends outside owned memory");
    return result;
}
}
bool nativeMathInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc==0x29e1a0){
        // The default math code installation has no execution or game-data
        // result. Its algorithms are compiled native record operations. Retire
        // the physical enqueue before it exists; never mark a job busy/complete.
        const uint32_t object=c->r[4].uw[0];auto *p=record(rt,object,0x54);
        uint32_t type=0;std::memcpy(&type,p,4);
        if(type!=0x1591d0 || (c->r[5].uw[0]&255)!=0 || p[0x50]!=0)
            throw std::runtime_error("unported math program selection or outstanding legacy job");
        return true;
    }
    switch(pc){
    case 0x217554:case 0x217558:case 0x21755c:case 0x217560:
    case 0x217564:case 0x217568:case 0x21756c:case 0x217570:case 0x21757c:
    case 0x21758c:case 0x217590:case 0x217594:
    case 0x216734:case 0x21673c:case 0x21674c:case 0x216750:case 0x216754:
        return true; // obsolete vector transport; no shadow register writes
    case 0x217580:case 0x216740:
        c->r[9].ud[0]=0;return true; // helper's ordinary CPU clobber
    case 0x217588:{
        math::Matrix local,parent;
        std::memcpy(local.data(),record(rt,c->r[5].uw[0],64),64);
        std::memcpy(parent.data(),record(rt,c->r[6].uw[0],64),64);
        const auto result=math::compose(local,parent);
        std::memcpy(record(rt,c->r[3].uw[0],64),result.data(),64);
        return true;
    }
    case 0x216748:{
        __m128 input;std::memcpy(&input,record(rt,c->r[5].uw[0],16),16);
        const auto result=math::quaternionMatrix(input);
        std::memcpy(record(rt,c->r[3].uw[0],64),result.data(),64);
        return true;
    }
    default:return false;
    }
}
}

#include "native_seed.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <stdexcept>
namespace vp2s {
namespace {
// Module 27 (the battle core at 0x35ec80) programs EE timer 0 to count H-blanks
// (mode 0x83, count 0) and reads the timers two ways:
//  'c' its streaming copier (3676f0..369374) rounds T0 up to the end of the frame
//      (525 lines) or field (262) minus a margin as a deadline, then copies as much as
//      the lines left allow and yields past it. Natively there is no scan-out: a read is
//      "the frame has just begun" (0), so a whole frame's budget always remains and the
//      copies are never cut by PS2 scan timing.
//      Its time-sliced worker mod028 47f5a0 (called at 36937c with a1 = deadline - T0)
//      sets its own deadline T0 + a1 - 3 and saves its progress (+d84..+d8c) and returns
//      when T0 passes it; the same rule (0) lets it finish its steps in one call.
//  'r' random sources (363440: count mod n; 36cbb4: four timers mixed into the battle
//      seed): a seeded 16-bit sample.
//  'w' the mode/count writes: nothing to program.
// The words are checked: mod003 occupies these addresses too.
struct TimerSite {uint32_t pc,word;char kind;};
constexpr TimerSite battleTimerSites[]={
    {0x363440,0x8c240000,'r'}, {0x3676f0,0x8c250000,'c'}, {0x367724,0x8c240000,'c'}, {0x367880,0x8c230000,'c'},
    {0x367934,0x8c220000,'c'}, {0x367a7c,0x8c220000,'c'}, {0x367bcc,0x8c220000,'c'}, {0x367d74,0x8c240000,'c'},
    {0x367fe4,0x8c230000,'c'}, {0x3684dc,0x8c220000,'c'}, {0x36868c,0x8c230000,'c'}, {0x369374,0x8c220000,'c'},
    {0x36c9e0,0xac200000,'w'}, {0x36cbbc,0x8c270000,'r'}, {0x36cbd0,0x8c231800,'r'}, {0x36cc00,0x8c260800,'r'},
    {0x36cc28,0x8c241000,'r'}, {0x36d3d4,0xac230010,'w'}, {0x36d3dc,0xac200000,'w'},
    {0x47f60c,0x8c220000,'c'}, {0x47f824,0x8c230000,'c'}, {0x47f894,0x8c230000,'c'}, {0x47fb4c,0x8c220000,'c'}, {0x47fe0c,0x8c230000,'c'},
    {0x480604,0x8c220000,'c'}, {0x480838,0x8c220000,'c'}, {0x4809ac,0x8c230000,'c'}, {0x481074,0x8c220000,'c'}};
}
bool nativeBootSeed(uint8_t *,R5900Context *ctx,Runtime *rt,uint32_t pc) {
    for(const auto &site:battleTimerSites){
        if(site.pc!=pc)continue;
        uint32_t word=0;if(const auto *at=rt?rt->memory.hostPointer(pc,4):nullptr)std::memcpy(&word,at,4);
        if(word!=site.word)return false;   // another overlay at this address
        auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
        auto *owner=d?dynamic_cast<SeedOwner *>(&d->host()):nullptr;
        if(!ctx || !owner)throw std::runtime_error("native battle timer entropy requires a per-run seed owner");
        if(site.kind=='r')ctx->r[(word>>16)&31].ud[0]=owner->initializationJitter();   // a count, 0..0xffff
        else if(site.kind=='c')ctx->r[(word>>16)&31].ud[0]=0;                           // the frame's first line
        return true;   // 'w': no timer to program
    }
    if(pc!=0x1003c4 && pc!=0x2c3cb4) return false;
    auto *d=rt ? dynamic_cast<bozo::ee::Dispatcher *>(rt->services) : nullptr;
    auto *owner=d ? dynamic_cast<SeedOwner *>(&d->host()) : nullptr;
    if(!ctx || !owner)throw std::runtime_error("native boot seed requires a per-run seed owner");
    if(pc==0x2c3cb4){
        // Retire the hardware-dependent timer-jitter warmup loop. The retained game
        // code derives its discard count from this bounded native entropy and
        // keeps its original scalar generator and shuffle. No timer is served.
        ctx->r[2].ud[0]=owner->initializationJitter();return true;
    }
    // The next original instructions mix this input with the game's constants
    // and call its real RNG initializer. No load address is inspected or served.
    ctx->r[3].sd[0]=static_cast<int32_t>(owner->bootSeed());
    return true;
}
}

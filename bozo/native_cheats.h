#pragma once
#include "bozo/runtime.h"
#include <atomic>
// Cheats (the F2 panel's Cheats tab, bozo/main.cpp). They change values only the way the game itself does: through
// its own setters, so its anti-cheat checksums (party records +14/+18/+34, docs/BOZO_MIGRATION.md) stay valid.
namespace vp2s {
struct NativeCheats {
    std::atomic<bool> infiniteHealth{false};   // party HP held at its maximum in battle
    std::atomic<int64_t> setMoney{-1};          // a request: the party's money (OTH) becomes this, then -1
    std::atomic<int64_t> money{-1};             // the money last read (-1: not known yet)
};
NativeCheats &nativeCheats();
// Party record layout (persistent records [1d3694] + i*0x140; battle copies 0x160 apart), its XOR keys.
namespace party {
constexpr uint32_t kHpKey=0x1c6f7603,kMaxHpKey=0x7db9fc97,kBaseMaxHpKey=0x358ff46b,kFlagsKey=0xa9c3;
constexpr uint8_t kLevelKey=0xb9,kKindKey=0x65;
constexpr uint32_t kMoneyKey=0x37ce67a3,kMoneyMax=99999999;
}
// At each frame's end (the game's thread, between frames): live party members in battle back to full HP, and a
// money request applied.
void applyNativeCheats(Runtime *);
}

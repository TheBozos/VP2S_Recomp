#pragma once
#include "bozo/runtime.h"
#include <array>
#include <atomic>
// Cheats (the F2 panel's Cheats tab, bozo/main.cpp). They change values only the way the game itself does: through
// its own setters, so its anti-cheat checksums (party records +14/+18/+34, docs/BOZO_MIGRATION.md) stay valid.
namespace vp2s {
struct NativeCheats {
    std::atomic<bool> infiniteHealth{false};   // party HP held at its maximum in battle
    std::atomic<bool> weakEnemies{false};      // live enemies in battle held at 1 HP (one hit wins)
    std::atomic<int64_t> setMoney{-1};          // a request: the party's money (OTH) becomes this, then -1
    std::atomic<int64_t> money{-1};             // the money last read (-1: not known yet)
    // EXP: gains multiplied by this (1: off). Each party member's EXP is watched between frames; a rise of d becomes
    // d * multiplier. The game decides level-ups when it grants EXP, so the extra counts from its next check.
    std::atomic<int> expMultiplier{1};
    // Items: a request (id, count; -1 none) and, read each frame, the count held of every item id.
    static constexpr unsigned kItems=0x2ef;
    std::atomic<int> giveItem{-1},giveCount{0};
    std::atomic<bool> inventoryKnown{false};
    std::array<std::atomic<uint8_t>,kItems> counts{};
};
NativeCheats &nativeCheats();
// Party record layout (persistent records [1d3694] + i*0x140; battle copies 0x160 apart), its XOR keys.
namespace party {
constexpr uint32_t kHpKey=0x1c6f7603,kMaxHpKey=0x7db9fc97,kBaseMaxHpKey=0x358ff46b,kFlagsKey=0xa9c3;
constexpr uint8_t kLevelKey=0xb9,kKindKey=0x65;
constexpr uint32_t kMoneyKey=0x37ce67a3,kMoneyMax=99999999;
// Inventory: [35ec18] -> one u32 per item id, (flag 1 << 24) | (count << 16) | id, unencoded. Item table: [35ec08],
// 0x38 bytes per item, +30 its stack limit (99 for most).
constexpr uint32_t kExpKey=0x6cdf397e;   // +04, under the flags checksum +14
constexpr uint32_t kInventory=0x35ec18,kItemTable=0x35ec08;
// Battle combatants: the scene [1d366c] holds a pointer to the battle (class 16f7a0) at +cb88 when started from map 79,
// but a variable-length block earlier in the scene moves it (+cb28 from a dungeon map: field encounters), so it is
// found by its class in +c800..+cd00. Party units +34..+40, enemy units +44..+60 of the battle, each unit's record at
// +818; the party's battle records also sit in four slots 0x44 after the battle pointer.
constexpr uint32_t kBattleClass=0x16f7a0,kBattleSearchFrom=0xc800,kBattleSearchTo=0xcd00,kPartySlots=0x44,kUnitRecord=0x818;
}
// At each frame's end (the game's thread, between frames): live party members in battle back to full HP, and a
// money request applied.
void applyNativeCheats(Runtime *);
}

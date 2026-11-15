#pragma once
#include "native_assets.h"
#include <optional>
#include <vector>
namespace vp2s {
// Scratchpad sprite batcher (mod001 class 152ac0). The writer builds each chunk
// in a scratchpad half: sections, each a CNT tag whose address is the writer's
// running RAM cursor (+7c) followed by its data, zero tags where an optional
// section is empty, and an END tag. A destination-chain fromSPR scattered them
// into the RAM buffer. Natively each section's data is published at its
// address when the original would have started the transfer; the walk only
// admits the writer's own layout.
struct SprWriterSection {uint32_t source=0,destination=0,qwords=0;};
// The sections of the chunk at `spr`: the first starts at `limitLow`, each next
// one where the previous ended, all inside [limitLow, limitHigh), ending exactly
// at `cursor`. Throws on anything else.
std::vector<SprWriterSection> nativeSprWriterSections(Runtime *,uint32_t spr,uint32_t limitLow,uint32_t limitHigh,uint32_t cursor);
std::optional<bool> nativeSprWriterInstruction(R5900Context *,Runtime *,uint32_t pc);
// sceDmaGetChan / sceDmaSync for the writer's own fromSPR channel (context byte 8):
// natively no transfer is ever in flight.
bool nativeSprWriterLibrary(R5900Context *,Runtime *,uint32_t target);
}

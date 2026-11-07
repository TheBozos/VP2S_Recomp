#pragma once
#include "bozo/runtime.h"
namespace vp2s {
// Game record algorithms shared with the existing native port, through Bozo's
// memory/CPU ABI. No vector bank, microprogram or transport is provided.
bool nativeMathInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
}

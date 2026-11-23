#pragma once
#include <cstdint>
// The sites of the handlers whose decision on a site depends on more than the site and its instruction word
// (state, or words elsewhere in memory). Each handler returns nullopt at once for a site its predicate rejects,
// so the predicate is exactly where the handler can do anything; the site dispatcher (bozo/main.cpp) offers a
// cached site only to the conditional handlers whose predicate accepts it.
namespace vp2s {
bool nativeUiWindowSite(uint32_t pc);
bool nativeProceduralDrawSite(uint32_t pc);
bool nativeUiSpriteSite(uint32_t pc);
bool nativeScreenRectSite(uint32_t pc);
bool nativeSprWriterSite(uint32_t pc);
bool nativeScreenZoomSite(uint32_t pc);
bool nativeColumnCaptureSite(uint32_t pc);
bool nativeBattleImageSite(uint32_t pc);
bool nativeBattleTextureSetSite(uint32_t pc);
bool nativeBattleSpriteSite(uint32_t pc);
bool nativeStateSetupSite(uint32_t pc);
}

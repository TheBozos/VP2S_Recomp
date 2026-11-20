#pragma once
#include "bozo/runtime.h"
#include <array>
#include <cstdint>
#include <optional>
#include <string>
namespace vp2s {
// Debug teleport (--warp). The game changes map with event-script op 0x3a
// (handler 3af3b0): args map, entry, x, y, z, camera (-1), facing in degrees.
// It builds a request (class 1656c0, 443b10) and queues it on the field
// manager (3d6140). A warp makes the next script instruction fetch of any
// running interpreter (class 1620e0, step 3ee150, fetch 3ee288) read one such
// instruction from free low RAM; the game's own handler does the rest.
struct WarpRequest {int32_t map=0,entry=0,x=0,y=0,z=0,camera=-1,facing=0;};
// "map,x,y,z[,facing[,entry]]", or "MAP" alone: that map's most used entry
// point in `table` (tools/vp2_warps.py). Throws on a malformed list or an
// unknown map.
WarpRequest parseWarp(const std::string &text,const std::string &table);
void armNativeWarp(const WarpRequest &request);
bool nativeWarpPending();
// The 8 words the interpreter reads: (0x3a << 8) | 7, then the 7 arguments.
std::array<uint32_t,8> warpInstruction(const WarpRequest &request);
std::optional<bool> nativeWarpInstruction(R5900Context *,Runtime *,uint32_t pc);
}

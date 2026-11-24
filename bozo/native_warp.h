#pragma once
#include "bozo/runtime.h"
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
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
// Every row of the warp table (tools/vp2_warps.py), in file order: the map's entry points, most used first.
struct WarpRow {int32_t map=0,entry=0,x=0,y=0,z=0,facing=0,uses=0;};
std::vector<WarpRow> loadWarpTable(const std::string &table);
// The 8 words the interpreter reads: (0x3a << 8) | 7, then the 7 arguments.
std::array<uint32_t,8> warpInstruction(const WarpRequest &request);
// Debug battle launcher (--battle). Battles start with event-script op 0x38
// (handler 3a0b90): arg0 = (stage << 16) | battle id (stage 0: the field's
// default, field manager +ae), arg1 = flags (low 24 bits; 0xff = a formation
// follows inline, as the Lost Forest's map script does) | (byte << 24). The
// game's scripted battles pass 0x01000100. The handler advances the pc itself
// from the count byte of the blob word at the pc, so the injection points the
// pc at the snippet relative to the blob, and that interpreter's next fetch
// gets its own pc back.
struct BattleRequest {uint32_t battle=0,stage=0,flags=0x01000100;};
// "ID[,stage[,flags]]" (decimal or 0x hex). Throws on malformed values or an id
// outside 0..0xfffe (0xffff selects the area table).
BattleRequest parseBattle(const std::string &text);
void armNativeBattle(const BattleRequest &request);
bool nativeBattlePending();
// Arming may come from another thread (the window's F2 panel): the requests are guarded by a lock.
// The 3 words the handler reads: (0x38 << 8) | 2, arg0, arg1.
std::array<uint32_t,3> battleInstruction(const BattleRequest &request);
std::optional<bool> nativeWarpInstruction(R5900Context *,Runtime *,uint32_t pc);
}

#pragma once
// Where the game loaded a piece of its data from (vp2s_loads.cpp): the asset part tools/extract.py names
// "<chain disc offset:010x>_<part:03d>" and the offset in it, from the game's own disc reads and decompression.
#include <cstdint>
#include <string>

class PS2Runtime;
void vp2sInstallLoads(PS2Runtime &runtime);
// The asset part holding the EE address and the address's offset in it; false when no load the game made covers it.
bool vp2sLoadedFrom(uint32_t address, std::string &part, uint32_t &offset);

#pragma once
// VP2's 2D layer (HUD, text, menus) drawn natively from the game's descriptions (vp2s_ui.cpp, docs/vp2_2d.md).
class PS2Runtime;
void vp2sInstallUi(PS2Runtime &runtime);
#include <cstdint>
// The sheet the game set up at a fixed DBP outside its texture manager (0x2F3630: the FIS file's key), if any.
bool vp2sUiFixedSheet(uint32_t dbp, uint32_t &tag, uint32_t &index);
// Notes a sheet another module's uploader put at a fixed DBP (the battle's mod027 0x36B3A0).
void vp2sUiNoteFixedSheet(uint32_t dbp, uint32_t tag, uint32_t index);

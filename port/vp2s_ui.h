#pragma once
// VP2's 2D layer (HUD, text, menus) drawn natively from the game's descriptions (vp2s_ui.cpp, docs/vp2_2d.md).
class PS2Runtime;
void vp2sInstallUi(PS2Runtime &runtime);
#include <cstdint>
// The sheet the game set up at a fixed DBP outside its texture manager (0x2F37B0: the FIS file's key), if any.
bool vp2sUiFixedSheet(uint32_t dbp, uint32_t &tag, uint32_t &index);
// Notes a sheet another module's uploader put at a fixed DBP (the battle's mod027 0x36B3A0).
void vp2sUiNoteFixedSheet(uint32_t dbp, uint32_t tag, uint32_t index);
// The field pause's background (scene object class 0x1670D0): the game draws its last field frame again, which the GS
// kept, as its packet does: at 112/128, then 16 copies at 80/128 with alpha 12/128, copy k moved by (-k, -2k) pixels
// of the 512 x 448 GS frame. Natively: a picture of the frame on screen when the object appears (the renderer's
// captureFrame), drawn the same way. Called between the 3D and the interface, with the scene pass's scene.
void vp2sUiFrozenFrame(uint8_t *rdram, uint32_t scene);

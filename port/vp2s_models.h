#pragma once
// VP2's models drawn by the native renderer from the game's own data (vp2s_models.cpp, docs/vp2_models.md).
class PS2Runtime;
void vp2sInstallModels(PS2Runtime &runtime);
// The native frame ends with the game's frame (the frame wait): the models drawn since the last end make one.
void vp2sModelsFrameEnd();

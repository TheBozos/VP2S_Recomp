#pragma once
// VP2's models drawn by the native renderer from the game's own data (vp2s_models.cpp, docs/vp2_models.md).
class PS2Runtime;
void vp2sInstallModels(PS2Runtime &runtime);
// The native frame ends with the game's frame (the frame wait): the models drawn since the last end make one.
void vp2sModelsFrameEnd();
// The frame's interface starts here (vp2s_ui.cpp, its first layer drawn): the game's post effect goes before it, as
// the GS runs the 3D, the post effect, then the interface's packets.
void vp2sModelsInterfaceStarts();

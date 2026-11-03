#pragma once
// VP2's models drawn by the native renderer from the game's own data (vp2s_models.cpp, docs/vp2_models.md).
class PS2Runtime;
void vp2sInstallModels(PS2Runtime &runtime);
// The native frame ends with the game's frame (the frame wait): the models drawn since the last end make one.
void vp2sModelsFrameEnd();
// The frame's interface starts here (vp2s_ui.cpp, its first layer drawn): the game's post effect goes before it, as
// the GS runs the 3D, the post effect, then the interface's packets.
void vp2sModelsInterfaceStarts();
// EE memory as the scene pass last saw it (null before the first frame): for diagnostics elsewhere.
const unsigned char *vp2sEeRam();
// The 3D of a camera whose GS frame is not the native screen's 640 x 448 (the events' cameras render 512 x 512 around
// the same centre, which the PS2 shows over the whole screen): a matrix to GS coordinates (column-major, x y z w) made
// to fill it, scaled around the camera's centre. The camera is the last one the models were drawn with.
void vp2sFitCameraScreen(float matrix[16]);

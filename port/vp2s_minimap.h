#pragma once
// VP2's field minimap drawn by the native renderer from the game's area map object (vp2s_minimap.cpp, docs/vp2_2d.md).
class PS2Runtime;
void vp2sInstallMinimap(PS2Runtime &runtime);
// Draws this frame's minimap (if the map object showed one) with the scene: after the 3D, before the game's effect
// passes (vp2s_models.cpp calls it once a frame, at the first effect pass), as the GS gets it.
void vp2sMinimapDraw();

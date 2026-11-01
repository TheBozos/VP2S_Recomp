#pragma once
// VP2's battle HUD (mod028's 2D widgets) drawn by the native renderer from the widgets' own descriptors
// (vp2s_battle_hud.cpp, docs/vp2_2d.md).
class PS2Runtime;
void vp2sInstallBattleHud(PS2Runtime &runtime);
// Draws the widgets' last primitives (vp2s_models.cpp calls it at the end of each native frame).
void vp2sBattleHudDraw();

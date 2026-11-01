#pragma once
// VP2's textures from the game's texture code and the offline conversion (vp2s_textures.cpp).
#include "runtime/native/native_texture_cache.h"

#include <cstdint>
#include <memory>

class PS2Runtime;
void vp2sInstallGameTextures(PS2Runtime &runtime);
// The texture with the game's key (file tag, index) as the conversion has it (the copy the game loaded), with the
// palette part `csa` for a 4-bit one; null when the conversion does not have it.
// paletteTag / paletteIndex (when not 0): the palette from that conversion instead.
std::shared_ptr<const ps2x::native::TextureCache::Texture> vp2sGameTexture(uint32_t tag, uint32_t index, uint32_t csa,
                                                                           uint32_t paletteTag = 0, uint32_t paletteIndex = 0);
// The 256-entry palette (RGBA8, CSM1 resolved) of the conversion with that key; false when it has none.
bool vp2sGamePalette(uint32_t tag, uint32_t index, uint32_t palette[256]);

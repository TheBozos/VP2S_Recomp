#pragma once
// Save states for engine pictures and the UI records that hold them. A picture (an engine
// texture) cannot be written to a file, but each one VP2S makes comes from one of three
// factories over offline data: a texture asset's palette part, a glyph of a converted font,
// or the camp menu backdrop built from two assets. The factories note each picture's source
// here; a held picture is saved as its source and made again on load.
#include "native_picture_source.h"
#include "native_ui_sprites.h"
#include "native_ui_text.h"
#include "native_ui_windows.h"
#include "bozo/render/texture.h"
namespace vp2s {
inline void io(bozo::state::Archive &a,GlyphFontKey &k){bozo::state::fields(a,k.disc,k.part,k.offset);}
struct PictureContext {NativeTextures &textures;NativeGlyphs &glyphs;};
void statePicture(bozo::state::Archive &,const PictureContext &,std::shared_ptr<const bozo::render::Texture> &);
void stateFont(bozo::state::Archive &,const PictureContext &,std::shared_ptr<const NativeGlyphFont> &);
void stateSprite(bozo::state::Archive &,const PictureContext &,NativeUiSprite &);
void stateWindow(bozo::state::Archive &,const PictureContext &,NativeUiWindow &);
void stateText(bozo::state::Archive &,const PictureContext &,NativeUiText &);
}

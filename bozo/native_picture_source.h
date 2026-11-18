#pragma once
// Where an engine picture came from (native_state_ui.h): the factories note it, save states
// write it instead of the picture.
#include "native_glyphs.h"
#include "native_textures.h"
namespace vp2s {
struct PictureSource {
    enum class Kind:uint8_t {None,Asset,Glyph,Backdrop} kind=Kind::None;
    TextureKey asset,second;  // Asset: asset + palette part; Backdrop: top, bottom
    GlyphFontKey font;        // Glyph: font + glyph index + palette part
    uint32_t index=0,part=0;
};
void notePicture(const std::shared_ptr<const bozo::render::Texture> &,const PictureSource &);
}

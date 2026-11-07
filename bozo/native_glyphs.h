#pragma once
#include "native_assets.h"
#include "bozo/render/texture.h"
#include <filesystem>
#include <memory>
#include <string>
#include <map>
namespace vp2s {
struct GlyphFontKey {
    uint64_t disc=0;
    uint32_t part=0,offset=0;
    auto operator<=>(const GlyphFontKey &)const=default;
};
struct NativeGlyphFont {
    uint32_t width=0,height=0,count=0,stride=0,cellWidth=0,cellHeight=0,border=0,lineHeight=0;
    uint64_t identity=0;
    std::vector<uint32_t> rgba; // Fully converted offline, palette parts 1 then 2.
    mutable std::map<uint32_t,std::weak_ptr<const bozo::render::Texture>> pictures;
    std::shared_ptr<const bozo::render::Texture> picture(uint32_t glyph,uint32_t palettePart)const;
};
class NativeGlyphs {
    std::filesystem::path root;
    std::map<GlyphFontKey,std::string> files;
    std::map<std::string,std::shared_ptr<const NativeGlyphFont>> loaded;
public:
    explicit NativeGlyphs(std::filesystem::path directory={});
    std::shared_ptr<const NativeGlyphFont> load(GlyphFontKey);
    std::shared_ptr<const NativeGlyphFont> font(Runtime *,const NativeAssets &,uint32_t fontObject);
};
}

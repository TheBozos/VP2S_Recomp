#include "native_glyphs.h"
#include <algorithm>
#include <atomic>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
namespace vp2s {
namespace {
std::atomic<uint64_t> serial{1};
uint32_t word(Runtime *rt,uint32_t address){
    address=NativeAssets::address(address);
    if(!rt || address<0x100000 || address>0x2000000-4)throw std::runtime_error("native glyph metadata outside EE RAM");
    uint32_t result;std::memcpy(&result,rt->memory.hostPointer(address,4),4);return result;
}
uint32_t cellSize(uint32_t n){uint32_t result=1;while(result<n)result*=2;return result;}
}
std::shared_ptr<const bozo::render::Texture> NativeGlyphFont::picture(uint32_t glyph,uint32_t palettePart)const{
    if(glyph>=count || palettePart<1 || palettePart>2)throw std::runtime_error("native glyph index/palette part outside converted font");
    const auto key=palettePart*65536+glyph;
    if(auto it=pictures.find(key);it!=pictures.end())if(auto held=it->second.lock())return held;
    auto texture=std::make_shared<bozo::render::Texture>();texture->width=cellWidth;texture->height=cellHeight;
    texture->key=(uint64_t(1)<<63)|(identity<<20)|key;texture->version=identity;
    const size_t pixels=size_t(cellWidth)*cellHeight,first=(size_t(palettePart-1)*count+glyph)*pixels;
    if(first>rgba.size() || pixels>rgba.size()-first)throw std::runtime_error("native glyph pixels outside converted font");
    texture->rgba.assign(rgba.begin()+first,rgba.begin()+first+pixels);pictures[key]=texture;return texture;
}
NativeGlyphs::NativeGlyphs(std::filesystem::path directory):root(std::move(directory)){
    if(root.empty())return;
    std::ifstream input(root/"manifest.txt");std::string line;
    if(!std::getline(input,line) || line!="VPGFM1")throw std::runtime_error("native glyph manifest unavailable/version mismatch");
    while(std::getline(input,line)){
        if(line.empty())continue;std::istringstream row(line);GlyphFontKey key;std::string name,extra;
        if(!(row>>std::hex>>key.disc>>key.part>>key.offset>>name) || row>>extra || name.size()!=46 ||
           name.substr(0,2)!="g_" || name.substr(42)!=".vpg" || name.find_first_not_of("0123456789abcdef",2)<42 ||
           !files.emplace(key,name).second)throw std::runtime_error("invalid/duplicate native glyph manifest entry");
    }
    if(!input.eof())throw std::runtime_error("native glyph manifest read failed");
}
std::shared_ptr<const NativeGlyphFont> NativeGlyphs::load(GlyphFontKey key){
    const auto found=files.find(key);if(found==files.end())throw std::runtime_error("font provenance requires offline glyph conversion");
    if(auto it=loaded.find(found->second);it!=loaded.end())return it->second;
    std::ifstream input(root/found->second,std::ios::binary);char magic[8];uint32_t h[9];
    if(!input.read(magic,8) || std::memcmp(magic,"VPGF0001",8) || !input.read(reinterpret_cast<char *>(h),sizeof(h)))
        throw std::runtime_error("invalid/truncated native glyph pack");
    if(!h[0] || h[0]>256 || !h[1] || h[1]>256 || !h[2] || h[2]>16384 || h[3]<(h[0]*h[1]+1)/2 ||
       h[4]!=cellSize(h[0]) || h[5]!=cellSize(h[1]) || h[6]>16 || !h[7] || h[7]>h[1] || h[8]!=2)
        throw std::runtime_error("invalid native glyph pack dimensions/metrics");
    const uint64_t words=uint64_t(h[2])*h[4]*h[5]*2;
    if(words>128*1024*1024/4)throw std::runtime_error("native glyph pack exceeds converted size bound");
    auto font=std::make_shared<NativeGlyphFont>();font->width=h[0];font->height=h[1];font->count=h[2];font->stride=h[3];
    font->cellWidth=h[4];font->cellHeight=h[5];font->border=h[6];font->lineHeight=h[7];font->identity=serial.fetch_add(1);
    font->rgba.resize(words);
    if(!input.read(reinterpret_cast<char *>(font->rgba.data()),words*4) || input.peek()!=std::char_traits<char>::eof())
        throw std::runtime_error("native glyph pack payload size mismatch");
    loaded.emplace(found->second,font);return font;
}
std::shared_ptr<const NativeGlyphFont> NativeGlyphs::font(Runtime *rt,const NativeAssets &assets,uint32_t object){
    const auto pixels=word(rt,object+0x10),base=word(rt,object);
    const auto origin=assets.origin(pixels),source=assets.origin(base);
    if(!origin || !source || origin->kind!=AssetOrigin::Kind::Part || source->kind!=AssetOrigin::Kind::Part ||
       source->disc!=origin->disc || source->part!=origin->part || source->offset)
        throw std::runtime_error("native glyph font has no complete decompressed-file provenance");
    auto result=load({origin->disc,origin->part,origin->offset});
    if(word(rt,object+0x14)!=result->lineHeight || word(rt,object+0x18)!=result->count || word(rt,object+0x20)!=result->stride ||
       word(rt,object+0x24)!=result->width || word(rt,object+0x28)!=result->height || word(rt,object+0x2c)!=result->border)
        throw std::runtime_error("native font metrics differ from converted provenance");
    return result;
}
}

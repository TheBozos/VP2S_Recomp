#include "native_glyphs.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <source_location>
namespace {
void check(bool ok,std::source_location loc=std::source_location::current()){
    if(!ok)throw std::runtime_error("native glyph asset assertion at "+std::to_string(loc.line()));
}
template<class F>void rejects(F fn){bool failed=false;try{fn();}catch(const std::runtime_error &){failed=true;}check(failed);}
void word(Runtime &rt,uint32_t address,uint32_t value){std::memcpy(rt.memory.hostPointer(address,4),&value,4);}
}
int main(int argc,char **argv){try{
    check(argc==2);const std::filesystem::path root=argv[1];vp2s::NativeGlyphs glyphs(root),other(root);
    const vp2s::GlyphFontKey key{0x21551000,0,0x80};auto font=glyphs.load(key);
    check(font==glyphs.load(key) && font!=other.load(key));
    check(font->width==3 && font->height==2 && font->cellWidth==4 && font->cellHeight==2 && font->count==2 && font->stride==3 && font->border==1 && font->lineHeight==2);
    auto picture=font->picture(0,1);check(picture==font->picture(0,1));
    check(picture->width==4 && picture->height==2 && picture->rgba[0]==0x20000011 && picture->rgba[3]==0 && picture->rgba[4]==0x20000014 && picture->rgba[7]==0);
    auto shadow=font->picture(1,2);check(shadow->rgba[0]==0x20000027 && shadow->rgba[4]==0x2000002a && shadow->key!=picture->key);
    rejects([&]{font->picture(2,1);});rejects([&]{font->picture(0,0);});rejects([&]{font->picture(0,3);});
    rejects([&]{glyphs.load({0x21551000,0,0x81});});
    for(const char *directory:{"bad_header","bad_size","extra"}){
        vp2s::NativeGlyphs invalid(root/directory);rejects([&]{invalid.load(key);});
    }
    rejects([&]{vp2s::NativeGlyphs invalid(root/"bad_manifest");});
    Runtime rt;vp2s::NativeAssets assets;
    assets.discRead(0x170000,1,0x21551000/2048);assets.begin(1,0x170000,0x180000,0);assets.finish(1,true,0x90);
    // Poison the runtime source, including its glyph bits: only provenance and
    // the game's numeric font record are consumed by this native loader.
    std::memset(rt.memory.hostPointer(0x180000,0x90),0x5a,0x90);
    word(rt,0x190000,0x180000);word(rt,0x190010,0x180080);word(rt,0x190014,2);word(rt,0x190018,2);
    word(rt,0x190020,3);word(rt,0x190024,3);word(rt,0x190028,2);word(rt,0x19002c,1);
    check(glyphs.font(&rt,assets,0x190000)==font);
    for(const auto [offset,value]:{std::pair{0x14u,2u},{0x18u,2u},{0x20u,3u},{0x24u,3u},{0x28u,2u},{0x2cu,1u}}){
        word(rt,0x190000+offset,value+1);rejects([&]{glyphs.font(&rt,assets,0x190000);});word(rt,0x190000+offset,value);
    }
    assets.overwrite(0x180080,6);rejects([&]{glyphs.font(&rt,assets,0x190000);});
    check(picture->rgba[0]==0x20000011 && shadow->rgba[0]==0x20000027);
    std::puts("native glyph pack bounds/palette/padding, actual file provenance, numeric metrics, source invalidation, immutable pixels and per-host lifetime passed");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

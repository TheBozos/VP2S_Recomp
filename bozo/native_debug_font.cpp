#include "native_debug_font.h"
#include "native_assets.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <stdexcept>
#include <format>
#include <cstdio>
#include <cstdlib>
namespace vp2s {
namespace {
const uint8_t *record(Runtime *rt,uint32_t at,uint32_t bytes){
    at=NativeAssets::address(at);
    if(!rt || at<0x100000 || at>=0x2000000 || bytes>0x2000000-at)throw std::runtime_error("native debug-font record outside EE RAM");
    return rt->memory.hostPointer(at,bytes);
}
template<class T>T value(Runtime *rt,uint32_t at){T v;std::memcpy(&v,record(rt,at,sizeof(v)),sizeof(v));return v;}
uint32_t writer(Runtime *rt,uint32_t object){
    if(value<uint32_t>(rt,object+0x38)!=0x159140)throw std::runtime_error("unowned native debug-font type");
    return value<uint32_t>(rt,object+0x34);
}
}
void NativeDebugFonts::begin(Runtime *rt,NativeTextures &textures,uint32_t stack,uint32_t object,uint32_t background){
    if(initializers.contains(stack))throw std::runtime_error("active debug-font initializer stack reused");
    for(const auto &[sp,p]:publications)if(p.font->object==object)throw std::runtime_error("debug-font reinitialized during publication");
    const uint32_t source=value<uint32_t>(rt,object+0x30);
    if(value<uint32_t>(rt,source)!=0x534946)throw std::runtime_error("native debug font is not a FIS asset");
    const TextureKey key{value<uint32_t>(rt,source+0x14),value<uint32_t>(rt,source+0x18)};
    if(key!=TextureKey{0x464b4e41,0})throw std::runtime_error("unported native debug-font atlas");
    initializers.emplace(stack,Initializer{object,background,textures.load(key)});
    fonts.erase(object);
}
void NativeDebugFonts::finish(Runtime *rt,uint32_t stack){
    auto it=initializers.find(stack);if(it==initializers.end())throw std::runtime_error("debug-font return has no initializer");
    const Initializer input=it->second;initializers.erase(it);
    const uint32_t buffer=writer(rt,input.object);if(!buffer)return;
    auto output=std::make_shared<NativeDebugFont>();output->object=input.object;output->writer=buffer;
    output->background=input.background;output->atlas=input.atlas;
    output->begin=value<uint32_t>(rt,buffer+12);output->end=value<uint32_t>(rt,buffer+8);
    output->glyphWidth=value<uint16_t>(rt,input.object+8);output->glyphHeight=value<uint16_t>(rt,input.object+10);
    fonts[input.object]=std::move(output);
}
std::shared_ptr<const NativeDebugFont> NativeDebugFonts::font(uint32_t object)const{
    auto it=fonts.find(object);return it==fonts.end()?nullptr:it->second;
}
void NativeDebugFonts::publish(Runtime *rt,uint32_t stack,uint32_t object){
    if(publications.contains(stack))throw std::runtime_error("active native debug-font publication stack reused");
    auto current=font(object);if(!current || writer(rt,object)!=current->writer)throw std::runtime_error("native debug font has no accepted initializer");
    const uint32_t begin=value<uint32_t>(rt,current->writer+12),end=value<uint32_t>(rt,current->writer+8);
    if(begin!=current->begin || end!=current->end || current->background || end<begin || (end-begin!=0 && end-begin!=80))
        throw std::runtime_error("native debug-font glyph/background producer required");
    record(rt,begin,end-begin);
    const uint64_t capacity=uint64_t(value<uint32_t>(rt,current->writer))+uint64_t(value<uint32_t>(rt,current->writer+16))*8;
    if(end!=begin && uint64_t(end)+16>capacity)throw std::runtime_error("native debug-font publication exceeds its writer");
    // This owner holds the immutable native atlas and glyph metrics. The
    // admitted range is only the constructor's state prefix: no image draw is
    // due. Any appended glyphs or optional background require their own owner.
    publications.emplace(stack,Publication{std::move(current)});
}
void NativeDebugFonts::retire(Runtime *rt,R5900Context *c){
    auto it=publications.find(c->r[29].uw[0]);
    if(it==publications.end() && value<uint32_t>(rt,c->r[29].uw[0]+0x20)==0x3d3648 && c->r[9].uw[0]>=0x100090){
        const auto object=c->r[9].uw[0]-0x90;record(rt,object,0xf8);
        throw std::runtime_error(std::format("unported native fade: object={:x} type={:x} modes={},{},{} flags={:x} colour={:08x} box={},{},{},{} depth={:x} writerBytes={}",object,value<uint32_t>(rt,object),value<int8_t>(rt,object+0xf2),value<int8_t>(rt,object+0xf3),value<int8_t>(rt,object+0xf4),value<uint8_t>(rt,object+0xf5),value<uint32_t>(rt,object+0xa0),value<int32_t>(rt,object+0xb0),value<int32_t>(rt,object+0xb4),value<int32_t>(rt,object+0xb8),value<int32_t>(rt,object+0xbc),value<uint32_t>(rt,object+0xc0),value<uint32_t>(rt,c->r[17].uw[0]+8)-value<uint32_t>(rt,c->r[17].uw[0]+12)));
    }
    if(it==publications.end() && std::getenv("VP2S_BOZO_TRACE_QUEUES") &&
       value<uint32_t>(rt,c->r[29].uw[0]+0x20)==0x3721a8 && c->r[9].uw[0]>=0x100090){
        const auto object=c->r[9].uw[0]-0x90,buffer=c->r[17].uw[0];record(rt,object,0xf8);
        std::fprintf(stderr,"[vp2s:producer] menu screen object=%x type=%x job=%x modes=%u/%u/%u colour=%08x rect=%d,%d,%d,%d z=%x alpha=%u writer-bytes=%u\n",
            object,value<uint32_t>(rt,object),value<uint32_t>(rt,object+0x90),value<uint8_t>(rt,object+0xf2),value<uint8_t>(rt,object+0xf3),value<uint8_t>(rt,object+0xf4),
            value<uint32_t>(rt,object+0x9c),value<int32_t>(rt,object+0xac),value<int32_t>(rt,object+0xb0),value<int32_t>(rt,object+0xb4),value<int32_t>(rt,object+0xb8),
            value<uint32_t>(rt,object+0xbc),value<uint8_t>(rt,object+0xf1),value<uint32_t>(rt,buffer+8)-value<uint32_t>(rt,buffer+12));
    }
    if(it==publications.end() || it->second.retired)throw std::runtime_error(std::format("unowned native graphics builder submission: writer={:x} caller={:x}",c->r[17].uw[0],value<uint32_t>(rt,c->r[29].uw[0]+0x20)));
    const auto &font=*it->second.font;
    if(font.end==font.begin || c->r[17].uw[0]!=font.writer || c->r[16].uw[0]!=font.end+16 ||
       value<uint32_t>(rt,font.writer+8)!=font.end || value<uint32_t>(rt,font.writer+12)!=font.begin ||
       c->r[5].uw[0]!=2 || c->r[6].uw[0] || c->r[8].uw[0]!=(font.end+16-font.begin)/16 || c->r[2].uw[0]!=font.begin)
        throw std::runtime_error("native debug-font submission identity changed");
    it->second.retired=true;
}
void NativeDebugFonts::returnFromFlush(uint32_t stack){
    auto it=publications.find(stack);
    // Other CPU-only/no-data flush callers are retained. A nonempty unowned
    // submission cannot reach this return because retire rejects it first.
    if(it==publications.end())return;
    if(it->second.font->end!=it->second.font->begin && !it->second.retired)throw std::runtime_error("debug-font transport was not retired at its producer");
    publications.erase(it);
}
std::optional<bool> nativeDebugFontInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x29af80 && pc!=0x29b2a8 && pc!=0x362328 && pc!=0x3d93f4 && pc!=0x120f4c && pc!=0x120f58)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<DebugFontOwner *>(&d->host()):nullptr;
    if(!c || !owner)throw std::runtime_error("native debug font requires an owner");
    auto &fonts=owner->nativeDebugFonts();const uint32_t sp=c->r[29].uw[0];
    if(pc==0x29af80){
        auto *textures=dynamic_cast<TextureOwner *>(&d->host());if(!textures)throw std::runtime_error("native debug font requires offline textures");
        fonts.begin(rt,textures->nativeTextures(),sp-0x60,c->r[4].uw[0],c->r[7].uw[0]);
    }else if(pc==0x29b2a8)fonts.finish(rt,sp);
    else if(pc==0x362328)fonts.publish(rt,sp-0x30,c->r[17].uw[0]);
    else if(pc==0x3d93f4)fonts.publish(rt,sp-0x30,c->r[16].uw[0]);
    else if(pc==0x120f4c)fonts.retire(rt,c);
    else fonts.returnFromFlush(sp);
    return false; // The original scalar instruction and all CPU bookkeeping remain.
}
}

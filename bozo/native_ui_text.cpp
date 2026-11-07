#include "native_ui_text.h"
#include "bozo/ee/dispatcher.h"
#include <bit>
#include <cstring>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
namespace vp2s {
namespace {
const uint8_t *record(Runtime *rt,uint32_t at,uint32_t size){
    if(!rt || at<0x100000 || at>=0x2000000 || size>0x2000000-at)throw std::runtime_error("native UI text metadata outside EE RAM");
    return rt->memory.hostPointer(at,size);
}
template<class T>T value(Runtime *rt,uint32_t at){T result;std::memcpy(&result,record(rt,at,sizeof(T)),sizeof(T));return result;}
void type(Runtime *rt,uint32_t element){if(value<uint32_t>(rt,element)!=0x15a450)throw std::runtime_error("unported native UI text class");}
}
void NativeUiTexts::enter(Runtime *rt,uint32_t stack,uint32_t element,uint32_t items,int32_t count){
    type(rt,element);
    if(count>8192 || builds.contains(stack))throw std::runtime_error("native UI text count/stack outside admitted lifetime");
    if(count>0)record(rt,items,uint32_t(count)*0x30);
    builds.emplace(stack,Build{NativeUiText{.element=element},items,count});
}
void NativeUiTexts::activate(uint32_t stack,uint32_t element,uint32_t items,int32_t count){
    auto &build=builds.at(stack);
    if(build.active || build.text.element!=element || build.items!=items || build.count!=count)
        throw std::runtime_error("native UI text changed through setup callback");
    build.active=true;
}
void NativeUiTexts::capture(Runtime *rt,NativeGlyphs &glyphs,const NativeAssets &assets,uint32_t stack,uint32_t element,uint32_t item,uint32_t index,bool image){
    auto &build=builds.at(stack);type(rt,element);
    if(!build.active || build.count<=0 || build.text.element!=element || index>=uint32_t(build.count) ||
       item!=build.items+index*0x30 || (build.hasLast && index<=build.last))
        throw std::runtime_error("native UI text item escaped its emitter lifetime");
    const auto code=value<uint16_t>(rt,item+0x2c);
    if(image){
        if(code!=0x4010)throw std::runtime_error("native inline image has no matching text item");
        build.text.inlineImages=true;
    }else{
        if(code&0x4000)throw std::runtime_error("native glyph capture received a control item");
        NativeUiGlyph result;
        std::memcpy(result.box.data(),record(rt,item,16),16);std::memcpy(result.scale.data(),record(rt,item+0x10,8),8);
        for(float f:result.box)if((std::bit_cast<uint32_t>(f)&0x7f800000)==0x7f800000)throw std::runtime_error("nonfinite native glyph bounds");
        for(float f:result.scale)if((std::bit_cast<uint32_t>(f)&0x7f800000)==0x7f800000)throw std::runtime_error("nonfinite native glyph scale");
        result.cellWidth=value<int32_t>(rt,item+0x1c);result.index=value<uint16_t>(rt,item+0x2e);
        result.colour=value<uint32_t>(rt,item+0x20);result.shadowMode=value<uint32_t>(rt,item+0x24);result.shadowColour=value<uint32_t>(rt,item+0x28);
        if(result.shadowMode>2 || result.cellWidth<0)throw std::runtime_error("unported native glyph shadow/width");
        result.font=glyphs.font(rt,assets,value<uint32_t>(rt,item+0x18));
        result.picture=result.font->picture(result.index,1);
        if(result.shadowMode)result.shadow=result.font->picture(result.index,result.shadowMode==1?1:2);
        build.text.glyphs.push_back(std::move(result));
    }
    build.last=index;build.hasLast=true;
}
void NativeUiTexts::finish(uint32_t stack,uint32_t element){
    auto it=builds.find(stack);if(it==builds.end() || !it->second.active || it->second.text.element!=element)
        throw std::runtime_error("native UI text finished outside its original callback return");
    auto result=std::make_shared<NativeUiText>(std::move(it->second.text));
    if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:UI] captured text=%x glyphs=%zu inline=%u\n",element,result->glyphs.size(),result->inlineImages);
    texts[element]=std::move(result);builds.erase(it);
}
void NativeUiTexts::release(uint32_t element){
    for(const auto &[stack,build]:builds)if(build.text.element==element)throw std::runtime_error("native UI text destroyed during capture");
    texts.erase(element);
}
std::shared_ptr<const NativeUiText> NativeUiTexts::text(uint32_t element)const{
    auto it=texts.find(element);return it==texts.end()?nullptr:it->second;
}
std::optional<bool> nativeUiTextInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x2e74f0 && pc!=0x2e7520 && pc!=0x2e7550 && pc!=0x2e757c && pc!=0x2e75a0 && pc!=0x2f0de0)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<UiTextOwner *>(&d->host()):nullptr;
    auto *assets=d?dynamic_cast<AssetOwner *>(&d->host()):nullptr;
    if(!c || !owner || !assets)throw std::runtime_error("native UI text requires glyph/provenance owners");
    auto &texts=owner->nativeUiTexts();const auto sp=c->r[29].uw[0];
    if(pc==0x2e74f0){
        if(c->pc!=pc || sp<0x50 || (sp&15) || !rt->memory.hostPointer(sp-0x50,0x50))throw std::runtime_error("native text emitter requires its complete original entry");
        texts.enter(rt,sp-0x50,c->r[4].uw[0],c->r[5].uw[0],c->r[6].sw[0]);
    }else if(pc==0x2f0de0){
        if(c->pc!=pc)throw std::runtime_error("native text release requires its destructor entry");
        if(c->r[4].uw[0]){type(rt,c->r[4].uw[0]);texts.release(c->r[4].uw[0]);}
    }else if(pc==0x2e7520)texts.activate(sp,c->r[19].uw[0],c->r[18].uw[0],c->r[17].sw[0]);
    else if(pc==0x2e75a0)texts.finish(sp,c->r[19].uw[0]);
    else{
        if(c->r[25].uw[0]!=(pc==0x2e757c?0x2e7e90u:0x2e76a0u))throw std::runtime_error("unported overridden native text item callback");
        texts.capture(rt,owner->nativeGlyphs(),assets->nativeAssets(),sp,c->r[19].uw[0],c->r[18].uw[0],c->r[16].uw[0],pc==0x2e7550);
    }
    return false; // Every original CPU instruction and real virtual callback stays compiled.
}
}

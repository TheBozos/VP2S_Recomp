#include "native_ui_text.h"
#include "native_ui_windows.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/ops_fpu.h"
#include <bit>
#include <cstring>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <format>
namespace vp2s {
namespace {
const uint8_t *record(Runtime *rt,uint32_t at,uint32_t size){
    if(!rt || at<0x100000 || at>=0x2000000 || size>0x2000000-at)throw std::runtime_error("native UI text metadata outside EE RAM");
    return rt->memory.hostPointer(at,size);
}
template<class T>T value(Runtime *rt,uint32_t at){T result;std::memcpy(&result,record(rt,at,sizeof(T)),sizeof(T));return result;}
void type(Runtime *rt,uint32_t element){const auto actual=value<uint32_t>(rt,element);if(actual!=0x15a450)throw std::runtime_error(std::format("unported native UI text class {:x} at {:x}",actual,element));}
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
    build.last=index;build.hasLast=true;build.lastIsGlyph=!image;
}
void NativeUiTexts::place(uint32_t stack,uint32_t element,const NativeUiGlyphPlacement &placement){
    auto &build=builds.at(stack);
    if(!build.active || build.text.element!=element || !build.lastIsGlyph || build.text.glyphs.empty() || build.text.glyphs.back().placed)
        throw std::runtime_error("native glyph placement escaped its original CPU callback");
    for(auto p:placement.fixedBox)if(p<0 || p>65535)throw std::runtime_error("native glyph position outside admitted fixed-point range");
    for(unsigned i=0;i<4;++i){
        const auto shifted=int64_t(placement.fixedBox[i])+placement.fixedShadowOffset[i&1];
        if(shifted<0 || shifted>65535)throw std::runtime_error("native glyph shadow outside admitted fixed-point range");
    }
    if(placement.depth>0xffffff)throw std::runtime_error("native glyph depth outside admitted range");
    auto &glyph=build.text.glyphs.back();
    if(uint64_t(glyph.cellWidth)+2*glyph.font->border>glyph.font->width || uint64_t(glyph.font->lineHeight)+2*glyph.font->border>glyph.font->height)
        throw std::runtime_error("native glyph UV extent exceeds its converted cell");
    glyph.placement=placement;glyph.placed=true;
}
std::array<bozo::render::Vertex,6> nativeUiGlyphVertices(const NativeUiGlyph &glyph,bool shadow){
    if(!glyph.placed || (shadow && !glyph.shadow))throw std::runtime_error("native glyph has no completed placement/picture");
    const auto &p=glyph.placement;
    const bool right[6]={false,true,false,true,true,false},bottom[6]={false,false,true,false,true,true};
    std::array<bozo::render::Vertex,6> vertices;
    for(unsigned i=0;i<6;++i){
        auto &v=vertices[i];
        v.x=(float(p.fixedBox[right[i]?2:0])+(shadow?float(p.fixedShadowOffset[0]):0.f))/16.f;
        v.y=(float(p.fixedBox[bottom[i]?3:1])+(shadow?float(p.fixedShadowOffset[1]):0.f))/16.f;
        v.z=float(shadow?(p.depth-1)&0xffffff:p.depth);v.rgba=shadow?p.shadowColour:p.colour;
        v.u=(right[i]?float(glyph.cellWidth+2*glyph.font->border)-0.5f:0.5f)/glyph.picture->width;
        v.v=(bottom[i]?float(glyph.font->lineHeight+2*glyph.font->border)-0.5f:0.5f)/glyph.picture->height;
    }
    return vertices;
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
void NativeUiTexts::icons(uint32_t element,std::vector<NativeUiSprite> icons){
    const auto old=text(element);if(!old)throw std::runtime_error("native text icons have no finished glyph snapshot");
    auto result=std::make_shared<NativeUiText>(*old);result->icons=std::move(icons);result->iconsCaptured=true;
    texts[element]=std::move(result);
}
void NativeUiTexts::spriteElement(uint32_t element,std::vector<NativeUiSprite> icons){
    auto result=std::make_shared<NativeUiText>();result->element=element;result->icons=std::move(icons);result->iconsCaptured=true;
    texts[element]=std::move(result);
}
// Chapter title bar, element class 168940: the arithmetic of its packet builder
// mod004 48fa00 (FPU definitions from the engine spec), emitted as native
// vertices instead of a GS packet. Title record T = element+54: shown when
// T+392 bit 0; centre x = T+40 + T+340 + 0.5*T+348, width T+348 * T+38c,
// both scaled 512/640 around 0x800 - screenW/2; rows T+44 + T+344 (+ T+34c)
// scaled 512.96/480 around 0x800 - screenH/2; opacity T+e0 * T+350 / 128.
// A 4x3 Gouraud grid: the last tenth at each end fades to 0.65 on the middle
// row and to colour 0 on the edge rows; colour f0f0f0; depth T+c4.
std::vector<bozo::render::Vertex> nativeChapterBar(Runtime *rt,uint32_t element){
    namespace F=bozo::ps2f;uint32_t flags=0;
    const auto add=[&](uint32_t a,uint32_t b){return F::add(a,b,flags);};const auto sub=[&](uint32_t a,uint32_t b){return F::sub(a,b,flags);};
    const auto mul=[&](uint32_t a,uint32_t b){return F::mul(a,b,flags);};const auto div=[&](uint32_t a,uint32_t b){return F::div(a,b,flags);};
    const auto title=value<uint32_t>(rt,element+0x54);if(title<0x100000)throw std::runtime_error("native chapter bar has no title record");
    record(rt,title,0x3a0);
    if(!(value<uint8_t>(rt,title+0x392)&1))return {};
    const auto display=value<uint32_t>(rt,0x35de60);if(display<0x100000)throw std::runtime_error("native chapter bar has no display record");
    const int32_t screenWidth=value<int16_t>(rt,display+0x3c6c),screenHeight=value<int16_t>(rt,display+0x3c6e);
    const auto at=[&](uint32_t offset){const auto bits=value<uint32_t>(rt,title+offset);
        if((bits&0x7f800000)==0x7f800000)throw std::runtime_error("unqualified native chapter bar parameter");return bits;};
    constexpr uint32_t half=0x3f000000,f512=0x44000000,f640=0x44200000,f480=0x43f00000,f128=0x43000000,f16=0x41800000,
                       yScale=0x44003d70,tenth=0x3dcccccd,endShade=0x3f266666;
    const auto width=mul(at(0x348),at(0x38c));
    const auto left=div(mul(f512,sub(add(mul(half,at(0x348)),add(at(0x40),at(0x340))),mul(half,width))),f640);
    const auto x0=add(F::fromInt(uint32_t(0x800-screenWidth/2)),left),x1=add(x0,div(mul(f512,width),f640));
    const auto top=add(at(0x44),at(0x344)),originY=F::fromInt(uint32_t(0x800-screenHeight/2));
    const auto y0=add(originY,div(mul(yScale,top),f480)),y1=add(originY,div(mul(yScale,add(at(0x34c),top)),f480));
    const auto opacity=div(mul(at(0xe0),at(0x350)),f128);
    // cvt.w.s and the >= 2^31 path both keep the low byte in bits 24..31.
    const auto alphaByte=[&](uint32_t v){
        const uint32_t i=(!(v&0x80000000u) && v>=0x4f000000u)?(F::toInt(F::sub(v,0x4f000000,flags))|0x80000000u):F::toInt(v);return i<<24;};
    const uint32_t full=alphaByte(opacity)|0xf0f0f0,end=alphaByte(mul(opacity,endShade))|0xf0f0f0;
    const auto fade=mul(tenth,sub(x1,x0));
    const uint32_t xs[4]={x0,add(x0,fade),sub(x1,fade),x1},ys[3]={y0,add(y0,mul(half,sub(y1,y0))),y1};
    const uint32_t colours[3][4]={{0,full,full,0},{end,full,full,end},{0,full,full,0}};
    const auto fixed=[&](uint32_t v){return float(int32_t(F::toInt(mul(f16,v))))/16;};
    const float depth=float(value<uint32_t>(rt,title+0xc4));
    std::vector<bozo::render::Vertex> triangles;
    const auto vertex=[&](unsigned col,unsigned row){bozo::render::Vertex v{};v.x=fixed(xs[col]);v.y=fixed(ys[row]);v.z=depth;v.rgba=colours[row][col];return v;};
    for(unsigned row=0;row<2;++row)for(unsigned col=0;col<3;++col){
        const auto a=vertex(col,row),b=vertex(col+1,row),c=vertex(col,row+1),d=vertex(col+1,row+1);
        triangles.insert(triangles.end(),{a,b,c,b,d,c});
    }
    return triangles;
}
std::array<bozo::render::Vertex,6> nativeUiSpriteVertices(const NativeUiSprite &sprite){
    const unsigned corners[6]={0,1,2,1,3,2};std::array<bozo::render::Vertex,6> vertices;
    for(unsigned i=0;i<6;++i){const auto corner=corners[i];auto &v=vertices[i];
        v.x=float(sprite.fixedCorners[corner][0])/16;v.y=float(sprite.fixedCorners[corner][1])/16;
        v.z=float(sprite.depth);v.rgba=sprite.colour;v.u=sprite.uv[corner&1?2:0];v.v=sprite.uv[corner>=2?3:1];
    }
    return vertices;
}
NativeUiLayer NativeUiTexts::layer(Runtime *rt,uint32_t layer)const{
    record(rt,layer,0xe4);
    const auto layerType=value<uint32_t>(rt,layer+0x10),objectType=value<uint32_t>(rt,layer+0x20),jobType=value<uint32_t>(rt,layer+0xb0);
    // The story/skip dialogue layer has the same original draw/build/completion
    // methods, via the mod002 adjustment thunks. Child admission is unchanged.
    const bool base=layerType==0x15ba00 && objectType==0x15ba1c && jobType==0x15ba98;
    const bool story=layerType==0x15bca0 && objectType==0x15bcbc && jobType==0x15bd38;
    // The mod004 field layer (168970) also forwards to the same mod002-resident
    // methods; its object +10 method only adds a resource release after the
    // base one (teardown), which the native draw/completion does not use.
    const bool field=layerType==0x168970 && objectType==0x16898c && jobType==0x168a08;
    if(value<uint32_t>(rt,layer)!=0x152060 || (!base && !story && !field) ||
       value<uint8_t>(rt,layer+0x83)!=1 || value<uint8_t>(rt,layer+0xca)!=2)
        throw std::runtime_error(std::format("unported native UI layer type/build mode: layer={:x} types={:x}/{:x}/{:x}/{:x} mode83={} modeca={}",
            layer,value<uint32_t>(rt,layer),value<uint32_t>(rt,layer+0x10),value<uint32_t>(rt,layer+0x20),value<uint32_t>(rt,layer+0xb0),
            value<uint8_t>(rt,layer+0x83),value<uint8_t>(rt,layer+0xca)));
    const auto current=value<uint32_t>(rt,0x35ddf8),object=layer+0x20;
    if(value<uint8_t>(rt,object+0x65)!=1 || value<uint32_t>(rt,current+0x40ac)!=object || value<uint8_t>(rt,current+0x40dd)!=1)
        throw std::runtime_error("native UI layer escaped its pending scene object");
    NativeUiLayer result{layer,object};std::set<uint32_t> seen;
    for(uint32_t element=value<uint32_t>(rt,layer+8);element!=layer;element=value<uint32_t>(rt,element+8)){
        if(!seen.insert(element).second || seen.size()>4096)throw std::runtime_error("native UI layer has an invalid child cycle");
        record(rt,element,0x40);
        if(value<uint32_t>(rt,element+0x14)!=layer)throw std::runtime_error("native UI element has a different layer owner");
        if(!value<uint8_t>(rt,element+0x39) || !value<uint8_t>(rt,element+0x3d) || !value<uint8_t>(rt,element+0x3b))continue;
        if(value<uint32_t>(rt,element)==0x15a1b0 || value<uint32_t>(rt,element)==0x15ac30){
            auto *d=dynamic_cast<bozo::ee::Dispatcher *>(rt->services);auto *o=d?dynamic_cast<UiWindowOwner *>(&d->host()):nullptr;
            const auto captured=o?o->nativeUiWindows().window(element):nullptr;
            if(!captured)throw std::runtime_error("native window has no finished scalar snapshot");
            auto item=std::make_shared<NativeUiText>();item->element=element;item->window=captured;result.texts.push_back(std::move(item));continue;
        }
        if(value<uint32_t>(rt,element)==0x168940){
            auto item=std::make_shared<NativeUiText>();item->element=element;item->additiveTriangles=nativeChapterBar(rt,element);
            result.texts.push_back(std::move(item));continue;
        }
        if(value<uint32_t>(rt,element)==0x15a690){
            auto captured=text(element);if(!captured || !captured->iconsCaptured || !captured->glyphs.empty())throw std::runtime_error("native image element has no finished sprite snapshot");
            result.texts.push_back(std::move(captured));continue;
        }
        type(rt,element);auto captured=text(element);
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES")){
            static unsigned traced=0;
            if(traced++<4){
                const auto control=value<uint32_t>(rt,element+0x100);
                if(control){
                    const auto count=value<uint32_t>(rt,control+0x28),sprites=value<uint32_t>(rt,control+0x24);
                    std::fprintf(stderr,"[vp2s:UI] text=%x reveal=%x inline-count=%u capacity=%u sprites=%x\n",element,value<uint32_t>(rt,element+0x74),count,value<uint32_t>(rt,control+0x2c),sprites);
                    if(count && count<=16){record(rt,sprites,count*0x70);
                        for(uint32_t i=0;i<count;++i){const auto sprite=sprites+i*0x70;
                            std::fprintf(stderr,"[vp2s:UI] inline sprite=%x enabled=%u item=%x fis=%x dtt=%x parts=%x part=%u\n",sprite,value<uint8_t>(rt,sprite+0x66),value<uint32_t>(rt,sprite+0x68),value<uint32_t>(rt,sprite),value<uint32_t>(rt,sprite+4),value<uint32_t>(rt,sprite+8),value<uint16_t>(rt,sprite+0x54));}
                    }
                }
            }
        }
        if(!captured || captured->inlineImages)throw std::runtime_error("native UI layer has uncaptured text/inline images");
        const auto control=value<uint32_t>(rt,element+0x100);
        if(control && value<int32_t>(rt,control+0x2c)>0 && value<int32_t>(rt,control+0x28)>0 && !captured->iconsCaptured)
            throw std::runtime_error("native UI layer has uncaptured button sprites");
        for(const auto &glyph:captured->glyphs)if(!glyph.placed)throw std::runtime_error("native UI layer has an unfinished glyph placement");
        result.texts.push_back(std::move(captured));
    }
    return result;
}
std::optional<bool> nativeUiTextInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x2e74f0 && pc!=0x2e7520 && pc!=0x2e7550 && pc!=0x2e757c && pc!=0x2e75a0 && pc!=0x2f0de0 && pc!=0x2e83e4 && pc!=0x2ec9e4)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<UiTextOwner *>(&d->host()):nullptr;
    auto *assets=d?dynamic_cast<AssetOwner *>(&d->host()):nullptr;
    if(!c || !owner || !assets)throw std::runtime_error("native UI text requires glyph/provenance owners");
    auto &texts=owner->nativeUiTexts();const auto sp=c->r[29].uw[0];
    if(pc==0x2ec9e4){
        const auto layer=c->r[4].uw[0];
        if(c->pc!=0x2ec9c0 || c->r[5].uw[0]!=layer+0xb0)throw std::runtime_error("native UI layer requires its complete original producer");
        const auto submission=texts.layer(rt,layer);owner->drawNativeUiLayer(submission);
        c->r[4].sd[0]=static_cast<int32_t>(submission.object);
        return true; // Copied native draws accepted; original compiled completion follows.
    }else if(pc==0x2e74f0){
        if(c->pc!=pc || sp<0x50 || (sp&15) || !rt->memory.hostPointer(sp-0x50,0x50))throw std::runtime_error("native text emitter requires its complete original entry");
        texts.enter(rt,sp-0x50,c->r[4].uw[0],c->r[5].uw[0],c->r[6].sw[0]);
    }else if(pc==0x2f0de0){
        if(c->pc!=pc)throw std::runtime_error("native text release requires its destructor entry");
        if(c->r[4].uw[0]){
            const auto element=c->r[4].uw[0];
            // The mod002 derived destructor writes its own vtable before
            // calling this base destructor. Admit only that proven teardown;
            // it does not qualify the derived class's rendering callbacks.
            if(value<uint32_t>(rt,element)!=0x15bd70 || c->r[31].uw[0]!=0x373ee0)type(rt,element);
            auto *sprites=dynamic_cast<UiSpriteOwner *>(&d->host());
            if(sprites && sprites->nativeUiSprites().buildingText(element))throw std::runtime_error("native text destroyed during icon capture");
            texts.release(element);
        }
    }else if(pc==0x2e83e4){
        // Original scalar layout has finished its FPU rounding and RGB/alpha
        // calculation. Observe those game-code results, never packet contents.
        NativeUiGlyphPlacement p;
        p.fixedBox={c->r[16].sw[0],c->r[25].sw[0],c->r[24].sw[0],c->r[15].sw[0]};
        p.fixedShadowOffset={c->r[14].sw[0],c->r[13].sw[0]};
        p.depth=c->r[6].uw[0];p.colour=c->r[8].uw[0]|c->r[3].uw[0];p.shadowColour=c->r[9].uw[0];
        texts.place(sp+0x80,c->r[4].uw[0],p);
    }else if(pc==0x2e7520)texts.activate(sp,c->r[19].uw[0],c->r[18].uw[0],c->r[17].sw[0]);
    else if(pc==0x2e75a0)texts.finish(sp,c->r[19].uw[0]);
    else{
        if(c->r[25].uw[0]!=(pc==0x2e757c?0x2e7e90u:0x2e76a0u))throw std::runtime_error("unported overridden native text item callback");
        texts.capture(rt,owner->nativeGlyphs(),assets->nativeAssets(),sp,c->r[19].uw[0],c->r[18].uw[0],c->r[16].uw[0],pc==0x2e7550);
    }
    return false; // Every original CPU instruction and real virtual callback stays compiled.
}
}

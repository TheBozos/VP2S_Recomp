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
// 165ae0 (mod003) derives from 15a2f0 (mod001, the reveal text), which derives
// from 15a450; both keep the emitter 2e74f0, the item
// callbacks (+2c 2e7e90, +30 2e76a0, checked per item), placement and draw
// (+10 2f32d0); its own +1c/+20 (2ed050/2ecfa0) only decide when to rebuild
// the 0x30-byte item array at +70..+74 and hand it to the same emitter.
void type(Runtime *rt,uint32_t element){const auto actual=value<uint32_t>(rt,element);if(actual!=0x15a450 && actual!=0x15a2f0 && actual!=0x165ae0)throw std::runtime_error(std::format("unported native UI text class {:x} at {:x}",actual,element));}
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
    // The mod003 town/message layer (15fe10/15fe2c/15fea8): its object +18 and
    // job +c are thunks 3b3150 (j 2ec9c0) and 3b31c0 (j 2ec9a0), the base
    // producer/build; its completion (job +10, 447680) calls the base 2eca00
    // and then runs its own scalar flags, unchanged. Thunk words are checked.
    const auto thunk=[&](uint32_t at,uint32_t target,uint16_t adjust){
        return value<uint32_t>(rt,at)==(0x08000000u|(target>>2)) && value<uint32_t>(rt,at+4)==(0x24840000u|adjust);};
    const bool message=layerType==0x15fe10 && objectType==0x15fe2c && jobType==0x15fea8 &&
        value<uint32_t>(rt,objectType+0x18)==0x3b3150 && value<uint32_t>(rt,jobType+0x0c)==0x3b31c0 &&
        thunk(0x3b3150,0x2ec9c0,0xffe0) && thunk(0x3b31c0,0x2ec9a0,0xff50);
    // The mod003 event layer (167850/16786c/1678e8, constructor 466c80): the same
    // producer/build thunks as the message layer, the base completion 2eca00 as
    // job +14, and object +10 = thunk 466ef0 -> 466c10, the base 2ef0f0 followed
    // by two releases (teardown; the native draw/completion does not use it).
    const bool event=layerType==0x167850 && objectType==0x16786c && jobType==0x1678e8 &&
        value<uint32_t>(rt,objectType+0x18)==0x3b3150 && value<uint32_t>(rt,jobType+0x0c)==0x3b31c0 && value<uint32_t>(rt,jobType+0x14)==0x2eca00 &&
        value<uint32_t>(rt,objectType+0x10)==0x466ef0 && thunk(0x3b3150,0x2ec9c0,0xffe0) && thunk(0x3b31c0,0x2ec9a0,0xff50) &&
        thunk(0x466ef0,0x466c10,0xffe0) && value<uint32_t>(rt,0x466c20)==(0x0c000000u|(0x2ef0f0u>>2));
    // Another mod003 layer (165d30/165d4c/165dc8): the same thunks, the base
    // teardown 2ef0f0 as object +10, and its own completion 44a540, which brackets
    // the base 2eca00 with two 100c90 calls (a context saved and restored around
    // it); that completion stays compiled.
    // Its sibling 165c60/165c7c/165cf8 (the town prompt layer) has the same entries.
    const bool bracketed=((layerType==0x165d30 && objectType==0x165d4c && jobType==0x165dc8) ||
                          (layerType==0x165c60 && objectType==0x165c7c && jobType==0x165cf8)) &&
        value<uint32_t>(rt,objectType+0x18)==0x3b3150 && value<uint32_t>(rt,jobType+0x0c)==0x3b31c0 && value<uint32_t>(rt,jobType+0x14)==0x44a540 &&
        value<uint32_t>(rt,objectType+0x10)==0x2ef0f0 && thunk(0x3b3150,0x2ec9c0,0xffe0) && thunk(0x3b31c0,0x2ec9a0,0xff50) &&
        value<uint32_t>(rt,0x44a58c)==(0x0c000000u|(0x2eca00u>>2));
    if(value<uint32_t>(rt,layer)!=0x152060 || (!base && !story && !field && !message && !event && !bracketed) ||
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
        if(value<uint32_t>(rt,element)==0x167820 || value<uint32_t>(rt,element)==0x165e00 || value<uint32_t>(rt,element)==0x165c30){
            auto captured=plate(element);if(!captured)throw std::runtime_error("native name plate has no finished snapshot");
            result.texts.push_back(std::move(captured));continue;
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
// Event name plate, builder 465150 path A (from 4653e8 to the join 465ee0),
// transcribed operation by operation on PS2 float words (FPU add/sub truncate,
// mul/div/cvt as ps2f defines them). Four SPRITEs (PRIM 356) over the
// resource image: [X, X+L+16] x [Y-4, Y-4+h1] (rect +b0), a cap of width
// f29 = 0.85*w, times 0.75 with the half-width flag (rect +150), then the
// second row shifted left by f29 (rect +140 and the cap again). Two flat
// strips (PRIM 244) from the rectangle writer 475050, dark then light.
NativeUiText nativeNamePlate(const NativeNamePlateInputs &in,std::shared_ptr<const bozo::render::Texture> picture,std::shared_ptr<const bozo::render::Texture> iconPicture){
    namespace F=bozo::ps2f;uint32_t flags=0;
    const auto A=[&](uint32_t a,uint32_t b){return F::add(a,b,flags);};
    const auto S=[&](uint32_t a,uint32_t b){return F::sub(a,b,flags);};
    const auto M=[&](uint32_t a,uint32_t b){return F::mul(a,b,flags);};
    const auto D=[&](uint32_t a,uint32_t b){return F::div(a,b,flags);};
    const auto I=[](uint32_t x){return int32_t(F::toInt(x));};
    const auto W=[](float v){return std::bit_cast<uint32_t>(v);};
    const auto U=[](uint16_t v){return F::fromInt(v);};
    const uint32_t k16=W(16.f),k4=W(4.f),k2=W(2.f),k28=W(28.f),k512=W(512.f),k640=W(640.f),k085=0x3f59999au,k075=0x3f400000u;
    const auto &r1=in.rects[0],&r3=in.rects[1],&r2=in.rects[2];
    // UV helper 410c30: (1<<log2)/256 times each table coordinate, truncated.
    const auto su=D(F::fromInt(1u<<in.log2Width),W(256.f)),sv=D(F::fromInt(1u<<in.log2Height),W(256.f));
    const auto uv=[&](const std::array<uint16_t,6> &r){return std::array<int32_t,4>{I(M(su,U(r[0]))),I(M(sv,U(r[1]))),I(M(su,U(r[2]))),I(M(sv,U(r[3])))};};
    const float texW=float(1u<<in.log2Width)*16,texH=float(1u<<in.log2Height)*16;
    NativeUiText out;
    const auto sprite=[&](int32_t x0,int32_t y0,int32_t x1,int32_t y1,const std::array<uint16_t,6> &r){
        const auto t=uv(r);NativeUiSprite s;s.colour=in.colour;s.depth=in.depth;s.picture=picture;
        s.fixedCorners={std::array<int32_t,2>{x0,y0},{x1,y0},{x0,y1},{x1,y1}};
        s.uv={float(t[0])/texW,float(t[1])/texH,float(t[2])/texW,float(t[3])/texH};out.icons.push_back(std::move(s));};
    const auto X=in.x,Y=in.y,L=in.length;
    const auto f20=A(X,L);auto f25=S(Y,k4);auto f26=A(k16,f20);const auto f27=A(f25,U(r1[5]));
    const auto row1y0=I(M(k16,f25));
    sprite(I(M(k16,X)),row1y0,I(M(k16,f26)),I(M(k16,f27)),r1);
    const auto f29=in.narrowCap?M(k075,M(k085,U(r2[4]))):M(k085,U(r2[4]));
    {const auto x0=I(M(k16,f26));f26=A(f26,f29);sprite(x0,row1y0,I(M(k16,f26)),I(M(k16,A(f25,U(r2[5])))),r2);}
    const auto g27=S(X,f29);f25=A(S(Y,k4),U(r1[5]));f26=A(k16,A(g27,L));const auto f28=A(f25,U(r3[5]));
    const auto row2y0=I(M(k16,f25));
    sprite(I(M(k16,g27)),row2y0,I(M(k16,f26)),I(M(k16,f28)),r3);
    {const auto x0=I(M(k16,f26));f26=A(f26,f29);sprite(x0,row2y0,I(M(k16,f26)),I(M(k16,A(f25,U(r2[5])))),r2);}
    // Strips: integer writer coordinates; the writer adds the screen origin.
    const auto half=[](int32_t v){return v>=0?v>>1:(v+1)>>1;};
    const int32_t ox=0x800-half(in.frameWidth),oy=0x800-half(in.frameHeight);
    const auto fox=F::fromInt(uint32_t(ox)),foy=F::fromInt(uint32_t(oy));
    const auto s27=S(A(X,D(M(k512,A(k16,in.tailOffset)),k640)),fox);
    const auto s26=S(f20,fox),s25=S(A(k28,Y),foy),s2=A(k2,s25);
    const float depth=float(in.depth);const uint32_t alpha=in.colour&0xff000000u;
    const auto strip=[&](int32_t x0,int32_t y0,int32_t x1,int32_t y1,uint32_t rgb){
        const auto v=[&](int32_t x,int32_t y){bozo::render::Vertex p{};p.x=float(x+ox);p.y=float(y+oy);p.z=depth;p.rgba=alpha|rgb;return p;};
        const auto a=v(x0,y0),b=v(x1,y0),c=v(x0,y1),d=v(x1,y1);out.alphaTriangles.insert(out.alphaTriangles.end(),{a,b,c,b,d,c});};
    strip(I(A(k2,s27)),I(s2),I(A(k2,s26)),I(A(k2,s2)),0x010101u);
    strip(I(s27),I(s25),I(s26),I(s2),0xf8f8f8u);
    if(in.icon){
        // x0 = 16+X, x1 = x0 + 0.85*w (w x0.75 with the flag); y0 = ((Y + s0/2) - 0.85*h/2) - 8, y1 = y0 + h.
        const auto &r=in.iconRect;
        const auto iu=D(F::fromInt(1u<<in.iconLog2Width),W(256.f)),iv=D(F::fromInt(1u<<in.iconLog2Height),W(256.f));
        const std::array<int32_t,4> t{I(M(iu,U(r[0]))),I(M(iv,U(r[1]))),I(M(iu,U(r[2]))),I(M(iv,U(r[3])))};
        auto w=U(r[4]);if(in.narrowCap)w=M(w,k075);
        const auto x0=A(k16,X),x1=A(x0,M(k085,w));
        const auto half=in.textSize>=0?in.textSize>>1:(in.textSize+1)>>1;
        const auto y0=S(S(A(Y,F::fromInt(uint32_t(half))),D(M(k085,U(r[5])),k2)),W(8.f)),y1=A(y0,U(r[5]));
        const uint32_t alpha=(F::toInt(M(in.alpha,in.alphaScale))&0xff)<<24;
        const float iw=float(1u<<in.iconLog2Width)*16,ih=float(1u<<in.iconLog2Height)*16;
        NativeUiSprite s;s.colour=alpha|0x808080u;s.depth=in.depth;s.picture=std::move(iconPicture);
        s.fixedCorners={std::array<int32_t,2>{I(M(k16,x0)),I(M(k16,y0))},{I(M(k16,x1)),I(M(k16,y0))},{I(M(k16,x0)),I(M(k16,y1))},{I(M(k16,x1)),I(M(k16,y1))}};
        s.uv={float(t[0])/iw,float(t[1])/ih,float(t[2])/iw,float(t[3])/ih};out.lateIcons.push_back(std::move(s));
    }
    return out;
}
// Builder 449580 (after 449790), on PS2 float words: y0 = 16*(top-6), y1 = y0 +
// 16*(h+5) (h of rect +a0); the centre is rect +a0 from left to the middle
// 0.5*(left+right) and mirrored (u1..u0) from there to right; the wings are
// rect +90 over [left-w, left] and, mirrored, [right, right+w].
NativeUiText nativeOrnament(const NativeOrnamentInputs &in,std::shared_ptr<const bozo::render::Texture> picture){
    namespace F=bozo::ps2f;uint32_t flags=0;
    const auto A=[&](uint32_t a,uint32_t b){return F::add(a,b,flags);};
    const auto S=[&](uint32_t a,uint32_t b){return F::sub(a,b,flags);};
    const auto M=[&](uint32_t a,uint32_t b){return F::mul(a,b,flags);};
    const auto D=[&](uint32_t a,uint32_t b){return F::div(a,b,flags);};
    const auto I=[](uint32_t x){return int32_t(F::toInt(x));};
    const auto W=[](float v){return std::bit_cast<uint32_t>(v);};
    const auto U=[](uint32_t v){return F::fromInt(v);};
    const uint32_t k16=W(16.f),k6=W(6.f),kHalf=W(0.5f);
    const auto &centre=in.rects[0],&wing=in.rects[1];
    const auto su=D(F::fromInt(1u<<in.log2Width),W(256.f)),sv=D(F::fromInt(1u<<in.log2Height),W(256.f));
    const auto uv=[&](const std::array<uint16_t,6> &r){return std::array<int32_t,4>{I(M(su,U(r[0]))),I(M(sv,U(r[1]))),I(M(su,U(r[2]))),I(M(sv,U(r[3])))};};
    const float texW=float(1u<<in.log2Width)*16,texH=float(1u<<in.log2Height)*16;
    const auto y0f=M(k16,S(in.top,k6));const auto y0=I(y0f),y1=I(A(y0f,M(k16,U(uint32_t(centre[5])+5))));
    const auto xL=I(M(k16,in.left)),xR=I(M(k16,in.right)),xM=I(M(k16,M(kHalf,A(in.left,in.right))));
    const auto xC=I(M(k16,S(in.left,U(wing[4])))),xD=I(M(k16,A(in.right,U(wing[4]))));
    NativeUiText out;
    const auto sprite=[&](int32_t x0,int32_t x1,const std::array<uint16_t,6> &r,bool mirrored){
        const auto t=uv(r);NativeUiSprite s;s.colour=in.colour;s.depth=in.depth;s.picture=picture;
        s.fixedCorners={std::array<int32_t,2>{x0,y0},{x1,y0},{x0,y1},{x1,y1}};
        s.uv={float(mirrored?t[2]:t[0])/texW,float(t[1])/texH,float(mirrored?t[0]:t[2])/texW,float(t[3])/texH};out.icons.push_back(std::move(s));};
    sprite(xL,xM,centre,false);sprite(xM,xR,centre,true);sprite(xC,xL,wing,false);sprite(xR,xD,wing,true);
    return out;
}
// Builder 4489b0 after 448d70: the cap (rect +160) over [x1-cap, x1] and the
// body (rect +170) over [x1, right], both over [y0, y1], on PS2 float words.
NativeUiText nativeBar(const NativeBarInputs &in,std::shared_ptr<const bozo::render::Texture> picture){
    namespace F=bozo::ps2f;uint32_t flags=0;
    const auto S=[&](uint32_t a,uint32_t b){return F::sub(a,b,flags);};
    const auto M=[&](uint32_t a,uint32_t b){return F::mul(a,b,flags);};
    const auto D=[&](uint32_t a,uint32_t b){return F::div(a,b,flags);};
    const auto I=[](uint32_t x){return int32_t(F::toInt(x));};
    const auto W=[](float v){return std::bit_cast<uint32_t>(v);};
    const auto U=[](uint32_t v){return F::fromInt(v);};
    const uint32_t k16=W(16.f);
    const auto su=D(F::fromInt(1u<<in.log2Width),W(256.f)),sv=D(F::fromInt(1u<<in.log2Height),W(256.f));
    const auto uv=[&](const std::array<uint16_t,6> &r){return std::array<int32_t,4>{I(M(su,U(r[0]))),I(M(sv,U(r[1]))),I(M(su,U(r[2]))),I(M(sv,U(r[3])))};};
    const float texW=float(1u<<in.log2Width)*16,texH=float(1u<<in.log2Height)*16;
    const auto y0=I(M(k16,in.y0)),y1=I(M(k16,in.y1));
    const auto xa=I(M(k16,S(in.x1,in.cap))),xb=I(M(k16,in.x1)),xc=I(M(k16,in.right));
    NativeUiText out;
    const auto sprite=[&](int32_t x0,int32_t x1,const std::array<uint16_t,6> &r){
        const auto t=uv(r);NativeUiSprite s;s.colour=in.colour;s.depth=in.depth;s.picture=picture;
        s.fixedCorners={std::array<int32_t,2>{x0,y0},{x1,y0},{x0,y1},{x1,y1}};
        s.uv={float(t[0])/texW,float(t[1])/texH,float(t[2])/texW,float(t[3])/texH};out.icons.push_back(std::move(s));};
    sprite(xa,xb,in.rects[0]);sprite(xb,xc,in.rects[1]);
    return out;
}
std::optional<bool> nativeNamePlateInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc==0x448d70){
        auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
        auto *owner=d?dynamic_cast<UiTextOwner *>(&d->host()):nullptr;auto *textures=d?dynamic_cast<TextureOwner *>(&d->host()):nullptr;
        if(!c || !owner || !textures || c->r[21].uw[0]<0x100000)throw std::runtime_error("native prompt bar requires its builder");
        const auto element=c->r[21].uw[0],resource=c->r[17].uw[0];record(rt,element,0x60);
        const auto layer=value<uint32_t>(rt,element+0x54);record(rt,layer,0xe0);record(rt,resource,0x240);
        if(value<uint32_t>(rt,element)!=0x165c30 || c->r[20].uw[0]!=element+0x40 || value<uint32_t>(rt,resource)!=0x165b40 ||
           value<uint32_t>(rt,value<uint32_t>(rt,0x48e048)+0x5c)!=resource)
            throw std::runtime_error("native prompt bar escaped its element/resource");
        const auto image=textures->nativeTextures().imageAsset(resource);
        NativeBarInputs in;in.x1=c->fu[23];in.y0=c->fu[21];in.y1=c->fu[20];in.cap=c->fu[24];in.right=c->fu[22];
        in.colour=c->r[18].uw[0];in.depth=value<uint32_t>(rt,layer+0xc4);
        in.log2Width=value<uint8_t>(rt,resource+0x238);in.log2Height=value<uint8_t>(rt,resource+0x239);
        if(!image || in.log2Width>10 || in.log2Height>10 || image->width!=(1u<<in.log2Width) || image->height!=(1u<<in.log2Height))
            throw std::runtime_error("native prompt bar has no published resource image");
        if((in.colour&0x00ffffffu)!=0x808080u)throw std::runtime_error("unqualified native prompt bar colour");
        const auto table=value<uint32_t>(rt,resource+0x224);record(rt,table+0x160,0x20);
        for(unsigned i=0;i<6;++i){in.rects[0][i]=value<uint16_t>(rt,table+0x160+2*i);in.rects[1][i]=value<uint16_t>(rt,table+0x170+2*i);}
        auto bar=std::make_shared<NativeUiText>(nativeBar(in,image->picture(0)));bar->element=element;bar->iconsCaptured=true;
        owner->nativeUiTexts().plates[element]=std::move(bar);
        return false; // The original builder continues; its packet is never sent.
    }
    if(pc==0x449790){
        auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
        auto *owner=d?dynamic_cast<UiTextOwner *>(&d->host()):nullptr;auto *textures=d?dynamic_cast<TextureOwner *>(&d->host()):nullptr;
        if(!c || !owner || !textures || c->r[20].uw[0]<0x100040)throw std::runtime_error("native ornament requires its builder");
        const auto layer=c->r[21].uw[0],element=c->r[20].uw[0]-0x40,resource=c->r[17].uw[0];
        record(rt,element,0x60);record(rt,layer,0x290);record(rt,resource,0x240);
        if(value<uint32_t>(rt,element)!=0x165e00 || value<uint32_t>(rt,element+0x54)!=layer || value<uint32_t>(rt,resource)!=0x165b40 ||
           value<uint32_t>(rt,value<uint32_t>(rt,0x48e048)+0x5c)!=resource)
            throw std::runtime_error("native ornament escaped its element/resource");
        const auto image=textures->nativeTextures().imageAsset(resource);
        NativeOrnamentInputs in;in.left=c->fu[21];in.right=c->fu[20];in.top=c->fu[22];in.colour=c->r[19].uw[0];in.depth=value<uint32_t>(rt,layer+0xc4);
        in.log2Width=value<uint8_t>(rt,resource+0x238);in.log2Height=value<uint8_t>(rt,resource+0x239);
        if(!image || in.log2Width>10 || in.log2Height>10 || image->width!=(1u<<in.log2Width) || image->height!=(1u<<in.log2Height))
            throw std::runtime_error("native ornament has no published resource image");
        if((in.colour&0x00ffffffu)!=0x808080u)throw std::runtime_error("unqualified native ornament colour");
        const auto table=value<uint32_t>(rt,resource+0x224);record(rt,table+0x90,0x20);
        for(unsigned i=0;i<6;++i){in.rects[0][i]=value<uint16_t>(rt,table+0xa0+2*i);in.rects[1][i]=value<uint16_t>(rt,table+0x90+2*i);}
        auto ornament=std::make_shared<NativeUiText>(nativeOrnament(in,image->picture(0)));ornament->element=element;ornament->iconsCaptured=true;
        owner->nativeUiTexts().plates[element]=std::move(ornament);
        return false; // The original builder continues; its packet is never sent.
    }
    if(pc!=0x4653e8 && pc!=0x465b38 && pc!=0x465f08)return std::nullopt;
    if(pc==0x465b38)throw std::runtime_error("unported native name plate: narrow path 465b38");
    if(pc==0x465f08){
        // The tail is reached exactly when the capture at 4653e8 decided it is.
        auto *dd=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;auto *o=dd?dynamic_cast<UiTextOwner *>(&dd->host()):nullptr;
        if(!c || !o || c->r[17].uw[0]<0x100040)throw std::runtime_error("native name plate tail requires its builder");
        const auto captured=o->nativeUiTexts().plate(c->r[17].uw[0]-0x40);
        if(!captured || captured->lateIcons.size()!=1){
            const auto layer=c->r[18].uw[0],uploader=layer>=0x100000?value<uint32_t>(rt,layer+0x26c):0u;
            throw std::runtime_error(std::format("native name plate tail was not captured: plate={} icons={} layer={:x} +270={:x} uploader={:x} done={}",
                bool(captured),captured?captured->lateIcons.size():0,layer,layer>=0x100000?value<uint32_t>(rt,layer+0x270):0u,uploader,uploader?value<uint8_t>(rt,uploader+0x12ac)&1:0));}
        return false;
    }
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<UiTextOwner *>(&d->host()):nullptr;auto *textures=d?dynamic_cast<TextureOwner *>(&d->host()):nullptr;
    if(!c || !owner || !textures || c->r[17].uw[0]<0x100040)throw std::runtime_error("native name plate requires its builder");
    const auto layer=c->r[18].uw[0],element=c->r[17].uw[0]-0x40,resource=c->r[21].uw[0];
    record(rt,element,0x60);record(rt,layer,0x2b0);record(rt,resource,0x240);
    if(value<uint32_t>(rt,element)!=0x167820 || value<uint32_t>(rt,element+0x54)!=layer || value<uint32_t>(rt,resource)!=0x165b40 ||
       value<uint32_t>(rt,value<uint32_t>(rt,0x48e048)+0x5c)!=resource)
        throw std::runtime_error("native name plate escaped its element/resource");
    const auto image=textures->nativeTextures().imageAsset(resource);
    NativeNamePlateInputs in;in.x=c->fu[24];in.y=c->fu[23];in.length=c->fu[28];in.colour=c->r[22].uw[0];in.depth=c->r[30].uw[0];
    in.tailOffset=value<uint32_t>(rt,layer+0x2a4);
    in.log2Width=value<uint8_t>(rt,resource+0x238);in.log2Height=value<uint8_t>(rt,resource+0x239);
    if(!image || in.log2Width>10 || in.log2Height>10 || image->width!=(1u<<in.log2Width) || image->height!=(1u<<in.log2Height))
        throw std::runtime_error("native name plate has no published resource image");
    const auto table=value<uint32_t>(rt,resource+0x224);record(rt,table+0xb0,0xb0);
    const uint32_t offsets[3]={0xb0,0x140,0x150};
    for(unsigned k=0;k<3;++k)for(unsigned i=0;i<6;++i)in.rects[k][i]=value<uint16_t>(rt,table+offsets[k]+2*i);
    in.narrowCap=(value<uint8_t>(rt,value<uint32_t>(rt,0x48e04c)+3)>>4)&1;
    const auto context=value<uint32_t>(rt,0x35de60);in.frameWidth=value<int16_t>(rt,context+0x3c6c);in.frameHeight=value<int16_t>(rt,context+0x3c6e);
    if((in.colour&0x00ffffffu)!=0x808080u)throw std::runtime_error("unqualified native name plate colour");
    // The tail's conditions, evaluated as 465ee0 does (nothing changes them in between).
    std::shared_ptr<const bozo::render::Texture> iconPicture;
    const auto uploader=value<uint32_t>(rt,layer+0x26c);
    if(value<uint32_t>(rt,layer+0x270) && uploader){record(rt,uploader,0x12b0);
        if(value<uint32_t>(rt,uploader)!=0x165630)throw std::runtime_error("native name plate tail has no texture uploader");
        if(value<uint8_t>(rt,uploader+0x12ac)&1){
            const auto index=value<uint32_t>(rt,layer+0x284);if(index>=16)throw std::runtime_error("native name plate image entry outside the uploader");
            const auto entry=uploader+0x10a8+0x18*index;const auto tex0=value<uint64_t>(rt,entry);
            const auto icon=textures->nativeTextures().imageAsset(entry);
            in.icon=true;in.textSize=c->r[16].sw[0];in.alpha=value<uint32_t>(rt,layer+0xe0);in.alphaScale=value<uint32_t>(rt,layer+0x298);
            in.iconLog2Width=unsigned(tex0>>26)&15;in.iconLog2Height=unsigned(tex0>>30)&15;
            const auto rect=value<uint32_t>(rt,layer+0x268);record(rt,rect,12);for(unsigned i=0;i<6;++i)in.iconRect[i]=value<uint16_t>(rt,rect+2*i);
            if(!icon || icon->width!=(1u<<in.iconLog2Width) || icon->height!=(1u<<in.iconLog2Height) || ((tex0>>56)&0x1f))
                throw std::runtime_error("native name plate image has no published uploader entry");
            if(value<uint8_t>(rt,layer+0x2ac)&1)throw std::runtime_error("unported native name plate: narrow image tail");
            iconPicture=icon->picture(0);
        }
    }
    auto plate=std::make_shared<NativeUiText>(nativeNamePlate(in,image->picture(0),std::move(iconPicture)));plate->element=element;plate->iconsCaptured=true;
    owner->nativeUiTexts().plates[element]=std::move(plate);
    return false; // The original scalar builder continues; its packet is never sent.
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

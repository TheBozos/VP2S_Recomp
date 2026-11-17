#include "native_ui_windows.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <mutex>
#include <map>
#include "native_textures.h"
namespace vp2s {
namespace {
template<class T>T value(Runtime *rt,uint32_t at){
    if(!rt || at<0x100000 || uint64_t(at)+sizeof(T)>0x2000000)throw std::runtime_error("native UI window metadata outside EE RAM");
    T v;std::memcpy(&v,rt->memory.hostPointer(at,sizeof(v)),sizeof(v));return v;
}
void windowType(Runtime *rt,uint32_t element){if(value<uint32_t>(rt,element)!=0x15a1b0)throw std::runtime_error("unported native window class");}
}
void NativeUiWindows::begin(Runtime *rt,uint32_t stack,uint32_t element){
    windowType(rt,element);
    if(builds.contains(stack))throw std::runtime_error("native window capture stack reused");
    if(!value<uint8_t>(rt,element+0xf6) || value<uint8_t>(rt,element+0x3b))return;
    if(value<uint8_t>(rt,element+0xf3) || value<uint8_t>(rt,element+0xf4))throw std::runtime_error("native window scroll controls require capture");
    ready.erase(element);builds.emplace(stack,Build{NativeUiWindow{element}});
}
void NativeUiWindows::beginBackground(uint32_t stack,uint32_t element){
    const auto parent=stack+0x40;auto it=builds.find(parent);
    if(it==builds.end() || it->second.window.element!=element || it->second.background || backgrounds.contains(stack))
        throw std::runtime_error("native window background escaped original builder");
    backgrounds.emplace(stack,Background{parent,element});
}
void NativeUiWindows::background(Runtime *rt,uint32_t stack,const std::array<uint32_t,3> &colours,uint32_t edge,uint32_t depth){
    auto &call=backgrounds.at(stack);auto &build=builds.at(call.parentStack);
    if(call.captured || depth>0xffffff)throw std::runtime_error("native window repeated/invalid background");
    std::array<int32_t,4> xs{},ys{};
    for(unsigned i=0;i<4;++i){xs[i]=value<int32_t>(rt,stack+0x10+8*i);ys[i]=value<int32_t>(rt,stack+0x14+8*i);
        if(xs[i]<0 || xs[i]>65535 || ys[i]<0 || ys[i]>65535)throw std::runtime_error("native window background outside fixed-point range");}
    build.window.background.push_back({{{{xs[3],ys[0]},{xs[3],ys[3]},{xs[0],ys[0]},{xs[0],ys[3]}}},
                                       {colours[0],colours[1],colours[2],colours[0]},depth});
    for(unsigned s=0;s<8;++s){NativeUiStrip strip;strip.depth=depth;
        for(unsigned v=0;v<4;++v){const auto at=0x357900+12*s+3*v;const auto x=value<uint8_t>(rt,at),y=value<uint8_t>(rt,at+1),colour=value<uint8_t>(rt,at+2);
            if(x>3 || y>3 || colour>1)throw std::runtime_error("native window edge topology invalid");
            strip.corners[v]={xs[x],ys[y]};strip.colours[v]=colour?0:edge;}
        build.window.background.push_back(strip);
    }
    call.captured=true;
}
void NativeUiWindows::endBackground(uint32_t stack){
    const auto call=backgrounds.at(stack);if(!call.captured)throw std::runtime_error("native window background has no scalar snapshot");
    builds.at(call.parentStack).background=true;backgrounds.erase(stack);
}
void NativeUiWindows::beginQuads(Runtime *rt,NativeUiSprites &sprites,uint32_t stack,uint32_t sprite,uint32_t caller){
    // Window border's original helper has a 0xf0-byte frame, nested under
    // the window's 0x30-byte frame. The list itself is ordinary CPU records.
    const auto parent=stack+0x60+0xf0;auto it=builds.find(parent);
    if(caller!=0x2e9e80 || it==builds.end() || it->second.window.element+0x50!=sprite || it->second.border || quads.contains(stack))
        throw std::runtime_error("unported native quad-list caller");
    const auto sheet=sprites.button(sprite);const auto count=value<int16_t>(rt,sprite+0x58);const auto list=value<uint32_t>(rt,sprite+12);
    if(!sheet || value<uint32_t>(rt,sprite+4)!=sheet->dtt || value<uint32_t>(rt,sprite+8)!=sheet->parts ||
       value<uint16_t>(rt,sprite+0x56)!=sheet->count || count<1 || count>256 || uint64_t(list)+24u*count>0x2000000)
        throw std::runtime_error("native window border lost its constructor atlas/list lease");
    value<uint32_t>(rt,list);quads.emplace(stack,Quads{parent,sprite,uint32_t(count),list,0,sheet});
}
// Camp menu image list, class 15aa20 (mod001 build 30c5a0, which calls the
// quad-list writer 2ead10 with its sprite +50 and returns to 30c628). Its
// sprite was bound by 2ec200 to a DTT of the menu's own file and to the
// layer's texture record (+5a page, +5c palette page, +5f format, +60/+61
// log2 size) written by the menu constructor 464240. The picture is the camp
// menu image the loader published under that same handle; no other texture
// state is read. The quads land in this element's own snapshot.
void NativeUiWindows::beginImageQuads(Runtime *rt,const NativeTextures &textures,uint32_t stack,uint32_t sprite,uint32_t element){
    if(value<uint32_t>(rt,element)!=0x15aa20 || sprite!=element+0x50 || quads.contains(stack) || imageBuilds.contains(stack))
        throw std::runtime_error("native camp menu image list escaped its builder");
    const auto layer=value<uint32_t>(rt,element+0x14);
    if(value<uint32_t>(rt,layer)!=0x152060)
        throw std::runtime_error("unported native quad-list caller: image list outside a UI layer");
    const auto dtt=value<uint32_t>(rt,sprite+4),parts=value<uint32_t>(rt,sprite+8),list=value<uint32_t>(rt,sprite+12);
    const auto entries=value<uint16_t>(rt,sprite+0x56);const auto count=value<int16_t>(rt,sprite+0x58);
    if(value<uint32_t>(rt,dtt)!=0x545444 || parts!=dtt+0x14 || entries!=value<uint16_t>(rt,dtt+0x10) || !entries ||
       count<1 || count>256 || uint64_t(list)+24u*count>0x2000000)
        throw std::runtime_error("native camp menu image list lost its DTT/list");
    const auto page=value<uint16_t>(rt,sprite+0x5a),palette=value<uint16_t>(rt,sprite+0x5c);
    const auto format=value<uint8_t>(rt,sprite+0x5f),tw=value<uint8_t>(rt,sprite+0x60),th=value<uint8_t>(rt,sprite+0x61);
    auto asset=textures.menuPage(page,format,palette);
    if(!asset)throw std::runtime_error(std::format("native camp menu image list names no published menu image (page {:x} format {:x} palette {:x})",page,format,palette));
    if(tw>10 || th>10 || asset->width!=1u<<tw || asset->height!=1u<<th)
        throw std::runtime_error(std::format("native camp menu image {}x{} differs from its record ({}, {})",asset->width,asset->height,tw,th));
    value<uint32_t>(rt,list);
    auto sheet=std::make_shared<NativeButtonSheet>(NativeButtonSheet{sprite,dtt,parts,entries,0,std::move(asset)});
    Quads q{0,sprite,uint32_t(count),list,0,std::move(sheet)};q.element=element;q.image=true;
    quads.emplace(stack,std::move(q));imageBuilds.emplace(stack,NativeUiWindow{element});
}
std::vector<NativeUiSprite> &NativeUiWindows::quadTarget(const Quads &q){
    if(q.image){for(auto &[stack,b]:imageBuilds)if(b.element==q.element)return b.border;throw std::runtime_error("native image list lost its capture");}
    return builds.at(q.parentStack).window.border;
}
void NativeUiWindows::colour(uint32_t stack,uint32_t colour){
    auto &q=quads.at(stack);if(q.coloured)throw std::runtime_error("native window border colour repeats");q.colour=colour;q.coloured=true;
}
void NativeUiWindows::quad(Runtime *rt,uint32_t stack,uint32_t part,uint32_t csa,uint32_t list){
    auto &q=quads.at(stack);const auto index=quadTarget(q).size();
    if(q.quad || !q.coloured || index>=q.count || list!=q.list+24*index)throw std::runtime_error("native window border quad is out of order");
    const auto selected=value<uint32_t>(rt,list);
    if(selected>=q.sheet->count || part!=q.sheet->parts+16*selected)throw std::runtime_error("native window border part escaped its atlas");
    q.quad=NativeUiSprite{.sprite=q.sprite,.colour=q.colour,.picture=q.sheet->asset->picture(csa)};q.uvMask=0;
}
void NativeUiWindows::corners(uint32_t stack,int32_t x0,int32_t y0,int32_t x1,int32_t y1,uint32_t depth){
    auto &q=quads.at(stack);if(!q.quad || depth>0xffffff || x0<0 || x0>65535 || x1<0 || x1>65535 || y0<0 || y0>65535 || y1<0 || y1>65535)
        throw std::runtime_error("native window border corners invalid");
    q.quad->fixedCorners={{{x0,y0},{x1,y0},{x0,y1},{x1,y1}}};q.quad->depth=depth;
}
void NativeUiWindows::uv(uint32_t stack,unsigned index,int32_t coordinate){
    auto &q=quads.at(stack);if(!q.quad || index>3 || (q.uvMask&(1u<<index)) || coordinate<0)throw std::runtime_error("native window border UV missing/repeated");
    q.quad->uv[index]=float(coordinate)/float(index&1?q.sheet->asset->height:q.sheet->asset->width);q.uvMask|=1u<<index;
    if(q.uvMask==15){quadTarget(q).push_back(std::move(*q.quad));q.quad.reset();}
}
void NativeUiWindows::finishQuads(uint32_t stack){
    const auto &q=quads.at(stack);
    if(q.quad || quadTarget(q).size()!=q.count)throw std::runtime_error("native window border returned before all quads");
    if(q.image){auto it=imageBuilds.find(stack);ready[q.element]=std::make_shared<NativeUiWindow>(std::move(it->second));imageBuilds.erase(it);}
    else builds.at(q.parentStack).border=true;
    quads.erase(stack);
}
void NativeUiWindows::finish(Runtime *rt,uint32_t stack,uint32_t element){
    windowType(rt,element);auto it=builds.find(stack);
    if(it==builds.end() || it->second.window.element!=element || bool(value<uint8_t>(rt,element+0xf1))!=it->second.background ||
       bool(value<uint8_t>(rt,element+0xf2))!=it->second.border)throw std::runtime_error("native window builder has unfinished components");
    ready[element]=std::make_shared<NativeUiWindow>(std::move(it->second.window));builds.erase(it);
    if(++captures<=4 && std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:UI] native window=%x background=%zu border=%zu\n",element,ready[element]->background.size(),ready[element]->border.size());
}
std::shared_ptr<const NativeUiWindow> NativeUiWindows::window(uint32_t element)const{
    auto it=ready.find(element);return it==ready.end()?nullptr:it->second;
}
void NativeUiWindows::release(uint32_t element){
    for(const auto &[stack,b]:builds)if(b.window.element==element)throw std::runtime_error("native window destroyed during capture");
    for(const auto &[stack,b]:imageBuilds)if(b.element==element)throw std::runtime_error("native image list destroyed during capture");
    for(const auto &[stack,b]:selections)if(b.element==element)throw std::runtime_error("native selection destroyed during capture");ready.erase(element);
}
void NativeUiWindows::beginSelection(Runtime *rt,uint32_t stack,uint32_t element){
    if(value<uint32_t>(rt,element)!=0x15ac30 || selections.contains(stack))throw std::runtime_error("native selection has invalid class/stack lifetime");
    if(value<uint8_t>(rt,element+0x3b))return;
    ready.erase(element);selections.emplace(stack,Selection{element});
}
void NativeUiWindows::selectionColour(uint32_t stack,uint32_t alpha,uint32_t depth){
    auto &s=selections.at(stack);if(s.coloured || alpha>255 || depth>0xffffff)throw std::runtime_error("native selection colour/depth invalid");
    s.alpha=alpha;s.depth=depth;s.coloured=true;
}
void NativeUiWindows::selectionCoordinate(uint32_t stack,unsigned index,int32_t coordinate){
    auto &s=selections.at(stack);if(index>5 || (s.mask&(1u<<index)) || coordinate<0 || coordinate>65535)throw std::runtime_error("native selection coordinate invalid/repeated");
    s.coordinates[index]=coordinate;s.mask|=1u<<index;
}
void NativeUiWindows::finishSelection(uint32_t stack,uint32_t element){
    const auto &s=selections.at(stack);if(s.element!=element || s.mask!=63 || !s.coloured)throw std::runtime_error("native selection returned before scalar capture");
    auto result=std::make_shared<NativeUiWindow>();result->element=element;
    const auto &c=s.coordinates;const uint32_t colour=s.alpha<<24,ends[4]={0,colour,colour,0};
    for(unsigned i=0;i<3;++i)result->background.push_back({{{{c[i],c[4]},{c[i+1],c[4]},{c[i],c[5]},{c[i+1],c[5]}}},{ends[i],ends[i+1],ends[i],ends[i+1]},s.depth});
    ready[element]=std::move(result);selections.erase(stack);
}
// mod003 class 167560 (camp menu panel), build 460d20: one gouraud,
// alpha-blended four-vertex strip. At 460f24 (its "built" flag store) the
// compiled scalars are still in registers: t1 = depth<<32 | yb<<16 | xb,
// t4 = depth<<32 | ya<<16 | xa, a2 = alpha<<24 (layer +e0 times element +28 over 128).
// Colours are the element's +50..+5c, each or-ed with that alpha, in the
// vertex order (xa,ya) (xb,ya) (xa,yb) (xb,yb).
void NativeUiWindows::panel(Runtime *rt,uint32_t element,uint64_t lowerRight,uint64_t upperLeft,uint32_t alpha){
    if(value<uint32_t>(rt,element)!=0x167560)throw std::runtime_error("native panel has another class");
    const auto depth=uint32_t(lowerRight>>32);
    if(depth>0xffffff || (alpha&0xffffff) || (upperLeft>>32)!=depth)throw std::runtime_error("native panel scalars outside the build's ranges");
    const int32_t xa=int32_t(upperLeft&0xffff),ya=int32_t((upperLeft>>16)&0xffff),xb=int32_t(lowerRight&0xffff),yb=int32_t((lowerRight>>16)&0xffff);
    NativeUiStrip strip;strip.depth=depth;strip.corners={{{xa,ya},{xb,ya},{xa,yb},{xb,yb}}};
    for(unsigned i=0;i<4;++i)strip.colours[i]=value<uint32_t>(rt,element+0x50+4*i)|alpha;
    auto result=std::make_shared<NativeUiWindow>();result->element=element;result->background.push_back(strip);
    ready[element]=std::move(result);
}
// Scissor element 15a420 (mod001 build 2ef5b0): one SCISSOR register write,
// the element's rectangle scaled to frame pixels and clamped (or the whole
// frame when +50 is set). At 2ef780 (its "built" flag store) the compiled
// register value is still in t2: x0 | x1<<16 | y0<<32 | y1<<48.
void NativeUiWindows::scissor(Runtime *rt,uint32_t element,uint64_t registerValue){
    if(value<uint32_t>(rt,element)!=0x15a420)throw std::runtime_error("native scissor has another class");
    const auto field=[&](unsigned shift){return int32_t((registerValue>>shift)&0xffff);};
    const std::array<int32_t,4> rect{field(0),field(32),field(16),field(48)};
    if(registerValue&0xf800f800f800f800ull)throw std::runtime_error("native scissor outside the register's 11-bit fields");
    auto result=std::make_shared<NativeUiWindow>();result->element=element;result->scissor=rect;ready[element]=std::move(result);
}
void NativeUiWindows::spriteElement(uint32_t element,std::vector<NativeUiSprite> sprites){
    if(sprites.size()!=1)throw std::runtime_error("native sprite element build drew another sprite count");
    auto result=std::make_shared<NativeUiWindow>();result->element=element;result->border=std::move(sprites);ready[element]=std::move(result);
}
std::shared_ptr<const bozo::render::Texture> nativeMenuBackdropPicture(const NativeTextureAsset &top,const NativeTextureAsset &bottom){
    static std::mutex lock;static std::map<std::pair<uint64_t,uint64_t>,std::shared_ptr<const bozo::render::Texture>> cache;
    std::lock_guard guard(lock);const auto key=std::pair{top.identity,bottom.identity};
    if(auto it=cache.find(key);it!=cache.end())return it->second;
    const auto rows=256+bottom.height;
    auto result=std::make_shared<bozo::render::Texture>();result->width=512;result->height=rows;result->rgba.resize(size_t(512)*rows);
    result->key=((top.identity*0x9e3779b97f4a7c15ull)^bottom.identity)|1ull<<63;result->version=result->key;result->asset=true;
    for(size_t i=0;i<512*256;++i)result->rgba[i]=top.palette[top.texels.at(i)];
    for(size_t i=0;i<size_t(512)*bottom.height;++i)result->rgba[512*256+i]=top.palette[bottom.texels.at(i)];
    cache[key]=result;return result;
}
// mod003 class 1672d0, the camp menu backdrop (a scene layer object like the
// fixed image 15a6f0). Its build 45e5b0 writes, once (+c8), one SPRITE (PRIM
// 356): a CLUT load from palette page +c4, then TEX0 page 800, PSMT4HL, width
// 512 (CSA 0), colour 80808080, UV (8, 8)..(1ff8, 17f8) in 1/16 texels, XY
// from (+848, +84c) of the UI manager to that plus 16 times the frame size
// (*35de60 +3c6c/+3c6e), z 0. Its submit 45e570 queued that packet with the
// scene completion; natively the queue call (45e590) is patched to the
// completion 21c9a0 and its delay slot (45e594) draws the same sprite from the
// same scalars, the picture being the camp menu image published under that
// (page, format, palette page) handle.
NativeUiLayer nativeMenuBackdrop(Runtime *rt,const NativeTextures &textures,uint32_t object){
    if(value<uint32_t>(rt,object)!=0x1672d0 || value<uint32_t>(rt,object+0x90)!=0x16734c || value<uint8_t>(rt,object+0x64)!=1 || value<uint8_t>(rt,object+0xc8)!=1)
        throw std::runtime_error("native camp menu backdrop escaped its built submit");
    const auto current=value<uint32_t>(rt,0x35ddf8);const auto layer=value<uint8_t>(rt,object+0x65);
    if(layer>1 || value<uint32_t>(rt,current+0x40a8+4u*layer)!=object || value<uint8_t>(rt,current+0x40dc+layer)!=1)
        throw std::runtime_error("native camp menu backdrop escaped its scene lifetime");
    // The sprite samples rows 0..383 of one 512-wide four-bit surface: page
    // 800 holds rows 0..255 (the loader's first image) and page 1000, 256 rows
    // further, the next rows (its second image, 512x128, loaded without a
    // palette). Both are read through the CLUT loaded from +c4, the first
    // image's palette. Natively that is one 512x384 picture composed from the
    // two offline assets, built once per pair.
    const auto palette=uint16_t(value<uint32_t>(rt,object+0xc4));
    const auto top=textures.menuPage(0x800,0x24,palette),bottom=textures.menuPage(0x1000,0x24,0);
    if(!top || !bottom)throw std::runtime_error(std::format("native camp menu backdrop images are not published (palette {:x})",palette));
    if(top->width!=512 || top->height!=256 || bottom->width!=512 || bottom->height<128 || bottom->height>256 || top->format!=0x14 || bottom->format!=0x14)
        throw std::runtime_error(std::format("native camp menu backdrop images are {}x{}/{:x} and {}x{}/{:x}, not 512x256 over 512x128..256, four-bit",
            top->width,top->height,top->format,bottom->width,bottom->height,bottom->format));
    const auto picture=nativeMenuBackdropPicture(*top,*bottom);
    const auto context=value<uint32_t>(rt,0x35de60),manager=value<uint32_t>(rt,0x35e170);
    const auto width=value<int16_t>(rt,context+0x3c6c),height=value<int16_t>(rt,context+0x3c6e);
    const float x=value<float>(rt,manager+0x848),y=value<float>(rt,manager+0x84c);
    const auto x0=int32_t(x),y0=int32_t(y),x1=int32_t(16.f*float(width)+x),y1=int32_t(16.f*float(height)+y);
    if(width<=0 || height<=0 || x0<0 || y0<0 || x1>65535 || y1>65535)throw std::runtime_error("native camp menu backdrop outside the frame");
    NativeUiSprite sprite{.sprite=object,.colour=0x80808080,.depth=0,.fixedCorners={{{x0,y0},{x1,y0},{x0,y1},{x1,y1}}},
        .uv={8.f/16/512,8.f/16/float(picture->height),float(0x1ff8)/16/512,float(0x17f8)/16/float(picture->height)},.picture=picture};
    auto window=std::make_shared<NativeUiWindow>();window->element=object;window->border.push_back(std::move(sprite));
    auto text=std::make_shared<NativeUiText>();text->element=object;text->window=std::move(window);
    NativeUiLayer result{object,object};result.texts.push_back(std::move(text));return result;
}
std::array<bozo::render::Vertex,6> nativeUiStripVertices(const NativeUiStrip &strip){
    std::array<bozo::render::Vertex,6> result;const unsigned order[6]={0,1,2,1,3,2};
    for(unsigned i=0;i<6;++i){const auto j=order[i];result[i].x=float(strip.corners[j][0])/16;result[i].y=float(strip.corners[j][1])/16;result[i].z=float(strip.depth);result[i].rgba=strip.colours[j];}
    return result;
}
bozo::render::Material nativeUiFlatMaterial(bozo::render::Material::Blend blend){
    bozo::render::Material material;material.textured=false;material.directColour=true;material.blend=blend;
    material.alphaTest=blend!=bozo::render::Material::Blend::Additive;material.alphaRef=0;
    return material;
}
std::optional<bool> nativeUiWindowInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    switch(pc){case 0x2e9640:case 0x2e970c:case 0x2e95d0:case 0x2e9e90:case 0x2ea154:case 0x2ea338:case 0x2ead10:
        case 0x2eae90:case 0x2eb07c:case 0x2eb284:case 0x2eb2ec:case 0x2eb300:case 0x2eb320:case 0x2eb340:case 0x2eb378:
        case 0x30ee00:case 0x30ef20:case 0x30ef2c:case 0x30ef44:case 0x30ef78:case 0x30efb8:case 0x30eff4:case 0x30f028:case 0x30f070:case 0x30ed90:case 0x30c530:case 0x460f24:case 0x460050:case 0x2ef780:case 0x2ef540:case 0x45e594:break;default:return std::nullopt;}
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;auto *o=d?dynamic_cast<UiWindowOwner *>(&d->host()):nullptr;
    auto *s=d?dynamic_cast<UiSpriteOwner *>(&d->host()):nullptr;if(!c || !o || !s)throw std::runtime_error("native window requires per-run owners");
    auto &w=o->nativeUiWindows();const auto sp=c->r[29].uw[0];
    if(pc==0x30ee00){if(c->pc!=pc)throw std::runtime_error("native selection requires complete original builder");w.beginSelection(rt,sp-0x90,c->r[4].uw[0]);}
    else if(pc==0x30ed90){if(c->r[4].uw[0])w.release(c->r[4].uw[0]);}
    else if(pc==0x30ef20)w.selectionColour(sp,c->r[2].uw[0],c->r[19].uw[0]);
    else if(pc==0x30ef2c)w.selectionCoordinate(sp,5,c->r[2].sw[0]);
    else if(pc==0x30ef44)w.selectionCoordinate(sp,0,c->r[2].sw[0]);
    else if(pc==0x30ef78)w.selectionCoordinate(sp,4,c->r[2].sw[0]);
    else if(pc==0x30efb8)w.selectionCoordinate(sp,1,c->r[2].sw[0]);
    else if(pc==0x30eff4)w.selectionCoordinate(sp,2,c->r[2].sw[0]);
    else if(pc==0x30f028)w.selectionCoordinate(sp,3,c->r[2].sw[0]);
    else if(pc==0x30f070)w.finishSelection(sp,c->r[16].uw[0]);
    else if(pc==0x2e9640){if(c->pc!=pc)throw std::runtime_error("native window requires complete builder");w.begin(rt,sp-0x30,c->r[4].uw[0]);}
    else if(pc==0x2e970c)w.finish(rt,sp,c->r[16].uw[0]);
    else if(pc==0x2e95d0){if(c->r[4].uw[0]){w.release(c->r[4].uw[0]);s->nativeUiSprites().reset(c->r[4].uw[0]+0x50);}}
    else if(pc==0x2e9e90)w.beginBackground(sp-0x40,c->r[4].uw[0]);
    else if(pc==0x2ea154)w.background(rt,sp,{c->r[10].uw[0],c->r[24].uw[0],c->r[15].uw[0]},value<uint32_t>(rt,sp+0x38),uint32_t(c->r[11].ud[0]>>32));
    else if(pc==0x2ea338)w.endBackground(sp);
    else if(pc==0x2ead10){
        if(c->r[31].uw[0]==0x30c628){
            auto *t=dynamic_cast<TextureOwner *>(&d->host());if(!t)throw std::runtime_error("native camp menu image list has no texture owner");
            w.beginImageQuads(rt,t->nativeTextures(),sp-0x60,c->r[4].uw[0],c->r[16].uw[0]);
        }else w.beginQuads(rt,s->nativeUiSprites(),sp-0x60,c->r[4].uw[0],c->r[31].uw[0]);
    }
    else if(pc==0x45e594){
        auto *textures=dynamic_cast<TextureOwner *>(&d->host());auto *texts=dynamic_cast<UiTextOwner *>(&d->host());
        if(!textures || !texts)throw std::runtime_error("native camp menu backdrop has no texture/draw owner");
        if(c->pc!=0x45e570)throw std::runtime_error("native camp menu backdrop requires its complete submit");
        texts->drawNativeUiLayer(nativeMenuBackdrop(rt,textures->nativeTextures(),c->r[4].uw[0]));
        return true; // a0 stays the object for the patched call to the scene completion 21c9a0.
    }
    else if(pc==0x2ef780){if(c->r[3].uw[0]!=1)throw std::runtime_error("native scissor site outside its build");w.scissor(rt,c->r[4].uw[0],c->r[10].ud[0]);}
    else if(pc==0x2ef540){if(c->r[4].uw[0]){if(value<uint32_t>(rt,c->r[4].uw[0])!=0x15a420)throw std::runtime_error("native scissor destructor class");w.release(c->r[4].uw[0]);}}
    else if(pc==0x460f24){if(c->r[7].uw[0]!=1)throw std::runtime_error("native panel site outside its build");w.panel(rt,c->r[4].uw[0],c->r[9].ud[0],c->r[12].ud[0],c->r[6].uw[0]);}
    else if(pc==0x460050){if(c->r[4].uw[0]){if(value<uint32_t>(rt,c->r[4].uw[0])!=0x167560)throw std::runtime_error("native panel destructor class");w.release(c->r[4].uw[0]);}}
    else if(pc==0x30c530){if(c->r[4].uw[0]){if(value<uint32_t>(rt,c->r[4].uw[0])!=0x15aa20)throw std::runtime_error("native image list destructor class");w.release(c->r[4].uw[0]);}}
    else if(pc==0x2eae90)w.colour(sp,c->r[8].uw[0]);
    else if(pc==0x2eb07c)w.quad(rt,sp,c->r[15].uw[0],c->r[9].uw[0],c->r[25].uw[0]);
    else if(pc==0x2eb284)w.corners(sp,c->r[14].sw[0],c->r[13].sw[0],c->r[12].sw[0],c->r[11].sw[0],c->r[10].uw[0]);
    else if(pc==0x2eb2ec)w.uv(sp,1,c->r[10].sw[0]);
    else if(pc==0x2eb300)w.uv(sp,0,c->r[10].sw[0]);
    else if(pc==0x2eb320)w.uv(sp,2,c->r[10].sw[0]);
    else if(pc==0x2eb340)w.uv(sp,3,c->r[10].sw[0]);
    else w.finishQuads(sp);
    return false;
}
}

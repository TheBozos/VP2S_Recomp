#include "native_ui_windows.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
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
void NativeUiWindows::colour(uint32_t stack,uint32_t colour){
    auto &q=quads.at(stack);if(q.coloured)throw std::runtime_error("native window border colour repeats");q.colour=colour;q.coloured=true;
}
void NativeUiWindows::quad(Runtime *rt,uint32_t stack,uint32_t part,uint32_t csa,uint32_t list){
    auto &q=quads.at(stack);const auto index=builds.at(q.parentStack).window.border.size();
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
    if(q.uvMask==15){builds.at(q.parentStack).window.border.push_back(std::move(*q.quad));q.quad.reset();}
}
void NativeUiWindows::finishQuads(uint32_t stack){
    const auto &q=quads.at(stack);auto &build=builds.at(q.parentStack);
    if(q.quad || build.window.border.size()!=q.count)throw std::runtime_error("native window border returned before all quads");
    build.border=true;quads.erase(stack);
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
std::array<bozo::render::Vertex,6> nativeUiStripVertices(const NativeUiStrip &strip){
    std::array<bozo::render::Vertex,6> result;const unsigned order[6]={0,1,2,1,3,2};
    for(unsigned i=0;i<6;++i){const auto j=order[i];result[i].x=float(strip.corners[j][0])/16;result[i].y=float(strip.corners[j][1])/16;result[i].z=float(strip.depth);result[i].rgba=strip.colours[j];}
    return result;
}
std::optional<bool> nativeUiWindowInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    switch(pc){case 0x2e9640:case 0x2e970c:case 0x2e95d0:case 0x2e9e90:case 0x2ea154:case 0x2ea338:case 0x2ead10:
        case 0x2eae90:case 0x2eb07c:case 0x2eb284:case 0x2eb2ec:case 0x2eb300:case 0x2eb320:case 0x2eb340:case 0x2eb378:
        case 0x30ee00:case 0x30ef20:case 0x30ef2c:case 0x30ef44:case 0x30ef78:case 0x30efb8:case 0x30eff4:case 0x30f028:case 0x30f070:case 0x30ed90:break;default:return std::nullopt;}
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
    else if(pc==0x2ead10)w.beginQuads(rt,s->nativeUiSprites(),sp-0x60,c->r[4].uw[0],c->r[31].uw[0]);
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

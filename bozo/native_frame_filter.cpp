#include "native_frame_filter.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <algorithm>
#include <format>
#include <stdexcept>
namespace vp2s {
namespace {
template<class T>T value(Runtime *rt,uint32_t at){
    if(!rt || at<0x100000 || uint64_t(at)+sizeof(T)>0x2000000)throw std::runtime_error("native frame filter outside EE RAM");
    const auto *p=rt->memory.hostPointer(at,sizeof(T));if(!p)throw std::runtime_error("unmapped frame filter");T v;std::memcpy(&v,p,sizeof(T));return v;
}
bool type(Runtime *rt,uint32_t object){
    if(value<uint32_t>(rt,object)!=0x155d50 || value<uint32_t>(rt,object+0x90)!=0x155dcc)
        throw std::runtime_error("unowned native frame filter type");
    // The original builder branches straight to its epilogue before reading
    // any mode/format/source data when disabled.
    if(!value<uint8_t>(rt,object+0xaf))return false;
    if(value<uint8_t>(rt,object+0xad) || value<uint8_t>(rt,object+0xae))
        throw std::runtime_error(std::format("unported native frame filter type/format/depth mode: object={:x} types={:x},{:x} selection={} format={} depth={} flag={}",object,value<uint32_t>(rt,object),value<uint32_t>(rt,object+0x90),value<uint8_t>(rt,object+0xac),value<uint8_t>(rt,object+0xad),value<uint8_t>(rt,object+0xae),value<uint8_t>(rt,object+0xaf)));
    const auto mode=value<uint8_t>(rt,object+0xac);if(mode<1 || mode>4)throw std::runtime_error("unported native frame filter level selection");
    return true;
}
}
void NativeFrameFilters::begin(Runtime *rt,uint32_t stack,uint32_t object){
    if(calls.contains(stack))throw std::runtime_error("native frame filter stack reused during capture");
    const bool enabled=type(rt,object);ready.erase(object);
    if(!enabled){NativeFrameFilter empty;empty.object=object;empty.enabled=false;empty.finished=true;ready[object]=std::move(empty);return;}
    calls.emplace(stack,NativeFrameFilter{object,value<uint32_t>(rt,object+0x94)});
}
void NativeFrameFilters::levels(uint32_t stack,uint32_t object,unsigned levels){
    auto it=calls.find(stack);if(it==calls.end() || it->second.object!=object || it->second.levels || levels<1 || levels>5)
        throw std::runtime_error("native frame filter level count escaped original builder");
    it->second.levels=levels;
}
void NativeFrameFilters::colour(Runtime *rt,uint32_t stack,uint32_t object,unsigned level,uint32_t colour){
    auto it=calls.find(stack);if(it==calls.end() || it->second.object!=object)throw std::runtime_error("native frame filter colour escaped builder");
    auto &call=it->second;
    if(!call.levels || level>=call.levels ||
       (!call.layers.empty() && call.layers.back().level>=level) || (colour&0xffffff)!=(value<uint32_t>(rt,object+0xa8)&0xffffff))
        throw std::runtime_error(std::format("native frame filter colour/order changed during builder: object={:x} level={} levels={} previous={} colour={:08x} source={:08x}",object,level,call.levels,call.layers.empty()?999:call.layers.back().level,colour,value<uint32_t>(rt,object+0xa8)));
    call.layers.push_back({level,colour});
}
void NativeFrameFilters::finish(uint32_t stack,uint32_t object){
    auto it=calls.find(stack);if(it==calls.end() || it->second.object!=object || !it->second.levels || it->second.layers.empty())
        throw std::runtime_error("native frame filter finished without scalar layer colours");
    it->second.finished=true;ready[object]=std::move(it->second);calls.erase(it);
}
NativeFrameFilter NativeFrameFilters::submit(Runtime *rt,uint32_t object)const{
    const bool enabled=type(rt,object);auto it=ready.find(object);if(it==ready.end() || !it->second.finished || it->second.enabled!=enabled || calls.end()!=std::find_if(calls.begin(),calls.end(),[&](const auto &p){return p.second.object==object;}))
        throw std::runtime_error("native frame filter submitted before builder completion");
    if(enabled && it->second.source!=value<uint32_t>(rt,object+0x94))throw std::runtime_error("native frame filter source changed after capture");
    const auto current=value<uint32_t>(rt,0x35ddf8);const auto layer=value<uint8_t>(rt,object+0x65);
    if(layer>1 || value<uint32_t>(rt,current+0x40a8+4*layer)!=object || value<uint8_t>(rt,current+0x40dc+layer)!=1)
        throw std::runtime_error("native frame filter missing pending scene ownership");
    return it->second;
}
std::optional<bool> nativeFrameFilterInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x25aa40 && pc!=0x25adbc && pc!=0x25bad0 && pc!=0x25bdcc && pc!=0x25aa30)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<FrameFilterOwner *>(&d->host()):nullptr;
    auto *scene=d?dynamic_cast<SceneOwner *>(&d->host()):nullptr;
    if(!owner || !scene || !c)throw std::runtime_error("native frame filter requires renderer/scene ownership");
    auto &filters=owner->nativeFrameFilters();const auto sp=c->r[29].uw[0];
    if(pc==0x25aa40){if(c->pc!=pc)throw std::runtime_error("native frame filter requires complete original builder");filters.begin(rt,sp-0x170,c->r[4].uw[0]);}
    else if(pc==0x25adbc)filters.levels(sp,c->r[21].uw[0],c->r[23].uw[0]);
    else if(pc==0x25bad0)filters.colour(rt,sp,c->r[21].uw[0],c->r[19].uw[0],c->r[3].uw[0]);
    else if(pc==0x25bdcc)filters.finish(sp,c->r[21].uw[0]);
    else{
        if(c->pc!=0x25aa10 || c->r[5].uw[0]<0x100090)throw std::runtime_error("native frame filter requires complete original producer");
        const auto submission=filters.submit(rt,c->r[5].uw[0]-0x90);
        const auto target=scene->nativeScene().frameTarget();
        if(submission.enabled){
            if(!target || target->object!=submission.source)throw std::runtime_error("native frame filter source is not active frame");
            owner->drawNativeFrameFilter(submission);
        }
        c->r[4].sd[0]=int32_t(submission.object);return true;
    }
    return false;
}
}

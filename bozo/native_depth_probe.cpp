#include "native_depth_probe.h"
#include "bozo/ee/dispatcher.h"
#include <algorithm>
#include <cstring>
#include <format>
#include <stdexcept>
namespace vp2s {
namespace {
template<class T>T value(Runtime *rt,uint32_t at){
    if(!rt || at<0x100000 || at>0x2000000-sizeof(T))throw std::runtime_error("native depth probe record outside EE RAM");
    T v;std::memcpy(&v,rt->memory.hostPointer(at,sizeof(T)),sizeof(T));return v;
}
template<class T>void store(Runtime *rt,uint32_t at,T v){
    if(!rt || at<0x100000 || at>0x2000000-sizeof(T))throw std::runtime_error("native depth probe record outside EE RAM");
    std::memcpy(rt->memory.hostPointer(at,sizeof(T)),&v,sizeof(T));
}
constexpr uint32_t kNodeClass=0x159010,kNodeJobClass=0x15908c,kEnqueue=0x29a4b0,kSite=0x29a4dc,kRequestReturn=0x29a584;
constexpr int32_t kPsmZ32=0x30,kPsmZ24=0x31;
int32_t half(const R5900Context *c,unsigned r){return int16_t(c->r[r].uw[0]&0xffff);}
}
std::vector<uint8_t> nativeDepthProbeBytes(const NativeDepthProbeRequest &q,const NativeDepthProbeFrame &frame,const NativeDepthSampler &sample){
    const unsigned bytes=q.psm==kPsmZ32?4:q.psm==kPsmZ24?3:0;
    if(!bytes)throw std::runtime_error(std::format("unported native readback format {:x} (only PSMZ32/PSMZ24 depth)",q.psm));
    if(q.width<1 || q.width>64 || q.height<1 || q.height>64)throw std::runtime_error(std::format("unported native depth probe size {}x{}",q.width,q.height));
    if(frame.width<1 || frame.height<1 || q.sbw*64!=frame.width)
        throw std::runtime_error(std::format("native depth probe buffer width {} does not match the frame {}x{}",q.sbw*64,frame.width,frame.height));
    const uint32_t nearest=bytes==4?0xffffffffu:0xffffffu;
    const float originX=2048.0f-float(frame.width)/2,originY=2048.0f-float(frame.height)/2;
    std::vector<uint8_t> out;out.reserve(size_t(q.width)*q.height*bytes);
    for(int32_t j=0;j<q.height;++j)for(int32_t i=0;i<q.width;++i){
        const int32_t px=q.x+i,py=q.y+j;uint32_t z=nearest;
        if(px>=0 && py>=0 && px<frame.width && py<frame.height){
            uint32_t sampled=0;
            if(sample && sample(originX+float(px)+0.5f,originY+float(py)+0.5f,sampled))z=std::min(sampled,nearest);
        }
        for(unsigned b=0;b<bytes;++b)out.push_back(uint8_t(z>>(8*b)));
    }
    return out;
}
void NativeDepthProbes::request(const R5900Context *c){
    if(c->r[31].uw[0]!=kRequestReturn)
        throw std::runtime_error(std::format("native StoreImage request outside the readback node (ra={:x})",c->r[31].uw[0]));
    NativeDepthProbeRequest q;q.packet=c->r[4].uw[0]&0x1ffffff;
    q.sbp=half(c,5);q.sbw=half(c,6);q.psm=half(c,7);q.x=half(c,8);q.y=half(c,9);q.width=half(c,10);q.height=half(c,11);
    requests[q.packet]=q;
}
uint32_t NativeDepthProbes::complete(Runtime *rt,const R5900Context *c,const NativeDepthProbeFrame &frame,const NativeDepthSampler &sample){
    if(c->pc!=kEnqueue || c->r[7].uw[0]<0x100090)throw std::runtime_error("native depth probe requires the original readback enqueue");
    const uint32_t object=c->r[7].uw[0]-0x90;
    if(value<uint32_t>(rt,object)!=kNodeClass || value<uint32_t>(rt,object+0x90)!=kNodeJobClass)
        throw std::runtime_error(std::format("unqualified native readback node {:x} class {:x}",object,value<uint32_t>(rt,object)));
    if(value<uint8_t>(rt,object+0x65))throw std::runtime_error("unported native readback node on the owner queue (+65)");
    const uint8_t slot=value<uint8_t>(rt,object+0x96);
    if(slot>1)throw std::runtime_error("native readback node slot outside its two packets");
    const uint32_t packet=object+0xe0+0x70*slot;
    const auto it=requests.find(packet);
    if(it==requests.end())throw std::runtime_error(std::format("native readback node {:x} slot {} has no recorded request",object,slot));
    const auto bytes=nativeDepthProbeBytes(it->second,frame,sample);
    const uint32_t destination=value<uint32_t>(rt,object+0x200+4*slot)&0x1ffffff;
    if(destination<0x100000 || destination+bytes.size()>0x2000000)throw std::runtime_error("native depth probe destination outside EE RAM");
    std::memcpy(rt->memory.hostPointer(destination,uint32_t(bytes.size())),bytes.data(),bytes.size());
    requests.erase(it);
    store<uint8_t>(rt,object+0x64,1);   // in flight, as 29a3a0 marks it; the real completion clears it
    return object;
}
std::optional<bool> nativeDepthProbeInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=kSite)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<DepthProbeOwner *>(&d->host()):nullptr;
    if(!c || !owner)throw std::runtime_error("native depth probe requires its renderer owner");
    const auto object=owner->nativeDepthProbes().complete(rt,c,owner->nativeDepthProbeFrame(),
        [owner](float x,float y,uint32_t &z){return owner->nativeDepthSample(x,y,z);});
    c->r[4].sd[0]=int32_t(object);
    return true; // Retire the job-pointer move; the patched call enters notify 299c00 with the node.
}
}

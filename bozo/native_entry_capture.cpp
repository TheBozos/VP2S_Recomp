#include "native_cast.h"
#include "native_entry_capture.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/ops_fpu.h"
#include "bozo/port.h"
#include "native_depth_probe.h"
#include <cstring>
#include <format>
#include <stdexcept>
namespace vp2s {
namespace {
template<class T>T value(Runtime *rt,uint32_t at){
    at&=0x1ffffffu;
    if(!rt || at<0x100000 || at>0x2000000-sizeof(T))throw std::runtime_error("native entry capture outside EE RAM");
    T v;std::memcpy(&v,rt->memory.hostPointer(at,sizeof(T)),sizeof(T));return v;
}
constexpr uint32_t kClass=0x15a850,kJobClass=0x15a8cc,kGroupClass=0x15a7a0,kSite=0x3096bc,kEnqueue=0x308c48,kGroupSubmit=0x307188;
constexpr uint32_t kTwo31=0x4f000000u;   // 2^31: the game's unsigned float -> int conversions
namespace F=bozo::ps2f;
// cvt.w.s, or (x - 2^31) | 0x80000000 at or above 2^31, as the listing converts.
uint32_t toUnsigned(uint32_t x){uint32_t f=0;return F::le(kTwo31,x)?(F::toInt(F::sub(x,kTwo31,f))|0x80000000u):F::toInt(x);}
EntryCaptureOwner &owner(Runtime *rt){
    auto *d=rt?vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services):nullptr;
    auto *o=d?vp2s::nativeCast<EntryCaptureOwner>(&d->host()):nullptr;
    if(!o)throw std::runtime_error("native entry capture requires its renderer owner");
    return *o;
}
}
NativeEntryCaptureDraw nativeEntryCaptureDraw(Runtime *rt,uint32_t object,int32_t snapshotWidth,int32_t snapshotHeight){
    if(snapshotWidth<1 || snapshotHeight<1)throw std::runtime_error("native entry capture snapshot has no size");
    if(value<uint32_t>(rt,object)!=kClass)throw std::runtime_error(std::format("native entry capture {:x} has class {:x}",object,value<uint32_t>(rt,object)));
    NativeEntryCaptureDraw draw;draw.object=object;
    const int32_t slices=value<int32_t>(rt,object+0x100);
    if(slices<1 || slices>16)throw std::runtime_error(std::format("unported native entry capture slice count {}",slices));
    const auto screen=value<uint32_t>(rt,0x35de60);
    draw.frameWidth=value<int16_t>(rt,screen+0x3c6c);draw.frameHeight=value<int16_t>(rt,screen+0x3c6e);
    if(draw.frameWidth<64 || draw.frameWidth>2048 || draw.frameHeight<64 || draw.frameHeight>2048)throw std::runtime_error("native entry capture frame size out of range");
    // Every slice header (3094d0): 64 x 416, the packet stride +fc.
    const uint32_t base=(value<uint32_t>(rt,object+0x9c)+0x7f)&~0x7fu,stride=value<uint32_t>(rt,object+0xfc)&~15u;
    for(int32_t k=0;k<slices;++k)
        if(value<uint16_t>(rt,base+k*stride)!=64 || value<uint16_t>(rt,base+k*stride+2)!=416)throw std::runtime_error("unported native entry capture slice size");
    uint32_t f=0;const auto word=[&](uint32_t at){return value<uint32_t>(rt,at);};
    // The grid pass (3078d0 after the strips): alpha cvt(160 * +c0), or 224 with +f9.
    const bool zoomed=value<uint8_t>(rt,object+0xf9)!=0;
    const uint32_t gridAlpha=toUnsigned(F::mul(zoomed?0x43600000u:0x43200000u,word(object+0xc0),f))&0xff;
    if(gridAlpha && zoomed)throw std::runtime_error(std::format("unported native entry capture zoomed grid (alpha {})",gridAlpha));
    if(gridAlpha){
        const uint32_t rgba=0x888888u|((0x60u*gridAlpha)>>7)<<24;
        const float x0=float(0x800-draw.frameWidth/2),y0=float(0x800-draw.frameHeight/2),x1=x0+float(draw.frameWidth),y1=y0+float(draw.frameHeight);
        const bool right[6]={false,true,false,true,true,false},bottom[6]={false,false,true,false,true,true};
        for(unsigned i=0;i<6;++i){bozo::render::Vertex v;v.x=right[i]?x1:x0;v.y=bottom[i]?y1:y0;v.z=0;v.u=right[i]?1.f:0.f;v.v=bottom[i]?1.f:0.f;v.rgba=rgba;draw.grid.push_back(v);}
    }
    // Colour (308c60): 0x7f7f7f, alpha 0x80 or 128 * (1 - min(+c8, +d0) / +d0).
    const uint32_t c8=word(object+0xc8),d0=word(object+0xd0);uint32_t alpha=0x80;
    if(!F::le(c8,0)){const auto m=F::le(c8,d0)?c8:d0;alpha=toUnsigned(F::mul(0x43000000u,F::sub(0x3f800000u,F::div(m,d0,f),f),f))&0xff;}
    draw.colour=0x007f7f7fu|alpha<<24;draw.blend=!F::eq(0,c8);
    const uint32_t c4=toUnsigned(word(object+0xc4))&0xffff;
    if(c4>208)throw std::runtime_error("unported native entry capture crop");
    const uint32_t z=word(0x35df00)-1-word(object+0xe0);
    const uint32_t points=word(object+0xa0);
    for(int32_t k=0;k<slices;++k)for(uint32_t h=0;h<2;++h){
        const uint32_t at=points+16*(2*k+h);uint32_t p[8];
        for(unsigned i=0;i<8;++i){p[i]=word(at+4*i);if(F::expOf(p[i])==0xff)throw std::runtime_error("native entry capture point is not finite");}
        // 3078d0's culling, in its order: x = p[0,2,4,6], y = p[1,3,5,7].
        const uint32_t maxX=F::max(F::max(F::max(p[0],p[2]),p[4]),p[6]),maxY=F::max(F::max(F::max(p[1],p[3]),p[5]),p[7]);
        constexpr uint32_t limit=0x47800000u;   // 65536
        if(!F::le(maxX,limit) || !F::le(maxY,limit))continue;
        if((F::le(p[0],0) && F::le(p[2],0)) || (F::le(p[4],0) && F::le(p[6],0)) || (F::le(p[1],0) && F::le(p[3],0)) || (F::le(p[5],0) && F::le(p[7],0)))continue;
        // UV (texels of the slice, 12.4 in the packet) to the frame image.
        const float u0=float(k*64)+float(h*32*16+12)/16,u1=float(k*64)+float((h+1)*32*16-12)/16,v0=float(c4),v1=float(416-c4);
        const auto vertex=[&](uint32_t x,uint32_t y,float u,float v){
            bozo::render::Vertex out;out.x=float(toUnsigned(x)&0xffff)/16;out.y=float(toUnsigned(y)&0xffff)/16;out.z=float(z);
            out.u=u/float(snapshotWidth);out.v=v/float(snapshotHeight);out.rgba=draw.colour;return out;};
        const bozo::render::Vertex strip[4]={vertex(p[0],p[1],u0,v0),vertex(p[2],p[3],u0,v1),vertex(p[4],p[5],u1,v0),vertex(p[6],p[7],u1,v1)};
        for(unsigned i:{0u,1u,2u,1u,3u,2u})draw.vertices.push_back(strip[i]);
    }
    return draw;
}
namespace {
void readbackEnqueue(uint8_t *ram,R5900Context *c,Runtime *rt){
    const auto node=c->r[4].uw[0];
    auto *d=rt?vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services):nullptr;
    auto *probes=d?vp2s::nativeCast<DepthProbeOwner>(&d->host()):nullptr;
    std::optional<NativeDepthProbeRequest> colour;
    if(probes && node && value<uint32_t>(rt,node)==0x159010 && value<uint32_t>(rt,node+0x90)==0x15908c && value<uint8_t>(rt,node+0x96)<2)
        colour=probes->nativeDepthProbes().takeColour(node+0xe0+0x70*value<uint8_t>(rt,node+0x96));
    if(!colour){
        const auto original=rt->functions.generated(0x29a460);
        if(!original)throw std::runtime_error("native readback enqueue missing compiled game code");
        original(ram,c,rt);return;
    }
    if(colour->psm!=2 || colour->width!=64 || colour->height!=416 || colour->y!=0 || colour->x<0 || colour->x%64 || !value<uint8_t>(rt,node+0x65))
        throw std::runtime_error(std::format("unported native colour readback psm {:x} {}x{} at ({},{})",colour->psm,colour->width,colour->height,colour->x,colour->y));
    *static_cast<uint8_t *>(rt->memory.hostPointer(node+0x64,1))=1;   // in flight, as 29a3a0 marks it
    c->r[4].sd[0]=int32_t(node);
    bozo_call_indirect(ram,c,rt,0x299c00);   // its notify: slot flag, scene completion
}
}
void installNativeEntryCapture(Runtime &rt){
    // mod001 (29a460) is registered when it loads; the override is looked up per call.
    rt.functions.setOverride(0x29a460,readbackEnqueue);
}
std::optional<bool> nativeEntryCaptureInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=kSite && pc!=kEnqueue && pc!=kGroupSubmit)return std::nullopt;
    if(!c)throw std::runtime_error("native entry capture site has no CPU record");
    if(pc==kGroupSubmit){
        const auto group=c->r[4].uw[0];
        if(value<uint32_t>(rt,group)!=kGroupClass)throw std::runtime_error("native entry capture group submit outside its class");
        owner(rt).copyNativeEntryFrame(group);
        return false;
    }
    if(pc==kEnqueue){
        const auto job=c->r[5].uw[0];
        if(value<uint32_t>(rt,job)!=kJobClass)throw std::runtime_error("native entry capture enqueue has no 15a8cc job");
        const auto object=job-0x90;
        auto &o=owner(rt);const auto image=o.nativeEntryFrame(object);
        // Before the group's first submit the slices hold no readback yet: nothing is drawn.
        if(!image){nativeEntryCaptureDraw(rt,object,1,1);return false;}
        const auto size=o.nativeEntryFrameSize(object);auto draw=nativeEntryCaptureDraw(rt,object,size[0],size[1]);draw.image=*image;
        o.drawNativeEntryCapture(draw);
        return false;   // the patched call runs the job's completion 308c10(object)
    }
    // Build state 1 after the readback group (+a4) was queued: the state already reads 2.
    const auto object=c->r[21].uw[0];
    if(value<uint32_t>(rt,object)!=kClass || value<uint8_t>(rt,object+0xf8)!=2 || !value<uint32_t>(rt,object+0xa4))
        throw std::runtime_error("native entry capture site outside its build");
    owner(rt).claimNativeEntryFrame(object,value<uint32_t>(rt,object+0xa4));
    return false;
}
}

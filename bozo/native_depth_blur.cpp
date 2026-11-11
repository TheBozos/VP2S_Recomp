#include "native_depth_blur.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <format>
#include <stdexcept>
namespace vp2s {
namespace {
template<class T>T value(Runtime *rt,uint32_t at){
    if(!rt || at<0x100000 || at>0x2000000-sizeof(T))throw std::runtime_error("native depth blur record outside EE RAM");
    T v;std::memcpy(&v,rt->memory.hostPointer(at,sizeof(T)),sizeof(T));return v;
}
template<class T>void store(Runtime *rt,uint32_t at,T v){
    if(!rt || at<0x100000 || at>0x2000000-sizeof(T))throw std::runtime_error("native depth blur record outside EE RAM");
    std::memcpy(rt->memory.hostPointer(at,sizeof(T)),&v,sizeof(T));
}
constexpr uint32_t kContextGlobal=0x35de60,kBuilder=0x1152c0,kBuilderEnd=0x115830;
// The builder's caller-side reads: frame size, colour/depth formats, filtering.
struct ContextFormat {int32_t width,height;bool bilinear;};
ContextFormat format(Runtime *rt,uint32_t context){
    if(value<uint32_t>(rt,kContextGlobal)!=context)throw std::runtime_error("native depth blur writer outside the graphics context");
    const ContextFormat f{value<int16_t>(rt,context+0x3c6c),value<int16_t>(rt,context+0x3c6e),value<uint8_t>(rt,context+0x3c77)!=0};
    // Admitted: PSMCT32 frame (3c70 0), PSMZ24 depth (3c74 0x31), whole 32-pixel strips.
    if(value<int16_t>(rt,context+0x3c70)!=0 || value<int16_t>(rt,context+0x3c74)!=0x31 ||
       f.width<32 || f.width>2048 || f.width%32 || f.height<1 || f.height>2048)
        throw std::runtime_error(std::format("unported native depth blur frame format: {}x{} psm {:x} zpsm {:x}",f.width,f.height,
            value<int16_t>(rt,context+0x3c70),value<int16_t>(rt,context+0x3c74)));
    return f;
}
float gsZ(uint32_t z){return float(z&0xffffffu);}   // PSMZ24 keeps the low 24 bits of XYZ2's Z (the engine's GS rule)
bozo::render::Vertex vertex(float x,float y,float z,float u,float v,uint32_t rgba){
    bozo::render::Vertex out{};out.x=x;out.y=y;out.z=z;out.u=u;out.v=v;out.rgba=rgba;return out;
}
void quad(std::vector<bozo::render::Vertex> &out,float x0,float y0,float x1,float y1,float z,float u0,float v0,float u1,float v1,uint32_t rgba){
    out.push_back(vertex(x0,y0,z,u0,v0,rgba));out.push_back(vertex(x1,y0,z,u1,v0,rgba));out.push_back(vertex(x0,y1,z,u0,v1,rgba));
    out.push_back(vertex(x1,y0,z,u1,v0,rgba));out.push_back(vertex(x1,y1,z,u1,v1,rgba));out.push_back(vertex(x0,y1,z,u0,v1,rgba));
}
int32_t halfTowardZero(int32_t v){return v<0?(v+1)>>1:v>>1;}
}
// 116070: tag + FRAME/FBA/TEST/ZBUF/PRIM/RGBAQ, tag + one XYZ2 pair per 64-pixel
// column, then tag + FRAME/ZBUF restored. 115830: tag + ten registers, tag + two
// UV/XYZ2 pairs per 32-pixel strip, tag + TEST reset, and the EOP tag when last.
uint32_t nativeDepthBlurPlaneQwords(int32_t width){return 8+uint32_t((width+63)/64)+3;}
uint32_t nativeDepthBlurCopyQwords(int32_t width,bool last){return 11+1+2*uint32_t((width+31)/32)+2+(last?1:0);}
std::vector<bozo::render::Vertex> nativeDepthBlurPlaneVertices(const NativeDepthBlur &blur,const NativeDepthBlurPass &pass){
    // Columns from x0 = 0x800 - w/2 in 64-pixel steps, the frame's full height.
    std::vector<bozo::render::Vertex> out;const float x0=float(0x800-blur.width/2),y0=float(0x800-blur.height/2);
    const uint32_t rgba=uint32_t(pass.alpha)<<24|0x010101u;
    for(int32_t x=0;x<blur.width;x+=64)quad(out,x0+float(x),y0,x0+float(x+64),y0+float(blur.height),gsZ(pass.z),0,0,0,0,rgba);
    return out;
}
std::vector<bozo::render::Vertex> nativeDepthBlurCopyVertices(const NativeDepthBlur &blur,const NativeDepthBlurPass &pass){
    // 32-pixel strips moved by dx/16 and (dy halved toward zero)/16; each samples
    // texels u 32k .. 32k+31.5 and v 0 .. h-0.5 of the frame (the writer's -8 in 1/16).
    std::vector<bozo::render::Vertex> out;
    const float x0=float(0x800-blur.width/2)+float(pass.dx)/16.f,y0=float(0x800-blur.height/2)+float(halfTowardZero(pass.dy))/16.f;
    const float w=float(blur.width),h=float(blur.height),v1=(h-0.5f)/h;const uint32_t rgba=uint32_t(pass.alpha)<<24|0x808080u;
    for(int32_t x=0;x<blur.width;x+=32)
        quad(out,x0+float(x),y0,x0+float(x+32),y0+h,gsZ(pass.z),float(x)/w,0,(float(x)+31.5f)/w,v1,rgba);
    return out;
}
void NativeDepthBlurs::plane(Runtime *rt,const R5900Context *c){
    const auto ra=c->r[31].uw[0],context=c->r[4].uw[0],at=c->r[5].uw[0];
    if(ra<=kBuilder || ra>=kBuilderEnd)throw std::runtime_error(std::format("unported native depth-blur plane writer caller {:x}",ra));
    const auto f=format(rt,context);
    if(c->r[6].sw[0]!=f.width || c->r[7].sw[0]!=f.height || c->r[8].uw[0]>0xff)
        throw std::runtime_error("native depth-blur plane escaped the frame rectangle");
    NativeDepthBlurPass pass;pass.z=c->r[9].uw[0];pass.alpha=uint8_t(c->r[8].uw[0]);
    // A build opens at its double-buffer base (+3c88[+3cb0]); later writers continue it.
    const uint32_t index=value<uint8_t>(rt,context+0x3cb0),base=value<uint32_t>(rt,context+0x3c88+4*index);
    if(index>1 || !base)throw std::runtime_error("native depth-blur plane has no packet buffer");
    if(at==base){builds[base]=Build{NativeDepthBlur{context,f.width,f.height,f.bilinear,{}},base,base};}
    auto it=builds.find(base);
    if(it==builds.end() || it->second.end!=at || it->second.blur.context!=context)
        throw std::runtime_error(std::format("native depth-blur plane writer out of sequence at {:x}",at));
    it->second.blur.passes.push_back(pass);it->second.end+=16*nativeDepthBlurPlaneQwords(f.width);
}
void NativeDepthBlurs::copy(Runtime *rt,const R5900Context *c){
    const auto ra=c->r[31].uw[0],context=c->r[4].uw[0],at=c->r[5].uw[0],sp=c->r[29].uw[0];
    if(ra<=kBuilder || ra>=kBuilderEnd)return;   // 115830's other callers are not part of this effect
    const auto f=format(rt,context);
    // The caller's stack may be EE RAM or the scratchpad (the game's update thread).
    const auto arg=[&](unsigned k){const auto *p=rt->memory.hostPointer(sp+8*k,4);
        if(!p)throw std::runtime_error("native depth-blur copy arguments outside owned memory");int32_t v;std::memcpy(&v,p,4);return v;};
    // Caller-side arguments (a2..t3, then the twelve stack doublewords): source
    // x 0, frame FBP, PSMCT32, source and destination sizes, ABE 1, no source
    // offsets, default (-1) extents.
    const auto frameBase=uint32_t(value<uint16_t>(rt,context+0x1e0)&0x1ff);
    if(c->r[6].uw[0] || c->r[7].uw[0]!=frameBase || c->r[8].uw[0] || c->r[9].sw[0]!=f.width || c->r[11].sw[0]!=f.width ||
       c->r[10].sw[0]!=f.height || arg(0)!=f.height || arg(2)!=1 || arg(8) || arg(9) || arg(10)!=-1 || arg(11)!=-1 ||
       (value<uint32_t>(rt,context+0x160)&0x1ff)!=frameBase)
        throw std::runtime_error("unported native depth-blur copy layout");
    NativeDepthBlurPass pass;pass.copy=true;pass.z=uint32_t(arg(1));pass.dx=arg(4);pass.dy=arg(5);
    if(arg(3)<1 || arg(3)>0x80 || arg(6)<1 || arg(6)>0xff || (arg(7)!=0 && arg(7)!=1))
        throw std::runtime_error(std::format("unported native depth-blur copy mode: alpha {:x} reference {:x} last {}",arg(3),arg(6),arg(7)));
    // The writer takes another (unexercised) path for a copy with no offset.
    if(!pass.dx && !pass.dy)throw std::runtime_error("unported native depth-blur copy without offset");
    pass.alpha=uint8_t(arg(3));pass.reference=uint8_t(arg(6));pass.last=arg(7)!=0;
    const uint32_t index=value<uint8_t>(rt,context+0x3cb0),base=value<uint32_t>(rt,context+0x3c88+4*index);
    auto it=builds.find(base);
    if(index>1 || it==builds.end() || it->second.end!=at || it->second.blur.context!=context || it->second.blur.passes.empty())
        throw std::runtime_error(std::format("native depth-blur copy writer out of sequence at {:x}",at));
    it->second.blur.passes.push_back(pass);it->second.end+=16*nativeDepthBlurCopyQwords(f.width,pass.last);
}
std::optional<NativeDepthBlur> NativeDepthBlurs::publish(Runtime *rt,const R5900Context *c){
    // 1ffe88 (the enqueue call 1ffe84's delay slot): 1ffe7c loaded v0 = object+90 (the effect object); its +8c is the context
    // the enqueue's delay slot passes. 114fa0 sends +3c98[i] for +3ca0[i] qwords,
    // i = +3cb1, when the queue (+3c84) exists and both are nonzero.
    const auto owner=c->r[2].uw[0];
    if(owner!=value<uint32_t>(rt,c->r[4].uw[0]+0x90))throw std::runtime_error("native depth-blur publication escaped its draw method");
    const auto context=value<uint32_t>(rt,owner+0x8c);
    if(context!=value<uint32_t>(rt,kContextGlobal))throw std::runtime_error("native depth-blur publication outside the graphics context");
    const auto index=value<uint8_t>(rt,context+0x3cb1);if(index>1)throw std::runtime_error("native depth-blur buffer index");
    const auto start=value<uint32_t>(rt,context+0x3c98+4*index),qwords=value<uint32_t>(rt,context+0x3ca0+4*index);
    if(!value<uint32_t>(rt,context+0x3c84) || !start || !qwords)return std::nullopt;   // the enqueue sends nothing either
    auto it=builds.find(start);
    if(it==builds.end() || uint64_t(it->second.end-it->second.base)!=uint64_t(qwords)*16 || it->second.blur.context!=context)
        throw std::runtime_error(std::format("native depth-blur packet {:x} ({} qwords) was not built by the observed writers",start,qwords));
    auto blur=std::move(it->second.blur);builds.erase(it);
    // Admitted shape: an alpha-0 plane, then per copy a plane and an
    // alpha-tested copy; optionally a final 0x80 plane and plane/copy pairs.
    if(blur.passes.size()<3 || blur.passes.front().copy || blur.passes.front().alpha!=0 || !blur.passes.back().copy || !blur.passes.back().last)
        throw std::runtime_error("unported native depth-blur pass sequence");
    for(size_t i=1;i<blur.passes.size();++i)if(blur.passes[i].last && i+1!=blur.passes.size())throw std::runtime_error("native depth-blur end tag before the last copy");
    return blur;
}
void NativeDepthBlurs::retire(Runtime *rt,uint32_t context){
    // 114ff8's store after a successful enqueue; 114fa0 then finds nothing to send.
    const auto index=value<uint8_t>(rt,context+0x3cb1);store<uint32_t>(rt,context+0x3c98+4*index,0);
}
std::optional<bool> nativeDepthBlurInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x116070 && pc!=0x115830 && pc!=0x1ffe88)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<DepthBlurOwner *>(&d->host()):nullptr;
    if(!owner || !c)throw std::runtime_error("native depth blur requires an owner");
    auto &blurs=owner->nativeDepthBlurs();
    if(pc==0x116070)blurs.plane(rt,c);
    else if(pc==0x115830)blurs.copy(rt,c);
    else if(const auto blur=blurs.publish(rt,c)){owner->drawNativeDepthBlur(*blur);blurs.retire(rt,blur->context);}
    return false; // the original instruction runs: writers still build, the enqueue finds no packet
}
}

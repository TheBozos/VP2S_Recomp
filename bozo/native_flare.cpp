#include "native_flare.h"
#include "native_cast.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/ops_fpu.h"
#include <bit>
#include <cstring>
#include <format>
#include <stdexcept>
namespace vp2s {
namespace {
// The writers' EE FPU in PS2 words: add.s / sub.s, mul.s, div.s, cvt.w.s, cvt.s.w.
struct Ee {
    uint32_t flags=0;
    uint32_t add(uint32_t a,uint32_t b){return bozo::ps2f::add(a,b,flags);}
    uint32_t sub(uint32_t a,uint32_t b){return bozo::ps2f::sub(a,b,flags);}
    uint32_t mul(uint32_t a,uint32_t b){return bozo::ps2f::mul(a,b,flags);}
    uint32_t div(uint32_t a,uint32_t b){return bozo::ps2f::div(a,b,flags);}
    static int32_t toInt(uint32_t a){return int32_t(bozo::ps2f::toInt(a));}
    static uint32_t fromInt(int32_t i){return bozo::ps2f::fromInt(uint32_t(i));}
    static bool lt(uint32_t a,uint32_t b){return bozo::ps2f::lt(a,b);}
    static bool le(uint32_t a,uint32_t b){return bozo::ps2f::le(a,b);}
};
constexpr uint32_t kZero=0,kHalf=0x3f000000u,kSixteen=0x41800000u,k128=0x43000000u,kLimit=0x457ff000u,kFifth=0x3e4ccccdu,kTwoPi=0x40c90fdbu;
uint32_t word(const uint8_t *p,unsigned at){uint32_t v;std::memcpy(&v,p+at,4);return v;}
// cvt.w.s of 128 f with the compiler's unsigned-conversion branch (f >= 2^31: f - 2^31 | 0x80000000), then the
// unsigned clamp to 128 (sltiu 0x81).
uint32_t alphaOf(Ee &e,uint32_t f){
    uint32_t v;if(Ee::le(0x4f000000u,f))v=uint32_t(Ee::toInt(e.sub(f,0x4f000000u)))|0x80000000u;else v=uint32_t(Ee::toInt(f));
    return v<0x81?v:0x80;
}
uint32_t scaled(uint32_t colour,uint32_t own,uint32_t alpha){
    uint32_t out=0;for(unsigned i=0;i<3;++i)out|=nativeFlareChannel((colour>>(8*i))&0xff,(own>>(8*i))&0xff)<<(8*i);
    return out|alpha<<24;
}
// The packet's XYZ2: FTOI(16 x), FTOI(16 y); the native vertex is that 12.4 position in pixels.
bozo::render::Vertex vertex(Ee &e,uint32_t x,uint32_t y,uint32_t rgba){
    bozo::render::Vertex v{};v.x=float(Ee::toInt(e.mul(kSixteen,x)))/16.f;v.y=float(Ee::toInt(e.mul(kSixteen,y)))/16.f;v.z=0;v.rgba=rgba;return v;}
void strip(std::vector<bozo::render::Vertex> &out,const std::vector<bozo::render::Vertex> &s){
    for(size_t k=2;k<s.size();++k){out.push_back(s[k-2]);out.push_back(s[k-1]);out.push_back(s[k]);}}
// The writers' common start: cull the outer radius against the window (0..4095), the alpha, the point count
// FTOI(0.2 r) + 9 (at most 48; none at <= 0), and the angles' unit points (cos x stretch, sin y stretch).
struct Frame {uint32_t alpha=0;int32_t count=0;std::vector<uint32_t> xs,ys;};
bool culled(Ee &e,uint32_t cx,uint32_t cy,uint32_t radius){
    return Ee::lt(e.sub(cx,radius),kZero) || !Ee::le(e.add(cx,radius),kLimit) || Ee::lt(e.sub(cy,radius),kZero) || !Ee::le(e.add(cy,radius),kLimit);}
std::optional<Frame> frame(Ee &e,const uint8_t *element,const NativeFlareCall &call,uint32_t countRadius,const TrigCoefficients &trig){
    Frame f;f.alpha=alphaOf(e,e.mul(k128,e.mul(call.f[4],word(element,0x10))));
    f.count=Ee::toInt(e.mul(kFifth,countRadius))+9;if(f.count>48)f.count=48;if(f.count<=0)return std::nullopt;
    const auto last=Ee::fromInt(f.count-1);
    for(int32_t i=0;i<f.count;++i){
        const auto angle=e.add(call.f[3],e.div(e.mul(kTwoPi,Ee::fromInt(i)),last));
        const auto sc=sineCosine(std::bit_cast<float>(angle),trig);
        f.xs.push_back(e.mul(std::bit_cast<uint32_t>(sc[2]),call.f[5]));f.ys.push_back(e.mul(std::bit_cast<uint32_t>(sc[3]),call.f[6]));}
    return f;
}
}
uint32_t nativeFlareChannel(uint32_t c,uint32_t o){const auto v=uint32_t((uint64_t(c*o)*0x80808081ull)>>32)>>7;return v<0x100?v:0xff;}
// 2550d0: radii r1 = +24, r2 = +24 + +28, r3 = +24 + (+28 + +2c), each times f14. A fan: the centre in colour +14, its
// rim at r1 in +18 (both scaled by the owner's colour); a strip r1 to r2 in the element's own words +18 / +1c, not
// scaled (2556d0 / 255738 OR them with the alpha as they are); a strip r2 (+1c scaled) to r3 (+20 scaled, alpha 0).
std::vector<bozo::render::Vertex> nativeFlareDisc(const uint8_t *element,const NativeFlareCall &call,const TrigCoefficients &trig){
    Ee e;std::vector<bozo::render::Vertex> out;const auto cx=call.f[0],cy=call.f[1],scale=call.f[2];
    const auto outer=e.mul(scale,e.add(word(element,0x24),e.add(word(element,0x2c),word(element,0x28))));
    if(culled(e,cx,cy,outer))return out;
    const auto middle=e.mul(scale,e.add(word(element,0x28),word(element,0x24))),inner=e.mul(word(element,0x24),scale);
    const auto f=frame(e,element,call,middle,trig);if(!f)return out;
    const auto c1=scaled(word(element,0x14),call.colour,f->alpha),c2=scaled(word(element,0x18),call.colour,f->alpha),
               c3=scaled(word(element,0x1c),call.colour,f->alpha),c4=scaled(word(element,0x20),call.colour,0);
    const auto raw2=word(element,0x18)|f->alpha<<24,raw3=word(element,0x1c)|f->alpha<<24;
    const auto at=[&](uint32_t radius,int32_t i,uint32_t rgba){return vertex(e,e.add(cx,e.mul(radius,f->xs[i])),e.add(cy,e.mul(radius,f->ys[i])),rgba);};
    const auto centre=vertex(e,cx,cy,c1);
    for(int32_t i=1;i<f->count;++i){out.push_back(centre);out.push_back(at(inner,i-1,c2));out.push_back(at(inner,i,c2));}
    std::vector<bozo::render::Vertex> s;
    for(int32_t i=0;i<f->count;++i){s.push_back(at(inner,i,raw2));s.push_back(at(middle,i,raw3));}
    strip(out,s);s.clear();
    for(int32_t i=0;i<f->count;++i){s.push_back(at(middle,i,c3));s.push_back(at(outer,i,c4));}
    strip(out,s);return out;
}
// 2549a0: four ring strips; ring k from R_k (R_0 = +28, R_k+1 = R_k + w_k, w_k = +2c + 4k) to R_k + w_k, points
// (R x_i) times f14; colours +14 + 4k inside (alpha 0 for k = 0) and +18 + 4k outside (alpha 0 for k = 3).
std::vector<bozo::render::Vertex> nativeFlareRings(const uint8_t *element,const NativeFlareCall &call,const TrigCoefficients &trig){
    Ee e;std::vector<bozo::render::Vertex> out;const auto cx=call.f[0],cy=call.f[1],scale=call.f[2];
    const auto total=e.add(word(element,0x28),e.add(word(element,0x2c),e.add(word(element,0x30),e.add(word(element,0x38),word(element,0x34)))));
    const auto outer=e.mul(scale,total);
    if(culled(e,cx,cy,outer))return out;
    const auto f=frame(e,element,call,outer,trig);if(!f)return out;
    auto radius=word(element,0x28);
    for(unsigned k=0;k<4;++k){
        const auto width=word(element,0x2c+4*k);
        const auto in=scaled(word(element,0x14+4*k),call.colour,k?f->alpha:0),outside=scaled(word(element,0x18+4*k),call.colour,k==3?0:f->alpha);
        std::vector<bozo::render::Vertex> s;
        for(int32_t i=0;i<f->count;++i){
            s.push_back(vertex(e,e.add(cx,e.mul(scale,e.mul(radius,f->xs[i]))),e.add(cy,e.mul(scale,e.mul(radius,f->ys[i]))),in));
            const auto r=e.add(radius,width);
            s.push_back(vertex(e,e.add(cx,e.mul(scale,e.mul(f->xs[i],r))),e.add(cy,e.mul(scale,e.mul(f->ys[i],r))),outside));}
        strip(out,s);radius=e.add(radius,width);}
    return out;
}
// 253770: rays. Scale s = f14 (0.5 + 0.5 f16); cull by +40 s; per ray (rays: +1c, each 0x44 bytes of +44) with angle
// a = f15 + ray[0] and per segment (+20 of them, at most 8) of angular width +3c: the edge point A (the previous end,
// radius ray[24 + 4 (k - 1)] s; the first ray[24] s), the end B (a + (k + 1) +3c, radius ray[24 + 4 k] s) and the tip
// C (mid angle, radius ray[4 + 4 k] s): triangles (centre, A, B) in +18 | A, +14 | A, +14 | A and (A, B, C) with C's
// alpha 0.
std::vector<bozo::render::Vertex> nativeFlareRays(const uint8_t *element,const uint8_t *rays,const NativeFlareCall &call,const TrigCoefficients &trig){
    Ee e;std::vector<bozo::render::Vertex> out;const auto cx=call.f[0],cy=call.f[1];
    const auto s=e.mul(call.f[2],e.add(kHalf,e.mul(kHalf,call.f[4])));
    if(culled(e,cx,cy,e.mul(word(element,0x40),s)))return out;
    const auto alpha=alphaOf(e,e.mul(k128,e.mul(call.f[4],word(element,0x10))));
    const auto count=int32_t(word(element,0x1c)),segments=int32_t(word(element,0x20));
    if(count<=0 || segments<=0)return out;
    if(segments>8 || count>4096 || !rays)throw std::runtime_error("native flare rays outside their table");
    const auto centreColour=scaled(word(element,0x18),call.colour,alpha),edge=scaled(word(element,0x14),call.colour,alpha),tip=scaled(word(element,0x14),call.colour,0);
    const auto centre=vertex(e,cx,cy,centreColour);const auto half=e.mul(kHalf,word(element,0x3c));
    const auto unit=[&](uint32_t angle){const auto sc=sineCosine(std::bit_cast<float>(angle),trig);
        return std::array<uint32_t,2>{e.mul(std::bit_cast<uint32_t>(sc[2]),call.f[5]),e.mul(std::bit_cast<uint32_t>(sc[3]),call.f[6])};};
    for(int32_t r=0;r<count;++r){const auto *ray=rays+0x44*r;
        auto angle=e.add(call.f[3],word(ray,0));auto radius=e.mul(s,word(ray,0x24));auto previous=unit(angle);angle=e.add(angle,half);
        for(int32_t k=0;k<segments;++k){
            const auto a=vertex(e,e.add(cx,e.mul(previous[0],radius)),e.add(cy,e.mul(previous[1],radius)),edge);
            const auto mid=unit(angle);angle=e.add(angle,half);const auto end=unit(angle);angle=e.add(angle,half);
            radius=e.mul(s,word(ray,0x24+4*k));
            const auto b=vertex(e,e.add(cx,e.mul(end[0],radius)),e.add(cy,e.mul(end[1],radius)),edge);
            const auto tipRadius=e.mul(s,word(ray,4+4*k));
            const auto c=vertex(e,e.add(cx,e.mul(mid[0],tipRadius)),e.add(cy,e.mul(mid[1],tipRadius)),tip);
            out.push_back(centre);out.push_back(a);out.push_back(b);out.push_back(a);out.push_back(b);out.push_back(c);
            previous=end;}}
    return out;
}
// 256404..2565b8: SPRITEs of 64 x height from x = 0x800 - width / 2 (+ 64 i), y = 0x800 - height / 2.
std::vector<bozo::render::Vertex> nativeFlareFlash(uint32_t colour,uint32_t intensity,int16_t width,int16_t height){
    Ee e;std::vector<bozo::render::Vertex> out;
    const auto alpha=uint32_t(Ee::toInt(e.mul(k128,intensity)));const auto rgba=(colour&0xffffff)|alpha<<24;
    const int32_t strips=width/64,x0=0x800-width/2,y0=0x800-height/2,y1=y0+height;
    for(int32_t i=0;i<strips;++i){
        const float l=float(x0+64*i),r=float(x0+64*(i+1));
        const bozo::render::Vertex q[4]={{l,float(y0),0,0,0,rgba,1,1},{r,float(y0),0,0,0,rgba,1,1},{l,float(y1),0,0,0,rgba,1,1},{r,float(y1),0,0,0,rgba,1,1}};
        for(unsigned k:{0u,1u,2u,1u,3u,2u})out.push_back(q[k]);}
    return out;
}
void NativeFlares::add(uint32_t owner,const std::vector<bozo::render::Vertex> &v){
    auto it=builds.find(owner);if(it==builds.end())throw std::runtime_error("native flare geometry outside its owner's build");
    it->second.insert(it->second.end(),v.begin(),v.end());}
std::vector<bozo::render::Vertex> NativeFlares::take(uint32_t owner){
    auto it=builds.find(owner);if(it==builds.end())throw std::runtime_error("native flare drawn without its build");
    auto out=std::move(it->second);builds.erase(it);return out;}
std::optional<bool> nativeFlareInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    static constexpr std::pair<uint32_t,uint32_t> sites[]={{0x255b60,0x27bdfc50},{0x2550d0,0x27bdfda0},{0x2549a0,0x27bdfdb0},{0x253770,0x27bdfeb0},
        {0x256408,0x0000282d},{0x255b50,0x24a4fdf0}};
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](auto &s){return s.first==pc;});
    if(site==std::end(sites) || !rt)return std::nullopt;
    uint32_t w=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&w,p,4);
    if(w!=site->second && !(pc==0x255b50 && w==0x8c24dde0u))return std::nullopt;   // another overlay at these addresses (255b50: the
                                                                                   // kernel's patched delay slot, or the original load)
    auto *d=vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services);auto *owner=d?vp2s::nativeCast<FlareOwner>(&d->host()):nullptr;
    if(!owner || !c)throw std::runtime_error("native flare requires its owner");
    auto &flares=owner->nativeFlares();
    const auto read=[&](uint32_t at,uint32_t bytes){const auto *p=rt->memory.hostPointer(at,bytes);if(!p || at<0x100000)throw std::runtime_error("native flare record outside RAM");return p;};
    const auto field=[&](uint32_t at){uint32_t v;std::memcpy(&v,read(at,4),4);return v;};
    if(pc==0x255b60){if(field(c->r[4].uw[0])!=0x155b50)throw std::runtime_error("native flare build of another class");flares.begin(c->r[4].uw[0]);return false;}
    if(pc==0x255b50){   // the enqueue's delay slot (a1 = object + 210)
        const auto object=c->r[5].uw[0]-0x210;
        if(c->r[5].uw[0]<0x210 || field(object)!=0x155b50)throw std::runtime_error("native flare draw of another class");
        owner->drawNativeFlare(flares.take(object));return false;}
    const auto flare=c->r[19].uw[0];
    if(pc==0x256408){   // the flash block's first writer call, in its delay slot
        const auto globals=c->r[16].uw[0];int16_t width,height;std::memcpy(&width,read(globals+0x3c6c,2),2);std::memcpy(&height,read(globals+0x3c6e,2),2);
        flares.add(flare,nativeFlareFlash(field(flare+0x230),std::bit_cast<uint32_t>(c->f[21]),width,height));return false;}
    // An element's +c, from the build's loop (jalr 2566ec).
    if(c->r[31].uw[0]!=0x2566f4 || field(flare)!=0x155b50)throw std::runtime_error("native flare element outside its build");
    NativeFlareCall call;call.colour=c->r[6].uw[0];for(unsigned i=0;i<7;++i)call.f[i]=std::bit_cast<uint32_t>(c->f[12+i]);
    const auto element=c->r[4].uw[0];const auto *e=read(element,0x48);const auto trig=nativeTrigCoefficients(rt);
    if(pc==0x2550d0)flares.add(flare,nativeFlareDisc(e,call,trig));
    else if(pc==0x2549a0)flares.add(flare,nativeFlareRings(e,call,trig));
    else{const auto count=field(element+0x1c);const auto table=field(element+0x44);
        flares.add(flare,nativeFlareRays(e,count && count<=4096?read(table,0x44*count):nullptr,call,trig));}
    return false;   // the compiled writer still builds its (never kicked) packet
}
}

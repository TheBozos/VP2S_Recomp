#include "native_depth_blur.h"
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <stdexcept>
namespace {
void check(bool ok,const char *what){if(!ok)throw std::runtime_error(what);}
template<class T>void put(Runtime &rt,uint32_t at,T v){std::memcpy(rt.memory.hostPointer(at,sizeof(T)),&v,sizeof(T));}
template<class T>T get(Runtime &rt,uint32_t at){T v;std::memcpy(&v,rt.memory.hostPointer(at,sizeof(T)),sizeof(T));return v;}
bool rejects(const std::function<void()> &f){try{f();}catch(const std::runtime_error &){return true;}return false;}
// The builder runs on the update thread's scratchpad stack (route: sp 70003e50).
constexpr uint32_t ctx=0x68e320,base=0x98ab40,stack=0x70003e00,object=0x700000,effect=0x650110;
// The ship-scene frame of the PCSX2 GS dump (2026-11-11, ~375 s): 512x512 at
// FBP 0xc6, n = 3, both passes. Near planes 84fe0/5ba58/45d20, far 11fee4/
// 164fad/1d56e9; copy offsets as the builder computed them (12.4 x, y doubled).
const uint32_t nearZ[3]={0x84fe0,0x5ba58,0x45d20},farZ[3]={0x11fee4,0x164fad,0x1d56e9};
const int32_t nearOffset[3][2]={{3,0},{-16,29},{-31,-55}},farOffset[3][2]={{4,0},{-25,43},{-48,-83}};
void setUp(Runtime &rt){
    std::memset(rt.memory.flat(),0,bozo::kFlatBytes);
    put<uint32_t>(rt,0x35de60,ctx);put<int16_t>(rt,ctx+0x3c6c,512);put<int16_t>(rt,ctx+0x3c6e,512);
    put<int16_t>(rt,ctx+0x3c70,0);put<int16_t>(rt,ctx+0x3c74,0x31);put<uint8_t>(rt,ctx+0x3c77,1);
    put<uint16_t>(rt,ctx+0x1e0,0xc6);put<uint64_t>(rt,ctx+0x160,0xc6|uint64_t(8)<<16);
    put<uint8_t>(rt,ctx+0x3cb0,1);put<uint32_t>(rt,ctx+0x3c88+4,base);put<uint32_t>(rt,ctx+0x3c84,0x59d030);
    put<uint32_t>(rt,object+0x90,effect);put<uint32_t>(rt,effect+0x8c,ctx);
}
R5900Context plane(uint32_t at,uint32_t z,uint8_t alpha,uint32_t ra=0x115394){
    R5900Context c{};c.r[31].uw[0]=ra;c.r[4].uw[0]=ctx;c.r[5].uw[0]=at;c.r[6].sw[0]=512;c.r[7].sw[0]=512;c.r[8].uw[0]=alpha;c.r[9].uw[0]=z;return c;
}
R5900Context copy(Runtime &rt,uint32_t at,uint32_t z,int32_t alpha,int32_t reference,int32_t dx,int32_t dy,bool last,uint32_t ra=0x1154a4){
    R5900Context c{};c.r[31].uw[0]=ra;c.r[4].uw[0]=ctx;c.r[5].uw[0]=at;c.r[6].uw[0]=0;c.r[7].uw[0]=0xc6;c.r[8].uw[0]=0;
    c.r[9].sw[0]=512;c.r[10].sw[0]=512;c.r[11].sw[0]=512;c.r[29].uw[0]=stack;
    const int64_t args[12]={512,int64_t(z),1,alpha,dx,dy,reference,last,0,0,-1,-1};
    for(unsigned k=0;k<12;++k)put<int64_t>(rt,stack+8*k,args[k]);
    return c;
}
// The route build: clear plane, 3 x (plane, copy), 0x80 plane, 3 x (plane, copy).
uint32_t build(Runtime &rt,vp2s::NativeDepthBlurs &blurs){
    uint32_t at=base;const auto p=vp2s::nativeDepthBlurPlaneQwords(512)*16;
    auto c=plane(at,0xffffff,0);blurs.plane(&rt,&c);at+=p;
    for(unsigned i=0;i<3;++i){
        c=plane(at,nearZ[i],uint8_t(0x40+i));blurs.plane(&rt,&c);at+=p;
        c=copy(rt,at,nearZ[i],0x80,0x40+i,nearOffset[i][0],nearOffset[i][1],false);blurs.copy(&rt,&c);at+=vp2s::nativeDepthBlurCopyQwords(512,false)*16;
    }
    c=plane(at,0xffffff,0x80);blurs.plane(&rt,&c);at+=p;
    for(unsigned i=0;i<3;++i){
        c=plane(at,farZ[i],0);blurs.plane(&rt,&c);at+=p;
        c=copy(rt,at,0xffffff,0x60,1,farOffset[i][0],farOffset[i][1],i==2);blurs.copy(&rt,&c);at+=vp2s::nativeDepthBlurCopyQwords(512,i==2)*16;
    }
    return (at-base)/16;
}
R5900Context publication(){R5900Context c{};c.r[4].uw[0]=object;c.r[2].uw[0]=effect;return c;}
void publishable(Runtime &rt,uint32_t qwords){put<uint8_t>(rt,ctx+0x3cb1,1);put<uint32_t>(rt,ctx+0x3c98+4,base);put<uint32_t>(rt,ctx+0x3ca0+4,qwords);}
}
int main(){try{
    Runtime rt;unsigned checks=0;
    // Writer sizes: 19 qwords per plane, 46 per copy (+1 end tag) at 512 wide; the
    // route's packet is the traced 429-qword type-2 entry.
    check(vp2s::nativeDepthBlurPlaneQwords(512)==19 && vp2s::nativeDepthBlurCopyQwords(512,false)==46 && vp2s::nativeDepthBlurCopyQwords(512,true)==47,"writer sizes");
    setUp(rt);vp2s::NativeDepthBlurs blurs;const auto qwords=build(rt,blurs);check(qwords==429,"route packet size");
    publishable(rt,qwords);const auto pub=publication();
    const auto blur=blurs.publish(&rt,&pub);check(blur && blurs.pending()==0,"publication");
    check(blur->width==512 && blur->height==512 && blur->bilinear && blur->passes.size()==14,"pass list");
    // Pass order and state as in the dump.
    const auto &ps=blur->passes;
    check(!ps[0].copy && ps[0].alpha==0 && ps[0].z==0xffffff,"clear plane");
    for(unsigned i=0;i<3;++i){
        const auto &pl=ps[1+2*i],&cp=ps[2+2*i];
        check(!pl.copy && pl.alpha==0x40+i && pl.z==nearZ[i],"near plane");
        check(cp.copy && cp.alpha==0x80 && cp.reference==0x40+i && cp.z==nearZ[i] && !cp.last,"near copy");
        const auto &fp=ps[8+2*i],&fc=ps[9+2*i];
        check(!fp.copy && fp.alpha==0 && fp.z==farZ[i],"far plane");
        check(fc.copy && fc.alpha==0x60 && fc.reference==1 && fc.z==0xffffff && fc.last==(i==2),"far copy");
    }
    check(!ps[7].copy && ps[7].alpha==0x80 && ps[7].z==0xffffff,"final plane");checks+=14;
    // Geometry against the dump's first strip of each copy (x0, y0) and texels.
    const float nearXY[3][2]={{1792.1875f,1792.f},{1791.f,1792.875f},{1790.0625f,1790.3125f}},farXY[3][2]={{1792.25f,1792.f},{1790.4375f,1793.3125f},{1789.f,1789.4375f}};
    using Probe=std::pair<const vp2s::NativeDepthBlurPass *,const float *>;
    for(unsigned i=0;i<3;++i)for(const auto &[pass,xy]:std::array<Probe,2>{Probe{&ps[2+2*i],nearXY[i]},Probe{&ps[9+2*i],farXY[i]}}){
        const auto v=vp2s::nativeDepthBlurCopyVertices(*blur,*pass);check(v.size()==16*6,"strip count");
        check(v[0].x==xy[0] && v[0].y==xy[1] && v[4].x==xy[0]+32 && v[4].y==xy[1]+512,"strip 0 position");
        check(v[6].x==xy[0]+32 && v[90+4].x==xy[0]+512,"strip steps");
        check(v[0].u==0 && v[0].v==0 && v[4].u==31.5f/512 && v[4].v==511.5f/512 && v[6].u==32.f/512,"strip texels");
        check(v[0].z==float(pass->z&0xffffff) && v[0].rgba==(uint32_t(pass->alpha)<<24|0x808080u),"strip colour/depth");checks+=5;
    }
    for(unsigned i=0;i<14;++i)if(!ps[i].copy){
        const auto v=vp2s::nativeDepthBlurPlaneVertices(*blur,ps[i]);check(v.size()==8*6,"plane columns");
        check(v[0].x==1792 && v[0].y==1792 && v[4].x==1856 && v[4].y==2304 && v[42+4].x==2304,"plane rectangle");
        check(v[0].rgba==(uint32_t(ps[i].alpha)<<24|0x010101u) && v[0].z==float(ps[i].z),"plane alpha/depth");checks+=3;
    }
    // Retirement: the published start is cleared as 114ff8 would, so 114fa0 sends nothing.
    blurs.retire(&rt,ctx);check(get<uint32_t>(rt,ctx+0x3c98+4)==0 && get<uint32_t>(rt,ctx+0x3ca0+4)==qwords,"retire");
    check(!blurs.publish(&rt,&pub),"nothing left to publish");checks+=2;
    // First-frame leftovers: Z past 24 bits keeps its low 24 bits (the engine's GS rule).
    {vp2s::NativeDepthBlur leftover{ctx,512,512,true,{}};vp2s::NativeDepthBlurPass p;p.z=0xdfff005a;p.alpha=0x41;
        check(vp2s::nativeDepthBlurPlaneVertices(leftover,p)[0].z==float(0xff005a),"leftover Z");++checks;}
    // No queue: the enqueue sends nothing, so nothing is drawn or retired.
    setUp(rt);blurs=vp2s::NativeDepthBlurs{};build(rt,blurs);publishable(rt,429);put<uint32_t>(rt,ctx+0x3c84,0);
    check(!blurs.publish(&rt,&pub) && get<uint32_t>(rt,ctx+0x3c98+4)==base,"no queue");++checks;
    // Rejections.
    setUp(rt);blurs=vp2s::NativeDepthBlurs{};build(rt,blurs);publishable(rt,428);
    check(rejects([&]{blurs.publish(&rt,&pub);}),"size mismatch");
    setUp(rt);blurs=vp2s::NativeDepthBlurs{};publishable(rt,429);check(rejects([&]{blurs.publish(&rt,&pub);}),"unbuilt packet");
    setUp(rt);blurs=vp2s::NativeDepthBlurs{};
    {auto c=plane(base,0xffffff,0,0x115900);check(rejects([&]{blurs.plane(&rt,&c);}),"plane caller");}
    {auto c=plane(base+16,0xffffff,0);check(rejects([&]{blurs.plane(&rt,&c);}),"plane out of sequence");}
    {auto c=plane(base,0xffffff,0);blurs.plane(&rt,&c);const auto at=base+19*16;
        auto o=copy(rt,at,nearZ[0],0x80,0x40,3,0,false,0x115120);blurs.copy(&rt,&o);   // another 115830 caller: not this effect
        auto z=copy(rt,at,nearZ[0],0x80,0,3,0,false);check(rejects([&]{blurs.copy(&rt,&z);}),"no alpha test");
        auto s=copy(rt,at,nearZ[0],0x80,0x40,0,0,false);check(rejects([&]{blurs.copy(&rt,&s);}),"no offset");
        auto g=copy(rt,at+16,nearZ[0],0x80,0x40,3,0,false);check(rejects([&]{blurs.copy(&rt,&g);}),"copy gap");
        auto w=copy(rt,at,nearZ[0],0x80,0x40,3,0,false);w.r[10].sw[0]=448;check(rejects([&]{blurs.copy(&rt,&w);}),"copy size");
        auto ok=copy(rt,at,nearZ[0],0x80,0x40,3,0,true);blurs.copy(&rt,&ok);}
    {setUp(rt);put<int16_t>(rt,ctx+0x3c74,0x30);auto c=plane(base,0xffffffff,0);check(rejects([&]{blurs.plane(&rt,&c);}),"Z32 frame");}
    {setUp(rt);put<int16_t>(rt,ctx+0x3c6c,500);auto c=plane(base,0xffffff,0);c.r[6].sw[0]=500;check(rejects([&]{blurs.plane(&rt,&c);}),"partial strips");}
    // A published packet that is only a plane and a copy is not the admitted shape.
    setUp(rt);blurs=vp2s::NativeDepthBlurs{};
    {auto c=plane(base,0xffffff,0);blurs.plane(&rt,&c);auto o=copy(rt,base+19*16,nearZ[0],0x80,0x40,3,0,true);blurs.copy(&rt,&o);
        publishable(rt,19+47);check(rejects([&]{blurs.publish(&rt,&pub);}),"short sequence");}
    checks+=11;
    std::printf("%u depth-blur checks: route packet 429 qwords, 14 passes, dump strip positions/texels, retirement, leftovers and rejections\n",checks);
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

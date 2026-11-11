#include "native_depth_probe.h"
#include <cstdio>
#include <cstring>
#include <functional>
#include <stdexcept>
namespace {
void check(bool ok,const char *what){if(!ok)throw std::runtime_error(what);}
template<class T>void put(Runtime &rt,uint32_t at,T v){std::memcpy(rt.memory.hostPointer(at,sizeof(T)),&v,sizeof(T));}
template<class T>T get(Runtime &rt,uint32_t at){T v;std::memcpy(&v,rt.memory.hostPointer(at,sizeof(T)),sizeof(T));return v;}
bool rejects(const std::function<void()> &f){try{f();}catch(const std::runtime_error &){return true;}return false;}
// The route's node (serial 13534, frame 25485): object 1187cb0, slot 1, packet
// 1187e00 (requested through its uncached alias), destination 1187870 (probe
// owner +20), PSMZ24 16x16 at (3868, 10492) of the 512-wide Z buffer at block 0.
constexpr uint32_t object=0x1187cb0,destination=0x1187870;
const vp2s::NativeDepthProbeFrame frame{512,512};
R5900Context request(int32_t x,int32_t y,int32_t w=16,int32_t h=16,int32_t psm=0x31,uint32_t packet=0x21187e00,uint32_t ra=0x29a584,int32_t sbw=8){
    R5900Context c{};c.r[31].uw[0]=ra;c.r[4].uw[0]=packet;c.r[5].uw[0]=0;c.r[6].uw[0]=uint32_t(sbw);c.r[7].uw[0]=uint32_t(psm);
    c.r[8].uw[0]=uint32_t(x)&0xffff;c.r[9].uw[0]=uint32_t(y)&0xffff;c.r[10].uw[0]=uint32_t(w);c.r[11].uw[0]=uint32_t(h);return c;
}
R5900Context enqueue(uint32_t node=object,uint32_t pc=0x29a4b0){R5900Context c{};c.pc=pc;c.r[7].uw[0]=node+0x90;return c;}
void node(Runtime &rt,uint8_t slot=1){
    std::memset(rt.memory.flat(),0,bozo::kFlatBytes);
    put<uint32_t>(rt,object,0x159010);put<uint32_t>(rt,object+0x90,0x15908c);put<uint8_t>(rt,object+0x96,slot);
    put<uint32_t>(rt,object+0x200+4*slot,destination);
}
// A depth picture: GS Z from the sample's buffer pixel.
uint32_t zAt(int32_t px,int32_t py){return uint32_t(py*512+px)*7+0x1000;}
bool picture(float gsX,float gsY,uint32_t &z){
    const float fx=gsX-1792.0f,fy=gsY-1792.0f;
    if(fx!=float(int32_t(fx))+0.5f || fy!=float(int32_t(fy))+0.5f)throw std::runtime_error("sample not at a pixel centre");
    z=zAt(int32_t(fx),int32_t(fy));return true;
}
uint32_t z24(const std::vector<uint8_t> &b,size_t k){return b[3*k]|b[3*k+1]<<8|b[3*k+2]<<16;}
}
int main(){try{
    Runtime rt;unsigned checks=0;vp2s::NativeDepthProbes probes;
    // Route request: far off the 512x512 frame, so every sample reads as hidden
    // (all-ones Z24); the consumer counts none below its light's Z.
    {node(rt);auto c=request(3868,10492);probes.request(&c);check(probes.pending()==1,"request recorded");++checks;
     auto e=enqueue();check(probes.complete(&rt,&e,frame,picture)==object,"route node completes");++checks;
     bool hidden=true;for(uint32_t k=0;k<16*16*3;++k)hidden=hidden && get<uint8_t>(rt,destination+k)==0xff;
     check(hidden,"off-frame route probe reads hidden");++checks;
     check(get<uint8_t>(rt,object+0x64)==1 && probes.pending()==0,"in flight, request consumed");++checks;
     check(get<uint8_t>(rt,destination+16*16*3)==0,"exactly 768 bytes written");++checks;
     check(rejects([&]{auto again=enqueue();probes.complete(&rt,&again,frame,picture);}),"a second enqueue has no request");++checks;}
    // On-frame Z24: rows top-down, 3 packed bytes per sample, read at pixel centres
    // of GS 2048-256+x.
    {const vp2s::NativeDepthProbeRequest q{0x1187e00,0,8,0x31,100,200,16,16};
     const auto b=vp2s::nativeDepthProbeBytes(q,frame,picture);check(b.size()==768,"Z24 size");++checks;
     bool ok=true;for(int32_t j=0;j<16;++j)for(int32_t i=0;i<16;++i)ok=ok && z24(b,size_t(j*16+i))==zAt(100+i,200+j);
     check(ok,"Z24 layout and coordinates");++checks;
     // The consumer (253360): count of samples below the light's Z, over 256.
     const uint32_t light=zAt(100,208);unsigned visible=0;for(size_t k=0;k<256;++k)visible+=z24(b,k)<light;
     check(visible==128,"half the probe lies below the light");++checks;}
    // Partly off the frame's right edge: columns 512.. read hidden.
    {const vp2s::NativeDepthProbeRequest q{0,0,8,0x31,504,0,16,1};
     const auto b=vp2s::nativeDepthProbeBytes(q,frame,picture);bool ok=true;
     for(int32_t i=0;i<16;++i)ok=ok && z24(b,size_t(i))==(504+i<512?zAt(504+i,0):0xffffffu);
     check(ok,"edge columns");++checks;}
    // No renderer depth (headless, tile not read yet): hidden.
    {const vp2s::NativeDepthProbeRequest q{0,0,8,0x31,0,0,4,4};
     const auto b=vp2s::nativeDepthProbeBytes(q,frame,[](float,float,uint32_t &){return false;});bool ok=true;
     for(size_t k=0;k<16;++k)ok=ok && z24(b,k)==0xffffffu;check(ok,"unavailable depth reads hidden");++checks;}
    // Z32 keeps 4 bytes; Z24 clamps a deeper value to 24 bits.
    {const auto deep=[](float,float,uint32_t &z){z=0x12345678;return true;};
     const auto b32=vp2s::nativeDepthProbeBytes({0,0,8,0x30,0,0,1,1},frame,deep);
     check(b32.size()==4 && b32[0]==0x78 && b32[3]==0x12,"Z32 bytes");++checks;
     const auto b24=vp2s::nativeDepthProbeBytes({0,0,8,0x31,0,0,1,1},frame,deep);
     check(b24.size()==3 && z24(b24,0)==0xffffff,"Z24 clamp");++checks;}
    // Rejections.
    check(rejects([&]{vp2s::nativeDepthProbeBytes({0,0,8,0x00,0,0,16,16},frame,picture);}),"colour format");++checks;
    check(rejects([&]{vp2s::nativeDepthProbeBytes({0,0,8,0x32,0,0,16,16},frame,picture);}),"Z16");++checks;
    check(rejects([&]{vp2s::nativeDepthProbeBytes({0,0,8,0x31,0,0,65,16},frame,picture);}),"size");++checks;
    check(rejects([&]{vp2s::nativeDepthProbeBytes({0,0,10,0x31,0,0,16,16},frame,picture);}),"buffer width");++checks;
    check(rejects([&]{vp2s::nativeDepthProbeBytes({0,0,8,0x31,0,0,16,16},{0,0},picture);}),"no frame");++checks;
    check(rejects([&]{auto c=request(0,0,16,16,0x31,0x21187e00,0x29a588);probes.request(&c);}),"foreign caller");++checks;
    const auto completeWith=[&](const std::function<void()> &arrange,uint32_t pc=0x29a4b0){
        node(rt);auto c=request(0,0);probes.request(&c);arrange();auto e=enqueue(object,pc);
        const bool failed=rejects([&]{probes.complete(&rt,&e,frame,picture);});probes=vp2s::NativeDepthProbes{};return failed;};
    check(!completeWith([]{}),"baseline completes");++checks;
    check(completeWith([]{},0x29a4c0),"not the original enqueue");++checks;
    check(completeWith([&]{put<uint32_t>(rt,object,0x153700);}),"class");++checks;
    check(completeWith([&]{put<uint32_t>(rt,object+0x90,0);}),"job class");++checks;
    check(completeWith([&]{put<uint8_t>(rt,object+0x65,1);}),"owner queue");++checks;
    check(completeWith([&]{put<uint8_t>(rt,object+0x96,2);}),"slot");++checks;
    check(completeWith([&]{put<uint8_t>(rt,object+0x96,0);}),"other slot has no request");++checks;
    check(completeWith([&]{put<uint32_t>(rt,object+0x204,0x2000000-0x100);}),"destination");++checks;
    std::printf("depth probe checks: %u passed\n",checks);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"FAILED: %s\n",e.what());return 1;}}

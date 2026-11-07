#include "native_ui_windows.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "window_original_functions.h"
#include "window_adapted_functions.h"
#include <fstream>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <source_location>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("window assertion at "+std::to_string(at.line()));}
template<class T>void put(Runtime &rt,uint32_t a,T v){std::memcpy(rt.memory.hostPointer(a,sizeof(v)),&v,sizeof(v));}
template<class F>void rejects(F f){bool caught=false;try{f();}catch(const std::exception &){caught=true;}check(caught);}
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::UiWindowOwner,vp2s::UiSpriteOwner {
    vp2s::NativeUiWindows windows;vp2s::NativeUiSprites sprites;
    vp2s::NativeUiWindows &nativeUiWindows()override{return windows;}
    vp2s::NativeUiSprites &nativeUiSprites()override{return sprites;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc)override{auto handled=vp2s::nativeUiWindowInstruction(c,rt,pc);check(handled.has_value());return *handled;}
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("library");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("I/O");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("I/O");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &why)override{throw std::runtime_error(why);}
};
}
int main(int argc,char **argv){try{
    check(argc==2);const std::filesystem::path fixture=argv[1];Host before,after;bozo::ee::Dispatcher old(before),native(after);
    old.setFunctions(window_original_function_table,window_original_function_count,window_original_resume_table,window_original_resume_count);
    native.setFunctions(window_adapted_function_table,window_adapted_function_count,window_adapted_resume_table,window_adapted_resume_count);
    old.boot(0x2e9640,0x1fff000);native.boot(0x2e9640,0x1fff000);
    old.runtime().port=&before;native.runtime().port=&after;
    vp2s::NativeTextures textures(fixture/"assets",fixture/"assets/manifest.txt");
    std::array<uint8_t,192> topology{};std::ifstream in(fixture/"topology.bin",std::ios::binary);check(bool(in.read(reinterpret_cast<char *>(topology.data()),topology.size())));
    constexpr uint32_t element=0x190000,layer=0x1b0000,manager=0x1c0000,dtt=0x1d0000,image=0x1e0000,fis=0x1e1000,stack=0x1fff000;
    for(unsigned seed=0;seed<64;++seed){
        auto &a=old.runtime(),&b=native.runtime();std::memset(a.memory.flat(),0,bozo::kFlatBytes);
        std::memcpy(a.memory.hostPointer(0x357900,topology.size()),topology.data(),topology.size());
        put(a,element,0x15a1b0u);put(a,element+0x14,layer);put(a,element+0x28,64.f+float(seed));put(a,element+0x2c,7u);put(a,element+0x48,0x200000u);
        put<uint8_t>(a,element+0xf6,1);put<uint8_t>(a,element+0xf1,seed&1);put<uint8_t>(a,element+0xf2,(seed>>1)&1);put<uint8_t>(a,element+0xf5,(seed>>2)&1);
        put(a,element+0xe8,seed&8?1u:0u);const auto rectangle=element+(seed&8?0xb8:0x18);
        put(a,rectangle,10.25f);put(a,rectangle+4,20.5f);put(a,rectangle+8,seed&16?120.f:80.f);put(a,rectangle+12,seed&32?128.f:64.f);
        put(a,layer+0x40,10.f);put(a,layer+0x44,20.f);put(a,layer+0xe0,128.f);put(a,layer+0xc4,0xff0000u);
        put(a,0x35e170,manager);put(a,manager,0x15a1e0u);put(a,manager+0x82c,image);
        put(a,manager+0x840,16.f);put(a,manager+0x844,16.f);put(a,manager+0x848,27648.f);put(a,manager+0x84c,29184.f);
        put(a,image,0x15a6f0u);put(a,image+0x90,0x15a76cu);put(a,image+0xc0,fis);put(a,fis,0x534946u);put(a,fis+0x14,0x00494853u);put(a,fis+0x18,0x3du);
        put(a,dtt+0x10,9u);put(a,element+0x54,dtt);put(a,element+0x58,dtt+0x14);put<uint16_t>(a,element+0xa6,9);
        put<int8_t>(a,element+0xb4,-1);put<uint8_t>(a,element+0xb0,4);put<uint8_t>(a,element+0xb1,4);
        for(unsigned i=0;i<9;++i){const auto part=dtt+0x14+16*i;
            put<uint16_t>(a,part,0x400);put<uint16_t>(a,part+2,0x800);put<uint16_t>(a,part+4,0xc00);put<uint16_t>(a,part+6,0x1000);
            put<uint16_t>(a,part+8,16);put<uint16_t>(a,part+10,24);
        }
        for(auto offset:{0x90u,0x94u,0x98u})put(a,element+offset,128.f);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);after.sprites.bindButton(&b,textures,element+0x50,0x27);
        R5900Context x{},y{};x.r[4].ud[0]=element;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;y=x;
        old.runtime().functions.generated(0x2e9640)(a.memory.flat(),&x,&a);native.runtime().functions.generated(0x2e9640)(b.memory.flat(),&y,&b);x.cycleLimit=y.cycleLimit;
        check(x.pc==0x170000 && y.pc==0x170000);check(!std::memcmp(&x,&y,sizeof(x)));check(!std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        const auto captured=after.windows.window(element);check(captured && captured->background.size()==(seed&1?9:0) && captured->border.size()==(seed&2?9:0));
        if(seed&1){const auto &s=captured->background[0];const auto colour=(seed&4?0x2d1612u:0x44211bu)|(uint32_t((64.f+seed)*0.95f)<<24);
            check(s.colours[0]==colour && s.depth==0xff0007 && s.corners[2]==std::array<int32_t,2>{27972,29832});
            const auto vertices=vp2s::nativeUiStripVertices(s);check(vertices[2].x==1748.25f && vertices[2].y==1864.5f && vertices[0].rgba==colour);
        }
        if(seed&2){const auto &s=captured->border[0];check(s.picture->rgba[0]==0x8040203du && s.depth==0xff0007);
            check(s.fixedCorners[0]==std::array<int32_t,2>{27844,29704});check(s.uv==std::array<float,4>{0.25f,0.5f,0.75f,1.f});}
        y={};y.r[4].ud[0]=element;y.r[29].ud[0]=stack;y.r[31].ud[0]=0x170000;
        native.runtime().functions.generated(0x2e9640)(b.memory.flat(),&y,&b);check(after.windows.window(element)==captured);
        after.windows.release(element);after.sprites.reset(element+0x50);check(!after.windows.window(element));
        check(captured->background.size()==(seed&1?9:0));
        if(seed&2)check(captured->border[0].picture->rgba[0]==0x8040203du);
    }
    std::puts("64 complete scalar window CPU/RAM cases: background/edge/border capture, original rounded geometry/UV/colour, cached rebuild, null FIS atlas lease and snapshot lifetime");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

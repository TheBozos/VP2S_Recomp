#include "native_ui_windows.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "window_original_functions.h"
#include "window_adapted_functions.h"
#include <fstream>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <functional>
#include <source_location>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("window assertion at "+std::to_string(at.line()));}
template<class T>void put(Runtime &rt,uint32_t a,T v){std::memcpy(rt.memory.hostPointer(a,sizeof(v)),&v,sizeof(v));}
template<class F>void rejects(F f){bool caught=false;try{f();}catch(const std::exception &){caught=true;}check(caught);}
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::UiWindowOwner,vp2s::UiSpriteOwner,vp2s::TextureOwner {
    vp2s::NativeUiWindows windows;vp2s::NativeUiSprites sprites;vp2s::NativeTextures *textures=nullptr;
    vp2s::NativeTextures &nativeTextures()override{if(!textures)throw std::runtime_error("textures");return *textures;}
    vp2s::NativeUiWindows &nativeUiWindows()override{return windows;}
    vp2s::NativeUiSprites &nativeUiSprites()override{return sprites;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc)override{auto handled=vp2s::nativeUiWindowInstruction(c,rt,pc);check(handled.has_value());return *handled;}
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *c,uint32_t pc)override{
        check(pc==0x125090 && std::isfinite(c->f[12]) && c->f[12]>=0 && c->f[12]<=65535);
        // Shared bounded float-to-integer callback; inputs are already scalar
        // integer coordinates from the original builder's cvt.w.s/cvt.s.w.
        c->r[2].ud[0]=uint64_t(int64_t(c->f[12]));c->pc=c->r[31].uw[0];
    }
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
    old.bindLibrary(0x125090);native.bindLibrary(0x125090);
    old.runtime().port=&before;native.runtime().port=&after;
    vp2s::NativeTextures textures(fixture/"assets",fixture/"assets/manifest.txt");after.textures=&textures;
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
    for(unsigned seed=0;seed<64;++seed){
        auto &a=old.runtime(),&b=native.runtime();std::memset(a.memory.flat(),0,bozo::kFlatBytes);
        put(a,element,0x15ac30u);put(a,element+0x14,layer);put(a,element+0x28,64.f+seed);put(a,element+0x2c,6u);put(a,element+0x48,0x200000u);
        put(a,element+0x18,10.25f+float(seed)/16);put(a,element+0x1c,20.5f);put(a,element+0x20,80.f);put(a,element+0x24,16.f);
        put(a,layer+0x40,10.f);put(a,layer+0x44,20.f);put(a,layer+0xe0,128.f);put(a,layer+0xc4,0xff0000u);
        put(a,0x35e170,manager);put(a,manager+0x840,16.f);put(a,manager+0x844,16.f);put(a,manager+0x848,27648.f);put(a,manager+0x84c,29184.f);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);R5900Context x{},y{};
        x.r[4].ud[0]=element;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;y=x;
        old.runtime().functions.generated(0x30ee00)(a.memory.flat(),&x,&a);native.runtime().functions.generated(0x30ee00)(b.memory.flat(),&y,&b);x.cycleLimit=y.cycleLimit;
        check(x.pc==0x170000 && y.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        const auto captured=after.windows.window(element);check(captured && captured->background.size()==3 && captured->border.empty());
        const auto alpha=uint32_t((64.f+seed)/4)<<24;const int32_t left=27972+seed;
        for(unsigned i=0;i<3;++i){const auto &s=captured->background[i];check(s.depth==0xff0006);
            const int32_t xx[4]={left,left+256,left+1024,left+1280};
            check(s.corners==std::array<std::array<int32_t,2>,4>{{{xx[i],29832},{xx[i+1],29832},{xx[i],30088},{xx[i+1],30088}}});
            check(s.colours==std::array<uint32_t,4>{i?alpha:0,i==2?0:alpha,i?alpha:0,i==2?0:alpha});
        }
        y={};y.r[4].ud[0]=element;y.r[29].ud[0]=stack;y.r[31].ud[0]=0x170000;
        native.runtime().functions.generated(0x30ee00)(b.memory.flat(),&y,&b);check(after.windows.window(element)==captured);
        after.windows.release(element);check(captured->background[0].colours[1]==alpha && !after.windows.window(element));
    }
    auto &rt=native.runtime();put<uint8_t>(rt,element+0x3b,0);after.windows.beginSelection(&rt,stack-0x90,element);
    const auto unchanged=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
    rejects([&]{after.windows.release(element);});rejects([&]{after.windows.selectionColour(stack-0x90,256,0);});
    after.windows.selectionColour(stack-0x90,64,0xff0006);rejects([&]{after.windows.selectionColour(stack-0x90,64,0xff0006);});
    rejects([&]{after.windows.selectionCoordinate(stack-0x90,0,-1);});after.windows.selectionCoordinate(stack-0x90,0,27972);
    rejects([&]{after.windows.selectionCoordinate(stack-0x90,0,27972);});rejects([&]{after.windows.finishSelection(stack-0x90,element);});
    for(unsigned i=1;i<6;++i)after.windows.selectionCoordinate(stack-0x90,i,30000+i);
    after.windows.finishSelection(stack-0x90,element);after.windows.release(element);check(!std::memcmp(unchanged.data(),rt.memory.flat(),unchanged.size()));
    // Camp menu image list 15aa20 (build 30c5a0, quad list 2ead10 returning to
    // 30c628): the sprite's record (+5a page, +5c palette page, +5f format,
    // +60/+61 log2 size) resolves the menu image the loader published under
    // that handle; the original and the native-site builds stay identical.
    textures.publishMenuPage(0x800,0x2c,0x3e48,{0x00494853,0x3d});
    unsigned imageCases=0;
    for(unsigned seed=0;seed<32;++seed){
        auto &a=old.runtime(),&b=native.runtime();std::memset(a.memory.flat(),0,bozo::kFlatBytes);
        std::memcpy(a.memory.hostPointer(0x357900,topology.size()),topology.data(),topology.size()); // includes the size table 357990
        const uint32_t list=0x1d1000,sprite=element+0x50;const int16_t count=int16_t(1+seed%4);
        put(a,element,0x15aa20u);put(a,element+0x14,layer);put<uint8_t>(a,element+0xc8,1);put(a,element+0x2c,5u);put(a,element+0x48,0x200000u);
        put(a,element+0x18,4.f*float(seed));put(a,element+0x1c,8.f);put(a,element+0x28,128.f-float(seed));
        put(a,layer,0x152060u);put(a,layer+0x10,0x167730u);put(a,layer+0x40,10.f);put(a,layer+0x44,20.f);put(a,layer+0xe0,128.f);put(a,layer+0xc4,0xff0000u);
        put(a,0x35e170,manager);put(a,manager+0x840,16.f);put(a,manager+0x844,16.f);put(a,manager+0x848,27648.f);put(a,manager+0x84c,29184.f);
        put(a,dtt,0x545444u);put<uint16_t>(a,dtt+0x10,7);put(a,sprite+4,dtt);put(a,sprite+8,dtt+0x14);put(a,sprite+12,list);
        put<uint16_t>(a,sprite+0x56,7);put<int16_t>(a,sprite+0x58,count);put<uint16_t>(a,sprite+0x5a,0x800);put<uint16_t>(a,sprite+0x5c,0x3e48);
        put<uint8_t>(a,sprite+0x5e,8);put<uint8_t>(a,sprite+0x5f,0x2c);put<uint8_t>(a,sprite+0x60,4);put<uint8_t>(a,sprite+0x61,4);put<int8_t>(a,sprite+0x64,seed&1?-1:0);
        for(unsigned i=0;i<7;++i){const auto part=dtt+0x14+16*i;
            // Part UVs are 1/4096 fractions of the picture: texel i..i+4 x 4..8.
            put<uint16_t>(a,part,uint16_t(0x100*i));put<uint16_t>(a,part+2,0x400);put<uint16_t>(a,part+4,uint16_t(0x100*i+0x400));put<uint16_t>(a,part+6,0x800);
            put<uint16_t>(a,part+8,4);put<uint16_t>(a,part+10,6);}
        for(int16_t i=0;i<count;++i){const auto entry=list+24u*uint32_t(i);
            put(a,entry,uint32_t((seed+uint32_t(i))%7));put(a,entry+4,8.f*float(i));put(a,entry+8,float(seed));put(a,entry+12,seed&2?12.f:0.f);put(a,entry+16,seed&4?10.f:0.f);put(a,entry+20,uint32_t(i));}
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{},y{};x.r[4].ud[0]=element;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;y=x;
        old.runtime().functions.generated(0x30c5a0)(a.memory.flat(),&x,&a);native.runtime().functions.generated(0x30c5a0)(b.memory.flat(),&y,&b);x.cycleLimit=y.cycleLimit;
        check(x.pc==0x170000 && y.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        const auto captured=after.windows.window(element);check(captured && captured->background.empty() && captured->border.size()==size_t(count));
        for(const auto &s:captured->border){check(s.picture && s.picture->width==16 && s.depth>=0xff0005 && s.depth<=0xff0005+3);}
        const auto first=captured->border[0];const auto part=(seed%7);
        check(first.uv==std::array<float,4>{float(part)/16,4.f/16,float(part+4)/16,8.f/16});
        after.windows.release(element);check(!after.windows.window(element));++imageCases;
        // Unpublished handle, an owner that is no UI layer: rejected before any capture.
        for(unsigned spoil=0;spoil<3;++spoil){
            std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);put<uint8_t>(b,element+0x3b,0);
            if(spoil==0)put<uint16_t>(b,sprite+0x5c,0x3e40);else if(spoil==1)put(b,layer,0x15a1e0u);else put<uint8_t>(b,sprite+0x60,5);
            y={};y.r[4].ud[0]=element;y.r[29].ud[0]=stack;y.r[31].ud[0]=0x170000;
            rejects([&]{native.runtime().functions.generated(0x30c5a0)(b.memory.flat(),&y,&b);});++imageCases;
            after.windows=vp2s::NativeUiWindows{};
        }
    }
    // Camp menu panel 167560 (build 460d20, site 460f24): the compiled corner,
    // depth and alpha scalars plus the element colours become one strip.
    {auto &r=native.runtime();std::memset(r.memory.flat(),0,bozo::kFlatBytes);constexpr uint32_t panel=0x190400;
        put(r,panel,0x167560u);for(unsigned i=0;i<4;++i)put(r,panel+0x50+4*i,0x502c20u+i);
        after.windows.panel(&r,panel,(uint64_t(0xff0009)<<32)|(uint64_t(30000)<<16)|29000u,(uint64_t(0xff0009)<<32)|(uint64_t(29500)<<16)|28000u,0x40000000u);
        const auto p=after.windows.window(panel);check(p && p->background.size()==1 && p->border.empty());const auto &s=p->background[0];
        check(s.depth==0xff0009 && s.corners==std::array<std::array<int32_t,2>,4>{{{28000,29500},{29000,29500},{28000,30000},{29000,30000}}});
        check(s.colours==std::array<uint32_t,4>{0x40502c20u,0x40502c21u,0x40502c22u,0x40502c23u});
        rejects([&]{after.windows.panel(&r,panel,uint64_t(0x1000000)<<32,0,0);});        // depth outside 24 bits
        rejects([&]{after.windows.panel(&r,panel,0,0,0x40000001u);});                    // alpha with colour bits
        rejects([&]{after.windows.panel(&r,panel,uint64_t(5)<<32,uint64_t(6)<<32,0);});  // depths differ between registers
        put(r,panel,0x15aa20u);rejects([&]{after.windows.panel(&r,panel,0,0,0);});       // another class
        after.windows.release(panel);check(!after.windows.window(panel));imageCases+=5;}
    // Scissor element 15a420 (build 2ef5b0, site 2ef780): the compiled
    // SCISSOR value becomes the element's frame-pixel rectangle.
    for(unsigned seed=0;seed<32;++seed){
        auto &a=old.runtime(),&b=native.runtime();std::memset(a.memory.flat(),0,bozo::kFlatBytes);constexpr uint32_t clip=0x190800;
        put(a,clip,0x15a420u);put(a,clip+0x14,layer);put<uint8_t>(a,clip+0x50,seed&1);put<uint8_t>(a,clip+0x51,(seed>>1)&1);put(a,clip+0x48,0x200000u);
        put(a,clip+0x18,float(seed)*4.f-8.f);put(a,clip+0x1c,12.f);put(a,clip+0x20,200.f+float(seed)*20.f);put(a,clip+0x24,100.f);
        put(a,layer+0x40,10.f);put(a,layer+0x44,20.f);
        put(a,0x35e170,manager);put(a,manager+0x838,639u);put(a,manager+0x83c,447u);put(a,manager+0x840,16.f);put(a,manager+0x844,16.f);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);R5900Context x{},y{};
        x.r[4].ud[0]=clip;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;y=x;
        old.runtime().functions.generated(0x2ef5b0)(a.memory.flat(),&x,&a);native.runtime().functions.generated(0x2ef5b0)(b.memory.flat(),&y,&b);x.cycleLimit=y.cycleLimit;
        check(x.pc==0x170000 && y.pc==0x170000 && !std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        const auto captured=after.windows.window(clip);check(captured && captured->scissor && captured->background.empty() && captured->border.empty());
        const auto &s=*captured->scissor;
        if(seed&1)check(s==std::array<int32_t,4>{0,0,639,447});
        else{const float dx=seed&2?10.f:0.f,dy=seed&2?20.f:0.f;
            const int32_t x0=std::max(0,int32_t((float(seed)*4.f-8.f+dx)*1.f)),x1=std::min(639,int32_t((float(seed)*4.f-8.f+dx+200.f+float(seed)*20.f)*1.f));
            check(s==std::array<int32_t,4>{x0,int32_t(12.f+dy),x1,int32_t(112.f+dy)});}
        after.windows.release(clip);++imageCases;
    }
    {auto &r=native.runtime();put(r,0x190800,0x15a420u);
        rejects([&]{after.windows.scissor(&r,0x190800,0x0800000000000000ull);});put(r,0x190800,0x15aa20u);  // field beyond 11 bits
        rejects([&]{after.windows.scissor(&r,0x190800,0);});imageCases+=2;}                                 // another class
    // Camp menu backdrop 1672d0 (submit 45e570, site 45e594): one sprite over
    // the frame with the menu image published under (800, 24, palette +c4).
    {auto &r=native.runtime();std::memset(r.memory.flat(),0,bozo::kFlatBytes);constexpr uint32_t backdrop=0x190c00,scene=0x1a0000,context=0x1a8000;
        textures.publishMenuPage(0x800,0x24,0x3e00,{0x00494853,0x3d});
        const auto setUp=[&]{put(r,backdrop,0x1672d0u);put(r,backdrop+0x90,0x16734cu);put<uint8_t>(r,backdrop+0x64,1);put<uint8_t>(r,backdrop+0xc8,1);put<uint8_t>(r,backdrop+0x65,0);
            put(r,backdrop+0xc4,0x3e00u);put(r,0x35ddf8,scene);put(r,scene+0x40a8,backdrop);put<uint8_t>(r,scene+0x40dc,1);
            put(r,0x35de60,context);put<int16_t>(r,context+0x3c6c,640);put<int16_t>(r,context+0x3c6e,448);
            put(r,0x35e170,manager);put(r,manager+0x848,27648.f);put(r,manager+0x84c,29184.f);};
        setUp();
        // The 16x16 fixture picture is no 512x256 part: rejected.
        rejects([&]{vp2s::nativeMenuBackdrop(&r,textures,backdrop);});
        const auto reject=[&](const std::function<void()> &spoil){setUp();spoil();rejects([&]{vp2s::nativeMenuBackdrop(&r,textures,backdrop);});};
        reject([&]{put(r,backdrop,0x15a6f0u);});                 // another class
        reject([&]{put<uint8_t>(r,backdrop+0xc8,0);});           // not built
        reject([&]{put(r,scene+0x40a8,0u);});                    // not the scene's pending layer
        reject([&]{put(r,backdrop+0xc4,0x3e40u);});              // palette page nobody published
        textures.publishMenuPage(0x800,0x24,0x3e00,{0x00494853,0x49});setUp();
        rejects([&]{vp2s::nativeMenuBackdrop(&r,textures,backdrop);});                                  // no second half yet
        textures.publishMenuPage(0x1000,0x24,0,{0x00494853,0x4a});
        const auto unchanged=std::vector<uint8_t>(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);
        const auto layer=vp2s::nativeMenuBackdrop(&r,textures,backdrop);check(!std::memcmp(unchanged.data(),r.memory.flat(),unchanged.size()));
        check(layer.texts.size()==1 && layer.texts[0]->window && layer.texts[0]->window->border.size()==1);
        const auto &s=layer.texts[0]->window->border[0];
        check(s.fixedCorners==std::array<std::array<int32_t,2>,4>{{{27648,29184},{27648+10240,29184},{27648,29184+7168},{27648+10240,29184+7168}}});
        check(s.uv==std::array<float,4>{.5f/512,.5f/384,511.5f/512,383.5f/384} && s.colour==0x80808080u && s.depth==0);
        check(s.picture->width==512 && s.picture->height==384 && s.picture->rgba.back()==0x80405060u && s.picture->rgba[0]==0x80102030u && s.picture->rgba[512*256]==0x80405060u);
        check(vp2s::nativeMenuBackdrop(&r,textures,backdrop).texts[0]->window->border[0].picture==s.picture);   // composed once
        imageCases+=8;}
    // Untextured UI primitives (TME=0): vertex RGB as display bytes, the GS
    // blend kept; PCSX2 shows the camp panel 20,2c,50 at 0x38 over black as
    // 0f,13,22, not twice that.
    for(const auto blend:{bozo::render::Material::Blend::Alpha,bozo::render::Material::Blend::Additive}){
        const auto m=vp2s::nativeUiFlatMaterial(blend);
        check(!m.textured && m.directColour && m.blend==blend && m.alphaTest==(blend==bozo::render::Material::Blend::Alpha) && m.alphaRef==0);++imageCases;}
    std::printf("%u camp-menu image-list/panel CPU/RAM cases and rejections\n",imageCases);
    after.windows=vp2s::NativeUiWindows{};
    std::puts("64 complete window +64 selection-builder CPU/RAM cases: original scalar geometry/UV/colour, cached rebuild and immutable lifetime; invalid/incomplete/repeated captures reject without RAM writes");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

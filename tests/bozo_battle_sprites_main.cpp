// Battle 2D sprites (bozo/native_battle_sprites.cpp): a build, its texture selection, sprites from
// both ST paths of the sprite writer 42db50, and the flush's whole-writer and scalar checks.
// The texel case is the battle's first sprite at frame 63470 (writer eb1030): texels
// (24, 840, 408, 1208) give the ST words the game wrote, 3bbfffe0 3e51ffdd 3dcbffde 3e96ffe7.
#include "native_battle_sprites.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include <bit>
#include <cstdio>
#include <cstring>
#include <source_location>
#include <stdexcept>
namespace {
void check(bool b,std::source_location at=std::source_location::current()){
    if(!b)throw std::runtime_error("native battle sprite assertion failed at "+std::to_string(at.line()));
}
template<class F>void rejects(F f){bool failed=false;try{f();}catch(const std::runtime_error &){failed=true;}check(failed);}
template<class T>void put(Runtime &rt,uint32_t at,T v){std::memcpy(rt.memory.hostPointer(at,sizeof(T)),&v,sizeof(T));}
struct Host:bozo::ee::Host,bozo::port::Host {
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t)override{throw std::runtime_error("instruction");}
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("leaf");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device write");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    const auto st=vp2s::nativeBattleSpriteTexels({24,840,408,1208});
    const uint32_t words[4]={0x3bbfffe0,0x3e51ffdd,0x3dcbffde,0x3e96ffe7};
    for(unsigned i=0;i<4;++i)check(std::bit_cast<uint32_t>(st[i])==words[i]);
    Host host;bozo::ee::Dispatcher d(host);Runtime &rt=d.runtime();
    auto asset=std::make_shared<vp2s::NativeTextureAsset>();asset->key={0x6972756b,0x11c};asset->width=asset->height=2;asset->format=0x13;
    asset->texels={0,1,2,3};asset->palette[1]=0x80ffffff;
    constexpr uint32_t object=0x190000,writer=0x1a0000,other=0x1a0100,base=0x280000,B=base|0x30000000u,desc=0x1b0000,image=0x1b0100,stack=0x1fff000;
    constexpr uint32_t vt=0x174e30,otherVt=0x175000;put(rt,vt+0x18,0x3cd0c0u);put(rt,vt+0xa0,0x3cd330u);put(rt,otherVt+0x18,0x3cd0c0u);
    put(rt,object,vt);put(rt,object+0x90,other);put(rt,object+0x94,writer|0x30000000u);put(rt,object+0xbb,uint8_t(1));put(rt,object+0xbc,uint8_t(0));
    put(rt,writer,base|0x30000000u);put(rt,writer+8,B+0x40);put(rt,writer+12,B+0x40);put(rt,writer+16,0x210u);
    put(rt,desc+0x20,image);put(rt,desc+0x24,0x00808080u);put(rt,desc+0x40,uint8_t(0));
    for(unsigned i=0;i<4;++i)put(rt,image+2*i,uint16_t(std::array<uint16_t,4>{24,840,408,1208}[i]));
    for(unsigned i=0;i<4;++i)put(rt,desc+0x30+4*i,float(i+1)/8);
    const uint32_t w=writer|0x30000000u;
    vp2s::NativeBattleSprites sprites;
    rejects([&]{put(rt,object,otherVt);sprites.begin(&rt,object,w);});put(rt,object,vt);   // not the battle UI family
    rejects([&]{sprites.begin(&rt,object,other);});              // not the +bb writer
    sprites.begin(&rt,object,w);
    R5900Context c{};c.r[19].uw[0]=object;c.r[18].uw[0]=desc;c.r[16].uw[0]=w;c.r[3].uw[0]=image;
    c.r[6].uw[0]=0x8f5b;c.r[10].uw[0]=0x7a9a;c.r[5].uw[0]=0x90c3;c.r[9].uw[0]=0x7bfe;c.r[7].uw[0]=0xfff479;
    rejects([&]{sprites.sprite(&rt,&c);});                        // no texture selected yet
    rejects([&]{sprites.texture(other,{0x36a0,0x3d74,0});});
    rejects([&]{sprites.texture(w,{0x36a0,0x3d74,16});});
    rejects([&]{sprites.texture(w,{0x36a0,0x3d74,0,48});});     // a selection writes one or two TEX0_2
    sprites.texture(w,{0x36a0,0x3d74,0});
    sprites.sprite(&rt,&c);
    put(rt,desc+0x40,uint8_t(2));sprites.sprite(&rt,&c);         // descriptor ST floats
    put(rt,desc+0x40,uint8_t(0));put(rt,desc+0x20,0u);c.r[3].uw[0]=0;sprites.sprite(&rt,&c);  // no image record
    { auto bad=c;bad.r[6].sw[0]=-1;rejects([&]{sprites.sprite(&rt,&bad);});
      bad=c;bad.r[7].uw[0]=0x1000000;rejects([&]{sprites.sprite(&rt,&bad);});
      bad=c;bad.r[3].uw[0]=image;rejects([&]{sprites.sprite(&rt,&bad);}); }
    const auto *build=sprites.build(w);check(build && build->sprites.size()==3 && build->textures.size()==1 && build->spriteTextures==std::vector<uint32_t>{0,0,0});
    const auto &s=build->sprites[0];
    check(s.colour==0x00808080 && s.depth==0xfff479 && !s.picture);
    check(s.fixedCorners[0]==std::array<int32_t,2>{0x8f5b,0x7a9a} && s.fixedCorners[1]==std::array<int32_t,2>{0x90c3,0x7a9a} &&
          s.fixedCorners[2]==std::array<int32_t,2>{0x8f5b,0x7bfe} && s.fixedCorners[3]==std::array<int32_t,2>{0x90c3,0x7bfe});
    for(unsigned i=0;i<4;++i)check(std::bit_cast<uint32_t>(s.uv[i])==words[i] && build->sprites[1].uv[i]==float(i+1)/8 && build->sprites[2].uv[i]==float(i+1)/8);
    // The writer must hold exactly ALPHA (32), one TEX0 (32) and three sprites (3 x 64).
    const uint32_t end=B+0x40+32+32+3*64;
    // Pictures come from the image published under the selection's blocks at the flush.
    const vp2s::NativeBattleSprites::ImageLookup published=[&](uint32_t block,bool palette)->std::shared_ptr<const vp2s::NativeTextureAsset>{
        return block==(palette?0x3d74u:0x36a0u)?asset:nullptr;};
    const vp2s::NativeBattleSprites::ImageLookup unpublished=[](uint32_t,bool)->std::shared_ptr<const vp2s::NativeTextureAsset>{return nullptr;};
    put(rt,writer+8,end+16);rejects([&]{sprites.flush(&rt,stack,object,w,published);});
    put(rt,writer+8,end);rejects([&]{sprites.flush(&rt,stack,object+0x10,w,published);});
    rejects([&]{sprites.flush(&rt,stack,object,w,unpublished);});check(!sprites.active(stack));
    const auto drawn=sprites.flush(&rt,stack,object,w,published);check(sprites.active(stack) && sprites.outstanding()==1);
    check(drawn.sprites.size()==3);for(const auto &d:drawn.sprites)check(d.picture==asset->picture(0));
    rejects([&]{sprites.begin(&rt,object,w);});                   // rebuilt while flushing
    rejects([&]{sprites.complete(stack);});
    R5900Context f{};f.r[29].uw[0]=stack-0x30;f.r[17].uw[0]=w;f.r[9].uw[0]=object+0xc0;f.r[16].uw[0]=end+16;f.r[2].uw[0]=B+0x40;
    f.r[5].uw[0]=2;f.r[8].uw[0]=(end+16-(B+0x40))/16;
    { auto bad=f;bad.r[9].uw[0]=object+0x90;rejects([&]{sprites.retire(&rt,&bad);}); }
    sprites.retire(&rt,&f);
    rejects([&]{sprites.flushed(&rt,stack);});                    // the writer was not consumed
    put(rt,writer+8,end+16);put(rt,writer+12,end+16);sprites.flushed(&rt,stack);sprites.complete(stack);
    check(!sprites.outstanding() && sprites.build(w));             // kept: a writer may be flushed again
    std::puts("native battle sprites: build, texture, both ST paths, flush checks");
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

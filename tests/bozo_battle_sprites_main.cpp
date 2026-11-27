// Battle 2D sprites (bozo/native_battle_sprites.cpp): a build, its texture selection, sprites from
// both ST paths of the sprite writer 42db50, a flat polygon from 42df80, and the flush's whole-writer and scalar checks.
// The texel case is the battle's first sprite at frame 63470 (writer eb1030): texels
// (24, 840, 408, 1208) give the ST words the game wrote, 3bbfffe0 3e51ffdd 3dcbffde 3e96ffe7.
#include "native_battle_sprites.h"
#include "native_ui_text.h"
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
    // 120860 with 0x44 opens a build only on a reset writer (+8 = +c = base + 0x40), in EE RAM.
    check(!sprites.begin(&rt,0x70000100u));put(rt,other,base|0x30000000u);put(rt,other+8,B+0x80);put(rt,other+12,B+0x40);check(!sprites.begin(&rt,other));
    check(sprites.begin(&rt,w));
    R5900Context c{};c.r[19].uw[0]=object;c.r[18].uw[0]=desc;c.r[16].uw[0]=w;c.r[3].uw[0]=image;
    c.r[6].uw[0]=0x8f5b;c.r[10].uw[0]=0x7a9a;c.r[5].uw[0]=0x90c3;c.r[9].uw[0]=0x7bfe;c.r[7].uw[0]=0xfff479;
    put(rt,object,otherVt);rejects([&]{sprites.texture(w,{0x36a0,0x3d74,0});sprites.sprite(&rt,&c);});put(rt,object,vt);   // not the battle UI family
    { vp2s::NativeBattleSprites fresh;check(fresh.begin(&rt,w));auto bad=c;bad.r[16].uw[0]=other;rejects([&]{fresh.sprite(&rt,&bad);}); }   // not the +bb writer
    check(sprites.begin(&rt,w));
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
    // A flat polygon from 42df80 (42e144): header tag 50d440 + PRIM 244 at the writer's open end,
    // corners at sp+e0 (pixels), colour descriptor +40, depth +74, mode +78.
    constexpr uint32_t poly=0x1b0200,psp=0x1ffe000;
    put(rt,base+0x40,0x6400000000000001ull);put(rt,base+0x48,0x555510ull);put(rt,base+0x50,0x244ull);
    put(rt,poly+0x40,0x40102030u);put(rt,poly+0x74,0xfff000u);put(rt,poly+0x78,uint8_t(0));
    const int32_t corners[4][2]={{40,300},{200,300},{40,316},{200,316}};
    for(unsigned i=0;i<4;++i){put(rt,psp+0xe0+8*i,corners[i][0]);put(rt,psp+0xe4+8*i,corners[i][1]);}
    R5900Context p{};p.r[22].uw[0]=object;p.r[21].uw[0]=poly;p.r[20].uw[0]=4;p.r[16].uw[0]=0x300;p.r[19].uw[0]=B+0x40+0x18;p.r[29].uw[0]=psp;
    { auto bad=p;bad.r[20].uw[0]=3;rejects([&]{sprites.polygon(&rt,&bad);});          // only 4-corner strips
      bad=p;bad.r[19].uw[0]+=0x40;rejects([&]{sprites.polygon(&rt,&bad);});           // not at the open end
      put(rt,poly+0x78,uint8_t(4));bad=p;bad.r[16].uw[0]=4;rejects([&]{sprites.polygon(&rt,&bad);});   // Gouraud bit with the flat header
      put(rt,poly+0x78,uint8_t(8));bad=p;bad.r[16].uw[0]=8;rejects([&]{sprites.polygon(&rt,&bad);});put(rt,poly+0x78,uint8_t(0));  // textured bit with the flat header
      put(rt,psp+0xe0,0x1000);rejects([&]{sprites.polygon(&rt,&p);});put(rt,psp+0xe0,corners[0][0]); }
    sprites.polygon(&rt,&p);
    // The battle's Gouraud bar (object ea03d0, frame ~63470): header 24c, tag 50d470, colours 80101010 x4.
    put(rt,base+0x80,0xa400000000000001ull);put(rt,base+0x88,0xf515151510ull);put(rt,base+0x90,0x24cull);
    put(rt,writer+8,B+0x80);put(rt,poly+0x78,uint8_t(4));for(unsigned i=0;i<4;++i)put(rt,poly+0x40+4*i,0x80101010u+i);
    { auto g=p;g.r[16].uw[0]=4;g.r[19].uw[0]=B+0x80+0x18;sprites.polygon(&rt,&g); }
    // The battle's textured Gouraud panel (object e8bcd0, caller 389a38): header 25c, tag 50d480,
    // image record texels (5e8, 738, f68, b68).
    constexpr uint32_t rec=0x1b0300;const uint16_t texels[4]={0x5e8,0x738,0xf68,0xb68};for(unsigned i=0;i<4;++i)put(rt,rec+2*i,texels[i]);
    put(rt,base+0xc0,0xe400000000000001ull);put(rt,base+0xc8,0xf5215215215210ull);put(rt,base+0xd0,0x25cull);
    put(rt,writer+8,B+0xc0);put(rt,poly+0x78,uint8_t(0xc));put(rt,poly+0x70,rec);
    { auto t=p;t.r[16].uw[0]=0xc;t.r[19].uw[0]=B+0xc0+0x18;sprites.polygon(&rt,&t);
      put(rt,poly+0x78,uint8_t(0xe));t.r[16].uw[0]=0xe;for(unsigned i=0;i<4;++i){put(rt,poly+0x50+8*i,float(i&1));put(rt,poly+0x54+8*i,float(i)/4);}
      put(rt,poly+0x50,std::bit_cast<float>(0x7f800000u));rejects([&]{sprites.polygon(&rt,&t);});   // bit 2: descriptor ST, a nonfinite one refuses
    }
    // A textured flat rectangle from 42ed50 (42efe0): 12.4 edges in v1/t0/v0/a0, depth s3, mode +6c,
    // colour +30, descriptor ST floats +40 (bit 2), image record +68 = s2.
    constexpr uint32_t rect=0x1b0400;
    put(rt,base+0x100,0xa400000000000001ull);put(rt,base+0x108,0x5252525210ull);put(rt,base+0x110,0x254ull);
    put(rt,writer+8,B+0x100);put(rt,rect+0x6c,uint8_t(0xa));put(rt,rect+0x68,0u);put(rt,rect+0x30,0x80808080u);
    for(unsigned i=0;i<4;++i){put(rt,rect+0x40+8*i,float(i&1)*0.5f);put(rt,rect+0x44+8*i,float(i>>1)*0.25f);}
    R5900Context r{};r.r[23].uw[0]=object;r.r[21].uw[0]=rect;r.r[16].uw[0]=0xa;r.r[20].uw[0]=B+0x100+0x18;r.r[19].uw[0]=0xfff000;
    r.r[3].uw[0]=0x7400;r.r[8].uw[0]=0x7800;r.r[2].uw[0]=0x7c00;r.r[4].uw[0]=0x7a00;
    { auto bad=r;bad.r[18].uw[0]=rec;rejects([&]{sprites.rectangle(&rt,&bad);});      // s2 is not the descriptor's image
      bad=r;bad.r[3].sw[0]=-16;rejects([&]{sprites.rectangle(&rt,&bad);}); }
    sprites.rectangle(&rt,&r);
    put(rt,writer+8,B+0x40);put(rt,poly+0x40,0x40102030u);put(rt,poly+0x78,uint8_t(0));put(rt,poly+0x70,0u);
    const auto *build=sprites.build(w);constexpr auto U=vp2s::NativeBattleSpriteBuild::kUntextured;
    check(build && build->sprites.size()==7 && build->textures.size()==1 && build->polygons==4 && build->polygonBytes==64+96+128+96 &&
          build->spriteTextures==std::vector<uint32_t>{0,0,0,U,U,0,0});
    const auto &rs=build->sprites[6];check(!rs.gouraud && rs.colour==0x80808080 && rs.depth==0xfff000 && rs.uv==std::array<float,4>{0,0,0.5f,0.25f} &&
          rs.fixedCorners[0]==std::array<int32_t,2>{0x7400,0x7800} && rs.fixedCorners[3]==std::array<int32_t,2>{0x7c00,0x7a00});
    const auto &tp=build->sprites[5];const auto tst=vp2s::nativeBattleSpriteTexels({0x5e8,0x738,0xf68,0xb68});check(tp.gouraud && tp.uv==tst);
    const auto &q=build->sprites[3];check(q.colour==0x40102030 && q.depth==0xfff000 && !q.gouraud);
    const auto &g=build->sprites[4];check(g.gouraud);for(unsigned i=0;i<4;++i)check(g.cornerColours[i]==0x80101010u+i);
    check(vp2s::nativeUiSpriteVertices(g)[4].rgba==0x80101013u && vp2s::nativeUiSpriteVertices(q)[4].rgba==0x40102030u);
    for(unsigned i=0;i<4;++i)check(q.fixedCorners[i]==std::array<int32_t,2>{corners[i][0]*16,corners[i][1]*16});
    const auto &s=build->sprites[0];
    check(s.colour==0x00808080 && s.depth==0xfff479 && !s.picture);
    check(s.fixedCorners[0]==std::array<int32_t,2>{0x8f5b,0x7a9a} && s.fixedCorners[1]==std::array<int32_t,2>{0x90c3,0x7a9a} &&
          s.fixedCorners[2]==std::array<int32_t,2>{0x8f5b,0x7bfe} && s.fixedCorners[3]==std::array<int32_t,2>{0x90c3,0x7bfe});
    for(unsigned i=0;i<4;++i)check(std::bit_cast<uint32_t>(s.uv[i])==words[i] && build->sprites[1].uv[i]==float(i+1)/8 && build->sprites[2].uv[i]==float(i+1)/8);
    // The writer must hold exactly ALPHA (32), one TEX0 (32), three sprites (3 x 64) and the polygons (64 + 96 + 128).
    const uint32_t end=B+0x40+32+32+3*64+64+96+128+96;
    // Pictures come from the image published under the selection's blocks at the flush.
    const vp2s::NativeBattleSprites::ImageLookup published=[&](uint32_t pixels,uint32_t palette)->std::shared_ptr<const vp2s::NativeTextureAsset>{
        return pixels==0x36a0u && palette==0x3d74u?asset:nullptr;};
    const vp2s::NativeBattleSprites::ImageLookup unpublished=[](uint32_t,uint32_t)->std::shared_ptr<const vp2s::NativeTextureAsset>{return nullptr;};
    put(rt,writer+8,end+16);rejects([&]{sprites.flush(&rt,stack,object,w,published);});
    put(rt,writer+8,end);rejects([&]{sprites.flush(&rt,stack,object+0x10,w,published);});
    rejects([&]{sprites.flush(&rt,stack,object,w,unpublished);});check(!sprites.active(stack));
    const auto drawn=sprites.flush(&rt,stack,object,w,published);check(sprites.active(stack) && sprites.outstanding()==1);
    check(drawn.sprites.size()==7 && drawn.sprites[6].picture==asset->picture(0) && !drawn.sprites[3].picture && !drawn.sprites[4].picture && drawn.sprites[5].picture==asset->picture(0));for(unsigned i=0;i<3;++i)check(drawn.sprites[i].picture==asset->picture(0));
    rejects([&]{sprites.begin(&rt,w);});                          // rebuilt while flushing
    rejects([&]{sprites.complete(stack);});
    R5900Context f{};f.r[29].uw[0]=stack-0x30;f.r[17].uw[0]=w;f.r[9].uw[0]=object+0xc0;f.r[16].uw[0]=end+16;f.r[2].uw[0]=B+0x40;
    f.r[5].uw[0]=2;f.r[8].uw[0]=(end+16-(B+0x40))/16;
    { auto bad=f;bad.r[9].uw[0]=object+0x90;rejects([&]{sprites.retire(&rt,&bad);}); }
    sprites.retire(&rt,&f);
    rejects([&]{sprites.flushed(&rt,stack);});                    // the writer was not consumed
    put(rt,writer+8,end+16);put(rt,writer+12,end+16);sprites.flushed(&rt,stack);sprites.complete(stack);
    check(!sprites.outstanding() && sprites.build(w));             // kept: a writer may be flushed again
    // The band writer 383cb0 (the battle's writer ebc290, frame 63482): three Gouraud strips read
    // back at 3843a0 (s4 object, s3 writer, v0 end), tag from the object's +d0/+d8, Q bytes stale.
    { constexpr uint32_t object2=0x191000,writer2=0x1a1000,base2=0x290000,B2=base2|0x30000000u;const uint32_t w2=writer2|0x30000000u;
      put(rt,object2,vt);put(rt,object2+0x94,w2);put(rt,object2+0xbb,uint8_t(1));put(rt,object2+0xbc,uint8_t(0));
      put(rt,object2+0xd0,0xa400000000000001ull);put(rt,object2+0xd8,0xf515151510ull);
      put(rt,writer2,B2);put(rt,writer2+8,B2+0x40);put(rt,writer2+12,B2+0x40);put(rt,writer2+16,0x210u);
      vp2s::NativeBattleSprites band;check(band.begin(&rt,w2));
      const uint16_t xy[3][4][2]={{{0x8343,0x7e2c},{0x82c0,0x7dcc},{0x8371,0x7dba},{0x8319,0x7d7a}},
                                  {{0x8371,0x7dba},{0x8319,0x7d7a},{0x839e,0x7d49},{0x8373,0x7d29}},{{0x839e,0x7d49},{0x8373,0x7d29},{0x83b0,0x7d00},{0x8390,0x7cf0}}};
      for(unsigned k=0;k<3;++k){const uint32_t at=base2+0x40+96*k;
        put(rt,at,0xa400000000000001ull);put(rt,at+8,0xf515151510ull);put(rt,at+16,0x24cull);put(rt,at+88,uint64_t(0));
        for(unsigned i=0;i<4;++i){put(rt,at+24+16*i,uint64_t(0x2193ed3au+16*k+i)|uint64_t(0x0e332b5bu+i)<<32);
          put(rt,at+32+16*i,uint64_t(xy[k][i][0])|uint64_t(xy[k][i][1])<<16|0xfff4ddull<<32);}}
      R5900Context b{};b.r[20].uw[0]=object2;b.r[19].uw[0]=w2;b.r[2].uw[0]=B2+0x40+3*96;
      { auto bad=b;bad.r[2].uw[0]-=96;rejects([&]{band.band(&rt,&bad);});                                    // not three strips
        bad=b;bad.r[19].uw[0]=w;rejects([&]{band.band(&rt,&bad);});                                         // not the object's writer
        put(rt,base2+0x40+96+88,uint64_t(1));rejects([&]{band.band(&rt,&b);});put(rt,base2+0x40+96+88,uint64_t(0));   // the NOP
        put(rt,base2+0x40+192+32+4,0xfff4deu);rejects([&]{band.band(&rt,&b);});put(rt,base2+0x40+192+32+4,0xfff4ddu);   // depth
        put(rt,object2+0xd8,0x5252525210ull);rejects([&]{band.band(&rt,&b);});put(rt,object2+0xd8,0xf515151510ull); }   // another tag
      check(band.build(w2)->sprites.empty());
      band.band(&rt,&b);
      const auto *bb=band.build(w2);check(bb && bb->sprites.size()==3 && bb->polygons==3 && bb->polygonBytes==288);
      for(unsigned k=0;k<3;++k){const auto &s=bb->sprites[k];check(s.gouraud && s.depth==0xfff4dd && bb->spriteTextures[k]==vp2s::NativeBattleSpriteBuild::kUntextured);
        for(unsigned i=0;i<4;++i)check(s.cornerColours[i]==0x2193ed3au+16*k+i && s.fixedCorners[i]==std::array<int32_t,2>{xy[k][i][0],xy[k][i][1]});} }
    // A Gouraud triangle from the triangle writer 42fec0 through 42df80 (object e69600, frame
    // 63484): 3 corners at sp+e0, colours 50608080 / 00608080 x2, header 50d430 + PRIM 24b. Drawn
    // as a strip whose fourth corner repeats the third, 80 bytes; the flat form (50d420, PRIM
    // 243) is 64; textured triangles and the 4-corner header refuse.
    { constexpr uint32_t object3=0x192000,writer3=0x1a2000,base3=0x2a0000,B3=base3|0x30000000u,desc3=0x1b2000,sp3=0x1ffd000;const uint32_t w3=writer3|0x30000000u;
      put(rt,object3,vt);put(rt,object3+0x94,w3);put(rt,object3+0xbb,uint8_t(1));put(rt,object3+0xbc,uint8_t(0));
      put(rt,writer3,B3);put(rt,writer3+8,B3+0x40);put(rt,writer3+12,B3+0x40);put(rt,writer3+16,0x210u);
      vp2s::NativeBattleSprites tri;check(tri.begin(&rt,w3));
      put(rt,base3+0x40,0x8400000000000001ull);put(rt,base3+0x48,0xf5151510ull);put(rt,base3+0x50,0x24bull);
      put(rt,desc3+0x40,0x50608080u);put(rt,desc3+0x44,0x00608080u);put(rt,desc3+0x48,0x00608080u);put(rt,desc3+0x4c,0x42d85504u);
      put(rt,desc3+0x74,0xfff3e9u);put(rt,desc3+0x78,uint8_t(4));put(rt,desc3+0x70,0u);
      const int32_t tc[3][2]={{2312,1925},{2354,1932},{2353,1914}};
      for(unsigned i=0;i<3;++i){put(rt,sp3+0xe0+8*i,tc[i][0]);put(rt,sp3+0xe4+8*i,tc[i][1]);}put(rt,sp3+0xf8,0);put(rt,sp3+0xfc,0);
      R5900Context t{};t.r[22].uw[0]=object3;t.r[21].uw[0]=desc3;t.r[20].uw[0]=3;t.r[16].uw[0]=4;t.r[19].uw[0]=B3+0x40+0x18;t.r[29].uw[0]=sp3;
      { auto bad=t;put(rt,desc3+0x78,uint8_t(0));bad.r[16].uw[0]=0;rejects([&]{tri.polygon(&rt,&bad);});      // flat bits with the Gouraud header
        put(rt,desc3+0x78,uint8_t(0xc));bad.r[16].uw[0]=0xc;rejects([&]{tri.polygon(&rt,&bad);});put(rt,desc3+0x78,uint8_t(4));   // a textured one
        bad=t;bad.r[20].uw[0]=2;rejects([&]{tri.polygon(&rt,&bad);});                                           // 2 corners
        put(rt,base3+0x48,0xf515151510ull);put(rt,base3+0x40,0xa400000000000001ull);put(rt,base3+0x50,0x24cull);rejects([&]{tri.polygon(&rt,&t);});   // the 4-corner header
        put(rt,base3+0x40,0x8400000000000001ull);put(rt,base3+0x48,0xf5151510ull);put(rt,base3+0x50,0x24bull); }
      check(tri.build(w3)->sprites.empty());
      tri.polygon(&rt,&t);
      const auto *tb=tri.build(w3);check(tb && tb->sprites.size()==1 && tb->polygons==1 && tb->polygonBytes==80);
      const auto &ts=tb->sprites[0];check(ts.gouraud && ts.depth==0xfff3e9);
      for(unsigned i=0;i<4;++i){const unsigned k=i==3?2:i;check(ts.fixedCorners[i]==std::array<int32_t,2>{tc[k][0]*16,tc[k][1]*16} && ts.cornerColours[i]==(k?0x00608080u:0x50608080u));}
      put(rt,writer3+8,B3+0x40+80);put(rt,base3+0x40+80,0x6400000000000001ull);put(rt,base3+0x48+80,0xf55510ull);put(rt,base3+0x50+80,0x243ull);
      put(rt,desc3+0x78,uint8_t(0));{auto f=t;f.r[16].uw[0]=0;f.r[19].uw[0]=B3+0x40+80+0x18;tri.polygon(&rt,&f);}
      check(tb->sprites.size()==2 && !tb->sprites[1].gouraud && tb->sprites[1].colour==0x50608080u && tb->polygonBytes==80+64 && tb->sprites[1].fixedCorners[3]==tb->sprites[1].fixedCorners[2]); }
    // A textured polygon whose descriptor ST is not a rectangle (object f384f0, class 174ee0,
    // frame 71300): mode 8, no image record, corners (1914,1999) (1941,1999) (1914,2006) x2, ST
    // (.02734,.02734) (.21641,.02734) (.02734,.07617) x2. Each strip corner keeps its own ST;
    // a nonfinite one still refuses.
    { constexpr uint32_t object7=0x193000,writer7=0x1a3000,base7=0x2b0000,B7=base7|0x30000000u,desc7=0x1b3000,sp7=0x1ffc000;const uint32_t w7=writer7|0x30000000u;
      put(rt,object7,vt);put(rt,object7+0x94,w7);put(rt,object7+0xbb,uint8_t(1));put(rt,object7+0xbc,uint8_t(0));
      put(rt,writer7,B7);put(rt,writer7+8,B7+0x40);put(rt,writer7+12,B7+0x40);put(rt,writer7+16,0x210u);
      vp2s::NativeBattleSprites freeSt;check(freeSt.begin(&rt,w7));freeSt.texture(w7,{0x36a0,0x3d74,0});
      put(rt,base7+0x40,0xa400000000000001ull);put(rt,base7+0x48,0x5252525210ull);put(rt,base7+0x50,0x254ull);
      put(rt,desc7+0x40,0x80034243u);put(rt,desc7+0x74,0xfffa8cu);put(rt,desc7+0x78,uint8_t(8));put(rt,desc7+0x70,0u);
      const float fst[4][2]={{0.02734367921948433f,0.02734367921948433f},{0.2164056897163391f,0.02734367921948433f},{0.02734367921948433f,0.07617168128490448f},{0.02734367921948433f,0.07617168128490448f}};
      const int32_t fc[4][2]={{1914,1999},{1941,1999},{1914,2006},{1914,2006}};
      for(unsigned i=0;i<4;++i){put(rt,desc7+0x50+8*i,fst[i][0]);put(rt,desc7+0x54+8*i,fst[i][1]);put(rt,sp7+0xe0+8*i,fc[i][0]);put(rt,sp7+0xe4+8*i,fc[i][1]);}
      R5900Context t{};t.r[22].uw[0]=object7;t.r[21].uw[0]=desc7;t.r[20].uw[0]=4;t.r[16].uw[0]=8;t.r[19].uw[0]=B7+0x40+0x18;t.r[29].uw[0]=sp7;
      put(rt,desc7+0x5c,std::bit_cast<float>(0x7fc00000u));rejects([&]{freeSt.polygon(&rt,&t);});put(rt,desc7+0x5c,fst[1][1]);
      check(freeSt.build(w7)->sprites.empty());
      freeSt.polygon(&rt,&t);
      const auto *fb=freeSt.build(w7);check(fb && fb->sprites.size()==1 && fb->polygonBytes==96 && fb->spriteTextures==std::vector<uint32_t>{0});
      const auto &fs=fb->sprites[0];check(fs.freeUv && !fs.gouraud && fs.colour==0x80034243u && fs.depth==0xfffa8c);
      for(unsigned i=0;i<4;++i)check(fs.cornerUv[i]==std::array<float,2>{fst[i][0],fst[i][1]} && fs.fixedCorners[i]==std::array<int32_t,2>{fc[i][0]*16,fc[i][1]*16});
      const auto fv=vp2s::nativeUiSpriteVertices(fs);const unsigned order[6]={0,1,2,1,3,2};
      for(unsigned i=0;i<6;++i)check(fv[i].u==fst[order[i]][0] && fv[i].v==fst[order[i]][1]);
      // A rotated one (42e570, descriptor +3c != 0) is read back at 42df80's exit 42ebd8 (s3 =
      // the packet's end): the first of object f384f0's strips at frame 71320, written as
      // RGBAQ 3f800000 7f056b6c, then (ST, XYZ2) x4 with corners (77a0,7cf0) (7950,7d10)
      // (77a0,7d60) x2. An unrotated descriptor is left to 42e144; a different colour or a
      // packet end outside the mode's size refuses.
      const uint32_t at=B7+0x40+96;put(rt,writer7+8,at);
      put(rt,base7+0x40+96,0xa400000000000001ull);put(rt,base7+0x48+96,0x5252525210ull);put(rt,base7+0x50+96,0x254ull);
      const uint64_t packet[9]={0x3f8000007f056b6cull,0x3cdfffda3cdfffdaull,0xfffa8c7cf077a0ull,0x3cdfffda3e5d9974ull,0xfffa8c7d107950ull,
                                0x3d9bffe63cdfffdaull,0xfffa8c7d6077a0ull,0x3d9bffe63cdfffdaull,0xfffa8c7d6077a0ull};
      for(unsigned i=0;i<9;++i)put(rt,base7+0x58+96+8*i,packet[i]);
      put(rt,desc7+0x40,0x7f056b6cu);
      for(unsigned i=0;i<4;++i){put(rt,desc7+0x50+8*i,uint32_t(packet[1+2*i]&0xffffffffu));put(rt,desc7+0x54+8*i,uint32_t(packet[1+2*i]>>32));}
      R5900Context r=t;r.r[19].uw[0]=at+0x18+72;
      put(rt,desc7+0x3c,0x80000000u);freeSt.rotatedPolygon(&rt,&r);check(fb->sprites.size()==1);   // -0: unrotated
      put(rt,desc7+0x3c,0x00000001u);freeSt.rotatedPolygon(&rt,&r);check(fb->sprites.size()==1);   // a denormal compares as zero
      put(rt,desc7+0x3c,0.5f);
      { auto bad=r;bad.r[19].uw[0]+=8;rejects([&]{freeSt.rotatedPolygon(&rt,&bad);});
        put(rt,desc7+0x40,0x7f056b6du);rejects([&]{freeSt.rotatedPolygon(&rt,&r);});put(rt,desc7+0x40,0x7f056b6cu); }
      vp2s::NativeBattleSprites rotated;put(rt,writer7+8,B7+0x40);put(rt,writer7+12,B7+0x40);check(rotated.begin(&rt,w7));rotated.texture(w7,{0x36a0,0x3d74,0});put(rt,writer7+8,at);
      rotated.rotatedPolygon(&rt,&r);
      const auto *rb=rotated.build(w7);check(rb && rb->sprites.size()==1 && rb->polygonBytes==96);
      const auto &rs7=rb->sprites[0];const int32_t rc[4][2]={{0x77a0,0x7cf0},{0x7950,0x7d10},{0x77a0,0x7d60},{0x77a0,0x7d60}};
      check(rs7.freeUv && rs7.colour==0x7f056b6cu && rs7.depth==0xfffa8c);
      for(unsigned i=0;i<4;++i)check(rs7.fixedCorners[i]==std::array<int32_t,2>{rc[i][0],rc[i][1]} &&
          std::bit_cast<uint32_t>(rs7.cornerUv[i][0])==uint32_t(packet[1+2*i]&0xffffffffu) && std::bit_cast<uint32_t>(rs7.cornerUv[i][1])==uint32_t(packet[1+2*i]>>32)); }
    // A later ALPHA_2 (120860 with 0x148) on an open build at exactly its recorded content: the
    // sprites after it are additive and the write's 32 bytes are accounted; a cursor elsewhere is
    // not this build's content, and another value is unported.
    { constexpr uint32_t writer6=0x1a6000,base6=0x2d0000,B6=base6|0x30000000u;const uint32_t w6=writer6|0x30000000u;
      put(rt,writer6,B6);put(rt,writer6+8,B6+0x40);put(rt,writer6+12,B6+0x40);put(rt,writer6+16,0x198u);
      vp2s::NativeBattleSprites blended;check(blended.begin(&rt,w6));
      const auto quad=[&](uint32_t at){put(rt,at,0x6400000000000001ull);put(rt,at+8,0x555510ull);put(rt,at+16,0x244ull);put(rt,at+24,uint64_t(0x40ffffffu));
          for(unsigned i=0;i<4;++i)put(rt,at+32+8*i,uint64_t(0x6e00+0x200*(i&1))|uint64_t(0x7300+0x1a00*(i>>1))<<16|0x10ull<<32);};
      put(rt,writer6+8,B6+0x60);quad(base6+0x60);blended.flatQuad(&rt,w6,B6+0x60+64);put(rt,writer6+8,B6+0xa0);
      check(!blended.blend(&rt,w6+0x10,0x148));                    // not a build's writer
      put(rt,writer6+8,B6+0xb0);check(!blended.blend(&rt,w6,0x148));put(rt,writer6+8,B6+0xa0);   // cursor past the content
      rejects([&]{blended.blend(&rt,w6,0x99);});
      check(blended.blend(&rt,w6,0x148));put(rt,writer6+8,B6+0xc0);quad(base6+0xc0);blended.flatQuad(&rt,w6,B6+0xc0+64);
      const auto *bb=blended.build(w6);check(bb && bb->sprites.size()==2 && !bb->sprites[0].additive && bb->sprites[1].additive && bb->accountedBytes()==32+32+128); }
    // 430340's TEST_2 at the open end (battle 0x11's gauge): 0x5200d (AFAIL ZB_ONLY) marks the sprites after it as depth
    // masks, 0x5000d (KEEP) ends that; each write's 32 bytes are accounted; another value is unported, and a write
    // that is not at the build's open end is refused.
    { constexpr uint32_t writer8=0x1a8000,base8=0x2f0000,B8=base8|0x30000000u;const uint32_t w8=writer8|0x30000000u;
      put(rt,writer8,B8);put(rt,writer8+8,B8+0x40);put(rt,writer8+12,B8+0x40);put(rt,writer8+16,0x198u);
      vp2s::NativeBattleSprites masked;check(masked.begin(&rt,w8));
      const auto quad=[&](uint32_t at){put(rt,at,0x6400000000000001ull);put(rt,at+8,0x555510ull);put(rt,at+16,0x244ull);put(rt,at+24,uint64_t(0x40ffffffu));
          for(unsigned i=0;i<4;++i)put(rt,at+32+8*i,uint64_t(0x6e00+0x200*(i&1))|uint64_t(0x7300+0x1a00*(i>>1))<<16|0x10ull<<32);};
      put(rt,writer8+8,B8+0x70);rejects([&]{masked.test(&rt,w8,0x5200du);});   // cursor past the content
      put(rt,writer8+8,B8+0x40+32);rejects([&]{masked.test(&rt,w8,0x7000du);});
      masked.test(&rt,w8,0x5200du);put(rt,writer8+8,B8+0x80);quad(base8+0x80);masked.flatQuad(&rt,w8,B8+0x80+64);
      put(rt,writer8+8,B8+0xc0);masked.test(&rt,w8,0x5000du);put(rt,writer8+8,B8+0xe0);quad(base8+0xe0);masked.flatQuad(&rt,w8,B8+0xe0+64);
      const auto *mb=masked.build(w8);
      check(mb && mb->sprites.size()==2 && mb->sprites[0].depthMask && !mb->sprites[1].depthMask && mb->testWrites==2 && mb->accountedBytes()==32+64+128); }
    // A reset writer opened with 0x148 (the gauge bars of class 16f0a0): an additive build from the start.
    { constexpr uint32_t writer7=0x1a7000,base7=0x2e0000,B7=base7|0x30000000u;const uint32_t w7=writer7|0x30000000u;
      put(rt,writer7,B7);put(rt,writer7+8,B7+0x40);put(rt,writer7+12,B7+0x40);put(rt,writer7+16,0x198u);
      vp2s::NativeBattleSprites gauges;check(gauges.begin(&rt,w7,0x148));put(rt,writer7+8,B7+0x60);
      put(rt,base7+0x60,0x6400000000000001ull);put(rt,base7+0x68,0x555510ull);put(rt,base7+0x70,0x244ull);put(rt,base7+0x78,uint64_t(0x41799999u));
      for(unsigned i=0;i<4;++i)put(rt,base7+0x80+8*i,uint64_t(0x6e00+0x200*(i&1))|uint64_t(0x7300+0x1a00*(i>>1))<<16|0xffee6bull<<32);
      gauges.flatQuad(&rt,w7,B7+0x60+64);const auto *gb=gauges.build(w7);check(gb && gb->sprites.size()==1 && gb->sprites[0].additive && gb->accountedBytes()==32+64); }
    // main 120a80, the engine's flat rectangle writer (the battle gauge bars, writer e5e7a0): one
    // 64-byte strip read back at its cursor commit, only into an open battle build.
    { constexpr uint32_t writer5=0x1a5000,base5=0x2c0000,B5=base5|0x30000000u;const uint32_t w5=writer5|0x30000000u;
      put(rt,writer5,B5);put(rt,writer5+8,B5+0x40);put(rt,writer5+12,B5+0x40);put(rt,writer5+16,0x198u);
      vp2s::NativeBattleSprites flat;
      const auto packet=[&](uint32_t at,uint64_t prim){put(rt,at,0x6400000000000001ull);put(rt,at+8,0x555510ull);put(rt,at+16,prim);put(rt,at+24,uint64_t(0x00ffffffu)|0x752c8022ull<<32);
          const uint16_t xy[4][2]={{0x6e00,0x7300},{0x7000,0x7300},{0x6e00,0x8d00},{0x7000,0x8d00}};
          for(unsigned i=0;i<4;++i)put(rt,at+32+8*i,uint64_t(xy[i][0])|uint64_t(xy[i][1])<<16|0xffee6bull<<32);};
      packet(base5+0x40,0x244);
      flat.flatQuad(&rt,w5,B5+0x40+64);check(!flat.build(w5));   // no open build: not recorded
      check(flat.begin(&rt,w5));
      rejects([&]{flat.flatQuad(&rt,w5,B5+0x40+48);});            // not one strip
      packet(base5+0x40,0x204);rejects([&]{flat.flatQuad(&rt,w5,B5+0x40+64);});packet(base5+0x40,0x244);   // no alpha blending
      put(rt,base5+0x40+56+4,0xffee6cu);rejects([&]{flat.flatQuad(&rt,w5,B5+0x40+64);});put(rt,base5+0x40+56+4,0xffee6bu);   // a corner at another depth
      flat.flatQuad(&rt,w5,B5+0x40+64);
      const auto *fb=flat.build(w5);check(fb && fb->sprites.size()==1 && fb->polygons==1 && fb->polygonBytes==64 && !fb->sprites[0].gouraud && fb->sprites[0].colour==0x00ffffffu && fb->sprites[0].depth==0xffee6b &&
          fb->sprites[0].fixedCorners[3]==std::array<int32_t,2>{0x7000,0x8d00} && fb->spriteTextures[0]==vp2s::NativeBattleSpriteBuild::kUntextured); }
    // Corner shade 175600 (mod028 4f7a40 / 4f72d0 / 4f7a00): the static register block opens the
    // build; at full fraction (+d8 >= +dc) mode 1's offset is -296.985 and mode 2's reach 593.97, so
    // p(i) runs from trunc(-296.985) = -296 to trunc(296.985) = 296 (mode 1) and the alphas are the
    // game's 0x80 66 4c 39 2c 20 13 06 00 (seen in its packets at the game-over boundary).
    for(const uint8_t mode:{uint8_t(1),uint8_t(2)}){
        constexpr uint32_t shade=0x2d0000,writer8=0x1a8000,base8=0x2f0000,B8=base8|0x30000000u;const uint32_t w8=writer8|0x30000000u;
        put(rt,0x175600+0x18,0x42d4c0u);put(rt,shade,0x175600u);put(rt,shade+0x90,w8);put(rt,shade+0xbb,uint8_t(0));
        put(rt,writer8,B8);put(rt,writer8+8,B8+0x40);put(rt,writer8+12,B8+0x40);put(rt,writer8+16,0x200u);
        put(rt,shade+0xb0,0.0f);put(rt,shade+0xb4,0.0f);put(rt,shade+0x98,2048);put(rt,shade+0x9c,2048);put(rt,shade+0xd0,0x102030u);
        put(rt,shade+0xd8,1.0f);put(rt,shade+0xdc,1.0f);put(rt,shade+0xe0,mode);
        const uint64_t open[8]={0x1000000000000003ull,0xe,0x44,0x43,0x3000c,0x48,0x101000000ull,0x4f},close[6]={0x1000000000000002ull,0xe,0x5000d,0x48,0x1000000,0x4f};
        for(unsigned i=0;i<8;++i)put(rt,base8+0x40+8*i,i==2?0x48ull:open[i]);
        vp2s::NativeBattleSprites shades;rejects([&]{shades.cornerShadeOpen(&rt,shade,w8);});   // another blend in the block
        put(rt,base8+0x50,0x44ull);shades.cornerShadeOpen(&rt,shade,w8);put(rt,writer8+8,B8+0x80);
        put(rt,shade+0xe0,uint8_t(0));check(!shades.cornerShade(&rt,shade,w8));put(rt,shade+0xe0,mode);   // other modes draw nothing
        check(shades.cornerShade(&rt,shade,w8));put(rt,writer8+8,B8+0x300);
        for(unsigned i=0;i<6;++i)put(rt,base8+0x300+8*i,close[i]);
        shades.cornerShadeClose(&rt,shade,w8);
        const auto *sb=shades.build(w8);check(sb && sb->sprites.size()==9 && sb->polygons==9 && sb->accountedBytes()==0x2f0);
        const uint32_t alphas[9]={0x80,0x66,0x4c,0x39,0x2c,0x20,0x13,0x06,0};
        for(unsigned i=0;i<8;++i){const auto &q=sb->sprites[i];
            check(q.gouraud && q.cornerColours[0]==(0x102030u|alphas[i]<<24) && q.cornerColours[2]==(0x102030u|alphas[i+1]<<24));
            if(i)check(q.fixedCorners[0]==sb->sprites[i-1].fixedCorners[2] && q.fixedCorners[1]==sb->sprites[i-1].fixedCorners[3]);}   // one continuous band
        const auto &first=sb->sprites[0].fixedCorners,&last=sb->sprites[7].fixedCorners,&tri=sb->sprites[8].fixedCorners;
        if(mode==1){
            check(first[0]==std::array<int32_t,2>{(2048-296)<<4,(2048-0x128)<<4} && first[1]==std::array<int32_t,2>{(2048-0x128)<<4,(2048-296)<<4});
            check(last[2]==std::array<int32_t,2>{(2048+296)<<4,(2048-0x128)<<4});
            check(tri[0]==std::array<int32_t,2>{(2048-0x128)<<4,(2048-0x128)<<4} && tri[1]==std::array<int32_t,2>{(2048-296)<<4,(2048-0x128)<<4});
        }else{
            check(first[0]==std::array<int32_t,2>{(2048+296)<<4,(2048-296)<<4} && first[1]==std::array<int32_t,2>{(2048-296)<<4,(2048+296)<<4});
            check(last[2]==std::array<int32_t,2>{(2048+296)<<4,(2048-890)<<4});
            check(tri[0]==std::array<int32_t,2>{(2048+296)<<4,(2048+296)<<4} && tri[1]==std::array<int32_t,2>{(2048+296)<<4,(2048-296)<<4});
        }
        check(sb->sprites[8].cornerColours[0]==(0x102030u|0x80000000u));
        put(rt,writer8+8,B8+0x330);rejects([&]{shades.cornerShadeClose(&rt,shade,w8);});   // a second close: content it does not own
    }
    // VIF1-direct UI 174530 (42d3e0 header, 42d340 submit): the build opens one quadword in, after the
    // DIRECT code; the submit checks DIRECT = qwc - 2 and qwc = content + 2 quadwords.
    { constexpr uint32_t direct=0x2d8000,writer9=0x1a9000,base9=0x2f8000,B9=base9|0x30000000u;const uint32_t w9=writer9|0x30000000u;
      put(rt,direct,0x174530u);put(rt,direct+0x90,w9);put(rt,direct+0xbb,uint8_t(0));put(rt,direct+0xbc,uint8_t(0));
      put(rt,writer9,B9);put(rt,writer9+8,B9+0x40);put(rt,writer9+12,B9+0x40);put(rt,writer9+16,0x200u);
      vp2s::NativeBattleSprites ui;
      put(rt,writer9+8,B9+0x50);check(!ui.begin(&rt,w9));                       // one quadword in, without the header: not a build
      put(rt,writer9+8,B9+0x40);ui.directHeader(&rt,direct,w9);put(rt,writer9+8,B9+0x50);check(ui.begin(&rt,w9));
      put(rt,writer9+8,B9+0x70);const uint32_t at=base9+0x70;
      put(rt,at,0x6400000000000001ull);put(rt,at+8,0x555510ull);put(rt,at+16,0x244ull);put(rt,at+24,uint64_t(0x80000000u));
      for(unsigned i=0;i<4;++i)put(rt,at+32+8*i,uint64_t(0x7300+0x1a00*(i&1))|uint64_t(0x6e00+0x200*(i>>1))<<16|0xfffa23ull<<32);
      ui.flatQuad(&rt,w9,B9+0x70+64);put(rt,writer9+8,B9+0xb0);
      const uint32_t qwc=(0xb0-0x40)/16+2;put(rt,direct+0xc4,qwc);put(rt,direct+0xcc,base9+0x40);put(rt,base9+0x40,0ull);put(rt,base9+0x48,0u);
      put(rt,base9+0x4c,0x51000000u|(qwc-1));rejects([&]{ui.submitDirect(&rt,direct,[](uint32_t,uint32_t){return nullptr;});});
      put(rt,base9+0x4c,0x51000000u|(qwc-2));
      const auto drawn=ui.submitDirect(&rt,direct,[](uint32_t,uint32_t){return nullptr;});
      check(drawn.sprites.size()==1 && drawn.sprites[0].colour==0x80000000u && drawn.accountedBytes()==0x70);
      put(rt,direct+0xc4,qwc+1);rejects([&]{ui.submitDirect(&rt,direct,[](uint32_t,uint32_t){return nullptr;});}); }
    // Screen lens 16e9e0 (mod028 3764e0 / 375db0 / 375c80): read back at the producers' commits under exact checks.
    // The display record [[[36d880] + 80] + 8]: page +670 = 0x75 (TEX0_2 668128ea0), ZBUF_2 +170; a field (f = 0.5).
    { constexpr uint32_t lens=0x2da000,writerL=0x1aa000,baseL=0x300000,BL=baseL|0x30000000u,rec=0x2dc000;const uint32_t wl=writerL|0x30000000u;
      put(rt,0x36d880,0x2db000u);put(rt,0x2db000u+0x80,0x2db100u);put(rt,0x2db100u+8,rec);put(rt,rec+0x670,uint16_t(0x75));put(rt,rec+0x170,0x1000000u);
      put(rt,0x1e5984,uint8_t(0));
      put(rt,0x16e9e0+0x18,0x42d4c0u);put(rt,lens,0x16e9e0u);put(rt,lens+0x90,wl);put(rt,lens+0xbb,uint8_t(0));put(rt,lens+0x17c,1);
      put(rt,writerL,BL);put(rt,writerL+8,BL+0x40);put(rt,writerL+12,BL+0x40);put(rt,writerL+16,0x400u);
      vp2s::NativeBattleSprites lenses;check(lenses.begin(&rt,wl,0x148));put(rt,writerL+8,BL+0x60);check(lenses.blend(&rt,wl,0x44));
      const uint64_t tex0=0x668128ea0ull;
      const auto ad=[&](uint32_t at,std::initializer_list<std::pair<uint64_t,uint64_t>> regs){put(rt,at,0x1000000000008000ull|regs.size());put(rt,at+8,0xeull);
          unsigned i=0;for(const auto &[d,r]:regs){put(rt,at+16+16*i,d);put(rt,at+24+16*i,r);++i;}};
      const uint32_t fan=baseL+0x60;
      ad(fan,{{tex0,7},{0x15,0x15},{0x35d,0},{1,0x46},{0x1000000,0x4f},{0x70000,0x48},{0x60,0x3b}});
      put(rt,fan+0x80,0x3400000000008022ull);put(rt,fan+0x88,0x531ull);
      for(unsigned v=0;v<34;++v){const auto q=fan+0x90+24*v;put(rt,q,uint64_t(v?0x40808080u:0x80808080u));
          put(rt,q+8,uint64_t(16*320+v)|uint64_t(16*112)<<16);put(rt,q+16,uint64_t(0x8000+16*v)|uint64_t(0x8000)<<16|0xffec7cull<<32);}
      const uint32_t fanEnd=BL+0x60+0x3c0;
      put(rt,fan+0x80,0x3400000000008021ull);rejects([&]{lenses.lensFans(&rt,lens,wl,fanEnd);});put(rt,fan+0x80,0x3400000000008022ull);   // another vertex count
      lenses.lensFans(&rt,lens,wl,fanEnd);put(rt,writerL+8,fanEnd);
      const auto *lb=lenses.build(wl);check(lb && lb->sprites.size()==32 && lb->spriteTextures[0]==vp2s::NativeBattleSpriteBuild::kPreviousFrame);
      check(lb->sprites[0].freeUv && lb->sprites[0].cornerUv[0][0]==0.5f && lb->sprites[0].cornerUv[0][1]==0.5f);   // (320, 112) of 640 x 224
      check(lb->sprites[0].cornerColours[0]==0x60808080u && lb->sprites[0].cornerColours[1]==0x30808080u);           // alpha * 0x60 / 0x80
      const uint32_t strip=fanEnd&0x1ffffff;
      ad(strip,{{1,0x15},{0x354,0},{0x01808080,1},{1,0x46},{0x1000000,0x4f},{0x70000,0x48},{0x60,0x3b}});
      put(rt,strip+0x80,0xa400000000008001ull);put(rt,strip+0x88,0xf535353537ull);put(rt,strip+0x90,tex0);
      for(unsigned j=0;j<4;++j){put(rt,strip+0x98+16*j,uint64_t(0x2800*(j&1))|uint64_t(0xe00*(j>>1))<<16);put(rt,strip+0xa0+16*j,uint64_t(0x6e00+0x2400*(j&1))|uint64_t(0x7300+0x1a00*(j>>1))<<16|0xffec7cull<<32);}
      const uint32_t stripEnd=fanEnd+0xe0;
      lenses.lensStrips(&rt,lens,wl,stripEnd);put(rt,writerL+8,stripEnd);
      check(lb->sprites.size()==33 && lb->sprites[32].cornerUv[3][0]==1.f && lb->sprites[32].cornerUv[3][1]==1.f);
      const uint32_t restore=stripEnd&0x1ffffff;
      ad(restore,{{1,0x3f},{4,0x1c},{0x61,0x15},{0x44,0x43},{0x5000d,0x48},{0x8000008080ull,0x3b},{0x1000000,0x4f}});
      lenses.lensRestore(&rt,wl,stripEnd+0x80);
      check(lb->accountedBytes()==0x20+0x20+0x3c0+0xe0+0x80); }   // two ALPHA_2 writes, fans, strips, restore
    // Screen shockwave 1746d0 (builder 4c28d0's shape): setup CNT 9, a fan of 4 (UV, XYZ2), one ring strip of 5 (RGBAQ, UV,
    // XYZ2), the END restore; the display record (as the lens's) gives TEX0_2, [35de60] + 170 the ZBUF_2.
    { constexpr uint32_t object=0x2e0000,writer=0x2e1000,chain=0x2e2000,ctx=0x2e3000;
      put(rt,0x35de60,ctx);put(rt,ctx+0x170,0x1000000u);
      put(rt,object,0x1746d0u);put(rt,object+0xc0,0x174754u);put(rt,object+0xbc,uint8_t(0));put(rt,object+0x90,writer);
      uint32_t at=chain;const auto q2=[&](uint64_t a,uint64_t b){put(rt,at,a);put(rt,at+8,b);at+=16;};
      q2(0x10000009,0);q2(0x11aac00000008008ull,0xe);
      for(const auto &[d,r]:std::initializer_list<std::pair<uint64_t,uint64_t>>{{0x668128ea0ull,7},{0x61,0x15},{0x30000,0x48},{0x8000000080ull,0x3b},{0x44,0x43},{0x1000000,0x4f},{0x40808080,1},{0,0x3f}})q2(d,r);
      const auto xyz=[](uint32_t x,uint32_t y,uint32_t z){return uint64_t(x)|uint64_t(y)<<16|uint64_t(z)<<32;};
      q2(0x10000005,0);q2(0x2400000000000004ull,0x53);
      const uint32_t fu[4]={0x740,0x87c,0x869,0x832},fv[4]={0x5a0,0x5a0,0x5e1,0x61b};
      for(unsigned v=0;v<4;++v)q2(fu[v]|uint64_t(fv[v])<<16,xyz(0x7486+16*v,0x7d72,0xffec8b));
      q2(0x1000000b,0);q2(0x11ae400000000001ull,0xe);q2(0x70000,0x48);q2(0x3400000000008005ull,0x531);
      uint64_t regs[16]{};for(unsigned v=0;v<5;++v){regs[3*v]=0x3f80000000808080ull|uint64_t(0x10*v)<<24;regs[3*v+1]=uint64_t(0x700+16*v)|uint64_t(0x500)<<16;regs[3*v+2]=xyz(0x7400+32*v,0x7d00+16*(v&1),0xffec7c);}
      for(unsigned k=0;k<8;++k)q2(regs[2*k],regs[2*k+1]);
      q2(0x70000008,0);q2(0x1000000000008007ull,0xe);
      for(const auto &[d,r]:std::initializer_list<std::pair<uint64_t,uint64_t>>{{0x61,0x15},{0,0x3f},{0x5000d,0x48},{0x8000008080ull,0x3b},{0x44,0x43},{0x1000000,0x4f},{0x19f023f0000ull,0x41}})q2(d,r);
      put(rt,writer+12,chain);put(rt,writer+8,at);
      const auto w=vp2s::nativeShockwave(&rt,object);
      check(w.linear && w.fanDepthWrite && w.stripDepthWrite && w.fan.size()==2 && w.strips.size()==2);
      const auto &f=w.fan[0];check(f.colour==0x40808080u && f.depth==0xffec8b && f.freeUv && f.fixedCorners[0]==std::array<int32_t,2>{0x7486,0x7d72});
      check(f.cornerUv[0][0]==float(0x740)/16.f/640.f && f.cornerUv[0][1]==float(0x5a0)/16.f/224.f && f.fixedCorners[3]==f.fixedCorners[2]);
      const auto &s0=w.strips[0],&s1=w.strips[1];check(s0.gouraud && s0.depth==0xffec7c && s0.cornerColours[3]==0x30808080u && s1.cornerColours[0]==0x20808080u);
      check(s1.fixedCorners[2]==std::array<int32_t,2>{0x7480,0x7d00} && s1.fixedCorners[3]==s1.fixedCorners[2] && s0.cornerUv[1][0]==float(0x710)/16.f/640.f);
      // Refusals: another texture, another blend, a strip tag, a chain longer than its blocks, another class.
      put(rt,chain+0x20,0x668128ec0ull);rejects([&]{vp2s::nativeShockwave(&rt,object);});put(rt,chain+0x20,0x668128ea0ull);
      put(rt,chain+0x60,uint64_t(0x48));rejects([&]{vp2s::nativeShockwave(&rt,object);});put(rt,chain+0x60,uint64_t(0x44));
      put(rt,chain+0x130,0x3400000000008004ull);rejects([&]{vp2s::nativeShockwave(&rt,object);});put(rt,chain+0x130,0x3400000000008005ull);
      put(rt,writer+8,at+16);rejects([&]{vp2s::nativeShockwave(&rt,object);});put(rt,writer+8,at);
      put(rt,object,0x174530u);rejects([&]{vp2s::nativeShockwave(&rt,object);});put(rt,object,0x1746d0u);
      check(vp2s::nativeShockwave(&rt,object).strips.size()==2); }
    std::puts("native battle sprites: build, texture, both ST paths, flat, Gouraud and textured polygons, flat and Gouraud triangles, engine flat rectangles, bands, rectangles, corner shades, VIF1-direct UI, screen lenses, flush checks");
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

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
      rejects([&]{sprites.polygon(&rt,&t);});                                       // bit 2: descriptor ST, here not an axis-aligned rectangle
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
    std::puts("native battle sprites: build, texture, both ST paths, flat, Gouraud and textured polygons, flat and Gouraud triangles, engine flat rectangles, bands, rectangles, flush checks");
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

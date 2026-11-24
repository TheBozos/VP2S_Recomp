#include <algorithm>
// Scene scissor element (1597e0) and the Lost Forest haze object (15a020) read
// from synthetic records in EE RAM, with a native frame target. No game data.
#include "native_haze.h"
#include "native_scene_clip.h"
#include "native_map_panel.h"
#include "bozo/port.h"
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <source_location>
#include <stdexcept>
#include <string>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("scene effect assertion at "+std::to_string(at.line()));}
template<class T>void put(Runtime &rt,uint32_t at,const T &v){std::memcpy(rt.memory.hostPointer(at,sizeof(T)),&v,sizeof(T));}
constexpr uint32_t current=0x400000,target=0x500000,object=0x600000,depthOwner=0x700000;
void frame(Runtime &rt,vp2s::NativeScene &scene,int16_t width,int16_t height){
    std::memset(rt.memory.flat(),0,bozo::kFlatBytes);scene=vp2s::NativeScene{};
    put(rt,0x1c8d00+0xcc84,uint8_t(1));put(rt,0x35ddf8,current);put(rt,current+0x40a8,object);put(rt,current+0x40dc,uint8_t(1));
    put(rt,target+0x3c76,uint8_t(1));put(rt,target+0x3c60,1.f);put(rt,target+0x3c64,1.f);put(rt,target+0x3c68,width);put(rt,target+0x3c6a,height);
    for(unsigned i=0;i<2;++i){const auto base=target+0xa70*i;put(rt,base+0x1cf0,uint8_t(1));put(rt,base+0x1cf2,uint8_t(i));put(rt,base+0x1cf5,uint8_t(1));
        put(rt,base+0x1cfa,width);put(rt,base+0x1cfc,int16_t(height/2));}
    scene.beginComposition(&rt,0x1f8000,target);
}
unsigned rejections(Runtime &rt,const std::function<void()> &f,const std::function<void(unsigned)> &mutate,unsigned count){
    unsigned rejected=0;
    for(unsigned which=0;which<count;++which){std::vector<uint8_t> saved(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);mutate(which);
        try{f();}catch(const std::runtime_error &){++rejected;}
        std::memcpy(rt.memory.flat(),saved.data(),saved.size());}
    return rejected;
}
}
int main(){try{
    Runtime rt;vp2s::NativeScene scene;
    // Scene scissor element.
    frame(rt,scene,512,512);check(scene.frameTarget() && scene.frameTarget()->width==512);
    put(rt,object,uint32_t(0x1597e0));put(rt,object+0x90,uint32_t(0x15985c));put(rt,object+0x63,uint8_t(1));
    put(rt,object+0xa4,int32_t(24));put(rt,object+0xa8,int32_t(24));put(rt,object+0xac,int32_t(179));put(rt,object+0xb0,int32_t(139));
    const auto clip=vp2s::nativeSceneClip(&rt,scene,object);
    check(clip.object==object && clip.rect==std::array<int32_t,4>{24,24,179,139});
    const auto clipRejected=rejections(rt,[&]{vp2s::nativeSceneClip(&rt,scene,object);},[&](unsigned w){
        switch(w){case 0:put(rt,object,uint32_t(0x15a020));break;case 1:put(rt,object+0x94,uint32_t(0x123450));break;case 2:put(rt,object+0x63,uint8_t(0));break;
        case 3:put(rt,object+0xac,int32_t(512));break;case 4:put(rt,object+0xac,int32_t(10));break;case 5:put(rt,current+0x40a8,uint32_t(0));break;
        default:put(rt,current+0x40dc,uint8_t(0));break;}},7);
    check(clipRejected==7);
    // Haze: a 128x256 frame (2 x 2 tiles), constants, ramp and tile records.
    frame(rt,scene,128,256);
    put(rt,object,uint32_t(0x15a020));put(rt,object+0x90,uint32_t(0x15a09c));put(rt,object+0xa8,int32_t(128));put(rt,object+0xac,int32_t(256));
    put(rt,object+0x94,depthOwner);put(rt,depthOwner+0x3c74,int16_t(0x31));put(rt,object+0x18951,uint8_t(5));
    for(unsigned i=0;i<256;++i)put(rt,object+0x134c4+4*i,uint32_t(i)|uint32_t(255-i)<<8|uint32_t(i/2)<<24); // alpha <= 0x7f
    const std::array<std::array<float,4>,25> constants=[]{std::array<std::array<float,4>,25> c{};
        c[1]={0.3f,-0.8f,-0.52f,-1};c[2]={-50,-20,-80,1};c[3]={180,120,60,0};c[4]={0.0008f,0.0013f,0.0026f,0};c[5]={0.03f,0.009f,0.005f,1.5f};
        c[7]={0.0597f,0.08f,1.04f,0.4f};c[9]={0.6f,-0.4f,-1.5f,0};c[10]={0.001f,0.002f,0.0005f,0};c[11][2]=std::bit_cast<float>(1u);return c;}();   // 11.z: the elevation mask
    for(unsigned q=0;q<25;++q)put(rt,object+0x18b30+16*q,constants[q]);
    put(rt,object+0x18940,uint8_t(2));
    for(unsigned row=0;row<2;++row){put(rt,object+0x18930+2*row,int16_t(128));put(rt,object+0x18938+2*row,int16_t(128*row));
        for(unsigned column=0;column<2;++column){const uint32_t record=object+0x13940+row*0x1400+column*0x200;
            put(rt,record,uint32_t(2));put(rt,record+4,uint32_t(5));put(rt,record+16,std::array<float,4>{1983.5f+64*column,1919.5f+128*row,0,0});
            put(rt,record+32,std::array<float,4>{32,32,0,0});
            put(rt,record+80,std::array<float,4>{100.f+40*column,-30.f+80*row,200,1});put(rt,record+96,std::array<float,4>{20,0,-5,0});put(rt,record+112,std::array<float,4>{0,-20,3,0});}}
    const auto haze=vp2s::nativeHaze(&rt,scene,object);
    check(haze.object==object && haze.vertices.size()==2*2*8*6);
    // The table: entry k is the ramp at hazeRampIndex(k << 14), R/255 and alpha/128.
    for(unsigned k=0;k<64;++k){const unsigned index=vp2s::hazeRampIndex(k<<14);
        check(haze.colours[4*k]==float(index)/255.f && haze.colours[4*k+1]==float(255-index)/255.f && haze.colours[4*k+3]==float(index/2)/128.f);}
    // The first vertex: frame origin (2048 - 64, 2048 - 128), colour of the tile's base point.
    check(haze.vertices[0].x==1984 && haze.vertices[0].y==1920 && haze.vertices[0].rgba==vp2s::hazeVertexColour(constants,{100,-30,200,1}));
    // The last tile's far corner point: base + 2 across + 4 down.
    bool farCorner=false;
    for(const auto &v:haze.vertices)if(v.x==1984+128 && v.y==1920+256)farCorner=v.rgba==vp2s::hazeVertexColour(constants,{140+40,50-80,200-10+12,1});
    check(farCorner);
    const auto hazeRejected=rejections(rt,[&]{vp2s::nativeHaze(&rt,scene,object);},[&](unsigned w){
        switch(w){case 0:put(rt,object+0x90,uint32_t(0));break;case 1:put(rt,depthOwner+0x3c74,int16_t(0x30));break;case 2:put(rt,object+0x18951,uint8_t(4));break;
        case 3:put(rt,object+0x134c4+40,uint32_t(0x81000000));break;case 4:put(rt,object+0x18b30+16*8,uint32_t(1));break;
        case 5:put(rt,object+0x18b30+16*11+4,uint32_t(1));break;case 6:put(rt,object+0x18940,uint8_t(1));break;
        case 7:put(rt,object+0x13940+0x200+16,1990.f);break;case 8:put(rt,object+0x13940,uint32_t(3));break;
        case 9:put(rt,object+0xa8,int32_t(192));break;case 10:put(rt,object+0x18930,int16_t(64));break;
        default:put(rt,object+0x13940+80,std::bit_cast<float>(0x7fc00000u));break;}},12);
    check(hazeRejected==12);
    // A 96-row tile (the 416-high battle-entry frame: 128, 96, 96, 96): 128 x 224 here, its
    // records with 3 cells of 32 down (+4 = 4).
    const auto reframe=[&](int16_t width,int16_t height){std::vector<uint8_t> kept(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);frame(rt,scene,width,height);
        for(const auto &[at,size]:{std::pair{object,0x19000u},std::pair{depthOwner,0x4000u}})std::memcpy(rt.memory.flat()+at,kept.data()+at,size);};
    reframe(128,224);put(rt,object+0xac,int32_t(224));for(unsigned column=0;column<2;++column)put(rt,object+0x13940+0x1400+column*0x200+4,uint32_t(4));put(rt,object+0x18930+2,int16_t(96));
    const auto shortRow=vp2s::nativeHaze(&rt,scene,object);
    check(shortRow.vertices.size()==2*2*4*6+2*2*3*6);
    float lowest=0;for(const auto &v:shortRow.vertices)lowest=std::max(lowest,v.y);check(lowest==1936+224);
    put(rt,object+0x18930+2,int16_t(80));bool ragged=false;   // a grid that does not fill the tile
   try{vp2s::nativeHaze(&rt,scene,object);}catch(const std::runtime_error &){ragged=true;}check(ragged);   // not whole cells
    reframe(128,256);put(rt,object+0xac,int32_t(256));put(rt,object+0x18930+2,int16_t(128));for(unsigned column=0;column<2;++column)put(rt,object+0x13940+0x1400+column*0x200+4,uint32_t(5));
    // Field map panel: the PCSX2 forest frame's prims (opacity 89.5: panel alpha
    // 0x59, border 0x43; rectangle 1816..1972 x 1816..1932, border 2 px outside).
    std::memset(rt.memory.flat(),0,bozo::kFlatBytes);
    put(rt,object,uint32_t(0x167200));put(rt,object+0x90,uint32_t(0x16727c));put(rt,object+0x63,uint8_t(1));put(rt,object+0xb0,89.5f);
    put(rt,object+0x94,int32_t(1792));put(rt,object+0x98,int32_t(1792));put(rt,object+0xa4,depthOwner);
    const auto panel=vp2s::nativeMapPanel(&rt,object);check(panel.vertices.size()==30);
    const auto at=[&](size_t i,float x,float y,uint32_t rgba){const auto &v=panel.vertices[i];return v.x==x && v.y==y && v.rgba==rgba;};
    check(at(0,1816,1816,0x594d81b7u) && at(1,1972,1816,0x594d81b7u) && at(2,1816,1932,0x5926547au) && at(4,1972,1932,0x5926547au));
    check(at(6,1814,1814,0x43101028u) && at(7,1816,1816,0x43101028u) && at(8,1974,1814,0x43101028u) && at(28,1972,1932,0x43101028u));
    // Grown (+1683 bit 2, t = 0.5): 24-12 = 12, 180+166 = 346, 140+186.98 -> 326.
    put(rt,depthOwner+0x1683,uint8_t(4));put(rt,object+0xb4,0.5f);
    const auto grown=vp2s::nativeMapPanel(&rt,object);
    check(grown.vertices[0].x==1792+12 && grown.vertices[0].y==1792+12 && grown.vertices[1].x==1792+346 && grown.vertices[2].y==1792+326);
    put(rt,depthOwner+0x1683,uint8_t(0));
    put(rt,object+0xb0,0.f);put(rt,object+0x63,uint8_t(0));check(vp2s::nativeMapPanel(&rt,object).vertices.empty());
    put(rt,object+0xb0,89.5f);put(rt,object+0x63,uint8_t(1));
    const auto panelRejected=rejections(rt,[&]{vp2s::nativeMapPanel(&rt,object);},[&](unsigned w){
        switch(w){case 0:put(rt,object,uint32_t(0x167210));break;case 1:put(rt,object+0x63,uint8_t(0));break;case 2:put(rt,object+0xb0,200.f);break;
        case 3:put(rt,object+0xb0,-1.f);break;case 4:put(rt,object+0x94,int32_t(-5));break;
        default:put(rt,depthOwner+0x1683,uint8_t(4));put(rt,object+0xb4,std::bit_cast<float>(0x7fc00000u));break;}},6);
    check(panelRejected==6);
    std::printf("scene clip: rect and %u rejections; haze: %zu vertices, 64 entries and %u rejections; map panel: 30 vertices, growth, %u rejections\n",
        clipRejected,haze.vertices.size(),hazeRejected,panelRejected);
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

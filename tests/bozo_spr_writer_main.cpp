#include "native_spr_writer.h"
#include "native_models.h"
#include <array>
#include <cstdio>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <vector>
namespace {
void check(bool ok,const char *what){if(!ok)throw std::runtime_error(what);}
template<class T>void put(Runtime &rt,uint32_t at,T v){std::memcpy(rt.memory.hostPointer(at,sizeof(T)),&v,sizeof(T));}
bool rejects(const std::function<void()> &f){try{f();}catch(const std::runtime_error &){return true;}return false;}
uint64_t cnt(uint32_t address,uint32_t qwords){return uint64_t(address)<<32|0x10000000u|qwords;}
constexpr uint64_t kEnd=0x70000000u;
// A chunk as 200e60 lays it out in a scratchpad half: header (CNT 4), the texture
// block either as CNT 16 or as 17 zero qwords, lanes at fixed offsets, END.
struct Chunk{std::vector<std::pair<uint32_t,uint32_t>> sections;uint32_t start,end;};
Chunk build(Runtime &rt,uint32_t spr,uint32_t start,bool texture,unsigned count){
    std::memset(rt.memory.hostPointer(spr,0xc00),0x5a,0xc00);
    Chunk out{{},start,start};uint32_t at=spr;
    const auto section=[&](uint32_t qwords){put(rt,at,cnt(out.end,qwords));put(rt,at+8,uint64_t(0x6c000000u|qwords));
        for(uint32_t i=0;i<qwords;++i)put(rt,at+16+16*i,uint32_t(out.end+16*i)^0xabcd0000u);
        out.sections.push_back({at+16,qwords});out.end+=16*qwords;at+=16+16*qwords;};
    section(4);
    if(texture)section(16);else{std::memset(rt.memory.hostPointer(at,17*16),0,17*16);at+=17*16;}
    for(uint32_t q:{(count+15)/16+1,count+1,(count+3)/4+1,(count+3)/4+1,count+2})section(q);
    put(rt,at,kEnd);put(rt,at+8,uint64_t(0));
    return out;
}
}
int main(){try{
    Runtime rt;unsigned checks=0;
    // The route's writer: RAM buffer from 138c680, 0x9c40 qwords, cursor 138c930 after its frame header.
    const uint32_t low=0x138c680,high=low+16*0x9c40,spr=0x70000000;
    for(const bool texture:{false,true})for(unsigned count:{1u,4u,40u}){
        const auto chunk=build(rt,texture?0x70000c00:spr,0x138c930,texture,count);
        const auto sections=vp2s::nativeSprWriterSections(&rt,texture?0x70000c00:spr,low,high,chunk.end);
        check(sections.size()==chunk.sections.size(),"one section per CNT tag, zero tags skipped");++checks;
        uint32_t expected=chunk.start;
        for(size_t i=0;i<sections.size();++i){check(sections[i].source==chunk.sections[i].first && sections[i].qwords==chunk.sections[i].second && sections[i].destination==expected,
            "sections are the writer's data, placed contiguously from its cursor");expected+=16*sections[i].qwords;}
        ++checks;
    }
    const auto chunk=build(rt,spr,0x138c930,false,4);
    check(rejects([&]{vp2s::nativeSprWriterSections(&rt,spr,low,high,chunk.end+16);}),"the chunk must end at the writer's cursor");++checks;
    check(rejects([&]{vp2s::nativeSprWriterSections(&rt,spr,0x138c940,high,chunk.end);}),"sections stay inside the buffer (low)");++checks;
    check(rejects([&]{vp2s::nativeSprWriterSections(&rt,spr,low,chunk.end-16,chunk.end);}),"sections stay inside the buffer (high)");++checks;
    check(rejects([&]{vp2s::nativeSprWriterSections(&rt,spr+4,low,high,chunk.end);}),"the chunk is aligned in scratchpad");++checks;
    check(rejects([&]{vp2s::nativeSprWriterSections(&rt,0x70003800,low,high,chunk.end);}),"the chunk fits its scratchpad half");++checks;
    {auto bad=build(rt,spr,0x138c930,false,4);const auto lane=spr+16+64+17*16;put(rt,lane,cnt(0x138c930+0x40+16,2));
     check(rejects([&]{vp2s::nativeSprWriterSections(&rt,spr,low,high,bad.end);}),"a section away from the running cursor rejects");++checks;
     put(rt,lane,uint64_t(0x30000000u|2)|uint64_t(0x138c970)<<32);
     check(rejects([&]{vp2s::nativeSprWriterSections(&rt,spr,low,high,bad.end);}),"only CNT, zero and END tags are admitted");++checks;}
    {const auto padded=build(rt,spr,0x138c930,false,4);
     // A lane's unused slot is zero-filled (up to 40 qwords): long empty runs are the writer's.
     const auto end=spr+16+64+17*16;std::vector<uint8_t> tail(0xc00-(end-spr));std::memcpy(tail.data(),rt.memory.hostPointer(end,tail.size()),tail.size());
     std::memset(rt.memory.hostPointer(end,40*16),0,40*16);std::memcpy(rt.memory.hostPointer(end+40*16,tail.size()-40*16),tail.data(),tail.size()-40*16);
     check(vp2s::nativeSprWriterSections(&rt,spr,low,high,padded.end).size()==padded.sections.size(),"a 40-qword empty run is skipped");++checks;
     std::memset(rt.memory.hostPointer(spr+16+64,0xc00-80),0,0xc00-80);
     check(rejects([&]{vp2s::nativeSprWriterSections(&rt,spr,low,high,0x138c930+0x40);}),"no END inside the half rejects");++checks;}
    {std::memset(rt.memory.hostPointer(spr,0xc00),0,16);put(rt,spr,kEnd);
     check(rejects([&]{vp2s::nativeSprWriterSections(&rt,spr,low,high,0x138c930);}),"an empty chunk is not the writer's");++checks;}
    // sceDmaGetChan/sceDmaSync for the writer's fromSPR channel, only from its two callers each.
    put(rt,0x35ddc8,uint32_t(0x598e20));put(rt,0x598e20+0x1010,uint8_t(8));
    {R5900Context c{};c.r[31].uw[0]=0x20097c;c.r[4].uw[0]=8;check(vp2s::nativeSprWriterLibrary(&c,&rt,0x123220) && c.r[2].uw[0]==0x1000d000 && c.pc==0x20097c,"fromSPR channel");++checks;
     c.r[31].uw[0]=0x201820;c.r[4].uw[0]=0x1000d000;c.r[5].uw[0]=c.r[6].uw[0]=0;check(vp2s::nativeSprWriterLibrary(&c,&rt,0x1237f0) && c.r[2].uw[0]==0 && c.pc==0x201820,"nothing in flight");++checks;
     c.r[31].uw[0]=0x112fc0;check(!vp2s::nativeSprWriterLibrary(&c,&rt,0x123220),"other callers keep their owner");++checks;
     c.r[31].uw[0]=0x201810;c.r[4].uw[0]=2;check(rejects([&]{vp2s::nativeSprWriterLibrary(&c,&rt,0x123220);}),"another channel rejects");++checks;}
    // The route's mesh (one 4-vertex strip, material, routine 32), laid out as the
    // writer's sections: header 4, material 16, strips 2, normals 5, UVs 2, colours 2, positions 6.
    {const uint32_t mesh=0x138c6e0;std::memset(rt.memory.hostPointer(mesh,0x300),0,0x300);
     const auto half=[&](unsigned i,int16_t v){put(rt,mesh+4+2*i,v);};
     put(rt,mesh,uint32_t(0x6d078000));half(0,592);half(1,23);half(2,1);half(3,22);half(8,9);half(9,4);half(20,32);
     const uint32_t mat=mesh+0x40;put(rt,mat+12,uint32_t(0x6c0f8007));put(rt,mat+16+16,uint32_t(0x42));put(rt,mat+16+20,uint32_t(2));
     put(rt,mat+16+32+4,uint32_t(5));put(rt,mat+16+6*16+4,uint32_t(0x302e4000));put(rt,mat+16+8*16,uint32_t(1));
     const uint32_t strips=mat+0x100;put(rt,strips+12,uint32_t(0x6201c016));put(rt,strips+16,uint8_t(4));
     const uint32_t normals=strips+0x20;put(rt,normals+8,uint32_t(0x01000104));put(rt,normals+12,uint32_t(0x6c048017));
     const uint32_t uvs=normals+0x50;put(rt,uvs+12,uint32_t(0x75048018));const uint32_t colours=uvs+0x20;put(rt,colours+12,uint32_t(0x6e04c019));
     const uint32_t positions=colours+0x20;put(rt,positions+12,uint32_t(0x6c04801a));put(rt,positions+16+64,uint32_t(0x01000101));
     for(unsigned i=0;i<4;++i){put(rt,normals+16+16*i,std::array<float,4>{0,-1,0,1});put(rt,uvs+16+4*i,std::array<int16_t,2>{int16_t(352+3392*(i>>1)),int16_t(416+3264*(i&1))});
        put(rt,colours+16+4*i,uint32_t(0x10808080));put(rt,positions+16+16*i,std::array<float,4>{1839.f+8*(i>>1),2,-372.f-4*(i&1),0});}
     const std::vector<std::pair<uint32_t,uint32_t>> sections{{mesh,4},{mat,16},{strips,2},{normals,5},{uvs,2},{colours,2},{positions,6}};
     vp2s::NativeModels models("");models.publishGeneratedMesh(&rt,sections);check(models.generatedMeshes()==1,"the route's mesh is admitted");++checks;
     models.publishGeneratedMesh(&rt,sections);check(models.generatedMeshes()==1,"republishing replaces it");++checks;
     const auto reject=[&](uint32_t at,uint32_t v,const char *what){uint32_t old;std::memcpy(&old,rt.memory.hostPointer(at,4),4);put(rt,at,v);
        vp2s::NativeModels fresh("");check(rejects([&]{fresh.publishGeneratedMesh(&rt,sections);}) && !fresh.generatedMeshes(),what);put(rt,at,old);++checks;};
     reject(uvs+12,0x75048019,"a lane at another VU row rejects");
     reject(positions+16+64,0x01000104,"a mesh must close with STCYCL 1/1");
     reject(mesh+4,uint32_t(591|23<<16),"the header size must cover its sections");
     reject(strips+16,3,"strip lengths must sum to the vertex count");
     reject(mesh+4+40,uint32_t(20),"routines with frame data (sphere map) reject");
     reject(mat+12,0x6c0e8007,"the material lane is the writer's");
     check(rejects([&]{vp2s::NativeModels fresh("");auto gap=sections;gap[3].first+=16;fresh.publishGeneratedMesh(&rt,gap);}),"sections must be contiguous");++checks;}
    std::printf("scratchpad writer: %u checks passed\n",checks);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"FAIL %s\n",e.what());return 1;}}

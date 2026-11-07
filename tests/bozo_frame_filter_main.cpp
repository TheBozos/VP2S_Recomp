#include "native_frame_filter.h"
#include <cstring>
#include <cstdio>
#include <source_location>
namespace {
void check(bool yes,std::source_location at=std::source_location::current()){if(!yes)throw std::runtime_error("frame filter assertion at "+std::to_string(at.line()));}
template<class T>void put(Runtime &rt,uint32_t at,T v){std::memcpy(rt.memory.hostPointer(at,sizeof(T)),&v,sizeof(T));}
template<class F>void rejects(F f){bool yes=false;try{f();}catch(const std::runtime_error &){yes=true;}check(yes);}
}
int main(){try{
    Runtime rt;std::memset(rt.memory.flat(),0,bozo::kFlatBytes);vp2s::NativeFrameFilters filters;
    constexpr uint32_t object=0x150000,source=0x160000,scene=0x170000,stack=0x1f0000;
    put(rt,object,uint32_t(0x155d50));put(rt,object+0x90,uint32_t(0x155dcc));put(rt,object+0x94,source);
    put(rt,object+0xaf,uint8_t(1));put(rt,0x35ddf8,scene);put(rt,scene+0x40a8,object);put(rt,scene+0x40dc,uint8_t(1));
    for(unsigned seed=0;seed<256;++seed){
        put(rt,object+0xac,uint8_t(seed%4+1));const uint32_t rgb=(seed*0x10101)&0xffffff;put(rt,object+0xa8,rgb);
        const auto before=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
        filters.begin(&rt,stack,object);rejects([&]{filters.begin(&rt,stack,object);});rejects([&]{filters.submit(&rt,object);});
        filters.levels(stack,object,5);rejects([&]{filters.levels(stack,object,5);});
        filters.colour(&rt,stack,object,0,rgb|0x08000000);rejects([&]{filters.colour(&rt,stack,object,0,rgb|0x08000000);});
        filters.colour(&rt,stack,object,3,rgb|0x09000000);filters.colour(&rt,stack,object,4,rgb|0x0a000000);
        rejects([&]{filters.colour(&rt,stack,object,5,rgb);});filters.finish(stack,object);
        const auto snapshot=filters.submit(&rt,object);check(snapshot.source==source && snapshot.levels==5 && snapshot.layers.size()==3 && snapshot.layers[1].level==3 && snapshot.layers[2].colour==(rgb|0x0a000000));
        check(!std::memcmp(before.data(),rt.memory.flat(),before.size()));
        put(rt,object+0x94,source+4);rejects([&]{filters.submit(&rt,object);});put(rt,object+0x94,source);
        put(rt,scene+0x40dc,uint8_t(0));rejects([&]{filters.submit(&rt,object);});put(rt,scene+0x40dc,uint8_t(1));
        put(rt,object+0xad,uint8_t(1));rejects([&]{filters.submit(&rt,object);});put(rt,object+0xad,uint8_t(0));
        check(snapshot.layers[0].colour==(rgb|0x08000000));
    }
    std::puts("256 native frame-filter snapshot/lifetime/ownership cases, no CPU/RAM writes");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

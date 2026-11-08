#include "native_feedback.h"
#include <cstring>
#include <cstdio>
#include <source_location>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("feedback assertion at "+std::to_string(at.line()));}
template<class T>void put(Runtime &rt,uint32_t at,T v){std::memcpy(rt.memory.hostPointer(at,sizeof(v)),&v,sizeof(v));}
template<class F>void rejects(F fn){bool bad=false;try{fn();}catch(const std::runtime_error &){bad=true;}check(bad);}
}
int main(){try{
    Runtime rt;std::memset(rt.memory.flat(),0,bozo::kFlatBytes);vp2s::NativeScene scene;vp2s::NativeFeedbackCopies copies;
    constexpr uint32_t object=0x180000,target=0x190000,current=0x1a0000,stack=0x1fff000,writer=0x230000;
    put(rt,0x35ddf8,current);put(rt,0x35d980,target);put(rt,object,uint32_t(0x152e80));put(rt,object+0x90,uint32_t(0x152efc));
    put(rt,object+0x94,writer);put(rt,object+0x10,current);
    put(rt,target+0x3c76,uint8_t(1));put(rt,target+0x3c60,1.f);put(rt,target+0x3c64,1.f);
    for(auto at:{0x3c68u,0x3c6au,0x3c6cu,0x3c6eu})put(rt,target+at,int16_t(512));
    for(unsigned i=0;i<2;++i){const auto at=target+0xa70*i;put(rt,at+0x1cf0,uint8_t(1));put(rt,at+0x1cf2,uint8_t(i));put(rt,at+0x1cf5,uint8_t(1));
        put(rt,at+0x1cfa,int16_t(512));put(rt,at+0x1cfc,int16_t(512));}
    for(unsigned seed=0;seed<128;++seed){
        const auto layer=uint8_t(seed&1);put(rt,object+0x99,uint8_t((seed>>2)&1));put(rt,object+0x65,layer);put(rt,object+0x63,uint8_t(1));put(rt,current+0x40a8+4*layer,object);put(rt,current+0x40dc+layer,uint8_t(1));
        scene.beginComposition(&rt,stack,target);const auto before=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
        rejects([&]{copies.submit(&rt,scene,object);});copies.begin(&rt,scene,stack,object);
        rejects([&]{copies.begin(&rt,scene,stack,object);});rejects([&]{copies.release(object);});
        put(rt,object+0x94,writer+16);rejects([&]{copies.finish(&rt,stack,object);});put(rt,object+0x94,writer);
        copies.finish(&rt,stack,object);
        put(rt,object+0x99,uint8_t(1-((seed>>2)&1)));rejects([&]{copies.submit(&rt,scene,object);});put(rt,object+0x99,uint8_t((seed>>2)&1));
        put(rt,current+0x40dc+layer,uint8_t(0));rejects([&]{copies.submit(&rt,scene,object);});put(rt,current+0x40dc+layer,uint8_t(1));
        if(seed&2){scene.finishComposition(stack);scene.beginComposition(&rt,stack,target);rejects([&]{copies.submit(&rt,scene,object);});
            copies.begin(&rt,scene,stack,object);copies.finish(&rt,stack,object);}
        const auto copy=copies.submit(&rt,scene,object);check(copy.object==object && copy.source==target && copy.width==512 && copy.height==512 && copy.frame==scene.frameTarget());
        check(!copies.image(scene));copies.published(copy);const auto published=copies.image(scene);check(published && published->object==object && published->frame==copy.frame && published->mode==copy.mode);
        check(!copies.image(vp2s::NativeScene{}));rejects([&]{copies.submit(&rt,scene,object);});copies.release(object);check(!copies.image(scene));
        put(rt,object+0x99,uint8_t(2));rejects([&]{copies.begin(&rt,scene,stack,object);});put(rt,object+0x99,uint8_t((seed>>2)&1));
        copies.begin(&rt,scene,stack,object);put(rt,object+0x63,uint8_t(0));copies.finish(&rt,stack,object);rejects([&]{copies.submit(&rt,scene,object);});put(rt,object+0x63,uint8_t(1));
        check(!std::memcmp(before.data(),rt.memory.flat(),before.size()));scene.finishComposition(stack);
    }
    std::puts("128 screen-feedback frame/owner/writer/pending/mode/disabled/lifetime cases, mutation-free metadata passed");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

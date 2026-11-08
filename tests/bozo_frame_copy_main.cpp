#include "native_frame_copy.h"
#include "bozo/render/renderer.h"
#include <cstring>
#include <cstdio>
#include <source_location>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("frame-copy assertion at "+std::to_string(at.line()));}
template<class T>void put(Runtime &rt,uint32_t at,T value){std::memcpy(rt.memory.hostPointer(at,sizeof(T)),&value,sizeof(T));}
template<class F>void rejects(F f){bool bad=false;try{f();}catch(const std::runtime_error &){bad=true;}check(bad);}
}
int main(){try{
    Runtime rt;std::memset(rt.memory.flat(),0,bozo::kFlatBytes);vp2s::NativeScene scene;vp2s::NativeFrameCopies copies;
    constexpr uint32_t object=0x180000,target=0x190000,current=0x1a0000,stack=0x1fff000;
    put(rt,0x35ddf8,current);put(rt,current+0x444c,object);put(rt,object,uint32_t(0x159ee0));put(rt,object+0x90,uint32_t(0x159f5c));
    put(rt,target+0x3c76,uint8_t(1));put(rt,target+0x3c60,1.f);put(rt,target+0x3c64,1.f);
    put(rt,target+0x3c68,int16_t(512));put(rt,target+0x3c6a,int16_t(512));
    put(rt,target+0x3c6c,int16_t(512));put(rt,target+0x3c6e,int16_t(512));
    for(unsigned i=0;i<2;++i){const auto at=target+0xa70*i;put(rt,at+0x1cf0,uint8_t(1));put(rt,at+0x1cf2,uint8_t(i));put(rt,at+0x1cf5,uint8_t(1));
        put(rt,at+0x1cfa,int16_t(512));put(rt,at+0x1cfc,int16_t(512));}
    put(rt,object+0x94,target);put(rt,object+0xa8,uint32_t(512));put(rt,object+0xac,uint32_t(512));
    put(rt,object+0x198,uint32_t(128));put(rt,object+0x19c,uint32_t(128));put(rt,object+0x63,uint8_t(1));
    unsigned cases=0;
    for(unsigned seed=0;seed<128;++seed){
        // Native projection keeps fractional logical size; the original copy
        // builder and actual target allocation use truncated pixel dimensions.
        put(rt,target+0x3c64,seed&2?1.001875f:1.f);
        scene.beginComposition(&rt,stack,target);
        const uint32_t width=128-seed%64,height=128-(seed*17)%64;put(rt,object+0x1a0,width);put(rt,object+0xec,width);put(rt,object+0x1a4,height);put(rt,object+0xf0,height);
        const uint8_t layer=seed&1;put(rt,object+0x65,layer);put(rt,current+0x40a8+4*layer,object);put(rt,current+0x40dc+layer,uint8_t(1));
        const auto before=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
        copies.begin(&rt,stack,object);rejects([&]{copies.begin(&rt,stack,object);});rejects([&]{copies.submit(&rt,scene,object);});
        rejects([&]{copies.begin(&rt,stack-0x100,object);});
        rejects([&]{copies.activate(stack,object+16,target);});copies.activate(stack,object,target);
        rejects([&]{copies.activate(stack,object,target);});rejects([&]{copies.release(object);});
        copies.finish(&rt,scene,stack,object);const auto snapshot=copies.submit(&rt,scene,object);
        check(snapshot.width==width && snapshot.height==height && snapshot.frame==scene.frameTarget());
        put(rt,object+0x1a0,width+1);rejects([&]{copies.submit(&rt,scene,object);});put(rt,object+0x1a0,width);
        put(rt,current+0x40dc+layer,uint8_t(0));rejects([&]{copies.submit(&rt,scene,object);});put(rt,current+0x40dc+layer,uint8_t(1));
        put(rt,object+0x94,target+16);rejects([&]{copies.submit(&rt,scene,object);});put(rt,object+0x94,target);
        copies.published(object);rejects([&]{copies.published(object);});rejects([&]{copies.submit(&rt,scene,object);});
        check(copies.image(scene,object)->frame==snapshot.frame);copies.release(object);check(!copies.image(scene,object));
        // A new native frame invalidates a captured image even at identical
        // source address/dimensions, before the next original builder runs.
        copies.begin(&rt,stack,object);copies.activate(stack,object,target);copies.finish(&rt,scene,stack,object);
        scene.finishComposition(stack);scene.beginComposition(&rt,stack,target);rejects([&]{copies.submit(&rt,scene,object);});
        copies.begin(&rt,stack,object);copies.activate(stack,object,target);copies.finish(&rt,scene,stack,object);copies.published(object);
        scene.finishComposition(stack);scene.beginComposition(&rt,stack,target);check(!copies.image(scene,object));
        copies.begin(&rt,stack,object);copies.finish(&rt,scene,stack,object);rejects([&]{copies.submit(&rt,scene,object);});
        check(!std::memcmp(before.data(),rt.memory.flat(),before.size()));
        scene.finishComposition(stack);++cases;
    }
    put(rt,target+0x3c64,1.f);scene.beginComposition(&rt,stack,target);
    copies.begin(&rt,stack,object);copies.activate(stack,object,target);
    put(rt,object+0x1a0,uint32_t(129));rejects([&]{copies.finish(&rt,scene,stack,object);});
    put(rt,object+0x1a0,uint32_t(128));put(rt,object+0xec,uint32_t(128));put(rt,object+0x1a4,uint32_t(128));put(rt,object+0xf0,uint32_t(128));
    copies.finish(&rt,scene,stack,object);
    const auto snapshot=copies.submit(&rt,scene,object);auto &renderer=bozo::render::Renderer::instance();renderer.setEnabled(true);
    renderer.beginFrame({1792,1792,512,512});vp2s::drawFrameCopy(snapshot);renderer.endFrame();
    const auto frame=renderer.published();check(frame && frame->batches.size()==2 && frame->vertices.size()==12);
    const auto &snapshotMaterial=frame->batches[0].material;check(snapshotMaterial.target==6 && snapshotMaterial.source==0 && snapshotMaterial.nearest);
    const auto &material=frame->batches[1].material;check(material.target==7 && material.source==6 && material.fromFrame==1 && !material.nearest && !material.depthTest && !material.depthWrite && !material.alphaBlend);
    check(frame->vertices[0].x==1792 && frame->vertices[0].u==0 && frame->vertices[5].y==2304 && frame->vertices[5].v==1);
    std::printf("%u frame-copy ownership/frame/alias/lifetime cases, mutation-free metadata and native draw publication passed\n",cases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

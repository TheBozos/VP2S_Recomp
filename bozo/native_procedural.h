#pragma once
#include "native_scene.h"
namespace vp2s {
// One procedural object's local-to-screen input, owned by its original scalar
// draw invocation. This is game geometry state, never a processor register bank.
struct NativeProceduralProjection {
    uint32_t object=0,camera=0,frame=0;
    std::array<float,16> screenFromLocal{};
};
class NativeProceduralTransforms {
    std::map<uint32_t,NativeProceduralProjection> draws;
public:
    void publish(Runtime *,const R5900Context *,const NativeScene &);
    const NativeProceduralProjection &projection(uint32_t frame,uint32_t object)const;
    void finish(uint32_t frame,uint32_t object);
    size_t pending()const{return draws.size();}
};
class ProceduralOwner {
public:
    virtual ~ProceduralOwner()=default;
    virtual NativeProceduralTransforms &nativeProceduralTransforms()=0;
};
std::optional<bool> nativeProceduralInstruction(R5900Context *,Runtime *,uint32_t);
}

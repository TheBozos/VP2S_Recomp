#pragma once
#include "native_assets.h"
#include <map>
namespace vp2s {
// A mode-3 emitter publishes explicit point chunks into the game's two RAM
// buffers. This owns software records/cursors only, with no transfer device.
class NativeParticles {
    struct Build {uint32_t object,emitter,base,limit,next,index,chunks=0,points=0;unsigned pending=0;};
    std::map<uint32_t,Build> builds;
public:
    void begin(Runtime *,R5900Context *);
    void publish(Runtime *,R5900Context *,NativeAssets &);
    void retire(Runtime *,R5900Context *,uint32_t);
    void finish(Runtime *,R5900Context *);
    size_t outstanding()const{return builds.size();}
};
class ParticleOwner {
public:
    virtual ~ParticleOwner()=default;
    virtual NativeParticles &nativeParticles()=0;
};
std::optional<bool> nativeParticleInstruction(R5900Context *,Runtime *,uint32_t);
}

#pragma once
#include "native_assets.h"
#include <array>
#include <map>
namespace vp2s {
struct ParticleBounds {std::array<float,4> minimum{},maximum{};};
inline constexpr uint32_t particleSphereSites[]={0x22efd8,0x22efdc,0x22efe0,0x22efe4,0x22efe8,0x22efec,0x22eff0,0x22eff4,0x22eff8,0x22effc,0x22f004,0x22f008,0x22f010,0x22f014,0x22f018};
// A mode-3 emitter publishes explicit point chunks into the game's two RAM
// buffers. This owns software records/cursors only, with no transfer device.
class NativeParticles {
    struct Build {uint32_t object,emitter,base,limit,next,index,chunks=0,points=0;unsigned pending=0;bool boundsReady=false;ParticleBounds bounds;bool boundsPublished=false;};
    std::map<uint32_t,Build> builds;
public:
    void begin(Runtime *,R5900Context *);
    void publish(Runtime *,R5900Context *,NativeAssets &);
    void retire(Runtime *,R5900Context *,uint32_t);
    void finish(Runtime *,R5900Context *);
    void publishSphere(Runtime *,R5900Context *);
    void includePoint(uint32_t producerFrame,uint32_t emitter,const std::array<float,4> &);
    void validateEmitter(uint32_t producerFrame,uint32_t emitter)const;
    const ParticleBounds &bounds(uint32_t producerFrame,uint32_t emitter)const;
    size_t outstanding()const{return builds.size();}
};
class ParticleOwner {
public:
    virtual ~ParticleOwner()=default;
    virtual NativeParticles &nativeParticles()=0;
};
std::optional<bool> nativeParticleInstruction(R5900Context *,Runtime *,uint32_t);
}

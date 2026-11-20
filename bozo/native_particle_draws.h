#pragma once
#include "native_models.h"
#include "native_particles.h"
namespace vp2s {
class NativeParticleDraws {
    // VPP3: VU1 program 0x6e0's pass header: passes (1..4), each one kick over every
    // point with the kind advanced by stride kinds per pass (header z / 5 qwords) and
    // ALPHA variant blends[pass]; second is the header's raw y word (not read by 0x6e0).
    struct Material {uint32_t kinds,extent,textureAlpha,function,passes,second,stride;std::array<std::array<uint32_t,5>,4> blends;};
    std::filesystem::path root;
    std::map<std::pair<uint64_t,uint32_t>,std::map<uint32_t,Material>> materials;
    const Material &material(const AssetOrigin &);
public:
    explicit NativeParticleDraws(std::filesystem::path directory):root(std::move(directory)){}
    NativeModelSubmission capture(Runtime *,NativeAssets &,NativeTextures &,const NativeScene &,const NativeParticles &,uint32_t object);
};
class ParticleDrawOwner {
public:
    virtual ~ParticleDrawOwner()=default;
    virtual NativeParticleDraws &nativeParticleDraws()=0;
};
std::optional<bool> nativeParticleDrawInstruction(R5900Context *,Runtime *,uint32_t);
}

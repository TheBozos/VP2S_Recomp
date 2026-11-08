#pragma once
#include "native_models.h"
#include "native_particles.h"
namespace vp2s {
class NativeParticleDraws {
    struct Material {uint32_t kinds,extent,textureAlpha,function,wrapU,wrapV;std::array<std::array<uint32_t,5>,4> blends;};
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

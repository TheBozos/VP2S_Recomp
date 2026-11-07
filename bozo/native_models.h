#pragma once
#include "native_assets.h"
#include "native_scene.h"
#include "native_textures.h"
#include "bozo/render/renderer.h"
#include <filesystem>
#include <map>
#include <vector>
namespace vp2s {
struct NativeModelBatch {
    bozo::render::Material material;
    std::array<float,16> matrix;
    std::vector<bozo::render::Vertex> vertices;
    std::shared_ptr<const bozo::render::Texture> texture;
};
struct NativeModelSubmission {
    uint32_t object=0,camera=0;
    std::vector<NativeModelBatch> batches;
};
class NativeModels {
public:
    struct Mesh;
    using Pack=std::map<uint32_t,std::shared_ptr<const Mesh>>;
private:
    std::filesystem::path root;
    std::map<std::pair<uint64_t,uint32_t>,std::shared_ptr<const Pack>> packs;
    std::shared_ptr<const Pack> load(const AssetOrigin &);
public:
    explicit NativeModels(std::filesystem::path directory):root(std::move(directory)){}
    NativeModelSubmission capture(Runtime *,NativeAssets &,NativeTextures &,const NativeScene &,uint32_t object);
};
class ModelOwner {
public:
    virtual ~ModelOwner()=default;
    virtual NativeModels &nativeModels()=0;
    virtual void drawNativeModel(const NativeModelSubmission &)=0;
};
std::optional<bool> nativeModelInstruction(R5900Context *,Runtime *,uint32_t pc);
}

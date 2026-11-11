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
// VU1 set-up routine 20 (code 0x1078 in the model programs): sphere-map
// texture coordinates. n is the object-space vertex normal, eye the unit
// object-space eye direction (VU row 72), rotation the object rotation with
// normalised rows (rows 73..75 are its transpose). Returns the 12-bit S, T, Q
// the routine stores in the vertex's UV row.
std::array<int32_t,3> sphereMapTexture(const std::array<float,3> &n,const std::array<float,3> &eye,const float (&rotation)[3][3]);
}

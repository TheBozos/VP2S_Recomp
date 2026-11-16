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
    // Meshes generated at run time (no disc origin): the semantic record and the
    // exact bytes it was read from, keyed by the mesh's RAM address.
    struct Generated {std::shared_ptr<const Mesh> mesh;std::vector<uint8_t> bytes;};
    std::map<uint32_t,Generated> generated;
public:
    explicit NativeModels(std::filesystem::path directory):root(std::move(directory)){}
    NativeModelSubmission capture(Runtime *,NativeAssets &,NativeTextures &,const NativeScene &,uint32_t object);
    // The scratchpad sprite batcher (mod001 152ac0) publishes one mesh per chunk:
    // `sections` are its published sections in the writer's order (RAM address,
    // qwords): header, material (when flagged), strip lengths, normals, UVs,
    // colours, positions. Each lane's VIF word must be the one the writer's code
    // composes; values are read from the writer's records, never interpreted.
    void publishGeneratedMesh(Runtime *,const std::vector<std::pair<uint32_t,uint32_t>> &sections);
    size_t generatedMeshes()const{return generated.size();}
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

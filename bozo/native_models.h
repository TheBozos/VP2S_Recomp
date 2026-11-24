#pragma once
#include "native_state.h"
#include "native_assets.h"
#include "native_scene.h"
#include "native_textures.h"
#include "native_feedback.h"
#include "bozo/render/renderer.h"
#include <array>
#include <filesystem>
#include <map>
#include <optional>
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
    // A shadow-class object (159880, 1599a0): its model is a shadow volume.
    // Faces in GS screen space (identity matrix), split by screen winding into
    // stencil increments and decrements; the host counts them (depth GREATER,
    // no colour), then darkens where the count is not 0.
    bool shadow=false;
    std::vector<bozo::render::Vertex> shadowIncrement,shadowDecrement;
    // Non-zero: a model composed by this shadow owner, drawn after the owner's
    // shadow pass. On the GS the caster is drawn first but its frame alpha
    // (0x7f) keeps it out of its own shadow; drawing it after the darkening
    // gives the same picture.
    uint32_t deferUntil=0;
};
// The VU1 shadow program 0x6f0 per strip triangle of the set-up vertices
// (object space): odd triangles swap their first two vertices; a triangle
// facing away from the light (dot(light, (b-a)x(c-a)) < 0, light in object
// space, directional; with `point` set, light is the light's object-space
// position and the direction is a - light) is skipped; a kept one becomes the 10-vertex strip
// a b c c' a a' b b' c' a', c' its copy through the flattened matrix. Each
// strip triangle goes to `increment` or `decrement` by its screen winding;
// a triangle with a vertex at w <= 0 or zero area is not drawn. top and flat
// are column-major (m[4*c+r]) object-to-GS-screen matrices.
// The shadow pass of one submission, as draws with the identity matrix over a
// width x height target centred on 2048: the stencil cleared by the first
// counting draw; increments then decrements (depth GREATER, no colour or depth
// writes); then the GS's darkening sprite where the count is not 0: black,
// alpha 0x3f, (Cs - Cd) * As + Cd. Empty when the volume has no face.
struct NativeShadowDraw {bozo::render::Material material;std::vector<bozo::render::Vertex> vertices;};
std::vector<NativeShadowDraw> nativeShadowPass(const NativeModelSubmission &,float width,float height);
void nativeShadowPrisms(const std::vector<std::array<float,3>> &positions,const std::vector<uint16_t> &strips,const std::array<float,3> &light,
                        const std::array<float,16> &top,const std::array<float,16> &flat,
                        std::vector<bozo::render::Vertex> &increment,std::vector<bozo::render::Vertex> &decrement,bool point=false);
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
    // feedback: the screen-feedback images, for meshes that sample texture L@tF:1.
    NativeModelSubmission capture(Runtime *,NativeAssets &,NativeTextures &,const NativeScene &,uint32_t object,const NativeFeedbackCopies *feedback=nullptr);
    // The scratchpad sprite batcher (mod001 152ac0) publishes one mesh per chunk:
    // `sections` are its published sections in the writer's order (RAM address,
    // qwords): header, material (when flagged), strip lengths, normals, UVs,
    // colours, positions. Each lane's VIF word must be the one the writer's code
    // composes; values are read from the writer's records, never interpreted.
    void publishGeneratedMesh(Runtime *,const std::vector<std::pair<uint32_t,uint32_t>> &sections);
    size_t generatedMeshes()const{return generated.size();}
    // Save states: the generated meshes (told once, at publication) are saved; the
    // disc packs are an immutable cache of converted disc files and stay.
    void serialize(bozo::state::Archive &);
};
class ModelOwner {
public:
    virtual ~ModelOwner()=default;
    virtual NativeModels &nativeModels()=0;
    virtual void drawNativeModel(const NativeModelSubmission &)=0;
};
std::optional<bool> nativeModelInstruction(R5900Context *,Runtime *,uint32_t pc);
// Diagnostic (VP2S_BOZO_TRACE_MODELS_AT): print every model batch while set.
void setNativeModelTrace(bool on);
// VU1 set-up routine 20 (code 0x1078 in the model programs): sphere-map
// texture coordinates. n is the object-space vertex normal, eye the unit
// object-space eye direction (VU row 72), rotation the object rotation with
// normalised rows (rows 73..75 are its transpose). Returns the 12-bit S, T, Q
// the routine stores in the vertex's UV row.
std::array<int32_t,3> sphereMapTexture(const std::array<float,3> &n,const std::array<float,3> &eye,const float (&rotation)[3][3]);
// VU1 set-up routine 26 (code 0x1220 in the model programs): a reflection map
// from the eye's position (vf08, object space) rather than its direction. With
// d = eye - p, inv = 1/|d|, c = n.d: r = 2c n - d, v = Rn r with v.y negated,
// x = v.x inv, y = 1 + v.y inv, z = v.z inv, q = sqrt|y|; S, T, Q =
// (x + 1.41421 q, z + 1.41421 q, 2.82843 q) as 12-bit fixed point. When the
// routine's data .y is set, alpha = FTOI0(base + scale (1 - c inv)^2), written
// to the colour row's w (lighting clamps it later).
struct ReflectionTexture {std::array<int32_t,3> stq{};std::optional<int32_t> alpha;};
ReflectionTexture reflectionTexture(const std::array<float,3> &n,const std::array<float,3> &p,const std::array<float,3> &eye,
                                    const float (&rotation)[3][3],float base,int32_t on,float scale);
// VU1 set-up routine 40 (code 0x2188 in the model programs, the same body in
// every build): texture coordinates projected from the screen. The vertex is
// pushed along its normal, q = n push + p, and taken through the guard matrix
// (rows 85..88, the clip test's): c = G (q, 1). With s = row38.xy * scale and
// o = row39.xy + offset (the data rows' .xy), S, T = (c.xy s + o c.w) / c.w as
// 12-bit fixed point; Q is 1.0 (4096).
struct ScreenProjection {float offset[2]{},push=0,scale[2]{};};
std::array<int32_t,2> screenTexture(const std::array<float,3> &n,const std::array<float,3> &p,const ScreenProjection &data,
                                    const std::array<float,2> &row38,const std::array<float,2> &row39,const std::array<float,16> &guard);
}

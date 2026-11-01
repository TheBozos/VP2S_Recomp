// VP2's models for the native renderer from the game's own data (docs/vp2_models.md): the meshes' vertices from the
// offline conversion of the game's files (tools/vp2_meshes.py: <dir>/<asset part>.msh), found by where the game
// loaded the mesh it draws; the object's matrix, the camera, the visible meshes and their order from the game's EE
// structures. Nothing reads VIF, GIF or VU data while the game runs.

#include "vp2s_models.h"
#include "vp2s_loads.h"
#include "vp2s_textures.h"

#include "ps2_overlay_modules.h"
#include "ps2_runtime.h"
#include "runtime/native/native_renderer.h"

#include <algorithm>
#include <chrono>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
    // One converted mesh (tools/vp2_meshes.py's record).
    struct Mesh
    {
        uint32_t flags = 0;
        int32_t group = 0;
        int32_t routines[4] = {};
        int32_t bases[4] = {};
        float centre[3] = {};
        bool hasMaterial = false;
        uint32_t rows8to12[20] = {}; // the unpacked rows 8..12 (the material block's: 8.y lighting mode, 9.w power, 12 weights)
        uint32_t material[16] = {};
        std::vector<uint16_t> strips;
        struct Vertex
        {
            float position[4];
            float normal[3];
            float uv[2];
            uint8_t rgba[4];
        };
        std::vector<Vertex> vertices;
        struct Bones
        {
            int32_t a;
            float wa;
            int32_t b;
            float wb;
        };
        std::vector<Bones> bones; // routine 4
    };

    // An asset part's meshes by their offset in it.
    using MeshFile = std::unordered_map<uint32_t, std::shared_ptr<const Mesh>>;

    std::mutex g_mutex;
    std::unordered_map<std::string, std::shared_ptr<const MeshFile>> g_files;

    std::string directory()
    {
        if (const char *dir = std::getenv("VP2S_MESHES"))
            return dir;
        return "../vp2_meshes";
    }

    template <typename T>
    bool take(const std::vector<uint8_t> &data, size_t &at, T &out)
    {
        if (at + sizeof(T) > data.size())
            return false;
        std::memcpy(&out, data.data() + at, sizeof(T));
        at += sizeof(T);
        return true;
    }

    // <dir>/<part>.msh (part: the asset file's name without its extension, "<chain offset>_<part>"), once.
    std::shared_ptr<const MeshFile> meshFile(const std::string &part)
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (const auto it = g_files.find(part); it != g_files.end())
            return it->second;
        std::shared_ptr<MeshFile> file;
        if (FILE *f = std::fopen((directory() + "/" + part + ".msh").c_str(), "rb"))
        {
            std::vector<uint8_t> data;
            uint8_t chunk[65536];
            for (size_t n; (n = std::fread(chunk, 1, sizeof(chunk), f)) > 0;)
                data.insert(data.end(), chunk, chunk + n);
            std::fclose(f);
            size_t at = 0;
            char magic[4] = {};
            uint32_t count = 0;
            if (take(data, at, magic) && std::memcmp(magic, "VPM2", 4) == 0 && take(data, at, count))
            {
                file = std::make_shared<MeshFile>();
                for (uint32_t m = 0; m < count; ++m)
                {
                    auto mesh = std::make_shared<Mesh>();
                    uint32_t offset = 0, vertices = 0, strips = 0, hasMaterial = 0;
                    if (!take(data, at, offset) || !take(data, at, vertices) || !take(data, at, strips) ||
                        !take(data, at, mesh->flags) || !take(data, at, mesh->group) || !take(data, at, mesh->routines) ||
                        !take(data, at, mesh->bases) || !take(data, at, mesh->centre) || !take(data, at, hasMaterial) ||
                        !take(data, at, mesh->rows8to12))
                        break;
                    mesh->hasMaterial = hasMaterial != 0;
                    if (mesh->hasMaterial && !take(data, at, mesh->material))
                        break;
                    mesh->strips.resize(strips);
                    if (at + strips * 2u > data.size())
                        break;
                    std::memcpy(mesh->strips.data(), data.data() + at, strips * 2u);
                    at += (strips * 2u + 3u) & ~3u;
                    mesh->vertices.resize(vertices);
                    if (at + vertices * sizeof(Mesh::Vertex) > data.size())
                        break;
                    std::memcpy(mesh->vertices.data(), data.data() + at, vertices * sizeof(Mesh::Vertex));
                    at += vertices * sizeof(Mesh::Vertex);
                    for (const int32_t routine : mesh->routines)
                        if (routine == 4)
                        {
                            mesh->bones.resize(vertices);
                            if (at + vertices * sizeof(Mesh::Bones) > data.size())
                                break;
                            std::memcpy(mesh->bones.data(), data.data() + at, vertices * sizeof(Mesh::Bones));
                            at += vertices * sizeof(Mesh::Bones);
                            break;
                        }
                    (*file)[offset] = std::move(mesh);
                }
            }
        }
        g_files[part] = file; // a missing file is remembered too
        return file;
    }
}

namespace
{
    // The model draw method (mod001 FUN_0023a8d0, a0 the object): its visible meshes, entry i of
    // obj[0x77C] + 12 * count + 4 * i -> {mesh, material block, depth}, count = model[0x34], model = obj[0x7BC].
    constexpr uint32_t kModelDraw = 0x0023A8D0u;
    PS2Runtime::RecompiledFunction g_draw = nullptr;

    uint32_t word(const uint8_t *rdram, uint32_t address)
    {
        uint32_t value;
        std::memcpy(&value, rdram + (address & 0x1FFFFFFu), 4);
        return value;
    }

    float real(const uint8_t *rdram, uint32_t address)
    {
        float value;
        std::memcpy(&value, rdram + (address & 0x1FFFFFFu), 4);
        return value;
    }

    using Mat4 = std::array<std::array<float, 4>, 4>; // rows, applied M * p

    Mat4 rows(const uint8_t *rdram, uint32_t address)
    {
        Mat4 m{};
        for (uint32_t r = 0; r < 4u; ++r)
            for (uint32_t c = 0; c < 4u; ++c)
                m[r][c] = real(rdram, address + 16u * r + 4u * c);
        return m;
    }

    Mat4 product(const Mat4 &a, const Mat4 &b)
    {
        Mat4 m{};
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                m[r][c] = a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c] + a[r][3] * b[3][c];
        return m;
    }

    // A bone matrix (row-vector form: p' = x R0 + y R1 + z R2 + w R3) applied to (x, y, z, w).
    std::array<float, 3> boneApply(const uint8_t *rdram, uint32_t matrix, const float *p, float w)
    {
        std::array<float, 3> out{};
        for (uint32_t c = 0; c < 3u; ++c)
            out[c] = p[0] * real(rdram, matrix + 4u * c) + p[1] * real(rdram, matrix + 16u + 4u * c) +
                     p[2] * real(rdram, matrix + 32u + 4u * c) + w * real(rdram, matrix + 48u + 4u * c);
        return out;
    }

    bool g_frameOpen = false;
    const uint8_t *g_rdram = nullptr; // EE memory, for the frame end's diagnostics
    size_t g_interfaceStart = SIZE_MAX; // the frame's first interface batch (vp2sModelsInterfaceStarts)
    // VP2S_MODELS_LOG: what became of the meshes since the last line (every 120 frames).
    struct Counts
    {
        uint64_t objects = 0, meshes = 0, unplaced = 0, noFile = 0, notConverted = 0, drawn = 0, triangles = 0, textured = 0;
        std::string firstUnplaced, firstMissing;
        float sampleX = 0, sampleY = 0, sampleZ = 0;
    } g_counts;

    // A word the game keeps in main memory or in the scratchpad (0x70000000: its texture manager lives there).
    uint32_t guestWord(const uint8_t *rdram, const uint8_t *scratch, uint32_t address)
    {
        uint32_t value;
        if ((address & 0xF0000000u) == 0x70000000u)
            std::memcpy(&value, scratch + (address & 0x3FFCu) + (address & 3u), 4);
        else
            std::memcpy(&value, rdram + (address & 0x1FFFFFFu), 4);
        return value;
    }

    void drawObject(const uint8_t *rdram, const uint8_t *scratch, uint32_t object, uint32_t camera)
    {
        const uint32_t model = word(rdram, object + 0x7BCu), list = word(rdram, object + 0x77Cu);
        if (model == 0u || camera == 0u)
            return;
        // The screen matrix: s = M330 * V * O (docs/vp2_models.md), as the renderer takes it (column c at [4c..4c+3]).
        const Mat4 screen = product(rows(rdram, camera + 0x330u), product(rows(rdram, camera + 0x2F0u), rows(rdram, object + 0x90u)));
        // The object's lights, from its VU block rows as the game writes them each frame (row r at obj + 0x230 + 16 r):
        // directions 1..4 (world space, taken to object space by O's normalised rotation, as the set-up program does),
        // colours 5..8, ambient 19, the object colour 13, the flags 14 (.w lighting table A), the colour limit 0.
        const auto objectRow = [&](uint32_t r) {
            std::array<float, 4> v{};
            for (uint32_t c = 0; c < 4u; ++c)
                v[c] = real(rdram, object + 0x230u + 16u * r + 4u * c);
            return v;
        };
        const Mat4 world = rows(rdram, object + 0x90u);
        float rn[3][3];
        for (int i = 0; i < 3; ++i)
        {
            const float length = std::sqrt(world[i][0] * world[i][0] + world[i][1] * world[i][1] + world[i][2] * world[i][2]);
            for (int j = 0; j < 3; ++j)
                rn[i][j] = length != 0.0f ? world[i][j] / length : 0.0f;
        }
        std::array<std::array<float, 3>, 4> lightDirection{};
        std::array<std::array<float, 4>, 4> lightColour{};
        for (uint32_t light = 0; light < 4u; ++light)
        {
            const auto d = objectRow(1u + light);
            for (int j = 0; j < 3; ++j)
                lightDirection[light][j] = rn[0][j] * d[0] + rn[1][j] * d[1] + rn[2][j] * d[2];
            lightColour[light] = objectRow(5u + light);
        }
        const auto ambient = objectRow(19u), objectColour = objectRow(13u), limit = objectRow(0u);
        const bool tableA = word(rdram, object + 0x230u + 16u * 14u + 12u) != 0u;
        const bool plainColour = word(rdram, object + 0x230u + 16u * 14u) != 0u; // 14.x: the object colour is 1
        float matrix[16];
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                matrix[4 * c + r] = screen[r][c];
        // The skeleton's bone palette (the buffer filled last).
        const uint32_t skeleton = word(rdram, object + 0x648u);
        uint32_t palette = 0u, bones = 0u;
        if (skeleton != 0u)
        {
            bones = word(rdram, word(rdram, skeleton + 0x34u) + 4u);
            palette = word(rdram, skeleton + 0x4Cu) + 0x40u + 64u * bones * (rdram[(skeleton + 0x50u) & 0x1FFFFFFu] & 1u);
        }
        const uint16_t hidden = static_cast<uint16_t>(word(rdram, object + 0x7A4u) & 0xFFFFu);
        const bool masks = rdram[(model + 0x59u) & 0x1FFFFFFu] != 0u;
        const uint32_t texman = word(rdram, 0x0035DE38u);
        const uint32_t count = word(rdram, model + 0x34u);
        const bool listed = list != 0u && rdram[(object + 0x7C2u) & 0x1FFFFFFu] != 0u;
        uint32_t cursor = model + 0x60u, block = 0u;
        uint32_t carriedAlpha = 0x44u, carriedPrim = 0x1Cu, carriedTex0Hi = 0x4u, carriedMode = 0u; // a mesh without its own material keeps the previous one's
        std::vector<ps2x::native::Vertex> triangles;
        for (uint32_t i = 0; i < count && i < 4096u; ++i)
        {
            uint32_t mesh;
            if (listed)
            {
                const uint32_t entry = word(rdram, list + 12u * count + 4u * i);
                mesh = word(rdram, entry);
                block = word(rdram, entry + 4u);
            }
            else
            {
                mesh = cursor;
                cursor += static_cast<uint32_t>(static_cast<int16_t>(word(rdram, mesh + 4u) & 0xFFFFu));
                if (word(rdram, mesh + 0x14u) & 1u)
                {
                    block = mesh + 0x40u;
                    if (const uint32_t entryIndex = word(rdram, block + 0x5Cu))
                        block = word(rdram, object + 0x734u) + 256u * (entryIndex - 1u);
                }
            }
            if (mesh == 0u)
                continue;
            if (masks && (static_cast<uint16_t>(word(rdram, mesh + 0x36u) & 0xFFFFu) & hidden))
                continue;
            std::string part;
            uint32_t offset = 0;
            ++g_counts.meshes;
            if (!vp2sLoadedFrom(mesh, part, offset))
            {
                // VP2S_MODELS_DUMP=<path>: the EE RAM written once, at the first mesh not placed (to find offline where its
                // data came from; game data: build tree only).
                static const char *s_dump = std::getenv("VP2S_MODELS_DUMP");
                static bool s_dumped = false;
                if (s_dump && !s_dumped)
                {
                    s_dumped = true;
                    if (FILE *f = std::fopen(s_dump, "wb"))
                    {
                        std::fwrite(rdram, 1, 32u << 20, f);
                        std::fclose(f);
                    }
                    std::printf("[vp2s-models] EE RAM written to %s at mesh %x (object %x model %x)\n", s_dump, mesh, object, model);
                }
                if (g_counts.unplaced++ == 0u)
                {
                    char text[96];
                    std::snprintf(text, sizeof(text), "%x (word %08x, listed %d, entry %x)", mesh, word(rdram, mesh), listed ? 1 : 0,
                                  listed ? word(rdram, list + 12u * count + 4u * i) : 0u);
                    g_counts.firstUnplaced = text;
                }
                continue;
            }
            const auto file = meshFile(part);
            if (!file)
            {
                if (g_counts.noFile++ == 0u)
                    g_counts.firstMissing = part;
                continue;
            }
            const auto found = file->find(offset);
            if (found == file->end())
            {
                if (g_counts.notConverted++ == 0u)
                    g_counts.firstMissing = part + " +" + std::to_string(offset);
                continue;
            }
            const Mesh &m = *found->second;
            // The material: the mesh's converted one; the texture by the game's key from its texture manager, through
            // the block's texture slot and palette part (row 15, the game's own fields).
            ps2x::native::Material material;
            if (m.hasMaterial)
            {
                carriedAlpha = m.material[0];
                carriedPrim = m.material[4];
                carriedTex0Hi = m.material[3];
                carriedMode = m.material[1]; // row 8.y: the lighting mode
            }
            else if (block >= 0x40u && block - 0x40u != mesh)
            {
                // The block the game picked is another mesh's own (mesh + 0x40): that mesh's converted material.
                const uint32_t owner = offset + (block - 0x40u) - mesh;
                if (const auto other = file->find(owner); other != file->end() && other->second->hasMaterial)
                {
                    carriedAlpha = other->second->material[0];
                    carriedPrim = other->second->material[4];
                    carriedTex0Hi = other->second->material[3];
                    carriedMode = other->second->material[1];
                }
            }
            const uint32_t alpha = carriedAlpha;
            const uint32_t prim = carriedPrim;
            material.alphaBlend = (prim >> 6) & 1u;
            material.blend = material.alphaBlend ? ps2x::native::blendFromAlpha(alpha) : ps2x::native::Material::Blend::Opaque;
            if (material.alphaBlend)
            {
                // The named blends cover (Cs - Cd) As + Cd, Cs As + Cd, Cd - Cs As, Cd (1 - As), Cd (1 + As) and D alone;
                // any other (A - B) C + D as the generic blend.
                const uint32_t a = alpha & 3u, b = (alpha >> 2) & 3u, c = (alpha >> 4) & 3u, d = (alpha >> 6) & 3u;
                const bool named = a == b || (c == 0u && d == 1u && ((a == 0u && (b == 1u || b == 2u)) || (a == 2u && (b == 0u || b == 1u)) ||
                                                                     (a == 1u && b == 2u)));
                if (!named)
                {
                    material.blend = ps2x::native::Material::Blend::Gs;
                    material.gsAlpha = static_cast<uint8_t>(alpha & 0xFFu);
                    material.gsFix = 0x80u;
                }
            }
            material.alphaTest = true; // VP2's meshes: alpha GREATER 0
            material.alphaRef = 0;
            // Blended meshes write their depth too, as the GS does with ZMSK clear (the title's menu text, a model,
            // stays out of the post effect that way). VP2S_MODELS_BLEND_DEPTH=0: they do not.
            static const bool s_blendDepth = [] { const char *v = std::getenv("VP2S_MODELS_BLEND_DEPTH"); return !(v && v[0] == '0'); }();
            material.depthWrite = !material.alphaBlend || s_blendDepth;
            std::shared_ptr<const ps2x::native::TextureCache::Texture> texture;
            if (block != 0u && texman != 0u && ((prim >> 4) & 1u))
            {
                const int32_t slot = static_cast<int32_t>(word(rdram, block + 0x98u));
                if (slot >= 0 && slot < 256)
                {
                    const int16_t e = static_cast<int16_t>(guestWord(rdram, scratch, texman + 0x2670u + 2u * static_cast<uint32_t>(slot)) & 0xFFFFu);
                    if (e >= 0)
                        texture = vp2sGameTexture(guestWord(rdram, scratch, texman + 0x1008u + 16u * static_cast<uint32_t>(e)),
                                                  guestWord(rdram, scratch, texman + 0x8u + 16u * static_cast<uint32_t>(e)), word(rdram, block + 0x9Cu));
                }
            }
            material.textured = texture != nullptr;
            // VP2S_MODELS_MATLOG: the first meshes' materials as drawn (diagnosis).
            {
                static const bool s_log = std::getenv("VP2S_MODELS_MATLOG") != nullptr;
                static int s_logged = 0;
                if (s_log && s_logged < 60)
                {
                    ++s_logged;
                    std::printf("[vp2s-models] mat object %x %s+%x alpha %02x prim %03x tex0hi %08x mode %u textured %d colour %.2f %.2f %.2f %.2f "
                                "table %d plain %d vertex rgba %u %u %u %u\n",
                                object, part.c_str(), offset, alpha & 0xFFu, prim, carriedTex0Hi, carriedMode, material.textured ? 1 : 0,
                                objectColour[0], objectColour[1], objectColour[2], objectColour[3], tableA ? 1 : 0, plainColour ? 1 : 0,
                                m.vertices.empty() ? 0u : m.vertices[0].rgba[0], m.vertices.empty() ? 0u : m.vertices[0].rgba[1],
                                m.vertices.empty() ? 0u : m.vertices[0].rgba[2], m.vertices.empty() ? 0u : m.vertices[0].rgba[3]);
                    std::fflush(stdout);
                }
            }
            material.textureAlpha = (carriedTex0Hi >> 2) & 1u; // TEX0 TCC (bit 34) of the material's template
            material.textureFunction = static_cast<uint8_t>((carriedTex0Hi >> 3) & 3u); // TFX (bits 35..36)
            // VP2S_MODELS_HIDE_UNTEXTURED_BLEND=1 (diagnosis): leaves out the blended meshes without a texture.
            static const bool s_hideUntexturedBlend = std::getenv("VP2S_MODELS_HIDE_UNTEXTURED_BLEND") != nullptr;
            if (s_hideUntexturedBlend && material.alphaBlend && !((prim >> 4) & 1u))
                continue;
            // The vertices (bones applied), the strips as triangles.
            const bool rigid = std::find(std::begin(m.routines), std::end(m.routines), 2) != std::end(m.routines);
            const bool weighted = !m.bones.empty();
            std::vector<ps2x::native::Vertex> vertices(m.vertices.size());
            for (size_t k = 0; k < m.vertices.size(); ++k)
            {
                const Mesh::Vertex &v = m.vertices[k];
                std::array<float, 3> p{v.position[0], v.position[1], v.position[2]};
                if (palette != 0u && rigid)
                {
                    const uint32_t bone = (static_cast<uint32_t>(v.position[3]) - 0x290u) / 4u;
                    if (bone < bones)
                        p = boneApply(rdram, palette + 64u * bone, v.position, 1.0f);
                }
                else if (palette != 0u && weighted)
                {
                    // Two bones with their weights; an unused second bone has weight 0 (and address 0).
                    const Mesh::Bones &b = m.bones[k];
                    const int32_t ba = (b.a - 0x290) / 4, bb = (b.b - 0x290) / 4;
                    std::array<float, 3> sum{};
                    bool any = false;
                    for (const auto &[bone, weight] : {std::pair{ba, b.wa}, std::pair{bb, b.wb}})
                        if (weight != 0.0f && bone >= 0 && static_cast<uint32_t>(bone) < bones)
                        {
                            const auto q = boneApply(rdram, palette + 64u * static_cast<uint32_t>(bone), v.position, 1.0f);
                            for (int c = 0; c < 3; ++c)
                                sum[c] += q[c] * weight;
                            any = true;
                        }
                    if (any)
                        p = sum;
                }
                ps2x::native::Vertex &out = vertices[k];
                out.x = p[0];
                out.y = p[1];
                out.z = p[2];
                out.u = v.uv[0];
                out.v = v.uv[1];
                // The vertex colour lit as the mesh program does (lighting modes 0 and 1; the others not yet: as stored).
                std::array<float, 4> factor{1.0f, 1.0f, 1.0f, 1.0f};
                if (carriedMode == 0u)
                {
                    factor = objectColour;
                    if (!tableA && !plainColour)
                        factor[0] = factor[1] = factor[2] = 0.0f;
                    // VP2S_MODELS_MODE0_GAIN=<f> (diagnosis): mode 0's RGB factor times f.
                    static const float s_gain = [] { const char *v = std::getenv("VP2S_MODELS_MODE0_GAIN"); return v ? static_cast<float>(std::atof(v)) : 1.0f; }();
                    for (int c = 0; c < 3; ++c)
                        factor[c] *= s_gain;
                }
                else if (carriedMode == 1u)
                {
                    // The normal, skinned like the position (w = 0).
                    std::array<float, 3> n{v.normal[0], v.normal[1], v.normal[2]};
                    if (palette != 0u && rigid)
                    {
                        const uint32_t bone = (static_cast<uint32_t>(v.position[3]) - 0x290u) / 4u;
                        if (bone < bones)
                            n = boneApply(rdram, palette + 64u * bone, v.normal, 0.0f);
                    }
                    else if (palette != 0u && weighted)
                    {
                        const Mesh::Bones &b = m.bones[k];
                        const int32_t ba = (b.a - 0x290) / 4, bb = (b.b - 0x290) / 4;
                        std::array<float, 3> sum{};
                        bool any = false;
                        for (const auto &[bone, weight] : {std::pair{ba, b.wa}, std::pair{bb, b.wb}})
                            if (weight != 0.0f && bone >= 0 && static_cast<uint32_t>(bone) < bones)
                            {
                                const auto q = boneApply(rdram, palette + 64u * static_cast<uint32_t>(bone), v.normal, 0.0f);
                                for (int c = 0; c < 3; ++c)
                                    sum[c] += q[c] * weight;
                                any = true;
                            }
                        if (any)
                            n = sum;
                    }
                    float dots[4];
                    for (int i = 0; i < 4; ++i)
                        dots[i] = std::max(0.0f, lightDirection[i][0] * n[0] + lightDirection[i][1] * n[1] + lightDirection[i][2] * n[2]);
                    for (int c = 0; c < 3; ++c)
                        factor[c] = lightColour[0][c] * dots[0] + lightColour[1][c] * dots[1] + lightColour[2][c] * dots[2] +
                                    lightColour[3][c] * dots[3] + ambient[c];
                    factor[3] = objectColour[3];
                }
                uint8_t lit[4];
                for (int c = 0; c < 4; ++c)
                    lit[c] = static_cast<uint8_t>(std::clamp(static_cast<float>(v.rgba[c]) * factor[c], 0.0f, limit[c] > 0.0f ? std::min(limit[c], 255.0f) : 255.0f));
                out.rgba = uint32_t{lit[0]} | uint32_t{lit[1]} << 8 | uint32_t{lit[2]} << 16 | uint32_t{lit[3]} << 24;
            }
            triangles.clear();
            size_t first = 0;
            for (const uint16_t length : m.strips)
            {
                for (uint32_t k = 2; k < length && first + k < vertices.size(); ++k)
                {
                    const size_t a = first + k - 2, b = first + k - 1, c = first + k;
                    triangles.push_back(vertices[a]);
                    triangles.push_back(vertices[(k & 1u) ? c : b]);
                    triangles.push_back(vertices[(k & 1u) ? b : c]);
                }
                first += length;
            }
            if (!triangles.empty() && g_counts.drawn++ == 0u)
            {
                const ps2x::native::Vertex &v = triangles[0];
                const float x = matrix[0] * v.x + matrix[4] * v.y + matrix[8] * v.z + matrix[12];
                const float y = matrix[1] * v.x + matrix[5] * v.y + matrix[9] * v.z + matrix[13];
                const float z = matrix[2] * v.x + matrix[6] * v.y + matrix[10] * v.z + matrix[14];
                const float w = matrix[3] * v.x + matrix[7] * v.y + matrix[11] * v.z + matrix[15];
                g_counts.sampleX = w != 0.0f ? x / w : 0.0f;
                g_counts.sampleY = w != 0.0f ? y / w : 0.0f;
                g_counts.sampleZ = w != 0.0f ? z / w : 0.0f;
            }
            g_counts.triangles += triangles.size() / 3u;
            g_counts.textured += material.textured ? 1u : 0u;
            if (!triangles.empty())
                ps2x::native::Renderer::instance().draw(material, matrix, triangles.data(), static_cast<uint32_t>(triangles.size()),
                                                        std::move(texture));
        }
    }

    void modelDrawHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t object = getRegU32(ctx, 4), camera = getRegU32(ctx, 6);
        const bool shown = (rdram[(object + 0x7C3u) & 0x1FFFFFFu] >> (rdram[(object + 0x7C5u) & 0x1FFFFFFu] & 7u)) & 1u;
        static bool s_first = true;
        if (s_first)
        {
            s_first = false;
            std::printf("[vp2s-models] first model draw: object %x camera %x shown %d in a scene pass %d\n", object, camera, shown ? 1 : 0,
                        g_frameOpen ? 1 : 0);
        }
        g_draw(rdram, ctx, runtime);
        if (g_frameOpen) // every object drawn this frame (0x7C3 only asks the game to rebuild its mesh list)
        {
            ++g_counts.objects;
            drawObject(rdram, runtime->memory().getScratchpad(), object, camera);
        }
    }

    ps2x::native::Screen nativeScreen()
    {
        ps2x::native::Screen screen;
        screen.originX = 2048.0f - 320.0f;
        screen.originY = 2048.0f - 224.0f;
        screen.width = 640.0f;
        screen.height = 448.0f;
        return screen;
    }
}

void vp2sModelsInterfaceStarts()
{
    if (g_frameOpen && g_interfaceStart == SIZE_MAX)
    {
        g_interfaceStart = ps2x::native::Renderer::instance().batchCount();
        static const bool s_log = std::getenv("VP2S_POST_LOG") != nullptr;
        static uint32_t s_logs = 0;
        if (s_log && ++s_logs % 300u == 1u)
        {
            std::printf("[vp2s-post] interface starts at batch %zu\n", g_interfaceStart);
            std::fflush(stdout);
        }
    }
}

void vp2sModelsFrameEnd()
{
    if (!g_frameOpen)
        return;
    g_interfaceStart = SIZE_MAX;
    static const bool s_log = std::getenv("VP2S_MODELS_LOG") != nullptr;
    static uint64_t s_frames = 0;
    if (s_log && ++s_frames % 120u == 0u)
    {
        const Counts &c = g_counts;
        std::printf("[vp2s-models] %llu frames: %llu objects, %llu meshes: %llu not placed (first at %s), %llu no file, %llu not converted "
                    "(first %s), %llu drawn (%llu textured, %llu triangles); first vertex at %.1f %.1f z %.0f\n",
                    static_cast<unsigned long long>(s_frames), static_cast<unsigned long long>(c.objects),
                    static_cast<unsigned long long>(c.meshes), static_cast<unsigned long long>(c.unplaced), c.firstUnplaced.c_str(),
                    static_cast<unsigned long long>(c.noFile), static_cast<unsigned long long>(c.notConverted), c.firstMissing.c_str(),
                    static_cast<unsigned long long>(c.drawn), static_cast<unsigned long long>(c.textured),
                    static_cast<unsigned long long>(c.triangles), c.sampleX, c.sampleY, c.sampleZ);
        g_counts = Counts{};
    }
    auto &renderer = ps2x::native::Renderer::instance();
    renderer.endFrame();
    renderer.beginFrame(nativeScreen());
    // VP2S_POST_LOG (diagnosis): every 2 s, the post-effect object's parameter blocks (docs/vp2_dof.md): flags +0xEF0C,
    // auto-focus +0x130D0, block +0xF45C and blocks +0xF330 + 0x3C i (enable, p0..p4, level, c, flags, radii, strength).
    {
        static const bool s_post = std::getenv("VP2S_POST_LOG") != nullptr;
        static auto s_last = std::chrono::steady_clock::now();
        if (s_post && std::chrono::steady_clock::now() - s_last > std::chrono::seconds(2))
        {
            s_last = std::chrono::steady_clock::now();
            const uint8_t *rdram = g_rdram;
            const uint32_t owner = rdram ? word(rdram, 0x0035DDF8u) : 0u;
            if (owner)
            {
                const uint32_t post = word(rdram, (owner & 0x1FFFFFFu) + 0x4448u) & 0x1FFFFFFu;
                std::printf("[vp2s-post] object %x flags %08x auto %02x", post, post ? word(rdram, post + 0xEF0Cu) : 0u,
                            post ? rdram[(post + 0x130D0u) & 0x1FFFFFFu] : 0u);
                const auto block = [&](uint32_t at) {
                    std::printf(" | %x:", at - post);
                    for (uint32_t k = 0; k < 0x3Cu; k += 4u)
                        std::printf(" %08x", word(rdram, at + k));
                };
                if (post)
                {
                    block(post + 0xF45Cu);
                    for (uint32_t i = 0; i < 4u; ++i)
                        if (word(rdram, post + 0xF330u + 0x3Cu * i))
                            block(post + 0xF330u + 0x3Cu * i);
                }
                std::printf("\n");
                std::fflush(stdout);
            }
        }
    }
    // Every 5 s: the native frames shown per second (the game's own frames, one per scene pass).
    static auto s_since = std::chrono::steady_clock::now();
    static uint32_t s_shown = 0;
    ++s_shown;
    const auto now = std::chrono::steady_clock::now();
    if (const double seconds = std::chrono::duration<double>(now - s_since).count(); seconds >= 5.0)
    {
        std::printf("[vp2s] native fps %.1f\n", s_shown / seconds);
        std::fflush(stdout);
        s_since = now;
        s_shown = 0;
    }
}

void vp2sInstallModels(PS2Runtime &)
{
    // The native models: drawn after the game's own draw method (whose packets go nowhere: no GS), one native frame a
    // game frame (vp2sModelsFrameEnd). VP2S_GS=1 (the old GS path, to compare) leaves them out.
    if (const char *gs = std::getenv("VP2S_GS"); gs && gs[0] == '1')
    {
        // VP2S_POST_SKIP=<hex address>[,...] with the GS comparison path (diagnosis): those mod001 methods skipped (the
        // effect objects' per-frame methods: 0x2D92A0 depth of field, 0x2DBD20, 0x2E0AE0), to see what the original's
        // picture owes each.
        if (const char *skip = std::getenv("VP2S_POST_SKIP"))
            for (const char *p = skip; *p;)
            {
                char *end = nullptr;
                const uint32_t address = static_cast<uint32_t>(std::strtoul(p, &end, 16));
                if (end == p)
                    break;
                ps2_overlays::replaceModuleFunction("mod001", address, [](uint8_t *, R5900Context *ctx, PS2Runtime *) {
                    ctx->pc = getRegU32(ctx, 31);
                });
                p = *end == ',' ? end + 1 : end;
            }
        return;
    }
    const ps2_overlays::Module *module = ps2_overlays::findModule("mod001");
    if (!module || kModelDraw < module->tableBase || kModelDraw >= module->tableEnd)
    {
        std::printf("[vp2s] native models: mod001 %s\n", module ? "does not hold the draw methods" : "not found");
        return;
    }
    g_draw = module->table[(kModelDraw - module->tableBase) >> 2];
    if (!g_draw)
        return;
    ps2_overlays::replaceModuleFunction("mod001", kModelDraw, &modelDrawHook);
    // The native frame ends with the game's scene pass (mod001 FUN_002211E0, every frame on every screen: the 3D,
    // then the interface layers): the field's frame wait alone left the title and the menus' frames unended.
    constexpr uint32_t kScenePass = 0x002211E0u;
    if (kScenePass >= module->tableBase && kScenePass < module->tableEnd)
    {
        static PS2Runtime::RecompiledFunction s_scenePass = module->table[(kScenePass - module->tableBase) >> 2];
        if (s_scenePass)
            ps2_overlays::replaceModuleFunction("mod001", kScenePass, [](uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime) {
                g_rdram = rdram;
                s_scenePass(rdram, ctx, runtime);
                vp2sModelsFrameEnd();
            });
    }
    // The post effect (docs/vp2_dof.md): where the game runs its object's per-frame method (0x2D92A0, between the 3D
    // and the interface), a native pass from its parameter blocks: each enabled block with the sky mode (+0x1F 0,
    // +0x20 & 3 == 1: drawn where the depth is the cleared one) puts a blurred copy of the frame brightened by g
    // (+0x131E0, 2 when unset) there, Gaussian sigma 0.555 x +0x24 (128-wide pixels), mixed by min(128, 2c) / 128 x
    // +0x1E / 128 (the table's weight at the farthest depth is 128 with p0..p4 0). The strongest block is drawn.
    // VP2S_POST=0: none.
    constexpr uint32_t kPostEffect = 0x002D92A0u;
    if (kPostEffect >= module->tableBase && kPostEffect < module->tableEnd &&
        !(std::getenv("VP2S_POST") && std::getenv("VP2S_POST")[0] == '0'))
    {
        static PS2Runtime::RecompiledFunction s_post = module->table[(kPostEffect - module->tableBase) >> 2];
        if (s_post)
            ps2_overlays::replaceModuleFunction("mod001", kPostEffect, [](uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime) {
                s_post(rdram, ctx, runtime);
                const uint32_t owner = word(rdram, 0x0035DDF8u) & 0x1FFFFFFu;
                const uint32_t post = owner ? word(rdram, owner + 0x4448u) & 0x1FFFFFFu : 0u; // the object (docs/vp2_dof.md)
                if (post < 0x100000u)
                    return;
                const auto real = [&](uint32_t address) {
                    float value;
                    std::memcpy(&value, rdram + (address & 0x1FFFFFFu), 4);
                    return value;
                };
                float g = real(post + 0x131E0u);
                if (!(g > 0.0f && g < 8.0f))
                    g = 2.0f;
                ps2x::native::Renderer::PostEffect best;
                bool any = false;
                for (uint32_t i = 0; i < 5u; ++i)
                {
                    const uint32_t block = post + 0xF330u + 0x3Cu * i;
                    if (!word(rdram, block) || rdram[block + 0x1Fu] != 0u)
                        continue;
                    ps2x::native::Renderer::PostEffect e;
                    e.width = static_cast<int>(std::clamp(word(rdram, post + 0x131ACu), 16u, 512u));
                    e.height = static_cast<int>(std::clamp(word(rdram, post + 0x131B0u), 16u, 512u));
                    if (e.width == 16 || e.height == 16 || e.width > 256 || e.height > 256) // per-frame fields, not kept
                    {
                        e.width = 128;
                        e.height = 89;
                    }
                    e.gain = g;
                    e.sigma = 0.555f * std::max(real(block + 0x24u), 0.0f);
                    e.alpha = std::min(128.0f, 2.0f * rdram[block + 0x1Cu]) / 128.0f * (rdram[block + 0x1Eu] / 128.0f);
                    // The sky mode (+0x20 & 3 == 1) passes the cleared depth only; the others weigh by the table, whose
                    // entries up to p0 (+0x04, Z24 >> 16) are set: from that depth on.
                    const uint32_t p0 = std::min<uint32_t>(word(rdram, block + 4u), 255u);
                    e.depthFrom = (rdram[block + 0x20u] & 3u) == 1u ? 1.0f : 1.0f - (p0 + 1u) * 65536.0f / 16777216.0f;
                    if (!any || e.alpha > best.alpha || (e.alpha == best.alpha && e.depthFrom < best.depthFrom))
                        best = e;
                    any = true;
                }
                static const bool s_log = std::getenv("VP2S_POST_LOG") != nullptr;
                static uint32_t s_calls = 0;
                if (s_log)
                    ps2x::native::Renderer::instance().wantDepth();
                if (s_log && ++s_calls % 300u == 1u)
                {
                    // The owner's other effect objects (+0x4440..+0x4460) and their vtables' first methods.
                    for (uint32_t k = 0x4440u; k <= 0x4460u; k += 4u)
                    {
                        const uint32_t o = word(rdram, owner + k) & 0x1FFFFFFu;
                        const uint32_t vt = o >= 0x100000u ? word(rdram, o) & 0x1FFFFFFu : 0u;
                        std::printf("[vp2s-post] owner+%x = %x vtable %x:", k, o, vt);
                        for (uint32_t m = 0; vt >= 0x100000u && m < 0x80u; m += 4u)
                            std::printf(" %x", word(rdram, vt + m));
                        std::printf("\n");
                    }
                    // The last table the game built (+0xEF10, a | v << 8 | v << 16 | a << 24): its non-zero entries.
                    std::printf("[vp2s-post] table:");
                    for (uint32_t i = 0; i < 256u; ++i)
                        if (const uint32_t t = word(rdram, post + 0xEF10u + 4u * i))
                            std::printf(" %u:%08x", i, t);
                    std::printf("\n");
                    // The presented frame's GS Z at a few points (the title's sky: the top; the field: the middle).
                    for (const float vy : {20.0f, 120.0f, 240.0f, 400.0f})
                    {
                        uint32_t z = 0;
                        const bool got = ps2x::native::Renderer::instance().depthAt(2048.0f, 1824.0f + vy * (448.0f / 480.0f), z);
                        std::printf("[vp2s-post] depth at 320,%.0f: %s %u\n", vy, got ? "Z" : "none", z);
                    }
                    std::printf("[vp2s-post] call %u: object %x, sky blocks %s, g %.2f, %dx%d sigma %.2f alpha %.2f from %.5f; batches %zu, "
                                "interface from %lld\n", s_calls, post, any ? "yes" : "no", g, best.width, best.height, best.sigma,
                                best.alpha, best.depthFrom, ps2x::native::Renderer::instance().batchCount(),
                                g_interfaceStart == SIZE_MAX ? -1ll : static_cast<long long>(g_interfaceStart));
                    std::fflush(stdout);
                }
                if (any && best.alpha > 0.0f)
                    ps2x::native::Renderer::instance().postEffect(best, g_interfaceStart);
            });
    }
    ps2x::native::Renderer::instance().setStreamDraws(false);
    ps2x::native::Renderer::instance().setEnabled(true);
    ps2x::native::Renderer::instance().beginFrame(nativeScreen());
    g_frameOpen = true;
    std::printf("[vp2s] native models from the game's data and the offline mesh conversion (%s)\n", directory().c_str());
}

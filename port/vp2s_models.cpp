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
#include <array>
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
            if (take(data, at, magic) && std::memcmp(magic, "VPM1", 4) == 0 && take(data, at, count))
            {
                file = std::make_shared<MeshFile>();
                for (uint32_t m = 0; m < count; ++m)
                {
                    auto mesh = std::make_shared<Mesh>();
                    uint32_t offset = 0, vertices = 0, strips = 0, hasMaterial = 0;
                    if (!take(data, at, offset) || !take(data, at, vertices) || !take(data, at, strips) ||
                        !take(data, at, mesh->flags) || !take(data, at, mesh->group) || !take(data, at, mesh->routines) ||
                        !take(data, at, mesh->bases) || !take(data, at, mesh->centre) || !take(data, at, hasMaterial))
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
        std::memcpy(&value, rdram + (address & 0x1FFFFFCu), 4);
        return value;
    }

    float real(const uint8_t *rdram, uint32_t address)
    {
        float value;
        std::memcpy(&value, rdram + (address & 0x1FFFFFCu), 4);
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
    // VP2S_MODELS_LOG: what became of the meshes since the last line (every 120 frames).
    struct Counts
    {
        uint64_t objects = 0, meshes = 0, unplaced = 0, noFile = 0, notConverted = 0, drawn = 0, triangles = 0, textured = 0;
        std::string firstUnplaced, firstMissing;
        float sampleX = 0, sampleY = 0, sampleZ = 0;
    } g_counts;

    void drawObject(const uint8_t *rdram, uint32_t object, uint32_t camera)
    {
        const uint32_t model = word(rdram, object + 0x7BCu), list = word(rdram, object + 0x77Cu);
        if (model == 0u || camera == 0u)
            return;
        // The screen matrix: s = M330 * V * O (docs/vp2_models.md), as the renderer takes it (column c at [4c..4c+3]).
        const Mat4 screen = product(rows(rdram, camera + 0x330u), product(rows(rdram, camera + 0x2F0u), rows(rdram, object + 0x90u)));
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
            const uint32_t alpha = m.hasMaterial ? m.material[0] : 0x44u;
            const uint32_t prim = m.hasMaterial ? m.material[4] : 0x1Cu;
            material.alphaBlend = (prim >> 6) & 1u;
            material.blend = material.alphaBlend ? ps2x::native::blendFromAlpha(alpha) : ps2x::native::Material::Blend::Opaque;
            material.alphaTest = true; // VP2's meshes: alpha GREATER 0
            material.alphaRef = 0;
            material.depthWrite = !material.alphaBlend;
            std::shared_ptr<const ps2x::native::TextureCache::Texture> texture;
            if (block != 0u && texman != 0u && ((prim >> 4) & 1u))
            {
                const int32_t slot = static_cast<int32_t>(word(rdram, block + 0x98u));
                if (slot >= 0 && slot < 256)
                {
                    const int16_t e = static_cast<int16_t>(word(rdram, texman + 0x2670u + 2u * static_cast<uint32_t>(slot)) & 0xFFFFu);
                    if (e >= 0)
                        texture = vp2sGameTexture(word(rdram, texman + 0x1008u + 16u * static_cast<uint32_t>(e)),
                                                  word(rdram, texman + 0x8u + 16u * static_cast<uint32_t>(e)), word(rdram, block + 0x9Cu));
                }
            }
            material.textured = texture != nullptr;
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
                    const Mesh::Bones &b = m.bones[k];
                    const uint32_t ba = (static_cast<uint32_t>(b.a) - 0x290u) / 4u, bb = (static_cast<uint32_t>(b.b) - 0x290u) / 4u;
                    if (ba < bones && bb < bones)
                    {
                        const auto pa = boneApply(rdram, palette + 64u * ba, v.position, 1.0f);
                        const auto pb = boneApply(rdram, palette + 64u * bb, v.position, 1.0f);
                        for (int c = 0; c < 3; ++c)
                            p[c] = pa[c] * b.wa + pb[c] * b.wb;
                    }
                }
                ps2x::native::Vertex &out = vertices[k];
                out.x = p[0];
                out.y = p[1];
                out.z = p[2];
                out.u = v.uv[0];
                out.v = v.uv[1];
                out.rgba = uint32_t{v.rgba[0]} | uint32_t{v.rgba[1]} << 8 | uint32_t{v.rgba[2]} << 16 | uint32_t{v.rgba[3]} << 24;
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
        if (shown && g_frameOpen)
        {
            ++g_counts.objects;
            drawObject(rdram, object, camera);
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

void vp2sModelsFrameEnd()
{
    if (!g_frameOpen)
        return;
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
}

void vp2sInstallModels(PS2Runtime &)
{
    // The native models: drawn after the game's own draw method (whose packets go nowhere: no GS), one native frame a
    // game frame (vp2sModelsFrameEnd). VP2S_GS=1 (the old GS path, to compare) leaves them out.
    if (const char *gs = std::getenv("VP2S_GS"); gs && gs[0] == '1')
        return;
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
    ps2x::native::Renderer::instance().setStreamDraws(false);
    ps2x::native::Renderer::instance().setEnabled(true);
    ps2x::native::Renderer::instance().beginFrame(nativeScreen());
    g_frameOpen = true;
    std::printf("[vp2s] native models from the game's data and the offline mesh conversion (%s)\n", directory().c_str());
}

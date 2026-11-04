// VP2's models for the native renderer from the game's own data (docs/vp2_models.md): the meshes' vertices from the
// offline conversion of the game's files (tools/vp2_meshes.py: <dir>/<asset part>.msh), found by where the game
// loaded the mesh it draws; the object's matrix, the camera, the visible meshes and their order from the game's EE
// structures. Nothing reads VIF, GIF or VU data while the game runs.

#include "vp2s_models.h"
#include "vp2s_minimap.h"
#include "vp2s_battle_hud.h"
#include "vp2s_loads.h"
#include "vp2s_textures.h"
#include "vp2s_ui.h"

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
        int32_t scrollRow = -1;   // routine 48 (texture scrolling): the UV offset is .zw of object row 38 + this
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
                    if (std::find(std::begin(mesh->routines), std::end(mesh->routines), 48) != std::end(mesh->routines) &&
                        !take(data, at, mesh->scrollRow))
                        break;
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
    constexpr uint32_t kModelDraw = 0x0023A950u;
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

    // A 4x4 inverse (Gauss-Jordan); false when singular.
    bool inverse(const Mat4 &m, Mat4 &out)
    {
        double a[4][8];
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 8; ++c)
                a[r][c] = c < 4 ? m[r][c] : (c - 4 == r ? 1.0 : 0.0);
        for (int c = 0; c < 4; ++c)
        {
            int pivot = c;
            for (int r = c + 1; r < 4; ++r)
                if (std::fabs(a[r][c]) > std::fabs(a[pivot][c]))
                    pivot = r;
            if (std::fabs(a[pivot][c]) < 1e-12)
                return false;
            std::swap(a[c], a[pivot]);
            const double d = a[c][c];
            for (int k = 0; k < 8; ++k)
                a[c][k] /= d;
            for (int r = 0; r < 4; ++r)
                if (r != c)
                {
                    const double f = a[r][c];
                    for (int k = 0; k < 8; ++k)
                        a[r][k] -= f * a[c][k];
                }
        }
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                out[r][c] = static_cast<float>(a[r][c + 4]);
        return true;
    }

    uint32_t g_camera = 0; // the camera of the last model drawn (the scattering pass's view)

    // The scattering object's in-scatter colour V for the screen's grid points (docs/vp2_dof.md; the microcode's
    // per-vertex term): dir the world direction through the point, cos = dir . sun (+0x13440), phases
    // phiR = 3 / 16pi (1 + cos^2), phiM(g) = (1 - g)^2 / 4pi / (1 + g^2 - 2g cos)^1.5; c = Esun (+0x13460) / betaTot
    // (+0x13470) x (betaR (+0x13480) phiR + betaM (+0x13490) phiM(g1 +0x134A0) [+ beta2 (+0x134B0) phiM(g2 +0x134C0) when
    // flag bit 6]), times max(0, 1 + k (+0x138E0) e^(h / (h + |dir.y|))) (h +0x138D0) when flag bit 7; V = min(255, 128 c).
    // The in-scatter the original shows is about 3x what this formula gives (fit of the GS comparison path, field,
    // scattering on against off: out = 0.55..0.67 F + (86, 24, 23)); its height term is read as e^(-|dir.y| / h) (the
    // original's green and blue in-scatter is not zero). Calibrated until the microcode is read.
    constexpr float kInScatterGain = 3.0f;

    bool scatterGrid(const uint8_t *rdram, uint32_t sc, uint32_t camera, int width, int height, std::vector<uint32_t> &grid)
    {
        if (camera < 0x100000u)
            return false;
        Mat4 inv;
        if (!inverse(product(rows(rdram, camera + 0x330u), rows(rdram, camera + 0x2F0u)), inv))
            return false;
        const auto vec = [&](uint32_t o) { return std::array<float, 3>{real(rdram, sc + o), real(rdram, sc + o + 4u), real(rdram, sc + o + 8u)}; };
        const auto sun = vec(0x13440u), esun = vec(0x13460u), total = vec(0x13470u), rayleigh = vec(0x13480u), mie = vec(0x13490u),
                   mie2 = vec(0x134B0u), k = vec(0x138E0u);
        const float g1 = real(rdram, sc + 0x134A0u), g2 = real(rdram, sc + 0x134C0u), h = real(rdram, sc + 0x138D0u);
        const uint8_t flags = rdram[(sc + 0x18980u) & 0x1FFFFFFu];
        const auto unproject = [&](float x, float y, float z) {
            std::array<float, 4> p{};
            for (int r = 0; r < 4; ++r)
                p[r] = inv[r][0] * x + inv[r][1] * y + inv[r][2] * z + inv[r][3];
            const float w = p[3] != 0.0f ? p[3] : 1.0f;
            return std::array<float, 3>{p[0] / w, p[1] / w, p[2] / w};
        };
        constexpr float kPi = 3.14159265f;
        const auto phase = [&](float g, float cosine) {
            return (1.0f - g) * (1.0f - g) / (4.0f * kPi) / std::pow(std::max(1.0f + g * g - 2.0f * g * cosine, 1e-6f), 1.5f);
        };
        grid.assign(static_cast<size_t>(width) * static_cast<size_t>(height), 0u);
        for (int gy = 0; gy < height; ++gy)
            for (int gx = 0; gx < width; ++gx)
            {
                const float x = 1728.0f + 640.0f * static_cast<float>(gx) / static_cast<float>(width - 1);
                const float y = 1824.0f + 448.0f * static_cast<float>(gy) / static_cast<float>(height - 1);
                const auto nearPoint = unproject(x, y, 16000000.0f), farPoint = unproject(x, y, 1000.0f);
                std::array<float, 3> dir{farPoint[0] - nearPoint[0], farPoint[1] - nearPoint[1], farPoint[2] - nearPoint[2]};
                const float length = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
                if (length > 0.0f)
                    for (float &d : dir)
                        d /= length;
                // The microcode takes the sun negated (its constants: -(+0x13440)).
                const float cosine = -(dir[0] * sun[0] + dir[1] * sun[1] + dir[2] * sun[2]);
                const float phiR = 3.0f / (16.0f * kPi) * (1.0f + cosine * cosine), phiM = phase(g1, cosine), phiM2 = phase(g2, cosine);
                uint32_t rgba = 0x7Fu << 24;
                for (int ch = 0; ch < 3; ++ch)
                {
                    float c = total[ch] != 0.0f ? esun[ch] / total[ch] * (rayleigh[ch] * phiR + mie[ch] * phiM + ((flags & 0x40u) ? mie2[ch] * phiM2 : 0.0f)) : 0.0f;
                    if (flags & 0x80u)
                        c *= std::max(0.0f, 1.0f + k[ch] * std::exp(-std::fabs(dir[1]) / std::max(h, 1e-6f)));
                    rgba |= static_cast<uint32_t>(std::clamp(128.0f * kInScatterGain * c, 0.0f, 255.0f)) << (8 * ch);
                }
                grid[static_cast<size_t>(gy) * static_cast<size_t>(width) + static_cast<size_t>(gx)] = rgba;
            }
        return true;
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

    // Where each vertex's normal (row v) and position (row v + 3) come from in a mesh packet: byte offsets in the
    // packet of their raw 32-bit x, y, z (UINT32_MAX when a row is not unpacked as raw words). The game rewrites
    // these in its own copy of a model for facial animation (Alicia's mouth and eyelids), so the draw reads them there.
    // The VIF codes are tools/vp2_meshes.py's unpack, run over the packet as it sits in EE memory.
    struct VertexSources
    {
        std::vector<uint32_t> normal, position;
    };

    VertexSources vertexSources(const uint8_t *rdram, uint32_t mesh, size_t count)
    {
        static thread_local std::array<std::array<uint32_t, 4>, 1024> mem;
        static thread_local std::array<std::array<uint32_t, 4>, 1024> src;
        for (auto &row : src)
            row.fill(UINT32_MAX);
        for (auto &row : mem)
            row.fill(0u);
        const uint32_t end = static_cast<uint32_t>(static_cast<int16_t>(word(rdram, mesh + 4u) & 0xFFFFu));
        uint32_t cl = 1u, wl = 1u, mask = 0u, p = 0u;
        std::array<uint32_t, 4> rowFill{};
        VertexSources out;
        while (p + 4u <= end)
        {
            const uint32_t w = word(rdram, mesh + p);
            const uint32_t cmd = (w >> 24) & 0x7Fu, num = (w >> 16) & 0xFFu, imm = w & 0xFFFFu;
            p += 4u;
            if ((cmd & 0x60u) == 0x60u)
            {
                const uint32_t vn = (cmd >> 2) & 3u, vl = cmd & 3u, n = num ? num : 256u, comps = vn + 1u;
                const bool unsignedValues = (imm >> 14) & 1u, masked = (cmd >> 4) & 1u;
                const uint32_t addr = imm & 0x3FFu;
                const uint32_t nbytes = vl == 3u ? 2u * n : comps * (32u >> vl) / 8u * n;
                for (uint32_t k = 0; k < n; ++k)
                {
                    const uint32_t qw = (addr + (k / wl) * cl + k % wl) % 1024u;
                    std::array<uint32_t, 4> values{}, from{UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
                    if (vl == 3u)
                    {
                        const uint32_t v = word(rdram, mesh + p + 2u * k) & 0xFFFFu;
                        values = {(v & 0x1Fu) << 3, ((v >> 5) & 0x1Fu) << 3, ((v >> 10) & 0x1Fu) << 3, ((v >> 15) & 1u) << 7};
                    }
                    else
                    {
                        const uint32_t size = 32u >> vl;
                        for (uint32_t c = 0; c < comps; ++c)
                        {
                            const uint32_t at = p + (k * comps + c) * size / 8u;
                            uint32_t raw = word(rdram, mesh + at);
                            if (size < 32u)
                            {
                                raw &= (1u << size) - 1u;
                                if (!unsignedValues && (raw >> (size - 1u)))
                                    raw |= ~((1u << size) - 1u);
                            }
                            else
                                from[c] = at;
                            values[c] = raw;
                        }
                        if (comps == 1u)
                        {
                            values.fill(values[0]);
                            from.fill(from[0]);
                        }
                        else
                            for (uint32_t c = comps; c < 4u; ++c)
                                values[c] = mem[qw][c], from[c] = src[qw][c];
                    }
                    for (uint32_t c = 0; c < 4u; ++c)
                    {
                        if (masked)
                        {
                            const uint32_t m = (mask >> (2u * c + 8u * std::min(k % wl, 3u))) & 3u;
                            if (m == 1u)
                            {
                                mem[qw][c] = rowFill[c];
                                src[qw][c] = UINT32_MAX;
                                continue;
                            }
                            if (m >= 2u)
                                continue;
                        }
                        mem[qw][c] = values[c];
                        src[qw][c] = from[c];
                    }
                }
                p += (nbytes + 3u) / 4u * 4u;
                continue;
            }
            if (cmd == 0x01u)
            {
                cl = imm & 0xFFu;
                wl = (imm >> 8) & 0xFFu;
                if (wl == 0u)
                    wl = 1u;
            }
            else if (cmd == 0x20u)
                mask = word(rdram, mesh + p), p += 4u;
            else if (cmd == 0x30u)
            {
                for (uint32_t c = 0; c < 4u; ++c)
                    rowFill[c] = word(rdram, mesh + p + 4u * c);
                p += 16u;
            }
            else if (cmd == 0x31u)
                p += 16u;
            else if (!(cmd <= 0x07u || cmd == 0x10u || cmd == 0x11u || cmd == 0x13u))
                return out;
        }
        const uint32_t vertices = mem[0][1];
        if (mem[2][1] != count)
            return out;
        // A row is usable when its x, y, z are three consecutive raw words.
        auto row = [&](uint32_t qw) {
            const auto &s = src[qw % 1024u];
            return s[0] != UINT32_MAX && s[1] == s[0] + 4u && s[2] == s[0] + 8u ? s[0] : UINT32_MAX;
        };
        out.normal.resize(count);
        out.position.resize(count);
        for (size_t k = 0; k < count; ++k)
        {
            out.normal[k] = row(vertices + 4u * static_cast<uint32_t>(k));
            out.position[k] = row(vertices + 4u * static_cast<uint32_t>(k) + 3u);
        }
        return out;
    }

    bool g_frameOpen = false;
    const uint8_t *g_rdram = nullptr; // EE memory, for the frame end's diagnostics
    size_t g_interfaceStart = SIZE_MAX; // the frame's first interface batch (vp2sModelsInterfaceStarts)
    bool g_sceneOverlaysDrawn = false;  // this frame's scene overlays (the minimap) are in, before its effect passes
    uint32_t g_scene = 0;               // the scene pass's scene (its a0), last seen

    // The scene's own 2D (the minimap) goes after the 3D and before the game's effect passes, as the GS gets them:
    // drawn at the first effect pass of the frame.
    void sceneOverlays()
    {
        if (g_sceneOverlaysDrawn)
            return;
        g_sceneOverlaysDrawn = true;
        vp2sMinimapDraw();
    }
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

    // shadow: the object's shadow volume instead of its meshes (FUN_002B6640's pass).
    void drawObject(const uint8_t *rdram, const uint8_t *scratch, uint32_t object, uint32_t camera, bool shadow = false)
    {
        std::vector<ps2x::native::Vertex> casters; // shadow: every mesh's triangles, object space
        const uint32_t model = word(rdram, object + 0x7BCu), list = word(rdram, object + 0x77Cu);
        if (model == 0u || camera == 0u)
            return;
        // The screen matrix: s = M330 * V * O (docs/vp2_models.md), as the renderer takes it (column c at [4c..4c+3]).
        const Mat4 screen = product(rows(rdram, camera + 0x330u), product(rows(rdram, camera + 0x2F0u), rows(rdram, object + 0x90u)));
        // The camera's fog, for the meshes whose PRIM has FGE (the town's distance haze, toward the fog colour): F = a + b / w
        // per vertex (w its clip w), at most 255 (no fog), a and b at + 0x5A8 / + 0x5AC (from its far and near at + 0x5B8 /
        // + 0x5BC and their F at + 0x5C4 / + 0x5C0), the fog colour at + 0x5A0 (also the clear colour). VP2S_FOG=0: no fog.
        static const bool s_fogOff = [] { const char *v = std::getenv("VP2S_FOG"); return v && v[0] == '0'; }();
        const float fogA = real(rdram, camera + 0x5A8u), fogB = real(rdram, camera + 0x5ACu);
        const uint32_t fogColour = word(rdram, camera + 0x5A0u) & 0xFFFFFFu;
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
        // Set-up routine 36's inputs, as program 0x0238 leaves them: V O and the eye in object space (its rigid inverse
        // applied to the origin).
        const Mat4 viewObject = product(rows(rdram, camera + 0x2F0u), world);
        std::array<float, 3> eye{};
        for (int j = 0; j < 3; ++j)
            eye[j] = -(viewObject[0][j] * viewObject[0][3] + viewObject[1][j] * viewObject[1][3] + viewObject[2][j] * viewObject[2][3]);
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
        vp2sFitCameraScreen(matrix);
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
        // The game's working copy of the model (+0x630), which its facial animation rewrites every frame: the same
        // meshes at the same offsets. Vertex positions and normals are read there.
        const uint32_t work = word(rdram, object + 0x630u) & 0x1FFFFFFu;
        const bool working = work >= 0x100000u && work != (model & 0x1FFFFFFu) && word(rdram, work + 0x34u) == count && count != 0u &&
                             word(rdram, work + 0x60u) == word(rdram, model + 0x60u) && word(rdram, work + 0x64u) == word(rdram, model + 0x64u);
        uint32_t cursor = model + 0x60u, block = 0u;
        uint32_t carriedAlpha = 0x44u, carriedPrim = 0x1Cu, carriedTex0Hi = 0x4u, carriedMode = 0u, carriedCull = 1u; // a mesh without its own material keeps the previous one's
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
            const uint32_t source = working ? work + (mesh - (model & 0x1FFFFFFu)) : 0u;
            const VertexSources *sources = nullptr;
            if (working && (mesh & 0x1FFFFFFu) >= (model & 0x1FFFFFFu) &&
                std::none_of(std::begin(m.routines), std::end(m.routines), [](int32_t r) { return r == 6 || r == 8; }))
            {
                static std::unordered_map<const Mesh *, VertexSources> s_sources;
                static std::mutex s_sourcesMutex;
                std::lock_guard<std::mutex> lock(s_sourcesMutex);
                auto it = s_sources.find(&m);
                if (it == s_sources.end())
                    it = s_sources.emplace(&m, vertexSources(rdram, mesh & 0x1FFFFFFu, m.vertices.size())).first;
                if (it->second.position.size() == m.vertices.size())
                    sources = &it->second;
            }
            static const bool s_morphLog = std::getenv("VP2S_MORPH_LOG") != nullptr; // (diagnosis, temporary)
            if (s_morphLog)
            {
                // A mesh whose packet in EE RAM changes from one frame to the next (the game edits its vertices).
                static std::unordered_map<uint32_t, uint64_t> s_hash;
                static std::unordered_map<uint32_t, int> s_changes;
                const int32_t size = static_cast<int16_t>(word(rdram, mesh + 4u) & 0xFFFFu);
                uint64_t h = 1469598103934665603ull;
                for (int32_t k = 0; k < size && k < 8192; k += 4)
                    h = (h ^ word(rdram, mesh + static_cast<uint32_t>(k))) * 1099511628211ull;
                auto [it, fresh] = s_hash.try_emplace(mesh, h);
                if (!fresh && it->second != h)
                {
                    it->second = h;
                    if (++s_changes[mesh] == 3)
                        std::printf("[morph] mesh %x object %x %s+%x size %d verts %zu routines %d %d %d %d\n", mesh, object, part.c_str(),
                                    offset, size, m.vertices.size(), m.routines[0], m.routines[1], m.routines[2], m.routines[3]);
                }
            }
            // The material: the mesh's converted one; the texture by the game's key from its texture manager, through
            // the block's texture slot and palette part (row 15, the game's own fields).
            ps2x::native::Material material;
            if (m.hasMaterial)
            {
                carriedAlpha = m.material[0];
                carriedPrim = m.material[4];
                carriedTex0Hi = m.material[3];
                carriedMode = m.material[1]; // row 8.y: the lighting mode
                carriedCull = m.material[5];
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
                    carriedCull = other->second->material[5];
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
            // Blended meshes of a skinned object (a character: Alicia's hair) write their depth, as the GS does with
            // ZMSK clear; the others (the town's glass and glows, the event's light shafts, additive and subtractive
            // effects) are drawn with ZMSK on the PS2 (its GS dump of the Solde sign shot). Their depth, screen-wide
            // planes near the camera, put the whole picture in the depth of field's blurred range.
            // VP2S_MODELS_BLEND_DEPTH=0: none write it, =2: all do.
            static const int s_blendDepth = [] { const char *v = std::getenv("VP2S_MODELS_BLEND_DEPTH"); return v ? std::atoi(v) : 1; }();
            material.depthWrite = !material.alphaBlend || s_blendDepth == 2 || (s_blendDepth == 1 && palette != 0u);
            // VP2S_MODELS_BLEND_LOG: each object with a blended mesh once (its class, fade +0x210, flags +0x7C0..+0x7C7,
            // the mesh's ALPHA and PRIM), to tell which blended meshes write depth on the GS.
            {
                static const bool s_log = std::getenv("VP2S_MODELS_BLEND_LOG") != nullptr;
                static std::vector<uint32_t> s_seen;
                if (s_log && material.alphaBlend && std::find(s_seen.begin(), s_seen.end(), object) == s_seen.end() && s_seen.size() < 200u)
                {
                    s_seen.push_back(object);
                    std::printf("[vp2s-models] blended: object %x class %x fade %x flags %08x %08x alpha %x prim %x mode %x\n", object,
                                word(rdram, object), word(rdram, object + 0x210u), word(rdram, object + 0x7C0u), word(rdram, object + 0x7C4u),
                                alpha, prim, carriedMode);
                    std::fflush(stdout);
                }
            }
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
            // VP2S_MODELS_MATLOG: materials as drawn (diagnosis). Optional PART substring filter and LIMIT
            // keep the title's meshes from consuming a field investigation's log.
            {
                static const bool s_log = std::getenv("VP2S_MODELS_MATLOG") != nullptr;
                static const char *s_part = std::getenv("VP2S_MODELS_MATLOG_PART");
                static const int s_limit = [] { const char *v = std::getenv("VP2S_MODELS_MATLOG_LIMIT"); return v ? std::clamp(std::atoi(v), 1, 10000) : 60; }();
                static int s_logged = 0;
                static std::unordered_map<std::string, bool> s_seen;
                if (s_log && s_logged < s_limit && (!s_part || part.find(s_part) != std::string::npos) &&
                    s_seen.emplace(std::to_string(object) + ":" + part + ":" + std::to_string(offset), true).second)
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
            // PRIM.FGE from the vertices' GIF tag in the block as it is in EE RAM (row 13 at + 0x70: EOP, PRIM, ST RGBAQ
            // XYZF2): the game sets it there, the converted material (the file as loaded) has it clear.
            const bool liveTag = block >= 0x100000u && word(rdram, block + 0x70u) == 0x8000u && word(rdram, block + 0x78u) == 0x412u;
            const uint32_t drawnPrim = liveTag ? (word(rdram, block + 0x74u) >> 15) & 0x7FFu : prim;
            material.fog = !s_fogOff && ((drawnPrim >> 5) & 1u) != 0u && fogB != 0.0f;
            material.fogColour = fogColour;
            // VP2S_MODELS_HIDE_UNTEXTURED_BLEND=1 (diagnosis): leaves out the blended meshes without a texture.
            static const bool s_hideUntexturedBlend = std::getenv("VP2S_MODELS_HIDE_UNTEXTURED_BLEND") != nullptr;
            if (s_hideUntexturedBlend && material.alphaBlend && !((prim >> 4) & 1u))
                continue;
            // The vertices (bones applied), the strips as triangles.
            const bool rigid = std::find(std::begin(m.routines), std::end(m.routines), 2) != std::end(m.routines);
            const bool weighted = !m.bones.empty();
            std::vector<ps2x::native::Vertex> vertices(m.vertices.size());
            // Routine 48 scrolls the texture: the UV (1.0 = the texture) plus .zw of the object's row 38 + index, which
            // the game moves each frame (the title's clouds).
            std::array<float, 2> scroll{};
            if (m.scrollRow >= 0 && m.scrollRow < 16)
            {
                const auto row = objectRow(38u + static_cast<uint32_t>(m.scrollRow));
                scroll = {row[2], row[3]};
            }
            const bool reflective = std::find(std::begin(m.routines), std::end(m.routines), 36) != std::end(m.routines);
            // A vertex's normal, skinned like the position (w = 0).
            const auto normalAt = [&](size_t k, const Mesh::Vertex &v) {
                std::array<float, 3> n{v.normal[0], v.normal[1], v.normal[2]};
                if (sources && sources->normal[k] != UINT32_MAX)
                    for (uint32_t c = 0; c < 3u; ++c)
                        n[c] = real(rdram, source + sources->normal[k] + 4u * c);
                const std::array<float, 3> rest = n;
                if (palette != 0u && rigid)
                {
                    const uint32_t bone = (static_cast<uint32_t>(v.position[3]) - 0x290u) / 4u;
                    if (bone < bones)
                        n = boneApply(rdram, palette + 64u * bone, rest.data(), 0.0f);
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
                            const auto q = boneApply(rdram, palette + 64u * static_cast<uint32_t>(bone), rest.data(), 0.0f);
                            for (int c = 0; c < 3; ++c)
                                sum[c] += q[c] * weight;
                            any = true;
                        }
                    if (any)
                        n = sum;
                }
                return n;
            };
            for (size_t k = 0; k < m.vertices.size(); ++k)
            {
                const Mesh::Vertex &v = m.vertices[k];
                std::array<float, 3> p{v.position[0], v.position[1], v.position[2]};
                if (sources && sources->position[k] != UINT32_MAX)
                    for (uint32_t c = 0; c < 3u; ++c)
                        p[c] = real(rdram, source + sources->position[k] + 4u * c);
                const std::array<float, 3> rest = p;
                if (palette != 0u && rigid)
                {
                    const uint32_t bone = (static_cast<uint32_t>(v.position[3]) - 0x290u) / 4u;
                    if (bone < bones)
                        p = boneApply(rdram, palette + 64u * bone, rest.data(), 1.0f);
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
                            const auto q = boneApply(rdram, palette + 64u * static_cast<uint32_t>(bone), rest.data(), 1.0f);
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
                if (material.fog)
                {
                    const float w = screen[3][0] * p[0] + screen[3][1] * p[1] + screen[3][2] * p[2] + screen[3][3];
                    out.fog = w > 0.0f ? std::clamp(fogA + fogB / w, 0.0f, 255.0f) / 255.0f : 1.0f;
                }
                out.u = v.uv[0] + scroll[0];
                out.v = v.uv[1] + scroll[1];
                if (reflective)
                {
                    // Set-up routine 36 (0x1FE0): a sphere map. R = 2 (n.e) n - e, e the vector to the eye, over |e|; taken
                    // to view space by mem[89..91] (V O's 3x3) with x and z negated; z + 1 = r; STQ = (x + sqrt 2 sqrt r,
                    // y + sqrt 2 sqrt r, 2 sqrt 2 sqrt r) replaces the vertex's texture coordinates (r kept above 0.1).
                    const std::array<float, 3> n = normalAt(k, v);
                    const std::array<float, 3> e{eye[0] - p[0], eye[1] - p[1], eye[2] - p[2]};
                    const float length = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
                    const float d = n[0] * e[0] + n[1] * e[1] + n[2] * e[2];
                    const std::array<float, 3> r{2.0f * d * n[0] - e[0], 2.0f * d * n[1] - e[1], 2.0f * d * n[2] - e[2]};
                    const float scale = length != 0.0f ? 1.0f / length : 0.0f;
                    float view[3];
                    for (int i = 0; i < 3; ++i)
                        view[i] = (viewObject[i][0] * r[0] + viewObject[i][1] * r[1] + viewObject[i][2] * r[2]) * scale;
                    const float q = std::sqrt(std::max(1.0f - view[2], 0.1f)), denominator = 2.82843f * q;
                    out.u = (-view[0] + 1.41421f * q) / denominator;
                    out.v = (view[1] + 1.41421f * q) / denominator;
                }
                // Lighting from the game's object and material fields; normals and positions are already skinned.
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
                    const std::array<float, 3> n = normalAt(k, v);
                    float dots[4];
                    for (int i = 0; i < 4; ++i)
                        dots[i] = std::max(0.0f, lightDirection[i][0] * n[0] + lightDirection[i][1] * n[1] + lightDirection[i][2] * n[2]);
                    for (int c = 0; c < 3; ++c)
                        factor[c] = lightColour[0][c] * dots[0] + lightColour[1][c] * dots[1] + lightColour[2][c] * dots[2] +
                                    lightColour[3][c] * dots[3] + ambient[c];
                    factor[3] = objectColour[3];
                }
                else if (carriedMode == 3u && !tableA && block != 0u &&
                         lightColour[3][0] == 0.0f && lightColour[3][1] == 0.0f && lightColour[3][2] == 0.0f)
                {
                    // The standard library's mode 3: ambient, diffuse and specular weights in material row 12,
                    // and repeated squaring (not an exponent) from row 9.w. Both library variants agree for the
                    // first three lights. Keep the old fallback for a nonzero fourth light until its variant is known.
                    const auto n = normalAt(k, v);
                    const float wa = real(rdram, block + 0x60u), wd = real(rdram, block + 0x64u), ws = real(rdram, block + 0x68u);
                    const int power = std::clamp(static_cast<int32_t>(word(rdram, block + 0x3Cu)), 0, 16);
                    const std::array<float, 3> e{eye[0] - p[0], eye[1] - p[1], eye[2] - p[2]};
                    const float length = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
                    const float inverse = length != 0.0f ? 1.0f / length : 0.0f;
                    float diffuse[3], specular[3];
                    for (int i = 0; i < 3; ++i)
                    {
                        diffuse[i] = std::max(0.0f, (lightDirection[i][0] * wd) * n[0] +
                                                      (lightDirection[i][1] * wd) * n[1] + (lightDirection[i][2] * wd) * n[2]);
                        float s = (lightDirection[i][0] * ws + (e[0] * ws) * inverse) * n[0] +
                                  (lightDirection[i][1] * ws + (e[1] * ws) * inverse) * n[1] +
                                  (lightDirection[i][2] * ws + (e[2] * ws) * inverse) * n[2];
                        for (int r = 0; r < power; ++r)
                            s *= s;
                        specular[i] = s;
                    }
                    for (int c = 0; c < 3; ++c)
                    {
                        float acc = ambient[c] * wa;
                        for (int i = 0; i < 3; ++i)
                            acc += lightColour[i][c] * diffuse[i];
                        acc += lightColour[0][c] * specular[0];
                        acc += lightColour[1][c] * specular[1];
                        factor[c] = acc + lightColour[2][c] * specular[2];
                    }
                    factor[3] = objectColour[3];
                }
                else // mode 2 and unsupported light tables: stored colour, alpha still the object's (a weapon put away is
                     // drawn at 0, Alicia's sword in town)
                    factor[3] = objectColour[3];
                uint8_t lit[4];
                for (int c = 0; c < 4; ++c)
                    lit[c] = static_cast<uint8_t>(std::clamp(static_cast<float>(v.rgba[c]) * factor[c], 0.0f, limit[c] > 0.0f ? std::min(limit[c], 255.0f) : 255.0f));
                out.rgba = uint32_t{lit[0]} | uint32_t{lit[1]} << 8 | uint32_t{lit[2]} << 16 | uint32_t{lit[3]} << 24;
            }
            triangles.clear();
            // Material row 15.x == 0 suppresses back faces. Test the skinned positions in the game's
            // screen space (Y down), before emitting the canonical strip triangles. Fading objects draw
            // both sides; shadow volumes need all caster faces, regardless of the camera's direction.
            const uint32_t cull = block >= 0x100000u ? word(rdram, block + 0x90u) : carriedCull;
            const bool cullBackFaces = !shadow && cull == 0u && rdram[(object + 0x210u) & 0x1FFFFFFu] == 0u;
            std::vector<std::array<float, 3>> projected;
            if (cullBackFaces)
            {
                projected.reserve(vertices.size());
                for (const auto &v : vertices)
                {
                    const float w = screen[3][0] * v.x + screen[3][1] * v.y + screen[3][2] * v.z + screen[3][3];
                    const float q = w > 0.0f ? 1.0f / w : 0.0f;
                    projected.push_back({(screen[0][0] * v.x + screen[0][1] * v.y + screen[0][2] * v.z + screen[0][3]) * q,
                                         (screen[1][0] * v.x + screen[1][1] * v.y + screen[1][2] * v.z + screen[1][3]) * q, w});
                }
            }
            size_t first = 0;
            for (const uint16_t length : m.strips)
            {
                for (uint32_t k = 2; k < length && first + k < vertices.size(); ++k)
                {
                    const size_t a = first + k - 2, b = first + k - 1, c = first + k;
                    if (cullBackFaces && projected[a][2] > 0.0f && projected[b][2] > 0.0f && projected[c][2] > 0.0f)
                    {
                        const auto &p = projected[a], &q = projected[b], &r = projected[c];
                        const float area = (p[0] - q[0]) * (q[1] - r[1]) - (p[1] - q[1]) * (q[0] - r[0]);
                        if (!std::signbit(area * ((k & 1u) ? -1.0f : 1.0f)))
                            continue;
                    }
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
            if (shadow)
            {
                casters.insert(casters.end(), triangles.begin(), triangles.end());
                continue;
            }
            g_counts.triangles += triangles.size() / 3u;
            g_counts.textured += material.textured ? 1u : 0u;
            if (!triangles.empty())
                ps2x::native::Renderer::instance().draw(material, matrix, triangles.data(), static_cast<uint32_t>(triangles.size()),
                                                        std::move(texture));
        }
        if (!shadow || casters.empty())
            return;
        // The shadow (FUN_002B6640, VU1 program 0x06f0): the object's triangles, each extruded along the light into a
        // closed prism down past its ground plane (y = 0 in object space); the union is the object's shadow volume, which
        // the renderer counts and darkens the frame in once by the 0x40 alpha of the game's darkening sprites. The light:
        // its block's row 37 taken to object space as program 0x0238 does (a point light's position by O's rigid inverse
        // when table A is on and row 40.w is set, else a direction by Rn^T).
        std::array<float, 4> light = objectRow(37u);
        const bool point = tableA && word(rdram, object + 0x230u + 16u * 40u + 12u) != 0u;
        std::array<float, 3> l{};
        for (int j = 0; j < 3; ++j)
            l[j] = point ? world[0][j] * (light[0] - world[0][3] * light[3]) + world[1][j] * (light[1] - world[1][3] * light[3]) +
                               world[2][j] * (light[2] - world[2][3] * light[3])
                         : rn[0][j] * light[0] + rn[1][j] * light[1] + rn[2][j] * light[2];
        if (std::fabs(l[1]) < 1e-3f)
            return; // light along the ground: no extrusion to it
        using P3 = std::array<float, 3>;
        const auto sub = [](const P3 &a, const P3 &b) { return P3{a[0] - b[0], a[1] - b[1], a[2] - b[2]}; };
        const auto cross = [](const P3 &a, const P3 &b) {
            return P3{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
        };
        const auto dot = [](const P3 &a, const P3 &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
        // Down the light well past the ground plane: the volume darkens what lies under the object along the light.
        const float length = std::sqrt(dot(l, l));
        const float reach = length > 0.0f ? 3000.0f / length : 0.0f;
        const auto flat = [&](const P3 &p) {
            const float t = std::min(0.0f, p[1] / l[1]) - reach;
            return P3{p[0] - l[0] * t, p[1] - l[1] * t, p[2] - l[2] * t};
        };
        std::vector<ps2x::native::Vertex> volume;
        volume.reserve(casters.size() * 8u);
        const auto emit = [&](const P3 &a, const P3 &b, const P3 &c, const P3 &inside) {
            const P3 n = cross(sub(b, a), sub(c, a));
            const P3 centre = {(a[0] + b[0] + c[0]) / 3.0f, (a[1] + b[1] + c[1]) / 3.0f, (a[2] + b[2] + c[2]) / 3.0f};
            const bool flip = dot(n, sub(centre, inside)) < 0.0f; // each face outward
            for (const P3 *p : {&a, flip ? &c : &b, flip ? &b : &c})
            {
                ps2x::native::Vertex vertex;
                vertex.x = (*p)[0];
                vertex.y = (*p)[1];
                vertex.z = (*p)[2];
                vertex.rgba = 0x40000000u;
                volume.push_back(vertex);
            }
        };
        for (size_t t = 0; t + 2u < casters.size(); t += 3u)
        {
            const P3 a = {casters[t].x, casters[t].y, casters[t].z}, b = {casters[t + 1].x, casters[t + 1].y, casters[t + 1].z},
                     c = {casters[t + 2].x, casters[t + 2].y, casters[t + 2].z};
            const P3 n = cross(sub(b, a), sub(c, a));
            if (dot(n, n) < 1e-12f)
                continue;
            const P3 a2 = flat(a), b2 = flat(b), c2 = flat(c);
            const P3 inside = {(a[0] + b[0] + c[0] + a2[0] + b2[0] + c2[0]) / 6.0f, (a[1] + b[1] + c[1] + a2[1] + b2[1] + c2[1]) / 6.0f,
                               (a[2] + b[2] + c[2] + a2[2] + b2[2] + c2[2]) / 6.0f};
            emit(a, b, c, inside);
            emit(a2, b2, c2, inside);
            emit(a, b, b2, inside);
            emit(a, b2, a2, inside);
            emit(b, c, c2, inside);
            emit(b, c2, b2, inside);
            emit(c, a, a2, inside);
            emit(c, a2, c2, inside);
        }
        if (volume.empty())
            return;
        ps2x::native::Material material;
        material.textured = false;
        material.blend = ps2x::native::Material::Blend::ShadowVolume;
        material.depthWrite = false;
        ps2x::native::Renderer::instance().draw(material, matrix, volume.data(), static_cast<uint32_t>(volume.size()));
    }

    bool g_orderFrame = false; // VP2S_DRAW_ORDER_LOG's frame
    int g_frameModelIndex = 0; // the models drawn so far this frame
    size_t g_brightnessAt = SIZE_MAX; // the batch where the frame's brightness object was drawn (this frame)

    void modelDrawHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t object = getRegU32(ctx, 4), camera = getRegU32(ctx, 6);
        g_camera = camera & 0x1FFFFFFu;
        // The frame starts from the camera's clear colour (+0x5A0, 0x00BBGGRR): the GS clear the scene pass sends
        // first (the title's teal (50, 100, 84), seen under the sky's lower edge and through the horizon's gradient).
        if (g_camera >= 0x100000u)
            ps2x::native::Renderer::instance().setClearColour(rdram[g_camera + 0x5A0u], rdram[g_camera + 0x5A1u], rdram[g_camera + 0x5A2u]);
        if (g_camera >= 0x100000u && std::getenv("VP2S_CAMERA_LOG")) // (diagnosis) a camera's screen fields as they change
        {
            static uint64_t s_last = 0;
            const uint64_t now = uint64_t{g_camera} << 32 ^ word(rdram, g_camera + 0x238u) ^ uint64_t{word(rdram, g_camera + 0x260u)} << 8 ^
                                 word(rdram, g_camera + 0x264u) ^ uint64_t{word(rdram, g_camera + 0x224u)} << 16;
            if (now != s_last)
            {
                s_last = now;
                std::printf("[vp2s-camera] %x centre %.1f %.1f viewport %.1f %.1f %.1f %.1f fov %.4f aspect %.4f +0x5A0 %08x\n", g_camera,
                            real(rdram, g_camera + 0x244u), real(rdram, g_camera + 0x248u), real(rdram, g_camera + 0x260u),
                            real(rdram, g_camera + 0x264u), real(rdram, g_camera + 0x268u), real(rdram, g_camera + 0x26Cu),
                            real(rdram, g_camera + 0x224u), real(rdram, g_camera + 0x238u), word(rdram, g_camera + 0x5A0u));
                std::fflush(stdout);
            }
        }
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
            const size_t before = ps2x::native::Renderer::instance().batchCount();
            // VP2S_MODELS_SKIP=<i>[,<i>...] (diagnosis): the frame's i-th model drawn (VP2S_DRAW_ORDER_LOG's order) left out.
            static const std::vector<int> s_skip = [] {
                std::vector<int> out;
                if (const char *v = std::getenv("VP2S_MODELS_SKIP"))
                    for (const char *at = v; *at;)
                    {
                        char *end = nullptr;
                        out.push_back(static_cast<int>(std::strtol(at, &end, 10)));
                        at = (end && *end == ',') ? end + 1 : (end ? end : at + 1);
                        if (end && *end == 0)
                            break;
                    }
                return out;
            }();
            const int index = g_frameModelIndex++;
            if (std::find(s_skip.begin(), s_skip.end(), index) == s_skip.end())
                drawObject(rdram, runtime->memory().getScratchpad(), object, camera);
            // VP2S_DRAW_ORDER_LOG=1 (diagnosis): one frame in 600, each object drawn with its camera and batches.
            static const bool s_order = std::getenv("VP2S_DRAW_ORDER_LOG") != nullptr;
            if (s_order && g_orderFrame)
                std::printf("[vp2s-order] object %x class %x camera %x batches %zu..%zu translation %.1f %.1f %.1f +60 %08x %08x %08x %08x +7c0 %08x %08x +210 %08x\n",
                            object & 0x1FFFFFFu, word(rdram, object & 0x1FFFFFFu), camera & 0x1FFFFFFu, before,
                            ps2x::native::Renderer::instance().batchCount(), real(rdram, (object & 0x1FFFFFFu) + 0x9Cu),
                            real(rdram, (object & 0x1FFFFFFu) + 0xACu), real(rdram, (object & 0x1FFFFFFu) + 0xBCu),
                            word(rdram, (object & 0x1FFFFFFu) + 0x60u), word(rdram, (object & 0x1FFFFFFu) + 0x64u),
                            word(rdram, (object & 0x1FFFFFFu) + 0x68u), word(rdram, (object & 0x1FFFFFFu) + 0x6Cu),
                            word(rdram, (object & 0x1FFFFFFu) + 0x7C0u), word(rdram, (object & 0x1FFFFFFu) + 0x7C4u),
                            word(rdram, (object & 0x1FFFFFFu) + 0x210u));
        }
    }

    // The model-like objects of class 0x154180 (Alicia's shot among them) are not drawn through 0x23A950: their draw
    // 0x22F140 puts the object's + 0x5B0 in the render list 0x201F20 (VU1 program 0x818 of mod001's microcode). Their
    // build (0x22F170) fills the same VU1 header as the models': drawn here as one, with the scene's camera.
    // VP2S_MODELS_CLASS154100=0: not drawn.
    constexpr uint32_t kModelLikeDraw = 0x0022F140u;
    PS2Runtime::RecompiledFunction g_modelLikeDraw = nullptr;

    // The particles of a class 0x154180 object (its emitter at + 0xA90 writes them each frame, 0x22E650): the buffer the
    // game wrote last (index (+0xAF9 == 0)), its chunks from + 0xAA0 + 4 i (count + 0xAC0 + 4 i), a chunk + 0x10 its
    // particles, + 0x14 its size in qwords, + 0x18 the points a primitive, the particles from + 0x20, three qwords each:
    // position (xyz, 1, the object's space), colour (rgba, 128 = 1), V (an angle, two scales, a kind as an integer).
    // Drawn as VU1 program 0x818 draws them: a quad in view space around the point, its corners (x0, y0) .. (x1, y1) of
    // the kind scaled by V.y / V.z and turned by V.x, UV (s0, t0) .. (s1, t1), its colour C * the kind's (at most 255).
    // The GS state: the object's block from + 0x710 (copied from its effect file by 0x22FC60, sent to VU 736): + 0x740
    // TEX0, + 0x760 ALPHA. The kinds: in the effect file (* (+0x628) + 0xC0, VU 745), five qwords each (corners (x0, y0),
    // (x1, y1), STQ 0, STQ 1, colour). The texture: the game's own at TEX0's TBP (its texture manager), CSA from TEX0.
    // Only mode 3 (+0xAFF: the code image 0x818 runs from). False: not drawn here (the game's VU1 path draws it).
    // VP2S_PARTICLES=0: not drawn.
    bool drawParticles(const uint8_t *rdram, const uint8_t *scratch, uint32_t object)
    {
        static const bool s_off = [] { const char *v = std::getenv("VP2S_PARTICLES"); return v && v[0] == '0'; }();
        static const bool s_log = std::getenv("VP2S_MODELS_LOG") != nullptr;
        const uint32_t o = object & 0x1FFFFFFu;
        if (s_off || g_camera == 0u || o < 0x100000u || rdram[o + 0xAFFu] != 3u)
            return false;
        const uint32_t block = o + 0x710u;
        const uint32_t kinds = (word(rdram, o + 0x628u) & 0x1FFFFFFu) + 0xC0u;
        // The block as 0x818 reads it: a GIF tag of two A+D registers at + 0x10 (TEX0 twice).
        if (block < 0x100000u || word(rdram, block + 0x10u) != 2u || word(rdram, block + 0x14u) != 0x10000000u)
        {
            static int s_bad = 0;
            if (s_log && s_bad++ < 4)
            {
                std::printf("[vp2s-models] particles of %x: no constant block (+0x720 = %x %x)\n", o, word(rdram, block + 0x10u), word(rdram, block + 0x14u));
                std::fflush(stdout);
            }
            return false;
        }
        const uint64_t tex0 = uint64_t{word(rdram, block + 0x30u)} | uint64_t{word(rdram, block + 0x34u)} << 32;
        const uint32_t alpha = word(rdram, block + 0x50u);
        const uint32_t index = rdram[o + 0xAF9u] != 0u ? 0u : 1u;
        uint32_t chunk = (word(rdram, o + 0xAA0u + 4u * index) + 0x20u) & 0x1FFFFFFu;
        const int32_t chunks = static_cast<int32_t>(word(rdram, o + 0xAC0u + 4u * index));
        // Object to view (0x818's VU 77..80), and view to screen.
        const Mat4 toView = product(rows(rdram, g_camera + 0x2F0u), rows(rdram, o + 0x90u));
        const Mat4 screen = rows(rdram, g_camera + 0x330u);
        float matrix[16];
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                matrix[4 * c + r] = screen[r][c];
        std::vector<ps2x::native::Vertex> triangles;
        for (int32_t i = 0; i < chunks && i < 64 && chunk >= 0x100000u; ++i)
        {
            const int32_t count = static_cast<int32_t>(word(rdram, chunk + 0x10u));
            const int32_t group = static_cast<int32_t>(word(rdram, chunk + 0x18u));
            const uint32_t qwords = word(rdram, chunk + 0x14u);
            if (group == 1)
                for (int32_t p = 0; p < count && p < 512; ++p)
                {
                    const uint32_t at = chunk + 0x20u + 48u * static_cast<uint32_t>(p);
                    const uint32_t kindIndex = word(rdram, at + 44u);
                    if (kindIndex > 15u)
                        continue;
                    const uint32_t kind = kinds + 0x50u * kindIndex;
                    float view[3];
                    for (int k = 0; k < 3; ++k)
                        view[k] = toView[k][0] * real(rdram, at) + toView[k][1] * real(rdram, at + 4u) + toView[k][2] * real(rdram, at + 8u) +
                                  toView[k][3];
                    const float angle = real(rdram, at + 32u), sx = real(rdram, at + 36u), sy = real(rdram, at + 40u);
                    const float c = std::cos(angle), s = std::sin(angle);
                    uint8_t c8[4];
                    for (int k = 0; k < 4; ++k)
                        c8[k] = static_cast<uint8_t>(std::clamp(real(rdram, at + 16u + 4u * k) * real(rdram, kind + 0x40u + 4u * k), 0.0f, 255.0f));
                    const uint32_t rgba = uint32_t{c8[0]} | uint32_t{c8[1]} << 8 | uint32_t{c8[2]} << 16 | uint32_t{c8[3]} << 24;
                    const float x[2] = {real(rdram, kind), real(rdram, kind + 0x10u)}, y[2] = {real(rdram, kind + 4u), real(rdram, kind + 0x14u)};
                    const float q[2] = {real(rdram, kind + 0x28u), real(rdram, kind + 0x38u)};
                    const float u[2] = {real(rdram, kind + 0x20u) / (q[0] != 0.0f ? q[0] : 1.0f), real(rdram, kind + 0x30u) / (q[1] != 0.0f ? q[1] : 1.0f)};
                    const float v[2] = {real(rdram, kind + 0x24u) / (q[0] != 0.0f ? q[0] : 1.0f), real(rdram, kind + 0x34u) / (q[1] != 0.0f ? q[1] : 1.0f)};
                    // 0x818's strip: (x0, y0), (x1, y0), (x0, y1), (x1, y1).
                    ps2x::native::Vertex corner[4]{};
                    for (int k = 0; k < 4; ++k)
                    {
                        const float cx = x[k & 1], cy = y[k >> 1];
                        corner[k].x = view[0] + c * sx * cx + s * sy * cy;
                        corner[k].y = view[1] - s * sx * cx + c * sy * cy;
                        corner[k].z = view[2];
                        corner[k].u = u[k & 1];
                        corner[k].v = v[k >> 1];
                        corner[k].rgba = rgba;
                    }
                    for (const int k : {0, 1, 2, 1, 3, 2})
                        triangles.push_back(corner[k]);
                }
            chunk = (chunk + 16u * qwords) & 0x1FFFFFFu;
        }
        // The texture the game has at TEX0's TBP (its texture manager, *0x35DE38 in the scratchpad: the list from the byte
        // at + 0x2008, key + 0x1008 / + 0x8 + 16 e, TEX0 + 0x1010 + 16 e).
        std::shared_ptr<const ps2x::native::TextureCache::Texture> texture;
        const uint32_t texman = word(rdram, 0x0035DE38u);
        if (scratch && (texman & 0xF0000000u) == 0x70000000u)
        {
            uint32_t e = scratch[(texman + 0x2008u) & 0x3FFFu];
            for (int guard = 0; e != 0u && guard < 256; ++guard, e = scratch[(texman + 0x2008u + e) & 0x3FFFu])
                if ((guestWord(rdram, scratch, texman + 0x1010u + 16u * e) & 0x3FFFu) == (tex0 & 0x3FFFu))
                {
                    texture = vp2sGameTexture(guestWord(rdram, scratch, texman + 0x1008u + 16u * e), guestWord(rdram, scratch, texman + 0x8u + 16u * e),
                                              static_cast<uint32_t>(tex0 >> 56) & 0x1Fu);
                    break;
                }
        }
        static int s_logged = 0;
        if (s_log && s_logged < 6)
        {
            ++s_logged;
            std::printf("[vp2s-models] particles of %x: %zu triangles, TEX0 %016llx ALPHA %x, texture %s, kind 0 at %x: %g %g %g %g | %g %g %g %g | %g %g %g\n", o,
                        triangles.size() / 3u, static_cast<unsigned long long>(tex0), alpha, texture ? "found" : "not found", kinds,
                        real(rdram, kinds), real(rdram, kinds + 4u), real(rdram, kinds + 0x10u), real(rdram, kinds + 0x14u), real(rdram, kinds + 0x20u),
                        real(rdram, kinds + 0x24u), real(rdram, kinds + 0x30u), real(rdram, kinds + 0x34u), real(rdram, kinds + 0x40u),
                        real(rdram, kinds + 0x44u), real(rdram, kinds + 0x4Cu));
            std::fflush(stdout);
        }
        if (triangles.empty())
            return true;
        ps2x::native::Material material;
        material.tex0 = tex0;
        material.textured = texture != nullptr;
        material.alphaBlend = true;
        material.blend = ps2x::native::blendFromAlpha(alpha);
        material.depthTest = true;
        material.depthWrite = false;
        material.alphaTest = false;
        material.textureAlpha = ((tex0 >> 34) & 1u) != 0u; // TCC
        vp2sFitCameraScreen(matrix);
        ps2x::native::Renderer::instance().draw(material, matrix, triangles.data(), static_cast<uint32_t>(triangles.size()), std::move(texture));
        return true;
    }

    // The draw of a class 0x154180 object: the original (0x201F20) queues the VIF1 packet its build made (0x22DDF0) for
    // VU1 (whose output is lost: the particles are drawn here). It stays queued: the end of each scene object's DMA
    // (0x21C9A0) is what lets the scene pass go on (its +0x40DD, waited for at 0x221030, cleared by the last object's),
    // so leaving it out hangs the game when this object is the frame's last. VP2S_PARTICLES_NO_VU1=1 (to study that
    // protocol): not queued.
    void modelLikeDrawHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t object = getRegU32(ctx, 4);
        static const bool s_off = [] { const char *v = std::getenv("VP2S_MODELS_CLASS154100"); return v && v[0] == '0'; }();
        static const bool s_log = std::getenv("VP2S_MODELS_LOG") != nullptr;
        static int s_logged = 0;
        if (s_log && s_logged < 8)
        {
            ++s_logged;
            // Its mesh (0x22DDF0): mode +0xAFF (the microcode image), chunk lists +0xAA0 / +0xAA4, counts +0xAC0 / +0xAC4;
            // a chunk: VIF data at its + 0x20, +0x14 its size in qwords; where the game loaded it from.
            const uint32_t o = object & 0x1FFFFFFu;
            std::printf("[vp2s-models] class 0x154180 draw: object %x camera %x frame open %d mode %u lists %x %x counts %d %d\n", object, g_camera,
                        g_frameOpen ? 1 : 0, rdram[o + 0xAFFu], word(rdram, o + 0xAA0u), word(rdram, o + 0xAA4u),
                        static_cast<int32_t>(word(rdram, o + 0xAC0u)), static_cast<int32_t>(word(rdram, o + 0xAC4u)));
            uint32_t chunk = (word(rdram, o + 0xAA0u) + 0x20u) & 0x1FFFFFFu;
            const int32_t count = static_cast<int32_t>(word(rdram, o + 0xAC0u));
            for (int32_t i = 0; i < count && i < 4 && chunk >= 0x100000u; ++i)
            {
                std::string part;
                uint32_t offset = 0;
                const bool loaded = vp2sLoadedFrom(chunk, part, offset);
                std::printf("  chunk %d at %x (%u qw, from %s +%x):", i, chunk, word(rdram, chunk + 0x14u), loaded ? part.c_str() : "-", offset);
                for (uint32_t k = 0; k < 0x60u; k += 4u)
                    std::printf(" %08x", word(rdram, chunk + k));
                std::printf("\n");
                chunk = (chunk + 16u * word(rdram, chunk + 0x14u)) & 0x1FFFFFFu;
            }
            std::fflush(stdout);
        }
        static const bool s_noVu1 = [] { const char *v = std::getenv("VP2S_PARTICLES_NO_VU1"); return v && v[0] == '1'; }();
        if (!s_off && g_frameOpen && g_camera != 0u && drawParticles(rdram, runtime->memory().getScratchpad(), object))
        {
            ++g_counts.objects;
            if (s_noVu1)
            {
                ctx->pc = getRegU32(ctx, 31);
                return;
            }
        }
        g_modelLikeDraw(rdram, ctx, runtime);
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

void vp2sFitCameraScreen(float matrix[16])
{
    if (!g_rdram || g_camera < 0x100000u)
        return;
    // The camera's frame: around its centre (+0x244, +0x248), from its viewport's corner (+0x260, +0x264).
    const float cx = real(g_rdram, g_camera + 0x244u), cy = real(g_rdram, g_camera + 0x248u);
    const float w = 2.0f * (cx - real(g_rdram, g_camera + 0x260u)), h = 2.0f * (cy - real(g_rdram, g_camera + 0x264u));
    if (!(w > 16.0f && w < 4096.0f && h > 16.0f && h < 4096.0f))
        return;
    const float sx = 640.0f / w, sy = 448.0f / h;
    if (std::fabs(sx - 1.0f) < 1e-4f && std::fabs(sy - 1.0f) < 1e-4f)
        return;
    for (int c = 0; c < 4; ++c)
    {
        matrix[4 * c + 0] = sx * matrix[4 * c + 0] + (1.0f - sx) * cx * matrix[4 * c + 3];
        matrix[4 * c + 1] = sy * matrix[4 * c + 1] + (1.0f - sy) * cy * matrix[4 * c + 3];
    }
}

const unsigned char *vp2sEeRam()
{
    return g_rdram;
}

namespace
{
    // Where a post pass goes: before the interface once it has started this frame, else here (all drawn so far is the
    // 3D: on the title the passes run before any layer, and at the frame's end they covered its menu too, washing out
    // the cursor's swoosh and the copyright line).
    size_t postEffectAt()
    {
        return g_interfaceStart != SIZE_MAX ? g_interfaceStart : ps2x::native::Renderer::instance().batchCount();
    }
}

// The depth of field's pass this frame (its index in the renderer's frame, the post object): its weight table
// (+0xEF10) is read again as the frame ends. On a camera cut the game is still building it when its object's method
// returns (a pass with everything blurred for one frame); the PS2 sends it to the GS later in the frame, finished.
struct DofPass
{
    size_t index = SIZE_MAX;
    uint32_t post = 0;
};
static DofPass g_dofPass;

// The depth of field's blocks as the game drew them this frame (0x2CD020, one call per enabled block, in its order):
// each block's own weight table (+0xEF10 holds only the last one built) and its passes' indices, so every block is
// drawn with its table, refreshed at the frame's end (a camera cut finishes them later in the frame).
struct DofBlock
{
    uint32_t block = 0;
    uint8_t weights[256] = {};
    size_t pass = SIZE_MAX;
    float weightScale = 1.0f; // 2c / 128: each weight times it, at most 128 (the game's alpha step)
};

// A block's table as its pass weighs pixels: each weight times 2c / 128, at most 128.
static void scaledWeights(const DofBlock &b, uint8_t out[256])
{
    for (int k = 0; k < 256; ++k)
        out[k] = static_cast<uint8_t>(std::min(128.0f, b.weights[k] * b.weightScale + 0.5f));
}
static std::vector<DofBlock> g_dofBlocks;
static float g_dofGain = 0.0f;        // the game's g for its blurred copy (+0x131E0) of this frame's passes
static float g_brightnessGain = 0.0f; // the brightness pass's gain this frame (0: none)
static size_t g_glowPass = SIZE_MAX;  // this frame's glow pass (its gain set at the frame's end)

// The scene's screen objects (a fade, the pause's frozen frame: vp2sUiSceneScreens) go between the 3D with its effect
// passes and the interface: drawn once per frame where the interface starts, else at the frame's end.
static bool g_screensDrawn = false;
static void sceneScreens()
{
    if (g_screensDrawn)
        return;
    g_screensDrawn = true;
    if (g_rdram && g_scene >= 0x100000u)
        vp2sUiSceneScreens(g_rdram, g_scene);
}

void vp2sModelsInterfaceStarts()
{
    if (g_frameOpen && g_interfaceStart == SIZE_MAX)
    {
        g_interfaceStart = ps2x::native::Renderer::instance().batchCount();
        sceneScreens();
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
    sceneScreens();
    g_screensDrawn = false;
    if (g_dofPass.index != SIZE_MAX && g_rdram)
        if (auto *pass = ps2x::native::Renderer::instance().adjustPostEffect(g_dofPass.index))
            for (uint32_t k = 0; k < 256u; ++k)
                pass->weights[k] = g_rdram[g_dofPass.post + 0xEF10u + 4u * k];
    g_dofPass = {};
    // The game's g brings its blurred copy (made from the frame before its brightness pass) to the finished frame's
    // level; the native copy is made after that pass: g over the brightness gain.
    for (const DofBlock &b : g_dofBlocks)
        if (auto *pass = b.pass != SIZE_MAX ? ps2x::native::Renderer::instance().adjustPostEffect(b.pass) : nullptr)
        {
            scaledWeights(b, pass->weights);
            if (g_brightnessGain > 0.0f)
                pass->gain = g_dofGain / g_brightnessGain;
        }
    g_dofBlocks.clear();
    if (g_glowPass != SIZE_MAX && g_brightnessGain > 0.0f)
        if (auto *pass = ps2x::native::Renderer::instance().adjustPostEffect(g_glowPass))
            pass->gain = 1.0f / g_brightnessGain;
    g_glowPass = SIZE_MAX;
    g_brightnessGain = 0.0f;
    g_interfaceStart = SIZE_MAX;
    g_brightnessAt = SIZE_MAX;
    g_frameModelIndex = 0;
    {
        static uint64_t s_orderFrames = 0;
        g_orderFrame = ++s_orderFrames % 600u == 300u;
    }
    g_sceneOverlaysDrawn = false;
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
    vp2sBattleHudDraw(); // the battle HUD's widgets, over everything
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
    // Untextured draws at the GS's colour (0x80 = 128): the title's white backdrop (object 0x9A5A60, vertex 0x80) was
    // twice as bright, saturated by the brightness pass, and hid the cursor's swoosh and the copyright line.
    ps2x::native::Renderer::instance().setGsUntexturedColour(true);
    // The native models: drawn after the game's own draw method (whose packets go nowhere: no GS), one native frame a
    // game frame (vp2sModelsFrameEnd). VP2S_GS=1 (the old GS path, to compare) leaves them out.
    if (const char *gs = std::getenv("VP2S_GS"); gs && gs[0] == '1')
    {
        // VP2S_POST_SKIP=<hex address>[,...] with the GS comparison path (diagnosis): those mod001 methods skipped (the
        // effect objects' per-frame methods: 0x2D9360 depth of field, 0x2DBDE0, 0x2E0BA0), to see what the original's
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
    g_modelLikeDraw = module->table[(kModelLikeDraw - module->tableBase) >> 2];
    if (g_modelLikeDraw)
        ps2_overlays::replaceModuleFunction("mod001", kModelLikeDraw, &modelLikeDrawHook);
    // An object's shadow (FUN_002B6640, a0 the object: its packet for VU1 program 0x06f0, dropped with VIF1): drawn
    // natively as a shadow volume from its meshes, with the camera of the models drawn last. VP2S_SHADOWS=0: none.
    constexpr uint32_t kShadowDraw = 0x002B6640u;
    static PS2Runtime::RecompiledFunction s_shadowDraw =
        kShadowDraw >= module->tableBase && kShadowDraw < module->tableEnd ? module->table[(kShadowDraw - module->tableBase) >> 2] : nullptr;
    if (s_shadowDraw && !(std::getenv("VP2S_SHADOWS") && std::getenv("VP2S_SHADOWS")[0] == '0'))
        ps2_overlays::replaceModuleFunction("mod001", kShadowDraw, [](uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime) {
            const uint32_t object = getRegU32(ctx, 4);
            s_shadowDraw(rdram, ctx, runtime);
            if (g_frameOpen && g_camera >= 0x100000u)
                drawObject(rdram, runtime->memory().getScratchpad(), object, g_camera, true);
        });
    // The native frame ends with the game's scene pass (mod001 FUN_002211E0, every frame on every screen: the 3D,
    // then the interface layers): the field's frame wait alone left the title and the menus' frames unended.
    constexpr uint32_t kScenePass = 0x00221260u;
    if (kScenePass >= module->tableBase && kScenePass < module->tableEnd)
    {
        static PS2Runtime::RecompiledFunction s_scenePass = module->table[(kScenePass - module->tableBase) >> 2];
        if (s_scenePass)
            ps2_overlays::replaceModuleFunction("mod001", kScenePass, [](uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime) {
                g_rdram = rdram;
                g_scene = getRegU32(ctx, 4) & 0x1FFFFFFu;
                s_scenePass(rdram, ctx, runtime);
                vp2sModelsFrameEnd();
            });
    }
    // A depth of field block drawn (0x2CD020 USA, 0x2CCF60 EU: a0 the post object, a2 the block): its table, as the
    // game has just built it at +0xEF10 (a | v << 8 | v << 16 | a << 24: v the blurred copy's weight, 128 = all).
    constexpr uint32_t kDofBlock = 0x002CD020u;
    if (kDofBlock >= module->tableBase && kDofBlock < module->tableEnd)
    {
        static PS2Runtime::RecompiledFunction s_block = module->table[(kDofBlock - module->tableBase) >> 2];
        if (s_block)
            ps2_overlays::replaceModuleFunction("mod001", kDofBlock, [](uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime) {
                const uint32_t post = getRegU32(ctx, 4) & 0x1FFFFFFu, block = getRegU32(ctx, 6) & 0x1FFFFFFu;
                s_block(rdram, ctx, runtime);
                if (post < 0x100000u || block < 0x100000u)
                    return;
                auto it = std::find_if(g_dofBlocks.begin(), g_dofBlocks.end(), [&](const DofBlock &b) { return b.block == block; });
                if (it == g_dofBlocks.end())
                {
                    g_dofBlocks.push_back({});
                    it = std::prev(g_dofBlocks.end());
                    it->block = block;
                }
                for (uint32_t k = 0; k < 256u; ++k)
                    it->weights[k] = rdram[post + 0xEF10u + 4u * k + 1u];
            });
    }
    // The post effect (docs/vp2_dof.md): where the game runs its object's per-frame method (0x2D9360, between the 3D
    // and the interface), a native pass from its parameter blocks: each enabled block with the sky mode (+0x1F 0,
    // +0x20 & 3 == 1: drawn where the depth is the cleared one) puts a blurred copy of the frame brightened by g
    // (+0x131E0, 2 when unset) there, Gaussian sigma 0.555 x +0x24 (128-wide pixels), mixed by min(128, 2c) / 128 x
    // +0x1E / 128 (the table's weight at the farthest depth is 128 with p0..p4 0). The strongest block is drawn.
    // VP2S_POST=0: none.
    constexpr uint32_t kPostEffect = 0x002D9360u;
    if (kPostEffect >= module->tableBase && kPostEffect < module->tableEnd &&
        !(std::getenv("VP2S_POST") && std::getenv("VP2S_POST")[0] == '0'))
    {
        static PS2Runtime::RecompiledFunction s_post = module->table[(kPostEffect - module->tableBase) >> 2];
        if (s_post)
            ps2_overlays::replaceModuleFunction("mod001", kPostEffect, [](uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime) {
                s_post(rdram, ctx, runtime);
                sceneOverlays();
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
                if (!(g >= 0.25f && g < 8.0f)) // a per-frame field: between frames a stale word (a denormal seen) is no gain
                    g = 2.0f;
                // The glow (FUN_002CC570, after the blurred copies): on with +0xDFBC and +0xEF09, or +0x130E0 and +0xEF0A.
                // The frame before the brightness pass, shrunk to 128 x 88 less 25 / 255, at 64 / 128 (Cs x As + Cs, the
                // frame's alpha about 0), as three levels (each half the last) blurred, added at 76 / 128 each x +0xEF08 /
                // 128 (values of the PS2's passes, PCSX2 GS dumps of the town and the harbour). Put first: it goes after the blurred copies put at the same batch. Its
                // gain (back to the frame before the brightness pass) is set at the frame's end. VP2S_GLOW=0: none.
                static const bool s_glowOff = [] { const char *v = std::getenv("VP2S_GLOW"); return v && v[0] == '0'; }();
                const uint32_t at = post + 0x10000u;
                if (!s_glowOff && ((rdram[at - 0x2044u] && rdram[at - 0x10F7u]) || (rdram[at + 0x30E0u] && rdram[at - 0x10F6u])))
                {
                    ps2x::native::Renderer::PostEffect e;
                    e.glow = true;
                    e.width = 128;
                    e.height = 88;
                    e.gain = 0.5f;
                    e.sigma = 1.5f;
                    e.glowThreshold = 25.0f / 255.0f;
                    e.glowLevelCount = 3;
                    e.glowLevelWeight = 0.5f * 76.0f / 128.0f;
                    // Its glare: the frame less (25 + 77) / 255, copied rotated four times (about +-70 degrees), each
                    // smeared along one axis by two shifted self-additions (x 0.96, x 0.22) and added back at 152 / 128:
                    // lines at 70 and 110 degrees here, two texels long, 1.5 each (0.3 x 2.2 x 1.19 a copy, four copies on two
                    // lines, the frame's alpha about 0 as for the glow). VP2S_GLARE=<weight>: its strength.
                    static const float s_glare = [] { const char *v = std::getenv("VP2S_GLARE"); return v ? static_cast<float>(std::atof(v)) : 1.5f; }();
                    e.glareAngleCount = 2;
                    e.glareAngles[0] = 70.0f;
                    e.glareAngles[1] = 110.0f;
                    e.glareThreshold = 102.0f / 255.0f;
                    e.glareLength = 2.0f;
                    e.glareWeight = s_glare;
                    e.alpha = rdram[at - 0x10F8u] / 128.0f;
                    if (e.alpha > 0.0f)
                        g_glowPass = ps2x::native::Renderer::instance().postEffect(e, postEffectAt());
                }
                // Every block the game drew this frame, in its order, each with its own table: alpha min(128, 2c) / 128
                // x +0x1E / 128 (c moved toward 128 by s, +0x34), and with +0x20 & 3 == 1 only where the frame's Z is at
                // most +0x38 (the game's Z test against its sprite's Z: 0 the cleared background). VP2S_DOF_BLOCKS=0:
                // the strongest block only, as before.
                static const bool s_blocks = [] { const char *v = std::getenv("VP2S_DOF_BLOCKS"); return !(v && v[0] == '0'); }();
                if (s_blocks && !g_dofBlocks.empty())
                {
                    g_dofGain = g;
                    // Passes put at one batch go before the earlier ones there: the last block first keeps the game's order.
                    for (auto at = g_dofBlocks.rbegin(); at != g_dofBlocks.rend(); ++at)
                    {
                        DofBlock &b = *at;
                        const uint32_t block = b.block;
                        if (!word(rdram, block) || rdram[block + 0x1Fu] != 0u)
                            continue;
                        ps2x::native::Renderer::PostEffect e;
                        // The colour blurred at the block's size L18 (s16 +0x18: 128, 91 or 64 wide, by the work maps'
                        // 128 x 89 ratio), Gaussian sigma 0.555 x +0x24 texels there.
                        const int l18 = std::clamp<int>(static_cast<int16_t>(word(rdram, block + 0x18u) & 0xFFFFu), 16, 256);
                        e.width = l18;
                        e.height = std::max(8, (l18 * 89 + 64) / 128);
                        e.gain = g;
                        e.sigma = 0.555f * std::max(real(block + 0x24u), 0.0f);
                        const float s = std::clamp(real(block + 0x34u), 0.0f, 1.0f);
                        const float c = rdram[block + 0x1Cu] + (128.0f - rdram[block + 0x1Cu]) * s;
                        // Each pixel's alpha: its weight times 2c / 128, at most 128 (c 0xB6 saturates the focus ramps),
                        // times +0x1E / 128.
                        b.weightScale = 2.0f * c / 128.0f;
                        e.alpha = rdram[block + 0x1Eu] / 128.0f;
                        e.depthWeights = true;
                        e.weightShift = 16; // the table's index: Z24 >> 16 (+0x131C4, the depth scale, 65536)
                        scaledWeights(b, e.weights);
                        if ((rdram[block + 0x20u] & 3u) == 1u)
                        {
                            e.depthLimit = true;
                            e.depthFrom = 1.0f - static_cast<float>(word(rdram, block + 0x38u) & 0xFFFFFFu) / 16777216.0f;
                        }
                        if (e.alpha > 0.0f)
                            b.pass = ps2x::native::Renderer::instance().postEffect(e, postEffectAt());
                    }
                    return;
                }
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
                    // The other modes: the table the game built (+0xEF10, a | v << 8 | v << 16 | a << 24, 128 = all
                    // blurred) weighs each pixel by its depth (Z24 >> 16), as the game's own mix does. A ramp (Logun's
                    // interior: 4 .. 110 / 128 with the distance) is not a step at p0. VP2S_POST_TABLE=0: the step.
                    static const bool s_table = [] { const char *v = std::getenv("VP2S_POST_TABLE"); return !(v && v[0] == '0'); }();
                    if (s_table && (rdram[block + 0x20u] & 3u) != 1u)
                    {
                        e.depthWeights = true;
                        e.weightShift = 16;
                        for (uint32_t k = 0; k < 256u; ++k)
                            e.weights[k] = rdram[post + 0xEF10u + 4u * k];
                    }
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
                    // The scattering object (owner + 0x4450, docs/vp2_dof.md): enable +0x18980 bit 0, n +0x18951, Esun
                    // +0x13460, sun +0x13440, its table +0x134C4 (R, G, B in-scatter, A transmittance).
                    {
                        const uint32_t sc = word(rdram, owner + 0x4450u) & 0x1FFFFFFu;
                        if (sc >= 0x100000u)
                        {
                            const auto f = [&](uint32_t o) { float v; std::memcpy(&v, rdram + sc + o, 4); return v; };
                            std::printf("[vp2s-post] scattering %x: flags %02x n %u Esun %.3f %.3f %.3f sun %.2f %.2f %.2f; T[0] %08x T[8] %08x "
                                        "T[16] %08x T[32] %08x T[64] %08x T[255] %08x\n", sc, rdram[sc + 0x18980u], rdram[sc + 0x18951u],
                                        f(0x13460u), f(0x13464u), f(0x13468u), f(0x13440u), f(0x13444u), f(0x13448u),
                                        word(rdram, sc + 0x134C4u), word(rdram, sc + 0x134C4u + 32u), word(rdram, sc + 0x134C4u + 64u),
                                        word(rdram, sc + 0x134C4u + 128u), word(rdram, sc + 0x134C4u + 256u), word(rdram, sc + 0x134C4u + 1020u));
                        }
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
                {
                    const size_t index = ps2x::native::Renderer::instance().postEffect(best, postEffectAt());
                    if (best.depthWeights)
                        g_dofPass = {index, post};
                }
            });
    }
    // The frame's brightness (the object at *(0x35DDF8) + 0x4444, per-frame method 0x2CB4A0, packets by 0x2C9100):
    // in its mode 1 every pixel becomes clamp(g x max(0, min(F, L0) - L1)), L0 +0xAC, L1 +0xB0 (u32 0x00BBGGRR), g the
    // RGB of +0xC0 (2.0: the game draws at half intensity; the camera's exposure sets it). Modes 2 / 3 add tone bands
    // (+0xB4 / +0xB8, gains +0xD0 / +0xE0), not drawn yet: mode 1's formula is used. VP2S_BRIGHTNESS=0: none.
    // Its place in the frame: where the scene's draw loop (0x221040) draws the object (vtable 0x159D10 slot 6,
    // 0x2CB470), in the scene list's order. Its per-frame method runs after the whole loop, and inserting there put
    // the title's front objects (the cursor's swoosh, the copyright line) under it, twice as bright as on the console.
    constexpr uint32_t kBrightnessDraw = 0x002CB470u;
    if (kBrightnessDraw >= module->tableBase && kBrightnessDraw < module->tableEnd)
    {
        static PS2Runtime::RecompiledFunction s_brightnessDraw = module->table[(kBrightnessDraw - module->tableBase) >> 2];
        if (s_brightnessDraw)
            ps2_overlays::replaceModuleFunction("mod001", kBrightnessDraw, [](uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime) {
                g_brightnessAt = ps2x::native::Renderer::instance().batchCount();
                if (g_orderFrame && std::getenv("VP2S_DRAW_ORDER_LOG"))
                    std::printf("[vp2s-order] brightness drawn at batch %zu\n", g_brightnessAt);
                s_brightnessDraw(rdram, ctx, runtime);
            });
    }
    constexpr uint32_t kBrightness = 0x002CB4A0u;
    if (kBrightness >= module->tableBase && kBrightness < module->tableEnd &&
        !(std::getenv("VP2S_BRIGHTNESS") && std::getenv("VP2S_BRIGHTNESS")[0] == '0'))
    {
        static PS2Runtime::RecompiledFunction s_brightness = module->table[(kBrightness - module->tableBase) >> 2];
        if (s_brightness)
            ps2_overlays::replaceModuleFunction("mod001", kBrightness, [](uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime) {
                const uint32_t object = getRegU32(ctx, 4) & 0x1FFFFFFu;
                s_brightness(rdram, ctx, runtime);
                sceneOverlays();
                if (object < 0x100000u || !rdram[object + 0xA4u])
                    return;
                ps2x::native::Renderer::PostEffect e;
                e.levels = true;
                const uint32_t high = word(rdram, object + 0xACu), low = word(rdram, object + 0xB0u);
                for (int c = 0; c < 3; ++c)
                {
                    float g;
                    std::memcpy(&g, rdram + object + 0xC0u + 4u * static_cast<uint32_t>(c), 4);
                    e.levelGain[c] = std::clamp(g, 0.0f, 4.0f);
                    e.levelHigh[c] = static_cast<float>((high >> (8 * c)) & 0xFFu) / 255.0f;
                    e.levelLow[c] = static_cast<float>((low >> (8 * c)) & 0xFFu) / 255.0f;
                }
                static const bool s_log = std::getenv("VP2S_POST_LOG") != nullptr;
                static uint32_t s_calls = 0;
                if (s_log && ++s_calls % 300u == 1u)
                {
                    std::printf("[vp2s-post] brightness: mode %u, high %06x low %06x gain %.2f %.2f %.2f; batches %zu\n", rdram[object + 0xA8u],
                                high & 0xFFFFFFu, low & 0xFFFFFFu, e.levelGain[0], e.levelGain[1], e.levelGain[2],
                                ps2x::native::Renderer::instance().batchCount());
                    std::fflush(stdout);
                }
                if (g_orderFrame && std::getenv("VP2S_DRAW_ORDER_LOG"))
                    std::printf("[vp2s-order] brightness pass at batch %zu\n", postEffectAt());
                g_brightnessGain = (e.levelGain[0] + e.levelGain[1] + e.levelGain[2]) / 3.0f;
                ps2x::native::Renderer::instance().postEffect(e, g_brightnessAt != SIZE_MAX ? g_brightnessAt : postEffectAt());
            });
    }
    // The aerial perspective (the object at *(0x35DDF8) + 0x4450, per-frame method 0x2E0BA0: the field's orange haze,
    // docs/vp2_dof.md): when enabled (+0x18980 bit 0), its finished per-depth table (+0x134C4), index shift 16 - n
    // (+0x18951) and the in-scatter colour on a grid (scatterGrid) become a native scatter pass. VP2S_SCATTER=0: none.
    constexpr uint32_t kScatter = 0x002E0BA0u;
    if (kScatter >= module->tableBase && kScatter < module->tableEnd &&
        !(std::getenv("VP2S_SCATTER") && std::getenv("VP2S_SCATTER")[0] == '0'))
    {
        static PS2Runtime::RecompiledFunction s_scatter = module->table[(kScatter - module->tableBase) >> 2];
        if (s_scatter)
            ps2_overlays::replaceModuleFunction("mod001", kScatter, [](uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime) {
                const uint32_t sc = getRegU32(ctx, 4) & 0x1FFFFFFu;
                s_scatter(rdram, ctx, runtime);
                sceneOverlays();
                if (sc < 0x100000u || !(rdram[sc + 0x18980u] & 1u))
                    return;
                ps2x::native::Renderer::PostEffect e;
                e.scatter = true;
                e.scatterShift = 16 - std::min<int>(rdram[sc + 0x18951u], 16);
                std::memcpy(e.scatterTable, rdram + sc + 0x134C4u, sizeof(e.scatterTable));
                e.gridWidth = 17;
                e.gridHeight = 15;
                if (!scatterGrid(rdram, sc, g_camera, e.gridWidth, e.gridHeight, e.scatterGrid))
                    return;
                static const bool s_log = std::getenv("VP2S_POST_LOG") != nullptr;
                static uint32_t s_calls = 0;
                if (s_log && ++s_calls % 300u == 1u)
                {
                    std::printf("[vp2s-post] scatter: shift %d, V corners %08x %08x %08x %08x centre %08x; batches %zu\n", e.scatterShift,
                                e.scatterGrid[0], e.scatterGrid[16], e.scatterGrid[14 * 17], e.scatterGrid[14 * 17 + 16], e.scatterGrid[7 * 17 + 8],
                                ps2x::native::Renderer::instance().batchCount());
                    // The microcode's constants (*(+0x13AE4) + 0x70, qwords 0..13) as floats.
                    const uint32_t vu = word(rdram, sc + 0x13AE4u) & 0x1FFFFFFu;
                    for (uint32_t q = 0; vu >= 0x100000u && q < 14u; ++q)
                    {
                        float f[4];
                        std::memcpy(f, rdram + vu + 0x70u + 16u * q, 16);
                        std::printf("[vp2s-post] scatter VU %u: %g %g %g %g\n", q, f[0], f[1], f[2], f[3]);
                    }
                    std::fflush(stdout);
                }
                ps2x::native::Renderer::instance().postEffect(e, postEffectAt());
            });
    }
    ps2x::native::Renderer::instance().setStreamDraws(false);
    ps2x::native::Renderer::instance().setEnabled(true);
    ps2x::native::Renderer::instance().beginFrame(nativeScreen());
    g_frameOpen = true;
    std::printf("[vp2s] native models from the game's data and the offline mesh conversion (%s)\n", directory().c_str());
}

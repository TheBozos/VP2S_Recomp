// VP2's field minimap, drawn natively from the area map object (mod003 class 0x165470) instead of the packets its
// polygon batcher sends to VU1 (docs/vp2_2d.md, "Field minimap").
//
// The map object M = *(*(*(0x48DB48) + 8) + 0xC8); its per-frame update 0x440FA0 (vtable 0x165470 + 0x14) builds the
// minimap every frame (0x457BE0) when M+0x1683 bit 1. Hooked here after it runs: the picture is made from M's data.
#include "vp2s_minimap.h"

#include "ps2_overlay_modules.h"
#include "ps2_runtime.h"
#include "runtime/native/native_renderer.h"
#include "vp2s_textures.h"

#include <algorithm>
#include <array>
#include <memory>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace
{
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

    constexpr uint32_t kMapUpdate = 0x00440FA0u;
    PS2Runtime::RecompiledFunction g_mapUpdate = nullptr;

    // VP2S_MINIMAP_LOG: now and then, the map object's fields the native minimap reads.
    void logMap(const uint8_t *rdram, uint32_t m)
    {
        const uint32_t map = word(rdram, m + 0x4C0u) & 0x1FFFFFFu, rooms = word(rdram, m + 0x1644u) & 0x1FFFFFFu;
        const uint32_t x = word(rdram, m + 0x1640u) & 0x1FFFFFFu, scene = word(rdram, m + 0x1630u) & 0x1FFFFFFu;
        const uint32_t camera = word(rdram, m + 0x1638u) & 0x1FFFFFFu, image = word(rdram, m + 0x29Cu) & 0x1FFFFFFu;
        std::printf("[vp2s-minimap] M %x flags %02x alpha %.1f layer %d room %x map %x (rooms %u layers %u) records %x; X %x (%.0f, %.0f, a %.2f "
                    "t %.2f); scene %x t (%.1f %.1f %.1f); camera %x; image %x (%08x %08x %08x %x)\n",
                    m, rdram[m + 0x1683u], real(rdram, m + 0x15B0u), static_cast<int8_t>(rdram[m + 0x15BEu]), word(rdram, m + 0x4C8u), map,
                    map ? word(rdram, map + 4u) : 0u, map ? static_cast<uint32_t>(*reinterpret_cast<const uint16_t *>(rdram + map + 6u)) : 0u,
                    rooms, x, x ? real(rdram, x + 0x94u) : 0.0f, x ? real(rdram, x + 0x98u) : 0.0f, x ? real(rdram, x + 0xB0u) : 0.0f,
                    x ? real(rdram, x + 0xB4u) : 0.0f, scene, scene ? real(rdram, scene + 0x9Cu) : 0.0f,
                    scene ? real(rdram, scene + 0xACu) : 0.0f, scene ? real(rdram, scene + 0xBCu) : 0.0f, camera, image,
                    image ? word(rdram, image) : 0u, image ? word(rdram, image + 0x14u) : 0u, image ? word(rdram, image + 0x18u) : 0u,
                    image ? word(rdram, image + 0x10u) : 0u);
        const uint32_t scissor = word(rdram, m + 0x1650u) & 0x1FFFFFFu;
        if (scissor >= 0x100000u && camera >= 0x100000u)
            std::printf("[vp2s-minimap]   scissor %d %d %d %d; camera centre %.1f %.1f viewport %.0f %.0f %.0f %.0f fov %.3f aspect %.3f; "
                        "X words %08x %08x %08x %08x\n", static_cast<int32_t>(word(rdram, scissor + 0xA4u)),
                        static_cast<int32_t>(word(rdram, scissor + 0xA8u)), static_cast<int32_t>(word(rdram, scissor + 0xACu)),
                        static_cast<int32_t>(word(rdram, scissor + 0xB0u)), real(rdram, camera + 0x244u), real(rdram, camera + 0x248u),
                        real(rdram, camera + 0x260u), real(rdram, camera + 0x264u), real(rdram, camera + 0x268u), real(rdram, camera + 0x26Cu),
                        real(rdram, camera + 0x224u), real(rdram, camera + 0x238u), word(rdram, x + 0x94u), word(rdram, x + 0x98u),
                        word(rdram, x + 0xB0u), word(rdram, x + 0xB4u));
        for (uint32_t r = 0; rooms >= 0x100000u && map && r < std::min<uint32_t>(word(rdram, map + 4u), 4u); ++r)
        {
            const uint32_t rec = rooms + 0xF0u * r, def = word(rdram, rec + 0xE0u) & 0x1FFFFFFu;
            std::printf("[vp2s-minimap]   room %u def %x layer byte %02x colour %u: p0 (%.0f %.0f %.0f) p3 (%.0f %.0f %.0f) uv %08x %08x %08x %08x\n", r, def,
                        def ? rdram[def + 3u] : 0u, rdram[rec + 0xE5u], real(rdram, rec + 0x50u), real(rdram, rec + 0x54u), real(rdram, rec + 0x58u),
                        real(rdram, rec + 0x80u), real(rdram, rec + 0x84u), real(rdram, rec + 0x88u), word(rdram, rec + 0xD0u),
                        word(rdram, rec + 0xD4u), word(rdram, rec + 0xD8u), word(rdram, rec + 0xDCu));
        }
        std::fflush(stdout);
    }

    const uint8_t *g_rdram = nullptr;
    uint32_t g_map = 0; // the map object the last update ran for (0: none this frame)

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

    // The minimap's 2D frame is its own context (origin X+0x94 / +0x98: 1792, 1808); the native screen is the 3D
    // context's (1728, 1824): its pixels moved there.
    constexpr float kOriginX = 1728.0f, kOriginY = 1824.0f;

    uint32_t rgb(float r, float g, float b, float a)
    {
        const auto c = [](float v) { return static_cast<uint32_t>(std::clamp(v, 0.0f, 255.0f)); };
        return c(r) | c(g) << 8 | c(b) << 16 | c(a) << 24;
    }

    void mapUpdateHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t m = getRegU32(ctx, 4) & 0x1FFFFFFu;
        g_mapUpdate(rdram, ctx, runtime);
        g_rdram = rdram;
        g_map = m;
        static const bool s_log = std::getenv("VP2S_MINIMAP_LOG") != nullptr;
        static uint32_t s_calls = 0;
        if (s_log && ++s_calls % 300u == 1u)
            logMap(rdram, m);
    }
}

void vp2sInstallMinimap(PS2Runtime &)
{
    if (const char *gs = std::getenv("VP2S_GS"); gs && gs[0] == '1')
        return;
    const ps2_overlays::Module *module = ps2_overlays::findModule("mod003");
    if (!module || kMapUpdate < module->tableBase || kMapUpdate >= module->tableEnd)
    {
        std::printf("[vp2s] native minimap: mod003 %s\n", module ? "does not hold the map update" : "not found");
        return;
    }
    g_mapUpdate = module->table[(kMapUpdate - module->tableBase) >> 2];
    if (g_mapUpdate)
        ps2_overlays::replaceModuleFunction("mod003", kMapUpdate, &mapUpdateHook);
}

// The minimap as the map object makes it (minimap mode: M+0x1683 bit 1 built, bit 2 clear), from M's data:
// - the background: X = *(M+0x1640), a gouraud rectangle (top 0x4D81B7, bottom 0x26547A, alpha X+0xB0) over its
//   frame's pixels 24..180 x 24..140 from (X+0x94, X+0x98), then a 2-pixel border 0x101028 at 0.75 x that alpha;
// - the player's layer (M+0x15BE) of rooms: records *(M+0x1644), 0xF0 bytes each, count u16 map+4 (map M+0x4C0): a
//   textured strip of +0x50 / +0x60 (top) / +0x70 / +0x80 (bottom), UVs +0xD0..+0xDC (s | t << 16, 4.12), the floor
//   image *(M+0x29C) (an FIS: key +0x14 / +0x18), colour by +0xE5 ({128,104,64}, {128,80,64}, {64,64,64} x (1 -
//   layer / 12), the bottom x 0.85), alpha 0.85 x M+0x15B0; the room's layer the signed low nibble of *(+0xE0) + 3;
// - through the scene object *(M+0x1630) (+0x90 world) and the camera *(M+0x1638) (+0x2F0 view, +0x330 projection),
//   scissored to the frame's pixels 24..179 x 24..139.
// Not drawn yet: unvisited rooms are drawn too, doors / stairs, icons, markers, the player's marker, the camera
// footprint, the full map.
void vp2sMinimapDraw()
{
    const uint8_t *rdram = g_rdram;
    const uint32_t m = g_map;
    g_map = 0;
    if (!rdram || m < 0x100000u)
        return;
    const uint8_t flags = rdram[m + 0x1683u];
    const float fade = real(rdram, m + 0x15B0u);
    if (!(flags & 2u) || (flags & 4u) || fade <= 0.0f)
        return;
    const uint32_t x = word(rdram, m + 0x1640u) & 0x1FFFFFFu, scene = word(rdram, m + 0x1630u) & 0x1FFFFFFu;
    const uint32_t camera = word(rdram, m + 0x1638u) & 0x1FFFFFFu, map = word(rdram, m + 0x4C0u) & 0x1FFFFFFu;
    if (x < 0x100000u || scene < 0x100000u || camera < 0x100000u || map < 0x100000u)
        return;
    const float frameX = static_cast<float>(word(rdram, x + 0x94u)), frameY = static_cast<float>(word(rdram, x + 0x98u));
    const float shiftX = kOriginX - frameX, shiftY = kOriginY - frameY; // the map's frame pixels to the native screen's
    auto &renderer = ps2x::native::Renderer::instance();
    static const float s_identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    ps2x::native::Material flat;
    flat.blend = ps2x::native::Material::Blend::Alpha;
    flat.alphaBlend = true;
    flat.depthTest = false;
    flat.depthWrite = false;
    flat.textured = false;
    const auto vertex = [&](float vx, float vy, uint32_t colour) {
        ps2x::native::Vertex v{};
        v.x = kOriginX + vx;
        v.y = kOriginY + vy;
        v.z = 16777215.0f;
        v.rgba = colour;
        return v;
    };
    // The background and its border (the frame's pixels, moved onto the native screen's).
    const float alpha = real(rdram, x + 0xB0u);
    {
        const float x0 = 24.0f, y0 = 24.0f, x1 = 180.0f, y1 = 140.0f;
        const uint32_t top = rgb(0xB7, 0x81, 0x4D, alpha), bottom = rgb(0x7A, 0x54, 0x26, alpha);
        const ps2x::native::Vertex quad[6] = {vertex(x0, y0, top), vertex(x1, y0, top), vertex(x0, y1, bottom),
                                              vertex(x1, y0, top), vertex(x1, y1, bottom), vertex(x0, y1, bottom)};
        renderer.draw(flat, s_identity, quad, 6u);
        const uint32_t edge = rgb(0x28, 0x10, 0x10, alpha * 0.75f);
        const float o = 2.0f;
        const ps2x::native::Vertex border[24] = {
            vertex(x0 - o, y0 - o, edge), vertex(x1 + o, y0 - o, edge), vertex(x0 - o, y0, edge),
            vertex(x1 + o, y0 - o, edge), vertex(x1 + o, y0, edge), vertex(x0 - o, y0, edge),
            vertex(x0 - o, y1, edge), vertex(x1 + o, y1, edge), vertex(x0 - o, y1 + o, edge),
            vertex(x1 + o, y1, edge), vertex(x1 + o, y1 + o, edge), vertex(x0 - o, y1 + o, edge),
            vertex(x0 - o, y0, edge), vertex(x0, y0, edge), vertex(x0 - o, y1, edge),
            vertex(x0, y0, edge), vertex(x0, y1, edge), vertex(x0 - o, y1, edge),
            vertex(x1, y0, edge), vertex(x1 + o, y0, edge), vertex(x1, y1, edge),
            vertex(x1 + o, y0, edge), vertex(x1 + o, y1, edge), vertex(x1, y1, edge)};
        renderer.draw(flat, s_identity, border, 24u);
    }
    // The rooms of the player's layer, through the map's camera, in the native screen's GS space.
    Mat4 s = product(rows(rdram, camera + 0x330u), product(rows(rdram, camera + 0x2F0u), rows(rdram, scene + 0x90u)));
    for (int c = 0; c < 4; ++c)
    {
        s[0][c] += shiftX * s[3][c];
        s[1][c] += shiftY * s[3][c];
    }
    float matrix[16];
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r)
            matrix[4 * c + r] = s[r][c];
    std::shared_ptr<const ps2x::native::TextureCache::Texture> texture;
    if (const uint32_t image = word(rdram, m + 0x29Cu) & 0x1FFFFFFu; image >= 0x100000u && word(rdram, image) == 0x00534946u)
        texture = vp2sGameTexture(word(rdram, image + 0x14u), word(rdram, image + 0x18u), 0u);
    ps2x::native::Material room = flat;
    room.textured = texture != nullptr;
    room.scissor[0] = kOriginX + 24.0f;
    room.scissor[1] = kOriginY + 24.0f;
    room.scissor[2] = kOriginX + 179.0f;
    room.scissor[3] = kOriginY + 139.0f;
    const int layer = static_cast<int8_t>(rdram[m + 0x15BEu]);
    const uint32_t records = word(rdram, m + 0x1644u) & 0x1FFFFFFu;
    const uint32_t count = std::min<uint32_t>(word(rdram, map + 4u) & 0xFFFFu, 256u);
    static const float s_colours[3][3] = {{128, 104, 64}, {128, 80, 64}, {64, 64, 64}};
    for (uint32_t i = 0; records >= 0x100000u && i < count; ++i)
    {
        const uint32_t record = records + 0xF0u * i, def = word(rdram, record + 0xE0u) & 0x1FFFFFFu;
        if (def < 0x100000u)
            continue;
        const int roomLayer = static_cast<int8_t>(static_cast<uint8_t>(rdram[def + 3u] << 4)) >> 4;
        if (roomLayer != layer)
            continue;
        const float *base = s_colours[std::min<uint32_t>(rdram[record + 0xE5u], 2u)];
        const float shade = 1.0f - static_cast<float>(roomLayer) / 12.0f;
        ps2x::native::Vertex corner[4];
        for (uint32_t k = 0; k < 4u; ++k)
        {
            const uint32_t p = record + 0x50u + 16u * k, uv = word(rdram, record + 0xD0u + 4u * k);
            const float f = shade * (k < 2u ? 1.0f : 0.85f);
            corner[k].x = real(rdram, p);
            corner[k].y = real(rdram, p + 4u);
            corner[k].z = real(rdram, p + 8u);
            corner[k].u = static_cast<float>(uv & 0xFFFFu) / 4096.0f;
            corner[k].v = static_cast<float>(uv >> 16) / 4096.0f;
            corner[k].rgba = rgb(base[0] * f, base[1] * f, base[2] * f, 0.85f * fade);
        }
        const ps2x::native::Vertex strip[6] = {corner[0], corner[1], corner[2], corner[1], corner[3], corner[2]};
        renderer.draw(room, matrix, strip, 6u, texture);
    }
}

// VP2's battle HUD: mod028 "2D widget" objects (class base 0x16FA60) that write their own GIF packets with three
// primitive emitters (docs/vp2_2d.md, "Battle HUD"). The native port hooks the emitters and the sheet setters,
// rebuilds each primitive from its descriptor in the widget's memory, and draws them natively.
#include "vp2s_battle_hud.h"

#include "ps2_overlay_modules.h"
#include "ps2_runtime.h"
#include "runtime/native/native_renderer.h"
#include "vp2s_textures.h"
#include "vp2s_ui.h"
#include "vp2s_loads.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

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

    // mod028: 0x42D740(W, d, snap) textured sprite, 0x42EAE0(W, d) quad, 0x42DB70(W, points, n, header) polygon; the
    // sheet setters 0x3CCE40(W, i) (the widget's own sheet), 0x4F2670 (the shared battle sheet), 0x4F2740 (portraits);
    // 0x3CCAE0(W) ends a widget's render (flips its packet buffers).
    constexpr uint32_t kSprite = 0x0042D740u, kQuad = 0x0042EAE0u, kPolygon = 0x0042DB70u, kOwnSheet = 0x003CCE40u,
                       kSharedSheet = 0x004F2670u, kPortrait = 0x004F2740u, kRenderEnd = 0x003CCAE0u;
    PS2Runtime::RecompiledFunction g_sprite = nullptr, g_quad = nullptr, g_polygon = nullptr, g_ownSheet = nullptr,
                                   g_sharedSheet = nullptr, g_portrait = nullptr, g_renderEnd = nullptr;

    // mod027 0x36B3A0(a0 image object: its 'FIS' file, a1, a2 PSM, a3 DBP of the pixels or 0, t0 DBP of the palette)
    // uploads the battle's fixed sheets (mod028 0x4F2670's TBP 0x800 among them): the DBP keyed to the FIS's key, as
    // the 2D layer does for 0x2F37B0.
    constexpr uint32_t kFixedUpload = 0x0036B3A0u;
    PS2Runtime::RecompiledFunction g_fixedUpload = nullptr;
    std::map<uint32_t, std::pair<uint32_t, uint32_t>> g_fixedCluts; // CBP -> the FIS key whose palette went there

    // VP2S_BATTLE_HUD_LOG: hit counts now and then, with the HUD's origin / scale (*0x36D780 [0], [1], [4], [5]) and
    // a first descriptor of each kind.
    uint64_t g_hits[7] = {};
    void count(int kind, const uint8_t *rdram, R5900Context *ctx)
    {
        static const bool s_log = std::getenv("VP2S_BATTLE_HUD_LOG") != nullptr;
        if (!s_log)
            return;
        ++g_hits[kind];
        static int s_shown[7] = {};
        if (s_shown[kind] < 3)
        {
            ++s_shown[kind];
            const uint32_t w = getRegU32(ctx, 4) & 0x1FFFFFFu, d = getRegU32(ctx, 5) & 0x1FFFFFFu;
            std::printf("[vp2s-hud] kind %d: W %x (vtable %x, origin %d %d scale %.3f %.3f) a1 %x a2 %x a3 %x:", kind, w, word(rdram, w),
                        static_cast<int32_t>(word(rdram, w + 0x98u)), static_cast<int32_t>(word(rdram, w + 0x9Cu)), real(rdram, w + 0xA8u),
                        real(rdram, w + 0xACu), getRegU32(ctx, 5), getRegU32(ctx, 6), getRegU32(ctx, 7));
            for (uint32_t k = 0; d >= 0x100000u && k < 0x80u; k += 4u)
                std::printf(" %08x", word(rdram, d + k));
            std::printf("\n");
            std::fflush(stdout);
        }
        static uint64_t s_total = 0;
        if (++s_total % 20000u == 1u)
        {
            std::printf("[vp2s-hud] hits sprite %llu quad %llu polygon %llu own sheet %llu shared sheet %llu portrait %llu render end %llu\n",
                        static_cast<unsigned long long>(g_hits[0]), static_cast<unsigned long long>(g_hits[1]),
                        static_cast<unsigned long long>(g_hits[2]), static_cast<unsigned long long>(g_hits[3]),
                        static_cast<unsigned long long>(g_hits[4]), static_cast<unsigned long long>(g_hits[5]),
                        static_cast<unsigned long long>(g_hits[6]));
            std::fflush(stdout);
        }
    }

    // A primitive as the widget emits it: a triangle strip of up to 8 vertices in native GS coordinates, its sheet by
    // the texture manager's key (tag 0: untextured).
    struct Primitive
    {
        std::array<ps2x::native::Vertex, 8> v{};
        uint32_t count = 0;
        uint32_t tag = 0, index = 0, csa = 0;
        uint32_t paletteTag = 0, paletteIndex = 0; // the palette's own key (a fixed sheet's CBP), or 0
    };
    struct Widget
    {
        std::vector<Primitive> building, shown;
        uint32_t tbp = 0, cbp = 0;           // the sheet its last TEX0 setter chose (a fixed slot's TBP, CBP), or
        uint32_t tag = 0, index = 0, csa = 0; // its key, when the setter names a texture manager entry
        uint64_t lastFrame = 0;
    };
    std::mutex g_mutex;
    std::map<uint32_t, Widget> g_widgets; // by the widget's address, drawn in that order
    uint32_t g_current = 0;               // the widget whose render runs (its own-sheet setter's or emitters' a0)
    uint64_t g_frame = 0;
    const uint8_t *g_scratch = nullptr;
    const uint8_t *g_rdram = nullptr; // EE memory (the draw's diagnostics)

    // The game's frame for these widgets is 512 x 480 at GS (1792, 1808); the native screen is 640 x 448 at (1728,
    // 1824): GS points moved and scaled onto it.
    float nativeX(float gs) { return 1728.0f + (gs - 1792.0f) * 1.25f; }
    float nativeY(float gs) { return 1824.0f + (gs - 1808.0f) * (448.0f / 480.0f); }

    // The resident texture at a TBP, from the game's texture manager (*0x35DE38 in the scratchpad: the list from the
    // byte at +0x2008, key +0x1008 / +0x8 + 16 e, TEX0 +0x1010 + 16 e).
    bool sheetAt(const uint8_t *rdram, uint32_t tbp, uint32_t &tag, uint32_t &index)
    {
        const uint32_t texman = word(rdram, 0x0035DE38u);
        if (!g_scratch || (texman & 0xF0000000u) != 0x70000000u)
            return false;
        const auto sw = [&](uint32_t o) {
            uint32_t v;
            std::memcpy(&v, g_scratch + ((texman + o) & 0x3FFFu), 4);
            return v;
        };
        uint32_t e = g_scratch[(texman + 0x2008u) & 0x3FFFu];
        for (int guard = 0; e != 0u && guard < 256; ++guard, e = g_scratch[(texman + 0x2008u + e) & 0x3FFFu])
            if ((sw(0x1010u + 16u * e) & 0x3FFFu) == tbp)
            {
                tag = sw(0x1008u + 16u * e);
                index = sw(0x8u + 16u * e);
                return true;
            }
        return false;
    }

    ps2x::native::Vertex vertex(float gx, float gy, float u, float v, uint32_t rgba)
    {
        ps2x::native::Vertex out{};
        out.x = nativeX(gx);
        out.y = nativeY(gy);
        out.z = 16777215.0f;
        out.u = u;
        out.v = v;
        out.rgba = rgba;
        return out;
    }

    void add(const uint8_t *rdram, uint32_t w, Primitive &&p, bool textured)
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        Widget &widget = g_widgets[w];
        if (textured && widget.tag != 0u)
        {
            p.tag = widget.tag;
            p.index = widget.index;
            p.csa = widget.csa;
        }
        else if (textured && (sheetAt(rdram, widget.tbp, p.tag, p.index) || vp2sUiFixedSheet(widget.tbp, p.tag, p.index)))
        {
            p.csa = widget.csa;
            if (const auto clut = g_fixedCluts.find(widget.cbp); clut != g_fixedCluts.end())
            {
                p.paletteTag = clut->second.first;
                p.paletteIndex = clut->second.second;
            }
        }
        else if (textured)
        {
            static const bool s_log = std::getenv("VP2S_BATTLE_HUD_LOG") != nullptr;
            static std::vector<uint32_t> s_missed;
            if (s_log && std::find(s_missed.begin(), s_missed.end(), widget.tbp) == s_missed.end() && s_missed.size() < 16u)
            {
                s_missed.push_back(widget.tbp);
                const uint32_t texman = word(rdram, 0x0035DE38u);
                std::printf("[vp2s-hud] widget %x: no resident texture at TBP %x; resident:", w, widget.tbp);
                if (g_scratch && (texman & 0xF0000000u) == 0x70000000u)
                {
                    const auto sw = [&](uint32_t o) {
                        uint32_t v;
                        std::memcpy(&v, g_scratch + ((texman + o) & 0x3FFFu), 4);
                        return v;
                    };
                    uint32_t e = g_scratch[(texman + 0x2008u) & 0x3FFFu];
                    for (int guard = 0; e != 0u && guard < 256; ++guard, e = g_scratch[(texman + 0x2008u + e) & 0x3FFFu])
                        std::printf(" %x:%08x/%x", sw(0x1010u + 16u * e) & 0x3FFFu, sw(0x1008u + 16u * e), sw(0x8u + 16u * e));
                }
                std::printf("\n");
                std::fflush(stdout);
            }
            return;
        }
        widget.building.push_back(std::move(p));
    }

    struct Frame
    {
        float ox, oy, scaleX, scaleY;
    };
    Frame frameOf(const uint8_t *rdram, uint32_t w)
    {
        return {static_cast<float>(static_cast<int32_t>(word(rdram, w + 0x98u))), static_cast<float>(static_cast<int32_t>(word(rdram, w + 0x9Cu))),
                real(rdram, w + 0xA8u), real(rdram, w + 0xACu) * 0.5f};
    }

    float half(const uint8_t *rdram, uint32_t address) { return static_cast<float>(word(rdram, address) & 0xFFFFu); }

    // 0x42D740(W, d, snap): a textured sprite. d +0x00 / +0x04 x, y, +0x10 / +0x14 scale, +0x18 / +0x1C pivot, +0x20
    // part (u0, v0, u1, v1 in 4.12, +8 w, +10 h), +0x24 RGBA, +0x28 / +0x2C w, h (no part, or flag bit 0), +0x30..+0x3C
    // UVs (flag bit 1), +0x40 flags; x0 = W+0x98 + W+0xA8 (x + px (w - w sx)), y0 = W+0x9C + W+0xAC / 2 (y + py (h -
    // h sy)), width w sx W+0xA8, height h sy W+0xAC / 2.
    void spriteHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        count(0, rdram, ctx);
        const uint32_t w = getRegU32(ctx, 4) & 0x1FFFFFFu, d = getRegU32(ctx, 5) & 0x1FFFFFFu;
        g_scratch = runtime->memory().getScratchpad();
        g_rdram = rdram;
        const uint32_t flags = word(rdram, d + 0x40u), part = word(rdram, d + 0x20u) & 0x1FFFFFFu;
        const bool havePart = part >= 0x100000u;
        const float pw = havePart && !(flags & 1u) ? half(rdram, part + 8u) : real(rdram, d + 0x28u);
        const float ph = havePart && !(flags & 1u) ? half(rdram, part + 10u) : real(rdram, d + 0x2Cu);
        float u0, v0, u1, v1;
        if ((flags & 2u) || !havePart)
        {
            u0 = real(rdram, d + 0x30u);
            v0 = real(rdram, d + 0x34u);
            u1 = real(rdram, d + 0x38u);
            v1 = real(rdram, d + 0x3Cu);
        }
        else
        {
            u0 = half(rdram, part) / 4096.0f;
            v0 = half(rdram, part + 2u) / 4096.0f;
            u1 = half(rdram, part + 4u) / 4096.0f;
            v1 = half(rdram, part + 6u) / 4096.0f;
        }
        const float sx = real(rdram, d + 0x10u), sy = real(rdram, d + 0x14u), px = real(rdram, d + 0x18u), py = real(rdram, d + 0x1Cu);
        const Frame f = frameOf(rdram, w);
        const float x0 = f.ox + f.scaleX * (real(rdram, d) + px * (pw - pw * sx));
        const float y0 = f.oy + f.scaleY * (real(rdram, d + 4u) + py * (ph - ph * sy));
        const float x1 = x0 + pw * sx * f.scaleX, y1 = y0 + ph * sy * f.scaleY;
        const uint32_t rgba = word(rdram, d + 0x24u);
        Primitive p;
        p.v[0] = vertex(x0, y0, u0, v0, rgba);
        p.v[1] = vertex(x1, y0, u1, v0, rgba);
        p.v[2] = vertex(x0, y1, u0, v1, rgba);
        p.v[3] = vertex(x1, y1, u1, v1, rgba);
        p.count = 4;
        add(rdram, w, std::move(p), true);
        g_current = w;
        g_sprite(rdram, ctx, runtime);
    }

    // 0x42EAE0(W, d): a quad (a 4-vertex strip) placed as the sprite, flags +0x6C (bit 3 textured, bit 2 gouraud), size
    // from the part (+0x68) when textured without flag bit 0, else +0x60 / +0x64; colours +0x30 (or +0x30..+0x3C).
    // Rotated quads (+0x2C) are not drawn yet.
    void quadHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        count(1, rdram, ctx);
        const uint32_t w = getRegU32(ctx, 4) & 0x1FFFFFFu, d = getRegU32(ctx, 5) & 0x1FFFFFFu;
        g_scratch = runtime->memory().getScratchpad();
        const uint32_t flags = rdram[(d + 0x6Cu) & 0x1FFFFFFu], part = word(rdram, d + 0x68u) & 0x1FFFFFFu;
        const bool textured = (flags & 8u) != 0u, gouraud = (flags & 4u) != 0u, havePart = part >= 0x100000u;
        if (real(rdram, d + 0x2Cu) == 0.0f)
        {
            const float pw = textured && havePart && !(flags & 1u) ? half(rdram, part + 8u) : real(rdram, d + 0x60u);
            const float ph = textured && havePart && !(flags & 1u) ? half(rdram, part + 10u) : real(rdram, d + 0x64u);
            const float sx = real(rdram, d + 0x10u), sy = real(rdram, d + 0x14u), px = real(rdram, d + 0x18u), py = real(rdram, d + 0x1Cu);
            const Frame f = frameOf(rdram, w);
            const float x0 = f.ox + f.scaleX * (real(rdram, d) + px * (pw - pw * sx));
            const float y0 = f.oy + f.scaleY * (real(rdram, d + 4u) + py * (ph - ph * sy));
            const float x1 = x0 + pw * sx * f.scaleX, y1 = y0 + ph * sy * f.scaleY;
            float u[4] = {0, 1, 0, 1}, v[4] = {0, 0, 1, 1};
            if (textured && havePart)
            {
                const float u0 = half(rdram, part) / 4096.0f, v0 = half(rdram, part + 2u) / 4096.0f;
                const float u1 = half(rdram, part + 4u) / 4096.0f, v1 = half(rdram, part + 6u) / 4096.0f;
                const float us[4] = {u0, u1, u0, u1}, vs[4] = {v0, v0, v1, v1};
                std::copy(us, us + 4, u);
                std::copy(vs, vs + 4, v);
            }
            Primitive p;
            const float xs[4] = {x0, x1, x0, x1}, ys[4] = {y0, y0, y1, y1};
            for (uint32_t k = 0; k < 4u; ++k)
                p.v[k] = vertex(xs[k], ys[k], u[k], v[k], word(rdram, d + 0x30u + (gouraud ? 4u * k : 0u)));
            p.count = 4;
            add(rdram, w, std::move(p), textured);
        }
        g_current = w;
        g_quad(rdram, ctx, runtime);
    }

    // 0x42DB70(W, d, n, packet): an n-point polygon (a strip): points (x, y) from d, its box's corner and size, scale
    // +0x20 / +0x24, pivot +0x28 / +0x2C, rotation +0x3C (0: none; rotated ones not drawn yet), colours +0x40 (one,
    // or one a vertex with flag bit 2), flags +0x78 (bit 3 textured: 4 points, UVs +0x50..+0x6C with flag bit 1 or no
    // part, else from the part at +0x70).
    void polygonHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        count(2, rdram, ctx);
        const uint32_t w = getRegU32(ctx, 4) & 0x1FFFFFFu, d = getRegU32(ctx, 5) & 0x1FFFFFFu;
        const uint32_t n = std::min<uint32_t>(getRegU32(ctx, 6), 8u);
        g_scratch = runtime->memory().getScratchpad();
        if (n >= 3u && real(rdram, d + 0x3Cu) == 0.0f)
        {
            float minX = real(rdram, d), maxX = minX, minY = real(rdram, d + 4u), maxY = minY;
            for (uint32_t i = 1; i < n; ++i)
            {
                minX = std::min(minX, real(rdram, d + 8u * i));
                maxX = std::max(maxX, real(rdram, d + 8u * i));
                minY = std::min(minY, real(rdram, d + 8u * i + 4u));
                maxY = std::max(maxY, real(rdram, d + 8u * i + 4u));
            }
            const float bw = maxX - minX, bh = maxY - minY;
            const float sx = real(rdram, d + 0x20u), sy = real(rdram, d + 0x24u), px = real(rdram, d + 0x28u), py = real(rdram, d + 0x2Cu);
            const Frame f = frameOf(rdram, w);
            const float baseX = f.ox + f.scaleX * (minX + px * (bw - bw * sx)), baseY = f.oy + f.scaleY * (minY + py * (bh - bh * sy));
            const uint32_t flags = word(rdram, d + 0x78u), part = word(rdram, d + 0x70u) & 0x1FFFFFFu;
            const bool textured = (flags & 8u) != 0u && n == 4u, gouraud = (flags & 4u) != 0u;
            float u[4] = {0, 1, 0, 1}, v[4] = {0, 0, 1, 1};
            if (textured)
            {
                if (part < 0x100000u || (flags & 2u))
                    for (uint32_t k = 0; k < 4u; ++k)
                    {
                        u[k] = real(rdram, d + 0x50u + 8u * k);
                        v[k] = real(rdram, d + 0x54u + 8u * k);
                    }
                else
                {
                    const uint32_t a = word(rdram, part), b = word(rdram, part + 4u);
                    const uint32_t uv[4] = {a, (b & 0xFFFFu) | (a & 0xFFFF0000u), (a & 0xFFFFu) | (b & 0xFFFF0000u), b};
                    for (uint32_t k = 0; k < 4u; ++k)
                    {
                        u[k] = static_cast<float>(uv[k] & 0xFFFFu) / 4096.0f;
                        v[k] = static_cast<float>(uv[k] >> 16) / 4096.0f;
                    }
                }
            }
            Primitive p;
            for (uint32_t i = 0; i < n; ++i)
                p.v[i] = vertex(baseX + sx * f.scaleX * (real(rdram, d + 8u * i) - minX), baseY + sy * f.scaleY * (real(rdram, d + 8u * i + 4u) - minY),
                                i < 4u ? u[i] : 0.0f, i < 4u ? v[i] : 0.0f, word(rdram, d + 0x40u + (gouraud ? 4u * i : 0u)));
            p.count = n;
            add(rdram, w, std::move(p), textured);
        }
        g_current = w;
        g_polygon(rdram, ctx, runtime);
    }

    // The sheet setters: the widget's own sheets (0x3CCE40(W, packet, texture, i, csa): the texture manager's entry
    // e = s16 *(G + 0x2670 + 2 *(texture + 4)), G = *0x35DE38, its key +0x1008 / +0x8 + 16 e, as the models'),
    // the shared battle sheet (0x4F2670: TBP G68+0xFC) and the portraits (0x4F2740: TBP G68+0xFE), G68 =
    // *(*0x36D780 + 0x68). The last one set is the current widget's sheet.
    void ownSheetHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        count(3, rdram, ctx);
        const uint32_t w = getRegU32(ctx, 4) & 0x1FFFFFFu, texture = getRegU32(ctx, 6) & 0x1FFFFFFu;
        const uint32_t texman = word(rdram, 0x0035DE38u);
        g_scratch = runtime->memory().getScratchpad();
        if (texture >= 0x100000u && g_scratch && (texman & 0xF0000000u) == 0x70000000u)
        {
            const auto sw = [&](uint32_t o) {
                uint32_t v;
                std::memcpy(&v, g_scratch + ((texman + o) & 0x3FFFu), 4);
                return v;
            };
            const uint32_t slot = word(rdram, texture + 4u) & 0xFFu;
            const int16_t e = static_cast<int16_t>(sw(0x2670u + 2u * slot) & 0xFFFFu);
            if (e > 0)
            {
                std::lock_guard<std::mutex> lock(g_mutex);
                Widget &widget = g_widgets[w];
                widget.tag = sw(0x1008u + 16u * static_cast<uint32_t>(e));
                widget.index = sw(0x8u + 16u * static_cast<uint32_t>(e));
                widget.csa = getRegU32(ctx, 8) & 0x1Fu;
            }
        }
        g_current = w;
        g_ownSheet(rdram, ctx, runtime);
    }

    // The TEX0 a battle sheet setter makes from G68 (*(*0x36D780 + 0x68)): TBP and CBP, the CSA.
    void battleSheet(const uint8_t *rdram, uint32_t offset, uint32_t tbpAdd, uint32_t cbpOffset, uint32_t cbpAdd, uint32_t csa)
    {
        const uint32_t g = word(rdram, 0x0036D780u) & 0x1FFFFFFu;
        const uint32_t g68 = g >= 0x100000u ? word(rdram, g + 0x68u) & 0x1FFFFFFu : 0u;
        if (g68 < 0x100000u || g_current == 0u)
            return;
        std::lock_guard<std::mutex> lock(g_mutex);
        Widget &widget = g_widgets[g_current];
        widget.tbp = ((word(rdram, g68 + offset) & 0xFFFFu) + tbpAdd) & 0x3FFFu;
        widget.cbp = ((word(rdram, g68 + cbpOffset) & 0xFFFFu) + cbpAdd) & 0x3FFFu;
        widget.csa = csa & 0x1Fu;
        widget.tag = 0;
        static const bool s_log = std::getenv("VP2S_BATTLE_HUD_LOG") != nullptr;
        static int s_logged = 0;
        if (s_log && s_logged++ < 2)
        {
            std::printf("[vp2s-hud] G68 %x +0xE0..+0x120:", g68);
            for (uint32_t k = 0xE0u; k < 0x120u; k += 4u)
            {
                const uint32_t v = word(rdram, g68 + k);
                std::printf(" %08x", v);
                if ((v & 0x1FFFFFFu) >= 0x100000u && (v & 0x1FFFFFFu) < 0x2000000u && word(rdram, v) == 0x00534946u)
                    std::printf("(FIS %08x %x)", word(rdram, v + 0x14u), word(rdram, v + 0x18u));
            }
            std::printf("\n");
            std::fflush(stdout);
        }
    }

    void sharedSheetHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        count(4, rdram, ctx);
        // 0x4F2670(packet, csa): TBP G68+0xFC, CBP G68+0x100, PSMT4HH.
        battleSheet(rdram, 0xFCu, 0u, 0x100u, 0u, getRegU32(ctx, 5));
        g_sharedSheet(rdram, ctx, runtime);
    }

    void portraitHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        count(5, rdram, ctx);
        // 0x4F2740(packet, set, face, uvs, alternate): TBP G68+0xFE (+0x100 for set 1), CBP G68+0x102 (+0x10 for set
        // 1) + 4 * face, or +0x20 + 4 * face with the alternate palette (set 0); PSMT8H, each face a quarter of the
        // sheet's width.
        const uint32_t set = getRegU32(ctx, 5), face = getRegU32(ctx, 6) & 0xFFu, alternate = set == 0u ? getRegU32(ctx, 8) : 0u;
        battleSheet(rdram, 0xFEu, set == 1u ? 0x100u : 0u, 0x102u, (alternate ? 0x20u : set == 1u ? 0x10u : 0u) + 4u * face, 0u);
        g_portrait(rdram, ctx, runtime);
    }

    // 0x3CCAE0(W): a widget's render ends: what it emitted becomes what it shows.
    void renderEndHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        count(6, rdram, ctx);
        const uint32_t w = getRegU32(ctx, 4) & 0x1FFFFFFu;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            Widget &widget = g_widgets[w];
            widget.shown.swap(widget.building);
            widget.building.clear();
            widget.lastFrame = g_frame;
        }
        g_renderEnd(rdram, ctx, runtime);
    }
}

namespace
{
    void fixedUploadHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t fis = getRegU32(ctx, 4) & 0x1FFFFFFu, dbp = getRegU32(ctx, 7) & 0x3FFFu;
        const bool known = word(rdram, fis) == 0x00534946u;
        const uint32_t cbp = getRegU32(ctx, 8) & 0x3FFFu;
        if (known && dbp != 0u)
            vp2sUiNoteFixedSheet(dbp, word(rdram, fis + 0x14u), word(rdram, fis + 0x18u));
        if (known && cbp != 0u)
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_fixedCluts[cbp] = {word(rdram, fis + 0x14u), word(rdram, fis + 0x18u)};
        }
        static const bool s_log = std::getenv("VP2S_BATTLE_HUD_LOG") != nullptr;
        if (s_log)
        {
            std::string part;
            uint32_t offset = 0;
            const bool loaded = vp2sLoadedFrom(fis, part, offset);
            std::printf("[vp2s-hud] fixed upload: image %x (%08x %08x %x) PSM %x DBP %x CBP %x, from %s +%x\n", fis, word(rdram, fis),
                        word(rdram, fis + 0x14u), word(rdram, fis + 0x18u), getRegU32(ctx, 6) & 0xFFu, dbp, getRegU32(ctx, 8) & 0x3FFFu,
                        loaded ? part.c_str() : "-", offset);
            std::fflush(stdout);
        }
        g_fixedUpload(rdram, ctx, runtime);
    }
}

void vp2sBattleHudDraw()
{
    std::lock_guard<std::mutex> lock(g_mutex);
    ++g_frame;
    if (g_widgets.empty())
        return;
    static const float s_identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    auto &renderer = ps2x::native::Renderer::instance();
    for (auto it = g_widgets.begin(); it != g_widgets.end();)
    {
        // A widget not rendered for 2 s is gone (the battle ended).
        if (g_frame - it->second.lastFrame > 120u)
        {
            it = g_widgets.erase(it);
            continue;
        }
        for (const Primitive &p : it->second.shown)
        {
            ps2x::native::Material material;
            material.blend = ps2x::native::Material::Blend::Alpha;
            material.alphaBlend = true;
            material.depthTest = false;
            material.depthWrite = true;
            material.alphaTest = true;
            material.alphaRef = 0;
            std::shared_ptr<const ps2x::native::TextureCache::Texture> texture;
            if (p.tag != 0u)
            {
                texture = vp2sGameTexture(p.tag, p.index, p.csa, p.paletteTag, p.paletteIndex);
                if (!texture)
                {
                    // Not converted: left out rather than drawn white. VP2S_BATTLE_HUD_LOG names it once.
                    static const bool s_log = std::getenv("VP2S_BATTLE_HUD_LOG") != nullptr;
                    static std::vector<uint64_t> s_missing;
                    const uint64_t key = uint64_t{p.tag} << 32 | p.index;
                    if (s_log && std::find(s_missing.begin(), s_missing.end(), key) == s_missing.end() && s_missing.size() < 32u)
                    {
                        s_missing.push_back(key);
                        std::printf("[vp2s-hud] no converted texture %08x %x (csa %u); its key in memory at", p.tag, p.index, p.csa);
                        // Where the key's info block (+4 tag, +8 index) sits in memory, and where the game loaded it from.
                        for (uint32_t at = 0x100000u, shown = 0; g_rdram && at + 12u < 0x2000000u && shown < 4u; at += 4u)
                            if (word(g_rdram, at + 4u) == p.tag && word(g_rdram, at + 8u) == p.index)
                            {
                                std::string part;
                                uint32_t offset = 0;
                                const bool known = vp2sLoadedFrom(at, part, offset);
                                std::printf(" %x (%s +%x)", at, known ? part.c_str() : "-", offset);
                                ++shown;
                            }
                        std::printf("\n");
                        std::fflush(stdout);
                    }
                    continue;
                }
            }
            material.textured = texture != nullptr;
            ps2x::native::Vertex triangles[18];
            uint32_t n = 0;
            for (uint32_t k = 2; k < p.count; ++k)
            {
                // A strip, triangle by triangle (no culling: their winding does not matter).
                triangles[n++] = p.v[k - 2];
                triangles[n++] = p.v[k - 1];
                triangles[n++] = p.v[k];
            }
            if (n)
                renderer.draw(material, s_identity, triangles, n, std::move(texture));
        }
        ++it;
    }
}

void vp2sInstallBattleHud(PS2Runtime &)
{
    if (const char *gs = std::getenv("VP2S_GS"); gs && gs[0] == '1')
        return;
    if (const char *hud = std::getenv("VP2S_BATTLE_HUD"); hud && hud[0] == '0')
        return;
    const ps2_overlays::Module *module = ps2_overlays::findModule("mod028");
    if (!module)
    {
        std::printf("[vp2s] native battle HUD: mod028 not found\n");
        return;
    }
    const auto take = [&](uint32_t address, PS2Runtime::RecompiledFunction &original, PS2Runtime::RecompiledFunction hook) {
        if (address < module->tableBase || address >= module->tableEnd)
            return;
        original = module->table[(address - module->tableBase) >> 2];
        if (original)
            ps2_overlays::replaceModuleFunction("mod028", address, hook);
    };
    take(kSprite, g_sprite, &spriteHook);
    take(kQuad, g_quad, &quadHook);
    take(kPolygon, g_polygon, &polygonHook);
    take(kOwnSheet, g_ownSheet, &ownSheetHook);
    take(kSharedSheet, g_sharedSheet, &sharedSheetHook);
    take(kPortrait, g_portrait, &portraitHook);
    take(kRenderEnd, g_renderEnd, &renderEndHook);
    if (const ps2_overlays::Module *battle = ps2_overlays::findModule("mod027");
        battle && kFixedUpload >= battle->tableBase && kFixedUpload < battle->tableEnd)
    {
        g_fixedUpload = battle->table[(kFixedUpload - battle->tableBase) >> 2];
        if (g_fixedUpload)
            ps2_overlays::replaceModuleFunction("mod027", kFixedUpload, &fixedUploadHook);
    }
}

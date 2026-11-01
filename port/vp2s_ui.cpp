// VP2's 2D layer for the native renderer from the game's own descriptions (docs/vp2_2d.md): each item an interface
// element builds (today: the text glyphs) is caught when the game lays it out, and drawn when the element's layer is
// drawn, in the layer's order after the 3D. Nothing reads the GIF packets the game builds for them.

#include "vp2s_ui.h"
#include "vp2s_loads.h"
#include "vp2s_textures.h"

#include "ps2_overlay_modules.h"
#include "ps2_runtime.h"
#include "runtime/native/native_renderer.h"

#include <algorithm>
#include <array>
#include <chrono>
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

    // A glyph as the game laid it out (its 0x30-byte item), with what drawing it needs.
    struct Glyph
    {
        float x0 = 0, y0 = 0, x1 = 0, y1 = 0; // the item's box (the element's space, virtual 640 x 480 units)
        float scale = 1.0f;
        uint32_t font = 0;  // the font object (EE)
        uint32_t index = 0; // glyph index in the font
        uint32_t colour = 0, shadowColour = 0, shadowMode = 0;
    };

    std::mutex g_mutex;
    std::unordered_map<uint32_t, std::vector<Glyph>> g_glyphs; // element -> its glyphs since its text began

    constexpr uint32_t kTextBegin = 0x002E85C0u, kGlyph = 0x002E7DD0u, kLayerDraw = 0x002EC8F0u, kSprite = 0x002EB2D0u;
    PS2Runtime::RecompiledFunction g_textBegin = nullptr, g_glyph = nullptr, g_layerDraw = nullptr, g_sprite = nullptr;

    // A sprite's quad as the game laid it out (virtual 640 x 480 units), with its texture by the sheet it samples.
    struct SpriteQuad
    {
        float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        float u0 = 0, v0 = 0, u1 = 0, v1 = 0; // normalised to the sheet (the parts' 4.12 values / 4096)
        uint32_t rgba = 0;
        uint32_t tbp = 0, csa = 0;
        uint32_t tag = 0, index = 0; // the sheet's key, from the texture manager (0: by TBP, sheetKey)
    };
    std::unordered_map<uint32_t, SpriteQuad> g_sprites; // element (sprite - 0x50) -> its quad

    // The sheets the interface samples, by the TBP the game uploads them to (docs/vp2_2d.md): their keys.
    bool sheetKey(uint32_t tbp, uint32_t &tag, uint32_t &index)
    {
        // VP2S_UI_SHEET800=<index> (diagnosis): the 'SHI\0' sheet to sample at TBP 0x800.
        if (const char *v = std::getenv("VP2S_UI_SHEET800"); v && tbp == 0x800u)
        {
            tag = 0x00494853u;
            index = static_cast<uint32_t>(std::strtoul(v, nullptr, 16));
            return true;
        }
        switch (tbp)
        {
        case 0x000u: tag = 0x00494853u; index = 0x41u; return true;  // 'SHI\0' 0x41
        case 0x400u: tag = 0x00494853u; index = 0x3Du; return true;  // 'SHI\0' 0x3D
        case 0x800u: tag = 0x4E484F4Au; index = 0x276Fu; return true; // 'JOHN' 0x276F (field HUD)
        default: return false;
        }
    }

    // The game's texture manager (*0x35DE38, in the scratchpad): a list of resident textures from the byte at +0x2008
    // (next: +0x2008 + e), each with its key (+0x1008 + 16e tag, +0x8 + 16e index) and its TEX0 (+0x1010 + 16e, TBP in
    // the low 14 bits; 0x29D8F0 returns it). A sprite samples the resident texture at its TBP (+0x5A). The capture
    // hooks set g_scratch.
    const uint8_t *g_scratch = nullptr;

    // Sheets the game puts at fixed VRAM outside the manager (the UI's TBP 0 / 0x400, the menus' 0x800 / 0x1000):
    // 0x2F3630(a0 image object, a1 its 'FIS' file, a2 PSM, a3 DBP of the pixels, t0 DBP of the palette or 0) sets up
    // the object 0x2F32F0 uploads. DBP -> the FIS file's key (+0x14 tag, +0x18 index).
    constexpr uint32_t kImageSetup = 0x002F3630u;
    PS2Runtime::RecompiledFunction g_imageSetup = nullptr;
    std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>> g_fixedSheets; // DBP -> (tag, index)

    void imageSetupHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t fis = getRegU32(ctx, 5) & 0x1FFFFFFu, dbp = getRegU32(ctx, 7) & 0x3FFFu;
        if (word(rdram, fis) == 0x00534946u)
        {
            const uint32_t tag = word(rdram, fis + 0x14u), index = word(rdram, fis + 0x18u);
            {
                std::lock_guard<std::mutex> lock(g_mutex);
                g_fixedSheets[dbp] = {tag, index};
            }
            static const bool s_log = std::getenv("VP2S_UI_SPRITE_LOG") != nullptr;
            if (s_log)
            {
                std::printf("[vp2s-ui] image set up: FIS %x (%08x %x) at DBP %x, PSM %x\n", fis, tag, index, dbp, getRegU32(ctx, 6) & 0xFFu);
                std::fflush(stdout);
            }
        }
        g_imageSetup(rdram, ctx, runtime);
    }

    bool sheetOf(const uint8_t *rdram, uint32_t sprite, uint32_t &tag, uint32_t &index)
    {
        const uint32_t texman = word(rdram, 0x0035DE38u);
        if (!g_scratch || (texman & 0xF0000000u) != 0x70000000u)
            return false;
        const auto scratchWord = [&](uint32_t offset) {
            uint32_t value;
            std::memcpy(&value, g_scratch + ((texman + offset) & 0x3FFFu), 4);
            return value;
        };
        const uint32_t tbp = (word(rdram, sprite + 0x58u) >> 16) & 0x3FFFu;
        static const bool s_log = std::getenv("VP2S_UI_SPRITE_LOG") != nullptr;
        if (s_log)
        {
            static std::vector<uint32_t> s_seen;
            if (std::find(s_seen.begin(), s_seen.end(), tbp) == s_seen.end() && s_seen.size() < 64u)
            {
                s_seen.push_back(tbp);
                std::printf("[vp2s-ui] TBP %x resident:", tbp);
                uint32_t r = g_scratch[(texman + 0x2008u) & 0x3FFFu];
                for (int guard = 0; r != 0u && guard < 256; ++guard, r = g_scratch[(texman + 0x2008u + r) & 0x3FFFu])
                    std::printf(" %x:%08x/%x", scratchWord(0x1010u + 16u * r) & 0x3FFFu, scratchWord(0x1008u + 16u * r), scratchWord(0x8u + 16u * r));
                std::printf("\n");
                std::fflush(stdout);
            }
        }
        uint32_t e = g_scratch[(texman + 0x2008u) & 0x3FFFu];
        for (int guard = 0; e != 0u && guard < 256; ++guard, e = g_scratch[(texman + 0x2008u + e) & 0x3FFFu])
            if ((scratchWord(0x1010u + 16u * e) & 0x3FFFu) == tbp)
            {
                tag = scratchWord(0x1008u + 16u * e);
                index = scratchWord(0x8u + 16u * e);
                return true;
            }
        if (const auto fixed = g_fixedSheets.find(tbp); fixed != g_fixedSheets.end())
        {
            tag = fixed->second.first;
            index = fixed->second.second;
            return true;
        }
        // The menus' sheets (their upload is not found yet; checked against the original's menus): TBP 0x800 the frame
        // ornaments 'SHI\0' 0x5E and 0x1000 the stone 'SHI\0' 0x3E, while their FIS files are in memory (the field HUD
        // uses 0x800 for 'JOHN' 0x276F otherwise). Looked up at most every half second.
        if (tbp == 0x800u || tbp == 0x1000u)
        {
            static auto s_checked = std::chrono::steady_clock::time_point{};
            static bool s_menu = false;
            if (const auto now = std::chrono::steady_clock::now(); now - s_checked > std::chrono::milliseconds(500))
            {
                s_checked = now;
                s_menu = false;
                for (uint32_t at = 0x100000u; at < 0x2000000u && !s_menu; at += 0x80u)
                    s_menu = word(rdram, at) == 0x00534946u && word(rdram, at + 0x14u) == 0x00494853u && word(rdram, at + 0x18u) == 0x5Eu;
            }
            if (s_menu)
            {
                tag = 0x00494853u;
                index = tbp == 0x800u ? 0x5Eu : 0x3Eu;
                return true;
            }
        }
        return false;
    }

    // 0x2EB2D0(a0 sprite = element + 0x50, ...): one quad. Sprite +0x08 parts (16 bytes: u0, v0, u1, v1 in 4.12 of the
    // sheet, w, h, +0xC CSA), +0x54 frame, +0x14 / +0x18 position, +0x2C / +0x30 scale, +0x40..+0x4C colour and
    // +0x50 alpha (128 = 1), +0x5A TBP, +0x64 CSA (negative: the part's).
    void captureSprite(const uint8_t *rdram, uint32_t sprite, uint32_t position)
    {
        const int32_t frame = static_cast<int16_t>(word(rdram, sprite + 0x54u) & 0xFFFFu);
        const uint32_t part = (word(rdram, sprite + 8u) + 16u * static_cast<uint32_t>(std::max(frame, 0))) & 0x1FFFFFFu;
        const auto half = [&](uint32_t address) { return static_cast<uint16_t>(word(rdram, address) & 0xFFFFu); };
        SpriteQuad q;
        q.u0 = half(part) / 4096.0f;
        q.v0 = half(part + 2u) / 4096.0f;
        q.u1 = half(part + 4u) / 4096.0f;
        q.v1 = half(part + 6u) / 4096.0f;
        const float w = half(part + 8u), h = half(part + 10u);
        const float sx = real(rdram, sprite + 0x2Cu), sy = real(rdram, sprite + 0x30u);
        // The position the caller passes (a2: its layer's or window's) plus the sprite's, less the pivot (+0x1C /
        // +0x20), scaled about +0x24 / +0x28.
        const float cx = real(rdram, sprite + 0x24u) * (1.0f - sx), cy = real(rdram, sprite + 0x28u) * (1.0f - sy);
        const float px = real(rdram, position) + real(rdram, sprite + 0x14u), py = real(rdram, position + 4u) + real(rdram, sprite + 0x18u);
        q.x0 = px + cx - real(rdram, sprite + 0x1Cu) * sx;
        q.y0 = py + cy - real(rdram, sprite + 0x20u) * sy;
        q.x1 = q.x0 + w * sx;
        q.y1 = q.y0 + h * sy;
        const auto channel = [&](uint32_t offset) { return std::min<uint32_t>(static_cast<uint32_t>(std::max(real(rdram, sprite + offset), 0.0f)), 255u); };
        // RGB times +0x4C / 128, alpha +0x50.
        const float scale = real(rdram, sprite + 0x4Cu) / 128.0f;
        const auto scaled = [&](uint32_t offset) { return std::min<uint32_t>(static_cast<uint32_t>(std::max(real(rdram, sprite + offset) * scale, 0.0f)), 255u); };
        q.rgba = scaled(0x40u) | scaled(0x44u) << 8 | scaled(0x48u) << 16 | channel(0x50u) << 24;
        q.tbp = (word(rdram, sprite + 0x58u) >> 16) & 0x3FFFu;
        const int8_t csa = static_cast<int8_t>(rdram[(sprite + 0x64u) & 0x1FFFFFFu]);
        q.csa = csa >= 0 ? static_cast<uint32_t>(csa) : rdram[(part + 0xCu) & 0x1FFFFFFu];
        std::lock_guard<std::mutex> lock(g_mutex);
        sheetOf(rdram, sprite, q.tag, q.index);
        g_sprites[(sprite - 0x50u) & 0x1FFFFFFu] = q;
    }

    // VP2S_UI_SPRITE_LOG: the first calls of the quad-list (0x2EAC40) and window-background (0x2E9DD0) emitters, raw,
    // to read their arguments and structures before drawing them.
    constexpr uint32_t kQuadList = 0x002EAC40u, kWindowBackground = 0x002E9DD0u;
    PS2Runtime::RecompiledFunction g_quadList = nullptr, g_windowBackground = nullptr;

    void logCall(const char *what, const uint8_t *rdram, R5900Context *ctx)
    {
        std::printf("[vp2s-ui] %s a0 %x a1 %x a2 %x a3 %x t0 %x t1 %x f12 %f:", what, getRegU32(ctx, 4), getRegU32(ctx, 5), getRegU32(ctx, 6),
                    getRegU32(ctx, 7), getRegU32(ctx, 8), getRegU32(ctx, 9), ctx->f[12]);
        const uint32_t a0 = getRegU32(ctx, 4) & 0x1FFFFFFu;
        for (uint32_t k = 0; k < 0x70u; k += 4u)
            std::printf(" %08x", word(rdram, a0 + k));
        const uint32_t list = word(rdram, a0 + 0xCu) & 0x1FFFFFFu;
        if (list)
        {
            std::printf(" | +0xC list:");
            for (uint32_t k = 0; k < 72u; k += 4u)
                std::printf(" %08x", word(rdram, list + k));
        }
        std::printf("\n");
        std::fflush(stdout);
    }

    std::unordered_map<uint32_t, std::vector<SpriteQuad>> g_quadLists; // element (sprite - 0x50) -> its quads

    // 0x2EAC40(a0 sprite): a list of quads at +0x0C, +0x58 of them, 24 bytes each: {part, x0, y0, x1, y1, z}
    // (x1 / y1 0: the part's size), placed from the sprite's position; the sprite's colour, alpha and sheet.
    void captureQuadList(const uint8_t *rdram, uint32_t sprite, uint32_t position)
    {
        const uint32_t list = word(rdram, sprite + 0xCu) & 0x1FFFFFFu;
        const uint32_t count = std::min<uint32_t>(word(rdram, sprite + 0x58u) & 0xFFFFu, 256u);
        const uint32_t parts = word(rdram, sprite + 8u) & 0x1FFFFFFu;
        const auto half = [&](uint32_t address) { return static_cast<uint16_t>(word(rdram, address) & 0xFFFFu); };
        const auto channel = [&](uint32_t offset) { return std::min<uint32_t>(static_cast<uint32_t>(std::max(real(rdram, sprite + offset), 0.0f)), 255u); };
        const uint32_t rgba = channel(0x40u) | channel(0x44u) << 8 | channel(0x48u) << 16 | channel(0x50u) << 24; // no +0x4C here
        const uint32_t tbp = (word(rdram, sprite + 0x58u) >> 16) & 0x3FFFu;
        const int8_t spriteCsa = static_cast<int8_t>(rdram[(sprite + 0x64u) & 0x1FFFFFFu]);
        const float baseX = real(rdram, position) + real(rdram, sprite + 0x14u), baseY = real(rdram, position + 4u) + real(rdram, sprite + 0x18u);
        std::vector<SpriteQuad> quads;
        for (uint32_t i = 0; list != 0u && i < count; ++i)
        {
            const uint32_t entry = list + 24u * i;
            const uint32_t part = (parts + 16u * (word(rdram, entry) & 0xFFFFu)) & 0x1FFFFFFu;
            SpriteQuad q;
            q.u0 = half(part) / 4096.0f;
            q.v0 = half(part + 2u) / 4096.0f;
            q.u1 = half(part + 4u) / 4096.0f;
            q.v1 = half(part + 6u) / 4096.0f;
            // x1 / y1 are the far corner (a stretched middle piece), <= 0 for the part's own size.
            const float x1 = real(rdram, entry + 12u), y1 = real(rdram, entry + 16u);
            q.x0 = baseX + real(rdram, entry + 4u);
            q.y0 = baseY + real(rdram, entry + 8u);
            q.x1 = x1 > 0.0f ? baseX + x1 : q.x0 + static_cast<float>(half(part + 8u));
            q.y1 = y1 > 0.0f ? baseY + y1 : q.y0 + static_cast<float>(half(part + 10u));
            q.rgba = rgba;
            q.tbp = tbp;
            q.csa = spriteCsa >= 0 ? static_cast<uint32_t>(spriteCsa) : rdram[(part + 0xCu) & 0x1FFFFFFu];
            quads.push_back(q);
        }
        std::lock_guard<std::mutex> lock(g_mutex);
        uint32_t tag = 0, index = 0;
        if (sheetOf(rdram, sprite, tag, index))
            for (SpriteQuad &q : quads)
            {
                q.tag = tag;
                q.index = index;
            }
        g_quadLists[(sprite - 0x50u) & 0x1FFFFFFu] = std::move(quads);
    }

    void quadListHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        g_scratch = runtime->memory().getScratchpad();
        captureQuadList(rdram, getRegU32(ctx, 4) & 0x1FFFFFFu, getRegU32(ctx, 6) & 0x1FFFFFFu);
        static const bool s_log = std::getenv("VP2S_UI_SPRITE_LOG") != nullptr;
        static int s_logged = 0;
        if (s_log && s_logged < 4)
        {
            ++s_logged;
            logCall("quad list", rdram, ctx);
        }
        g_quadList(rdram, ctx, runtime);
    }

    // A window's background as 0x2E9DD0(a0 window) builds it: triangle strips of four gouraud vertices (virtual units).
    struct StripVertex
    {
        float x = 0, y = 0;
        uint32_t rgba = 0;
    };
    std::unordered_map<uint32_t, std::vector<std::array<StripVertex, 4>>> g_windows; // window element -> its strips

    // The rectangle (+0x18, or +0xB8 when +0xE8) placed by its layer (+0x14: +0x40 / +0x44); a 4-colour gradient
    // (+0xF5 picks the darker set) at 0.95 x the alpha, then 8 strips darkening the edges (half the alpha, fading to
    // nothing inwards over a quarter of the size, at most 24) from the game's table at 0x357900 {x, y, colour} x 4.
    void captureWindow(const uint8_t *rdram, uint32_t window)
    {
        const uint32_t rect = word(rdram, window + 0xE8u) ? window + 0xB8u : window + 0x18u;
        const uint32_t layer = word(rdram, window + 0x14u) & 0x1FFFFFFu;
        const float x = real(rdram, layer + 0x40u) + real(rdram, rect), y = real(rdram, layer + 0x44u) + real(rdram, rect + 4u);
        const float w = real(rdram, rect + 8u), h = real(rdram, rect + 12u);
        const float mx = w >= 96.0f ? 24.0f : w * 0.25f, my = h >= 96.0f ? 24.0f : h * 0.25f;
        const float xs[4] = {x, x + mx, x + w - mx, x + w}, ys[4] = {y, y + my, y + h - my, y + h};
        const float alpha = real(rdram, layer + 0xE0u) * real(rdram, window + 0x28u) / 128.0f;
        const uint32_t a95 = static_cast<uint32_t>(std::clamp(alpha * 0.95f, 0.0f, 255.0f)) << 24;
        const bool dark = rdram[(window + 0xF5u) & 0x1FFFFFFu] != 0;
        const uint32_t c0 = (dark ? 0x2D1612u : 0x44211Bu) | a95, c1 = (dark ? 0x2A120Fu : 0x3F1B17u) | a95,
                       c2 = (dark ? 0x362718u : 0x513A24u) | a95;
        std::vector<std::array<StripVertex, 4>> strips;
        strips.push_back({StripVertex{xs[3], ys[0], c0}, StripVertex{xs[3], ys[3], c1}, StripVertex{xs[0], ys[0], c2},
                          StripVertex{xs[0], ys[3], c0}});
        const uint32_t edge[2] = {static_cast<uint32_t>(std::clamp(alpha * 0.5f, 0.0f, 255.0f)) << 24, 0u};
        for (uint32_t s = 0; s < 8u; ++s)
        {
            std::array<StripVertex, 4> strip;
            for (uint32_t v = 0; v < 4u; ++v)
            {
                const uint8_t *t = rdram + 0x357900u + 12u * s + 3u * v;
                strip[v] = {xs[t[0] & 3u], ys[t[1] & 3u], edge[t[2] & 1u]};
            }
            strips.push_back(strip);
        }
        std::lock_guard<std::mutex> lock(g_mutex);
        g_windows[window] = std::move(strips);
    }

    // VP2S_UI_SPRITE_LOG: 0x2EC130(a0 sprite, a1 DTT, a2 TEX0 record or 0) gives a sprite its parts and sheet; where
    // the record and the DTT were loaded from, to key the sheets outside the texture manager (TBP 0x800, 0x1000).
    constexpr uint32_t kSpriteSheet = 0x002EC130u;
    PS2Runtime::RecompiledFunction g_spriteSheet = nullptr;

    void spriteSheetHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        static int s_logged = 0;
        const uint32_t record = getRegU32(ctx, 6) & 0x1FFFFFFu, dtt = getRegU32(ctx, 5) & 0x1FFFFFFu;
        if (record && s_logged < 40)
        {
            ++s_logged;
            std::string recordPart, dttPart;
            uint32_t recordOffset = 0, dttOffset = 0;
            const bool r = vp2sLoadedFrom(record, recordPart, recordOffset), d = vp2sLoadedFrom(dtt, dttPart, dttOffset);
            std::printf("[vp2s-ui] sprite sheet: sprite %x DTT %x (%s +%x) record %x (%s +%x): %08x %08x\n", getRegU32(ctx, 4), dtt,
                        d ? dttPart.c_str() : "-", dttOffset, record, r ? recordPart.c_str() : "-", recordOffset, word(rdram, record),
                        word(rdram, record + 4u));
            // Once: every FIS in memory with its pixel format and size (image header at FIS + 0x10 + [+0x24]: +0x20 PSM in
            // bits 48..53, +0x40 width / height), to find the sheet a fixed TBP holds.
            static bool s_listed = false;
            if (!s_listed && (word(rdram, record) & 0x3FFFu) == 0x800u)
            {
                s_listed = true;
                for (uint32_t at = 0x100000u; at < 0x2000000u; at += 16u)
                    if (word(rdram, at) == 0x00534946u)
                    {
                        const uint32_t header = at + 0x10u + word(rdram, at + 0x24u);
                        if (header >= 0x2000000u - 0x48u)
                            continue;
                        std::string part;
                        uint32_t offset = 0;
                        const bool known = vp2sLoadedFrom(at, part, offset);
                        std::printf("[vp2s-ui] FIS %x %08x %x: PSM %x, %u x %u, %s +%x; pointed to from", at, word(rdram, at + 0x14u),
                                    word(rdram, at + 0x18u), (word(rdram, header + 0x24u) >> 16) & 0x3Fu, word(rdram, header + 0x40u) & 0xFFFu,
                                    word(rdram, header + 0x44u) & 0xFFFu, known ? part.c_str() : "-", offset);
                        int found = 0;
                        for (uint32_t p = 0x100000u; p < 0x2000000u && found < 6; p += 4u)
                            if ((word(rdram, p) & 0x1FFFFFFu) == at && (word(rdram, p) & 0xC0000000u) == 0u)
                            {
                                std::printf(" %x", p);
                                ++found;
                            }
                        std::printf("\n");
                    }
            }
            std::fflush(stdout);
        }
        g_spriteSheet(rdram, ctx, runtime);
    }

    void windowBackgroundHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        captureWindow(rdram, getRegU32(ctx, 4) & 0x1FFFFFFu);
        static const bool s_log = std::getenv("VP2S_UI_SPRITE_LOG") != nullptr;
        static int s_logged = 0;
        if (s_log && s_logged < 4)
        {
            ++s_logged;
            logCall("window background", rdram, ctx);
        }
        g_windowBackground(rdram, ctx, runtime);
    }

    // VP2S_UI_SPRITE_LOG: the first sprites as the game hands them to 0x2EB2D0, raw, to read their layout.
    void spriteHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        g_scratch = runtime->memory().getScratchpad();
        captureSprite(rdram, getRegU32(ctx, 4) & 0x1FFFFFFu, getRegU32(ctx, 6) & 0x1FFFFFFu);
        static const bool s_log = std::getenv("VP2S_UI_SPRITE_LOG") != nullptr;
        static int s_logged = 0;
        // The first sprites, and the first ones sampling the texture manager's textures (TBP 0x3000 and up).
        static int s_managed = 0;
        const bool managed = ((word(rdram, (getRegU32(ctx, 4) & 0x1FFFFFFu) + 0x58u) >> 16) & 0x3FFFu) >= 0x3000u;
        if (s_log && (s_logged < 6 || (managed && s_managed < 6)))
        {
            ++s_logged;
            s_managed += managed ? 1 : 0;
            const uint32_t sprite = getRegU32(ctx, 4) & 0x1FFFFFFu;
            std::printf("[vp2s-ui] sprite %x a0 %x a2 %x a3 %x (%.1f, %.1f) t0 %x t1 %x:", sprite, getRegU32(ctx, 4), getRegU32(ctx, 6),
                        getRegU32(ctx, 7), real(rdram, getRegU32(ctx, 7)), real(rdram, getRegU32(ctx, 7) + 4u), getRegU32(ctx, 8), getRegU32(ctx, 9));
            for (uint32_t k = 0; k < 0x70u; k += 4u)
                std::printf(" %08x", word(rdram, sprite + k));
            const uint32_t frame = static_cast<uint32_t>(static_cast<int16_t>(word(rdram, sprite + 0x54u) & 0xFFFFu));
            const uint32_t parts = (word(rdram, sprite + 8u) + 16u * frame) & 0x1FFFFFFu, quads = word(rdram, sprite + 0xCu) & 0x1FFFFFFu;
            std::printf(" | part0:");
            for (uint32_t k = 0; k < 16u; k += 4u)
                std::printf(" %08x", word(rdram, parts + k));
            std::printf(" | quad0:");
            for (uint32_t k = 0; k < 24u; k += 4u)
                std::printf(" %08x", word(rdram, quads + k));
            std::printf("\n");
            std::fflush(stdout);
        }
        g_sprite(rdram, ctx, runtime);
    }

    void textBeginHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_glyphs[getRegU32(ctx, 4) & 0x1FFFFFFu].clear();
        }
        { static int s_n = 0; if (s_n < 5) { ++s_n; std::printf("[vp2s-ui] text begin %x\n", getRegU32(ctx, 4)); std::fflush(stdout); } }
        g_textBegin(rdram, ctx, runtime);
    }

    // 0x2E7DD0(element, item): item +0x00..0x0C box, +0x10 scale, +0x18 font, +0x20 colour, +0x24 shadow mode, +0x28
    // shadow colour, +0x2E glyph index.
    void glyphHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t element = getRegU32(ctx, 4) & 0x1FFFFFFu, item = getRegU32(ctx, 5);
        Glyph g;
        g.x0 = real(rdram, item);
        g.y0 = real(rdram, item + 4u);
        g.x1 = real(rdram, item + 8u);
        g.y1 = real(rdram, item + 0xCu);
        g.scale = real(rdram, item + 0x10u);
        g.font = word(rdram, item + 0x18u);
        g.colour = word(rdram, item + 0x20u);
        g.shadowMode = word(rdram, item + 0x24u);
        g.shadowColour = word(rdram, item + 0x28u);
        g.index = word(rdram, item + 0x2Cu) >> 16;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_glyphs[element].push_back(g);
        }
        { static int s_n = 0; if (s_n < 5) { ++s_n; std::printf("[vp2s-ui] glyph %x\n", getRegU32(ctx, 4)); std::fflush(stdout); } }
        g_glyph(rdram, ctx, runtime);
    }

    // A glyph's picture: 4 bits a pixel at font+0x10 + index * font+0x20, font+0x24 x font+0x28, with palette part csa of
    // the 'SHI\0' 0x41 sheet (the game's own glyph palettes).
    std::shared_ptr<const ps2x::native::TextureCache::Texture> glyphTexture(const uint8_t *rdram, uint32_t font, uint32_t index,
                                                                            uint32_t csa)
    {
        static std::unordered_map<uint64_t, std::shared_ptr<const ps2x::native::TextureCache::Texture>> s_cache;
        static uint32_t s_palette[256];
        static bool s_havePalette = vp2sGamePalette(0x00494853u, 0x41u, s_palette);
        const uint64_t key = (uint64_t{font} << 32) ^ (uint64_t{index} << 4) ^ csa;
        if (const auto it = s_cache.find(key); it != s_cache.end())
            return it->second;
        if (!s_havePalette)
            return nullptr;
        const uint32_t width = word(rdram, font + 0x24u), height = word(rdram, font + 0x28u);
        const uint32_t bytes = word(rdram, font + 0x20u), base = word(rdram, font + 0x10u) + index * bytes;
        if (width == 0u || width > 256u || height == 0u || height > 256u)
            return nullptr;
        ps2x::native::TextureCache::Texture texture;
        texture.width = 1u;
        while (texture.width < width)
            texture.width <<= 1;
        texture.height = 1u;
        while (texture.height < height)
            texture.height <<= 1;
        texture.key = key;
        texture.version = 1;
        texture.rgba.assign(static_cast<size_t>(texture.width) * texture.height, 0u);
        for (uint32_t y = 0; y < height; ++y)
            for (uint32_t x = 0; x < width; ++x)
            {
                const uint32_t k = y * width + x;
                const uint8_t pair = rdram[(base + k / 2u) & 0x1FFFFFFu];
                const uint32_t nibble = (k & 1u) ? pair >> 4 : pair & 15u;
                texture.rgba[static_cast<size_t>(y) * texture.width + x] = s_palette[(csa * 16u + nibble) & 0xFFu];
            }
        auto shared = std::make_shared<const ps2x::native::TextureCache::Texture>(std::move(texture));
        s_cache[key] = shared;
        return shared;
    }

    // A quad in GS pixels (the virtual 640 x 480 screen: x = 1728 + vx, y = 1824 + 0.9333 vy), drawn over the 3D.
    void drawQuad(float x0, float y0, float x1, float y1, float u1, float v1, uint32_t rgba,
                  std::shared_ptr<const ps2x::native::TextureCache::Texture> texture)
    {
        // VP2S_UI_DRAW=0 (diagnosis): the items made but not drawn.
        static const bool s_noDraw = [] { const char *v = std::getenv("VP2S_UI_DRAW"); return v && v[0] == '0'; }();
        if (s_noDraw)
            return;
        static const float s_identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        ps2x::native::Material material;
        material.blend = ps2x::native::Material::Blend::Alpha;
        material.alphaBlend = true;
        material.depthTest = false;
        material.depthWrite = false;
        material.alphaTest = true;
        material.alphaRef = 0;
        material.textured = texture != nullptr;
        const auto vertex = [&](float vx, float vy, float u, float v) {
            ps2x::native::Vertex out;
            out.x = 1728.0f + vx;
            out.y = 1824.0f + vy * (448.0f / 480.0f);
            out.z = 16777215.0f;
            out.u = u;
            out.v = v;
            out.rgba = rgba;
            return out;
        };
        const ps2x::native::Vertex quad[6] = {vertex(x0, y0, 0, 0), vertex(x1, y0, u1, 0), vertex(x0, y1, 0, v1),
                                              vertex(x1, y0, u1, 0), vertex(x1, y1, u1, v1), vertex(x0, y1, 0, v1)};
        ps2x::native::Renderer::instance().draw(material, s_identity, quad, 6u, std::move(texture));
    }

    // An untextured triangle strip of four gouraud vertices (a window background's).
    void drawStrip(const std::array<StripVertex, 4> &strip)
    {
        static const float s_identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        ps2x::native::Material material;
        material.blend = ps2x::native::Material::Blend::Alpha;
        material.alphaBlend = true;
        material.depthTest = false;
        material.depthWrite = false;
        material.textured = false;
        const auto vertex = [&](const StripVertex &s) {
            ps2x::native::Vertex out;
            out.x = 1728.0f + s.x;
            out.y = 1824.0f + s.y * (448.0f / 480.0f);
            out.z = 16777215.0f;
            out.rgba = s.rgba;
            return out;
        };
        const ps2x::native::Vertex triangles[6] = {vertex(strip[0]), vertex(strip[1]), vertex(strip[2]),
                                                   vertex(strip[1]), vertex(strip[3]), vertex(strip[2])};
        ps2x::native::Renderer::instance().draw(material, s_identity, triangles, 6u, nullptr);
    }

    // The same with the texture's own rectangle (u0, v0)..(u1, v1).
    void drawQuadUv(float x0, float y0, float x1, float y1, float u0, float v0, float u1, float v1, uint32_t rgba,
                    std::shared_ptr<const ps2x::native::TextureCache::Texture> texture)
    {
        static const float s_identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        ps2x::native::Material material;
        material.blend = ps2x::native::Material::Blend::Alpha;
        material.alphaBlend = true;
        material.depthTest = false;
        material.depthWrite = false;
        material.alphaTest = true;
        material.alphaRef = 0;
        material.textured = texture != nullptr;
        const auto vertex = [&](float vx, float vy, float u, float v) {
            ps2x::native::Vertex out;
            out.x = 1728.0f + vx;
            out.y = 1824.0f + vy * (448.0f / 480.0f);
            out.z = 16777215.0f;
            out.u = u;
            out.v = v;
            out.rgba = rgba;
            return out;
        };
        const ps2x::native::Vertex quad[6] = {vertex(x0, y0, u0, v0), vertex(x1, y0, u1, v0), vertex(x0, y1, u0, v1),
                                              vertex(x1, y0, u1, v0), vertex(x1, y1, u1, v1), vertex(x0, y1, u0, v1)};
        ps2x::native::Renderer::instance().draw(material, s_identity, quad, 6u, std::move(texture));
    }

    uint32_t scaled(uint32_t rgba, uint32_t colourScale, uint32_t alpha)
    {
        const auto channel = [&](int c) { return std::min<uint32_t>(((rgba >> (8 * c)) & 0xFFu) * colourScale / 128u, 0xFFu); };
        return channel(0) | channel(1) << 8 | channel(2) << 16 | std::min<uint32_t>(alpha, 0xFFu) << 24;
    }

    // 0x2EC8F0(layer): every layer's draw. Its children with +0x39, +0x3D and +0x3B set (the game's own rule) are drawn.
    void layerDrawHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t layer = getRegU32(ctx, 4) & 0x1FFFFFFu;
        { static int s_n = 0; if (s_n < 5) { ++s_n; std::printf("[vp2s-ui] layer draw %x\n", getRegU32(ctx, 4)); std::fflush(stdout); } }
        g_layerDraw(rdram, ctx, runtime);
        { static int s_n = 0; if (s_n < 5) { ++s_n; std::printf("[vp2s-ui] layer drawn\n"); std::fflush(stdout); } }
        const float layerX = real(rdram, layer + 0x40u), layerY = real(rdram, layer + 0x44u);
        // Alphas and the colour multiplier are floats, 128 = 1.
        const auto byte = [&](uint32_t address) { return static_cast<uint32_t>(std::clamp(real(rdram, address), 0.0f, 255.0f)); };
        const uint32_t layerAlpha = byte(layer + 0xE0u);
        std::lock_guard<std::mutex> lock(g_mutex);
        // The children list (next at +8) may come back round to its start or the layer: each child once.
        std::vector<uint32_t> seen;
        uint32_t element = word(rdram, layer + 8u);
        for (int guard = 0; element != 0u && guard < 512; ++guard, element = word(rdram, element + 8u))
        {
            element &= 0x1FFFFFFu;
            if (element == layer || std::find(seen.begin(), seen.end(), element) != seen.end())
                break;
            seen.push_back(element);
            {
                static const bool s_log = std::getenv("VP2S_UI_SPRITE_LOG") != nullptr;
                static int s_logged = 0;
                if (s_log && g_sprites.count(element) && s_logged < 10)
                {
                    ++s_logged;
                    const SpriteQuad &q = g_sprites[element];
                    std::printf("[vp2s-ui] sprite element %x in layer %x: flags 39=%d 3D=%d 3B=%d; quad %.1f,%.1f-%.1f,%.1f uv %.1f,%.1f-%.1f,%.1f rgba %08x; "
                                "layer alpha byte %u word %08x, element +0x28 byte %u word %08x\n",
                                element, layer, rdram[element + 0x39u], rdram[element + 0x3Du], rdram[element + 0x3Bu], q.x0, q.y0, q.x1, q.y1, q.u0,
                                q.v0, q.u1, q.v1, q.rgba, rdram[layer + 0xE0u], word(rdram, layer + 0xE0u), rdram[element + 0x28u],
                                word(rdram, element + 0x28u));
                    std::fflush(stdout);
                }
            }
            if (!rdram[element + 0x39u] || !rdram[element + 0x3Du] || !rdram[element + 0x3Bu])
                continue;
            {
                static int s_traced = 0;
                if (s_traced < 5)
                {
                    ++s_traced;
                    std::printf("[vp2s-ui] child %x shown\n", element);
                    std::fflush(stdout);
                }
            }
            // VP2S_UI_SPRITE_LOG: once, after a while, the sprites captured and the children a layer walks.
            {
                static const bool s_log = std::getenv("VP2S_UI_SPRITE_LOG") != nullptr;
                static uint64_t s_seen = 0;
                if (s_log && ++s_seen == 3000u)
                {
                    std::printf("[vp2s-ui] %zu sprites captured:", g_sprites.size());
                    for (const auto &[e, q] : g_sprites)
                        std::printf(" %x(tbp %x csa %u at %.0f,%.0f)", e, q.tbp, q.csa, q.x0, q.y0);
                    std::printf("\n[vp2s-ui] layer %x walks %x flags %d %d %d\n", layer, element, rdram[element + 0x39u], rdram[element + 0x3Du],
                                rdram[element + 0x3Bu]);
                    std::fflush(stdout);
                }
            }
            const auto drawSprite = [&](const SpriteQuad &q) {
                uint32_t tag = q.tag, index = q.index;
                if (tag != 0u || sheetKey(q.tbp, tag, index))
                    if (auto texture = vp2sGameTexture(tag, index, q.csa))
                    {
                        const uint32_t alpha = ((q.rgba >> 24) * layerAlpha / 128u) * byte(element + 0x28u) / 128u;
                        drawQuadUv(q.x0, q.y0, q.x1, q.y1, q.u0, q.v0, q.u1, q.v1, (q.rgba & 0xFFFFFFu) | std::min<uint32_t>(alpha, 255u) << 24,
                                   std::move(texture));
                    }
            };
            if (const auto window = g_windows.find(element); window != g_windows.end())
                for (const auto &strip : window->second)
                    drawStrip(strip);
            if (const auto sprite = g_sprites.find(element); sprite != g_sprites.end())
                drawSprite(sprite->second);
            if (const auto list = g_quadLists.find(element); list != g_quadLists.end())
                for (const SpriteQuad &q : list->second)
                    drawSprite(q);
            const auto found = g_glyphs.find(element);
            if (found == g_glyphs.end())
                continue;
            const float ex = layerX + real(rdram, element + 0x18u), ey = layerY + real(rdram, element + 0x1Cu);
            const uint32_t alpha = layerAlpha * byte(element + 0x28u) / 128u;
            const uint32_t colourScale = byte(element + 0x90u);
            for (const Glyph &g : found->second)
            {
                const uint32_t font = g.font & 0x1FFFFFFu;
                const float border = static_cast<float>(word(rdram, font + 0x2Cu)) * g.scale;
                const float width = static_cast<float>(word(rdram, font + 0x24u)), height = static_cast<float>(word(rdram, font + 0x28u));
                for (int pass = 0; pass < 2; ++pass)
                {
                    if (pass == 0 && g.shadowMode == 0u)
                        continue;
                    const uint32_t csa = pass == 0 ? (g.shadowMode == 1u ? 1u : 2u) : 1u;
                    static int s_traced = 0;
                    if (s_traced < 3)
                    {
                        std::printf("[vp2s-ui] glyph texture font %x index %u csa %u\n", font, g.index, csa);
                        std::fflush(stdout);
                    }
                    auto texture = glyphTexture(rdram, font, g.index, csa);
                    if (s_traced < 3)
                    {
                        ++s_traced;
                        std::printf("[vp2s-ui] glyph texture %s\n", texture ? "made" : "none");
                        std::fflush(stdout);
                    }
                    if (!texture)
                        continue;
                    const float shift = pass == 0 && g.shadowMode == 1u ? 1.0f : 0.0f;
                    // (the UV extent read before the texture is handed over: arguments are evaluated in no set order)
                    const float u1 = width / static_cast<float>(texture->width), v1 = height / static_cast<float>(texture->height);
                    drawQuad(ex + g.x0 - border + shift, ey + g.y0 - border + shift, ex + g.x1 + border + shift, ey + g.y1 + border + shift,
                             u1, v1, scaled(pass == 0 ? g.shadowColour : g.colour, colourScale, alpha), std::move(texture));
                }
            }
        }
    }
}

void vp2sInstallUi(PS2Runtime &)
{
    if (const char *gs = std::getenv("VP2S_GS"); gs && gs[0] == '1')
        return;
    if (const char *ui = std::getenv("VP2S_UI"); ui && ui[0] == '0')
        return;
    const ps2_overlays::Module *module = ps2_overlays::findModule("mod001");
    if (!module)
        return;
    const auto take = [&](uint32_t address, PS2Runtime::RecompiledFunction &original, PS2Runtime::RecompiledFunction hook) {
        if (address < module->tableBase || address >= module->tableEnd)
            return;
        original = module->table[(address - module->tableBase) >> 2];
        if (original)
            ps2_overlays::replaceModuleFunction("mod001", address, hook);
    };
    // VP2S_UI_HOOKS=<mask> (diagnosis): 1 the text start, 2 the glyphs, 4 the layer draw (all by default).
    const unsigned hooks = [] { const char *v = std::getenv("VP2S_UI_HOOKS"); return v ? static_cast<unsigned>(std::atoi(v)) : 31u; }();
    if (hooks & 1u)
        take(kTextBegin, g_textBegin, &textBeginHook);
    if (hooks & 2u)
        take(kGlyph, g_glyph, &glyphHook);
    if (hooks & 4u)
        take(kLayerDraw, g_layerDraw, &layerDrawHook);
    if (hooks & 8u)
        take(kSprite, g_sprite, &spriteHook);
    if (hooks & 16u)
    {
        take(kQuadList, g_quadList, &quadListHook);
        take(kWindowBackground, g_windowBackground, &windowBackgroundHook);
        take(kImageSetup, g_imageSetup, &imageSetupHook);
        if (std::getenv("VP2S_UI_SPRITE_LOG"))
            take(kSpriteSheet, g_spriteSheet, &spriteSheetHook);
    }
    std::printf("[vp2s] native 2D: text from the game's layouts\n");
}

// VP2's 2D layer for the native renderer from the game's own descriptions (docs/vp2_2d.md): each item an interface
// element builds (today: the text glyphs) is caught when the game lays it out, and drawn when the element's layer is
// drawn, in the layer's order after the 3D. Nothing reads the GIF packets the game builds for them.

#include "vp2s_ui.h"
#include "vp2s_loads.h"
#include "vp2s_models.h"
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
    // VP2S_UI_TREE_LOG's (layer, class) pairs already printed; cleared at each data menu state (VP2S_MENU_LOG), so
    // the log after a state shows what that state displays.
    std::vector<uint64_t> g_treeSeen;
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
        float scale = 1.0f, scaleY = 1.0f; // the border's x and y scales (item +0x10 / +0x14)
        float cellWidth = 0.0f;             // the glyph's own width in texels (item +0x1C, an int)
        uint32_t font = 0;  // the font object (EE)
        uint32_t index = 0; // glyph index in the font
        uint32_t colour = 0, shadowColour = 0, shadowMode = 0;
    };

    std::mutex g_mutex;
    std::unordered_map<uint32_t, std::vector<Glyph>> g_glyphs; // element -> its glyphs since its text began

    constexpr uint32_t kTextBegin = 0x002E8680u, kGlyph = 0x002E7E90u, kLayerDraw = 0x002EC9C0u, kSprite = 0x002EB3A0u;
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
        case 0x800u: tag = 0x4E484F4Au; index = 0x272Bu; return true; // 'JOHN' 0x272B (field HUD; 0x276F on the European disc)
        default: return false;
        }
    }

    // The game's texture manager (*0x35DE38, in the scratchpad): a list of resident textures from the byte at +0x2008
    // (next: +0x2008 + e), each with its key (+0x1008 + 16e tag, +0x8 + 16e index) and its TEX0 (+0x1010 + 16e, TBP in
    // the low 14 bits; 0x29D9B0 returns it). A sprite samples the resident texture at its TBP (+0x5A). The capture
    // hooks set g_scratch.
    const uint8_t *g_scratch = nullptr;

    // Sheets the game puts at fixed VRAM outside the manager (the UI's TBP 0 / 0x400, the menus' 0x800 / 0x1000):
    // 0x2F37B0(a0 image object, a1 its 'FIS' file, a2 PSM, a3 DBP of the pixels, t0 DBP of the palette or 0) sets up
    // the object 0x2F3470 uploads. DBP -> the FIS file's key (+0x14 tag, +0x18 index).
    constexpr uint32_t kImageSetup = 0x002F37B0u;
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

    // The resident texture at a TBP in the game's texture manager, if any.
    bool residentAt(const uint8_t *rdram, uint32_t tbp, uint32_t &tag, uint32_t &index)
    {
        const uint32_t texman = word(rdram, 0x0035DE38u);
        if (!g_scratch || (texman & 0xF0000000u) != 0x70000000u)
            return false;
        const auto scratchWord = [&](uint32_t offset) {
            uint32_t value;
            std::memcpy(&value, g_scratch + ((texman + offset) & 0x3FFFu), 4);
            return value;
        };
        uint32_t e = g_scratch[(texman + 0x2008u) & 0x3FFFu];
        for (int guard = 0; e != 0u && guard < 256; ++guard, e = g_scratch[(texman + 0x2008u + e) & 0x3FFFu])
            if ((scratchWord(0x1010u + 16u * e) & 0x3FFFu) == tbp)
            {
                tag = scratchWord(0x1008u + 16u * e);
                index = scratchWord(0x8u + 16u * e);
                return true;
            }
        return false;
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
        if (residentAt(rdram, tbp, tag, index))
            return true;
        if (const auto fixed = g_fixedSheets.find(tbp); fixed != g_fixedSheets.end())
        {
            tag = fixed->second.first;
            index = fixed->second.second;
            return true;
        }
        // The menus' sheets (their upload is not found yet; checked against the original's menus): TBP 0x800 the frame
        // ornaments 'SHI\0' 0x5E and 0x1000 the stone 'SHI\0' 0x3E, while their FIS files are in memory (the field HUD
        // uses 0x800 for 'JOHN' 0x272B otherwise, with its palette at CBP 0x3FF4: the menus' are 0x3FC8 / 0x3FCC).
        // Looked up at most every half second.
        const uint32_t cbp = word(rdram, sprite + 0x5Cu) & 0x3FFFu;
        if ((tbp == 0x800u || tbp == 0x1000u) && cbp != 0x3FF4u)
        {
            static auto s_checked = std::chrono::steady_clock::time_point{};
            static bool s_menu = false;
            if (const auto now = std::chrono::steady_clock::now(); now - s_checked > std::chrono::milliseconds(500))
            {
                s_checked = now;
                s_menu = false;
                const bool was = s_menu;
                for (uint32_t at = 0x100000u; at < 0x2000000u && !s_menu; at += 0x80u)
                    s_menu = word(rdram, at) == 0x00534946u && word(rdram, at + 0x14u) == 0x00494853u && word(rdram, at + 0x18u) == 0x5Eu;
                static const bool s_log = std::getenv("VP2S_UI_SPRITE_LOG") != nullptr;
                if (s_log && was != s_menu)
                {
                    std::printf("[vp2s-ui] menu sheets %s (TBP %x asked)\n", s_menu ? "in memory" : "gone", tbp);
                    std::fflush(stdout);
                }
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

    // 0x2EB3A0(a0 sprite = element + 0x50, ...): one quad. Sprite +0x08 parts (16 bytes: u0, v0, u1, v1 in 4.12 of the
    // sheet, w, h, +0xC CSA), +0x54 frame, +0x14 / +0x18 position, +0x2C / +0x30 scale, +0x40..+0x4C colour and
    // +0x50 alpha (128 = 1), +0x5A TBP, +0x64 CSA (negative: the part's).
    // A sprite's quad from (baseX, baseY) (its caller's position) as 0x2EB3A0 lays it out. Called under g_mutex.
    SpriteQuad spriteQuad(const uint8_t *rdram, uint32_t sprite, float baseX, float baseY)
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
        const float px = baseX + real(rdram, sprite + 0x14u), py = baseY + real(rdram, sprite + 0x18u);
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
        sheetOf(rdram, sprite, q.tag, q.index);
        return q;
    }

    void captureSprite(const uint8_t *rdram, uint32_t sprite, uint32_t position)
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_sprites[(sprite - 0x50u) & 0x1FFFFFFu] = spriteQuad(rdram, sprite, real(rdram, position), real(rdram, position + 4u));
    }

    // VP2S_UI_SPRITE_LOG: the first calls of the quad-list (0x2EAD10) and window-background (0x2E9E90) emitters, raw,
    // to read their arguments and structures before drawing them.
    constexpr uint32_t kQuadList = 0x002EAD10u, kWindowBackground = 0x002E9E90u;
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
    std::unordered_map<uint32_t, std::vector<SpriteQuad>> g_icons;     // text element -> its button icons

    // 0x2EFBA0(a0 text element): the text class's build (vtable 0x15A450). Its button icons (control 0x4019) are
    // sprites drawn by 0x2EFE40 with direct calls: element +0x100 -> +0x24 sprites of 0x70 bytes (+0x28 of them), each
    // shown when +0x66, placed by its item (+0x14 / +0x18 written there), from the layer's position plus the
    // element's (+0x18 / +0x1C) less its scroll (+0x78 / +0x7C). Captured once the build has run.
    constexpr uint32_t kTextBuild = 0x002EFBA0u;
    PS2Runtime::RecompiledFunction g_textBuild = nullptr;

    // The scrolling text class's build (0x2ECFA0: message windows, the battle tutorial) draws its icons the same way.
    constexpr uint32_t kScrollTextBuild = 0x002ECFA0u;
    PS2Runtime::RecompiledFunction g_scrollTextBuild = nullptr;

    void captureIcons(uint8_t *rdram, PS2Runtime *runtime, uint32_t element, bool building);

    void textBuildHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t element = getRegU32(ctx, 4) & 0x1FFFFFFu;
        const bool building = !rdram[(element + 0x3Bu) & 0x1FFFFFFu];
        g_textBuild(rdram, ctx, runtime);
        captureIcons(rdram, runtime, element, building);
    }

    void scrollTextBuildHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t element = getRegU32(ctx, 4) & 0x1FFFFFFu;
        const bool building = !rdram[(element + 0x3Bu) & 0x1FFFFFFu];
        g_scrollTextBuild(rdram, ctx, runtime);
        captureIcons(rdram, runtime, element, building);
    }

    void captureIcons(uint8_t *rdram, PS2Runtime *runtime, uint32_t element, bool building)
    {
        const uint32_t icons = word(rdram, element + 0x100u) & 0x1FFFFFFu;
        if (!building || !rdram[(element + 0x3Bu) & 0x1FFFFFFu] || icons < 0x100000u)
            return;
        const uint32_t layer = word(rdram, element + 0x14u) & 0x1FFFFFFu;
        const float x = real(rdram, layer + 0x40u) + real(rdram, element + 0x18u) - real(rdram, element + 0x78u);
        const float y = real(rdram, layer + 0x44u) + real(rdram, element + 0x1Cu) - real(rdram, element + 0x7Cu);
        g_scratch = runtime->memory().getScratchpad();
        std::lock_guard<std::mutex> lock(g_mutex);
        std::vector<SpriteQuad> &list = g_icons[element];
        list.clear();
        const uint32_t sprites = word(rdram, icons + 0x24u) & 0x1FFFFFFu;
        const uint32_t count = std::min<uint32_t>(word(rdram, icons + 0x28u), 64u);
        for (uint32_t i = 0; sprites >= 0x100000u && i < count; ++i)
        {
            const uint32_t sprite = sprites + 0x70u * i;
            if (rdram[sprite + 0x66u] && word(rdram, sprite + 4u))
                list.push_back(spriteQuad(rdram, sprite, x, y));
        }
    }

    // 0x2EAD10(a0 sprite): a list of quads at +0x0C, +0x58 of them, 24 bytes each: {part, x0, y0, x1, y1, z}
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

    // A window's background as 0x2E9E90(a0 window) builds it: triangle strips of four gouraud vertices (virtual units).
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

    // VP2S_UI_SPRITE_LOG: 0x2EC200(a0 sprite, a1 DTT, a2 TEX0 record or 0) gives a sprite its parts and sheet; where
    // the record and the DTT were loaded from, to key the sheets outside the texture manager (TBP 0x800, 0x1000).
    constexpr uint32_t kSpriteSheet = 0x002EC200u;
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
            const uint32_t element = (getRegU32(ctx, 4) - 0x50u) & 0x1FFFFFFu, vtable = word(rdram, element) & 0x1FFFFFFu;
            std::printf("[vp2s-ui] sprite sheet: sprite %x DTT %x (%s +%x) record %x (%s +%x): %08x %08x; element vtable %x build %x draw %x\n",
                        getRegU32(ctx, 4), dtt, d ? dttPart.c_str() : "-", dttOffset, record, r ? recordPart.c_str() : "-", recordOffset,
                        word(rdram, record), word(rdram, record + 4u), vtable, word(rdram, vtable + 0x20u), word(rdram, vtable + 0x1Cu));
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

    // 0x2EAA90(a0 element): the build of the single-sprite element class (vtable 0x15A200: the title bars, the dialog
    // arrows), which calls 0x2EB3A0 directly with its layer's position (+0x14 -> +0x40); captured here as that call
    // would be.
    constexpr uint32_t kSpriteElementBuild = 0x002EAA90u;
    PS2Runtime::RecompiledFunction g_spriteElementBuild = nullptr;

    void spriteElementBuildHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t element = getRegU32(ctx, 4) & 0x1FFFFFFu;
        if (!rdram[(element + 0x3Bu) & 0x1FFFFFFu] && (word(rdram, element + 0xC4u) || word(rdram, element + 0xB8u)) &&
            word(rdram, element + 0x54u))
        {
            g_scratch = runtime->memory().getScratchpad();
            captureSprite(rdram, element + 0x50u, (word(rdram, element + 0x14u) + 0x40u) & 0x1FFFFFFu);
            static const bool s_log = std::getenv("VP2S_UI_SPRITE_LOG") != nullptr;
            static int s_logged = 0;
            if (s_log && s_logged++ < 20)
            {
                std::lock_guard<std::mutex> lock(g_mutex);
                const SpriteQuad &q = g_sprites[element];
                std::printf("[vp2s-ui] sprite element %x built: TBP %x key %08x/%x at %.0f,%.0f-%.0f,%.0f uv %.3f,%.3f-%.3f,%.3f rgba %08x\n", element,
                            q.tbp, q.tag, q.index, q.x0, q.y0, q.x1, q.y1, q.u0, q.v0, q.u1, q.v1, q.rgba);
                std::fflush(stdout);
            }
        }
        g_spriteElementBuild(rdram, ctx, runtime);
    }

    // VP2S_UI_SPRITE_LOG: the first sprites as the game hands them to 0x2EB3A0, raw, to read their layout.
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

    // 0x2E7E90(element, item): item +0x00..0x0C box, +0x10 / +0x14 x / y scale, +0x18 font, +0x20 colour, +0x24 shadow mode, +0x28
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
        g.scaleY = real(rdram, item + 0x14u);
        g.cellWidth = static_cast<float>(static_cast<int32_t>(word(rdram, item + 0x1Cu)));
        {
            static const bool s_log = std::getenv("VP2S_UI_GLYPH_LOG") != nullptr;
            static int s_logged = 0;
            if (s_log && s_logged++ < 12)
            {
                const uint32_t font = g.font & 0x1FFFFFFu;
                std::printf("[vp2s-ui] glyph item %x: box %.1f,%.1f-%.1f,%.1f scale %.2f %.2f +1C %08x (%.2f) index %u; font +10 %x +14 %u +1C %u "
                            "+20 %u +24 %u +28 %u +2C %u\n", item, g.x0, g.y0, g.x1, g.y1, g.scale, g.scaleY, word(rdram, item + 0x1Cu),
                            g.cellWidth, g.index, word(rdram, font + 0x10u), word(rdram, font + 0x14u), word(rdram, font + 0x1Cu),
                            word(rdram, font + 0x20u), word(rdram, font + 0x24u), word(rdram, font + 0x28u), word(rdram, font + 0x2Cu));
                std::fflush(stdout);
            }
        }
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
        texture.rawAlpha = true; // the conversion's alpha is the GS's own (tools/vp2_textures.py)
        for (uint32_t y = 0; y < height; ++y)
            for (uint32_t x = 0; x < width; ++x)
            {
                const uint32_t k = y * width + x;
                const uint8_t pair = rdram[(base + k / 2u) & 0x1FFFFFFu];
                const uint32_t nibble = (k & 1u) ? pair >> 4 : pair & 15u;
                texture.rgba[static_cast<size_t>(y) * texture.width + x] = s_palette[(csa * 16u + nibble) & 0xFFu];
            }
        // VP2S_UI_GLYPH_DUMP=<dir> (diagnosis): the first glyphs as raw RGBA files, glyph_<index>_<csa>_<w>x<h>.rgba,
        // with their 4-bit source as glyph_<index>.bits (font +0x20 bytes from base).
        if (static const char *s_dump = std::getenv("VP2S_UI_GLYPH_DUMP"); s_dump && s_cache.size() < 40u)
        {
            char path[512];
            std::snprintf(path, sizeof(path), "%s/glyph_%u_%u_%ux%u.rgba", s_dump, index, csa, texture.width, texture.height);
            if (FILE *f = std::fopen(path, "wb"))
            {
                std::fwrite(texture.rgba.data(), 4, texture.rgba.size(), f);
                std::fclose(f);
            }
            std::snprintf(path, sizeof(path), "%s/glyph_%u.bits", s_dump, index);
            if (FILE *f = std::fopen(path, "wb"))
            {
                std::fwrite(rdram + (base & 0x1FFFFFFu), 1, bytes, f);
                std::fclose(f);
            }
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
        material.depthWrite = true; // the game's interface writes its Z (near): the post effect leaves it alone
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
        vp2sModelsInterfaceStarts(); // the interface from its first piece: the post passes go before it
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
        material.depthWrite = true; // the game's interface writes its Z (near): the post effect leaves it alone
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
        vp2sModelsInterfaceStarts(); // the interface from its first piece: the post passes go before it
        ps2x::native::Renderer::instance().draw(material, s_identity, triangles, 6u, nullptr);
    }

    // The same with the texture's own rectangle (u0, v0)..(u1, v1).
    // A quad with a colour per corner (top left, top right, bottom left, bottom right: a Gouraud panel).
    void drawQuadCorners(float x0, float y0, float x1, float y1, float u0, float v0, float u1, float v1, const uint32_t corners[4],
                         std::shared_ptr<const ps2x::native::TextureCache::Texture> texture)
    {
        static const float s_identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        ps2x::native::Material material;
        material.blend = ps2x::native::Material::Blend::Alpha;
        material.alphaBlend = true;
        material.depthTest = false;
        material.depthWrite = true; // the game's interface writes its Z (near): the post effect leaves it alone
        material.alphaTest = true;
        material.alphaRef = 0;
        material.textured = texture != nullptr;
        const auto vertex = [&](float vx, float vy, float u, float v, uint32_t rgba) {
            ps2x::native::Vertex out;
            out.x = 1728.0f + vx;
            out.y = 1824.0f + vy * (448.0f / 480.0f);
            out.z = 16777215.0f;
            out.u = u;
            out.v = v;
            out.rgba = rgba;
            return out;
        };
        const ps2x::native::Vertex quad[6] = {vertex(x0, y0, u0, v0, corners[0]), vertex(x1, y0, u1, v0, corners[1]),
                                              vertex(x0, y1, u0, v1, corners[2]), vertex(x1, y0, u1, v0, corners[1]),
                                              vertex(x1, y1, u1, v1, corners[3]), vertex(x0, y1, u0, v1, corners[2])};
        vp2sModelsInterfaceStarts(); // the interface from its first piece: the post passes go before it
        ps2x::native::Renderer::instance().draw(material, s_identity, quad, 6u, std::move(texture));
    }

    void drawQuadUv(float x0, float y0, float x1, float y1, float u0, float v0, float u1, float v1, uint32_t rgba,
                    std::shared_ptr<const ps2x::native::TextureCache::Texture> texture)
    {
        const uint32_t corners[4] = {rgba, rgba, rgba, rgba};
        drawQuadCorners(x0, y0, x1, y1, u0, v0, u1, v1, corners, std::move(texture));
    }

    // The renderer's captured frame over the whole screen, moved by (dx, dy), grey `colour` and `alpha` (0x80 = 1).
    void drawFrozenFrame(float dx, float dy, uint32_t colour, uint32_t alpha)
    {
        static const float s_identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        ps2x::native::Material material;
        material.blend = ps2x::native::Material::Blend::Alpha;
        material.alphaBlend = true;
        material.depthTest = false;
        material.depthWrite = true;
        material.textured = true;
        material.tex0 = ps2x::native::Renderer::kCapturedFrame;
        material.clampU = material.clampV = true;
        const uint32_t rgba = colour | colour << 8 | colour << 16 | alpha << 24;
        const auto vertex = [&](float vx, float vy, float u, float v) {
            ps2x::native::Vertex out;
            out.x = 1728.0f + vx + dx;
            out.y = 1824.0f + (vy + dy) * (448.0f / 480.0f);
            out.z = 16777215.0f;
            out.u = u;
            out.v = v;
            out.rgba = rgba;
            return out;
        };
        const ps2x::native::Vertex quad[6] = {vertex(0, 0, 0, 0), vertex(640, 0, 1, 0), vertex(0, 480, 0, 1),
                                              vertex(640, 0, 1, 0), vertex(640, 480, 1, 1), vertex(0, 480, 0, 1)};
        vp2sModelsInterfaceStarts();
        ps2x::native::Renderer::instance().draw(material, s_identity, quad, 6u);
    }

    uint32_t scaled(uint32_t rgba, uint32_t colourScale, uint32_t alpha)
    {
        const auto channel = [&](int c) { return std::min<uint32_t>(((rgba >> (8 * c)) & 0xFFu) * colourScale / 128u, 0xFFu); };
        return channel(0) | channel(1) << 8 | channel(2) << 16 | std::min<uint32_t>(alpha, 0xFFu) << 24;
    }

    // 0x2EC9C0(layer): every layer's draw. Its children with +0x39, +0x3D and +0x3B set (the game's own rule) are drawn.
    // A layer's elements drawn natively (its children list from +8, position +0x40 / +0x44, alpha +0xE0).
    void drawLayer(uint8_t *rdram, uint32_t layer)
    {
        const float layerX = real(rdram, layer + 0x40u), layerY = real(rdram, layer + 0x44u);
        // Alphas and the colour multiplier are floats, 128 = 1.
        const auto byte = [&](uint32_t address) { return static_cast<uint32_t>(std::clamp(real(rdram, address), 0.0f, 255.0f)); };
        const uint32_t layerAlpha = byte(layer + 0xE0u);
        std::lock_guard<std::mutex> lock(g_mutex);
        {
            static const bool s_tree = std::getenv("VP2S_UI_TREE_LOG") != nullptr;
            const uint64_t pair = uint64_t{layer} << 32 | 0xFFFFFFFFu;
            if (s_tree && std::find(g_treeSeen.begin(), g_treeSeen.end(), pair) == g_treeSeen.end() && g_treeSeen.size() < 400u)
            {
                g_treeSeen.push_back(pair);
                std::printf("[vp2s-ui] layer %x (vtable %x) at %.0f,%.0f alpha %u first child %x:", layer, word(rdram, layer), layerX, layerY,
                            layerAlpha, word(rdram, layer + 8u));
                for (uint32_t k = 0; k < 0x100u; k += 4u)
                    std::printf(" %08x", word(rdram, layer + k));
                std::printf("\n");
                std::fflush(stdout);
            }
        }
        // The children list (next at +8) may come back round to its start or the layer: each child once.
        std::vector<uint32_t> seen;
        uint32_t element = word(rdram, layer + 8u);
        for (int guard = 0; element != 0u && guard < 512; ++guard, element = word(rdram, element + 8u))
        {
            element &= 0x1FFFFFFu;
            if (element == layer || std::find(seen.begin(), seen.end(), element) != seen.end())
                break;
            seen.push_back(element);
            // VP2S_UI_TREE_LOG: each element class (vtable) each layer holds, once, with its build method and what the
            // native port has of it (sprite, quad list, window, glyphs) and whether it is shown.
            {
                static const bool s_tree = std::getenv("VP2S_UI_TREE_LOG") != nullptr;
                const uint32_t vtable = word(rdram, element) & 0x1FFFFFFu;
                const uint64_t pair = uint64_t{layer} << 32 | vtable;
                if (s_tree && std::find(g_treeSeen.begin(), g_treeSeen.end(), pair) == g_treeSeen.end() && g_treeSeen.size() < 400u)
                {
                    g_treeSeen.push_back(pair);
                    std::printf("[vp2s-ui] class %x (build %x, draw %x): element %x layer %x shown %d%d%d; sprite %d quads %d window %d glyphs %d; "
                                "at %.0f,%.0f size %.0f,%.0f\n",
                                vtable, word(rdram, vtable + 0x20u), word(rdram, vtable + 0x1Cu), element, layer, rdram[element + 0x39u],
                                rdram[element + 0x3Du], rdram[element + 0x3Bu], g_sprites.count(element) ? 1 : 0, g_quadLists.count(element) ? 1 : 0,
                                g_windows.count(element) ? 1 : 0, g_glyphs.count(element) ? 1 : 0, real(rdram, element + 0x18u),
                                real(rdram, element + 0x1Cu), real(rdram, element + 0x20u), real(rdram, element + 0x24u));
                    std::fflush(stdout);
                }
            }
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
            // The prompt's backing bar (class 0x165C30, mod003 build 0x4489B0, drawn each frame here from live data): a
            // rounded cap (part 22) then a body stretched to the screen's right edge (part 23) of the field HUD sheet
            // (TBP 0x800, CBP 0x3FF4: 'JOHN' 0x272B), parts at *(*(*(0x48E048) + 0x5C) + 0x224) (u0, v0, u1, v1 in 4.12,
            // w, h); left edge at the layer's x + +0x58 + 5, top at its y + +0x5C + 2, h + 4 high, the cap h (x 0.75
            // when bit 4 of *(*(0x48E04C) + 3)) GS pixels wide.
            // The Gouraud panel (class 0x167560, mod003 build 0x460D20: the data menus' footer bar and its line): at the
            // layer's position + (+0x18, +0x1C), +0x20 x +0x24, corner colours +0x50 / +0x54 / +0x58 / +0x5C (top left,
            // top right, bottom left, bottom right; 0x80 = 128), alpha the layer's x +0x28 / 128, untextured.
            if ((word(rdram, element) & 0x1FFFFFFu) == 0x167560u)
            {
                const float x0 = layerX + real(rdram, element + 0x18u), y0 = layerY + real(rdram, element + 0x1Cu);
                const uint32_t alpha = std::min<uint32_t>(layerAlpha * byte(element + 0x28u) / 128u, 255u);
                uint32_t corners[4];
                for (uint32_t c = 0; c < 4u; ++c)
                    corners[c] = (word(rdram, element + 0x50u + 4u * c) & 0xFFFFFFu) | alpha << 24;
                drawQuadCorners(x0, y0, x0 + real(rdram, element + 0x20u), y0 + real(rdram, element + 0x24u), 0.0f, 0.0f, 0.0f, 0.0f,
                                corners, nullptr);
            }
            // The field pause's logo (class 0x166EF0, mod003 build 0x451450 -> 0x4514F0): part 3 of the field HUD sheet
            // (as the prompt bar below), at the layer's position + (73, 79), its w x h GS pixels (the GS frame is
            // 512 x 512.96 over the 640 x 480 screen), alpha the layer's.
            if ((word(rdram, element) & 0x1FFFFFFu) == 0x166EF0u)
            {
                const uint32_t holder = word(rdram, (word(rdram, 0x0048E048u) & 0x1FFFFFFu) + 0x5Cu) & 0x1FFFFFFu;
                const uint32_t parts = holder >= 0x100000u ? word(rdram, holder + 0x224u) & 0x1FFFFFFu : 0u;
                if (parts >= 0x100000u)
                {
                    const uint32_t part = parts + 0x30u;
                    const auto half = [&](uint32_t address) { return static_cast<float>(word(rdram, address) & 0xFFFFu); };
                    SpriteQuad logo;
                    logo.u0 = half(part) / 4096.0f;
                    logo.v0 = half(part + 2u) / 4096.0f;
                    logo.u1 = half(part + 4u) / 4096.0f;
                    logo.v1 = half(part + 6u) / 4096.0f;
                    logo.x0 = layerX + 73.0f;
                    logo.y0 = layerY + 79.0f;
                    logo.x1 = logo.x0 + half(part + 8u) * (640.0f / 512.0f);
                    logo.y1 = logo.y0 + half(part + 0xAu) * (480.0f / 512.96f);
                    logo.rgba = 0x80808080u;
                    logo.tbp = 0x800u;
                    logo.tag = 0x4E484F4Au;
                    logo.index = 0x272Bu;
                    drawSprite(logo);
                }
            }
            // The field pause's bottom bar (class 0x1670A0, mod003 build 0x451960 -> 0x451A60), at the layer's position:
            // black, 512 x 128 GS pixels, then a line 4 high from 0.19 down, (67, 18, 43) to (189, 81, 43) over 160
            // pixels and back to (67, 18, 43) at 982 (GS pixels; the frame is 512 x 512.96 over the 640 x 480 screen).
            if ((word(rdram, element) & 0x1FFFFFFu) == 0x1670A0u)
            {
                constexpr float sx = 640.0f / 512.0f, sy = 480.0f / 512.96f;
                const uint32_t black[4] = {0x80000000u, 0x80000000u, 0x80000000u, 0x80000000u};
                drawQuadCorners(layerX, layerY, layerX + 512.0f * sx, layerY + 128.0f * sy, 0.0f, 0.0f, 0.0f, 0.0f, black, nullptr);
                const uint32_t dark = 0x802B1243u, light = 0x802B51BDu;
                const float y0 = layerY + 0.19f * sy, y1 = layerY + 4.19f * sy;
                const uint32_t rising[4] = {dark, light, dark, light}, falling[4] = {light, dark, light, dark};
                drawQuadCorners(layerX, y0, layerX + 160.0f * sx, y1, 0.0f, 0.0f, 0.0f, 0.0f, rising, nullptr);
                drawQuadCorners(layerX + 160.0f * sx, y0, layerX + 982.0f * sx, y1, 0.0f, 0.0f, 0.0f, 0.0f, falling, nullptr);
            }
            if ((word(rdram, element) & 0x1FFFFFFu) == 0x165C30u)
            {
                const uint32_t holder = word(rdram, (word(rdram, 0x0048E048u) & 0x1FFFFFFu) + 0x5Cu) & 0x1FFFFFFu;
                const uint32_t parts = holder >= 0x100000u ? word(rdram, holder + 0x224u) & 0x1FFFFFFu : 0u;
                if (parts >= 0x100000u)
                {
                    const auto half = [&](uint32_t address) { return static_cast<float>(word(rdram, address) & 0xFFFFu); };
                    const float h = half(parts + 0x16Au);
                    const bool narrow = (rdram[((word(rdram, 0x0048E04Cu) & 0x1FFFFFFu) + 3u) & 0x1FFFFFFu] >> 4) & 1u;
                    const float capWidth = (narrow ? h * 0.75f : h) * (640.0f / 512.0f);
                    const float x0 = layerX + real(rdram, element + 0x58u) + 5.0f, y0 = layerY + real(rdram, element + 0x5Cu) + 2.0f;
                    SpriteQuad cap, body;
                    for (SpriteQuad *q : {&cap, &body})
                    {
                        const uint32_t part = parts + (q == &cap ? 0x160u : 0x170u);
                        q->u0 = half(part) / 4096.0f;
                        q->v0 = half(part + 2u) / 4096.0f;
                        q->u1 = half(part + 4u) / 4096.0f;
                        q->v1 = half(part + 6u) / 4096.0f;
                        q->y0 = y0;
                        q->y1 = y0 + h + 4.0f;
                        q->rgba = 0x80808080u;
                        q->tbp = 0x800u;
                        q->tag = 0x4E484F4Au;
                        q->index = 0x272Bu;
                    }
                    cap.x0 = x0 - capWidth;
                    cap.x1 = x0;
                    body.x0 = x0;
                    body.x1 = 640.0f;
                    drawSprite(cap);
                    drawSprite(body);
                }
            }
            if (const auto window = g_windows.find(element); window != g_windows.end())
                for (const auto &strip : window->second)
                    drawStrip(strip);
            if (const auto sprite = g_sprites.find(element); sprite != g_sprites.end())
                drawSprite(sprite->second);
            if (const auto list = g_quadLists.find(element); list != g_quadLists.end())
                for (const SpriteQuad &q : list->second)
                    drawSprite(q);
            if (const auto icons = g_icons.find(element); icons != g_icons.end())
                for (const SpriteQuad &q : icons->second)
                    drawSprite(q);
            const auto found = g_glyphs.find(element);
            if (found == g_glyphs.end())
                continue;
            // The layer's position plus the element's, less its scroll (+0x78 / +0x7C), as 0x2E7E90 places it.
            const float ex = layerX + real(rdram, element + 0x18u) - real(rdram, element + 0x78u);
            const float ey = layerY + real(rdram, element + 0x1Cu) - real(rdram, element + 0x7Cu);
            const uint32_t alpha = layerAlpha * byte(element + 0x28u) / 128u;
            const uint32_t colourScale = byte(element + 0x90u);
            for (const Glyph &g : found->second)
            {
                const uint32_t font = g.font & 0x1FFFFFFu;
                const float border = static_cast<float>(word(rdram, font + 0x2Cu)) * g.scale;
                const float borderY = static_cast<float>(word(rdram, font + 0x2Cu)) * g.scaleY;
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
                    // The glyph's own cell (0x2E7E90's UVs): its width (item +0x1C) and the font's height (+0x14), each with
                    // the border (+0x2C) on both sides; the rest of the 32-wide cell is not drawn. (Read before the
                    // texture is handed over: arguments are evaluated in no set order.)
                    const float pad = 2.0f * static_cast<float>(word(rdram, font + 0x2Cu));
                    const float cellW = g.cellWidth > 0.0f ? std::min(static_cast<float>(static_cast<int>(g.cellWidth)) + pad, width) : width;
                    const float cellH = std::min(static_cast<float>(word(rdram, font + 0x14u)) + pad, height);
                    const float u1 = cellW / static_cast<float>(texture->width), v1 = (cellH > 0.0f ? cellH : height) / static_cast<float>(texture->height);
                    drawQuad(ex + g.x0 - border + shift, ey + g.y0 - borderY + shift, ex + g.x1 + border + shift, ey + g.y1 + borderY + shift,
                             u1, v1, scaled(pass == 0 ? g.shadowColour : g.colour, colourScale, alpha), std::move(texture));
                }
            }
        }
    }

    // When each layer last went through 0x2EC9C0 (the dialog container below leaves those alone).
    std::unordered_map<uint32_t, std::chrono::steady_clock::time_point> g_layerDrawnAt;

    void layerDrawHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t layer = getRegU32(ctx, 4) & 0x1FFFFFFu;
        vp2sModelsInterfaceStarts();
        g_layerDraw(rdram, ctx, runtime);
        g_layerDrawnAt[layer] = std::chrono::steady_clock::now();
        drawLayer(rdram, layer);
    }

    // mod003 0x45E570: the draw of the menus' dialog container (vtable 0x1672D0, no children of its own). Its manager
    // (container + 0xB0, class 0x167590) lists its windows from +0x4C; a dialog window (class 0x1675B0: "load this
    // file?", "checking the card", "loading") has its layer at +0xB0 (vtable 0x167648), built as any layer (0x2ECA00)
    // but queued with the container (0x2EF390(a0 + 0xB4, a0 + 0x90)), never through 0x2EC9C0: drawn here.
    constexpr uint32_t kDialogLayerDraw = 0x0045E570u;
    PS2Runtime::RecompiledFunction g_dialogLayerDraw = nullptr;

    void dialogLayerDrawHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t container = getRegU32(ctx, 4) & 0x1FFFFFFu;
        vp2sModelsInterfaceStarts();
        g_dialogLayerDraw(rdram, ctx, runtime);
        // The container's backdrop (its packet at +0xBC, built once by 0x45E5B0, +0xC8 set): the stone over the whole
        // screen, 512 x 384 texels of the sheet at TBP 0x800 (PSMT4HL): 'SHI\0' 0x49 (rows 0..255) over 0x4A (256..383).
        // The interface's space is 640 x 480 (the GS's 448 lines).
        if (rdram[container + 0xC8u] != 0u)
        {
            // UV half a texel in from each edge, as the game's (0.5 .. 511.5): the filter does not reach across the
            // seam between the two pieces (a line at their join otherwise).
            const float split = 480.0f * 256.0f / 384.0f, u0 = 0.5f / 512.0f, u1 = 1.0f - u0;
            if (auto top = vp2sGameTexture(0x00494853u, 0x49u, 0u))
                drawQuadUv(0.0f, 0.0f, 640.0f, split, u0, 0.5f / 256.0f, u1, 1.0f - 0.5f / 256.0f, 0x80808080u, std::move(top));
            if (auto bottom = vp2sGameTexture(0x00494853u, 0x4Au, 0u))
                drawQuadUv(0.0f, split, 640.0f, 480.0f, u0, 0.5f / 128.0f, u1, 1.0f - 0.5f / 128.0f, 0x80808080u, std::move(bottom));
        }
        const uint32_t manager = word(rdram, container + 0xB0u) & 0x1FFFFFFu;
        if (manager < 0x100000u)
            return;
        for (uint32_t slot = 0x4Cu; slot < 0x80u; slot += 4u)
        {
            const uint32_t window = word(rdram, manager + slot) & 0x1FFFFFFu;
            static const bool s_tree = std::getenv("VP2S_UI_TREE_LOG") != nullptr;
            const uint64_t pair = uint64_t{window} << 32 | 0xFFFFFFFEu;
            if (s_tree && window >= 0x100000u && std::find(g_treeSeen.begin(), g_treeSeen.end(), pair) == g_treeSeen.end() && g_treeSeen.size() < 400u)
            {
                g_treeSeen.push_back(pair);
                std::printf("[vp2s-ui] dialog manager %x slot %x: window %x class %x, +0xB0 %x\n", manager, slot, window, word(rdram, window),
                            word(rdram, window + 0xB0u));
                std::fflush(stdout);
            }
            // Its windows' layers (+0xB0) the 2D manager did not draw itself in the last 0.1 s: the container's.
            if (window < 0x100000u || word(rdram, window + 0xB0u) < 0x150000u || word(rdram, window + 0xB0u) >= 0x170000u)
                continue;
            const auto drawn = g_layerDrawnAt.find(window + 0xB0u);
            if (drawn != g_layerDrawnAt.end() && std::chrono::steady_clock::now() - drawn->second < std::chrono::milliseconds(100))
                continue;
            drawLayer(rdram, window + 0xB0u);
        }
    }
}

bool vp2sUiFixedSheet(uint32_t dbp, uint32_t &tag, uint32_t &index)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const auto found = g_fixedSheets.find(dbp & 0x3FFFu);
    if (found == g_fixedSheets.end())
        return false;
    tag = found->second.first;
    index = found->second.second;
    return true;
}

void vp2sUiNoteFixedSheet(uint32_t dbp, uint32_t tag, uint32_t index)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    g_fixedSheets[dbp & 0x3FFFu] = {tag, index};
}

void vp2sUiFrozenFrame(uint8_t *rdram, uint32_t scene)
{
    // The scene's master list (+0x7C, count +0xFB80) with an object of class 0x1670D0 in it, not frozen or hidden
    // (+0x6A & 0x49, the cull's own test at 0x2207EC).
    const uint32_t list = word(rdram, scene + 0x7Cu) & 0x1FFFFFFu, count = std::min(word(rdram, scene + 0xFB80u), 0x2000u);
    uint32_t frozen = 0;
    for (uint32_t i = 0; list >= 0x100000u && i < count && !frozen; ++i)
    {
        const uint32_t object = word(rdram, list + 4u * i) & 0x1FFFFFFu;
        if (object >= 0x100000u && (word(rdram, object) & 0x1FFFFFFu) == 0x1670D0u && !(rdram[object + 0x6Au] & 0x49u))
            frozen = object;
    }
    // Captured as the object appears (the frame on screen is the last one without it), not again while it stays.
    static uint32_t s_object = 0;
    const uint32_t before = s_object;
    s_object = frozen;
    if (!frozen)
        return;
    if (frozen != before)
    {
        ps2x::native::Renderer::instance().captureFrame();
        std::printf("[vp2s-ui] frozen frame %x: the frame on screen captured\n", frozen);
        std::fflush(stdout);
    }
    drawFrozenFrame(0.0f, 0.0f, 0x70u, 0x80u);
    for (int k = 1; k <= 16; ++k)
        drawFrozenFrame(-k * 640.0f / 512.0f, -2.0f * k * 480.0f / 512.96f, 0x50u, 0x0Cu);
}

namespace
{
    // VP2S_MENU_LOG: the data menu's state machine (mod016 0x4992A0(a0 menu): state +0x70; its dialog *(menu + 4),
    // +0x104 0 once answered, +0x10C the answer, 1 yes) logged at each change of state, for scripted runs to wait on
    // ("[vp2s-menu] state 19"). 0x19 asks to load the slot chosen (the read only with yes), 0x1A reads, 0x1B applies.
    constexpr uint32_t kDataMenu = 0x004992A0u;
    PS2Runtime::RecompiledFunction g_dataMenu = nullptr;

    void dataMenuHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t menu = getRegU32(ctx, 4) & 0x1FFFFFFu;
        g_dataMenu(rdram, ctx, runtime);
        static uint32_t s_last = 0xFFFFFFFFu;
        const uint32_t state = word(rdram, menu + 0x70u), dialog = word(rdram, menu + 4u) & 0x1FFFFFFu;
        if (state != s_last)
        {
            s_last = state;
            g_treeSeen.clear();
            std::printf("[vp2s-menu] state %x (dialog %x answer %d) dialog object %x class %x:", state,
                        static_cast<int32_t>(word(rdram, dialog + 0x104u)), static_cast<int32_t>(word(rdram, dialog + 0x10Cu)), dialog,
                        word(rdram, dialog));
            for (uint32_t k = 0; k < 0x180u; k += 4u)
                std::printf(" %08x", word(rdram, dialog + k));
            std::printf("\n");
            std::fflush(stdout);
        }
    }
}

void vp2sInstallUi(PS2Runtime &)
{
    if (std::getenv("VP2S_MENU_LOG"))
        if (const ps2_overlays::Module *data = ps2_overlays::findModule("mod016");
            data && kDataMenu >= data->tableBase && kDataMenu < data->tableEnd)
        {
            g_dataMenu = data->table[(kDataMenu - data->tableBase) >> 2];
            if (g_dataMenu)
                ps2_overlays::replaceModuleFunction("mod016", kDataMenu, &dataMenuHook);
        }
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
    {
        take(kLayerDraw, g_layerDraw, &layerDrawHook);
        if (const ps2_overlays::Module *menus = ps2_overlays::findModule("mod003");
            menus && kDialogLayerDraw >= menus->tableBase && kDialogLayerDraw < menus->tableEnd)
        {
            g_dialogLayerDraw = menus->table[(kDialogLayerDraw - menus->tableBase) >> 2];
            if (g_dialogLayerDraw)
                ps2_overlays::replaceModuleFunction("mod003", kDialogLayerDraw, &dialogLayerDrawHook);
        }
    }
    if (hooks & 8u)
        take(kSprite, g_sprite, &spriteHook);
        take(kSpriteElementBuild, g_spriteElementBuild, &spriteElementBuildHook);
        take(kTextBuild, g_textBuild, &textBuildHook);
        take(kScrollTextBuild, g_scrollTextBuild, &scrollTextBuildHook);
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

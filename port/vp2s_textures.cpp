// VP2's textures for the native renderer from the game's own texture code, not from GS transfers: the game's VRAM
// texture cache (mod001 FUN_0029d8f0, a0 its object, a2 / a3 the texture's key: file tag and index, v0 the VRAM
// address it gives, TBP = v0 >> 6) is hooked to note which texture the game puts where; a draw's TEX0 then names its
// texture by TBP, and the picture comes from the offline conversion of the game's files (tools/vp2_textures.py:
// <dir>/<tag>_<index>.tex), with the CSA the TEX0 picks. Nothing reads GS packets or transfer blocks while the game
// runs. VP2S_TEXTURES=<dir> (default build/vp2_textures next to the executable's build tree); VP2S_GAME_TEXTURES=1
// turns it on (not yet the default, see vp2sInstallGameTextures).

#include "vp2s_textures.h"
#include "vp2s_loads.h"

#include "ps2_overlay_modules.h"
#include "ps2_runtime.h"
#include "runtime/native/native_texture_cache.h"

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
    constexpr uint32_t kTextureCache = 0x0029D9B0u; // mod001

    struct Key
    {
        uint32_t tag = 0, index = 0;
    };

    struct Converted
    {
        uint32_t width = 0, height = 0, psm = 0;
        std::vector<uint8_t> texels;      // indices (T4 / T8) or RGBA8 (CT32)
        std::array<uint32_t, 256> palette{}; // RGBA8, CSM1 resolved
    };

    std::mutex g_mutex;
    std::unordered_map<uint32_t, Key> g_placed;                          // TBP -> the texture the game put there
    std::unordered_map<uint64_t, std::shared_ptr<const ps2x::native::TextureCache::Texture>> g_textures; // + TEX0 bits
    uint64_t g_version = 0;
    PS2Runtime::RecompiledFunction g_original = nullptr;
    uint32_t word(const uint8_t *rdram, uint32_t address);
    void noteCopy(const Key &key, uint32_t header);

    std::string directory()
    {
        if (const char *dir = std::getenv("VP2S_TEXTURES"))
            return dir;
        return "../vp2_textures";
    }

    // The game's texture cache: the texture with key (a2, a3) is at the VRAM address it returns.
    void textureCacheHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t tag = getRegU32(ctx, 6), index = getRegU32(ctx, 7);
        g_original(rdram, ctx, runtime);
        const uint32_t address = getRegU32(ctx, 2);
        if (address == 0u)
            return;
        std::lock_guard<std::mutex> lock(g_mutex);
        g_placed[(address >> 6) & 0x3FFFu] = {tag, index};
    }

    // The game's VRAM block allocator (mod001 FUN_0023da80): every texture that is not resident comes here, from
    // the texture cache (FUN_0029d8f0's miss path), the per-model texture update FUN_0023cb20 (the field's map
    // textures, sent again whenever the small VRAM drops them) and the character models (FUN_0022d150). a2 / a3 the
    // key (read before the call: it changes a3), v0 the VRAM address (0 failure, 1 no room: the caller evicts and
    // calls again).
    constexpr uint32_t kTextureAllocate = 0x0023DB00u;
    PS2Runtime::RecompiledFunction g_allocate = nullptr;

    void textureAllocateHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t tag = getRegU32(ctx, 6), index = getRegU32(ctx, 7), header = getRegU32(ctx, 5);
        g_allocate(rdram, ctx, runtime);
        const uint32_t address = getRegU32(ctx, 2);
        if (address < 2u)
            return;
        std::lock_guard<std::mutex> lock(g_mutex);
        g_placed[(address >> 6) & 0x3FFFu] = {tag, index};
        noteCopy({tag, index}, header);
    }

    // The game's CLUT allocator (mod001 FUN_0023d810): a palette goes to VRAM on its own, keyed by the FIS info
    // block in a1 (+4 tag, +8 index), often another texture's than the one drawn with it. v0 the VRAM address (0: it
    // is resident already). TEX0's CBP then names the palette by that key.
    constexpr uint32_t kClutAllocate = 0x0023D890u;
    PS2Runtime::RecompiledFunction g_clutAllocate = nullptr;
    std::unordered_map<uint32_t, Key> g_cluts; // CBP -> the palette the game put there

    void clutAllocateHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t info = getRegU32(ctx, 5);
        Key key;
        std::memcpy(&key.tag, rdram + ((info + 4u) & 0x1FFFFFFu), 4);
        std::memcpy(&key.index, rdram + ((info + 8u) & 0x1FFFFFFu), 4);
        g_clutAllocate(rdram, ctx, runtime);
        const uint32_t address = getRegU32(ctx, 2);
        static const bool s_log = std::getenv("VP2S_GAME_TEXTURES_LOG") != nullptr;
        static int s_logged = 0;
        if (s_log && s_logged < 40)
        {
            ++s_logged;
            std::printf("[vp2s-textures] clut %08x_%x at %x (cbp %x)\n", key.tag, key.index, address, (address >> 6) & 0x3FFFu);
        }
        if (address < 2u)
            return;
        std::lock_guard<std::mutex> lock(g_mutex);
        g_cluts[(address >> 6) & 0x3FFFu] = key;
        noteCopy(key, info + word(rdram, info + 0xCu));
    }

    // Which copy of a key: many keys are in several of the game's files (one per map, a palette tinted for each), so
    // the conversion names each copy by where it is on the disc, and the game's own loading (vp2s_loads.cpp) says which
    // one it took: a texture's header address gives (asset part, offset), and <dir>/variants.txt the copy's file.
    std::unordered_map<std::string, std::string> g_variants; // "<chain>_<part> <offset>" -> file name
    std::unordered_map<uint64_t, std::string> g_keyFile;      // (tag << 32 | index) -> the copy the game loaded

    uint32_t word(const uint8_t *rdram, uint32_t address)
    {
        uint32_t value;
        std::memcpy(&value, rdram + (address & 0x1FFFFFCu), 4);
        return value;
    }

    // The copy of the texture whose header is at `header`: its file, or empty.
    std::string copyAt(uint32_t header)
    {
        std::string part;
        uint32_t offset = 0;
        if (!vp2sLoadedFrom(header, part, offset))
            return {};
        char name[64];
        std::snprintf(name, sizeof(name), "%s %x", part.c_str(), offset);
        const auto it = g_variants.find(name);
        return it != g_variants.end() ? it->second : std::string();
    }

    void noteCopy(const Key &key, uint32_t header)
    {
        std::string file = copyAt(header);
        // VP2S_GAME_TEXTURES_LOG: the first copies looked for.
        static const bool s_log = std::getenv("VP2S_GAME_TEXTURES_LOG") != nullptr;
        static int s_logged = 0;
        if (s_log && s_logged < 40)
        {
            ++s_logged;
            std::printf("[vp2s-textures] copy of %08x_%x at header %x: %s\n", key.tag, key.index, header,
                        file.empty() ? "none" : file.c_str());
        }
        if (!file.empty())
            g_keyFile[(uint64_t{key.tag} << 32) | key.index] = std::move(file);
    }

    std::string fileOf(const Key &key)
    {
        if (const auto it = g_keyFile.find((uint64_t{key.tag} << 32) | key.index); it != g_keyFile.end())
            return it->second;
        char name[64];
        std::snprintf(name, sizeof(name), "%08x_%x.tex", key.tag, key.index);
        return name;
    }

    std::unordered_map<std::string, std::shared_ptr<Converted>> g_loaded; // file -> its conversion

    std::shared_ptr<Converted> load(const Key &key)
    {
        const std::string name = fileOf(key);
        if (const auto it = g_loaded.find(name); it != g_loaded.end())
            return it->second;
        std::shared_ptr<Converted> converted;
        if (FILE *f = std::fopen((directory() + "/" + name).c_str(), "rb"))
        {
            converted = std::make_shared<Converted>();
            uint32_t head[3] = {};
            if (std::fread(head, 4, 3, f) == 3)
            {
                converted->width = head[0];
                converted->height = head[1];
                converted->psm = head[2];
                converted->texels.resize(static_cast<size_t>(head[0]) * head[1] * (head[2] == 0u ? 4u : 1u));
                if (std::fread(converted->texels.data(), 1, converted->texels.size(), f) != converted->texels.size() ||
                    std::fread(converted->palette.data(), 4, 256, f) != 256)
                    converted.reset();
            }
            else
                converted.reset();
            std::fclose(f);
        }
        g_loaded[name] = converted; // a missing file is remembered too
        return converted;
    }

    std::shared_ptr<const ps2x::native::TextureCache::Texture> provide(uint64_t tex0, uint64_t)
    {
        const uint32_t tbp = static_cast<uint32_t>(tex0 & 0x3FFFu);
        std::lock_guard<std::mutex> lock(g_mutex);
        // VP2S_GAME_TEXTURES_LOG: every 100000 lookups, how many found a placed texture with a converted file.
        static const bool s_log = std::getenv("VP2S_GAME_TEXTURES_LOG") != nullptr;
        static uint64_t s_lookups = 0, s_unplaced = 0, s_unconverted = 0;
        if (s_log && ++s_lookups % 100000u == 0u)
            std::printf("[vp2s-textures] %llu lookups: %llu at a TBP the game's cache did not give, %llu not converted, %zu placed, %zu files\n",
                        static_cast<unsigned long long>(s_lookups), static_cast<unsigned long long>(s_unplaced),
                        static_cast<unsigned long long>(s_unconverted), g_placed.size(), g_loaded.size());
        const auto placed = g_placed.find(tbp);
        if (placed == g_placed.end())
        {
            ++s_unplaced;
            return nullptr;
        }
        const uint32_t csa = static_cast<uint32_t>((tex0 >> 56) & 0x1Fu);
        const uint32_t tw = static_cast<uint32_t>((tex0 >> 26) & 0xFu), th = static_cast<uint32_t>((tex0 >> 30) & 0xFu);
        const uint32_t cbp = static_cast<uint32_t>((tex0 >> 37) & 0x3FFFu);
        const uint32_t psm = static_cast<uint32_t>((tex0 >> 20) & 0x3Fu);
        const auto clut = (psm == 0x13u || psm == 0x14u) ? g_cluts.find(cbp) : g_cluts.end();
        // The texture drawn: the key the palette was allocated with (an indexed texture's palette comes from the same
        // info block as the texture; it also names a texture placed where the TBP still names an older one), else the
        // TBP's. The game notes both a frame before the draws use them, so a key counts only when its conversion has
        // TEX0's format and size (1 << tw by 1 << th, the conversion at most that and more than half).
        const auto fits = [&](const Key &key) {
            const auto file = load(key);
            const uint32_t w = 1u << std::min<uint32_t>(10u, tw), h = 1u << std::min<uint32_t>(10u, th);
            return file && file->psm == psm && file->width <= w && file->width * 2u > w && file->height <= h && file->height * 2u > h;
        };
        Key textureKey;
        if (clut != g_cluts.end() && fits(clut->second))
            textureKey = clut->second;
        else if (fits(placed->second))
            textureKey = placed->second;
        else
        {
            ++s_unconverted;
            static std::unordered_map<uint64_t, uint64_t> s_misses;
            if (s_log && s_misses.size() < 100000u)
                ++s_misses[tex0 & 0x00FFFFFFFFFFFFFFull];
            // VP2S_GAME_TEXTURES_LOG: every 2000000 misses, the TEX0s (without CLD / CSA) most often without a texture.
            if (s_log && s_unconverted % 2000000u == 0u)
            {
                std::vector<std::pair<uint64_t, uint64_t>> top(s_misses.begin(), s_misses.end());
                std::sort(top.begin(), top.end(), [](const auto &a, const auto &b) { return a.second > b.second; });
                for (size_t i = 0; i < top.size() && i < 8; ++i)
                    std::printf("[vp2s-textures] most missed tex0 %016llx: %llu\n", static_cast<unsigned long long>(top[i].first),
                                static_cast<unsigned long long>(top[i].second));
            }
            static std::unordered_map<uint64_t, int> s_failed;
            if (s_log && psm != 0u && s_failed.size() < 200 && s_failed.emplace(tex0 & 0x00FFFFFFFFFFFFFFull, 0).second)
            {
                const auto describe = [&](const Key &key) {
                    const auto file = load(key);
                    char text[160];
                    std::snprintf(text, sizeof(text), "%08x_%x -> %s %ux%u psm %x", key.tag, key.index, fileOf(key).c_str(),
                                  file ? file->width : 0u, file ? file->height : 0u, file ? file->psm : 0xFFu);
                    return std::string(text);
                };
                std::printf("[vp2s-textures] tex0 %016llx (%ux%u psm %x): no conversion fits; palette: %s; tbp: %s\n",
                            static_cast<unsigned long long>(tex0), 1u << tw, 1u << th, psm,
                            clut != g_cluts.end() ? describe(clut->second).c_str() : "none", describe(placed->second).c_str());
            }
            return nullptr;
        }
        const Key paletteKey = textureKey;
        const uint64_t id = std::hash<std::string>()(fileOf(textureKey)) * 0x9E3779B97F4A7C15ull ^ (uint64_t{csa} << 8 | tw << 4 | th);
        if (const auto it = g_textures.find(id); it != g_textures.end())
            return it->second;
        const std::shared_ptr<Converted> converted = load(textureKey);
        // A texture of another format at that TBP (a picture the game drew there, text it built) is not the converted one.
        if (!converted || converted->psm != psm)
        {
            ++s_unconverted;
            g_textures[id] = nullptr;
            return nullptr;
        }
        std::shared_ptr<Converted> paletteFile = converted;
        if (converted->psm != 0u && clut != g_cluts.end())
            if (auto other = load(paletteKey))
                paletteFile = other;
        const auto &palette = paletteFile->palette;
        ps2x::native::TextureCache::Texture texture;
        texture.width = 1u << std::min<uint32_t>(10u, tw);
        texture.height = 1u << std::min<uint32_t>(10u, th);
        texture.key = id;
        texture.version = ++g_version;
        texture.rgba.assign(static_cast<size_t>(texture.width) * texture.height, 0u);
        texture.rawAlpha = true; // the conversion's alpha is the GS's own (tools/vp2_textures.py)
        for (uint32_t y = 0; y < texture.height && y < converted->height; ++y)
            for (uint32_t x = 0; x < texture.width && x < converted->width; ++x)
            {
                const size_t k = static_cast<size_t>(y) * converted->width + x;
                uint32_t rgba;
                if (converted->psm == 0u)
                    std::memcpy(&rgba, converted->texels.data() + k * 4u, 4u);
                else
                {
                    const uint32_t index = converted->texels[k];
                    rgba = palette[(converted->psm == 0x14u ? csa * 16u + index : index) & 0xFFu];
                }
                texture.rgba[static_cast<size_t>(y) * texture.width + x] = rgba;
            }
        auto shared = std::make_shared<const ps2x::native::TextureCache::Texture>(std::move(texture));
        g_textures[id] = shared;
        // VP2S_GAME_TEXTURES_COMPARE: each new texture against the one the uploads made (diagnosis only): the keys
        // whose conversion differs, with how much.
        static const bool s_compare = std::getenv("VP2S_GAME_TEXTURES_COMPARE") != nullptr;
        if (s_compare)
            if (const auto reference = ps2x::native::textures().uploadedTexture(tex0, 0); reference && reference->rgba.size() == shared->rgba.size())
            {
                size_t wrong = 0;
                for (size_t i = 0; i < shared->rgba.size(); ++i)
                    wrong += reference->rgba[i] != shared->rgba[i];
                std::printf("[vp2s-textures] %08x_%x tex0 %016llx palette %08x_%x%s: %zu of %zu texels differ%s\n", textureKey.tag,
                            textureKey.index, static_cast<unsigned long long>(tex0), paletteKey.tag, paletteKey.index,
                            clut != g_cluts.end() ? "" : " (own)", wrong, shared->rgba.size(), wrong ? "" : " (same)");
                // VP2S_GAME_TEXTURES_COMPARE=<dir>: the first 20 that differ written there as <n>_ours.rgba / <n>_uploads.rgba
                // (u32 width, height, then RGBA8).
                static int s_written = 0;
                const char *dir = std::getenv("VP2S_GAME_TEXTURES_COMPARE");
                if (wrong && dir[0] != '1' && s_written < 20)
                {
                    const int n = s_written++;
                    for (int which = 0; which < 2; ++which)
                    {
                        char path[600];
                        std::snprintf(path, sizeof(path), "%s/%02d_%08x_%x_%s.rgba", dir, n, textureKey.tag, textureKey.index,
                                      which ? "uploads" : "ours");
                        if (FILE *f = std::fopen(path, "wb"))
                        {
                            const auto &t = which ? *reference : *shared;
                            std::fwrite(&t.width, 4, 1, f);
                            std::fwrite(&t.height, 4, 1, f);
                            std::fwrite(t.rgba.data(), 4, t.rgba.size(), f);
                            std::fclose(f);
                        }
                    }
                }
            }
        return shared;
    }
}

void vp2sInstallGameTextures(PS2Runtime &runtime)
{
    // Always on: the native models take their textures here by the game's keys (vp2sGameTexture); the TEX0 provider
    // only serves the old GS path (VP2S_GS=1, to compare).
    const ps2_overlays::Module *module = ps2_overlays::findModule("mod001");
    if (!module || kTextureCache < module->tableBase || kTextureCache >= module->tableEnd)
        return;
    g_original = module->table[(kTextureCache - module->tableBase) >> 2];
    if (!g_original || !ps2_overlays::replaceModuleFunction("mod001", kTextureCache, &textureCacheHook))
        return;
    g_allocate = module->table[(kTextureAllocate - module->tableBase) >> 2];
    if (g_allocate)
        ps2_overlays::replaceModuleFunction("mod001", kTextureAllocate, &textureAllocateHook);
    g_clutAllocate = module->table[(kClutAllocate - module->tableBase) >> 2];
    if (g_clutAllocate)
        ps2_overlays::replaceModuleFunction("mod001", kClutAllocate, &clutAllocateHook);
    if (FILE *f = std::fopen((directory() + "/variants.txt").c_str(), "r"))
    {
        char part[64], file[64];
        unsigned offset = 0;
        while (std::fscanf(f, "%63s %x %63s", part, &offset, file) == 3)
        {
            char name[96];
            std::snprintf(name, sizeof(name), "%s %x", part, offset);
            g_variants[name] = file;
        }
        std::fclose(f);
    }
    vp2sInstallLoads(runtime);
    ps2x::native::textures().setProvider(&provide, false);
    std::printf("[vp2s] native textures from the game's texture cache and the offline conversion (%s, %zu copies by place)%s%s\n",
                directory().c_str(), g_variants.size(), g_allocate ? "" : " (no VRAM allocator hook)",
                g_clutAllocate ? "" : " (no CLUT allocator hook)");
}

std::shared_ptr<const ps2x::native::TextureCache::Texture> vp2sGameTexture(uint32_t tag, uint32_t index, uint32_t csa,
                                                                           uint32_t paletteTag, uint32_t paletteIndex)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const Key key{tag, index};
    const std::shared_ptr<Converted> converted = load(key);
    if (!converted || converted->width == 0u || converted->height == 0u)
        return nullptr;
    // Another conversion's palette (a sheet whose palette the game sent from another file, as the battle portraits).
    std::shared_ptr<Converted> paletteFrom = converted;
    uint64_t id = std::hash<std::string>()(fileOf(key)) * 0x9E3779B97F4A7C15ull ^ (uint64_t{csa & 0x1Fu} << 8) ^ 0x5A5Au;
    if (paletteTag != 0u && (paletteTag != tag || paletteIndex != index))
    {
        const Key paletteKey{paletteTag, paletteIndex};
        paletteFrom = load(paletteKey);
        if (!paletteFrom)
            return nullptr;
        id ^= std::hash<std::string>()(fileOf(paletteKey)) * 0xC2B2AE3D27D4EB4Full;
    }
    if (const auto it = g_textures.find(id); it != g_textures.end())
        return it->second;
    // The texture is the conversion's size rounded up to powers of two (TW / TH as the game sets them).
    ps2x::native::TextureCache::Texture texture;
    texture.width = 1u;
    while (texture.width < converted->width && texture.width < 1024u)
        texture.width <<= 1;
    texture.height = 1u;
    while (texture.height < converted->height && texture.height < 1024u)
        texture.height <<= 1;
    texture.key = id;
    texture.version = ++g_version;
    texture.rgba.assign(static_cast<size_t>(texture.width) * texture.height, 0u);
    texture.rawAlpha = true; // the conversion's alpha is the GS's own (tools/vp2_textures.py)
    for (uint32_t y = 0; y < converted->height && y < texture.height; ++y)
        for (uint32_t x = 0; x < converted->width && x < texture.width; ++x)
        {
            const size_t k = static_cast<size_t>(y) * converted->width + x;
            uint32_t rgba;
            if (converted->psm == 0u)
                std::memcpy(&rgba, converted->texels.data() + k * 4u, 4u);
            else
                rgba = paletteFrom->palette[(converted->psm == 0x14u ? (csa & 0x1Fu) * 16u + converted->texels[k] : converted->texels[k]) & 0xFFu];
            texture.rgba[static_cast<size_t>(y) * texture.width + x] = rgba;
        }
    // VP2S_GAME_TEXTURES_LOG: each texture made by key once, with its file and mean colour.
    static const bool s_log = std::getenv("VP2S_GAME_TEXTURES_LOG") != nullptr;
    if (s_log)
    {
        uint64_t sum[4] = {};
        for (const uint32_t c : texture.rgba)
            for (int k = 0; k < 4; ++k)
                sum[k] += (c >> (8 * k)) & 0xFFu;
        const size_t n = std::max<size_t>(texture.rgba.size(), 1u);
        std::printf("[vp2s-textures] key %08x_%x csa %u -> %s %ux%u psm %x, mean rgba %llu %llu %llu %llu\n", tag, index, csa & 0x1Fu,
                    fileOf(key).c_str(), texture.width, texture.height, converted->psm, static_cast<unsigned long long>(sum[0] / n),
                    static_cast<unsigned long long>(sum[1] / n), static_cast<unsigned long long>(sum[2] / n),
                    static_cast<unsigned long long>(sum[3] / n));
    }
    auto shared = std::make_shared<const ps2x::native::TextureCache::Texture>(std::move(texture));
    g_textures[id] = shared;
    return shared;
}

bool vp2sGamePalette(uint32_t tag, uint32_t index, uint32_t palette[256])
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const std::shared_ptr<Converted> converted = load(Key{tag, index});
    if (!converted)
        return false;
    std::memcpy(palette, converted->palette.data(), 256u * 4u);
    return true;
}

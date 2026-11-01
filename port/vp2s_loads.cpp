// Where the game loaded each piece of its data from, by its own requests: every disc read (sceCdRead, told by the
// runtime: sector, sectors, buffer; the game's disc thread FUN_0011C4D0 reads its archive's files there, the sector
// from its TOC) and every decompression (main FUN_001025D0: a1 the chain as read, a2 the destination, a3 the part;
// a chunk's header +8 its decompressed size, +0xC the offset of the next). A decompressed region is then the asset
// part tools/extract.py writes as <chain disc offset:010x>_<part:03d>.

#include "vp2s_loads.h"

#include "ps2_runtime.h"
#include "runtime/cd_read_observer.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <vector>

namespace
{
    struct Read
    {
        uint32_t buffer = 0, bytes = 0;
        uint64_t disc = 0;
    };
    struct Region
    {
        uint32_t begin = 0, end = 0, part = 0;
        uint64_t chain = 0;
        uint32_t base = 0; // the part's offset at begin (a copy of a piece of the part starts inside it)
    };

    std::mutex g_mutex;
    std::vector<Read> g_reads;
    std::vector<Region> g_regions;
    PS2Runtime::RecompiledFunction g_decompress = nullptr;
    constexpr uint32_t kDecompress = 0x001025D0u;

    void cdRead(uint32_t lbn, uint32_t sectors, uint32_t buffer)
    {
        const Read read{buffer & 0x1FFFFFFu, sectors * 2048u, uint64_t{lbn} * 2048u};
        static const bool s_log = std::getenv("VP2S_LOADS_LOG") != nullptr;
        if (s_log)
            std::printf("[vp2s-loads] read sector %x x%x into %x\n", lbn, sectors, read.buffer);
        std::lock_guard<std::mutex> lock(g_mutex);
        std::erase_if(g_reads, [&](const Read &r) { return r.buffer < read.buffer + read.bytes && read.buffer < r.buffer + r.bytes; });
        g_reads.push_back(read);
    }

    uint32_t word(const uint8_t *rdram, uint32_t address)
    {
        uint32_t value;
        std::memcpy(&value, rdram + (address & 0x1FFFFFCu), 4);
        return value;
    }

    // VP2S_LOADS_DUMP=<chain disc offset, hex>: that chain as the game is about to decompress it (after the decryptions
    // done before), 16 MB at most from its start written to vp2s_chain_<offset>.bin, to compare with the disc's bytes.
    void dumpChain(const uint8_t *rdram, uint32_t source, const Read &read)
    {
        static const unsigned long long s_dump = [] { const char *v = std::getenv("VP2S_LOADS_DUMP"); return v ? std::strtoull(v, nullptr, 16) : 0ull; }();
        if (s_dump == 0ull || read.disc + (source - read.buffer) != s_dump)
            return;
        const uint32_t bytes = std::min<uint32_t>(read.buffer + read.bytes - source, 16u << 20);
        char name[64];
        std::snprintf(name, sizeof(name), "vp2s_chain_%010llx.bin", s_dump);
        if (FILE *f = std::fopen(name, "wb"))
        {
            std::fwrite(rdram + source, 1, std::min<uint32_t>(bytes, 0x2000000u - source), f);
            std::fclose(f);
        }
        std::printf("[vp2s-loads] dumped chain %010llx (%u bytes; read of %x bytes at disc %010llx into %x)\n", s_dump, bytes, read.bytes,
                    static_cast<unsigned long long>(read.disc), read.buffer);
        std::fflush(stdout);
    }

    void decompressHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t source = getRegU32(ctx, 5) & 0x1FFFFFFu, destination = getRegU32(ctx, 6) & 0x1FFFFFFu;
        const uint32_t part = getRegU32(ctx, 7);
        // The part's chunk: its decompressed size (read before the call, which may decrypt the chunk in place).
        uint32_t chunk = source;
        for (uint32_t i = 0; i < part && i < 4096u; ++i)
            chunk += word(rdram, chunk + 0xCu);
        const uint32_t size = word(rdram, chunk + 8u);
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            const auto read = std::find_if(g_reads.begin(), g_reads.end(),
                                           [&](const Read &r) { return source >= r.buffer && source < r.buffer + r.bytes; });
            std::erase_if(g_regions, [&](const Region &r) { return r.begin < destination + size && destination < r.end; });
            if (read != g_reads.end() && size != 0u && size < (64u << 20))
                g_regions.push_back({destination, destination + size, part, read->disc + (source - read->buffer)});
            static const bool s_log = std::getenv("VP2S_LOADS_LOG") != nullptr;
            if (s_log)
                std::printf("[vp2s-loads] decompress %x part %u -> %x..%x %s %010llx\n", source, part, destination, destination + size,
                            read != g_reads.end() ? "chain" : "from no read",
                            read != g_reads.end() ? static_cast<unsigned long long>(read->disc + (source - read->buffer)) : 0ull);
            if (read != g_reads.end())
                dumpChain(rdram, source, *read);
        }
        g_decompress(rdram, ctx, runtime);
    }
}

namespace
{
    // The game's memcpy (main FUN_00132C08: a0 destination, a1 source, a2 bytes): a copy of loaded data keeps where
    // it came from (VP2 copies its models out of the buffers it decompresses them into).
    constexpr uint32_t kMemcpy = 0x00132C08u;
    PS2Runtime::RecompiledFunction g_memcpy = nullptr;

    void memcpyHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t destination = getRegU32(ctx, 4) & 0x1FFFFFFu, source = getRegU32(ctx, 5) & 0x1FFFFFFu;
        const uint32_t bytes = getRegU32(ctx, 6);
        if (bytes >= 64u && bytes < (64u << 20))
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            const auto from = std::find_if(g_regions.begin(), g_regions.end(),
                                           [&](const Region &r) { return source >= r.begin && source < r.end; });
            if (from != g_regions.end())
            {
                const Region copy{destination, destination + std::min(bytes, from->end - source), from->part, from->chain,
                                  from->base + (source - from->begin)};
                std::erase_if(g_regions, [&](const Region &r) { return r.begin < copy.end && copy.begin < r.end; });
                g_regions.push_back(copy);
            }
            // A copy of what a disc read brought (a member kept compressed, decompressed later from the copy): the copy
            // is that part of the disc too.
            const auto read = std::find_if(g_reads.begin(), g_reads.end(),
                                           [&](const Read &r) { return source >= r.buffer && source < r.buffer + r.bytes; });
            if (read != g_reads.end())
            {
                const Read copy{destination, std::min(bytes, read->buffer + read->bytes - source), read->disc + (source - read->buffer)};
                std::erase_if(g_reads, [&](const Read &r) { return r.buffer < copy.buffer + copy.bytes && copy.buffer < r.buffer + r.bytes; });
                g_reads.push_back(copy);
            }
        }
        g_memcpy(rdram, ctx, runtime);
    }

    // The game's asynchronous decompression (main FUN_001059B0: a0 the context, a1 the job, a2 the chain as read, a3 the
    // destination, t0 the part), worked a slice a frame by FUN_001058C0: the field's characters and scenes. After the
    // call (which may decrypt the chunk's header in place), the job holds the size (+8) and the destination (+0xC).
    constexpr uint32_t kDecompressAsync = 0x001059B0u;
    PS2Runtime::RecompiledFunction g_decompressAsync = nullptr;

    void decompressAsyncHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        // Recorded before the call: the work may stop at an EE checkpoint and resume inside the function, past this
        // hook. a2 the chain, a3 the destination (| 0x30000000), t0 the part; the part's chunk +8 its size.
        const uint32_t source = getRegU32(ctx, 6) & 0x1FFFFFFu, destination = getRegU32(ctx, 7) & 0x1FFFFFFu;
        const uint32_t part = getRegU32(ctx, 8);
        uint32_t chunk = source;
        for (uint32_t i = 0; i < part && i < 4096u; ++i)
            chunk += word(rdram, chunk + 0xCu);
        const uint32_t size = word(rdram, chunk + 8u);
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            const auto read = std::find_if(g_reads.begin(), g_reads.end(),
                                           [&](const Read &r) { return source >= r.buffer && source < r.buffer + r.bytes; });
            static const bool s_log = std::getenv("VP2S_LOADS_LOG") != nullptr;
            if (s_log)
                std::printf("[vp2s-loads] async decompress %x part %u -> %x..%x %s\n", source, part, destination, destination + size,
                            read != g_reads.end() ? "chain" : "from no read");
            if (read != g_reads.end())
                dumpChain(rdram, source, *read);
            if (read != g_reads.end() && size != 0u && size < (64u << 20))
            {
                std::erase_if(g_regions, [&](const Region &r) { return r.begin < destination + size && destination < r.end; });
                g_regions.push_back({destination, destination + size, part, read->disc + (source - read->buffer)});
            }
        }
        g_decompressAsync(rdram, ctx, runtime);
    }
}

bool vp2sLoadedFrom(uint32_t address, std::string &part, uint32_t &offset)
{
    address &= 0x1FFFFFFu;
    std::lock_guard<std::mutex> lock(g_mutex);
    for (const Region &r : g_regions)
        if (address >= r.begin && address < r.end)
        {
            char name[32];
            std::snprintf(name, sizeof(name), "%010llx_%03u", static_cast<unsigned long long>(r.chain), r.part);
            part = name;
            offset = r.base + (address - r.begin);
            return true;
        }
    // VP2S_LOADS_LOG: the first misses, with the regions around them.
    static const bool s_log = std::getenv("VP2S_LOADS_LOG") != nullptr;
    static int s_logged = 0;
    if (s_log && s_logged < 5)
    {
        ++s_logged;
        uint32_t below = 0, above = 0xFFFFFFFFu;
        for (const Region &r : g_regions)
        {
            if (r.end <= address && r.end > below)
                below = r.end;
            if (r.begin > address && r.begin < above)
                above = r.begin;
        }
        std::printf("[vp2s-loads] %x in no region (%zu regions, %zu reads; nearest ends below %x, starts above %x)\n", address,
                    g_regions.size(), g_reads.size(), below, above);
    }
    return false;
}

void vp2sInstallLoads(PS2Runtime &runtime)
{
    static bool s_installed = false;
    if (s_installed)
        return;
    s_installed = true;
    ps2x::setCdReadObserver(&cdRead);
    g_decompress = runtime.lookupFunction(kDecompress);
    if (g_decompress)
        runtime.replaceFunction(kDecompress, &decompressHook);
    g_memcpy = runtime.lookupFunction(kMemcpy);
    if (g_memcpy)
        runtime.replaceFunction(kMemcpy, &memcpyHook);
    g_decompressAsync = runtime.lookupFunction(kDecompressAsync);
    if (g_decompressAsync)
        runtime.replaceFunction(kDecompressAsync, &decompressAsyncHook);
}

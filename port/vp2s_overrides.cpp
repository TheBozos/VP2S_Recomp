// Game-specific runtime setup for Valkyrie Profile 2: Silmeria (SLUS-21452, the American disc).

#include "game_overrides.h"
#include "ps2_runtime.h"
#include "runtime/ee_scheduler.h"
#include "vp2s_vu1_hle.h"
#include "vp2s_textures.h"
#include "vp2s_loads.h"
#include "vp2s_models.h"
#include "vp2s_ui.h"
#include "vp2s_minimap.h"
#include "vp2s_battle_hud.h"
#include "ps2_overlay_modules.h"
#include "ps2x/iop/native_mixer.h"

namespace ps2_stubs // the runtime's libmpeg HLE (Kernel/Stubs/MPEG.h)
{
    using MpegAudioSink = void (*)(uint32_t mpegAddr, uint32_t channel, const uint8_t *data, size_t bytes, uint32_t picturesServed);
    using MpegPictureHook = void (*)(uint32_t mpegAddr, uint32_t picture);
    void setMpegAudioSink(MpegAudioSink sink);
    void setMpegPictureHook(MpegPictureHook hook);
    bool mpegStreamEnded(uint32_t mpegAddr);
}

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <filesystem>
#include <iostream>
#include <unordered_map>

namespace
{
    // The game reads its hidden archive by raw sector (sceCdRead), so the CD device
    // needs the disc image itself: VP2S_ISO, else an .iso found walking up from the ELF
    // directory (build_usa/extract/iso -> project root), one named "(USA)" first: the port
    // follows the American disc.
    std::filesystem::path findDiscImage(const std::filesystem::path &elfDirectory)
    {
        if (const char *configured = std::getenv("VP2S_ISO"); configured && configured[0] != '\0')
        {
            return configured;
        }
        std::filesystem::path dir = elfDirectory;
        for (int depth = 0; depth < 5 && !dir.empty(); ++depth, dir = dir.parent_path())
        {
            std::error_code ec;
            std::filesystem::path found;
            for (const auto &entry : std::filesystem::directory_iterator(dir, ec))
            {
                if (entry.is_regular_file() && entry.path().extension() == ".iso" &&
                    (found.empty() || entry.path().filename().string().find("(USA)") != std::string::npos))
                {
                    found = entry.path();
                }
            }
            if (!found.empty())
            {
                return found;
            }
            if (dir == dir.root_path())
            {
                break;
            }
        }
        return {};
    }

    // The game uses nearly all user RAM: its heap runs from the end of the overlay area
    // (0x549080) up to the main thread stack (0x1FF0000), where the runtime keeps its HLE
    // pools and invocation stacks by default. Move those into the HLE-unused kernel memory
    // below the ELF and let a rest-of-RAM heap reach the stack, as on the console. The range starts at 64 KiB:
    // past the 256 KiB of HLE pools it holds the 16 KiB invocation stacks the scheduler keeps per thread and nesting
    // depth, and from 0x80000 it fitted only 16 of them (the scene after the first field skip ran out).
    constexpr uint32_t kReservedBase = 0x00010000u;
    constexpr uint32_t kReservedTop = 0x00100000u;
    constexpr uint32_t kHeapLimit = 0x01FF0000u;

    // Guest register and memory helpers for the native replacements below.
    uint32_t reg(const R5900Context *ctx, int index)
    {
        return getRegU32(ctx, index);
    }

    void setReg(R5900Context *ctx, int index, uint32_t value)
    {
        ctx->r[index] = _mm_insert_epi64(ctx->r[index], static_cast<int64_t>(static_cast<int32_t>(value)), 0);
    }

    uint8_t *guest(uint8_t *rdram, uint32_t address)
    {
        return rdram + (address & PS2_RAM_MASK);
    }

    // FUN_0011f520(object, count): zero the vblank flag at +0xC, spin until the vblank handler sets it (count 0) or
    // until the counter at +0x10 reaches count, return the field byte at +0x2FA; count -1 only returns the byte.
    constexpr uint32_t kFrameWait = 0x0011F5A0u;
    constexpr uint32_t kFrameWaitCount = 0x0011F5B8u; // the count > 0 path, left to the recompiled code

    // VP2's field loop (mod003 FUN_003d72e0, its object at *0x48E048) draws one frame per 0x98 vblanks (2: 25 fps in
    // PAL) and scales its logic by the global step it derives each frame from 0xDC (also 2): dt = 0xDC * 1.2 *
    // 0xA0 * 0xA8 frames of 1/60 s, at 0x35DED8 (and /60 in seconds at 0x35DEE8). FUN_0011f510 returns the vblanks
    // elapsed since the last frame (read and cleared once per frame, called only by that loop).
    constexpr uint32_t kElapsedVBlanks = 0x0011F590u;
    constexpr uint32_t kFieldLoopObject = 0x0048E048u;

    uint32_t load32(uint8_t *rdram, uint32_t address)
    {
        uint32_t value = 0;
        std::memcpy(&value, guest(rdram, address), sizeof(value));
        return value;
    }

    void store32(uint8_t *rdram, uint32_t address, uint32_t value)
    {
        std::memcpy(guest(rdram, address), &value, sizeof(value));
    }

    // Native loop (VP2S_NATIVE=1): no vblank in VP2's frame pacing. The flip's frame wait (FUN_0011f520 with count
    // 0) becomes the frame end: the runtime's FramePacer holds it to the frame cap (PS2X_FRAME_CAP, 0 = unlimited)
    // and the frame is shown at once. The field loop then counts one vblank per frame, and its step is the real
    // time the frame took: 0x35DED8 = seconds * 60 * 0xA0 * 0xA8 (1/60 s units, as the game's own 0xDC * 1.2 *
    // 0xA0 * 0xA8 with 0xDC PAL vblanks), 0x35DEE8 the same in seconds.
    constexpr uint32_t kStepFrames = 0x0035DED8u;
    constexpr uint32_t kStepSeconds = 0x0035DEE8u;
    constexpr double kEeClockHz = 294912000.0;
    double g_frameSeconds = 1.0 / 60.0;

    float loadFloat(uint8_t *rdram, uint32_t address)
    {
        float value = 0.0f;
        std::memcpy(&value, guest(rdram, address), sizeof(value));
        return value;
    }

    void storeFloat(uint8_t *rdram, uint32_t address, float value)
    {
        std::memcpy(guest(rdram, address), &value, sizeof(value));
    }

    void endFrame(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime, uint32_t object)
    {
        // (the native frame ends with the game's scene pass: vp2s_models.cpp)
        g_frameSeconds = runtime->framePacer().frameEnd();
        // The game no longer spins its wait, so EE time would fall behind real time and the events it waits on
        // (vblank handlers, timers, the IOP's loads) would stall: charge what the frame did not account.
        EeScheduler &scheduler = runtime->eeScheduler();
        static uint64_t s_lastFrameCycle = scheduler.currentCycle();
        const uint64_t accounted = scheduler.currentCycle() - s_lastFrameCycle;
        const uint64_t realCycles = static_cast<uint64_t>(g_frameSeconds * kEeClockHz);
        if (realCycles > accounted)
            scheduler.accountCycles(static_cast<uint32_t>(std::min<uint64_t>(realCycles - accounted, 0xFFFFFFFFu)));
        s_lastFrameCycle = scheduler.currentCycle();
        scheduler.presentGuestFrame();
        setReg(ctx, 2, *guest(rdram, object + 0x2FAu));
        ctx->pc = reg(ctx, 31);
    }

    void nativeFrameEnd(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t object = reg(ctx, 4);
        const int32_t count = static_cast<int32_t>(reg(ctx, 5));
        if (count > 0)
        {
            ctx->pc = kFrameWaitCount;
            return;
        }
        std::memset(guest(rdram, 0x001D8F00u), 0, 4);
        if (count == 0)
        {
            // Until the frame is due the thread waits, as on the PS2 for its vblank: the game's other threads (the
            // disc loader, the decompression worker) run meanwhile. Sleeping in the pacer held the EE instead, and
            // each hand-over between them waited for a frame end (the black screen before the Chapter 1 title: 5.5 s,
            // 4.2 s now, 4.5 s in PCSX2). The pacer spins the last 2 ms.
            const auto due = runtime->framePacer().due();
            if (due && *due - std::chrono::steady_clock::now() > std::chrono::milliseconds(3))
                runtime->eeScheduler().waitUntil(*due - std::chrono::milliseconds(2), [rdram, runtime, object](R5900Context &resumed) {
                    endFrame(rdram, &resumed, runtime, object);
                });
            endFrame(rdram, ctx, runtime, object);
            return;
        }
        setReg(ctx, 2, *guest(rdram, object + 0x2FAu));
        ctx->pc = reg(ctx, 31);
    }

    void nativeElapsed(uint8_t *rdram, R5900Context *ctx, PS2Runtime *)
    {
        const uint32_t counter = reg(ctx, 4);
        store32(rdram, counter + 0x10u, 0u);
        setReg(ctx, 2, 1u);
        ctx->pc = reg(ctx, 31);

        const uint32_t loop = load32(rdram, kFieldLoopObject);
        if (loop < 0x00100000u || loop >= 0x02000000u)
            return;
        const uint32_t interval = load32(rdram, loop + 0x98u);
        if (interval < 1u || interval > 4u)
            return;
        store32(rdram, loop + 0x98u, 1u);
        *guest(rdram, loop + 0xDCu) = 1u;
        const float scale = loadFloat(rdram, loop + 0xA0u) * loadFloat(rdram, loop + 0xA8u);
        const float seconds = static_cast<float>(g_frameSeconds) * scale;
        storeFloat(rdram, kStepFrames, seconds * 60.0f);
        storeFloat(rdram, kStepSeconds, seconds);
    }

    // FUN_001025d0(loader, file, slot, index) decompresses the index-th SLZ/SLE chunk of a file already in memory
    // into a module slot. Many modules are not found on the disc by the extractor (docs/AUDIT.md: declared slots with
    // no module; their files carry another layer of encryption): every module it loads that the port does not know
    // is saved once to build/captured_modules/<load>_<id>.bin, where tools/pipeline.py picks it up. VP2S_MODULE_TRACE=1
    // also prints each chunk's header and first bytes.
    constexpr uint32_t kModuleLoad = 0x00102650u;
    PS2Runtime::RecompiledFunction g_moduleLoad = nullptr;

    void captureUnknownModule(uint8_t *rdram, uint32_t slot)
    {
        // A module image: header +8 load address (the slot), +4 id, +12 text end, +16 data size.
        if (slot < 0x00100000u || slot >= 0x01000000u || load32(rdram, slot + 8u) != slot)
            return;
        // Known when a registered module loaded at this very address is resident (a module of another slot may
        // cover the address too: the camp modules at 0x495500 reach past slot 5 at 0x4aeb80).
        if (const ps2_overlays::Module *known = ps2_overlays::residentModuleFor(rdram, slot + 0x80u);
            known && known->loadAddress == slot)
            return;
        const uint32_t id = load32(rdram, slot + 4u), size = load32(rdram, slot + 12u) + load32(rdram, slot + 16u);
        if (id == 0u || id > 0xFFFFu || size < 0x100u || size > 0x200000u)
            return;
        char name[96];
        std::snprintf(name, sizeof(name), "../captured_modules/%08x_%03u.bin", slot, id);
        if (std::filesystem::exists(name))
            return;
        std::error_code ec;
        std::filesystem::create_directories("../captured_modules", ec);
        if (std::FILE *out = std::fopen(name, "wb"))
        {
            std::fwrite(guest(rdram, slot), 1, size, out);
            std::fclose(out);
            std::cout << "[vp2s] captured unknown module " << id << " at 0x" << std::hex << slot << std::dec << " -> "
                      << name << std::endl;
        }
    }

    void traceModuleLoad(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        uint32_t file = reg(ctx, 5);
        const uint32_t slot = reg(ctx, 6), index = reg(ctx, 7);
        static const char traceLevel = [] { const char *v = std::getenv("VP2S_MODULE_TRACE"); return v ? v[0] : '0'; }();
        const bool trace = traceLevel == '1' || traceLevel == '2';
        if (trace)
        {
            for (uint32_t k = 0; k < index && k < 64u; ++k)
                file += load32(rdram, file + 0xCu);
            std::cout << std::hex << "[vp2s-load] slot=0x" << slot << " file=0x" << reg(ctx, 5) << " index=" << index
                      << " chunk=0x" << file << " head=";
            for (uint32_t k = 0; k < 48u; ++k)
            {
                const uint8_t b = *guest(rdram, file + k);
                std::cout << "0123456789abcdef"[b >> 4] << "0123456789abcdef"[b & 15];
            }
            std::cout << std::dec << std::endl;
            // VP2S_MODULE_TRACE=2: the whole chunk as the loader gets it, to compare with the disc.
            const bool save = traceLevel == '2';
            const uint32_t comp = load32(rdram, file + 4u);
            if (save && comp > 0u && comp < 0x400000u)
            {
                char name[96];
                std::snprintf(name, sizeof(name), "vp2s_chunk_%08x_%08x.bin", slot, file);
                if (std::FILE *out = std::fopen(name, "wb"))
                {
                    std::fwrite(guest(rdram, file), 1, comp + 16u, out);
                    std::fclose(out);
                }
            }
        }
        g_moduleLoad(rdram, ctx, runtime);
        captureUnknownModule(rdram, slot);
    }

    // FUN_001210a0(module header, ...) clears a module's bss and runs its constructors: every module goes through it
    // whatever loaded it, so an unknown one is captured there too (before its constructors call into it).
    constexpr uint32_t kModuleInit = 0x00121120u;
    PS2Runtime::RecompiledFunction g_moduleInit = nullptr;

    void captureAtModuleInit(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        captureUnknownModule(rdram, reg(ctx, 4));
        g_moduleInit(rdram, ctx, runtime);
    }

    // VP2S_NATIVE_MESH=1: the native renderer's frame ends where the game waits for its frame (FUN_0011f520 count 0).
    PS2Runtime::RecompiledFunction g_frameWait = nullptr;

    void nativeMeshFrameWait(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        if (static_cast<int32_t>(reg(ctx, 5)) == 0)
            vp2sNativeMeshFrameEnd();
        g_frameWait(rdram, ctx, runtime);
    }

    // VP2's streamed music (town and event themes) is decoded by a VU1 program at 0x06E0: FUN_001060a0 sends 2 KiB of
    // the stream (UNPACK into VU1 memory), the decoder's tables and its MSCALs through one of two VIF1 chains, and the
    // EE copies the PCM out of VU1 memory into the buffers it sends to the IOP. The first decode goes through a chain
    // that uploads the microcode (MPG) and then runs on into the first of the two. These chains still run with VIF1
    // dropped (USA addresses).
    bool keptVif1Chain(uint32_t tadr)
    {
        return tadr == 0x0014A300u || tadr == 0x0014C730u || tadr == 0x0014C8E0u;
    }

    // The music stream's PCM goes to the IOP 4 KiB at a time, every packet into the same IOP buffer. The EE sends the
    // next one when the IOP acknowledges a packet (SIF command 4: FUN_001076F0 kind 4 -> FUN_001077E0(player, 1)) but
    // also when an IOP status message arrives (FUN_00106D00 -> FUN_001077E0(player, 0)), acknowledged or not. With
    // the PS2's timings the IOP's sound driver has copied the packet by then; here it may be waiting for its SPU2 ring
    // to have room, and the next packet overwrote the one it held: the first packet of a ring half lost and the next
    // one played twice, a click every quarter second. One packet is in flight at a time: the status path does not
    // send while a packet waits for its acknowledgement.
    constexpr uint32_t kMusicSends[] = {0x001080A0u, 0x001081C0u}; // SIF command send (interrupt / thread)
    constexpr uint32_t kMusicCommand = 0x001076F0u;                  // FUN_001076F0(player, kind of IOP command)
    constexpr uint32_t kMusicNext = 0x001077E0u;                     // FUN_001077E0(player, from the acknowledgement)
    std::unordered_map<uint32_t, PS2Runtime::RecompiledFunction> g_musicOriginals;
    std::atomic<bool> g_musicPacketInFlight{false};

    void musicSend(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        if (reg(ctx, 5) == 0u && static_cast<uint32_t>(reg(ctx, 8)) == 0x1040u) // command 0, header + 4 KiB of PCM
            g_musicPacketInFlight.store(true);
        g_musicOriginals[ctx->pc](rdram, ctx, runtime);
    }

    void musicCommand(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        if (reg(ctx, 5) == 4u)
            g_musicPacketInFlight.store(false);
        g_musicOriginals[kMusicCommand](rdram, ctx, runtime);
    }

    void musicNext(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        if (reg(ctx, 5) == 0u && g_musicPacketInFlight.load())
        {
            ctx->pc = reg(ctx, 31);
            return;
        }
        g_musicOriginals[kMusicNext](rdram, ctx, runtime);
    }

    // A movie played to its end: VP2's movie module (mod001) closes it once its sceMpegCbNodata callback has been
    // called 11 times with no video left in the ring it feeds the IPU from (FUN_002b9330 counts at object -0x19B4).
    // The runtime decodes the movie from its demux and never drains that ring, so the callback found 0x18E sectors
    // there, handed them to an IPU nobody reads and reset its count: the screen stayed black. Once the runtime has
    // shown the last picture, a call counts as one with nothing left.
    constexpr uint32_t kMovieNodata = 0x002B9330u; // mod001
    PS2Runtime::RecompiledFunction g_movieNodata = nullptr;

    void movieNodata(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        if (ps2_stubs::mpegStreamEnded(static_cast<uint32_t>(reg(ctx, 4))))
        {
            uint8_t *count = rdram + ((static_cast<uint32_t>(reg(ctx, 6)) + 0x160000u - 0x19B4u) & 0x1FFFFFFu);
            int32_t calls;
            std::memcpy(&calls, count, 4);
            ++calls;
            std::memcpy(count, &calls, 4);
            setReg(ctx, 2, 1u);
            ctx->pc = reg(ctx, 31);
            return;
        }
        g_movieNodata(rdram, ctx, runtime);
    }

    // VP2S_HOOK_TRACE=<address>[,<address>...]: prints a0..a3 and ra of the first 40 calls of each function.
    std::unordered_map<uint32_t, std::pair<PS2Runtime::RecompiledFunction, uint32_t>> g_traced;

    void traceCall(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const auto it = g_traced.find(ctx->pc);
        if (it == g_traced.end())
            return;
        const uint32_t pc = ctx->pc;
        const uint64_t a0 = reg(ctx, 4), a1 = reg(ctx, 5), a2 = reg(ctx, 6), a3 = reg(ctx, 7), t0 = reg(ctx, 8), t1 = reg(ctx, 9),
                       ra = reg(ctx, 31);
        // Each distinct (function, a0..a3, ra) once, up to 300 a function.
        static std::unordered_map<uint64_t, bool> s_seen;
        const uint64_t key = ((uint64_t{pc} * 31u + a0) * 31u + a1) * 31u + a2 * 7u + a3 * 3u + ra;
        const bool show = s_seen.emplace(key, true).second && it->second.second++ < 300u;
        it->second.first(rdram, ctx, runtime);
        // VP2S_HOOK_SAVE=<dir>: with each line shown, 64 KiB of guest memory from a1 written to <dir>/<pc>_<a2>_<a3>.bin
        // (the game's data a hooked routine works on, studied offline; game data: build tree only, never committed).
        static const char *s_save = std::getenv("VP2S_HOOK_SAVE");
        if (show && s_save && (a1 & 0x1FFFFFFu) + 0x10000u <= PS2_RAM_SIZE)
        {
            char path[600];
            std::snprintf(path, sizeof(path), "%s/%x_%llx_%llx.bin", s_save, pc, static_cast<unsigned long long>(a2),
                          static_cast<unsigned long long>(a3));
            if (FILE *f = std::fopen(path, "wb"))
            {
                std::fwrite(rdram + (a1 & 0x1FFFFFFu), 1, 0x10000u, f);
                std::fclose(f);
            }
        }
        if (show)
            std::cout << std::hex << "[vp2s-hook] 0x" << pc << " a0=0x" << a0 << " a1=0x" << a1 << " a2=0x" << a2 << " a3=0x"
                      << a3 << " t0=0x" << t0 << " t1=0x" << t1 << " ra=0x" << ra << " -> v0=0x" << reg(ctx, 2) << std::dec
                      << std::endl;
    }

    // VP2's movie audio, native: its PSS carries 48 kHz 16-bit stereo PCM on private stream 1 channel 0, in blocks of
    // 4096 bytes (1024 left samples, then 1024 right). The game copies it to an EE ring, sends it to its IOP module by
    // RPC and has it written into the sound data input in 8 KiB pieces; on this runtime the pieces came out of order
    // (repeated / skipped: the crackle PCSX2 does not have). Here the runtime's demuxer hands the PCM to the native
    // mixer directly, which plays it from the movie's first picture in place of the game's input (the game's path keeps
    // running, unheard). VP2S_MOVIE_AUDIO=0: the game's path.
    struct MovieAudio
    {
        std::vector<uint8_t> block; // a 4096-byte block being assembled
        bool shown = false; // its first picture served (playing)
    };
    MovieAudio g_movieAudio;

    void movieAudioSink(uint32_t, uint32_t channel, const uint8_t *data, size_t bytes, uint32_t picturesServed)
    {
        if (channel != 0u)
            return;
        ps2x::audio::NativeMixer &mixer = ps2x::audio::NativeMixer::instance();
        // A new playback (no picture yet) while the last one's audio is still queued or playing: start over.
        if (picturesServed == 0u && g_movieAudio.shown)
        {
            mixer.movieStop();
            g_movieAudio = MovieAudio{};
        }
        constexpr size_t kBlock = 4096u, kSamples = kBlock / 4u;
        for (size_t at = 0; at < bytes;)
        {
            const size_t take = std::min(bytes - at, kBlock - g_movieAudio.block.size());
            g_movieAudio.block.insert(g_movieAudio.block.end(), data + at, data + at + take);
            at += take;
            if (g_movieAudio.block.size() < kBlock)
                break;
            int16_t stereo[kSamples * 2u];
            for (size_t k = 0; k < kSamples; ++k)
            {
                std::memcpy(&stereo[2u * k], g_movieAudio.block.data() + 2u * k, 2u);
                std::memcpy(&stereo[2u * k + 1u], g_movieAudio.block.data() + kBlock / 2u + 2u * k, 2u);
            }
            mixer.moviePush(stereo, kSamples);
            g_movieAudio.block.clear();
        }
    }

    void moviePictureHook(uint32_t, uint32_t picture)
    {
        if (picture == 0u)
        {
            ps2x::audio::NativeMixer::instance().movieStart(0);
            g_movieAudio.shown = true;
        }
    }

    void applyVp2s(PS2Runtime &runtime)
    {
        if (const char *v = std::getenv("VP2S_MOVIE_AUDIO"); !(v && v[0] == '0'))
        {
            const char *native = std::getenv("PS2X_NATIVE_AUDIO");
            if (!(native && native[0] == '0'))
            {
                ps2_stubs::setMpegAudioSink(&movieAudioSink);
                ps2_stubs::setMpegPictureHook(&moviePictureHook);
            }
        }
        for (uint32_t address : {kMusicSends[0], kMusicSends[1], kMusicCommand, kMusicNext})
            if (PS2Runtime::RecompiledFunction original = runtime.lookupFunction(address))
            {
                g_musicOriginals[address] = original;
                runtime.replaceFunction(address, address == kMusicCommand ? &musicCommand
                                                 : address == kMusicNext  ? &musicNext
                                                                          : &musicSend);
            }
        if (const ps2_overlays::Module *movies = ps2_overlays::findModule("mod001");
            movies && kMovieNodata >= movies->tableBase && kMovieNodata < movies->tableEnd &&
            (g_movieNodata = movies->table[(kMovieNodata - movies->tableBase) >> 2]))
            ps2_overlays::replaceModuleFunction("mod001", kMovieNodata, &movieNodata);
        if (const char *hooks = std::getenv("VP2S_HOOK_TRACE"))
        {
            for (const char *cursor = hooks; *cursor;)
            {
                // <address> in the main program, or <module>:<address> in a code module (mod001:0x29d9b0).
                std::string module;
                if (const char *colon = std::strchr(cursor, ':'); colon && (!std::strchr(cursor, ',') || colon < std::strchr(cursor, ',')))
                {
                    module.assign(cursor, colon);
                    cursor = colon + 1;
                }
                char *end = nullptr;
                const uint32_t address = static_cast<uint32_t>(std::strtoul(cursor, &end, 0));
                if (end == cursor)
                    break;
                if (!module.empty())
                {
                    const ps2_overlays::Module *m = ps2_overlays::findModule(module.c_str());
                    if (m && address >= m->tableBase && address < m->tableEnd && ((address - m->tableBase) >> 2) < m->slotCount &&
                        m->table[(address - m->tableBase) >> 2])
                    {
                        g_traced[address] = {m->table[(address - m->tableBase) >> 2], 0u};
                        ps2_overlays::replaceModuleFunction(module.c_str(), address, &traceCall);
                    }
                }
                else if (PS2Runtime::RecompiledFunction original = runtime.lookupFunction(address))
                {
                    g_traced[address] = {original, 0u};
                    runtime.replaceFunction(address, &traceCall);
                }
                cursor = *end == ',' ? end + 1 : end;
            }
        }
        g_moduleLoad = runtime.lookupFunction(kModuleLoad);
        if (g_moduleLoad)
            runtime.replaceFunction(kModuleLoad, &traceModuleLoad);
        g_moduleInit = runtime.lookupFunction(kModuleInit);
        if (g_moduleInit)
            runtime.replaceFunction(kModuleInit, &captureAtModuleInit);
        runtime.configureReservedMemory(kReservedBase, kReservedTop, kHeapLimit);

        // VP2 waits for its vblank by spinning on a counter in FUN_0011f520 (main thread, 0x11f5a0-0x11f634), never in
        // the kernel: tell the scheduler, or it takes the picture only every third vblank (16 fps shown in towns
        // where the game draws ~45). VP2S_VBLANK_HOLD=1 also holds a vblank while the frame is still being drawn.
        {
            const char *hold = std::getenv("VP2S_VBLANK_HOLD");
            runtime.eeScheduler().setGuestFrameWait(1, 0x0011F5A0u, 0x0011F634u, hold && hold[0] == '1');
            if (const char *limit = std::getenv("VP2S_VBLANK_HOLD_US"))
                runtime.eeScheduler().setVBlankHoldLimit(std::chrono::microseconds(std::strtoul(limit, nullptr, 10)));
        }

        // The mesh microprogram (pc 0x0880) done natively, VU1 only for what it does not do yet (vp2s_vu1_hle.cpp,
        // docs/vp2_vu1.md): the same GS output, field 41 -> 48 distinct frames/s. VP2S_VU1_HLE=0 runs it on VU1.
        if (const char *hle = std::getenv("VP2S_VU1_HLE"); !hle || hle[0] != '0')
        {
            ps2x::native::setVu1Hle(&vp2sVu1Hle);
            vp2sInstallGameTextures(runtime);
            std::cout << "[vp2s] VU1 mesh program done natively" << std::endl;
        }
        // Where the game loads its data from (its disc reads and decompression), and its models drawn natively from
        // that data (vp2s_loads.cpp, vp2s_models.cpp).
        vp2sInstallLoads(runtime);
        vp2sInstallModels(runtime);
        // Nothing on screen comes from VU1 any more (models, particles, 2D from the game's data): VIF1 data is not
        // processed, each transfer ends at once (the game's DMA queue, end interrupts and callbacks keep their order).
        // VP2S_NATIVE_MESH=1 (the GS comparison path, which reads the meshes' packets) or VP2S_VIF1=1 processes it.
        {
            const char *mesh = std::getenv("VP2S_NATIVE_MESH");
            const char *vif1 = std::getenv("VP2S_VIF1");
            if (!(mesh && mesh[0] == '1') && !(vif1 && vif1[0] == '1'))
            {
                ps2x::native::setVif1Dropped(true);
                ps2x::native::setVif1ChainKept(&keptVif1Chain);
                std::cout << "[vp2s] VIF1 data not processed (no VU1 programs run but the music decoder)" << std::endl;
            }
        }
        vp2sInstallUi(runtime);
        vp2sInstallMinimap(runtime);
        vp2sInstallBattleHud(runtime);

        // VP2 draws its picture at 640x448 and copies every other line of it into a 640x256 buffer per interlaced
        // field: the window shows the 640x448 picture, without the half-line bob from one field to the next.
        runtime.gsForPresentation().setPresentProgressiveSource(true);
        // The native loop is the default (frames paced by the host, real-time step, no vblank in VP2's pacing):
        // VP2S_NATIVE=0 goes back to the PAL vblank pacing.
        if (const char *native = std::getenv("VP2S_NATIVE"); !native || native[0] != '0')
        {
            runtime.eeScheduler().setNativeFrames(true);
            // VP2 flips its display per interlaced field in its vblank handler: each frame end shows the buffer it
            // just finished instead.
            runtime.gsForPresentation().setPresentNewestDisplayBuffer(true);
            runtime.replaceFunction(kFrameWait, &nativeFrameEnd);
            runtime.replaceFunction(kElapsedVBlanks, &nativeElapsed);
            std::cout << "[vp2s] native loop: frames paced by the host (cap " << runtime.framePacer().cap()
                      << " fps, 0 = unlimited), real-time step" << std::endl;
        }

        if (vp2sNativeMeshEnabled())
        {
            g_frameWait = runtime.lookupFunction(kFrameWait); // the native loop's replacement when it is on
            if (g_frameWait)
                runtime.replaceFunction(kFrameWait, &nativeMeshFrameWait);
            std::cout << "[vp2s] native meshes: VP2's objects drawn by the native renderer" << std::endl;
            // The objects' textures from the game's own uploads, kept from the first one (store mode: no GS memory).
            if (const char *v = std::getenv("VP2S_NATIVE_MATERIAL"); !(v && v[0] == '0'))
            {
                ps2x::native::textures().setStoreMode(true);
                ps2x::native::setUploadsWanted(true);
            }
        }

        PS2Runtime::IoPaths paths = PS2Runtime::getIoPaths();
        paths.cdImage = findDiscImage(paths.elfDirectory);
        if (paths.cdImage.empty())
        {
            std::cerr << "[vp2s] no disc image found: set VP2S_ISO to the game ISO" << std::endl;
            return;
        }
        std::cout << "[vp2s] disc image: " << paths.cdImage.string() << std::endl;
        PS2Runtime::setIoPaths(paths);
    }
}

PS2_REGISTER_GAME_OVERRIDE("Valkyrie Profile 2: Silmeria (NTSC-U)", "SLUS_214.52", 0x00100008u, 0u, &applyVp2s)

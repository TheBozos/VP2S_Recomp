#pragma once
#include "bozo/runtime.h"
#include "../port/vp2s_music_math.h"
#include <filesystem>
#include <map>
namespace bozo::port { class Registry; }
namespace vp2s {
class NativeMusic {
    struct Decoded {
        music::Block output{},history{};
        uint32_t player=0,source=0,destination=0,historyAddress=0;
        unsigned channel=0;
    };
    music::Tables tables{};
    std::map<uint32_t,Decoded> pending;
    uint64_t frames=0;
public:
    explicit NativeMusic(const std::filesystem::path &);
    void decode(Runtime *,uint32_t player,uint32_t job,unsigned channel);
    // Commit only in the original software-message worker. Returns a next
    // channel/frame to submit, or false after completing an output half.
    bool complete(Runtime *,uint32_t job);
    bool advance(Runtime *,uint32_t player,uint32_t job);
    size_t pendingCount()const{return pending.size();}
    uint64_t completedFrames()const{return frames;}
};
class MusicOwner {
public:
    virtual ~MusicOwner()=default;
    virtual NativeMusic &nativeMusic()=0;
};
void installNativeMusic(Runtime &);
void registerNativeMusic(bozo::port::Registry &);
}

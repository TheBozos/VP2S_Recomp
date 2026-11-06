#pragma once
#include "bozo/libs/pad.h"
#include "bozo/native/compiled_libraries.h"
#include <array>
#include <cstdint>
namespace vp2s {
// DBC's game-owned 128-byte input record. No controller wire commands or device state.
using ControllerRecord=std::array<uint8_t,128>;
void produceControllerRecord(ControllerRecord &record,const bozo::libs::PadInputState &input,
    uint32_t sequence,uint32_t ready,bool pressure,const std::array<uint8_t,4> &capabilities);
class ControllerBridge {
    bozo::libs::Libraries &libraries;
    uint64_t samples=0;
    void initialize(bozo::native::irx::Env &);
    void sample(bozo::native::irx::Env &);
public:
    explicit ControllerBridge(bozo::libs::Libraries &source):libraries(source){}
    void bind(bozo::native::irx::CompiledLibraries &);
    void reset(){samples=0;}
    uint64_t sampleCount()const{return samples;}
};
}

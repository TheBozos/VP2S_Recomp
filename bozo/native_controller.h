#pragma once
#include "bozo/libs/pad.h"
#include <array>
#include <cstdint>
namespace vp2s {
// DBC's game-owned 128-byte input record. No controller wire commands or device state.
using ControllerRecord=std::array<uint8_t,128>;
void produceControllerRecord(ControllerRecord &record,const bozo::libs::PadInputState &input,
    uint32_t sequence,uint32_t ready,bool pressure,const std::array<uint8_t,4> &capabilities);
}

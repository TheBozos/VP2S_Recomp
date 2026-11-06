#pragma once
#include "bozo/port.h"
namespace vp2s {
// The kernel dispatch ABI is already native. Retire only its SDK firmware-table
// discovery; do not accept arbitrary SetSyscall requests or expose kernel RAM.
void installNativeKernelBindings(bozo::port::Registry &registry);
void initNativeSyscallServices(uint8_t *,R5900Context *,Runtime *);
}

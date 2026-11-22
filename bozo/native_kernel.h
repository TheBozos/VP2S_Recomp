#pragma once
#include <optional>
#include "bozo/port.h"
namespace vp2s {
// The kernel dispatch ABI is already native. Retire only its SDK firmware-table
// discovery; do not accept arbitrary SetSyscall requests or expose kernel RAM.
void installNativeKernelBindings(bozo::port::Registry &registry);
// Preserve the port's qualified raw ABI when the shared engine adds services.
// Only the original native main-thread stack/heap setup uses raw syscalls.
void validateNativeKernelCall(const R5900Context *,uint32_t kind);
void initNativeSyscallServices(uint8_t *,R5900Context *,Runtime *);
void nativeMemmove(uint8_t *,R5900Context *,Runtime *);
// This retires installation only. Runtime alarm/deadline APIs remain unsupported.
void retireAlarmFirmwareInstaller(uint8_t *,R5900Context *,Runtime *);
void initNativeTimer(uint8_t *,R5900Context *,Runtime *);
void startNativeTimer(uint8_t *,R5900Context *,Runtime *);
void queryNativeTimer(uint8_t *,R5900Context *,Runtime *);
void rearmNativeTimer(uint8_t *,R5900Context *,Runtime *);
void dispatchNativeTimer(uint8_t *,R5900Context *,Runtime *);
void loadNativeModule(uint8_t *,R5900Context *,Runtime *);
void nativeModuleTransfer(uint8_t *,R5900Context *,Runtime *);
void nativeModuleTransferStatus(uint8_t *,R5900Context *,Runtime *);
void sendNativeCommand(uint8_t *,R5900Context *,Runtime *);
void freeNativeModuleBuffer(uint8_t *,R5900Context *,Runtime *);
void nativeOsdSettings(uint8_t *,R5900Context *,Runtime *);
void retireEventFirmwareInstaller(uint8_t *,R5900Context *,Runtime *);
void nativeCacheSync(uint8_t *,R5900Context *,Runtime *);
// Observe the guarded object's temporary work-stack switch; original scalar
// SP assignments still execute in generated game code.
// The game's scratchpad work-stack switches: `move sp, t0` (entry) and `move sp, saved` (back).
struct NativeWorkStackSite{uint32_t entry,back;unsigned saved;};
std::optional<NativeWorkStackSite> nativeWorkStackSite(uint32_t pc);
void nativeWorkStack(R5900Context *,Runtime *,uint32_t pc);
}

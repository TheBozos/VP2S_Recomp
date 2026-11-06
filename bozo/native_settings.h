#pragma once
#include <cstdint>
namespace vp2s {
// Logical PC-run preferences encoded only at the SDK buffer boundary. English,
// 4:3, no digital output, OSD ABI version 1. No firmware memory is retained.
struct Settings {
    uint32_t userBits=0x10011u;
    uint32_t packed() const { return (userBits&~0xe000u)|0x2000u; }
    void assign(uint32_t value) { userBits=value&~0xe000u; }
};
class SettingsOwner {
public:
    virtual ~SettingsOwner()=default;
    virtual Settings &settings()=0;
};
}

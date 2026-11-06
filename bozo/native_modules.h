#pragma once
#include <cstdint>
#include <span>
#include <string>
#include <map>
#include <vector>
#include <filesystem>
#include <functional>
#include <string_view>
#include <stdexcept>
namespace vp2s {
struct ModuleResult { int32_t id=0, startResult=0; };
// Sparse host-owned allocations used by SDK file/service jobs. No IOP address
// space, instruction memory, device bank or executable module is modeled.
class ModuleBuffers {
    std::map<uint32_t,std::vector<uint8_t>> buffers;
    uint32_t next=0x40000000,transferId=0;
public:
    uint32_t allocate(uint32_t bytes);
    bool release(uint32_t handle);
    std::span<const uint8_t> view(uint32_t handle) const;
    void validate(uint32_t destination,size_t bytes) const;
    void copy(uint32_t destination,std::span<const uint8_t> data);
    uint32_t completeTransfer();
    void reset();
};
class NativeServiceModules {
    std::filesystem::path root;
    std::function<bool()> ready;
    std::function<int32_t(std::string_view,std::span<const uint8_t>)> startCustom;
    std::map<std::string,int32_t> services;
public:
    NativeServiceModules(std::filesystem::path source,std::function<bool()> nativePadCardReady,
        std::function<int32_t(std::string_view,std::span<const uint8_t>)> custom={})
        :root(std::move(source)),ready(std::move(nativePadCardReady)),startCustom(std::move(custom)){}
    ModuleResult loadBuffer(std::span<const uint8_t> image,std::span<const uint8_t> args);
    void reset(){services.clear();}
};
class ModuleOwner {
public:
    virtual ~ModuleOwner()=default;
    virtual ModuleResult loadNativeModule(const std::string &,std::span<const uint8_t>)=0;
    virtual ModuleResult loadNativeModuleBuffer(uint32_t,std::span<const uint8_t>)=0;
    virtual ModuleBuffers &moduleBuffers()=0;
    virtual void validateNativeTransfer(uint32_t destination,size_t bytes) {
        moduleBuffers().validate(destination,bytes);
    }
    virtual void copyNativeTransfer(uint32_t destination,std::span<const uint8_t> data) {
        moduleBuffers().copy(destination,data);
    }
    virtual uint32_t submitNativeCommand(uint32_t,std::span<const uint8_t>) {
        throw std::runtime_error("native user command provider is not installed");
    }
};
}

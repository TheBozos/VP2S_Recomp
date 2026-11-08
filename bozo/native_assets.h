#pragma once
#include "bozo/runtime.h"
#include <map>
#include <optional>
#include <vector>
namespace vp2s {
// Byte provenance in RAM/scratch storage from actual game I/O and CPU producers. This is not a
// graphics-address map: no transfer payload, hardware layout or device state.
struct AssetOrigin {
    enum class Kind { Disc, Part } kind=Kind::Disc;
    uint64_t disc=0;
    uint32_t part=0,offset=0;
    bool operator==(const AssetOrigin &)const=default;
};
class NativeAssets {
    struct Range {uint32_t begin,end;AssetOrigin origin;};
    struct Invocation {std::optional<AssetOrigin> chain;uint32_t destination,part,job;};
    struct Decode {uint32_t destination,size;std::optional<AssetOrigin> origin;};
    std::vector<Range> ranges;
    std::map<uint32_t,Invocation> calls;
    std::map<uint32_t,Decode> jobs;
    std::map<uint32_t,uint32_t> steps;
    void checkWrite(uint32_t at,uint32_t bytes,uint32_t worker=0)const;
    void assign(uint32_t at,uint32_t bytes,std::optional<AssetOrigin>);
public:
    static uint32_t address(uint32_t);
    void overwrite(uint32_t at,uint32_t bytes);
    void discRead(uint32_t at,uint32_t sectors,uint32_t lbn);
    void copy(uint32_t destination,uint32_t source,uint32_t bytes,uint32_t worker=0);
    std::optional<AssetOrigin> origin(uint32_t at)const;
    void begin(uint32_t stack,uint32_t chain,uint32_t destination,uint32_t part,uint32_t job=0);
    void finish(uint32_t stack,bool success,uint32_t size);
    void beginStep(uint32_t stack,uint32_t job);
    void finishStep(uint32_t stack,bool complete);
    size_t pendingCount()const{return calls.size()+steps.size();}
    size_t jobCount()const{return jobs.size();}
};
class AssetOwner {
public:
    virtual ~AssetOwner()=default;
    virtual NativeAssets &nativeAssets()=0;
};
// Known observations return false so the original scalar instruction executes.
// nullopt means this PC belongs to another adapter.
std::optional<bool> nativeAssetInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
}

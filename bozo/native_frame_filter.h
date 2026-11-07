#pragma once
#include "native_scene.h"
#include <map>
#include <vector>
namespace vp2s {
struct NativeFrameFilter {
    uint32_t object=0,source=0;
    unsigned levels=0;
    struct Layer{unsigned level;uint32_t colour;};
    std::vector<Layer> layers;
    bool finished=false,enabled=true;
};
class NativeFrameFilters {
    std::map<uint32_t,NativeFrameFilter> calls;
    std::map<uint32_t,NativeFrameFilter> ready;
public:
    void begin(Runtime *,uint32_t stack,uint32_t object);
    void levels(uint32_t stack,uint32_t object,unsigned levels);
    void colour(Runtime *,uint32_t stack,uint32_t object,unsigned level,uint32_t colour);
    void finish(uint32_t stack,uint32_t object);
    NativeFrameFilter submit(Runtime *,uint32_t object)const;
};
class FrameFilterOwner {
public:
    virtual ~FrameFilterOwner()=default;
    virtual NativeFrameFilters &nativeFrameFilters()=0;
    virtual void drawNativeFrameFilter(const NativeFrameFilter &)=0;
};
std::optional<bool> nativeFrameFilterInstruction(R5900Context *,Runtime *,uint32_t pc);
}

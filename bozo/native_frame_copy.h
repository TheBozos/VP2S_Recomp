#pragma once
#include "native_scene.h"
#include <map>
namespace vp2s {
// Semantic colour image used by the field DOF passes. Dimensions come from
// the original scalar builder; no packed transfer/texture description is read.
struct NativeFrameCopy {
    uint32_t object=0,width=0,height=0;
    std::shared_ptr<const NativeFrameTarget> frame;
};
class NativeFrameCopies {
    struct Build {uint32_t object=0,source=0;bool active=false;};
    std::map<uint32_t,Build> builds;
    std::map<uint32_t,NativeFrameCopy> ready,images;
public:
    void begin(Runtime *,uint32_t stack,uint32_t object);
    void activate(uint32_t stack,uint32_t object,uint32_t source);
    void finish(Runtime *,const NativeScene &,uint32_t stack,uint32_t object);
    NativeFrameCopy submit(Runtime *,const NativeScene &,uint32_t object)const;
    void published(uint32_t object);
    void release(uint32_t object);
    std::optional<NativeFrameCopy> image(const NativeScene &,uint32_t object)const;
};
class FrameCopyOwner {
public:
    virtual ~FrameCopyOwner()=default;
    virtual NativeFrameCopies &nativeFrameCopies()=0;
    virtual void drawNativeFrameCopy(const NativeFrameCopy &)=0;
};
void drawFrameCopy(const NativeFrameCopy &);
std::optional<bool> nativeFrameCopyInstruction(R5900Context *,Runtime *,uint32_t pc);
}

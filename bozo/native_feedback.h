#pragma once
#include "native_scene.h"
namespace vp2s {
struct NativeFeedbackCopy {
    uint32_t object=0,source=0,writer=0,width=0,height=0;
    std::shared_ptr<const NativeFrameTarget> frame;
    uint8_t mode=0;
    uint32_t sourceWidth=0,sourceHeight=0;
};
class NativeFeedbackCopies {
    std::map<uint32_t,NativeFeedbackCopy> builds,ready;
public:
    void begin(Runtime *,const NativeScene &,uint32_t stack,uint32_t object);
    void finish(Runtime *,uint32_t stack,uint32_t object);
    NativeFeedbackCopy submit(Runtime *,const NativeScene &,uint32_t object);
    void release(uint32_t object);
};
class FeedbackOwner {
public:
    virtual ~FeedbackOwner()=default;
    virtual NativeFeedbackCopies &nativeFeedbackCopies()=0;
    virtual void drawNativeFeedbackCopy(const NativeFeedbackCopy &)=0;
};
void drawFeedbackCopy(const NativeFeedbackCopy &);
std::optional<bool> nativeFeedbackInstruction(R5900Context *,Runtime *,uint32_t);
}

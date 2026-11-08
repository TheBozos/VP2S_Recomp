#pragma once
#include "native_frame_copy.h"
#include <vector>
namespace vp2s {
struct NativeDofBlock {
    uint32_t address=0,width=0,height=0,weightWidth=0,weightHeight=0,z=0;
    float sigma=0,weightSigma=0,gain=0,weightGain=0,weightScale=0,alpha=0;
    bool depthLimit=false;
    std::array<float,256> weights{};
};
struct NativeDof {
    uint32_t object=0,source=0,width=0,height=0;
    float gain=0,depthScale=0,glowAlpha=0;
    bool glow=false;
    std::shared_ptr<const NativeFrameTarget> frame;
    std::vector<NativeDofBlock> blocks;
};
class NativeDofs {
    struct Build{NativeDof effect;};
    std::map<uint32_t,Build> builds;
    std::map<uint32_t,NativeDof> ready;
    struct BlockCall{uint32_t object=0,address=0;};
    std::map<uint32_t,BlockCall> blockCalls;
public:
    void begin(Runtime *,const NativeScene &,uint32_t stack,uint32_t object);
    void block(Runtime *,uint32_t object,uint32_t block);
    void beginBlock(uint32_t stack,uint32_t object,uint32_t block);
    void finishBlock(Runtime *,uint32_t stack,uint32_t object,uint32_t block);
    void finish(Runtime *,const NativeScene &,uint32_t stack,uint32_t object);
    NativeDof submit(Runtime *,const NativeScene &,const NativeFrameCopies &,uint32_t object);
    void release(uint32_t object);
};
class DofOwner {
public:
    virtual ~DofOwner()=default;
    virtual NativeDofs &nativeDofs()=0;
    virtual void drawNativeDof(const NativeDof &)=0;
};
void drawDof(const NativeDof &);
std::optional<bool> nativeDofInstruction(R5900Context *,Runtime *,uint32_t);
}

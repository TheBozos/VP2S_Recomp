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
    float gain=0,depthScale=0;
    // Effect A (post +dfbc, 2d6ea0/2d4ca0): threshold/weight /255 and /128,
    // and the symmetric kernel of the game's Gaussian lerp-tap chain.
    bool glowA=false;float glowThreshold=0,glowWeight=0;uint8_t glowRadius=0;std::array<float,17> glowKernel{};
    // Effect B (post +130e0, 2d71e0/2d7a30): per rotation, the four corners of
    // the 445 image in the (work+1) x (rows+1) work image, truncated as the
    // game's xy/uv; threshold /255, rotate-in and rotate-back gains.
    bool glareB=false;float glareThreshold=0,glareIn=0,glareOut=0;uint32_t glareWork=0,glareRows=0;
    std::vector<std::array<std::array<int32_t,2>,4>> glareCorners;
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

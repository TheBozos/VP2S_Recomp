#pragma once
#include "bozo/runtime.h"
#include <array>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
namespace vp2s {
struct NativeFrameTarget {
    uint32_t object=0;
    float width=0,height=0;
};
struct NativeCamera {
    uint32_t object=0,clearColour=0;
    std::array<float,16> view{},screen{},guard{};
    std::array<float,4> viewport{};
    std::array<float,2> centre{},fog{};
};
struct NativeCameraFrustum {
    uint32_t object=0;
    std::array<std::array<float,4>,10> points{},normals{};
};
struct NativeSphereVisibility {
    uint32_t object=0,source=0;
    std::array<float,4> input{},projected{},nearPoint{},farPoint{};
    std::optional<uint8_t> angle;
    bool outside=false,intersects=false;
};
struct NativeFacingVisibility {
    uint32_t object=0,source=0;
    std::array<float,16> world{};
    std::array<float,4> result{};
    bool ready=false,published=false;
};
struct NativeVisibilityInputs {
    uint32_t scene=0,camera=0;
    std::array<float,16> clip{},view{};
    std::shared_ptr<const NativeCameraFrustum> frustum;
    bool viewReady=false;
    std::optional<NativeSphereVisibility> sphere;
    std::optional<NativeFacingVisibility> facing;
};
// An offscreen draw target (class 153880, scene target n = +68): FRAME_1 / ZBUF_1 / XYOFFSET_1 / SCISSOR_1 from
// scene + e320 + (n - 1) x 60, then its clear (scene + c320 + (n - 1) x 100): sprites over the whole target in one
// colour and Z, Z test GEQUAL after it, SCISSOR inset (`clip`, target pixels, inclusive). The image is the engine's.
struct NativeDrawTarget {
    uint32_t object=0,frame=0,depthBuffer=0,clearColour=0,clearDepth=0;
    int32_t width=0,height=0;
    std::array<int32_t,4> clip{};   // x0, y0, x1, y1 (inclusive) in target pixels
    uint8_t image=0;
    uint32_t block()const{return (frame&0x1ff)*32;}
    uint32_t bufferWidth()const{return (frame>>16)&0x3f;}
    uint32_t format()const{return (frame>>24)&0x3f;}
};
NativeDrawTarget nativeOffscreenTarget(Runtime *,uint32_t object);
struct NativeTextureManagerFrame {uint32_t scene=0,manager=0;bool entered=false,left=false;};
struct NativeBrightness {
    uint32_t object=0;
    bool enabled=false;
    std::array<float,3> gain{},high{},low{};
};
NativeBrightness nativeBrightness(Runtime *,uint32_t object);
class NativeScene {
    std::map<uint32_t,std::shared_ptr<const NativeCamera>> cameras;
    uint64_t cameraPublications=0;
    std::map<uint32_t,NativeFrameTarget> compositions;
    std::shared_ptr<const NativeFrameTarget> target;
    std::shared_ptr<const NativeCameraFrustum> activeFrustum;
    std::shared_ptr<const NativeCamera> selectedCamera;
    uint64_t frustumPublications=0;
    std::map<uint32_t,NativeVisibilityInputs> visibilityInputs;
    std::map<uint32_t,NativeTextureManagerFrame> textureManagerFrames;
    std::optional<NativeDrawTarget> offscreen;
    std::map<uint32_t,NativeDrawTarget> targetTextures;   // by TEX0 block (FBP x 32), the last draw into each
public:
    // Draws go to `target` until endOffscreen (the frame switch 1537e0 back to the main frame).
    void beginOffscreen(const NativeDrawTarget &target){
        if(offscreen)throw std::runtime_error("native offscreen target selected twice");
        offscreen=target;targetTextures[target.block()]=target;}
    void endOffscreen(){offscreen.reset();}
    const std::optional<NativeDrawTarget> &offscreenTarget()const{return offscreen;}
    const NativeDrawTarget *targetTexture(uint32_t block)const{auto it=targetTextures.find(block);return it==targetTextures.end()?nullptr:&it->second;}
    NativeFrameTarget beginComposition(Runtime *,uint32_t stack,uint32_t object);
    void finishComposition(uint32_t stack);
    const NativeFrameTarget &composition(uint32_t stack)const;
    std::shared_ptr<const NativeFrameTarget> frameTarget()const{return target;}
    void publishCamera(Runtime *,uint32_t object);
    std::shared_ptr<const NativeCamera> camera(uint32_t object)const;
    void selectCamera(Runtime *,uint32_t object);
    std::shared_ptr<const NativeCamera> drawingCamera()const{return selectedCamera;}
    uint64_t publicationCount()const{return cameraPublications;}
    void publishFrustum(Runtime *,uint32_t object);
    std::shared_ptr<const NativeCameraFrustum> frustum()const{return activeFrustum;}
    uint64_t frustumPublicationCount()const{return frustumPublications;}
    void beginVisibility(uint32_t stack,uint32_t object);
    void visibilityClip(Runtime *,uint32_t stack,uint32_t camera);
    void visibilityView(Runtime *,uint32_t stack,uint32_t camera);
    // The pass's switch to another camera (an object's own, then back): its frustum just published, its view and clip.
    void visibilityCamera(Runtime *,uint32_t stack,uint32_t camera);
    void visibilitySphere(Runtime *,uint32_t stack,uint32_t object,uint32_t source);
    void visibilityAngle(uint32_t stack,uint32_t object,uint32_t source);
    void beginFacing(Runtime *,uint32_t stack,uint32_t object,uint32_t source);
    void calculateFacing(Runtime *,uint32_t stack,uint32_t object);
    const NativeFacingVisibility &facing(uint32_t stack,uint32_t object)const;
    void publishFacing(Runtime *,uint32_t stack,uint32_t object);
    const NativeSphereVisibility &visibilitySphere(uint32_t stack,uint32_t object,uint32_t source)const;
    const NativeVisibilityInputs &visibility(uint32_t stack)const;
    void finishVisibility(uint32_t stack,uint32_t object);
    void beginTextureManagerFrame(Runtime *,uint32_t stack,uint32_t scene,uint32_t manager);
    const NativeTextureManagerFrame &textureManagerFrame(uint32_t stack)const;
    void textureManagerBoundary(Runtime *,uint32_t stack,uint32_t scene,uint32_t source,uint32_t destination,bool leaving);
    void finishTextureManagerFrame(Runtime *,uint32_t stack,uint32_t scene);
};
class SceneOwner {
public:
    virtual ~SceneOwner()=default;
    virtual NativeScene &nativeScene()=0;
    virtual void beginNativeFrame(const NativeFrameTarget &){throw std::runtime_error("native frame target has no renderer owner");}
    virtual void drawNativeBrightness(const NativeBrightness &){throw std::runtime_error("native brightness has no renderer owner");}
    virtual void beginNativeDrawTarget(const NativeDrawTarget &){throw std::runtime_error("native offscreen target has no renderer owner");}
    virtual void endNativeDrawTarget(){throw std::runtime_error("native offscreen target has no renderer owner");}
};
bool nativeSceneInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
std::optional<bool> nativeCompositionInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
std::optional<bool> nativeVisibilityInstruction(R5900Context *,Runtime *,uint32_t pc);
std::optional<bool> nativeTextureManagerInstruction(R5900Context *,Runtime *,uint32_t pc);
// Frame switches: 153880 (enqueue 21db10, start 21dab0, completion 21daf0) selects an offscreen target, 1537e0 the
// main frame again. Their enqueues complete at once (vp2_bozo_kernel.py); native sites 21db30 / 21d8a0 switch.
// Frame switch 1537e0 (its enqueue 21d880; start 21d820 kicks its chain at +e0 on VIF1, completion 21d860 -> 21c9a0):
// REF 6 qwords (FLUSH, DIRECTHL 5: A+D FRAME_1, ZBUF_1, XYOFFSET_1, SCISSOR_1), CNT 3 (DIRECTHL 2: TEXFLUSH), END.
// True when that chain selects the source's own frame (+1e0, CT32, unmasked; XYOFFSET centred, SCISSOR the
// whole +3c6c x +3c6e): natively nothing changes.
bool nativeFrameSwitchSelectsMain(Runtime *,uint32_t object,uint32_t source);
std::optional<bool> nativeFrameSwitchInstruction(R5900Context *,Runtime *,uint32_t pc);
}

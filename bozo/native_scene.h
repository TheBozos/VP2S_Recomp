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
    std::array<float,4> projected{},nearPoint{},farPoint{};
    bool outside=false,intersects=false;
};
struct NativeVisibilityInputs {
    uint32_t scene=0,camera=0;
    std::array<float,16> clip{},view{};
    std::shared_ptr<const NativeCameraFrustum> frustum;
    bool viewReady=false;
    std::optional<NativeSphereVisibility> sphere;
};
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
public:
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
    void visibilitySphere(Runtime *,uint32_t stack,uint32_t object,uint32_t source);
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
};
bool nativeSceneInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
std::optional<bool> nativeCompositionInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
std::optional<bool> nativeVisibilityInstruction(R5900Context *,Runtime *,uint32_t pc);
std::optional<bool> nativeTextureManagerInstruction(R5900Context *,Runtime *,uint32_t pc);
}

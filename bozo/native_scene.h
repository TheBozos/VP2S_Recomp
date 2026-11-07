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
class NativeScene {
    std::map<uint32_t,std::shared_ptr<const NativeCamera>> cameras;
    uint64_t cameraPublications=0;
    std::map<uint32_t,NativeFrameTarget> compositions;
    std::shared_ptr<const NativeFrameTarget> target;
    std::shared_ptr<const NativeCameraFrustum> activeFrustum;
    uint64_t frustumPublications=0;
public:
    NativeFrameTarget beginComposition(Runtime *,uint32_t stack,uint32_t object);
    void finishComposition(uint32_t stack);
    const NativeFrameTarget &composition(uint32_t stack)const;
    std::shared_ptr<const NativeFrameTarget> frameTarget()const{return target;}
    void publishCamera(Runtime *,uint32_t object);
    std::shared_ptr<const NativeCamera> camera(uint32_t object)const;
    uint64_t publicationCount()const{return cameraPublications;}
    void publishFrustum(Runtime *,uint32_t object);
    std::shared_ptr<const NativeCameraFrustum> frustum()const{return activeFrustum;}
    uint64_t frustumPublicationCount()const{return frustumPublications;}
};
class SceneOwner {
public:
    virtual ~SceneOwner()=default;
    virtual NativeScene &nativeScene()=0;
    virtual void beginNativeFrame(const NativeFrameTarget &){throw std::runtime_error("native frame target has no renderer owner");}
};
bool nativeSceneInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
std::optional<bool> nativeCompositionInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
}

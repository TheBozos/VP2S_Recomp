#pragma once
#include "bozo/runtime.h"
#include <array>
#include <map>
#include <memory>
namespace vp2s {
struct NativeCamera {
    uint32_t object=0,clearColour=0;
    std::array<float,16> view{},screen{},guard{};
    std::array<float,4> viewport{};
    std::array<float,2> centre{},fog{};
};
class NativeScene {
    std::map<uint32_t,std::shared_ptr<const NativeCamera>> cameras;
    uint64_t cameraPublications=0;
public:
    void publishCamera(Runtime *,uint32_t object);
    std::shared_ptr<const NativeCamera> camera(uint32_t object)const;
    uint64_t publicationCount()const{return cameraPublications;}
};
class SceneOwner {
public:
    virtual ~SceneOwner()=default;
    virtual NativeScene &nativeScene()=0;
};
bool nativeSceneInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t pc);
}

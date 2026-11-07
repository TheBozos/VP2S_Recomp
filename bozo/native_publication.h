#pragma once
#include "native_scene.h"
namespace vp2s {
// Invoked only after the original source-frame wait has actually returned.
class NativeFramePublisher {
    bool building=false;
public:
    void next(const NativeFrameTarget &);
};
}

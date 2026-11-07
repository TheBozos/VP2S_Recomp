#pragma once
#include "bozo/runtime.h"
#include "bozo/libs/mpeg.h"
#include <functional>
#include <map>
#include <optional>
namespace vp2s {
struct NativeMediaBuffer {
    uint32_t object=0,capacity=0,semaphore=0;
    std::unique_ptr<bozo::libs::VideoDecoder> decoder;
};
class NativeMediaBuffers {
    using Factory=std::function<std::unique_ptr<bozo::libs::VideoDecoder>()>;
    Factory factory;
    std::map<uint32_t,std::shared_ptr<NativeMediaBuffer>> pending,prepared;
public:
    explicit NativeMediaBuffers(Factory create=bozo::libs::makeVideoDecoder):factory(std::move(create)){}
    void begin(Runtime *,uint32_t stack,uint32_t object);
    void finish(Runtime *,uint32_t stack);
    void release(uint32_t object);
    std::shared_ptr<const NativeMediaBuffer> buffer(uint32_t object)const;
    size_t outstanding()const{return pending.size();}
};
class MediaBufferOwner {
public:
    virtual ~MediaBufferOwner()=default;
    virtual NativeMediaBuffers &nativeMediaBuffers()=0;
};
std::optional<bool> nativeMediaBufferInstruction(R5900Context *,Runtime *,uint32_t pc);
}

#pragma once
#include "bozo/runtime.h"
#include "bozo/libs/mpeg.h"
#include <functional>
#include <map>
#include <optional>
#include <array>
namespace vp2s {
struct NativeMediaBuffer {
    struct Callback {uint32_t function=0,data=0,gp=0;bool set=false;};
    uint32_t object=0,capacity=0,semaphore=0;
    uint64_t pacingFieldCycles=0;
    bool initialized=false;
    bool created=false;
    uint32_t mp=0,work=0,workBytes=0;
    std::array<Callback,6> callbacks{};
    std::unique_ptr<bozo::libs::VideoDecoder> decoder;
};
class NativeMediaBuffers {
    using Factory=std::function<std::unique_ptr<bozo::libs::VideoDecoder>()>;
    Factory factory;
    std::map<uint32_t,std::shared_ptr<NativeMediaBuffer>> pending,prepared;
    struct Creation {std::shared_ptr<NativeMediaBuffer> owner;uint32_t mp,work,bytes;};
    std::map<uint32_t,Creation> creations;
    std::shared_ptr<NativeMediaBuffer> stream(Runtime *,uint32_t mp)const;
public:
    explicit NativeMediaBuffers(Factory create=bozo::libs::makeVideoDecoder):factory(std::move(create)){}
    void begin(Runtime *,uint32_t stack,uint32_t object);
    void finish(Runtime *,uint32_t stack);
    void release(uint32_t object);
    void preparePacing(Runtime *,uint32_t object,uint64_t fieldCycles);
    void initialize(Runtime *,uint32_t parent);
    void beginCreation(Runtime *,uint32_t stack,uint32_t object,uint32_t mp,uint32_t work,uint32_t bytes);
    void creationEntry(uint32_t stack,uint32_t mp,uint32_t work,uint32_t bytes)const;
    void finishCreation(Runtime *,uint32_t stack,uint32_t result);
    void validateCallback(Runtime *,uint32_t mp,uint32_t type,uint32_t data)const;
    void captureCallback(Runtime *,uint32_t mp,uint32_t type);
    std::shared_ptr<const NativeMediaBuffer> buffer(uint32_t object)const;
    size_t outstanding()const{return pending.size()+creations.size();}
};
class MediaBufferOwner {
public:
    virtual ~MediaBufferOwner()=default;
    virtual NativeMediaBuffers &nativeMediaBuffers()=0;
};
std::optional<bool> nativeMediaBufferInstruction(R5900Context *,Runtime *,uint32_t pc);
bool nativeMediaLibrary(R5900Context *,Runtime *,uint32_t target);
}

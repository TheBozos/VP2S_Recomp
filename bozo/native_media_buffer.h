#pragma once
#include "bozo/runtime.h"
#include "bozo/libs/mpeg.h"
#include "native_movie_video.h"
#include <functional>
#include <map>
#include <optional>
#include <array>
#include <stdexcept>
namespace vp2s {
class MediaBufferOwner;
struct NativeMediaBuffer {
    struct Callback {uint32_t function=0,data=0,gp=0;bool set=false;};
    struct StreamCallback {uint64_t pattern=0,mask=0;Callback callback;};
    uint32_t object=0,capacity=0,semaphore=0;
    uint64_t pacingFieldCycles=0;
    bool initialized=false;
    bool created=false;
    bool inputResetPending=false;
    uint32_t mp=0,work=0,workBytes=0;
    std::array<Callback,6> callbacks{};
    std::vector<StreamCallback> streams;
    std::unique_ptr<bozo::libs::VideoDecoder> decoder;
    NativeMovieVideo video;
    uint64_t presentationBase=0,displayedPictures=0;
    bool presentationStarted=false,presentationClosed=false;
    std::function<void()> closePresentation;
};
class NativeMediaBuffers {
    using Factory=std::function<std::unique_ptr<bozo::libs::VideoDecoder>()>;
    Factory factory;
    std::map<uint32_t,std::shared_ptr<NativeMediaBuffer>> pending,prepared;
    struct Creation {std::shared_ptr<NativeMediaBuffer> owner;uint32_t mp,work,bytes;};
    std::map<uint32_t,Creation> creations;
    struct Reset {std::shared_ptr<NativeMediaBuffer> owner;std::unique_ptr<bozo::libs::VideoDecoder> decoder;};
    std::map<uint32_t,Reset> resets;
    struct Demux {std::shared_ptr<NativeMediaBuffer> owner;uint32_t ring,capacity,input,bytes;};
    std::map<uint32_t,Demux> demuxes;
    std::shared_ptr<NativeMediaBuffer> stream(Runtime *,uint32_t mp)const;
public:
    explicit NativeMediaBuffers(Factory create=bozo::libs::makeVideoDecoder):factory(std::move(create)){}
    void begin(Runtime *,uint32_t stack,uint32_t object);
    void finish(Runtime *,uint32_t stack);
    void beginInputReset(Runtime *,uint32_t stack,uint32_t object);
    void release(uint32_t object);
    void preparePacing(Runtime *,uint32_t object,uint64_t fieldCycles);
    void initialize(Runtime *,uint32_t parent);
    void beginCreation(Runtime *,uint32_t stack,uint32_t object,uint32_t mp,uint32_t work,uint32_t bytes);
    void creationEntry(uint32_t stack,uint32_t mp,uint32_t work,uint32_t bytes)const;
    void finishCreation(Runtime *,uint32_t stack,uint32_t result);
    void validateCallback(Runtime *,uint32_t mp,uint32_t type,uint32_t data)const;
    void captureCallback(Runtime *,uint32_t mp,uint32_t type);
    void validateStreamRegistration(Runtime *,uint32_t mp)const;
    void captureStreams(Runtime *,uint32_t mp);
    void beginReset(Runtime *,uint32_t stack,uint32_t mp);
    void resetEntry(uint32_t stack,uint32_t mp)const;
    void resetHelper(uint32_t stack,uint32_t work)const;
    void finishReset(Runtime *,uint32_t stack);
    void beginDemux(Runtime *,uint32_t stack,uint32_t mp,uint32_t input,uint32_t bytes,uint32_t ring,uint32_t capacity);
    void demuxEntry(uint32_t stack,uint32_t mp,uint32_t input,uint32_t bytes,uint32_t ring,uint32_t capacity)const;
    void finishDemux(uint32_t stack,uint32_t consumed);
    void finishPss(Runtime *,uint32_t stack,uint32_t consumed);
    bool videoPayload(Runtime *,R5900Context *);
    void validateAudioPayload(Runtime *,R5900Context *,uint32_t function)const;
    int advanceMovie(Runtime *,uint32_t object,uint64_t now,MediaBufferOwner &);
    bool presentationReady(Runtime *,uint32_t audio,uint32_t originalOutput)const;
    void endPresentation(uint32_t object);
    bool referenceEmpty(Runtime *,uint32_t mp)const;
    std::shared_ptr<const NativeMediaBuffer> buffer(uint32_t object)const;
    size_t outstanding()const{return pending.size()+creations.size()+resets.size()+demuxes.size();}
};
class MediaBufferOwner {
public:
    virtual ~MediaBufferOwner()=default;
    virtual NativeMediaBuffers &nativeMediaBuffers()=0;
    virtual void presentNativeMovie(uint32_t,const bozo::libs::YuvPicture &,uint64_t){throw std::runtime_error("native movie presenter is missing");}
    virtual void closeNativeMovie(uint32_t){throw std::runtime_error("native movie presenter is missing");}
};
std::optional<bool> nativeMediaBufferInstruction(R5900Context *,Runtime *,uint32_t pc);
bool nativeMediaLibrary(R5900Context *,Runtime *,uint32_t target);
}

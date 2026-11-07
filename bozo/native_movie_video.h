#pragma once
#include "bozo/libs/mpeg.h"
#include <deque>
namespace vp2s {
// Native MPEG elementary-stream framing. No SDK/device protocol lives here.
// Start-code grouping follows BozoEngine's format-only MPEG scanner.
class NativeMovieVideo {
    std::vector<uint8_t> coded;
    std::vector<size_t> cuts;
    size_t scanned=0;
    bool headerOpen=false,ended=false,flushed=false;
    void pump(bozo::libs::VideoDecoder &);
public:
    struct Timestamp {uint64_t byte;int64_t pts,dts;};
    std::deque<Timestamp> timestamps;
    std::deque<bozo::libs::YuvPicture> pictures;
    uint64_t acceptedBytes=0,submittedPictures=0;
    static constexpr size_t capacity=0xc8000,pictureCapacity=8;
    bool accept(bozo::libs::VideoDecoder &,const uint8_t *,size_t,int64_t pts,int64_t dts);
    bool takePicture(bozo::libs::VideoDecoder &,bozo::libs::YuvPicture &);
    bool sequenceEnded()const{return ended;}
    size_t pendingBytes()const{return coded.size();}
};
}

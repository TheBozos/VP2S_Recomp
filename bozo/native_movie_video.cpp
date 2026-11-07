#include "native_movie_video.h"
#include <stdexcept>
#include <algorithm>
namespace vp2s {
void NativeMovieVideo::pump(bozo::libs::VideoDecoder &decoder){
    while(pictures.size()<pictureCapacity && (cuts.size()>1 || (ended && !flushed))){
        const auto bytes=cuts.size()>1?cuts[1]:coded.size();
        if(!cuts.empty() && bytes>cuts.front()){
            decoder.picture(coded.data()+cuts.front(),bytes-cuts.front());++submittedPictures;
        }
        if(cuts.size()>1){
            coded.erase(coded.begin(),coded.begin()+bytes);scanned-=bytes;
            cuts.erase(cuts.begin());for(auto &cut:cuts)cut-=bytes;
        }else{
            coded.clear();cuts.clear();scanned=0;decoder.flush();flushed=true;
        }
        bozo::libs::YuvPicture picture;
        while(decoder.next(picture)){
            if(picture.width<=0 || picture.height<=0 || picture.width>4096 || picture.height>4096 ||
               picture.y.size()!=size_t(picture.width)*picture.height ||
               picture.u.size()!=size_t(picture.width/2)*(picture.height/2) || picture.v.size()!=picture.u.size())
                throw std::runtime_error("native movie decoder returned an invalid YUV picture");
            pictures.push_back(std::move(picture));
        }
    }
}
bool NativeMovieVideo::accept(bozo::libs::VideoDecoder &decoder,const uint8_t *data,size_t bytes,int64_t pts,int64_t dts){
    if(!data || !bytes || bytes>capacity)throw std::runtime_error("native movie payload is empty or exceeds input capacity");
    if(ended)throw std::runtime_error("native movie payload follows sequence end");
    if(pictures.size()>=pictureCapacity || bytes>capacity-coded.size())return false;
    timestamps.push_back({acceptedBytes,pts,dts});acceptedBytes+=bytes;
    coded.insert(coded.end(),data,data+bytes);
    size_t at=scanned;
    while(at+4<=coded.size()){
        if(coded[at]==0 && coded[at+1]==0 && coded[at+2]==1){
            const auto code=coded[at+3];
            if(code==0){if(!headerOpen)cuts.push_back(at);headerOpen=false;}
            else if(code==0xb3 || code==0xb8){if(!headerOpen)cuts.push_back(at);headerOpen=true;}
            else if(code==0xb7)ended=true;
            at+=4;
        }else ++at;
    }
    scanned=at;pump(decoder);
    // Timestamp storage follows the bounded coded/picture queues. Preserve
    // the last timestamp preceding the oldest retained coded byte.
    const auto retained=acceptedBytes-coded.size();
    while(timestamps.size()>1 && timestamps[1].byte<retained)timestamps.pop_front();
    return true;
}
bool NativeMovieVideo::takePicture(bozo::libs::VideoDecoder &decoder,bozo::libs::YuvPicture &out){
    pump(decoder);if(pictures.empty())return false;
    out=std::move(pictures.front());pictures.pop_front();pump(decoder);return true;
}
}

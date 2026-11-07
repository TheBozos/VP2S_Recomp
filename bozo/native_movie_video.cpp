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
            else if(code==0xb3 || code==0xb8){
                if(code==0xb3){
                    if(at+8>coded.size())break;
                    static constexpr uint32_t numerators[9]={0,24000,24,25,30000,30,50,60000,60};
                    const auto rate=coded[at+7]&15;
                    if(!rate || rate>8)throw std::runtime_error("unsupported native MPEG sequence frame rate");
                    const auto numerator=numerators[rate],denominator=(rate==1 || rate==4 || rate==7)?1001u:1u;
                    if(frameRateNumerator && (frameRateNumerator!=numerator || frameRateDenominator!=denominator))
                        throw std::runtime_error("native MPEG sequence changes frame rate during playback");
                    frameRateNumerator=numerator;frameRateDenominator=denominator;
                }
                if(!headerOpen)cuts.push_back(at);headerOpen=true;
            }
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
void NativeMovieVideo::finishInput(bozo::libs::VideoDecoder &decoder){ended=true;pump(decoder);}
std::shared_ptr<const bozo::render::Texture> nativeMoviePicture(const bozo::libs::YuvPicture &p,uint64_t key,uint64_t version){
    if(p.width<=0 || p.height<=0 || p.width>4096 || p.height>4096 || (p.width&1) || (p.height&1) ||
       p.y.size()!=size_t(p.width)*p.height || p.u.size()!=size_t(p.width/2)*(p.height/2) || p.v.size()!=p.u.size())
        throw std::runtime_error("native movie presentation has invalid YUV planes");
    auto texture=std::make_shared<bozo::render::Texture>();texture->width=p.width;texture->height=p.height;
    texture->key=key;texture->version=version;texture->rgba.resize(p.y.size());
    const auto clamp=[](int value){return uint32_t(std::clamp(value,0,255));};
    for(int y=0;y<p.height;++y)for(int x=0;x<p.width;++x){
        const auto pixel=size_t(y)*p.width+x,chroma=size_t(y/2)*(p.width/2)+x/2;
        const int l=(9539*(int(p.y[pixel])-16))>>13,u=int(p.u[chroma])-128,v=int(p.v[chroma])-128;
        const auto r=clamp(l+((13074*v)>>13)),g=clamp(l+((-3210*u)>>13)+((-6660*v)>>13)),b=clamp(l+((16524*u)>>13));
        texture->rgba[pixel]=r|(g<<8)|(b<<16)|0xff000000;
    }
    return texture;
}
}

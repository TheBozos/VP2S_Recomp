#include "native_procedural.h"
#include "native_assets.h"
#include "bozo/ee/dispatcher.h"
#include "../port/vp2s_music_math.h"
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>
namespace vp2s {
namespace {
uint32_t address(uint32_t at){return at>=0x70000000 && at<0x70004000?at:NativeAssets::address(at);}
uint8_t *record(Runtime *rt,uint32_t address,uint32_t bytes){
    const auto at=vp2s::address(address);
    if(!rt || (at&15) || !((at>=0x100000 && uint64_t(at)+bytes<=0x2000000) || (at>=0x70000000 && uint64_t(at)+bytes<=0x70004000)))throw std::runtime_error("procedural projection outside owned memory");
    auto *p=rt->memory.hostPointer(address,bytes);if(!p)throw std::runtime_error("procedural projection missing memory");return p;
}
uint32_t word(const uint8_t *source){uint32_t out;std::memcpy(&out,source,4);return out;}
void producer(const R5900Context *c){
    if(!c || c->pc!=0x2481d0 || c->r[31].uw[0]!=0x2481d0 || c->r[4].uw[0]!=c->r[29].uw[0]+0x130)throw std::runtime_error("procedural projection escaped original producer");
}
float normal(float v){const auto b=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((b&0x7f800000)?b:b&0x80000000);}
float multiply(float a,float b){volatile float v=a*b;return normal(v);}
float add(float a,float b){volatile float v=a+b;return normal(v);}
std::array<float,4> projected(const std::array<float,16> &matrix,const std::array<float,4> &position){
    music::CodecRounding rounding;
    const auto qualify=[](float v,float limit){const auto b=std::bit_cast<uint32_t>(v)&0x7fffffff;
        if(b>=0x7f800000 || (b && b<0x00800000) || std::abs(v)>limit)throw std::runtime_error("unqualified procedural geometry input");};
    for(float v:matrix)qualify(v,1.e12f);for(unsigned i=0;i<3;++i)qualify(position[i],1.e8f);
    std::array<float,4> result;
    for(unsigned row=0;row<4;++row){const auto *m=matrix.data()+4*row;
        result[row]=add(add(add(multiply(m[0],position[0]),multiply(m[1],position[1])),multiply(m[2],position[2])),multiply(m[3],1.f));}
    if(result[3]==0)throw std::runtime_error("unqualified procedural projection divisor");volatile float reciprocal=1.f/result[3];
    const auto q=normal(reciprocal);if(!std::isfinite(q))throw std::runtime_error("procedural projection reciprocal overflow");
    for(float &v:result){if(std::abs(double(v)*q)>std::numeric_limits<float>::max())throw std::runtime_error("procedural projection overflow");v=multiply(v,q);}return result;
}
}
void NativeProceduralTransforms::publish(Runtime *rt,const R5900Context *c,const NativeScene &scene){
    producer(c);const auto stack=c->r[29].uw[0],object=c->r[17].uw[0],camera=c->r[16].uw[0];auto *frame=record(rt,stack,0x170);
    const auto caller=word(frame+0x20);if(caller!=0x202308 && caller!=0x202bc8 && caller!=0x203d28)throw std::runtime_error("procedural projection has unknown scalar caller");
    record(rt,object,0x250);const auto view=scene.drawingCamera();if(!view || view->object!=camera)throw std::runtime_error("procedural projection has no selected camera");
    NativeProceduralProjection input{object,camera,address(stack+0x170)};
    for(unsigned row=0;row<4;++row)for(unsigned column=0;column<4;++column){
        const auto bits=word(frame+0x130+16*column+4*row),magnitude=bits&0x7fffffff;
        if(magnitude>=0x7f800000 || (magnitude && magnitude<0x00800000))throw std::runtime_error("invalid procedural projection coefficient");
        input.screenFromLocal[4*row+column]=std::bit_cast<float>(bits);
    }
    if(!draws.emplace(input.frame,input).second)throw std::runtime_error("procedural projection overlaps an active draw");
}
const NativeProceduralProjection &NativeProceduralTransforms::projection(uint32_t frame,uint32_t object)const{
    const auto found=draws.find(address(frame));if(found==draws.end() || found->second.object!=object)throw std::runtime_error("procedural point has no matching draw projection");return found->second;
}
void NativeProceduralTransforms::finish(uint32_t frame,uint32_t object){projection(frame,object);const auto key=address(frame);
    if(points.contains(key))throw std::runtime_error("procedural finish has an unpublished point");draws.erase(key);}
void NativeProceduralTransforms::point(Runtime *rt,R5900Context *c,uint32_t pc){
    if(!c || (c->pc!=0x247700 && c->pc!=0x2478c0 && c->pc!=0x247974) || c->r[31].uw[0]!=0x247700)throw std::runtime_error("procedural point escaped original scalar owner");
    const auto stack=c->r[29].uw[0],key=address(stack+0xb0),object=c->r[21].uw[0],index=c->r[5].uw[0],count=c->r[20].uw[0];
    const auto &input=projection(key,object);if(!count || count>65536 || index>=count)throw std::runtime_error("procedural point index outside scalar array");
    const auto first64=uint64_t(c->r[19].uw[0])+16ull*index,second64=uint64_t(c->r[18].uw[0])+16ull*index;
    if(first64>UINT32_MAX || second64>UINT32_MAX)throw std::runtime_error("procedural point address overflow");
    const auto first=uint32_t(first64),second=uint32_t(second64);
    if(pc==0x2478c8){
        if(c->r[9].uw[0]!=first || points.contains(key))throw std::runtime_error("procedural point begins twice or at wrong source");
        std::array<float,4> position;std::memcpy(position.data(),record(rt,first,16),16);Point point{index,first,second};point.projected=projected(input.screenFromLocal,position);points.emplace(key,point);return;
    }
    auto found=points.find(key);if(found==points.end() || found->second.index!=index || found->second.first!=first || found->second.second!=second)throw std::runtime_error("procedural point has no matching snapshot");auto &p=found->second;
    if(pc==0x247968){
        if(p.phase || c->r[9].uw[0]!=second)throw std::runtime_error("procedural second point read out of order");
        std::array<float,4> position;std::memcpy(position.data(),record(rt,second,16),16);p.destination=position;p.phase=1;
    }else if(pc==0x24796c){
        if(p.phase!=1 || c->r[4].uw[0]!=stack+0x90)throw std::runtime_error("procedural projected point publication out of order");
        auto *out=record(rt,stack+0x90,16);std::memcpy(out,p.projected.data(),16);p.phase=2;
    }else if(pc==0x247974){
        if(p.phase!=2)throw std::runtime_error("procedural destination calculation out of order");
        music::CodecRounding rounding;
        const auto position=projected(input.screenFromLocal,p.destination);std::array<int32_t,4> fixed;
        for(unsigned i=0;i<4;++i){const auto scaled=double(position[i])*16;
            fixed[i]=scaled>=INT32_MAX?INT32_MAX:scaled<=INT32_MIN?INT32_MIN:int32_t(scaled);}p.fixed=fixed;p.phase=3;
    }else if(pc==0x247a18){if(p.phase!=3)throw std::runtime_error("procedural constant read before destination");c->r[9].ud[0]=0;p.phase=4;
    }else if(pc==0x247a1c){
        if(p.phase!=4 || c->r[3].uw[0]!=stack+0x80)throw std::runtime_error("procedural fixed destination publication out of order");
        auto *out=record(rt,stack+0x80,16);std::memcpy(out,p.fixed.data(),16);points.erase(found);
    }else if(p.phase)throw std::runtime_error("procedural projection calculation after publication");
}
std::optional<bool> nativeProceduralInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool publish=pc>=0x248260 && pc<=0x24826c && !(pc&3);
    const bool point=pc==0x2478c8 || pc==0x2478cc || pc==0x2478d0 || pc==0x2478d4 || pc==0x2478d8 || pc==0x247918 || pc==0x24795c || pc==0x247968 || pc==0x24796c || pc==0x247974 || pc==0x247a18 || pc==0x247a1c;
    if(!publish && !point && pc!=0x247ec0)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;auto *owner=d?dynamic_cast<ProceduralOwner *>(&d->host()):nullptr;auto *scene=d?dynamic_cast<SceneOwner *>(&d->host()):nullptr;
    if(!c || !owner || !scene)throw std::runtime_error("procedural projection lacks semantic owner");
    if(point){owner->nativeProceduralTransforms().point(rt,c,pc);return true;}
    if(publish){producer(c);if(pc==0x24826c)owner->nativeProceduralTransforms().publish(rt,c,scene->nativeScene());return true;}
    if(c->pc!=0x247ec0 || (c->r[31].uw[0]!=0x202960 && c->r[31].uw[0]!=0x2038d0 && c->r[31].uw[0]!=0x2053f8))throw std::runtime_error("procedural draw finish has unknown caller");
    owner->nativeProceduralTransforms().finish(c->r[29].uw[0],c->r[4].uw[0]);return false;
}
}

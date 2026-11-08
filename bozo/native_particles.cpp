#include "native_particles.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <bit>
#include <algorithm>
#include <cmath>
#include <immintrin.h>
#include <stdexcept>
namespace vp2s {
namespace {
uint8_t *record(Runtime *rt,uint32_t at,uint32_t bytes){
    const bool scratch=at>=bozo::kScratchpadBase && at<bozo::kScratchpadBase+bozo::kScratchpadBytes;
    if(!rt || (at&15) || (!scratch && ((at&0x1ffffff)<0x100000 || (at>>28)==7)))throw std::runtime_error("invalid native particle record");
    auto *p=rt->memory.hostPointer(at,bytes);if(!p)throw std::runtime_error("native particle record outside owned storage");return p;
}
uint32_t word(const uint8_t *p,unsigned at){uint32_t v;std::memcpy(&v,p+at,4);return v;}
bool overlaps(uint8_t *a,size_t na,uint8_t *b,size_t nb){const auto x=reinterpret_cast<uintptr_t>(a),y=reinterpret_cast<uintptr_t>(b);return x<y+nb && y<x+na;}
}
void NativeParticles::begin(Runtime *rt,R5900Context *c){
    const auto stack=c->r[29].uw[0],object=c->r[21].uw[0],index=c->r[30].uw[0];auto *frame=record(rt,stack,0x160),*owner=record(rt,object,0xb10);
    if(c->pc!=0x22eadc || word(owner,0)!=0x154180 || owner[0xaff]!=3 || owner[0xb04] || index>1 || owner[0xaf9]!=index)
        throw std::runtime_error("unqualified native particle producer");
    const auto first=NativeAssets::address(word(owner,0xaa0)),second=NativeAssets::address(word(owner,0xaa4));
    if(second<=first || second-first<0x20 || second-first>0x1000000)throw std::runtime_error("invalid native particle buffer halves");
    const auto span=second-first,base=index?second:first,limit=base+span,emitter=word(owner,0xa90);
    auto *buffers=record(rt,first,span*2);record(rt,emitter,16);
    if(overlaps(frame,0x160,owner,0xb10) || overlaps(buffers,span*2,owner,0xb10) || overlaps(buffers,span*2,frame,0x160) || word(frame,0xe0)!=base+0x20)
        throw std::runtime_error("native particle buffer/frame ownership mismatch");
    for(const auto &[active,build]:builds)if(build.object==object || overlaps(frame,0x160,record(rt,active,0x160),0x160))
        throw std::runtime_error("native particle producer repeats an active lifetime");
    builds.emplace(stack,Build{object,emitter,base,limit,base+0x20,index});
}
void NativeParticles::publish(Runtime *rt,R5900Context *c,NativeAssets &assets){
    const auto stack=c->r[29].uw[0];auto it=builds.find(stack);
    if(it==builds.end())throw std::runtime_error("native particle publication has no producer");
    auto &build=it->second;auto *owner=record(rt,build.object,0xb10),*frame=record(rt,stack,0x160);
    const auto source=c->r[16].uw[0],count=c->r[2].uw[0],group=c->r[18].uw[0],qwords=c->r[5].uw[0];
    const auto span=build.limit-build.base,first=build.base-build.index*span;
    if(build.pending || c->r[21].uw[0]!=build.object || word(owner,0)!=0x154180 || owner[0xaff]!=3 || word(owner,0xa90)!=build.emitter ||
       c->r[30].uw[0]!=build.index || owner[0xaf9]!=build.index || owner[0xb04] ||
       NativeAssets::address(word(owner,0xaa0))!=first || NativeAssets::address(word(owner,0xaa4))!=first+span ||
       source!=bozo::kScratchpadBase+(build.chunks&1?0xa00:0) || c->r[19].uw[0]!=build.chunks+1 ||
       !count || count>48 || !group || group>16 || count%group || qwords!=count*3+2 || c->r[4].uw[0]!=qwords*16 ||
       word(frame,0xe0)!=build.next || c->r[3].uw[0]!=build.next || qwords*16>build.limit-build.next)
        throw std::runtime_error("native particle publication escaped its explicit chunk/buffer");
    auto *from=record(rt,source,qwords*16),*to=record(rt,build.next,qwords*16);
    if(word(from,0x10)!=count || word(from,0x14)!=qwords || word(from,0x18)!=group || overlaps(from,qwords*16,frame,0x160))
        throw std::runtime_error("native particle point record mismatch");
    // The original scalar emitter/compactor already wrote complete semantic
    // point records. Publish those bytes directly, before its cursor update.
    assets.overwrite(build.next,qwords*16);std::memcpy(to,from,qwords*16);
    build.next+=qwords*16;++build.chunks;build.points+=count;build.pending=1;
}
void NativeParticles::retire(Runtime *rt,R5900Context *c,uint32_t pc){
    auto it=builds.find(c->r[29].uw[0]);if(it==builds.end())throw std::runtime_error("native particle retirement has no producer");
    auto &build=it->second;
    if(pc==0x22ec9c || pc==0x22eca0){
        if(build.pending || c->r[21].uw[0]!=build.object || c->r[3].uw[0]!=0x35dbc0 || c->r[2].uw[0]!=0x35dbe0)
            throw std::runtime_error("native particle bounds source mismatch");
        if(pc==0x22ec9c){if(build.boundsReady)throw std::runtime_error("native particle bounds source repeated");ParticleBounds box;std::memcpy(box.minimum.data(),record(rt,0x35dbc0,16),16);std::memcpy(box.maximum.data(),record(rt,0x35dbe0,16),16);
            for(const auto *v:{&box.minimum,&box.maximum})for(unsigned i=0;i<3;++i){const auto bits=std::bit_cast<uint32_t>((*v)[i]),magnitude=bits&0x7fffffff;
                if(magnitude>0x7f800000 || (magnitude && magnitude<0x00800000))throw std::runtime_error("unqualified native particle bounds source");}
            build.bounds=box;build.boundsReady=true;
        }else if(!build.boundsReady)throw std::runtime_error("native particle bounds source order mismatch");
        return;
    }
    constexpr uint32_t sites[]={0x22ee94,0x22ee9c,0x22eea4,0x22eebc};
    if(!build.pending || build.pending>4 || sites[build.pending-1]!=pc || c->r[21].uw[0]!=build.object || c->r[19].uw[0]!=build.chunks)
        throw std::runtime_error("native particle publication retirement is out of order");
    if(pc==0x22eebc && word(record(rt,c->r[29].uw[0],0x160),0xe0)!=build.next)throw std::runtime_error("native particle cursor was not committed");
    build.pending=build.pending==4?0:build.pending+1;
}
void NativeParticles::validateEmitter(uint32_t frame,uint32_t emitter)const{
    const auto it=builds.find(frame);if(it==builds.end() || it->second.emitter!=emitter)throw std::runtime_error("native particle bounds have no matching producer");
}
void NativeParticles::validateDraw(uint32_t object)const{
    for(const auto &[frame,build]:builds)if(build.object==object)throw std::runtime_error("native particle draw overlaps an unfinished point producer");
}
void NativeParticles::includePoint(uint32_t frame,uint32_t emitter,const std::array<float,4> &point){
    validateEmitter(frame,emitter);auto it=builds.find(frame);
    for(unsigned i=0;i<3;++i){const auto bits=std::bit_cast<uint32_t>(point[i]),magnitude=bits&0x7fffffff;
        if(magnitude>=0x7f800000 || (magnitude && magnitude<0x00800000))throw std::runtime_error("unqualified native particle bounds point");}
    // Earlier simulation substeps do not publish bounds. The final substep's
    // explicit source observation starts the box used by the original caller.
    auto &build=it->second;if(!build.boundsReady)return;
    for(unsigned i=0;i<3;++i){if(point[i]<build.bounds.minimum[i])build.bounds.minimum[i]=point[i];if(point[i]>build.bounds.maximum[i])build.bounds.maximum[i]=point[i];}
}
const ParticleBounds &NativeParticles::bounds(uint32_t frame,uint32_t emitter)const{
    const auto it=builds.find(frame);if(it==builds.end() || it->second.emitter!=emitter || !it->second.boundsReady)throw std::runtime_error("native particle bounds have no completed source");return it->second.bounds;
}
void NativeParticles::publishSphere(Runtime *rt,R5900Context *c){
    struct ArithmeticScope{unsigned saved=_mm_getcsr();~ArithmeticScope(){_mm_setcsr(saved);}} arithmetic;
    auto it=builds.find(c->r[29].uw[0]);if(it==builds.end())throw std::runtime_error("native particle sphere has no producer");
    auto &build=it->second;auto *owner=record(rt,build.object,0xb10),*frame=record(rt,c->r[29].uw[0],0x160);
    if(c->pc!=0x22efc8 || c->r[21].uw[0]!=build.object || c->r[2].uw[0]!=build.object+0xa60 || word(owner,0)!=0x154180 || owner[0xaf9]!=build.index || c->r[19].uw[0]!=build.chunks || build.pending || !build.points || !build.boundsReady || build.boundsPublished ||
       word(owner,0xa90)!=build.emitter || !record(rt,build.emitter,16)[4] || word(owner,0xabc)!=build.points || word(owner,0xac0+build.index*4)!=build.chunks || word(frame,0xe0)!=build.next)
        throw std::runtime_error("native particle sphere ownership mismatch");
    float padding;std::memcpy(&padding,frame+0xf0,4);
    const auto qualify=[](float value){const auto bits=std::bit_cast<uint32_t>(value),m=bits&0x7fffffff;
        if(m>=0x7f800000 || (m && m<0x00800000) || std::abs(value)>1.e8f)throw std::runtime_error("unqualified native particle sphere input");};
    qualify(padding);qualify(build.bounds.maximum[3]);
    for(unsigned i=0;i<3;++i){qualify(build.bounds.minimum[i]);qualify(build.bounds.maximum[i]);if(build.bounds.minimum[i]>build.bounds.maximum[i])throw std::runtime_error("empty native particle sphere bounds");}
    const auto normal=[](float value){const auto bits=std::bit_cast<uint32_t>(value);return std::bit_cast<float>((bits&0x7f800000)?bits:bits&0x80000000);};
    const auto sum=[&](float a,float b){volatile float v=a+b;return normal(v);};
    const auto product=[&](float a,float b){volatile float v=a*b;return normal(v);};
    std::array<float,4> sphere;float square[3];
    for(unsigned i=0;i<3;++i){sphere[i]=product(sum(build.bounds.minimum[i],build.bounds.maximum[i]),build.bounds.maximum[3]);
        const float extent=sum(sum(build.bounds.maximum[i],-sphere[i]),padding);square[i]=product(extent,extent);}
    const auto squared=sum(sum(square[0],square[1]),product(1.f,square[2]));
    volatile float root=static_cast<float>(std::sqrt(std::abs(double(squared))));sphere[3]=product(1.f,root);
    std::memcpy(owner+0xa40,build.bounds.minimum.data(),16);std::memcpy(owner+0xa50,build.bounds.maximum.data(),16);std::memcpy(owner+0xa60,sphere.data(),16);build.boundsPublished=true;
}
void NativeParticles::finish(Runtime *rt,R5900Context *c){
    auto it=builds.find(c->r[29].uw[0]);if(it==builds.end())return;const auto &build=it->second;
    auto *owner=record(rt,build.object,0xb10),*frame=record(rt,c->r[29].uw[0],0x160);
    if(build.pending || (build.points && record(rt,build.emitter,16)[4] && !build.boundsPublished) || c->r[21].uw[0]!=build.object || owner[0xaf9]!=(build.index^1) || word(owner,0xac0+build.index*4)!=build.chunks ||
       word(owner,0xabc)!=build.points || word(frame,0xe0)!=build.next)
        throw std::runtime_error("native particle producer has incomplete output/counters");
    builds.erase(it);
}
std::optional<bool> nativeParticleInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool sphere=std::find(std::begin(particleSphereSites),std::end(particleSphereSites),pc)!=std::end(particleSphereSites);
    if(!sphere && pc!=0x22eae0 && pc!=0x22ec9c && pc!=0x22eca0 && pc!=0x22ee88 && pc!=0x22ee94 && pc!=0x22ee9c && pc!=0x22eea4 && pc!=0x22eebc && pc!=0x22f0a0)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;auto *owner=d?dynamic_cast<ParticleOwner *>(&d->host()):nullptr;
    auto *assets=d?dynamic_cast<AssetOwner *>(&d->host()):nullptr;
    if(!c || !owner || !assets)throw std::runtime_error("native particles require explicit publication/provenance owners");
    auto &particles=owner->nativeParticles();
    if(sphere){if(pc==0x22f018)particles.publishSphere(rt,c);}
    else if(pc==0x22eae0)particles.begin(rt,c);
    else if(pc==0x22ee88)particles.publish(rt,c,assets->nativeAssets());
    else if(pc==0x22f0a0){particles.finish(rt,c);return false;}
    else particles.retire(rt,c,pc);
    return true;
}
}

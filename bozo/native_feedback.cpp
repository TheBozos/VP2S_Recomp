#include "native_feedback.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/render/renderer.h"
#include <cstring>
#include <immintrin.h>
#include <format>
namespace vp2s {namespace {
template<class T>T value(Runtime *rt,uint32_t at){T v;std::memcpy(&v,rt->memory.hostPointer(at,sizeof(T)),sizeof(T));return v;}
void type(Runtime *rt,uint32_t object){
    if(object<0x100000 || value<uint32_t>(rt,object)!=0x152e80 || value<uint32_t>(rt,object+0x90)!=0x152efc || value<uint8_t>(rt,object+0x99)>1)
        throw std::runtime_error(std::format("unqualified native screen feedback class/mode: object={:x} class={:x} job={:x} mode={}",object,value<uint32_t>(rt,object),value<uint32_t>(rt,object+0x90),value<uint8_t>(rt,object+0x99)));
}
}
void NativeFeedbackCopies::begin(Runtime *rt,const NativeScene &scene,uint32_t stack,uint32_t object){
    type(rt,object);if(builds.contains(stack))throw std::runtime_error("screen feedback builder stack reused");
    for(const auto &[at,copy]:builds)if(copy.object==object)throw std::runtime_error("screen feedback object already building");
    const auto source=value<uint32_t>(rt,0x35d980);const auto frame=scene.frameTarget();
    if(!frame || frame->object!=source)throw std::runtime_error("screen feedback has no current native source");
    const auto width=value<int16_t>(rt,source+0x3c6c),height=value<int16_t>(rt,source+0x3c6e);
    if(width<1 || width>2048 || height<1 || height>2048)throw std::runtime_error("screen feedback dimensions outside native image range");
    NativeFeedbackCopy copy{object,source,value<uint32_t>(rt,object+0x94),uint32_t(width),uint32_t(height),frame};
    copy.mode=value<uint8_t>(rt,object+0x99);copy.sourceWidth=width;copy.sourceHeight=height;
    if(copy.mode==1){copy.width=512;copy.height=512;}
    ready.erase(object);builds.emplace(stack,copy);
}
void NativeFeedbackCopies::finish(Runtime *rt,uint32_t stack,uint32_t object){
    const auto it=builds.find(stack);type(rt,object);
    if(it==builds.end() || it->second.object!=object || it->second.writer!=value<uint32_t>(rt,object+0x94) ||
       it->second.mode!=value<uint8_t>(rt,object+0x99) || it->second.source!=value<uint32_t>(rt,0x35d980))
        throw std::runtime_error("screen feedback escaped original builder");
    if(value<uint8_t>(rt,object+0x63))ready[object]=it->second;
    builds.erase(it);
}
NativeFeedbackCopy NativeFeedbackCopies::submit(Runtime *rt,const NativeScene &scene,uint32_t object){
    type(rt,object);const auto it=ready.find(object);const auto current=value<uint32_t>(rt,0x35ddf8);const auto layer=value<uint8_t>(rt,object+0x65);
    if(it==ready.end() || it->second.frame!=scene.frameTarget() || layer>1 || !value<uint8_t>(rt,current+0x40dc+layer) ||
       value<uint32_t>(rt,current+0x40a8+4*layer)!=object || value<uint32_t>(rt,object+0x10)!=current ||
       it->second.source!=value<uint32_t>(rt,0x35d980) || it->second.writer!=value<uint32_t>(rt,object+0x94) ||
       it->second.mode!=value<uint8_t>(rt,object+0x99) || it->second.sourceWidth!=uint32_t(value<int16_t>(rt,it->second.source+0x3c6c)) ||
       it->second.sourceHeight!=uint32_t(value<int16_t>(rt,it->second.source+0x3c6e)))
        throw std::runtime_error("screen feedback submitted outside original owner/frame");
    auto copy=it->second;ready.erase(it);return copy;
}
void NativeFeedbackCopies::release(uint32_t object){
    for(const auto &[stack,copy]:builds)if(copy.object==object)throw std::runtime_error("screen feedback destroyed during builder");
    ready.erase(object);
    if(lastPublished && lastPublished->object==object)lastPublished.reset();
}
void drawFeedbackCopy(const NativeFeedbackCopy &copy){
    const auto fp=_mm_getcsr();struct Restore{unsigned fp;~Restore(){_mm_setcsr(fp);}} restore{fp};
    if(!copy.frame || !copy.width || !copy.height || copy.width>2048 || copy.height>2048 || copy.mode>1 ||
       (copy.mode && (!copy.sourceWidth || !copy.sourceHeight || copy.sourceWidth>2048 || copy.sourceHeight>2048)))
        throw std::runtime_error("invalid native screen feedback image");
    auto &renderer=bozo::render::Renderer::instance();bozo::render::Material material;
    material.textured=false;material.depthTest=false;material.depthWrite=false;material.blend=bozo::render::Material::Blend::Opaque;
    material.fromFrame=1;material.target=5;material.targetWidth=uint16_t(copy.width);material.targetHeight=uint16_t(copy.height);
    material.nearest=true;material.clampU=material.clampV=true;
    const float id[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},xy[6][2]={{0,0},{1,0},{0,1},{1,0},{1,1},{0,1}};
    bozo::render::Vertex vertices[6];for(unsigned i=0;i<6;++i){auto &v=vertices[i];v.x=2048-copy.frame->width/2+xy[i][0]*copy.frame->width;
        v.y=2048-copy.frame->height/2+xy[i][1]*copy.frame->height;v.u=xy[i][0];v.v=xy[i][1];v.rgba=0x80808080;}
    // Filter the independent snapshot, avoiding the main sampler's extra mip
    // bias. Mode zero copies with nearest sampling at the source dimensions.
    if(copy.mode){
        material.target=6;material.targetWidth=copy.sourceWidth;material.targetHeight=copy.sourceHeight;
        renderer.draw(material,id,vertices,6);
        material.target=5;material.targetWidth=copy.width;material.targetHeight=copy.height;material.source=6;material.nearest=false;
    }
    renderer.draw(material,id,vertices,6);
    if(copy.mode){
        material.fromFrame=0;material.source=0;material.nearest=true;
        const auto black=[&](float x0,float y0,float x1,float y1){
            bozo::render::Vertex border[6];for(unsigned i=0;i<6;++i){auto &v=border[i];
                v.x=2048-copy.frame->width/2+(x0+(x1-x0)*xy[i][0])*copy.frame->width;
                v.y=2048-copy.frame->height/2+(y0+(y1-y0)*xy[i][1])*copy.frame->height;v.rgba=0x80000000;}
            renderer.draw(material,id,border,6);
        };
        // Original mode 1 closes a black line around (-1,-1)..(w-1,h-1).
        black(1-1.f/copy.width,0,1,1);black(0,1-1.f/copy.height,1,1);
    }
}
std::optional<bool> nativeFeedbackInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x205ab0 && pc!=0x206808 && pc!=0x205aa0 && pc!=0x206830)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<FeedbackOwner *>(&d->host()):nullptr;auto *scene=d?dynamic_cast<SceneOwner *>(&d->host()):nullptr;
    if(!c || !owner || !scene)throw std::runtime_error("screen feedback requires native scene/image owner");
    auto &copies=owner->nativeFeedbackCopies();const auto stack=c->r[29].uw[0];
    if(pc==0x206830)copies.release(c->r[4].uw[0]);
    else if(pc==0x205ab0){if(c->pc!=pc)throw std::runtime_error("screen feedback requires complete builder");copies.begin(rt,scene->nativeScene(),stack-0x80,c->r[4].uw[0]);}
    else if(pc==0x206808)copies.finish(rt,stack,c->r[19].uw[0]);
    else{
        if(c->pc!=0x205a80 || c->r[5].uw[0]<0x100090 || c->r[4].uw[0]!=c->r[5].uw[0]-0x90)
            throw std::runtime_error("screen feedback requires original pre-enqueue producer");
        const auto object=c->r[4].uw[0];const auto copy=copies.submit(rt,scene->nativeScene(),object);
        owner->drawNativeFeedbackCopy(copy);copies.published(copy);c->r[4].sd[0]=int32_t(object);return true;
    }
    return false;
}
}

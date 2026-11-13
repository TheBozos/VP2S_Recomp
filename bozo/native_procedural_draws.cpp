#include "native_procedural_draws.h"
#include "bozo/ee/dispatcher.h"
#include "../port/vp2s_music_math.h"
#include <cmath>
#include <cstring>
#include <format>
#include <string>
namespace vp2s {namespace {
template<class T>T value(Runtime *rt,uint32_t at){
    const bool scratch=at>=0x70000000 && uint64_t(at)+sizeof(T)<=0x70004000;
    const auto physical=scratch?at:NativeAssets::address(at);
    if(!rt || (!scratch && (physical<0x100000 || uint64_t(physical)+sizeof(T)>0x2000000)))throw std::runtime_error("procedural draw input outside owned memory");
    T v;auto *p=rt->memory.hostPointer(at,sizeof(T));if(!p)throw std::runtime_error("procedural draw missing memory");std::memcpy(&v,p,sizeof(v));return v;
}
uint32_t key(uint32_t at){return at>=0x70000000 && at<0x70004000?at:NativeAssets::address(at);}
}
void NativeProceduralDraws::beginMesh(Runtime *rt,const R5900Context *c,const NativeScene &scene,const NativeProceduralTransforms &projections){
    if(!c || c->pc!=0x247700 || c->r[31].uw[0]!=0x247700)throw std::runtime_error("procedural mesh lacks scalar builder");
    music::CodecRounding arithmetic;const auto frame=key(c->r[29].uw[0]+0xb0),object=c->r[21].uw[0];const auto &projection=projections.projection(frame,object);
    const auto camera=scene.drawingCamera();const auto target=scene.frameTarget();
    if(!target || !camera || camera->object!=projection.camera || value<uint32_t>(rt,object)!=0x152d50 || ready.contains(object))throw std::runtime_error("unqualified procedural mesh class/frame");
    const auto manager=value<uint32_t>(rt,0x35de38);const auto slot=value<int16_t>(rt,manager+0x2670);
    if(slot<0 || slot>=256 || value<uint32_t>(rt,manager+0x1008+16*slot)!=0x4674404c || value<uint32_t>(rt,manager+8+16*slot)!=1)throw std::runtime_error("procedural mesh has no screen feedback texture key");
    const auto count=c->r[20].uw[0],fan=c->r[22].uw[0],blend=c->r[16].uw[0];const auto always=value<uint8_t>(rt,object+0x244),noWrite=value<uint8_t>(rt,object+0x245);
    if(count>65536 || fan>1 || blend>1 || always>1 || noWrite>1)throw std::runtime_error("unported procedural primitive variant");
    auto found=active.find(frame);if(found!=active.end() && (found->second.object!=object || found->second.frame!=target || found->second.camera!=camera || found->second.mesh))throw std::runtime_error("procedural mesh overlaps active owner");
    Mesh mesh;mesh.count=count;mesh.fan=fan;auto &m=mesh.batch.material;m.fromFrame=1;m.source=5;m.textured=false;m.textureAlpha=false;m.textureFunction=bozo::render::Material::TextureFunction::Modulate;
    m.alphaBlend=blend;m.blend=blend?bozo::render::Material::Blend::Alpha:bozo::render::Material::Blend::Opaque;m.depthTest=!always;m.depthWrite=!noWrite;m.alphaTest=false;
    const float width=2*(camera->centre[0]-camera->viewport[0]),height=2*(camera->centre[1]-camera->viewport[1]);
    if(!std::isfinite(width) || !std::isfinite(height) || width<=16 || height<=16 || width>=4096 || height>=4096)throw std::runtime_error("procedural mesh viewport invalid");
    mesh.batch.matrix={target->width/width,0,0,0,0,target->height/height,0,0,0,0,1,0,(1-target->width/width)*camera->centre[0],(1-target->height/height)*camera->centre[1],0,1};
    if(found==active.end())found=active.emplace(frame,Draw{object,target,camera}).first;found->second.mesh=std::move(mesh);
}
void NativeProceduralDraws::vertex(Runtime *rt,const R5900Context *c){
    if(!c || (c->pc!=0x247700 && c->pc!=0x2478c0 && c->pc!=0x247974) || c->r[31].uw[0]!=0x247700)throw std::runtime_error("procedural vertex escaped scalar builder");
    music::CodecRounding arithmetic;const auto frame=key(c->r[29].uw[0]+0xb0);auto found=active.find(frame);
    if(found==active.end() || found->second.object!=c->r[21].uw[0] || !found->second.mesh)throw std::runtime_error("procedural vertex lacks mesh owner");auto &mesh=*found->second.mesh;
    const auto index=c->r[5].uw[0],stack=c->r[29].uw[0];if(index!=mesh.points.size() || index>=mesh.count || c->r[20].uw[0]!=mesh.count)throw std::runtime_error("procedural vertex order/count mismatch");
    const auto position=value<std::array<int32_t,4>>(rt,stack+0x80);const auto uv=value<std::array<float,2>>(rt,stack+0xa8);
    // 2478c0..2479c0: UV is affine in the projected first position (scale +24c/+250
    // and offsets). A point projected far off screen or near the camera carries an
    // arbitrarily large position and UV (the town's sheet: x 251198, uv 2200/-5682);
    // the GS sees it only through a drawn triangle, qualified below.
    const auto detail=[&](const char *what){
        // The point and the mesh's earlier ones (GS pixels, raw ADC word, UV as stored).
        std::string out=std::format("{} [object {:x} point {}/{} at {},{} adc {:x} uv {},{}; earlier:",what,c->r[21].uw[0],index,mesh.count,
            position[0]/16.f,position[1]/16.f,uint32_t(position[3]),uv[0],uv[1]);
        for(size_t i=mesh.points.size()>8?mesh.points.size()-8:0;i<mesh.points.size();++i)
            out+=std::format(" ({},{} uv {},{})",mesh.points[i].x,mesh.points[i].y,mesh.points[i].v,mesh.points[i].u);
        return out+"]";};
    if(!std::isfinite(uv[0]) || !std::isfinite(uv[1]))throw std::runtime_error(detail("unqualified procedural texture coordinates"));
    const auto colour64=uint64_t(c->r[17].uw[0])+4ull*index;if(colour64>UINT32_MAX)throw std::runtime_error("procedural colour array overflow");
    bozo::render::Vertex v;v.x=position[0]/16.f;v.y=position[1]/16.f;v.z=float(uint32_t(position[2]));v.u=uv[1];v.v=uv[0];v.rgba=value<uint32_t>(rt,uint32_t(colour64));
    mesh.points.push_back(v);if(index>=2 && !(uint32_t(position[3])&0x8000)){
        const unsigned first=mesh.fan?0:index-2,second=index-1;const unsigned order[]={mesh.fan || !(index&1)?first:second,mesh.fan || !(index&1)?second:first,index};
        // A drawn triangle's points must be GS-representable (XYZ2 12.4, 0..4096 px):
        // the GS wraps larger coordinates, which a native draw would not.
        for(unsigned i:order)if(!(mesh.points[i].x>=0 && mesh.points[i].x<4096 && mesh.points[i].y>=0 && mesh.points[i].y<4096))
            throw std::runtime_error(detail("procedural triangle outside the GS coordinate range"));
        for(unsigned i:order)mesh.batch.vertices.push_back(mesh.points[i]);}
}
void NativeProceduralDraws::endMesh(const R5900Context *c){
    if(!c || (c->pc!=0x247ac0 && c->pc!=0x247700) || c->r[31].uw[0]!=0x247ac0)throw std::runtime_error("procedural mesh finish escaped scalar commit");
    auto found=active.find(key(c->r[29].uw[0]+0xb0));if(found==active.end() || found->second.object!=c->r[21].uw[0] || !found->second.mesh || found->second.mesh->points.size()!=found->second.mesh->count)throw std::runtime_error("procedural mesh finished incomplete");
    auto &draw=found->second;if(!draw.mesh->batch.vertices.empty())draw.batches.push_back(std::move(draw.mesh->batch));draw.mesh.reset();
}
void NativeProceduralDraws::finish(const R5900Context *c,const NativeProceduralTransforms &projections){
    if(!c || c->pc!=0x247ec0 || (c->r[31].uw[0]!=0x202960 && c->r[31].uw[0]!=0x2038d0 && c->r[31].uw[0]!=0x2053f8))throw std::runtime_error("procedural draw finish has unknown scalar caller");
    const auto frame=key(c->r[29].uw[0]),object=c->r[4].uw[0];projections.projection(frame,object);auto found=active.find(frame);
    if(found==active.end() || found->second.object!=object || found->second.mesh || projections.pendingPoints() || ready.contains(object))throw std::runtime_error("procedural draw finished incomplete or overlapping");
    ready.emplace(object,std::move(found->second));active.erase(found);
}
NativeModelSubmission NativeProceduralDraws::submission(Runtime *rt,const NativeScene &scene,const NativeFeedbackCopies &copies,uint32_t object)const{
    const auto found=ready.find(object);const auto image=copies.image(scene);const auto current=value<uint32_t>(rt,0x35ddf8);const auto layer=value<uint8_t>(rt,object+0x65);
    if(found==ready.end() || !image || found->second.frame!=scene.frameTarget() || layer>1 || !value<uint8_t>(rt,current+0x40dc+layer) || value<uint32_t>(rt,current+0x40a8+4*layer)!=object || !value<uint8_t>(rt,object+0x63) || value<uint32_t>(rt,object)!=0x152d50)throw std::runtime_error("procedural submission has no current scene/feedback image");
    return {object,found->second.camera->object,found->second.batches};
}
void NativeProceduralDraws::consumed(uint32_t object){if(!ready.erase(object))throw std::runtime_error("procedural draw consumed twice");}
std::optional<bool> nativeProceduralDrawInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x247700 && pc!=0x247a90 && pc!=0x247ac0 && pc!=0x247ec0 && pc!=0x24769c)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;auto *owner=d?dynamic_cast<ProceduralDrawOwner *>(&d->host()):nullptr;
    if(pc==0x247ec0 && !owner)return std::nullopt;
    auto *projections=d?dynamic_cast<ProceduralOwner *>(&d->host()):nullptr;auto *scene=d?dynamic_cast<SceneOwner *>(&d->host()):nullptr;
    if(!c || !owner || !projections || !scene)throw std::runtime_error("procedural draw lacks native owners");auto &draws=owner->nativeProceduralDraws();
    if(pc==0x247700)draws.beginMesh(rt,c,scene->nativeScene(),projections->nativeProceduralTransforms());
    else if(pc==0x247a90)draws.vertex(rt,c);
    else if(pc==0x247ac0)draws.endMesh(c);
    else if(pc==0x247ec0){draws.finish(c,projections->nativeProceduralTransforms());return std::nullopt;}
    else{
        auto *models=dynamic_cast<ModelOwner *>(&d->host());auto *feedback=dynamic_cast<FeedbackOwner *>(&d->host());
        if(!models || !feedback || (c->pc!=0x247650 && c->pc!=0x247668 && c->pc!=0x247690) || c->r[31].uw[0]!=0x2476a0 || c->r[16].uw[0]<0x100210)throw std::runtime_error("procedural draw escaped original pre-enqueue");
        const auto object=c->r[16].uw[0]-0x210;const auto batch=draws.submission(rt,scene->nativeScene(),feedback->nativeFeedbackCopies(),object);models->drawNativeModel(batch);draws.consumed(object);c->r[4].sd[0]=int32_t(object);
    }
    return false; // Original scalar instruction and real completion remain compiled.
}
}

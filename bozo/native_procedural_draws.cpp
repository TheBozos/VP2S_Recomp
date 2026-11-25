#include "native_cast.h"
#include "native_conditional_sites.h"
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
// Procedural mesh classes on the shared builder 2476b0 / 247700, finish 247ec0 and pre-enqueue 247650: 152d50 (its
// mesh method +60 is 203ca0) and 152ca0 (battle 0x1e; +60 is 202b40, another geometry generator, and its own
// destructor). Both vtables are otherwise the same; both write their ALPHA_1 (+294) with 1207d0 first.
// Class 152b40 (its mesh method +60 2022a0: header 247f90, builder 247af0, finish 247ec0 from 202960) draws with
// the second builder: PRIM strip / fan with TME (ABE by s0), one RGBAQ (s1, Q 1), then ST and XYZF2 per point.
bool proceduralMeshClass(uint32_t vtable){return vtable==0x152d50 || vtable==0x152ca0 || vtable==0x152b40;}
// Builder sites: the return from 120fa0 (mesh start), the point's stored XYZF2 (its LQ of SP+80), the return from
// 120f70 (mesh end), the loop head and the block after VCALLMS 958; the register holding the point index.
struct BuilderSites{uint32_t begin,vertex,end,loop,call;unsigned index;bool constantColour;};
constexpr BuilderSites kBuilders[2]={{0x247700,0x247a90,0x247ac0,0x2478c0,0x247974,5,false},{0x247b40,0x247e60,0x247e90,0x247d08,0x247d44,6,true}};
const BuilderSites &builderAt(uint32_t pc){for(const auto &b:kBuilders)if(pc==b.begin || pc==b.vertex || pc==b.end)return b;
    throw std::runtime_error("procedural draw site of no builder");}
}
void NativeProceduralDraws::beginMesh(Runtime *rt,const R5900Context *c,const NativeScene &scene,const NativeProceduralTransforms &projections,uint32_t pc){
    const auto &b=builderAt(pc);
    if(!c || c->pc!=b.begin || c->r[31].uw[0]!=b.begin)throw std::runtime_error("procedural mesh lacks scalar builder");
    music::CodecRounding arithmetic;const auto frame=key(c->r[29].uw[0]+0xb0),object=c->r[21].uw[0];const auto &projection=projections.projection(frame,object);
    const auto camera=scene.drawingCamera();const auto target=scene.frameTarget();
    if(!target || !camera || camera->object!=projection.camera || !proceduralMeshClass(value<uint32_t>(rt,object)) || ready.contains(object))throw std::runtime_error("unqualified procedural mesh class/frame");
    const auto manager=value<uint32_t>(rt,0x35de38);const auto slot=value<int16_t>(rt,manager+0x2670);
    // The builder's header 247f90 writes TEX0_1 = *(35d9c0), the screen copy (PSMCT16 512 x 512, RGB, modulate;
    // TBP 3180 or 2ec0 by scene), then TEXA, PABE and the object's ALPHA_1. With class 152d50 the selected manager
    // texture is the feedback key as well; class 152ca0 (battle 0x1e) leaves another texture selected, which the
    // draw never reads.
    const auto tex0=value<uint64_t>(rt,0x35d9c0);
    const bool feedbackKey=slot>=0 && slot<256 && value<uint32_t>(rt,manager+0x1008+16*slot)==0x4674404c && value<uint32_t>(rt,manager+8+16*slot)==1;
    const bool screenCopy=((tex0>>20)&0x3f)==2 && ((tex0>>26)&15)==9 && ((tex0>>30)&15)==9 && !((tex0>>34)&1) && !((tex0>>35)&3);
    if(!feedbackKey && !screenCopy)throw std::runtime_error(std::format("procedural mesh has no screen feedback texture (TEX0 {:x})",tex0));
    const auto count=c->r[20].uw[0],fan=c->r[22].uw[0],blend=c->r[16].uw[0];const auto always=value<uint8_t>(rt,object+0x244),noWrite=value<uint8_t>(rt,object+0x245);
    if(count>65536 || fan>1 || blend>1 || always>1 || noWrite>1)throw std::runtime_error("unported procedural primitive variant");
    auto found=active.find(frame);if(found!=active.end() && (found->second.object!=object || found->second.frame!=target || found->second.camera!=camera || found->second.mesh))throw std::runtime_error("procedural mesh overlaps active owner");
    Mesh mesh;mesh.count=count;mesh.fan=fan;mesh.builder=&b-kBuilders;auto &m=mesh.batch.material;m.fromFrame=1;m.source=5;m.textured=false;m.textureAlpha=false;m.textureFunction=bozo::render::Material::TextureFunction::Modulate;
    m.alphaBlend=blend;m.blend=blend?bozo::render::Material::Blend::Alpha:bozo::render::Material::Blend::Opaque;m.depthTest=!always;m.depthWrite=!noWrite;m.alphaTest=false;
    const float width=2*(2048.f-camera->viewport[0]),height=2*(2048.f-camera->viewport[1]);
    if(!std::isfinite(width) || !std::isfinite(height) || width<=16 || height<=16 || width>=4096 || height>=4096)throw std::runtime_error("procedural mesh viewport invalid");
    mesh.batch.matrix={target->width/width,0,0,0,0,target->height/height,0,0,0,0,1,0,(1-target->width/width)*2048.f,(1-target->height/height)*2048.f,0,1};
    if(found==active.end())found=active.emplace(frame,Draw{object,target,camera}).first;found->second.mesh=std::move(mesh);
}
void NativeProceduralDraws::vertex(Runtime *rt,const R5900Context *c,uint32_t pc){
    const auto &b=builderAt(pc);
    if(!c || (c->pc!=b.begin && c->pc!=b.loop && c->pc!=b.call && c->pc!=b.call+4) || c->r[31].uw[0]!=b.begin)throw std::runtime_error("procedural vertex escaped scalar builder");
    music::CodecRounding arithmetic;const auto frame=key(c->r[29].uw[0]+0xb0);auto found=active.find(frame);
    if(found==active.end() || found->second.object!=c->r[21].uw[0] || !found->second.mesh || found->second.mesh->builder!=unsigned(&b-kBuilders))
        throw std::runtime_error("procedural vertex lacks mesh owner");auto &mesh=*found->second.mesh;
    const auto index=c->r[b.index].uw[0],stack=c->r[29].uw[0];if(index!=mesh.points.size() || index>=mesh.count || c->r[20].uw[0]!=mesh.count)throw std::runtime_error("procedural vertex order/count mismatch");
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
    // Builder 1 reads a colour per point (s1 + 4 index); builder 2 wrote one RGBAQ, s1 itself, before its points.
    const auto colour64=uint64_t(c->r[17].uw[0])+4ull*index;if(!b.constantColour && colour64>UINT32_MAX)throw std::runtime_error("procedural colour array overflow");
    bozo::render::Vertex v;v.x=position[0]/16.f;v.y=position[1]/16.f;v.z=float(uint32_t(position[2]));v.u=uv[1];v.v=uv[0];
    v.rgba=b.constantColour?c->r[17].uw[0]:value<uint32_t>(rt,uint32_t(colour64));
    mesh.points.push_back(v);if(index>=2 && !(uint32_t(position[3])&0x8000)){
        const unsigned first=mesh.fan?0:index-2,second=index-1;const unsigned order[]={mesh.fan || !(index&1)?first:second,mesh.fan || !(index&1)?second:first,index};
        // A drawn triangle's points must be GS-representable (XYZ2 12.4, 0..4096 px):
        // the GS wraps larger coordinates, which a native draw would not.
        for(unsigned i:order)if(!(mesh.points[i].x>=0 && mesh.points[i].x<4096 && mesh.points[i].y>=0 && mesh.points[i].y<4096))
            throw std::runtime_error(detail("procedural triangle outside the GS coordinate range"));
        for(unsigned i:order)mesh.batch.vertices.push_back(mesh.points[i]);}
}
void NativeProceduralDraws::endMesh(const R5900Context *c,uint32_t pc){
    const auto &b=builderAt(pc);
    if(!c || (c->pc!=b.end && c->pc!=b.begin) || c->r[31].uw[0]!=b.end)throw std::runtime_error("procedural mesh finish escaped scalar commit");
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
    if(found==ready.end() || found->second.frame!=scene.frameTarget() || layer>1 || !value<uint8_t>(rt,current+0x40dc+layer) || value<uint32_t>(rt,current+0x40a8+4*layer)!=object || !value<uint8_t>(rt,object+0x63) || !proceduralMeshClass(value<uint32_t>(rt,object)))
        throw std::runtime_error(std::format("procedural submission has no current scene/feedback image (object {:x}: ready {}, image {}, frame {}, layer {}, scene layer {}, current {:x})",
            object,found!=ready.end(),bool(image),found!=ready.end() && found->second.frame==scene.frameTarget(),layer,layer<=1?value<uint8_t>(rt,current+0x40dc+layer):0,
            layer<=1?value<uint32_t>(rt,current+0x40a8+4*layer):0));
    // Battles 0x1e, 0x24, 0x29, 0x37 and 0x3e build the mesh over the screen copy's TEX0 (*(35d9c0)) with no copy
    // published in that frame. The user's rule (2026-11-24, "A: draw from previous frame"): sample the previous
    // presentation, with the same coordinates as a whole-frame copy.
    NativeModelSubmission out{object,found->second.camera->object,found->second.batches};
    if(!image)for(auto &batch:out.batches)batch.material.source=bozo::render::Material::kPreviousFrame;
    return out;
}
void NativeProceduralDraws::consumed(uint32_t object){if(!ready.erase(object))throw std::runtime_error("procedural draw consumed twice");}
std::optional<bool> nativeProceduralDrawInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(!nativeProceduralDrawSite(pc))return std::nullopt;   // its sites: native_conditional_sites.h
    if(pc!=0x247700 && pc!=0x247a90 && pc!=0x247ac0 && pc!=0x247b40 && pc!=0x247e60 && pc!=0x247e90 && pc!=0x247ec0 && pc!=0x24769c)return std::nullopt;
    auto *d=rt?vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services):nullptr;auto *owner=d?vp2s::nativeCast<ProceduralDrawOwner>(&d->host()):nullptr;
    if(pc==0x247ec0 && !owner)return std::nullopt;
    auto *projections=d?vp2s::nativeCast<ProceduralOwner>(&d->host()):nullptr;auto *scene=d?vp2s::nativeCast<SceneOwner>(&d->host()):nullptr;
    if(!c || !owner || !projections || !scene)throw std::runtime_error("procedural draw lacks native owners");auto &draws=owner->nativeProceduralDraws();
    if(pc==0x247700 || pc==0x247b40)draws.beginMesh(rt,c,scene->nativeScene(),projections->nativeProceduralTransforms(),pc);
    else if(pc==0x247a90 || pc==0x247e60)draws.vertex(rt,c,pc);
    else if(pc==0x247ac0 || pc==0x247e90)draws.endMesh(c,pc);
    else if(pc==0x247ec0){draws.finish(c,projections->nativeProceduralTransforms());return std::nullopt;}
    else{
        auto *models=vp2s::nativeCast<ModelOwner>(&d->host());auto *feedback=vp2s::nativeCast<FeedbackOwner>(&d->host());
        if(!models || !feedback || (c->pc!=0x247650 && c->pc!=0x247668 && c->pc!=0x247690) || c->r[31].uw[0]!=0x2476a0 || c->r[16].uw[0]<0x100210)throw std::runtime_error("procedural draw escaped original pre-enqueue");
        const auto object=c->r[16].uw[0]-0x210;const auto batch=draws.submission(rt,scene->nativeScene(),feedback->nativeFeedbackCopies(),object);models->drawNativeModel(batch);draws.consumed(object);c->r[4].sd[0]=int32_t(object);
    }
    return false; // Original scalar instruction and real completion remain compiled.
}
}

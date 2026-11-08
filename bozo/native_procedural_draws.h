#pragma once
#include "native_procedural.h"
#include "native_models.h"
#include "native_feedback.h"
namespace vp2s {
class NativeProceduralDraws {
    struct Mesh {uint32_t count=0;bool fan=false;NativeModelBatch batch;std::vector<bozo::render::Vertex> points;};
    struct Draw {uint32_t object=0;std::shared_ptr<const NativeFrameTarget> frame;std::shared_ptr<const NativeCamera> camera;std::vector<NativeModelBatch> batches;std::optional<Mesh> mesh;};
    std::map<uint32_t,Draw> active,ready;
public:
    void beginMesh(Runtime *,const R5900Context *,const NativeScene &,const NativeProceduralTransforms &);
    void vertex(Runtime *,const R5900Context *);
    void endMesh(const R5900Context *);
    void finish(const R5900Context *,const NativeProceduralTransforms &);
    NativeModelSubmission submission(Runtime *,const NativeScene &,const NativeFeedbackCopies &,uint32_t object)const;
    void consumed(uint32_t object);
    size_t pending()const{return active.size()+ready.size();}
};
class ProceduralDrawOwner {
public:
    virtual ~ProceduralDrawOwner()=default;
    virtual NativeProceduralDraws &nativeProceduralDraws()=0;
};
std::optional<bool> nativeProceduralDrawInstruction(R5900Context *,Runtime *,uint32_t);
}

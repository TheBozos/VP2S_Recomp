#include "native_math.h"
#include "native_object_math.h"
#include "native_trig.h"
#include "native_frustum.h"
#include "bozo/ee/dispatcher.h"
#include "../port/vp2s_math.h"
#include <cstring>
#include <format>
#include <stdexcept>
namespace vp2s {
namespace {
uint8_t *record(Runtime *rt,uint32_t address,uint32_t bytes){
    const bool scratch=address>=bozo::kScratchpadBase && address<bozo::kScratchpadBase+bozo::kScratchpadBytes;
    const auto offset=address&0x1ffffffu;
    if(!rt || (address&15) || (!scratch && (offset<0x100000 || (address>>28)==7)))
        throw std::runtime_error("invalid native math record");
    auto *result=rt->memory.hostPointer(address,bytes);
    if(!result)throw std::runtime_error("native math record extends outside owned memory");
    return result;
}
float pairNormal(float v){const auto b=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((b&0x7f800000)?b:(b&0x80000000));}
float pairProduct(float a,float b){volatile float value=a*b;return pairNormal(value);}
float pairSum(float a,float b){volatile float value=a+b;return pairNormal(value);}
void pairQualified(float v){const auto b=std::bit_cast<uint32_t>(v),e=b&0x7f800000;
    if(e==0x7f800000 || (!e && (b&0x7fffff)))throw std::runtime_error("unqualified constrained-position input");}
struct PositionPairGeometry {std::array<float,3> difference;float distance;};
PositionPairGeometry positionPairGeometry(const float first[4],const float second[4]){
    for(unsigned i=0;i<3;++i){pairQualified(first[i]);pairQualified(second[i]);}
    {music::CodecRounding validation;const double limit=std::sqrt(double(std::numeric_limits<float>::max())/3);
        for(unsigned i=0;i<3;++i)if(std::abs(double(first[i]))+std::abs(double(second[i]))>limit)
            throw std::runtime_error("constrained-position distance overflow");}
    PositionPairGeometry out;float squares[3];
    for(unsigned i=0;i<3;++i){out.difference[i]=pairSum(second[i],-first[i]);squares[i]=pairProduct(out.difference[i],out.difference[i]);}
    const float squared=pairSum(pairSum(squares[0],squares[1]),pairProduct(1.f,squares[2]));
    volatile float root=static_cast<float>(std::sqrt(double(squared)));
    out.distance=pairSum(0.f,root);return out;
}
void separatePairFrame(uint8_t *frame,uint8_t *data,size_t bytes){
    const auto a=reinterpret_cast<uintptr_t>(frame),b=reinterpret_cast<uintptr_t>(data);
    if(a<b+bytes && b<a+16)throw std::runtime_error("constrained-position temporary aliases live inputs");
}
}
void NativeMotion::prepare(uint32_t frame,uint32_t object,std::array<uint32_t,4> position){
    if(!updates.emplace(frame,Update{object,position}).second)throw std::runtime_error("native motion update repeats its frame");
}
std::array<uint32_t,4> NativeMotion::finish(uint32_t frame,uint32_t object){
    auto it=updates.find(frame);if(it==updates.end() || it->second.object!=object)throw std::runtime_error("native motion publication has no matching update");
    const auto result=it->second.position;updates.erase(it);return result;
}
bool nativeMathInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc){
    if(nativeTrigInstruction(nullptr,c,rt,pc))return true;
    if(nativeObjectMathInstruction(nullptr,c,rt,pc))return true;
    if(nativeFrustumInstruction(c,rt,pc))return true;
    switch(pc){
    case 0x1fc86c:case 0x1fc870:case 0x1fc898:case 0x1fc8a0:case 0x1fc8a4:case 0x1fc8a8:case 0x1fc8ac:case 0x1fc8b4:
    case 0x1fc8f8:case 0x1fc8fc:case 0x1fc900:case 0x1fc90c:case 0x1fc948:case 0x1fc950:case 0x1fc970:case 0x1fc974:
    case 0x1fc978:case 0x1fc97c:case 0x1fc980:case 0x1fc98c:case 0x1fc9ac:case 0x1fc9b8:case 0x1fc9bc:case 0x1fc9c0:
    case 0x1fc9c4:case 0x1fc9c8:case 0x1fc9d0:return true;
    default:break;
    }
    if(pc==0x1fc908){
        const auto frame=c->r[29].uw[0];auto *stack=record(rt,frame,16);
        if(c->r[3].uw[0]!=frame)throw std::runtime_error("constrained-position distance destination mismatch");
        auto *first=record(rt,c->r[4].uw[0]+0x30,16),*second=record(rt,c->r[6].uw[0]+0x30,16),*constraint=record(rt,c->r[5].uw[0],0x30);
        separatePairFrame(stack,first,16);separatePairFrame(stack,second,16);separatePairFrame(stack,constraint,0x30);
        float a[4],b[4];std::memcpy(a,first,16);std::memcpy(b,second,16);const auto geometry=positionPairGeometry(a,b);
        // The scalar tail reads only this distance. YZW die at frame release;
        // aliases that could make those retired stores observable reject above.
        std::memcpy(stack,&geometry.distance,4);return true;
    }
    if(pc==0x1fc984 || pc==0x1fc9cc){
        auto *first=record(rt,c->r[5].uw[0],16),*second=record(rt,c->r[4].uw[0],16),*frame=record(rt,c->r[29].uw[0],16);
        if(c->r[4].uw[0]!=c->r[6].uw[0]+0x30)throw std::runtime_error("constrained-position output record mismatch");
        separatePairFrame(frame,first,16);separatePairFrame(frame,second,16);
        float a[4],b[4];std::memcpy(a,first,16);std::memcpy(b,second,16);const auto geometry=positionPairGeometry(a,b);
        const float firstWeight=pc==0x1fc984?c->f[0]:c->f[1],secondWeight=c->f[0];pairQualified(firstWeight);pairQualified(secondWeight);
        volatile float inverse=geometry.distance==0?std::numeric_limits<float>::max():1.f/geometry.distance;
        float direction[3];for(unsigned i=0;i<3;++i)direction[i]=pairProduct(geometry.difference[i],pairNormal(inverse));
        {music::CodecRounding validation;const double limit=std::numeric_limits<float>::max();
            for(unsigned i=0;i<3;++i)if(std::abs(double(a[i]))+std::abs(double(direction[i])*firstWeight)>limit ||
                std::abs(double(b[i]))+std::abs(double(direction[i])*secondWeight)>limit)throw std::runtime_error("constrained-position correction overflow");}
        std::array<uint32_t,4> outA,outB;std::memcpy(outA.data(),a,16);std::memcpy(outB.data(),b,16);
        for(unsigned i=0;i<3;++i){outA[i]=std::bit_cast<uint32_t>(pairSum(pairProduct(a[i],1.f),pairProduct(direction[i],firstWeight)));
            outB[i]=std::bit_cast<uint32_t>(pairSum(pairProduct(b[i],1.f),-pairProduct(direction[i],secondWeight)));}
        // Both original stores precede the next dispatch checkpoint, including
        // the symmetric branch's delay slot. Capture both before either write.
        std::memcpy(first,outA.data(),16);std::memcpy(second,outB.data(),16);return true;
    }
    if(pc==0x1fca90 || pc==0x1fca9c || pc==0x1fcaa0 || pc==0x1fcaa4 || pc==0x1fcaa8 || pc==0x1fcaac ||
       pc==0x1fcac4 || pc==0x1fcad8 || pc==0x1fcadc || pc==0x1fcafc || pc==0x1fcb00 || pc==0x1fcb04)return true;
    if(pc==0x1fcb08 || pc==0x1fcb24){
        auto *dispatcher=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
        auto *owner=dispatcher?dynamic_cast<MotionOwner *>(&dispatcher->host()):nullptr;
        if(!owner)throw std::runtime_error("native motion requires an update owner");
        const auto object=c->r[16].uw[0],frame=c->r[29].uw[0];auto *source=record(rt,object,0x70);
        if(pc==0x1fcb24){
            if(c->r[3].uw[0]!=object+0x30)throw std::runtime_error("native motion publication destination mismatch");
            const auto result=owner->nativeMotion().finish(frame,object);std::memcpy(source+0x30,result.data(),16);return true;
        }
        record(rt,frame,0x30);
        if(c->r[6].uw[0]!=object+0x30 || c->r[3].uw[0]!=object+0x50 || c->r[4].uw[0]!=0x359100)
            throw std::runtime_error("native motion input record mismatch");
        uint32_t targetObject,table,callback;std::memcpy(&targetObject,source+4,4);
        std::memcpy(&table,record(rt,targetObject,0x60),4);std::memcpy(&callback,record(rt,table+0x30,16)+4,4);
        if(!owner->qualifiedMotionCallback(rt,callback))throw std::runtime_error("unqualified native motion callback");
        const auto flags=source[1];float position[4],velocity[4],acceleration[4]{},gravity[4]{};
        std::array<uint32_t,4> out;std::memcpy(out.data(),source+0x30,16);std::memcpy(position,out.data(),16);std::memcpy(velocity,source+0x50,16);
        if(flags&1)std::memcpy(gravity,record(rt,0x359100,16),16);
        if(flags&0x40)std::memcpy(acceleration,record(rt,c->r[5].uw[0],16),16);
        const float dt=c->f[12],damping=flags&2?c->f[0]:0.f;
        const auto qualified=[](float v){const auto b=std::bit_cast<uint32_t>(v),e=b&0x7f800000;
            if(e==0x7f800000 || (!e && (b&0x7fffff)))throw std::runtime_error("unqualified native motion input");};
        qualified(dt);if(flags&2)qualified(damping);if(flags&1)qualified(gravity[1]);
        for(unsigned i=0;i<3;++i){qualified(position[i]);qualified(velocity[i]);if(flags&0x40)qualified(acceleration[i]);}
        {music::CodecRounding validation;const double limit=std::numeric_limits<float>::max(),t=std::abs(double(dt)),d=std::abs(double(damping));
            if(t*t>limit || d*t>limit)throw std::runtime_error("native motion coefficient overflow");
            for(unsigned i=0;i<3;++i){double bound=std::abs(double(position[i]))+std::abs(double(velocity[i]))*t;
                if((flags&1) && i==1)bound+=std::abs(double(gravity[1]))*t*t;
                if(flags&0x40)bound+=std::abs(double(acceleration[i]))*t;
                if(flags&2)bound+=std::abs(double(velocity[i]))*d*t;
                if(bound>limit)throw std::runtime_error("native motion position overflow");}}
        const auto normal=[](float v){const auto b=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((b&0x7f800000)?b:(b&0x80000000));};
        const auto product=[&](float a,float b){volatile float v=a*b;return normal(v);};
        const auto sum=[&](float a,float b){volatile float v=a+b;return normal(v);};
        const float squared=product(dt,dt),drag=flags&2?product(damping,dt):0.f;
        for(unsigned i=0;i<3;++i){float value=sum(position[i],0.f);
            if((flags&1) && i==1)value=sum(value,product(gravity[1],squared));
            if(flags&0x40)value=sum(value,product(acceleration[i],dt));
            if(flags&2)value=sum(value,-product(velocity[i],drag));
            out[i]=std::bit_cast<uint32_t>(sum(value,product(velocity[i],dt)));}
        owner->nativeMotion().prepare(frame,object,out);return true;
    }
    if((pc>=0x1fb1e4 && pc<0x1fb200 && !(pc&3)) ||
       (pc>=0x1fb2fc && pc<0x1fb318 && !(pc&3)) ||
       (pc>=0x1fb42c && pc<0x1fb448 && !(pc&3)) ||
       (pc>=0x1fb624 && pc<0x1fb640 && !(pc&3)))return true;
    if(pc==0x1fb200 || pc==0x1fb318 || pc==0x1fb448 || pc==0x1fb640){
        math::RigidBasis basis;__m128 vector;
        std::memcpy(basis.data(),record(rt,c->r[5].uw[0],48),48);std::memcpy(&vector,record(rt,c->r[4].uw[0],16),16);
        float rows[3][4],point[4];std::memcpy(rows,basis.data(),48);std::memcpy(point,&vector,16);
        const auto qualified=[](float v){const auto b=std::bit_cast<uint32_t>(v),e=b&0x7f800000;
            if(e==0x7f800000 || (!e && (b&0x7fffff)))throw std::runtime_error("unqualified native linear transform input");};
        for(auto &row:rows)for(float v:row)qualified(v);for(unsigned i=0;i<3;++i)qualified(point[i]);
        {music::CodecRounding validation;
            for(unsigned lane=0;lane<4;++lane){double bound=0;for(unsigned row=0;row<3;++row)bound+=std::abs(double(rows[row][lane])*point[row]);
                if(bound>std::numeric_limits<float>::max())throw std::runtime_error("native linear transform overflow");}}
        const auto result=math::linearVectorProduct(basis,vector);
        std::memcpy(record(rt,c->r[3].uw[0],16),&result,16);return true;
    }
    if((pc>=0x1fb3f4 && pc<0x1fb418 && !(pc&3)) ||
       (pc>=0x1fb5ec && pc<0x1fb610 && !(pc&3)) ||
       (pc>=0x1fb87c && pc<0x1fb8a0 && !(pc&3)))return true;
    if(pc==0x1fb418 || pc==0x1fb610 || pc==0x1fb8a0){
        math::Matrix matrix;__m128 vector;
        std::memcpy(matrix.data(),record(rt,c->r[5].uw[0],64),64);std::memcpy(&vector,record(rt,c->r[4].uw[0],16),16);
        float rows[4][4],point[4];std::memcpy(rows,matrix.data(),64);std::memcpy(point,&vector,16);
        const auto qualified=[](float v){const auto b=std::bit_cast<uint32_t>(v),e=b&0x7f800000;
            if(e==0x7f800000 || (!e && (b&0x7fffff)))throw std::runtime_error("unqualified native full transform input");};
        for(auto &row:rows)for(float v:row)qualified(v);for(float v:point)qualified(v);
        {music::CodecRounding validation;
            for(unsigned lane=0;lane<4;++lane){double bound=0;for(unsigned row=0;row<4;++row)bound+=std::abs(double(rows[row][lane])*point[row]);
                if(bound>std::numeric_limits<float>::max())throw std::runtime_error("native full transform overflow");}}
        const auto result=math::matrixVectorProduct(matrix,vector);
        std::memcpy(record(rt,c->r[3].uw[0],16),&result,16);return true;
    }
    if((pc>=0x1fb768 && pc<=0x1fb798 && !(pc&3)) || pc==0x1fb7a0)return true;
    if(pc==0x1fb79c){
        math::Matrix matrix;math::PointDirection vectors;
        std::memcpy(matrix.data(),record(rt,c->r[6].uw[0],64),64);
        std::memcpy(&vectors[0],record(rt,c->r[5].uw[0],16),16);std::memcpy(&vectors[1],record(rt,c->r[4].uw[0],16),16);
        float rows[4][4],points[2][4];std::memcpy(rows,matrix.data(),64);std::memcpy(points,vectors.data(),32);
        const auto qualified=[](float v){const auto b=std::bit_cast<uint32_t>(v),e=b&0x7f800000;
            if(e==0x7f800000 || (!e && (b&0x7fffff)))throw std::runtime_error("unqualified native point/direction input");};
        for(auto &row:rows)for(float v:row)qualified(v);
        for(unsigned which=0;which<2;++which)for(unsigned i=0;i<(which?3u:4u);++i)qualified(points[which][i]);
        {music::CodecRounding validation;
            for(unsigned which=0;which<2;++which)for(unsigned lane=0;lane<4;++lane){double bound=0;
                for(unsigned row=0;row<(which?3u:4u);++row)bound+=std::abs(double(rows[row][lane])*points[which][row]);
                if(bound>std::numeric_limits<float>::max())throw std::runtime_error("native point/direction overflow");}}
        const auto result=math::transformPointDirection(matrix,vectors);
        std::memcpy(record(rt,c->r[5].uw[0],16),&result[0],16);
        std::memcpy(record(rt,c->r[4].uw[0],16),&result[1],16);return true;
    }
    if(pc>=0x216d34 && pc<=0x216d54 && !(pc&3))return true;
    if(pc==0x216d5c){
        math::RigidBasis basis;__m128 translation;
        std::memcpy(basis.data(),record(rt,c->r[5].uw[0],48),48);
        std::memcpy(&translation,record(rt,c->r[4].uw[0]+0x30,16),16);
        float rows[3][4],point[4];std::memcpy(rows,basis.data(),48);std::memcpy(point,&translation,16);
        const auto qualified=[](float v){const auto b=std::bit_cast<uint32_t>(v),e=b&0x7f800000;
            if(e==0x7f800000 || (!e && (b&0x7fffff)))throw std::runtime_error("unqualified native column inverse input");};
        for(unsigned i=0;i<3;++i){qualified(point[i]);for(unsigned row=0;row<3;++row)qualified(rows[row][i]);}
        {music::CodecRounding validation;
            for(unsigned lane=0;lane<3;++lane){double bound=0;for(unsigned row=0;row<3;++row)bound+=std::abs(double(rows[row][lane])*point[row]);
                if(bound>std::numeric_limits<float>::max())throw std::runtime_error("native column inverse translation overflow");}}
        // Scalar clear/transpose has already run, including all alias effects.
        // This shared game operation retains caller rounding and source W.
        const auto result=math::rigidInverseColumnTranslation(basis,translation);
        std::memcpy(record(rt,c->r[5].uw[0]+0x30,16),&result,16);return true;
    }
    if(pc==0x248570 || pc==0x248574 || pc==0x248578 || pc==0x248580 || pc==0x2485a4 || pc==0x2485a8 || pc==0x2485ac)return true;
    if(pc==0x2485b4){
        const auto object=c->r[5].uw[0],stack=c->r[29].uw[0];
        auto *owner=record(rt,object,0x210),*frame=record(rt,stack,0x20);
        const auto ownerAddress=reinterpret_cast<uintptr_t>(owner),frameAddress=reinterpret_cast<uintptr_t>(frame);
        if(c->r[2].uw[0]!=stack+0x10 || (ownerAddress<frameAddress+0x20 && frameAddress<ownerAddress+0x210))
            throw std::runtime_error("unqualified native inverse-selector record alias");
        float basis[3][4],result[3];std::memcpy(basis,owner+0x90,48);
        for(auto &row:basis)for(unsigned i=0;i<3;++i){const auto b=std::bit_cast<uint32_t>(row[i]),e=b&0x7f800000;
            if(e==0x7f800000 || (!e && (b&0x7fffff)))throw std::runtime_error("unqualified native inverse-selector basis");}
        const auto normal=[](float v){const auto b=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((b&0x7f800000)?b:(b&0x80000000));};
        music::CodecRounding rounding;
        for(unsigned axis=0;axis<3;++axis){float squares[3];
            for(unsigned row=0;row<3;++row){if(double(basis[row][axis])*basis[row][axis]>std::numeric_limits<float>::max())
                    throw std::runtime_error("native inverse-selector square overflow");
                volatile float square=basis[row][axis]*basis[row][axis];squares[row]=normal(square);}
            if(double(squares[0])+squares[1]+squares[2]>std::numeric_limits<float>::max())throw std::runtime_error("native inverse-selector length overflow");
            const auto squared=normal(music::codecSum(normal(music::codecSum(squares[0],squares[1])),squares[2]));
            volatile float length=std::sqrt(squared);volatile float positive=0.f+length;
            const auto difference=normal(music::codecSum(positive,-1.f));
            result[axis]=std::bit_cast<float>(std::bit_cast<uint32_t>(difference)&0x7fffffffu);
        }
        // Only XYZ is live: the following scalar comparisons consume these
        // three words before the frame is released. Its inherited fourth lane
        // and unused control read are retired, without constructing VU state.
        std::memcpy(frame+0x10,result,12);return true;
    }
    if(pc==0x23c270 || pc==0x23c2a8 || pc==0x23c2c8){
        const auto global=c->r[18].uw[0],model=c->r[19].uw[0],destination=c->r[29].uw[0]+0x50;
        record(rt,global,0x520);record(rt,model,0x7f0);
        const bool combined=pc!=0x23c2c8;
        const auto coefficient=global+(pc==0x23c270?0x3c0:pc==0x23c2a8?0x390:0x310);
        if(combined?(c->r[6].uw[0]!=coefficient || c->r[5].uw[0]!=model+0x7e0 || c->r[3].uw[0]!=global+0x310 || c->r[4].uw[0]!=destination):
                    (c->r[5].uw[0]!=coefficient || c->r[4].uw[0]!=model+0x7e0 || c->r[3].uw[0]!=destination))
            throw std::runtime_error("native model parameter combination escaped its records");
        float first[4],second[4],offset[4]{},result[4];
        std::memcpy(first,record(rt,coefficient,16),16);std::memcpy(second,record(rt,model+0x7e0,16),16);
        if(combined)std::memcpy(offset,record(rt,global+0x310,16),16);
        for(unsigned i=0;i<4;++i){for(float v:{first[i],second[i],offset[i]}){const auto b=std::bit_cast<uint32_t>(v),e=b&0x7f800000;
                if(e==0x7f800000 || (!e && (b&0x7fffff)))throw std::runtime_error("unqualified native model parameter input");}
            if(std::abs(double(first[i])*second[i])>std::numeric_limits<float>::max())throw std::runtime_error("native model parameter product overflow");}
        const auto normal=[](float v){const auto b=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((b&0x7f800000)?b:(b&0x80000000));};
        music::CodecRounding rounding;
        for(unsigned i=0;i<4;++i){volatile float value=first[i]*second[i];result[i]=normal(value);
            if(combined){if(std::abs(double(result[i])+offset[i])>std::numeric_limits<float>::max())throw std::runtime_error("native model parameter sum overflow");
                result[i]=normal(music::codecSum(result[i],offset[i]));}}
        std::memcpy(record(rt,destination,16),result,16);return true;
    }
    switch(pc){
    case 0x23c258:case 0x23c25c:case 0x23c260:case 0x23c264:case 0x23c268:
    case 0x23c290:case 0x23c294:case 0x23c298:case 0x23c29c:case 0x23c2a0:
    case 0x23c2bc:case 0x23c2c0:case 0x23c2c4:return true;
    }
    if(pc==0x20f26c || pc==0x20f2ac){
        const auto object=c->r[21].uw[0],source=object+(pc==0x20f26c?0xd0:0x150),destination=object+0x1d0;
        record(rt,object,0x210);
        if(c->r[4].uw[0]!=source || c->r[3].uw[0]!=destination)
            throw std::runtime_error("native camera scale escaped its matrix records");
        float matrix[4][4],scale[4],result[4][4];
        std::memcpy(matrix,record(rt,source,64),64);std::memcpy(scale,record(rt,c->r[30].uw[0],16),16);
        const auto qualified=[](float v){const auto b=std::bit_cast<uint32_t>(v),e=b&0x7f800000;
            if(e==0x7f800000 || (!e && (b&0x7fffff)))throw std::runtime_error("unqualified native camera scale input");};
        for(unsigned row=0;row<4;++row){qualified(scale[row]);for(float v:matrix[row]){
            qualified(v);if(std::abs(double(v)*scale[row])>std::numeric_limits<float>::max())throw std::runtime_error("native camera scale overflow");}}
        music::CodecRounding rounding;
        for(unsigned row=0;row<4;++row)for(unsigned lane=0;lane<4;++lane){volatile float value=matrix[row][lane]*scale[row];
            const auto b=std::bit_cast<uint32_t>(float(value));result[row][lane]=std::bit_cast<float>((b&0x7f800000)?b:(b&0x80000000));}
        std::memcpy(record(rt,destination,64),result,64);return true;
    }
    switch(pc){
    case 0x20f248:case 0x20f24c:case 0x20f250:case 0x20f254:case 0x20f258:
    case 0x20f25c:case 0x20f260:case 0x20f264:case 0x20f268:
    case 0x20f270:case 0x20f274:case 0x20f27c:
    case 0x20f288:case 0x20f28c:case 0x20f290:case 0x20f294:case 0x20f298:
    case 0x20f29c:case 0x20f2a0:case 0x20f2a4:case 0x20f2a8:
    case 0x20f2b0:case 0x20f2b4:case 0x20f2b8:return true;
    }
    if(pc==0x43d2a0 || pc==0x43d2a4 || pc==0x43d2a8)return true;
    if(pc==0x43d2ac){
        const auto object=c->r[16].uw[0];
        if(c->r[6].uw[0]!=object+0xd0 || c->r[4].uw[0]!=object+0xe0 || c->r[3].uw[0]!=c->r[29].uw[0]+0x30)
            throw std::runtime_error("native model colour product escaped its CPU producer");
        float first[4],second[4],result[4];
        std::memcpy(first,record(rt,c->r[6].uw[0],16),16);std::memcpy(second,record(rt,c->r[4].uw[0],16),16);
        for(unsigned i=0;i<4;++i){for(float v:{first[i],second[i]}){const auto b=std::bit_cast<uint32_t>(v),e=b&0x7f800000;
                if(e==0x7f800000 || (!e && (b&0x7fffff)))throw std::runtime_error("unqualified native model colour input");}
            if(std::abs(double(first[i])*second[i])>std::numeric_limits<float>::max())throw std::runtime_error("native model colour product overflow");}
        music::CodecRounding rounding;
        for(unsigned i=0;i<4;++i){volatile float value=first[i]*second[i];const auto b=std::bit_cast<uint32_t>(float(value));
            result[i]=std::bit_cast<float>((b&0x7f800000)?b:(b&0x80000000));}
        std::memcpy(record(rt,c->r[3].uw[0],16),result,16);return true;
    }
    if(pc>=0x37faa0 && pc<=0x37fad4 && !(pc&3))return true;
    if(pc==0x37fad8){
        const auto box=c->r[4].uw[0];
        if(c->r[8].uw[0]!=box+0x10 || c->r[7].uw[0]!=box+0x20)
            throw std::runtime_error("native box sphere escaped its input/output records");
        float corners[8],half;std::memcpy(corners,record(rt,box,32),32);
        std::memcpy(&half,record(rt,c->r[9].uw[0],16),4);
        if(half!=0.5f)throw std::runtime_error("unqualified native box midpoint coefficient");
        for(unsigned i=0;i<8;++i)if(i%4!=3){const auto b=std::bit_cast<uint32_t>(corners[i]),e=b&0x7f800000;
            if(e==0x7f800000 || (!e && (b&0x7fffff)))throw std::runtime_error("unqualified native box corner");}
        const auto normal=[](float v){const auto b=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((b&0x7f800000)?b:(b&0x80000000));};
        float result[4],squares[3];music::CodecRounding rounding;
        for(unsigned i=0;i<3;++i){
            const double bound=std::numeric_limits<float>::max();
            if(std::abs(double(corners[i+4])+corners[i])>bound || std::abs(double(corners[i+4])-corners[i])>bound)
                throw std::runtime_error("native box corner reduction overflow");
            const auto midpoint=normal(music::codecSum(corners[i+4],corners[i]));
            volatile float center=midpoint*half;result[i]=normal(center);
            const auto opposite=std::bit_cast<float>(std::bit_cast<uint32_t>(corners[i])^0x80000000u);
            const auto span=normal(music::codecSum(corners[i+4],opposite));
            volatile float extent=span*half;const auto reduced=normal(extent);
            if(double(reduced)*reduced>bound)throw std::runtime_error("native box extent square overflow");
            volatile float square=reduced*reduced;squares[i]=normal(square);
        }
        if(double(squares[0])+squares[1]+squares[2]>std::numeric_limits<float>::max())
            throw std::runtime_error("native box squared length overflow");
        const auto squared=normal(music::codecSum(normal(music::codecSum(squares[0],squares[1])),squares[2]));
        if(!std::isfinite(squared))throw std::runtime_error("native box sphere overflow");
        volatile float root=std::sqrt(squared);
        volatile float reciprocal=root==0?std::numeric_limits<float>::max():1.f/root;
        volatile float radius=squared*reciprocal;result[3]=normal(radius);
        for(float value:result)if(!std::isfinite(value))throw std::runtime_error("native box sphere result overflow");
        std::memcpy(record(rt,c->r[7].uw[0],16),result,16);return true;
    }
    if(pc>=0x1fedd4 && pc<=0x1fedf8 && !(pc&3))return true;
    if(pc==0x1fedfc){
        float first[4],second[4],result[4];
        std::memcpy(first,record(rt,c->r[5].uw[0],16),16);
        std::memcpy(second,record(rt,c->r[2].uw[0],16),16);std::memcpy(result,first,16);
        const auto normal=[](float value){const auto b=std::bit_cast<uint32_t>(value);return std::bit_cast<float>((b&0x7f800000)?b:(b&0x80000000));};
        for(unsigned i=0;i<3;++i)for(float value:{first[i],second[i]}){const auto b=std::bit_cast<uint32_t>(value),e=b&0x7f800000;
            if(e==0x7f800000 || (!e && (b&0x7fffff)))throw std::runtime_error("unqualified native pair distance input");}
        music::CodecRounding rounding;
        for(unsigned i=0;i<3;++i){
            const auto opposite=std::bit_cast<float>(std::bit_cast<uint32_t>(second[i])^0x80000000u);
            const auto difference=normal(music::codecSum(first[i],opposite));
            volatile float square=difference*difference;result[i]=normal(square);
        }
        const auto squared=music::codecSum(music::codecSum(result[0],result[1]),result[2]);
        if(!std::isfinite(squared))throw std::runtime_error("native pair distance overflow");
        volatile float length=std::sqrt(squared);volatile float output=0.f+length;result[0]=output;
        std::memcpy(record(rt,c->r[4].uw[0],16),result,16);return true;
    }
    if(pc==0x237284 || pc==0x2372bc){
        const auto word=[&](uint32_t at){uint32_t v;std::memcpy(&v,record(rt,at&~15u,16)+(at&15),4);return v;};
        const auto owner=c->r[20].uw[0],list=word(owner+0x3c),bind=word(owner+0x40);
        const auto count=word(word(owner+0x34)+4),step=c->r[6].uw[0];
        if(!count || count>4096 || c->r[7].uw[0]!=count || !step || step>count || c->r[17].uw[0]!=list)
            throw std::runtime_error("native bone matrix list escaped its CPU producer");
        const auto index=step-1,bytes=count*64;
        const auto selector=*(record(rt,owner+0x50,16));
        const auto output=word(owner+0x4c)+0x40+selector*bytes;
        const auto destination=c->r[19].uw[0];
        if(selector>1 || (destination&0x1ffffff)!=(output+index*64 & 0x1ffffff))
            throw std::runtime_error("native bone matrix output escaped its owned buffer");
        const auto overlaps=[&](uint32_t at,uint32_t size){at&=0x1ffffff;const auto out=output&0x1ffffff;return at<out+bytes && out<at+size;};
        // The original pipelines one iteration ahead. Reject source/output
        // aliasing rather than rereading an input after a prior output store.
        const auto inverse=c->r[29].uw[0]+0x60;
        if(overlaps(bind,bytes) || overlaps(list,count*4) || overlaps(inverse,64))
            throw std::runtime_error("unqualified native bone matrix alias");
        for(unsigned i=0;i<count;++i)if(overlaps(word(list+i*4)+0x90,64))
            throw std::runtime_error("unqualified native bone source alias");
        math::Matrix base,bone,local,result;
        std::memcpy(base.data(),record(rt,inverse,64),64);
        std::memcpy(bone.data(),record(rt,word(list+index*4)+0x90,64),64);
        std::memcpy(local.data(),record(rt,bind+index*64,64),64);
        for(const auto *m:{&base,&bone,&local})for(auto row:*m){uint32_t bits[4];std::memcpy(bits,&row,16);
            for(auto b:bits){const auto e=b&0x7f800000;if(e==0x7f800000 || (!e && (b&0x7fffff)))
                throw std::runtime_error("unqualified native bone matrix input");}}
        const auto normal=[](__m128 v){const auto bits=_mm_castps_si128(v);
            const auto tiny=_mm_cmpeq_epi32(_mm_and_si128(bits,_mm_set1_epi32(0x7f800000)),_mm_setzero_si128());
            return _mm_castsi128_ps(_mm_andnot_si128(_mm_and_si128(tiny,_mm_set1_epi32(0x7fffffff)),bits));};
        const auto transform=[&](const math::Matrix &m,__m128 v){
            auto total=normal(_mm_mul_ps(m[0],music::broadcast<0>(v)));
            total=normal(music::codecSum(total,normal(_mm_mul_ps(m[1],music::broadcast<1>(v)))));
            total=normal(music::codecSum(total,normal(_mm_mul_ps(m[2],music::broadcast<2>(v)))));
            return normal(music::codecSum(total,normal(_mm_mul_ps(m[3],music::broadcast<3>(v)))));};
        music::CodecRounding rounding;
        math::Matrix relative;
        for(unsigned i=0;i<3;++i)relative[i]=transform(bone,base[i]);relative[3]=bone[3];
        for(unsigned i=0;i<3;++i)result[i]=transform(relative,local[i]);result[3]=relative[3];
        _MM_TRANSPOSE4_PS(result[0],result[1],result[2],result[3]);
        // The original transpose adds zero off the diagonal and scales each
        // translation component by one; preserve the moved diagonal payloads.
        for(unsigned i=0;i<4;++i){const auto adjusted=normal(_mm_add_ps(_mm_setzero_ps(),result[i]));
            switch(i){case 0:result[i]=_mm_blend_ps(adjusted,result[i],9);break;
                      case 1:result[i]=_mm_blend_ps(adjusted,result[i],10);break;
                      case 2:result[i]=_mm_blend_ps(adjusted,result[i],12);break;
                      case 3:result[i]=_mm_blend_ps(adjusted,result[i],8);break;}
            if(i<3)result[i]=_mm_blend_ps(result[i],normal(_mm_mul_ps(result[i],_mm_set1_ps(1))),8);
        }
        for(auto row:result)if(_mm_movemask_ps(_mm_cmpunord_ps(row,row)) ||
            _mm_movemask_ps(_mm_cmpeq_ps(_mm_andnot_ps(_mm_set1_ps(-0.f),row),_mm_set1_ps(INFINITY))))
            throw std::runtime_error("native bone matrix overflow");
        std::memcpy(record(rt,destination,64),result.data(),64);return true;
    }
    switch(pc){
    case 0x2371e8:case 0x2371ec:case 0x2371f0:case 0x2371f4:
    case 0x2371fc:case 0x237200:case 0x237204:case 0x237208:
    case 0x23720c:case 0x237210:case 0x237214:case 0x237218:case 0x237224:
    case 0x237260:case 0x237264:case 0x237268:case 0x23726c:
    case 0x237270:case 0x237274:case 0x237278:case 0x23727c:case 0x237280:
    case 0x237288:case 0x23728c:case 0x237290:case 0x23729c:
    case 0x2372b8:case 0x2372c0:case 0x2372c4:case 0x2372c8:return true;
    }
    if(pc==0x207720 || pc==0x207724 || pc==0x207728 || pc==0x20772c || pc==0x207738){
        const auto object=c->r[17].uw[0];uint32_t type,child;
        std::memcpy(&type,record(rt,object,16),4);
        std::memcpy(&child,record(rt,object+0x90,16)+8,4);
        if(type!=0x1653d0 || child!=object+0x90)
            throw std::runtime_error(std::format("unported populated native child projection: object={:x} type={:x} child={:x} head={:x}",object,type,child,object+0x90));
        // This builder's original loop executes no child and stores no numeric
        // result from the matrix setup. Retire only that dead empty-list path.
        // No inherited math state is constructed for later unported consumers.
        return true;
    }
    if(!(pc&3) && pc>=0x410df0 && pc<=0x410e44)return true;
    if(pc==0x410e4c){
        // Four row dot products. The original reduction order differs per
        // output component, including W; no inherited basis/vector state.
        float matrix[4][4],vector[4],result[4];
        std::memcpy(matrix,record(rt,c->r[4].uw[0],64),64);std::memcpy(vector,record(rt,c->r[5].uw[0],16),16);
        const auto normal=[](float value){const auto bits=std::bit_cast<uint32_t>(value);return std::bit_cast<float>((bits&0x7f800000)?bits:(bits&0x80000000));};
        const auto finite=[](float value){const auto bits=std::bit_cast<uint32_t>(value),e=bits&0x7f800000;
            if(e==0x7f800000 || (!e && (bits&0x7fffff)))throw std::runtime_error("unqualified native row dot input");};
        for(auto &row:matrix)for(float value:row)finite(value);for(float value:vector)finite(value);
        const unsigned order[4][4]={{0,1,2,3},{1,0,2,3},{2,1,0,3},{3,1,2,0}};
        music::CodecRounding rounding;
        for(unsigned row=0;row<4;++row){float products[4];
            for(unsigned i=0;i<4;++i){volatile float product=matrix[row][i]*vector[i];products[i]=normal(product);}
            auto total=normal(music::codecSum(products[order[row][0]],products[order[row][1]]));
            total=normal(music::codecSum(total,products[order[row][2]]));total=normal(music::codecSum(total,products[order[row][3]]));
            if(!std::isfinite(total))throw std::runtime_error("native row dot overflow");result[row]=total;
        }
        std::memcpy(record(rt,c->r[6].uw[0],16),result,16);return true;
    }
    if(pc==0x29e1a0 || pc==0x29e160){
        // The default math code installation has no execution or game-data
        // result. Its algorithms are compiled native record operations. Retire
        // the physical enqueue before it exists; never mark a job busy/complete.
        const uint32_t object=c->r[4].uw[0];auto *p=record(rt,object,0x54);
        uint32_t type=0;std::memcpy(&type,p,4);
        if(type!=0x1591d0 || (c->r[5].uw[0]&255)!=0 || p[0x50]!=0)
            throw std::runtime_error("unported math program selection or outstanding legacy job");
        return true;
    }
    switch(pc){
    case 0x37b548:case 0x37b54c:case 0x37b550:case 0x37b554:
    case 0x37b558:case 0x37b55c:case 0x37b560:case 0x37b564:return true;
    case 0x379234:case 0x37923c:case 0x379278:case 0x379284:case 0x379288:return true;
    case 0x379280:{
        // The scalar producer has already copied the chosen quaternion into
        // its object. Its identity fourth row stays built by the EE code.
        const auto object=c->r[19].uw[0];__m128 input;
        if(c->r[2].uw[0]!=object+0x1c0)throw std::runtime_error("native field rotation destination changed");
        std::memcpy(&input,record(rt,object+0x190,16),16);const auto matrix=math::quaternionMatrix(input);
        std::memcpy(record(rt,object+0x1c0,48),matrix.data(),48);return true;
    }
    case 0x3c5d58:case 0x3c5d5c:case 0x3c5d60:case 0x3c5d64:
    case 0x3c5d68:case 0x3c5d6c:case 0x3c5d70:case 0x3c5d74:
    case 0x378980:case 0x378984:case 0x378988:case 0x37898c:
    case 0x378990:case 0x378994:case 0x378998:case 0x37899c:
    case 0x382020:case 0x382024:case 0x382028:case 0x38202c:
    case 0x382030:case 0x382034:case 0x382038:case 0x38203c:
    case 0x37f994:case 0x37f998:case 0x37f99c:case 0x37f9a0:
    case 0x37f9a4:case 0x37f9a8:case 0x37f9ac:case 0x37f9b0:return true;
    case 0x3789a0:case 0x3c5d78:case 0x37b568:case 0x382040:case 0x37f9b4:{
        // The original scalar builder subtracts the two game positions.
        // Preserve its full temporary record and publish the consumed length.
        const auto source=c->r[pc==0x382040?4:pc==0x37f9b4?10:5].uw[0];
        const auto destination=c->r[pc==0x382040?3:pc==0x37f9b4?9:4].uw[0];
        float value[4],result[4];std::memcpy(value,record(rt,source,16),16);std::memcpy(result,value,16);
        const auto normal=[](float v){const auto bits=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((bits&0x7f800000)?bits:(bits&0x80000000));};
        for(unsigned i=0;i<3;++i){const auto bits=std::bit_cast<uint32_t>(value[i]),e=bits&0x7f800000;
            if(e==0x7f800000 || (!e && (bits&0x7fffff)))throw std::runtime_error("unqualified native field distance");}
        music::CodecRounding rounding;
        for(unsigned i=0;i<3;++i){volatile float square=value[i]*value[i];result[i]=normal(square);}
        auto squared=music::codecSum(result[0],result[1]);squared=music::codecSum(squared,result[2]);
        if(!std::isfinite(squared))throw std::runtime_error("native field distance overflow");
        volatile float length=std::sqrt(squared);volatile float output=0.f+length;result[0]=output;
        std::memcpy(record(rt,destination,16),result,16);return true;
    }
    case 0x239cfc:case 0x239d00:case 0x239d04:case 0x239d0c:
    case 0x239dc8:case 0x239dcc:case 0x239dd0:case 0x239dd4:case 0x239dd8:
    case 0x239ddc:case 0x239de0:case 0x239de4:case 0x239de8:return true;
    case 0x239d90:c->r[9].ud[0]=0;return true;
    case 0x239d98:{
        math::RigidBasis basis;std::memcpy(basis.data(),record(rt,c->r[4].uw[0]+0x90,48),48);
        const auto scale=math::largestAxisScale(basis);
        std::memcpy(record(rt,c->r[2].uw[0],16),&scale,4);return true;
    }
    case 0x239dec:{
        const auto qualified=[](__m128 value){uint32_t bits[4];std::memcpy(bits,&value,16);
            for(auto b:bits){const auto e=b&0x7f800000;
                if(e==0x7f800000 || (!e && (b&0x7fffff)))throw std::runtime_error("unqualified native sphere center");}};
        math::Matrix basis;for(unsigned i=0;i<4;++i){
            std::memcpy(&basis[i],&c->r[std::array<unsigned,4>{24,11,10,8}[i]],16);
            basis[i]=_mm_blend_ps(basis[i],_mm_setzero_ps(),8);qualified(basis[i]);
        }
        __m128 sphere;std::memcpy(&sphere,record(rt,c->r[3].uw[0],16),16);qualified(sphere);
        const auto result=_mm_blend_ps(math::matrixVectorProduct(basis,_mm_blend_ps(sphere,_mm_set1_ps(1),8)),sphere,8);
        qualified(result);
        std::memcpy(record(rt,c->r[2].uw[0],16),&result,16);return true;
    }
    case 0x20f8fc:case 0x20f900:case 0x20f904:return true;
    case 0x20f908:{
        __m128 left,right;std::memcpy(&left,record(rt,c->r[5].uw[0],16),16);std::memcpy(&right,record(rt,c->r[4].uw[0],16),16);
        auto normal=[](__m128 v){const auto bits=_mm_castps_si128(v);
            const auto tiny=_mm_cmpeq_epi32(_mm_and_si128(bits,_mm_set1_epi32(0x7f800000)),_mm_setzero_si128());
            return _mm_castsi128_ps(_mm_andnot_si128(_mm_and_si128(tiny,_mm_set1_epi32(0x7fffffff)),bits));};
        auto finite=[](__m128 v){const auto exponent=_mm_and_si128(_mm_castps_si128(v),_mm_set1_epi32(0x7f800000));
            if(_mm_movemask_epi8(_mm_cmpeq_epi32(exponent,_mm_set1_epi32(0x7f800000))))throw std::runtime_error("unqualified native light-colour product");};
        finite(left);finite(right);const auto result=normal(_mm_mul_ps(normal(left),normal(right)));finite(result);
        std::memcpy(record(rt,c->r[3].uw[0],16),&result,16);return true;
    }
    case 0x208430:case 0x208434:case 0x208438:case 0x20843c:
    case 0x208440:case 0x208444:case 0x208448:case 0x20844c:return true;
    case 0x208450:{
        float input[4],result[4];std::memcpy(input,record(rt,c->r[6].uw[0],16),16);std::memcpy(result,input,16);
        auto normal=[](float v){const auto b=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((b&0x7f800000)?b:(b&0x80000000));};
        for(unsigned i=0;i<3;++i){const auto b=std::bit_cast<uint32_t>(input[i]),exponent=b&0x7f800000;
            if(exponent==0x7f800000 || (!exponent && (b&0x7fffff)))throw std::runtime_error("unqualified native camera distance input");
            volatile float square=input[i]*input[i];result[i]=normal(square);}
        volatile float xy=result[0]+result[1];volatile float z=result[2]*1.f;
        volatile float squared=normal(xy)+normal(z);
        if(!std::isfinite(squared))throw std::runtime_error("unqualified native camera distance overflow");
        volatile float length=std::sqrt(normal(squared));volatile float value=0.f+length;result[0]=normal(value);
        std::memcpy(record(rt,c->r[5].uw[0],16),result,16);return true;
    }
    case 0x208f64:case 0x208f68:case 0x208f6c:case 0x208f70:
    case 0x208f7c:case 0x208fb0:case 0x208fbc:return true;
    case 0x209020:c->r[9].ud[0]=0;return true;
    case 0x209028:{
        math::Matrix screen;__m128 point;
        std::memcpy(screen.data(),record(rt,c->r[16].uw[0]+0x330,64),64);
        auto *destination=record(rt,c->r[2].uw[0],16);std::memcpy(&point,destination,16);
        const auto result=math::cameraProjectedPoint(screen,point);std::memcpy(destination,&result,16);return true;
    }
    case 0x2090f8:case 0x2090fc:case 0x209100:case 0x209104:
    case 0x209108:case 0x20910c:case 0x209110:case 0x209114:
    case 0x209118:case 0x20911c:case 0x209120:case 0x209124:
    case 0x209128:case 0x20912c:case 0x209130:case 0x209134:
    case 0x209138:case 0x20913c:case 0x209140:case 0x209144:
    case 0x209148:case 0x20914c:case 0x209150:case 0x209154:
    case 0x20915c:case 0x209160:case 0x209164:return true;
    case 0x209158:{
        math::Matrix local,view;
        std::memcpy(local.data(),record(rt,c->r[5].uw[0],64),64);
        std::memcpy(view.data(),record(rt,c->r[4].uw[0],64),64);
        const auto result=math::projectionProduct(local,view);
        std::memcpy(record(rt,c->r[6].uw[0],64),result.data(),64);return true;
    }
    case 0x216970:case 0x216974:case 0x216978:case 0x21697c:
    case 0x216984:case 0x216994:case 0x216998:case 0x21699c:return true;
    case 0x216988:c->r[9].ud[0]=0;return true;
    case 0x216990:{
        math::Matrix input;std::memcpy(input.data(),record(rt,c->r[5].uw[0],64),64);
        const auto result=math::normalizedMatrixColumns(input);
        std::memcpy(record(rt,c->r[2].uw[0],64),result.data(),64);return true;
    }
    case 0x213e1c:case 0x213e20:case 0x213e24:case 0x213e28:
    case 0x213e2c:case 0x213e30:case 0x213e34:case 0x213e38:
    case 0x213e3c:case 0x213e40:case 0x213e44:case 0x213e48:
    case 0x213e4c:case 0x213e50:case 0x213e54:case 0x213e58:
    case 0x213e5c:case 0x213e60:case 0x213e64:case 0x213e68:
    case 0x213e6c:case 0x213e70:case 0x213e74:case 0x213e78:
    case 0x213e80:case 0x213e84:case 0x213e88:return true;
    case 0x213e7c:{
        // Both input records were completed by retained scalar builders and
        // calls. Snapshot before the destination can overlap either record.
        math::Matrix local,basis;
        std::memcpy(local.data(),record(rt,c->r[8].uw[0],64),64);
        std::memcpy(basis.data(),record(rt,c->r[7].uw[0],64),64);
        const auto result=math::projectionProduct(local,basis);
        std::memcpy(record(rt,c->r[16].uw[0],64),result.data(),64);return true;
    }
    case 0x216ee0:case 0x216ee4:case 0x216ee8:case 0x216eec:
    case 0x216ef0:case 0x216ef4:case 0x216ef8:return true;
    case 0x216efc:{
        math::RigidBasis basis;__m128 translation;
        std::memcpy(basis.data(),record(rt,c->r[5].uw[0],48),48);
        auto *temporary=record(rt,c->r[3].uw[0],16);std::memcpy(&translation,temporary,16);
        const auto result=math::rigidInverseTranslation(basis,translation);
        std::memcpy(temporary,&result,16);return true;
    }
    case 0x2156a0:case 0x2156a8:case 0x2156ac:return true;
    case 0x2156b0:{
        // The retained scalar axis-angle builder has finished XYZ*sin and
        // cos(W/2). Normalize that game record in place. Its control read is
        // unused by the leaf: no vector register or status result is supplied.
        __m128 input;auto *at=record(rt,c->r[17].uw[0],16);std::memcpy(&input,at,16);
        const auto normalized=math::normalizeQuaternion(input);std::memcpy(at,&normalized,16);return true;
    }
    case 0x2158a4:case 0x2158ac:case 0x2158bc:case 0x2158c4:
    case 0x2158d4:case 0x2158d8:case 0x2158dc:return true;
    case 0x2158b0:case 0x2158c8:c->r[9].ud[0]=0;return true;
    case 0x2158b8:{
        // Preserve the normalized temporary even though the retained Euler
        // builder takes its rotation matrix from the original input record.
        __m128 input;auto *at=record(rt,c->r[2].uw[0],16);std::memcpy(&input,at,16);
        const auto normalized=vp2s::math::normalizeQuaternion(input);std::memcpy(at,&normalized,16);return true;
    }
    case 0x2158d0:{
        __m128 input;std::memcpy(&input,record(rt,c->r[5].uw[0],16),16);
        const auto matrix=vp2s::math::quaternionMatrix(input);
        std::memcpy(record(rt,c->r[2].uw[0],64),matrix.data(),64);return true;
    }
    }
    // The object's update uses the same audited calculation as construction,
    // with its quaternion/position, temporary records and result in different
    // ordinary CPU registers. No inherited vector state supplies either path.
    const bool updating=pc>=0x2492ac && pc<=0x249378;
    if(updating)pc+=0x36c;
    switch(pc){
    case 0x211dac:case 0x211db0:case 0x211db4:
    case 0x23c17c:case 0x23c180:case 0x23c184:return true;
    case 0x211dbc:case 0x23c188:{
        __m128 a,b;
        std::memcpy(&a,record(rt,c->r[pc==0x23c188?4:6].uw[0],16),16);
        std::memcpy(&b,record(rt,c->r[5].uw[0],16),16);
        const auto exponent=_mm_set1_epi32(0x7f800000);
        const auto ae=_mm_and_si128(_mm_castps_si128(a),exponent);
        const auto be=_mm_and_si128(_mm_castps_si128(b),exponent);
        if(_mm_movemask_epi8(_mm_or_si128(_mm_cmpeq_epi32(ae,exponent),_mm_cmpeq_epi32(be,exponent))))
            throw std::runtime_error("nonfinite native component-product input is unqualified");
        auto normal=[&](__m128 value){
            const auto bits=_mm_castps_si128(value);
            const auto tiny=_mm_cmpeq_epi32(_mm_and_si128(bits,exponent),_mm_setzero_si128());
            return _mm_castsi128_ps(_mm_andnot_si128(_mm_and_si128(tiny,_mm_set1_epi32(0x7fffffff)),bits));
        };
        // Four independent game-record products, with the reference's chop /
        // signed-zero policy. Host FP state and all vector-machine state stay
        // outside this record API; no flags or instruction are executed.
        music::CodecRounding rounding;
        const auto result=normal(_mm_mul_ps(normal(a),normal(b)));
        std::memcpy(record(rt,c->r[3].uw[0],16),&result,16);return true;
    }
    case 0x249618:case 0x249620:case 0x249628:
    case 0x249664:case 0x249668:
    case 0x249678:case 0x24967c:case 0x249680:case 0x249684:
    case 0x249688:case 0x24968c:case 0x249690:case 0x249694:
    case 0x249698:case 0x24969c:case 0x2496a0:case 0x2496a4:
    case 0x2496a8:case 0x2496ac:case 0x2496b0:case 0x2496b4:
    case 0x2496b8:case 0x2496bc:case 0x2496c0:case 0x2496c4:
    case 0x2496c8:case 0x2496cc:case 0x2496d0:case 0x2496d4:
    case 0x2496dc:case 0x2496e0:case 0x2496e4:
        return true;
    case 0x249658:c->r[9].ud[0]=0;return true;
    case 0x249660:{
        const uint32_t object=c->r[16].uw[0];__m128 quaternion,position;
        std::memcpy(&quaternion,record(rt,object+0x1e0,16),16);
        std::memcpy(&position,record(rt,object+0x1d0,16),16);
        auto result=math::quaternionMatrix(quaternion);
        // This entry preserves the supplied position instead of selecting the
        // common helper's zero position. It goes in the first three row Ws;
        // the fourth row is built by the retained scalar CPU instructions.
        const auto bits=_mm_castps_si128(position);
        const auto tiny=_mm_cmpeq_epi32(_mm_and_si128(bits,_mm_set1_epi32(0x7f800000)),_mm_setzero_si128());
        position=_mm_castsi128_ps(_mm_andnot_si128(_mm_and_si128(tiny,_mm_set1_epi32(0x7fffffff)),bits));
        result[0]=_mm_blend_ps(result[0],music::broadcast<0>(position),8);
        result[1]=_mm_blend_ps(result[1],music::broadcast<1>(position),8);
        result[2]=_mm_blend_ps(result[2],music::broadcast<2>(position),8);
        std::memcpy(record(rt,c->r[updating?3:2].uw[0],48),result.data(),48);return true;
    }
    case 0x2496d8:{
        math::Matrix rotation,scale,result;
        std::memcpy(rotation.data(),record(rt,c->r[updating?4:3].uw[0],64),64);
        std::memcpy(scale.data(),record(rt,c->r[updating?3:2].uw[0],64),64);
        for(unsigned i=0;i<4;++i)result[i]=math::matrixVectorProduct(scale,rotation[i]);
        std::memcpy(record(rt,c->r[updating?5:4].uw[0],64),result.data(),64);return true;
    }
    case 0x217554:case 0x217558:case 0x21755c:case 0x217560:
    case 0x217564:case 0x217568:case 0x21756c:case 0x217570:case 0x21757c:
    case 0x21758c:case 0x217590:case 0x217594:
    case 0x216734:case 0x21673c:case 0x21674c:case 0x216750:case 0x216754:
        return true; // obsolete vector transport; no shadow register writes
    case 0x217580:case 0x216740:
        c->r[9].ud[0]=0;return true; // helper's ordinary CPU clobber
    case 0x217588:{
        math::Matrix local,parent;
        std::memcpy(local.data(),record(rt,c->r[5].uw[0],64),64);
        std::memcpy(parent.data(),record(rt,c->r[6].uw[0],64),64);
        const auto result=math::compose(local,parent);
        std::memcpy(record(rt,c->r[3].uw[0],64),result.data(),64);
        return true;
    }
    case 0x216748:{
        __m128 input;std::memcpy(&input,record(rt,c->r[5].uw[0],16),16);
        const auto result=math::quaternionMatrix(input);
        std::memcpy(record(rt,c->r[3].uw[0],64),result.data(),64);
        return true;
    }
    default:return false;
    }
}
}

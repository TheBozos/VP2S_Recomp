#include "native_math.h"
#include "native_object_math.h"
#include "native_trig.h"
#include "native_frustum.h"
#include "bozo/ee/dispatcher.h"
#include "../port/vp2s_math.h"
#include <cstring>
#include <cmath>
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
    if(e==0x7f800000 || (!e && (b&0x7fffff)))throw std::runtime_error("unqualified native geometric input");}
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
struct SegmentTerms {std::array<float,3> delta;float firstLength,negativeCross,secondLength,firstProjection,separation,determinant,negativeSecondProjection,secondNumerator,firstNumerator;};
struct PointSegmentTerms {std::array<float,3> delta,direction;float projection,lengthSquared;};
PointSegmentTerms pointSegmentTerms(Runtime *rt,R5900Context *c){
    if(c->pc!=0x211fb0 || (c->r[31].uw[0]!=0x1fc244 && c->r[31].uw[0]!=0x1fc4f0))throw std::runtime_error("unqualified point/segment caller");
    auto *frame=record(rt,c->r[29].uw[0],32),*segment=record(rt,c->r[4].uw[0],32),*point=record(rt,c->r[5].uw[0],16);
    const auto address=c->r[6].uw[0];auto *parameter=rt->memory.hostPointer(address,4);
    if((address&3) || !parameter)throw std::runtime_error("invalid point/segment parameter output");
    const auto separate=[](uint8_t *a,size_t n,uint8_t *b,size_t m){const auto x=reinterpret_cast<uintptr_t>(a),y=reinterpret_cast<uintptr_t>(b);
        if(x<y+m && y<x+n)throw std::runtime_error("point/segment temporary or parameter aliases live input");};
    separate(frame,32,segment,32);separate(frame,32,point,16);separate(frame,32,parameter,4);
    // Original CPU code publishes the parameter before the final distance.
    // These spans must survive unchanged until that final typed calculation.
    separate(parameter,4,segment,32);separate(parameter,4,point,16);
    float s[2][4],p[4];std::memcpy(s,segment,32);std::memcpy(p,point,16);
    for(unsigned i=0;i<3;++i)for(float value:{s[0][i],s[1][i],p[i]}){pairQualified(value);if(std::abs(value)>1.e8f)throw std::runtime_error("unqualified point/segment magnitude");}
    PointSegmentTerms result;float products[3],squares[3];
    for(unsigned i=0;i<3;++i){result.delta[i]=pairSum(p[i],-s[0][i]);result.direction[i]=s[1][i];products[i]=pairProduct(result.delta[i],s[1][i]);squares[i]=pairProduct(s[1][i],s[1][i]);}
    result.projection=pairSum(pairSum(products[0],products[1]),pairProduct(1.f,products[2]));
    result.lengthSquared=pairSum(pairSum(squares[0],squares[1]),pairProduct(1.f,squares[2]));return result;
}
std::array<float,3> collisionNormal(Runtime *rt,R5900Context *c,uint32_t pc){
    if(c->pc!=0x1fc5f4)throw std::runtime_error("unqualified collision-normal continuation");
    const auto stack=c->r[29].uw[0];auto *frame=record(rt,stack,0x70),*normal=record(rt,c->r[18].uw[0],16);
    uint32_t caller;std::memcpy(&caller,frame+0x50,4);
    if(caller!=0x1fdf24 && caller!=0x1fe000)throw std::runtime_error("unqualified collision-normal consumer");
    if(c->r[18].uw[0]!=stack+0x280 || c->r[17].uw[0]!=stack+0x52c || c->r[16].uw[0]!=stack+0x528)
        throw std::runtime_error("collision-normal consumer output setup mismatch");
    auto *first=record(rt,c->r[19].uw[0]+0x250,32),*second=record(rt,c->r[20].uw[0]+0x230,32);
    const auto separate=[](uint8_t *a,size_t n,uint8_t *b,size_t m){const auto x=reinterpret_cast<uintptr_t>(a),y=reinterpret_cast<uintptr_t>(b);
        if(x<y+m && y<x+n)throw std::runtime_error("collision-normal live span alias");};
    separate(normal,16,frame,0x70);
    for(unsigned reg:{16u,17u}){const auto address=c->r[reg].uw[0];auto *p=rt->memory.hostPointer(address,4);
        if((address&3) || !p)throw std::runtime_error("invalid collision-normal scalar output");
        separate(p,4,first,32);separate(p,4,second,32);separate(p,4,normal,16);}
    if(pc==0x1fc628 && (c->r[3].uw[0]!=c->r[20].uw[0]+0x230 || c->r[2].uw[0]!=c->r[20].uw[0]+0x240 ||
        c->r[5].uw[0]!=c->r[19].uw[0]+0x250 || c->r[4].uw[0]!=c->r[19].uw[0]+0x260))
        throw std::runtime_error("collision-normal operand setup mismatch");
    const float firstWeight=c->f[2],secondWeight=c->f[1];pairQualified(firstWeight);pairQualified(secondWeight);
    if(std::abs(firstWeight)>2 || std::abs(secondWeight)>2)throw std::runtime_error("unqualified collision-normal parameters");
    float a[2][4],b[2][4];std::memcpy(a,first,32);std::memcpy(b,second,32);
    for(auto *vectors:{a,b})for(unsigned row=0;row<2;++row)for(unsigned i=0;i<3;++i){pairQualified(vectors[row][i]);
        if(std::abs(vectors[row][i])>1.e8f)throw std::runtime_error("unqualified collision-normal magnitude");}
    std::array<float,3> delta,result;float squared[3];
    for(unsigned i=0;i<3;++i){auto value=pairSum(b[0][i],0.f);value=pairSum(value,pairProduct(b[1][i],secondWeight));
        value=pairSum(value,-pairProduct(a[0][i],1.f));delta[i]=pairSum(value,-pairProduct(a[1][i],firstWeight));squared[i]=pairProduct(delta[i],delta[i]);}
    const auto lengthSquared=pairSum(pairSum(squared[0],squared[1]),pairProduct(1.f,squared[2]));
    volatile float root=static_cast<float>(std::sqrt(std::abs(double(lengthSquared))));
    volatile float reciprocal=root==0?std::numeric_limits<float>::max():static_cast<float>(1.0/double(root));
    for(unsigned i=0;i<3;++i)result[i]=pairProduct(delta[i],pairNormal(reciprocal));return result;
}
SegmentTerms segmentTerms(Runtime *rt,R5900Context *c){
    if(c->pc!=0x2120a0 || (c->r[31].uw[0]!=0x1fc3dc && c->r[31].uw[0]!=0x1fc5f4))
        throw std::runtime_error("unqualified segment-distance caller");
    auto *frame=record(rt,c->r[29].uw[0],16),*first=record(rt,c->r[4].uw[0],32),*second=record(rt,c->r[5].uw[0],32);
    separatePairFrame(frame,first,32);separatePairFrame(frame,second,32);
    for(unsigned reg:{6u,7u}){const auto address=c->r[reg].uw[0];auto *output=rt->memory.hostPointer(address,4);
        if((address&3) || !output)throw std::runtime_error("invalid segment-distance parameter output");separatePairFrame(frame,output,4);}
    float a[2][4],b[2][4];std::memcpy(a,first,32);std::memcpy(b,second,32);
    for(auto *vectors:{a,b})for(unsigned row=0;row<2;++row)for(unsigned i=0;i<3;++i){pairQualified(vectors[row][i]);
        if(std::abs(vectors[row][i])>1.e8f)throw std::runtime_error("unqualified segment-distance magnitude");}
    SegmentTerms result;std::array<float,3> u,v,negativeV;
    for(unsigned i=0;i<3;++i){result.delta[i]=pairSum(a[0][i],-b[0][i]);u[i]=a[1][i];v[i]=b[1][i];negativeV[i]=pairProduct(v[i],-1.f);}
    const auto dot=[](const auto &x,const auto &y){return pairSum(pairSum(pairProduct(x[0],y[0]),pairProduct(x[1],y[1])),pairProduct(1.f,pairProduct(x[2],y[2])));};
    result.firstLength=dot(u,u);result.negativeCross=pairProduct(dot(u,v),-1.f);result.secondLength=dot(v,v);
    result.firstProjection=dot(result.delta,u);result.separation=dot(result.delta,result.delta);
    result.determinant=std::abs(pairSum(pairProduct(result.firstLength,result.secondLength),-pairProduct(result.negativeCross,result.negativeCross)));
    result.negativeSecondProjection=dot(result.delta,negativeV);
    result.secondNumerator=pairSum(pairProduct(result.negativeCross,result.firstProjection),-pairProduct(result.firstLength,result.negativeSecondProjection));
    result.firstNumerator=pairSum(pairProduct(result.negativeCross,result.negativeSecondProjection),-pairProduct(result.secondLength,result.firstProjection));return result;
}
void collisionPositionResponse(Runtime *rt,R5900Context *c,uint32_t pc){
    if(c->pc!=0x1fc030 || (c->r[31].uw[0]!=0x1fdf68 && c->r[31].uw[0]!=0x1fe024))
        throw std::runtime_error("unqualified collision position-response caller");
    const auto normalAddress=c->r[29].uw[0]+0x210;auto *normal=record(rt,normalAddress,16),*coefficients=record(rt,0x359100,16);
    if(pc==0x1fc038){
        if(c->r[6].uw[0]!=normalAddress || c->r[3].uw[0]!=0x359100)throw std::runtime_error("collision position-response argument setup mismatch");
        record(rt,c->r[4].uw[0],0x40);auto *part=record(rt,c->r[5].uw[0],0x40);uint32_t other;std::memcpy(&other,part+0x38,4);record(rt,other,0x40);return;
    }
    const bool paired=pc==0x1fc0c4,secondOnly=pc==0x1fc128;
    auto *first=record(rt,c->r[4].uw[0]+(paired?0:0x30),16),*second=record(rt,c->r[6].uw[0]+0x30,16);
    auto *destination=secondOnly?second:first;
    if(c->r[3].uw[0]!=(paired || secondOnly?c->r[6].uw[0]+0x30:c->r[4].uw[0]+0x30))
        throw std::runtime_error("collision position-response destination mismatch");
    float n[4],a[4],b[4],scale,fallback=0;std::memcpy(n,normal,16);std::memcpy(a,first,16);std::memcpy(b,second,16);std::memcpy(&scale,coefficients+12,4);
    const auto parameter=c->f[12],depth=c->f[13];pairQualified(parameter);pairQualified(depth);pairQualified(scale);
    if(std::abs(parameter)>2 || std::abs(depth)>1.e8f || std::abs(scale)>1.e8f)throw std::runtime_error("unqualified collision response coefficients");
    for(unsigned i=0;i<3;++i){pairQualified(n[i]);if(std::abs(n[i])>1.e8f)throw std::runtime_error("unqualified collision response normal");}
    const auto scaledDepth=pairProduct(depth,scale);std::array<float,3> correction;
    for(unsigned i=0;i<3;++i)correction[i]=pairProduct(n[i],scaledDepth);
    float weights[2];if(paired){weights[0]=pairSum(1.f,-parameter);weights[1]=parameter;}
    else{const bool capped=secondOnly?parameter<std::bit_cast<float>(0x3c23d70au):parameter>std::bit_cast<float>(0x3f7d70a4u);
        if(capped){std::memcpy(&fallback,coefficients+8,4);pairQualified(fallback);weights[0]=fallback;}
        else{volatile float inverse=parameter==0?std::copysign(std::numeric_limits<float>::max(),parameter):1.f/parameter;weights[0]=pairNormal(inverse);}}
    std::array<std::array<float,4>,2> results;const unsigned count=paired?2:1;
    for(unsigned which=0;which<count;++which){const auto *input=paired?(which?b:a):secondOnly?b:a;std::memcpy(results[which].data(),input,16);
        for(unsigned i=0;i<3;++i){pairQualified(input[i]);}
        {music::CodecRounding validation;for(unsigned i=0;i<3;++i){const double bound=std::abs(double(input[i]))+std::abs(double(correction[i])*weights[which]);
            if(bound>double(std::numeric_limits<float>::max())/2)throw std::runtime_error("collision response position overflow");}}
        for(unsigned i=0;i<3;++i){const auto base=paired?pairSum(input[i],0.f):pairProduct(input[i],1.f);results[which][i]=pairSum(base,pairProduct(correction[i],weights[which]));}
    }
    // Pair results are snapshotted before either store; only the already
    // loaded position's own W survives either source path.
    if(paired){std::memcpy(first,results[0].data(),16);std::memcpy(second,results[1].data(),16);}
    else std::memcpy(destination,results[0].data(),16);
}
struct AngularDirections {std::array<float,3> first,second;float cosine;};
AngularDirections angularDirections(Runtime *rt,R5900Context *c){
    const auto stack=c->r[29].uw[0];auto *frame=record(rt,stack,0x1a0);
    if(c->r[4].uw[0]!=stack+0x190 || c->r[5].uw[0]!=stack+0x150)
        throw std::runtime_error("angular-constraint source record mismatch");
    const auto external=[&](uint32_t at,uint32_t bytes){auto *p=record(rt,at,bytes);
        const auto a=reinterpret_cast<uintptr_t>(p),b=reinterpret_cast<uintptr_t>(frame);
        if(a<b+0x1a0 && b<a+bytes)throw std::runtime_error("angular-constraint object aliases its caller frame");return p;};
    auto *part=external(c->r[21].uw[0],0x40);uint32_t pointRecord;std::memcpy(&pointRecord,part+0x38,4);
    external(c->r[19].uw[0],0xa0);external(c->r[17].uw[0],0x210);external(c->r[16].uw[0],0x210);
    auto *point=external(pointRecord,0xa0);
    math::Matrix matrix;float rows[4][4],points[2][4];std::memcpy(matrix.data(),frame+0x150,64);std::memcpy(rows,matrix.data(),64);
    std::memcpy(points[0],frame+0x190,16);std::memcpy(points[1],point+0x30,16);
    for(auto &row:rows)for(unsigned i=0;i<3;++i)pairQualified(row[i]);
    for(auto &v:points)for(unsigned i=0;i<3;++i)pairQualified(v[i]);
    {music::CodecRounding validation;const double limit=std::sqrt(double(std::numeric_limits<float>::max())/3);
        for(auto &v:points)for(unsigned lane=0;lane<3;++lane){double bound=std::abs(double(rows[3][lane]));
            for(unsigned i=0;i<3;++i)bound+=std::abs(double(rows[i][lane])*v[i]);
            if(bound>limit)throw std::runtime_error("angular-constraint transform/length overflow");}}
    // Only XYZ survives normalization and the cosine consumer. Discard dead
    // input/output W work explicitly; all three translation coordinates remain.
    for(auto &row:matrix)row=_mm_blend_ps(row,_mm_setzero_ps(),8);
    AngularDirections result;
    for(unsigned which=0;which<2;++which){__m128 pointVector;std::memcpy(&pointVector,points[which],16);
        pointVector=_mm_blend_ps(pointVector,_mm_set1_ps(1),8);
        const auto transformed=math::matrixVectorProduct(matrix,pointVector);float value[4];std::memcpy(value,&transformed,16);
        const auto squared=pairSum(pairSum(pairProduct(value[0],value[0]),pairProduct(value[1],value[1])),pairProduct(1.f,pairProduct(value[2],value[2])));
        volatile float length=static_cast<float>(std::sqrt(double(squared)));
        volatile float reciprocal=length==0?std::numeric_limits<float>::max():static_cast<float>(1.0/double(length));
        auto &direction=which?result.second:result.first;
        {music::CodecRounding validation;for(unsigned i=0;i<3;++i)if(std::abs(double(value[i])*reciprocal)>std::numeric_limits<float>::max())
            throw std::runtime_error("angular-constraint normalization overflow");}
        for(unsigned i=0;i<3;++i)direction[i]=pairProduct(value[i],pairNormal(reciprocal));
    }
    {music::CodecRounding validation;double bound=0;for(unsigned i=0;i<3;++i)bound+=std::abs(double(result.first[i])*result.second[i]);
        if(bound>std::numeric_limits<float>::max())throw std::runtime_error("angular-constraint cosine overflow");}
    result.cosine=pairSum(pairSum(pairProduct(result.first[0],result.second[0]),pairProduct(result.first[1],result.second[1])),pairProduct(result.first[2],result.second[2]));
    return result;
}
std::array<float,4> composeConstraintQuaternions(const std::array<float,4> &a,const std::array<float,4> &b){
    for(float v:a)pairQualified(v);for(float v:b)pairQualified(v);
    {music::CodecRounding validation;const double limit=std::numeric_limits<float>::max();double scalarBound=0;
        for(unsigned i=0;i<4;++i)scalarBound+=std::abs(double(a[i])*b[i]);
        if(scalarBound>limit)throw std::runtime_error("joint quaternion scalar overflow");
        for(unsigned i=0;i<3;++i){const auto j=(i+1)%3,k=(i+2)%3;
            const double bound=std::abs(double(b[i])*a[3])+std::abs(double(a[i])*b[3])+std::abs(double(a[j])*b[k])+std::abs(double(b[j])*a[k]);
            if(bound>limit)throw std::runtime_error("joint quaternion vector overflow");}}
    float products[4];for(unsigned i=0;i<4;++i)products[i]=pairProduct(a[i],b[i]);std::array<float,4> result;
    for(unsigned i=0;i<3;++i){const auto j=(i+1)%3,k=(i+2)%3;
        const auto cross=pairSum(pairProduct(a[j],b[k]),-pairProduct(b[j],a[k]));
        const auto base=pairSum(pairProduct(b[i],a[3]),pairProduct(a[i],b[3]));result[i]=pairSum(base,pairProduct(cross,1.f));}
    result[3]=pairSum(pairSum(pairSum(products[3],-products[2]),-products[1]),-products[0]);return result;
}
std::array<float,4> linearQuaternionBlend(const std::array<float,4> &a,const std::array<float,4> &b,float firstWeight,float secondWeight,bool subtract,float scale=1.f){
    for(float v:a)pairQualified(v);for(float v:b)pairQualified(v);pairQualified(firstWeight);pairQualified(secondWeight);pairQualified(scale);
    {music::CodecRounding validation;const double limit=std::sqrt(double(std::numeric_limits<float>::max())/4);
        for(unsigned i=0;i<4;++i){const double raw=std::abs(double(b[i])*secondWeight)+std::abs(double(a[i])*firstWeight);
            if(raw>std::numeric_limits<float>::max() || raw*std::abs(double(scale))>limit)throw std::runtime_error("quaternion blend/length overflow");}}
    std::array<float,4> value,result;float squares[4];
    for(unsigned i=0;i<4;++i){const auto first=pairProduct(a[i],firstWeight),second=pairProduct(b[i],secondWeight);
        value[i]=pairProduct(pairSum(second,subtract?-first:first),scale);squares[i]=pairProduct(value[i],value[i]);}
    const auto squared=pairSum(pairSum(pairSum(squares[0],squares[1]),pairProduct(1.f,squares[2])),pairProduct(1.f,squares[3]));
    volatile float root=static_cast<float>(std::sqrt(std::abs(double(squared))));
    volatile float reciprocal=root==0?std::numeric_limits<float>::max():static_cast<float>(1.0/double(root));
    {music::CodecRounding validation;for(float v:value)if(std::abs(double(v)*reciprocal)>std::numeric_limits<float>::max())
        throw std::runtime_error("linear quaternion normalization overflow");}
    for(unsigned i=0;i<4;++i)result[i]=pairProduct(value[i],pairNormal(reciprocal));return result;
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
    if(pc==0x211fe8 || pc==0x212020 || pc==0x212064 || pc==0x212084){
        const auto terms=pointSegmentTerms(rt,c);const auto stack=c->r[29].uw[0];float value;
        if(c->r[2].uw[0]!=(pc==0x212020?stack:stack+0x10))throw std::runtime_error("point/segment temporary destination mismatch");
        if(pc==0x211fe8)value=terms.projection;
        else if(pc==0x212020)value=terms.lengthSquared;
        else if(pc==0x212064){
            if(!(terms.projection>0 && terms.projection<terms.lengthSquared))throw std::runtime_error("point/segment interior branch mismatch");
            volatile float ratio=terms.projection/terms.lengthSquared;value=pairSum(0.f,pairNormal(ratio));
        }else{
            auto difference=terms.delta;
            if(terms.projection>0){
                if(terms.projection>=terms.lengthSquared){for(unsigned i=0;i<3;++i)difference[i]=pairSum(difference[i],-terms.direction[i]);}
                else{volatile float ratio=terms.projection/terms.lengthSquared;for(unsigned i=0;i<3;++i)difference[i]=pairSum(pairSum(difference[i],0.f),-pairProduct(terms.direction[i],pairNormal(ratio)));}
            }
            float squared[3];for(unsigned i=0;i<3;++i)squared[i]=pairProduct(difference[i],difference[i]);value=pairSum(pairSum(squared[0],squared[1]),pairProduct(1.f,squared[2]));
        }
        // Only X is used; complete frame/input/output span checks above guard
        // retirement of the other three temporary words.
        std::memcpy(record(rt,c->r[2].uw[0],16),&value,4);return true;
    }
    switch(pc){
    case 0x211fb8:case 0x211fbc:case 0x211fc0:case 0x211fc4:case 0x211fc8:case 0x211fcc:case 0x211fd0:case 0x211fd4:
    case 0x211fd8:case 0x211fe0:case 0x211fe4:case 0x212010:case 0x21201c:case 0x212048:case 0x212054:case 0x212058:
    case 0x21205c:case 0x212060:case 0x212068:case 0x21207c:case 0x212080:return true;
    default:break;
    }
    if(pc==0x1fc038 || pc==0x1fc0c4 || pc==0x1fc128 || pc==0x1fc174){collisionPositionResponse(rt,c,pc);return true;}
    switch(pc){
    case 0x1fc044:case 0x1fc050:case 0x1fc054:case 0x1fc05c:case 0x1fc080:case 0x1fc084:case 0x1fc0ac:case 0x1fc0b0:
    case 0x1fc0b4:case 0x1fc0b8:case 0x1fc0bc:case 0x1fc0c0:case 0x1fc0cc:case 0x1fc0e0:case 0x1fc100:case 0x1fc108:
    case 0x1fc110:case 0x1fc114:case 0x1fc118:case 0x1fc11c:case 0x1fc134:case 0x1fc154:case 0x1fc15c:case 0x1fc160:
    case 0x1fc164:case 0x1fc168:case 0x1fc16c:return true;
    default:break;
    }
    if(pc==0x1fc628){collisionNormal(rt,c,pc);return true;}
    if(pc==0x1fc690){const auto normal=collisionNormal(rt,c,pc);
        // Both audited physics continuations feed this record to 1fc030,
        // which consumes XYZ only. No stale normal W is synthesized.
        std::memcpy(record(rt,c->r[18].uw[0],16),normal.data(),12);return true;}
    if(pc==0x1fc62c || pc==0x1fc638 || pc==0x1fc63c || pc==0x1fc640 || pc==0x1fc64c ||
       (pc>=0x1fc650 && pc<=0x1fc670 && !(pc&3)) || pc==0x1fc688 || pc==0x1fc68c)return true;
    if((pc>=0x2120ac && pc<=0x212110 && !(pc&3)) || pc==0x212118 || pc==0x212138 ||
       (pc>=0x212170 && pc<=0x212184 && !(pc&3)))return true;
    if(pc==0x21211c || pc==0x212120 || pc==0x212128 || pc==0x212130 || pc==0x21213c || pc==0x212144 || pc==0x21214c || pc==0x212188 || pc==0x212190 || pc==0x212198){
        const auto terms=segmentTerms(rt,c);
        if(pc==0x21211c){if(c->r[2].uw[0]!=c->r[29].uw[0])throw std::runtime_error("segment-distance temporary mismatch");
            // XYZ is read by the original parallel-segment scalar branch.
            // W is dead; frame/input/output aliases that could expose it reject.
            std::memcpy(record(rt,c->r[29].uw[0],16),terms.delta.data(),12);return true;}
        float value=0;switch(pc){
        case 0x212120:value=terms.firstLength;break;case 0x212128:value=terms.negativeCross;break;
        case 0x212130:value=terms.secondLength;break;case 0x21213c:value=terms.firstProjection;break;
        case 0x212144:value=terms.separation;break;case 0x21214c:value=terms.determinant;break;
        case 0x212188:value=terms.negativeSecondProjection;break;case 0x212190:value=terms.secondNumerator;break;
        case 0x212198:value=terms.firstNumerator;break;}
        // Only the scalar word is consumed; both audited callers overwrite
        // these integer temporaries before using a packed or upper component.
        c->r[pc==0x21214c?3:2].uw[0]=std::bit_cast<uint32_t>(value);return true;
    }
    if(pc==0x1fde1c || pc==0x1fde20 || (pc>=0x1fde40 && pc<=0x1fde60 && !(pc&3)) || pc==0x1fde70 || pc==0x1fde78)return true;
    if(pc==0x1fde6c){
        const auto stack=c->r[29].uw[0];auto *origin=record(rt,stack+0x460,16),*direction=record(rt,stack+0x220,16);
        if(c->r[2].uw[0]!=stack+0x460)throw std::runtime_error("collision query origin destination mismatch");
        auto *part=record(rt,c->r[17].uw[0],0x40),*current=record(rt,c->r[18].uw[0],0x40);
        uint32_t otherAddress;std::memcpy(&otherAddress,part+0x38,4);auto *other=record(rt,otherAddress+0x30,16);
        float a[4],b[4];std::memcpy(a,other,16);std::memcpy(b,current+0x30,16);
        for(unsigned i=0;i<3;++i){pairQualified(a[i]);pairQualified(b[i]);}
        float offset[3]{};const bool shifted=part[0x20]&4;
        if(shifted){
            uint32_t owner;std::memcpy(&owner,current+4,4);auto *basis=record(rt,owner+0x150,48);
            float matrix[3][4],local[4];std::memcpy(matrix,basis,48);std::memcpy(local,part,16);
            for(unsigned i=0;i<3;++i){pairQualified(local[i]);for(unsigned j=0;j<3;++j)pairQualified(matrix[i][j]);}
            {music::CodecRounding validation;for(unsigned lane=0;lane<3;++lane){double bound=0;
                for(unsigned i=0;i<3;++i)bound+=std::abs(double(matrix[i][lane])*local[i]);
                if(bound>std::numeric_limits<float>::max())throw std::runtime_error("collision query offset overflow");}}
            for(unsigned lane=0;lane<3;++lane)offset[lane]=pairSum(pairSum(pairProduct(matrix[0][lane],local[0]),pairProduct(matrix[1][lane],local[1])),pairProduct(matrix[2][lane],local[2]));
        }
        {music::CodecRounding validation;for(unsigned i=0;i<3;++i){const double bound=std::abs(double(a[i]))+std::abs(double(b[i]))+2*std::abs(double(offset[i]));
            if(bound>std::numeric_limits<float>::max())throw std::runtime_error("collision query endpoint/direction overflow");}}
        for(unsigned i=0;i<3;++i){if(shifted){a[i]=pairSum(a[i],offset[i]);b[i]=pairSum(b[i],offset[i]);}a[i]=pairSum(a[i],-b[i]);}
        // Both results precede the first original store. Preserve source W
        // words, including opaque payloads, and snapshot all aliases first.
        std::memcpy(origin,b,16);std::memcpy(direction,a,16);return true;
    }
    if(pc==0x1fe8b0 || pc==0x1fe8f0 || pc==0x1fe8f4 || pc==0x1fe8f8 || pc==0x1fe8fc || pc==0x1fe924 || pc==0x1fe928)return true;
    if(pc==0x1fe934){
        const auto object=c->r[5].uw[0];auto *source=record(rt,object,0x70);
        if(c->r[3].uw[0]!=object+0x50 || c->r[4].uw[0]!=object+0x30)throw std::runtime_error("velocity update record mismatch");
        float current[4],previous[4],contact;std::array<uint32_t,4> result;
        std::memcpy(current,source+0x30,16);std::memcpy(previous,source+0x40,16);std::memcpy(&contact,source+0x1c,4);std::memcpy(result.data(),current,16);
        pairQualified(contact);const bool damped=contact>0;const auto coefficient=c->f[20],factor=c->f[0];pairQualified(coefficient);if(damped)pairQualified(factor);
        for(unsigned i=0;i<3;++i){pairQualified(current[i]);pairQualified(previous[i]);}
        {music::CodecRounding validation;const double limit=std::numeric_limits<float>::max();
            for(unsigned i=0;i<3;++i){const double bound=std::abs(double(current[i])*coefficient)+std::abs(double(previous[i])*coefficient);
                if(bound>limit || (damped && bound*std::abs(double(factor))>limit))throw std::runtime_error("velocity update arithmetic overflow");}}
        for(unsigned i=0;i<3;++i){auto value=pairSum(pairProduct(current[i],coefficient),-pairProduct(previous[i],coefficient));
            if(damped)value=pairProduct(value,factor);result[i]=std::bit_cast<uint32_t>(value);}
        std::memcpy(source+0x50,result.data(),16);return true;
    }
    if((pc>=0x215298 && pc<=0x2152a4 && !(pc&3)) || pc==0x2152ac ||
       (pc>=0x2152b4 && pc<=0x2152c0 && !(pc&3)) || pc==0x2152c8 ||
       (pc>=0x215348 && pc<=0x21535c && !(pc&3)) || pc==0x215364 || pc==0x215368)return true;
    if((pc>=0x2152ec && pc<=0x215304 && !(pc&3)) || pc==0x21530c || (pc>=0x215324 && pc<=0x215340 && !(pc&3)))return true;
    if(pc==0x21536c){
        const bool nonlinear=c->pc==0x2152ec || c->pc==0x215324;
        if((!nonlinear && c->pc!=0x215260 && c->pc!=0x215274) || c->r[19].ud[0]>1 ||
           bool(c->r[19].ud[0])!=(c->pc==0x215274 || c->pc==0x2152ec))throw std::runtime_error("unported interior quaternion interpolation");
        const auto stack=c->r[29].uw[0];float weights[2],scale=1.f;
        if(nonlinear){
            if(std::bit_cast<uint32_t>(c->f[0])!=std::bit_cast<uint32_t>(c->f[14]) || std::bit_cast<uint32_t>(c->f[21])!=0x3c23d70a || c->f[0]<c->f[21])
                throw std::runtime_error("nonlinear quaternion interpolation scalar arguments mismatch");
            const auto sines=nativeThreeSines(rt,{c->f[12],c->f[13],c->f[14]});weights[0]=sines[0];weights[1]=sines[1];
            volatile float reciprocal=sines[2]==0?std::copysign(std::numeric_limits<float>::max(),sines[2]):1.f/sines[2];scale=pairNormal(reciprocal);
        }else {
            std::memcpy(weights,record(rt,stack+0x60,16),8);
            if(std::bit_cast<uint32_t>(weights[0])!=std::bit_cast<uint32_t>(c->f[0]) || std::bit_cast<uint32_t>(weights[1])!=std::bit_cast<uint32_t>(c->f[22]))
                throw std::runtime_error("linear quaternion interpolation weight record mismatch");
        }
        std::array<float,4> a,b;std::memcpy(a.data(),record(rt,c->r[17].uw[0],16),16);std::memcpy(b.data(),record(rt,c->r[16].uw[0],16),16);
        const auto result=linearQuaternionBlend(a,b,weights[0],weights[1],c->r[19].ud[0]!=0,scale);
        std::memcpy(record(rt,c->r[18].uw[0],16),result.data(),16);return true;
    }
    // The angular-constraint algorithm owns a literal half coefficient. Its
    // obsolete vector-register initialization publishes no game data. Remaining
    // unported consumers still fail at their own explicit native boundaries.
    if(pc==0x1fcb7c)return true;
    if(pc==0x1fcdfc || pc==0x1fce00 || pc==0x1fce04 || (pc>=0x1fce0c && pc<=0x1fce2c && !(pc&3)))return true;
    if(pc==0x1fce30){
        const auto destination=c->r[29].uw[0]+0x130,source=c->r[17].uw[0]+0x30;
        if(c->r[2].uw[0]!=destination || c->r[3].uw[0]!=source)throw std::runtime_error("joint quaternion composition record mismatch");
        auto *out=record(rt,destination,16);std::array<float,4> a,b;std::memcpy(a.data(),out,16);std::memcpy(b.data(),record(rt,source,16),16);
        const auto result=composeConstraintQuaternions(a,b);std::memcpy(out,result.data(),16);return true;
    }
    if((pc>=0x1fcd08 && pc<=0x1fcd5c && !(pc&3)) ||
       (pc>=0x1fcd6c && pc<=0x1fcda4 && !(pc&3)))return true;
    if(pc==0x1fcdac){
        const auto stack=c->r[29].uw[0];if(c->r[3].uw[0]!=stack+0x140)throw std::runtime_error("angular-constraint cosine destination mismatch");
        const auto directions=angularDirections(rt,c);std::memcpy(record(rt,stack+0x140,16),&directions.cosine,4);return true;
    }
    if(pc==0x1fcdb0 || pc==0x1fcdb4 || pc==0x1fcdc8 || pc==0x1fcdd0 || pc==0x1fcdd4 || pc==0x1fcdd8 ||
       pc==0x1fcddc || pc==0x1fcde0 || pc==0x1fcde4 || pc==0x1fcde8 || pc==0x1fcdec)return true;
    if(pc==0x1fcdf0){
        const auto stack=c->r[29].uw[0];auto *result=record(rt,stack+0x130,16);
        if(c->r[2].uw[0]!=stack+0x130 || c->r[3].uw[0]!=stack+0x140 || std::bit_cast<uint32_t>(c->f[1])!=0x3f7ffffe)
            throw std::runtime_error("angular-constraint quaternion record mismatch");
        uint32_t cosineBits;std::memcpy(&cosineBits,record(rt,stack+0x140,16),4);
        const auto directions=angularDirections(rt,c);
        if(std::bit_cast<uint32_t>(directions.cosine)!=cosineBits || std::bit_cast<uint32_t>(c->f[0])!=cosineBits || !(c->f[0]<c->f[1]))
            throw std::runtime_error("angular-constraint quaternion has stale cosine/branch inputs");
        const auto &a=directions.first,&b=directions.second;
        {music::CodecRounding validation;const double limit=std::numeric_limits<float>::max();
            if(2+2*std::abs(double(directions.cosine))>limit)throw std::runtime_error("angular-constraint quaternion coefficient overflow");
            for(unsigned i=0;i<3;++i){const auto j=(i+1)%3,k=(i+2)%3;
                if(std::abs(double(b[j])*a[k])+std::abs(double(a[j])*b[k])>limit)throw std::runtime_error("angular-constraint cross product overflow");}}
        const auto squared=pairSum(2.f,pairProduct(directions.cosine,2.f));
        volatile float root=static_cast<float>(std::sqrt(std::abs(double(squared))));
        volatile float reciprocal=root==0?std::numeric_limits<float>::max():1.f/root;
        float cross[3];for(unsigned i=0;i<3;++i){const auto j=(i+1)%3,k=(i+2)%3;cross[i]=pairSum(pairProduct(b[j],a[k]),-pairProduct(a[j],b[k]));}
        {music::CodecRounding validation;for(float v:cross)if(std::abs(double(v)*reciprocal)>std::numeric_limits<float>::max())
            throw std::runtime_error("angular-constraint quaternion axis overflow");}
        std::array<float,4> quaternion;for(unsigned i=0;i<3;++i)quaternion[i]=pairProduct(cross[i],pairNormal(reciprocal));quaternion[3]=pairProduct(root,0.5f);
        std::memcpy(result,quaternion.data(),16);return true;
    }
    if(pc>=0x216da4 && pc<=0x216dbc && !(pc&3))return true;
    if(pc==0x216e04){
        const auto matrixAddress=c->r[5].uw[0],frame=c->r[29].uw[0];
        auto *matrixRecord=record(rt,matrixAddress,64),*translationRecord=record(rt,frame,16);
        if(c->r[3].uw[0]!=frame)throw std::runtime_error("native transposed inverse translation destination mismatch");
        const auto a=reinterpret_cast<uintptr_t>(matrixRecord),b=reinterpret_cast<uintptr_t>(translationRecord);
        if(a<b+16 && b<a+64)throw std::runtime_error("native transposed inverse output aliases its temporary");
        math::Matrix matrix;__m128 translation;float rows[4][4],point[4];
        std::memcpy(matrix.data(),matrixRecord,64);std::memcpy(&translation,translationRecord,16);std::memcpy(rows,matrix.data(),64);std::memcpy(point,&translation,16);
        for(unsigned i=0;i<3;++i){pairQualified(point[i]);for(unsigned j=0;j<3;++j)pairQualified(rows[i][j]);}
        {music::CodecRounding validation;for(unsigned lane=0;lane<3;++lane){double bound=0;
            for(unsigned i=0;i<3;++i)bound+=std::abs(double(rows[lane][i])*point[i]);
            if(bound>std::numeric_limits<float>::max())throw std::runtime_error("native transposed inverse translation overflow");}}
        // Scalar clear, sequential translation loads and integer transpose have
        // already executed. Output/frame separation keeps those inputs intact.
        const auto result=math::transposedRigidInverseTranslation(matrix,translation);
        std::memcpy(translationRecord,&result,16);return true;
    }
    switch(pc){
    case 0x1fc6dc:case 0x1fc6e0:case 0x1fc708:case 0x1fc70c:case 0x1fc714:case 0x1fc71c:case 0x1fc720:case 0x1fc760:
    case 0x1fc78c:case 0x1fc790:case 0x1fc794:case 0x1fc7a0:case 0x1fc7bc:case 0x1fc7c4:case 0x1fc7e4:case 0x1fc7e8:
    case 0x1fc7ec:case 0x1fc7f0:case 0x1fc7f4:case 0x1fc800:case 0x1fc824:case 0x1fc830:case 0x1fc834:case 0x1fc838:
    case 0x1fc83c:case 0x1fc840:case 0x1fc848:return true;
    case 0x1fc86c:case 0x1fc870:case 0x1fc898:case 0x1fc8a0:case 0x1fc8a4:case 0x1fc8a8:case 0x1fc8ac:case 0x1fc8b4:
    case 0x1fc8f8:case 0x1fc8fc:case 0x1fc900:case 0x1fc90c:case 0x1fc948:case 0x1fc950:case 0x1fc970:case 0x1fc974:
    case 0x1fc978:case 0x1fc97c:case 0x1fc980:case 0x1fc98c:case 0x1fc9ac:case 0x1fc9b8:case 0x1fc9bc:case 0x1fc9c0:
    case 0x1fc9c4:case 0x1fc9c8:case 0x1fc9d0:return true;
    default:break;
    }
    if(pc==0x1fc908 || pc==0x1fc79c){
        const auto frame=c->r[29].uw[0];auto *stack=record(rt,frame,16);
        if(c->r[3].uw[0]!=frame)throw std::runtime_error("constrained-position distance destination mismatch");
        auto *first=record(rt,c->r[4].uw[0]+0x30,16),*second=record(rt,c->r[6].uw[0]+0x30,16),*constraint=record(rt,c->r[5].uw[0],0x30);
        separatePairFrame(stack,first,16);separatePairFrame(stack,second,16);separatePairFrame(stack,constraint,0x30);
        float a[4],b[4];std::memcpy(a,first,16);std::memcpy(b,second,16);const auto geometry=positionPairGeometry(a,b);
        // The scalar tail reads only this distance. YZW die at frame release;
        // aliases that could make those retired stores observable reject above.
        std::memcpy(stack,&geometry.distance,4);return true;
    }
    if(pc==0x1fc984 || pc==0x1fc9cc || pc==0x1fc7f8 || pc==0x1fc844){
        auto *first=record(rt,c->r[5].uw[0],16),*second=record(rt,c->r[4].uw[0],16),*frame=record(rt,c->r[29].uw[0],16);
        if(c->r[4].uw[0]!=c->r[6].uw[0]+0x30)throw std::runtime_error("constrained-position output record mismatch");
        separatePairFrame(frame,first,16);separatePairFrame(frame,second,16);
        float a[4],b[4];std::memcpy(a,first,16);std::memcpy(b,second,16);const auto geometry=positionPairGeometry(a,b);
        const float firstWeight=pc==0x1fc984 || pc==0x1fc7f8?c->f[0]:c->f[1],secondWeight=c->f[0];pairQualified(firstWeight);pairQualified(secondWeight);
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
       (pc>=0x1fb87c && pc<0x1fb8a0 && !(pc&3)) ||
       (pc>=0x1fbdcc && pc<0x1fbdf0 && !(pc&3)) ||
       (pc>=0x1fbe6c && pc<0x1fbe90 && !(pc&3)) ||
       (pc>=0x1fbf78 && pc<0x1fbf9c && !(pc&3)))return true;
    if(pc==0x1fb418 || pc==0x1fb610 || pc==0x1fb8a0 || pc==0x1fbdf0 || pc==0x1fbe90 || pc==0x1fbf9c){
        const bool local=pc==0x1fbdf0 || pc==0x1fbe90,world=pc==0x1fbf9c;
        const unsigned matrixRegister=local || world?3:5,vectorRegister=local?5:world?6:4,destinationRegister=local?2:world?6:3;
        math::Matrix matrix;__m128 vector;
        std::memcpy(matrix.data(),record(rt,c->r[matrixRegister].uw[0],64),64);std::memcpy(&vector,record(rt,c->r[vectorRegister].uw[0],16),16);
        float rows[4][4],point[4];std::memcpy(rows,matrix.data(),64);std::memcpy(point,&vector,16);
        const auto qualified=[](float v){const auto b=std::bit_cast<uint32_t>(v),e=b&0x7f800000;
            if(e==0x7f800000 || (!e && (b&0x7fffff)))throw std::runtime_error("unqualified native full transform input");};
        for(auto &row:rows)for(float v:row)qualified(v);for(float v:point)qualified(v);
        {music::CodecRounding validation;
            for(unsigned lane=0;lane<4;++lane){double bound=0;for(unsigned row=0;row<4;++row)bound+=std::abs(double(rows[row][lane])*point[row]);
                if(bound>std::numeric_limits<float>::max())throw std::runtime_error("native full transform overflow");}}
        const auto result=math::matrixVectorProduct(matrix,vector);
        std::memcpy(record(rt,c->r[destinationRegister].uw[0],16),&result,16);return true;
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

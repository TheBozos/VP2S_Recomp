#include "native_math.h"
#include "native_object_math.h"
#include "native_trig.h"
#include "native_particles.h"
#include "native_frustum.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/ops_fpu.h"
#include "../port/vp2s_math.h"
#include <cstring>
#include <map>
#include <algorithm>
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
// Magnitude above 1e18 by bits: comparing a denormal as a float would raise
// the host's denormal-operand flag outside any rounding guard.
bool beyondRange(float v){return (std::bit_cast<uint32_t>(v)&0x7fffffffu)>std::bit_cast<uint32_t>(1.e18f);}
void publishUnitQuaternion(Runtime *rt,uint32_t at){
    __m128 input;auto *source=record(rt,at,16);std::memcpy(&input,source,16);float values[4];std::memcpy(values,&input,16);
    for(auto value:values){pairQualified(value);if(std::abs(value)>1.e8f)throw std::runtime_error("unqualified native Euler quaternion input");}
    const auto result=math::normalizeQuaternion(input);std::memcpy(source,&result,16);
}
void publishFullPoint(Runtime *rt,uint32_t matrix,uint32_t point,uint32_t destination,bool homogeneous){
    struct ArithmeticScope{unsigned saved=_mm_getcsr();~ArithmeticScope(){_mm_setcsr(saved);}} arithmetic;
    math::Matrix basis;__m128 position;std::memcpy(basis.data(),record(rt,matrix,64),64);std::memcpy(&position,record(rt,point,16),16);
    float values[20];std::memcpy(values,basis.data(),64);std::memcpy(values+16,&position,16);
    for(float value:values){pairQualified(value);if(std::abs(value)>1.e8f)throw std::runtime_error("unqualified native object-update point input");}
    if(homogeneous && std::bit_cast<uint32_t>(values[19])!=0x3f800000)
        throw std::runtime_error("object-update point lacks original scalar homogeneous component");
    auto *output=record(rt,destination,16);const auto result=math::matrixVectorProduct(basis,position);std::memcpy(output,&result,16);
}
void publishLinearVector(Runtime *rt,uint32_t matrix,uint32_t vector,uint32_t destination){
    struct ArithmeticScope{unsigned saved=_mm_getcsr();~ArithmeticScope(){_mm_setcsr(saved);}} arithmetic;
    math::RigidBasis basis;__m128 input;std::memcpy(basis.data(),record(rt,matrix,48),48);std::memcpy(&input,record(rt,vector,16),16);
    float coefficients[12],components[4];std::memcpy(coefficients,basis.data(),48);std::memcpy(components,&input,16);
    for(float value:coefficients){pairQualified(value);if(std::abs(value)>1.e8f)throw std::runtime_error("unqualified native linear basis");}
    for(unsigned i=0;i<3;++i){pairQualified(components[i]);if(std::abs(components[i])>1.e8f)throw std::runtime_error("unqualified native linear vector");}
    // Input W is not a weight. Basis W still contributes to the full output.
    auto *output=record(rt,destination,16);const auto result=math::linearVectorProduct(basis,input);std::memcpy(output,&result,16);
}
std::array<float,4> rowPoint(Runtime *rt,uint32_t matrixAt,uint32_t pointAt){
    float matrix[4][4],point[4];std::array<float,4> result;
    std::memcpy(matrix,record(rt,matrixAt,64),64);std::memcpy(point,record(rt,pointAt,16),16);
    for(auto &row:matrix)for(float value:row){pairQualified(value);if(std::abs(value)>1.e8f)throw std::runtime_error("unqualified native row-point matrix");}
    for(float value:point){pairQualified(value);if(std::abs(value)>1.e8f)throw std::runtime_error("unqualified native row-point input");}
    music::CodecRounding rounding;
    constexpr unsigned order[4][4]={{0,1,2,3},{1,0,2,3},{2,1,0,3},{3,1,2,0}};
    for(unsigned row=0;row<4;++row){float terms[4];for(unsigned i=0;i<4;++i)terms[i]=pairProduct(matrix[row][i],point[i]);
        result[row]=pairSum(pairSum(pairSum(terms[order[row][0]],terms[order[row][1]]),pairProduct(1.f,terms[order[row][2]])),pairProduct(1.f,terms[order[row][3]]));}
    return result;
}
float basisYScale(uint8_t *basis){
    float values[3];for(unsigned i=0;i<3;++i){std::memcpy(values+i,basis+i*16+4,4);pairQualified(values[i]);
        if(std::abs(values[i])>1.e8f)throw std::runtime_error("unqualified native vertical basis");}
    music::CodecRounding rounding;
    const auto squared=pairSum(pairSum(pairProduct(values[0],values[0]),pairProduct(values[1],values[1])),pairProduct(values[2],values[2]));
    volatile float length=std::sqrt(squared);return pairSum(0.f,length);
}
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
struct LeaflessQuery {uint8_t *node,*object,*query,*frame;};
LeaflessQuery leaflessQuery(Runtime *rt,R5900Context *c,bool entry=false){
    const auto stack=c->r[29].uw[0],nodeAt=c->r[18].uw[0],queryAt=c->r[17].uw[0];
    auto *node=record(rt,nodeAt,32),*query=record(rt,queryAt,32),*frame=record(rt,stack,0xf0);
    uint32_t child,objectAt;int16_t first;std::memcpy(&child,node,4);std::memcpy(&objectAt,node+4,4);std::memcpy(&first,node+14,2);
    // Populated nodes run their compiled triangle loop (leaf tests 1f6be0 /
    // 1f6e10 are native) and child recursion; the VF spills/restores here hold
    // no native state either way.
    (void)child;(void)first;
    auto *object=record(rt,objectAt,0x210);
    const auto apart=[](uint8_t *a,size_t an,uint8_t *b,size_t bn){const auto x=reinterpret_cast<uintptr_t>(a),y=reinterpret_cast<uintptr_t>(b);
        if(x<y+bn && y<x+an)throw std::runtime_error("native collision-query live records overlap");};
    apart(frame,0xf0,node,32);apart(frame,0xf0,query,32);apart(frame,0xf0,object,0x210);
    apart(node,32,object,0x210);apart(query,32,object,0x210);apart(node,32,query,32);
    if(entry){uint32_t caller=0;auto *saved=rt->memory.hostPointer(stack+0xf0,4);if(saved)std::memcpy(&caller,saved,4);
        if(c->r[31].uw[0]!=0x1f72f4 && (c->r[31].uw[0]!=0x1f7390 || (caller!=0x3e70bc && caller!=0x3e753c)))
            throw std::runtime_error("unqualified native leafless-query caller");}
    return {node,object,query,frame};
}
std::array<float,3> queryBasisScales(uint8_t *basis){
    float rows[3][4];std::memcpy(rows,basis,48);std::array<float,3> scales;
    music::CodecRounding rounding;
    for(unsigned axis=0;axis<3;++axis){float squares[3];for(unsigned row=0;row<3;++row){const auto value=rows[row][axis];pairQualified(value);
            if(std::abs(value)>1.e8f)throw std::runtime_error("unqualified native query basis");squares[row]=pairProduct(value,value);}
        const auto squared=pairSum(pairSum(squares[0],squares[1]),squares[2]);volatile float length=std::sqrt(squared);scales[axis]=pairSum(0.f,length);}
    return scales;
}
std::array<float,4> leaflessDistance(const LeaflessQuery &data){
    float columns[4][4],sphere[4],query[8],scale;std::memcpy(columns,data.object+0x150,64);std::memcpy(sphere,data.node+16,16);
    std::memcpy(query,data.query,32);std::memcpy(&scale,data.object+0x204,4);
    const auto qualified=[](float v){pairQualified(v);if(std::abs(v)>1.e8f)throw std::runtime_error("unqualified native query magnitude");};
    for(auto &column:columns)for(unsigned i=0;i<3;++i)qualified(column[i]);for(auto value:sphere)qualified(value);qualified(scale);
    for(unsigned i=0;i<3;++i){qualified(query[i]);qualified(query[4+i]);}
    // This is inline geometry, unlike the scale microprogram. Keep the caller's
    // rounding mode while leaving the host exception flags as they arrived.
    struct ArithmeticScope {unsigned saved=_mm_getcsr();~ArithmeticScope(){_mm_setcsr(saved);}} arithmetic;
    float difference[3],squared[3],dot[3];
    for(unsigned i=0;i<3;++i){auto center=pairProduct(columns[0][i],sphere[0]);
        for(unsigned column=1;column<4;++column)center=pairSum(center,pairProduct(columns[column][i],column==3?1.f:sphere[column]));
        difference[i]=pairSum(center,-query[i]);squared[i]=pairProduct(query[4+i],query[4+i]);dot[i]=pairProduct(difference[i],query[4+i]);}
    const auto sum3=[](const float values[3]){return pairSum(pairSum(pairProduct(1.f,values[0]),pairProduct(1.f,values[1])),pairProduct(1.f,values[2]));};
    const auto denominator=sum3(squared);volatile float reciprocal=denominator==0?std::numeric_limits<float>::max():1.f/denominator;
    const auto projection=sum3(dot),parameter=pairProduct(projection>0?projection:0.f,pairNormal(reciprocal));
    for(unsigned i=0;i<3;++i){const auto residual=pairSum(difference[i],-pairProduct(query[4+i],parameter));squared[i]=pairProduct(residual,residual);}
    const auto radius=pairProduct(sphere[3],scale),distance=pairSum(sum3(squared),-pairProduct(radius,radius));pairQualified(distance);
    return {sphere[0],sphere[1],sphere[2],distance};
}
struct SegmentTerms {std::array<float,3> delta;float firstLength,negativeCross,secondLength,firstProjection,separation,determinant,negativeSecondProjection,secondNumerator,firstNumerator;};
struct PointSegmentTerms {std::array<float,3> delta,direction;float projection,lengthSquared;};
struct PlaneContact {std::array<float,4> point;std::array<float,3> normal;float distance;};
PlaneContact planeContact(Runtime *rt,R5900Context *c){
    if(c->pc!=0x1fbfb0 || c->r[31].uw[0]!=0x1fe210)throw std::runtime_error("unqualified plane-contact caller");
    const auto stack=c->r[29].uw[0],owner=c->r[4].uw[0],pointAddress=c->r[17].uw[0];
    auto *frame=record(rt,stack,16),*plane=record(rt,owner,0x240),*point=record(rt,pointAddress,0x40),*output=record(rt,c->r[6].uw[0],16);
    uint32_t attached;std::memcpy(&attached,point+8,4);
    if(attached!=owner || c->r[5].uw[0]!=pointAddress+0x30 || c->r[6].uw[0]!=stack+0x1f0 || c->r[3].uw[0]!=owner+0x230 || c->r[2].uw[0]!=stack)
        throw std::runtime_error("plane-contact argument or temporary setup mismatch");
    separatePairFrame(frame,plane,0x240);separatePairFrame(frame,point,0x40);separatePairFrame(frame,output,16);
    const float margin=c->f[12];uint32_t marginBits;std::memcpy(&marginBits,point+0x18,4);
    if(marginBits!=std::bit_cast<uint32_t>(margin))throw std::runtime_error("plane-contact margin argument mismatch");
    pairQualified(margin);if(std::abs(margin)>1.e8f)throw std::runtime_error("unqualified plane-contact margin");
    PlaneContact result;float normal[4],origin[4],products[3];std::memcpy(result.point.data(),point+0x30,16);std::memcpy(normal,plane+0x220,16);std::memcpy(origin,plane+0x230,16);
    for(unsigned i=0;i<3;++i){for(float value:{result.point[i],normal[i],origin[i]}){pairQualified(value);if(std::abs(value)>1.e8f)throw std::runtime_error("unqualified plane-contact magnitude");}
        result.normal[i]=normal[i];products[i]=pairProduct(pairSum(result.point[i],-origin[i]),normal[i]);}
    result.distance=pairSum(pairSum(pairSum(products[0],products[1]),pairProduct(1.f,products[2])),-margin);return result;
}
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
// Callers in 1fd110: 1fdf68, 1fe024 (after the +90 virtual test) and 1fdc84
// (after the 1fc300 capsule pair); all pass a0=s2, a1=s1, a2=SP+210 and the
// parameter/depth that the test wrote to SP+4bc/4b8.
void collisionPositionResponse(Runtime *rt,R5900Context *c,uint32_t pc){
    if(c->pc!=0x1fc030 || (c->r[31].uw[0]!=0x1fdf68 && c->r[31].uw[0]!=0x1fe024 && c->r[31].uw[0]!=0x1fdc84))
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
void publishMagnitude(Runtime *rt,uint32_t source,uint32_t destination){
    std::array<float,4> out;std::memcpy(out.data(),record(rt,source,16),16);auto *to=record(rt,destination,16);
    for(unsigned i=0;i<3;++i){pairQualified(out[i]);if(std::abs(out[i])>1.e8f)throw std::runtime_error("unqualified native magnitude input");}
    struct ArithmeticScope {unsigned saved=_mm_getcsr();~ArithmeticScope(){_mm_setcsr(saved);}} arithmetic;
    for(unsigned i=0;i<3;++i)out[i]=pairProduct(out[i],out[i]);
    const auto squared=pairSum(pairSum(out[0],out[1]),pairProduct(1.f,out[2]));
    volatile float length=std::sqrt(squared);out[0]=pairSum(0.f,length);std::memcpy(to,out.data(),16);
}
}
void qualifySquadSines(Runtime *rt,R5900Context *c,uint32_t leafFrame){
    auto *dispatcher=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=dispatcher?dynamic_cast<SquadOwner *>(&dispatcher->host()):nullptr;
    if(!owner || !c)throw std::runtime_error("native squad has no publication owner");
    record(rt,leafFrame,0xc0);auto &calls=owner->nativeSquads().calls;
    const auto it=calls.find(leafFrame+0x50);
    if(it==calls.end() || it->second.phase>=3 || it->second.published)throw std::runtime_error("native squad helper has no live phase");
    const auto &call=it->second;const auto frame=leafFrame+0x50;
    constexpr uint32_t returns[]={0x214c74,0x214cb8,0x214cdc};uint32_t savedReturn;
    std::memcpy(&savedReturn,record(rt,leafFrame+0x30,16),4);
    const auto first=call.phase==2?frame+0x60:call.inputs[call.phase*2];
    const auto second=call.phase==2?frame+0x50:call.inputs[call.phase*2+1];
    if(savedReturn!=returns[call.phase] || c->r[17].uw[0]!=first || c->r[16].uw[0]!=second || c->r[18].uw[0]!=call.output)
        throw std::runtime_error("native squad helper argument/return mismatch");
}
bool nativeSquadInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    auto *dispatcher=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=dispatcher?dynamic_cast<SquadOwner *>(&dispatcher->host()):nullptr;
    if(!c || !owner)throw std::runtime_error("native squad has no publication owner");
    auto &calls=owner->nativeSquads().calls;const auto stack=c->r[29].uw[0];
    if(pc==0x214c40){
        if(c->pc!=0x214c40 || stack<0x70)throw std::runtime_error("native squad entry mismatch");
        const auto frame=stack-0x70;auto *saved=record(rt,frame-0x70,0xe0);
        NativeSquads::Call call{c->r[4].uw[0],{c->r[5].uw[0],c->r[6].uw[0],c->r[7].uw[0],c->r[8].uw[0]}};
        const auto separate=[&](uint8_t *p){const auto a=reinterpret_cast<uintptr_t>(p),b=reinterpret_cast<uintptr_t>(saved);
            if(a<b+0xc0 && b<a+16)throw std::runtime_error("native squad operands alias saved/helper frame");};
        separate(record(rt,call.output,16));for(auto input:call.inputs)separate(record(rt,input,16));
        for(const auto &[active,unused]:calls){const auto a=reinterpret_cast<uintptr_t>(record(rt,active-0x70,0xe0)),b=reinterpret_cast<uintptr_t>(saved);
            if(a<b+0xe0 && b<a+0xe0)throw std::runtime_error("native squad repeats/overlaps an active frame");}
        calls.emplace(frame,call);return false;
    }
    if(pc>=0x214db8){
        if(c->pc!=0x214d10 && c->pc!=0x214db8)throw std::runtime_error("native squad leaf entry mismatch");
        qualifySquadSines(rt,c,stack);auto &call=calls.at(stack+0x50);
        if(pc==0x214e1c){
            struct ArithmeticScope {unsigned saved=_mm_getcsr();~ArithmeticScope(){_mm_setcsr(saved);}} arithmetic;
            float weights[2],scale=1.f;
            if(c->pc==0x214db8){
                const auto sines=nativeThreeSines(rt,{c->f[12],c->f[13],c->f[14]});weights[0]=sines[0];weights[1]=sines[1];
                volatile float reciprocal=sines[2]==0?std::copysign(std::numeric_limits<float>::max(),sines[2]):1.f/sines[2];scale=pairNormal(reciprocal);
            }else{
                std::memcpy(weights,record(rt,stack+0x40,16),8);
                if(std::bit_cast<uint32_t>(weights[0])!=std::bit_cast<uint32_t>(c->f[0]) || std::bit_cast<uint32_t>(weights[1])!=std::bit_cast<uint32_t>(c->f[21]))
                    throw std::runtime_error("native squad scalar weight mismatch");
            }
            std::array<float,4> a,b;std::memcpy(a.data(),record(rt,c->r[17].uw[0],16),16);std::memcpy(b.data(),record(rt,c->r[16].uw[0],16),16);
            const auto destination=call.phase==0?stack+0xb0:call.phase==1?stack+0xa0:call.output;
            auto *out=record(rt,destination,16);const auto result=linearQuaternionBlend(a,b,weights[0],weights[1],false,scale);
            std::memcpy(out,result.data(),16);call.published=true;
        }return true;
    }
    const auto it=calls.find(stack);
    if(it==calls.end())throw std::runtime_error("native squad caller has no live publication");
    auto &call=it->second;
    if(pc==0x214ce8){
        if(c->pc!=0x214cdc || call.phase!=3 || call.published)throw std::runtime_error("native squad incomplete exit");
        calls.erase(it);return false;
    }
    constexpr uint32_t entries[]={0x214c74,0x214cb8,0x214cdc},starts[]={0x214c98,0x214cbc,0x214cdc},stores[]={0x214ca0,0x214cc4,0x214ce4};
    if(call.phase>=3 || !call.published || c->pc!=entries[call.phase] || pc<starts[call.phase] || pc>stores[call.phase])
        throw std::runtime_error("native squad publication order mismatch");
    const auto destination=call.phase==0?stack+0x60:call.phase==1?stack+0x50:call.output;
    if(c->r[call.phase==0?3:call.phase==1?2:18].uw[0]!=destination)throw std::runtime_error("native squad output mismatch");
    if(pc==stores[call.phase]){call.published=false;++call.phase;}
    return true;
}
bool nativeParticleCallbackInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    constexpr uint32_t sites[]={0x41074c,0x410750,0x410790,0x410794,0x4107a8,0x4107b0,0x4107b4};
    const auto site=std::find(std::begin(sites),std::end(sites),pc);if(site==std::end(sites))return false;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *updates=d?dynamic_cast<ParticleUpdateOwner *>(&d->host()):nullptr;auto *particles=d?dynamic_cast<ParticleOwner *>(&d->host()):nullptr;
    if(!c || !updates || !particles)throw std::runtime_error("native particle callback requires explicit integration owner");
    const auto stack=c->r[29].uw[0],parent=stack+0x80;auto &state=updates->nativeParticleUpdates();
    auto *frame=record(rt,stack,0x80);const auto word=[](const uint8_t *p){uint32_t value;std::memcpy(&value,p,4);return value;};
    const auto completed=state.completed.find(parent);
    if(completed==state.completed.end() || state.updates.contains(parent))throw std::runtime_error("native particle callback has no completed point");
    const auto &point=completed->second;auto *emitter=record(rt,point.emitter,0xe0);
    particles->nativeParticles().validateEmitter(parent+0x150,point.emitter);
    if(c->r[21].uw[0]!=point.emitter || c->r[19].uw[0]!=point.index || (word(record(rt,parent,0x150)+0xa0)!=0x22ed38 && word(record(rt,parent,0x150)+0xa0)!=0x22ef50))
        throw std::runtime_error("native particle callback escaped its integration frame");
    if(pc==sites[0]){
        const auto object=word(emitter+0x6c),table=word(record(rt,object,16));auto *method=rt->memory.hostPointer(table+0x98,4);
        const auto address=c->r[31].uw[0];const unsigned flags=emitter[0xd2];
        if(c->pc!=0x410640 || state.callbacks.contains(stack) || c->r[4].uw[0]!=point.emitter || !method || word(method)!=0x2262b0 ||
           (address!=0x40ac20 && address!=0x40acb0 && address!=0x40ad50 && address!=0x40ade4) || word(frame+0x20)!=address ||
           c->r[10].uw[0]!=(flags&1?stack+0x70:0) || c->r[5].uw[0]!=(flags&2?stack+0x60:0) || c->r[3].uw[0]!=(flags&4?stack+0x50:0) ||
           c->r[6].uw[0]!=point.point+0x30 || c->r[7].uw[0]!=point.point+0x40 || c->r[8].uw[0] || c->r[9].uw[0])
            throw std::runtime_error("unqualified native particle object-update callback");
        // The numeric saves served the old implicit integration registers.
        // Integration now owns explicit records/timestep/bounds, so only this
        // semantic call lifetime survives the original compiled callback.
        state.callbacks.emplace(stack,NativeParticleUpdates::Callback{point.emitter,point.point,point.index,object,address});return true;
    }
    auto call=state.callbacks.find(stack);
    if(call==state.callbacks.end() || call->second.phase!=unsigned(site-std::begin(sites)) || call->second.emitter!=point.emitter || call->second.point!=point.point || call->second.index!=point.index ||
       word(frame+0x20)!=call->second.returnAddress || word(emitter+0x6c)!=call->second.object || c->pc!=(pc==0x410750?0x410640:0x410790))
        throw std::runtime_error("native particle callback retirement order mismatch");
    if(pc==0x410750 && c->r[2].uw[0]!=stack+0x30)throw std::runtime_error("particle callback spill address mismatch");
    if(pc==0x410790 && c->r[31].uw[0]!=0x410790)throw std::runtime_error("particle callback did not return through original caller");
    if(pc==0x4107a8 && c->r[3].uw[0]!=word(record(rt,0x35dee0,16)+8))throw std::runtime_error("particle callback timestep source mismatch");
    if((pc==0x4107b0 || pc==0x4107b4) && c->r[3].uw[0]!=stack+0x30)throw std::runtime_error("particle callback restore address mismatch");
    if(pc==0x4107b4)state.callbacks.erase(call);else ++call->second.phase;return true;
}
bool nativeParticlePhysicsInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool recordStore=pc==0x40abb8 || pc==0x40abbc || pc==0x40abc0 || pc==0x40ac2c || pc==0x40ac34 || pc==0x40ac38 ||
        pc==0x40ace0 || pc==0x40ace4 || pc==0x40ace8 || pc==0x40ad58 || pc==0x40ad5c || pc==0x40ad60 || pc==0x40adf0 || pc==0x40adf4 || pc==0x40adf8;
    const bool calculation=pc==0x40ab24 || pc==0x40ab28 || pc==0x40ab2c || pc==0x40ab30 || pc==0x40ab34 || pc==0x40ab38 || pc==0x40ab3c || pc==0x40ab40 || pc==0x40ab44 || pc==0x40ab54 || pc==0x40ab58;
    const bool squaredAttribute=pc==0x40ac28 || pc==0x40ac30 || pc==0x40ac3c || pc==0x40ac40 || pc==0x40ac4c;
    const bool transformedPoint=(pc>=0x40acbc && pc<=0x40acd8 && !(pc&3)) || pc==0x40acec;
    if(!recordStore && !calculation && pc!=0x40a950 && pc!=0x40a958 && pc!=0x40aa30 && pc!=0x40aa38 && pc!=0x40aa40 && pc!=0x40aa48 &&
       pc!=0x40aad8 && pc!=0x40aae4 && pc!=0x40aae8 && pc!=0x40aaec && pc!=0x40ab5c && pc!=0x40b040 && !squaredAttribute && !transformedPoint)return false;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *updates=d?dynamic_cast<ParticleUpdateOwner *>(&d->host()):nullptr;auto *particles=d?dynamic_cast<ParticleOwner *>(&d->host()):nullptr;
    if(!c || !updates || !particles)throw std::runtime_error("native particle integration requires game update/bounds owners");
    const auto stack=c->r[29].uw[0],emitter=c->r[21].uw[0];auto *owner=record(rt,emitter,0xe0),*frame=record(rt,stack,0x150);
    const auto word=[](const uint8_t *p,unsigned at){uint32_t v;std::memcpy(&v,p+at,4);return v;};
    if(word(frame,0xa0)!=0x22ed38 && word(frame,0xa0)!=0x22ef50)throw std::runtime_error("native particle integration has no original producer return");
    particles->nativeParticles().validateEmitter(stack+0x150,emitter);auto &pending=updates->nativeParticleUpdates().updates;
    auto &completed=updates->nativeParticleUpdates().completed;
    if(pc==0x40b040){if(pending.contains(stack) || updates->nativeParticleUpdates().callbacks.contains(stack-0x80))throw std::runtime_error("native particle integration exits with an incomplete point/callback");completed.erase(stack);return true;}
    if(pc==0x40a950){if(c->r[2].uw[0]!=word(record(rt,0x35dee0,16),8))throw std::runtime_error("native particle integration timestep mismatch");return true;}
    if(pc==0x40a958){if(c->r[2].uw[0]!=emitter+0xc0)throw std::runtime_error("native particle drift source mismatch");return true;}
    if(squaredAttribute || transformedPoint){
        const auto index=c->r[19].uw[0],count=word(owner,0xc),pool=word(owner,0x14);
        if(!count || count>65536 || index>=count || owner[0xd0]!=(squaredAttribute?2:5))throw std::runtime_error("native particle lifetime branch mismatch");
        record(rt,pool,count*0x90);const auto point=pool+index*0x90;
        if(pc==0x40ac4c || pc==0x40acec){
            const auto destination=stack+(squaredAttribute?0x140:0x130);
            const auto done=completed.find(stack);
            if(c->r[2].uw[0]!=destination || c->r[17].uw[0]!=point+0x10 || c->r[8].uw[0]!=point+0x50 || pending.contains(stack) ||
               done==completed.end() || done->second.point!=point || done->second.emitter!=emitter || done->second.index!=index)
                throw std::runtime_error("native particle lifetime temporary mismatch");
            struct ArithmeticScope {unsigned saved=_mm_getcsr();~ArithmeticScope(){_mm_setcsr(saved);}} arithmetic;
            if(squaredAttribute){std::array<float,4> value;std::memcpy(value.data(),record(rt,point+0x50,16),16);
                for(unsigned i=0;i<3;++i){pairQualified(value[i]);if(std::abs(value[i])>1.e18f)throw std::runtime_error("unqualified native particle attribute square");value[i]=pairProduct(value[i],value[i]);}
                value[0]=pairSum(pairSum(value[0],value[1]),pairProduct(1.f,value[2]));std::memcpy(record(rt,destination,16),value.data(),16);
            }else {math::Matrix matrix;std::memcpy(matrix.data(),record(rt,emitter+0x80,64),64);std::array<float,4> pointValue;
                std::memcpy(pointValue.data(),record(rt,point+0x10,16),16);pointValue[3]=1.f;
                float values[16];std::memcpy(values,matrix.data(),64);for(float value:values){pairQualified(value);if(std::abs(value)>1.e8f)throw std::runtime_error("unqualified native particle lifetime matrix");}
                for(unsigned i=0;i<3;++i){pairQualified(pointValue[i]);if(std::abs(pointValue[i])>1.e22f)throw std::runtime_error("unqualified native particle lifetime point");}
                __m128 input;std::memcpy(&input,pointValue.data(),16);const auto result=math::matrixVectorProduct(matrix,input);std::memcpy(record(rt,destination,16),&result,16);
            }
        }return true;
    }
    if(pc==0x40aa30){
        const auto count=word(owner,0xc),index=c->r[19].uw[0],pool=word(owner,0x14),group=word(owner,0x10);
        if(!count || count>65536 || index>=count || !group || group>16 || c->r[18].uw[0]!=group || pending.contains(stack) || updates->nativeParticleUpdates().callbacks.contains(stack-0x80))throw std::runtime_error("native particle integration repeats/escapes its point");
        record(rt,pool,count*0x90);const auto point=pool+index*0x90,output=c->r[20].uw[0];auto *source=record(rt,point,0x90),*out=record(rt,output,group*48);
        if(c->r[6].uw[0]!=point || c->r[17].uw[0]!=point+0x10)throw std::runtime_error("native particle integration position source mismatch");
        const auto a=reinterpret_cast<uintptr_t>(source),b=reinterpret_cast<uintptr_t>(out),f=reinterpret_cast<uintptr_t>(frame),o=reinterpret_cast<uintptr_t>(owner);
        if((a<b+group*48 && b<a+0x90) || (b<f+0x150 && f<b+group*48) || (a<f+0x150 && f<a+0x90) || (b<o+0xe0 && o<b+group*48))
            throw std::runtime_error("native particle integration aliases live source/output/frame");
        NativeParticleUpdates::Update u{emitter,point,output,index};
        std::memcpy(u.position.data(),source+0x10,16);std::memcpy(u.rate.data(),source+0x30,16);std::memcpy(u.attributes.data(),source+0x50,16);
        std::memcpy(u.change.data(),source+0x40,16);std::memcpy(u.drift.data(),owner+0xc0,16);std::memcpy(&u.timestep,record(rt,0x35dee0,16)+8,4);
        const auto qualify=[](float v){pairQualified(v);if(std::abs(v)>1.e8f)throw std::runtime_error("unqualified native particle integration input");};
        qualify(u.timestep);for(unsigned i=0;i<3;++i){qualify(u.position[i]);qualify(u.rate[i]);qualify(u.attributes[i]);qualify(u.change[i]);qualify(u.drift[i]);}
        qualify(u.rate[3]);qualify(u.attributes[3]);pending.emplace(stack,u);completed.erase(stack);return true;
    }
    const auto it=pending.find(stack);if(it==pending.end() || it->second.emitter!=emitter || it->second.index!=c->r[19].uw[0])throw std::runtime_error("native particle integration has no matching point");
    auto &u=it->second;
    if(pc==0x40aaec){
        if(u.phase || c->r[3].uw[0]!=word(owner,0x68) || c->r[5].uw[0]>=65536)throw std::runtime_error("native particle curve source mismatch");
        const auto duration=c->f[1];pairQualified(duration);if(std::abs(duration)>1.e8f)throw std::runtime_error("unqualified native particle curve duration");
        std::array<float,4> curve;std::memcpy(curve.data(),record(rt,word(owner,0x68)+c->r[5].uw[0]*16,16),16);
        for(unsigned i=0;i<3;++i){pairQualified(curve[i]);if(std::abs(curve[i])>1.e8f)throw std::runtime_error("unqualified native particle curve value");}
        struct ArithmeticScope {unsigned saved=_mm_getcsr();~ArithmeticScope(){_mm_setcsr(saved);}} arithmetic;
        auto position=u.position;for(unsigned i=0;i<3;++i)position[i]=pairSum(pairSum(position[i],0.f),pairProduct(curve[i],duration));u.position=position;
    }else if(pc==0x40ab5c){
        if(u.phase || c->r[20].uw[0]!=u.output)throw std::runtime_error("native particle point publication mismatch");
        struct ArithmeticScope {unsigned saved=_mm_getcsr();~ArithmeticScope(){_mm_setcsr(saved);}} arithmetic;
        auto result=u;
        for(unsigned i=0;i<3;++i){result.position[i]=pairSum(pairSum(pairSum(u.position[i],0.f),pairProduct(u.drift[i],1.f)),pairProduct(u.rate[i],u.timestep));
            result.attributes[i]=pairSum(pairSum(u.attributes[i],0.f),pairProduct(u.rate[i],u.timestep));
            result.rate[i]=pairSum(pairSum(u.rate[i],0.f),pairProduct(u.change[i],u.timestep));}
        result.rate[3]=pairSum(u.rate[3],-u.timestep);result.attributes[3]=pairSum(u.attributes[3],u.timestep);
        particles->nativeParticles().includePoint(stack+0x150,emitter,result.position);
        std::memcpy(record(rt,u.output,16),result.position.data(),16);result.phase=1;u=result;
    }else if(recordStore){
        const unsigned phase=(pc==0x40abb8 || pc==0x40ac2c || pc==0x40ace0 || pc==0x40ad58 || pc==0x40adf0)?1:
            (pc==0x40abbc || pc==0x40ac34 || pc==0x40ace4 || pc==0x40ad5c || pc==0x40adf4)?2:3;
        const auto address=u.point+(phase==1?0x50:phase==2?0x10:0x30);const unsigned reg=phase==1?8:phase==2?17:16;
        if(u.phase!=phase || c->r[reg].uw[0]!=address)throw std::runtime_error("native particle feedback publication order mismatch");
        const auto &value=phase==1?u.attributes:phase==2?u.position:u.rate;std::memcpy(record(rt,address,16),value.data(),16);
        if(phase==3){completed[stack]={u.emitter,u.point,u.index};pending.erase(it);}else ++u.phase;
    }else if(u.phase)throw std::runtime_error("native particle calculation after publication");
    return true;
}
void NativeMotion::prepare(uint32_t frame,uint32_t object,std::array<uint32_t,4> position){
    if(!updates.emplace(frame,Update{object,position}).second)throw std::runtime_error("native motion update repeats its frame");
}
std::array<uint32_t,4> NativeMotion::finish(uint32_t frame,uint32_t object){
    auto it=updates.find(frame);if(it==updates.end() || it->second.object!=object)throw std::runtime_error("native motion publication has no matching update");
    const auto result=it->second.position;updates.erase(it);return result;
}
// Object pair overlap, main 1f8aa0. Its VU0 register spills/restores hold no
// native state (the SP+a0 area has no EE reader). Two common-math 0x7f0 calls
// publish each object's per-axis basis lengths; the transform block computes
// ((dx*dx+dy*dy)-r*r)+dz*dz between the two transformed sphere centres.
// VU0 mul/add/sqrt round to nearest even (engine spec Q1/Q3/Q10).
float vu0Input(float v){const auto bits=std::bit_cast<uint32_t>(v);
    if((bits&0x7f800000)==0x7f800000 || beyondRange(v))throw std::runtime_error("unqualified native object overlap input");
    return pairNormal(v);}
bool nativeObjectOverlap(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool spill=pc>=0x1f8b34 && pc<=0x1f8b54,restore=(pc>=0x1f8e70 && pc<=0x1f8e90) || (pc>=0x1f8fec && pc<=0x1f900c)
        || (pc>=0x1f907c && pc<=0x1f909c) || (pc>=0x1f90dc && pc<=0x1f90fc);
    const bool firstScale=pc==0x1f8d00 || pc==0x1f8d04 || pc==0x1f8d08 || pc==0x1f8d10 || pc==0x1f8d14 || pc==0x1f8d1c;
    const bool secondScale=pc==0x1f8d78 || pc==0x1f8d7c || pc==0x1f8d80 || pc==0x1f8d88 || pc==0x1f8d8c || pc==0x1f8d94;
    const bool overlap=pc>=0x1f8ddc && pc<=0x1f8e58 && (pc&3)==0 && pc!=0x1f8e54;
    if(!(spill || restore || firstScale || secondScale || overlap) || (pc&3))return false;
    if(!c)throw std::runtime_error("native object overlap has no CPU record");
    const auto stack=c->r[29].uw[0],first=c->r[18].uw[0],second=c->r[17].uw[0];
    if(spill || restore){
        if(c->r[2].uw[0]!=stack+0xa0)throw std::runtime_error("native object overlap spill escaped its frame");
        return true;
    }
    if(firstScale || secondScale){
        const auto object=firstScale?first:second;const auto out=stack+(firstScale?0x70:0x60);
        if(c->r[5].uw[0]!=object+0x90)throw std::runtime_error("native object scale basis mismatch");
        if(pc==0x1f8d14 || pc==0x1f8d8c){c->r[9].ud[0]=0;return true;}
        if(pc!=0x1f8d1c && pc!=0x1f8d94)return true;
        if(c->r[2].uw[0]!=out)throw std::runtime_error("native object scale destination mismatch");
        float rows[3][4];std::memcpy(rows,record(rt,object+0x90,48),48);
        for(auto &row:rows)for(unsigned i=0;i<3;++i)row[i]=vu0Input(row[i]);
        float lengths[3];
        {music::NearestRounding rounding;
            for(unsigned i=0;i<3;++i){
                auto squared=pairSum(pairSum(pairProduct(rows[0][i],rows[0][i]),pairProduct(rows[1][i],rows[1][i])),pairProduct(rows[2][i],rows[2][i]));
                volatile float root=std::sqrt(squared);lengths[i]=pairSum(0.f,root);
            }}
        // The micro program leaves W as inherited register state; no reader.
        std::memcpy(record(rt,out,12),lengths,12);return true;
    }
    if(c->r[4].uw[0]!=c->r[20].uw[0]+0x10 || c->r[3].uw[0]!=c->r[19].uw[0]+0x10 || c->r[6].uw[0]!=first+0x150 || c->r[5].uw[0]!=second+0x150)
        throw std::runtime_error("native object overlap escaped its records");
    if(pc!=0x1f8e58){if(pc<0x1f8e50 && c->r[2].uw[0]!=stack+0x90)throw std::runtime_error("native object overlap scale record mismatch");return true;}
    if(c->r[2].uw[0]!=stack+0x80)throw std::runtime_error("native object overlap destination mismatch");
    float scale[4],a[4],b[4],m1[4][4],m2[4][4];
    std::memcpy(scale,record(rt,stack+0x90,16),16);std::memcpy(a,record(rt,c->r[4].uw[0],16),16);std::memcpy(b,record(rt,c->r[3].uw[0],16),16);
    std::memcpy(m1,record(rt,first+0x150,64),64);std::memcpy(m2,record(rt,second+0x150,64),64);
    // Only the lanes the original consumes: scales X/Y, both centres, all of
    // the first transform (its W is published) and the second's XYZ columns.
    for(unsigned i=0;i<2;++i)scale[i]=vu0Input(scale[i]);for(float &v:a)v=vu0Input(v);for(float &v:b)v=vu0Input(v);
    for(auto &row:m1)for(float &v:row)v=vu0Input(v);for(auto &row:m2)for(unsigned i=0;i<3;++i)row[i]=vu0Input(row[i]);
    float result[4];
    {music::NearestRounding rounding;
        const auto transform=[](const float (&m)[4][4],const float (&p)[4],float (&out)[4],unsigned lanes){
            for(unsigned i=0;i<lanes;++i)out[i]=pairSum(pairSum(pairSum(pairProduct(m[0][i],p[0]),pairProduct(m[1][i],p[1])),pairProduct(m[2][i],p[2])),pairProduct(m[3][i],1.f));};
        float t1[4],t2[4]{};transform(m1,a,t1,4);transform(m2,b,t2,3);
        const auto radius=pairSum(pairProduct(a[3],scale[0]),pairProduct(b[3],scale[1]));const auto radius2=pairProduct(radius,radius);
        float squared[3];for(unsigned i=0;i<3;++i){const auto d=pairSum(t1[i],-t2[i]);squared[i]=pairProduct(d,d);}
        auto total=pairSum(pairProduct(1.f,squared[0]),pairProduct(1.f,squared[1]));
        total=pairSum(total,-pairProduct(1.f,radius2));total=pairSum(total,pairProduct(1.f,squared[2]));
        for(float v:{total,t1[3]})if(!std::isfinite(v))throw std::runtime_error("native object overlap overflow");
        result[0]=total;result[1]=squared[1];result[2]=squared[2];result[3]=t1[3];
    }
    std::memcpy(record(rt,stack+0x80,16),result,16);return true;
}
// Camera point projection, mod001 209aa0. Common-math 0x1f0 loads the camera
// matrix at s2+330 as columns (0+x canonical lanes, diagonal moves, 1*x for
// the fourth row); 0x0 then projects a stack point: p = M*v, Q = 1/p.w,
// result (p.x*Q, p.y*Q, p.z*Q, Q). Between the load and each projection the
// original writes only its stack frame and s0's +3e8c/+3ecc table, so the
// native projection rereads the matrix record; overlap with those is refused.
bool nativeCameraProjection(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool loadA=(pc>=0x209bd0 && pc<=0x209bdc) || pc==0x209be8,loadB=(pc>=0x20a00c && pc<=0x20a018) || pc==0x20a024;
    const bool first=pc==0x209c80 || pc==0x209c8c || pc==0x209c90 || pc==0x209c94;
    const bool second=pc==0x209d00 || pc==0x209d0c || pc==0x209d10 || pc==0x209d14;
    const bool third=pc==0x20a06c || pc==0x20a078 || pc==0x20a07c || pc==0x20a084;
    if(!(loadA || loadB || first || second || third) || (pc&3))return false;
    if(!c || !c->r[18].uw[0])throw std::runtime_error("native camera projection has no camera record");
    const auto stack=c->r[29].uw[0],matrixAt=c->r[18].uw[0]+0x330;
    if(loadA || loadB){if(c->r[2].uw[0]!=matrixAt)throw std::runtime_error("native camera matrix source mismatch");return true;}
    const uint32_t point=stack+(first?0x80:second?0x70:0x60);
    const unsigned pointReg=first?2:second?10:2,outReg=first?11:second?8:2;
    if((pc==0x209c80 || pc==0x209d00 || pc==0x20a06c) && c->r[pointReg].uw[0]!=point)throw std::runtime_error("native camera projection source mismatch");
    if(pc==0x209c90 || pc==0x209d10 || pc==0x20a07c){c->r[9].ud[0]=0;return true;}
    if(pc!=0x209c94 && pc!=0x209d14 && pc!=0x20a084)return true;
    if(c->r[outReg].uw[0]!=point)throw std::runtime_error("native camera projection destination mismatch");
    const auto overlaps=[](uint32_t a,uint32_t an,uint32_t b,uint32_t bn){return (a&0x1fffffff)<(b&0x1fffffff)+bn && (b&0x1fffffff)<(a&0x1fffffff)+an;};
    if(overlaps(matrixAt,0x40,stack+0x60,0x30))throw std::runtime_error("native camera matrix aliases the projection frame");
    if(!third){uint32_t table;std::memcpy(&table,record(rt,c->r[16].uw[0]+0x80,16)+0xc,4);
        if(overlaps(matrixAt,0x40,table+0x3e8c,0x40+0x200))throw std::runtime_error("native camera matrix aliases the screen table");}
    float m[4][4],v[4];std::memcpy(m,record(rt,matrixAt,64),64);std::memcpy(v,record(rt,point,16),16);
    for(auto &row:m)for(float &x:row)x=vu0Input(x);for(float &x:v)x=vu0Input(x);
    float result[4];
    {music::NearestRounding rounding;
        float p[4];
        for(unsigned i=0;i<4;++i){
            // Column load: lane i of each column is row i, canonicalized as
            // 0x1f0 does (0+x except the diagonal move and the 1*x W row).
            float column[4];for(unsigned k=0;k<4;++k)column[k]=k==i?m[i][k]:i==3?pairProduct(1.f,m[3][k]):pairSum(0.f,m[i][k]);
            p[i]=pairSum(pairSum(pairSum(pairProduct(column[0],v[0]),pairProduct(column[1],v[1])),pairProduct(column[2],v[2])),pairProduct(column[3],v[3]));
        }
        if((std::bit_cast<uint32_t>(p[3])&0x7fffffff)==0)throw std::runtime_error("native camera projection has zero depth");
        volatile float q=1.f/p[3];const auto reciprocal=pairNormal(q);
        for(unsigned i=0;i<3;++i)result[i]=pairProduct(p[i],reciprocal);result[3]=pairProduct(1.f,reciprocal);
        for(float x:result)if(!std::isfinite(x))throw std::runtime_error("native camera projection overflow");
    }
    std::memcpy(record(rt,point,16),result,16);return true;
}
// Object matrix composition, mod003 39f0d0: when s0+290 bit 0 (B = s0+210)
// and/or bit 1 (B = s0+250, the rotation 39f300 publishes) is set, the
// object's matrices at s0+90 and s0+d0 become row*B. Each row lane is
// ((B0*r.x + B1*r.y) + B2*r.z) + B3*r.w, VU0 mul/madd nearest even. B stays
// in registers between the two products; natively it is reread (B never
// overlaps the products' records, all at fixed offsets of the same object).
bool nativeObjectCompose(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc<0x39f11c || pc>0x39f2d8 || (pc&3) || (pc>=0x39f1f0 && pc<0x39f208) || pc==0x39f18c || pc==0x39f278)return false;
    if(!c || !c->r[16].uw[0])throw std::runtime_error("native object composition has no object");
    const auto object=c->r[16].uw[0];const bool second=pc>=0x39f208;
    const bool lower=second?pc>=0x39f27c:pc>=0x39f190;   // product into s0+d0
    const uint32_t b=object+(second?0x250:0x210),target=object+(lower?0xd0:0x90);
    if(c->r[lower?3:4].uw[0]!=target || (!lower && c->r[3].uw[0]!=b) || (lower && c->r[4].uw[0]!=object+0x90))
        throw std::runtime_error("native object composition escaped its records");
    const uint32_t store=lower?(second?0x39f2cc:0x39f1e0):(second?0x39f268:0x39f17c);
    if(pc!=store)return true;
    float m[4][4],rows[4][4],out[4][4];std::memcpy(m,record(rt,b,64),64);std::memcpy(rows,record(rt,target,64),64);
    for(auto &r:m)for(float &v:r)v=vu0Input(v);for(auto &r:rows)for(float &v:r)v=vu0Input(v);
    {music::NearestRounding rounding;
        for(unsigned k=0;k<4;++k)for(unsigned i=0;i<4;++i){
            out[k][i]=pairSum(pairSum(pairSum(pairProduct(m[0][i],rows[k][0]),pairProduct(m[1][i],rows[k][1])),pairProduct(m[2][i],rows[k][2])),pairProduct(m[3][i],rows[k][3]));
            if(!std::isfinite(out[k][i]))throw std::runtime_error("native object composition overflow");}
    }
    std::memcpy(record(rt,target,64),out,64);return true;
}
// Segment/sphere contact normal, mod001 1fc4b0. The CPU found the segment
// parameter t (SP+6c, also moved to v0); the VU0 block forms
// d = (0+A + D*t) - P from the segment record at s3+230 (A, D at +10) and
// the sphere centre at s4+230, then publishes d * (1/sqrt(|d|^2)) to the
// caller's normal record (a2, kept in s2). VU0 mul/add/sqrt/div round to
// nearest even. The original's 128-bit store leaves W as inherited register
// state; natively W is untouched. The interleaved CPU stores to s0/s1 must
// not alias the inputs (checked).
bool nativeSegmentContact(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc<0x1fc51c || pc>0x1fc570 || (pc&3))return false;
    if(pc==0x1fc524 || pc==0x1fc528 || (pc>0x1fc550 && pc<0x1fc568))return false;
    if(!c)throw std::runtime_error("native segment contact has no CPU record");
    const auto segment=c->r[19].uw[0]+0x230,sphere=c->r[20].uw[0]+0x230;
    if(c->r[4].uw[0]!=segment || c->r[3].uw[0]!=sphere || (pc==0x1fc520 && c->r[2].uw[0]!=segment+0x10))
        throw std::runtime_error("native segment contact escaped its records");
    if(pc!=0x1fc570)return true;
    const auto normalAt=c->r[18].uw[0]&~15u;   // 128-bit stores ignore the low address bits
    const auto overlaps=[](uint32_t a,uint32_t an,uint32_t b,uint32_t bn){return (a&0x1fffffff)<(b&0x1fffffff)+bn && (b&0x1fffffff)<(a&0x1fffffff)+an;};
    for(uint32_t scalar:{c->r[16].uw[0],c->r[17].uw[0]})
        if(overlaps(scalar,4,segment,0x20) || overlaps(scalar,4,sphere,0x10))throw std::runtime_error("native segment contact inputs alias its scalar outputs");
    float a[4],d[4],p[4];std::memcpy(a,record(rt,segment,16),16);std::memcpy(d,record(rt,segment+0x10,16),16);std::memcpy(p,record(rt,sphere,16),16);
    const float t=vu0Input(std::bit_cast<float>(c->r[2].uw[0]));
    for(unsigned i=0;i<3;++i){a[i]=vu0Input(a[i]);d[i]=vu0Input(d[i]);p[i]=vu0Input(p[i]);}
    float normal[3];
    {music::NearestRounding rounding;
        float delta[3],squared[3];
        for(unsigned i=0;i<3;++i){delta[i]=pairSum(pairSum(pairSum(0.f,a[i]),pairProduct(d[i],t)),-pairProduct(p[i],1.f));squared[i]=pairProduct(delta[i],delta[i]);}
        const auto length2=pairSum(pairSum(squared[0],squared[1]),pairProduct(1.f,squared[2]));
        if(!std::bit_cast<uint32_t>(length2) || !std::isfinite(length2))throw std::runtime_error("unqualified native segment contact length");
        volatile float root=std::sqrt(length2);volatile float reciprocal=1.f/pairNormal(root);
        for(unsigned i=0;i<3;++i)normal[i]=pairProduct(delta[i],pairNormal(reciprocal));
    }
    std::memcpy(record(rt,normalAt,16),normal,12);return true;
}
// Animated UVs of a model's VU1 packet, mod001 2188d0. The original builds a
// scratchpad transfer each frame (loader 219620) and lets UV-scroll elements
// (class 1591b0, callback 29cfe0 with mode-1 micro program 0xc80) rewrite the
// copied UV words. Natively the same result is computed from the records the
// loader reads: the model iterator *(*(o+54)+644) walks source mesh records M
// and packet records P in step; mesh j (bit j of *(o+48), or all when o+8d
// bit 2) has N = s16 M+16 UV words at offset u16 M+1c. Destination P+off gets
// the source M+off quadwords (N rounded up to 4 words), then each element
// (list at SP+e0, count s5) whose group bit is set (e+19, or bit j of
// *(e+24)) takes its next descriptor (cursor from e+34, size at +8): for each
// listed index, uv = ftoi12(itof12(uv) + scroll) per half, scroll at e+40/44.
uint32_t uvWord(Runtime *rt,uint32_t at){
    if((at&3) || (at&0x1ffffff)<0x100000 || (at>>28)==7)throw std::runtime_error("native UV animation record outside EE RAM");
    const auto *p=rt->memory.hostPointer(at,4);if(!p)throw std::runtime_error("native UV animation record unmapped");uint32_t v;std::memcpy(&v,p,4);return v;}
uint8_t *uvBytes(Runtime *rt,uint32_t at,uint32_t bytes){
    if((at&15) || (at&0x1ffffff)<0x100000 || (at>>28)==7 || bytes>0x10000)throw std::runtime_error("native UV animation span outside EE RAM");
    auto *p=rt->memory.hostPointer(at,bytes);if(!p)throw std::runtime_error("native UV animation span unmapped");return p;}
int32_t uvScrolled(uint32_t half,float scroll){
    // itof12 (exact), VU add (nearest even, denormals as zero), ftoi12 (truncate, saturate).
    volatile float value=float(half)*(1.f/4096.f)+pairNormal(scroll);const double fixed=double(pairNormal(value))*4096.;
    if(fixed>=2147483647.)return 0x7fffffff;if(fixed<=-2147483648.)return int32_t(0x80000000u);return int32_t(fixed);}
void skinMesh(Runtime *rt,uint32_t table,unsigned vertices,uint8_t *positions,uint8_t *normals,uint8_t *packed);
bool nativeUvAnimation(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x219210)return false;
    if(!c)throw std::runtime_error("native UV animation has no CPU record");
    const auto object=c->r[23].uw[0];
    const auto halfAt=object+0x84;const int16_t chunks=int16_t(uvWord(rt,halfAt&~3u)>>((halfAt&2)*8));
    if(chunks>0){
        const auto flags=uvWord(rt,object+0x58);
        // 1 positions, 2 normals, 8 UVs, 80 packed vectors, 20 element data;
        // 40 is only part of the skeleton's flag word (e3). 4, 10, 100 reject.
        if(!(flags&0x20) || (flags&~0xebu)){
            // Report the element classes: the next port step depends on them.
            std::string list;const auto n=c->r[21].uw[0],at=c->r[29].uw[0]+0xe0;
            for(unsigned i=0;i<n && i<16;++i){const auto e=uvWord(rt,at+4*i),vt=uvWord(rt,e);list+=std::format(" e{:x}:vt{:x}/+10={:x}",e,vt,uvWord(rt,vt+0x10));}
            throw std::runtime_error(std::format("unported animated vertex attribute set {:x} object={:x} elements{}",flags,object,list));}
        const auto iterator=uvWord(rt,uvWord(rt,object+0x54)+0x644),data=uvWord(rt,iterator+8);
        if(!data)throw std::runtime_error("native UV animation model has no mesh data");
        const bool unaligned=(uvWord(rt,iterator+0x1c)&0xff)!=0;const uint32_t base=unaligned?data:((data+0x7f)&~0x7fu);
        const int32_t meshes=int32_t(uvWord(rt,base+0x10+0x34));
        const auto packets=uvWord(rt,iterator+0x14);uint32_t source=base+0x70,packet=packets?packets+0x60:base+0x70;
        const bool all=(uvWord(rt,(object+0x8d)&~3u)>>(((object+0x8d)&3)*8)&4)!=0;const auto mask=uvWord(rt,object+0x48);
        const auto count=c->r[21].uw[0],list=c->r[29].uw[0]+0xe0;
        if(count>16)throw std::runtime_error("native UV animation element list outside its frame");
        // Skeleton elements (interface 154690: +10 is 218220, +18 is 2372f0)
        // skin positions/normals/packed vectors; their table for each included
        // mesh is the +18 cursor, which 237320 resets to +44 and each included
        // mesh advances by its leading halfword.
        struct Element{uint32_t at,cursor;float scroll[2];bool all;uint32_t bits;bool skeleton;};std::vector<Element> elements;
        for(unsigned i=0;i<count;++i){const auto e=uvWord(rt,list+4*i);const auto vt=uvWord(rt,e);
            if(vt==0x154690){
                if(uvWord(rt,vt+0x10)!=0x218220 || uvWord(rt,vt+0x18)!=0x2372f0)throw std::runtime_error("unqualified skeleton element methods");
                if((flags&3)!=3)throw std::runtime_error("skeleton element without streamed positions and normals");
                elements.push_back({e,uvWord(rt,e+0x44),{0,0},(uvWord(rt,e+0x18)>>8&0xff)!=0,uvWord(rt,e+0x24),true});continue;}
            if(vt!=0x1591b0)throw std::runtime_error("unported animated vertex element class");
            Element el{e,uvWord(rt,e+0x34),{std::bit_cast<float>(uvWord(rt,e+0x40)),std::bit_cast<float>(uvWord(rt,e+0x44))},(uvWord(rt,e+0x18)>>8&0xff)!=0,uvWord(rt,e+0x24),false};
            for(float s:el.scroll)if((std::bit_cast<uint32_t>(s)&0x7f800000)==0x7f800000)throw std::runtime_error("unqualified UV scroll");
            elements.push_back(el);}
        // The loader numbers element data by the object's element chain
        // (+d or +18 set); 218220 indexes it by this list. They must agree.
        {unsigned i=0;for(auto e=uvWord(rt,object+8);e!=object;e=uvWord(rt,e+8)){
            const auto bytes=uvWord(rt,e+0xc),flagWord=uvWord(rt,e+0x18);
            if(!((bytes>>8)&0xff) && !(flagWord&0xff))continue;
            if(i>=elements.size() || elements[i].at!=e)throw std::runtime_error("animated element list differs from the object's chain");
            if(++i>16)throw std::runtime_error("animated element chain outside bounds");}
         if(i!=elements.size())throw std::runtime_error("animated element list differs from the object's chain");}
        music::NearestRounding rounding;
        for(int32_t j=0;j<meshes;++j){
            const auto sourceStep=int16_t(uvWord(rt,source+4)),packetStep=int16_t(uvWord(rt,packet+4));
            // 25cb00/25cb30 return 0 for a record whose step is 0: never inside the count.
            if(!sourceStep || !packetStep)throw std::runtime_error("native UV animation mesh list ends early");
            const bool included=all || (uvWord(rt,mask+4*(uint32_t(j)>>5))>>(j&31)&1);
            if(included){
                const int32_t n=int16_t(uvWord(rt,source+0x14)>>16);const uint32_t offset=uvWord(rt,source+0x1c)&0xffff;
                if(n<0 || n>0x400)throw std::runtime_error("native UV animation mesh count outside bounds");
                const uint32_t qwords=flags&8?(uint32_t(n)+3)>>2:0;
                std::vector<uint32_t> words(qwords*4);if(qwords)std::memcpy(words.data(),uvBytes(rt,source+offset,qwords*16),qwords*16);
                // Streamed positions/normals (N qwords at u16 +18/+1a) and packed
                // vectors (N*8 bytes, rounded to qwords, at +34 & 3fff when its mode
                // bits 14-15 are 0), returned to the same packet offsets.
                const uint32_t positionAt=uvWord(rt,source+0x18)&0xffff,normalAt=uvWord(rt,source+0x18)>>16,packedHalf=uvWord(rt,source+0x34)&0xffff;
                const bool hasPacked=(flags&0x80) && packedHalf && !(packedHalf>>14);const uint32_t packedAt=packedHalf&0x3fff,packedBytes=((uint32_t(n)+1)>>1)*16;
                std::vector<uint8_t> positions(flags&1?16*n:0),normals(flags&2?16*n:0),packedData(hasPacked?packedBytes:0);
                if(n && (flags&1))std::memcpy(positions.data(),uvBytes(rt,source+positionAt,16*n),16*n);
                if(n && (flags&2))std::memcpy(normals.data(),uvBytes(rt,source+normalAt,16*n),16*n);
                if(n && hasPacked)std::memcpy(packedData.data(),uvBytes(rt,source+packedAt,packedBytes),packedBytes);
                for(auto &el:elements){
                    if(!el.skeleton)continue;
                    const auto table=el.cursor;if(table&1)throw std::runtime_error("skeleton element table misaligned");el.cursor+=uvWord(rt,table&~3u)>>((table&2)*8)&0xffff;
                    if(!n || (!el.all && !(uvWord(rt,el.bits+4*(uint32_t(j)>>5))>>(j&31)&1)))continue;
                    skinMesh(rt,table,unsigned(n),positions.data(),normals.data(),hasPacked?packedData.data():nullptr);
                }
                if(n && (flags&1))std::memcpy(uvBytes(rt,packet+positionAt,16*n),positions.data(),16*n);
                if(n && (flags&2))std::memcpy(uvBytes(rt,packet+normalAt,16*n),normals.data(),16*n);
                if(n && hasPacked)std::memcpy(uvBytes(rt,packet+packedAt,packedBytes),packedData.data(),packedBytes);
                if(n)for(auto &el:elements){
                    if(el.skeleton)continue;
                    if(!el.all && !(uvWord(rt,el.bits+4*(uint32_t(j)>>5))>>(j&31)&1))continue;
                    const auto descriptor=el.cursor;el.cursor+=uvWord(rt,descriptor+8);
                    const auto listed=int32_t(uvWord(rt,descriptor));// A zero count makes 29cfe0 read one index past the descriptor, from
                    // whatever its scratchpad copy holds there: not reproducible from records.
                    if(listed<1 || listed>256)throw std::runtime_error("native UV descriptor count outside bounds");
                    for(int32_t k=0;k<listed;++k){const auto index=(uvWord(rt,(descriptor+0x10+k)&~3u)>>(((descriptor+0x10+k)&3)*8))&0xff;
                        if(index>=words.size())throw std::runtime_error("native UV descriptor index outside its mesh");
                        auto &w=words[index];w=(uint32_t(uvScrolled(w>>16,el.scroll[1]))<<16)|(uint32_t(uvScrolled(w&0xffff,el.scroll[0]))&0xffff);}
                }
                if(qwords)std::memcpy(uvBytes(rt,packet+offset,qwords*16),words.data(),qwords*16);
            }
            source+=sourceStep;packet+=packetStep;
        }
    }
    // The replaced instruction: lh v0,0x84(s7).
    c->r[2].sd[0]=chunks;return true;
}
// Vector length sites: one VU0 macro sequence (VMUL.xyz, VADDA.x, VMADD.x,
// VSQRT, VADD.x 0+Q) publishing {|xyz|, y*y, z*z, w} from a scalar-built
// record. Each block's source/destination registers are checked against the
// record they must hold. mul, add and sqrt round to nearest even.
// sourceStack: source = SP+offset; else source = s0+offset, or (argument
// records, offset ~0u) whatever the source register points to.
struct LengthSite{uint32_t first;unsigned sourceReg;bool sourceStack;uint32_t sourceOffset;unsigned destinationReg;uint32_t destinationOffset;uint32_t returnAddress;};
constexpr LengthSite lengthSites[]={
    {0x380664,4,true,0x270,3,0xa0,0x3805f0},   // movement step, first pass
    {0x380844,4,true,0x270,3,0x70,0x3807d0},   // movement step, second pass
    {0x439258,5,true,0x100,4,0x90,0},          // mod003 439180 update
    {0x439880,5,true,0x100,4,0x80,0},
    {0x439ba8,4,false,0x1f0,3,0x70,0},         // s0 records
    {0x439bd8,3,false,0x230,2,0x60,0},
    {0x3cdc84,19,false,~0u,2,0xa0,0}};         // mod003 3cdb60: argument record (s3)
bool nativeLengthSite(R5900Context *c,Runtime *rt,uint32_t pc){
    const LengthSite *site=nullptr;
    for(const auto &s:lengthSites)if(pc>=s.first && pc<=s.first+0x20 && !(pc&3)){site=&s;break;}
    if(!site)return false;
    if(!c)throw std::runtime_error("native vector length has no CPU record");
    const bool argument=!site->sourceStack && site->sourceOffset==~0u;
    const auto stack=c->r[29].uw[0],base=site->sourceStack?stack:argument?c->r[site->sourceReg].uw[0]:c->r[17].uw[0];
    const auto sourceAt=argument?base:base+site->sourceOffset;
    if((!site->sourceStack && base<0x100000) || (site->returnAddress && c->r[31].uw[0]!=site->returnAddress) ||
       c->r[site->sourceReg].uw[0]!=sourceAt || c->r[site->destinationReg].uw[0]!=stack+site->destinationOffset)
        throw std::runtime_error("native vector length escaped its scalar records");
    if(pc!=site->first+0x20)return true;
    float value[4],result[4];std::memcpy(value,record(rt,sourceAt,16),16);std::memcpy(result,value,16);
    for(unsigned i=0;i<3;++i){if((std::bit_cast<uint32_t>(value[i])&0x7f800000)==0x7f800000 || beyondRange(value[i]))
        throw std::runtime_error("unqualified native vector length");value[i]=pairNormal(value[i]);}
    {music::NearestRounding rounding;
        for(unsigned i=0;i<3;++i){volatile float square=value[i]*value[i];result[i]=pairNormal(square);}
        volatile float partial=result[0]+result[1];volatile float squared=pairNormal(partial)+result[2];
        volatile float length=std::sqrt(pairNormal(squared));volatile float output=0.f+length;result[0]=output;}
    std::memcpy(record(rt,stack+site->destinationOffset,16),result,16);return true;
}
// mod003 439c54..439c68: dot product of s0+1f0 and s0+230 into SP+50, as
// VMUL.xyz then two VADD.x; the stored register keeps the first input's YZW.
bool nativeRecordDot(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc<0x439c54 || pc>0x439c68 || (pc&3))return false;
    if(!c || c->r[17].uw[0]<0x100000)throw std::runtime_error("native record dot has no owner");
    const auto owner=c->r[17].uw[0],stack=c->r[29].uw[0];
    if(c->r[4].uw[0]!=owner+0x1f0 || c->r[3].uw[0]!=owner+0x230 || c->r[2].uw[0]!=stack+0x50)throw std::runtime_error("native record dot escaped its records");
    if(pc!=0x439c68)return true;
    float a[4],b[4],out[4];std::memcpy(a,record(rt,owner+0x1f0,16),16);std::memcpy(b,record(rt,owner+0x230,16),16);std::memcpy(out,a,16);
    for(unsigned i=0;i<3;++i){if((std::bit_cast<uint32_t>(a[i])&0x7f800000)==0x7f800000 || (std::bit_cast<uint32_t>(b[i])&0x7f800000)==0x7f800000 ||
        beyondRange(a[i]) || beyondRange(b[i]))throw std::runtime_error("unqualified native record dot");a[i]=pairNormal(a[i]);b[i]=pairNormal(b[i]);}
    {music::NearestRounding rounding;out[0]=pairSum(pairSum(pairProduct(a[0],b[0]),pairProduct(a[1],b[1])),pairProduct(a[2],b[2]));}
    std::memcpy(record(rt,stack+0x50,16),out,16);return true;
}
// Packed unit-quaternion keys: the CPU packs three signed 16-bit components
// into a GPR (lanes x, y, z); the VU0 block converts them (ITOF15, exact) and
// rebuilds w = sqrt(|1 - x*x - y*y - z*z|) (MSUB chain, nearest even), storing
// {x, y, z, w}. Blocks: mod001 1f9d40 (t2 to SP+10, then t2 to SP+0) and
// 2a44e0 (t2 to SP+110 and t5 to SP+100 in one interleaved block).
struct PackedKeySite{uint32_t first,last,store;unsigned laneReg,destinationReg;uint32_t destinationOffset;};
constexpr PackedKeySite packedKeySites[]={
    {0x1f9d74,0x1f9da4,0x1f9da4,10,3,0x10},{0x1f9df8,0x1f9e28,0x1f9e28,10,3,0x0},
    {0x2a492c,0x2a49a0,0x2a4994,10,4,0x110},{0x2a492c,0x2a49a0,0x2a49a0,13,3,0x100},
    // 2a44e0's three single-key paths (first, last and interpolated keys).
    {0x2a45d8,0x2a4608,0x2a4608,10,3,0x80},{0x2a46d8,0x2a4708,0x2a4708,10,3,0x70},{0x2a4798,0x2a47c8,0x2a47c8,10,3,0x60},
    // mod001 2a40e0: two single keys, then an interleaved pair.
    {0x2a415c,0x2a418c,0x2a418c,10,3,0x30},{0x2a4228,0x2a4258,0x2a4258,10,3,0x20},
    {0x2a440c,0x2a44a8,0x2a4478,10,3,0x50},{0x2a440c,0x2a44a8,0x2a44a8,13,2,0x40}};
bool nativePackedQuaternion(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc&3)return false;bool inside=false;
    for(const auto &site:packedKeySites){
        if(pc<site.first || pc>site.last)continue;inside=true;
        if(pc!=site.store)continue;
        if(!c)throw std::runtime_error("native packed quaternion has no CPU record");
        const auto destination=c->r[29].uw[0]+site.destinationOffset;
        if(c->r[site.destinationReg].uw[0]!=destination)throw std::runtime_error("native packed quaternion destination mismatch");
        for(unsigned i=0;i<3;++i){const auto lane=c->r[site.laneReg].sw[i];if(lane<-32768 || lane>32767)throw std::runtime_error("native packed quaternion lane is not a 16-bit key");}
        float out[4];
        {music::NearestRounding rounding;
            float squares[3];for(unsigned i=0;i<3;++i){out[i]=float(c->r[site.laneReg].sw[i])/32768.f;squares[i]=pairProduct(out[i],out[i]);}
            auto rest=pairSum(1.f,-squares[0]);rest=pairSum(rest,-squares[1]);rest=pairSum(rest,-squares[2]);
            volatile float root=std::sqrt(std::abs(rest));out[3]=pairProduct(1.f,pairNormal(root));}
        std::memcpy(record(rt,destination,16),out,16);
    }
    if(inside && !c)throw std::runtime_error("native packed quaternion has no CPU record");
    return inside;
}
// Collision-tree leaf tests called from 1f6f40's triangle loop. Their VU0
// inputs VF1..VF4 are the current object's matrix (object+150 rows) and VF5.x
// its scale (object+204), loaded by 1f6f40, which also stores the object to
// the game global 358cd8 just before the loop. VU0 arithmetic, nearest even.
using Lanes=std::array<float,4>;
struct LeafInputs{Lanes rows[4];float scale;};
float leafLane(float v){const auto bits=std::bit_cast<uint32_t>(v);
    if((bits&0x7f800000)==0x7f800000 || beyondRange(v))throw std::runtime_error("unqualified native collision leaf input");return pairNormal(v);}
Lanes leafRecord(Runtime *rt,uint32_t at){Lanes v;std::memcpy(v.data(),record(rt,at,16),16);for(float &x:v)x=leafLane(x);return v;}
// VU arithmetic on raw words with the engine's PS2 float definitions (ps2f:
// nearest-even, no Inf/NaN, exponent 255 is a number, saturation to +/-max,
// denormals read as zero); used where inputs may be arbitrary bit patterns
// (e.g. a stale sphere cache). Pure functions: no MXCSR or VU state.
using VuWords=std::array<uint32_t,4>;
VuWords vuWords(Runtime *rt,uint32_t at){VuWords v;std::memcpy(v.data(),record(rt,at,16),16);return v;}
uint32_t vuMul(uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);}
uint32_t vuAdd(uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::addNearest(a,b,f);}
uint32_t vuSub(uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::subNearest(a,b,f);}
LeafInputs leafInputs(Runtime *rt,bool withScale=true){
    uint32_t object;std::memcpy(&object,record(rt,0x358cd0,16)+8,4);
    if(object<0x100000)throw std::runtime_error("native collision leaf has no current object");
    LeafInputs in;for(unsigned i=0;i<4;++i)in.rows[i]=leafRecord(rt,object+0x150+16*i);
    in.scale=0;if(withScale){float scale;std::memcpy(&scale,record(rt,object+0x200,16)+4,4);in.scale=leafLane(scale);}return in;
}
// acc = R1*v.x + R2*v.y + R3*v.z + R4*1, all lanes.
Lanes leafTransform(const LeafInputs &in,const Lanes &v){Lanes out;
    for(unsigned i=0;i<4;++i)out[i]=pairSum(pairSum(pairSum(pairProduct(in.rows[0][i],v[0]),pairProduct(in.rows[1][i],v[1])),pairProduct(in.rows[2][i],v[2])),pairProduct(in.rows[3][i],1.f));
    return out;}
float leafAbs(float v){return std::bit_cast<float>(std::bit_cast<uint32_t>(v)&0x7fffffffu);}
// 1f6e10: sphere (a0: centre, radius w) against segment a1 (+0 point, +10
// direction); stores {S.xyz, distance} to its frame, returns distance <= 0.
bool nativeLeafSphere(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc<0x1f6e18 || pc>0x1f6e74 || (pc&3) || pc==0x1f6e70)return false;
    if(!c || c->r[31].uw[0]!=0x1f724c || c->r[4].uw[0]!=c->r[16].uw[0]+0x10)throw std::runtime_error("native collision sphere escaped its triangle loop");
    if(pc!=0x1f6e74)return true;
    if(c->r[2].uw[0]!=c->r[29].uw[0])throw std::runtime_error("native collision sphere frame mismatch");
    const auto in=leafInputs(rt);const auto sphere=leafRecord(rt,c->r[4].uw[0]),point=leafRecord(rt,c->r[5].uw[0]),direction=leafRecord(rt,c->r[5].uw[0]+0x10);
    Lanes out;std::memcpy(out.data(),record(rt,c->r[4].uw[0],16),16);   // stored raw, W replaced
    {music::NearestRounding rounding;
        const auto centre=leafTransform(in,sphere);
        const auto radius=pairProduct(sphere[3],in.scale),radius2=pairProduct(radius,radius);
        Lanes delta,product;for(unsigned i=0;i<3;++i){delta[i]=pairSum(centre[i],-point[i]);product[i]=pairProduct(delta[i],direction[i]);}
        auto along=pairSum(pairSum(pairProduct(1.f,product[0]),pairProduct(1.f,product[1])),pairProduct(1.f,product[2]));
        // VMAX.w with 0: zeros compare equal, so -0 stays; negative values lose.
        if(std::bit_cast<int32_t>(along)<0 && (std::bit_cast<uint32_t>(along)&0x7fffffffu))along=0.f;
        Lanes square;for(unsigned i=0;i<3;++i){const auto off=pairSum(delta[i],-pairProduct(direction[i],along));square[i]=pairProduct(off,off);}
        const auto distance=pairSum(pairSum(pairProduct(1.f,square[0]),pairProduct(1.f,square[1])),pairProduct(1.f,square[2]));
        out[3]=pairSum(distance,-radius2);}
    std::memcpy(record(rt,c->r[29].uw[0],16),out.data(),16);return true;
}
// 1f6be0: separating-axis test of the box record a0 (centre) / a1 (half
// extents) against the query box a2 (centre, +10 extents), in six stages.
// Each store site recomputes the stages so far and writes the lanes they
// defined (the rest of those registers is inherited state in a dead frame).
bool nativeLeafBox(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc<0x1f6be4 || pc>0x1f6de0 || (pc&3))return false;
    static constexpr uint32_t stores[]={0x1f6c54,0x1f6c58,0x1f6cb4,0x1f6cb8,0x1f6d10,0x1f6d14,0x1f6d68,0x1f6da8,0x1f6de0};
    // Only VU instructions reach here; EE words in the range are compiled.
    if(!c || c->r[31].uw[0]!=0x1f7238 || c->r[4].uw[0]!=c->r[5].uw[0]+0x10 || c->r[6].uw[0]!=c->r[17].uw[0])
        throw std::runtime_error("native collision box escaped its triangle loop");
    unsigned stage=99;for(unsigned i=0;i<9;++i)if(pc==stores[i])stage=i;
    if(stage==99)return true;
    if(c->r[2].uw[0]!=c->r[29].uw[0])throw std::runtime_error("native collision box frame mismatch");
    const auto in=leafInputs(rt);const auto box=leafRecord(rt,c->r[4].uw[0]),half=leafRecord(rt,c->r[5].uw[0]);
    const auto query=leafRecord(rt,c->r[6].uw[0]),extent=leafRecord(rt,c->r[6].uw[0]+0x10);
    const unsigned level=stage<2?0:stage<4?1:stage<6?2:stage==6?3:stage==7?4:5;
    Lanes v24{},v25{};bool d24[4]{},d25[4]{};
    {music::NearestRounding rounding;
        const auto &R1=in.rows[0],&R2=in.rows[1],&R3=in.rows[2];const auto centre=leafTransform(in,box);
        Lanes t;for(unsigned i=0;i<3;++i)t[i]=pairSum(query[i],-centre[i]);   // vf7
        const auto dotXYZ=[&](const Lanes &a,const Lanes &r){return pairSum(pairSum(pairProduct(a[0],r[0]),pairProduct(a[1],r[1])),pairProduct(1.f,pairProduct(a[2],r[2])));};
        const auto dotYXZ=[&](const Lanes &a,const Lanes &r){return pairSum(pairSum(pairProduct(a[1],r[1]),pairProduct(a[0],r[0])),pairProduct(1.f,pairProduct(a[2],r[2])));};
        const auto dotZXY=[&](const Lanes &a,const Lanes &r){return pairSum(pairSum(pairProduct(a[2],r[2]),pairProduct(a[0],r[0])),pairProduct(1.f,pairProduct(a[1],r[1])));};
        Lanes v10,v11,v12,v13;
        v10[0]=dotXYZ(extent,R1);v12[0]=dotXYZ(t,R1);v10[1]=dotYXZ(extent,R2);v11[0]=leafAbs(v10[0]);v13[0]=leafAbs(v12[0]);v12[1]=dotYXZ(t,R2);
        v25[0]=pairProduct(v12[0],v10[0]);v24[0]=pairSum(v13[0],-half[0]);d24[0]=d25[0]=true;
        if(level>=1){v11[1]=leafAbs(v10[1]);v13[1]=leafAbs(v12[1]);v25[1]=pairProduct(v12[1],v10[1]);v24[1]=pairSum(v13[1],-half[1]);
            v10[2]=dotZXY(extent,R3);v12[2]=dotZXY(t,R3);d24[1]=d25[1]=true;}
        Lanes v15,v16;
        if(level>=2){v11[2]=leafAbs(v10[2]);v13[2]=leafAbs(v12[2]);
            // OPMULA/OPMSUB: v15 = extent x t.
            v15[0]=pairSum(pairProduct(extent[1],t[2]),-pairProduct(t[1],extent[2]));
            v15[1]=pairSum(pairProduct(extent[2],t[0]),-pairProduct(t[2],extent[0]));
            v15[2]=pairSum(pairProduct(extent[0],t[1]),-pairProduct(t[0],extent[1]));
            v25[2]=pairProduct(v12[2],v10[2]);v24[2]=pairSum(v13[2],-half[2]);d24[2]=d25[2]=true;}
        if(level>=3){v16[0]=leafAbs(dotXYZ(v15,R1));
            v24[1]=pairSum(pairSum(pairProduct(v11[1],half[2]),pairProduct(half[1],v11[2])),-pairProduct(1.f,v16[0]));}
        if(level>=4){v16[1]=leafAbs(dotYXZ(v15,R2));
            v24[0]=pairSum(pairSum(pairProduct(v11[0],half[2]),pairProduct(half[0],v11[2])),-pairProduct(1.f,v16[1]));}
        if(level>=5){v16[2]=leafAbs(dotZXY(v15,R3));
            v24[0]=pairSum(pairSum(pairProduct(v11[0],half[1]),pairProduct(half[0],v11[1])),-pairProduct(1.f,v16[2]));}
    }
    auto *frame=record(rt,c->r[29].uw[0],32);
    const bool second=stage==1 || stage==3 || stage==5;   // VF25 at +10
    for(unsigned i=0;i<4;++i){if(!second && d24[i])std::memcpy(frame+4*i,&v24[i],4);if(second && d25[i])std::memcpy(frame+0x10+4*i,&v25[i],4);}
    return true;
}
// mod001 1f6740: segment against the hit box after a collision-tree query.
// The query (1f6f40) returns on a hit without restoring VF1..VF4, so they hold
// the hit object's matrix; that object is in the global 358cd8 and the query
// segment (+0 start P, +10 direction D) in 358d54, which the caller passes as
// a2. Block A stores the segment in box space: start R.(P - M*C) at SP+90 and
// direction R.D at SP+80 (C = a0, the box centre). The scalar helper 1f6670
// clips [tmin, tmax] at SP+a0/a4; block B stores P + D*t points to a3 (s4).
bool nativeSegmentBox(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool blockA=pc>=0x1f6770 && pc<=0x1f6800,blockB=pc>=0x1f6964 && pc<=0x1f69f4;
    if(!(blockA || blockB) || (pc&3))return false;
    if(!c)throw std::runtime_error("native segment box has no CPU record");
    uint32_t queryAt;std::memcpy(&queryAt,record(rt,0x358d50,16)+4,4);
    if(queryAt<0x100000)throw std::runtime_error("native segment box has no query segment");
    const auto stack=c->r[29].uw[0];
    if(blockA){
        if(c->r[6].uw[0]!=queryAt)throw std::runtime_error("native segment box query mismatch");
        if(pc!=0x1f67fc && pc!=0x1f6800)return true;
        if(c->r[3].uw[0]!=stack+0x90 || c->r[2].uw[0]!=stack+0x80)throw std::runtime_error("native segment box frame mismatch");
        const auto in=leafInputs(rt,false);const auto centre=leafRecord(rt,c->r[4].uw[0]),point=leafRecord(rt,queryAt),direction=leafRecord(rt,queryAt+0x10);
        Lanes start,along;
        {music::NearestRounding rounding;
            const auto world=leafTransform(in,centre);Lanes delta;for(unsigned i=0;i<3;++i)delta[i]=pairSum(point[i],-world[i]);
            const auto &R1=in.rows[0],&R2=in.rows[1],&R3=in.rows[2];
            const auto dot=[&](const Lanes &a,const Lanes &r,unsigned i,unsigned j,unsigned k){return pairSum(pairSum(pairProduct(a[i],r[i]),pairProduct(a[j],r[j])),pairProduct(1.f,pairProduct(a[k],r[k])));};
            start={dot(delta,R1,0,1,2),dot(delta,R2,1,0,2),dot(delta,R3,2,0,1),0};
            along={dot(direction,R1,0,1,2),dot(direction,R2,1,0,2),dot(direction,R3,2,0,1),0};}
        // W of both registers is inherited state; only XYZ is written.
        std::memcpy(record(rt,pc==0x1f67fc?stack+0x90:stack+0x80,16),pc==0x1f67fc?start.data():along.data(),12);return true;
    }
    if(pc!=0x1f69ac && pc!=0x1f69b0 && pc!=0x1f69c0 && pc!=0x1f69f4)return true;
    if(c->r[2].uw[0]!=stack+0xa0 && pc==0x1f6964)throw std::runtime_error("native segment box clip mismatch");
    float clip[4];std::memcpy(clip,record(rt,stack+0xa0,16),16);
    const float t=leafLane(pc==0x1f69b0 || pc==0x1f69f4?clip[1]:clip[0]);
    const auto point=leafRecord(rt,queryAt),direction=leafRecord(rt,queryAt+0x10);
    float out[3];{music::NearestRounding rounding;for(unsigned i=0;i<3;++i)out[i]=pairSum(pairProduct(point[i],1.f),pairProduct(direction[i],t));}
    std::memcpy(record(rt,c->r[20].uw[0]+(pc==0x1f69b0?0x10:0),16),out,12);return true;
}
// mod001 246760: two row.point products of the matrix at SP+130 (points
// SP+1b0 / SP+1a0 into SP+190 / SP+180), each followed by an in-place divide
// by its W (Q = 1/w, all four lanes), then a composition of the rows at SP+e0
// with the matrix at s5+90, written back over s5+90. Row sums keep the
// original orders: x (x+y)+z+w, y (y+x)+z+w, z (z+y)+x+w, w (w+y)+z+x.
bool nativeProjectionBlocks(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Block{uint32_t first,last,store;unsigned kind;uint32_t point,out;};   // kind 0 product, 1 divide, 2 compose
    static constexpr Block blocks[]={{0x246944,0x24699c,0x24699c,0,0x1b0,0x190},{0x2469b8,0x2469c8,0x2469c8,1,0x190,0x190},
                                     {0x2469d8,0x246a30,0x246a30,0,0x1a0,0x180},{0x246a4c,0x246a5c,0x246a5c,1,0x180,0x180},
                                     {0x246e1c,0x246e88,0x246e7c,2,0,0}};
    const Block *block=nullptr;for(const auto &b:blocks)if(pc>=b.first && pc<=b.last && !(pc&3)){block=&b;break;}
    if(!block)return false;
    if(!c)throw std::runtime_error("native projection block has no CPU record");
    const auto stack=c->r[29].uw[0];
    if(block->kind==2){
        const auto target=c->r[21].uw[0]+0x90;
        if(c->r[3].uw[0]!=stack+0xe0 || c->r[2].uw[0]!=target || c->r[4].uw[0]!=target || c->r[21].uw[0]<0x100000)
            throw std::runtime_error("native projection composition escaped its records");
        if(pc!=block->store)return true;
        Lanes rows[4],m[4],out[4];for(unsigned k=0;k<4;++k){rows[k]=leafRecord(rt,stack+0xe0+16*k);m[k]=leafRecord(rt,target+16*k);}
        {music::NearestRounding rounding;
            for(unsigned k=0;k<4;++k)for(unsigned i=0;i<4;++i){
                out[k][i]=pairSum(pairSum(pairSum(pairProduct(m[0][i],rows[k][0]),pairProduct(m[1][i],rows[k][1])),pairProduct(m[2][i],rows[k][2])),pairProduct(m[3][i],rows[k][3]));
                if(!std::isfinite(out[k][i]))throw std::runtime_error("native projection composition overflow");}}
        std::memcpy(record(rt,target,64),out,64);return true;
    }
    if(block->kind==0 && (c->r[4].uw[0]!=stack+0x130 || c->r[3].uw[0]!=stack+block->point || c->r[2].uw[0]!=stack+block->out))
        throw std::runtime_error("native projection product escaped its records");
    if(block->kind==1 && (pc==block->first?c->r[3].uw[0]!=stack+block->point:pc==block->store && c->r[2].uw[0]!=stack+block->out))
        throw std::runtime_error("native projection divide escaped its record");
    if(pc!=block->store)return true;
    Lanes out;
    if(block->kind==0){
        Lanes m[4];for(unsigned k=0;k<4;++k)m[k]=leafRecord(rt,stack+0x130+16*k);const auto v=leafRecord(rt,stack+block->point);
        music::NearestRounding rounding;Lanes p[4];for(unsigned k=0;k<4;++k)for(unsigned i=0;i<4;++i)p[k][i]=pairProduct(m[k][i],v[i]);
        constexpr unsigned order[4][4]={{0,1,2,3},{1,0,2,3},{2,1,0,3},{3,1,2,0}};
        for(unsigned k=0;k<4;++k)out[k]=pairSum(pairSum(pairSum(p[k][order[k][0]],p[k][order[k][1]]),pairProduct(1.f,p[k][order[k][2]])),pairProduct(1.f,p[k][order[k][3]]));
    }else{
        const auto v=leafRecord(rt,stack+block->point);
        if(!(std::bit_cast<uint32_t>(v[3])&0x7fffffffu))throw std::runtime_error("native projection divide by zero depth");
        music::NearestRounding rounding;volatile float q=1.f/v[3];const auto reciprocal=pairNormal(q);
        for(unsigned i=0;i<4;++i)out[i]=pairProduct(v[i],reciprocal);
    }
    for(float x:out)if(!std::isfinite(x))throw std::runtime_error("native projection block overflow");
    std::memcpy(record(rt,stack+block->out,16),out.data(),16);return true;
}
// mod001 1fd110 bone aim: 1fe400 puts 0.5 in VF29 (consumed only by 1fe674
// below, natively a constant). Block 1fe598..1fe680 transforms a (SP+1c0) and
// b (*(s8+38)+30) by the matrix at SP+180, normalizes them (1/sqrt), stores
// their dot to SP+170 (X only; the rest of that register is inherited) and the
// rotation a->b as q = (b x a / sqrt(2+2d), sqrt(2+2d)/2) to SP+160. Block
// 1fe68c..1fe6c0 multiplies it by s1+30 in place:
// xyz = q2*q1.w + q1*q2.w + q1 x q2, w = ((w1w2 - z1z2) - y1y2) - x1x2.
bool nativeBoneAim(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc==0x1fe400){if(!c || c->r[3].uw[0]!=0x3f000000)throw std::runtime_error("native bone aim constant mismatch");return true;}
    const bool aim=pc>=0x1fe598 && pc<=0x1fe680,multiply=pc>=0x1fe68c && pc<=0x1fe6c0;
    if(!(aim || multiply) || (pc&3))return false;
    if(!c)throw std::runtime_error("native bone aim has no CPU record");
    const auto stack=c->r[29].uw[0];
    if(multiply){
        if(c->r[2].uw[0]!=stack+0x160 || c->r[3].uw[0]!=c->r[17].uw[0]+0x30)throw std::runtime_error("native bone quaternion product escaped its records");
        if(pc!=0x1fe6c0)return true;
        const auto q1=leafRecord(rt,stack+0x160),q2=leafRecord(rt,c->r[17].uw[0]+0x30);Lanes out;
        {music::NearestRounding rounding;
            const Lanes cross={pairSum(pairProduct(q1[1],q2[2]),-pairProduct(q2[1],q1[2])),pairSum(pairProduct(q1[2],q2[0]),-pairProduct(q2[2],q1[0])),
                               pairSum(pairProduct(q1[0],q2[1]),-pairProduct(q2[0],q1[1])),0};
            for(unsigned i=0;i<3;++i)out[i]=pairSum(pairSum(pairProduct(q2[i],q1[3]),pairProduct(q1[i],q2[3])),pairProduct(cross[i],1.f));
            Lanes p;for(unsigned i=0;i<4;++i)p[i]=pairProduct(q1[i],q2[i]);
            out[3]=pairSum(pairSum(pairSum(p[3],-p[2]),-pairProduct(1.f,p[1])),-pairProduct(1.f,p[0]));}
        std::memcpy(record(rt,stack+0x160,16),out.data(),16);return true;
    }
    const auto partnerAt=c->r[30].uw[0]+0x38;if((partnerAt&3) || (partnerAt&0x1ffffff)<0x100000)throw std::runtime_error("native bone aim partner record outside EE RAM");
    uint32_t partner;std::memcpy(&partner,rt->memory.hostPointer(partnerAt,4),4);
    if(pc==0x1fe598 && (c->r[5].uw[0]!=stack+0x180 || c->r[4].uw[0]!=stack+0x1c0 || c->r[3].uw[0]!=partner+0x30))throw std::runtime_error("native bone aim sources mismatch");
    if(pc!=0x1fe63c && pc!=0x1fe680)return true;
    if(pc==0x1fe63c?c->r[3].uw[0]!=stack+0x170:c->r[2].uw[0]!=stack+0x160)throw std::runtime_error("native bone aim destination mismatch");
    LeafInputs m;for(unsigned i=0;i<4;++i)m.rows[i]=leafRecord(rt,stack+0x180+16*i);m.scale=0;
    const auto a=leafRecord(rt,stack+0x1c0),b=leafRecord(rt,partner+0x30);
    float dot;Lanes quaternion;
    {music::NearestRounding rounding;
        const auto unit=[&](const Lanes &v){const auto t=leafTransform(m,v);
            const auto squared=pairSum(pairSum(pairProduct(t[0],t[0]),pairProduct(t[1],t[1])),pairProduct(1.f,pairProduct(t[2],t[2])));
            if(!(std::bit_cast<uint32_t>(squared)&0x7fffffffu))throw std::runtime_error("native bone aim has a zero vector");
            volatile float root=std::sqrt(std::abs(squared));volatile float reciprocal=1.f/pairNormal(root);
            Lanes u;for(unsigned i=0;i<3;++i)u[i]=pairProduct(t[i],pairNormal(reciprocal));return u;};
        const auto ua=unit(a),ub=unit(b);
        dot=pairSum(pairSum(pairProduct(ua[0],ub[0]),pairProduct(ua[1],ub[1])),pairProduct(ua[2],ub[2]));
        const Lanes cross={pairSum(pairProduct(ub[1],ua[2]),-pairProduct(ua[1],ub[2])),pairSum(pairProduct(ub[2],ua[0]),-pairProduct(ua[2],ub[0])),
                           pairSum(pairProduct(ub[0],ua[1]),-pairProduct(ua[0],ub[1])),0};
        const auto sum=pairSum(pairSum(0.f,2.f),pairProduct(dot,2.f));
        volatile float root=std::sqrt(std::abs(sum));const auto q=pairProduct(1.f,pairNormal(root));
        if(!(std::bit_cast<uint32_t>(q)&0x7fffffffu))throw std::runtime_error("native bone aim has opposite vectors");
        volatile float reciprocal=1.f/q;for(unsigned i=0;i<3;++i)quaternion[i]=pairProduct(cross[i],pairNormal(reciprocal));
        quaternion[3]=pairProduct(q,0.5f);}
    if(pc==0x1fe63c)std::memcpy(record(rt,stack+0x170,16),&dot,4);
    else std::memcpy(record(rt,stack+0x160,16),quaternion.data(),16);
    return true;
}
// mod001 1fd110 chain constraint blocks (s1 = part record, s2 = chain owner,
// owner object = *(s2+4)). VU registers that cross EE code between blocks
// are reread from the records the earlier block stored (bit-exact):
//  1fda08..1fda84  P = *(s1+38)+30 by the object matrix +190 (with
//                  translation); flag 4 adds the s1 offset; offset (or VF0)
//                  to SP+460, point to SP+220.
//  1fdacc..1fdaf0  point * s1+30 to SP+460, point * f22 to SP+470.
//  1fdb54, 1fdc48  SP+210 rotated by the rows +150 (no translation), in place.
//  1fdeb0..1fdee8  origin (SP+460) + dir*a to SP+200, dir - dir*f22 to SP+1f0
//                  (dir SP+220; W lanes kept raw).
//  1fe790..1fe7e8  matrix SP+120 . point s0+30 into SP+110 (row orders as
//                  the 246760 products).
bool nativeChainBlocks(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool place=pc>=0x1fda08 && pc<=0x1fda84,split=pc>=0x1fdacc && pc<=0x1fdaf0,rotateA=pc>=0x1fdb54 && pc<=0x1fdb74,
               rotateB=pc>=0x1fdc48 && pc<=0x1fdc68,blend=pc>=0x1fdeb0 && pc<=0x1fdee8,product=pc>=0x1fe790 && pc<=0x1fe7e8;
    if(!(place || split || rotateA || rotateB || blend || product) || (pc&3))return false;
    if(!c)throw std::runtime_error("native chain block has no CPU record");
    const auto stack=c->r[29].uw[0],part=c->r[17].uw[0];
    const auto word=[&](uint32_t at){if((at&3) || (at&0x1ffffff)<0x100000)throw std::runtime_error("native chain record outside EE RAM");
        uint32_t v;std::memcpy(&v,rt->memory.hostPointer(at,4),4);return v;};
    const auto rowsAt=[&](uint32_t at,unsigned n){LeafInputs m;for(unsigned i=0;i<4;++i)m.rows[i]=i<n?leafRecord(rt,at+16*i):Lanes{0,0,0,0};m.scale=0;return m;};
    const auto raw=[&](uint32_t at){Lanes v;std::memcpy(v.data(),record(rt,at,16),16);return v;};
    if(place){
        const auto point=word(part+0x38)+0x30,matrix=word(c->r[18].uw[0]+4)+0x190;
        if(pc==0x1fda08 && c->r[2].uw[0]!=point)throw std::runtime_error("native chain point source mismatch");
        if(pc!=0x1fda70 && pc!=0x1fda7c && pc!=0x1fda84)return true;
        if(c->r[2].uw[0]!=(pc==0x1fda84?stack+0x220:stack+0x460))throw std::runtime_error("native chain placement destination mismatch");
        const bool offset=(word(part+0x20)&4)!=0;
        if(pc==0x1fda70){if(!offset)throw std::runtime_error("native chain offset branch mismatch");std::memcpy(record(rt,stack+0x460,16),record(rt,part,16),16);return true;}
        if(pc==0x1fda7c){if(offset)throw std::runtime_error("native chain offset branch mismatch");const float identity[4]={0,0,0,1};std::memcpy(record(rt,stack+0x460,16),identity,16);return true;}
        const auto m=rowsAt(matrix,4);const auto p=leafRecord(rt,point);Lanes out;
        {music::NearestRounding rounding;out=leafTransform(m,p);
            if(offset){const auto shift=leafRecord(rt,part);for(unsigned i=0;i<3;++i)out[i]=pairSum(out[i],shift[i]);}}
        for(float x:out)if(!std::isfinite(x))throw std::runtime_error("native chain placement overflow");
        std::memcpy(record(rt,stack+0x220,16),out.data(),16);return true;
    }
    if(split){
        if(pc!=0x1fdaec && pc!=0x1fdaf0)return true;
        if(pc==0x1fdaec?c->r[3].uw[0]!=stack+0x460:c->r[2].uw[0]!=stack+0x470)throw std::runtime_error("native chain split destination mismatch");
        const auto v=leafRecord(rt,stack+0x220);const float scale=leafLane(pc==0x1fdaec?std::bit_cast<float>(word(part+0x30)):c->f[22]);
        Lanes out;{music::NearestRounding rounding;for(unsigned i=0;i<4;++i)out[i]=pairProduct(v[i],scale);}
        std::memcpy(record(rt,pc==0x1fdaec?stack+0x460:stack+0x470,16),out.data(),16);return true;
    }
    if(rotateA || rotateB){
        const auto matrix=word(c->r[18].uw[0]+4)+0x150;const uint32_t store=rotateA?0x1fdb74:0x1fdc68;
        if((rotateA && pc==0x1fdb54 && (c->r[3].uw[0]!=stack+0x210 || c->r[2].uw[0]!=matrix)) || (rotateB && pc==0x1fdc48 && (c->r[2].uw[0]!=stack+0x210 || c->r[3].uw[0]!=matrix)))
            throw std::runtime_error("native chain rotation source mismatch");
        if(pc!=store)return true;
        if(c->r[2].uw[0]!=stack+0x210)throw std::runtime_error("native chain rotation destination mismatch");
        const auto m=rowsAt(matrix,3);const auto v=leafRecord(rt,stack+0x210);Lanes out;
        {music::NearestRounding rounding;for(unsigned i=0;i<4;++i)out[i]=pairSum(pairSum(pairProduct(m.rows[0][i],v[0]),pairProduct(m.rows[1][i],v[1])),pairProduct(m.rows[2][i],v[2]));}
        std::memcpy(record(rt,stack+0x210,16),out.data(),16);return true;
    }
    if(blend){
        if(pc==0x1fdeb0 && c->r[2].uw[0]!=stack+0x220)throw std::runtime_error("native chain blend source mismatch");
        if(pc!=0x1fdee4 && pc!=0x1fdee8)return true;
        if(pc==0x1fdee4?c->r[3].uw[0]!=stack+0x200:c->r[2].uw[0]!=stack+0x1f0)throw std::runtime_error("native chain blend destination mismatch");
        const auto dir=leafRecord(rt,stack+0x220),origin=leafRecord(rt,stack+0x460);
        auto out=raw(pc==0x1fdee4?stack+0x460:stack+0x220);   // W kept raw
        {music::NearestRounding rounding;
            const float a=leafLane(std::bit_cast<float>(word(part+0x30))),s=leafLane(c->f[22]);
            for(unsigned i=0;i<3;++i)out[i]=pc==0x1fdee4?pairSum(pairSum(origin[i],0.f),pairProduct(dir[i],a)):pairSum(pairSum(dir[i],0.f),-pairProduct(dir[i],s));}
        std::memcpy(record(rt,pc==0x1fdee4?stack+0x200:stack+0x1f0,16),out.data(),16);return true;
    }
    // product
    const auto owner=c->r[16].uw[0];
    if(c->r[4].uw[0]!=stack+0x120 || c->r[3].uw[0]!=owner+0x30 || c->r[2].uw[0]!=stack+0x110)throw std::runtime_error("native chain product escaped its records");
    if(pc!=0x1fe7e8)return true;
    const auto m=rowsAt(stack+0x120,4);const auto v=leafRecord(rt,owner+0x30);Lanes out;
    {music::NearestRounding rounding;Lanes p[4];for(unsigned k=0;k<4;++k)for(unsigned i=0;i<4;++i)p[k][i]=pairProduct(m.rows[k][i],v[i]);
        constexpr unsigned order[4][4]={{0,1,2,3},{1,0,2,3},{2,1,0,3},{3,1,2,0}};
        for(unsigned k=0;k<4;++k)out[k]=pairSum(pairSum(pairSum(p[k][order[k][0]],p[k][order[k][1]]),pairProduct(1.f,p[k][order[k][2]])),pairProduct(1.f,p[k][order[k][3]]));}
    for(float x:out)if(!std::isfinite(x))throw std::runtime_error("native chain product overflow");
    std::memcpy(record(rt,stack+0x110,16),out.data(),16);return true;
}
// mod001 1fc300 capsule pair (virtual +94 of the chain part, called only from
// 1fdc24 with ra 1fdc2c). a0 = capsule A, a1 = capsule B, a2 = per-axis
// weights, a3 = separation output. Its VF24..27 are the chain owner's object
// matrix +190, loaded at 1fda1c on the only path to that call; the caller's s2
// is in 1fc300's frame at SP+20.
//  1fc334..1fc3c4  P = M*(A+250) (with translation) scaled per lane by
//                  (rA+rB)/(rA + w*rB), to SP+60; D = rows*(A+260) to SP+70.
//  1fc410..1fc478  delta = (B+230 + B+240*tB) - P - D*tA, normalized with
//                  RSQRT; XYZ to a3 (W of that register is inherited).
bool nativeCapsulePair(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool place=pc>=0x1fc334 && pc<=0x1fc3c4,separate=pc>=0x1fc410 && pc<=0x1fc478;
    if(!(place || separate) || (pc&3))return false;
    if(!c)throw std::runtime_error("native capsule pair has no CPU record");
    const auto stack=c->r[29].uw[0],a=c->r[20].uw[0],b=c->r[19].uw[0];
    const auto word=[&](uint32_t at){if(at&3)throw std::runtime_error("native capsule word misaligned");
        uint32_t v;std::memcpy(&v,rt->memory.hostPointer(at,4),4);return v;};
    const auto lane=[&](uint32_t at){return leafLane(std::bit_cast<float>(word(at)));};
    if(place){
        if(pc==0x1fc334 && (c->r[2].uw[0]!=a+0x250 || c->r[31].uw[0]!=0x1fdc2c))throw std::runtime_error("native capsule pair caller mismatch");
        if(pc!=0x1fc3c0 && pc!=0x1fc3c4)return true;
        if(c->r[2].uw[0]!=stack+0x60)throw std::runtime_error("native capsule pair destination mismatch");
        LeafInputs m;const auto matrix=word(word(stack+0x20)+4)+0x190;for(unsigned i=0;i<4;++i)m.rows[i]=leafRecord(rt,matrix+16*i);m.scale=0;
        Lanes out;
        if(pc==0x1fc3c4){
            const auto d=leafRecord(rt,a+0x260);
            {music::NearestRounding rounding;for(unsigned i=0;i<4;++i)out[i]=pairSum(pairSum(pairProduct(m.rows[0][i],d[0]),pairProduct(m.rows[1][i],d[1])),pairProduct(m.rows[2][i],d[2]));}
            for(float x:out)if(!std::isfinite(x))throw std::runtime_error("native capsule direction overflow");
            std::memcpy(record(rt,stack+0x70,16),out.data(),16);return true;
        }
        const auto weights=leafRecord(rt,c->r[6].uw[0]);const float ra=lane(a+0x220),rb=lane(b+0x220);
        {music::NearestRounding rounding;out=leafTransform(m,leafRecord(rt,a+0x250));const auto sum=pairSum(rb,ra);
            for(unsigned i=0;i<3;++i){const auto den=pairSum(pairSum(0.f,ra),pairProduct(weights[i],rb));
                if(!(std::bit_cast<uint32_t>(den)&0x7fffffffu))throw std::runtime_error("native capsule pair has a zero weight");
                volatile float q=sum/den;out[i]=pairProduct(out[i],pairNormal(q));}}
        for(float x:out)if(!std::isfinite(x))throw std::runtime_error("native capsule point overflow");
        std::memcpy(record(rt,stack+0x60,16),out.data(),16);return true;
    }
    if(pc==0x1fc410 && (c->r[3].uw[0]!=b+0x230 || c->r[2].uw[0]!=b+0x240 || c->r[5].uw[0]!=stack+0x60 || c->r[4].uw[0]!=stack+0x70))
        throw std::runtime_error("native capsule separation sources mismatch");
    if(pc!=0x1fc478)return true;
    const auto target=c->r[18].uw[0];if(target&15)throw std::runtime_error("native capsule separation destination misaligned");
    const auto p0=leafRecord(rt,b+0x230),p1=leafRecord(rt,b+0x240),p=leafRecord(rt,stack+0x60),d=leafRecord(rt,stack+0x70);
    const float tb=lane(stack+0x88),ta=lane(stack+0x8c);Lanes out;
    {music::NearestRounding rounding;Lanes delta;
        for(unsigned i=0;i<3;++i)delta[i]=pairSum(pairSum(pairSum(pairSum(p0[i],0.f),pairProduct(p1[i],tb)),-pairProduct(p[i],1.f)),-pairProduct(d[i],ta));
        const auto squared=pairSum(pairSum(pairProduct(delta[0],delta[0]),pairProduct(delta[1],delta[1])),pairProduct(1.f,pairProduct(delta[2],delta[2])));
        if(!(std::bit_cast<uint32_t>(squared)&0x7fffffffu))throw std::runtime_error("native capsule separation is zero");
        volatile float root=std::sqrt(std::abs(squared));volatile float reciprocal=1.f/pairNormal(root);
        for(unsigned i=0;i<3;++i)out[i]=pairProduct(delta[i],pairNormal(reciprocal));}
    std::memcpy(record(rt,target,16),out.data(),12);return true;
}
// mod001 1fbd60 plane contact (virtual +8c, a0 = plane object, a1 = point,
// a2 = output, f12 = radius); all inputs are explicit:
//  1fbd6c..1fbda0  d = ((m.x + m.y) + 1*m.z) - r, m = (p - a0+230) * a0+220,
//                  to SP+10 (only X is read back; the rest of that local
//                  register is dead after return).
//  1fbdb8..1fbdc0  (p + 0) - n*d to a2, W = the point's raw W.
// The local transform 1fbdcc..1fbdf0 stays with the shared full-transform
// fragment below (qualified in the bones suite).
bool nativePlaneContact(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool distance=pc>=0x1fbd6c && pc<=0x1fbda0,project=pc>=0x1fbdb8 && pc<=0x1fbdc0;
    if(!(distance || project) || (pc&3))return false;
    if(!c)throw std::runtime_error("native plane contact has no CPU record");
    const auto stack=c->r[29].uw[0],plane=c->r[4].uw[0],point=c->r[5].uw[0];
    if(distance){
        if(pc==0x1fbd6c && (c->r[3].uw[0]!=plane+0x230 || c->r[2].uw[0]!=plane+0x220))throw std::runtime_error("native plane contact source mismatch");
        if(pc!=0x1fbda0)return true;
        if(c->r[2].uw[0]!=stack+0x10)throw std::runtime_error("native plane contact destination mismatch");
        const auto p=leafRecord(rt,point),o=leafRecord(rt,plane+0x230),n=leafRecord(rt,plane+0x220);const float r=leafLane(c->f[12]);float d;
        {music::NearestRounding rounding;Lanes m;for(unsigned i=0;i<3;++i)m[i]=pairProduct(pairSum(p[i],-o[i]),n[i]);
            d=pairSum(pairSum(pairSum(m[0],m[1]),pairProduct(1.f,m[2])),-r);}
        std::memcpy(record(rt,stack+0x10,16),&d,4);return true;
    }
    {
        if(pc!=0x1fbdc0)return true;
        const auto target=c->r[6].uw[0];if(target&15)throw std::runtime_error("native plane contact output misaligned");
        const auto p=leafRecord(rt,point),n=leafRecord(rt,plane+0x220);float d;std::memcpy(&d,record(rt,stack+0x10,16),4);d=leafLane(d);
        Lanes out;std::memcpy(out.data(),record(rt,point,16),16);
        {music::NearestRounding rounding;for(unsigned i=0;i<3;++i)out[i]=pairSum(pairSum(p[i],0.f),-pairProduct(n[i],d));}
        std::memcpy(record(rt,target,16),out.data(),16);return true;
    }
    return true;
}
// mod001 1fbad0 (a0 = object, a1 = point; called from mod003 4466c0). All
// inputs are explicit and every stored lane is defined:
//  1fbb00..1fbb24  the point (with its own W) through the rows a0+220 to SP+30.
//  1fbb40..1fbb54  SP+20 (= SP+30 with Y cleared by EE code): squared length
//                  ((x2+y2)+1*z2, y2, z2, raw W) to SP+10.
//  1fbb88..1fbb9c  the same for SP+30, to SP+0.
bool nativeRangeTest(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool transform=pc>=0x1fbb00 && pc<=0x1fbb24,flat=pc>=0x1fbb40 && pc<=0x1fbb54,full=pc>=0x1fbb88 && pc<=0x1fbb9c;
    if(!(transform || flat || full) || (pc&3))return false;
    if(!c)throw std::runtime_error("native range test has no CPU record");
    const auto stack=c->r[29].uw[0],object=c->r[4].uw[0];
    if(transform){
        if(pc==0x1fbb00 && (c->r[6].uw[0]!=object+0x220 || c->r[3].uw[0]!=stack+0x30))throw std::runtime_error("native range test source mismatch");
        if(pc!=0x1fbb24)return true;
        if(c->r[3].uw[0]!=stack+0x30)throw std::runtime_error("native range test destination mismatch");
        LeafInputs m;for(unsigned i=0;i<4;++i)m.rows[i]=leafRecord(rt,object+0x220+16*i);
        const auto p=leafRecord(rt,c->r[5].uw[0]);Lanes out;
        {music::NearestRounding rounding;for(unsigned i=0;i<4;++i)out[i]=pairSum(pairSum(pairSum(pairProduct(m.rows[0][i],p[0]),pairProduct(m.rows[1][i],p[1])),pairProduct(m.rows[2][i],p[2])),pairProduct(m.rows[3][i],p[3]));}
        for(float x:out)if(!std::isfinite(x))throw std::runtime_error("native range test transform overflow");
        std::memcpy(record(rt,stack+0x30,16),out.data(),16);return true;
    }
    const auto source=flat?stack+0x20:stack+0x30,target=flat?stack+0x10:stack;
    const unsigned sourceRegister=flat?6:5,targetRegister=flat?5:3;
    if((pc==0x1fbb40 || pc==0x1fbb88) && c->r[sourceRegister].uw[0]!=source)throw std::runtime_error("native range test length source mismatch");
    if(pc!=0x1fbb54 && pc!=0x1fbb9c)return true;
    if(c->r[targetRegister].uw[0]!=target)throw std::runtime_error("native range test length destination mismatch");
    const auto v=leafRecord(rt,source);Lanes out;std::memcpy(out.data(),record(rt,source,16),16);
    {music::NearestRounding rounding;Lanes sq;for(unsigned i=0;i<3;++i)sq[i]=pairProduct(v[i],v[i]);
        out[0]=pairSum(pairSum(sq[0],sq[1]),pairProduct(1.f,sq[2]));out[1]=sq[1];out[2]=sq[2];}
    for(float x:out)if(!std::isfinite(x))throw std::runtime_error("native range test length overflow");
    std::memcpy(record(rt,target,16),out.data(),16);return true;
}
// mod001 2178c0 matrix build: rows a_k at SP+40 (built by EE code from
// f12..f16) composed with the 216e40 result B at SP+80; row k of s0 is
// ((B0*a.x + B1*a.y) + B2*a.z) + B3*a.w. All four rows are computed before
// the first store (2179a8), as the original holds them in VF1..VF4.
bool nativeMatrixBuild(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc<0x217948 || pc>0x2179b4 || (pc&3))return false;
    if(!c)throw std::runtime_error("native matrix build has no CPU record");
    const auto stack=c->r[29].uw[0];
    if(pc==0x217948 && (c->r[5].uw[0]!=stack+0x40 || c->r[3].uw[0]!=stack+0x80))throw std::runtime_error("native matrix build source mismatch");
    if(pc!=0x2179a8)return true;
    const auto target=c->r[16].uw[0];if(target&15)throw std::runtime_error("native matrix build destination misaligned");
    Lanes a[4],b[4],out[4];for(unsigned k=0;k<4;++k){a[k]=leafRecord(rt,stack+0x40+16*k);b[k]=leafRecord(rt,stack+0x80+16*k);}
    {music::NearestRounding rounding;
        for(unsigned k=0;k<4;++k)for(unsigned i=0;i<4;++i)
            out[k][i]=pairSum(pairSum(pairSum(pairProduct(b[0][i],a[k][0]),pairProduct(b[1][i],a[k][1])),pairProduct(b[2][i],a[k][2])),pairProduct(b[3][i],a[k][3]));}
    for(const auto &row:out)for(float x:row)if(!std::isfinite(x))throw std::runtime_error("native matrix build overflow");
    std::memcpy(record(rt,target,64),out,64);return true;
}
// mod001 29f5e0 list box query (called from 23e1a4): per node s0, the query
// point copied to SP+30 (W set to 1 by EE code) through the rows
// *(s0+230)+190, all four lanes to SP+20; the box test is scalar EE code.
bool nativeListBoxQuery(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc<0x29f674 || pc>0x29f698 || (pc&3))return false;
    if(!c)throw std::runtime_error("native list box query has no CPU record");
    const auto stack=c->r[29].uw[0],node=c->r[16].uw[0];
    if(node&3)throw std::runtime_error("native list box query node misaligned");
    uint32_t object;std::memcpy(&object,rt->memory.hostPointer(node+0x230,4),4);const auto rows=object+0x190;
    if(pc==0x29f674 && (c->r[9].uw[0]!=stack+0x30 || c->r[10].uw[0]!=rows || c->r[8].uw[0]!=stack+0x20))throw std::runtime_error("native list box query source mismatch");
    if(pc!=0x29f698)return true;
    if(c->r[8].uw[0]!=stack+0x20)throw std::runtime_error("native list box query destination mismatch");
    // The query point is the self+7f0 sphere cache. The getter (virtual +5c)
    // returns before writing it while the model at +7bc is still null, and
    // 23c888 assigns the model in the same frame, so on an object's first
    // frame the point is whatever the allocation held (any bit pattern, VU
    // saturation possible): the engine's PS2 float arithmetic (ps2f) computes it.
    VuWords m[4];for(unsigned k=0;k<4;++k)m[k]=vuWords(rt,rows+16*k);const auto p=vuWords(rt,stack+0x30);VuWords out;
    for(unsigned i=0;i<4;++i)out[i]=vuAdd(vuAdd(vuAdd(vuMul(m[0][i],p[0]),vuMul(m[1][i],p[1])),vuMul(m[2][i],p[2])),vuMul(m[3][i],p[3]));
    std::memcpy(record(rt,stack+0x20,16),out.data(),16);return true;
}
// mod001 23e0d0 (after the 29f5e0 query; s0 = hit node, s2 = owner, s3 = self):
//  23e1bc..23e228  rows A = s0+270 composed with B = s3+90 into SP+e0:
//                  row k = ((B0*a.x + B1*a.y) + B2*a.z) + B3*a.w.
//  23e300..23e358  the rows at SP+e0 (216e40 result, swapped in by EE copies)
//                  . the point SP+d0 into s2+2b0, row sums in the 1fe790 orders.
bool nativeHitFrame(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool compose=pc>=0x23e1bc && pc<=0x23e228,product=pc>=0x23e300 && pc<=0x23e358;
    if(!(compose || product) || (pc&3))return false;
    if(!c)throw std::runtime_error("native hit frame has no CPU record");
    const auto stack=c->r[29].uw[0];
    if(compose){
        const auto a=c->r[16].uw[0]+0x270,b=c->r[19].uw[0]+0x90;
        if(c->r[4].uw[0]!=a || c->r[3].uw[0]!=b || c->r[2].uw[0]!=stack+0xe0)throw std::runtime_error("native hit frame composition escaped its records");
        if(pc!=0x23e21c)return true;
        Lanes A[4],B[4],out[4];for(unsigned k=0;k<4;++k){A[k]=leafRecord(rt,a+16*k);B[k]=leafRecord(rt,b+16*k);}
        {music::NearestRounding rounding;
            for(unsigned k=0;k<4;++k)for(unsigned i=0;i<4;++i)
                out[k][i]=pairSum(pairSum(pairSum(pairProduct(B[0][i],A[k][0]),pairProduct(B[1][i],A[k][1])),pairProduct(B[2][i],A[k][2])),pairProduct(B[3][i],A[k][3]));}
        for(const auto &row:out)for(float x:row)if(!std::isfinite(x))throw std::runtime_error("native hit frame composition overflow");
        std::memcpy(record(rt,stack+0xe0,64),out,64);return true;
    }
    const auto target=c->r[18].uw[0]+0x2b0;
    if(c->r[7].uw[0]!=stack+0xd0 || c->r[8].uw[0]!=stack+0xe0 || c->r[6].uw[0]!=target)throw std::runtime_error("native hit frame product escaped its records");
    if(pc!=0x23e358)return true;
    Lanes m[4],out;for(unsigned k=0;k<4;++k)m[k]=leafRecord(rt,stack+0xe0+16*k);const auto v=leafRecord(rt,stack+0xd0);
    {music::NearestRounding rounding;Lanes p[4];for(unsigned k=0;k<4;++k)for(unsigned i=0;i<4;++i)p[k][i]=pairProduct(m[k][i],v[i]);
        constexpr unsigned order[4][4]={{0,1,2,3},{1,0,2,3},{2,1,0,3},{3,1,2,0}};
        for(unsigned k=0;k<4;++k)out[k]=pairSum(pairSum(pairSum(p[k][order[k][0]],p[k][order[k][1]]),pairProduct(1.f,p[k][order[k][2]])),pairProduct(1.f,p[k][order[k][3]]));}
    for(float x:out)if(!std::isfinite(x))throw std::runtime_error("native hit frame product overflow");
    std::memcpy(record(rt,target,16),out.data(),16);return true;
}
// mod001 214590 plane distance (a0 = point, a1 = normal, a2 = origin; 20d9d0
// passes the self+7f0 sphere cache as a0, so it computes in ps2f):
// SP+0 = (((p-o)*n).x + ((p-o)*n).y) + 1*((p-o)*n).z, then the point's raw Y/Z/W
// (VF16 is only written in X); the caller reads X as the return value.
bool nativePlaneDistance(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc<0x214598 || pc>0x2145b8 || (pc&3))return false;
    if(!c)throw std::runtime_error("native plane distance has no CPU record");
    if(pc!=0x2145b8)return true;
    const auto stack=c->r[29].uw[0];if(c->r[2].uw[0]!=stack)throw std::runtime_error("native plane distance destination mismatch");
    auto out=vuWords(rt,c->r[4].uw[0]);const auto n=vuWords(rt,c->r[5].uw[0]),o=vuWords(rt,c->r[6].uw[0]);uint32_t m[3];
    for(unsigned i=0;i<3;++i)m[i]=vuMul(vuSub(out[i],o[i]),n[i]);
    out[0]=vuAdd(vuAdd(m[0],m[1]),vuMul(0x3f800000u,m[2]));
    std::memcpy(record(rt,stack,16),out.data(),16);return true;
}
// mod001 20d9d0 sphere sweep (s6 = self+7f0 sphere cache, s2 = list node,
// s5 = owner, s4 = self+7e0 scale); ps2f arithmetic (stale cache inputs):
//  20dc20..20dc34  (node+240 - cache)^2 on all lanes to SP+d0.
//  20dd34..20dd48  the same to SP+c0.
//  20e49c..20e4cc  row k of s5+240 scaled by lane k of *s4, in place.
bool nativeSphereSweep(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool first=pc>=0x20dc20 && pc<=0x20dc34,second=pc>=0x20dd34 && pc<=0x20dd48,scale=pc>=0x20e49c && pc<=0x20e4cc;
    if(!(first || second || scale) || (pc&3))return false;
    if(!c)throw std::runtime_error("native sphere sweep has no CPU record");
    const auto stack=c->r[29].uw[0];
    if(scale){
        const auto rows=c->r[21].uw[0]+0x240;
        if(c->r[4].uw[0]!=rows || (pc>=0x20e4c0 && c->r[3].uw[0]!=rows))throw std::runtime_error("native sphere sweep scale escaped its records");
        if(pc!=0x20e4c0)return true;
        const auto s=vuWords(rt,c->r[20].uw[0]);VuWords m[4];for(unsigned k=0;k<4;++k)m[k]=vuWords(rt,rows+16*k);
        for(unsigned k=0;k<4;++k)for(unsigned i=0;i<4;++i)m[k][i]=vuMul(m[k][i],s[k]);
        std::memcpy(record(rt,rows,64),m,64);return true;
    }
    const auto node=c->r[18].uw[0]+0x240,target=first?stack+0xd0:stack+0xc0;
    if((first && c->r[30].uw[0]!=node) || (second && pc<=0x20dd38 && c->r[3].uw[0]!=node))throw std::runtime_error("native sphere sweep source mismatch");
    if(pc!=0x20dc34 && pc!=0x20dd48)return true;
    if(c->r[3].uw[0]!=target)throw std::runtime_error("native sphere sweep destination mismatch");
    const auto a=vuWords(rt,c->r[22].uw[0]),b=vuWords(rt,node);VuWords out;
    for(unsigned i=0;i<4;++i){const auto d=vuSub(b[i],a[i]);out[i]=vuMul(d,d);}
    std::memcpy(record(rt,target,16),out.data(),16);return true;
}
// mod003 3ef010 (s0 = owner; a2 = *(*(48e048)+14), SP+90 = a2+250 - a2+40 by
// EE code), ps2f arithmetic:
//  A 3ef07c..3ef09c  (sqrt((x2+y2)+1*z2), y2, z2, raw W) of SP+90 to SP+40.
//  B 3ef0d4..3ef0f0  (when that length >= 300) *a3*1 + SP+90*f0 to SP+90, raw W.
//  C 3ef100..3ef160  segment s0+280 -> s0+290 and point *a3: d = b-a,
//                    t = (((p-a)*d).x+y)+z / ((d*d).y+x)+z, a + d*t to SP+80,
//                    d to SP+70, p-a to SP+60, p-b to SP+50 (XYZ only: their W
//                    lanes are inherited VU state; SP+80's W is set to 1 by EE
//                    code and the others are dead locals).
//  D 3ef170..3ef184  ((SP+70 * SP+60).x+y)+z with SP+70's raw Y/Z/W to SP+30.
//  E 3ef1b4..3ef1c8  the same with SP+50, to SP+20.
bool nativeSegmentApproach(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool length=pc>=0x3ef07c && pc<=0x3ef09c,clamp=pc>=0x3ef0d4 && pc<=0x3ef0f0,project=pc>=0x3ef100 && pc<=0x3ef160,
               dotA=pc>=0x3ef170 && pc<=0x3ef184,dotB=pc>=0x3ef1b4 && pc<=0x3ef1c8;
    if(!(length || clamp || project || dotA || dotB) || (pc&3))return false;
    if(!c)throw std::runtime_error("native segment approach has no CPU record");
    const auto stack=c->r[29].uw[0];const auto reg=[&](unsigned r){return c->r[r].uw[0];};
    const auto mismatch=[](){throw std::runtime_error("native segment approach escaped its records");};
    const auto rooted=[](auto f){const auto saved=_mm_getcsr();const auto v=f();_mm_setcsr(saved);return v;};
    if(length){
        if(reg(4)!=stack+0x90 || reg(3)!=stack+0x40)mismatch();
        if(pc!=0x3ef09c)return true;
        auto out=vuWords(rt,stack+0x90);for(unsigned i=0;i<3;++i)out[i]=vuMul(out[i],out[i]);
        const auto sum=vuAdd(vuAdd(out[0],out[1]),vuMul(0x3f800000u,out[2]));
        out[0]=vuAdd(0,rooted([&]{uint32_t f=0;return bozo::ps2f::sqrt(sum,f);}));
        std::memcpy(record(rt,stack+0x40,16),out.data(),16);return true;
    }
    if(clamp){
        if(reg(4)!=stack+0x90 || (pc>=0x3ef0e4 && reg(3)!=stack+0x90))mismatch();
        if(pc!=0x3ef0f0)return true;
        auto out=vuWords(rt,stack+0x90);const auto base=vuWords(rt,reg(7));const auto scale=std::bit_cast<uint32_t>(c->f[0]);
        for(unsigned i=0;i<3;++i)out[i]=vuAdd(vuMul(base[i],0x3f800000u),vuMul(out[i],scale));
        std::memcpy(record(rt,stack+0x90,16),out.data(),16);return true;
    }
    if(project){
        const auto owner=reg(16);
        if(pc==0x3ef100 && (reg(3)!=owner+0x280 || reg(2)!=owner+0x290))mismatch();
        if(pc<0x3ef154)return true;
        if(reg(5)!=stack+0x80 || reg(4)!=stack+0x70 || reg(3)!=stack+0x60 || reg(2)!=stack+0x50)mismatch();
        if(pc!=0x3ef154)return true;
        const auto a=vuWords(rt,owner+0x280),b=vuWords(rt,owner+0x290),p=vuWords(rt,reg(7));
        VuWords d{},e{},f{},g{},sq{},pr{};
        for(unsigned i=0;i<3;++i){d[i]=vuSub(b[i],a[i]);e[i]=vuSub(p[i],a[i]);sq[i]=vuMul(d[i],d[i]);pr[i]=vuMul(e[i],d[i]);}
        const auto num=vuAdd(vuAdd(pr[0],pr[1]),pr[2]),den=vuAdd(vuAdd(sq[1],sq[0]),sq[2]);
        const auto t=rooted([&]{uint32_t fl=0;return bozo::ps2f::div(num,den,fl);});
        for(unsigned i=0;i<3;++i){f[i]=vuAdd(a[i],vuMul(d[i],t));g[i]=vuSub(p[i],b[i]);}
        std::memcpy(record(rt,stack+0x80,16),f.data(),12);std::memcpy(record(rt,stack+0x70,16),d.data(),12);
        std::memcpy(record(rt,stack+0x60,16),e.data(),12);std::memcpy(record(rt,stack+0x50,16),g.data(),12);return true;
    }
    const auto other=dotA?stack+0x60:stack+0x50,target=dotA?stack+0x30:stack+0x20;
    if(reg(4)!=stack+0x70 || reg(3)!=other || reg(2)!=target)mismatch();
    if(pc!=(dotA?0x3ef184u:0x3ef1c8u))return true;
    auto out=vuWords(rt,stack+0x70);const auto v=vuWords(rt,other);
    uint32_t m[3];for(unsigned i=0;i<3;++i)m[i]=vuMul(out[i],v[i]);out[0]=vuAdd(vuAdd(m[0],m[1]),m[2]);
    std::memcpy(record(rt,target,16),out.data(),16);return true;
}
// mod001 248e40 world position (a0 = object, a1 = output), with the resident
// mode-0 common-math programs (ps2f, in their instruction order):
//  0x728  rows R0..R2 from the quaternion q = a0+30 (2q products), W column
//         t = a0+20 (1*t), R3 = (0,0,0,1); the scale s = a0+40 gets W = 1.
//  parent P = *(a0+208)+d0 (when set): 0xae8 rows k = ((R0*P_k.x + R1*P_k.y)
//         + R2*P_k.z) + R3*P_k.w, then * s; when (a0+20c & 1c0) == 0, 0x830
//         normalizes each lane over the three rows (RSQRT of the lane sums of
//         squares) and EE code scales by s again. No parent: rows R_k * s.
//  rows M = a0+110..140: SP+k = ((M0*v.x + M1*v.y) + M2*v.z) + M3*v.w for the
//  three rows, SP+30 = M3. EE code then copies the W lanes to a1 (W = 1).
bool nativeWorldPosition(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc<0x248e64 || pc>0x248f90 || (pc&3))return false;
    if(!c)throw std::runtime_error("native world position has no CPU record");
    if(pc==0x248e64){
        const auto object=c->r[4].uw[0];uint32_t parent;std::memcpy(&parent,rt->memory.hostPointer(object+0x208,4),4);
        if(c->r[3].uw[0]!=object+0x40 || c->r[6].uw[0]!=(parent?parent+0xd0:0))throw std::runtime_error("native world position sources mismatch");
        return true;
    }
    if(pc==0x248eb4){c->r[9].ud[0]=0;return true;}     // CFC2 of VI0 after the 0xae8 program
    if(pc!=0x248f18 && pc!=0x248f84)return true;
    const auto stack=c->r[29].uw[0],object=c->r[3].uw[0]-0x110;const bool parented=pc==0x248f18;
    if(c->r[4].uw[0]!=stack || (parented && !c->r[6].uw[0]))throw std::runtime_error("native world position destination mismatch");
    const auto q=vuWords(rt,object+0x30),t=vuWords(rt,object+0x20);auto s=vuWords(rt,object+0x40);s[3]=0x3f800000u;
    const uint32_t one=0x3f800000u;
    // 0x728, upper instructions in order (lower moves: vf11 = vf0, vf17.w = 1).
    VuWords v6,v7,v12,v13,v14,v15,r8{},r9{},r10{};
    for(unsigned i=0;i<4;++i){v6[i]=vuMul(q[i],q[i]);v7[i]=vuAdd(q[i],q[i]);}
    r8[0]=vuAdd(0,one);r9[1]=vuAdd(0,one);for(unsigned i=0;i<4;++i)v6[i]=vuAdd(v6[i],v6[i]);
    for(unsigned i=0;i<4;++i){v12[i]=vuMul(q[i],v7[0]);v13[i]=vuMul(q[i],v7[3]);v14[i]=vuMul(q[i],v7[1]);}
    r8[0]=vuSub(r8[0],v6[1]);for(unsigned i=0;i<4;++i)v15[i]=vuMul(q[i],v7[2]);
    r8[1]=vuAdd(v12[1],v13[2]);r8[2]=vuSub(v12[2],v13[1]);r8[0]=vuSub(r8[0],v6[2]);r9[1]=vuSub(r9[1],v6[0]);r10[2]=vuAdd(0,one);
    r9[0]=vuSub(v14[0],v13[2]);r9[2]=vuAdd(v14[2],v12[3]);r9[1]=vuSub(r9[1],v6[2]);r10[2]=vuSub(r10[2],v6[0]);
    r10[0]=vuAdd(v15[0],v13[1]);r10[1]=vuSub(v15[1],v12[3]);r8[3]=vuMul(one,t[0]);r10[2]=vuSub(r10[2],v6[1]);r9[3]=vuMul(one,t[1]);r10[3]=vuMul(one,t[2]);
    const VuWords r11{0,0,0,one};VuWords rows[3];
    if(parented){
        const auto parent=c->r[6].uw[0];
        for(unsigned k=0;k<3;++k){const auto p=vuWords(rt,parent+16*k);
            for(unsigned i=0;i<4;++i)rows[k][i]=vuMul(s[i],vuAdd(vuAdd(vuAdd(vuMul(r8[i],p[0]),vuMul(r9[i],p[1])),vuMul(r10[i],p[2])),vuMul(r11[i],p[3])));}
        uint16_t flags;std::memcpy(&flags,rt->memory.hostPointer(object+0x20c,2),2);
        if(!(flags&0x1c0)){
            VuWords sum;for(unsigned i=0;i<4;++i)sum[i]=vuAdd(vuAdd(vuMul(rows[0][i],rows[0][i]),vuMul(rows[1][i],rows[1][i])),vuMul(rows[2][i],rows[2][i]));
            const auto saved=_mm_getcsr();
            for(unsigned i=0;i<3;++i){uint32_t f=0;const auto inv=bozo::ps2f::rsqrt(one,sum[i],f);for(auto &row:rows)row[i]=vuMul(row[i],inv);}
            _mm_setcsr(saved);
            for(auto &row:rows)for(unsigned i=0;i<4;++i)row[i]=vuMul(row[i],s[i]);
        }
    }else{const VuWords *r[3]={&r8,&r9,&r10};for(unsigned k=0;k<3;++k)for(unsigned i=0;i<4;++i)rows[k][i]=vuMul(s[i],(*r[k])[i]);}
    VuWords m[4];for(unsigned k=0;k<4;++k)m[k]=vuWords(rt,object+0x110+16*k);
    VuWords out[4];for(unsigned k=0;k<3;++k)for(unsigned i=0;i<4;++i)
        out[k][i]=vuAdd(vuAdd(vuAdd(vuMul(m[0][i],rows[k][0]),vuMul(m[1][i],rows[k][1])),vuMul(m[2][i],rows[k][2])),vuMul(m[3][i],rows[k][3]));
    out[3]=m[3];std::memcpy(record(rt,stack,64),out,64);return true;
}
// main 10da70 camera target (s1 = camera owner, s0 = target point), ps2f:
//  10dc44..10dc68  ((R0*p.x + R1*p.y) + R2*p.z) + R3*1, R = *(s1+1390)+190,
//                  all lanes to SP+80 (the point's own W is not a weight).
//  10dc84..10dc98  (((x2+y2)+1*z2), y2, z2, raw W) of SP+80 (Z adjusted by
//                  EE code in between) to SP+50.
bool nativeCameraTarget(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool transform=pc>=0x10dc44 && pc<=0x10dc68,length=pc>=0x10dc84 && pc<=0x10dc98;
    if(!(transform || length) || (pc&3))return false;
    if(!c)throw std::runtime_error("native camera target has no CPU record");
    const auto stack=c->r[29].uw[0];
    if(transform){
        uint32_t owner;std::memcpy(&owner,rt->memory.hostPointer(c->r[17].uw[0]+0x1390,4),4);
        if(c->r[3].uw[0]!=owner+0x190 || c->r[2].uw[0]!=stack+0x80)throw std::runtime_error("native camera target transform escaped its records");
        if(pc!=0x10dc68)return true;
        VuWords m[4];for(unsigned k=0;k<4;++k)m[k]=vuWords(rt,owner+0x190+16*k);const auto p=vuWords(rt,c->r[16].uw[0]);VuWords out;
        for(unsigned i=0;i<4;++i)out[i]=vuAdd(vuAdd(vuAdd(vuMul(m[0][i],p[0]),vuMul(m[1][i],p[1])),vuMul(m[2][i],p[2])),vuMul(m[3][i],0x3f800000u));
        std::memcpy(record(rt,stack+0x80,16),out.data(),16);return true;
    }
    if(c->r[3].uw[0]!=stack+0x80 || c->r[2].uw[0]!=stack+0x50)throw std::runtime_error("native camera target length escaped its records");
    if(pc!=0x10dc98)return true;
    auto out=vuWords(rt,stack+0x80);for(unsigned i=0;i<3;++i)out[i]=vuMul(out[i],out[i]);
    out[0]=vuAdd(vuAdd(out[0],out[1]),vuMul(0x3f800000u,out[2]));
    std::memcpy(record(rt,stack+0x50,16),out.data(),16);return true;
}
// Vector lengths: LQC2 vf24 <- src; MUL.xyz squares; ADDW.xyz vf25 = 1;
// ADDAy/MADDz x = (x2+y2)+1*z2; SQRT; ADDQ.x x = 0+Q; SQC2 vf24 -> dst. The
// stored record is (length, y2, z2, raw W): every lane is defined. ps2f.
// Each side is a register that must hold base register + offset.
struct RootLengthSite{uint32_t first,store;unsigned sourceReg,sourceBase;uint32_t sourceOffset;unsigned targetReg,targetBase;uint32_t targetOffset;};
constexpr RootLengthSite rootLengthSites[]={
    {0x42d390,0x42d3b0,3,29,0x100,2,29,0x70},   // mod003 42d280: SP+100 -> SP+70
    {0x3d0514,0x3d0534,4,20,0x20,3,29,0xa0},    // mod003 3cfde0: s4+20 -> SP+a0
};
bool nativeRootLength(R5900Context *c,Runtime *rt,uint32_t pc){
    const auto site=std::find_if(std::begin(rootLengthSites),std::end(rootLengthSites),[&](const auto &s){return pc>=s.first && pc<=s.store;});
    if(site==std::end(rootLengthSites) || (pc&3))return false;
    if(!c)throw std::runtime_error("native root length has no CPU record");
    const auto source=c->r[site->sourceBase].uw[0]+site->sourceOffset,target=c->r[site->targetBase].uw[0]+site->targetOffset;
    if(c->r[site->sourceReg].uw[0]!=source || c->r[site->targetReg].uw[0]!=target)throw std::runtime_error("native root length escaped its records");
    if(pc!=site->store)return true;
    auto out=vuWords(rt,source);for(unsigned i=0;i<3;++i)out[i]=vuMul(out[i],out[i]);
    const auto sum=vuAdd(vuAdd(out[0],out[1]),vuMul(0x3f800000u,out[2]));
    const auto saved=_mm_getcsr();uint32_t f=0;const auto root=bozo::ps2f::sqrt(sum,f);_mm_setcsr(saved);
    out[0]=vuAdd(0,root);std::memcpy(record(rt,target,16),out.data(),16);return true;
}
// mod001 220d20 visibility loop. Per object (s5) the sphere getter (virtual
// +5c) returns v0; VCALLMS 0xb60 tests it against the frustum that 207cc0
// last wrote to VU0 data qwords 8..19 (points camera+460, normals camera+500),
// and CFC2 reads VI2 = status & 0x80 (sticky sign). The loop keeps that camera
// in s0 ? s1 : s2 (s0 set after 207cc0(s4 = s1), cleared after 207cc0(s2)).
// 0xb60's FSSET (pair c50) lands after every earlier flag write; the two
// ADDW.xyz (d+r) at c58/c60 land before FSAND at c80 (>= 4 cycles); the
// SUBW.yz at c68 does not (3 cycles, no stall in between). So VI2 = 0x80 iff
// a sign bit is set among d_i + r, with d_i the program's lane sums of
// (s - P_i) * N_i: rows 0/3 (x+y)+z, 1/4 (y+x)+z, 2/5 (z+y)+x. The camera
// matrices (0x1f0 into vf1..4, +3f0 into vf24..27) feed only 0xb60's
// projected outputs (vf6/vf8/vf9), which nothing reads: those loads are no-ops.
bool nativeVisibilityLoop(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool load=(pc>=0x220d70 && pc<=0x220d7c) || (pc>=0x220dd8 && pc<=0x220e00) || (pc>=0x220f04 && pc<=0x220f40) ||
                    (pc>=0x220f64 && pc<=0x220fa0) || pc==0x220fb8 || pc==0x220fc4;
    if(!(load || pc==0x220fd0) || (pc&3))return false;
    if(load)return true;
    if(!c)throw std::runtime_error("native visibility test has no CPU record");
    const auto sphere=c->r[2].uw[0],camera=(c->r[16].uw[0]&0xff)?c->r[17].uw[0]:c->r[18].uw[0];
    if((sphere&15) || !camera || (camera&15))throw std::runtime_error("native visibility test records misaligned");
    const auto s=vuWords(rt,sphere);bool negative=false;
    constexpr unsigned order[6][3]={{0,1,2},{1,0,2},{2,1,0},{0,1,2},{1,0,2},{2,1,0}};
    for(unsigned i=0;i<6;++i){const auto p=vuWords(rt,camera+0x460+16*i),n=vuWords(rt,camera+0x500+16*i);uint32_t m[3];
        for(unsigned k=0;k<3;++k)m[k]=vuMul(vuSub(s[k],p[k]),n[k]);
        const auto d=vuAdd(vuAdd(m[order[i][0]],m[order[i][1]]),vuMul(0x3f800000u,m[order[i][2]]));
        negative=negative || (vuAdd(d,s[3])&0x80000000u);}
    c->r[3].ud[0]=negative?0x80:0;return true;
}
// mod001 2156d0 quaternion normalize leaf: *a0 through program 0x670 (sum
// ((x2+y2)+1*z2)+1*w2, RSQRT, all lanes * Q) back to *a0 (the SQC2 sits in the
// JR delay slot); the CFC2 reads VI0. ps2f. (The older 2156b0 site of the same
// program uses math::normalizeQuaternion, an audited truncating helper.)
bool nativeQuaternionLeaf(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x2156d0 && pc!=0x2156d8 && pc!=0x2156dc && pc!=0x2156e4)return false;
    if(!c)throw std::runtime_error("native quaternion leaf has no CPU record");
    if(pc==0x2156dc){c->r[9].ud[0]=0;return true;}
    if(pc!=0x2156e4)return true;
    const auto at=c->r[4].uw[0];auto q=vuWords(rt,at);
    uint32_t sq[4];for(unsigned i=0;i<4;++i)sq[i]=vuMul(q[i],q[i]);
    const auto sum=vuAdd(vuAdd(vuAdd(sq[0],sq[1]),vuMul(0x3f800000u,sq[2])),vuMul(0x3f800000u,sq[3]));
    const auto saved=_mm_getcsr();uint32_t f=0;const auto inv=bozo::ps2f::rsqrt(0x3f800000u,sum,f);_mm_setcsr(saved);
    for(auto &x:q)x=vuMul(x,inv);
    std::memcpy(record(rt,at,16),q.data(),16);return true;
}
// mod001 237320 bone palette. Bone j (j-th record *(s2+4j), inverse bind
// *(s4+40)+64j, object rows O = SP+60 from 216e40) through program 0x898:
//  M_k = ((IB0*B_k.x + IB1*B_k.y) + IB2*B_k.z) + IB3*B_k.w (k<3), M_3 = B_3;
//  N_k = ((M0*O_k.x + M1*O_k.y) + M2*O_k.z) + M3*O_k.w (k<3), N_3 = M_3;
// then the B at 0x948 (target pc+8+imm*8 = 0x1f0) runs the whole transpose
// P = N^T (0x1f0..0x248). Transposed lanes: MOVE (raw), ADD 0+x, MUL 1*x as
// listed. EE stores P to the scratchpad palette s1 = 70000010 + 64j after the
// interlocked CFC2 (the next bone's input loads overlap the run, by design).
bool nativeBonePalette(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool load=(pc>=0x237450 && pc<=0x23748c) || (pc>=0x2374c8 && pc<=0x2374e4) || pc==0x237504 || pc==0x2374e8;
    const bool store=(pc>=0x2374ec && pc<=0x2374f8) || (pc>=0x237580 && pc<=0x237590);
    if(!(load || store) || (pc&3))return false;
    if(!c)throw std::runtime_error("native bone palette has no CPU record");
    if(pc==0x237580){c->r[9].ud[0]=0;return true;}
    if(load || (pc!=0x2374ec && pc!=0x237584))return true;
    const auto s1=c->r[17].uw[0];if(s1<0x70000010u || ((s1-0x70000010u)&63) || s1>=0x70004000u)throw std::runtime_error("native bone palette outside the scratchpad palette");
    const auto j=(s1-0x70000010u)/64;
    uint32_t bone,binds,count;std::memcpy(&bone,rt->memory.hostPointer(c->r[18].uw[0]+4*j,4),4);std::memcpy(&binds,rt->memory.hostPointer(c->r[20].uw[0]+0x40,4),4);
    {uint32_t list;std::memcpy(&list,rt->memory.hostPointer(c->r[20].uw[0]+0x34,4),4);std::memcpy(&count,rt->memory.hostPointer(list+4,4),4);}
    if(int32_t(count)<=0 || j>=count)throw std::runtime_error("native bone palette index outside the skeleton");
    VuWords B[4],IB[4],O[3],M[4],N[4];const auto stack=c->r[29].uw[0];
    for(unsigned k=0;k<4;++k){B[k]=vuWords(rt,bone+0x90+16*k);IB[k]=vuWords(rt,binds+64*j+16*k);}
    for(unsigned k=0;k<3;++k)O[k]=vuWords(rt,stack+0x60+16*k);
    const auto rows=[](const VuWords *R,const VuWords &v){VuWords o;for(unsigned i=0;i<4;++i)o[i]=vuAdd(vuAdd(vuAdd(vuMul(R[0][i],v[0]),vuMul(R[1][i],v[1])),vuMul(R[2][i],v[2])),vuMul(R[3][i],v[3]));return o;};
    for(unsigned k=0;k<3;++k)M[k]=rows(IB,B[k]);M[3]=B[3];
    for(unsigned k=0;k<3;++k)N[k]=rows(M,O[k]);N[3]=M[3];
    const uint32_t one=0x3f800000u;VuWords P[4];
    P[0]={N[0][0],vuAdd(0,N[1][0]),vuAdd(0,N[2][0]),vuMul(one,N[3][0])};
    P[1]={vuAdd(0,N[0][1]),N[1][1],vuAdd(0,N[2][1]),vuMul(one,N[3][1])};
    P[2]={vuAdd(0,N[0][2]),vuAdd(0,N[1][2]),N[2][2],vuMul(one,N[3][2])};
    P[3]={vuAdd(0,N[0][3]),vuAdd(0,N[1][3]),vuAdd(0,N[2][3]),N[3][3]};
    std::memcpy(record(rt,s1,64),P,64);return true;
}
// VU0 data qwords 16.. as 237320 leaves them: its tail writes a V4-32 UNPACK
// of the scratchpad palette (4n qwords to VU0 qword 16) and starts VIF0. The
// only readers are the mode-1 skinning programs that 218220 runs. Natively the
// palette is kept at that kick; no VIF/DMA transfer or completion exists.
struct SkinPalette{const Runtime *owner=nullptr;uint32_t bones=0;std::vector<VuWords> rows;};
// One VU0 per runtime (tests run an original and a native runtime side by side).
SkinPalette &skinPalette(const Runtime *rt){static std::map<const Runtime *,SkinPalette> palettes;return palettes[rt];}
uint32_t skinWord(Runtime *rt,uint32_t at){
    if(at&3)throw std::runtime_error("native skinning word misaligned");
    const bool scratch=at>=0x70000000u && at<0x70004000u;
    const auto *p=rt->memory.hostPointer(at,4);if(!p || (!scratch && (at&0x1ffffffu)<0x100000))throw std::runtime_error("native skinning record outside EE RAM");
    uint32_t v;std::memcpy(&v,p,4);return v;}
uint8_t skinByte(Runtime *rt,uint32_t at){
    const bool scratch=at>=0x70000000u && at<0x70004000u;
    const auto *p=rt->memory.hostPointer(at,1);if(!p || (!scratch && (at&0x1ffffffu)<0x100000))throw std::runtime_error("native skinning stream outside EE RAM");return *p;}
bool nativeSkinUpload(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x237568 && pc!=0x23756c && pc!=0x237574 && pc!=0x23757c && pc!=0x2375a4)return false;
    if(!c)throw std::runtime_error("native skin palette upload has no CPU record");
    // 237568 busy byte, 23756c QWC, 237574 MADR, 23757c D_STAT, 2375a4 CHCR:
    // no transfer is started, so no completion will clear a busy flag.
    if(pc!=0x2375a4)return true;
    const auto skeleton=c->r[20].uw[0];const auto bones=skinWord(rt,skinWord(rt,skeleton+0x34)+4);
    if(int32_t(bones)<1 || bones>60)throw std::runtime_error("native skin palette outside VU0 data memory");
    uint32_t tag;std::memcpy(&tag,record(rt,0x70000000,16)+12,4);
    if(c->r[6].uw[0]!=4*bones+1 || tag!=(0x6c000010u|(4*bones)<<16))throw std::runtime_error("native skin palette upload escaped its original tag");
    auto &palette=skinPalette(rt);palette.owner=rt;palette.bones=bones;palette.rows.resize(4*bones);
    for(uint32_t i=0;i<4*bones;++i)palette.rows[i]=vuWords(rt,0x70000010+16*i);
    return true;
}
// mod001 218220 (skeleton vtable +34): software skinning of one mesh with the
// mode-1 programs, palette rows R at VU0 qwords 16+4j as uploaded above.
// rot(R,v) = ((R0*v.x + R1*v.y) + R2*v.z), aff(R,p) = rot(R,p) + R3*1 (all
// lanes); weighted sums accumulate left to right as one VU ACC chain. Records:
// desc+0 count (at least one vertex is processed), +4 positions, +8 normals
// (16 bytes each, in place), +14 packed s16x4 vectors (236bc0 path, 0 = none),
// +1c+4i table: +6 rigid flag, +8 bone-address bytes, +c weights, +10 counts.
// Faithful quirks of the listings (the PS2 computes them):
// - 0x340..: T = itof12(packed); out vector = ftoi12(blend of rot(R,T)).xyz | T.w.
// - 0x3b8: the normal's second term is the blended T (it reuses vf29).
// - 0x4b0: weight 1 is B0.x (lq vf19 replaces it); the normal's second term is
//   the blended T, its third rot(C, rot(B,T)).
// - 0x610: the normal's fourth term is rot(D, aff(C,P)) (vf24 reused).
struct SkinVertex{VuWords p,n,t;bool packed;uint32_t bone[4];uint32_t w[4];unsigned count;};
VuWords skinRot(const VuWords *R,const VuWords &v){VuWords o;for(unsigned i=0;i<4;++i)o[i]=vuAdd(vuAdd(vuMul(R[0][i],v[0]),vuMul(R[1][i],v[1])),vuMul(R[2][i],v[2]));return o;}
VuWords skinAff(const VuWords *R,const VuWords &v){auto o=skinRot(R,v);for(unsigned i=0;i<4;++i)o[i]=vuAdd(o[i],vuMul(R[3][i],0x3f800000u));return o;}
VuWords skinBlend(std::initializer_list<std::pair<VuWords,uint32_t>> terms){
    VuWords acc{};bool first=true;
    for(const auto &[v,w]:terms){for(unsigned i=0;i<4;++i)acc[i]=first?vuMul(v[i],w):vuAdd(acc[i],vuMul(v[i],w));first=false;}
    return acc;}
uint32_t skinItof12(uint32_t v){volatile float f=float(int32_t(v))*(1.f/4096.f);return std::bit_cast<uint32_t>(f);}
uint32_t skinFtoi12(uint32_t v){
    // PS2 floats have no infinities or NaNs: exponent 255 is just large.
    if((v&0x7f800000)==0x7f800000)return v&0x80000000?0x80000000u:0x7fffffffu;
    const auto b=(v&0x7f800000)?v:(v&0x80000000);const double fixed=double(std::bit_cast<float>(b))*4096.;
    if(fixed>=2147483647.)return 0x7fffffffu;if(fixed<=-2147483648.)return 0x80000000u;return uint32_t(int32_t(fixed));}
void skinVertex(const SkinPalette &palette,const SkinVertex &v,VuWords &position,VuWords &normal,VuWords &packed){
    const uint32_t one=0x3f800000u;
    VuWords B[4][4];for(unsigned k=0;k<v.count;++k)for(unsigned r=0;r<4;++r)B[k][r]=palette.rows[4*v.bone[k]+r];
    VuWords P,N,T{};for(unsigned i=0;i<4;++i){P[i]=vuMul(v.p[i],one);N[i]=vuMul(v.n[i],one);if(v.packed)T[i]=skinItof12(v.t[i]);}
    const auto *A=B[0];
    if(!v.packed){
        if(v.count==1){position=skinAff(A,P);normal=skinRot(A,N);return;}
        if(v.count==2){position=skinBlend({{skinAff(A,P),v.w[0]},{skinAff(B[1],P),v.w[1]}});normal=skinBlend({{skinRot(A,N),v.w[0]},{skinRot(B[1],N),v.w[1]}});return;}
        if(v.count==3){position=skinBlend({{skinAff(A,P),v.w[0]},{skinAff(B[1],P),v.w[1]},{skinAff(B[2],P),v.w[2]}});
            normal=skinBlend({{skinRot(A,N),v.w[0]},{skinRot(B[1],N),v.w[1]},{skinRot(B[2],N),v.w[2]}});return;}
        position=skinBlend({{skinAff(A,P),v.w[0]},{skinAff(B[1],P),v.w[1]},{skinAff(B[2],P),v.w[2]},{skinAff(B[3],P),v.w[3]}});
        normal=skinBlend({{skinRot(A,N),v.w[0]},{skinRot(B[1],N),v.w[1]},{skinRot(B[2],N),v.w[2]},{skinRot(B[3],N),v.w[3]}});return;
    }
    VuWords tangent;
    if(v.count==1){position=skinAff(A,P);normal=skinRot(A,N);tangent=skinRot(A,T);}
    else if(v.count==2){
        position=skinBlend({{skinAff(A,P),v.w[0]},{skinAff(B[1],P),v.w[1]}});
        tangent=skinBlend({{skinRot(A,T),v.w[0]},{skinRot(B[1],T),v.w[1]}});tangent[3]=T[3];
        normal=skinBlend({{skinRot(A,N),v.w[0]},{tangent,v.w[1]}});
    }else if(v.count==3){
        const uint32_t w1=B[1][0][0];const auto bt=skinRot(B[1],T);
        position=skinBlend({{skinAff(A,P),w1},{skinAff(B[1],P),v.w[1]},{skinAff(B[2],P),v.w[2]}});
        tangent=skinBlend({{skinRot(A,T),w1},{bt,v.w[1]},{skinRot(B[2],T),v.w[2]}});tangent[3]=T[3];
        normal=skinBlend({{skinRot(A,N),w1},{tangent,v.w[1]},{skinRot(B[2],bt),v.w[2]}});
    }else{
        position=skinBlend({{skinAff(A,P),v.w[0]},{skinAff(B[1],P),v.w[1]},{skinAff(B[2],P),v.w[2]},{skinAff(B[3],P),v.w[3]}});
        tangent=skinBlend({{skinRot(A,T),v.w[0]},{skinRot(B[1],T),v.w[1]},{skinRot(B[2],T),v.w[2]},{skinRot(B[3],T),v.w[3]}});
        normal=skinBlend({{skinRot(A,N),v.w[0]},{skinRot(B[1],N),v.w[1]},{skinRot(B[2],N),v.w[2]},{skinRot(B[3],skinAff(B[2],P)),v.w[3]}});
    }
    tangent[3]=T[3];for(unsigned i=0;i<4;++i)packed[i]=skinFtoi12(tangent[i]);
}
// One mesh in place: `vertices` 16-byte positions and normals and, when
// `packed` is set, 8-byte s16x4 vectors, with the element table at `table`
// (guest RAM). Requires VU0 mode 1 and an uploaded palette, as the programs do.
void skinMesh(Runtime *rt,uint32_t table,unsigned vertices,uint8_t *positions,uint8_t *normals,uint8_t *packed){
    const auto manager=skinWord(rt,0x35ddb8);
    if(skinByte(rt,manager+0x51)!=1)throw std::runtime_error("native skinning outside VU0 mode 1");
    const auto &palette=skinPalette(rt);if(palette.owner!=rt || !palette.bones)throw std::runtime_error("native skinning without an uploaded palette");
    if(!vertices || vertices>0x10000)throw std::runtime_error("native skinning vertex count outside bounds");
    uint32_t bones=table+skinWord(rt,table+8),weights=table+skinWord(rt,table+0xc),counts=table+0x10;
    const bool rigid=skinByte(rt,table+6)!=0;
    const auto address=[&](uint32_t qword){if(qword<16 || (qword-16)%4 || (qword-16)/4>=palette.bones)throw std::runtime_error("native skinning bone outside the palette");return (qword-16)/4;};
    const auto extra=[&](uint8_t b){const int32_t j=int8_t(b);if(j<0 || uint32_t(j)>=palette.bones)throw std::runtime_error("native skinning bone outside the palette");return uint32_t(j);};
    const uint32_t savedCsr=_mm_getcsr();
    for(unsigned k=0;k<vertices;++k){
        SkinVertex v{};v.packed=packed!=nullptr;
        std::memcpy(v.p.data(),positions+16*k,16);std::memcpy(v.n.data(),normals+16*k,16);
        if(v.packed){const auto *h=packed+8*k;for(unsigned i=0;i<4;++i)v.t[i]=uint32_t(int32_t(int16_t(h[2*i]|h[2*i+1]<<8)));}
        if(rigid){v.count=1;v.bone[0]=address(skinByte(rt,bones++));}
        else{
            const unsigned n=skinByte(rt,counts++);v.bone[0]=address(skinByte(rt,bones++));
            v.count=n==0?1:n==1?2:n==2?3:4;
            for(unsigned j=1;j<v.count;++j)v.bone[j]=extra(skinByte(rt,bones++));
            for(unsigned j=0;j<v.count;++j)v.w[j]=skinWord(rt,weights+4*j);
            weights+=4*v.count;
        }
        VuWords position,normal,out{};skinVertex(palette,v,position,normal,out);
        std::memcpy(positions+16*k,position.data(),16);std::memcpy(normals+16*k,normal.data(),16);
        if(v.packed){auto *h=packed+8*k;for(unsigned i=0;i<4;++i){h[2*i]=uint8_t(out[i]);h[2*i+1]=uint8_t(out[i]>>8);}}
    }
    _mm_setcsr(savedCsr);
}
bool nativeSoftSkin(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x21822c)return false;
    if(!c)throw std::runtime_error("native skinning has no CPU record");
    const auto desc=c->r[5].uw[0];const auto index=c->r[6].uw[0];
    const auto packedAt=skinWord(rt,desc+0x14);c->r[3].sd[0]=int32_t(packedAt);   // the replaced lw v1,14(a1)
    const int32_t count=int32_t(skinWord(rt,desc));const unsigned vertices=count<1?1u:unsigned(count);
    if(vertices>0x10000)throw std::runtime_error("native skinning vertex count outside bounds");
    const uint32_t positions=skinWord(rt,desc+4),normals=skinWord(rt,desc+8);
    // Work on copies so a rejected vertex leaves the records untouched.
    std::vector<uint8_t> p(16*vertices),n(16*vertices),t(packedAt?8*vertices:0);
    std::memcpy(p.data(),record(rt,positions,16*vertices),p.size());std::memcpy(n.data(),record(rt,normals,16*vertices),n.size());
    if(packedAt)std::memcpy(t.data(),rt->memory.hostPointer(packedAt,t.size()),t.size());
    skinMesh(rt,skinWord(rt,desc+0x1c+4*index),vertices,p.data(),n.data(),packedAt?t.data():nullptr);
    std::memcpy(record(rt,positions,16*vertices),p.data(),p.size());std::memcpy(record(rt,normals,16*vertices),n.data(),n.size());
    if(packedAt)std::memcpy(rt->memory.hostPointer(packedAt,t.size()),t.data(),t.size());
    return true;
}
bool nativeMathInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc){
    if(nativeSkinUpload(c,rt,pc) || nativeSoftSkin(c,rt,pc))return true;
    if(nativeProjectionBlocks(c,rt,pc) || nativeBoneAim(c,rt,pc) || nativeChainBlocks(c,rt,pc) || nativeCapsulePair(c,rt,pc) || nativePlaneContact(c,rt,pc) || nativeRangeTest(c,rt,pc) || nativeMatrixBuild(c,rt,pc) || nativeListBoxQuery(c,rt,pc) || nativeHitFrame(c,rt,pc) || nativePlaneDistance(c,rt,pc) || nativeSphereSweep(c,rt,pc) || nativeSegmentApproach(c,rt,pc) || nativeWorldPosition(c,rt,pc) || nativeCameraTarget(c,rt,pc) || nativeRootLength(c,rt,pc) || nativeVisibilityLoop(c,rt,pc) || nativeQuaternionLeaf(c,rt,pc) || nativeBonePalette(c,rt,pc))return true;
    if(nativeLeafSphere(c,rt,pc) || nativeLeafBox(c,rt,pc) || nativeSegmentBox(c,rt,pc))return true;
    if(nativeLengthSite(c,rt,pc) || nativeRecordDot(c,rt,pc) || nativePackedQuaternion(c,rt,pc))return true;
    if(nativeObjectOverlap(c,rt,pc))return true;
    if(nativeUvAnimation(c,rt,pc))return true;
    if(nativeSegmentContact(c,rt,pc))return true;
    if(nativeObjectCompose(c,rt,pc))return true;
    if(nativeCameraProjection(c,rt,pc))return true;
    if(pc==0x38011c || pc==0x380124 || pc==0x380154 || pc==0x38015c || pc==0x380160 || pc==0x380164){
        if(!c || c->pc!=0x380118 || c->r[31].uw[0]!=0x380118)throw std::runtime_error("native movement rotation escaped scalar quaternion builder");
        const auto stack=c->r[29].uw[0];
        if((pc==0x38011c || pc==0x380124) && c->r[2].uw[0]!=stack+0x290)throw std::runtime_error("movement quaternion source mismatch");
        if(pc==0x380154)c->r[9].ud[0]=0;
        else if(pc>=0x38015c){
            if(c->r[2].uw[0]!=stack+0x2a0)throw std::runtime_error("movement rotation destination mismatch");
            __m128 input;std::memcpy(&input,record(rt,stack+0x290,16),16);float values[4];std::memcpy(values,&input,16);
            for(float v:values){const auto magnitude=std::bit_cast<uint32_t>(v)&0x7fffffff;if(magnitude>=0x7f800000 || magnitude>std::bit_cast<uint32_t>(1.e8f))throw std::runtime_error("unqualified movement quaternion");}
            const auto result=math::quaternionMatrix(input);const auto row=(pc-0x38015c)/4;std::memcpy(record(rt,stack+0x2a0+16*row,16),&result[row],16);
        }return true;
    }
    if(pc==0x39f31c || pc==0x39f324 || pc==0x39f360 || pc==0x39f368 || pc==0x39f36c || pc==0x39f370){
        // Object rotation from the quaternion its getter wrote at SP+20; the
        // rows land at s0+250, the scalar code builds the identity fourth row.
        if(!c || c->pc!=0x39f318 || c->r[31].uw[0]!=0x39f318)throw std::runtime_error("native object rotation escaped scalar quaternion builder");
        const auto stack=c->r[29].uw[0],object=c->r[16].uw[0];
        if((pc==0x39f31c || pc==0x39f324) && c->r[3].uw[0]!=stack+0x20)throw std::runtime_error("object quaternion source mismatch");
        if(pc==0x39f360)c->r[9].ud[0]=0;
        else if(pc>=0x39f368){
            if(c->r[3].uw[0]!=object+0x250)throw std::runtime_error("object rotation destination mismatch");
            __m128 input;std::memcpy(&input,record(rt,stack+0x20,16),16);float values[4];std::memcpy(values,&input,16);
            for(float v:values){const auto magnitude=std::bit_cast<uint32_t>(v)&0x7fffffff;if(magnitude>=0x7f800000 || magnitude>std::bit_cast<uint32_t>(1.e8f))throw std::runtime_error("unqualified object quaternion");}
            const auto result=math::quaternionMatrix(input);const auto row=(pc-0x39f368)/4;std::memcpy(record(rt,object+0x250+16*row,16),&result[row],16);
        }return true;
    }
    if(pc==0x396c24 || pc==0x396c2c || pc==0x396c5c || pc==0x396c64 || pc==0x396c68 || pc==0x396c6c){
        // mod003 396c00: the quaternion its caller passed is stored at s0+250;
        // program 0x720 turns it into three rows at SP+40, which the scalar
        // code then hands to 410df0 (native) with its own (0,0,1,1) vector.
        if(!c)throw std::runtime_error("native facing rotation has no CPU record");
        const auto stack=c->r[29].uw[0],object=c->r[16].uw[0];
        if(pc==0x396c24 && c->r[3].uw[0]!=object+0x250)throw std::runtime_error("facing quaternion source mismatch");
        if(pc==0x396c5c)c->r[9].ud[0]=0;
        else if(pc>=0x396c64){
            if(c->r[2].uw[0]!=stack+0x40)throw std::runtime_error("facing rotation destination mismatch");
            __m128 input;std::memcpy(&input,record(rt,object+0x250,16),16);float values[4];std::memcpy(values,&input,16);
            for(float v:values){const auto magnitude=std::bit_cast<uint32_t>(v)&0x7fffffff;if(magnitude>=0x7f800000 || magnitude>std::bit_cast<uint32_t>(1.e8f))throw std::runtime_error("unqualified facing quaternion");}
            const auto result=math::quaternionMatrix(input);const auto row=(pc-0x396c64)/4;std::memcpy(record(rt,stack+0x40+16*row,16),&result[row],16);
        }return true;
    }
    if(nativeParticleCallbackInstruction(c,rt,pc))return true;
    if(pc==0x22f300 || pc==0x22f304 || pc==0x22f308 || pc==0x22f30c){
        if(!c || c->pc<0x22f170 || c->pc>0x22f2f8 || !c->r[18].uw[0] || c->r[3].uw[0]!=c->r[20].uw[0]+0x580 || c->r[4].uw[0]!=c->r[18].uw[0]+0x340 || c->r[2].uw[0]!=c->r[29].uw[0]+0xb0)
            throw std::runtime_error("native particle colour product escaped its scalar records");
        if(pc==0x22f30c){
            struct ArithmeticScope{unsigned saved=_mm_getcsr();~ArithmeticScope(){_mm_setcsr(saved);}} arithmetic;
            std::array<float,4> colour,light,result;std::memcpy(colour.data(),record(rt,c->r[3].uw[0],16),16);std::memcpy(light.data(),record(rt,c->r[4].uw[0],16),16);
            for(const auto *input:{&colour,&light})for(float value:*input){pairQualified(value);if(std::abs(value)>1.e8f)throw std::runtime_error("unqualified native particle colour product");}
            auto *output=record(rt,c->r[2].uw[0],16);for(unsigned i=0;i<4;++i)result[i]=pairProduct(colour[i],light[i]);std::memcpy(output,result.data(),16);
        }return true;
    }
    const bool particleSphere=pc>=0x22f9e0 && pc<=0x22fa38 && !(pc&3),originSphere=pc>=0x2482b4 && pc<=0x24830c && !(pc&3);
    if(particleSphere || originSphere){
        const unsigned pointReg=particleSphere?6:3,basisReg=particleSphere?7:5,outputReg=particleSphere?5:2;
        if(!c || c->pc!=(particleSphere?0x22f9c0:0x248290) || c->r[pointReg].uw[0]!=c->r[29].uw[0] || c->r[basisReg].uw[0]!=c->r[4].uw[0]+0x90 || c->r[outputReg].uw[0]!=c->r[4].uw[0]+(particleSphere?0xa70:0x220))
            throw std::runtime_error("native sphere getter escaped its scalar records");
        if(pc==0x22fa38 || pc==0x24830c){
            float rows[3][4],point[4];std::memcpy(rows,record(rt,c->r[basisReg].uw[0],48),48);std::memcpy(point,record(rt,c->r[pointReg].uw[0],16),16);
            for(auto &row:rows)for(float value:row){pairQualified(value);if(std::abs(value)>1.e8f)throw std::runtime_error("unqualified particle sphere basis");}
            for(unsigned i=0;i<3;++i){pairQualified(point[i]);if(std::abs(point[i])>1.e8f)throw std::runtime_error("unqualified particle sphere centre");}
            if(std::bit_cast<uint32_t>(point[3])!=0x3f800000)throw std::runtime_error("particle sphere getter lacks scalar homogeneous component");
            if(originSphere)for(unsigned i=0;i<3;++i)if(std::bit_cast<uint32_t>(point[i]))throw std::runtime_error("origin sphere getter lacks original scalar zero centre");
            auto *output=record(rt,c->r[outputReg].uw[0],16);music::CodecRounding rounding;std::array<float,3> result;
            constexpr unsigned order[3][4]={{0,1,2,3},{1,0,2,3},{2,1,0,3}};
            for(unsigned row=0;row<3;++row){float terms[4];for(unsigned i=0;i<4;++i)terms[i]=pairProduct(rows[row][i],point[i]);
                result[row]=pairSum(pairSum(pairSum(terms[order[row][0]],terms[order[row][1]]),pairProduct(1.f,terms[order[row][2]])),pairProduct(1.f,terms[order[row][3]]));}
            // The original following scalar store supplies the radius. The
            // fourth basis row's temporary dot product has no consumer.
            std::memcpy(output,result.data(),12);
        }return true;
    }
    if(pc>=0x377d8c && pc<=0x377dac && !(pc&3)){
        if(!c || c->pc!=0x377d20 || c->r[7].uw[0]!=c->r[29].uw[0]+0x20 || c->r[6].uw[0]!=c->r[29].uw[0])
            throw std::runtime_error("automatic camera distance escaped its scalar frame");
        if(pc==0x377dac){
            struct ArithmeticScope{unsigned saved=_mm_getcsr();~ArithmeticScope(){_mm_setcsr(saved);}} arithmetic;
            std::array<float,4> result;std::memcpy(result.data(),record(rt,c->r[7].uw[0],16),16);
            for(unsigned i=0;i<3;++i){pairQualified(result[i]);if(std::abs(result[i])>1.e18f)throw std::runtime_error("unqualified automatic camera distance");result[i]=pairProduct(result[i],result[i]);}
            const auto squared=pairSum(pairSum(result[0],result[1]),pairProduct(1.f,result[2]));
            volatile float root=static_cast<float>(std::sqrt(double(squared)));result[0]=pairSum(0.f,root);
            std::memcpy(record(rt,c->r[6].uw[0],16),result.data(),16);
        }return true;
    }
    const bool callbackPoint=pc>=0x410660 && pc<=0x410684 && !(pc&3);
    const bool callbackSecond=pc>=0x4106b0 && pc<=0x4106cc && !(pc&3);
    const bool callbackThird=pc>=0x4106f8 && pc<=0x410714 && !(pc&3);
    if(callbackPoint || callbackSecond || callbackThird){
        if(!c || c->pc!=0x410640)throw std::runtime_error("native particle callback transform lacks scalar owner");
        const auto stack=c->r[29].uw[0],emitter=c->r[4].uw[0],destination=stack+(callbackPoint?0x70:callbackSecond?0x60:0x50);
        auto *owner=record(rt,emitter,0xe0);record(rt,stack,0x80);
        const unsigned flag=callbackPoint?1:callbackSecond?2:4;
        if(!(owner[0xd2]&flag) || c->r[2].uw[0]!=destination || c->r[3].uw[0]!=emitter+0x80)
            throw std::runtime_error("native particle callback transform argument mismatch");
        if(pc==0x410684 || pc==0x4106cc || pc==0x410714){
            const auto input=c->r[callbackPoint?5:callbackSecond?6:7].uw[0];
            if(callbackPoint){
                struct ArithmeticScope{unsigned saved=_mm_getcsr();~ArithmeticScope(){_mm_setcsr(saved);}} arithmetic;
                math::Matrix basis;std::array<float,4> position;std::memcpy(basis.data(),owner+0x80,64);std::memcpy(position.data(),record(rt,input,16),16);position[3]=1.f;
                float values[16];std::memcpy(values,basis.data(),64);for(float value:values){pairQualified(value);if(std::abs(value)>1.e8f)throw std::runtime_error("unqualified particle callback basis");}
                for(unsigned i=0;i<3;++i){pairQualified(position[i]);if(std::abs(position[i])>1.e8f)throw std::runtime_error("unqualified particle callback position");}
                __m128 point;std::memcpy(&point,position.data(),16);const auto result=math::matrixVectorProduct(basis,point);std::memcpy(record(rt,destination,16),&result,16);
            }else publishLinearVector(rt,emitter+0x80,input,destination);
        }return true;
    }
    if(pc==0x216f54){
        if(!c || c->pc!=0x216f50)throw std::runtime_error("inverse pivot weights require complete scalar owner");
        auto *input=record(rt,c->r[4].uw[0],64),*output=record(rt,c->r[5].uw[0],64),*frame=record(rt,c->r[29].uw[0],0x90);
        const auto apart=[&](uint8_t *p){const auto a=reinterpret_cast<uintptr_t>(p),b=reinterpret_cast<uintptr_t>(frame);
            if(a<b+0x90 && b<a+64)throw std::runtime_error("inverse pivot weights alias their scalar frame");};
        apart(input);apart(output);
        float matrix[4][4],maxima[4]={},weights[4][4];std::memcpy(matrix,input,64);
        for(unsigned row=0;row<4;++row)for(unsigned lane=0;lane<4;++lane){
            pairQualified(matrix[row][lane]);if(std::abs(matrix[row][lane])>1.e8f)throw std::runtime_error("unqualified inverse pivot matrix input");
            matrix[row][lane]=std::abs(matrix[row][lane]);maxima[lane]=std::max(maxima[lane],matrix[row][lane]);
        }
        // Publish only the pivot-selection table the original scalar inverse
        // consumes. Its raw matrix copy, pivot swaps, elimination, solve and
        // output writes remain original game code. No implicit numeric state.
        music::CodecRounding rounding;
        for(unsigned row=0;row<4;++row){
            volatile float inverse=maxima[row]==0?std::numeric_limits<float>::max():1.f/maxima[row];
            const auto scale=pairNormal(inverse);
            for(unsigned lane=0;lane<4;++lane){
                if(double(matrix[row][lane])*scale>std::numeric_limits<float>::max())throw std::runtime_error("inverse pivot weight overflow");
                weights[row][lane]=pairProduct(matrix[row][lane],scale);
            }
        }
        std::memcpy(frame,weights,64);return true;
    }
    switch(pc){
    case 0x216f58:case 0x216f5c:case 0x216f60:case 0x216f64:case 0x216f68:case 0x216f6c:case 0x216f70:case 0x216f74:
    case 0x216f78:case 0x216f80:case 0x216f8c:case 0x216f98:case 0x216fa4:case 0x216fb0:case 0x216fb4:case 0x216fb8:
    case 0x216fbc:case 0x216fc0:case 0x216fd0:case 0x216fd4:case 0x216fd8:case 0x216fdc:case 0x216fe0:case 0x216fe8:
    case 0x216ff8:case 0x216ffc:case 0x217000:case 0x217004:case 0x217008:case 0x217010:case 0x217020:case 0x217024:
    case 0x21702c:case 0x217030:return true;
    default:break;
    }
    if((pc>=0x227d78 && pc<=0x227d9c && !(pc&3)) || (pc>=0x227de8 && pc<=0x227e0c && !(pc&3)) ||
       (pc>=0x227f08 && pc<=0x227f2c && !(pc&3))){
        const bool offset=pc<0x227de0,query=pc>=0x227f08;const auto stack=c->r[29].uw[0];
        const auto point=stack+(query?0x440:0x390),destination=stack+(query?0x2f0:0x440);
        const auto matrix=query?c->r[18].uw[0]:offset?stack+0x450:c->r[19].uw[0];
        if(c->pc<0x2262b0 || c->pc>=0x22b880 || c->r[2].uw[0]!=destination ||
           (offset?(c->r[4].uw[0]!=point || c->r[3].uw[0]!=matrix):c->r[3].uw[0]!=point))
            throw std::runtime_error("native object-update point setup mismatch");
        record(rt,stack,0x490);
        if(pc==0x227d9c || pc==0x227e0c || pc==0x227f2c)publishFullPoint(rt,matrix,point,destination,!query);
        return true;
    }
    if((pc>=0x229164 && pc<=0x229180 && !(pc&3)) || (pc>=0x229228 && pc<=0x229240 && !(pc&3)) || pc==0x229248 ||
       (pc>=0x2292f8 && pc<=0x229314 && !(pc&3)) || (pc>=0x2295d4 && pc<=0x2295f0 && !(pc&3)) ||
       (pc>=0x2295fc && pc<=0x229614 && !(pc&3)) || pc==0x22961c || (pc>=0x229628 && pc<=0x229644 && !(pc&3))){
        const auto stack=c->r[29].uw[0];const bool change=pc>=0x2295d4;
        const bool first=pc<=0x229180 || (pc>=0x2295d4 && pc<=0x2295f0),second=(pc>=0x229228 && pc<=0x229248) || (pc>=0x2295fc && pc<=0x22961c);
        const auto temporary=change?0x420u:0x430u,source=change?0x210u:0x2d0u;
        const auto vector=stack+(second?temporary:source),matrix=c->r[first?19:18].uw[0];
        const auto destination=first?stack+temporary:c->r[16].uw[0]+(change?0x40:0x30);
        if(c->pc<0x2262b0 || c->pc>=0x22b880 ||
           (first?(c->r[3].uw[0]!=vector || c->r[2].uw[0]!=destination):(second || change)?(c->r[2].uw[0]!=vector || c->r[3].uw[0]!=destination):
               (c->r[3].uw[0]!=vector || c->r[4].uw[0]!=destination)))
            throw std::runtime_error("native object-update linear-vector setup mismatch");
        record(rt,stack,0x450);
        if(pc==0x229180 || pc==0x229248 || pc==0x229314 || pc==0x2295f0 || pc==0x22961c || pc==0x229644)
            publishLinearVector(rt,matrix,vector,destination);
        return true;
    }
    if(pc==0x40b040){nativeParticlePhysicsInstruction(c,rt,pc);return false;}
    if(pc>=0x40a950 && pc<=0x40adf8 && nativeParticlePhysicsInstruction(c,rt,pc))return true;
    static constexpr std::array<uint32_t,26> particleSites{0x408f10,0x408f84,0x408f88,0x408fa8,0x408fac,0x408fb8,0x408fbc,
        0x409064,0x409068,0x40908c,0x409090,0x40909c,0x4090a0,0x409118,0x4091c0,0x4091c8,0x40931c,0x409320,0x40932c,0x409334,
        0x409400,0x409404,0x40956c,0x409570,0x40957c,0x409584};
    if(pc>=particleSites.front() && pc<=particleSites.back() && std::binary_search(particleSites.begin(),particleSites.end(),pc)){
        const auto object=c->r[4].uw[0],stack=c->r[29].uw[0];auto *owner=record(rt,object,0xf0);record(rt,stack,0x80);
        const auto readWord=[](const uint8_t *p,unsigned at){uint32_t v;std::memcpy(&v,p+at,4);return v;};
        const auto group=readWord(owner,0x10),pool=readWord(owner,0x14),count=readWord(owner,0xc);
        if(!group || group>16 || group!=c->r[25].uw[0] || !count || count>65536 ||
           (c->pc!=0x408e90 && c->pc!=0x408f30 && c->pc!=0x408fe8 && c->pc!=0x409148 && c->pc!=0x409360 && c->pc!=0x4091d0 && c->pc!=0x409418))
            throw std::runtime_error("native particle motion owner/continuation mismatch");
        auto *timeRecord=record(rt,0x35dee0,16);float time;std::memcpy(&time,timeRecord+8,4);
        if(pc==0x408f10 || pc==0x409118){
            const bool simple=owner[0xd8]&1;
            if(simple!=(pc==0x408f10) || c->r[pc==0x408f10?3:16].uw[0]!=std::bit_cast<uint32_t>(time))
                throw std::runtime_error("native particle timestep source mismatch");
            return true;
        }
        const unsigned path=pc<=0x408fbc?0:pc<=0x4090a0?1:pc<=0x409334?2:3;
        const bool simple=owner[0xd8]&1;
        if(simple!=(path<2) || (group==1)!=(path%2==0))throw std::runtime_error("native particle motion branch mismatch");
        const bool loading=pc==0x408f84 || pc==0x408f88 || pc==0x409064 || pc==0x409068 || pc==0x4091c0 || pc==0x4091c8 || pc==0x409400 || pc==0x409404;
        const auto cursor=c->r[24].uw[0];if((!loading && !cursor) || (loading?cursor:cursor-1)>=count)throw std::runtime_error("native particle source index outside owner");
        const auto index=loading?cursor:cursor-1,source=pool+index*0x90+0x60;
        record(rt,pool,count*0x90);auto *input=record(rt,source,32);
        constexpr unsigned sourceRegisters[]={10,3,22,21};
        if(c->r[sourceRegisters[path]].uw[0]!=source)throw std::runtime_error("native particle position source mismatch");
        const bool publish=pc==0x408fb8 || pc==0x40909c || pc==0x40932c || pc==0x40957c;
        const bool feedback=pc==0x408fbc || pc==0x4090a0 || pc==0x409334 || pc==0x409584;
        if(publish || feedback){
            const auto output=c->r[6].uw[0]+0x20,destination=publish?output:source;
            constexpr unsigned outputRegisters[]={3,9,16,16};
            if(publish && c->r[outputRegisters[path]].uw[0]!=output)throw std::runtime_error("native particle output pointer mismatch");
            auto *out=record(rt,output,16),*frame=record(rt,stack,0x80);
            const auto a=reinterpret_cast<uintptr_t>(input),b=reinterpret_cast<uintptr_t>(out),f=reinterpret_cast<uintptr_t>(frame),o=reinterpret_cast<uintptr_t>(owner),t=reinterpret_cast<uintptr_t>(timeRecord);
            if((a<b+16 && b<a+32) || (b<f+0x80 && f<b+16) || (b<o+0xf0 && o<b+16) || (b<t+16 && t<b+16) || (a<t+16 && t<a+32))
                throw std::runtime_error("native particle position aliases its live operands/frame");
            std::array<float,4> position,velocity;std::memcpy(position.data(),input,16);std::memcpy(velocity.data(),input+16,16);
            pairQualified(time);if(std::abs(time)>1.e8f)throw std::runtime_error("unqualified native particle timestep");
            for(unsigned i=0;i<3;++i){pairQualified(position[i]);pairQualified(velocity[i]);if(std::abs(position[i])>1.e8f || std::abs(velocity[i])>1.e8f)
                throw std::runtime_error("unqualified native particle position/velocity");}
            struct ArithmeticScope {unsigned saved=_mm_getcsr();~ArithmeticScope(){_mm_setcsr(saved);}} arithmetic;
            for(unsigned i=0;i<3;++i)position[i]=pairSum(pairSum(position[i],0.f),pairProduct(velocity[i],time));
            std::memcpy(record(rt,destination,16),position.data(),16);
        }return true;
    }
    if(pc==0x214c40 || pc==0x214ce8 ||
       (pc>=0x214c98 && pc<=0x214ca0 && !(pc&3)) || (pc>=0x214cbc && pc<=0x214cc4 && !(pc&3)) ||
       (pc>=0x214cdc && pc<=0x214ce4 && !(pc&3)) ||
       (pc>=0x214db8 && pc<=0x214dd0 && !(pc&3)) || pc==0x214dd8 ||
       (pc>=0x214df0 && pc<=0x214e00 && !(pc&3)) || (pc>=0x214e08 && pc<=0x214e1c && !(pc&3)))
        return nativeSquadInstruction(c,rt,pc);
    if((pc>=0x2a68d4 && pc<=0x2a68f4 && !(pc&3)) || pc==0x2a68fc){
        const auto stack=c->r[29].uw[0];
        if(c->pc!=0x2a68c8 || c->r[4].uw[0]!=0x35dfd0)throw std::runtime_error("native Hermite vector setup mismatch");
        record(rt,stack,0x40);
        if(pc==0x2a68fc){
            if(c->r[3].uw[0]!=stack+0x30)throw std::runtime_error("native Hermite vector destination mismatch");
            math::Matrix controls;__m128 weights;std::memcpy(controls.data(),record(rt,c->r[16].uw[0],0x50),64);
            std::memcpy(&weights,record(rt,0x35dfd0,16),16);float inputs[20];std::memcpy(inputs,controls.data(),64);std::memcpy(inputs+16,&weights,16);
            for(float value:inputs){pairQualified(value);if(std::abs(value)>1.e8f)throw std::runtime_error("unqualified native Hermite vector input");}
            struct ArithmeticScope {unsigned saved=_mm_getcsr();~ArithmeticScope(){_mm_setcsr(saved);}} arithmetic;
            const auto result=math::matrixVectorProduct(controls,weights);std::memcpy(record(rt,stack+0x30,16),&result,16);
        }return true;
    }
    if((pc>=0x2d7c04 && pc<=0x2d7c70 && !(pc&3)) ||
       (pc>=0x2d7c80 && pc<=0x2d7cec && !(pc&3)) ||
       (pc>=0x2d7cfc && pc<=0x2d7d68 && !(pc&3))){
        const auto stack=c->r[29].uw[0],local=stack+(pc<=0x2d7c70?0x140:pc<=0x2d7cec?0x180:0xc0);
        if(c->pc!=0x2d7be0 || c->r[4].uw[0]!=local || c->r[3].uw[0]!=stack+0x100 || c->r[5].uw[0]!=stack+0x100)
            throw std::runtime_error("native DOF matrix setup mismatch");
        record(rt,stack,0x240);
        if(pc==0x2d7c64 || pc==0x2d7ce0 || pc==0x2d7d5c){
            math::Matrix first,parent;std::memcpy(first.data(),record(rt,local,64),64);std::memcpy(parent.data(),record(rt,stack+0x100,64),64);
            for(const auto *matrix:{&first,&parent}){float values[16];std::memcpy(values,matrix->data(),64);
                for(float value:values){pairQualified(value);if(std::abs(value)>1.e8f)throw std::runtime_error("unqualified native DOF matrix input");}}
            struct ArithmeticScope {unsigned saved=_mm_getcsr();~ArithmeticScope(){_mm_setcsr(saved);}} arithmetic;
            const auto result=math::projectionProduct(first,parent);std::memcpy(record(rt,stack+0x100,64),result.data(),64);
        }return true;
    }
    if(pc==0x215a8c || pc==0x215a94 || pc==0x215a98 || pc==0x215aa0 ||
       pc==0x215af0 || pc==0x215af8 || pc==0x215afc || pc==0x215b04 ||
       pc==0x215b54 || pc==0x215b5c || pc==0x215b60 || pc==0x215b68 ||
       pc==0x215dd8 || pc==0x215de0 || pc==0x215de4 || pc==0x215dec){
        if(c->pc!=0x2159c0 && c->pc!=0x215a58 && c->pc!=0x215abc && c->pc!=0x215b20)
            throw std::runtime_error("unqualified positive-axis Euler continuation");
        const auto stack=c->r[29].uw[0];record(rt,stack,0x180);
        if(pc==0x215a98 || pc==0x215afc || pc==0x215b60 || pc==0x215de4){c->r[9].ud[0]=0;return true;}
        if(pc==0x215aa0 || pc==0x215b04 || pc==0x215b68 || pc==0x215dec){
            const auto at=stack+(pc==0x215aa0?0x140:pc==0x215b04?0x130:pc==0x215b68?0x120:0x110);
            if(c->r[pc==0x215b68 || pc==0x215dec?3:2].uw[0]!=at || (pc==0x215dec && c->r[4].uw[0]!=at))
                throw std::runtime_error("positive-axis Euler temporary mismatch");
            publishUnitQuaternion(rt,at);
        }return true;
    }
    if(pc>=0x41278c && pc<=0x4127ac && !(pc&3)){
        const auto stack=c->r[29].uw[0],object=c->r[16].uw[0];
        if(c->pc!=0x4126a0 || c->r[3].uw[0]!=object+0x220 || c->r[2].uw[0]!=stack+0x20)
            throw std::runtime_error("native action magnitude setup mismatch");
        auto *frame=record(rt,stack,0x30),*owner=record(rt,object,0x7d0);
        const auto f=reinterpret_cast<uintptr_t>(frame),o=reinterpret_cast<uintptr_t>(owner);
        if(f<o+0x7d0 && o<f+0x30)throw std::runtime_error("native action magnitude frame overlaps owner");
        if(pc==0x4127ac)publishMagnitude(rt,object+0x220,stack+0x20);
        return true;
    }
    if(pc>=0x446724 && pc<=0x446744 && !(pc&3)){
        const auto stack=c->r[29].uw[0],object=c->r[18].uw[0];
        if((c->pc!=0x44665c && c->pc!=0x4466c0) || c->r[4].uw[0]!=stack+0xb0 || c->r[3].uw[0]!=stack+0x50)
            throw std::runtime_error("native animation movement magnitude setup mismatch");
        const auto f=reinterpret_cast<uintptr_t>(record(rt,stack,0xe0)),o=reinterpret_cast<uintptr_t>(record(rt,object,0x130));
        if(f<o+0x130 && o<f+0xe0)throw std::runtime_error("native animation movement frame aliases owner");
        if(pc==0x446744)publishMagnitude(rt,stack+0xb0,stack+0x50);
        return true;
    }
    if(pc==0x216390 || pc==0x216398 || pc==0x21639c || pc==0x2163a4 ||
       pc==0x2163f4 || pc==0x2163fc || pc==0x216400 || pc==0x216408 ||
       pc==0x216458 || pc==0x216460 || pc==0x216464 || pc==0x21646c ||
       pc==0x2166d8 || pc==0x2166e0 || pc==0x2166e4 || pc==0x2166e8){
        if(c->pc!=0x2162c0 && c->pc!=0x21635c && c->pc!=0x2163c0 && c->pc!=0x216424)
            throw std::runtime_error("unqualified native Euler quaternion continuation");
        record(rt,c->r[29].uw[0],0x170);
        if(pc==0x21639c || pc==0x216400 || pc==0x216464 || pc==0x2166e4){c->r[9].ud[0]=0;return true;}
        if(pc==0x2163a4 || pc==0x216408 || pc==0x21646c || pc==0x2166e8){
            const auto stack=c->r[29].uw[0],at=pc==0x2163a4?stack+0x130:pc==0x216408?stack+0x120:pc==0x21646c?stack+0x110:c->r[16].uw[0];
            if(pc!=0x2166e8 && c->r[pc==0x21646c?3:2].uw[0]!=at)throw std::runtime_error("native Euler quaternion temporary mismatch");
            publishUnitQuaternion(rt,at);
        }return true;
    }
    if((pc>=0x1f6f94 && pc<=0x1f6fa4 && !(pc&3)) ||
       (pc>=0x1f7194 && pc<=0x1f71a4 && !(pc&3)) ||
       (pc>=0x1f72cc && pc<=0x1f72dc && !(pc&3)) ||
       (pc>=0x1f7334 && pc<=0x1f7344 && !(pc&3))){
        // This measured root has no leaves/children. These stores are consumed
        // only by the matching old intermediate restores, never scalar code.
        leaflessQuery(rt,c,pc==0x1f6f94);return true;
    }
    if(pc==0x1f7098 || pc==0x1f709c || pc==0x1f70a0 || pc==0x1f70a8 || pc==0x1f70ac || pc==0x1f70b4){
        const auto data=leaflessQuery(rt,c);
        if(pc==0x1f70b4){if(c->r[2].uw[0]!=c->r[29].uw[0]+0x40)throw std::runtime_error("native query scale destination mismatch");
            const auto scales=queryBasisScales(data.object+0x90);std::memcpy(data.frame+0x40,scales.data(),12);}return true;
    }
    if(pc>=0x1f70f0 && pc<=0x1f717c && !(pc&3)){
        const auto data=leaflessQuery(rt,c);
        if(pc==0x1f717c){if(c->r[2].uw[0]!=c->r[29].uw[0]+0x50)throw std::runtime_error("native query distance destination mismatch");
            const auto result=leaflessDistance(data);std::memcpy(data.frame+0x50,result.data(),16);}return true;
    }
    if(pc>=0x252ec8 && pc<=0x252f20 && !(pc&3)){
        if(pc==0x252f20){const auto destination=c->r[29].uw[0]+0xd0;
            if(c->r[2].uw[0]!=destination || c->r[3].uw[0]!=c->r[16].uw[0]+0x210)
                throw std::runtime_error("native height-query transform setup mismatch");
            const auto result=rowPoint(rt,c->r[3].uw[0],c->r[17].uw[0]);std::memcpy(record(rt,destination,16),result.data(),16);}
        return true;
    }
    if(pc==0x252fdc || pc==0x252fe0 || pc==0x252fe4 || pc==0x252fec || pc==0x252ff0 || pc==0x252ff8){
        if(pc==0x252ff8){const auto object=c->r[16].uw[0],stack=c->r[29].uw[0];
            if(c->r[2].uw[0]!=stack+0xb0)throw std::runtime_error("native height-query scale destination mismatch");
            auto *owner=record(rt,object,0x260),*temporary=record(rt,stack+0xb0,16);
            uint32_t descriptor;std::memcpy(&descriptor,owner+0x250,4);auto *header=record(rt,descriptor,32);uint32_t width,height;
            std::memcpy(&width,header+8,4);std::memcpy(&height,header+12,4);
            const auto bytes=32ull+uint64_t(width)*height*2;
            if(!width || !height || width>2048 || height>2048 || bytes>0x2000000)throw std::runtime_error("unqualified native height-query grid");
            // The scalar interpolation consumes scale.Y only; prevent the
            // retired temporary lanes from aliasing any live input/grid record.
            separatePairFrame(temporary,owner,0x260);separatePairFrame(temporary,record(rt,descriptor,uint32_t(bytes)),size_t(bytes));
            separatePairFrame(temporary,record(rt,c->r[17].uw[0],16),16);
            const auto scale=basisYScale(owner+0x90);std::memcpy(temporary+4,&scale,4);}
        return true;
    }
    if(((pc>=0x3e7edc && pc<=0x3e7f34) || (pc>=0x3e7fa0 && pc<=0x3e7ff8)) && !(pc&3)){
        if(pc==0x3e7f34 || pc==0x3e7ff8){
            const auto destination=c->r[29].uw[0]+0x70;
            if(c->r[3].uw[0]!=destination)throw std::runtime_error("native corner transform destination mismatch");
            const auto result=rowPoint(rt,c->r[2].uw[0],destination);
            std::memcpy(record(rt,destination,16),result.data(),16);
        }
        return true;
    }
    if(pc==0x3e5ca4 || pc==0x3e5ca8 || pc==0x3e5cac || pc==0x3e5cb4 || pc==0x3e5cb8 || pc==0x3e5cc0){
        const auto object=c->r[18].uw[0],stack=c->r[29].uw[0],output=c->r[19].uw[0];
        if(c->r[4].uw[0]!=object+0x90 || (pc==0x3e5cc0 && c->r[3].uw[0]!=stack+0x50))
            throw std::runtime_error("native height-bound argument setup mismatch");
        auto *basis=record(rt,object+0x90,48),*temporary=record(rt,stack+0x50,16);
        auto *destination=rt->memory.hostPointer(output,8);
        if((output&3) || output<0x100000 || !destination)throw std::runtime_error("native height-bound output outside owned memory");
        separatePairFrame(temporary,record(rt,object,0x254),0x254);separatePairFrame(temporary,destination,8);
        const auto scale=basisYScale(basis);
        // Only scale.Y reaches the scalar min/max consumers. Other temporary
        // lanes and the unused control read retire with the record separated
        // from the complete object/output; no inherited vector state is needed.
        if(pc==0x3e5cc0)std::memcpy(temporary+4,&scale,4);
        return true;
    }
    if(nativeTrigInstruction(nullptr,c,rt,pc))return true;
    if(nativeObjectMathInstruction(nullptr,c,rt,pc))return true;
    if(nativeFrustumInstruction(c,rt,pc))return true;
    if(pc==0x1fbff0 || pc==0x1fc010){
        auto contact=planeContact(rt,c);
        if(pc==0x1fbff0)std::memcpy(record(rt,c->r[29].uw[0],16),&contact.distance,4);
        else{
            if(contact.distance>0 || std::bit_cast<uint32_t>(c->f[1])!=std::bit_cast<uint32_t>(contact.distance))throw std::runtime_error("plane-contact projection branch mismatch");
            for(unsigned i=0;i<3;++i)contact.point[i]=pairSum(pairSum(contact.point[i],0.f),-pairProduct(contact.normal[i],contact.distance));
            std::memcpy(record(rt,c->r[6].uw[0],16),contact.point.data(),16);
        }return true;
    }
    if(pc==0x1fbfbc || pc==0x1fbfc0 || pc==0x1fbfc4 || pc==0x1fbfd0 || pc==0x1fbfd8 || pc==0x1fbfdc || pc==0x1fbfe0 || pc==0x1fbfe4 || pc==0x1fbfe8 || pc==0x1fbfec || pc==0x1fc008 || pc==0x1fc00c)return true;
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
        music::NearestRounding rounding;
        for(unsigned row=0;row<4;++row){float products[4];
            for(unsigned i=0;i<4;++i){volatile float product=matrix[row][i]*vector[i];products[i]=normal(product);}
            auto total=pairSum(products[order[row][0]],products[order[row][1]]);
            total=pairSum(total,products[order[row][2]]);total=pairSum(total,products[order[row][3]]);
            if(!std::isfinite(total))throw std::runtime_error("native row dot overflow");result[row]=total;
        }
        std::memcpy(record(rt,c->r[6].uw[0],16),result,16);return true;
    }
    if(pc==0x29e1a0 || pc==0x29e160){
        // The default math code installation has no execution or game-data
        // result. Its algorithms are compiled native record operations. Retire
        // the physical enqueue before it exists; never mark a job busy/complete.
        // Mode 1 (selected around the mod001 2188d0 update) installs a second
        // fingerprinted VU0 program the same way: one REFE of code only, no
        // UNPACK or MSCAL. Its routines run only from later VIF0 jobs, which
        // stay unported boundaries; the game's own mode byte (+51) records it.
        const uint32_t object=c->r[4].uw[0];auto *p=record(rt,object,0x54);
        uint32_t type=0;std::memcpy(&type,p,4);const auto mode=c->r[5].uw[0]&255;
        if(type!=0x1591d0 || (mode!=0 && (mode!=1 || pc!=0x29e1a0)) || p[0x50]!=0)
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

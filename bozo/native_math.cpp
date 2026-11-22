#include "native_math.h"
#include "native_object_math.h"
#include "native_trig.h"
#include "native_particles.h"
#include "native_frustum.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/ops_fpu.h"
#include "bozo/hash.h"
#include "bozo/state.h"
#include "../port/vp2s_math.h"
#include <cstring>
#include <map>
#include <algorithm>
#include <cmath>
#include <format>
#include <stdexcept>
namespace vp2s {
namespace {
std::map<std::pair<const Runtime *,uint32_t>,bool> &modelLightVariants(){static std::map<std::pair<const Runtime *,uint32_t>,bool> variants;return variants;}
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
// A VU0 word with the engine's PS2 float definitions (ps2f): nearest-even
// results, no Inf/NaN (exponent 255 is a number, overflow saturates to
// +/-0x7fffffff), denormals read as zero, division by zero gives +/-max.
// For normal finite operands every operation equals the host's correctly
// rounded float result, so converted sites keep their in-range values bit for
// bit while far-away chains (anchors parked ~1e5 away, nodes running to 1e36)
// stay defined as on PS2. No host floating-point state is touched.
struct Ps2Word{
    uint32_t bits=0;
    static Ps2Word of(float v){return {std::bit_cast<uint32_t>(v)};}
    static Ps2Word load(const uint8_t *p){Ps2Word w;std::memcpy(&w.bits,p,4);return w;}
    void store(uint8_t *p)const{std::memcpy(p,&bits,4);}
    bool zero()const{return !(bits&0x7f800000u);}
    int64_t key()const{if(zero())return 0;const int64_t m=bits&0x7fffffffu;return (bits&0x80000000u)?-m:m;}
    Ps2Word operator-()const{return {bits^0x80000000u};}
    Ps2Word abs()const{return {bits&0x7fffffffu};}
    friend Ps2Word operator*(Ps2Word a,Ps2Word b){uint32_t f=0;return {bozo::ps2f::mul(a.bits,b.bits,f)};}
    friend Ps2Word operator+(Ps2Word a,Ps2Word b){uint32_t f=0;return {bozo::ps2f::addNearest(a.bits,b.bits,f)};}
    friend Ps2Word operator-(Ps2Word a,Ps2Word b){uint32_t f=0;return {bozo::ps2f::subNearest(a.bits,b.bits,f)};}
    friend Ps2Word operator/(Ps2Word a,Ps2Word b){uint32_t f=0;return {bozo::ps2f::div(a.bits,b.bits,f)};}
    friend bool operator<(Ps2Word a,Ps2Word b){return a.key()<b.key();}
    friend bool operator>(Ps2Word a,Ps2Word b){return a.key()>b.key();}
    friend bool operator<=(Ps2Word a,Ps2Word b){return a.key()<=b.key();}
    friend bool operator>=(Ps2Word a,Ps2Word b){return a.key()>=b.key();}
    // ps2f's integer square root seeds itself with a host double sqrt; keep
    // the host's floating-point status exactly as it was.
    Ps2Word root()const{const auto saved=_mm_getcsr();uint32_t f=0;const Ps2Word r{bozo::ps2f::sqrt(bits,f)};_mm_setcsr(saved);return r;}
    static Ps2Word reciprocalRoot(Ps2Word a,Ps2Word b){const auto saved=_mm_getcsr();uint32_t f=0;const Ps2Word r{bozo::ps2f::rsqrt(a.bits,b.bits,f)};_mm_setcsr(saved);return r;}
};
constexpr Ps2Word kPs2One{0x3f800000u},kPs2Zero{0};
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
    const auto bad=[&](const char *what,const float *values,unsigned count,uint32_t at){std::string list;
        for(unsigned i=0;i<count;++i)list+=std::format(" {:08x}",std::bit_cast<uint32_t>(values[i]));
        throw std::runtime_error(std::format("unqualified native row-point {} at {:x}:{}",what,at,list));};
    for(auto &row:matrix)for(float value:row){const auto b=std::bit_cast<uint32_t>(value),e=b&0x7f800000;
        if(e==0x7f800000 || (!e && (b&0x7fffff)) || std::abs(value)>1.e8f)bad("matrix",&matrix[0][0],16,matrixAt);}
    // The point reads as VU0 reads it: a denormal is zero (ps2f). Battle set-up (mod028
    // 4233c4) queries the height at a stack record it never wrote; its saved-register
    // words (0x43c4fc, 0x90) are denormals, so the point is (0, 0, 1, 0) as on PS2. That
    // call's result is unused, and the next query clears its outside-the-map flag first.
    for(float &value:point){const auto b=std::bit_cast<uint32_t>(value),e=b&0x7f800000;
        if(e==0x7f800000 || (b&0x7fffffff)>std::bit_cast<uint32_t>(1.e8f))bad("input",point,4,pointAt); // integer test: no host denormal flag
        if(!e)value=std::bit_cast<float>(b&0x80000000);}
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
struct PositionPairGeometry {std::array<Ps2Word,3> difference;Ps2Word distance;};
// PS2 words (see Ps2Word): every position is a VU number, no magnitude gate.
PositionPairGeometry positionPairGeometry(const uint8_t *first,const uint8_t *second){
    PositionPairGeometry out;Ps2Word squares[3];
    for(unsigned i=0;i<3;++i){out.difference[i]=Ps2Word::load(second+4*i)-Ps2Word::load(first+4*i);squares[i]=out.difference[i]*out.difference[i];}
    const auto squared=(squares[0]+squares[1])+kPs2One*squares[2];
    out.distance=kPs2Zero+squared.root();return out;
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
        // Admitted: recursion; the 1f7370 wrapper from 3e70bc/3e753c (they read the
        // hit object's VF1..VF4 afterwards, see the hit-box/hit-point ports); and
        // mod003 41f340's object-list loop (direct call, 41f3f0), which keeps only
        // the hit flag and reads no VU register afterwards.
        if(c->r[31].uw[0]!=0x1f72f4 && c->r[31].uw[0]!=0x41f3f0 && (c->r[31].uw[0]!=0x1f7390 || (caller!=0x3e70bc && caller!=0x3e753c)))
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
struct SegmentTerms {std::array<uint32_t,3> delta;uint32_t firstLength,negativeCross,secondLength,firstProjection,separation,determinant,negativeSecondProjection,secondNumerator,firstNumerator;};
struct PointSegmentTerms {std::array<Ps2Word,3> delta,direction;Ps2Word projection,lengthSquared;};
struct PlaneContact {std::array<uint8_t,16> point;std::array<Ps2Word,3> normal;Ps2Word distance;};
PlaneContact planeContact(Runtime *rt,R5900Context *c){
    if(c->pc!=0x1fbfb0 || c->r[31].uw[0]!=0x1fe210)throw std::runtime_error("unqualified plane-contact caller");
    const auto stack=c->r[29].uw[0],owner=c->r[4].uw[0],pointAddress=c->r[17].uw[0];
    auto *frame=record(rt,stack,16),*plane=record(rt,owner,0x240),*point=record(rt,pointAddress,0x40),*output=record(rt,c->r[6].uw[0],16);
    uint32_t attached;std::memcpy(&attached,point+8,4);
    if(attached!=owner || c->r[5].uw[0]!=pointAddress+0x30 || c->r[6].uw[0]!=stack+0x1f0 || c->r[3].uw[0]!=owner+0x230 || c->r[2].uw[0]!=stack)
        throw std::runtime_error("plane-contact argument or temporary setup mismatch");
    separatePairFrame(frame,plane,0x240);separatePairFrame(frame,point,0x40);separatePairFrame(frame,output,16);
    uint32_t marginBits;std::memcpy(&marginBits,point+0x18,4);
    if(marginBits!=std::bit_cast<uint32_t>(c->f[12]))throw std::runtime_error("plane-contact margin argument mismatch");
    // PS2 words (see Ps2Word), 1fbfd8..1fbfec: d = ((p-o)*n summed (x+y)+1*z) - margin.
    const auto margin=Ps2Word::of(c->f[12]);PlaneContact result;std::memcpy(result.point.data(),point+0x30,16);Ps2Word products[3];
    for(unsigned i=0;i<3;++i){result.normal[i]=Ps2Word::load(plane+0x220+4*i);products[i]=(Ps2Word::load(point+0x30+4*i)-Ps2Word::load(plane+0x230+4*i))*result.normal[i];}
    result.distance=((products[0]+products[1])+kPs2One*products[2])-margin;return result;
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
    // PS2 words (see Ps2Word): every input is a VU number, so no magnitude gate.
    Ps2Word s[2][3],p[3];for(unsigned i=0;i<3;++i){s[0][i]=Ps2Word::load(segment+4*i);s[1][i]=Ps2Word::load(segment+16+4*i);p[i]=Ps2Word::load(point+4*i);}
    PointSegmentTerms result;Ps2Word products[3],squares[3];
    for(unsigned i=0;i<3;++i){result.delta[i]=p[i]-s[0][i];result.direction[i]=s[1][i];products[i]=result.delta[i]*s[1][i];squares[i]=s[1][i]*s[1][i];}
    result.projection=(products[0]+products[1])+kPs2One*products[2];
    result.lengthSquared=(squares[0]+squares[1])+kPs2One*squares[2];return result;
}
std::array<Ps2Word,3> collisionNormal(Runtime *rt,R5900Context *c,uint32_t pc){
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
    // PS2 words (see Ps2Word): ACC = (0+B) + D2*s - A*1, normal = (ACC - D1*t) * RSQRT(1, |.|^2).
    // The segment parameters come clamped from the EE code; anything else is not this caller.
    const auto firstWeight=Ps2Word::of(c->f[2]),secondWeight=Ps2Word::of(c->f[1]);const Ps2Word two{0x40000000u};
    if(firstWeight.abs()>two || secondWeight.abs()>two)throw std::runtime_error("unqualified collision-normal parameters");
    std::array<Ps2Word,3> delta,result;Ps2Word squared[3];
    for(unsigned i=0;i<3;++i){const auto value=((Ps2Word::load(second+4*i)+kPs2Zero)+Ps2Word::load(second+16+4*i)*secondWeight)-Ps2Word::load(first+4*i)*kPs2One;
        delta[i]=value-Ps2Word::load(first+16+4*i)*firstWeight;squared[i]=delta[i]*delta[i];}
    const auto q=Ps2Word::reciprocalRoot(kPs2One,(squared[0]+squared[1])+kPs2One*squared[2]);
    for(unsigned i=0;i<3;++i)result[i]=delta[i]*q;return result;
}
SegmentTerms segmentTerms(Runtime *rt,R5900Context *c){
    if(c->pc!=0x2120a0 || (c->r[31].uw[0]!=0x1fc3dc && c->r[31].uw[0]!=0x1fc5f4))
        throw std::runtime_error("unqualified segment-distance caller");
    auto *frame=record(rt,c->r[29].uw[0],16),*first=record(rt,c->r[4].uw[0],32),*second=record(rt,c->r[5].uw[0],32);
    separatePairFrame(frame,first,32);separatePairFrame(frame,second,32);
    for(unsigned reg:{6u,7u}){const auto address=c->r[reg].uw[0];auto *output=rt->memory.hostPointer(address,4);
        if((address&3) || !output)throw std::runtime_error("invalid segment-distance parameter output");separatePairFrame(frame,output,4);}
    // VU0 words with the engine's PS2 float definitions (nearest even, no
    // Inf/NaN, saturation, denormals read as zero): identical to IEEE nearest
    // for normal finite operands, and defined for every word, so the chain
    // stretched by a far-parked anchor (positions ~1e36) saturates as on PS2.
    uint32_t a[2][4],b[2][4];std::memcpy(a,first,32);std::memcpy(b,second,32);
    const auto mul=[](uint32_t x,uint32_t y){uint32_t f=0;return bozo::ps2f::mul(x,y,f);};
    const auto add=[](uint32_t x,uint32_t y){uint32_t f=0;return bozo::ps2f::addNearest(x,y,f);};
    const auto sub=[](uint32_t x,uint32_t y){uint32_t f=0;return bozo::ps2f::subNearest(x,y,f);};
    constexpr uint32_t one=0x3f800000u,minusOne=0xbf800000u;
    SegmentTerms result;std::array<uint32_t,3> u,v,negativeV;
    for(unsigned i=0;i<3;++i){result.delta[i]=sub(a[0][i],b[0][i]);u[i]=a[1][i];v[i]=b[1][i];negativeV[i]=mul(v[i],minusOne);}
    const auto dot=[&](const auto &x,const auto &y){return add(add(mul(x[0],y[0]),mul(x[1],y[1])),mul(one,mul(x[2],y[2])));};
    result.firstLength=dot(u,u);result.negativeCross=mul(dot(u,v),minusOne);result.secondLength=dot(v,v);
    result.firstProjection=dot(result.delta,u);result.separation=dot(result.delta,result.delta);
    result.determinant=sub(mul(result.firstLength,result.secondLength),mul(result.negativeCross,result.negativeCross))&0x7fffffffu;
    result.negativeSecondProjection=dot(result.delta,negativeV);
    result.secondNumerator=sub(mul(result.negativeCross,result.firstProjection),mul(result.firstLength,result.negativeSecondProjection));
    result.firstNumerator=sub(mul(result.negativeCross,result.negativeSecondProjection),mul(result.secondLength,result.firstProjection));return result;
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
    // PS2 words (see Ps2Word), 1fc038..1fc174: scaled = n * (depth * w-coefficient);
    // paired: (A+0) + scaled*(1-t), (B+0) + scaled*t; single: P*1 + scaled*k with
    // k = the +8 coefficient when capped, else DIV(1, t). t comes clamped from the
    // segment solve; anything else is not this caller.
    const auto parameter=Ps2Word::of(c->f[12]),depth=Ps2Word::of(c->f[13]),scale=Ps2Word::load(coefficients+12);
    if(parameter.abs()>Ps2Word{0x40000000u})throw std::runtime_error("unqualified collision response coefficients");
    const auto scaledDepth=depth*scale;Ps2Word correction[3];
    for(unsigned i=0;i<3;++i)correction[i]=Ps2Word::load(normal+4*i)*scaledDepth;
    Ps2Word weights[2];if(paired){weights[0]=kPs2One-parameter;weights[1]=parameter;}
    else{const bool capped=secondOnly?parameter<Ps2Word{0x3c23d70au}:parameter>Ps2Word{0x3f7d70a4u};
        weights[0]=capped?Ps2Word::load(coefficients+8):kPs2One/parameter;}
    std::array<std::array<uint8_t,16>,2> results;const unsigned count=paired?2:1;
    for(unsigned which=0;which<count;++which){const auto *input=paired?(which?second:first):secondOnly?second:first;std::memcpy(results[which].data(),input,16);
        for(unsigned i=0;i<3;++i){const auto value=Ps2Word::load(input+4*i);const auto base=paired?value+kPs2Zero:value*kPs2One;
            (base+correction[i]*weights[which]).store(results[which].data()+4*i);}
    }
    // Pair results are snapshotted before either store; only the already
    // loaded position's own W survives either source path.
    if(paired){std::memcpy(first,results[0].data(),16);std::memcpy(second,results[1].data(),16);}
    else std::memcpy(destination,results[0].data(),16);
}
struct AngularDirections {std::array<Ps2Word,3> first,second;Ps2Word cosine;};
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
    // PS2 words (see Ps2Word), transcribed from 1fcd08..1fcda4: the frame
    // matrix at SP+150 transforms SP+190 and the point record +30 (MULA/MADDA
    // x,y,z then MADDw with 1); each length is RSQRT(1, (x*x+y*y)+1*(z*z));
    // the cosine is (a.x*b.x + a.y*b.y) + a.z*b.z through MR32 rotations.
    Ps2Word rows[4][3],points[2][3];
    for(unsigned r=0;r<4;++r)for(unsigned i=0;i<3;++i)rows[r][i]=Ps2Word::load(frame+0x150+16*r+4*i);
    for(unsigned i=0;i<3;++i){points[0][i]=Ps2Word::load(frame+0x190+4*i);points[1][i]=Ps2Word::load(point+0x30+4*i);}
    AngularDirections result;
    for(unsigned which=0;which<2;++which){Ps2Word value[3],squares[3];
        for(unsigned l=0;l<3;++l){value[l]=((rows[0][l]*points[which][0]+rows[1][l]*points[which][1])+rows[2][l]*points[which][2])+rows[3][l]*kPs2One;squares[l]=value[l]*value[l];}
        const auto q=Ps2Word::reciprocalRoot(kPs2One,(squares[0]+squares[1])+kPs2One*squares[2]);
        auto &direction=which?result.second:result.first;for(unsigned i=0;i<3;++i)direction[i]=value[i]*q;}
    result.cosine=(result.first[0]*result.second[0]+result.first[1]*result.second[1])+result.first[2]*result.second[2];
    return result;
}
// PS2 words (see Ps2Word): xyz = (b*a.w + a*b.w) + (a.yzx*b.zxy - b.yzx*a.zxy)*1,
// w = ((a.w*b.w - a.z*b.z) - a.y*b.y) - a.x*b.x, in the listing's order.
std::array<Ps2Word,4> composeConstraintQuaternions(const std::array<Ps2Word,4> &a,const std::array<Ps2Word,4> &b){
    Ps2Word products[4];for(unsigned i=0;i<4;++i)products[i]=a[i]*b[i];std::array<Ps2Word,4> result;
    for(unsigned i=0;i<3;++i){const auto j=(i+1)%3,k=(i+2)%3;
        const auto cross=a[j]*b[k]-b[j]*a[k];
        const auto base=b[i]*a[3]+a[i]*b[3];result[i]=base+cross*kPs2One;}
    result[3]=((products[3]-products[2])-products[1])-products[0];return result;
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
// The battle's copies in mod001 (only module-local jal targets differ): the physics step
// 291910 = 40a8b0 - 178fa0 calls its own callback 299140 = 410640 - 177500.
constexpr uint32_t kPhysicsCopyShift=0x178fa0,kCallbackCopyShift=0x177500;
bool nativeParticleCallbackInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    constexpr uint32_t sites[]={0x41074c,0x410750,0x410790,0x410794,0x4107a8,0x4107b0,0x4107b4};
    const bool copy=pc>=sites[0]-kCallbackCopyShift && pc<=sites[6]-kCallbackCopyShift;
    const uint32_t shift=copy?kCallbackCopyShift:0,physicsShift=copy?kPhysicsCopyShift:0;pc+=shift;
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
        if(c->pc+shift!=0x410640 || state.callbacks.contains(stack) || c->r[4].uw[0]!=point.emitter || !method || word(method)!=0x2262b0 ||
           (address+physicsShift!=0x40ac20 && address+physicsShift!=0x40acb0 && address+physicsShift!=0x40ad50 && address+physicsShift!=0x40ade4) || word(frame+0x20)!=address ||
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
       word(frame+0x20)!=call->second.returnAddress || word(emitter+0x6c)!=call->second.object || c->pc+shift!=(pc==0x410750?0x410640u:0x410790u))
        throw std::runtime_error("native particle callback retirement order mismatch");
    if(pc==0x410750 && c->r[2].uw[0]!=stack+0x30)throw std::runtime_error("particle callback spill address mismatch");
    if(pc==0x410790 && c->r[31].uw[0]+shift!=0x410790)throw std::runtime_error("particle callback did not return through original caller");
    if(pc==0x4107a8 && c->r[3].uw[0]!=word(record(rt,0x35dee0,16)+8))throw std::runtime_error("particle callback timestep source mismatch");
    if((pc==0x4107b0 || pc==0x4107b4) && c->r[3].uw[0]!=stack+0x30)throw std::runtime_error("particle callback restore address mismatch");
    if(pc==0x4107b4)state.callbacks.erase(call);else ++call->second.phase;return true;
}
bool nativeParticlePhysicsInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc>=0x40a950-kPhysicsCopyShift && pc<=0x40b040-kPhysicsCopyShift)pc+=kPhysicsCopyShift;   // the mod001 copy
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
struct ObjectPairRegisters{std::array<std::array<uint32_t,4>,8> rows{};std::array<uint32_t,4> scale{};bool valid=false;};
ObjectPairRegisters &objectPairRegisters(const Runtime *rt){static std::map<const Runtime *,ObjectPairRegisters> states;return states[rt];}
// Common-math 0x7f0 on an object's basis rows +90..+b0: the per-axis lengths
// (column norms) to out.xyz. The micro program leaves W as inherited register
// state; no reader.
void objectBasisLengths(Runtime *rt,uint32_t object,uint32_t out){
    float rows[3][4];std::memcpy(rows,record(rt,object+0x90,48),48);
    for(auto &row:rows)for(unsigned i=0;i<3;++i)row[i]=vu0Input(row[i]);
    float lengths[3];
    {music::NearestRounding rounding;
        for(unsigned i=0;i<3;++i){
            auto squared=pairSum(pairSum(pairProduct(rows[0][i],rows[0][i]),pairProduct(rows[1][i],rows[1][i])),pairProduct(rows[2][i],rows[2][i]));
            volatile float root=std::sqrt(squared);lengths[i]=pairSum(0.f,root);
        }}
    std::memcpy(record(rt,out,12),lengths,12);
}
// Object pair set-up, mod001 1f9130 (called from mod003 398324): the same two
// 0x7f0 basis-length calls (first a0's object s1 -> sp+40, then a1's s0 ->
// sp+30; scalar code takes each maximum to +204), then it leaves vf1..vf4 =
// s1+150, vf5..vf8 = s0+150 and vf9 = (s1+204, s0+204, stack, stack) for the
// shape test its caller runs: recorded as the object-pair registers.
bool nativeObjectPairSetup(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool firstScale=pc==0x1f92f0 || pc==0x1f92f4 || pc==0x1f92f8 || pc==0x1f9300 || pc==0x1f9304 || pc==0x1f930c;
    const bool secondScale=pc==0x1f9368 || pc==0x1f936c || pc==0x1f9370 || pc==0x1f9378 || pc==0x1f937c || pc==0x1f9384;
    const bool load=pc>=0x1f93c4 && pc<=0x1f93e4 && (pc&3)==0;
    if(!(firstScale || secondScale || load))return false;
    if(!c)throw std::runtime_error("native object pair set-up has no CPU record");
    const auto stack=c->r[29].uw[0],first=c->r[17].uw[0],second=c->r[16].uw[0];
    if(firstScale || secondScale){
        const auto object=firstScale?first:second;const auto out=stack+(firstScale?0x40:0x30);
        if(c->r[6].uw[0]!=object+0x90)throw std::runtime_error("native object pair basis mismatch");
        if(pc==0x1f9304 || pc==0x1f937c){c->r[9].ud[0]=0;return true;}
        if(pc!=0x1f930c && pc!=0x1f9384)return true;
        if(c->r[3].uw[0]!=out)throw std::runtime_error("native object pair scale destination mismatch");
        objectBasisLengths(rt,object,out);return true;
    }
    if(c->r[5].uw[0]!=first+0x150 || c->r[4].uw[0]!=second+0x150 || c->r[3].uw[0]!=stack+0x50)
        throw std::runtime_error("native object pair set-up escaped its records");
    if(pc!=0x1f93e4)return true;
    auto &regs=objectPairRegisters(rt);
    std::memcpy(regs.rows[0].data(),record(rt,first+0x150,64),64);std::memcpy(regs.rows[4].data(),record(rt,second+0x150,64),64);
    std::memcpy(regs.scale.data(),record(rt,stack+0x50,16),16);regs.valid=true;
    return true;
}
// The mod003 push-out loop 397780 (map 79) keeps the object-pair registers
// across two virtual calls: SQC2 vf1..vf9 to sp+3a0 (3984f4..398514), LQC2
// them back (3985e0..398600), then LQC2 vf8 from sp+460 (398608: the second
// object's position row, as its +20 method moved it) before 1f88d0 runs the
// shape test on them. The spill writes the registers' words, as the original.
bool nativeObjectPairSpill(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool spill=pc>=0x3984f4 && pc<=0x398514,restore=pc>=0x3985e0 && pc<=0x398600,moved=pc==0x398608;
    if(!(spill || restore || moved) || (pc&3) || !rt)return false;
    const unsigned index=spill?(pc-0x3984f4)/4:restore?(pc-0x3985e0)/4:7;
    const uint32_t expected=moved?0xd8480000u:(spill?0xf8600000u|((index+1)<<16):0xd8400000u|((index+1)<<16))|(moved?0:index*0x10);
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=expected)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native object pair spill has no CPU record");
    const auto stack=c->r[29].uw[0];auto &regs=objectPairRegisters(rt);
    const auto base=c->r[spill?3:2].uw[0];
    if(base!=stack+(moved?0x460:0x3a0))throw std::runtime_error("native object pair spill escaped its frame");
    auto *row=index<8?regs.rows[index].data():regs.scale.data();
    if(spill){if(!regs.valid)throw std::runtime_error("native object pair spill without registers");std::memcpy(record(rt,base+index*16,16),row,16);return true;}
    if(moved){if(!regs.valid)throw std::runtime_error("native object pair row without registers");std::memcpy(row,record(rt,base,16),16);return true;}
    if(index==0)regs.valid=false;
    std::memcpy(row,record(rt,base+index*16,16),16);if(index==8)regs.valid=true;
    return true;
}
// Common-math 0x7f0 basis-length blocks (LQC2 vf12..vf14 = object+90.., VCALLMS 0x7f0,
// CFC2.I t1/vi0, SQC2 vf6 to the output; the callers read x, y, z):
//   mod028 43c120: battle height set-up 43c0a0, object s0 (base a0), output sp+70.
//   mod001 239b54: 239b20 (scene object +50 method, called from 2203b8), object a0
//                  (base v0), output sp+0; it checks the lengths are 1 within 0.002.
//   mod028 4f56f8: battle shape test 4f5630, object s4 (base a2), output sp+60.
//   mod028 4f5a68: battle segment test 4f58f0, object s2 (base a2), output sp+c0.
struct BasisLengthSite{uint32_t load,call,store;unsigned object,base,output;uint32_t offset;};
//   mod028 4a6e10 / 4a6f40: battle object overlap 4a6c40, objects s3 / s2 (base a1), outputs sp+b0 / sp+a0.
//   mod001 1f5d90: shape tree 1f5c30, object s0 (base a1), output sp+50.
//   mod028 4a52a8: nearest-point walk 4a50b0's node path, object s1 (base a1), output sp+70.
constexpr BasisLengthSite basisLengthSites[]={{0x43c120,0x43c130,0x43c13c,16,4,3,0x70},{0x239b54,0x239b64,0x239b70,4,2,2,0},{0x4f56f8,0x4f5708,0x4f5714,20,6,3,0x60},{0x4f5a68,0x4f5a78,0x4f5a84,18,6,3,0xc0},
    {0x4a6e10,0x4a6e20,0x4a6e2c,19,5,2,0xb0},{0x4a6f40,0x4a6f50,0x4a6f5c,18,5,2,0xa0},{0x1f5d90,0x1f5da0,0x1f5dac,16,5,2,0x50},{0x4a52a8,0x4a52b8,0x4a52c4,17,5,2,0x70}};
bool nativeBattleBasisLengths(R5900Context *c,Runtime *rt,uint32_t pc){
    if(!rt || (pc&3))return false;
    const auto site=std::find_if(std::begin(basisLengthSites),std::end(basisLengthSites),[&](auto &s){return pc>=s.load && pc<=s.store;});
    if(site==std::end(basisLengthSites))return false;
    const unsigned slot=(pc-site->load)/4;
    const uint32_t expected=slot<3?0xd8000000u|(site->base<<21)|((12+slot)<<16)|(slot*16):pc==site->call?0x4a003fb8u:pc==site->call+4?0x48490001u:
        pc==site->store?0xf8060000u|(site->output<<21):pc==site->load+12?0u:1u;   // 1: a scalar word between (runs as compiled)
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(expected==1u || word!=expected)return false;   // scalar word, or another overlay at these addresses
    if(!c)throw std::runtime_error("native basis lengths have no CPU record");
    const auto object=c->r[site->object].uw[0];
    if(slot<3 && c->r[site->base].uw[0]!=object+0x90)throw std::runtime_error("native basis mismatch");
    if(pc==site->call+4){c->r[9].ud[0]=0;return true;}   // VI0 is the constant zero
    if(pc!=site->store)return true;
    if(c->r[site->output].uw[0]!=c->r[29].uw[0]+site->offset)throw std::runtime_error("native basis destination mismatch");
    objectBasisLengths(rt,object,c->r[site->output].uw[0]);return true;
}
// VU FTOIn on one ps2f word: value * 2^n truncated toward zero, saturating to int32; a
// denormal reads as zero and exponent 255 is an ordinary (huge) number.
uint32_t vuFtoi(uint32_t bits,unsigned shift){
    const unsigned exponent=(bits>>23)&255;if(!exponent)return 0;
    const double magnitude=std::ldexp(double((bits&0x7fffff)|0x800000),int(exponent)-150+int(shift));
    const double value=bits>>31?-magnitude:magnitude;
    if(value>=2147483647.)return 0x7fffffffu;if(value<=-2147483648.)return 0x80000000u;return uint32_t(int32_t(value));
}
// Battle vertex packer, mod028 4d7150 (a0 builder, a1 position, a2 vector): +d1 == 0 takes
// FTOI12 of a2 (LQC2 4d71cc, FTOI12 4d71d0, SQC2 4d71d4 to sp+0); +d0 == 2 takes FTOI4 of
// a1 (LQC2 4d7260, FTOI4 4d7264, SQC2 4d7268 to sp+0). The scalar code packs the halves.
// Its sibling 4d7060 (a0 builder, a1 position, a2/a3 words) takes FTOI4 of a1 the same way
// (LQC2 4d7068, FTOI4 4d706c, SQC2 4d7070 to t2 = sp+0) before packing x, y, z as halfwords.
bool nativeBattleVertexPack(R5900Context *c,Runtime *rt,uint32_t pc){
    constexpr struct {uint32_t pc,word;} sites[]={{0x4d71cc,0xd8c10000},{0x4d71d0,0x4be1097e},{0x4d71d4,0xf8610000},{0x4d7260,0xd8a10000},{0x4d7264,0x4be1097d},{0x4d7268,0xf8c10000},
        {0x4d7068,0xd8a10000},{0x4d706c,0x4be1097d},{0x4d7070,0xf9410000}};
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](auto &s){return s.pc==pc;});
    if(site==std::end(sites) || !rt)return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle vertex pack has no CPU record");
    if(pc!=0x4d71d4 && pc!=0x4d7268 && pc!=0x4d7070)return true;
    const bool normal=pc==0x4d71d4;const auto source=c->r[normal?6:5].uw[0],out=c->r[normal?3:pc==0x4d7070?10:6].uw[0];
    if(out!=c->r[29].uw[0])throw std::runtime_error("native battle vertex pack destination mismatch");
    uint32_t in[4],fixed[4];std::memcpy(in,record(rt,source,16),16);
    for(unsigned i=0;i<4;++i)fixed[i]=vuFtoi(in[i],normal?12:4);
    std::memcpy(record(rt,out,16),fixed,16);return true;
}
// Battle shape set-up, mod028 48ee30(a0 = s5 shape, a1 = source, a2 = s4 optional matrix):
// LQC2 vf1 = a1+10 (48ee74), MULw.w vf1.w = 1 (48ee88, a delay slot: always), and with s4
// LQC2 vf2 = (s4+c, s4+1c, s4+2c) from sp+90 (48ef84) and ADD.xyz vf1 += vf2 (48ef88);
// SQC2 vf1 to s5+1b0 (48ef94). Between them the ring loop calls sin/cos (213fd0), so the
// value is held per call frame (sp) from its load to its store; no state boundary in between.
struct BattleOrigin{std::array<float,4> value{};bool offset=false;};
std::map<std::pair<const Runtime *,uint32_t>,BattleOrigin> &battleOrigins(){static std::map<std::pair<const Runtime *,uint32_t>,BattleOrigin> m;return m;}
bool nativeBattleShapeOrigin(R5900Context *c,Runtime *rt,uint32_t pc){
    constexpr struct {uint32_t pc,word;} sites[]={{0x48ee74,0xd8610000},{0x48ee88,0x4a20005b},{0x48ef84,0xd8820000},{0x48ef88,0x4bc20868},{0x48ef94,0xf8610000}};
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](auto &s){return s.pc==pc;});
    if(site==std::end(sites) || !rt)return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle shape origin has no CPU record");
    const auto shape=c->r[21].uw[0],stack=c->r[29].uw[0];const auto key=std::make_pair(static_cast<const Runtime *>(rt),stack);
    auto &origins=battleOrigins();
    if(pc==0x48ee74){
        if(c->r[3].uw[0]!=c->r[5].uw[0]+0x10)throw std::runtime_error("native battle shape origin source mismatch");
        BattleOrigin origin;float in[4];std::memcpy(in,record(rt,c->r[3].uw[0],16),16);
        for(unsigned i=0;i<3;++i)origin.value[i]=vu0Input(in[i]);origin.value[3]=1;   // w: MULw.w vf0w*vf0w at 48ee88
        origins[key]=origin;return true;
    }
    auto it=origins.find(key);if(it==origins.end())throw std::runtime_error("native battle shape origin has no load");
    if(pc==0x48ee88)return true;
    if(pc==0x48ef84){
        if(c->r[4].uw[0]!=stack+0x90 || !c->r[20].uw[0] || it->second.offset)throw std::runtime_error("native battle shape offset escaped its record");
        return true;
    }
    if(pc==0x48ef88){
        float in[4];std::memcpy(in,record(rt,stack+0x90,16),16);music::NearestRounding rounding;
        for(unsigned i=0;i<3;++i)it->second.value[i]=pairSum(it->second.value[i],vu0Input(in[i]));
        it->second.offset=true;return true;
    }
    if(c->r[3].uw[0]!=shape+0x1b0 || it->second.offset!=(c->r[20].uw[0]!=0))throw std::runtime_error("native battle shape origin destination mismatch");
    std::memcpy(record(rt,shape+0x1b0,16),it->second.value.data(),16);origins.erase(it);return true;
}
// Battle shape world point, mod028 48edb0(a0 shape, a1 matrix): unless +1d is set, the
// same row-matrix * point block as the height query (252ec8, same accumulation order)
// takes the shape origin a2 = a0+1b0 through the matrix's rows to v1 = a0+1c0.
bool nativeBattleShapePoint(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc<0x48edc0 || pc>0x48ee18 || (pc&3) || !rt)return false;
    constexpr uint32_t words[]={0xd8cf0000,0xd8a10000,0xd8a20010,0xd8a30020,0xd8a40030,0x4bc00183,0x4bef0aea,0x4bef132a,0x4bef1b6a,0x4bef23aa,
        0x4b0b583d,0x4b0b30be,0x4b0b33cb,0x4a8c603c,0x4a8c30be,0x4a8c33cb,0x4a4d683d,0x4a4d30bc,0x4a4d33cb,0x4a2e703d,0x4a2e00be,0x4a2e03c8,0xf86f0000};
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=words[(pc-0x48edc0)/4])return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle shape point has no CPU record");
    const auto shape=c->r[4].uw[0];
    if(c->r[6].uw[0]!=shape+0x1b0 || c->r[3].uw[0]!=shape+0x1c0)throw std::runtime_error("native battle shape point escaped its records");
    if(pc!=0x48ee18)return true;
    const auto result=rowPoint(rt,c->r[5].uw[0],shape+0x1b0);std::memcpy(record(rt,shape+0x1c0,16),result.data(),16);return true;
}
// Battle box corners, mod028 48f0d0(a0 box, a1 = extent at +0 / centre at +10, a2 matrix
// or 0): the centre's 16 bytes go to a0+180 as loaded; vf1..vf8 = centre with w = 1
// (MOVE.w from vf0), each lane +-extent (VU0 add/sub, nearest) in the order below, stored
// to a0+20..+90. With a2, the loop 48f188..48f1cc takes each corner in place through the
// matrix's rows (the height query's row-point block, same accumulation order).
bool nativeBattleBoxCorners(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool build=pc>=0x48f0dc && pc<=0x48f164,loop=pc>=0x48f168 && pc<=0x48f1cc;
    if(!(build || loop) || (pc&3) || !rt)return false;
    constexpr uint32_t buildWords[]={0xd8e10000,0xd8aa0000,0xf8610000,0x4a21033c,0x4be20b3c,0x4be30b3c,0x4be40b3c,0x4be50b3c,0x4be60b3c,0x4be70b3c,0x4be80b3c,0x24830020,
        0x4b4a0868,0x4b0a1080,0x4bca18e8,0x4b8a2128,0x4a4a2942,0x4aca39e8,0x4a8a4201,0x4a8a0845,0x4aca10ac,0x4a4a2106,0x4b8a296c,0x4bca31ac,0x4b0a39c4,0x4b4a422c,
        0xf8610000,0xf8620010,0xf8630020,0xf8640030,0xf8650040,0xf8660050,0xf8670060,0x10c00021,0xf8680070};
    constexpr uint32_t loopWords[]={0xd8cb0000,0xd8cc0010,0xd8cd0020,0xd8ce0030,0x4bc007c3,0x24850020,0x10000015,0x0000202d,0x4be15d6a,0x4be165aa,0x4be16dea,0x4be1762a,
        0xd8a10010,0x4b15a83d,0x4b15f8be,0x4b15f94b,0x4a96b03c,0x4a96f8be,0x4a96f94b,0x4a57b83d,0x4a57f8bc,0x4a57f94b,0x4a38c03d,0x4a3800be,0x4a380148,0xf8a50000};
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=(build?buildWords[(pc-0x48f0dc)/4]:loopWords[(pc-0x48f168)/4]))return false;   // another overlay at these addresses
    if(const auto op=word>>26;op!=0x12 && op!=0x36 && op!=0x3e)return false;   // the scalar words run as compiled
    if(!c)throw std::runtime_error("native battle box has no CPU record");
    if(build){
        if(pc!=0x48f0dc)return true;
        const auto box=c->r[4].uw[0],source=c->r[5].uw[0];
        if(c->r[7].uw[0]!=source+0x10 || c->r[3].uw[0]!=box+0x180)throw std::runtime_error("native battle box escaped its records");
        float centre[4],extent[4];std::memcpy(centre,record(rt,source+0x10,16),16);std::memcpy(extent,record(rt,source,16),16);
        float in[3],half[3];for(unsigned i=0;i<3;++i){in[i]=vu0Input(centre[i]);half[i]=vu0Input(extent[i]);}
        std::memcpy(record(rt,box+0x180,16),centre,16);
        constexpr int signs[8][3]={{1,-1,1},{1,-1,-1},{1,1,1},{1,1,-1},{-1,-1,1},{-1,-1,-1},{-1,1,1},{-1,1,-1}};
        float corners[8][4];music::NearestRounding rounding;
        for(unsigned k=0;k<8;++k){for(unsigned i=0;i<3;++i)corners[k][i]=pairSum(in[i],signs[k][i]>0?half[i]:-half[i]);corners[k][3]=1;}
        std::memcpy(record(rt,box+0x20,128),corners,128);return true;
    }
    if(pc!=0x48f1cc)return true;
    const auto index=c->r[4].uw[0],at=c->r[5].uw[0],matrix=c->r[6].uw[0];
    if(index>=8 || !matrix)throw std::runtime_error("native battle box transform escaped its loop");
    const auto result=rowPoint(rt,matrix,at);std::memcpy(record(rt,at,16),result.data(),16);return true;
}
// Battle box to world, mod028 48efd0(a0 box, a1 matrix): unless +1d, LQC2 vf1..vf4 = the
// matrix rows (48efdc..48efe8), then each corner a0+20+16k to a0+a0+16k (loop 48f000..
// 48f044, a1 reused as its counter) and the centre a0+180 to a0+190 (48f06c..48f0b0), all
// by the row-point block. The leaf has no calls: the matrix address is held from its load.
std::map<std::pair<const Runtime *,uint32_t>,uint32_t> &battleBoxMatrices(){static std::map<std::pair<const Runtime *,uint32_t>,uint32_t> m;return m;}
bool nativeBattleBoxWorld(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool load=pc>=0x48efdc && pc<=0x48efec,corner=pc>=0x48f000 && pc<=0x48f044,centre=pc>=0x48f06c && pc<=0x48f0b0;
    if(!(load || corner || centre) || (pc&3) || !rt)return false;
    constexpr uint32_t block[]={0x4bef0aea,0x4bef132a,0x4bef1b6a,0x4bef23aa,0x4b0b583d,0x4b0bf8be,0x4b0bfbcb,0x4a8c603c,0x4a8cf8be,0x4a8cfbcb,
        0x4a4d683d,0x4a4df8bc,0x4a4dfbcb,0x4a2e703d,0x4a2e00be,0x4a2e03c8};   // MUL vf11..vf14 .. MADDx.w vf15
    constexpr uint32_t loadWords[]={0xd8a10000,0xd8a20010,0xd8a30020,0xd8a40030,0x4bc007c3};
    uint32_t expected;
    if(load)expected=loadWords[(pc-0x48efdc)/4];
    else{const auto first=corner?0x48f000u:0x48f06cu,last=corner?0x48f044u:0x48f0b0u;
        expected=pc==first?(corner?0xd8ef0000u:0xd8af0000u):pc==last?(corner?0xf8cf0000u:0xf86f0000u):block[(pc-first)/4-1];}
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=expected)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle box world has no CPU record");
    const auto box=c->r[4].uw[0];const auto key=std::make_pair(static_cast<const Runtime *>(rt),box);auto &matrices=battleBoxMatrices();
    if(load){
        if(pc==0x48efdc){const auto matrix=c->r[5].uw[0];record(rt,matrix,64);
            if(matrix<box+0x1a0 && box+0xa0<matrix+64)throw std::runtime_error("native battle box matrix overlaps its outputs");
            matrices[key]=matrix;}
        return true;
    }
    auto it=matrices.find(key);if(it==matrices.end())throw std::runtime_error("native battle box world has no matrix load");
    if(corner){
        if(pc!=0x48f044)return true;
        const auto index=c->r[5].uw[0],from=c->r[7].uw[0],to=c->r[6].uw[0];
        if(index>=8 || from!=box+0x20+16*index || to!=box+0xa0+16*index)throw std::runtime_error("native battle box corner escaped its loop");
        const auto result=rowPoint(rt,it->second,from);std::memcpy(record(rt,to,16),result.data(),16);return true;
    }
    if(c->r[5].uw[0]!=box+0x180 || c->r[3].uw[0]!=box+0x190)throw std::runtime_error("native battle box centre escaped its records");
    if(pc!=0x48f0b0)return true;
    const auto result=rowPoint(rt,it->second,box+0x180);std::memcpy(record(rt,box+0x190,16),result.data(),16);matrices.erase(it);return true;
}
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
        objectPairRegisters(rt).valid=false;   // vf1..vf9 belong to the caller again
        return true;
    }
    if(firstScale || secondScale){
        const auto object=firstScale?first:second;const auto out=stack+(firstScale?0x70:0x60);
        if(c->r[5].uw[0]!=object+0x90)throw std::runtime_error("native object scale basis mismatch");
        if(pc==0x1f8d14 || pc==0x1f8d8c){c->r[9].ud[0]=0;return true;}
        if(pc!=0x1f8d1c && pc!=0x1f8d94)return true;
        if(c->r[2].uw[0]!=out)throw std::runtime_error("native object scale destination mismatch");
        objectBasisLengths(rt,object,out);return true;
    }
    if(c->r[4].uw[0]!=c->r[20].uw[0]+0x10 || c->r[3].uw[0]!=c->r[19].uw[0]+0x10 || c->r[6].uw[0]!=first+0x150 || c->r[5].uw[0]!=second+0x150)
        throw std::runtime_error("native object overlap escaped its records");
    if(pc!=0x1f8e58){if(pc<0x1f8e50 && c->r[2].uw[0]!=stack+0x90)throw std::runtime_error("native object overlap scale record mismatch");return true;}
    if(c->r[2].uw[0]!=stack+0x80)throw std::runtime_error("native object overlap destination mismatch");
    {   // vf1..vf4 / vf5..vf8 / vf9 as the block loaded them; the shape tests reached
        // when the spheres overlap (box pair 1f82c0) run on them.
        auto &regs=objectPairRegisters(rt);
        std::memcpy(regs.rows[0].data(),record(rt,first+0x150,64),64);std::memcpy(regs.rows[4].data(),record(rt,second+0x150,64),64);
        std::memcpy(regs.scale.data(),record(rt,stack+0x90,16),16);regs.valid=true;
    }
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
// Battle object overlap, mod028 4a6c40: the battle's own compile of 1f8aa0's algorithm (not
// a byte copy). SQC2 vf1..vf9 to sp+d0 on entry (4a6cf4..4a6d14) and LQC2 them back on each
// exit (4a7034.., 4a7250.., 4a7290..): no native state. Its two 0x7f0 basis-length calls are
// basisLengthSites rows. The overlap block (4a6fa4..4a701c): vf9 = sp+c0 (the two radii
// +204 the CPU just stored), vf10 = a1 (s5+10), vf11 = a0 (s4+10), vf1..vf4 = s3+150, vf5..vf8 =
// s2+150; vf10.w *= vf9.x, vf11.w *= vf9.y; vf12 = M1 vf10, vf13 = M2 vf11 (w by 1);
// ((dx^2 + dy^2) - (r1 + r2)^2) + dz^2 to sp+c0 .x (y, z the squares, w M1's). vf1..vf9 stay
// loaded for the shape tests that follow: recorded as the object-pair registers, as 1f8aa0's.
bool nativeBattleObjectOverlap(R5900Context *c,Runtime *rt,uint32_t pc){
    if((pc&3) || !rt || pc<0x4a6cf4 || pc>0x4a72b0)return false;
    const bool spill=pc<=0x4a6d14,restore=(pc>=0x4a7034 && pc<=0x4a7054) || (pc>=0x4a7250 && pc<=0x4a7270) || (pc>=0x4a7290 && pc<=0x4a72b0);
    constexpr uint32_t block[]={0x4a6fa4,0x4a6fa8,0x4a6fac,0x4a6fb0,0x4a6fb4,0x4a6fb8,0x4a6fbc,0x4a6fc0,0x4a6fc4,0x4a6fc8,0x4a6fcc,0x4a6fd0,0x4a6fd4,0x4a6fd8,0x4a6fdc,
        0x4a6fe0,0x4a6fe4,0x4a6fe8,0x4a6fec,0x4a6ff0,0x4a6ff4,0x4a6ff8,0x4a6ffc,0x4a7000,0x4a7004,0x4a7008,0x4a700c,0x4a7010,0x4a7014,0x4a7018,0x4a701c};
    constexpr uint32_t words[]={0xd8c90000,0xd8aa0000,0xd88b0000,0xd8610000,0xd8620010,0x4a295298,0x4a295ad9,0xd8630020,0xd8640030,0xd8450000,0xd8460010,0xd8470020,0xd8480030,
        0x4bea09bc,0x4bea10bd,0x4bea18be,0x4be0230b,0x4beb29bc,0x4beb30bd,0x4beb38be,0x4be0434b,0x4bcd632c,0x4a2b52a8,0x4bcc632a,0x4b000343,0x4a2a52aa,0x4b0c69bc,0x4b0c68bd,
        0x4b0a68ff,0x4b0c6b0a,0xf8cc0000};
    const auto at=std::find(std::begin(block),std::end(block),pc);
    if(!spill && !restore && at==std::end(block))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    const uint32_t expected=spill?0xf8400000u|((pc-0x4a6cf4)/4+1)<<16|(pc-0x4a6cf4)*4:
        restore?0xd8400000u|((pc-(pc>=0x4a7290?0x4a7290:pc>=0x4a7250?0x4a7250:0x4a7034))/4+1)<<16|(pc-(pc>=0x4a7290?0x4a7290:pc>=0x4a7250?0x4a7250:0x4a7034))*4:words[at-std::begin(block)];
    if(word!=expected)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle object overlap has no CPU record");
    const auto stack=c->r[29].uw[0],first=c->r[19].uw[0],second=c->r[18].uw[0];
    if(spill || restore){
        if(c->r[2].uw[0]!=stack+0xd0)throw std::runtime_error("native battle object overlap spill escaped its frame");
        objectPairRegisters(rt).valid=false;   // vf1..vf9 belong to the caller again
        return true;
    }
    if(c->r[6].uw[0]!=stack+0xc0 || c->r[5].uw[0]!=c->r[21].uw[0]+0x10 || c->r[4].uw[0]!=c->r[20].uw[0]+0x10 || c->r[3].uw[0]!=first+0x150 ||
       (pc<0x4a6fd8 && pc>=0x4a6fc8 && c->r[2].uw[0]!=second+0x150))
        throw std::runtime_error("native battle object overlap escaped its records");
    if(pc!=0x4a701c)return true;
    auto &regs=objectPairRegisters(rt);
    std::memcpy(regs.rows[0].data(),record(rt,first+0x150,64),64);std::memcpy(regs.rows[4].data(),record(rt,second+0x150,64),64);
    std::memcpy(regs.scale.data(),record(rt,stack+0xc0,16),16);regs.valid=true;
    float scale[4],a[4],b[4],m1[4][4],m2[4][4];
    std::memcpy(scale,record(rt,stack+0xc0,16),16);std::memcpy(a,record(rt,c->r[5].uw[0],16),16);std::memcpy(b,record(rt,c->r[4].uw[0],16),16);
    std::memcpy(m1,record(rt,first+0x150,64),64);std::memcpy(m2,record(rt,second+0x150,64),64);
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
        for(float v:{total,t1[3]})if(!std::isfinite(v))throw std::runtime_error("native battle object overlap overflow");
        result[0]=total;result[1]=squared[1];result[2]=squared[2];result[3]=t1[3];
    }
    std::memcpy(record(rt,stack+0xc0,16),result,16);return true;
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
// Lens-flare light projection, mod001 flare owner (block entry 256038). LQC2
// vf1..vf4 load the rows at SP+360 (no 0x1f0 canonicalization: program 0 reads
// them as its column registers), the FPU code runs (its call 217370 uses no
// COP2), LQC2 vf5 loads the light at SP+340, VCALLMS 0, CFC2.I t1/vi0, SQC2 vf6
// to SP+2c0. Program 0: ACC = vf1*v.x + vf2*v.y + vf3*v.z, vf6 = ACC + vf4*v.w,
// Q = 1/vf6.w, vf6.w = 1, vf6 *= Q: (p.x*Q, p.y*Q, p.z*Q, Q). The rows are
// captured at their loads; a path that leaves before the store (no intensity)
// just drops them.
struct FlareProjection{float rows[4][4]{};unsigned loaded=0;bool point=false;float v[4]{};};
// Common-math program 0 on directly loaded column registers: ACC = vf1*v.x +
// vf2*v.y + vf3*v.z, p = ACC + vf4*v.w, Q = 1/p.w, (p.x*Q, p.y*Q, p.z*Q, Q).
std::array<float,4> program0Projection(const FlareProjection &state,const char *what){
    std::array<float,4> result;
    {music::NearestRounding rounding;
        float p[4];
        for(unsigned i=0;i<4;++i)
            p[i]=pairSum(pairSum(pairSum(pairProduct(state.rows[0][i],state.v[0]),pairProduct(state.rows[1][i],state.v[1])),pairProduct(state.rows[2][i],state.v[2])),pairProduct(state.rows[3][i],state.v[3]));
        if((std::bit_cast<uint32_t>(p[3])&0x7fffffff)==0)throw std::runtime_error(std::format("native {} projection has zero depth",what));
        volatile float q=1.f/p[3];const auto reciprocal=pairNormal(q);
        for(unsigned i=0;i<3;++i)result[i]=pairProduct(p[i],reciprocal);result[3]=pairProduct(1.f,reciprocal);
        for(float x:result)if(!std::isfinite(x))throw std::runtime_error(std::format("native {} projection overflow",what));
    }
    return result;
}
FlareProjection &flareProjection(const Runtime *rt){static std::map<const Runtime *,FlareProjection> states;return states[rt];}
bool nativeFlareProjection(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool row=pc>=0x256068 && pc<=0x256074 && !(pc&3);
    if(!row && pc!=0x256170 && pc!=0x25617c && pc!=0x256180 && pc!=0x256188)return false;
    if(!c)throw std::runtime_error("native flare projection has no CPU record");
    const auto stack=c->r[29].uw[0],base=c->r[3].uw[0];auto &state=flareProjection(rt);
    if(row){
        if(base!=stack+0x360)throw std::runtime_error("native flare projection matrix source mismatch");
        const unsigned k=(pc-0x256068)/4;if(k==0){state.loaded=0;state.point=false;}
        std::memcpy(state.rows[k],record(rt,base+16*k,16),16);for(float &x:state.rows[k])x=vu0Input(x);
        state.loaded|=1u<<k;return true;
    }
    if(pc==0x256170){
        if(base!=stack+0x340)throw std::runtime_error("native flare projection light source mismatch");
        std::memcpy(state.v,record(rt,base,16),16);for(float &x:state.v)x=vu0Input(x);state.point=true;return true;
    }
    if(pc==0x25617c)return true;
    if(pc==0x256180){c->r[9].ud[0]=0;return true;}
    if(base!=stack+0x2c0)throw std::runtime_error("native flare projection destination mismatch");
    if(state.loaded!=15 || !state.point)throw std::runtime_error("native flare projection stored without its matrix and light");
    const auto result=program0Projection(state,"flare");std::memcpy(record(rt,base,16),result.data(),16);state.loaded=0;state.point=false;return true;
}
// Battle camera projections, mod028 (384b0c, 384b74, 387330 in 386fbc, 4c8670): the camera
// rows at *(*(*36d880+80)+18) loaded in place, R..R+c: LQC2 vf1..vf4 = 0..30(rA); R+14:
// LQC2 vf5 = 0(rB) (the point); R+20: VCALLMS 0; R+24: CFC2.I t1/vi0; R+2c: SQC2 vf6 to
// 0(rC): program 0 (program0Projection). R+10/18/1c/28 are CPU code (pointer moves, nops).
FlareProjection &battleCameraProjection(const Runtime *rt,uint32_t site){static std::map<std::pair<const Runtime *,uint32_t>,FlareProjection> m;return m[{rt,site}];}
bool nativeBattleCameraProjection(R5900Context *c,Runtime *rt,uint32_t pc){
    constexpr uint32_t sites[]={0x384b0c,0x384b74,0x387330,0x4c8670};
    if((pc&3) || !rt)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](uint32_t r){return pc>=r && pc<=r+0x2c;});
    if(site==std::end(sites))return false;
    const unsigned slot=(pc-*site)/4;
    if(slot==4 || slot==6 || slot==7 || slot==10)return false;   // CPU code between
    const auto word=[&](uint32_t at){uint32_t w=0;if(const auto *p=rt->memory.hostPointer(at,4))std::memcpy(&w,p,4);return w;};
    const uint32_t first=word(*site),rows=(first>>21)&31;
    for(unsigned k=0;k<4;++k)if(word(*site+4*k)!=(0xd8000000u|rows<<21|(k+1)<<16|16*k))return false;   // another overlay here
    const uint32_t point=word(*site+0x14),store=word(*site+0x2c);
    if((point&0xfc1fffffu)!=0xd8050000u || word(*site+0x20)!=0x4a000038u || word(*site+0x24)!=0x48490001u || (store&0xfc1fffffu)!=0xf8060000u)return false;
    if(!c)throw std::runtime_error("native battle camera projection has no CPU record");
    auto &state=battleCameraProjection(rt,*site);
    if(slot<4){if(slot==0){state.loaded=0;state.point=false;}
        std::memcpy(state.rows[slot],record(rt,c->r[rows].uw[0]+16*slot,16),16);for(float &x:state.rows[slot])x=vu0Input(x);state.loaded|=1u<<slot;return true;}
    if(slot==5){std::memcpy(state.v,record(rt,c->r[(point>>21)&31].uw[0],16),16);for(float &x:state.v)x=vu0Input(x);state.point=true;return true;}
    if(slot==8)return true;
    if(slot==9){c->r[9].ud[0]=0;return true;}
    if(state.loaded!=15 || !state.point)throw std::runtime_error(std::format("native battle camera projection {:x} stored without its rows and point",*site));
    const auto result=program0Projection(state,"battle camera");std::memcpy(record(rt,c->r[(store>>21)&31].uw[0],16),result.data(),16);
    state.loaded=0;state.point=false;return true;
}
// Battle unit projection, mod028 4d10a0: LQC2 vf1..vf4 = the camera rows at
// *(*(*36d880+80)+18) once (4d1140..4d114c), then for each of 4 units: LQC2 vf5 = sp+80
// (4d11e0), VCALLMS 0, CFC2.I t1/vi0, SQC2 vf6 to sp+70 (4d11f8): program 0 (see
// program0Projection). Between the loop's sites run only scalar code, the unit's
// virtual +28 getter and 42db50 (no COP2), so the rows are captured once and kept.
FlareProjection &battleUnitProjection(const Runtime *rt){static std::map<const Runtime *,FlareProjection> states;return states[rt];}
bool nativeBattleUnitProjection(R5900Context *c,Runtime *rt,uint32_t pc){
    constexpr std::pair<uint32_t,uint32_t> sites[]={{0x4d1140,0xd8410000},{0x4d1144,0xd8420010},{0x4d1148,0xd8430020},{0x4d114c,0xd8440030},
        {0x4d11e0,0xd8850000},{0x4d11ec,0x4a000038},{0x4d11f0,0x48490001},{0x4d11f8,0xf8460000}};
    if((pc&3) || !rt || pc<0x4d1140 || pc>0x4d11f8)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](auto &x){return x.first==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->second)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle unit projection has no CPU record");
    auto &state=battleUnitProjection(rt);const auto stack=c->r[29].uw[0];
    if(pc<=0x4d114c){const unsigned k=(pc-0x4d1140)/4;if(k==0){state.loaded=0;state.point=false;}
        std::memcpy(state.rows[k],record(rt,c->r[2].uw[0]+16*k,16),16);for(float &x:state.rows[k])x=vu0Input(x);state.loaded|=1u<<k;return true;}
    if(pc==0x4d11e0){if(c->r[4].uw[0]!=stack+0x80)throw std::runtime_error("native battle unit projection point source mismatch");
        std::memcpy(state.v,record(rt,stack+0x80,16),16);for(float &x:state.v)x=vu0Input(x);state.point=true;return true;}
    if(pc==0x4d11ec)return true;
    if(pc==0x4d11f0){c->r[9].ud[0]=0;return true;}
    if(c->r[2].uw[0]!=stack+0x70)throw std::runtime_error("native battle unit projection destination mismatch");
    if(state.loaded!=15 || !state.point)throw std::runtime_error("native battle unit projection stored without its rows and point");
    const auto result=program0Projection(state,"battle unit");std::memcpy(record(rt,stack+0x70,16),result.data(),16);state.point=false;return true;
}
// Battle label projection, mod028 4fcae0 (what the projected points label is not identified): LQC2 vf1..vf4 = the camera rows at
// *(*(*36d880+80)+18) once (4fcb3c..4fcb48), then program-0 blocks (LQC2 vf5 = (v0), VCALLMS
// 0, CFC2.I t1/vi0, SQC2 vf6 to (v0)) at 4fcd14, 4fce04, 4fceec, 4fcfdc (to sp+e0) and in its
// only callee that projects, 4fc750 (frame 0x70, called from 4fcbcc only), at 4fc884 (to its
// sp+50). Between them run scalar code and calls whose only COP2 (main 214100) uses vf12..vf18
// and vf31, so the rows stay the loaded ones; each block checks it runs in the loading frame.
struct BattleLabelRows{FlareProjection rows;uint32_t frame=0;};
BattleLabelRows &battleLabelRows(const Runtime *rt){static std::map<const Runtime *,BattleLabelRows> states;return states[rt];}
bool nativeBattleLabelProjection(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Block{uint32_t at,frameOffset;};
    constexpr Block blocks[]={{0x4fcd14,0},{0x4fce04,0},{0x4fceec,0},{0x4fcfdc,0},{0x4fc884,0x70}};
    if((pc&3) || !rt || pc<0x4fc884 || pc>0x4fcff4)return false;
    const bool row=pc>=0x4fcb3c && pc<=0x4fcb48;
    const Block *block=nullptr;unsigned slot=0;
    if(!row){for(const auto &b:blocks)if(pc>=b.at && pc<=b.at+0x18){block=&b;slot=(pc-b.at)/4;}
        if(!block || (slot!=0 && slot!=3 && slot!=4 && slot!=6))return false;}
    const uint32_t expected=row?0xd8400000u|((pc-0x4fcb3c)/4+1)<<16|(pc-0x4fcb3c)*4:slot==0?0xd8450000u:slot==3?0x4a000038u:slot==4?0x48490001u:0xf8460000u;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=expected)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle label projection has no CPU record");
    auto &st=battleLabelRows(rt);const auto stack=c->r[29].uw[0];
    if(row){const unsigned k=(pc-0x4fcb3c)/4;if(k==0){st=BattleLabelRows{};st.frame=stack;}
        else if(st.frame!=stack || st.rows.loaded!=(1u<<k)-1)throw std::runtime_error("native battle label rows out of order");
        std::memcpy(st.rows.rows[k],record(rt,c->r[2].uw[0]+16*k,16),16);for(float &x:st.rows.rows[k])x=vu0Input(x);st.rows.loaded|=1u<<k;return true;}
    if(st.rows.loaded!=15 || st.frame!=stack+block->frameOffset)throw std::runtime_error(std::format("native battle label projection at {:x} outside the frame that loaded its rows",pc));
    if(slot==0){std::memcpy(st.rows.v,record(rt,c->r[2].uw[0],16),16);for(float &x:st.rows.v)x=vu0Input(x);st.rows.point=true;return true;}
    if(slot==3)return true;
    if(slot==4){c->r[9].ud[0]=0;return true;}
    if(!st.rows.point)throw std::runtime_error("native battle label projection stored without its point");
    const auto result=program0Projection(st.rows,"battle label");std::memcpy(record(rt,c->r[2].uw[0],16),result.data(),16);st.rows.point=false;return true;
}
// Two-point projection, mod001 2dfeb8 (block 2e0184). The MMI transpose before
// it writes the rows at SP+120; LQC2 vf1..vf4 load them (2e0234..2e0240). Then
// twice (2e0248 / 2e02e8): LQC2 vf5 = SP+160, VCALLMS 0, CFC2.I t1/vi0, SQC2
// vf6 to SP+170. Only FPU/integer code runs between the sites, so vf1..vf4 hold
// the same rows for both calls: they are captured at their loads and kept.
FlareProjection &pairProjection(const Runtime *rt){static std::map<const Runtime *,FlareProjection> states;return states[rt];}
bool nativePairProjection(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool row=pc>=0x2e0234 && pc<=0x2e0240 && !(pc&3),load=pc==0x2e0248 || pc==0x2e02e8,call=pc==0x2e0254 || pc==0x2e02f4;
    const bool status=pc==0x2e0258 || pc==0x2e02f8,store=pc==0x2e0260 || pc==0x2e0300;
    if(!row && !load && !call && !status && !store)return false;
    if(!c)throw std::runtime_error("native pair projection has no CPU record");
    const auto stack=c->r[29].uw[0],base=c->r[3].uw[0];auto &state=pairProjection(rt);
    if(row){
        if(base!=stack+0x120)throw std::runtime_error("native pair projection matrix source mismatch");
        const unsigned k=(pc-0x2e0234)/4;if(k==0){state.loaded=0;state.point=false;}
        std::memcpy(state.rows[k],record(rt,base+16*k,16),16);for(float &x:state.rows[k])x=vu0Input(x);
        state.loaded|=1u<<k;return true;
    }
    if(load){
        if(base!=stack+0x160)throw std::runtime_error("native pair projection point source mismatch");
        std::memcpy(state.v,record(rt,base,16),16);for(float &x:state.v)x=vu0Input(x);state.point=true;return true;
    }
    if(call)return true;
    if(status){c->r[9].ud[0]=0;return true;}
    if(base!=stack+0x170)throw std::runtime_error("native pair projection destination mismatch");
    if(state.loaded!=15 || !state.point)throw std::runtime_error("native pair projection stored without its matrix and point");
    const auto result=program0Projection(state,"pair");std::memcpy(record(rt,base,16),result.data(),16);state.point=false;return true;
}
// Exponential ramp, mod001 2df060 (block 2df0e8). The loads put (1,1,1,1) SP+50
// in vf23, (255,255,255,128) SP+40 in vf24, s1+13470 in vf1, SP+60 in vf26
// (X used), the coefficient rows 3578b0 in vf20..vf22 and SP+30 in vf25; vf13 =
// 0. Then 256 times (2df2c0..2df384), with i = vf13.x = a1:
//   q = max(vf25.x / (i*vf25.z + vf25.y), vf26.x); x = vf1*q (all lanes);
//   e = ((((1+x) + x2*c0 + x3*c1 + x4*c2 + x5*c3) + x6*c4 ... ) + x13*c11), with
//   x2=x*x, x4=x2*x2, x3=x2*x, x5=x4*x, then each power times x4 for the next four;
//   r = 1/e per lane; xyz = vf23 - r; out = FTOI0(min(vf24*(xyz, r.w), vf24.x));
//   QMFC2.I a0 = out (the MMI pack and the store to s0[i] stay compiled).
// No VU state survives an iteration except the counter, so the native QMFC2
// rereads the inputs, which the loop never writes; an output table that would
// overlap them is refused.
bool nativeExponentialRamp(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc<0x2df210 || pc>0x2df384 || (pc&3))return false;
    static constexpr uint32_t loads[]={0x2df210,0x2df214,0x2df228,0x2df22c,0x2df238,0x2df23c,0x2df240,0x2df2b0};
    const bool load=std::find(std::begin(loads),std::end(loads),pc)!=std::end(loads);
    if(!load && pc!=0x2df2b4 && pc<0x2df2c0)return false;   // 2df2b8/2df2bc: EE branch and delay slot
    if(!c)throw std::runtime_error("native exponential ramp has no CPU record");
    const auto stack=c->r[29].uw[0],object=c->r[17].uw[0],coefficients=0x3578b0u;
    if(load){
        const unsigned reg=pc==0x2df210?6:pc==0x2df228?4:pc==0x2df2b0?5:3;
        const uint32_t expected=pc==0x2df210?stack+0x50:pc==0x2df214?stack+0x40:pc==0x2df228?object+0x13470:pc==0x2df22c?stack+0x60:pc==0x2df2b0?stack+0x30:coefficients;
        if(c->r[reg].uw[0]!=expected)throw std::runtime_error("native exponential ramp input source mismatch");return true;
    }
    if(pc!=0x2df384)return true;
    const auto index=c->r[5].uw[0],table=c->r[16].uw[0];if(index>=256)throw std::runtime_error("native exponential ramp index outside its loop");
    const auto overlaps=[](uint32_t a,uint32_t an,uint32_t b,uint32_t bn){return (a&0x1fffffff)<(b&0x1fffffff)+bn && (b&0x1fffffff)<(a&0x1fffffff)+an;};
    if(overlaps(table,0x400,stack+0x30,0x40) || overlaps(table,0x400,object+0x13470,16) || overlaps(table,0x400,coefficients,0x30))
        throw std::runtime_error("native exponential ramp table aliases its inputs");
    const auto vector=[&](uint32_t at){std::array<float,4> v;std::memcpy(v.data(),record(rt,at,16),16);for(float &x:v)x=vu0Input(x);return v;};
    const auto one=vector(stack+0x50),scale=vector(stack+0x40),base=vector(object+0x13470),params=vector(stack+0x30);
    float floor;std::memcpy(&floor,record(rt,stack+0x60,4),4);floor=vu0Input(floor);
    std::array<std::array<float,4>,3> k;for(unsigned r=0;r<3;++r)k[r]=vector(coefficients+16*r);
    uint32_t out[4];
    {music::NearestRounding rounding;
        const auto quotient=[](float a,float b,const char *what){
            if((std::bit_cast<uint32_t>(b)&0x7fffffff)==0)throw std::runtime_error(std::format("native exponential ramp divides by zero ({})",what));
            volatile float q=a/b;const auto v=pairNormal(q);if(!std::isfinite(v))throw std::runtime_error("native exponential ramp quotient overflow");return v;};
        // VU MAX: ordered as signed magnitudes, so -0 < +0.
        const auto maximum=[](float a,float b){const auto ia=std::bit_cast<int32_t>(a),ib=std::bit_cast<int32_t>(b);
            const auto key=[](int32_t v){return v<0?-(v&0x7fffffff)-1:v;};return key(ia)>=key(ib)?a:b;};
        const auto denominator=pairSum(pairProduct(float(index),params[2]),params[1]);
        const auto q=maximum(pairSum(0.f,quotient(params[0],denominator,"step")),floor);
        for(unsigned lane=0;lane<4;++lane){
            const auto x=pairProduct(base[lane],q);
            auto x2=pairProduct(x,x);const auto x4=pairProduct(x2,x2);auto x3=pairProduct(x2,x);auto x5=pairProduct(x4,x);auto x4b=x4;
            auto acc=pairSum(x,1.f);
            for(unsigned r=0;r<3;++r){
                if(r){x2=pairProduct(x2,x4);x3=pairProduct(x3,x4);x4b=pairProduct(x4b,x4);x5=pairProduct(x5,x4);}
                acc=pairSum(acc,pairProduct(x2,k[r][0]));acc=pairSum(acc,pairProduct(x3,k[r][1]));
                acc=pairSum(acc,pairProduct(x4b,k[r][2]));acc=pairSum(acc,pairProduct(x5,k[r][3]));
            }
            if(!std::isfinite(acc))throw std::runtime_error("native exponential ramp series overflow");
            const auto reciprocal=quotient(1.f,acc,"series");
            // ADDq.x/y/z write 0+Q, MULq.w writes 1*Q; SUB.xyz then takes vf23 - vf12.
            const auto value=lane<3?pairSum(one[lane],-pairSum(0.f,reciprocal)):pairProduct(1.f,reciprocal);
            auto scaled=pairProduct(scale[lane],value);if(!std::isfinite(scaled))throw std::runtime_error("native exponential ramp output overflow");
            if(scaled>scale[0])scaled=scale[0];
            // FTOI0: truncate toward zero, saturating.
            out[lane]=scaled>=2147483648.f?0x7fffffffu:scaled<=-2147483648.f?0x80000000u:uint32_t(int32_t(scaled));
        }
    }
    std::memcpy(c->r[4].uw,out,16);return true;
}
// Grid point, mod001 2e1a7c (block 2e1a7c). Before the loop 2e2190..2e2d90, LQC2
// loads vf1 = SP+200, vf2 = SP+1f0, vf4 = s5+188f0 (2e215c..2e2164). Each pass:
// LQC2 vf3 = SP+1e0 (2e2500), ACC = vf4 + vf0, ACC += vf1*vf3.x, vf10 = ACC +
// vf2*vf3.y, SQC2 vf10 to SP+210 (2e2514). The loop writes none of the three
// records, keeps s5/sp and calls no VU0 code (2f3a60, 2f3900, 2f39c0, 2c6b40,
// 2f3b00), so the native store rereads them; overlapping records are refused.
bool nativeGridPoint(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool rows=pc==0x2e215c || pc==0x2e2160 || pc==0x2e2164,body=pc==0x2e2500 || pc==0x2e2504 || pc==0x2e2508 || pc==0x2e250c;
    if(!rows && !body && pc!=0x2e2514)return false;
    if(!c)throw std::runtime_error("native grid point has no CPU record");
    const auto stack=c->r[29].uw[0],field=c->r[21].uw[0]+0x188f0;
    if(rows){const unsigned reg=pc==0x2e215c?10:pc==0x2e2160?9:8;const uint32_t expected=pc==0x2e215c?stack+0x200:pc==0x2e2160?stack+0x1f0:field;
        if(c->r[reg].uw[0]!=expected)throw std::runtime_error("native grid point basis source mismatch");return true;}
    if(body){if(pc==0x2e2500 && c->r[2].uw[0]!=stack+0x1e0)throw std::runtime_error("native grid point source mismatch");return true;}
    if(c->r[2].uw[0]!=stack+0x210)throw std::runtime_error("native grid point destination mismatch");
    const auto overlaps=[](uint32_t a,uint32_t an,uint32_t b,uint32_t bn){return (a&0x1fffffff)<(b&0x1fffffff)+bn && (b&0x1fffffff)<(a&0x1fffffff)+an;};
    if(overlaps(field,16,stack+0x1e0,0x40))throw std::runtime_error("native grid point origin aliases its frame");
    const auto vector=[&](uint32_t at,unsigned lanes){std::array<float,4> v{};std::memcpy(v.data(),record(rt,at,16),16);for(unsigned i=0;i<lanes;++i)v[i]=vu0Input(v[i]);return v;};
    const auto u=vector(stack+0x200,4),v=vector(stack+0x1f0,4),origin=vector(field,4),point=vector(stack+0x1e0,2);
    std::array<float,4> out;
    {music::NearestRounding rounding;
        for(unsigned i=0;i<4;++i){
            auto acc=pairSum(origin[i],i==3?1.f:0.f);acc=pairSum(acc,pairProduct(u[i],point[0]));out[i]=pairSum(acc,pairProduct(v[i],point[1]));
            if(!std::isfinite(out[i]))throw std::runtime_error("native grid point overflow");
        }
    }
    std::memcpy(record(rt,stack+0x210,16),out.data(),16);return true;
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
    // PS2 words (see Ps2Word): no magnitude gate; Q = RSQRT(1, |d|^2) as one VU op.
    const auto *A=record(rt,segment,16),*D=record(rt,segment+0x10,16),*P=record(rt,sphere,16);
    const Ps2Word t{c->r[2].uw[0]};Ps2Word delta[3],squared[3];
    for(unsigned i=0;i<3;++i){delta[i]=((kPs2Zero+Ps2Word::load(A+4*i))+Ps2Word::load(D+4*i)*t)-Ps2Word::load(P+4*i)*kPs2One;squared[i]=delta[i]*delta[i];}
    const auto q=Ps2Word::reciprocalRoot(kPs2One,(squared[0]+squared[1])+kPs2One*squared[2]);
    uint8_t normal[12];for(unsigned i=0;i<3;++i)(delta[i]*q).store(normal+4*i);
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
void morphMesh(Runtime *rt,uint32_t element,unsigned vertices,uint8_t *positions,uint8_t *normals,uint8_t *packed);
bool overlayCode(Runtime *rt,uint32_t lo,uint32_t hi,uint64_t expected);
// EE FPU (COP1) operations on raw words, as the recompiled code runs them (ps2f: add/sub
// truncate after one guard bit; mul, div, sqrt round to nearest even).
uint32_t eeAdd(uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::add(a,b,f);}
uint32_t eeSub(uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::sub(a,b,f);}
uint32_t eeMul(uint32_t a,uint32_t b){uint32_t f=0;return bozo::ps2f::mul(a,b,f);}
int16_t uvHalf(Runtime *rt,uint32_t at){if(at&1)throw std::runtime_error("native halfword misaligned");return int16_t(uvWord(rt,at&~3u)>>((at&2)*8));}
void uvStore(Runtime *rt,uint32_t at,uint32_t v){uvWord(rt,at);std::memcpy(rt->memory.hostPointer(at,4),&v,4);}
uint8_t uvByte(Runtime *rt,uint32_t at){return uint8_t(uvWord(rt,at&~3u)>>((at&3)*8));}
// Battle ground height, mod028 4ab140(p, 0, 0) as the ribbon element calls it (EE FPU):
//   0 when bit 7 of *(*36d880+53) is clear; else 10000 unless p.x/p.z lie strictly inside
//   the bounds at 513318 (x lo, z lo, x hi, z hi) and inside a tile of the list
//   *(*(*36d880+68)+f0) (+118 count, tiles at +d8): ints x0 +270, z0 +274, x1 +278, z1 +27c.
//   Cell col = cvt.w(*(+288) * (x - x0)), row = cvt.w(*(+28c) * (z - z0)); s16 heights at
//   *(+250) + 20 + 2*(col + row * *(+268)): h00, h01 = +2, h10 = +stride, h11 = both; 7fff
//   or more is a hole, filled from its neighbours in a fixed order (an unfilled h01 skips
//   the tile). Bilinear over the cell (sizes *(+260), *(+264)), times *(+284), plus *(+280).
// From the second tile on, the choice reads p.y, which the ribbon's frame never writes
// (sp+e4): several tiles are refused until the user defines that value.
uint32_t battleGroundHeight(Runtime *rt,uint32_t x,uint32_t z){
    namespace F=bozo::ps2f;constexpr uint32_t none=0x461c4000u;   // 10000.0
    const auto control=uvWord(rt,0x36d880);
    if(!(uvByte(rt,control+0x53)&0x80))return 0;
    if(!F::lt(uvWord(rt,0x513318),x) || !F::lt(uvWord(rt,0x51331c),z) || F::le(uvWord(rt,0x513320),x) || F::le(uvWord(rt,0x513324),z))return none;
    const auto list=uvWord(rt,uvWord(rt,control+0x68)+0xf0);const auto tiles=int32_t(uvWord(rt,list+0x118));
    if(tiles>1)throw std::runtime_error(std::format("native battle ground height over {} tiles reads the ribbon's never-written y (ask the user)",tiles));
    uint32_t best=none,offset=0;
    for(int32_t t=0;t<tiles;++t){
        const auto tile=uvWord(rt,list+0xd8+4*uint32_t(t));
        const auto x0=F::fromInt(uvWord(rt,tile+0x270)),x1=F::fromInt(uvWord(rt,tile+0x278)),z0=F::fromInt(uvWord(rt,tile+0x274)),z1=F::fromInt(uvWord(rt,tile+0x27c));
        if(F::lt(x,x0) || !F::lt(x,x1) || F::lt(z,z0) || !F::lt(z,z1))continue;
        const auto sx=uvWord(rt,tile+0x260),sz=uvWord(rt,tile+0x264),stride=uvWord(rt,tile+0x268),grid=uvWord(rt,tile+0x250);
        const auto row=F::toInt(eeMul(uvWord(rt,tile+0x28c),eeSub(z,z0))),col=F::toInt(eeMul(uvWord(rt,tile+0x288),eeSub(x,x0)));
        const uint32_t at=(col+row*stride)*2+grid;
        int32_t h00=uvHalf(rt,at+0x20),h01=uvHalf(rt,at+0x22),h11=uvHalf(rt,at+0x22+2*stride),h10=uvHalf(rt,at+0x20+2*stride);
        bool v00=h00<0x7fff,v01=h01<0x7fff,v10=h10<0x7fff;const bool v11=h11<0x7fff;
        if(!v00){if(v01){h00=h01;v00=true;}else if(v10){h00=h10;v00=true;}}
        if(!v01){if(v00){h01=h00;v01=true;}else if(v11)h01=h11;}
        if(!v10){if(v00){h10=h00;v10=true;}else if(v11)h10=h11;}
        if(!v11){if(v01)h11=h01;else if(v10)h11=h10;}
        if(!v01)continue;
        const uint32_t ox=F::toInt(x0)+col*sx,oz=F::toInt(z0)+row*sz;
        const auto fx=eeSub(x,F::fromInt(ox)),rx=eeSub(F::fromInt(sx),fx),fz=eeSub(z,F::fromInt(oz)),rz=eeSub(F::fromInt(sz),fz);
        const auto a=eeMul(eeMul(fx,fz),F::fromInt(uint32_t(h11))),b=eeMul(eeMul(rx,fz),F::fromInt(uint32_t(h10)));
        const auto c=eeMul(eeMul(rx,rz),F::fromInt(uint32_t(h00))),d=eeMul(eeMul(fx,rz),F::fromInt(uint32_t(h01)));
        best=eeMul(eeAdd(eeAdd(c,d),eeAdd(b,a)),uvWord(rt,tile+0x284));offset=uvWord(rt,tile+0x280);   // the first tile always takes it
    }
    return F::eq(none,best)?best:eeAdd(best,offset);
}
// Battle ribbon element (class 172b10), its +10 method mod028 498940(e, mesh record), EE FPU.
// The mesh's vertices come in quads of 6 (two triangles per segment); segment s uses the
// element's points at e+70+30*i (i = e+254, then +1 per segment; e+254 keeps the end):
//   i >= e+250: the segment's 6 words of the flag-4 stream are zeroed, positions left;
//   else p0 = +0, p1 = +10 (x, z), width w = +20: d = p1 - p0 (xz), len = sqrt(dx^2 + dz^2)
//   (MULA/MADD), d /= len (d = e+50/e+58 when len is 0); a = len * (0.5 * e+68),
//   b = 1.5 * (w * e+64); when a < b: back = b - a, a = b; centre c = p0 - d*back - e+40;
//   the vertices (x, z) are c + d*a +- (-dz.. as written below) and only x, z are stored,
//   or, with byte e+258 set, also y = height(e+40 + v) - e+40.y - 0.2 (e+40.y - 0.2 when
//   that difference is exactly 10000). The xz bounds at *(e+60) (min x, max x, min z, max z)
//   grow over every written vertex.
// The battle task colour pairs (class 171f20; see nativeBattleTaskColours): what each task's
// 47a9b0 loaded into vf21/vf22, kept per task (user decision 2026-11-21).
uint32_t vuMul(uint32_t a,uint32_t b);uint32_t vuAdd(uint32_t a,uint32_t b);uint32_t vuSub(uint32_t a,uint32_t b);
struct BattleTaskColours{std::array<uint32_t,4> first{},second{};};
std::map<std::pair<const Runtime *,uint32_t>,BattleTaskColours> &battleTaskColours(){static std::map<std::pair<const Runtime *,uint32_t>,BattleTaskColours> m;return m;}
// Battle colour element (class 171f20, +10 = mod028 47a910(element, mesh record)): colours
// the 4-byte vertex stream (+1e) two vertices at a time. Per pair, from the cursor +34 (one
// float each) and the phase +40 (EE FPU): t = phase + entry; when not t <= 1, t -= 2;
// t = |t|; u = 1 - t. VU0: ACC = vf21 t; vf10 = ACC + vf22 u (xyzw); FTOI0; PPACH/PPACB keep
// the low byte of each lane (r, g, b, a), stored for both vertices of the pair. The cursor
// is written back as its final value - 4 (47a9a0..47a9a8, as the game does).
void taskColourMesh(Runtime *rt,uint32_t element,unsigned vertices,uint8_t *stream,size_t streamBytes){
    namespace F=bozo::ps2f;
    const auto pair=battleTaskColours().find({rt,element});
    if(pair==battleTaskColours().end())throw std::runtime_error(std::format("native battle colour element {:x} drawn without the colour pair its 47a9b0 loads",element));
    if(4ull*((vertices+1)&~1u)>streamBytes)throw std::runtime_error("native battle colour element outside its vertex stream");
    constexpr uint32_t one=0x3f800000u,two=0x40000000u;
    const auto phase=uvWord(rt,element+0x40);auto cursor=uvWord(rt,element+0x34);
    for(unsigned i=0;i<vertices;i+=2,cursor+=4){
        auto t=eeAdd(phase,uvWord(rt,cursor));if(!F::le(t,one))t=eeSub(t,two);t&=0x7fffffffu;const auto u=eeSub(one,t);
        uint8_t rgba[4];for(unsigned c=0;c<4;++c)rgba[c]=uint8_t(vuFtoi(vuAdd(vuMul(pair->second.first[c],t),vuMul(pair->second.second[c],u)),0));
        std::memcpy(stream+4*i,rgba,4);std::memcpy(stream+4*i+4,rgba,4);}
    uvStore(rt,element+0x34,cursor-4);
}
// Battle flash element (class 172080): its slot +1c, mod028 47cdc0, resets the cursor +3c and
// ends with QMTC2 t0 = 255.0f -> vf23, MULx.w vf23.w = vf0.w * vf23.x (47cdd8, 47cddc), then
// returns; only its element draw +10, 47ccf0, reads vf23.w, in a later call. Same rule as the
// colour pairs (user decision 2026-11-21): each task keeps the value its own 47cdc0 loaded.
std::map<std::pair<const Runtime *,uint32_t>,uint32_t> &battleTaskFlashes(){static std::map<std::pair<const Runtime *,uint32_t>,uint32_t> m;return m;}
// 47ccf0(element, mesh record): per group of 4 vertices of the 4-byte stream, a 24-bit index
// from 3 bytes at +34 + cursor (+3c, then + 3), the factor k = byte (+38)[index] / 128 (EE FPU),
// and the group's first colour c (bytes widened, ITOF0): vf21 = (c.rgb, vf23.w - c.a);
// vf20 = c * 1 + vf21 k (MULAw / MADDx, xyzw); FTOI0; PPACH/PPACB keep each lane's low byte,
// stored for all four vertices. The cursor is written back.
void taskFlashMesh(Runtime *rt,uint32_t element,unsigned vertices,uint8_t *stream,size_t streamBytes){
    const auto flash=battleTaskFlashes().find({rt,element});
    if(flash==battleTaskFlashes().end())throw std::runtime_error(std::format("native battle flash element {:x} drawn without the value its 47cdc0 loads",element));
    const unsigned groups=(vertices+3)/4;if(16ull*groups>streamBytes)throw std::runtime_error("native battle flash element outside its vertex stream");
    constexpr uint32_t one=0x3f800000u,eighth=0x3c000000u;
    auto cursor=uvWord(rt,element+0x3c);const auto bytes=uvWord(rt,element+0x34),table=uvWord(rt,element+0x38);
    for(unsigned g=0;g<groups;++g,cursor+=3){
        const auto at=bytes+cursor;const uint32_t index=uvByte(rt,at)+(uint32_t(uvByte(rt,at+1))<<8)+(uint32_t(uvByte(rt,at+2))<<16);
        uint32_t f=0;const auto k=bozo::ps2f::mul(eighth,bozo::ps2f::fromInt(int32_t(uvByte(rt,table+index))),f);
        uint8_t *group=stream+16*g;std::array<uint32_t,4> c{},v21{};
        for(unsigned i=0;i<4;++i)c[i]=std::bit_cast<uint32_t>(float(group[i]));
        v21={c[0],c[1],c[2],vuSub(flash->second,c[3])};
        uint8_t rgba[4];for(unsigned i=0;i<4;++i)rgba[i]=uint8_t(vuFtoi(vuAdd(vuMul(c[i],one),vuMul(v21[i],k)),0));
        for(unsigned v=0;v<4;++v)std::memcpy(group+4*v,rgba,4);}
    uvStore(rt,element+0x3c,cursor);
}
// Battle trail element (class 1730d0), its +10 method mod028 4a83e0(e, mesh record): two rings of
// 60 qwords (+40 and +400, newest at +7c0 minus the count +808) feed the positions stream (record +4)
// alternately. With the toggle (+80c) set a vertex copies ring A at the current entry; otherwise it
// copies ring B there, then the entry steps back one (wrapping from 0 to 59) and the count grows.
// The count is stored back minus one and the toggle as it ends (EE code, whole 128-bit copies).
void trailMesh(Runtime *rt,uint32_t element,unsigned vertices,uint8_t *positions){
    auto count=uvWord(rt,element+0x808);const auto current=int32_t(uvWord(rt,element+0x7c0));bool toggle=uvByte(rt,element+0x80c)!=0;
    int32_t entry=current-int32_t(count);if(entry<0)entry+=60;
    if(entry<0 || entry>59)throw std::runtime_error("native battle trail entry outside its rings");
    for(unsigned i=0;i<vertices;++i){
        const auto from=element+0x10*uint32_t(entry)+(toggle?0x40u:0x400u);std::memcpy(positions+16*i,uvBytes(rt,from,16),16);
        if(!toggle){if(--entry<0)entry=59;++count;}
        toggle=!toggle;}
    uvStore(rt,element+0x808,count-1);
    const auto at=element+0x80c;auto word=uvWord(rt,at&~3u);const unsigned shift=(at&3)*8;word=(word&~(0xffu<<shift))|(uint32_t(toggle)<<shift);uvStore(rt,at&~3u,word);
}
void ribbonMesh(Runtime *rt,uint32_t element,unsigned vertices,uint8_t *positions,uint8_t *stream){
    namespace F=bozo::ps2f;
    if(vertices%6)throw std::runtime_error("battle ribbon mesh is not made of 6-vertex segments");
    const auto origin=[&]{std::array<uint32_t,3> o;for(unsigned i=0;i<3;++i)o[i]=uvWord(rt,element+0x40+4*i);return o;}();
    const auto boundsAt=uvWord(rt,element+0x60),widthScale=uvWord(rt,element+0x64),lengthScale=uvWord(rt,element+0x68);
    const bool ground=uvByte(rt,element+0x258)!=0;const auto limit=int32_t(uvWord(rt,element+0x250));auto index=int32_t(uvWord(rt,element+0x254));
    uint32_t bounds[4];for(unsigned i=0;i<4;++i)bounds[i]=uvWord(rt,boundsAt+4*i);
    constexpr uint32_t half=0x3f000000u,threeHalves=0x3fc00000u,one=0x3f800000u,lift=0x3e4ccccdu,none=0x461c4000u;
    const auto put=[](uint8_t *at,uint32_t v){std::memcpy(at,&v,4);};
    for(unsigned s=0;s<vertices/6;++s,++index){
        if(index>=limit){std::memset(stream+0x18*s,0,0x18);continue;}
        if(index<0)throw std::runtime_error("battle ribbon point index negative");
        const auto p=element+0x70+0x30*uint32_t(index);
        const auto x0=uvWord(rt,p),z0=uvWord(rt,p+8);
        auto dx=eeSub(uvWord(rt,p+0x10),x0),dz=eeSub(uvWord(rt,p+0x18),z0);
        uint32_t len;{uint32_t f=0;len=F::sqrt(eeAdd(eeMul(dx,dx),eeMul(dz,dz)),f);}
        if(F::eq(0,len)){dx=uvWord(rt,element+0x50);dz=uvWord(rt,element+0x58);}
        else{uint32_t f=0;const auto q=F::div(one,len,f);dx=eeMul(dx,q);dz=eeMul(dz,q);}
        auto a=eeMul(len,eeMul(half,lengthScale));const auto b=eeMul(threeHalves,eeMul(uvWord(rt,p+0x20),widthScale));uint32_t back=0;
        if(F::lt(a,b)){back=eeSub(b,a);a=b;}
        const auto f9=eeMul(dx,b),f8=eeMul(dz,b),f7=eeMul(dx,a),f0=eeMul(dz,a);
        const auto cx=eeSub(eeSub(x0,eeMul(dx,back)),origin[0]),cz=eeSub(eeSub(z0,eeMul(dz,back)),origin[2]);
        auto f6=eeAdd(f7,cx),f5=eeAdd(f0,cz);const auto f4=eeAdd(f6,f8),f3=eeSub(f5,f9);f6=eeSub(f6,f8);f5=eeAdd(f5,f9);
        const uint32_t xz[6][2]={{eeSub(f6,f7),eeSub(f5,f0)},{eeSub(f4,f7),eeSub(f3,f0)},{f6,f5},{f4,f3},{eeAdd(f6,f7),eeAdd(f5,f0)},{eeAdd(f4,f7),eeAdd(f3,f0)}};
        for(unsigned v=0;v<6;++v){
            auto *out=positions+16*(6*s+v);const auto x=xz[v][0],z=xz[v][1];
            if(ground){const auto rise=eeSub(battleGroundHeight(rt,eeAdd(origin[0],x),eeAdd(origin[2],z)),origin[1]);
                put(out+4,F::eq(none,rise)?eeSub(origin[1],lift):eeSub(rise,lift));}
            put(out,x);put(out+8,z);
            bounds[0]=F::min(bounds[0],x);bounds[1]=F::max(bounds[1],x);bounds[2]=F::min(bounds[2],z);bounds[3]=F::max(bounds[3],z);
        }
    }
    for(unsigned i=0;i<4;++i)uvStore(rt,boundsAt+4*i,bounds[i]);
    uvStore(rt,element+0x254,uint32_t(index));
}
bool nativeUvAnimation(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x219210)return false;
    if(!c)throw std::runtime_error("native UV animation has no CPU record");
    const auto object=c->r[23].uw[0];
    const auto halfAt=object+0x84;const int16_t chunks=int16_t(uvWord(rt,halfAt&~3u)>>((halfAt&2)*8));
    if(chunks>0){
        const auto flags=uvWord(rt,object+0x58);
        // 1 positions, 2 normals, 4 the 4-byte vertex stream at +1e, 8 UVs, 80 packed
        // vectors, 20 element data; 40 is only part of the skeleton's flag word (e3).
        // 10 and 100 reject.
        if(flags&~0x1efu){
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
        struct Element{uint32_t at,cursor;float scroll[2];bool all;uint32_t bits;bool skeleton;bool morph=false;bool ribbon=false;bool colour=false;bool flash=false;bool trail=false;};std::vector<Element> elements;
        for(unsigned i=0;i<count;++i){const auto e=uvWord(rt,list+4*i);const auto vt=uvWord(rt,e);
            if(vt==0x154690){
                if(uvWord(rt,vt+0x10)!=0x218220 || uvWord(rt,vt+0x18)!=0x2372f0)throw std::runtime_error("unqualified skeleton element methods");
                if((flags&3)!=3)throw std::runtime_error("skeleton element without streamed positions and normals");
                elements.push_back({e,uvWord(rt,e+0x44),{0,0},(uvWord(rt,e+0x18)>>8&0xff)!=0,uvWord(rt,e+0x24),true});continue;}
            if(vt==0x1536a0){
                // Morph element: +10 is 218600 (programs 0x7d8/0x848), reading
                // its own RAM table at the +38 cursor.
                if(uvWord(rt,vt+0x10)!=0x218600)throw std::runtime_error("unqualified morph element methods");
                if((flags&3)!=3)throw std::runtime_error("morph element without streamed positions and normals");
                elements.push_back({e,uvWord(rt,e+0x38),{0,0},(uvWord(rt,e+0x18)>>8&0xff)!=0,uvWord(rt,e+0x24),true});elements.back().morph=true;continue;}
            if(vt==0x172b10){
                // Battle ribbon element: +10 is mod028 498940 (ground height 4ab140), see ribbonMesh.
                if(uvWord(rt,vt+0x10)!=0x498940 || !overlayCode(rt,0x498940,0x498d58,0x40ff9fc8352959adull) || !overlayCode(rt,0x4ab140,0x4ab5e4,0xcc9361cbd4fb640dull))
                    throw std::runtime_error("unqualified battle ribbon element methods");
                if((flags&5)!=5)throw std::runtime_error("battle ribbon element without streamed positions and its vertex stream");
                elements.push_back({e,0,{0,0},(uvWord(rt,e+0x18)>>8&0xff)!=0,uvWord(rt,e+0x24),true});elements.back().ribbon=true;continue;}
            if(vt==0x172080){
                // Battle flash element: +10 is mod028 47ccf0 (see taskFlashMesh).
                if(uvWord(rt,vt+0x10)!=0x47ccf0 || !overlayCode(rt,0x47ccf0,0x47cdc0,0x4ccd27596acefa72ull))
                    throw std::runtime_error("unqualified battle flash element methods");
                if(!(flags&4))throw std::runtime_error("battle flash element without its vertex stream");
                elements.push_back({e,0,{0,0},(uvWord(rt,e+0x18)>>8&0xff)!=0,uvWord(rt,e+0x24),true});elements.back().flash=true;continue;}
            if(vt==0x171f20){
                // Battle colour element: +10 is mod028 47a910 (see taskColourMesh).
                if(uvWord(rt,vt+0x10)!=0x47a910 || !overlayCode(rt,0x47a910,0x47a9b0,0x3a7920c6b9ab95d9ull))
                    throw std::runtime_error("unqualified battle colour element methods");
                if(!(flags&4))throw std::runtime_error("battle colour element without its vertex stream");
                elements.push_back({e,0,{0,0},(uvWord(rt,e+0x18)>>8&0xff)!=0,uvWord(rt,e+0x24),true});elements.back().colour=true;continue;}
            if(vt==0x1730d0){
                // Battle trail element: +10 is mod028 4a83e0 (see trailMesh).
                if(uvWord(rt,vt+0x10)!=0x4a83e0 || !overlayCode(rt,0x4a83e0,0x4a84bc,0x73a14087028e5f1cull))
                    throw std::runtime_error("unqualified battle trail element methods");
                if(!(flags&1))throw std::runtime_error("battle trail element without streamed positions");
                elements.push_back({e,0,{0,0},(uvWord(rt,e+0x18)>>8&0xff)!=0,uvWord(rt,e+0x24),true});elements.back().trail=true;continue;}
            if(vt!=0x1591b0)throw std::runtime_error(std::format("unported animated vertex element class {:x} (+10={:x})",vt,uvWord(rt,vt+0x10)));
            Element el{e,uvWord(rt,e+0x34),{std::bit_cast<float>(uvWord(rt,e+0x40)),std::bit_cast<float>(uvWord(rt,e+0x44))},(uvWord(rt,e+0x18)>>8&0xff)!=0,uvWord(rt,e+0x24),false};
            for(float s:el.scroll)if((std::bit_cast<uint32_t>(s)&0x7f800000)==0x7f800000)throw std::runtime_error("unqualified UV scroll");
            elements.push_back(el);}
        // The loader numbers element data by the object's element chain
        // (+d or +18 set); the +10 callbacks index it by this list. Elements
        // that read that data (skeleton, UV scroll) must sit at the same index
        // in both; the morph element reads its own RAM table.
        {std::vector<uint32_t> chain;for(auto e=uvWord(rt,object+8);e!=object;e=uvWord(rt,e+8)){
            const auto bytes=uvWord(rt,e+0xc),flagWord=uvWord(rt,e+0x18);
            if(!((bytes>>8)&0xff) && !(flagWord&0xff))continue;
            chain.push_back(e);if(chain.size()>64)throw std::runtime_error("animated element chain outside bounds");}
         for(size_t i=0;i<elements.size();++i){if(elements[i].morph || elements[i].ribbon || elements[i].colour || elements[i].flash || elements[i].trail)continue;
            if(i>=chain.size() || chain[i]!=elements[i].at)throw std::runtime_error("animated element data index differs from the object's chain");}}
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
                // Bit 100 streams the same +34 data for every mode (8/18/32/24
                // bytes per vertex); only mode 0 is the s16x4 layout ported here.
                if((flags&0x100) && packedHalf && (packedHalf>>14))throw std::runtime_error(std::format("unported +34 stream mode {}",packedHalf>>14));
                const bool hasPacked=(flags&0x180) && packedHalf && !(packedHalf>>14);const uint32_t packedAt=packedHalf&0x3fff,packedBytes=((uint32_t(n)+1)>>1)*16;
                std::vector<uint8_t> positions(flags&1?16*n:0),normals(flags&2?16*n:0),packedData(hasPacked?packedBytes:0);
                // Flag 4: 4 bytes per vertex at u16 +1e, (N+3)/4 qwords.
                const uint32_t streamAt=uvWord(rt,source+0x1c)>>16,streamBytes=flags&4?((uint32_t(n)+3)>>2)*16:0;std::vector<uint8_t> stream(streamBytes);
                if(n && streamBytes)std::memcpy(stream.data(),uvBytes(rt,source+streamAt,streamBytes),streamBytes);
                if(n && (flags&1))std::memcpy(positions.data(),uvBytes(rt,source+positionAt,16*n),16*n);
                if(n && (flags&2))std::memcpy(normals.data(),uvBytes(rt,source+normalAt,16*n),16*n);
                if(n && hasPacked)std::memcpy(packedData.data(),uvBytes(rt,source+packedAt,packedBytes),packedBytes);
                for(auto &el:elements){
                    if(el.flash){if(!n || (!el.all && !(uvWord(rt,el.bits+4*(uint32_t(j)>>5))>>(j&31)&1)))continue;
                        taskFlashMesh(rt,el.at,unsigned(n),stream.data(),stream.size());continue;}
                    if(el.colour){if(!n || (!el.all && !(uvWord(rt,el.bits+4*(uint32_t(j)>>5))>>(j&31)&1)))continue;
                        taskColourMesh(rt,el.at,unsigned(n),stream.data(),stream.size());continue;}
                    if(el.trail){if(!n || (!el.all && !(uvWord(rt,el.bits+4*(uint32_t(j)>>5))>>(j&31)&1)))continue;
                        trailMesh(rt,el.at,unsigned(n),positions.data());continue;}
                    if(el.ribbon){if(!n || (!el.all && !(uvWord(rt,el.bits+4*(uint32_t(j)>>5))>>(j&31)&1)))continue;
                        ribbonMesh(rt,el.at,unsigned(n),positions.data(),stream.data());continue;}
                    if(el.morph){if(!n || (!el.all && !(uvWord(rt,el.bits+4*(uint32_t(j)>>5))>>(j&31)&1)))continue;
                        morphMesh(rt,el.at,unsigned(n),positions.data(),normals.data(),hasPacked?packedData.data():nullptr);continue;}
                    if(!el.skeleton)continue;
                    const auto table=el.cursor;if(table&1)throw std::runtime_error("skeleton element table misaligned");el.cursor+=uvWord(rt,table&~3u)>>((table&2)*8)&0xffff;
                    if(!n || (!el.all && !(uvWord(rt,el.bits+4*(uint32_t(j)>>5))>>(j&31)&1)))continue;
                    skinMesh(rt,table,unsigned(n),positions.data(),normals.data(),hasPacked?packedData.data():nullptr);
                }
                if(n && (flags&1))std::memcpy(uvBytes(rt,packet+positionAt,16*n),positions.data(),16*n);
                if(n && (flags&2))std::memcpy(uvBytes(rt,packet+normalAt,16*n),normals.data(),16*n);
                if(n && streamBytes)std::memcpy(uvBytes(rt,packet+streamAt,streamBytes),stream.data(),streamBytes);
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
    {0x3cdc84,19,false,~0u,2,0xa0,0},          // mod003 3cdb60: argument record (s3)
    {0x42c6c8,5,true,0x60,4,0x30,0}};          // mod003 42c600 distance (SP+60 less s0)
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
// The same length block is inlined about 110 times (mod003, mod001): LQC2
// vf24, MUL.xyz vf24*vf24, ADDw.xyz vf25 = 1, ADDAy, MADDz, SQRT, WAITQ,
// ADDq.x, SQC2 vf24. Recognised by its nine exact words in the loaded code
// around the PC; source and destination come from the LQC2/SQC2 encodings.
// Publishes {|xyz|, y*y, z*z, w} like the table sites above.
bool nativeLengthIdiom(R5900Context *c,Runtime *rt,uint32_t pc){
    if(!rt || (pc&3))return false;
    static constexpr uint32_t body[7]={0x4bd8c62a,0x4bc00643,0x4b18c03d,0x4b18ce0a,0x4a1803bd,0x4a0003bf,0x4b000620};
    uint32_t start=0,words[9];bool found=false;
    for(unsigned k=0;k<9 && !found;++k){
        const auto at=pc-4*k;const auto *code=rt->memory.hostPointer(at,36);if(!code)continue;std::memcpy(words,code,36);
        found=words[0]>>26==0x36 && ((words[0]>>16)&31)==24 && words[8]>>26==0x3e && ((words[8]>>16)&31)==24 && !std::memcmp(words+1,body,sizeof(body));
        if(found)start=at;}
    if(!found)return false;
    if(!c)throw std::runtime_error("native length idiom has no CPU record");
    if(pc!=start+0x20)return true;
    const auto address=[&](uint32_t w){return c->r[(w>>21)&31].uw[0]+uint32_t(int32_t(int16_t(w&0xffff)));};
    const auto sourceAt=address(words[0]),destinationAt=address(words[8]);
    float value[4],result[4];std::memcpy(value,record(rt,sourceAt,16),16);std::memcpy(result,value,16);
    for(unsigned i=0;i<3;++i){if((std::bit_cast<uint32_t>(value[i])&0x7f800000)==0x7f800000 || beyondRange(value[i]))
        throw std::runtime_error(std::format("unqualified native vector length at {:x}",start));value[i]=pairNormal(value[i]);}
    {music::NearestRounding rounding;
        for(unsigned i=0;i<3;++i){volatile float square=value[i]*value[i];result[i]=pairNormal(square);}
        volatile float partial=result[0]+result[1];volatile float squared=pairNormal(partial)+result[2];
        volatile float length=std::sqrt(pairNormal(squared));volatile float output=0.f+length;result[0]=output;}
    std::memcpy(record(rt,destinationAt,16),result,16);return true;
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
// Battle point distance blocks (mod028): vf24 = A - B (xyz), squared; vf24.x = (x+y) + 1*z;
// Q = SQRT; vf24.x = 0 + Q; SQC2 vf24 to OUT: (length, dy^2, dz^2, A.w). ps2f, nearest.
// A length block (b = 0) has no B: vf24 = A, the same steps after it.
//   3fd3f8 (in 3fd198): A = a1, B = v1 (s0+50 + 16*a2, or 0 when a2 is outside s0's +40
//                       entries: address 0 is refused), OUT = a0.
//   382154 (in 38204c): A = t4, B = t2, OUT = t3.
//   37a820 (in 37a7d8): A = a0 (s0+1f0), B = v0 (s0+200), OUT = v1 (sp+60).
//   38256c, 3825a8 (in 382470): A = a2, B = v1, OUT = a1; 3826d0: length of t1 to t0;
//   382798: length of t1 to a3; 3d7b34 (in 3d76a0): A = a0, B = v0, OUT = v1.
//   381b84 (in 381ae0, state 1): A = a3 (a0+30), B = v1 (a0+200), OUT = a2 (sp+0).
struct DistanceSite{uint32_t lo;unsigned a,b,out;};
constexpr DistanceSite distanceSites[]={{0x3fd3f8,5,3,4},{0x382154,12,10,11},{0x37a820,4,2,3},{0x38256c,6,3,5},{0x3825a8,6,3,5},{0x3826d0,9,0,8},{0x382798,9,0,7},{0x3d7b34,4,2,3},{0x381b84,7,3,6},
    // The same block elsewhere in mod028 (found by its words; served when RAM holds them):
    {0x37abf4,4,2,3},{0x37c224,5,3,4},{0x381274,7,3,6},{0x381990,5,3,4},{0x395e74,3,4,2},{0x3960bc,4,5,3},{0x39c28c,5,6,2},{0x3a2f60,5,3,4},{0x3c0810,5,3,4},{0x3c0924,22,3,2},{0x3d65ec,7,11,5},{0x41dca0,6,4,5},{0x48ac80,5,6,4},{0x48aee0,6,7,4},{0x49c8fc,18,3,4},{0x49dadc,18,3,4},{0x49db24,4,2,3},{0x4a85d8,3,21,2},{0x502c48,5,2,3},
    // mod068 (a battle overlay; 527cac's caller chain): three copies, A = a0, B = v0, OUT = v1.
    {0x5272e4,4,2,3},{0x527974,4,2,3},{0x527cdc,4,2,3}};
bool nativeBattlePointDistance(R5900Context *c,Runtime *rt,uint32_t pc){
    constexpr uint32_t middle[]={0x4bd9c62c,0x4bd8c62a,0x4bc00643,0x4b18c03d,0x4b18ce0a,0x4a1803bd,0x4a0003bf,0x4b000620};
    if((pc&3) || !rt)return false;
    const auto site=std::find_if(std::begin(distanceSites),std::end(distanceSites),[&](auto &s){return pc>=s.lo && pc<=s.lo+(s.b?40u:32u);});
    if(site==std::end(distanceSites))return false;
    const bool length=!site->b;const unsigned slot=(pc-site->lo)/4,last=length?8:10;
    const uint32_t expected=slot==0?0xd8180000u|(site->a<<21):slot==last?0xf8180000u|(site->out<<21):
        length?middle[slot]:slot==1?0xd8190000u|(site->b<<21):middle[slot-2];
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=expected)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle point distance has no CPU record");
    if(!length && !c->r[site->b].uw[0])throw std::runtime_error("native battle point distance reads address 0 (index outside its table): undefined natively");
    if(slot!=last)return true;
    uint32_t a[4],b[4]{};std::memcpy(a,record(rt,c->r[site->a].uw[0],16),16);if(!length)std::memcpy(b,record(rt,c->r[site->b].uw[0],16),16);
    constexpr uint32_t one=0x3f800000u;uint32_t out[4];
    for(unsigned i=0;i<3;++i){const auto d=length?a[i]:vuSub(a[i],b[i]);out[i]=vuMul(d,d);}
    const auto length2=vuAdd(vuAdd(out[0],out[1]),vuMul(one,out[2]));
    uint32_t flags=0;out[0]=vuAdd(0,bozo::ps2f::sqrt(length2,flags));out[3]=a[3];
    std::memcpy(record(rt,c->r[site->out].uw[0],16),out,16);return true;
}
// mod028 382470's other two VU0 blocks (ps2f, nearest):
//   3827e4..382804: t1.xyz *= 1/sqrt((x^2+y^2) + 1*z^2) (RSQRT), w kept, in place.
//   3829b8..382a10: s0 = rows(t1) . s0 in place (the height query's row-point block).
bool nativeBattleTrackBlocks(R5900Context *c,Runtime *rt,uint32_t pc){
    constexpr std::pair<uint32_t,uint32_t> sites[]={{0x3827e4,0xd9390000},{0x3827e8,0x4bc007c3},{0x3827ec,0x4bd9ceaa},{0x3827f0,0x4b1ad03d},{0x3827f4,0x4b1afe8a},{0x3827f8,0x4a7a03be},{0x3827fc,0x4a0003bf},{0x382800,0x4bc0ce5c},{0x382804,0xf9390000},
        {0x3829b8,0xda0f0000},{0x3829bc,0xd92b0000},{0x3829c0,0xd92c0010},{0x3829c4,0xd92d0020},{0x3829c8,0xd92e0030},{0x3829cc,0x4bc007c3},{0x3829d0,0x4bef5aea},{0x3829d4,0x4bef632a},{0x3829d8,0x4bef6b6a},{0x3829dc,0x4bef73aa},{0x3829e0,0x4b0b583d},{0x3829e4,0x4b0bf8be},{0x3829e8,0x4b0bf94b},{0x3829ec,0x4a8c603c},{0x3829f0,0x4a8cf8be},{0x3829f4,0x4a8cf94b},{0x3829f8,0x4a4d683d},{0x3829fc,0x4a4df8bc},{0x382a00,0x4a4df94b},{0x382a04,0x4a2e703d},{0x382a08,0x4a2e00be},{0x382a0c,0x4a2e0148},{0x382a10,0xfa050000}};
    if(pc<0x3827e4 || pc>0x382a10 || (pc&3) || !rt)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](auto &x){return x.first==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->second)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle track block has no CPU record");
    if(pc==0x382804){
        const auto at=c->r[9].uw[0];uint32_t v[4];std::memcpy(v,record(rt,at,16),16);
        constexpr uint32_t one=0x3f800000u;uint32_t sq[3];for(unsigned i=0;i<3;++i)sq[i]=vuMul(v[i],v[i]);
        const auto length2=vuAdd(vuAdd(sq[0],sq[1]),vuMul(one,sq[2]));uint32_t flags=0;const auto q=bozo::ps2f::rsqrt(one,length2,flags);
        for(unsigned i=0;i<3;++i)v[i]=vuMul(v[i],q);std::memcpy(record(rt,at,16),v,16);return true;
    }
    if(pc==0x382a10){
        const auto point=c->r[16].uw[0];const auto result=rowPoint(rt,c->r[9].uw[0],point);std::memcpy(record(rt,point,16),result.data(),16);return true;
    }
    return true;
}
// Battle segment direction, mod028 4f58f0 (4f5b28..4f5b54): sp+d0 = (s0 - s3).xyz *
// 1/sqrt((x^2+y^2) + 1*z^2) (RSQRT), w = s3.w; the same block as 4f44f0's direction.
bool nativeBattleSegmentDirection(R5900Context *c,Runtime *rt,uint32_t pc){
    constexpr std::pair<uint32_t,uint32_t> sites[]={{0x4f5b28,0xda7a0000},{0x4f5b2c,0xda1b0000},{0x4f5b30,0x4bc007c3},{0x4f5b38,0x4bdadeac},{0x4f5b3c,0x4bdad6ea},{0x4f5b40,0x4b1bd83d},{0x4f5b44,0x4b1bfeca},{0x4f5b48,0x4a7b03be},{0x4f5b4c,0x4a0003bf},{0x4f5b50,0x4bc0d69c},{0x4f5b54,0xf85a0000}};
    if((pc&3) || !rt || pc<0x4f5b28 || pc>0x4f5b54)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](auto &x){return x.first==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->second)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle segment direction has no CPU record");
    if(pc!=0x4f5b54)return true;
    if(c->r[2].uw[0]!=c->r[29].uw[0]+0xd0)throw std::runtime_error("native battle segment direction escaped its frame");
    constexpr uint32_t one=0x3f800000u;
    uint32_t from[4],to[4],d[4],sq[3];std::memcpy(from,record(rt,c->r[19].uw[0],16),16);std::memcpy(to,record(rt,c->r[16].uw[0],16),16);
    for(unsigned i=0;i<3;++i){d[i]=vuSub(to[i],from[i]);sq[i]=vuMul(d[i],d[i]);}
    uint32_t flags=0;const auto q=bozo::ps2f::rsqrt(one,vuAdd(vuAdd(sq[0],sq[1]),vuMul(one,sq[2])),flags);
    for(unsigned i=0;i<3;++i)d[i]=vuMul(d[i],q);d[3]=from[3];std::memcpy(record(rt,c->r[2].uw[0],16),d,16);return true;
}
// Battle approach, mod028 4f44f0 (twice, the second copy 108 later, word for word; ps2f):
//   4f4514..4f4540: sp+80 = (s4 - s1).xyz * 1/sqrt((x^2+y^2) + 1*z^2) (RSQRT), w = s1.w.
//   4f4550..4f456c: s1.xyz = s1*1 + v1 * t0 (QMTC2 vf3 = t0; v1 = sp+80, t0 = 50), in place.
bool nativeBattleApproach(R5900Context *c,Runtime *rt,uint32_t pc){
    constexpr std::pair<uint32_t,uint32_t> sites[]={{0x4f4514,0xda3a0000},{0x4f4518,0xda9b0000},{0x4f451c,0x4bc007c3},{0x4f4524,0x4bdadeac},{0x4f4528,0x4bdad6ea},{0x4f452c,0x4b1bd83d},{0x4f4530,0x4b1bfeca},{0x4f4534,0x4a7b03be},{0x4f4538,0x4a0003bf},{0x4f453c,0x4bc0d69c},{0x4f4540,0xf85a0000},
        {0x4f4550,0xda210000},{0x4f4554,0xd8620000},{0x4f4560,0x48a81800},{0x4f4564,0x4bc009bf},{0x4f4568,0x4bc31048},{0x4f456c,0xfa210000}};
    if((pc&3) || !rt || pc<0x4f4514 || pc>0x4f456c+0x108)return false;
    const uint32_t listed=pc>0x4f456c?pc-0x108:pc;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](auto &x){return x.first==listed;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->second)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle approach has no CPU record");
    constexpr uint32_t one=0x3f800000u;
    if(listed==0x4f4540){
        if(c->r[2].uw[0]!=c->r[29].uw[0]+0x80)throw std::runtime_error("native battle approach direction escaped its frame");
        uint32_t from[4],to[4],d[4],sq[3];std::memcpy(from,record(rt,c->r[17].uw[0],16),16);std::memcpy(to,record(rt,c->r[20].uw[0],16),16);
        for(unsigned i=0;i<3;++i){d[i]=vuSub(to[i],from[i]);sq[i]=vuMul(d[i],d[i]);}
        uint32_t flags=0;const auto q=bozo::ps2f::rsqrt(one,vuAdd(vuAdd(sq[0],sq[1]),vuMul(one,sq[2])),flags);
        for(unsigned i=0;i<3;++i)d[i]=vuMul(d[i],q);d[3]=from[3];std::memcpy(record(rt,c->r[2].uw[0],16),d,16);return true;
    }
    if(listed==0x4f456c){
        uint32_t point[4],step[4];std::memcpy(point,record(rt,c->r[17].uw[0],16),16);std::memcpy(step,record(rt,c->r[3].uw[0],16),16);
        const auto t=c->r[8].uw[0];
        for(unsigned i=0;i<3;++i)point[i]=vuAdd(vuMul(point[i],one),vuMul(step[i],t));
        std::memcpy(record(rt,c->r[17].uw[0],16),point,16);return true;
    }
    return true;
}
// Guarded 4-lane normalise, mod028 (3e8a34 in 3e8548, 491d4c, 4938a0): LQC2 vf25 = L;
// vf31.xyz = 0 + 1 (ADDw); vf26 = vf25 * vf25 (xyzw); ACC.x = x + y; ACC.x += 1*z;
// vf26.x = ACC.x + 1*w; QMFC2 t0 = vf26 (the CPU branches past when its low 64 bits are 0);
// Q = RSQRT(1, vf26.x); vf25 *= Q (xyzw); SQC2 vf25 to S. ps2f, nearest. vf25 is held
// per call frame from the load to the store (no calls in between; the addiu and beqz
// between are compiled). L and S are registers read at the load and the store.
struct NormaliseSite{uint32_t lo;unsigned load,store;};
constexpr NormaliseSite normaliseSites[]={{0x3e8a34,2,2},{0x491d4c,3,2},{0x4938a0,5,4}};
std::map<std::pair<const Runtime *,uint32_t>,std::array<uint32_t,4>> &battleNormalise(){static std::map<std::pair<const Runtime *,uint32_t>,std::array<uint32_t,4>> m;return m;}
bool nativeBattleNormalise(R5900Context *c,Runtime *rt,uint32_t pc){
    constexpr uint32_t body[]={0,0x4bc007c3,0,0x4bf9ceaa,0x4b1ad03d,0x4b1af8be,0x4b1afe8b,0x4828d000,0,0,0x4a7a03be,0x4a0003bf,0x4be0ce5c,0};
    if((pc&3) || !rt)return false;
    const auto site=std::find_if(std::begin(normaliseSites),std::end(normaliseSites),[&](auto &s){return pc>=s.lo && pc<=s.lo+52;});
    if(site==std::end(normaliseSites))return false;
    const unsigned slot=(pc-site->lo)/4;
    const uint32_t expected=slot==0?0xd8190000u|(site->load<<21):slot==13?0xf8190000u|(site->store<<21):body[slot];
    if(!expected)return false;   // addiu, beqz, its delay slot: compiled
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=expected)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle normalise has no CPU record");
    auto &states=battleNormalise();const auto key=std::make_pair(static_cast<const Runtime *>(rt),c->r[29].uw[0]);
    if(slot==0){std::array<uint32_t,4> v;std::memcpy(v.data(),record(rt,c->r[site->load].uw[0],16),16);states[key]=v;return true;}
    if(slot!=7 && slot!=13)return true;
    auto it=states.find(key);if(it==states.end())throw std::runtime_error("native battle normalise has no load");
    auto &v=it->second;constexpr uint32_t one=0x3f800000u;
    uint32_t sq[4];for(unsigned i=0;i<4;++i)sq[i]=vuMul(v[i],v[i]);
    const auto sum=vuAdd(vuAdd(vuAdd(sq[0],sq[1]),vuMul(one,sq[2])),vuMul(one,sq[3]));
    if(slot==7){c->r[8].uw[0]=sum;c->r[8].uw[1]=sq[1];c->r[8].uw[2]=sq[2];c->r[8].uw[3]=sq[3];return true;}
    uint32_t f=0;const auto q=bozo::ps2f::rsqrt(one,sum,f);uint32_t out[4];for(unsigned i=0;i<4;++i)out[i]=vuMul(v[i],q);
    std::memcpy(record(rt,c->r[site->store].uw[0],16),out,16);states.erase(it);return true;
}
// Battle side axes, mod028 3e9470 (two blocks; ps2f, nearest). Block 1 (3e9764..3e97bc):
// LQC2 vf1 = a (sp+170), vf2 = b (sp+180), vf28 = P0 (sp+1a0), vf27 = P (sp+160);
// vf25.xyz = 1 (ADDw vf0); d = P - P0 (xyz); ACC.x = d.x^2 + d.y^2; s = ACC + 1*d.z^2;
// n = d * RSQRT(1, s); OPMULA/OPMSUB: vf10 = a x n, vf11 = b x n; SQC2 vf10, vf11 to v0
// (sp+100, sp+110). Block 2 (3e989c..3e98e0, in the loop after it): LQC2 vf27 = P (sp+e0)
// only: a, b, P0 and vf25 are the registers block 1 left (the loop's 214100 call and
// scalar code do not touch COP2), so they are kept per call frame; SQC2 to v1 (sp+c0/d0).
// The stores write xyz only: their w lanes are VU registers this function never set, and
// it reads back only the y lanes (sp+104/114, sp+c4/d4), so w stays as in memory.
struct BattleSideAxes{VuWords a{},b{},base{},point{};unsigned loaded=0;VuWords first{},second{};bool done=false;};
std::map<std::pair<const Runtime *,uint32_t>,BattleSideAxes> &battleSideAxes(){static std::map<std::pair<const Runtime *,uint32_t>,BattleSideAxes> m;return m;}
// Its sibling, mod028 481260 (per flagged corner of four): LQC2 vf1, vf2 = object +a00 +
// 32 a1 and +10 (4813f4, 4813f8, once); per corner LQC2 vf26 = sp+50 (base, 481434) and
// vf27 = the corner (481438, its y set to sp+54 first); ADDw.xyz vf31 = 1; SUB.xyz vf26 =
// vf27 - vf26; the same squares, RSQRT, MULq and crosses; SQC2 vf10, vf11 to scratchpad
// 3fe0 / 3ff0 (xyz only, like 3e9764's stores; the CPU reads their y).
bool nativeBattleSideAxes(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Site{uint32_t pc,word;char role;};   // a/b axes, o base, p point, 1/2 stores, - folded
    constexpr Site sites[]={{0x3e9764,0xd8410000,'a'},{0x3e9768,0xd8420010,'b'},{0x3e977c,0xd87c0000,'o'},{0x3e9780,0xd85b0000,'p'},
        {0x3e9784,0x4bc00643,'-'},{0x3e9788,0x4bdcdeac,'-'},{0x3e978c,0x4bdad6ea,'-'},{0x3e9790,0x4b1bd83d,'-'},{0x3e9794,0x4b1bceca,'-'},{0x3e9798,0x4a7b03be,'-'},
        {0x3e97a0,0x4a0003bf,'-'},{0x3e97a4,0x4bc0d69c,'-'},{0x3e97a8,0x4bda0afe,'-'},{0x3e97ac,0x4bc1d2ae,'-'},{0x3e97b0,0x4bda12fe,'-'},{0x3e97b4,0x4bc2d2ee,'-'},
        {0x3e97b8,0xf84a0000,'1'},{0x3e97bc,0xf84b0010,'2'},
        {0x3e989c,0xd85b0000,'p'},{0x3e98a0,0x4bdcdeac,'-'},{0x3e98a4,0x4bdad6ea,'-'},{0x3e98a8,0x4b1bd83d,'-'},{0x3e98ac,0x4b1bceca,'-'},{0x3e98b0,0x4a7b03be,'-'},
        {0x3e98c4,0x4a0003bf,'-'},{0x3e98c8,0x4bc0d69c,'-'},{0x3e98cc,0x4bda0afe,'-'},{0x3e98d0,0x4bc1d2ae,'-'},{0x3e98d4,0x4bda12fe,'-'},{0x3e98d8,0x4bc2d2ee,'-'},
        {0x3e98dc,0xf86a0000,'1'},{0x3e98e0,0xf86b0010,'2'},
        {0x4813f4,0xd8410000,'a'},{0x4813f8,0xd8420010,'b'},{0x481434,0xd89a0000,'o'},{0x481438,0xd91b0000,'p'},
        {0x48143c,0x4bc007c3,'-'},{0x481440,0x4bdadeac,'-'},{0x481444,0x4bdad6ea,'-'},{0x481448,0x4b1bd83d,'-'},{0x48144c,0x4b1bfeca,'-'},{0x481450,0x4a7b03be,'-'},
        {0x481454,0x4a0003bf,'-'},{0x481458,0x4bc0d69c,'-'},{0x48145c,0x4bda0afe,'-'},{0x481460,0x4bc1d2ae,'-'},{0x481464,0x4bda12fe,'-'},{0x481468,0x4bc2d2ee,'-'},
        {0x48146c,0xf86a0000,'1'},{0x481470,0xf86b0010,'2'}};
    if((pc&3) || !rt || !((pc>=0x3e9764 && pc<=0x3e98e0) || (pc>=0x4813f4 && pc<=0x481470)))return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](const Site &x){return x.pc==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle side axes have no CPU record");
    auto &states=battleSideAxes();const auto key=std::make_pair(static_cast<const Runtime *>(rt),c->r[29].uw[0]);
    const auto reg=[&](uint32_t w){return c->r[(w>>21)&31].uw[0]+(w&0xffff);};
    if(site->role=='a'){auto &st=states[key];st=BattleSideAxes{};st.a=vuWords(rt,reg(word));st.loaded=1;return true;}
    auto it=states.find(key);
    if(it==states.end())throw std::runtime_error(std::format("native battle side axes at {:x} without their axis loads",pc));
    auto &st=it->second;
    switch(site->role){
    case 'b':st.b=vuWords(rt,reg(word));st.loaded|=2;return true;
    case 'o':st.base=vuWords(rt,reg(word));st.loaded|=4;return true;
    case 'p':st.point=vuWords(rt,reg(word));st.loaded|=8;st.done=false;return true;
    case '1':{
        if(st.loaded!=15)throw std::runtime_error("native battle side axes stored without their loads");
        constexpr uint32_t one=0x3f800000u;uint32_t d[3],sq[3];
        for(unsigned i=0;i<3;++i){d[i]=vuSub(st.point[i],st.base[i]);sq[i]=vuMul(d[i],d[i]);}
        uint32_t f=0;const uint32_t savedCsr=_mm_getcsr();
        const auto q=bozo::ps2f::rsqrt(one,vuAdd(vuAdd(sq[0],sq[1]),vuMul(one,sq[2])),f);_mm_setcsr(savedCsr);
        uint32_t n[3];for(unsigned i=0;i<3;++i)n[i]=vuMul(d[i],q);
        const auto cross=[&](const VuWords &v){VuWords r{};
            r[0]=vuSub(vuMul(v[1],n[2]),vuMul(n[1],v[2]));r[1]=vuSub(vuMul(v[2],n[0]),vuMul(n[2],v[0]));r[2]=vuSub(vuMul(v[0],n[1]),vuMul(n[0],v[1]));return r;};
        st.first=cross(st.a);st.second=cross(st.b);st.done=true;
        std::memcpy(record(rt,reg(word),12),st.first.data(),12);return true;}
    case '2':
        if(!st.done)throw std::runtime_error("native battle side axes stored vf11 before vf10");
        std::memcpy(record(rt,reg(word),12),st.second.data(),12);return true;
    default:return true;   // arithmetic: folded into the stores
    }
}
// Battle ground plane, mod028 4981b4 (one copy; ps2f, nearest). The object's three ground
// points P0 = +8f0 (s2), P1 = +900 (s1), P2 = +910 (s0): LQC2 vf1 = P0 (4981f0) before the
// first 4ab730 call, vf2 = P1 (498228) and vf3 = P2 (498260) after the CPU may have set their
// y to P0's (no COP2 in between; P0 is kept per call frame). Then (498264..4982c8):
// a = (P1 - P0) * RSQRT(|P1 - P0|^2), b = (P2 - P0) * RSQRT(|P2 - P0|^2) (ADDAy/MADDz sums);
// vf30 = (-1, 0, _, -1); n = a x b (OPMULA/OPMSUB); n.w = -1 * (n . P0); SQC2 n to sp+80
// (4982d8). If n.y < 0 (CPU test), n *= -1 (4982f0) and SQC2 again (4982f4). Then the CPU's
// k = -1 / n.y (or d51184e7 when n.y = 0) enters vf1 by QMTC2 t0 (49834c); vf30.xzw = n * k
// (498350) and SQC2 vf30 (y = 0) to +8c0 (498354): the plane as y = f(x, z).
struct BattleGroundPlane{VuWords p0{},p1{},p2{},n{};unsigned loaded=0;bool plane=false;uint32_t k=0;};
std::map<std::pair<const Runtime *,uint32_t>,BattleGroundPlane> &battleGroundPlanes(){static std::map<std::pair<const Runtime *,uint32_t>,BattleGroundPlane> m;return m;}
bool nativeBattleGroundPlane(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Site{uint32_t pc,word;};
    constexpr Site sites[]={{0x4981f0,0xda410000},{0x498228,0xda220000},{0x498260,0xda030000},
        {0x498264,0x4bc112ac},{0x498268,0x4bc007c3},{0x49826c,0x4bc11aec},{0x498270,0x4bca552a},{0x498274,0x4b14a03d},{0x498278,0x4b14fd0a},{0x49827c,0x4a7403be},
        {0x498280,0x4bcb5d6a},{0x498284,0x4b15a83d},{0x498288,0x4b15fd4a},{0x49828c,0x4a0003bf},{0x498290,0x4bc0529c},{0x498294,0x4a7503be},{0x498298,0x4b000787},
        {0x49829c,0x4a800781},{0x4982a8,0x4a0003bf},{0x4982ac,0x4bc05adc},{0x4982b0,0x4a3e0798},{0x4982b4,0x4bcb52fe},{0x4982b8,0x4bca5d2e},{0x4982bc,0x4bc1a66a},
        {0x4982c0,0x4b19c83d},{0x4982c4,0x4b19fe4a},{0x4982c8,0x4a39f518},{0x4982d8,0xf8540000},{0x4982f0,0x4bfea518},{0x4982f4,0xf8540000},
        {0x49834c,0x48a80800},{0x498350,0x4b61a798},{0x498354,0xf85e0000}};
    if((pc&3) || !rt || pc<0x4981f0 || pc>0x498354)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](const Site &x){return x.pc==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle ground plane has no CPU record");
    auto &states=battleGroundPlanes();const auto key=std::make_pair(static_cast<const Runtime *>(rt),c->r[29].uw[0]);
    const auto at=[&](uint32_t w){return c->r[(w>>21)&31].uw[0]+(w&0xffff);};
    if(pc==0x4981f0){auto &st=states[key];st=BattleGroundPlane{};st.p0=vuWords(rt,at(word));st.loaded=1;return true;}
    auto it=states.find(key);
    if(it==states.end())throw std::runtime_error(std::format("native battle ground plane at {:x} without its first load",pc));
    auto &st=it->second;constexpr uint32_t one=0x3f800000u,minusOne=0xbf800000u;
    switch(pc){
    case 0x498228:st.p1=vuWords(rt,at(word));st.loaded|=2;return true;
    case 0x498260:st.p2=vuWords(rt,at(word));st.loaded|=4;return true;
    case 0x4982d8:{
        if(st.loaded!=7)throw std::runtime_error("native battle ground plane stored without its points");
        uint32_t f=0;const uint32_t savedCsr=_mm_getcsr();
        const auto unit=[&](const VuWords &q){uint32_t d[3],sq[3];for(unsigned i=0;i<3;++i){d[i]=vuSub(q[i],st.p0[i]);sq[i]=vuMul(d[i],d[i]);}
            const auto r=bozo::ps2f::rsqrt(one,vuAdd(vuAdd(sq[0],sq[1]),vuMul(one,sq[2])),f);std::array<uint32_t,3> u;for(unsigned i=0;i<3;++i)u[i]=vuMul(d[i],r);return u;};
        const auto a=unit(st.p1),b=unit(st.p2);_mm_setcsr(savedCsr);
        st.n[0]=vuSub(vuMul(a[1],b[2]),vuMul(b[1],a[2]));st.n[1]=vuSub(vuMul(a[2],b[0]),vuMul(b[2],a[0]));st.n[2]=vuSub(vuMul(a[0],b[1]),vuMul(b[0],a[1]));
        uint32_t m[3];for(unsigned i=0;i<3;++i)m[i]=vuMul(st.n[i],st.p0[i]);
        st.n[3]=vuMul(minusOne,vuAdd(vuAdd(m[0],m[1]),vuMul(one,m[2])));st.plane=true;
        std::memcpy(record(rt,at(word),16),st.n.data(),16);return true;}
    case 0x4982f0:if(!st.plane)throw std::runtime_error("native battle ground plane flipped before its plane");
        for(auto &x:st.n)x=vuMul(x,minusOne);return true;
    case 0x4982f4:if(!st.plane)throw std::runtime_error("native battle ground plane stored before its plane");
        std::memcpy(record(rt,at(word),16),st.n.data(),16);return true;
    case 0x49834c:st.k=c->r[8].uw[0];return true;   // QMTC2 t0: vf1.x = k
    case 0x498354:{
        if(!st.plane)throw std::runtime_error("native battle ground plane slope without its plane");
        const VuWords out{vuMul(st.n[0],st.k),0u,vuMul(st.n[2],st.k),vuMul(st.n[3],st.k)};
        std::memcpy(record(rt,at(word),16),out.data(),16);states.erase(it);return true;}
    default:return true;   // arithmetic: folded into the stores
    }
}
// Battle task colours (vtable 171f20): its slot +1c, mod028 47a9b0(task s1, object s0), ends
// with LQC2 vf21 = object +9a0, vf22 = +9b0 (47aa5c, 47aa60; the pair 47ae40 fills) and
// returns. Only its draw slot +10, 47a910, reads vf21/vf22, in a later call after other code
// that also uses them: on the PS2 the blend gets whatever the registers last held. Rule (user
// decision 2026-11-21): each task keeps the pair its own 47a9b0 loaded (battleTaskColours),
// and the native element draw (taskColourMesh, from 219210) blends that. The pairs live for
// the run (not in save states: a draw without its pair is refused).
bool nativeBattleTaskColours(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc==0x47cdd8 || pc==0x47cddc){   // the flash task's 47cdc0 (see taskFlashMesh)
        if(!rt)return false;
        uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
        if(word!=(pc==0x47cdd8?0x48a8b800u:0x4a3705d8u))return false;   // another overlay at these addresses
        if(!c)throw std::runtime_error("native battle flash value has no CPU record");
        if(pc==0x47cdd8)battleTaskFlashes()[{rt,c->r[4].uw[0]}]=vuMul(0x3f800000u,c->r[8].uw[0]);   // w = vf0.w * t0
        return true;}
    if(pc!=0x47aa5c && pc!=0x47aa60)return false;
    if(!rt)return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=(pc==0x47aa5c?0xd8550000u:0xd8560010u))return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle task colours have no CPU record");
    if(pc==0x47aa60)return true;   // the second load: taken with the first
    if(c->r[2].uw[0]!=c->r[16].uw[0]+0x9a0)throw std::runtime_error("native battle task colours load another pair than the object's +9a0");
    auto &pair=battleTaskColours()[{rt,c->r[17].uw[0]}];pair.first=vuWords(rt,c->r[2].uw[0]);pair.second=vuWords(rt,c->r[2].uw[0]+16);
    return true;
}
// Battle point scale, mod028 47e8c0 (one copy): QMTC2 t0 -> vf30 at 47ec2c (the delay slot
// of 47ec28, so every path runs it; t0 = 0.4f), then four blocks (47ecc4, 47ed9c, 47ef64,
// 47f0ec): LQC2 vf1 = (v0); MULx.xyz vf1 *= vf30.x; SQC2 vf1 back to the same (v0) before the
// vertex packer 4d7060 -> {p.xyz * k, p.w}. vf30 is kept per call frame; a block without it,
// or storing elsewhere than it loaded, is refused.
struct BattlePointScale{uint32_t k=0;bool scale=false;VuWords point{};uint32_t at=0;bool loaded=false;};
std::map<std::pair<const Runtime *,uint32_t>,BattlePointScale> &battlePointScales(){static std::map<std::pair<const Runtime *,uint32_t>,BattlePointScale> m;return m;}
bool nativeBattlePointScale(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Site{uint32_t pc,word;char role;};
    constexpr Site sites[]={{0x47ec2c,0x48a8f000,'k'},
        {0x47ecc4,0xd8410000,'l'},{0x47ecd8,0x4bde0858,'-'},{0x47ece8,0xf8410000,'s'},
        {0x47ed9c,0xd8410000,'l'},{0x47edb0,0x4bde0858,'-'},{0x47edc0,0xf8410000,'s'},
        {0x47ef64,0xd8410000,'l'},{0x47ef78,0x4bde0858,'-'},{0x47ef98,0xf8410000,'s'},
        {0x47f0ec,0xd8410000,'l'},{0x47f100,0x4bde0858,'-'},{0x47f120,0xf8410000,'s'}};
    if((pc&3) || !rt || pc<0x47ec2c || pc>0x47f120)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](const Site &x){return x.pc==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle point scale has no CPU record");
    auto &st=battlePointScales()[{rt,c->r[29].uw[0]}];
    switch(site->role){
    case 'k':st=BattlePointScale{};st.k=c->r[8].uw[0];st.scale=true;return true;
    case 'l':if(!st.scale)throw std::runtime_error(std::format("native battle point scale at {:x} without its scale",pc));
        st.at=c->r[2].uw[0];st.point=vuWords(rt,st.at);st.loaded=true;return true;
    case 's':{
        if(!st.loaded || c->r[2].uw[0]!=st.at)throw std::runtime_error(std::format("native battle point scale at {:x} stored without its load",pc));
        VuWords out=st.point;for(unsigned i=0;i<3;++i)out[i]=vuMul(st.point[i],st.k);
        std::memcpy(record(rt,st.at,16),out.data(),16);st.loaded=false;return true;}
    default:return true;   // MULx: folded into the store
    }
}
// Battle half vectors, mod028 49f310 (two copies, 49f418 and 49f4e0): QMTC2 t0 (0.5f) -> vf3;
// LQC2 vf1 = (s4), vf2 = (s3); MULx.xyz vf1, vf2 *= vf3.x; SQC2 vf1 to the new object (a1 /
// s1) and vf2 to its +10 (v0) -> {xyz * k, w}. Both results come from the loads, made before
// either store. Kept per call frame; stores without their loads are refused.
struct BattleHalfVectors{uint32_t k=0;VuWords a{},b{};unsigned loaded=0;};
std::map<std::pair<const Runtime *,uint32_t>,BattleHalfVectors> &battleHalfVectors(){static std::map<std::pair<const Runtime *,uint32_t>,BattleHalfVectors> m;return m;}
bool nativeBattleHalfVectors(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Site{uint32_t pc,word;char role;};
    constexpr Site sites[]={{0x49f418,0x48a81800,'k'},{0x49f41c,0xda810000,'a'},{0x49f420,0xda620000,'b'},{0x49f428,0x4bc30858,'-'},{0x49f42c,0x4bc31098,'-'},
        {0x49f430,0xf8a10000,'1'},{0x49f434,0xf8420000,'2'},
        {0x49f4e0,0x48a81800,'k'},{0x49f4e4,0xda810000,'a'},{0x49f4e8,0xda620000,'b'},{0x49f4f0,0x4bc30858,'-'},{0x49f4f4,0x4bc31098,'-'},
        {0x49f4f8,0xfa210000,'1'},{0x49f4fc,0xf8420000,'2'}};
    if((pc&3) || !rt || pc<0x49f418 || pc>0x49f4fc)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](const Site &x){return x.pc==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle half vectors have no CPU record");
    auto &st=battleHalfVectors()[{rt,c->r[29].uw[0]}];
    const auto reg=[&](uint32_t w){return c->r[(w>>21)&31].uw[0]+(w&0xffff);};
    const auto half=[&](const VuWords &v){VuWords o=v;for(unsigned i=0;i<3;++i)o[i]=vuMul(v[i],st.k);return o;};
    switch(site->role){
    case 'k':st=BattleHalfVectors{};st.k=c->r[8].uw[0];st.loaded=1;return true;
    case 'a':st.a=vuWords(rt,reg(word));st.loaded|=2;return true;
    case 'b':st.b=vuWords(rt,reg(word));st.loaded|=4;return true;
    case '1':case '2':{
        if((st.loaded&7)!=7)throw std::runtime_error(std::format("native battle half vectors at {:x} stored without their loads",pc));
        const auto out=half(site->role=='1'?st.a:st.b);std::memcpy(record(rt,reg(word),16),out.data(),16);return true;}
    default:return true;   // MULx: folded into the stores
    }
}
// Battle segment points, mod028 48b260(object, index): LQC2 vf1 = A (the segment +0), vf2 = B
// (+10) once (48b280, 48b284); per record QMTC2 t0 -> vf3 (t, from 1), t1 -> vf4 (u, from 0)
// (48b2d0, 48b2d4); MULAx.xyzw ACC = A t; MADDx.xyzw vf3 = ACC + B u; SQC2 vf3 to the record
// +20 (48b314) -> A t + B u. Kept per call frame; a store without the points or its weights
// is refused.
struct BattleSegmentPoints{VuWords a{},b{};uint32_t t=0,u=0;unsigned loaded=0;};
std::map<std::pair<const Runtime *,uint32_t>,BattleSegmentPoints> &battleSegmentPoints(){static std::map<std::pair<const Runtime *,uint32_t>,BattleSegmentPoints> m;return m;}
bool nativeBattleSegmentPoints(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Site{uint32_t pc,word;char role;};
    constexpr Site sites[]={{0x48b280,0xd8e10000,'a'},{0x48b284,0xd8c20000,'b'},{0x48b2d0,0x48a81800,'t'},{0x48b2d4,0x48a92000,'u'},
        {0x48b30c,0x4be309bc,'-'},{0x48b310,0x4be410c8,'-'},{0x48b314,0xf8a30000,'s'}};
    if((pc&3) || !rt || pc<0x48b280 || pc>0x48b314)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](const Site &x){return x.pc==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle segment points have no CPU record");
    auto &st=battleSegmentPoints()[{rt,c->r[29].uw[0]}];
    const auto reg=[&](uint32_t w){return c->r[(w>>21)&31].uw[0]+(w&0xffff);};
    switch(site->role){
    case 'a':st=BattleSegmentPoints{};st.a=vuWords(rt,reg(word));st.loaded=1;return true;
    case 'b':st.b=vuWords(rt,reg(word));st.loaded|=2;return true;
    case 't':st.t=c->r[8].uw[0];st.loaded=(st.loaded&3)|4;return true;
    case 'u':st.u=c->r[9].uw[0];st.loaded|=8;return true;
    case 's':{
        if(st.loaded!=15)throw std::runtime_error("native battle segment point stored without its points or weights");
        VuWords out;for(unsigned i=0;i<4;++i)out[i]=vuAdd(vuMul(st.a[i],st.t),vuMul(st.b[i],st.u));
        std::memcpy(record(rt,reg(word),16),out.data(),16);st.loaded&=3;return true;}
    default:return true;   // MULAx / MADDx: folded into the store
    }
}
// Battle record shift, mod028 48ad10: LQC2 vf1 = the last record's +10 (48adc0) before the CPU
// shifts the records' +10 fields along (LQ/SQ), then SQC2 vf1 into each remaining record's
// +10 (48ae0c, a loop): a 128-bit copy of the saved value. Kept per call frame; a fill
// without the save is refused.
struct BattleRecordShift{VuWords saved{};bool loaded=false;};
std::map<std::pair<const Runtime *,uint32_t>,BattleRecordShift> &battleRecordShifts(){static std::map<std::pair<const Runtime *,uint32_t>,BattleRecordShift> m;return m;}
bool nativeBattleRecordShift(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x48adc0 && pc!=0x48ae0c)return false;
    if(!rt)return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=(pc==0x48adc0?0xd8610000u:0xf8610000u))return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle record shift has no CPU record");
    auto &st=battleRecordShifts()[{rt,c->r[29].uw[0]}];
    if(pc==0x48adc0){st.saved=vuWords(rt,c->r[3].uw[0]);st.loaded=true;return true;}
    if(!st.loaded)throw std::runtime_error("native battle record fill without its saved value");
    std::memcpy(record(rt,c->r[3].uw[0],16),st.saved.data(),16);return true;
}
// Battle advance, mod028 49c3b0 (49c7a4..49c7e8): QMTC2 t0 (f21) -> vf4; LQC2 vf1 = (s5), vf2 =
// (s2), vf3 = (s1); MULx.xyz vf5 = vf1 * k; MULAw.xyz ACC = vf2 * 1; MADDx.xyz vf2 = ACC + vf3 *
// vf5.x; ADDy.x vf1.x += vf5.y; SQC2 vf2 to (s2), vf1 to (s5) (the delay slot of 49c7e4).
// So vf2.xyz += vf3.xyz * (vf1.x k) and vf1.x += vf1.y k; the other lanes are stored as loaded.
// Kept per call frame; stores without their loads are refused.
struct BattleAdvance{uint32_t k=0;VuWords a{},b{},c{};unsigned loaded=0;bool stored=false;VuWords outA{},outB{};};
std::map<std::pair<const Runtime *,uint32_t>,BattleAdvance> &battleAdvances(){static std::map<std::pair<const Runtime *,uint32_t>,BattleAdvance> m;return m;}
bool nativeBattleAdvance(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Site{uint32_t pc,word;char role;};
    constexpr Site sites[]={{0x49c7a4,0x48a82000,'k'},{0x49c7a8,0xdaa10000,'1'},{0x49c7ac,0xda420000,'2'},{0x49c7b0,0xda230000,'3'},
        {0x49c7d0,0x4bc40958,'-'},{0x49c7d4,0x4bc011bf,'-'},{0x49c7d8,0x4bc51888,'-'},{0x49c7dc,0x4b050841,'-'},{0x49c7e0,0xfa420000,'b'},{0x49c7e8,0xfaa10000,'a'}};
    if((pc&3) || !rt || pc<0x49c7a4 || pc>0x49c7e8)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](const Site &x){return x.pc==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle advance has no CPU record");
    auto &st=battleAdvances()[{rt,c->r[29].uw[0]}];
    const auto reg=[&](uint32_t w){return c->r[(w>>21)&31].uw[0]+(w&0xffff);};
    switch(site->role){
    case 'k':st=BattleAdvance{};st.k=c->r[8].uw[0];st.loaded=1;return true;
    case '1':st.a=vuWords(rt,reg(word));st.loaded|=2;return true;
    case '2':st.b=vuWords(rt,reg(word));st.loaded|=4;return true;
    case '3':st.c=vuWords(rt,reg(word));st.loaded|=8;return true;
    case 'b':case 'a':{
        if(st.loaded!=15)throw std::runtime_error(std::format("native battle advance at {:x} stored without its loads",pc));
        if(!st.stored){constexpr uint32_t one=0x3f800000u;const auto fx=vuMul(st.a[0],st.k),fy=vuMul(st.a[1],st.k);
            st.outB=st.b;for(unsigned i=0;i<3;++i)st.outB[i]=vuAdd(vuMul(st.b[i],one),vuMul(st.c[i],fx));
            st.outA=st.a;st.outA[0]=vuAdd(st.a[0],fy);st.stored=true;}
        const auto &out=site->role=='b'?st.outB:st.outA;std::memcpy(record(rt,reg(word),16),out.data(),16);
        if(site->role=='a')st=BattleAdvance{};
        return true;}
    default:return true;   // MULx / MULAw / MADDx / ADDy: folded into the stores
    }
}
// Battle nearest point, mod028 4a50b0(node a0, out a1, best a2), a recursive walk: per node,
// LQC2 vf11..vf14 = the object's rows (s1+150, 4a52f4..4a5300), vf15 = the reference point
// (*(5136d8), 4a5304), ADDw.xyz vf31 = 1 (4a5308). Per entry (a1+10, 4a5354): vf10 = ((r1 x +
// r2 y) + r3 z) + r4 * 1 (MULAx/MADDAy/MADDAz/MADDw, all lanes); vf24.xyz = (vf10 - vf15)^2,
// vf24.x = (x + y) + 1 * z, SQRT, ADDq.x (4a5370..4a53b0); QMFC2 a3 (4a53b4): the compiled
// code reads only its low word (mtc1 f7), the distance. A closer entry stores vf10 to (s4)
// (SQC2 4a5488). The other lanes of a3 are written as computed (y, z the squares; w 0: this
// function never writes vf24.w and nothing reads it). ps2f, nearest; kept per call frame.
struct BattleNearestPoint{std::array<VuWords,4> rows{};VuWords reference{},point{};unsigned loaded=0;bool transformed=false;uint32_t squares[3]{},distance=0;};
std::map<std::pair<const Runtime *,uint32_t>,BattleNearestPoint> &battleNearestPoints(){static std::map<std::pair<const Runtime *,uint32_t>,BattleNearestPoint> m;return m;}
bool nativeBattleNearestPoint(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Site{uint32_t pc,word;char role;};
    constexpr Site sites[]={{0x4a52f4,0xd86b0000,'0'},{0x4a52f8,0xd86c0010,'1'},{0x4a52fc,0xd86d0020,'2'},{0x4a5300,0xd86e0030,'3'},{0x4a5304,0xd84f0000,'r'},{0x4a5308,0x4bc007c3,'-'},
        {0x4a5354,0xd84a0000,'p'},{0x4a5358,0x4bea59bc,'-'},{0x4a535c,0x4bea60bd,'-'},{0x4a5360,0x4bea68be,'-'},{0x4a5364,0x4be0728b,'t'},
        {0x4a5370,0x4bcf562c,'-'},{0x4a5374,0x4bd8c62a,'-'},{0x4a5378,0x4b18c03d,'-'},{0x4a537c,0x4b18fe0a,'-'},{0x4a5380,0x4a1803bd,'q'},{0x4a53ac,0x4a0003bf,'-'},{0x4a53b0,0x4b000620,'-'},
        {0x4a53b4,0x4827c000,'m'},{0x4a5488,0xfa8a0000,'s'}};
    if((pc&3) || !rt || pc<0x4a52f4 || pc>0x4a5488)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](const Site &x){return x.pc==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle nearest point has no CPU record");
    auto &st=battleNearestPoints()[{rt,c->r[29].uw[0]}];
    const auto at=[&]{return c->r[(word>>21)&31].uw[0]+(word&0xffff);};
    switch(site->role){
    case '0':st=BattleNearestPoint{};st.rows[0]=vuWords(rt,at());st.loaded=1;return true;
    case '1':case '2':case '3':{const unsigned i=site->role-'0';if(st.loaded!=(1u<<i)-1)throw std::runtime_error("native battle nearest point rows out of order");
        st.rows[i]=vuWords(rt,at());st.loaded|=1u<<i;return true;}
    case 'r':if(st.loaded!=15)throw std::runtime_error("native battle nearest point reference without its rows");st.reference=vuWords(rt,at());st.loaded|=16;return true;
    case 'p':if(!(st.loaded&16))throw std::runtime_error("native battle nearest point entry without its rows");st.point=vuWords(rt,at());st.loaded|=32;st.transformed=false;return true;
    case 't':{if(!(st.loaded&32))throw std::runtime_error("native battle nearest point transform without its entry");
        constexpr uint32_t one=0x3f800000u;VuWords v;
        for(unsigned i=0;i<4;++i)v[i]=vuAdd(vuAdd(vuAdd(vuMul(st.rows[0][i],st.point[0]),vuMul(st.rows[1][i],st.point[1])),vuMul(st.rows[2][i],st.point[2])),vuMul(st.rows[3][i],one));
        st.point=v;st.transformed=true;return true;}
    case 'q':{if(!st.transformed)throw std::runtime_error("native battle nearest point distance without its transform");
        constexpr uint32_t one=0x3f800000u;for(unsigned i=0;i<3;++i){const auto d=vuSub(st.point[i],st.reference[i]);st.squares[i]=vuMul(d,d);}
        uint32_t flags=0;st.distance=vuAdd(0,bozo::ps2f::sqrt(vuAdd(vuAdd(st.squares[0],st.squares[1]),vuMul(one,st.squares[2])),flags));st.loaded|=64;return true;}
    case 'm':if(!(st.loaded&64))throw std::runtime_error("native battle nearest point QMFC2 without its distance");
        c->r[7].uw[0]=st.distance;c->r[7].uw[1]=st.squares[1];c->r[7].uw[2]=st.squares[2];c->r[7].uw[3]=0;st.loaded&=~64u;return true;
    case 's':if(!st.transformed)throw std::runtime_error("native battle nearest point store without its transform");
        std::memcpy(record(rt,at(),16),st.point.data(),16);return true;
    default:return true;   // MULA/MADDA/SUB/MUL/ADDA/MADD/WAITQ/ADDq/ADDw: folded into the roles above
    }
}
// Contact point, mod001 1f7b80(out a0)'s general path (1f7f20..1f7fac): LQC2 vf11 = (a2) and
// vf12 = (a1) (two extent vectors, sp+a0 / sp+90), vf9 = (t0) and vf10 = (v0) (two centres),
// vf1..vf4 = rows (a0), vf5..vf8 = rows (v1). MULAx/MADDAy/MADDz: vf11 = (r1 a.x + r2 a.y) +
// r3 a.z (all lanes, then ABS.xyz), vf12 likewise by vf5..vf7; MULAx/MADDAy/MADDAz/MADDw: vf9 =
// ((r1 c.x + r2 c.y) + r3 c.z) + r4 * 1, vf10 by vf5..vf8. SQC2 vf9 -> sp+80, vf10 -> sp+70,
// vf11 -> sp+60, vf12 -> sp+50; the scalar code then takes the overlap midpoints. ps2f, nearest,
// kept per call frame.
struct ContactPoint{std::array<VuWords,13> vf{};uint32_t loaded=0;bool done=false;VuWords out[4]{};};
std::map<std::pair<const Runtime *,uint32_t>,ContactPoint> &contactPoints(){static std::map<std::pair<const Runtime *,uint32_t>,ContactPoint> m;return m;}
bool nativeContactPoint(R5900Context *c,Runtime *rt,uint32_t pc){
    // Loads: register number; stores: -1 - output index; 0: folded arithmetic.
    struct Site{uint32_t pc,word;int role;};
    constexpr Site sites[]={{0x1f7f20,0xd8cb0000,11},{0x1f7f24,0xd8ac0000,12},{0x1f7f28,0xd9090000,9},{0x1f7f2c,0xd8810000,1},{0x1f7f30,0xd8820010,2},{0x1f7f34,0xd8830020,3},{0x1f7f38,0xd8840030,4},
        {0x1f7f3c,0xd84a0000,10},{0x1f7f40,0xd8650000,5},{0x1f7f44,0xd8660010,6},{0x1f7f48,0xd8670020,7},{0x1f7f4c,0xd8680030,8},
        {0x1f7f50,0x4beb09bc,0},{0x1f7f54,0x4beb10bd,0},{0x1f7f58,0x4beb1aca,0},{0x1f7f5c,0x4bec29bc,0},{0x1f7f60,0x4bec30bd,0},{0x1f7f64,0x4bec3b0a,0},{0x1f7f68,0x4bcb59fd,0},
        {0x1f7f6c,0x4be909bc,0},{0x1f7f70,0x4be910bd,0},{0x1f7f74,0x4bcc61fd,0},{0x1f7f78,0x4be918be,0},{0x1f7f7c,0x4be0224b,0},{0x1f7f80,0x4bea29bc,0},{0x1f7f84,0x4bea30bd,0},
        {0x1f7f88,0x4bea38be,0},{0x1f7f8c,0x4be0428b,0},{0x1f7fa0,0xf8a90000,-1},{0x1f7fa4,0xf88a0000,-2},{0x1f7fa8,0xf86b0000,-3},{0x1f7fac,0xf84c0000,-4}};
    if((pc&3) || !rt || pc<0x1f7f20 || pc>0x1f7fac)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](const Site &x){return x.pc==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native contact point has no CPU record");
    auto &st=contactPoints()[{rt,c->r[29].uw[0]}];
    const auto at=[&]{return c->r[(word>>21)&31].uw[0]+(word&0xffff);};
    if(site->role>0){if(pc==0x1f7f20)st=ContactPoint{};else if(st.done || !(st.loaded&(1u<<11)))throw std::runtime_error("native contact point loads out of order");
        st.vf[site->role]=vuWords(rt,at());st.loaded|=1u<<site->role;return true;}
    if(site->role==0)return true;
    if(st.loaded!=0x1ffeu)throw std::runtime_error(std::format("native contact point store at {:x} without its loads",pc));
    if(!st.done){constexpr uint32_t one=0x3f800000u;const auto &v=st.vf;
        const auto extent=[&](unsigned r,const VuWords &a){VuWords o;for(unsigned i=0;i<4;++i)o[i]=vuAdd(vuAdd(vuMul(v[r][i],a[0]),vuMul(v[r+1][i],a[1])),vuMul(v[r+2][i],a[2]));
            for(unsigned i=0;i<3;++i)o[i]&=0x7fffffffu;return o;};
        const auto centre=[&](unsigned r,const VuWords &p){VuWords o;for(unsigned i=0;i<4;++i)o[i]=vuAdd(vuAdd(vuAdd(vuMul(v[r][i],p[0]),vuMul(v[r+1][i],p[1])),vuMul(v[r+2][i],p[2])),vuMul(v[r+3][i],one));return o;};
        st.out[0]=centre(1,v[9]);st.out[1]=centre(5,v[10]);st.out[2]=extent(1,v[11]);st.out[3]=extent(5,v[12]);st.done=true;}
    const unsigned index=unsigned(-1-site->role);std::memcpy(record(rt,at(),16),st.out[index].data(),16);
    if(index==3)st=ContactPoint{};
    return true;
}
// Nearest of points, mod028 4ab680(point a0, points a1, count a2, out a3): LQC2 vf30 = (a0)
// (4ab698), ADDw.xyz vf25 = 1; per point LQC2 vf31 = (a1) (4ab6b8), SUB.xyz / MUL.xyz, vf31.x =
// (x + y) + 1 * z, SQRT, ADDq.x; QMFC2 v1 (4ab6dc): x the distance, y and z the squares, w the
// point's own w (SUB/MUL write xyz only). The scalar loop keeps the smallest. ps2f, nearest.
struct NearestOfPoints{VuWords centre{},point{};bool centred=false,loaded=false;uint32_t out[4]{};bool ready=false;};
std::map<std::pair<const Runtime *,uint32_t>,NearestOfPoints> &nearestOfPoints(){static std::map<std::pair<const Runtime *,uint32_t>,NearestOfPoints> m;return m;}
bool nativeNearestOfPoints(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Site{uint32_t pc,word;char role;};
    constexpr Site sites[]={{0x4ab698,0xd89e0000,'c'},{0x4ab69c,0x4bc00643,'-'},{0x4ab6b8,0xd8bf0000,'p'},{0x4ab6bc,0x4bdeffec,'-'},{0x4ab6c4,0x4bdfffea,'-'},{0x4ab6c8,0x4b1ff83d,'-'},
        {0x4ab6cc,0x4b1fcfca,'-'},{0x4ab6d0,0x4a1f03bd,'q'},{0x4ab6d4,0x4a0003bf,'-'},{0x4ab6d8,0x4b0007e0,'-'},{0x4ab6dc,0x4823f800,'m'}};
    if((pc&3) || !rt || pc<0x4ab698 || pc>0x4ab6dc)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](const Site &x){return x.pc==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native nearest of points has no CPU record");
    auto &st=nearestOfPoints()[{rt,c->r[29].uw[0]}];
    switch(site->role){
    case 'c':st=NearestOfPoints{};st.centre=vuWords(rt,c->r[4].uw[0]);st.centred=true;return true;
    case 'p':if(!st.centred)throw std::runtime_error("native nearest of points without its centre");st.point=vuWords(rt,c->r[5].uw[0]);st.loaded=true;st.ready=false;return true;
    case 'q':{if(!st.loaded)throw std::runtime_error("native nearest of points distance without its point");constexpr uint32_t one=0x3f800000u;
        uint32_t sq[3];for(unsigned i=0;i<3;++i){const auto d=vuSub(st.point[i],st.centre[i]);sq[i]=vuMul(d,d);}
        uint32_t flags=0;st.out[0]=vuAdd(0,bozo::ps2f::sqrt(vuAdd(vuAdd(sq[0],sq[1]),vuMul(one,sq[2])),flags));st.out[1]=sq[1];st.out[2]=sq[2];st.out[3]=st.point[3];
        st.ready=true;st.loaded=false;return true;}
    case 'm':if(!st.ready)throw std::runtime_error("native nearest of points QMFC2 without its distance");
        for(unsigned i=0;i<4;++i)c->r[3].uw[i]=st.out[i];st.ready=false;return true;
    default:return true;   // ADDw / SUB / MUL / ADDA / MADD / WAITQ / ADDq: folded into the distance
    }
}
// Battle colour pair, mod028 47ae40(object a0, colour a1, colour a2) (one copy): the CPU
// widens each RGBA byte colour to four words (PEXTLB/PEXTLH with zero, compiled); QMTC2 t0
// -> vf1, t1 -> vf2, t2 (0.5f) -> vf3; ITOF0 vf1, vf2; MULx.xyz by vf3.x; SQC2 vf1, vf2 to
// +9a0 / +9b0 (v1): (r, g, b) / 2 and the alpha byte as is (w is not scaled). The words are
// bytes (0..255), so ITOF0 and the halving are exact.
struct BattleColourPair{uint32_t first[4]{},second[4]{},half=0;unsigned loaded=0;};
std::map<std::pair<const Runtime *,uint32_t>,BattleColourPair> &battleColourPairs(){static std::map<std::pair<const Runtime *,uint32_t>,BattleColourPair> m;return m;}
bool nativeBattleColourPair(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Site{uint32_t pc,word;};
    constexpr Site sites[]={{0x47ae78,0x48a80800},{0x47ae7c,0x48a91000},{0x47ae80,0x48aa1800},{0x47ae84,0x4be1093c},{0x47ae88,0x4be2113c},
        {0x47ae90,0x4bc30858},{0x47ae94,0x4bc31098},{0x47ae98,0xf8610000},{0x47ae9c,0xf8620010}};
    if((pc&3) || !rt || pc<0x47ae78 || pc>0x47ae9c)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](const Site &x){return x.pc==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle colour pair has no CPU record");
    auto &states=battleColourPairs();const auto key=std::make_pair(static_cast<const Runtime *>(rt),c->r[29].uw[0]);
    if(pc==0x47ae78){auto &st=states[key];st=BattleColourPair{};for(unsigned i=0;i<4;++i)st.first[i]=c->r[8].uw[i];st.loaded=1;return true;}
    auto it=states.find(key);if(it==states.end())throw std::runtime_error(std::format("native battle colour pair at {:x} without its first move",pc));
    auto &st=it->second;
    if(pc==0x47ae7c){for(unsigned i=0;i<4;++i)st.second[i]=c->r[9].uw[i];st.loaded|=2;return true;}
    if(pc==0x47ae80){st.half=c->r[10].uw[0];st.loaded|=4;return true;}
    if(pc!=0x47ae98 && pc!=0x47ae9c)return true;   // ITOF0 / MULx: folded into the stores
    if(st.loaded!=7)throw std::runtime_error("native battle colour pair stored without its moves");
    const auto &words=pc==0x47ae98?st.first:st.second;VuWords out;
    for(unsigned i=0;i<4;++i){if(words[i]>0xff)throw std::runtime_error("native battle colour pair lane is not a byte");
        const auto f=std::bit_cast<uint32_t>(float(words[i]));out[i]=i<3?vuMul(f,st.half):f;}
    std::memcpy(record(rt,c->r[3].uw[0]+(pc==0x47ae98?0u:16u),16),out.data(),16);
    if(pc==0x47ae9c)states.erase(it);
    return true;
}
// Battle segment ends, mod028 3e47b0 (per element s5 of 0x30 bytes; ps2f):
//   3e48bc..3e48d8: d = a0 - s5; vf1 = d*v1.x + s5*1, vf2 = a0 - d*v1.y (xyz; w as loaded).
//   with s3+2c set, 3e48f0..3e494c: p = s3+10, rows = s1+90; vf11..vf14 = rows * p (lanes);
//     ACC.x = (x+y) + 1*z of row 0, ACC.y = (y+x) + 1*z of row 1, ACC.z = (z+y) + 1*x of
//     row 2 (the MADDw writes to vf5 leave ACC); vf1 += ACC, vf2 += ACC (MADDw.xyz, *1).
//   3e4954/3e4958: vf1 to s5, vf2 to s5+10 (v0, set at 3e4950 on both paths).
// The ends are held per call frame from 3e48d8 to the stores (no calls in between).
std::map<std::pair<const Runtime *,uint32_t>,std::array<std::array<uint32_t,4>,2>> &battleSegmentEnds(){
    static std::map<std::pair<const Runtime *,uint32_t>,std::array<std::array<uint32_t,4>,2>> m;return m;}
bool nativeBattleSegmentEnds(R5900Context *c,Runtime *rt,uint32_t pc){
    constexpr std::pair<uint32_t,uint32_t> sites[]={{0x3e48bc,0xdaa10000},{0x3e48c0,0xd8820000},{0x3e48c4,0x4bc1112c},{0x3e48c8,0xd8630000},{0x3e48cc,0x4bc321bc},{0x3e48d0,0x4bc32119},{0x3e48d4,0x4bc0084b},{0x3e48d8,0x4bc410ac},
        {0x3e48f0,0xd84f0000},{0x3e48f4,0xd86b0000},{0x3e48f8,0xd86c0010},{0x3e48fc,0xd86d0020},{0x3e4900,0xd86e0030},{0x3e4904,0x4bc007c3},{0x3e4908,0x4bef5aea},{0x3e490c,0x4bef632a},{0x3e4910,0x4bef6b6a},{0x3e4914,0x4bef73aa},{0x3e4918,0x4b0b583d},{0x3e491c,0x4b0bf8be},{0x3e4920,0x4b0bf94b},{0x3e4924,0x4a8c603c},{0x3e4928,0x4a8cf8be},{0x3e492c,0x4a8cf94b},{0x3e4930,0x4a4d683d},{0x3e4934,0x4a4df8bc},{0x3e4938,0x4a4df94b},{0x3e493c,0x4a2e703d},{0x3e4940,0x4a2e00be},{0x3e4944,0x4a2e00bc},{0x3e4948,0x4bc0084b},{0x3e494c,0x4bc0108b},{0x3e4954,0xfaa10000},{0x3e4958,0xf8420000}};
    if((pc&3) || !rt || pc<0x3e48bc || pc>0x3e4958)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](auto &x){return x.first==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->second)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle segment ends have no CPU record");
    using Words=std::array<uint32_t,4>;
    const auto load=[&](uint32_t at){Words v;std::memcpy(v.data(),record(rt,at,16),16);return v;};
    constexpr uint32_t one=0x3f800000u;
    auto &states=battleSegmentEnds();const auto key=std::make_pair(static_cast<const Runtime *>(rt),c->r[29].uw[0]);
    if(pc==0x3e48d8){
        auto start=load(c->r[21].uw[0]),end=load(c->r[4].uw[0]);const auto t=load(c->r[3].uw[0]);
        for(unsigned i=0;i<3;++i){const auto d=vuSub(end[i],start[i]);start[i]=vuAdd(vuMul(d,t[0]),vuMul(start[i],one));end[i]=vuSub(end[i],vuMul(d,t[1]));}
        states[key]={start,end};return true;
    }
    if(pc<0x3e4948)return true;
    auto it=states.find(key);if(it==states.end())throw std::runtime_error("native battle segment ends have no first block");
    if(pc==0x3e4948 || pc==0x3e494c){
        if(pc==0x3e494c)return true;   // both ends take the same ACC at 3e4948
        const auto point=load(c->r[2].uw[0]);uint32_t m[3][3];
        for(unsigned r=0;r<3;++r){const auto row=load(c->r[3].uw[0]+16*r);for(unsigned i=0;i<3;++i)m[r][i]=vuMul(row[i],point[i]);}
        const uint32_t acc[3]={vuAdd(vuAdd(m[0][0],m[0][1]),vuMul(one,m[0][2])),vuAdd(vuAdd(m[1][1],m[1][0]),vuMul(one,m[1][2])),vuAdd(vuAdd(m[2][2],m[2][1]),vuMul(one,m[2][0]))};
        for(auto &end:it->second)for(unsigned i=0;i<3;++i)end[i]=vuAdd(acc[i],vuMul(end[i],one));
        return true;
    }
    const auto element=c->r[21].uw[0];
    if(pc==0x3e4954){std::memcpy(record(rt,element,16),it->second[0].data(),16);return true;}
    if(c->r[2].uw[0]!=element+0x10)throw std::runtime_error("native battle segment end escaped its element");
    std::memcpy(record(rt,element+0x10,16),it->second[1].data(),16);states.erase(it);return true;
}
// Battle box sphere, mod028 3fd5f0(a0 box: min +0, max +10): LQC2 vf27 = sp+0 (x = 0.5),
// vf28 = min, vf29 = max; vf31.xyz = (max+min)*0.5, vf30.xyz = (max-min)*0.5, vf28 = vf30^2;
// ACC.w = 1*x, +1*y, vf29.w = +1*z; RSQRT Q = 1/sqrt(vf29.w); vf31.w = vf29.w*Q; SQC2 vf31 to
// a3 = a0+20: (centre, radius). ps2f, nearest; the scalar code around it is compiled.
bool nativeBattleBoxSphere(R5900Context *c,Runtime *rt,uint32_t pc){
    constexpr uint32_t words[]={0xd93b0000,0xd89c0000,0xd91d0000,0x4bdcefe8,0x4bdcefac,0x4bdbffd8,0x4bdbf798,0x4bdef72a,
        0x4a3c01bc,0x4a3c00bd,0x4a3c074a,0x4bfd03be,0x4a0003bf,0x4a20efdc,0xf8ff0000};
    if(pc<0x3fd660 || pc>0x3fd698 || (pc&3) || !rt)return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=words[(pc-0x3fd660)/4])return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle box sphere has no CPU record");
    const auto box=c->r[4].uw[0],stack=c->r[29].uw[0];
    if(c->r[9].uw[0]!=stack || c->r[8].uw[0]!=box+0x10 || c->r[7].uw[0]!=box+0x20)throw std::runtime_error("native battle box sphere escaped its records");
    if(pc!=0x3fd698)return true;
    uint32_t half,low[4],high[4];std::memcpy(&half,record(rt,stack,16),4);std::memcpy(low,record(rt,box,16),16);std::memcpy(high,record(rt,box+0x10,16),16);
    if(half!=0x3f000000u)throw std::runtime_error("native battle box sphere scale is not one half");
    constexpr uint32_t one=0x3f800000u;uint32_t out[4],squares[3];
    for(unsigned i=0;i<3;++i){out[i]=vuMul(vuAdd(high[i],low[i]),half);const auto h=vuMul(vuSub(high[i],low[i]),half);squares[i]=vuMul(h,h);}
    const auto length2=vuAdd(vuAdd(vuMul(one,squares[0]),vuMul(one,squares[1])),vuMul(one,squares[2]));
    uint32_t flags=0;const auto q=bozo::ps2f::rsqrt(one,length2,flags);out[3]=vuMul(length2,q);
    std::memcpy(record(rt,box+0x20,16),out,16);return true;
}
// Box pair separating-axis test, mod001 1f82c0 (called from the shape pair
// dispatch at 1f8964 once 1f8aa0's spheres overlap). It runs on the VU
// registers that block left: vf1..vf4 = object A rows, vf5..vf8 = object B
// rows, vf9 = (scale A, scale B). a0/a2 hold the box centres, a1/a3 the half
// extents. Transcribed from the listing's terms (ps2f mul, nearest-even add):
// R[i][j] = Ai.Bj and the T dots sum as ((1*x + 1*y) + 1*z); T = PB - PA with
// P = ((r0*c.x + r1*c.y) + r2*c.z) + r3*1. Fifteen axes in listing order; the
// EE code compares |T.L| <= radius (FPU compare) and returns 0 at the first
// separating axis, else 1. Some cross-axis radii use the listing's own |R|
// entries (not the textbook ones); they are kept.
uint32_t nativeBoxPairTest(const std::array<std::array<uint32_t,4>,8> &rows,const std::array<uint32_t,4> &scale,const VuWords &ca,const VuWords &ea,const VuWords &cb,const VuWords &eb){
    constexpr uint32_t one=0x3f800000u;
    const auto M=vuMul;const auto A=vuAdd;const auto S=vuSub;const auto abs=[](uint32_t x){return x&0x7fffffffu;};
    const auto dot=[&](const std::array<uint32_t,4> &u,const std::array<uint32_t,4> &v){return A(A(M(one,M(u[0],v[0])),M(one,M(u[1],v[1]))),M(one,M(u[2],v[2])));};
    std::array<uint32_t,4> pa{},pb{},t{};
    for(unsigned l=0;l<4;++l){pa[l]=A(A(A(M(rows[0][l],ca[0]),M(rows[1][l],ca[1])),M(rows[2][l],ca[2])),M(rows[3][l],one));
        pb[l]=A(A(A(M(rows[4][l],cb[0]),M(rows[5][l],cb[1])),M(rows[6][l],cb[2])),M(rows[7][l],one));}
    for(unsigned l=0;l<3;++l)t[l]=S(pb[l],pa[l]);
    uint32_t r[3][3],ar[3][3],ta[3],tb[3],eA[3],eB[3];
    for(unsigned i=0;i<3;++i){ta[i]=dot(rows[i],t);tb[i]=dot(rows[4+i],t);eA[i]=M(ea[i],scale[0]);eB[i]=M(eb[i],scale[1]);
        for(unsigned j=0;j<3;++j){r[i][j]=dot(rows[i],rows[4+j]);ar[i][j]=abs(r[i][j]);}}
    // The FPU compare: signed magnitude, zero and denormals equal to zero.
    const auto key=[](uint32_t x){const int64_t m=(x&0x7f800000u)?int64_t(x&0x7fffffffu):0;return (x&0x80000000u)?-m:m;};
    const auto separated=[&](uint32_t distance,uint32_t radius){return !(key(distance)<=key(radius));};
    for(unsigned i=0;i<3;++i)
        if(separated(abs(ta[i]),A(A(A(M(one,M(eB[0],ar[i][0])),M(one,M(eB[1],ar[i][1]))),M(one,eA[i])),M(one,M(eB[2],ar[i][2])))))return 0;
    for(unsigned j=0;j<3;++j)
        if(separated(abs(tb[j]),A(A(A(M(ar[0][j],eA[0]),M(ar[1][j],eA[1])),M(one,eB[j])),M(ar[2][j],eA[2]))))return 0;
    // Cross axes Ai x Bj, listing order: |r[p][j]*ta[q] - r[q][j]*ta[p]| against
    // (|r[ri1][j]|*eA[a1] + |r[ri2][j]|*eA[a2]) + (eB[b1]*|r[bi1][bj1]| + |r[bi2][bj2]|*eB[b2]),
    // every index as the listing reads it.
    struct Cross{unsigned p,q,j,ri1,a1,ri2,a2,b1,bi1,bj1,bi2,bj2,b2;};
    constexpr Cross cross[9]={
        {1,2,0, 2,1,1,2, 1,0,2,1,1,2},{1,2,1, 2,1,1,2, 0,0,2,0,0,2},{1,2,2, 2,1,1,2, 0,0,1,0,0,1},
        {2,0,0, 2,0,0,2, 1,1,2,1,1,2},{2,0,1, 2,0,0,2, 0,1,2,1,0,2},{2,0,2, 2,0,0,2, 0,1,1,1,0,1},
        {0,1,0, 1,0,0,1, 1,2,2,2,1,2},{0,1,1, 1,0,0,1, 0,2,2,2,0,2},{0,1,2, 1,0,0,1, 0,2,1,2,0,1}};
    for(const auto &x:cross){
        const auto distance=abs(S(M(r[x.p][x.j],ta[x.q]),M(r[x.q][x.j],ta[x.p])));
        const auto radius=A(A(M(ar[x.ri1][x.j],eA[x.a1]),M(ar[x.ri2][x.j],eA[x.a2])),A(M(eB[x.b1],ar[x.bi1][x.bj1]),M(ar[x.bi2][x.bj2],eB[x.b2])));
        if(separated(distance,radius))return 0;
    }
    return 1;
}
// Battle shape tests: mod028 4f5630 (an object s4 against battle shape s0 and its list s1)
// sets up the object-pair registers and calls mod001 1f73a0, which dispatches by shape kind
// to the sphere/sphere test 1f8840 or (after 248560 rebuilds a stale inverse +190 with
// 216cb0 / 216f50) the box/sphere test 1f8100. The registers are objectPairRegisters:
// vf1..vf4 = rows[0..3], vf5..vf8 = rows[4..7], vf9 = scale. All VU0 arithmetic is ps2f.
//   4f5754..4f57a4: vf10 = s1+10 (sphere), vf9 = s0+40 (scales, read before the store),
//     vf11 = s0 (centre, radius); vf12 = rows(s4+150) . vf10 - vf11; s0+40 =
//     (((dx^2+dy^2) + 1*dz^2) - (vf10.w*vf9.x + vf11.w)^2, dy^2, dz^2, w); rows[0..3] and
//     the scales are the registers from here on.
//   4f5800..4f580c: vf5..vf7 = s0+10..30, vf8 = vf0 = (0, 0, 0, 1): rows[4..7].
//   1f740c/1f7410 spill vf6/vf8 to sp+30/40 around 248560; 1f7424/1f7428 restore them;
//     1f88d0 does the same at sp+70 (1f8994..1f89b0) and sp+30 (1f8a1c..1f8a38).
//   248570..2485b4: basis lengths of a1+90 (0x7f0 into vf6), |len - 1| to sp+10.
//   216d34..216d5c (rigid inverse 216cb0, after its scalar transpose into a1): a1+30 =
//     -(a1 rows . a0+30) in xyz, w = a0+3c.
//   1f8844..1f889c: sphere a0 through rows[0..3], sphere a1 through rows[4..7]; sp+0.x =
//     ((1*dx^2 + 1*dy^2) - 1*(a0.w*s.x + a1.w*s.y)^2) + 1*dz^2.
//   1f8104..1f81c0: sphere a2 through rows[4..7] (a3 set, scale y) or rows[0..3] (scale x),
//     then through t0's rows into box space: sp+20 = (that, r^2 * sqrt(max column length^2
//     of t0)); box a0 +- a1 * the other object's scale: sp+0 (max) and sp+10 (min).
struct ShapeSite{uint32_t pc,word;};
constexpr ShapeSite battleShapeSites[]={
    {0x4f5754,0xd86a0000},{0x4f5758,0xda090040},{0x4f575c,0xda0b0000},{0x4f5760,0xd8810000},{0x4f5764,0xd8820010},{0x4f5768,0x4a295298},{0x4f576c,0xd8830020},{0x4f5770,0xd8840030},{0x4f5774,0x4bea09bc},{0x4f5778,0x4bea10bd},{0x4f577c,0x4bea18be},{0x4f5780,0x4be0230b},{0x4f5784,0x4bcb632c},{0x4f5788,0x4a2b52a8},{0x4f578c,0x4bcc632a},{0x4f5790,0x4a2a52aa},{0x4f5794,0x4b000343},{0x4f5798,0x4b0c603d},{0x4f579c,0x4b0c68be},{0x4f57a0,0x4b0a6b0f},{0x4f57a4,0xfa0c0040},
    {0x4f5800,0xda050010},{0x4f5804,0xda060020},{0x4f5808,0xda070030},{0x4f580c,0x4be8033c},
    {0x1f740c,0xf8460000},{0x1f7410,0xf8480010},{0x1f7424,0xd8460000},{0x1f7428,0xd8480010},
    {0x1f8994,0xf8460000},{0x1f8998,0xf8480010},{0x1f89ac,0xd8460000},{0x1f89b0,0xd8480010},
    {0x1f8a1c,0xf8460000},{0x1f8a20,0xf8480010},{0x1f8a34,0xd8460000},{0x1f8a38,0xd8480010},
    {0x248570,0xd84c0000},{0x248574,0xd84d0010},{0x248578,0xd84e0020},{0x248580,0x4a003fb8},{0x2485a4,0x48490001},{0x2485a8,0x4bc03187},{0x2485ac,0x4bc631fd},{0x2485b4,0xf8460000},
    {0x216d34,0xd89f0030},{0x216d38,0xd8bc0000},{0x216d3c,0xd8bd0010},{0x216d40,0xd8be0020},{0x216d44,0x4b0006c7},{0x216d48,0x4bdfe1bc},{0x216d4c,0x4bdfe8bd},{0x216d50,0x4bdff7ca},{0x216d54,0x4bdbffd8},{0x216d5c,0xf8bf0030},
    {0x1f8844,0xd88a0000},{0x1f8848,0xd8ab0000},{0x1f884c,0x4bea09bc},{0x1f8850,0x4bea10bd},{0x1f8854,0x4bea18be},{0x1f8858,0x4be0234b},{0x1f885c,0x4beb29bc},{0x1f8860,0x4beb30bd},{0x1f8864,0x4beb38be},{0x1f8868,0x4be0438b},{0x1f886c,0x4a295298},{0x1f8870,0x4a295ad9},{0x1f8874,0x4bce6b6c},{0x1f8878,0x4a2b52a8},{0x1f887c,0x4bcd6b6a},{0x1f8880,0x4b000383},{0x1f8884,0x4a2a52aa},{0x1f8888,0x4b0d71bc},{0x1f888c,0x4b0d70bd},{0x1f8890,0x4b0a70ff},{0x1f8894,0x4b0d72ca},{0x1f889c,0xf84b0000},
    {0x1f8104,0xd9100000},{0x1f8108,0xd9110010},{0x1f810c,0xd9120020},{0x1f8110,0xd9130030},{0x1f8118,0xd8cd0000},{0x1f811c,0x4bed29bc},{0x1f8120,0x4bed30bd},{0x1f8124,0x4bed38be},{0x1f8128,0x4be0438b},{0x1f8130,0x4a296b59},{0x1f8138,0x4bed09bc},{0x1f813c,0x4bed10bd},{0x1f8140,0x4bed18be},{0x1f8144,0x4be0238b},{0x1f8148,0x4a296b58},{0x1f8150,0x4bd082be},{0x1f8154,0x4bd18abd},{0x1f8158,0x4bd293e9},{0x1f815c,0x4b0f7bd1},{0x1f8160,0x4a2d6b6a},{0x1f8164,0x4b0f7bd2},{0x1f8168,0x4a0f03bd},{0x1f816c,0x4bee81bc},{0x1f8170,0x4bee88bd},{0x1f8174,0x4bee90be},{0x1f8178,0x4bc09b8b},{0x1f817c,0x4a0003bf},{0x1f8180,0x4a206b9c},{0x1f8188,0xf84e0000},{0x1f818c,0xd88f0000},{0x1f8194,0xd8b00000},{0x1f819c,0x4bc98418},{0x1f81a0,0x4bc98419},{0x1f81a8,0x4bd07c68},{0x1f81ac,0x4bd07cac},{0x1f81bc,0xf8510000},{0x1f81c0,0xf8720000}};
bool nativeBattleShapes(R5900Context *c,Runtime *rt,uint32_t pc){
    if((pc&3) || !rt || !((pc>=0x4f5754 && pc<=0x4f580c) || (pc>=0x1f740c && pc<=0x1f7428) || (pc>=0x1f8994 && pc<=0x1f8a38) || (pc>=0x248570 && pc<=0x2485b4) ||
       (pc>=0x216d34 && pc<=0x216d5c) || (pc>=0x1f8844 && pc<=0x1f889c) || (pc>=0x1f8104 && pc<=0x1f81c0)))return false;
    const auto site=std::find_if(std::begin(battleShapeSites),std::end(battleShapeSites),[&](auto &x){return x.pc==pc;});
    if(site==std::end(battleShapeSites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native battle shape test has no CPU record");
    using Words=std::array<uint32_t,4>;
    constexpr uint32_t one=0x3f800000u;
    const auto load=[&](uint32_t at){Words v;std::memcpy(v.data(),record(rt,at,16),16);return v;};
    const auto store=[&](uint32_t at,const Words &v){std::memcpy(record(rt,at,16),v.data(),16);};
    // rows . v: lane k = ((r0k*v.x + r1k*v.y) + r2k*v.z) + r3k*1 (MULAx, MADDAy, MADDAz, MADDw).
    const auto through=[&](const std::array<Words,4> &r,const Words &v){Words o;
        for(unsigned k=0;k<4;++k)o[k]=vuAdd(vuAdd(vuAdd(vuMul(r[0][k],v[0]),vuMul(r[1][k],v[1])),vuMul(r[2][k],v[2])),vuMul(r[3][k],one));return o;};
    auto &regs=objectPairRegisters(rt);
    const auto rows=[&](unsigned first){std::array<Words,4> r;for(unsigned i=0;i<4;++i)r[i]=regs.rows[first+i];return r;};
    const auto need=[&]{if(!regs.valid)throw std::runtime_error(std::format("native battle shape test at {:x} has no object-pair registers",pc));};
    switch(pc){
    case 0x4f57a4:{
        const auto object=c->r[20].uw[0],shape=c->r[16].uw[0];
        if(c->r[4].uw[0]!=object+0x150 || c->r[3].uw[0]!=c->r[17].uw[0]+0x10)
            throw std::runtime_error("native battle shape overlap escaped its records");
        const auto sphere=load(c->r[3].uw[0]),scale=load(shape+0x40),centre=load(shape);
        std::array<Words,4> m;for(unsigned i=0;i<4;++i)m[i]=load(object+0x150+16*i);
        auto t=through(m,sphere);for(unsigned i=0;i<3;++i){t[i]=vuSub(t[i],centre[i]);t[i]=vuMul(t[i],t[i]);}
        auto radius=vuAdd(vuMul(sphere[3],scale[0]),centre[3]);radius=vuMul(radius,radius);
        t[0]=vuSub(vuAdd(vuAdd(t[0],t[1]),vuMul(one,t[2])),vuMul(one,radius));
        for(unsigned i=0;i<4;++i)regs.rows[i]=m[i];regs.scale=scale;regs.valid=true;
        store(shape+0x40,t);return true;}
    case 0x4f580c:{
        need();const auto shape=c->r[16].uw[0];
        for(unsigned i=0;i<3;++i)regs.rows[4+i]=load(shape+0x10+16*i);regs.rows[7]={0,0,0,one};return true;}
    case 0x1f740c:case 0x1f7410:case 0x1f8994:case 0x1f8998:case 0x1f8a1c:case 0x1f8a20:
    case 0x1f7424:case 0x1f7428:case 0x1f89ac:case 0x1f89b0:case 0x1f8a34:case 0x1f8a38:{
        // vf6 / vf8 around 248560 (its 0x7f0 call writes vf6): 1f73a0 at sp+30, 1f88d0 at
        // sp+70 (second object box) and sp+30 (first object box).
        need();const bool spill=(site->word>>26)==0x3e,eight=(site->word&0xffff)==0x10;
        if(c->r[2].uw[0]!=c->r[29].uw[0]+(pc==0x1f8994 || pc==0x1f8998 || pc==0x1f89ac || pc==0x1f89b0?0x70u:0x30u))
            throw std::runtime_error("native battle shape spill escaped its frame");
        if(spill)store(c->r[2].uw[0]+(eight?0x10:0),regs.rows[eight?7:5]);else regs.rows[eight?7:5]=load(c->r[2].uw[0]+(eight?0x10:0));
        return true;}
    case 0x2485a4:c->r[9].ud[0]=0;return true;   // CFC2.I t1, vi0: the constant zero
    case 0x2485b4:{
        const auto object=c->r[5].uw[0],out=c->r[2].uw[0];
        if(out!=c->r[29].uw[0]+0x10)throw std::runtime_error("native battle shape scale check escaped its frame");
        objectBasisLengths(rt,object,out);uint32_t lengths[3];std::memcpy(lengths,record(rt,out,16),12);
        for(auto &v:lengths)v=vuSub(v,one)&0x7fffffffu;std::memcpy(record(rt,out,16),lengths,12);return true;}
    case 0x216d5c:{
        const auto in=load(c->r[4].uw[0]+0x30),r0=load(c->r[5].uw[0]),r1=load(c->r[5].uw[0]+0x10),r2=load(c->r[5].uw[0]+0x20);
        const uint32_t minusOne=vuSub(0,one);Words t;
        for(unsigned k=0;k<3;++k)t[k]=vuMul(vuAdd(vuAdd(vuMul(r0[k],in[0]),vuMul(r1[k],in[1])),vuMul(r2[k],in[2])),minusOne);
        t[3]=in[3];store(c->r[5].uw[0]+0x30,t);return true;}
    case 0x1f889c:{
        need();if(c->r[2].uw[0]!=c->r[29].uw[0])throw std::runtime_error("native battle sphere pair escaped its frame");
        const auto a=load(c->r[4].uw[0]),b=load(c->r[5].uw[0]);
        auto ta=through(rows(0),a);const auto tb=through(rows(4),b);
        auto bw=vuMul(b[3],regs.scale[1]);auto radius=vuAdd(vuMul(a[3],regs.scale[0]),bw);radius=vuMul(radius,radius);
        for(unsigned i=0;i<3;++i){ta[i]=vuSub(ta[i],tb[i]);ta[i]=vuMul(ta[i],ta[i]);}
        Words out=b;out[3]=bw;
        out[0]=vuAdd(vuSub(vuAdd(vuMul(one,ta[0]),vuMul(one,ta[1])),vuMul(one,radius)),vuMul(one,ta[2]));
        store(c->r[29].uw[0],out);return true;}
    case 0x1f8188:{
        need();if(c->r[2].uw[0]!=c->r[29].uw[0]+0x20)throw std::runtime_error("native battle box/sphere escaped its frame");
        const bool second=c->r[7].uw[0]!=0;const auto sphere=load(c->r[6].uw[0]);
        std::array<Words,4> m;for(unsigned i=0;i<4;++i)m[i]=load(c->r[8].uw[0]+16*i);
        auto t=through(rows(second?4:0),sphere);
        auto r2=vuMul(sphere[3],regs.scale[second?1:0]);r2=vuMul(r2,r2);
        uint32_t column[3];for(unsigned k=0;k<3;++k)column[k]=vuAdd(vuAdd(vuMul(m[0][k],m[0][k]),vuMul(m[1][k],m[1][k])),vuMul(m[2][k],m[2][k]));
        const auto widest=bozo::ps2f::max(bozo::ps2f::max(column[0],column[1]),column[2]);
        uint32_t flags=0;const auto q=bozo::ps2f::sqrt(widest,flags);
        Words out;for(unsigned k=0;k<3;++k)out[k]=vuAdd(vuAdd(vuAdd(vuMul(m[0][k],t[0]),vuMul(m[1][k],t[1])),vuMul(m[2][k],t[2])),vuMul(m[3][k],one));
        out[3]=vuMul(r2,q);store(c->r[29].uw[0]+0x20,out);return true;}
    case 0x1f81bc:case 0x1f81c0:{
        need();const auto stack=c->r[29].uw[0];
        if(c->r[2].uw[0]!=stack || c->r[3].uw[0]!=stack+0x10)throw std::runtime_error("native battle box/sphere bounds escaped their frame");
        const bool second=c->r[7].uw[0]!=0;const auto centre=load(c->r[4].uw[0]),extent=load(c->r[5].uw[0]);
        const auto m=load(c->r[8].uw[0]+(pc==0x1f81bc?0x10:0x20));   // vf17 / vf18 keep the t0 row's w
        Words out=centre;out[3]=m[3];
        for(unsigned k=0;k<3;++k){const auto e=vuMul(extent[k],regs.scale[second?0:1]);out[k]=pc==0x1f81bc?vuAdd(centre[k],e):vuSub(centre[k],e);}
        store(pc==0x1f81bc?stack:stack+0x10,out);return true;}
    default:return true;   // loads and intermediate steps: the results are computed at the stores
    }
}
// Battle shape tree, mod028 4a6470(node s5, a1, a2; recursive over the node's children +0 ..
// +20): SQC2 vf1..vf5 to sp+90 on entry (4a64e4..4a64f4), loads vf5 = sp+80 (the shape's
// radius +204 and a global scale) and vf1..vf4 = s0+150 (4a664c..4a665c) for main's
// segment/sphere test 1f59c0 and 1f5ba0, and LQC2 vf1..vf5 back from sp+90 on its exits
// (4a6770.., 4a67a0..). vf1..vf5 are the object-pair registers' rows 0..3 and 4: the spill
// keeps them per frame and the restore gives them back (a child's call leaves its parent's).
struct ShapeTreeSpill{std::array<std::array<uint32_t,4>,5> rows{};bool valid=false;};
std::map<std::pair<const Runtime *,uint32_t>,ShapeTreeSpill> &shapeTreeSpills(){static std::map<std::pair<const Runtime *,uint32_t>,ShapeTreeSpill> m;return m;}
// mod001 1f5c30 is the same walk (the field's original; 4a6470 is the battle's compile): spill
// to sp+70 (1f5c8c), vf5 = sp+60 (1f5df0, v0), vf1..vf4 = s0+150 (1f5df4.., v1), restores at
// 1f5e1c, 1f5f64, 1f5fcc. Both instances are this table.
struct ShapeTreeSite{uint32_t spill,frame,scale,scaleReg,scaleOffset,rows,rowsReg;std::array<uint32_t,3> restores;};
constexpr ShapeTreeSite shapeTreeSites[]={{0x4a64e4,0x90,0x4a664c,3,0x80,0x4a6650,2,{0x4a6770,0x4a67a0,0}},{0x1f5c8c,0x70,0x1f5df0,2,0x60,0x1f5df4,3,{0x1f5e1c,0x1f5f64,0x1f5fcc}}};
bool nativeBattleShapeTree(R5900Context *c,Runtime *rt,uint32_t pc){
    if((pc&3) || !rt)return false;
    const ShapeTreeSite *site=nullptr;bool spill=false,restore=false,scale=false,rows=false;unsigned index=0;
    for(const auto &t:shapeTreeSites){
        if(pc>=t.spill && pc<t.spill+20){site=&t;spill=true;index=(pc-t.spill)/4;break;}
        if(pc==t.scale){site=&t;scale=true;break;}
        if(pc>=t.rows && pc<t.rows+16){site=&t;rows=true;index=(pc-t.rows)/4;break;}
        for(auto r:t.restores)if(r && pc>=r && pc<r+20){site=&t;restore=true;index=(pc-r)/4;}
        if(site)break;
    }
    if(!site)return false;
    const uint32_t expected=spill?0xf8400000u|(index+1)<<16|index*16:restore?0xd8400000u|(index+1)<<16|index*16:
        scale?0xd8050000u|site->scaleReg<<21:0xd8000000u|site->rowsReg<<21|(index+1)<<16|index*16;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=expected)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native shape tree has no CPU record");
    const auto stack=c->r[29].uw[0];auto &regs=objectPairRegisters(rt);
    if(spill || restore){
        if(c->r[2].uw[0]!=stack+site->frame)throw std::runtime_error("native shape tree spill escaped its frame");
        const auto key=std::make_pair(static_cast<const Runtime *>(rt),stack);
        if(spill && index==0){auto &saved=shapeTreeSpills()[key];for(unsigned i=0;i<5;++i)saved.rows[i]=regs.rows[i];saved.valid=regs.valid;}
        if(restore && index==4){const auto it=shapeTreeSpills().find(key);if(it==shapeTreeSpills().end())throw std::runtime_error("native shape tree restore without its spill");
            for(unsigned i=0;i<5;++i)regs.rows[i]=it->second.rows[i];regs.valid=it->second.valid;shapeTreeSpills().erase(it);}
        return true;
    }
    const auto node=c->r[16].uw[0];
    if(scale){if(c->r[site->scaleReg].uw[0]!=stack+site->scaleOffset)throw std::runtime_error("native shape tree scale escaped its frame");
        std::memcpy(regs.rows[4].data(),record(rt,stack+site->scaleOffset,16),16);regs.valid=false;return true;}
    if(c->r[site->rowsReg].uw[0]!=node+0x150)throw std::runtime_error("native shape tree rows escaped their record");
    std::memcpy(regs.rows[index].data(),record(rt,node+0x150+16*index,16),16);
    if(index==3)regs.valid=true;
    return true;
}
// Segment against battle shapes: mod028 4f58f0 loads vf5 = s3+20 (x: the object's scale
// +204, y: s3+24) and vf1..vf4 = the rows of s2+150 (4f5ab8..4f5ac8: objectPairRegisters
// rows[4] and rows[0..3]) and calls mod001 1f5ba0 per segment; kind 1 shapes run the
// segment/sphere test 1f59c0(a0 sphere, a1 start p0, a2 end p1), ps2f in program order:
//   c = rows . a0 (xyzw), r = a0.w*vf5.x + 1*vf5.y; d = p1 - p0;
//   a = (d.x^2+d.y^2) + 1*d.z^2; b = (2(d*(p0-c))).x + .y + 1*.z;
//   k = ((((c.x^2+c.y^2) + 1*c.z^2) + 1*p0.x^2 + 1*p0.y^2 + 1*p0.z^2) - 1*(2 p0.c)) - 1*r^2;
//   disc = b*b - a*k - a*k - a*k - k*a (four MSUBs); sp+20.x = disc;
//   m = -1 * (1/(2a)); s = sqrt(disc); sp+0.x = m*b + m*s; sp+10.x = m*b - m*s.
// Only the x lanes are defined (the rest are register leftovers no reader takes).
bool nativeSegmentSphere(R5900Context *c,Runtime *rt,uint32_t pc){
    constexpr std::pair<uint32_t,uint32_t> sites[]={{0x1f59c4,0xd8860000},{0x1f59c8,0xd8a90000},{0x1f59cc,0xd8ca0000},{0x1f59d0,0x4bc00747},{0x1f59d4,0x4be609bc},{0x1f59d8,0x4be610bd},{0x1f59dc,0x4be618be},{0x1f59e0,0x4be021cb},{0x1f59e4,0x4a2531bc},{0x1f59e8,0x4a2501c9},{0x1f59ec,0x4bc9522c},{0x1f59f0,0x4bc74d2c},{0x1f59f4,0x4bc74daa},{0x1f59f8,0x4bc007c3},{0x1f59fc,0x4bc73dea},{0x1f5a00,0x4a2739ea},{0x1f5a04,0x4bc842ea},{0x1f5a08,0x4bd4456a},{0x1f5a0c,0x4b16b03d},{0x1f5a10,0x4b16fd8a},{0x1f5a14,0x4b0b583d},{0x1f5a18,0x4bd5ad68},{0x1f5a1c,0x4b0bfaca},{0x1f5a20,0x4b16b5a8},{0x1f5a24,0x4bc94e2a},{0x1f5a28,0x4b15a83d},{0x1f5a2c,0x4b0b5be8},{0x1f5a30,0x4b15fb0a},{0x1f5a34,0x4b17b83d},{0x1f5a38,0x4b17f8be},{0x1f5a3c,0x4a6f03bc},{0x1f5a40,0x4b18f8bc},{0x1f5a44,0x4b18f8bd},{0x1f5a48,0x4b18f8be},{0x1f5a4c,0x4b16f8fc},{0x1f5a50,0x4b07fb4f},{0x1f5a54,0x4b0c62be},{0x1f5a58,0x4b0d5afd},{0x1f5a5c,0x4b0d5afd},{0x1f5a60,0x4b0d5afd},{0x1f5a64,0x4b0b6bad},{0x1f5a68,0x4b00ebdc},{0x1f5a70,0x4a0e03bd},{0x1f5a74,0xf84e0000},
        {0x1f5ae0,0x4b0c7abe},{0x1f5ae4,0x4b007c61},{0x1f5ae8,0x4b0c7abe},{0x1f5aec,0x4b007ca5},{0x1f5af0,0xf8710000},{0x1f5af4,0xf8520000},
        {0x4f5ab8,0xd8850000},{0x4f5abc,0xd8610000},{0x4f5ac0,0xd8620010},{0x4f5ac4,0xd8630020},{0x4f5ac8,0xd8640030},
        {0x4a562c,0xd8610000},{0x4a5630,0xd8620010},{0x4a5634,0xd8630020},{0x4a5638,0xd8640030},{0x4a563c,0x48a82800}};
    if((pc&3) || !rt || !((pc>=0x1f59c4 && pc<=0x1f5af4) || (pc>=0x4f5ab8 && pc<=0x4f5ac8) || (pc>=0x4a562c && pc<=0x4a563c)))return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](auto &x){return x.first==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->second)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native segment test has no CPU record");
    using Words=std::array<uint32_t,4>;
    const auto load=[&](uint32_t at){Words v;std::memcpy(v.data(),record(rt,at,16),16);return v;};
    auto &regs=objectPairRegisters(rt);
    if(pc==0x4f5ac8){
        if(c->r[4].uw[0]!=c->r[19].uw[0]+0x20 || c->r[3].uw[0]!=c->r[18].uw[0]+0x150)throw std::runtime_error("native segment registers escaped their records");
        regs.rows[4]=load(c->r[4].uw[0]);for(unsigned i=0;i<4;++i)regs.rows[i]=load(c->r[3].uw[0]+16*i);regs.valid=true;return true;
    }
    // mod028 4a55e0 (after the nearest-point walk 4a50b0 picked *(5136c8)): vf1..vf4 = that object's
    // rows (v1 = +150, 4a562c..4a5638), then QMTC2 t0 -> vf5 (4a563c): the whole 128-bit register,
    // x the object's scale +204 (mfc1), y its sign extension; 1f5ba0's tests read x and y only.
    if(pc==0x4a563c){
        uint32_t object;std::memcpy(&object,record(rt,0x5136c0,16)+8,4);
        if(c->r[3].uw[0]!=object+0x150)throw std::runtime_error("native segment registers escaped the picked object's rows");
        for(unsigned i=0;i<4;++i)regs.rows[i]=load(c->r[3].uw[0]+16*i);for(unsigned i=0;i<4;++i)regs.rows[4][i]=c->r[8].uw[i];regs.valid=true;return true;
    }
    if(pc>=0x4a562c && pc<0x4a563c)return true;
    if(pc!=0x1f5a74 && pc!=0x1f5af0 && pc!=0x1f5af4)return true;
    if(!regs.valid)throw std::runtime_error("native segment/sphere test has no registers");
    constexpr uint32_t one=0x3f800000u;
    const auto v6=load(c->r[4].uw[0]),p0=load(c->r[5].uw[0]),p1=load(c->r[6].uw[0]);const auto &s5=regs.rows[4];
    Words centre;for(unsigned k=0;k<4;++k)centre[k]=vuAdd(vuAdd(vuAdd(vuMul(regs.rows[0][k],v6[0]),vuMul(regs.rows[1][k],v6[1])),vuMul(regs.rows[2][k],v6[2])),vuMul(regs.rows[3][k],one));
    auto radius=vuAdd(vuMul(v6[3],s5[0]),vuMul(one,s5[1]));radius=vuMul(radius,radius);
    uint32_t d[3],toward[3],product[3],c2[3],d2[3],mixed[3],p2[3];
    for(unsigned i=0;i<3;++i){d[i]=vuSub(p1[i],p0[i]);toward[i]=vuSub(p0[i],centre[i]);product[i]=vuMul(p0[i],centre[i]);c2[i]=vuMul(centre[i],centre[i]);
        d2[i]=vuMul(d[i],d[i]);mixed[i]=vuMul(d[i],toward[i]);mixed[i]=vuAdd(mixed[i],mixed[i]);p2[i]=vuMul(p0[i],p0[i]);}
    auto pc2=vuAdd(vuAdd(product[0],product[1]),vuMul(one,product[2]));pc2=vuAdd(pc2,pc2);
    const auto a=vuAdd(vuAdd(d2[0],d2[1]),vuMul(one,d2[2]));const auto twiceA=vuAdd(a,a);
    const auto b=vuAdd(vuAdd(mixed[0],mixed[1]),vuMul(one,mixed[2]));
    uint32_t flags=0;const auto inverse=bozo::ps2f::div(one,twiceA,flags);
    auto acc=vuAdd(vuAdd(c2[0],c2[1]),vuMul(one,c2[2]));
    for(unsigned i=0;i<3;++i)acc=vuAdd(acc,vuMul(one,p2[i]));
    acc=vuSub(acc,vuMul(one,pc2));const auto k=vuSub(acc,vuMul(one,radius));
    acc=vuMul(b,b);for(unsigned i=0;i<3;++i)acc=vuSub(acc,vuMul(a,k));const auto disc=vuSub(acc,vuMul(k,a));
    if(pc==0x1f5a74){if(c->r[2].uw[0]!=c->r[29].uw[0]+0x20)throw std::runtime_error("native segment/sphere escaped its frame");
        std::memcpy(record(rt,c->r[2].uw[0],16),&disc,4);return true;}
    const auto m=vuMul(vuSub(0,one),inverse);flags=0;const auto root=bozo::ps2f::sqrt(disc,flags);
    const auto base=vuMul(m,b),offset=vuMul(m,root);
    const auto out=pc==0x1f5af0?vuAdd(base,offset):vuSub(base,offset);const auto at=c->r[pc==0x1f5af0?3:2].uw[0];
    if(at!=c->r[29].uw[0]+(pc==0x1f5af0?0u:0x10u))throw std::runtime_error("native segment/sphere roots escaped their frame");
    std::memcpy(record(rt,at,16),&out,4);return true;
}
// Segment/box, mod001 1f5640(a0 box centre, a1 half extents, a2 p0, a3 p1) on the segment
// registers (vf1..vf4 = rows[0..3], vf5 = rows[4]; see nativeSegmentSphere), ps2f:
//   c = rows . a0 (xyzw); sp+80 = a1 + vf5.y (xyz; w = a1.w);
//   sp+70 = (p0 - c) against each row: x = (r0x+r0y) + 1*r0z, y = (r1y+r1x) + 1*r1z,
//   z = (r2z+r2x) + 1*r2y of the lane products; sp+60 the same for p1 - p0.
// Only xyz of sp+70 / sp+60 are defined (w: register leftovers, not read). The slab
// tests after it are scalar (1f6670).
bool nativeBattleSegmentBox(R5900Context *c,Runtime *rt,uint32_t pc){
    constexpr std::pair<uint32_t,uint32_t> sites[]={{0x1f565c,0xd88f0000},{0x1f5660,0xd8cc0000},{0x1f5664,0xd8eb0000},{0x1f5668,0xd8b10000},{0x1f566c,0x4bef09bc},{0x1f5670,0x4bef10bd},{0x1f5674,0x4bef18be},{0x1f5678,0x4be023cb},{0x1f567c,0x4bcf62ac},{0x1f5680,0x4bc58c41},{0x1f56ac,0xf8510000},{0x1f56b0,0x4bcc5aec},{0x1f56b4,0x4bc1552a},{0x1f56b8,0x4bc2556a},{0x1f56bc,0x4bc355aa},{0x1f56c0,0x4bc007c3},{0x1f56c4,0x4b14a03d},{0x1f56c8,0x4b14fb8a},{0x1f56cc,0x4a95a83c},{0x1f56d0,0x4a95fb8a},{0x1f56d4,0x4bc15d2a},{0x1f56d8,0x4a56b03c},{0x1f56dc,0x4a56fb89},{0x1f56e0,0x4bc25d6a},{0x1f56e4,0x4bc35daa},{0x1f56e8,0x4b14a03d},{0x1f56ec,0x4b14fb4a},{0x1f56f0,0x4a95a83c},{0x1f56f4,0x4a95fb4a},{0x1f56f8,0x4a56b03c},{0x1f56fc,0x4a56fb49},{0x1f5708,0xf86e0000},{0x1f570c,0xf84d0000}};
    if((pc&3) || !rt || pc<0x1f565c || pc>0x1f570c)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](auto &x){return x.first==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->second)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native segment/box test has no CPU record");
    if(pc!=0x1f56ac && pc!=0x1f5708 && pc!=0x1f570c)return true;
    auto &regs=objectPairRegisters(rt);if(!regs.valid)throw std::runtime_error("native segment/box test has no registers");
    using Words=std::array<uint32_t,4>;
    const auto load=[&](uint32_t at){Words v;std::memcpy(v.data(),record(rt,at,16),16);return v;};
    constexpr uint32_t one=0x3f800000u;const auto stack=c->r[29].uw[0];
    if(pc==0x1f56ac){
        if(c->r[2].uw[0]!=stack+0x80)throw std::runtime_error("native segment/box extents escaped their frame");
        auto e=load(c->r[5].uw[0]);for(unsigned i=0;i<3;++i)e[i]=vuAdd(e[i],regs.rows[4][1]);std::memcpy(record(rt,stack+0x80,16),e.data(),16);return true;
    }
    const bool start=pc==0x1f5708;const auto out=c->r[start?3:2].uw[0];
    if(out!=stack+(start?0x70u:0x60u))throw std::runtime_error("native segment/box point escaped its frame");
    const auto centre=load(c->r[4].uw[0]),p0=load(c->r[6].uw[0]),p1=load(c->r[7].uw[0]);
    Words world;for(unsigned k=0;k<4;++k)world[k]=vuAdd(vuAdd(vuAdd(vuMul(regs.rows[0][k],centre[0]),vuMul(regs.rows[1][k],centre[1])),vuMul(regs.rows[2][k],centre[2])),vuMul(regs.rows[3][k],one));
    uint32_t v[3];for(unsigned i=0;i<3;++i)v[i]=start?vuSub(p0[i],world[i]):vuSub(p1[i],p0[i]);
    uint32_t result[3];
    for(unsigned r=0;r<3;++r){uint32_t m[3];for(unsigned i=0;i<3;++i)m[i]=vuMul(v[i],regs.rows[r][i]);
        constexpr unsigned order[3][3]={{0,1,2},{1,0,2},{2,0,1}};
        result[r]=vuAdd(vuAdd(m[order[r][0]],m[order[r][1]]),vuMul(one,m[order[r][2]]));}
    std::memcpy(record(rt,out,16),result,12);return true;
}
bool nativeBoxPairLibrary(R5900Context *c,Runtime *rt,uint32_t target){
    if(target!=0x1f82c0)return false;
    if(!c || !rt || c->pc!=target || c->r[31].uw[0]!=0x1f896c)throw std::runtime_error("native box pair test requires its shape-pair caller");
    const auto &regs=objectPairRegisters(rt);
    if(!regs.valid)throw std::runtime_error("native box pair test has no object-pair registers");
    const auto ca=vuWords(rt,c->r[4].uw[0]),ea=vuWords(rt,c->r[5].uw[0]),cb=vuWords(rt,c->r[6].uw[0]),eb=vuWords(rt,c->r[7].uw[0]);
    const auto saved=_mm_getcsr();const auto result=nativeBoxPairTest(regs.rows,regs.scale,ca,ea,cb,eb);_mm_setcsr(saved);
    c->r[2].sd[0]=result;c->pc=c->r[31].uw[0];return true;
}
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
// PS2-word lanes for the chain owner 1fd110 and its capsule pair 1fc300 (see
// Ps2Word): the same orders as leafRecord/leafTransform, for every word.
using Ps2Lanes=std::array<Ps2Word,4>;
Ps2Lanes ps2Record(Runtime *rt,uint32_t at){Ps2Lanes v;const auto *p=record(rt,at,16);for(unsigned i=0;i<4;++i)v[i]=Ps2Word::load(p+4*i);return v;}
void ps2Publish(Runtime *rt,uint32_t at,const Ps2Lanes &v,unsigned lanes=4){auto *p=record(rt,at,16);for(unsigned i=0;i<lanes;++i)v[i].store(p+4*i);}
struct Ps2Rows{Ps2Lanes rows[4];};
Ps2Rows ps2Rows(Runtime *rt,uint32_t at,unsigned n){Ps2Rows m;for(unsigned i=0;i<4;++i)m.rows[i]=i<n?ps2Record(rt,at+16*i):Ps2Lanes{};return m;}
Ps2Lanes ps2Transform(const Ps2Rows &m,const Ps2Lanes &v){Ps2Lanes out;
    for(unsigned i=0;i<4;++i)out[i]=((m.rows[0][i]*v[0]+m.rows[1][i]*v[1])+m.rows[2][i]*v[2])+m.rows[3][i]*kPs2One;return out;}
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
// mod003 3e6d30 (block entry 3e7104): after a collision-tree hit (v0 == 1 from
// the box/sphere follow-up 1f6740 / 3e6c20), the point at SP+160 goes through
// VF1..VF4, still the hit object's matrix (358cd8, as for 1f6740), in place:
// ((R1*x + R2*y) + R3*z) + R4*1 on all four lanes. LQC2 vf10 3e7158, four ACC
// ops, SQC2 vf11 3e716c. The point is read at the store (only the arithmetic
// sits between).
bool nativeHitPointTransform(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc<0x3e7158 || pc>0x3e716c || (pc&3))return false;
    if(!c)throw std::runtime_error("native hit-point transform has no CPU record");
    const auto point=c->r[29].uw[0]+0x160;
    if(c->r[3].uw[0]!=point)throw std::runtime_error("native hit-point transform frame mismatch");
    if(pc!=0x3e716c)return true;
    const auto in=leafInputs(rt,false);const auto v=leafRecord(rt,point);Lanes out;
    {music::NearestRounding rounding;out=leafTransform(in,v);}
    for(float x:out)if(!std::isfinite(x))throw std::runtime_error("native hit-point transform overflow");
    std::memcpy(record(rt,point,16),out.data(),16);return true;
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
    // PS2 words (see Ps2Word): every word is a VU number, degenerate vectors
    // follow RSQRT/DIV (+/-max) as the VU does.
    if(multiply){
        if(c->r[2].uw[0]!=stack+0x160 || c->r[3].uw[0]!=c->r[17].uw[0]+0x30)throw std::runtime_error("native bone quaternion product escaped its records");
        if(pc!=0x1fe6c0)return true;
        const auto q1=ps2Record(rt,stack+0x160),q2=ps2Record(rt,c->r[17].uw[0]+0x30);Ps2Lanes out;
        const Ps2Word cross[3]={q1[1]*q2[2]-q2[1]*q1[2],q1[2]*q2[0]-q2[2]*q1[0],q1[0]*q2[1]-q2[0]*q1[1]};
        for(unsigned i=0;i<3;++i)out[i]=(q2[i]*q1[3]+q1[i]*q2[3])+cross[i]*kPs2One;
        Ps2Word p[4];for(unsigned i=0;i<4;++i)p[i]=q1[i]*q2[i];
        out[3]=((p[3]-p[2])-kPs2One*p[1])-kPs2One*p[0];
        ps2Publish(rt,stack+0x160,out);return true;
    }
    const auto partnerAt=c->r[30].uw[0]+0x38;if((partnerAt&3) || (partnerAt&0x1ffffff)<0x100000)throw std::runtime_error("native bone aim partner record outside EE RAM");
    uint32_t partner;std::memcpy(&partner,rt->memory.hostPointer(partnerAt,4),4);
    if(pc==0x1fe598 && (c->r[5].uw[0]!=stack+0x180 || c->r[4].uw[0]!=stack+0x1c0 || c->r[3].uw[0]!=partner+0x30))throw std::runtime_error("native bone aim sources mismatch");
    if(pc!=0x1fe63c && pc!=0x1fe680)return true;
    if(pc==0x1fe63c?c->r[3].uw[0]!=stack+0x170:c->r[2].uw[0]!=stack+0x160)throw std::runtime_error("native bone aim destination mismatch");
    const auto m=ps2Rows(rt,stack+0x180,4);const auto a=ps2Record(rt,stack+0x1c0),b=ps2Record(rt,partner+0x30);
    const auto unit=[&](const Ps2Lanes &v){const auto t=ps2Transform(m,v);
        const auto q=Ps2Word::reciprocalRoot(kPs2One,(t[0]*t[0]+t[1]*t[1])+kPs2One*(t[2]*t[2]));
        Ps2Lanes u{};for(unsigned i=0;i<3;++i)u[i]=t[i]*q;return u;};
    const auto ua=unit(a),ub=unit(b);
    const auto dot=(ua[0]*ub[0]+ua[1]*ub[1])+ua[2]*ub[2];
    if(pc==0x1fe63c){dot.store(record(rt,stack+0x170,16));return true;}
    const Ps2Word cross[3]={ub[1]*ua[2]-ua[1]*ub[2],ub[2]*ua[0]-ua[2]*ub[0],ub[0]*ua[1]-ua[0]*ub[1]};
    const Ps2Word two{0x40000000u};const auto q=kPs2One*((kPs2Zero+two)+dot*two).root(),reciprocal=kPs2One/q;
    Ps2Lanes quaternion;for(unsigned i=0;i<3;++i)quaternion[i]=cross[i]*reciprocal;quaternion[3]=q*Ps2Word{0x3f000000u};
    ps2Publish(rt,stack+0x160,quaternion);return true;
}
// mod001 248a00 (a chain solver: per link, aim it from P toward T; called from mod068 527d54),
// three VU0 blocks, ps2f in listing order:
//  248b84..248c24  rows M = sp+e0, P = sp+140, T = (s4): v10 = M.P + r4*1, v11 = M.T + r4*1 (vf0.w,
//                  not the points' w); u = v10 * RSQRT(|v10|^2), v = v11 * RSQRT(|v11|^2);
//                  dot = (u.x v.x + u.y v.y) + u.z v.z through MR32 rotations (their other lanes
//                  read registers this function never wrote) stored to sp+d0 (x only: the EE code
//                  reads x; the other lanes are left as they are); cross = v x u (OPMULA/OPMSUB).
//  248c38..248c50  (dot < 0x3f7ffffe) axis = cross * RSQRT(|cross|^2) to sp+c0 (xyz: its w lane is
//                  the angle the EE code stores after it).
//  248c78..248cac  q1 = sp+b0 (215640's rotation), q2 = s2+30: xyz = (q2*q1.w + q1*q2.w) + (q1 x q2)*1,
//                  w = ((w1w2 - z1z2) - 1*y1y2) - 1*x1x2, to sp+b0.
struct ChainAim{Ps2Lanes cross{};uint32_t frame=0;bool aimed=false;};
std::map<const Runtime *,ChainAim> &chainAims(){static std::map<const Runtime *,ChainAim> m;return m;}
bool nativeChainAim(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Site{uint32_t pc,word;};
    constexpr Site sites[]={{0x248b84,0xd8610000},{0x248b88,0xd8620010},{0x248b8c,0xd8630020},{0x248b90,0xd8640030},{0x248b94,0xd8450000},{0x248b98,0xda860000},{0x248b9c,0x4be007d3},{0x248ba0,0x4be509bc},{0x248ba4,0x4be510bd},{0x248ba8,0x4be518be},{0x248bac,0x4be0228b},{0x248bb0,0x4be609bc},{0x248bb4,0x4be610bd},{0x248bb8,0x4be618be},{0x248bbc,0x4bca532a},{0x248bc0,0x4be022cb},{0x248bc4,0x4b0c603d},{0x248bc8,0x4b0cfb4a},{0x248bcc,0x4bcb5baa},{0x248bd0,0x4a6d03be},{0x248bd4,0x4b0e703d},{0x248bd8,0x4b0efbca},{0x248be8,0x4a0003bf},{0x248bec,0x4bc0541c},{0x248bf0,0x4a0002ff},{0x248bf4,0x4a0002ff},{0x248bf8,0x4a6f03be},{0x248bfc,0x4bd2833d},{0x248c00,0x4bd3933d},{0x248c04,0x4a0003bf},{0x248c08,0x4bc05c5c},{0x248c0c,0x4b1182be},{0x248c10,0x4b1190bd},{0x248c14,0x4b119cca},{0x248c18,0x4bd08afe},{0x248c1c,0x4bd1852e},{0x248c24,0xf8530000},
        {0x248c38,0x4bd4a2aa},{0x248c3c,0x4b0a503d},{0x248c40,0x4b0afaca},{0x248c44,0x4a6b03be},{0x248c48,0x4a0003bf},{0x248c4c,0x4bc0a31c},{0x248c50,0xf84c0000},
        {0x248c78,0xd8410000},{0x248c7c,0xd8620000},{0x248c80,0x4be00153},{0x248c88,0x4bc20afe},{0x248c8c,0x4bc110ee},{0x248c90,0x4be2092a},{0x248c94,0x4bc111bf},{0x248c98,0x4bc208bf},{0x248c9c,0x4bc0184b},{0x248ca0,0x4a24207e},{0x248ca4,0x4a2428fd},{0x248ca8,0x4a24284c},{0x248cac,0xf8410000}};
    if((pc&3) || !rt || pc<0x248b84 || pc>0x248cac)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](const Site &x){return x.pc==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native chain aim has no CPU record");
    const auto stack=c->r[29].uw[0];auto &st=chainAims()[rt];
    const auto unitOf=[](const Ps2Lanes &t){
        const auto q=Ps2Word::reciprocalRoot(kPs2One,(t[0]*t[0]+t[1]*t[1])+kPs2One*(t[2]*t[2]));Ps2Lanes u{};for(unsigned i=0;i<3;++i)u[i]=t[i]*q;return u;};
    if(pc==0x248c24){
        if(c->r[2].uw[0]!=stack+0xd0)throw std::runtime_error("native chain aim dot destination mismatch");
        const auto m=ps2Rows(rt,stack+0xe0,4);const auto u=unitOf(ps2Transform(m,ps2Record(rt,stack+0x140))),v=unitOf(ps2Transform(m,ps2Record(rt,c->r[20].uw[0])));
        const auto dot=(u[0]*v[0]+u[1]*v[1])+u[2]*v[2];
        st.cross={v[1]*u[2]-u[1]*v[2],v[2]*u[0]-u[2]*v[0],v[0]*u[1]-u[0]*v[1],Ps2Word{}};st.frame=stack;st.aimed=true;
        dot.store(record(rt,stack+0xd0,16));return true;}
    if(pc==0x248c50){
        if(!st.aimed || st.frame!=stack || c->r[2].uw[0]!=stack+0xc0)throw std::runtime_error("native chain aim axis without its aim");
        ps2Publish(rt,stack+0xc0,unitOf(st.cross),3);st.aimed=false;return true;}
    if(pc==0x248cac){
        if(c->r[2].uw[0]!=stack+0xb0 || c->r[3].uw[0]!=c->r[18].uw[0]+0x30)throw std::runtime_error("native chain aim rotation escaped its records");
        const auto q1=ps2Record(rt,stack+0xb0),q2=ps2Record(rt,c->r[18].uw[0]+0x30);Ps2Lanes out;
        const Ps2Word cross[3]={q1[1]*q2[2]-q2[1]*q1[2],q1[2]*q2[0]-q2[2]*q1[0],q1[0]*q2[1]-q2[0]*q1[1]};
        for(unsigned i=0;i<3;++i)out[i]=(q2[i]*q1[3]+q1[i]*q2[3])+cross[i]*kPs2One;
        Ps2Word p[4];for(unsigned i=0;i<4;++i)p[i]=q1[i]*q2[i];
        out[3]=((p[3]-p[2])-kPs2One*p[1])-kPs2One*p[0];
        ps2Publish(rt,stack+0xb0,out);return true;}
    return true;   // loads and arithmetic: folded into the three stores
}
// mod028 4ab9a0(a0, a1, a2 vector, a3, f12 limit): the vector at a2 (copied to sp+50) turned
// toward the direction a1 - a0, ps2f in listing order:
//  4ab9c0..4ab9fc  n = (a1 - a0).xyz * RSQRT(|a1 - a0|^2) to s0 = sp+60 (w: a1's).
//  4aba00..4aba10  (a3 = 0) QMFC2 v0 = (v.n, v.y n.y, v.z n.z, v.w) with v = s1 = sp+50.
//  4aba5c..4abab4  after 214100(a2, -angle) stored (k4, k5, cos, sin) at a2 (its vf13, read from
//                  there): c = v x n; d = (c.x v.x + c.y v.y) + c.z v.z; out = ((v x c) * sin + c*d)
//                  + (v - c*d) * cos, normalized, to a2 with w = 1 (MAX.w vf0, vf0).
struct VectorTurn{Ps2Lanes n{};uint32_t frame=0;bool aimed=false;};
std::map<const Runtime *,VectorTurn> &vectorTurns(){static std::map<const Runtime *,VectorTurn> m;return m;}
bool nativeVectorTurn(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Site{uint32_t pc,word;};
    constexpr Site sites[]={{0x4ab9c0,0xd8ac0000},{0x4ab9c4,0xd88d0000},{0x4ab9c8,0x4bc007c3},{0x4ab9dc,0x4bcd632c},{0x4ab9e0,0x4bcc636a},{0x4ab9e4,0x4b0d683d},{0x4ab9e8,0x4b0dfb4a},{0x4ab9ec,0x4a6d03be},{0x4ab9f0,0x4a0003bf},{0x4ab9f4,0x4bc0631c},{0x4ab9fc,0xfa0c0000},
        {0x4aba00,0xda300000},{0x4aba04,0x4bcc842a},{0x4aba08,0x4b10803d},{0x4aba0c,0x4b10fc0a},{0x4aba10,0x48228000},
        {0x4aba5c,0xda0e0000},{0x4aba60,0xda300000},{0x4aba64,0x4a20032b},{0x4aba68,0x4bce82fe},{0x4aba6c,0x4bd073ae},{0x4aba70,0x4bc007c3},{0x4aba74,0x4bd073ea},{0x4aba78,0x4bce82fe},{0x4aba7c,0x4bd0732e},{0x4aba80,0x4b0f783d},{0x4aba84,0x4b0ffbca},{0x4aba88,0x4bcd631b},{0x4aba8c,0x4bcf7398},{0x4aba90,0x4bce842c},{0x4aba94,0x4bce62bc},{0x4aba98,0x4bcd830a},{0x4aba9c,0x4bcc636a},{0x4abaa0,0x4b0d683d},{0x4abaa4,0x4b0dfb4a},{0x4abaa8,0x4a6d03be},{0x4abaac,0x4a0003bf},{0x4abab0,0x4bc0631c},{0x4abab4,0xfa4c0000}};
    if((pc&3) || !rt || pc<0x4ab9c0 || pc>0x4abab4)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](const Site &x){return x.pc==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native vector turn has no CPU record");
    const auto stack=c->r[29].uw[0];auto &st=vectorTurns()[rt];
    const auto unitOf=[](const Ps2Lanes &t){
        const auto q=Ps2Word::reciprocalRoot(kPs2One,(t[0]*t[0]+t[1]*t[1])+kPs2One*(t[2]*t[2]));Ps2Lanes u=t;for(unsigned i=0;i<3;++i)u[i]=t[i]*q;return u;};
    if(pc==0x4ab9fc){
        if(c->r[16].uw[0]!=stack+0x60)throw std::runtime_error("native vector turn direction destination mismatch");
        const auto a=ps2Record(rt,c->r[5].uw[0]),b=ps2Record(rt,c->r[4].uw[0]);Ps2Lanes d=a;for(unsigned i=0;i<3;++i)d[i]=a[i]-b[i];
        st.n=unitOf(d);st.frame=stack;st.aimed=true;ps2Publish(rt,stack+0x60,st.n);return true;}
    if(pc==0x4aba10){
        if(!st.aimed || st.frame!=stack || c->r[17].uw[0]!=stack+0x50)throw std::runtime_error("native vector turn dot without its direction");
        const auto v=ps2Record(rt,stack+0x50);Ps2Word p[3];for(unsigned i=0;i<3;++i)p[i]=v[i]*st.n[i];
        const auto dot=(p[0]+p[1])+kPs2One*p[2];
        c->r[2].uw[0]=dot.bits;c->r[2].uw[1]=p[1].bits;c->r[2].uw[2]=p[2].bits;c->r[2].uw[3]=v[3].bits;return true;}
    if(pc==0x4abab4){
        if(c->r[16].uw[0]!=stack+0x60 || c->r[17].uw[0]!=stack+0x50)throw std::runtime_error("native vector turn escaped its records");
        const auto n=ps2Record(rt,stack+0x60),v=ps2Record(rt,stack+0x50),t=ps2Record(rt,c->r[18].uw[0]);   // t: 214100's (k4, k5, cos, sin)
        const Ps2Word cr[3]={v[1]*n[2]-n[1]*v[2],v[2]*n[0]-n[2]*v[0],v[0]*n[1]-n[0]*v[1]};
        const Ps2Word vc[3]={v[1]*cr[2]-cr[1]*v[2],v[2]*cr[0]-cr[2]*v[0],v[0]*cr[1]-cr[0]*v[1]};
        const auto d=(cr[0]*v[0]+cr[1]*v[1])+kPs2One*(cr[2]*v[2]);
        Ps2Lanes out{};for(unsigned i=0;i<3;++i){const auto turned=vc[i]*t[3],along=cr[i]*d,rest=v[i]-along;out[i]=(turned+along)+rest*t[2];}
        out=unitOf(out);out[3]=kPs2One;ps2Publish(rt,c->r[18].uw[0],out);st.aimed=false;return true;}
    return true;   // loads and arithmetic: folded into the stores and the QMFC2
}
// mod028 500df0 (a palette crossfade over a2 entries; from 501114): LQC2 vf1 = sp = (t, 1 - t,
// 255, 0) and vf2 = the entry's weights (t0+20) once (500ec0/500ec4); per colour the EE code widens
// two RGBA words to four words each (PEXTLB/PEXTLH), QMTC2 t0 -> vf3 (A), t1 -> vf4 (B), then ITOF0
// both; ACC = A * t; vf4 = min(B * w, 255) (MINIz); vf3 = min(ACC + vf4 * (1 - t), 255); FTOI0;
// QMFC2 t0 (500f10), all four lanes; the EE code packs the bytes and keeps B's alpha byte.
struct PaletteFade{VuWords fade{},weights{};bool loaded=false;uint32_t a[4]{},b[4]{};unsigned moved=0;};
std::map<std::pair<const Runtime *,uint32_t>,PaletteFade> &paletteFades(){static std::map<std::pair<const Runtime *,uint32_t>,PaletteFade> m;return m;}
bool nativePaletteFade(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Site{uint32_t pc,word;char role;};
    constexpr Site sites[]={{0x500ec0,0xd8610000,'f'},{0x500ec4,0xd8820000,'w'},{0x500ee8,0x48a81800,'a'},{0x500eec,0x48a92000,'b'},{0x500ef0,0x4be3193c,'-'},{0x500ef4,0x4be4213c,'-'},
        {0x500ef8,0x4be119bc,'-'},{0x500efc,0x4be2212a,'-'},{0x500f00,0x4be12116,'-'},{0x500f04,0x4be120c9,'-'},{0x500f08,0x4be118d6,'-'},{0x500f0c,0x4be3197c,'-'},{0x500f10,0x48281800,'m'}};
    if((pc&3) || !rt || pc<0x500ec0 || pc>0x500f10)return false;
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](const Site &x){return x.pc==pc;});
    if(site==std::end(sites))return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native palette fade has no CPU record");
    auto &st=paletteFades()[{rt,c->r[29].uw[0]}];
    switch(site->role){
    case 'f':st=PaletteFade{};st.fade=vuWords(rt,c->r[3].uw[0]);return true;
    case 'w':st.weights=vuWords(rt,c->r[4].uw[0]);st.loaded=true;return true;
    case 'a':if(!st.loaded)throw std::runtime_error("native palette fade colour without its factors");for(unsigned i=0;i<4;++i)st.a[i]=c->r[8].uw[i];st.moved=1;return true;
    case 'b':if(st.moved!=1)throw std::runtime_error("native palette fade colours out of order");for(unsigned i=0;i<4;++i)st.b[i]=c->r[9].uw[i];st.moved=2;return true;
    case 'm':{if(st.moved!=2)throw std::runtime_error("native palette fade result without its colours");
        const auto toFloat=[](uint32_t v){return std::bit_cast<uint32_t>(float(int32_t(v)));};   // ITOF0 (exact for these byte lanes)
        const auto minimum=[](uint32_t x,uint32_t y){return std::bit_cast<float>(x)<std::bit_cast<float>(y)?x:y;};
        for(unsigned i=0;i<4;++i){
            if(st.a[i]>255 || st.b[i]>255)throw std::runtime_error("native palette fade lane outside a byte");
            const auto acc=vuMul(toFloat(st.a[i]),st.fade[0]);
            const auto b=minimum(vuMul(toFloat(st.b[i]),st.weights[i]),st.fade[2]);
            c->r[8].uw[i]=vuFtoi(minimum(vuAdd(acc,vuMul(b,st.fade[1])),st.fade[2]),0);}
        st.moved=0;return true;}
    default:return true;   // ITOF0 / MULA / MUL / MINI / MADD / FTOI0: folded into the QMFC2
    }
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
    // PS2 words (see Ps2Word): every word is a VU number (no overflow gates).
    if(place){
        const auto point=word(part+0x38)+0x30,matrix=word(c->r[18].uw[0]+4)+0x190;
        if(pc==0x1fda08 && c->r[2].uw[0]!=point)throw std::runtime_error("native chain point source mismatch");
        if(pc!=0x1fda70 && pc!=0x1fda7c && pc!=0x1fda84)return true;
        if(c->r[2].uw[0]!=(pc==0x1fda84?stack+0x220:stack+0x460))throw std::runtime_error("native chain placement destination mismatch");
        const bool offset=(word(part+0x20)&4)!=0;
        if(pc==0x1fda70){if(!offset)throw std::runtime_error("native chain offset branch mismatch");std::memcpy(record(rt,stack+0x460,16),record(rt,part,16),16);return true;}
        if(pc==0x1fda7c){if(offset)throw std::runtime_error("native chain offset branch mismatch");const float identity[4]={0,0,0,1};std::memcpy(record(rt,stack+0x460,16),identity,16);return true;}
        auto out=ps2Transform(ps2Rows(rt,matrix,4),ps2Record(rt,point));
        if(offset){const auto shift=ps2Record(rt,part);for(unsigned i=0;i<3;++i)out[i]=out[i]+shift[i];}
        ps2Publish(rt,stack+0x220,out);return true;
    }
    if(split){
        if(pc!=0x1fdaec && pc!=0x1fdaf0)return true;
        if(pc==0x1fdaec?c->r[3].uw[0]!=stack+0x460:c->r[2].uw[0]!=stack+0x470)throw std::runtime_error("native chain split destination mismatch");
        const auto v=ps2Record(rt,stack+0x220);const auto scale=pc==0x1fdaec?Ps2Word{word(part+0x30)}:Ps2Word::of(c->f[22]);
        Ps2Lanes out;for(unsigned i=0;i<4;++i)out[i]=v[i]*scale;
        ps2Publish(rt,pc==0x1fdaec?stack+0x460:stack+0x470,out);return true;
    }
    if(rotateA || rotateB){
        const auto matrix=word(c->r[18].uw[0]+4)+0x150;const uint32_t store=rotateA?0x1fdb74:0x1fdc68;
        if((rotateA && pc==0x1fdb54 && (c->r[3].uw[0]!=stack+0x210 || c->r[2].uw[0]!=matrix)) || (rotateB && pc==0x1fdc48 && (c->r[2].uw[0]!=stack+0x210 || c->r[3].uw[0]!=matrix)))
            throw std::runtime_error("native chain rotation source mismatch");
        if(pc!=store)return true;
        if(c->r[2].uw[0]!=stack+0x210)throw std::runtime_error("native chain rotation destination mismatch");
        const auto m=ps2Rows(rt,matrix,3);const auto v=ps2Record(rt,stack+0x210);Ps2Lanes out;
        for(unsigned i=0;i<4;++i)out[i]=(m.rows[0][i]*v[0]+m.rows[1][i]*v[1])+m.rows[2][i]*v[2];
        ps2Publish(rt,stack+0x210,out);return true;
    }
    if(blend){
        if(pc==0x1fdeb0 && c->r[2].uw[0]!=stack+0x220)throw std::runtime_error("native chain blend source mismatch");
        if(pc!=0x1fdee4 && pc!=0x1fdee8)return true;
        if(pc==0x1fdee4?c->r[3].uw[0]!=stack+0x200:c->r[2].uw[0]!=stack+0x1f0)throw std::runtime_error("native chain blend destination mismatch");
        const auto dir=ps2Record(rt,stack+0x220),origin=ps2Record(rt,stack+0x460);
        auto out=ps2Record(rt,pc==0x1fdee4?stack+0x460:stack+0x220);   // W kept raw
        const Ps2Word a{word(part+0x30)};const auto s=Ps2Word::of(c->f[22]);
        for(unsigned i=0;i<3;++i)out[i]=pc==0x1fdee4?(origin[i]+kPs2Zero)+dir[i]*a:(dir[i]+kPs2Zero)-dir[i]*s;
        ps2Publish(rt,pc==0x1fdee4?stack+0x200:stack+0x1f0,out);return true;
    }
    // product
    const auto owner=c->r[16].uw[0];
    if(c->r[4].uw[0]!=stack+0x120 || c->r[3].uw[0]!=owner+0x30 || c->r[2].uw[0]!=stack+0x110)throw std::runtime_error("native chain product escaped its records");
    if(pc!=0x1fe7e8)return true;
    const auto m=ps2Rows(rt,stack+0x120,4);const auto v=ps2Record(rt,owner+0x30);Ps2Lanes out;
    Ps2Word p[4][4];for(unsigned k=0;k<4;++k)for(unsigned i=0;i<4;++i)p[k][i]=m.rows[k][i]*v[i];
    constexpr unsigned order[4][4]={{0,1,2,3},{1,0,2,3},{2,1,0,3},{3,1,2,0}};
    for(unsigned k=0;k<4;++k)out[k]=((p[k][order[k][0]]+p[k][order[k][1]])+kPs2One*p[k][order[k][2]])+kPs2One*p[k][order[k][3]];
    ps2Publish(rt,stack+0x110,out);return true;
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
    const auto lane=[&](uint32_t at){return Ps2Word{word(at)};};
    // PS2 words (see Ps2Word): every word is a VU number; zero weights and
    // separations follow DIV/RSQRT (+/-max) as the VU does.
    if(place){
        if(pc==0x1fc334 && (c->r[2].uw[0]!=a+0x250 || c->r[31].uw[0]!=0x1fdc2c))throw std::runtime_error("native capsule pair caller mismatch");
        if(pc!=0x1fc3c0 && pc!=0x1fc3c4)return true;
        if(c->r[2].uw[0]!=stack+0x60)throw std::runtime_error("native capsule pair destination mismatch");
        const auto m=ps2Rows(rt,word(word(stack+0x20)+4)+0x190,4);Ps2Lanes out;
        if(pc==0x1fc3c4){
            const auto d=ps2Record(rt,a+0x260);
            for(unsigned i=0;i<4;++i)out[i]=(m.rows[0][i]*d[0]+m.rows[1][i]*d[1])+m.rows[2][i]*d[2];
            ps2Publish(rt,stack+0x70,out);return true;
        }
        const auto weights=ps2Record(rt,c->r[6].uw[0]);const auto ra=lane(a+0x220),rb=lane(b+0x220);
        out=ps2Transform(m,ps2Record(rt,a+0x250));const auto sum=rb+ra;
        for(unsigned i=0;i<3;++i)out[i]=out[i]*(sum/((kPs2Zero+ra)+weights[i]*rb));
        ps2Publish(rt,stack+0x60,out);return true;
    }
    if(pc==0x1fc410 && (c->r[3].uw[0]!=b+0x230 || c->r[2].uw[0]!=b+0x240 || c->r[5].uw[0]!=stack+0x60 || c->r[4].uw[0]!=stack+0x70))
        throw std::runtime_error("native capsule separation sources mismatch");
    if(pc!=0x1fc478)return true;
    const auto target=c->r[18].uw[0];if(target&15)throw std::runtime_error("native capsule separation destination misaligned");
    const auto p0=ps2Record(rt,b+0x230),p1=ps2Record(rt,b+0x240),p=ps2Record(rt,stack+0x60),d=ps2Record(rt,stack+0x70);
    const auto tb=lane(stack+0x88),ta=lane(stack+0x8c);Ps2Lanes delta{};
    for(unsigned i=0;i<3;++i)delta[i]=(((p0[i]+kPs2Zero)+p1[i]*tb)-p[i]*kPs2One)-d[i]*ta;
    const auto q=Ps2Word::reciprocalRoot(kPs2One,(delta[0]*delta[0]+delta[1]*delta[1])+kPs2One*(delta[2]*delta[2]));
    for(unsigned i=0;i<3;++i)delta[i]=delta[i]*q;
    ps2Publish(rt,target,delta,3);return true;
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
        // PS2 words (see Ps2Word): every word is a VU number.
        const auto p=ps2Record(rt,point),o=ps2Record(rt,plane+0x230),n=ps2Record(rt,plane+0x220);Ps2Word m[3];
        for(unsigned i=0;i<3;++i)m[i]=(p[i]-o[i])*n[i];
        (((m[0]+m[1])+kPs2One*m[2])-Ps2Word::of(c->f[12])).store(record(rt,stack+0x10,16));return true;
    }
    {
        if(pc!=0x1fbdc0)return true;
        const auto target=c->r[6].uw[0];if(target&15)throw std::runtime_error("native plane contact output misaligned");
        auto out=ps2Record(rt,point);const auto n=ps2Record(rt,plane+0x220);const auto d=Ps2Word::load(record(rt,stack+0x10,16));
        for(unsigned i=0;i<3;++i)out[i]=(out[i]+kPs2Zero)-n[i]*d;
        ps2Publish(rt,target,out);return true;
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
// mod001 2202d0 cull transform, per listed object s0 (vtable +68 returns its
// matrix in v0): block 2203e0 composes the rows at s4+2f0 with that matrix
// into SP+a0 (row k = sum_j A[k][j]*B[j], VU0 MULA/MADDA/MADD order); the
// EE code transposes SP+a0 in place; block 2204a4 transforms the point at
// SP+e0 by those rows into SP+90. Each block publishes at its first store.
bool nativeCullTransform(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool compose=pc>=0x2203e0 && pc<=0x22044c,point=pc>=0x2204a4 && pc<=0x2204c8;
    if(!(compose || point) || (pc&3))return false;
    if(!c)throw std::runtime_error("native cull transform has no CPU record");
    const auto stack=c->r[29].uw[0];
    const auto qualify=[](const Lanes &row){for(float x:row)if(!std::isfinite(x))throw std::runtime_error("native cull transform overflow");};
    if(compose){
        if(c->r[3].uw[0]!=c->r[20].uw[0]+0x2f0 || c->r[4].uw[0]!=stack+0xa0)throw std::runtime_error("native cull composition records mismatch");
        if(pc!=0x220440)return true;
        const auto rowsAt=c->r[3].uw[0],matrixAt=c->r[2].uw[0];
        if((rowsAt|matrixAt)&15)throw std::runtime_error("native cull composition source misaligned");
        Lanes a[4],b[4],out[4];for(unsigned k=0;k<4;++k){a[k]=leafRecord(rt,rowsAt+16*k);b[k]=leafRecord(rt,matrixAt+16*k);}
        {music::NearestRounding rounding;
            for(unsigned k=0;k<4;++k)for(unsigned i=0;i<4;++i)
                out[k][i]=pairSum(pairSum(pairSum(pairProduct(b[0][i],a[k][0]),pairProduct(b[1][i],a[k][1])),pairProduct(b[2][i],a[k][2])),pairProduct(b[3][i],a[k][3]));}
        for(const auto &row:out)qualify(row);
        std::memcpy(record(rt,stack+0xa0,64),out,64);return true;
    }
    if(c->r[4].uw[0]!=stack+0xe0 || c->r[3].uw[0]!=stack+0xa0 || c->r[2].uw[0]!=stack+0x90)throw std::runtime_error("native cull point records mismatch");
    if(pc!=0x2204c8)return true;
    Lanes rows[4],p=leafRecord(rt,stack+0xe0),out;for(unsigned k=0;k<4;++k)rows[k]=leafRecord(rt,stack+0xa0+16*k);
    {music::NearestRounding rounding;
        for(unsigned i=0;i<4;++i)out[i]=pairSum(pairSum(pairSum(pairProduct(rows[0][i],p[0]),pairProduct(rows[1][i],p[1])),pairProduct(rows[2][i],p[2])),pairProduct(rows[3][i],p[3]));}
    qualify(out);std::memcpy(record(rt,stack+0x90,16),out.data(),16);return true;
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
// Code in the shared overlay area (0x48ef00, mod004..mod013): a site there is
// served only while RAM holds its module's block (FNV-1a 64 of [lo,hi)).
bool overlayCode(Runtime *rt,uint32_t lo,uint32_t hi,uint64_t expected){return bozo::hash::fnv1a64(record(rt,lo,hi-lo),hi-lo)==expected;}
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
// mod010 48fa10 marker projection (code block 491f40..4923a0, checked in RAM:
// mod004..mod013 share these addresses). After 207cc0(camera) the case loads
// LQC2 vf24..27 = camera+3f0 rows R0..R3 and runs program 0x1f0 on camera+2f0
// into vf1..4, copied to vf20..23; only 0xb60 reads VU registers afterwards,
// and it reads vf24..27, vf5 and vf0. Then for 64 records (s1 += 0x40, the
// loop counter reuses s0, so the camera is gone) LQC2 vf5 = *s1, VCALLMS 0xb60,
// CFC2.I t1 = VI0, SQC2 vf6 -> SP+170. Program 0xb60 (VU0 micro listing):
// p = ((R0*v.x + R1*v.y) + R2*v.z) + R3*1 (MULAx/MADDAy/MADDAz/MADDw), DIV Q =
// 1/p.z, MOVE.w vf6 = v.w, MULQ.xy: vf6 = (p.x*Q, p.y*Q, p.z, v.w). Its frustum
// half (vf10..19, vf28..31, VI2/VI3) is not read here. The rows live across
// calls and the loop's dispatcher checkpoint, so they are kept per runtime
// from their loads (serialized in save states) and used by each projection.
struct MarkerRows{std::array<VuWords,4> rows{};unsigned loaded=0;bool point=false;VuWords v{};};
MarkerRows &markerRows(const Runtime *rt){static std::map<const Runtime *,MarkerRows> states;return states[rt];}
constexpr uint32_t markerCodeLo=0x491f40,markerCodeHi=0x4923a0;constexpr uint64_t markerCode=0x3a0229bb90002a3aull;
bool nativeMarkerProjection(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool rows=pc>=0x491f44 && pc<=0x491f50,unused=(pc>=0x491f90 && pc<=0x491f9c) || (pc>=0x491fa8 && pc<=0x491fb8);
    const bool projection=pc==0x492380 || pc==0x49238c || pc==0x492390 || pc==0x492398;
    if(!(rows || unused || projection) || (pc&3))return false;
    // The code is checked at the first row load and at each point load; the
    // sites in between follow them in straight-line code.
    auto &state=markerRows(rt);const unsigned k=(pc-0x491f44)/4;
    if((pc==0x491f44 || pc==0x492380) && !overlayCode(rt,markerCodeLo,markerCodeHi,markerCode))return false;
    if(rows && k && !(state.loaded&(1u<<(k-1))))return false;
    if(unused && (state.loaded&15)!=15)return false;
    if((pc==0x49238c || pc==0x492390 || pc==0x492398) && !state.point)return false;
    if(!c)throw std::runtime_error("native marker projection has no CPU record");
    const auto camera=c->r[16].uw[0];
    if(rows){
        if(c->r[2].uw[0]!=camera+0x3f0)throw std::runtime_error("native marker projection rows escaped the camera");
        if(k==0)state=MarkerRows{};
        state.rows[k]=vuWords(rt,camera+0x3f0+16*k);state.loaded|=1u<<k;return true;
    }
    if(unused){if(pc==0x491f90 && c->r[3].uw[0]!=camera+0x2f0)throw std::runtime_error("native marker transpose escaped the camera");return true;}
    if(pc==0x492380){
        if((state.loaded&15)!=15)throw std::runtime_error("native marker projection without its camera rows");
        state.v=vuWords(rt,c->r[17].uw[0]);state.point=true;return true;
    }
    if(pc==0x49238c)return true;
    if(pc==0x492390){c->r[9].ud[0]=0;return true;}
    const auto stack=c->r[29].uw[0];
    if(c->r[3].uw[0]!=stack+0x170)throw std::runtime_error("native marker projection destination mismatch");
    if(!state.point)throw std::runtime_error("native marker projection stored without its point");
    VuWords out;const auto &R=state.rows;const auto &v=state.v;
    for(unsigned i=0;i<4;++i)out[i]=vuAdd(vuAdd(vuAdd(vuMul(R[0][i],v[0]),vuMul(R[1][i],v[1])),vuMul(R[2][i],v[2])),vuMul(R[3][i],0x3f800000u));
    uint32_t f=0;const auto q=bozo::ps2f::div(0x3f800000u,out[2],f);
    out[0]=vuMul(out[0],q);out[1]=vuMul(out[1],q);out[3]=v[3];
    std::memcpy(record(rt,stack+0x170,16),out.data(),16);state.point=false;return true;
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
// mod001 218600 (element class 1536a0 +10): one morph step for a mesh. The
// table at e+38 holds count c (+0), its size (+8, the cursor advance), the
// packed-vector byte count (+c) and c entries of 32 bytes from +10: position
// delta (16 bytes) and normal delta whose .w word is the vertex index. Weights
// are e+40 (.x positions/normals, .y packed vectors). Program 0x7d8 is
// pipelined: each call outputs the previous entry, so the EE stores one call
// late and repeats the last call to flush. Per entry j, vertex k:
//   p.xyz = p*1 + dp*w.x; n' = n*1 + dn*w.x; n.xyz = n' * rsqrt(len2(n'))
//   with len2 = (1*n'x^2 + 1*n'y^2) + 1*n'z^2; n.w = the next entry's input
//   normal .w (its own for the last); p.w is vf12.w, a VU register never
//   written by the program: the port keeps the record's w.
// Program 0x848, for the first (+c)/8 packed vectors in order with deltas
// following the entries: v = ftoi12(itof12(v)*1 + itof12(d)*w.y), all lanes.
void morphMesh(Runtime *rt,uint32_t element,unsigned vertices,uint8_t *positions,uint8_t *normals,uint8_t *packed){
    const auto manager=skinWord(rt,0x35ddb8);
    if(skinByte(rt,manager+0x51)!=1)throw std::runtime_error("native morph outside VU0 mode 1");
    const auto table=skinWord(rt,element+0x38);const VuWords weights=vuWords(rt,element+0x40);
    const int32_t entries=int32_t(skinWord(rt,table));
    if(entries<1 || entries>0x10000)throw std::runtime_error(std::format("unqualified morph entry count {}",entries));
    const uint32_t one=0x3f800000u,savedCsr=_mm_getcsr();
    struct Out{unsigned index;VuWords p,n;};std::vector<Out> out(entries);std::vector<bool> seen(vertices);
    for(int32_t j=0;j<entries;++j){
        const uint32_t entry=table+0x10+0x20*uint32_t(j);const auto dp=vuWords(rt,entry),dn=vuWords(rt,entry+0x10);
        const auto index=dn[3];if(index>=vertices)throw std::runtime_error("morph vertex index outside its mesh");
        // The EE loads entry j+1 before storing entry j-1: repeated indices would
        // read a half-updated vertex. Not observed, not ported.
        if(seen[index])throw std::runtime_error("unported repeated morph vertex index");seen[index]=true;
        VuWords p,n;std::memcpy(p.data(),positions+16*index,16);std::memcpy(n.data(),normals+16*index,16);
        auto &o=out[j];o.index=index;o.p=p;o.n=n;
        for(unsigned i=0;i<3;++i){o.p[i]=vuAdd(vuMul(p[i],one),vuMul(dp[i],weights[0]));o.n[i]=vuAdd(vuMul(n[i],one),vuMul(dn[i],weights[0]));}
        uint32_t sq[3];for(unsigned i=0;i<3;++i)sq[i]=vuMul(o.n[i],o.n[i]);
        const auto len2=vuAdd(vuAdd(vuMul(one,sq[0]),vuMul(one,sq[1])),vuMul(one,sq[2]));
        uint32_t f=0;const auto q=bozo::ps2f::rsqrt(one,len2,f);_mm_setcsr(savedCsr);
        for(unsigned i=0;i<3;++i)o.n[i]=vuMul(o.n[i],q);
        o.n[3]=n[3];
    }
    for(int32_t j=0;j<entries;++j){auto &o=out[j];if(j+1<entries)o.n[3]=out[j+1].n[3];
        std::memcpy(positions+16*o.index,o.p.data(),16);std::memcpy(normals+16*o.index,o.n.data(),16);}
    const auto bytes=int32_t(skinWord(rt,table+0xc));
    if(packed && bytes){
        if(bytes<0 || (bytes>>3)>int32_t(vertices))throw std::runtime_error("morph packed vectors outside its mesh");
        uint32_t delta=table+0x10+0x20*uint32_t(entries);
        for(int32_t i=0;i<(bytes>>3);++i,delta+=8){
            uint8_t d[8];std::memcpy(d,rt->memory.hostPointer(delta,8),8);auto *v=packed+8*i;
            for(unsigned l=0;l<4;++l){const auto a=skinItof12(uint32_t(int32_t(int16_t(v[2*l]|v[2*l+1]<<8)))),b=skinItof12(uint32_t(int32_t(int16_t(d[2*l]|d[2*l+1]<<8))));
                const auto r=skinFtoi12(vuAdd(vuMul(a,one),vuMul(b,weights[1])));v[2*l]=uint8_t(r);v[2*l+1]=uint8_t(r>>8);}
        }
    }
    _mm_setcsr(savedCsr);
    // 218720: the cursor advances by the table's size.
    const auto advanced=table+skinWord(rt,table+8);std::memcpy(rt->memory.hostPointer(element+0x38,4),&advanced,4);
}
// Three more inlined VU0 macro idioms (mod001 20e5b8 has 16 of them):
//  negate:  LQC2 vf1; SUB.xyz vf1 = vf0 - vf1; SQC2 vf1        -> {-x,-y,-z,w}
//  dot:     LQC2 vf20,vf21,vf22; SUB vf23 = vf21 - vf20; ADDw.x vf24 = 1;
//           MUL vf23 = vf22*vf23; ADDAy; MADDz; SQC2 vf23     -> {(px+py)+1*pz, py, pz, pw}
//  square:  LQC2 vf20,vf21; SUB vf22 = vf21 - vf20; MUL vf22 = vf22*vf22; SQC2 vf22
//  normalize (mod001 258230), see the table.
// Plain EE instructions may sit between the COP2 words, and they may reuse a
// load's base register as the store's, so each load captures its quadword at
// its own PC; the store computes the fixed formula. Recognised only from the
// exact COP2 words of a straight block (no branch in between).
struct MacroIdiom{const char *name;std::array<int32_t,28> words;unsigned size;};
// >=0: exact word; -1-n: LQC2 into vf n; -100-n: SQC2 from vf n; -200: QMFC2 from vf24 to any GPR.
constexpr MacroIdiom macroIdioms[]={
    {"negate",{-2,0x4bc1006c,-101},3},
    {"dot",{-21,-22,-23,0x4bf4adec,0x4b000603,0x4bf7b5ea,0x4b17b83d,0x4b17c5ca,-123},9},
    {"square",{-21,-22,0x4bf4adac,0x4bf6b5aa,-122},5},
    // normalize: LQC2 vf20; MUL.xyz vf22 = vf20^2; ADDw.x vf21 = 1; ADDAy; MADDz;
    // RSQRT Q = 1/sqrt(vf22.x); WAITQ; MULq.xyz vf20; SQC2 vf20 -> {xyz*Q, w}
    {"normalize",{-21,0x4bd4a5aa,0x4b000543,0x4b16b03d,0x4b16ad8a,0x4a7603be,0x4a0003bf,0x4bc0a51c,-120},9},
    // squared length (mod003 397a9c and its kin): LQC2 vf24; MUL.xyz vf24^2;
    // ADDw.xyz vf25 = 1; ADDAy; MADDz.x; SQC2 vf24 -> {(x2+y2)+1*z2, y2, z2, w}.
    // The length idiom (same words followed by SQRT) is claimed before this.
    {"squared",{-25,0x4bd8c62a,0x4bc00643,0x4b18c03d,0x4b18ce0a,-124},6},
    // plain dot (mod003 436eac): LQC2 vf1,vf2; MUL.xyz vf3 = vf1*vf2; ADDy.x vf1 = vf3.x+vf3.y;
    // ADDz.x vf1 += vf3.z; SQC2 vf1 -> {(x1x2+y1y2)+z1z2, y1, z1, w1}.
    {"dot3",{-2,-3,0x4bc208ea,0x4b031841,0x4b030842,-101},6},
    // scaled sum (mod003 433c70): LQC2 vf1 (a), vf2 (b); QMTC2 v0 -> vf3 (s in .x, a GPR
    // value: 2.0 there); MULAw.xyz ACC = b*1; MADDx.xyz vf1 = ACC + a*s; SQC2 vf1
    // -> {b + a*s (xyz), a.w}.
    {"scaledsum",{-2,-3,0x48a21800,0x4bc011bf,0x4bc30848,-101},6},
    // the same with s from v1 (mod003 38cea0, twice; an MFC1 f0 precedes the QMTC2).
    {"scaledsum",{-2,-3,0x48a31800,0x4bc011bf,0x4bc30848,-101},6},
    // point advanced along a direction (mod028 47bfa4 at 47c108): LQC2 vf1 (point), vf2
    // (direction); QMTC2 t0 -> vf3 (k in .x: the FPU's (f2 + f1) + 0.5 f3, moved by MFC1);
    // MULAw.xyz ACC = vf1*1; MADDx.xyz vf1 = ACC + vf2*k; SQC2 vf1 -> {p + d*k (xyz), p.w}.
    {"advance",{-2,-3,0x48a81800,0x4bc009bf,0x4bc31048,-101},6},
    // colour scale (mod028 47bfa4 at 47c280): LQC2 vf1 (the object's +9a0 colour); QMTC2 t0
    // -> vf2 (k in .x: 1/128 there); MULx.xyz vf1 = vf1*k; MULx.w vf1 = vf1.w*vf0.x (0);
    // SQC2 vf1 -> {rgb*k, w*0}.
    {"colourscale",{-2,0x48a81000,0x4bc20858,0x4a200858,-101},5},
    // row point (mod001 2b802c): LQC2 vf15 (point), vf11..vf14 (rows); ADDw.xyz vf31 = 1;
    // MUL vf11..vf14 *= vf15; per row ADDA/MADDA/MADD with 1*: x ((x+y)+1z)+1w,
    // y ((y+x)+1z)+1w, z ((z+y)+1x)+1w, w ((w+y)+1z)+1x; SQC2 vf5.
    {"rowpoint",{-16,-12,-13,-14,-15,0x4bc007c3,0x4bef5aea,0x4bef632a,0x4bef6b6a,0x4bef73aa,0x4b0b583d,0x4b0bf8be,0x4b0bf94b,
        0x4a8c603c,0x4a8cf8be,0x4a8cf94b,0x4a4d683d,0x4a4df8bc,0x4a4df94b,0x4a2e703d,0x4a2e00be,0x4a2e0148,-105},23},
    // homogeneous divide (mod001 2b80a0): LQC2 vf20; DIV Q = 1/vf20.w; WAITQ;
    // MULq.xyzw vf20; SQC2 vf20 -> {x*Q, y*Q, z*Q, w*Q}.
    {"homogeneous",{-21,0x4bf403bc,0x4a0003bf,0x4be0a51c,-120},5},
    // component product (mod003 366d10): LQC2 vf12, vf13; MUL.xyzw vf12 = vf12*vf13;
    // SQC2 vf12 in the delay slot of the routine's jr ra -> {x1x2, y1y2, z1z2, w1w2}.
    {"product",{-13,-14,0x4bed632a,-112},4},
    // particle draw setup (mod001 22f3a4 / 22f3d8, store in the delay slot of b 22f418):
    // LQC2 vf1, vf2, vf3; MULA.xyzw ACC = vf1*vf2; MADDw.xyzw vf1 = ACC + vf3*1; SQC2 vf1.
    {"muladd",{-2,-3,-4,0x4be20abe,0x4be0184b,-101},6},
    // and its third variant (22f404): LQC2 vf1, vf2; MUL.xyzw vf1 = vf1*vf2; SQC2 vf1.
    {"product2",{-2,-3,0x4be2086a,-101},4},
    // mod028 (battle, about 44 copies, e.g. 4e6678): LQC2 vf25; ADDw.xyz vf31 = 1; MUL.xyz
    // vf26 = vf25^2; ADDAy.x; MADDz.x vf26.x = ACC + 1*z2; RSQRT; WAITQ; MULq.xyz vf25;
    // SQC2 vf25 -> {xyz*Q, w}, the "normalize" formula in other registers.
    {"normalize25",{-26,0x4bc007c3,0x4bd9ceaa,0x4b1ad03d,0x4b1afe8a,0x4a7a03be,0x4a0003bf,0x4bc0ce5c,-125},9},
    // mod028/mod068 direction (15 copies, e.g. 4a4094, 3e4dc8, 527948): LQC2 vf26 (a), vf27 (b); ADDw.xyz
    // vf31 = 1; SUB.xyz vf26 = b - a; MUL.xyz vf27 = vf26^2; ADDAy.x; MADDz.x vf27.x = ACC + 1*z2; RSQRT;
    // WAITQ; MULq.xyz vf26; SQC2 vf26 -> {(b - a).xyz * Q, a.w}.
    {"direction26",{-27,-28,0x4bc007c3,0x4bdadeac,0x4bdad6ea,0x4b1bd83d,0x4b1bfeca,0x4a7b03be,0x4a0003bf,0x4bc0d69c,-126},11},
    // mod028 4e6654: LQC2 vf4 (point), vf1..vf3 (rows); MULAx/MADDAy/MADDz.xyzw vf4 =
    // (r1*x + r2*y) + r3*z per lane; SQC2 vf4.
    {"rows3",{-5,-2,-3,-4,0x4be409bc,0x4be410bd,0x4be4190a,-104},8},
    // its four-row form (mod001 22b734): LQC2 vf5 (point), vf1..vf4 (rows); MULAx/MADDAy/MADDAz/
    // MADDw.xyzw vf5 = ((r1*x + r2*y) + r3*z) + r4*w (w from the point); SQC2 vf5.
    {"rows4",{-6,-2,-3,-4,-5,0x4be509bc,0x4be510bd,0x4be518be,0x4be5214b,-105},10},
    // mod028 xz length to a GPR (4e657c): LQC2 vf24; MUL.xyz; ADDz.x x = x2 + z2; SQRT;
    // WAITQ; ADDq.x x = 0 + Q; QMFC2 rN = {|xz|, y2, z2, w} (v0, v1, a0 or a1). Ends in the QMFC2, not a store.
    {"xzlength",{-25,0x4bd8c62a,0x4b18c602,0x4a1803bd,0x4a0003bf,0x4b000620,-200},7},
    // its difference form (13 copies, e.g. 4e5fb0): LQC2 vf24 (a), vf25 (b); SUB.xyz vf24 = a - b; then the same.
    {"xzdistance",{-25,-26,0x4bd9c62c,0x4bd8c62a,0x4b18c602,0x4a1803bd,0x4a0003bf,0x4b000620,-200},9},
    // matrix product (mod001 2b84a0): LQC2 vf1..vf4 (rows A), vf5..vf8 (rows B); per row k
    // MULAx/MADDAy/MADDAz/MADDw: A_k = ((B0*A_k.x + B1*A_k.y) + B2*A_k.z) + B3*A_k.w;
    // SQC2 vf1..vf4 (each store publishes its row; the capture lives to the last).
    {"compose",{-2,-3,-4,-5,-6,-7,-8,-9,0x4be129bc,0x4be130bd,0x4be138be,0x4be1404b,0x4be229bc,0x4be230bd,0x4be238be,0x4be2408b,
        0x4be329bc,0x4be330bd,0x4be338be,0x4be340cb,0x4be429bc,0x4be430bd,0x4be438be,0x4be4410b,-101,-102,-103,-104},28}};
struct MacroCapture{std::array<VuWords,32> vf{};uint32_t valid=0,block=0;};
MacroCapture &macroCapture(const Runtime *rt){static std::map<const Runtime *,MacroCapture> captures;return captures[rt];}
// SQC2 vf0: VU0's vf0 is hard-wired to (0, 0, 0, 1.0), so storing it is that
// constant whatever surrounds it (mod027 35fe28/35fe30 clear two rows this way).
bool nativeConstantStore(R5900Context *c,Runtime *rt,uint32_t pc){
    if(!rt || (pc&3))return false;
    uint32_t w;const auto *p=rt->memory.hostPointer(pc,4);if(!p)return false;std::memcpy(&w,p,4);
    if((w>>26)!=0x3e || ((w>>16)&31)!=0)return false;
    if(!c)throw std::runtime_error("native vf0 store has no CPU record");
    const VuWords one{0,0,0,0x3f800000u};
    std::memcpy(record(rt,c->r[(w>>21)&31].uw[0]+uint32_t(int32_t(int16_t(w&0xffff))),16),one.data(),16);
    return true;
}
bool nativeMacroIdiom(R5900Context *c,Runtime *rt,uint32_t pc){
    if(!rt || (pc&3))return false;
    const auto word=[&](uint32_t at,uint32_t &out){const auto *p=rt->memory.hostPointer(at,4);if(!p)return false;std::memcpy(&out,p,4);return true;};
    const auto cop2=[](uint32_t w){const auto op=w>>26;return op==0x12 || op==0x36 || op==0x3e;};
    const auto control=[](uint32_t w){const auto op=w>>26;return (op>=1 && op<=7) || (op>=0x14 && op<=0x17) || (op==0 && ((w&63)==8 || (w&63)==9)) || (op==0x11 && ((w>>21)&31)==8);};
    const auto matches=[](int32_t want,uint32_t w){
        if(want>=0)return w==uint32_t(want);
        if(want==-200)return (w>>21)==0x241 && (w&0xffff)==0xc000;
        if(want<=-100)return w>>26==0x3e && int32_t((w>>16)&31)==-100-want;
        return w>>26==0x36 && int32_t((w>>16)&31)==-1-want;};
    uint32_t self;if(!word(pc,self) || !cop2(self))return false;
    // A jr ra or an unconditional b (beq zero,zero) may stand between the last two
    // words when the idiom's final store is its delay slot (the store always runs).
    const auto returnSlot=[&](const MacroIdiom &idiom,unsigned f,uint32_t at,uint32_t w){
        uint32_t next;return (w==0x03e00008u || (w>>16)==0x1000u) && f+1==idiom.size && word(at+4,next) && matches(idiom.words[f],next);};
    for(const auto &idiom:macroIdioms){
        // Try every position this PC could hold in the idiom.
        for(unsigned k=0;k<idiom.size;++k){
            if(!matches(idiom.words[k],self))continue;
            // Walk back k COP2 words (at most 6 plain EE words between them), then forward.
            uint32_t at=pc,addresses[28];bool ok=true;addresses[k]=pc;
            for(unsigned b=k;b>0 && ok;--b){unsigned gap=0;uint32_t w;
                do{at-=4;if(!word(at,w) || (control(w) && !(gap==0 && b==k && returnSlot(idiom,k,at,w))) || ++gap>7){ok=false;break;}}while(!cop2(w));
                if(ok && !matches(idiom.words[b-1],w))ok=false;else addresses[b-1]=at;}
            at=pc;
            for(unsigned f=k+1;f<idiom.size && ok;++f){unsigned gap=0;uint32_t w;
                do{at+=4;if(!word(at,w) || (control(w) && !returnSlot(idiom,f,at,w)) || ++gap>7){ok=false;break;}}while(!cop2(w));
                if(ok && !matches(idiom.words[f],w))ok=false;else addresses[f]=at;}
            if(!ok)continue;
            if(!c)throw std::runtime_error("native macro idiom has no CPU record");
            auto &capture=macroCapture(rt);const auto first=addresses[0];
            const int32_t want=idiom.words[k];
            const auto address=[&](uint32_t w){return c->r[(w>>21)&31].uw[0]+uint32_t(int32_t(int16_t(w&0xffff)));};
            if(want<0 && want>-100){   // load: capture
                if(k==0 || capture.block!=first){capture.valid=0;capture.block=first;}
                const unsigned n=unsigned(-1-want);capture.vf[n]=vuWords(rt,address(self));capture.valid|=1u<<n;return true;}
            const std::string_view name=idiom.name;
            if(k+1==idiom.size && (self>>21)==0x241 && (name=="xzlength" || name=="xzdistance")){
                // QMFC2 at the end: the result goes to the GPR.
                if(capture.block!=first || !(capture.valid>>24&1) || (name=="xzdistance" && !(capture.valid>>25&1)))
                    throw std::runtime_error(std::format("native {} idiom at {:x} moved without its loads",idiom.name,first));
                auto v=capture.vf[24];if(name=="xzdistance")for(unsigned i=0;i<3;++i)v[i]=vuSub(v[i],capture.vf[25][i]);
                for(unsigned i=0;i<3;++i)v[i]=vuMul(v[i],v[i]);
                const uint32_t savedCsr=_mm_getcsr();uint32_t f=0;const auto root=bozo::ps2f::sqrt(vuAdd(v[0],v[2]),f);_mm_setcsr(savedCsr);
                v[0]=vuAdd(0,root);auto &g=c->r[(self>>16)&31];for(unsigned i=0;i<4;++i)g.uw[i]=v[i];capture.valid=0;return true;}
            if(want>=0){                // arithmetic: folded into the store
                // QMTC2 brings a GPR's quadword into a VF register: captured here.
                if((self>>21)==0x245){if(capture.block!=first)throw std::runtime_error(std::format("native {} idiom at {:x} moved without its loads",idiom.name,first));
                    const auto &g=c->r[(self>>16)&31];const VuWords q{g.uw[0],g.uw[1],g.uw[2],g.uw[3]};const unsigned n=(self>>11)&31;capture.vf[n]=q;capture.valid|=1u<<n;}
                return true;}
            // store
            const auto need=[&](unsigned n){if(capture.block!=first || !(capture.valid>>n&1))throw std::runtime_error(std::format("native {} idiom at {:x} stored without its loads",idiom.name,first));return capture.vf[n];};
            const uint32_t one=0x3f800000u,savedCsr=_mm_getcsr();VuWords out{};
            const std::string_view kind=idiom.name;
            if(kind=="normalize" || kind=="normalize25"){const auto v=need(kind=="normalize"?20:25);uint32_t sq[3];for(unsigned i=0;i<3;++i)sq[i]=vuMul(v[i],v[i]);
                uint32_t f=0;const auto q=bozo::ps2f::rsqrt(one,vuAdd(vuAdd(sq[0],sq[1]),vuMul(one,sq[2])),f);_mm_setcsr(savedCsr);
                for(unsigned i=0;i<3;++i)out[i]=vuMul(v[i],q);out[3]=v[3];}
            else if(kind=="direction26"){const auto a=need(26),b=need(27);uint32_t d[3],sq[3];for(unsigned i=0;i<3;++i){d[i]=vuSub(b[i],a[i]);sq[i]=vuMul(d[i],d[i]);}
                uint32_t f=0;const auto q=bozo::ps2f::rsqrt(one,vuAdd(vuAdd(sq[0],sq[1]),vuMul(one,sq[2])),f);_mm_setcsr(savedCsr);
                for(unsigned i=0;i<3;++i)out[i]=vuMul(d[i],q);out[3]=a[3];}
            else if(kind=="rows4"){const auto v=need(5),r1=need(1),r2=need(2),r3=need(3),r4=need(4);
                for(unsigned i=0;i<4;++i)out[i]=vuAdd(vuAdd(vuAdd(vuMul(r1[i],v[0]),vuMul(r2[i],v[1])),vuMul(r3[i],v[2])),vuMul(r4[i],v[3]));}
            else if(kind=="rows3"){const auto v=need(4),r1=need(1),r2=need(2),r3=need(3);
                for(unsigned i=0;i<4;++i)out[i]=vuAdd(vuAdd(vuMul(r1[i],v[0]),vuMul(r2[i],v[1])),vuMul(r3[i],v[2]));}
            else if(kind=="squared"){const auto v=need(24);for(unsigned i=0;i<3;++i)out[i]=vuMul(v[i],v[i]);
                out[0]=vuAdd(vuAdd(out[0],out[1]),vuMul(one,out[2]));out[3]=v[3];}
            else if(kind=="dot3"){const auto a=need(1),b=need(2);uint32_t m[3];for(unsigned i=0;i<3;++i)m[i]=vuMul(a[i],b[i]);
                out=a;out[0]=vuAdd(vuAdd(m[0],m[1]),m[2]);}
            else if(kind=="compose"){const unsigned row=unsigned(-100-want)-1;const auto a=need(1+row);
                for(unsigned i=0;i<4;++i)out[i]=vuAdd(vuAdd(vuAdd(vuMul(need(5)[i],a[0]),vuMul(need(6)[i],a[1])),vuMul(need(7)[i],a[2])),vuMul(need(8)[i],a[3]));
                _mm_setcsr(savedCsr);std::memcpy(record(rt,address(self),16),out.data(),16);if(row==3)capture.valid=0;return true;}
            else if(kind=="rowpoint"){const auto v=need(15);constexpr unsigned order[4][4]={{0,1,2,3},{1,0,2,3},{2,1,0,3},{3,1,2,0}};
                for(unsigned row=0;row<4;++row){const auto m=need(11+row);uint32_t t[4];for(unsigned i=0;i<4;++i)t[i]=vuMul(m[i],v[i]);
                    out[row]=vuAdd(vuAdd(vuAdd(t[order[row][0]],t[order[row][1]]),vuMul(one,t[order[row][2]])),vuMul(one,t[order[row][3]]));}}
            else if(kind=="homogeneous"){const auto v=need(20);uint32_t f=0;const auto q=bozo::ps2f::div(one,v[3],f);_mm_setcsr(savedCsr);
                for(unsigned i=0;i<4;++i)out[i]=vuMul(v[i],q);}
            else if(kind=="scaledsum"){const auto a=need(1),b=need(2),k=need(3);
                for(unsigned i=0;i<3;++i)out[i]=vuAdd(vuMul(b[i],one),vuMul(a[i],k[0]));out[3]=a[3];}
            else if(kind=="advance"){const auto p=need(1),d=need(2),k=need(3);
                for(unsigned i=0;i<3;++i)out[i]=vuAdd(vuMul(p[i],one),vuMul(d[i],k[0]));out[3]=p[3];}
            else if(kind=="colourscale"){const auto v=need(1),k=need(2);
                for(unsigned i=0;i<3;++i)out[i]=vuMul(v[i],k[0]);out[3]=vuMul(v[3],0);}
            else if(kind=="muladd"){const auto a=need(1),b=need(2),k=need(3);for(unsigned i=0;i<4;++i)out[i]=vuAdd(vuMul(a[i],b[i]),vuMul(k[i],one));}
            else if(kind=="product2"){const auto a=need(1),b=need(2);for(unsigned i=0;i<4;++i)out[i]=vuMul(a[i],b[i]);}
            else if(kind=="product"){const auto a=need(12),b=need(13);for(unsigned i=0;i<4;++i)out[i]=vuMul(a[i],b[i]);}
            else if(kind=="negate"){const auto v=need(1);for(unsigned i=0;i<3;++i)out[i]=vuSub(0,v[i]);out[3]=v[3];}
            else if(kind=="dot"){const auto a=need(20),b=need(21),m=need(22);
                for(unsigned i=0;i<4;++i)out[i]=vuMul(m[i],vuSub(b[i],a[i]));out[0]=vuAdd(vuAdd(out[0],out[1]),vuMul(one,out[2]));}
            else{const auto a=need(20),b=need(21);for(unsigned i=0;i<4;++i){const auto d=vuSub(b[i],a[i]);out[i]=vuMul(d,d);}}
            _mm_setcsr(savedCsr);
            std::memcpy(record(rt,address(self),16),out.data(),16);capture.valid=0;return true;
        }
    }
    return false;
}
// Particle step (mod003 3652d0, four loop variants): the step dt arrives once per
// call through QMTC2 into vf3 (365350 from v1 = *(35dee8), 36554c from s1). Per
// particle: LQC2 vf1 (position), vf2 (velocity); VADDA.xyz ACC = vf1 + vf0;
// VMADDx.xyz vf1 = ACC + vf2*vf3.x; SQC2 vf1 twice -> {p + v*dt (xyz), p.w}.
// Every site and its exact word are listed; the result exists only after both
// loads of the same particle, and only stores read it. Its copy 38d900 (a
// second particle class: record strides x4/x32 instead of x8/x16, VU0 words
// identical) has every site 0x28630 higher.
struct ParticleSite{uint32_t pc,word;char role;};   // q: dt, p/v: loads, a: ACC, m: result, s: store
constexpr ParticleSite particleSites[]={
    {0x365350,0x48a31800,'q'},{0x36554c,0x48b11800,'q'},
    {0x3653c4,0xda010000,'p'},{0x3653c8,0xd8620000,'v'},{0x3653e8,0x4bc00abc,'a'},{0x3653ec,0x4bc31048,'m'},{0x3653f8,0xf8610000,'s'},{0x3653fc,0xfa010000,'s'},
    {0x3654ac,0xd8610000,'p'},{0x3654b0,0xd9220000,'v'},{0x3654d0,0x4bc00abc,'a'},{0x3654d4,0x4bc31048,'m'},{0x3654e0,0xf9210000,'s'},{0x3654e4,0xf8610000,'s'},
    {0x3655e8,0xda610000,'p'},{0x3655f0,0xdaa20000,'v'},{0x3656f4,0x4bc00abc,'a'},{0x3656f8,0x4bc31048,'m'},{0x365704,0xfa410000,'s'},{0x36570c,0xfa610000,'s'},
    {0x3657e4,0xda610000,'p'},{0x3657e8,0xda020000,'v'},{0x3658fc,0x4bc00abc,'a'},{0x365900,0x4bc31048,'m'},{0x36590c,0xfa010000,'s'},{0x365914,0xfa610000,'s'}};
// A third particle class's step, 409700 (called from 3f4df4; records of 0xb0
// bytes, position at +80, velocity at +90): the same VU0 words in its own
// layout (each load pair 8 bytes later than 3652d0's), so its sites are listed
// exactly. dt: 409780 from v1 = *(35dee8), 4099a8 from s0.
constexpr ParticleSite particleSites409700[]={
    {0x409780,0x48a31800,'q'},{0x4099a8,0x48b01800,'q'},
    {0x4097fc,0xd9410000,'p'},{0x409800,0xd8620000,'v'},{0x409828,0x4bc00abc,'a'},{0x40982c,0x4bc31048,'m'},{0x409838,0xf8610000,'s'},{0x40983c,0xf9410000,'s'},
    {0x4098ec,0xd8610000,'p'},{0x4098f0,0xd9220000,'v'},{0x40991c,0x4bc00abc,'a'},{0x409920,0x4bc31048,'m'},{0x40992c,0xf9210000,'s'},{0x409930,0xf8610000,'s'},
    {0x409a58,0xdac10000,'p'},{0x409a60,0xda020000,'v'},{0x409bbc,0x4bc00abc,'a'},{0x409bc0,0x4bc31048,'m'},{0x409bcc,0xfa010000,'s'},{0x409bd4,0xfac10000,'s'},
    {0x409ca8,0xdaa10000,'p'},{0x409cac,0xda020000,'v'},{0x409e1c,0x4bc00abc,'a'},{0x409e20,0x4bc31048,'m'},{0x409e2c,0xfa010000,'s'},{0x409e34,0xfaa10000,'s'}};
// A fourth class's step, 409fb0 (map 79 of the Lost Forest; records of 0x120
// bytes, position at +f0, velocity at +100): the same words and roles again.
// dt: 40a030 from v1 = *(35dee8), 40a238 from s0.
constexpr ParticleSite particleSites409fb0[]={
    {0x40a030,0x48a31800,'q'},{0x40a238,0x48b01800,'q'},
    {0x40a0a4,0xd9410000,'p'},{0x40a0a8,0xd8620000,'v'},{0x40a0c8,0x4bc00abc,'a'},{0x40a0cc,0x4bc31048,'m'},{0x40a0d8,0xf8610000,'s'},{0x40a0dc,0xf9410000,'s'},
    {0x40a184,0xd8610000,'p'},{0x40a188,0xd9220000,'v'},{0x40a1ac,0x4bc00abc,'a'},{0x40a1b0,0x4bc31048,'m'},{0x40a1bc,0xf9210000,'s'},{0x40a1c0,0xf8610000,'s'},
    {0x40a2e0,0xdac10000,'p'},{0x40a2e8,0xda020000,'v'},{0x40a43c,0x4bc00abc,'a'},{0x40a440,0x4bc31048,'m'},{0x40a44c,0xfa010000,'s'},{0x40a454,0xfac10000,'s'},
    {0x40a520,0xdaa10000,'p'},{0x40a524,0xda020000,'v'},{0x40a68c,0x4bc00abc,'a'},{0x40a690,0x4bc31048,'m'},{0x40a69c,0xfa010000,'s'},{0x40a6a4,0xfaa10000,'s'}};
struct ParticleState{uint32_t dt=0;bool step=false;VuWords position{},velocity{},result{};uint8_t have=0;};
ParticleState &particleState(const Runtime *rt){static std::map<const Runtime *,ParticleState> states;return states[rt];}
bool nativeParticleStep(R5900Context *c,Runtime *rt,uint32_t pc){
    // Byte-identical copies of 3652d0: 38d900 (+28630) and the forest emitter 44b8f0 (+e6620).
    constexpr uint32_t kCopies[]={0,0x38d900-0x3652d0,0x44b8f0-0x3652d0};
    if(!rt)return false;
    const ParticleSite *site=nullptr;
    // mod001 holds byte-identical copies (only module-local jal targets differ) of both,
    // run by the battle's emitters: 284050 = 409700 - 1856b0, 28a8a0 = 409fb0 - 17f710.
    struct Table{const ParticleSite *begin,*end;uint32_t lo,hi,offset;};
    constexpr Table tables[]={{std::begin(particleSites409700),std::end(particleSites409700),0x409780,0x409e34,0},
        {std::begin(particleSites409fb0),std::end(particleSites409fb0),0x40a030,0x40a6a4,0},
        {std::begin(particleSites409700),std::end(particleSites409700),0x409780,0x409e34,0x1856b0},
        {std::begin(particleSites409fb0),std::end(particleSites409fb0),0x40a030,0x40a6a4,0x17f710}};
    bool tabled=false;
    for(const auto &t:tables){const uint32_t listed=pc+t.offset;
        if(listed>=t.lo && listed<=t.hi){tabled=true;for(auto *s=t.begin;s!=t.end;++s)if(s->pc==listed)site=s;}}
    if(!tabled){
        for(const auto copy:kCopies){const uint32_t listed=pc-copy;
            if(listed>=0x365350 && listed<=0x365914)for(const auto &s:particleSites)if(s.pc==listed)site=&s;}
    }
    if(!site)return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    if(!c)throw std::runtime_error("native particle step has no CPU record");
    auto &st=particleState(rt);
    const auto address=[&]{return c->r[(word>>21)&31].uw[0]+uint32_t(int32_t(int16_t(word&0xffff)));};
    switch(site->role){
    case 'q':st.dt=c->r[(word>>16)&31].uw[0];st.step=true;st.have=0;return true;
    case 'p':st.position=vuWords(rt,address());st.have=1;return true;
    case 'v':if(st.have!=1)throw std::runtime_error(std::format("native particle velocity at {:x} without its position",pc));
        st.velocity=vuWords(rt,address());st.have=2;return true;
    case 'a':if(st.have!=2)throw std::runtime_error(std::format("native particle accumulate at {:x} without its loads",pc));return true;
    case 'm':{if(st.have!=2 || !st.step)throw std::runtime_error(std::format("native particle step at {:x} without its loads or dt",pc));
        const auto savedCsr=_mm_getcsr();
        for(unsigned i=0;i<3;++i)st.result[i]=vuAdd(vuAdd(st.position[i],0),vuMul(st.velocity[i],st.dt));
        st.result[3]=st.position[3];_mm_setcsr(savedCsr);st.have=3;return true;}
    default:if(st.have!=3)throw std::runtime_error(std::format("native particle store at {:x} without its step",pc));
        std::memcpy(record(rt,address(),16),st.result.data(),16);return true;
    }
}
// Particle box (mod003 3649f0, an emitter method called from the driver at 22ed38):
// LQC2 vf1 (364a84, the point just copied to the output at a2); VMINI.xyz
// vf26 = min(vf26, vf1) (364ac4); VMAX.xyz vf27 = max(vf27, vf1) (364ac8).
// vf26/vf27 are the driver's box (loaded at 22ec9c/22eca0, published at 22efd8):
// the point joins NativeParticles' explicit bounds of that driver frame (sp+40).
// A second emitter method, 38cea0 (a leaf: the driver frame is its sp), has the
// same three words at 38cfec/38d010/38d018 (VMAX in the delay slot of beqz a3).
struct ParticleBoxState{bool loaded=false,included=false;uint32_t frame=0,emitter=0;std::array<float,4> point{};};
ParticleBoxState &particleBoxState(const Runtime *rt){static std::map<const Runtime *,ParticleBoxState> states;return states[rt];}
bool nativeParticleBox(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Box{uint32_t load,min,max,frameOffset,loadWord;unsigned source,emitter;};
    // A third, 44aef0 (the forest emitter class; frame 0x40 as 3649f0). A fourth, mod028
    // 48a820 (a battle emitter's +18, frame 0xf0): the point is the record's +10 at v0
    // (48a9cc), the emitter is kept in s6 (a0 is reused), MINI/MAX at 48aa10/48aa14.
    // mod001 2847e0 (an emitter method called from the driver at 22ed20, frame 0x240, the
    // emitter kept in s5): ten copies of the triplet, one per point kind, the point loaded
    // from the output just written (r3) or from the record (r4).
    constexpr Box boxes[]={{0x364a84,0x364ac4,0x364ac8,0x40,0xd8c10000,6,4},{0x38cfec,0x38d010,0x38d018,0,0xd8c10000,6,4},
        {0x44af7c,0x44afa4,0x44afa8,0x40,0xd8c10000,6,4},{0x48a9cc,0x48aa10,0x48aa14,0xf0,0xd8410000,2,22},
        {0x284934,0x28498c,0x284990,0x240,0xd8610000,3,21},{0x2849f0,0x284a4c,0x284a50,0x240,0xd8610000,3,21},{0x284b18,0x284b74,0x284b78,0x240,0xd8810000,4,21},{0x284bd4,0x284c30,0x284c34,0x240,0xd8610000,3,21},{0x284cf0,0x284d1c,0x284d20,0x240,0xd8610000,3,21},
        {0x284d80,0x284db8,0x284dbc,0x240,0xd8610000,3,21},{0x284e78,0x284eb0,0x284eb4,0x240,0xd8610000,3,21},{0x284f14,0x284f58,0x284f5c,0x240,0xd8610000,3,21},{0x285050,0x2850a8,0x2850ac,0x240,0xd8810000,4,21},{0x28510c,0x285150,0x285154,0x240,0xd8610000,3,21}};
    const Box *box=nullptr;unsigned role=0;
    for(const auto &b:boxes){if(pc==b.load){box=&b;role=0;}else if(pc==b.min){box=&b;role=1;}else if(pc==b.max){box=&b;role=2;}}
    if(!rt || !box)return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=(role==0?box->loadWord:role==1?0x4bc1d6afu:0x4bc1deebu))return false;   // another overlay at these addresses
    auto *d=dynamic_cast<bozo::ee::Dispatcher *>(rt->services);auto *particles=d?dynamic_cast<ParticleOwner *>(&d->host()):nullptr;
    if(!c || !particles)throw std::runtime_error("native particle box requires its bounds owner");
    auto &st=particleBoxState(rt);const auto frame=c->r[29].uw[0]+box->frameOffset,emitter=c->r[box->emitter].uw[0];
    if(role==0){
        particles->nativeParticles().validateEmitter(frame,emitter);
        std::memcpy(st.point.data(),record(rt,c->r[box->source].uw[0],16),16);st=ParticleBoxState{true,false,frame,emitter,st.point};return true;}
    if(!st.loaded || st.frame!=frame || st.emitter!=emitter || st.included!=(role==2))
        throw std::runtime_error(std::format("native particle box at {:x} out of order",pc));
    if(role==1){particles->nativeParticles().includePoint(frame,emitter,st.point);st.included=true;}
    else st=ParticleBoxState{};
    return true;
}
// Trail emitter step (mod003 40b080, an emitter method called from the driver at
// 22ed38; driver frame = sp + 170). Per live particle of 0xb0 bytes (record R =
// [emitter+14] + index*b0): position +10, colour +20, velocity +30, accel +40,
// anchor +50 (w: age), colour rate +60, rate change +70. dt = *(35dee8) (QMTC2
// vf5), drift = emitter+c0 (vf20), clamp = the 255s at sp+160 (vf29).
//   optional curve (emitter+68 table, 1/60 rows): p.xyz = (p + 0) + row * f
//   P.xyz = ((p + 0) + drift*1) + v*dt            -> out[0], record +10
//   C     = min(max((c + 0) + rate*dt, 0), 255)   -> out[0]+10, record +20
//   A.xyz = (a + 0) + v*dt,  A.w = a.w + dt        -> record +50
//   V.xyz = (v + 0) + accel*dt, V.w = v.w - dt     -> record +30
//   R     = rate*1 + change*dt                     -> record +60
// P joins the driver's box (VMINI/VMAX vf26/vf27). Mode +d0: 2 publishes
// (|A|^2 as (x2+y2)+1*z2, y2, z2, A.w) at sp+150; 5 publishes the emitter+80
// rows applied to P (x, y, z, then row 3 * 1) at sp+140; the scalar code
// decides deletion. Trail +d7 (1, 2) or a non-unit +54 (3) rebuilds the
// particle's output points 1..n-1 from out[0]: d = next - prev; kinds 1/2
// normalise d with RSQRT (Q = 1/sqrt((x2+y2)+1*z2)); anchor.xyz += kind 2:
// d*1 + dir*len, kind 1: dir*len, kind 3: d*len; each new anchor is stored
// and joins the box. len = emitter+54. Sequential VU0 macro semantics (ps2f,
// nearest; MAX/MINI with ps2f::max/min).
struct TrailSite{uint32_t pc,word;char role;uint8_t param;};
constexpr TrailSite trailSites[]={
    {0x40b124,0x48a22800,'T',0},{0x40b12c,0xd8540000,'T',1},{0x40b168,0x4be00798,'T',2},{0x40b170,0xd85d0000,'T',3},
    {0x40b258,0xda410000,'L',0},{0x40b260,0xda220000,'L',1},{0x40b268,0xd9040000,'L',2},{0x40b270,0xdac30000,'L',3},
    {0x40b278,0xda060000,'L',4},{0x40b280,0xd8670000,'L',5},{0x40b288,0xd8480000,'L',6},
    {0x40b318,0xd88b0000,'E',0},{0x40b324,0x48a46000,'E',1},{0x40b328,0x4bc00abc,'E',2},{0x40b32c,0x4bcc5848,'E',3},
    {0x40b364,0x4bc00abc,'k',0},{0x40b368,0x4bc0a0bf,'k',1},{0x40b36c,0x4bc51048,'k',2},{0x40b370,0x4bfe32bc,'k',3},{0x40b374,0x4be53988,'k',4},
    {0x40b378,0x4bc022bc,'k',5},{0x40b37c,0x4bc51108,'k',6},{0x40b380,0x4bc012bc,'k',7},{0x40b384,0x4bc51888,'k',8},{0x40b388,0x4a251084,'k',9},
    {0x40b38c,0x4be03190,'k',10},{0x40b390,0x4a252100,'k',11},{0x40b394,0x4be039bf,'k',12},{0x40b398,0x4be541c8,'k',13},{0x40b39c,0x4bc1d6af,'k',14},
    {0x40b3a0,0x4bfd3194,'k',15},{0x40b3a4,0x4bc1deeb,'k',16},
    {0x40b3a8,0xfae10000,'O',0},{0x40b3ac,0xfae60010,'O',1},
    // Record stores, the same five in every mode branch (1, 3, 2, 5, 6, other).
    {0x40b410,0xf9040000,'R',0},{0x40b414,0xfa410000,'R',1},{0x40b418,0xfa220000,'R',2},{0x40b41c,0xfa060000,'R',3},{0x40b420,0xf8670000,'R',4},
    {0x40b488,0xf9040000,'R',0},{0x40b48c,0xfa410000,'R',1},{0x40b490,0xfa220000,'R',2},{0x40b494,0xfa060000,'R',3},{0x40b498,0xf8670000,'R',4},
    {0x40b514,0xf9040000,'R',0},{0x40b51c,0xfa410000,'R',1},{0x40b520,0xfa220000,'R',2},{0x40b524,0xfa060000,'R',3},{0x40b528,0xf8670000,'R',4},
    {0x40b5cc,0xf9040000,'R',0},{0x40b5d0,0xfa410000,'R',1},{0x40b5d4,0xfa220000,'R',2},{0x40b5d8,0xfa060000,'R',3},{0x40b5dc,0xf8670000,'R',4},
    {0x40b650,0xf9040000,'R',0},{0x40b654,0xfa410000,'R',1},{0x40b658,0xfa220000,'R',2},{0x40b65c,0xfa060000,'R',3},{0x40b660,0xf8670000,'R',4},
    {0x40b6f0,0xf9040000,'R',0},{0x40b6f4,0xfa410000,'R',1},{0x40b6f8,0xfa220000,'R',2},{0x40b6fc,0xfa060000,'R',3},{0x40b700,0xf8670000,'R',4},
    // Mode 2: ones, square (after the anchor store), (x2+y2), +1*z2, publish.
    {0x40b510,0x4bc007c3,'2',0},{0x40b518,0x4bc4212a,'2',1},{0x40b52c,0x4b04203d,'2',2},{0x40b530,0x4b04f90a,'2',3},{0x40b53c,0xf8440000,'2',4},
    // Mode 5: four rows, the product chain, publish.
    {0x40b5ac,0xd8480000,'5',0},{0x40b5b0,0xd8490010,'5',1},{0x40b5b4,0xd84a0020,'5',2},{0x40b5b8,0xd84b0030,'5',3},
    {0x40b5bc,0x4be141bc,'5',4},{0x40b5c0,0x4be148bd,'5',5},{0x40b5c4,0x4be150be,'5',6},{0x40b5c8,0x4be05c8b,'5',7},{0x40b5e4,0xf8520000,'5',8},
    // Trails: head, len, anchor = head (param = kind), then per point: load ('p'), steps ('i'), store ('a'), box ('b').
    {0x40b738,0xd8810000,'h',2},{0x40b748,0x48a21800,'n',2},{0x40b74c,0x4be40b3c,'v',2},{0x40b768,0xd8420000,'p',2},
    {0x40b76c,0x4be111ac,'i',1},{0x40b770,0x4be1133c,'i',2},{0x40b774,0x4bc007c3,'i',3},{0x40b778,0x4bc631ea,'i',4},{0x40b77c,0x4b07383d,'i',5},{0x40b780,0x4b07f9ca,'i',6},
    {0x40b784,0x4a6703be,'i',7},{0x40b788,0x4a0003bf,'i',8},{0x40b78c,0x4bc0321c,'i',9},{0x40b790,0x4bc022bc,'i',10},{0x40b794,0x4bc030bf,'i',11},{0x40b798,0x4bc34108,'i',12},
    {0x40b79c,0xf8440000,'a',13},{0x40b7a0,0x4bc4d6af,'b',14},{0x40b7a4,0x4bc4deeb,'b',15},
    {0x40b7d8,0xd8810000,'h',1},{0x40b7e8,0x48a21800,'n',1},{0x40b7ec,0x4be40b3c,'v',1},{0x40b808,0xd8420000,'p',1},
    {0x40b80c,0x4be111ac,'i',1},{0x40b810,0x4be1133c,'i',2},{0x40b814,0x4bc007c3,'i',3},{0x40b818,0x4bc631ea,'i',4},{0x40b81c,0x4b07383d,'i',5},{0x40b820,0x4b07f9ca,'i',6},
    {0x40b824,0x4a6703be,'i',7},{0x40b828,0x4a0003bf,'i',8},{0x40b82c,0x4bc0319c,'i',9},{0x40b830,0x4bc022bc,'i',10},{0x40b834,0x4bc33108,'i',11},
    {0x40b838,0xf8440000,'a',12},{0x40b83c,0x4bc4d6af,'b',13},{0x40b840,0x4bc4deeb,'b',14},
    {0x40b878,0xd8810000,'h',3},{0x40b888,0x48a21800,'n',3},{0x40b88c,0x4be40b3c,'v',3},{0x40b8a8,0xd8420000,'p',3},
    {0x40b8ac,0x4be111ac,'i',1},{0x40b8b0,0x4be1133c,'i',2},{0x40b8b4,0x4bc022bc,'i',3},{0x40b8b8,0x4bc33108,'i',4},
    {0x40b8bc,0xf8440000,'a',5},{0x40b8c0,0x4bc4d6af,'b',6},{0x40b8c4,0x4bc4deeb,'b',7}};
// Orbit emitter step (mod003 40b990, called from the same driver 22ed38; frame
// = sp + 1e0). The trail emitter's step on records of 0x120 bytes, plus an
// orbit: radius +b0 (vf9), angles +80 (vf10), radius rate +c0 (vf11), angle
// rate +90 (vf13), radius accel +d0 (vf12), angle accel +a0 (vf14), last offset
// +e0. Before the main step (after the position load):
//   radius.xyz = (radius + 0) + radiusRate*dt -> +b0
//   angles.xyz = (angles + 0) + angleRate*dt  -> +80
// then scalar code (sine/cosine 213fd0) writes the new offset to +e0 and the
// change since the last one to sp+1a0 (vf15), which joins P and A:
//   P.xyz = (((p + 0) + offset*1) + drift*1) + v*dt
//   A.xyz = ((a + 0) + offset*1) + v*dt
// and after the clamp: radiusRate.xyz += radiusAccel*dt, angleRate.xyz +=
// angleAccel*dt, stored with the records (+c0, +90; before the rows in mode 5).
// Output = *(sp+160) (advanced by the point count s6*0x30 before the trail);
// mode 2 publishes at sp+190, mode 5 at sp+180. Curve index a3, trails as 40b080.
constexpr TrailSite orbitSites[]={
    {0x40ba48,0x48a22800,'T',0},{0x40ba50,0xd8540000,'T',1},{0x40ba8c,0x4be00798,'T',2},{0x40ba94,0xd85d0000,'T',3},
    {0x40bb68,0xda410000,'L',0},
    {0x40bb80,0xd8490000,'W',0},{0x40bb88,0xd84a0000,'W',1},{0x40bb90,0xdaeb0000,'W',2},{0x40bb98,0xdbcd0000,'W',3},
    {0x40bba4,0x4bc04abc,'W',4},{0x40bba8,0x4bc55a48,'W',5},{0x40bbac,0x4bc052bc,'W',6},{0x40bbb0,0x4bc56a88,'W',7},
    {0x40bbb8,0xf8490000,'W',8},{0x40bbc0,0xf84a0000,'W',9},
    {0x40bc54,0xda220000,'L',1},{0x40bc5c,0xd9040000,'L',2},{0x40bc6c,0xd8430000,'L',3},{0x40bc74,0xda060000,'L',4},{0x40bc7c,0xd8670000,'L',5},
    {0x40bc84,0xd8480000,'L',6},{0x40bc8c,0xd84f0000,'L',7},{0x40bc94,0xd84c0000,'L',8},{0x40bc9c,0xd84e0000,'L',9},{0x40bca0,0xdaeb0000,'L',10},{0x40bca4,0xdbcd0000,'L',11},
    {0x40bd30,0xd8900000,'E',0},{0x40bd3c,0x48a48800,'E',1},{0x40bd40,0x4bc00abc,'E',2},{0x40bd44,0x4bd18048,'E',3},
    {0x40bd7c,0x4bc00abc,'k',0},{0x40bd80,0x4bc078bf,'k',1},{0x40bd84,0x4bc0a0bf,'k',2},{0x40bd88,0x4bc51048,'k',3},{0x40bd8c,0x4bfe32bc,'k',4},{0x40bd90,0x4be53988,'k',5},
    {0x40bd94,0x4bc022bc,'k',6},{0x40bd98,0x4bc078bf,'k',7},{0x40bd9c,0x4bc51108,'k',8},{0x40bda0,0x4bc012bc,'k',9},{0x40bda4,0x4bc51888,'k',10},{0x40bda8,0x4a251084,'k',11},
    {0x40bdac,0x4be03190,'k',12},{0x40bdb0,0x4a252100,'k',13},{0x40bdb4,0x4be039bf,'k',14},{0x40bdb8,0x4be541c8,'k',15},{0x40bdbc,0x4bc1d6af,'k',16},{0x40bdc0,0x4bfd3194,'k',17},
    {0x40bdc4,0x4bc1deeb,'k',18},{0x40bdc8,0x4bc05abc,'k',19},{0x40bdcc,0x4bc562c8,'k',20},{0x40bdd0,0x4bc06abc,'k',21},{0x40bdd4,0x4bc57348,'k',22},
    {0x40bddc,0xf8810000,'O',0},{0x40bde4,0xf8860010,'O',1},
    // Record stores (param = A, P, V, C, R, radius rate, angle rate) in each mode branch.
    {0x40be50,0xf9040000,'R',0},{0x40be54,0xfa410000,'R',1},{0x40be58,0xfa220000,'R',2},{0x40be5c,0xfa060000,'R',3},{0x40be60,0xf8670000,'R',4},{0x40be64,0xfaeb0000,'R',5},{0x40be68,0xfbcd0000,'R',6},
    {0x40bed8,0xf9040000,'R',0},{0x40bedc,0xfa410000,'R',1},{0x40bee0,0xfa220000,'R',2},{0x40bee4,0xfa060000,'R',3},{0x40bee8,0xf8670000,'R',4},{0x40beec,0xfaeb0000,'R',5},{0x40bef0,0xfbcd0000,'R',6},
    {0x40bf6c,0xf9040000,'R',0},{0x40bf74,0xfa410000,'R',1},{0x40bf78,0xfa220000,'R',2},{0x40bf7c,0xfa060000,'R',3},{0x40bf80,0xf8670000,'R',4},{0x40bf84,0xfaeb0000,'R',5},{0x40bf88,0xfbcd0000,'R',6},
    {0x40c010,0xfaeb0000,'R',5},{0x40c014,0xfbcd0000,'R',6},
    {0x40c03c,0xf9040000,'R',0},{0x40c040,0xfa410000,'R',1},{0x40c044,0xfa220000,'R',2},{0x40c048,0xfa060000,'R',3},{0x40c04c,0xf8670000,'R',4},
    {0x40c0c8,0xf9040000,'R',0},{0x40c0cc,0xfa410000,'R',1},{0x40c0d0,0xfa220000,'R',2},{0x40c0d4,0xfa060000,'R',3},{0x40c0d8,0xf8670000,'R',4},{0x40c0dc,0xfaeb0000,'R',5},{0x40c0e0,0xfbcd0000,'R',6},
    {0x40c178,0xf9040000,'R',0},{0x40c17c,0xfa410000,'R',1},{0x40c180,0xfa220000,'R',2},{0x40c184,0xfa060000,'R',3},{0x40c188,0xf8670000,'R',4},{0x40c18c,0xfaeb0000,'R',5},{0x40c190,0xfbcd0000,'R',6},
    {0x40bf68,0x4bc007c3,'2',0},{0x40bf70,0x4bc4212a,'2',1},{0x40bf8c,0x4b04203d,'2',2},{0x40bf90,0x4b04f90a,'2',3},{0x40bf9c,0xf8440000,'2',4},
    {0x40c01c,0xd8480000,'5',0},{0x40c020,0xd8490010,'5',1},{0x40c024,0xd84a0020,'5',2},{0x40c028,0xd84b0030,'5',3},
    {0x40c02c,0x4be141bc,'5',4},{0x40c030,0x4be148bd,'5',5},{0x40c034,0x4be150be,'5',6},{0x40c038,0x4be05c8b,'5',7},{0x40c054,0xf8520000,'5',8},
    {0x40c1cc,0xd8810000,'h',2},{0x40c1dc,0x48a21800,'n',2},{0x40c1e0,0x4be40b3c,'v',2},{0x40c200,0xd8420000,'p',2},
    {0x40c204,0x4be111ac,'i',1},{0x40c208,0x4be1133c,'i',2},{0x40c20c,0x4bc007c3,'i',3},{0x40c210,0x4bc631ea,'i',4},{0x40c214,0x4b07383d,'i',5},{0x40c218,0x4b07f9ca,'i',6},
    {0x40c21c,0x4a6703be,'i',7},{0x40c220,0x4a0003bf,'i',8},{0x40c224,0x4bc0321c,'i',9},{0x40c228,0x4bc022bc,'i',10},{0x40c22c,0x4bc030bf,'i',11},{0x40c230,0x4bc34108,'i',12},
    {0x40c234,0xf8440000,'a',13},{0x40c238,0x4bc4d6af,'b',14},{0x40c23c,0x4bc4deeb,'b',15},
    {0x40c274,0xd8810000,'h',1},{0x40c284,0x48a21800,'n',1},{0x40c288,0x4be40b3c,'v',1},{0x40c2a8,0xd8420000,'p',1},
    {0x40c2ac,0x4be111ac,'i',1},{0x40c2b0,0x4be1133c,'i',2},{0x40c2b4,0x4bc007c3,'i',3},{0x40c2b8,0x4bc631ea,'i',4},{0x40c2bc,0x4b07383d,'i',5},{0x40c2c0,0x4b07f9ca,'i',6},
    {0x40c2c4,0x4a6703be,'i',7},{0x40c2c8,0x4a0003bf,'i',8},{0x40c2cc,0x4bc0319c,'i',9},{0x40c2d0,0x4bc022bc,'i',10},{0x40c2d4,0x4bc33108,'i',11},
    {0x40c2d8,0xf8440000,'a',12},{0x40c2dc,0x4bc4d6af,'b',13},{0x40c2e0,0x4bc4deeb,'b',14},
    {0x40c31c,0xd8810000,'h',3},{0x40c32c,0x48a21800,'n',3},{0x40c330,0x4be40b3c,'v',3},{0x40c350,0xd8420000,'p',3},
    {0x40c354,0x4be111ac,'i',1},{0x40c358,0x4be1133c,'i',2},{0x40c35c,0x4bc022bc,'i',3},{0x40c360,0x4bc33108,'i',4},
    {0x40c364,0xf8440000,'a',5},{0x40c368,0x4bc4d6af,'b',6},{0x40c36c,0x4bc4deeb,'b',7}};
// The two layouts: first/last site, frame = sp + frame, record stride, loads,
// main-step sites, record stores, mode 2/5 publication (sp+), curve index reg.
// offset: a byte-identical copy in another module runs at pc = listed - offset (mod001's
// 2929e0 is mod003's trail emitter 1786a0 lower, 293da0 its orbit emitter 177bf0 lower;
// only their module-local jal targets differ).
struct TrailLayout{uint32_t lo,hi,frame,stride;const TrailSite *begin,*end;bool orbit;unsigned loads,main,records;uint32_t mode2,mode5;unsigned curveIndex;uint32_t offset=0;};
constexpr TrailLayout trailLayouts[]={
    {0x40b124,0x40b8c4,0x170,0xb0,std::begin(trailSites),std::end(trailSites),false,7,17,5,0x150,0x140,9},
    {0x40ba48,0x40c36c,0x1e0,0x120,std::begin(orbitSites),std::end(orbitSites),true,12,23,7,0x190,0x180,7},
    {0x40b124,0x40b8c4,0x170,0xb0,std::begin(trailSites),std::end(trailSites),false,7,17,5,0x150,0x140,9,0x1786a0},
    {0x40ba48,0x40c36c,0x1e0,0x120,std::begin(orbitSites),std::end(orbitSites),true,12,23,7,0x190,0x180,7,0x177bf0}};
struct TrailState{
    const TrailLayout *layout=nullptr;unsigned setup=0;uint32_t frame=0,emitter=0,dt=0;VuWords drift{},clamp{};
    uint32_t record=0,output=0;unsigned loads=0,orbit=0,curve=0,main=0,outputs=0,stored=0,extra=0,extraStep=0;
    VuWords p{},v{},a{},accel{},colour{},rate{},change{},row{},P{},C{},A{},V{},R{},published{};std::array<VuWords,4> rows{};uint32_t fraction=0;
    VuWords radius{},angles{},radiusRate{},angleRate{},radiusAccel{},angleAccel{},offset{},RR{},AR{};
    unsigned trail=0,trailStep=0,trailIndex=0;uint32_t length=0,next=0;VuWords head{},prev{},anchor{};
};
TrailState &trailState(const Runtime *rt){static std::map<const Runtime *,TrailState> states;return states[rt];}
bool nativeTrailEmitter(R5900Context *c,Runtime *rt,uint32_t pc){
    if(!rt)return false;
    const TrailLayout *layout=nullptr;const TrailSite *site=nullptr;
    for(const auto &l:trailLayouts){const uint32_t listed=pc+l.offset;
        if(listed>=l.lo && listed<=l.hi)for(auto *s=l.begin;s!=l.end;++s)if(s->pc==listed){layout=&l;site=s;}}
    if(!site)return false;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(word!=site->word)return false;   // another overlay at these addresses
    auto *d=dynamic_cast<bozo::ee::Dispatcher *>(rt->services);auto *particles=d?dynamic_cast<ParticleOwner *>(&d->host()):nullptr;
    if(!c || !particles)throw std::runtime_error("native trail emitter requires its bounds owner");
    struct ArithmeticScope{unsigned saved=_mm_getcsr();~ArithmeticScope(){_mm_setcsr(saved);}} arithmetic;
    auto &st=trailState(rt);
    const auto fail=[&](const char *what){st=TrailState{};throw std::runtime_error(std::format("native trail emitter at {:x}: {}",pc,what));};
    const auto reg=[&](unsigned r){return c->r[r].uw[0];};
    const auto address=[&]{return reg((word>>21)&31)+uint32_t(int32_t(int16_t(word&0xffff)));};
    const auto put=[&](uint32_t at,const VuWords &v){std::memcpy(record(rt,at,16),v.data(),16);};
    const auto field=[&](uint32_t at){const auto *p=rt->memory.hostPointer(at,4);if(!p)fail("scalar field outside RAM");uint32_t v;std::memcpy(&v,p,4);return v;};
    constexpr uint32_t one=0x3f800000u;
    const auto vmax=[](uint32_t a,uint32_t b){return bozo::ps2f::max(a,b);};
    const auto vmin=[](uint32_t a,uint32_t b){return bozo::ps2f::min(a,b);};
    const auto include=[&](const VuWords &v){std::array<float,4> f;for(unsigned i=0;i<4;++i)f[i]=std::bit_cast<float>(v[i]);particles->nativeParticles().includePoint(st.frame,st.emitter,f);};
    const auto sumSquares=[&](const VuWords &v){return vuAdd(vuAdd(vuMul(v[0],v[0]),vuMul(v[1],v[1])),vuMul(one,vuMul(v[2],v[2])));};
    const auto scaled=[&](const VuWords &base,const VuWords &rate){VuWords r=base;for(unsigned i=0;i<3;++i)r[i]=vuAdd(vuAdd(base[i],0),vuMul(rate[i],st.dt));return r;};
    const auto outputBase=[&]{return layout->orbit?field(reg(29)+0x160):reg(23);};
    const unsigned allStored=(1u<<layout->records)-1;
    if(site->role=='T'){
        if(site->param==0){
            st=TrailState{};st.layout=layout;st.frame=reg(29)+layout->frame;st.emitter=reg(21);particles->nativeParticles().validateEmitter(st.frame,st.emitter);
            if(reg(2)!=field(0x35dee8))fail("timestep source");st.dt=reg(2);st.setup=1;return true;}
        if(st.layout!=layout || st.setup!=site->param || reg(21)!=st.emitter || reg(29)+layout->frame!=st.frame)fail("set-up order");
        if(site->param==1){if(reg(2)!=st.emitter+0xc0)fail("drift source");st.drift=vuWords(rt,address());}
        if(site->param==3){if(reg(2)!=st.frame-0x10)fail("clamp source");st.clamp=vuWords(rt,address());}
        ++st.setup;return true;
    }
    if(st.layout!=layout || st.setup!=4 || reg(21)!=st.emitter || reg(29)+layout->frame!=st.frame)fail("site outside its set-up call");
    switch(site->role){
    case 'L':{
        // Record offsets (0 = the stack offset sp+1a0 of the orbit layout).
        constexpr uint32_t offsets[12]={0x10,0x30,0x50,0x40,0x20,0x60,0x70,0,0xd0,0xa0,0xc0,0x90};
        if(site->param==0){
            if(st.loads && st.stored!=allStored)fail("particle left unfinished");
            const auto setup=std::tuple{st.layout,st.setup,st.frame,st.emitter,st.dt,st.drift,st.clamp};
            st=TrailState{};std::tie(st.layout,st.setup,st.frame,st.emitter,st.dt,st.drift,st.clamp)=setup;
            st.record=field(st.emitter+0x14)+reg(20)*layout->stride;
        }else if(st.loads!=site->param || (layout->orbit && st.orbit!=10))fail("record load order");
        if(address()!=(site->param==7?reg(29)+0x1a0:st.record+offsets[site->param]))fail("record field address");
        VuWords *targets[12]={&st.p,&st.v,&st.a,&st.accel,&st.colour,&st.rate,&st.change,&st.offset,&st.radiusAccel,&st.angleAccel,&st.radiusRate,&st.angleRate};
        *targets[site->param]=vuWords(rt,address());
        ++st.loads;return true;}
    case 'W':{
        constexpr uint32_t offsets[4]={0xb0,0x80,0xc0,0x90};
        if(st.loads!=1 || st.orbit!=site->param)fail("orbit order");
        if(site->param<4){if(address()!=st.record+offsets[site->param])fail("orbit field address");
            VuWords *targets[4]={&st.radius,&st.angles,&st.radiusRate,&st.angleRate};*targets[site->param]=vuWords(rt,address());}
        if(site->param==4){st.radius=scaled(st.radius,st.radiusRate);st.angles=scaled(st.angles,st.angleRate);}
        if(site->param>=8){if(address()!=st.record+offsets[site->param-8])fail("orbit store address");put(address(),site->param==8?st.radius:st.angles);}
        ++st.orbit;return true;}
    case 'E':
        if(st.loads!=layout->loads || st.main || st.curve!=site->param)fail("curve order");
        if(site->param==0){if(address()!=field(st.emitter+0x68)+reg(layout->curveIndex)*16)fail("curve row address");st.row=vuWords(rt,address());}
        if(site->param==1){if(reg(4)!=std::bit_cast<uint32_t>(c->f[1]))fail("curve fraction source");st.fraction=reg(4);}
        if(site->param==3){for(unsigned i=0;i<3;++i)st.p[i]=vuAdd(vuAdd(st.p[i],0),vuMul(st.row[i],st.fraction));st.curve=0;return true;}
        ++st.curve;return true;
    case 'k':
        if(st.loads!=layout->loads || st.curve || st.main!=site->param)fail("main step order");
        if(site->param==0){
            for(unsigned i=0;i<3;++i){
                auto P=vuAdd(st.p[i],0),A=vuAdd(st.a[i],0);
                if(layout->orbit){P=vuAdd(P,vuMul(st.offset[i],one));A=vuAdd(A,vuMul(st.offset[i],one));}
                st.P[i]=vuAdd(vuAdd(P,vuMul(st.drift[i],one)),vuMul(st.v[i],st.dt));
                st.A[i]=vuAdd(A,vuMul(st.v[i],st.dt));
                st.V[i]=vuAdd(vuAdd(st.v[i],0),vuMul(st.accel[i],st.dt));}
            st.P[3]=st.p[3];st.A[3]=vuAdd(st.a[3],st.dt);st.V[3]=vuSub(st.v[3],st.dt);
            for(unsigned i=0;i<4;++i){
                st.C[i]=vmin(vmax(vuAdd(vuAdd(st.colour[i],0),vuMul(st.rate[i],st.dt)),0),st.clamp[0]);
                st.R[i]=vuAdd(vuMul(st.rate[i],one),vuMul(st.change[i],st.dt));}
            if(layout->orbit){st.RR=scaled(st.radiusRate,st.radiusAccel);st.AR=scaled(st.angleRate,st.angleAccel);}
        }
        if(word==0x4bc1d6af)include(st.P);
        ++st.main;return true;
    case 'O':
        if(st.main!=layout->main || st.outputs!=site->param)fail("output order");
        if(site->param==0){if(address()!=outputBase())fail("output address");st.output=address();put(st.output,st.P);}
        else{if(address()!=st.output+0x10)fail("output address");put(address(),st.C);}
        ++st.outputs;return true;
    case 'R':{
        constexpr uint32_t offsets[7]={0x50,0x10,0x30,0x20,0x60,0xc0,0x90};
        if(st.outputs!=2 || site->param>=layout->records || (st.stored>>site->param&1))fail("record store order");
        if((st.extra==2 && st.extraStep!=(site->param?2:1)) || (st.extra==5 && st.extraStep!=8))fail("record store inside a mode step");
        if(address()!=st.record+offsets[site->param])fail("record store address");
        const VuWords *sources[7]={&st.A,&st.P,&st.V,&st.C,&st.R,&st.RR,&st.AR};put(address(),*sources[site->param]);
        st.stored|=1u<<site->param;return true;}
    case '2':
        if(st.outputs!=2)fail("mode 2 order");
        if(site->param==0){if(st.stored || st.extra)fail("mode 2 order");st.extra=2;st.extraStep=1;return true;}
        if(st.extra!=2 || st.extraStep!=site->param || (site->param==1 && st.stored!=1) || (site->param>=2 && st.stored!=allStored))fail("mode 2 order");
        if(site->param==3){st.published=st.A;st.published[0]=sumSquares(st.A);for(unsigned i=1;i<3;++i)st.published[i]=vuMul(st.A[i],st.A[i]);}
        if(site->param==4){if(address()!=reg(29)+layout->mode2)fail("mode 2 publication address");put(address(),st.published);st.extra=st.extraStep=0;return true;}
        ++st.extraStep;return true;
    case '5':
        if(st.outputs!=2)fail("mode 5 order");
        if(site->param==0){if((st.stored&0x1f) || st.extra)fail("mode 5 order");st.extra=5;st.extraStep=0;}
        if(st.extra!=5 || st.extraStep!=site->param || (site->param<8 && (st.stored&0x1f)) || (site->param==8 && st.stored!=allStored))fail("mode 5 order");
        if(site->param<4){if(reg(2)!=st.emitter+0x80 || address()!=st.emitter+0x80+0x10*site->param)fail("mode 5 row address");st.rows[site->param]=vuWords(rt,address());}
        if(site->param==7)for(unsigned i=0;i<4;++i)
            st.published[i]=vuAdd(vuAdd(vuAdd(vuMul(st.rows[0][i],st.P[0]),vuMul(st.rows[1][i],st.P[1])),vuMul(st.rows[2][i],st.P[2])),vuMul(st.rows[3][i],one));
        if(site->param==8){if(address()!=reg(29)+layout->mode5)fail("mode 5 publication address");put(address(),st.published);st.extra=st.extraStep=0;return true;}
        ++st.extraStep;return true;
    case 'h':{
        if(st.stored!=allStored || st.extra || st.trail)fail("trail before the particle's records");
        const auto head=layout->orbit?outputBase()-reg(22)*0x30:reg(23)-reg(19)*0x30;
        if(address()!=head || address()!=st.output)fail("trail head address");
        st.head=vuWords(rt,address());st.trail=site->param;st.trailStep=1;return true;}
    case 'n':
        if(st.trail!=site->param || st.trailStep!=1)fail("trail length order");
        if(reg(2)!=field(st.emitter+0x54))fail("trail length source");st.length=reg(2);st.trailStep=2;return true;
    case 'v':
        if(st.trail!=site->param || st.trailStep!=2)fail("trail anchor order");
        st.anchor=st.prev=st.head;st.trailStep=0;st.trailIndex=0;return true;
    case 'p':{
        const unsigned last=site->param==2?16:site->param==1?15:8;
        if(st.trail!=site->param || (st.trailStep!=0 && st.trailStep!=last) || reg(3)!=st.trailIndex+1)fail("trail point order");
        if(address()!=st.output+reg(3)*0x30)fail("trail point address");
        const auto next=vuWords(rt,address());VuWords delta{},direction{};
        for(unsigned i=0;i<4;++i)delta[i]=vuSub(next[i],st.prev[i]);st.prev=next;
        if(site->param!=3){uint32_t flags=0;const auto root=bozo::ps2f::rsqrt(one,sumSquares(delta),flags);
            for(unsigned i=0;i<3;++i)direction[i]=vuMul(delta[i],root);}
        for(unsigned i=0;i<3;++i){
            const auto start=vuAdd(st.anchor[i],0);
            st.anchor[i]=site->param==2?vuAdd(vuAdd(start,vuMul(delta[i],one)),vuMul(direction[i],st.length)):
                         site->param==1?vuAdd(start,vuMul(direction[i],st.length)):vuAdd(start,vuMul(delta[i],st.length));}
        st.next=address();st.trailIndex=reg(3);st.trailStep=1;return true;}
    case 'i':case 'a':case 'b':
        if(!st.trail || st.trailStep!=site->param)fail("trail step order");
        if(site->role=='a'){if(address()!=st.next)fail("trail store address");put(address(),st.anchor);}
        if(site->role=='b' && word==0x4bc4d6af)include(st.anchor);
        ++st.trailStep;return true;
    }
    return false;
}
// VU0 program 0x2d0 (mode 0): in-place 4x4 Gauss-Jordan inverse of rows
// vf1..vf4, no row swaps. Each pivot is first clamped away from zero by the
// sign of a CLIP test against |0|: negative -> min(p, c.x), else max(p, c.y),
// with c = VU0 data qword 0 = (-1e-5, 1e-5, 0, 0) from mode 0's UNPACK; the
// fourth step's negative clamp uses vf29.x (the negated original M[1][0])
// instead of c.x, as the listing does. Elimination order, lanes and the
// 0 - x / x * Q forms follow the listing pair by pair (sequential VU).
std::array<VuWords,4> program2d0(std::array<VuWords,4> v){
    const uint32_t one=0x3f800000u,cx=0xb727c5acu,cy=0x3727c5acu;
    const auto negative=[](uint32_t x){return (x&0x80000000u) && (x&0x7f800000u);};
    const auto key=[](uint32_t x){const int32_t m=int32_t(x&0x7fffffffu);return (x&0x80000000u)?-m:m;};
    const auto vmin=[&](uint32_t a,uint32_t b){return key(a)<=key(b)?a:b;};
    const auto vmax=[&](uint32_t a,uint32_t b){return key(a)>=key(b)?a:b;};
    const auto inv=[&](uint32_t p){uint32_t f=0;const auto saved=_mm_getcsr();const auto q=bozo::ps2f::div(one,p,f);_mm_setcsr(saved);return q;};
    auto &r1=v[0],&r2=v[1],&r3=v[2],&r4=v[3];VuWords t29{},t28{},t27{},t26{},t25{},t24{},p{};
    const auto lanes=[](VuWords &d,unsigned mask,auto f){for(unsigned i=0;i<4;++i)if(mask>>i&1)d[i]=f(i);};
    // step 1: pivot r1.x
    p[0]=r1[0];p[0]=negative(p[0])?vmin(p[0],cx):vmax(p[0],cy);
    t29[0]=vuSub(0,r2[0]);t28[0]=vuSub(0,r3[0]);t27[0]=vuSub(0,r4[0]);
    auto q=inv(p[0]);lanes(r1,15,[&](unsigned i){return vuMul(r1[i],q);});r1[0]=vuAdd(0,q);
    lanes(t26,14,[&](unsigned i){return vuMul(r1[i],t29[0]);});lanes(t25,14,[&](unsigned i){return vuMul(r1[i],t28[0]);});lanes(t24,14,[&](unsigned i){return vuMul(r1[i],t27[0]);});
    r2[0]=vuMul(t29[0],q);lanes(r2,14,[&](unsigned i){return vuAdd(r2[i],t26[i]);});
    r3[0]=vuMul(t28[0],q);r4[0]=vuMul(t27[0],q);lanes(r3,14,[&](unsigned i){return vuAdd(r3[i],t25[i]);});
    const bool n2=negative(r2[1]);lanes(r4,14,[&](unsigned i){return vuAdd(r4[i],t24[i]);});p[1]=r2[1];
    // step 2: pivot r2.y
    p[1]=n2?vmin(p[1],cx):vmax(p[1],cy);
    t29[1]=vuSub(0,r1[1]);t28[1]=vuSub(0,r3[1]);t27[1]=vuSub(0,r4[1]);
    q=inv(p[1]);lanes(r2,15,[&](unsigned i){return vuMul(r2[i],q);});r2[1]=vuAdd(0,q);
    lanes(t25,13,[&](unsigned i){return vuMul(r2[i],t28[1]);});lanes(t26,13,[&](unsigned i){return vuMul(r2[i],t29[1]);});lanes(t24,13,[&](unsigned i){return vuMul(r2[i],t27[1]);});
    r1[1]=vuMul(t29[1],q);lanes(r3,13,[&](unsigned i){return vuAdd(r3[i],t25[i]);});r3[1]=vuMul(t28[1],q);r4[1]=vuMul(t27[1],q);
    lanes(r1,13,[&](unsigned i){return vuAdd(r1[i],t26[i]);});
    const bool n3=negative(r3[2]);p[2]=r3[2];lanes(r4,13,[&](unsigned i){return vuAdd(r4[i],t24[i]);});
    // step 3: pivot r3.z
    p[2]=n3?vmin(p[2],cx):vmax(p[2],cy);
    t29[2]=vuSub(0,r1[2]);t28[2]=vuSub(0,r2[2]);t27[2]=vuSub(0,r4[2]);
    q=inv(p[2]);lanes(r3,15,[&](unsigned i){return vuMul(r3[i],q);});
    lanes(t24,11,[&](unsigned i){return vuMul(r3[i],t27[2]);});r3[2]=vuAdd(0,q);
    r1[2]=vuMul(t29[2],q);r2[2]=vuMul(t28[2],q);lanes(r4,11,[&](unsigned i){return vuAdd(r4[i],t24[i]);});
    lanes(t25,11,[&](unsigned i){return vuMul(r3[i],t28[2]);});lanes(t26,11,[&](unsigned i){return vuMul(r3[i],t29[2]);});r4[2]=vuMul(t27[2],q);
    const auto t28x=vuAdd(0,r4[3]);p[3]=r4[3];
    lanes(r2,11,[&](unsigned i){return vuAdd(r2[i],t25[i]);});lanes(r1,11,[&](unsigned i){return vuAdd(r1[i],t26[i]);});
    // step 4: pivot r4.w (negative clamp with vf29.x)
    p[3]=negative(t28x)?vmin(p[3],t29[0]):vmax(p[3],cy);
    t29[3]=vuSub(0,r1[3]);t28[3]=vuSub(0,r2[3]);t27[3]=vuSub(0,r3[3]);
    q=inv(p[3]);lanes(r4,15,[&](unsigned i){return vuMul(r4[i],q);});r4[3]=vuAdd(0,q);
    r1[3]=vuMul(t29[3],q);r2[3]=vuMul(t28[3],q);
    lanes(t26,7,[&](unsigned i){return vuMul(r4[i],t29[3]);});lanes(t25,7,[&](unsigned i){return vuMul(r4[i],t28[3]);});lanes(t24,7,[&](unsigned i){return vuMul(r4[i],t27[3]);});
    r3[3]=vuMul(t27[3],q);lanes(r1,7,[&](unsigned i){return vuAdd(r1[i],t26[i]);});lanes(r2,7,[&](unsigned i){return vuAdd(r2[i],t25[i]);});lanes(r3,7,[&](unsigned i){return vuAdd(r3[i],t24[i]);});
    return v;
}
// mod001 212950 (called from the cull loop 2202d0 at 2204fc): separating-axis
// test of an oriented box against the camera frustum. a1 = box record (+0
// centre, +10..+30 axes, +40 half extents), a0 = frustum record (+0 origin,
// +10..+30 axes, +40 extents; +48..+58 limits read by the EE code). The VU0
// macro work is transcribed below in listing order, one native ps2f statement
// per macro op (ACC products rounded, then summed, as the VU does). VF lanes
// carry definition bits: only lanes this function writes or loads are stored,
// so a store never publishes a stale register lane. The EE code between the
// blocks (comparisons, early returns) stays compiled; every block runs in
// address order (no branch skips a VU word except to the exit).
struct BoxFrustumStore{uint32_t pc;unsigned reg;uint32_t offset;};
struct BoxFrustumLoad{uint32_t pc;unsigned reg,frustum;uint32_t offset,immediate;};
struct BoxFrustumVu{
    VuWords vf[32]{};unsigned def[32]{};VuWords acc{};unsigned accDef=0;
    std::array<VuWords,38> stored{};std::array<unsigned,38> storedDef{};
    BoxFrustumVu(){vf[0]={0,0,0,0x3f800000u};def[0]=15;}
    uint32_t lane(unsigned r,unsigned i,int bc)const{return vf[r][bc<0?i:unsigned(bc)];}
    bool known(unsigned r,unsigned i,int bc)const{return def[r]>>(bc<0?i:unsigned(bc))&1;}
    void load(unsigned r,const VuWords &w){vf[r]=w;def[r]=15;}
    void store(unsigned k,unsigned r){stored[k]=vf[r];storedDef[k]=def[r];}
    template<class F>void write(unsigned fd,unsigned fs,unsigned ft,int bc,unsigned m,bool useAcc,F f){
        VuWords out=vf[fd];unsigned d=def[fd];
        for(unsigned i=0;i<4;++i)if(m>>i&1){out[i]=f(vf[fs][i],lane(ft,i,bc),acc[i]);
            const bool ok=(def[fs]>>i&1) && known(ft,i,bc) && (!useAcc || (accDef>>i&1));d=ok?d|1u<<i:d&~(1u<<i);}
        vf[fd]=out;def[fd]=d;}
    template<class F>void accumulate(unsigned fs,unsigned ft,int bc,unsigned m,bool useAcc,F f){
        VuWords out=acc;unsigned d=accDef;
        for(unsigned i=0;i<4;++i)if(m>>i&1){out[i]=f(vf[fs][i],lane(ft,i,bc),acc[i]);
            const bool ok=(def[fs]>>i&1) && known(ft,i,bc) && (!useAcc || (accDef>>i&1));d=ok?d|1u<<i:d&~(1u<<i);}
        acc=out;accDef=d;}
    static uint32_t larger(uint32_t a,uint32_t b){const auto k=[](uint32_t x){const int32_t m=int32_t(x&0x7fffffffu);return (x&0x80000000u)?-m:m;};return k(a)>=k(b)?a:b;}
    void add(unsigned d,unsigned s,unsigned t,int bc,unsigned m){write(d,s,t,bc,m,false,[](uint32_t a,uint32_t b,uint32_t){return vuAdd(a,b);});}
    void sub(unsigned d,unsigned s,unsigned t,int bc,unsigned m){write(d,s,t,bc,m,false,[](uint32_t a,uint32_t b,uint32_t){return vuSub(a,b);});}
    void mul(unsigned d,unsigned s,unsigned t,int bc,unsigned m){write(d,s,t,bc,m,false,[](uint32_t a,uint32_t b,uint32_t){return vuMul(a,b);});}
    void max(unsigned d,unsigned s,unsigned t,int bc,unsigned m){write(d,s,t,bc,m,false,[](uint32_t a,uint32_t b,uint32_t){return larger(a,b);});}
    void madd(unsigned d,unsigned s,unsigned t,int bc,unsigned m){write(d,s,t,bc,m,true,[](uint32_t a,uint32_t b,uint32_t c){return vuAdd(c,vuMul(a,b));});}
    void msub(unsigned d,unsigned s,unsigned t,int bc,unsigned m){write(d,s,t,bc,m,true,[](uint32_t a,uint32_t b,uint32_t c){return vuSub(c,vuMul(a,b));});}
    void adda(unsigned s,unsigned t,int bc,unsigned m){accumulate(s,t,bc,m,false,[](uint32_t a,uint32_t b,uint32_t){return vuAdd(a,b);});}
    void mula(unsigned s,unsigned t,int bc,unsigned m){accumulate(s,t,bc,m,false,[](uint32_t a,uint32_t b,uint32_t){return vuMul(a,b);});}
    void madda(unsigned s,unsigned t,int bc,unsigned m){accumulate(s,t,bc,m,true,[](uint32_t a,uint32_t b,uint32_t c){return vuAdd(c,vuMul(a,b));});}
    void msuba(unsigned s,unsigned t,int bc,unsigned m){accumulate(s,t,bc,m,true,[](uint32_t a,uint32_t b,uint32_t c){return vuSub(c,vuMul(a,b));});}
    void abs(unsigned t,unsigned s,unsigned m){for(unsigned i=0;i<4;++i)if(m>>i&1){vf[t][i]=vf[s][i]&0x7fffffffu;def[t]=(def[s]>>i&1)?def[t]|1u<<i:def[t]&~(1u<<i);}}
};
constexpr BoxFrustumStore kBoxFrustumStores[]={{0x2129c4,5,0x0},{0x2129e0,5,0x10},{0x212a78,2,0x0},{0x212a7c,2,0x10},{0x212ad0,2,0x0},{0x212ad4,2,0x10},{0x212b70,2,0x0},{0x212b74,2,0x10},{0x212bc8,2,0x10},{0x212bcc,2,0x0},{0x212c2c,2,0x0},{0x212c30,2,0x10},{0x212c34,2,0x20},{0x212c38,2,0x30},{0x212da4,2,0x0},{0x212da8,2,0x10},{0x212dac,2,0x20},{0x212e04,2,0x30},{0x212f78,2,0x0},{0x212f7c,2,0x10},{0x212f80,2,0x20},{0x212fd8,2,0x30},{0x213164,2,0x0},{0x213168,2,0x10},{0x21316c,2,0x20},{0x2131b8,2,0x30},{0x213334,2,0x0},{0x213338,2,0x10},{0x21333c,2,0x20},{0x213388,2,0x30},{0x213508,2,0x0},{0x21350c,2,0x10},{0x213510,2,0x20},{0x21355c,2,0x30},{0x2136e0,2,0x0},{0x2136e4,2,0x10},{0x2136e8,2,0x20},{0x213734,2,0x30}};
constexpr BoxFrustumLoad kBoxFrustumLoads[]={{0x212954,5,0,0x0,0x0},{0x212958,4,1,0x0,0x0},{0x212964,3,0,0x10,0x0},{0x212968,3,0,0x20,0x10},{0x21296c,3,0,0x30,0x20},{0x212970,2,0,0x40,0x0},{0x21297c,2,1,0x30,0x0},{0x2129cc,3,1,0x10,0x0},{0x2129d4,2,1,0x40,0x0},{0x212b0c,2,1,0x20,0x0}};
BoxFrustumVu boxFrustumProgram(const std::array<VuWords,5> &box,const std::array<VuWords,5> &frustum){
    BoxFrustumVu v;
    v.load(6,box[0]); v.load(7,frustum[0]); v.load(1,box[1]); v.load(2,box[2]); v.load(3,box[3]); v.load(4,box[4]); v.sub(5,6,7,-1,7);
    v.load(6,frustum[3]); v.max(31,0,0,3,15); v.sub(30,0,0,3,15); v.mul(29,5,6,-1,7); v.mul(27,3,6,-1,7); v.mul(28,2,6,-1,7); v.adda(29,29,1,4);
    v.madd(7,31,29,0,4); v.mul(29,1,6,-1,7); v.adda(27,27,1,4); v.madd(8,31,27,0,4); v.adda(28,28,0,2); v.madd(8,31,28,2,2); v.adda(29,29,1,1);
    v.madd(8,31,29,2,1); v.store(0,7); v.abs(29,8,7); v.load(6,frustum[1]); v.mul(28,4,29,-1,7); v.load(10,frustum[4]); v.adda(28,28,1,1);
    v.madd(27,31,28,2,1); v.store(1,27);
    v.mul(29,1,6,-1,7); v.mul(28,2,6,-1,7); v.mul(27,3,6,-1,7); v.adda(29,29,1,1); v.madd(9,31,29,2,1); v.adda(28,28,0,2); v.madd(9,31,28,2,2);
    v.adda(27,27,1,4); v.madd(9,31,27,0,4); v.mul(29,5,6,-1,7); v.mul(11,8,10,0,7); v.mul(12,9,10,2,7); v.sub(13,12,11,-1,7); v.adda(29,29,1,1);
    v.madd(7,31,29,2,1); v.abs(28,13,7); v.mul(27,28,4,-1,7); v.mul(14,7,10,2,1); v.mul(15,7,10,0,4); v.adda(27,27,1,1); v.madd(27,31,27,2,1);
    v.sub(28,14,15,2,1);
    v.store(2,27); v.store(3,28);
    v.add(16,12,11,-1,7); v.abs(29,16,7); v.mul(28,29,4,-1,7); v.adda(28,28,1,1); v.madd(27,31,28,2,1); v.mula(30,14,-1,1); v.madd(28,30,15,2,1);
    v.store(4,27); v.store(5,28);
    v.load(6,frustum[2]); v.mul(29,1,6,-1,7); v.mul(28,2,6,-1,7); v.mul(27,3,6,-1,7); v.adda(29,29,1,1); v.madd(17,31,29,2,1); v.adda(28,28,0,2);
    v.madd(17,31,28,2,2); v.adda(27,27,1,4); v.madd(17,31,27,0,4); v.mul(18,8,10,1,7); v.mul(19,17,10,2,7); v.sub(20,19,18,-1,7); v.mul(29,5,6,-1,7);
    v.abs(28,20,7); v.adda(29,29,0,2); v.madd(7,31,29,2,2); v.mul(27,28,4,-1,7); v.mul(14,7,10,2,2); v.mul(21,7,10,1,4); v.adda(27,27,1,1);
    v.madd(27,31,27,2,1); v.sub(28,14,21,2,2);
    v.store(6,27); v.store(7,28);
    v.add(22,19,18,-1,7); v.abs(29,22,7); v.mul(28,29,4,-1,7); v.mula(30,14,1,1); v.madd(29,30,21,2,1); v.adda(28,28,1,1); v.madd(27,31,28,2,1);
    v.store(8,29); v.store(9,27);
    v.abs(29,9,7); v.abs(28,17,7); v.mul(23,8,10,2,7); v.mula(29,10,0,7); v.madd(27,28,10,1,7); v.mula(9,7,0,7); v.madda(17,7,1,7); v.madd(28,8,7,2,7);
    v.sub(29,23,27,-1,7); v.add(27,23,27,-1,7); v.store(10,28); v.store(11,29); v.store(12,27); v.store(13,4);
    v.abs(29,8,7); v.mul(29,29,10,1,7); v.sub(28,19,29,-1,7); v.add(27,19,29,-1,7); v.mula(17,7,2,7); v.msub(29,8,7,1,7); v.store(14,29); v.store(15,28);
    v.store(16,27); v.mula(17,8,0,7); v.msub(29,8,17,0,7); v.mula(17,8,1,7); v.msub(28,8,17,1,7); v.mula(17,8,2,7); v.msub(27,8,17,2,7); v.abs(29,29,7);
    v.abs(28,28,7); v.abs(27,27,7); v.mul(29,29,4,-1,7); v.mul(28,28,4,-1,7); v.mul(27,27,4,-1,7); v.adda(29,29,1,1); v.madd(29,31,29,2,1);
    v.adda(28,28,0,2); v.madd(29,31,28,2,2); v.adda(27,27,1,4); v.madd(29,31,27,0,4);
    v.store(17,29);
    v.abs(29,8,7); v.mul(28,29,10,0,7); v.mula(30,12,-1,7); v.msub(29,28,0,3,7); v.sub(28,28,12,-1,7); v.mula(8,7,0,7); v.msub(27,9,7,2,7);
    v.store(18,29); v.store(19,28); v.store(20,27); v.mula(9,8,0,7); v.msub(29,8,9,0,7); v.mula(9,8,1,7); v.msub(28,8,9,1,7); v.mula(9,8,2,7);
    v.msub(27,8,9,2,7); v.abs(29,29,7); v.abs(28,28,7); v.abs(27,27,7); v.mul(29,29,4,-1,7); v.mul(28,28,4,-1,7); v.mul(27,27,4,-1,7); v.adda(29,29,1,1);
    v.madd(29,31,29,2,1); v.adda(28,28,0,2); v.madd(29,31,28,2,2); v.adda(27,27,1,4); v.madd(29,31,27,0,4);
    v.store(21,29);
    v.mul(24,17,10,0,7); v.mul(25,9,10,1,7); v.add(26,24,25,-1,7); v.sub(23,24,25,-1,7); v.abs(29,20,7); v.abs(28,13,7); v.mula(29,10,0,7);
    v.madd(27,28,10,1,7); v.mul(28,23,10,2,7); v.sub(29,28,27,-1,7); v.add(28,28,27,-1,7); v.mula(13,7,1,7); v.msuba(20,7,0,7); v.madd(27,23,7,2,7);
    v.store(22,29); v.store(23,28); v.store(24,27); v.mula(13,17,0,7); v.msuba(20,9,0,7); v.madd(29,23,8,0,7); v.mula(13,17,1,7); v.msuba(20,9,1,7);
    v.madd(28,23,8,1,7); v.mula(13,17,2,7); v.msuba(20,9,2,7); v.madd(27,23,8,2,7); v.abs(29,29,7); v.abs(28,28,7); v.abs(27,27,7); v.mula(29,4,0,7);
    v.madda(28,4,1,7); v.madd(29,27,4,2,7);
    v.store(25,29);
    v.abs(29,22,7); v.abs(28,13,7); v.mula(29,10,0,7); v.madd(29,28,10,1,7); v.mul(27,26,10,2,7); v.sub(28,27,29,-1,7); v.add(27,27,29,-1,7);
    v.mula(13,7,1,7); v.msuba(22,7,0,7); v.madd(29,26,7,2,7); v.store(26,28); v.store(27,27); v.store(28,29); v.mula(13,17,0,7); v.msuba(22,9,0,7);
    v.madd(29,26,8,0,7); v.mula(13,17,1,7); v.msuba(22,9,1,7); v.madd(28,26,8,1,7); v.mula(13,17,2,7); v.msuba(22,9,2,7); v.madd(27,26,8,2,7);
    v.abs(29,29,7); v.abs(28,28,7); v.abs(27,27,7); v.mula(29,4,0,7); v.madda(28,4,1,7); v.madd(29,27,4,2,7);
    v.store(29,29);
    v.abs(29,20,7); v.abs(28,16,7); v.mula(29,10,0,7); v.madd(29,28,10,1,7); v.mula(0,0,0,7); v.msub(27,26,10,2,7); v.sub(28,27,29,-1,7);
    v.add(27,27,29,-1,7); v.mula(16,7,1,7); v.msuba(20,7,0,7); v.msub(29,26,7,2,7); v.store(30,28); v.store(31,27); v.store(32,29); v.mula(16,17,0,7);
    v.msuba(20,9,0,7); v.msub(29,26,8,0,7); v.mula(16,17,1,7); v.msuba(20,9,1,7); v.msub(28,26,8,1,7); v.mula(16,17,2,7); v.msuba(20,9,2,7);
    v.msub(27,26,8,2,7); v.abs(29,29,7); v.abs(28,28,7); v.abs(27,27,7); v.mula(29,4,0,7); v.madda(28,4,1,7); v.madd(29,27,4,2,7);
    v.store(33,29);
    v.abs(29,22,7); v.abs(28,16,7); v.mula(29,10,0,7); v.madd(29,28,10,1,7); v.mula(0,0,0,7); v.msub(27,23,10,2,7); v.sub(28,27,29,-1,7);
    v.add(27,27,29,-1,7); v.mula(16,7,1,7); v.msuba(22,7,0,7); v.msub(29,23,7,2,7); v.store(34,28); v.store(35,27); v.store(36,29); v.mula(16,17,0,7);
    v.msuba(22,9,0,7); v.msub(29,23,8,0,7); v.mula(16,17,1,7); v.msuba(22,9,1,7); v.msub(28,23,8,1,7); v.mula(16,17,2,7); v.msuba(22,9,2,7);
    v.msub(27,23,8,2,7); v.abs(29,29,7); v.abs(28,28,7); v.abs(27,27,7); v.mula(29,4,0,7); v.madda(28,4,1,7); v.madd(29,27,4,2,7);
    v.store(37,29);
    return v;
}
struct BoxFrustumCall{BoxFrustumVu vu;uint32_t sp=0,box=0,frustum=0;bool active=false;};
BoxFrustumCall &boxFrustumCall(const Runtime *rt){static std::map<const Runtime *,BoxFrustumCall> calls;return calls[rt];}
bool nativeBoxFrustum(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc<0x212954 || pc>0x213734 || (pc&3))return false;
    if(!c)throw std::runtime_error("native box/frustum test has no CPU record");
    auto &call=boxFrustumCall(rt);
    if(pc==0x212954){
        call.active=false;call.frustum=c->r[4].uw[0];call.box=c->r[5].uw[0];call.sp=c->r[29].uw[0];
        if((call.frustum|call.box|call.sp)&15)throw std::runtime_error("native box/frustum records misaligned");
        const auto overlaps=[](uint32_t a,uint32_t an,uint32_t b,uint32_t bn){return (a&0x1fffffff)<(b&0x1fffffff)+bn && (b&0x1fffffff)<(a&0x1fffffff)+an;};
        if(overlaps(call.box,0x50,call.sp,0x40) || overlaps(call.frustum,0x50,call.sp,0x40))throw std::runtime_error("native box/frustum records alias its frame");
        std::array<VuWords,5> box,frustum;for(unsigned k=0;k<5;++k){box[k]=vuWords(rt,call.box+16*k);frustum[k]=vuWords(rt,call.frustum+16*k);}
        call.vu=boxFrustumProgram(box,frustum);call.active=true;
    }
    if(!call.active)throw std::runtime_error("native box/frustum block reached without its entry");
    for(const auto &l:kBoxFrustumLoads)if(pc==l.pc){
        if(c->r[l.reg].uw[0]!=(l.frustum?call.frustum:call.box)+l.offset-l.immediate)throw std::runtime_error("native box/frustum load source mismatch");
        return true;}
    for(unsigned k=0;k<std::size(kBoxFrustumStores);++k)if(pc==kBoxFrustumStores[k].pc){
        const auto &s=kBoxFrustumStores[k];if(c->r[s.reg].uw[0]!=call.sp)throw std::runtime_error("native box/frustum store frame mismatch");
        auto *out=record(rt,call.sp+s.offset,16);
        for(unsigned i=0;i<4;++i)if(call.vu.storedDef[k]>>i&1)std::memcpy(out+4*i,&call.vu.stored[k][i],4);
        return true;}
    return true;
}
// mod003 3cfde0's second path 3d077c..3d0908: three quaternion matrices
// (program 0x720, entry with vf16 = vf0 so every lane is defined) to SP+240,
// SP+200, SP+200, then program 0x2d0 on SP+240. The base registers of the
// loads are reused for the stores, so the inputs are captured at the loads.
struct FacingPath{std::array<VuWords,4> rows{};VuWords quaternion{};unsigned loaded=0;};
FacingPath &facingPath(const Runtime *rt){static std::map<const Runtime *,FacingPath> paths;return paths[rt];}
bool nativeFacingPath(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Site{uint32_t load,cfc,store;unsigned loadReg,storeReg;};
    constexpr Site quats[3]={{0x3d077c,0x3d07b4,0x3d07bc,2,2},{0x3d07e4,0x3d081c,0x3d0824,3,2},{0x3d0888,0x3d08c0,0x3d08c8,2,2}};
    const bool calls=pc==0x3d0784 || pc==0x3d07ec || pc==0x3d0890;
    const Site *site=nullptr;for(const auto &s:quats)if(pc==s.load || pc==s.cfc || (pc>=s.store && pc<=s.store+8))site=&s;
    if(!calls && !site)return false;
    if(!c)throw std::runtime_error("native facing path has no CPU record");
    if(calls)return true;
    auto &state=facingPath(rt);
    if(pc==0x3d07b4 || pc==0x3d081c || pc==0x3d08c0){c->r[9].ud[0]=0;return true;}
    if(site && pc==site->load){state.quaternion=vuWords(rt,c->r[site->loadReg].uw[0]);state.loaded|=1;return true;}
    if(site){
        if(!(state.loaded&1))throw std::runtime_error("native facing rotation stored without its quaternion");
        float values[4];std::memcpy(values,state.quaternion.data(),16);
        for(float f:values){const auto m=std::bit_cast<uint32_t>(f)&0x7fffffff;if(m>=0x7f800000 || m>std::bit_cast<uint32_t>(1.e8f))throw std::runtime_error("unqualified facing path quaternion");}
        __m128 input;std::memcpy(&input,values,16);const auto result=math::quaternionMatrix(input);const unsigned row=(pc-site->store)/4;
        std::memcpy(record(rt,c->r[site->storeReg].uw[0]+16*row,16),&result[row],16);if(row==2)state.loaded&=~1u;return true;}
    return false;
}
// mod003 3d15c0: three program-0x720 blocks (LQC2 vf5, VCALLMS 0x720,
// CFC2 t1/vi0, SQC2 vf8..vf10). Entry 0x720 sets VF16 = VF0, so every row
// lane is defined (W 0). Sources: SP+1e0 and SP+1d0 (quaternions the EE code
// built from a sine/cosine pair), then *(s2+84)+b0; the rows land at SP+230,
// SP+1f0 and SP+1f0. The load base is rewritten before the stores, so the
// quaternion is captured at the load.
struct RotationBlocks{VuWords quaternion{};bool loaded=false;};
RotationBlocks &rotationBlocks(const Runtime *rt){static std::map<const Runtime *,RotationBlocks> blocks;return blocks[rt];}
bool nativeRotationBlocks(R5900Context *c,Runtime *rt,uint32_t pc){
    struct Block{uint32_t load,call,cfc,store;unsigned loadReg;uint32_t destination;};
    constexpr Block blocks[3]={{0x3d166c,0x3d1674,0x3d16a4,0x3d16ac,2,0x230},{0x3d16d4,0x3d16dc,0x3d170c,0x3d1714,3,0x1f0},{0x3d1778,0x3d1780,0x3d17b0,0x3d17b8,2,0x1f0}};
    const Block *block=nullptr;for(unsigned i=0;i<3;++i){const auto &b=blocks[i];if(pc==b.load || pc==b.call || pc==b.cfc || (pc>=b.store && pc<=b.store+8 && !(pc&3)))block=&b;}
    if(!block)return false;
    if(!c)throw std::runtime_error("native rotation block has no CPU record");
    const auto stack=c->r[29].uw[0];auto &state=rotationBlocks(rt);
    if(pc==block->call)return true;
    if(pc==block->cfc){c->r[9].ud[0]=0;return true;}
    if(pc==block->load){
        const auto source=c->r[block->loadReg].uw[0];
        const uint32_t expected=block->load==0x3d166c?stack+0x1e0:block->load==0x3d16d4?stack+0x1d0:skinWord(rt,c->r[18].uw[0]+0x84)+0xb0;
        if(source!=expected)throw std::runtime_error("native rotation block source mismatch");
        state.quaternion=vuWords(rt,source);state.loaded=true;return true;}
    if(!state.loaded)throw std::runtime_error("native rotation block stored without its quaternion");
    if(c->r[2].uw[0]!=stack+block->destination)throw std::runtime_error("native rotation block destination mismatch");
    float values[4];std::memcpy(values,state.quaternion.data(),16);
    for(float f:values){const auto m=std::bit_cast<uint32_t>(f)&0x7fffffff;if(m>=0x7f800000 || m>std::bit_cast<uint32_t>(1.e8f))throw std::runtime_error("unqualified rotation block quaternion");}
    __m128 input;std::memcpy(&input,values,16);const auto result=math::quaternionMatrix(input);const unsigned row=(pc-block->store)/4;
    std::memcpy(record(rt,stack+block->destination+16*row,16),&result[row],16);if(row==2)state.loaded=false;return true;
}
// The 0x2d0 inverse is inlined about 24 times (mod001 21732c, mod003 3802dc,
// 3d08f0, ...): LQC2 vf1..vf4 at 0/16/32/48 of one base, two NOPs, VCALLMS
// 0x2d0, CFC2.I t1,vi0, at most one plain EE word, SQC2 vf1..vf4 at 0/16/32/48
// of a store base. Recognised by these words around the PC; the rows are
// captured at the loads (the store base may be the same register, rewritten).
struct InverseCapture{std::array<VuWords,4> rows{};uint32_t call=0;unsigned loaded=0;};
InverseCapture &inverseCapture(const Runtime *rt){static std::map<const Runtime *,InverseCapture> captures;return captures[rt];}
// VU0 program 0x720 (quaternion -> rotation rows), inlined about 39 times
// (mod001 215740, mod003 37e180, 3c3d74, 4369c8, ...): LQC2 vf5 (the
// quaternion), VCALLMS 0x720 one or two words later, then CFC2.I t1,vi0 as
// the next COP2 word, then SQC2 vf8, vf9, vf10 at 0/16/32 of one base. Plain
// EE words may stand between them; no branch, jump or other COP2 word does.
// Entry 0x720 sets VF16 = VF0, so every row lane is defined (W 0). The
// quaternion is captured at the load (its base may be rewritten before the
// stores). Recognised by these words around the PC.
struct QuaternionCapture{VuWords quaternion{};uint32_t load=0;bool loaded=false;};
QuaternionCapture &quaternionCapture(const Runtime *rt){static std::map<const Runtime *,QuaternionCapture> captures;return captures[rt];}
bool nativeQuaternionIdiom(R5900Context *c,Runtime *rt,uint32_t pc){
    if(!rt || (pc&3))return false;
    const auto word=[&](uint32_t at,uint32_t &out){const auto *p=rt->memory.hostPointer(at,4);if(!p)return false;std::memcpy(&out,p,4);return true;};
    const auto cop2=[](uint32_t w){const auto op=w>>26;return op==0x12 || op==0x36 || op==0x3e;};
    const auto control=[](uint32_t w){const auto op=w>>26;return (op>=1 && op<=7) || (op>=0x14 && op<=0x17) || (op==0 && ((w&63)==8 || (w&63)==9)) || (op==0x11 && ((w>>21)&31)==8);};
    uint32_t self;if(!word(pc,self) || !cop2(self))return false;
    for(uint32_t load=pc;load+0x80>=pc && load>=4;load-=4){
        uint32_t w;if(!word(load,w) || w>>26!=0x36 || ((w>>16)&31)!=5)continue;
        uint32_t call=0;
        for(uint32_t k=4;k<=8;k+=4){uint32_t x;if(!word(load+k,x))break;if(x==0x4a003938u){call=load+k;break;}if(cop2(x) || control(x))break;}
        if(!call)continue;
        uint32_t cfc=0,at=call+4;
        for(unsigned n=0;n<24;++n,at+=4){uint32_t x;if(!word(at,x))break;if(x==0x48490001u){cfc=at;break;}if(cop2(x) || control(x))break;}
        if(!cfc)continue;
        uint32_t store=0;at=cfc+4;
        for(unsigned n=0;n<4;++n,at+=4){uint32_t x;if(!word(at,x))break;if(x>>26==0x3e && ((x>>16)&31)==8 && !(x&0xffff)){store=at;break;}if(cop2(x) || control(x))break;}
        if(!store)continue;
        uint32_t s[3];bool rows=true;
        for(unsigned r=0;r<3 && rows;++r)rows=word(store+4*r,s[r]) && s[r]>>26==0x3e && ((s[r]>>16)&31)==8+r && (s[r]&0xffff)==16*r && ((s[r]>>21)&31)==((s[0]>>21)&31);
        if(!rows)continue;
        const bool atStore=pc>=store && pc<store+12;
        if(pc!=load && pc!=call && pc!=cfc && !atStore)continue;
        if(!c)throw std::runtime_error("native quaternion idiom has no CPU record");
        auto &capture=quaternionCapture(rt);
        if(pc==load){capture.quaternion=vuWords(rt,c->r[(w>>21)&31].uw[0]+uint32_t(int32_t(int16_t(w&0xffff))));capture.load=load;capture.loaded=true;return true;}
        if(pc==call)return true;
        if(pc==cfc){c->r[9].ud[0]=0;return true;}
        if(!capture.loaded || capture.load!=load)throw std::runtime_error(std::format("native 0x720 quaternion at {:x} stored without its load",load));
        float values[4];std::memcpy(values,capture.quaternion.data(),16);
        for(float f:values){const auto m=std::bit_cast<uint32_t>(f)&0x7fffffff;if(m>=0x7f800000 || m>std::bit_cast<uint32_t>(1.e8f))throw std::runtime_error("unqualified 0x720 quaternion");}
        __m128 input;std::memcpy(&input,values,16);const auto result=math::quaternionMatrix(input);const unsigned row=(pc-store)/4;
        std::memcpy(record(rt,c->r[(s[row]>>21)&31].uw[0]+16*row,16),&result[row],16);if(row==2)capture.loaded=false;return true;
    }
    return false;
}
bool nativeInverseIdiom(R5900Context *c,Runtime *rt,uint32_t pc){
    if(!rt || (pc&3))return false;
    const auto word=[&](uint32_t at,uint32_t &out){const auto *p=rt->memory.hostPointer(at,4);if(!p)return false;std::memcpy(&out,p,4);return true;};
    const auto isLoad=[](uint32_t w,unsigned row){return w>>26==0x36 && ((w>>16)&31)==1+row && (w&0xffff)==16*row;};
    const auto isStore=[](uint32_t w,unsigned row){return w>>26==0x3e && ((w>>16)&31)==1+row && (w&0xffff)==16*row;};
    // The block spans at most 17 words; find its VCALLMS within reach of the PC.
    for(int k=-12;k<=12;++k){
        const uint32_t call=pc+4*k;uint32_t w;if(!word(call,w) || w!=0x4a0016b8)continue;
        uint32_t loads[4],a,b,cfc;bool ok=word(call-8,a) && word(call-4,b) && a==0 && b==0 && word(call+4,cfc) && cfc==0x48490001;
        for(unsigned r=0;r<4 && ok;++r){ok=word(call-24+4*r,loads[r]) && isLoad(loads[r],r) && ((loads[r]>>21)&31)==((loads[0]>>21)&31);}
        if(!ok)continue;
        uint32_t first;if(!word(call+8,first))continue;uint32_t storeAt=call+8;
        if(!isStore(first,0)){const auto op=first>>26;
            if(op==0x12 || op==0x36 || op==0x3e || (op>=1 && op<=7) || (op>=0x14 && op<=0x17) || (op==0 && ((first&63)==8 || (first&63)==9)))continue;
            storeAt=call+12;}
        uint32_t stores[4];for(unsigned r=0;r<4 && ok;++r)ok=word(storeAt+4*r,stores[r]) && isStore(stores[r],r) && ((stores[r]>>21)&31)==((stores[0]>>21)&31);
        if(!ok)continue;
        const bool inLoads=pc>=call-24 && pc<call-8,inStores=pc>=storeAt && pc<storeAt+16;
        if(!inLoads && !inStores && pc!=call && pc!=call+4)continue;
        if(!c)throw std::runtime_error("native inverse idiom has no CPU record");
        auto &capture=inverseCapture(rt);
        if(inLoads){const unsigned r=(pc-(call-24))/4;if(r==0 || capture.call!=call){capture.loaded=0;capture.call=call;}
            capture.rows[r]=vuWords(rt,c->r[(loads[r]>>21)&31].uw[0]+16*r);capture.loaded|=1u<<r;return true;}
        if(pc==call)return true;
        if(pc==call+4){c->r[9].ud[0]=0;return true;}
        if(capture.call!=call || capture.loaded!=15)throw std::runtime_error(std::format("native 0x2d0 inverse at {:x} stored without its rows",call));
        const unsigned r=(pc-storeAt)/4;const auto result=program2d0(capture.rows);
        std::memcpy(record(rt,c->r[(stores[r]>>21)&31].uw[0]+16*r,16),result[r].data(),16);if(r==3)capture.loaded=0;return true;
    }
    return false;
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
std::optional<bool> nativeModelLightVariant(const Runtime *rt,uint32_t model){
    const auto &map=modelLightVariants();const auto it=map.find({rt,model});return it==map.end()?std::nullopt:std::optional(it->second);}
BoxFrustumStores boxFrustumStores(const std::array<std::array<uint32_t,4>,5> &box,const std::array<std::array<uint32_t,4>,5> &frustum){
    const auto vu=boxFrustumProgram(box,frustum);BoxFrustumStores out;out.words=vu.stored;out.defined=vu.storedDef;return out;}
bool nativeMathInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc){
    if(nativeSkinUpload(c,rt,pc) || nativeSoftSkin(c,rt,pc))return true;
    if(nativeFacingPath(c,rt,pc) || nativeRotationBlocks(c,rt,pc))return true;
    if(nativeProjectionBlocks(c,rt,pc) || nativeBoneAim(c,rt,pc) || nativeChainBlocks(c,rt,pc) || nativeCapsulePair(c,rt,pc) || nativePlaneContact(c,rt,pc) || nativeRangeTest(c,rt,pc) || nativeMatrixBuild(c,rt,pc) || nativeCullTransform(c,rt,pc) || nativeBoxFrustum(c,rt,pc) || nativeListBoxQuery(c,rt,pc) || nativeHitFrame(c,rt,pc) || nativePlaneDistance(c,rt,pc) || nativeSphereSweep(c,rt,pc) || nativeSegmentApproach(c,rt,pc) || nativeWorldPosition(c,rt,pc) || nativeCameraTarget(c,rt,pc) || nativeRootLength(c,rt,pc) || nativeVisibilityLoop(c,rt,pc) || nativeMarkerProjection(c,rt,pc) || nativeQuaternionLeaf(c,rt,pc) || nativeBonePalette(c,rt,pc))return true;
    if(nativeLeafSphere(c,rt,pc) || nativeLeafBox(c,rt,pc) || nativeSegmentBox(c,rt,pc) || nativeHitPointTransform(c,rt,pc))return true;
    if(nativeLengthSite(c,rt,pc) || nativeRecordDot(c,rt,pc) || nativePackedQuaternion(c,rt,pc))return true;
    if(nativeObjectOverlap(c,rt,pc))return true;
    if(nativeObjectPairSetup(c,rt,pc))return true;
    if(nativeObjectPairSpill(c,rt,pc))return true;
    if(nativeBattleBasisLengths(c,rt,pc))return true;
    if(nativeBattleShapeOrigin(c,rt,pc))return true;
    if(nativeBattleVertexPack(c,rt,pc))return true;
    if(nativeBattleBoxSphere(c,rt,pc))return true;
    if(nativeBattlePointDistance(c,rt,pc))return true;
    if(nativeBattleShapes(c,rt,pc))return true;
    if(nativeBattleTrackBlocks(c,rt,pc))return true;
    if(nativeBattleApproach(c,rt,pc))return true;
    if(nativeBattleSegmentEnds(c,rt,pc))return true;
    if(nativeBattleNormalise(c,rt,pc))return true;
    if(nativeBattleSideAxes(c,rt,pc))return true;
    if(nativeBattleGroundPlane(c,rt,pc))return true;
    if(nativeBattleColourPair(c,rt,pc))return true;
    if(nativeBattleShapeTree(c,rt,pc))return true;
    if(nativeBattleObjectOverlap(c,rt,pc))return true;
    if(nativeBattleAdvance(c,rt,pc))return true;
    if(nativeBattleRecordShift(c,rt,pc))return true;
    if(nativeBattleSegmentPoints(c,rt,pc))return true;
    if(nativeBattleHalfVectors(c,rt,pc))return true;
    if(nativeBattleNearestPoint(c,rt,pc))return true;
    if(nativeContactPoint(c,rt,pc))return true;
    if(nativeNearestOfPoints(c,rt,pc))return true;
    if(nativeBattleLabelProjection(c,rt,pc))return true;
    if(nativeChainAim(c,rt,pc))return true;
    if(nativeVectorTurn(c,rt,pc))return true;
    if(nativePaletteFade(c,rt,pc))return true;
    if(nativeBattlePointScale(c,rt,pc))return true;
    if(nativeBattleTaskColours(c,rt,pc))return true;
    if(nativeBattleUnitProjection(c,rt,pc))return true;
    if(nativeBattleCameraProjection(c,rt,pc))return true;
    if(nativeBattleSegmentDirection(c,rt,pc))return true;
    if(nativeSegmentSphere(c,rt,pc))return true;
    if(nativeBattleSegmentBox(c,rt,pc))return true;
    if(nativeBattleShapePoint(c,rt,pc))return true;
    if(nativeBattleBoxCorners(c,rt,pc))return true;
    if(nativeBattleBoxWorld(c,rt,pc))return true;
    if(nativeUvAnimation(c,rt,pc))return true;
    if(nativeSegmentContact(c,rt,pc))return true;
    if(nativeObjectCompose(c,rt,pc))return true;
    if(nativeCameraProjection(c,rt,pc))return true;
    if(nativeFlareProjection(c,rt,pc))return true;
    if(nativePairProjection(c,rt,pc))return true;
    if(nativeExponentialRamp(c,rt,pc))return true;
    if(nativeGridPoint(c,rt,pc))return true;
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
    if(pc==0x396a8c || pc==0x396a94 || pc==0x396ac4 || pc==0x396acc || pc==0x396ad0 || pc==0x396ad4){
        // mod003 396a80 (yaw from an object's quaternion): LQC2 vf5 from a0+b0,
        // program 0x720, rows to SP+20, which the scalar code hands to 410df0
        // (native) and atan2. a0 stays the object until after the stores.
        if(!c || c->pc!=0x396a80)throw std::runtime_error("native object yaw escaped its routine");
        const auto stack=c->r[29].uw[0],object=c->r[4].uw[0];
        if(pc==0x396a8c && c->r[2].uw[0]!=object+0xb0)throw std::runtime_error("object yaw quaternion source mismatch");
        if(pc==0x396ac4)c->r[9].ud[0]=0;
        else if(pc>=0x396acc){
            if(c->r[2].uw[0]!=stack+0x20)throw std::runtime_error("object yaw rotation destination mismatch");
            __m128 input;std::memcpy(&input,record(rt,object+0xb0,16),16);float values[4];std::memcpy(values,&input,16);
            for(float v:values){const auto magnitude=std::bit_cast<uint32_t>(v)&0x7fffffff;if(magnitude>=0x7f800000 || magnitude>std::bit_cast<uint32_t>(1.e8f))throw std::runtime_error("unqualified object yaw quaternion");}
            const auto result=math::quaternionMatrix(input);const auto row=(pc-0x396acc)/4;std::memcpy(record(rt,stack+0x20+16*row,16),&result[row],16);
        }return true;
    }
    // Effect 15e870 rotation blocks: nine identical program-0x720 blocks in
    // mod003 (417f50 twice with s2, 412870 twice and 431000/4335xx with s0,
    // 4197xx with s3). Each builds a 15e870 object from its owner: owner
    // +a0/+b0 copied to obj+70/+80, LQC2 vf5 from obj+80, program 0x720,
    // CFC2.I t1/vi0, rows SQC2 vf8..vf10 to obj+30. Between the load and the
    // stores the EE code writes only obj+60..6c. The recompiled block starts
    // after the allocator call (block-5c) or at the null check it branches to
    // (block-1c); no branch lies inside. Every block's bytes are guarded.
    struct EffectBlock{uint32_t at;unsigned object;};
    constexpr EffectBlock effectBlocks[]={{0x412bcc,16},{0x412ddc,16},{0x41861c,18},{0x418814,18},{0x419884,19},
                                          {0x43128c,16},{0x4314d4,16},{0x433614,16},{0x433904,16}};
    const EffectBlock *effect=nullptr;for(const auto &b:effectBlocks)if(pc>=b.at && pc<=b.at+0x54)effect=&b;
    if(const uint32_t block=effect?effect->at:0;
       block && (pc==block || pc==block+8 || pc==block+0x44 || pc==block+0x4c || pc==block+0x50 || pc==block+0x54)){
        if(!c || (c->pc!=block-0x5c && c->pc!=block-0x1c))throw std::runtime_error("native effect rotation escaped its routine");
        const auto object=c->r[effect->object].uw[0];
        if(pc==block){if(c->r[3].uw[0]!=object+0x80)throw std::runtime_error("effect rotation quaternion source mismatch");}
        else if(pc==block+0x44)c->r[9].ud[0]=0;
        else if(pc>=block+0x4c){
            if(c->r[2].uw[0]!=object+0x30)throw std::runtime_error("effect rotation destination mismatch");
            __m128 input;std::memcpy(&input,record(rt,object+0x80,16),16);float values[4];std::memcpy(values,&input,16);
            for(float v:values){const auto magnitude=std::bit_cast<uint32_t>(v)&0x7fffffff;if(magnitude>=0x7f800000 || magnitude>std::bit_cast<uint32_t>(1.e8f))throw std::runtime_error("unqualified effect rotation quaternion");}
            const auto result=math::quaternionMatrix(input);const auto row=(pc-block-0x4c)/4;std::memcpy(record(rt,object+0x30+16*row,16),&result[row],16);
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
    if(pc==0x40b040 || pc==0x40b040-kPhysicsCopyShift){nativeParticlePhysicsInstruction(c,rt,pc);return false;}
    if(((pc>=0x40a950 && pc<=0x40adf8) || (pc>=0x40a950-kPhysicsCopyShift && pc<=0x40adf8-kPhysicsCopyShift)) && nativeParticlePhysicsInstruction(c,rt,pc))return true;
    // 408e90 (mod003) and its byte-identical copy mod001 27da10 (18b480 lower; only module-local
    // jal targets differ), run by the battle's emitters: both map onto the listed sites.
    const uint32_t motionShift=pc>=0x408f10-0x18b480 && pc<=0x409584-0x18b480?0x18b480u:0u,mpc=pc+motionShift,mentry=c?c->pc+motionShift:0;
    static constexpr std::array<uint32_t,26> particleSites{0x408f10,0x408f84,0x408f88,0x408fa8,0x408fac,0x408fb8,0x408fbc,
        0x409064,0x409068,0x40908c,0x409090,0x40909c,0x4090a0,0x409118,0x4091c0,0x4091c8,0x40931c,0x409320,0x40932c,0x409334,
        0x409400,0x409404,0x40956c,0x409570,0x40957c,0x409584};
    if(mpc>=particleSites.front() && mpc<=particleSites.back() && std::binary_search(particleSites.begin(),particleSites.end(),mpc)){
        const auto object=c->r[4].uw[0],stack=c->r[29].uw[0];auto *owner=record(rt,object,0xf0);record(rt,stack,0x80);
        const auto readWord=[](const uint8_t *p,unsigned at){uint32_t v;std::memcpy(&v,p+at,4);return v;};
        const auto group=readWord(owner,0x10),pool=readWord(owner,0x14),count=readWord(owner,0xc);
        if(!group || group>16 || group!=c->r[25].uw[0] || !count || count>65536 ||
           (mentry!=0x408e90 && mentry!=0x408f30 && mentry!=0x408fe8 && mentry!=0x409148 && mentry!=0x409360 && mentry!=0x4091d0 && mentry!=0x409418))
            throw std::runtime_error("native particle motion owner/continuation mismatch");
        auto *timeRecord=record(rt,0x35dee0,16);float time;std::memcpy(&time,timeRecord+8,4);
        if(mpc==0x408f10 || mpc==0x409118){
            const bool simple=owner[0xd8]&1;
            if(simple!=(mpc==0x408f10) || c->r[mpc==0x408f10?3:16].uw[0]!=std::bit_cast<uint32_t>(time))
                throw std::runtime_error("native particle timestep source mismatch");
            return true;
        }
        const unsigned path=mpc<=0x408fbc?0:mpc<=0x4090a0?1:mpc<=0x409334?2:3;
        const bool simple=owner[0xd8]&1;
        if(simple!=(path<2) || (group==1)!=(path%2==0))throw std::runtime_error("native particle motion branch mismatch");
        const bool loading=mpc==0x408f84 || mpc==0x408f88 || mpc==0x409064 || mpc==0x409068 || mpc==0x4091c0 || mpc==0x4091c8 || mpc==0x409400 || mpc==0x409404;
        const auto cursor=c->r[24].uw[0];if((!loading && !cursor) || (loading?cursor:cursor-1)>=count)throw std::runtime_error("native particle source index outside owner");
        const auto index=loading?cursor:cursor-1,source=pool+index*0x90+0x60;
        record(rt,pool,count*0x90);auto *input=record(rt,source,32);
        constexpr unsigned sourceRegisters[]={10,3,22,21};
        if(c->r[sourceRegisters[path]].uw[0]!=source)throw std::runtime_error("native particle position source mismatch");
        const bool publish=mpc==0x408fb8 || mpc==0x40909c || mpc==0x40932c || mpc==0x40957c;
        const bool feedback=mpc==0x408fbc || mpc==0x4090a0 || mpc==0x409334 || mpc==0x409584;
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
        // Class 163c40 (mod003 3ad540/3f4930) joins the collider table when it is
        // built but writes its plane (+220, +230) only when its spawn timer +240
        // (5.0, minus 2 per update in 3f26c0) runs out and 3f26c0 stores 0 before
        // calling 1fb530. Until then the plane words are whatever the heap held
        // (an old thread stack, different on every run): the port defines that
        // read as no plane, a positive distance, so 1fbff8 takes the miss branch.
        if(pc==0x1fbff0){uint32_t vtable,timer;const auto owner=c->r[4].uw[0];
            std::memcpy(&vtable,record(rt,owner,16),4);
            if(vtable==0x163c40){std::memcpy(&timer,record(rt,owner+0x240,16),4);
                if(c->pc!=0x1fbfb0 || c->r[31].uw[0]!=0x1fe210 || c->r[2].uw[0]!=c->r[29].uw[0])throw std::runtime_error("unqualified plane-contact caller");
                if(timer){kPs2One.store(record(rt,c->r[29].uw[0],16));return true;}}}
        auto contact=planeContact(rt,c);
        if(pc==0x1fbff0)contact.distance.store(record(rt,c->r[29].uw[0],16));
        else{
            if(contact.distance>kPs2Zero || std::bit_cast<uint32_t>(c->f[1])!=contact.distance.bits)throw std::runtime_error("plane-contact projection branch mismatch");
            // ADDAx then MSUBx: p' = (p + 0) - n*d.
            for(unsigned i=0;i<3;++i)((Ps2Word::load(contact.point.data()+4*i)+kPs2Zero)-contact.normal[i]*contact.distance).store(contact.point.data()+4*i);
            std::memcpy(record(rt,c->r[6].uw[0],16),contact.point.data(),16);
        }return true;
    }
    if(pc==0x1fbfbc || pc==0x1fbfc0 || pc==0x1fbfc4 || pc==0x1fbfd0 || pc==0x1fbfd8 || pc==0x1fbfdc || pc==0x1fbfe0 || pc==0x1fbfe4 || pc==0x1fbfe8 || pc==0x1fbfec || pc==0x1fc008 || pc==0x1fc00c)return true;
    if(pc==0x211fe8 || pc==0x212020 || pc==0x212064 || pc==0x212084){
        const auto terms=pointSegmentTerms(rt,c);const auto stack=c->r[29].uw[0];Ps2Word value;
        if(c->r[2].uw[0]!=(pc==0x212020?stack:stack+0x10))throw std::runtime_error("point/segment temporary destination mismatch");
        if(pc==0x211fe8)value=terms.projection;
        else if(pc==0x212020)value=terms.lengthSquared;
        else if(pc==0x212064){
            if(!(terms.projection>kPs2Zero && terms.projection<terms.lengthSquared))throw std::runtime_error("point/segment interior branch mismatch");
            value=kPs2Zero+terms.projection/terms.lengthSquared;
        }else{
            auto difference=terms.delta;
            if(terms.projection>kPs2Zero){
                if(terms.projection>=terms.lengthSquared){for(unsigned i=0;i<3;++i)difference[i]=difference[i]-terms.direction[i];}
                else{const auto ratio=terms.projection/terms.lengthSquared;for(unsigned i=0;i<3;++i)difference[i]=(difference[i]+kPs2Zero)-terms.direction[i]*ratio;}
            }
            Ps2Word squared[3];for(unsigned i=0;i<3;++i)squared[i]=difference[i]*difference[i];value=(squared[0]+squared[1])+kPs2One*squared[2];
        }
        // Only X is used; complete frame/input/output span checks above guard
        // retirement of the other three temporary words.
        value.store(record(rt,c->r[2].uw[0],16));return true;
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
        auto *out=record(rt,c->r[18].uw[0],16);for(unsigned i=0;i<3;++i)normal[i].store(out+4*i);return true;}
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
        uint32_t value=0;switch(pc){
        case 0x212120:value=terms.firstLength;break;case 0x212128:value=terms.negativeCross;break;
        case 0x212130:value=terms.secondLength;break;case 0x21213c:value=terms.firstProjection;break;
        case 0x212144:value=terms.separation;break;case 0x21214c:value=terms.determinant;break;
        case 0x212188:value=terms.negativeSecondProjection;break;case 0x212190:value=terms.secondNumerator;break;
        case 0x212198:value=terms.firstNumerator;break;}
        // Only the scalar word is consumed; both audited callers overwrite
        // these integer temporaries before using a packed or upper component.
        c->r[pc==0x21214c?3:2].uw[0]=value;return true;
    }
    if(pc==0x1fde1c || pc==0x1fde20 || (pc>=0x1fde40 && pc<=0x1fde60 && !(pc&3)) || pc==0x1fde70 || pc==0x1fde78)return true;
    if(pc==0x1fde6c){
        const auto stack=c->r[29].uw[0];auto *origin=record(rt,stack+0x460,16),*direction=record(rt,stack+0x220,16);
        if(c->r[2].uw[0]!=stack+0x460)throw std::runtime_error("collision query origin destination mismatch");
        auto *part=record(rt,c->r[17].uw[0],0x40),*current=record(rt,c->r[18].uw[0],0x40);
        uint32_t otherAddress;std::memcpy(&otherAddress,part+0x38,4);auto *other=record(rt,otherAddress+0x30,16);
        // PS2 words (see Ps2Word): offset = (M0*l.x + M1*l.y) + M2*l.z when
        // shifted (MULA/MADDA/MADD), then A' = A + offset, B' = B + offset,
        // origin = B', direction = A' - B'. W lanes stay the sources' words.
        std::array<uint8_t,16> a,b;std::memcpy(a.data(),other,16);std::memcpy(b.data(),current+0x30,16);
        Ps2Word offset[3]{};const bool shifted=part[0x20]&4;
        if(shifted){
            uint32_t owner;std::memcpy(&owner,current+4,4);auto *basis=record(rt,owner+0x150,48);
            for(unsigned l=0;l<3;++l)offset[l]=(Ps2Word::load(basis+4*l)*Ps2Word::load(part)+Ps2Word::load(basis+16+4*l)*Ps2Word::load(part+4))+Ps2Word::load(basis+32+4*l)*Ps2Word::load(part+8);
        }
        for(unsigned i=0;i<3;++i){auto x=Ps2Word::load(a.data()+4*i),y=Ps2Word::load(b.data()+4*i);
            if(shifted){x=x+offset[i];y=y+offset[i];}y.store(b.data()+4*i);(x-y).store(a.data()+4*i);}
        // Both results precede the first original store; snapshot all aliases first.
        std::memcpy(origin,b.data(),16);std::memcpy(direction,a.data(),16);return true;
    }
    if(pc==0x1fe8b0 || pc==0x1fe8f0 || pc==0x1fe8f4 || pc==0x1fe8f8 || pc==0x1fe8fc || pc==0x1fe924 || pc==0x1fe928)return true;
    if(pc==0x1fe934){
        const auto object=c->r[5].uw[0];auto *source=record(rt,object,0x70);
        if(c->r[3].uw[0]!=object+0x50 || c->r[4].uw[0]!=object+0x30)throw std::runtime_error("velocity update record mismatch");
        // PS2 words (see Ps2Word): v = cur*c - prev*c, times the factor while in contact.
        std::array<uint8_t,16> result;std::memcpy(result.data(),source+0x30,16);
        const bool damped=Ps2Word::load(source+0x1c)>kPs2Zero;const auto coefficient=Ps2Word::of(c->f[20]),factor=Ps2Word::of(c->f[0]);
        for(unsigned i=0;i<3;++i){auto value=Ps2Word::load(source+0x30+4*i)*coefficient-Ps2Word::load(source+0x40+4*i)*coefficient;
            if(damped)value=value*factor;value.store(result.data()+4*i);}
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
        auto *out=record(rt,destination,16);const auto *in=record(rt,source,16);std::array<Ps2Word,4> a,b;
        for(unsigned i=0;i<4;++i){a[i]=Ps2Word::load(out+4*i);b[i]=Ps2Word::load(in+4*i);}
        const auto result=composeConstraintQuaternions(a,b);for(unsigned i=0;i<4;++i)result[i].store(out+4*i);return true;
    }
    if((pc>=0x1fcd08 && pc<=0x1fcd5c && !(pc&3)) ||
       (pc>=0x1fcd6c && pc<=0x1fcda4 && !(pc&3)))return true;
    if(pc==0x1fcdac){
        const auto stack=c->r[29].uw[0];if(c->r[3].uw[0]!=stack+0x140)throw std::runtime_error("angular-constraint cosine destination mismatch");
        const auto directions=angularDirections(rt,c);directions.cosine.store(record(rt,stack+0x140,16));return true;
    }
    if(pc==0x1fcdb0 || pc==0x1fcdb4 || pc==0x1fcdc8 || pc==0x1fcdd0 || pc==0x1fcdd4 || pc==0x1fcdd8 ||
       pc==0x1fcddc || pc==0x1fcde0 || pc==0x1fcde4 || pc==0x1fcde8 || pc==0x1fcdec)return true;
    if(pc==0x1fcdf0){
        const auto stack=c->r[29].uw[0];auto *result=record(rt,stack+0x130,16);
        if(c->r[2].uw[0]!=stack+0x130 || c->r[3].uw[0]!=stack+0x140 || std::bit_cast<uint32_t>(c->f[1])!=0x3f7ffffe)
            throw std::runtime_error("angular-constraint quaternion record mismatch");
        uint32_t cosineBits;std::memcpy(&cosineBits,record(rt,stack+0x140,16),4);
        const auto directions=angularDirections(rt,c);
        if(directions.cosine.bits!=cosineBits || std::bit_cast<uint32_t>(c->f[0])!=cosineBits || !(Ps2Word::of(c->f[0])<Ps2Word::of(c->f[1])))
            throw std::runtime_error("angular-constraint quaternion has stale cosine/branch inputs");
        // PS2 words, 1fcdb0..1fcdec: root = SQRT(2 + cos*2), Q = DIV(1, root),
        // xyz = (OPMULA/OPMSUB cross) * Q, w = (1*root) * vf29.x (0.5).
        const auto &a=directions.first,&b=directions.second;const Ps2Word two{0x40000000u},half{0x3f000000u};
        const auto root=(two+directions.cosine*two).root(),reciprocal=kPs2One/root;
        Ps2Word quaternion[4];for(unsigned i=0;i<3;++i){const auto j=(i+1)%3,k=(i+2)%3;quaternion[i]=(b[j]*a[k]-a[j]*b[k])*reciprocal;}
        quaternion[3]=(kPs2One*root)*half;for(unsigned i=0;i<4;++i)quaternion[i].store(result+4*i);return true;
    }
    if(pc>=0x216da4 && pc<=0x216dbc && !(pc&3))return true;
    if(pc==0x216e04){
        const auto matrixAddress=c->r[5].uw[0],frame=c->r[29].uw[0];
        auto *matrixRecord=record(rt,matrixAddress,64),*translationRecord=record(rt,frame,16);
        if(c->r[3].uw[0]!=frame)throw std::runtime_error("native transposed inverse translation destination mismatch");
        const auto a=reinterpret_cast<uintptr_t>(matrixRecord),b=reinterpret_cast<uintptr_t>(translationRecord);
        if(a<b+16 && b<a+64)throw std::runtime_error("native transposed inverse output aliases its temporary");
        // Scalar clear, sequential translation loads and integer transpose have
        // already executed (the rows at a1 are the source's columns). PS2 words
        // (see Ps2Word), 216db4..216dbc: xyz = (r0*t.x + r1*t.y) + r2*t.z;
        // W keeps the loaded word. Every word is a VU number: the game reads
        // a collider plane before 3f26c0 has written it (class 163c40, first
        // updates after spawn), and the VU saturates what that heap garbage gives.
        const auto rows=ps2Rows(rt,matrixAddress,3);const auto point=ps2Record(rt,frame);
        for(unsigned lane=0;lane<3;++lane)((rows.rows[lane][0]*point[0]+rows.rows[lane][1]*point[1])+rows.rows[lane][2]*point[2]).store(translationRecord+4*lane);
        return true;
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
        const auto geometry=positionPairGeometry(first,second);
        // The scalar tail reads only this distance. YZW die at frame release;
        // aliases that could make those retired stores observable reject above.
        geometry.distance.store(stack);return true;
    }
    if(pc==0x1fc984 || pc==0x1fc9cc || pc==0x1fc7f8 || pc==0x1fc844){
        auto *first=record(rt,c->r[5].uw[0],16),*second=record(rt,c->r[4].uw[0],16),*frame=record(rt,c->r[29].uw[0],16);
        if(c->r[4].uw[0]!=c->r[6].uw[0]+0x30)throw std::runtime_error("constrained-position output record mismatch");
        separatePairFrame(frame,first,16);separatePairFrame(frame,second,16);
        const auto geometry=positionPairGeometry(first,second);
        const auto firstWeight=Ps2Word::of(pc==0x1fc984 || pc==0x1fc7f8?c->f[0]:c->f[1]),secondWeight=Ps2Word::of(c->f[0]);
        // VU DIV Q = 1/distance (+/-max for a zero distance), MULq, MULAw/MADDx.
        const auto inverse=kPs2One/geometry.distance;Ps2Word direction[3];for(unsigned i=0;i<3;++i)direction[i]=geometry.difference[i]*inverse;
        std::array<uint8_t,16> outA,outB;std::memcpy(outA.data(),first,16);std::memcpy(outB.data(),second,16);
        for(unsigned i=0;i<3;++i){(Ps2Word::load(first+4*i)*kPs2One+direction[i]*firstWeight).store(outA.data()+4*i);
            (Ps2Word::load(second+4*i)*kPs2One-direction[i]*secondWeight).store(outB.data()+4*i);}
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
        if(!owner->qualifiedMotionCallback(rt,callback))throw std::runtime_error(std::format("unqualified native motion callback {:x} (object {:x}, target {:x})",callback,object,targetObject));
        // PS2 words (see Ps2Word): p' = (((p+0) + g.y*dt^2) + a*dt - v*(damping*dt)) + v*dt,
        // each term present only for its flag (1 gravity on Y, 0x40 acceleration, 2 damping).
        const auto flags=source[1];std::array<uint32_t,4> out;std::memcpy(out.data(),source+0x30,16);
        const uint8_t *gravity=flags&1?record(rt,0x359100,16):nullptr,*acceleration=flags&0x40?record(rt,c->r[5].uw[0],16):nullptr;
        const auto dt=Ps2Word::of(c->f[12]),damping=flags&2?Ps2Word::of(c->f[0]):kPs2Zero;
        const auto squared=dt*dt,drag=flags&2?damping*dt:kPs2Zero;
        for(unsigned i=0;i<3;++i){const auto velocity=Ps2Word::load(source+0x50+4*i);auto value=Ps2Word::load(source+0x30+4*i)+kPs2Zero;
            if(gravity && i==1)value=value+Ps2Word::load(gravity+4)*squared;
            if(acceleration)value=value+Ps2Word::load(acceleration+4*i)*dt;
            if(flags&2)value=value-velocity*drag;
            out[i]=(value+velocity*dt).bits;}
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
        modelLightVariants()[{rt,model}]=*rt->memory.hostPointer(global+0x517,1)!=0;
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
        // Program 0x898 (vf9-12 bone rows, vf13-16 local rows, vf28-31 base
        // rows): M_k = sum_j local_j * bone_k[j], N_k = sum_j M_j * base_k[j]
        // (k < 3), M_3 = N_3 = bone_3. (Fixed 2026-11-10: the bone/base roles
        // were swapped, which spiked close-up characters.)
        for(unsigned i=0;i<3;++i)relative[i]=transform(local,bone[i]);relative[3]=bone[3];
        for(unsigned i=0;i<3;++i)result[i]=transform(relative,base[i]);result[3]=relative[3];
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
            if(e==0x7f800000)throw std::runtime_error(std::format("unqualified native row dot input {:08x}",bits));};
        // The VU reads denormal inputs as signed zero (an uninitialized stack W
        // lane reaches this from mod003 380300), so flush them rather than reject.
        for(auto &row:matrix)for(float &value:row){finite(value);value=normal(value);}
        for(float &value:vector){finite(value);value=normal(value);}
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
    default:
        // Inlined-block recognisers last: they read code around the PC, so
        // only sites no specific handler claimed pay for that scan.
        return nativeParticleStep(c,rt,pc) || nativeParticleBox(c,rt,pc) || nativeTrailEmitter(c,rt,pc) || nativeConstantStore(c,rt,pc) || nativeLengthIdiom(c,rt,pc) || nativeMacroIdiom(c,rt,pc) || nativeInverseIdiom(c,rt,pc) || nativeQuaternionIdiom(c,rt,pc);
    }
}
// Save states: the marker projection rows kept between their loads and the
// projections (a dispatcher safe point can fall in between).
void serializeMathState(bozo::state::Archive &a,const Runtime *rt){
    auto &m=markerRows(rt);for(auto &row:m.rows)bozo::state::io(a,row);bozo::state::fields(a,m.loaded,m.point);bozo::state::io(a,m.v);
}
}

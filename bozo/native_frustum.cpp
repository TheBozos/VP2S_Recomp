#include "native_frustum.h"
#include "native_scene.h"
#include <algorithm>
namespace vp2s {
namespace {
__m128 normal(__m128 v){
    const auto bits=_mm_castps_si128(v);
    const auto tiny=_mm_cmpeq_epi32(_mm_and_si128(bits,_mm_set1_epi32(0x7f800000)),_mm_setzero_si128());
    return _mm_castsi128_ps(_mm_andnot_si128(_mm_and_si128(tiny,_mm_set1_epi32(0x7fffffff)),bits));
}
__m128 xyz(__m128 v){return _mm_blend_ps(v,_mm_setzero_ps(),8);}
__m128 mul(__m128 a,__m128 b){return normal(_mm_mul_ps(a,b));}
__m128 sub(__m128 a,__m128 b){return normal(_mm_sub_ps(a,b));}
__m128 cross(__m128 a,__m128 b){
    a=xyz(a);b=xyz(b);
    const auto ayzx=_mm_shuffle_ps(a,a,_MM_SHUFFLE(3,0,2,1));
    const auto azxy=_mm_shuffle_ps(a,a,_MM_SHUFFLE(3,1,0,2));
    const auto byzx=_mm_shuffle_ps(b,b,_MM_SHUFFLE(3,0,2,1));
    const auto bzxy=_mm_shuffle_ps(b,b,_MM_SHUFFLE(3,1,0,2));
    return sub(mul(ayzx,bzxy),mul(byzx,azxy));
}
void qualified(__m128 v,unsigned count=4){uint32_t bits[4];std::memcpy(bits,&v,16);
    for(unsigned i=0;i<count;++i){const auto exponent=bits[i]&0x7f800000;
        if(exponent==0x7f800000 || (!exponent && (bits[i]&0x7fffff)))
            throw std::runtime_error("unqualified native frustum input/result");}}
__m128 unit(__m128 v){
    qualified(v,3);const auto square=mul(v,v);
    auto length2=normal(_mm_add_ss(square,music::broadcast<1>(square)));
    length2=normal(_mm_add_ss(length2,mul(_mm_set1_ps(1),music::broadcast<2>(square))));
    const float squared=music::laneValue(length2,0);
    if(!std::isfinite(squared))throw std::runtime_error("unqualified native frustum length overflow");
    volatile float length=std::sqrt(squared);
    volatile float reciprocal=length==0?std::numeric_limits<float>::max():1.f/length;
    const auto out=mul(v,_mm_set1_ps(reciprocal));qualified(out,3);return out;
}
uint8_t *record(Runtime *rt,uint32_t at,uint32_t bytes){
    if(!rt || (at&15))throw std::runtime_error("unaligned native frustum record");
    const bool scratch=at>=bozo::kScratchpadBase && at<bozo::kScratchpadBase+bozo::kScratchpadBytes;
    if(!scratch && ((at&0x1ffffff)<0x100000 || (at>>28)==7))throw std::runtime_error("invalid native frustum record");
    auto *p=rt->memory.hostPointer(at,bytes);if(!p)throw std::runtime_error("native frustum record outside owned memory");return p;
}
}
FrustumResult cameraFrustum(math::Matrix world,const FrustumRecords &points){
    for(auto v:world)qualified(v);for(auto v:points)qualified(v,3);
    _MM_TRANSPOSE4_PS(world[0],world[1],world[2],world[3]);
    {music::CodecRounding rounding;
        for(unsigned i=0;i<4;++i){float column[4];std::memcpy(column,&world[i],16);
            for(unsigned lane=0;lane<4;++lane)if(i!=lane){
                volatile float v=lane==3?1.f*column[lane]:0.f+column[lane];column[lane]=v;}
            std::memcpy(&world[i],column,16);}}
    FrustumResult out;
    for(unsigned i=0;i<10;++i){
        const auto point=_mm_blend_ps(points[i],_mm_set1_ps(1),8);
        out.points[i]=math::matrixVectorProduct(world,point);qualified(out.points[i]);
    }
    auto edge=[&](unsigned to,unsigned from){return sub(xyz(out.points[to]),xyz(out.points[from]));};
    auto radial=[&](unsigned i){return sub(xyz(out.points[i]),xyz(world[3]));};
    for(unsigned i=0;i<4;++i)out.normals[i]=unit(cross(radial(i),edge((i+1)%4,i)));
    out.normals[4]=unit(cross(edge(1,0),edge(2,0)));
    out.normals[5]=mul(out.normals[4],_mm_set1_ps(-1));
    for(unsigned i=0;i<4;++i)out.normals[6+i]=unit(cross(radial(6+i),edge(6+(i+1)%4,6+i)));
    // Masked XYZ calculations preserve W from the six actual stack records,
    // and the second four normals retain W from the first four normals.
    for(unsigned i=0;i<6;++i)out.normals[i]=_mm_blend_ps(out.normals[i],points[4+i],8);
    for(unsigned i=0;i<4;++i)out.normals[6+i]=_mm_blend_ps(out.normals[6+i],out.normals[i],8);
    return out;
}
uint8_t sphereAngleVisibility(const FrustumResult &planes,__m128 sphere){
    qualified(sphere);const auto radius=music::laneValue(sphere,3);
    if(radius<0)throw std::runtime_error("negative native angle sphere radius");
    for(unsigned i=6;i<10;++i){qualified(planes.points[i],3);qualified(planes.normals[i],3);}
    music::CodecRounding rounding;
    const auto sum=[](__m128 a,__m128 b){return normal(music::codecSum(a,b));};
    const auto difference=[&](__m128 a,__m128 b){return sum(a,_mm_xor_ps(b,_mm_castsi128_ps(_mm_set1_epi32(int(0x80000000u)))));};
    bool intersects=false;
    for(unsigned i=0;i<4;++i){
        const auto delta=difference(xyz(sphere),xyz(planes.points[6+i]));
        const auto terms=mul(delta,xyz(planes.normals[6+i]));
        const auto first=i&1?sum(music::broadcast<1>(terms),music::broadcast<0>(terms)):
                             sum(music::broadcast<0>(terms),music::broadcast<1>(terms));
        const auto distance=sum(first,music::broadcast<2>(terms));qualified(distance,1);
        const auto margin=difference(distance,_mm_set1_ps(radius));qualified(margin,1);
        intersects|=std::signbit(music::laneValue(margin,0));
    }
    // This is the original caller's classification byte, not a status bank.
    return intersects?0x80:0;
}
NativeSphereVisibility sphereVisibility(math::Matrix clip,const FrustumResult &planes,__m128 sphere){
    qualified(sphere);const auto radius=music::laneValue(sphere,3);
    if(radius<0)throw std::runtime_error("negative native visibility sphere radius");
    for(auto &column:clip){qualified(column,3);column=xyz(column);}
    for(unsigned i=0;i<6;++i){qualified(planes.points[i],3);qualified(planes.normals[i],3);}
    music::CodecRounding rounding;
    const auto sum=[](__m128 a,__m128 b){return normal(music::codecSum(a,b));};
    const auto difference=[&](__m128 a,__m128 b){return sum(a,_mm_xor_ps(b,_mm_castsi128_ps(_mm_set1_epi32(int(0x80000000u)))));};
    auto projected=mul(clip[0],music::broadcast<0>(sphere));
    projected=sum(projected,mul(clip[1],music::broadcast<1>(sphere)));
    projected=sum(projected,mul(clip[2],music::broadcast<2>(sphere)));
    projected=sum(projected,mul(clip[3],_mm_set1_ps(1)));
    qualified(projected,3);const auto z=music::laneValue(projected,2);
    if(z==0)throw std::runtime_error("unqualified native visibility projection at zero depth");
    volatile float inverse=1.f/z;
    const auto divided=mul(projected,_mm_set1_ps(inverse));qualified(divided,2);
    projected=_mm_blend_ps(projected,divided,3);projected=_mm_blend_ps(projected,sphere,8);
    NativeSphereVisibility out;std::memcpy(out.input.data(),&sphere,16);std::memcpy(out.projected.data(),&projected,16);
    out.nearPoint=out.farPoint=out.projected;
    float distances[6];
    for(unsigned i=0;i<6;++i){
        const auto delta=difference(xyz(sphere),xyz(planes.points[i]));const auto terms=mul(delta,xyz(planes.normals[i]));
        const auto first=i%3==2?sum(music::broadcast<2>(terms),music::broadcast<1>(terms)):
                         i%3==1?sum(music::broadcast<1>(terms),music::broadcast<0>(terms)):
                                sum(music::broadcast<0>(terms),music::broadcast<1>(terms));
        const auto distance=sum(first,i%3==2?music::broadcast<0>(terms):music::broadcast<2>(terms));
        qualified(distance,1);distances[i]=music::laneValue(distance,0);
        out.outside|=std::signbit(music::laneValue(sum(distance,_mm_set1_ps(radius)),0));
    }
    for(unsigned i:{4u,5u})out.intersects|=std::signbit(music::laneValue(difference(_mm_set1_ps(distances[i]),_mm_set1_ps(radius)),0));
    out.nearPoint[2]=music::laneValue(difference(_mm_set1_ps(z),_mm_set1_ps(radius)),0);
    out.farPoint[2]=music::laneValue(sum(_mm_set1_ps(z),_mm_set1_ps(radius)),0);
    return out;
}
bool nativeFrustumInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc&3)return false;
    // Every word in these two spans is an audited vector calculation/transfer;
    // the scalar cursor changes at 207fc8,207ff4,2080f8 stay outside them.
    const bool early=pc==0x207d78 || pc==0x207d7c || pc==0x207d80 || pc==0x207d84 || pc==0x207d90;
    const bool math=(pc>=0x207ef4 && pc<=0x207fc4) || (pc>=0x207fcc && pc<=0x207ff0) ||
                    (pc>=0x207ff8 && pc<=0x2080f4) || (pc>=0x2080fc && pc<=0x2081bc);
    if(!early && !math)return false;
    if(pc==0x207fcc || pc==0x208080){
        math::Matrix world;FrustumRecords points;
        std::memcpy(world.data(),record(rt,c->r[4].uw[0]+0x90,64),64);
        std::memcpy(points.data(),record(rt,c->r[29].uw[0],160),160);
        const auto result=cameraFrustum(world,points);
        std::memcpy(record(rt,c->r[4].uw[0]+(pc==0x207fcc?0x460:0x500),160),
                    pc==0x207fcc?result.points.data():result.normals.data(),160);
    }
    return true;
}
}

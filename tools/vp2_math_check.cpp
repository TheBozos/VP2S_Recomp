// Offline comparison of native matrices with the original lifted game algorithm.
#include "vp2s_math.h"
#include "runtime/ps2_vu1.h"
#include <cstdio>
#include <random>

struct R5900Context;
extern const uint32_t g_ps2RecompiledFunctionTableBase=0u;
extern const uint32_t g_ps2RecompiledFunctionTableEnd=0u;
extern const uint32_t g_ps2RecompiledFunctionTableSlotCount=0u;
void (*g_ps2RecompiledFunctionTable[1])(uint8_t*,R5900Context*,PS2Runtime*)={nullptr};
namespace ps2x::vu1lift {
using Kick=void (*)(void*,const uint8_t*,uint32_t);
template<bool> uint64_t matrix_reference(uint32_t,VU1State&,uint8_t*,uint32_t,Kick,void*);
template<bool> uint64_t normalize_reference(uint32_t,VU1State&,uint8_t*,uint32_t,Kick,void*);
}
int main(int argc,char **argv) {
    if(argc!=2) return 2;
    using vp2s::math::Matrix;
    uint64_t cases=0,differences=0,capturedDifferences=0;
    vp2s::music::CodecRounding rounding;
    auto test=[&](const Matrix &a,const Matrix &b,const Matrix *captured) {
        VU1State state{}; state.vf[0][3]=1.f;
        std::memcpy(state.vf[1],a.data(),64); std::memcpy(state.vf[5],b.data(),64);
        alignas(16) uint8_t data[4096]{};
        ps2x::vu1lift::matrix_reference<false>(0,state,data,0,nullptr,nullptr);
        const auto result=vp2s::math::compose(a,b);
        differences+=std::memcmp(result.data(),state.vf[1],64)!=0;
        if(captured) capturedDifferences+=std::memcmp(captured->data(),state.vf[1],64)!=0;
        ++cases;
    };
    auto testQuaternion=[&](__m128 value,const __m128 *captured) {
        VU1State state{}; state.vf[0][3]=1.f;
        std::memcpy(state.vf[5],&value,16);
        alignas(16) uint8_t data[4096]{};
        ps2x::vu1lift::normalize_reference<false>(0,state,data,0,nullptr,nullptr);
        const auto result=vp2s::math::normalizeQuaternion(value);
        differences+=std::memcmp(&result,state.vf[6],16)!=0;
        if(captured) capturedDifferences+=std::memcmp(captured,state.vf[6],16)!=0;
        ++cases;
    };
    bool quaternionOnly=std::strcmp(argv[1],"--normalize-synthetic")==0;
    if(quaternionOnly) {
        std::mt19937 random(0x56503253u);
        std::uniform_real_distribution<float> value(-1000.f,1000.f);
        for(unsigned k=0;k<65536;++k)
            testQuaternion(_mm_setr_ps(value(random),value(random),value(random),value(random)),nullptr);
        for(float scale:{0.f,-0.f,1.f,-1.f,1.e-18f,1.e18f})
            for(int lane=0;lane<4;++lane) {
                float v[4]{};v[lane]=scale;
                testQuaternion(_mm_loadu_ps(v),nullptr);
            }
    } else if(std::strcmp(argv[1],"--synthetic")==0) {
        std::mt19937 random(0x56503253u);
        std::uniform_real_distribution<float> value(-1000.f,1000.f);
        for(unsigned k=0;k<65536;++k) {
            Matrix a,b;
            for(unsigned column=0;column<4;++column) {
                a[column]=_mm_setr_ps(value(random),value(random),value(random),value(random));
                b[column]=_mm_setr_ps(value(random),value(random),value(random),value(random));
            }
            test(a,b,nullptr);
            if(k<16) test(a,a,nullptr);
        }
    } else {
        FILE *file=std::fopen(argv[1],"rb"); if(!file) return 2;
        for(;;) {
            uint32_t tag=0;
            if(std::fread(&tag,4,1,file)!=1) break;
            if(tag==0x3152344eu) {
                __m128 value,result;
                if(std::fread(&value,1,16,file)!=16 || std::fread(&result,1,16,file)!=16) {std::fclose(file);return 2;}
                testQuaternion(value,&result);quaternionOnly=true;continue;
            }
            Matrix a,b,result;
            if(tag!=0x3152344du || std::fread(&a,1,64,file)!=64 || std::fread(&b,1,64,file)!=64 ||
               std::fread(&result,1,64,file)!=64) {std::fclose(file);return 2;}
            test(a,b,&result);
        }
        std::fclose(file);
    }
    std::printf("native %s: %llu cases / %llu scalar outputs, reference differences %llu, captured differences %llu\n",
                quaternionOnly?"quaternions":"matrices",cases,cases*(quaternionOnly?4:16),differences,capturedDifferences);
    return cases && !differences && !capturedDifferences?0:1;
}

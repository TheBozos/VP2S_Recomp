// Offline comparison of native matrices with the original lifted game algorithm.
#include "vp2s_math.h"
#include "runtime/ps2_vu1.h"
#include "ps2_runtime_macros.h"
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
template<bool> uint64_t quaternion_matrix_reference(uint32_t,VU1State&,uint8_t*,uint32_t,Kick,void*);
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
    auto testQuaternionMatrix=[&](__m128 value,const Matrix *captured) {
        VU1State state{}; state.vf[0][3]=1.f;
        std::memcpy(state.vf[5],&value,16);
        alignas(16) uint8_t data[4096]{};
        ps2x::vu1lift::quaternion_matrix_reference<false>(0,state,data,0,nullptr,nullptr);
        const auto result=vp2s::math::quaternionMatrix(value);
        if(captured && std::memcmp(result.data(),state.vf[8],64)) {
            uint32_t native[16],reference[16];
            std::memcpy(native,result.data(),64);std::memcpy(reference,state.vf[8],64);
            for(unsigned lane=0;lane<16;++lane) if(native[lane]!=reference[lane])
                std::printf("quaternion matrix lane %u: native %08x reference %08x\n",lane,native[lane],reference[lane]);
        }
        differences+=std::memcmp(result.data(),state.vf[8],64)!=0;
        if(captured) capturedDifferences+=std::memcmp(captured->data(),state.vf[8],64)!=0;
        ++cases;
    };
    bool quaternionOnly=std::strcmp(argv[1],"--normalize-synthetic")==0;
    const bool quaternionZeros=std::strcmp(argv[1],"--quat-matrix-zeros")==0;
    const bool quaternionSmall=std::strcmp(argv[1],"--quat-matrix-small")==0;
    bool quaternionMatrix=std::strcmp(argv[1],"--quat-matrix-synthetic")==0 || quaternionZeros || quaternionSmall;
    if(quaternionSmall) {
        testQuaternionMatrix(_mm_setr_ps(0.f,0.f,1.e-20f,1.f),nullptr);
        std::mt19937 random(0x56503253u);
        for(unsigned k=0;k<65536;++k) {
            uint32_t bits[4];
            for(auto &v:bits) v=(random()&0x807fffffu)|((random()%128u)<<23);
            testQuaternionMatrix(_mm_castsi128_ps(_mm_loadu_si128(reinterpret_cast<const __m128i*>(bits))),nullptr);
        }
    } else if(quaternionZeros) {
        const float values[]={0.f,-0.f,1.f,-1.f,0.70710677f,-0.70710677f};
        for(float x:values) for(float y:values) for(float z:values) for(float w:values)
            testQuaternionMatrix(_mm_setr_ps(x,y,z,w),nullptr);
    } else if(std::strcmp(argv[1],"--attachment-synthetic")==0) {
        std::mt19937 random(0x56503253u);
        std::uniform_real_distribution<float> value(-10.f,10.f);
        for(unsigned mode:{unsigned(_MM_ROUND_NEAREST),unsigned(_MM_ROUND_TOWARD_ZERO)}) {
            _mm_setcsr((_mm_getcsr() & ~_MM_ROUND_MASK)|mode);
            for(unsigned k=0;k<32768;++k) {
                const auto q=vp2s::math::normalizeQuaternion(_mm_setr_ps(value(random),value(random),value(random),value(random)));
                const auto rotation=vp2s::math::quaternionMatrix(q);
                VU1State referenceState{};referenceState.vf[0][3]=1.f;
                std::memcpy(referenceState.vf[5],&q,16);
                alignas(16) uint8_t referenceData[4096]{};
                {
                    vp2s::music::CodecRounding referenceRounding;
                    ps2x::vu1lift::quaternion_matrix_reference<false>(0,referenceState,referenceData,0,nullptr,nullptr);
                }
                Matrix referenceRotation;
                std::memcpy(referenceRotation.data(),referenceState.vf[8],64);
                Matrix attachment,original;
                for(unsigned column=0;column<4;++column)
                    attachment[column]=_mm_setr_ps(value(random),value(random),value(random),value(random));
                // Original generated EE macro arithmetic, independently of the
                // VU microprogram reference. No execution context is involved.
                for(unsigned column=0;column<4;++column) {
                    auto acc=PS2_VMUL(referenceRotation[0],vp2s::music::broadcast<0>(attachment[column]));
                    acc=PS2_VADD(acc,PS2_VMUL(referenceRotation[1],vp2s::music::broadcast<1>(attachment[column])));
                    acc=PS2_VADD(acc,PS2_VMUL(referenceRotation[2],vp2s::music::broadcast<2>(attachment[column])));
                    original[column]=PS2_VADD(acc,PS2_VMUL(referenceRotation[3],vp2s::music::broadcast<3>(attachment[column])));
                }
                // The three callback-bearing routines use these existing stack
                // slots. Check the stored rotation plus the later composition,
                // with the original quaternion-to-matrix and EE macro sequence
                // as the independent oracle, under both host rounding modes.
                for(unsigned offset:{0x50u,0xb0u,0x90u}) {
                    alignas(16) uint8_t frame[256];std::memset(frame,0xc5,sizeof(frame));
                    std::memcpy(frame+offset,rotation.data(),64);
                    Matrix staged;std::memcpy(staged.data(),frame+offset,64);
                    const auto result=vp2s::math::composeAttachment(attachment,staged);
                    std::memcpy(frame+offset,result.data(),64);
                    differences+=std::memcmp(frame+offset,original.data(),64)!=0;
                    for(unsigned byte=0;byte<sizeof(frame);++byte)
                        if((byte<offset || byte>=offset+64) && frame[byte]!=0xc5) ++differences;
                    ++cases;
                }
            }
        }
    } else if(quaternionMatrix) {
        std::mt19937 random(0x56503253u);
        std::uniform_real_distribution<float> value(-1000.f,1000.f);
        for(unsigned k=0;k<65536;++k) {
            const auto q=_mm_setr_ps(value(random),value(random),value(random),value(random));
            testQuaternionMatrix(q,nullptr);
            testQuaternionMatrix(vp2s::math::normalizeQuaternion(q),nullptr);
        }
        for(float scale:{0.f,-0.f,1.f,-1.f,1.e-18f,1.e18f})
            for(int lane=0;lane<4;++lane) {
                float v[4]{};v[lane]=scale;
                testQuaternionMatrix(_mm_loadu_ps(v),nullptr);
            }
    } else if(quaternionOnly) {
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
            if(tag==0x31524d51u) {
                __m128 value;Matrix result;
                if(std::fread(&value,1,16,file)!=16 || std::fread(&result,1,64,file)!=64) {std::fclose(file);return 2;}
                testQuaternionMatrix(value,&result);quaternionMatrix=true;continue;
            }
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
                quaternionMatrix?"quaternion matrices":quaternionOnly?"quaternions":"matrices",cases,cases*(quaternionOnly?4:16),differences,capturedDifferences);
    return cases && !differences && !capturedDifferences?0:1;
}

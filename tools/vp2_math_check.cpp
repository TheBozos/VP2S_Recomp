// Offline comparison of native matrices with the original lifted game algorithm.
#include "vp2s_math.h"
#include "runtime/ps2_vu1.h"
#include "ps2_runtime_macros.h"
#include <cstdio>
#include <random>
// Exercise the exact production record bridges in the offline function tests.
// This target never calls vp2sInstallMath or starts a game/runtime instance.
#include "../port/vp2s_math.cpp"

namespace vp2s::mathfixture {
void originalPerspective(uint8_t*,R5900Context*,PS2Runtime*);
void originalOrthographic(uint8_t*,R5900Context*,PS2Runtime*);
void nativePerspective(uint8_t*,R5900Context*,PS2Runtime*);
void nativeOrthographic(uint8_t*,R5900Context*,PS2Runtime*);
void originalColumnRigid(uint8_t*,R5900Context*,PS2Runtime*);
void nativeColumnRigid(uint8_t*,R5900Context*,PS2Runtime*);
void originalOffsetDot(uint8_t*,R5900Context*,PS2Runtime*);
void nativeOffsetDot(uint8_t*,R5900Context*,PS2Runtime*);
void originalTransposedRigid(uint8_t*,R5900Context*,PS2Runtime*);
void nativeTransposedRigid(uint8_t*,R5900Context*,PS2Runtime*);
void originalObjectLinear(uint8_t*,R5900Context*,PS2Runtime*);
void nativeObjectLinear(uint8_t*,R5900Context*,PS2Runtime*);
void originalLinear0(uint8_t*,R5900Context*,PS2Runtime*);
void originalLinear1(uint8_t*,R5900Context*,PS2Runtime*);
void originalLinear2(uint8_t*,R5900Context*,PS2Runtime*);
void originalLinear3(uint8_t*,R5900Context*,PS2Runtime*);
void nativeLinear0(uint8_t*,R5900Context*,PS2Runtime*);
void nativeLinear1(uint8_t*,R5900Context*,PS2Runtime*);
void nativeLinear2(uint8_t*,R5900Context*,PS2Runtime*);
void nativeLinear3(uint8_t*,R5900Context*,PS2Runtime*);
void originalMatrixVector0(uint8_t*,R5900Context*,PS2Runtime*);
void originalMatrixVector1(uint8_t*,R5900Context*,PS2Runtime*);
void originalMatrixVector2(uint8_t*,R5900Context*,PS2Runtime*);
void nativeMatrixVector0(uint8_t*,R5900Context*,PS2Runtime*);
void nativeMatrixVector1(uint8_t*,R5900Context*,PS2Runtime*);
void nativeMatrixVector2(uint8_t*,R5900Context*,PS2Runtime*);
void originalObjectMatrix0(uint8_t*,R5900Context*,PS2Runtime*);
void originalObjectMatrix1(uint8_t*,R5900Context*,PS2Runtime*);
void originalObjectMatrix2(uint8_t*,R5900Context*,PS2Runtime*);
void nativeObjectMatrix0(uint8_t*,R5900Context*,PS2Runtime*);
void nativeObjectMatrix1(uint8_t*,R5900Context*,PS2Runtime*);
void nativeObjectMatrix2(uint8_t*,R5900Context*,PS2Runtime*);
}

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
    auto testRigid=[&](const vp2s::math::RigidBasis &basis,__m128 translation,uint32_t mode,const __m128 *captured,bool column=false) {
        const auto saved=_mm_getcsr();_mm_setcsr((saved&~0xe040u)|mode);
        // The three original EE macro operations, as generated from the USA
        // helper. Only xyz are written back; the fourth payload is untouched.
        auto acc=PS2_VMUL(basis[0],vp2s::music::broadcast<0>(translation));
        acc=PS2_VADD(acc,PS2_VMUL(basis[1],vp2s::music::broadcast<1>(translation)));
        auto original=_mm_blend_ps(PS2_VADD(acc,PS2_VMUL(basis[2],vp2s::music::broadcast<2>(translation))),translation,8);
        if(column) original=_mm_blend_ps(PS2_VMUL(original,_mm_set1_ps(-1.f)),translation,8);
        const auto result=column?vp2s::math::rigidInverseColumnTranslation(basis,translation):vp2s::math::rigidInverseTranslation(basis,translation);
        differences+=std::memcmp(&result,&original,16)!=0;
        if(captured) capturedDifferences+=std::memcmp(captured,&original,16)!=0;
        _mm_setcsr(saved);++cases;
    };
    auto testProjection=[&](const Matrix &local,const Matrix &view,uint32_t mode,const Matrix *captured) {
        const auto saved=_mm_getcsr();_mm_setcsr((saved&~0xe040u)|mode);
        Matrix original;
        for(unsigned column=0;column<4;++column) {
            auto acc=PS2_VMUL(view[0],vp2s::music::broadcast<0>(local[column]));
            acc=PS2_VADD(acc,PS2_VMUL(view[1],vp2s::music::broadcast<1>(local[column])));
            acc=PS2_VADD(acc,PS2_VMUL(view[2],vp2s::music::broadcast<2>(local[column])));
            original[column]=PS2_VADD(acc,PS2_VMUL(view[3],vp2s::music::broadcast<3>(local[column])));
        }
        const auto result=vp2s::math::projectionProduct(local,view);
        differences+=std::memcmp(result.data(),original.data(),64)!=0;
        if(captured) capturedDifferences+=std::memcmp(captured->data(),original.data(),64)!=0;
        _mm_setcsr(saved);++cases;
    };
    auto testOffsetDot=[&](__m128 point,__m128 axis,__m128 origin,uint32_t mode,const __m128 *captured) {
        const auto saved=_mm_getcsr();_mm_setcsr((saved&~0xe040u)|mode);
        const auto zero=_mm_setzero_ps();
        const auto difference=PS2_VSUB(_mm_blend_ps(point,zero,8),_mm_blend_ps(origin,zero,8));
        const auto terms=PS2_VMUL(difference,_mm_blend_ps(axis,zero,8));
        const auto xy=PS2_VADD(terms,vp2s::music::broadcast<1>(terms));
        const auto dot=PS2_VADD(xy,PS2_VMUL(_mm_set1_ps(1.f),vp2s::music::broadcast<2>(terms)));
        const auto original=_mm_move_ss(point,dot);
        const auto result=vp2s::math::offsetDotRecord(point,axis,origin);
        differences+=std::memcmp(&result,&original,16)!=0;
        if(captured)capturedDifferences+=std::memcmp(captured,&original,16)!=0;
        _mm_setcsr(saved);++cases;
    };
    auto testTransposedRigid=[&](const Matrix &matrix,__m128 translation,uint32_t mode,const __m128 *captured) {
        const auto saved=_mm_getcsr();_mm_setcsr((saved&~0xe040u)|mode);
        float values[16],basis[3][4];std::memcpy(values,matrix.data(),64);
        for(unsigned row=0;row<3;++row)for(unsigned lane=0;lane<4;++lane)basis[row][lane]=values[lane*4+row];
        auto acc=PS2_VMUL(_mm_loadu_ps(basis[0]),vp2s::music::broadcast<0>(translation));
        acc=PS2_VADD(acc,PS2_VMUL(_mm_loadu_ps(basis[1]),vp2s::music::broadcast<1>(translation)));
        const auto original=_mm_blend_ps(PS2_VADD(acc,PS2_VMUL(_mm_loadu_ps(basis[2]),vp2s::music::broadcast<2>(translation))),translation,8);
        const auto result=vp2s::math::transposedRigidInverseTranslation(matrix,translation);
        differences+=std::memcmp(&result,&original,16)!=0;
        if(captured)capturedDifferences+=std::memcmp(captured,&original,16)!=0;
        _mm_setcsr(saved);++cases;
    };
    auto testLinear=[&](const vp2s::math::RigidBasis &basis,__m128 vector,uint32_t mode,const __m128 *captured) {
        const auto saved=_mm_getcsr();_mm_setcsr((saved&~0xe040u)|mode);
        auto acc=PS2_VMUL(basis[0],vp2s::music::broadcast<0>(vector));
        acc=PS2_VADD(acc,PS2_VMUL(basis[1],vp2s::music::broadcast<1>(vector)));
        const auto original=PS2_VADD(acc,PS2_VMUL(basis[2],vp2s::music::broadcast<2>(vector)));
        const auto result=vp2s::math::linearVectorProduct(basis,vector);
        differences+=std::memcmp(&result,&original,16)!=0;
        if(captured)capturedDifferences+=std::memcmp(captured,&original,16)!=0;
        _mm_setcsr(saved);++cases;
    };
    bool linearOnly=std::strcmp(argv[1],"--linear-vector-synthetic")==0;
    bool matrixVectorOnly=std::strcmp(argv[1],"--matrix-vector-synthetic")==0;
    auto testMatrixVector=[&](const Matrix &matrix,__m128 vector,uint32_t mode,const __m128 *captured) {
        const auto saved=_mm_getcsr();_mm_setcsr((saved&~0xe040u)|mode);
        auto acc=PS2_VMUL(matrix[0],vp2s::music::broadcast<0>(vector));
        acc=PS2_VADD(acc,PS2_VMUL(matrix[1],vp2s::music::broadcast<1>(vector)));
        acc=PS2_VADD(acc,PS2_VMUL(matrix[2],vp2s::music::broadcast<2>(vector)));
        const auto original=PS2_VADD(acc,PS2_VMUL(matrix[3],vp2s::music::broadcast<3>(vector)));
        const auto result=vp2s::math::matrixVectorProduct(matrix,vector);
        differences+=std::memcmp(&result,&original,16)!=0;
        if(captured)capturedDifferences+=std::memcmp(captured,&original,16)!=0;
        _mm_setcsr(saved);++cases;
    };
    const bool matrixBlocks=std::strcmp(argv[1],"--matrix-vector-blocks")==0;
    if(std::strcmp(argv[1],"--linear-vector-blocks")==0 || matrixBlocks) {
        if(vp2sLinearVectorReferenceEnabled() || vp2sMatrixVectorReferenceEnabled())return 2;
        using namespace vp2s::mathfixture;
        const std::array<PS2Runtime::RecompiledFunction,4> originals=matrixBlocks?
            std::array<PS2Runtime::RecompiledFunction,4>{originalMatrixVector0,originalMatrixVector1,originalMatrixVector2,nullptr}:
            std::array<PS2Runtime::RecompiledFunction,4>{originalLinear0,originalLinear1,originalLinear2,originalLinear3};
        const std::array<PS2Runtime::RecompiledFunction,4> natives=matrixBlocks?
            std::array<PS2Runtime::RecompiledFunction,4>{nativeMatrixVector0,nativeMatrixVector1,nativeMatrixVector2,nullptr}:
            std::array<PS2Runtime::RecompiledFunction,4>{nativeLinear0,nativeLinear1,nativeLinear2,nativeLinear3};
        std::mt19937 random(0x56503253u);
        std::uniform_real_distribution<float> value(-1000.f,1000.f);
        std::vector<uint8_t> originalRam(0x130000),nativeRam;
        for(unsigned mode:{0u,0x6000u,0x8040u,0xe040u})for(unsigned k=0;k<32;++k)
            for(unsigned site=0;site<(matrixBlocks?3u:4u);++site)for(unsigned alias=0;alias<4;++alias) {
                std::fill(originalRam.begin(),originalRam.end(),0xa5);
                Matrix basis;
                for(auto &row:basis)row=_mm_setr_ps(value(random),value(random),value(random),value(random));
                auto vector=_mm_setr_ps(value(random),value(random),value(random),0.f);
                if(matrixBlocks)vector=_mm_blend_ps(vector,_mm_set1_ps(value(random)),8);
                else {uint32_t payload=random();std::memcpy(reinterpret_cast<uint8_t*>(&vector)+12,&payload,4);}
                constexpr uint32_t basisAddress=0x110000,vectorAddress=0x110100;
                const auto destination=alias==0?0x120000u:alias==1?basisAddress:alias==2?basisAddress+16:vectorAddress;
                std::memcpy(originalRam.data()+basisAddress,basis.data(),matrixBlocks?64:48);
                std::memcpy(originalRam.data()+vectorAddress,&vector,16);nativeRam=originalRam;
                R5900Context initial{};
                initial.r[3]=_mm_cvtsi32_si128(destination);initial.r[4]=_mm_cvtsi32_si128(vectorAddress);initial.r[5]=_mm_cvtsi32_si128(basisAddress);
                std::memset(initial.vu0_vf,0xc5,sizeof(initial.vu0_vf));
                initial.vu0_acc=_mm_set1_ps(-123.f);initial.vu0_mac_flags=0x8765;initial.vu0_status=0x1234;
                auto original=initial,native=initial;
                const auto saved=_mm_getcsr();_mm_setcsr((saved&~0xe040u)|mode);
                originals[site](originalRam.data(),&original,nullptr);
                _mm_setcsr((saved&~0xe040u)|mode);
                natives[site](nativeRam.data(),&native,nullptr);_mm_setcsr(saved);
                differences+=originalRam!=nativeRam;
                differences+=std::memcmp(original.r,native.r,sizeof(original.r))!=0;
                differences+=std::memcmp(original.f,native.f,sizeof(original.f))!=0;
                differences+=original.pc!=native.pc || original.branch_pc!=native.branch_pc || original.in_delay_slot!=native.in_delay_slot;
                differences+=std::memcmp(initial.vu0_vf,native.vu0_vf,sizeof(initial.vu0_vf))!=0;
                differences+=std::memcmp(&initial.vu0_acc,&native.vu0_acc,sizeof(initial.vu0_acc))!=0;
                differences+=initial.vu0_mac_flags!=native.vu0_mac_flags || initial.vu0_status!=native.vu0_status;
                ++cases;
            }
        std::printf("original/native %s blocks: %llu cases, RAM/CPU/VU-preservation differences %llu\n",matrixBlocks?"matrix vector":"linear vector",cases,differences);
        return cases && !differences?0:1;
    }
    const bool matrixFunctions=std::strcmp(argv[1],"--matrix-vector-functions")==0;
    if(std::strcmp(argv[1],"--linear-vector-functions")==0 || matrixFunctions) {
        if(vp2sLinearVectorReferenceEnabled() || vp2sMatrixVectorReferenceEnabled())return 2;
        using namespace vp2s::mathfixture;
        const auto originals=std::array{originalObjectMatrix0,originalObjectMatrix1,originalObjectMatrix2};
        const auto natives=std::array{nativeObjectMatrix0,nativeObjectMatrix1,nativeObjectMatrix2};
        std::mt19937 random(0x56503253u);
        std::uniform_real_distribution<float> value(-1000.f,1000.f);
        std::vector<uint8_t> originalRam(0x400000),nativeRam;
        constexpr uint32_t object=0x110000,stack=0x1f0000,returnPc=0x100000;
        for(unsigned mode:{0u,0x6000u,0x8040u,0xe040u}) for(unsigned k=0;k<128;++k)
            for(unsigned site=0;site<(matrixFunctions?3u:1u);++site)for(bool skip:{false,true}) {
            if(matrixFunctions && site==0 && skip)continue; // entered after its helper, beyond the flag branch
            std::fill(originalRam.begin(),originalRam.end(),0xa5);
            Matrix basis;
            for(auto &row:basis)row=_mm_setr_ps(value(random),value(random),value(random),value(random));
            std::memcpy(originalRam.data()+object+0x90,basis.data(),64);
            std::memcpy(originalRam.data()+object+0x150,basis.data(),64);
            auto point=_mm_setr_ps(value(random),value(random),value(random),value(random));
            std::memcpy(originalRam.data()+0x359110,&point,16);
            auto vector=_mm_setr_ps(value(random),value(random),value(random),0.f);
            uint32_t payload=random();std::memcpy(reinterpret_cast<uint8_t*>(&vector)+12,&payload,4);
            std::memcpy(originalRam.data()+0x359120,&vector,16);
            originalRam[object+0x20e]=skip?8:16;
            uint64_t returned=returnPc;std::memcpy(originalRam.data()+stack+16,&returned,8);
            nativeRam=originalRam;
            R5900Context initial{};
            for(unsigned reg=1;reg<32;++reg)initial.r[reg]=_mm_set1_epi32(int(0x12120000u+reg));
            initial.r[16]=_mm_cvtsi32_si128(object);initial.r[29]=_mm_cvtsi32_si128(stack);
            const uint32_t entry=matrixFunctions?(site==0?0x1fb3e4u:site==1?0x1fb54cu:0x1fb7dcu):0x1fb25cu;
            initial.r[31]=_mm_cvtsi32_si128(entry);initial.pc=entry;
            std::memset(initial.vu0_vf,0xc5,sizeof(initial.vu0_vf));
            initial.vu0_vf[0]=_mm_setr_ps(0.f,0.f,0.f,1.f);
            initial.vu0_acc=_mm_set1_ps(-123.f);initial.vu0_mac_flags=0x8765;initial.vu0_status=0x1234;
            auto original=initial,native=initial;
            const auto saved=_mm_getcsr();_mm_setcsr((saved&~0xe040u)|mode);
            (matrixFunctions?originals[site]:originalObjectLinear)(originalRam.data(),&original,nullptr);
            _mm_setcsr((saved&~0xe040u)|mode);
            (matrixFunctions?natives[site]:nativeObjectLinear)(nativeRam.data(),&native,nullptr);
            _mm_setcsr(saved);
            differences+=originalRam!=nativeRam;
            differences+=std::memcmp(original.r,native.r,sizeof(original.r))!=0;
            differences+=std::memcmp(original.f,native.f,sizeof(original.f))!=0;
            differences+=original.pc!=native.pc || native.pc!=returnPc;
            differences+=original.branch_pc!=native.branch_pc || native.in_delay_slot;
            differences+=original.fcr31!=native.fcr31;
            differences+=std::memcmp(initial.vu0_vf,native.vu0_vf,sizeof(initial.vu0_vf))!=0;
            differences+=std::memcmp(&initial.vu0_acc,&native.vu0_acc,sizeof(initial.vu0_acc))!=0;
            differences+=initial.vu0_mac_flags!=native.vu0_mac_flags || initial.vu0_status!=native.vu0_status;
            ++cases;
        }
        std::printf("object %s callback continuations: %llu cases, RAM/CPU/VU-preservation differences %llu\n",matrixFunctions?"matrix vector":"linear vector",cases,differences);
        return cases && !differences?0:1;
    }
    bool transposedOnly=std::strcmp(argv[1],"--transposed-rigid-synthetic")==0;
    bool offsetOnly=std::strcmp(argv[1],"--offset-dot-synthetic")==0;
    bool projectionOnly=std::strcmp(argv[1],"--projection-product-synthetic")==0;
    const bool rigidColumn=std::strcmp(argv[1],"--rigid-column-synthetic")==0;
    bool rigidOnly=std::strcmp(argv[1],"--rigid-inverse-synthetic")==0 || rigidColumn;
    bool quaternionOnly=std::strcmp(argv[1],"--normalize-synthetic")==0;
    const bool quaternionZeros=std::strcmp(argv[1],"--quat-matrix-zeros")==0;
    const bool quaternionSmall=std::strcmp(argv[1],"--quat-matrix-small")==0;
    bool quaternionMatrix=std::strcmp(argv[1],"--quat-matrix-synthetic")==0 || quaternionZeros || quaternionSmall;
    const bool columnFunctions=std::strcmp(argv[1],"--rigid-column-functions")==0;
    const bool offsetFunctions=std::strcmp(argv[1],"--offset-dot-functions")==0;
    const bool transposedFunctions=std::strcmp(argv[1],"--transposed-rigid-functions")==0;
    if(std::strcmp(argv[1],"--projection-functions")==0 || columnFunctions || offsetFunctions || transposedFunctions) {
        if(vp2sProjectionProductReferenceEnabled() || vp2sRigidInverseReferenceEnabled() || vp2sOffsetDotReferenceEnabled()) return 2;
        std::mt19937 random(0x56503253u);
        std::uniform_real_distribution<float> value(-200.f,200.f);
        std::vector<uint8_t> originalRam(0x200000),nativeRam(0x200000);
        constexpr uint32_t input=0x110000,output=0x120000,stack=0x1f0000,returnPc=0x100000;
        for(unsigned mode:{0u,0x6000u,0x8040u,0xe040u}) for(unsigned k=0;k<128;++k) {
            const auto q=vp2s::math::normalizeQuaternion(_mm_setr_ps(value(random),value(random),value(random),value(random)));
            auto view=vp2s::math::quaternionMatrix(q);
            if(columnFunctions) view[3]=_mm_setr_ps(value(random),value(random),value(random),1.f);
            else for(unsigned row=0;row<3;++row) view[row]=_mm_blend_ps(view[row],_mm_set1_ps(value(random)),8);
            for(bool orthographic:{false,true}) for(unsigned aliasMode=0;aliasMode<(transposedFunctions?4u:2u);++aliasMode) {
                const bool alias=aliasMode!=0;
                if((columnFunctions || offsetFunctions || transposedFunctions) && orthographic)continue;
                std::fill(originalRam.begin(),originalRam.end(),0xa5);
                std::memcpy(originalRam.data()+input,view.data(),64);nativeRam=originalRam;
                const uint32_t pointAddress=alias?stack-16:input;
                if(offsetFunctions) {
                    for(uint32_t address:{pointAddress,input+0x100u,input+0x200u}) {
                        auto v=_mm_setr_ps(value(random),value(random),value(random),0.f);
                        uint32_t bits=random();std::memcpy(reinterpret_cast<uint8_t*>(&v)+12,&bits,4);
                        std::memcpy(originalRam.data()+address,&v,16);
                    }
                    nativeRam=originalRam;
                }
                R5900Context initial{};
                for(unsigned reg=1;reg<32;++reg)initial.r[reg]=_mm_set1_epi32(int(0x12120000u+reg));
                initial.r[4]=_mm_cvtsi32_si128(columnFunctions?input:(alias?input:output));
                initial.r[5]=_mm_cvtsi32_si128(columnFunctions?(alias?input:output):input);
                if(transposedFunctions) {
                    initial.r[4]=_mm_cvtsi32_si128(input);
                    initial.r[5]=_mm_cvtsi32_si128(aliasMode==1?input:aliasMode==2?input+16:aliasMode==3?input-16:output);
                }
                if(offsetFunctions) {
                    initial.r[4]=_mm_cvtsi32_si128(pointAddress);
                    initial.r[5]=_mm_cvtsi32_si128(input+0x100u);initial.r[6]=_mm_cvtsi32_si128(input+0x200u);
                }
                initial.r[29]=_mm_cvtsi32_si128(stack);initial.r[31]=_mm_cvtsi32_si128(returnPc);
                for(unsigned reg=0;reg<32;++reg)initial.f[reg]=float(reg)+0.125f;
                initial.f[12]=320.f+float(k);initial.f[13]=640.f;initial.f[14]=448.f;
                initial.f[15]=2048.f+float(k);initial.f[16]=2048.f-float(k);
                std::memset(initial.vu0_vf,0xc5,sizeof(initial.vu0_vf));
                initial.vu0_vf[0]=_mm_setr_ps(0.f,0.f,0.f,1.f);
                initial.vu0_acc=_mm_set1_ps(-123.f);initial.vu0_mac_flags=0x8765;initial.vu0_status=0x1234;
                R5900Context original=initial,native=initial;
                const auto saved=_mm_getcsr();_mm_setcsr((saved&~0xe040u)|mode);
                (transposedFunctions?vp2s::mathfixture::originalTransposedRigid:offsetFunctions?vp2s::mathfixture::originalOffsetDot:columnFunctions?vp2s::mathfixture::originalColumnRigid:orthographic?vp2s::mathfixture::originalOrthographic:vp2s::mathfixture::originalPerspective)(originalRam.data(),&original,nullptr);
                _mm_setcsr((saved&~0xe040u)|mode);
                (transposedFunctions?vp2s::mathfixture::nativeTransposedRigid:offsetFunctions?vp2s::mathfixture::nativeOffsetDot:columnFunctions?vp2s::mathfixture::nativeColumnRigid:orthographic?vp2s::mathfixture::nativeOrthographic:vp2s::mathfixture::nativePerspective)(nativeRam.data(),&native,nullptr);
                _mm_setcsr(saved);
                differences+=std::memcmp(originalRam.data(),nativeRam.data(),originalRam.size())!=0;
                differences+=std::memcmp(original.r,native.r,sizeof(original.r))!=0;
                differences+=std::memcmp(original.f,native.f,sizeof(original.f))!=0;
                differences+=original.pc!=native.pc || native.pc!=returnPc;
                differences+=original.branch_pc!=native.branch_pc || native.in_delay_slot;
                differences+=original.fcr31!=native.fcr31;
                differences+=std::memcmp(initial.vu0_vf,native.vu0_vf,sizeof(initial.vu0_vf))!=0;
                differences+=std::memcmp(&initial.vu0_acc,&native.vu0_acc,sizeof(initial.vu0_acc))!=0;
                differences+=initial.vu0_mac_flags!=native.vu0_mac_flags || initial.vu0_status!=native.vu0_status;
                ++cases;
            }
        }
        std::printf("%s game functions: %llu cases, RAM/CPU/VU-preservation differences %llu\n",transposedFunctions?"transposed rigid inverse":offsetFunctions?"offset dot":columnFunctions?"column rigid inverse":"projection",cases,differences);
        return cases && !differences?0:1;
    } else if(matrixVectorOnly) {
        std::mt19937 random(0x56503253u);
        std::uniform_real_distribution<float> value(-1000.f,1000.f);
        for(unsigned mode:{0u,0x6000u,0x8040u,0xe040u}) {
            for(unsigned k=0;k<32768;++k) {
                auto sample=[&] {return k<16384?value(random):std::bit_cast<float>((random()&0x807fffffu)|((random()%128u)<<23));};
                Matrix matrix;
                for(auto &row:matrix)row=_mm_setr_ps(sample(),sample(),sample(),sample());
                const auto vector=_mm_setr_ps(sample(),sample(),sample(),sample());
                testMatrixVector(matrix,vector,mode,nullptr);
            }
            for(float scale:{0.f,-0.f,1.f,-1.f,1.e-20f,1.e18f})for(unsigned lane=0;lane<4;++lane)for(bool negative:{false,true}) {
                Matrix matrix={_mm_setr_ps(scale,0,0,scale),_mm_setr_ps(0,scale,0,-scale),_mm_setr_ps(0,0,scale,scale),_mm_setr_ps(scale,-scale,scale,-scale)};
                float vector[4]{};vector[lane]=negative?-scale:scale;
                testMatrixVector(matrix,_mm_loadu_ps(vector),mode,nullptr);
            }
        }
    } else if(linearOnly) {
        std::mt19937 random(0x56503253u);
        std::uniform_real_distribution<float> value(-1000.f,1000.f);
        for(unsigned mode:{0u,0x6000u,0x8040u,0xe040u}) {
            for(unsigned k=0;k<32768;++k) {
                auto sample=[&] {return k<16384?value(random):std::bit_cast<float>((random()&0x807fffffu)|((random()%128u)<<23));};
                vp2s::math::RigidBasis basis;
                for(auto &row:basis)row=_mm_setr_ps(sample(),sample(),sample(),sample());
                auto vector=_mm_setr_ps(sample(),sample(),sample(),0.f);
                uint32_t payload=random();std::memcpy(reinterpret_cast<uint8_t*>(&vector)+12,&payload,4);
                testLinear(basis,vector,mode,nullptr);
            }
            for(float scale:{0.f,-0.f,1.f,-1.f,1.e-20f,1.e18f})for(unsigned lane=0;lane<3;++lane)for(bool negative:{false,true}) {
                vp2s::math::RigidBasis basis={_mm_setr_ps(scale,0,0,scale),_mm_setr_ps(0,scale,0,-scale),_mm_setr_ps(0,0,scale,scale)};
                float vector[4]{};vector[lane]=negative?-scale:scale;
                testLinear(basis,_mm_loadu_ps(vector),mode,nullptr);
            }
        }
    } else if(transposedOnly) {
        std::mt19937 random(0x56503253u);
        std::uniform_real_distribution<float> value(-1000.f,1000.f);
        for(unsigned mode:{0u,0x6000u,0x8040u,0xe040u}) {
            for(unsigned k=0;k<32768;++k) {
                Matrix matrix;float t[4];
                auto sample=[&] {
                    return k<16384 ? value(random) : std::bit_cast<float>((random()&0x807fffffu)|((random()%128u)<<23));
                };
                for(auto &v:matrix)v=_mm_setr_ps(sample(),sample(),sample(),sample());
                for(float &v:t)v=sample();
                uint32_t payload=random();std::memcpy(&t[3],&payload,4);
                testTransposedRigid(matrix,_mm_loadu_ps(t),mode,nullptr);
            }
            for(float scale:{0.f,-0.f,1.f,-1.f,1.e-20f,1.e18f})
                for(unsigned lane=0;lane<3;++lane) for(bool negative:{false,true}) {
                    Matrix matrix={_mm_setr_ps(scale,0,0,0),_mm_setr_ps(0,scale,0,0),_mm_setr_ps(0,0,scale,0),_mm_setr_ps(0,0,0,1)};
                    float t[4]{};t[lane]=negative?-scale:scale;
                    testTransposedRigid(matrix,_mm_loadu_ps(t),mode,nullptr);
                }
        }
    } else if(offsetOnly) {
        std::mt19937 random(0x56503253u);
        std::uniform_real_distribution<float> value(-1000.f,1000.f);
        for(unsigned mode:{0u,0x6000u,0x8040u,0xe040u}) {
            for(unsigned k=0;k<32768;++k) {
                __m128 vectors[3];
                auto sample=[&] {
                    return k<16384 ? value(random) : std::bit_cast<float>((random()&0x807fffffu)|((random()%128u)<<23));
                };
                for(auto &v:vectors) {
                    v=_mm_setr_ps(sample(),sample(),sample(),0.f);
                    uint32_t payload=random();std::memcpy(reinterpret_cast<uint8_t*>(&v)+12,&payload,4);
                }
                testOffsetDot(vectors[0],vectors[1],vectors[2],mode,nullptr);
            }
            for(float scale:{0.f,-0.f,1.f,-1.f,1.e-20f,1.e18f})
                for(unsigned lane=0;lane<3;++lane) for(bool negative:{false,true}) {
                    float p[4]{},axis[4]{};p[lane]=scale;axis[lane]=negative?-scale:scale;
                    testOffsetDot(_mm_loadu_ps(p),_mm_loadu_ps(axis),_mm_setzero_ps(),mode,nullptr);
                }
        }
    } else if(projectionOnly) {
        std::mt19937 random(0x56503253u);
        std::uniform_real_distribution<float> value(-1000.f,1000.f);
        for(unsigned mode:{0u,0x6000u,0x8040u,0xe040u}) {
            for(unsigned k=0;k<32768;++k) {
                Matrix local,view;
                auto sample=[&] {
                    return k<16384 ? value(random) : std::bit_cast<float>((random()&0x807fffffu)|((random()%128u)<<23));
                };
                for(auto *matrix:{&local,&view}) for(auto &column:*matrix)
                    column=_mm_setr_ps(sample(),sample(),sample(),sample());
                testProjection(local,view,mode,nullptr);
            }
            for(unsigned k=0;k<64;++k) {
                Matrix view;
                for(auto &column:view)column=_mm_setr_ps(value(random),value(random),value(random),value(random));
                Matrix perspective={_mm_setr_ps(value(random),0,0,value(random)),_mm_setr_ps(0,value(random),0,value(random)),
                                    _mm_setr_ps(0,0,0,1),_mm_setr_ps(0,0,1,0)};
                Matrix orthographic={_mm_setr_ps(value(random),0,value(random),0),_mm_setr_ps(0,value(random),value(random),0),
                                     _mm_setr_ps(0,0,1,0),_mm_setr_ps(0,0,0,1)};
                testProjection(perspective,view,mode,nullptr);testProjection(orthographic,view,mode,nullptr);
            }
        }
    } else if(rigidOnly) {
        std::mt19937 random(0x56503253u);
        std::uniform_real_distribution<float> value(-1000.f,1000.f);
        for(unsigned mode:{0u,0x6000u,0x8040u,0xe040u}) {
            for(unsigned k=0;k<32768;++k) {
                vp2s::math::RigidBasis basis;float t[4];
                auto sample=[&] {
                    return k<16384 ? value(random) : std::bit_cast<float>((random()&0x807fffffu)|((random()%128u)<<23));
                };
                for(auto &row:basis) row=_mm_setr_ps(sample(),sample(),sample(),sample());
                for(float &v:t)v=sample();
                uint32_t payload=random();std::memcpy(&t[3],&payload,4);
                testRigid(basis,_mm_loadu_ps(t),mode,nullptr,rigidColumn);
            }
            for(float scale:{0.f,-0.f,1.f,-1.f,1.e-20f,1.e18f})
                for(unsigned lane=0;lane<3;++lane) for(bool negative:{false,true}) {
                    vp2s::math::RigidBasis basis={_mm_setr_ps(scale,0,0,0),_mm_setr_ps(0,scale,0,0),_mm_setr_ps(0,0,scale,0)};
                    float t[4]{};t[lane]=negative?-scale:scale;
                    testRigid(basis,_mm_loadu_ps(t),mode,nullptr,rigidColumn);
                }
        }
    } else if(quaternionSmall) {
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
            if(tag==0x3152564du) {
                uint32_t mode;Matrix matrix;__m128 vector,result;
                if(std::fread(&mode,4,1,file)!=1 || (mode&~0xe040u) || std::fread(matrix.data(),1,64,file)!=64 ||
                   std::fread(&vector,1,16,file)!=16 || std::fread(&result,1,16,file)!=16) {std::fclose(file);return 2;}
                testMatrixVector(matrix,vector,mode,&result);matrixVectorOnly=true;continue;
            }
            if(tag==0x3152564cu) {
                uint32_t mode;vp2s::math::RigidBasis basis;__m128 vector,result;
                if(std::fread(&mode,4,1,file)!=1 || (mode&~0xe040u) || std::fread(basis.data(),1,48,file)!=48 ||
                   std::fread(&vector,1,16,file)!=16 || std::fread(&result,1,16,file)!=16) {std::fclose(file);return 2;}
                testLinear(basis,vector,mode,&result);linearOnly=true;continue;
            }
            if(tag==0x31524954u) {
                uint32_t mode;Matrix matrix;__m128 translation,result;
                if(std::fread(&mode,4,1,file)!=1 || (mode&~0xe040u) || std::fread(matrix.data(),1,64,file)!=64 ||
                   std::fread(&translation,1,16,file)!=16 || std::fread(&result,1,16,file)!=16) {std::fclose(file);return 2;}
                testTransposedRigid(matrix,translation,mode,&result);transposedOnly=true;continue;
            }
            if(tag==0x31523344u) {
                uint32_t mode;__m128 point,axis,origin,result;
                if(std::fread(&mode,4,1,file)!=1 || (mode&~0xe040u) || std::fread(&point,1,16,file)!=16 ||
                   std::fread(&axis,1,16,file)!=16 || std::fread(&origin,1,16,file)!=16 || std::fread(&result,1,16,file)!=16) {std::fclose(file);return 2;}
                testOffsetDot(point,axis,origin,mode,&result);offsetOnly=true;continue;
            }
            if(tag==0x31523450u) {
                uint32_t mode;Matrix local,view,result;
                if(std::fread(&mode,4,1,file)!=1 || (mode&~0xe040u) || std::fread(local.data(),1,64,file)!=64 ||
                   std::fread(view.data(),1,64,file)!=64 || std::fread(result.data(),1,64,file)!=64) {std::fclose(file);return 2;}
                testProjection(local,view,mode,&result);projectionOnly=true;continue;
            }
            if(tag==0x31524952u || tag==0x314e4952u) {
                uint32_t mode;vp2s::math::RigidBasis basis;__m128 translation,result;
                if(std::fread(&mode,4,1,file)!=1 || (mode&~0xe040u) || std::fread(basis.data(),1,48,file)!=48 ||
                   std::fread(&translation,1,16,file)!=16 || std::fread(&result,1,16,file)!=16) {std::fclose(file);return 2;}
                testRigid(basis,translation,mode,&result,tag==0x314e4952u);rigidOnly=true;continue;
            }
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
                matrixVectorOnly?"matrix vector products":linearOnly?"linear vector products":transposedOnly?"transposed rigid inverses":offsetOnly?"offset dot records":projectionOnly?"projection products":rigidOnly?"rigid inverse translations":quaternionMatrix?"quaternion matrices":quaternionOnly?"quaternions":"matrices",
                cases,cases*((matrixVectorOnly || linearOnly || quaternionOnly || rigidOnly || offsetOnly || transposedOnly)?4:16),differences,capturedDifferences);
    return cases && !differences && !capturedDifferences?0:1;
}

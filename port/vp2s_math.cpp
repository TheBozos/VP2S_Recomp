// Shared game matrix operations on EE records, with no VU state or transport.
#include "vp2s_math.h"
#include "ps2_runtime.h"
#include "ps2_overlay_modules.h"
#include "runtime/ps2_memory.h"
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>
#include <algorithm>

namespace {
using vp2s::math::Matrix;
constexpr uint32_t kProduct=0x00217550u;
constexpr uint32_t kQuaternionMatrix=0x00216730u;
PS2Runtime::RecompiledFunction g_original=nullptr;
PS2Runtime::RecompiledFunction g_originalQuaternionMatrix=nullptr;
uint8_t *g_workingMemory=nullptr;
uint8_t *recordPointer(uint8_t *ram,uint32_t address,size_t size=sizeof(Matrix)) {
    // Some game routines temporarily put their EE stack in the 16 KiB working
    // area, as the existing native model path also observes. Access only the
    // record's bytes: no DMA operation, transfer packet or VU memory is involved.
    if(address>=0x70000000u && address<=0x70004000u-size && !(address&15u) && g_workingMemory)
        return g_workingMemory+(address-0x70000000u);
    if((address&0xF0000000u)!=0x70000000u) {
        const uint32_t offset=address&0x1FFFFFFu;
        if(offset>=0x100000u && offset<=0x2000000u-size && !(offset&15u)) return ram+offset;
    }
    char detail[100];
    std::snprintf(detail,sizeof(detail),"invalid native math record address=%x size=%zu",address,size);
    throw std::runtime_error(detail);
}
struct Capture {
    std::string path;
    std::vector<uint8_t> bytes;
    std::thread writer;
    unsigned saved=0;
    unsigned limit=4096;
    Capture(const char *environment,size_t recordSize) {
        if(const char *v=std::getenv("VP2S_MATH_CAPTURE_LIMIT")) {
            limit=unsigned(std::strtoul(v,nullptr,10));
            if(!limit || limit>4096) throw std::runtime_error("invalid native math capture limit");
        }
        if(const char *p=std::getenv(environment)) {
            path=p; bytes.reserve(limit*recordSize);
        }
    }
    static void save(const std::string &path,const std::vector<uint8_t> &bytes) {
        FILE *file=std::fopen(path.c_str(),"wb");
        if(!file) { std::fprintf(stderr,"[math:native] cannot open matrix capture\n"); return; }
        const bool ok=std::fwrite(bytes.data(),1,bytes.size(),file)==bytes.size();
        std::fclose(file);
        if(!ok) std::fprintf(stderr,"[math:native] incomplete matrix capture\n");
    }
    ~Capture() {
        if(writer.joinable()) writer.join();
        else if(!path.empty() && !bytes.empty()) save(path,bytes);
    }
    void append(const void *data,size_t size) {
        const auto *p=static_cast<const uint8_t*>(data);
        bytes.insert(bytes.end(),p,p+size);
    }
    bool begin(uint32_t tag) {
        if(path.empty() || saved==limit) return false;
        append(&tag,4);
        return true;
    }
    void end() {
        if(++saved==limit) writer=std::thread([p=path,data=std::move(bytes)] {save(p,data);});
    }
};
Capture &capture() { static Capture value("VP2S_MATRIX_RECORD",4u+3u*sizeof(Matrix)); return value; }
Capture &quaternionCapture() { static Capture value("VP2S_NORMALIZE_RECORD",36u); return value; }
Capture &quaternionMatrixCapture() { static Capture value("VP2S_QUAT_MATRIX_RECORD",84u); return value; }
Capture &inlineQuaternionCapture() { static Capture value("VP2S_INLINE_QUAT_MATRIX_RECORD",84u); return value; }
Capture &rigidInverseCapture() { static Capture value("VP2S_RIGID_INVERSE_RECORD",88u); return value; }
Capture &rigidColumnCapture() { static Capture value("VP2S_RIGID_INVERSE_COLUMN_RECORD",88u); return value; }
Capture &projectionCapture() { static Capture value("VP2S_PROJECTION_PRODUCT_RECORD",200u); return value; }
Capture &offsetDotCapture() { static Capture value("VP2S_OFFSET_DOT_RECORD",72u); return value; }
Capture &transposedRigidCapture() { static Capture value("VP2S_TRANSPOSED_RIGID_RECORD",104u); return value; }
Capture &linearVectorCapture() { static Capture value("VP2S_LINEAR_VECTOR_RECORD",88u); return value; }
Capture &matrixVectorCapture() { static Capture value("VP2S_MATRIX_VECTOR_RECORD",104u); return value; }
Capture &pointDirectionCapture() { static Capture value("VP2S_POINT_DIRECTION_RECORD",136u); return value; }
void quaternionMatrix(uint8_t *ram,R5900Context *ctx,PS2Runtime *runtime) {
    const uint32_t destination=getRegU32(ctx,4),stack=getRegU32(ctx,29);
    const __m128 input=_mm_loadu_ps(reinterpret_cast<const float*>(recordPointer(ram,getRegU32(ctx,5),16)));
    const auto result=vp2s::math::quaternionMatrix(input);
    static const bool check=[] {const char *v=std::getenv("VP2S_QUAT_MATRIX_CHECK");return v && v[0]=='1';}();
    if(check) {
        g_originalQuaternionMatrix(ram,ctx,runtime);
        if(ctx->pc!=getRegU32(ctx,31) || std::memcmp(recordPointer(ram,destination),&result,sizeof(result)))
            throw std::runtime_error("native quaternion matrix differs from original game helper");
    }
    // Keep the EE helper's temporary record and GPR clobbers. Its VU registers
    // are not an input/output API and are untouched by the default native path.
    std::memcpy(recordPointer(ram,stack-64u),&result,sizeof(result));
    std::memcpy(recordPointer(ram,destination),&result,sizeof(result));
    ctx->r[3]=_mm_castps_si128(result[3]);
    ctx->r[9]=_mm_insert_epi64(ctx->r[9],0,0);
    ctx->pc=getRegU32(ctx,31);
    auto &recorder=quaternionMatrixCapture();
    if(recorder.begin(0x31524d51u)) {
        recorder.append(&input,16); recorder.append(&result,64); recorder.end();
    }
    static uint64_t calls=0;
    if(++calls==1 || calls%4096==0)
        std::fprintf(stderr,"[math:native] quaternion matrix %llu calls%s\n",calls,check?" (original comparison exact)":" (no VU execution)");
}
void product(uint8_t *ram,R5900Context *ctx,PS2Runtime *runtime) {
    const uint32_t destination=getRegU32(ctx,4), stack=getRegU32(ctx,29);
    Matrix local,parent;
    std::memcpy(&local,recordPointer(ram,getRegU32(ctx,5)),sizeof(local));
    std::memcpy(&parent,recordPointer(ram,getRegU32(ctx,6)),sizeof(parent));
    const auto result=vp2s::math::compose(local,parent);
    static const bool check=[] {const char *v=std::getenv("VP2S_MATRIX_CHECK");return v && v[0]=='1';}();
    if(check) {
        // Explicit diagnostic comparison only. The default never executes the
        // original wrapper or reads its VU register/data state.
        g_original(ram,ctx,runtime);
        if(ctx->pc!=getRegU32(ctx,31) || std::memcmp(recordPointer(ram,destination),&result,sizeof(result)))
            throw std::runtime_error("native matrix product differs from original game helper");
    }
    // Preserve the wrapper's ordinary EE ABI and temporary stack record. Load
    // all inputs before any write, including in-place output/input aliases.
    std::memcpy(recordPointer(ram,stack-64u),&result,sizeof(result));
    std::memcpy(recordPointer(ram,destination),&result,sizeof(result));
    ctx->r[3]=_mm_castps_si128(result[3]);
    ctx->r[9]=_mm_insert_epi64(ctx->r[9],0,0);
    ctx->pc=getRegU32(ctx,31);
    auto &recorder=capture();
    if(recorder.begin(0x3152344du)) {
        recorder.append(&local,sizeof(local)); recorder.append(&parent,sizeof(parent)); recorder.append(&result,sizeof(result));
        recorder.end();
    }
    static uint64_t calls=0;
    if(++calls==1 || calls%4096==0)
        std::fprintf(stderr,"[math:native] matrix product %llu calls%s\n",calls,check?" (original comparison exact)":" (no VU execution)");
}
}
bool vp2sInlineQuaternionEnabled() {
    static const bool enabled=[] {const char *v=std::getenv("VP2S_NATIVE_INLINE_QUAT_MATRIX");return !(v && v[0]=='0');}();
    return enabled;
}
bool vp2sInlineQuaternionReferenceEnabled() {
    static const bool check=[] {const char *v=std::getenv("VP2S_INLINE_QUAT_MATRIX_CHECK");return v && v[0]=='1';}();
    return !vp2sInlineQuaternionEnabled() || check;
}
void vp2sInlineQuaternionMatrix(uint8_t *ram,R5900Context *ctx,uint32_t source,uint32_t destination,unsigned columns,uint32_t site) {
    if(columns!=3 && columns!=4) throw std::runtime_error("invalid native quaternion matrix column count");
    const __m128 input=_mm_loadu_ps(reinterpret_cast<const float*>(recordPointer(ram,source,16)));
    const auto result=vp2s::math::quaternionMatrix(input);
    if(vp2sInlineQuaternionReferenceEnabled() && std::memcmp(result.data(),&ctx->vu0_vf[8],64)) {
        if(const char *path=std::getenv("VP2S_INLINE_QUAT_FAILURE_RECORD"))
            if(FILE *file=std::fopen(path,"wb")) {
                const uint32_t tag=0x31524d51u;
                std::fwrite(&tag,4,1,file);std::fwrite(&input,16,1,file);
                std::fwrite(&ctx->vu0_vf[8],64,1,file);std::fclose(file);
            }
        char detail[160];
        std::snprintf(detail,sizeof(detail),"native inline quaternion matrix differs: site=%x source=%x destination=%x",site,source,destination);
        throw std::runtime_error(detail);
    }
    std::memcpy(recordPointer(ram,destination,columns*16),result.data(),columns*16);
    auto &recorder=inlineQuaternionCapture();
    if(recorder.begin(0x31524d51u)) {
        recorder.append(&input,16);recorder.append(result.data(),64);recorder.end();
    }
    static std::array<uint32_t,56> observed{};
    static unsigned distinct=0;
    if(std::find(observed.begin(),observed.begin()+distinct,site)==observed.begin()+distinct) {
        if(distinct==observed.size()) throw std::runtime_error("unexpected native inline quaternion site");
        observed[distinct++]=site;
        std::fprintf(stderr,"[math:native] inline quaternion site %x, %u columns%s\n",site,columns,
                     vp2sInlineQuaternionReferenceEnabled()?" (original comparison exact)":" (no VU execution)");
    }
    static uint64_t calls=0;
    if(++calls%4096==0) std::fprintf(stderr,"[math:native] inline quaternion matrices %llu calls, %u sites\n",calls,distinct);
}
void vp2sInlineQuaternionCompose(uint8_t *ram,R5900Context *ctx,uint32_t source,uint32_t destination) {
    Matrix attachment,rotation;
    std::memcpy(attachment.data(),recordPointer(ram,source),64);
    std::memcpy(rotation.data(),recordPointer(ram,destination),64);
    const auto result=vp2s::math::composeAttachment(attachment,rotation);
    if(vp2sInlineQuaternionReferenceEnabled() && std::memcmp(result.data(),&ctx->vu0_vf[1],64))
        throw std::runtime_error("native quaternion attachment differs from original game calculation");
    std::memcpy(recordPointer(ram,destination),result.data(),64);
}
bool vp2sNormalizeEnabled() {
    static const bool enabled=[] {const char *v=std::getenv("VP2S_NATIVE_NORMALIZE");return !(v && v[0]=='0');}();
    return enabled;
}
bool vp2sOffsetDotEnabled() {
    static const bool enabled=[] {const char *v=std::getenv("VP2S_NATIVE_OFFSET_DOT");return !(v && v[0]=='0');}();
    return enabled;
}
bool vp2sOffsetDotReferenceEnabled() {
    static const bool check=[] {const char *v=std::getenv("VP2S_OFFSET_DOT_CHECK");return v && v[0]=='1';}();
    return !vp2sOffsetDotEnabled() || check;
}
void vp2sOffsetDotRecord(uint8_t *ram,R5900Context *ctx,uint32_t pointAddress,uint32_t axisAddress,uint32_t originAddress,uint32_t destination) {
    __m128 point,axis,origin;
    std::memcpy(&point,recordPointer(ram,pointAddress,16),16);
    std::memcpy(&axis,recordPointer(ram,axisAddress,16),16);
    std::memcpy(&origin,recordPointer(ram,originAddress,16),16);
    const auto result=vp2s::math::offsetDotRecord(point,axis,origin);
    const uint32_t rounding=_mm_getcsr()&0xe040u;
    if(vp2sOffsetDotReferenceEnabled() && std::memcmp(&result,&ctx->vu0_vf[16],16)) {
        if(const char *path=std::getenv("VP2S_OFFSET_DOT_FAILURE_RECORD"))
            if(FILE *file=std::fopen(path,"wb")) {
                const uint32_t tag=0x31523344u;
                std::fwrite(&tag,4,1,file);std::fwrite(&rounding,4,1,file);
                std::fwrite(&point,16,1,file);std::fwrite(&axis,16,1,file);std::fwrite(&origin,16,1,file);
                std::fwrite(&ctx->vu0_vf[16],16,1,file);std::fclose(file);
            }
        throw std::runtime_error("native offset dot differs from original game calculation");
    }
    std::memcpy(recordPointer(ram,destination,16),&result,16);
    auto &recorder=offsetDotCapture();
    if(recorder.begin(0x31523344u)) {
        recorder.append(&rounding,4);recorder.append(&point,16);recorder.append(&axis,16);recorder.append(&origin,16);recorder.append(&result,16);recorder.end();
    }
    static uint64_t calls=0;
    if(++calls==1 || calls%4096==0)
        std::fprintf(stderr,"[math:native] offset dot %llu calls%s\n",calls,
                     vp2sOffsetDotReferenceEnabled()?" (original comparison exact)":" (no VU state)");
}
bool vp2sProjectionProductEnabled() {
    static const bool enabled=[] {const char *v=std::getenv("VP2S_NATIVE_PROJECTION_PRODUCT");return !(v && v[0]=='0');}();
    return enabled;
}
bool vp2sProjectionProductReferenceEnabled() {
    static const bool check=[] {const char *v=std::getenv("VP2S_PROJECTION_PRODUCT_CHECK");return v && v[0]=='1';}();
    return !vp2sProjectionProductEnabled() || check;
}
void vp2sProjectionProduct(uint8_t *ram,R5900Context *ctx,uint32_t localAddress,uint32_t viewAddress,uint32_t destination,uint32_t site) {
    Matrix local,view;
    std::memcpy(local.data(),recordPointer(ram,localAddress),64);
    std::memcpy(view.data(),recordPointer(ram,viewAddress),64);
    const auto result=vp2s::math::projectionProduct(local,view);
    const uint32_t rounding=_mm_getcsr()&0xe040u;
    if(vp2sProjectionProductReferenceEnabled() && std::memcmp(result.data(),&ctx->vu0_vf[1],64)) {
        if(const char *path=std::getenv("VP2S_PROJECTION_PRODUCT_FAILURE_RECORD"))
            if(FILE *file=std::fopen(path,"wb")) {
                const uint32_t tag=0x31523450u;
                std::fwrite(&tag,4,1,file);std::fwrite(&rounding,4,1,file);
                std::fwrite(local.data(),64,1,file);std::fwrite(view.data(),64,1,file);
                std::fwrite(&ctx->vu0_vf[1],64,1,file);std::fclose(file);
            }
        throw std::runtime_error("native projection product differs from original game calculation");
    }
    std::memcpy(recordPointer(ram,destination),result.data(),64);
    auto &recorder=projectionCapture();
    if(recorder.begin(0x31523450u)) {
        recorder.append(&rounding,4);recorder.append(local.data(),64);recorder.append(view.data(),64);
        recorder.append(result.data(),64);recorder.end();
    }
    const unsigned index=site==0x2179a8u?0:site==0x217ad0u?1:2;
    if(index==2) throw std::runtime_error("unexpected native projection product site");
    static uint64_t calls[2]{};
    if(++calls[index]==1 || calls[index]%4096==0)
        std::fprintf(stderr,"[math:native] projection product %x: %llu calls%s\n",site,calls[index],
                     vp2sProjectionProductReferenceEnabled()?" (original comparison exact)":" (no VU state)");
}
bool vp2sRigidInverseEnabled() {
    static const bool enabled=[] {const char *v=std::getenv("VP2S_NATIVE_RIGID_INVERSE");return !(v && v[0]=='0');}();
    return enabled;
}
bool vp2sLinearVectorEnabled() {
    static const bool enabled=[] {const char *v=std::getenv("VP2S_NATIVE_LINEAR_VECTOR");return !(v && v[0]=='0');}();
    return enabled;
}
bool vp2sMatrixVectorEnabled() {
    static const bool enabled=[] {const char *v=std::getenv("VP2S_NATIVE_MATRIX_VECTOR");return !(v && v[0]=='0');}();
    return enabled;
}
bool vp2sPointDirectionEnabled() {
    static const bool enabled=[] {const char *v=std::getenv("VP2S_NATIVE_POINT_DIRECTION");return !(v && v[0]=='0');}();
    return enabled;
}
bool vp2sPointDirectionReferenceEnabled() {
    static const bool check=[] {const char *v=std::getenv("VP2S_POINT_DIRECTION_CHECK");return v && v[0]=='1';}();
    return !vp2sPointDirectionEnabled() || check;
}
void vp2sTransformPointDirection(uint8_t *ram,R5900Context *ctx,uint32_t matrixAddress,uint32_t pointAddress,uint32_t directionAddress) {
    Matrix matrix;vp2s::math::PointDirection vectors;
    std::memcpy(matrix.data(),recordPointer(ram,matrixAddress),64);
    std::memcpy(&vectors[0],recordPointer(ram,pointAddress,16),16);
    std::memcpy(&vectors[1],recordPointer(ram,directionAddress,16),16);
    const auto result=vp2s::math::transformPointDirection(matrix,vectors);
    const uint32_t rounding=_mm_getcsr()&0xe040u;
    if(vp2sPointDirectionReferenceEnabled() && std::memcmp(result.data(),&ctx->vu0_vf[5],32)) {
        if(const char *path=std::getenv("VP2S_POINT_DIRECTION_FAILURE_RECORD"))
            if(FILE *file=std::fopen(path,"wb")) {
                const uint32_t tag=0x31524450u;
                std::fwrite(&tag,4,1,file);std::fwrite(&rounding,4,1,file);std::fwrite(matrix.data(),64,1,file);
                std::fwrite(vectors.data(),32,1,file);std::fwrite(&ctx->vu0_vf[5],32,1,file);std::fclose(file);
            }
        throw std::runtime_error("native point direction transform differs from original game calculation");
    }
    // Both results must be computed before either in-place vector is changed.
    std::memcpy(recordPointer(ram,pointAddress,16),&result[0],16);
    std::memcpy(recordPointer(ram,directionAddress,16),&result[1],16);
    auto &recorder=pointDirectionCapture();
    if(recorder.begin(0x31524450u)) {
        recorder.append(&rounding,4);recorder.append(matrix.data(),64);recorder.append(vectors.data(),32);recorder.append(result.data(),32);recorder.end();
    }
    static uint64_t calls=0;
    if(++calls==1 || calls%4096==0)
        std::fprintf(stderr,"[math:native] point direction %llu calls%s\n",calls,
                     vp2sPointDirectionReferenceEnabled()?" (original comparison exact)":" (no VU state)");
}
bool vp2sMatrixVectorReferenceEnabled() {
    static const bool check=[] {const char *v=std::getenv("VP2S_MATRIX_VECTOR_CHECK");return v && v[0]=='1';}();
    return !vp2sMatrixVectorEnabled() || check;
}
void vp2sMatrixVectorProduct(uint8_t *ram,R5900Context *ctx,uint32_t matrixAddress,uint32_t vectorAddress,uint32_t destination,uint32_t site) {
    Matrix matrix;__m128 vector;
    std::memcpy(matrix.data(),recordPointer(ram,matrixAddress),64);
    std::memcpy(&vector,recordPointer(ram,vectorAddress,16),16);
    const auto result=vp2s::math::matrixVectorProduct(matrix,vector);
    const uint32_t rounding=_mm_getcsr()&0xe040u;
    if(vp2sMatrixVectorReferenceEnabled() && std::memcmp(&result,&ctx->vu0_vf[5],16)) {
        if(const char *path=std::getenv("VP2S_MATRIX_VECTOR_FAILURE_RECORD"))
            if(FILE *file=std::fopen(path,"wb")) {
                const uint32_t tag=0x3152564du;
                std::fwrite(&tag,4,1,file);std::fwrite(&rounding,4,1,file);std::fwrite(matrix.data(),64,1,file);
                std::fwrite(&vector,16,1,file);std::fwrite(&ctx->vu0_vf[5],16,1,file);std::fclose(file);
            }
        throw std::runtime_error("native matrix vector product differs from original game calculation");
    }
    std::memcpy(recordPointer(ram,destination,16),&result,16);
    auto &recorder=matrixVectorCapture();
    if(recorder.begin(0x3152564du)) {
        recorder.append(&rounding,4);recorder.append(matrix.data(),64);recorder.append(&vector,16);recorder.append(&result,16);recorder.end();
    }
    const unsigned index=site==0x1fb418u?0:site==0x1fb610u?1:site==0x1fb8a0u?2:3;
    if(index==3) throw std::runtime_error("unexpected native matrix vector site");
    static uint64_t calls[3]{};
    if(++calls[index]==1 || calls[index]%4096==0)
        std::fprintf(stderr,"[math:native] matrix vector %x: %llu calls%s\n",site,calls[index],
                     vp2sMatrixVectorReferenceEnabled()?" (original comparison exact)":" (no VU state)");
}
bool vp2sLinearVectorReferenceEnabled() {
    static const bool check=[] {const char *v=std::getenv("VP2S_LINEAR_VECTOR_CHECK");return v && v[0]=='1';}();
    return !vp2sLinearVectorEnabled() || check;
}
void vp2sLinearVectorProduct(uint8_t *ram,R5900Context *ctx,uint32_t basisAddress,uint32_t vectorAddress,uint32_t destination,uint32_t site) {
    vp2s::math::RigidBasis basis;__m128 vector;
    std::memcpy(basis.data(),recordPointer(ram,basisAddress,48),48);
    std::memcpy(&vector,recordPointer(ram,vectorAddress,16),16);
    const auto result=vp2s::math::linearVectorProduct(basis,vector);
    const uint32_t rounding=_mm_getcsr()&0xe040u;
    if(vp2sLinearVectorReferenceEnabled() && std::memcmp(&result,&ctx->vu0_vf[4],16)) {
        if(const char *path=std::getenv("VP2S_LINEAR_VECTOR_FAILURE_RECORD"))
            if(FILE *file=std::fopen(path,"wb")) {
                const uint32_t tag=0x3152564cu;
                std::fwrite(&tag,4,1,file);std::fwrite(&rounding,4,1,file);std::fwrite(basis.data(),48,1,file);
                std::fwrite(&vector,16,1,file);std::fwrite(&ctx->vu0_vf[4],16,1,file);std::fclose(file);
            }
        throw std::runtime_error("native linear vector product differs from original game calculation");
    }
    std::memcpy(recordPointer(ram,destination,16),&result,16);
    auto &recorder=linearVectorCapture();
    if(recorder.begin(0x3152564cu)) {
        recorder.append(&rounding,4);recorder.append(basis.data(),48);recorder.append(&vector,16);recorder.append(&result,16);recorder.end();
    }
    const unsigned index=site==0x1fb200u?0:site==0x1fb318u?1:site==0x1fb448u?2:site==0x1fb640u?3:4;
    if(index==4) throw std::runtime_error("unexpected native linear vector site");
    static uint64_t calls[4]{};
    if(++calls[index]==1 || calls[index]%4096==0)
        std::fprintf(stderr,"[math:native] linear vector %x: %llu calls%s\n",site,calls[index],
                     vp2sLinearVectorReferenceEnabled()?" (original comparison exact)":" (no VU state)");
}
bool vp2sRigidInverseReferenceEnabled() {
    static const bool check=[] {const char *v=std::getenv("VP2S_RIGID_INVERSE_CHECK");return v && v[0]=='1';}();
    return !vp2sRigidInverseEnabled() || check;
}
void vp2sRigidInverseTranslation(uint8_t *ram,R5900Context *ctx,uint32_t basisAddress,uint32_t translationAddress) {
    vp2s::math::RigidBasis basis;
    __m128 translation;
    std::memcpy(basis.data(),recordPointer(ram,basisAddress,48),48);
    std::memcpy(&translation,recordPointer(ram,translationAddress,16),16);
    const auto result=vp2s::math::rigidInverseTranslation(basis,translation);
    const uint32_t rounding=_mm_getcsr()&0xe040u;
    if(vp2sRigidInverseReferenceEnabled() && std::memcmp(&result,&ctx->vu0_vf[4],16)) {
        if(const char *path=std::getenv("VP2S_RIGID_INVERSE_FAILURE_RECORD"))
            if(FILE *file=std::fopen(path,"wb")) {
                const uint32_t tag=0x31524952u;
                std::fwrite(&tag,4,1,file);std::fwrite(&rounding,4,1,file);std::fwrite(basis.data(),48,1,file);
                std::fwrite(&translation,16,1,file);std::fwrite(&ctx->vu0_vf[4],16,1,file);std::fclose(file);
            }
        throw std::runtime_error("native rigid inverse translation differs from original game calculation");
    }
    std::memcpy(recordPointer(ram,translationAddress,16),&result,16);
    auto &recorder=rigidInverseCapture();
    if(recorder.begin(0x31524952u)) {
        recorder.append(&rounding,4);recorder.append(basis.data(),48);recorder.append(&translation,16);recorder.append(&result,16);recorder.end();
    }
    static uint64_t calls=0;
    if(++calls==1 || calls%4096==0)
        std::fprintf(stderr,"[math:native] rigid inverse translation %llu calls%s\n",calls,
                     vp2sRigidInverseReferenceEnabled()?" (original comparison exact)":" (no VU state)");
}
bool vp2sNormalizeReferenceEnabled() {
    static const bool check=[] {const char *v=std::getenv("VP2S_NORMALIZE_CHECK");return v && v[0]=='1';}();
    return !vp2sNormalizeEnabled() || check;
}
void vp2sTransposedRigidInverseTranslation(uint8_t *ram,R5900Context *ctx,uint32_t matrixAddress,uint32_t translationAddress) {
    Matrix matrix;__m128 translation;
    std::memcpy(matrix.data(),recordPointer(ram,matrixAddress),64);
    std::memcpy(&translation,recordPointer(ram,translationAddress,16),16);
    const auto result=vp2s::math::transposedRigidInverseTranslation(matrix,translation);
    const uint32_t rounding=_mm_getcsr()&0xe040u;
    if(vp2sRigidInverseReferenceEnabled() && std::memcmp(&result,&ctx->vu0_vf[4],16)) {
        if(const char *path=std::getenv("VP2S_TRANSPOSED_RIGID_FAILURE_RECORD"))
            if(FILE *file=std::fopen(path,"wb")) {
                const uint32_t tag=0x31524954u;
                std::fwrite(&tag,4,1,file);std::fwrite(&rounding,4,1,file);std::fwrite(matrix.data(),64,1,file);
                std::fwrite(&translation,16,1,file);std::fwrite(&ctx->vu0_vf[4],16,1,file);std::fclose(file);
            }
        throw std::runtime_error("native transposed rigid inverse differs from original game calculation");
    }
    std::memcpy(recordPointer(ram,translationAddress,16),&result,16);
    auto &recorder=transposedRigidCapture();
    if(recorder.begin(0x31524954u)) {
        recorder.append(&rounding,4);recorder.append(matrix.data(),64);recorder.append(&translation,16);recorder.append(&result,16);recorder.end();
    }
    static uint64_t calls=0;
    if(++calls==1 || calls%4096==0)
        std::fprintf(stderr,"[math:native] transposed rigid inverse %llu calls%s\n",calls,
                     vp2sRigidInverseReferenceEnabled()?" (original comparison exact)":" (no VU state)");
}
void vp2sRigidInverseColumnTranslation(uint8_t *ram,R5900Context *ctx,uint32_t basisAddress,uint32_t translationAddress,uint32_t destination) {
    vp2s::math::RigidBasis basis;__m128 translation;
    std::memcpy(basis.data(),recordPointer(ram,basisAddress,48),48);
    std::memcpy(&translation,recordPointer(ram,translationAddress,16),16);
    const auto result=vp2s::math::rigidInverseColumnTranslation(basis,translation);
    const uint32_t rounding=_mm_getcsr()&0xe040u;
    if(vp2sRigidInverseReferenceEnabled() && std::memcmp(&result,&ctx->vu0_vf[31],16)) {
        if(const char *path=std::getenv("VP2S_RIGID_INVERSE_COLUMN_FAILURE_RECORD"))
            if(FILE *file=std::fopen(path,"wb")) {
                const uint32_t tag=0x314e4952u;
                std::fwrite(&tag,4,1,file);std::fwrite(&rounding,4,1,file);std::fwrite(basis.data(),48,1,file);
                std::fwrite(&translation,16,1,file);std::fwrite(&ctx->vu0_vf[31],16,1,file);std::fclose(file);
            }
        throw std::runtime_error("native column rigid inverse differs from original game calculation");
    }
    std::memcpy(recordPointer(ram,destination,16),&result,16);
    auto &recorder=rigidColumnCapture();
    if(recorder.begin(0x314e4952u)) {
        recorder.append(&rounding,4);recorder.append(basis.data(),48);recorder.append(&translation,16);recorder.append(&result,16);recorder.end();
    }
    static uint64_t calls=0;
    if(++calls==1 || calls%4096==0)
        std::fprintf(stderr,"[math:native] rigid column inverse %llu calls%s\n",calls,
                     vp2sRigidInverseReferenceEnabled()?" (original comparison exact)":" (no VU state)");
}
void vp2sNormalizeRecord(uint8_t *ram,R5900Context *ctx,uint32_t source,uint32_t destination) {
    const __m128 input=_mm_loadu_ps(reinterpret_cast<const float*>(recordPointer(ram,source,16)));
    const __m128 result=vp2s::math::normalizeQuaternion(input);
    if(vp2sNormalizeReferenceEnabled() && std::memcmp(&result,&ctx->vu0_vf[6],16)) {
        if(const char *path=std::getenv("VP2S_NORMALIZE_FAILURE_RECORD"))
            if(FILE *file=std::fopen(path,"wb")) {
                const uint32_t tag=0x3152344eu;
                std::fwrite(&tag,4,1,file); std::fwrite(&input,16,1,file);
                std::fwrite(&ctx->vu0_vf[6],16,1,file); std::fclose(file);
            }
        throw std::runtime_error("native quaternion differs from original game calculation");
    }
    std::memcpy(recordPointer(ram,destination,16),&result,16);
    auto &recorder=quaternionCapture();
    if(recorder.begin(0x3152344eu)) {
        recorder.append(&input,16); recorder.append(&result,16); recorder.end();
    }
    static uint64_t calls=0;
    if(++calls==1 || calls%4096==0)
        std::fprintf(stderr,"[math:native] quaternion normalization %llu calls%s\n",calls,
                     vp2sNormalizeReferenceEnabled()?" (original comparison exact)":" (no VU execution)");
}
void vp2sInstallMath(PS2Runtime &runtime) {
    g_workingMemory=runtime.memory().getScratchpad();
    (void)inlineQuaternionCapture();
    (void)rigidInverseCapture();
    (void)rigidColumnCapture();
    (void)projectionCapture();
    (void)offsetDotCapture();
    (void)transposedRigidCapture();
    (void)linearVectorCapture();
    (void)matrixVectorCapture();
    (void)pointDirectionCapture();
    (void)quaternionCapture();
    const auto *module=ps2_overlays::findModule("mod001");
    if(!module) throw std::runtime_error("native math module missing");
    if(const char *v=std::getenv("VP2S_NATIVE_QUAT_MATRIX");!(v && v[0]=='0')) {
        if(kQuaternionMatrix<module->tableBase || kQuaternionMatrix>=module->tableEnd ||
           !(g_originalQuaternionMatrix=module->table[(kQuaternionMatrix-module->tableBase)>>2]))
            throw std::runtime_error("native quaternion matrix hook missing");
        (void)quaternionMatrixCapture();
        ps2_overlays::replaceModuleFunction("mod001",kQuaternionMatrix,&quaternionMatrix);
    }
    if(const char *v=std::getenv("VP2S_NATIVE_MATRIX");v && v[0]=='0') return;
    if(!module || kProduct<module->tableBase || kProduct>=module->tableEnd ||
       !(g_original=module->table[(kProduct-module->tableBase)>>2]))
        throw std::runtime_error("native matrix product hook missing");
    (void)capture();
    ps2_overlays::replaceModuleFunction("mod001",kProduct,&product);
}

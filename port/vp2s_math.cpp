// Shared game matrix operations on EE records, with no VU state or transport.
#include "vp2s_math.h"
#include "ps2_runtime.h"
#include "ps2_overlay_modules.h"
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

namespace {
using vp2s::math::Matrix;
constexpr uint32_t kProduct=0x00217550u;
PS2Runtime::RecompiledFunction g_original=nullptr;
uint8_t *recordPointer(uint8_t *ram,uint32_t address) {
    address&=0x1FFFFFFu;
    if(address<0x100000u || address>0x2000000u-sizeof(Matrix) || (address&15u))
        throw std::runtime_error("invalid native matrix record");
    return ram+address;
}
struct Capture {
    std::string path;
    std::vector<uint8_t> bytes;
    std::thread writer;
    unsigned saved=0;
    Capture() {
        if(const char *p=std::getenv("VP2S_MATRIX_RECORD")) {
            path=p; bytes.reserve(4096u*(4u+3u*sizeof(Matrix)));
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
    void record(const Matrix &a,const Matrix &b,const Matrix &result) {
        if(path.empty() || saved==4096) return;
        const uint32_t tag=0x3152344du; // M4R1: two input matrices and a native result
        append(&tag,4); append(&a,sizeof(a)); append(&b,sizeof(b)); append(&result,sizeof(result));
        if(++saved==4096) writer=std::thread([p=path,data=std::move(bytes)] {save(p,data);});
    }
};
Capture &capture() { static Capture value; return value; }
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
    capture().record(local,parent,result);
    static uint64_t calls=0;
    if(++calls==1 || calls%4096==0)
        std::fprintf(stderr,"[math:native] matrix product %llu calls%s\n",calls,check?" (original comparison exact)":" (no VU execution)");
}
}
void vp2sInstallMath(PS2Runtime &) {
    if(const char *v=std::getenv("VP2S_NATIVE_MATRIX");v && v[0]=='0') return;
    const auto *module=ps2_overlays::findModule("mod001");
    if(!module || kProduct<module->tableBase || kProduct>=module->tableEnd ||
       !(g_original=module->table[(kProduct-module->tableBase)>>2]))
        throw std::runtime_error("native matrix product hook missing");
    (void)capture();
    ps2_overlays::replaceModuleFunction("mod001",kProduct,&product);
}

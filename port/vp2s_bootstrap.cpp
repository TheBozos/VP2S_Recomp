// Native startup key derivation from the game's Csi response, not VU registers.
#include "vp2s_bootstrap.h"
#include "ps2_runtime.h"
#include <cstdio>
#include <cstdlib>

namespace {
const vp2s::bootstrap::Tables &tables() {
    static const auto value=[] {
        vp2s::bootstrap::Tables result{};
        const char *path=std::getenv("VP2S_BOOTSTRAP_TABLES");
        if(!path) path="../native_bootstrap/tables.bin";
        FILE *file=std::fopen(path,"rb");
        char magic[4]{}; uint32_t size=0;
        const bool ok=file && std::fread(magic,1,4,file)==4 && !std::memcmp(magic,"VPBK",4) &&
                      std::fread(&size,4,1,file)==1 && size==sizeof(result) &&
                      std::fread(&result,1,sizeof(result),file)==sizeof(result);
        if(file) std::fclose(file);
        if(!ok) throw std::runtime_error("native startup tables missing/invalid: run tools/vp2_bootstrap.py");
        return result;
    }();
    return value;
}
}
bool vp2sBootstrapEnabled() {
    static const bool enabled=[] { const char *v=std::getenv("VP2S_NATIVE_BOOTSTRAP"); return !(v && v[0]=='0'); }();
    return enabled;
}
void vp2sInstallBootstrap() {
    if(vp2sBootstrapEnabled()) (void)tables();
}
void vp2sBootstrapKey(uint8_t *ram,R5900Context *ctx) {
    // The Csi #14 RPC has completed. Its four float words occupy this EE stack
    // record; the original code replaces it with the key and loads GPR v0.
    const uint32_t address=getRegU32(ctx,16)&0x1FFFFFFu;
    if(address<0x100000u || address>0x2000000u-16u)
        throw std::runtime_error("invalid native startup seed record");
    vp2s::bootstrap::Words seed;
    std::memcpy(seed.data(),ram+address,16);
    const auto key=vp2s::bootstrap::derive(seed,tables());
    std::memcpy(ram+address,key.data(),16);
    ctx->r[2]=_mm_loadu_si128(reinterpret_cast<const __m128i*>(key.data()));
    ctx->r[9]=_mm_insert_epi64(ctx->r[9],0,0); // CFC2's low-64-bit result; preserve the upper half
    ctx->pc=0x100930u;
    if(const char *path=std::getenv("VP2S_BOOTSTRAP_RECORD")) {
        if(FILE *file=std::fopen(path,"wb")) {
            std::fwrite(seed.data(),1,16,file); std::fwrite(key.data(),1,16,file);
            std::fclose(file);
        }
    }
    std::fprintf(stderr,"[bootstrap:native] derived startup key directly from Csi seed; no VIF0 upload or VU0 execution\n");
}

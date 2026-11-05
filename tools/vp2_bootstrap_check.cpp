// Offline verification only; no reference VU state is linked into the native bridge.
#include "vp2s_bootstrap.h"
#include "runtime/ps2_vu1.h"
#include <cstdio>
#include <random>

class PS2Runtime;
extern const uint32_t g_ps2RecompiledFunctionTableBase=0u;
extern const uint32_t g_ps2RecompiledFunctionTableEnd=0u;
extern const uint32_t g_ps2RecompiledFunctionTableSlotCount=0u;
void (*g_ps2RecompiledFunctionTable[1])(uint8_t*,R5900Context*,PS2Runtime*)={nullptr};
namespace ps2x::vu1lift {
using Kick=void (*)(void*,const uint8_t*,uint32_t);
template<bool> uint64_t bootstrap_reference(uint32_t,VU1State&,uint8_t*,uint32_t,Kick,void*);
}
int main(int argc,char **argv) {
    if(argc!=3) return 2;
    using namespace vp2s::bootstrap;
    Tables tables{};
    FILE *file=std::fopen(argv[1],"rb");
    char magic[4]{}; uint32_t size=0;
    if(!file || std::fread(magic,1,4,file)!=4 || std::memcmp(magic,"VPBK",4) ||
       std::fread(&size,4,1,file)!=1 || size!=sizeof(tables) || std::fread(&tables,1,sizeof(tables),file)!=sizeof(tables)) return 2;
    std::fclose(file);
    Words capturedSeed{},capturedKey{};
    file=std::fopen(argv[2],"rb");
    if(!file || std::fread(capturedSeed.data(),1,16,file)!=16 || std::fread(capturedKey.data(),1,16,file)!=16) return 2;
    std::fclose(file);
    std::mt19937 random(0x56503253u);
    std::uniform_real_distribution<float> value(-10000.f,10000.f);
    unsigned differences=0;
    vp2s::music::CodecRounding rounding;
    for(unsigned test=0;test<65537;++test) {
        Words seed=capturedSeed;
        if(test) for(auto &word:seed) word=std::bit_cast<uint32_t>(value(random));
        VU1State state{}; state.vf[0][3]=1.f;
        std::memcpy(state.vf[1],seed.data(),16);
        alignas(16) uint8_t data[4096]{};
        ps2x::vu1lift::bootstrap_reference<false>(0,state,data,0,nullptr,nullptr);
        const auto result=derive(seed,tables);
        if(std::memcmp(result.data(),state.vf[3],16)) ++differences;
        if(!test && result!=capturedKey) ++differences;
    }
    std::printf("native bootstrap: 65537 seeds, %u differences; key values suppressed\n",differences);
    return differences?1:0;
}

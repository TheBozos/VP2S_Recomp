// Offline only. The game never links these original reference programs or
// their VU-state interface; native records contain typed DSP buffers.
#include "vp2s_music_math.h"
#include "runtime/ps2_vu1.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

// Arithmetic-library runner symbols. No runtime is constructed and no EE code runs.
struct R5900Context;
class PS2Runtime;
extern const uint32_t g_ps2RecompiledFunctionTableBase=0u;
extern const uint32_t g_ps2RecompiledFunctionTableEnd=0u;
extern const uint32_t g_ps2RecompiledFunctionTableSlotCount=0u;
void (*g_ps2RecompiledFunctionTable[1])(uint8_t*,R5900Context*,PS2Runtime*)={nullptr};
namespace ps2x::vu1lift {
using Kick=void (*)(void*,const uint8_t*,uint32_t);
template<bool> uint64_t music_6e0(uint32_t,VU1State&,uint8_t*,uint32_t,Kick,void*);
template<bool> uint64_t music_1410(uint32_t,VU1State&,uint8_t*,uint32_t,Kick,void*);
template<bool> uint64_t music_1690(uint32_t,VU1State&,uint8_t*,uint32_t,Kick,void*);
}
using namespace vp2s::music;
void referenceDecode(const Quantized &input,const Tables &tables,const Block &before,Block &output,Block &after) {
    alignas(16) std::array<uint8_t,16384> data{};
    VU1State state{}; state.vf[0][3]=1.f;
    auto copy=[&](unsigned row,const auto &value) {std::memcpy(data.data()+row*16,&value,sizeof(value));};
    copy(256,input.header); copy(257,input.bands); copy(264,input.coefficients);
    copy(496,tables.rotations); copy(512,tables.inverseQuant);
    ps2x::vu1lift::music_6e0<false>(0x6e0,state,data.data(),0,nullptr,nullptr);
    copy(768,tables.windowMatrix);
    ps2x::vu1lift::music_1410<false>(0x1410,state,data.data(),0,nullptr,nullptr);
    copy(512,before); copy(256,tables.synthesis);
    ps2x::vu1lift::music_1690<false>(0x1690,state,data.data(),0,nullptr,nullptr);
    std::memcpy(output.data(),data.data()+768*16,4096);
    std::memcpy(after.data(),data.data()+512*16,4096);
}
int check(int argc,char **argv) {
    if(argc!=3 && argc!=4) {
        std::fprintf(stderr,"usage: vp2s_music_check <tables.bin> <records.bin|--synthetic> [reference-output.bin]\n");
        return 2;
    }
    const bool synthetic=std::strcmp(argv[2],"--synthetic")==0;
    FILE *tf=std::fopen(argv[1],"rb"), *rf=synthetic?nullptr:std::fopen(argv[2],"rb");
    FILE *writer=argc==4?std::fopen(argv[3],"wb"):nullptr;
    if(!tf || (!synthetic && !rf) || (argc==4 && !writer)) return 2;
    Tables tables{}; char magic[4]; uint32_t size=0;
    if(std::fread(magic,1,4,tf)!=4 || std::memcmp(magic,"VPMU",4) || std::fread(&size,4,1,tf)!=1 ||
       size!=sizeof(tables) || std::fread(&tables,1,sizeof(tables),tf)!=sizeof(tables)) return 2;
    std::fclose(tf);
    uint64_t blocks=0,pcmErrors=0,historyErrors=0,captureErrors=0;
    CodecRounding rounding;
    auto test=[&](const Quantized &input,const Block &before,const Block *captured,const Block *capturedHistory) {
        Block expected{},expectedHistory{},spectrum{},output{},history=before;
        referenceDecode(input,tables,before,expected,expectedHistory);
        dequantize(input,tables,spectrum);
        window(spectrum,tables,output);
        synthesize(tables,history,output,float(input.header[0]));
        const bool pcm=std::memcmp(expected.data(),output.data(),4096)==0;
        const bool h=std::memcmp(expectedHistory.data(),history.data(),4096)==0;
        pcmErrors+=!pcm; historyErrors+=!h;
        if(captured && (std::memcmp(captured->data(),expected.data(),4096) ||
                       std::memcmp(capturedHistory->data(),expectedHistory.data(),4096))) ++captureErrors;
        if((!pcm || !h) && pcmErrors+historyErrors<10)
            std::fprintf(stderr,"block %llu native PCM %s history %s\n",blocks,pcm?"exact":"different",h?"exact":"different");
        if(writer) {
            const uint32_t tag=0x3143554du;
            std::fwrite(&tag,4,1,writer); std::fwrite(&input,sizeof(input),1,writer);
            std::fwrite(&before,sizeof(before),1,writer); std::fwrite(&expected,sizeof(expected),1,writer);
            std::fwrite(&expectedHistory,sizeof(expectedHistory),1,writer);
        }
        ++blocks;
    };
    if(synthetic) {
        for(int gain:{1,6501,16256}) for(int scale:{0,4,127}) for(int width=0;width<=16;++width) {
            Quantized input{}; Block before{};
            input.header[0]=gain; input.bands.fill(width); input.bands[0]=scale;
            const int radius=width?1<<(width-1):0;
            for(size_t k=0;k<216*4;++k) input.coefficients[k/4][k%4]=radius?(k%2?radius-1:-radius):0;
            test(input,before,nullptr,nullptr);
        }
        for(int zeros=1;zeros<=27;++zeros) {
            Quantized input{}; Block before{};
            input.header[0]=16256; input.bands.fill(8); input.bands[0]=0;
            for(int k=0;k<zeros;++k) input.bands[k+1]=0;
            for(size_t k=0;k<216*4;++k) input.coefficients[k/4][k%4]=k%2?127:-128;
            test(input,before,nullptr,nullptr);
        }
    } else for(;;) {
        uint32_t tag=0;
        if(std::fread(&tag,4,1,rf)!=1) break;
        Quantized input{}; Block before{},output{},after{};
        if(tag!=0x3143554du || std::fread(&input,sizeof(input),1,rf)!=1 || std::fread(&before,sizeof(before),1,rf)!=1 ||
           std::fread(&output,sizeof(output),1,rf)!=1 || std::fread(&after,sizeof(after),1,rf)!=1) return 2;
        test(input,before,writer?nullptr:&output,writer?nullptr:&after);
    }
    if(rf) std::fclose(rf);
    if(writer) std::fclose(writer);
    std::printf("native music: %llu blocks, %llu PCM samples, PCM differences %llu, history differences %llu, captured-block differences %llu\n",
                blocks,blocks*1024,pcmErrors,historyErrors,captureErrors);
    return blocks && !pcmErrors && !historyErrors && !captureErrors ? 0 : 1;
}
int main(int argc,char **argv) {
    try { return check(argc,argv); }
    catch(const std::exception &error) { std::fprintf(stderr,"music check: %s\n",error.what()); return 2; }
}

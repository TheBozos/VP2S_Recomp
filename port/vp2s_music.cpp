// Streamed music from the game's coefficient records directly to PCM. The IOP
// service is still present; the EE-side decoder needs no VIF/VU/DMA environment.
#include "vp2s_music.h"
#include "vp2s_music_math.h"
#include "ps2_runtime.h"
#include "ps2_overlay_modules.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {
using namespace vp2s::music;
constexpr uint32_t kQueue = 0x00201F20u, kNext = 0x001060A0u, kJobClass = 0x00151D30u;
constexpr uint32_t kComplete=0x00105B80u, kPostMessage=0x001131C0u, kReturn=0x00106038u;
PS2Runtime::RecompiledFunction g_queue = nullptr;
PS2Runtime::RecompiledFunction g_postMessage=nullptr;

uint32_t reg(R5900Context *ctx, unsigned n) { return getRegU32(ctx,n); }
void setReg(R5900Context *ctx,unsigned n,uint32_t v) { ctx->r[n]=_mm_insert_epi64(ctx->r[n],int64_t(int32_t(v)),0); }
uint8_t *pointer(uint8_t *ram, uint32_t address, size_t size) {
    address &= 0x1FFFFFFu;
    if (address < 0x100000u || size > 0x2000000u - address)
        throw std::runtime_error("invalid native music buffer");
    return ram + address;
}
uint32_t read(uint8_t *ram, uint32_t address) {
    uint32_t value; std::memcpy(&value,pointer(ram,address,4),4); return value;
}
void write(uint8_t *ram, uint32_t address,uint32_t value) { std::memcpy(pointer(ram,address,4),&value,4); }
const Tables &tables() {
    static const Tables value=[] {
        Tables t{};
        const char *path=std::getenv("VP2S_MUSIC_TABLES");
        if(!path) path="../native_music/tables.bin";
        FILE *f=std::fopen(path,"rb");
        char magic[4]{}; uint32_t size=0;
        bool ok=f && std::fread(magic,1,4,f)==4 && std::memcmp(magic,"VPMU",4)==0 &&
                std::fread(&size,4,1,f)==1 && size==sizeof(t) && std::fread(&t,1,sizeof(t),f)==sizeof(t);
        if(f) std::fclose(f);
        if(!ok) throw std::runtime_error("native music tables missing/invalid: run tools/vp2_music.py");
        return t;
    }();
    return value;
}
Quantized coefficients(const uint8_t *source) {
    Quantized input{};
    // The first four reserved bytes are not part of the coefficient record.
    auto get=[&](size_t k) { int16_t v; std::memcpy(&v,source+4+2*k,2); return int32_t(v); };
    for(size_t k=0;k<4;++k) input.header[k]=get(k);
    for(size_t k=0;k<28;++k) input.bands[k]=get(4+k);
    // 27 coded bands, 32 coefficients each. The final eight vectors are unused
    // lookahead slots in the lifted algorithm and contain native zero padding.
    for(size_t k=0;k<216*4;++k) input.coefficients[k/4][k%4]=get(32+k);
    return input;
}
struct Capture {
    std::string path;
    std::vector<uint8_t> bytes;
    std::thread writer;
    unsigned considered=0, saved=0, skip=0;
    Capture() {
        if(const char *p=std::getenv("VP2S_NATIVE_MUSIC_RECORD")) {
            path=p;
            bytes.reserve(1024u*(4u+sizeof(Quantized)+3u*sizeof(Block)));
        }
        if(const char *v=std::getenv("VP2S_NATIVE_MUSIC_RECORD_SKIP")) skip=unsigned(std::strtoul(v,nullptr,10));
    }
    static void save(const std::string &path,const std::vector<uint8_t> &bytes) {
        FILE *file=std::fopen(path.c_str(),"wb");
        if(!file) { std::fprintf(stderr,"[music:native] cannot open capture %s\n",path.c_str()); return; }
        const bool ok=std::fwrite(bytes.data(),1,bytes.size(),file)==bytes.size();
        std::fclose(file);
        if(!ok) std::fprintf(stderr,"[music:native] incomplete capture %s\n",path.c_str());
    }
    ~Capture() {
        if(writer.joinable()) writer.join();
        else if(!path.empty() && !bytes.empty()) save(path,bytes);
    }
    void append(const void *data,size_t size) {
        const auto *begin=static_cast<const uint8_t*>(data);
        bytes.insert(bytes.end(),begin,begin+size);
    }
    void record(const Quantized &input,const Block &before,const Block &output,const Block &after) {
        if(path.empty() || considered++ < skip || saved>=1024) return;
        const uint32_t magic=0x3143554Du; // MUC1: typed inputs/outputs, no hardware state
        append(&magic,4); append(&input,sizeof(input)); append(&before,sizeof(before));
        append(&output,sizeof(output)); append(&after,sizeof(after));
        if(++saved==1024) {
            // Disk flushes in the sound worker can starve its existing IOP service.
            // Capture only bounded host memory here; persist on a host worker.
            writer=std::thread([outputPath=path,data=std::move(bytes)] { save(outputPath,data); });
        }
    }
};
Capture &capture() { static Capture value; return value; }
void copyAnalysisRing(uint8_t *ram,uint32_t job,unsigned channel,const Block &output) {
    const uint32_t base=read(ram,job+0x10u+4u*channel);
    if(!base) return;
    const uint32_t size=read(ram,job+0x18u), reader=read(ram,job+0x1Cu+4u*channel);
    const uint32_t writer=read(ram,job+0x24u+4u*channel);
    uint32_t next=writer+4096u;
    bool room;
    if(next < base+size) room=next<reader || writer>=reader;
    else { room=writer>=reader; next-=size; }
    if(room) { std::memcpy(pointer(ram,writer,4096),output.data(),4096); write(ram,job+0x24u+4u*channel,next); }
}
struct Decoded {
    Quantized input{};
    Block before{},output{},history{};
    uint32_t player=0;
    unsigned channel=0;
};
std::unordered_map<uint32_t,Decoded> g_pending;
void decode(uint8_t *ram,uint32_t player,uint32_t job,unsigned channel) {
    const uint32_t source=read(ram,player+0x1Cu);
    if(g_pending.contains(job&0x1FFFFFFu)) throw std::runtime_error("native music job already pending");
    CodecRounding rounding;
        const auto input=coefficients(pointer(ram,source+channel*2048u,2048));
        Block spectrum{}, output{}, history{};
        uint8_t *historyBuffer=pointer(ram,read(ram,player+0xF0u+4u*channel),4096);
        std::memcpy(history.data(),historyBuffer,4096);
        const Block before=history;
        unsigned stage=1;
        try {
            dequantize(input,tables(),spectrum);
            stage=2; window(spectrum,tables(),output);
            stage=3; synthesize(tables(),history,output,float(input.header[0]));
        } catch(const std::exception &error) {
            if(const char *path=std::getenv("VP2S_NATIVE_MUSIC_FAILURE_RECORD"))
                if(FILE *f=std::fopen(path,"wb")) {
                    const uint32_t magic=0x4C46554Du; // MUFL: typed failing inputs, before any fallback
                    std::fwrite(&magic,4,1,f); std::fwrite(&stage,4,1,f);
                    std::fwrite(&input,sizeof(input),1,f); std::fwrite(&before,sizeof(before),1,f);
                    std::fclose(f);
                }
            throw std::runtime_error("native music stage "+std::to_string(stage)+" player "+std::to_string(player)+
                                     " source "+std::to_string(source+channel*2048u)+": "+error.what());
        }
        g_pending.emplace(job&0x1FFFFFFu,Decoded{input,before,output,history,player,channel});
}
bool advance(uint8_t *ram,uint32_t player,uint32_t job) {
    write(ram,player+0x1Cu,read(ram,player+0x1Cu)+4096u);
    write(ram,player+0x50u,read(ram,player+0x50u)+4096u);
    const unsigned half=*pointer(ram,player+0x1Au,1);
    if(half>1) throw std::runtime_error("invalid native music input half");
    const uint32_t count=read(ram,player+0x2Cu+4u*half);
    if(!count || count>1024) throw std::runtime_error("invalid native music frame count");
    write(ram,player+0x2Cu+4u*half,count-1);
    if(count>1) {
        // Preserve the game's per-frame marker cache, later copied into the
        // outgoing SIF packet header. Consuming a marker needs no VIF command.
        const uint32_t marker=read(ram,player+0xC8u);
        auto *record=pointer(ram,read(ram,player+0x1Cu),1);
        *pointer(ram,marker,1)=*record;
        *record=0;
        write(ram,player+0xC8u,marker+1u);
        write(ram,job+8u,read(ram,player+0x50u)|0x20000000u);
        *pointer(ram,job+12u,1)=1;
        return true;
    }
    const unsigned outputHalf=*pointer(ram,player+0x4Bu,1);
    if(outputHalf>1) throw std::runtime_error("invalid native music output half");
    *pointer(ram,player+0x48u+outputHalf,1)=1;
    *pointer(ram,player+0x1Au,1)=uint8_t(1-half);
    write(ram,player+0x1Cu,read(ram,player+(half?0x3Cu:0x40u))+0x80u);
    *pointer(ram,player+0x4Bu,1)=uint8_t(1-outputHalf);
    write(ram,player+0x50u,read(ram,player+(outputHalf?0x64u:0x68u))+0x80u);
    *pointer(ram,player+0xB9u,1)=0;
    *pointer(ram,job+0xCu,1)=0;
    *pointer(ram,read(ram,0x0035DDC0u)+6u,1)=0;
    return false;
}
void returned(uint8_t *ram,R5900Context *ctx,PS2Runtime*) {
    const uint32_t sp=reg(ctx,29), caller=read(ram,sp), job=read(ram,sp+8u);
    if(read(ram,sp+12u)!=0x4D534731u || reg(ctx,2)==0)
        throw std::runtime_error("native music completion message was not queued");
    const uint64_t ra=uint64_t(read(ram,sp))|(uint64_t(read(ram,sp+4u))<<32);
    ctx->r[31]=_mm_insert_epi64(ctx->r[31],int64_t(ra),0);
    setReg(ctx,29,sp+16u); setReg(ctx,2,job); ctx->pc=caller;
}
void postCompletion(uint8_t *ram,R5900Context *ctx,PS2Runtime *runtime,uint32_t job) {
    // Ordinary game-message submission, not a DMA completion/interrupt. Keep a
    // native return continuation so a cooperative EE yield is supported too.
    const uint32_t sp=reg(ctx,29)-16u;
    const uint64_t ra=uint64_t(_mm_cvtsi128_si64(ctx->r[31]));
    write(ram,sp,uint32_t(ra)); write(ram,sp+4u,uint32_t(ra>>32));
    write(ram,sp+8u,job); write(ram,sp+12u,0x4D534731u);
    setReg(ctx,29,sp); setReg(ctx,31,kReturn);
    setReg(ctx,4,read(ram,0x0035DD78u)); setReg(ctx,5,5u); setReg(ctx,6,0u);
    setReg(ctx,7,job); setReg(ctx,8,0u); setReg(ctx,9,0u);
    ctx->pc=kPostMessage; g_postMessage(ram,ctx,runtime);
    if(ctx->pc==kReturn) returned(ram,ctx,runtime);
}
void submit(uint8_t *ram,R5900Context *ctx,PS2Runtime *runtime,uint32_t player,uint32_t job,unsigned channel) {
    decode(ram,player,job,channel);
    postCompletion(ram,ctx,runtime,job);
}
void complete(uint8_t *ram,R5900Context *ctx,PS2Runtime *runtime) {
    const uint32_t job=reg(ctx,4), state=*pointer(ram,job+12u,1);
    auto it=g_pending.find(job&0x1FFFFFFu);
    if(!state) { if(it!=g_pending.end()) g_pending.erase(it); ctx->pc=reg(ctx,31); return; }
    if(it==g_pending.end() || state!=it->second.channel+1u)
        throw std::runtime_error("unexpected native music completion");
    Decoded ready=std::move(it->second); g_pending.erase(it);
    const uint32_t player=ready.player;
    std::memcpy(pointer(ram,read(ram,player+0xF0u+4u*ready.channel),4096),ready.history.data(),4096);
    const uint32_t destination=read(ram,job+8u);
    auto *pcm=pointer(ram,destination,2048);
    const auto *samples=reinterpret_cast<const int32_t*>(ready.output.data());
    for(size_t k=0;k<1024;++k) { const int16_t sample=int16_t(samples[k]); std::memcpy(pcm+2*k,&sample,2); }
    write(ram,job+8u,destination+2048u);
    copyAnalysisRing(ram,job,ready.channel,ready.output);
    capture().record(ready.input,ready.before,ready.output,ready.history);
    if(!ready.channel) {
        *pointer(ram,job+12u,1)=2;
        submit(ram,ctx,runtime,player,job,1);
        return;
    }
    static uint64_t frames=0;
    if(++frames==1 || frames%128==0)
        std::fprintf(stderr,"[music:native] %llu stereo frames decoded directly from EE coefficients\n",frames);
    if(advance(ram,player,job)) submit(ram,ctx,runtime,player,job,0);
    else ctx->pc=reg(ctx,31);
}
void next(uint8_t *ram,R5900Context *ctx,PS2Runtime *runtime) {
    const uint32_t player=reg(ctx,4), job=read(ram,player+0x100u);
    if(advance(ram,player,job)) submit(ram,ctx,runtime,player,job,0);
    else ctx->pc=reg(ctx,31);
}
void queue(uint8_t *ram,R5900Context *ctx,PS2Runtime *runtime) {
    const uint32_t job=reg(ctx,5);
    if((job&0x1FFFFFFu)<0x100000u || read(ram,job)!=kJobClass) { g_queue(ram,ctx,runtime); return; }
    const uint32_t player=read(ram,job+4u);
    if((read(ram,player+0x100u)&0x1FFFFFFu)!=(job&0x1FFFFFFu) || *pointer(ram,job+12u,1)!=1)
        throw std::runtime_error("unexpected native music job");
    submit(ram,ctx,runtime,player,job,0);
}
}
bool vp2sNativeMusicEnabled() {
    static bool enabled=[] { const char *v=std::getenv("VP2S_NATIVE_MUSIC"); return !(v && v[0]=='0'); }();
    return enabled;
}
void vp2sInstallMusic(PS2Runtime &runtime) {
    if(!vp2sNativeMusicEnabled()) return; // explicit comparison only; never an automatic fallback
    (void)tables();
    (void)capture(); // reserve diagnostic storage before the game/sound workers start
    const auto *module=ps2_overlays::findModule("mod001");
    if(!runtime.hasFunction(kNext) || !runtime.hasFunction(kComplete) || runtime.hasFunction(kReturn) ||
       !(g_postMessage=runtime.lookupFunction(kPostMessage)) || !module || kQueue<module->tableBase || kQueue>=module->tableEnd ||
       !(g_queue=module->table[(kQueue-module->tableBase)>>2]))
        throw std::runtime_error("native music queue entry missing");
    ps2_overlays::replaceModuleFunction("mod001",kQueue,&queue);
    runtime.replaceFunction(kNext,&next);
    runtime.replaceFunction(kComplete,&complete);
    runtime.registerFunction(kReturn,&returned); // verified padding in the main ELF, never executed as MIPS
    std::fprintf(stderr,"[music:native] direct coefficient decoder installed; no music VIF1 chains\n");
}

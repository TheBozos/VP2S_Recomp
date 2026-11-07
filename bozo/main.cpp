// Initial BozoEngine bring-up. Unsupported boundaries fail explicitly.
#include "bozo/ee/dispatcher.h"
#include "bozo/host/free_host.h"
#include "bozo/port.h"
#include "vp2_bozo_images.h"
#include "native_kernel.h"
#include "native_timer.h"
#include "native_settings.h"
#include "native_seed.h"
#include "native_modules.h"
#include "vp2_dbc_image.h"
#include "vp2_controller_image.h"
#include "vp2_sound_image.h"
#include "vp2_csm_image.h"
#include "vp2_csd_image.h"
#include "vp2_csi_image.h"
#include "native_controller.h"
#include "native_frame.h"
#include "native_bootstrap.h"
#include "native_random.h"
#include "native_math.h"
#include "native_textures.h"
#include "native_scene.h"
#include <random>
#include <algorithm>
#include <charconv>
#include <filesystem>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace {
class StrictHost final : public bozo::ee::Host, public vp2s::TimerOwner, public vp2s::SettingsOwner,public vp2s::SeedOwner,public vp2s::ModuleOwner,public vp2s::BootstrapOwner,public vp2s::RandomOwner,public vp2s::TextureOwner,public vp2s::SceneOwner {
public:
    vp2s::NativeScene scene;
    vp2s::NativeScene &nativeScene()override{return scene;}
    vp2s::NativeTextures textures{std::filesystem::path(VP2S_BOZO_IMAGES).parent_path().parent_path()/"vp2_textures",std::filesystem::path(VP2S_BOZO_IMAGES).parent_path()/"textures.manifest"};
    vp2s::NativeTextures &nativeTextures()override{return textures;}
    vp2s::ProceduralRandom procedural;
    vp2s::ProceduralRandom &proceduralRandom()override{return procedural;}
    vp2s::bootstrap::Tables startupTables{};
    const vp2s::bootstrap::Tables &bootstrapTables() const override{return startupTables;}
    vp2s::ModuleBuffers moduleStorage;
    std::unique_ptr<vp2s::NativeServiceModules> serviceModules;
    vp2s::ModuleBuffers &moduleBuffers() override {return moduleStorage;}
    void validateNativeTransfer(uint32_t destination,size_t bytes) override {
        if(destination>=0x40000000)moduleStorage.validate(destination,bytes);
        else {
            if(bytes>UINT32_MAX)throw std::runtime_error("native sound buffer transfer is too large");
            dbc->allocatedBuffer(destination,static_cast<uint32_t>(bytes));
        }
    }
    void copyNativeTransfer(uint32_t destination,std::span<const uint8_t> data) override {
        validateNativeTransfer(destination,data.size());
        if(destination>=0x40000000)moduleStorage.copy(destination,data);
        else {auto to=dbc->allocatedBuffer(destination,static_cast<uint32_t>(data.size()));
            std::copy(data.begin(),data.end(),to.begin());}
    }
    uint32_t submitNativeCommand(uint32_t command,std::span<const uint8_t> data) override {
        return dbc->submitCommand(command,data);
    }
    vp2s::ModuleResult loadNativeModule(const std::string &path,std::span<const uint8_t>) override {
        throw std::runtime_error("unported native service module: "+path);
    }
    vp2s::ModuleResult loadNativeModuleBuffer(uint32_t handle,std::span<const uint8_t> args) override {
        return serviceModules->loadBuffer(moduleStorage.view(handle),args);
    }
    uint32_t seed=0;
    uint32_t bootSeed() const override { return seed; }
    bozo::host::FreeHost native;
    vp2s::ControllerBridge controller;
    std::unique_ptr<bozo::native::irx::CompiledLibraries> dbc;
    vp2s::NativeCommandReplies replies;
    vp2s::TimerClock sdkClock;
    vp2s::TimerClock &timerClock() override { return sdkClock; }
    vp2s::Settings preferences;
    vp2s::Settings &settings() override { return preferences; }
    explicit StrictHost(const bozo::host::FreeOptions &options) : native(options),controller(native.libraries()) {
        std::ifstream startup(std::filesystem::path(VP2S_BOZO_IMAGES)/"bootstrap.bin",std::ios::binary);
        char magic[4];uint32_t tableBytes=0;
        if(!startup.read(magic,4) || std::memcmp(magic,"VPBK",4) ||
           !startup.read(reinterpret_cast<char *>(&tableBytes),4) || tableBytes!=sizeof(startupTables) ||
           !startup.read(reinterpret_cast<char *>(&startupTables),sizeof(startupTables)))
            throw std::runtime_error("private native startup tables are missing or invalid");
        dbc=std::make_unique<bozo::native::irx::CompiledLibraries>(native.libraries(),0x80000);
        dbc->useReadyAudioCompletions();
        dbc->useNativeFrameCallbacks();
        native.onFrame=[this](uint64_t,const R5900Context *){dbc->notifyFrameCompleted();};
        addVp2DbcImage(*dbc);
        addVp2ControllerImage(*dbc);controller.bind(*dbc);
        addVp2SoundImage(*dbc);
        addVp2CsmImage(*dbc);
        addVp2CsdImage(*dbc);
        addVp2CsiImage(*dbc);
        const auto root=std::filesystem::path(VP2S_BOZO_IMAGES).parent_path().parent_path()/"extract/iop";
        serviceModules=std::make_unique<vp2s::NativeServiceModules>(root,[this]{
            return native.libraries().find("scePadInit") && native.libraries().find("sceMcInit");},
            [this](std::string_view name,std::span<const uint8_t> args){
                if(!args.empty())throw std::runtime_error("unported native controller/DBC startup arguments");
                if(dbc->startImage(std::string(name)+";1")<=0)throw std::runtime_error("native DBC initialization failed");
                // DBC's guarded initialization and the replacement native
                // controller entry both return zero after successful setup.
                return 0;
            });
        auto &sif=native.libraries().sif;
        sif.allocateBuffer=[this](uint32_t bytes){return moduleStorage.allocate(bytes);};
        sif.freeBuffer=[this](uint32_t h){return moduleStorage.release(h);};
        sif.transfer=[this](uint32_t to,const uint8_t *data,uint32_t bytes){copyNativeTransfer(to,{data,bytes});};
        sif.resetProvider=[this]{moduleStorage.reset();serviceModules->reset();dbc->reset();controller.reset();replies.reset();};
        sif.receiveCommand=[this](uint32_t command,std::span<const uint8_t> data,uint32_t to,std::span<const uint8_t> extra){
            return replies.submit(native.libraries(),command,data,to,extra);
        };
        native.libraries().audioProvider=[this](uint64_t cycle){return dbc->advanceTo(cycle);};
        sif.moduleLoader=[this](const std::string &path){return loadNativeModule(path,{}).id;};
        if(const char *specified=std::getenv("VP2S_BOZO_SEED")) {
            const char *end=specified+std::char_traits<char>::length(specified);
            auto parsed=std::from_chars(specified,end,seed);
            if(parsed.ec!=std::errc{} || parsed.ptr!=end || specified==end)
                throw std::runtime_error("VP2S_BOZO_SEED must be an unsigned decimal 32-bit seed");
        } else seed=std::random_device{}();
    }
    void boundaryHash(bozo::ee::Dispatcher &d) {
        if(!std::getenv("VP2S_BOZO_BOUNDARY_HASH")) return;
        uint64_t hash=14695981039346656037ull;
        const auto *ram=d.runtime().memory.flat();
        for(uint32_t i=0;i<bozo::kFlatBytes;++i) { hash^=ram[i];hash*=1099511628211ull; }
        std::fprintf(stderr,"[vp2s:bozo] boundary RAM hash=%016llx\n",(unsigned long long)hash);
    }
    unsigned traceLibrary=0;
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t target) override {
        // The engine's TMB upload retirement assumes that port already owns
        // the texture. VP2S has not installed its native asset owner yet.
        if(target==0x122bf8u)
            throw std::runtime_error(std::format("native VP2S image owner required at {:x} ra={:x}",target,c->r[31].uw[0]));
        if(target==0x123550u || target==0x1235b8u || target==0x123630u || target==0x1237f0u)
            throw std::runtime_error(std::format("native VP2S draw/data owner required at {:x} ra={:x}",target,c->r[31].uw[0]));
        if(std::getenv("VP2S_BOZO_TRACE_CALLS") && traceLibrary++<160)
            std::fprintf(stderr,"[vp2s:call] pc=%x ra=%x a0=%x a1=%x\n",target,c->r[31].uw[0],c->r[4].uw[0],c->r[5].uw[0]);
        native.libraryCall(d,c,target);
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *c,uint32_t address) override {
        throw std::runtime_error(std::format("unported VP2S VU0 algorithm {:x} at pc={:x}",address,c->pc));
    }
    void ioRead(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t address,uint32_t,uint64_t[2]) override {
        boundaryHash(d);
        throw std::runtime_error(std::format("unported device read {:x} at pc={:x}",address,c->pc));
    }
    void ioWrite(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t address,uint32_t,const uint64_t[2]) override {
        boundaryHash(d);
        if(std::getenv("VP2S_BOZO_TRACE_CALLS"))
            std::fprintf(stderr,"[vp2s:device] pc=%x a0=%x a1=%x ra=%x frames=%llu\n",
                         c->pc,c->r[4].uw[0],c->r[5].uw[0],c->r[31].uw[0],
                         (unsigned long long)native.framesEnded());
        throw std::runtime_error(std::format("unported device write {:x} at pc={:x}",address,c->pc));
    }
    bool cop0Condition(bozo::ee::Dispatcher &,R5900Context *c) override {
        throw std::runtime_error(std::format("unported device completion condition at pc={:x}",c->pc));
    }
    void fault(bozo::ee::Dispatcher &d,R5900Context *c,const std::string &what) override {
        std::fprintf(stderr,"[vp2s:bozo] fault args a0=%x a1=%x a2=%x a3=%x ra=%x\n",
                     c->r[4].uw[0],c->r[5].uw[0],c->r[6].uw[0],c->r[7].uw[0],c->r[31].uw[0]);
        native.fault(d,c,what);
    }
    void idle(bozo::ee::Dispatcher &d,uint64_t from,uint64_t to) override { native.idle(d,from,to); }
    void resuming(bozo::ee::Dispatcher &d,bozo::ee::Thread &t,bozo::ee::Frame &f,uint32_t depth) override { native.resuming(d,t,f,depth); }
    unsigned traceKernel=0;
    void kernelCallEnd(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t target,bool stop) override {
        if(std::getenv("VP2S_BOZO_TRACE_CALLS") && traceKernel++<160)
            std::fprintf(stderr,"[vp2s:kernel] pc=%x a0=%x result=%lld stop=%d\n",target,c->r[4].uw[0],(long long)c->r[2].sd[0],stop);
        native.kernelCallEnd(d,c,target,stop);
    }
    void kernelCallBegin(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t target,uint32_t kind) override { native.kernelCallBegin(d,c,target,kind); }
    void waitCompleted(bozo::ee::Dispatcher &d,bozo::ee::Thread &t) override { native.waitCompleted(d,t); }
    void handlerCompleted(bozo::ee::Dispatcher &d,bozo::ee::Thread &t,const bozo::ee::Frame &f) override { native.handlerCompleted(d,t,f); }
};
bool noNativeInstruction(uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc) {
    if(pc==0x1131c0){
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))
            std::fprintf(stderr,"[vp2s:enqueue] ra=%x queue=%x type=%u source=%x count=%u owner=%x\n",
                         ctx->r[31].uw[0],ctx->r[4].uw[0],ctx->r[5].uw[0],ctx->r[7].uw[0],ctx->r[8].uw[0],ctx->r[9].uw[0]);
        return false; // Observe the original CPU entry; execute its scalar prologue unchanged.
    }
    if(vp2s::nativeSceneInstruction(ram,ctx,rt,pc))return true;
    if(vp2s::nativeTextureInstruction(ram,ctx,rt,pc))return true;
    if(vp2s::nativeBootSeed(ram,ctx,rt,pc))return true;
    if(vp2s::nativeBootstrapKey(ram,ctx,rt,pc))return true;
    if(vp2s::nativeRandomSeed(ram,ctx,rt,pc))return true;
    if(vp2s::nativeMathInstruction(ram,ctx,rt,pc))return true;
    if(vp2s::nativeFrameInput(ram,ctx,rt,pc))return true;
    throw std::runtime_error(std::format("unported native VP2S algorithm at pc={:x}",pc));
}
}

int main(int argc,char **argv) {
    bozo::host::FreeOptions options;
    options.elf=std::string(VP2S_BOZO_IMAGES)+"/main.elf";
    options.frames=1; options.stallFields=120; options.printGameOutput=true;
    for(int i=1;i<argc;++i) {
        const std::string arg=argv[i];
        if(arg=="--iso" && i+1<argc)options.iso=argv[++i];
        else if(arg=="--card-dir" && i+1<argc)options.cardDir=argv[++i];
        else if(arg=="--frames" && i+1<argc)options.frames=std::stoul(argv[++i]);
        else { std::fprintf(stderr,"usage: vp2s-bozo-headless --iso FILE [--card-dir PRIVATE_DIR] [--frames N]\n");return 2; }
    }
    if(options.iso.empty()) {std::fprintf(stderr,"user-owned USA ISO required\n");return 2;}
    try {
        StrictHost host(options);
        bozo::ee::Dispatcher dispatcher(host);
        dispatcher.setFunctions(vp2_main_function_table,vp2_main_function_count,vp2_main_resume_table,vp2_main_resume_count);
        bozo::port::Registry registry;
        vp2s::installNativeKernelBindings(registry);
        registry.nativeGraphics(&noNativeInstruction); // guard retired graphics; no compatibility fallback
        std::string error;
        if(!bozo::port::undeclared(registry,vp2_main_external_table,vp2_main_external_count).empty())
            throw std::runtime_error("native kernel bindings missing from generated externals");
        if(!host.native.setUp(dispatcher,vp2_main_external_table,vp2_main_external_count,&registry,error))
            throw std::runtime_error(error);
        vp2sBozoRegisterImages(dispatcher.runtime(),VP2S_BOZO_IMAGES);
        const auto frames=host.native.run();
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES")) {
            // Only the audited graphics-work queue header and descriptor metadata.
            // Do not dump payloads, arbitrary CPU pointers, or bootstrap material.
            auto &memory=dispatcher.runtime().memory;
            const auto word=[&](uint32_t address) {
                uint32_t value=0;
                const auto *p=memory.hostPointer(address,sizeof(value));
                if(!p)throw std::runtime_error("queue diagnostic address outside native RAM");
                std::memcpy(&value,p,sizeof(value));return value;
            };
            const uint32_t owner=word(0x35dd78);
            if(owner>=0x100000 && owner<=0x1ffffe8) {
                const uint32_t slots=word(owner+4), indices=word(owner+8), flags=word(owner+12);
                const unsigned capacity=indices&0xffff, active=indices>>16, next=flags&0xffff;
                std::fprintf(stderr,"[vp2s:queue] owner=%x slots=%x capacity=%u active=%u next=%u flags=%08x tail=%08x\n",
                             owner,slots,capacity,active,next,flags,word(owner+16));
                if(capacity<=1024 && active<capacity && next<capacity && slots>=0x100000 && slots<=0x2000000-capacity*16)
                    for(unsigned i=active,n=0;i!=next && n<16;i=(i+1)%capacity,++n)
                        std::fprintf(stderr,"[vp2s:queue] slot=%u type=%u source=%x count=%u\n",i,
                                     word(slots+i*16)&0xffff,word(slots+i*16+4),word(slots+i*16+8));
            }
        }
        const auto &stats=host.native.stats();
        std::fprintf(stderr,"[vp2s:bozo] frames=%llu libraries=%llu end=%s\n",
                     (unsigned long long)frames,(unsigned long long)stats.libraryCalls,host.native.endReason().c_str());
        return frames>=options.frames?0:1;
    } catch(const std::exception &e) {
        std::fprintf(stderr,"[vp2s:bozo] native boundary: %s\n",e.what());return 1;
    }
}

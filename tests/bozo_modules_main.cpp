#include "native_kernel.h"
#include "native_modules.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <cstdio>
#include <stdexcept>
#include <fstream>
#include <iterator>
namespace {
void check(bool v){if(!v)throw std::runtime_error("native module ABI/buffer mismatch");}
template<class F>void rejects(F f){bool bad=false;try{f();}catch(const std::runtime_error &){bad=true;}check(bad);}
struct Host final:bozo::ee::Host,vp2s::ModuleOwner {
    vp2s::ModuleBuffers buffers;unsigned calls=0;std::string path;std::vector<uint8_t> args;
    vp2s::ModuleBuffers &moduleBuffers() override{return buffers;}
    vp2s::ModuleResult loadNativeModule(const std::string &p,std::span<const uint8_t> a) override {
        ++calls;path=p;args.assign(a.begin(),a.end());return {7,99};
    }
    vp2s::ModuleResult loadNativeModuleBuffer(uint32_t h,std::span<const uint8_t> a) override {
        check(buffers.view(h)[0]==0x55);++calls;args.assign(a.begin(),a.end());return {8,-17};
    }
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t) override {throw std::runtime_error("library");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t) override {throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2]) override {throw std::runtime_error("IO");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2]) override {throw std::runtime_error("IO");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s) override {throw std::runtime_error(s);}
};
}
int main(int argc,char **argv){try{
    Host h,other;bozo::ee::Dispatcher d(h);d.boot(0x100000,0x1fff000);
    auto &rt=d.runtime();auto *ram=rt.memory.ram();
    auto handle=h.buffers.allocate(32);check(handle>=0x40000000);
    check(!rt.memory.hostPointer(handle,1)); // opaque SDK buffer, never EE/IOP RAM
    rejects([&]{other.buffers.view(handle);});
    rejects([&]{h.buffers.allocate(0);});
    std::memset(ram+0x110000,0x55,32);
    uint32_t descriptors[8]={0x110000,handle,16,0,0x110010,handle+16,16,0};
    std::memcpy(ram+0x120000,descriptors,sizeof(descriptors));
    R5900Context c;c.pc=0x1375a0;c.r[4].uw[0]=0x120000;c.r[5].uw[0]=2;c.r[31].uw[0]=0x100008;
    d.kernel().addHandler(true,5,0x155000,0,0,0,0);
    vp2s::nativeModuleTransfer(nullptr,&c,&rt);
    check(c.r[2].sd[0]==1 && c.pc==0x100008 && h.buffers.view(handle)[31]==0x55);
    check(d.kernel().handlerQueued());d.kernel().stackHandler(*d.kernel().thread(1));d.kernel().endHandler(*d.kernel().thread(1));
    auto before=std::vector<uint8_t>(h.buffers.view(handle).begin(),h.buffers.view(handle).end());
    ram[0x110000]=0x66;descriptors[5]=handle+31;
    std::memcpy(ram+0x120000,descriptors,sizeof(descriptors));c.pc=0x1375a0;
    rejects([&]{vp2s::nativeModuleTransfer(nullptr,&c,&rt);});
    check(std::equal(before.begin(),before.end(),h.buffers.view(handle).begin()) && !d.kernel().handlerQueued());
    descriptors[5]=handle+16;descriptors[7]=1;std::memcpy(ram+0x120000,descriptors,sizeof(descriptors));
    rejects([&]{vp2s::nativeModuleTransfer(nullptr,&c,&rt);});
    c.r[5].uw[0]=33;rejects([&]{vp2s::nativeModuleTransfer(nullptr,&c,&rt);});
    c.pc=0x137580;vp2s::nativeModuleTransferStatus(nullptr,&c,&rt);check(c.r[2].sd[0]==-1);
    const char path[]="rom0:provider";std::memcpy(ram+0x130000,path,sizeof(path));
    ram[0x131000]=1;ram[0x131001]=0;ram[0x131002]=2;
    c.pc=0x1401b8;c.r[4].uw[0]=0x130000;c.r[5].uw[0]=3;c.r[6].uw[0]=0x131000;c.r[7].uw[0]=0x132000;c.r[8].uw[0]=0;
    vp2s::loadNativeModule(nullptr,&c,&rt);int32_t result;std::memcpy(&result,ram+0x132000,4);
    check(c.r[2].sd[0]==7 && result==99 && h.path==path && h.args==std::vector<uint8_t>({1,0,2}));
    c.pc=0x1401b8;c.r[8].uw[0]=1;rejects([&]{vp2s::loadNativeModule(nullptr,&c,&rt);});
    c.r[8].uw[0]=0;c.r[7].uw[0]=0x1fffffe;rejects([&]{vp2s::loadNativeModule(nullptr,&c,&rt);});
    check(h.calls==1);
    c.pc=0x13fba8;c.r[4].uw[0]=handle;c.r[7].uw[0]=0x132000;c.r[8].uw[0]=0xdeadbeef; // not an arg in buffer ABI
    vp2s::loadNativeModule(nullptr,&c,&rt);std::memcpy(&result,ram+0x132000,4);
    check(c.r[2].sd[0]==8 && result==-17 && h.calls==2);
    c.pc=0x13f680;c.r[4].uw[0]=handle;vp2s::freeNativeModuleBuffer(nullptr,&c,&rt);check(c.r[2].sd[0]==0);
    vp2s::freeNativeModuleBuffer(nullptr,&c,&rt);check(c.r[2].sd[0]==-1);
    rejects([&]{h.buffers.view(handle);});
    h.buffers.allocate(16);h.buffers.reset();rejects([&]{h.buffers.view(handle);});
    if(argc==2) {
        const std::filesystem::path root=argv[1];
        auto image=[&](const char *name){
            std::ifstream file(root/name,std::ios::binary);
            std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(file),{}};
            check(!bytes.empty());bytes.resize((bytes.size()+15)&~size_t(15));return bytes;
        };
        auto sio=image("sio2man.irx"),mux=image("sio2d.irx"),custom=image("Dbc_Manager.irx");
        vp2s::NativeServiceModules services(root,[]{return true;}),unready(root,[]{return false;});
        rejects([&]{unready.loadBuffer(sio,{});});
        const uint8_t option=1;rejects([&]{services.loadBuffer(sio,{&option,1});});
        check(services.loadBuffer(sio,{}).id==1 && services.loadBuffer(mux,{}).id==2);
        check(services.loadBuffer(sio,{}).id==1);
        rejects([&]{services.loadBuffer(custom,{});});
        unsigned starts=0;
        vp2s::NativeServiceModules compiled(root,[]{return true;},[&](std::string_view name,std::span<const uint8_t> args){
            check((name=="Dbc_Manager.irx" || name=="ds2o_d.irx") && args.empty());++starts;return 0;
        });
        check(compiled.loadBuffer(sio,{}).id==1 && compiled.loadBuffer(mux,{}).id==2);
        check(compiled.loadBuffer(custom,{}).id==3 && compiled.loadBuffer(custom,{}).id==3 && starts==1);
        check(compiled.loadBuffer(image("ds2o_d.irx"),{}).id==4 && starts==2);
        sio[0]^=1;rejects([&]{services.loadBuffer(sio,{});});sio[0]^=1;
        services.reset();check(services.loadBuffer(mux,{}).id==1);
        std::puts("private source identities, provider admission/readiness/arguments, duplicate/reset isolation passed");
    }
    std::puts("native module path/buffer ABI, copied bytes, bounds, atomic batches, completion, lifecycle and isolation passed");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

#include "native_kernel.h"
#include "native_modules.h"
#include "bozo/ee/dispatcher.h"
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <fstream>
#include <iterator>
#include <cstdio>
namespace vp2s {
ModuleResult NativeServiceModules::loadBuffer(std::span<const uint8_t> buffer,std::span<const uint8_t> args) {
    for(const char *name:{"sio2man.irx","sio2d.irx","ds2o_d.irx","mcman.irx","mcserv.irx",
                         "Sound_Device_Library.irx","Csi_.irx","Csd_.irx","Csm_.irx","Dbc_Manager.irx"}) {
        std::ifstream file(root/name,std::ios::binary);
        std::vector<uint8_t> original{std::istreambuf_iterator<char>(file),{}};
        if(original.empty() || buffer.size()!=((original.size()+15)&~size_t(15)) ||
           !std::equal(original.begin(),original.end(),buffer.begin()))continue;
        // These two libraries only transport controller/card transactions.
        // The native EE services own those transactions. The later custom
        // Dbc_Manager protocol is a separate provider and still rejects.
        if(std::string_view(name)!="sio2man.irx" && std::string_view(name)!="sio2d.irx")
            throw std::runtime_error(std::string("unported native service module: ")+name);
        if(!ready || !ready())throw std::runtime_error("native pad/card service is not installed");
        if(!args.empty())throw std::runtime_error("unported native system-module arguments");
        auto [it,inserted]=services.try_emplace(name,static_cast<int32_t>(services.size()+1));
        std::fprintf(stderr,"[vp2s:module] native pad/card service owns %s id=%d\n",name,it->second);
        return {it->second,0};
    }
    throw std::runtime_error("unidentified native service module buffer");
}
uint32_t ModuleBuffers::allocate(uint32_t bytes) {
    if(!bytes || bytes>64u*1024*1024 || uint64_t(next)+bytes+16>=0x70000000ull)
        throw std::runtime_error("invalid/exhausted native module buffer allocation");
    const auto handle=next;std::vector<uint8_t> data(bytes);
    buffers.emplace(handle,std::move(data));next+=((bytes+15)&~15u)+16;
    return handle;
}
bool ModuleBuffers::release(uint32_t h) {return buffers.erase(h)!=0;}
std::span<const uint8_t> ModuleBuffers::view(uint32_t h) const {
    auto it=buffers.find(h);
    if(it==buffers.end())throw std::runtime_error("unknown native module buffer handle");
    return it->second;
}
void ModuleBuffers::validate(uint32_t dest,size_t size) const {
    auto it=buffers.upper_bound(dest);
    if(it==buffers.begin())throw std::runtime_error("native transfer has no destination buffer");
    --it;const uint64_t offset=uint64_t(dest)-it->first;
    if(offset>it->second.size() || size>it->second.size()-offset)
        throw std::runtime_error("native transfer exceeds destination buffer");
}
void ModuleBuffers::copy(uint32_t dest,std::span<const uint8_t> data) {
    validate(dest,data.size());auto it=std::prev(buffers.upper_bound(dest));
    if(data.empty())return;
    std::memcpy(it->second.data()+dest-it->first,data.data(),data.size());
}
uint32_t ModuleBuffers::completeTransfer() {
    if(transferId==0x7fffffffu)throw std::runtime_error("native transfer handles exhausted");
    return ++transferId;
}
void ModuleBuffers::reset(){buffers.clear();next=0x40000000;transferId=0;}

namespace {
ModuleOwner &ownerFor(Runtime *rt,R5900Context *ctx) {
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *o=d?dynamic_cast<ModuleOwner *>(&d->host()):nullptr;
    if(!ctx || !o)throw std::runtime_error("native module service requires per-run ownership");
    return *o;
}
}
void loadNativeModule(uint8_t *,R5900Context *ctx,Runtime *rt) {
    auto *d=rt ? dynamic_cast<bozo::ee::Dispatcher *>(rt->services) : nullptr;
    auto *owner=d ? dynamic_cast<ModuleOwner *>(&d->host()) : nullptr;
    if(!ctx || !owner || (ctx->pc!=0x1401b8 && ctx->pc!=0x13fba8))
        throw std::runtime_error("native SDK module loading requires a per-run service owner");
    const bool buffer=ctx->pc==0x13fba8;
    if(!buffer && ctx->r[8].uw[0]!=0)throw std::runtime_error("unported asynchronous native SDK module load mode");
    const uint32_t pathAt=ctx->r[4].uw[0],bytes=ctx->r[5].uw[0],argsAt=ctx->r[6].uw[0],resultAt=ctx->r[7].uw[0];
    auto *out=rt->memory.hostPointer(resultAt,4);
    const auto *args=bytes?rt->memory.hostPointer(argsAt,bytes):nullptr;
    if(!out || bytes>252 || (bytes && !args))throw std::runtime_error("invalid native SDK module argument/result buffer");
    std::string path;
    for(uint32_t i=0;!buffer && i<252;++i) {
        if(pathAt>0xffffffffu-i)throw std::runtime_error("invalid native SDK module path");
        const auto *c=rt->memory.hostPointer(pathAt+i,1);
        if(!c)throw std::runtime_error("invalid native SDK module path");
        if(!*c)break;
        path.push_back(static_cast<char>(*c));
        if(i==251)throw std::runtime_error("native SDK module path too long");
    }
    if(!buffer && path.empty())throw std::runtime_error("empty native SDK module path");
    const auto result=buffer?owner->loadNativeModuleBuffer(pathAt,{args,bytes}):owner->loadNativeModule(path,{args,bytes});
    std::memcpy(out,&result.startResult,4);
    ctx->r[2].sd[0]=result.id;ctx->r[2].ud[1]=0;ctx->pc=ctx->r[31].uw[0];
}
void nativeModuleTransfer(uint8_t *,R5900Context *ctx,Runtime *rt) {
    auto &owner=ownerFor(rt,ctx);auto &buffers=owner.moduleBuffers();
    const uint32_t count=ctx->r[5].uw[0];
    if(count>32)throw std::runtime_error("native module transfer batch too large");
    struct Copy {uint32_t dest;std::vector<uint8_t> data;};std::vector<Copy> batch;
    for(uint32_t i=0;i<count;++i) {
        const auto address=uint64_t(ctx->r[4].uw[0])+uint64_t(i)*16;
        auto *p=address<=0xffffffffu?rt->memory.hostPointer(static_cast<uint32_t>(address),16):nullptr;
        if(!p)throw std::runtime_error("invalid native module transfer descriptor");
        uint32_t words[4];std::memcpy(words,p,16);
        if(words[3]!=0)throw std::runtime_error("unported native module transfer attributes");
        auto *source=rt->memory.hostPointer(words[0],words[2]);
        if(!source && words[2])throw std::runtime_error("invalid native module transfer source");
        buffers.validate(words[1],words[2]);
        batch.push_back({words[1],words[2]?std::vector<uint8_t>(source,source+words[2]):std::vector<uint8_t>{}});
    }
    const auto token=count?buffers.completeTransfer():0;
    for(const auto &copy:batch)buffers.copy(copy.dest,copy.data);
    if(count)dynamic_cast<bozo::ee::Dispatcher &>(*rt->services).kernel().irq(true,5);
    ctx->r[2].sd[0]=token;ctx->r[2].ud[1]=0;ctx->pc=ctx->r[31].uw[0];
}
void nativeModuleTransferStatus(uint8_t *,R5900Context *ctx,Runtime *rt) {
    ownerFor(rt,ctx); // Native copies finish before their real job handle returns.
    ctx->r[2].sd[0]=-1;ctx->r[2].ud[1]=0;ctx->pc=ctx->r[31].uw[0];
}
void freeNativeModuleBuffer(uint8_t *,R5900Context *ctx,Runtime *rt) {
    const bool freed=ownerFor(rt,ctx).moduleBuffers().release(ctx->r[4].uw[0]);
    ctx->r[2].sd[0]=freed?0:-1;ctx->r[2].ud[1]=0;ctx->pc=ctx->r[31].uw[0];
}
}

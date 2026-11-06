#include "native_modules.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/libs/libraries.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
namespace {
void check(bool ok){if(!ok)throw std::runtime_error("native EE reply assertion failed");}
template<class F>void rejects(F f){bool rejected=false;try{f();}catch(const std::runtime_error &){rejected=true;}check(rejected);}
struct Host:bozo::ee::Host {
    bozo::libs::Libraries libraries;
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("library");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
void callback(uint8_t *,R5900Context *ctx,Runtime *rt){
    bozo::libs::Guest guest(*rt,ctx);
    check(guest.arg(1)==0x555 && ctx->r[28].uw[0]==0x12345678);
    for(unsigned i=0;i<32;++i)check(guest.read8(guest.arg(0)+i)==uint8_t(i*13));
    guest.write32(0x150100,guest.read32(0x150100)+1);ctx->pc=ctx->r[31].uw[0];
}
struct Fixture {
    Host host;bozo::ee::Dispatcher dispatcher{host};vp2s::NativeCommandReplies replies;
    Fixture(){dispatcher.boot(0x100000,0x1fff000);host.libraries.attach(dispatcher);host.libraries.heap.reset(0x200000,0x210000);
        dispatcher.runtime().functions.setOverride(0x160000,callback);
        R5900Context context{};bozo::libs::Guest guest(dispatcher.runtime(),&context);
        guest.write32(0x1df82c,0x150000);guest.write32(0x1df830,3);
        guest.write32(0x15000c,0x160000);guest.write32(0x150010,0x555);guest.write32(0x150014,0x12345678);
    }
    void run(){
        R5900Context context{};bozo::libs::Guest guest(dispatcher.runtime(),&context);
        std::vector<uint8_t> data(32),extra(16,0x77);for(unsigned i=0;i<32;++i)data[i]=uint8_t(i*13);
        rejects([&]{replies.submit(host.libraries,3,data,0x151000,extra);});check(host.libraries.heap.blocks()==0);
        rejects([&]{replies.submit(host.libraries,1,data,0x11004000,extra);});check(host.libraries.heap.blocks()==0);
        check(replies.submit(host.libraries,1,data,0x151000,extra)==1);
        std::fill(data.begin(),data.end(),0xff);check(guest.read32(0x150100)==0 && host.libraries.heap.blocks()==1);
        for(unsigned i=0;i<16;++i)check(guest.read8(0x151000+i)==0x77);
        auto &thread=*dispatcher.kernel().thread(1);auto &frame=dispatcher.kernel().stackHandler(thread);
        check(frame.pc==0x160000 && frame.kind==bozo::ee::kInvokeSifCommand);
        callback(nullptr,&frame.ctx,&dispatcher.runtime());
        auto completion=host.libraries.takeHandlerEnd(frame.sequence);check(bool(completion));
        bozo::libs::Guest handlerGuest(dispatcher.runtime(),&frame.ctx);completion(host.libraries,handlerGuest);
        dispatcher.kernel().endHandler(thread);
        check(guest.read32(0x150100)==1 && host.libraries.heap.blocks()==0 && !dispatcher.kernel().handlerQueued());
    }
};
}
int main(){try{Fixture first,second;first.run();second.run();first.replies.reset();
    std::puts("native EE reply actual callback/data/extra/GP/once/lifetime/invalid target/two hosts passed");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

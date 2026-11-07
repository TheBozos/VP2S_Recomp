#include "bozo/ee/dispatcher.h"
#include "install_original_functions.h"
#include "install_adapted_functions.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
namespace {
void check(bool value){if(!value)throw std::runtime_error("graphics install retirement assertion failed");}
struct Host:bozo::ee::Host {
    unsigned enqueues=0;
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *c,uint32_t pc)override{
        check(pc==0x201f20 && c->r[4].uw[0]==0x180000 && c->r[5].uw[0]==0x190000);
        ++enqueues;c->r[2].sd[0]=1;c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("unexpected VU execution");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("unexpected device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("unexpected device write");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host oldHost,nativeHost;bozo::ee::Dispatcher old(oldHost),native(nativeHost);
    old.setFunctions(install_original_function_table,install_original_function_count,install_original_resume_table,install_original_resume_count);
    native.setFunctions(install_adapted_function_table,install_adapted_function_count,install_adapted_resume_table,install_adapted_resume_count);
    old.boot(0x29e2e0,0x1fff000);native.boot(0x29e2e0,0x1fff000);old.bindLibrary(0x201f20);
    for(unsigned seed=0;seed<64;++seed){
        auto *a=old.runtime().memory.flat(),*b=native.runtime().memory.flat();
        std::memset(a,seed+1,bozo::kFlatBytes);uint32_t queue=0x180000;std::memcpy(a+0x35dde0,&queue,4);
        std::memcpy(b,a,bozo::kFlatBytes);
        R5900Context x{},y{};
        for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x1234000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        x.r[4].ud[0]=0x190000;x.r[29].ud[0]=0x1fff000;x.r[31].ud[0]=0x170000;y=x;
        old.runtime().functions.generated(0x29e2e0)(a,&x,&old.runtime());
        native.runtime().functions.generated(0x29e2e0)(b,&y,&native.runtime());
        check(x.pc==0x170000 && y.pc==x.pc && oldHost.enqueues==seed+1 && nativeHost.enqueues==0);
        check(b[0x190004]==0 && b[0x190005]==255);
        check(!std::memcmp(a,b,bozo::kFlatBytes));
        // The void producer's enqueue result and transient call ra are dead in
        // its only audited caller. No completion callback/state is synthesized.
        x.r[2]=y.r[2];check(!std::memcmp(x.r,y.r,sizeof(x.r)));
    }
    std::puts("64 original/retired full-RAM producer comparisons passed; no queue or device completion synthesized");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

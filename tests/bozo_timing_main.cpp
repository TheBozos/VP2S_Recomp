// Compare the real statically compiled prefix at its first I/O with the exact
// production adaptation at its following call. Neither boundary is answered.
#include "bozo/ee/dispatcher.h"
#include "timing_original_functions.h"
#include "timing_adapted_functions.h"
#include <cstring>
#include <cstdio>
#include <stdexcept>
namespace {
void check(bool value) { if(!value) throw std::runtime_error("timing-prefix comparison failed"); }
struct Boundary {};
struct Host final : bozo::ee::Host {
    bool adapted=false; unsigned stops=0;
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t pc) override {
        check(adapted && pc==0x1211d0); ++stops; throw Boundary{};
    }
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t address,uint32_t size,const uint64_t value[2]) override {
        check(!adapted && address==0x10000010 && size==4 && value[0]==0x83); ++stops; throw Boundary{};
    }
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2]) override { throw std::runtime_error("read"); }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t) override { throw std::runtime_error("VU"); }
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s) override { throw std::runtime_error(s); }
};
}
int main() {
 try {
    for(unsigned seed=0;seed<32;++seed) {
        Host beforeHost,afterHost; afterHost.adapted=true;
        bozo::ee::Dispatcher before(beforeHost),after(afterHost);
        before.setFunctions(timing_original_function_table,timing_original_function_count,timing_original_resume_table,timing_original_resume_count);
        after.setFunctions(timing_adapted_function_table,timing_adapted_function_count,timing_adapted_resume_table,timing_adapted_resume_count);
        after.bindLibrary(0x1211d0);
        before.boot(0x1002c0,0x1fff000); after.boot(0x1002c0,0x1fff000);
        auto &a=before.runtime(); auto &b=after.runtime();
        std::memset(a.memory.flat(),0x41+seed,bozo::kFlatBytes);
        std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x,y;
        for(unsigned r=1;r<32;++r) { x.r[r].ud[0]=0xaaff001122330000ull+seed*32+r; x.r[r].ud[1]=~x.r[r].ud[0]; }
        x.pc=0x1002c0; y=x;
        try { a.functions.generated(0x1002c0)(a.memory.flat(),&x,&a); check(false); } catch(const Boundary &) {}
        try { b.functions.generated(0x1002c0)(b.memory.flat(),&y,&b); check(false); } catch(const Boundary &) {}
        check(beforeHost.stops==1 && afterHost.stops==1);
        check(std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes)==0);
        for(unsigned r=0;r<31;++r) check(std::memcmp(&x.r[r],&y.r[r],sizeof(x.r[r]))==0);
        check(y.r[31].uw[0]==0x100304);
        uint64_t magic; std::memcpy(&magic,b.memory.ram()+0x1ff0000,8);
        check(magic!=0 && magic!=0x4141414141414141ull);
    }
    std::puts("32 original/adapted complete-RAM timing prefix comparisons passed"); return 0;
 } catch(const std::exception &e) { std::fprintf(stderr,"%s\n",e.what()); return 1; }
}

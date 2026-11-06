#include "native_bootstrap.h"
#include "bozo/ee/dispatcher.h"
#include <fstream>
#include <cstdio>
#include <stdexcept>
namespace {
void check(bool ok){if(!ok)throw std::runtime_error("native startup derivation ABI assertion failed");}
struct Host:bozo::ee::Host,vp2s::BootstrapOwner {
    vp2s::bootstrap::Tables tables{};
    const vp2s::bootstrap::Tables &bootstrapTables()const override{return tables;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("library");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(int argc,char **argv){try{
    if(argc!=3)throw std::runtime_error("private tables and captured startup record required");
    Host first,second;char magic[4];uint32_t size=0;
    std::ifstream table(argv[1],std::ios::binary),record(argv[2],std::ios::binary);
    check(bool(table.read(magic,4)) && !std::memcmp(magic,"VPBK",4) && bool(table.read(reinterpret_cast<char *>(&size),4)) && size==sizeof(first.tables));
    check(bool(table.read(reinterpret_cast<char *>(&first.tables),sizeof(first.tables))));second.tables=first.tables;
    vp2s::bootstrap::Words seed{},key{};check(bool(record.read(reinterpret_cast<char *>(seed.data()),16)) && bool(record.read(reinterpret_cast<char *>(key.data()),16)));
    unsigned cases=0;
    for(auto *host:{&first,&second}){bozo::ee::Dispatcher dispatcher(*host);auto &rt=dispatcher.runtime();
        for(unsigned test=0;test<32;++test){R5900Context ctx{};
            for(unsigned i=1;i<32;++i){ctx.r[i].ud[0]=0x1234000000000000ull+i+test*32;ctx.r[i].ud[1]=~ctx.r[i].ud[0];}
            const uint32_t at=0x180000+test*16;ctx.r[16].uw[0]=at;ctx.pc=0x10090c;
            std::memcpy(rt.memory.ram()+at,seed.data(),16);
            auto expected=ctx;std::memcpy(&expected.r[2],key.data(),16);expected.r[9].ud[0]=0;
            const unsigned rounding=_mm_getcsr();
            check(vp2s::nativeBootstrapKey(nullptr,&ctx,&rt,0x10090c));
            check(!std::memcmp(&ctx,&expected,sizeof(ctx)) && !std::memcmp(rt.memory.ram()+at,key.data(),16) && _mm_getcsr()==rounding);++cases;
        }
        for(uint32_t at:{0u,0x1fffff8u,0x11004000u,0x40000000u}){
            R5900Context ctx{};ctx.r[16].uw[0]=at;bool rejected=false;
            try{vp2s::nativeBootstrapKey(nullptr,&ctx,&rt,0x10090c);}catch(const std::runtime_error &){rejected=true;}check(rejected);
        }
    }
    std::printf("%u native startup ABI/private reference/FP-rounding/independent-host cases passed; private values suppressed\n",cases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

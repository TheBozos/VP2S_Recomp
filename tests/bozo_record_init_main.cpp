#include "bozo/ee/dispatcher.h"
#include "record_init_functions.h"
#include <cstdio>
#include <cstring>
#include <source_location>
#include <vector>
#include <stdexcept>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("record init assertion at "+std::to_string(at.line()));}
constexpr uint32_t records=0x200000,root=0x190000,object=0x1a0000;
struct Host:bozo::ee::Host {
    struct Done{};unsigned seed=0,initialized=0,flushes=0,polled=0,updates=0,renders=0,rootLookups=0,objectLookups=0;
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        auto &rt=d.runtime();const auto a0=c->r[4].uw[0],a1=c->r[5].uw[0];
        if(pc==0x3d77cc)throw Done{};
        if(pc==0x2f8770){
            check(initialized<36 && a0==records+0x140*initialized && a1==0 && c->r[6].uw[0]==0xffffffff);
            for(unsigned k=0;k<80;++k){const uint32_t v=seed*100000+initialized*80+k;std::memcpy(rt.memory.ram()+a0+4*k,&v,4);}++initialized;
        }else if(pc==0x137460){check(a0==0 && initialized==flushes+1);++flushes;}
        else if(pc==0x1118e0){check(a0==root && a1==0xffffffff && polled+1==initialized);++polled;c->r[2].ud[0]=(seed+polled)&1;}
        else if(pc==0x10d0f0){++rootLookups;c->r[2].ud[0]=root;}
        else if(pc==0x101080){check(a0==root);++objectLookups;c->r[2].ud[0]=object;}
        else if(pc==0x10e820){check(a0==object && a1==1);++updates;}
        else if(pc==0x10e290){check(a0==object);++renders;}
        else throw std::runtime_error("unexpected record init callback");
        c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("unexpected device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("unexpected device write");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &why)override{throw std::runtime_error(why);}
};
}
int main(){try{
    for(unsigned seed=0;seed<32;++seed){Host host;host.seed=seed;bozo::ee::Dispatcher d(host);
        d.setFunctions(record_init_function_table,record_init_function_count,record_init_resume_table,record_init_resume_count);
        d.boot(0x3d7708,0x1fff000);for(const auto pc:{0x2f8770u,0x137460u,0x1118e0u,0x10d0f0u,0x101080u,0x10e820u,0x10e290u,0x3d77ccu})d.bindLibrary(pc);
        auto &rt=d.runtime();std::memset(rt.memory.flat(),0x67,bozo::kFlatBytes);std::memcpy(rt.memory.ram()+0x35ddb0,&root,4);
        auto expected=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
        for(unsigned i=0;i<36;++i)for(unsigned k=0;k<80;++k){const uint32_t value=seed*100000+i*80+k;std::memcpy(expected.data()+records+0x140*i+4*k,&value,4);}
        R5900Context c{};c.r[2].ud[0]=records;c.r[29].ud[0]=0x1fff000;c.r[31].ud[0]=0x170000;
        bool done=false;try{rt.functions.generated(0x3d7708)(rt.memory.flat(),&c,&rt);}catch(const Host::Done &){done=true;}
        check(done && host.initialized==36 && host.flushes==36 && host.polled==36 && host.updates==18 && host.renders==18);
        check(host.rootLookups==36 && host.objectLookups==36 && c.r[16].uw[0]==36 && c.r[21].uw[0]==records);
        check(!std::memcmp(expected.data(),rt.memory.flat(),expected.size()));
    }
    std::puts("32 compiled record-initialization loop cases: all1152 constructor/visibility/UI-predicate calls,576 UI updates, whole RAM and loop state match independent expected work; zero device I/O");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

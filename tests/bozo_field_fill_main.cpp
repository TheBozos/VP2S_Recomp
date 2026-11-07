#include "bozo/ee/dispatcher.h"
#include "field_fill_functions.h"
#include <cstdio>
#include <cstring>
#include <source_location>
#include <stdexcept>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("field fill assertion at "+std::to_string(at.line()));}
constexpr uint32_t heap=0x190000,buffer=0x200000;
struct Host:bozo::ee::Host {
    bool fail=false,allocated=false,filled=false,freed=false;unsigned descriptors=0,flushes=0,resets=0;uint8_t fill=0;
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        const auto a0=c->r[4].uw[0],a1=c->r[5].uw[0],a2=c->r[6].uw[0];auto &rt=d.runtime();
        if(pc==0x100cb0)c->r[2].ud[0]=heap;
        else if(pc==0x118520){check(a0==heap && a1==0x40000 && a2==0x80 && !allocated);allocated=true;c->r[2].ud[0]=fail?0:buffer;}
        else if(pc==0x132e40){check(!fail && allocated && !filled && a0==buffer && a1==fill && a2==0x40000);std::memset(rt.memory.ram()+buffer,fill,0x40000);filled=true;}
        else if(pc==0x1228d0){
            check(filled && !freed && a1==descriptors*0x400 && a2==4 && c->r[7].uw[0]==1 && !c->r[8].uw[0] && !c->r[9].uw[0] && c->r[10].uw[0]==256 && c->r[11].uw[0]==256);++descriptors;
        }else if(pc==0x137460){check(a0==0 && descriptors==flushes+1 && !freed);++flushes;}
        else if(pc==0x2b8f90){check(filled && !freed && descriptors==flushes);++resets;}
        else if(pc==0x100be0){check(a0==buffer && resets==1 && !freed);for(unsigned i=0;i<0x40000;++i)check(rt.memory.ram()[buffer+i]==fill);freed=true;}
        else throw std::runtime_error("unexpected physical fill upload/sync callback");
        c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("I/O");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("I/O");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &why)override{throw std::runtime_error(why);}
};
}
int main(){try{
    for(unsigned seed=0;seed<32;++seed){Host host;host.fail=seed&4;host.fill=uint8_t(seed*17);bozo::ee::Dispatcher d(host);
        d.setFunctions(field_fill_function_table,field_fill_function_count,field_fill_resume_table,field_fill_resume_count);d.boot(0x3d5670,0x1fff000);
        for(const auto pc:{0x1225b8u,0x122bf8u,0x100cb0u,0x118520u,0x132e40u,0x1228d0u,0x137460u,0x2b8f90u,0x100be0u})d.bindLibrary(pc);
        auto &rt=d.runtime();std::memset(rt.memory.flat(),0x63,bozo::kFlatBytes);const uint32_t dimensions=0x1a0000;std::memcpy(rt.memory.ram()+0x18000c,&dimensions,4);
        const int16_t width=int16_t(384+64*(seed%4)),height=int16_t(224+112*(seed%3));
        std::memcpy(rt.memory.ram()+dimensions+0x3c6c,&width,2);std::memcpy(rt.memory.ram()+dimensions+0x3c6e,&height,2);
        R5900Context c{};c.r[4].ud[0]=0x180000;c.r[5].ud[0]=host.fill;c.r[29].ud[0]=0x1fff000;c.r[31].ud[0]=0x170000;
        rt.functions.generated(0x3d5670)(rt.memory.flat(),&c,&rt);check(c.pc==0x170000 && c.r[29].uw[0]==0x1fff000 && host.allocated);
        const unsigned count=(unsigned(width)*unsigned(height)*8+0x118000)/0x40000+1;
        check(host.descriptors==(host.fail?0:count) && host.flushes==host.descriptors && host.resets==(host.fail?0:1) && host.freed==!host.fail);
    }
    std::puts("32 original field-fill CPU cases: allocation failure,262144-byte fills, exact descriptor/visibility work, reset-before-free lifetime, zero physical upload/sync or device I/O");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

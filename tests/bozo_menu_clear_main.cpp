#include "bozo/ee/dispatcher.h"
#include "menu_clear_original_functions.h"
#include "menu_clear_adapted_functions.h"
#include <array>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>
namespace {
void check(bool b){if(!b)throw std::runtime_error("menu physical-clear comparison failed");}
struct Call {uint32_t pc;std::array<uint32_t,8> args;bool operator==(const Call &)const=default;};
struct Host:bozo::ee::Host {
    bool original=false,failAllocation=false;unsigned uploads=0;std::vector<Call> calls;
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        if(pc==0x122bf8){check(original && c->r[5].uw[0]==0x240000);++uploads;c->r[2].sd[0]=0;}
        else {
            Call call{pc,{}};for(unsigned i=0;i<8;++i)call.args[i]=c->r[4+i].uw[0];calls.push_back(call);
            if(pc==0x100cb0)c->r[2].sd[0]=0x250000;
            else if(pc==0x118520){check(call.args[0]==0x250000 && call.args[1]==0x40000 && call.args[2]==128);c->r[2].sd[0]=failAllocation?0:0x240000;}
            else if(pc==0x132e40){check(call.args[0]==0x240000 && call.args[2]==0x40000);std::memset(d.runtime().memory.ram()+0x240000,call.args[1],0x40000);c->r[2].sd[0]=0x240000;}
            else if(pc==0x1228d0){
                check(call.args[2]==4 && call.args[3]==1 && call.args[4]==0 && call.args[5]==0 && call.args[6]==256 && call.args[7]==256);
                std::memcpy(d.runtime().memory.ram()+call.args[0],call.args.data(),sizeof(call.args));c->r[2].sd[0]=0;
            }else if(pc==0x100be0){check(call.args[0]==0x240000);c->r[2].sd[0]=0;}
            else if(pc==0x1225b8 || pc==0x137460)c->r[2].sd[0]=0;
            else throw std::runtime_error("unexpected menu-clear leaf");
        }
        c->pc=c->r[31].uw[0];
    }
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device write");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host aHost,bHost;aHost.original=true;bozo::ee::Dispatcher a(aHost),b(bHost);
    a.setFunctions(menu_clear_original_function_table,menu_clear_original_function_count,menu_clear_original_resume_table,menu_clear_original_resume_count);
    b.setFunctions(menu_clear_adapted_function_table,menu_clear_adapted_function_count,menu_clear_adapted_resume_table,menu_clear_adapted_resume_count);
    for(auto *d:{&a,&b})for(uint32_t pc:{0x1225b8u,0x100cb0u,0x118520u,0x132e40u,0x1228d0u,0x137460u,0x122bf8u,0x100be0u})d->bindLibrary(pc);
    unsigned count=0;
    for(unsigned seed=0;seed<64;++seed){
        auto *x=a.runtime().memory.flat(),*y=b.runtime().memory.flat();std::memset(x,0x31+seed,bozo::kFlatBytes);
        const uint32_t control=0x190000;std::memcpy(x+0x180004,&control,4);
        const int16_t w=seed&1?640:512,h=seed&2?480:448;
        std::memcpy(x+control+0x3c6c,&w,2);std::memcpy(x+control+0x3c6e,&h,2);std::memcpy(y,x,bozo::kFlatBytes);
        R5900Context before{},after{};
        for(unsigned i=1;i<32;++i){before.r[i].ud[0]=0x1234000000000000ull+seed*32+i;before.r[i].ud[1]=~before.r[i].ud[0];}
        std::memset(before.vu0_vf,0xd3,sizeof(before.vu0_vf));before.r[4].ud[0]=0x180000;before.r[5].ud[0]=seed*7;
        before.r[29].ud[0]=0x1fff000;before.r[31].ud[0]=0x170000;after=before;
        aHost.failAllocation=bHost.failAllocation=(seed&4)!=0;aHost.calls.clear();bHost.calls.clear();aHost.uploads=bHost.uploads=0;
        a.runtime().functions.generated(0x360dd0)(x,&before,&a.runtime());b.runtime().functions.generated(0x360dd0)(y,&after,&b.runtime());
        check(before.pc==0x170000 && bHost.uploads==0 && aHost.calls==bHost.calls && !std::memcmp(x,y,bozo::kFlatBytes));
        check(aHost.failAllocation?aHost.uploads==0:aHost.uploads>0);
        check(before.cycles==after.cycles+aHost.uploads*8 && before.checkpoints==after.checkpoints+aHost.uploads);
        before.cycles-=aHost.uploads*8;before.checkpoints-=aHost.uploads;
        check(!std::memcmp(&before,&after,sizeof(before)));++count;
    }
    std::printf("%u whole menu-clear CPU/RAM comparisons, including allocation failure; exact retired-call scheduling delta; no physical upload/completion\n",count);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

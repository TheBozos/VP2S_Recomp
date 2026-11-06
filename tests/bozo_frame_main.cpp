#include "native_frame.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/libs/libraries.h"
#include "bozo/port.h"
#include "frame_original_functions.h"
#include "frame_adapted_functions.h"
#include <array>
#include <vector>
#include <cstring>
#include <cstdio>
#include <stdexcept>
namespace {
void check(bool ok){if(!ok)throw std::runtime_error("native frame-event comparison failed");}
struct Call {uint32_t pc;std::array<uint32_t,6> args;bool operator==(const Call &)const=default;};
struct Host:bozo::ee::Host,bozo::port::Host {
    bozo::libs::Libraries libs;std::vector<Call> calls;unsigned profile=0,boundaries=0;bool allowFrames=false;
    bozo::libs::Libraries &libraries()override{return libs;}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *t,uint32_t pc)override{return vp2s::nativeFrameInput(r,c,t,pc);}
    void frameEnd(Runtime *,R5900Context *)override{if(!allowFrames)throw std::runtime_error("unexpected frame publication");++boundaries;}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        Call call{pc,{}};for(unsigned i=0;i<6;++i)call.args[i]=c->r[4+i].uw[0];calls.push_back(call);
        switch(pc){
        case 0x114240:case 0x113a60:case 0x1035e0:case 0x119650:break;
        case 0x10d0f0:c->r[2].sd[0]=0x1c8d00;break;
        case 0x101060:c->r[2].sd[0]=int32_t(c->r[4].uw[0]+0xca24);break;
        case 0x1195e0:{const uint32_t message=profile&2?0x10:0x20;
            std::memcpy(d.runtime().memory.ram()+c->r[5].uw[0],&message,4);c->r[2].sd[0]=profile&1;break;}
        default:throw std::runtime_error("unexpected frame-event leaf");
        }
        c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("GS/device read was executed");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("GS/device write was executed");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
void word(uint8_t *ram,uint32_t at,uint32_t value){std::memcpy(ram+at,&value,4);}
}
int main(){try{
    unsigned cases=0;
    for(unsigned seed=0;seed<64;++seed)for(unsigned frame=0;frame<4;++frame){
        Host beforeHost,afterHost;beforeHost.profile=afterHost.profile=seed%4;
        bozo::ee::Dispatcher before(beforeHost),after(afterHost);
        before.setFunctions(frame_original_function_table,frame_original_function_count,frame_original_resume_table,frame_original_resume_count);
        after.setFunctions(frame_adapted_function_table,frame_adapted_function_count,frame_adapted_resume_table,frame_adapted_resume_count);
        for(auto *d:{&before,&after}){
            d->boot(0x11fb40,0x1fff000);d->kernel().processEvents(frame*d->kernel().field());
            for(uint32_t pc:{0x114240u,0x113a60u,0x10d0f0u,0x101060u,0x1035e0u,0x1195e0u,0x119650u})d->bindLibrary(pc);
        }
        before.runtime().port=&beforeHost;after.runtime().port=&afterHost;
        auto *a=before.runtime().memory.ram(),*b=after.runtime().memory.ram();
        std::memset(before.runtime().memory.flat(),0x30+seed,bozo::kFlatBytes);
        constexpr uint32_t object=0x180000,display=0x190000,stack=0x1fff000;
        a[object+0x2f9]=uint8_t(seed*7);a[object+0x2fb]=uint8_t(seed&1);a[object+0x2fc]=uint8_t((seed>>1)&1);
        word(a,object+0x324,display);word(a,object+0x250,seed*13);
        const uint32_t sizes[]={0u,1u,0x7fffffu,0x800000u,0xffffffffu,0x12345u};
        word(a,display+0x3e40,sizes[seed%6]);word(a,object+0x300,seed&4?0xffffffff:seed*31);
        word(a,object+8,seed&8?0xffffffff:seed);word(a,object+12,seed);word(a,object+16,seed&16?0xffffffff:seed);
        std::memcpy(after.runtime().memory.flat(),before.runtime().memory.flat(),bozo::kFlatBytes);
        R5900Context x,y;
        for(unsigned i=1;i<32;++i){x.r[i].ud[0]=0x1234000000000000ull+seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        x.r[4].ud[0]=object;x.r[29].ud[0]=stack;x.r[31].ud[0]=0x170000;y=x;
        // Reference begins at the already-decoded logical phase. Its retired
        // load/extraction are never called or answered. Reproduce only prologue.
        x.r[29].ud[0]=stack-0x50;x.r[1].ud[0]=0x12000000;x.r[2].ud[0]=before.kernel().vblankCount()&1;
        std::memcpy(a+stack-0x50,&x.r[16],16);std::memcpy(a+stack-0x40,&x.r[31].ud[0],8);
        before.runtime().functions.generated(0x11fb58)(a,&x,&before.runtime());
        after.runtime().functions.generated(0x11fb40)(b,&y,&after.runtime());
        check(x.pc==0x170000 && y.pc==x.pc && beforeHost.calls==afterHost.calls);
        check(!std::memcmp(x.r,y.r,sizeof(x.r)));
        check(!std::memcmp(before.runtime().memory.flat(),after.runtime().memory.flat(),bozo::kFlatBytes));
        check(b[object+0x2fa]==(after.kernel().vblankCount()&1));
        afterHost.allowFrames=true;const auto saved=y;
        check(vp2s::nativeFrameInput(b,&y,&after.runtime(),0x11f62c) && afterHost.boundaries==1);
        check(!std::memcmp(&saved,&y,sizeof(y)));
        check(!std::memcmp(before.runtime().memory.flat(),after.runtime().memory.flat(),bozo::kFlatBytes));++cases;
    }
    std::printf("%u original/native full-RAM frame-event bookkeeping and ordered leaf-call comparisons passed; no GS read answered\n",cases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

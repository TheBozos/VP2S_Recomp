#include "bozo/native/compiled_irx.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>
#include <array>
using namespace bozo::native::irx;
void vp2CsmTimerReference(Env &);
void vp2CsmTimerNative(Env &);
namespace {
struct Fixture:Env {
    NativeCpu state{};
    std::vector<uint8_t> data=std::vector<uint8_t>(0x200000);
    std::vector<std::array<uint32_t,5>> calls;
    unsigned fail=0;
    Fixture(unsigned seed,unsigned failure):Env(state),fail(failure){ram=data.data();ramSize=uint32_t(data.size());
        for(auto &v:data){seed=seed*1664525u+1013904223u;v=uint8_t(seed>>24);}
        for(auto &r:cpu.gpr){seed=seed*1664525u+1013904223u;r=seed;}
        cpu.gpr[0]=0;cpu.gpr[29]=0x100000;cpu.gpr[31]=0x12345678;cpu.pc=0x44978;budget=1000000;
    }
    bool call(uint32_t target,uint32_t ret)override{
        if(target!=0x457c4)calls.push_back({target,cpu.gpr[4],cpu.gpr[5],cpu.gpr[6],cpu.gpr[7]});
        switch(target){
        case 0x45788:cpu.gpr[2]=fail==1?uint32_t(-150):7;break;
        case 0x45790:cpu.gpr[2]=fail==2?uint32_t(-1):0;break;
        case 0x45798:cpu.gpr[2]=fail==3?uint32_t(-1):0;break;
        case 0x457c4:break; // Original display alignment has no game-data result.
        case 0x457a0:cpu.gpr[2]=fail==4?uint32_t(-1):0;break;
        case 0x457cc:cpu.gpr[2]=fail==5?uint32_t(-1):0;break;
        default:throw std::runtime_error("unexpected Csm startup call");
        }
        cpu.pc=ret;return true;
    }
    uint8_t read8(uint32_t a)override{return data.at(a);}
    uint16_t read16(uint32_t a)override{uint16_t v;std::memcpy(&v,&data.at(a),2);return v;}
    uint32_t read32(uint32_t a)override{uint32_t v;std::memcpy(&v,&data.at(a),4);return v;}
    void write8(uint32_t a,uint8_t v)override{data.at(a)=v;}
    void write16(uint32_t a,uint16_t v)override{std::memcpy(&data.at(a),&v,2);}
    void write32(uint32_t a,uint32_t v)override{std::memcpy(&data.at(a),&v,4);}
};
}
int main(){try{
    for(unsigned fail=0;fail<6;++fail)for(unsigned seed=1;seed<=16;++seed){
        Fixture reference(seed,fail),native(seed,fail);vp2CsmTimerReference(reference);vp2CsmTimerNative(native);
        if(reference.data!=native.data || reference.calls!=native.calls ||
           std::memcmp(&reference.cpu,&native.cpu,sizeof(NativeCpu)))
            throw std::runtime_error("Csm native timer startup state/call mismatch");
    }
    std::puts("96 original/native Csm timer-startup RAM/CPU/ordered-call comparisons passed; display alignment retired");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

#include "native_controller.h"
#include "bozo/native/compiled_irx.h"
#include <vector>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
void vp2ControllerReference(bozo::native::irx::Env &);
namespace {
void check(bool ok){if(!ok)throw std::runtime_error("native controller record mismatch");}
struct Reference final:bozo::native::irx::Env {
    std::vector<uint8_t> data=std::vector<uint8_t>(0x40000);
    unsigned submits=0;int32_t slot=0;
    Reference(bozo::native::irx::NativeCpu &c):Env(c){ram=data.data();ramSize=uint32_t(data.size());budget=10000;}
    uint8_t read8(uint32_t)override{throw std::runtime_error("reference outside data");}
    uint16_t read16(uint32_t)override{throw std::runtime_error("reference outside data");}
    uint32_t read32(uint32_t)override{throw std::runtime_error("reference outside data");}
    void write8(uint32_t a,uint8_t v)override{check(a<data.size());data[a]=v;}
    void write16(uint32_t a,uint16_t v)override{check(uint64_t(a)+2<=data.size());std::memcpy(data.data()+a,&v,2);}
    void write32(uint32_t a,uint32_t v)override{check(uint64_t(a)+4<=data.size());std::memcpy(data.data()+a,&v,4);}
    bool call(uint32_t target,uint32_t ret)override{
        // Scalar record assembly only. Observe critical-section/submission
        // boundaries without running a controller or transfer worker.
        if(target==0x28064){w32(cpu.gpr[4],0);cpu.gpr[2]=0;}
        else if(target==0x2806c){check(cpu.gpr[4]==0);cpu.gpr[2]=0;}
        else if(target==0x27d50){++submits;check(cpu.gpr[4]==1 && cpu.gpr[6]==0x28400);
            check(int32_t(cpu.gpr[5])==slot);cpu.gpr[2]=1;}
        else throw std::runtime_error("unexpected controller reference call");
        cpu.pc=ret;return true;
    }
};
}
int main(int argc,char **argv){try{
    check(argc==2);std::ifstream file(argv[1],std::ios::binary);
    const std::vector<uint8_t> image((std::istreambuf_iterator<char>(file)),{});check(image.size()==0x2700);
    unsigned cases=0;
    for(unsigned seed=0;seed<128;++seed)for(bool pressure:{false,true})for(bool connected:{false,true})
    for(uint32_t sequence:{0u,0x7fffffffu,0xffffffffu}){
        bozo::native::irx::NativeCpu cpu{};Reference ref(cpu);
        for(unsigned i=0;i<ref.data.size();++i)ref.data[i]=uint8_t(i*13+seed*7);
        std::copy(image.begin(),image.end(),ref.data.begin()+0x26000);
        constexpr uint32_t state=0x282d0,out=state+0x130;
        bozo::libs::PadInputState input;input.connected=connected;input.buttons=uint16_t(seed*517);
        input.rx=uint8_t(seed*3);input.ry=uint8_t(seed*5);input.lx=uint8_t(seed*7);input.ly=uint8_t(seed*11);
        for(unsigned i=0;i<12;++i)input.pressure[i]=uint8_t(seed*19+i*23);
        ref.w32(state+0x1b0,sequence);ref.w32(state+0x1b4,1);ref.w32(state+0x38,seed&1);
        ref.w8(state+0x3d,connected?1:0);ref.w8(state+0x3f,pressure?0x79:0x73);
        ref.w16(state+11,uint16_t(~input.buttons));ref.w8(state+13,input.rx);ref.w8(state+14,input.ry);
        ref.w8(state+15,input.lx);ref.w8(state+16,input.ly);
        for(unsigned i=0;i<12;++i)ref.w8(state+17+i,input.pressure[i]);
        vp2s::ControllerRecord native;std::copy_n(ref.data.begin()+out,128,native.begin());
        std::array<uint8_t,4> caps;std::copy_n(ref.data.begin()+(pressure?0x28218:0x28210),4,caps.begin());
        ref.slot=int32_t(sequence)%2;cpu.gpr[4]=state;cpu.gpr[29]=0x3f000;cpu.gpr[31]=0xdead;cpu.pc=0x2722c;
        vp2ControllerReference(ref);vp2s::produceControllerRecord(native,input,sequence,seed&1,pressure,caps);
        check(cpu.pc==0xdead && cpu.gpr[2]==1 && ref.submits==1);
        check(ref.r32(state+0x1b0)==sequence+1);check(!std::memcmp(native.data(),ref.data.data()+out,128));++cases;
    }
    std::printf("%u original/native complete controller records and submission slots passed; no hardware worker executed\n",cases);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

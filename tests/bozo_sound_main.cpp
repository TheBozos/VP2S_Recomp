#include "vp2_sound_image.h"
#include <cstdio>
#include <stdexcept>
namespace {
using namespace bozo::native::irx;
void check(bool ok){if(!ok)throw std::runtime_error("native VP2S sound provider mismatch");}
void invoke(Env &e,uint32_t address){e.cpu.gpr[31]=0x38004;check(e.call(address,0x38004));}
void consumer(Env &e){
    const auto caller=e.cpu.gpr[31];
    e.cpu.gpr[4]=0;invoke(e,0x38300);
    for(unsigned core=0;core<2;++core)for(unsigned voice=0;voice<24;++voice){
        const uint32_t entry=(voice<<1)|core,value=0x100+voice*127+core;
        e.cpu.gpr[4]=entry;e.cpu.gpr[5]=value;invoke(e,0x38308);
        e.cpu.gpr[4]=entry;invoke(e,0x38310);check(e.cpu.gpr[2]==value);
    }
    for(unsigned i=0;i<64;++i)e.w8(0x38100+i,uint8_t(i*31));
    e.cpu.gpr[4]=0;e.cpu.gpr[5]=0;e.cpu.gpr[6]=0x38100;e.cpu.gpr[7]=0x1000;
    e.w32(e.cpu.gpr[29]+16,64);invoke(e,0x38318);check(e.cpu.gpr[2]==64);
    e.cpu.gpr[4]=0;invoke(e,0x38320);check(e.cpu.gpr[2]==1);
    e.cpu.pc=caller;
}
const Entry entries[]={{0x38000,consumer}};
const Registration registration("sound-probe",0x50d,0x38000,0x400,entries,1);
void images(CompiledLibraries &h){
    addVp2SoundImage(h);
    h.addImage({"sound-probe",0x50d,0x38000,0x38000,0,std::vector<uint8_t>(0x400),
        {{0x38300,"libsd",4},{0x38308,"libsd",5},{0x38310,"libsd",6},
         {0x38318,"libsd",17},{0x38320,"libsd",19}},{},false});
}
void run(CompiledLibraries &h){
    check(h.startImage("Sound_Device_Library.irx;1")==1);
    check(h.startImage("sound-probe")==2);
    auto *mixer=h.advanceTo(0);uint8_t actual[64];mixer->read(0x1000,actual,64);
    for(unsigned i=0;i<64;++i)check(actual[i]==uint8_t(i*31));
    check(h.startImage("Sound_Device_Library.irx;1")==1);
}
}
int main(){try{
    bozo::libs::Libraries a,b;CompiledLibraries first(a,0x40000),second(b,0x40000);
    images(first);images(second);run(first);run(second);first.reset();run(first);
    std::puts("native VP2S libsd two-core voice parameters, copied mixer bytes, repeat/reset/isolation passed; soundtrack not qualified");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

// Native compiled module initialization probe. No devices, firmware or input.
#include "vp2_dbc_image.h"
#include <cstdio>
#include <stdexcept>
namespace {
using namespace bozo::native::irx;
void driverRegistration(Env &e){
    const auto caller=e.cpu.gpr[31];
    // Logical DBC driver descriptor, not a controller bus or a SIO2 response.
    e.w32(0x30100,1);e.w32(0x30104,0);e.w32(0x30108,0x30140);
    for(unsigned port=0;port<2;++port){
        e.cpu.gpr[4]=0x30100;e.cpu.gpr[31]=0x30004;
        if(!e.call(0x30200,0x30004) || e.cpu.gpr[2]!=port)
            throw std::runtime_error("compiled DBC driver registration mismatch");
        const uint32_t record=0x227c0+port*0x28;
        if(e.r32(record)!=1 || e.r32(record+4)!=1 || e.r32(record+8)!=0 ||
           e.r32(record+12)!=0x30140 || e.r32(record+16)!=0xffffffff ||
           e.r32(record+20)!=0xffffffff || e.r8(record+24)!=0)
            throw std::runtime_error("compiled DBC driver descriptor mismatch");
    }
    // No callback is implemented in this probe: any attempted use must fail.
    e.cpu.pc=caller;
}
const Entry entries[]={{0x30000,driverRegistration}};
const Registration registration("dbc-driver-probe",0xdbc1,0x30000,0x300,entries,1);
void addProbe(CompiledLibraries &host){
    host.addImage({"dbc-driver-probe",0xdbc1,0x30000,0x30000,0,
        std::vector<uint8_t>(0x300),{{0x30200,"dbcman",4}},{},false});
}
}
int main(){try{
    bozo::libs::Libraries a,b;
    bozo::native::irx::CompiledLibraries first(a,0x40000),second(b,0x40000);
    addVp2DbcImage(first);addVp2DbcImage(second);
    if(first.startImage("Dbc_Manager.irx;1")!=1 || first.nativeCalls()!=9)
        throw std::runtime_error("native Dbc_Manager initialization mismatch");
    if(first.startImage("Dbc_Manager.irx;1")!=1 || first.nativeCalls()!=9 || second.nativeCalls()!=0)
        throw std::runtime_error("native Dbc_Manager repeat/ownership mismatch");
    if(second.startImage("Dbc_Manager.irx;1")!=1 || second.nativeCalls()!=9)
        throw std::runtime_error("native Dbc_Manager second-host mismatch");
    addProbe(first);addProbe(second);
    if(first.startImage("dbc-driver-probe")!=2 || second.startImage("dbc-driver-probe")!=2)
        throw std::runtime_error("native DBC driver probe failed");
    const auto calls=first.nativeCalls();
    first.reset();
    if(first.startImage("Dbc_Manager.irx;1")!=1 || first.nativeCalls()!=calls+9 ||
       first.startImage("dbc-driver-probe")!=2)
        throw std::runtime_error("native Dbc_Manager reset mismatch");
    std::puts("native Dbc_Manager initialization/two-driver registration/repeat/reset/two-host isolation passed; controller data not qualified");
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"native Dbc_Manager boundary: %s\n",e.what());return 1;}}

#include "native_controller.h"
#include "vp2_dbc_image.h"
#include "vp2_controller_image.h"
#include <cstdio>
#include <stdexcept>
struct Input:bozo::libs::PadInput {
    bool connected=false;unsigned calls=0;
    void read(uint32_t,int port,bozo::libs::PadInputState &out)override{
        ++calls;out.connected=connected && port==0;out.buttons=bozo::libs::kPadCross;
    }
};
namespace {
using namespace bozo::native::irx;
void inspect(Env &){throw std::runtime_error("unbound controller inspection");}
const Entry inspectionEntries[]={{0x38000,inspect}};
const Registration inspection("controller-inspection",0xc017,0x38000,0x100,inspectionEntries,1);
}
int main(int argc,char **){try{
    bozo::libs::Libraries libs;Input input;libs.pad.input=&input;
    vp2s::ControllerBridge controller(libs);
    bozo::native::irx::CompiledLibraries host(libs,0x40000);
    addVp2DbcImage(host);addVp2ControllerImage(host);controller.bind(host);
    host.addImage({"controller-inspection",0xc017,0x38000,0x38000,0,std::vector<uint8_t>(0x100),{},{},false});
    host.bindNative(0x38000,[&](Env &e){
        for(unsigned port=0;port<2;++port){
            const uint32_t state=0x282d0+port*0x1bc,out=state+0x130;
            const bool active=input.connected && port==0;
            if(e.r32(state+0x1b0)==0 || e.r8(out)!=(active?1:0) ||
               (active && (e.r16(out+28)!=uint16_t(~bozo::libs::kPadCross) || e.r8(out+32)!=0x80)))
                throw std::runtime_error("native input producer data inspection failed");
        }
        e.cpu.pc=e.cpu.gpr[31];
    });
    host.startImage("Dbc_Manager.irx;1");
    if(argc>1)input.connected=true;
    host.startImage("ds2o_d.irx;1");
    if(controller.sampleCount()!=2 || input.calls!=2)throw std::runtime_error("native controller task initialization mismatch");
    host.startImage("controller-inspection");
    host.advanceTo(0);host.advanceTo(10000000);host.advanceTo(10000000);
    if(controller.sampleCount()<4)throw std::runtime_error("native input tasks did not resume");
    host.reset();controller.reset();host.startImage("Dbc_Manager.irx;1");host.startImage("ds2o_d.irx;1");
    if(controller.sampleCount()!=2)throw std::runtime_error("native controller reset mismatch");
    host.startImage("controller-inspection");
    std::puts("native controller task initialization/sample/delay/resume/reset passed");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"native controller bridge: %s\n",e.what());return 1;}}

#include "native_controller.h"
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <stdexcept>
namespace vp2s {
void produceControllerRecord(ControllerRecord &record,const bozo::libs::PadInputState &input,
    uint32_t sequence,uint32_t ready,bool pressure,const std::array<uint8_t,4> &capabilities){
    auto word=[&](unsigned offset,uint32_t value){for(unsigned i=0;i<4;++i)record[offset+i]=uint8_t(value>>(8*i));};
    word(124,sequence);record[0]=input.connected?1:0;
    if(!input.connected)return;
    std::fill(record.begin()+28,record.begin()+60,0);
    const unsigned bytes=pressure?18:6;
    record[1]=uint8_t(bytes+4);record[2]=uint8_t(bytes);record[3]=4;word(4,ready);
    const uint16_t released=uint16_t(~input.buttons);
    record[28]=uint8_t(released);record[29]=uint8_t(released>>8);
    record[30]=input.rx;record[31]=input.ry;record[32]=input.lx;record[33]=input.ly;
    if(pressure)std::copy(input.pressure.begin(),input.pressure.end(),record.begin()+34);
    std::copy(capabilities.begin(),capabilities.end(),record.begin()+28+bytes);
}
namespace {
using bozo::native::irx::Env;
void call(Env &e,uint32_t target){
    e.cpu.gpr[31]=0x26000;
    if(!e.call(target,0x26000))throw std::runtime_error("unexpected native controller initialization yield");
}
}
void ControllerBridge::bind(bozo::native::irx::CompiledLibraries &host){
    host.bindNative(0x26000,[this](Env &e){initialize(e);});
    host.bindNative(0x261dc,[this](Env &e){sample(e);});
    host.bindNative(0x26208,[this](Env &e){sample(e);});
}
void ControllerBridge::initialize(Env &e){
    const auto caller=e.cpu.gpr[31],stack=e.cpu.gpr[29];
    e.cpu.gpr[29]-=64;
    for(unsigned port=0;port<2;++port){
        // Keep the compiled game descriptor registration; replace the transport
        // producer with a native periodic input task. No event/VBlank/SIO2 worker.
        e.cpu.gpr[4]=port;call(e,0x27c10);
        if(e.cpu.gpr[2]!=1)throw std::runtime_error("native controller DBC registration failed");
        const uint32_t desc=e.cpu.gpr[29]+16;
        const uint32_t fields[]={0,port,0x261dc,0x800,0x2e};
        for(unsigned i=0;i<5;++i)e.w32(desc+i*4,fields[i]);
        e.cpu.gpr[4]=desc;call(e,0x2223c);
        const int32_t id=int32_t(e.cpu.gpr[2]);if(id<=0)throw std::runtime_error("native input task allocation failed");
        e.cpu.gpr[4]=uint32_t(id);e.cpu.gpr[5]=port;call(e,0x22244);
    }
    e.cpu.gpr[29]=stack;e.cpu.gpr[2]=0;e.cpu.pc=caller;
}
void ControllerBridge::sample(Env &e){
    auto &c=e.cpu;
    // Persistent task argument lives in a saved scalar ABI slot across the native delay.
    const unsigned port=c.pc==0x261dc?c.gpr[4]:c.gpr[16];
    if(port>=2)throw std::runtime_error("invalid native input task port");
    c.gpr[16]=port;
    bozo::libs::PadInputState input;
    if(libraries.pad.input)libraries.pad.input->read(libraries.framesEnded,int(port),input);
    else input.connected=port==0;
    if(input.buttons && std::getenv("VP2S_BOZO_TRACE_INPUT"))std::fprintf(stderr,"[vp2s:input] frame=%u port=%u buttons=%04x connected=%u\n",libraries.framesEnded,port,input.buttons,input.connected);
    ++samples;
    const uint32_t state=0x282d0+port*0x1bc,output=state+0x130;
    ControllerRecord record;std::copy_n(e.ramBase()+output,128,record.begin());
    std::array<uint8_t,4> capabilities;std::copy_n(e.ramBase()+0x28210,4,capabilities.begin());
    const auto sequence=e.r32(state+0x1b0);
    produceControllerRecord(record,input,sequence,1,false,capabilities);
    for(unsigned i=0;i<record.size();++i)e.w8(output+i,record[i]);
    e.w32(state+0x1b0,sequence+1);e.w32(state+0x1b4,port);
    e.w8(state+0x3d,input.connected?1:0);e.w8(state+0x3f,input.connected?0x73:0);e.w8(state+0x40,input.connected?0x73:0);
    if(input.connected){
        const auto handle=e.r32(0x286c0+port*4);
        const auto registered=e.r32(0x227c0+handle*0x28);
        if(registered==1){
            c.gpr[4]=port;c.gpr[5]=port;c.gpr[6]=0;c.gpr[31]=0x26208;
            if(!e.call(0x27cd4,0x26208))throw std::runtime_error("native controller open yielded");
            if(c.gpr[2]!=1)throw std::runtime_error("native controller DBC open failed");
        }
        c.gpr[4]=port;c.gpr[5]=uint32_t(int32_t(sequence)%2);c.gpr[6]=output;c.gpr[31]=0x26208;
        if(!e.call(0x27d50,0x26208))throw std::runtime_error("native controller delivery yielded");
        if(c.gpr[2]!=1)throw std::runtime_error("native controller DBC delivery failed");
    }
    c.gpr[4]=16667;c.gpr[31]=0x26208;
    (void)e.call(0x22254,0x26208);
}
}

#include "native_controller.h"
#include <algorithm>
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
}

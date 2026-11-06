// Native compiled module initialization probe. No devices, firmware or input.
#include "vp2_dbc_image.h"
#include <cstdio>
#include <stdexcept>
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
    first.reset();
    if(first.startImage("Dbc_Manager.irx;1")!=1 || first.nativeCalls()!=18)
        throw std::runtime_error("native Dbc_Manager reset mismatch");
    std::puts("native Dbc_Manager compiled initialization/repeat/reset/two-host isolation passed; controller protocol not qualified");
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"native Dbc_Manager boundary: %s\n",e.what());return 1;}}

#include "vp2_csm_image.h"
#include "vp2_csd_image.h"
#include <cstdio>
#include <stdexcept>
using bozo::native::irx::CompiledLibraries;
void images(CompiledLibraries &host){addVp2CsmImage(host);addVp2CsdImage(host);}
void run(CompiledLibraries &host){
    if(host.startImage("Csm_.irx;1")!=1 || host.startImage("Csd_.irx;1")!=2)
        throw std::runtime_error("native Csd initialization mismatch");
    const auto calls=host.nativeCalls();
    if(host.startImage("Csd_.irx;1")!=2 || host.nativeCalls()!=calls)
        throw std::runtime_error("native Csd duplicate start mismatch");
    host.advanceTo(0);host.advanceTo(1000000);
    if(host.nativeCalls()!=calls)throw std::runtime_error("native Csd sleep was bypassed");
}
int main(){try{
    bozo::libs::Libraries a,b;CompiledLibraries first(a,0x80000),second(b,0x80000);
    images(first);images(second);run(first);run(second);first.reset();run(first);
    std::puts("native compiled Csd/Csm initialization, retained sleeping task, repeat/reset/isolation passed; playback not qualified");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"native Csd boundary: %s\n",e.what());return 1;}}

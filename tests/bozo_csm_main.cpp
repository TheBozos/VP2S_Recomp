#include "vp2_csm_image.h"
#include <cstdio>
#include <stdexcept>
int main(){try{
    bozo::libs::Libraries a,b;
    bozo::native::irx::CompiledLibraries first(a,0x80000),second(b,0x80000);
    addVp2CsmImage(first);addVp2CsmImage(second);
    if(first.startImage("Csm_.irx;1")!=1 || first.nativeCalls()!=1 || second.nativeCalls()!=0 ||
       first.startImage("Csm_.irx;1")!=1 || first.nativeCalls()!=1 || second.startImage("Csm_.irx;1")!=1)
        throw std::runtime_error("native Csm initialization/repeat/isolation mismatch");
    first.reset();if(first.startImage("Csm_.irx;1")!=1 || first.nativeCalls()!=2)
        throw std::runtime_error("native Csm reset mismatch");
    std::puts("native compiled Csm initialization/repeat/reset/two-host isolation passed; playback not qualified");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

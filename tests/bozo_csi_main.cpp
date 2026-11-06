#include "vp2_csi_image.h"
#include <cstdio>
#include <stdexcept>
#include <fstream>
#include <cstring>
using bozo::native::irx::CompiledLibraries;
void run(CompiledLibraries &host,bozo::libs::Libraries &libs){
    if(host.startImage("Csi_.irx;1")!=1 || !libs.sif.servers.contains(0x1234) || libs.sif.servers.size()!=1)
        throw std::runtime_error("native Csi server initialization mismatch");
    const auto calls=host.nativeCalls();
    if(host.startImage("Csi_.irx;1")!=1 || host.nativeCalls()!=calls)
        throw std::runtime_error("native Csi duplicate initialization mismatch");
    host.advanceTo(0);host.advanceTo(1000000);
    if(host.nativeCalls()!=calls)throw std::runtime_error("native Csi RPC worker did not remain parked");
}
int main(int argc,char **argv){try{
    bozo::libs::Libraries a,b;CompiledLibraries first(a,0x80000),second(b,0x80000);
    addVp2CsiImage(first);addVp2CsiImage(second);run(first,a);
    if(!b.sif.servers.empty())throw std::runtime_error("native Csi server ownership leaked");
    run(second,b);first.reset();if(!a.sif.servers.empty() || b.sif.servers.empty())
        throw std::runtime_error("native Csi server reset isolation failed");
    run(first,a);
    if(argc>1){
        Runtime runtime;R5900Context context{};bozo::libs::Guest guest(runtime,&context);
        guest.fill(0x100000,0,32);guest.fill(0x110000,0x5a,32);
        if(argc!=3)throw std::runtime_error("private relocated Csi data required for request probe");
        std::ifstream file(argv[2],std::ios::binary);
        const std::vector<uint8_t> image((std::istreambuf_iterator<char>(file)),{});
        if(image.size()!=0xe00)throw std::runtime_error("invalid Csi reference image");
        for(auto *libs:{&a,&b}){
            guest.fill(0x110000,0x5a,32);
            libs->sif.servers.at(0x1234)(*libs,guest,{14,0x100000,16,0x110000,20});
            if(guest.read32(0x110000)!=1)throw std::runtime_error("Csi seed request failed");
            for(unsigned i=0;i<4;++i){
                uint32_t original;std::memcpy(&original,image.data()+0xd28+i*4,4);
                if(guest.read32(0x110004+i*4)!=~original)throw std::runtime_error("Csi private seed reply mismatch");
            }
            for(unsigned i=20;i<32;++i)if(guest.read8(0x110000+i)!=0x5a)
                throw std::runtime_error("Csi reply exceeded requested span");
        }
        std::puts("native Csi request 14 complete reply/canaries/two-host comparison passed; private words not logged");
    }
    std::puts("native Csi RPC registration/park/repeat/reset/two-host isolation passed; requests not qualified");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"native Csi boundary: %s\n",e.what());return 1;}}

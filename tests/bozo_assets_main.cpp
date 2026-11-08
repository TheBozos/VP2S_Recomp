#include "native_assets.h"
#include <cstdio>
#include <random>
#include <source_location>
#include <stdexcept>
#include <vector>
namespace {
void check(bool value,std::source_location at=std::source_location::current()){
    if(!value)throw std::runtime_error("asset provenance assertion at "+std::to_string(at.line()));
}
template<class F>void rejects(F f){bool threw=false;try{f();}catch(const std::runtime_error &){threw=true;}check(threw);}
}
int main(){try{
    using Origin=vp2s::AssetOrigin;using Kind=Origin::Kind;
    vp2s::NativeAssets a,other;
    a.discRead(0x30100000,2,9);
    check(a.origin(0x80100013)==Origin{Kind::Disc,9*2048,0,19});
    check(!other.origin(0x100013));
    a.overwrite(0x100100,64);
    check(a.origin(0x1000ff)->offset==255 && !a.origin(0x100100));
    check(a.origin(0x100140)->offset==320);
    a.copy(0x110000,0x1000e0,128);
    check(a.origin(0x110000)->offset==224 && !a.origin(0x110020) && a.origin(0x110060)->offset==320);
    a.copy(0x100080,0x100000,384); // Alias-safe metadata snapshot, including the unknown hole.
    check(a.origin(0x100080)->offset==0 && !a.origin(0x100180) && a.origin(0x1001c0)->offset==320);
    a.begin(0x1ff000,0x110000,0x120000,7);
    rejects([&]{a.begin(0x1ff000,0x110000,0x120000,7);});
    a.discRead(0x110000,1,88); // The decode keeps its accepted source identity across yields/rebinding.
    a.finish(0x1ff000,true,1024);
    check(a.origin(0x120005)==Origin{Kind::Part,9*2048+224,7,5});
    a.begin(0x1ff000,0x110000,0x120000,8);a.finish(0x1ff000,false,1024);
    check(a.origin(0x120005)->part==7 && a.pendingCount()==0);
    a.begin(0x1ff000,0x110000,0x120000,8,0x180000);a.finish(0x1ff000,true,1024);
    check(!a.origin(0x120000) && a.jobCount()==1);
    rejects([&]{a.copy(0x120000,0x110000,1);});
    rejects([&]{a.discRead(0x120000,1,4);});
    rejects([&]{a.copy(0x120000,0x110000,1,0x180000);});
    a.beginStep(0x1fe000,0x180000);
    a.copy(0x120000,0x110000,256,0x180000);
    check(!a.origin(0x120000));a.finishStep(0x1fe000,false);
    check(a.jobCount()==1 && a.pendingCount()==0);
    a.beginStep(0x1fe000,0x180000);a.finishStep(0x1fe000,true);
    check(a.origin(0x1203ff)==Origin{Kind::Part,88*2048,8,1023} && a.jobCount()==0);
    a.copy(0x121000,0x120080,128);
    check(a.origin(0x12107f)==Origin{Kind::Part,88*2048,8,255});
    a.begin(0x1ff000,0x190000,0x120080,9);a.finish(0x1ff000,true,128);
    check(!a.origin(0x120080) && a.origin(0x120100)->offset==256);
    rejects([&]{a.overwrite(0x1ffffff,2);});rejects([&]{a.origin(0x12000000);});
    rejects([&]{a.discRead(0x100000,0xffffffff,1);});
    // Real field memset uses a scratch-stack record at SP+0x80. Copies through
    // that storage keep byte provenance, overlap holes and worker ownership.
    a.discRead(0x180000,1,99);a.copy(0x70003800,0x180000,2048);
    check(a.origin(0x70003fff)==Origin{Kind::Disc,99*2048,0,2047});
    a.overwrite(0x70003960,0x200);
    check(!a.origin(0x70003960) && !a.origin(0x70003b5f));
    check(a.origin(0x7000395f)->offset==0x15f && a.origin(0x70003b60)->offset==0x360);
    a.copy(0x70003880,0x70003800,0x400);
    check(a.origin(0x70003880)->offset==0 && !a.origin(0x700039e0) && a.origin(0x70003be0)->offset==0x360);
    a.copy(0x190000,0x70003880,0x400);
    check(a.origin(0x190000)->offset==0 && !a.origin(0x190160) && a.origin(0x190360)->offset==0x360);
    check(a.origin(0x180160)->offset==0x160); // Distinct main-RAM bytes unchanged.
    a.begin(0x1ff000,0x180000,0x70000000,19,0x181000);a.finish(0x1ff000,true,256);
    check(!a.origin(0x70000000));rejects([&]{a.overwrite(0x70000000,1);});
    a.beginStep(0x1fe000,0x181000);a.copy(0x70000000,0x180000,128,0x181000);a.finishStep(0x1fe000,true);
    check(a.origin(0x700000ff)==Origin{Kind::Part,99*2048,19,255});
    rejects([&]{vp2s::NativeAssets::address(0x70000000);}); // Public RAM-only pointer contract unchanged.
    rejects([&]{a.overwrite(0x70003fff,2);});rejects([&]{a.origin(0x70004000);});
    rejects([&]{a.copy(0x180000,0x70003ff0,32);});rejects([&]{a.copy(0x70003ff0,0x180000,32);});
    check(a.origin(0x180000)->offset==0 && a.origin(0x70003fff)->offset==2047);
    // An independent per-byte oracle catches interval splitting, multi-source
    // copies, unknown overwrites and source/destination overlap in every step.
    vp2s::NativeAssets fuzz;
    std::vector<std::optional<Origin>> bytes(8192);
    std::mt19937 rng(0x29cc30);unsigned checked=0;
    for(unsigned step=0;step<4096;++step){
        uint32_t start=rng()%4096,length=rng()%1024,source=rng()%4096,kind=rng()%3;
        if(kind==0){start=(rng()%3)*2048;length=2048;const uint32_t lbn=rng()%65536;
            fuzz.discRead(0x100000+start,1,lbn);
            for(unsigned i=0;i<length;++i)bytes[start+i]=Origin{Kind::Disc,uint64_t(lbn)*2048,0,i};
        }else if(kind==1){fuzz.overwrite(0x100000+start,length);
            for(unsigned i=0;i<length;++i)bytes[start+i].reset();
        }else{const auto snapshot=bytes;fuzz.copy(0x100000+start,0x100000+source,length);
            for(unsigned i=0;i<length;++i)bytes[start+i]=snapshot[source+i];
        }
        for(unsigned i=0;i<bytes.size();++i){check(fuzz.origin(0x100000+i)==bytes[i]);++checked;}
    }
    std::printf("native asset provenance: %u byte comparisons, split/alias/unknown writes and resumed ownership passed\n",checked);
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

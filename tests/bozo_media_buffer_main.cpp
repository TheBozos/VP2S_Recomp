#include "native_media_buffer.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "media_buffer_original_functions.h"
#include "media_buffer_adapted_functions.h"
#include <cstdio>
#include <cstring>
#include <source_location>
#include <stdexcept>
namespace {
void check(bool ok,std::source_location at=std::source_location::current()){
    if(!ok)throw std::runtime_error("native media buffer assertion at "+std::to_string(at.line()));
}
template<class T>void put(Runtime &rt,uint32_t at,T value){std::memcpy(rt.memory.hostPointer(at,sizeof(T)),&value,sizeof(T));}
struct Decoder:bozo::libs::VideoDecoder {
    unsigned &freed;explicit Decoder(unsigned &n):freed(n){}~Decoder(){++freed;}
    void picture(const uint8_t *,size_t)override{throw std::runtime_error("unexpected coded data in allocation test");}
    void flush()override{throw std::runtime_error("unexpected flush");}
    bool next(bozo::libs::YuvPicture &)override{throw std::runtime_error("unexpected picture request");}
};
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::MediaBufferOwner {
    unsigned created=0,freed=0;bool fail=false,pauseMem=false;uint32_t allocation=0;struct Pause{};
    vp2s::NativeMediaBuffers buffers{[this]()->std::unique_ptr<bozo::libs::VideoDecoder>{if(fail)return {}; ++created;return std::make_unique<Decoder>(freed);}};
    vp2s::NativeMediaBuffers &nativeMediaBuffers()override{return buffers;}
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc)override{auto r=vp2s::nativeMediaBufferInstruction(c,rt,pc);check(r.has_value());return *r;}
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        if(pc==0x1371e0)c->r[2].sd[0]=3;
        else if(pc==0x100ab0){check(c->r[4].uw[0]==0x8100);c->r[2].ud[0]=allocation;}
        else if(pc==0x2bd380){check(c->r[4].uw[0]==allocation);c->r[2].ud[0]=allocation;}
        else if(pc==0x132e40){
            auto *dest=d.runtime().memory.hostPointer(c->r[4].uw[0],c->r[6].uw[0]);check(dest && !c->r[5].uw[0]);
            std::memset(dest,0,c->r[6].uw[0]);c->r[2].ud[0]=c->r[4].uw[0];
        }else if(pc==0x1f4ce0 || pc==0x1f4d38)c->r[2].ud[0]=0;
        else throw std::runtime_error("transport/interrupt tail executed");
        c->pc=c->r[31].uw[0];
        if(pc==0x132e40 && pauseMem){
            if(buffers.outstanding()){
                bool rejected=false;try{buffers.release(c->r[19].uw[0]-0xce500);}catch(const std::exception &){rejected=true;}check(rejected);
            }
            pauseMem=false;throw Pause{};
        }
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device write");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &why)override{throw std::runtime_error(why);}
};
void create(bozo::ee::Dispatcher &d,R5900Context &c,bool native){
    auto fn=d.runtime().functions.generated(0x1ef6f8);unsigned attempts=0;
    while(true){try{
        if(!attempts && native)check(vp2s::nativeMediaLibrary(&c,&d.runtime(),0x1ef6f8));
        else fn(d.runtime().memory.flat(),&c,&d.runtime());
        if(c.pc==0x2ba524)break;
    }catch(const Host::Pause &){}
        check(++attempts<8);const auto entry=d.runtime().functions.resumeEntry(c.pc);check(entry);fn=entry->resume;
    }
}
}
int main(){try{
    Host before,after;bozo::ee::Dispatcher old(before),native(after);
    old.setFunctions(media_buffer_original_function_table,media_buffer_original_function_count,media_buffer_original_resume_table,media_buffer_original_resume_count);
    native.setFunctions(media_buffer_adapted_function_table,media_buffer_adapted_function_count,media_buffer_adapted_resume_table,media_buffer_adapted_resume_count);
    for(auto *d:{&old,&native}){d->boot(0x2ba600,0x1fff000);for(auto pc:{0x1371e0u,0x100ab0u,0x2bd380u,0x132e40u,0x1f4ce0u,0x1f4d38u})d->bindLibrary(pc);}
    old.runtime().port=&before;native.runtime().port=&after;constexpr uint32_t object=0x180000;
    for(unsigned seed=0;seed<64;++seed){
        auto &a=old.runtime(),&b=native.runtime();std::memset(a.memory.flat(),0xa5,bozo::kFlatBytes);
        put(a,object+0xce4ec,0x159af0u);put(a,object+0xc9990,0x190u);put(a,object+0xc99a0,0xc8000u);put(a,object+0xce4e0,0x320u);
        put(a,object+0xc99c8,seed+1);put(a,object+0xc99cc,0x158ff8u);std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        R5900Context x{},y{};for(unsigned i=1;i<32;++i){x.r[i].ud[0]=seed*32+i;x.r[i].ud[1]=~x.r[i].ud[0];}
        x.r[4].ud[0]=object;x.r[29].ud[0]=seed&1?0x70003f00:0x1fff000;x.r[31].ud[0]=0x2bc3e0;y=x;
        old.runtime().functions.generated(0x2bbfc0)(a.memory.flat(),&x,&a);native.runtime().functions.generated(0x2bbfc0)(b.memory.flat(),&y,&b);x.cycleLimit=y.cycleLimit;
        check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        auto held=after.buffers.buffer(object);check(held && held->decoder && held->capacity==0xc8000 && held->semaphore==seed+1 && !after.buffers.outstanding());
        after.buffers.release(object);check(!after.buffers.buffer(object) && after.freed==seed);
        held.reset();check(after.freed==seed+1 && after.created==seed+1);
    }
    auto &rt=native.runtime();const auto rejects=[](auto action){bool failed=false;try{action();}catch(const std::exception &){failed=true;}check(failed);};
    after.fail=true;rejects([&]{after.buffers.begin(&rt,0x1ff0000,object);});check(!after.buffers.outstanding());after.fail=false;
    after.buffers.begin(&rt,0x1ff0000,object);rejects([&]{after.buffers.begin(&rt,0x1ff0000,object);});rejects([&]{after.buffers.release(object);});
    put(rt,object+0xc9994,1u);rejects([&]{after.buffers.finish(&rt,0x1ff0000);});check(after.buffers.outstanding()==1 && !after.buffers.buffer(object));
    put(rt,object+0xc9994,0u);after.buffers.finish(&rt,0x1ff0000);after.buffers.release(object);
    check(after.created==after.freed && !after.buffers.outstanding());
    for(unsigned seed=0;seed<64;++seed){
        auto &a=old.runtime(),&b=native.runtime();std::memset(a.memory.flat(),0xa5,bozo::kFlatBytes);std::memcpy(b.memory.flat(),a.memory.flat(),bozo::kFlatBytes);
        before.allocation=after.allocation=seed&1?0x500000:0;
        R5900Context x{},y{};x.r[4].ud[0]=object;x.r[29].ud[0]=seed&2?0x70003f00:0x1fff000;x.r[31].ud[0]=0x170000;y=x;
        old.runtime().functions.generated(0x2ba600)(a.memory.flat(),&x,&a);native.runtime().functions.generated(0x2ba600)(b.memory.flat(),&y,&b);x.cycleLimit=y.cycleLimit;
        check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        auto held=after.buffers.buffer(object);check(held && held->pacingFieldCycles==native.kernel().field() && held->pacingFieldCycles);
        rejects([&]{after.buffers.preparePacing(&b,object,0);});rejects([&]{after.buffers.preparePacing(&b,object,native.kernel().field());});
        constexpr uint32_t parent=object-0xc0;put(a,parent+0x15e77c,0x159bf8u);put(b,parent+0x15e77c,0x159bf8u);
        auto api=y;api.pc=0x1f02a0;api.r[19].ud[0]=parent;api.r[31].ud[0]=0x2be6d8;
        rejects([&]{vp2s::nativeMediaLibrary(&api,&b,0x1f02a0);});check(!held->initialized);
        api.r[31].ud[0]=0x2be6dc;auto expected=api;expected.r[2].sd[0]=1;expected.pc=0x2be6dc;
        check(vp2s::nativeMediaLibrary(&api,&b,0x1f02a0) && held->initialized);
        check(!std::memcmp(&api,&expected,sizeof(api)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        api.pc=0x1f02a0;rejects([&]{vp2s::nativeMediaLibrary(&api,&b,0x1f02a0);});
        auto reference=api;reference.pc=0x1ef6f8;reference.r[4].ud[0]=object+0xce500;
        reference.r[5].ud[0]=0x500000+(seed&1?4:0);reference.r[6].ud[0]=seed&2?0x1980:0x3000;
        reference.r[17].ud[0]=object;reference.r[31].ud[0]=0x2ba524;auto converted=reference;
        before.pauseMem=after.pauseMem=seed&4;create(old,reference,false);create(native,converted,true);reference.cycleLimit=converted.cycleLimit;
        check(!std::memcmp(&reference,&converted,sizeof(reference)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
        check(held->created==bool(converted.r[2].uw[0]) && !after.buffers.outstanding());
        if(held->created)check(held->mp==object+0xce500 && held->work==0x500000 && held->workBytes==0x3000);
        after.buffers.release(object);held.reset();
    }
    check(after.created==after.freed);
    std::puts("64 native-contract media resets +64 constructors +64 original software MPEG Create CPU/RAM cases: decoder lifetime, native init, CPU records, allocation/alignment failure and nested resume; no transport or playback qualification");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

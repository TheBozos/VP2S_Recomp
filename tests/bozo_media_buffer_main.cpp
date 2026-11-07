#include "native_media_buffer.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "media_buffer_original_functions.h"
#include "media_buffer_adapted_functions.h"
#include <cstdio>
#include <cstring>
#include <source_location>
#include <stdexcept>
#include <fstream>
#include <iterator>
#include <filesystem>
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
struct FramingDecoder:bozo::libs::VideoDecoder {
    std::vector<std::vector<uint8_t>> packets;unsigned flushes=0,ready=0;
    void picture(const uint8_t *data,size_t bytes)override{packets.emplace_back(data,data+bytes);++ready;}
    void flush()override{++flushes;}
    bool next(bozo::libs::YuvPicture &out)override{
        if(!ready)return false;--ready;out={2,2,{1,2,3,4},{5},{6}};return true;
    }
};
void videoFraming(){
    const std::vector<uint8_t> coded={0,0,1,0xb3,0xaa,0,0,1,0xb8,0xbb,0,0,1,0,0x11,0x22,0,0,1,1,0xaa,0,0,1,0,0x33,0x44,0,0,1,0xb7};
    for(size_t split=1;split<coded.size();++split){
        FramingDecoder decoder;vp2s::NativeMovieVideo video;
        check(video.accept(decoder,coded.data(),split,123,120));
        check(video.accept(decoder,coded.data()+split,coded.size()-split,-1,-1));
        check(video.sequenceEnded() && video.pendingBytes()==0 && video.acceptedBytes==coded.size() && video.submittedPictures==2 && decoder.flushes==1);
        check(decoder.packets.size()==2 && decoder.packets[0]==std::vector<uint8_t>(coded.begin(),coded.begin()+21) && decoder.packets[1]==std::vector<uint8_t>(coded.begin()+21,coded.end()));
        bozo::libs::YuvPicture out;check(video.takePicture(decoder,out) && out.y==std::vector<uint8_t>({1,2,3,4}));
        check(video.takePicture(decoder,out) && !video.takePicture(decoder,out));
    }
    FramingDecoder decoder;vp2s::NativeMovieVideo video;
    std::vector<uint8_t> many;for(unsigned i=0;i<20;++i)many.insert(many.end(),{0,0,1,0,uint8_t(i)});
    check(video.accept(decoder,many.data(),many.size(),10,9) && video.pictures.size()==8);
    const auto accepted=video.acceptedBytes,pending=video.pendingBytes();
    check(!video.accept(decoder,many.data(),many.size(),20,19) && video.acceptedBytes==accepted && video.pendingBytes()==pending);
    bozo::libs::YuvPicture out;for(unsigned i=0;i<19;++i)check(video.takePicture(decoder,out));
    check(!video.takePicture(decoder,out));
}
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::MediaBufferOwner {
    unsigned created=0,freed=0;bool fail=false,pauseMem=false,liveVideo=false;uint32_t allocation=0;struct Pause{};
    struct Packet {uint32_t function,gp,data;std::array<uint8_t,32> descriptor;};std::vector<Packet> packets;
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
        else if(pc==0x132c88){
            const auto bytes=c->r[6].uw[0];auto *dest=d.runtime().memory.hostPointer(c->r[4].uw[0],bytes);
            const auto *source=d.runtime().memory.hostPointer(c->r[5].uw[0],bytes);check(dest && source);
            std::memcpy(dest,source,bytes);c->r[2].ud[0]=c->r[4].uw[0];
        }
        else if(pc==0x132e40){
            auto *dest=d.runtime().memory.hostPointer(c->r[4].uw[0],c->r[6].uw[0]);check(dest && !c->r[5].uw[0]);
            std::memset(dest,0,c->r[6].uw[0]);c->r[2].ud[0]=c->r[4].uw[0];
        }else if(pc==0x1f4ce0 || pc==0x1f4d38)c->r[2].ud[0]=0;
        else if(pc==0x2b90a0 || pc==0x2bdac0 || pc==0x2bd9b0){
            Packet packet{pc,c->r[28].uw[0],c->r[6].uw[0],{}};
            std::memcpy(packet.descriptor.data(),d.runtime().memory.hostPointer(c->r[5].uw[0],32),32);packets.push_back(packet);
            if(liveVideo && pc==0x2b90a0){check(vp2s::nativeMediaLibrary(c,&d.runtime(),pc));return;}
            c->r[2].sd[0]=1; // Test sink accepted the actual descriptor; production handlers remain guarded.
        }
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
int main(int argc,char **argv){try{
    videoFraming();
    check(argc==2);std::ifstream input(argv[1],std::ios::binary);std::vector<uint8_t> constants{std::istreambuf_iterator<char>(input),{}};check(constants.size()==0xa0);
    Host before,after;bozo::ee::Dispatcher old(before),native(after);
    old.setFunctions(media_buffer_original_function_table,media_buffer_original_function_count,media_buffer_original_resume_table,media_buffer_original_resume_count);
    native.setFunctions(media_buffer_adapted_function_table,media_buffer_adapted_function_count,media_buffer_adapted_resume_table,media_buffer_adapted_resume_count);
    for(auto *d:{&old,&native}){d->boot(0x2ba600,0x1fff000);for(auto pc:{0x1371e0u,0x100ab0u,0x2bd380u,0x132e40u,0x132c88u,0x1f4ce0u,0x1f4d38u,0x2b90a0u})d->bindLibrary(pc);}
    after.liveVideo=true;
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
        if(held->created){
            check(held->mp==object+0xce500 && held->work==0x500000 && held->workBytes==0x3000);
            constexpr uint32_t returns[6]={0x2ba544,0x2ba564,0x2ba584,0x2ba5a4,0x2ba5e4,0x2ba5c4};
            constexpr uint32_t functions[6]={0x2b9550,0x2b9330,0x2b9300,0x2b92e0,0x2b9320,0x2b92a0};
            for(unsigned pass=0;pass<2;++pass)for(unsigned type=0;type<6;++type){
                auto x=converted;x.pc=0x1f0510;x.r[4].ud[0]=held->mp;x.r[5].ud[0]=type;x.r[6].ud[0]=functions[type];
                x.r[7].ud[0]=object;x.r[28].ud[0]=0x350000+seed*64+pass*4;x.r[31].ud[0]=returns[type];auto y=x;
                old.runtime().functions.generated(0x1f0510)(a.memory.flat(),&x,&a);check(vp2s::nativeMediaLibrary(&y,&b,0x1f0510));
                x.cycleLimit=y.cycleLimit;check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
                const auto &cb=held->callbacks[type];check(cb.set && cb.function==functions[type] && cb.data==object && cb.gp==0x350000+seed*64+pass*4);
                if(pass)check(y.r[2].uw[0]==functions[type]);
                auto invalid=y;invalid.pc=0x1f0510;invalid.r[4].ud[0]=held->mp;invalid.r[5].ud[0]=type;invalid.r[6].ud[0]=functions[type]+4;
                rejects([&]{vp2s::nativeMediaLibrary(&invalid,&b,0x1f0510);});check(!std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
            }
            std::memcpy(a.memory.flat()+0x313f00,constants.data(),constants.size());std::memcpy(b.memory.flat()+0x313f00,constants.data(),constants.size());
            constexpr uint32_t streamFunctions[3]={0x2b90a0,0x2bdac0,0x2bd9b0},callers[3]={0x2be738,0x2be75c,0x2be780};
            for(unsigned pass=0;pass<2;++pass)for(unsigned slot=0;slot<3;++slot){
                auto x=converted;x.pc=0x1f4990;x.r[4].ud[0]=held->mp;x.r[5].ud[0]=slot?4:0;x.r[6].ud[0]=slot?slot-1:0;
                x.r[7].ud[0]=streamFunctions[slot];x.r[8].ud[0]=object+(slot?0x15e680:0);
                x.r[28].ud[0]=0x351000+seed*64+pass*4;x.r[31].ud[0]=0x2ba3c8;
                put(a,x.r[29].uw[0],uint64_t(callers[slot]));put(b,x.r[29].uw[0],uint64_t(callers[slot]));auto y=x;
                old.runtime().functions.generated(0x1f4990)(a.memory.flat(),&x,&a);check(vp2s::nativeMediaLibrary(&y,&b,0x1f4990));
                x.cycleLimit=y.cycleLimit;check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
                const auto &cb=held->streams.at(slot).callback;
                check(cb.set && cb.function==streamFunctions[slot] && cb.data==object+(slot?0x15e680:0) && cb.gp==0x351000+seed*64+pass*4);
                if(pass)check(y.r[2].uw[0]==streamFunctions[slot]);
                auto invalid=y;invalid.pc=0x1f4990;invalid.r[4].ud[0]=held->mp;invalid.r[5].ud[0]=4;invalid.r[6].ud[0]=2;
                rejects([&]{vp2s::nativeMediaLibrary(&invalid,&b,0x1f4990);});check(!std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
            }
            const auto ring=(object+0xce5c0+63)&~63u;
            auto pesPath=std::filesystem::path(argv[1]).parent_path()/"pes.bin";
            std::ifstream pesInput(pesPath,std::ios::binary);std::vector<uint8_t> pesConstants{std::istreambuf_iterator<char>(pesInput),{}};check(pesConstants.size()==16);
            for(auto *runtime:{&a,&b})std::memcpy(runtime->memory.flat()+0x356270,pesConstants.data(),16);
            for(unsigned wrap=0;wrap<4;++wrap){
                const std::vector<uint8_t> packet={0,0,1,0xe0,0,19,0x80,0,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,0,0,1,0xb9,0,0,0,0};
                const auto start=wrap?0x90000-wrap*7:0;
                for(auto *runtime:{&a,&b})for(size_t i=0;i<packet.size();++i)put(*runtime,ring+(start+i)%0x90000,packet[i]);
                auto x=converted;x.pc=0x1f4590;x.r[4].ud[0]=held->mp;x.r[5].ud[0]=ring+start;x.r[6].ud[0]=packet.size();
                x.r[7].ud[0]=ring;x.r[8].ud[0]=0x90000;x.r[31].ud[0]=0x2bdf80;auto y=x;
                before.packets.clear();after.packets.clear();
                old.runtime().functions.generated(0x1f4590)(a.memory.flat(),&x,&a);check(vp2s::nativeMediaLibrary(&y,&b,0x1f4590));
                x.cycleLimit=y.cycleLimit;check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
                check(!after.buffers.outstanding() && before.packets.size()==1 && after.packets.size()==1);
                const auto &p=after.packets.front();check(p.function==0x2b90a0 && p.gp==0x351000+seed*64+4 && p.data==object);
                check(p.descriptor==before.packets.front().descriptor);
                uint32_t payload=0,length=0;std::memcpy(&payload,p.descriptor.data()+8,4);std::memcpy(&length,p.descriptor.data()+12,4);
                check(length==16 && payload==ring+(start+9)%0x90000);
                check(y.r[2].uw[0]==25);
            }
            check(held->video.acceptedBytes==64 && held->video.pendingBytes()==64);
            for(unsigned channel=0;channel<2;++channel)for(unsigned wrap=0;wrap<4;++wrap){
                std::vector<uint8_t> packet={0,0,1,0xbd,0,23,0x80,0,0,0xff,0x90,0,uint8_t(channel),1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,0,0,1,0xb9,0,0,0,0};
                const auto start=wrap?0x90000-wrap*7:0;const auto audio=object+0x15e680,cursor=wrap?0x1000-wrap*3:0;
                for(auto *runtime:{&a,&b}){
                    for(size_t i=0;i<packet.size();++i)put(*runtime,ring+(start+i)%0x90000,packet[i]);
                    put(*runtime,audio,object+0xce580);put(*runtime,audio+12,0x600000u);put(*runtime,audio+16,cursor);
                    put(*runtime,audio+20,0u);put(*runtime,audio+24,0x1000u);put(*runtime,audio+28,0u);
                    put(*runtime,object+0x15e608,0x90000u);
                }
                auto x=converted;x.pc=0x1f4590;x.r[4].ud[0]=held->mp;x.r[5].ud[0]=ring+start;x.r[6].ud[0]=packet.size();
                x.r[7].ud[0]=ring;x.r[8].ud[0]=0x90000;x.r[31].ud[0]=0x2bdf80;auto y=x;
                old.runtime().functions.generated(0x1f4590)(a.memory.flat(),&x,&a);check(vp2s::nativeMediaLibrary(&y,&b,0x1f4590));
                x.cycleLimit=y.cycleLimit;check(!std::memcmp(&x,&y,sizeof(x)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
                check(y.r[2].uw[0]==29 && !after.buffers.outstanding());
                for(unsigned i=0;i<16;++i)check(b.memory.flat()[0x600000+(cursor+i)%0x1000]==i+1);
            }
            const auto demuxStack=converted.r[29].uw[0]-0x150;
            after.buffers.beginDemux(&b,demuxStack,held->mp,ring,32,ring,0x90000);
            rejects([&]{after.buffers.release(object);});rejects([&]{after.buffers.beginReset(&b,demuxStack,held->mp);});
            rejects([&]{after.buffers.beginDemux(&b,demuxStack+16,held->mp,ring,32,ring,0x90000);});
            rejects([&]{after.buffers.finishDemux(demuxStack,33);});after.buffers.finishDemux(demuxStack,0);
            auto reset=converted;reset.pc=0x1f0438;reset.r[4].ud[0]=held->mp;reset.r[31].ud[0]=0x2b9ba0;
            const auto originalDecoder=held->decoder.get();const auto freed=after.freed;
            auto rejected=reset;after.fail=true;
            rejects([&]{vp2s::nativeMediaLibrary(&rejected,&b,0x1f0438);});after.fail=false;
            check(!std::memcmp(&reset,&rejected,sizeof(reset)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
            check(held->decoder.get()==originalDecoder && !held->inputResetPending && !after.buffers.outstanding());
            const auto resetStack=reset.r[29].uw[0]-0x20;
            after.buffers.beginReset(&b,resetStack,held->mp);
            rejects([&]{after.buffers.release(object);});
            rejects([&]{after.buffers.beginReset(&b,resetStack+0x40,held->mp);});
            rejects([&]{after.buffers.resetHelper(resetStack,held->work+4);});
            auto referenceReset=reset;
            old.runtime().functions.generated(0x1f0438)(a.memory.flat(),&referenceReset,&a);
            native.runtime().functions.generated(0x1f0438)(b.memory.flat(),&reset,&b);
            referenceReset.cycleLimit=reset.cycleLimit;
            check(!std::memcmp(&referenceReset,&reset,sizeof(reset)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
            check(reset.r[2].uw[0]==1 && reset.pc==0x2b9ba0 && held->decoder.get()!=originalDecoder && after.freed==freed+1);
            check(!held->video.acceptedBytes && !held->video.pendingBytes());
            check(held->inputResetPending && !after.buffers.outstanding() && held->callbacks[4].function==functions[4] && held->streams.size()==6);
            rejects([&]{after.buffers.beginReset(&b,resetStack,held->mp);});
            auto inputReset=converted;inputReset.pc=0x2bbfc0;inputReset.r[4].ud[0]=object;inputReset.r[31].ud[0]=0x2b9bb4;
            auto referenceInput=inputReset;const auto freshDecoder=held->decoder.get();
            old.runtime().functions.generated(0x2bbfc0)(a.memory.flat(),&referenceInput,&a);
            native.runtime().functions.generated(0x2bbfc0)(b.memory.flat(),&inputReset,&b);
            referenceInput.cycleLimit=inputReset.cycleLimit;
            check(!std::memcmp(&referenceInput,&inputReset,sizeof(inputReset)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
            check(!held->inputResetPending && !after.buffers.outstanding() && held->decoder.get()==freshDecoder);
            rejects([&]{after.buffers.beginInputReset(&b,resetStack,object);});
            // Exercise the declared API path after the paired input reset, too.
            reset=converted;reset.pc=0x1f0438;reset.r[4].ud[0]=held->mp;reset.r[31].ud[0]=0x2b9ba0;referenceReset=reset;
            old.runtime().functions.generated(0x1f0438)(a.memory.flat(),&referenceReset,&a);
            check(vp2s::nativeMediaLibrary(&reset,&b,0x1f0438));referenceReset.cycleLimit=reset.cycleLimit;
            check(!std::memcmp(&referenceReset,&reset,sizeof(reset)) && !std::memcmp(a.memory.flat(),b.memory.flat(),bozo::kFlatBytes));
            check(held->inputResetPending && !after.buffers.outstanding());
        }
        after.buffers.release(object);held.reset();
    }
    check(after.created==after.freed);
    std::puts("64 initializers +64 constructors +64 MPEG Create +192 callbacks +96 registrations +32 decoder resets +16 paired resets +64 video/PSS +128 audio/PSS CPU/RAM cases;30 native framing splits, capacity/YUV/flush, decoder lifetime/failure; no presentation qualification");return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

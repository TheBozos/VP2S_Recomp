#include "native_music.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include <fstream>
#include <cstdio>
namespace vp2s {
namespace {
uint32_t canonical(uint32_t a){
    if(a&~0x21ffffffu)throw std::runtime_error("native music address is not a bounded EE buffer");
    return a&0x1ffffffu;
}
uint8_t *buffer(Runtime *rt,uint32_t a,uint32_t n){
    a=canonical(a);
    if(!rt || a<0x100000 || uint64_t(a)+n>0x2000000)throw std::runtime_error("native music buffer outside EE RAM");
    return rt->memory.hostPointer(a,n);
}
template<class T>T read(Runtime *rt,uint32_t a){T v;std::memcpy(&v,buffer(rt,a,sizeof(v)),sizeof(v));return v;}
template<class T>void write(Runtime *rt,uint32_t a,T v){std::memcpy(buffer(rt,a,sizeof(v)),&v,sizeof(v));}
void validate(Runtime *rt,uint32_t player,uint32_t job){
    if(read<uint32_t>(rt,job)!=0x151d30 || canonical(read<uint32_t>(rt,job+4))!=canonical(player) ||
       canonical(read<uint32_t>(rt,player+0x100))!=canonical(job))
        throw std::runtime_error("native music lost typed player/job ownership");
}
NativeMusic &owner(Runtime *rt){
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *o=d?dynamic_cast<MusicOwner *>(&d->host()):nullptr;
    if(!o)throw std::runtime_error("native music requires its per-run owner");
    return o->nativeMusic();
}
void post(uint8_t *ram,R5900Context *c,Runtime *rt,uint32_t job){
    // Retain the original scalar message sender and its compiled return/resume
    // frame. It posts message5 after real decoding; no DMA callback is invoked.
    c->r[4].sd[0]=int32_t(job);
    bozo_call_indirect(ram,c,rt,0x106000);
}
void queue(uint8_t *ram,R5900Context *c,Runtime *rt){
    const auto job=c->r[5].uw[0];
    if(read<uint32_t>(rt,job)!=0x151d30){
        const auto original=rt->functions.generated(0x201f20);
        if(!original)throw std::runtime_error("native music queue missing resident compiled game code");
        original(ram,c,rt);return;
    }
    const auto player=read<uint32_t>(rt,job+4);
    if(read<uint8_t>(rt,job+12)!=1)throw std::runtime_error("native music initial channel is not ready");
    owner(rt).decode(rt,player,job,0);post(ram,c,rt,job);
}
void complete(uint8_t *ram,R5900Context *c,Runtime *rt){
    const auto job=c->r[4].uw[0];
    if(owner(rt).complete(rt,job))post(ram,c,rt,job);
    else c->pc=c->r[31].uw[0];
}
void next(uint8_t *ram,R5900Context *c,Runtime *rt){
    const auto player=c->r[4].uw[0],job=read<uint32_t>(rt,player+0x100);
    auto &music=owner(rt);
    if(music.advance(rt,player,job)){music.decode(rt,player,job,0);post(ram,c,rt,job);}
    else c->pc=c->r[31].uw[0];
}
}
NativeMusic::NativeMusic(const std::filesystem::path &path){
    std::ifstream in(path,std::ios::binary);char magic[4];uint32_t bytes=0;
    if(!in.read(magic,4) || std::memcmp(magic,"VPMU",4) || !in.read(reinterpret_cast<char *>(&bytes),4) ||
       bytes!=sizeof(tables) || !in.read(reinterpret_cast<char *>(&tables),sizeof(tables)) || in.peek()!=EOF)
        throw std::runtime_error("native music tables missing or invalid: run tools/vp2_music.py");
}
void NativeMusic::decode(Runtime *rt,uint32_t player,uint32_t job,unsigned channel){
    validate(rt,player,job);job=canonical(job);
    if(channel>1 || read<uint8_t>(rt,job+12)!=channel+1 || pending.contains(job))
        throw std::runtime_error("native music duplicate or unordered decode");
    Decoded d;d.player=player;d.channel=channel;
    d.source=read<uint32_t>(rt,player+0x1c)+2048*channel;
    d.destination=read<uint32_t>(rt,job+8);buffer(rt,d.destination,2048);
    d.historyAddress=read<uint32_t>(rt,player+0xf0+4*channel);
    std::memcpy(d.history.data(),buffer(rt,d.historyAddress,4096),4096);
    const auto *source=buffer(rt,d.source,2048);music::Quantized input{};
    const auto get=[&](size_t k){int16_t v;std::memcpy(&v,source+4+2*k,2);return int32_t(v);};
    for(size_t k=0;k<4;++k)input.header[k]=get(k);
    for(size_t k=0;k<28;++k)input.bands[k]=get(4+k);
    for(size_t k=0;k<864;++k)input.coefficients[k/4][k%4]=get(32+k);
    music::CodecRounding rounding;music::Block spectrum{};
    music::dequantize(input,tables,spectrum);
    music::window(spectrum,tables,d.output);
    music::synthesize(tables,d.history,d.output,float(input.header[0]));
    pending.emplace(job,std::move(d));
}
bool NativeMusic::advance(Runtime *rt,uint32_t player,uint32_t job){
    validate(rt,player,job);
    const auto half=read<uint8_t>(rt,player+0x1a),out=read<uint8_t>(rt,player+0x4b);
    if(half>1 || out>1)throw std::runtime_error("native music invalid buffer half");
    const auto count=read<uint32_t>(rt,player+0x2c+4*half);
    if(!count || count>1024)throw std::runtime_error("native music invalid coefficient frame count");
    const auto source=read<uint32_t>(rt,player+0x1c)+4096,destination=read<uint32_t>(rt,player+0x50)+4096;
    write(rt,player+0x1c,source);write(rt,player+0x50,destination);write(rt,player+0x2c+4*half,count-1);
    if(count>1){
        const auto marker=read<uint32_t>(rt,player+0xc8);
        write(rt,marker,read<uint8_t>(rt,source));write<uint8_t>(rt,source,0);write(rt,player+0xc8,marker+1);
        write(rt,job+8,destination|0x20000000u);write<uint8_t>(rt,job+12,1);return true;
    }
    write<uint8_t>(rt,player+0x48+out,1);write<uint8_t>(rt,player+0x1a,1-half);
    write(rt,player+0x1c,read<uint32_t>(rt,player+(half?0x3c:0x40))+0x80);
    write<uint8_t>(rt,player+0x4b,1-out);write(rt,player+0x50,read<uint32_t>(rt,player+(out?0x64:0x68))+0x80);
    write<uint8_t>(rt,player+0xb9,0);write<uint8_t>(rt,job+12,0);
    write<uint8_t>(rt,read<uint32_t>(rt,0x35ddc0)+6,0);return false;
}
bool NativeMusic::complete(Runtime *rt,uint32_t job){
    job=canonical(job);const auto state=read<uint8_t>(rt,job+12);auto it=pending.find(job);
    if(!state){if(it!=pending.end())pending.erase(it);return false;}
    if(it==pending.end() || state!=it->second.channel+1)throw std::runtime_error("native music completion has no decoded owner");
    const auto &d=it->second;const auto player=d.player;validate(rt,player,job);
    if(read<uint32_t>(rt,job+8)!=d.destination || read<uint32_t>(rt,player+0xf0+4*d.channel)!=d.historyAddress ||
       read<uint32_t>(rt,player+0x1c)+2048*d.channel!=d.source)
        throw std::runtime_error("native music buffer lifetime changed before publication");
    auto *pcm=buffer(rt,d.destination,2048);auto *history=buffer(rt,d.historyAddress,4096);
    const auto base=read<uint32_t>(rt,job+0x10+4*d.channel);uint32_t ring=0,nextRing=0;
    if(base){
        const auto size=read<uint32_t>(rt,job+0x18),reader=read<uint32_t>(rt,job+0x1c+4*d.channel);
        const auto writer=read<uint32_t>(rt,job+0x24+4*d.channel);
        if(size<4096 || uint64_t(base)+size>0x2000000 || writer<base || writer>=base+size || reader<base || reader>=base+size)
            throw std::runtime_error("native music analysis ring outside owned buffer");
        nextRing=writer+4096;bool room;
        if(nextRing<base+size)room=nextRing<reader || writer>=reader;
        else{room=writer>=reader;nextRing-=size;}
        if(room){buffer(rt,writer,4096);ring=writer;}
    }
    std::memcpy(history,d.history.data(),4096);
    for(size_t k=0;k<1024;++k){int32_t word;std::memcpy(&word,reinterpret_cast<const uint8_t *>(d.output.data())+4*k,4);
        const int16_t sample=int16_t(word);std::memcpy(pcm+2*k,&sample,2);}
    write(rt,job+8,d.destination+2048);
    if(ring){std::memcpy(buffer(rt,ring,4096),d.output.data(),4096);write(rt,job+0x24+4*d.channel,nextRing);}
    const auto channel=d.channel;pending.erase(it);
    if(!channel){write<uint8_t>(rt,job+12,2);decode(rt,player,job,1);return true;}
    if(++frames==1 || frames%128==0)std::fprintf(stderr,"[vp2s:music] native stereo frames=%llu\n",(unsigned long long)frames);
    if(advance(rt,player,job)){decode(rt,player,job,0);return true;}return false;
}
void installNativeMusic(Runtime &rt){
    for(const auto target:{0x106000u})if(!rt.functions.generated(target))
        throw std::runtime_error("native music missing compiled main entry");
    rt.functions.setOverride(0x201f20,queue);
}
void registerNativeMusic(bozo::port::Registry &registry){
    registry.registerFunction(0x105b80,complete);registry.registerFunction(0x1060a0,next);
}
}

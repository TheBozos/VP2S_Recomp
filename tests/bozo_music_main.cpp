#include "native_music.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include <cstdio>
#include <fstream>
#include <source_location>
#include <vector>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){
    if(!v)throw std::runtime_error("native music assertion at "+std::to_string(at.line()));
}
template<class F>void rejects(F f){bool caught=false;try{f();}catch(const std::runtime_error &){caught=true;}check(caught);}
template<class T>void put(Runtime &rt,uint32_t a,T v){std::memcpy(rt.memory.hostPointer(a,sizeof(v)),&v,sizeof(v));}
template<class T>T get(Runtime &rt,uint32_t a){T v;std::memcpy(&v,rt.memory.hostPointer(a,sizeof(v)),sizeof(v));return v;}
struct Record {vp2s::music::Quantized input{};vp2s::music::Block before{},output{},after{};};
struct Host:bozo::ee::Host,vp2s::MusicOwner{
    vp2s::NativeMusic music;
    explicit Host(const char *path):music(path){}
    vp2s::NativeMusic &nativeMusic()override{return music;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("unexpected library");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("unexpected VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("unexpected I/O");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("unexpected I/O");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &)override{throw std::runtime_error("unexpected fault");}
};
bool record(std::ifstream &in,Record &r){
    uint32_t magic=0;if(!in.read(reinterpret_cast<char *>(&magic),4))return false;check(magic==0x3143554d);
    check(bool(in.read(reinterpret_cast<char *>(&r.input),sizeof(r.input))));
    for(auto *b:{&r.before,&r.output,&r.after})check(bool(in.read(reinterpret_cast<char *>(b),4096)));return true;
}
constexpr uint32_t player=0x400000,job=0x401000,source=0x410000,pcm=0x420000,history=0x430000,status=0x440000;
void prepare(Runtime &rt,const Record &a,const Record &b){
    std::memset(rt.memory.flat(),0,bozo::kFlatBytes);
    put(rt,job,uint32_t(0x151d30));put(rt,job+4,player);put(rt,job+8,pcm|0x20000000);put<uint8_t>(rt,job+12,1);
    put(rt,player+0x100,job);put(rt,player+0x1c,source);put(rt,player+0x50,pcm);put<uint32_t>(rt,player+0x2c,1);
    put(rt,player+0x40,source+0x10000);put(rt,player+0x68,pcm+0x10000);put(rt,player+0xf0,history);put(rt,player+0xf4,history+4096);
    put(rt,0x35ddc0,status);put<uint8_t>(rt,status+6,1);put<uint8_t>(rt,player+0xb9,1);
    for(unsigned ch=0;const auto *r:{&a,&b}){
        std::memcpy(rt.memory.hostPointer(history+4096*ch,4096),r->before.data(),4096);
        const auto start=source+2048*ch;put<uint32_t>(rt,start,0xdeadbeef);
        for(unsigned k=0;k<4;++k)put<int16_t>(rt,start+4+2*k,int16_t(r->input.header[k]));
        for(unsigned k=0;k<28;++k)put<int16_t>(rt,start+12+2*k,int16_t(r->input.bands[k]));
        for(unsigned k=0;k<864;++k)put<int16_t>(rt,start+68+2*k,int16_t(r->input.coefficients[k/4][k%4]));++ch;
    }
}
void published(Runtime &rt,const Record &r,unsigned ch){
    check(!std::memcmp(rt.memory.hostPointer(history+4096*ch,4096),r.after.data(),4096));
    for(unsigned k=0;k<1024;++k){int32_t word;std::memcpy(&word,reinterpret_cast<const uint8_t *>(r.output.data())+4*k,4);
        check(get<int16_t>(rt,pcm+2048*ch+2*k)==int16_t(word));}
}
}
int main(int argc,char **argv){try{
    if(argc!=3)throw std::runtime_error("tables and typed live records required");
    Host host(argv[1]);auto &music=host.music;vp2s::NativeMusic other(argv[1]);bozo::ee::Dispatcher dispatcher(host);
    Runtime &rt=dispatcher.runtime();std::ifstream in(argv[2],std::ios::binary);
    Record a,b;unsigned pairs=0;const auto saved=_mm_getcsr();
    while(record(in,a)){
        check(record(in,b));prepare(rt,a,b);const auto rounding=(pairs%4)*0x2000u;
        _mm_setcsr((saved&~_MM_ROUND_MASK)|rounding);
        const auto before=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
        music.decode(&rt,player,job,0);check(music.pendingCount()==1 && other.pendingCount()==0);
        check((_mm_getcsr()&_MM_ROUND_MASK)==rounding);check(!std::memcmp(before.data(),rt.memory.flat(),before.size()));
        rejects([&]{music.decode(&rt,player,job,0);});rejects([&]{other.complete(&rt,job);});
        put(rt,job+8,pcm+16);rejects([&]{music.complete(&rt,job);});put(rt,job+8,pcm|0x20000000);
        check(!std::memcmp(before.data(),rt.memory.flat(),before.size()));
        check(music.complete(&rt,job));published(rt,a,0);check(music.pendingCount()==1 && get<uint8_t>(rt,job+12)==2);
        check(!music.complete(&rt,job));published(rt,b,1);check(music.pendingCount()==0);
        check(get<uint32_t>(rt,player+0x2c)==0 && get<uint8_t>(rt,player+0x48)==1 && get<uint8_t>(rt,player+0x4b)==1);
        check(get<uint8_t>(rt,player+0x1a)==1 && get<uint8_t>(rt,job+12)==0 && get<uint8_t>(rt,status+6)==0 && get<uint8_t>(rt,player+0xb9)==0);
        check(get<uint32_t>(rt,player+0x1c)==source+0x10080 && get<uint32_t>(rt,player+0x50)==pcm+0x10080);
        ++pairs;
    }
    check(pairs>0 && music.completedFrames()==pairs);
    prepare(rt,a,b);music.decode(&rt,player,job,0);put<uint8_t>(rt,job+12,0);
    const auto cancelled=std::vector<uint8_t>(rt.memory.flat(),rt.memory.flat()+bozo::kFlatBytes);
    check(!music.complete(&rt,job) && music.pendingCount()==0);check(!std::memcmp(cancelled.data(),rt.memory.flat(),cancelled.size()));
    bozo::port::Registry registry;vp2s::registerNativeMusic(registry);R5900Context c{};c.pc=0x105b80;c.r[4].uw[0]=job;c.r[31].uw[0]=0x123456;
    registry.find(0x105b80)(rt.memory.flat(),&c,&rt);check(c.pc==0x123456);
    prepare(rt,a,b);c.pc=0x1060a0;c.r[4].uw[0]=player;
    registry.find(0x1060a0)(rt.memory.flat(),&c,&rt);check(c.pc==0x123456 && get<uint8_t>(rt,job+12)==0);
    prepare(rt,a,b);put<uint32_t>(rt,player+0x2c,2);put(rt,player+0xc8,status+0x100);
    std::memcpy(rt.memory.hostPointer(source+4096,4096),rt.memory.hostPointer(source,4096),4096);put<uint8_t>(rt,source+4096,0xab);
    music.decode(&rt,player,job,0);check(music.complete(&rt,job));check(music.complete(&rt,job));
    check(get<uint8_t>(rt,status+0x100)==0xab && get<uint8_t>(rt,source+4096)==0 && get<uint32_t>(rt,player+0xc8)==status+0x101);
    check(get<uint32_t>(rt,player+0x2c)==1 && get<uint8_t>(rt,job+12)==1 && music.pendingCount()==1);
    check(music.complete(&rt,job));check(!music.complete(&rt,job));check(music.completedFrames()==pairs+2);
    prepare(rt,a,b);
    put<uint8_t>(rt,job+12,1);put<uint8_t>(rt,player+0x1a,2);rejects([&]{music.advance(&rt,player,job);});
    _mm_setcsr(saved);
    std::printf("%u native music stereo publications: exact recorded PCM/history, deferred visibility, ownership/lifetime, cancellation, halves and rounding\n",pairs);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

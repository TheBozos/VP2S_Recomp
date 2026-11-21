// Battle column capture (bozo/native_column_capture.cpp): registration at 4de910, the nine
// readback requests (battle 0x18's: PSMCT24, 64 x 417 columns of the 576-wide frame), the
// sprite 4de130 computes (the battle's first frame: column 1 at 1824,1840 .. 1888,2256), its
// build's end, the flush lease with its scalar-flush checks, and the last column's completion.
#include "native_column_capture.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include <cstdio>
#include <cstring>
#include <source_location>
#include <stdexcept>
namespace {
void check(bool b,std::source_location at=std::source_location::current()){
    if(!b)throw std::runtime_error("native column capture assertion failed at "+std::to_string(at.line()));
}
template<class F>void rejects(F f){bool failed=false;try{f();}catch(const std::runtime_error &){failed=true;}check(failed);}
template<class T>void put(Runtime &rt,uint32_t at,T v){std::memcpy(rt.memory.hostPointer(at,sizeof(T)),&v,sizeof(T));}
struct Host:bozo::ee::Host,bozo::port::Host {
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *,R5900Context *,Runtime *,uint32_t)override{throw std::runtime_error("instruction");}
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("leaf");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device read");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device write");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host host;bozo::ee::Dispatcher d(host);Runtime &rt=d.runtime();
    constexpr uint32_t vt=0x175290,object=0x87c510,buffer=0x1f399d0,records=0x1f39a00,stride=0x13980,manager=0x651340;
    put(rt,vt+0x14,0x4de080u);put(rt,vt+0x18,0x4dde60u);put(rt,object,vt);put(rt,object+0x94,buffer);
    put(rt,object+0x1e4,stride);put(rt,object+0x1e8,int16_t(1));put(rt,object+0x1ea,uint8_t(2));put(rt,object+0x1c0,0x80808080u);
    for(unsigned k=0;k<9;++k)put(rt,object+0x98+4*k,0xe8a920u+0x230u*k);
    vp2s::NativeColumnCaptures columns;
    rejects([&]{put(rt,object+0x1ea,uint8_t(3));columns.begin(&rt,object);});put(rt,object+0x1ea,uint8_t(2));   // not the capture state
    auto &cap=columns.begin(&rt,object);check(cap.records==records && cap.stride==stride && cap.psm==1 && cap.nodes[8]==0xe8a920u+0x230u*8);
    // The readbacks: column k of the frame into record k + 80; other nodes are not the capture's.
    vp2s::NativeDepthProbeRequest q;q.sbp=0x1100;q.sbw=9;q.psm=1;q.y=0;q.width=64;q.height=417;
    check(!columns.column(0xe00000,q,records+0x80));
    for(unsigned k=0;k<9;++k){q.x=int32_t(64*k);
        rejects([&]{auto bad=q;bad.x+=1;columns.column(0xe8a920u+0x230u*k,bad,records+k*stride+0x80);});
        rejects([&]{columns.column(0xe8a920u+0x230u*k,q,records+k*stride+0x90);});
        const auto r=columns.column(0xe8a920u+0x230u*k,q,records+k*stride+0x80);check(r && r->first==&cap && r->second==k);}
    check(cap.height==417);
    rejects([&]{auto bad=q;bad.x=0;bad.height=416;columns.column(0xe8a920u,bad,records+0x80);});   // one height per capture
    // 4de718 for column 1: record +0/+2 = 64/417, TEX0 of the current cache slot (CT24, TCC 0).
    const uint32_t rec=records+stride,writer=0x6554a0,sp=0x1fffc00;
    put(rt,rec,uint16_t(64));put(rt,rec+2,uint16_t(417));put(rt,0x35de38,manager);put(rt,manager+0x2670,int16_t(241));
    const uint64_t tex0=0x36a0ull|1ull<<20|6ull<<26|9ull<<30;put(rt,manager+0x1010+16*241,tex0);
    put(rt,writer,0x30655440u);put(rt,sp+0xc0,1824);put(rt,sp+0xd0,1840);
    R5900Context c{};c.r[21].uw[0]=object;c.r[17].uw[0]=rec;c.r[20].uw[0]=writer;c.r[29].uw[0]=sp;c.r[19].sw[0]=1888;c.r[3].sw[0]=2256;c.r[2].ud[0]=0xfffffeull<<32;
    rejects([&]{columns.sprite(&rt,&c);});                               // the frame was not copied yet
    cap.copied=true;
    { auto bad=c;bad.r[17].uw[0]=rec+16;rejects([&]{columns.sprite(&rt,&bad);});
      put(rt,manager+0x1010+16*241,tex0|1ull<<34);rejects([&]{columns.sprite(&rt,&c);});put(rt,manager+0x1010+16*241,tex0);   // TCC 1: unported
      bad=c;bad.r[19].sw[0]=0x1000;rejects([&]{columns.sprite(&rt,&bad);}); }
    columns.sprite(&rt,&c);
    const auto &s=cap.sprites[1];
    check(s.writer==writer && s.begin==0x30655480u && s.colour==0x80808080 && s.z==0xfffffe && s.x0==1824 && s.y0==1840 && s.x1==1888 && s.y1==2256);
    check(s.u0==0.5f && s.v0==0.5f && s.u1==64 && s.v1==417 && !s.built);
    // 4de888: the build's end; begin must still be the reset's.
    put(rt,writer+12,0x30655480u);put(rt,writer+8,0x30655560u);columns.built(&rt,&c);check(s.built && s.end==0x30655560u);
    // 4ddef4 (column 1): v0 = the draw set (+bc + 12) + 4 * its +a.
    const uint32_t set=object+0xbc+12,fsp=0x1fffdb0;put(rt,set+0xa,uint8_t(1));put(rt,set+4,writer);
    R5900Context f{};f.r[19].uw[0]=object;f.r[16].uw[0]=1;f.r[29].uw[0]=fsp;f.r[2].uw[0]=set+4;
    rejects([&]{columns.flush(&rt,&f,0x4dded0);});                       // only the last column flushes with its owner
    const auto [flushed,k]=columns.flush(&rt,&f,0x4ddef4);check(flushed==&cap && k==1 && columns.active(fsp) && !s.built);
    rejects([&]{columns.flush(&rt,&f,0x4ddef4);});                      // drawn once
    R5900Context r{};r.r[29].uw[0]=fsp-0x30;r.r[17].uw[0]=writer;r.r[9].uw[0]=0;r.r[16].uw[0]=0x30655570u;r.r[2].uw[0]=0x30655480u;r.r[5].uw[0]=2;r.r[8].uw[0]=(0x30655570u-0x30655480u)/16;
    { auto bad=r;bad.r[9].uw[0]=object+0x90;rejects([&]{columns.retire(&rt,&bad);}); }
    columns.retire(&rt,&r);rejects([&]{columns.flushed(&rt,fsp);});
    put(rt,writer+8,0x30655570u);put(rt,writer+12,0x30655570u);columns.flushed(&rt,fsp);columns.complete(fsp);check(!columns.active(fsp));
    columns.complete(fsp);                                               // the last column passes 4ddef8 twice
    rejects([&]{columns.completion(fsp);});                             // no last column flushed yet
    std::puts("native column capture: registration, readbacks, sprite, build, flush, completion");
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

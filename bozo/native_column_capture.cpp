#include "native_cast.h"
#include "native_conditional_sites.h"
#include "native_column_capture.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/hash.h"
#include <cstring>
#include <format>
#include <stdexcept>
namespace vp2s {
namespace {
const uint8_t *record(Runtime *rt,uint32_t at,uint32_t bytes){
    at&=0x1ffffff;   // writer pointers are uncached (0x30000000) mirrors
    if(!rt || at<0x100000 || at>=0x2000000 || bytes>0x2000000-at)throw std::runtime_error("native column capture record outside EE RAM");
    return rt->memory.hostPointer(at,bytes);
}
template<class T>T value(Runtime *rt,uint32_t at){T v;std::memcpy(&v,record(rt,at,sizeof(T)),sizeof(T));return v;}
bool code(Runtime *rt,uint32_t lo,uint32_t hi,uint64_t expected){return bozo::hash::fnv1a64(record(rt,lo,hi-lo),hi-lo)==expected;}
constexpr uint32_t kVtable=0x175290,kCapture=0x4de910,kSprite=0x4de718,kBuilt=0x4de888,kFlushLast=0x4dded0,kFlush=0x4ddef4,kReturn=0x4ddef8,kEnd=0x4ddf24;
}
NativeColumnCapture &NativeColumnCaptures::begin(Runtime *rt,uint32_t object){
    // The class's methods (vtable 175290: +14 4de080, +18 4dde60) and the capture state
    // (+1ea = 2: 4de080 is about to call 4de910).
    if(value<uint32_t>(rt,object)!=kVtable || value<uint32_t>(rt,kVtable+0x14)!=0x4de080 || value<uint32_t>(rt,kVtable+0x18)!=0x4dde60 ||
       value<uint8_t>(rt,object+0x1ea)!=2)
        throw std::runtime_error(std::format("native column capture {:x} outside its class or state",object));
    for(const auto &[stack,lease]:leases)if(lease.object==object)throw std::runtime_error("native column capture restarted during its flush");
    NativeColumnCapture c;c.object=object;
    const auto buffer=value<uint32_t>(rt,object+0x94);
    c.records=(buffer+0x7f)&~0x7fu;c.stride=value<uint32_t>(rt,object+0x1e4)&~15u;c.psm=value<int16_t>(rt,object+0x1e8);
    if(c.psm<0 || c.psm>2)throw std::runtime_error(std::format("native column capture format {:x} is not a colour format",c.psm));
    for(unsigned k=0;k<9;++k){c.nodes[k]=value<uint32_t>(rt,object+0x98+4*k)&0x1ffffff;if(!c.nodes[k])throw std::runtime_error("native column capture misses a readback node");}
    auto &slot=captures[object];slot=c;return slot;
}
std::optional<std::pair<NativeColumnCapture *,unsigned>> NativeColumnCaptures::column(uint32_t node,const NativeDepthProbeRequest &q,uint32_t destination){
    for(auto &[object,c]:captures)for(unsigned k=0;k<9;++k)if(c.nodes[k]==node){
        // The request 4deb3c makes for column k (into record k + 80).
        // The record holds 64 x height pixels after its 0x80-byte header; every column has one height.
        const unsigned bytes=c.psm==0?4:c.psm==1?3:2;
        if(destination!=((c.records+k*c.stride+0x80)&0x1ffffff) || q.psm!=c.psm || q.x!=int32_t(64*k) || q.y!=0 || q.width!=64 || q.sbw!=9 ||
           q.height<1 || q.height>1024 || c.stride<0x80u+64u*bytes*uint32_t(q.height) || (c.height && q.height!=c.height))
            throw std::runtime_error(std::format("native column capture {:x} column {} request differs from 4de910's",object,k));
        c.height=q.height;
        return std::make_pair(&c,k);
    }
    return std::nullopt;
}
void NativeColumnCaptures::sprite(Runtime *rt,const R5900Context *c){
    // 4de718: s5 object, s1 record k, s4 draw writer, s3 = x1, v1 = y1, v0 = z << 32;
    // x0, y0 at sp+c0 / sp+d0 (screen pixels, from cvt.w).
    const auto object=c->r[21].uw[0],rec=c->r[17].uw[0],writer=c->r[20].uw[0],sp=c->r[29].uw[0];
    auto it=captures.find(object);
    if(it==captures.end() || !it->second.copied)throw std::runtime_error(std::format("native column sprite {:x} has no copied capture",object));
    auto &cap=it->second;
    if(rec<cap.records || (rec-cap.records)%cap.stride || (rec-cap.records)/cap.stride>=9)
        throw std::runtime_error("native column sprite record outside its capture");
    const unsigned k=(rec-cap.records)/cap.stride;
    // The texture the sprite selected (4de574: the cache's current slot): the record's format,
    // TCC 0 (alpha = the vertex's), modulate, a 2^tw x 2^th area holding the column.
    const auto manager=value<uint32_t>(rt,0x35de38);const auto slot=value<int16_t>(rt,manager+0x2670);
    if(slot<0 || slot>=256)throw std::runtime_error("native column sprite has no texture slot");
    const auto tex0=value<uint64_t>(rt,manager+0x1010+16*uint32_t(slot));
    const uint32_t width=value<uint16_t>(rt,rec),height=value<uint16_t>(rt,rec+2);
    const unsigned psmBits[3]={0x00,0x01,0x02};
    if(((tex0>>20)&0x3f)!=psmBits[cap.psm] || ((tex0>>34)&1) || ((tex0>>35)&3) || (1u<<((tex0>>26)&15))<width || (1u<<((tex0>>30)&15))<height)
        throw std::runtime_error(std::format("unported native column sprite texture {:x} (object {:x} column {})",tex0,object,k));
    if(width!=64 || int32_t(height)!=cap.height)throw std::runtime_error("native column sprite record differs from its readback");
    auto &s=cap.sprites[k];s=NativeColumnSprite{};
    s.writer=writer;s.begin=value<uint32_t>(rt,writer)+0x40;s.colour=value<uint32_t>(rt,object+0x1c0);s.z=uint32_t(c->r[2].ud[0]>>32);
    s.x0=value<int32_t>(rt,sp+0xc0);s.y0=value<int32_t>(rt,sp+0xd0);s.x1=c->r[19].sw[0];s.y1=c->r[3].sw[0];
    // XYZ2 packs x << 4 | (y << 4) << 16 from 32-bit words: only 12-bit corners reach the GS intact.
    for(int32_t v:{s.x0,s.y0,s.x1,s.y1})if(v<0 || v>0xfff)throw std::runtime_error("native column sprite corner outside the GS range");
    s.u0=0.5f;s.v0=0.5f;s.u1=float(width);s.v1=float(height);   // UV 0x80008, then (+0 << 4, +2 << 4)
}
void NativeColumnCaptures::built(Runtime *rt,const R5900Context *c){
    const auto object=c->r[21].uw[0],writer=c->r[20].uw[0];
    auto it=captures.find(object);if(it==captures.end())throw std::runtime_error("native column build has no capture");
    for(auto &s:it->second.sprites)if(s.writer==writer && !s.built){
        if(value<uint32_t>(rt,writer+12)!=s.begin)throw std::runtime_error("native column build escaped its writer reset");
        s.end=value<uint32_t>(rt,writer+8);s.built=true;return;}
    throw std::runtime_error(std::format("native column build {:x} ended without its sprite",object));
}
std::pair<const NativeColumnCapture *,unsigned> NativeColumnCaptures::flush(Runtime *rt,const R5900Context *c,uint32_t pc){
    // 4dde60's loop: s3 object, s0 column, v0 = the draw set (+bc + 12k) + 4 * its +a; owner
    // a1 = +90 for the last column (4dded0), 0 for the others (4ddef4, after its delay slot).
    const auto object=c->r[19].uw[0],k=c->r[16].uw[0],sp=c->r[29].uw[0];
    if(leases.contains(sp))throw std::runtime_error("native column flush stack reused");
    auto it=captures.find(object);
    if(it==captures.end() || k>=9 || (pc==kFlushLast)!=(k==8))throw std::runtime_error(std::format("native column flush {:x} column {} has no capture",object,k));
    const auto set=object+0xbc+12*k;
    if(c->r[2].uw[0]!=set+4u*value<uint8_t>(rt,set+0xa))throw std::runtime_error("native column flush differs from 4dde60");
    const auto writer=value<uint32_t>(rt,c->r[2].uw[0]);const uint32_t owner=pc==kFlushLast?c->r[5].uw[0]:0;
    if(pc==kFlushLast && owner!=object+0x90)throw std::runtime_error("native column flush lost its owner");
    auto &s=it->second.sprites[k];
    if(!s.built || s.writer!=writer || value<uint32_t>(rt,writer+12)!=s.begin || value<uint32_t>(rt,writer+8)!=s.end)
        throw std::runtime_error(std::format("native column flush {:x} column {} has no recorded build",object,k));
    leases.emplace(sp,Lease{object,k,writer,owner,s.begin,s.end});
    s.built=false;   // drawn once
    return {&it->second,k};
}
void NativeColumnCaptures::retire(Runtime *rt,const R5900Context *c){
    auto &lease=leases.at(c->r[29].uw[0]+0x30);
    if(lease.retired || c->r[17].uw[0]!=lease.writer || c->r[9].uw[0]!=lease.owner ||
       c->r[16].uw[0]!=lease.end+16 || c->r[2].uw[0]!=lease.begin || c->r[5].uw[0]!=2 || c->r[6].uw[0] || c->r[8].uw[0]!=(lease.end+16-lease.begin)/16 ||
       value<uint32_t>(rt,lease.writer+8)!=lease.end || value<uint32_t>(rt,lease.writer+12)!=lease.begin)
        throw std::runtime_error("native column draw changed during the original flush");
    lease.retired=true;
}
void NativeColumnCaptures::flushed(Runtime *rt,uint32_t stack){
    auto &lease=leases.at(stack);
    if(!lease.retired || lease.flushed || value<uint32_t>(rt,lease.writer+8)!=lease.end+16 || value<uint32_t>(rt,lease.writer+12)!=lease.end+16)
        throw std::runtime_error("native column draw did not finish its scalar flush");
    lease.flushed=true;
}
void NativeColumnCaptures::complete(uint32_t stack){
    auto it=leases.find(stack);if(it==leases.end())return;   // the last column passes 4ddef8 twice
    if(!it->second.flushed)throw std::runtime_error("native column flush returned before it finished");
    if(it->second.column==8)finished[stack]=it->second.object;
    leases.erase(it);
}
uint32_t NativeColumnCaptures::completion(uint32_t stack){
    auto it=finished.find(stack);
    if(it==finished.end())throw std::runtime_error("native column completion without its last column's flush");
    const auto object=it->second;finished.erase(it);return object;
}
std::optional<bool> nativeColumnCaptureInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(!nativeColumnCaptureSite(pc))return std::nullopt;   // its sites: native_conditional_sites.h
    if(pc!=kCapture && pc!=kSprite && pc!=kBuilt && pc!=kFlushLast && pc!=kFlush && pc!=kReturn && pc!=kEnd && pc!=0x120f4c && pc!=0x120f58)return std::nullopt;
    auto *d=rt?vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services):nullptr;
    auto *owner=d?vp2s::nativeCast<ColumnCaptureOwner>(&d->host()):nullptr;
    if(pc==0x120f4c || pc==0x120f58){
        if(!owner || !c || !owner->nativeColumnCaptures().active(c->r[29].uw[0]+0x30))return std::nullopt;
        auto &columns=owner->nativeColumnCaptures();
        if(pc==0x120f4c)columns.retire(rt,c);else columns.flushed(rt,c->r[29].uw[0]+0x30);
        return false;
    }
    if(!rt || value<uint32_t>(rt,0x4de910)!=0x27bdff10u)return std::nullopt;   // another overlay here
    if(!c || !owner)throw std::runtime_error("native column capture requires its owner");
    auto &columns=owner->nativeColumnCaptures();
    if(pc==kCapture){
        if(!code(rt,0x4de910,0x4deba0,0xe40e1529d15da04dull))throw std::runtime_error("native column capture differs from the USA 4de910");
        const auto object=c->r[4].uw[0];
        if(!value<uint32_t>(rt,object+0x94))return false;   // no buffer: the original returns at once
        auto &capture=columns.begin(rt,object);
        capture.image=owner->claimNativeColumnImage(object);
        return false;   // the original lays out the records and asks the nine readbacks
    }
    if(pc==kSprite || pc==kBuilt){
        if(!code(rt,0x4de130,0x4de910,0xef06e6d3243481eaull))throw std::runtime_error("native column build differs from the USA 4de130");
        if(pc==kSprite)columns.sprite(rt,c);else columns.built(rt,c);
        return false;
    }
    if(pc==kReturn){columns.complete(c->r[29].uw[0]);return false;}
    if(pc==kEnd){
        if(value<uint32_t>(rt,0x4ddf20)!=0x03e00008u)throw std::runtime_error("native column flush return differs (the kernel patches its jr ra to j 4ddb70)");
        c->r[4].sd[0]=int32_t(columns.completion(c->r[29].uw[0]));
        return false;   // addiu sp runs; the patched jump enters 4ddb70(object)
    }
    if(!code(rt,0x4dde60,0x4ddf30,0xd0abb4d9174b01e0ull))throw std::runtime_error("native column flush differs from the USA 4dde60");
    const auto sp=c->r[29].uw[0];
    const auto [capture,k]=columns.flush(rt,c,pc);
    try{owner->drawNativeColumn(*capture,k);}catch(...){columns.cancel(sp);throw;}
    return false;   // the original delay slot and the writer's scalar flush remain
}
}

#include "native_assets.h"
#include "bozo/ee/dispatcher.h"
#include <algorithm>
#include <cstring>
#include <format>
#include <limits>
#include <stdexcept>
namespace vp2s {
namespace {
uint32_t endOf(uint32_t at,uint32_t bytes){
    if(at>=0x2000000 || bytes>0x2000000-at)throw std::runtime_error(std::format("native asset range outside EE RAM: address={:x} bytes={:x}",at,bytes));
    return at+bytes;
}
AssetOrigin advance(AssetOrigin origin,uint32_t bytes){
    if(bytes>std::numeric_limits<uint32_t>::max()-origin.offset)throw std::runtime_error("native asset offset overflow");
    origin.offset+=bytes;return origin;
}
uint32_t word(Runtime *rt,uint32_t at){
    at=NativeAssets::address(at);endOf(at,4);uint32_t value;
    std::memcpy(&value,rt->memory.hostPointer(at,4),4);return value;
}
}
uint32_t NativeAssets::address(uint32_t at){
    // The game uses physical, uncached-accelerated and KSEG aliases of EE RAM.
    // Reject other address spaces instead of wrapping arbitrary device pointers.
    const uint32_t segment=at&0xf0000000u;
    if(segment!=0 && segment!=0x20000000u && segment!=0x30000000u && segment!=0x80000000u && segment!=0xa0000000u)
        throw std::runtime_error("native asset address is not EE RAM");
    at&=0x0fffffffu;endOf(at,0);return at;
}
void NativeAssets::assign(uint32_t at,uint32_t bytes,std::optional<AssetOrigin> value){
    at=address(at);const uint32_t end=endOf(at,bytes);if(!bytes)return;
    std::vector<Range> next;next.reserve(ranges.size()+2);
    for(const auto &r:ranges){
        if(r.end<=at || r.begin>=end){next.push_back(r);continue;}
        if(r.begin<at)next.push_back({r.begin,at,r.origin});
        if(r.end>end)next.push_back({end,r.end,advance(r.origin,end-r.begin)});
    }
    if(value)next.push_back({at,end,*value});
    std::sort(next.begin(),next.end(),[](const Range &a,const Range &b){return a.begin<b.begin;});
    ranges=std::move(next);
}
void NativeAssets::checkWrite(uint32_t at,uint32_t bytes,uint32_t worker)const{
    at=address(at);const uint32_t end=endOf(at,bytes);if(!bytes)return;
    for(const auto &[job,d]:jobs)if(at<d.destination+d.size && d.destination<end && job!=worker)
        throw std::runtime_error("another producer overwrote an unfinished native asset job");
    if(worker && std::none_of(steps.begin(),steps.end(),[&](const auto &s){return s.second==worker;}))
        throw std::runtime_error("asset worker copy has no active invocation");
}
void NativeAssets::overwrite(uint32_t at,uint32_t bytes){checkWrite(at,bytes);assign(at,bytes,std::nullopt);}
void NativeAssets::discRead(uint32_t at,uint32_t sectors,uint32_t lbn){
    if(sectors>0x2000000/2048)throw std::runtime_error("native asset disc read too large");
    checkWrite(at,sectors*2048);
    assign(at,sectors*2048,AssetOrigin{AssetOrigin::Kind::Disc,uint64_t(lbn)*2048,0,0});
}
void NativeAssets::copy(uint32_t to,uint32_t from,uint32_t bytes,uint32_t worker){
    to=address(to);from=address(from);endOf(to,bytes);const uint32_t end=endOf(from,bytes);
    checkWrite(to,bytes,worker);
    // Snapshot first: an overlapping copy must not resolve its own new mapping.
    std::vector<Range> copied;
    for(const auto &r:ranges){
        const uint32_t lo=std::max(from,r.begin),hi=std::min(end,r.end);
        if(lo<hi)copied.push_back({to+lo-from,to+hi-from,advance(r.origin,lo-r.begin)});
    }
    assign(to,bytes,std::nullopt);
    for(const auto &r:copied)assign(r.begin,r.end-r.begin,r.origin);
}
std::optional<AssetOrigin> NativeAssets::origin(uint32_t at)const{
    at=address(at);
    // A sliced decode is unavailable until the original compiled worker reports
    // completion; interim raw memcpy calls cannot publish partially ready data.
    for(const auto &[job,d]:jobs)if(at>=d.destination && at<d.destination+d.size)return std::nullopt;
    for(const auto &r:ranges)if(at>=r.begin && at<r.end)return advance(r.origin,at-r.begin);
    return std::nullopt;
}
void NativeAssets::begin(uint32_t stack,uint32_t chain,uint32_t destination,uint32_t part,uint32_t job){
    if(calls.contains(stack))throw std::runtime_error("active asset invocation stack reused");
    auto source=origin(chain);if(source && source->kind!=AssetOrigin::Kind::Disc)source.reset();
    calls.emplace(stack,Invocation{source,address(destination),part,job});
}
void NativeAssets::finish(uint32_t stack,bool success,uint32_t size){
    auto it=calls.find(stack);if(it==calls.end())throw std::runtime_error("asset publication without invocation");
    const Invocation call=it->second;calls.erase(it);
    if(!success)return;
    endOf(call.destination,size);
    std::optional<AssetOrigin> result;
    if(call.chain)result=AssetOrigin{AssetOrigin::Kind::Part,call.chain->disc+call.chain->offset,call.part,0};
    if(call.job){
        if(jobs.contains(call.job))throw std::runtime_error("unfinished native asset job reused");
        overwrite(call.destination,size);jobs.emplace(call.job,Decode{call.destination,size,result});
    }else {checkWrite(call.destination,size);assign(call.destination,size,result);}
}
void NativeAssets::beginStep(uint32_t stack,uint32_t job){
    if(!steps.emplace(stack,job).second)throw std::runtime_error("active asset worker stack reused");
}
void NativeAssets::finishStep(uint32_t stack,bool complete){
    auto it=steps.find(stack);if(it==steps.end())throw std::runtime_error("asset worker return without invocation");
    const uint32_t job=it->second;steps.erase(it);if(!complete)return;
    auto found=jobs.find(job);if(found==jobs.end())return; // A job begun before provenance tracking has no inferred identity.
    const Decode decoded=found->second;jobs.erase(found);
    assign(decoded.destination,decoded.size,decoded.origin);
}
std::optional<bool> nativeAssetInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x102650 && pc!=0x102998 && pc!=0x105a30 && pc!=0x105b58 && pc!=0x105940 && pc!=0x105a10 && pc!=0x118230)return std::nullopt;
    auto *dispatcher=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=dispatcher?dynamic_cast<AssetOwner *>(&dispatcher->host()):nullptr;
    if(!owner || !c)throw std::runtime_error("native asset publication requires an owner");
    auto &assets=owner->nativeAssets();const uint32_t sp=c->r[29].uw[0];
    if(pc==0x102650)assets.begin(sp-0xa0,c->r[5].uw[0],c->r[6].uw[0],c->r[7].uw[0]);
    else if(pc==0x102998)assets.finish(sp,c->r[2].uw[0]!=0,c->r[2].uw[0]?word(rt,c->r[17].uw[0]+8):0);
    else if(pc==0x105a30)assets.begin(sp-0x60,c->r[6].uw[0],c->r[7].uw[0],c->r[8].uw[0],c->r[5].uw[0]);
    else if(pc==0x105b58)assets.finish(sp,c->r[2].uw[0]!=0,c->r[2].uw[0]?word(rt,c->r[19].uw[0]+8):0);
    else if(pc==0x105940)assets.beginStep(sp-0x30,c->r[5].uw[0]);
    else if(pc==0x105a10)assets.finishStep(sp,c->r[2].uw[0]!=0);
    else {
        // Original heap block: 0x20-byte software header, total size at +0.
        // Invalidate before coalescing changes that header. Published native
        // assets own immutable files and keep no pointer into this allocation.
        const uint32_t at=NativeAssets::address(c->r[4].uw[0]);
        if(at<0x20)throw std::runtime_error("invalid native asset heap free");
        const uint32_t bytes=word(rt,at-0x20);
        if(bytes<0x20)throw std::runtime_error("invalid native asset heap block size");
        assets.overwrite(at,bytes-0x20);
    }
    return false; // All CPU instructions, including the load at each return, remain compiled game code.
}
}

#include "native_occlusion.h"
#include "native_cast.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/ops_fpu.h"
#include <cstring>
#include <format>
#include <map>
#include <stdexcept>
namespace vp2s {
namespace {
namespace F=bozo::ps2f;
using W=OcclusionWords;
constexpr uint32_t one=0x3f800000u,minusOne=0xbf800000u;
uint32_t mul(uint32_t a,uint32_t b){uint32_t f=0;return F::mul(a,b,f);}
uint32_t add(uint32_t a,uint32_t b){uint32_t f=0;return F::addNearest(a,b,f);}
uint32_t sub(uint32_t a,uint32_t b){uint32_t f=0;return F::subNearest(a,b,f);}
// ((A c.x + B c.y) + C c.z) + D 1 per lane (MULAx, MADDAy, MADDAz, MADDw with vf0.w).
W transform(const std::array<W,4> &m,const W &c){W o;for(unsigned i=0;i<4;++i)o[i]=add(add(add(mul(m[0][i],c[0]),mul(m[1][i],c[1])),mul(m[2][i],c[2])),mul(m[3][i],one));return o;}
// SUB.xyz: the destination's w is not written (callers keep what they need).
W difference(const W &a,const W &b){return {sub(a[0],b[0]),sub(a[1],b[1]),sub(a[2],b[2]),0};}
// OPMULA.xyz ACC, a, b; OPMSUB.xyz d, b, a: d = a x b.
W cross(const W &a,const W &b){return {sub(mul(a[1],b[2]),mul(b[1],a[2])),sub(mul(a[2],b[0]),mul(b[2],a[0])),sub(mul(a[0],b[1]),mul(b[0],a[1])),0};}
}
NativeOccluder nativeOccluder(const std::array<W,4> &world,const std::array<W,4> &local,const W &eye,const std::array<W,4> &rows){
    // Program 0x1f0: T_j lane i = world_i lane j; the diagonal moved raw, the W row (i = 3) 1 * x, the rest 0 + x.
    std::array<W,4> t;
    for(unsigned j=0;j<4;++j)for(unsigned i=0;i<4;++i)t[j][i]=i==j?world[i][j]:i==3?mul(one,world[i][j]):add(0,world[i][j]);
    NativeOccluder o;
    for(unsigned k=0;k<4;++k)o.corners[k]=transform(t,local[k]);
    const auto &p=o.corners;
    const W e0=difference(p[0],eye),e1=difference(p[1],eye),e2=difference(p[2],eye),e3=difference(p[3],eye);
    std::array<W,5> n={cross(difference(p[1],p[0]),e0),cross(difference(p[2],p[1]),e1),cross(difference(p[3],p[2]),e2),
                       cross(difference(p[0],p[3]),e3),cross(difference(p[0],p[1]),difference(p[2],p[0]))};
    for(unsigned k=0;k<5;++k){
        // MUL.xyz squares, ADDA (x + y), MADD (+ 1 z) into .x; RSQRT Q = 1 / sqrt; MULq.xyz.
        const uint32_t length=add(add(mul(n[k][0],n[k][0]),mul(n[k][1],n[k][1])),mul(one,mul(n[k][2],n[k][2])));
        uint32_t f=0;const uint32_t q=F::rsqrt(one,length,f);
        for(unsigned c=0;c<3;++c)o.normals[k][c]=mul(n[k][c],q);
    }
    for(unsigned k=0;k<4;++k)o.clip[k]=transform(rows,p[k]);
    o.face=cross(difference(o.clip[1],o.clip[0]),difference(o.clip[2],o.clip[0]));
    return o;
}
W nativeOccluderProjection(const W &c){uint32_t f=0;const uint32_t q=F::div(one,c[2],f);return {mul(c[0],q),mul(c[1],q),c[2],c[3]};}
bool nativeOccluded(const W &s,const std::array<W,9> &r){
    // MAXw vf31 = 1; v_k = s - p_k; products with n0..n3 and (s - p0) with n4.
    const W v0=difference(s,r[0]),v1=difference(s,r[1]),v2=difference(s,r[2]),v3=difference(s,r[3]);
    const auto product=[](const W &a,const W &b){return W{mul(a[0],b[0]),mul(a[1],b[1]),mul(a[2],b[2]),0};};
    const W a=product(v0,r[4]),b=product(v1,r[5]),c=product(v2,r[6]),d=product(v3,r[7]),e=product(v0,r[8]);
    // ADDAy.x / MADDz.x, ADDAx.y / MADDz.y, ADDAx.z / MADDy.z, ADDAy.x / MADDz.x, ADDAx.y / MADDz.y (the A operand first).
    const uint32_t distance[5]={add(add(a[0],a[1]),mul(one,a[2])),add(add(b[1],b[0]),mul(one,b[2])),add(add(c[2],c[0]),mul(one,c[1])),
                                add(add(d[0],d[1]),mul(one,d[2])),add(add(e[1],e[0]),mul(one,e[2]))};
    // FSSET 0, then SUBw (xyz, then xy) against the radius: the sticky sign flag is any result's sign.
    for(uint32_t x:distance)if(sub(x,s[3])&0x80000000u)return false;
    return true;
}
namespace {
struct Pass {
    bool open=false;std::array<W,4> rows{};unsigned vi1=0;std::array<W,256> memory{};W sphere{};uint32_t vi2=0;
    // The leaf in flight.
    bool leaf=false;uint32_t object=0,stack=0;NativeOccluder occluder;bool negated=false;
};
std::map<const Runtime *,Pass> &passes(){static std::map<const Runtime *,Pass> p;return p;}
W words(Runtime *rt,uint32_t at){const auto *p=rt->memory.hostPointer(at,16);if(!p || (at&15))throw std::runtime_error(std::format("native occlusion record {:x} outside owned memory",at));W w;std::memcpy(w.data(),p,16);return w;}
void store(Runtime *rt,uint32_t at,const W &w){auto *p=rt->memory.hostPointer(at,16);if(!p || (at&15))throw std::runtime_error(std::format("native occlusion store {:x} outside owned memory",at));std::memcpy(p,w.data(),16);}
bool cop2(uint32_t w){const auto op=w>>26;return op==0x12 || op==0x36 || op==0x3e;}
}
bool nativeOcclusionOpen(const Runtime *rt){auto &p=passes();const auto it=p.find(rt);return it!=p.end() && it->second.open;}
std::optional<bool> nativeOcclusionInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool rowsLoad=pc>=0x22293c && pc<=0x222948,leafSite=pc>=0x222b18 && pc<=0x22302c,
               test=pc==0x2201d4 || pc==0x2201d8 || pc==0x220204 || pc==0x220208 || pc==0x220218,close=pc==0x220294;
    if(!rt || (pc&3) || !(rowsLoad || leafSite || test || close))return std::nullopt;
    uint32_t word=0;if(const auto *p=rt->memory.hostPointer(pc,4))std::memcpy(&word,p,4);
    if(close){if(word!=0x8ea44368u)return std::nullopt;passes()[rt].open=false;return false;}   // the original lw runs
    if(!cop2(word))return std::nullopt;
    if(!c)throw std::runtime_error("native occlusion has no CPU record");
    auto &st=passes()[rt];const auto sp=c->r[29].uw[0];
    if(rowsLoad){
        if(pc==0x22293c){st.leaf=false;
            const auto base=c->r[18].uw[0];for(unsigned k=0;k<4;++k)st.rows[k]=words(rt,base+16*k);st.open=true;}
        return true;
    }
    if(test){
        if(!st.open)throw std::runtime_error("native occlusion test outside its pass");
        switch(pc){
        case 0x2201d4:st.sphere=words(rt,c->r[2].uw[0]);break;
        case 0x2201d8:if(word!=0x4a010232u)throw std::runtime_error("native occlusion index word differs");st.vi1=8;break;
        case 0x220204:if(word!=0x4a010a72u)throw std::runtime_error("native occlusion index word differs");st.vi1=(st.vi1+9)&0xffff;break;
        case 0x220208:{if(word!=0x4a006af8u)throw std::runtime_error("native occlusion program differs");
            std::array<W,9> record;for(unsigned k=0;k<9;++k)record[k]=st.memory[(st.vi1+k)&0xff];
            st.vi2=nativeOccluded(st.sphere,record)?0u:0x80u;st.vi1=(st.vi1+9)&0xffff;break;}
        case 0x220218:c->r[2].ud[0]=st.vi2;break;
        }
        return true;
    }
    // The leaf 222b10 (a0 object, a1 the camera position, a2 the record index).
    if(pc==0x222b18){
        if(!st.open || c->r[2].uw[0]!=c->r[4].uw[0]+0x90)throw std::runtime_error("native occluder outside its pass or input");
        const auto object=c->r[4].uw[0];std::array<W,4> world,local;
        for(unsigned k=0;k<4;++k){world[k]=words(rt,object+0x90+16*k);local[k]=words(rt,object+0x220+16*k);}
        st.occluder=nativeOccluder(world,local,words(rt,c->r[5].uw[0]),st.rows);
        st.occluder.face[3]=local[0][3];   // SUB.xyz / OPMSUB.xyz keep vf20's w: the first corner's
        st.leaf=true;st.object=object;st.stack=sp;st.negated=false;return true;
    }
    if(!st.leaf || st.stack!=sp)throw std::runtime_error(std::format("native occluder site {:x} outside its leaf",pc));
    const auto &o=st.occluder;
    switch(pc){
    case 0x222b28:st.vi1=c->r[6].uw[0]&0xffff;break;   // CTC2 a2, vi1
    case 0x222b94:case 0x222b98:case 0x222b9c:case 0x222ba0:st.memory[st.vi1&0xff]=o.corners[(pc-0x222b94)/4];st.vi1=(st.vi1+1)&0xffff;break;
    case 0x222cb4:case 0x222cb8:case 0x222cd4:case 0x222cd8:{
        const unsigned k=pc==0x222cb4?0:pc==0x222cb8?1:pc==0x222cd4?2:3;
        if(c->r[2].uw[0]!=sp)throw std::runtime_error("native occluder corner store escaped its stack");store(rt,sp+16*k,o.clip[k]);break;}
    case 0x222cdc:if(c->r[2].uw[0]!=sp)throw std::runtime_error("native occluder face store escaped its stack");store(rt,sp+0x40,o.face);break;
    case 0x222d48:st.negated=true;break;   // MULx.xyz by vf30.x = 0 - 1 (the face points away)
    case 0x222d70:case 0x222d74:case 0x222d78:case 0x222d7c:case 0x222d80:{
        W n=o.normals[(pc-0x222d70)/4];if(st.negated)for(unsigned i=0;i<3;++i)n[i]=mul(n[i],minusOne);
        st.memory[st.vi1&0xff]=n;st.vi1=(st.vi1+1)&0xffff;break;}
    case 0x222db0:case 0x222e88:case 0x222f60:case 0x22302c:{
        const unsigned k=pc==0x222db0?0:pc==0x222e88?1:pc==0x222f60?2:3;
        if(c->r[2].uw[0]!=sp)throw std::runtime_error("native occluder projection escaped its stack");
        store(rt,sp,nativeOccluderProjection(o.clip[k]));if(k==3)st.leaf=false;break;}
    default:break;   // loads, the program call, the arithmetic and the Q waits: folded into the stores above
    }
    // The leaf returns early (an outcode 0, or a single-sided back face) past no later COP2 site; the next leaf or
    // pass restarts the capture.
    return true;
}
}

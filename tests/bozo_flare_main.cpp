#include "native_flare.h"
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <stdexcept>
#include <string>
namespace {
void check(bool ok,const std::string &what){if(!ok)throw std::runtime_error(what);}
uint32_t w(float v){return std::bit_cast<uint32_t>(v);}
float f(uint32_t v){return std::bit_cast<float>(v);}
// Taylor coefficients in the game's layout (x^13 .. x^3); the trigonometric leaf has its own tests.
vp2s::TrigCoefficients trig(){vp2s::TrigCoefficients c;c.pi=3.14159265f;c.inversePi=1/3.14159265f;c.halfPi=1.57079633f;
    c.sine={1.f/6227020800.f,-1.f/39916800.f,1.f/362880.f,-1.f/5040.f,1.f/120.f,-1.f/6.f};return c;}
struct Element{uint8_t bytes[0x48]{};void put(unsigned at,uint32_t v){std::memcpy(bytes+at,&v,4);}void putf(unsigned at,float v){put(at,w(v));}};
uint32_t scaled(uint32_t c,uint32_t o,uint32_t a){uint32_t out=0;for(unsigned i=0;i<3;++i)out|=vp2s::nativeFlareChannel((c>>(8*i))&0xff,(o>>(8*i))&0xff)<<(8*i);return out|a<<24;}
// The listings' geometry in double precision: a point's 12.4 position truncated toward zero.
double fixed(double v){return std::trunc(v*16)/16;}
bool near(float got,double want){return std::fabs(got-want)<=1.0/16+1e-6*std::fabs(want);}
void position(const bozo::render::Vertex &v,double x,double y,const std::string &what){
    if(!near(v.x,fixed(x)) || !near(v.y,fixed(y)))throw std::runtime_error(what+": ("+std::to_string(v.x)+", "+std::to_string(v.y)+") / ("+std::to_string(fixed(x))+", "+std::to_string(fixed(y))+")");}
uint32_t alpha(double intensity,float strength){const auto v=int64_t(128.f*float(intensity*strength));return v<0 || v>128?128:uint32_t(v);}
}
int main(){try{
    const auto t=trig();std::mt19937 random(0xf1a2e);std::uniform_real_distribution<float> unit(0,1);
    unsigned discs=0,rings=0,rays=0,triangles=0,culled=0;
    for(unsigned c=0;c<300;++c){
        vp2s::NativeFlareCall call;const float cx=1800+unit(random)*500,cy=1900+unit(random)*300,scale=0.5f+unit(random),offset=unit(random)*6,intensity=0.2f+unit(random)*0.8f,
            sx=0.8f+unit(random)*0.4f,sy=0.8f+unit(random)*0.4f;
        call.f={w(cx),w(cy),w(scale),w(offset),w(intensity),w(sx),w(sy)};call.colour=0x204080u+c;
        Element e;e.putf(0x10,0.5f+unit(random));for(unsigned k=0;k<5;++k)e.put(0x14+4*k,0x10203040u*(k+1)+c);
        const auto a=alpha(1.0,128.f*(intensity*std::bit_cast<float>(*reinterpret_cast<const uint32_t *>(e.bytes+0x10)))/128.f);
        const unsigned kind=c%3;
        if(kind==0){   // disc 2550d0
            for(unsigned k=0;k<3;++k)e.putf(0x24+4*k,10+unit(random)*60);
            const double r1=double(f(*reinterpret_cast<uint32_t *>(e.bytes+0x24)))*scale,e28=f(*reinterpret_cast<uint32_t *>(e.bytes+0x28)),e2c=f(*reinterpret_cast<uint32_t *>(e.bytes+0x2c));
            const double r2=(e28+r1/scale)*scale,r3=(r1/scale+(e2c+e28))*scale;
            const auto out=vp2s::nativeFlareDisc(e.bytes,call,t);
            if(cx-r3<0 || cx+r3>4095 || cy-r3<0 || cy+r3>4095){check(out.empty(),"off-screen disc drawn");++culled;continue;}
            int n=int(0.2*r2)+9;if(n>48)n=48;
            check(out.size()==size_t(3*((n-1)+2*(2*n-2))),"disc triangle count");
            for(int i=0;i<n-1;++i){
                const auto at=[&](int k){const auto sc=vp2s::sineCosine(float(offset+2*3.14159265358979*k/(n-1)),t);return std::array<double,2>{sc[2]*double(sx),sc[3]*double(sy)};};
                const auto p=at(i),q=at(i+1);const auto *v=&out[3*i];
                position(v[0],cx,cy,"disc centre");position(v[1],cx+r1*p[0],cy+r1*p[1],"disc rim");position(v[2],cx+r1*q[0],cy+r1*q[1],"disc rim next");
                check(v[0].rgba==scaled(*reinterpret_cast<uint32_t *>(e.bytes+0x14),call.colour,a) && v[1].rgba==scaled(*reinterpret_cast<uint32_t *>(e.bytes+0x18),call.colour,a),"disc fan colours");}
            // The first ring's colours are the element's own words (not scaled); the second's outer edge has alpha 0.
            const auto *ring1=&out[3*(n-1)],*ring2=&out[3*(n-1)+3*(2*n-2)];
            check(ring1[0].rgba==(*reinterpret_cast<uint32_t *>(e.bytes+0x18)|a<<24) && ring1[1].rgba==(*reinterpret_cast<uint32_t *>(e.bytes+0x1c)|a<<24),"disc ring 1 colours");
            check(ring2[0].rgba==scaled(*reinterpret_cast<uint32_t *>(e.bytes+0x1c),call.colour,a) && ring2[1].rgba==scaled(*reinterpret_cast<uint32_t *>(e.bytes+0x20),call.colour,0),"disc ring 2 colours");
            const auto p0=vp2s::sineCosine(offset,t);position(ring2[1],cx+r3*p0[2]*double(sx),cy+r3*p0[3]*double(sy),"disc outer edge");
            triangles+=unsigned(out.size()/3);++discs;
        }else if(kind==1){   // rings 2549a0
            float total=0;for(unsigned k=0;k<5;++k){const float v=5+unit(random)*25;e.putf(0x28+4*k,v);total+=v;}
            const double outer=double(total)*scale;
            const auto out=vp2s::nativeFlareRings(e.bytes,call,t);
            if(cx-outer<0 || cx+outer>4095 || cy-outer<0 || cy+outer>4095){check(out.empty(),"off-screen rings drawn");++culled;continue;}
            int n=int(0.2*outer)+9;if(n>48)n=48;
            check(out.size()==size_t(4*3*(2*n-2)),"ring triangle count");
            double radius=f(*reinterpret_cast<uint32_t *>(e.bytes+0x28));
            for(unsigned k=0;k<4;++k){const auto *s=&out[3*(2*n-2)*k];const double width=f(*reinterpret_cast<uint32_t *>(e.bytes+0x2c+4*k));
                const auto p=vp2s::sineCosine(offset,t);
                position(s[0],cx+scale*radius*p[2]*sx,cy+scale*radius*p[3]*sy,"ring inner");position(s[1],cx+scale*(radius+width)*p[2]*sx,cy+scale*(radius+width)*p[3]*sy,"ring outer");
                check(s[0].rgba==scaled(*reinterpret_cast<uint32_t *>(e.bytes+0x14+4*k),call.colour,k?a:0) && s[1].rgba==scaled(*reinterpret_cast<uint32_t *>(e.bytes+0x18+4*k),call.colour,k==3?0:a),"ring colours");
                radius+=width;}
            triangles+=unsigned(out.size()/3);++rings;
        }else{   // rays 253770
            const unsigned count=1+c%5,segments=1+c%8;e.put(0x1c,count);e.put(0x20,segments);e.putf(0x3c,0.05f+unit(random)*0.2f);e.putf(0x40,40+unit(random)*80);
            std::vector<uint8_t> table(0x44*count);for(unsigned r=0;r<count;++r){float v[17];v[0]=unit(random)*6;for(unsigned k=1;k<17;++k)v[k]=10+unit(random)*90;std::memcpy(&table[0x44*r],v,0x44);}
            const double s=scale*(0.5+0.5*double(intensity));const double reach=double(f(*reinterpret_cast<uint32_t *>(e.bytes+0x40)))*s;
            const auto out=vp2s::nativeFlareRays(e.bytes,table.data(),call,t);
            if(cx-reach<0 || cx+reach>4095 || cy-reach<0 || cy+reach>4095){check(out.empty(),"off-screen rays drawn");++culled;continue;}
            check(out.size()==size_t(6*count*segments),"ray triangle count");
            const double width=f(*reinterpret_cast<uint32_t *>(e.bytes+0x3c));
            for(unsigned r=0;r<count;++r){float v[17];std::memcpy(v,&table[0x44*r],0x44);const double base=offset+double(v[0]);
                for(unsigned k=0;k<segments;++k){const auto *q=&out[6*(r*segments+k)];
                    const auto dir=[&](double angle){const auto sc=vp2s::sineCosine(float(angle),t);return std::array<double,2>{sc[2]*double(sx),sc[3]*double(sy)};};
                    const auto pa=dir(base+k*width),pc=dir(base+k*width+width/2),pb=dir(base+(k+1)*width);
                    const double ra=s*v[9+(k?k-1:0)],rb=s*v[9+k],rc=s*v[1+k];
                    position(q[0],cx,cy,"ray centre");position(q[1],cx+ra*pa[0],cy+ra*pa[1],"ray edge A");position(q[2],cx+rb*pb[0],cy+rb*pb[1],"ray edge B");
                    position(q[5],cx+rc*pc[0],cy+rc*pc[1],"ray tip");
                    check(q[0].rgba==scaled(*reinterpret_cast<uint32_t *>(e.bytes+0x18),call.colour,a) && q[1].rgba==scaled(*reinterpret_cast<uint32_t *>(e.bytes+0x14),call.colour,a) &&
                          q[5].rgba==scaled(*reinterpret_cast<uint32_t *>(e.bytes+0x14),call.colour,0),"ray colours");}}
            triangles+=unsigned(out.size()/3);++rays;
        }
    }
    check(discs>40 && rings>40 && rays>40,"case coverage");
    // A light whose outer radius leaves the window (x - r < 0) is not drawn by any writer.
    {vp2s::NativeFlareCall call;call.f={w(20.f),w(2000.f),w(1.f),0,w(1.f),w(1.f),w(1.f)};Element e;e.putf(0x10,1);
     for(unsigned k=0;k<5;++k)e.putf(0x24+4*k,30);e.put(0x1c,1);e.put(0x20,1);e.putf(0x40,40);uint8_t table[0x44]{};
     check(vp2s::nativeFlareDisc(e.bytes,call,t).empty() && vp2s::nativeFlareRings(e.bytes,call,t).empty() && vp2s::nativeFlareRays(e.bytes,table,call,t).empty(),"edge light culled");
     call.f[0]=w(2000.f);check(!vp2s::nativeFlareDisc(e.bytes,call,t).empty(),"centred light drawn");culled+=3;}
    // The flash: 640 / 64 = 10 sprites of 64 x 448 from (1728, 1824), alpha FTOI(128 f).
    const auto flash=vp2s::nativeFlareFlash(0x102030,w(0.75f),640,448);check(flash.size()==60,"flash sprite count");
    check(flash[0].x==1728 && flash[0].y==1824 && flash[4].x==1792 && flash[4].y==2272 && flash[0].rgba==(0x102030u|96u<<24),"flash geometry");
    // The colour scale: 255 x 255 stays 255, 255 x 128 is 128, 0 is 0.
    check(vp2s::nativeFlareChannel(255,255)==255 && vp2s::nativeFlareChannel(255,128)==128 && vp2s::nativeFlareChannel(0,200)==0,"colour scale");
    std::printf("%u discs, %u ring sets, %u ray sets (%u triangles, %u culled off screen) match the writers' listings; flash and colour scale\n",discs,rings,rays,triangles,culled);
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

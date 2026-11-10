#include "native_screen_zoom.h"
#include <bit>
#include <cmath>
#include <cstdio>
#include <stdexcept>
namespace {
void check(bool ok,const char *what){if(!ok)throw std::runtime_error(what);}
uint32_t word(float v){return std::bit_cast<uint32_t>(v);}
// The builder 374c20 restated in double precision from the listing: pages of
// 64x32 over 640 x 224*f9; zoomed about (320, 112) by obj+a4, else shifted by
// obj+b0/b4; screen scales W/W0 and H/H0*(1+fields); origin 0x800 - size/2.
struct Expected{double x0,y0,x1,y1;};
Expected expected(const vp2s::NativeScreenZoomInputs &in,unsigned column,unsigned row){
    const double sx=double(in.width)/in.baseWidth,sy=double(in.height)/in.baseHeight*(in.fields?2:1),f9=in.fields?2:1;
    const double z=std::bit_cast<float>(in.zoom)-1.0,bx=std::bit_cast<float>(in.shiftX),by=std::bit_cast<float>(in.shiftY);
    const double ox=0x800-in.width/2,oy=0x800-in.height/2,x=64.0*column,y=32.0*row;
    const auto zx=[&](double v){return in.zoomed?sx*(v+(v*z-320*z))+ox:sx*(v+bx)+ox;};
    const auto zy=[&](double v){return in.zoomed?sy*(v+(v*z-112*z))/f9+oy:sy*(v+by)/f9+oy;};
    return {zx(x),zy(y),zx(x+64),zy(y+32)};
}
}
int main(){try{
    unsigned strips=0;double worst=0;
    for(unsigned c=0;c<48;++c){
        vp2s::NativeScreenZoomInputs in{.visibility=word(float(c%10)/10.f),.zoom=word(1.f+float(c%7)/20.f),.shiftX=word(float(int(c%5)-2)*3.25f),
            .shiftY=word(float(int(c%3)-1)*5.5f),.zoomed=(c&1)!=0,.fields=(c&2)!=0,.width=640,.height=int16_t(c&2?448:224),.baseWidth=640,.baseHeight=int16_t(c&2?448:224)};
        if(c%8==5){in.baseWidth=512;in.baseHeight=int16_t(c&2?384:192);}
        if(c%4==3){in.width=512;in.height=512;}   // event camera target
        const auto zoom=vp2s::nativeScreenZoomGeometry(in);
        const unsigned rows=in.fields?14:7;
        check(zoom.strips.size()==rows*10 && zoom.width==in.width && zoom.height==in.height && zoom.sourceWidth==640 && zoom.sourceHeight==int32_t(rows*32),"strip layout");
        // The GS modulates the vertex alpha (1-v)*160 with TEXA's 0x60.
        const unsigned vertex=unsigned((1.0f-std::bit_cast<float>(in.visibility))*160.0f);
        check(std::abs(int(zoom.alpha)-int((0x60*vertex)>>7))<=1,"alpha");
        for(const auto &s:zoom.strips){
            check(s.column<10 && s.row<rows && &s-&zoom.strips[0]==s.row*10+s.column,"page order");
            const auto e=expected(in,s.column,s.row);
            for(const auto [got,want]:{std::pair{s.x0,e.x0},{s.y0,e.y0},{s.x1,e.x1},{s.y1,e.y1}}){
                const double err=std::abs(got-want);worst=std::max(worst,err);check(err<=0.125,"geometry");}
            ++strips;
        }
        const auto quad=vp2s::nativeScreenZoomVertices(zoom,zoom.strips.back());
        check(quad[0].u==576.f/640 && quad[4].u==1.f && quad[4].v==1.f && quad[0].z==float(0xfffe06) && (quad[0].rgba>>24)==zoom.alpha,"vertices");
    }
    bool rejected=false;try{vp2s::nativeScreenZoomGeometry({.width=512,.height=224,.baseWidth=640,.baseHeight=224,.sourcePages=8});}catch(const std::runtime_error &){rejected=true;}
    check(rejected,"frame layout rejection");
    std::printf("%u screen zoom strips match the builder's geometry (worst %.4f px) and the frame-layout rejection holds\n",strips,worst);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

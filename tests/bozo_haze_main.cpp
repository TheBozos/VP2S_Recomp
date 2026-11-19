// Lost Forest haze: the depth -> ramp index the GS packet builds, and the VU1
// vertex scattering colour. No game data: synthetic constants in the program's
// layout; the expected colours come from running the VU1 program's 15f0 path on
// them (an analysis interpreter outside the runtime).
#include "native_haze.h"
#include <bit>
#include <cmath>
#include <cstdio>
#include <source_location>
#include <stdexcept>
#include <string>
namespace {
void check(bool v,std::source_location at=std::source_location::current()){if(!v)throw std::runtime_error("haze assertion at "+std::to_string(at.line()));}
// The GS passes as byte operations, step by step (independent of hazeRampIndex).
unsigned gsIndex(uint32_t z,unsigned shift=5){
    uint8_t blue=uint8_t(z>>16),alpha=0;                     // pass 1: Z24 copied as colour, TEXA alpha 0
    const uint16_t low=uint16_t(z),high=uint16_t(blue|alpha<<8);
    uint16_t written=high;
    if(low&0x8000)written=uint16_t((high&0x3fff)|(low&0xc000)); // pass 2: As 0x80 copies bits 14..15 (FBMSK 0x3fff)
    blue=uint8_t(written);alpha=uint8_t(written>>8);
    for(unsigned i=0;i<shift;++i)blue=uint8_t(std::min(255,2*blue));   // ALPHA 0x49 (Cd*As + Cd, As 0x80), clamped
    const unsigned added=std::min(255u,unsigned(blue)+((0x10u<<(shift-5)>>0)*alpha>>7)); // ALPHA 0x58 (Cs*Ad + Cd), Cs = 2^(shift-1)
    const unsigned mask=(1u<<shift)-1;return (blue&~mask&0xffu)|(added&mask);   // FBMSK: blue's low shift bits only
}
}
int main(){try{
    // Every 24-bit depth: the step-by-step GS result, which only depends on Z >> 14.
    unsigned saturated=0;
    for(uint32_t k=0;k<1024;++k){const unsigned expected=gsIndex(k<<14);if(expected==255)++saturated;
        for(uint32_t z=k<<14;z<(k+1)<<14;++z)check(vp2s::hazeRampIndex(z)==expected && gsIndex(z)==expected);}
    check(vp2s::hazeRampIndex(0)==0 && vp2s::hazeRampIndex(0x8000)==0x10 && vp2s::hazeRampIndex(0xc000)==0x18 && vp2s::hazeRampIndex(0x70000)==0xe0);
    check(saturated==1024-32);
    // Shift 8 (deeper in the forest: 8 doublings, add 0x80 * Ad / 128, all of blue writable).
    for(uint32_t k=0;k<1024;++k){const unsigned expected=gsIndex(k<<14,8);for(uint32_t z=k<<14;z<(k+1)<<14;z+=97)check(vp2s::hazeRampIndex(z,8)==expected);}
    check(vp2s::hazeRampIndex(0,8)==0 && vp2s::hazeRampIndex(0x8000,8)==0x80 && vp2s::hazeRampIndex(0xc000,8)==0xc0 && vp2s::hazeRampIndex(0x10000,8)==255);
    // Vertex colours: synthetic constants, expected bytes from the VU1 program.
    std::array<std::array<float,4>,25> c{};
    c[1]={0.3f,-0.8f,-0.52f,-1};c[2]={-50,-20,-80,1};c[3]={180,120,60,0};c[4]={0.0008f,0.0013f,0.0026f,0};c[5]={0.03f,0.009f,0.005f,1.5f};
    c[7]={0.0597f,0.08f,1.04f,0.4f};c[8]={0,0.007f,1.49f,-1.4f};c[9]={0.6f,-0.4f,-1.5f,0};c[10]={0.001f,0.002f,0.0005f,0};
    struct Case{std::array<float,4> point;uint32_t rgba;};
    const Case cases[]={{{120,-40,300,1},0x80000867u},{{-200,35,90,1},0x80000b8fu},{{10,-300,40,1},0x80000540u},
                        {{400,60,-500,1},0x80000543u},{{-90,-10,600,1},0x8000097cu},{{0,5,-81,1},0x80000759u}};
    unsigned exact=0;
    for(const auto &k:cases){const auto got=vp2s::hazeVertexColour(c,k.point);
        // The VU1 evaluates ln/exp by short series on its EFU; a byte may differ by one.
        if(got!=k.rgba)std::printf("vertex colour %08x, program %08x\n",got,k.rgba);for(unsigned i=0;i<4;++i){const int a=int((got>>(8*i))&0xff),b=int((k.rgba>>(8*i))&0xff);check(std::abs(a-b)<=1);}
        exact+=got==k.rgba;}
    // The light colour scales linearly; zero light keeps only the ramp (alpha 0x80).
    auto dark=c;dark[3]={0,0,0,0};check(vp2s::hazeVertexColour(dark,{120,-40,300,1})==0x80000000u);
    auto bright=c;bright[3]={1e6f,1e6f,1e6f,0};check((vp2s::hazeVertexColour(bright,{120,-40,300,1})&0xffff)==0xffffu);
    // Rejections: a point at the camera, a non-finite point, a Mie base at or below zero, a negative colour.
    unsigned rejected=0;
    const auto reject=[&](auto f){try{f();}catch(const std::runtime_error &){++rejected;return;}check(false);};
    reject([&]{vp2s::hazeVertexColour(c,{-50,-20,-80,1});});
    reject([&]{vp2s::hazeVertexColour(c,{std::bit_cast<float>(0x7fc00000u),0,0,1});});
    reject([&]{auto b=c;b[7][2]=0.3f;b[7][3]=1;vp2s::hazeVertexColour(b,{-50-3.f,-20+8.f,-80+5.2f,1});}); // ray against the light: cos -1
    reject([&]{auto b=c;b[3]={-100,0,0,0};vp2s::hazeVertexColour(b,{120,-40,300,1});});
    std::printf("haze: 2^24 depth indices (%u saturated groups), %zu vertex colours (%u exact), %u rejections\n",saturated,std::size(cases),exact,rejected);
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

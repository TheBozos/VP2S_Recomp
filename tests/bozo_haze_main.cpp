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
    const unsigned added=std::min(255u,unsigned(blue)+(((1u<<(shift-1)))*alpha>>7)); // ALPHA 0x58 (Cs*Ad + Cd), Cs = 2^(shift-1)
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
    // Shift 3 (maps 392, 407, 414, 426: three doublings, add 4 * Ad / 128 into blue's low three bits).
    for(uint32_t k=0;k<1024;++k){const unsigned expected=gsIndex(k<<14,3);for(uint32_t z=k<<14;z<(k+1)<<14;z+=97)check(vp2s::hazeRampIndex(z,3)==expected);}
    check(vp2s::hazeRampIndex(0x8000,3)==4 && vp2s::hazeRampIndex(0xc000,3)==6 && vp2s::hazeRampIndex(0x10000,3)==8);
    // Vertex colours: synthetic constants, expected bytes from the VU1 program.
    std::array<std::array<float,4>,25> c{};
    c[1]={0.3f,-0.8f,-0.52f,-1};c[2]={-50,-20,-80,1};c[3]={180,120,60,0};c[4]={0.0008f,0.0013f,0.0026f,0};c[5]={0.03f,0.009f,0.005f,1.5f};
    c[7]={0.0597f,0.08f,1.04f,0.4f};c[8]={0,0.007f,1.49f,-1.4f};c[9]={0.6f,-0.4f,-1.5f,0};c[10]={0.001f,0.002f,0.0005f,0};
    c[11][2]=std::bit_cast<float>(1u);   // 11.z: the elevation mask path (the forest)
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
    // The gamma path (11.y = 1, 11.z = 0; maps 259-271, 535): 256 * exp(11.x ln(scatter / 2)) by the program's series:
    // ln's coefficient qwords 13.. are 2 / (2k + 1) (12.x = 2: three of them), exp's 21.. are 1 / n! (12.y = 1: two).
    auto g=c;g[11]={0.4f,std::bit_cast<float>(1u),0,0};g[12]={std::bit_cast<float>(2u),std::bit_cast<float>(1u),0,0};
    for(unsigned k=0;k<32;++k)g[13+k/4][k%4]=2.f/float(2*k+3);
    {float f=1;for(unsigned k=0;k<12;++k){f*=float(k+2);g[21+k/4][k%4]=1.f/f;}}
    // With no light (scatter 0) the series give 0.1655, not 0: byte 42, as the VU1 analysis interpreter computes it on the
    // program's own constants (map 259's 2 / (2k + 1) and 1 / n! words).
    auto gd=g;gd[3]={0,0,0,0};check(vp2s::hazeVertexColour(gd,{120,-40,300,1})==0x802a2a2au);
    // Near 1 the series match ln / exp; the program's exp multiplies its second batch by the argument, not its fourth power.
    check(std::abs(vp2s::hazeSeriesLn(g,1.5f)-std::log(1.5f))<1e-5f && std::abs(vp2s::hazeSeriesExp(g,0.5f)-std::exp(0.5f))<1e-3f);
    {const auto lit=vp2s::hazeVertexColour(g,{120,-40,300,1});for(unsigned i=0;i<3;++i)check(((lit>>(8*i))&0xff)>42);}
    // 8.x: the second lobe 8.y / (8.z + 8.w cos)^1.5 weighted by qword 6, and no clamp of cos by 1.w.
    {auto b=c;b[8][0]=std::bit_cast<float>(1u);b[6]={0.01f,0.02f,0.03f,0};const auto two=vp2s::hazeVertexColour(b,{120,-40,300,1});
     auto z=b;z[6]={0,0,0,0};z[1][3]=0.99f;   // without the weights and with a high clamp the lobe-free colour ignores the clamp
     auto u=c;u[1][3]=0.99f;check(two!=vp2s::hazeVertexColour(c,{120,-40,300,1}) && vp2s::hazeVertexColour(z,{120,-40,300,1})!=vp2s::hazeVertexColour(u,{120,-40,300,1}));}
    // Both (11.y and 11.z): the mask scales the scatter before the gamma.
    {auto b=g;b[11][2]=std::bit_cast<float>(1u);const auto both=vp2s::hazeVertexColour(b,{120,-40,300,1}),plain=vp2s::hazeVertexColour(g,{120,-40,300,1});check(both!=plain);}
    reject([&]{auto b=g;b[12][0]=std::bit_cast<float>(8u);vp2s::hazeVertexColour(b,{120,-40,300,1});});   // ln series past its constants
    std::printf("haze: 2^24 depth indices (%u saturated groups), %zu vertex colours (%u exact), %u rejections\n",saturated,std::size(cases),exact,rejected);
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}

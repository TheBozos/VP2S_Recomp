// The Lost Forest haze as one native depth-colour pass (see native_haze.h).
#include "native_haze.h"
#include "bozo/ee/dispatcher.h"
#include <cmath>
#include <cstring>
#include <format>
namespace vp2s {
namespace {
template<class T>T value(Runtime *rt,uint32_t at){
    if(!rt || at<0x100000 || uint64_t(at)+sizeof(T)>0x2000000)throw std::runtime_error("native haze record outside EE RAM");
    T v;std::memcpy(&v,rt->memory.hostPointer(at,sizeof(T)),sizeof(T));return v;
}
std::array<float,4> quad(Runtime *rt,uint32_t at){return value<std::array<float,4>>(rt,at);}
float finite(float v){if(!std::isfinite(v) || std::abs(v)>1.e12f)throw std::runtime_error("unqualified native haze input");return v;}
constexpr uint32_t kClass=0x15a020,kNode=0x15a09c;
}
unsigned hazeRampIndex(uint32_t z,unsigned shift){
    if(shift<1 || shift>8)throw std::runtime_error("native haze shift outside 1..8");
    // Pass 1 copies the depth tile as colour (blue = Z bits 16..23, alpha 0).
    // Pass 2 views it as 16-bit pixels: sprites shifted by 8 columns copy each
    // pixel's low half onto its high half, bits 14..15 only, where bit 15 is set
    // (TEXA alpha 0x80): alpha becomes 0x80 | 0x40 * bit 14. Five ALPHA 0x49
    // sprites double blue with clamping; one ALPHA 0x58 sprite adds 0x10 * Ad /
    // 128 to blue's low five bits. Pass 3 reads blue through the ramp.
    z&=0xffffff;const unsigned b=(z>>16)&0xff;
    unsigned blue=std::min(255u,b<<shift);
    const unsigned alpha=(z&0x8000)?(0x80u|((z&0x4000)?0x40u:0u)):0u;
    const unsigned mask=(1u<<shift)-1,sum=std::min(255u,blue+(((1u<<(shift-1))*alpha)>>7));
    return (blue&~mask&0xffu)|(sum&mask);
}
uint32_t hazeVertexColour(const std::array<std::array<float,4>,25> &c,const std::array<float,4> &point){
    // VU1 1410/15f0, 17f8, 18c8 (the vertex pipelines one call behind; each
    // colour depends on its own point only). r: camera (qword 2) to the point.
    double r[3],length2=0;for(unsigned i=0;i<3;++i){r[i]=double(finite(point[i]))-double(c[2][i]);length2+=r[i]*r[i];}
    if(!(length2>0))throw std::runtime_error("native haze point at the camera");
    const double length=std::sqrt(length2);
    double dot=0;for(unsigned i=0;i<3;++i)dot+=r[i]*double(c[1][i]);
    // cos to the light direction (qword 1), clamped below by its w; sine of the elevation.
    const double cosine=std::max(dot/length,double(c[1][3])),sine=std::abs(r[1])/length;
    // Rayleigh phase (7.x (1 + cos^2)) and the Mie term 7.y / (7.z + 7.w cos)^1.5.
    const double rayleigh=double(c[7][0])*(1+cosine*cosine),base=double(c[7][2])+double(c[7][3])*cosine;
    if(!(base>0))throw std::runtime_error("native haze Mie phase outside its domain");
    const double mie=double(c[7][1])/std::exp(1.5*std::log(base));
    uint32_t rgba=0x80u<<24;
    for(unsigned i=0;i<3;++i){
        const double scatter=(double(c[4][i])*rayleigh+double(c[5][i])*mie)*double(c[3][i]);
        // Elevation mask: max(1 + 9 * exp(10 / (10 + sine)), 0) per channel.
        const double mask=std::max(1+double(c[9][i])*std::exp(double(c[10][i])/(double(c[10][i])+sine)),0.0);
        const double v=std::min(128*scatter*mask,255.0);
        if(!std::isfinite(v) || v<0)throw std::runtime_error("native haze scattering colour outside 0..255");
        rgba|=uint32_t(int(float(v)))<<(8*i);
    }
    return rgba;
}
NativeHaze nativeHaze(Runtime *rt,const NativeScene &scene,uint32_t object){
    if(object<0x100000 || value<uint32_t>(rt,object)!=kClass || value<uint32_t>(rt,object+0x90)!=kNode)
        throw std::runtime_error("native haze escaped its class");
    const auto current=value<uint32_t>(rt,0x35ddf8);const unsigned layer=value<uint8_t>(rt,object+0x65);
    if(current<0x100000 || layer>1 || value<uint32_t>(rt,current+0x40a8+4*layer)!=object || !value<uint8_t>(rt,current+0x40dc+layer))
        throw std::runtime_error("native haze outside its scene slot");
    NativeHaze haze;haze.object=object;haze.frame=scene.frameTarget();
    if(!haze.frame)throw std::runtime_error("native haze has no frame");
    const auto width=value<int32_t>(rt,object+0xa8),height=value<int32_t>(rt,object+0xac);
    if(width<=0 || width%64 || width>2048 || height<=0 || height>2048 || float(width)!=haze.frame->width || !(std::abs(float(height)-haze.frame->height)<1.f))   // the frame height is scaled (512.96)
        throw std::runtime_error(std::format("native haze size {}x{} differs from the frame {}x{}",width,height,haze.frame->width,haze.frame->height));
    // The packet's depth index assumes a 24-bit Z (0x31) read at shift 5.
    const int depthFormat=value<int16_t>(rt,value<uint32_t>(rt,object+0x94)+0x3c74),shift=value<uint8_t>(rt,object+0x18951);
    if(depthFormat!=0x31 || (shift!=5 && shift!=8))throw std::runtime_error(std::format("unported native haze depth variant (format {:x}, shift {})",depthFormat,shift));
    // The ramp (2df060's table), RGBA bytes; alpha above 0x80 has no native form.
    std::array<uint32_t,256> ramp;for(unsigned i=0;i<256;++i){ramp[i]=value<uint32_t>(rt,object+0x134c4+4*i);
        if((ramp[i]>>24)>0x80)throw std::runtime_error("native haze ramp alpha above 0x80");}
    for(unsigned k=0;k<64;++k){const auto entry=ramp[hazeRampIndex(k<<14,shift)];
        for(unsigned i=0;i<3;++i)haze.colours[4*k+i]=float((entry>>(8*i))&0xff)/255.f;haze.colours[4*k+3]=float(entry>>24)/128.f;}
    // The VU1 constants (25 qwords) and its branch flags.
    std::array<std::array<float,4>,25> constants;for(unsigned i=0;i<25;++i)constants[i]=quad(rt,object+0x18b30+16*i);
    const auto flag=[&](unsigned q,unsigned lane){return value<uint32_t>(rt,object+0x18b30+16*q+4*lane);};
    if(flag(8,0) || flag(11,1) || flag(11,2)!=1)
        throw std::runtime_error(std::format("unported native haze scattering variant ({:x} {:x} {:x})",flag(8,0),flag(11,1),flag(11,2)));
    for(unsigned q:{1u,2u,3u,4u,5u,7u,9u,10u})for(float v:constants[q])finite(v);
    // Tiles: rows from +18940 (height +18930, offset +18938), width / 64 columns,
    // records at +13940 + row * 0x1400 + column * 0x200.
    const unsigned rows=value<uint8_t>(rt,object+0x18940),columns=unsigned(width)/64;
    if(!rows || rows>8)throw std::runtime_error("unqualified native haze tile rows");
    const float originX=2048.f-haze.frame->width/2,originY=2048.f-haze.frame->height/2;
    float x0=0,y0=0;int32_t covered=0;
    for(unsigned row=0;row<rows;++row){
        const auto tileHeight=value<int16_t>(rt,object+0x18930+2*row),offset=value<int16_t>(rt,object+0x18938+2*row);
        if(tileHeight!=128 || offset!=covered)throw std::runtime_error("unported native haze tile layout");
        covered+=tileHeight;
        for(unsigned column=0;column<columns;++column){
            const uint32_t record=object+0x13940+row*0x1400+column*0x200;
            // The cell grid: x cells across and (y - 1) cells down, each step px (2x4 at 32 at the
            // entry, 4x8 at 16 deeper in); the VU1 row past the 128 rows is cut by the tile scissor.
            const auto cellsX=value<uint32_t>(rt,record),cellsY=value<uint32_t>(rt,record+4)-1;
            if(cellsX<1 || cellsX>16 || cellsY<1 || cellsY>16)throw std::runtime_error("unported native haze tile grid");
            const auto screen=quad(rt,record+16),step=quad(rt,record+32);
            if(!row && !column){x0=screen[0];y0=screen[1];}
            if(screen[0]!=x0+64.f*float(column) || screen[1]!=y0+float(offset) || step[0]*float(cellsX)!=64.f || step[1]*float(cellsY)!=128.f)
                throw std::runtime_error("native haze tile outside its grid");
            const auto base=quad(rt,record+80),across=quad(rt,record+96),down=quad(rt,record+112);
            uint32_t colour[17][17];
            for(unsigned i=0;i<=cellsX;++i)for(unsigned j=0;j<=cellsY;++j){std::array<float,4> point;
                for(unsigned l=0;l<3;++l)point[l]=finite(base[l])+float(i)*finite(across[l])+float(j)*finite(down[l]);point[3]=1;
                colour[i][j]=hazeVertexColour(constants,point);}
            const auto vertex=[&](unsigned i,unsigned j){bozo::render::Vertex v;v.x=originX+float(64*column)+step[0]*float(i);
                v.y=originY+float(offset)+step[1]*float(j);v.rgba=colour[i][j];return v;};
            for(unsigned i=0;i<cellsX;++i)for(unsigned j=0;j<cellsY;++j)
                for(const auto &[a,b]:{std::pair{i,j},std::pair{i+1,j},std::pair{i,j+1},std::pair{i+1,j},std::pair{i+1,j+1},std::pair{i,j+1}})
                    haze.vertices.push_back(vertex(a,b));
        }
    }
    if(covered!=height)throw std::runtime_error("native haze tiles do not cover the frame");
    return haze;
}
void drawHaze(const NativeHaze &haze){
    using Material=bozo::render::Material;
    Material m;m.textured=false;m.depthTest=false;m.depthWrite=false;m.blend=Material::Blend::DestinationBySource;m.colourMask=7;
    auto lookup=std::make_shared<Material::ImageProcessing>();lookup->depthLookup=true;lookup->depthColours=true;
    lookup->depthScale=16777215.f/16384.f;std::copy(haze.colours.begin(),haze.colours.end(),lookup->depthWeights.begin());
    m.imageProcessing=lookup;const float id[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    bozo::render::Renderer::instance().draw(m,id,haze.vertices.data(),uint32_t(haze.vertices.size()));
}
std::optional<bool> nativeHazeInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x2e0b90)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=d?dynamic_cast<HazeOwner *>(&d->host()):nullptr;auto *scene=d?dynamic_cast<SceneOwner *>(&d->host()):nullptr;
    if(!c || !owner || !scene)throw std::runtime_error("native haze requires its scene/renderer owner");
    // The enqueue 2e0b70: a1 = a0 + 0x90 (the queue node); its call becomes the
    // class completion 2e0b50(object), so no transfer is ever queued.
    if(c->pc!=0x2e0b70 || c->r[5].uw[0]<0x100090 || c->r[4].uw[0]!=c->r[5].uw[0]-0x90)
        throw std::runtime_error("native haze requires its original pre-enqueue producer");
    const auto object=c->r[4].uw[0];owner->drawNativeHaze(nativeHaze(rt,scene->nativeScene(),object));
    c->r[4].sd[0]=int32_t(object);return true;
}
}

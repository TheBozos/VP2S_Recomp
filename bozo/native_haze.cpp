// The Lost Forest haze as one native depth-colour pass (see native_haze.h).
#include "native_cast.h"
#include "native_haze.h"
#include "bozo/ee/dispatcher.h"
#include <bit>
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
float hazeSeriesLn(const std::array<std::array<float,4>,25> &c,float x){
    // VU1 19b8 per lane: t = (x - 1) / (x + 1); 2t + sum of k * t^(2i+1) over the coefficient qwords 13.. (12.x of them
    // in the loop at 1958, then one more at 1998), the powers t^3, t^5, t^7, t^9 stepping by t^8.
    const unsigned batches=std::bit_cast<uint32_t>(c[12][0])&0xffff;if(batches>7)throw std::runtime_error("native haze ln series past its constants");
    const float t=(x-1.f)*(1.f/(x+1.f));float acc=t+t;
    const float t2=t*t,t4=t2*t2,t8=t4*t4;float p[4]={t*t2,t4*t,0,0};p[2]=p[0]*t4;p[3]=t4*p[1];
    for(unsigned q=0;;++q){const auto &k=c[13+q];
        if(q==batches){acc=acc+p[0]*k[0];acc=acc+p[1]*k[1];acc=acc+p[2]*k[2];return acc+p[3]*k[3];}
        for(unsigned i=0;i<4;++i)acc=acc+p[i]*k[i];for(float &v:p)v=v*t8;}
}
float hazeSeriesExp(const std::array<std::array<float,4>,25> &c,float a){
    // VU1 1c60 per lane: s = |a|; 1 + s + q21 . (s^2, s^3, s^4, s^5), then 12.y more qwords from q22 with those powers
    // multiplied by a itself (vf24 = the argument, as the program does: not s^4); the reciprocal when a is negative
    // (its MAC sign bit, vi04 = MAC & 0xe0).
    const unsigned batches=std::bit_cast<uint32_t>(c[12][1])&0xffff;if(batches>3)throw std::runtime_error("native haze exp series past its constants");
    const float s=std::abs(a);float acc=s+1.f;const float s2=s*s;float p[4]={s2,s*s2,s2*s2,0};p[3]=s2*p[1];
    for(unsigned q=0;;++q){const auto &k=c[21+q];
        if(q==batches){acc=acc+p[0]*k[0];acc=acc+p[1]*k[1];acc=acc+p[2]*k[2];acc=acc+p[3]*k[3];break;}
        for(unsigned i=0;i<4;++i)acc=acc+p[i]*k[i];for(float &v:p)v=v*a;}
    return std::signbit(a)?1.f/acc:acc;
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
    // 17f8: 11.z selects the elevation mask (1808..1860); 1870: 11.y selects the gamma (1878..18c0: 256 *
    // exp(11.x * ln(scatter / 2)) by the program's series), else 128 * scatter (18c8).
    const bool elevation=std::bit_cast<uint32_t>(c[11][2])!=0,gamma=std::bit_cast<uint32_t>(c[11][1])!=0;
    uint32_t rgba=0x80u<<24;
    for(unsigned i=0;i<3;++i){
        double scatter=(double(c[4][i])*rayleigh+double(c[5][i])*mie)*double(c[3][i]);
        // Elevation mask: max(1 + 9 * exp(10 / (10 + sine)), 0) per channel.
        if(elevation)scatter*=std::max(1+double(c[9][i])*std::exp(double(c[10][i])/(double(c[10][i])+sine)),0.0);
        double v;
        if(gamma){const float x=float(scatter)*0.5f;if(!std::isfinite(x) || x<0)throw std::runtime_error("native haze gamma input outside its domain");
            v=std::min(256.0*double(hazeSeriesExp(c,hazeSeriesLn(c,x)*c[11][0])),255.0);}
        else v=std::min(128*scatter,255.0);
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
    if(flag(8,0) || flag(11,1)>1 || flag(11,2)>1 || flag(11,1)==flag(11,2))
        throw std::runtime_error(std::format("unported native haze scattering variant ({:x} {:x} {:x})",flag(8,0),flag(11,1),flag(11,2)));
    for(unsigned q:{1u,2u,3u,4u,5u,7u,9u,10u})for(float v:constants[q])finite(v);
    if(flag(11,1)){finite(constants[11][0]);for(unsigned q=13;q<25;++q)for(unsigned l=0;l<4;++l)if(!std::isfinite(constants[q][l]))throw std::runtime_error("unqualified native haze series constant");}
    // Tiles: rows from +18940 (height +18930, offset +18938), width / 64 columns,
    // records at +13940 + row * 0x1400 + column * 0x200.
    const unsigned rows=value<uint8_t>(rt,object+0x18940),columns=unsigned(width)/64;
    if(!rows || rows>8)throw std::runtime_error("unqualified native haze tile rows");
    const float originX=2048.f-haze.frame->width/2,originY=2048.f-haze.frame->height/2;
    float x0=0,y0=0;int32_t covered=0;
    for(unsigned row=0;row<rows;++row){
        const auto tileHeight=value<int16_t>(rt,object+0x18930+2*row),offset=value<int16_t>(rt,object+0x18938+2*row);
        // 128-row tiles, or 96-row ones in a 416-high frame (128, 96, 96, 96: the
        // battle-entry frame), each with its own grid of that height.
        if(tileHeight<=0 || tileHeight>128 || offset!=covered)throw std::runtime_error("unported native haze tile layout");
        covered+=tileHeight;
        for(unsigned column=0;column<columns;++column){
            const uint32_t record=object+0x13940+row*0x1400+column*0x200;
            // The cell grid: x cells across and (y - 1) cells down, each step px (2x4 at 32 at the
            // entry, 4x8 at 16 deeper in); the VU1 row past the 128 rows is cut by the tile scissor.
            const auto cellsX=value<uint32_t>(rt,record),cellsY=value<uint32_t>(rt,record+4)-1;
            if(cellsX<1 || cellsX>16 || cellsY<1 || cellsY>16)throw std::runtime_error("unported native haze tile grid");
            const auto screen=quad(rt,record+16),step=quad(rt,record+32);
            if(!row && !column){x0=screen[0];y0=screen[1];}
            if(screen[0]!=x0+64.f*float(column) || screen[1]!=y0+float(offset) || step[0]*float(cellsX)!=64.f || step[1]*float(cellsY)!=float(tileHeight))
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
    auto *d=rt?vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services):nullptr;
    auto *owner=d?vp2s::nativeCast<HazeOwner>(&d->host()):nullptr;auto *scene=d?vp2s::nativeCast<SceneOwner>(&d->host()):nullptr;
    if(!c || !owner || !scene)throw std::runtime_error("native haze requires its scene/renderer owner");
    // The enqueue 2e0b70: a1 = a0 + 0x90 (the queue node); its call becomes the
    // class completion 2e0b50(object), so no transfer is ever queued.
    if(c->pc!=0x2e0b70 || c->r[5].uw[0]<0x100090 || c->r[4].uw[0]!=c->r[5].uw[0]-0x90)
        throw std::runtime_error("native haze requires its original pre-enqueue producer");
    const auto object=c->r[4].uw[0];owner->drawNativeHaze(nativeHaze(rt,scene->nativeScene(),object));
    c->r[4].sd[0]=int32_t(object);return true;
}
}

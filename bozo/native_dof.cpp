#include "native_dof.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/render/renderer.h"
#include <cstring>
#include <cmath>
#include <immintrin.h>
namespace vp2s {namespace {
struct PreserveFp{unsigned state=_mm_getcsr();~PreserveFp(){_mm_setcsr(state);}};
template<class T>T value(Runtime *rt,uint32_t at){T v;std::memcpy(&v,rt->memory.hostPointer(at,sizeof(T)),sizeof(T));return v;}
uint32_t owner(Runtime *rt,uint32_t object){
    const auto scene=value<uint32_t>(rt,0x35ddf8);
    if(object<0x100000 || value<uint32_t>(rt,object)!=0x159db0 || value<uint32_t>(rt,object+0x90)!=0x159e2c ||
       scene<0x100000 || value<uint32_t>(rt,scene+0x4448)!=object)throw std::runtime_error("DOF escaped its scene owner");
    return scene;
}
float finite(Runtime *rt,uint32_t at,float low,float high){const auto f=value<float>(rt,at);
    if(!std::isfinite(f) || f<low || f>high)throw std::runtime_error("unqualified native DOF parameter");return f;}
}
// The game's Gaussian lerp-tap builder (mod001 2e6070), single precision as on
// the EE: g[i] = exp(-1.625 i^2 / r^2) until g < 1.96e-4 (at most 255 taps),
// normalised by 1 + 2 sum g; taps >= 1/255 kept; neighbours paired into one
// bilinear fetch at 1 + 2j + g2/(g1+g2) (1/16 px) with integer 128ths, and the
// pair alphas corrected for the lerps that follow. The draw order is far pair
// first, negative offset first; each lerp is C += (S(offset) - C) * a/128 over
// a base copy. Returns the effective symmetric integer-tap kernel.
std::array<float,17> glowKernel(float r,uint8_t &radius){
    std::array<float,256> g{};g[0]=1;volatile float sum=1;const volatile float k=-1.625f/(r*r);unsigned n=1;
    for(;n<255;++n){volatile float arg=k*float(n*n);const float v=std::exp(float(arg));g[n]=v;sum=sum+2*v;if(v<1.96e-4f)break;}
    unsigned kept=0;for(unsigned i=0;i<=n;++i){volatile float q=g[i]/sum;g[i]=q;if(q<1.f/255)break;++kept;}
    if(kept==n+1)--kept;unsigned pairs=std::min<unsigned>(2*(kept/2),32);
    struct Tap{int sixteenths;float alpha;};std::vector<std::pair<Tap,Tap>> made;float f=1;int previous=0;
    for(unsigned t=0;t<pairs;t+=2){const int far=int(128*g[t+2]+0.5f),near=int(128*g[t+1]+0.5f),both=far+near;
        int a0;if(t==0)a0=both;else{f*=128.f/float(128-previous);a0=int(f*float(both)+0.5f);}
        f*=128.f/float(128-a0);const int a2=int(f*float(both)+0.5f);
        const int offset=both?16+32*int(t/2)+(far*16)/both:255;made.push_back({{offset,a0/128.f},{-offset,a2/128.f}});previous=a2;}
    std::array<double,34> w{};w[16]=1;   // index 16 = centre, -16..+17
    const auto lerp=[&](int sixteenths,float alpha){for(auto &x:w)x*=1-alpha;
        const int whole=sixteenths>=0?sixteenths/16:-((-sixteenths)/16);const float frac=float(std::abs(sixteenths)%16)/16;
        const int next=sixteenths>=0?whole+1:whole-1;
        if(std::abs(whole)>16 || std::abs(next)>16)throw std::runtime_error("native glow tap outside the kernel");
        w[16+whole]+=alpha*(1-frac);w[16+next]+=alpha*frac;};
    for(auto it=made.rbegin();it!=made.rend();++it){lerp(it->second.sixteenths,it->second.alpha);lerp(it->first.sixteenths,it->first.alpha);}
    std::array<float,17> kernel{};radius=0;
    for(unsigned i=0;i<=16;++i){kernel[i]=float(i?(w[16+i]+w[16-i])/2:w[16]);if(kernel[i]>1e-6f)radius=uint8_t(i);}
    return kernel;
}
void NativeDofs::begin(Runtime *rt,const NativeScene &scene,uint32_t stack,uint32_t object){
    owner(rt,object);if(builds.contains(stack))throw std::runtime_error("DOF builder stack reused");
    for(const auto &[at,b]:builds)if(b.effect.object==object)throw std::runtime_error("DOF builder object reused");
    const auto frame=scene.frameTarget();if(!frame)throw std::runtime_error("DOF has no native frame");
    NativeDof effect;effect.object=object;effect.frame=frame;ready.erase(object);builds.emplace(stack,Build{effect});
}
void NativeDofs::block(Runtime *rt,uint32_t object,uint32_t address){
    PreserveFp arithmetic;
    auto it=std::find_if(builds.begin(),builds.end(),[&](const auto &entry){return entry.second.effect.object==object;});
    if(it==builds.end())throw std::runtime_error("DOF block escaped original builder");
    owner(rt,object);
    if(address!=object+0xf45c && (address<object+0xf330 || address>=object+0xf45c || (address-object-0xf330)%0x3c))
        throw std::runtime_error("unqualified DOF parameter owner");
    if(!value<uint32_t>(rt,address))return;
    // +1e is the composite sprite's vertex alpha (RGBAQ A): a constant opacity
    // applied after the clamped weight map. Opacities above 1 are unqualified.
    if(value<uint8_t>(rt,address+0x1f) || !value<uint8_t>(rt,address+0x1e) || value<uint8_t>(rt,address+0x1e)>128)
        throw std::runtime_error("unqualified native DOF block mode: block="+std::to_string(address-object)+" mode1f="+
            std::to_string(value<uint8_t>(rt,address+0x1f))+" alpha1e="+std::to_string(value<uint8_t>(rt,address+0x1e)));
    auto &effect=it->second.effect;
    if(effect.blocks.size()>=6 || std::any_of(effect.blocks.begin(),effect.blocks.end(),[&](const auto &b){return b.address==address;}))
        throw std::runtime_error("DOF block published twice");
    const auto workWidth=value<uint32_t>(rt,object+0x131ac),workHeight=value<uint32_t>(rt,object+0x131b0);
    const auto width=value<int16_t>(rt,address+0x18),weightWidth=value<int16_t>(rt,address+0x1a);
    if(!workWidth || workWidth>512 || !workHeight || workHeight>512 || width<16 || width>256 || weightWidth<16 || weightWidth>256)
        throw std::runtime_error("unqualified native DOF work dimensions");
    NativeDofBlock block;block.address=address;block.width=width;block.weightWidth=weightWidth;
    block.height=std::max(1u,uint32_t(width)*workHeight/workWidth);
    block.weightHeight=std::max(1u,uint32_t(weightWidth)*workHeight/workWidth);
    block.sigma=0.555f*finite(rt,address+0x24,0,24);block.weightSigma=0.555f*finite(rt,address+0x28,0,24);
    block.gain=finite(rt,address+0x2c,0,8);block.weightGain=finite(rt,address+0x30,0,8);
    const auto strength=finite(rt,address+0x34,0,1);const float c=value<uint8_t>(rt,address+0x1c);
    block.weightScale=2*(c+(128-c)*strength)/128;
    block.alpha=value<uint8_t>(rt,address+0x1e)/128.f;
    block.depthLimit=(value<uint8_t>(rt,address+0x20)&3)==1;block.z=value<uint32_t>(rt,address+0x38);
    for(unsigned i=0;i<256;++i){const auto weight=value<uint8_t>(rt,object+0xef11+4*i);
        if(weight>128)throw std::runtime_error("unqualified DOF table weight");block.weights[i]=weight/128.f;}
    effect.blocks.push_back(block);
}
void NativeDofs::beginBlock(uint32_t stack,uint32_t object,uint32_t address){
    if(blockCalls.contains(stack) || std::none_of(builds.begin(),builds.end(),[&](const auto &b){return b.second.effect.object==object;}))
        throw std::runtime_error("DOF block invocation escaped outer builder");
    blockCalls.emplace(stack,BlockCall{object,address});
}
void NativeDofs::finishBlock(Runtime *rt,uint32_t stack,uint32_t object,uint32_t address){
    const auto it=blockCalls.find(stack);
    if(it==blockCalls.end() || it->second.object!=object || it->second.address!=address)
        throw std::runtime_error("DOF block completion escaped invocation");
    block(rt,object,address);blockCalls.erase(it);
}
void NativeDofs::finish(Runtime *rt,const NativeScene &scene,uint32_t stack,uint32_t object){
    PreserveFp arithmetic;const auto it=builds.find(stack);owner(rt,object);
    if(it==builds.end() || it->second.effect.object!=object || it->second.effect.frame!=scene.frameTarget())
        throw std::runtime_error("DOF finished outside original frame/builder");
    if(std::any_of(blockCalls.begin(),blockCalls.end(),[&](const auto &call){return call.second.object==object;}))
        throw std::runtime_error("DOF finished with incomplete block writer");
    if(value<uint8_t>(rt,object+0x63)){
        auto effect=it->second.effect;effect.source=value<uint32_t>(rt,object+0x94);
        effect.width=value<uint32_t>(rt,object+0x131ac);effect.height=value<uint32_t>(rt,object+0x131b0);
        if(effect.source!=effect.frame->object || !effect.width || effect.width>512 || !effect.height || effect.height>512)
            throw std::runtime_error("DOF source dimensions escaped native frame");
        effect.gain=finite(rt,object+0x131e0,0,8);
        // The game leaves depth fields uninitialized when only glow runs.
        // They are inputs only when its block writer actually executed.
        if(!effect.blocks.empty()){
            const auto shift=value<int32_t>(rt,object+0x131d0);
            if(shift<0 || shift>7)throw std::runtime_error("unqualified DOF depth shift "+std::to_string(shift));
            effect.depthScale=256.f*float(1u<<shift);
        }
        // Effect B, called by 2d9360 when +130e0 is set (2da190/2da1e8).
        effect.glareB=value<uint8_t>(rt,object+0x130e0)!=0;
        if(effect.glareB){
            // 2da124: thresholds subtract in increasing order, so each effect
            // sees F less its own count. With B first, A's first level write
            // (ALPHA 0x88) would replace B in 428: not observed, not ported.
            if(value<uint8_t>(rt,object+0xdfbc) && value<uint8_t>(rt,object+0x130e1)<value<uint8_t>(rt,object+0xdfc9))
                throw std::runtime_error("unported native glare-before-glow order");
            effect.glareThreshold=value<uint8_t>(rt,object+0x130e1)/255.f;
            // 2d71e0: work size s4 = table 0x314720[+130e4]; rows s3 =
            // clamp(int(6 * +ac / +130f8), 16, s4); angles by mode +130e8.
            const auto index=value<uint32_t>(rt,object+0x130e4);
            if(index>3)throw std::runtime_error("unqualified native glare work index");
            const auto work=value<int32_t>(rt,0x314720+4*index);
            volatile float rowsF=float(6*value<int32_t>(rt,object+0xac))/finite(rt,object+0x130f8,1e-3f,1e6f);
            int32_t rows=std::max(16,int32_t(float(rowsF)));rows=std::min(rows,work);
            if(work<16 || work>512)throw std::runtime_error("unqualified native glare work size");
            effect.glareWork=uint32_t(work);effect.glareRows=uint32_t(rows);
            const float base=finite(rt,object+0x130f0,-100,100),offset=finite(rt,object+0x130ec,-100,100);
            const float pi=3.14159274f;std::vector<float> angles;
            switch(value<uint32_t>(rt,object+0x130e8)){
            case 0:for(int k=0;k<4;++k){volatile float a=pi*(0.25f+float(k)/2.f);a=a+offset;angles.push_back(base+a);}break;
            case 1:for(int k=0;k<6;++k){volatile float a=pi*(0.25f+float(k)/3.f);a=a+offset;angles.push_back(base+a);}break;
            case 2:for(int k=0;k<2;++k){volatile float a=pi*(0.5f+float(k));angles.push_back(a+offset);}break;
            default:throw std::runtime_error("unqualified native glare mode");
            }
            // 2d7a30: the 445 rectangle scaled to fit its half-diagonal in the
            // work square, rotated about its centre, rows scaled by s3/s4.
            const int32_t w=value<int32_t>(rt,object+0x131ac),h=value<int32_t>(rt,object+0x131b0);
            const int32_t hw=w/2,hh=h/2;   // sra with round toward zero
            const volatile float diag=std::sqrt(float(hw*hw+hh*hh));
            const volatile float scale=float(work)/diag,fw=float(hw)*scale,fh=float(hh)*scale,full=scale*diag;
            const volatile float left=0.5f*(full-fw),top=(full-fh)/2.f;
            const float right=fw+left,bottom=fh+top,cx=0.5f*fw+left,cy=0.5f*fh+top;
            const volatile float rowScale=float(rows)/float(work);
            for(const float angle:angles){
                const float c=std::cos(angle),sn=std::sin(angle);std::array<std::array<int32_t,2>,4> corners{};
                const float xs[4]={left,right,left,right},ys[4]={top,top,bottom,bottom};
                for(unsigned i=0;i<4;++i){
                    volatile float x=cx+(xs[i]-cx)*c-(ys[i]-cy)*sn,y=(cy+(xs[i]-cx)*sn+(ys[i]-cy)*c)*rowScale;
                    if(!(x>-1 && y>-1 && x<float(work+1) && y<float(rows+1)))throw std::runtime_error("native glare corner outside its work image");
                    corners[i]={int32_t(float(x)),int32_t(float(y))};}
                effect.glareCorners.push_back(corners);}
            // Rotate-in: vertex colour min(255, int(64 * +130f4)) with ALPHA
            // 0x08 (Cs*As + Cs, As 0x80): twice that over 128. Above 4 the game
            // adds further passes, not ported.
            const float strength=finite(rt,object+0x130f4,0,4);
            effect.glareIn=2.f*float(std::min(255,int(64.f*strength)))/128.f;
            // Rotate-back alpha +130e2, times the DOF gain when +dfb4 (2d8a68).
            unsigned out=value<uint8_t>(rt,object+0x130e2);
            if(value<uint8_t>(rt,object+0xdfb4)){volatile float scaled=float(out)*effect.gain;out=std::min(255u,unsigned(int(scaled)));}
            effect.glareOut=out/128.f;
        }
        // Effect A, called by 2d9360 when +dfbc is set (2da148/2da238).
        effect.glowA=value<uint8_t>(rt,object+0xdfbc)!=0;
        if(effect.glowA){
            effect.glowThreshold=value<uint8_t>(rt,object+0xdfc9)/255.f;
            // 2d58c8: weight = +dfc8, scaled by the DOF gain / 3 when +dfb4.
            unsigned weight=value<uint8_t>(rt,object+0xdfc8);
            if(value<uint8_t>(rt,object+0xdfb4)){volatile float scaled=float(weight)*(effect.gain/3.f);weight=std::min(255u,unsigned(int(scaled)));}
            effect.glowWeight=weight/128.f;
            // 2d6ea0: radius = +dfc0 * work width / +131ac, at most 50; above 3
            // it is scaled by 0.4 and blurred by 2d4ca0 (else 2d5e80).
            const auto workWidth=value<int32_t>(rt,object+0x13204),baseWidth=value<int32_t>(rt,object+0x131ac);
            if(workWidth<=0 || baseWidth<=0)throw std::runtime_error("unqualified native glow work width");
            volatile float radius=finite(rt,object+0xdfc0,0,1000)*float(workWidth)/float(baseWidth);radius=std::min(float(radius),50.f);
            if(!(radius>3))throw std::runtime_error("unported native glow small-radius path (2d5e80)");
            radius=radius*0.4f;
            effect.glowKernel=glowKernel(radius,effect.glowRadius);
        }
        ready[object]=std::move(effect);
    }
    builds.erase(it);
}
NativeDof NativeDofs::submit(Runtime *rt,const NativeScene &scene,const NativeFrameCopies &copies,uint32_t object){
    const auto current=owner(rt,object);const auto it=ready.find(object);const auto layer=value<uint8_t>(rt,object+0x65);
    const auto colour=copies.image(scene,value<uint32_t>(rt,current+0x444c));
    if(it==ready.end() || it->second.frame!=scene.frameTarget() || !colour || colour->frame!=it->second.frame || layer>1 ||
       value<uint32_t>(rt,current+0x40a8+4*layer)!=object || !value<uint8_t>(rt,current+0x40dc+layer))
        throw std::runtime_error("DOF submitted outside completed builder/colour image");
    auto effect=it->second;ready.erase(it);return effect;
}
void NativeDofs::release(uint32_t object){
    if(std::any_of(builds.begin(),builds.end(),[&](const auto &build){return build.second.effect.object==object;}) ||
       std::any_of(blockCalls.begin(),blockCalls.end(),[&](const auto &call){return call.second.object==object;}))
        throw std::runtime_error("DOF destroyed during builder");
    ready.erase(object);
}
void drawDof(const NativeDof &effect){
    PreserveFp arithmetic;
    if(!effect.frame)throw std::runtime_error("DOF draw has no frame");
    auto &renderer=bozo::render::Renderer::instance();using Material=bozo::render::Material;
    const float id[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    const auto quad=[&](Material material,uint32_t rgba=0x80808080){
        bozo::render::Vertex vertices[6];const float xy[6][2]={{0,0},{1,0},{0,1},{1,0},{1,1},{0,1}};
        for(unsigned i=0;i<6;++i){auto &v=vertices[i];v.x=2048-effect.frame->width/2+xy[i][0]*effect.frame->width;
            v.y=2048-effect.frame->height/2+xy[i][1]*effect.frame->height;v.u=xy[i][0];v.v=xy[i][1];v.rgba=rgba;}
        renderer.draw(material,id,vertices,6);
    };
    const auto image=[&](uint8_t source,uint8_t target,uint32_t width,uint32_t height){Material m;m.textured=false;m.depthTest=false;m.depthWrite=false;
        m.blend=Material::Blend::Opaque;m.fromFrame=source?1:0;m.source=source;m.target=target;m.targetWidth=width;m.targetHeight=height;m.clampU=m.clampV=true;return m;};
    const auto blur=[&](uint8_t source,uint8_t temporary,uint8_t target,uint32_t width,uint32_t height,float sigma,float gain){
        if(sigma==0){auto m=image(source,target,width,height);m.levels.enabled=true;std::fill_n(m.levels.gain,3,gain);quad(m);return;}
        auto processing=std::make_shared<Material::ImageProcessing>();processing->radius=std::min(16,int(std::ceil(3*sigma)));
        double sum=0;for(unsigned i=0;i<=processing->radius;++i){const double w=std::exp(-double(i*i)/(2*double(sigma)*sigma));processing->weights[i]=float(w);sum+=w*(i?2:1);}
        for(unsigned i=0;i<=processing->radius;++i)processing->weights[i]/=float(sum);
        processing->step[0]=1.f/width;auto horizontal=image(source,temporary,width,height);horizontal.imageProcessing=processing;quad(horizontal);
        processing=std::make_shared<Material::ImageProcessing>(*processing);processing->step={0,1.f/height};
        auto vertical=image(temporary,target,width,height);vertical.imageProcessing=processing;vertical.levels.enabled=true;
        std::fill_n(vertical.levels.gain,3,gain);quad(vertical);
    };
    for(const auto &block:effect.blocks){
        auto weights=image(0,1,effect.width,effect.height);auto table=std::make_shared<Material::ImageProcessing>();table->depthLookup=true;
        table->depthScale=effect.depthScale;table->depthWeights=block.weights;weights.imageProcessing=table;quad(weights);
        quad(image(1,2,block.weightWidth,block.weightHeight));
        blur(2,1,2,block.weightWidth,block.weightHeight,block.weightSigma,block.weightGain);
        auto colour=image(7,3,block.width,block.height);colour.levels.enabled=true;std::fill_n(colour.levels.gain,3,effect.gain);quad(colour);
        blur(3,4,3,block.width,block.height,block.sigma,block.gain);
        auto composite=image(3,0,0,0);composite.blend=Material::Blend::Alpha;composite.colourMask=7;
        auto alpha=std::make_shared<Material::ImageProcessing>();alpha->alphaSource=2;alpha->alphaScale=block.weightScale;
        composite.imageProcessing=alpha;
        // The block's constant opacity (+1e, vertex alpha) scales the output
        // alpha after the clamped map: the shader clamps vertex alpha first.
        const uint32_t rgba=0x00808080u|(uint32_t(std::lround(block.alpha*128))<<24);
        if(block.depthLimit){composite.depthTest=true; // Screen depth is larger-nearer; sprite at block Z.
            bozo::render::Vertex vertices[6];const float xy[6][2]={{0,0},{1,0},{0,1},{1,0},{1,1},{0,1}};
            for(unsigned i=0;i<6;++i){auto &v=vertices[i];v.x=2048-effect.frame->width/2+xy[i][0]*effect.frame->width;
                v.y=2048-effect.frame->height/2+xy[i][1]*effect.frame->height;v.z=float(block.z);v.u=xy[i][0];v.v=xy[i][1];v.rgba=rgba;}
            renderer.draw(composite,id,vertices,6);
        }else quad(composite,rgba);
    }
    if(effect.glowA){
        // Effect A as the GS dump shows it: the frame copy (image 7, before the
        // 2x brightness pass) less the threshold, as three levels (full, half,
        // quarter), each blurred by the game's tap kernel and added at weight/128.
        const uint32_t width=effect.width,height=effect.height;
        auto threshold=image(7,5,width,height);threshold.levels.enabled=true;std::fill_n(threshold.levels.low,3,effect.glowThreshold);quad(threshold);
        quad(image(5,2,width/2,height/2));quad(image(2,1,width/4,height/4));
        const uint8_t levels[3]={5,2,1};
        for(unsigned level=0;level<3;++level){const uint32_t w=width>>level,h=height>>level;
            auto horizontal=std::make_shared<Material::ImageProcessing>();horizontal->radius=effect.glowRadius;horizontal->weights=effect.glowKernel;horizontal->step={1.f/w,0};
            auto pass=image(levels[level],4,w,h);pass.imageProcessing=horizontal;quad(pass);
            auto vertical=std::make_shared<Material::ImageProcessing>(*horizontal);vertical->step={0,1.f/h};
            pass=image(4,3,w,h);pass.imageProcessing=vertical;quad(pass);
            auto add=image(3,0,0,0);add.blend=Material::Blend::Additive;add.colourMask=7;add.levels.enabled=true;
            std::fill_n(add.levels.gain,3,effect.glowWeight);quad(add);}
    }
    if(effect.glareB){
        // Effect B as 2d7a30 draws it, per rotation: clear the work image, draw
        // the thresholded 445 rotated into it, smear it twice along its rows
        // (each pass reads the image before the pass), rotate it back into the
        // 428 accumulator; 428 is added to the frame at the end.
        const uint32_t width=effect.width,height=effect.height,workW=effect.glareWork+1,workH=effect.glareRows+1;
        // A draw over target pixels (x, y) with source coordinates (u, v), both
        // normalised by the target and source sizes; corners in strip order.
        const auto strip=[&](Material m,uint32_t tw,uint32_t th,const float (&x)[4][2],const float (&uv)[4][2]){
            bozo::render::Vertex v[6];const unsigned order[6]={0,1,2,1,3,2};
            for(unsigned i=0;i<6;++i){const auto c=order[i];auto &o=v[i];
                o.x=2048-effect.frame->width/2+x[c][0]/float(tw)*effect.frame->width;
                o.y=2048-effect.frame->height/2+x[c][1]/float(th)*effect.frame->height;o.u=uv[c][0];o.v=uv[c][1];o.rgba=0x80808080;}
            renderer.draw(m,id,v,6);
        };
        const auto scaled=[&](uint8_t source,uint8_t target,uint32_t w,uint32_t h,float gain){
            auto m=image(source,target,w,h);m.levels.enabled=true;std::fill_n(m.levels.gain,3,gain);return m;};
        auto threshold=image(7,5,width,height);threshold.levels.enabled=true;std::fill_n(threshold.levels.low,3,effect.glareThreshold);quad(threshold);
        quad(scaled(5,4,width,height,0));
        const float W=float(effect.glareWork),H=float(effect.glareRows);
        for(const auto &corners:effect.glareCorners){
            quad(scaled(5,1,workW,workH,0));
            float x[4][2],uv[4][2];const float unit[4][2]={{0,0},{1,0},{0,1},{1,1}};
            for(unsigned i=0;i<4;++i){x[i][0]=float(corners[i][0]);x[i][1]=float(corners[i][1]);}
            strip(scaled(5,1,workW,workH,effect.glareIn),workW,workH,x,unit);
            // 0.6^k streak: pass 0 at rows + 1.375 (0x7b), pass 1 nearest at + 3 (0x1c).
            const float row0[4][2]={{0,1.375f},{W,1.375f},{0,H+1.375f},{W,H+1.375f}},row1[4][2]={{0,3},{W,3},{0,H+3},{W,H+3}};
            const float src[4][2]={{0,0},{W/workW,0},{0,H/workH},{W/workW,H/workH}};
            auto smear=scaled(1,1,workW,workH,123.f/128);smear.blend=Material::Blend::Additive;smear.colourMask=7;strip(smear,workW,workH,row0,src);
            smear=scaled(1,1,workW,workH,28.f/128);smear.blend=Material::Blend::Additive;smear.colourMask=7;smear.nearest=true;strip(smear,workW,workH,row1,src);
            const float edge[4][2]={{0,0},{float(width),0},{0,float(height)},{float(width),float(height)}};
            for(unsigned i=0;i<4;++i){uv[i][0]=float(corners[i][0])/float(workW);uv[i][1]=float(corners[i][1])/float(workH);}
            auto back=scaled(1,4,width,height,effect.glareOut);back.blend=Material::Blend::Additive;back.colourMask=7;strip(back,width,height,edge,uv);
        }
        auto add=image(4,0,0,0);add.blend=Material::Blend::Additive;add.colourMask=7;quad(add);
    }
}
std::optional<bool> nativeDofInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x2d9360 && pc!=0x2cd020 && pc!=0x2d18a0 && pc!=0x2da980 && pc!=0x2d9350 && pc!=0x2da9b0)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *dof=d?dynamic_cast<DofOwner *>(&d->host()):nullptr;auto *scene=d?dynamic_cast<SceneOwner *>(&d->host()):nullptr;
    auto *copies=d?dynamic_cast<FrameCopyOwner *>(&d->host()):nullptr;
    if(!c || !dof || !scene || !copies)throw std::runtime_error("native DOF requires scene/image owner");
    auto &manager=dof->nativeDofs();const auto stack=c->r[29].uw[0];
    if(pc==0x2da9b0)manager.release(c->r[4].uw[0]);
    else if(pc==0x2d9360){if(c->pc!=pc)throw std::runtime_error("DOF requires complete builder");manager.begin(rt,scene->nativeScene(),stack-0xa0,c->r[4].uw[0]);}
    else if(pc==0x2cd020){if(c->pc!=pc)throw std::runtime_error("DOF requires complete block writer");manager.beginBlock(stack-0x320,c->r[4].uw[0],c->r[6].uw[0]);}
    else if(pc==0x2d18a0)manager.finishBlock(rt,stack,c->r[17].uw[0],c->r[16].uw[0]);
    else if(pc==0x2da980)manager.finish(rt,scene->nativeScene(),stack,c->r[21].uw[0]);
    else{
        if(c->pc!=0x2d9330 || c->r[5].uw[0]<0x100090 || c->r[4].uw[0]!=c->r[5].uw[0]-0x90)
            throw std::runtime_error("DOF requires original pre-enqueue producer");
        const auto object=c->r[4].uw[0];const auto effect=manager.submit(rt,scene->nativeScene(),copies->nativeFrameCopies(),object);
        dof->drawNativeDof(effect);c->r[4].sd[0]=int32_t(object);return true;
    }
    return false;
}
}

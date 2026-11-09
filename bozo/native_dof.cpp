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
        effect.glow=(value<uint8_t>(rt,object+0xdfbc) && value<uint8_t>(rt,object+0xef09)) ||
                    (value<uint8_t>(rt,object+0x130e0) && value<uint8_t>(rt,object+0xef0a));
        effect.glowAlpha=value<uint8_t>(rt,object+0xef08)/128.f;
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
    if(effect.glow && effect.glowAlpha>0){
        auto threshold=image(7,5,128,88);threshold.levels.enabled=true;
        std::fill_n(threshold.levels.gain,3,0.5f);std::fill_n(threshold.levels.low,3,25.f/255);quad(threshold);
        uint32_t width=128,height=88;uint8_t unblurred=5;
        for(unsigned level=0;level<3;++level){
            quad(image(unblurred,3,width,height));blur(3,4,3,width,height,1.5f,1);
            auto add=image(3,0,0,0);add.blend=Material::Blend::Additive;add.colourMask=7;add.levels.enabled=true;
            std::fill_n(add.levels.gain,3,0.5f*76.f/128*effect.glowAlpha);quad(add);
            width/=2;height/=2;if(level!=2){const uint8_t next=unblurred==5?2:5;quad(image(unblurred,next,width,height));unblurred=next;}
        }
        // Preserve the established native glare equation; its strength still
        // needs a bright-scene PCSX2 comparison, as documented before migration.
        threshold=image(7,5,128,88);threshold.levels.enabled=true;
        std::fill_n(threshold.levels.gain,3,0.5f);std::fill_n(threshold.levels.low,3,102.f/255);quad(threshold);
        for(float degrees:{70.f,110.f}){
            auto line=std::make_shared<Material::ImageProcessing>();line->radius=6;double sum=0;
            for(unsigned i=0;i<=6;++i){const double w=std::exp(-double(i*i)/8);line->weights[i]=float(w);sum+=w*(i?2:1);}
            for(unsigned i=0;i<=6;++i)line->weights[i]/=float(sum);
            const float angle=degrees*3.14159265f/180;line->step={std::cos(angle)/128,std::sin(angle)/88};
            auto smear=image(5,3,128,88);smear.imageProcessing=line;quad(smear);
            auto add=image(3,0,0,0);add.blend=Material::Blend::Additive;add.colourMask=7;add.levels.enabled=true;
            std::fill_n(add.levels.gain,3,1.5f*effect.glowAlpha);quad(add);
        }
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

#include "native_textures.h"
#include "native_scene.h"
#include "bozo/ee/dispatcher.h"
#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <format>
#include <stdexcept>
#include <sstream>
namespace vp2s {
namespace {
std::atomic<uint64_t> assetSerial{1};
uint8_t *record(Runtime *rt,uint32_t at,uint32_t size){
    if(!rt || at<0x100000 || at>=0x2000000 || size>0x2000000-at)
        throw std::runtime_error("native texture metadata outside EE RAM");
    return rt->memory.hostPointer(at,size);
}
template<class T>T read(Runtime *rt,uint32_t at){T value;std::memcpy(&value,record(rt,at,sizeof(T)),sizeof(T));return value;}
uint32_t rgba(uint32_t colour){
    const unsigned alpha=std::min(255u,((colour>>24)*255u+64u)/128u);
    return (colour&0xffffffu)|(alpha<<24);
}
}
std::shared_ptr<const bozo::render::Texture> NativeTextureAsset::picture(unsigned csa)const{
    if(csa>15)throw std::runtime_error("native texture palette part outside range");
    auto result=std::make_shared<bozo::render::Texture>();result->width=width;result->height=height;
    result->key=(identity<<4)|csa;result->version=identity;
    result->rgba.resize(size_t(width)*height);
    for(size_t i=0;i<result->rgba.size();++i){
        uint32_t value;
        if(format==0)std::memcpy(&value,texels.data()+i*4,4);
        else value=palette[format==0x14?csa*16+texels[i]:texels[i]];
        result->rgba[i]=rgba(value);
    }
    return result;
}
NativeTextures::NativeTextures(std::filesystem::path directory,std::filesystem::path manifest):root(std::move(directory)){
    if(manifest.empty())return; // Synthetic tests may deliberately have no source.
    std::ifstream in(manifest);std::string magic,name;in>>magic;
    if(magic!="VPNT1")throw std::runtime_error("native texture manifest missing or invalid");
    std::string line;std::getline(in,line);
    while(std::getline(in,line)){
        if(line.empty())continue;
        uint32_t tag,index;std::string extra;std::istringstream row(line);
        if(!(row>>std::hex>>tag>>index>>name) || row>>extra)
            throw std::runtime_error("truncated native texture manifest entry");
        if(name.size()!=22 || name.substr(0,2)!="c_" || name.substr(18)!=".tex" ||
           name.find_first_not_of("0123456789abcdef",2)<18 || !files.emplace(TextureKey{tag,index},name).second)
            throw std::runtime_error("invalid or duplicate native texture manifest entry");
    }
    if(!in.eof())throw std::runtime_error("truncated native texture manifest");
}
std::shared_ptr<const NativeTextureAsset> NativeTextures::load(TextureKey key){
    if(auto it=loaded.find(key);it!=loaded.end())return it->second;
    const auto found=files.find(key);
    if(found==files.end())throw std::runtime_error(std::format("native texture {:08x}:{:x} requires offline conversion/load provenance",key.tag,key.index));
    std::ifstream in(root/found->second,std::ios::binary);uint32_t header[3];
    if(!in.read(reinterpret_cast<char *>(header),sizeof(header)))throw std::runtime_error("truncated native texture header");
    if(!header[0] || !header[1] || header[0]>1024 || header[1]>1024 || (header[2]!=0 && header[2]!=0x13 && header[2]!=0x14))
        throw std::runtime_error("unsupported native texture conversion");
    auto asset=std::make_shared<NativeTextureAsset>();asset->key=key;asset->identity=assetSerial.fetch_add(1);asset->width=header[0];asset->height=header[1];asset->format=header[2];
    asset->texels.resize(size_t(header[0])*header[1]*(header[2]==0?4:1));
    if(!in.read(reinterpret_cast<char *>(asset->texels.data()),asset->texels.size()) ||
       !in.read(reinterpret_cast<char *>(asset->palette.data()),sizeof(asset->palette)) || in.peek()!=std::char_traits<char>::eof())
        throw std::runtime_error("native texture conversion size mismatch");
    if(asset->format==0x14 && std::any_of(asset->texels.begin(),asset->texels.end(),[](auto i){return i>15;}))
        throw std::runtime_error("invalid offline four-bit texture index");
    loaded.emplace(key,asset);return asset;
}
void NativeTextures::begin(uint32_t stack,TextureKey key){
    if(pending.contains(stack))throw std::runtime_error("native texture invocation stack reused while active");
    auto asset=load(key);pending.emplace(stack,Invocation{std::move(asset)});
}
void NativeTextures::publish(uint32_t stack,uint16_t slot,bool palette){
    auto it=pending.find(stack);
    if(it==pending.end() || slot>=0x8000)throw std::runtime_error("native texture publication has no active asset/slot");
    auto &job=it->second;auto &done=palette?job.palette:job.pixels;
    if(done)throw std::runtime_error("native texture part published twice");
    (palette?palettes:pixels)[slot]=job.asset;done=true;++publications;
}
void NativeTextures::finish(uint32_t stack){
    if(pending.erase(stack)!=1)throw std::runtime_error("native texture return has no invocation");
}
void NativeTextures::publishStaticPalette(uint16_t slot){
    if(slot>=0x8000)throw std::runtime_error("native static palette slot outside range");
    auto asset=load(kStaticScenePalette);palettes[slot]=std::move(asset);++publications;
}
void NativeTextures::publishImage(uint32_t object,TextureKey key){
    if(object<0x100000 || object>0x2000000-0xf0)throw std::runtime_error("native image object outside EE RAM");
    auto asset=load(key);images[object]=std::move(asset);++publications;
}
void NativeTextures::releaseImage(uint32_t object){images.erase(object);}
std::shared_ptr<const NativeTextureAsset> NativeTextures::imageAsset(uint32_t object)const{
    auto it=images.find(object);return it==images.end()?nullptr:it->second;
}
std::optional<bool> nativeFixedImageInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x2f3714 && pc!=0x2ea9a0)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *textures=d?dynamic_cast<TextureOwner *>(&d->host()):nullptr;
    if(!textures || !c)throw std::runtime_error("native fixed image has no texture owner");
    const auto object=c->r[4].uw[0];
    if(pc==0x2ea9a0 && !object)return false;
    record(rt,object,0xf0);
    if(read<uint32_t>(rt,object)!=0x15a6f0 || read<uint32_t>(rt,object+0x90)!=0x15a76c)
        throw std::runtime_error("unowned native fixed image type");
    if(pc==0x2ea9a0){
        if(c->pc!=pc)throw std::runtime_error("native image release requires its destructor entry");
        textures->nativeTextures().releaseImage(object);return false;
    }
    auto *scene=dynamic_cast<SceneOwner *>(&d->host());
    if(!scene || c->pc!=0x2f36f0 || c->r[5].uw[0]!=object+0x90)
        throw std::runtime_error("native fixed image requires its complete producer");
    const auto current=read<uint32_t>(rt,0x35ddf8);
    record(rt,current,0x40ec);const auto layer=read<uint8_t>(rt,object+0x65);
    if(layer>1 || read<uint32_t>(rt,current+0x40a8+layer*4)!=object || read<uint8_t>(rt,current+0x40dc+layer)!=1)
        throw std::runtime_error("native fixed image escaped its scene lifetime");
    const auto fis=read<uint32_t>(rt,object+0xc0);
    if(read<uint32_t>(rt,fis)!=0x534946)throw std::runtime_error("native fixed image has no FIS identity");
    textures->nativeTextures().publishImage(object,{read<uint32_t>(rt,fis+0x14),read<uint32_t>(rt,fis+0x18)});
    return true; // Keep a0 as the image object for the real compiled software completion.
}
std::shared_ptr<const NativeTextureAsset> NativeTextures::pixelAsset(uint16_t slot)const{
    auto it=pixels.find(slot);return it==pixels.end()?nullptr:it->second;
}
std::shared_ptr<const NativeTextureAsset> NativeTextures::paletteAsset(uint16_t slot)const{
    auto it=palettes.find(slot);return it==palettes.end()?nullptr:it->second;
}
bool nativeTextureInstruction(uint8_t *,R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x29cc00 && pc!=0x29cc30 && pc!=0x29cd7c && pc!=0x29cf14 && pc!=0x29cf30)return false;
    auto *dispatcher=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *owner=dispatcher?dynamic_cast<TextureOwner *>(&dispatcher->host()):nullptr;
    if(!owner || !c)throw std::runtime_error("native image publication requires a texture owner");
    auto &textures=owner->nativeTextures();const uint32_t stack=c->r[29].uw[0];
    if(pc==0x29cc00){
        if(c->r[18].uw[0]!=0x314200)throw std::runtime_error("unported dynamic palette producer");
        textures.publishStaticPalette(static_cast<uint16_t>(c->r[16].uw[0]));
        c->r[4].ud[0]=c->r[17].ud[0];
    }else if(pc==0x29cc30){
        const uint32_t fis=c->r[4].uw[0];
        record(rt,stack-0x80,0x80);
        if(read<uint32_t>(rt,fis)!=0x534946)throw std::runtime_error("native image input is not a FIS resource");
        textures.begin(stack-0x80,{read<uint32_t>(rt,fis+0x14),read<uint32_t>(rt,fis+0x18)});
        c->r[29].sd[0]=static_cast<int32_t>(stack-0x80);
    }else if(pc==0x29cf30){
        textures.finish(stack);c->r[31].ud[0]=read<uint64_t>(rt,stack+0x70);
    }else{
        const bool palette=pc==0x29cd7c;
        textures.publish(stack,static_cast<uint16_t>(c->r[palette?21:20].uw[0]),palette);
        // Patched scalar argument for the retained compiled heap free. Nothing
        // submits a graphics job, and the descriptor itself is never read here.
        c->r[4].ud[0]=c->r[palette?17:19].ud[0];
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:texture] native %s slot=%u, descriptor released by compiled free\n",palette?"palette":"pixels",c->r[palette?21:20].uw[0]);
    }
    return true;
}
}

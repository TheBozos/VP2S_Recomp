#include "native_textures.h"
#include "native_picture_source.h"
#include "native_scene.h"
#include "native_assets.h"
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
}
std::shared_ptr<const bozo::render::Texture> NativeTextureAsset::picture(unsigned csa)const{
    if(csa>15)throw std::runtime_error("native texture palette part outside range");
    std::lock_guard lock(picturesLock);if(pictures[csa])return pictures[csa];
    auto result=std::make_shared<bozo::render::Texture>();result->width=width;result->height=height;
    result->key=(identity<<4)|csa;result->version=identity;
    result->rgba.resize(size_t(width)*height);
    for(size_t i=0;i<result->rgba.size();++i){
        uint32_t value;
        if(format==0)std::memcpy(&value,texels.data()+i*4,4);
        else value=palette[format==0x14?csa*16+texels[i]:texels[i]];
        // Both engine presenters expand the game's 0x80-scale alpha. Preserve
        // it here so partial coverage is scaled once, at presentation.
        result->rgba[i]=value;
    }
    pictures[csa]=result;notePicture(result,PictureSource{.kind=PictureSource::Kind::Asset,.asset=key,.part=csa});return result;
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
void NativeTextures::beginMenuLoad(uint32_t object,uint32_t page,uint32_t format,uint32_t palette){menuLoads[object]=MenuLoad{page,format,palette};}
NativeTextures::MenuLoad NativeTextures::takeMenuLoad(uint32_t object){
    auto it=menuLoads.find(object);if(it==menuLoads.end())throw std::runtime_error("native menu image publication has no recorded load call");
    auto result=it->second;menuLoads.erase(it);return result;
}
void NativeTextures::publishMenuPage(uint16_t page,uint8_t format,uint16_t palette,TextureKey key){
    auto asset=load(key);menuPages[(uint32_t(page)<<8)|format]=MenuPage{std::move(asset),palette};++publications;
}
std::shared_ptr<const NativeTextureAsset> NativeTextures::menuPage(uint16_t page,uint8_t format,uint16_t palette)const{
    auto it=menuPages.find((uint32_t(page)<<8)|format);
    return it==menuPages.end() || it->second.palette!=palette?nullptr:it->second.asset;
}
void NativeTextures::publishBlock(uint32_t block,bool palette,TextureKey key){
    if(!block || block>=0x4000)throw std::runtime_error("native battle image block outside graphics memory");
    auto asset=load(key);(palette?blockPalettes:blockPixels)[block]=std::move(asset);++publications;
}
std::shared_ptr<const NativeTextureAsset> NativeTextures::blockAsset(uint32_t block,bool palette)const{
    const auto &m=palette?blockPalettes:blockPixels;auto it=m.find(block);return it==m.end()?nullptr:it->second;
}
std::shared_ptr<const NativeTextureAsset> NativeTextures::imageAsset(uint32_t object)const{
    auto it=images.find(object);return it==images.end()?nullptr:it->second;
}
// mod003 class 165630: a texture uploader for event effects. Its update 443270
// fills up to 16 entries (0x18 bytes at +10a8: TEX0 pair +10a8/+10b0, image
// +10b8, callback +10bc) through the game's texture manager, and builds one
// REF(packet)/REF(pixels)/END chain per upload at +1228 (count +12a8). Its draw
// 4431c0 queued each chain as a type-3 job (the last with owner +90, completion
// 443190). Natively the enqueue call at 443234 is patched to 443190 and this
// site (its delay slot 443238) publishes every registered FIS image under its
// entry address, the identity the event-effect consumers reach through the
// getter 4430e0. With several chains the completion runs once per chain; the
// repeats are no-ops (the flag is already set and 21c9a0 finds the scene slot
// cleared).
std::optional<bool> nativeTextureUploaderInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x443238)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *textures=d?dynamic_cast<TextureOwner *>(&d->host()):nullptr;auto *assets=d?dynamic_cast<AssetOwner *>(&d->host()):nullptr;
    // Block entries: the draw method (first chain) or the loop label after the previous call.
    if(!textures || !assets || !c || (c->pc!=0x4431c0 && c->pc!=0x44323c))throw std::runtime_error("native texture uploader requires its draw method");
    const auto object=c->r[18].uw[0],slot=c->r[16].uw[0];
    record(rt,object,0x12b0);
    if(read<uint32_t>(rt,object)!=0x165630 || read<uint32_t>(rt,object+0x90)!=0x1656ac)throw std::runtime_error("unowned native texture uploader");
    const auto count=read<int32_t>(rt,object+0x12a8),index=int32_t(c->r[17].uw[0]);
    if(count<1 || count>32 || index<0 || index>=count || slot!=object+4*uint32_t(index) || c->r[4].uw[0]!=read<uint32_t>(rt,0x35dd78) ||
       c->r[5].uw[0]!=3 || c->r[6].uw[0] || c->r[7].uw[0]!=read<uint32_t>(rt,slot+0x1228) || c->r[9].uw[0]!=(index==count-1?object+0x90:0u))
        throw std::runtime_error("native texture uploader submission escaped its draw");
    for(uint32_t k=0;k<16;++k){
        const auto entry=object+0x10a8+0x18*k,image=read<uint32_t>(rt,entry+0x10);
        if(!image){textures->nativeTextures().releaseImage(entry);continue;}
        const auto header=(image+0x7f)&~0x7fu;record(rt,header,0x20);
        const auto origin=assets->nativeAssets().origin(header);
        if(!origin || origin->kind!=AssetOrigin::Kind::Part || read<uint32_t>(rt,header)!=0x534946)
            throw std::runtime_error(std::format("native texture uploader entry {} has no decoded FIS file (image {:x})",k,image));
        textures->nativeTextures().publishImage(entry,{read<uint32_t>(rt,header+0x14),read<uint32_t>(rt,header+0x18)});
    }
    c->r[8].ud[0]=0;c->r[4].sd[0]=int32_t(object);
    return true; // The patched call enters the real completion 443190 with the uploader.
}
// mod003 class 167368: the camp menu's image loader (four, built by 45d940
// with (page, format, palette page) = (800, 24, R+94), (1000, 24, none),
// (800, 2c, R+98), (1000, 2c, R+9c)). 45e870 loads one FIS file into a heap
// buffer (+f0, through the tracked file reader 102650), fills a palette
// descriptor (+10, when +f4) and a pixel descriptor (+80), then queued both as
// type-1 graphics-memory uploads. The images are deleted right after (45db60),
// and the menu's draws name them only by that handle: the layer constructor
// 464240 writes the record (800, 2c, R+98) its 15aa20 elements copy into their
// sprite (+5a page, +5c palette page, +5f format). Natively both enqueues
// (45e9f0, 45ea10) are patched out and this site (45ea18, after them) publishes
// the file's texture under that game-assigned handle; the ready check 45e7f0
// then finds nothing queued. The handle is the call's own arguments, recorded
// at its entry (a3 page, a2 format, t0 palette page: s1 is reused for the
// picture width before 45ea18), and cross-checked against s2/s3 there.
std::optional<bool> nativeMenuImageInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    if(pc!=0x45ea18 && pc!=0x45e870)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *textures=d?dynamic_cast<TextureOwner *>(&d->host()):nullptr;auto *assets=d?dynamic_cast<AssetOwner *>(&d->host()):nullptr;
    if(!textures || !assets || !c)throw std::runtime_error("native menu image has no texture owner");
    if(pc==0x45e870){
        if(c->pc!=pc)throw std::runtime_error("native menu image load requires its entry");
        textures->nativeTextures().beginMenuLoad(c->r[4].uw[0],c->r[7].uw[0],c->r[6].uw[0],c->r[8].uw[0]&0xffff);return false;
    }
    const auto object=c->r[20].uw[0];
    record(rt,object,0x100);
    if(read<uint32_t>(rt,object)!=0x167368)throw std::runtime_error(std::format("unowned native menu image {:x} (class {:x})",object,read<uint32_t>(rt,object)));
    const auto fis=read<uint32_t>(rt,object+0xf0);record(rt,fis,0x30);
    const auto origin=assets->nativeAssets().origin(fis);
    if(!origin || read<uint32_t>(rt,fis)!=0x534946 || read<uint32_t>(rt,object+0x70)!=fis+0x10+read<uint32_t>(rt,fis+0x24)+0x80)
        throw std::runtime_error(std::format("native menu image {:x} has no decoded FIS file (buffer {:x})",object,fis));
    const auto load=textures->nativeTextures().takeMenuLoad(object);
    const auto page=load.page,format=load.format,palette=load.palette;const bool hasPalette=read<uint8_t>(rt,object+0xf4);
    if(c->r[18].uw[0]!=page || c->r[19].uw[0]!=format || page>=0x4000 || (format!=0x24 && format!=0x2c) || hasPalette!=(palette!=0))
        throw std::runtime_error(std::format("native menu image {:x} has an unported handle (page {:x}, format {:x}, palette {:x})",object,page,format,palette));
    const TextureKey key{read<uint32_t>(rt,fis+0x14),read<uint32_t>(rt,fis+0x18)};
    textures->nativeTextures().publishMenuPage(uint16_t(page),uint8_t(format),uint16_t(palette),key);
    if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:texture] native menu image %x page=%x format=%x palette=%x key=%08x:%x\n",object,page,format,palette,key.tag,key.index);
    return false;
}
void guestDeferredFree(Runtime *rt,uint32_t list,uint32_t pointer){
    if(!pointer)return;
    auto *ring=record(rt,list,8+4*0x400);uint32_t write,read;std::memcpy(&write,ring,4);std::memcpy(&read,ring+4,4);
    if(write>=0x400)throw std::runtime_error("deferred free ring index outside its 0x400 entries");
    if(read==write)return; // Full: the game's 1205b0 drops the pointer too.
    std::memcpy(ring+8+4*write,&pointer,4);write=write+1>=0x400?0:write+1;std::memcpy(ring,&write,4);
}
// Module 27 (the battle core) loads its images with 36b4a0(FIS a0, buffer a1, a2, palette
// block a3, pixel block t0, ...): it builds a 0x70-byte image packet per part (1228d0, pixels
// at +60) and queued each as a type-1 graphics upload with flags 1 (free the packet) and, on
// the last part, an owner (class 175c60, built by mod028 506420) whose completion 5063e0
// frees the buffer (+4) and itself. The blocks come from the battle's layout table, and its
// draws name the images by them. Natively both enqueue calls (36b5d0, 36b6b0) are patched
// out: 36b5d8, which both paths reach with s2 = FIS+0x10, publishes the decoded file under
// its blocks, and each former enqueue (36b5d4, 36b6b4) runs the queue completion's frees
// through the same deferred-free rings.
std::optional<bool> nativeBattleImageInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    constexpr struct {uint32_t pc,word;} sites[]={{0x36b5d4,0x0000402d},{0x36b5d8,0x8e43000c},{0x36b6b4,0x0000402d}};
    const auto site=std::find_if(std::begin(sites),std::end(sites),[&](auto &s){return s.pc==pc;});
    if(site==std::end(sites))return std::nullopt;
    if(!rt || read<uint32_t>(rt,pc)!=site->word || read<uint32_t>(rt,0x36b4a0)!=0x27bdff50u)return std::nullopt; // another overlay here
    auto *d=dynamic_cast<bozo::ee::Dispatcher *>(rt->services);
    auto *textures=d?dynamic_cast<TextureOwner *>(&d->host()):nullptr;auto *assets=d?dynamic_cast<AssetOwner *>(&d->host()):nullptr;
    if(!c || !textures || !assets)throw std::runtime_error("native battle image has no texture owner");
    if(pc==0x36b5d8){
        const auto fis=c->r[18].uw[0]-0x10;record(rt,fis,0x30);
        if(read<uint32_t>(rt,fis)!=0x534946 || !assets->nativeAssets().origin(fis,0x30))
            throw std::runtime_error(std::format("native battle image {:x} is not a decoded FIS file (a derived image is unported)",fis));
        const TextureKey key{read<uint32_t>(rt,fis+0x14),read<uint32_t>(rt,fis+0x18)};
        const auto pixels=c->r[19].uw[0]&0xffff,palette=c->r[20].uw[0]&0xffff;
        if(pixels)textures->nativeTextures().publishBlock(pixels,false,key);
        if(palette)textures->nativeTextures().publishBlock(palette,true,key);
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))
            std::fprintf(stderr,"[vp2s:texture] battle image %x key=%08x:%x pixels=%x palette=%x\n",fis,key.tag,key.index,pixels,palette);
        return false;
    }
    // A former enqueue: (queue, 1, 1, packet, 0, owner).
    const auto packet=c->r[7].uw[0],owner=c->r[9].uw[0];record(rt,packet,0x70);
    const auto picture=pc==0x36b5d4?c->r[18].uw[0]+read<uint32_t>(rt,c->r[18].uw[0]+0x14):c->r[18].uw[0];
    if(c->r[4].uw[0]!=read<uint32_t>(rt,0x35dd78) || c->r[5].uw[0]!=1 || c->r[6].uw[0]!=1 || read<uint32_t>(rt,packet+0x60)!=picture+0x80)
        throw std::runtime_error("native battle image upload escaped its loader");
    guestDeferredFree(rt,0x1d3710,packet); // 112c70, flags 1: the packet (list 101070(1c8d00)).
    if(owner){
        record(rt,owner,8);const auto vtable=read<uint32_t>(rt,owner);
        if(vtable!=0x175c60 || read<uint32_t>(rt,vtable+0xc)!=0x5063e0 || read<uint32_t>(rt,0x5063e0)!=0x27bdffe0u)
            throw std::runtime_error(std::format("native battle image owner {:x} has an unported completion",owner));
        const auto list=read<uint32_t>(rt,0x35dda8);
        guestDeferredFree(rt,list,read<uint32_t>(rt,owner+4));guestDeferredFree(rt,list,owner); // 5063e0
    }
    return false;
}
std::optional<bool> nativeFixedImageInstruction(R5900Context *c,Runtime *rt,uint32_t pc){
    const bool resource=pc==0x447af0 || pc==0x447b84 || pc==0x447d60 || pc==0x447a48 || pc==0x447f50;
    if(!resource && pc!=0x2f3714 && pc!=0x2ea9a0)return std::nullopt;
    auto *d=rt?dynamic_cast<bozo::ee::Dispatcher *>(rt->services):nullptr;
    auto *textures=d?dynamic_cast<TextureOwner *>(&d->host()):nullptr;
    if(!textures || !c)throw std::runtime_error("native fixed image has no texture owner");
    if(resource){
        const bool lifecycle=pc==0x447af0 || pc==0x447f50;
        const auto object=lifecycle?c->r[4].uw[0]:pc==0x447b84?c->r[17].uw[0]:c->r[9].uw[0]-0x90;
        record(rt,object,0x240);
        if(read<uint32_t>(rt,object)!=0x165b40 || read<uint32_t>(rt,object+0x90)!=0x165bbc)
            throw std::runtime_error("unowned native FIS resource object");
        if(lifecycle){textures->nativeTextures().releaseImage(object);return false;}
        const auto source=read<uint32_t>(rt,object+0x228);record(rt,source,0x1c);
        auto *assets=dynamic_cast<AssetOwner *>(&d->host());const auto origin=assets?assets->nativeAssets().origin(source):std::nullopt;
        if(!origin || origin->kind!=AssetOrigin::Kind::Part || origin->part || origin->offset || read<uint32_t>(rt,source)!=0x534946)
            throw std::runtime_error("native FIS resource has no completed decoded file owner");
        const TextureKey key{read<uint32_t>(rt,source+0x14),read<uint32_t>(rt,source+0x18)};
        if(pc==0x447b84){
            if(c->r[16].uw[0]!=source)throw std::runtime_error("native FIS resource changed after decode");
            textures->nativeTextures().publishImage(object,key);return false;
        }
        const auto image=textures->nativeTextures().imageAsset(object);
        if(!image || image->key!=key || c->r[4].uw[0]!=read<uint32_t>(rt,0x35dd78) || c->r[5].uw[0]!=3 || c->r[6].uw[0] ||
           c->r[7].uw[0]!=NativeAssets::address(read<uint32_t>(rt,object+0x22c)) ||
           (pc==0x447a48 && read<uint8_t>(rt,object+0x64)!=1))
            throw std::runtime_error("native FIS resource submission escaped its asset lifetime");
        // Native asset publication really succeeded before this original game
        // completion runs. No physical upload, packet input or IRQ result.
        c->r[4].sd[0]=int32_t(object);return false;
    }
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
void stateAsset(bozo::state::Archive &a,NativeTextures &textures,std::shared_ptr<const NativeTextureAsset> &asset){
    bool has=asset!=nullptr;TextureKey key=has?asset->key:TextureKey{};   // before `has` is read back
    bozo::state::io(a,has);io(a,key);
    if(a.loading())asset=has?textures.load(key):nullptr;
}
void NativeTextures::serialize(bozo::state::Archive &a){
    namespace st=bozo::state;
    if(a.loading())loaded.clear();
    state::map(a,pending,[&](Invocation &v){stateAsset(a,*this,v.asset);st::fields(a,v.palette,v.pixels);});
    state::map(a,palettes,[&](auto &v){stateAsset(a,*this,v);});
    state::map(a,pixels,[&](auto &v){stateAsset(a,*this,v);});
    state::map(a,images,[&](auto &v){stateAsset(a,*this,v);});
    state::map(a,menuPages,[&](MenuPage &v){stateAsset(a,*this,v.asset);st::io(a,v.palette);});
    state::map(a,menuLoads,[&](MenuLoad &v){st::fields(a,v.page,v.format,v.palette);});
    st::io(a,publications);
}
void NativeTextures::serializeBlocks(bozo::state::Archive &a){
    state::map(a,blockPixels,[&](auto &v){stateAsset(a,*this,v);});
    state::map(a,blockPalettes,[&](auto &v){stateAsset(a,*this,v);});
}
}

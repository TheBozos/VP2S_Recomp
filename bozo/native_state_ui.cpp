#include "native_state_ui.h"
#include <map>
#include <mutex>
#include <stdexcept>
namespace vp2s {
namespace {
std::mutex registryLock;
std::map<const bozo::render::Texture *,std::pair<std::weak_ptr<const bozo::render::Texture>,PictureSource>> registry;
namespace st=bozo::state;
template<class T>void podVector(st::Archive &a,std::vector<T> &v){auto n=st::count(a,v.size());if(a.loading())v.resize(n);for(auto &e:v)st::pod(a,e);}
}
void notePicture(const std::shared_ptr<const bozo::render::Texture> &picture,const PictureSource &source){
    if(!picture)return;
    std::lock_guard guard(registryLock);
    if(registry.size()>4096)std::erase_if(registry,[](const auto &e){return e.second.first.expired();});
    registry[picture.get()]={picture,source};
}
void statePicture(st::Archive &a,const PictureContext &c,std::shared_ptr<const bozo::render::Texture> &picture){
    PictureSource s;
    if(!a.loading() && picture){std::lock_guard guard(registryLock);
        auto it=registry.find(picture.get());
        if(it==registry.end() || it->second.first.lock()!=picture)throw std::runtime_error("save state: a held picture has no recorded source");
        s=it->second.second;}
    st::fields(a,s.kind,s.asset,s.second,s.font,s.index,s.part);
    if(!a.loading())return;
    switch(s.kind){
    case PictureSource::Kind::None:picture=nullptr;break;
    case PictureSource::Kind::Asset:picture=c.textures.load(s.asset)->picture(s.part);break;
    case PictureSource::Kind::Glyph:picture=c.glyphs.load(s.font)->picture(s.index,s.part);break;
    case PictureSource::Kind::Backdrop:picture=nativeMenuBackdropPicture(*c.textures.load(s.asset),*c.textures.load(s.second));break;
    default:throw std::runtime_error("save state: unknown picture source");
    }
}
void stateFont(st::Archive &a,const PictureContext &c,std::shared_ptr<const NativeGlyphFont> &font){
    bool has=font!=nullptr;GlyphFontKey key=has?font->key:GlyphFontKey{};   // before `has` is read back
    st::io(a,has);io(a,key);
    if(a.loading())font=has?c.glyphs.load(key):nullptr;
}
void stateSprite(st::Archive &a,const PictureContext &c,NativeUiSprite &s){
    st::fields(a,s.sprite,s.colour,s.depth,s.fixedCorners,s.uv,s.additive);statePicture(a,c,s.picture);
}
void stateWindow(st::Archive &a,const PictureContext &c,NativeUiWindow &w){
    st::io(a,w.element);
    auto n=st::count(a,w.background.size());if(a.loading())w.background.resize(n);
    for(auto &s:w.background)st::fields(a,s.corners,s.colours,s.depth);
    n=st::count(a,w.border.size());if(a.loading())w.border.resize(n);
    for(auto &s:w.border)stateSprite(a,c,s);
    state::optional(a,w.scissor);
}
void stateText(st::Archive &a,const PictureContext &c,NativeUiText &t){
    st::fields(a,t.element,t.inlineImages,t.iconsCaptured);
    auto n=st::count(a,t.glyphs.size());if(a.loading())t.glyphs.resize(n);
    for(auto &g:t.glyphs){
        st::fields(a,g.box,g.scale,g.cellWidth,g.index,g.colour,g.shadowMode,g.shadowColour,g.placed);
        st::pod(a,g.placement);stateFont(a,c,g.font);statePicture(a,c,g.picture);statePicture(a,c,g.shadow);
    }
    for(auto *icons:{&t.icons,&t.lateIcons}){n=st::count(a,icons->size());if(a.loading())icons->resize(n);for(auto &s:*icons)stateSprite(a,c,s);}
    bool hasWindow=t.window!=nullptr;st::io(a,hasWindow);
    if(a.loading()){if(hasWindow){auto w=std::make_shared<NativeUiWindow>();stateWindow(a,c,*w);t.window=std::move(w);}else t.window=nullptr;}
    else if(hasWindow){NativeUiWindow copy=*t.window;stateWindow(a,c,copy);}
    podVector(a,t.additiveTriangles);podVector(a,t.alphaTriangles);
}
void NativeUiTexts::serialize(st::Archive &a,const PictureContext &c){
    if(!builds.empty())throw std::runtime_error("native UI text state saved with a build in flight");
    for(auto *m:{&texts,&plates})state::map(a,*m,[&](std::shared_ptr<const NativeUiText> &p){
        NativeUiText t=p?*p:NativeUiText{};stateText(a,c,t);if(a.loading())p=std::make_shared<NativeUiText>(std::move(t));});
}
void NativeUiWindows::serialize(st::Archive &a,const PictureContext &c){
    if(!unsaveable().empty())throw std::runtime_error("native UI window state saved with a capture in flight");
    state::map(a,ready,[&](std::shared_ptr<const NativeUiWindow> &p){
        NativeUiWindow w=p?*p:NativeUiWindow{};stateWindow(a,c,w);if(a.loading())p=std::make_shared<NativeUiWindow>(std::move(w));});
    st::io(a,captures);
}
}

// Initial BozoEngine bring-up. Unsupported boundaries fail explicitly.
#include "native_cast.h"
#include "bozo/ee/dispatcher.h"
#include <bit>
#include <unordered_map>
#include <x86intrin.h>
#include "bozo/host/free_host.h"
#include "bozo/port.h"
#include "vp2_bozo_images.h"
#include "native_kernel.h"
#include "native_warp.h"
#include "native_timer.h"
#include "native_settings.h"
#include "native_seed.h"
#include "native_modules.h"
#include "vp2_dbc_image.h"
#include "vp2_controller_image.h"
#include "vp2_sound_image.h"
#include "vp2_csm_image.h"
#include "vp2_csd_image.h"
#include "vp2_csi_image.h"
#include "native_controller.h"
#include "native_frame.h"
#include "native_conditional_sites.h"
#include "native_cheats.h"
#include "native_bootstrap.h"
#ifdef VP2S_BOZO_WINDOW
#include "bozo/app/app.h"
#include "bozo/app/audio_device.h"
#include "bozo/app/device_pad.h"
#include "bozo/app/frame_pacer.h"
#include "bozo/app/window.h"
#include "bozo/app/settings_tabs.h"
#include <imgui.h>
#include <sstream>
#endif
#include "native_random.h"
#include "native_math.h"
#include "native_textures.h"
#include "native_scene.h"
#include "native_assets.h"
#include "native_debug_font.h"
#include "native_state_setup.h"
#include "native_screen_rect.h"
#include "native_battle_sprites.h"
#include "native_capture_fade.h"
#include "native_entry_capture.h"
#include "native_spr_writer.h"
#include "native_ui_text.h"
#include "native_screen_zoom.h"
#include "native_depth_blur.h"
#include "native_depth_probe.h"
#include "native_column_capture.h"
#include "native_ui_sprites.h"
#include "native_media_buffer.h"
#include "native_publication.h"
#include "native_models.h"
#include "native_frame_filter.h"
#include "native_frame_copy.h"
#include "native_dof.h"
#include "native_haze.h"
#include "native_scene_clip.h"
#include "native_map_panel.h"
#include "native_feedback.h"
#include "native_particles.h"
#include "native_particle_draws.h"
#include "native_procedural.h"
#include "native_procedural_draws.h"
#include "native_music.h"
#include "native_ui_windows.h"
#include "native_state_ui.h"
#include "native_load_progress.h"
#ifdef VP2S_BOZO_CAPTURE
#include "native_capture.h"
#endif
#include "bozo/render/renderer.h"
#include <random>
#include <algorithm>
#include <charconv>
#include <filesystem>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <set>

namespace {
uint32_t lastImageEnqueue[4]={}; // diagnostic: ra, source, count, owner of the last type-1 enqueue
class StrictHost final : public bozo::ee::Host,public vp2s::TimerOwner,public vp2s::SettingsOwner,public vp2s::SeedOwner,
    public vp2s::ModuleOwner,public vp2s::BootstrapOwner,public vp2s::RandomOwner,public vp2s::TextureOwner,public vp2s::SceneOwner,
    public vp2s::AssetOwner,public vp2s::DebugFontOwner,public vp2s::StateSetupOwner,public vp2s::ScreenRectOwner,public vp2s::CaptureFadeOwner,public vp2s::EntryCaptureOwner,public vp2s::ScreenZoomOwner,public vp2s::UiTextOwner,
    public vp2s::UiSpriteOwner,public vp2s::BattleSpriteOwner,public vp2s::MediaBufferOwner,public vp2s::ModelOwner,public vp2s::FrameFilterOwner,public vp2s::MusicOwner,
    public vp2s::UiWindowOwner,public vp2s::LoadProgressOwner,public vp2s::MotionOwner,public vp2s::FrameCopyOwner,public vp2s::DofOwner,public vp2s::FeedbackOwner,public vp2s::SquadOwner,
    public vp2s::ParticleOwner,public vp2s::ParticleUpdateOwner,public vp2s::ParticleDrawOwner,public vp2s::ProceduralOwner,public vp2s::ProceduralDrawOwner,public vp2s::DepthBlurOwner,public vp2s::DepthProbeOwner,public vp2s::ColumnCaptureOwner,public vp2s::HazeOwner,public vp2s::SceneClipOwner,public vp2s::MapPanelOwner {
public:
    vp2s::NativeProceduralDraws proceduralDraws;
    vp2s::NativeProceduralDraws &nativeProceduralDraws()override{return proceduralDraws;}
    vp2s::NativeProceduralTransforms proceduralTransforms;
    vp2s::NativeProceduralTransforms &nativeProceduralTransforms()override{return proceduralTransforms;}
    vp2s::NativeParticles particles;
    vp2s::NativeParticles &nativeParticles()override{return particles;}
    vp2s::NativeParticleDraws particleDraws{std::filesystem::path(VP2S_BOZO_IMAGES).parent_path()/"particle_materials"};
    vp2s::NativeParticleDraws &nativeParticleDraws()override{return particleDraws;}
    vp2s::NativeParticleUpdates particleUpdates;
    vp2s::NativeParticleUpdates &nativeParticleUpdates()override{return particleUpdates;}
    vp2s::NativeSquads squads;
    vp2s::NativeSquads &nativeSquads()override{return squads;}
    vp2s::NativeMotion motion;
    vp2s::NativeMotion &nativeMotion()override{return motion;}
    bool qualifiedMotionCallback(Runtime *rt,uint32_t target)const override{
        if(target==0x3749f0)return rt->functions.generated(target)==vp2_mod003_linked_entry_003749f0_0x3749f0;
        // mod001 1e80e0 (the battle's objects): the same five words (+50 = 1, a1 copied to +30),
        // so it leaves the motion's inputs as 3749f0 does.
        constexpr uint32_t words[5]={0x24030001,0xa0830050,0x78a30000,0x03e00008,0x7c830030};
        const auto *code=target==0x1e80e0?rt->memory.hostPointer(target,20):nullptr;
        return code && !std::memcmp(code,words,20);
    }
    vp2s::NativeLoadProgress loadingProgress;
    vp2s::NativeLoadProgress &nativeLoadProgress()override{return loadingProgress;}
    vp2s::NativeUiWindows windows;
    vp2s::NativeUiWindows &nativeUiWindows()override{return windows;}
    vp2s::NativeMusic music{std::filesystem::path(VP2S_BOZO_IMAGES).parent_path().parent_path()/"native_music/tables.bin"};
    vp2s::NativeMusic &nativeMusic()override{return music;}
    vp2s::NativeFrameFilters frameFilters;
    vp2s::NativeFrameFilters &nativeFrameFilters()override{return frameFilters;}
    void drawNativeFrameFilter(const vp2s::NativeFrameFilter &filter)override{flushDeferredModels();
        const auto target=scene.frameTarget();if(!target || target->object!=filter.source)throw std::runtime_error("native frame filter has no source frame");
        auto &renderer=bozo::render::Renderer::instance();
        const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        const float xy[6][2]={{0,0},{1,0},{0,1},{1,0},{1,1},{0,1}};
        const auto draw=[&](bozo::render::Material material,uint32_t colour){
            bozo::render::Vertex vertices[6];for(unsigned i=0;i<6;++i){auto &v=vertices[i];v.x=2048-target->width/2+xy[i][0]*target->width;
                v.y=2048-target->height/2+xy[i][1]*target->height;v.u=xy[i][0];v.v=xy[i][1];v.rgba=colour;}
            renderer.draw(material,identity,vertices,6);
        };
        // One immutable native copy of the source, followed by the game's
        // successive 256,128,64,32,16 square bilinear reductions. No transport.
        bozo::render::Material material;material.textured=false;material.depthTest=false;material.depthWrite=false;
        material.blend=bozo::render::Material::Blend::Opaque;material.alphaBlend=false;material.fromFrame=1;
        material.target=6;material.nearest=true;renderer.setTargetSize(6,uint16_t(target->width),uint16_t(target->height));draw(material,0x80808080);
        for(unsigned i=0;i<filter.levels;++i){const auto side=256u>>i;renderer.setTargetSize(uint8_t(i+1),side,side);
            material.source=i?uint8_t(i):6;material.target=uint8_t(i+1);material.nearest=false;draw(material,0x80808080);}
        material.target=0;material.alphaBlend=true;material.blend=bozo::render::Material::Blend::Alpha;
        for(const auto &layer:filter.layers){material.source=uint8_t(layer.level+1);draw(material,layer.colour);}
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:filter] native object=%x levels=%u layers=%zu\n",filter.object,filter.levels,filter.layers.size());
    }
    vp2s::NativeModels models{std::filesystem::path(VP2S_BOZO_IMAGES).parent_path()/"meshes"};
    vp2s::NativeFrameCopies frameCopies;
    vp2s::NativeFrameCopies &nativeFrameCopies()override{return frameCopies;}
    void drawNativeFrameCopy(const vp2s::NativeFrameCopy &copy)override{flushDeferredModels();vp2s::drawFrameCopy(copy);}
    vp2s::NativeDofs dofs;
    vp2s::NativeDofs &nativeDofs()override{return dofs;}
    void drawNativeDof(const vp2s::NativeDof &effect)override{
        flushDeferredModels();
        static const unsigned long long traceFrom=[]{const char *v=std::getenv("VP2S_BOZO_TRACE_DOF");return v?std::strtoull(v,nullptr,10):~0ull;}();
        if(native.framesEnded()>=traceFrom)
            std::fprintf(stderr,"[vp2s:dof] frame=%llu size=%ux%u gain=%g blocks=%zu glowA=%d thr=%g weight=%g taps=%zu work=%ux%u gains=%g/%g/%g glareB=%d thr=%g in=%g out=%g work=%u rows=%u rotations=%zu\n",
                static_cast<unsigned long long>(native.framesEnded()),effect.width,effect.height,effect.gain,effect.blocks.size(),effect.glowA,effect.glowThreshold*255.f,effect.glowWeight*128.f,
                effect.glowTaps.size(),effect.glowWidth,effect.glowHeight,effect.glowLevelGain,effect.glowHorizontal,effect.glowVertical,effect.glareB,effect.glareThreshold*255.f,effect.glareIn,effect.glareOut*128.f,effect.glareWork,effect.glareRows,effect.glareCorners.size());
        vp2s::drawDof(effect);}
    void drawNativeHaze(const vp2s::NativeHaze &haze)override{flushDeferredModels();vp2s::drawHaze(haze);
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:haze] native object=%x vertices=%zu frame=%llu\n",haze.object,haze.vertices.size(),static_cast<unsigned long long>(native.framesEnded()));}
    void drawNativeMapPanel(const vp2s::NativeMapPanel &panel)override{flushDeferredModels();vp2s::drawMapPanel(panel);}
    // A scene SCISSOR (class 1597e0) clips every later draw of the frame, as on the GS; earlier deferred models
    // are drawn first, and the next native frame starts unclipped.
    void setNativeSceneClip(const vp2s::NativeSceneClip &clip)override{flushDeferredModels();const auto target=scene.frameTarget();
        const float originX=2048.f-float(target->width/2),originY=2048.f-float(target->height/2);const auto &r=clip.rect;
        sceneClip={originX+float(r[0]),originY+float(r[1]),originX+float(r[2]+1),originY+float(r[3]+1)};
        bozo::render::Renderer::instance().setClip(sceneClip[0],sceneClip[1],sceneClip[2],sceneClip[3]);}
    std::array<float,4> sceneClip{};   // the scene SCISSOR in force (x1 <= x0: none)
    vp2s::NativeFeedbackCopies feedbackCopies;
    vp2s::NativeFeedbackCopies &nativeFeedbackCopies()override{return feedbackCopies;}
    void drawNativeFeedbackCopy(const vp2s::NativeFeedbackCopy &copy)override{flushDeferredModels();vp2s::drawFeedbackCopy(copy);}
    vp2s::NativeModels &nativeModels()override{return models;}
    // Models composed by a shadow owner wait for the owner's shadow pass (see
    // NativeModelSubmission::deferUntil); any other draw or a frame boundary
    // draws them first, so nothing else changes order.
    std::vector<vp2s::NativeModelSubmission> deferredModels;uint32_t deferredOwner=0;
    void drawModelBatches(const vp2s::NativeModelSubmission &submission){
        for(const auto &batch:submission.batches)bozo::render::Renderer::instance().draw(batch.material,batch.matrix.data(),batch.vertices.data(),batch.vertices.size(),batch.texture);
    }
    void flushDeferredModels(){
        for(const auto &s:deferredModels)drawModelBatches(s);
        deferredModels.clear();deferredOwner=0;
    }
    void drawNativeModel(const vp2s::NativeModelSubmission &submission)override{
        if(submission.shadow){
            if(deferredOwner && deferredOwner!=submission.object)flushDeferredModels();
            const auto target=scene.frameTarget();if(!target)throw std::runtime_error("native shadow has no active frame");
            const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
            for(const auto &d:vp2s::nativeShadowPass(submission,target->width,target->height))
                bozo::render::Renderer::instance().draw(d.material,identity,d.vertices.data(),d.vertices.size());
            flushDeferredModels();
        }
        else if(submission.deferUntil){
            if(deferredOwner && deferredOwner!=submission.deferUntil)flushDeferredModels();
            deferredModels.push_back(submission);deferredOwner=submission.deferUntil;
        }
        else{flushDeferredModels();drawModelBatches(submission);}
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:models] native object=%x camera=%x batches=%zu shadow=%zu/%zu defer=%x\n",submission.object,submission.camera,
            submission.batches.size(),submission.shadowIncrement.size()/3,submission.shadowDecrement.size()/3,submission.deferUntil);
    }
    std::set<uint32_t> overlayExternals;
    vp2s::NativeMediaBuffers mediaBuffers;
    vp2s::NativeMediaBuffers &nativeMediaBuffers()override{return mediaBuffers;}
    void presentNativeMovie(uint32_t object,const bozo::libs::YuvPicture &picture,uint64_t version)override{flushDeferredModels();
        const auto target=scene.frameTarget();if(!target)throw std::runtime_error("native movie has no active presentation target");
        bozo::render::Screen screen;screen.width=target->width;screen.height=target->height;
        screen.originX=2048-target->width/2;screen.originY=2048-target->height/2;
        const auto texture=vp2s::nativeMoviePicture(picture,0x5650325300000000ull|object,version);
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES") && (version<=2 || version%30==0)){
            uint64_t sum=0;uint32_t lit=0;
            for(const auto rgba:texture->rgba){const auto rgb=(rgba&255)+((rgba>>8)&255)+((rgba>>16)&255);sum+=rgb;if(rgb>30)++lit;}
            std::fprintf(stderr,"[vp2s:media] native movie pixels version=%llu meanRGB=%.3f lit=%u\n",(unsigned long long)version,double(sum)/(texture->rgba.size()*3),lit);
        }
        bozo::render::Renderer::instance().setBackdrop(texture,screen,screen.originX,screen.originY,screen.width,screen.height,true);
    }
    void closeNativeMovie(uint32_t)override{bozo::render::Renderer::instance().clearBackdrop();}
    vp2s::NativeUiSprites sprites;
    vp2s::NativeUiSprites &nativeUiSprites()override{return sprites;}
    vp2s::NativeGlyphs glyphs{std::filesystem::path(VP2S_BOZO_IMAGES).parent_path()/"glyphs"};
    vp2s::NativeUiTexts uiTexts;
    vp2s::NativeUiTexts &nativeUiTexts()override{return uiTexts;}
    vp2s::NativeGlyphs &nativeGlyphs()override{return glyphs;}
    void drawNativeUiLayer(const vp2s::NativeUiLayer &layer)override{flushDeferredModels();
        if(!scene.frameTarget())throw std::runtime_error("native UI layer has no active frame");
        const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        bozo::render::Material material;material.blend=bozo::render::Material::Blend::Alpha;
        material.alphaTest=true;material.alphaRef=0;
        // A scissor element (15a420) clips everything after it in this layer;
        // the layer starts and ends unclipped. Frame pixels map onto primitive
        // coordinates from the frame's origin, as the presenter expects.
        auto &renderer=bozo::render::Renderer::instance();renderer.clearClip();sceneClip={};
        const float originX=2048.f-float(scene.frameTarget()->width/2),originY=2048.f-float(scene.frameTarget()->height/2);
        for(const auto &text:layer.texts){
        if(text->window && text->window->scissor){const auto &s=*text->window->scissor;
            renderer.setClip(originX+float(s[0]),originY+float(s[1]),originX+float(s[2]+1),originY+float(s[3]+1));continue;}
        material.blend=bozo::render::Material::Blend::Alpha;for(const auto &glyph:text->glyphs){
            for(bool shadow:{true,false}){
                if(shadow && !glyph.shadow)continue;
                const auto vertices=vp2s::nativeUiGlyphVertices(glyph,shadow);
                bozo::render::Renderer::instance().draw(material,identity,vertices.data(),vertices.size(),shadow?glyph.shadow:glyph.picture);
            }
        }
        if(text->window){
            const auto flat=vp2s::nativeUiFlatMaterial(bozo::render::Material::Blend::Alpha);
            for(const auto &strip:text->window->background){const auto vertices=vp2s::nativeUiStripVertices(strip);bozo::render::Renderer::instance().draw(flat,identity,vertices.data(),vertices.size());}
            for(const auto &sprite:text->window->border){const auto vertices=vp2s::nativeUiSpriteVertices(sprite);bozo::render::Renderer::instance().draw(material,identity,vertices.data(),vertices.size(),sprite.picture);}
        }
        if(!text->additiveTriangles.empty()){
            const auto additive=vp2s::nativeUiFlatMaterial(bozo::render::Material::Blend::Additive);
            bozo::render::Renderer::instance().draw(additive,identity,text->additiveTriangles.data(),text->additiveTriangles.size());
        }
        for(const auto &icon:text->icons){
            material.blend=icon.additive?bozo::render::Material::Blend::Additive:bozo::render::Material::Blend::Alpha;
            const auto vertices=vp2s::nativeUiSpriteVertices(icon);
            bozo::render::Renderer::instance().draw(material,identity,vertices.data(),vertices.size(),icon.picture);
        }
        if(!text->alphaTriangles.empty()){
            const auto flat=vp2s::nativeUiFlatMaterial(bozo::render::Material::Blend::Alpha);
            bozo::render::Renderer::instance().draw(flat,identity,text->alphaTriangles.data(),text->alphaTriangles.size());
        }
        for(const auto &icon:text->lateIcons){
            material.blend=bozo::render::Material::Blend::Alpha;
            const auto vertices=vp2s::nativeUiSpriteVertices(icon);
            bozo::render::Renderer::instance().draw(material,identity,vertices.data(),vertices.size(),icon.picture);
        }}
        renderer.clearClip();sceneClip={};
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:UI] native layer=%x texts=%zu frame=%llu\n",layer.layer,layer.texts.size(),static_cast<unsigned long long>(native.framesEnded()));
    }
    vp2s::NativeBattleSprites battleSprites;
    vp2s::NativeBattleSprites &nativeBattleSprites()override{return battleSprites;}
    // The battle UI's sprite list (class 174e30): textured, ALPHA_2 = 0x44, the UI material.
    void drawNativeBattleSprites(const vp2s::NativeBattleSpriteBuild &build)override{flushDeferredModels();
        if(!scene.frameTarget())throw std::runtime_error("native battle sprites have no active frame");
        const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        bozo::render::Material material;material.blend=bozo::render::Material::Blend::Alpha;material.alphaTest=true;material.alphaRef=0;
        // Flat polygons (42df80) carry no picture: untextured, the same blend.
        // Screen lenses (class 16e9e0) sample the frame presented last, nearest, depth-tested GREATER with depth
        // writes (TEST_2 70000, ZBUF_2 without ZMSK); their vertex alpha already carries TEXA's 0x60.
        bozo::render::Material lens;lens.textured=false;lens.fromFrame=1;lens.source=bozo::render::Material::kPreviousFrame;lens.nearest=true;
        lens.textureAlpha=false;lens.depthTest=true;lens.depthGreater=true;lens.depthWrite=true;
        for(size_t i=0;i<build.sprites.size();++i){const auto &sprite=build.sprites[i];const auto vertices=vp2s::nativeUiSpriteVertices(sprite);
            const auto blend=sprite.additive?bozo::render::Material::Blend::Additive:bozo::render::Material::Blend::Alpha;
            if(build.spriteTextures.at(i)==vp2s::NativeBattleSpriteBuild::kPreviousFrame){lens.blend=blend;bozo::render::Renderer::instance().draw(lens,identity,vertices.data(),vertices.size());continue;}
            material.textured=bool(sprite.picture);material.blend=blend;
            bozo::render::Renderer::instance().draw(material,identity,vertices.data(),vertices.size(),sprite.picture);}
    }
    // Screen shockwave 1746d0: the frame presented last, redrawn displaced. TEX1_2 61 (bilinear) or 1; TEXA 80 with a
    // PSMCT24 texture, so the vertex alpha is the blend's As (ALPHA_2 44); the fan depth-tests always, the ring GREATER;
    // depth writes unless ZBUF_2 has ZMSK.
    void drawNativeShockwave(const vp2s::NativeShockwave &wave)override{flushDeferredModels();
        if(!scene.frameTarget())throw std::runtime_error("native screen shockwave has no active frame");
        const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        bozo::render::Material m;m.textured=false;m.fromFrame=1;m.source=bozo::render::Material::kPreviousFrame;m.nearest=!wave.linear;
        m.textureAlpha=false;m.blend=bozo::render::Material::Blend::Alpha;
        const auto draw=[&](const std::vector<vp2s::NativeUiSprite> &sprites,bool greater,bool write){
            m.depthTest=greater;m.depthGreater=greater;m.depthWrite=write;
            for(const auto &s:sprites){const auto v=vp2s::nativeUiSpriteVertices(s);bozo::render::Renderer::instance().draw(m,identity,v.data(),v.size());}};
        draw(wave.fan,false,wave.fanDepthWrite);draw(wave.strips,true,wave.stripDepthWrite);
    }
    vp2s::NativeScreenRects screenRects;
    unsigned screenRectDraws=0;
    vp2s::NativeScreenRects &nativeScreenRects()override{return screenRects;}
    void drawNativeScreenRect(const vp2s::NativeScreenRect &rect)override{flushDeferredModels();
        if(!scene.frameTarget())throw std::runtime_error("native screen rectangle has no source frame");
        bozo::render::Material material;material.textured=false;material.alphaTest=true;
        material.blend=bozo::render::Material::Blend::Alpha;
        if(rect.alpha!=0x44){   // the fade's own ALPHA_2 (+f4: e.g. 0x148, Cs As + Cd)
            material.gsBlend=true;material.blendA=rect.alpha&3;material.blendB=(rect.alpha>>2)&3;material.blendC=(rect.alpha>>4)&3;material.blendD=(rect.alpha>>6)&3;
            material.blendFix=(rect.alpha>>32)&0xff;}
        const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        if(rect.wipe){
            // The fade's Gouraud strip (PRIM 24c) as triangles: 10 for the wipe, 2 for mode 1.
            std::vector<bozo::render::Vertex> triangles;for(unsigned k=0;k+2<rect.stripVertices && k+2<rect.strip.size();++k)for(unsigned j=0;j<3;++j)triangles.push_back(rect.strip[k+j]);
            bozo::render::Renderer::instance().draw(material,identity,triangles.data(),triangles.size());
        }else{
            const auto vertices=vp2s::nativeScreenRectangleVertices(rect);
            bozo::render::Renderer::instance().draw(material,identity,vertices.data(),vertices.size());
        }
        // VP2S_BOZO_TRACE_RECTS_FROM=<frame>: every rectangle from that frame on (fades, curtains).
        static const unsigned long long rectsFrom=[]{const char *v=std::getenv("VP2S_BOZO_TRACE_RECTS_FROM");return v?std::strtoull(v,nullptr,10):~0ull;}();
        if((++screenRectDraws<=4 && std::getenv("VP2S_BOZO_TRACE_QUEUES")) || native.framesEnded()>=rectsFrom)
            std::fprintf(stderr,"[vp2s:screen] native rect object=%x colour=%08x box=%d,%d,%d,%d frame=%llu\n",rect.object,rect.colour,rect.x,rect.y,rect.right,rect.bottom,
                (unsigned long long)native.framesEnded());
    }
    vp2s::NativeCaptureFades captureFades;
    vp2s::NativeCaptureFades &nativeCaptureFades()override{return captureFades;}
    // The frame the GS held when the capture fade began: the previous presentation,
    // copied once into a kept engine image at the main frame's own resolution.
    void copyNativeFrameToImage(uint8_t image)override{flushDeferredModels();
        if(!scene.frameTarget())throw std::runtime_error("native capture fade has no frame to copy");
        bozo::render::Material material;material.textured=false;material.fromFrame=1;material.source=bozo::render::Material::kPreviousFrame;
        material.target=image;material.nearest=true;material.alphaBlend=false;material.blend=bozo::render::Material::Blend::Opaque;
        material.depthTest=false;material.depthWrite=false;
        vp2s::NativeCaptureFadeDraw whole{.colour=0x80808080,.width=int32_t(scene.frameTarget()->width),.height=int32_t(scene.frameTarget()->height)};
        const auto vertices=vp2s::nativeCaptureFadeVertices(whole);
        const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        // The image keeps the copied frame's size when the frame changes (the battle
        // entry goes from 512x512 to 576x416): otherwise a later frame recreates it empty.
        bozo::render::Renderer::instance().setTargetSize(image,uint16_t(whole.width),uint16_t(whole.height));
        bozo::render::Renderer::instance().draw(material,identity,vertices.data(),vertices.size());
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))
            std::fprintf(stderr,"[vp2s:screen] native capture copy image=%u frame=%llu\n",image,(unsigned long long)native.framesEnded());
    }
    void drawNativeCaptureFade(const vp2s::NativeCaptureFadeDraw &draw)override{flushDeferredModels();
        const auto target=scene.frameTarget();
        if(!target || int32_t(target->width)!=draw.width || int32_t(target->height)!=draw.height)
            throw std::runtime_error("native capture fade has no matching frame");
        // TEX0.TCC 0 and modulate: the vertex alpha; ALPHA 0x44; ZTST always, ZMSK.
        bozo::render::Material material;material.textured=false;material.fromFrame=1;material.source=draw.image;material.target=0;material.nearest=true;
        material.textureAlpha=false;material.blend=bozo::render::Material::Blend::Alpha;material.depthTest=false;material.depthWrite=false;
        const auto vertices=vp2s::nativeCaptureFadeVertices(draw);
        const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        bozo::render::Renderer::instance().draw(material,identity,vertices.data(),vertices.size());
        // The field pause's copies (452040): the same sprite state, after the strips.
        if(const auto trail=vp2s::nativeCaptureFadeTrail(draw);!trail.empty())
            bozo::render::Renderer::instance().draw(material,identity,trail.data(),trail.size());
        static const unsigned long long rectsFrom=[]{const char *v=std::getenv("VP2S_BOZO_TRACE_RECTS_FROM");return v?std::strtoull(v,nullptr,10):~0ull;}();
        if(native.framesEnded()>=rectsFrom)
            std::fprintf(stderr,"[vp2s:screen] native capture fade object=%x image=%u colour=%08x copies alpha=%u frame=%llu\n",draw.object,draw.image,draw.colour,draw.trailAlpha,(unsigned long long)native.framesEnded());
    }
    // Battle-entry capture (class 15a850): the capture fade's snapshot slots and copy.
    std::map<uint32_t,uint32_t> entryGroups;   // readback group -> capture object
    uint8_t claimNativeEntryFrame(uint32_t object,uint32_t group)override{
        if(!group)throw std::runtime_error("native entry capture has no readback group");
        entryGroups[group]=object;return captureFades.capture(object);
    }
    // At the group's submit: the last complete frame (the previous presentation; the
    // frame drawn so far holds only what its scene drew before the group, e.g. the map
    // panel without the field), copied into the claimed image at its size, which the
    // image keeps when the frame changes size.
    void copyNativeEntryFrame(uint32_t group)override{flushDeferredModels();
        const auto it=entryGroups.find(group);if(it==entryGroups.end())return;   // another group
        const auto object=it->second;entryGroups.erase(it);
        const auto image=captureFades.pendingCopy(object);if(!image)throw std::runtime_error("native entry capture group has no claimed image");
        const auto target=scene.frameTarget();if(!target)throw std::runtime_error("native entry capture has no frame to copy");
        bozo::render::Material material;material.textured=false;material.fromFrame=1;material.source=bozo::render::Material::kPreviousFrame;
        material.textureAlpha=false;   // the field's alpha is 0 (the display ignores it): copy the colour as shown
        material.target=*image;material.nearest=true;material.alphaBlend=false;material.blend=bozo::render::Material::Blend::Opaque;
        material.depthTest=false;material.depthWrite=false;
        vp2s::NativeCaptureFadeDraw whole{.colour=0x80808080,.width=int32_t(target->width),.height=int32_t(target->height)};
        const auto vertices=vp2s::nativeCaptureFadeVertices(whole);
        const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        bozo::render::Renderer::instance().setTargetSize(*image,uint16_t(whole.width),uint16_t(whole.height));
        // A GS readback ignores the scissor: no scene clip (the map panel's) on the copy.
        auto &renderer=bozo::render::Renderer::instance();renderer.clearClip();
        renderer.draw(material,identity,vertices.data(),vertices.size());
        if(sceneClip[2]>sceneClip[0])renderer.setClip(sceneClip[0],sceneClip[1],sceneClip[2],sceneClip[3]);
        entrySizes[object]={whole.width,whole.height};
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))
            std::fprintf(stderr,"[vp2s:screen] native entry capture copy object=%x image=%u frame=%llu\n",object,*image,(unsigned long long)native.framesEnded());
    }
    std::map<uint32_t,std::array<int32_t,2>> entrySizes;
    std::array<int32_t,2> nativeEntryFrameSize(uint32_t object)override{const auto it=entrySizes.find(object);return it==entrySizes.end()?std::array<int32_t,2>{0,0}:it->second;}
    std::optional<uint8_t> nativeEntryFrame(uint32_t object)override{return captureFades.image(object);}
    void drawNativeEntryCapture(const vp2s::NativeEntryCaptureDraw &draw)override{flushDeferredModels();
        const auto target=scene.frameTarget();
        if(!target || int32_t(target->width)!=draw.frameWidth || int32_t(target->height)!=draw.frameHeight)
            throw std::runtime_error(std::format("native entry capture frame {}x{} has no matching target",draw.frameWidth,draw.frameHeight));
        // TEX0.TCC 0, modulate; TEX1 MMAG linear; ALPHA 0x44 with PRIM ABE; ZTE off.
        bozo::render::Material material;material.textured=false;material.fromFrame=1;material.source=draw.image;material.target=0;material.nearest=false;
        material.textureAlpha=false;material.depthTest=false;material.depthWrite=false;
        if(std::getenv("VP2S_BOZO_DEBUG_ENTRY_SHOW")){   // diagnostic: the snapshot itself, 1:1 over the frame
            vp2s::NativeCaptureFadeDraw whole{.colour=0x80808080,.width=draw.frameWidth,.height=draw.frameHeight};
            const auto quad=vp2s::nativeCaptureFadeVertices(whole);const float id[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
            bozo::render::Renderer::instance().draw(material,id,quad.data(),quad.size());return;}
        if(draw.blend)material.blend=bozo::render::Material::Blend::Alpha;else{material.alphaBlend=false;material.blend=bozo::render::Material::Blend::Opaque;}
        const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        if(!draw.vertices.empty())bozo::render::Renderer::instance().draw(material,identity,draw.vertices.data(),draw.vertices.size());
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))std::fprintf(stderr,"[vp2s:screen] native entry capture object=%x image=%u strips=%zu colour=%08x blend=%d grid=%zu frame=%llu\n",
            draw.object,draw.image,draw.vertices.size()/6,draw.colour,int(draw.blend),draw.grid.size(),(unsigned long long)native.framesEnded());
        if(!draw.grid.empty()){
            // The trail: the previous presentation, 1:1, alpha-blended (TEX1 nearest).
            bozo::render::Material trail;trail.textured=false;trail.fromFrame=1;trail.source=bozo::render::Material::kPreviousFrame;trail.target=0;trail.nearest=true;
            trail.textureAlpha=false;trail.blend=bozo::render::Material::Blend::Alpha;trail.depthTest=false;trail.depthWrite=false;
            bozo::render::Renderer::instance().draw(trail,identity,draw.grid.data(),draw.grid.size());
        }
    }
    vp2s::NativeScreenZooms screenZooms;
    unsigned screenZoomDraws=0;
    vp2s::NativeScreenZooms &nativeScreenZooms()override{return screenZooms;}
    void drawNativeScreenZoom(const vp2s::NativeScreenZoom &zoom)override{flushDeferredModels();
        const auto target=scene.frameTarget();
        if(!target || int32_t(target->width)!=zoom.width || int32_t(target->height)!=zoom.height)throw std::runtime_error("native screen zoom has no matching source frame");
        // The previous presentation is the texture (the display buffer the GS
        // pages come from): nearest, vertex alpha only (TEXA 0x60 already folded
        // in), GS alpha blend, depth test always with the game's depth written.
        bozo::render::Material material;material.fromFrame=1;material.source=bozo::render::Material::kPreviousFrame;material.target=0;material.nearest=true;
        material.textureAlpha=false;material.blend=bozo::render::Material::Blend::Alpha;material.depthTest=false;material.depthWrite=true;
        std::vector<bozo::render::Vertex> vertices;vertices.reserve(zoom.strips.size()*6);
        for(const auto &strip:zoom.strips){const auto quad=vp2s::nativeScreenZoomVertices(zoom,strip);vertices.insert(vertices.end(),quad.begin(),quad.end());}
        const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        bozo::render::Renderer::instance().draw(material,identity,vertices.data(),vertices.size());
        if(++screenZoomDraws<=4 && std::getenv("VP2S_BOZO_TRACE_QUEUES"))
            std::fprintf(stderr,"[vp2s:screen] native zoom object=%x alpha=%u strips=%zu first=%g,%g,%g,%g\n",zoom.object,zoom.alpha,zoom.strips.size(),
                zoom.strips[0].x0,zoom.strips[0].y0,zoom.strips[0].x1,zoom.strips[0].y1);
    }
    vp2s::NativeDepthProbes depthProbes;
    vp2s::NativeDepthProbes &nativeDepthProbes()override{return depthProbes;}
    vp2s::NativeDepthProbeFrame nativeDepthProbeFrame()override{
        const auto target=scene.frameTarget();
        if(!target)throw std::runtime_error("native depth probe has no frame");
        return {int32_t(target->width),int32_t(target->height)};
    }
    // Colour readbacks: the battle column capture's (native_column_capture.h); others refuse.
    bool nativeColourReadback(uint32_t node,const vp2s::NativeDepthProbeRequest &q,uint32_t destination)override{
        auto column=columnCaptures.column(node,q,destination);if(!column)return false;
        auto &capture=*column->first;
        if(!capture.copied){copyNativeColumnFrame(capture);capture.copied=true;}
        return true;
    }
    vp2s::NativeColumnCaptures columnCaptures;
    vp2s::NativeColumnCaptures &nativeColumnCaptures()override{return columnCaptures;}
    uint8_t claimNativeColumnImage(uint32_t object)override{return captureFades.capture(object);}
    void copyNativeColumnFrame(vp2s::NativeColumnCapture &capture)override{flushDeferredModels();
        const auto image=captureFades.pendingCopy(capture.object);if(!image || *image!=capture.image)throw std::runtime_error("native column capture has no claimed image");
        const auto target=scene.frameTarget();if(!target)throw std::runtime_error("native column capture has no frame to copy");
        // The frame drawn so far (source 0): the readback's place in the scene's list.
        bozo::render::Material material;material.textured=false;material.fromFrame=1;material.source=0;
        material.textureAlpha=false;material.target=*image;material.nearest=true;material.alphaBlend=false;material.blend=bozo::render::Material::Blend::Opaque;
        material.depthTest=false;material.depthWrite=false;
        vp2s::NativeCaptureFadeDraw whole{.colour=0x80808080,.width=int32_t(target->width),.height=int32_t(target->height)};
        const auto vertices=vp2s::nativeCaptureFadeVertices(whole);
        const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        auto &renderer=bozo::render::Renderer::instance();renderer.setTargetSize(*image,uint16_t(whole.width),uint16_t(whole.height));
        renderer.clearClip();renderer.draw(material,identity,vertices.data(),vertices.size());
        if(sceneClip[2]>sceneClip[0])renderer.setClip(sceneClip[0],sceneClip[1],sceneClip[2],sceneClip[3]);
        capture.frameWidth=whole.width;capture.frameHeight=whole.height;
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES"))
            std::fprintf(stderr,"[vp2s:screen] native column capture copy object=%x image=%u size=%dx%d frame=%llu\n",capture.object,*image,whole.width,whole.height,(unsigned long long)native.framesEnded());
    }
    // One column of the battle column capture: a sprite from column k of the copied frame.
    // TEX1_2 1 (nearest), TCC 0 and modulate (the vertex colour, alpha from the vertex),
    // ALPHA 0x44 with PRIM ABE, ZTST always with ZMSK.
    void drawNativeColumn(const vp2s::NativeColumnCapture &capture,unsigned k)override{flushDeferredModels();
        const auto target=scene.frameTarget();if(!target)throw std::runtime_error("native column draw has no frame");
        // Rows the readback takes past the frame (the game asks for 1 + 224 * +1dc = 417 of a
        // 416-row frame) clamp to its last row, the battle entry capture's rule for texels past
        // its snapshot (the PS2 would read the graphics memory below the frame).
        if(capture.frameWidth<64*9 || capture.frameHeight<1 || capture.height>capture.frameHeight+16)throw std::runtime_error(std::format("native column draw has no copied frame of its size (object {:x} copied {} frame {}x{} height {})",capture.object,capture.copied,capture.frameWidth,capture.frameHeight,capture.height));
        const auto &s=capture.sprites[k];
        bozo::render::Material material;material.textured=false;material.fromFrame=1;material.source=capture.image;material.target=0;material.nearest=true;
        material.textureAlpha=false;material.blend=bozo::render::Material::Blend::Alpha;material.depthTest=false;material.depthWrite=false;
        material.clampU=material.clampV=true;   // the clamp rule above
        const float W=float(capture.frameWidth),H=float(capture.frameHeight);
        const bool right[6]={false,true,false,true,true,false},bottom[6]={false,false,true,false,true,true};
        std::array<bozo::render::Vertex,6> vertices;
        for(unsigned i=0;i<6;++i){auto &v=vertices[i];v.x=float(right[i]?s.x1:s.x0);v.y=float(bottom[i]?s.y1:s.y0);v.z=float(s.z);v.rgba=s.colour;
            v.u=(64.f*float(k)+(right[i]?s.u1:s.u0))/W;v.v=(bottom[i]?s.v1:s.v0)/H;}
        const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        bozo::render::Renderer::instance().draw(material,identity,vertices.data(),vertices.size());
        static unsigned traced=0;
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES") && traced++<12)
            std::fprintf(stderr,"[vp2s:screen] native column object=%x column=%u box=%d,%d,%d,%d colour=%08x frame=%llu\n",capture.object,k,s.x0,s.y0,s.x1,s.y1,s.colour,(unsigned long long)native.framesEnded());
    }
    bool nativeDepthSample(float gsX,float gsY,uint32_t &gsZ)override{
        // The frame presented last, read back by tiles (false headless or before
        // the tile has been read; the probe then counts the sample as hidden).
        return bozo::render::Renderer::instance().depthAt(gsX,gsY,gsZ);
    }
    vp2s::NativeDepthBlurs depthBlurs;
    unsigned depthBlurDraws=0;
    vp2s::NativeDepthBlurs &nativeDepthBlurs()override{return depthBlurs;}
    void drawNativeDepthBlur(const vp2s::NativeDepthBlur &blur)override{flushDeferredModels();
        // Main 1152c0's multi-copy depth blur. The markers live in image 6, drawn
        // with the main frame's depth (the GS writes them into the frame's alpha,
        // which here is the presentation's coverage). Before each copy the frame's
        // RGB is snapshotted into image 6's RGB; the copy samples image 6 (RGB and
        // markers), offset, alpha-tested against AREF and depth-tested at its Z, into
        // the frame's RGB with ALPHA 0x44, and into image 6's alpha (As), as the GS
        // write of one RGBA frame would leave both.
        const auto target=scene.frameTarget();
        if(!target || int32_t(target->width)!=blur.width || int32_t(target->height)!=blur.height)
            throw std::runtime_error("native depth blur has no matching frame");
        static const bool skip=std::getenv("VP2S_BOZO_DEPTH_BLUR_SKIP")!=nullptr;   // diagnostic comparison only
        if(skip)return;
        using bozo::render::Material;auto &renderer=bozo::render::Renderer::instance();
        const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        constexpr uint8_t image=6;
        Material plane;plane.textured=false;plane.target=image;plane.frameDepth=true;plane.depthTest=true;plane.depthWrite=false;
        plane.gsBlend=true;plane.blendA=2;plane.blendB=2;plane.blendC=0;plane.blendD=0;plane.colourMask=8;   // alpha replaced
        Material snapshot;snapshot.textured=false;snapshot.fromFrame=1;snapshot.source=0;snapshot.target=image;snapshot.frameDepth=true;
        snapshot.nearest=true;snapshot.depthTest=false;snapshot.depthWrite=false;snapshot.blend=Material::Blend::Opaque;snapshot.colourMask=7;
        Material copy;copy.textured=false;copy.fromFrame=1;copy.source=image;copy.nearest=!blur.bilinear;copy.clampU=copy.clampV=true;
        copy.alphaTest=true;copy.alphaFunc=Material::AlphaFunc::GreaterEqual;copy.depthTest=true;copy.depthWrite=false;
        copy.gsBlend=true;copy.blendA=0;copy.blendB=1;copy.blendC=0;copy.blendD=1;copy.colourMask=7;
        bozo::render::Vertex whole[6];const float xy[6][2]={{0,0},{1,0},{0,1},{1,0},{1,1},{0,1}};
        for(unsigned i=0;i<6;++i){auto &v=whole[i];v.x=2048-blur.width/2+xy[i][0]*blur.width;v.y=2048-blur.height/2+xy[i][1]*blur.height;
            v.u=xy[i][0];v.v=xy[i][1];v.rgba=0x80808080;}
        for(const auto &pass:blur.passes){
            if(!pass.copy){const auto v=vp2s::nativeDepthBlurPlaneVertices(blur,pass);renderer.draw(plane,identity,v.data(),uint32_t(v.size()));continue;}
            renderer.draw(snapshot,identity,whole,6);
            const auto v=vp2s::nativeDepthBlurCopyVertices(blur,pass);
            auto rgb=copy;rgb.alphaRef=pass.reference;renderer.draw(rgb,identity,v.data(),uint32_t(v.size()));
            auto alpha=rgb;alpha.target=image;alpha.frameDepth=true;alpha.colourMask=8;renderer.draw(alpha,identity,v.data(),uint32_t(v.size()));
        }
        if(++depthBlurDraws<=4 && std::getenv("VP2S_BOZO_TRACE_QUEUES"))
            std::fprintf(stderr,"[vp2s:depth-blur] native context=%x passes=%zu %dx%d\n",blur.context,blur.passes.size(),blur.width,blur.height);
    }
    vp2s::NativeStateSetups stateSetups;
    vp2s::NativeStateSetups &nativeStateSetups()override{return stateSetups;}
    vp2s::NativeDebugFonts fonts;
    vp2s::NativeDebugFonts &nativeDebugFonts()override{return fonts;}
    vp2s::NativeAssets assets;
    vp2s::NativeAssets &nativeAssets()override{return assets;}
    vp2s::NativeScene scene;
    vp2s::NativeScene &nativeScene()override{return scene;}
    vp2s::NativeFramePublisher publisher;
    void beginNativeFrame(const vp2s::NativeFrameTarget &target)override{flushDeferredModels();
        publisher.next(target);bozo::render::Renderer::instance().clearClip();sceneClip={};
    }
    void drawNativeBrightness(const vp2s::NativeBrightness &effect)override{flushDeferredModels();
        const auto target=scene.frameTarget();
        if(!target)throw std::runtime_error("native brightness has no active frame");
        if(!effect.enabled)return;
        bozo::render::Material material;material.textured=false;
        material.fromFrame=1;material.nearest=true;material.depthTest=false;material.depthWrite=false;
        material.blend=bozo::render::Material::Blend::Opaque;material.colourMask=7;material.levels.enabled=true;
        std::copy(effect.gain.begin(),effect.gain.end(),material.levels.gain);
        std::copy(effect.high.begin(),effect.high.end(),material.levels.high);
        std::copy(effect.low.begin(),effect.low.end(),material.levels.low);
        const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        const float xy[6][2]={{0,0},{1,0},{0,1},{1,0},{1,1},{0,1}};
        bozo::render::Vertex vertices[6];
        for(unsigned i=0;i<6;++i){
            vertices[i].x=2048-target->width/2+xy[i][0]*target->width;
            vertices[i].y=2048-target->height/2+xy[i][1]*target->height;
            vertices[i].u=xy[i][0];vertices[i].v=xy[i][1];vertices[i].rgba=0x80808080;
        }
        bozo::render::Renderer::instance().draw(material,identity,vertices,6);
    }
    vp2s::NativeTextures textures{std::filesystem::path(VP2S_BOZO_IMAGES).parent_path().parent_path()/"vp2_textures",std::filesystem::path(VP2S_BOZO_IMAGES).parent_path()/"textures.manifest"};
    vp2s::NativeTextures &nativeTextures()override{return textures;}
    vp2s::ProceduralRandom procedural;
    vp2s::ProceduralRandom &proceduralRandom()override{return procedural;}
    vp2s::bootstrap::Tables startupTables{};
    const vp2s::bootstrap::Tables &bootstrapTables() const override{return startupTables;}
    vp2s::ModuleBuffers moduleStorage;
    std::unique_ptr<vp2s::NativeServiceModules> serviceModules;
    vp2s::ModuleBuffers &moduleBuffers() override {return moduleStorage;}
    void validateNativeTransfer(uint32_t destination,size_t bytes) override {
        if(destination>=0x40000000)moduleStorage.validate(destination,bytes);
        else {
            if(bytes>UINT32_MAX)throw std::runtime_error("native sound buffer transfer is too large");
            dbc->allocatedBuffer(destination,static_cast<uint32_t>(bytes));
        }
    }
    void copyNativeTransfer(uint32_t destination,std::span<const uint8_t> data) override {
        validateNativeTransfer(destination,data.size());
        if(destination>=0x40000000)moduleStorage.copy(destination,data);
        else {auto to=dbc->allocatedBuffer(destination,static_cast<uint32_t>(data.size()));
            std::copy(data.begin(),data.end(),to.begin());}
    }
    uint32_t submitNativeCommand(uint32_t command,std::span<const uint8_t> data) override {
        return dbc->submitCommand(command,data);
    }
    vp2s::ModuleResult loadNativeModule(const std::string &path,std::span<const uint8_t>) override {
        throw std::runtime_error("unported native service module: "+path);
    }
    vp2s::ModuleResult loadNativeModuleBuffer(uint32_t handle,std::span<const uint8_t> args) override {
        return serviceModules->loadBuffer(moduleStorage.view(handle),args);
    }
    uint32_t seed=0;
    uint32_t bootSeed() const override { return seed; }
    bozo::host::FreeHost native;
    vp2s::ControllerBridge controller;
    std::unique_ptr<bozo::native::irx::CompiledLibraries> dbc;
    vp2s::NativeCommandReplies replies;
    vp2s::TimerClock sdkClock;
    vp2s::TimerClock &timerClock() override { return sdkClock; }
    vp2s::Settings preferences;
    vp2s::Settings &settings() override { return preferences; }
    explicit StrictHost(const bozo::host::FreeOptions &options) : native(options),controller(native.libraries()) {
        std::ifstream startup(std::filesystem::path(VP2S_BOZO_IMAGES)/"bootstrap.bin",std::ios::binary);
        char magic[4];uint32_t tableBytes=0;
        if(!startup.read(magic,4) || std::memcmp(magic,"VPBK",4) ||
           !startup.read(reinterpret_cast<char *>(&tableBytes),4) || tableBytes!=sizeof(startupTables) ||
           !startup.read(reinterpret_cast<char *>(&startupTables),sizeof(startupTables)))
            throw std::runtime_error("private native startup tables are missing or invalid");
        dbc=std::make_unique<bozo::native::irx::CompiledLibraries>(native.libraries(),0x80000);
        dbc->useReadyAudioCompletions();
        dbc->useNativeFrameCallbacks();
        dbc->useNativeDiscFiles();
        native.onFrame=[this](uint64_t,const R5900Context *){dbc->notifyFrameCompleted();};
        addVp2DbcImage(*dbc);
        addVp2ControllerImage(*dbc);controller.bind(*dbc);
        addVp2SoundImage(*dbc);
        addVp2CsmImage(*dbc);
        addVp2CsdImage(*dbc);
        addVp2CsiImage(*dbc);
        const auto root=std::filesystem::path(VP2S_BOZO_IMAGES).parent_path().parent_path()/"extract/iop";
        serviceModules=std::make_unique<vp2s::NativeServiceModules>(root,[this]{
            return native.libraries().find("scePadInit") && native.libraries().find("sceMcInit");},
            [this](std::string_view name,std::span<const uint8_t> args){
                if(!args.empty())throw std::runtime_error("unported native controller/DBC startup arguments");
                if(dbc->startImage(std::string(name)+";1")<=0)throw std::runtime_error("native DBC initialization failed");
                // DBC's guarded initialization and the replacement native
                // controller entry both return zero after successful setup.
                return 0;
            });
        auto &sif=native.libraries().sif;
        sif.allocateBuffer=[this](uint32_t bytes){return moduleStorage.allocate(bytes);};
        sif.freeBuffer=[this](uint32_t h){return moduleStorage.release(h);};
        sif.transfer=[this](uint32_t to,const uint8_t *data,uint32_t bytes){copyNativeTransfer(to,{data,bytes});};
        sif.resetProvider=[this]{moduleStorage.reset();serviceModules->reset();dbc->reset();controller.reset();replies.reset();};
        sif.receiveCommand=[this](uint32_t command,std::span<const uint8_t> data,uint32_t to,std::span<const uint8_t> extra){
            return replies.submit(native.libraries(),command,data,to,extra);
        };
        native.libraries().audioProvider=[this](uint64_t cycle){return dbc->advanceTo(cycle);};
        sif.moduleLoader=[this](const std::string &path){return loadNativeModule(path,{}).id;};
        if(const char *specified=std::getenv("VP2S_BOZO_SEED")) {
            const char *end=specified+std::char_traits<char>::length(specified);
            auto parsed=std::from_chars(specified,end,seed);
            if(parsed.ec!=std::errc{} || parsed.ptr!=end || specified==end)
                throw std::runtime_error("VP2S_BOZO_SEED must be an unsigned decimal 32-bit seed");
        } else seed=std::random_device{}();
    }
    void boundaryHash(bozo::ee::Dispatcher &d) {
        if(!std::getenv("VP2S_BOZO_BOUNDARY_HASH")) return;
        uint64_t hash=14695981039346656037ull;
        const auto *ram=d.runtime().memory.flat();
        for(uint32_t i=0;i<bozo::kFlatBytes;++i) { hash^=ram[i];hash*=1099511628211ull; }
        std::fprintf(stderr,"[vp2s:bozo] boundary RAM hash=%016llx\n",(unsigned long long)hash);
    }
    unsigned traceLibrary=0;
    // Diagnostic: VP2S_BOZO_WATCH also reports library calls that changed the
    // watched 16 bytes, and changes made by compiled code between two calls.
    uint32_t watchSeen[4]={};bool watchPrimed=false;
    void watchLibrary(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t target,const char *when) {
        static const uint32_t watch=[]{const char *v=std::getenv("VP2S_BOZO_WATCH");return v?uint32_t(std::strtoul(v,nullptr,16))&~15u:0u;}();
        if(!watch)return;
        uint32_t now[4]={};if(const auto *p=d.runtime().memory.hostPointer(watch,16))std::memcpy(now,p,16);
        if(watchPrimed && std::memcmp(now,watchSeen,16))
            std::fprintf(stderr,"[vp2s:watch] %s library %x ra=%x a0=%x a1=%x a2=%x frames=%llu: %08x %08x %08x %08x -> %08x %08x %08x %08x\n",when,target,c->r[31].uw[0],
                c->r[4].uw[0],c->r[5].uw[0],c->r[6].uw[0],(unsigned long long)native.framesEnded(),watchSeen[0],watchSeen[1],watchSeen[2],watchSeen[3],now[0],now[1],now[2],now[3]);
        std::memcpy(watchSeen,now,16);watchPrimed=true;
    }
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t target) override {
        watchLibrary(d,c,target,"before");
        struct WatchAfter{StrictHost *self;bozo::ee::Dispatcher &d;R5900Context *c;uint32_t target;
            ~WatchAfter(){self->watchLibrary(d,c,target,"during");}} watchAfter{this,d,c,target};
        if(vp2s::nativeMediaLibrary(c,&d.runtime(),target))return;
        if(vp2s::nativeBoxPairLibrary(c,&d.runtime(),target))return;
        if(vp2s::nativeSprWriterLibrary(c,&d.runtime(),target))return;
        // The shared libmpeg wrapper retains a TMB-specific reference ABI and
        // an IPU-demand flag. VP2S needs its native media provider first.
        if(target==0x1ef6f8 || target==0x1f02a0 || target==0x1f0398 || target==0x1f0438 || target==0x1f0510 || target==0x1f4590 || target==0x1f4990)
            throw std::runtime_error(std::format("native VP2S MPEG provider required at {:x} ra={:x}",target,c->r[31].uw[0]));
        if(overlayExternals.contains(target))throw std::runtime_error(std::format("native VP2S overlay library owner required at {:x}",target));
        // A generic SDK retirement cannot establish VP2S source ownership.
        // Known producers publish offline assets before retiring their transport.
        if(target==0x122bf8u)
            throw std::runtime_error(std::format("native VP2S image owner required at {:x} ra={:x} a0={:x} a1={:x} (last image enqueue ra={:x} source={:x} count={} owner={:x})",target,c->r[31].uw[0],c->r[4].uw[0],c->r[5].uw[0],
                lastImageEnqueue[0],lastImageEnqueue[1],lastImageEnqueue[2],lastImageEnqueue[3]));
        if(target==0x123220u || target==0x123550u || target==0x1235b8u || target==0x123630u || target==0x1237f0u){
            std::string detail;
            if(c->r[31].uw[0]==0x112fc0){
                // Type-2 queue entry: the 1152c0 multi-copy frame effect's packet (ctx = *(35de60)).
                auto &m=d.runtime().memory;const auto word=[&](uint32_t at){uint32_t v=0;if(const auto *p=m.hostPointer(at,4))std::memcpy(&v,p,4);return v;};
                const auto ctx=word(0x35de60),flags=word(ctx+0x3e7c);float r0,r1;const auto f0=word(ctx+0x3e80),f1=word(ctx+0x3e84);std::memcpy(&r0,&f0,4);std::memcpy(&r1,&f1,4);
                detail=std::format(" [effect ctx={:x} on/planes/final/n={:08x} r={}..{} final={:x} frame={:x} 3c70={:x} 3c74={:x} 3c77={:x} size={}x{} near:",ctx,flags,r0,r1,word(ctx+0x3e88),
                    word(ctx+0x1e0)&0xffff,word(ctx+0x3c70)&0xffff,word(ctx+0x3c74)&0xffff,(word(ctx+0x3c74)>>24)&0xff,int16_t(word(ctx+0x3c6c)),int16_t(word(ctx+0x3c6e)));
                for(unsigned i=0;i<8;++i)detail+=std::format(" {:x}",word(ctx+0x3e8c+4*i));
                detail+=" far:";for(unsigned i=0;i<8;++i)detail+=std::format(" {:x}",word(ctx+0x3ecc+4*i));
                detail+=std::format(" owner={:x}]",c->r[17].uw[0]);
            }
            if(c->r[31].uw[0]==0x112fe0){
                // Type-3 queue entry: a source-chain DMA (s1); each REF with its asset origin.
                auto &m=d.runtime().memory;const auto qword=[&](uint32_t at){uint64_t v=0;if(const auto *p=m.hostPointer(at,8))std::memcpy(&v,p,8);return v;};
                detail=" [chain";uint32_t at=c->r[17].uw[0]&0x1ffffff;
                for(unsigned k=0;k<8;++k){const auto tag=qword(at);const unsigned id=unsigned(tag>>28)&7,qwc=unsigned(tag&0xffff);const auto address=uint32_t(tag>>32)&0x1ffffff;
                    const auto origin=nativeAssets().origin(address);
                    detail+=std::format(" {}:{:x}/{:x}",id,address,qwc);
                    if(origin)detail+=std::format("(asset {}/{:x}+{:x})",origin->part,origin->disc,origin->offset);
                    // CNT's payload follows its tag (data, not tags); NEXT goes to its address.
                    if(id==7 || id==0)break;at=id==1?at+16+16*qwc:id==2?address:at+16;}
                // The queue (s2) entry being run: +0 type, +4 chain, +8 t0, +c owner (1131c0's t1).
                const auto word=[&](uint32_t a){uint32_t v=0;if(const auto *p=m.hostPointer(a,4))std::memcpy(&v,p,4);return v;};
                const auto queue=c->r[18].uw[0],entry=word(queue+4)+16*uint32_t(int16_t(word(queue+8)>>16));
                const auto owner=word(entry+0xc);
                detail+=std::format("] [entry {:x} owner={:x} class={:x} {:x}]",entry,owner,owner?word(owner):0u,owner?word(owner+4):0u);
            }
            throw std::runtime_error(std::format("native VP2S draw/data owner required at {:x} ra={:x}{}",target,c->r[31].uw[0],detail));
        }
        if(target==0x122160u || target==0x1224d0u)
            throw std::runtime_error(std::format("native VP2S display owner required at {:x} ra={:x}",target,c->r[31].uw[0]));
        if(target==0x122ab8u){
            // sceGsSetDefStoreImage builds no packet natively: the readback node's
            // request is recorded here and served from native depth at its enqueue.
            depthProbes.request(c);
            bozo::render::Renderer::instance().wantDepth();
            if(std::getenv("VP2S_BOZO_TRACE_READBACK"))
                std::fprintf(stderr,"[vp2s:readback] request packet=%x sbp=%x sbw=%x psm=%x x=%d y=%d w=%d h=%d frames=%llu\n",c->r[4].uw[0],c->r[5].uw[0],c->r[6].uw[0],c->r[7].uw[0],
                    c->r[8].sw[0],c->r[9].sw[0],c->r[10].sw[0],c->r[11].sw[0],(unsigned long long)native.framesEnded());
        }
        if(std::getenv("VP2S_BOZO_TRACE_CALLS") && traceLibrary++<160)
            std::fprintf(stderr,"[vp2s:call] pc=%x ra=%x a0=%x a1=%x\n",target,c->r[31].uw[0],c->r[4].uw[0],c->r[5].uw[0]);
        const uint32_t a0=c->r[4].uw[0],a1=c->r[5].uw[0],a2=c->r[6].uw[0];
        const uint32_t assetWorker=c->r[31].uw[0]==0x1059a8?c->r[17].uw[0]:0;
        if((target==0x132c88u || target==0x132e40u) && a2>0x2000000u)   // diagnostic: a copy/fill larger than RAM names its caller
            std::fprintf(stderr,"[vp2s:bozo] library %x size %x: ra=%x a0=%x a1=%x sp=%x s0..s3=%x %x %x %x\n",target,a2,c->r[31].uw[0],a0,a1,
                c->r[29].uw[0],c->r[16].uw[0],c->r[17].uw[0],c->r[18].uw[0],c->r[19].uw[0]);
        try{native.libraryCall(d,c,target);}catch(const std::exception &e){
            // Diagnostic: the failing library call's arguments and caller.
            std::fprintf(stderr,"[vp2s:bozo] library %x failed: ra=%x a0=%x a1=%x a2=%x sp=%x s0..s3=%x %x %x %x\n",target,c->r[31].uw[0],a0,a1,a2,
                c->r[29].uw[0],c->r[16].uw[0],c->r[17].uw[0],c->r[18].uw[0],c->r[19].uw[0]);
            throw;}
        if(target==0x12a360u){
            // The native CD library copies bytes synchronously before charging
            // the actual read duration; failed reads publish no provenance.
            if(c->r[2].uw[0]==1 && native.libraries().cd.error==0 &&
               uint64_t(native.libraries().cd.readBytes)==uint64_t(a1)*2048)
                assets.discRead(a2,a1,a0);
        }else if(target==0x132c88u || target==0x132e40u){
            try{if(target==0x132c88u)assets.copy(a0,a1,a2,assetWorker);else assets.overwrite(a0,a2);}
            catch(const std::runtime_error &e){throw std::runtime_error(std::format("{} [asset library={:x} caller={:x} destination={:x} source/fill={:x} bytes={:x}]",e.what(),target,c->r[31].uw[0],a0,a1,a2));}
        }
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *c,uint32_t address) override {
        throw std::runtime_error(std::format("unported VP2S VU0 algorithm {:x} at pc={:x}",address,c->pc));
    }
    void ioRead(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t address,uint32_t,uint64_t[2]) override {
        boundaryHash(d);
        // Diagnostic: the code addresses on the first 0x200 stack bytes (saved return addresses) name the path.
        std::string stack;
        for(uint32_t k=0;k<0x200;k+=4)if(const auto *p=d.runtime().memory.hostPointer(c->r[29].uw[0]+k,4)){uint32_t v;std::memcpy(&v,p,4);
            if(v>=0x100000 && v<0x600000 && !(v&3))stack+=std::format(" sp+{:x}={:x}",k,v);}
        throw std::runtime_error(std::format("unported device read {:x} at pc={:x} (ra {:x}, a0 {:x}, t9 {:x}, sp {:x}) stack:{}",address,c->pc,c->r[31].uw[0],c->r[4].uw[0],c->r[25].uw[0],c->r[29].uw[0],stack));
    }
    void ioWrite(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t address,uint32_t,const uint64_t[2]) override {
        if(address==0x1000b420 && std::getenv("VP2S_BOZO_TRACE_CALLS")){
            const auto &mpeg=native.libraries().mpeg;
            std::fprintf(stderr,"[vp2s:movie-probe] reset-object=%x native-active=%u mp=%x decoder=%u delivered=%u\n",c->r[4].uw[0],mpeg.active,mpeg.mp,bool(mpeg.decoder),mpeg.delivered);
        }
        boundaryHash(d);
        if(std::getenv("VP2S_BOZO_TRACE_CALLS"))
            std::fprintf(stderr,"[vp2s:device] pc=%x a0=%x a1=%x ra=%x frames=%llu\n",
                         c->pc,c->r[4].uw[0],c->r[5].uw[0],c->r[31].uw[0],
                         (unsigned long long)native.framesEnded());
        // Diagnostic: s0..s7 and the code addresses found on the first 0x200 stack
        // bytes (saved return addresses), so a virtual-call boundary names its owner.
        std::string stack;
        const uint32_t sp=c->r[29].uw[0];
        for(uint32_t k=0;k<0x200;k+=4)
            if(const auto *p=d.runtime().memory.hostPointer(sp+k,4)){uint32_t v;std::memcpy(&v,p,4);
                if(v>=0x100000 && v<0x400000 && !(v&3))stack+=std::format(" sp+{:x}={:x}",k,v);}
        throw std::runtime_error(std::format("unported device write {:x} at pc={:x} a0={:x} a1={:x} a2={:x} ra={:x} sp={:x} s0..s7={:x} {:x} {:x} {:x} {:x} {:x} {:x} {:x} stack:{}",
            address,c->pc,c->r[4].uw[0],c->r[5].uw[0],c->r[6].uw[0],c->r[31].uw[0],sp,
            c->r[16].uw[0],c->r[17].uw[0],c->r[18].uw[0],c->r[19].uw[0],c->r[20].uw[0],c->r[21].uw[0],c->r[22].uw[0],c->r[23].uw[0],stack));
    }
    bool cop0Condition(bozo::ee::Dispatcher &,R5900Context *c) override {
        throw std::runtime_error(std::format("unported device completion condition at pc={:x}",c->pc));
    }
    void fault(bozo::ee::Dispatcher &d,R5900Context *c,const std::string &what) override {
        std::fprintf(stderr,"[vp2s:bozo] fault args a0=%x a1=%x a2=%x a3=%x ra=%x\n",
                     c->r[4].uw[0],c->r[5].uw[0],c->r[6].uw[0],c->r[7].uw[0],c->r[31].uw[0]);
        native.fault(d,c,what);
    }
    void idle(bozo::ee::Dispatcher &d,uint64_t from,uint64_t to) override { native.idle(d,from,to); }
    void resuming(bozo::ee::Dispatcher &d,bozo::ee::Thread &t,bozo::ee::Frame &f,uint32_t depth) override { native.resuming(d,t,f,depth); }
    unsigned traceKernel=0;
    void kernelCallEnd(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t target,bool stop) override {
        if(std::getenv("VP2S_BOZO_TRACE_CALLS") && traceKernel++<160)
            std::fprintf(stderr,"[vp2s:kernel] pc=%x a0=%x result=%lld stop=%d\n",target,c->r[4].uw[0],(long long)c->r[2].sd[0],stop);
        native.kernelCallEnd(d,c,target,stop);
    }
    void kernelCallBegin(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t target,uint32_t kind) override {
        vp2s::validateNativeKernelCall(c,kind);native.kernelCallBegin(d,c,target,kind);
    }
    void waitCompleted(bozo::ee::Dispatcher &d,bozo::ee::Thread &t) override { native.waitCompleted(d,t); }
    void handlerCompleted(bozo::ee::Dispatcher &d,bozo::ee::Thread &t,const bozo::ee::Frame &f) override { native.handlerCompleted(d,t,f); }
};
// The native sites' handlers, in the order they are offered a site. Each declines a site that is not
// its own (nullopt) or claims it (true: the site is done; false: the original instruction runs on).
using NativeStep=std::optional<bool>(*)(uint8_t *,R5900Context *,Runtime *,uint32_t);
std::optional<bool> nativeObservedSites(uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc){
    (void)ram;
    if(pc==0x239f70 || pc==0x25aa30){
        if(pc==0x25aa30){
            const auto object=ctx->r[5].uw[0]-0x90;
            const auto read=[&](uint32_t at){uint32_t v;const auto *p=rt->memory.hostPointer(at,4);if(at<0x100000 || at>0x2000000-4 || !p)throw std::runtime_error("native mipmap metadata outside EE RAM");std::memcpy(&v,p,4);return v;};
            const auto current=read(0x35ddf8);
            throw std::runtime_error(std::format("native mipmap producer required: object={:x} type={:x} source={:x} flags={:08x} parameters={:08x}/{:08x}/{:08x} sceneMipmap={:x}",
                object,read(object),read(object+0x94),read(object+0xac),read(object+0xa0),read(object+0xa4),read(object+0xa8),read(current+0x444c)));
        }
        const auto object=ctx->r[4].uw[0];
        const auto word=[&](uint32_t at){uint32_t value;const auto *p=rt->memory.hostPointer(at,4);
            if(at<0x100000 || uint64_t(at)+4>0x2000000 || !p)throw std::runtime_error("native mesh metadata outside EE RAM");
            std::memcpy(&value,p,4);return value;};
        const auto model=word(object+0x7bc);
        auto *d=vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services);auto *owner=d?vp2s::nativeCast<vp2s::AssetOwner>(&d->host()):nullptr;
        const auto origin=owner?owner->nativeAssets().origin(model+0x60):std::nullopt;
        throw std::runtime_error(std::format("native mesh producer required: object={:x} type={:x} job={:x} model={:x} count={} list={:x} asset={:x}/{}+{:x}",
            object,word(object),word(object+0x650),model,word(model+0x34),word(object+0x77c),origin?origin->disc:0,origin?origin->part:0,origin?origin->offset:0));
    }
    if(pc==0x1131c0){
        // The last type-1 (image) enqueue, named by the image-owner boundary (122bf8).
        if(ctx->r[5].uw[0]==1){lastImageEnqueue[0]=ctx->r[31].uw[0];lastImageEnqueue[1]=ctx->r[7].uw[0];lastImageEnqueue[2]=ctx->r[8].uw[0];lastImageEnqueue[3]=ctx->r[9].uw[0];}
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES")){
            std::fprintf(stderr,"[vp2s:enqueue] ra=%x queue=%x type=%u source=%x count=%u owner=%x\n",
                         ctx->r[31].uw[0],ctx->r[4].uw[0],ctx->r[5].uw[0],ctx->r[7].uw[0],ctx->r[8].uw[0],ctx->r[9].uw[0]);
            if(ctx->r[31].uw[0]==0x447d64 && ctx->r[9].uw[0]>=0x100090){
                const auto word=[&](uint32_t at){uint32_t value;const auto *p=rt->memory.hostPointer(at,4);
                    if(at<0x100000 || at>0x2000000-4 || !p)throw std::runtime_error("resource metadata diagnostic outside EE RAM");
                    std::memcpy(&value,p,4);return value;};
                const auto object=ctx->r[9].uw[0]-0x90,buffer=word(object+0x228);
                auto *d=vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services);auto *owner=d?vp2s::nativeCast<vp2s::AssetOwner>(&d->host()):nullptr;
                const auto origin=owner?owner->nativeAssets().origin(buffer):std::nullopt;
                std::fprintf(stderr,"[vp2s:resource] object=%x type=%x buffer=%x origin=%u/%llx/%u+%x\n",object,word(object),buffer,origin?unsigned(origin->kind):999,origin?(unsigned long long)origin->disc:0,origin?origin->part:0,origin?origin->offset:0);
            }
            if(ctx->r[31].uw[0]==0x2ef3b8){
                const auto *saved=rt->memory.hostPointer(ctx->r[29].uw[0],8);uint64_t caller=0;
                if(!saved)throw std::runtime_error("UI producer return outside owned RAM");
                std::memcpy(&caller,saved,8);
                std::fprintf(stderr,"[vp2s:producer] UI caller=%x\n",static_cast<uint32_t>(caller));
                if(caller==0x2ec9e8 && ctx->r[9].uw[0]>=0x1000b0){
                    const auto layer=ctx->r[9].uw[0]-0xb0;
                    const auto word=[&](uint32_t at){uint32_t v;const auto *p=rt->memory.hostPointer(at,4);
                        if(at<0x100000 || at>0x2000000-4 || !p)throw std::runtime_error("UI metadata diagnostic outside EE RAM");
                        std::memcpy(&v,p,4);return v;};
                    std::fprintf(stderr,"[vp2s:UI] layer=%x type=%x kind=%x first=%x\n",layer,word(layer),word(layer+0x10),word(layer+8));
                    const auto current=word(0x35ddf8);
                    std::fprintf(stderr,"[vp2s:UI] object type=%x job=%x pass=%u pending=%x,%x flags=%x control=%x\n",
                        word(layer+0x20),word(layer+0xb0),unsigned(*rt->memory.hostPointer(layer+0x85,1)),
                        word(current+0x40a8),word(current+0x40ac),word(current+0x40dc),word(layer+0xc8));
                    std::array<uint32_t,16> seen{};auto element=word(layer+8);
                    for(unsigned i=0;i<seen.size() && element && element!=layer;++i){
                        if(std::find(seen.begin(),seen.begin()+i,element)!=seen.begin()+i)break;seen[i]=element;
                        const auto type=word(element);const auto *flags=rt->memory.hostPointer(element+0x39,5);
                        if(!flags)throw std::runtime_error("UI flags diagnostic outside EE RAM");
                        std::fprintf(stderr,"[vp2s:UI] element=%x type=%x build=%x draw=%x visible=%u built=%u enabled=%u\n",element,type,word(type+0x20),word(type+0x1c),flags[0],flags[2],flags[4]);
                        if(type==0x15a450){
                            const auto font=word(element+0x6c);
                            std::fprintf(stderr,"[vp2s:UI] text font=%x items=%x capacity=%u glyphs=%u text=%x control=%x\n",font,word(element+0x70),word(element+0xb4),word(element+0xb8),word(element+0xfc),word(element+0x100));
                            if(font>=0x100000 && font<0x2000000-0x30){
                                const auto pixels=word(font+0x10);
                                std::fprintf(stderr,"[vp2s:UI] font pixels=%x bytes=%u size=%ux%u border=%u\n",pixels,word(font+0x20),word(font+0x24),word(font+0x28),word(font+0x2c));
                                auto *d=vp2s::nativeCast<bozo::ee::Dispatcher>(rt->services);auto *owner=d?vp2s::nativeCast<vp2s::AssetOwner>(&d->host()):nullptr;
                                if(owner)if(auto origin=owner->nativeAssets().origin(pixels))std::fprintf(stderr,"[vp2s:UI] font origin kind=%u disc=%llu part=%u offset=%u\n",unsigned(origin->kind),(unsigned long long)origin->disc,origin->part,origin->offset);
                            }
                        }
                        element=word(element+8);
                    }
                }
            }
            if(ctx->r[31].uw[0]==0x120f50){
                const auto *saved=rt->memory.hostPointer(ctx->r[29].uw[0]+0x20,8);uint64_t caller=0;
                if(!saved)throw std::runtime_error("packet-builder return record outside RAM");
                std::memcpy(&caller,saved,8);
                std::fprintf(stderr,"[vp2s:producer] builder=%x caller=%x\n",ctx->r[17].uw[0],static_cast<uint32_t>(caller));
            }
        }
        return false; // Observe the original CPU entry; execute its scalar prologue unchanged.
    }
    return std::nullopt;
}
constexpr NativeStep nativeSteps[]={
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeLoadProgressInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeUiWindowInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeUiTextInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeModelInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeParticleDrawInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeProceduralDrawInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeProceduralInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeFrameFilterInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeFrameCopyInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeDofInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeHazeInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeSceneClipInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeMapPanelInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeFeedbackInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeParticleInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeUiSpriteInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeMediaBufferInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeScreenRectInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeCaptureFadeInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeEntryCaptureInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeSprWriterInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeScreenZoomInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeDepthBlurInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeColumnCaptureInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeDepthProbeInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeFixedImageInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeTextureUploaderInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeMenuImageInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeBattleImageInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeBattleTextureSetInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeBattleSpriteInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeNamePlateInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeTextureManagerInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeVisibilityInstruction(ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeStateSetupInstruction(ram,ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeDebugFontInstruction(ram,ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeCompositionInstruction(ram,ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeAssetInstruction(ram,ctx,rt,pc);},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{(void)ram;return vp2s::nativeWarpInstruction(ctx,rt,pc);},
    nativeObservedSites,
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{return vp2s::nativeSceneInstruction(ram,ctx,rt,pc)?std::optional(true):std::nullopt;},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{return vp2s::nativeTextureInstruction(ram,ctx,rt,pc)?std::optional(true):std::nullopt;},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{return vp2s::nativeBootSeed(ram,ctx,rt,pc)?std::optional(true):std::nullopt;},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{return vp2s::nativeBootstrapKey(ram,ctx,rt,pc)?std::optional(true):std::nullopt;},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{return vp2s::nativeRandomSeed(ram,ctx,rt,pc)?std::optional(true):std::nullopt;},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{return vp2s::nativeRandomNext(ram,ctx,rt,pc)?std::optional(true):std::nullopt;},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{if(pc==0x214c40 || pc==0x214ce8 || pc==0x40b040)return vp2s::nativeMathInstruction(ram,ctx,rt,pc);return std::nullopt;},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{return vp2s::nativeMathInstruction(ram,ctx,rt,pc)?std::optional(true):std::nullopt;},
    [](uint8_t *,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{if(vp2s::nativeWorkStackSite(pc)){vp2s::nativeWorkStack(ctx,rt,pc);return false;}return std::nullopt;},
    [](uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc)->std::optional<bool>{return vp2s::nativeFrameInput(ram,ctx,rt,pc)?std::optional(true):std::nullopt;},
};
// The handlers whose decision on a site they list depends on more than the site and its instruction word:
// state (ScreenRect, ScreenZoom, ColumnCapture, BattleSprite, StateSetup: an active build; ProceduralDraw:
// 247ec0 observed, then passed on), or words elsewhere (UiWindow, UiSprite, SprWriter, BattleImage,
// BattleTextureSet: which overlay is loaded). They are offered every site the steps after them own.
// Every other handler declines by the site's address and word alone (filters, overlay word checks), so
// once a (site, word) has been claimed by handler i, the others before i decline it for the whole run.
constexpr uint64_t kConditionalSteps=0x470b28022ull;
constexpr size_t kMathStep=47;
// Which conditional handlers may act on a site: their own site predicates (native_conditional_sites.h).
uint64_t conditionalStepsFor(uint32_t pc){
    using Site=bool(*)(uint32_t);
    static constexpr std::pair<unsigned,Site> sites[]={{1,vp2s::nativeUiWindowSite},{5,vp2s::nativeProceduralDrawSite},{15,vp2s::nativeUiSpriteSite},
        {17,vp2s::nativeScreenRectSite},{20,vp2s::nativeSprWriterSite},{21,vp2s::nativeScreenZoomSite},{23,vp2s::nativeColumnCaptureSite},
        {28,vp2s::nativeBattleImageSite},{29,vp2s::nativeBattleTextureSetSite},{30,vp2s::nativeBattleSpriteSite},{34,vp2s::nativeStateSetupSite}};
    uint64_t mask=0;for(const auto &[step,site]:sites)if(site(pc))mask|=uint64_t(1)<<step;
    if(mask&~kConditionalSteps)throw std::runtime_error("conditional site predicates disagree with the conditional steps");
    return mask;
}
constexpr size_t kNativeSteps=std::size(nativeSteps);
static_assert(kNativeSteps<=64);
bool dispatchNativeInstructionTimed(uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc);
// Verify mode: each site whose claimant changed, once (stderr), for the shared-site list.
void verifyMismatch(uint32_t pc,unsigned before,unsigned now){
    static std::set<uint32_t> seen;if(seen.insert(pc).second)std::fprintf(stderr,"[vp2s:site-verify] site %x claimed by step %u, then step %u\n",pc,before,now);}
// Diagnostic only (VP2S_BOZO_SITE_COUNTS=<file>): calls and host cycles per native site, written at exit,
// to see which sites and handlers the frame time goes to.
struct SiteCounts{
    const char *path=std::getenv("VP2S_BOZO_SITE_COUNTS");std::unordered_map<uint32_t,std::pair<uint64_t,uint64_t>> sites;std::unordered_map<uint32_t,uint64_t> slow;   // math sites with no direct sub-handler
    ~SiteCounts(){if(!path)return;FILE *f=std::fopen(path,"w");if(!f)return;
        std::vector<std::pair<uint32_t,std::pair<uint64_t,uint64_t>>> v(sites.begin(),sites.end());
        std::sort(v.begin(),v.end(),[](auto &a,auto &b){return a.second.second>b.second.second;});
        for(auto &[pc,s]:v)std::fprintf(f,"%x %llu %llu\n",pc,(unsigned long long)s.first,(unsigned long long)s.second);
        for(auto &[pc,n]:slow)std::fprintf(f,"slow %x %llu\n",pc,(unsigned long long)n);std::fclose(f);}
};
SiteCounts siteCounts;
bool dispatchNativeInstruction(uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc) {
    if(!siteCounts.path)return dispatchNativeInstructionTimed(ram,ctx,rt,pc);
    const auto start=__rdtsc();const bool handled=dispatchNativeInstructionTimed(ram,ctx,rt,pc);
    auto &s=siteCounts.sites[pc];++s.first;s.second+=__rdtsc()-start;return handled;
}
bool dispatchNativeInstructionTimed(uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc) {
    try {
    // The fast path: a site (keyed with its instruction word, so another overlay's code at the address
    // is another site) goes straight to the handler that claimed it, and a math site to the math
    // sub-handler that did (vp2s::nativeMathClaimer). A handler that declines a site it claimed
    // before ends the run. VP2S_BOZO_SITE_VERIFY=1 runs every site through the whole chains and
    // checks the claimant against the fast path's; VP2S_BOZO_NO_SITE_CACHE=1: whole chains, no check.
    static const bool cached=!std::getenv("VP2S_BOZO_NO_SITE_CACHE"),verify=std::getenv("VP2S_BOZO_SITE_VERIFY")!=nullptr;
    // Per site: a page of 1024 entries per 4 KB of code, made on first use; each entry holds the word it
    // was claimed with (another overlay's word at the address finds the entry empty for it).
    struct FastSite{uint32_t word=0;uint8_t step=0xff;vp2s::NativeMathSite math=nullptr;uint64_t before=0;};   // before: conditional steps to offer first
    static std::unordered_map<uint32_t,std::unique_ptr<std::array<FastSite,1024>>> pages;
    static uint32_t lastPage=~0u;static std::array<FastSite,1024> *lastEntries=nullptr;
    uint32_t word=0;if(const auto *p=rt?rt->memory.hostPointer(pc,4):nullptr)std::memcpy(&word,p,4);
    const auto entries=[&]()->std::array<FastSite,1024> &{
        if(pc>>12!=lastPage){auto &page=pages[pc>>12];if(!page)page=std::make_unique<std::array<FastSite,1024>>();lastPage=pc>>12;lastEntries=page.get();}
        return *lastEntries;};
    auto &entry=entries()[(pc>>2)&1023];
    if(cached && !verify && entry.step!=0xff && entry.word==word){
        const auto site=entry;
        if(siteCounts.path && site.step==kMathStep && !site.math)++siteCounts.slow[pc];
        // The conditional handlers before the owner still see the site (one may observe it and pass it on).
        for(uint64_t m=site.before;m;m&=m-1)
            if(auto handled=nativeSteps[std::countr_zero(m)](ram,ctx,rt,pc))return *handled;
        if(site.math){if(site.math(ctx,rt,pc))return true;}
        else if(auto handled=nativeSteps[site.step](ram,ctx,rt,pc))return *handled;
        throw std::runtime_error(std::format("native site {:x} declined by the handler that claimed it before",pc));
    }
    // Sites of the conditional handlers: the handlers before them in the conditional set first.
    static std::unordered_map<uint32_t,uint8_t> claimant;
    size_t from=0;
    if(cached && !verify)if(const auto it=claimant.find(pc);it!=claimant.end()){
        const size_t owner=it->second;
        for(uint64_t m=kConditionalSteps&((uint64_t(1)<<owner)-1);m;m&=m-1)
            if(auto handled=nativeSteps[std::countr_zero(m)](ram,ctx,rt,pc))return *handled;
        if(auto handled=nativeSteps[owner](ram,ctx,rt,pc))return *handled;
        from=owner+1;
        for(size_t i=from;i<kNativeSteps;++i)if(auto handled=nativeSteps[i](ram,ctx,rt,pc))return *handled;
        throw std::runtime_error(std::format("unported native VP2S algorithm at pc={:x}",pc));
    }
    for(size_t i=0;i<kNativeSteps;++i)if(auto handled=nativeSteps[i](ram,ctx,rt,pc)){
        if(cached){
            const FastSite site{word,uint8_t(i),i==kMathStep?vp2s::nativeMathClaimer():nullptr,conditionalStepsFor(pc)&((uint64_t(1)<<i)-1)};
            const bool known=entry.step!=0xff && entry.word==word;
            if((kConditionalSteps>>i)&1){
                if(verify && known && !((entry.before>>i)&1))verifyMismatch(pc,entry.step,i);   // offered first on the fast path: not a change
                claimant.emplace(pc,uint8_t(i));}
            else if(known && (entry.step!=site.step || entry.math!=site.math))verifyMismatch(pc,entry.step,i);
            else if(!known && (entry.step==0xff || verify))entry=site;   // a second overlay's word at a known site keeps the first
        }
        return *handled;
    }
    throw std::runtime_error(std::format("unported native VP2S algorithm at pc={:x}",pc));
    } catch(const std::runtime_error &e) {
        throw std::runtime_error(std::format("{} [native site={:x} entry={:x} ra={:x} a0={:x} a1={:x} a2={:x} a3={:x} sp={:x}]",
            e.what(),pc,ctx->pc,ctx->r[31].uw[0],ctx->r[4].uw[0],ctx->r[5].uw[0],ctx->r[6].uw[0],ctx->r[7].uw[0],ctx->r[29].uw[0]));
    }
}
// Diagnostic: VP2S_BOZO_WATCH=<hex address> reports every native site whose
// handler changed the 16 bytes at that address (native handlers are where
// host arithmetic results enter guest memory).
bool watchedNativeInstruction(uint8_t *ram,R5900Context *ctx,Runtime *rt,uint32_t pc) {
    static const uint32_t watch=[]{const char *v=std::getenv("VP2S_BOZO_WATCH");return v?uint32_t(std::strtoul(v,nullptr,16))&~15u:0u;}();
    if(!watch || !rt)return dispatchNativeInstruction(ram,ctx,rt,pc);
    uint32_t before[4]={};if(const auto *p=rt->memory.hostPointer(watch,16))std::memcpy(before,p,16);
    static uint32_t last[4]={},lastPc=0;static bool primed=false;
    if(primed && std::memcmp(before,last,16))std::fprintf(stderr,"[vp2s:watch] between pc=%x and pc=%x ra=%x %08x %08x %08x %08x -> %08x %08x %08x %08x\n",lastPc,pc,ctx?ctx->r[31].uw[0]:0u,
        last[0],last[1],last[2],last[3],before[0],before[1],before[2],before[3]);
    const bool handled=dispatchNativeInstruction(ram,ctx,rt,pc);
    uint32_t after[4]={};if(const auto *p=rt->memory.hostPointer(watch,16))std::memcpy(after,p,16);
    if(std::memcmp(before,after,16))std::fprintf(stderr,"[vp2s:watch] pc=%x ra=%x %08x %08x %08x %08x -> %08x %08x %08x %08x\n",pc,ctx?ctx->r[31].uw[0]:0u,
        before[0],before[1],before[2],before[3],after[0],after[1],after[2],after[3]);
    std::memcpy(last,after,16);lastPc=pc;primed=true;
    return handled;
}
}

#ifdef _WIN32
// Opt-in diagnostic sampling profiler (VP2S_BOZO_PROFILE=out.txt, optional
// VP2S_BOZO_PROFILE_FROM=frame): a side thread suspends the main thread every
// millisecond and records its host RIP (as an offset from the image base) and
// the guest PC/RA last published through onFrame's context. Off by default;
// it changes no game state.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <thread>
#include <atomic>
#include <map>
#include <vector>
#include <algorithm>
namespace {
std::atomic<const R5900Context *> profileContext{nullptr};std::atomic<uint64_t> profileFrame{0};
struct SamplingProfiler{
    HANDLE target=nullptr;std::atomic<bool> stop{false};std::thread worker;std::string path;uint64_t from=0;
    std::map<uint64_t,uint64_t> host;std::map<uint32_t,uint64_t> guest,caller,outside;uint64_t samples=0;
    void start(const char *out,uint64_t first){path=out;from=first;
        DuplicateHandle(GetCurrentProcess(),GetCurrentThread(),GetCurrentProcess(),&target,0,FALSE,DUPLICATE_SAME_ACCESS);
        timeBeginPeriod(1);
        worker=std::thread([this]{const auto base=uint64_t(GetModuleHandleW(nullptr));
            while(!stop){Sleep(1);if(profileFrame<from)continue;
                if(SuspendThread(target)==DWORD(-1))continue;CONTEXT ctx{};ctx.ContextFlags=CONTEXT_CONTROL;
                if(GetThreadContext(target,&ctx)){++host[ctx.Rip-base];if(const auto *c=profileContext.load()){++guest[c->pc];++caller[c->r[31].uw[0]];if(ctx.Rip-base>=0x10000000)++outside[c->pc];}++samples;}
                ResumeThread(target);}});}
    ~SamplingProfiler(){if(!worker.joinable())return;stop=true;worker.join();timeEndPeriod(1);
        FILE *f=std::fopen(path.c_str(),"w");if(!f)return;
        const auto dump=[&](const char *name,auto &map){std::vector<std::pair<uint64_t,uint64_t>> v(map.begin(),map.end());
            std::sort(v.begin(),v.end(),[](auto a,auto b){return a.second>b.second;});
            std::fprintf(f,"## %s (%llu samples)\n",name,(unsigned long long)samples);
            for(size_t i=0;i<v.size() && i<100000;++i)std::fprintf(f,"%llx %llu\n",(unsigned long long)v[i].first,(unsigned long long)v[i].second);};
        dump("host rva",host);dump("guest pc",guest);dump("guest ra",caller);dump("outside guest pc",outside);
        // Samples outside the executable, by module (the module bases move every run).
        std::map<std::string,uint64_t> modules;const auto base=uint64_t(GetModuleHandleW(nullptr));
        for(const auto &[rva,n]:host){if(rva<0x10000000)continue;HMODULE m=nullptr;char name[MAX_PATH]="?";
            if(GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,reinterpret_cast<LPCSTR>(base+rva),&m))GetModuleFileNameA(m,name,MAX_PATH);
            modules[name]+=n;}
        std::fprintf(f,"## modules\n");for(const auto &[name,n]:modules)std::fprintf(f,"%s %llu\n",name.c_str(),(unsigned long long)n);
        std::fclose(f);}
};
}
#endif
int main(int argc,char **argv) {
    bozo::host::FreeOptions options;
    options.elf=std::string(VP2S_BOZO_IMAGES)+"/main.elf";
    options.frames=1; options.stallFields=120; options.printGameOutput=true;
    std::string capturePath;
    bool windowed=false,hiddenWindow=false,framesGiven=false;
    for(int i=1;i<argc;++i) {
        const std::string arg=argv[i];
        if(arg=="--frames")framesGiven=true;
#ifdef VP2S_BOZO_WINDOW
        if(arg=="--window"){windowed=true;continue;}
        // Test runs only: the window never shows and never takes the PC's devices (with --pad-script).
        if(arg=="--hidden-window"){windowed=hiddenWindow=true;continue;}
#endif
        if(arg=="--iso" && i+1<argc)options.iso=argv[++i];
        else if(arg=="--card-dir" && i+1<argc)options.cardDir=argv[++i];
        else if(arg=="--frames" && i+1<argc)options.frames=std::stoul(argv[++i]);
        else if(arg=="--pad-script" && i+1<argc)options.padScript=argv[++i];
        else if(arg=="--load-state" && i+1<argc)options.loadState=argv[++i];
        else if(arg=="--load-state-at" && i+1<argc)options.loadStateFrame=std::stoul(argv[++i]);
        else if(arg=="--save-state" && i+1<argc)options.saveState=argv[++i];
        else if(arg=="--save-state-at" && i+1<argc)options.saveStateFrame=std::stoul(argv[++i]);
        else if(arg=="--warp" && i+1<argc){try{vp2s::armNativeWarp(vp2s::parseWarp(argv[++i],std::string(VP2S_BOZO_IMAGES)+"/../warps.tsv"));}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 2;}}
        else if(arg=="--battle" && i+1<argc){try{vp2s::armNativeBattle(vp2s::parseBattle(argv[++i]));}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 2;}}
#ifdef VP2S_BOZO_CAPTURE
        else if(arg=="--capture" && i+1<argc)capturePath=argv[++i];
#endif
        else { std::fprintf(stderr,"usage: vp2s-bozo-headless --iso FILE [--card-dir PRIVATE_DIR] [--frames N] [--pad-script FILE]\n"
                                   "       [--load-state FILE [--load-state-at FRAME]] [--save-state FILE --save-state-at FRAME]\n"
                                   "       [--warp MAP or MAP,X,Y,Z[,FACING[,ENTRY]]] (debug: the next event-script step changes map)\n"
                                   "       [--battle ID[,STAGE[,FLAGS]]] (debug: the next event-script step starts battle ID, op 0x38)\n");
#ifdef VP2S_BOZO_CAPTURE
            std::fprintf(stderr,"GPU build also accepts --capture OUTPUT.png (hidden window, no host input)\n");
#endif
#ifdef VP2S_BOZO_WINDOW
            std::fprintf(stderr,"       [--window] (play: a window, the PC's gamepads and keyboard, sound, cards in the user folder)\n");
#endif
            return 2; }
    }
    if(options.iso.empty())if(const char *iso=std::getenv("VP2S_ISO"))options.iso=iso;
    if(options.iso.empty()) {std::fprintf(stderr,"user-owned USA ISO required\n");return 2;}
#ifdef VP2S_BOZO_WINDOW
    if(windowed) {
        // Play: the game on the PC's clock, until the window closes (unless --frames).
        if(!capturePath.empty()){std::fprintf(stderr,"--capture is for headless runs (the window has F8)\n");return 2;}
        options.realTime=true;options.printGameOutput=false;options.stallFields=600;
        if(!framesGiven)options.frames=0;
        bozo::app::setIdentity({.title="Valkyrie Profile 2: Silmeria",.folder="VP2S_BozoEngine",.xdgFolder="vp2s-bozoengine",.bootElf="SLUS_214.52"});
        // The memory cards in the user folder (%APPDATA%\VP2S_BozoEngine\cards), never the
        // older runner's: test runs (--hidden-window) keep fresh cards in memory.
        if(options.cardDir.empty() && !hiddenWindow && !bozo::app::userDirectory().empty()){
            const auto cards=bozo::app::userDirectory()/"cards";std::filesystem::create_directories(cards);options.cardDir=cards.string();}
        if(!hiddenWindow){bozo::app::startLogging();bozo::app::installCrashHandler();}
    }
#endif
    options.gameTag="SLUS-21452";
    try {
        StrictHost host(options);
        bozo::ee::Dispatcher dispatcher(host);
        dispatcher.setFunctions(vp2_main_function_table,vp2_main_function_count,vp2_main_resume_table,vp2_main_resume_count);
        bozo::port::Registry registry;
        vp2s::installNativeKernelBindings(registry);
        vp2s::registerNativeMusic(registry);
        registry.nativeGraphics(&watchedNativeInstruction); // guard retired graphics; no compatibility fallback
        std::string error;
        if(!bozo::port::undeclared(registry,vp2_main_external_table,vp2_main_external_count).empty())
            throw std::runtime_error("native kernel bindings missing from generated externals");
        if(!host.native.setUp(dispatcher,vp2_main_external_table,vp2_main_external_count,&registry,error))
            throw std::runtime_error(error);
        vp2sBozoRegisterImages(dispatcher.runtime(),VP2S_BOZO_IMAGES,[&](uint32_t pc){host.overlayExternals.insert(pc);dispatcher.bindLibrary(pc);});
        vp2s::installNativeMusic(dispatcher.runtime());
        vp2s::installNativeEntryCapture(dispatcher.runtime());
        // Cheats (F2 > Cheats; VP2S_CHEAT_* for tests): applied between frames on the game's thread.
        {auto previous=host.native.onFrame;Runtime *rt=&dispatcher.runtime();
            host.native.onFrame=[previous,rt](uint64_t frame,const R5900Context *ctx){if(previous)previous(frame,ctx);vp2s::applyNativeCheats(rt);};}
        // Save states (--save-state FILE --save-state-at N, --load-state FILE [--load-state-at N]):
        // VP2S's own part. The engine saves EE memory, the dispatcher, the kernel, its libraries
        // and its host; VP2S adds the IOP modules' host, the SDK clock and the native owners that
        // keep what the game told them once (docs/BOZO_MIGRATION.md, save states). Owners with a
        // capture in flight make a save wait. VP2S_STATE_TRACE=1 names each owner as it is saved
        // or loaded (a failed load then names its owner).
        host.native.stateSections()->add({"vp2s",[&](bozo::state::Archive &a){host.dbc->serialize(a);
                auto &c=host.sdkClock;bozo::state::fields(a,c.initialized,c.running,c.epoch,c.base,c.deadlineSerial);
                static const bool trace=std::getenv("VP2S_STATE_TRACE")!=nullptr;
                const auto step=[&](const char *name){if(trace)std::fprintf(stderr,"[state] vp2s %s\n",name);};
                step("assets");host.assets.serialize(a);step("models");host.models.serialize(a);
                step("textures");host.textures.serialize(a);step("sprites");host.sprites.serialize(a,host.textures);
                const vp2s::PictureContext pictures{host.textures,host.glyphs};
                step("windows");host.windows.serialize(a,pictures);step("texts");host.uiTexts.serialize(a,pictures);step("math");vp2s::serializeMathState(a,&dispatcher.runtime());
                // Added after the first states: an older state's section ends here.
                step("battle images");if(!a.loading() || !a.atEnd())host.textures.serializeBlocks(a);
                step("battle sprites");if(!a.loading() || !a.atEnd())host.battleSprites.serialize(a);
                step("random");if(!a.loading() || !a.atEnd())host.procedural.serialize(a);
                else if(const char *seed=std::getenv("VP2S_STATE_RANDOM_SEED")){
                    // Explicit upgrade of a state older than the random field: the stream's bits as
                    // measured from the same seeded boot (no draw had advanced it before 4a45d8).
                    host.procedural.seed(uint32_t(std::stoul(seed,nullptr,16)));std::fprintf(stderr,"[state] vp2s random stream set to %06x\n",host.procedural.bits());}
                step("battle blends");if(!a.loading() || !a.atEnd())host.battleSprites.serializeBlends(a);
                step("battle uvs");if(!a.loading() || !a.atEnd())host.battleSprites.serializeUvs(a);
                step("done");},
            [&]{for(auto why:{host.dbc->unsaveable(),host.sprites.unsaveable(),host.windows.unsaveable(),host.uiTexts.unsaveable()})
                    if(!why.empty())return why;
                // Owners whose open records later publish into guest memory: a save waits for them.
                const std::pair<const char *,size_t> open[]={{"particle build",host.particles.outstanding()},{"particle update",host.particleUpdates.outstanding()},
                    {"squad",host.squads.outstanding()},{"motion update",host.motion.outstanding()},{"media buffer",host.mediaBuffers.outstanding()},
                    {"screen rectangle",host.screenRects.outstanding()},{"battle sprite flush",host.battleSprites.outstanding()},{"screen zoom",host.screenZooms.outstanding()},
                    {"state setup",host.stateSetups.outstanding()},{"debug font",host.fonts.outstanding()}};
                for(const auto &[name,count]:open)if(count)return std::format("a native {} is in flight",name);
                return std::string();}});
        // A state that fails to load ends the run: going on from boot would replay the
        // route silently (a model-section change once made every named state fail).
        host.native.onStateMessage=[](bool ok,const std::string &message){std::fprintf(stderr,"[state] %s\n",message.c_str());
            if(!ok && message.ends_with("the state was put back as it was"))throw std::runtime_error("save state failed to load: "+message);};
#ifdef _WIN32
        SamplingProfiler profiler;
        if(const char *out=std::getenv("VP2S_BOZO_PROFILE")){
            const char *from=std::getenv("VP2S_BOZO_PROFILE_FROM");
            auto previous=host.native.onFrame;
            host.native.onFrame=[previous](uint64_t frame,const R5900Context *ctx){profileFrame=frame;profileContext=ctx;if(previous)previous(frame,ctx);};
            profiler.start(out,from?std::stoull(from):0);
        }
#endif
        if(const char *at=std::getenv("VP2S_BOZO_TRACE_MODELS_AT")){
            // Diagnostic only: every model batch (texture, palette, colours, box) of one frame.
            const uint64_t traced=std::stoull(at);auto previous=host.native.onFrame;
            host.native.onFrame=[previous,traced](uint64_t frame,const R5900Context *ctx){if(previous)previous(frame,ctx);vp2s::setNativeModelTrace(frame+1==traced);};
        }
#ifdef VP2S_BOZO_CAPTURE
        if(const char *every=std::getenv("VP2S_BOZO_CAPTURE_EVERY"); every && !capturePath.empty()){
            // Diagnostic only: a numbered capture every N frames from VP2S_BOZO_CAPTURE_FROM,
            // so one long route run shows what each scripted input did.
            const uint64_t step=std::max<uint64_t>(1,std::stoull(every));
            const char *fromText=std::getenv("VP2S_BOZO_CAPTURE_FROM");const uint64_t from=fromText?std::stoull(fromText):0;
            const auto stem=capturePath.size()>4 && capturePath.ends_with(".png")?capturePath.substr(0,capturePath.size()-4):capturePath;
            auto previous=host.native.onFrame;
            host.native.onFrame=[previous,step,from,stem](uint64_t frame,const R5900Context *ctx){
                if(previous)previous(frame,ctx);
                if(frame<from || (frame-from)%step)return;
                try{vp2s::captureNativeFrame(stem+"_"+std::to_string(frame)+".png");}
                catch(const std::exception &e){std::fprintf(stderr,"[vp2s:diagnostic] capture at frame %llu unavailable: %s\n",static_cast<unsigned long long>(frame),e.what());}
            };
        }
#endif
        uint64_t frames=0;
#ifdef VP2S_BOZO_WINDOW
        if(windowed) {
            // The engine's real-time layer: the game on a thread of its own, the window showing what the
            // renderer publishes, the PC's gamepads and keyboard as the pads, the mixer's samples played.
            bozo::app::DevicePad pad;
            if(options.padScript.empty() && !hiddenWindow)host.native.libraries().pad.input=&pad;
            bozo::app::AudioDevice audio;
            // A test run plays nothing on the PC's speakers: it counts the samples and their loudest.
            uint64_t testSamples=0;int testPeak=0;
            if(hiddenWindow)host.native.onAudio=[&](uint64_t,const int16_t *samples,size_t count){testSamples+=count;
                for(size_t k=0;k<2*count;++k)testPeak=std::max(testPeak,std::abs(int(samples[k])));};
            else if(audio.open())host.native.onAudio=[&audio](uint64_t,const int16_t *samples,size_t count){audio.push(samples,count);};
            else std::fprintf(stderr,"[vp2s:window] no audio device: the game plays without sound\n");
            // Frames held to 60 a second at most; a slower frame is not held.
            bozo::app::FramePacer pacer;
            {auto previous=host.native.onFrame;host.native.onFrame=[previous,&pacer](uint64_t frame,const R5900Context *ctx){if(previous)previous(frame,ctx);pacer.frameEnd();};}
            // F2 > Cheats: applied on the game's thread by the native sites (bozo/native_cheats.cpp).
            bozo::app::ui::registerTab("Cheats",[]{
                auto &cheats=vp2s::nativeCheats();
                bool health=cheats.infiniteHealth;
                if(ImGui::Checkbox("Infinite health (party, in battle)",&health))cheats.infiniteHealth=health;
                bool weak=cheats.weakEnemies;
                if(ImGui::Checkbox("Weak enemies (1 HP, in battle)",&weak))cheats.weakEnemies=weak;
                ImGui::TextDisabled("Live party members return to full HP every frame; the records' anti-cheat checksums are rewritten as the game does.");
                int multiplier=cheats.expMultiplier;ImGui::SetNextItemWidth(160);
                if(ImGui::SliderInt("EXP multiplier",&multiplier,1,10,"x%d"))cheats.expMultiplier=multiplier;
                ImGui::Separator();
                static int wanted=99999999;const auto money=cheats.money.load();
                if(money>=0)ImGui::Text("Money (OTH): %lld",(long long)money);else ImGui::TextDisabled("Money (OTH): not loaded yet");
                ImGui::SetNextItemWidth(160);ImGui::InputInt("##money",&wanted,1000,100000);wanted=std::clamp(wanted,0,99999999);
                ImGui::SameLine();if(ImGui::Button("Set money"))cheats.setMoney=wanted;
                ImGui::Separator();
                // Items: names from tools/vp2_item_names.py (build_usa/bozo/item_names.txt, from your disc).
                static const std::vector<std::string> names=[]{std::vector<std::string> v;
                    std::ifstream in(std::filesystem::path(VP2S_BOZO_IMAGES).parent_path()/"item_names.txt");
                    for(std::string line;std::getline(in,line);)v.push_back(line);return v;}();
                if(names.empty()){ImGui::TextDisabled("Items: no item_names.txt (run tools/vp2_item_names.py)");return;}
                if(!cheats.inventoryKnown){ImGui::TextDisabled("Items: the inventory is not loaded yet");return;}
                static char search[64]="";static int selected=-1,amount=99;
                ImGui::SetNextItemWidth(240);ImGui::InputTextWithHint("##search","Search items",search,sizeof search);
                std::string needle=search;std::transform(needle.begin(),needle.end(),needle.begin(),[](unsigned char c){return char(std::tolower(c));});
                if(ImGui::BeginChild("items",ImVec2(0,220),ImGuiChildFlags_Borders)){
                    for(unsigned id=0;id<names.size() && id<vp2s::NativeCheats::kItems;++id){
                        if(names[id].empty())continue;
                        std::string lower=names[id];std::transform(lower.begin(),lower.end(),lower.begin(),[](unsigned char c){return char(std::tolower(c));});
                        if(!needle.empty() && lower.find(needle)==std::string::npos)continue;
                        const auto held=cheats.counts[id].load();
                        const std::string label=held?std::format("{}  x{}##{}",names[id],held,id):std::format("{}##{}",names[id],id);
                        if(ImGui::Selectable(label.c_str(),selected==int(id)))selected=int(id);}
                }
                ImGui::EndChild();
                ImGui::SetNextItemWidth(120);ImGui::InputInt("Amount",&amount,1,10);amount=std::clamp(amount,0,99);
                ImGui::BeginDisabled(selected<0);
                ImGui::SameLine();if(ImGui::Button("Give")){cheats.giveCount=amount;cheats.giveItem=selected;}
                ImGui::SameLine();if(ImGui::Button("Remove")){cheats.giveCount=0;cheats.giveItem=selected;}
                ImGui::EndDisabled();
                if(selected>=0)ImGui::TextDisabled("Selected: %s (id %d), held %d",names[selected].c_str(),selected,int(cheats.counts[selected].load()));
            });
            // F2 > Teleport: the debug teleport (--warp) and battle launcher (--battle) from the panel. A request fires at
            // the next event-script step of any running interpreter (the field runs one; the title screen does not).
            bozo::app::ui::registerTab("Teleport",[]{
                static const std::vector<vp2s::WarpRow> rows=vp2s::loadWarpTable(std::string(VP2S_BOZO_IMAGES)+"/../warps.tsv");
                static std::string last;
                if(rows.empty()){ImGui::TextDisabled("No warps.tsv (run tools/vp2_warps.py)");}
                else{
                    static char search[16]="";static int selectedMap=-1,selectedRow=-1;
                    ImGui::SetNextItemWidth(120);ImGui::InputTextWithHint("##map","Map id",search,sizeof search,ImGuiInputTextFlags_CharsDecimal);
                    ImGui::SameLine();ImGui::TextDisabled("%zu entry points, ids only", rows.size());
                    if(ImGui::BeginChild("maps",ImVec2(140,260),ImGuiChildFlags_Borders)){
                        int previous=-1;
                        for(const auto &r:rows){if(r.map==previous)continue;previous=r.map;
                            const auto label=std::to_string(r.map);if(search[0] && label.find(search)!=0)continue;
                            if(ImGui::Selectable(std::format("Map {}##m{}",r.map,r.map).c_str(),selectedMap==r.map)){selectedMap=r.map;selectedRow=-1;}}
                    }
                    ImGui::EndChild();ImGui::SameLine();
                    if(ImGui::BeginChild("entries",ImVec2(0,260),ImGuiChildFlags_Borders)){
                        if(selectedMap<0)ImGui::TextDisabled("Pick a map");
                        for(size_t i=0;i<rows.size();++i){const auto &r=rows[i];if(r.map!=selectedMap)continue;
                            const auto label=std::format("entry {}  ({}, {}, {})  facing {}  used {}x##r{}",r.entry,r.x,r.y,r.z,r.facing,r.uses,i);
                            if(ImGui::Selectable(label.c_str(),selectedRow==int(i)))selectedRow=int(i);}
                    }
                    ImGui::EndChild();
                    ImGui::BeginDisabled(selectedRow<0);
                    if(ImGui::Button("Warp")){const auto &r=rows[selectedRow];
                        vp2s::armNativeWarp({r.map,r.entry,r.x,r.y,r.z,-1,r.facing});last=std::format("Warp to map {} entry {} armed",r.map,r.entry);}
                    ImGui::EndDisabled();
                    if(vp2s::nativeWarpPending()){ImGui::SameLine();ImGui::TextColored(ImVec4(1,0.8f,0.3f,1),"waiting for the next script step...");}
                }
                ImGui::Separator();
                static char battle[16]="0x18";static int stage=0;
                ImGui::SetNextItemWidth(100);ImGui::InputText("Battle id",battle,sizeof battle);
                ImGui::SameLine();ImGui::SetNextItemWidth(100);ImGui::InputInt("Stage",&stage);stage=std::clamp(stage,0,0x7fff);
                ImGui::SameLine();
                if(ImGui::Button("Start battle")){
                    try{auto request=vp2s::parseBattle(std::string(battle)+","+std::to_string(stage));vp2s::armNativeBattle(request);
                        last=std::format("Battle {:#x} stage {} armed",request.battle,request.stage);}
                    catch(const std::exception &e){last=e.what();}}
                if(vp2s::nativeBattlePending()){ImGui::SameLine();ImGui::TextColored(ImVec4(1,0.8f,0.3f,1),"waiting...");}
                ImGui::TextDisabled("Stage 0 is the field's default. Requests fire at the next event-script step (not on the title screen).");
                if(!last.empty())ImGui::TextUnformatted(last.c_str());
            });
            bozo::app::WindowOptions window;
            window.title="Valkyrie Profile 2: Silmeria";window.aspect=4.0f/3.0f;window.stats=std::getenv("VP2S_WINDOW_STATS")!=nullptr;
            window.hidden=hiddenWindow;window.captureScene=hiddenWindow;
            if(const char *dir=std::getenv("VP2S_WINDOW_CAPTURE_DIR"))window.captureDir=dir;
            if(const char *list=std::getenv("VP2S_WINDOW_CAPTURE_FRAMES")){std::stringstream in(list);for(std::string item;std::getline(in,item,',');)if(!item.empty())window.captureFrames.push_back(std::stoull(item));}
            std::string failure;
            bozo::app::Game game;
            game.run=[&]{try{frames=host.native.run();return 0;}catch(const std::exception &e){failure=e.what();return 1;}};
            game.stop=[&]{host.native.requestStop();};
            game.framesEnded=[&]{return host.native.framesEnded();};
            game.saveState=[&](int slot){host.native.requestSave(bozo::app::stateSlotPath(slot));};
            game.loadState=[&](int slot){host.native.requestLoad(bozo::app::stateSlotPath(slot));};
            // A failed load is reported, not fatal, while playing.
            host.native.onStateMessage=[](bool,const std::string &message){std::fprintf(stderr,"[state] %s\n",message.c_str());bozo::app::notify(message);};
            const int code=bozo::app::runWindow(window,game,hiddenWindow?nullptr:&pad);
            audio.close();
            if(hiddenWindow)std::fprintf(stderr,"[vp2s:window] sound: %llu stereo frames at 48 kHz (%.1f s), peak %d\n",
                                         (unsigned long long)testSamples,testSamples/48000.0,testPeak);
            const auto &stats=host.native.stats();
            std::fprintf(stderr,"[vp2s:bozo] frames=%llu libraries=%llu end=%s\n",(unsigned long long)frames,(unsigned long long)stats.libraryCalls,
                         failure.empty()?host.native.endReason().c_str():"native boundary");
            if(!failure.empty()){
                std::fprintf(stderr,"[vp2s:bozo] native boundary: %s\n",failure.c_str());
                if(!hiddenWindow)bozo::app::showMessage("The game reached code that is not native yet, so it stopped:\n\n"+failure,true);
                return 1;
            }
            return code==3?3:0;
        }
#endif
        try{frames=host.native.run();}catch(...){
            // Diagnostic only: EE RAM at the rejecting boundary, for comparison with a PCSX2 state's eeMemory.bin.
            if(const char *ramPath=std::getenv("VP2S_BOZO_BOUNDARY_RAM")){
                if(FILE *out=std::fopen(ramPath,"wb")){std::fwrite(dispatcher.runtime().memory.flat(),1,0x2000000,out);std::fclose(out);
                    std::fprintf(stderr,"[vp2s:diagnostic] EE RAM at the boundary written to %s\n",ramPath);}
            }
#ifdef VP2S_BOZO_CAPTURE
            if(!capturePath.empty() && std::getenv("VP2S_BOZO_CAPTURE_BOUNDARY")){
                // Explicit diagnostic only: this is an incomplete frame, never
                // successful boot evidence. The original boundary still fails.
                try{
                    host.flushDeferredModels();bozo::render::Renderer::instance().endFrame();vp2s::captureNativeFrame(capturePath);
                    std::fprintf(stderr,"[vp2s:diagnostic] partial frame at rejecting boundary captured; run still failed\n");
                }catch(const std::exception &e){std::fprintf(stderr,"[vp2s:diagnostic] boundary capture unavailable: %s\n",e.what());}
            }
#endif
            throw;
        }
        // Diagnostic only: EE RAM at the frame limit too, for state comparisons at a chosen frame.
        if(const char *ramPath=std::getenv("VP2S_BOZO_END_RAM")){
            if(FILE *out=std::fopen(ramPath,"wb")){std::fwrite(dispatcher.runtime().memory.flat(),1,0x2000000,out);std::fclose(out);
                std::fprintf(stderr,"[vp2s:diagnostic] EE RAM at the frame limit written to %s\n",ramPath);}
        }
#ifdef VP2S_BOZO_CAPTURE
        if(!capturePath.empty())vp2s::captureNativeFrame(capturePath);
#endif
        if(std::getenv("VP2S_BOZO_TRACE_QUEUES")) {
            // Only the audited graphics-work queue header and descriptor metadata.
            // Do not dump payloads, arbitrary CPU pointers, or bootstrap material.
            auto &memory=dispatcher.runtime().memory;
            const auto word=[&](uint32_t address) {
                uint32_t value=0;
                const auto *p=memory.hostPointer(address,sizeof(value));
                if(!p)throw std::runtime_error("queue diagnostic address outside native RAM");
                std::memcpy(&value,p,sizeof(value));return value;
            };
            const auto scene=word(0x35ddf8);
            if(scene>=0x100000 && scene<=0x2000000-0x40e0){
                std::fprintf(stderr,"[vp2s:scene-wait] scene=%x flags=%08x control=%x pending=%x/%x/%x\n",scene,word(scene+0x40dc),word(scene+0x40d0),word(scene+0x40a8),word(scene+0x40ac),word(scene+0x40b0));
                for(unsigned layer=0;layer<3;++layer){const auto object=word(scene+0x40a8+layer*4);
                    if(object>=0x100000 && object<=0x2000000-0x100){const auto type=word(object);
                        if(type>=0x100000 && type<=0x2000000-0x40)std::fprintf(stderr,"[vp2s:scene-wait] layer=%u object=%x class=%x method18=%x\n",layer,object,type,word(type+0x18));}
                }
            }
            for(uint32_t global:{0x35ddc8u,0x35ddd0u,0x35dde0u}){const auto queue=word(global);
                if(queue>=0x100000 && queue<=0x2000000-0x1014){const auto job=word(queue+0x1000),head=word(queue+0x1008),tail=word(queue+0x1004);
                    std::fprintf(stderr,"[vp2s:job-queue] global=%x queue=%x active=%x head=%u tail=%u controls=%08x\n",global,queue,job,head,tail,word(queue+0x1010));
                    if(job>=0x100000 && job<=0x2000000-0x10){const auto type=word(job);
                        if(type>=0x100000 && type<=0x2000000-0x10)std::fprintf(stderr,"[vp2s:job-queue] active class=%x begin=%x complete=%x\n",type,word(type+8),word(type+0xc));}
                    if(head<1024 && head!=tail){const auto pending=word(queue+head*4);if(pending>=0x100000 && pending<=0x2000000-0x10)std::fprintf(stderr,"[vp2s:job-queue] next=%x class=%x\n",pending,word(pending));}
                }
            }
            const uint32_t owner=word(0x35dd78);
            if(owner>=0x100000 && owner<=0x1ffffe8) {
                const uint32_t slots=word(owner+4), indices=word(owner+8), flags=word(owner+12);
                const unsigned capacity=indices&0xffff, active=indices>>16, next=flags&0xffff;
                std::fprintf(stderr,"[vp2s:queue] owner=%x slots=%x capacity=%u active=%u next=%u flags=%08x tail=%08x\n",
                             owner,slots,capacity,active,next,flags,word(owner+16));
                if(capacity<=1024 && active<capacity && next<capacity && slots>=0x100000 && slots<=0x2000000-capacity*16)
                    for(unsigned i=active,n=0;i!=next && n<16;i=(i+1)%capacity,++n)
                        std::fprintf(stderr,"[vp2s:queue] slot=%u type=%u source=%x count=%u\n",i,
                                     word(slots+i*16)&0xffff,word(slots+i*16+4),word(slots+i*16+8));
            }
        }
        const auto &stats=host.native.stats();
        std::fprintf(stderr,"[vp2s:bozo] frames=%llu libraries=%llu end=%s\n",
                     (unsigned long long)frames,(unsigned long long)stats.libraryCalls,host.native.endReason().c_str());
        return frames>=options.frames?0:1;
    } catch(const std::exception &e) {
        std::fprintf(stderr,"[vp2s:bozo] native boundary: %s\n",e.what());return 1;
    }
}

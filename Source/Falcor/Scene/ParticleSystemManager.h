#pragma once
#include "Falcor.h"

using namespace Falcor;

class Falcor::Scene;

class dlldecl ParticleSystemManager
{
public:
    ParticleSystemManager() {};
    enum class ParticleShadingModel
    {
        ConstColor = 0,
        ColorInterp = 1,
        Textured = 2,
        Count
    };

    struct GuiData
    {
        int32_t mSystemIndex = -1;
        uint32_t mPixelShaderIndex = 0;
        bool mSortSystem = false;
        int32_t mMaxParticles = 4096;
        int32_t mMaxEmitPerFrame = 512;
        Gui::DropdownList mTexDropdown;
    } mGuiData;

    struct ParticleMaterialDesc
    {
        ParticleMaterialDesc() {};
        ParticleMaterialDesc(float4 color) { type = ParticleShadingModel::ConstColor; colorData.color1 = color; }
        ParticleMaterialDesc(ColorInterpPsPerFrame data) { type = ParticleShadingModel::ColorInterp; colorData = data; }
        ParticleMaterialDesc(uint32_t newTexIndex, ColorInterpPsPerFrame data)
        {
            type = ParticleShadingModel::Textured;
            texIndex = newTexIndex;
            colorData = data;
        }

        ParticleShadingModel type;
        ColorInterpPsPerFrame colorData;
        uint32_t texIndex;
    };

    void Init();

    std::vector<ParticleMaterialDesc> mPsData;
    std::vector<Texture::SharedPtr> mpTextures;

    void addInitialParticleSystem(std::shared_ptr<Scene> pScene);
    void createParticleSystemIO(int32_t maxParticles, int32_t maxEmitPerFrame, float fixedInterval, uint32_t maxRenderFrames,
        uint32_t shadingModel, bool shouldSort, ParticleSystem::EmitterData* pEmitterData, ParticleMaterialDesc* pMaterialDesc, const std::string textureFile,
        std::shared_ptr<Falcor::Scene> pScene);
    bool createSystemGui(Gui::Widgets& widget, std::shared_ptr<Falcor::Scene> pScene);
    bool editPropertiesGui(Gui::Widgets& widget, std::shared_ptr<Falcor::Scene> pScene);
    bool updateColorInterpolation(Gui::Widgets& widget, std::shared_ptr<Falcor::Scene> pScene);
};

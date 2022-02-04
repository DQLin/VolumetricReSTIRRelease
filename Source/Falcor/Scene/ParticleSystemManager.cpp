#include "ParticleSystemManager.h"
#include "Scene.h"
#include "stdafx.h"

const Gui::DropdownList kPixelShaders
{
    {0, "ConstColor"},
    {1, "ColorInterp"},
    {2, "Textured"}
};
const char* kConstColorPs = "Scene/ParticleSystem/ParticleConstColor.ps.slang";
const char* kColorInterpPs = "Scene/ParticleSystem/ParticleInterpColor.ps.slang";
const char* kTexturedPs = "Scene/ParticleSystem/ParticleTexture.ps.slang";
const std::string kDefaultTexture = "smoke-puff.png";


void ParticleSystemManager::Init()
{
    mpTextures.push_back(Texture::createFromFile(kDefaultTexture, true, false));
    mGuiData.mTexDropdown.push_back({ 0, kDefaultTexture });
}

void ParticleSystemManager::addInitialParticleSystem(std::shared_ptr<Scene>  pScene)
{
    Material::SharedPtr psMaterial = Material::create("whatever");
    ParticleSystem::SharedPtr pSys;
    pSys = ParticleSystem::create(mGuiData.mMaxParticles,
        mGuiData.mMaxEmitPerFrame, kTexturedPs, ParticleSystem::kDefaultSimulateShader, mGuiData.mSortSystem);
    ColorInterpPsPerFrame perFrame;
    perFrame.color1 = float4(1.f, 1.f, 1.f, 1.f);
    perFrame.colorT1 = pSys->getParticleDuration();
    perFrame.color2 = float4(1.f, 1.f, 1.f, 0.1f);
    perFrame.colorT2 = 0.f;
    mPsData.push_back(ParticleMaterialDesc(0, perFrame));
    //pSys->getDrawVars()->setTexture("gTex", mpTextures[0]); //(enable for rasterization)
    psMaterial->setBaseColorTexture(mpTextures[0]);
    psMaterial->setShadingModel(2);
    pSys->getDrawVars()->getParameterBlock("PsPerFrame")->
        setBlob(&mPsData[mPsData.size() - 1].colorData, 0, sizeof(ColorInterpPsPerFrame));
    pSys->setScale(0.5f, 0.f);
    psMaterial->setBaseColor(mPsData[mPsData.size() - 1].colorData.color1);
    psMaterial->setSpecularParams(mPsData[mPsData.size() - 1].colorData.color2);
    psMaterial->setIndexOfRefraction(mPsData[mPsData.size() - 1].colorData.colorT1);
    psMaterial->setEmissiveFactor(mPsData[mPsData.size() - 1].colorData.colorT2);
    pScene->addParticleSystem(pSys, psMaterial);
}

void ParticleSystemManager::createParticleSystemIO(int32_t maxParticles, int32_t maxEmitPerFrame, float fixedInterval, uint32_t maxRenderFrames,
    uint32_t shadingModel, bool shouldSort,
    ParticleSystem::EmitterData* pEmitterData, ParticleMaterialDesc* pMaterialDesc, const std::string textureFile, std::shared_ptr<Scene> pScene)
{
    Material::SharedPtr psMaterial = Material::create("whatever");
    ParticleSystem::SharedPtr pSys;
    switch ((ParticleShadingModel)shadingModel)
    {
    case ParticleShadingModel::ConstColor:
    {
        pSys = ParticleSystem::create(maxParticles,
            maxEmitPerFrame, kConstColorPs, ParticleSystem::kDefaultSimulateShader, shouldSort);
        if (pMaterialDesc) mPsData.push_back(*pMaterialDesc);
        else mPsData.push_back(float4(0.f, 0.f, 0.f, 1.f));
        psMaterial->setShadingModel(0);
        pSys->getDrawVars()->getParameterBlock("PsPerFrame")->
            setBlob(&mPsData[mPsData.size() - 1].colorData.color1, 0, sizeof(float4));
        break;
    }
    case ParticleShadingModel::ColorInterp:
    {
        pSys = ParticleSystem::create(maxParticles,
            maxEmitPerFrame, kColorInterpPs, ParticleSystem::kDefaultSimulateShader, shouldSort);
        if (pMaterialDesc) mPsData.push_back(*pMaterialDesc);
        else
        {
            ColorInterpPsPerFrame perFrame;
            perFrame.color1 = float4(1.f, 0.f, 0.f, 1.f);
            perFrame.colorT1 = pSys->getParticleDuration();
            perFrame.color2 = float4(0.f, 0.f, 1.f, 1.f);
            perFrame.colorT2 = 0.f;
            mPsData.push_back(perFrame);
        }
        psMaterial->setShadingModel(1);
        pSys->getDrawVars()->getParameterBlock("PsPerFrame")->
            setBlob(&mPsData[mPsData.size() - 1].colorData, 0, sizeof(ColorInterpPsPerFrame));
        break;
    }
    case ParticleShadingModel::Textured:
    {
        pSys = ParticleSystem::create(maxParticles,
            maxEmitPerFrame, kTexturedPs, ParticleSystem::kDefaultSimulateShader, shouldSort);
        if (pMaterialDesc) mPsData.push_back(*pMaterialDesc);
        else
        {
            ColorInterpPsPerFrame perFrame;
            perFrame.color1 = float4(1.f, 1.f, 1.f, 1.f);
            perFrame.colorT1 = pSys->getParticleDuration();
            perFrame.color2 = float4(1.f, 1.f, 1.f, 0.1f);
            perFrame.colorT2 = 0.f;
            mPsData.push_back(ParticleMaterialDesc(0, perFrame));
        }

        if (textureFile != "")
        {
            mpTextures.push_back(Texture::createFromFile(textureFile, true, false));
            mPsData.back().texIndex = (uint32_t)mpTextures.size() - 1;
            mGuiData.mTexDropdown.push_back({ mPsData.back().texIndex, textureFile });
            psMaterial->setBaseColorTexture(mpTextures.back());
        }
        else
        {
            mPsData.back().texIndex = 0;
            psMaterial->setBaseColorTexture(mpTextures[0]);
        }
        //pSys->getDrawVars()->setTexture("gTex", mpTextures[0]); //(enable for rasterization)
        psMaterial->setShadingModel(2);
        pSys->getDrawVars()->getParameterBlock("PsPerFrame")->
            setBlob(&mPsData[mPsData.size() - 1].colorData, 0, sizeof(ColorInterpPsPerFrame));
        break;
    }
    default:
    {
        should_not_get_here();
    }
    }

    if (pEmitterData) pSys->mEmitter = *pEmitterData;

    if (fixedInterval > 0)
    {
        pSys->mUseFixedInterval = true;
        pSys->mFixedInterval = fixedInterval;
        pSys->maxRenderFrames = maxRenderFrames;
    }

    psMaterial->setBaseColor(mPsData[mPsData.size() - 1].colorData.color1);
    psMaterial->setSpecularParams(mPsData[mPsData.size() - 1].colorData.color2);
    psMaterial->setIndexOfRefraction(mPsData[mPsData.size() - 1].colorData.colorT1);
    psMaterial->setEmissiveFactor(mPsData[mPsData.size() - 1].colorData.colorT2);

    pScene->addParticleSystem(pSys, psMaterial);
    pScene->mNewParticleSystemAdded = true;
}

bool ParticleSystemManager::createSystemGui(Gui::Widgets& w, std::shared_ptr<Scene> pScene)
{
    bool newSystemCreated = false;
    if (Gui::Group(w, "Create System", true).open())
    {
        w.var("Max Particles", mGuiData.mMaxParticles, 0);
        w.var("Max Emit Per Frame", mGuiData.mMaxEmitPerFrame, 0);
        w.checkbox("Sorted", mGuiData.mSortSystem);
        w.dropdown("PixelShader", kPixelShaders, mGuiData.mPixelShaderIndex);
        if (w.button("Create"))
        {
            newSystemCreated = true;
            createParticleSystemIO(mGuiData.mMaxParticles, mGuiData.mMaxEmitPerFrame, -1.f, 1000, mGuiData.mPixelShaderIndex, mGuiData.mSortSystem, nullptr, nullptr, "", pScene);
            //pSceneBuilder->addCamera(mpCamera);
            //pScene = pSceneBuilder->getScene();
            //pScene->setCameraController(Scene::CameraControllerType::Orbiter);
        }
    }

    if (w.button("Delete All"))
    {
        newSystemCreated = true;
        pScene->deleteAllParticleSystems();
        mGuiData.mSystemIndex = -1;
        mPsData.clear();
    }

    return newSystemCreated;
}

bool ParticleSystemManager::editPropertiesGui(Gui::Widgets& w, std::shared_ptr<Scene> pScene)
{
    bool dirty = false;
    dirty |= w.var("System index", mGuiData.mSystemIndex, 0, ((int32_t)pScene->getParticleSystemCount()) - 1);
    w.separator();
    //If there are no systems yet, don't let user touch properties
    if (mGuiData.mSystemIndex < 0)
        return dirty;    

    if (Gui::Group(w, "Common Properties", true).open())
    {
        pScene->getParticleSystem(mGuiData.mSystemIndex)->renderUi(w);
    }

    if (Gui::Group(w, "Pixel Shader Properties", true).open())
    {
        switch ((ParticleShadingModel)mPsData[mGuiData.mSystemIndex].type)
        {
        case ParticleShadingModel::ConstColor:
        {
            if (w.rgbaColor("Color", mPsData[mGuiData.mSystemIndex].colorData.color1))
            {
                dirty = true;
                uint32_t materialID = pScene->getParticleSystemDesc(mGuiData.mSystemIndex).materialID;
                pScene->getMaterial(materialID)->setBaseColor(mPsData[mGuiData.mSystemIndex].colorData.color1);
                pScene->getParticleSystem(mGuiData.mSystemIndex)->getDrawVars()->getParameterBlock("PsPerFrame")->setBlob(&mPsData[mGuiData.mSystemIndex].colorData.color1, 0, sizeof(float4));
            }
            break;
        }
        case ParticleShadingModel::ColorInterp:
        {
            dirty |= updateColorInterpolation(w, pScene);
            break;
        }
        case ParticleShadingModel::Textured:
        {
            if (w.button("Add Texture"))
            {
                std::string filename;
                FileDialogFilterVec filters = { {"bmp"}, {"jpg"}, {"dds"}, {"png"}, {"tiff"}, {"tif"}, {"tga"} };
                openFileDialog(filters, filename);
                mpTextures.push_back(Texture::createFromFile(filename, true, false));
                mGuiData.mTexDropdown.push_back({ (uint32_t)mGuiData.mTexDropdown.size(), filename });
            }

            uint32_t texIndex = mPsData[mGuiData.mSystemIndex].texIndex;
            if (w.dropdown("Texture", mGuiData.mTexDropdown, texIndex))
            {
                dirty = true;
                uint32_t materialID = pScene->getParticleSystemDesc(mGuiData.mSystemIndex).materialID;
                pScene->getMaterial(materialID)->setBaseColorTexture(mpTextures[texIndex]);
                pScene->getParticleSystem(mGuiData.mSystemIndex)->getDrawVars()->setTexture("gTex", mpTextures[texIndex]);
                mPsData[mGuiData.mSystemIndex].texIndex = texIndex;
            }

            dirty |= updateColorInterpolation(w, pScene);
            break;
        }
        default:
            should_not_get_here();
        }
    }
    return dirty;
}

bool ParticleSystemManager::updateColorInterpolation(Gui::Widgets& w, std::shared_ptr<Scene> pScene)
{
    bool dirty = w.rgbaColor("Color1", mPsData[mGuiData.mSystemIndex].colorData.color1);
    dirty |= w.var("Color T1", mPsData[mGuiData.mSystemIndex].colorData.colorT1);
    dirty |= w.rgbaColor("Color2", mPsData[mGuiData.mSystemIndex].colorData.color2);
    dirty |= w.var("Color T2", mPsData[mGuiData.mSystemIndex].colorData.colorT2);

    if (dirty)
    {
        pScene->getParticleSystem(mGuiData.mSystemIndex)->getDrawVars()->getParameterBlock("PsPerFrame")->setBlob(&mPsData[mGuiData.mSystemIndex].colorData, 0, sizeof(ColorInterpPsPerFrame));
        uint32_t materialID = pScene->getParticleSystemDesc(mGuiData.mSystemIndex).materialID;
        pScene->getMaterial(materialID)->setBaseColor(mPsData[mGuiData.mSystemIndex].colorData.color1);
        pScene->getMaterial(materialID)->setSpecularParams(mPsData[mGuiData.mSystemIndex].colorData.color2);
        pScene->getMaterial(materialID)->setIndexOfRefraction(mPsData[mGuiData.mSystemIndex].colorData.colorT1);
        pScene->getMaterial(materialID)->setEmissiveFactor(mPsData[mGuiData.mSystemIndex].colorData.colorT2);
    }
    return dirty;
}

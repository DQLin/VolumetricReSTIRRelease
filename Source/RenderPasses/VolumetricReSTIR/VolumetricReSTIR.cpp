/***************************************************************************
 # Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
#include "VolumetricReSTIR.h"
#include "RenderGraph/RenderPassHelpers.h"
#include "Utils.h"
#include <random>

namespace
{
    const std::string kShaderDirectory = "RenderPasses/VolumetricReSTIR/";
    const std::string kAccumulatedColorOutput = "accumulated_color";
    const std::string kMotionVec = "mvec";

    const Falcor::ChannelList kOutputChannels =
    {
        { kAccumulatedColorOutput,     "gOutputFrame",    "accumulated output color (linear)", true /* optional */      },
        { kMotionVec,     "gMotionVec",    "motion vector", true /* optional */, ResourceFormat::RG32Float      }
    };

    const Gui::DropdownList kEmissiveSamplerList =
    {
        { (uint32_t)EmissiveLightSamplerType::Uniform, "Uniform" },
        { (uint32_t)EmissiveLightSamplerType::LightBVH, "LightBVH" },
        { (uint32_t)EmissiveLightSamplerType::Power, "Power" }
    };

};


// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("VolumetricReSTIR", "Render Pass Template", VolumetricReSTIR::create);
}

VolumetricReSTIR::SharedPtr VolumetricReSTIR::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new VolumetricReSTIR(dict));
    return pPass;
}

VolumetricReSTIR::VolumetricReSTIR(const Dictionary& dict)
{
    mDefaultDefines.add("SAMPLE_GENERATOR_TYPE", "SAMPLE_GENERATOR_UNIFORM");

    mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);

    Sampler::Desc samplerDesc;
    samplerDesc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Linear);
    samplerDesc.setBorderColor(float4(0.f));
    samplerDesc.setAddressingMode(Sampler::AddressMode::Border, Sampler::AddressMode::Border, Sampler::AddressMode::Border);
    mpSampler = Sampler::create(samplerDesc);

    samplerDesc.setFilterMode(Sampler::Filter::Point, Sampler::Filter::Point, Sampler::Filter::Point);
    mpPointSampler = Sampler::create(samplerDesc);

	if (dict.size() > 0) hasExternalDict = true;

	serializePass<true>(dict);

    if (mEmissiveSamplerTypeId == 0)
    {
        mEmissiveSamplerType = EmissiveLightSamplerType::Uniform;
    }
    else if (mEmissiveSamplerTypeId == 1)
    {
        mEmissiveSamplerType = EmissiveLightSamplerType::LightBVH;
    }
    else
    {
        mEmissiveSamplerType = EmissiveLightSamplerType::Power;
    }

    // TODO: allow overriding these options

    if (mParams.mUseSurfaceScene)
    {
        mDefaultDefines.add("SURFACE_SCENE");
        mDefaultDefines.add("VBUFFERDECLARE", "VBufferItem vItem,");
        mDefaultDefines.add("VBUFFERITEM", "vItem,");
    }
    else
    {
        mDefaultDefines.add("VBUFFERDECLARE", "");
        mDefaultDefines.add("VBUFFERITEM", "");
    }

    if (mParams.mVertexReuse)
    {
        mDefaultDefines.add("VERTEX_REUSE");
        mDefaultDefines.add("REUSETYPE", "inout");
    }
    else
    {
        mDefaultDefines.add("REUSETYPE", "");
    }

    mDefaultDefines.add("_EMISSIVE_LIGHT_SAMPLER_TYPE", "0"); // uniform
    mDefaultDefines.add("MAX_BOUNCES", std::to_string(mParams.mMaxBounces));

    mLastMaxBounces = mParams.mMaxBounces;

    mpTraceRaysPass = createSimpleComputePass(kShaderDirectory + "TraceRays.cs.slang", "main", mDefaultDefines);
    mSpatialReusePass = createSimpleComputePass(kShaderDirectory + "SpatialReuse.cs.slang", "main", mDefaultDefines);
    mTemporalReusePass = createSimpleComputePass(kShaderDirectory + "TemporalReuse.cs.slang", "main", mDefaultDefines);
    mGenerateFeaturePass = createSimpleComputePass(kShaderDirectory + "GenerateFeatures.cs.slang", "main", mDefaultDefines);
    mCopyReservoirPass = createSimpleComputePass(kShaderDirectory + "CopyReservoirs.cs.slang", "main", mDefaultDefines);
    mFinalShadingPass = createSimpleComputePass(kShaderDirectory + "FinalShading.cs.slang", "main", mDefaultDefines);
    mpPixelDebug = PixelDebug::create();
}

Dictionary VolumetricReSTIR::getScriptingDictionary()
{
	Dictionary dict;
	serializePass<false>(dict);
	return dict;
}

RenderPassReflection VolumetricReSTIR::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    addRenderPassOutputs(reflector, kOutputChannels);
    return reflector;
}

bool VolumetricReSTIR::updateLights(RenderContext* pRenderContext)
{
    // If no scene is loaded, we disable everything.
    if (!mpScene)
    {
        mpEmissiveSampler = nullptr;
        return false;
    }

    // Request the light collection if emissive lights are enabled.
    if (mParams.mUseEmissiveLights)
    {
        mpScene->getLightCollection(pRenderContext);
    }

    bool lightingChanged = false;
    if (!mpScene->useEmissiveLights())
    {
        mpEmissiveSampler = nullptr;
    }
    else
    {
        // Create emissive light sampler if it doesn't already exist.
        if (mpEmissiveSampler == nullptr)
        {
            switch (mEmissiveSamplerType)
            {
            case EmissiveLightSamplerType::Uniform:
                mpEmissiveSampler = EmissiveUniformSampler::create(pRenderContext, mpScene, mUniformSamplerOptions);
                break;
            case EmissiveLightSamplerType::LightBVH:
                mpEmissiveSampler = LightBVHSampler::create(pRenderContext, mpScene, mLightBVHSamplerOptions);
                break;
            case EmissiveLightSamplerType::Power:
                mpEmissiveSampler = EmissivePowerSampler::create(pRenderContext, mpScene, mPowerSamplerOptions);
                break;
            default:
                logError("Unknown emissive light sampler type");
            }
            if (!mpEmissiveSampler) throw std::exception("Failed to create emissive light sampler");

            // need to recreate vars;
            mRequestRecreateVarsForEmissiveSampler = true;
        }

        // Update the emissive sampler to the current frame.
        assert(mpEmissiveSampler);
        lightingChanged = mpEmissiveSampler->update(pRenderContext);
    }

    return lightingChanged;
}

void VolumetricReSTIR::beginFrame(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Update lights. Returns true if emissive lights have changed.
    updateLights(pRenderContext);
}

void VolumetricReSTIR::toggleCameraAnimation()
{
    if (mCameraFramesMoved == 0)
    {
        mBackedupCameraPosition = mpScene->getCamera()->getPosition();
        mBackedupCameraTarget = mpScene->getCamera()->getTarget();
        mCameraFramesMoved = 1;
        mSavedVDBAnimationState = mpScene->mPauseVDBAnimation;
    }
    else
    {
        mFreezeFrame = false;
        resetCamera(true);
        mCameraFramesMoved = 0;
        mpScene->mPauseVDBAnimation = mSavedVDBAnimationState;
    }
}

void VolumetricReSTIR::overrideVolumeDesc()
{
    if (volumeDensityScaleExtraControl > 0)
    {
        mpScene->getCurrentVolumeDesc().densityScaleFactor = volumeDensityScaleExtraControl;
        mpScene->updateVolumeDesc();
    }

    if (volumeAnisotropyExtraControl > 0)
    {
        mpScene->getCurrentVolumeDesc().PhaseFunctionConstantG = volumeAnisotropyExtraControl;
        mpScene->updateVolumeDesc();
    }

    if (volumeAlbedoExtraControl > 0)
    {
        mpScene->getCurrentVolumeDesc().sigma_s = float3(mpScene->getCurrentVolumeDesc().sigma_t) * volumeAlbedoExtraControl;
        mpScene->getCurrentVolumeDesc().sigma_a = float3(mpScene->getCurrentVolumeDesc().sigma_t) - mpScene->getCurrentVolumeDesc().sigma_s;
        mpScene->updateVolumeDesc();
    }
}

void VolumetricReSTIR::moveCameraRight(float distance, float3 anchorPosition)
{
    float3 viewDir = normalize(mpScene->getCamera()->getTarget() - mpScene->getCamera()->getPosition());
    float3 rightVec = normalize(cross(viewDir, mpScene->getCamera()->getUpVector()));
    float3 newCamPos = anchorPosition + distance * rightVec;
    mpScene->getCamera()->setPosition(newCamPos);
    mpScene->getCamera()->setTarget(newCamPos + viewDir);
}

void VolumetricReSTIR::_forwardCameraInterval(float distance, float3 anchorPosition)
{
    float3 viewDir = normalize(mpScene->getCamera()->getTarget() - anchorPosition);
    float3 forwardCenter = anchorPosition + mCameraForwardScale * viewDir;
    float3 newCamPos = forwardCenter - viewDir * distance;
    mpScene->getCamera()->setPosition(newCamPos);
    mpScene->getCamera()->setTarget(newCamPos + viewDir);
}

void VolumetricReSTIR::resetCamera(bool useLastCameraPosition)
{
    mpScene->getCamera()->setPosition(useLastCameraPosition ? mBackedupCameraPosition : mInitialCameraPosition);
    mpScene->getCamera()->setTarget(useLastCameraPosition ? mBackedupCameraTarget : mInitialCameraTarget);
}

void VolumetricReSTIR::updateSceneDefines(Falcor::ComputePass::SharedPtr& pPass, const Falcor::Scene::SharedPtr& pScene)
{
    if (!pScene) return;

    pPass->getProgram()->addDefines(pScene->getSceneDefines());
    pPass->getProgram()->addDefine("_DEFAULT_ALPHA_TEST");
    pPass->getProgram()->addDefine("MAX_BOUNCES", std::to_string(mParams.mMaxBounces));
    if (mParams.mUseSurfaceScene)
    {
        pPass->getProgram()->addDefine("SURFACE_SCENE");
        pPass->getProgram()->addDefine("VBUFFERDECLARE", "VBufferItem vItem,");
        pPass->getProgram()->addDefine("VBUFFERITEM", "vItem,");
    }
    else
    {
        pPass->getProgram()->removeDefine("SURFACE_SCENE");
        pPass->getProgram()->addDefine("VBUFFERDECLARE", "");
        pPass->getProgram()->addDefine("VBUFFERITEM", "");
    }
    pPass->setVars(nullptr);
    pPass->getRootVar()["gScene"] = pScene->getParameterBlock();
}

void VolumetricReSTIR::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    beginFrame(pRenderContext, renderData);

    mpPixelDebug->beginFrame(pRenderContext, renderData.getDefaultTextureDims());

    if (mRequestRecreateVarsForEmissiveSampler)
    {
        mRequestRecreateVarsForEmissiveSampler = false;

        // Create emissive light sampler if it doesn't already exist.

        // Update the emissive sampler to the current frame.
        assert(mpEmissiveSampler);

        bool isDirty = mpEmissiveSampler->prepareProgram(mSpatialReusePass->getProgram().get());
        mpEmissiveSampler->prepareProgram(mTemporalReusePass->getProgram().get());
        mpEmissiveSampler->prepareProgram(mFinalShadingPass->getProgram().get());
        mpEmissiveSampler->prepareProgram(mpTraceRaysPass->getProgram().get());
        mpEmissiveSampler->prepareProgram(mGenerateFeaturePass->getProgram().get());

        mSpatialReusePass->recreateVars();
        mTemporalReusePass->recreateVars();
        mFinalShadingPass->recreateVars();
        mpTraceRaysPass->recreateVars();
        mGenerateFeaturePass->recreateVars();

        mSpatialReusePass->getRootVar()["gScene"] = mpScene->getParameterBlock();
        mTemporalReusePass->getRootVar()["gScene"] = mpScene->getParameterBlock();
        mFinalShadingPass->getRootVar()["gScene"] = mpScene->getParameterBlock();
        mpTraceRaysPass->getRootVar()["gScene"] = mpScene->getParameterBlock();
        mGenerateFeaturePass->getRootVar()["gScene"] = mpScene->getParameterBlock();
    }

    if (mpScene->mNewEnvMapLoaded)
    {
        mpEnvMapSampler = EnvMapSampler::create(pRenderContext, mpScene->getEnvMap());
        mSavedEnvMapRotation = mpScene->getEnvMap()->getRotation();
        mpScene->mNewEnvMapLoaded = false;
    }

    uint32_t scrWidth = renderData.getDefaultTextureDims().x;
    uint32_t scrHeight = renderData.getDefaultTextureDims().y;

    bool wasOptionsChanged = mOptionsChanged;

    if (mOptionsChanged)
    {
        if (mRandomizeFrameSpeed) mFrameCount = rand() % 65536;
        else mFrameCount = 0;
        mTemporalSampleAccumulated = 0;
        InternalDictionary& dict = renderData.getDictionary();
        auto flags = dict.getValue(kRenderPassRefreshFlags, Falcor::RenderPassRefreshFlags::None);
        if (mOptionsChanged) flags |= Falcor::RenderPassRefreshFlags::RenderOptionsChanged;
        dict[Falcor::kRenderPassRefreshFlags] = flags;
        mOptionsChanged = false;
    }

    int reservoirCount = scrWidth * scrHeight;

    uint32_t reservoirSize = sizeof(Reservoir) + (mParams.mMaxBounces == 1 ? 0 : 4) + (mParams.mMaxBounces > 1 && mParams.mVertexReuse ? 4 : 0);

    if (mParams.mMaxBounces != mLastMaxBounces && !mParams.mUseReference)
    {
        mSpatialReusePass->addDefine("MAX_BOUNCES", std::to_string(mParams.mMaxBounces));
        mTemporalReusePass->addDefine("MAX_BOUNCES", std::to_string(mParams.mMaxBounces));
        mGenerateFeaturePass->addDefine("MAX_BOUNCES", std::to_string(mParams.mMaxBounces));
        mCopyReservoirPass->addDefine("MAX_BOUNCES", std::to_string(mParams.mMaxBounces));
        mFinalShadingPass->addDefine("MAX_BOUNCES", std::to_string(mParams.mMaxBounces));
        mpTraceRaysPass->addDefine("MAX_BOUNCES", std::to_string(mParams.mMaxBounces));
		mLastMaxBounces = mParams.mMaxBounces;
    }

    if (mParams.mMaxBounces > 1 && mLastVertexReuse != mParams.mVertexReuse)
    {
        if (mParams.mVertexReuse)
        {
            mSpatialReusePass->addDefine("VERTEX_REUSE");
            mTemporalReusePass->addDefine("VERTEX_REUSE");
            mGenerateFeaturePass->addDefine("VERTEX_REUSE");
            mFinalShadingPass->addDefine("VERTEX_REUSE");
            mCopyReservoirPass->addDefine("VERTEX_REUSE");
            mpTraceRaysPass->addDefine("VERTEX_REUSE");

            mSpatialReusePass->addDefine("REUSETYPE", "inout");
            mTemporalReusePass->addDefine("REUSETYPE", "inout");
            mGenerateFeaturePass->addDefine("REUSETYPE", "inout");
            mFinalShadingPass->addDefine("REUSETYPE", "inout");
            mpTraceRaysPass->addDefine("REUSETYPE", "inout");
        }
        else
        {
            mSpatialReusePass->removeDefine("VERTEX_REUSE");
            mTemporalReusePass->removeDefine("VERTEX_REUSE");
            mGenerateFeaturePass->removeDefine("VERTEX_REUSE");
            mCopyReservoirPass->removeDefine("VERTEX_REUSE");
            mFinalShadingPass->removeDefine("VERTEX_REUSE");
            mpTraceRaysPass->removeDefine("VERTEX_REUSE");

            mSpatialReusePass->addDefine("REUSETYPE", "");
            mTemporalReusePass->addDefine("REUSETYPE", "");
            mGenerateFeaturePass->addDefine("REUSETYPE", "");
            mFinalShadingPass->addDefine("REUSETYPE", "");
            mpTraceRaysPass->addDefine("REUSETYPE", "");
        }
        mLastVertexReuse = mParams.mVertexReuse;
    }

    bool isScreenSizeChanged = renderData[kAccumulatedColorOutput]->asTexture()->getHeight() != scrHeight || renderData[kAccumulatedColorOutput]->asTexture()->getWidth() != scrWidth;

    // compute extra bounce storage
    int totalReservoirCount = reservoirCount;
    int totalExtraBounceReservoirCount = reservoirCount * (mParams.mMaxBounces - 1);

    uint32_t ExtraBounceReservoirSizeCollection = (mParams.mMaxBounces - 1) * 12;

    if (!mParams.mUseReference && (isScreenSizeChanged || !mPerPixelReservoirBuffer[0] || wasOptionsChanged && mPerPixelReservoirBuffer[0]->getSize() != totalReservoirCount * reservoirSize))
    {
        printf("Total Reservoir Count: %d\n", totalReservoirCount);
        mPerPixelReservoirBuffer[0] = Buffer::createStructured(reservoirSize, totalReservoirCount);
        mPerPixelReservoirBuffer[1] = Buffer::createStructured(reservoirSize, totalReservoirCount);
        mTemporalReservoirBuffer = Buffer::createStructured(reservoirSize, totalReservoirCount);
        mReservoirFeatureBuffer = Buffer::createStructured(sizeof(ReservoirFeatures), totalReservoirCount);
        mTemporalReservoirFeatureBuffer = Buffer::createStructured(sizeof(ReservoirFeatures), totalReservoirCount);
        printf("Reservoir size: %d\n", (int)reservoirSize);
    }

    if (mParams.mUseSurfaceScene && (isScreenSizeChanged || !mVBuffer || wasOptionsChanged || (mPerPixelReservoirBuffer[0] && mPerPixelReservoirBuffer[0]->getSize() != totalReservoirCount * reservoirSize)))
    {
        mVBuffer = Buffer::createStructured(sizeof(VBufferItem), scrHeight * scrWidth);
        mTemporalVBuffer = Buffer::createStructured(sizeof(VBufferItem), scrHeight * scrWidth);
    }

    if (!mParams.mUseReference && mParams.mMaxBounces > 1 && (isScreenSizeChanged || !mPerPixelExtraBounceReservoirBuffer[0] || wasOptionsChanged && mPerPixelExtraBounceReservoirBuffer[0]->getSize() != totalExtraBounceReservoirCount * ExtraBounceReservoirSizeCollection))
    {
        mPerPixelExtraBounceReservoirBuffer[0] = Buffer::createStructured(ExtraBounceReservoirSizeCollection, totalExtraBounceReservoirCount);
        mPerPixelExtraBounceReservoirBuffer[1] = Buffer::createStructured(ExtraBounceReservoirSizeCollection, totalExtraBounceReservoirCount);
        mTemporalExtraBounceReservoirBuffer = Buffer::createStructured(ExtraBounceReservoirSizeCollection, totalExtraBounceReservoirCount);
    }


    // handle window resizing / change of temporal SPP
    if (!mPerPixelColorBuffer[0] ||
        isScreenSizeChanged)
    {
        mPerPixelColorBuffer[0] = Texture::create2D(scrWidth, scrHeight, ResourceFormat::RGBA32Float, 1, 1, nullptr, ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource);
        mPerPixelColorBuffer[1] = Texture::create2D(scrWidth, scrHeight, ResourceFormat::RGBA32Float, 1, 1, nullptr, ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource);
    }

    int numInitialSamplingRounds = 1;
    int numTotalRounds = (int)(mParams.mEnableSpatialReuse ? mParams.mSpatialReuseRounds : 0) + (int)mParams.mEnableTemporalReuse + 1 + numInitialSamplingRounds;

    R2Params r2Params = { reservoirCount, mParams.mSpatialSampleCount, mParams.mSpatialReuseRounds, 16 };

    SamplingOptions initialOptions = {
    kAnalyticTracking,
    mParams.mInitialLightingTrackingMethod,
    mParams.mInitialLightSamples, // only 1 or 0
    mParams.mInitialLightingMipLevel,
    1,
    mParams.mInitialVisibilityUseLinearSampler ? mParams.mInitialBaseMipLevel : mParams.mInitialBaseMipLevel + kNumMaxMips,
    mParams.mInitialVisibilityUseLinearSampler,
    mParams.mInitialLightingUseLinearSampler,
    mParams.mInitialVisibilityTStepScale,
    mParams.mInitialLightingTStepScale,
    mParams.mUseEnvironmentLights, mParams.mUseAnalyticLights, mParams.mUseEmissiveLights, mParams.mVertexReuseStartBounce };

    SamplingOptions spatialOptions = {
    mParams.mSpatialVisibilityTrackingMethod,
    mParams.mSpatialLightingTrackingMethod,
    1,
    mParams.mSpatialLightingMipLevel,
    1,
    mParams.mSpatialVisibilityMipLevel,
    mParams.mSpatialVisibilityUseLinearSampler,
    mParams.mSpatialLightingUseLinearSampler,
    mParams.mSpatialVisibilityTStepScale,
    mParams.mSpatialLightingTStepScale,
    mParams.mUseEnvironmentLights, mParams.mUseAnalyticLights, mParams.mUseEmissiveLights, mParams.mVertexReuseStartBounce };

    // temporal use the same options as spatial

    SamplingOptions finalOptions = {
    mParams.mFinalVisibilityTrackingMethod,
    mParams.mFinalLightTrackingMethod,
    (mParams.mFinalLightTrackingMethod == kAnalyticTracking || mParams.mFinalLightTrackingMethod == kRayMarching) ? 1 : mParams.mFinalLightSamples,
    0, // mip level
    (mParams.mFinalVisibilityTrackingMethod == kAnalyticTracking || mParams.mFinalVisibilityTrackingMethod == kRayMarching) ? 1 : mParams.mFinalVisibilitySamples,
    0, // mip level
    true,
    true,
    mParams.mFinalTStepScale,
    mParams.mFinalTStepScale,
    mParams.mUseEnvironmentLights, mParams.mUseAnalyticLights, mParams.mUseEmissiveLights, mParams.mVertexReuseStartBounce };

    // Generate Feature Map
    if (!mFreezeFrame)
    {
        Profiler::startEvent("Generate Features");

        auto vars = mGenerateFeaturePass->getRootVar();
        mpPixelDebug->prepareProgram(mGenerateFeaturePass->getProgram(), vars);

        mpScene->setVolumeShaderData(vars);
        vars["gLinearSampler"] = mpSampler;
        vars["gPointSampler"] = mpPointSampler;

        initialOptions.setShaderData(vars["CB"]["gInitialSamplingOptions"]);
        vars["CB"]["gResolution"] = uint2(scrWidth, scrHeight);
        vars["CB"]["gUseReference"] = mParams.mUseReference;
        vars["gReservoirFeatureBuffer"] = mReservoirFeatureBuffer;
        if (mParams.mUseSurfaceScene)
        {
            vars["gVBuffer"] = mVBuffer;
            mpScene->setRaytracingAcceleraitonStructure(pRenderContext, vars["gAccelerationStructure"]);
        }
        mGenerateFeaturePass->getRootVar()["gScene"] = mpScene->getParameterBlock();
        mGenerateFeaturePass->execute(pRenderContext, uint3(renderData.getDefaultTextureDims(), 1));

        Profiler::endEvent("Generate Features");
    }

    if (!mFreezeFrame)
    {
        Profiler::startEvent("Generate Samples");
        auto vars = mpTraceRaysPass->getRootVar();

        mpPixelDebug->prepareProgram(mpTraceRaysPass->getProgram(), vars);

        mpScene->setVolumeShaderData(vars);

        vars["gLinearSampler"] = mpSampler;
        vars["gPointSampler"] = mpPointSampler;

        vars["gOutputColor"] = mPerPixelColorBuffer[0];
        vars["gOutputReservoirs"] = mPerPixelReservoirBuffer[0];
        vars["gReservoirFeatureBuffer"] = mReservoirFeatureBuffer;
        vars["gOutputExtraBounceReservoirs"] = mPerPixelExtraBounceReservoirBuffer[0];
        if (mParams.mUseSurfaceScene)
            vars["gVBuffer"] = mVBuffer;

        vars["CB"]["gResolution"] = renderData.getDefaultTextureDims();
        vars["CB"]["gFrameCount"] = mFrameCount;
        vars["CB"]["gNumTotalRounds"] = numTotalRounds;
        vars["CB"]["gMaxBounces"] = mParams.mMaxBounces;
        vars["CB"]["gUseReference"] = mParams.mUseReference;
        vars["CB"]["gBaselineSamplePerPixel"] = mParams.mBaselineSamplePerPixel;
        vars["CB"]["gNumInitialSamples"] = mParams.mInitialM;
        vars["CB"]["gUseRussianRoulette"] = mParams.mInitialUseRussianRoulette;
        vars["CB"]["gNoReuse"] = !mParams.mEnableSpatialReuse && !mParams.mEnableTemporalReuse;
        vars["CB"]["gUseCoarserGridForIndirectBounce"] = mParams.mInitialUseCoarserGridForIndirectBounce;

        initialOptions.setShaderData(vars["CB"]["gInitialSamplingOptions"]);
        spatialOptions.setShaderData(vars["CB"]["gSpatialSamplingOptions"]);

        if (mpEnvMapSampler) mpEnvMapSampler->setShaderData(vars["CB"]["envMapSampler"]);
        if (mpEmissiveSampler) mpEmissiveSampler->setShaderData(vars["CB"]["emissiveSampler"]);

        if (mParams.mUseSurfaceScene)
            mpScene->setRaytracingAcceleraitonStructure(pRenderContext, vars["gAccelerationStructure"]);
        mpTraceRaysPass->getRootVar()["gScene"] = mpScene->getParameterBlock();

        mpTraceRaysPass->execute(pRenderContext, uint3(renderData.getDefaultTextureDims(), 1));

        Profiler::endEvent("Generate Samples");
    }

    int totalRoundId = 0;

    if (mFreezeFrame) totalRoundId = mParams.mEnableSpatialReuse ? 1 : 0;

    // temporal reuse
    if (!mFreezeFrame)
        if (!mParams.mUseReference && mParams.mEnableTemporalReuse)
        {
            Profiler::startEvent("Temporal Reuse");
            auto vars = mTemporalReusePass->getRootVar();

            mpPixelDebug->prepareProgram(mTemporalReusePass->getProgram(), vars);

            mpScene->setVolumeShaderData(vars);
            vars["gLinearSampler"] = mpSampler;
            vars["gPointSampler"] = mpPointSampler;

            spatialOptions.setShaderData(vars["CB"]["gSamplingOptions"]);

            vars["gCurReservoirs"] = mPerPixelReservoirBuffer[totalRoundId];
            vars["gTemporalReservoirs"] = mTemporalReservoirBuffer;
            vars["gCurExtraBounceReservoirs"] = mPerPixelExtraBounceReservoirBuffer[totalRoundId];
            vars["gTemporalExtraBounceReservoirs"] = mTemporalExtraBounceReservoirBuffer;
            vars["gReservoirFeatureBuffer"] = mReservoirFeatureBuffer;
            vars["gTemporalReservoirFeatureBuffer"] = mTemporalReservoirFeatureBuffer;
            vars["gMotionVec"] = renderData[kMotionVec]->asTexture();

            if (mParams.mUseSurfaceScene)
            {
                vars["gVBuffer"] = mVBuffer;
                vars["gTemporalVBuffer"] = mTemporalVBuffer;
            }

            vars["CB"]["gResolution"] = uint2(scrWidth, scrHeight);
            vars["CB"]["gTemporalHistoryThreshold"] = mParams.mTemporalReuseMThreshold;
            vars["CB"]["gNumTotalRounds"] = numTotalRounds;
            vars["CB"]["gRoundOffset"] = numInitialSamplingRounds;
            vars["CB"]["gIsFirstFrame"] = mTemporalSampleAccumulated == 0;
            vars["CB"]["gFrameCount"] = mFrameCount;
            vars["CB"]["gPrevViewMat"] = mPrevViewMat;
            vars["CB"]["gPrevProjMat"] = mPrevProjMat;
            vars["CB"]["gReprojectionMode"] = mParams.mTemporalReprojectionMode;
            vars["CB"]["gPrevCameraU"] = mPrevCameraU;
            vars["CB"]["gPrevCameraV"] = mPrevCameraV;
            vars["CB"]["gPrevCameraW"] = mPrevCameraW;
            vars["CB"]["gPrevCameraPosW"] = mPrevCameraPosW;
            vars["CB"]["gMISMethod"] = mParams.mTemporalMISMethod;
            vars["CB"]["gOutputMotionVec"] = mOutputMotionVec;
            vars["CB"]["gReprojectionMipLevel"] = mParams.mTemporalReprojectionMipLevel;

            if (mpEnvMapSampler) mpEnvMapSampler->setShaderData(vars["CB"]["envMapSampler"]);
            if (mpEmissiveSampler) mpEmissiveSampler->setShaderData(vars["CB"]["emissiveSampler"]);

            if (mParams.mUseSurfaceScene)
                mpScene->setRaytracingAcceleraitonStructure(pRenderContext, vars["gAccelerationStructure"]);
            mTemporalReusePass->getRootVar()["gScene"] = mpScene->getParameterBlock();

            mTemporalReusePass->execute(pRenderContext, (int)scrWidth , (int)scrHeight );

            if (!mParams.mEnableSpatialReuse)
            {
                pRenderContext->copyResource(mTemporalReservoirBuffer.get(), mPerPixelReservoirBuffer[totalRoundId].get());
                if (mTemporalExtraBounceReservoirBuffer && mParams.mMaxBounces > 1)
                    pRenderContext->copyResource(mTemporalExtraBounceReservoirBuffer.get(), mPerPixelExtraBounceReservoirBuffer[totalRoundId].get());
            }

            pRenderContext->copyResource(mTemporalReservoirFeatureBuffer.get(), mReservoirFeatureBuffer.get());
            if (mParams.mUseSurfaceScene)
                pRenderContext->copyResource(mTemporalVBuffer.get(), mVBuffer.get());

            Profiler::endEvent("Temporal Reuse");
        }

    if (!mFreezeFrame)
        if (!mParams.mUseReference && mParams.mEnableSpatialReuse)
            // spatial reuse
        {
            Profiler::startEvent("Spatial Reuse");

            int startRoundId = 0;
            int endRoundId = mParams.mSpatialReuseRounds;

            int totalRoundIdBackup = totalRoundId;
            totalRoundId = totalRoundIdBackup;
            for (int roundId = startRoundId; roundId < endRoundId; roundId++)
            {
                auto vars = mSpatialReusePass->getRootVar();
                mpPixelDebug->prepareProgram(mSpatialReusePass->getProgram(), vars);

                mpScene->setVolumeShaderData(vars);
                vars["gLinearSampler"] = mpSampler;
                vars["gPointSampler"] = mpPointSampler;

                vars["gInputReservoirs"] = mPerPixelReservoirBuffer[totalRoundId % 2];
                vars["gOutputReservoirs"] = mPerPixelReservoirBuffer[(totalRoundId + 1) % 2];
                vars["gInputColors"] = mPerPixelColorBuffer[totalRoundId % 2];
                vars["gOutputColors"] = mPerPixelColorBuffer[(totalRoundId + 1) % 2];
                vars["gInputExtraBounceReservoirs"] = mPerPixelExtraBounceReservoirBuffer[totalRoundId % 2];
                vars["gOutputExtraBounceReservoirs"] = mPerPixelExtraBounceReservoirBuffer[(totalRoundId + 1) % 2];
                vars["gReservoirFeatureBuffer"] = mReservoirFeatureBuffer;
                if (mParams.mUseSurfaceScene)
                    vars["gVBuffer"] = mVBuffer;

                spatialOptions.setShaderData(vars["CB"]["gSamplingOptions"]);
                r2Params.setShaderData(vars["CB"]["gR2Params"]);

                vars["CB"]["gResolution"] = uint2(scrWidth, scrHeight);
                vars["CB"]["gFrameCount"] = mFrameCount;
                vars["CB"]["gRoundId"] = roundId;
                vars["CB"]["gNumRounds"] = mParams.mSpatialReuseRounds;
                vars["CB"]["gRoundOffset"] = (int)mParams.mEnableTemporalReuse + numInitialSamplingRounds;
                vars["CB"]["gMISMethod"] = mParams.mSpatialMISMethod;
                vars["CB"]["gRandomSamplerType"] = mParams.mRandomSamplerType;
                vars["CB"]["gSampleRadius"] = mParams.mSampleRadius;
                vars["CB"]["gSampleCount"] = mParams.mSpatialSampleCount;

                if (mpEnvMapSampler) mpEnvMapSampler->setShaderData(vars["CB"]["envMapSampler"]);
                if (mpEmissiveSampler) mpEmissiveSampler->setShaderData(vars["CB"]["emissiveSampler"]);

                if (mParams.mUseSurfaceScene)
                    mpScene->setRaytracingAcceleraitonStructure(pRenderContext, vars["gAccelerationStructure"]);
                mSpatialReusePass->getRootVar()["gScene"] = mpScene->getParameterBlock();

                mSpatialReusePass->execute(pRenderContext, (int)scrWidth , (int)scrHeight );
                totalRoundId++;
            }
            Profiler::endEvent("Spatial Reuse");
        }

    if (!mFreezeFrame)
        if (!mParams.mUseReference && mParams.mEnableTemporalReuse)
        {
            // launch a shader to copy resources
            Profiler::startEvent("Copy resource");

            auto vars = mCopyReservoirPass->getRootVar();
            vars["CB"]["gResolution"] = uint2(scrWidth, scrHeight);

            if (mParams.mMaxBounces > 1)
            {
                vars["gCurExtraBounceReservoirs"] = mPerPixelExtraBounceReservoirBuffer[totalRoundId % 2];
                vars["gTemporalExtraBounceReservoirs"] = mTemporalExtraBounceReservoirBuffer;
            }
            vars["gCurReservoirs"] = mPerPixelReservoirBuffer[totalRoundId % 2];
            vars["gTemporalReservoirs"] = mTemporalReservoirBuffer;
            mCopyReservoirPass->execute(pRenderContext, uint3(renderData.getDefaultTextureDims(), 1));

            Profiler::endEvent("Copy resource");
        }

    {
        Profiler::startEvent("Final Shading");

        auto vars = mFinalShadingPass->getRootVar();

        mpPixelDebug->prepareProgram(mFinalShadingPass->getProgram(), vars);

        mpScene->setVolumeShaderData(vars);
        vars["gLinearSampler"] = mpSampler;
        vars["gPointSampler"] = mpPointSampler;

        vars["gCurrentColors"] = mPerPixelColorBuffer[totalRoundId % 2];
        vars["gCurReservoirs"] = mPerPixelReservoirBuffer[totalRoundId % 2];
        vars["gCurExtraBounceReservoirs"] = mPerPixelExtraBounceReservoirBuffer[totalRoundId % 2];
        vars["gOutputFrame"] = renderData[kAccumulatedColorOutput]->asTexture();
        vars["gReservoirFeatureBuffer"] = mReservoirFeatureBuffer;

        if (mParams.mUseSurfaceScene)
            vars["gVBuffer"] = mVBuffer;

        finalOptions.setShaderData(vars["CB"]["gSamplingOptions"]);

        vars["CB"]["gResolution"] = uint2(scrWidth, scrHeight);
        vars["CB"]["gNumTotalRounds"] = numTotalRounds;
        vars["CB"]["gFrameCount"] = mFreezeFrame ? mFrameCount - 1 : mFrameCount;
        vars["CB"]["gSpatialReuse"] = mParams.mEnableSpatialReuse;
        vars["CB"]["gTemporalReuse"] = mParams.mEnableTemporalReuse;
        vars["CB"]["gUseReference"] = mParams.mUseReference;
        vars["CB"]["gMaxBounces"] = mParams.mMaxBounces;
        vars["CB"]["gVisualizeTotalTransmittance"] = mParams.mVisualizeTotalTransmittance;
        vars["CB"]["gNoReuse"] = !mParams.mEnableSpatialReuse && !mParams.mEnableTemporalReuse;

        if (mpEnvMapSampler) mpEnvMapSampler->setShaderData(vars["CB"]["envMapSampler"]);
        if (mpEmissiveSampler) mpEmissiveSampler->setShaderData(vars["CB"]["emissiveSampler"]);

        if (mParams.mUseSurfaceScene)
            mpScene->setRaytracingAcceleraitonStructure(pRenderContext, vars["gAccelerationStructure"]);
        mFinalShadingPass->getRootVar()["gScene"] = mpScene->getParameterBlock();

        mFinalShadingPass->execute(pRenderContext, scrWidth, scrHeight);

        Profiler::endEvent("Final Shading");
    }


    mTemporalSampleAccumulated = 1;

    mPrevViewMat = mpScene->getCamera()->getViewMatrix();
    mPrevProjMat = mpScene->getCamera()->getProjMatrix();
    mpScene->getCamera()->getRayTracingFrames(mPrevCameraU, mPrevCameraV, mPrevCameraW, mPrevCameraPosW);

    if (!mFreezeFrame)
        mFrameCount++;

    if (!mFreezeFrame && !mFreezeAnimation)
        mAnimationFrameCount++;

    if (mAnimationFrameCount == mAnimationFreezedFrame)
    {
        mFreezeFrame = true;
    }

    mpPixelDebug->endFrame(pRenderContext);

    // Update point lights

    float animationTime = (float)mAnimationFrameCount;

    if (mAnimateEnvLight)
    {
        float rotationSpeed = mEnvLightRotationSpeed;
        float rotDeg = 60 * sin(animationTime * rotationSpeed);
        mpScene->getEnvMap()->setRotation(float3(0, rotDeg, 0));
    }
    else // reset envlight rotation
    {
        if (mLastAnimateEnvLight != mAnimateEnvLight)
            mpScene->getEnvMap()->setRotation(mSavedEnvMapRotation);
    }
    mLastAnimateEnvLight = mAnimateEnvLight;

    if (mCameraFramesMoved > 0)
    {
        if (mCameraAnimationMode == 0)
        {
            int roundFrames = mCameraShakeRoundsBeforePause * mCameraFrameInterval + mCameraPauseInterval;
            int totalFrames = mCameraShakeTotalRounds * (mCameraShakeRoundsBeforePause * mCameraFrameInterval + mCameraPauseInterval);
            float cameraMoveSpeed = 2 * (float)M_PI / mCameraFrameInterval;

            if (mCameraFramesMoved++ < totalFrames)
            {
                int roundSubFrameId = mCameraFramesMoved % roundFrames;
                if (roundSubFrameId < mCameraShakeRoundsBeforePause * mCameraFrameInterval)
                {
                    mFreezeFrame = false;
                    mpScene->mPauseVDBAnimation = mSavedVDBAnimationState;
                    moveCameraRight(sin(cameraMoveSpeed * mCameraFramesMoved) * mCameraMoveScale, mBackedupCameraPosition);
                }
                else
                {
                    mFreezeFrame = true;
                    mpScene->mPauseVDBAnimation = true;
                }
            }
        }
        else if (mCameraAnimationMode == 1)
        {
            if (mCameraFramesMoved++ < 6 * mCameraFrameInterval)
            {
                int frameInterval = mCameraFrameInterval;

                float cameraMoveSpeed = 2 * (float)M_PI / frameInterval;

                if (mCameraFramesMoved < frameInterval)
                {
                    moveCameraRight(sin(cameraMoveSpeed * mCameraFramesMoved) * mCameraMoveScale, mBackedupCameraPosition);
                }
                else if (mCameraFramesMoved >= frameInterval && mCameraFramesMoved < 2 * frameInterval)
                {
                    mFreezeFrame = true;
                    mpScene->mPauseVDBAnimation = true;
                }
                else if (mCameraFramesMoved >= 2 * frameInterval && mCameraFramesMoved < 3 * frameInterval)
                {
                    mFreezeFrame = false;
                    mpScene->mPauseVDBAnimation = mSavedVDBAnimationState;
                    moveCameraRight(sin(cameraMoveSpeed * (mCameraFramesMoved - frameInterval)) * mCameraMoveScale, mBackedupCameraPosition);
                    mBackedupCameraPosition2 = mpScene->getCamera()->getPosition();
                }
                else if (mCameraFramesMoved >= 3 * frameInterval && mCameraFramesMoved < 3 * frameInterval + frameInterval / 4)
                {
                    mFreezeFrame = false;
                    mpScene->mPauseVDBAnimation = mSavedVDBAnimationState;
                    _forwardCameraInterval(mCameraForwardScale * cos(cameraMoveSpeed * (mCameraFramesMoved - 3 * frameInterval)), mBackedupCameraPosition2);
                }
                else if (mCameraFramesMoved >= 3 * frameInterval + frameInterval / 4 && mCameraFramesMoved < 4 * frameInterval + frameInterval / 4)
                {
                    mFreezeFrame = true;
                    mpScene->mPauseVDBAnimation = true;
                }
                else if (mCameraFramesMoved >= 4 * frameInterval + frameInterval / 4 && mCameraFramesMoved < 4 * frameInterval + 3 * frameInterval / 4)
                {
                    mFreezeFrame = false;
                    mpScene->mPauseVDBAnimation = mSavedVDBAnimationState;
                    _forwardCameraInterval(mCameraForwardScale * cos(cameraMoveSpeed * (mCameraFramesMoved - 4 * frameInterval)), mBackedupCameraPosition2);
                }
                else if (mCameraFramesMoved >= 4 * frameInterval + 3 * frameInterval / 4 && mCameraFramesMoved < 5 * frameInterval + 3 * frameInterval / 4)
                {
                    mFreezeFrame = true;
                    mpScene->mPauseVDBAnimation = true;
                }
                else if (mCameraFramesMoved >= 5 * frameInterval + 3 * frameInterval / 4 && mCameraFramesMoved < 6 * frameInterval)
                {
                    mFreezeFrame = false;
                    mpScene->mPauseVDBAnimation = mSavedVDBAnimationState;
                    _forwardCameraInterval(mCameraForwardScale * cos(cameraMoveSpeed * (mCameraFramesMoved - 5 * frameInterval)), mBackedupCameraPosition2);
                }
            }
        }
        else
        {
            float cameraMoveSpeed = 2 * (float)M_PI / mCameraFrameInterval;

            // find a point that is on the initial view ray and closest to the volume center to be the center
            float3 rotCenter = mBackedupCameraPosition + dot(mpScene->getSceneVolumeCenter() - mBackedupCameraPosition, normalize(mBackedupCameraTarget - mBackedupCameraPosition)) * normalize(mBackedupCameraTarget - mBackedupCameraPosition);

            if (mCameraFramesMoved++ < mCameraRotationFrames)
            {
                mFreezeFrame = false;
                mpScene->mPauseVDBAnimation = mSavedVDBAnimationState;

                float angleRotated = mCameraFramesMoved * mCameraRotationSpeed * (float)M_PI / 180.f;

                float3 Xdir = normalize(mBackedupCameraPosition - rotCenter);
                float3 Ydir = normalize(cross(float3(0,1,0), Xdir));
                Xdir = normalize(cross(Ydir, float3(0, 1, 0)));

                float3 actualRotCenter = rotCenter + (mBackedupCameraPosition - rotCenter) - (dot((mBackedupCameraPosition - rotCenter), Xdir) * Xdir);

                float camDist = mCameraRotationDistance <= 0 ? length(mBackedupCameraPosition - actualRotCenter) : mCameraRotationDistance;
                float3 camPos = actualRotCenter + camDist * (Xdir * cos(angleRotated) + Ydir * sin(angleRotated));
                mpScene->getCamera()->setPosition(camPos);
                mpScene->getCamera()->setTarget(rotCenter);
            }
        }
    }
}

void VolumetricReSTIR::renderUI(Gui::Widgets& widget)
{
    bool dirty = false;

    if (auto logGroup = widget.group("Logging"))
    {
        // Pixel debugger.
        mpPixelDebug->renderUI(logGroup);
    }


    if (mpScene && mParams.mUseEmissiveLights)
    {
        widget.text("Emissive sampler:");
        widget.tooltip("Selects which light sampler to use for importance sampling of emissive geometry.", true);
        if (widget.dropdown("##EmissiveSampler", kEmissiveSamplerList, (uint32_t&)mEmissiveSamplerType, true))
        {
            mpEmissiveSampler = nullptr;
            dirty = true;
        }
    }

    if (mpEmissiveSampler)
    {
        if (auto emissiveGroup = widget.group("Emissive sampler options"))
        {
            if (mpEmissiveSampler->renderUI(emissiveGroup))
            {
                // Get the latest options for the current sampler. We need these to re-create the sampler at scene changes and for pass serialization.
                switch (mEmissiveSamplerType)
                {
                case EmissiveLightSamplerType::Uniform:
                    mUniformSamplerOptions = std::static_pointer_cast<EmissiveUniformSampler>(mpEmissiveSampler)->getOptions();
                    break;
                case EmissiveLightSamplerType::LightBVH:
                    mLightBVHSamplerOptions = std::static_pointer_cast<LightBVHSampler>(mpEmissiveSampler)->getOptions();
                    break;
                case EmissiveLightSamplerType::Power:
                    mPowerSamplerOptions = std::static_pointer_cast<EmissivePowerSampler>(mpEmissiveSampler)->getOptions();
                    break;
                default:
                    should_not_get_here();
                }
                dirty = true;
            }
        }
    }


    auto group_ = Gui::Group(widget, "Animation", false);
    if (group_.open())
    {
        bool isDirty = widget.checkbox("Animate Env Lights", mAnimateEnvLight);
        if (isDirty && mAnimateEnvLight) {
            mAnimationFrameCount = 0;
            mpScene->getEnvMap()->setRotation(mSavedEnvMapRotation);
            mAnimationStartTime = gpFramework->getGlobalClock().getTime();
        }
        isDirty |= widget.var("Env Light Animation Speed", mEnvLightRotationSpeed);
        dirty |= isDirty;

        widget.var("camera frame interval", mCameraFrameInterval, 10, 2000);
        widget.var("camera move scale", mCameraMoveScale, 0.01f, 20.f);

        Gui::DropdownList op;
        op.push_back({ 0, "Shake" });
        op.push_back({ 1, "Shake+Zoom" });
        op.push_back({ 2, "Rotation" });
        widget.dropdown("camera animation mode (press \"B\" to animate)", op, mCameraAnimationMode);

        if (mCameraAnimationMode == 0)
        {
            widget.var("camera pause interval", mCameraPauseInterval, 10, 2000);
            widget.var("camera shake total rounds", mCameraShakeTotalRounds, 1, 5);
            widget.var("camera shake rounds before pause", mCameraShakeRoundsBeforePause, 1, 20);
        }
        else if (mCameraAnimationMode == 1)
        {
            widget.var("camera forward scale", mCameraForwardScale, 0.01f, 100.f);
        }
        else
        {
            widget.var("camera rotation frames", mCameraRotationFrames, 1, 129600);
            widget.var("camera rotation speed", mCameraRotationSpeed, 0.001f, 10.f);
            widget.var("camera rotation distance", mCameraRotationDistance, 0.f, 100.f);
        }

        dirty |= widget.var("Freeze At Animation Frame", mAnimationFreezedFrame, -1, 5000);

        dirty |= isDirty;
        group_.release();
    }

    dirty |= mpScene->renderVolumeUI(widget);

    auto group = Gui::Group(widget, "VolumetricReSTIR", true);

    if (group.open())
    {
        auto groupReservoir = Gui::Group(widget, "ReSTIR settings", true);

        if (groupReservoir.open())
        {
            dirty |= widget.checkbox("Temporal Reuse", mParams.mEnableTemporalReuse);
            dirty |= widget.checkbox("Spatial Reuse", mParams.mEnableSpatialReuse);
            dirty |= widget.checkbox("Vertex Reuse", mParams.mVertexReuse);
            dirty |= widget.var("Vertex Reuse Start Bounce", mParams.mVertexReuseStartBounce, 1, 64);
            dirty |= widget.checkbox("Visualize Total Transmittance", mParams.mVisualizeTotalTransmittance);

            groupReservoir.release();
        }

        auto groupGen = Gui::Group(widget, "General Settings", true);

        if (groupGen.open())
        {
            bool isFreezeChanged = widget.checkbox("Freeze Frame", mFreezeFrame);
            if (isFreezeChanged)
            {
                mpScene->mFreezeCamera = mFreezeFrame;
                if (mFreezeFrame)
                    mSavedVDBAnimationState = mpScene->mPauseVDBAnimation;
                mpScene->mPauseVDBAnimation = mFreezeFrame ? true : mSavedVDBAnimationState;
            }

            // global controls
            dirty |= widget.var("max bounces", mParams.mMaxBounces, 1, 64);

            dirty |= widget.checkbox("Use Volume Path Tracing", mParams.mUseReference);
            widget.tooltip("Switch to baseline (volume path tracing).");
            if (mParams.mUseReference)
            {
                dirty |= widget.var("Volume Path Tracing spp", mParams.mBaselineSamplePerPixel, 1, 32);
            }

            dirty |= widget.checkbox("use environment lights", mParams.mUseEnvironmentLights);
            dirty |= widget.checkbox("use analytic lights", mParams.mUseAnalyticLights);
            dirty |= widget.checkbox("use emissive lights", mParams.mUseEmissiveLights);

            groupGen.release();
        }


        auto group0 = Gui::Group(widget, "Initial Sampling", false);
        if (group0.open())
        {
            dirty |= widget.var("M", mParams.mInitialM, 1, 32);

            dirty |= widget.var("Initial Tracking Mip Level", mParams.mInitialBaseMipLevel, 0, mpScene->getVolumeNumMips() - 1);
            widget.tooltip("Volume mip-map level used for regular tracking to produce the initial samples.");
            dirty |= widget.var("NEE Transmittance Mip Level", mParams.mInitialLightingMipLevel, 0, mpScene->getVolumeNumMips() - 1);
            widget.tooltip("Volume mip-map level used for estimate the light transmittance of the initial samples.");

            dirty |= widget.checkbox("Use Trilinear Reg Tracking", mParams.mInitialVisibilityUseLinearSampler);
            widget.tooltip("Regular tracking with trilinear density (more expensive).");
            dirty |= widget.checkbox("NEE Transmittance Use Linear Sampler", mParams.mInitialLightingUseLinearSampler);
            widget.tooltip("Use trilinearly filtered density for estimating transmittance.");

            //dirty |= widget.var("Vis T Step Scale (for total transmittance)", mParams.mInitialVisibilityTStepScale, 1.f, 10.f);
            dirty |= widget.checkbox("Use Coarser Grid for Reg Tracking Indirect Bounces", mParams.mInitialUseCoarserGridForIndirectBounce);
            widget.tooltip("Use Initial Tracking Mip Level + 1 for bounce > 0");
            dirty |= widget.var("NEE Transmittance T Step Scale", mParams.mInitialLightingTStepScale, 1.f, 10.f);
            widget.tooltip("Step size (X times voxel size) in ray marching.");

            dirty |= widget.checkbox("Use Russian Roulette", mParams.mInitialUseRussianRoulette);


            bool computeTL = mParams.mInitialLightSamples == 0 ? false : true;
            bool changed = widget.checkbox("Compute NEE Transmittance", computeTL);
            widget.tooltip("Use volumetric shadow in target PDF for initial resampling");
            if (changed) mParams.mInitialLightSamples = computeTL ? 1 : 0;
            dirty |= changed;

            {
                Gui::DropdownList op;

                uint32_t temp = mParams.mInitialLightingTrackingMethod - kAnalyticTracking;
                op.push_back({ 0, "Regular" });
                op.push_back({ 1, "Ray Marching" });
                dirty |= widget.dropdown("Light Tracking Method", op, temp);
				mParams.mInitialLightingTrackingMethod = temp + kAnalyticTracking;
            }
            group0.release();
        }

        auto group__ = Gui::Group(widget, "Spatiotemporal Shared Options", false);

        if (group__.open())
        {
            {
                dirty |= widget.var("Transmittance Mip Level", mParams.mSpatialVisibilityMipLevel, 0, mpScene->getVolumeNumMips() - 1);
                widget.tooltip("Shared volume mip-map level used for primary visibility resampling for spatial/temporal neighbors.");
                dirty |= widget.var("NEE Transmittance Mip Level", mParams.mSpatialLightingMipLevel, 0, mpScene->getVolumeNumMips() - 1);
                widget.tooltip("Shared volume mip-map level used for light transmittance resampling for spatial/temporal neighbors.");
            }

            {
                dirty |= widget.checkbox("Transmittance Use Linear Sampler", mParams.mSpatialVisibilityUseLinearSampler);
                dirty |= widget.checkbox("NEE Transmittance Use Linear Sampler", mParams.mSpatialLightingUseLinearSampler);
            }

            {
                dirty |= widget.var("Transmittance T Step Scale", mParams.mSpatialVisibilityTStepScale, 1.f, 10.f);
                dirty |= widget.var("NEE Transmittance T Step Scale", mParams.mSpatialLightingTStepScale, 1.f, 10.f);
                widget.tooltip("Step size (X times voxel size) in ray marching.");
            }


            {
                Gui::DropdownList op;
                op.push_back({ 0, "Regular" });
                op.push_back({ 1, "Ray Marching" });

                uint32_t tempVis = mParams.mSpatialVisibilityTrackingMethod - kAnalyticTracking;
                uint32_t tempLight = mParams.mSpatialLightingTrackingMethod - kAnalyticTracking;

                dirty |= widget.dropdown("Transmittance Tracking Method", op, tempVis);
                dirty |= widget.dropdown("NEE Transmittance Tracking Method", op, tempLight);

                mParams.mSpatialVisibilityTrackingMethod = tempVis + kAnalyticTracking;
                mParams.mSpatialLightingTrackingMethod = tempLight + kAnalyticTracking;
            }

            group__.release();
        }

        auto group2 = Gui::Group(widget, "Spatial Reuse Options", false);
        if (group2.open())
        {
            dirty |= widget.var("Spatial Reuse Rounds", mParams.mSpatialReuseRounds, 0, 9);

            {
                Gui::DropdownList op;
                op.push_back({ 0, "Hammersley" });
                op.push_back({ 1, "R2" });
                dirty |= widget.dropdown("Sampler Type", op, mParams.mRandomSamplerType);
                widget.tooltip("Using R2 allows rotating the spatial kernel per frame.");
            }

            dirty |= widget.var("Sample Radius", mParams.mSampleRadius, 1.f, 100.f);
            dirty |= widget.var("Sample Count", mParams.mSpatialSampleCount, 1, 16);
            widget.tooltip("Total spatial sample count, which is number of spatial neighbors + 1.");


            {
                Gui::DropdownList op;
                op.push_back({ 0, "No MIS (biased)" });
                op.push_back({ 1, "Talbot MIS" });
                dirty |= widget.dropdown("MIS Method", op, mParams.mSpatialMISMethod);
            }

            group2.release();
        }

        auto group3 = Gui::Group(widget, "Temporal Reuse Options", false);
        if (group3.open())
        {
            dirty |= widget.var("Temporal Reuse M threshold", mParams.mTemporalReuseMThreshold, 0.f, 1000.f);
            widget.tooltip("A value X Limits the temporal reservoir's M to X times current frame's reservoir's M.");

            {
                Gui::DropdownList op;
                op.push_back({ 0, "Reproject with Velocity Resampling" });
                op.push_back({ 1, "No Reprojection" });
                op.push_back({ 2, "Reproject without Velocity Resampling" });
                dirty |= widget.dropdown("Reprojection Mode", op, mParams.mTemporalReprojectionMode);
            }

            dirty |= widget.var("Reprojection Mip Level", mParams.mTemporalReprojectionMipLevel, 0, mpScene->getVolumeNumMips() - 1);
            {
                Gui::DropdownList op;
                op.push_back({ 0, "No MIS (biased when moving)" });
                op.push_back({ 1, "Talbot MIS" });

                dirty |= widget.dropdown("MIS Method", op, mParams.mTemporalMISMethod);
            }

            group3.release();
        }

        auto group5 = Gui::Group(widget, "Final Shading Options", false);
        widget.tooltip("Sampling options to estimate the final integrand F.");
        if (group5.open())
        {
            {
                Gui::DropdownList op;
                op.push_back({ 0, "Ratio" });
                op.push_back({ 1, "Analytic" });
                op.push_back({ 2, "Ray Marching (biased)" });
                op.push_back({ 3, "Residual Ratio Tracking" });
                op.push_back({ 4, "Analog Residual Ratio Tracking" });

                dirty |= widget.dropdown("Transmittance Tracking method", op, mParams.mFinalVisibilityTrackingMethod);
                if (!(mParams.mFinalVisibilityTrackingMethod == kAnalyticTracking || mParams.mFinalVisibilityTrackingMethod == kRayMarching))
                    dirty |= widget.var("Transmittance samples", mParams.mFinalVisibilitySamples, 1, 16);

                dirty |= widget.dropdown("NEE transmittance tracking method", op, mParams.mFinalLightTrackingMethod);
                if (!(mParams.mFinalLightTrackingMethod == kAnalyticTracking || mParams.mFinalLightTrackingMethod == kRayMarching))
                    dirty |= widget.var("NEE transmittance samples", mParams.mFinalLightSamples, 1, 16);
            }

            group5.release();
        }

        group.release();
    }

    if (dirty) mOptionsChanged = true;
}

inline void VolumetricReSTIR::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    if (pScene)
    {
        mpScene = pScene;
        this->updateSceneDefines(mpTraceRaysPass, pScene);
        this->updateSceneDefines(mSpatialReusePass, pScene);
        this->updateSceneDefines(mTemporalReusePass, pScene);
        this->updateSceneDefines(mGenerateFeaturePass, pScene);
        this->updateSceneDefines(mFinalShadingPass, pScene);

        mpScene->getCurrentVolumeDesc().usePrevGridForReproj = mParams.mUsePrevVolumeForReproj;

        mpTraceRaysPass->getProgram()->addDefines(mpSampleGenerator->getDefines());
        mSpatialReusePass->getProgram()->addDefines(mpSampleGenerator->getDefines());
        mTemporalReusePass->getProgram()->addDefines(mpSampleGenerator->getDefines());
        mGenerateFeaturePass->getProgram()->addDefines(mpSampleGenerator->getDefines());
        mFinalShadingPass->getProgram()->addDefines(mpSampleGenerator->getDefines());

        bool success = mpSampleGenerator->setShaderData(mpTraceRaysPass->getRootVar());
        success |= mpSampleGenerator->setShaderData(mSpatialReusePass->getRootVar());
        success |= mpSampleGenerator->setShaderData(mTemporalReusePass->getRootVar());
        success |= mpSampleGenerator->setShaderData(mGenerateFeaturePass->getRootVar());
        success |= mpSampleGenerator->setShaderData(mFinalShadingPass->getRootVar());

        if (pScene->getEnvMap())
        {
            mSavedEnvMapRotation = mpScene->getEnvMap()->getRotation();
            mpEnvMapSampler = EnvMapSampler::create(pRenderContext, pScene->getEnvMap());
        }

        mpEmissiveSampler = nullptr;

        overrideVolumeDesc();
    }

    mInitialCameraPosition = mpScene->getCamera()->getPosition();
    mInitialCameraTarget = mpScene->getCamera()->getTarget();
}

bool VolumetricReSTIR::onMouseEvent(const MouseEvent& mouseEvent)
{
    return mpPixelDebug->onMouseEvent(mouseEvent);
}

bool VolumetricReSTIR::onKeyEvent(const KeyboardEvent& keyEvent)
{
    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == KeyboardEvent::Key::R)
    {
        resetCamera(false);
        mOptionsChanged = true;
        return true;
    }

    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == KeyboardEvent::Key::B)
    {
        toggleCameraAnimation();
    }

    return false;
}

void VolumetricReSTIR::updateDict(const Dictionary& dict)
{
    serializePass<true>(dict);

    if ((int)mEmissiveSamplerType != mEmissiveSamplerTypeId)
    {
        if (mEmissiveSamplerTypeId == 0)
            mEmissiveSamplerType = EmissiveLightSamplerType::Uniform;
        else if (mEmissiveSamplerTypeId == 1)
            mEmissiveSamplerType = EmissiveLightSamplerType::LightBVH;
        else if (mEmissiveSamplerTypeId == 2)
            mEmissiveSamplerType = EmissiveLightSamplerType::Power;

        mpEmissiveSampler = nullptr;
    }

    // reset animation
    if (mVolumeAnimationSelectedFrameId == -1)
    {
        if (mpScene->mUseAnimatedVolume)
            mpScene->mVDBAnimationFrameId = -1;
        else
            mpScene->mVDBAnimationFrameId = 0;
        mpScene->mPauseVDBAnimation = false;
    }
    else
    {
        mpScene->mVDBAnimationFrameId = mVolumeAnimationSelectedFrameId - 1;
        mpScene->mPauseVDBAnimation = true;
    }

    overrideVolumeDesc();
    mpScene->getCurrentVolumeDesc().usePrevGridForReproj = mParams.mUsePrevVolumeForReproj;

    mAnimationFrameCount = 0;
    mpScene->getEnvMap()->setRotation(mSavedEnvMapRotation);

    if (dict.keyExists("ToggleCameraAnimation"))
    {
        toggleCameraAnimation();
    }

    if (dict.keyExists("moveCameraRight"))
    {
        float distance = dict["moveCameraRight"];
        moveCameraRight(distance, mInitialCameraPosition);
    }

    if (dict.keyExists("resetCamera"))
    {
        resetCamera(false);
    }

    if (dict.keyExists("randomizeFrameSeed"))
    {
        if (!mRandomizeFrameSpeed) srand(123);
        mRandomizeFrameSpeed = true;
    }

    mOptionsChanged = true;

    if (dict.keyExists("moveCameraRight"))
    {
        mOptionsChanged = false;
    }

    printf("update pass!\n");
}

SCRIPT_BINDING(VolumetricReSTIR)
{
	// Register our parameters struct.
	ScriptBindings::SerializableStruct<VolumetricReSTIR::VolumetricReSTIRParams> params(m, "VolumetricReSTIRParams");
#define field(f_) field(#f_, &VolumetricReSTIR::VolumetricReSTIRParams::f_)
	// General

    params.field(mMaxBounces);

    params.field(mVertexReuse);
    params.field(mVertexReuseStartBounce);

    params.field(mEnableTemporalReuse);
    params.field(mEnableSpatialReuse);
    params.field(mUseReference);

    params.field(mUseEnvironmentLights);
    params.field(mUseAnalyticLights);
    params.field(mUseEmissiveLights);

    // Lower-res reservoirs
    params.field(mBaselineSamplePerPixel);

    // Alternatives
    params.field(mVisualizeTotalTransmittance);
    params.field(mUseSurfaceScene);

    /////////////////////
    /// Initial Sampling
    /////////////////////
    params.field(mInitialBaseMipLevel);
    params.field(mInitialM);
    params.field(mInitialLightSamples);
    params.field(mInitialLightingMipLevel);
    params.field(mInitialLightingUseLinearSampler);
    params.field(mInitialVisibilityUseLinearSampler);
    params.field(mInitialLightingTrackingMethod);
    params.field(mInitialVisibilityTStepScale);
    params.field(mInitialLightingTStepScale);
    params.field(mInitialUseCoarserGridForIndirectBounce);
    params.field(mInitialUseRussianRoulette);

    /////////////////////
    /// Temporal Reuse
    /////////////////////
    params.field(mTemporalReuseMThreshold);
    params.field(mTemporalReprojectionMode);
    params.field(mTemporalMISMethod);
    params.field(mTemporalReprojectionMipLevel);

    /////////////////////
    /// Spatial Reuse
    /////////////////////
    params.field(mSpatialReuseRounds);
    params.field(mSpatialVisibilityMipLevel);
    params.field(mSpatialLightingMipLevel);
    params.field(mSpatialVisibilityUseLinearSampler);
    params.field(mSpatialLightingUseLinearSampler);
    params.field(mSpatialVisibilityTStepScale);
    params.field(mSpatialLightingTStepScale);
    params.field(mSpatialVisibilityTrackingMethod);
    params.field(mSpatialLightingTrackingMethod);
    params.field(mRandomSamplerType);
    params.field(mSampleRadius);

    params.field(mSpatialSampleCount);
    params.field(mEnableVisibilitySimilarityRejection);
    params.field(mSpatialMISMethod);

    /////////////////////
    /// Final Shading
    /////////////////////
    params.field(mFinalLightSamples);
    params.field(mFinalVisibilitySamples);
    params.field(mFinalVisibilityTrackingMethod);
    params.field(mFinalLightTrackingMethod);
    params.field(mFinalRandomSamplerType);
    params.field(mFinalTStepScale);

#undef field

}

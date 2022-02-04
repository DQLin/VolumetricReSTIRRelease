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
#pragma once
#include "Falcor.h"
#include "FalcorExperimental.h"
#include "Utils/Sampling/SampleGenerator.h"
#include "Experimental/Scene/Lights/EnvMapSampler.h"
#include "Experimental/Scene/Lights/LightBVHSampler.h"
#include "Experimental/Scene/Lights/EmissivePowerSampler.h"
#include "HostDeviceSharedDefinitions.h"
#include "Utils/Debug/PixelDebug.h"
#include "HostDeviceSharedConstants.slang"

using namespace Falcor;

class VolumetricReSTIR : public RenderPass
{
public:
    using SharedPtr = std::shared_ptr<VolumetricReSTIR>;

    /** Create a new render pass object.
        \param[in] pRenderContext The render context.
        \param[in] dict Dictionary of serialized parameters.
        \return A new object, or an exception is thrown if creation failed.
    */
    static SharedPtr create(RenderContext* pRenderContext = nullptr, const Dictionary& dict = {});

    virtual std::string getDesc() override { return "Click to expand options"; }
    virtual Dictionary getScriptingDictionary() override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pContext, const CompileData& compileData) override {}
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene) override;
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override;
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override;
    virtual void updateDict(const Dictionary& dict) override;

private:
    void updateSceneDefines(Falcor::ComputePass::SharedPtr& pPass, const Falcor::Scene::SharedPtr& pScene);
    bool updateLights(RenderContext* pRenderContext);
    void beginFrame(RenderContext* pRenderContext, const RenderData& renderData);
    void toggleCameraAnimation();
    void overrideVolumeDesc();
    void moveCameraRight(float distance, float3 anchorPosition);
    void _forwardCameraInterval(float distance, float3 anchorPosition);
    void resetCamera(bool useLastCameraPosition);

    Sampler::SharedPtr mpSampler;
    Sampler::SharedPtr mpPointSampler;

    Scene::SharedPtr mpScene;

    VolumetricReSTIR(const Dictionary& dict);

    int mFrameCount = 0;

    SampleGenerator::SharedPtr          mpSampleGenerator;              ///< GPU sample generator.
    EnvMapSampler::SharedPtr            mpEnvMapSampler;                ///< Environment map sampler or nullptr if disabled.
    EmissiveLightSampler::SharedPtr     mpEmissiveSampler;

    int mEmissiveSamplerTypeId = 2;
    EmissiveLightSamplerType mEmissiveSamplerType = EmissiveLightSamplerType::Power;
    EmissiveUniformSampler::Options mUniformSamplerOptions;
    LightBVHSampler::Options            mLightBVHSamplerOptions;        ///< Current options for the light BVH sampler.
    EmissivePowerSampler::Options            mPowerSamplerOptions;        ///< Current options for the light BVH sampler.

    PixelDebug::SharedPtr               mpPixelDebug;                    ///< Utility class for pixel debugging (print in shaders).

    std::vector<Texture::SharedPtr> mTemporalSampleBuffers;
    Buffer::SharedPtr mPerPixelReservoirBuffer[2];
    Buffer::SharedPtr mTemporalReservoirBuffer;
    Buffer::SharedPtr mPerPixelExtraBounceReservoirBuffer[2];
    Buffer::SharedPtr mTemporalExtraBounceReservoirBuffer;
    Buffer::SharedPtr mReservoirFeatureBuffer;
    Buffer::SharedPtr mTemporalReservoirFeatureBuffer;
    Buffer::SharedPtr mVBuffer;
    Buffer::SharedPtr mTemporalVBuffer;

    // for debug purpose
    Texture::SharedPtr mAccumulateBuffer;
    Texture::SharedPtr mPerPixelColorBuffer[2];
    Texture::SharedPtr mOutputBackupBuffer;

    ComputePass::SharedPtr mpTraceRaysPass;
    ComputePass::SharedPtr mTemporalReusePass;
    ComputePass::SharedPtr mSpatialReusePass;
    ComputePass::SharedPtr mGenerateFeaturePass;
    ComputePass::SharedPtr mCopyReservoirPass;
    ComputePass::SharedPtr mFinalShadingPass;
    ComputePass::SharedPtr mComputeAvgDensityPass;


	bool mFreezeFrame = false;
    bool mFreezeAnimation = false;

    /////////////////////
    /// Global controls
    /////////////////////
    public:

    int mLastMaxBounces = 1;
    bool mLastVertexReuse = false;

    struct VolumetricReSTIRParams
    {
        int mMaxBounces = 1;

        bool mEnableTemporalReuse = true;
        bool mEnableSpatialReuse = true;
        bool mVertexReuse = false;
        int mVertexReuseStartBounce = 1;

        bool mUseReference = false;

        bool mUseEnvironmentLights = true;
        bool mUseAnalyticLights = false;
        bool mUseEmissiveLights = false;

        int mBaselineSamplePerPixel = 1;

        // Alternatives

        bool mVisualizeTotalTransmittance = false;

        bool mUseSurfaceScene = false;
        bool mUsePrevVolumeForReproj = true;

        /////////////////////
        /// Initial Sampling
        /////////////////////
        int mInitialBaseMipLevel = 1;
        int mInitialM = 4;
        int mInitialLightSamples = 1;
        int mInitialLightingMipLevel = 2;
        bool mInitialVisibilityUseLinearSampler = false;
        bool mInitialLightingUseLinearSampler = true;

        uint32_t mInitialLightingTrackingMethod = kRayMarching;
        float mInitialVisibilityTStepScale = 1.f;
        float mInitialLightingTStepScale = 2.f;
        bool mInitialUseRussianRoulette = true;
        bool mInitialUseCoarserGridForIndirectBounce = true;

        /////////////////////
        /// Temporal Reuse
        /////////////////////
        float mTemporalReuseMThreshold = 4.f;
        uint32_t mTemporalReprojectionMode = kReprojectionLinear;
        uint32_t mTemporalMISMethod = kMISTalbot;
        int mTemporalReprojectionMipLevel = 1;

        /////////////////////
        /// Spatial Reuse
        /////////////////////
        int mSpatialReuseRounds = 1;
        int mSpatialVisibilityMipLevel = 1;
        int mSpatialLightingMipLevel = 1;
        bool mSpatialVisibilityUseLinearSampler = true;
        bool mSpatialLightingUseLinearSampler = true;
        float mSpatialVisibilityTStepScale = 1.f;
        float mSpatialLightingTStepScale = 1.f;
        uint32_t mSpatialVisibilityTrackingMethod = kRayMarching;
        uint32_t mSpatialLightingTrackingMethod = kRayMarching;
        uint32_t mRandomSamplerType = kR2;
        float mSampleRadius = 10.f;

        int mSpatialSampleCount = 4;
        bool mEnableVisibilitySimilarityRejection = false;

        uint32_t mSpatialMISMethod = kMISTalbot;

        /////////////////////
        /// Final Shading
        /////////////////////
        int mFinalLightSamples = 1;
        int mFinalVisibilitySamples = 1;
        uint32_t mFinalVisibilityTrackingMethod = kAnalyticTracking;
        uint32_t mFinalLightTrackingMethod = kAnalyticTracking;
        uint32_t mFinalRandomSamplerType = kR2;
        float mFinalTStepScale = 0.2f;
    } mParams;

    private:

    bool hasExternalDict = false;

    /////////////////////
    /// Internal Variables
    /////////////////////
    int mTemporalSampleAccumulated = 0;

    float4x4 mPrevViewMat;
    float4x4 mPrevProjMat;
    float3 mPrevCameraU, mPrevCameraV, mPrevCameraW, mPrevCameraPosW;

    // camera animation
    float3 mBackedupCameraPosition;
    float3 mBackedupCameraTarget;
    float3 mBackedupCameraPosition2;
    float3 mInitialCameraPosition;
    float3 mInitialCameraTarget;
    int mCameraFramesMoved = 0;
    float mCameraMoveScale = 0.3f;
    int mCameraFrameInterval = 50;
    float mCameraForwardScale = 1.f;
    uint32_t mCameraAnimationMode = 0;
    int mCameraPauseInterval = 150;
    int mCameraShakeRoundsBeforePause = 3;
    int mCameraShakeTotalRounds = 1;

    // rotating around model
    int mCameraRotationFrames = 360;
    float mCameraRotationSpeed = 1; // degrees per frame
    float mCameraRotationDistance = 0; // if <= 0, use initial distance to the center of the volume
    bool mEnableCameraRotation = false;

    bool mOptionsChanged = true;
    bool mRandomizeFrameSpeed = false;

    // light animation
    bool mLastAnimateEnvLight = false;
    bool mAnimateEnvLight = false;
    double mAnimationStartTime = 0;
    int mAnimationFrameCount = 0;
    int mAnimationFreezedFrame = -1;
    float3 mSavedEnvMapRotation;

    // extra volume desc control
    float volumeAnisotropyExtraControl = -1.f;
	float volumeDensityScaleExtraControl = -1.f;
    float volumeAlbedoExtraControl = -1.f;

    float mEnvLightRotationSpeed = 0.1f;

    // Volume animation
    bool mSavedVDBAnimationState = false;

    int mVolumeAnimationSelectedFrameId = -1; // this is for freezing animation

    bool mOutputMotionVec = false; // enable when using Optix 7.3

    bool mRequestRecreateVarsForEmissiveSampler = false;
    Shader::DefineList mDefaultDefines;

    std::vector<float3> mPointLightUnitSpherePos;

    // Scripting
#define serialize(var) \
        if constexpr (!loadFromDict) dict[#var] = var; \
        else if (dict.keyExists(#var)) { if constexpr (std::is_same<decltype(var), std::string>::value) var = (const std::string &)dict[#var]; else var = dict[#var]; vars.emplace(#var); }

    template<bool loadFromDict, typename DictType>
    void serializePass(DictType& dict)
    {
        std::unordered_set<std::string> vars;

        // Add variables here that should be serialized to/from the dictionary.
        serialize(mParams);
        serialize(mCameraMoveScale);
        serialize(mCameraForwardScale);
        serialize(mCameraFrameInterval);
        serialize(mCameraPauseInterval);
        serialize(mCameraShakeTotalRounds);
        serialize(mCameraShakeRoundsBeforePause);

        serialize(mCameraAnimationMode);

        serialize(mAnimateEnvLight);
        serialize(mAnimationFreezedFrame);
        serialize(mEnvLightRotationSpeed);
        serialize(mFreezeFrame);
		serialize(mVolumeAnimationSelectedFrameId);
		serialize(mEmissiveSamplerTypeId);
		serialize(volumeDensityScaleExtraControl);
        serialize(volumeAlbedoExtraControl);
        serialize(volumeAnisotropyExtraControl);
        serialize(mOutputMotionVec);

        if constexpr (loadFromDict)
        {
            for (const auto& [key, value] : dict)
            {
                if (vars.find(key) == vars.end()) logWarning("Unknown field '" + key + "' in a PathTracer dictionary");
            }
        }
    }
#undef serialize

};

#pragma once
#include "Utils/HostDeviceShared.slangh"
BEGIN_NAMESPACE_FALCOR

//#define PRECOMPUTE_P_PARTIAL

#ifdef HOST_CODE
#include "Falcor.h"
#define half short
#define half2 int
#endif

#if defined(HOST_CODE) || defined(DEVICE_NEED_INCLUDE)
// screen space reservoir that samples primary collision depth and optionally primary scatter direction

struct Reservoir
{
    float runningSum; // can be half
    float M; // can be half
    float depth; 
    float p_y; // can be half
    float2 lightUV; //incoming direction
    int lightID; // scene light ID, currently only supports environmental light and directional light   (-2 means sampling self emission)
    // approxPHat assumes that T remains unchanged
    int sampledPixel; // pack into one int  // pixel that provides the sample
#ifndef HOST_CODE
#if MAX_BOUNCES > 1
    int extraBounceStartId; // can be inferred, to remove
#ifdef VERTEX_REUSE
    float p_partial; // can be half
#endif
#endif
#endif

#ifdef VERTEX_REUSE
#ifndef HOST_CODE
    [mutating]
#endif
    void SetPPartial(float input) { p_partial = input; }
    float GetPPartial() { return p_partial; }
#else
    void SetPPartial(float input) { }
    float GetPPartial() { return 1.f; }
#endif
};


#ifdef HOST_CODE
#undef half
#undef half2 
#endif

struct ExtraBounceReservoir
{
    float3 wi_dist; // convert into float3
};

struct MBMRSampleInfo
{
    int sampledPixel;
    int lightID;
    float2 lightUV;
    float p_y;
    float runningSum;
};

struct ReservoirFeatures
{
    int noReflectiveSurface;
    float transmittance;
};

struct VBufferItem
{
    int meshInstanceID;
    int primitiveID;
    float2 barycentrics;
    float rayT;
};

// params for R2 quasi-random sampling
struct R2Params
{
#ifdef HOST_CODE
    void setShaderData(ShaderVar const& var)
    {
        var["reservoirCount"] = reservoirCount;
        var["sampleCountUpperBound"] = sampleCountUpperBound;
        var["numSpatialReuseTotalRounds"] = numSpatialReuseTotalRounds;
        var["modFrames"] = modFrames;
    }
#endif

    int reservoirCount;
    int sampleCountUpperBound;
    int numSpatialReuseTotalRounds;
    int modFrames;
};

struct SamplingOptions
{
#ifdef HOST_CODE
    using SharedPtr = std::shared_ptr<SamplingOptions>;

    void setShaderData(ShaderVar const& var)
    {
        var["visibilityTrackingMethod"] = visibilityTrackingMethod;
        var["lightingTrackingMethod"] = lightingTrackingMethod;
        var["visibilitySamples"] = visibilitySamples;
        var["visibilityMipLevel"] = visibilityMipLevel;
        var["lightSamples"] = lightSamples;
        var["lightingMipLevel"] = lightingMipLevel;
        var["visibilityUseLinearSampler"] = visibilityUseLinearSampler;
        var["lightingUseLinearSampler"] = lightingUseLinearSampler;
        var["visibilityTStepScale"] = visibilityTStepScale;
        var["lightingTStepScale"] = lightingTStepScale;
        var["useEnvironmentLights"] = useEnvironmentLights;
        var["useAnalyticLights"] = useAnalyticLights;
        var["useEmissiveLights"] = useEmissiveLights;
        var["vertexReuseStartBounce"] = vertexReuseStartBounce;
    }
#endif

    uint visibilityTrackingMethod;
    uint lightingTrackingMethod;
    int lightSamples;
    int lightingMipLevel;
    int visibilitySamples;
    int visibilityMipLevel;
    bool visibilityUseLinearSampler;
    bool lightingUseLinearSampler;
    float visibilityTStepScale;
    float lightingTStepScale;
    bool useEnvironmentLights;
    bool useAnalyticLights;
    bool useEmissiveLights;
    int vertexReuseStartBounce;
};

#endif

END_NAMESPACE_FALCOR

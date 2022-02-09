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

__exported import ReSTIRHelper;
import ComputeInitialSample;
#include "HostDeviceSharedDefinitions.h"

import Utils.Debug.PixelDebug;
import VolumePathTracingFunctions;

cbuffer CB
{
    uint2   gResolution;
    int     gFrameCount;
    int     gNumTotalRounds;
    bool gUseReference;
    int  gMaxBounces;
    int     gBaselineSamplePerPixel;
    int     gNumInitialSamples;
    bool    gUseRussianRoulette;
    bool    gNoReuse;
    bool    gUseCoarserGridForIndirectBounce;
    EnvMapSampler           envMapSampler;
    EmissiveLightSampler           emissiveSampler;
    SamplingOptions gInitialSamplingOptions;
    SamplingOptions gSpatialSamplingOptions;
}

RWTexture2D<float4> gOutputColor;
RWStructuredBuffer<Reservoir> gOutputReservoirs;
RWStructuredBuffer<ExtraBounceReservoir> gOutputExtraBounceReservoirs;
StructuredBuffer<ReservoirFeatures> gReservoirFeatureBuffer;
#ifdef SURFACE_SCENE
StructuredBuffer<VBufferItem> gVBuffer;
#endif

static const int GROUP_SIZE_1D = 16;

[numthreads(GROUP_SIZE_1D, GROUP_SIZE_1D, 1)]
void main(uint3 DTid : SV_DispatchThreadID, uint3 GTId : SV_GroupThreadID, uint GroupIndex : SV_GroupIndex, uint3 GroupId : SV_GroupID)
{
    uint2 launchIndex = DTid.xy;
    //printSetPixel(launchIndex);

    SampleGenerator sg = SampleGenerator.create(launchIndex, gNumTotalRounds * gFrameCount);

    float3 rayOrigin = gScene.camera.getPosition();

    if (!IsWithinRange(launchIndex, gResolution)) return;

    float3 avgL = 0.f;
    float3 L = 0.f;

    int reservoirId = launchIndex.y * gResolution.x + launchIndex.x;

    float3 rayDir = normalize(gScene.camera.computeNonNormalizedRayDirPinhole(launchIndex, gResolution));

    Ray ray = { rayOrigin, rayDir, 0.f, kRayTMax };

    if (gUseReference)
    {
        #ifdef SURFACE_SCENE
        VBufferItem vItem = gVBuffer[reservoirId];
        HitInfo hitInfo = { vItem.meshInstanceID, vItem.primitiveID, vItem.barycentrics, vItem.rayT };
        #endif

        for (int rId = 0; rId < gBaselineSamplePerPixel; rId++)
        {
            L = IntegrateByVolumePathTracing(ray, sg, gInitialSamplingOptions.visibilityUseLinearSampler, max(1, gInitialSamplingOptions.lightSamples), gInitialSamplingOptions.useEnvironmentLights, gInitialSamplingOptions.useAnalyticLights, gInitialSamplingOptions.useEmissiveLights,
                envMapSampler, emissiveSampler,
                #ifdef SURFACE_SCENE
                hitInfo, 
                #endif
                0, gMaxBounces, true, gUseRussianRoulette);
            avgL += L;
        }
        gOutputColor[launchIndex] = float4(avgL / gBaselineSamplePerPixel, 1);
    }
    else
    {
        #if MAX_BOUNCES > 1
        ExtraBounceReservoir finalExtrabounceReservoir[MAX_BOUNCES - 1];
        #endif
        Reservoir finalReservoir = createNewReservoir();

        int intialSampleRounds = (gNumInitialSamples + 3) / 4;

        #ifdef SURFACE_SCENE
        // use vbuffer
        VBufferItem vItem = gVBuffer[reservoirId];
        HitInfo hitInfo = { vItem.meshInstanceID, vItem.primitiveID, vItem.barycentrics, vItem.rayT };

        // or trace a new ray
        //HitInfo hitInfo = FindSurfaceHit(ray.origin, ray.dir, 0, kRayTMax);

        ray.tMax = hitInfo.rayT;
        #endif

        bool noReuse = gNoReuse;

        // we generate at most 4 initial samples (in one tracking) per round
        for (int roundId = 0; roundId < intialSampleRounds; roundId++)
        {
            float hitDistances[MaxInitialSampleCount];
            float pdfDists[MaxInitialSampleCount];
            float outTrs[MaxInitialSampleCount];

            int roundSamples = roundId == intialSampleRounds - 1 ? (gNumInitialSamples - 4 * (intialSampleRounds - 1)) : 4;

            if (!noReuse)
                SampleMediumAnalyticGeneric(ray, sg, gInitialSamplingOptions.visibilityUseLinearSampler, hitDistances, gInitialSamplingOptions.visibilityMipLevel, pdfDists, outTrs, roundSamples);

            for (int sampleId = 0; sampleId < roundSamples; sampleId++)
            {
                #if MAX_BOUNCES > 1
                ExtraBounceReservoir extrabounceReservoir[MAX_BOUNCES - 1];
                #endif
                Reservoir outReservoir = ComputeInitialSample(ray, hitDistances[sampleId], pdfDists[sampleId], outTrs[sampleId], gMaxBounces, gNumInitialSamples,
                    sg, gInitialSamplingOptions, envMapSampler, emissiveSampler, gUseCoarserGridForIndirectBounce, gUseRussianRoulette, noReuse
                    #ifdef SURFACE_SCENE
                    , hitInfo
                    #endif
                    #if MAX_BOUNCES > 1
                    , extrabounceReservoir
                    #endif
                );

                bool isSelected = simpleResampleStep(outReservoir, finalReservoir, sg);

                if (isSelected)
                {
                    #if MAX_BOUNCES > 1
                    int maxIndirectBounces = decodeMaxIndirectBounces(finalReservoir.sampledPixel);
                    for (int bounceId = 0; bounceId < maxIndirectBounces; bounceId++)
                    {
                        finalExtrabounceReservoir[bounceId] = extrabounceReservoir[bounceId];
                    }
                    #endif
                }

            }
        }

        ArrayArrayDataProvider curDataProvider;
#if MAX_BOUNCES > 1
        curDataProvider.data = finalExtrabounceReservoir;
#endif
        SingleExternalVisibilityProvider externalVis;
        externalVis.Initialize();

        float p_hat = evaluate_P_hat(ray, sg, envMapSampler, emissiveSampler, VBUFFERITEM curDataProvider,
            gSpatialSamplingOptions, finalReservoir, externalVis, false, true, false);

        if (finalReservoir.runningSum > 0.f)
        {
            finalReservoir.runningSum *= finalReservoir.p_y == 0.f ? 0.f : float(p_hat) / finalReservoir.p_y;
            finalReservoir.p_y = p_hat;
        }


        {
            #if MAX_BOUNCES > 1
            finalReservoir.extraBounceStartId = reservoirId * (MAX_BOUNCES - 1);
            #endif

            gOutputReservoirs[reservoirId] = finalReservoir;
            #if MAX_BOUNCES > 1
            int maxIndirectBounces = decodeMaxIndirectBounces(finalReservoir.sampledPixel);
            for (int bounceId = 0; bounceId < maxIndirectBounces; bounceId++)
            {
                gOutputExtraBounceReservoirs[finalReservoir.extraBounceStartId + bounceId] = finalExtrabounceReservoir[bounceId];
            }
            #endif
        }
    }
}

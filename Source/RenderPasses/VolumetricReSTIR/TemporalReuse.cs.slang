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

 /** Temporal accumulation render pass.

     There are entry points for each of the three supported accumulation modes.
     Note that for the compensated summation mode the shader _must_ be compiled
     for precise floating-point operations (no reordering).

     In all modes, the shader writes the current accumulated average to the
     output texture. The intermediate buffers are internal to the pass.
 */

#include "Utils/Math/MathConstants.slangh"
//static const float kRayTMax = FLT_MAX;
__exported import ReSTIRHelper;
#include "HostDeviceSharedDefinitions.h"
import Utils.Debug.PixelDebug;

cbuffer CB
{
    uint2   gResolution;
    int     gFrameCount;
    int     gNumTotalRounds;
    int     gRoundOffset;
    float   gTemporalHistoryThreshold;
    bool    gIsFirstFrame;
    uint    gMISMethod;
    float4x4 gPrevViewMat;
    float4x4 gPrevProjMat;
    int     gReprojectionMipLevel;
    uint    gReprojectionMode;
    bool    gOutputMotionVec;
    float3  gPrevCameraU;
    float3  gPrevCameraV;
    float3  gPrevCameraW;
    float3  gPrevCameraPosW;
    SamplingOptions gSamplingOptions;
    EnvMapSampler           envMapSampler;      ///< Environment map sampler.
    EmissiveLightSampler           emissiveSampler;  
}

// Input data to accumulate and accumulated output.
RWStructuredBuffer<Reservoir> gCurReservoirs;
RWStructuredBuffer<Reservoir> gTemporalReservoirs;
RWStructuredBuffer<ExtraBounceReservoir> gCurExtraBounceReservoirs;
RWStructuredBuffer<ExtraBounceReservoir> gTemporalExtraBounceReservoirs;
StructuredBuffer<ReservoirFeatures> gReservoirFeatureBuffer;
StructuredBuffer<ReservoirFeatures> gTemporalReservoirFeatureBuffer;
RWTexture2D<float2> gMotionVec;
#ifdef SURFACE_SCENE
StructuredBuffer<VBufferItem> gVBuffer;
StructuredBuffer<VBufferItem> gTemporalVBuffer;
#endif

[numthreads(16, 16, 1)]
void main(uint3 DTid : SV_DispatchThreadID)
{
    RWStructuredBufferArrayDataProvider curDataProvider;
    curDataProvider.data = gCurExtraBounceReservoirs;
    RWStructuredBufferArrayDataProvider tempDataProvider;
    tempDataProvider.data = gTemporalExtraBounceReservoirs;

    const int2 pixelPos = DTid.xy;

    if (any(DTid.xy >= gResolution)) return;
    printSetPixel(pixelPos);

    if (!gIsFirstFrame)
    {
        SampleGenerator sg = SampleGenerator.create(DTid.xy, gNumTotalRounds * gFrameCount + gRoundOffset); // prevent correlation with ray traicng shaders

        int selectedId = -1;
        float selectedSamplePy = 0.f;

        int pixelId = getPixelID(pixelPos, gResolution);

        // temporal reprojection using motion vectors

        int numUsedReservoirs = 1;
        Reservoir taps[2];
        taps[0] = gCurReservoirs[pixelId];

        CameraRay cameraRay = gScene.camera.computeRayPinholeScaled(pixelPos, 1, 0.5f, gResolution);

        Ray ray = { cameraRay.origin, cameraRay.dir, 0, kRayTMax };

        Reservoir output;
        if (gMISMethod == kMISTalbot) output = createNewReservoir();
        else output = taps[0];
        selectedSamplePy = output.p_y;

    #if MAX_BOUNCES > 1
        int centerExtraBounceStartId = taps[0].extraBounceStartId;
    #endif
        // for background samples, find a random reprojection point

        float temporalOriginalDepth = 0.f;
        int2 reprojScreenPos = 0;

        ReservoirFeatures centerFeatures = gReservoirFeatureBuffer[pixelId];
        bool isBackgroundReservoir = centerFeatures.transmittance == 1.f && centerFeatures.noReflectiveSurface;

        bool useFallbackReservoir = true;

        if (gReprojectionMode != kReprojectionNone)
        {
            float reprojDepth = taps[0].depth;
            bool reprojectUsingVolumeSample = true;

            if (reprojDepth == kRayTMax && gReprojectionMode != kReprojectionNoBackground && !isBackgroundReservoir)
            {
            #ifdef SURFACE_SCENE
                VBufferItem vItem = gVBuffer[pixelId];
                HitInfo hitInfo = { vItem.meshInstanceID, vItem.primitiveID, vItem.barycentrics, vItem.rayT };

                // if we have reflective surfaces, they also need temporal resampling to reduce noise.
                // so we randomly decide using surface or volume (velocity resampling) by total transmittance
                if (!centerFeatures.noReflectiveSurface)
                {
                    reprojDepth = hitInfo.rayT;
                    // are we almost surface or almost volume?
                    if (sampleNext1D(sg) < centerFeatures.transmittance)
                    {
                        reprojectUsingVolumeSample = false;
                    }
                }

                if (reprojectUsingVolumeSample)
            #endif
                {
                    // velocity resampling
                    reprojDepth = RejectionSampleRandomPointByDensity(ray, sg, kNumMaxMips + gReprojectionMipLevel); // always use conservative grid!
                }
            }

            float3 p = cameraRay.origin + cameraRay.dir * reprojDepth;

            // find last frame world position if having velocity grid
            if (gScene.volumeDesc.hasVelocity && gScene.volumeDesc.hasAnimation && reprojectUsingVolumeSample && gReprojectionMode != kReprojectionLinearNoVelocity)
            {
                float3 v = VelocityWorld(p) *  gScene.volumeDesc.velocityScale;
                p = p - v;
            }

            float4 viewPos = mul(float4(p, 1.f), gPrevViewMat);
            float4 clipPos = mul(viewPos, gPrevProjMat);

            float2 scrPos = clipPos.xy / clipPos.w;
            int2 scrPosI; 

            if (reprojDepth == kRayTMax)
            {
                scrPos = pixelPos + 0.5f;
                scrPosI = pixelPos;
            }
            else
            {
                scrPos.x = 0.5 * scrPos.x + 0.5;
                scrPos.y = -0.5 * scrPos.y + 0.5;
                scrPos *= gResolution;
                scrPosI = int2(scrPos);
            }

            {
                int scrPosI_Id = getPixelID(scrPosI, gResolution);

                ReservoirFeatures tapFeatures = gTemporalReservoirFeatureBuffer[scrPosI_Id];

                bool isTapBackgroundReservoir = tapFeatures.transmittance == 1.f && tapFeatures.noReflectiveSurface;
                if (isBackgroundReservoir && !isTapBackgroundReservoir)
                {
                    if (gOutputMotionVec) gMotionVec[pixelPos] = (reprojScreenPos - pixelPos) / float2(gResolution);
                    return;
                }
            }

            {
                scrPosI = int2(scrPos);
                reprojScreenPos = scrPosI;

                if (IsWithinRange(scrPosI, gResolution))
                {
                    numUsedReservoirs++;
                    taps[1] = gTemporalReservoirs[getPixelID(scrPosI, gResolution)];
                }
            }

            if (numUsedReservoirs > 1) useFallbackReservoir = false;
        }

        if (useFallbackReservoir)
        {
            numUsedReservoirs++;
            reprojScreenPos = pixelPos;
            taps[1] = gTemporalReservoirs[pixelId];
        }

        if (gOutputMotionVec) gMotionVec[pixelPos] = (reprojScreenPos - pixelPos) / float2(gResolution);

        float curM = taps[0].M;
        float MaxPrevM = gTemporalHistoryThreshold * curM;

        // convert depths to current frame depths
        if (numUsedReservoirs == 2)
        {
            temporalOriginalDepth = taps[1].depth;
            if (taps[1].depth != kRayTMax)
            {
                int2 tapPos = reprojScreenPos;
                float3 worldPos = gScene.camera.computeWorldPositionWithRayTracingFrameScaled(tapPos, 1,
                                        0.5f, gResolution, gPrevCameraPosW, gPrevCameraU, gPrevCameraV, gPrevCameraW, taps[1].depth);

                taps[1].depth = length(worldPos - ray.origin);
            }
        }

        float centerPrevFrameDepth = taps[0].depth;
        if (centerPrevFrameDepth != kRayTMax)
        {
            float3 worldPos = ray.at(centerPrevFrameDepth);
            centerPrevFrameDepth = length(worldPos - gPrevCameraPosW);
        }
        //

        bool hasSelection = output.runningSum > 0.f;

        int startSampleId = gMISMethod == kMISTalbot ? 0 : 1;
        if (startSampleId == 1) selectedId = 0;

        for (int i = startSampleId; i < numUsedReservoirs; i++)
        {
            // if not using initial RIS, use Talbot in temporal causes noise
            float talbotMISWeight = 1.f;

            float neighbor_py = 0.f;

            if (taps[i].p_y > 0.f)
            {
                int2 resampleRayPixelPos = pixelPos;

                Ray resampleRay = ray;

                neighbor_py = taps[i].p_y;

                if (isnan(taps[i].runningSum) || isinf(taps[i].runningSum)) taps[i].runningSum = 0.f;

                // don't need to resample self for talbot
                if (i > 0)
                {
                    #ifdef SURFACE_SCENE
                    VBufferItem vItem = gVBuffer[getPixelID(resampleRayPixelPos, gResolution)];
                    #endif
                    // tap's runningSum and p_y are overwritten!
                    // for vertex reuse: p_partial is also overwritten!
                    bool isValid = resampleNeighbor(taps[i], resampleRay, sg, envMapSampler, emissiveSampler, VBUFFERITEM tempDataProvider, gSamplingOptions);
                    if (!isValid) continue;
                }
            }
            else
            {
                taps[i].p_y = 0.f;
                taps[i].runningSum = 0.f;
            }

            if (gMISMethod == kMISTalbot && taps[i].runningSum > 0.f)
            {
                float p_sum = 0;
                float p_qi = 0;
                float k = 0;
            
                for (int j = 0; j < numUsedReservoirs; j++)
                {
                    int2 tapPos2 = int2(j == 0 ? pixelPos.x : reprojScreenPos.x, j == 0 ? pixelPos.y : reprojScreenPos.y);
                    float correctedM = min(MaxPrevM, taps[j].M);
            
                    k += correctedM;

                    if (j == 0)
                    {
                        p_qi = taps[i].p_y; // already resampled at center pixel!
                        p_sum += taps[i].p_y * correctedM;
                    }
                    else if (i == j)
                    {
                        p_qi = neighbor_py; // this is the original p_y!
                        p_sum += neighbor_py * correctedM;
                    }
                    else
                    {
                        int2 neighborRayPixelPos = tapPos2;

                        CameraRay neighborCameraRay;
                        if (j == 0) neighborCameraRay = gScene.camera.computeRayPinholeScaled(tapPos2, 1, 0.5, gResolution);
                        else neighborCameraRay = gScene.camera.computeRayPinholeWithRayTracingFrameScaled(tapPos2, 1, 0.5, gResolution, gPrevCameraPosW, gPrevCameraU, gPrevCameraV, gPrevCameraW);
            
                        float usedDepth = j == 0 ? taps[i].depth : (i == 0 ? centerPrevFrameDepth : temporalOriginalDepth);
            
                        Ray neighborRay = { neighborCameraRay.origin, neighborCameraRay.dir, 0, usedDepth };
                        SingleExternalVisibilityProvider externalVis;
                        externalVis.Initialize();

                        #ifdef SURFACE_SCENE
                        VBufferItem vItem;
                        if (j == 0) vItem = gVBuffer[getPixelID(neighborRayPixelPos, gResolution)];
                        else vItem = gTemporalVBuffer[getPixelID(neighborRayPixelPos, gResolution)];
                        #endif

                        float backupDepth = taps[i].depth;
                        taps[i].depth = usedDepth;
                        float p_y;

                        if (i == 0) p_y = evaluatePHatReadOnly(taps[i], neighborRay, sg, envMapSampler, emissiveSampler,
                            VBUFFERITEM curDataProvider, gSamplingOptions, j > 0, true, false);
                        else p_y = evaluatePHatReadOnly(taps[i], neighborRay, sg, envMapSampler, emissiveSampler,
                            VBUFFERITEM tempDataProvider, gSamplingOptions, j > 0, true, false);

                        taps[i].depth = backupDepth;
                        if (isinf(p_y) || isnan(p_y)) p_y = 0.f;
                        p_sum += p_y * correctedM;
                    }
                }
            
                if (p_sum > 0)
                    talbotMISWeight = p_qi * k / p_sum;
            }

            taps[i].runningSum *= float(talbotMISWeight);

            bool isCurrentSelected = simpleResampleStepWithMaxM(taps[i], MaxPrevM, output, sg);
            hasSelection |= isCurrentSelected;
            if (isCurrentSelected)
            {
                selectedSamplePy = neighbor_py;
                selectedId = i;
            }
        }

    #if MAX_BOUNCES > 1
        if (hasSelection && selectedId > 0)
        {
            // copy the extra bounce reservoirs
            int maxIndirectBounces = decodeMaxIndirectBounces(output.sampledPixel);

            for (int bounceId = 0; bounceId < maxIndirectBounces; bounceId++)
            {
                gCurExtraBounceReservoirs[centerExtraBounceStartId + bounceId] = gTemporalExtraBounceReservoirs[output.extraBounceStartId + bounceId];
            }
        }
        output.extraBounceStartId = centerExtraBounceStartId;
    #endif

        gCurReservoirs[pixelId] = output;

    }
}

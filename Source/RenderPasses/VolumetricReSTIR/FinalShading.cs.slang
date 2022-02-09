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
__exported import ReSTIRHelper;
#include "HostDeviceSharedDefinitions.h"
import Utils.Debug.PixelDebug;
import Utils.Math.MathHelpers;
#include "Utils/Math/MathConstants.slangh"

cbuffer CB
{
    uint2   gResolution;
    int     gNumTotalRounds;
    int     gFrameCount;
    bool    gTemporalReuse;
    bool    gSpatialReuse;
    bool    gUseReference;
    int     gMaxBounces;
    bool    gVisualizeTotalTransmittance;
    bool    gNoReuse;
    SamplingOptions gSamplingOptions;
    EnvMapSampler           envMapSampler; 
    EmissiveLightSampler    emissiveSampler; 
}
// Input data to accumulate and accumulated output.
StructuredBuffer<Reservoir> gCurReservoirs;
StructuredBuffer<ExtraBounceReservoir> gCurExtraBounceReservoirs;
StructuredBuffer<ReservoirFeatures> gReservoirFeatureBuffer;
RWTexture2D<float4> gCurrentColors;
RWTexture2D<float4> gOutputFrame;
#ifdef SURFACE_SCENE
StructuredBuffer<VBufferItem> gVBuffer;
#endif

import ComputeInitialSample;

[numthreads(16, 16, 1)]
void main(uint3 DTid : SV_DispatchThreadID)
{
    StructuredBufferArrayDataProvider dataProvider;
    dataProvider.data = gCurExtraBounceReservoirs;

    uint2 pixelPos = DTid.xy;
    //printSetPixel(pixelPos);

    if (any(DTid.xy >= gResolution)) return;

    uint pixelId = pixelPos.y * gResolution.x + pixelPos.x;

    float3 outputColor = 0;

    SampleGenerator sg = SampleGenerator.create(pixelPos, gNumTotalRounds * gFrameCount + gNumTotalRounds - 1);

    if (gUseReference) // output raw
    {
        outputColor = gCurrentColors[pixelPos].rgb;
    }
    else if (gVisualizeTotalTransmittance)
    {
        outputColor = pow(gReservoirFeatureBuffer[pixelId].transmittance, 2.2f);
    }
    else
    {
        float3 rayOrigin = gScene.camera.getPosition();
        float3 rayDir = normalize(gScene.camera.computeNonNormalizedRayDirPinhole(DTid.xy, gResolution));
        Ray ray = { rayOrigin, rayDir, 0, kRayTMax };

        Reservoir curReservoir = gCurReservoirs[pixelId];

        if (curReservoir.runningSum > 0.f)
        {
            //// background
            float3 rayOrigin = gScene.camera.getPosition();
            float3 rayDir = normalize(gScene.camera.computeNonNormalizedRayDirPinholeScaled(DTid.xy, 1, 0.5f, gResolution));

            Ray ray = { rayOrigin, rayDir, 0, curReservoir.depth };

            float3 curOutputColor;

            #ifdef SURFACE_SCENE
            VBufferItem vItem = gVBuffer[pixelId];
            #endif

            curOutputColor = evaluate_F(curReservoir, VBUFFERITEM dataProvider, envMapSampler, emissiveSampler, ray, sg, gSamplingOptions, gNoReuse);

            float W = curReservoir.p_y == 0.0 ? 1.f : curReservoir.runningSum / (curReservoir.p_y * curReservoir.M);

            //print(curOutputColor);
            //print(curReservoir.runningSum);
            //print(curReservoir.p_y);
            //print(curReservoir.M);
            //print(W);

            curOutputColor *= W;

            outputColor += curOutputColor;
        }
    }

    float4 curColor = float4(outputColor, 1);

    pixelPos = DTid.xy;

    if (any(isnan(curColor) || isinf(curColor))) curColor = 0.f;

    gOutputFrame[pixelPos] = curColor;
}



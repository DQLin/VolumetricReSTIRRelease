
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
#include "Utils/Math/MathConstants.slangh"
__exported import ReSTIRHelper;
#include "HostDeviceSharedDefinitions.h"
import Utils.Helpers;
import Utils.Math.MathHelpers;
import Utils.Debug.PixelDebug;

Texture2D<float4> gInputColors;
RWTexture2D<float4> gOutputColors;
StructuredBuffer<Reservoir> gInputReservoirs;
RWStructuredBuffer<Reservoir> gOutputReservoirs;
StructuredBuffer<ReservoirFeatures> gReservoirFeatureBuffer;
StructuredBuffer<ExtraBounceReservoir> gInputExtraBounceReservoirs;
RWStructuredBuffer<ExtraBounceReservoir> gOutputExtraBounceReservoirs;
#ifdef SURFACE_SCENE
StructuredBuffer<VBufferItem> gVBuffer;
#endif

cbuffer CB
{
    uint2   gResolution;
    int     gFrameCount;
    int     gRoundId;
    int     gNumRounds;
    int     gRoundOffset;
    uint    gMISMethod;
    float   gSampleRadius;
    int     gSampleCount;
    int     gRandomSamplerType;
    R2Params gR2Params;
    SamplingOptions gSamplingOptions;
    EnvMapSampler           envMapSampler;      ///< Environment map sampler.
    EmissiveLightSampler           emissiveSampler;   
}

int2 generateNeighborOffset(int sampleId, int sampleCount, float sampleRadius, int pixelId, int frameId)
{
    int2 offset;
    float2 u;
    if (gRandomSamplerType == kHammersley)
    {
        u = getHammersley(sampleId, sampleCount);
    }
    else// R2
    {
        double multiplier = frameId * gR2Params.sampleCountUpperBound + sampleId;
        u = sampleId == 0 ? float2(0, 0) : float2(frac(double2(0.754877669, 0.569840296) * multiplier));
    }

    offset = int2(sampleRadius * sample_disk(u));

    return offset;
}

#if MAX_BOUNCES > 1
void writeExtraBounceReservoirs(in Reservoir output, int outputStartId, int inputStartId)
{
    int maxIndirectBounces = decodeMaxIndirectBounces(output.sampledPixel);
    for (int bounceId = 0; bounceId < maxIndirectBounces; bounceId++)
    {
        gOutputExtraBounceReservoirs[outputStartId + bounceId] = gInputExtraBounceReservoirs[inputStartId + bounceId];
    }
}
#endif

[numthreads(16, 16, 1)]
void main(uint3 DTid : SV_DispatchThreadID)
{
    StructuredBufferArrayDataProvider dataProvider;
    dataProvider.data = gInputExtraBounceReservoirs;

    const int2 pixelPos = DTid.xy;

    //printSetPixel(pixelPos);

    if (!IsWithinRange(pixelPos, gResolution)) return;

    int numRounds = gNumRounds;
    int roundId = gRoundId;

    int r2TimeSeed = ((gR2Params.numSpatialReuseTotalRounds + 1) * gFrameCount + roundId) % gR2Params.modFrames;

    numRounds += gRoundOffset + 1;
    roundId += gRoundOffset;

    SampleGenerator sg = SampleGenerator.create(DTid.xy, numRounds * gFrameCount + roundId); // prevent correlation with ray traicng shaders

    const int pixelId = getPixelID(pixelPos, gResolution);

    bool IsSelfBackground = true;

    Reservoir output = gInputReservoirs[pixelId];

    #if MAX_BOUNCES > 1
    int centerExtraBounceStartId = output.extraBounceStartId;
    #endif

    CameraRay cameraRay = gScene.camera.computeRayPinholeScaled(pixelPos, 1, 0.5f, gResolution);

    Ray ray = { cameraRay.origin, cameraRay.dir, 0, kRayTMax };

    if (gMISMethod == kMISTalbot) output = createNewReservoir();

    bool hasSelection = output.runningSum > 0.f;

    ReservoirFeatures centerFeatures = gReservoirFeatureBuffer[pixelId];
    if (centerFeatures.transmittance != 1.f)
    {
        IsSelfBackground = false;
    }
    #ifdef SURFACE_SCENE
    if (!centerFeatures.noReflectiveSurface)
    {
        IsSelfBackground = false;
    }
    #endif

    if (IsSelfBackground)
    {
        if (gMISMethod == kMISTalbot) gOutputReservoirs[pixelId] = gInputReservoirs[pixelId];
        else gOutputReservoirs[pixelId] = output;

    #if MAX_BOUNCES > 1
        writeExtraBounceReservoirs(output, output.extraBounceStartId, output.extraBounceStartId);
    #endif
        return;
    }

    int startSampleId = gMISMethod == kMISTalbot ? 0 : 1;

    for (int sampleId = startSampleId; sampleId < gSampleCount; sampleId++)
    {
        int2 offset = generateNeighborOffset(sampleId, gSampleCount, gSampleRadius, pixelId, r2TimeSeed);
        int2 tapPos = pixelPos + offset;
        if (IsWithinRange(tapPos, gResolution))
        {
            // method 1
            // smart resampling
            int tapReservoirId = getPixelID(tapPos, gResolution);

            Reservoir tap = gInputReservoirs[tapReservoirId];

            float MISWeight = 1.f;

            float neighbor_py = tap.p_y;

            if (sampleId > 0) // don't need to resample self for talbot
            {
                int2 resampleRayPixelPos = pixelPos;

                Ray resampleRay = ray;

                #ifdef SURFACE_SCENE
                VBufferItem vItem = gVBuffer[getPixelID(resampleRayPixelPos, gResolution)];
                #endif

                // tap's runningSum and p_y are overwritten!
                resampleNeighborSpatialReuse(tap, resampleRay, sg, envMapSampler, emissiveSampler, VBUFFERITEM dataProvider, gSamplingOptions);
            }

            if (gMISMethod == kMISTalbot && tap.runningSum > 0.f)
            {
                float p_sum = 0;
                float p_qi = 0;
                float k = 0;

                for (int j = 0; j < gSampleCount; j++)
                {
                    int2 offset2 = generateNeighborOffset(j, gSampleCount, gSampleRadius, pixelId, r2TimeSeed);
                    int2 tapPos2 = pixelPos + offset2;
                    if (!IsWithinRange(tapPos2, gResolution)) continue;
                    int tapReservoirId2 = getPixelID(tapPos2, gResolution);
                    Reservoir tap2 = gInputReservoirs[tapReservoirId2];

                    k += tap2.M;

                    if (j == 0)
                    {
                        p_qi = tap.p_y; // this tap's p_y resampled at center pixel
                        p_sum += tap.p_y * tap2.M;
                    }
                    else if (sampleId == j)
                    {
                        p_qi = tap2.p_y;
                        p_sum += tap2.p_y * tap2.M;
                    }
                    else
                    {
                        int2 neighborRayPixelPos = tapPos2;

                        float3 neighborRayDir = normalize(gScene.camera.computeNonNormalizedRayDirPinholeScaled(tapPos2, 1, 0.5f, gResolution));

                        Ray neighborRay = { ray.origin, neighborRayDir, 0, tap.depth };

                        SingleExternalVisibilityProvider externalVis;
                        externalVis.Initialize();

                        #ifdef SURFACE_SCENE
                        VBufferItem vItem = gVBuffer[getPixelID(neighborRayPixelPos, gResolution)];
                        #endif

                        float p_y = evaluatePHatReadOnly(tap, neighborRay, sg, envMapSampler, emissiveSampler, VBUFFERITEM dataProvider, gSamplingOptions, false, true, true);
                        if (isinf(p_y) || isnan(p_y)) p_y = 0.f;

                        p_sum += p_y * tap2.M;
                    }
                }

                if (p_sum > 0)
                    MISWeight = p_qi * k / p_sum;
            }

            tap.runningSum *= float(MISWeight);

            bool isSelected = simpleResampleStep(tap, output, sg);

            if (isSelected)
            {
                hasSelection = true;
            }
        }
    }

#if MAX_BOUNCES > 1
    if (hasSelection)
    {
        // copy the extra bounce reservoirs
        writeExtraBounceReservoirs(output, centerExtraBounceStartId, output.extraBounceStartId);
    }
#endif

#if MAX_BOUNCES > 1
    output.extraBounceStartId = centerExtraBounceStartId;
#endif
    gOutputReservoirs[pixelId] = output;

}

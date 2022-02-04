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
__exported import ReSTIRHelper;
#include "HostDeviceSharedDefinitions.h"
import Utils.Debug.PixelDebug;

cbuffer CB
{
    uint2   gResolution;
    bool    gUseReference;
    SamplingOptions gInitialSamplingOptions;
}

// Input data to accumulate and accumulated output.
RWStructuredBuffer<ReservoirFeatures> gReservoirFeatureBuffer;
#ifdef SURFACE_SCENE
RWStructuredBuffer<VBufferItem> gVBuffer;
#endif

[numthreads(16, 16, 1)]
void main(uint3 DTid : SV_DispatchThreadID)
{

    const int2 pixelPos = DTid.xy;
    //printSetPixel(pixelPos );

    int2 resolution = gResolution;
    if (any(DTid.xy >= resolution)) return;

    // sg is not important, since we are not using it anyway
    SampleGenerator sg;

    int pixelId = getPixelID(pixelPos, resolution);

    CameraRay cameraRay = gScene.camera.computeRayPinholeScaled(pixelPos, 1, 0.5f, gResolution);

    Ray ray = { cameraRay.origin, cameraRay.dir, 0, kRayTMax };

    float3 surfaceNormal = 0.f;

#ifdef SURFACE_SCENE
    HitInfo hitInfo = FindSurfaceHit(ray.origin, ray.dir, 0, kRayTMax);
    if (hitInfo.isValidHit())
    {
        ShadingData shadingInfo = computeSurfaceShadingInfo(hitInfo, -ray.dir);
        surfaceNormal = shadingInfo.N;
        if (any(shadingInfo.emissive > 0.f)) surfaceNormal = 0.f;
    }

    ray.tMax = hitInfo.rayT;
    VBufferItem vbufferItem = { hitInfo.meshInstanceID, hitInfo.primitiveIndex, hitInfo.barycentrics, hitInfo.rayT };
#endif

#ifdef SURFACE_SCENE
    gVBuffer[pixelId] = vbufferItem;
#endif

    if (!gUseReference)
    {
        float accuTransmittance;
        ReservoirFeatureRayMarchingGeneric(ray, sg, 0, true, gInitialSamplingOptions.visibilityTStepScale, accuTransmittance);
        ReservoirFeatures features = { int(all(surfaceNormal == 0.f)), accuTransmittance };
        gReservoirFeatureBuffer[pixelId] = features;
    }
}

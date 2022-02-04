/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
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
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
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
import Scene.Camera.CameraData;
import Utils.Math.MathHelpers;

struct CameraRay
{
    float3 origin;
    float tMin;
    float3 dir;
    float tMax;

    RayDesc toRayDesc()
    {
        return { origin, tMin, dir, tMax };
    }
};

struct Camera
{
    CameraData data;

    float3 getPosition() { return data.posW; }
    float4x4 getViewProj() { return data.viewProjMat; }
    float4x4 getView() { return data.viewMat; }

    /** Computes a camera ray for a given pixel assuming a pinhole camera model.
        The camera jitter is taken into account to compute the sample position on the image plane.
        \param[in] pixel Pixel coordinates with origin in top-left.
        \param[in] frameDim Image plane dimensions in pixels.
        \return Returns the camera ray.
    */
    CameraRay computeRayPinhole(uint2 pixel, uint2 frameDim)
    {
        CameraRay ray;

        // Compute the normalized ray direction assuming a pinhole camera.
        ray.origin = data.posW;
        ray.dir = normalize(computeNonNormalizedRayDirPinhole(pixel, frameDim));

        float invCos = 1.f / dot(normalize(data.cameraW), ray.dir);
        ray.tMin = data.nearZ * invCos;
        ray.tMax = data.farZ * invCos;

        return ray;
    }

    CameraRay computeRayPinholeScaled(uint2 pixel, int scale, uint2 frameDim)
    {
        float2 pixelPos = (pixel+0.5) * scale;

        CameraRay ray;

        // Compute the normalized ray direction assuming a pinhole camera.
        ray.origin = data.posW;

        ray.dir = normalize(computeNonNormalizedRayDirPinhole(int2(pixelPos), pixelPos - int2(pixelPos), frameDim));

        float invCos = 1.f / dot(normalize(data.cameraW), ray.dir);
        ray.tMin = data.nearZ * invCos;
        ray.tMax = data.farZ * invCos;

        return ray;
    }

    CameraRay computeRayPinholeScaled(uint2 pixel, int scale, float2 offset, uint2 frameDim)
    {
        float2 pixelPos = (pixel + offset) * scale;

        CameraRay ray;

        // Compute the normalized ray direction assuming a pinhole camera.
        ray.origin = data.posW;

        ray.dir = normalize(computeNonNormalizedRayDirPinhole(int2(pixelPos), pixelPos - int2(pixelPos), frameDim));

        float invCos = 1.f / dot(normalize(data.cameraW), ray.dir);
        ray.tMin = data.nearZ * invCos;
        ray.tMax = data.farZ * invCos;

        return ray;
    }

    CameraRay computeRayPinholeWithRayTracingFrame(uint2 pixel, uint2 frameDim, float3 posW, float3 cameraU, float3 cameraV, float3 cameraW)
    {
        CameraRay ray;

        // Compute the normalized ray direction assuming a pinhole camera.
        ray.origin = posW;
        ray.dir = normalize(computeNonNormalizedRayDirPinholeWithRayTracingFrame(pixel, frameDim, cameraU, cameraV, cameraW));

        float invCos = 1.f / dot(normalize(cameraW), ray.dir);
        ray.tMin = data.nearZ * invCos;
        ray.tMax = data.farZ * invCos;

        return ray;
    }

    CameraRay computeRayPinholeWithRayTracingFrameScaled(uint2 pixel, int scale, float2 offset, uint2 frameDim, float3 posW, float3 cameraU, float3 cameraV, float3 cameraW)
    {
        float2 pixelPos = (pixel + offset) * scale;

        CameraRay ray;

        // Compute the normalized ray direction assuming a pinhole camera.
        ray.origin = posW;
        ray.dir = normalize(computeNonNormalizedRayDirPinholeWithRayTracingFrame(uint2(pixelPos), pixelPos - int2(pixelPos), frameDim, cameraU, cameraV, cameraW));

        float invCos = 1.f / dot(normalize(cameraW), ray.dir);
        ray.tMin = data.nearZ * invCos;
        ray.tMax = data.farZ * invCos;

        return ray;
    }

    /////////////////////////////////
    float3 computeWorldPositionWithRayTracingFrame(uint2 pixel, uint2 frameDim, float3 posW, float3 cameraU, float3 cameraV, float3 cameraW, float depth)
    {
        float3 origin = posW;
        float3 dir = normalize(computeNonNormalizedRayDirPinholeWithRayTracingFrame(pixel, frameDim, cameraU, cameraV, cameraW));
        return origin + depth * dir;
    }

    float3 computeWorldPositionWithRayTracingFrameScaled(uint2 pixel, int scale, uint2 frameDim, float3 posW, float3 cameraU, float3 cameraV, float3 cameraW, float depth)
    {
        float2 pixelPos = (pixel + 0.5) * scale;
        float3 origin = posW;
        float3 dir = normalize(computeNonNormalizedRayDirPinholeWithRayTracingFrame(pixelPos, pixelPos - int2(pixelPos), frameDim, cameraU, cameraV, cameraW));
        return origin + depth * dir;
    }

    float3 computeWorldPositionWithRayTracingFrameScaled(uint2 pixel, int scale, float2 offset, uint2 frameDim, float3 posW, float3 cameraU, float3 cameraV, float3 cameraW, float depth)
    {
        float2 pixelPos = (pixel + offset) * scale;
        float3 origin = posW;
        float3 dir = normalize(computeNonNormalizedRayDirPinholeWithRayTracingFrame(pixelPos, pixelPos - int2(pixelPos), frameDim, cameraU, cameraV, cameraW));
        return origin + depth * dir;
    }
    /////////////////////////////////

    /** Computes the primary ray's direction, non-normalized assuming pinhole camera model.
        The camera jitter is taken into account to compute the sample position on the image plane.
        \param[in] pixel Pixel coordinates with origin in top-left.
        \param[in] frameDim Image plane dimensions in pixels.
        \return Returns the non-normalized ray direction
    */
    float3 computeNonNormalizedRayDirPinhole(uint2 pixel, uint2 frameDim)
    {
        // Compute sample position in screen space in [0,1] with origin at the top-left corner.
        // The camera jitter offsets the sample by +-0.5 pixels from the pixel center.
        float2 p = (pixel + float2(0.5f, 0.5f)) / frameDim + float2(-data.jitterX, data.jitterY);
        float2 ndc = float2(2, -2) * p + float2(-1, 1);

        // Compute the non-normalized ray direction assuming a pinhole camera.
        return ndc.x * data.cameraU + ndc.y * data.cameraV + data.cameraW;
    }


    float3 computeNonNormalizedRayDirPinhole(uint2 pixel, float2 offsets, uint2 frameDim)
    {
        // Compute sample position in screen space in [0,1] with origin at the top-left corner.
        // The camera jitter offsets the sample by +-0.5 pixels from the pixel center.
        float2 p = (pixel + offsets) / frameDim + float2(-data.jitterX, data.jitterY);
        float2 ndc = float2(2, -2) * p + float2(-1, 1);

        // Compute the non-normalized ray direction assuming a pinhole camera.
        return ndc.x * data.cameraU + ndc.y * data.cameraV + data.cameraW;
    }


    // no jitter is used
    float3 computeNonNormalizedRayDirPinholeWithRayTracingFrame(uint2 pixel, uint2 frameDim, float3 cameraU, float3 cameraV, float3 cameraW)
    {
        // Compute sample position in screen space in [0,1] with origin at the top-left corner.
        // The camera jitter offsets the sample by +-0.5 pixels from the pixel center.
        float2 p = (pixel + float2(0.5f, 0.5f)) / frameDim;
        float2 ndc = float2(2, -2) * p + float2(-1, 1);

        // Compute the non-normalized ray direction assuming a pinhole camera.
        return ndc.x * cameraU + ndc.y * cameraV + cameraW;
    }

    float3 computeNonNormalizedRayDirPinholeWithRayTracingFrame(uint2 pixel, float2 offsets, uint2 frameDim, float3 cameraU, float3 cameraV, float3 cameraW)
    {
        // Compute sample position in screen space in [0,1] with origin at the top-left corner.
        // The camera jitter offsets the sample by +-0.5 pixels from the pixel center.
        float2 p = (pixel + offsets) / frameDim;
        float2 ndc = float2(2, -2) * p + float2(-1, 1);

        // Compute the non-normalized ray direction assuming a pinhole camera.
        return ndc.x * cameraU + ndc.y * cameraV + cameraW;
    }
    float3 computeNonNormalizedRayDirPinholeScaled(uint2 pixel, int scale, uint2 frameDim)
    {
        float2 pixelPos = (pixel + 0.5) * scale;
        return computeNonNormalizedRayDirPinhole(int2(pixelPos), pixelPos - int2(pixelPos), frameDim);
    }

    float3 computeNonNormalizedRayDirPinholeScaled(uint2 pixel, int scale, float2 offset, uint2 frameDim)
    {
        float2 pixelPos = (pixel + offset) * scale;
        return computeNonNormalizedRayDirPinhole(int2(pixelPos), pixelPos - int2(pixelPos), frameDim);
    }

    /** Computes a camera ray for a given pixel assuming a thin-lens camera model.
        The camera jitter is taken into account to compute the sample position on the image plane.
        \param[in] pixel Pixel coordinates with origin in top-left.
        \param[in] frameDim Image plane dimensions in pixels.
        \param[in] u Uniform 2D sample.
        \return Returns the camera ray.
    */
    CameraRay computeRayThinlens(uint2 pixel, uint2 frameDim, float2 u)
    {
        CameraRay ray;

        // Sample position in screen space in [0,1] with origin at the top-left corner.
        // The camera jitter offsets the sample by +-0.5 pixels from the pixel center.
        float2 p = (pixel + float2(0.5f, 0.5f)) / frameDim + float2(-data.jitterX, data.jitterY);
        float2 ndc = float2(2, -2) * p + float2(-1, 1);

        // Compute the normalized ray direction assuming a thin-lens camera.
        ray.origin = data.posW;
        ray.dir = ndc.x * data.cameraU + ndc.y * data.cameraV + data.cameraW;
        float2 apertureSample = sample_disk(u); // Sample lies in the unit disk [-1,1]^2
        float3 rayTarget = ray.origin + ray.dir;
        ray.origin += data.apertureRadius * (apertureSample.x * normalize(data.cameraU) + apertureSample.y * normalize(data.cameraV));
        ray.dir = normalize(rayTarget - ray.origin);

        float invCos = 1.f / dot(normalize(data.cameraW), ray.dir);
        ray.tMin = data.nearZ * invCos;
        ray.tMax = data.farZ * invCos;

        return ray;
    }
};

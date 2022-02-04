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
#include "Utils/Math/MathConstants.slangh"

import Scene.Scene;
import Utils.Sampling.SampleGenerator;
import Experimental.Scene.Lights.EmissiveLightSamplerHelpers;
import Experimental.Scene.Lights.EmissiveLightSamplerInterface;
import Utils.Sampling.AliasTable;
import Utils.Math.MathHelpers;

/** Emissive light sampler using uniform sampling of the emissive triangles.

    The sampler implements the IEmissiveLightSampler interface (see
    EmissiveLightSamplerInterface.slang for usage information).

    The struct wraps a LightCollection that stores the pre-processed lights.
    The program should instantiate the struct below. See EmissiveLightSampler.slang.
*/
struct EmissivePowerSampler : IEmissiveLightSampler
{
    AliasTable _aliasTable;

    /** Draw a single light sample.
        \param[in] posW Shading point in world space.
        \param[in] normalW Normal at the shading point in world space.
        \param[in,out] sg Sample generator.
        \param[out] ls Light sample. Only valid if true is returned.
        \return True if a sample was generated, false otherwise.
    */
    bool sampleLight(const float3 posW, const float3 normalW, inout SampleGenerator sg, out TriangleLightSample ls)
    {
        if (gScene.lightCollection.getActiveTriangleCount() == 0) return false;

        // Randomly pick a triangle out of the global list of active triangles with uniform probability.
        //float uLight = sampleNext1D(sg);
        //uint triangleCount = gScene.lightCollection.getActiveTriangleCount();
        //uint idx = min((uint)(uLight * triangleCount), triangleCount - 1); // Safety precaution as the result of the multiplication may be rounded to triangleCount even if uLight < 1.0 when triangleCount is large.
        //uint triangleIndex = gScene.lightCollection.activeTriangles[idx];
        //float triangleSelectionPdf = 1.f / (float)triangleCount;

        uint triangleIndex = _aliasTable.sample(sampleNext2D(sg));
        float triangleSelectionPdf = _aliasTable.getWeight(triangleIndex) / _aliasTable.weightSum;

        // Sample the triangle uniformly.
        float2 u = sampleNext2D(sg);
        if (!sampleTriangle(posW, triangleIndex, u, ls)) return false;

        // The final probability density is the product of the sampling probabilities.
        ls.pdf *= triangleSelectionPdf;
        ls.pdfArea *= triangleSelectionPdf;

        return true;
    }

    bool getEmissiveLightSample(const float3 posW, int lightID, float2 uv, out TriangleLightSample ls)
    {
        if (gScene.lightCollection.getActiveTriangleCount() == 0) return false;
        uint triangleIndex = lightID;
        uint triangleCount = gScene.lightCollection.getActiveTriangleCount();
        // Sample the triangle uniformly.
        float2 u = uv;
        if (!sampleTriangle(posW, triangleIndex, u, ls)) return false;

        return true;
    }

    /** Evaluate the PDF at a shading point given a hit point on an emissive triangle.
        \param[in] posW Shading point in world space.
        \param[in] normalW Normal at the shading point in world space.
        \param[in] hit Triangle hit data.
        \return Probability density with respect to solid angle at the shading point.
    */
    float evalPdf(float3 posW, float3 normalW, const TriangleHit hit)
    {
        if (hit.triangleIndex == LightCollection::kInvalidIndex) return 0;

        float triangleSelectionPdf = _aliasTable.getWeight(hit.triangleIndex) / _aliasTable.weightSum;

        // Compute triangle sampling probability with respect to solid angle from the shading point.
        float trianglePdf = evalTrianglePdf(posW, hit);

        // The final probability density is the product of the sampling probabilities.
        return triangleSelectionPdf * trianglePdf;
    }


    float evalPdf(const float3 posW, int lightID, float2 lightUV)
    {
        TriangleHit hit;
        hit.triangleIndex = lightID;
        const EmissiveTriangle tri = gScene.lightCollection.getTriangle(lightID);
        // Sample the triangle uniformly.
        const float3 barycentrics = sample_triangle(lightUV);
        hit.posW = tri.getPosition(barycentrics);
        // Get triangle normal.
        hit.normalW = tri.normal;

        // Compute light vector and squared distance.
        float3 toLight = hit.posW - posW; // Unnormalized light vector
        const float distSqr = dot(toLight, toLight);
        if (distSqr <= FLT_MIN) return 0.f; // Avoid NaNs below
        float3 L = toLight / sqrt(distSqr);

        // Cosine of angle between the light's normal and the light vector (flip L since it points towards the light).
        float cosTheta = dot(hit.normalW, -L);
        if (cosTheta <= 0.f) return 0.f;

        // Compute probability density with respect to solid angle from the shading point.
        // The farther away the light is and the larger the angle it is at, the larger the pdf becomes. The probability goes to infinity in the limit.
        // Note: Guard against div-by-zero here by clamping.
        // TODO: Do we need the clamp here? distSqr is already clamped, so NaN should not be possible (but +inf is).
        float denom = max(FLT_MIN, cosTheta * gScene.lightCollection.getTriangle(hit.triangleIndex).area);
        return distSqr / denom;
    }

};

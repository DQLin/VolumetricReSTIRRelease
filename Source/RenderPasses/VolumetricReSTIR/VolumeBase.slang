__exported import Scene.Scene;
#include "Utils/Math/MathConstants.slangh"
__exported import HostDeviceSharedConstants;

__exported import Scene.GVDB.gvdb;
__exported import Scene.GVDB.gvdbDda;
__exported import Utils.Sampling.SampleGenerator;

static const bool cbVDBUseTrilinear = true;

static const float kRayTMax = FLT_MAX;

Texture1D<float4> gBlackBodyRadiationTex;

// 0 - 7 normal mips
// 8 - 15 conservative mips

SamplerState gLinearSampler;
SamplerState gPointSampler;

ParameterBlock<VDBInfo> gvdb;

struct Ray
{
    float3 origin;
    float3 dir;
    float tMin;
    float tMax;

    float3 at(float t)
    {
        return origin + dir * t;
    }
};

struct SceneLightSample
{
    // Light sample
    float3  dir;            ///< Direction from the shading point to the light sample in world space (normalized). This is used for BRDF evaluation.
    float   distance;       ///< Distance from the shading point to the light sample.
    float3  Li;             ///< Incident radiance at the shading point (unshadowed). Note: Already divided by the pdf and multiplied by MIS weight.
    float   pdf;            ///< Probability density function with respect to solid angle (pdf == 0 for invalid samples).
    float   pdfArea;
    // Shadow ray parameters
    float3  rayDir;         ///< Ray direction for visibility evaluation (normalized). This may differ from 'dir' due to ray offsets.
    float   rayDistance;    ///< Ray distance for visibility evaluation. This may differ from 'distance' due to ray offsets.
};


inline void CoordinateSystem(float3 v1, inout float3 v2, inout float3 v3) {
    if (abs(v1.x) > abs(v1.y))
        v2 = float3(-v1.z, 0, v1.x) / sqrt(v1.x * v1.x + v1.z * v1.z);
    else
        v2 = float3(0, v1.z, -v1.y) / sqrt(v1.y * v1.y + v1.z * v1.z);
    v3 = cross(v1, v2);
}

inline float3 SphericalDirection(float sinTheta, float cosTheta, float phi,
    float3 x, float3 y, float3 z) {
    return sinTheta * cos(phi) * x + sinTheta * sin(phi) * y + cosTheta * z;
}


float PhaseHG(float cosTheta, float g) {
    float denom = 1 + g * g + 2 * g * cosTheta;
    const float Inv4Pi = 0.07957747154594766788444188168626;
    return Inv4Pi * (1 - g * g) / (denom * sqrt(denom));
}

struct MediumInteraction
{
    float3 p;
    float3 wo;
    float g;
    bool isValid;

    float phaseFunction(float3 wo, float3 wi) {
        return PhaseHG(dot(wo, wi), g);
    }

    float Sample_p(float3 wo, inout float3 wi, float2 u) {
        float cosTheta;
        if (abs(g) < 1e-3)
            cosTheta = 1 - 2 * u[0];
        else {
            float sqrTerm = (1 - g * g) / (1 + g - 2 * g * u[0]);
            cosTheta = -(1 + g * g - sqrTerm * sqrTerm) / (2 * g);
        }

        float sinTheta = sqrt(max(0, 1 - cosTheta * cosTheta));
        float phi = 2 * M_PI * u[1];
        float3 v1, v2;
        CoordinateSystem(wo, v1, v2);
        wi = SphericalDirection(sinTheta, cosTheta, phi, v1, v2, wo);
        return PhaseHG(cosTheta, g);
    }

    Ray SpawnRay(float3 d) {
        return { p, d, 0, kRayTMax };
    }
}

Ray WorldToMedium(Ray r, int mipLevel)
{
    float4x4 WorldToMediumMatrix = mul(gScene.volumeExternalWorldToModelMatrix, gvdb.invxform[mipLevel]);
    float4 o = mul(float4(r.origin, 1), WorldToMediumMatrix);
    float3 d = mul(float4(r.dir, 0), WorldToMediumMatrix).xyz;
    return { o.xyz / o.w, d.xyz, r.tMin, r.tMax }; 
}

float3 WorldToMedium(float3 p, int mipLevel)
{
    float4x4 WorldToMediumMatrix = mul(gScene.volumeExternalWorldToModelMatrix, gvdb.invxform[mipLevel]);
    float4 o = mul(float4(p, 1), WorldToMediumMatrix);
    return o.xyz / o.w;
}


float3 MediumToWorldVec(float3 v, int mipLevel)
{
    float4x4 MediumToWorldMatrix = mul(gvdb.xform[mipLevel], gScene.volumeExternalModelToWorldMatrix);
    return mul(float4(v, 0), MediumToWorldMatrix).xyz;
}

float3 MediumToWorld(float3 p, int mipLevel)
{
    float4x4 MediumToWorldMatrix = mul(gvdb.xform[mipLevel], gScene.volumeExternalModelToWorldMatrix);
    float4 p_ = mul(float4(p, 1), MediumToWorldMatrix);
    return p_.xyz / p_.w;
}

struct Bounds3f
{
    float3 pMin;
    float3 pMax;

    bool IntersectP(Ray ray, inout float hitt0, inout float hitt1) {
        float t0 = 0, t1 = ray.tMax;
        for (int i = 0; i < 3; ++i) {
            float invRayDir = 1 / ray.dir[i];
            float tNear = (pMin[i] - ray.origin[i]) * invRayDir;
            float tFar = (pMax[i] - ray.origin[i]) * invRayDir;

            if (tNear > tFar)
            {
                float temp = tNear;
                tNear = tFar;
                tFar = temp;
            }

            t0 = tNear > t0 ? tNear : t0;
            t1 = tFar < t1 ? tFar : t1;
            if (t0 > t1) return false;
        }
        hitt0 = t0;
        hitt1 = t1;
        return true;
    }
}


bool IntersectVolumeBound(Ray ray, inout float tMin, inout float tMax, int mipLevel, bool vertexCenter = false)
{
    Bounds3f b;

    b.pMin = gvdb.bmin[mipLevel];
    b.pMax = gvdb.bmax[mipLevel];
    if (vertexCenter)
    {
        b.pMin -= float3(0.5);
        b.pMax -= float3(0.5);
    }

    return b.IntersectP(ray, tMin, tMax);
}

float GetVolumeMaxDensity(int mipLevel)
{
    return gvdb.maxValue[mipLevel] * (gScene.volumeDesc.densityScaleFactorByScaling);
}

// velocity is specified in world space
float3 Velocity(float3 p, bool isLastFrame = false)
{
    return mul(float4(getVelocityAtPoint(gvdb, p, 0, gLinearSampler, isLastFrame), 0), gScene.volumeExternalModelToWorldMatrix).xyz;
}

float3 VelocityWorld(float3 pW, bool isLastFrame = false)
{
    int velocityGridOffset = isLastFrame ? kPrevFrameExtraGridOffset : 0;
    float3 p_Medium = WorldToMedium(pW, kVelocityGridId + velocityGridOffset);
    float3 indexSpaceVelocity = Velocity(p_Medium, isLastFrame);
    return indexSpaceVelocity;
}

float Temperature(float3 p, bool isLastFrame = false)
{
    int temperatureGridOffset = isLastFrame ? kPrevFrameExtraGridOffset : 0;
    if (cbVDBUseTrilinear)
        return getValueAtPoint(gvdb, kTemperatureGridId + temperatureGridOffset, p, 0, gLinearSampler);
    else
        return getValueAtPoint(gvdb, kTemperatureGridId + temperatureGridOffset, p, 0, gPointSampler);
}

float TemperatureWorldSpace(float3 pW, bool isLastFrame = false)
{
    float3 p_Medium = WorldToMedium(pW, kTemperatureGridId);
    return Temperature(p_Medium, isLastFrame);
}

// sample RGB black body spectrum from PBRT4
float3 ConvertTempToColor(float temp)
{
    temp = min(6400, (temp - gScene.volumeDesc.temperatureCutOff) * gScene.volumeDesc.temperatureScale);
    float queryPoint = (temp - 25) / 6400;
    return gScene.volumeDesc.LeScale * gBlackBodyRadiationTex.SampleLevel(gLinearSampler, queryPoint, 0).rgb;
}

float3 Emission(float3 p, bool isLastFrame=false)
{
    if (!isLastFrame && !gScene.volumeDesc.hasEmission || isLastFrame && !gScene.volumeDesc.lastFrameHasEmission) return 0.f;
    float temp = Temperature(p, isLastFrame);
    return ConvertTempToColor(temp);
}

float3 EmissionWorldSpace(float3 pW, bool isLastFrame = false)
{
    if (!isLastFrame && !gScene.volumeDesc.hasEmission || isLastFrame && !gScene.volumeDesc.lastFrameHasEmission) return 0.f;
    float3 p_Medium = WorldToMedium(pW, isLastFrame ? kTemperatureGridId + kPrevFrameExtraGridOffset: kTemperatureGridId);
    float temp = Temperature(p_Medium, isLastFrame);
    return ConvertTempToColor(temp);
}

float Density(float3 p, int mipLevel) {
    if (any(p < gvdb.bmin[mipLevel]) || any(p >= gvdb.bmax[mipLevel])) return 0.f;
    return getValueAtPoint(gvdb, mipLevel, p, 0, gLinearSampler) * (gScene.volumeDesc.densityScaleFactorByScaling);
}

float DensityWorldSpace(float3 p, int mipLevel)
{
    float3 p_Medium = WorldToMedium(p, mipLevel);
    return Density(p_Medium, mipLevel);
}

void FetchEightVoxelsInAtlas(float3 p_atlas, int mipLevel, inout float4 densities[2]) {
    float3 anchor = floor(p_atlas - 0.5) + 0.5f;
    [unroll]
    for (int i = 0; i < 8; i++)
    {
        densities[i/4][i%4] = getValueAtlasCoord(gvdb, anchor + float3(i % 2, (i % 4) / 2, i / 4), mipLevel, gPointSampler) * gvdb.densityCompressScaleFactor[mipLevel] * (gScene.volumeDesc.densityScaleFactorByScaling);
    }
}

float DensityInAtlas(float3 p_atlas, int mipLevel, bool useLinearSampler) {
    if (useLinearSampler)
    {
        return getValueAtlasCoord(gvdb, p_atlas, mipLevel, gLinearSampler) * gvdb.densityCompressScaleFactor[mipLevel] * (gScene.volumeDesc.densityScaleFactorByScaling);
    }
    else
    {
        return getValueAtlasCoord(gvdb, p_atlas, mipLevel, gPointSampler) * gvdb.densityCompressScaleFactor[mipLevel] * (gScene.volumeDesc.densityScaleFactorByScaling);
    }
}

__exported import Reservoir;
__exported import VolumeUtils;
__exported import Utils.Color.ColorHelpers;
__exported import ArrayDataProvider;
#include "HostDeviceSharedDefinitions.h"
import Utils.Debug.PixelDebug;

__exported import HostDeviceSharedConstants;

float3 decodeEmissivePosition(int lightID, float2 lightUV)
{
    return float3(lightUV.x, lightUV.y, asfloat(lightID));
}

void encodeEmissivePosition(float3 pos, inout int lightID, inout float2 lightUV)
{
    lightID = asint(pos.z);
    lightUV = pos.xy;
}

float4 decodeWiDist(float3 input, bool reuseAsVertex)
{
    if (reuseAsVertex)
    {
        if (input.x == kRayTMax) return float4(0.f, 0.f, 0.f, kRayTMax);
        else return float4(input, -1.f);
    }
    else
    {
        float3 wi;
        wi.xy = input.xy;
        wi.z = sqrt(1 - (input.x * input.x + input.y * input.y));
        if (isnan(wi.z)) wi.z = 0.f;
        if (input.z < 0) {
            wi.z = -wi.z; input.z = -input.z;
        }
        return float4(wi, input.z);
    }
}

float3 encodeWiDist(float4 input)
{
    float3 wi;
    wi.xy = input.xy;
    wi.z = input.w;
    if (input.z < 0)
    {
        wi.z = -wi.z;
    }
    return wi;
}

int2 decodeSampledPixel(int storage)
{
    int x = storage << 24 >> 24;
    int y = storage << 16 >> 24;
    return int2(x, y);
}

int encodeSampledPixel(int storage, int2 sampledPixel)
{
    return (storage & 0xFFFF0000) | (((sampledPixel.y << 8) | (sampledPixel.x & 0xFF)) & 0xFFFF);
}

// allow a maximum of 2^11 = 2048 bounces (highest bit must be zero!) 
int decodeMaxIndirectBounces(int storage)
{
#if MAX_BOUNCES == 1
    return 0;
#else
    return storage >> 20;
#endif
}

int encodeMaxIndirectBounces(int storage, int bounce)
{
    return (bounce << 20) | (storage & 0xFFFFF);
}

// 0 -- scattering path, 1 -- emissive path
int decodePathTag(int storage)
{
    return (storage >> 16) & 0xF;
}

int encodePathTag(int storage, int pathTag)
{
    return (pathTag << 16) | (storage & 0xFFF0FFFF);
}

float3 evaluate_F_<ArrayDataProvider:IArrayDataProvider, ExternalVisibilityProvider : IExternalVisibilityProvider>(
    REUSETYPE Reservoir tap, VBUFFERDECLARE ArrayDataProvider extraBounceReservoirs, EnvMapSampler envMapSampler, EmissiveLightSampler emissiveSampler,
    Ray ray, inout SampleGenerator sg, SamplingOptions options, inout ExternalVisibilityProvider externalVis, bool isLastFrame, bool noReuse, bool spatialReuse, bool isFinalShading)
{
    bool useLastFrameGrid = gScene.volumeDesc.usePrevGridForReproj && isLastFrame && gScene.volumeDesc.hasAnimation;
    int mipLevelOffset = useLastFrameGrid ? kPrevFrameDensityGridOffset : 0;
    bool isBackgroundSample = tap.depth == kRayTMax;

    ray.tMax = tap.depth;

    float3 F = 1.f;

#if MAX_BOUNCES > 1
    int maxIndirectBounces = decodeMaxIndirectBounces(tap.sampledPixel);
    bool isSelfEmission = maxIndirectBounces == 0 && tap.lightID == kSelfEmissionLightID;
#else
    bool isSelfEmission = tap.lightID == kSelfEmissionLightID;
#endif

    float visibility = 1.f;
    float3 p_World = ray.at(ray.tMax);
    // estimate radiance at resampled position
    MediumInteraction mi = { ray.at(ray.tMax), -ray.dir, gScene.volumeDesc.PhaseFunctionConstantG, true };

    bool isLastVertexSurface = false;

#ifdef SURFACE_SCENE
    ShadingData shadingInfo;
#endif

    {
        float3 sigma_s = 1.f;
        float density = (isBackgroundSample || noReuse) ? 1.f : DensityWorldSpace(p_World, mipLevelOffset);

        if (density == 0.f) return 0.f;

    #ifdef SURFACE_SCENE
        bool hitEmissiveSurface = false;
        bool earlyExit = false;
        if (isBackgroundSample)
        {
            HitInfo hitInfo = { vItem.meshInstanceID, vItem.primitiveID, vItem.barycentrics, vItem.rayT };
            //HitInfo hitInfo2 = FindSurfaceHit(ray.origin, ray.dir, 0, kRayTMax);

            if (hitInfo.isValidHit())
            {
                shadingInfo = computeSurfaceShadingInfo(hitInfo, -ray.dir);
                p_World = shadingInfo.posW;

                // we hit an emissive surface
                if (isEmissive(shadingInfo))
                {
                    if (!isSelfEmission) earlyExit = true;
                    else { F *= shadingInfo.emissive; hitEmissiveSurface = true; }
                }
                else if (isSelfEmission) earlyExit = true;

                ray.tMax = hitInfo.rayT;
                isLastVertexSurface = true;
            }
            else if (!isSelfEmission) earlyExit = true;
            // lightID == kSelfEmissionLightID represents non-reflective kRayTMax Hits (background or emissive surface ) in surface scene
        }
        if (earlyExit) return 0.f;
    #endif

        if (!noReuse)
        {
            if (externalVis.getVisibility(0) != -1.f) visibility = externalVis.getVisibility(0);
            else
            {
            #ifdef SURFACE_SCENE
                // correct for shadow acne result in floating point arithmetic compilation optimization discrepancy when using V Buffer
                if (isLastVertexSurface)
                {
                    float3 dir = p_World - ray.origin;
                    ray.tMax = length(dir);
                    ray.dir = dir / ray.tMax;
                }
                visibility = computeGeneralPathSpaceVisibility(true, !isBackgroundSample, ray, sg, options.visibilitySamples, options.visibilityMipLevel + mipLevelOffset, options.visibilityUseLinearSampler, options.visibilityTrackingMethod, options.visibilityTStepScale, !isFinalShading);
            #else
                visibility = computeVisibility(ray, sg, options.visibilitySamples, options.visibilityMipLevel + mipLevelOffset, options.visibilityUseLinearSampler, options.visibilityTrackingMethod, options.visibilityTStepScale);
            #endif
                externalVis.putVisibility(0, visibility);
            }
        }
        sigma_s = isBackgroundSample ? float3(1.f) : (isSelfEmission ? gScene.volumeDesc.sigma_a : gScene.volumeDesc.sigma_s);

        if (noReuse && !isBackgroundSample) sigma_s /= gScene.volumeDesc.sigma_t;
        F *= visibility * density * sigma_s;

    #ifdef SURFACE_SCENE
        if (hitEmissiveSurface) return F;
    #endif
    }

    float3 P_prefix = 1.f;

    int bounceId = 0;

    if (any(F > 0.f))
    {
        #ifdef SURFACE_SCENE
        if (isLastVertexSurface)
            isBackgroundSample = false;
        #endif
        if (isBackgroundSample)
        {
            float3 backgroundIntensity = envMapSampler.eval(ray.dir, isLastFrame);
            F *= backgroundIntensity;
        }
        else if (isSelfEmission)
        {
            float3 emissionIntensity = EmissionWorldSpace(p_World, useLastFrameGrid);
            F *= emissionIntensity;
        }
        else
        {
            bool isScatterSelfEmission = false;

        #if MAX_BOUNCES > 1
            if (maxIndirectBounces > 0)
            {
                Ray scatterRay;

                isScatterSelfEmission = decodePathTag(tap.sampledPixel) == 1;

                int numIndirectBounces = maxIndirectBounces;

                for (; bounceId < numIndirectBounces; bounceId++)
                {
                    bool isCurrentVertexEmissive = isScatterSelfEmission && bounceId == numIndirectBounces - 1;

                    float4 wiDist = decodeWiDist(extraBounceReservoirs.getData(tap.extraBounceStartId + bounceId).wi_dist,
                        #ifdef VERTEX_REUSE
                        bounceId + 1 >= options.vertexReuseStartBounce
                        #else
                        false
                        #endif
                    );

                    // haven't reach where vertex reuse should start yet, so need to treat emissive vertex differently
                    if (isCurrentVertexEmissive
                    #ifdef VERTEX_REUSE
                        && bounceId + 1 < options.vertexReuseStartBounce
                    #endif
                        )
                    {
                        wiDist.xyz = decodeEmissivePosition(tap.lightID, tap.lightUV);
                        wiDist.w = -1.f; // this means wiDist stores a vertex position
                    }

                #ifdef SURFACE_SCENE
                    HitInfo hitInfo;
                    hitInfo.setInvalid();
                #endif

                    if (wiDist.w == kRayTMax)
                #ifdef SURFACE_SCENE
                    {
                        // find surface intersection
                        // if background, return 0.f
                        hitInfo = FindSurfaceHit(p_World, wiDist.xyz, 0, kRayTMax);
                        if (!hitInfo.isValidHit()) return 0.f;
                        wiDist.w = hitInfo.rayT;
                    }
                #else
                        return 0.f;
                #endif

                    // wiDist.xyz encodes a vertex
                    float dist = 1.f;
                    if (wiDist.w == -1.f)
                    {
                        float3 disp = wiDist.xyz - p_World;
                        dist = length(disp);
                        float3 dir = normalize(disp);
                        scatterRay = { p_World, dir, 0.f, dist };
                    }
                    else
                    {
                        scatterRay = { p_World, wiDist.xyz, 0.f, wiDist.w };
                    }

                    float3 bsdf = 1.f;

                    // this bsdf is the last vertex's bsdf
                #ifdef SURFACE_SCENE
                    if (isLastVertexSurface)
                        bsdf = evalBSDFCosine(shadingInfo, scatterRay.dir);
                    else
                #endif
                        bsdf = mi.phaseFunction(mi.wo, scatterRay.dir);

                    F *= bsdf;
                    if (all(F == 0.f)) return 0.f;


                #ifdef VERTEX_REUSE
                    // actually bounce == bounceId + 1 == options.vertexReuseStartBounce + 1
                    if (bounceId == options.vertexReuseStartBounce)
                    {
                        if (spatialReuse) return F * tap.GetPPartial();
                        else P_prefix = F;
                    }
                #endif

                    // exclude emissive hit
                #ifdef SURFACE_SCENE
                    if (hitInfo.isValidHit())
                    {
                        shadingInfo = computeSurfaceShadingInfo(hitInfo, -scatterRay.dir);
                        if (isEmissive(shadingInfo)) return 0.f;
                    }
                #endif

                    // advance scatter point
                    if (wiDist.w == -1.f)
                    {
                        p_World = wiDist.xyz;
                    }
                    else
                    {
                        p_World = scatterRay.at(scatterRay.tMax);
                    }

                    float3 sigma_s = 1.f;
                    float scatterDensity = 1.f;

                    #ifdef SURFACE_SCENE
                    if (hitInfo.isValidHit())
                    {
                        isLastVertexSurface = true;
                        p_World = shadingInfo.posW;
                    }
                    else
                    #endif
                    {
                        if (!noReuse)
                        {
                            sigma_s = isCurrentVertexEmissive ? gScene.volumeDesc.sigma_a : gScene.volumeDesc.sigma_s;
                            scatterDensity = max(0, DensityWorldSpace(p_World, mipLevelOffset)); // TODO: find out why scatterDensity go to zero here
                        }
                        else
                        {
                            sigma_s = isCurrentVertexEmissive ? gScene.volumeDesc.sigma_a / gScene.volumeDesc.sigma_t
                                : gScene.volumeDesc.sigma_s / gScene.volumeDesc.sigma_t;
                            scatterDensity = 1.f;
                        }
                        isLastVertexSurface = false;
                    }

                    F *= scatterDensity * sigma_s;

                    #ifdef VERTEX_REUSE
                    if (bounceId + 1 == options.vertexReuseStartBounce || (isCurrentVertexEmissive && bounceId + 1 < options.vertexReuseStartBounce))
                    #else
                    if (isCurrentVertexEmissive)
                    #endif
                    {
                        F *= 1.f / (dist * dist);
                    }

                    if (all(F == 0.f)) return 0.f;

                    float scatterVisibility = 1.f;
                    if (!noReuse)
                    {
                        if (externalVis.getVisibility(bounceId + 1) != -1.f) scatterVisibility = externalVis.getVisibility(bounceId + 1);
                        else
                        {
                            float curBounceVisibilityTStepScale = options.visibilityTStepScale * options.visibilityMipLevel;

                        #ifdef SURFACE_SCENE
                            scatterVisibility = computeGeneralPathSpaceVisibility(true, !isLastVertexSurface, scatterRay, sg, options.visibilitySamples, options.visibilityMipLevel + mipLevelOffset, options.visibilityUseLinearSampler, options.visibilityTrackingMethod, curBounceVisibilityTStepScale, !isFinalShading);
                        #else
                            scatterVisibility = computeVisibility(scatterRay, sg, options.visibilitySamples, options.visibilityMipLevel + mipLevelOffset, options.visibilityUseLinearSampler, options.visibilityTrackingMethod, curBounceVisibilityTStepScale);
                        #endif
                            externalVis.putVisibility(bounceId + 1, scatterVisibility);
                        }
                    }

                #ifdef SURFACE_SCENE
                    if (scatterVisibility == 0.f) return 0.f;
                #endif

                    F *= scatterVisibility;

                    mi.wo = -scatterRay.dir;
                    mi.p = p_World;

                    if (all(F == 0.f)) return 0.f;
                }

                if (isScatterSelfEmission)
                    F *= EmissionWorldSpace(p_World, useLastFrameGrid);
                else
                    mi = { scatterRay.at(scatterRay.tMax), -scatterRay.dir, gScene.volumeDesc.PhaseFunctionConstantG, true };
            }
        #endif

            // bounceId already ++
            if (!isScatterSelfEmission && any(F > 0.f))
            {
                bool lightVisibilityReuse = false;
                float precomputedVisibility = tap.GetPPartial();
            #ifdef VERTEX_REUSE
                if (bounceId == options.vertexReuseStartBounce && spatialReuse)
                    lightVisibilityReuse = true;
            #endif

                //bounceId is already "bounce"
            #ifdef SURFACE_SCENE
                if (isLastVertexSurface) F *= evaluate_L_on_surface(shadingInfo, tap.lightID, tap.lightUV, precomputedVisibility, bounceId, sg, options, envMapSampler, emissiveSampler, lightVisibilityReuse, isLastFrame, true, !isFinalShading);
                else
            #endif  
                    F *= evaluate_L_in_volume(mi, tap.lightID, tap.lightUV, precomputedVisibility, bounceId, sg, options, envMapSampler, emissiveSampler, lightVisibilityReuse, isLastFrame, true, !isFinalShading); // bounceId already +1 by for loop

                #ifdef VERTEX_REUSE
                if (bounceId == options.vertexReuseStartBounce && !spatialReuse)
                    tap.SetPPartial(precomputedVisibility);
                #endif
            }
        }
    }

    #ifdef VERTEX_REUSE
    // we must guarantee that no component in P_prefix is zero, in this case, the BSDF shouldn't contain zero component
    // if finding violated, we need to collect the product P_suffix instead of dividing by P_prefix 
    if (bounceId > options.vertexReuseStartBounce && !spatialReuse)
        tap.SetPPartial(luminance(F / P_prefix)); 
    #endif

    return F;
}


float evaluate_P_hat<ArrayDataProvider:IArrayDataProvider, ExternalVisibilityProvider : IExternalVisibilityProvider>(Ray ray, inout SampleGenerator sg, EnvMapSampler envMapSampler, EmissiveLightSampler emissiveSampler,
    VBUFFERDECLARE ArrayDataProvider extraBounceReservoirs, SamplingOptions options,
    REUSETYPE Reservoir tap, inout ExternalVisibilityProvider externalVis, bool isLastFrame = false, bool nonBinary = true, bool spatialReuse = false) // only set to false when computing binary MIS
{
    float3 F = evaluate_F_(tap, VBUFFERITEM extraBounceReservoirs, envMapSampler, emissiveSampler, ray, sg, options, externalVis, isLastFrame, false, spatialReuse, false);
    return !nonBinary && any(F > 0.f) ? 1.f : luminance(F);
}

float3 evaluate_F<ArrayDataProvider:IArrayDataProvider>(Reservoir tap, VBUFFERDECLARE ArrayDataProvider extraBounceReservoirs, EnvMapSampler envMapSampler, EmissiveLightSampler emissiveSampler,
    Ray ray, inout SampleGenerator sg, SamplingOptions options, bool noReuse)
{
    SingleExternalVisibilityProvider externalVis;
    externalVis.Initialize();
    return evaluate_F_(tap, VBUFFERITEM extraBounceReservoirs, envMapSampler, emissiveSampler, ray, sg, options, externalVis, false, noReuse, false, true);
}

// Include phase function
float3 evaluate_L_in_volume(MediumInteraction mi, int lightID, float2 lightUV, inout float precomputedVisibility, int bounce, inout SampleGenerator sg, SamplingOptions options,
                            EnvMapSampler envMapSampler, EmissiveLightSampler emissiveSampler, bool lightVisibilityReuse=false, bool isLastFrame=false, bool nonBinary=true, bool cullNonOpaqueGeometry=false)
{
    Ray shadowRay;
    float3 Ld = 0.f;
    bool isValidSample = true;

    bool useLastFrameGrid = gScene.volumeDesc.usePrevGridForReproj && isLastFrame && gScene.volumeDesc.hasAnimation;
    int extraGridOffset = useLastFrameGrid ? kPrevFrameExtraGridOffset : 0;
    int densityGridOffset = useLastFrameGrid ? kPrevFrameDensityGridOffset : 0;

    // voxel light
    if (lightID < 0)
    {
        float3 wiWorld = envMapSampler.decodeLightUV(lightUV, lightID, isLastFrame);
        shadowRay = { mi.p, wiWorld, 0, kRayTMax };
        Ld = envMapSampler.eval(wiWorld, isLastFrame) * mi.phaseFunction(mi.wo, wiWorld);
    }
    // analytical light (currently only supporting directional and point lights
    else if (lightID < (int)gScene.getLightCount())
    {
        SceneLightSample ls = getAnalyticalLightSample(lightID, mi.p, sg); // TODO: remove sg
        shadowRay = { mi.p, ls.rayDir, 0, ls.rayDistance };
        Ld = ls.Li * mi.phaseFunction(mi.wo, ls.rayDir);
    }
    // emissive light (TODO: consider invalid case for unbiased reuse)
    else
    {
        TriangleLightSample ls;
        isValidSample = emissiveSampler.getEmissiveLightSample(mi.p, lightID - (int)gScene.getLightCount(), lightUV, ls);
        if (isValidSample)
        {
            shadowRay = { mi.p, ls.dir, 0, ls.distance };
            Ld = ls.Le * gScene.emissiveIntensityMultiplier * mi.phaseFunction(mi.wo, ls.dir) * ls.cosTheta / (ls.distance * ls.distance);
        }
    }

    float Tr = 1.f;
    if (isValidSample)
    {
        if (lightVisibilityReuse) Tr = precomputedVisibility;
        else
        {
            // TODO: reuse only if two sample points are similar enough
            float curBounceLightingTStepScale = options.lightingTStepScale * options.lightingMipLevel;

#ifdef SURFACE_SCENE
            Tr = computeGeneralPathSpaceVisibility(nonBinary, true, shadowRay, sg, options.lightSamples, cullNonOpaqueGeometry ? options.lightingMipLevel + densityGridOffset : 0, options.lightingUseLinearSampler, options.lightingTrackingMethod, curBounceLightingTStepScale, cullNonOpaqueGeometry);
#else
            Tr = computeVisibility(shadowRay, sg, options.lightSamples, cullNonOpaqueGeometry ? options.lightingMipLevel + densityGridOffset : 0, options.lightingUseLinearSampler, options.lightingTrackingMethod, curBounceLightingTStepScale);
#endif
            precomputedVisibility = Tr;
        }
    }

    return Tr * Ld;
}

#ifdef SURFACE_SCENE
float3 evaluate_L_on_surface(ShadingData shadingInfo, int lightID, float2 lightUV, inout float precomputedVisibility, int bounce, inout SampleGenerator sg, SamplingOptions options,
    EnvMapSampler envMapSampler, EmissiveLightSampler emissiveSampler, bool lightVisibilityReuse=false, bool isLastFrame=false, bool nonBinary=true, bool cullNonOpaqueGeometry = false)
{
    Ray shadowRay;
    float3 Ld = 0.f;
    bool isValidSample = true;

    bool useLastFrameGrid = gScene.volumeDesc.usePrevGridForReproj && isLastFrame && gScene.volumeDesc.hasAnimation;
    int extraGridOffset = useLastFrameGrid ? kPrevFrameExtraGridOffset : 0;
    int densityGridOffset = useLastFrameGrid ? kPrevFrameDensityGridOffset : 0;

    // voxel light
    if (lightID < 0)
    {
        float3 wiWorld = envMapSampler.decodeLightUV(lightUV, lightID, isLastFrame);
        shadowRay = { shadingInfo.posW, wiWorld, 0, kRayTMax };
        Ld = envMapSampler.eval(wiWorld, isLastFrame) * evalBSDFCosine(shadingInfo, wiWorld);
    }
    // analytical light (currently only supporting directional and point lights
    else if (lightID < (int)gScene.getLightCount())
    {
        SceneLightSample ls = getAnalyticalLightSample(lightID, shadingInfo.posW, sg); // TODO: remove sg
        shadowRay = { shadingInfo.posW, ls.rayDir, 0, ls.rayDistance };
        Ld = ls.Li * evalBSDFCosine(shadingInfo, ls.rayDir);
    }
    // emissive light (TODO: consider invalid case for unbiased reuse)
    else
    {
        TriangleLightSample ls;
        isValidSample = emissiveSampler.getEmissiveLightSample(shadingInfo.posW, lightID - (int)gScene.getLightCount(), lightUV, ls);
        if (isValidSample)
        {
            shadowRay = { shadingInfo.posW, ls.dir, 0, ls.distance };
            Ld = ls.Le * gScene.emissiveIntensityMultiplier * evalBSDFCosine(shadingInfo, ls.dir) * ls.cosTheta / (ls.distance * ls.distance);
        }
    }

    float Tr = 1.f;
    if (isValidSample)
    {
        if (lightVisibilityReuse) Tr = precomputedVisibility;
        else
        {
            float curBounceLightingTStepScale = options.lightingTStepScale * options.lightingMipLevel;
            Tr = computeGeneralPathSpaceVisibility(nonBinary, true, shadowRay, sg, options.lightSamples, options.lightingMipLevel + densityGridOffset, options.lightingUseLinearSampler, options.lightingTrackingMethod, curBounceLightingTStepScale, cullNonOpaqueGeometry);

            precomputedVisibility = Tr;
        }
    }

    return Tr * Ld;
}
#endif


bool IsWithinRange(int2 pos, int2 resolution)
{
    return pos.x >= 0 && pos.x < resolution.x&& pos.y >= 0 && pos.y < resolution.y;
}

inline int getPixelID(int2 pixelPos, int2 resolution)
{
    return pixelPos.y * resolution.x + pixelPos.x;
}

bool resampleNeighborSpatialReuse<ArrayDataProvider:IArrayDataProvider>(inout Reservoir tap, Ray ray, inout SampleGenerator sg, EnvMapSampler envMapSampler, EmissiveLightSampler emissiveSampler,
    VBUFFERDECLARE ArrayDataProvider extraBounceReservoirs, SamplingOptions options)
{
    if (tap.runningSum == 0.f) return true;

    SingleExternalVisibilityProvider externalVis;
    externalVis.Initialize();

    float p_y_hat = evaluate_P_hat(ray, sg, envMapSampler, emissiveSampler, VBUFFERITEM extraBounceReservoirs, options, tap, externalVis, false, true, true);

    float weight = p_y_hat / tap.p_y;

    if (isinf(weight) || isnan(weight)) weight = 0.f;
    tap.runningSum *= weight;
    tap.p_y = p_y_hat;

    return true;
}

bool resampleNeighbor<ArrayDataProvider:IArrayDataProvider>(inout Reservoir tap, Ray ray, inout SampleGenerator sg, EnvMapSampler envMapSampler, EmissiveLightSampler emissiveSampler, VBUFFERDECLARE ArrayDataProvider extraBounceReservoirs, SamplingOptions options)
{
    if (tap.runningSum == 0.f) return true;

    SingleExternalVisibilityProvider externalVis;
    externalVis.Initialize();

    float p_y_hat = evaluate_P_hat(ray, sg, envMapSampler, emissiveSampler, VBUFFERITEM extraBounceReservoirs, options, tap, externalVis, false, true, false);

    float weight = p_y_hat / tap.p_y;

    if (isinf(weight) || isnan(weight)) weight = 0.f;
    tap.runningSum *= weight;
    tap.p_y = p_y_hat;

    return true;
}

float evaluatePHatReadOnly<ArrayDataProvider:IArrayDataProvider>(Reservoir r, Ray neighborRay, inout SampleGenerator sg,
    EnvMapSampler envMapSampler, EmissiveLightSampler emissiveSampler, VBUFFERDECLARE ArrayDataProvider extraBounceReservoirs, SamplingOptions options, bool isLastFrame, bool useReSTIRMIS, bool spatialReuse)
{
    SingleExternalVisibilityProvider externalVis;
    externalVis.Initialize();
    float currentPdf = evaluate_P_hat(neighborRay, sg, envMapSampler, emissiveSampler, VBUFFERITEM extraBounceReservoirs, options, r, externalVis, isLastFrame, useReSTIRMIS, spatialReuse);
    return currentPdf;
}


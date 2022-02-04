__exported import VolumeUtils;

float3 IntegrateByVolumePathTracing(Ray ray, inout SampleGenerator sg, bool useLinearAnalyticalSampler, int lightSamples, bool useEnvironmentLights, bool useAnalyticLights, bool useEmissiveLights,
    EnvMapSampler envMapSampler, EmissiveLightSampler emissiveSampler,
    #ifdef SURFACE_SCENE
    HitInfo hitInfo,
    #endif
    int mipLevel, int maxBounces, bool useNEE, bool useRussianRoulette)
{
    MediumInteraction mi;
    float3 beta = 1.f;
    float3 L = 0.f;

    if (!useNEE)
        maxBounces += 1;

    for (int bounce = 0; bounce < maxBounces; bounce++)
    {
        mi.isValid = false;

        // intersect with surface scene to find tMax
#ifdef SURFACE_SCENE
        if (bounce > 0)
            hitInfo = FindSurfaceHit(ray.origin, ray.dir, 0, kRayTMax);
        ray.tMax = hitInfo.rayT;
#endif
        // decomposition tracking
        SampleMediumSuperVoxelGeneric(ray, sg, mi, mipLevel);

        if (mi.isValid)
        {
            float3 albedo = gScene.volumeDesc.sigma_s / gScene.volumeDesc.sigma_t;
            float3 one_minus_albedo = 1.f - albedo;

            {
                float3 Le = EmissionWorldSpace(mi.p);
                L += Le * one_minus_albedo * beta;
            }

            beta *= albedo;

            if (useNEE)
            {
                float3 Ld = directLighting(sg, mi,
                        envMapSampler, emissiveSampler, lightSamples, useEnvironmentLights, useAnalyticLights, useEmissiveLights, true, mipLevel, kResidualRatioTracking);

                L += beta * Ld;
            }

            float3 wo = -ray.dir, wi;

            if (maxBounces > 1)
            {
                // generate next bounce direction
                mi.Sample_p(wo, wi, sampleNext2D(sg));
            }

            if (bounce < maxBounces - 1)
            {
                ray = mi.SpawnRay(wi);

                if (useRussianRoulette && bounce >= 2)
                {
                    if (sampleNext1D(sg) < albedo.r) // assuming monochromatic albedo
                    {
                        beta /= albedo.r;
                    }
                    else
                    {
                        bounce = maxBounces;
                    }
                }
            }
        }
        else
        {
#ifdef SURFACE_SCENE
            // do we hit background?
            if (hitInfo.isValidHit())
            {
                ShadingData shadingInfo = computeSurfaceShadingInfo(hitInfo, -ray.dir);

                if (isEmissive(shadingInfo))
                {
                    if (!useNEE || bounce == 0)
                    {
                        float3 Le = shadingInfo.emissive;
                        L += beta * Le;
                    }
                    bounce = maxBounces; 
                }
                else
                {
                    // compute direct lighting
                    if (useNEE)
                    {
                        float3 Ld = directLightingSurface(sg, shadingInfo, envMapSampler, emissiveSampler, lightSamples, useEnvironmentLights, useAnalyticLights,
                            useEmissiveLights, true, mipLevel, true ? kResidualRatioTracking : kRatioTracking);

                        L += beta * Ld;
                    }

                    // create next bounce
                    if (bounce < maxBounces - 1)
                    {
                        BSDFSample result;
                        sampleBSDF(shadingInfo, sg, result);
                        float3 wi = result.wi;
                        beta *= result.weight;
                        mi.p = shadingInfo.posW;
                        ray = mi.SpawnRay(wi);
                    }
                }
            }
            else
#endif
            {
                float3 Le = envMapSampler.eval(ray.dir);

                // hit background
                if (!useNEE || bounce == 0)
                {
                    L += beta * Le;
                }
                bounce = maxBounces;
            }
        }
    }

    return L;
}


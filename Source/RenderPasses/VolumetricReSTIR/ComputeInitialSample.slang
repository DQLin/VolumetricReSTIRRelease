import ReSTIRHelper;
import Utils.Debug.PixelDebug;

Reservoir ComputeInitialSample(Ray primaryRay, float precomputedHitDistance, float precomputedPdfDist, float precomputedTr, int maxBounces, int numInitialSamples,
    inout SampleGenerator sg, SamplingOptions options, EnvMapSampler envMapSampler, EmissiveLightSampler emissiveSampler,
    bool useCoarserGridForIndirectBounce, bool useRussianRoulette, bool noReuse
#ifdef SURFACE_SCENE
    , HitInfo hitInfo
#endif
#if MAX_BOUNCES > 1
    , inout ExtraBounceReservoir extrabounceReservoir[MAX_BOUNCES - 1]
#endif
)
{
    float pathPdf = 1.f;
    float pathPHat = 1.f;
    float lastDepth = 0.f;

    Ray ray = primaryRay;

#if MAX_BOUNCES > 1
    Reservoir combinedReservoir = createNewReservoir();
#endif

    Reservoir outReservoir;

    int bounce = 0;

    float primaryScatterDepth = 0;

    for (; bounce < maxBounces; bounce++)
    {
        outReservoir = createNewReservoir();
        outReservoir.M = 1;

        float curHitDist;
        float pdfDist = 0;

        MediumInteraction mi;

        float Tr;
        // sample distances again for scattered rays
        if (bounce >= 1 || noReuse)
        {
            // intersect with surface scene to find tMax
        #ifdef SURFACE_SCENE
            hitInfo = FindSurfaceHit(ray.origin, ray.dir, 0, kRayTMax);
            ray.tMax = hitInfo.rayT;
        #endif

            if (noReuse)
            {
                SampleMediumSuperVoxelGeneric(ray, sg, mi, 0);
                pdfDist = 1.f;
                Tr = 1.f;
                curHitDist = mi.isValid ? length(mi.p - ray.origin) : kRayTMax;
            }
            else
            {
                //options.visibilityMipLevel >= kNumMaxMips means using conservative grid
                int curBounceVisibilityMipLevel = options.visibilityMipLevel;

                if (useCoarserGridForIndirectBounce)
                {
                    curBounceVisibilityMipLevel = min((options.visibilityMipLevel >= kNumMaxMips ? kNumMaxMips : 0) + gScene.volumeDesc.numMips - 1, curBounceVisibilityMipLevel+1);
                }

                float hitDistances[MaxInitialSampleCount];
                float pdfDists[MaxInitialSampleCount];
                float outTrs[MaxInitialSampleCount];
                SampleMediumAnalyticGeneric(ray, sg, options.visibilityUseLinearSampler, hitDistances, curBounceVisibilityMipLevel, pdfDists, outTrs, 1);
                curHitDist = hitDistances[0];
                pdfDist = pdfDists[0];
                Tr = outTrs[0];
                mi = { ray.at(curHitDist), -ray.dir, gScene.volumeDesc.PhaseFunctionConstantG, curHitDist != kRayTMax };
            }
        }
        else
        {
            curHitDist = precomputedHitDistance;
            pdfDist = precomputedPdfDist;
            Tr = precomputedTr;
            mi = { ray.at(curHitDist), -ray.dir, gScene.volumeDesc.PhaseFunctionConstantG, curHitDist != kRayTMax };
        }

        pathPdf *= pdfDist;

        #ifdef VERTEX_REUSE
        if (bounce == options.vertexReuseStartBounce && curHitDist != kRayTMax)
        {
            pathPdf /= curHitDist * curHitDist;
            pathPHat /= curHitDist * curHitDist;
        }
        #endif

        bool hitEmpty = false;

        float actualVolumeDensity = 0.f;

        if (bounce == 0)
        {
        #if MAX_BOUNCES > 1
            outReservoir.sampledPixel = encodeMaxIndirectBounces(outReservoir.sampledPixel, 0);
        #endif
            outReservoir.depth = mi.isValid ? curHitDist : kRayTMax;
            primaryScatterDepth = outReservoir.depth;
            outReservoir.p_y = pathPdf;
        }
        else
        {
            outReservoir.depth = primaryScatterDepth;

        #if MAX_BOUNCES > 1
            outReservoir.sampledPixel = encodeMaxIndirectBounces(outReservoir.sampledPixel, bounce);

        #ifdef VERTEX_REUSE
            if (bounce < options.vertexReuseStartBounce)
            {
                extrabounceReservoir[bounce - 1].wi_dist = encodeWiDist(float4(ray.dir, !mi.isValid ? kRayTMax : curHitDist));
            }
            else
            {
                extrabounceReservoir[bounce - 1].wi_dist = !mi.isValid ? float3(kRayTMax) : mi.p;
            }
        #else
            extrabounceReservoir[bounce - 1].wi_dist = encodeWiDist(float4(ray.dir, !mi.isValid ? kRayTMax : curHitDist));
        #endif

        #endif

            outReservoir.p_y = pathPdf;
        }

        if (mi.isValid)
        {
            if (noReuse) actualVolumeDensity = 1.f;
            else actualVolumeDensity = DensityWorldSpace(mi.p, 0);
        }

        // density is zero in actual volume
        if (
    #ifndef SURFACE_SCENE
        (!mi.isValid && bounce > 0) ||
    #endif
            (mi.isValid && actualVolumeDensity == 0))
        {
            outReservoir.p_y = 0.f;
            outReservoir.runningSum = 0.f;
            hitEmpty = true;
        }

        float pdfDir = 1.f;

        if (!hitEmpty)
        {
            if (mi.isValid
            #ifdef SURFACE_SCENE
                || hitInfo.isValidHit()
            #endif
                )
            {
                bool isSurfaceHit = !mi.isValid;

                // ideally this should vary with material
                float3 albedo = gScene.volumeDesc.sigma_s / gScene.volumeDesc.sigma_t;

            #ifdef SURFACE_SCENE
                ShadingData shadingInfo;
                if (isSurfaceHit)
                {
                    shadingInfo = computeSurfaceShadingInfo(hitInfo, -ray.dir);
                    if (isEmissive(shadingInfo))
                    {
                        if (bounce == 0)
                        {
                            float3 Le = shadingInfo.emissive;
                            pathPHat *= Tr;
                            // Computing P Hat
                            float p_y = pathPHat * luminance(Le);
                            outReservoir.runningSum = p_y / outReservoir.p_y;
                            outReservoir.p_y = p_y;
                            outReservoir.lightID = kSelfEmissionLightID;
                        }
                        else
                        {
                            outReservoir.runningSum = 0.f;
                            outReservoir.p_y = 0.f;
                        }
                        hitEmpty = true;
                    }
                }
            #endif

                if (!hitEmpty)
                {

                    outReservoir.lightID = -1;
                    outReservoir.lightUV = float2(0);
                    float outLightPdf = 0.f;
                    float outVisibility = 1.f;
                    float3 Ld = 0.f;
                    float3 Le = 0.f;

                    // absorption ratio
                    float3 one_minus_albedo = 1.f - albedo;

                    bool shouldSampleLightOrEmission = true;

                    // include emission if it has emission
                    if (gScene.volumeDesc.hasEmission && (actualVolumeDensity > 0.f) && !isSurfaceHit)
                    {
                        Le = EmissionWorldSpace(mi.p);
                    }

                    bool shouldComputePHat = true;

                    bool shouldComputeLightVisibility = options.lightSamples == 0 ? false : shouldComputePHat;

                    #ifdef SURFACE_SCENE 
                    if (isSurfaceHit)
                    {
                        Ld = SampleDirectLightingSurface(sg, outLightPdf, shadingInfo,
                                envMapSampler, emissiveSampler, options.useEnvironmentLights, options.useAnalyticLights, options.useEmissiveLights,
                            shouldComputeLightVisibility, options.lightingMipLevel, options.lightingTrackingMethod, options.lightingUseLinearSampler, options.lightSamples, options.lightingTStepScale,
                            outReservoir.lightID, outReservoir.lightUV, outVisibility);
                    }
                    else
                    #endif
                    {
                        Ld = SampleDirectLighting(sg, outLightPdf, mi,
                            envMapSampler, emissiveSampler, options.useEnvironmentLights, options.useAnalyticLights, options.useEmissiveLights,
                            shouldComputeLightVisibility, options.lightingMipLevel, options.lightingTrackingMethod, options.lightingUseLinearSampler, options.lightSamples, options.lightingTStepScale,
                            outReservoir.lightID, outReservoir.lightUV, outVisibility);
                    }

                    float3 wo = -ray.dir, wi;

                    if (maxBounces > 1)
                    {
                    #ifdef SURFACE_SCENE
                        if (isSurfaceHit)
                        {
                            BSDFSample scatterSampleResult;
                            scatterSampleResult.weight = 1.f;
                            scatterSampleResult.pdf = 1.f;
                            // generate next bounce direction
                            sampleBSDF(shadingInfo, sg, scatterSampleResult);
                            wi = scatterSampleResult.wi;
                            pdfDir = scatterSampleResult.pdf;
                        }
                        else
                    #endif
                        {
                            // generate next bounce direction
                            pdfDir = mi.Sample_p(wo, wi, sampleNext2D(sg));
                        }
                    }

                    float p_src = outReservoir.p_y;

                #ifdef SURFACE_SCENE
                    if (isSurfaceHit)
                    {
                        p_src *= outLightPdf;
                    }
                    else
                #endif
                        if (shouldSampleLightOrEmission)
                        {
                            // this is actually an RIS step!
                            float emissionRatio = luminance(one_minus_albedo * Le) / (luminance(one_minus_albedo * Le) + luminance(albedo * Ld));
                            if (isnan(emissionRatio)) emissionRatio = 0.f;

                            // Sample between emission and in-scattering
                            if (sampleNext1D(sg) < emissionRatio)
                            {
                                p_src *= emissionRatio;
                                outReservoir.lightID = kSelfEmissionLightID;                                
                            }
                            else
                            {
                                p_src *= outLightPdf * (1 - emissionRatio);
                            }
                        }

                    outReservoir.runningSum = p_src == 0.f ? 0.f : 1.f;
                    outReservoir.p_y = p_src;

                    // Computing P Hat
                    if (shouldComputePHat)
                    {
                        float p_y;
                        pathPHat *= Tr;

                    #ifdef SURFACE_SCENE
                        if (isSurfaceHit)
                        {
                            p_y = pathPHat * luminance(Ld * outLightPdf); // Ld is bsdf * L / pdf
                            pathPHat *= luminance(evalBSDFCosine(shadingInfo, wi));
                        }
                        else
                    #endif
                        {
                            pathPHat *= actualVolumeDensity;
                            if (outReservoir.lightID == kSelfEmissionLightID) // emission
                                p_y = pathPHat * luminance(gScene.volumeDesc.sigma_a * Le);
                            else
                                p_y = pathPHat * luminance(gScene.volumeDesc.sigma_s * Ld * outLightPdf); // Ld is phasefunction * L / pdf

                            pathPHat *= luminance(gScene.volumeDesc.sigma_s) * pdfDir;
                            if (noReuse)
                            {
                                p_y /= gScene.volumeDesc.sigma_t;
                                pathPHat /= gScene.volumeDesc.sigma_t;
                            }
                        }

                        if (outReservoir.runningSum > 0.f)
                        {
                            outReservoir.runningSum = outReservoir.p_y == 0.f ? 0.f : p_y / outReservoir.p_y;

                            if (outReservoir.lightID == kSelfEmissionLightID && bounce > 0)
                            {
                            #ifdef VERTEX_REUSE
                                if (bounce < options.vertexReuseStartBounce)
                            #endif
                                {
                                    encodeEmissivePosition(mi.p, outReservoir.lightID, outReservoir.lightUV);
                                    p_y /= (curHitDist * curHitDist); // because we do vertex reuse, we need to include geometry factor
                                }
                                outReservoir.sampledPixel = encodePathTag(outReservoir.sampledPixel, 1);
                            }
                            outReservoir.p_y = p_y;
                        }
                    }

                    pathPdf *= pdfDir;

                #ifdef SURFACE_SCENE
                    if (isSurfaceHit)
                        mi.p = shadingInfo.posW;
                #endif

                #if MAX_BOUNCES > 1 
                    if (bounce < maxBounces - 1)
                    {
                        ray = mi.SpawnRay(wi);
                        if (useRussianRoulette && bounce >= 2 && !isSurfaceHit)
                        {
                            if (sampleNext1D(sg) < albedo.r)
                            {
                                pathPdf *= albedo.r;
                            }
                            else
                            {
                                hitEmpty = true;
                                combinedReservoir.M++; // no actual effect, just to symbolize an null sample added to the reservoir
                            }
                        }
                    }
                #endif
                }
            }
            else
            {
                // hit the background
                float3 Le = envMapSampler.eval(ray.dir);

                pathPHat *= Tr;
                // Computing P Hat
                float p_y = pathPHat * luminance(Le);
                outReservoir.runningSum = outReservoir.p_y == 0.f ? 0.f : p_y / outReservoir.p_y;
                outReservoir.p_y = p_y;

            #ifdef SURFACE_SCENE 
                outReservoir.lightID = kSelfEmissionLightID;
            #endif

                hitEmpty = true;
            }
        }

    #if MAX_BOUNCES > 1
        simpleResampleStep(outReservoir, combinedReservoir, sg);
    #endif
        if (hitEmpty) break;
    }

#if MAX_BOUNCES > 1 
    combinedReservoir.M = 1;
    return combinedReservoir;
#else
    return outReservoir;
#endif
}

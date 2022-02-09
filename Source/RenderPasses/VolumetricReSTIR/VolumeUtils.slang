__exported import VolumeBase;
import VolumeTrackingAdapterGVDB;
__exported import Experimental.Scene.Lights.EnvMapSampler;
__exported import Experimental.Scene.Lights.EmissiveLightSampler;
__exported import Utils.Helpers; // For computeRayOrigin()
import Utils.Debug.PixelDebug;

import Experimental.Scene.Lights.LightHelpers;
__exported import InlineRayTracingHelpers;
__exported import Experimental.Scene.Material.MaterialShading;

bool sampleSceneLights(const EnvMapSampler envMapSampler, const EmissiveLightSampler emissiveSampler, const float3 rayOrigin, bool kUseEnvironmentLights, bool kUseAnalyticLights, bool kUseEmissiveLights,
    inout SampleGenerator sg, out SceneLightSample ls, out int outLightIndex, out float2 outLightUV, float3 surfaceNormal = float3(0.f))
{
    if (!kUseEnvironmentLights && !kUseAnalyticLights && !kUseEmissiveLights ) return false;
    // Set relative probabilities of the different sampling techniques.
    // TODO: These should use estimated irradiance from each light type. Using equal probabilities for now.
    // TODO: Refactor the selection pdf code the helpers above for evaluating the individual probabilities.
    float p[3];
    p[0] = kUseEnvironmentLights ? 1.f : 0.f;
    p[1] = kUseAnalyticLights ? 1.f : 0.f;
    p[2] = kUseEmissiveLights ? 1.f : 0.f;

    // Normalize probabilities. Early out if zero.
    float sum = p[0] + p[1] + p[2];
    if (sum == 0.f) return false;

    float invSum = 1.f / sum;
    p[0] *= invSum;
    p[1] *= invSum;
    p[2] *= invSum;

    // Sample based on uniform random number. Rescale u to [0,1) afterwards.
    float invPdf = 0.f;
    float u = sampleNext1D(sg);

    // We use explicit checks for which light types are enabled so that the compiler
    // can remove the unused code. It won't otherwise since u is unknown at compile time.
    if (kUseEnvironmentLights)
    {
        if (u < p[0])
        {
            float selectionPdf = p[0];

            // Sample environment map.
            EnvMapSample lightSample;
            envMapSampler.sample(sampleNext2D(sg), lightSample);

            // Reject sample if lower hemisphere.
            //if (dot(sd.N, lightSample.dir) < kMinCosTheta) return false;

            float pdf = selectionPdf * lightSample.pdf;
            // Setup returned sample.
            ls.rayDir = ls.dir = lightSample.dir;
            ls.rayDistance = ls.distance = kRayTMax;
            ls.pdf = pdf;
            ls.pdfArea = pdf; // environment lights do not have area measure
            ls.Li = pdf > 0.f ? lightSample.Le / pdf : 0.f;

            outLightIndex = -1;
            outLightUV = envMapSampler.encodeLightUV(ls.rayDir, outLightIndex);

            return !any(isnan(ls.rayDir)); // TODO: figure out why sampled dir can be nan
        }
        u -= p[0];
    }

    if (kUseAnalyticLights)
    {
        if (u < p[1])
        {
            // Sample analytic light source selected uniformly from the light list.
            // TODO: Sample based on estimated contributions as pdf.
            u /= p[1]; // Rescale to [0,1)
            uint lightCount = gScene.getLightCount();
            uint lightIndex = min(uint(u * lightCount), lightCount - 1);
            float selectionPdf = p[1] / lightCount; // TODO: Precompute 1.f/lightCount in cbuffer

            // Sample local light source.
            AnalyticLightSample lightSample;
            bool valid = sampleLight(rayOrigin, gScene.getLight(lightIndex), sg, lightSample);

            outLightIndex = lightIndex;

            // Reject sample if lower hemisphere.
            //if (!valid || dot(sd.N, lightSample.dir) < kMinCosTheta) return false;

            // Setup returned sample.
            // Analytic lights do not currently have a geometric representation in the scene.
            // Do not worry about adjusting the ray to avoid self-intersections at the light.
            ls.rayDir = ls.dir = lightSample.dir;
            ls.rayDistance = ls.distance = lightSample.distance;
            if (lightSample.pdf == 0) lightSample.pdf = 1.f; // setting dirac function to 1.f
            ls.pdf = selectionPdf * lightSample.pdf;
            ls.pdfArea = ls.pdf; // punctual lights do not have area measure.
            ls.Li = lightSample.Li / ls.pdf;

            // currently analytic lights do not have uv
            outLightUV = 0;
            return true;
        }
        u -= p[1];
    }


    if (kUseEmissiveLights)
    {
        if (u < p[2]) // Always true
        {
            float selectionPdf = p[2];

            // Sample emissive lights.
            TriangleLightSample lightSample;
            bool valid = emissiveSampler.sampleLight(rayOrigin, surfaceNormal, sg, lightSample);

            // Reject sample if lower hemisphere.
            //if (!valid || dot(sd.N, lightSample.dir) < kMinCosTheta) return false;
            // Evaluate MIS with BRDF sampling as the other sampling strategy.
            float pdf = selectionPdf * lightSample.pdf;
            float pdfArea = selectionPdf * lightSample.pdfArea;
            // Compute offset light sample position to reduce self-intersections at the light.
            // We compute the shadow ray parameters based on the offset position.
            float3 offsetPos = computeRayOrigin(lightSample.posW, lightSample.normalW);
            float3 toLight = offsetPos - rayOrigin;
            ls.rayDistance = length(toLight);
            ls.rayDir = normalize(toLight);

            // Setup returned sample.
            ls.dir = lightSample.dir;
            ls.distance = lightSample.distance;
            ls.pdf = pdf;
            ls.pdfArea = pdfArea;
            ls.Li = pdf > 0.f ? lightSample.Le * gScene.emissiveIntensityMultiplier / pdf : float3(0);

            outLightIndex = lightSample.triangleIndex + gScene.getLightCount();
            outLightUV = lightSample.uv;

            if (!valid)
            {
                return false;
            }

            return true;
        }
        u -= p[2];
    }

    return false;
}

SceneLightSample getAnalyticalLightSample(int lightIndex, float3 rayOrigin, inout SampleGenerator sg)
{
    AnalyticLightSample lightSample;
    bool valid = sampleLight(rayOrigin, gScene.getLight(lightIndex), sg, lightSample);

    // Reject sample if lower hemisphere.
    //if (!valid || dot(sd.N, lightSample.dir) < kMinCosTheta) return false;

    // Setup returned sample.
    // Analytic lights do not currently have a geometric representation in the scene.
    // Do not worry about adjusting the ray to avoid self-intersections at the light.
    SceneLightSample ls;
    ls.rayDir = ls.dir = lightSample.dir;
    ls.rayDistance = ls.distance = lightSample.distance;
    ls.pdf = lightSample.pdf;
    ls.Li = lightSample.Li;
    return ls;
}


void VolumeTrackingGVDB<VolumeTrackingAdapterGVDB:IVolumeTrackingAdapterGVDB>(Ray rWorld, int mipLevel, inout SampleGenerator sg, inout VolumeTrackingAdapterGVDB adapter, bool vertexCenter)
{
    int		nodeid[3];					// level variables
    float	tMax[3];
    int		b;

    float epsilon = 0.01;

    // GVDB - Iterative Hierarchical 3DDA on GPU
    float3 vmin;
    int lev = gvdb.top_lev[mipLevel];
    int topLev = lev;
    nodeid[lev] = 0;		// rootid ndx

    Ray ray = WorldToMedium(rWorld, mipLevel);

    if (vertexCenter)
    {
        ray.origin -= 0.5f;
    }

    float tNear, tFar;
    float3 sign = float3(ray.dir.x < 0.f ? -1.f : 1.f, ray.dir.y < 0.f ? -1.f : 1.f, ray.dir.z < 0.f ? -1.f : 1.f);

    if (!IntersectVolumeBound(ray, tNear, tFar, mipLevel, vertexCenter))
    {
        adapter.ExecuteEndStep();
        return;
    }

    adapter.SetRayInfo(tNear, tFar, ray);
    adapter.ExecuteStartStep();

    float3 tStart = float3(tNear, tFar, 0);	// intersect ray with bounding box	
    VDBNode node = getNode(gvdb, mipLevel, lev, nodeid[lev], vmin);			// get root VDB node	

    // 3DDA variables		
    tStart.x += epsilon;
    tMax[lev] = tStart.y;
    int iter = 0;

    HDDAState dda;
    dda.SetFromRay(ray.origin, ray.dir, tStart);
    dda.Prepare(vmin, gvdb.vdel[lev + mipLevel * 3]);

    if (vertexCenter)
    {
        int iter = 0;
        while (iter++<3 && (any(dda.p < 0 || dda.p > gvdb.res[lev + mipLevel * 3]))) // guarantee that the point "enters"
        {
            dda.Next();
            dda.Step();
            dda.t.x += epsilon;
        }
    }

    float t = tNear;

    float Tr = 0.f;

    for (; iter < 4096 && lev > 0 && lev <= topLev && all(dda.p >= 0 && dda.p <= gvdb.res[lev + mipLevel * 3]); iter++) {

        dda.Next();

        // node active test
        b = (((int(dda.p.z) << gvdb.dim[lev + mipLevel * 3]) + int(dda.p.y)) << gvdb.dim[lev + mipLevel * 3]) + int(dda.p.x);	// bitmaskpos

        uint childNodeId = getChild(gvdb, mipLevel, node, lev, b);

        if (childNodeId != ID_UNDEFL)
        {
            if (lev == 1) {										// enter brick function..S
                nodeid[0] = childNodeId;

                t = dda.t.x - epsilon;

                float3 vmin_leaf;
                VDBNode node = getNode(gvdb, mipLevel, 0, nodeid[0], vmin_leaf);				// Get the VDB leaf node
                float3 o = unpackValue(node.mPackedPosValue);	// Atlas sub-volume to trace	

                bool shouldExit = adapter.ExecuteMainStep(dda, vmin_leaf, o, node.mDensityBounds * gScene.volumeDesc.densityScaleFactorByScaling, mipLevel, t, sg);
                if (shouldExit) return;

                dda.Step();
                dda.t.x += epsilon;
            }
            else {
                lev--;													// step down tree
                nodeid[lev] = childNodeId;				// get child
                node = getNode(gvdb, mipLevel, lev, nodeid[lev], vmin);// child node
                tMax[lev] = dda.t.y;						// t.x = entry point, t.y = exit point							
                dda.Prepare(vmin, gvdb.vdel[lev + mipLevel * 3]);
            }
        }
        else
        {
            // empty voxel, step DDA
            dda.Step();
            dda.t.x += epsilon;
        }

        while (dda.t.x > tMax[lev] && lev <= topLev) {
            lev++;												// step up tree
            if (lev <= topLev) {
                node = getNode(gvdb, mipLevel, lev, nodeid[lev], vmin);
                dda.Prepare(vmin, gvdb.vdel[lev + mipLevel * 3]);				// restore dda at next level up
            }
        }
    }

    adapter.ExecuteEndStep();
}

float MediumTrAnalyticGeneric(Ray rWorld, int mipLevel, inout SampleGenerator sg, bool useLinearSampler)
{
    MediumTrAnalyticAdapterGVDB adapter;
    adapter.InitializeValues();
    adapter.useLinearSampler = useLinearSampler;
    // execute main function
    VolumeTrackingGVDB(rWorld, mipLevel, sg, adapter, useLinearSampler);
    return adapter.Tr;
}

void SampleMediumAnalyticGeneric(Ray rWorld, inout SampleGenerator sg, bool useLinearSampler, inout float hitDistances[MaxInitialSampleCount],
    int mipLevel, inout float pdf[MaxInitialSampleCount], inout float outTr[MaxInitialSampleCount], int numSamples)
{
    SampleMediumAnalyticAdapterGVDB adapter;
    adapter.InitializeValues();
    adapter.numSamples = numSamples;
    adapter.useLinearSampler = useLinearSampler;

    if (useLinearSampler)
        for (int i = 0; i < numSamples; i++)
        {
            adapter.outTr[i] = -log(1 - sampleNext1D(sg)); // must use 1-u here!
        }

    VolumeTrackingGVDB(rWorld, mipLevel, sg, adapter, useLinearSampler);

    hitDistances = adapter.hitDistances;
    outTr = adapter.outTr;
    pdf = adapter.pdf;
}

void SampleMediumSuperVoxelGeneric(Ray rWorld, inout SampleGenerator sg, inout MediumInteraction mi, int mipLevel)
{
    DecompositionTrackingGVDBAdapter adapter;
    adapter.InitializeValues();

    Ray rMip0 = WorldToMedium(rWorld, mipLevel);
    adapter.rMip0 = rMip0;
    adapter.rWorld = rWorld;
    adapter.mi = mi;
    VolumeTrackingGVDB(rWorld, mipLevel, sg, adapter, false);
    mi = adapter.mi;
}

float MediumTrResidualRatioTrackingGeneric(Ray rWorld, int mipLevel, inout SampleGenerator sg, bool useAnalogResidual, bool useGlobalMajorant)
{
    ResidualRatioTrackingGVDBAdapter adapter;
    adapter.InitializeValues();
    Ray rMip0 = WorldToMedium(rWorld, mipLevel);
    adapter.rMip0 = rMip0;
    adapter.useGlobalMajorant = useGlobalMajorant;
    adapter.useAnalogResidual = useAnalogResidual || useGlobalMajorant;
    VolumeTrackingGVDB(rWorld, mipLevel, sg, adapter, false);
    float Tr = adapter.Tr;
    return Tr;
}


float2 SampleVolumeCellByDensityGeneric(Ray rWorld, inout SampleGenerator sg, int mipLevel, out float densityBound)
{
    SampleVolumeCellByDensityAdapterGVDB adapter;
    adapter.InitializeValues();
    VolumeTrackingGVDB(rWorld, mipLevel, sg, adapter, false);
    return adapter.selectedInterval;
}

float MediumTrRayMarchingGeneric(Ray rWorld, int mipLevel, bool useLinearSampler, float tStepScale, inout SampleGenerator sg)
{
    int effectiveMipLevel = mipLevel >= kPrevFrameDensityGridOffset ? mipLevel - kPrevFrameDensityGridOffset : mipLevel;
    effectiveMipLevel = effectiveMipLevel >= kNumMaxMips ? effectiveMipLevel - kNumMaxMips : effectiveMipLevel;

    MediumTrRayMarchingAdapterGVDB adapter;
    adapter.InitializeValues();

    adapter.tStep = gScene.volumeDesc.tStep * gScene.volumeWorldScaling * tStepScale * (effectiveMipLevel + 1);
    adapter.useLinearSampler = useLinearSampler;
    VolumeTrackingGVDB(rWorld, mipLevel, sg, adapter, false);
    return adapter.Tr;
}

void ReservoirFeatureRayMarchingGeneric(Ray rWorld, inout SampleGenerator sg, int mipLevel, bool useLinearSampler, float tStepScale, inout float accuTransmittance)
{
    ReservoirFeatureRayMarchingAdapterGVDB adapter;
    adapter.InitializeValues();
    float3 volumeCenterW = mul(float4((gvdb.bmin[mipLevel] + gvdb.bmax[mipLevel]) * 0.5f, 1.f), gvdb.xform[mipLevel]).xyz;
    float3 diagonalVector = mul(float4(gvdb.bmax[mipLevel] - gvdb.bmin[mipLevel], 1.f), gvdb.xform[mipLevel]).xyz;
    float volumeDiagonalLength = length(diagonalVector);
    float farDepth = length(volumeCenterW - rWorld.origin) + 10 * volumeDiagonalLength;
    adapter.farDepth = farDepth;
    adapter.tSurface = rWorld.tMax;
    adapter.tStep = gScene.volumeDesc.tStep * gScene.volumeWorldScaling * tStepScale;
    adapter.useLinearSampler = useLinearSampler; 
    VolumeTrackingGVDB(rWorld, mipLevel, sg, adapter, false);
    accuTransmittance = adapter.accuTransmittance;
}


#ifdef SURFACE_SCENE
float3 directLightingSurface(inout SampleGenerator sg, ShadingData shadingInfo, EnvMapSampler envMapSampler, EmissiveLightSampler emissiveSampler, int kLightSamplesPerVertex, bool kUseEnvironmentLights,
    bool kUseAnalyticLights, bool kUseEmissiveLights, bool enableShadow, int mipLevel, uint trackingMethod)
{
    // Sample the scene lights.
    float3 Ld = 0.f;

    float3 posW = shadingInfo.posW;

    for (int lightSampleId = 0; lightSampleId < kLightSamplesPerVertex; lightSampleId++)
    {
        SceneLightSample ls;
        int outLightIndex;
        float2 outLightUV;

        bool valid = sampleSceneLights(envMapSampler, emissiveSampler, posW, kUseEnvironmentLights, kUseAnalyticLights, kUseEmissiveLights, sg, ls, outLightIndex, outLightUV, shadingInfo.N);
        if (!valid) continue;
        ls.pdf = 1.f;

        Ray shadowRay = { posW, ls.rayDir, 0, ls.rayDistance };

        if (enableShadow)
        {
            float visibility = computeGeneralPathSpaceVisibility(true, true, shadowRay, sg, 1, mipLevel, true, trackingMethod, 1.f);
            ls.Li *= visibility;
        }

        float3 f = evalBSDFCosine(shadingInfo, ls.rayDir);
        Ld += f * ls.Li / ls.pdf;
    }

    Ld /= kLightSamplesPerVertex;

    return Ld;
}
#endif

// Warning: the lighting result here is already times with phase function and divided by pdf!
float3 directLighting(inout SampleGenerator sg, MediumInteraction mi, EnvMapSampler envMapSampler, EmissiveLightSampler emissiveSampler, int kLightSamplesPerVertex, bool kUseEnvironmentLights,
    bool kUseAnalyticLights, bool kUseEmissiveLights, bool enableShadow, int mipLevel, uint trackingMethod)
{
    // Sample the scene lights.
    float3 Ld = 0.f;

    for (int lightSampleId = 0; lightSampleId < kLightSamplesPerVertex; lightSampleId++)
    {
        SceneLightSample ls;
        int outLightIndex;
        float2 outLightUV;
        bool valid = sampleSceneLights(envMapSampler, emissiveSampler, mi.p, kUseEnvironmentLights, kUseAnalyticLights, kUseEmissiveLights, sg, ls, outLightIndex, outLightUV);
        if (!valid) continue;
        ls.pdf = 1.f;
        Ray shadowRay = { mi.p, ls.rayDir, 0, ls.rayDistance };
        if (enableShadow)
        {
#ifdef SURFACE_SCENE
            float visibility = computeGeneralPathSpaceVisibility(true, true, shadowRay, sg, 1, mipLevel, true, trackingMethod, 1.f);
#else
            float visibility = computeVisibility(shadowRay, sg, 1, mipLevel, true, trackingMethod, 1.f);
#endif
            ls.Li *= visibility;
        }
        float p = mi.phaseFunction(mi.wo, ls.dir);
        float3 f = p;

        Ld += f * ls.Li / ls.pdf;
    }

    Ld /= kLightSamplesPerVertex;

    return Ld;
}

float3 SampleDirectLighting(inout SampleGenerator sg, out float pdf, MediumInteraction mi, EnvMapSampler envMapSampler, EmissiveLightSampler emissiveSampler,
    bool kUseEnvironmentLights, bool kUseAnalyticLights, bool kUseEmissiveLights, bool enableShadow, int mipLevel, uint trackingMethod, bool useLinearSampler, int lightVisibilitySamples, float tStepScale,
    out int outLightIndex, out float2 outLightUV, out float outVisibility)
{
    pdf = 0.f;
    // Sample the scene lights.
    float3 Ld = 0.f;

    SceneLightSample ls;

    bool valid = sampleSceneLights(envMapSampler, emissiveSampler, mi.p, kUseEnvironmentLights, kUseAnalyticLights, kUseEmissiveLights, sg, ls, outLightIndex, outLightUV);
    pdf = ls.pdfArea;
    if (!valid)
    {
        pdf = 0.f;
        return 0.f;
    }

    ls.pdf = 1.f; // ls.Li is already pre-divided
    Ray shadowRay = { mi.p, ls.rayDir, 0, ls.rayDistance };
    if (enableShadow)
    {
#ifdef SURFACE_SCENE
        // skip surface shadow for initial sampling
        float visibility = computeGeneralPathSpaceVisibility(true, false, shadowRay, sg, lightVisibilitySamples, mipLevel, useLinearSampler, trackingMethod, tStepScale);
#else
        float visibility = computeVisibility(shadowRay, sg, lightVisibilitySamples, mipLevel, useLinearSampler, trackingMethod, tStepScale);
#endif

        outVisibility = visibility;
        ls.Li *= visibility;
    }
    float p = mi.phaseFunction(mi.wo, ls.dir);
    float3 f = p;

    Ld += f * ls.Li / ls.pdf;

    return Ld;
}

#ifdef SURFACE_SCENE
float3 SampleDirectLightingSurface(inout SampleGenerator sg, out float pdf, ShadingData shadingInfo, EnvMapSampler envMapSampler, EmissiveLightSampler emissiveSampler,
    bool kUseEnvironmentLights, bool kUseAnalyticLights, bool kUseEmissiveLights, bool enableShadow, int mipLevel, uint trackingMethod, bool useLinearSampler, int lightVisibilitySamples, float tStepScale,
    out int outLightIndex, out float2 outLightUV, out float outVisibility, bool allowRussianRoulette = false, bool clampValue = false)
{
    pdf = 0.f;
    // Sample the scene lights.
    float3 Ld = 0.f;

    SceneLightSample ls;
    bool valid = sampleSceneLights(envMapSampler, emissiveSampler, shadingInfo.posW, kUseEnvironmentLights, kUseAnalyticLights, kUseEmissiveLights, sg, ls, outLightIndex, outLightUV, shadingInfo.N);
    pdf = ls.pdfArea;
    if (!valid)
    {
        pdf = 0.f;
        return 0.f;
    }

    ls.pdf = 1.f; // ls.Li is already pre-divided
    Ray shadowRay = { shadingInfo.posW, ls.rayDir, 0, ls.rayDistance };
    if (enableShadow)
    {
        // skip surface shadow for initial sampling
        float visibility = computeGeneralPathSpaceVisibility(true, false, shadowRay, sg, lightVisibilitySamples, mipLevel, useLinearSampler, trackingMethod, tStepScale);
        outVisibility = visibility;
        ls.Li *= visibility;
    }

    float3 f = evalBSDFCosine(shadingInfo, ls.rayDir);
    Ld += f * ls.Li / ls.pdf;

    return Ld;
}
#endif

#ifdef SURFACE_SCENE
// consider both surface and volume occlusion
float computeGeneralPathSpaceVisibility(bool includeVolume, bool includeSurface, Ray ray, inout SampleGenerator sg, int visibilitySamples, int mipLevel, bool useLinearSampler, uint trackingMethod, float tStepScale = 1.f, bool cullNonOpaqueGeometry = false)
{
    float visibility = 1.f;
    // shoot ray to light
    bool isOccludedBySurface = false;
    if (includeSurface)
        isOccludedBySurface = FindIfOccluded(ray.origin, ray.dir, ray.tMin, ray.tMax, cullNonOpaqueGeometry);

    if (isOccludedBySurface)
    {
        visibility = 0.f;
    }
    else
    {
        if (includeVolume)
            visibility = computeVisibility(ray, sg, 1, mipLevel, true, trackingMethod, 1.f);
    }
    return visibility;
}
#endif

// Reduce the number of cases to speed up shader compilation
float computeVisibility(Ray ray, inout SampleGenerator sg, int visibilitySamples, int mipLevel, bool useLinearSampler, uint trackingMethod, float tStepScale = 1.f)
{
    float visibility = 0;

    for (int i = 0; i < visibilitySamples; i++)
    {
        if (trackingMethod == kRatioTracking || trackingMethod == kResidualRatioTracking || trackingMethod == kAnalogResidualRatioTracking)
            visibility += MediumTrResidualRatioTrackingGeneric(ray, mipLevel, sg, trackingMethod == kAnalogResidualRatioTracking, trackingMethod == kRatioTracking);
        else
        if (trackingMethod == kRayMarching)
            visibility += MediumTrRayMarchingGeneric(ray, mipLevel, useLinearSampler, tStepScale, sg);
        else if (trackingMethod == kAnalyticTracking)
            visibility += MediumTrAnalyticGeneric(ray, mipLevel, sg, useLinearSampler);
    }

    visibility /= visibilitySamples;

    return visibility;
}

float RejectionSampleRandomPointByDensity(Ray rWorld, inout SampleGenerator sg, int mipLevel)
{
    float densityBound;
    float2 selectedInterval = SampleVolumeCellByDensityGeneric(rWorld, sg, mipLevel, densityBound);
    if (selectedInterval.x == -1) return kRayTMax;

    float sampledDepth = selectedInterval.x + (selectedInterval.y - selectedInterval.x) * sampleNext1D(sg);
    float sampledY = sampleNext1D(sg) * densityBound;
    return sampledDepth;
}

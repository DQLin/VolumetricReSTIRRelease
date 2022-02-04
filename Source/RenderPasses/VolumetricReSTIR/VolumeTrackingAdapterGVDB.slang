__exported import VolumeBase;

// This might not be enough for some rare situations
#define MAX_BRICK_STEPS 128 

interface IVolumeTrackingAdapterGVDB
{
    [mutating] void SetRayInfo(float tNear_, float tFar_, Ray ray_);

    [mutating] void InitializeValues();

    [mutating] void ExecuteStartStep();

    // return: should terminate
    [mutating] bool ExecuteMainStep(in HDDAState dda, float3 vmin_leaf, float3 o, float4 brickDensityBounds, int mipLevel, inout float t, inout SampleGenerator sg);

    [mutating] void ExecuteEndStep();
};

struct MediumTrAnalyticAdapterGVDB : IVolumeTrackingAdapterGVDB
{
    float Tr = 0.f;
    bool useLinearSampler;
    float tNear;
    float tFar;
    Ray ray;
    bool hasInitialized = false;

    [mutating] void InitializeValues()
    {
        Tr = 0.f;
        hasInitialized = false;
    }

    [mutating] void SetRayInfo(float tNear_, float tFar_, Ray ray_)
    {
        tNear = tNear_;
        tFar = tFar_;
        ray = ray_;
    }

    [mutating] void ExecuteStartStep()
    {
    }

    [mutating] bool SkipNode(in HDDAState dda, float4 brickDensityBounds, inout float t, inout bool shouldTerminate)
    {
        return false;
    }

    [mutating] bool ExecuteMainStep(in HDDAState dda, float3 vmin_leaf, float3 o, float4 brickDensityBounds, int mipLevel, inout float t, inout SampleGenerator sg)
    {
        HDDAState dda_leaf = dda;

        dda_leaf.PrepareLeaf(vmin_leaf);

        for (int iter = 0; iter < MAX_BRICK_STEPS && all(dda_leaf.p >= 0 && dda_leaf.p < gvdb.res[mipLevel * 3]); iter++)
        {
            // compute the next step location, but don't step yet
            dda_leaf.Next();

            float maxDeltaT = dda_leaf.t.y - t;

            if (useLinearSampler)
            {
                // trilinear regular tracking
                float4 densities[2];

                FetchEightVoxelsInAtlas(dda_leaf.p + o + 0.5f, mipLevel, densities);

                densities[0] *= gScene.volumeDesc.sigma_t;
                densities[1] *= gScene.volumeDesc.sigma_t;

                float v_000 = densities[0][0];
                float v_100 = densities[0][1];
                float v_010 = densities[0][2];
                float v_110 = densities[0][3];
                float v_001 = densities[1][0];
                float v_101 = densities[1][1];
                float v_011 = densities[1][2];
                float v_111 = densities[1][3];

                float mxyz = v_111 - v_011 - v_101 - v_110 + v_100 + v_010 + v_001 - v_000;
                float mxy = v_000 - v_100 - v_010 + v_110;
                float mxz = v_000 - v_100 - v_001 + v_101;
                float myz = v_000 - v_010 - v_001 + v_011;
                float mx = v_100 - v_000;
                float my = v_010 - v_000;
                float mz = v_001 - v_000;

                float3 d = ray.dir;
                float3 p0 = dda_leaf.pos + dda_leaf.t.x * dda_leaf.dir - (dda_leaf.p + vmin_leaf);
                float c3 = mxyz * d.x * d.y * d.z;
                float c2 = (p0.z * d.x * d.y + p0.y * d.x * d.z + p0.x * d.y * d.z) * mxyz + mxy * d.x * d.y + mxz * d.x * d.z + myz * d.y * d.z;
                float c1 = (p0.y * p0.z * d.x + p0.x * p0.z * d.y + p0.x * p0.y * d.z) * mxyz +
                    mx * d.x + my * d.y + mz * d.z + (p0.y * d.x + p0.x * d.y) * mxy + (p0.z * d.x + p0.x * d.z) * mxz + (p0.z * d.y + p0.y * d.z) * myz;
                float c0 = p0.x * p0.y * p0.z * mxyz + p0.x * p0.y * mxy + p0.x * p0.z * mxz + p0.y * p0.z * myz + p0.x * mx + p0.y * my + p0.z * mz + v_000;

                float t_dist = min(tFar - t, maxDeltaT);

                float t2 = t_dist * t_dist;
                float t3 = t2 * t_dist;
                float t4 = t2 * t2;

                Tr += -(c3 * t4 / 4 + c2 * t3 / 3 + c1 * t2 / 2 + c0 * t_dist);
            }
            else
            {
                // find density
                float density = DensityInAtlas(dda_leaf.p + o + 0.5f, mipLevel, false);

                float sigma_t = density * gScene.volumeDesc.sigma_t;

                Tr += -min(tFar - t, maxDeltaT) * sigma_t;
            }

            if (t + maxDeltaT >= tFar)
            {
                Tr = exp(Tr);
                return true;
            }

            t += maxDeltaT;
            dda_leaf.Step();
        }
        return false;
    }

    [mutating] void ExecuteEndStep()
    {
        //if (hasInitialized) Tr = 0.f;
        //else Tr = 1.f;
        Tr = exp(Tr);
    }

}


// TODO: consider the case where whole brick has constant density
struct MediumTrRayMarchingAdapterGVDB : IVolumeTrackingAdapterGVDB
{
    float Tr = 0.f;
    bool useLinearSampler;
    float tNear;
    float tFar;
    Ray ray;
    float tStep; // ray marching step

    bool hasInitialized = false;

    [mutating] void InitializeValues()
    {
        Tr = 0.f;
        hasInitialized = false;
    }

    [mutating] void SetRayInfo(float tNear_, float tFar_, Ray ray_)
    {
        tNear = tNear_;
        tFar = tFar_;
        ray = ray_;
    }

    [mutating] void ExecuteStartStep()
    {
        hasInitialized = true;
    }


    [mutating] bool ExecuteMainStep(in HDDAState dda, float3 vmin_leaf, float3 o, float4 brickDensityBounds, int mipLevel, inout float t, inout SampleGenerator sg)
    {
        //t = tStep * ceil(t / tStep);	// Start on sampling wavefront (avoids subvoxel banding artifacts)
        t = tNear + (floor((t - tNear) / tStep) + 0.5f) * tStep;
        if (t < dda.t.x) t += tStep;


        float tStepMultipler = 1.f;

        {
            float3 wp = ray.origin + t * ray.dir;				// Sample position in index space
            float3 p = wp - vmin_leaf;					// Sample point in sub-volume (in units of voxels)
            const float3 wpt = tStepMultipler * tStep * ray.dir;

            for (int iter = 0; iter < MAX_BRICK_STEPS && all(p >= 0 && p < gvdb.res[mipLevel * 3]); iter++)
            {
                if (t >= tFar)
                {
                    Tr = exp(Tr);
                    return true;
                }
                // find density
                float density = DensityInAtlas(p + o, mipLevel, useLinearSampler);
                float sigma_t = density * gScene.volumeDesc.sigma_t;
                Tr += -sigma_t * (iter == 0 ? 1.f : tStepMultipler) * tStep;
                p += wpt;
                t += tStepMultipler * tStep;
            }
        }

        return false;
    }

    [mutating] void ExecuteEndStep()
    {
        if (hasInitialized) Tr = exp(Tr);
        else Tr = 1.f;
    }
}


// TODO: consider the case where whole brick has constant density
struct SampleMediumAnalyticAdapterGVDB : IVolumeTrackingAdapterGVDB
{
    float hitDistances[MaxInitialSampleCount];
    float outTr[MaxInitialSampleCount]; // also used to store sampled optical thickness
    float pdf[MaxInitialSampleCount];
    int numSamples;
    float opticalThickness = 0.f;
    float tNear;
    float tFar;
    Ray ray;
    bool hasInitialized = false;
    bool useLinearSampler = false;

    [mutating] void InitializeValues()
    {
        opticalThickness = 0.f;
        hasInitialized = false;
    }

    [mutating] void ExecuteStartStep()
    {
        for (int sampleId = 0; sampleId < numSamples; sampleId++) {
            hitDistances[sampleId] = -1;
        }
        hasInitialized = true;
    }

    [mutating] void SetRayInfo(float tNear_, float tFar_, Ray ray_)
    {
        tNear = tNear_;
        tFar = tFar_;
        ray = ray_;
    }


    float computeTrilinearOpticalThickness(float t, float c3, float c2, float c1, float c0)
    {
        float t2 = t * t;
        float t3 = t2 * t;
        float t4 = t2 * t2;
        return c3 * t4 / 4 + c2 * t3 / 3 + c1 * t2 / 2 + c0 * t;
    }

    float computeTrilinearSigmaT(float t, float c3, float c2, float c1, float c0)
    {
        float t2 = t * t;
        float t3 = t2 * t;
        return c3 * t3 + c2 * t2 + c1 * t + c0;
    }


    [mutating] bool SkipNode(in HDDAState dda, float4 brickDensityBounds, inout float t, inout bool shouldTerminate)
    {
        return false;
    }


    [mutating] bool ExecuteMainStep(in HDDAState dda, float3 vmin_leaf, float3 o, float4 brickDensityBounds, int mipLevel, inout float t, inout SampleGenerator sg)
    {
        HDDAState dda_leaf = dda;
        dda_leaf.PrepareLeaf(vmin_leaf);

        for (int iter = 0; iter < MAX_BRICK_STEPS && all(dda_leaf.p >= 0 && dda_leaf.p < gvdb.res[mipLevel * 3]); iter++)
        {
            // compute the next step location, but don't step yet
            dda_leaf.Next();

            float maxDeltaT = min(tFar - t, dda_leaf.t.y - t);

            float currentTMax = min(tFar, dda_leaf.t.y);

            int finishedCount = 0;

            float deltaThickness = 0.f;

            if (useLinearSampler)
            {
                // trilinear regular tracking
                float4 densities[2];
                FetchEightVoxelsInAtlas(dda_leaf.p + o + 0.5f, mipLevel, densities);

                densities[0] *= gScene.volumeDesc.sigma_t;
                densities[1] *= gScene.volumeDesc.sigma_t;

                float v_000 = densities[0][0];
                float v_100 = densities[0][1];
                float v_010 = densities[0][2];
                float v_110 = densities[0][3];
                float v_001 = densities[1][0];
                float v_101 = densities[1][1];
                float v_011 = densities[1][2];
                float v_111 = densities[1][3];


                float mxyz = v_111 - v_011 - v_101 - v_110 + v_100 + v_010 + v_001 - v_000;
                float mxy = v_000 - v_100 - v_010 + v_110;
                float mxz = v_000 - v_100 - v_001 + v_101;
                float myz = v_000 - v_010 - v_001 + v_011;
                float mx = v_100 - v_000;
                float my = v_010 - v_000;
                float mz = v_001 - v_000;

                float3 d = ray.dir;
                float3 p0 = dda_leaf.pos + dda_leaf.t.x * dda_leaf.dir - (dda_leaf.p + vmin_leaf);
                float c3 = mxyz * d.x * d.y * d.z;
                float c2 = (p0.z * d.x * d.y + p0.y * d.x * d.z + p0.x * d.y * d.z) * mxyz + mxy * d.x * d.y + mxz * d.x * d.z + myz * d.y * d.z;
                float c1 = (p0.y * p0.z * d.x + p0.x * p0.z * d.y + p0.x * p0.y * d.z) * mxyz +
                    mx * d.x + my * d.y + mz * d.z + (p0.y * d.x + p0.x * d.y) * mxy + (p0.z * d.x + p0.x * d.z) * mxz + (p0.z * d.y + p0.y * d.z) * myz;
                float c0 = p0.x * p0.y * p0.z * mxyz + p0.x * p0.y * mxy + p0.x * p0.z * mxz + p0.y * p0.z * myz + p0.x * mx + p0.y * my + p0.z * mz + v_000;

                deltaThickness = computeTrilinearOpticalThickness(maxDeltaT, c3, c2, c1, c0);

                // backtrack sampler
                for (int sampleId = 0; sampleId < numSamples; sampleId++)
                {
                    if (hitDistances[sampleId] == -1)
                    {
                        // opticalThickness is positive here!
                        if (opticalThickness + deltaThickness >= outTr[sampleId])
                        {
                            // find hit point

                            // regular falsi

                            float tau_target = outTr[sampleId] - opticalThickness;

                            float t_low = 0; float t_high = maxDeltaT;
                            float tau_low = 0; float tau_high = deltaThickness;

                            float t_sol = 0;

                            int iter = 0;

                            while (iter++ < 32 && t_high - t_low > maxDeltaT * 0.001f) // one thousandth of voxel intersection length
                            {
                                t_sol = t_low + (t_high - t_low) * (tau_target - tau_low) / (tau_high - tau_low);
                                float tau = computeTrilinearOpticalThickness(t_sol, c3, c2, c1, c0);
                                if (tau < tau_target) { t_low = t_sol; tau_low = tau; }
                                else { t_high = t_sol; tau_high = tau; }
                            }

                            hitDistances[sampleId] = t + t_sol;
                            outTr[sampleId] = exp(-outTr[sampleId]);

                            pdf[sampleId] = computeTrilinearSigmaT(t_sol, c3, c2, c1, c0) * outTr[sampleId];
                            finishedCount++;
                        }
                    }
                    else finishedCount++;
                }
            }
            else
            {
                // find density
                float density = DensityInAtlas(dda_leaf.p + o + 0.5f, mipLevel, false);

                float sigma_t = density * gScene.volumeDesc.sigma_t;

                for (int sampleId = 0; sampleId < numSamples; sampleId++)
                {
                    if (hitDistances[sampleId] == -1)
                    {
                        float dT = -log(1 - sampleNext1D(sg)) / sigma_t;
                        float curT = t + dT;
                        if (isnan(curT) || isinf(curT)) curT = kRayTMax;

                        if (curT < currentTMax)
                        {
                            hitDistances[sampleId] = curT;
                            outTr[sampleId] = exp(-(dT * sigma_t + opticalThickness));
                            pdf[sampleId] = sigma_t * outTr[sampleId];
                            finishedCount++;
                        }
                    }
                    else finishedCount++;
                }

                deltaThickness = maxDeltaT * sigma_t;
            }


            if (finishedCount == numSamples) return true;

            t = currentTMax;

            opticalThickness += deltaThickness;

            if (t >= tFar)
            {
                ExecuteEndStep();
                return true;
            }

            dda_leaf.Step();
        }

        return false;
    }


    [mutating] void ExecuteEndStep()
    {
        if (hasInitialized)
        {
            for (int sampleId = 0; sampleId < numSamples; sampleId++)
            {
                if (hitDistances[sampleId] == -1)
                {
                    hitDistances[sampleId] = kRayTMax;
                    outTr[sampleId] = exp(-opticalThickness);
                    pdf[sampleId] = outTr[sampleId];
                }
            }
        }
        else
        {
            for (int sampleId = 0; sampleId < numSamples; sampleId++)
            {
                hitDistances[sampleId] = kRayTMax;
                outTr[sampleId] = 1.f;
                pdf[sampleId] = 1.f;
            }
        }
    }
};


struct SampleVolumeCellByDensityAdapterGVDB : IVolumeTrackingAdapterGVDB
{
    int densityBound = 0.f;
    float2 selectedInterval = float2(-1, -1);
    float tNear;
    float tFar;
    Ray ray;
    float runningSum = 0.f;
    float Tr = 0.f;

    [mutating] void InitializeValues()
    {
        densityBound = 0.f;
        selectedInterval = float2(-1, -1);
        runningSum = 0.f;
        Tr = 0.f;
    }

    [mutating] void ExecuteStartStep()
    {
        selectedInterval = float2(-1, -1);
    }

    [mutating] void SetRayInfo(float tNear_, float tFar_, Ray ray_)
    {
        tNear = tNear_;
        tFar = tFar_;
        ray = ray_;
    }


    [mutating] bool ExecuteMainStep(in HDDAState dda, float3 vmin_leaf, float3 o, float4 brickDensityBounds, int mipLevel, inout float t, inout SampleGenerator sg)
    {
        HDDAState dda_leaf = dda;
        dda_leaf.PrepareLeaf(vmin_leaf);

        for (int iter = 0; iter < MAX_BRICK_STEPS && all(dda_leaf.p >= 0 && dda_leaf.p < gvdb.res[mipLevel * 3]); iter++)
        {
            // find density
            float density = DensityInAtlas(dda_leaf.p + o + 0.5f, mipLevel, false);

            // compute the next step location, but don't step yet
            dda_leaf.Next();

            float maxDeltaT = dda_leaf.t.y - t;

            float sigma_t = density * gScene.volumeDesc.sigma_t;

            float weight = exp(Tr) * sigma_t;

            runningSum += weight;

            if (runningSum > 0.f && sampleNext1D(sg) < weight / runningSum)
            {
                densityBound = density;
                selectedInterval = float2(t, min(tFar, t + maxDeltaT));
            }

            Tr += -maxDeltaT * sigma_t;

            if (t + maxDeltaT >= tFar)
            {
                return true;
            }

            t += maxDeltaT;

            dda_leaf.Step();
        }
        return false;
    }


    [mutating] void ExecuteEndStep()
    {
    }
};


struct ReservoirFeatureRayMarchingAdapterGVDB : IVolumeTrackingAdapterGVDB
{
    float accuTransmittance = 1.f;
    float tNear;
    float tFar;
    float tSurface;
    Ray ray;
    float farDepth; // need to provide farDepth
    float Tr = 0.f;
    bool useLinearSampler;
    float tStep;

    bool earlyTerminate = false;
    bool hasInitialized = false;


    [mutating] void InitializeValues()
    {
        accuTransmittance = 1.f;
        Tr = 0.f;
        hasInitialized = false;
        earlyTerminate = false;
    }

    [mutating] void ExecuteStartStep()
    {
        hasInitialized = true;
    }


    [mutating] bool SkipNode(in HDDAState dda, float4 brickDensityBounds, inout float t, inout bool shouldTerminate)
    {
        return false;
    }


    [mutating] bool ExecuteMainStep(in HDDAState dda, float3 vmin_leaf, float3 o, float4 brickDensityBounds, int mipLevel, inout float t, inout SampleGenerator sg)
    {
        //t = tStep * ceil(t / tStep);	// Start on sampling wavefront (avoids subvoxel banding artifacts)
        t = tNear + (floor((t - tNear) / tStep) + 0.5f) * tStep;
        if (t < dda.t.x) t += tStep;

        float3 wp = ray.origin + t * ray.dir;				// Sample position in index space
        float3 p = wp - vmin_leaf;					// Sample point in sub-volume (in units of voxels)
        const float3 wpt = tStep * ray.dir;

        for (int iter = 0; iter < MAX_BRICK_STEPS && all(p >= 0 && p < gvdb.res[mipLevel * 3]); iter++)
        {
            float curTransmittance = exp(Tr);
            if (curTransmittance < 0.01f)
            {
                earlyTerminate = true;
                ExecuteEndStep();
                return true;
            }

            // find density
            float density = DensityInAtlas(p + o, mipLevel, useLinearSampler);
            float sigma_t = density * gScene.volumeDesc.sigma_t;

            float negOpticalLength = -sigma_t * tStep;
            float weight = (1 - exp(negOpticalLength)) * curTransmittance;
            Tr += negOpticalLength;
            p += wpt;
            t += tStep;
        }

        return false;
    }

    [mutating] void SetRayInfo(float tNear_, float tFar_, Ray ray_)
    {
        tNear = tNear_;
        tFar = tFar_;
        ray = ray_;
    }

    [mutating] void ExecuteEndStep()
    {
        if (hasInitialized)
        {
            accuTransmittance = exp(Tr);
        }
    }
}


//
struct DecompositionTrackingGVDBAdapter : IVolumeTrackingAdapterGVDB
{
    float tNear;
    float tFar;
    Ray ray; // ray in fine medium
    Ray rMip0;
    Ray rWorld;
    MediumInteraction mi;

    [mutating] void InitializeValues()
    {
    }

    [mutating] void ExecuteStartStep()
    {
    }

    [mutating] void SetRayInfo(float tNear_, float tFar_, Ray ray_)
    {
        tNear = tNear_;
        tFar = tFar_;
        ray = ray_;
    }


    [mutating] bool SkipNode(in HDDAState dda, float4 brickDensityBounds, inout float t, inout bool shouldTerminate)
    {
        return false;
    }


    [mutating] bool ExecuteMainStep(in HDDAState dda, float3 vmin_leaf, float3 o, float4 brickDensityBounds, int mipLevel, inout float t, inout SampleGenerator sg)
    {
        float minDensity = brickDensityBounds.x;
        float maxDensity = brickDensityBounds.y;

        float maxDeltaT = min(tFar - t, dda.t.y - t);
        float currentTMax = min(tFar, dda.t.y);

        float t_control;
        if (minDensity == 0.f) t_control = kRayTMax;
        else t_control = t - log(1 - sampleNext1D(sg)) / (minDensity * gScene.volumeDesc.sigma_t);
        float invMaxDensity = 1.f / (maxDensity - minDensity);

        if (maxDensity - minDensity > 0.f)
        {
            // Run delta-tracking iterations to sample a medium interaction
            while (true) {
                t -= log(1 - sampleNext1D(sg)) * invMaxDensity / gScene.volumeDesc.sigma_t;

                if (t >= t_control || t >= currentTMax) break;

                float3 wp = ray.origin + t * ray.dir;				// Sample position in index space
                float3 p = wp - vmin_leaf;					// Sample point in sub-volume (in units of voxels)
                float density = DensityInAtlas(p + o, mipLevel, true);

                if ((density - minDensity) * invMaxDensity > sampleNext1D(sg)) {
                    // Populate _mi_ with medium interaction information and return
                    mi = { rWorld.at(t), -rWorld.dir, gScene.volumeDesc.PhaseFunctionConstantG, true };
                    return true;
                }
            }

            t = min(t_control, t);

            if (t < currentTMax)
            {
                mi = { rWorld.at(t), -rWorld.dir, gScene.volumeDesc.PhaseFunctionConstantG, true };
                return true;
            }

            t = currentTMax;

            if (t >= tFar)
            {
                ExecuteEndStep();
                return true;
            }
        }
        else
        {
            if (t_control < currentTMax)
            {
                mi = { rWorld.at(t_control), -rWorld.dir, gScene.volumeDesc.PhaseFunctionConstantG, true };
                return true;
            }
            else
            {
                t = currentTMax;
                return false;
            }
        }

        return false;
    }


    [mutating] void ExecuteEndStep()
    {
        mi.isValid = false;
    }
}

//
struct ResidualRatioTrackingGVDBAdapter : IVolumeTrackingAdapterGVDB
{
    float tNear;
    float tFar;
    Ray ray; // ray in fine medium
    Ray rMip0;
    float Tr;
    bool useAnalogResidual;
    bool useGlobalMajorant;

    [mutating] void InitializeValues()
    {
        Tr = 1.f;
    }

    [mutating] void ExecuteStartStep()
    {
        if (useGlobalMajorant) useAnalogResidual = true;
    }

    [mutating] void SetRayInfo(float tNear_, float tFar_, Ray ray_)
    {
        tNear = tNear_;
        tFar = tFar_;
        ray = ray_;
    }


    [mutating] bool SkipNode(in HDDAState dda, float4 brickDensityBounds, inout float t, inout bool shouldTerminate)
    {
        return false;
    }


    [mutating] bool ExecuteMainStep(in HDDAState dda, float3 vmin_leaf, float3 o, float4 brickDensityBounds, int mipLevel, inout float t, inout SampleGenerator sg)
    {
        float mu_min = useGlobalMajorant ? 0.f : brickDensityBounds.x * gScene.volumeDesc.sigma_t;
        float mu_max = useGlobalMajorant ? GetVolumeMaxDensity(mipLevel) * gScene.volumeDesc.sigma_t : brickDensityBounds.y * gScene.volumeDesc.sigma_t;
        float mu_avg = brickDensityBounds.z * gScene.volumeDesc.sigma_t;

        float maxDeltaT = min(tFar - t, dda.t.y - t);
        float currentTMax = min(tFar, dda.t.y);
        float mu_r_temp = mu_max - mu_min;
        float D = gvdb.superVoxelWorldSpaceDiagonalLength;// diagonal length of supervoxel
        float gamma = 2;
        float mu_c = (mu_r_temp == 0.f || useAnalogResidual) ? mu_min : min(mu_avg, max(mu_min, mu_min + mu_r_temp * (pow(gamma, 1.f / (D * mu_r_temp)) - 1)));
        float mu_r = max(mu_c - mu_min, mu_max - mu_c);
        float inv_mu_r = 1.f / mu_r;
        float T_c = exp(-mu_c * min(tFar - t, maxDeltaT));
        float T_r = 1;

        if (mu_r > 0.f)
        {
            while (true) {
                t -= log(1 - sampleNext1D(sg)) * inv_mu_r;

                if (t >= currentTMax)
                {
                    break;
                }

                float3 wp = ray.origin + t * ray.dir;				// Sample position in index space
                float3 p = wp - vmin_leaf;					// Sample point in sub-volume (in units of voxels)
                float density = DensityInAtlas(p + o, mipLevel, true);

                float mu = density * gScene.volumeDesc.sigma_t;
                T_r *= 1 - (mu - mu_c) * inv_mu_r;
            }
        }

        Tr *= T_c * T_r;
        t = currentTMax;

        if (t >= tFar)
        {
            return true;
        }

        return false;
    }

    [mutating] void ExecuteEndStep()
    {
    }
}

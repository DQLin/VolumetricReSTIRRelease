import Scene.Scene;
import Scene.Shading;
import Utils.Math.MathHelpers;
#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Helpers;

RaytracingAccelerationStructure gAccelerationStructure;


float3 computeScatterRayOrigin(float3 posW, float3 faceN)
{
    return computeRayOrigin(posW, faceN);
}

struct HitInfo
{
    uint    meshInstanceID;     ///< Mesh instance ID at hit.
    uint    primitiveIndex;     ///< Primitive index at hit.
    float2  barycentrics;       ///< Barycentric coordinates at ray hit, always in [0,1].
    float   rayT;

    static const uint kInvalidIndex = 0xffffffff;

    static HitInfo createHitInfo(uint meshInstanceID, uint primitiveIndex, float2 barycentrics, float rayT) { return { meshInstanceID, primitiveIndex, barycentrics, rayT  }; }
    static HitInfo createInvalidHitInfo() { return {-1,-1,float2(0), FLT_MAX }; }

    [mutating] void setInvalid() { rayT = FLT_MAX;  }

    bool isValidHit() { return rayT != FLT_MAX; }

    /** Return the barycentric weights.
    */
    float3 getBarycentricWeights()
    {
        return float3(1.f - barycentrics.x - barycentrics.y, barycentrics.x, barycentrics.y);
    }

};

// return the meshId and primitive ID
HitInfo FindSurfaceHit(float3 rayOrigin, float3 rayDir, float RayTMin, float RayTMax)
{
    // shoot a ray
    RayDesc ray;
    ray.Origin = rayOrigin;
    ray.Direction = rayDir;
    ray.TMin = max(0,RayTMin);
    ray.TMax = RayTMax;

    RayQuery<RAY_FLAG_NONE> q;
    // Set up a trace
    q.TraceRayInline(
        gAccelerationStructure,
        RAY_FLAG_NONE,
        -1,
        ray);

    bool hasHit = false;

    while (q.Proceed())
    {
        switch (q.CandidateType())
        {
        case CANDIDATE_PROCEDURAL_PRIMITIVE:
        {
            // procedural primitive not supported
            break;
        }
        case CANDIDATE_NON_OPAQUE_TRIANGLE:
        {
            uint globalHitID = q.CandidateInstanceID() + q.CandidateGeometryIndex();
            // Alpha test for non-opaque geometry.
            float2 barycentrics = q.CandidateTriangleBarycentrics();
            VertexData v = gScene.getVertexData(globalHitID, q.CandidatePrimitiveIndex(), float3(1-barycentrics.x-barycentrics.y, barycentrics.x, barycentrics.y));
            const uint materialID = gScene.getMaterialID(globalHitID);
            if (!alphaTest(v, gScene.materials[materialID], gScene.materialResources[materialID], 0.f))
            {
                q.CommitNonOpaqueTriangleHit();
            }
        }
        }
    }

    // Examine and act on the result of the traversal.
    // Was a hit committed?
    if (q.CommittedStatus() == COMMITTED_TRIANGLE_HIT)
    {
        return HitInfo.createHitInfo(q.CommittedInstanceID() + q.CommittedGeometryIndex(), q.CommittedPrimitiveIndex(), q.CommittedTriangleBarycentrics(), q.CommittedRayT());
    }
    else // COMMITTED_NOTHING
         // From template specialization,
         // COMMITTED_PROCEDURAL_PRIMITIVE can't happen.
    {
        return HitInfo.createInvalidHitInfo();
    }
}


// return the meshId and primitive ID
bool FindIfOccluded(float3 rayOrigin, float3 rayDir, float RayTMin, float RayTMax, bool cullNonOpaqueGeometry = false)
{
    // shoot a ray
    RayDesc ray;
    ray.Origin = rayOrigin;
    ray.Direction = rayDir;
    ray.TMin = max(0, RayTMin);
    ray.TMax = RayTMax;

    RayQuery<RAY_FLAG_SKIP_CLOSEST_HIT_SHADER | RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH> q;
    // Set up a trace

    uint rayFlags = RAY_FLAG_SKIP_CLOSEST_HIT_SHADER | RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH;
    if (cullNonOpaqueGeometry) rayFlags |= RAY_FLAG_CULL_NON_OPAQUE;

    q.TraceRayInline(
        gAccelerationStructure,
        rayFlags,
        -1,
        ray);

    while (q.Proceed())
    {
        switch (q.CandidateType())
        {
        case CANDIDATE_PROCEDURAL_PRIMITIVE:
        {
            // procedural primitive not supported
            break;
        }
        case CANDIDATE_NON_OPAQUE_TRIANGLE:
        {
            uint globalHitID = q.CandidateInstanceID() + q.CandidateGeometryIndex();
            // Alpha test for non-opaque geometry.
            float2 barycentrics = q.CandidateTriangleBarycentrics();
            VertexData v = gScene.getVertexData(globalHitID, q.CandidatePrimitiveIndex(), float3(1 - barycentrics.x - barycentrics.y, barycentrics.x, barycentrics.y));
            const uint materialID = gScene.getMaterialID(globalHitID);
            if (!alphaTest(v, gScene.materials[materialID], gScene.materialResources[materialID], 0.f))
            {
                q.CommitNonOpaqueTriangleHit();
            }
        }
        }
    }


    if (q.CommittedStatus() == COMMITTED_TRIANGLE_HIT)
    {
        return true;
    }
    else // COMMITTED_NOTHING
    {
        return false;
    }
}

ShadingData computeSurfaceShadingInfo(int meshInstanceId, VertexData v, float3 viewDir)
{
    const uint materialID = gScene.getMaterialID(meshInstanceId);
    ShadingData sd = prepareShadingData(v, materialID, gScene.materials[materialID], gScene.materialResources[materialID], viewDir, 0.f);
    sd.posW = computeScatterRayOrigin(sd.posW, sd.frontFacing ? sd.faceN : -sd.faceN);
    return sd;
}

ShadingData computeSurfaceShadingInfo(HitInfo hitInfo, float3 viewDir)
{
    uint globalHitID = hitInfo.meshInstanceID;
    uint primitiveIndex = hitInfo.primitiveIndex;
    float2 barycentrics = hitInfo.barycentrics;
    float3 barycentrics3 = float3(1.0 - barycentrics.x - barycentrics.y, barycentrics.x, barycentrics.y);
    VertexData v = gScene.getVertexData(globalHitID, primitiveIndex, barycentrics3);
    return computeSurfaceShadingInfo(globalHitID, v, viewDir);
}

bool isEmissive(ShadingData sd)
{
    return any(sd.emissive > 0);
}

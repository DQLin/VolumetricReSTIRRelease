import Utils.Sampling.SampleGenerator;

#define DEVICE_NEED_INCLUDE
#include "HostDeviceSharedDefinitions.h"
#include "Utils/Math/MathConstants.slangh"
import Scene.Scene;

#ifdef VERTEX_REUSE

#if MAX_BOUNCES == 1
Reservoir createNewReservoir() { return { 0.f, 0.f, FLT_MAX, 0.f, float2(0,0), 0, 0.f, 0 }; }
#else
Reservoir createNewReservoir() { return { 0.f, 0.f, FLT_MAX, 0.f, float2(0,0), 0, 0.f, 0, 0 }; }
#endif

#else

#if MAX_BOUNCES == 1
Reservoir createNewReservoir() { return { 0.f, 0.f, FLT_MAX, 0.f, float2(0,0), 0, 0 }; }
#else
Reservoir createNewReservoir() { return { 0.f, 0.f, FLT_MAX, 0.f, float2(0,0), 0, 0, 0 }; }
#endif

#endif

bool simpleResampleStep(Reservoir reservoir, inout Reservoir state, inout SampleGenerator sg)
{
    // What's the current weight (times any prior-step RIS normalization factor)
    float sampleWeight = reservoir.runningSum;
    state.M += reservoir.M;

    if (sampleWeight <= 0.0) return false;

    // Our *effective* candidate pool is the sum of our candidates plus those of our neighbors

    // Update the weight
    state.runningSum += sampleWeight;

    // Decide if we will randomly pick this sample
    bool selectSample = sampleNext1D(sg) * state.runningSum < sampleWeight;

    state.depth = selectSample ? reservoir.depth : state.depth;
    state.p_y = selectSample ? reservoir.p_y : state.p_y;
    state.lightUV = selectSample ? reservoir.lightUV : state.lightUV;
    state.lightID = selectSample ? reservoir.lightID : state.lightID;
#ifdef VERTEX_REUSE
    state.p_partial = selectSample ? reservoir.p_partial : state.p_partial;
#endif

#if MAX_BOUNCES > 1
    state.extraBounceStartId = selectSample ? reservoir.extraBounceStartId : state.extraBounceStartId;
#endif

    state.sampledPixel = selectSample ? reservoir.sampledPixel : state.sampledPixel;

    return selectSample;
}

bool simpleResampleStepWithMaxM(Reservoir reservoir, float MThreshold, inout Reservoir state, inout SampleGenerator sg)
{
    float correctedM = min(MThreshold, reservoir.M);
    float sampleWeight = correctedM == 0.0 ? 0.0 : correctedM/reservoir.M * reservoir.runningSum;
    state.M += correctedM;
    if (sampleWeight <= 0.0) return false;

    // Update the weight
    state.runningSum += sampleWeight;

    bool selectSample = sampleNext1D(sg) * state.runningSum < sampleWeight;

    // If we did select this sample, update the relevant data
    state.depth = selectSample ? reservoir.depth : state.depth;
    state.p_y = selectSample ? reservoir.p_y : state.p_y;
    state.lightUV = selectSample ? reservoir.lightUV : state.lightUV;
    state.lightID = selectSample ? reservoir.lightID : state.lightID;
#ifdef VERTEX_REUSE
    state.p_partial = selectSample ? reservoir.p_partial : state.p_partial;
#endif

#if MAX_BOUNCES > 1
    state.extraBounceStartId = selectSample ? reservoir.extraBounceStartId : state.extraBounceStartId;
#endif

    state.sampledPixel = selectSample ? reservoir.sampledPixel : state.sampledPixel;

    return selectSample;
}

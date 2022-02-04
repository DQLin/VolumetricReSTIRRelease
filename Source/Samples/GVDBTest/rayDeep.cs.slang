// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0

//	\file renderFogVolume.c
//	\author Wil Braithwaite
//	\date July 24, 2020
//	\brief General (C99-ish) implementation of the FogVolume rendering code.
//
////////////////////////////////////////////////////////

//#define CNANOVDB_VIEWER_USE_RATIO_TRACKED_TRANSMISSION 1

/////////////////////////

import Scene.GVDB.gvdbRaycast;
ParameterBlock<VDBInfo> gvdb;
RWTexture2D<float4> kOutImage;


float3 getViewPos_()
{
    return mul(float4(scn.campos, 1), gvdb.invxform).xyz;
}

float3 getViewRay_(float x, float y)
{
    float3 v = x * scn.camu + y * scn.camv + scn.cams;
    return normalize(mul(float4(v, 1), gvdb.invxrot).xyz);
}

[numthreads(8, 8, 1)]
void main(uint3 DTid : SV_DispatchThreadID)
{
    if (DTid.x >= scn.width || DTid.y >= scn.height) return;

    float4 clr = float4(0, 0, 0, 1);
    float3 hit = float3(0, 0, NOHIT);
    float3 norm;
    float3 rpos = getViewPos_();
    float3 rdir = getViewRay_(float(DTid.x + 0.5f) / float(scn.width), float(DTid.y + 0.5f) / float(scn.height));

    // ray deep sampling	
    rayCast(gvdb, 0, rpos, rdir, hit, norm, clr, DTid.x, DTid.y);
    clr = float4(lerp(scn.backclr, clr, 1.0 - clr.w).xyz, 1.0 - clr.w);

    kOutImage[DTid.xy] = pow(clr,2.2f);
}


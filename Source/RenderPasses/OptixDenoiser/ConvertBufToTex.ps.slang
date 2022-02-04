/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
cbuffer GlobalCB
{
    uint gStride;
}

Buffer<float4>      gInBuf;

float4 main(
    float2 texC : TEXCOORD,
    float4 posH : SV_POSITION) : SV_Target0
{
    uint2 pixel = uint2(posH.xy);
    uint bufIdx = pixel.x + pixel.y * gStride;
    return gInBuf[bufIdx];
}

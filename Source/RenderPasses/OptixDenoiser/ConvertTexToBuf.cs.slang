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

Texture2D<float4>   gInTex;
RWBuffer<float4>    gOutBuf;

[numthreads(8, 8, 1)]
void main(uint3 thrdId : SV_DispatchThreadID)
{
    uint bufIdx = thrdId.x + thrdId.y * gStride;
    gOutBuf[bufIdx] = gInTex[thrdId.xy];
}

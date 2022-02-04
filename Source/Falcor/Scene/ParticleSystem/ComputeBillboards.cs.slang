/***************************************************************************
 # Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
import ParticleData;
#include "Utils/HostDeviceShared.slangh"

#define M_SQRT6 2.4494897427831780981972840747059

static const int xOffset[4] = { -1, -1, 1, 1 };
static const int yOffset[4] = { 1, -1, 1, -1 };
static const float xTex[4] = { 0.f, 0.f, 1.f, 1.f };
static const float yTex[4] = { 0.f, 1.f, 0.f, 1.f };

cbuffer PerFrame
{
    ComputeBillboardPerFrame frameData;
};

struct DrawArguments
{
    uint vertexCountPerInstance;
    uint instanceCount;
    uint startVertexLocation;
    uint startInstanceLocation;
};

struct ParticleVertex
{
    float3 position;
#ifdef PROCEDURAL_PARTICLE
    float scale;
    float rotation;
#else
    float2 texCoords;
#endif
    int particlePoolIndex;
}

#ifdef _SORT
StructuredBuffer<SortData> aliveList : register(t0);
#else
StructuredBuffer<uint> aliveList : register(t0);
#endif
RWStructuredBuffer<Particle> particlePool : register(u2);
RWStructuredBuffer<ParticleVertex> vbo : register(u0);
#ifdef PROCEDURAL_PARTICLE
RWStructuredBuffer<float3> aabbs : register(u3);
#endif
//RWStructuredBuffer<int> particleIndices : register(u1);
RWStructuredBuffer<DrawArguments> drawArgs : register(u1);
ByteAddressBuffer numAlive : register(t2);

#ifdef PROCEDURAL_PARTICLE
[numthreads(1024, 1, 1)]
#else
[numthreads(256, 4, 1)]
#endif
void main(uint3 DTid : SV_DispatchThreadID)
{
    uint particleIndex = DTid.x;
#ifdef _SORT
    uint poolIndex = aliveList[particleIndex].index;
#else
    uint poolIndex = aliveList[particleIndex];
#endif
    Particle p = particlePool[poolIndex];

#ifdef PROCEDURAL_PARTICLE
    float3 posMin = p.pos - M_SQRT6 * p.scale;
    float3 posMax = p.pos + M_SQRT6 * p.scale;
    int numAliveParticles = (uint) (numAlive.Load(0));
    if (p.scale == 0.f || particleIndex >= numAliveParticles)
    {
        posMin = float3(0, 0, 0); // collapse
        posMax = posMin;
    }
    
    aabbs[2 * (frameData.globalParticleOffset + particleIndex)] = posMin;
    aabbs[2 * (frameData.globalParticleOffset + particleIndex) + 1] = posMax;

    ParticleVertex vertex;
    vertex.position = p.pos;
    vertex.particlePoolIndex = poolIndex;
    vertex.scale = p.scale;
    vertex.rotation = p.rot;
    particlePool[poolIndex].billboardID = particleIndex;
    vbo[(frameData.globalParticleOffset + particleIndex)] = vertex;
#else
    uint billboardIndex = DTid.y;
    float4 viewPos = mul(float4(p.pos, 1.f), frameData.view);
    float2 rotOffset = vectorRotate(float2(xOffset[billboardIndex], yOffset[billboardIndex]), sin(p.rot), cos(p.rot));
    viewPos.xy += float2(p.scale, p.scale) * rotOffset;
    float4 worldPos = mul(viewPos, frameData.invView);
    ParticleVertex vertex;
    vertex.position = worldPos.xyz;
    vertex.texCoords = float2(xTex[billboardIndex], yTex[billboardIndex]);
    vertex.particlePoolIndex = poolIndex;
    int numAliveParticles = (uint) (numAlive.Load(0));
    if (p.scale == 0.f || particleIndex >= numAliveParticles)
    {
        vertex.position = float3(0, 0, 0); // collapse
    }
    else
    {
        if (DTid.y == 0)
        {
            particlePool[poolIndex].billboardID = particleIndex;
        }
    }
    
    vbo[4 * (frameData.globalParticleOffset + particleIndex) + billboardIndex] = vertex;
#endif
    //particleIndices[particleIndex] = poolIndex;
}


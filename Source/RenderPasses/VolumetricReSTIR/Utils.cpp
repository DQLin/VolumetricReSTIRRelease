/***************************************************************************
 # Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/

#include "Utils.h"

using namespace Falcor;

// Define shared constants
const Falcor::Resource::BindFlags   kDefaultFlags = Resource::BindFlags::ShaderResource | Resource::BindFlags::UnorderedAccess | Resource::BindFlags::RenderTarget;
const Falcor::Resource::BindFlags   kDepthFlags = Resource::BindFlags::ShaderResource | Resource::BindFlags::DepthStencil;
const Falcor::Gui::WindowFlags      kPassGuiFlags = Gui::WindowFlags::ShowTitleBar | Gui::WindowFlags::AllowMove;

////////////////////////////////////////////////////////////////////
// Convert from std::vector<> to glm vectors.
//    -> Useful for converting from pybind11 list values to glm vectors

float2 _toVec2(std::vector<float> pyVec, float2 def)
{
    if (pyVec.size() >= 2)
    {
        return float2(pyVec[0], pyVec[1]);
    }
    return def;
}

float3 _toVec3(std::vector<float> pyVec, float3 def)
{
    if (pyVec.size() >= 3)
    {
        return float3(pyVec[0], pyVec[1], pyVec[2]);
    }
    return def;
}

float4 _toVec4(std::vector<float> pyVec, float4 def)
{
    if (pyVec.size() >= 4)
    {
        return float4(pyVec[0], pyVec[1], pyVec[2], pyVec[3]);
    }
    return def;
}

Sampler::SharedPtr createLinearSampler()
{
    Sampler::Desc desc;
    desc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Point).setAddressingMode(Sampler::AddressMode::Clamp, Sampler::AddressMode::Clamp, Sampler::AddressMode::Clamp);
    return Sampler::create(desc);
}

Sampler::SharedPtr createNearestSampler()
{
    Sampler::Desc desc;
    desc.setFilterMode(Sampler::Filter::Point, Sampler::Filter::Point, Sampler::Filter::Point).setAddressingMode(Sampler::AddressMode::Clamp, Sampler::AddressMode::Clamp, Sampler::AddressMode::Clamp);
    return Sampler::create(desc);
}

Texture::SharedPtr createNeighborOffsetTexture( int numSamples )
{
    int R = 250;
    std::unique_ptr<int8_t[]> offsets(new int8_t[numSamples * 2]);
    const float phi2 = 1.0f / 1.3247179572447f;
    int num = 0;
    float u = 0.5f;
    float v = 0.5f;
    while (num < numSamples * 2) {
        u += phi2;
        v += phi2 * phi2;
        if (u >= 1.0f) u -= 1.0f;
        if (v >= 1.0f) v -= 1.0f;

        float rSq = (u - 0.5f)*(u - 0.5f) + (v - 0.5f)*(v - 0.5f);
        if (rSq > 0.25f)
            continue;

        offsets[num++] = int8_t((u - 0.5f)*R);
        offsets[num++] = int8_t((v - 0.5f)*R);
    }

    return Texture::create1D(numSamples, ResourceFormat::RG8Int, 1, 1, offsets.get());
}
ComputePass::SharedPtr createSimpleComputePass(const std::string &file, const std::string &mainEntry,
    Shader::DefineList defs)
{
    // To avoid not being able to compile compute shaders #import'ing Scene.slang, make sure to define a MATERIAL_COUNT parameter.
    //   NOTE:  This just avoids the compile error on shader load, this parameter *still* needs to be set to the correct value when
    //   the scene is loaded (by calling updateSceneDefines(), below)...
    Shader::DefineList matlDefs = { { "MATERIAL_COUNT", "1" }, {"PARTICLE_SYSTEM_COUNT", "1"}, {"INDEXED_VERTICES", "1"} };
    matlDefs.add(defs);
    matlDefs.add("_MS_DISABLE_ALPHA_TEST");    
   // matlDefs.add("_DEFAULT_ALPHA_TEST");

    Program::Desc risDesc(file);
    risDesc.setShaderModel("6_5");

    // Enable this for debugging in NSight
    //risDesc.setCompilerFlags(Shader::CompilerFlags::GenerateDebugInfo);
    
    return ComputePass::create(risDesc.csEntry(mainEntry), matlDefs);
}

void updateSceneDefines(ComputePass::SharedPtr& pPass, const Scene::SharedPtr& pScene)
{
    if (!pScene) return;

    pPass->getProgram()->addDefines(pScene->getSceneDefines());
    pPass->getProgram()->addDefine("MAX_BOUNCES", "1");
    pPass->setVars(nullptr);
    pPass->getRootVar()["gScene"] = pScene->getParameterBlock();
}

/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
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
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
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
#include "stdafx.h"
#include "EmissivePowerSampler.h"

namespace Falcor
{
    EmissivePowerSampler::SharedPtr EmissivePowerSampler::create(RenderContext* pRenderContext, Scene::SharedPtr pScene, const Options& options)
    {
        return SharedPtr(new EmissivePowerSampler(pRenderContext, pScene, options));
    }

    bool EmissivePowerSampler::update(RenderContext* pRenderContext)
    {
        PROFILE("EmissivePowerSampler::update");

        bool samplerChanged = false;

        // Rebuild BVH if it's marked as dirty.
        if (mNeedsRebuild)
        {
            mNeedsRebuild = false;
            // TODO: allow dynamic light intensity
            samplerChanged = true;
        }

        return samplerChanged;
    }

    bool EmissivePowerSampler::setShaderData(const ShaderVar& var) const
    {
        assert(mpTable);
        mpTable->setShaderData(var["_aliasTable"]);
        return true;
    }

    EmissivePowerSampler::EmissivePowerSampler(RenderContext* pRenderContext, Scene::SharedPtr pScene, const Options& options)
        : EmissiveLightSampler(EmissiveLightSamplerType::Power, pScene)
        , mOptions(options)
    {
        // Make sure the light collection is created.
        const std::vector<LightCollection::MeshLightTriangle>& meshLightTriangles = pScene->getLightCollection(pRenderContext)->getMeshLightTriangles();

        std::vector<float> weights(meshLightTriangles.size());

        for (int triId = 0; triId < meshLightTriangles.size(); triId++)
        {
            weights[triId] = meshLightTriangles[triId].flux;
        }

        std::default_random_engine rng(123);

        mpTable = AliasTable::create(weights, rng);
    }

    SCRIPT_BINDING(EmissivePowerSampler)
    {
        // TODO use a nested class in the bindings when supported.
        ScriptBindings::SerializableStruct<EmissivePowerSampler::Options> options(m, "EmissivePowerSamplerOptions");
#define field(f_) field(#f_, &EmissivePowerSampler::Options::f_)
        // TODO
        //options.field(usePreintegration);
#undef field
    }
}

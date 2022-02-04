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
#include "Scene.h"
#include "HitInfo.h"
#include "Raytracing/RtProgram/RtProgram.h"
#include "Raytracing/RtProgramVars.h"
#include <sstream>
#include <nanovdb/util/IO.h>
#include <nanovdb/util/GridBuilder.h>
#include <nanovdb/util/NanoToOpenVDB.h>
#include <nanovdb/util/OpenToNanoVDB.h>
#include <openvdb/tools/GridTransformer.h>
#include <gvdb_volume_gvdb.h>
#include <glm/detail/type_half.hpp>
#include "Utils/Image/BCHelper.h"
#include "../RenderPasses/VolumetricReSTIR/HostDeviceSharedConstants.slang"
 //#define ONE_CURVE_PER_BLAS

// 0 -- no, 1 -- uint8, 2 -- BC4
#define ATLAS_COMPRESSION 2

namespace Falcor
{
    static_assert(sizeof(PackedStaticVertexData) % 16 == 0, "PackedStaticVertexData size should be a multiple of 16");
    static_assert(sizeof(PackedMeshInstanceData) % 16 == 0, "PackedMeshInstanceData size should be a multiple of 16");
    static_assert(PackedMeshInstanceData::kMatrixBits + PackedMeshInstanceData::kMeshBits + PackedMeshInstanceData::kFlagsBits <= 32);

    namespace
    {
        // Checks if the transform flips the coordinate system handedness (its determinant is negative).
        bool doesTransformFlip(const glm::mat4& m)
        {
            return glm::determinant((glm::mat3)m) < 0.f;
        }

        const std::string kParameterBlockName = "gScene";
        const std::string kMeshBufferName = "meshes";
        const std::string kCurveBufferName = "curves";
        const std::string kParticleSystemBufferName = "particleSystems";
        const std::string kParticlePoolBufferName = "particlePools";
        const std::string kMeshInstanceBufferName = "meshInstances";
        const std::string kTriangleInstanceMappingBufferName = "triangleInstanceMapping";
        const std::string kIndexBufferName = "indices";
        const std::string kVertexBufferName = "vertices";
        const std::string kPrevVertexBufferName = "prevVertices";
        const std::string kCurveVertexBufferName = "curveVertices";
        const std::string kMaterialsBufferName = "materials";
        const std::string kLightsBufferName = "lights";
        const std::string kParticleVertexBufferName = "particleVertices";
        const std::string kParticleIndexBufferName = "particleIndices";
        const std::string kVolumeDataTextureName = "volumeData";

        const std::string kSurfaceGlobalAlphaMultipler = "surfaceGlobalAlphaMultipler";
        const std::string kParticleCurveGlobalAlphaMultipler = "particleCurveGlobalAlphaMultipler";

        const std::string kCamera = "camera";
        const std::string kCameras = "cameras";
        const std::string kCameraSpeed = "cameraSpeed";
        const std::string kAnimated = "animated";
        const std::string kRenderSettings = "renderSettings";
        const std::string kEnvMap = "envMap";
        const std::string kMaterials = "materials";
        const std::string kGetLight = "getLight";
        const std::string kGetMaterial = "getMaterial";
        const std::string kSetEnvMap = "setEnvMap";
        const std::string kSetEnvMapIntensity = "setEnvMapIntensity";
        const std::string kSetEnvMapRotation = "setEnvMapRotation";
        const std::string kSetEmissiveIntensityMultiplier = "setEmissiveIntensityMultiplier";
        const std::string kAddViewpoint = "addViewpoint";
        const std::string kRemoveViewpoint = "kRemoveViewpoint";
        const std::string kSelectViewpoint = "selectViewpoint";
        const std::string kGlobalSurfaceAlphaMultipler = "surfaceAlphaMultipler";
        const std::string kGlobalParticleCurveAlphaMultipler = "particleCurveAlphaMultipler";

    }

    const FileDialogFilterVec& Scene::getFileExtensionFilters()
    {
        return Importer::getFileExtensionFilters();
    }

    Scene::Scene()
    {
        mpFrontClockwiseRS = RasterizerState::create(RasterizerState::Desc().setFrontCounterCW(false));
        mParticleSystemManager.Init();
    }

    Scene::SharedPtr Scene::create(const std::string& filename)
    {
        auto pBuilder = SceneBuilder::create(filename);
        return pBuilder ? pBuilder->getScene() : nullptr;
    }

    Scene::SharedPtr Scene::create()
    {
        return Scene::SharedPtr(new Scene());
    }

    Shader::DefineList Scene::getSceneDefines() const
    {
        Shader::DefineList defines;
        defines.add("MATERIAL_COUNT", std::to_string(mMaterials.size()));
        defines.add("PARTICLE_SYSTEM_COUNT", std::to_string(std::max((size_t)1, mParticleSystemDesc.size())));
        defines.add("INDEXED_VERTICES", hasIndexBuffer() ? "1" : "0");
        defines.add(HitInfo::getDefines(this));
        return defines;
    }

    const LightCollection::SharedPtr& Scene::getLightCollection(RenderContext* pContext)
    {
        if (!mpLightCollection)
        {
            mpLightCollection = LightCollection::create(pContext, shared_from_this());
            mpLightCollection->setShaderData(mpSceneBlock["lightCollection"]);
        }
        return mpLightCollection;
    }

    void Scene::render(RenderContext* pContext, GraphicsState* pState, GraphicsVars* pVars, RenderFlags flags)
    {
        PROFILE("renderScene");

        pState->setVao(mpVao);
        pVars->setParameterBlock("gScene", mpSceneBlock);

        bool overrideRS = !is_set(flags, RenderFlags::UserRasterizerState);
        auto pCurrentRS = pState->getRasterizerState();
        bool isIndexed = hasIndexBuffer();

        if (mDrawCounterClockwiseMeshes.count)
        {
            if (overrideRS) pState->setRasterizerState(nullptr);
            if (isIndexed) pContext->drawIndexedIndirect(pState, pVars, mDrawCounterClockwiseMeshes.count, mDrawCounterClockwiseMeshes.pBuffer.get(), 0, nullptr, 0);
            else pContext->drawIndirect(pState, pVars, mDrawCounterClockwiseMeshes.count, mDrawCounterClockwiseMeshes.pBuffer.get(), 0, nullptr, 0);
        }

        if (mDrawClockwiseMeshes.count)
        {
            if (overrideRS) pState->setRasterizerState(mpFrontClockwiseRS);
            if (isIndexed) pContext->drawIndexedIndirect(pState, pVars, mDrawClockwiseMeshes.count, mDrawClockwiseMeshes.pBuffer.get(), 0, nullptr, 0);
            else pContext->drawIndirect(pState, pVars, mDrawClockwiseMeshes.count, mDrawClockwiseMeshes.pBuffer.get(), 0, nullptr, 0);
        }

        if (overrideRS) pState->setRasterizerState(pCurrentRS);
    }

    void Scene::raytrace(RenderContext* pContext, RtProgram* pProgram, const std::shared_ptr<RtProgramVars>& pVars, uint3 dispatchDims)
    {
        PROFILE("raytraceScene");

        // if a type have non-zero hit program count, they must be the same
        uint32_t typeHitProgramCounts[3] = { pProgram->getHitProgramCount(), pProgram->getParticleHitProgramCount(), pProgram->getCurveHitProgramCount() };
        uint32_t numHitPrograms = 0;
        for (int i = 0; i < 3; i++)
        {
            if (typeHitProgramCounts[i] != 0)
            {
                if (numHitPrograms == 0) numHitPrograms = typeHitProgramCounts[i];
                else assert(numHitPrograms == typeHitProgramCounts[i]);
            }
        }

        auto rayTypeCount = std::max(pProgram->getCurveHitProgramCount(), std::max(pProgram->getParticleHitProgramCount(), pProgram->getHitProgramCount()));
        setRaytracingShaderData(pContext, pVars->getRootVar(), rayTypeCount);

        // If not set yet, set geometry indices for this RtProgramVars.
        if (pVars->getSceneForGeometryIndices().get() != this)
        {
            setGeometryIndexIntoRtVars(pVars);
            pVars->setSceneForGeometryIndices(shared_from_this());
        }

        // Set ray type constant.
        pVars->getRootVar()["DxrPerFrame"]["hitProgramCount"] = rayTypeCount;

        pContext->raytrace(pProgram, pVars.get(), dispatchDims.x, dispatchDims.y, dispatchDims.z);
    }

    void Scene::initResources()
    {
        GraphicsProgram::SharedPtr pProgram = GraphicsProgram::createFromFile("Scene/SceneBlock.slang", "", "main", getSceneDefines());
        ParameterBlockReflection::SharedConstPtr pReflection = pProgram->getReflector()->getParameterBlock(kParameterBlockName);
        assert(pReflection);

        if (!mpSceneBlock) mpSceneBlock = ParameterBlock::create(pReflection);
        else mpSceneBlock->Reinit(pReflection->getProgramVersion(), pReflection);
        mpMeshesBuffer = Buffer::createStructured(mpSceneBlock[kMeshBufferName], (uint32_t)mMeshDesc.size(), Resource::BindFlags::ShaderResource, Buffer::CpuAccess::None, nullptr, false);
        mpMeshesBuffer->setName("Scene::mpMeshesBuffer");
        mpMeshInstancesBuffer = Buffer::createStructured(mpSceneBlock[kMeshInstanceBufferName], (uint32_t)mMeshInstanceData.size(), Resource::BindFlags::ShaderResource, Buffer::CpuAccess::None, nullptr, false);
        mpMeshInstancesBuffer->setName("Scene::mpMeshInstancesBuffer");

        mpParticleSystemsBuffer = Buffer::createStructured(mpSceneBlock[kParticleSystemBufferName], std::max(1u, (uint32_t)mParticleSystemDesc.size()), Resource::BindFlags::ShaderResource);
        mpParticleSystemsBuffer->setName("Scene::mpParticleSystemBuffer");

        mpCurvesBuffer = Buffer::createStructured(mpSceneBlock[kCurveBufferName], std::max(1u, (uint32_t)mCurveDesc.size()), Resource::BindFlags::ShaderResource);
        mpCurvesBuffer->setName("Scene::mpCurvesBuffer");

        mpMaterialsBuffer = Buffer::createStructured(mpSceneBlock[kMaterialsBufferName], (uint32_t)mMaterials.size(), Resource::BindFlags::ShaderResource, Buffer::CpuAccess::None, nullptr, false);
        mpMaterialsBuffer->setName("Scene::mpMaterialsBuffer");

        mTriangleInstanceMapping.clear();
        for (int meshInstanceId = 0; meshInstanceId < mMeshInstanceData.size(); meshInstanceId++)
        {
            uint32_t meshID = mMeshInstanceData[meshInstanceId].meshID;
            auto& meshDesc = mMeshDesc[meshID];
            uint32_t numTriangles = meshDesc.getTriangleCount();

            for (uint32_t triangleId = 0; triangleId < numTriangles; triangleId++)
            {
                mTriangleInstanceMapping.push_back(uint2(meshInstanceId, triangleId));
            }
        }
        mNumMeshInstances = (uint32_t)mMeshInstanceData.size();
        mNumTriangleInstances = (uint32_t)mTriangleInstanceMapping.size();
        mpTriangleInstanceMappingBuffer = Buffer::createStructured(mpSceneBlock[kTriangleInstanceMappingBufferName],
            (uint32_t)mTriangleInstanceMapping.size(), Resource::BindFlags::ShaderResource, Buffer::CpuAccess::None, nullptr, false);

        if (mLights.size())
        {
            mpLightsBuffer = Buffer::createStructured(mpSceneBlock[kLightsBufferName], (uint32_t)mLights.size(), Resource::BindFlags::ShaderResource, Buffer::CpuAccess::None, nullptr, false);
            mpLightsBuffer->setName("Scene::mpLightsBuffer");
        }

        mpCurveVertexBuffer = Buffer::createStructured(mpSceneBlock[kCurveVertexBufferName], std::max(1u, (uint32_t)mCPUCurveVertexBuffer.size()), Resource::BindFlags::ShaderResource);
    }

    void Scene::uploadResources()
    {
        // Upload geometry
        mpMeshesBuffer->setBlob(mMeshDesc.data(), 0, sizeof(MeshDesc) * mMeshDesc.size());
        mpTriangleInstanceMappingBuffer->setBlob(mTriangleInstanceMapping.data(), 0, sizeof(uint2) * mTriangleInstanceMapping.size());
        mpParticleSystemsBuffer->setBlob(mParticleSystemDesc.data(), 0, sizeof(ParticleSystemDesc) * mParticleSystemDesc.size());
        mpCurveVertexBuffer->setBlob(mCPUCurveVertexBuffer.data(), 0, sizeof(CurveVertexData) * mCPUCurveVertexBuffer.size());
        mpCurvesBuffer->setBlob(mCurveDesc.data(), 0, sizeof(CurveDesc) * mCurveDesc.size());

        mpSceneBlock["volumeDesc"].setBlob(mVolumeDesc);
        mpSceneBlock["numMeshInstances"] = mNumMeshInstances;
        mpSceneBlock["numTriangleInstances"] = mNumTriangleInstances;
        mpSceneBlock["curveInstanceOffset"] = mNumMeshInstances + mParticleSystemDesc.size();
        mpSceneBlock->setBuffer(kMeshInstanceBufferName, mpMeshInstancesBuffer);
        mpSceneBlock->setBuffer(kMeshBufferName, mpMeshesBuffer);
        mpSceneBlock->setBuffer(kCurveBufferName, mpCurvesBuffer);
        mpSceneBlock->setBuffer(kTriangleInstanceMappingBufferName, mpTriangleInstanceMappingBuffer);
        mpSceneBlock->setBuffer(kParticleSystemBufferName, mpParticleSystemsBuffer);
        for (int i = 0; i < mParticleSystems.size(); i++)
        {
            mpSceneBlock[kParticlePoolBufferName][i] = mParticleSystems[i]->getParticlePool();
        }
        if (mParticleSystems.empty())
        {
            int i = 0;
            mpSceneBlock[kParticlePoolBufferName][i] = Buffer::create(sizeof(Particle));
        }

        mpSceneBlock[kSurfaceGlobalAlphaMultipler] = mSurfaceGlobalAlphaMultipler;
        mpSceneBlock[kParticleCurveGlobalAlphaMultipler] = mParticleCurveGlobalAlphaMultipler;

        mpSceneBlock->setBuffer(kLightsBufferName, mpLightsBuffer);
        mpSceneBlock->setBuffer(kMaterialsBufferName, mpMaterialsBuffer);
        if (hasIndexBuffer()) mpSceneBlock->setBuffer(kIndexBufferName, mpVao->getIndexBuffer());
        mpSceneBlock->setBuffer(kVertexBufferName, mpVao->getVertexBuffer(Scene::kStaticDataBufferIndex));
        mpSceneBlock->setBuffer(kPrevVertexBufferName, mpVao->getVertexBuffer(Scene::kPrevVertexBufferIndex));
        //if (mpUniformGridVolumeTexture) mpSceneBlock->setTexture(kVolumeDataTextureName, mpUniformGridVolumeTexture);

        mpSceneBlock->setBuffer(kCurveVertexBufferName, mpCurveVertexBuffer);

        if (mpParticleVao)
        {
            mpSceneBlock->setBuffer(kParticleVertexBufferName, mpParticleVao->getVertexBuffer(Scene::kStaticDataBufferIndex));
            if (mpParticleVao->getIndexBuffer()) mpSceneBlock->setBuffer(kParticleIndexBufferName, mpParticleVao->getIndexBuffer());
            else // bogus
            {
                std::vector<unsigned int> indices(1);
                Buffer::SharedPtr pIB = Buffer::create(indices.size() * sizeof(unsigned int), Resource::BindFlags::Index | ResourceBindFlags::ShaderResource, Buffer::CpuAccess::None, indices.data());
                mpSceneBlock->setBuffer(kParticleIndexBufferName, pIB);
            }
        }
        else
        {
            std::vector<unsigned int> indices(1);
            ResourceBindFlags ibBindFlags = Resource::BindFlags::Index | ResourceBindFlags::ShaderResource;
            Buffer::SharedPtr pIB = Buffer::create(indices.size() * sizeof(unsigned int), ibBindFlags, Buffer::CpuAccess::None, indices.data());
            // Create the vertex data as structured buffers
            ResourceBindFlags vbBindFlags = ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess | ResourceBindFlags::Vertex;
            Buffer::SharedPtr pStaticBuffer = Buffer::createStructured(sizeof(ParticleVertexData), 1, vbBindFlags, Buffer::CpuAccess::None, nullptr, false);
            mpSceneBlock->setBuffer(kParticleVertexBufferName, pStaticBuffer);
            mpSceneBlock->setBuffer(kParticleIndexBufferName, pIB);
        }

        if (mpLightProbe)
        {
            mpLightProbe->setShaderData(mpSceneBlock["lightProbe"]);
        }

        mpSceneBlock["volumeWorldTranslation"] = mVolumeWorldTranslation;
        mpSceneBlock["volumeWorldScaling"] = mVolumeWorldScaling;

        {
            glm::mat4 externalModelToWorldMatrix = computeVolumeExternalModelToWorldMatrix();
            glm::mat4 externalWorldToModelMatrix = glm::inverse(externalModelToWorldMatrix);
            mpSceneBlock["volumeExternalWorldToModelMatrix"] = externalWorldToModelMatrix;
            mpSceneBlock["volumeExternalModelToWorldMatrix"] = externalModelToWorldMatrix;
        }

        mpSceneBlock["emissiveIntensityMultiplier"] = mEmissiveIntensityMultiplier;
    }

    // TODO: On initial upload of materials, we could improve this by not having separate calls to setElement()
    // but instead prepare a buffer containing all data.
    void Scene::uploadMaterial(uint32_t materialID)
    {
        assert(materialID < mMaterials.size());

        const auto& material = mMaterials[materialID];

        mpMaterialsBuffer->setElement(materialID, material->getData());

        const auto& resources = material->getResources();

        auto var = mpSceneBlock["materialResources"][materialID];

#define set_texture(texName) var[#texName] = resources.texName;
        set_texture(baseColor);
        set_texture(specular);
        set_texture(emissive);
        set_texture(normalMap);
        set_texture(occlusionMap);
#undef set_texture

        var["samplerState"] = resources.samplerState;
    }

    void Scene::uploadSelectedCamera()
    {
        getCamera()->setShaderData(mpSceneBlock[kCamera]);
    }

    void Scene::updateBounds()
    {
        const auto& globalMatrices = mpAnimationController->getGlobalMatrices();
        std::vector<BoundingBox> instanceBBs;
        instanceBBs.reserve(mMeshInstanceData.size());

        for (const auto& inst : mMeshInstanceData)
        {
            const BoundingBox& meshBB = mMeshBBs[inst.meshID];
            const glm::mat4& transform = globalMatrices[inst.globalMatrixID];
            instanceBBs.push_back(meshBB.transform(transform));
        }

        mSceneBB = instanceBBs.front();
        for (const BoundingBox& bb : instanceBBs)
        {
            mSceneBB = BoundingBox::fromUnion(mSceneBB, bb);
        }

        // a temporary hardcoded bounding box for particle systems
        if (mParticleSystemDesc.size() > 0)
            mSceneBB = BoundingBox::fromUnion(mSceneBB, BoundingBox::fromMinMax(float3(-2, -2, -2), float3(2, 2, 2)));

        for (const BoundingBox& bb : mCurvePatchBBs)
        {
            mSceneBB = BoundingBox::fromUnion(mSceneBB, bb);
        }

        mSceneVolumeBB = BoundingBox::fromMinMax(float3(1e20f), float3(-1e20f));

        for (const BoundingBox& bb : mVDBVolumeBBs)
        {
            mSceneBB = BoundingBox::fromUnion(mSceneBB, bb);
            mSceneVolumeBB = BoundingBox::fromUnion(mSceneVolumeBB, bb);
        }
    }

    void Scene::updateMeshInstances(bool forceUpdate)
    {
        bool dataChanged = false;
        const auto& globalMatrices = mpAnimationController->getGlobalMatrices();

        for (auto& inst : mMeshInstanceData)
        {
            uint32_t prevFlags = inst.flags;
            inst.flags = (uint32_t)MeshInstanceFlags::None;

            const glm::mat4& transform = globalMatrices[inst.globalMatrixID];
            if (doesTransformFlip(transform)) inst.flags |= (uint32_t)MeshInstanceFlags::Flipped;

            dataChanged |= (inst.flags != prevFlags);
        }

        if (forceUpdate || dataChanged)
        {
            // Make sure the scene data fits in the packed format.
            // TODO: If we run into the limits, use bits from the materialID field.
            if (globalMatrices.size() >= (1 << PackedMeshInstanceData::kMatrixBits)) throw std::exception("Number of transform matrices exceed the maximum");
            if (getMeshCount() >= (1 << PackedMeshInstanceData::kMeshBits)) throw std::exception("Number of meshes exceed the maximum");

            // Prepare packed mesh instance data.
            assert(mMeshInstanceData.size() > 0);
            mPackedMeshInstanceData.resize(mMeshInstanceData.size());

            for (size_t i = 0; i < mMeshInstanceData.size(); i++)
            {
                mPackedMeshInstanceData[i].pack(mMeshInstanceData[i]);
            }

            size_t byteSize = sizeof(PackedMeshInstanceData) * mPackedMeshInstanceData.size();
            assert(mpMeshInstancesBuffer && mpMeshInstancesBuffer->getSize() == byteSize);
            mpMeshInstancesBuffer->setBlob(mPackedMeshInstanceData.data(), 0, byteSize);
        }
    }

    void Scene::createCurveVao()
    {
        size_t staticVbSize = sizeof(CurveVertexData) * mCPUCurveVertexBuffer.size();
        assert(staticVbSize <= UINT32_MAX);

        ResourceBindFlags vbBindFlags = ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess | ResourceBindFlags::Vertex;
        Buffer::SharedPtr pStaticBuffer = Buffer::createStructured(sizeof(CurveVertexData), (uint32_t)mCPUCurveVertexBuffer.size(), vbBindFlags, Buffer::CpuAccess::None, mCPUCurveVertexBuffer.data());

        Vao::BufferVec pVBs(1);
        pVBs[Scene::kStaticDataBufferIndex] = pStaticBuffer;

        // The layout only initialized the static and optional data. The skinning data doesn't get passed into the vertex-shader
        VertexLayout::SharedPtr pLayout = VertexLayout::create();

        // Static data
        VertexBufferLayout::SharedPtr pStaticLayout = VertexBufferLayout::create();
        pStaticLayout->addElement("POSITION", offsetof(CurveVertexData, position), ResourceFormat::RGBA32Float, 1, CURVE_VERTEX_POSITION_LOC);
        pStaticLayout->addElement("NORMAL", offsetof(CurveVertexData, normal), ResourceFormat::RGBA32Float, 1, CURVE_VERTEX_NORMAL_LOC);
        pStaticLayout->addElement("COLOR", offsetof(CurveVertexData, color), ResourceFormat::RGBA32Float, 1, CURVE_VERTEX_COLOR_LOC);
        pLayout->addBufferLayout(Scene::kStaticDataBufferIndex, pStaticLayout);

        // Add the draw ID layout
        VertexBufferLayout::SharedPtr pInstLayout = VertexBufferLayout::create();
        pInstLayout->addElement(INSTANCE_DRAW_ID_NAME, 0, ResourceFormat::R32Uint, 1, INSTANCE_DRAW_ID_LOC);
        pInstLayout->setInputClass(VertexBufferLayout::InputClass::PerInstanceData, 1);
        pLayout->addBufferLayout(Scene::kDrawIdBufferIndex, pInstLayout);

        mpCurveVao = Vao::create(Vao::Topology::LineList, pLayout, pVBs, nullptr, ResourceFormat::R32Uint);
    }

    void Scene::createParticleVao()
    {
        // Create the index buffer

#ifndef PROCEDURAL_PARTICLE
        Buffer::SharedPtr pIB = nullptr;

        std::vector<unsigned int> indices;
        for (uint32_t sysId = 0; sysId < mParticleSystems.size(); sysId++)
        {
            uint32_t maxParticles = mParticleSystems[sysId]->getMaxParticles();
            for (uint32_t i = 0; i < maxParticles; i++)
            {
                indices.push_back(4 * i);
                indices.push_back(4 * i + 1);
                indices.push_back(4 * i + 2);
                indices.push_back(4 * i + 2);
                indices.push_back(4 * i + 1);
                indices.push_back(4 * i + 3);
            }
        }

        {
            ResourceBindFlags ibBindFlags = Resource::BindFlags::Index | ResourceBindFlags::ShaderResource;
            pIB = Buffer::create(indices.size() * sizeof(unsigned int), ibBindFlags, Buffer::CpuAccess::None, indices.data());
        }
#endif

        // Create the vertex data as structured buffers
        ResourceBindFlags vbBindFlags = ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess | ResourceBindFlags::Vertex;
#ifdef PROCEDURAL_PARTICLE
        Buffer::SharedPtr pStaticBuffer = Buffer::createStructured(sizeof(ParticleVertexData), (uint32_t)(mTotalParticles), vbBindFlags, Buffer::CpuAccess::None, nullptr, false);
        mpParticleAABBBuffer = Buffer::createStructured(24, (uint32_t)mTotalParticles);
#else
        Buffer::SharedPtr pStaticBuffer = Buffer::createStructured(sizeof(ParticleVertexData), (uint32_t)(4 * mTotalParticles), vbBindFlags, Buffer::CpuAccess::None, nullptr, false);
#endif

        Vao::BufferVec pVBs(1);
        pVBs[Scene::kStaticDataBufferIndex] = pStaticBuffer;

        // The layout only initializes the vertex data and draw ID layout. The skinning data doesn't get passed into the vertex shader.
        VertexLayout::SharedPtr pLayout = VertexLayout::create();

        // Add the packed static vertex data layout
        VertexBufferLayout::SharedPtr pStaticLayout = VertexBufferLayout::create();
        pStaticLayout->addElement(VERTEX_POSITION_NAME, offsetof(ParticleVertexData, position), ResourceFormat::RGB32Float, 1, PARTICLE_VERTEX_POSITION_LOC);
#ifdef PROCEDURAL_PARTICLE
        pStaticLayout->addElement("SCALE", offsetof(ParticleVertexData, scale), ResourceFormat::R32Float, 1, PARTICLE_VERTEX_SCALE_LOC);
        pStaticLayout->addElement("ROTATION", offsetof(ParticleVertexData, rotation), ResourceFormat::R32Float, 1, PARTICLE_VERTEX_ROTATION_LOC);
        pStaticLayout->addElement("PARTICLEPOOLINDEX", offsetof(ParticleVertexData, particlePoolIndex), ResourceFormat::R32Int, 1, 3);
#else
        pStaticLayout->addElement(VERTEX_TEXCOORD_NAME, offsetof(ParticleVertexData, texCrd), ResourceFormat::RG32Float, 1, PARTICLE_VERTEX_TEXCOORD_LOC);
        pStaticLayout->addElement("PARTICLEPOOLINDEX", offsetof(ParticleVertexData, particlePoolIndex), ResourceFormat::R32Int, 1, 2);
#endif
        pLayout->addBufferLayout(Scene::kStaticDataBufferIndex, pStaticLayout);

        // Add the draw ID layouts
#ifdef PROCEDURAL_PARTICLE
        mpParticleVao = Vao::create(Vao::Topology::PointList, pLayout, pVBs, nullptr, ResourceFormat::R32Uint);
#else
        mpParticleVao = Vao::create(Vao::Topology::TriangleList, pLayout, pVBs, pIB, ResourceFormat::R32Uint);
#endif
    }

    void Scene::finalize(bool isReinit)
    {
        sortMeshes();
        initResources();
        mpAnimationController->bindBuffers();
        if (isReinit) mpAnimationController->mAnimationChanged = true;
        mpAnimationController->animate(gpDevice->getRenderContext(), 0); // Requires Scene block to exist
        updateMeshInstances(true);
        updateBounds();
        createDrawList();
        if (mCameras.size() == 0)
        {
            // Create a new camera to use in the event of a scene with no cameras
            mCameras.push_back(Camera::create());
            resetCamera();
        }
        if (isReinit)
        {
            //resetCamera();
            mBlasData.clear();
            mpBlas = nullptr;
            mpBlasScratch = nullptr;
            mRebuildBlas = true;
            mpTlasScratch = nullptr;
            mTlasCache.clear();
            mInstanceDescs.clear();
        }

        setCameraController(mCamCtrlType);
        initializeCameras();
        uploadSelectedCamera();
        addViewpoint();
        updateLights(true);
        updateEnvMap(true);
        updateMaterials(true);
        uploadResources(); // Upload data after initialization is complete
        updateGeometryStats();
        updateLightStats();
        prepareUI();
    }

    void Scene::initializeCameras()
    {
        for (auto& camera : mCameras)
        {
            updateAnimatable(*camera, *mpAnimationController, true);
            camera->beginFrame();
        }
    }

    void Scene::prepareUI()
    {
        for (uint32_t camId = 0; camId < (uint32_t)mCameras.size(); camId++)
        {
            mCameraList.push_back({ camId, mCameras[camId]->getName() });
        }
    }

    void Scene::updateGeometryStats()
    {
        auto& s = mSceneStats;

        s.uniqueVertexCount = 0;
        s.uniqueTriangleCount = 0;
        s.instancedVertexCount = 0;
        s.instancedTriangleCount = 0;

        for (uint32_t meshID = 0; meshID < getMeshCount(); meshID++)
        {
            const auto& mesh = getMesh(meshID);
            s.uniqueVertexCount += mesh.vertexCount;
            s.uniqueTriangleCount += mesh.getTriangleCount();
        }
        for (uint32_t instanceID = 0; instanceID < getMeshInstanceCount(); instanceID++)
        {
            const auto& instance = getMeshInstance(instanceID);
            const auto& mesh = getMesh(instance.meshID);
            s.instancedVertexCount += mesh.vertexCount;
            s.instancedTriangleCount += mesh.getTriangleCount();
        }
    }

    void Scene::updateRaytracingStats()
    {
        auto& s = mSceneStats;

        s.blasCount = mBlasData.size();
        s.blasCompactedCount = 0;
        s.blasMemoryInBytes = 0;

        for (const auto& blas : mBlasData)
        {
            if (blas.useCompaction) s.blasCompactedCount++;
            s.blasMemoryInBytes += blas.blasByteSize;
        }
    }

    void Scene::updateLightStats()
    {
        auto& s = mSceneStats;

        s.activeLightCount = 0;
        s.totalLightCount = 0;
        s.pointLightCount = 0;
        s.directionalLightCount = 0;
        s.rectLightCount = 0;
        s.sphereLightCount = 0;
        s.distantLightCount = 0;

        for (const auto& light : mLights)
        {
            if (light->isActive()) s.activeLightCount++;
            s.totalLightCount++;

            switch (light->getType())
            {
            case LightType::Point:
                s.pointLightCount++;
                break;
            case LightType::Directional:
                s.directionalLightCount++;
                break;
            case LightType::Rect:
                s.rectLightCount++;
                break;
            case LightType::Sphere:
                s.sphereLightCount++;
                break;
            case LightType::Distant:
                s.distantLightCount++;
                break;
            }
        }
    }

    bool Scene::updateAnimatable(Animatable& animatable, const AnimationController& controller, bool force)
    {
        uint32_t nodeID = animatable.getNodeID();

        // It is possible for this to be called on an object with no associated node in the scene graph (kInvalidNode),
        // e.g. non-animated lights. This check ensures that we return immediately instead of trying to check
        // matrices for a non-existent node.
        if (nodeID == kInvalidNode) return false;

        if (force || (animatable.hasAnimation() && animatable.isAnimated()))
        {
            if (!controller.didMatrixChanged(nodeID) && !force) return false;

            glm::mat4 transform = controller.getGlobalMatrices()[nodeID];
            animatable.updateFromAnimation(transform);
            return true;
        }
        return false;
    }

    Scene::UpdateFlags Scene::updateSelectedCamera(bool forceUpdate)
    {
        auto camera = mCameras[mSelectedCamera];

        if (forceUpdate || (camera->hasAnimation() && camera->isAnimated()))
        {
            updateAnimatable(*camera, *mpAnimationController, forceUpdate);
        }
        else
        {
            mpCamCtrl->update();
        }

        UpdateFlags flags = UpdateFlags::None;
        auto cameraChanges = camera->beginFrame();
        if (mCameraSwitched || cameraChanges != Camera::Changes::None)
        {
            uploadSelectedCamera();
            if (is_set(cameraChanges, Camera::Changes::Movement)) flags |= UpdateFlags::CameraMoved;
            if ((cameraChanges & (~Camera::Changes::Movement)) != Camera::Changes::None) flags |= UpdateFlags::CameraPropertiesChanged;
            if (mCameraSwitched) flags |= UpdateFlags::CameraSwitched;
        }
        mCameraSwitched = false;
        return flags;
    }

    Scene::UpdateFlags Scene::updateLights(bool forceUpdate)
    {
        Light::Changes combinedChanges = Light::Changes::None;

        // Animate lights and get list of changes.
        for (const auto& light : mLights)
        {
            updateAnimatable(*light, *mpAnimationController, forceUpdate);
            auto changes = light->beginFrame();
            combinedChanges |= changes;
        }

        // Update changed lights.
        uint32_t lightCount = 0;

        for (const auto& light : mLights)
        {
            if (!light->isActive()) continue;
            auto changes = light->getChanges();

            if (changes != Light::Changes::None || is_set(combinedChanges, Light::Changes::Active) || forceUpdate)
            {
                // TODO: This is slow since the buffer is not CPU writable. Copy into CPU buffer and upload once instead.
                mpLightsBuffer->setElement(lightCount, light->getData());
            }

            lightCount++;
        }

        if (combinedChanges != Light::Changes::None || forceUpdate)
        {
            mpSceneBlock["lightCount"] = lightCount;
            updateLightStats();
        }

        // Compute update flags.
        UpdateFlags flags = UpdateFlags::None;
        if (is_set(combinedChanges, Light::Changes::Intensity)) flags |= UpdateFlags::LightIntensityChanged;
        if (is_set(combinedChanges, Light::Changes::Position)) flags |= UpdateFlags::LightsMoved;
        if (is_set(combinedChanges, Light::Changes::Direction)) flags |= UpdateFlags::LightsMoved;
        if (is_set(combinedChanges, Light::Changes::Active)) flags |= UpdateFlags::LightCountChanged;
        const Light::Changes otherChanges = ~(Light::Changes::Intensity | Light::Changes::Position | Light::Changes::Direction | Light::Changes::Active);
        if ((combinedChanges & otherChanges) != Light::Changes::None) flags |= UpdateFlags::LightPropertiesChanged;

        return flags;
    }

    Scene::UpdateFlags Scene::updateEnvMap(bool forceUpdate)
    {
        UpdateFlags flags = UpdateFlags::None;

        if (mpEnvMap)
        {
            auto envMapChanges = mpEnvMap->beginFrame();
            if (envMapChanges != EnvMap::Changes::None || forceUpdate)
            {
                if (envMapChanges != EnvMap::Changes::None) flags |= UpdateFlags::EnvMapChanged;
            }
            mpEnvMap->setShaderData(mpSceneBlock[kEnvMap]);
            mpEnvMap->overridePrevData();
        }

        return flags;
    }

    Scene::UpdateFlags Scene::updateMaterials(bool forceUpdate)
    {
        UpdateFlags flags = UpdateFlags::None;

        // Early out if no materials have changed
        if (!forceUpdate && Material::getGlobalUpdates() == Material::UpdateFlags::None) return flags;

        for (uint32_t materialId = 0; materialId < (uint32_t)mMaterials.size(); ++materialId)
        {
            auto& material = mMaterials[materialId];
            auto materialUpdates = material->getUpdates();
            if (forceUpdate || materialUpdates != Material::UpdateFlags::None)
            {
                material->clearUpdates();
                uploadMaterial(materialId);
                flags |= UpdateFlags::MaterialsChanged;
            }
        }

        Material::clearGlobalUpdates();

        return flags;
    }

    Scene::UpdateFlags Scene::update(RenderContext* pContext, double currentTime)
    {
        if (mParticleSystemDesc.size())
        {
            Profiler::startEvent("Simulate Particles");
            uint32_t particleSystemId = 0;
            if (mLastParticleTime == 0.f) mLastParticleTime = currentTime;
            for (auto it = mParticleSystems.begin(); it != mParticleSystems.end(); ++it, ++particleSystemId)
            {
                float elapsedTime = (float)(currentTime - mLastParticleTime);
#ifdef PROCEDURAL_PARTICLE
                (*it)->updateExternalVao(pContext, elapsedTime, getCamera()->getViewMatrix(), getParticleVao(), mpParticleAABBBuffer, mParticleSystemDesc[particleSystemId].vbOffset, 0);
#else
                (*it)->updateExternalVao(pContext, elapsedTime, getCamera()->getViewMatrix(), getParticleVao(), mpParticleAABBBuffer, mParticleSystemDesc[particleSystemId].vbOffset / 4, 0);
#endif
            }
            mLastParticleTime = currentTime;
            Profiler::endEvent("Simulate Particles");
        }

        mUpdates = UpdateFlags::None;

        if (mUseAnimatedVolume && !mPauseVDBAnimation && !mVolumeDescArray.empty())
        {
            // update volumdesc
            mVDBAnimationFrameId = (mVDBAnimationFrameId + 1) % mVDBAnimationFrames;
            // temporary hack to prevent value (that can be changed in UI) being overwritten
            float densityScale = mVolumeDesc.densityScaleFactor;
            float tStep = mVolumeDesc.tStep;
            float LeScale = mVolumeDesc.LeScale;
            float temperatureCutOff = mVolumeDesc.temperatureCutOff;
            float temperatureScale = mVolumeDesc.temperatureScale;
            float velocityScale = mVolumeDesc.velocityScale;
            bool usePrevGridForReproj = mVolumeDesc.usePrevGridForReproj;
            bool hasEmission = mVolumeDesc.hasEmission;
            float3 sigma_s = mVolumeDesc.sigma_s;
            float3 sigma_a = mVolumeDesc.sigma_a;
            float g = mVolumeDesc.PhaseFunctionConstantG;
            mVolumeDesc = mVolumeDescArray[mVDBAnimationFrameId];
            mVolumeDesc.densityScaleFactor = densityScale;
            mVolumeDesc.tStep = tStep;
            mVolumeDesc.lastFrameHasEmission = mVolumeDescArray[mVDBLastAnimationFrameId].hasEmission;
            mVolumeDesc.LeScale = LeScale;
            mVolumeDesc.temperatureCutOff = temperatureCutOff;
            mVolumeDesc.temperatureScale = temperatureScale;
            mVolumeDesc.velocityScale = velocityScale;
            mVolumeDesc.sigma_s = sigma_s;
            mVolumeDesc.sigma_a = sigma_a;
            mVolumeDesc.PhaseFunctionConstantG = g;
            mVolumeDesc.densityScaleFactorByScaling = densityScale / mVolumeWorldScaling;
            mVolumeDesc.hasAnimation = true;
            mVolumeDesc.usePrevGridForReproj = usePrevGridForReproj;
            mpSceneBlock["volumeDesc"].setBlob(mVolumeDesc);
        }
        else if (!mVolumeDescArray.empty())
        {
            mVolumeDesc.hasAnimation = false;
            mVolumeDesc.lastFrameHasEmission = mVolumeDescArray[mVDBLastAnimationFrameId].hasEmission;
            mVolumeDesc.densityScaleFactorByScaling = mVolumeDesc.densityScaleFactor / mVolumeWorldScaling;
            mpSceneBlock["volumeDesc"].setBlob(mVolumeDesc);
        }

        if (mpAnimationController->isExternallyAnimated)
        {
            mUpdates |= UpdateFlags::SceneGraphChanged;
            mUpdates |= UpdateFlags::MeshesMoved;
            mpAnimationController->isExternallyAnimated = false;
        }
        else if (mpAnimationController->animate(pContext, currentTime))
        {
            mUpdates |= UpdateFlags::SceneGraphChanged;
            for (const auto& inst : mMeshInstanceData)
            {
                if (mpAnimationController->didMatrixChanged(inst.globalMatrixID))
                {
                    mUpdates |= UpdateFlags::MeshesMoved;
                }
            }
        }

        mUpdates |= updateSelectedCamera(false);
        mUpdates |= updateLights(false);
        mUpdates |= updateEnvMap(false);
        mUpdates |= updateMaterials(false);
        pContext->flush();
        if (is_set(mUpdates, UpdateFlags::MeshesMoved))
        {
            mTlasCache.clear();
            updateMeshInstances(false);
        }

        // If a transform in the scene changed, update BLASes with skinned meshes
        if (mHasCurveDisplayWidthChanged || mBlasData.size() && (mParticleSystemDesc.size() || mHasSkinnedMesh && is_set(mUpdates, UpdateFlags::SceneGraphChanged)))
        {
            mHasCurveDisplayWidthChanged = false;
            mTlasCache.clear();
            buildBlas(pContext);
        }

        // Update light collection
        if (mpLightCollection && mpLightCollection->update(pContext)) mUpdates |= UpdateFlags::LightCollectionChanged;

        if (mRenderSettings != mPrevRenderSettings)
        {
            mUpdates |= UpdateFlags::RenderSettingsChanged;
            mPrevRenderSettings = mRenderSettings;
        }

        return mUpdates;
    }

    void Scene::renderUI(Gui::Widgets& widget)
    {
        if (mpAnimationController->hasAnimations())
        {
            bool isEnabled = mpAnimationController->isEnabled();
            if (widget.checkbox("Animate Scene", isEnabled)) mpAnimationController->setEnabled(isEnabled);
        }

        auto camera = mCameras[mSelectedCamera];
        if (camera->hasAnimation())
        {
            bool isAnimated = camera->isAnimated();
            if (widget.checkbox("Animate Camera", isAnimated)) camera->setIsAnimated(isAnimated);
        }

        if (widget.var("Camera Speed", mCameraSpeed, 0.f, std::numeric_limits<float>::max(), 0.01f))
        {
            mpCamCtrl->setCameraSpeed(mCameraSpeed);
        }

        if (mCameraList.size() > 1)
        {
            uint32_t camIndex = mSelectedCamera;
            if (widget.dropdown("Selected Camera", mCameraList, camIndex)) selectCamera(camIndex);
        }

        if (widget.button("Add Viewpoint")) addViewpoint();

        if (mViewpoints.size() > 1)
        {
            if (widget.button("Remove Viewpoint", true)) removeViewpoint();

            Gui::DropdownList viewpoints;
            viewpoints.push_back({ 0, "Default Viewpoint" });
            for (uint32_t viewId = 1; viewId < (uint32_t)mViewpoints.size(); viewId++)
            {
                viewpoints.push_back({ viewId, "Viewpoint " + std::to_string(viewId) });
            }
            uint32_t viewIndex = mCurrentViewpoint;
            if (widget.dropdown("Viewpoints", viewpoints, viewIndex)) selectViewpoint(viewIndex);
        }

        if (auto cameraGroup = widget.group("Camera"))
        {
            camera->renderUI(cameraGroup);
        }

        if (auto lightingGroup = widget.group("Render Settings"))
        {
            lightingGroup.checkbox("Use environment light", mRenderSettings.useEnvLight);
            lightingGroup.tooltip("This enables using the environment map as a distant light source.", true);

            lightingGroup.checkbox("Use analytic lights", mRenderSettings.useAnalyticLights);
            lightingGroup.tooltip("This enables using analytic lights.", true);

            lightingGroup.checkbox("Emissive", mRenderSettings.useEmissiveLights);
            lightingGroup.tooltip("This enables using emissive triangles as lights.", true);
        }

        if (mpEnvMap)
        {
            if (auto envMapGroup = widget.group("EnvMap"))
            {
                bool reloadImage = false;
                std::string imageName;
                if (widget.button("Load File")) { reloadImage |= openFileDialog({}, imageName); }
                if (reloadImage)
                {
                    loadEnvMap(imageName);
                    mNewEnvMapLoaded = true;
                }
                mpEnvMap->renderUI(envMapGroup);
            }
        }

        if (auto lightsGroup = widget.group("Lights"))
        {
            uint32_t lightID = 0;
            for (auto& light : mLights)
            {
                auto name = std::to_string(lightID) + ": " + light->getName();
                if (auto lightGroup = lightsGroup.group(name))
                {
                    light->renderUI(lightGroup);
                }
                lightID++;
            }
        }

        if (auto EmissiveMultplierGroup = widget.group("Emissive Multiplier"))
        {
            bool dirty = widget.var("Multiplier", mEmissiveIntensityMultiplier, 0.f, 100.f);
            if (dirty) mpSceneBlock["emissiveIntensityMultiplier"] = mEmissiveIntensityMultiplier;
        }

        if (auto materialsGroup = widget.group("Materials"))
        {
            uint32_t materialID = 0;
            for (auto& material : mMaterials)
            {
                auto name = std::to_string(materialID) + ": " + material->getName();
                if (auto materialGroup = materialsGroup.group(name))
                {
                    if (material->renderUI(materialGroup)) uploadMaterial(materialID);
                }
                materialID++;
            }
        }

        if (auto globalAlphaGroup = widget.group("Global Alpha"))
        {
            widget.var("Surface", mSurfaceGlobalAlphaMultipler, 0.0f, 1.0f);
            widget.var("Particle", mParticleCurveGlobalAlphaMultipler, 0.0f, 1.0f);

            if (mPrevSurfaceGlobalAlphaMultipler != mSurfaceGlobalAlphaMultipler) {
                mPrevSurfaceGlobalAlphaMultipler = mSurfaceGlobalAlphaMultipler;
                mpSceneBlock[kSurfaceGlobalAlphaMultipler] = mSurfaceGlobalAlphaMultipler;
            }
            if (mPrevParticleCurveGlobalAlphaMultipler != mParticleCurveGlobalAlphaMultipler)
            {
                mPrevParticleCurveGlobalAlphaMultipler = mParticleCurveGlobalAlphaMultipler;
                mpSceneBlock[kParticleCurveGlobalAlphaMultipler] = mParticleCurveGlobalAlphaMultipler;
            }
        }

        if (auto psysGroup = widget.group("Particle System"))
        {
            mNewParticleSystemAdded = mParticleSystemManager.createSystemGui(widget, this->shared_from_this());
            //widget.separator();
            if (mParticleSystems.size() > 0)
            {
                mParticleSystemManager.editPropertiesGui(widget, this->shared_from_this());
            }
        }

        if (auto statsGroup = widget.group("Statistics"))
        {
            std::ostringstream oss;

            // Geometry stats.
            oss << "Geometry stats:" << std::endl
                << "  Mesh count: " << getMeshCount() << std::endl
                << "  Mesh instance count: " << getMeshInstanceCount() << std::endl
                << "  Transform matrix count: " << getAnimationController()->getGlobalMatrices().size() << std::endl
                << "  Unique triangle count: " << mSceneStats.uniqueTriangleCount << std::endl
                << "  Unique vertex count: " << mSceneStats.uniqueVertexCount << std::endl
                << "  Instanced triangle count: " << mSceneStats.instancedTriangleCount << std::endl
                << "  Instanced vertex count: " << mSceneStats.instancedVertexCount << std::endl
                << std::endl;

            // Raytracing stats.
            oss << "Raytracing stats:" << std::endl
                << "  BLAS count (total): " << mSceneStats.blasCount << std::endl
                << "  BLAS count (compacted): " << mSceneStats.blasCompactedCount << std::endl
                << "  BLAS memory (bytes): " << mSceneStats.blasMemoryInBytes << std::endl
                << std::endl;

            // Material stats.
            oss << "Materials stats:" << std::endl
                << "  Material count: " << getMaterialCount() << std::endl
                << std::endl;

            // Analytic light stats.
            oss << "Analytic light stats:" << std::endl
                << "  Active light count: " << mSceneStats.activeLightCount << std::endl
                << "  Total light count: " << mSceneStats.totalLightCount << std::endl
                << "  Point light count: " << mSceneStats.pointLightCount << std::endl
                << "  Directional light count: " << mSceneStats.directionalLightCount << std::endl
                << "  Rect light count: " << mSceneStats.rectLightCount << std::endl
                << "  Sphere light count: " << mSceneStats.sphereLightCount << std::endl
                << "  Distant light count: " << mSceneStats.distantLightCount << std::endl
                << std::endl;

            // Emissive light stats.
            oss << "Emissive light stats:" << std::endl;
            if (mpLightCollection)
            {
                auto stats = mpLightCollection->getStats();
                oss << "  Active triangle count: " << stats.trianglesActive << std::endl
                    << "  Active uniform triangle count: " << stats.trianglesActiveUniform << std::endl
                    << "  Active textured triangle count: " << stats.trianglesActiveTextured << std::endl
                    << "  Details:" << std::endl
                    << "    Total mesh count: " << stats.meshLightCount << std::endl
                    << "    Textured mesh count: " << stats.meshesTextured << std::endl
                    << "    Total triangle count: " << stats.triangleCount << std::endl
                    << "    Texture triangle count: " << stats.trianglesTextured << std::endl
                    << "    Culled triangle count: " << stats.trianglesCulled << std::endl;
            }
            else
            {
                oss << "  N/A" << std::endl;
            }
            oss << std::endl;

            // Environment map stats.
            oss << "Environment map:" << std::endl;
            if (mpEnvMap)
            {
                oss << "  Filename: " << mpEnvMap->getFilename() << std::endl;
                oss << "  Resolution: " << mpEnvMap->getEnvMap()->getWidth() << "x" << mpEnvMap->getEnvMap()->getHeight() << std::endl;
            }
            else
            {
                oss << "  N/A" << std::endl;
            }
            oss << std::endl;

            statsGroup.text(oss.str());
        }

        // Filtering mode
        // Camera controller
    }

    bool Scene::renderVolumeUI(Gui::Widgets& widget)
    {
        bool isDirty = false;
        if (auto volumeGroup = widget.group("Volume Settings", false))
        {
            isDirty = widget.var("Ray marching step", mVolumeDesc.tStep, 0.001f, 100.f, 0.001f);
            isDirty |= widget.var("Density scale factor", mVolumeDesc.densityScaleFactor, 0.001f, 2.f, 0.001f);
            float3 tempAlbedo = mVolumeDesc.sigma_s / mVolumeDesc.sigma_t;
            bool isAlbedoChanged = widget.rgbColor("Albedo", tempAlbedo);
            if (isAlbedoChanged)
            {
                mVolumeDesc.sigma_s = mVolumeDesc.sigma_t * tempAlbedo;
                mVolumeDesc.sigma_a = mVolumeDesc.sigma_t - mVolumeDesc.sigma_s;
            }
            isDirty |= isAlbedoChanged;
            isDirty |= widget.var("Phase Function g", mVolumeDesc.PhaseFunctionConstantG, -1.f, 1.f);
            isDirty |= widget.var("Le Scale", mVolumeDesc.LeScale, 0.001f, 10.f, 0.001f);
            isDirty |= widget.var("temperature cutoff", mVolumeDesc.temperatureCutOff, 0.f, 10000.f);
            isDirty |= widget.var("temperature scale", mVolumeDesc.temperatureScale, 0.f, 1000.f, 10.f);
            isDirty |= widget.var("velocity scale", mVolumeDesc.velocityScale, -1000.f, 1000.f, 0.001f);
            isDirty |= widget.checkbox("Use prev volume for reproj", mVolumeDesc.usePrevGridForReproj);

            bool isVolumeTranslationScalingDirty = false;

            isVolumeTranslationScalingDirty |= widget.var("World Translation X", mVolumeWorldTranslation.x, -1000.f, 1000.f);
            isVolumeTranslationScalingDirty |= widget.var("World Translation Y", mVolumeWorldTranslation.y, -1000.f, 1000.f);
            isVolumeTranslationScalingDirty |= widget.var("World Translation Z", mVolumeWorldTranslation.z, -1000.f, 1000.f);
            isVolumeTranslationScalingDirty |= widget.var("World Rotation X", mVolumeWorldRotation.x, -180.f, 180.f, 1.f);
            isVolumeTranslationScalingDirty |= widget.var("World Rotation Y", mVolumeWorldRotation.y, -180.f, 180.f, 1.f);
            isVolumeTranslationScalingDirty |= widget.var("World Rotation Z", mVolumeWorldRotation.z, -180.f, 180.f, 1.f);
            isVolumeTranslationScalingDirty |= widget.var("World Scale", mVolumeWorldScaling, 0.001f, 100.f);

            isDirty |= isVolumeTranslationScalingDirty;

            if (isVolumeTranslationScalingDirty)
            {
                mpSceneBlock["volumeWorldTranslation"] = mVolumeWorldTranslation;
                mpSceneBlock["volumeWorldScaling"] = mVolumeWorldScaling;

                glm::mat4 externalModelToWorldMatrix = computeVolumeExternalModelToWorldMatrix();
                glm::mat4 externalWorldToModelMatrix = glm::inverse(externalModelToWorldMatrix);
                mpSceneBlock["volumeExternalWorldToModelMatrix"] = externalWorldToModelMatrix;
                mpSceneBlock["volumeExternalModelToWorldMatrix"] = externalModelToWorldMatrix;
            }

            if (isDirty) mpSceneBlock["volumeDesc"].setBlob(mVolumeDesc);

            if (mVDBAnimationFrames > 0)
            {
                widget.checkbox("Pause animation", mPauseVDBAnimation);
                widget.var("Select Anim Frame", mVDBAnimationFrameId, 0, mVDBAnimationFrames - 1);
                widget.text("Animation frame " + std::to_string(mVDBAnimationFrameId));
            }
            volumeGroup.release();
        }

        return isDirty;
    }

    bool Scene::useEnvBackground() const
    {
        return mpEnvMap != nullptr;
    }

    bool Scene::useEnvLight() const
    {
        return mRenderSettings.useEnvLight && mpEnvMap != nullptr;
    }

    bool Scene::useAnalyticLights() const
    {
        return mRenderSettings.useAnalyticLights && mLights.empty() == false;
    }

    bool Scene::useEmissiveLights() const
    {
        return mRenderSettings.useEmissiveLights && mpLightCollection != nullptr && mpLightCollection->getActiveLightCount() > 0;
    }

    void Scene::setCamera(const Camera::SharedPtr& pCamera)
    {
        auto name = pCamera->getName();
        for (uint index = 0; index < mCameras.size(); index++)
        {
            if (mCameras[index]->getName() == name)
            {
                selectCamera(index);
                return;
            }
        }
        logWarning("Selected camera " + name + " does not exist.");
        pybind11::print("Selected camera", name, "does not exist.");
    }

    void Scene::selectCamera(uint32_t index)
    {
        if (index == mSelectedCamera) return;
        if (index >= mCameras.size())
        {
            logWarning("Selected camera index " + std::to_string(index) + " is invalid.");
            pybind11::print("Selected camera index", index, "is invalid.");
            return;
        }

        mSelectedCamera = index;
        mCameraSwitched = true;
        setCameraController(mCamCtrlType);
        updateSelectedCamera(false);
    }

    void Scene::resetCamera(bool resetDepthRange)
    {
        auto camera = getCamera();
        float radius = length(mSceneBB.extent);
        camera->setPosition(mSceneBB.center);
        camera->setTarget(mSceneBB.center + float3(0, 0, -1));
        camera->setUpVector(float3(0, 1, 0));

        if (resetDepthRange)
        {
            float nearZ = std::max(0.1f, radius / 750.0f);
            float farZ = radius * 50;
            camera->setDepthRange(nearZ, farZ);
        }
    }

    void Scene::setCameraSpeed(float speed)
    {
        mCameraSpeed = clamp(speed, 0.f, std::numeric_limits<float>::max());
        mpCamCtrl->setCameraSpeed(speed);
    }

    void Scene::addViewpoint()
    {
        auto camera = getCamera();
        addViewpoint(camera->getPosition(), camera->getTarget(), camera->getUpVector(), mSelectedCamera);
    }

    void Scene::addViewpoint(const float3& position, const float3& target, const float3& up, uint32_t cameraIndex)
    {
        Viewpoint viewpoint = { cameraIndex, position, target, up };
        mViewpoints.push_back(viewpoint);
        mCurrentViewpoint = (uint32_t)mViewpoints.size() - 1;
    }

    void Scene::removeViewpoint()
    {
        if (mCurrentViewpoint == 0)
        {
            logWarning("Cannot remove default viewpoint");
            return;
        }
        mViewpoints.erase(mViewpoints.begin() + mCurrentViewpoint);
        mCurrentViewpoint = std::min(mCurrentViewpoint, (uint32_t)mViewpoints.size() - 1);
    }

    void Scene::selectViewpoint(uint32_t index)
    {
        if (index >= mViewpoints.size())
        {
            logWarning("Viewpoint does not exist");
            return;
        }

        auto& viewpoint = mViewpoints[index];
        selectCamera(viewpoint.index);
        auto camera = getCamera();
        camera->setPosition(viewpoint.position);
        camera->setTarget(viewpoint.target);
        camera->setUpVector(viewpoint.up);
        mCurrentViewpoint = index;
    }


    void Scene::setGlobalSurfaceAlphaMultipler(float surfaceGlobalAlphaMultipler)
    {
        mSurfaceGlobalAlphaMultipler = surfaceGlobalAlphaMultipler;
        mpSceneBlock[kSurfaceGlobalAlphaMultipler] = mSurfaceGlobalAlphaMultipler;
    }


    void Scene::setGlobalParticleCurveAlphaMultipler(float AlphaMultipler)
    {
        mParticleCurveGlobalAlphaMultipler = AlphaMultipler;
        mpSceneBlock[kParticleCurveGlobalAlphaMultipler] = mParticleCurveGlobalAlphaMultipler;
    }

    void Scene::updateVolumeDesc()
    {
        mpSceneBlock["volumeDesc"].setBlob(mVolumeDesc);
    }

    Material::SharedPtr Scene::getMaterialByName(const std::string& name) const
    {
        for (const auto& m : mMaterials)
        {
            if (m->getName() == name) return m;
        }

        return nullptr;
    }

    Light::SharedPtr Scene::getLightByName(const std::string& name) const
    {
        for (const auto& l : mLights)
        {
            if (l->getName() == name) return l;
        }

        return nullptr;
    }

    Light::SharedPtr Scene::getLightByID(int lightID) const
    {
        return mLights[lightID];
    }

    void Scene::toggleAnimations(bool animate)
    {
        for (auto& light : mLights) light->setIsAnimated(animate);
        for (auto& camera : mCameras) camera->setIsAnimated(animate);
        mpAnimationController->setEnabled(animate);
    }

    void Scene::setBlasUpdateMode(UpdateMode mode)
    {
        if (mode != mBlasUpdateMode) mRebuildBlas = true;
        mBlasUpdateMode = mode;
    }

    void Scene::createDrawList()
    {
        auto pMatricesBuffer = mpSceneBlock->getBuffer("worldMatrices");
        const glm::mat4* matrices = (glm::mat4*)pMatricesBuffer->map(Buffer::MapType::Read); // #SCENEV2 This will cause the pipeline to flush and sync, but it's probably not too bad as this only happens once

        auto createBuffers = [&](const auto& drawClockwiseMeshes, const auto& drawCounterClockwiseMeshes)
        {
            // Create the draw-indirect buffer
            if (drawCounterClockwiseMeshes.size())
            {
                mDrawCounterClockwiseMeshes.pBuffer = Buffer::create(sizeof(drawCounterClockwiseMeshes[0]) * drawCounterClockwiseMeshes.size(), Resource::BindFlags::IndirectArg, Buffer::CpuAccess::None, drawCounterClockwiseMeshes.data());
                mDrawCounterClockwiseMeshes.pBuffer->setName("Scene::mDrawCounterClockwiseMeshes::pBuffer");
                mDrawCounterClockwiseMeshes.count = (uint32_t)drawCounterClockwiseMeshes.size();
            }

            if (drawClockwiseMeshes.size())
            {
                mDrawClockwiseMeshes.pBuffer = Buffer::create(sizeof(drawClockwiseMeshes[0]) * drawClockwiseMeshes.size(), Resource::BindFlags::IndirectArg, Buffer::CpuAccess::None, drawClockwiseMeshes.data());
                mDrawClockwiseMeshes.pBuffer->setName("Scene::mDrawClockwiseMeshes::pBuffer");
                mDrawClockwiseMeshes.count = (uint32_t)drawClockwiseMeshes.size();
            }

            size_t drawCount = drawClockwiseMeshes.size() + drawCounterClockwiseMeshes.size();
            assert(drawCount <= std::numeric_limits<uint32_t>::max());
        };

        if (hasIndexBuffer())
        {
            std::vector<D3D12_DRAW_INDEXED_ARGUMENTS> drawClockwiseMeshes, drawCounterClockwiseMeshes;

            for (const auto& instance : mMeshInstanceData)
            {
                const auto& mesh = mMeshDesc[instance.meshID];
                const auto& transform = matrices[instance.globalMatrixID];

                D3D12_DRAW_INDEXED_ARGUMENTS draw;
                draw.IndexCountPerInstance = mesh.indexCount;
                draw.InstanceCount = 1;
                draw.StartIndexLocation = mesh.ibOffset;
                draw.BaseVertexLocation = mesh.vbOffset;
                draw.StartInstanceLocation = (uint32_t)(drawClockwiseMeshes.size() + drawCounterClockwiseMeshes.size());

                (doesTransformFlip(transform)) ? drawClockwiseMeshes.push_back(draw) : drawCounterClockwiseMeshes.push_back(draw);
            }
            createBuffers(drawClockwiseMeshes, drawCounterClockwiseMeshes);
        }
        else
        {
            std::vector<D3D12_DRAW_ARGUMENTS> drawClockwiseMeshes, drawCounterClockwiseMeshes;

            for (const auto& instance : mMeshInstanceData)
            {
                const auto& mesh = mMeshDesc[instance.meshID];
                const auto& transform = matrices[instance.globalMatrixID];
                assert(mesh.indexCount == 0);

                D3D12_DRAW_ARGUMENTS draw;
                draw.VertexCountPerInstance = mesh.vertexCount;
                draw.InstanceCount = 1;
                draw.StartVertexLocation = mesh.vbOffset;
                draw.StartInstanceLocation = (uint32_t)(drawClockwiseMeshes.size() + drawCounterClockwiseMeshes.size());

                (doesTransformFlip(transform)) ? drawClockwiseMeshes.push_back(draw) : drawCounterClockwiseMeshes.push_back(draw);
            }
            createBuffers(drawClockwiseMeshes, drawCounterClockwiseMeshes);
        }
    }

    void Scene::sortMeshes()
    {
        // We first sort meshes into groups with the same transform.
        // The mesh instances list is then reordered to match this order.
        //
        // For ray tracing, we create one BLAS per mesh group and the mesh instances
        // can therefore be directly indexed by [InstanceID() + GeometryIndex()].
        // This avoids the need to have a lookup table from hit IDs to mesh instance.

        // Build a list of mesh instance indices per mesh.
        std::vector<std::vector<size_t>> instanceLists(mMeshDesc.size());
        for (size_t i = 0; i < mMeshInstanceData.size(); i++)
        {
            assert(mMeshInstanceData[i].meshID < instanceLists.size());
            instanceLists[mMeshInstanceData[i].meshID].push_back(i);
        }

        // The non-instanced meshes are grouped based on what global matrix ID their transform is.
        std::unordered_map<uint32_t, std::vector<uint32_t>> nodeToMeshList;
        for (uint32_t meshId = 0; meshId < (uint32_t)instanceLists.size(); meshId++)
        {
            const auto& instanceList = instanceLists[meshId];
            if (instanceList.size() > 1) continue; // Only processing non-instanced meshes here

            assert(instanceList.size() == 1);
            uint32_t globalMatrixId = mMeshInstanceData[instanceList[0]].globalMatrixID;
            nodeToMeshList[globalMatrixId].push_back(meshId);
        }

        // Build final result. Format is a list of Mesh ID's per mesh group.

        // This should currently only be run on scene initialization.
        //assert(mMeshGroups.empty());

        if (!mMeshGroups.empty()) mMeshGroups.clear();

        // Non-instanced meshes were sorted above so just copy each list.
        for (const auto& it : nodeToMeshList) mMeshGroups.push_back({ it.second });

        // Meshes that have multiple instances go in their own groups.
        for (uint32_t meshId = 0; meshId < (uint32_t)instanceLists.size(); meshId++)
        {
            const auto& instanceList = instanceLists[meshId];
            if (instanceList.size() == 1) continue; // Only processing instanced meshes here
            mMeshGroups.push_back({ std::vector<uint32_t>({ meshId }) });
        }

        // Calculate mapping from new mesh instance ID to existing instance index.
        // Here, just append existing instance ID's in order they appear in the mesh groups.
        std::vector<size_t> instanceMapping;
        for (const auto& meshGroup : mMeshGroups)
        {
            for (const uint32_t meshId : meshGroup.meshList)
            {
                const auto& instanceList = instanceLists[meshId];
                for (size_t idx : instanceList)
                {
                    instanceMapping.push_back(idx);
                }
            }
        }
        assert(instanceMapping.size() == mMeshInstanceData.size());
        {
            // Check that all indices exist
            std::set<size_t> instanceIndices(instanceMapping.begin(), instanceMapping.end());
            assert(instanceIndices.size() == mMeshInstanceData.size());
        }

        // Now reorder mMeshInstanceData based on the new mapping.
        // We'll make a copy of the existing data first, and the populate the array.
        std::vector<MeshInstanceData> prevInstanceData = mMeshInstanceData;
        for (size_t i = 0; i < mMeshInstanceData.size(); i++)
        {
            assert(instanceMapping[i] < prevInstanceData.size());
            mMeshInstanceData[i] = prevInstanceData[instanceMapping[i]];
        }

        // Create mapping of meshes to their instances.
        mMeshIdToInstanceIds.clear();
        mMeshIdToInstanceIds.resize(mMeshDesc.size());
        for (uint32_t instId = 0; instId < (uint32_t)mMeshInstanceData.size(); instId++)
        {
            mMeshIdToInstanceIds[mMeshInstanceData[instId].meshID].push_back(instId);
        }
    }

    void Scene::initGeomDesc()
    {
        assert(mBlasData.empty());

        const VertexBufferLayout::SharedConstPtr& pVbLayout = mpVao->getVertexLayout()->getBufferLayout(kStaticDataBufferIndex);
        const Buffer::SharedPtr& pVb = mpVao->getVertexBuffer(kStaticDataBufferIndex);
        const Buffer::SharedPtr& pIb = mpVao->getIndexBuffer();

        assert(mMeshGroups.size() > 0);

#ifdef ONE_CURVE_PER_BLAS
        mBlasData.resize(mMeshGroups.size() + mParticleSystemDesc.size() + mCurveDesc.size());
#else
        mBlasData.resize(mMeshGroups.size() + mParticleSystemDesc.size() + !mCurveDesc.empty());
#endif
        mRebuildBlas = true;
        mHasSkinnedMesh = false;

        for (size_t i = 0; i < mMeshGroups.size(); i++)
        {
            const auto& meshList = mMeshGroups[i].meshList;
            auto& blas = mBlasData[i];
            auto& geomDescs = blas.geomDescs;
            geomDescs.resize(meshList.size());

            for (size_t j = 0; j < meshList.size(); j++)
            {
                const MeshDesc& mesh = mMeshDesc[meshList[j]];
                blas.hasSkinnedMesh |= mMeshHasDynamicData[meshList[j]];

                D3D12_RAYTRACING_GEOMETRY_DESC& desc = geomDescs[j];
                desc.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
                desc.Triangles.Transform3x4 = 0;

                // If this is an opaque mesh, set the opaque flag
                const auto& material = mMaterials[mesh.materialID];
                bool opaque = material->getAlphaMode() == AlphaModeOpaque;
                desc.Flags = opaque ? D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE : D3D12_RAYTRACING_GEOMETRY_FLAG_NONE;
                desc.Flags |= D3D12_RAYTRACING_GEOMETRY_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION;

                // Set the position data
                desc.Triangles.VertexBuffer.StartAddress = pVb->getGpuAddress() + (mesh.vbOffset * pVbLayout->getStride());
                desc.Triangles.VertexBuffer.StrideInBytes = pVbLayout->getStride();
                desc.Triangles.VertexCount = mesh.vertexCount;
                desc.Triangles.VertexFormat = getDxgiFormat(pVbLayout->getElementFormat(0));

                // Set index data
                if (pIb)
                {
                    desc.Triangles.IndexBuffer = pIb->getGpuAddress() + (mesh.ibOffset * getFormatBytesPerBlock(mpVao->getIndexBufferFormat()));
                    desc.Triangles.IndexCount = mesh.indexCount;
                    desc.Triangles.IndexFormat = getDxgiFormat(mpVao->getIndexBufferFormat());
                }
                else
                {
                    assert(mesh.indexCount == 0);
                    desc.Triangles.IndexBuffer = NULL;
                    desc.Triangles.IndexCount = 0;
                    desc.Triangles.IndexFormat = DXGI_FORMAT_UNKNOWN;
                }
            }

            mHasSkinnedMesh |= blas.hasSkinnedMesh;
        }

        if (mParticleSystemDesc.size() > 0)
        {
#ifdef PROCEDURAL_PARTICLE
            const size_t blasDataOffset = mMeshGroups.size();
            for (size_t i = 0; i < mParticleSystemDesc.size(); i++)
            {
                auto& blas = mBlasData[blasDataOffset + i];
                blas.hasSkinnedMesh = true;
                blas.updateMode = UpdateMode::Rebuild;
                blas.useCompaction = false;
                auto& geomDescs = blas.geomDescs;
                geomDescs.resize(1);

                const ParticleSystemDesc& particleSystem = mParticleSystemDesc[i];

                D3D12_RAYTRACING_GEOMETRY_DESC& desc = geomDescs[0];
                desc.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS;
                desc.AABBs.AABBCount = particleSystem.vertexCount;
                desc.AABBs.AABBs.StartAddress = mpParticleAABBBuffer->getGpuAddress() + particleSystem.vbOffset * 24;
                desc.AABBs.AABBs.StrideInBytes = 24;

                // If this is an opaque mesh, set the opaque flag
                const auto& material = mMaterials[particleSystem.materialID];
                bool opaque = (material->getAlphaMode() == AlphaModeOpaque) && material->getSpecularTransmission() == 0.f;
                desc.Flags = opaque ? D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE : D3D12_RAYTRACING_GEOMETRY_FLAG_NONE;
                desc.Flags |= D3D12_RAYTRACING_GEOMETRY_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION;

                mHasSkinnedMesh |= blas.hasSkinnedMesh;
            }
#else
            const VertexBufferLayout::SharedConstPtr& pVbLayout = mpParticleVao->getVertexLayout()->getBufferLayout(kStaticDataBufferIndex);
            const Buffer::SharedPtr& pVb = mpParticleVao->getVertexBuffer(kStaticDataBufferIndex);
            const Buffer::SharedPtr& pIb = mpParticleVao->getIndexBuffer();
            assert(pIb);
            const size_t blasDataOffset = mMeshGroups.size();

            for (size_t i = 0; i < mParticleSystemDesc.size(); i++)
            {
                auto& blas = mBlasData[blasDataOffset + i];
                blas.hasSkinnedMesh = true;
                blas.updateMode = UpdateMode::Rebuild;
                blas.useCompaction = false;
                auto& geomDescs = blas.geomDescs;
                geomDescs.resize(1);

                {
                    const ParticleSystemDesc& particleSystem = mParticleSystemDesc[i];

                    D3D12_RAYTRACING_GEOMETRY_DESC& desc = geomDescs[0];
                    desc.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
                    desc.Triangles.Transform3x4 = 0;

                    // If this is an opaque mesh, set the opaque flag
                    const auto& material = mMaterials[particleSystem.materialID];
                    bool opaque = (material->getAlphaMode() == AlphaModeOpaque) && material->getSpecularTransmission() == 0.f;
                    desc.Flags = opaque ? D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE : D3D12_RAYTRACING_GEOMETRY_FLAG_NONE;
                    desc.Flags |= D3D12_RAYTRACING_GEOMETRY_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION;

                    // Set the position data
                    desc.Triangles.VertexBuffer.StartAddress = pVb->getGpuAddress() + (particleSystem.vbOffset * pVbLayout->getStride());
                    desc.Triangles.VertexBuffer.StrideInBytes = pVbLayout->getStride();
                    desc.Triangles.VertexCount = particleSystem.vertexCount;
                    desc.Triangles.VertexFormat = getDxgiFormat(pVbLayout->getElementFormat(0));

                    // Set index data
                    desc.Triangles.IndexBuffer = pIb->getGpuAddress() + (particleSystem.ibOffset * getFormatBytesPerBlock(mpParticleVao->getIndexBufferFormat()));
                    desc.Triangles.IndexCount = particleSystem.indexCount;
                    desc.Triangles.IndexFormat = getDxgiFormat(mpParticleVao->getIndexBufferFormat());
                }

                mHasSkinnedMesh |= blas.hasSkinnedMesh;
            }
#endif
        }


        if (mCurveDesc.size() > 0)
        {
            // add curves (bezier patches)
            // we assume that all curves belong to one blas
            const size_t blasDataOffset = mMeshGroups.size() + mParticleSystemDesc.size();

            int globalPatchId = 0;
            ResourceBindFlags vbBindFlags = ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess;

            std::vector<D3D12_RAYTRACING_AABB> aabbs;

            for (uint32_t curveId = 0; curveId < (uint32_t)mCurveDesc.size(); curveId++)
            {
                int numPatches = mCurveDesc[curveId].getPatchCount();
                // fill CPU AABB array
                for (uint32_t patchId = 0; patchId < (uint32_t)numPatches; patchId++)
                {
                    D3D12_RAYTRACING_AABB aabb;
                    aabb.MinX = mCurvePatchBBs[globalPatchId].getMinPos().x;
                    aabb.MinY = mCurvePatchBBs[globalPatchId].getMinPos().y;
                    aabb.MinZ = mCurvePatchBBs[globalPatchId].getMinPos().z;
                    aabb.MaxX = mCurvePatchBBs[globalPatchId].getMaxPos().x;
                    aabb.MaxY = mCurvePatchBBs[globalPatchId].getMaxPos().y;
                    aabb.MaxZ = mCurvePatchBBs[globalPatchId].getMaxPos().z;
                    aabbs.push_back(aabb);
                    globalPatchId++;
                }
            }

            mpCurvePatchAABBBuffer = Buffer::create(aabbs.size() * sizeof(D3D12_RAYTRACING_AABB), vbBindFlags, Buffer::CpuAccess::None, aabbs.data());

#ifdef ONE_CURVE_PER_BLAS
            for (uint32_t curveId = 0; curveId < (uint32_t)mCurveDesc.size(); curveId++)
            {
                auto& blas = mBlasData[blasDataOffset + curveId];
                blas.geomDescs.resize(1);
                D3D12_RAYTRACING_GEOMETRY_DESC& desc = blas.geomDescs[0];
                int numPatches = mCurveDesc[curveId].getPatchCount();
                desc.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS;
                desc.AABBs.AABBCount = numPatches;
                desc.AABBs.AABBs.StartAddress = mpCurvePatchAABBBuffer->getGpuAddress() + mCurveDesc[curveId].vbOffset / mCurveDesc[curveId].getVerticesPerPatch() * 24;
                desc.AABBs.AABBs.StrideInBytes = 24;
            }
#else
            mBlasData[blasDataOffset].geomDescs.resize(mCurveDesc.size());
            mBlasData[blasDataOffset].hasSkinnedMesh = true; // temporal hack for rebuilding when display width changes

            for (uint32_t curveId = 0; curveId < (uint32_t)mCurveDesc.size(); curveId++)
            {
                D3D12_RAYTRACING_GEOMETRY_DESC& desc = mBlasData[blasDataOffset].geomDescs[curveId];
                int numPatches = mCurveDesc[curveId].getPatchCount();
                desc.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS;
                desc.AABBs.AABBCount = numPatches;
                desc.AABBs.AABBs.StartAddress = mpCurvePatchAABBBuffer->getGpuAddress() + mCurveDesc[curveId].vbOffset / (uint32_t)mCurveDesc[curveId].getVerticesPerPatch() * 24;
                desc.AABBs.AABBs.StrideInBytes = 24;
            }
#endif
            mHasSkinnedMesh |= mBlasData[blasDataOffset].hasSkinnedMesh;
        }
    }

    void Scene::buildBlas(RenderContext* pContext)
    {
        PROFILE("buildBlas");

        // Add barriers for the VB and IB which will be accessed by the build.
        const Buffer::SharedPtr& pVb = mpVao->getVertexBuffer(kStaticDataBufferIndex);
        const Buffer::SharedPtr& pIb = mpVao->getIndexBuffer();
        pContext->resourceBarrier(pVb.get(), Resource::State::NonPixelShader);
        if (pIb) pContext->resourceBarrier(pIb.get(), Resource::State::NonPixelShader);

        if (mpParticleVao)
        {
            const Buffer::SharedPtr& pVb = mpParticleVao->getVertexBuffer(kStaticDataBufferIndex);
            const Buffer::SharedPtr& pIb = mpParticleVao->getIndexBuffer();
            pContext->resourceBarrier(pVb.get(), Resource::State::NonPixelShader);
            if (pIb) pContext->resourceBarrier(pIb.get(), Resource::State::NonPixelShader);
        }

        if (mpCurveVao)
        {
            const Buffer::SharedPtr& pVb = mpCurveVao->getVertexBuffer(kStaticDataBufferIndex);
            pContext->resourceBarrier(pVb.get(), Resource::State::NonPixelShader);
        }

        // On the first time, or if a full rebuild is necessary we will:
        // - Update all build inputs and prebuild info
        // - Calculate total intermediate buffer sizes
        // - Build all BLASes into an intermediate buffer
        // - Calculate total compacted buffer size
        // - Compact/clone all BLASes to their final location
        if (mRebuildBlas)
        {
            uint64_t totalMaxBlasSize = 0;
            uint64_t totalScratchSize = 0;

            for (auto& blas : mBlasData)
            {
                // Determine how BLAS build/update should be done.
                // The default choice is to compact all static BLASes and those that don't need to be rebuilt every frame. For those compaction just adds overhead.
                // TODO: Add compaction on/off switch for profiling.
                // TODO: Disable compaction for skinned meshes if update performance becomes a problem.
                if (blas.updateMode != UpdateMode::Rebuild) blas.updateMode = mBlasUpdateMode;
                if (blas.useCompaction != false) blas.useCompaction = !blas.hasSkinnedMesh || blas.updateMode != UpdateMode::Rebuild;

                // Setup build parameters.
                D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS& inputs = blas.buildInputs;
                inputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
                inputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
                inputs.NumDescs = (uint32_t)blas.geomDescs.size();
                inputs.pGeometryDescs = blas.geomDescs.data();
                inputs.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_NONE;

                // Add necessary flags depending on settings.
                if (blas.useCompaction)
                {
                    inputs.Flags |= D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_ALLOW_COMPACTION;
                }
                if (blas.hasSkinnedMesh && blas.updateMode == UpdateMode::Refit)
                {
                    inputs.Flags |= D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_ALLOW_UPDATE;
                }

                // Set optional performance hints.
                // TODO: Set FAST_BUILD for skinned meshes if update/rebuild performance becomes a problem.
                // TODO: Add FAST_TRACE on/off switch for profiling. It is disabled by default as it is scene-dependent.
                //if (!blas.hasSkinnedMesh)
                //{
                //    inputs.Flags |= D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;
                //}

                // Get prebuild info.
                GET_COM_INTERFACE(gpDevice->getApiHandle(), ID3D12Device5, pDevice5);
                pDevice5->GetRaytracingAccelerationStructurePrebuildInfo(&inputs, &blas.prebuildInfo);

                // Figure out the padded allocation sizes to have proper alignement.
                uint64_t paddedMaxBlasSize = align_to(D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BYTE_ALIGNMENT, blas.prebuildInfo.ResultDataMaxSizeInBytes);
                blas.blasByteOffset = totalMaxBlasSize;
                totalMaxBlasSize += paddedMaxBlasSize;

                uint64_t scratchSize = std::max(blas.prebuildInfo.ScratchDataSizeInBytes, blas.prebuildInfo.UpdateScratchDataSizeInBytes);
                uint64_t paddedScratchSize = align_to(D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BYTE_ALIGNMENT, scratchSize);
                blas.scratchByteOffset = totalScratchSize;
                totalScratchSize += paddedScratchSize;
            }

            // Allocate intermediate buffers and scratch buffer.
            // The scratch buffer we'll retain because it's needed for subsequent rebuilds and updates.
            // TODO: Save memory by reducing the scratch buffer to the minimum required for the dynamic objects.
            if (mpBlasScratch == nullptr || mpBlasScratch->getSize() < totalScratchSize)
            {
                mpBlasScratch = Buffer::create(totalScratchSize, Buffer::BindFlags::UnorderedAccess, Buffer::CpuAccess::None);
                mpBlasScratch->setName("Scene::mpBlasScratch");
            }
            else
            {
                // If we didn't need to reallocate, just insert a barrier so it's safe to use.
                pContext->uavBarrier(mpBlasScratch.get());
            }

            Buffer::SharedPtr pDestBuffer = Buffer::create(totalMaxBlasSize, Buffer::BindFlags::AccelerationStructure, Buffer::CpuAccess::None);

            const size_t postBuildInfoSize = sizeof(D3D12_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_COMPACTED_SIZE_DESC);
            static_assert(postBuildInfoSize == sizeof(D3D12_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_CURRENT_SIZE_DESC));
            Buffer::SharedPtr pPostbuildInfoBuffer = Buffer::create(mBlasData.size() * postBuildInfoSize, Buffer::BindFlags::None, Buffer::CpuAccess::Read);

            // Build the BLASes into the intermediate destination buffer.
            // We output postbuild info to a separate buffer to find out the final size requirements.
            assert(pDestBuffer && pPostbuildInfoBuffer && mpBlasScratch);
            uint64_t postBuildInfoOffset = 0;

            for (const auto& blas : mBlasData)
            {
                D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC asDesc = {};
                asDesc.Inputs = blas.buildInputs;
                asDesc.ScratchAccelerationStructureData = mpBlasScratch->getGpuAddress() + blas.scratchByteOffset;
                asDesc.DestAccelerationStructureData = pDestBuffer->getGpuAddress() + blas.blasByteOffset;

                // Need to find out the the postbuild compacted BLAS size to know the final allocation size.
                D3D12_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_DESC postbuildInfoDesc = {};
                postbuildInfoDesc.InfoType = blas.useCompaction ? D3D12_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_COMPACTED_SIZE : D3D12_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_CURRENT_SIZE;
                postbuildInfoDesc.DestBuffer = pPostbuildInfoBuffer->getGpuAddress() + postBuildInfoOffset;
                postBuildInfoOffset += postBuildInfoSize;

                GET_COM_INTERFACE(pContext->getLowLevelData()->getCommandList(), ID3D12GraphicsCommandList4, pList4);
                pList4->BuildRaytracingAccelerationStructure(&asDesc, 1, &postbuildInfoDesc);
            }

            // Release scratch buffer if there is no animated content. We will not need it.
            if (!mHasSkinnedMesh) mpBlasScratch.reset();

            // Read back the calculated final size requirements for each BLAS.
            // For this purpose we have to flush and map the postbuild info buffer for readback.
            // TODO: We could copy to a staging buffer first and wait on a GPU fence for when it's ready.
            // But there is no other work to do inbetween so it probably wouldn't help. This is only done once at startup anyway.
            pContext->flush(true);
            const D3D12_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_COMPACTED_SIZE_DESC* postBuildInfo =
                (const D3D12_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_COMPACTED_SIZE_DESC*)pPostbuildInfoBuffer->map(Buffer::MapType::Read);

            uint64_t totalBlasSize = 0;
            for (size_t i = 0; i < mBlasData.size(); i++)
            {
                auto& blas = mBlasData[i];
                blas.blasByteSize = postBuildInfo[i].CompactedSizeInBytes;
                assert(blas.blasByteSize <= blas.prebuildInfo.ResultDataMaxSizeInBytes);
                uint64_t paddedBlasSize = align_to(D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BYTE_ALIGNMENT, blas.blasByteSize);
                totalBlasSize += paddedBlasSize;
            }
            pPostbuildInfoBuffer->unmap();

            // Allocate final BLAS buffer.
            if (mpBlas == nullptr || mpBlas->getSize() < totalBlasSize)
            {
                mpBlas = Buffer::create(totalBlasSize, Buffer::BindFlags::AccelerationStructure, Buffer::CpuAccess::None);
                mpBlas->setName("Scene::mpBlas");
            }
            else
            {
                // If we didn't need to reallocate, just insert a barrier so it's safe to use.
                pContext->uavBarrier(mpBlas.get());
            }

            // Insert barriers for the intermediate buffer. This is probably not necessary since we flushed above, but it's not going to hurt.
            pContext->uavBarrier(pDestBuffer.get());

            // Compact/clone all BLASes to their final location.
            uint64_t blasOffset = 0;
            for (auto& blas : mBlasData)
            {
                GET_COM_INTERFACE(pContext->getLowLevelData()->getCommandList(), ID3D12GraphicsCommandList4, pList4);
                pList4->CopyRaytracingAccelerationStructure(
                    mpBlas->getGpuAddress() + blasOffset,
                    pDestBuffer->getGpuAddress() + blas.blasByteOffset,
                    blas.useCompaction ? D3D12_RAYTRACING_ACCELERATION_STRUCTURE_COPY_MODE_COMPACT : D3D12_RAYTRACING_ACCELERATION_STRUCTURE_COPY_MODE_CLONE);

                uint64_t paddedBlasSize = align_to(D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BYTE_ALIGNMENT, blas.blasByteSize);
                blas.blasByteOffset = blasOffset;
                blasOffset += paddedBlasSize;
            }
            assert(blasOffset == totalBlasSize);

            // Insert barrier. The BLAS buffer is now ready for use.
            pContext->uavBarrier(mpBlas.get());

            updateRaytracingStats();
            mRebuildBlas = false;

            return;
        }

        // If we get here, all BLASes have previously been built and compacted. We will:
        // - Early out if there are no animated meshes.
        // - Update or rebuild in-place the ones that are animated.
        assert(!mRebuildBlas);
        if (mHasSkinnedMesh == false) return;

        // Insert barriers. The buffers are now ready to be written to.
        assert(mpBlas && mpBlasScratch);
        pContext->uavBarrier(mpBlas.get());
        pContext->uavBarrier(mpBlasScratch.get());

        for (const auto& blas : mBlasData)
        {
            // Skip updating BLASes not containing skinned meshes.
            if (!blas.hasSkinnedMesh) continue;

            // Build/update BLAS.
            D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC asDesc = {};
            asDesc.Inputs = blas.buildInputs;
            asDesc.ScratchAccelerationStructureData = mpBlasScratch->getGpuAddress() + blas.scratchByteOffset;
            asDesc.DestAccelerationStructureData = mpBlas->getGpuAddress() + blas.blasByteOffset;

            if (blas.updateMode == UpdateMode::Refit)
            {
                // Set source address to destination address to update in place.
                asDesc.SourceAccelerationStructureData = asDesc.DestAccelerationStructureData;
                asDesc.Inputs.Flags |= D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PERFORM_UPDATE;
            }
            else
            {
                // We'll rebuild in place. The BLAS should not be compacted, check that size matches prebuild info.
                assert(blas.blasByteSize == blas.prebuildInfo.ResultDataMaxSizeInBytes);
            }

            GET_COM_INTERFACE(pContext->getLowLevelData()->getCommandList(), ID3D12GraphicsCommandList4, pList4);
            pList4->BuildRaytracingAccelerationStructure(&asDesc, 0, nullptr);
        }

        // Insert barrier. The BLAS buffer is now ready for use.
        pContext->uavBarrier(mpBlas.get());
    }

    void Scene::fillInstanceDesc(std::vector<D3D12_RAYTRACING_INSTANCE_DESC>& instanceDescs, uint32_t rayCount, bool perMeshHitEntry) const
    {
        assert(mpBlas);
        instanceDescs.clear();
        uint32_t instanceContributionToHitGroupIndex = 0;
        uint32_t instanceId = 0;

        for (size_t i = 0; i < mMeshGroups.size(); i++)
        {
            const auto& meshList = mMeshGroups[i].meshList;

            D3D12_RAYTRACING_INSTANCE_DESC desc = {};
            desc.AccelerationStructure = mpBlas->getGpuAddress() + mBlasData[i].blasByteOffset;
            desc.InstanceMask = 0xFF;
            desc.InstanceContributionToHitGroupIndex = perMeshHitEntry ? instanceContributionToHitGroupIndex : 0;
            instanceContributionToHitGroupIndex += rayCount * (uint32_t)meshList.size();

            // If multiple meshes are in a BLAS:
            // - Their global matrix is the same.
            // - From sortMeshes(), each mesh in the BLAS is guaranteed to be non-instanced, so only one INSTANCE_DESC is needed
            if (meshList.size() > 1)
            {
                assert(mMeshIdToInstanceIds[meshList[0]].size() == 1);
                assert(mMeshIdToInstanceIds[meshList[0]][0] == instanceId); // Mesh instances are sorted by instanceId
                desc.InstanceID = instanceId;
                instanceId += (uint32_t)meshList.size();

                // Any instances of the mesh will get you the correct matrix, so just pick the first mesh then the first instance.
                uint32_t matrixId = mMeshInstanceData[desc.InstanceID].globalMatrixID;
                glm::mat4 transform4x4 = transpose(mpAnimationController->getGlobalMatrices()[matrixId]);
                std::memcpy(desc.Transform, &transform4x4, sizeof(desc.Transform));
                instanceDescs.push_back(desc);
            }
            // If only one mesh is in the BLAS, there CAN be multiple instances of it. It is either:
            // - A non-instanced mesh that was unable to be merged with others
            // - A mesh with multiple instances
            else
            {
                assert(meshList.size() == 1);
                const auto& instanceList = mMeshIdToInstanceIds[meshList[0]];

                // For every instance of the mesh, create an INSTANCE_DESC
                for (uint32_t instId : instanceList)
                {
                    assert(instId == instanceId); // Mesh instances are sorted by instanceId
                    desc.InstanceID = instanceId++;
                    uint32_t matrixId = mMeshInstanceData[desc.InstanceID].globalMatrixID;
                    glm::mat4 transform4x4 = transpose(mpAnimationController->getGlobalMatrices()[matrixId]);
                    std::memcpy(desc.Transform, &transform4x4, sizeof(desc.Transform));
                    instanceDescs.push_back(desc);
                }
            }
        }

        size_t blasDataOffset = mMeshGroups.size();
        //instanceId = 0; // reset instanceId to 0
        for (size_t i = 0; i < mParticleSystemDesc.size(); i++)
        {
            D3D12_RAYTRACING_INSTANCE_DESC desc = {};
            desc.AccelerationStructure = mpBlas->getGpuAddress() + mBlasData[blasDataOffset + i].blasByteOffset;
            desc.InstanceMask = 0xFF;
            desc.InstanceContributionToHitGroupIndex = perMeshHitEntry ? instanceContributionToHitGroupIndex : 0;
            instanceContributionToHitGroupIndex += rayCount;

            // If multiple meshes are in a BLAS:
            // - Their global matrix is the same.
            // - From sortMeshes(), each mesh in the BLAS is guaranteed to be non-instanced, so only one INSTANCE_DESC is needed
            {
                desc.InstanceID = instanceId++;
                // Any instances of the mesh will get you the correct matrix, so just pick the first mesh then the first instance.
                glm::mat4 transform4x4(1); // currently not supporting transformation matrix for particle system
                std::memcpy(desc.Transform, &transform4x4, sizeof(desc.Transform));
                instanceDescs.push_back(desc);
            }
        }

#ifdef ONE_CURVE_PER_BLAS
        blasDataOffset = mMeshGroups.size() + mParticleSystemDesc.size();
        for (uint32_t i = 0; i < (uint32_t)mCurveDesc.size(); i++)
        {
            D3D12_RAYTRACING_INSTANCE_DESC desc = {};
            desc.AccelerationStructure = mpBlas->getGpuAddress() + mBlasData[blasDataOffset + i].blasByteOffset;
            desc.InstanceMask = 0xFF;
            desc.InstanceContributionToHitGroupIndex = perMeshHitEntry ? instanceContributionToHitGroupIndex : 0;
            instanceContributionToHitGroupIndex += rayCount;

            // TODO: consider curve instancing
            {
                desc.InstanceID = instanceId++;
                // Any instances of the mesh will get you the correct matrix, so just pick the first mesh then the first instance.
                glm::mat4 transform4x4(1); // currently not supporting transformation matrix for curves
                std::memcpy(desc.Transform, &transform4x4, sizeof(desc.Transform));
                instanceDescs.push_back(desc);
            }
        }
#else
        if (mCurveDesc.size() > 0)
        {
            uint32_t curveBlasOffset = (uint32_t)(mMeshGroups.size() + mParticleSystemDesc.size());
            auto& curveList = mBlasData[curveBlasOffset].geomDescs;
            D3D12_RAYTRACING_INSTANCE_DESC desc = {};
            desc.AccelerationStructure = mpBlas->getGpuAddress() + mBlasData[curveBlasOffset].blasByteOffset;
            desc.InstanceMask = 0xFF;
            desc.InstanceContributionToHitGroupIndex = perMeshHitEntry ? instanceContributionToHitGroupIndex : 0;
            instanceContributionToHitGroupIndex += rayCount * (uint32_t)curveList.size();

            // TODO: consider curve instancing
            {
                desc.InstanceID = instanceId;
                instanceId += (uint32_t)curveList.size();
                // Any instances of the mesh will get you the correct matrix, so just pick the first mesh then the first instance.
                glm::mat4 transform4x4(1); // currently not supporting transformation matrix for curves
                std::memcpy(desc.Transform, &transform4x4, sizeof(desc.Transform));
                instanceDescs.push_back(desc);
            }
        }
#endif

    }

    void Scene::buildTlas(RenderContext* pContext, uint32_t rayCount, bool perMeshHitEntry)
    {
        PROFILE("buildTlas");

        TlasData tlas;
        auto it = mTlasCache.find(rayCount);
        if (it != mTlasCache.end()) tlas = it->second;

        fillInstanceDesc(mInstanceDescs, rayCount, perMeshHitEntry);

        D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS inputs = {};
        inputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
        inputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
        inputs.NumDescs = (uint32_t)mInstanceDescs.size();
        inputs.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_NONE;

        // Add build flags for dynamic scenes if TLAS should be updating instead of rebuilt
        if (mpAnimationController->hasAnimations() && mTlasUpdateMode == UpdateMode::Refit)
        {
            inputs.Flags |= D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_ALLOW_UPDATE;

            // If TLAS has been built already and it was built with ALLOW_UPDATE
            if (tlas.pTlas != nullptr && tlas.updateMode == UpdateMode::Refit) inputs.Flags |= D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PERFORM_UPDATE;
        }

        tlas.updateMode = mTlasUpdateMode;

        // On first build for the scene, create scratch buffer and cache prebuild info. As long as INSTANCE_DESC count doesn't change, we can reuse these
        if (mpTlasScratch == nullptr)
        {
            // Prebuild
            GET_COM_INTERFACE(gpDevice->getApiHandle(), ID3D12Device5, pDevice5);
            pDevice5->GetRaytracingAccelerationStructurePrebuildInfo(&inputs, &mTlasPrebuildInfo);
            mpTlasScratch = Buffer::create(mTlasPrebuildInfo.ScratchDataSizeInBytes, Buffer::BindFlags::UnorderedAccess, Buffer::CpuAccess::None);
            mpTlasScratch->setName("Scene::mpTlasScratch");

            // #SCENE This isn't guaranteed according to the spec, and the scratch buffer being stored should be sized differently depending on update mode
            assert(mTlasPrebuildInfo.UpdateScratchDataSizeInBytes <= mTlasPrebuildInfo.ScratchDataSizeInBytes);
        }

        // Setup GPU buffers
        D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC asDesc = {};
        asDesc.Inputs = inputs;

        // If first time building this TLAS
        if (tlas.pTlas == nullptr)
        {
            assert(tlas.pInstanceDescs == nullptr); // Instance desc should also be null if no TLAS
            tlas.pTlas = Buffer::create(mTlasPrebuildInfo.ResultDataMaxSizeInBytes, Buffer::BindFlags::AccelerationStructure, Buffer::CpuAccess::None);
            tlas.pInstanceDescs = Buffer::create((uint32_t)mInstanceDescs.size() * sizeof(D3D12_RAYTRACING_INSTANCE_DESC), Buffer::BindFlags::None, Buffer::CpuAccess::Write, mInstanceDescs.data());
        }
        // Else update instance descs and barrier TLAS buffers
        else
        {
            assert(mpAnimationController->hasAnimations());
            pContext->uavBarrier(tlas.pTlas.get());
            pContext->uavBarrier(mpTlasScratch.get());
            tlas.pInstanceDescs->setBlob(mInstanceDescs.data(), 0, inputs.NumDescs * sizeof(D3D12_RAYTRACING_INSTANCE_DESC));
            asDesc.SourceAccelerationStructureData = tlas.pTlas->getGpuAddress(); // Perform the update in-place
        }

        assert((inputs.NumDescs != 0) && tlas.pInstanceDescs->getApiHandle() && tlas.pTlas->getApiHandle() && mpTlasScratch->getApiHandle());

        asDesc.Inputs.InstanceDescs = tlas.pInstanceDescs->getGpuAddress();
        asDesc.ScratchAccelerationStructureData = mpTlasScratch->getGpuAddress();
        asDesc.DestAccelerationStructureData = tlas.pTlas->getGpuAddress();

        // Set the source buffer to update in place if this is an update
        if ((inputs.Flags & D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PERFORM_UPDATE) > 0) asDesc.SourceAccelerationStructureData = asDesc.DestAccelerationStructureData;

        // Create TLAS
        GET_COM_INTERFACE(pContext->getLowLevelData()->getCommandList(), ID3D12GraphicsCommandList4, pList4);
        pContext->resourceBarrier(tlas.pInstanceDescs.get(), Resource::State::NonPixelShader);
        pList4->BuildRaytracingAccelerationStructure(&asDesc, 0, nullptr);
        pContext->uavBarrier(tlas.pTlas.get());

        // Create TLAS SRV
        if (tlas.pSrv == nullptr)
        {
            D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
            srvDesc.ViewDimension = D3D12_SRV_DIMENSION_RAYTRACING_ACCELERATION_STRUCTURE;
            srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
            srvDesc.RaytracingAccelerationStructure.Location = tlas.pTlas->getGpuAddress();

            DescriptorSet::Layout layout;
            layout.addRange(DescriptorSet::Type::TextureSrv, 0, 1);
            DescriptorSet::SharedPtr pSet = DescriptorSet::create(gpDevice->getCpuDescriptorPool(), layout);
            gpDevice->getApiHandle()->CreateShaderResourceView(nullptr, &srvDesc, pSet->getCpuHandle(0));

            ResourceWeakPtr pWeak = tlas.pTlas;
            tlas.pSrv = std::make_shared<ShaderResourceView>(pWeak, pSet, 0, 1, 0, 1);
        }

        mTlasCache[rayCount] = tlas;
    }

    void Scene::setGeometryIndexIntoRtVars(const std::shared_ptr<RtProgramVars>& pVars)
    {
        // Sets the 'geometryIndex' hit shader variable for each mesh.
        // This is the local index of which mesh in the BLAS was hit.
        // In DXR 1.0 we have to pass it via a constant buffer to the shader,
        // in DXR 1.1 it is available through the GeometryIndex() system value.
        //
        assert(!mBlasData.empty());
        uint32_t meshCount = getMeshCount();
        uint32_t descHitCount = pVars->getDescHitGroupCount();

        uint32_t blasIndex = 0;
        uint32_t geometryIndex = 0;
        for (uint32_t meshId = 0; meshId < meshCount; meshId++)
        {
            for (uint32_t hit = 0; hit < descHitCount; hit++)
            {
                auto pHitVars = pVars->getHitVars(hit, meshId);
                auto var = pHitVars->findMember(0).findMember("geometryIndex");
                if (var.isValid())
                {
                    var = geometryIndex;
                }
            }

            geometryIndex++;

            // If at the end of this BLAS, reset counters and start checking next BLAS
            uint32_t geomCount = (uint32_t)mMeshGroups[blasIndex].meshList.size();
            if (geometryIndex == geomCount)
            {
                geometryIndex = 0;
                blasIndex++;
            }
        }

        // we only have one geometry under one blas for particle systems
        uint32_t particleSystemCount = getParticleSystemCount();
        uint32_t descParticleHitCount = pVars->getDescParticleHitGroupCount();

        for (uint32_t particleSystemId = 0; particleSystemId < particleSystemCount; particleSystemId++)
        {
            for (uint32_t hit = 0; hit < descParticleHitCount; hit++)
            {
                auto pHitVars = pVars->getParticleHitVars(hit, particleSystemId);
                auto var = pHitVars->findMember(0).findMember("geometryIndex");
                if (var.isValid())
                {
                    var = 0;
                }
            }
        }


#ifdef ONE_CURVE_PER_BLAS
        // each curve takes one blas
        // we only have one geometry under one blas for curves
        uint32_t curveCount = getCurveCount();
        uint32_t descCurveHitCount = pVars->getDescCurveHitGroupCount();

        for (uint32_t curveId = 0; curveId < curveCount; curveId++)
        {
            for (uint32_t hit = 0; hit < descCurveHitCount; hit++)
            {
                auto pHitVars = pVars->getCurveHitVars(hit, curveId);
                auto var = pHitVars->findMember(0).findMember("geometryIndex");
                if (var.isValid())
                {
                    var = 0;
                }
            }
        }
#else
        // all curves in one blass
        auto curveCount = getCurveCount();
        uint32_t descCurveHitCount = pVars->getDescCurveHitGroupCount();
        uint32_t curveBlasOffset = (uint32_t)(mMeshGroups.size() + mParticleSystemDesc.size());
        uint32_t curveBlasIndex = 0;
        uint32_t curveInBlasIndex = 0;
        for (uint32_t curveId = 0; curveId < curveCount; curveId++)
        {
            for (uint32_t hit = 0; hit < descCurveHitCount; hit++)
            {
                auto pHitVars = pVars->getCurveHitVars(hit, curveId);
                auto var = pHitVars->findMember(0).findMember("geometryIndex");
                if (var.isValid())
                {
                    var = curveInBlasIndex;
                }
            }

            curveInBlasIndex++;

            // If at the end of this BLAS, reset counters and start checking next BLAS
            uint32_t curveCountInBlas = (uint32_t)mBlasData[curveBlasOffset + curveBlasIndex].geomDescs.size();
            if (curveInBlasIndex == curveCountInBlas)
            {
                curveInBlasIndex = 0;
                curveBlasIndex++;
            }
        }
#endif
    }

    void Scene::setRaytracingShaderData(RenderContext* pContext, const ShaderVar& var, uint32_t rayTypeCount)
    {
        // On first execution, create BLAS for each mesh.
        if (mBlasData.empty())
        {
            initGeomDesc();
            buildBlas(pContext);
        }

        // On first execution, when meshes have moved, when there's a new ray count, or when a BLAS has changed, create/update the TLAS
        //
        // TODO: The notion of "ray count" is being treated as fundamental here, and intrinsically
        // linked to the number of hit groups in the program, without checking if this matches
        // other things like the number of miss shaders. If/when we support meshes with custom
        // intersection shaders, then the assumption that number of ray types and number of
        // hit groups match will be incorrect.
        //
        // It really seems like a first-class notion of ray types (and the number thereof) is required.
        //
        auto tlasIt = mTlasCache.find(rayTypeCount);
        if (tlasIt == mTlasCache.end())
        {
            // We need a hit entry per mesh right now to pass GeometryIndex()
            buildTlas(pContext, rayTypeCount, true);

            // If new TLAS was just created, get it so the iterator is valid
            if (tlasIt == mTlasCache.end()) tlasIt = mTlasCache.find(rayTypeCount);
        }

        assert(mpSceneBlock);
        assert(tlasIt->second.pSrv);

        // Bind Scene parameter block.
        getCamera()->setShaderData(mpSceneBlock[kCamera]);
        var["gScene"] = mpSceneBlock;

        // Bind TLAS.
        var["gRtScene"].setSrv(tlasIt->second.pSrv);
    }


    void Scene::setRaytracingAcceleraitonStructure(RenderContext* pContext, const ShaderVar& var)
    {
        // On first execution, create BLAS for each mesh.
        if (mBlasData.empty())
        {
            initGeomDesc();
            buildBlas(pContext);
        }

        // On first execution, when meshes have moved, when there's a new ray count, or when a BLAS has changed, create/update the TLAS
        //
        // TODO: The notion of "ray count" is being treated as fundamental here, and intrinsically
        // linked to the number of hit groups in the program, without checking if this matches
        // other things like the number of miss shaders. If/when we support meshes with custom
        // intersection shaders, then the assumption that number of ray types and number of
        // hit groups match will be incorrect.
        //
        // It really seems like a first-class notion of ray types (and the number thereof) is required.
        //

        auto firstElement = mTlasCache.begin();
        uint32_t rayTypeCount = firstElement == mTlasCache.end() ? 0 : firstElement->first;

        auto tlasIt = mTlasCache.find(rayTypeCount);
        if (tlasIt == mTlasCache.end())
        {
            // We need a hit entry per mesh right now to pass GeometryIndex()
            buildTlas(pContext, rayTypeCount, true);

            // If new TLAS was just created, get it so the iterator is valid
            if (tlasIt == mTlasCache.end()) tlasIt = mTlasCache.find(rayTypeCount);
        }

        assert(tlasIt->second.pSrv);

        // Bind TLAS.
        var.setSrv(tlasIt->second.pSrv);
    }

    void Scene::setEnvMap(EnvMap::SharedPtr pEnvMap)
    {
        if (mpEnvMap == pEnvMap) return;
        mpEnvMap = pEnvMap;
        if (mpEnvMap) mpEnvMap->setShaderData(mpSceneBlock[kEnvMap]);
    }

    void Scene::loadEnvMap(const std::string& filename)
    {
        EnvMap::SharedPtr pEnvMap = EnvMap::create(filename);
        setEnvMap(pEnvMap);
    }

    void Scene::setEnvMapIntensity(float intensity)
    {
        mpEnvMap->setIntensity(intensity);
    }

    void Scene::setEmissiveIntensityMultiplier(float multiplier)
    {
        mEmissiveIntensityMultiplier = multiplier;
        mpSceneBlock["emissiveIntensityMultiplier"] = mEmissiveIntensityMultiplier;
    }

    void Scene::setEnvMapRotation(const float3& rotDegrees)
    {
        mpEnvMap->setRotation(rotDegrees);
    }

    void Scene::calculateCurvePatchBoundingBoxes()
    {
        // Calculate curve patch bounding boxes
        for (uint32_t curveId = 0; curveId < (uint32_t)mCurveDesc.size(); curveId++)
        {
            const auto& curve = mCurveDesc[curveId];

            int numPatches = mCurveDesc[curveId].getPatchCount();
            int numVerticesPerPatch = mCurveDesc[curveId].getVerticesPerPatch();

            for (int patch = 0; patch < numPatches; patch++)
            {
                float3 boxMin(FLT_MAX);
                float3 boxMax(-FLT_MAX);
                float maxWidth = 0;
                for (int v = 0; v < numVerticesPerPatch; v++)
                {
                    boxMin = glm::min(boxMin, float3(mCPUCurveVertexBuffer[curve.vbOffset + numVerticesPerPatch * patch + v].position));
                    boxMax = glm::max(boxMax, float3(mCPUCurveVertexBuffer[curve.vbOffset + numVerticesPerPatch * patch + v].position));
                    maxWidth = glm::max(maxWidth, mCPUCurveVertexBuffer[curve.vbOffset + numVerticesPerPatch * patch + v].position.w);
                }
                maxWidth *= mCurveDisplayWidthMultiplier;
                // extend with width
                BoundingBox aabb = BoundingBox::fromMinMax(boxMin - 0.5f * maxWidth, boxMax + 0.5f * maxWidth);
                mCurvePatchBBs.push_back(aabb);
            }
        }
    }

    void Scene::updateCurveDisplayWidth(float displayWidth)
    {
        // Calculate curve patch bounding boxes
        mCurveDisplayWidthMultiplier = displayWidth;
        int count = 0;
        for (uint32_t curveId = 0; curveId < (uint32_t)mCurveDesc.size(); curveId++)
        {
            const auto& curve = mCurveDesc[curveId];
            int numPatches = curve.getPatchCount();
            int numVerticesPerPatch = curve.getVerticesPerPatch();
            for (int patch = 0; patch < numPatches; patch++)
            {
                float3 boxMin(FLT_MAX);
                float3 boxMax(-FLT_MAX);
                float maxWidth = 0;
                for (int v = 0; v < numVerticesPerPatch; v++)
                {
                    boxMin = glm::min(boxMin, float3(mCPUCurveVertexBuffer[curve.vbOffset + numVerticesPerPatch * patch + v].position));
                    boxMax = glm::max(boxMax, float3(mCPUCurveVertexBuffer[curve.vbOffset + numVerticesPerPatch * patch + v].position));
                    maxWidth = glm::max(maxWidth, mCPUCurveVertexBuffer[curve.vbOffset + numVerticesPerPatch * patch + v].position.w);
                }
                maxWidth *= mCurveDisplayWidthMultiplier;
                // extend with width
                mCurvePatchBBs[count++] = BoundingBox::fromMinMax(boxMin - 0.5f * maxWidth, boxMax + 0.5f * maxWidth);
            }
        }
        updateBounds();

        int globalPatchId = 0;
        std::vector<D3D12_RAYTRACING_AABB> aabbs;
        for (uint32_t curveId = 0; curveId < (uint32_t)mCurveDesc.size(); curveId++)
        {
            int numPatches = mCurveDesc[curveId].getPatchCount();
            // fill CPU AABB array
            for (uint32_t patchId = 0; patchId < (uint32_t)numPatches; patchId++)
            {
                D3D12_RAYTRACING_AABB aabb;
                aabb.MinX = mCurvePatchBBs[globalPatchId].getMinPos().x;
                aabb.MinY = mCurvePatchBBs[globalPatchId].getMinPos().y;
                aabb.MinZ = mCurvePatchBBs[globalPatchId].getMinPos().z;
                aabb.MaxX = mCurvePatchBBs[globalPatchId].getMaxPos().x;
                aabb.MaxY = mCurvePatchBBs[globalPatchId].getMaxPos().y;
                aabb.MaxZ = mCurvePatchBBs[globalPatchId].getMaxPos().z;
                aabbs.push_back(aabb);
                globalPatchId++;
            }
        }

        mpCurvePatchAABBBuffer->setBlob(aabbs.data(), 0, sizeof(D3D12_RAYTRACING_AABB) * aabbs.size());
        mHasCurveDisplayWidthChanged = true;
    }

    uint32_t Scene::addLight(const Light::SharedPtr& pLight)
    {
        assert(pLight);
        mLights.push_back(pLight);
        mUpdates |= UpdateFlags::LightCollectionChanged;
        assert(mLights.size() <= std::numeric_limits<uint32_t>::max());
        mpLightsBuffer = Buffer::createStructured(mpSceneBlock[kLightsBufferName], (uint32_t)mLights.size(), Resource::BindFlags::ShaderResource, Buffer::CpuAccess::None, nullptr, false);
        mpLightsBuffer->setName("Scene::mpLightsBuffer");
        mpSceneBlock->setBuffer(kLightsBufferName, mpLightsBuffer);
        return (uint32_t)mLights.size() - 1;
    }


    uint32_t Scene::addDirectionalLight(float3 worldDirection, float3 intensity)
    {
        DirectionalLight::SharedPtr dirLight = DirectionalLight::create();
        dirLight->setWorldDirection(worldDirection);
        dirLight->setIntensity(intensity);
        return addLight(dirLight);
    }


    uint32_t Scene::addPointLight(float3 worldPosition, float3 worldDirection, float openingAngle, float3 intensity)
    {
        PointLight::SharedPtr pointLight = PointLight::create();
        pointLight->setWorldPosition(worldPosition);
        pointLight->setWorldDirection(worldDirection);
        pointLight->setIntensity(intensity);
        return addLight(pointLight);
    }

    void Scene::setVolumeShaderData(ShaderVar const& var, int volumeId /*= -1*/)
    {
        setGVDBShaderData(var, volumeId);
    }

    void Scene::setCameraAspectRatio(float ratio)
    {
        getCamera()->setAspectRatio(ratio);
    }

    void Scene::bindSamplerToMaterials(const Sampler::SharedPtr& pSampler)
    {
        for (auto& pMaterial : mMaterials)
        {
            pMaterial->setSampler(pSampler);
        }
    }

    void Scene::setCameraController(CameraControllerType type)
    {
        if (!mCameraSwitched && mCamCtrlType == type && mpCamCtrl) return;

        auto camera = getCamera();
        switch (type)
        {
        case CameraControllerType::FirstPerson:
            mpCamCtrl = FirstPersonCameraController::create(camera);
            break;
        case CameraControllerType::Orbiter:
            mpCamCtrl = OrbiterCameraController::create(camera);
            ((OrbiterCameraController*)mpCamCtrl.get())->setModelParams(mSceneBB.center, length(mSceneBB.extent), 3.5f);
            break;
        case CameraControllerType::SixDOF:
            mpCamCtrl = SixDoFCameraController::create(camera);
            break;
        default:
            should_not_get_here();
        }
        mpCamCtrl->setCameraSpeed(mCameraSpeed);
    }

    bool Scene::onMouseEvent(const MouseEvent& mouseEvent)
    {
        if (mFreezeCamera) return false;
        return mpCamCtrl->onMouseEvent(mouseEvent);
    }

    bool Scene::onKeyEvent(const KeyboardEvent& keyEvent)
    {
        if (keyEvent.type == KeyboardEvent::Type::KeyPressed)
        {
            if (!(keyEvent.mods.isAltDown || keyEvent.mods.isCtrlDown || keyEvent.mods.isShiftDown))
            {
                if (keyEvent.key == KeyboardEvent::Key::F3)
                {
                    addViewpoint();
                    return true;
                }
            }
        }
        return mpCamCtrl->onKeyEvent(keyEvent);
    }

    std::string Scene::getScript(const std::string& sceneVar)
    {
        std::string c;

        // Render settings.
        c += Scripting::makeSetProperty(sceneVar, kRenderSettings, mRenderSettings);

        // Animations.
        if (hasAnimation() && !isAnimated())
        {
            c += Scripting::makeSetProperty(sceneVar, kAnimated, false);
        }
        for (size_t i = 0; i < mLights.size(); ++i)
        {
            const auto& light = mLights[i];
            if (light->hasAnimation() && !light->isAnimated())
            {
                c += Scripting::makeSetProperty(sceneVar + "." + kGetLight + "(" + std::to_string(i) + ").", kAnimated, false);
            }
        }

        // Camera.
        if (mSelectedCamera != 0)
        {
            c += sceneVar + "." + kCamera + " = " + sceneVar + "." + kCameras + "[" + std::to_string(mSelectedCamera) + "]\n";
        }
        c += getCamera()->getScript(sceneVar + "." + kCamera);

        // Camera speed.
        c += Scripting::makeSetProperty(sceneVar, kCameraSpeed, mCameraSpeed);

        // Viewpoints.
        if (hasSavedViewpoints())
        {
            for (size_t i = 1; i < mViewpoints.size(); i++)
            {
                auto v = mViewpoints[i];
                c += Scripting::makeMemberFunc(sceneVar, kAddViewpoint, v.position, v.target, v.up, v.index);
            }
        }

        // particle systems
        for (size_t i = 0; i < mParticleSystems.size(); i++)
        {
            auto pSys = mParticleSystems[i];
            c += std::string("m.addParticleSystem(") +
                "maxParticles=" + std::to_string(pSys->getMaxParticles()) +
                ",maxEmitPerFrame=" + std::to_string(pSys->getMaxEmitParticles()) +
                ",useFixedInterval=" + (pSys->mUseFixedInterval ? std::string("True") : std::string("False")) +
                ",fixedInterval=" + std::to_string(pSys->mFixedInterval) +
                ",maxRenderFrames=" + std::to_string(pSys->maxRenderFrames) +
                ",shouldSort=" + (pSys->isParticleSorted() ? std::string("True") : std::string("False")) +
                ",duration=" + std::to_string(pSys->mEmitter.duration) +
                ",durationOffset=" + std::to_string(pSys->mEmitter.durationOffset) +
                ",emitFrequency=" + std::to_string(pSys->mEmitter.emitFrequency) +
                ",emitCount=" + std::to_string(pSys->mEmitter.emitCount) +
                ",emitCountOffset=" + std::to_string(pSys->mEmitter.emitCountOffset) +
                ",spawnPos=float3(" + std::to_string(pSys->mEmitter.spawnPos.x) + "," + std::to_string(pSys->mEmitter.spawnPos.y) + "," + std::to_string(pSys->mEmitter.spawnPos.z) + ")" +
                ",spawnPosOffset=float3(" + std::to_string(pSys->mEmitter.spawnPosOffset.x) + "," + std::to_string(pSys->mEmitter.spawnPosOffset.y) + "," + std::to_string(pSys->mEmitter.spawnPosOffset.z) + ")" +
                ",vel=float3(" + std::to_string(pSys->mEmitter.vel.x) + "," + std::to_string(pSys->mEmitter.vel.y) + "," + std::to_string(pSys->mEmitter.vel.x) + ")" +
                ",velOffset=float3(" + std::to_string(pSys->mEmitter.velOffset.x) + "," + std::to_string(pSys->mEmitter.velOffset.y) + "," + std::to_string(pSys->mEmitter.velOffset.x) + ")" +
                ",scale=" + std::to_string(pSys->mEmitter.scale) +
                ",scaleOffset=" + std::to_string(pSys->mEmitter.scaleOffset) +
                ",growth=" + std::to_string(pSys->mEmitter.growth) +
                ",growthOffset=" + std::to_string(pSys->mEmitter.growthOffset) +
                ",billboardRotation=" + std::to_string(pSys->mEmitter.billboardRotation) +
                ",billboardRotationOffset=" + std::to_string(pSys->mEmitter.billboardRotationOffset) +
                ",billboardRotationVel=" + std::to_string(pSys->mEmitter.billboardRotationVel) +
                ",billboardRotationVelOffset=" + std::to_string(pSys->mEmitter.billboardRotationVelOffset) +
                ",shadingType=" + std::to_string((uint32_t)mParticleSystemManager.mPsData[i].type) +
                ",startColor=float4(" + std::to_string(mParticleSystemManager.mPsData[i].colorData.color1.x) + "," + std::to_string(mParticleSystemManager.mPsData[i].colorData.color1.y) + ","
                + std::to_string(mParticleSystemManager.mPsData[i].colorData.color1.z) + "," + std::to_string(mParticleSystemManager.mPsData[i].colorData.color1.w) + ")" +
                ",endColor=float4(" + std::to_string(mParticleSystemManager.mPsData[i].colorData.color2.x) + "," + std::to_string(mParticleSystemManager.mPsData[i].colorData.color2.y) + ","
                + std::to_string(mParticleSystemManager.mPsData[i].colorData.color2.z) + "," + std::to_string(mParticleSystemManager.mPsData[i].colorData.color2.w) + ")" +
                ",startT=" + std::to_string(mParticleSystemManager.mPsData[i].colorData.colorT1) +
                ",endT=" + std::to_string(mParticleSystemManager.mPsData[i].colorData.colorT2) +
                ",textureFile=\"" + mParticleSystemManager.mpTextures[mParticleSystemManager.mPsData[i].texIndex]->getSourceFilename()
                + "\")\n";
        }

        return c;
    }

    uint32_t Scene::addSimpleCurveModel(std::string filename, float width, float3 color)
    {
        Material::SharedPtr pMaterial = Material::create("defaultCurveMaterial");
        pMaterial->setShadingModel(2); // specular gloss
        pMaterial->setBaseColor(float4(color, 1));
        mMaterials.push_back(pMaterial);

        if (getExtensionFromFile(filename) == "ccp")
        {
            mCurveFileLoader.loadKnitCCPFile(this->shared_from_this(), filename, (int)mMaterials.size() - 1, width);
        }
        else if (getExtensionFromFile(filename) == "hair")
        {
            mCurveFileLoader.loadHairFile(this->shared_from_this(), filename, (int)mMaterials.size() - 1, width);
        }
        else
        {
            std::cout << "unsupported file type for curves!" << std::endl;
            exit(1);
        }
        createCurveVao();
        calculateCurvePatchBoundingBoxes();
        finalize(true);
        mNewParticleSystemAdded = true;
        return (uint32_t)mCurveDesc.size() - 1;
    }

    void Scene::GVDBInfo::bindParameterBlock(ParameterBlock::SharedPtr block, int mipId)
    {
        for (int lev = 0; lev <= top_lev[mipId]; lev++)
        {
            block["dim"][lev + mipId * MAX_LEVELS] = dim[lev + mipId * MAX_LEVELS];
            block["res"][lev + mipId * MAX_LEVELS] = res[lev + mipId * MAX_LEVELS];
            block["vdel"][lev + mipId * MAX_LEVELS] = vdel[lev + mipId * MAX_LEVELS];
            block["noderange"][lev + mipId * MAX_LEVELS] = noderange[lev + mipId * MAX_LEVELS];
            block["nodecnt"][lev + mipId * MAX_LEVELS] = nodecnt[lev + mipId * MAX_LEVELS];
            block["nodewid"][lev + mipId * MAX_LEVELS] = nodewid[lev + mipId * MAX_LEVELS];
            block["childwid"][lev + mipId * MAX_LEVELS] = childwid[lev + mipId * MAX_LEVELS];
            block["nodelist"][lev + mipId * MAX_LEVELS] = nodelist[lev + mipId * MAX_LEVELS];
            if (lev > 0)
                block["childlist"][lev + mipId * MAX_LEVELS] = childlist[lev + mipId * MAX_LEVELS];
        }
        block["top_lev"][mipId] = top_lev[mipId];
        if (mipId == 0)
        {
            block["max_iter"] = max_iter;
            block["epsilon"] = epsilon;
            block["update"] = update;
            block["clr_chan"] = clr_chan;
            block["superVoxelWorldSpaceDiagonalLength"] = superVoxelWorldSpaceDiagonalLength;
        }
        block["bmin"][mipId] = bmin[mipId];
        block["bmax"][mipId] = bmax[mipId];
        if (mipId < 2 * kNumMaxMips + 1)
        {
            block["volIn"][mipId] = volIn[mipId];
            block["volIn_part2"][mipId] = volIn_part2[mipId];
        }
        else if (mipId == 2 * kNumMaxMips + 1)
        {
            int texId = 0;
            block["velocityIn"][texId] = velocityIn[0];
            block["velocityIn_part2"][texId] = velocityIn_part2[0];
        }

        block["volInDimensions"][mipId] = volInDimensions[mipId];
        block["volInDimensions_part2"][mipId] = volInDimensions_part2[mipId];
        block["invVolInDimensions"][mipId] = float3(1.f) / float3(volInDimensions[mipId]);
        block["invVolInDimensions_part2"][mipId] = float3(1.f) / float3(volInDimensions_part2[mipId]);
        block["xform"][mipId] = xform[mipId];
        block["invxform"][mipId] = invxform[mipId];
        block["invxrot"][mipId] = invxrot[mipId];
        block["maxValue"][mipId] = maxValue[mipId];
        block["invMaxValue"][mipId] = invMaxValue[mipId];
        block["densityCompressScaleFactor"][mipId] = densityCompressScaleFactor[mipId];
    }

    void Scene::GVDBInfo::bindPrevParameterBlock(ParameterBlock::SharedPtr block, int mipId)
    {
        int storedMipId = mipId >= 2 * kNumMaxMips ? mipId + kPrevFrameExtraGridOffset : mipId + kPrevFrameDensityGridOffset;
        for (int lev = 0; lev <= top_lev[mipId]; lev++)
        {
            block["dim"][lev + storedMipId * MAX_LEVELS] = dim[lev + mipId * MAX_LEVELS];
            block["res"][lev + storedMipId * MAX_LEVELS] = res[lev + mipId * MAX_LEVELS];
            block["vdel"][lev + storedMipId * MAX_LEVELS] = vdel[lev + mipId * MAX_LEVELS];
            block["noderange"][lev + storedMipId * MAX_LEVELS] = noderange[lev + mipId * MAX_LEVELS];
            block["nodecnt"][lev + storedMipId * MAX_LEVELS] = nodecnt[lev + mipId * MAX_LEVELS];
            block["nodewid"][lev + storedMipId * MAX_LEVELS] = nodewid[lev + mipId * MAX_LEVELS];
            block["childwid"][lev + storedMipId * MAX_LEVELS] = childwid[lev + mipId * MAX_LEVELS];
            block["nodelist"][lev + storedMipId * MAX_LEVELS] = nodelist[lev + mipId * MAX_LEVELS];
            if (lev > 0)
                block["childlist"][lev + storedMipId * MAX_LEVELS] = childlist[lev + mipId * MAX_LEVELS];
        }
        block["top_lev"][storedMipId] = top_lev[mipId];
        block["bmin"][storedMipId] = bmin[mipId];
        block["bmax"][storedMipId] = bmax[mipId];

        if (mipId < 2 * kNumMaxMips + 1)
        {
            block["volIn"][storedMipId] = volIn[mipId];
            block["volIn_part2"][storedMipId] = volIn_part2[mipId];
        }
        else if (mipId == 2 * kNumMaxMips + 1)
        {
            int texId = 1;
            block["velocityIn"][texId] = velocityIn[0];
            block["velocityIn_part2"][texId] = velocityIn_part2[0];
        }

        block["volInDimensions"][storedMipId] = volInDimensions[mipId];
        block["volInDimensions_part2"][storedMipId] = volInDimensions_part2[mipId];
        block["invVolInDimensions"][storedMipId] = float3(1.f) / float3(volInDimensions[mipId]);
        block["invVolInDimensions_part2"][storedMipId] = float3(1.f) / float3(volInDimensions_part2[mipId]);
        block["xform"][storedMipId] = xform[mipId];
        block["invxform"][storedMipId] = invxform[mipId];
        block["invxrot"][storedMipId] = invxrot[mipId];
        block["maxValue"][storedMipId] = maxValue[mipId];
        block["invMaxValue"][storedMipId] = invMaxValue[mipId];
        block["densityCompressScaleFactor"][storedMipId] = densityCompressScaleFactor[mipId];
    }


    uint32_t Scene::addGVDBVolume(int curFrameId, float3 sigma_a, float3 sigma_s, float g, std::string vbxFile, int numMips, float DensityScale, bool hasVelocityGrid, bool hasEmissionGrid, float LeScale, float temperatureCutOff, float temperatureScale, float3 worldTranslation, float3 worldRotation, float worldScaling)
    {
        if (curFrameId <= 0) // non-animated uses curFrameId == -1
        {
            mVolumeWorldTranslation = worldTranslation;
            mVolumeWorldScaling = worldScaling;
            mVolumeWorldRotation = worldRotation;
        }

        ComputeProgram::SharedPtr pProgram = ComputeProgram::createFromFile("Scene/GVDBParameterBlock.slang", "main");
        ParameterBlockReflection::SharedConstPtr pReflection = pProgram->getReflector()->getParameterBlock("gVDBInfo");
        assert(pReflection);

        GVDBParamBlocks gvdbParamBlocks;

        std::string filenameWithoutPath = vbxFile;
        // Remove directory if present.
        // Do this before extension removal incase directory has a period character.
        const size_t last_slash_idx = filenameWithoutPath.find_last_of("\\/");
        if (std::string::npos != last_slash_idx)
        {
            filenameWithoutPath.erase(0, last_slash_idx + 1);
        }

        int mipId = 0;

        ParameterBlock::SharedPtr gvdbBlock = ParameterBlock::create(pReflection);

        GVDBInfo gvdbInfo;

        assert(numMips <= kNumMaxMips);

        gvdbParamBlocks.numMips = numMips;
        gvdbParamBlocks.hasEmissionGrid = false;

        std::string temperatureFilename = vbxFile + "/" + filenameWithoutPath + "_temperature.vbx";
        std::string temperaturefullpath;
        if (!findFileInDataDirectories(temperatureFilename, temperaturefullpath)) hasEmissionGrid = false;

        bool noConservativeGrid = false;

        for (mipId = 0; mipId < numMips; mipId++)
        {
            std::string filename = vbxFile;

            int numTypes = mipId >= 2 * kNumMaxMips ? 1 : 2;

            for (int typeId = 0; typeId < numTypes; typeId++)
            {
                if (numMips > 1 && mipId < 2 * kNumMaxMips)
                {
                    // vbxfile is treated as folder name and the filename prefix (before _mipX.vbx)
                    filename = vbxFile + "/" + filenameWithoutPath + "_mip" + std::to_string(mipId) + (typeId == 0 ? "" : "c") + ".vbx";
                }
                else if (mipId == 2 * kNumMaxMips) //hasEmissionGrid
                {
                    filename = vbxFile + "/" + filenameWithoutPath + "_temperature.vbx";
                }
                else if (mipId == 2 * kNumMaxMips + 1)
                {
                    filename = vbxFile + "/" + filenameWithoutPath + "_velocity_x.vbx";
                }
                else if (mipId == 2 * kNumMaxMips + 2)
                {
                    std::cout << "Error! Supervoxel already disabled!" << std::endl;
                    exit(1);
                }

                std::string fullpath;
                if (!findFileInDataDirectories(filename, fullpath))
                {
                    if (mipId == 2 * kNumMaxMips) {
                        gvdbParamBlocks.hasEmissionGrid = false;
                        continue;
                    }
                    else if (mipId == 2 * kNumMaxMips + 1) {
                        gvdbParamBlocks.hasVelocityGrid = false;
                        break;
                    }
                    else
                    {
                        bool shouldSkip = false;
                        if (typeId == 1)
                        {
                            std::cout << "Warning: No conservative grid detected!\n" << std::endl;
                            numTypes = 1;
                            noConservativeGrid = true;

                            if (mipId == numMips - 1)
                            {
                                shouldSkip = true;
                            }
                        }
                        else
                        {
                            gvdbParamBlocks.numMips = mipId;
                            shouldSkip = true;
                        }

                        if (shouldSkip)
                        {
                            if (hasEmissionGrid)
                            {
                                numMips = 2 * kNumMaxMips + 1;
                                mipId = 2 * kNumMaxMips - 1; // will be increment to 8 shortly
                            }
                            else if (hasVelocityGrid)
                            {
                                numMips = 2 * kNumMaxMips + 2;
                                mipId = 2 * kNumMaxMips;
                            }
                        }
                        continue;
                    }
                }

                if (mipId == 0)
                    std::cout << filename << std::endl;

                if (mipId == 2 * kNumMaxMips)
                {
                    gvdbParamBlocks.hasEmissionGrid = true;

                    if (!mpBlackBodyRadiationTexture)
                    {
                        mCPUBlackBodyRadiationTexture.clear();
                        // load BlackBodyRadiationTexture
                        std::string fullpath = "";
                        findFileInDataDirectories("LUT/BlackBodyRadiationRGB_50K-6400K.txt", fullpath);
                        std::ifstream f(fullpath);

                        for (int i = 0; i < 128; i++)
                        {
                            float r, g, b;
                            f >> r >> g >> b;
                            mCPUBlackBodyRadiationTexture.push_back(float4(r, g, b, 0));
                        }

                        mpBlackBodyRadiationTexture = Texture::create1D(128, ResourceFormat::RGBA32Float, 1, 1, mCPUBlackBodyRadiationTexture.data());
                    }
                }

                if (mipId == 2 * kNumMaxMips + 1) gvdbParamBlocks.hasVelocityGrid = true;

                VolumeGVDB	gvdb;
                gvdb.Initialize(true);
                gvdb.LoadVBX(fullpath, true);

                int slotId = mipId;
                if (typeId == 1) slotId += kNumMaxMips;

                // Send VDB info
                int levs = gvdb.mPool->getNumLevels();

                struct NewVDBNode
                {
                    int3 mPackedPosValue;
                    uint mChildList;
                    float4 mDensityBounds;
                };

                std::vector<std::vector<NewVDBNode>> newNodePool;

                for (int n = 0; n <= levs - 1; n++) {
                    //for (int n = levs - 1; n >= 0; n--) {
                    gvdbInfo.nodecnt[n + GVDBInfo::MAX_LEVELS * slotId] = static_cast<int>(gvdb.mPool->getPoolTotalCnt(0, n));
                    if (gvdbInfo.nodecnt[n + GVDBInfo::MAX_LEVELS * slotId] == 0) continue;
                    gvdbInfo.dim[n + GVDBInfo::MAX_LEVELS * slotId] = gvdb.getLD(n);
                    gvdbInfo.res[n + GVDBInfo::MAX_LEVELS * slotId] = gvdb.getRes(n);
                    nvdb::Vector3DI range = gvdb.getRange(n);
                    nvdb::Vector3DI res3DI = gvdb.getRes3DI(n);
                    gvdbInfo.vdel[n + GVDBInfo::MAX_LEVELS * slotId] = int3(range.x, range.y, range.z);
                    gvdbInfo.vdel[n + GVDBInfo::MAX_LEVELS * slotId] /= float3(res3DI.x, res3DI.y, res3DI.z);
                    gvdbInfo.noderange[n + GVDBInfo::MAX_LEVELS * slotId] = int3(range.x, range.y, range.z);		// integer (cannot send cover)
                    gvdbInfo.nodewid[n + GVDBInfo::MAX_LEVELS * slotId] = static_cast<int>(gvdb.mPool->getPoolWidth(0, n));
                    gvdbInfo.childwid[n + GVDBInfo::MAX_LEVELS * slotId] = static_cast<int>(gvdb.mPool->getPoolWidth(1, n));
                    assert(gvdb.mPool->getPoolWidth(0, n) % 64 == 0);

                    struct OldVDBNode
                    {
                        uint        mPackedLevFlagPriority; // check endianess
                        int3		mPos;			// Pos			Max = +/- 4 mil (linear space/range)	12 bytes
                        int3		mValue;			// Value		Max = +8 mil		4 bytes
                        float3		mVRange;
                        uint2		mParent;		// Parent ID						8 bytes
                        uint2		mChildList;		// Child List						8 bytes
                        uint2		mMask;			// Bitmask starts - Must keep here, even if not USE_BITMASKS
                    };

                    // simplify the node struct from 64 Bytes to 16 Bytes
                    assert(gvdbInfo.nodewid[n + GVDBInfo::MAX_LEVELS * slotId] == 64);
                    int oldPoolWidth = 64;
                    int newPoolWidth = 32;
                    int numElements = gvdb.mPool->getPoolTotalCnt(0, n);
                    char* nodePoolPtr = gvdb.mPool->getPoolCPU(0, n);

                    if (slotId == 0) mVolumeDesc.maxDensity = gvdb.mGridValMax;

                    newNodePool.push_back(std::vector<NewVDBNode>(numElements));

                    // cache the newNodePool
                    std::string nodeCacheFilename = getDirectoryFromFile(fullpath) + "\\" + swapFileExtension(getFilenameFromPath(fullpath), ".vbx", "") + "_level" + std::to_string(n) + "nodes.bin";

                    if (doesFileExist(nodeCacheFilename))
                    {
                        FILE* f = fopen(nodeCacheFilename.c_str(), "rb");
                        fread(newNodePool.back().data(), newPoolWidth, gvdb.mPool->getPoolTotalCnt(0, n), f);
                        fclose(f);
                    }
                    else
                    {
                        for (int i = 0; i < numElements; i++)
                        {
                            OldVDBNode oldnode;
                            NewVDBNode newnode;
                            memcpy(&oldnode, nodePoolPtr + oldPoolWidth * i, oldPoolWidth);

                            newnode.mPackedPosValue.x = (oldnode.mPos.x & 0xFFFF) | (oldnode.mPos.y << 16);
                            newnode.mPackedPosValue.y = (oldnode.mPos.z & 0xFFFF) | (oldnode.mValue.x << 16);
                            newnode.mPackedPosValue.z = (oldnode.mValue.y & 0xFFFF) | (oldnode.mValue.z << 16);

                            uint2 listid = oldnode.mChildList; // assuming little-endianess for uint64
                            if (listid.x == 0xFFFFFFFF)
                            {
                                newnode.mChildList = 0xFFFFFFFF;
                                if (true)
                                {
                                    int3 vMin = oldnode.mPos;

                                    float sum = 0;
                                    float sum2 = 0;

                                    float minDensity = FLT_MAX;
                                    float maxDensity = 0.f;

                                    for (int i = -1; i <= 8; i++)
                                        for (int j = -1; j <= 8; j++)
                                            for (int k = -1; k <= 8; k++)
                                            {
                                                float density = 0.f;
                                                int tcX, tcY, tcZ;
                                                if (vMin.x + i < gvdb.mEffectiveVoxMin.x || vMin.x + i > gvdb.mEffectiveVoxMax.x - 1 ||
                                                    vMin.y + j < gvdb.mEffectiveVoxMin.y || vMin.y + j > gvdb.mEffectiveVoxMax.y - 1 ||
                                                    vMin.z + k < gvdb.mEffectiveVoxMin.z || vMin.z + k > gvdb.mEffectiveVoxMax.z - 1)
                                                {
                                                    density = 0.f;
                                                }
                                                else
                                                    density = gvdb.getValueWithTexCoordFromRoot(Vector3DF(vMin.x + i + 0.5, vMin.y + j + 0.5, vMin.z + k + 0.5), gvdb.mPool->getAtlasCPU(0), tcX, tcY, tcZ);

                                                minDensity = std::min(minDensity, density);
                                                maxDensity = std::max(maxDensity, density);
                                                sum += density;
                                                sum2 += density * density;
                                            }

                                    float avgDensity = sum / 512.f;

                                    newnode.mDensityBounds = float4(minDensity, maxDensity, avgDensity, 0.f);
                                }
                            }
                            else
                            {
                                uint clev = (listid.x >> 8) & 0xFF;
                                int cndx = ((listid.y & 0xFFFF) << 16) | ((listid.x >> 16) & 0xFFFF);
                                newnode.mChildList = cndx;
                            }

                            newNodePool.back()[i] = newnode;
                        }

                        FILE* nodeCacheFile = fopen(nodeCacheFilename.c_str(), "wb");
                        fwrite(newNodePool.back().data(), newPoolWidth, gvdb.mPool->getPoolTotalCnt(0, n), nodeCacheFile);
                        fclose(nodeCacheFile);
                    }

                    gvdbInfo.nodewid[n + GVDBInfo::MAX_LEVELS * slotId] = newPoolWidth;
                    gvdbInfo.nodelist[n + GVDBInfo::MAX_LEVELS * slotId] = Buffer::createStructured(newPoolWidth, gvdb.mPool->getPoolTotalCnt(0, n),
                        ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess, Buffer::CpuAccess::None, newNodePool.back().data(), false, 0, true);

                    gvdbInfo.nodelist[n + GVDBInfo::MAX_LEVELS * slotId]->setName("nodelist" + std::to_string(n));
                    assert(gvdb.mPool->getPoolWidth(0, n) * gvdb.mPool->getPoolTotalCnt(0, n) == gvdb.mPool->getPoolSize(0, n));
                    if (n > 0)
                    {
                        // simplify the child list element from 8 bytes to 4 bytes
                        int numElements = gvdb.mPool->getPoolSize(1, n) / 8;
                        char* childPoolPtr = gvdb.mPool->getPoolCPU(1, n);

                        std::vector<uint> newChildPool(numElements);
                        for (int i = 0; i < numElements; i++)
                        {
                            uint2 oldElement;
                            memcpy(&oldElement.x, childPoolPtr + 8 * i, 8);
                            uint c = ((oldElement.y & 0xFFFF) << 16) | ((oldElement.x >> 16) & 0xFFFF);
                            newChildPool[i] = c;
                        }
                        gvdbInfo.childwid[n + GVDBInfo::MAX_LEVELS * slotId] /= 2;
                        gvdbInfo.childlist[n + GVDBInfo::MAX_LEVELS * slotId] = Buffer::create(gvdb.mPool->getPoolSize(1, n) / 2,
                            ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess, Buffer::CpuAccess::None, newChildPool.data(), 0, true);
                        newChildPool.clear();

                        gvdbInfo.childlist[n + GVDBInfo::MAX_LEVELS * slotId]->setName("childList" + std::to_string(n));
                    }
                    if (gvdbInfo.nodecnt[n + GVDBInfo::MAX_LEVELS * slotId] == 1) gvdbInfo.top_lev[slotId] = n;		// get top level for rendering
                }

                gvdbInfo.densityCompressScaleFactor[slotId] = 1.f;

                nvdb::Matrix4F xform = gvdb.mVDBXform;
                nvdb::Matrix4F invXform = gvdb.mVDBXformInv;
                nvdb::Matrix4F invRot = invXform;
                invRot(3, 0) = 0;
                invRot(3, 1) = 0;
                invRot(3, 2) = 0;

                memcpy((void*)&gvdbInfo.xform[slotId], &xform, sizeof(float) * 16);
                memcpy((void*)&gvdbInfo.invxform[slotId], &invXform, sizeof(float) * 16);
                memcpy((void*)&gvdbInfo.invxrot[slotId], &invRot, sizeof(float) * 16);

                if (slotId == 0)
                {
                    gvdbInfo.epsilon = gvdb.getEpsilon();
                    gvdbInfo.max_iter = gvdb.getMaxIter();
                    gvdbInfo.superVoxelWorldSpaceDiagonalLength =
                      8.f * sqrt(
                            dot(float3(gvdbInfo.xform[slotId][0]), float3(gvdbInfo.xform[slotId][0])) + dot(float3(gvdbInfo.xform[slotId][1]), float3(gvdbInfo.xform[slotId][1])) +
                            dot(float3(gvdbInfo.xform[slotId][2]), float3(gvdbInfo.xform[slotId][2])));
                }
                if (gvdb.mIsCustomVersion)
                {
                    gvdbInfo.bmin[slotId] = float3(gvdb.mEffectiveVoxMin.x, gvdb.mEffectiveVoxMin.y, gvdb.mEffectiveVoxMin.z);
                    gvdbInfo.bmax[slotId] = float3(gvdb.mEffectiveVoxMax.x, gvdb.mEffectiveVoxMax.y, gvdb.mEffectiveVoxMax.z); //mEffectiveVoxMax already + 1.f
                    gvdbInfo.maxValue[slotId] = gvdb.mGridValMax;
                    gvdbInfo.invMaxValue[slotId] = 1.f / gvdbInfo.maxValue[slotId];
                }
                else
                {
                    gvdbInfo.bmin[slotId] = float3(gvdb.mObjMin.x, gvdb.mObjMin.y, gvdb.mObjMin.z);
                    gvdbInfo.bmax[slotId] = float3(gvdb.mObjMax.x, gvdb.mObjMax.y, gvdb.mObjMax.z);
                }

                gvdbInfo.clr_chan = 0xFFFFFFFF;

                if (gvdb.mPool->getNumAtlas() == 0) {
                    printf("ERROR: No atlas created.\n");
                }
                else
                {
                    assert(gvdb.mPool->getAtlas(0).type == 3); //T_FLOAT

                    std::vector<float3> velocities; // abusing the velocities term for supervoxel

                    if (mipId == 2 * kNumMaxMips + 1)
                    {
                        std::string fullpath_y, fullpath_z;
                        std::string filename_y = vbxFile + "/" + filenameWithoutPath + "_velocity_y.vbx";
                        std::string filename_z = vbxFile + "/" + filenameWithoutPath + "_velocity_z.vbx";
                        findFileInDataDirectories(filename_y, fullpath_y);
                        findFileInDataDirectories(filename_z, fullpath_z);

                        VolumeGVDB	gvdb_y;
                        gvdb_y.Initialize(true);
                        gvdb_y.LoadVBX(fullpath_y, true);

                        VolumeGVDB	gvdb_z;
                        gvdb_z.Initialize(true);
                        gvdb_z.LoadVBX(fullpath_z, true);

                        float* vx = gvdb.mPool->getAtlasCPU(0);
                        float* vy = gvdb_y.mPool->getAtlasCPU(0);
                        float* vz = gvdb_z.mPool->getAtlasCPU(0);
                        int numTexels = gvdb.mPool->getAtlasRes(0).x * gvdb.mPool->getAtlasRes(0).y * gvdb.mPool->getAtlasRes(0).z;

                        velocities.resize(numTexels);
                        for (int i = 0; i < numTexels; i++)
                        {
                            velocities[i] = float3(vx[i], vy[i], vz[i]);
                        }
                    }

                    if (mipId == 2 * kNumMaxMips + 2)
                        assert(gvdb.mPool->getAtlasRes(0).z <= 2048);

                    assert(gvdb.mPool->getAtlasRes(0).z <= 4096);

                    // split into two 3d textures
                    int brickRes = gvdb.mPool->getAtlasBrickres(0);
                    int atlasDepth = gvdb.mPool->getAtlasRes(0).z;
                    int atlasWidth = gvdb.mPool->getAtlasRes(0).x;
                    int atlasHeight = gvdb.mPool->getAtlasRes(0).y;
                    int part1Depth = atlasDepth > 2048 ? 2048 / brickRes * brickRes : atlasDepth;
                    int part2Depth = atlasDepth - part1Depth;

                #if ATLAS_COMPRESSION == 2
                    int atlasWidthPadded = (atlasWidth + 3) / 4 * 4;
                    int atlasHeightPadded = (atlasHeight + 3) / 4 * 4;
                #endif

                    int numAtlasElements = atlasHeight * atlasWidth * atlasDepth;

                    std::vector<float> clampedDensity(numAtlasElements); // clamp extremely small density that causes problems
                    for (int i = 0; i < numAtlasElements; i++)
                        clampedDensity[i] = gvdb.mPool->getAtlasCPU(0)[i] / gvdb.mGridValMax < 1e-9 ? 0 : gvdb.mPool->getAtlasCPU(0)[i];

                    std::vector<uint8_t> compressedDensities;
                    std::vector<uint8_t> compressedDensities2;

                    bool useTextureCompression = mipId > 0 && mipId < kNumMaxMips || mipId == 0 && typeId == 1;

                    if (useTextureCompression)
                    {
                    #if ATLAS_COMPRESSION == 1
                        gvdbInfo.densityCompressScaleFactor[slotId] = gvdb.mGridValMax;
                        compressedDensities.resize(numAtlasElements);
                        for (int i = 0; i < numAtlasElements; i++)
                        {
                            float density = clampedDensity[i];
                            compressedDensities[i] = (uint8_t)std::max(0, std::min(255, (int)round(255 * (density / gvdb.mGridValMax))));
                            if (compressedDensities[i] == 0 && density > 0.f && typeId == 1) compressedDensities[i] = 1; // This might cause some trouble in some high dynamic range volumes, in that case, use float for distance sampling
                        }
                    #endif

                    #if ATLAS_COMPRESSION == 2
                        gvdbInfo.densityCompressScaleFactor[slotId] = gvdb.mGridValMax;
                        // convert to BC4
                        BCHelper::CompressImage(clampedDensity.data(), int3(atlasWidth, atlasHeight, part1Depth), gvdb.mGridValMax, typeId == 1, compressedDensities);
                        BCHelper::CompressImage(clampedDensity.data() + atlasWidth * atlasHeight * part1Depth, int3(atlasWidth, atlasHeight, part2Depth), gvdb.mGridValMax, typeId == 1, compressedDensities2);
                    #endif
                    }

                    // TODO: maybe even change to BC4 texture?
                    if (mipId < 2 * kNumMaxMips + 1)
                    {
                    #if ATLAS_COMPRESSION > 0
                        if (useTextureCompression)
                    #if ATLAS_COMPRESSION == 1
                            gvdbInfo.volIn[slotId] = Texture::create3D(atlasWidth, atlasHeight, part1Depth, ResourceFormat::R8Unorm, 1, compressedDensities.data(), ResourceBindFlags::ShaderResource, false, true);
                    #else
                            gvdbInfo.volIn[slotId] = Texture::create3D(atlasWidthPadded, atlasHeightPadded, part1Depth, ResourceFormat::BC4Unorm, 1, compressedDensities.data(), ResourceBindFlags::ShaderResource, false, true);
                    #endif
                        else
                    #endif
                        gvdbInfo.volIn[slotId] = Texture::create3D(atlasWidth, atlasHeight, part1Depth, ResourceFormat::R32Float, 1, clampedDensity.data(), ResourceBindFlags::ShaderResource, false, true);
                        gvdbInfo.volIn[slotId]->setName("volIn");
                    }
                    else if (mipId == 2 * kNumMaxMips + 1)
                    {
                        int texId = 0;
                        gvdbInfo.velocityIn[texId] = Texture::create3D(atlasWidth, atlasHeight, part1Depth, ResourceFormat::RGB32Float, 1, velocities.data(), ResourceBindFlags::ShaderResource, false, true);
                    }

                    #if ATLAS_COMPRESSION == 2
                    gvdbInfo.volInDimensions[slotId] = int3(atlasWidthPadded, atlasHeightPadded, part1Depth);
                    #else
                    gvdbInfo.volInDimensions[slotId] = int3(atlasWidth, atlasHeight, part1Depth);
                    #endif

                    if (atlasDepth > 2048)
                    {
                        if (mipId != 2 * kNumMaxMips + 1)
                        {
                    #if ATLAS_COMPRESSION > 0
                            if (useTextureCompression)
                    #if ATLAS_COMPRESSION == 1
                                gvdbInfo.volIn_part2[slotId] = Texture::create3D(atlasWidth, atlasHeight, part2Depth, ResourceFormat::R8Unorm, 1, compressedDensities.data() + atlasWidth * atlasHeight * part1Depth, ResourceBindFlags::ShaderResource, false, true);
                    #else
                                gvdbInfo.volIn_part2[slotId] = Texture::create3D(atlasWidthPadded, atlasHeightPadded, part2Depth, ResourceFormat::BC4Unorm, 1, compressedDensities2.data(), ResourceBindFlags::ShaderResource, false, true);
                    #endif
                            else
                    #endif
                                gvdbInfo.volIn_part2[slotId] = Texture::create3D(atlasWidth, atlasHeight, part2Depth, ResourceFormat::R32Float, 1, clampedDensity.data() + atlasWidth * atlasHeight * part1Depth, ResourceBindFlags::ShaderResource, false, true);
                            gvdbInfo.volIn_part2[slotId]->setName("volIn_part2");
                        }
                        else
                        {
                            int texId = 0;
                            gvdbInfo.velocityIn_part2[texId] = Texture::create3D(atlasWidth, atlasHeight, part2Depth, ResourceFormat::RGB32Float, 1, velocities.data() + atlasWidth * atlasHeight * part1Depth, ResourceBindFlags::ShaderResource, false, true);
                            gvdbInfo.volIn_part2[slotId]->setName("velocityIn_part2");
                        }


                    #if ATLAS_COMPRESSION == 2
                        gvdbInfo.volInDimensions_part2[slotId] = int3(atlasWidthPadded, atlasHeightPadded, part2Depth);
                    #else
                        gvdbInfo.volInDimensions_part2[slotId] = int3(atlasWidth, atlasHeight, part2Depth);
                    #endif
                    }
                }

                // bind shader block
                gvdbInfo.bindParameterBlock(gvdbBlock, slotId);

                if (mipId == 0)
                {
                    mVolumeDesc.PhaseFunctionConstantG = g;
                    mVolumeDesc.sigma_s = sigma_s;
                    mVolumeDesc.sigma_a = sigma_a;
                    assert(sigma_s.x + sigma_a.x == sigma_s.y + sigma_a.y && sigma_s.y + sigma_a.y == sigma_s.z + sigma_a.z);
                    mVolumeDesc.sigma_t = sigma_s.x + sigma_a.x;
                    mVolumeDesc.tStep = (length(float3(gvdbInfo.xform[0][0])) + length(float3(gvdbInfo.xform[0][1])) + length(float3(gvdbInfo.xform[0][2]))) / 3.f; // TODO: adjust this in shader for ray marching in mipmaps
                    mVolumeDesc.densityScaleFactor = DensityScale;
                    mVolumeDesc.densityScaleFactorByScaling = DensityScale / mVolumeWorldScaling;
                    mVolumeDesc.invMaxDensity = gvdbInfo.invMaxValue[0];
                    mVolumeDesc.gridRes = uint3(round(gvdbInfo.bmax - gvdbInfo.bmin));
                    mVolumeDesc.hasEmission = false;
                    mVolumeDesc.LeScale = LeScale;
                    mVolumeDesc.temperatureCutOff = temperatureCutOff;
                    mVolumeDesc.temperatureScale = temperatureScale;
                }

                if (typeId == numTypes - 1 && mipId == numMips - 1)
                {
                    if (hasEmissionGrid && mipId < 2 * kNumMaxMips)
                    {
                        mipId = 2 * kNumMaxMips - 1;
                        numMips = 2 * kNumMaxMips + 1;
                    }
                    else if (hasVelocityGrid && mipId < 2 * kNumMaxMips + 1)
                    {
                        mipId = 2 * kNumMaxMips;
                        numMips = 2 * kNumMaxMips + 2;
                    }
                }
            }
        }

        if (curFrameId >= 1)
        {
            // bind last frame mips
            for (int i = 0; i < gvdbParamBlocks.numMips; i++)
            {
                mGVDBInfos[curFrameId - 1].bindPrevParameterBlock(gvdbBlock, i);
            }
            if (gvdbParamBlocks.hasEmissionGrid) mGVDBInfos[curFrameId - 1].bindPrevParameterBlock(gvdbBlock, 2 * kNumMaxMips);
            if (gvdbParamBlocks.hasVelocityGrid) mGVDBInfos[curFrameId - 1].bindPrevParameterBlock(gvdbBlock, 2 * kNumMaxMips + 1);
        }

        mVolumeDesc.numMips = gvdbParamBlocks.numMips;
        mVolumeDesc.velocityScale = 1;
        mVolumeDesc.hasEmission = gvdbParamBlocks.hasEmissionGrid;
        mVolumeDesc.hasVelocity = gvdbParamBlocks.hasVelocityGrid;
        mVolumeDesc.hasAnimation = curFrameId >= 0;
        mVolumeDesc.usePrevGridForReproj = false;
        mVolumeDescArray.push_back(mVolumeDesc);
        mpSceneBlock["volumeDesc"].setBlob(mVolumeDesc);


        if (curFrameId <= 0)
        {
            mpSceneBlock["volumeWorldTranslation"] = mVolumeWorldTranslation;
            mpSceneBlock["volumeWorldScaling"] = mVolumeWorldScaling;
            glm::mat4 externalModelToWorldMatrix = computeVolumeExternalModelToWorldMatrix();
            glm::mat4 externalWorldToModelMatrix = glm::inverse(externalModelToWorldMatrix);
            mpSceneBlock["volumeExternalWorldToModelMatrix"] = externalWorldToModelMatrix;
            mpSceneBlock["volumeExternalModelToWorldMatrix"] = externalModelToWorldMatrix;
        }

        glm::mat4 externalModelToWorldMatrix = computeVolumeExternalModelToWorldMatrix();
        float3 worldMin = externalModelToWorldMatrix * gvdbInfo.xform[0] * float4(gvdbInfo.bmin[0], 1);
        float3 worldMax = externalModelToWorldMatrix * gvdbInfo.xform[0] * float4(gvdbInfo.bmax[0], 1);
        mVDBVolumeBBs.push_back(BoundingBox::fromMinMax(worldMin, worldMax));

        gvdbParamBlocks.paramBlock = gvdbBlock;
        mGVDBInfos.push_back(gvdbInfo);
        mGVDBVolumes.push_back(gvdbParamBlocks);

        updateBounds();
        resetCamera();

        gpDevice->flushAndSync();

        return (uint32_t)mGVDBVolumes.size() - 1;
    }


    uint32_t Scene::addGVDBVolumeSequence(float3 sigma_a, float3 sigma_s, float g, std::string dataFilePrefix, int numberFixedLength, int startFrame, int numFrames, int numMips /*= 1*/, float DensityScale, bool hasVelocityGrid /*= false*/, bool hasEmissionGrid /*= false*/, float LeScale /*= 0.005f*/, float temperatureCutOff /*= 1.f*/, float temperatureScale /*= 100.f*/, float3 worldTranslation, float3 worldRotation, float worldScaling)
    {
        mUseAnimatedVolume = true;
        mVDBAnimationFrames = numFrames;
        mVDBAnimationFrameId = mVDBAnimationFrames - 1;

        for (int i = 0; i < numFrames; i++)
        {
            std::string frameName = std::to_string(startFrame + i);
            int numChars = (int)frameName.length();
            for (int j = numChars; j < numberFixedLength; j++)
                frameName = "0" + frameName;
            addGVDBVolume(i, sigma_a, sigma_s, g, dataFilePrefix + frameName, numMips, DensityScale, hasVelocityGrid, hasEmissionGrid, LeScale, temperatureCutOff, temperatureScale, worldTranslation, worldRotation, worldScaling);
        }

        // the "previous frame" of first frame is the last frame
        {
            for (int i = 0; i < mGVDBVolumes[0].numMips; i++)
            {
                mGVDBInfos[numFrames - 1].bindPrevParameterBlock(mGVDBVolumes[0].paramBlock, i);
            }
            if (mGVDBVolumes[0].hasEmissionGrid) mGVDBInfos[numFrames - 1].bindPrevParameterBlock(mGVDBVolumes[0].paramBlock, 2 * kNumMaxMips);
            if (mGVDBVolumes[0].hasVelocityGrid) mGVDBInfos[numFrames - 1].bindPrevParameterBlock(mGVDBVolumes[0].paramBlock, 2 * kNumMaxMips + 1);
        }

        return numFrames;
    }

    uint32_t Scene::addParticleSystem(ParticleSystem::SharedPtr pParticleSystem, const Material::SharedPtr& pMaterial)
    {
        ParticleSystemDesc desc;
        mParticleSystems.push_back(pParticleSystem);
#ifdef PROCEDURAL_PARTICLE
        desc.ibOffset = mTotalParticles;
        desc.vbOffset = mTotalParticles;
        desc.indexCount = pParticleSystem->getMaxParticles();
        desc.vertexCount = pParticleSystem->getMaxParticles();
#else
        desc.ibOffset = 6 * mTotalParticles;
        desc.vbOffset = 4 * mTotalParticles;
        desc.indexCount = 6 * pParticleSystem->getMaxParticles();
        desc.vertexCount = 4 * pParticleSystem->getMaxParticles();
#endif
        mTotalParticles += pParticleSystem->getMaxParticles();
        mMaterials.push_back(pMaterial);
        desc.materialID = (uint32_t)mMaterials.size() - 1;
        mParticleSystemDesc.push_back(desc);
        createParticleVao();
        finalize(true);
        return (uint32_t)mParticleSystemDesc.size() - 1;
    }

    void Scene::createParticleSystemIO(int32_t maxParticles, int32_t maxEmitPerFrame, float fixedInterval, uint32_t maxRenderFrames,
        uint32_t shadingModel, bool shouldSort,
        ParticleSystem::EmitterData* pEmitterData, ParticleSystemManager::ParticleMaterialDesc* pMaterialDesc, const std::string textureFile)
    {
        mParticleSystemManager.createParticleSystemIO(maxParticles, maxEmitPerFrame, fixedInterval, maxRenderFrames, shadingModel, shouldSort,
            pEmitterData, pMaterialDesc, textureFile, this->shared_from_this());
    }

    void Scene::deleteAllParticleSystems()
    {
        mTotalParticles = 0;
        mParticleSystemDesc.clear();
        mParticleSystems.clear();
        mpParticleVao = nullptr;
        finalize(true);
    }

    void Scene::bindParticlePoolVar(const ShaderVar& var)
    {
        for (uint32_t i = 0; i < getParticleSystemCount(); ++i)
        {
            auto pSys = getParticleSystem(i);
            var[i] = pSys->getParticlePool();
        }
    }

    SCRIPT_BINDING(Scene)
    {
        pybind11::class_<Scene, Scene::SharedPtr> scene(m, "Scene");
        scene.def_property(kCamera.c_str(), &Scene::getCamera, &Scene::setCamera);
        scene.def_property_readonly(kCameras.c_str(), &Scene::getCameras);
        scene.def_property_readonly(kEnvMap.c_str(), &Scene::getEnvMap);
        scene.def_property_readonly(kMaterials.c_str(), &Scene::getMaterials);
        scene.def_property(kCameraSpeed.c_str(), &Scene::getCameraSpeed, &Scene::setCameraSpeed);
        scene.def_property(kAnimated.c_str(), &Scene::isAnimated, &Scene::setIsAnimated);
        scene.def_property(kRenderSettings.c_str(), pybind11::overload_cast<void>(&Scene::getRenderSettings, pybind11::const_), &Scene::setRenderSettings);

        scene.def_property(kGlobalSurfaceAlphaMultipler.c_str(), &Scene::getGlobalSurfaceAlphaMultipler, &Scene::setGlobalSurfaceAlphaMultipler);
        scene.def_property(kGlobalParticleCurveAlphaMultipler.c_str(), &Scene::getGlobalParticleCurveAlphaMultipler, &Scene::setGlobalParticleCurveAlphaMultipler);

        scene.def("animate", &Scene::toggleAnimations, "animate"_a); // PYTHONDEPRECATED
        auto animateCamera = [](Scene* pScene, bool animate) { pScene->getCamera()->setIsAnimated(animate); };
        scene.def("animateCamera", animateCamera, "animate"_a); // PYTHONDEPRECATED
        auto animateLight = [](Scene* pScene, uint32_t index, bool animate) { pScene->getLight(index)->setIsAnimated(animate); };
        scene.def("animateLight", animateLight, "index"_a, "animate"_a); // PYTHONDEPRECATED

        scene.def(kSetEnvMap.c_str(), &Scene::loadEnvMap, "filename"_a);
        scene.def(kSetEnvMapIntensity.c_str(), &Scene::setEnvMapIntensity, "intensity"_a);
        scene.def(kSetEnvMapRotation.c_str(), &Scene::setEnvMapRotation, "rotDegrees"_a);
        scene.def(kSetEmissiveIntensityMultiplier.c_str(), &Scene::setEmissiveIntensityMultiplier, "multiplier"_a);
        scene.def(kGetLight.c_str(), &Scene::getLight, "index"_a);
        scene.def(kGetLight.c_str(), &Scene::getLightByName, "name"_a);
        scene.def("light", &Scene::getLight); // PYTHONDEPRECATED
        scene.def("light", &Scene::getLightByName); // PYTHONDEPRECATED
        scene.def(kGetMaterial.c_str(), &Scene::getMaterial, "index"_a);
        scene.def(kGetMaterial.c_str(), &Scene::getMaterialByName, "name"_a);
        scene.def("material", &Scene::getMaterial); // PYTHONDEPRECATED
        scene.def("material", &Scene::getMaterialByName); // PYTHONDEPRECATED

        scene.def("addDirectionalLight", &Scene::addDirectionalLight, "worldDirection"_a, "intensity"_a);
        scene.def("addPointLight", &Scene::addPointLight, "worldPosition"_a, "worldDirection"_a, "openingAngle"_a, "intensity"_a);

        // Viewpoints
        scene.def(kAddViewpoint.c_str(), pybind11::overload_cast<>(&Scene::addViewpoint)); // add current camera as viewpoint
        scene.def(kAddViewpoint.c_str(), pybind11::overload_cast<const float3&, const float3&, const float3&, uint32_t>(&Scene::addViewpoint), "position"_a, "target"_a, "up"_a, "cameraIndex"_a = 0); // add specified viewpoint
        scene.def(kRemoveViewpoint.c_str(), &Scene::removeViewpoint); // remove the selected viewpoint
        scene.def(kSelectViewpoint.c_str(), &Scene::selectViewpoint, "index"_a); // select a viewpoint by index

        scene.def("viewpoint", pybind11::overload_cast<>(&Scene::addViewpoint)); // PYTHONDEPRECATED save the current camera position etc.
        scene.def("viewpoint", pybind11::overload_cast<uint32_t>(&Scene::selectViewpoint)); // PYTHONDEPRECATED select a previously saved camera viewpoint

        // RenderSettings
        ScriptBindings::SerializableStruct<Scene::RenderSettings> renderSettings(m, "SceneRenderSettings");
#define field(f_) field(#f_, &Scene::RenderSettings::f_)
        renderSettings.field(useEnvLight);
        renderSettings.field(useAnalyticLights);
        renderSettings.field(useEmissiveLights);
#undef field
    }
}



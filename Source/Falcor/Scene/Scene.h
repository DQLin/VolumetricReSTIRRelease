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
#pragma once
#include "Core/API/VAO.h"
#include "Animation/Animation.h"
#include "Lights/Light.h"
#include "Lights/LightProbe.h"
#include "Camera/Camera.h"
#include "Material/Material.h"
#include "Utils/Math/AABB.h"
#include "Animation/AnimationController.h"
#include "Camera/CameraController.h"
#include "Experimental/Scene/Lights/LightCollection.h"
#include "Experimental/Scene/Lights/EnvMap.h"
#include "SceneTypes.slang"
#include "Scene/ParticleSystem/ParticleSystem.h"
#include "Scene/ParticleSystemManager.h"
#include "Scene/CurveFileLoader.h"

namespace Falcor
{
    class RtProgramVars;

    /** DXR Scene and Resources Layout:
        - BLAS creation logic is similar to Falcor 3.0, and are grouped in the following order:
            1) For non-instanced meshes, group them if they use the same scene graph transform matrix. One BLAS is created per group.
                a) It is possible a non-instanced mesh has no other meshes to merge with. In that case, the mesh goes in its own BLAS.
            2) For instanced meshes, one BLAS is created per mesh.

        - TLAS Construction:
            - Hit shaders use InstanceID() and GeometryIndex() to identify what was hit.
            - InstanceID is set like a starting offset so that (InstanceID + GeometryIndex) maps to unique indices.
            - Shader table has one hit group per mesh. InstanceContribution is set accordingly for correct lookup.

        Acceleration Structure Layout Example (Scene with 8 meshes, 10 instances total):

                               ----------------------------------------------------------------
                               |                            Value(s)                          |
        ---------------------------------------------------------------------------------------
        | InstanceID           |  0                    |  4  |  5  |  6  |  7  |  8  |  9     |  7 INSTANCE_DESCs in TLAS
        | InstanceContribution |  0                    |  4  |  5  |  6  |  7  |  7  |  7     |  Helps look up one hit group per MESH
        | BLAS Geometry Index  |  0  ,  1  ,  2  ,  3  |  0  |  0  |  0  |  0                 |  5 BLAS's containing 8 meshes total
        ---------------------------------------------------------------------------------------
        | Notes                | Meshes merged into    | One instance    | Multiple instances |
        |                      | one BLAS              | per mesh        | of a mesh          |
        --------------------------------------------------------------------------------------|

        - "InstanceID() + GeometryIndex()" is used for indexing into MeshInstanceData.
        - This is wrapped in getGlobalHitID() in Raytracing.slang.
    */

    class dlldecl Scene : public std::enable_shared_from_this<Scene>
    {
    public:
        friend class CurveFileLoader;

        using SharedPtr = std::shared_ptr<Scene>;
        using LightList = std::vector<Light::SharedPtr>;
        static const uint32_t kMaxBonesPerVertex = 4;

        static const FileDialogFilterVec& getFileExtensionFilters();

        static SharedPtr create(const std::string& filename);

        // #SCENE: we should get rid of this. We can't right now because we can't create a structured-buffer of materials (MaterialData contains textures)
        Shader::DefineList getSceneDefines() const;

        /** Render settings determining how the scene is rendered.
            This is used primarily by the path tracer renderers.
        */
        struct RenderSettings
        {
            bool useEnvLight = true;        ///< Enable distant lighting from environment map.
            bool useAnalyticLights = true;  ///< Enable lighting from analytic lights.
            bool useEmissiveLights = true;  ///< Enable lighting from emissive lights.

            bool operator==(const RenderSettings& other) const
            {
                return (useEnvLight == other.useEnvLight) &&
                    (useAnalyticLights == other.useAnalyticLights) &&
                    (useEmissiveLights == other.useEmissiveLights);
            }

            bool operator!=(const RenderSettings& other) const { return !(*this == other); }
        };

        enum class RenderFlags
        {
            None                    = 0x0,
            UserRasterizerState     = 0x1,  ///< Use the rasterizer state currently bound to `pState`. If this flag is not set, the default rasterizer state will be used.
                                            ///< Note that we need to change the rasterizer state during rendering because some meshes have a negative scale factor, and hence the triangles will have a different winding order.
                                            ///< If such meshes exist, overriding the state may result in incorrect rendering output
        };

        /** Flags indicating if and what was updated in the scene
        */
        enum class UpdateFlags
        {
            None                        = 0x0,   ///< Nothing happened
            MeshesMoved                 = 0x1,   ///< Meshes moved
            CameraMoved                 = 0x2,   ///< The camera moved
            CameraPropertiesChanged     = 0x4,   ///< Some camera properties changed, excluding position
            CameraSwitched              = 0x8,   ///< Selected a different camera
            LightsMoved                 = 0x10,  ///< Lights were moved
            LightIntensityChanged       = 0x20,  ///< Light intensity changed
            LightPropertiesChanged      = 0x40,  ///< Other light changes not included in LightIntensityChanged and LightsMoved
            SceneGraphChanged           = 0x80,  ///< Any transform in the scene graph changed.
            LightCollectionChanged      = 0x100, ///< Light collection changed (mesh lights)
            MaterialsChanged            = 0x200, ///< Materials changed
            EnvMapChanged               = 0x400, ///< Environment map changed (check EnvMap::getChanges() for more specific information)
            LightCountChanged           = 0x800, ///< Number of active lights changed
            RenderSettingsChanged       = 0x1000,///< Render settings changed

            All                         = -1
        };

        /** Settings for how the scene is updated
        */
        enum class UpdateMode
        {
            Rebuild,    ///< Recreate acceleration structure when updates are needed
            Refit       ///< Update acceleration structure when updates are needed
        };

        enum class CameraControllerType
        {
            FirstPerson,
            Orbiter,
            SixDOF
        };

        /** Get the render settings.
        */
        const RenderSettings& getRenderSettings() const { return mRenderSettings; }

        /** Get the render settings.
        */
        RenderSettings& getRenderSettings() { return mRenderSettings; }

        /** Set the render settings.
        */
        void setRenderSettings(const RenderSettings& renderSettings) { mRenderSettings = renderSettings; }

        /** Returns true if environment map is available and should be used as the background.
        */
        bool useEnvBackground() const;

        /** Returns true if environment map is available and should be used as a distant light.
        */
        bool useEnvLight() const;

        /** Returns true if there are active analytic lights and they should be used for lighting.
        */
        bool useAnalyticLights() const;

        /** Returns true if there are active emissive lights and they should be used for lighting.
        */
        bool useEmissiveLights() const;

        /** Access the scene's currently selected camera to change properties or to use elsewhere.
        */
        const Camera::SharedPtr& getCamera() { return mCameras[mSelectedCamera]; }

        /** Get a list of all cameras in the scene.
        */
        const std::vector<Camera::SharedPtr>& getCameras() { return mCameras; };

        /** Select a different camera to use. The camera must already exist in the scene.
        */
        void setCamera(const Camera::SharedPtr& pCamera);

        /** Set the currently selected camera's aspect ratio
        */
        void setCameraAspectRatio(float ratio);

        /** Set the camera controller type
        */
        void setCameraController(CameraControllerType type);

        /** Get the camera controller type
        */
        CameraControllerType getCameraControllerType() const { return mCamCtrlType; }

        /** Toggle whether the currently selected camera is animated.
        */
        deprecate("4.0.2", "Use Camera::setIsAnimated() instead.")
        void toggleCameraAnimation(bool active) { mCameras[mSelectedCamera]->setIsAnimated(active); }

        /** Reset the currently selected camera.
            This function will place the camera at the center of scene and optionally set the depth range to some reasonable pre-determined values
        */
        void resetCamera(bool resetDepthRange = true);

        /** Set the camera's speed
        */
        void setCameraSpeed(float speed);

        /** Get the camera's speed
        */
        float getCameraSpeed() const { return mCameraSpeed; }

        /** Add the currently selected camera's viewpoint to the list of viewpoints.
        */
        void addViewpoint();

        /** Select a camera to be used by index.
        */
        void selectCamera(uint32_t index);

        /** Select a camera to be used by name.
        */
        void selectCamera(std::string name);

        deprecate("4.0.1", "Use addViewpoint() instead.")
        void saveNewViewpoint() { addViewpoint(); }

        /** Add a new viewpoint to the list of viewpoints.
        */
        void addViewpoint(const float3& position, const float3& target, const float3& up, uint32_t cameraIndex = 0);

        /** Remove the currently active viewpoint.
        */
        void removeViewpoint();

        /** Select a viewpoint and move the camera to it.
        */
        void selectViewpoint(uint32_t index);

        deprecate("4.0.1", "Use selectViewpoint() instead.")
        void gotoViewpoint(uint32_t index) { selectViewpoint(index); }

        /** Returns true if there are saved viewpoints (used for dumping to config)
        */
        bool hasSavedViewpoints() { return mViewpoints.size() > 1; }


        /** Get/Set global object alpha
        */

        float getGlobalSurfaceAlphaMultipler() { return mSurfaceGlobalAlphaMultipler; }
        void setGlobalSurfaceAlphaMultipler(float surfaceGlobalAlphaMultipler);

        float getGlobalParticleCurveAlphaMultipler() { return mParticleCurveGlobalAlphaMultipler; }
        void setGlobalParticleCurveAlphaMultipler(float AlphaMultipler);

        /** Get the number of meshes
        */
        uint32_t getMeshCount() const { return (uint32_t)mMeshDesc.size(); }
        
        uint32_t getCurveCount() const { return (uint32_t)mCurveDesc.size(); }

        uint32_t getParticleSystemCount() const { return (uint32_t)mParticleSystemDesc.size(); }

        /** Get a mesh desc
        */
        const MeshDesc& getMesh(uint32_t meshID) const { return mMeshDesc[meshID]; }
        const CurveDesc& getCurve(uint32_t curveID) const { return mCurveDesc[curveID]; }
        VolumeDesc& getCurrentVolumeDesc() { return mVolumeDesc; }
        void updateVolumeDesc();

        const ParticleSystemDesc& getParticleSystemDesc(uint32_t particleSystemID) const { return mParticleSystemDesc[particleSystemID]; }
        const ParticleSystem::SharedPtr& getParticleSystem(uint32_t particleSystemID) const { return mParticleSystems[particleSystemID]; }

        /** Get the number of mesh instances
        */
        uint32_t getMeshInstanceCount() const { return (uint32_t)mMeshInstanceData.size(); }

        /** Get a mesh instance desc
        */
        const MeshInstanceData& getMeshInstance(uint32_t instanceID) const { return mMeshInstanceData[instanceID]; }

        /** Get a list of all materials in the scene.
        */
        const std::vector<Material::SharedPtr>& getMaterials() const { return mMaterials; }

        /** Get the number of materials in the scene
        */
        uint32_t getMaterialCount() const { return (uint32_t)mMaterials.size(); }

        /** Get a material
        */
        const Material::SharedPtr& getMaterial(uint32_t materialID) const { return mMaterials[materialID]; }

        /** Get a material by name
        */
        Material::SharedPtr getMaterialByName(const std::string& name) const;

        /** Get the scene bounds
        */
        const BoundingBox& getSceneBounds() const { return mSceneBB; }

        /** Get a mesh's bounds
        */
        const BoundingBox& getMeshBounds(uint32_t meshID) const { return mMeshBBs[meshID]; }

        /** Get the number of lights in the scene
        */
        uint32_t getLightCount() const { return (uint32_t)mLights.size(); }

        /** Get a light
        */
        const Light::SharedPtr& getLight(uint32_t lightID) const { return mLights[lightID]; }

        /** Get a light by name
        */
        Light::SharedPtr getLightByName(const std::string& name) const;

        Light::SharedPtr getLightByID(int lightID) const;

        /** Get the light collection representing all the mesh lights in the scene.
            The light collection is created lazily on the first call. It needs a render context.
            to run the initialization shaders.
            \param[in] pContext Render context.
            \return Returns the light collection.
        */
        const LightCollection::SharedPtr& getLightCollection(RenderContext* pContext);

        /** Get the environment map or nullptr if it doesn't exist.
        */
        const EnvMap::SharedPtr& getEnvMap() const { return mpEnvMap; }

        /** Get the light probe or nullptr if it doesn't exist.
        */
        const LightProbe::SharedPtr& getLightProbe() const { return mpLightProbe; }

        /** Toggle whether the specified light is animated.
        */
        deprecate("4.0.2", "Use Light::setIsAnimated() instead.")
        void toggleLightAnimation(int index, bool active) { mLights[index]->setIsAnimated(active); }

        /** Set how the scene's TLASes are updated when raytracing.
            TLASes are REBUILT by default
        */
        void setTlasUpdateMode(UpdateMode mode) { mTlasUpdateMode = mode; }

        /** Get the scene's TLAS update mode when raytracing.
        */
        UpdateMode getTlasUpdateMode() { return mTlasUpdateMode; }

        /** Set how the scene's BLASes are updated when raytracing.
            BLASes are REFIT by default
        */
        void setBlasUpdateMode(UpdateMode mode);

        /** Get the scene's BLAS update mode when raytracing.
        */
        UpdateMode getBlasUpdateMode() { return mBlasUpdateMode; }

        /** Update the scene. Call this once per frame to update the camera location, animations, etc.
            \param pContext
            \param currentTime The current time in seconds
        */
        UpdateFlags update(RenderContext* pContext, double currentTime);

        /** Get the changes that happened during the last update
            The flags only change during an `update()` call, if something changed between calling `update()` and `getUpdates()`, the returned result will not reflect it
        */
        UpdateFlags getUpdates() const { return mUpdates; }

        /** Render the scene using the rasterizer
        */
        void render(RenderContext* pContext, GraphicsState* pState, GraphicsVars* pVars, RenderFlags flags = RenderFlags::None);

        /** Render the scene using raytracing
        */
        void raytrace(RenderContext* pContext, RtProgram* pProgram, const std::shared_ptr<RtProgramVars>& pVars, uint3 dispatchDims);

        /** Render the UI
        */
        void renderUI(Gui::Widgets& widget);

        bool renderVolumeUI(Gui::Widgets& widget);

        /** Bind a sampler to the materials
        */
        void bindSamplerToMaterials(const Sampler::SharedPtr& pSampler);

        /** Get the scene's VAO
        */
        const Vao::SharedPtr& getVao() const { return mpVao; }

        /** Set an environment map.
            \param[in] pEnvMap Environment map. Can be nullptr.
        */
        void setEnvMap(EnvMap::SharedPtr pEnvMap);

        /** Load an environment from an image.
            \param[in] filename Texture filename.
        */
        void loadEnvMap(const std::string& filename);

        void setEnvMapIntensity(float intensity);

        void setEmissiveIntensityMultiplier(float multiplier);

        void setEnvMapRotation(const float3& rotDegrees);

        /** Handle mouse events
        */
        bool onMouseEvent(const MouseEvent& mouseEvent);

        /** Handle keyboard events
        */
        bool onKeyEvent(const KeyboardEvent& keyEvent);

        /** Get the filename that the scene was loaded from
        */
        const std::string& getFilename() const { return mFilename; }

        /** Get the animation controller.
        */
        const AnimationController* getAnimationController() const { return mpAnimationController.get(); }

        AnimationController* getModifiableAnimationController() const { return mpAnimationController.get(); }

        /** Returns true if scene has animation data.
        */
        bool hasAnimation() const { return mpAnimationController->hasAnimations(); }

        /** Enable/disable scene animation.
        */
        void setIsAnimated(bool isAnimated) { mpAnimationController->setEnabled(isAnimated); }

        /** Returns true if scene animation is enabled.
        */
        bool isAnimated() const { return mpAnimationController->isEnabled(); };

        /** Toggle all animations on or off.
        */
        void toggleAnimations(bool animate);

        /** Get the parameter block with all scene resources.
            Note that the camera is not bound automatically.
        */
        const ParameterBlock::SharedPtr& getParameterBlock() const { return mpSceneBlock; }

        /** Set the BLAS geometry index into the local vars for each geometry.
            This is a workaround before GeometryIndex() is supported in shaders.
        */
        void setGeometryIndexIntoRtVars(const std::shared_ptr<RtProgramVars>& pVars);

        /** Set the scene ray tracing resources into a shader var.
            The acceleration structure is created lazily, which requires the render context.
            \param[in] pContext Render context.
            \param[in] var Shader variable to set data into, usually the root var.
            \param[in] rayTypeCount Number of ray types in raygen program. Not needed for DXR 1.1.
        */
        void setRaytracingShaderData(RenderContext* pContext, const ShaderVar& var, uint32_t rayTypeCount = 1);

        void setRaytracingAcceleraitonStructure(RenderContext* pContext, const ShaderVar& var);

        std::string getScript(const std::string& sceneVar);

        uint32_t addSimpleCurveModel(std::string filename, float width, float3 color);

        uint32_t addGVDBVolume(int curFrameId, float3 sigma_a, float3 sigma_s, float g, std::string vbxFile, int numMips = 1, float DensityScale = 1.f, bool hasVelocityGrid = false, bool hasEmissionGrid = false, float LeScale = 0.005f, float temperatureCutOff = 1.f, float temperatureScale = 100.f, float3 worldTranslation = float3(0,0,0), float3 worldRotation = float3(0,0,0), float worldScaling = 1.f );

        uint32_t addGVDBVolumeSequence(float3 sigma_a, float3 sigma_s, float g, std::string dataFilePrefix, int numberFixedLength, int startFrame, int numFrames, int numMips = 1, float DensityScale = 1.f, bool hasVelocityGrid = false, bool hasEmissionGrid = false, float LeScale = 0.005f, float temperatureCutOff = 1.f, float temperatureScale = 100.f, float3 worldTranslation = float3(0, 0, 0), float3 worldRotation = float3(0, 0, 0), float worldScaling = 1.f);

        Vao::SharedPtr getParticleVao() { return mpParticleVao; };

        uint32_t addParticleSystem(ParticleSystem::SharedPtr pParticleSystem, const Material::SharedPtr& pMaterial);

        void createParticleSystemIO(int32_t maxParticles, int32_t maxEmitPerFrame, float fixedInterval, uint32_t maxRenderFrames,
            uint32_t shadingModel, bool shouldSort,
            ParticleSystem::EmitterData* pEmitterData, ParticleSystemManager::ParticleMaterialDesc* pMaterialDesc, const std::string textureFile);

        void deleteAllParticleSystems();

        void bindParticlePoolVar(const ShaderVar& var);

        void updateCurveDisplayWidth(float displayWidth);

        uint32_t addLight(const Light::SharedPtr& pLight);

        uint32_t addDirectionalLight(float3 worldDirection, float3 intensity);

        uint32_t addPointLight(float3 worldPosition, float3 worldDirection, float openingAngle, float3 intensity);

        BoundingBox getVDBBoundingBox(int volumeId)
        {
            return mVDBVolumeBBs[volumeId];
        }

        float3 getSceneVolumeCenter()
        {
            return mSceneVolumeBB.center;
        }

        void setVolumeShaderData(ShaderVar const& var, int volumeId = -1);

        void setGVDBShaderData(ShaderVar const& var, int volumeId = -1)
        {
            if (volumeId == -1) volumeId = mVDBAnimationFrameId;

            var["gvdb"] = mGVDBVolumes[volumeId].paramBlock;

            if (mGVDBVolumes[volumeId].hasEmissionGrid)
                var["gBlackBodyRadiationTex"] = mpBlackBodyRadiationTexture;

            mVDBLastAnimationFrameId = volumeId;
        }

        int getVolumeNumMips(int volumeId = -1)
        {
            if (volumeId == -1) volumeId = mVDBAnimationFrameId;
            return mGVDBVolumes[volumeId].numMips;
        }

        bool mFreezeCamera = false;

    private:
        friend class SceneBuilder;
        friend class AnimationController;

        static constexpr uint32_t kStaticDataBufferIndex = 0;
        static constexpr uint32_t kPrevVertexBufferIndex = kStaticDataBufferIndex + 1;
        static constexpr uint32_t kDrawIdBufferIndex = kPrevVertexBufferIndex + 1;
        static constexpr uint32_t kVertexBufferCount = kDrawIdBufferIndex + 1;

        static SharedPtr create();

        void createCurveVao();
        void calculateCurvePatchBoundingBoxes();

        void createParticleVao();

        /** Create scene parameter block and retrieve pointers to buffers
        */
        void initResources();

        /** Uploads scene data to parameter block
        */
        void uploadResources();

        /** Uploads a single material.
        */
        void uploadMaterial(uint32_t materialID);

        /** Uploads the currently selected camera.
        */
        void uploadSelectedCamera();

        /** Update the scene's global bounding box.
        */
        void updateBounds();

        /** Update mesh instances.
        */
        void updateMeshInstances(bool forceUpdate);

        /** Do any additional initialization required after scene data is set and draw lists are determined.
        */
        void finalize(bool isReinit = false);

        /** Create the draw list for rasterization
        */
        void createDrawList();

        /** Sort meshes into groups by transform. Updates mMeshInstances and mMeshGroups.
        */
        void sortMeshes();

        /** Initialize geometry descs for each BLAS
        */
        void initGeomDesc();

        /** Generate bottom level acceleration structures for all meshes
        */
        void buildBlas(RenderContext* pContext);

        /** Generate data for creating a TLAS.
            #SCENE TODO: Add argument to build descs based off a draw list
        */
        void fillInstanceDesc(std::vector<D3D12_RAYTRACING_INSTANCE_DESC>& instanceDescs, uint32_t rayCount, bool perMeshHitEntry) const;

        /** Generate top level acceleration structure for the scene. Automatically determines whether to build or refit.
            \param[in] rayCount Number of ray types in the shader. Required to setup how instances index into the Shader Table
        */
        void buildTlas(RenderContext* pContext, uint32_t rayCount, bool perMeshHitEntry);

        /** Check whether scene has an index buffer.
        */
        bool hasIndexBuffer() const { return mpVao->getIndexBuffer() != nullptr; }

        /** Initialize all cameras in the scene through the animation controller using their corresponding scene graph nodes.
        */
        void initializeCameras();

        /** Prepare all UI-related objects that do not change over the course of execution.
        */
        void prepareUI();

        /** Update an animatable object.
        */
        bool updateAnimatable(Animatable& animatable, const AnimationController& controller, bool force = false);

        UpdateFlags updateSelectedCamera(bool forceUpdate);
        UpdateFlags updateLights(bool forceUpdate);
        UpdateFlags updateEnvMap(bool forceUpdate);
        UpdateFlags updateMaterials(bool forceUpdate);

        void updateGeometryStats();
        void updateRaytracingStats();
        void updateLightStats();

        struct SceneStats
        {
            // Geometry stats
            size_t uniqueTriangleCount = 0;     ///< Number of unique triangles. A triangle can exist in multiple instances.
            size_t uniqueVertexCount = 0;       ///< Number of unique vertices. A vertex can be referenced by multiple triangles/instances.
            size_t instancedTriangleCount = 0;  ///< Number of instanced triangles. This is the total number of rendered triangles.
            size_t instancedVertexCount = 0;    ///< Number of instanced vertices. This is the total number of vertices in the rendered triangles.

            // Raytracing stats
            size_t blasCount = 0;               ///< Number of BLASes.
            size_t blasCompactedCount = 0;      ///< Number of compacted BLASes.
            size_t blasMemoryInBytes = 0;       ///< Total memory in bytes used by the BLASes.

            // Light stats
            size_t activeLightCount = 0;        ///< Number of active lights.
            size_t totalLightCount = 0;         ///< Number of lights in the scene.
            size_t pointLightCount = 0;         ///< Number of point lights.
            size_t directionalLightCount = 0;   ///< Number of directional lights.
            size_t rectLightCount = 0;          ///< Number of rect lights.
            size_t sphereLightCount = 0;        ///< Number of sphere lights.
            size_t distantLightCount = 0;       ///< Number of distant lights.
        };

        Scene();

        // Scene Geometry
        Vao::SharedPtr mpParticleVao;
        Vao::SharedPtr mpCurveVao;
        Vao::SharedPtr mpVao;
        struct DrawArgs
        {
            Buffer::SharedPtr pBuffer;
            uint32_t count = 0;
        } mDrawClockwiseMeshes, mDrawCounterClockwiseMeshes;

        std::vector<CurveVertexData> mCPUCurveVertexBuffer;
        Buffer::SharedPtr mpCurveVertexBuffer;

        static const uint32_t kInvalidNode = -1;

        struct Node
        {
            Node() = default;
            Node(const std::string& n, uint32_t p, const glm::mat4& t, const glm::mat4& l2b) : parent(p), name(n), transform(t), localToBindSpace(l2b) {};
            std::string name;
            uint32_t parent = kInvalidNode;
            glm::mat4 transform;  // The node's transformation matrix
            glm::mat4 localToBindSpace; // Local to bind space transformation
        };

        struct MeshGroup
        {
            std::vector<uint32_t> meshList;     ///< List of meshId's that are part of the group.
        };

        // #SCENE We don't need those vectors on the host
        std::vector<MeshDesc> mMeshDesc;                            ///< Copy of mesh data GPU buffer (mpMeshes)
        std::vector<CurveDesc> mCurveDesc;                    ///< Copy of GPU buffer (mpCurves)
        std::vector<ParticleSystemDesc> mParticleSystemDesc;
        VolumeDesc mVolumeDesc;
        std::vector<VolumeDesc> mVolumeDescArray; // for animated volume sequence
        std::vector<MeshInstanceData> mMeshInstanceData;            ///< Mesh instance data.
        std::vector<uint2> mTriangleInstanceMapping;
        uint32_t mNumMeshInstances;
        uint32_t mNumTriangleInstances;
        std::vector<PackedMeshInstanceData> mPackedMeshInstanceData;///< Copy of packed mesh instance data GPU buffer (mpMeshInstances)
        std::vector<MeshGroup> mMeshGroups;                         ///< Groups of meshes with identical transforms. Each group maps to a BLAS for ray tracing.
        std::vector<Node> mSceneGraph;                              ///< For each index i, the array element indicates the parent node. Indices are in relation to mLocalToWorldMatrices

        std::vector<Material::SharedPtr> mMaterials;                ///< Bound to parameter block
        std::vector<Light::SharedPtr> mLights;                      ///< Bound to parameter block
        LightCollection::SharedPtr mpLightCollection;               ///< Bound to parameter block
        LightProbe::SharedPtr mpLightProbe;                         ///< Bound to parameter block
        EnvMap::SharedPtr mpEnvMap;                                 ///< Bound to parameter block

        // Scene Metadata (CPU Only)
        std::vector<BoundingBox> mMeshBBs;                          ///< Bounding boxes for meshes (not instances)
        std::vector<BoundingBox> mCurvePatchBBs;                    ///< Bounding boxes for curve patches
        std::vector<BoundingBox> mParticleSystemBBs;
        std::vector<BoundingBox> mVDBVolumeBBs;

        struct VDBBuffers
        {
            int numMips = 1;
            bool hasEmissionGrid = false; // the last grid is emission grid
            std::vector<Buffer::SharedPtr> kNodeLevel0s;
            std::vector<Buffer::SharedPtr> kNodeLevel1s;
            std::vector<Buffer::SharedPtr> kNodeLevel2s;
            std::vector<Buffer::SharedPtr> kRootData_roots;
            std::vector<Buffer::SharedPtr> kRootData_tiles;
            std::vector<Buffer::SharedPtr> kGridDatas;
        };


        struct GVDBInfo
        {
            static const int MAX_LEVELS = 3;
            static const int MAX_MIPS = 30; // mip 2*kNumMaxMips is reserved for temperature/emission, mip 2*kNumMaxMips+1 for velocity, mip 2*kNumMaxMips+2 for supervoxels
            // mip 2 * kNumMaxMips + 3 -- 10  mip 0-7 of previous frame,  mip 2 * kNumMaxMips + 11 temperature grid of previous frame,  2*kNumMaxMips + 12 velocity of previous frame
            static const int MAX_CHANNELS = 1;

            int			dim[MAX_LEVELS * MAX_MIPS]; // Log base 2 of lateral resolution of each node per level
            int			res[MAX_LEVELS * MAX_MIPS]; // Lateral resolution of each node per level
            float3		vdel[MAX_LEVELS * MAX_MIPS]; // How many voxels on a side a child of each level covers
            int3		noderange[MAX_LEVELS * MAX_MIPS]; // How many voxels on a side a node of each level covers
            int			nodecnt[MAX_LEVELS * MAX_MIPS]; // Total number of allocated nodes per level
            int			nodewid[MAX_LEVELS * MAX_MIPS]; // Size of a node at each level in bytes
            int			childwid[MAX_LEVELS * MAX_MIPS]; // Size of the child list per node at each level in bytes
            Buffer::SharedPtr nodelist[MAX_LEVELS * MAX_MIPS];
            Buffer::SharedPtr childlist[MAX_LEVELS * MAX_MIPS];
            //Buffer::SharedPtr atlas_map;
            //int3		atlas_cnt; // Number of bricks on each axis of the atlas
            //int3		atlas_res; // Total resolution in voxels of the atlas
            //int			atlas_apron; // Apron size
            //int			brick_res; // Resolution of a single brick
            //int			apron_table[8]; // Unused
            int			top_lev[MAX_MIPS]; // Top level (i.e. tree spans from voxels to level 0 to level top_lev)
            int			max_iter; // Unused
            float		epsilon; // Epsilon used for voxel ray tracing
            bool		update; // Whether this information needs to be updated from the latest volume data
            //uchar		clr_chan; // Index of the color channel for rendering color information
            uint        clr_chan;
            float3		bmin[MAX_MIPS]; // Inclusive minimum of axis-aligned bounding box in voxels
            float3		bmax[MAX_MIPS]; // Inclusive maximum of axis-aligned bounding box in voxels
            Texture::SharedPtr	volIn[MAX_CHANNELS * MAX_MIPS]; // Texture reference (read plus interpolation) to atlas per channel
            Texture::SharedPtr	volIn_part2[MAX_CHANNELS * MAX_MIPS]; //For depth slices that exceeds 2048
            Texture::SharedPtr  velocityIn[2];
            Texture::SharedPtr  velocityIn_part2[2];
            float superVoxelWorldSpaceDiagonalLength;
			int3 volInDimensions[MAX_CHANNELS * MAX_MIPS];
            int3 volInDimensions_part2[MAX_CHANNELS * MAX_MIPS];
            float3 invVolInDimensions[MAX_CHANNELS * MAX_MIPS];
            float3 invVolInDimensions_part2[MAX_CHANNELS * MAX_MIPS];
            float4x4 xform[MAX_MIPS];
            float4x4 invxform[MAX_MIPS];
            float4x4 invxrot[MAX_MIPS];
            float maxValue[MAX_MIPS];
            float invMaxValue[MAX_MIPS];
            float densityCompressScaleFactor[MAX_MIPS];

            void bindParameterBlock(ParameterBlock::SharedPtr block, int mipId);
            void bindPrevParameterBlock(ParameterBlock::SharedPtr block, int mipId);

        };

        struct GVDBParamBlocks
        {
            ParameterBlock::SharedPtr paramBlock;
            int numMips = 1;
            bool hasEmissionGrid = false; // the last grid is emission grid
            bool hasVelocityGrid = false;
        };

        public:

        // Don't use, this is only for single mip level volumes
        GVDBInfo getGVDBInfo(int volumeId)
        {
            return mGVDBInfos[volumeId];
        }

        bool mNewEnvMapLoaded = false;

        private:

        glm::mat4 computeVolumeExternalModelToWorldMatrix()
        {
            glm::mat4 externalModelToWorldMatrix;
            externalModelToWorldMatrix = glm::translate(externalModelToWorldMatrix, mVolumeWorldTranslation);
            externalModelToWorldMatrix = glm::rotate(externalModelToWorldMatrix, glm::radians(mVolumeWorldRotation.x), float3(1, 0, 0));
            externalModelToWorldMatrix = glm::rotate(externalModelToWorldMatrix, glm::radians(mVolumeWorldRotation.y), float3(0, 1, 0));
            externalModelToWorldMatrix = glm::rotate(externalModelToWorldMatrix, glm::radians(mVolumeWorldRotation.z), float3(0, 0, 1));
            externalModelToWorldMatrix = glm::scale(externalModelToWorldMatrix, float3(mVolumeWorldScaling));
            return externalModelToWorldMatrix;
        }

        std::vector<GVDBParamBlocks> mGVDBVolumes;
        std::vector<GVDBInfo> mGVDBInfos;
		std::vector<float4> mCPUBlackBodyRadiationTexture;
        Texture::SharedPtr mpBlackBodyRadiationTexture;

        std::vector<VDBBuffers> mVDBVolumes;

        std::vector<std::vector<uint32_t>> mMeshIdToInstanceIds;    ///< Mapping of what instances belong to which mesh
        BoundingBox mSceneBB;                                       ///< Bounding boxes of the entire scene
		BoundingBox mSceneVolumeBB;                                 ///< Bounding boxes of volumes in the scene
        std::vector<bool> mMeshHasDynamicData;                      ///< Whether a Mesh has dynamic data, meaning it is skinned
        SceneStats mSceneStats;                                     ///< Scene statistics.
        RenderSettings mRenderSettings;                             ///< Render settings.
        RenderSettings mPrevRenderSettings;

        // Resources
        Buffer::SharedPtr mpMeshesBuffer;
        Buffer::SharedPtr mpCurvesBuffer;
        Buffer::SharedPtr mpParticleSystemsBuffer;
        public:
        float3 mVolumeWorldTranslation = float3(0,0,0);
        float3 mVolumeWorldRotation = float3(0, 0, 0);
        float mVolumeWorldScaling = 1.f;
        float mEmissiveIntensityMultiplier = 1.f;

        bool mUseAnimatedVolume = false;
        int mVDBAnimationFrameId = 0;
        int mVDBLastAnimationFrameId = 0;
        int mVDBAnimationFrames = 0;
        bool mPauseVDBAnimation = false;

        private:
        Buffer::SharedPtr mpMeshInstancesBuffer;
        Buffer::SharedPtr mpTriangleInstanceMappingBuffer;
        Buffer::SharedPtr mpMaterialsBuffer;
        Buffer::SharedPtr mpLightsBuffer;
        ParameterBlock::SharedPtr mpSceneBlock;

		//
        float mSurfaceGlobalAlphaMultipler = 1.f;
        float mParticleCurveGlobalAlphaMultipler = 1.f;

        float mPrevSurfaceGlobalAlphaMultipler = 1.f;
        float mPrevParticleCurveGlobalAlphaMultipler = 1.f;

        // particle data

        std::vector<ParticleSystem::SharedPtr> mParticleSystems;
        Buffer::SharedPtr mpParticleAABBBuffer; //for procedural particles
        ParticleSystemManager mParticleSystemManager;
        public:
            bool mNewParticleSystemAdded = false;
        private:
            uint32_t mTotalParticles = 0;
            double mLastParticleTime = 0;

        // Curve data
        CurveFileLoader mCurveFileLoader;
        Buffer::SharedPtr mpCurvePatchAABBBuffer;
        float mCurveDisplayWidthMultiplier = 1.f;
        bool mHasCurveDisplayWidthChanged = false;

        // Camera
        CameraControllerType mCamCtrlType = CameraControllerType::FirstPerson;
        public:
        CameraController::SharedPtr mpCamCtrl;
        private:
        std::vector<Camera::SharedPtr> mCameras;
        uint32_t mSelectedCamera = 0;
        float mCameraSpeed = 1.0f;
        bool mCameraSwitched = false;

        Gui::DropdownList mCameraList;

        // Saved Camera Viewpoints
        struct Viewpoint
        {
            uint32_t index;
            float3 position;
            float3 target;
            float3 up;
        };
        std::vector<Viewpoint> mViewpoints;
        uint32_t mCurrentViewpoint = 0;

        // Rendering
        RasterizerState::SharedPtr mpFrontClockwiseRS;
        UpdateFlags mUpdates = UpdateFlags::All;
        AnimationController::UniquePtr mpAnimationController;

        // Raytracing Data
        UpdateMode mTlasUpdateMode = UpdateMode::Rebuild;   ///< How the TLAS should be updated when there are changes in the scene
        UpdateMode mBlasUpdateMode = UpdateMode::Refit;     ///< How the BLAS should be updated when there are changes to meshes

        std::vector<D3D12_RAYTRACING_INSTANCE_DESC> mInstanceDescs; ///< Shared between TLAS builds to avoid reallocating CPU memory

        struct TlasData
        {
            Buffer::SharedPtr pTlas;
            ShaderResourceView::SharedPtr pSrv;             ///< Shader Resource View for binding the TLAS
            Buffer::SharedPtr pInstanceDescs;               ///< Buffer holding instance descs for the TLAS
            UpdateMode updateMode = UpdateMode::Rebuild;    ///< Update mode this TLAS was created with.
        };

        std::unordered_map<uint32_t, TlasData> mTlasCache;  ///< Top Level Acceleration Structure for scene data cached per shader ray count
                                                            ///< Number of ray types in program affects Shader Table indexing
        Buffer::SharedPtr mpTlasScratch;                    ///< Scratch buffer used for TLAS builds. Can be shared as long as instance desc count is the same, which for now it is.
        D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO mTlasPrebuildInfo; ///< This can be reused as long as the number of instance descs doesn't change.

        struct BlasData
        {
            D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO prebuildInfo;
            D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS buildInputs;
            std::vector<D3D12_RAYTRACING_GEOMETRY_DESC> geomDescs;

            uint64_t blasByteSize = 0;                      ///< Size of the final BLAS.
            uint64_t blasByteOffset = 0;                    ///< Offset into the BLAS buffer to where it is stored.
            uint64_t scratchByteOffset = 0;                 ///< Offset into the scratch buffer to use for updates/rebuilds.

            bool hasSkinnedMesh = false;                    ///< Whether the BLAS contains a skinned mesh, which means the BLAS may need to be updated.
            bool useCompaction = false;                     ///< Whether the BLAS should be compacted after build.
            UpdateMode updateMode = UpdateMode::Refit;      ///< Update mode this BLAS was created with.
        };

        std::vector<BlasData> mBlasData;    ///< All data related to the scene's BLASes.
        Buffer::SharedPtr mpBlas;           ///< Buffer containing all BLASes.
        Buffer::SharedPtr mpBlasScratch;    ///< Scratch buffer used for BLAS builds.
        bool mRebuildBlas = true;           ///< Flag to indicate BLASes need to be rebuilt.
        bool mHasSkinnedMesh = false;       ///< Whether the scene has a skinned mesh at all.

        std::string mFilename;
    };

    enum_class_operators(Scene::RenderFlags);
    enum_class_operators(Scene::UpdateFlags);
}

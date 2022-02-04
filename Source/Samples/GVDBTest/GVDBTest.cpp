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
#include "GVDBTest.h"
#include <gvdb_volume_gvdb.h>
#include <gvdb_scene.h>

uint32_t mSampleGuiWidth = 250;
uint32_t mSampleGuiHeight = 200;
uint32_t mSampleGuiPositionX = 20;
uint32_t mSampleGuiPositionY = 40;

void GVDBTest::onGuiRender(Gui* pGui)
{
    Gui::Window w(pGui, "Falcor", { 250, 200 });
    gpFramework->renderGlobalUI(pGui);
    w.text("Hello from GVDBTest");
    if (w.button("Click Here"))
    {
        msgBox("Now why would you do that?");
    }
    if (mpScene) mpScene->renderUI(w);

    if (auto logGroup = w.group("Logging"))
    {
        // Pixel debugger.
        mpPixelDebug->renderUI(logGroup);
    }
}

ComputePass::SharedPtr createSimpleComputePass(const std::string& file, const std::string& mainEntry,
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
    return ComputePass::create(risDesc.csEntry(mainEntry), matlDefs);
}

void GVDBTest::onLoad(RenderContext* pRenderContext)
{
    mpSceneBuilder = SceneBuilder::create("default.obj");
    //mpSceneBuilder->loadKnitCCPFile(filename, 10.f);
    mpScene = mpSceneBuilder->getScene();

    //mpScene->addGVDBVolume(float3(10.f), float3(90.f), 0, "E:/Graphics/gvdb-voxels/source/shared_assets/explosion.vbx");
    //mpScene->addGVDBVolume(float3(10.f), float3(90.f), 0, "E:/Graphics/gvdb-voxels/_output/bin/bunny_cloud.vbx");

    mpScene->setCameraController(Falcor::Scene::CameraControllerType::FirstPerson);
    if (!mpScene) return;

    mpCamera = mpScene->getCamera();
    mpCamera->setPosition(float3(47.614, 42.340, 33.321));
    mpCamera->setTarget(float3(46.847, 41.904, 32.851));
    mpCamera->setUpVector(float3(0, 1, 0));

    mComputePass = createSimpleComputePass("Samples/GVDBTest/rayDeep.cs.slang", "main", {});
    mpPixelDebug = PixelDebug::create();

    BoundingBox bb = mpScene->getVDBBoundingBox(0);
    auto wBboxSize = bb.extent;

    //m_pretrans.Set(-125, -160, -125);
    //m_scale.Set(1, 1, 1);
    //m_angs.Set(0, 0, 0);
    //m_trans.Set(0, 0, 0);

    // Set volume params
    gvdb.mCpuOnly = true;
    //gvdb.SetTransform(m_pretrans, m_scale, m_angs, m_trans);
    gvdb.mScene = new nvdb::Scene();
    //gvdb.getScene()->SetSteps(.25f, 16, .25f);			// Set raycasting steps
    //gvdb.getScene()->SetExtinct(-1.0f, 1.0f, 0.0f);		// Set volume extinction
    //gvdb.getScene()->SetVolumeRange(0.1f, 0.0f, .5f);	// Set volume value range
    //gvdb.getScene()->SetCutoff(0.005f, 0.005f, 0.0f);
    //gvdb.getScene()->SetBackgroundClr(0.1f, 0.2f, 0.4f, 1.0);

    Camera3D* cam = new Camera3D;
    cam->setFov(120.f);
    //cam->setOrbit(Vector3DF(20, 30, 0), Vector3DF(0, 0, 0), 700, 1.0);

    cam->setPos(-11.513717f, 29.613581f, 50.603836f);
    cam->setToPos(-11.452119f, 29.373362f, 49.635075f);

    // Create Light
    Camera3D* lgt = new Camera3D;
    lgt->setOrbit(Vector3DF(299, 57.3f, 0), Vector3DF(132, -20, 50), 200, 1.0);
    gvdb.getScene()->SetLight(0, lgt);

    mScnInfo.camnear = cam->getNear();
    mScnInfo.camfar = cam->getFar();
    mScnInfo.campos = float3(cam->origRayWorld.x, cam->origRayWorld.y, cam->origRayWorld.z);
    mScnInfo.cams = float3(cam->tlRayWorld.x, cam->tlRayWorld.y, cam->tlRayWorld.z);
    mScnInfo.camu = float3(cam->trRayWorld.x, cam->trRayWorld.y, cam->trRayWorld.z); mScnInfo.camu -= mScnInfo.cams;
    mScnInfo.camv = float3(cam->blRayWorld.x, cam->blRayWorld.y, cam->blRayWorld.z); mScnInfo.camv -= mScnInfo.cams;
    gvdb.getScene()->SetCamera(cam);
    gvdb.getScene()->SetRes(1920, 1080);

    mScnInfo.cams = float3(cam->tlRayWorld.x, cam->tlRayWorld.y, cam->tlRayWorld.z);
    mScnInfo.camu = float3(cam->trRayWorld.x, cam->trRayWorld.y, cam->trRayWorld.z); mScnInfo.camu -= mScnInfo.cams;
    mScnInfo.camv = float3(cam->blRayWorld.x, cam->blRayWorld.y, cam->blRayWorld.z); mScnInfo.camv -= mScnInfo.cams;

    nvdb::Matrix4F xform = gvdb.getTransform();
    nvdb::Matrix4F invXform = gvdb.getInvTransform();
    nvdb::Matrix4F invRot = gvdb.getInvRot();
    //mScnInfo.bias = m_bias;
    // Transform the light position from application space to voxel space,
    // like getViewPos:
    nvdb::Vector4DF lightPos4 = gvdb.getScene()->getLight()->getPos();
    lightPos4.w = 1.0f; // Since it's a position, not a direction
    lightPos4 *= invXform;
    mScnInfo.light_pos = float3(lightPos4.x, lightPos4.y, lightPos4.z);
    Vector3DF crs = Vector3DF(0, 0, 0);
    //mScnInfo.slice_pnt = gvdb.getScene()->getSectionPnt();
    //mScnInfo.slice_norm = gvdb.getScene()->getSectionNorm();
    //mScnInfo.shading = shading;				// getScene()->getShading();	
    //mScnInfo.filtering = getScene()->getFilterMode();
    //mScnInfo.frame = getScene()->getFrame();
    //mScnInfo.samples = getScene()->getSample();
    //mScnInfo.shadow_params = getScene()->getShadowParams();
    mScnInfo.backclr = float4(0.1f, 0.2f, 0.4f, 1.0);
    mScnInfo.extinct = float3(-1.0f, 1.0f, 0.0f);
    mScnInfo.steps = float3(.25f, 16, .25f);
    mScnInfo.cutoff = float3(0.005f, 0.005f, 0.0f);
    mScnInfo.thresh = float3(0.1f, 0.0f, .5f);
    // Grid transform
    memcpy((void*)&mScnInfo.xform, &xform, sizeof(float) * 16);
    memcpy((void*)&mScnInfo.invxform, &invXform, sizeof(float) * 16);
    memcpy((void*)&mScnInfo.invxrot, &invRot, sizeof(float) * 16);

    Sampler::Desc samplerDesc;
    samplerDesc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Linear);
    samplerDesc.setBorderColor(float4(0.f));
    samplerDesc.setAddressingMode(Sampler::AddressMode::Border, Sampler::AddressMode::Border, Sampler::AddressMode::Border);
    mpLinearSampler = Sampler::create(samplerDesc);

    samplerDesc.setFilterMode(Sampler::Filter::Point, Sampler::Filter::Point, Sampler::Filter::Point);
    mpPointSampler = Sampler::create(samplerDesc);
}

void GVDBTest::onFrameRender(RenderContext* pRenderContext, const Fbo::SharedPtr& pTargetFbo)
{
    const float4 clearColor(0.38f, 0.52f, 0.10f, 1);
    pRenderContext->clearFbo(pTargetFbo.get(), clearColor, 1.0f, 0, FboAttachmentType::All);
    //m_angs.y += 0.05f;
    //gvdb.SetTransform(m_pretrans, m_scale, m_angs, m_trans);
    nvdb::Matrix4F xform = gvdb.getTransform();
    nvdb::Matrix4F invXform = gvdb.getInvTransform();
    nvdb::Matrix4F invRot = gvdb.getInvRot();
    memcpy((void*)&mScnInfo.xform, &xform, sizeof(float) * 16);
    memcpy((void*)&mScnInfo.invxform, &invXform, sizeof(float) * 16);
    memcpy((void*)&mScnInfo.invxrot, &invRot, sizeof(float) * 16);

    uint32_t scrWidth = pTargetFbo->getWidth();
    uint32_t scrHeight = pTargetFbo->getHeight();
    mScnInfo.width = scrWidth;
    mScnInfo.height = scrHeight;

    mpPixelDebug->beginFrame(pRenderContext, uint2(scrWidth, scrHeight));

    Falcor::Scene::UpdateFlags flags = mpScene->update(pRenderContext, gpFramework->getGlobalClock().getTime());

    if (!mpOutputTex || mpOutputTex->getWidth() != scrWidth || mpOutputTex->getHeight() != scrHeight)
    {
        mpOutputTex = Texture::create2D(scrWidth, scrHeight, ResourceFormat::RGBA32Float, 1, 1, nullptr, ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess);
    }

    auto vars = mComputePass->getRootVar();

    mpScene->setGVDBShaderData(vars, 0);
    mScnInfo.setShaderData(vars["CB"]["scn"]);
    vars["CB"]["gVolInDimensions"] = mpScene->getGVDBInfo(0).volInDimensions[0];
    vars["kOutImage"] = mpOutputTex;
    vars["gLinearSampler"] = mpLinearSampler;
    vars["gPointSampler"] = mpPointSampler;

    mpPixelDebug->prepareProgram(mComputePass->getProgram(), vars);

    mComputePass->execute(pRenderContext, scrWidth, scrHeight);
    pRenderContext->blit(mpOutputTex->getSRV(), pTargetFbo->getRenderTargetView(0));

    mpPixelDebug->endFrame(pRenderContext);
}

void GVDBTest::onShutdown()
{
}

bool GVDBTest::onKeyEvent(const KeyboardEvent& keyEvent)
{
    if (mpScene && mpScene->onKeyEvent(keyEvent)) return true;
    return false;
}

bool GVDBTest::onMouseEvent(const MouseEvent& mouseEvent)
{
    bool ret = mpScene ? mpScene->onMouseEvent(mouseEvent) : false;
    ret |= mpPixelDebug->onMouseEvent(mouseEvent);
    return ret;
}

void GVDBTest::onHotReload(HotReloadFlags reloaded)
{
}

void GVDBTest::onResizeSwapChain(uint32_t width, uint32_t height)
{
}

//int WINAPI WinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPSTR lpCmdLine, _In_ int nShowCmd)
int wmain(int argc, wchar_t** argv)
{
    GVDBTest::UniquePtr pRenderer = std::make_unique<GVDBTest>();
    SampleConfig config;
    config.windowDesc.title = "Falcor Project Template";
    config.windowDesc.resizableWindow = true;
    Sample::run(config, pRenderer);
    return 0;
}

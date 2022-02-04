/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
#include "OptixDenoiser.h"
#include "CudaUtils.h"

namespace
{
    const char kDesc[] = "Apply the OptiX AI Denoiser";

    // Names for pass input and output textures
    const char kColorInput[] = "color";
    const char kAlbedoInput[] = "albedo";
    const char kNormalInput[] = "normal";
    const char kMotionInput[] = "mvec";
    const char kOutput[] = "output";

    const char kEnabled[] = "enabled";
    const char kBlend[] = "blend";
    const char kDenoiseAlpha[] = "denoiseAlpha";

    const std::string kConvertTexToBufFile = "RenderPasses/OptixDenoiser/ConvertTexToBuf.cs.slang";
    const std::string kConvertMotionVecFile = "RenderPasses/OptixDenoiser/ConvertMotionVectorInputs.cs.slang";
    const std::string kConvertBufToTexFile = "RenderPasses/OptixDenoiser/ConvertBufToTex.ps.slang";

    const Falcor::Resource::BindFlags   kSharedBufferFlags = Resource::BindFlags::ShaderResource | Resource::BindFlags::UnorderedAccess | Resource::BindFlags::RenderTarget | Resource::BindFlags::Shared;
};

static void regOptixDenoiser(pybind11::module& m)
{
    pybind11::class_<OptixDenoiserPass, RenderPass, OptixDenoiserPass::SharedPtr> pass(m, "OptixDenoiserPass");
    pass.def_property(kEnabled, &OptixDenoiserPass::getEnabled, &OptixDenoiserPass::setEnabled);

    /*
    pybind11::enum_<OptixDenoiserPass::Model> model(m, "OptixDenoiserModel");
    model.value("LDR", OptixDenoiserPass::Model::LDR);
    model.value("HDR", OptixDenoiserPass::Model::HDR);
    */
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("OptixDenoiser", kDesc, OptixDenoiserPass::create);
    ScriptBindings::registerBinding(regOptixDenoiser);
}

OptixDenoiserPass::OptixDenoiserPass(const Dictionary& dict)
{
    for (const auto& [key, value] : dict)
    {
        if (key == kEnabled) mEnabled = value;
        else if (key == kBlend) mDenoiser.params.blendFactor = value;
        else if (key == kDenoiseAlpha) mDenoiser.params.denoiseAlpha = (value ? 1u : 0u);
        else logWarning("Unknown field '" + key + "' in a OptixDenoiserPass dictionary");
    }

    mpConvertTexToBuf = ComputePass::create(kConvertTexToBufFile, "main");
    mpConvertMotionVectors = ComputePass::create(kConvertMotionVecFile, "main");
    mpConvertBufToTex = FullScreenPass::create(kConvertBufToTexFile);
    mpFbo = Fbo::create();
}

OptixDenoiserPass::SharedPtr OptixDenoiserPass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    return SharedPtr(new OptixDenoiserPass(dict));
}

std::string OptixDenoiserPass::getDesc() { return kDesc; }

Dictionary OptixDenoiserPass::getScriptingDictionary()
{
    Dictionary d;
    d[kEnabled] = mEnabled;
    d[kBlend] = mDenoiser.params.blendFactor;
    d[kDenoiseAlpha] = bool(mDenoiser.params.denoiseAlpha > 0);

    return d;
}

RenderPassReflection OptixDenoiserPass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection r;
    r.addInput(kColorInput, "Color input");
    r.addInput(kAlbedoInput, "Albedo input").flags(RenderPassReflection::Field::Flags::Optional);
    r.addInput(kNormalInput, "Normal input").flags(RenderPassReflection::Field::Flags::Optional);
    r.addInput(kMotionInput, "Motion vector input").flags(RenderPassReflection::Field::Flags::Optional);
    r.addOutput(kOutput, "Denoised output").format(ResourceFormat::RGBA32Float);   
    return r;
}

void OptixDenoiserPass::compile(RenderContext* pContext, const CompileData& compileData)
{
    if (!initializeOptix()) { return; }

    // Determine available inputs
    mHasColorInput = (compileData.connectedResources.getField(kColorInput) != nullptr);
    mHasAlbedoInput = (compileData.connectedResources.getField(kAlbedoInput) != nullptr);
    mHasNormalInput = (compileData.connectedResources.getField(kNormalInput) != nullptr);
    mHasMotionInput = (compileData.connectedResources.getField(kMotionInput) != nullptr);

    // Set correct parameters for the provided inputs.
    mDenoiser.options.guideNormal = mHasAlbedoInput ? 1u : 0u;
    mDenoiser.options.guideAlbedo = mHasNormalInput ? 1u : 0u;

    // Create a dropdown menu for selecting the denoising mode
    mModelChoices = {};
    mModelChoices.push_back({ OPTIX_DENOISER_MODEL_KIND_LDR, "LDR denoising" });
    mModelChoices.push_back({ OPTIX_DENOISER_MODEL_KIND_HDR, "HDR denoising" });
    if (mHasMotionInput)
    {
        mModelChoices.push_back({ OPTIX_DENOISER_MODEL_KIND_TEMPORAL, "Temporal denoising" });
    }

    // (Re-)allocate temporary buffers when render resolution changes
    uint2 newSize = compileData.defaultTexDims;

    // If allowing tiled denoising, these may be smaller than the window size (TODO; not currently handled)
    mDenoiser.tileWidth = newSize.x;
    mDenoiser.tileHeight = newSize.y;

    // Reallocate / reszize our staging buffers for transferring data to and from OptiX / CUDA / DXR
    if (newSize != mBufferSize && newSize.x > 0 && newSize.y > 0)
    {
        reallocateStagingBuffers(pContext, newSize);
    }

    // Resize intensity and hdrAverage buffers.
    mDenoiser.intensityBuffer.resize(1 * sizeof(float));
    mDenoiser.hdrAverageBuffer.resize(3 * sizeof(float));

    // Create an intensity GPU buffer to pass to OptiX when appropriate
    if (!mDenoiser.kernelPredictionMode || !mDenoiser.useAOVs)
    {
        mDenoiser.params.hdrIntensity = mDenoiser.intensityBuffer.getDevicePtr();
        mDenoiser.params.hdrAverageColor = static_cast<CUdeviceptr>(0);
    } 
    else  // Create an HDR average color GPU buffer to pass to OptiX when appropriate
    {
        mDenoiser.params.hdrIntensity = static_cast<CUdeviceptr>(0);
        mDenoiser.params.hdrAverageColor = mDenoiser.hdrAverageBuffer.getDevicePtr();
    }

    mRecreateDenoiser = true;
}

void OptixDenoiserPass::reallocateStagingBuffers(RenderContext* pContext, uint2 newSize)
{
    mBufferSize = newSize;

    // Allocate buffer for our noisy inputs to the denoiser
    allocateStagingBuffer(pContext, mDenoiser.interop.denoiserInput, mDenoiser.layer.input);

    // Allocate buffer for our denoised outputs from the denoiser
    allocateStagingBuffer(pContext, mDenoiser.interop.denoiserOutput, mDenoiser.layer.output);

    // Allocate a guide buffer for our normals (if necessary)
    if (mDenoiser.options.guideNormal > 0)
        allocateStagingBuffer(pContext, mDenoiser.interop.normal, mDenoiser.guideLayer.normal);
    else
        freeStagingBuffer(mDenoiser.interop.normal, mDenoiser.guideLayer.normal);

    // Allocate a guide buffer for our albedo (if necessary)
    if (mDenoiser.options.guideAlbedo > 0)
        allocateStagingBuffer(pContext, mDenoiser.interop.albedo, mDenoiser.guideLayer.albedo);
    else
        freeStagingBuffer(mDenoiser.interop.albedo, mDenoiser.guideLayer.albedo);

    // Allocate a guide buffer for our motion vectors (if necessary)
    if (mHasMotionInput) // i.e., if using temporal denoising
        allocateStagingBuffer(pContext, mDenoiser.interop.motionVec, mDenoiser.guideLayer.flow, OPTIX_PIXEL_FORMAT_FLOAT2);
    else
        freeStagingBuffer(mDenoiser.interop.motionVec, mDenoiser.guideLayer.flow);
}

void OptixDenoiserPass::allocateStagingBuffer(RenderContext* pContext, Interop& interop, OptixImage2D& image, OptixPixelFormat format)
{
    uint32_t elemSize = 4 * sizeof(float);
    ResourceFormat falcorFormat = ResourceFormat::RGBA32Float;
    switch (format)
    {
    case OPTIX_PIXEL_FORMAT_FLOAT4:
        elemSize = 4 * sizeof(float);
        falcorFormat = ResourceFormat::RGBA32Float;
        break;
    case OPTIX_PIXEL_FORMAT_FLOAT2:
        elemSize = 2 * sizeof(float);
        falcorFormat = ResourceFormat::RG32Float;
        break;
    default:
        logError("OptixDenoiserPass called allocateStagingBuffer() with unsupported format!");
        return;
    }

    if (interop.devicePtr) freeSharedDevicePtr((void*)interop.devicePtr);

    interop.buffer = Buffer::createTyped(falcorFormat, mBufferSize.x * mBufferSize.y, kSharedBufferFlags);
    interop.devicePtr = (CUdeviceptr)exportBufferToCudaDevice(interop.buffer);

    image.width = mBufferSize.x;
    image.height = mBufferSize.y;
    image.rowStrideInBytes = mBufferSize.x * elemSize;
    image.pixelStrideInBytes = elemSize;
    image.format = format;  
    image.data = interop.devicePtr;
}

void OptixDenoiserPass::freeStagingBuffer(Interop& interop, OptixImage2D& image)
{
    if (interop.devicePtr) freeSharedDevicePtr((void*)interop.devicePtr);
    interop.buffer = nullptr;
    image.data = static_cast<CUdeviceptr>(0);
}

void OptixDenoiserPass::execute(RenderContext* pContext, const RenderData& data)
{
    if (mEnabled)
    {
        if (mRecreateDenoiser)
        {
            // Sanity checking.  Do not attempt to use temporal denoising without appropriate inputs!
            //     If so set, reset model to something sensible.
            if (!mHasMotionInput && mDenoiser.modelKind == OptixDenoiserModelKind::OPTIX_DENOISER_MODEL_KIND_TEMPORAL)
            {
                mSelectedModel = OptixDenoiserModelKind::OPTIX_DENOISER_MODEL_KIND_HDR;
                mDenoiser.modelKind = OptixDenoiserModelKind::OPTIX_DENOISER_MODEL_KIND_HDR;
            }

            // Setup or recreate our denoiser
            setupDenoiser();
            mRecreateDenoiser = false;
            mIsFirstFrame = true;
        }

        // Copy input textures to buffers
        convertTexToBuf(pContext, data[kColorInput]->asTexture(), mDenoiser.interop.denoiserInput.buffer, mBufferSize);
        if (mDenoiser.options.guideAlbedo) convertTexToBuf(pContext, data[kAlbedoInput]->asTexture(), mDenoiser.interop.albedo.buffer, mBufferSize);
        if (mDenoiser.options.guideNormal) convertTexToBuf(pContext, data[kNormalInput]->asTexture(), mDenoiser.interop.normal.buffer, mBufferSize);
        if (mDenoiser.modelKind == OptixDenoiserModelKind::OPTIX_DENOISER_MODEL_KIND_TEMPORAL)
        {
            convertMotionVectors(pContext, data[kMotionInput]->asTexture(), mDenoiser.interop.motionVec.buffer, mBufferSize);
        }

        // TODO: Find a better way to synchronize
        // https://gitlab-master.nvidia.com/nvresearch-gfx/Tools/Falcor/issues/772
        pContext->flush(true);

        // Compute average intensity, if needed
        if (mDenoiser.params.hdrIntensity)
        {
            optixDenoiserComputeIntensity(
                mDenoiser.denoiser,
                nullptr, // CUDA stream
                &mDenoiser.layer.input,
                mDenoiser.params.hdrIntensity,
                mDenoiser.scratchBuffer.getDevicePtr(),
                mDenoiser.scratchBuffer.getSize()
            );
        }

        // Compute average color, if needed
        if (mDenoiser.params.hdrAverageColor)
        {
            optixDenoiserComputeAverageColor(
                mDenoiser.denoiser,
                nullptr, // CUDA stream
                &mDenoiser.layer.input,
                mDenoiser.params.hdrAverageColor,
                mDenoiser.scratchBuffer.getDevicePtr(),
                mDenoiser.scratchBuffer.getSize()
            );
        }

        // On the first frame with a new denoiser, we have no prior input for temporal denoising.
        //    In this case, pass in our current frame as both the current and prior frame.
        if (mIsFirstFrame)
        {
            mDenoiser.layer.previousOutput = mDenoiser.layer.input;
        }

        // Run denoiser
        optixDenoiserInvoke(mDenoiser.denoiser,
            nullptr,                 // CUDA stream
            &mDenoiser.params,
            mDenoiser.stateBuffer.getDevicePtr(), mDenoiser.stateBuffer.getSize(),
            &mDenoiser.guideLayer,   // Our set of normal / albedo / motion vector guides
            &mDenoiser.layer,        // Array of input or AOV layers (also contains denoised per-layer outputs)
            1u,                      // Nuumber of layers in the above array
            0u,                      // (Tile) Input offset X
            0u,                      // (Tile) Input offset Y
            mDenoiser.scratchBuffer.getDevicePtr(), mDenoiser.scratchBuffer.getSize());

        // Copy output buffer to texture for Falcor to consume
        convertBufToTex(pContext, mDenoiser.interop.denoiserOutput.buffer, data[kOutput]->asTexture(), mBufferSize);

        // Make sure we set the previous frame output to the correct location for future frames.
        //    Everything in this if() cluase could happen every frame, but is redundant after the first frame.
        if (mIsFirstFrame)
        {
            // Note: This is a deep copy that can dangerously point to deallocated memory when resetting denoiser settings.
            //     This is (partly) why in the first frame, the layer.previousOutput is set to layer.input (above).
            mDenoiser.layer.previousOutput = mDenoiser.layer.output;

            // We're no longer in the first frame of denoising; no special processing needed now.
            mIsFirstFrame = false;
        }
    }
    else
    {
        pContext->blit(data[kColorInput]->asTexture()->getSRV(), data[kOutput]->asTexture()->getRTV());
    }
}

void OptixDenoiserPass::renderUI(Gui::Widgets& widget)
{
    widget.checkbox("Use OptiX Denoiser?", mEnabled);

    if (mEnabled)
    {
        if (widget.dropdown("Model", mModelChoices, mSelectedModel))
        {
            mDenoiser.modelKind = static_cast<OptixDenoiserModelKind>(mSelectedModel);
            mRecreateDenoiser = true;
        }
        widget.tooltip("Selects the OptiX denosing model.  See OptiX documentation for details.\n\nFor best results:\n   LDR assumes inputs [0..1]\n   HDR assumes inputs [0..10,000]");

        if (mHasAlbedoInput)
        {
            bool useAlbedoGuide = mDenoiser.options.guideAlbedo != 0u;
            if (widget.checkbox("Use albedo guide?", useAlbedoGuide))
            {
                mDenoiser.options.guideAlbedo = useAlbedoGuide ? 1u : 0u;
                mRecreateDenoiser = true;
            }
            widget.tooltip("Use input, noise-free albedo channel to help guide denoising.");
        }

        if (mHasNormalInput)
        {
            bool useNormalGuide = mDenoiser.options.guideNormal != 0u;
            if (widget.checkbox("Use normal guide?", useNormalGuide))
            {
                mDenoiser.options.guideNormal = useNormalGuide ? 1u : 0u;
                mRecreateDenoiser = true;
            }
            widget.tooltip("Use input, noise-free normal buffer to help guide denoising.  (Note: The Optix 7.3 API is unclear on this point, but, correct use of normal guides appears to also require using an albedo guide.)");
        }

        {
            bool denoiseAlpha = mDenoiser.params.denoiseAlpha != 0;
            if (widget.checkbox("Denoise Alpha?", denoiseAlpha))
            {
                mDenoiser.params.denoiseAlpha = denoiseAlpha ? 1u : 0u;
            }
            widget.tooltip("Enable denoising the alpha channel, not just RGB.");
        }

        widget.slider("Blend", mDenoiser.params.blendFactor, 0.f, 1.f);
        widget.tooltip("Blend between denoised and original input. (0 = denoised only, 1 = noisy only)");
    }
}

void * OptixDenoiserPass::exportBufferToCudaDevice(Buffer::SharedPtr &buf)
{
    if (buf == nullptr) return nullptr;
    return getSharedDevicePtr(buf->createSharedApiHandle(), (uint32_t)buf->getSize());
}

bool OptixDenoiserPass::initializeOptix()
{
    if (!mOptixInitialized) mOptixInitialized = initOptix(mOptixContext) >= 0;

    return mOptixInitialized;
}

void OptixDenoiserPass::setupDenoiser()
{
    // Destroy the denoiser, if it already exists
    if (mDenoiser.denoiser)
    {
        optixDenoiserDestroy(mDenoiser.denoiser);
    }

    // Create the denoiser
    optixDenoiserCreate(mOptixContext,
        mDenoiser.modelKind,
        &mDenoiser.options,
        &mDenoiser.denoiser);

    // Find out how much memory is needed for the requested denoiser
    optixDenoiserComputeMemoryResources(mDenoiser.denoiser, mDenoiser.tileWidth, mDenoiser.tileHeight, &mDenoiser.sizes);

    // Allocate/resize some temporary CUDA buffers for internal OptiX processing/state
    mDenoiser.scratchBuffer.resize(mDenoiser.sizes.withoutOverlapScratchSizeInBytes);
    mDenoiser.stateBuffer.resize(mDenoiser.sizes.stateSizeInBytes);

    // Finish setup of the denoiser
    optixDenoiserSetup(mDenoiser.denoiser,
        nullptr,
        mDenoiser.tileWidth + 2 * mDenoiser.tileOverlap,   // Should work with tiling if parameters set appropriately 
        mDenoiser.tileHeight + 2 * mDenoiser.tileOverlap,  // Should work with tiling if parameters set appropriately 
        mDenoiser.stateBuffer.getDevicePtr(), mDenoiser.stateBuffer.getSize(),
        mDenoiser.scratchBuffer.getDevicePtr(), mDenoiser.scratchBuffer.getSize());
}

void OptixDenoiserPass::convertMotionVectors(RenderContext* pContext, const Texture::SharedPtr& tex, const Buffer::SharedPtr& buf, const uint2& size)
{
    auto vars = mpConvertMotionVectors->getVars();
    vars["GlobalCB"]["gStride"] = size.x;
    vars["GlobalCB"]["gSize"] = size;
    vars["gInTex"] = tex;
    vars["gOutBuf"] = buf;
    mpConvertMotionVectors->execute(pContext, size.x, size.y);
}

void OptixDenoiserPass::convertTexToBuf(RenderContext* pContext, const Texture::SharedPtr& tex, const Buffer::SharedPtr& buf, const uint2& size)
{
    auto vars = mpConvertTexToBuf->getVars();
    vars["GlobalCB"]["gStride"] = size.x;
    vars["gInTex"] = tex;
    vars["gOutBuf"] = buf;
    mpConvertTexToBuf->execute(pContext, size.x, size.y);
}

void OptixDenoiserPass::convertBufToTex(RenderContext* pContext, const Buffer::SharedPtr& buf, const Texture::SharedPtr& tex, const uint2& size)
{
    auto vars = mpConvertBufToTex->getVars();
    vars["GlobalCB"]["gStride"] = size.x;
    vars["gInBuf"] = buf;
    mpFbo->attachColorTarget(tex, 0);
    mpConvertBufToTex->execute(pContext, mpFbo);
}

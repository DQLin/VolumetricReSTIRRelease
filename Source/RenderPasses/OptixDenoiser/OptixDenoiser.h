/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/

/* This is a simple encapsulating of the OptiX Denoiser. It is not guaranteed
   optimal.  In fact, it is certainly suboptimal, as it aims to keep flexibility
   for use in *any* Falcor render graph without needing awareness of any
   resource interop requirements for: Falcor <-> OptiX <-> CUDA.

   The pass thus uses extraneous resource copies that could be optimized away.
   This adds some overhead, though on a RTX 3090, this pass takes about 3ms at 
   1080p, which seems quite reasonable for quick prototyping.

   Using this pass:
     * Connect noisy color image to the "color" pass texture
          - Can be LDR or HDR.  My testing shows the HDR model works just fine
            on LDR inputs... so this pass defaults to always using HDR.
     * (Optionally) connect non-noisy albedo and normals to the "albedo" and
       "normal" pass inputs.  Think:  These come directly from your G-buffer.
     * (Optionally) connect non-noisy motion vectors to the "mvec" pass input.
       Motion vectors should be in image space, as output by the Falcor G-Buffer pass.
     * Denoised results get output to the "output" pass texture
     * Basic UI controls many OptiX settings, though a few are not yet exposed.
     * The following parameters can be used in Python / scripting to control
       startup / initial default settings:
          - inputs [OptixDenoiserInputs.Automatic/Color/ColorAlbedo/ColorAlbedoNormal]
          - model [OptixDenoiserModel.LDR/HDR]
          - denoiseAlpha [True/False]:  Should denoising run on alpha channel of input?
          - blend [0...1]:  Blends output between denoised and noisy input image
               (here 0 means fully denoised, which is the default)
 */

#pragma once

#include "Falcor.h"
#include "FalcorExperimental.h"
#include "CudaUtils.h"

using namespace Falcor;


class OptixDenoiserPass : public RenderPass
{
public:
    using SharedPtr = std::shared_ptr<OptixDenoiserPass>;

    static SharedPtr create(RenderContext* pRenderContext, const Dictionary& dict);

    virtual std::string getDesc() override;
    virtual Dictionary getScriptingDictionary() override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pContext, const CompileData& compileData) override;
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;

    // Scripting functions
    bool getEnabled() const { return mEnabled; }
    void setEnabled(bool enabled) { mEnabled = enabled; }

private:
    OptixDenoiserPass(const Dictionary& dict);

    // Initializes OptiX & CUDA contexts.  Returns true on success (if false, everything else will fail).
    bool initializeOptix();

    // Call when we need to (re-)create an OptiX denoiser, on initialization or when settings change.
    void setupDenoiser();

    // The OptiX denoiser expects inputs and outputs as flat arrays (i.e., not CUDA arrays / textures
    //    with z-order internal memory layout).  We can either bang on CUDA/OptiX to support that *or* we can
    //    convert layout on the DX size with a pre-/post-pass to convert to a flat array, then share flat
    //    arrays with OptiX.  While conversion is non-optimal, we'd need to do internal blit()s anyways (to
    //    avoid exposing OptiX interop outside this render pass) so this isn't much slower than a better-designed
    //    sharing of GPU memory between DX and OptiX.
    void convertTexToBuf(RenderContext* pContext, const Texture::SharedPtr& tex, const Buffer::SharedPtr& buf, const uint2& size);
    void convertBufToTex(RenderContext* pContext, const Buffer::SharedPtr& buf, const Texture::SharedPtr& tex, const uint2& size);
    void convertMotionVectors(RenderContext* pContext, const Texture::SharedPtr& tex, const Buffer::SharedPtr& buf, const uint2& size);

    // Options and parameters for the Falcor render pass
    bool                        mEnabled = true;            ///< True = using OptiX denoiser, False = pass is a no-op
    bool                        mIsFirstFrame = true;       ///< True on the first frame after (re-)creating a denoiser
    bool                        mHasColorInput = true;      ///< Do we have a color input? 
    bool                        mHasAlbedoInput = false;    ///< Do we have an albedo guide image for denoising?
    bool                        mHasNormalInput = false;    ///< Do we have a normal guide image for denoising?
    bool                        mHasMotionInput = false;    ///< Do we have input motion vectors for temporal denoising?
    uint2                       mBufferSize = uint2(0, 0);  ///< Current window / render size
    bool                        mRecreateDenoiser = true;   ///< Do we need to (re-)initialize the denoiser before invoking it?

    // GUI helpers for choosing between different OptiX AI denoiser modes
    Gui::DropdownList           mModelChoices = {};
    uint32_t                    mSelectedModel = OptixDenoiserModelKind::OPTIX_DENOISER_MODEL_KIND_TEMPORAL;

    // Optix context
    bool                        mOptixInitialized = false;
    OptixDeviceContext          mOptixContext = nullptr;

    // Structure to encapsulate DX <-> CUDA interop data for a buffer
    struct Interop
    {
        Buffer::SharedPtr       buffer;                       // Falcor buffer
        CUdeviceptr             devicePtr = (CUdeviceptr)0;   // CUDA pointer to buffer
    };

    // Encapsulte our denoiser parameters, settings, and state.
    struct
    {
        // Various OptiX denoiser parameters and handles
        OptixDenoiserOptions    options = { 0u, 0u };
        OptixDenoiserModelKind  modelKind = OptixDenoiserModelKind::OPTIX_DENOISER_MODEL_KIND_TEMPORAL;
        OptixDenoiser           denoiser = nullptr;
        OptixDenoiserParams     params = { 0u, static_cast<CUdeviceptr>(0), 0.0f, static_cast<CUdeviceptr>(0) };
        OptixDenoiserSizes      sizes = {};

        // TODO: Parameters currently set to false and not exposed to the user.  These parameters are here to
        //    allow the code to (start) configuring settings differently when these are enabled, *however*
        //    there has not been testing or even validation that all parameters are set correctly to enable
        //    these settings.
        bool                    kernelPredictionMode = false;  
        bool                    useAOVs = false;
        uint32_t                tileOverlap = 0u;

        // If using tiled denoising, set appropriately, otherwise use the full input / output image size.
        uint32_t                tileWidth = 0u;
        uint32_t                tileHeight = 0u;

        // A wrapper around denoiser inputs for guide normals, albedo, and motion vectors
        OptixDenoiserGuideLayer guideLayer = {};

        // A wrapper around denoiser input color, output color, and prior frame's output (for temporal reuse)
        OptixDenoiserLayer      layer = {};

        // A wrapper around our guide layer interop with DirectX
        struct Intermediates
        {
            Interop             normal;
            Interop             albedo;
            Interop             motionVec;
            Interop             denoiserInput;
            Interop             denoiserOutput;
        } interop;

        // GPU memory we need to allocate for the Optix denoiser to play in & store temporaries
        CudaBuffer  scratchBuffer, stateBuffer, intensityBuffer, hdrAverageBuffer;

    } mDenoiser;

    // Our shaders for converting buffers on input and output from OptiX
    ComputePass::SharedPtr      mpConvertTexToBuf;
    ComputePass::SharedPtr      mpConvertMotionVectors;
    FullScreenPass::SharedPtr   mpConvertBufToTex;
    Fbo::SharedPtr              mpFbo;

    // Allocate a DX <-> CUDA staging buffer
    void allocateStagingBuffer(RenderContext* pContext, Interop& interop, OptixImage2D& image, OptixPixelFormat format = OPTIX_PIXEL_FORMAT_FLOAT4);

    // Not strictly required, but can be used to deallocate a staging buffer if a user toggles its use off
    void freeStagingBuffer(Interop& interop, OptixImage2D& image);

    // Reallocate all our staging buffers for DX <-> CUDA/Optix interop
    void reallocateStagingBuffers(RenderContext* pContext, uint2 newSize);

    // Get a device pointer from a buffer.  This wrapper gracefully handles nullptrs (i.e., if buf == nullptr)
    void* exportBufferToCudaDevice(Buffer::SharedPtr& buf);

};

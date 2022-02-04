/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
#pragma once
#include <optix.h>
#include <optix_stubs.h>
#include <vector>

// Note:  There's some CUDA / Falcor type conflicts.  This header includes Falcor-facing functions
//        for accessing the OptiX denoiser.  Including <cuda_runtime.h> or <cuda.h> was a pain
//        I wanted to avoid, so CUDA-specific stuff lives in CudaUtils.cpp

// Initializes OptiX.  Returns < 0 on error.
int initOptix(OptixDeviceContext& optixContext);

// This takes a Windows Handle (e.g., from Falcor::Resource::getApiHandle()) on a resource
//    that has been declared "shared" [with Resource::BindFlags::Shared] plus a size of that
//    resource in bytes and returns a CUDA device pointer from that resource that can be
//    passed into OptiX or CUDA.  This pointer is a value returned from
//    cudaExternalMemoryGetMappedBuffer(), so should follow its rules (e.g., the docs claim
//    you are responsible for calling cudaFree() on this pointer).
void* getSharedDevicePtr(HANDLE sharedHandle, uint32_t bytes);

// Calls cudaFree() on the provided pointer;
bool freeSharedDevicePtr(void* ptr);

// A utility class for a GPU/device buffer for use with CUDA.  This is essentially stolen
//    from Ingo Wald's SIGGRAPH 2019 tutorial code for OptiX 7.
class CudaBuffer
{
public:
    CudaBuffer() {}

    CUdeviceptr getDevicePtr() { return (CUdeviceptr)mpDevicePtr; }
    size_t      getSize() { return mSizeBytes; }

    void allocate(size_t size);
    void resize(size_t size);
    void free();

    template<typename T>
    void allocAndUpload(const std::vector<T>& vt);

    template<typename T>
    bool download(T* t, size_t count);

    template<typename T>
    bool upload(const T* t, size_t count);

private:
    size_t  mSizeBytes  = 0;
    void*   mpDevicePtr = nullptr;
};

/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
#include "CudaUtils.h"

#include <cuda_runtime.h>
#include <sstream>

// These live in _BootstrapUtils.cpp since they use Falcor includes / namespace,
//    which does not appear to play nice with the CUDA includes / namespace.
extern void logFatal(std::string str);
extern void logError(std::string str);
extern void logOptixWarning(unsigned int level, const char* tag, const char* message, void*);

// Apparently: this include may only appear in a single source file:
#include <optix_function_table_definition.h>

// Some debug macros
#define CUDA_CHECK(call)							                    \
    {							                                		\
      cudaError_t rc = call;                                            \
      if (rc != cudaSuccess) {                                          \
        std::stringstream txt;                                          \
        cudaError_t err =  rc; /*cudaGetLastError();*/                  \
        txt << "CUDA Error " << cudaGetErrorName(err)                   \
            << " (" << cudaGetErrorString(err) << ")";                  \
        logFatal(txt.str());                                            \
        throw std::runtime_error(txt.str());                            \
      }                                                                 \
    }

#define CUDA_CHECK_NOEXCEPT(call)                                       \
    {					                                				\
      call;                                                             \
    }

#define OPTIX_CHECK( call )                                             \
  {                                                                     \
    OptixResult res = call;                                             \
    if( res != OPTIX_SUCCESS )                                          \
      {                                                                 \
        char buf[1024]; \
        sprintf( buf, "Optix call (%s) failed with code %d (line %d)\n", #call, res, __LINE__ ); \
        logError(std::string(buf)); \
        exit( 2 );                                                      \
      }                                                                 \
  }

#define CUDA_SYNC_CHECK()                                               \
  {                                                                     \
    cudaDeviceSynchronize();                                            \
    cudaError_t error = cudaGetLastError();                             \
    if( error != cudaSuccess )                                          \
      {                                                                 \
        char buf[1024]; \
        sprintf( buf, "error (%s: line %d): %s\n", __FILE__, __LINE__, cudaGetErrorString( error ) ); \
        logError(std::string(buf)); \
        exit( 2 );                                                      \
      }                                                                 \
  }


unsigned int initCuda(void)
{
    cudaFree(0);
    int32_t numDevices;
    cudaGetDeviceCount(&numDevices);
    return numDevices;
}

int initOptix(OptixDeviceContext& optixContext)
{
    // Initialize CUDA
    uint32_t devices = initCuda();
    if (devices <= 0) return -1;

    // Initialize Optix.
    OPTIX_CHECK(optixInit());

    // Setup which device to work on.  Hard coded to device #0
    int32_t deviceId = 0;
    CUDA_CHECK(cudaSetDevice(deviceId));

    // Create a CUDA stream
    CUstream stream;
    CUDA_CHECK(cudaStreamCreate(&stream));

    // Get device information
    cudaDeviceProp deviceProps;
    cudaGetDeviceProperties(&deviceProps, deviceId);

    // Get the current context
    CUcontext cudaContext;
    CUresult cuRes = cuCtxGetCurrent(&cudaContext);

    // Build our OptiX context
    OPTIX_CHECK(optixDeviceContextCreate(cudaContext, 0, &optixContext));

    // Tell Optix how to write to our Falcor log.
    OPTIX_CHECK(optixDeviceContextSetLogCallback(optixContext,
        logOptixWarning, nullptr, 4));

    return 1;
}

void CudaBuffer::allocate(size_t size)
{
    if (mpDevicePtr) free();
    mSizeBytes = size;
    CUDA_CHECK(cudaMalloc((void**)&mpDevicePtr, mSizeBytes));
}

void CudaBuffer::resize(size_t size)
{
    allocate(size);
}

void CudaBuffer::free(void)
{
    CUDA_CHECK(cudaFree(mpDevicePtr));
    mpDevicePtr = nullptr;
    mSizeBytes = 0;
}

template<typename T>
bool CudaBuffer::download(T* t, size_t count)
{
    if (!mpDevicePtr) return false;
    if (mSizeBytes <= (count * sizeof(T))) return false;

    CUDA_CHECK(cudaMemcpy((void*)t, mpDevicePtr, count * sizeof(T), cudaMemcpyDeviceToHost));
    return true; // might be an error caught by CUDA_CHECK?  TODO: process any such error through
}

template<typename T>
bool CudaBuffer::upload(const T* t, size_t count)
{
    if (!mpDevicePtr) return false;
    if (mSizeBytes <= (count * sizeof(T))) return false;

    CUDA_CHECK(cudaMemcpy(mpDevicePtr, (void*)t, count * sizeof(T), cudaMemcpyHostToDevice));
    return true; // might be an error caught by CUDA_CHECK?  TODO: process any such error through
}

template<typename T>
void CudaBuffer::allocAndUpload(const std::vector<T>& vt)
{
    allocate(vt.size() * sizeof(T));
    upload((const T*)vt.data(), vt.size());
}


void* getSharedDevicePtr(HANDLE sharedHandle, uint32_t bytes)
{
    // No handle?  No pointer!
    if (sharedHandle == NULL) return nullptr;

    // Create the descriptor of our shared memory buffer
    cudaExternalMemoryHandleDesc externalMemoryHandleDesc;
    memset(&externalMemoryHandleDesc, 0, sizeof(externalMemoryHandleDesc));
    externalMemoryHandleDesc.type = cudaExternalMemoryHandleTypeD3D12Resource;
    externalMemoryHandleDesc.handle.win32.handle = sharedHandle;
    externalMemoryHandleDesc.size = bytes;
    externalMemoryHandleDesc.flags = cudaExternalMemoryDedicated;

    // Get a handle to that memory
    cudaExternalMemory_t externalMemory;
    CUDA_CHECK(cudaImportExternalMemory(&externalMemory, &externalMemoryHandleDesc));

    // Create a descriptor for our shared buffer pointer
    cudaExternalMemoryBufferDesc bufDesc;
    memset(&bufDesc, 0, sizeof(bufDesc));
    bufDesc.size = bytes;

    // Actually map the buffer
    void* devPtr = nullptr;
    CUDA_CHECK(cudaExternalMemoryGetMappedBuffer(&devPtr, externalMemory, &bufDesc));
    return devPtr;
}

bool freeSharedDevicePtr(void* ptr)
{
    if (!ptr) return false;
    return cudaSuccess == cudaFree(ptr);
}

/***************************************************************************
 # Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/

// A bunch of syntactic sugar and utilities Chris likes and has accumulated.

#pragma once
#include "Falcor.h"
#include "Utils/HostDeviceShared.slangh"

// This include needs to go below Falcor.h
#include "glm/glm.hpp"

// Some constants I use frequently, to avoid long string of Resource::BindFlags constants |'d together.
extern const Falcor::Resource::BindFlags   kDefaultFlags;
extern const Falcor::Resource::BindFlags   kDepthFlags;
extern const Falcor::Gui::WindowFlags      kPassGuiFlags;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  We use pybind11 for (a) reading initialization parameters from our renderer definition Python script and
//    (b) for render pass to render pass communication using a shared Python dictionary.
//
//  I find pybind11 <-> C++ conversions painful, since they are often not automatic, possibly depend on
//     compiler version, and manual conversion are difficult (for me) to remember.  Instead, I have a bunch of
//     helpers I use to encapsulate and simplify the code.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


/** Converts a pybind dictionary entry into a shared pointer of the specified type.  For some reason this
    cannot be a single line conversion and *must* have the assignment operator; VC++ won't do it implicitly
    *or* explicitly without the assignment.  Not sure if this is a compiler issue or some pybind11 template
    metaprogramming gunk I don't understand.  Blegh.
    \param[in] v A Falcor/Python dictionary value that is actually a shared_ptr<> of some Falcor class.
    \return Returns the value cast as a shared_ptr<> of the appropriate type.  Be aware, there is only light error checking.
*/
template<typename T>
std::shared_ptr<T> as_ptr(const Falcor::Dictionary::Value &v)
{
    std::shared_ptr<T> ptr = v;
    return ptr;
}

/** For cleanly accessing dictionary entries as a buffer.  (Macro assumes mDict exists in current scope)
    \param[in] s  A string name from the Python dictionary that should be treated as a texture
    \return Returns the dictonary as a Buffer::SharedPtr (or nullptr on error)
*/
#define _asBuffer(_s)   (mDict.keyExists((_s)) ? as_ptr<Falcor::Buffer>(mDict[(_s)]) : nullptr)

/** For cleanly accessing dictionary entries as a texture.  (Macro assumes mDict exists in current scope)
    \param[in] s  A string name from the Python dictionary that should be treated as a texture
    \return Returns the dictonary as a Texture::SharedPtr (or nullptr on error)
*/
#define _asTexture(_s)  (mDict.keyExists((_s)) ? as_ptr<Falcor::Texture>(mDict[(_s)]) : nullptr)

/** Converts a Python dictionary entry to the appropriate type; if nonexistant, use the default value.
    I keep thinking I should add variants without a default, but this forces me to specify reasonable
    defaults of variables, handling the (somewhat common) case I screw up and forget to add the requested
    variable anywhere in the Python dictionary.
    \param[in] s    A string name from the Python dictionary to be treated as the appropriate type
    \param[in] def  A default value to use if errors occur.
    \return Returns the value from the dictionary cast to the appropriate type
*/
#define _asFloat(_s,_def)        (mDict.keyExists((_s)) ? float(mDict[(_s)]) : float(_def))
#define _asInt(_s,_def)          (mDict.keyExists((_s)) ? int32_t(mDict[(_s)]) : int32_t(_def))
#define _asUInt(_s,_def)         (mDict.keyExists((_s)) ? uint32_t(mDict[(_s)]) : uint32_t(_def))
#define _asBool(_s,_def)         (mDict.keyExists((_s)) ? bool(mDict[(_s)]) : bool(_def))

/** Converts a Python dictionary entry to the appropriate GLM vector type.  This is a pain because
    it's not currently castable directly, but you can implicitly cast a pybind array into a std::vector.
    These cast input parameters to std::vector then return the appropriate glm type.  If there's an error
    (e.g., the vector isn't long enough for the specified type), the default is returned.
    \param[in] pyVec A Python Dictionary::Value (auto-cast to a std::vector<float>) representing the vector data
    \param[in] def   A default value to use if errors occur.
    \return Returns the value from the dictionary cast to the appropriate type
*/
float2 _toVec2(std::vector<float> pyVec, float2 def = float2(0, 0));
float3 _toVec3(std::vector<float> pyVec, float3 def = float3(0, 0, 0));
float4 _toVec4(std::vector<float> pyVec, float4 def = float4(0, 0, 0, 0));

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   (...Done with pybind11 helpers...)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


/** Helpers to create a new / unique linear or nearest neightbor sampler
*/
Falcor::Sampler::SharedPtr createLinearSampler();
Falcor::Sampler::SharedPtr createNearestSampler();

/** We create a texture with a specified number of entries.  Each entry is a low-discrepenency
    ("random") sample/offset within 1 unit around the origin (0,0).  The texture is RG8Int,
    rather than RG32Float (or other format) for compactness.  This is created WITHOUT mipmapping.
    \param[in] numSamples How many entries this neighbor offset texture should have
*/
Falcor::Texture::SharedPtr createNeighborOffsetTexture( int numSamples = 8192 );

/** Helpers to create simplistic/basic Falcor ray tracing shaders and compute shaders with minimal code
*/
Falcor::ComputePass::SharedPtr createSimpleComputePass(const std::string& file, const std::string& mainEntry, Shader::DefineList defs = {});

/** Maps the specified GPU buffer, reads back the data into the var[] array, reading
    a number of elements specified.  Note:  If this buffer was *not* created as a structure or
    typed buffer, this will likely fail.  TODO: Does something like this belong in the Falcor Buffer class?
*/
template <typename T>
void readbackBufferData(Falcor::Buffer::SharedPtr buf, T *var, uint32_t numElems)
{
    if (numElems > 0)
    {
        const T* bufPtr = reinterpret_cast<const T*>(buf->map(Buffer::MapType::Read));
        CopyMemory(var, bufPtr, sizeof(T) * numElems);
        buf->unmap();
    }
}

/** What's this do?  When using a Falcor ComputePass (rather than a raster or a ray passes),
    none of the scene information is automatically sent to the shader with geometry/material information.
    The user is expected to send any such information down as required.  This function sets appropriate
    #defines in the shader so that you can #import Shading in the compute shader to access the data.
    (The function is slowly evloving closer to a no-op, but is still required for now.)
*/
void updateSceneDefines(Falcor::ComputePass::SharedPtr& pPass, const Falcor::Scene::SharedPtr& pScene);

/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/

// These are wrappers around Falcor::logError() and friends so they can be used in
//    CudaUtils.cpp (for logging of CUDA errors on the standard Falcor logs) without
//    me debugging why the CUDA and Falcor includes/namespaces don't like each other.
// (Signs point to type conflicts, like Falcor float4 and CUDA float4, being a key issue.)

#include "Falcor.h"

void logFatal(std::string str)
{
    Falcor::logFatal(str);
}

void logError(std::string str)
{
    Falcor::logError(str);
}

void logOptixWarning(unsigned int level, const char* tag, const char* message, void*)
{
    char buf[2048];
    sprintf(buf, "[OptiX][%2d][%12s]: %s", (int)level, tag, message);
    Falcor::logWarning(buf);
}

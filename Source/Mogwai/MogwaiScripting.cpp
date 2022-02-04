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

namespace Mogwai
{
    namespace
    {
        const std::string kRunScript = "script";
        const std::string kLoadScene = "loadScene";
        const std::string kSaveConfig = "saveConfig";
        const std::string kAddParticleSystem = "addParticleSystem";
        const std::string kAddSimpleCurveModel = "addSimpleCurveModel";
        const std::string kAddGVDBVolume = "addGVDBVolume";
        const std::string kAddGVDBVolumeSequence = "addGVDBVolumeSequence";        
        const std::string kAddGraph = "addGraph";
        const std::string kRemoveGraph = "removeGraph";
        const std::string kGetGraph = "getGraph";
        const std::string kUI = "ui";
        const std::string kResizeSwapChain = "resizeSwapChain";
        const std::string kCaptureScreen = "captureScreen";
        const std::string kActiveGraph = "activeGraph";
        const std::string kScene = "scene";

        const std::string kRendererVar = "m";
        const std::string kTimeVar = "t";

        template<typename T>
        std::string prepareHelpMessage(const T& g)
        {
            std::string s = Renderer::getVersionString() + "\nGlobal utility objects:\n";
            static const size_t kMaxSpace = 8;
            for (auto n : g)
            {
                s += "\t'" + n.first + "'";
                s += (n.first.size() >= kMaxSpace) ? " " : std::string(kMaxSpace - n.first.size(), ' ');
                s += n.second;
                s += "\n";
            }

            s += "\nGlobal functions\n";
            s += "\trenderFrame()      Render a frame. If the clock is not paused, it will advance by one tick. You can use it inside for loops, for example to loop over a specific time-range\n";
            s += "\texit()             Exit Mogwai\n";
            return s;
        }

        std::string windowConfig()
        {
            std::string s;
            SampleConfig c = gpFramework->getConfig();
            s += "# Window Configuration\n";
            s += Scripting::makeMemberFunc(kRendererVar, kResizeSwapChain, c.windowDesc.width, c.windowDesc.height);
            s += Scripting::makeSetProperty(kRendererVar, kUI, c.showUI);
            return s;
        }
    }

    void Renderer::saveConfig(const std::string& filename) const
    {
        std::string s;

        if (!mGraphs.empty())
        {
            s += "# Graphs\n";
            for (const auto& g : mGraphs)
            {
                s += RenderGraphExporter::getIR(g.pGraph);
                s += kRendererVar + "." + kAddGraph + "(" + RenderGraphIR::getFuncName(g.pGraph->getName()) + "())\n";
            }
            s += "\n";
        }

        if (mpScene)
        {
            s += "# Scene\n";
            s += Scripting::makeMemberFunc(kRendererVar, kLoadScene, Scripting::getFilenameString(mpScene->getFilename()));
            const std::string sceneVar = kRendererVar + "." + kScene;
            s += mpScene->getScript(sceneVar);
            s += "\n";
        }

        s += windowConfig() + "\n";

        s += "# Time Settings\n";
        s += gpFramework->getGlobalClock().getScript(kTimeVar) + "\n";

        for (auto& pe : mpExtensions)
        {
            auto eStr = pe->getScript();
            if (eStr.size()) s += eStr + "\n";
        }

        std::ofstream(filename) << s;
    }

    void Renderer::registerScriptBindings(pybind11::module& m)
    {
        pybind11::class_<Renderer> renderer(m, "Renderer");

        renderer.def(kRunScript.c_str(), &Renderer::loadScript, "filename"_a = std::string());
        renderer.def(kAddSimpleCurveModel.c_str(), &Renderer::addSimpleCurveModel, "filename"_a, "width"_a = 1.f, "diffuseColor"_a = float3(0.8, 0.8, 0.8));
        renderer.def(kAddGVDBVolume.c_str(), &Renderer::addGVDBVolume, "sigma_a"_a, "sigma_s"_a, "g"_a, "dataFile"_a, "numMips"_a = 1, "densityScale"_a = 1.f,
            "hasVelocity"_a = false, "hasEmission"_a = false, "LeScale"_a = 0.005f, "temperatureCutoff"_a = 1.f, "temperatureScale"_a = 100.f, "worldTranslation"_a = float3(0,0,0), "worldRotation"_a = float3(0, 0, 0), "worldScaling"_a = 1.f);
        renderer.def(kAddGVDBVolumeSequence.c_str(), &Renderer::addGVDBVolumeSequence, "sigma_a"_a, "sigma_s"_a, "g"_a, "dataFilePrefix"_a, "numberFixedLength"_a, "startFrame"_a,
            "numFrames"_a, "numMips"_a = 1, "densityScale"_a = 1.f, "hasVelocity"_a = false, "hasEmission"_a = false, "LeScale"_a = 0.005f, "temperatureCutoff"_a = 1.f, "temperatureScale"_a = 100.f,
            "worldTranslation"_a = float3(0, 0, 0), "worldRotation"_a = float3(0, 0, 0), "worldScaling"_a = 1.f);
        renderer.def(kAddParticleSystem.c_str(), &Renderer::addParticleSystem, "maxParticles"_a = 4096, "maxEmitPerFrame"_a = 256, "useFixedInterval"_a = false,
            "fixedInterval"_a = 0.01f, "maxRenderFrames"_a = 1000, "shouldSort"_a = false,
            "duration"_a = 3.0, "durationOffset"_a = 0.f, "emitFrequency"_a = 0.1f, "emitCount"_a = 32, "emitCountOffset"_a = 0,
            "spawnPos"_a = float3(0.f,0.f,0.f), "spawnPosOffset"_a = float3(0.f,0.5f,0.f), "vel"_a = float3(0,5,0), "velOffset"_a = float3(2,1,2),
            "scale"_a = 0.2f, "scaleOffset"_a = 0.f, "growth"_a = -0.05f, "growthOffset"_a = 0.f, "billboardRotation"_a = 0.f,
            "billboardRotationOffset"_a = 0.25f, "billboardRotationVel"_a = 0.f, "billboardRotationVelOffset"_a = 0.25f, 
            "shadingType"_a = 2, "startColor"_a = float4(1,1,1,1), "endColor"_a = float4(1,1,1,0.1), "startT"_a = 3.f, "endT"_a = 0.f, "textureFile"_a = "");
        renderer.def(kLoadScene.c_str(), &Renderer::loadScene, "filename"_a = std::string(), "buildFlags"_a = SceneBuilder::Flags::Default);
        renderer.def(kSaveConfig.c_str(), &Renderer::saveConfig, "filename"_a);
        renderer.def(kAddGraph.c_str(), &Renderer::addGraph, "graph"_a);
        renderer.def(kRemoveGraph.c_str(), pybind11::overload_cast<const std::string&>(&Renderer::removeGraph), "name"_a);
        renderer.def(kRemoveGraph.c_str(), pybind11::overload_cast<const RenderGraph::SharedPtr&>(&Renderer::removeGraph), "graph"_a);
        renderer.def(kGetGraph.c_str(), &Renderer::getGraph, "name"_a);
        renderer.def("graph", &Renderer::getGraph); // PYTHONDEPRECATED
        auto envMap = [](Renderer* pRenderer, const std::string& filename) { if (pRenderer->getScene()) pRenderer->getScene()->loadEnvMap(filename); };
        renderer.def("envMap", envMap, "filename"_a); // PYTHONDEPRECATED

        // PYTHONDEPRECATED Use the global function defined in the script bindings in Sample.cpp when resizing from a Python script.
        auto resize = [](Renderer* pRenderer, uint32_t width, uint32_t height) {gpFramework->resizeSwapChain(width, height); };
        renderer.def(kResizeSwapChain.c_str(), resize);

        // capture screen to file
        auto captureScreen = [](Renderer* pRenderer, const std::string& filename, const std::string directory) {gpFramework->captureScreen(filename, directory); };
        renderer.def(kCaptureScreen.c_str(), captureScreen);

        renderer.def_property_readonly(kScene.c_str(), &Renderer::getScene);
        renderer.def_property_readonly(kActiveGraph.c_str(), &Renderer::getActiveGraph);

        auto getUI = [](Renderer* pRenderer) { return gpFramework->isUiEnabled(); };
        auto setUI = [](Renderer* pRenderer, bool show) { gpFramework->toggleUI(show); };
        renderer.def_property(kUI.c_str(), getUI, setUI);

        Extension::Bindings b(m, renderer);
        b.addGlobalObject(kRendererVar, this, "The engine");
        b.addGlobalObject(kTimeVar, &gpFramework->getGlobalClock(), "Time Utilities");
        for (auto& pe : mpExtensions) pe->scriptBindings(b);
        mGlobalHelpMessage = prepareHelpMessage(b.mGlobalObjects);

        // Replace the `help` function
        auto globalHelp = [this]() { pybind11::print(mGlobalHelpMessage);};
        m.def("help", globalHelp);

        auto objectHelp = [](pybind11::object o)
        {
            auto b = pybind11::module::import("builtins");
            auto h = b.attr("help");
            h(o);
        };
        m.def("help", objectHelp, "object"_a);
    }
}

# Graphs
from falcor import *
import copy

gAccumulateParams = {
    "enableAccumulation": False
}

def render_graph():
    g = RenderGraph("Volumetric ReSTIR")
    loadRenderPassLibrary("AccumulatePass.dll")    
    loadRenderPassLibrary("VolumetricReSTIR.dll")
    loadRenderPassLibrary("AntiAliasing.dll")
    VolumetricReSTIR = createPass("VolumetricReSTIR", {'mTestAniso': True})
    AccumulatePass = createPass("AccumulatePass", gAccumulateParams)
    g.addPass(VolumetricReSTIR, "VolumetricReSTIR")
    g.addPass(AccumulatePass, "AccumulatePass")    
    g.addEdge("VolumetricReSTIR.accumulated_color", "AccumulatePass.input")    
    g.markOutput("AccumulatePass.output")
   
    return g
    
# Scene
m.loadScene("default.obj")

m.scene.setEnvMap("kiara_1_dawn_8k.hdr")
m.scene.setEnvMapIntensity(1.1)
m.scene.setEnvMapRotation(float3(-37.5, 93.5, 27))

m.addGVDBVolume(sigma_a=float3(1,1,1), sigma_s=float3(9,9,9), g=0.0, dataFile="wdas_cloud_quarter", numMips=6, densityScale = 0.03, hasVelocity=False, hasEmission=False, LeScale=1.0)

# kiara_1_dawn_8k
m.scene.camera.position = float3(258.785950,7.888093,-265.658325)
m.scene.camera.target = float3(257.988922,8.031697,-265.071716)
m.scene.camera.up = float3(0.003697,0.999990,-0.002712)
m.scene.cameraSpeed = 35.47999954223633

graph = render_graph()

m.addGraph(graph)

# Window Configuration
m.resizeSwapChain(1920, 1080)
m.ui = True

# Time Settings
t.time = 0
t.framerate = 0
# If framerate is not zero, you can use the frame property to set the start frame
# t.frame = 0

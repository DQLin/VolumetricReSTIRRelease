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
    VolumetricReSTIR = createPass("VolumetricReSTIR")
    AccumulatePass = createPass("AccumulatePass", gAccumulateParams)
    g.addPass(VolumetricReSTIR, "VolumetricReSTIR")
    g.addPass(AccumulatePass, "AccumulatePass")    
    g.addEdge("VolumetricReSTIR.accumulated_color", "AccumulatePass.input")    
    g.markOutput("AccumulatePass.output")
   
    return g
    
# Scene
m.loadScene("default.obj")
m.scene.setEnvMap("green_sanctuary_8k.hdr")
m.scene.setEnvMapIntensity(1.5)
m.addGVDBVolume(sigma_a=float3(1,1,1), sigma_s=float3(9,9,9), g=0.0, dataFile="bunny_cloud", numMips=7, densityScale = 1.0, hasVelocity=False, hasEmission=False, LeScale=1.0)

# bunny_cloud
m.scene.camera.position = float3(-17.290087,23.748606,45.495544)
m.scene.camera.target = float3(-17.053705,23.593506,44.536343)
m.scene.camera.up = float3(0.003480,0.999892,-0.014248)

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


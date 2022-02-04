# Graphs
from falcor import *

gAccumulateParams = {
    "enableAccumulation": False
}

def render_graph():
    g = RenderGraph("Volumetric ReSTIR")
    loadRenderPassLibrary("AccumulatePass.dll")    
    loadRenderPassLibrary("VolumetricReSTIR.dll")
        
    VolumetricReSTIR = createPass('VolumetricReSTIR', {'mOutputMotionVec': True})
    AccumulatePass = createPass("AccumulatePass", gAccumulateParams)

    g.addPass(VolumetricReSTIR, "VolumetricReSTIR")
    g.addPass(AccumulatePass, "AccumulatePass")
    
    g.addEdge("VolumetricReSTIR.accumulated_color", "AccumulatePass.input")	
    
    g.markOutput("AccumulatePass.output")

    return g
    
# Scene
m.loadScene("default.obj")
m.scene.setEnvMap("hansaplatz_8k.hdr")


m.addGVDBVolumeSequence(sigma_a=float3(6,6,6), sigma_s=float3(14,14,14), g=0.0, dataFilePrefix="fire115/fire115.", numberFixedLength=4, startFrame=100, numFrames=200, numMips=4, densityScale = 0.1, hasVelocity=True, hasEmission=False, LeScale=0.01, temperatureCutoff = 900.0, temperatureScale = 0.0, worldTranslation=float3(0,1.686,0), worldRotation=float3(0,0,0), worldScaling=0.013)

m.scene.cameraSpeed = 1

m.scene.camera.position = float3(1.977354,2.411630,2.242076)
m.scene.camera.target = float3(1.366226,2.220231,1.474033)
m.scene.camera.up = float3(0.000000,1.000000,0.000000)


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

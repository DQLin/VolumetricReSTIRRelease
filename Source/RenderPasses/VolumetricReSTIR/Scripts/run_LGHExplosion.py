# Graphs
from falcor import *

gAccumulateParams = {
    "enableAccumulation": False
}

def render_graph():
    g = RenderGraph("Volumetric ReSTIR")
    loadRenderPassLibrary("AccumulatePass.dll")    
    loadRenderPassLibrary("VolumetricReSTIR.dll")
    VolumetricReSTIR = createPass("VolumetricReSTIR", {"mParams": VolumetricReSTIRParams(mMaxBounces=2)})
    AccumulatePass = createPass("AccumulatePass", gAccumulateParams)
    g.addPass(VolumetricReSTIR, "VolumetricReSTIR")
    g.addPass(AccumulatePass, "AccumulatePass")
        
    g.addEdge("VolumetricReSTIR.accumulated_color", "AccumulatePass.input")	
    g.markOutput("AccumulatePass.output")
   
    return g
    
# Scene
m.loadScene("default.obj")
m.scene.setEnvMap("skylight-dusk.exr")
m.scene.setEnvMapRotation(float3(13,136,340))

m.addGVDBVolume(sigma_a=float3(10,10,10), sigma_s=float3(10,10,10), g=0.0, dataFile="LGHExplosion/LGHExplosion.183", numMips=4, densityScale = 0.02, hasVelocity=False, hasEmission=True, LeScale=1.0, temperatureCutoff = 0.0, temperatureScale = 750.0)

m.scene.camera.position = float3(-29.487331,10.408340,0.493491)
m.scene.camera.target = float3(-28.505859,10.597045,0.460278)
m.scene.camera.up = float3(-0.000909,1.000000,0.000030)

# representative animation frame: 77
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

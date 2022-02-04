# Graphs
from falcor import *

gAccumulateParams = {
    "enableAccumulation": False
}

def render_graph():
    g = RenderGraph("Volumetric ReSTIR")
    loadRenderPassLibrary("AccumulatePass.dll")    
    loadRenderPassLibrary("VolumetricReSTIR.dll")
    loadRenderPassLibrary("ToneMapper.dll")
    
    #if override bias settings, must guarantee all values that are desired to be used exist here
    VolumetricReSTIR = createPass('VolumetricReSTIR', {'mParams': VolumetricReSTIRParams(mUseSurfaceScene = True, mUseEmissiveLights=True, mUseEnvironmentLights=False, mTemporalReuseMThreshold=10.0)})

    g.addPass(VolumetricReSTIR, 'VolumetricReSTIR')
    AccumulatePass = createPass('AccumulatePass', gAccumulateParams)
    g.addPass(AccumulatePass, 'AccumulatePass')
    ToneMapper = createPass('ToneMapper', {'exposureCompensation': 8.0, 'autoExposure': False, 'exposureValue': 0.0, 'filmSpeed': 100.0, 'whiteBalance': False, 'whitePoint': 6500.0, 'operator': ToneMapOp.Aces, 'clamp': True, 'whiteMaxLuminance': 1.0, 'whiteScale': 11.199999809265137, 'fNumber': 1.0, 'shutter': 1.0, 'exposureMode': ExposureMode.AperturePriority})
    g.addPass(ToneMapper, 'ToneMapper')
    g.addEdge('VolumetricReSTIR.accumulated_color', 'AccumulatePass.input')
    g.addEdge('AccumulatePass.output', 'ToneMapper.src')
    g.markOutput('ToneMapper.dst')
    
    return g
    
# Scene
m.loadScene("Bistro_5_1/BistroExterior.fbx")
m.scene.setEnvMap("skylight-morn.exr")
m.scene.setEnvMapRotation(float3(0,72.5,0))
m.scene.setEnvMapIntensity(0)

m.addGVDBVolume(sigma_a=float3(10,10,10), sigma_s=float3(80,80,80), g=0.0, dataFile="smoke-plume-2", numMips=4)

m.scene.animated = False
m.scene.camera.animated = False
m.scene.camera.position = float3(-15.149291,8.352362,-8.399609)
m.scene.camera.target = float3(-14.742913,8.025879,-7.546224)
m.scene.camera.up = float3(0.004061,0.999961,0.007782)
m.scene.cameraSpeed = 1.0

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





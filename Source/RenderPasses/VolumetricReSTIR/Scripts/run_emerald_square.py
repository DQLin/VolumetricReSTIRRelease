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
   
    VolumetricReSTIR = createPass('VolumetricReSTIR', {'mParams': VolumetricReSTIRParams(mMaxBounces = 1, mUseSurfaceScene = True, mUseEmissiveLights=True, mUseEnvironmentLights=True, mTemporalReuseMThreshold=10)})

    g.addPass(VolumetricReSTIR, 'VolumetricReSTIR')
    AccumulatePass = createPass('AccumulatePass', gAccumulateParams)
    g.addPass(AccumulatePass, 'AccumulatePass')
    ToneMapper = createPass('ToneMapper', {'exposureCompensation': 8.5, 'autoExposure': False, 'exposureValue': 0.0, 'filmSpeed': 100.0, 'whiteBalance': False, 'whitePoint': 6500.0, 'operator': ToneMapOp.Aces, 'clamp': True, 'whiteMaxLuminance': 1.0, 'whiteScale': 11.199999809265137, 'fNumber': 1.0, 'shutter': 1.0, 'exposureMode': ExposureMode.AperturePriority})
    g.addPass(ToneMapper, 'ToneMapper')
    g.addEdge('VolumetricReSTIR.accumulated_color', 'AccumulatePass.input')
    g.addEdge('AccumulatePass.output', 'ToneMapper.src')
    g.markOutput('ToneMapper.dst')
    
    return g
    
# Scene
m.loadScene("EmeraldSquare_v4/EmeraldSquare_Dusk.fbx")
m.scene.setEnvMap("skylight-morn.exr")
m.scene.setEnvMapRotation(float3(0,72.5,0))
m.scene.setEnvMapIntensity(0.003)

m.addGVDBVolume(sigma_a=float3(6,6,6), sigma_s=float3(14,14,14), g=0.0, dataFile="Fog/Fog.0043", numMips=4, densityScale = 0.1, hasVelocity=False, hasEmission=False, LeScale=0.01, temperatureCutoff = 900.0, temperatureScale = 0.0, worldTranslation=float3(0,5.676,-1.149), worldRotation=float3(-90,0,0), worldScaling=0.123)

m.scene.animated = False
m.scene.camera.animated = False

# emerald square default
m.scene.camera.position = float3(-18.527357,8.396429,3.235079)
m.scene.camera.target = float3(-17.847946,8.211088,2.525114)
m.scene.camera.up = float3(-0.031144,0.998985,0.032544)
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

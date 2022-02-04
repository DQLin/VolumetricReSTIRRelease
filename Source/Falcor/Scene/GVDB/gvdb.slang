__exported import Scene.GVDB.gvdbNodes;
import RenderPasses.VolumetricReSTIR.HostDeviceSharedConstants;

static const uint ID_UNDEFL = 0xFFFFFFFF;

float getValueAtPoint(VDBInfo gvdb, int mipId, float3 pos, int channel, SamplerState sampler)
{
    float3 offs, vmin;
    uint node_id;

    VDBNode node = getNodeAtPoint(gvdb, mipId, pos, offs, vmin, node_id);

    if (node_id == ID_UNDEFL) return 0.f;

    float3 o = offs;	
    float3 p_rel = pos - vmin;

    float3 texCoords = o + p_rel;

    if (texCoords.z > gvdb.volInDimensions[mipId].z)
    {
        texCoords.z -= gvdb.volInDimensions[mipId].z;
        return gvdb.volIn_part2[mipId].SampleLevel(sampler, texCoords * gvdb.invVolInDimensions_part2[mipId], 0);
    }
    else
    {
        return gvdb.volIn[mipId].SampleLevel(sampler, texCoords * gvdb.invVolInDimensions[mipId], 0);
    }
}

float3 getVelocityAtPoint(VDBInfo gvdb, float3 pos, int channel, SamplerState sampler, bool isLastFrame = false)
{
    int mipId = isLastFrame ? kVelocityGridId + kPrevFrameExtraGridOffset : kVelocityGridId;
    int velocityTexId = isLastFrame ? 1 : 0;

    float3 offs, vmin;
    uint node_id;

    VDBNode node = getNodeAtPoint(gvdb, mipId, pos, offs, vmin, node_id);

    if (node_id == ID_UNDEFL) return float3(0.f);

    float3 o = offs;
    float3 p_rel = pos - vmin;

    float3 texCoords = o + p_rel;

    if (texCoords.z > gvdb.volInDimensions[mipId].z)
    {
        texCoords.z -= gvdb.volInDimensions[mipId].z;
        return gvdb.velocityIn_part2[velocityTexId].SampleLevel(sampler, texCoords * gvdb.invVolInDimensions_part2[mipId], 0);
    }
    else
    {
        return gvdb.velocityIn[velocityTexId].SampleLevel(sampler, texCoords * gvdb.invVolInDimensions[mipId], 0);
    }
}


float getValueAtlasCoord(VDBInfo gvdb, float3 texCoords, int mipId, SamplerState sampler)
{
    if (texCoords.z > gvdb.volInDimensions[mipId].z)
    {
        texCoords.z -= gvdb.volInDimensions[mipId].z;
        return gvdb.volIn_part2[mipId].SampleLevel(sampler, texCoords * gvdb.invVolInDimensions_part2[mipId], 0);
    }
    else
    {
        return gvdb.volIn[mipId].SampleLevel(sampler, texCoords * gvdb.invVolInDimensions[mipId], 0);
    }
}


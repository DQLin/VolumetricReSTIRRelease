#include "CurveFileLoader.h"
#include "Scene/Scene.h"
#include "stdafx.h"
#include "../Externals/cyHair/cyHairFile.h"

#define AUTOMATIC_YARN_SEGMENTATION


void CurveFileLoader::loadKnitCCPFile(std::shared_ptr<Scene> pScene, const std::string& filename, int materialID, float width, float scale, bool yz)
{
    using KnitData = std::vector<std::vector<float3>>;
    KnitData	knit_data;

    std::string filefullpath;
    findFileInDataDirectories(filename, filefullpath);

    FILE* fp = fopen(filefullpath.c_str(), "r");
    if (!fp) return;

    std::vector<float3>	yarn_rnd_pts;

    CurveIOBuffer buffer;

    yarn_rnd_pts.clear();

    float3 p;
    while (buffer.ReadLine(fp)) {
        if (buffer.IsCommand("v")) {
            if (yz) buffer.ReadVertex(p);
            else	buffer.ReadVertexYZ(p);
        }

        yarn_rnd_pts.push_back(p * scale);
        if (feof(fp)) break;
    }

    knit_data.clear();
    std::vector<float3> yarn;
    const int subSegNum = 11;
    for (int i = 0; i < (int)yarn_rnd_pts.size() - 3; i++) {

        float localLength = 0.0;
        for (int j = 0; j < subSegNum - 1; j++) {
            float t0 = j / float(subSegNum - 1);
            float t1 = (j + 1) / float(subSegNum - 1);
            const float3 p0 = float3((float)pow(1 - t0, 3)) * yarn_rnd_pts[i + 0] + float3(3 * (float)pow(1 - t0, 2) * t0) * yarn_rnd_pts[i + 1] + float3(3 * (float)pow(t0, 2) * (1 - t0)) * yarn_rnd_pts[i + 2] + float3((float)pow(t0, 3)) * yarn_rnd_pts[i + 3];
            const float3 p1 = float3((float)pow(1 - t1, 3)) * yarn_rnd_pts[i + 0] + float3(3 * (float)pow(1 - t1, 2) * t1) * yarn_rnd_pts[i + 1] + float3(3 * (float)pow(t1, 2) * (1 - t1)) * yarn_rnd_pts[i + 2] + float3((float)pow(t1, 3)) * yarn_rnd_pts[i + 3];
            const float3 diff = p0 - p1;
            localLength += length(diff);
        }

#ifdef AUTOMATIC_YARN_SEGMENTATION
        if (localLength > 10.0f) {
            if (yarn.size() > 3) knit_data.push_back(yarn);
            yarn.clear();
            continue;
        }
#endif

        yarn.push_back(yarn_rnd_pts[i]);
    }

    if (!yarn.empty()) knit_data.push_back(yarn);

    std::cout << "load " << knit_data.size() << " yarns\n";
    for (int yi = 0; yi < knit_data.size(); yi++)
        std::cout << "Yarn " << yi << " " << knit_data[yi].size() << std::endl;

    fclose(fp);

    GenerateBezierPatchesFromKnitData(pScene, knit_data, materialID, width);
}

// using a default width of 1
void CurveFileLoader::GenerateBezierPatchesFromKnitData(std::shared_ptr<Scene> pScene, const std::vector<std::vector<float3>>& knitData, int materialID, float width)
{
    for (int yarnId = 0; yarnId < knitData.size(); yarnId++)
    {
        int curveId = (int)pScene->mCurveDesc.size();
        CurveDesc spec;
        spec.curveType = 1;
        spec.vbOffset = (uint32_t)pScene->mCPUCurveVertexBuffer.size();
        const std::vector<float3>& yarn = knitData[yarnId];
        int yarnSize = (int)yarn.size();
        spec.vertexCount = (yarnSize - 3) * 4; // there are yarnSize bezier patches, each with 4 vertices (TODO: remove redundant vertex storage)
        // TODO: assign curve material
        spec.materialID = materialID;
        pScene->mCurveDesc.push_back(spec);

        for (int i = 3; i < yarnSize; i++)
        {
            // convert B-spline patches to bezier patches
            const int i0 = std::min(std::max(0, i - 3), int(yarnSize - 1));
            const int i1 = std::min(std::max(0, i - 2), int(yarnSize - 1));
            const int i2 = std::min(std::max(0, i - 1), int(yarnSize - 1));
            const int i3 = std::min(std::max(0, i + 0), int(yarnSize - 1));

            float3 p012 = yarn[i0];
            float3 p123 = yarn[i1];
            float3 p234 = yarn[i2];
            float3 p345 = yarn[i3];

            float3 p122 = lerp(p012, p123, float3(2.f / 3.f));
            float3 p223 = lerp(p123, p234, float3(1.f / 3.f));
            float3 p233 = lerp(p123, p234, float3(2.f / 3.f));
            float3 p334 = lerp(p234, p345, float3(1.f / 3.f));

            float3 p222 = lerp(p122, p223, float3(0.5f));
            float3 p333 = lerp(p233, p334, float3(0.5f));

            CurveVertexData vertexData[4];

            vertexData[0].position = float4(p222, width);
            vertexData[1].position = float4(p223, width);
            vertexData[2].position = float4(p233, width);
            vertexData[3].position = float4(p333, width);

            if (i == 3) vertexData[0].normal = float4(normalize(p223 - p222), 1);
            else
            {
                pScene->mCPUCurveVertexBuffer.back().normal = float4(normalize(float3(pScene->mCPUCurveVertexBuffer.back().normal) + (p223 - p222)), 1);
                vertexData[0].normal = float4(normalize((p222 - float3(pScene->mCPUCurveVertexBuffer.back().position)) + (p223 - p222)), 1);
            }
            vertexData[1].normal = float4(normalize((p223 - p222) + (p233 - p223)), 1);
            vertexData[2].normal = float4(normalize((p233 - p223) + (p333 - p233)), 1);
            if (i == yarnSize - 1) vertexData[3].normal = float4(normalize(p333 - p233), 1);
            else vertexData[3].normal = float4(p333 - p233, 1);

            pScene->mCPUCurveVertexBuffer.push_back(vertexData[0]);
            pScene->mCPUCurveVertexBuffer.push_back(vertexData[1]);
            pScene->mCPUCurveVertexBuffer.push_back(vertexData[2]);
            pScene->mCPUCurveVertexBuffer.push_back(vertexData[3]);
        }
    }
}

void CurveFileLoader::loadHairFile(std::shared_ptr<Scene> pScene, const std::string& filename, int materialID, float width)
{
    std::string filefullpath;
    findFileInDataDirectories(filename, filefullpath);
    cyHairFile cyhair;
    cyhair.Initialize();
    cyhair.LoadFromFile(filefullpath.c_str());

    std::vector<float3> dirs(cyhair.GetHeader().point_count);
    cyhair.FillDirectionArray((float*)dirs.data());

    int hairCount = cyhair.GetHeader().hair_count;
    const unsigned short* segments = cyhair.GetSegmentsArray();
    float3* points = (float3*)cyhair.GetPointsArray();

    // use line segments
    int pointIndex = 0;
    for (int hairIndex = 0; hairIndex < hairCount; hairIndex++) {
        int curveId = (int)pScene->mCurveDesc.size();
        CurveDesc spec;
        spec.curveType = 0;
        spec.vbOffset = (uint32_t)pScene->mCPUCurveVertexBuffer.size();
        uint32_t numSegments = segments ? (uint32_t)segments[hairIndex] : cyhair.GetHeader().d_segments;
        spec.vertexCount = numSegments * 2;
        // TODO: assign curve material
        spec.materialID = materialID;
        pScene->mCurveDesc.push_back(spec);
        for (uint32_t segId = 0; segId < numSegments; segId++)
        {
            CurveVertexData vertexData[2];
            vertexData[0].position = float4(points[pointIndex + segId], width);
            vertexData[1].position = float4(points[pointIndex + segId + 1], width);
            vertexData[0].normal = float4(dirs[pointIndex + segId], 1);
            vertexData[1].normal = float4(dirs[pointIndex + segId + 1], 1);

            std::swap(vertexData[0].position.y, vertexData[0].position.z);
            vertexData[0].position.z = -vertexData[0].position.z;
            std::swap(vertexData[1].position.y, vertexData[1].position.z);
            vertexData[1].position.z = -vertexData[1].position.z;

            std::swap(vertexData[0].normal.y, vertexData[0].normal.z);
            vertexData[0].normal.z = -vertexData[0].normal.z;
            std::swap(vertexData[1].normal.y, vertexData[1].normal.z);
            vertexData[1].normal.z = -vertexData[1].normal.z;

            pScene->mCPUCurveVertexBuffer.push_back(vertexData[0]);
            pScene->mCPUCurveVertexBuffer.push_back(vertexData[1]);
        }
        pointIndex += numSegments + 1;
    }

    // use bezier curve patches
    //using KnitData = std::vector<std::vector<float3>>;
    //KnitData	knit_data;
    //int pointIndex = 0;
    //for (int hairIndex = 0; hairIndex < hairCount; hairIndex++) {
    //    uint32_t numSegments = segments ? (uint32_t)segments[hairIndex] : cyhair.GetHeader().d_segments;
    //    std::vector<float3> yarn;
    //    for (uint32_t segId = 0; segId <= numSegments; segId++)
    //    {
    //        yarn.push_back(float3(points[pointIndex + segId]));
    //    }
    //    if (!yarn.empty()) knit_data.push_back(yarn);
    //    pointIndex += numSegments + 1;
    //}
    //GenerateBezierPatchesFromKnitData(knit_data);

}

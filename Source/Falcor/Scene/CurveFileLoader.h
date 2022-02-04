#pragma once
#include "Falcor.h"

using namespace Falcor;

class Falcor::Scene;

namespace Falcor
{
    class dlldecl CurveFileLoader
    {
    public:
        CurveFileLoader() {};

        void loadKnitCCPFile(std::shared_ptr<Scene> pScene, const std::string& filename, int materialID, float width = 1.f, float scale = 1.f, bool yz = true);
        void loadHairFile(std::shared_ptr<Scene> pScene, const std::string& filename, int materialID, float width = 1.f);

    private:

        class CurveIOBuffer
        {
            char data[1024];
            int readLine;
        public:
            int ReadLine(FILE* fp)
            {
                char c = fgetc(fp);
                while (!feof(fp)) {
                    while (isspace(c) && (!feof(fp) || c != '\0')) c = fgetc(fp);	// skip empty space
                    if (c == '#') while (!feof(fp) && c != '\n' && c != '\r' && c != '\0') c = fgetc(fp);	// skip comment line
                    else break;
                }
                int i = 0;
                bool inspace = false;
                while (i < 1024 - 1) {
                    if (feof(fp) || c == '\n' || c == '\r' || c == '\0') break;
                    if (isspace(c)) {	// only use a single space as the space character
                        inspace = true;
                    }
                    else {
                        if (inspace) data[i++] = ' ';
                        inspace = false;
                        data[i++] = c;
                    }
                    c = fgetc(fp);
                }
                data[i] = '\0';
                readLine = i;
                return i;
            }
            char& operator[](int i) { return data[i]; }
            void ReadVertex(float3& v) const { sscanf(data + 2, "%f %f %f", &v.x, &v.y, &v.z); }
            void ReadTexCrds(float2& v) const { sscanf(data + 3, "%f %f", &v.x, &v.y); }
            void ReadVertexYZ(float3& v) const { sscanf(data + 2, "%f %f %f", &v.x, &v.z, &v.y); }
            void ReadFloat(float& f, int start) { sscanf(data + start, "%f", &f); }
            void ReadInt(int& i, int start) { sscanf(data + start, "%d", &i); }
            void ReadFloat3(float3& v, int start) const { sscanf(data + start, "%f %f %f", &v.x, &v.y, &v.z); }
            void ReadString(std::string& s, int start) const { char str[128]; sscanf(data + start, "%s", str); s = str; }

            bool IsCommand(const char* cmd) const {
                int i = 0;
                while (cmd[i] != '\0') {
                    if (cmd[i] != data[i]) return false;
                    i++;
                }
                return (data[i] == '\0' || data[i] == ' ');
            }
            void Copy(char* a, int count, int start = 0) const {
                strncpy(a, data + start, count - 1);
                a[count - 1] = '\0';
            }
        };

        void GenerateBezierPatchesFromKnitData(std::shared_ptr<Scene> pScene, const std::vector<std::vector<float3>>& knitData, int materialID, float width);
    };
}

__exported import Reservoir;
#include "Utils/Math/MathConstants.slangh"

interface IArrayDataProvider
{
    ExtraBounceReservoir getData(int index);
};

struct StructuredBufferArrayDataProvider : IArrayDataProvider
{
    StructuredBuffer<ExtraBounceReservoir> data;
    ExtraBounceReservoir getData(int index)
    {
        return data[index];
    }

}

struct RWStructuredBufferArrayDataProvider : IArrayDataProvider
{
    RWStructuredBuffer<ExtraBounceReservoir> data;
    ExtraBounceReservoir getData(int index)
    {
        return data[index];
    }
}

struct ArrayArrayDataProvider : IArrayDataProvider
{
    bool excludeInitialSegment;

#if MAX_BOUNCES > 1
    ExtraBounceReservoir data[MAX_BOUNCES - 1];
    ExtraBounceReservoir getData(int index)
    {
        return data[index];
    }
#else
    ExtraBounceReservoir getData(int index)
    {
        return {};
    }
#endif
}


interface IExternalVisibilityProvider
{
    float getVisibility(int bounce);
    [mutating] void putVisibility(int bounce, float visibility);
    [mutating] void Initialize();
};

struct SingleExternalVisibilityProvider : IExternalVisibilityProvider
{
    float data = -1.f;
    float getVisibility(int bounce)
    {
        if (bounce == 0) return data;
        else return -1.f;
    }
    [mutating] void putVisibility(int bounce, float visibility) {}
    [mutating] void Initialize() { data = -1.f; }
};

struct MultipleExternalVisibilityProvider : IExternalVisibilityProvider
{
    bool enabled;
    float data[MAX_BOUNCES];
    float getVisibility(int bounce)
    {
        if (enabled) return data[bounce];
        else return -1;
    }
    [mutating] void putVisibility(int bounce, float visibility)
    {
        if (enabled) data[bounce] = visibility;
    }
    [mutating] void Initialize()
    {
        for (int i = 0; i < MAX_BOUNCES; i++)
            data[i] = -1.f;
    }

};


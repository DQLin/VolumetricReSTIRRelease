#ifdef HOST_CODE
#include "Falcor.h"
#endif

static const uint kMISNone = 0;
static const uint kMISTalbot = 1;

static const uint kHammersley = 0;
static const uint kR2 = 1;

static const uint kRatioTracking = 0;
static const uint kAnalyticTracking = 1;
static const uint kRayMarching = 2;
static const uint kResidualRatioTracking = 3;
static const uint kAnalogResidualRatioTracking = 4;

static const int MaxInitialSampleCount = 4;

static const int kSelfEmissionLightID = -3;

static const int kNumMaxMips = 8;

static const int kTemperatureGridId = 2 * kNumMaxMips;
static const int kVelocityGridId = 2 * kNumMaxMips + 1;
static const int kSuperVoxelGridId = 2 * kNumMaxMips + 2;

static const int kPrevFrameDensityGridOffset = 2 * kNumMaxMips + 3;
static const int kPrevFrameExtraGridOffset = 11;

static const int kReprojectionLinear = 0;
static const int kReprojectionNone = 1;
static const int kReprojectionNoBackground = 2;


#include "stdafx.h"
#include "BCHelper.h"

// Because these are used in SAL annotations, they need to remain macros rather than const values
#define BLOCK_LEN 4
// length of each block in texel

#define BLOCK_SIZE (BLOCK_LEN * BLOCK_LEN)
// total texels in a 4x4 block.

// Because these are used in SAL annotations, they need to remain macros rather than const values
#define NUM_PIXELS_PER_BLOCK 16


#pragma warning(push)
#pragma warning(disable : 4127)
template <bool bRange> void OptimizeAlpha(float* pX, float* pY, const float* pPoints, uint32_t cSteps) noexcept
{
    static const float pC6[] = { 5.0f / 5.0f, 4.0f / 5.0f, 3.0f / 5.0f, 2.0f / 5.0f, 1.0f / 5.0f, 0.0f / 5.0f };
    static const float pD6[] = { 0.0f / 5.0f, 1.0f / 5.0f, 2.0f / 5.0f, 3.0f / 5.0f, 4.0f / 5.0f, 5.0f / 5.0f };
    static const float pC8[] = { 7.0f / 7.0f, 6.0f / 7.0f, 5.0f / 7.0f, 4.0f / 7.0f, 3.0f / 7.0f, 2.0f / 7.0f, 1.0f / 7.0f, 0.0f / 7.0f };
    static const float pD8[] = { 0.0f / 7.0f, 1.0f / 7.0f, 2.0f / 7.0f, 3.0f / 7.0f, 4.0f / 7.0f, 5.0f / 7.0f, 6.0f / 7.0f, 7.0f / 7.0f };

    const float* pC = (6 == cSteps) ? pC6 : pC8;
    const float* pD = (6 == cSteps) ? pD6 : pD8;

    const float MAX_VALUE = 1.0f;
    const float MIN_VALUE = (bRange) ? -1.0f : 0.0f;

    // Find Min and Max points, as starting point
    float fX = MAX_VALUE;
    float fY = MIN_VALUE;

    if (8 == cSteps)
    {
        for (size_t iPoint = 0; iPoint < NUM_PIXELS_PER_BLOCK; iPoint++)
        {
            if (pPoints[iPoint] < fX)
                fX = pPoints[iPoint];

            if (pPoints[iPoint] > fY)
                fY = pPoints[iPoint];
        }
    }
    else
    {
        for (size_t iPoint = 0; iPoint < NUM_PIXELS_PER_BLOCK; iPoint++)
        {
            if (pPoints[iPoint] < fX && pPoints[iPoint] > MIN_VALUE)
                fX = pPoints[iPoint];

            if (pPoints[iPoint] > fY && pPoints[iPoint] < MAX_VALUE)
                fY = pPoints[iPoint];
        }

        if (fX == fY)
        {
            fY = MAX_VALUE;
        }
    }

    // Use Newton's Method to find local minima of sum-of-squares error.
    auto fSteps = static_cast<float>(cSteps - 1);

    for (size_t iIteration = 0; iIteration < 8; iIteration++)
    {
        if ((fY - fX) < (1.0f / 256.0f))
            break;

        float fScale = fSteps / (fY - fX);

        // Calculate new steps
        float pSteps[8];

        for (size_t iStep = 0; iStep < cSteps; iStep++)
            pSteps[iStep] = pC[iStep] * fX + pD[iStep] * fY;

        if (6 == cSteps)
        {
            pSteps[6] = MIN_VALUE;
            pSteps[7] = MAX_VALUE;
        }

        // Evaluate function, and derivatives
        float dX = 0.0f;
        float dY = 0.0f;
        float d2X = 0.0f;
        float d2Y = 0.0f;

        for (size_t iPoint = 0; iPoint < NUM_PIXELS_PER_BLOCK; iPoint++)
        {
            float fDot = (pPoints[iPoint] - fX) * fScale;

            uint32_t iStep;
            if (fDot <= 0.0f)
            {
                // D3DX10 / D3DX11 didn't take into account the proper minimum value for the bRange (BC4S/BC5S) case
                iStep = ((6 == cSteps) && (pPoints[iPoint] <= (fX + MIN_VALUE) * 0.5f)) ? 6u : 0u;
            }
            else if (fDot >= fSteps)
            {
                iStep = ((6 == cSteps) && (pPoints[iPoint] >= (fY + MAX_VALUE) * 0.5f)) ? 7u : (cSteps - 1);
            }
            else
            {
                iStep = uint32_t(fDot + 0.5f);
            }

            if (iStep < cSteps)
            {
                // D3DX had this computation backwards (pPoints[iPoint] - pSteps[iStep])
                // this fix improves RMS of the alpha component
                float fDiff = pSteps[iStep] - pPoints[iPoint];

                dX += pC[iStep] * fDiff;
                d2X += pC[iStep] * pC[iStep];

                dY += pD[iStep] * fDiff;
                d2Y += pD[iStep] * pD[iStep];
            }
        }

        // Move endpoints
        if (d2X > 0.0f)
            fX -= dX / d2X;

        if (d2Y > 0.0f)
            fY -= dY / d2Y;

        if (fX > fY)
        {
            float f = fX; fX = fY; fY = f;
        }

        if ((dX * dX < (1.0f / 64.0f)) && (dY * dY < (1.0f / 64.0f)))
            break;
    }

    *pX = (fX < MIN_VALUE) ? MIN_VALUE : (fX > MAX_VALUE) ? MAX_VALUE : fX;
    *pY = (fY < MIN_VALUE) ? MIN_VALUE : (fY > MAX_VALUE) ? MAX_VALUE : fY;
}
#pragma warning(pop)


void FindEndPointsBC4U(
    _In_reads_(BLOCK_SIZE) const float theTexelsU[],
    _Out_ uint8_t& endpointU_0,
    _Out_ uint8_t& endpointU_1) noexcept
{
    // The boundary of codec for signed/unsigned format
    const float MIN_NORM = 0.f;
    const float MAX_NORM = 1.f;

    // Find max/min of input texels
    float fBlockMax = theTexelsU[0];
    float fBlockMin = theTexelsU[0];
    for (size_t i = 0; i < BLOCK_SIZE; ++i)
    {
        if (theTexelsU[i] < fBlockMin)
        {
            fBlockMin = theTexelsU[i];
        }
        else if (theTexelsU[i] > fBlockMax)
        {
            fBlockMax = theTexelsU[i];
        }
    }

    //  If there are boundary values in input texels, should use 4 interpolated color values to guarantee
    //  the exact code of the boundary values.
    bool bUsing4BlockCodec = (MIN_NORM == fBlockMin || MAX_NORM == fBlockMax);

    // Using Optimize
    float fStart, fEnd;

    if (!bUsing4BlockCodec)
    {
        // 6 interpolated color values
        OptimizeAlpha<false>(&fStart, &fEnd, theTexelsU, 8);

        auto iStart = static_cast<uint8_t>(fStart * 255.0f);
        auto iEnd = static_cast<uint8_t>(fEnd * 255.0f);

        endpointU_0 = iEnd;
        endpointU_1 = iStart;
    }
    else
    {
        // 4 interpolated color values
        OptimizeAlpha<false>(&fStart, &fEnd, theTexelsU, 6);

        auto iStart = static_cast<uint8_t>(fStart * 255.0f);
        auto iEnd = static_cast<uint8_t>(fEnd * 255.0f);

        endpointU_1 = iEnd;
        endpointU_0 = iStart;
    }
}


void FindClosestUNORM(
    _Inout_ BC4_UNORM* pBC,
    _In_reads_(NUM_PIXELS_PER_BLOCK) const float theTexelsU[]) noexcept
{
    float rGradient[8];
    for (size_t i = 0; i < 8; ++i)
    {
        rGradient[i] = pBC->DecodeFromIndex(i);
    }

    for (size_t i = 0; i < NUM_PIXELS_PER_BLOCK; ++i)
    {
        size_t uBestIndex = 0;
        float fBestDelta = 100000;
        for (size_t uIndex = 0; uIndex < 8; uIndex++)
        {
            float fCurrentDelta = fabsf(rGradient[uIndex] - theTexelsU[i]);
            if (fCurrentDelta < fBestDelta)
            {
                uBestIndex = uIndex;
                fBestDelta = fCurrentDelta;
            }
        }
        pBC->SetIndex(i, uBestIndex);
    }
}


// if a float value is not zero, it is guaranteed not to be zero
void FindClosestUNORMConservative(
    _Inout_ BC4_UNORM* pBC,
    _In_reads_(NUM_PIXELS_PER_BLOCK) const float theTexelsU[]) noexcept
{
    float rGradient[8];
    for (size_t i = 0; i < 8; ++i)
    {
        rGradient[i] = pBC->DecodeFromIndex(i);
    }

    bool needRecompute = false;

    for (size_t i = 0; i < NUM_PIXELS_PER_BLOCK; ++i)
    {
        size_t uBestIndex = 0;
        float fBestDelta = 100000;
        for (size_t uIndex = 0; uIndex < 8; uIndex++)
        {
            //if (rGradient[uIndex] == 0.f && theTexelsU[i] > 0.f) continue;
            float fCurrentDelta = fabsf(rGradient[uIndex] - theTexelsU[i]);
            if (fCurrentDelta < fBestDelta)
            {
                uBestIndex = uIndex;
                fBestDelta = fCurrentDelta;
            }
        }

        pBC->SetIndex(i, uBestIndex);

        // need to raise end point to 1
        if (rGradient[uBestIndex] == 0.f && theTexelsU[i] > 0.f)
        {
            if (pBC->red_1 < pBC->red_0)
            {
                pBC->red_1 = 1;
                pBC->red_0 = std::max((uint8_t)1, pBC->red_0);
            }
            else
            {
                pBC->red_0 = 1;
                pBC->red_1 = std::max((uint8_t)1, pBC->red_1);
            }

            needRecompute = true;
            break;
        }
       // if (needRaiseEndPoints) printf("(%f %f) ", theTexelsU[i], pBC->DecodeFromIndex(uBestIndex));
    }

    if (needRecompute)
    {
        for (size_t i = 0; i < 8; ++i)
        {
            rGradient[i] = pBC->DecodeFromIndex(i);
        }

        for (size_t i = 0; i < NUM_PIXELS_PER_BLOCK; ++i)
        {
            size_t uBestIndex = 0;
            float fBestDelta = 100000;
            for (size_t uIndex = 0; uIndex < 8; uIndex++)
            {
                if (rGradient[uIndex] == 0.f && theTexelsU[i] > 0.f) continue;
                float fCurrentDelta = fabsf(rGradient[uIndex] - theTexelsU[i]);
                if (fCurrentDelta < fBestDelta)
                {
                    uBestIndex = uIndex;
                    fBestDelta = fCurrentDelta;
                }
            }

            pBC->SetIndex(i, uBestIndex);
        }
    }

    //printf("\n");
}

void BCHelper::CompressImage(float* data, int3 imgDim, float maxVal, bool isConservative, std::vector<uint8_t>& compressedData)
{
    int depth = imgDim[2];
    int height = imgDim[1];
    int width = imgDim[0];

    int paddedHeight = (height + 3) / 4 * 4;
    int paddedWidth = (width + 3) / 4 * 4;

    compressedData.resize(paddedHeight * paddedWidth * depth);

    int sliceOffset = 0;
    int offset = 0;

    for (int d = 0; d < depth; d++)
    {
        //
        float* curSliceData = &data[sliceOffset];

        for (int h = 0; h < height; h += 4)
        {
            int ph = std::min<int>(4, height - h);
            for (int w = 0; w < width; w += 4)
            {
                int pw = std::min<int>(4, width - w);

                float theTexelsU[NUM_PIXELS_PER_BLOCK];

                for (int j = 0; j < 4; ++j)
                {
                    for (int i = 0; i < 4; ++i)
                    {
                        int index = 4 * j + i;
                        if (i >= pw || j >= ph) theTexelsU[index] = 0.f;
                        else theTexelsU[index] = std::min(1.f, std::max(0.f, curSliceData[(h + j) * width + w + i] / maxVal));
                    }
                }

                uint8_t* pBC = compressedData.data() + offset;

                static_assert(sizeof(BC4_UNORM) == 8, "BC4_UNORM should be 8 bytes");

                memset(pBC, 0, sizeof(BC4_UNORM));

                auto pBC4 = reinterpret_cast<BC4_UNORM*>(pBC);

                FindEndPointsBC4U(theTexelsU, pBC4->red_0, pBC4->red_1);

                if (isConservative)
                    FindClosestUNORMConservative(pBC4, theTexelsU);
                else
                    FindClosestUNORM(pBC4, theTexelsU);

                offset += 8; // a BC4 block uses 8 bytes
            }
        }

        sliceOffset += height * width;
    }
}



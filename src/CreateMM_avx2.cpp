#include "simd_avx2.h"

void __stdcall
and_masks_simd_avx2(uint8_t* dstp0, uint8_t* dstp1, const uint8_t* srcp0,
    const uint8_t* srcp1, const uint8_t* srcp2, const int dpitch,
    const int spitch0, const int spitch1, const int spitch2,
    const int width, const int height) noexcept
{
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; x += sizeof(__m256i)) {
            __m256i s0 = loada<__m256i>(srcp0 + x);
            __m256i s1 = loada<__m256i>(srcp1 + x);
            __m256i s2 = loada<__m256i>(srcp2 + x);
            stream(dstp0 + x, and3(s0, s1, s2));
        }
        dstp0[-1] = dstp0[1];
        dstp0[width] = dstp0[width - 2];
        srcp0 += spitch0;
        srcp1 += spitch1;
        srcp2 += spitch2;
        dstp0 += dpitch;
    }
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; x += sizeof(__m256i)) {
            __m256i s0 = loada<__m256i>(srcp0 + x);
            __m256i s1 = loada<__m256i>(srcp1 + x);
            __m256i s2 = loada<__m256i>(srcp2 + x);
            stream(dstp1 + x, and3(s0, s1, s2));
        }
        dstp1[-1] = dstp1[1];
        dstp1[width] = dstp1[width - 2];
        srcp0 += spitch0;
        srcp1 += spitch1;
        srcp2 += spitch2;
        dstp1 += dpitch;
    }
}

void __stdcall
combine_masks_simd_avx2(uint8_t* dstp, const uint8_t* sqp, const uint8_t* shp,
    const int dpitch, const int spitch, const int width,
    const int height, const int _c) noexcept
{
    const uint8_t* sqp0 = sqp + spitch;
    const uint8_t* sqp1 = sqp;
    const uint8_t* sqp2 = sqp + spitch;

    const __m256i zero = setzero<__m256i>();
    const __m256i cstr = set1<__m256i>(static_cast<int8_t>(_c));

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; x += sizeof(__m256i)) {
            __m256i count = loadu<__m256i>(sqp0 + x - 1);
            count = adds(count, loada<__m256i>(sqp0 + x));
            count = adds(count, loadu<__m256i>(sqp0 + x + 1));
            count = adds(count, loadu<__m256i>(sqp1 + x - 1));
            __m256i sq = loada<__m256i>(sqp1 + x);
            count = adds(count, loadu<__m256i>(sqp1 + x + 1));
            count = adds(count, loadu<__m256i>(sqp2 + x - 1));
            count = adds(count, loada<__m256i>(sqp2 + x));
            count = adds(count, loadu<__m256i>(sqp2 + x + 1));

            count = cmpeq(max(count, cstr), count);
            sq = cmpgt(sq, zero);
            __m256i sh = loada<__m256i>(shp + x);
            sh = cmpgt(sh, zero);

            __m256i count2 = and_reg(count, sh);
            __m256i count3 = or_reg(count2, sq);

            stream(dstp + x, count3);
        }
        sqp0 = sqp1;
        sqp1 = sqp2;
        sqp2 += y < height - 1 ? spitch : -spitch;
        shp += spitch;
        dstp += dpitch;
    }
}

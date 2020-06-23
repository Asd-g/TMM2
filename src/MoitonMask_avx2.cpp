#include "simd_avx2.h"

void __stdcall
proc_simd_avx2(uint8_t* dqp, const uint8_t* mqp0, const uint8_t* mqp1,
    const int dpitch, const int spitch0, const int spitch1,
    const int width, const int height, const int8_t* params,
    const int*) noexcept
{
    const uint8_t* mhp0 = mqp0 + static_cast<int64_t>(spitch0) * height;
    const uint8_t* srcp0 = mhp0 + static_cast<int64_t>(spitch0) * height;
    const uint8_t* mhp1 = mqp1 + static_cast<int64_t>(spitch1) * height;
    const uint8_t* srcp1 = mhp1 + static_cast<int64_t>(spitch1) * height;
    uint8_t* dhp = dqp + static_cast<int64_t>(dpitch) * height;

    const __m256i nt = set1<__m256i>(params[0]);
    const __m256i minth = set1<__m256i>(params[1]);
    const __m256i maxth = set1<__m256i>(params[2]);
    const __m256i one = set1<__m256i>(0x01);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; x += sizeof(__m256i)) {
            const __m256i diff = absdiff(loada<__m256i>(srcp0 + x), loada<__m256i>(srcp1 + x));
            __m256i th = min(loada<__m256i>(mqp0 + x), loada<__m256i>(mqp1 + x));
            th = min(max(adds(th, nt), minth), maxth);
            stream(dqp + x, and_reg(cmple(diff, th), one));

            th = min(loada<__m256i>(mhp0 + x), loada<__m256i>(mhp1 + x));
            th = min(max(adds(th, nt), minth), maxth);
            stream(dhp + x, and_reg(cmple(diff, th), one));
        }
        mqp0 += spitch0;
        mhp0 += spitch0;
        srcp0 += spitch0;
        mqp1 += spitch1;
        mhp1 += spitch1;
        srcp1 += spitch1;
        dqp += dpitch;
        dhp += dpitch;
    }
}

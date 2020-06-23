#include "simd.h"

template <typename V, int TTYPE, int HS, int VS>
static __forceinline V
get_at_simd(const uint8_t* s0, const uint8_t* s1, const uint8_t* s2,
    const V& zero, const V& two, const V& mask) noexcept
{
    if (TTYPE == 0 || TTYPE == 2 || TTYPE == 4) {
        V up = loada<V>(s0);
        V left = loadu<V>(s1 - 1);
        V center = loada<V>(s1);
        V right = loadu<V>(s1 + 1);
        V down = loada<V>(s2);

        if (TTYPE == 0) {
            V min0 = absdiff(min(up, down), center);
            V max0 = absdiff(max(up, down), center);
            if (VS == 1) {
                min0 = avg(min0, zero);
                max0 = avg(max0, zero);
            }
            if (VS == 2) {
                min0 = rshift2(min0, two, mask);
                max0 = rshift2(max0, two, mask);
            }
            V min1 = absdiff(min(left, right), center);
            V max1 = absdiff(max(left, right), center);
            if (HS == 1) {
                min1 = avg(min1, zero);
                max1 = avg(max1, zero);
            }
            return max(min0, max0, min1, max1);
        }
        else if (TTYPE == 2) {
            V min0 = absdiff(min(left, right, up, down), center);
            V max0 = absdiff(max(left, right, up, down), center);
            return max(min0, max0);
        }
        else if (TTYPE == 4) {
            V min0 = min(min(up, down, left, right), center);
            V max0 = max(max(up, down, left, right), center);
            return subs(max0, min0);
        }
    }
    else {
        V ul = loadu<V>(s0 - 1);
        V up = loada<V>(s0);
        V ur = loadu<V>(s0 + 1);
        V left = loadu<V>(s1 - 1);
        V center = loada<V>(s1);
        V right = loadu<V>(s1 + 1);
        V dl = loadu<V>(s2 - 1);
        V down = loada<V>(s2);
        V dr = loadu<V>(s2 + 1);

        if (TTYPE == 1) {
            V min0 = absdiff(min(min(ul, ur, dl, dr), min(up, down)), center);
            V max0 = absdiff(max(max(ul, ur, dl, dr), max(up, down)), center);
            if (VS == 1) {
                min0 = avg(min0, zero);
                max0 = avg(max0, zero);
            }
            if (VS == 2) {
                min0 = rshift2(min0, two, mask);
                max0 = rshift2(max0, two, mask);
            }
            V min1 = absdiff(min(left, right), center);
            V max1 = absdiff(max(left, right), center);
            if (HS == 1) {
                min1 = avg(min1, zero);
                max1 = avg(max1, zero);
            }
            return max(min0, max0, min1, max1);
        }
        else if (TTYPE == 3) {
            V min0 = min(min(ul, ur, dl, dr), min(up, down, left, right));
            V max0 = max(max(ul, ur, dl, dr), max(up, down, left, right));
            return max(absdiff(min0, center), absdiff(max0, center));
        }
        else {
            V min0 = min(min(min(ul, ur, dl, dr), min(up, down, left, right)), center);
            V max0 = max(max(max(ul, ur, dl, dr), max(up, down, left, right)), center);
            return subs(max0, min0);
        }
    }
}

template <int TTYPE, int HS, int VS>
void __stdcall
proc_simd_sse2(uint8_t* dqp, const uint8_t* srcp, const int pitch,
    const int width, const int height, const int hs, const int vs) noexcept
{
    uint8_t* dhp = dqp + pitch * height;

    const uint8_t* s0 = srcp + pitch;
    const uint8_t* s1 = srcp;
    const uint8_t* s2 = srcp + pitch;

    const __m128i zero = setzero<__m128i>();
    const __m128i two = set1<__m128i>(0x02);
    const __m128i mask = set1<__m128i>(0x3F);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; x += sizeof(__m128i)) {
            __m128i at = get_at_simd<__m128i, TTYPE, HS, VS>(
                s0 + x, s1 + x, s2 + x, zero, two, mask);
            __m128i dq = rshift2(at, two, mask);
            __m128i dh = avg(at, zero);

            stream(dqp + x, dq);
            stream(dhp + x, dh);
        }
        s0 = s1;
        s1 = s2;
        s2 += (y < height - 2) ? pitch : -pitch;
        dqp += pitch;
        dhp += pitch;
    }
}

template void __stdcall proc_simd_sse2<0, 0, 1>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<1, 0, 1>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<2, 0, 1>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<3, 0, 1>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<4, 0, 1>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<5, 0, 1>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;

template void __stdcall proc_simd_sse2<0, 0, 2>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<1, 0, 2>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<2, 0, 2>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<3, 0, 2>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<4, 0, 2>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<5, 0, 2>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;

template void __stdcall proc_simd_sse2<0, 1, 1>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<1, 1, 1>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<2, 1, 1>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<3, 1, 1>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<4, 1, 1>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<5, 1, 1>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;

template void __stdcall proc_simd_sse2<0, 1, 2>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<1, 1, 2>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<2, 1, 2>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<3, 1, 2>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<4, 1, 2>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template void __stdcall proc_simd_sse2<5, 1, 2>(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;

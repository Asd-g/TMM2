/*
TMM2 - rewrite of TMM for Avisynth2.6/Avisynth+.
Copyright (C) 2016 OKA Motofumi

TMM - builds a motion-mask for TDeint.
Copyright (C) 2007 Kevin Stone

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/


#include "TMM2.h"

template <int TTYPE, int HS, int VS>
extern void __stdcall proc_simd_sse2(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;
template <int TTYPE, int HS, int VS>
extern void __stdcall proc_simd_avx2(uint8_t* dqp, const uint8_t* srcp, const int pitch, const int width, const int height, const int hs, const int vs) noexcept;

static __forceinline int absd(int x, int y) noexcept
{
    return x > y ? x - y : y - x;
}

template <int TTYPE>
static __forceinline int
get_at4_c(const int c, const int u, const int d, const int l, const int r,
    const int hs, const int vs) noexcept
{
    int min0 = u, max0 = u;

    if (TTYPE == 0) {
        int min1 = l, max1 = l;
        if (min1 > r) min1 = r;
        if (max1 < r) max1 = r;
        if (min0 > d) min0 = d;
        if (max0 < d) max0 = d;
        min0 = (absd(c, min0) + vs) >> vs;
        max0 = (absd(c, max0) + vs) >> vs;
        min1 = (absd(c, min1) + hs) >> hs;
        max1 = (absd(c, max1) + hs) >> hs;
        return std::max(std::max(min0, max0), std::max(min1, max1));
    }
    if (max0 < l) max0 = l;
    if (min0 > l) min0 = l;
    if (max0 < r) max0 = r;
    if (min0 > r) min0 = r;
    if (min0 > d) min0 = d;
    if (max0 < d) max0 = d;
    if (TTYPE == 2) {
        return std::max(absd(c, min0), absd(c, max0));
    }
    if (max0 < c) max0 = c;
    if (min0 > c) min0 = c;
    return max0 - min0;
}


template <int TTYPE>
static __forceinline int
get_at8_c(const int c, const int ul, const int u, const int ur, const int l,
    const int r, const int dl, const int d, const int dr, const int hs,
    const int vs) noexcept
{
    int min0 = ul, max0 = ul;
    if (max0 < u)  max0 = u;
    if (min0 > u)  min0 = u;
    if (max0 < ur) max0 = ur;
    if (min0 > ur) min0 = ur;

    if (TTYPE == 1) {
        int min1 = l, max1 = l;
        if (min1 > r) min1 = r;
        if (max1 < r) max1 = r;
        if (max0 < dl) max0 = dl;
        if (min0 > dl) min0 = dl;
        if (max0 < d)  max0 = d;
        if (min0 > d)  min0 = d;
        if (max0 < dr) max0 = dr;
        if (min0 > dr) min0 = dr;
        min0 = (absd(c, min0) + vs) >> vs;
        max0 = (absd(c, max0) + vs) >> vs;
        min1 = (absd(c, min1) + hs) >> hs;
        max1 = (absd(c, max1) + hs) >> hs;
        return std::max(std::max(min0, max0), std::max(min1, max1));
    }
    if (max0 < l) max0 = l;
    if (min0 > l) min0 = l;
    if (max0 < r) max0 = r;
    if (min0 > r) min0 = r;
    if (max0 < dl) max0 = dl;
    if (min0 > dl) min0 = dl;
    if (max0 < d)  max0 = d;
    if (min0 > d)  min0 = d;
    if (max0 < dr) max0 = dr;
    if (min0 > dr) min0 = dr;
    if (TTYPE == 3) {
        return std::max(absd(c, min0), absd(c, max0));
    }
    if (max0 < c) max0 = c;
    if (min0 > c) min0 = c;
    return max0 - min0;
}



template <int TTYPE>
static void __stdcall
proc_c(uint8_t* dqp, const uint8_t* srcp, const int pitch,
    const int width, const int height, const int hs, const int vs) noexcept
{
    uint8_t* dhp = dqp + static_cast<int64_t>(pitch) * height;

    const uint8_t* s0 = srcp + pitch;
    const uint8_t* s1 = srcp;
    const uint8_t* s2 = srcp + pitch;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int at;
            if (TTYPE == 0 || TTYPE == 2 || TTYPE == 4) {
                at = get_at4_c<TTYPE>(
                    s1[x], s0[x], s2[x], s1[x - 1], s1[x + 1], hs, vs);
            }
            else {
                at = get_at8_c<TTYPE>(
                    s1[x], s0[x - 1], s0[x], s0[x + 1], s1[x - 1], s1[x + 1],
                    s2[x - 1], s2[x], s2[x + 1], hs, vs);
            }
            dqp[x] = (at + 2) / 4;
            dhp[x] = (at + 1) / 2;
        }
        s0 = s1;
        s1 = s2;
        s2 += (y < height - 2) ? pitch : -pitch;
        dqp += pitch;
        dhp += pitch;
    }
}

static const proc_thmask functions[] = {
    proc_c<0>, proc_c<1>, proc_c<2>, proc_c<3>, proc_c<4>, proc_c<5>,
    // template <typename V, int TTYPE, int HS, int VS>
    proc_simd_sse2<0, 0, 1>, proc_simd_sse2<1, 0, 1>,
    proc_simd_sse2<2, 0, 1>, proc_simd_sse2<3, 0, 1>,
    proc_simd_sse2<4, 0, 1>, proc_simd_sse2<5, 0, 1>,

    proc_simd_sse2<0, 0, 2>, proc_simd_sse2<1, 0, 2>,
    proc_simd_sse2<2, 0, 2>, proc_simd_sse2<3, 0, 2>,
    proc_simd_sse2<4, 0, 2>, proc_simd_sse2<5, 0, 2>,

    proc_simd_sse2<0, 1, 1>, proc_simd_sse2<1, 1, 1>,
    proc_simd_sse2<2, 1, 1>, proc_simd_sse2<3, 1, 1>,
    proc_simd_sse2<4, 1, 1>, proc_simd_sse2<5, 1, 1>,

    proc_simd_sse2<0, 1, 2>, proc_simd_sse2<1, 1, 2>,
    proc_simd_sse2<2, 1, 2>, proc_simd_sse2<3, 1, 2>,
    proc_simd_sse2<4, 1, 2>, proc_simd_sse2<5, 1, 2>,

    proc_simd_avx2<0, 0, 1>, proc_simd_avx2<1, 0, 1>,
    proc_simd_avx2<2, 0, 1>, proc_simd_avx2<3, 0, 1>,
    proc_simd_avx2<4, 0, 1>, proc_simd_avx2<5, 0, 1>,

    proc_simd_avx2<0, 0, 2>, proc_simd_avx2<1, 0, 2>,
    proc_simd_avx2<2, 0, 2>, proc_simd_avx2<3, 0, 2>,
    proc_simd_avx2<4, 0, 2>, proc_simd_avx2<5, 0, 2>,

    proc_simd_avx2<0, 1, 1>, proc_simd_avx2<1, 1, 1>,
    proc_simd_avx2<2, 1, 1>, proc_simd_avx2<3, 1, 1>,
    proc_simd_avx2<4, 1, 1>, proc_simd_avx2<5, 1, 1>,

    proc_simd_avx2<0, 1, 2>, proc_simd_avx2<1, 1, 2>,
    proc_simd_avx2<2, 1, 2>, proc_simd_avx2<3, 1, 2>,
    proc_simd_avx2<4, 1, 2>, proc_simd_avx2<5, 1, 2>,
};


ThreshMask::
ThreshMask(PClip c, int ttype, int mtql, int mthl, int mtqc, int mthc,
           arch_t arch, IScriptEnvironment* env) :
    GVFmod(c, arch, env)
{
    mtq[0] = mtql; mtq[1] = mtq[2] = mtqc;
    mth[0] = mthl; mth[1] = mth[2] = mthc;

    hs[0] = 0;
    hs[1] = hs[2] = vi.IsYV24() ? 0 : 1;
    vs[0] = 1;
    vs[1] = vs[2] = vi.IsYV12() ? 2 : 1;

    vi.width += 4; // prepare for image edge processing
    vi.height *= 3;

    for (int i = 0; i < 3; ++i) {
        if (arch == arch_t::NO_SIMD) {
            proc[i] = functions[ttype];
            continue;
        }
        int idx = arch == arch_t::USE_SSE2 ? 6 : 30;
        idx += hs[i] * 12;
        idx += vs[i] == 2 ? 6 : 0;
        proc[i] = functions[idx + ttype];
    }
}


static inline void
mirror_copy(uint8_t* dstp, const int dpitch, const uint8_t* srcp,
            const int spitch, const int width, const int height) noexcept
{
    for (int y = 0; y < height; ++y) {
        memcpy(dstp, srcp, width);
        dstp[-1] = dstp[1];
        dstp[width] = dstp[width - 2];
        dstp += dpitch;
        srcp += spitch;
    }
}


PVideoFrame __stdcall ThreshMask::GetFrame(int n, IScriptEnvironment* env)
{
    auto src = child->GetFrame(n, env);
    PVideoFrame dst;
    if (has_at_least_v8) dst = env->NewVideoFrameP(vi, &src, align); else dst = env->NewVideoFrame(vi, align);

    for (int p = 0; p < numPlanes; ++p) {
        int plane = planes[p];

        const int pitch = dst->GetPitch(plane);
        const int width = src->GetRowSize(plane);
        const int height = src->GetHeight(plane);

        uint8_t* dstp = dst->GetWritePtr(plane);

        mirror_copy(dstp + static_cast<int64_t>(pitch) * height * 2, pitch, src->GetReadPtr(plane),
                    src->GetPitch(plane), width, height);

        const uint8_t* srcp = dstp + static_cast<int64_t>(pitch) * height * 2;

        if (mtq[p] > -1 && mth[p] > -1) {
            memset(dstp, mtq[p], static_cast<int64_t>(pitch) * height);
            memset(dstp + static_cast<int64_t>(pitch) * height, mth[p], static_cast<int64_t>(pitch)* height);
            continue;
        }

        proc[p](dstp, srcp, pitch, width, height, hs[p], vs[p]);

        if (mtq[p] > -1) {
            memset(dstp, mtq[p], static_cast<int64_t>(pitch) * height);
        } else if (mth[p] > -1) {
            memset(dstp + static_cast<int64_t>(pitch) * height, mth[p], static_cast<int64_t>(pitch) * height);
        }
    }

    return dst;
}
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

extern void __stdcall proc_simd_sse2(uint8_t* dqp, const uint8_t* mqp0, const uint8_t* mqp1, const int dpitch, const int spitch0, const int spitch1, const int width, const int height, const int8_t* params, const int*) noexcept;
extern void __stdcall proc_simd_avx2(uint8_t* dqp, const uint8_t* mqp0, const uint8_t* mqp1, const int dpitch, const int spitch0, const int spitch1, const int width, const int height, const int8_t* params, const int*) noexcept;

static __forceinline int absd(int x, int y) noexcept
{
    return x > y ? x - y : y - x;
}

static void __stdcall
proc_c(uint8_t* dqp, const uint8_t* mqp0, const uint8_t* mqp1,
       const int dpitch, const int spitch0, const int spitch1, const int width,
       const int height, const int8_t*, const int* mlut) noexcept
{
    const uint8_t* mhp0 = mqp0 + static_cast<int64_t>(spitch0) * height;
    const uint8_t* srcp0 = mhp0 + static_cast<int64_t>(spitch0) * height;
    const uint8_t* mhp1 = mqp1 + static_cast<int64_t>(spitch1) * height;
    const uint8_t* srcp1 = mhp1 + static_cast<int64_t>(spitch1) * height;
    uint8_t* dhp = dqp + static_cast<int64_t>(dpitch) * height;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int diff = absd(srcp0[x], srcp1[x]);
            int thq = std::min(mqp0[x], mqp1[x]);
            int thh = std::min(mhp0[x], mhp1[x]);
            dqp[x] = diff <= mlut[thq] ? 0x01 : 0;
            dhp[x] = diff <= mlut[thh] ? 0x01 : 0;
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

MotionMask::MotionMask(PClip tm, int minth, int maxth, int nt, int d,
                       arch_t arch, IScriptEnvironment* env) :
    GVFmod(tm, arch, env), distance(d)
{
    vi.width -= 4;
    vi.height = (vi.height / 3) * 2;

    params[0] = nt;
    params[1] = minth;
    params[2] = maxth;

    if (arch == arch_t::NO_SIMD) {
        mlut.resize(256);
        for (int i = 0; i < 256; ++i) {
            mlut[i] = clamp(i + nt, minth, maxth);
        }
    }
    switch (arch) {
    case arch_t::USE_AVX2: proc = proc_simd_avx2; break;
    case arch_t::USE_SSE2: proc = proc_simd_sse2; break;
    default:proc = proc_c; 
    }
}


PVideoFrame __stdcall MotionMask::GetFrame(int n, IScriptEnvironment* env)
{
    int nf = vi.num_frames - 1;
    PVideoFrame src1 = child->GetFrame(clamp(n + distance, 0, nf), env);
    PVideoFrame src0 = child->GetFrame(clamp(n, 0, nf), env);

    PVideoFrame dst;
    if (has_at_least_v8) dst = env->NewVideoFrameP(vi, &src0, align); else dst = env->NewVideoFrame(vi, align);

    for (int p = 0; p < numPlanes; ++p) {
        const int plane = planes[p];

        const int spitch0 = src0->GetPitch(plane);
        const int spitch1 = src1->GetPitch(plane);
        const int dpitch = dst->GetPitch(plane);
        const int width = dst->GetRowSize(plane);
        const int height = dst->GetHeight(plane) / 2;
        const uint8_t* mqp0 = src0->GetReadPtr(plane);
        const uint8_t* mqp1 = src1->GetReadPtr(plane);
        uint8_t* dstp = dst->GetWritePtr(plane);

        proc(dstp, mqp0, mqp1, dpitch, spitch0, spitch1, width, height,
             reinterpret_cast<int8_t*>(params), mlut.data());
    }

    return dst;
}

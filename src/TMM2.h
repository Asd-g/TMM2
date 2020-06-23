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


#ifndef TMM2_H
#define TMM2_H

#include <cstdint>
#include <algorithm>
#include <vector>
#include <emmintrin.h>
#include <immintrin.h>
#define WIN32_LEAN_AND_MEAN
#define VC_EXTRALEAN
#define NOMINMAX
#define NOGDI
#include <windows.h>
#include <stdexcept>

#include "avisynth.h"


#define TMM2_VERSION "0.1.4"

enum class arch_t {
    NO_SIMD,
    USE_SSE2,
    USE_AVX2,
};


class GVFmod : public GenericVideoFilter {
protected:
    const int planes[3] = { PLANAR_Y, PLANAR_U, PLANAR_V };
    const size_t align;
    int numPlanes;
    bool has_at_least_v8;

public:
    GVFmod(PClip c, arch_t a, IScriptEnvironment* env) :
        GenericVideoFilter(c), align(a == arch_t::USE_AVX2 ? 32 : 16)
    {
        numPlanes = vi.IsY8() ? 1 : 3;

        has_at_least_v8 = true;
        try { env->CheckVersion(8); }
        catch (const AvisynthError&) { has_at_least_v8 = false; }
    }
    int __stdcall SetCacheHints(int cache_hints, int frame_range) override
    {
        return cache_hints == CACHE_GET_MTMODE ? MT_NICE_FILTER : 0;
    }
};



typedef void(__stdcall *proc_thmask)(
    uint8_t*, const uint8_t*, const int, const int, const int, const int,
    const int);

class ThreshMask : public GVFmod {
    int mtq[3];
    int mth[3];
    int hs[3];
    int vs[3];

    proc_thmask proc[3];

public:
    ThreshMask(PClip child, int ttype, int mtqL, int mthL, int mtqC, int mthC,
               arch_t arch, IScriptEnvironment* env);
    PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env);
};



class MotionMask : public GVFmod {
    int distance;
    uint8_t params[3];
    std::vector<int> mlut;

    void(__stdcall *proc)(
        uint8_t* dstp, const uint8_t* srcp0, const uint8_t* srcp1,
        const int dpitch, const int spitch0, const int spitch1, const int width,
        const int height, const int8_t* params, const int* mlut);

public:
    MotionMask(PClip child, int minth, int maxth, int nt, int d, arch_t arch, IScriptEnvironment* env);
    PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env);
};


struct AndBuff {
    IScriptEnvironment* env;
    void* orig;
    uint8_t* am0;
    uint8_t* am1;
    const int pitch;
    bool has_at_least_v8;
    AndBuff(int width, int height, size_t align, IScriptEnvironment* e);
    ~AndBuff();
};

class CreateMM : public GVFmod {
    PClip mmask2;
    const int cstr;
    AndBuff* abuff;

    void(__stdcall *and_masks)(
        uint8_t* dstp0, uint8_t* dstp1, const uint8_t* srcp0,
        const uint8_t* srcp1, const uint8_t* srcp2, const int dpitch,
        const int spitch0, const int spitch1, const int spitch2,
        const int width, const int height);

    void(__stdcall *combine_masks)(
        uint8_t* dstp, const uint8_t* srcp0, const uint8_t* srcp1,
        const int dpitch, const int spitch, const int width, const int height,
        const int cstr);

public:
    CreateMM(PClip mm1, PClip mm2, int cstr, arch_t arch, bool is_avsplus, IScriptEnvironment* env);
    PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env);
};

class BuildMM : public GVFmod {
    PClip btmf;
    int nfSrc;
    PVideoFrame black;
    int order;
    int field;
    int mode;
    int length;
    int tmmlutf[2][64];
    std::vector<int> gvlut;
    const int* vlut;
    struct {
        int tstart;
        int tstop;
        int bstart;
        int bstop;
        int ocount;
        int ccount;
    } vals[2];


public:
    BuildMM(PClip topf, PClip btmf, int mode, int order, int field, int length,
            int mtype, arch_t arch, IScriptEnvironment* env);
    PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env);
    void setVals();
};


static __forceinline void validate(bool cond, const char* msg)
{
    if (cond) {
        throw std::runtime_error(msg);
    }
}


static __forceinline int clamp(int val, int minimum, int maximum)
{
    return std::min(std::max(val, minimum), maximum);
}
#endif // TMM2_H

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


#include <string>
#include "TMM2.h"


static inline arch_t get_arch(int opt, bool is_avsplus, ise_t* env)
{
    const bool has_sse2 = env->GetCPUFlags() & CPUF_SSE2;
    const bool has_avx2 = env->GetCPUFlags() & CPUF_AVX2;

    if (opt == 0 || !has_sse2) {
        return arch_t::NO_SIMD;
    }
#if !defined(__AVX2__)
    return USE_SSE2;
#else
    if (opt == 1 || !has_avx2 || !is_avsplus) {
        return arch_t::USE_SSE2;
    }
    return arch_t::USE_AVX2;
#endif
}


static AVSValue __cdecl create_tmm(AVSValue args, void*, ise_t* env)
{
    try {
        PClip orig = args[0].AsClip();
        const VideoInfo& vi = orig->GetVideoInfo();
        validate(vi.IsYV411(), "YV411 is not sapported.");
        validate(!vi.IsPlanar(), "clip is not planar format.");
        validate(vi.IsFieldBased(), "clip must be frame based.");

        int mode = args[1].AsInt(0);
        validate(mode != 0 && mode != 1, "mode must be set to 0 or 1.");

        int order = args[2].AsInt(-1);
        validate(order < -1 || order > 1, "order must be set to -1, 0 or 1.");
        if (order == -1) {
            order = orig->GetParity(0) ? 1 : 0;
        }

        int field = args[3].AsInt(-1);
        validate(field < -1 || field > 1, "field must be set to  -1, 0 or 1.");
        if (field == -1) {
            field = order;
        }

        int ttype = args[6].AsInt(1);
        int mtql = args[7].AsInt(-1);
        int mthl = args[8].AsInt(-1);
        int mtqc = args[9].AsInt(-1);
        int mthc = args[10].AsInt(-1);
        int length = args[4].AsInt(10);
        int mtype = args[5].AsInt(1);

        validate(ttype < 0 || ttype > 5,
                 "ttype must be set to 0, 1, 2, 3, 4 or 5.");
        validate(mtql < -1 || mtql > 255,
                 "mtqL must be between -1 and 255 inclusive.");
        validate(mtqc < -1 || mtqc > 255,
                 "mtqC must be between -1 and 255 inclusive.");
        validate(mthl < -1 || mthl > 255,
                 "mthL must be between -1 and 255 inclusive.");
        validate(mthc < -1 || mthc > 255,
                 "mthC must be between -1 and 255 inclusive.");
        validate(length < 6, "length must be greater than or equal to 6.");
        validate(mtype < 0 || mtype > 2, "mtype must be set to 0, 1 or 2");

        int nt = clamp(args[11].AsInt(2), 0, 255);
        int minth = clamp(args[12].AsInt(4), 0, 255);
        int maxth = clamp(args[13].AsInt(75), 0, 255);
        int cstr = clamp(args[14].AsInt(4), 0, 8);
        bool is_avsplus = env->FunctionExists("SetFilterMTMode");
        arch_t arch = get_arch(args[15].AsInt(-1), is_avsplus, env);

        orig = env->Invoke("SeparateFields", orig).AsClip();
        const char* filter[] = { "SelectEven", "SelectOdd" };
        int parity = orig->GetParity(0) ? 0 : 1;
        PClip topf = env->Invoke(filter[parity], orig).AsClip();
        PClip btmf = env->Invoke(filter[!parity], orig).AsClip();

        topf = new ThreshMask(topf, ttype, mtql, mthl, mtqc, mthc, arch, env);
        btmf = new ThreshMask(btmf, ttype, mtql, mthl, mtqc, mthc, arch, env);
        topf = env->Invoke("InternalCache", topf).AsClip();
        btmf = env->Invoke("InternalCache", btmf).AsClip();

        PClip topf0 = new MotionMask(topf, minth, maxth, nt, 1, arch, env);
        topf0 = env->Invoke("InternalCache", topf0).AsClip();
        PClip topf1 = new MotionMask(topf, minth, maxth, nt, 2, arch, env);

        PClip btmf0 = new MotionMask(topf, minth, maxth, nt, 1, arch, env);
        btmf0 = env->Invoke("InternalCache", btmf0).AsClip();
        PClip btmf1 = new MotionMask(btmf, minth, maxth, nt, 2, arch, env);

        topf = new CreateMM(topf0, topf1, cstr, arch, is_avsplus, env);
        topf = env->Invoke("InternalCache", topf).AsClip();

        btmf = new CreateMM(btmf0, btmf1, cstr, arch, is_avsplus, env);
        btmf = env->Invoke("InternalCache", btmf).AsClip();

        return new BuildMM(topf, btmf, mode, order, field, length, mtype, arch, env);

    } catch (AvisynthError& e) {
        env->ThrowError("TMM2: %s", e.msg);
    } catch (std::runtime_error& e) {
        env->ThrowError("TMM2: %s", e.what());
    }
    return 0;
}

const AVS_Linkage* AVS_linkage = nullptr;

extern "C" __declspec(dllexport) const char* __stdcall
AvisynthPluginInit3(ise_t* env, const AVS_Linkage* const vectors)
{
    AVS_linkage = vectors;

    const char* args =
        "c"             // 0
        "[mode]i"       // 1
        "[order]i"      // 2
        "[field]i"      // 3
        "[length]i"     // 4
        "[mtype]i"      // 5
        "[ttype]i"      // 6
        "[mtqL]i"       // 7
        "[mthL]i"       // 8
        "[mtqC]i"       // 9
        "[mthC]i"       //10
        "[nt]i"         //11
        "[minthresh]i"  //12
        "[maxthresh]i"  //13
        "[cstr]i"       //14
        "[opt]i";       //15

    env->AddFunction("TMM2", args, create_tmm, nullptr);

    return "TMM for avs2.6/avs+ ver. " TMM2_VERSION;
}
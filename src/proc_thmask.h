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


#ifndef PROC_THRESH_MASK_H
#define PROC_THRESH_MASK_H


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
        int min1 =l, max1 = l;
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
    uint8_t* dhp = dqp + pitch * height;

    const uint8_t* s0 = srcp + pitch;
    const uint8_t* s1 = srcp;
    const uint8_t* s2 = srcp + pitch;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int at;
            if (TTYPE == 0 || TTYPE == 2 || TTYPE == 4) {
                at = get_at4_c<TTYPE>(
                    s1[x], s0[x], s2[x], s1[x - 1], s1[x + 1], hs, vs);
            } else {
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

#endif // PROC_THRESH_MASK_H


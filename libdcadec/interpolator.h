/*
 * This file is part of libdcadec.
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at your
 * option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef INTERPOLATOR_H
#define INTERPOLATOR_H

#define MAX_LFE_HISTORY     12

struct interpolator;
struct idct_context;

typedef void (*interpolate_lfe_cb)(int *pcm_samples, int *lfe_samples,
                                   int npcmblocks, bool dec_select);

typedef void (*interpolate_sub_cb)(struct interpolator *dsp, int *pcm_samples,
                                   int **subband_samples_lo,
                                   int **subband_samples_hi,
                                   int nsamples, bool perfect);

struct interpolator {
    struct idct_context *idct;
    void *history;
    interpolate_sub_cb interpolate;
};

struct interpolator *interpolator_create(struct idct_context *parent, int flags)
    __attribute__((cold));
void interpolator_clear(struct interpolator *dsp) __attribute__((cold));

#define INTERPOLATE_LFE(x) \
    void interpolate_##x(int *pcm_samples, int *lfe_samples, \
                         int npcmblocks, bool dec_select)

#define INTERPOLATE_SUB(x) \
    void interpolate_##x(struct interpolator *dsp, int *pcm_samples, \
                         int **subband_samples_lo, \
                         int **subband_samples_hi, \
                         int nsamples, bool perfect)

INTERPOLATE_LFE(lfe_float_fir);
INTERPOLATE_LFE(lfe_float_fir_2x);
INTERPOLATE_LFE(lfe_float_iir);
INTERPOLATE_SUB(sub32_float);
INTERPOLATE_SUB(sub64_float);

INTERPOLATE_LFE(lfe_fixed_fir);
INTERPOLATE_SUB(sub32_fixed);
INTERPOLATE_SUB(sub64_fixed);

#endif

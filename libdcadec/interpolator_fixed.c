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

#include "common.h"
#include "interpolator.h"
#include "idct.h"
#include "fixed_math.h"
#include "fir_fixed.h"

INTERPOLATE_LFE(lfe_fixed_fir)
{
    (void)dec_select;
    assert(!dec_select);

    // Select decimation factor
    int nlfesamples = npcmblocks >> 1;

    // Interpolation
    for (int i = 0; i < nlfesamples; i++) {
        int *src = lfe_samples + MAX_LFE_HISTORY + i;

        // One decimated sample generates 64 interpolated ones
        for (int j = 0; j < 32; j++) {
            // Clear accumulation
            int64_t res1 = INT64_C(0);
            int64_t res2 = INT64_C(0);

            // Accumulate
            for (int k = 0; k < 8; k++) {
                res1 += (int64_t)lfe_fir_64[      j * 8 + k] * src[-k];
                res2 += (int64_t)lfe_fir_64[255 - j * 8 - k] * src[-k];
            }

            // Save interpolated samples
            pcm_samples[     j] = clip23(norm23(res1));
            pcm_samples[32 + j] = clip23(norm23(res2));
        }

        // Advance output pointer
        pcm_samples += 64;
    }

    // Update history
    for (int n = MAX_LFE_HISTORY - 1; n >= MAX_LFE_HISTORY - 8; n--)
        lfe_samples[n] = lfe_samples[nlfesamples + n];
}

INTERPOLATE_SUB(sub32_fixed)
{
    (void)subband_samples_hi;
    assert(!subband_samples_hi);

    // Get history pointer
    int *history = dsp->history;

    // Select filter
    const int32_t *filter_coeff = perfect ? band_fir_perfect : band_fir_nonperfect;

    // Interpolation begins
    for (int sample = 0; sample < nsamples; sample++) {
        int i, j, k;

        // Load in one sample from each subband
        int input[32];
        for (i = 0; i < 32; i++)
            input[i] = subband_samples_lo[i][sample];

        // Inverse DCT
        int output[32];
        idct_fixed32(input, output);

        // Store history
        for (i = 0, k = 31; i < 16; i++, k--) {
            history[     i] = clip23(output[i] - output[k]);
            history[16 + i] = clip23(output[i] + output[k]);
        }

        // One subband sample generates 32 interpolated ones
        for (i = 0, k = 15; i < 16; i++, k--) {
            // Clear accumulation
            int64_t res1 = INT64_C(0);
            int64_t res2 = INT64_C(0);

            // Accumulate
            for (j = 32; j < 512; j += 64) {
                res1 += (int64_t)history[16 + i + j] * filter_coeff[     i + j];
                res2 += (int64_t)history[16 + k + j] * filter_coeff[16 + i + j];
            }

            res1 = round21(res1);
            res2 = round21(res2);

            for (j = 0; j < 512; j += 64) {
                res1 += (int64_t)history[i + j] * filter_coeff[     i + j];
                res2 += (int64_t)history[k + j] * filter_coeff[16 + i + j];
            }

            // Save interpolated samples
            pcm_samples[     i] = clip23(norm21(res1));
            pcm_samples[16 + i] = clip23(norm21(res2));
        }

        // Advance output pointer
        pcm_samples += 32;

        // Shift history
        for (i = 511; i >= 32; i--)
            history[i] = history[i - 32];
    }
}

INTERPOLATE_SUB(sub64_fixed)
{
    (void)perfect;

    // Get history pointer
    int *history = dsp->history;

    // Interpolation begins
    for (int sample = 0; sample < nsamples; sample++) {
        int i, j, k;

        // Load in one sample from each subband
        int input[64];
        if (subband_samples_hi) {
            // Full 64 subbands, first 32 are residual coded
            for (i =  0; i < 32; i++)
                input[i] = subband_samples_lo[i][sample] + subband_samples_hi[i][sample];
            for (i = 32; i < 64; i++)
                input[i] = subband_samples_hi[i][sample];
        } else {
            // Only first 32 subbands
            for (i =  0; i < 32; i++)
                input[i] = subband_samples_lo[i][sample];
            for (i = 32; i < 64; i++)
                input[i] = 0;
        }

        // Inverse DCT
        int output[64];
        idct_fixed64(input, output);

        // Store history
        for (i = 0, k = 63; i < 32; i++, k--) {
            history[     i] = clip23(output[i] - output[k]);
            history[32 + i] = clip23(output[i] + output[k]);
        }

        // One subband sample generates 64 interpolated ones
        for (i = 0, k = 31; i < 32; i++, k--) {
            // Clear accumulation
            int64_t res1 = INT64_C(0);
            int64_t res2 = INT64_C(0);

            // Accumulate
            for (j = 64; j < 1024; j += 128) {
                res1 += (int64_t)history[32 + i + j] * band_fir_x96[     i + j];
                res2 += (int64_t)history[32 + k + j] * band_fir_x96[32 + i + j];
            }

            res1 = round20(res1);
            res2 = round20(res2);

            for (j = 0; j < 1024; j += 128) {
                res1 += (int64_t)history[i + j] * band_fir_x96[     i + j];
                res2 += (int64_t)history[k + j] * band_fir_x96[32 + i + j];
            }

            // Save interpolated samples
            pcm_samples[     i] = clip23(norm20(res1));
            pcm_samples[32 + i] = clip23(norm20(res2));
        }

        // Advance output pointer
        pcm_samples += 64;

        // Shift history
        for (i = 1023; i >= 64; i--)
            history[i] = history[i - 64];
    }
}

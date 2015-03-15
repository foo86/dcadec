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
#include "fir_float.h"

static const double lfe_iir_scale = 0.001985816114019982;

static const double lfe_iir[12] = {
    -0.9698530866436986,  1.9696040724997900,  0.9999999999999996,
    -1.9643358221499630, -0.9787938538720836,  1.9785545764679620,
     1.0000000000000020, -1.9951847249255470, -0.9925096137076496,
     1.9922787089263100,  1.0000000000000000, -1.9974180593495760
};

static double cos_mod_32[32][32];
static double filter_coeff_32[2][512];

static double cos_mod_64[64][64];
static double filter_coeff_64[1024];

static inline int convert(double a)
{
    if (a > 0x7fffff)
        return 0x7fffff;
    if (a < -0x800000)
        return -0x800000;
    return (int)floor(a + 0.5);
}

INTERPOLATE_LFE(lfe_float_fir)
{
    // Select decimation filter
    int dec_factor              = dec_select ? 128 : 64;
    const double *filter_coeff  = dec_select ? lfe_fir_128 : lfe_fir_64;

    // Interpolation
    for (int i = 0; i < nsamples; i++) {
        // One decimated sample generates 64 or 128 interpolated ones
        for (int j = 0; j < dec_factor; j++) {
            // Clear accumulation
            double res = 0.0;

            // Accumulate
            for (int k = 0; k < 512 / dec_factor; k++)
                res += lfe_samples[MAX_LFE_HISTORY + i - k] *
                    filter_coeff[k * dec_factor + j];

            // Save interpolated samples
            pcm_samples[(i * dec_factor + j) << synth_x96] = convert(res);
        }
    }

    // Update history
    for (int n = MAX_LFE_HISTORY - 1; n >= 0; n--)
        lfe_samples[n] = lfe_samples[nsamples + n];
}

INTERPOLATE_LFE(lfe_float_iir)
{
    // Select decimation factor
    int dec_factor = 64 << dec_select;

    // Load history
    double lfe_history[6];
    for (int i = 0; i < 6; i++)
        lfe_history[i] = ((double *)lfe_samples)[i];

    // Interpolation
    for (int i = 0; i < nsamples; i++) {
        double res1 = lfe_samples[MAX_LFE_HISTORY + i] * lfe_iir_scale;
        double res2;

        // One decimated sample generates 64 or 128 interpolated ones
        for (int j = 0; j < dec_factor; j++) {
            // Filter
            for (int k = 0; k < 3; k++) {
                double tmp1 = lfe_history[k * 2 + 0];
                double tmp2 = lfe_history[k * 2 + 1];

                res2 = tmp1 * lfe_iir[k * 4 + 0] + tmp2 * lfe_iir[k * 4 + 1] + res1;
                res1 = tmp1 * lfe_iir[k * 4 + 2] + tmp2 * lfe_iir[k * 4 + 3] + res2;

                lfe_history[k * 2 + 0] = tmp2;
                lfe_history[k * 2 + 1] = res2;
            }

            // Save interpolated samples
            pcm_samples[(i * dec_factor + j) << synth_x96] = convert(res1);
            res1 = 0.0;
        }
    }

    // Store history
    for (int i = 0; i < 6; i++)
        ((double *)lfe_samples)[i] = lfe_history[i];
}

INTERPOLATE_SUB(sub32_float)
{
    // Get history pointer
    double *history = dsp->history;

    // Select filter
    const double *filter_coeff = filter_coeff_32[perfect];

    // Interpolation begins
    for (int sample = 0; sample < nsamples; sample++) {
        int i, j, k;

        // Load in one sample from each subband
        double input[32];
        for (i = 0; i < nsubbands; i++)
            input[i] = subband_samples[i][sample];

        // Clear inactive subband_samples
        for (i = nsubbands; i < 32; i++)
            input[i] = 0;

        // Inverse DCT
        double output[32];
        for (i = 0; i < 32; i++) {
            double res = 0.0;
            for (j = 0; j < 32; j++)
                res += input[j] * cos_mod_32[i][j];
            output[i] = res;
        }

        // Store history
        for (i = 0, k = 31; i < 16; i++, k--) {
            history[     i] = output[i] - output[k];
            history[16 + i] = output[i] + output[k];
        }

        // One subband sample generates 32 interpolated ones
        for (i = 0; i < 16; i++) {
            // Clear accumulation
            double res = 0.0;

            // Accumulate
            for (j =  0; j < 512; j += 64)
                res += history[     i + j] * filter_coeff[i + j];
            for (j = 32; j < 512; j += 64)
                res += history[16 + i + j] * filter_coeff[i + j];

            // Save interpolated samples
            pcm_samples[sample * 32 + i] = convert(res);
        }

        for (i = 16, k = 15; i < 32; i++, k--) {
            // Clear accumulation
            double res = 0.0;

            // Accumulate
            for (j =  0; j < 512; j += 64)
                res += history[     k + j] * filter_coeff[i + j];
            for (j = 32; j < 512; j += 64)
                res += history[16 + k + j] * filter_coeff[i + j];

            // Save interpolated samples
            pcm_samples[sample * 32 + i] = convert(res);
        }

        // Shift history
        for (i = 511; i >= 32; i--)
            history[i] = history[i - 32];
    }
}

INTERPOLATE_SUB(sub64_float)
{
    (void)perfect;

    // Get history pointer
    double *history = dsp->history;

    // Interpolation begins
    for (int sample = 0; sample < nsamples; sample++) {
        int i, j, k;

        // Load in one sample from each subband
        double input[64];
        for (i = 0; i < nsubbands; i++)
            input[i] = subband_samples[i][sample];

        // Clear inactive subband_samples
        for (i = nsubbands; i < 64; i++)
            input[i] = 0;

        // Inverse DCT
        double output[64];
        for (i = 0; i < 64; i++) {
            double res = 0.0;
            for (j = 0; j < 64; j++)
                res += input[j] * cos_mod_64[i][j];
            output[i] = res;
        }

        // Store history
        for (i = 0, k = 63; i < 32; i++, k--) {
            history[     i] = output[i] - output[k];
            history[32 + i] = output[i] + output[k];
        }

        // One subband sample generates 64 interpolated ones
        for (i = 0; i < 32; i++) {
            // Clear accumulation
            double res = 0.0;

            // Accumulate
            for (j =  0; j < 1024; j += 128)
                res += history[     i + j] * filter_coeff_64[i + j];
            for (j = 64; j < 1024; j += 128)
                res += history[32 + i + j] * filter_coeff_64[i + j];

            // Save interpolated samples
            pcm_samples[sample * 64 + i] = convert(res);
        }

        for (i = 32, k = 31; i < 64; i++, k--) {
            // Clear accumulation
            double res = 0.0;

            // Accumulate
            for (j =  0; j < 1024; j += 128)
                res += history[     k + j] * filter_coeff_64[i + j];
            for (j = 64; j < 1024; j += 128)
                res += history[32 + k + j] * filter_coeff_64[i + j];

            // Save interpolated samples
            pcm_samples[sample * 64 + i] = convert(res);
        }

        // Shift history
        for (i = 1023; i >= 64; i--)
            history[i] = history[i - 64];
    }
}

void interpolate_sub32_float_init(void)
{
    static bool initialized;

    if (initialized)
        return;

    for (int i = 0; i < 32; i++)
        for (int j = 0; j < 32; j++)
            cos_mod_32[i][j] = 0.25 * cos((2 * i + 1) * (2 * j + 1) * M_PI / 128);

    // Pre-scale floating point FIR coefficients
    for (int i = 0; i < 512; i++) {
        filter_coeff_32[0][i] = band_fir_nonperfect[i] * M_SQRT2 * 128;
        filter_coeff_32[1][i] = band_fir_perfect[i] * M_SQRT2 * 128;
        if ((i & 63) >= 16) {
            // Invert sign to make all equations positive
            filter_coeff_32[0][i] = -filter_coeff_32[0][i];
            filter_coeff_32[1][i] = -filter_coeff_32[1][i];
        }
    }

    initialized = true;
}

void interpolate_sub64_float_init(void)
{
    static bool initialized;

    if (initialized)
        return;

    for (int i = 0; i < 64; i++)
        for (int j = 0; j < 64; j++)
            cos_mod_64[i][j] = 0.125 * cos((2 * i + 1) * (2 * j + 1) * M_PI / 256);

    // Pre-scale floating point FIR coefficients
    for (int i = 0; i < 1024; i++) {
        filter_coeff_64[i] = band_fir_x96[i] * M_SQRT2 * 512;
        if ((i & 127) >= 32)
            // Invert sign to make all equations positive
            filter_coeff_64[i] = -filter_coeff_64[i];
    }

    initialized = true;
}

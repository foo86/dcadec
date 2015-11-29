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
#include "idct.h"

struct idct_context *idct_init(struct core_decoder *parent)
{
    int i, j, k;

    struct idct_context *idct = ta_new(parent, struct idct_context);
    if (!idct)
        return NULL;

    for (i = 0; i < DCT_A_ROWS; i++) {
        for (j = 0, k = DCT_A_COLS - 1; j < DCT_A_COLS; j++, k--) {
            if (i & 1)
                idct->dct_a[i][j] = -sin((2 * i + 1) * (2 * k + 1) * M_PI / 32);
            else
                idct->dct_a[i][j] =  sin((2 * i + 1) * (2 * k + 1) * M_PI / 32);
        }
    }

    for (i = 0; i < DCT_B_ROWS; i++)
        for (j = 0; j < DCT_B_COLS; j++)
            idct->dct_b[i][j] = cos((2 * i + 1) * (1 + j) * M_PI / 16);

    for (i = 0; i < MOD_A_HALF; i++)
        idct->mod_a[i] =  0.5 / cos((2 * i + 1) * M_PI / 64);

    for (i = MOD_A_HALF, k = MOD_A_HALF - 1; i < MOD_A_SIZE; i++, k--)
        idct->mod_a[i] = -0.5 / sin((2 * k + 1) * M_PI / 64);

    for (i = 0; i < MOD_B_HALF; i++)
        idct->mod_b[i] = 0.5 / cos((2 * i + 1) * M_PI / 32);

    for (i = MOD_B_HALF, k = MOD_B_HALF - 1; i < MOD_B_SIZE; i++, k--)
        idct->mod_b[i] = 0.5 / sin((2 * k + 1) * M_PI / 32);

    for (i = 0; i < MOD_C_HALF; i++)
        idct->mod_c[i] =  0.125 / cos((2 * i + 1) * M_PI / 128);

    for (i = MOD_C_HALF, k = MOD_C_HALF - 1; i < MOD_C_SIZE; i++, k--)
        idct->mod_c[i] = -0.125 / sin((2 * k + 1) * M_PI / 128);

    for (i = 0; i < MOD64_A_HALF; i++)
        idct->mod64_a[i] =  0.5 / cos((2 * i + 1) * M_PI / 128);

    for (i = MOD64_A_HALF, k = MOD64_A_HALF - 1; i < MOD64_A_SIZE; i++, k--)
        idct->mod64_a[i] = -0.5 / sin((2 * k + 1) * M_PI / 128);

    for (i = 0; i < MOD64_B_HALF; i++)
        idct->mod64_b[i] = 0.5 / cos((2 * i + 1) * M_PI / 64);

    for (i = MOD64_B_HALF, k = MOD64_B_HALF - 1; i < MOD64_B_SIZE; i++, k--)
        idct->mod64_b[i] = 0.5 / sin((2 * k + 1) * M_PI / 64);

    for (i = 0; i < MOD64_C_HALF; i++)
        idct->mod64_c[i] =  0.125 / cos((2 * i + 1) * M_PI / 256);

    for (i = MOD64_C_HALF, k = MOD64_C_HALF - 1; i < MOD64_C_SIZE; i++, k--)
        idct->mod64_c[i] = -0.125 / sin((2 * k + 1) * M_PI / 256);

    return idct;
}

static void sum_a(const double * restrict input, double * restrict output, int len)
{
    for (int i = 0; i < len; i++)
        output[i] = input[2 * i] + input[2 * i + 1];
}

static void sum_b(const double * restrict input, double * restrict output, int len)
{
    output[0] = input[0];
    for (int i = 1; i < len; i++)
        output[i] = input[2 * i] + input[2 * i - 1];
}

static void sum_c(const double * restrict input, double * restrict output, int len)
{
    for (int i = 0; i < len; i++)
        output[i] = input[2 * i];
}

static void sum_d(const double * restrict input, double * restrict output, int len)
{
    output[0] = input[1];
    for (int i = 1; i < len; i++)
        output[i] = input[2 * i - 1] + input[2 * i + 1];
}

static void dct_a(const struct idct_context * restrict idct,
                  const double * restrict input, double * restrict output)
{
    for (int i = 0; i < DCT_A_ROWS; i++) {
        double res = 0.0;
        for (int j = 0; j < DCT_A_COLS; j++)
            res += idct->dct_a[i][j] * input[j];
        output[i] = res;
    }
}

static void dct_b(const struct idct_context * restrict idct,
                  const double * restrict input, double * restrict output)
{
    for (int i = 0; i < DCT_B_ROWS; i++) {
        double res = input[0];
        for (int j = 0; j < DCT_B_COLS; j++)
            res += idct->dct_b[i][j] * input[1 + j];
        output[i] = res;
    }
}

static void mod_a(const struct idct_context * restrict idct,
                  const double * restrict input, double * restrict output)
{
    for (int i = 0; i < MOD_A_HALF; i++)
        output[i] = idct->mod_a[i] * (input[i] + input[MOD_A_HALF + i]);

    for (int i = MOD_A_HALF, k = MOD_A_HALF - 1; i < MOD_A_SIZE; i++, k--)
        output[i] = idct->mod_a[i] * (input[k] - input[MOD_A_HALF + k]);
}

static void mod_b(const struct idct_context * restrict idct,
                  double * restrict input, double * restrict output)
{
    for (int i = 0; i < MOD_B_SIZE; i++) {
        input[MOD_B_SIZE + i] = idct->mod_b[i] * input[MOD_B_SIZE + i];
        output[i] = input[i] + input[MOD_B_SIZE + i];
    }

    for (int i = 0, k = MOD_B_SIZE - 1; i < MOD_B_SIZE; i++, k--)
        output[MOD_B_SIZE + i] = input[k] - input[MOD_B_SIZE + k];
}

static void mod_c(const struct idct_context * restrict idct,
                  const double * restrict input, double * restrict output)
{
    for (int i = 0; i < MOD_C_HALF; i++)
        output[i] = idct->mod_c[i] * (input[i] + input[MOD_C_HALF + i]);

    for (int i = MOD_C_HALF, k = MOD_C_HALF - 1; i < MOD_C_SIZE; i++, k--)
        output[i] = idct->mod_c[i] * (input[k] - input[MOD_C_HALF + k]);
}

void idct_perform32_float(const struct idct_context * restrict idct,
                          double * restrict input, double * restrict output)
{
    sum_a(input, output +  0, 16);
    sum_b(input, output + 16, 16);

    sum_a(output +  0, input +  0, 8);
    sum_b(output +  0, input +  8, 8);
    sum_c(output + 16, input + 16, 8);
    sum_d(output + 16, input + 24, 8);

    dct_a(idct, input +  0, output +  0);
    dct_b(idct, input +  8, output +  8);
    dct_b(idct, input + 16, output + 16);
    dct_b(idct, input + 24, output + 24);

    mod_a(idct, output +  0, input +  0);
    mod_b(idct, output + 16, input + 16);

    mod_c(idct, input, output);
}

static void mod64_a(const struct idct_context * restrict idct,
                    const double * restrict input, double * restrict output)
{
    for (int i = 0; i < MOD64_A_HALF; i++)
        output[i] = idct->mod64_a[i] * (input[i] + input[MOD64_A_HALF + i]);

    for (int i = MOD64_A_HALF, k = MOD64_A_HALF - 1; i < MOD64_A_SIZE; i++, k--)
        output[i] = idct->mod64_a[i] * (input[k] - input[MOD64_A_HALF + k]);
}

static void mod64_b(const struct idct_context * restrict idct,
                    double * restrict input, double * restrict output)
{
    for (int i = 0; i < MOD64_B_SIZE; i++) {
        input[MOD64_B_SIZE + i] = idct->mod64_b[i] * input[MOD64_B_SIZE + i];
        output[i] = input[i] + input[MOD64_B_SIZE + i];
    }

    for (int i = 0, k = MOD64_B_SIZE - 1; i < MOD64_B_SIZE; i++, k--)
        output[MOD64_B_SIZE + i] = input[k] - input[MOD64_B_SIZE + k];
}

static void mod64_c(const struct idct_context * restrict idct,
                    const double * restrict input, double * restrict output)
{
    for (int i = 0; i < MOD64_C_HALF; i++)
        output[i] = idct->mod64_c[i] * (input[i] + input[MOD64_C_HALF + i]);

    for (int i = MOD64_C_HALF, k = MOD64_C_HALF - 1; i < MOD64_C_SIZE; i++, k--)
        output[i] = idct->mod64_c[i] * (input[k] - input[MOD64_C_HALF + k]);
}

void idct_perform64_float(const struct idct_context * restrict idct,
                          double * restrict input, double * restrict output)
{
    sum_a(input, output +  0, 32);
    sum_b(input, output + 32, 32);

    sum_a(output +  0, input +  0, 16);
    sum_b(output +  0, input + 16, 16);
    sum_c(output + 32, input + 32, 16);
    sum_d(output + 32, input + 48, 16);

    sum_a(input +  0, output +  0, 8);
    sum_b(input +  0, output +  8, 8);
    sum_c(input + 16, output + 16, 8);
    sum_d(input + 16, output + 24, 8);
    sum_c(input + 32, output + 32, 8);
    sum_d(input + 32, output + 40, 8);
    sum_c(input + 48, output + 48, 8);
    sum_d(input + 48, output + 56, 8);

    dct_a(idct, output +  0, input +  0);
    dct_b(idct, output +  8, input +  8);
    dct_b(idct, output + 16, input + 16);
    dct_b(idct, output + 24, input + 24);
    dct_b(idct, output + 32, input + 32);
    dct_b(idct, output + 40, input + 40);
    dct_b(idct, output + 48, input + 48);
    dct_b(idct, output + 56, input + 56);

    mod_a(idct, input +  0, output +  0);
    mod_b(idct, input + 16, output + 16);
    mod_b(idct, input + 32, output + 32);
    mod_b(idct, input + 48, output + 48);

    mod64_a(idct, output +  0, input +  0);
    mod64_b(idct, output + 32, input + 32);

    mod64_c(idct, input, output);
}

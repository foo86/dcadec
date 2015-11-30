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
    sum_a(input, output + 0 * IDCT_SIZE_2, IDCT_SIZE_2);
    sum_b(input, output + 1 * IDCT_SIZE_2, IDCT_SIZE_2);

    sum_a(output + 0 * IDCT_SIZE_2, input + 0 * IDCT_SIZE_4, IDCT_SIZE_4);
    sum_b(output + 0 * IDCT_SIZE_2, input + 1 * IDCT_SIZE_4, IDCT_SIZE_4);
    sum_c(output + 1 * IDCT_SIZE_2, input + 2 * IDCT_SIZE_4, IDCT_SIZE_4);
    sum_d(output + 1 * IDCT_SIZE_2, input + 3 * IDCT_SIZE_4, IDCT_SIZE_4);

    dct_a(idct, input + 0 * IDCT_SIZE_4, output + 0 * IDCT_SIZE_4);
    dct_b(idct, input + 1 * IDCT_SIZE_4, output + 1 * IDCT_SIZE_4);
    dct_b(idct, input + 2 * IDCT_SIZE_4, output + 2 * IDCT_SIZE_4);
    dct_b(idct, input + 3 * IDCT_SIZE_4, output + 3 * IDCT_SIZE_4);

    mod_a(idct, output + 0 * IDCT_SIZE_2, input + 0 * IDCT_SIZE_2);
    mod_b(idct, output + 1 * IDCT_SIZE_2, input + 1 * IDCT_SIZE_2);

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
    sum_a(input, output + 0 * IDCT64_SIZE_2, IDCT64_SIZE_2);
    sum_b(input, output + 1 * IDCT64_SIZE_2, IDCT64_SIZE_2);

    sum_a(output + 0 * IDCT64_SIZE_2, input + 0 * IDCT64_SIZE_4, IDCT64_SIZE_4);
    sum_b(output + 0 * IDCT64_SIZE_2, input + 1 * IDCT64_SIZE_4, IDCT64_SIZE_4);
    sum_c(output + 1 * IDCT64_SIZE_2, input + 2 * IDCT64_SIZE_4, IDCT64_SIZE_4);
    sum_d(output + 1 * IDCT64_SIZE_2, input + 3 * IDCT64_SIZE_4, IDCT64_SIZE_4);

    sum_a(input + 0 * IDCT64_SIZE_4, output + 0 * IDCT64_SIZE_8, IDCT64_SIZE_8);
    sum_b(input + 0 * IDCT64_SIZE_4, output + 1 * IDCT64_SIZE_8, IDCT64_SIZE_8);
    sum_c(input + 1 * IDCT64_SIZE_4, output + 2 * IDCT64_SIZE_8, IDCT64_SIZE_8);
    sum_d(input + 1 * IDCT64_SIZE_4, output + 3 * IDCT64_SIZE_8, IDCT64_SIZE_8);
    sum_c(input + 2 * IDCT64_SIZE_4, output + 4 * IDCT64_SIZE_8, IDCT64_SIZE_8);
    sum_d(input + 2 * IDCT64_SIZE_4, output + 5 * IDCT64_SIZE_8, IDCT64_SIZE_8);
    sum_c(input + 3 * IDCT64_SIZE_4, output + 6 * IDCT64_SIZE_8, IDCT64_SIZE_8);
    sum_d(input + 3 * IDCT64_SIZE_4, output + 7 * IDCT64_SIZE_8, IDCT64_SIZE_8);

    dct_a(idct, output + 0 * IDCT64_SIZE_8, input + 0 * IDCT64_SIZE_8);
    dct_b(idct, output + 1 * IDCT64_SIZE_8, input + 1 * IDCT64_SIZE_8);
    dct_b(idct, output + 2 * IDCT64_SIZE_8, input + 2 * IDCT64_SIZE_8);
    dct_b(idct, output + 3 * IDCT64_SIZE_8, input + 3 * IDCT64_SIZE_8);
    dct_b(idct, output + 4 * IDCT64_SIZE_8, input + 4 * IDCT64_SIZE_8);
    dct_b(idct, output + 5 * IDCT64_SIZE_8, input + 5 * IDCT64_SIZE_8);
    dct_b(idct, output + 6 * IDCT64_SIZE_8, input + 6 * IDCT64_SIZE_8);
    dct_b(idct, output + 7 * IDCT64_SIZE_8, input + 7 * IDCT64_SIZE_8);

    mod_a(idct, input + 0 * IDCT64_SIZE_4, output + 0 * IDCT64_SIZE_4);
    mod_b(idct, input + 1 * IDCT64_SIZE_4, output + 1 * IDCT64_SIZE_4);
    mod_b(idct, input + 2 * IDCT64_SIZE_4, output + 2 * IDCT64_SIZE_4);
    mod_b(idct, input + 3 * IDCT64_SIZE_4, output + 3 * IDCT64_SIZE_4);

    mod64_a(idct, output + 0 * IDCT64_SIZE_2, input + 0 * IDCT64_SIZE_2);
    mod64_b(idct, output + 1 * IDCT64_SIZE_2, input + 1 * IDCT64_SIZE_2);

    mod64_c(idct, input, output);
}

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

#ifndef IDCT_H
#define IDCT_H

#define DCT_A_ROWS      8
#define DCT_A_COLS      8
#define DCT_B_ROWS      8
#define DCT_B_COLS      7

#define IDCT_SIZE       32
#define IDCT_SIZE_2     (IDCT_SIZE / 2)
#define IDCT_SIZE_4     (IDCT_SIZE / 4)

#define MOD_A_SIZE      IDCT_SIZE_2
#define MOD_B_SIZE      IDCT_SIZE_4
#define MOD_C_SIZE      IDCT_SIZE
#define MOD_A_HALF      (MOD_A_SIZE / 2)
#define MOD_B_HALF      (MOD_B_SIZE / 2)
#define MOD_C_HALF      (MOD_C_SIZE / 2)

#define IDCT64_SIZE     64
#define IDCT64_SIZE_2   (IDCT64_SIZE / 2)
#define IDCT64_SIZE_4   (IDCT64_SIZE / 4)
#define IDCT64_SIZE_8   (IDCT64_SIZE / 8)

#define MOD64_A_SIZE    IDCT64_SIZE_2
#define MOD64_B_SIZE    IDCT64_SIZE_4
#define MOD64_C_SIZE    IDCT64_SIZE
#define MOD64_A_HALF    (MOD64_A_SIZE / 2)
#define MOD64_B_HALF    (MOD64_B_SIZE / 2)
#define MOD64_C_HALF    (MOD64_C_SIZE / 2)

struct core_decoder;

struct idct_context {
    double dct_a[DCT_A_ROWS][DCT_A_COLS];
    double dct_b[DCT_B_ROWS][DCT_B_COLS];

    double mod_a[MOD_A_SIZE];
    double mod_b[MOD_B_SIZE];
    double mod_c[MOD_C_SIZE];

    double mod64_a[MOD64_A_SIZE];
    double mod64_b[MOD64_B_SIZE];
    double mod64_c[MOD64_C_SIZE];
};

struct idct_context *idct_init(struct core_decoder *parent) __attribute__((cold));

void idct_perform32_float(const struct idct_context * restrict idct,
                          double * restrict input, double * restrict output);
void idct_perform64_float(const struct idct_context * restrict idct,
                          double * restrict input, double * restrict output);

void idct_perform32_fixed(int * restrict input, int * restrict output);
void idct_perform64_fixed(int * restrict input, int * restrict output);

#endif

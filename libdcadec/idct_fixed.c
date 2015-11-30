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
#include "fixed_math.h"
#include "idct.h"

static void sum_a(const int * restrict input, int * restrict output, int len)
{
    for (int i = 0; i < len; i++)
        output[i] = input[2 * i] + input[2 * i + 1];
}

static void sum_b(const int * restrict input, int * restrict output, int len)
{
    output[0] = input[0];
    for (int i = 1; i < len; i++)
        output[i] = input[2 * i] + input[2 * i - 1];
}

static void sum_c(const int * restrict input, int * restrict output, int len)
{
    for (int i = 0; i < len; i++)
        output[i] = input[2 * i];
}

static void sum_d(const int * restrict input, int * restrict output, int len)
{
    output[0] = input[1];
    for (int i = 1; i < len; i++)
        output[i] = input[2 * i - 1] + input[2 * i + 1];
}

static void dct_a(const int * restrict input, int * restrict output)
{
    //  floor(sin((2 * i + 1) * (2 * (7 - j) + 1) * PI / 32) * (1 << 23) + 0.5), i = 2 * k
    // -floor(sin((2 * i + 1) * (2 * (7 - j) + 1) * PI / 32) * (1 << 23) + 0.5), i = 2 * k + 1
    static const int cos_mod[DCT_A_ROWS][DCT_A_COLS] = {
         { 8348215,  8027397,  7398092,  6484482,  5321677,  3954362,  2435084,   822227 },
         { 8027397,  5321677,   822227, -3954362, -7398092, -8348215, -6484482, -2435084 },
         { 7398092,   822227, -6484482, -8027397, -2435084,  5321677,  8348215,  3954362 },
         { 6484482, -3954362, -8027397,   822227,  8348215,  2435084, -7398092, -5321677 },
         { 5321677, -7398092, -2435084,  8348215,  -822227, -8027397,  3954362,  6484482 },
         { 3954362, -8348215,  5321677,  2435084, -8027397,  6484482,   822227, -7398092 },
         { 2435084, -6484482,  8348215, -7398092,  3954362,   822227, -5321677,  8027397 },
         {  822227, -2435084,  3954362, -5321677,  6484482, -7398092,  8027397, -8348215 }
    };

    for (int i = 0; i < DCT_A_ROWS; i++) {
        int64_t res = INT64_C(0);
        for (int j = 0; j < DCT_A_COLS; j++)
            res += (int64_t)cos_mod[i][j] * input[j];
        output[i] = norm23(res);
    }
}

static void dct_b(const int * restrict input, int * restrict output)
{
    // floor(cos((2 * i + 1) * (j + 1) * PI / 16) * (1 << 23) + 0.5)
    static const int cos_mod[DCT_B_ROWS][DCT_B_COLS] = {
        {  8227423,  7750063,  6974873,  5931642,  4660461,  3210181,  1636536 },
        {  6974873,  3210181, -1636536, -5931642, -8227423, -7750063, -4660461 },
        {  4660461, -3210181, -8227423, -5931642,  1636536,  7750063,  6974873 },
        {  1636536, -7750063, -4660461,  5931642,  6974873, -3210181, -8227423 },
        { -1636536, -7750063,  4660461,  5931642, -6974873, -3210181,  8227423 },
        { -4660461, -3210181,  8227423, -5931642, -1636536,  7750063, -6974873 },
        { -6974873,  3210181,  1636536, -5931642,  8227423, -7750063,  4660461 },
        { -8227423,  7750063, -6974873,  5931642, -4660461,  3210181, -1636536 }
    };

    for (int i = 0; i < DCT_B_ROWS; i++) {
        int64_t res = (int64_t)input[0] * (1 << 23);
        for (int j = 0; j < DCT_B_COLS; j++)
            res += (int64_t)cos_mod[i][j] * input[1 + j];
        output[i] = norm23(res);
    }
}

static void mod_a(const int * restrict input, int * restrict output)
{
    //  floor(0.5 / cos((2 * (     i) + 1) * PI / 64) * (1 << 23) + 0.5), i = 0 ..  8
    // -floor(0.5 / sin((2 * (15 - i) + 1) * PI / 64) * (1 << 23) + 0.5), i = 8 .. 16
    static const int cos_mod[MOD_A_SIZE] = {
          4199362,   4240198,   4323885,   4454708,
          4639772,   4890013,   5221943,   5660703,
         -6245623,  -7040975,  -8158494,  -9809974,
        -12450076, -17261920, -28585092, -85479984
    };

    for (int i = 0; i < MOD_A_HALF; i++)
        output[i] = mul23(cos_mod[i], input[i] + input[MOD_A_HALF + i]);

    for (int i = MOD_A_HALF, k = MOD_A_HALF - 1; i < MOD_A_SIZE; i++, k--)
        output[i] = mul23(cos_mod[i], input[k] - input[MOD_A_HALF + k]);
}

static void mod_b(int * restrict input, int * restrict output)
{
    // floor(0.5 / cos((2 * (    i) + 1) * PI / 32) * (1 << 23) + 0.5), i = 0 .. 4
    // floor(0.5 / sin((2 * (7 - i) + 1) * PI / 32) * (1 << 23) + 0.5), i = 4 .. 8
    static const int cos_mod[MOD_B_SIZE] = {
        4214598,  4383036,  4755871,  5425934,
        6611520,  8897610, 14448934, 42791536
    };

    for (int i = 0; i < MOD_B_SIZE; i++) {
        input[MOD_B_SIZE + i] = mul23(cos_mod[i], input[MOD_B_SIZE + i]);
        output[i] = input[i] + input[MOD_B_SIZE + i];
    }

    for (int i = 0, k = MOD_B_SIZE - 1; i < MOD_B_SIZE; i++, k--)
        output[MOD_B_SIZE + i] = input[k] - input[MOD_B_SIZE + k];
}

static void mod_c(const int * restrict input, int * restrict output)
{
    //  floor(0.125 / cos((2 * (     i) + 1) * PI / 128) * (1 << 23) + 0.5), i =  0 .. 16
    // -floor(0.125 / sin((2 * (31 - i) + 1) * PI / 128) * (1 << 23) + 0.5), i = 16 .. 32
    static const int cos_mod[MOD_C_SIZE] = {
         1048892,  1051425,   1056522,   1064244,
         1074689,  1087987,   1104313,   1123884,
         1146975,  1173922,   1205139,   1241133,
         1282529,  1330095,   1384791,   1447815,
        -1520688, -1605358,  -1704360,  -1821051,
        -1959964, -2127368,  -2332183,  -2587535,
        -2913561, -3342802,  -3931480,  -4785806,
        -6133390, -8566050, -14253820, -42727120
    };

    for (int i = 0; i < MOD_C_HALF; i++)
        output[i] = mul23(cos_mod[i], input[i] + input[MOD_C_HALF + i]);

    for (int i = MOD_C_HALF, k = MOD_C_HALF - 1; i < MOD_C_SIZE; i++, k--)
        output[i] = mul23(cos_mod[i], input[k] - input[MOD_C_HALF + k]);
}

static void clp_v(int *input, int len)
{
    for (int i = 0; i < len; i++)
        input[i] = clip23(input[i]);
}

void idct_perform32_fixed(int * restrict input, int * restrict output)
{
    int mag = 0;
    for (int i = 0; i < IDCT_SIZE; i++)
        mag += abs(input[i]);

    int shift = mag > 0x400000 ? 2 : 0;
    int round = shift > 0 ? 1 << (shift - 1) : 0;

    for (int i = 0; i < IDCT_SIZE; i++)
        input[i] = (input[i] + round) >> shift;

    sum_a(input, output + 0 * IDCT_SIZE_2, IDCT_SIZE_2);
    sum_b(input, output + 1 * IDCT_SIZE_2, IDCT_SIZE_2);
    clp_v(output, IDCT_SIZE);

    sum_a(output + 0 * IDCT_SIZE_2, input + 0 * IDCT_SIZE_4, IDCT_SIZE_4);
    sum_b(output + 0 * IDCT_SIZE_2, input + 1 * IDCT_SIZE_4, IDCT_SIZE_4);
    sum_c(output + 1 * IDCT_SIZE_2, input + 2 * IDCT_SIZE_4, IDCT_SIZE_4);
    sum_d(output + 1 * IDCT_SIZE_2, input + 3 * IDCT_SIZE_4, IDCT_SIZE_4);
    clp_v(input, IDCT_SIZE);

    dct_a(input + 0 * IDCT_SIZE_4, output + 0 * IDCT_SIZE_4);
    dct_b(input + 1 * IDCT_SIZE_4, output + 1 * IDCT_SIZE_4);
    dct_b(input + 2 * IDCT_SIZE_4, output + 2 * IDCT_SIZE_4);
    dct_b(input + 3 * IDCT_SIZE_4, output + 3 * IDCT_SIZE_4);
    clp_v(output, IDCT_SIZE);

    mod_a(output + 0 * IDCT_SIZE_2, input + 0 * IDCT_SIZE_2);
    mod_b(output + 1 * IDCT_SIZE_2, input + 1 * IDCT_SIZE_2);
    clp_v(input, IDCT_SIZE);

    mod_c(input, output);

    for (int i = 0; i < IDCT_SIZE; i++)
        output[i] = clip23(output[i] * (1 << shift));
}

static void mod64_a(const int * restrict input, int * restrict output)
{
    //  floor(0.5 / cos((2 * (     i) + 1) * PI / 128) * (1 << 23) + 0.5), i =  0 .. 16
    // -floor(0.5 / sin((2 * (31 - i) + 1) * PI / 128) * (1 << 23) + 0.5), i = 16 .. 32
    static const int cos_mod[MOD64_A_SIZE] = {
          4195568,   4205700,   4226086,    4256977,
          4298755,   4351949,   4417251,    4495537,
          4587901,   4695690,   4820557,    4964534,
          5130115,   5320382,   5539164,    5791261,
         -6082752,  -6421430,  -6817439,   -7284203,
         -7839855,  -8509474,  -9328732,  -10350140,
        -11654242, -13371208, -15725922,  -19143224,
        -24533560, -34264200, -57015280, -170908480
    };

    for (int i = 0; i < MOD64_A_HALF; i++)
        output[i] = mul23(cos_mod[i], input[i] + input[MOD64_A_HALF + i]);

    for (int i = MOD64_A_HALF, k = MOD64_A_HALF - 1; i < MOD64_A_SIZE; i++, k--)
        output[i] = mul23(cos_mod[i], input[k] - input[MOD64_A_HALF + k]);
}

static void mod64_b(int * restrict input, int * restrict output)
{
    // floor(0.5 / cos((2 * (     i) + 1) * PI / 64) * (1 << 23) + 0.5), i = 0 ..  8
    // floor(0.5 / sin((2 * (15 - i) + 1) * PI / 64) * (1 << 23) + 0.5), i = 8 .. 16
    static const int cos_mod[MOD64_B_SIZE] = {
         4199362,  4240198,  4323885,  4454708,
         4639772,  4890013,  5221943,  5660703,
         6245623,  7040975,  8158494,  9809974,
        12450076, 17261920, 28585092, 85479984
    };

    for (int i = 0; i < MOD64_B_SIZE; i++) {
        input[MOD64_B_SIZE + i] = mul23(cos_mod[i], input[MOD64_B_SIZE + i]);
        output[i] = input[i] + input[MOD64_B_SIZE + i];
    }

    for (int i = 0, k = MOD64_B_SIZE - 1; i < MOD64_B_SIZE; i++, k--)
        output[MOD64_B_SIZE + i] = input[k] - input[MOD64_B_SIZE + k];
}

static void mod64_c(const int * restrict input, int * restrict output)
{
    //  floor(0.125 / SQRT2 / cos((2 * (     i) + 1) * PI / 256) * (1 << 23) + 0.5), i =  0 .. 32
    // -floor(0.125 / SQRT2 / sin((2 * (63 - i) + 1) * PI / 256) * (1 << 23) + 0.5), i = 32 .. 64
    static const int cos_mod[MOD64_C_SIZE] = {
          741511,    741958,    742853,    744199,
          746001,    748262,    750992,    754197,
          757888,    762077,    766777,    772003,
          777772,    784105,    791021,    798546,
          806707,    815532,    825054,    835311,
          846342,    858193,    870912,    884554,
          899181,    914860,    931667,    949686,
          969011,    989747,   1012012,   1035941,
        -1061684,  -1089412,  -1119320,  -1151629,
        -1186595,  -1224511,  -1265719,  -1310613,
        -1359657,  -1413400,  -1472490,  -1537703,
        -1609974,  -1690442,  -1780506,  -1881904,
        -1996824,  -2128058,  -2279225,  -2455101,
        -2662128,  -2909200,  -3208956,  -3579983,
        -4050785,  -4667404,  -5509372,  -6726913,
        -8641940, -12091426, -20144284, -60420720
    };

    for (int i = 0; i < MOD64_C_HALF; i++)
        output[i] = mul23(cos_mod[i], input[i] + input[MOD64_C_HALF + i]);

    for (int i = MOD64_C_HALF, k = MOD64_C_HALF - 1; i < MOD64_C_SIZE; i++, k--)
        output[i] = mul23(cos_mod[i], input[k] - input[MOD64_C_HALF + k]);
}

void idct_perform64_fixed(int * restrict input, int * restrict output)
{
    int mag = 0;
    for (int i = 0; i < IDCT64_SIZE; i++)
        mag += abs(input[i]);

    int shift = mag > 0x400000 ? 2 : 0;
    int round = shift > 0 ? 1 << (shift - 1) : 0;

    for (int i = 0; i < IDCT64_SIZE; i++)
        input[i] = (input[i] + round) >> shift;

    sum_a(input, output + 0 * IDCT64_SIZE_2, IDCT64_SIZE_2);
    sum_b(input, output + 1 * IDCT64_SIZE_2, IDCT64_SIZE_2);
    clp_v(output, IDCT64_SIZE);

    sum_a(output + 0 * IDCT64_SIZE_2, input + 0 * IDCT64_SIZE_4, IDCT64_SIZE_4);
    sum_b(output + 0 * IDCT64_SIZE_2, input + 1 * IDCT64_SIZE_4, IDCT64_SIZE_4);
    sum_c(output + 1 * IDCT64_SIZE_2, input + 2 * IDCT64_SIZE_4, IDCT64_SIZE_4);
    sum_d(output + 1 * IDCT64_SIZE_2, input + 3 * IDCT64_SIZE_4, IDCT64_SIZE_4);
    clp_v(input, IDCT64_SIZE);

    sum_a(input + 0 * IDCT64_SIZE_4, output + 0 * IDCT64_SIZE_8, IDCT64_SIZE_8);
    sum_b(input + 0 * IDCT64_SIZE_4, output + 1 * IDCT64_SIZE_8, IDCT64_SIZE_8);
    sum_c(input + 1 * IDCT64_SIZE_4, output + 2 * IDCT64_SIZE_8, IDCT64_SIZE_8);
    sum_d(input + 1 * IDCT64_SIZE_4, output + 3 * IDCT64_SIZE_8, IDCT64_SIZE_8);
    sum_c(input + 2 * IDCT64_SIZE_4, output + 4 * IDCT64_SIZE_8, IDCT64_SIZE_8);
    sum_d(input + 2 * IDCT64_SIZE_4, output + 5 * IDCT64_SIZE_8, IDCT64_SIZE_8);
    sum_c(input + 3 * IDCT64_SIZE_4, output + 6 * IDCT64_SIZE_8, IDCT64_SIZE_8);
    sum_d(input + 3 * IDCT64_SIZE_4, output + 7 * IDCT64_SIZE_8, IDCT64_SIZE_8);
    clp_v(output, IDCT64_SIZE);

    dct_a(output + 0 * IDCT64_SIZE_8, input + 0 * IDCT64_SIZE_8);
    dct_b(output + 1 * IDCT64_SIZE_8, input + 1 * IDCT64_SIZE_8);
    dct_b(output + 2 * IDCT64_SIZE_8, input + 2 * IDCT64_SIZE_8);
    dct_b(output + 3 * IDCT64_SIZE_8, input + 3 * IDCT64_SIZE_8);
    dct_b(output + 4 * IDCT64_SIZE_8, input + 4 * IDCT64_SIZE_8);
    dct_b(output + 5 * IDCT64_SIZE_8, input + 5 * IDCT64_SIZE_8);
    dct_b(output + 6 * IDCT64_SIZE_8, input + 6 * IDCT64_SIZE_8);
    dct_b(output + 7 * IDCT64_SIZE_8, input + 7 * IDCT64_SIZE_8);
    clp_v(input, IDCT64_SIZE);

    mod_a(input + 0 * IDCT64_SIZE_4, output + 0 * IDCT64_SIZE_4);
    mod_b(input + 1 * IDCT64_SIZE_4, output + 1 * IDCT64_SIZE_4);
    mod_b(input + 2 * IDCT64_SIZE_4, output + 2 * IDCT64_SIZE_4);
    mod_b(input + 3 * IDCT64_SIZE_4, output + 3 * IDCT64_SIZE_4);
    clp_v(output, IDCT64_SIZE);

    mod64_a(output + 0 * IDCT64_SIZE_2, input + 0 * IDCT64_SIZE_2);
    mod64_b(output + 1 * IDCT64_SIZE_2, input + 1 * IDCT64_SIZE_2);
    clp_v(input, IDCT64_SIZE);

    mod64_c(input, output);

    for (int i = 0; i < IDCT64_SIZE; i++)
        output[i] = clip23(output[i] * (1 << shift));
}

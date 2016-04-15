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

#define IDCT_BITS   7
#define IDCT_SIZE   (1 << IDCT_BITS)

struct idct_context {
    double cs[IDCT_SIZE / 2];
    double ac[IDCT_SIZE / 2];
    double as[IDCT_SIZE / 2];
    uint8_t permute[IDCT_SIZE / 2];
    int nbits;
};

struct idct_context *idct_init(void *parent, int nbits, double scale) __attribute__((cold));
void idct_fast(const struct idct_context *s, const double *input, double *output);
void imdct_fast(const struct idct_context *s, const float *input, float *output);

void idct_fixed32(int * restrict input, int * restrict output);
void idct_fixed64(int * restrict input, int * restrict output);

#endif

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

struct idct_context *idct_init(void *parent, int nbits, double scale)
{
    int i, j, k, p, base;

    if (nbits < 2 || nbits > IDCT_BITS)
        return NULL;

    struct idct_context *s = ta_new(parent, struct idct_context);
    if (!s)
        return NULL;

    int m = nbits;
    int n = 1 << m;
    int n2 = n >> 1;
    int n4 = n >> 2;

    for (i = base = 0; i < m - 1; i++, base += p) {
        p = 1 << i;
        for (j = 0; j < p; j++)
            s->cs[base + j] = cos(M_PI * (4 * j + 1) * (n4 >> i) / n);
    }

    for (i = 0; i < n2; i++) {
        double a = M_PI * (1.0 / (n << 2) + 1.0 * i / (n << 1));
        s->ac[i] = scale * cos(a);
        s->as[i] = scale * sin(a);
    }

    for (i = 1; i < n2 - 1; i++) {
        for (j = k = 0; j < m - 1; j++)
            k = (k << 1) | ((i >> j) & 1);
        s->permute[i] = k;
    }

    s->nbits = nbits;
    return s;
}

static void proc(const struct idct_context *s, double *x, int flag)
{
    double y[IDCT_SIZE / 2], tmp;
    int i, j, k, p, q, base;

    int m = s->nbits - 1;
    int n = 1 << m;
    int n2 = n >> 1;

    for (i = 0; i < n; i++)
        y[i] = x[i];

    for (i = m - 2; i >= 0; i--) {
        int f0 = n >> i;
        int f1 = f0 >> 1;
        int f2 = f1 >> 1;
        int f3 = ((1 << i) - 1) << 1;
        for (j = f2; j > 0; j--) {
            for (k = f3; k >= 0; k--) {
                p = f0 - j + k * f1;
                q = f1 - j + k * f1;
                y[q] -= y[p];
                y[p] += y[p];
            }
        }
    }

    for (i = 1; i < n - 1; i++) {
        k = s->permute[i];
        if (i < k) {
            tmp = y[i];
            y[i] = y[k];
            y[k] = tmp;
        }
    }

    for (i = base = 0; i < m; i++, base += p) {
        p = 1 << i;
        q = 2 << i;
        for (j = 0; j < p; j++) {
            for (k = j; k < n; k += q) {
                tmp = y[k + p] * s->cs[base + j];
                y[k + p] = y[k] - tmp;
                y[k] += tmp;
            }
        }
    }

    for (i = 0; i < n2; i++) {
        x[2 * i] = y[i];
        if (flag)
            x[2 * i + 1] = -y[n - 1 - i];
        else
            x[2 * i + 1] =  y[n - 1 - i];
    }
}

void idct_fast(const struct idct_context *s, const double *input, double *output)
{
    double a[IDCT_SIZE / 2];
    double b[IDCT_SIZE / 2];
    int i;

    int m = s->nbits;
    int n = 1 << m;
    int n2 = n >> 1;

    a[0] = input[0];
    b[0] = input[n - 1];
    for (i = 1; i < n2; i++) {
        a[     i] = input[2 * i - 1] + input[2 * i];
        b[n2 - i] = input[2 * i - 1] - input[2 * i];
    }

    proc(s, a, 0);
    proc(s, b, 1);

    for (i = 0; i < n2; i++) {
        output[    i    ] = a[i] * s->ac[i] + b[i] * s->as[i];
        output[n - i - 1] = a[i] * s->as[i] - b[i] * s->ac[i];
    }
}

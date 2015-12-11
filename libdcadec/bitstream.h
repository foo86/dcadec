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

#ifndef BITSTREAM_H
#define BITSTREAM_H

#include "huffman.h"

#define BITS_INVALID_VLC_UN  32768
#define BITS_INVALID_VLC_SI -16384

struct bitstream {
    uint32_t    *data;
    int         total;
    int         index;
};

static inline void bits_init(struct bitstream *bits, uint8_t *data, int size)
{
    assert(data && !((uintptr_t)data & 3));
    assert(size > 0 && size < INT_MAX / 8);
    bits->data = (uint32_t *)data;
    bits->total = size * 8;
    bits->index = 0;
}

bool bits_get1(struct bitstream *bits);
int bits_get(struct bitstream *bits, int n);
int bits_get_signed(struct bitstream *bits, int n);
int bits_get_signed_linear(struct bitstream *bits, int n);
int bits_get_signed_rice(struct bitstream *bits, int k);
int bits_get_unsigned_vlc(struct bitstream *bits, const struct huffman *h);
int bits_get_signed_vlc(struct bitstream *bits, const struct huffman *h);

static inline void bits_skip(struct bitstream *bits, int n)
{
    assert(n >= 0);
    bits->index += n;
}

static inline void bits_skip1(struct bitstream *bits)
{
    bits->index++;
}

static inline int bits_seek(struct bitstream *bits, int n)
{
    if (n < bits->index || n > bits->total)
        return -DCADEC_EBADREAD;
    bits->index = n;
    return 0;
}

static inline int bits_align1(struct bitstream *bits)
{
    bits->index = DCA_ALIGN(bits->index, 8);
    return bits->index;
}

static inline int bits_align4(struct bitstream *bits)
{
    bits->index = DCA_ALIGN(bits->index, 32);
    return bits->index;
}

int bits_check_crc(struct bitstream *bits, int p1, int p2);
void bits_get_array(struct bitstream *bits, int *array, int size, int n);
void bits_get_signed_array(struct bitstream *bits, int *array, int size, int n);
void bits_get_signed_linear_array(struct bitstream *bits, int *array, int size, int n);
void bits_get_signed_rice_array(struct bitstream *bits, int *array, int size, int k);
int bits_get_signed_vlc_array(struct bitstream *bits, int *array, int size, const struct huffman *h);

#endif

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
#include "bitstream.h"

static inline uint32_t bits_peek(struct bitstream *bits)
{
    if (bits->index >= bits->total)
        return 0;

    int pos = bits->index >> 5;
    int shift = bits->index & 31;

    uint32_t v = DCA_32BE(bits->data[pos]);
    if (shift) {
        v <<= shift;
        v |= DCA_32BE(bits->data[pos + 1]) >> (32 - shift);
    }

    return v;
}

bool bits_get1(struct bitstream *bits)
{
    if (bits->index >= bits->total)
        return false;

    uint32_t v = DCA_32BE(bits->data[bits->index >> 5]);
    v <<= bits->index & 31;
    v >>= 32 - 1;

    bits->index++;
    return v;
}

int bits_get(struct bitstream *bits, int n)
{
    uint32_t v = bits_peek(bits);
    v >>= 32 - n;

    bits->index += n;
    return v;
}

int bits_get_signed(struct bitstream *bits, int n)
{
    int32_t v = bits_peek(bits);
    v >>= 32 - n;

    bits->index += n;
    return v;
}

int bits_get_signed_linear(struct bitstream *bits, int n)
{
    if (n == 0)
        return 0;

    unsigned int v = bits_get(bits, n);
    return (v >> 1) ^ -(v & 1);
}

static int bits_get_unsigned_rice(struct bitstream *bits, int k)
{
    unsigned int unary = 0;

    while (bits->index < bits->total) {
        uint32_t v = bits_peek(bits);
        if (v) {
            int z = dca_clz(v);
            bits->index += z + 1;
            unary += z;
            break;
        }
        bits->index += 32;
        unary += 32;
    }

    return k > 0 ? (unary << k) | bits_get(bits, k) : unary;
}

int bits_get_signed_rice(struct bitstream *bits, int k)
{
    unsigned int v = bits_get_unsigned_rice(bits, k);
    return (v >> 1) ^ -(v & 1);
}

int bits_get_unsigned_vlc(struct bitstream *bits, const struct huffman *h)
{
    uint32_t v = bits_peek(bits);

    for (int i = 0; i < h->size; i++) {
        if (v >> (32 - h->len[i]) == h->code[i]) {
            bits->index += h->len[i];
            return i;
        }
    }

    return BITS_INVALID_VLC_UN;
}

int bits_get_signed_vlc(struct bitstream *bits, const struct huffman *h)
{
    unsigned int v = bits_get_unsigned_vlc(bits, h);
    return ((v >> 1) ^ ((v & 1) - 1)) + 1;
}

static uint16_t crc16(const uint8_t *data, int size)
{
    static const uint16_t crctab[16] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
    };

    uint16_t res = 0xffff;
    for (int i = 0; i < size; i++) {
        res = (res << 4) ^ crctab[(data[i] >> 4) ^ (res >> 12)];
        res = (res << 4) ^ crctab[(data[i] & 15) ^ (res >> 12)];
    }

    return res;
}

int bits_check_crc(struct bitstream *bits, int p1, int p2)
{
    if (((p1 | p2) & 7) || p1 < 0 || p2 > bits->total || p2 - p1 < 16)
        return -DCADEC_EBADREAD;
    if (crc16((uint8_t *)bits->data + p1 / 8, (p2 - p1) / 8))
        return -DCADEC_EBADCRC;
    return 0;
}

void bits_get_array(struct bitstream *bits, int *array, int size, int n)
{
    for (int i = 0; i < size; i++)
        array[i] = bits_get(bits, n);
}

void bits_get_signed_array(struct bitstream *bits, int *array, int size, int n)
{
    for (int i = 0; i < size; i++)
        array[i] = bits_get_signed(bits, n);
}

void bits_get_signed_linear_array(struct bitstream *bits, int *array, int size, int n)
{
    if (n == 0)
        memset(array, 0, sizeof(*array) * size);
    else for (int i = 0; i < size; i++)
        array[i] = bits_get_signed_linear(bits, n);
}

void bits_get_signed_rice_array(struct bitstream *bits, int *array, int size, int k)
{
    for (int i = 0; i < size; i++)
        array[i] = bits_get_signed_rice(bits, k);
}

int bits_get_signed_vlc_array(struct bitstream *bits, int *array, int size, const struct huffman *h)
{
    for (int i = 0; i < size; i++)
        if ((array[i] = bits_get_signed_vlc(bits, h)) == BITS_INVALID_VLC_SI)
            return -DCADEC_EBADDATA;
    return 0;
}

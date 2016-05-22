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

struct bytestream {
    uint8_t *data;
    int total;
    int index;
};

static void bits2_init(struct bitstream2 *bits, uint8_t *data, size_t size)
{
    bits->data = data;
    bits->index = 0;
    bits->accum = 0;
    bits->avail = 0;
    bits->count = size << 3;
}

static int bits2_peek(struct bitstream2 *bits, int n)
{
    assert(n > 0 && n <= 32);

    while (bits->avail < n) {
        bits->accum |= bits->data[bits->index++] << bits->avail;
        bits->avail += 8;
    }

    return bits->accum & (0xffffffff >> (32 - n));
}

static void bits2_skip(struct bitstream2 *bits, int n)
{
    assert(n > 0 && n <= bits->avail);

    bits->accum >>= n;
    bits->avail -= n;
    bits->count -= n;
}

static int bits2_get(struct bitstream2 *bits, int n)
{
    int v = bits2_peek(bits, n);
    bits2_skip(bits, n);
    return v;
}

static int bits2_getz(struct bitstream2 *bits, int n)
{
    if (n == 0)
        return 0;
    return bits2_get(bits, n);
}

static bool bits2_get1(struct bitstream2 *bits)
{
    return bits2_get(bits, 1);
}

#if 0
static void bits2_skip_long(struct bitstream2 *bits, int n)
{
    if (n > bits->count)
        n = bits->count;

    bits->count -= n;

    if (n <= bits->avail) {
        bits->accum >>= n;
        bits->avail -= n;
        return;
    }

    n -= bits->avail;
    bits->index += n >> 3;
    bits->accum = 0;
    bits->avail = 0;

    n &= 7;
    if (n) {
        bits->accum = bits->data[bits->index++] >> n;
        bits->avail = 8 - n;
    }
}
#endif

static int bits2_get_vlc(struct bitstream2 *bits, const uint8_t *table, size_t size)
{
    size_t index = 0;

    assert(size && !(size & 1));
    while (table[index] != 0xff) {
        if (bits->count < 1)
            return -1;
        assert(table[index] == 1);
        index += 2 * table[index + bits2_get1(bits)];
        if (index >= size)
            return -1;
    }

    if (table[index + 1])
        return table[index + 1] - 1;

    if (bits->count < 3)
        return -1;

    int n = bits2_get(bits, 3) + 1;
    if (bits->count < n)
        return -1;

    return bits2_get(bits, n);
}

static void bytes_init(struct bytestream *bytes, uint8_t *data, size_t size)
{
    bytes->data = data;
    bytes->total = size;
    bytes->index = 0;
}

static int bytes_get(struct bytestream *bytes)
{
    uint8_t v = 0;

    if (bytes->index < bytes->total)
        v = bytes->data[bytes->index];

    bytes->index++;
    return v;
}

static int bytes_get16le(struct bytestream *bytes)
{
    uint16_t v = 0;

    if (bytes->index + 1 < bytes->total)
        v = DCA_MEM16LE(&bytes->data[bytes->index]);

    bytes->index += 2;
    return v;
}

static int bytes_get16be(struct bytestream *bytes)
{
    uint16_t v = 0;

    if (bytes->index + 1 < bytes->total)
        v = DCA_MEM16BE(&bytes->data[bytes->index]);

    bytes->index += 2;
    return v;
}

static int bytes_get32be(struct bytestream *bytes)
{
    uint32_t v = 0;

    if (bytes->index + 3 < bytes->total)
        v = DCA_MEM32BE(&bytes->data[bytes->index]);

    bytes->index += 4;
    return v;
}

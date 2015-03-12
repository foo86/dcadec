/*
 * This file is part of dcadec.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "common.h"
#include "dca_waveout.h"

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#else
#include <unistd.h>
#endif

struct dcadec_waveout {
    FILE        *fp;
    uint64_t    size;
    uint8_t     *buffer;

    int         channel_mask;
    int         nchannels;
    int         sample_rate;
    int         bits_per_sample;
    int         bytes_per_sample;
    int         container_shift;
    int         block_align;
};

static void write_raw(struct dcadec_waveout *wave, const char *s)
{
    fwrite(s, strlen(s), 1, wave->fp);
}

static void write_short(struct dcadec_waveout *wave, int v)
{
    uint8_t buf[] = {
        (v >> 0) & 0xff,
        (v >> 8) & 0xff
    };
    fwrite(buf, sizeof(buf), 1, wave->fp);
}

static void write_int(struct dcadec_waveout *wave, int v)
{
    uint8_t buf[] = {
        (v >>  0) & 0xff,
        (v >>  8) & 0xff,
        (v >> 16) & 0xff,
        (v >> 24) & 0xff
    };
    fwrite(buf, sizeof(buf), 1, wave->fp);
}

static int write_header(struct dcadec_waveout *wave, int channel_mask,
                        int sample_rate, int bits_per_sample)
{
    if (!channel_mask)
        return -DCADEC_EINVAL;
    if (sample_rate < 8000 || sample_rate > 384000)
        return -DCADEC_EINVAL;
    if (bits_per_sample < 8 || bits_per_sample > 32)
        return -DCADEC_EINVAL;

    wave->channel_mask = channel_mask;
    wave->nchannels = dca_popcount(channel_mask);
    wave->sample_rate = sample_rate;
    wave->bits_per_sample = bits_per_sample;
    wave->bytes_per_sample = (bits_per_sample + 7) >> 3;
    wave->container_shift = (wave->bytes_per_sample << 3) - bits_per_sample;
    wave->block_align = wave->nchannels * wave->bytes_per_sample;

    write_raw(wave, "RIFF");
    write_int(wave, 0);
    write_raw(wave, "WAVE");

    write_raw(wave, "fmt ");
    write_int(wave, 40);

    // wFormatTag
    write_short(wave, 0xfffe);

    // nChannels
    write_short(wave, wave->nchannels);

    // nSamplesPerSec
    write_int(wave, wave->sample_rate);

    // nAvgBytesPerSec
    write_int(wave, wave->sample_rate * wave->block_align);

    // nBlockAlign
    write_short(wave, wave->block_align);

    // wBitsPerSample
    write_short(wave, wave->bytes_per_sample << 3);

    // cbSize
    write_short(wave, 22);

    // wValidBitsPerSample
    write_short(wave, wave->bits_per_sample);

    // dwChannelMask
    write_int(wave, wave->channel_mask);

    // SubFormat
    write_int(wave, 1);
    write_int(wave, 0x00100000);
    write_int(wave, 0xaa000080);
    write_int(wave, 0x719b3800);

    write_raw(wave, "data");
    write_int(wave, 0);

    if (ferror(wave->fp))
        return -DCADEC_EIO;
    return 0;
}

int dcadec_waveout_write(struct dcadec_waveout *wave, int **samples,
                         int nsamples, int channel_mask, int sample_rate,
                         int bits_per_sample)
{
    int ret;

    if (nsamples == 0)
        return 0;
    if (nsamples < 0)
        return -DCADEC_EINVAL;
    if (!wave)
        return -DCADEC_EINVAL;
    if (!samples)
        return -DCADEC_EINVAL;

    if (!wave->size) {
        if ((ret = write_header(wave, channel_mask,
                                sample_rate, bits_per_sample)) < 0)
            return ret;
        wave->size = 60;
    } else {
        if (channel_mask != wave->channel_mask)
            return -DCADEC_EINVAL;
        if (sample_rate != wave->sample_rate)
            return -DCADEC_EINVAL;
        if (bits_per_sample != wave->bits_per_sample)
            return -DCADEC_EINVAL;
    }

    if ((ret = dca_realloc(wave, &wave->buffer, nsamples, wave->block_align)) < 0)
        return ret;

    int limit = 1 << (wave->bits_per_sample - 1);
    int mask = ~((1 << wave->bits_per_sample) - 1);

    uint8_t *dst = wave->buffer;
    for (int i = 0; i < nsamples; i++) {
        for (int j = 0; j < wave->nchannels; j++) {
            int sample = samples[j][i];

            if ((sample + limit) & mask)
                return -DCADEC_EOVERFLOW;

            sample <<= wave->container_shift;

            switch (wave->bytes_per_sample) {
            case 4:
                dst[0] = (sample >>  0) & 0xff;
                dst[1] = (sample >>  8) & 0xff;
                dst[2] = (sample >> 16) & 0xff;
                dst[3] = (sample >> 24) & 0xff;
                break;
            case 3:
                dst[0] = (sample >>  0) & 0xff;
                dst[1] = (sample >>  8) & 0xff;
                dst[2] = (sample >> 16) & 0xff;
                break;
            case 2:
                dst[0] = (sample >>  0) & 0xff;
                dst[1] = (sample >>  8) & 0xff;
                break;
            case 1:
                dst[0] = (sample >>  0) & 0xff;
                break;
            default:
                return -DCADEC_EINVAL;
            }

            dst += wave->bytes_per_sample;
        }
    }

    if (fwrite(wave->buffer, wave->block_align, nsamples, wave->fp) != (size_t)nsamples)
        return -DCADEC_EIO;
    wave->size += nsamples * wave->block_align;
    return 0;
}

struct dcadec_waveout *dcadec_waveout_open(const char *name)
{
    struct dcadec_waveout *wave = ta_znew(NULL, struct dcadec_waveout);
    if (!wave)
        return NULL;

    if (name) {
        if (!(wave->fp = fopen(name, "wb")))
            goto fail;
    } else {
        int fd;
#ifdef _WIN32
        if ((fd = _dup(STDOUT_FILENO)) < 0)
            goto fail;
        if (_setmode(fd, _O_BINARY) < 0) {
            _close(fd);
            goto fail;
        }
        if (!(wave->fp = fdopen(fd, "wb"))) {
            _close(fd);
            goto fail;
        }
#else
        if ((fd = dup(STDOUT_FILENO)) < 0)
            goto fail;
        if (!(wave->fp = fdopen(fd, "wb"))) {
            close(fd);
            goto fail;
        }
#endif
    }

    return wave;

fail:
    ta_free(wave);
    return NULL;
}

void dcadec_waveout_close(struct dcadec_waveout *wave)
{
    if (!wave)
        return;

    if (wave->size > 60) {
        if (fseeko(wave->fp, 4, SEEK_SET) == 0) {
            if (wave->size <= UINT32_MAX)
                write_int(wave, wave->size);
            else
                write_int(wave, 0);
        }
        if (fseeko(wave->fp, 64, SEEK_SET) == 0) {
            if (wave->size <= UINT32_MAX)
                write_int(wave, wave->size - 60);
            else
                write_int(wave, 0);
        }
    }

    fclose(wave->fp);
    ta_free(wave);
}

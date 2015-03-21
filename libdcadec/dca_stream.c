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
#include "dca_stream.h"

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#else
#include <unistd.h>
#endif

#define BUFFER_ALIGN    4096
#define HEADER_SIZE     16
#define SYNC_SIZE       4

struct dcadec_stream {
    FILE    *fp;

    off_t   stream_size;
    off_t   stream_start;
    off_t   stream_end;

    uint8_t     *buffer;
    size_t      packet_size;
    uint32_t    backup_sync;
};

// Check for DTS-HD container format. Such files have an extra `blackout'
// frame at the end that we don't wont to parse. Called only if the stream
// is seekable.
static int parse_hd_hdr(struct dcadec_stream *stream)
{
    uint64_t header[2];

    if (fread(header, sizeof(header), 1, stream->fp) != 1)
        return fseeko(stream->fp, 0, SEEK_SET);

    if (memcmp(header, "DTSHDHDR", 8))
        return fseeko(stream->fp, 0, SEEK_SET);

    while (true) {
        off_t size = DCA_64BE(header[1]);
        if (size < 0)
            return -1;
        if (!memcmp(header, "STRMDATA", 8)) {
            off_t pos = ftello(stream->fp);
            if (pos < 0)
                return -1;
            stream->stream_size = size;
            stream->stream_start = pos;
            stream->stream_end = pos + size;
            return 1;
        }
        if (fseeko(stream->fp, size, SEEK_CUR) < 0)
            return -1;
        if (fread(header, sizeof(header), 1, stream->fp) != 1)
            return -1;
    }

    return 0;
}

DCADEC_API struct dcadec_stream *dcadec_stream_open(const char *name)
{
    struct dcadec_stream *stream = ta_znew(NULL, struct dcadec_stream);
    if (!stream)
        return NULL;

    if (name) {
        if (!(stream->fp = fopen(name, "rb")))
            goto fail1;
    } else {
        int fd;
#ifdef _WIN32
        if ((fd = _dup(STDIN_FILENO)) < 0)
            goto fail1;
        if (_setmode(fd, _O_BINARY) < 0) {
            _close(fd);
            goto fail1;
        }
        if (!(stream->fp = fdopen(fd, "rb"))) {
            _close(fd);
            goto fail1;
        }
#else
        if ((fd = dup(STDIN_FILENO)) < 0)
            goto fail1;
        if (!(stream->fp = fdopen(fd, "rb"))) {
            close(fd);
            goto fail1;
        }
#endif
    }

    if (fseeko(stream->fp, 0, SEEK_END) == 0) {
        off_t pos = ftello(stream->fp);
        if (pos > 0)
            stream->stream_size = pos;
        if (fseeko(stream->fp, 0, SEEK_SET) < 0)
            goto fail2;
        if (pos > 0 && parse_hd_hdr(stream) < 0)
            goto fail2;
    }

    if (!(stream->buffer = ta_alloc_size(stream, BUFFER_ALIGN)))
        goto fail2;

    return stream;

fail2:
    fclose(stream->fp);
fail1:
    ta_free(stream);
    return NULL;
}

DCADEC_API void dcadec_stream_close(struct dcadec_stream *stream)
{
    if (stream) {
        fclose(stream->fp);
        ta_free(stream);
    }
}

static int realloc_buffer(struct dcadec_stream *stream, size_t size)
{
    size = (size + DCADEC_BUFFER_PADDING + BUFFER_ALIGN - 1) & ~(BUFFER_ALIGN - 1);
    if (ta_get_size(stream->buffer) < size) {
        void *buf = ta_realloc_size(stream, stream->buffer, size);
        if (!buf)
            return -1;
        stream->buffer = buf;
        return 1;
    }
    return 0;
}

static void swap16(uint32_t *data, size_t size)
{
    while (size--) {
        uint32_t v = *data;
        *data++ = ((v & 0x00ff00ff) << 8) | ((v & 0xff00ff00) >> 8);
    }
}

#if (defined _WIN32)
#define DCA_FGETC   _fgetc_nolock
#elif (defined _BSD_SOURCE)
#define DCA_FGETC   fgetc_unlocked
#else
#define DCA_FGETC   fgetc
#endif

static int read_frame(struct dcadec_stream *stream, uint32_t *sync_p)
{
    // Stop at position indicated by STRMDATA if known
    if (stream->stream_end > 0 && ftello(stream->fp) >= stream->stream_end)
        return 0;

    // Start with a backed up sync word. If there is none, advance one byte at
    // a time until proper sync word is read from the input byte stream.
    uint32_t sync = stream->backup_sync;
    while (sync != SYNC_WORD_CORE
        && sync != SYNC_WORD_EXSS
        && sync != SYNC_WORD_CORE_LE
        && sync != SYNC_WORD_EXSS_LE) {
        int c = DCA_FGETC(stream->fp);
        if (c == EOF)
            return 0;
        sync = (sync << 8) | c;
    }

    // Tried to read the second (EXSS) frame and it was core again. Back up
    // the sync word just read and return.
    if ((sync != SYNC_WORD_EXSS && sync != SYNC_WORD_EXSS_LE) && !sync_p) {
        stream->backup_sync = sync;
        return -2;
    }

    // Clear backed up sync word
    stream->backup_sync = 0;

    // Reallocate frame buffer
    if (realloc_buffer(stream, stream->packet_size + HEADER_SIZE) < 0)
        return -1;

    // Read the frame header
    uint8_t *buf = stream->buffer + stream->packet_size;
    if (fread(buf + SYNC_SIZE, HEADER_SIZE - SYNC_SIZE, 1, stream->fp) != 1)
        return 0;

    bool swap = false;
    switch (sync) {
    case SYNC_WORD_CORE_LE:
        sync = SYNC_WORD_CORE;
        swap = true;
        break;
    case SYNC_WORD_EXSS_LE:
        sync = SYNC_WORD_EXSS;
        swap = true;
        break;
    }

    if (swap)
        swap16((uint32_t *)buf, HEADER_SIZE / 4);

    struct bitstream bits;

    bits_init(&bits, buf + SYNC_SIZE, HEADER_SIZE - SYNC_SIZE);

    size_t frame_size;

    if (sync == SYNC_WORD_CORE) {
        bool normal_frame = bits_get1(&bits);
        int deficit_samples = bits_get(&bits, 5) + 1;
        if (normal_frame && deficit_samples != 32)
            return -2;
        bits_skip1(&bits);
        int npcmblocks = bits_get(&bits, 7) + 1;
        if (npcmblocks < 6)
            return -2;
        frame_size = bits_get(&bits, 14) + 1;
        if (frame_size < 96)
            return -2;
    } else {
        bits_skip(&bits, 10);
        bool wide_hdr = bits_get1(&bits);
        bits_skip(&bits, 8 + 4 * wide_hdr);
        frame_size = bits_get(&bits, 16 + 4 * wide_hdr) + 1;
        if (frame_size < HEADER_SIZE)
             return -2;
    }

    // Align frame size to 4-byte boundary
    size_t aligned_size = (frame_size + 3) & ~3;

    // Reallocate frame buffer
    if (realloc_buffer(stream, stream->packet_size + aligned_size) < 0)
        return -1;

    // Read the rest of the frame
    buf = stream->buffer + stream->packet_size;
    if (fread(buf + HEADER_SIZE, frame_size - HEADER_SIZE, 1, stream->fp) != 1)
        return 0;

    if (swap)
        swap16((uint32_t *)(buf + HEADER_SIZE), (aligned_size - HEADER_SIZE) / 4);

    stream->packet_size += aligned_size;

    // Shut up memcheck
    memset(buf + frame_size, 0, aligned_size - frame_size + DCADEC_BUFFER_PADDING);

    // Restore sync word
    buf[0] = (sync >> 24) & 0xff;
    buf[1] = (sync >> 16) & 0xff;
    buf[2] = (sync >>  8) & 0xff;
    buf[3] = (sync >>  0) & 0xff;

    if (sync_p)
        *sync_p = sync;
    return 1;
}

DCADEC_API int dcadec_stream_read(struct dcadec_stream *stream, uint8_t **data, size_t *size)
{
    uint32_t sync;
    int ret;

    // Loop until valid DTS core or standalone EXSS frame is read or EOF is
    // reached
    while (true) {
        ret = read_frame(stream, &sync);
        if (ret == 1)
            break;
        if (ret == 0)
            return ferror(stream->fp) ? -DCADEC_EIO : 0;
        if (ret == -1)
            return -DCADEC_ENOMEM;
    }

    // Check for EXSS that may follow core frame and try to concatenate both
    // frames into single packet
    if (sync == SYNC_WORD_CORE) {
        ret = read_frame(stream, NULL);
        if (ret == -1)
            return -DCADEC_ENOMEM;
    }

    *data = stream->buffer;
    *size = stream->packet_size;

    stream->packet_size = 0;
    return 1;
}

DCADEC_API int dcadec_stream_progress(struct dcadec_stream *stream)
{
    if (stream->stream_size > 0) {
        off_t pos = ftello(stream->fp);
        if (pos < stream->stream_start)
            return 0;
        if (pos >= stream->stream_start + stream->stream_size)
            return 100;
        return (int)((pos - stream->stream_start) * 100 / stream->stream_size);
    }
    return -1;
}

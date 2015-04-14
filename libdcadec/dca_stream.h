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

#ifndef DCA_STREAM_H
#define DCA_STREAM_H

#include "dca_context.h"

struct dcadec_stream;

struct dcadec_stream_info {
    uint64_t    stream_size;
    uint32_t    sample_rate;
    uint32_t    nframes;
    uint32_t    nframesamples;
    uint64_t    npcmsamples;
    uint32_t    ch_mask;
    uint32_t    ndelaysamples;
};

DCADEC_API struct dcadec_stream *dcadec_stream_open(const char *name);
DCADEC_API void dcadec_stream_close(struct dcadec_stream *stream);
DCADEC_API int dcadec_stream_read(struct dcadec_stream *stream, uint8_t **data, size_t *size);
DCADEC_API int dcadec_stream_progress(struct dcadec_stream *stream);
DCADEC_API struct dcadec_stream_info *dcadec_stream_get_info(struct dcadec_stream *stream);
DCADEC_API void dcadec_stream_free_info(struct dcadec_stream_info *info);

#endif

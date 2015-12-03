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

#ifndef EXSS_PARSER_H
#define EXSS_PARSER_H

#include "bitstream.h"

#define exss_err(...)   dca_log(ERROR, exss, __VA_ARGS__)

#define exss_err_once(...)  dca_log_once(ERROR, exss, err_shown, __VA_ARGS__)

struct exss_parser;

struct exss_asset {
    struct exss_parser  *parser;

    size_t  asset_offset;
    size_t  asset_size;
    int     asset_index;

    int     pcm_bit_res;
    int     max_sample_rate;
    int     nchannels_total;
    int     spkr_mask;
    int     representation_type;

    int     coding_mode;
    int     extension_mask;

    size_t  core_offset;
    size_t  core_size;

    size_t  xbr_offset;
    size_t  xbr_size;

    size_t  xxch_offset;
    size_t  xxch_size;

    size_t  x96_offset;
    size_t  x96_size;

    size_t  lbr_offset;
    size_t  lbr_size;

    size_t  xll_offset;
    size_t  xll_size;
    size_t  xll_sync_offset;
    int     xll_delay_nframes;
    int     hd_stream_id;
    bool    xll_sync_present;
    bool    one_to_one_map_ch_to_spkr;
    bool    embedded_stereo;
    bool    embedded_6ch;
    bool    spkr_mask_enabled;
};

struct exss_parser {
    struct bitstream   bits;

    dcadec_log_cb   log_cb;
    void            *log_cbarg;

    int     exss_index;
    int     exss_size_nbits;
    size_t  exss_size;

    int     npresents;
    int     nassets;

    bool    err_shown;
    bool    static_fields_present;
    bool    mix_metadata_enabled;
    int     nmixoutconfigs;
    int     nmixoutchs[4];
    struct exss_asset   *assets;
};

extern const uint32_t exss_sample_rates[16];

int exss_parse(struct exss_parser *exss, uint8_t *data, size_t size);
struct dcadec_exss_info *exss_get_info(struct exss_parser *exss) __attribute__((cold));

#endif

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

    int     asset_offset;
    int     asset_size;
    int     asset_index;

    int     pcm_bit_res;
    int     max_sample_rate;
    int     nchannels_total;
    bool    one_to_one_map_ch_to_spkr;
    bool    embedded_stereo;
    bool    embedded_6ch;
    bool    spkr_mask_enabled;
    int     spkr_mask;
    int     representation_type;

    int     coding_mode;
    int     extension_mask;

    int     core_offset;
    int     core_size;

    int     xbr_offset;
    int     xbr_size;

    int     xxch_offset;
    int     xxch_size;

    int     x96_offset;
    int     x96_size;

    int     lbr_offset;
    int     lbr_size;

    int     xll_offset;
    int     xll_size;
    bool    xll_sync_present;
    int     xll_delay_nframes;
    int     xll_sync_offset;

    int     hd_stream_id;
};

struct exss_parser {
    struct bitstream   bits;

    dcadec_log_cb   log_cb;
    void            *log_cbarg;
    bool    err_shown;

    int     exss_index;
    int     exss_size_nbits;
    int     exss_size;

    bool    static_fields_present;
    int     npresents;
    int     nassets;

    bool    mix_metadata_enabled;
    int     nmixoutconfigs;
    int     nmixoutchs[4];

    struct exss_asset   *assets;
};

extern const uint32_t exss_sample_rates[16];

int exss_parse(struct exss_parser *exss, uint8_t *data, int size);
struct dcadec_exss_info *exss_get_info(struct exss_parser *exss) __attribute__((cold));

#endif

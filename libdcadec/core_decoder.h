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

#ifndef CORE_DECODER_H
#define CORE_DECODER_H

#include "bitstream.h"

#define MAX_CHANNELS            7
#define MAX_SUBBANDS            32
#define MAX_LFE_SAMPLES         16
#define MAX_SUBFRAMES           16
#define NUM_SUBBAND_SAMPLES     8
#define NUM_PCMBLOCK_SAMPLES    32
#define NUM_ADPCM_COEFFS        4
#define NUM_CODE_BOOKS          10

#define MAX_SUBBANDS_X96        64

#define MAX_CHANNELS_CORE       6
#define MAX_CHANNELS_DMIX       4
#define MAX_CHANNELS_XXCH       2

#define core_err(...)   dca_log(ERROR, core, __VA_ARGS__)
#define core_warn(...)  dca_log(WARNING, core, __VA_ARGS__)

#define core_err_once(...)     dca_log_once(ERROR, core, err_shown, __VA_ARGS__)
#define core_warn_once(...)    dca_log_once(WARNING, core, warn_shown, __VA_ARGS__)

struct exss_asset;

struct core_decoder {
    struct bitstream    bits;

    dcadec_log_cb   log_cb;
    void            *log_cbarg;
    bool    err_shown;
    bool    warn_shown;

    bool    crc_present;
    int     npcmblocks;
    size_t  frame_size;
    int     audio_mode;
    int     sample_rate;
    int     bit_rate;
    bool    drc_present;
    bool    ts_present;
    bool    aux_present;
    int     ext_audio_type;
    bool    ext_audio_present;
    bool    sync_ssf;
    int     lfe_present;
    bool    predictor_history;
    bool    filter_perfect;
    int     source_pcm_res;
    bool    es_format;
    bool    sumdiff_front;
    bool    sumdiff_surround;

    int     nsubframes;
    int     nsubsubframes[MAX_SUBFRAMES];

    int     nchannels;
    int     ch_mask;

    int8_t      nsubbands[MAX_CHANNELS];
    int8_t      subband_vq_start[MAX_CHANNELS];
    int8_t      joint_intensity_index[MAX_CHANNELS];
    int8_t      transition_mode_sel[MAX_CHANNELS];
    int8_t      scale_factor_sel[MAX_CHANNELS];
    int8_t      bit_allocation_sel[MAX_CHANNELS];
    int8_t      quant_index_sel[MAX_CHANNELS][NUM_CODE_BOOKS];
    int32_t     scale_factor_adj[MAX_CHANNELS][NUM_CODE_BOOKS];

    int8_t      prediction_mode[MAX_CHANNELS][MAX_SUBBANDS_X96];
    int16_t     prediction_vq_index[MAX_CHANNELS][MAX_SUBBANDS_X96];
    int8_t      bit_allocation[MAX_CHANNELS][MAX_SUBBANDS_X96];
    int8_t      transition_mode[MAX_SUBFRAMES][MAX_CHANNELS][MAX_SUBBANDS];
    int32_t     scale_factors[MAX_CHANNELS][MAX_SUBBANDS][2];
    int8_t      joint_scale_sel[MAX_CHANNELS];
    int32_t     joint_scale_factors[MAX_CHANNELS][MAX_SUBBANDS_X96];

    int                 *subband_buffer;
    int                 *subband_samples[MAX_CHANNELS][MAX_SUBBANDS];
    struct interpolator *subband_dsp[MAX_CHANNELS];
    struct idct_context *subband_dsp_idct;

    int     *lfe_samples;

    bool    prim_dmix_embedded;
    int     prim_dmix_type;
    int     prim_dmix_coeff[MAX_CHANNELS_DMIX * MAX_CHANNELS_CORE];

    int     ext_audio_mask;

    size_t  xch_pos;

    bool    xxch_crc_present;
    int     xxch_mask_nbits;
    int     xxch_core_mask;
    int     xxch_spkr_mask;
    bool    xxch_dmix_embedded;
    int     xxch_dmix_scale_inv;
    int     xxch_dmix_mask[MAX_CHANNELS_XXCH];
    int     xxch_dmix_coeff[MAX_CHANNELS_XXCH * MAX_CHANNELS_CORE];
    size_t  xxch_pos;

    int     x96_rev_no;
    bool    x96_crc_present;
    int     x96_nchannels;
    bool    x96_high_res;
    int     x96_subband_start;
    int     x96_rand;
    size_t  x96_pos;

    int     *x96_subband_buffer;
    int     *x96_subband_samples[MAX_CHANNELS][MAX_SUBBANDS_X96];

    int     *output_buffer;
    int     *output_samples[SPEAKER_COUNT];
    int     output_history_lfe;

    int     npcmsamples;
    int     output_rate;

    int     filter_flags;
};

int core_parse(struct core_decoder *core, uint8_t *data, size_t size,
               int flags, struct exss_asset *asset);
int core_parse_exss(struct core_decoder *core, uint8_t *data,
                    int flags, struct exss_asset *asset);
int core_filter(struct core_decoder *core, int flags);
void core_clear(struct core_decoder *core) __attribute__((cold));
struct dcadec_core_info *core_get_info(struct core_decoder *core) __attribute__((cold));
struct dcadec_exss_info *core_get_info_exss(struct core_decoder *core) __attribute__((cold));

#endif

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

#ifndef LBR_DECODER_H
#define LBR_DECODER_H

#define LBR_CHANNELS    8
#define LBR_SUBBANDS    32
#define LBR_TONES       512

#define LBR_TIME_SAMPLES    128
#define LBR_TIME_HISTORY    8

#define lbr_err(...)        dca_log(lbr, ERROR, __VA_ARGS__)
#define lbr_warn(...)       dca_log(lbr, WARNING, __VA_ARGS__)
#define lbr_info(...)       dca_log(lbr, INFO, __VA_ARGS__)
#define lbr_debug(...)      dca_log(lbr, DEBUG, __VA_ARGS__)

struct exss_asset;

struct bitstream2 {
    uint8_t *data;
    int index;
    uint32_t accum;
    int avail;
    int count;
};

struct lbr_tone {
    uint8_t x_freq;
    uint8_t f_delt;
    uint8_t ph_rot;
    uint8_t pad;
    uint8_t amp[LBR_CHANNELS];
    uint8_t phs[LBR_CHANNELS];
};

struct lbr_decoder {
    struct dcadec_context   *ctx; ///< Parent context
    struct bitstream2       bits; ///< Bitstream reader

    int ctx_flags;

    int sample_rate;
    int ch_mask;
    int flags;
    int bit_rate_orig;
    int bit_rate_scaled;

    int nchannels;
    int nchannels_total;
    int undo_dmix;
    int freq_range;
    int band_limit;
    int limited_rate;
    int limited_range;
    int res_profile;
    int nsubbands;
    int g3_avg_only_start_sb;
    int min_mono_subband;
    int max_mono_subband;

    int framenum;
    int lbr_rand;

    uint8_t can_replace_sf[LBR_CHANNELS * LBR_SUBBANDS * 4 / 8];
    uint8_t can_replace_ch[LBR_CHANNELS * LBR_SUBBANDS * 4 / 8];

    uint8_t quant_levels[LBR_CHANNELS / 2][LBR_SUBBANDS];
    uint8_t sb_indices[LBR_SUBBANDS];

    uint8_t sec_ch_sbms[LBR_CHANNELS / 2][LBR_SUBBANDS];
    uint8_t sec_ch_lrms[LBR_CHANNELS / 2][LBR_SUBBANDS];
    uint32_t ch_pres[LBR_CHANNELS];

    uint8_t grid_1_scf[LBR_CHANNELS][12][8];
    uint8_t grid_2_scf[LBR_CHANNELS][6][64];

    int8_t grid_3_avg[LBR_CHANNELS][28];
    int8_t grid_3_scf[LBR_CHANNELS][28][8];
    uint32_t grid_3_pres[LBR_CHANNELS];

    uint8_t high_res_scf[LBR_CHANNELS][LBR_SUBBANDS][8];

    uint8_t part_stereo[LBR_CHANNELS][LBR_SUBBANDS / 4][5];
    uint8_t spatial_info[LBR_CHANNELS - 2][LBR_SUBBANDS / 4][5];

    uint8_t part_stereo_pres;
    uint8_t spatial_info_pres;

    float lpc_coeff[LBR_CHANNELS][3][2][2][8];

    float sb_scf[LBR_SUBBANDS];

    float time_samples[LBR_CHANNELS][LBR_SUBBANDS][LBR_TIME_HISTORY + LBR_TIME_SAMPLES];
    float imdct_history[LBR_CHANNELS][LBR_SUBBANDS * 4];

    uint8_t tonal_scf[6];
    uint16_t tonal_bounds[5][32][2];
    struct lbr_tone tones[LBR_TONES];
    int ntones;

    float lfe_data[64];
    float lfe_history[5][2];

    struct idct_context *imdct;
    float sin_tab[256];
    float lpc_tab[16];

    int channel_buffer[LBR_CHANNELS][LBR_TIME_SAMPLES * LBR_SUBBANDS];

    int *output_samples[SPEAKER_COUNT];
    int output_mask;
};

int lbr_parse(struct lbr_decoder *lbr, uint8_t *data, size_t size, struct exss_asset *asset);
int lbr_filter(struct lbr_decoder *lbr);
void lbr_clear(struct lbr_decoder *lbr) __attribute__((cold));

#endif
